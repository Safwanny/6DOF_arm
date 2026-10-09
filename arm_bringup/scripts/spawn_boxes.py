#!/usr/bin/env python3
"""Replace the boxes in the Gazebo world: spawn_boxes.py colour x y yaw length width height [colour ...]

Each box gets its colour from scene.yaml and is named box_<colour>. Nothing tells the robot where the
boxes are or how big they are: the packer measures them with the depth camera."""
import math
import os
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory

WORLD = "/world/table_world"
SCENE = yaml.safe_load(open(os.path.join(get_package_share_directory("arm_bringup"), "config", "scene.yaml")))
LEGACY = [f"cube_{i}" for i in range(6)]  # cubes from before boxes had sizes

BOX_SDF = """<sdf version='1.9'><model name='{name}'><link name='link'>
<inertial><mass>{mass}</mass><inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='collision'><geometry><box><size>{l} {w} {h}</size></box></geometry>
<surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface></collision>
<visual name='visual'><geometry><box><size>{l} {w} {h}</size></box></geometry>
<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>
</link></model></sdf>"""


def gz(service, reqtype, req):
    return subprocess.run(["gz", "service", "-s", f"{WORLD}/{service}", "--reqtype", reqtype,
                           "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                          capture_output=True, text=True).stdout


def main():
    args = sys.argv[1:]
    # Old boxes from a previous run; removing a missing one just fails quietly
    for name in [f"box_{c}" for c in SCENE["colours"]] + LEGACY:
        gz("remove", "gz.msgs.Entity", f'name: "{name}", type: MODEL')
    top = SCENE["table"]["top"]
    for i in range(0, len(args), 7):
        colour = args[i]
        x, y, yaw, l, w, h = (float(v) for v in args[i + 1:i + 7])
        b = SCENE["boxes"]
        mass = max(b["min_mass"], b["density"] * l * w * h)
        sdf = BOX_SDF.format(name=f"box_{colour}", mass=mass, l=l, w=w, h=h,
                             ixx=mass * (w * w + h * h) / 12, iyy=mass * (l * l + h * h) / 12,
                             izz=mass * (l * l + w * w) / 12,
                             rgb=" ".join(str(v) for v in SCENE["colours"][colour]))
        sdf = sdf.replace("\n", " ").replace('"', '\\"')
        out = gz("create", "gz.msgs.EntityFactory",
                 f'sdf: "{sdf}", pose: {{position: {{x: {x}, y: {y}, z: {top + h / 2 + 0.002}}},'
                 f' orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}}}')
        if "true" not in out:
            sys.exit(f"failed to spawn box_{colour}: {out}")
        print(f"spawned box_{colour} {l * 100:.1f} x {w * 100:.1f} x {h * 100:.1f} cm "
              f"at {x:.3f} {y:.3f} yaw {yaw:.2f}")


if __name__ == "__main__":
    main()
