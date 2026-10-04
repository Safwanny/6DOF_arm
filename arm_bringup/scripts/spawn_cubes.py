#!/usr/bin/env python3
"""Replace the cubes in the Gazebo world: spawn_cubes.py x y yaw [x y yaw ...]

Each cube publishes its own pose on /model/cube_N/pose, which pick_place uses to re-measure."""
import math
import subprocess
import sys

WORLD = "/world/table_world"
MAX_CUBES = 6
SIZE, MASS, TABLE_TOP = 0.04, 0.1, 0.15  # match OBJECT_SIZE / TABLE_TOP in pick_place.cpp
I = MASS * SIZE**2 / 6

CUBE_SDF = f"""<sdf version='1.9'><model name='{{name}}'><link name='link'>
<inertial><mass>{MASS}</mass><inertia><ixx>{I}</ixx><iyy>{I}</iyy><izz>{I}</izz>
<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
<collision name='collision'><geometry><box><size>{SIZE} {SIZE} {SIZE}</size></box></geometry>
<surface><friction><ode><mu>1.5</mu><mu2>1.5</mu2></ode></friction></surface></collision>
<visual name='visual'><geometry><box><size>{SIZE} {SIZE} {SIZE}</size></box></geometry>
<material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material></visual>
</link>
<plugin filename='gz-sim-pose-publisher-system' name='gz::sim::systems::PosePublisher'>
<publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose>
<publish_visual_pose>false</publish_visual_pose><publish_collision_pose>false</publish_collision_pose>
<use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>20</update_frequency>
</plugin></model></sdf>"""


def gz(service, reqtype, req):
    return subprocess.run(["gz", "service", "-s", f"{WORLD}/{service}", "--reqtype", reqtype,
                           "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req],
                          capture_output=True, text=True).stdout


def main():
    vals = [float(v) for v in sys.argv[1:]]
    # Old cubes from a previous run; removing a missing one just fails quietly
    for i in range(MAX_CUBES):
        gz("remove", "gz.msgs.Entity", f'name: "cube_{i}", type: MODEL')
    for i in range(len(vals) // 3):
        x, y, yaw = vals[3 * i:3 * i + 3]
        sdf = CUBE_SDF.format(name=f"cube_{i}").replace("\n", " ").replace('"', '\\"')
        out = gz("create", "gz.msgs.EntityFactory",
                 f'sdf: "{sdf}", pose: {{position: {{x: {x}, y: {y}, z: {TABLE_TOP + SIZE / 2 + 0.002}}},'
                 f' orientation: {{z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}}}')
        if "true" not in out:
            sys.exit(f"failed to spawn cube_{i}: {out}")
        print(f"spawned cube_{i} at {x:.3f} {y:.3f} yaw {yaw:.2f}")


if __name__ == "__main__":
    main()
