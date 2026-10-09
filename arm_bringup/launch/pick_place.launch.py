import math
import os
import random

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from moveit_configs_utils import MoveItConfigsBuilder

SCENE = yaml.safe_load(open(os.path.join(get_package_share_directory("arm_bringup"), "config", "scene.yaml")))
# Open fingers reach 8 cm from the grasp centre along the grip axis (plus margin): boxes keep at least this
# far apart beyond their own half-diagonals, so no grasp of one box can hit another
FINGER_REACH = 0.085


def sample_boxes(count, rng):
    """Random boxes on the free side of the table: (colour, x, y, yaw, length, width, height).
    Width is the gripped side, never longer than the length."""
    area, size = SCENE["scatter"], SCENE["boxes"]
    t = SCENE["table"]
    colours = list(SCENE["colours"])
    rng.shuffle(colours)
    boxes = []
    for _ in range(5000):
        if len(boxes) == count:
            return boxes
        length = rng.uniform(*size["length"])
        width = rng.uniform(size["width"][0], min(size["width"][1], length))
        height = rng.uniform(*size["height"])
        r = math.hypot(length, width) / 2
        x, y = rng.uniform(*area["x"]), rng.uniform(*area["y"])
        on_table = (abs(x - t["x"]) + r < t["size_x"] / 2 - 0.01 and abs(y - t["y"]) + r < t["size_y"] / 2 - 0.01)
        reach = max(r, FINGER_REACH)
        if (on_table and math.hypot(x, y) <= area["max_radius"]
                and all(math.hypot(x - b[1], y - b[2]) >= reach + max(b[7], FINGER_REACH) for b in boxes)):
            boxes.append((colours[len(boxes)], x, y, rng.uniform(0, math.pi), length, width, height, r))
    raise RuntimeError(f"could not fit {count} boxes on the table, try another seed")


def setup(context):
    count = int(LaunchConfiguration("count").perform(context))
    if not 1 <= count <= len(SCENE["colours"]):
        raise RuntimeError(f"count must be 1-{len(SCENE['colours'])} (one colour per box)")
    seed_arg = LaunchConfiguration("seed").perform(context)
    seed = int(seed_arg) if seed_arg else random.randrange(10000)
    boxes = [b[:7] for b in sample_boxes(count, random.Random(seed))]
    sim = LaunchConfiguration("sim").perform(context) == "true"
    use_sim_time = sim or LaunchConfiguration("use_sim_time").perform(context) == "true"

    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="arm_moveit_config").to_dict()
    # C++ runs the MTC tasks, packer.py decides which box goes where. In Gazebo it finds and measures the
    # boxes with the camera only, so the spawn layout goes to Gazebo and never to the packer.
    packer = {"use_sim_time": use_sim_time, "sim": sim}
    if not sim:
        packer["box_names"] = [f"box_{b[0]}" for b in boxes]
        packer["boxes"] = [float(v) for b in boxes for v in b[1:]]  # x, y, yaw, length, width, height
    pick_place = [Node(package="arm_commander", executable="pick_place", output="screen",
                       parameters=[moveit_config, {"use_sim_time": use_sim_time}]),
                  Node(package="arm_commander", executable="packer.py", output="screen", parameters=[packer])]
    actions = [LogInfo(msg=f"box seed {seed} (rerun with seed:={seed})")]
    if not sim:
        return actions + pick_place
    # Gazebo: put the boxes in the world first, then plan
    spawn = ExecuteProcess(cmd=[[FindPackagePrefix("arm_bringup"), "/lib/arm_bringup/spawn_boxes.py"]]
                           + [b[0] if i == 0 else f"{b[i]:.4f}" for b in boxes for i in range(7)], output="screen")
    failed = LogInfo(msg="spawning boxes failed, is arm_gz.launch.xml running?")
    after_spawn = OnProcessExit(target_action=spawn,
                                on_exit=lambda event, _: pick_place if event.returncode == 0 else [failed])
    return actions + [spawn, RegisterEventHandler(after_spawn)]


def generate_launch_description():
    return LaunchDescription([
        # true when running against arm_gz.launch.xml: also spawns the boxes in Gazebo
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # number of boxes, 1 to the number of colours in scene.yaml (6)
        DeclareLaunchArgument("count", default_value=str(len(SCENE["colours"]))),
        # empty picks a random layout; the seed is logged so a layout can be repeated
        DeclareLaunchArgument("seed", default_value=""),
        OpaqueFunction(function=setup),
    ])
