import math
import random

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from moveit_configs_utils import MoveItConfigsBuilder

# Free half of the table (the tray is at y >= 0.055), see pick_place.cpp
X_RANGE = (0.48, 0.80)
Y_RANGE = (-0.38, -0.02)
# Open finger reaches 8 cm along the grip axis plus 2.8 cm half-diagonal of a turned cube:
# at this spacing no grasp of one cube can hit another
MIN_GAP = 0.13
MAX_CUBES = 6


def sample_cubes(count, rng):
    cubes = []
    for _ in range(1000):
        if len(cubes) == count:
            return cubes
        x, y = rng.uniform(*X_RANGE), rng.uniform(*Y_RANGE)
        if all(math.hypot(x - cx, y - cy) >= MIN_GAP for cx, cy, _ in cubes):
            cubes.append((x, y, rng.uniform(0, math.pi / 2)))
    if len(cubes) == count:
        return cubes
    raise RuntimeError(f"could not fit {count} cubes {MIN_GAP} m apart, try another seed")


def setup(context):
    count = int(LaunchConfiguration("count").perform(context))
    if not 1 <= count <= MAX_CUBES:
        raise RuntimeError(f"count must be 1-{MAX_CUBES}")
    seed_arg = LaunchConfiguration("seed").perform(context)
    seed = int(seed_arg) if seed_arg else random.randrange(10000)
    poses = [v for cube in sample_cubes(count, random.Random(seed)) for v in cube]
    sim = LaunchConfiguration("sim").perform(context) == "true"
    use_sim_time = sim or LaunchConfiguration("use_sim_time").perform(context) == "true"

    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="arm_moveit_config").to_dict()
    # C++ runs the MTC tasks, packer.py decides which cube goes where. In Gazebo it finds the cubes
    # with the camera only, so the spawn layout goes to Gazebo and never to the packer.
    packer = {"use_sim_time": use_sim_time, "sim": sim}
    if not sim:
        packer["cube_poses"] = poses
    pick_place = [Node(package="arm_commander", executable="pick_place", output="screen",
                       parameters=[moveit_config, {"use_sim_time": use_sim_time}]),
                  Node(package="arm_commander", executable="packer.py", output="screen", parameters=[packer])]
    actions = [LogInfo(msg=f"cube seed {seed} (rerun with seed:={seed})")]
    if not sim:
        return actions + pick_place
    # Gazebo: put the cubes in the world first, then plan
    spawn = ExecuteProcess(cmd=[[FindPackagePrefix("arm_bringup"), "/lib/arm_bringup/spawn_cubes.py"]]
                           + [f"{v:.4f}" for v in poses], output="screen")
    failed = LogInfo(msg="spawning cubes failed, is arm_gz.launch.xml running?")
    after_spawn = OnProcessExit(target_action=spawn,
                                on_exit=lambda event, _: pick_place if event.returncode == 0 else [failed])
    return actions + [spawn, RegisterEventHandler(after_spawn)]


def generate_launch_description():
    return LaunchDescription([
        # true when running against arm_gz.launch.xml: also spawns the cubes in Gazebo
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # number of cubes (1-6), one per tray slot
        DeclareLaunchArgument("count", default_value=str(MAX_CUBES)),
        # empty picks a random layout; the seed is logged so a layout can be repeated
        DeclareLaunchArgument("seed", default_value=""),
        OpaqueFunction(function=setup),
    ])
