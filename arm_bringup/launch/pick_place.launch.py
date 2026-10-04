from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="arm_moveit_config").to_dict()
    return LaunchDescription([
        # true when running against arm_gz.launch.xml
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        # tray slot 0-5 to place the box in
        DeclareLaunchArgument("slot", default_value="0"),
        Node(package="arm_commander", executable="pick_place", output="screen",
             parameters=[moveit_config, {"use_sim_time": LaunchConfiguration("use_sim_time"),
                                         "slot": LaunchConfiguration("slot")}]),
    ])
