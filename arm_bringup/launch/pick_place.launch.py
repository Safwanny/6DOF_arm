from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("robot_arm", package_name="arm_moveit_config").to_dict()
    return LaunchDescription([
        Node(package="arm_commander", executable="pick_place", output="screen",
             parameters=[moveit_config]),
    ])
