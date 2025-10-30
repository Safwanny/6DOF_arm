# 6DOF_arm
Manipulator Arm Using MoveIt for Path Planning

This project showcases the development of a 6-DOF robotic manipulator built from scratch and integrated with the MoveIt 2 stack for motion planning. It also includes a C++ API interface for sending joint and pose commands, demonstrating a complete learning path in robot development with MoveIt.

## Overview
This projects demostrates:
- Defining a manipulator arm using URDF/Xacro
- Integrating the MoveIt 2 package for motion planning
- Using the C++ API to send commands to the arm for joint and pose goals
- Implementing a custom ROS 2 interface (PoseCommand) for communication between nodes

## Repository Structure

| Package               | Description                                                                |
| --------------------- | -------------------------------------------------------------------------- |
| **arm_bringup**       | Contains launch files for starting the nodes and RViz visualization        |
| **arm_commander**     | Contains test and API command files for controlling the robot              |
| **arm_description**   | Contains URDF/Xacro models and RViz configuration files                    |
| **arm_interfaces**    | Defines the custom ROS 2 interface `PoseCommand`                           |
| **arm_moveit_config** | Contains MoveIt 2 configuration files, including ROS 2 control integration |


## Dependencies
Ensure you are using Ubuntu 24.04 with a working ROS 2 (Humble or Jazzy) installation.
Install the required dependencies:
```bash
sudo apt install ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-rviz2 \
                 ros-${ROS_DISTRO}-moveit \
                 ros-${ROS_DISTRO}-robot-state-publisher \
```
Note: Follow the official MoveIt 2 installation guide to ensure proper installation.
There may be middleware compatibility considerations for CycloneDDS.

## Building the Package
Clone the repository into your ROS 2 workspace and build:

```bash
cd ~/ros2_ws/src
git clone (USING TH SSH KEY)
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## Visualization
Currently, visualization is available through RViz.

View the robot model:
```bash
ros2 launch arm_description display.launch.xml
```

Launch the manipulator with MoveIt configuration:
```bash
ros2 launch arm_bringup arm.launch.xml
```
After launching, add the MoveIt MotionPlanning plugin in RViz:
1. Click Add (bottom-left corner in RViz)
2. Select MotionPlanning (under moveit_ros_visualization)

## Controlling the arm
there are 2 ways to control the robot:

### 1. With the test_moveit.cpp file
Step 1: Launch the main bringup file
```bash
ros2 launch arm_bringup arm.launch.xml
```
Step 2: In another terminal, run the MoveIt test node:
```bash
ros2 run arm_commander test_moveit
```
You can edit test_moveit.cpp to experiment with different types of motion:
- Named Goals
- Joint Goals
- Pose Goals
- Cartesian Paths
Each of these methods is demonstrated within the file.

### 2. Controlling the arm with C++ API
You can also control the manipulator by publishing messages to a topic.

Step 1: Launch the bringup:
```bash
ros2 launch arm_bringup arm.launch.xml
```
Step 2: Run the command interface:
```bash
ros2 run arm_commander commander
```
Step 3: Send pose commands:
```bash
ros2 topic pub -1 /pose_command arm_interafaces/msg/PoseCommand "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```
Step 4: Control the gripper:
```bash
ros2 topic pub -1 /open_gripper example_interfaces/msg/bool "{data: false}"
```
true → Open gripper
false → Close gripper

make sure that all terminal are run seperately and rviz window is open side by side.


