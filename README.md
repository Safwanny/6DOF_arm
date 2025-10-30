# 6DOF_arm
Manipulator arm using Moveit for path planning

contructed a 6-DOF arm roboot from scratch and integrated Moveit stack to enable path planning, also integrating C++ API to give commands to the manipulator for joints and pose goals.

Demonstrated a learning path in developing robots with moveit stack.

## Overview
This projects demostrates:
- defining a mmanipulator arm using URDF/Xacro.
- integrate Moveit package for path planning.
- C++ API to sedn commands to to arm for joints and pose goals.
- custom interface for sending and recieving messgaes for PoseCommand

## Repository Structure
- arm_bringup: contains the launch files for respective nodes and Rviz.
- arm_commander: contains the test file for path planning and C++ file for API commands.
- arm_description: contains the URDF/Xacro and Rviz config files.
- arm_interfaces: contains the information for the custom interface PoseCommand 
- arm_moveit_config: contains multiple files required for Moveit path planning. here also files for integrating ros2_control.

## Dependencies
insure a Ubuntu 24.04 and a working ROS2 installation (Humble or Jazzy) before proceeding.

```bash
sudo apt install ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-rviz2 \
                 ros-${ROS_DISTRO}-moveit \
                 ros-${ROS_DISTRO}-robot-state-publisher \
```
please follow the instructions on the Moveit2 main websote to download the pakcage properly. there are some middlware compatibility for CycloneDDS

## Building the Package
Clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone (USING TH SSH KEY)
cd ~/ros2_ws
colcon build
source install/setup.bash
```
## Visualization
the visualization is currently only in Rviz. 

for visualising the robot build and respective joints run command:
```bash
ros2 launch arm_description display.launch.xml
```

for launching the manipulator with moveit configurations and controller packages:
```bash
ros2 launch arm_bringup arm.launch.xml
```
after launchin you will need to add the moveit visualization by:
- Add (bottom left in the Rviz window)
- MotionPlanning (this you can find under the moveit_ros_visualization)

## Controlling the arm
there are 2 ways to control the robot:
### 1. With the test_moveit.cpp file
in first terminal run:
```bash
ros2 launch arm_bringup arm.launch.xml
```
makesure the rvize window and terminal window are both open side by side
then run the test_moveit.cpp file
```bash
ros2 run arm_commander test_moveit
```
you can edit the test_moveit.cpp for different movements there are different ways to achieve paths and goals such as NamedGoals, JointGoals, PoseGoals and CartesianPaths. all of which are in the file.

### 2. Controlling the arm with C++ API
here you can send commands via the termnial for robot motion.
```bash
ros2 launch arm_bringup arm.launch.xml
```
run the commander package
```bash
ros2 run arm_commander commander
```
send commands to topic 
```bash
ros2 topic pub -1 /pose_command arm_interafaces/msg/PoseCommand "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```
to open and close gripper
```bash
ros2 topic pub -1 /open_gripper example_interfaces/msg/bool "{data: false}"
```
false - close 
true - open

make sure that all terminal are run seperately and rviz window is open side by side.


