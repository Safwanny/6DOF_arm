# 6DOF_arm
Manipulator Arm Using MoveIt for Path Planning

This project showcases the development of a 6-DOF robotic manipulator built from scratch and integrated with the MoveIt 2 stack for motion planning. It also includes a C++ API interface for sending joint and pose commands, demonstrating a complete learning path in robot development with MoveIt.

## Overview
This project demonstrates:
- Defining a manipulator arm using URDF/Xacro
- Integrating the MoveIt 2 package for motion planning
- Using the C++ API to send commands to the arm for joint and pose goals
- Implementing a custom ROS 2 interface (PoseCommand) for communication between nodes
- A pick-and-place task built with MoveIt Task Constructor (MTC)

## Repository Structure

| Package               | Description                                                                |
| --------------------- | -------------------------------------------------------------------------- |
| **arm_bringup**       | Contains launch files for starting the nodes and RViz visualization        |
| **arm_commander**     | Test, topic commander and pick-and-place nodes for controlling the robot   |
| **arm_description**   | Contains URDF/Xacro models and RViz configuration files                    |
| **arm_interfaces**    | Defines the custom ROS 2 interface `PoseCommand`                           |
| **arm_moveit_config** | Contains MoveIt 2 configuration files, including ROS 2 control integration |


## Dependencies
Tested on **Ubuntu 24.04 + ROS 2 Jazzy**. (The code includes `move_group_interface.hpp`, which is the Jazzy+ header, so Humble will not compile without changing it to `.h`.)

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-rviz2 ros-jazzy-moveit \
                 ros-jazzy-robot-state-publisher ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers ros-jazzy-example-interfaces \
                 ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-moveit-task-constructor-core \
                 ros-jazzy-moveit-task-constructor-capabilities \
                 ros-jazzy-moveit-task-constructor-visualization
```

## Building
```bash
cd ~/ros2_ws/src
git clone <this repo>
cd ~/ros2_ws
colcon build
source install/setup.bash
```
If the build fails with `No rule to make target '/opt/ros/jazzy/lib/libfastcdr.so...'`, a ROS update changed a library under an old build cache. Clean and rebuild:
```bash
rm -rf build/arm_interfaces install/arm_interfaces build/arm_commander install/arm_commander && colcon build
```

## Step-by-step: see it working
Open 3 terminals. In every terminal run:
```bash
cd ~/ros2_ws && source install/setup.bash
```

**1. View the model only (optional)**
```bash
ros2 launch arm_description display.launch.xml
```
Use the joint slider GUI to move each joint.

**2. Launch the arm with MoveIt** (terminal 1)
```bash
ros2 launch arm_bringup arm.launch.xml
```
This starts robot_state_publisher, ros2_control (mock hardware), the `joint_state_broadcaster`, `arm_controller` and `gripper_controller`, `move_group`, and RViz.

Check the controllers are up (terminal 2):
```bash
ros2 control list_controllers
```
All three should be `active`.

To plan interactively in RViz: **Add → moveit_ros_visualization → MotionPlanning**, drag the interactive marker, then **Plan & Execute**.

**3. Run the scripted demo** (terminal 2)
```bash
ros2 run arm_commander test_moveit
```
The arm moves to a pose goal (x=0.7, z=0.4, gripper pointing down), then follows a Cartesian path (down 0.2 m, sideways 0.2 m, back). Named and joint goal examples are in [test_moveit.cpp](arm_commander/src/test_moveit.cpp); uncomment a section to try it.

**4. Control the arm via topics** (terminal 2, then send commands from terminal 3)
```bash
ros2 run arm_commander commander
```
Pose goal (`cartesian_path: true` moves in a straight line instead):
```bash
ros2 topic pub -1 /pose_command arm_interfaces/msg/PoseCommand "{x: 0.7, y: 0.0, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```
Joint goal (6 values, radians, in order `base_shoulder_joint`, `shoulder_arm_joint`, `arm_elbow_joint`, `elbow_forearm_joint`, `forearm_wrist_joint`, `wrist_hand_joint`):
```bash
ros2 topic pub -1 /joint_command example_interfaces/msg/Float64MultiArray "{data: [0.5, 0.3, 0.2, 0.0, 0.4, 0.0]}"
```
Gripper (`true` = open, `false` = close):
```bash
ros2 topic pub -1 /open_gripper example_interfaces/msg/Bool "{data: false}"
```
Verify the motion:
```bash
ros2 topic echo --once /joint_states
```

**5. Pick and place (MTC)** (terminal 2, with step 2 running)
```bash
ros2 launch arm_bringup pick_place.launch.py
```
The node adds a table and a 4×4×10 cm box to the planning scene, then MTC plans and runs the whole task: open gripper → move above the box → lower → close → lift → move → lower onto the table 40 cm to the side → open → retreat → home. It logs `Pick and place done` on success; on failure it prints which stage found 0 solutions. Stop it with Ctrl+C. Running it again puts the box back at the start.

To see the table and box in RViz, add the **MotionPlanning** display (see step 2). To step through each stage, also add **Add → moveit_task_constructor_visualization → Motion Planning Tasks**.

The box and table positions are constants at the top of [pick_place.cpp](arm_commander/src/pick_place.cpp). If you move them out of reach, the `grasp pose IK` or `place pose IK` stage fails.

## Named poses (from the SRDF)
| Group   | Names                                           |
| ------- | ----------------------------------------------- |
| arm     | `home`, `pose1`, `pose2`                        |
| gripper | `gripper_open`, `gripper_half_open`, `gripper_close` |

## Known limitations
- Hardware is `mock_components` only: nothing is simulated physically (no Gazebo, no gravity/contacts).
- Pick and place grasps by attaching the box to `tool_link` in the planning scene; the fingers never actually squeeze it.
- If a pose goal is unreachable or a Cartesian path is <100% feasible, the commander silently does nothing — watch the `move_group` log in terminal 1.
- Commands block the commander's callback while planning/executing; a new command sent mid-motion is queued, not preempting.
