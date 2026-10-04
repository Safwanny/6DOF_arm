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
- Gazebo (Harmonic) simulation with physics, a real friction grasp and a depth camera

## Repository Structure

| Package               | Description                                                                |
| --------------------- | -------------------------------------------------------------------------- |
| **arm_bringup**       | Launch files (mock and Gazebo), Gazebo world, RViz configs, camera viewer  |
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
                 ros-jazzy-moveit-task-constructor-visualization \
                 ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
                 ros-jazzy-cv-bridge python3-opencv
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
The node adds a table, a 4×4×10 cm box and a shallow 2×3 tray to the planning scene, then MTC plans and runs the whole task: open gripper → move above the box → lower → close → lift → move above a tray slot → lower straight into it → half open → retreat → home. It logs `Pick and place done` on success; on failure it prints which stage found 0 solutions. Stop it with Ctrl+C. Running it again puts the box back at the start.

Pick the tray slot (0-5, default 0) with `slot:=N`. Slot 0 is the corner nearest the arm on the box side, slots count along y first: 0-2 are the near row (x = 0.555), 3-5 the far row (x = 0.645), at y = 0.11, 0.20, 0.29:
```bash
ros2 launch arm_bringup pick_place.launch.py slot:=4
```

To see the table and box in RViz, add the **MotionPlanning** display (see step 2). To step through each stage, also add **Add → moveit_task_constructor_visualization → Motion Planning Tasks**.

The box, table and tray positions are constants at the top of [pick_place.cpp](arm_commander/src/pick_place.cpp). If you move them out of reach, the `grasp pose IK` or `place pose IK` stage fails.

## Gazebo simulation
The same arm in Gazebo Harmonic, with gravity, contacts and a depth camera. Open 2 terminals and in each run `cd ~/ros2_ws && source install/setup.bash`.

**1. Start the simulation** (terminal 1)
```bash
ros2 launch arm_bringup arm_gz.launch.xml
```
After about 30 s, three windows are open:
- **Gazebo**: the arm bolted to the floor, a table with a red box and a grey tray, and a depth camera on a stand behind the table.
- **RViz**: the robot, the planning scene (table, box and tray) and the camera's point cloud.
- **Camera window**: colour image (left) and depth image (right, red = near, blue = far).

Closing any of these windows, or Ctrl+C in terminal 1, shuts the whole simulation down.

**2. Check the controllers** (terminal 2): all three should be `active`
```bash
ros2 control list_controllers
```

**3. Pick and place** (terminal 2)
```bash
ros2 launch arm_bringup pick_place.launch.py use_sim_time:=true
```
The fingers physically grip the box, carry it to the tray and stand it in the chosen slot. Check where it ended up (slot 0 should be near `0.555 0.11 0.205`, upright):
```bash
gz model -m box -p
```
To run it again, put the box back first:
```bash
gz service -s /world/table_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 --req 'name: "box", position: {x: 0.6, y: -0.2, z: 0.2}, orientation: {w: 1}'
```

**Other nodes on the simulation** need sim time, e.g.:
```bash
ros2 run arm_commander test_moveit --ros-args -p use_sim_time:=true
```

Camera topics: `/camera/image`, `/camera/depth_image`, `/camera/camera_info`, `/camera/points` (frame `camera_link`).

How it fits together: `robot_arm.urdf.xacro` takes `sim:=true` to swap mock hardware for `gz_ros2_control` and bolt the base to the Gazebo world; the world ([table.sdf](arm_bringup/worlds/table.sdf)) matches the table/box constants in [pick_place.cpp](arm_commander/src/pick_place.cpp). If you move the camera in the world file, update the `camera_link` transform in [arm_gz.launch.xml](arm_bringup/launch/arm_gz.launch.xml) to match.

## Named poses (from the SRDF)
| Group   | Names                                           |
| ------- | ----------------------------------------------- |
| arm     | `home`, `pose1`, `pose2`                        |
| gripper | `gripper_open`, `gripper_half_open`, `gripper_close`, `gripper_grasp` (closes onto the 4 cm box) |

## Known limitations
- In mock mode (`arm.launch.xml`) nothing is physical: pick and place "grasps" by attaching the box in the planning scene. Use the Gazebo launch for a real grasp.
- Pick and place uses the hard-coded box position; it does not use the camera yet.
- The tray has a 9 cm slot pitch because the half-open fingers reach 5 cm from the box centre; its 1.5 cm walls sit below the finger tips. If you change the tray in [table.sdf](arm_bringup/worlds/table.sdf), update the `TRAY_*` constants in pick_place.cpp to match.
- Grasps are limited to 90° steps around the box so the fingers land flat on its faces; a diagonal grasp squeezes the corners and flicks the box out in Gazebo.
- If a pose goal is unreachable or a Cartesian path is <100% feasible, the commander silently does nothing — watch the `move_group` log in terminal 1.
- Commands block the commander's callback while planning/executing; a new command sent mid-motion is queued, not preempting.
