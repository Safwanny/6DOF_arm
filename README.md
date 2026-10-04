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
The launch file scatters 6 red 4 cm cubes at random positions and angles on the left half of the table, at least 13 cm apart so the open gripper never hits a neighbour. The node adds the table, the cubes and a shallow 2×3 tray to the planning scene. Then it works in a loop until every cube is in the tray:

1. **Measure** where every cube is now. In Gazebo the real poses come from the simulator; in mock mode the planning scene is the truth.
2. **Choose** the nearest upright cube on the table that isn't in the tray, and the first free slot.
3. **Run** one MTC task: open gripper → move above the cube → lower → close → lift → move above the slot → lower straight into it → half open → retreat → **ready**. Ready is a waiting pose 35 cm above the table, tool down.

Re-measuring after every cube catches ones that slipped, got knocked or missed their slot, and retries them. A cube is given up after 3 tries, or if it isn't standing upright on the table (fallen off, or leaning on something). When nothing is left to pick, the node logs `Packed N/6 cubes` and names any cube left out. Then it moves the arm **home** and logs `Job done, arm is home`. If a cube can't be planned, the log names the failing stage and the reason. Stop it with Ctrl+C. Each run starts from a fresh random layout.

Options:
```bash
ros2 launch arm_bringup pick_place.launch.py count:=3 seed:=42
```
- `count` (1-6, default 6): how many cubes.
- `seed`: repeat a layout. Every run logs its seed (`cube seed 42 (rerun with seed:=42)`).

Slots fill in order 0 → 5. Slot 0 is the corner nearest the arm, on the cube side. Slots count along y first: 0-2 are the near row (x = 0.555) and 3-5 the far row (x = 0.645), at y = 0.11, 0.20 and 0.29.

To see the table, cubes and tray in RViz, add the **MotionPlanning** display (see step 2). To step through each stage, also add **Add → moveit_task_constructor_visualization → Motion Planning Tasks**.

The table and tray are constants at the top of [pick_place.cpp](arm_commander/src/pick_place.cpp). The area the cubes are scattered over and their spacing are set at the top of [pick_place.launch.py](arm_bringup/launch/pick_place.launch.py).

## Gazebo simulation
The same arm in Gazebo Harmonic, with gravity, contacts and a depth camera. Open 2 terminals and in each run `cd ~/ros2_ws && source install/setup.bash`.

**1. Start the simulation** (terminal 1)
```bash
ros2 launch arm_bringup arm_gz.launch.xml
```
After about 30 s, three windows are open:
- **Gazebo**: the arm bolted to the floor, a table and a grey tray (the cubes are added by pick and place), and a depth camera on a stand behind the table.
- **RViz**: the robot, the planning scene (table, tray and cubes) and the camera's point cloud.
- **Camera window**: colour image (left) and depth image (right, red = near, blue = far).

Closing any of these windows, or Ctrl+C in terminal 1, shuts the whole simulation down.

**2. Check the controllers** (terminal 2): all three should be `active`
```bash
ros2 control list_controllers
```

**3. Pick and place** (terminal 2)
```bash
ros2 launch arm_bringup pick_place.launch.py sim:=true
```
`sim:=true` spawns the random cubes in Gazebo, replacing any left from an earlier run, and puts the node on sim time. The fingers grip each cube with a set force, carry it to the tray and stand it in the next free slot. `count:=` and `seed:=` work as in mock mode. Check where a cube ended up (slot 0 is near `0.555 0.11 0.175`, upright):
```bash
gz model -m cube_0 -p
```
To run it again, just relaunch it: the old cubes are removed and a new layout is spawned.

**Other nodes on the simulation** need sim time, e.g.:
```bash
ros2 run arm_commander test_moveit --ros-args -p use_sim_time:=true
```

Camera topics: `/camera/image`, `/camera/depth_image`, `/camera/camera_info`, `/camera/points` (frame `camera_link`).

How it fits together: `robot_arm.urdf.xacro` takes `sim:=true` to swap mock hardware for `gz_ros2_control` and bolt the base to the Gazebo world; the world ([table.sdf](arm_bringup/worlds/table.sdf)) matches the table/tray constants in [pick_place.cpp](arm_commander/src/pick_place.cpp). If you move the camera in the world file, update the `camera_link` transform in [arm_gz.launch.xml](arm_bringup/launch/arm_gz.launch.xml) to match.

## Named poses (from the SRDF)
| Group   | Names                                           |
| ------- | ----------------------------------------------- |
| arm     | `home`, `ready` (waiting pose above the table), `pose1`, `pose2` |
| gripper | `gripper_open`, `gripper_half_open`, `gripper_close`, `gripper_grasp` (aims 1 cm inside a 4 cm cube, so the fingers squeeze it) |

## Known limitations
- In mock mode (`arm.launch.xml`) nothing is physical: pick and place "grasps" by attaching the cube in the planning scene. Use the Gazebo launch for a real grasp.
- Pick and place measures cubes from Gazebo's own model poses (`/world/table_world/pose/info`, bridged in arm_gz.launch.xml), not from the camera yet.
- The tray has a 9 cm slot pitch because the half-open fingers reach 5 cm from the cube centre. Its 1 cm walls sit below the finger tips, which stop 1.5 cm above the cube bottom. If you change the tray in [table.sdf](arm_bringup/worlds/table.sdf), update the `TRAY_*` constants in pick_place.cpp to match.
- Grasps are limited to 90° steps around the cube so the fingers land flat on its faces; a diagonal grasp squeezes the corners and flicks the cube out in Gazebo.
- The two fingers are separate joints driven with the same value (the right one has a mirrored axis), so they close together. Two Gazebo workarounds keep them in step. They weigh 0.2 kg each, because very light fingers left the physics solver driving one finger before the other. Their joint ranges also start 1 cm below the open position, because a finger resting on its limit could stall. Without these, one finger pushed the cube into the other.
- In Gazebo the fingers are force-controlled: the gripper controller sends effort (PD on position, gains in [ros2_controllers_gz.yaml](arm_bringup/config/ros2_controllers_gz.yaml)), and each finger is capped at `FINGER_FORCE` (15 N) in [gripper.xacro](arm_description/urdf/gripper.xacro). Finger friction is 2.0. A closed finger stops on the cube short of its target, so MoveIt allows the fingers' next move to start up to 2 cm off ([moveit_controllers.yaml](arm_moveit_config/config/moveit_controllers.yaml)). Mock mode keeps position control.
- A cube that tips onto its side inside the tray still counts as packed.
- If a pose goal is unreachable or a Cartesian path is <100% feasible, the commander silently does nothing — watch the `move_group` log in terminal 1.
- Commands block the commander's callback while planning/executing; a new command sent mid-motion is queued, not preempting.
