# 6DOF_arm
Manipulator Arm Using MoveIt for Path Planning

This project showcases the development of a 6-DOF robotic manipulator built from scratch and integrated with the MoveIt 2 stack for motion planning. It also includes a C++ API interface for sending joint and pose commands, demonstrating a complete learning path in robot development with MoveIt.

## Overview
This project demonstrates:
- Defining a manipulator arm using URDF/Xacro
- Integrating the MoveIt 2 package for motion planning
- Using the C++ API to send commands to the arm for joint and pose goals
- Implementing a custom ROS 2 interface (PoseCommand) for communication between nodes
- A pick-and-place task built with MoveIt Task Constructor (MTC) that packs boxes of different sizes Tetris-style
- Gazebo (Harmonic) simulation with physics, a real friction grasp and a depth camera

## Repository Structure

| Package               | Description                                                                |
| --------------------- | -------------------------------------------------------------------------- |
| **arm_bringup**       | Launch files (mock and Gazebo), Gazebo world, scene layout (`scene.yaml`), RViz configs, camera viewer, box detector |
| **arm_commander**     | Test and topic commander nodes, the C++ pick-and-place task server and the Python packer |
| **arm_description**   | Contains URDF/Xacro models and RViz configuration files                    |
| **arm_interfaces**    | Custom ROS 2 interfaces: `PoseCommand`, `PickPlace`, `DetectedBox(es)`     |
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
The launch file scatters 6 boxes of random size on the free side of the table: length 3–15 cm, width 3–8 cm (the side the fingers grip; the open gap is 12 cm), height 2–8 cm. Each has its own colour (red, orange, yellow, magenta, purple, cyan) and is named after it (`box_red`, …). They stand at least the open-finger reach apart, so a grasp never hits a neighbour. Two nodes run:
- **`pick_place`** (C++): runs one MoveIt Task Constructor task per request. `/pick_place` puts a box at a spot in the tray; the request carries the gripped width and height, so the fingers close 1 cm inside the box, hold it with their tips 1 cm above its bottom, and open just 1.5 cm clear of it to let go. Moves with a box in the hand run at half speed so it doesn't swing out. `/go_home` sends the arm home.
- **`packer.py`** (Python): plays a small game of Tetris. It sets up the planning scene (table, tray, boxes) and loops until nothing more fits:

1. **Measure** every box. In Gazebo the depth camera finds and sizes them (see below); in mock mode the planning scene is the truth.
2. **Choose** the box with the largest footprint (ties: the longest) that is upright on the table, and the best spot for it in the tray:
   - The box may be turned 0° or 90°, and gripped across its width (or across its length, if that is 8 cm or less).
   - A spot keeps 5 mm from the walls and other boxes, rests at least 90 % of its footprint (and its centre) on a flat surface, and leaves room for the fingers beside the gripped faces (2 cm thick, 6 cm wide, 1.5 cm clear of the box).
   - Lowest spot wins (beside the others before on top of them), then the one nearest the tray corner closest to the arm.
3. **Run** one MTC task through `/pick_place`: open gripper → move above the box → lower → close → lift → move above the spot → lower straight onto it → open → retreat → **ready**. Ready is the waiting pose: tool down 35 cm above table height, the arm turned 90° to the tray side so it lies beside the table, out of the camera's view; the next measurement sees every box whole. After the last box the arm goes **home** (straight up).

Re-measuring after every box catches boxes that slipped or landed off their spot; the next ones are packed around them as they are. If a pick fails, that spot is avoided for that box next time. A box is given up after 3 tries, if it isn't standing upright on the table, or if it fits nowhere. When nothing more fits, the packer logs `Packed N/6 boxes` with the reason for each box left out, moves the arm **home** and logs `Job done, arm is home`. If a box can't be planned, the log names the failing stage and the reason. Stop it with Ctrl+C. Each run starts from a fresh random layout.

Options:
```bash
ros2 launch arm_bringup pick_place.launch.py count:=4 seed:=5
```
- `count` (1-6, default 6): how many boxes (one per colour).
- `seed`: repeat a layout. Every run logs its seed (`box seed 5 (rerun with seed:=5)`).

The tray is 20 × 24 cm inside: 6 boxes usually need a second layer (seed 5 stacks two).

To see the table, boxes and tray in RViz, add the **MotionPlanning** display (see step 2). To step through each stage, also add **Add → moveit_task_constructor_visualization → Motion Planning Tasks**.

The table, tray, scatter area, box sizes and colours are in [scene.yaml](arm_bringup/config/scene.yaml), read by the launch file, the packer, the detector and the spawner. The Gazebo world [table.sdf](arm_bringup/worlds/table.sdf) has to match it by hand. The packing rules are constants at the top of [packer.py](arm_commander/scripts/packer.py); check them with `python3 arm_commander/test/test_tetris.py` (after sourcing the workspace).

## Gazebo simulation
The same arm in Gazebo Harmonic, with gravity, contacts and a depth camera. Open 2 terminals and in each run `cd ~/ros2_ws && source install/setup.bash`.

**1. Start the simulation** (terminal 1)
```bash
ros2 launch arm_bringup arm_gz.launch.xml
```
After about 30 s, three windows are open:
- **Gazebo**: the arm bolted to the floor, a 0.7 × 1.2 m table and a grey tray (the boxes are added by pick and place), and a depth camera (640 × 480, 5 Hz) on a stand behind the table.
- **RViz**: the robot, the planning scene (table, tray and boxes) and the camera's point cloud.
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
`sim:=true` spawns the random boxes in Gazebo, replacing any left from an earlier run, and puts the nodes on sim time. The spawn layout goes only to Gazebo: the packer finds and measures the boxes with the depth camera alone, so it doesn't know where they are, how big they are or how many there are. When nothing more fits, it moves the arm home and looks once more, so the arm can't hide a box. The fingers grip each box with a set force, carry it to the tray and set it down beside or on top of the others. `count:=` and `seed:=` work as in mock mode. Check where a box ended up:
```bash
gz model -m box_red -p
```
To run it again, just relaunch it: the old boxes are removed and a new layout is spawned.

**Other nodes on the simulation** need sim time, e.g.:
```bash
ros2 run arm_commander test_moveit --ros-args -p use_sim_time:=true
```

Camera topics: `/camera/image`, `/camera/depth_image`, `/camera/camera_info`, `/camera/points` (frame `camera_link`).

**How the camera finds the boxes**: [box_detector.py](arm_bringup/scripts/box_detector.py) reads the organized point cloud (xyz + colour per pixel) and publishes `/detected_boxes` (`arm_interfaces/DetectedBoxes`): per box, the centre and yaw of its top face in `base_link`, its length and width, and whether it is upright. In Gazebo it is within about 1 mm, 1° and 1 mm in size of the truth.
1. Only the pixels that see the table are searched (worked out once from the first frame), between the table top and 30 cm above it.
2. Compute each pixel's surface normal from its neighbours; pixels facing up are top faces.
3. Per palette colour (by hue), the largest up-facing region is that box's top: the smallest rectangle around it gives centre, yaw, length and width, plus a small edge correction (the normals drop the outermost pixel ring).
4. A colour with no up-facing face is a tipped box, published as not upright so the packer leaves it alone.

Colours keep touching and stacked boxes apart, and give each box its name. The detector doesn't measure height: the packer takes it once, while the box still stands on the table (top − table), and remembers it, since in the tray a box may stand on another box. The packer averages 5 frames per measurement. Check the detector: `python3 arm_bringup/test/test_box_detector.py` (after sourcing the workspace).

How it fits together: `robot_arm.urdf.xacro` takes `sim:=true` to swap mock hardware for `gz_ros2_control` and bolt the base to the Gazebo world; the world ([table.sdf](arm_bringup/worlds/table.sdf)) matches [scene.yaml](arm_bringup/config/scene.yaml). If you move the camera in the world file, update the `camera_link` transform in [arm_gz.launch.xml](arm_bringup/launch/arm_gz.launch.xml) to match.

## Named poses (from the SRDF)
| Group   | Names                                           |
| ------- | ----------------------------------------------- |
| arm     | `home`, `ready` (waiting pose beside the table, out of the camera's view), `pose1`, `pose2` |
| gripper | `gripper_open`, `gripper_half_open`, `gripper_close`, `gripper_grasp` (pick and place sets the fingers from each box's width instead) |

## Known limitations
- In mock mode (`arm.launch.xml`) nothing is physical: pick and place "grasps" by attaching the box in the planning scene. Use the Gazebo launch for a real grasp.
- The detector tells boxes apart by colour: at most 6 boxes (one per palette colour), and anything else with those hues would count as a box. The hue tolerance and edge correction (`HUE_TOL`, `EDGE_PX`) are tuned for the simulated light and camera.
- Box sizes (length, width, height) are measured once, on the table, and remembered: in the tray a box may stand on another or be partly hidden by its neighbours. A box first seen already stacked would get a wrong height.
- A box dropped off the table leaves the camera's view. The packer only reports it as `no longer seen`.
- Boxes are scattered between 0.53 m and 1.0 m from the arm base: closer in, the arm can't come straight down on a box.
- Placements are turned 0° or 90° to the tray only, and the packer never moves a box once it is in the tray.
- The tray's walls rise 0.5 cm above its floor, below the finger tips (1 cm above the box bottom), so the fingers pass over them. Higher tips cleared taller walls but left a flat box only a few millimetres of grip, and boxes slipped out. If you change the tray in [table.sdf](arm_bringup/worlds/table.sdf), update [scene.yaml](arm_bringup/config/scene.yaml) to match.
- Grasps are across the box's width (or its length, when that is 8 cm or less), so the fingers land flat on two faces; a diagonal grasp squeezes the corners and flicks the box out in Gazebo.
- The two fingers are separate joints driven with the same value (the right one has a mirrored axis), so they close together. Two Gazebo workarounds keep them in step. They weigh 0.2 kg each, because very light fingers left the physics solver driving one finger before the other. Their joint ranges also start 1 cm below the open position, because a finger resting on its limit could stall. Without these, one finger pushed the box into the other.
- In Gazebo the fingers are force-controlled: the gripper controller sends effort (PD on position, gains in [ros2_controllers_gz.yaml](arm_bringup/config/ros2_controllers_gz.yaml)), and each finger is capped at `FINGER_FORCE` (15 N) in [gripper.xacro](arm_description/urdf/gripper.xacro). Finger friction is 2.0. A closed finger stops on the box short of its target, so MoveIt allows the fingers' next move to start up to 2 cm off ([moveit_controllers.yaml](arm_moveit_config/config/moveit_controllers.yaml)). Mock mode keeps position control.
- A box that tips over inside the tray still counts as packed.
- The camera stays at 640 × 480. At 1280 × 960 the rendering load starved the force-controlled gripper's control loop (Gazebo fell to half real time and the finger effort updated only every 0.15–0.2 s), so the fingers slammed open and shut and came down closed on the boxes. MoveIt's execution time limits are also relaxed (`allowed_execution_duration_scaling` in [moveit_controllers.yaml](arm_moveit_config/config/moveit_controllers.yaml)) because it times moves on the wall clock while Gazebo may run slower than real time.
- If a pose goal is unreachable or a Cartesian path is <100% feasible, the commander silently does nothing — watch the `move_group` log in terminal 1.
- Commands block the commander's callback while planning/executing; a new command sent mid-motion is queued, not preempting.
