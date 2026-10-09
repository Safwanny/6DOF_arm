#!/usr/bin/env python3
"""Pack the boxes into the tray like a small game of Tetris.

Largest footprint first: each box goes beside the boxes already packed while there is room, and on top of
them (stacking) when there isn't. The motion itself is one /pick_place call to the C++ task server
(pick_place.cpp). Re-measuring with the camera after every box catches boxes that slipped or landed off
their spot, and packs around them as they are."""
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from arm_interfaces.msg import DetectedBoxes
from arm_interfaces.srv import PickPlace
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger

SCENE = yaml.safe_load(open(os.path.join(get_package_share_directory('arm_bringup'), 'config', 'scene.yaml')))
TABLE, TRAY = SCENE['table'], SCENE['tray']
TABLE_TOP = TABLE['top']
FLOOR = TABLE_TOP + TRAY['floor']  # top of the tray floor
# Boxes float this far above their support in the planning scene: touching counts as colliding once attached
LIFT = 0.001
# Give up on a box after this many tries (planning or execution), so a stuck box can't loop forever
MAX_ATTEMPTS = 3
# Gazebo: average this many /detected_boxes frames per measurement to smooth out pixel noise
FRAMES = 5

# Tetris rules. Gripper numbers match pick_place.cpp: the release opens 1.5 cm clear of the box and the
# fingers are 2 cm thick and 6 cm wide; their tips stay 1.5 cm above the box bottom.
CELL = 0.005          # height map resolution
GAP = 0.005           # to neighbours and the walls
FLAT = 0.005          # a support cell is this close to the base height
SUPPORT = 0.9         # share of the footprint that must rest on the support
FINGER_DEPTH = 0.035  # release clearance + finger thickness, beside the gripped faces
FINGER_WIDTH = 0.06
TIP_CLEARANCE = 0.015
MAX_GRIP = 0.08       # widest side the open fingers (12 cm apart) can take
MAX_STACK = 0.20      # stack height above the tray floor
WALL = TRAY['height'] - TRAY['floor']  # wall top above the floor


class Box:
    """A box as measured: centre, yaw of its length side, size, and whether a face points up."""

    def __init__(self, name, x, y, z, yaw, length, width, height, upright=True):
        self.name, self.x, self.y, self.z, self.yaw = name, x, y, z, yaw
        self.length, self.width, self.height, self.upright = length, width, height, upright

    def in_tray(self):
        return abs(self.x - TRAY['x']) < TRAY['inner_x'] / 2 and abs(self.y - TRAY['y']) < TRAY['inner_y'] / 2

    def on_table(self):
        """Upright and standing on the table top (not fallen off, not leaning on something)."""
        return (self.upright and abs(self.z - self.height / 2 - TABLE_TOP) < 0.01
                and abs(self.x - TABLE['x']) < TABLE['size_x'] / 2 and abs(self.y - TABLE['y']) < TABLE['size_y'] / 2)


def tray_grid(placed):
    """Height map of the tray above its floor, CELL square, padded around the inside with the wall height
    so finger zones over the walls can be checked. placed: boxes in the tray. Returns (map, pad)."""
    pad = math.ceil((FINGER_DEPTH + GAP) / CELL) + 1
    nx, ny = round(TRAY['inner_x'] / CELL), round(TRAY['inner_y'] / CELL)
    grid = np.full((nx + 2 * pad, ny + 2 * pad), WALL)
    grid[pad:pad + nx, pad:pad + ny] = 0.0
    x0, y0 = TRAY['x'] - TRAY['inner_x'] / 2, TRAY['y'] - TRAY['inner_y'] / 2
    i, j = np.mgrid[0:grid.shape[0], 0:grid.shape[1]]
    cx, cy = x0 + (i - pad + 0.5) * CELL, y0 + (j - pad + 0.5) * CELL
    for b in placed:
        u = (cx - b.x) * math.cos(b.yaw) + (cy - b.y) * math.sin(b.yaw)
        v = -(cx - b.x) * math.sin(b.yaw) + (cy - b.y) * math.cos(b.yaw)
        inside = (np.abs(u) <= b.length / 2) & (np.abs(v) <= b.width / 2)
        grid[inside] = np.maximum(grid[inside], b.z + b.height / 2 - FLOOR)
    return grid, pad


def place(length, width, height, grid, pad):
    """Where the next box goes, as (x, y, bottom z, grip yaw, grip width) in base_link, or None if it fits nowhere.

    Tried: the box turned 0 or 90 degrees, gripped across its width (or its length if that is narrow enough),
    at every cell. A spot must keep GAP to everything, rest SUPPORT of its footprint (and its centre) on a
    flat base, and leave room for the fingers beside the gripped faces. Best spot: lowest base (beside before
    above), then nearest the tray corner closest to the arm, then the long side along the tray's long side."""
    nx, ny = grid.shape[0] - 2 * pad, grid.shape[1] - 2 * pad
    gap, depth, fw = math.ceil(GAP / CELL), math.ceil(FINGER_DEPTH / CELL), math.ceil(FINGER_WIDTH / CELL)
    x0, y0 = TRAY['x'] - TRAY['inner_x'] / 2, TRAY['y'] - TRAY['inner_y'] / 2
    long_y = TRAY['inner_y'] >= TRAY['inner_x']
    best = None
    for turned in (False, True):  # length along tray x, or along tray y
        fx, fy = (width, length) if turned else (length, width)
        kx, ky = math.ceil(fx / CELL - 1e-9), math.ceil(fy / CELL - 1e-9)
        for grip_length in (False, True):
            grip = length if grip_length else width
            if grip > MAX_GRIP or (grip_length and length == width):
                continue
            grip_along_x = grip_length != turned
            for i in range(pad + gap, pad + nx - gap - kx + 1):
                for j in range(pad + gap, pad + ny - gap - ky + 1):
                    base = grid[i - gap:i + kx + gap, j - gap:j + ky + gap].max()
                    if base + height > MAX_STACK:
                        continue
                    score = (round(base / CELL), i + j, turned != long_y)
                    if best is not None and score >= best[0]:
                        continue
                    foot = grid[i:i + kx, j:j + ky] >= base - FLAT
                    if foot.mean() < SUPPORT or not foot[kx // 2, ky // 2]:
                        continue
                    if grip_along_x:
                        c = j + ky // 2 - fw // 2
                        zones = (grid[i - depth:i, c:c + fw], grid[i + kx:i + kx + depth, c:c + fw])
                    else:
                        c = i + kx // 2 - fw // 2
                        zones = (grid[c:c + fw, j - depth:j], grid[c:c + fw, j + ky:j + ky + depth])
                    if max(z.max() for z in zones) > base + TIP_CLEARANCE - GAP + 1e-9:
                        continue
                    x = x0 + (i - pad + kx / 2) * CELL
                    y = y0 + (j - pad + ky / 2) * CELL
                    best = (score, (x, y, FLOOR + base, 0.0 if grip_along_x else math.pi / 2, grip))
    return best[1] if best else None


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def box_object(name, size, x, y, z, yaw=0.0):
    obj = CollisionObject(id=name, operation=CollisionObject.ADD)
    obj.header.frame_id = 'base_link'
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])]
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = float(x), float(y), float(z)
    o = obj.pose.orientation
    o.x, o.y, o.z, o.w = yaw_quat(yaw)
    return obj


def scene_box(b, grip_length=False):
    """The planning-scene box, its x axis along the side the fingers grip (what pick_place.cpp expects)."""
    if grip_length:
        return box_object(b.name, (b.length, b.width, b.height), b.x, b.y, b.z + LIFT, b.yaw)
    return box_object(b.name, (b.width, b.length, b.height), b.x, b.y, b.z + LIFT, b.yaw + math.pi / 2)


def table_and_tray():
    """The fixed scene: table, and the tray as its floor plus 4 walls, all primitives of one object."""
    t = TRAY
    ox, oy = t['inner_x'] + 2 * t['wall'], t['inner_y'] + 2 * t['wall']
    wx, wy = (t['inner_x'] + t['wall']) / 2, (t['inner_y'] + t['wall']) / 2
    parts = [((ox, oy, t['floor']), (0, 0, t['floor'] / 2)),
             ((t['wall'], oy, t['height']), (-wx, 0, t['height'] / 2)),
             ((t['wall'], oy, t['height']), (wx, 0, t['height'] / 2)),
             ((ox, t['wall'], t['height']), (0, -wy, t['height'] / 2)),
             ((ox, t['wall'], t['height']), (0, wy, t['height'] / 2))]
    tray = box_object('tray', parts[0][0], t['x'], t['y'], TABLE_TOP)
    tray.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size]) for size, _ in parts]
    tray.primitive_poses = []
    for _, (px, py, pz) in parts:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(px), float(py), float(pz)
        pose.orientation.w = 1.0
        tray.primitive_poses.append(pose)
    table = box_object('table', (TABLE['size_x'], TABLE['size_y'], TABLE_TOP), TABLE['x'], TABLE['y'], TABLE_TOP / 2)
    return [table, tray]


class Packer(Node):
    def __init__(self):
        super().__init__('packer')
        self.sim = self.declare_parameter('sim', False).value
        # Mock hardware has no camera: the boxes start as the launch file sampled them
        # (x, y, yaw, length, width, height per box). In Gazebo the camera finds and measures them.
        names = self.declare_parameter('box_names', ['box_red']).value
        sizes = self.declare_parameter('boxes', [0.6, -0.2, 0.0, 0.08, 0.05, 0.04]).value
        self.mock_boxes = [Box(n, x, y, TABLE_TOP + h / 2, yaw, l, w, h)
                           for n, (x, y, yaw, l, w, h) in zip(names, np.reshape(sizes, (-1, 6)).tolist())]
        self.pick_place = self.create_client(PickPlace, 'pick_place')
        self.go_home = self.create_client(Trigger, 'go_home')
        self.apply_scene = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.get_scene = self.create_client(GetPlanningScene, 'get_planning_scene')

        # The depth camera's view of the boxes (box_detector.py), and each box's height: measured once,
        # standing on the table, since in the tray it may stand on another box
        self.lock = threading.Lock()
        self.frames = []
        self.heights = {}
        self.create_subscription(DetectedBoxes, 'detected_boxes', self.on_detected, 10)

    def on_detected(self, msg):
        with self.lock:
            self.frames = (self.frames + [{b.name: b for b in msg.boxes}])[-FRAMES:]

    def apply(self, objects=(), detach=()):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = list(objects)
        scene.robot_state.is_diff = True
        for name in detach:
            a = AttachedCollisionObject()
            a.object.id, a.object.operation = name, CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(a)
        self.apply_scene.call(ApplyPlanningScene.Request(scene=scene))

    def scene_objects(self):
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                      | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        return self.get_scene.call(req).scene

    def look(self):
        """Each box over the next FRAMES camera frames, averaged. None if the camera is silent."""
        with self.lock:
            self.frames = []
        for _ in range(100):  # 5 s at the camera's 10 Hz
            time.sleep(0.05)
            with self.lock:
                if len(self.frames) >= FRAMES:
                    break
        with self.lock:
            frames = list(self.frames)
        if len(frames) < FRAMES:
            return None
        boxes = []
        for name, last in frames[-1].items():
            seen = [f[name] for f in frames if name in f]
            x, y, top = (sum(getattr(b.top.position, a) for b in seen) / len(seen) for a in 'xyz')
            if not last.upright:
                boxes.append(Box(name, x, y, top, 0.0, 0.0, 0.0, self.heights.get(name, 0.0), False))
                continue
            # The yaw of a box repeats every 180 degrees: average it as an angle on that circle
            two = [4 * math.atan2(b.top.orientation.z, b.top.orientation.w) for b in seen]
            yaw = math.atan2(sum(map(math.sin, two)), sum(map(math.cos, two))) / 2
            length = sum(b.length for b in seen) / len(seen)
            width = sum(b.width for b in seen) / len(seen)
            if name not in self.heights:
                self.heights[name] = top - TABLE_TOP
            h = self.heights[name]
            boxes.append(Box(name, x, y, top - h / 2, yaw, length, width, h))
        return boxes

    def measure(self):
        """Where the boxes are now, or None if they can't be seen. The planning scene is moved to match."""
        scene = self.scene_objects()
        # A task that failed mid-carry leaves its box attached to the hand; put it back in the world
        attached = [a.object.id for a in scene.robot_state.attached_collision_objects]
        if attached:
            self.apply(detach=attached)
        if not self.sim:
            # Mock hardware: nothing moves by itself, so the planning scene is the truth
            if attached:
                scene = self.scene_objects()
            boxes = []
            for o in scene.world.collision_objects:
                if o.id.startswith('box_'):
                    gx, gy, h = o.primitives[0].dimensions
                    q, p = o.pose.orientation, o.pose.position
                    yaw = 2 * math.atan2(q.z, q.w)  # of the gripped side (x)
                    if gx >= gy:
                        boxes.append(Box(o.id, p.x, p.y, p.z - LIFT, yaw, gx, gy, h))
                    else:
                        boxes.append(Box(o.id, p.x, p.y, p.z - LIFT, yaw - math.pi / 2, gy, gx, h))
            return boxes

        boxes = self.look()
        if boxes is None:
            return None
        names = {b.name for b in boxes}
        # Boxes the camera no longer sees (fell off the table) leave the planning scene
        gone = [CollisionObject(id=o.id, operation=CollisionObject.REMOVE) for o in scene.world.collision_objects
                if o.id.startswith('box_') and o.id not in names]
        self.apply(gone + [scene_box(b) for b in boxes if b.upright])
        return boxes

    def run(self):
        for client in (self.pick_place, self.go_home, self.apply_scene, self.get_scene):
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'waiting for {client.srv_name}')
        # Clear an earlier run, including a box left attached to the hand by a stopped task
        old = self.scene_objects()
        self.apply(detach=[a.object.id for a in old.robot_state.attached_collision_objects])
        self.apply([CollisionObject(id=o.id, operation=CollisionObject.REMOVE)
                    for o in self.scene_objects().world.collision_objects] + table_and_tray())
        if not self.sim:
            self.apply([scene_box(b) for b in self.mock_boxes])

        attempts, no_fit = {}, set()
        boxes, ever_seen = [], set()
        home = False  # arm already home, out of the camera's view
        while rclpy.ok():
            boxes = self.measure()
            if boxes is None:
                self.get_logger().error('No camera frames on /detected_boxes, is arm_gz.launch.xml running?')
                return
            ever_seen.update(b.name for b in boxes)
            todo = sorted((b for b in boxes if not b.in_tray() and b.on_table()
                           and attempts.get(b.name, 0) < MAX_ATTEMPTS),
                          key=lambda b: (-b.length * b.width, -b.length))
            grid, pad = tray_grid([b for b in boxes if b.in_tray()])
            box, spot, no_fit = None, None, set()
            for b in todo:
                spot = place(b.length, b.width, b.height, grid, pad)
                if spot:
                    box = b
                    break
                no_fit.add(b.name)
            if box is None:
                # Nothing left to pack in sight: look once more from home, where the arm can't hide a box
                if self.sim and not home:
                    home = self.go_home.call(Trigger.Request()).success
                    if home:
                        continue
                break
            home = False
            x, y, z, grip_yaw, grip = spot
            grip_length = grip == box.length and box.length != box.width
            attempts[box.name] = attempts.get(box.name, 0) + 1
            where = 'on the floor' if z - FLOOR < FLAT else f'stacked {(z - FLOOR) * 100:.1f} cm up'
            self.get_logger().info(
                f'{box.name} {box.length * 100:.1f} x {box.width * 100:.1f} x {box.height * 100:.1f} cm at '
                f'({box.x:.3f}, {box.y:.3f}) -> ({x:.3f}, {y:.3f}) {where} (try {attempts[box.name]})')
            self.apply([scene_box(box, grip_length)])  # its x axis along the side the fingers take
            self.pick_place.call(PickPlace.Request(object=box.name, grip_width=grip, height=box.height,
                                                   x=x, y=y, z=z, yaw=grip_yaw))
            if self.sim:
                time.sleep(1.0)  # let the released box settle

        packed = 0
        for b in boxes:
            if b.in_tray():
                packed += 1
            else:
                why = ('not standing upright on the table' if not b.on_table() else
                       'gave up after retries' if attempts.get(b.name, 0) >= MAX_ATTEMPTS else
                       'does not fit in the tray' if b.name in no_fit else 'left on the table')
                self.get_logger().warn(f'{b.name} left out at ({b.x:.3f}, {b.y:.3f}, {b.z:.3f}): {why}')
        lost = sorted(ever_seen - {b.name for b in boxes})
        for name in lost:
            self.get_logger().warn(f'{name} left out: no longer seen, dropped off the table?')
        self.get_logger().info(f'Packed {packed}/{len(boxes) + len(lost)} boxes')
        if home or self.go_home.call(Trigger.Request()).success:
            self.get_logger().info('Job done, arm is home')


def main():
    rclpy.init()
    node = Packer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
