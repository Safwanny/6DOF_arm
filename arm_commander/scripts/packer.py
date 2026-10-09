#!/usr/bin/env python3
"""Pack the cubes into the tray: measure, pick the nearest cube on the table, send it to a free slot, repeat.

The motion itself is one /pick_place call to the C++ task server (pick_place.cpp). Re-measuring after
every cube catches ones that slipped, got knocked or missed their slot, and retries them."""
import math
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from arm_interfaces.srv import PickPlace
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

# Scene layout (base_link, metres), matches pick_place.cpp and table.sdf
TABLE_TOP = 0.15
SIZE = 0.04
# Cubes float this far above the table in the planning scene: touching counts as colliding once attached
CUBE_LIFT = 0.001
TRAY_XY = (0.6, 0.2)
TRAY_INNER = (0.18, 0.27)
SLOT_PITCH = 0.09
SLOT_ROWS, SLOT_COLS = 2, 3  # along x, along y
# Give up on a cube after this many tries (planning or execution), so a stuck cube can't loop forever
MAX_ATTEMPTS = 3
# Gazebo: average this many /detected_cubes frames per measurement to smooth out pixel noise
FRAMES = 5
# Cube spot used when no cube_poses are given (x, y, yaw); pick_place.launch.py randomises them
DEFAULT_CUBE_POSES = [0.6, -0.2, 0.0]


def slot_xy(slot):
    """Slot centre; slot 0 is the -x, -y corner, counting along y first."""
    row, col = divmod(slot, SLOT_COLS)
    return (TRAY_XY[0] + (row - (SLOT_ROWS - 1) / 2) * SLOT_PITCH,
            TRAY_XY[1] + (col - (SLOT_COLS - 1) / 2) * SLOT_PITCH)


class Cube:
    def __init__(self, name, x, y, z, q):
        self.name, self.x, self.y, self.z, self.q = name, x, y, z, q
        # A cube is symmetric, so any face up is fine: find the cube axis closest to world z
        qx, qy, qz, qw = q
        rot = [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
               [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
               [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]
        up = max(range(3), key=lambda k: abs(rot[2][k]))
        side = (up + 1) % 3
        self.upright = abs(rot[2][up]) > 0.97  # a face points up, so it can be grasped from above
        self.yaw = math.atan2(rot[1][side], rot[0][side])

    def in_tray(self):
        return (abs(self.x - TRAY_XY[0]) < TRAY_INNER[0] / 2 and abs(self.y - TRAY_XY[1]) < TRAY_INNER[1] / 2
                and self.z < TABLE_TOP + 0.1)

    def on_table(self):
        """Upright and standing on the table top (not fallen off, not leaning on something)."""
        return (self.upright and abs(self.z - (TABLE_TOP + SIZE / 2)) < 0.01
                and abs(self.x - 0.65) < 0.2 and abs(self.y) < 0.4)

    def nearest_slot(self):
        return min(range(SLOT_ROWS * SLOT_COLS),
                   key=lambda s: math.hypot(self.x - slot_xy(s)[0], self.y - slot_xy(s)[1]))


def yaw_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def cube_object(name, x, y, z, q):
    obj = CollisionObject(id=name, operation=CollisionObject.ADD)
    obj.header.frame_id = 'base_link'
    obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[SIZE, SIZE, SIZE])]
    obj.pose.position.x, obj.pose.position.y, obj.pose.position.z = x, y, z
    o = obj.pose.orientation
    o.x, o.y, o.z, o.w = q
    return obj


class Packer(Node):
    def __init__(self):
        super().__init__('packer')
        self.sim = self.declare_parameter('sim', False).value
        # Mock hardware has no camera: the cubes start where cube_poses says. In Gazebo the camera finds them.
        poses = self.declare_parameter('cube_poses', DEFAULT_CUBE_POSES).value
        if len(poses) % 3 or len(poses) // 3 > SLOT_ROWS * SLOT_COLS:
            raise ValueError(f'cube_poses must be x, y, yaw triples, at most {SLOT_ROWS * SLOT_COLS} cubes')
        self.poses = [poses[i:i + 3] for i in range(0, len(poses), 3)]
        self.pick_place = self.create_client(PickPlace, 'pick_place')
        self.go_home = self.create_client(Trigger, 'go_home')
        self.apply_scene = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.get_scene = self.create_client(GetPlanningScene, 'get_planning_scene')

        # The depth camera's view of the cubes (cube_detector.py)
        self.lock = threading.Lock()
        self.frames = []
        self.create_subscription(TFMessage, 'detected_cubes', self.on_detected, 10)

    def on_detected(self, msg):
        with self.lock:
            self.frames = (self.frames + [{t.child_frame_id: t.transform for t in msg.transforms}])[-FRAMES:]

    def apply(self, objects=(), detach=()):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = list(objects)
        scene.robot_state.is_diff = True
        for name in detach:
            a = AttachedCollisionObject()
            a.object.id, a.object.operation = name, CollisionObject.REMOVE
            scene.robot_state.attached_collision_objects.append(a)
        self.apply_scene.call(ApplyPlanningScene.Request(scene=scene))

    def scene_cubes(self):
        """Mock hardware: nothing moves by itself, so the planning scene is the truth."""
        req = GetPlanningScene.Request()
        req.components.components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
                                      | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
        return self.get_scene.call(req).scene

    def look(self):
        """Each cube in the next FRAMES camera frames, averaged: {name: (x, y, z, quaternion)}.
        Frames taken after the call show the world after the last motion. None if the camera is silent."""
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
        cubes = {}
        for name, last in frames[-1].items():
            seen = [f[name] for f in frames if name in f]
            x, y, z = (sum(getattr(t.translation, a) for t in seen) / len(seen) for a in 'xyz')
            r = last.rotation
            if abs(r.x) > 0.1 or abs(r.y) > 0.1:  # tipped over: the detector tilts it, keep that
                cubes[name] = (x, y, z, (r.x, r.y, r.z, r.w))
                continue
            # The yaw of a cube repeats every 90 degrees: average it as an angle on that circle
            four = [4 * 2 * math.atan2(t.rotation.z, t.rotation.w) for t in seen]
            cubes[name] = (x, y, z, yaw_quat(math.atan2(sum(map(math.sin, four)), sum(map(math.cos, four))) / 4))
        return cubes

    def measure(self):
        """Where the cubes are now, or None if they can't be seen. In Gazebo the planning scene is moved to match."""
        scene = self.scene_cubes()
        # A task that failed mid-carry leaves its cube attached to the hand; put it back in the world
        attached = [a.object.id for a in scene.robot_state.attached_collision_objects]
        if attached:
            self.apply(detach=attached)
        if not self.sim:
            if attached:
                scene = self.scene_cubes()
            cubes = []
            for o in scene.world.collision_objects:
                if o.id.startswith('cube_'):
                    p, q = o.pose.position, o.pose.orientation
                    cubes.append(Cube(o.id, p.x, p.y, p.z, (q.x, q.y, q.z, q.w)))
            return cubes

        seen = self.look()
        if seen is None:
            return None
        cubes = [Cube(name, *m) for name, m in sorted(seen.items())]
        # Cubes the camera no longer sees (fell off the table) leave the planning scene
        gone = [CollisionObject(id=o.id, operation=CollisionObject.REMOVE) for o in scene.world.collision_objects
                if o.id.startswith('cube_') and o.id not in seen]
        # Tipped over: keep the tilt for collisions
        self.apply(gone + [cube_object(c.name, c.x, c.y, c.z + CUBE_LIFT, yaw_quat(c.yaw) if c.upright else c.q)
                           for c in cubes])
        return cubes

    def run(self):
        for client in (self.pick_place, self.go_home, self.apply_scene, self.get_scene):
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info(f'waiting for {client.srv_name}')
        if not self.sim:
            self.apply([cube_object(f'cube_{i}', x, y, TABLE_TOP + SIZE / 2 + CUBE_LIFT, yaw_quat(yaw))
                        for i, (x, y, yaw) in enumerate(self.poses)])

        attempts = {}
        cubes, ever_seen = [], set()
        home = False  # arm already home, out of the camera's view
        while rclpy.ok():
            cubes = self.measure()
            if cubes is None:
                self.get_logger().error('No camera frames on /detected_cubes, is arm_gz.launch.xml running?')
                return
            ever_seen.update(c.name for c in cubes)
            occupied = {c.nearest_slot() for c in cubes if c.in_tray()}
            todo = [c for c in cubes
                    if not c.in_tray() and c.on_table() and attempts.get(c.name, 0) < MAX_ATTEMPTS]
            free = [s for s in range(SLOT_ROWS * SLOT_COLS) if s not in occupied]
            if not todo or not free:
                # Nothing left in sight: look once more from home, where the arm can't hide a cube
                if self.sim and free and not home:
                    home = self.go_home.call(Trigger.Request()).success
                    if home:
                        continue
                break
            home = False
            cube = min(todo, key=lambda c: math.hypot(c.x, c.y))
            slot = free[0]
            attempts[cube.name] = attempts.get(cube.name, 0) + 1
            self.get_logger().info(f'{cube.name} at ({cube.x:.3f}, {cube.y:.3f}) -> slot {slot} '
                                   f'(try {attempts[cube.name]})')
            x, y = slot_xy(slot)
            self.pick_place.call(PickPlace.Request(object=cube.name, x=x, y=y))
            if self.sim:
                time.sleep(1.0)  # let the released cube settle

        packed = 0
        for c in cubes:
            if c.in_tray():
                packed += 1
            else:
                why = ('not standing on the table' if not c.on_table() else
                       'gave up after retries' if attempts.get(c.name, 0) >= MAX_ATTEMPTS else 'tray is full')
                self.get_logger().warn(f'{c.name} left out at ({c.x:.3f}, {c.y:.3f}, {c.z:.3f}): {why}')
        lost = sorted(ever_seen - {c.name for c in cubes})
        for name in lost:
            self.get_logger().warn(f'{name} left out: no longer seen, dropped off the table?')
        self.get_logger().info(f'Packed {packed}/{len(cubes) + len(lost)} cubes')
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
