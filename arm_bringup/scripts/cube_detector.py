#!/usr/bin/env python3
"""Find the red cubes in the depth camera's point cloud and publish their poses in base_link.

/camera/points (organized, xyz + rgb in camera_link) -> /detected_cubes (TFMessage, one transform per cube).
Names stay with the cube between frames, so pick and place can track which cube it moved."""
import math

import cv2
import numpy as np

TABLE_TOP, SIZE = 0.15, 0.04  # match pick_place.cpp
# Red cube colour (0.8, 0.1, 0.1) under the scene light; table is brown, tray grey, arm grey/blue/green
MIN_RED, MAX_GREEN_BLUE = 120, 80
# Cubes on the table or in the tray; a cube lifted in the gripper is ignored
Z_RANGE = (TABLE_TOP, TABLE_TOP + 0.1)
MIN_PIXELS = 30
# Points this close to the highest point of a blob belong to its top face
TOP_FACE = 0.003
# A detection this close to a cube's last spot is that cube
SAME_CUBE = 0.03


def detect(xyz, bgr, rot, trans):
    """Cubes as (x, y, z, yaw) in base_link.

    xyz: h x w x 3 points in the camera frame, bgr: h x w x 3 uint8, rot/trans: camera pose in base_link."""
    b, g, r = (bgr[..., i].astype(int) for i in range(3))
    mask = (r > MIN_RED) & (g < MAX_GREEN_BLUE) & (b < MAX_GREEN_BLUE) & np.isfinite(xyz).all(axis=2)
    pts = xyz @ rot.T + trans
    mask &= (pts[..., 2] > Z_RANGE[0]) & (pts[..., 2] < Z_RANGE[1])
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    cubes = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < MIN_PIXELS:
            continue
        blob = pts[labels == i]
        top_z = blob[:, 2].max()
        top = blob[blob[:, 2] > top_z - TOP_FACE]
        x, y = top[:, 0].mean(), top[:, 1].mean()
        (_, _), (_, _), angle = cv2.minAreaRect(top[:, :2].astype(np.float32))
        cubes.append((float(x), float(y), float(top_z - SIZE / 2), math.radians(angle) % (math.pi / 2)))
    return cubes


def match(prev, dets):
    """Give each detection a cube name: {name: (x, y, z, yaw)}.

    prev holds every known cube's last spot, so a cube that is briefly hidden keeps its name."""
    named, left, free = {}, list(dets), dict(prev)

    def take(name, det):
        named[name] = det
        left.remove(det)
        del free[name]

    def dist(name, det):
        return math.hypot(free[name][0] - det[0], free[name][1] - det[1])

    # Cubes that stayed put
    for det in sorted(dets, key=lambda d: min((dist(n, d) for n in free), default=math.inf)):
        name = min(free, key=lambda n: dist(n, det), default=None)
        if name is not None and dist(name, det) < SAME_CUBE:
            take(name, det)
    # Cubes that moved (picked and placed): leftover names to leftover detections, nearest first
    while left and free:
        name, det = min(((n, d) for n in free for d in left), key=lambda nd: dist(*nd))
        take(name, det)
    # Cubes seen for the first time
    i = 0
    for det in list(left):
        while f'cube_{i}' in prev or f'cube_{i}' in named:
            i += 1
        named[f'cube_{i}'] = det
        left.remove(det)
    return named


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener

    class CubeDetector(Node):
        def __init__(self):
            super().__init__('cube_detector')
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.known = {}
            self.pub = self.create_publisher(TFMessage, 'detected_cubes', 10)
            self.create_subscription(PointCloud2, '/camera/points', self.on_cloud, 1)

        def on_cloud(self, msg):
            try:
                t = self.tf.lookup_transform('base_link', msg.header.frame_id, Time())
            except Exception as e:  # static TF not received yet
                self.get_logger().warn(f'no camera transform yet: {e}', throttle_duration_sec=5.0)
                return
            q, p = t.transform.rotation, t.transform.translation
            rot = np.array([
                [1 - 2 * (q.y**2 + q.z**2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x**2 + q.z**2), 2 * (q.y * q.z - q.x * q.w)],
                [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x**2 + q.y**2)]])
            xyz = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=False)
            xyz = xyz.reshape(msg.height, msg.width, 3)
            # rgb is a packed float: bytes b, g, r, a
            off = next(f.offset for f in msg.fields if f.name == 'rgb')
            raw = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, msg.point_step)
            named = match(self.known, detect(xyz, raw[..., off:off + 3], rot, np.array([p.x, p.y, p.z])))
            self.known.update(named)

            out = TFMessage()
            for name, (x, y, z, yaw) in sorted(named.items()):
                c = TransformStamped()
                c.header.stamp, c.header.frame_id, c.child_frame_id = msg.header.stamp, 'base_link', name
                c.transform.translation.x, c.transform.translation.y, c.transform.translation.z = x, y, z
                c.transform.rotation.z, c.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                out.transforms.append(c)
            self.pub.publish(out)

    rclpy.init()
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
