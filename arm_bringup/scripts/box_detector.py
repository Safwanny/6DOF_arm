#!/usr/bin/env python3
"""Find the coloured boxes in the depth camera's point cloud and measure their top faces in base_link.

/camera/points (organized, xyz + rgb in camera_link) -> /detected_boxes (arm_interfaces/DetectedBoxes).
Every box has its own colour (scene.yaml), so touching and stacked boxes stay apart and keep their name."""
import math
import os

import cv2
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

SCENE = yaml.safe_load(open(os.path.join(get_package_share_directory('arm_bringup'), 'config', 'scene.yaml')))
TABLE_TOP = SCENE['table']['top']
# Boxes on the table, in the tray or stacked; the gripper hand above that is ignored
Z_RANGE = (TABLE_TOP - 0.005, TABLE_TOP + 0.3)
# A pixel belongs to a box colour when its hue is this close (OpenCV hue, 0-180) and it is saturated and lit
HUE_TOL, MIN_SAT, MIN_VAL = 6, 110, 40
MIN_PIXELS = 15
# A surface whose normal is this close to vertical is a top face
UP = 0.9
# The normals drop the outermost pixel ring of a face: add this many pixels per side to its size (tuned in Gazebo)
EDGE_PX = 0.75


def palette_hues(colours):
    """OpenCV hue of each palette colour: {name: hue}."""
    return {name: int(cv2.cvtColor(np.uint8([[[b * 255, g * 255, r * 255]]]), cv2.COLOR_BGR2HSV)[0, 0, 0])
            for name, (r, g, b) in colours.items()}


def detect(xyz, bgr, rot, trans, hues):
    """Boxes as {name: (x, y, top_z, yaw, length, width, upright)} in base_link.

    xyz: h x w x 3 points in the camera frame, bgr: h x w x 3 uint8, rot/trans: camera pose in base_link,
    hues: palette_hues(). The pose is the centre of the top face; yaw points along the length."""
    pts = xyz @ rot.T + trans
    hsv = cv2.cvtColor(np.ascontiguousarray(bgr), cv2.COLOR_BGR2HSV).astype(int)
    hue, lit = hsv[..., 0], (hsv[..., 1] > MIN_SAT) & (hsv[..., 2] > MIN_VAL)
    valid = lit & np.isfinite(pts).all(axis=2) & (pts[..., 2] > Z_RANGE[0]) & (pts[..., 2] < Z_RANGE[1])
    # Surface normal per pixel from its neighbours; |dx| / 2 is the pixel spacing
    dx, dy = np.zeros_like(pts), np.zeros_like(pts)
    dx[:, 1:-1] = pts[:, 2:] - pts[:, :-2]
    dy[1:-1] = pts[2:] - pts[:-2]
    normal = np.cross(dx, dy)
    up = np.abs(normal[..., 2]) > UP * np.linalg.norm(normal, axis=2)

    boxes = {}
    for name, h in hues.items():
        d = np.abs(hue - h)
        mask = valid & (np.minimum(d, 180 - d) <= HUE_TOL)
        if mask.sum() < MIN_PIXELS:
            continue
        # Largest top region of this colour: stray pixels elsewhere don't matter
        top = (mask & up).astype(np.uint8)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(top, connectivity=8)
        if n < 2 or stats[1:, cv2.CC_STAT_AREA].max() < MIN_PIXELS:
            c = pts[mask].mean(axis=0)
            boxes[name] = (float(c[0]), float(c[1]), float(c[2]), 0.0, 0.0, 0.0, False)
            continue
        region = labels == 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        face = pts[region]
        (cx, cy), (a, b), angle = cv2.minAreaRect(face[:, :2].astype(np.float32))
        pad = 2 * EDGE_PX * float(np.median(np.linalg.norm(dx[region], axis=1) + np.linalg.norm(dy[region], axis=1)) / 4)
        yaw = math.radians(angle) if a >= b else math.radians(angle) + math.pi / 2
        boxes[name] = (float(cx), float(cy), float(np.median(face[:, 2])), (yaw + math.pi / 2) % math.pi - math.pi / 2,
                       max(a, b) + pad, min(a, b) + pad, True)
    return boxes


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from arm_interfaces.msg import DetectedBox, DetectedBoxes
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from tf2_ros import Buffer, TransformListener

    class BoxDetector(Node):
        def __init__(self):
            super().__init__('box_detector')
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.hues = palette_hues(SCENE['colours'])
            self.roi = None
            self.pub = self.create_publisher(DetectedBoxes, 'detected_boxes', 10)
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
            trans = np.array([p.x, p.y, p.z])
            xyz = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=False)
            xyz = xyz.reshape(msg.height, msg.width, 3)
            # rgb is a packed float: bytes b, g, r, a
            off = next(f.offset for f in msg.fields if f.name == 'rgb')
            bgr = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, msg.point_step)[..., off:off + 3]
            if self.roi is None:
                self.roi = self.table_pixels(xyz, rot, trans)
            r0, r1, c0, c1 = self.roi
            found = detect(xyz[r0:r1, c0:c1], bgr[r0:r1, c0:c1], rot, trans, self.hues)

            out = DetectedBoxes(header=msg.header)
            out.header.frame_id = 'base_link'
            for name, (x, y, z, yaw, length, width, upright) in sorted(found.items()):
                box = DetectedBox(name=f'box_{name}', length=length, width=width, upright=upright)
                box.top.position.x, box.top.position.y, box.top.position.z = x, y, z
                box.top.orientation.z, box.top.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
                out.boxes.append(box)
            self.pub.publish(out)

        def table_pixels(self, xyz, rot, trans):
            """Rows and columns that see the table (with margin): only these are searched, which keeps
            the 1280 x 960 cloud fast. The camera is fixed, so this is worked out once."""
            t = SCENE['table']
            pts = xyz @ rot.T + trans
            on = ((np.abs(pts[..., 0] - t['x']) < t['size_x'] / 2 + 0.05)
                  & (np.abs(pts[..., 1] - t['y']) < t['size_y'] / 2 + 0.05)
                  & (pts[..., 2] > Z_RANGE[0] - 0.05))
            rows, cols = np.nonzero(on.any(axis=1))[0], np.nonzero(on.any(axis=0))[0]
            margin = 60  # pixels, for tall stacks seen above the table edge
            return (max(rows[0] - margin, 0), rows[-1] + 1, max(cols[0] - margin, 0), cols[-1] + margin + 1)

    rclpy.init()
    node = BoxDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
