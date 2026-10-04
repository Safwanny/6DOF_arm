#!/usr/bin/env python3
"""Show the Gazebo camera's colour and depth images side by side in one window."""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

WINDOW = 'Camera: colour | depth'


def main():
    rclpy.init()
    node = rclpy.create_node('camera_view')
    bridge = CvBridge()
    frames = {}
    node.create_subscription(Image, '/camera/image',
                             lambda m: frames.update(colour=bridge.imgmsg_to_cv2(m, 'bgr8')), 1)
    node.create_subscription(Image, '/camera/depth_image',
                             lambda m: frames.update(depth=bridge.imgmsg_to_cv2(m, '32FC1')), 1)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1280, 480)
    shown = False
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        if 'colour' in frames and 'depth' in frames:
            d = frames['depth']
            valid = np.isfinite(d)
            # Scale each frame to its own near/far range: near = red, far = blue
            lo, hi = (d[valid].min(), d[valid].max()) if valid.any() else (0.0, 1.0)
            d8 = np.zeros(d.shape, np.uint8)
            d8[valid] = (255 * (hi - d[valid]) / max(hi - lo, 1e-6)).astype(np.uint8)
            depth = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
            depth[~valid] = 0
            depth = cv2.resize(depth, (frames['colour'].shape[1], frames['colour'].shape[0]))
            cv2.imshow(WINDOW, np.hstack([frames['colour'], depth]))
            shown = True
        cv2.waitKey(1)
        # Closing the window ends the node, which shuts down the whole launch
        if shown and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
