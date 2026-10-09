#!/usr/bin/env python3
"""Self-check for cube_detector: python3 arm_bringup/test/test_cube_detector.py (no ROS needed)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from cube_detector import SIZE, TABLE_TOP, detect, match  # noqa: E402


def render(cubes, h=240, w=320, step=0.002):
    """Straight-down view of the table, camera frame = base_link: only top faces are seen."""
    ys, xs = np.mgrid[0:h, 0:w]
    xyz = np.stack([0.4 + xs * step, -0.3 + ys * step, np.full((h, w), TABLE_TOP)], axis=2)
    bgr = np.zeros((h, w, 3), np.uint8)
    bgr[:] = (50, 90, 128)  # brown table
    for x, y, yaw in cubes:
        dx, dy = xyz[..., 0] - x, xyz[..., 1] - y
        u = dx * math.cos(yaw) + dy * math.sin(yaw)
        v = -dx * math.sin(yaw) + dy * math.cos(yaw)
        on = (abs(u) < SIZE / 2) & (abs(v) < SIZE / 2)
        xyz[on, 2] = TABLE_TOP + SIZE
        bgr[on] = (25, 25, 200)  # red cube
    return xyz, bgr


truth = [(0.5, -0.2, 0.3), (0.7, -0.1, 1.1)]
found = sorted(detect(*render(truth), np.eye(3), np.zeros(3)))
assert len(found) == 2, found
for (x, y, yaw), (fx, fy, fz, fyaw) in zip(truth, found):
    assert math.hypot(fx - x, fy - y) < 0.005, (x, y, fx, fy)
    assert abs(fz - (TABLE_TOP + SIZE / 2)) < 0.002, fz
    err = (fyaw - yaw) % (math.pi / 2)
    assert min(err, math.pi / 2 - err) < math.radians(3), (yaw, fyaw)

# Names follow the cubes: a small shift keeps them, a cube moved to the tray keeps its name too
prev = {'cube_0': (0.5, -0.2, 0.17, 0), 'cube_1': (0.7, -0.1, 0.17, 0)}
assert match(prev, [(0.701, -0.101, 0.17, 0), (0.501, -0.199, 0.17, 0)]) == {
    'cube_0': (0.501, -0.199, 0.17, 0), 'cube_1': (0.701, -0.101, 0.17, 0)}
moved = match(prev, [(0.7, -0.1, 0.17, 0), (0.555, 0.11, 0.175, 0)])
assert moved == {'cube_1': (0.7, -0.1, 0.17, 0), 'cube_0': (0.555, 0.11, 0.175, 0)}, moved
# A hidden cube is simply missing; a new one gets a fresh name
assert match(prev, [(0.7, -0.1, 0.17, 0)]) == {'cube_1': (0.7, -0.1, 0.17, 0)}
assert set(match(prev, [(0.5, -0.2, 0.17, 0), (0.7, -0.1, 0.17, 0), (0.6, -0.3, 0.17, 0)])) == {
    'cube_0', 'cube_1', 'cube_2'}
print('cube_detector ok')
