#!/usr/bin/env python3
"""Self-check for box_detector: python3 arm_bringup/test/test_box_detector.py (after sourcing the workspace)."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from box_detector import SCENE, TABLE_TOP, detect, palette_hues  # noqa: E402


def bgr(colour):
    r, g, b = SCENE['colours'][colour]
    return (int(b * 255), int(g * 255), int(r * 255))


def render(boxes, tipped=(), h=460, w=620, step=0.0013):
    """Straight-down view of the table at about the real camera resolution, camera frame = base_link.
    boxes: (colour, x, y, yaw, length, width, height). A tipped box shows as a patch sloping at 45 degrees."""
    ys, xs = np.mgrid[0:h, 0:w]
    xyz = np.stack([0.3 + xs * step, -0.3 + ys * step, np.full((h, w), TABLE_TOP)], axis=2)
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (140, 140, 140)  # grey table
    for colour, x, y, yaw, length, width, height in boxes:
        dx, dy = xyz[..., 0] - x, xyz[..., 1] - y
        u = dx * math.cos(yaw) + dy * math.sin(yaw)
        v = -dx * math.sin(yaw) + dy * math.cos(yaw)
        on = (abs(u) < length / 2) & (abs(v) < width / 2)
        xyz[on, 2] = TABLE_TOP + height
        img[on] = bgr(colour)
    for colour, x, y in tipped:
        on = (abs(xyz[..., 0] - x) < 0.02) & (abs(xyz[..., 1] - y) < 0.02)
        xyz[on, 2] = TABLE_TOP + 0.03 + (xyz[on, 0] - x)
        img[on] = bgr(colour)
    return xyz, img


# Two boxes touching side by side, of different heights; one apart and turned; one tipped over
truth = {'red': (0.50, -0.20, 0.0, 0.12, 0.06, 0.05),
         'yellow': (0.50, -0.20 + 0.03 + 0.025, 0.0, 0.10, 0.05, 0.03),
         'cyan': (0.70, 0.0, 0.6, 0.08, 0.04, 0.07)}
found = detect(*render([(c, *t) for c, t in truth.items()], tipped=[('purple', 0.85, 0.15)]),
               np.eye(3), np.zeros(3), palette_hues(SCENE['colours']))
assert set(found) == {'red', 'yellow', 'cyan', 'purple'}, found
assert not found['purple'][6], found['purple']
for colour, (x, y, yaw, length, width, height) in truth.items():
    fx, fy, fz, fyaw, flen, fwid, upright = found[colour]
    assert upright, colour
    assert math.hypot(fx - x, fy - y) < 0.004, (colour, x, y, fx, fy)
    assert abs(fz - (TABLE_TOP + height)) < 0.002, (colour, fz)
    assert abs(flen - length) < 0.005 and abs(fwid - width) < 0.005, (colour, flen, fwid)
    err = (fyaw - yaw) % math.pi
    assert min(err, math.pi - err) < math.radians(3), (colour, yaw, fyaw)
print('box_detector ok')
