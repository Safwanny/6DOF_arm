#!/usr/bin/env python3
"""Self-check for the Tetris packing in packer.py: python3 arm_commander/test/test_tetris.py
(after sourcing the workspace, for scene.yaml)."""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from packer import FLAT, FLOOR, GAP, MAX_GRIP, TRAY, Box, place, tray_grid  # noqa: E402

x0, x1 = TRAY['x'] - TRAY['inner_x'] / 2, TRAY['x'] + TRAY['inner_x'] / 2
y0, y1 = TRAY['y'] - TRAY['inner_y'] / 2, TRAY['y'] + TRAY['inner_y'] / 2


def put(placed, name, length, width, height):
    """Place a box with place() and add it to the tray as placed. Returns the spot or None."""
    spot = place(length, width, height, *tray_grid(placed))
    if spot:
        x, y, z, grip_yaw, grip = spot
        yaw = grip_yaw if grip == length and length != width else grip_yaw + math.pi / 2
        placed.append(Box(name, x, y, z + height / 2, yaw, length, width, height))
    return spot


def rect(b):
    """Axis-aligned footprint (placements are turned 0 or 90 degrees)."""
    along_x = abs(math.cos(b.yaw)) > 0.5
    hx, hy = (b.length / 2, b.width / 2) if along_x else (b.width / 2, b.length / 2)
    return b.x - hx, b.x + hx, b.y - hy, b.y + hy


def check(placed):
    for b in placed:
        bx0, bx1, by0, by1 = rect(b)
        # Inside the tray, clear of the walls
        assert bx0 >= x0 + GAP - 1e-6 and bx1 <= x1 - GAP + 1e-6 and by0 >= y0 + GAP - 1e-6 and by1 <= y1 - GAP + 1e-6, b.name
        bottom = b.z - b.height / 2
        below = []
        for o in placed:
            if o is b:
                continue
            ox0, ox1, oy0, oy1 = rect(o)
            overlap = min(bx1, ox1) - max(bx0, ox0) > 1e-6 and min(by1, oy1) - max(by0, oy0) > 1e-6
            obottom, otop = o.z - o.height / 2, o.z + o.height / 2
            if overlap:
                # Never inside each other: one stands on the other
                assert bottom >= otop - FLAT or obottom >= bottom + b.height - FLAT, (b.name, o.name)
                if abs(bottom - otop) < FLAT:
                    below.append(o)
        # On the floor, or resting on boxes below
        assert abs(bottom - FLOOR) < FLAT or below, b.name


# The first (largest) box goes to the tray corner nearest the arm, on the floor
placed = []
x, y, z, grip_yaw, grip = put(placed, 'first', 0.14, 0.07, 0.04)
assert abs(z - FLOOR) < 1e-9 and grip == 0.07, (z, grip)
bx0, _, by0, _ = rect(placed[0])
assert bx0 < x0 + 0.02 and by0 < y0 + 0.02, rect(placed[0])

# The next one goes beside it, still on the floor, with room for the fingers
put(placed, 'second', 0.10, 0.06, 0.04)
assert abs(placed[1].z - placed[1].height / 2 - FLOOR) < 1e-9
check(placed)

# Keep adding: once the floor is full, boxes go on top
rng = random.Random(1)
for k in range(40):
    if not put(placed, f'box{k}', rng.uniform(0.04, 0.12), rng.uniform(0.03, 0.06), rng.uniform(0.02, 0.05)):
        break
check(placed)
assert any(b.z - b.height / 2 > FLOOR + FLAT for b in placed), 'nothing was stacked'

# Too long for the tray, or too wide for the gripper either way: no spot
assert place(0.40, 0.05, 0.03, *tray_grid([])) is None
assert place(0.10, MAX_GRIP + 0.01, 0.03, *tray_grid([])) is None

# A 5 cm channel between two boxes fits a 3.5 cm box, but not the 6 cm wide fingers either way round
gap_boxes = [Box('a', x0 + 0.055, TRAY['y'], FLOOR + 0.02, math.pi / 2, TRAY['inner_y'] - 0.02, 0.09, 0.04),
             Box('b', x1 - 0.055, TRAY['y'], FLOOR + 0.02, math.pi / 2, TRAY['inner_y'] - 0.02, 0.09, 0.04)]
spot = place(0.04, 0.035, 0.03, *tray_grid(gap_boxes))
assert spot is not None and spot[2] > FLOOR + FLAT, spot  # goes on top instead

# A spot the arm failed to reach is avoided next time
first = place(0.10, 0.05, 0.03, *tray_grid([]))
second = place(0.10, 0.05, 0.03, *tray_grid([]), avoid=[first[:4]])
assert second is not None and math.hypot(second[0] - first[0], second[1] - first[1]) >= 0.01, (first, second)

print(f'tetris ok: {len(placed)} boxes, {sum(b.z - b.height / 2 > FLOOR + FLAT for b in placed)} stacked')
