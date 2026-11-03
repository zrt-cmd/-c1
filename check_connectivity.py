#!/usr/bin/env python3
"""Raster connectivity check for math_room: can the car get from G to C?"""
import numpy as np
from collections import deque

RES = 0.01
X_MIN, X_MAX = -2.7, 2.7
Y_MIN, Y_MAX = -2.0, 2.0
W = int(round((X_MAX - X_MIN) / RES)) + 1
H = int(round((Y_MAX - Y_MIN) / RES)) + 1

# name, cx, cy, sx, sy
base_walls = [
    ("wall_north", 0.0, 2.0, 5.5, 0.1),
    ("wall_south_left", -1.65, -2.0, 2.2, 0.1),
    ("wall_south_right", 1.65, -2.0, 2.2, 0.1),
    ("wall_west", -2.7, 0.0, 0.1, 4.0),
    ("wall_east", 2.7, 0.0, 0.1, 4.0),
    ("entrance_left", -0.6, -2.6, 0.1, 1.2),
    ("entrance_right", 0.6, -2.6, 0.1, 1.2),
    ("divider_left", -2.075, 0.35, 1.25, 0.1),
    ("divider_right", 1.075, 0.35, 3.25, 0.1),
    ("upper_left_vertical", -0.9, 1.45, 0.1, 1.10),
    ("step_vertical_left", -0.35, 1.7, 0.1, 0.6),
    ("step_horizontal", 0.075, 1.45, 0.85, 0.1),
    ("step_vertical_right", 0.50, 1.175, 0.1, 0.55),
    ("step_horizontal_lower", 1.025, 0.90, 1.15, 0.1),
    ("upper_vertical_right", 1.05, 1.6875, 0.1, 0.625),
    ("u_left", 1.60, 1.20, 0.1, 0.50),
    ("u_top", 1.875, 1.45, 0.55, 0.1),
    ("u_right", 2.15, 1.20, 0.1, 0.50),
    ("short_vertical", 0.15, 0.625, 0.1, 0.45),
]


def rasterize(walls, inflate):
    grid = np.zeros((H, W), dtype=bool)
    for name, cx, cy, sx, sy in walls:
        x0 = cx - sx / 2 - inflate
        x1 = cx + sx / 2 + inflate
        y0 = cy - sy / 2 - inflate
        y1 = cy + sy / 2 + inflate
        i0 = max(0, int((x0 - X_MIN) / RES))
        i1 = min(W - 1, int((x1 - X_MIN) / RES))
        j0 = max(0, int((y0 - Y_MIN) / RES))
        j1 = min(H - 1, int((y1 - Y_MIN) / RES))
        grid[j0:j1 + 1, i0:i1 + 1] = True
    return grid


def to_idx(x, y):
    i = int(round((x - X_MIN) / RES))
    j = int(round((y - Y_MIN) / RES))
    return i, j


def reachable(grid, start, goal):
    si, sj = to_idx(*start)
    gi, gj = to_idx(*goal)
    if grid[sj, si] or grid[gj, gi]:
        return False, "start/goal inside inflated wall"
    seen = np.zeros_like(grid, dtype=bool)
    q = deque([(si, sj)])
    seen[sj, si] = True
    while q:
        i, j = q.popleft()
        if i == gi and j == gj:
            return True, "reachable"
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < W and 0 <= nj < H and not seen[nj, ni] and not grid[nj, ni]:
                seen[nj, ni] = True
                q.append((ni, nj))
    return False, "not connected"


G = (0.15, 1.70)
C = (-1.80, 1.20)

variants = {}
variants["current"] = base_walls
variants["no_lower"] = [w for w in base_walls if w[0] != "step_horizontal_lower"]

# variant: no lower horizontal, verticals extended down to divider (y=0.45..1.45)
extended = [w for w in base_walls if w[0] != "step_horizontal_lower"]
ext_adj = []
for name, cx, cy, sx, sy in extended:
    if name == "step_vertical_right":
        ext_adj.append((name, cx, 0.95, sx, 1.00))
    elif name in ("u_left", "u_right"):
        ext_adj.append((name, cx, 0.95, sx, 1.00))
    else:
        ext_adj.append((name, cx, cy, sx, sy))
variants["extend_to_divider"] = ext_adj

for inflate in (0.13, 0.15, 0.17):
    print(f"=== inflation {inflate:.2f} m ===")
    for name, walls in variants.items():
        grid = rasterize(walls, inflate)
        ok, msg = reachable(grid, G, C)
        print(f"  {name:20s}: {msg}")
