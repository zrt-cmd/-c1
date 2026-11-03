#!/usr/bin/env python3
"""Find the G->C path in the fixed math_room map and report clearances."""
import math
from collections import deque

import numpy as np
from PIL import Image, ImageDraw

RES = 0.01
X_MIN, X_MAX = -2.7, 2.7
Y_MIN, Y_MAX = -2.0, 2.0
W = int(round((X_MAX - X_MIN) / RES)) + 1
H = int(round((Y_MAX - Y_MIN) / RES)) + 1

walls = [
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


def rasterize(inflate):
    grid = np.zeros((H, W), dtype=bool)
    for _, cx, cy, sx, sy in walls:
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
    return int(round((x - X_MIN) / RES)), int(round((y - Y_MIN) / RES))


def to_xy(i, j):
    return X_MIN + i * RES, Y_MIN + j * RES


def bfs(grid, start, goal):
    si, sj = to_idx(*start)
    gi, gj = to_idx(*goal)
    if grid[sj, si] or grid[gj, gi]:
        return None
    prev = -np.ones((H, W, 2), dtype=np.int32)
    seen = np.zeros((H, W), dtype=bool)
    q = deque([(si, sj)])
    seen[sj, si] = True
    while q:
        i, j = q.popleft()
        if (i, j) == (gi, gj):
            path = [(i, j)]
            while (i, j) != (si, sj):
                pi, pj = prev[j, i]
                i, j = int(pi), int(pj)
                path.append((i, j))
            path.reverse()
            return path
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < W and 0 <= nj < H and not seen[nj, ni] and not grid[nj, ni]:
                seen[nj, ni] = True
                prev[nj, ni] = (i, j)
                q.append((ni, nj))
    return None


def dist_to_rect(x, y, cx, cy, sx, sy):
    dx = max(abs(x - cx) - sx / 2, 0.0)
    dy = max(abs(y - cy) - sy / 2, 0.0)
    return math.hypot(dx, dy)


def clearance(x, y):
    return min(dist_to_rect(x, y, cx, cy, sx, sy) for _, cx, cy, sx, sy in walls)


G = (0.15, 1.70)
C = (-1.80, 1.20)

for infl in (0.10, 0.115, 0.125, 0.135):
    grid = rasterize(infl)
    path = bfs(grid, G, C)
    print(f"inflation {infl:.3f}: {'PATH FOUND' if path else 'NO PATH'}")

grid = rasterize(0.10)
path = bfs(grid, G, C)
if path is None:
    raise SystemExit("no path")

pts = [to_xy(i, j) for i, j in path]
# simplify: keep every 5 cm
simp = pts[::5]
if simp[-1] != pts[-1]:
    simp.append(pts[-1])
clear = [clearance(x, y) for x, y in pts]
min_clear = min(clear)
min_idx = clear.index(min_clear)
print(f"path points: {len(pts)}")
print(f"min clearance to wall along path: {min_clear:.3f} m at {pts[min_idx]}")
print("waypoints (x, y):")
for x, y in simp[::10]:
    print(f"  ({x:+.2f}, {y:+.2f})")

# draw
X0, X1 = -3.0, 3.0
Y0, Y1 = -2.3, 2.3
SC = 190
IW = int((X1 - X0) * SC)
IH = int((Y1 - Y0) * SC)


def px(x, y):
    return (x - X0) * SC, (Y1 - y) * SC


img = Image.new("RGB", (IW, IH), (250, 250, 250))
d = ImageDraw.Draw(img)
for name, cx, cy, sx, sy in walls:
    p0 = px(cx - sx / 2, cy + sy / 2)
    p1 = px(cx + sx / 2, cy - sy / 2)
    d.rectangle([p0[0], p0[1], p1[0], p1[1]], fill=(70, 70, 80))
for (x, y) in [(pts[min_idx])]:
    p = px(x, y)
    d.ellipse([p[0] - 8, p[1] - 8, p[0] + 8, p[1] + 8], outline=(255, 0, 0), width=3)
for k in range(len(pts) - 1):
    d.line([px(*pts[k]), px(*pts[k + 1])], fill=(0, 140, 0), width=2)
p = px(*G)
d.ellipse([p[0] - 7, p[1] - 7, p[0] + 7, p[1] + 7], fill=(0, 80, 220))
p = px(*C)
d.ellipse([p[0] - 7, p[1] - 7, p[0] + 7, p[1] + 7], fill=(200, 0, 0))
img.save("/mnt/c/Users/ZRT/Desktop/小车/g_to_c_path.png")
print("saved g_to_c_path.png")
