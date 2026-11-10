#!/usr/bin/env python3
"""Build a stop-and-turn waypoint route G->C from the fixed map."""
import json
import math
from collections import deque

import numpy as np

RES = 0.01
X_MIN, X_MAX = -2.7, 2.7
Y_MIN, Y_MAX = -2.0, 2.0
W = int(round((X_MAX - X_MIN) / RES)) + 1
H = int(round((Y_MAX - Y_MIN) / RES)) + 1

walls = [
    (0.0, 2.0, 5.5, 0.1), (-1.65, -2.0, 2.2, 0.1), (1.65, -2.0, 2.2, 0.1),
    (-2.7, 0.0, 0.1, 4.0), (2.7, 0.0, 0.1, 4.0),
    (-0.6, -2.6, 0.1, 1.2), (0.6, -2.6, 0.1, 1.2),
    (-2.075, 0.35, 1.25, 0.1), (1.075, 0.35, 3.25, 0.1),
    (-0.9, 1.45, 0.1, 1.10), (-0.35, 1.7, 0.1, 0.6),
    (0.075, 1.45, 0.85, 0.1), (0.50, 1.175, 0.1, 0.55),
    (1.025, 0.90, 1.15, 0.1), (1.05, 1.6875, 0.1, 0.625),
    (1.60, 1.20, 0.1, 0.50), (1.875, 1.45, 0.55, 0.1),
    (2.15, 1.20, 0.1, 0.50), (-0.05, 0.625, 0.1, 0.45),
]

inflate = 0.11
grid = np.zeros((H, W), dtype=bool)
for cx, cy, sx, sy in walls:
    x0 = cx - sx / 2 - inflate
    x1 = cx + sx / 2 + inflate
    y0 = cy - sy / 2 - inflate
    y1 = cy + sy / 2 + inflate
    i0 = max(0, int((x0 - X_MIN) / RES))
    i1 = min(W - 1, int((x1 - X_MIN) / RES))
    j0 = max(0, int((y0 - Y_MIN) / RES))
    j1 = min(H - 1, int((y1 - Y_MIN) / RES))
    grid[j0:j1 + 1, i0:i1 + 1] = True


def to_idx(x, y):
    return int(round((x - X_MIN) / RES)), int(round((y - Y_MIN) / RES))


def bfs(start, goal):
    si, sj = to_idx(*start)
    gi, gj = to_idx(*goal)
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
            return [(X_MIN + i * RES, Y_MIN + j * RES) for i, j in path]
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)):
            ni, nj = i + di, j + dj
            if 0 <= ni < W and 0 <= nj < H and not seen[nj, ni] and not grid[nj, ni]:
                seen[nj, ni] = True
                prev[nj, ni] = (i, j)
                q.append((ni, nj))
    return None


G = (0.15, 1.70)
C = (-1.80, 1.20)
path = bfs(G, C)
if path is None:
    raise SystemExit("no path")

# simplify: keep points every 8 cm, then corners and 30 cm spacing
step = 8
pts = path[::step]
if pts[-1] != path[-1]:
    pts.append(path[-1])

wp = [pts[0]]
last_dir = None
for k in range(1, len(pts)):
    dx = pts[k][0] - wp[-1][0]
    dy = pts[k][1] - wp[-1][1]
    d = math.hypot(dx, dy)
    if last_dir is not None:
        ang = abs(math.atan2(math.sin(math.atan2(dy, dx) - last_dir),
                             math.cos(math.atan2(dy, dx) - last_dir)))
    else:
        ang = 0.0
    if d >= 0.50 or ang > math.radians(35):
        wp.append(pts[k])
        last_dir = math.atan2(dy, dx)

# drop waypoints too close to the previous one (no stops inside 45cm passages)
filt = [wp[0]]
for p in wp[1:]:
    if math.hypot(p[0] - filt[-1][0], p[1] - filt[-1][1]) >= 0.35:
        filt.append(p)
if math.hypot(C[0] - filt[-1][0], C[1] - filt[-1][1]) > 0.02:
    filt.append(C)
wp = filt

# goal yaw at each waypoint = heading of the next segment
out = []
for k, (x, y) in enumerate(wp):
    if k + 1 < len(wp):
        yaw = math.atan2(wp[k + 1][1] - y, wp[k + 1][0] - x)
    else:
        yaw = out[-1]["yaw"] if out else 0.0
    out.append({"x": round(x, 3), "y": round(y, 3), "yaw": round(yaw, 3)})

json.dump(out, open("C:/Users/ZRT/Desktop/小车/tune/route.json", "w"), indent=1)
print("waypoints:", len(out))
for p in out:
    print("  (%+.2f,%+.2f) yaw=%+6.1f" % (p["x"], p["y"], math.degrees(p["yaw"])))
