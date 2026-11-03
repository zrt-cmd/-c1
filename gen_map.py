#!/usr/bin/env python3
"""Generate a Nav2 occupancy map (.pgm + .yaml) from math_room.sdf walls."""
import numpy as np
from PIL import Image

RES = 0.05
X_MIN, X_MAX = -2.7, 2.7
Y_MIN, Y_MAX = -2.0, 2.0
W = int(round((X_MAX - X_MIN) / RES))
H = int(round((Y_MAX - Y_MIN) / RES))

walls = [
    ("wall_north", 0.0, 2.0, 5.5, 0.1),
    ("wall_south_left", -1.65, -2.0, 2.2, 0.1),
    ("wall_south_right", 1.65, -2.0, 2.2, 0.1),
    ("wall_west", -2.7, 0.0, 0.1, 4.0),
    ("wall_east", 2.7, 0.0, 0.1, 4.0),
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

img = np.full((H, W), 254, dtype=np.uint8)  # free
xs = X_MIN + (np.arange(W) + 0.5) * RES
# image row 0 is the top = Y_MAX
ys = Y_MAX - (np.arange(H) + 0.5) * RES

for name, cx, cy, sx, sy in walls:
    mx = np.abs(xs - cx) <= sx / 2.0
    my = np.abs(ys - cy) <= sy / 2.0
    if mx.any() and my.any():
        img[np.ix_(my, mx)] = 0  # occupied

out_pgm = "/mnt/c/Users/ZRT/Desktop/小车/ucar-3.0/src/xflz_eai_car1/slam/car1_slam/maps/math_room.pgm"
Image.fromarray(img, mode="L").save(out_pgm)

with open(out_pgm.replace(".pgm", ".yaml"), "w") as f:
    f.write("image: math_room.pgm\n")
    f.write("mode: trinary\n")
    f.write(f"resolution: {RES}\n")
    f.write(f"origin: [{X_MIN}, {Y_MIN}, 0.0]\n")
    f.write("negate: 0\n")
    f.write("occupied_thresh: 0.65\n")
    f.write("free_thresh: 0.25\n")

print(f"saved {out_pgm} ({W}x{H})")
