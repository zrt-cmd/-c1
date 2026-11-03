#!/usr/bin/env python3
"""Zoomed top-down view of region E with the car footprint and corner labels."""
from PIL import Image, ImageDraw

# walls: name, cx, cy, sx, sy
walls = [
    ("u_left", 1.60, 1.20, 0.1, 0.50),
    ("u_top", 1.875, 1.45, 0.55, 0.10),
    ("u_right", 2.15, 1.20, 0.1, 0.50),
    ("step_lower", 1.10, 0.95, 1.00, 0.10),
    ("east_wall", 2.70, 0.0, 0.1, 4.0),
    ("upper_right", 1.20, 1.6875, 0.1, 0.625),
    ("divider_right", 1.075, 0.35, 3.25, 0.1),
]

X_MIN, X_MAX = 1.35, 2.85
Y_MIN, Y_MAX = 0.30, 1.90
SCALE = 420
W = int((X_MAX - X_MIN) * SCALE)
H = int((Y_MAX - Y_MIN) * SCALE)


def to_px(x, y):
    return (x - X_MIN) * SCALE, (Y_MAX - y) * SCALE


img = Image.new("RGB", (W, H), (245, 245, 245))
d = ImageDraw.Draw(img)

# grid 0.1 m
x = X_MIN
while x <= X_MAX + 1e-6:
    px, _ = to_px(x, 0)
    d.line([(px, 0), (px, H)], fill=(215, 215, 215), width=1)
    x = round(x + 0.1, 2)
y = Y_MIN
while y <= Y_MAX + 1e-6:
    _, py = to_px(0, y)
    d.line([(0, py), (W, py)], fill=(215, 215, 215), width=1)
    y = round(y + 0.1, 2)

for name, cx, cy, sx, sy in walls:
    p0 = to_px(cx - sx / 2, cy + sy / 2)
    p1 = to_px(cx + sx / 2, cy - sy / 2)
    d.rectangle([p0[0], p0[1], p1[0], p1[1]], fill=(70, 70, 80))
    d.text((p0[0] + 2, p0[1] + 2), name, fill=(255, 255, 255))

# car footprint at (2.42, 0.60), yaw = +90 deg (facing north)
cx, cy = 2.42, 0.60
half_w, half_l = 0.135, 0.18
p0 = to_px(cx - half_w, cy + half_l)
p1 = to_px(cx + half_w, cy - half_l)
d.rectangle([p0[0], p0[1], p1[0], p1[1]], outline=(0, 120, 255), width=3)
d.text((p0[0], p0[1] - 16), "car 0.27 x 0.36", fill=(0, 90, 200))

# corner labels
corners = {
    "A": (2.20, 0.95, "U right bottom (flush)"),
    "B": (1.55, 0.95, "U left bottom (flush)"),
    "C": (1.60, 1.00, "step/U junction"),
}
for key, (x, y, desc) in corners.items():
    px, py = to_px(x, y)
    d.ellipse([px - 8, py - 8, px + 8, py + 8], outline=(220, 0, 0), width=3)
    d.text((px + 12, py - 8), f"{key}: {desc}", fill=(180, 0, 0))

# passage width arrow (right passage x=2.20..2.65)
y_arrow = 0.45
p0 = to_px(2.20, y_arrow)
p1 = to_px(2.65, y_arrow)
d.line([p0, p1], fill=(0, 150, 0), width=3)
d.text(((p0[0] + p1[0]) / 2 - 30, p0[1] + 4), "0.45 m", fill=(0, 120, 0))

d.text((8, 8), "Region E zoom (grid 0.1 m)", fill=(0, 0, 0))
img.save("/mnt/c/Users/ZRT/Desktop/小车/e_area.png")
print("saved")
