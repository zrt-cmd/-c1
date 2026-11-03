#!/usr/bin/env python3
"""Numbered corner map for the E region."""
from PIL import Image, ImageDraw

walls = [
    ("u_left", 1.60, 1.20, 0.1, 0.50),
    ("u_top", 1.875, 1.45, 0.55, 0.10),
    ("u_right", 2.15, 1.20, 0.1, 0.50),
    ("step_lower", 1.10, 0.95, 1.00, 0.10),
    ("east_wall", 2.70, 0.0, 0.1, 4.0),
    ("upper_right", 1.20, 1.6875, 0.1, 0.625),
    ("step_v_right", 0.65, 1.175, 0.1, 0.55),
    ("step_horizontal", 0.15, 1.40, 1.00, 0.10),
]

X_MIN, X_MAX = 0.40, 2.90
Y_MIN, Y_MAX = 0.70, 1.90
SCALE = 400
W = int((X_MAX - X_MIN) * SCALE)
H = int((Y_MAX - Y_MIN) * SCALE)


def to_px(x, y):
    return (x - X_MIN) * SCALE, (Y_MAX - y) * SCALE


img = Image.new("RGB", (W, H), (247, 247, 247))
d = ImageDraw.Draw(img)

# 0.1m grid
x = X_MIN
while x <= X_MAX + 1e-6:
    px, _ = to_px(x, 0)
    d.line([(px, 0), (px, H)], fill=(220, 220, 220), width=1)
    x = round(x + 0.1, 2)
y = Y_MIN
while y <= Y_MAX + 1e-6:
    _, py = to_px(0, y)
    d.line([(0, py), (W, py)], fill=(220, 220, 220), width=1)
    y = round(y + 0.1, 2)

for name, cx, cy, sx, sy in walls:
    p0 = to_px(cx - sx / 2, cy + sy / 2)
    p1 = to_px(cx + sx / 2, cy - sy / 2)
    d.rectangle([p0[0], p0[1], p1[0], p1[1]], fill=(65, 65, 75))
    d.text((p0[0] + 2, p0[1] + 2), name, fill=(255, 255, 255))

# car footprint in right passage
cx, cy = 2.42, 0.95
half_w, half_l = 0.135, 0.18
p0 = to_px(cx - half_w, cy + half_l)
p1 = to_px(cx + half_w, cy - half_l)
d.rectangle([p0[0], p0[1], p1[0], p1[1]], outline=(0, 110, 230), width=3)
d.text((p0[0], p1[1] + 2), "car", fill=(0, 90, 200))

corners = [
    ("1", 2.20, 0.95, "U right bottom (outer)"),
    ("2", 1.55, 0.95, "U left bottom (outer)"),
    ("3", 2.20, 1.45, "U right top"),
    ("4", 1.60, 1.45, "U left top"),
    ("5", 1.20, 1.375, "upper vertical bottom"),
    ("6", 0.65, 1.40, "step top-right"),
    ("7", 0.65, 0.90, "step bottom-right"),
    ("8", 1.60, 0.95, "U left inner"),
]

for key, x, y, desc in corners:
    px, py = to_px(x, y)
    d.ellipse([px - 13, py - 13, px + 13, py + 13], outline=(220, 0, 0), width=3)
    d.text((px - 5, py - 8), key, fill=(220, 0, 0))
    # label offset to avoid overlap
    d.text((px + 16, py - 6), desc, fill=(150, 0, 0))

d.text((8, 8), "E region corners (grid 0.1 m) - which number?", fill=(0, 0, 0))
img.save("/mnt/c/Users/ZRT/Desktop/小车/e_corners.png")
print("saved")
