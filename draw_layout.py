#!/usr/bin/env python3
"""Draw a top-down annotated layout of the simplified math_room world."""
from PIL import Image, ImageDraw

# name, center_x, center_y, size_x, size_y
walls = [
    ("wall_north", 0.0, 2.0, 5.5, 0.1),
    ("wall_south_left", -1.65, -2.0, 2.2, 0.1),
    ("wall_south_right", 1.65, -2.0, 2.2, 0.1),
    ("wall_west", -2.7, 0.0, 0.1, 4.0),
    ("wall_east", 2.7, 0.0, 0.1, 4.0),
    ("entrance_left", -0.6, -2.6, 0.1, 1.2),
    ("entrance_right", 0.6, -2.6, 0.1, 1.2),
    ("divider_left", -2.075, 0.4, 1.25, 0.1),
    ("divider_right", 1.075, 0.4, 3.25, 0.1),
    ("upper_left_vertical", -0.9, 1.45, 0.1, 1.10),
    ("step_vertical_left", -0.35, 1.7, 0.1, 0.6),
    ("step_horizontal", 0.15, 1.4, 1.0, 0.1),
    ("step_vertical_right", 0.65, 1.175, 0.1, 0.55),
    ("step_horizontal_lower", 1.10, 0.95, 1.00, 0.1),
    ("upper_vertical_right", 1.20, 1.6875, 0.1, 0.625),
    ("u_left", 1.60, 1.20, 0.1, 0.5),
    ("u_top", 1.875, 1.45, 0.55, 0.1),
    ("u_right", 2.15, 1.20, 0.1, 0.5),
    ("short_vertical", -0.05, 0.625, 0.1, 0.45),
]

X_MIN, X_MAX = -3.2, 3.2
Y_MIN, Y_MAX = -3.6, 2.6
SCALE = 150  # px per meter
W = int((X_MAX - X_MIN) * SCALE)
H = int((Y_MAX - Y_MIN) * SCALE)


def to_px(x, y):
    px = (x - X_MIN) * SCALE
    py = (Y_MAX - y) * SCALE
    return px, py


img = Image.new("RGB", (W, H), (245, 245, 245))
d = ImageDraw.Draw(img)

# grid every 0.5 m
x = X_MIN
while x <= X_MAX + 1e-6:
    px, _ = to_px(x, 0)
    color = (200, 200, 200) if abs(x - round(x)) > 1e-6 else (160, 160, 160)
    d.line([(px, 0), (px, H)], fill=color, width=1)
    if abs(x - round(x)) < 1e-6:
        d.text((px + 2, H - 14), f"{x:.0f}", fill=(60, 60, 60))
    x += 0.5

y = Y_MIN
while y <= Y_MAX + 1e-6:
    _, py = to_px(0, y)
    color = (200, 200, 200) if abs(y - round(y)) > 1e-6 else (160, 160, 160)
    d.line([(0, py), (W, py)], fill=color, width=1)
    if abs(y - round(y)) < 1e-6:
        d.text((2, py - 12), f"{y:.0f}", fill=(60, 60, 60))
    y += 0.5

# walls
for name, cx, cy, sx, sy in walls:
    x0, y0 = to_px(cx - sx / 2, cy + sy / 2)
    x1, y1 = to_px(cx + sx / 2, cy - sy / 2)
    d.rectangle([x0, y0, x1, y1], fill=(70, 70, 80))
    # label small walls
    if sx < 1.3 and sy < 1.3:
        d.text((x0, y0 - 12), name[:14], fill=(40, 40, 120))

# current start
sx, sy = to_px(0.15, 1.7)
d.ellipse([sx - 7, sy - 7, sx + 7, sy + 7], fill=(220, 30, 30))
d.text((sx + 10, sy - 6), "START (G)", fill=(220, 30, 30))

# region labels
regions = {
    "A": (0.0, -2.9, "entrance corridor"),
    "B": (0.0, -1.0, "main open area"),
    "C": (-1.8, 1.2, "upper-left room"),
    "D": (-0.15, 0.9, "step / center"),
    "E": (1.95, 0.55, "right U area"),
    "F": (1.8, 1.9, "top-right corridor"),
    "G": (0.15, 1.8, "top-middle"),
}
for key, (rx, ry, desc) in regions.items():
    px, py = to_px(rx, ry)
    d.rectangle([px - 12, py - 12, px + 12, py + 12], outline=(200, 0, 0), width=2)
    d.text((px + 16, py - 20), f"{key}", fill=(200, 0, 0))
    d.text((px + 16, py - 6), desc, fill=(120, 0, 0))

d.text((10, 10), "math_room top view (meters), grid 0.5m", fill=(0, 0, 0))
img.save("/mnt/c/Users/ZRT/Desktop/小车/math_room_layout.png")
print("saved")
