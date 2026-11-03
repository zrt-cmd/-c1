#!/usr/bin/env python3
"""Annotated top-down layout of math_room.sdf with clear passage widths."""
from PIL import Image, ImageDraw, ImageFont

# name, cx, cy, sx, sy  (current math_room.sdf)
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
    ("u_left", 1.60, 1.20, 0.1, 0.50),
    ("u_top", 1.875, 1.45, 0.55, 0.1),
    ("u_right", 2.15, 1.20, 0.1, 0.50),
    ("short_vertical", 0.15, 0.625, 0.1, 0.45),
]

X_MIN, X_MAX = -3.25, 3.25
Y_MIN, Y_MAX = -3.55, 2.55
SCALE = 175
W = int((X_MAX - X_MIN) * SCALE)
H = int((Y_MAX - Y_MIN) * SCALE)

try:
    FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 13)
    FONT_SMALL = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
    FONT_BIG = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16)
except Exception:
    FONT = FONT_SMALL = FONT_BIG = ImageFont.load_default()


def to_px(x, y):
    return (x - X_MIN) * SCALE, (Y_MAX - y) * SCALE


img = Image.new("RGB", (W, H), (250, 250, 250))
d = ImageDraw.Draw(img)


def text_bg(xy, s, font=FONT, fg=(0, 0, 0), bg=(255, 255, 255)):
    x, y = xy
    bbox = d.textbbox((x, y), s, font=font)
    d.rectangle([bbox[0] - 2, bbox[1] - 1, bbox[2] + 2, bbox[3] + 1], fill=bg)
    d.text((x, y), s, font=font, fill=fg)


# grid 0.5m
x = X_MIN
while x <= X_MAX + 1e-6:
    px, _ = to_px(x, 0)
    major = abs(x - round(x)) < 1e-6
    d.line([(px, 0), (px, H)], fill=(185, 185, 185) if major else (225, 225, 225), width=1)
    if major:
        text_bg((px + 2, H - 17), f"{x:.0f}", FONT_SMALL, (70, 70, 70), (250, 250, 250))
    x = round(x + 0.5, 2)

y = Y_MIN
while y <= Y_MAX + 1e-6:
    _, py = to_px(0, y)
    major = abs(y - round(y)) < 1e-6
    d.line([(0, py), (W, py)], fill=(185, 185, 185) if major else (225, 225, 225), width=1)
    if major:
        text_bg((2, py - 13), f"{y:.0f}", FONT_SMALL, (70, 70, 70), (250, 250, 250))
    y = round(y + 0.5, 2)

# walls
for name, cx, cy, sx, sy in walls:
    p0 = to_px(cx - sx / 2, cy + sy / 2)
    p1 = to_px(cx + sx / 2, cy - sy / 2)
    d.rectangle([p0[0], p0[1], p1[0], p1[1]], fill=(72, 72, 84))
    if sx > 0.45 and sy < 0.2:
        text_bg(((p0[0] + p1[0]) / 2 - 20, (p0[1] + p1[1]) / 2 - 6), name[:16], FONT_SMALL, (255, 255, 255), (72, 72, 84))
    elif sy > 0.45 and sx < 0.2:
        text_bg((p0[0] + 2, (p0[1] + p1[1]) / 2 - 6), name[:16], FONT_SMALL, (255, 255, 255), (72, 72, 84))


def dim_h(x1, x2, y, label, color=(0, 130, 0), text_dy=-16):
    p1 = to_px(x1, y)
    p2 = to_px(x2, y)
    d.line([p1, p2], fill=color, width=2)
    for p in (p1, p2):
        d.line([(p[0], p[1] - 6), (p[0], p[1] + 6)], fill=color, width=2)
    mx, my = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
    text_bg((mx - 16, my + text_dy), label, FONT, color, (255, 255, 255))


def dim_v(y1, y2, x, label, color=(0, 130, 0), text_dx=6):
    p1 = to_px(x, y1)
    p2 = to_px(x, y2)
    d.line([p1, p2], fill=color, width=2)
    for p in (p1, p2):
        d.line([(p[0] - 6, p[1]), (p[0] + 6, p[1])], fill=color, width=2)
    mx, my = (p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2
    text_bg((mx + text_dx, my - 6), label, FONT, color, (255, 255, 255))


# Dimensions
dim_h(-0.55, 0.55, -2.9, "1.10 m", (0, 120, 220), -18)          # entrance
dim_h(-1.45, -0.55, 0.40, "0.90 m", (0, 120, 220), -18)         # divider door
dim_v(0.45, 0.90, -0.90, "0.45 m", (220, 80, 0), 8)             # upper-left entrance
dim_h(-0.85, -0.40, 1.72, "0.45 m", (220, 80, 0), -18)          # left dead-end slot
dim_v(1.45, 1.95, 0.95, "0.50 m", (0, 120, 220), 8)             # G pocket height
dim_h(0.70, 1.15, 1.40, "0.45 m", (220, 80, 0), -18)            # G east slot
dim_h(0.65, 1.15, 1.55, "0.50 m", (0, 120, 220), -18)           # G southeast opening
dim_v(0.45, 0.90, 1.30, "0.45 m", (220, 80, 0), 8)              # lower corridor
dim_h(1.65, 2.10, 1.20, "0.45 m", (220, 80, 0), -18)            # U interior width
dim_v(0.95, 1.40, 2.00, "0.45 m", (220, 80, 0), 8)              # U interior height
dim_h(2.20, 2.65, 1.10, "0.45 m", (220, 80, 0), -18)            # U right passage
dim_v(-3.20, -2.00, 0.85, "1.20 m", (0, 120, 220), 6)           # entrance length

# room dimensions
dim_h(-2.70, 2.70, 2.30, "5.40 m room width", (90, 90, 90), -18)
dim_v(-2.00, 2.00, -2.95, "4.00 m room depth", (90, 90, 90), 6)

# region labels
regions = {
    "A entrance": (0.0, -2.95),
    "B open": (0.0, -1.0),
    "C upper-left": (-1.8, 1.2),
    "D step": (-0.15, 0.95),
    "E U-area": (1.95, 0.55),
    "F top-right": (1.8, 1.85),
    "G start": (0.15, 1.75),
}
for label, (rx, ry) in regions.items():
    px, py = to_px(rx, ry)
    text_bg((px - 8, py - 8), label, FONT_SMALL, (200, 0, 0), (255, 255, 255))

# car footprint + heading arrow at G
cx, cy = 0.15, 1.70
p0 = to_px(cx - 0.18, cy + 0.135)
p1 = to_px(cx + 0.18, cy - 0.135)
d.rectangle([p0[0], p0[1], p1[0], p1[1]], outline=(0, 80, 220), width=2)
ax0 = to_px(cx + 0.18, cy)
ax1 = to_px(cx + 0.42, cy)
d.line([ax0, ax1], fill=(0, 80, 220), width=2)
d.polygon([ax1, (ax1[0] - 7, ax1[1] - 4), (ax1[0] - 7, ax1[1] + 4)], fill=(0, 80, 220))
text_bg((ax1[0] - 10, ax1[1] + 4), "car +X", FONT_SMALL, (0, 80, 220), (255, 255, 255))

text_bg((8, 8), "math_room top view - clear widths (m), grid 0.5 m", FONT_BIG, (0, 0, 0), (250, 250, 250))
text_bg((8, 32), "blue = main passages, orange = complex 0.45 m passages", FONT_SMALL, (60, 60, 60), (250, 250, 250))

img.save("/mnt/c/Users/ZRT/Desktop/小车/map_annotated.png")
print("saved")
