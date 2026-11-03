#!/usr/bin/env python3
"""Zoom on the G->C bottleneck: 0.30 m gap vs 0.27 m car."""
from PIL import Image, ImageDraw, ImageFont

walls = [
    ("step_vertical_right", 0.65, 1.175, 0.1, 0.55),
    ("step_horizontal", 0.15, 1.40, 1.00, 0.10),
    ("step_horizontal_lower", 1.10, 0.95, 1.00, 0.10),
    ("upper_vertical_right", 1.20, 1.6875, 0.1, 0.625),
    ("u_left", 1.60, 1.20, 0.1, 0.50),
    ("u_top", 1.875, 1.45, 0.55, 0.10),
    ("u_right", 2.15, 1.20, 0.1, 0.50),
    ("wall_north", 0.0, 2.0, 5.5, 0.1),
]

X_MIN, X_MAX = 0.75, 2.40
Y_MIN, Y_MAX = 0.75, 2.05
SCALE = 620
W = int((X_MAX - X_MIN) * SCALE)
H = int((Y_MAX - Y_MIN) * SCALE)

try:
    F = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
    FS = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 13)
    FB = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
except Exception:
    F = FS = FB = ImageFont.load_default()


def px(x, y):
    return (x - X_MIN) * SCALE, (Y_MAX - y) * SCALE


img = Image.new("RGB", (W, H), (250, 250, 250))
d = ImageDraw.Draw(img)


def label(xy, s, font=F, fg=(0, 0, 0), bg=(255, 255, 255)):
    bbox = d.textbbox(xy, s, font=font)
    d.rectangle([bbox[0] - 2, bbox[1] - 1, bbox[2] + 2, bbox[3] + 1], fill=bg)
    d.text(xy, s, font=font, fill=fg)


# grid 0.1 m
x = X_MIN
while x <= X_MAX + 1e-6:
    p, _ = px(x, 0)
    d.line([(p, 0), (p, H)], fill=(225, 225, 225), width=1)
    x = round(x + 0.1, 2)
y = Y_MIN
while y <= Y_MAX + 1e-6:
    _, p = px(0, y)
    d.line([(0, p), (W, p)], fill=(225, 225, 225), width=1)
    y = round(y + 0.1, 2)

for name, cx, cy, sx, sy in walls:
    p0 = px(cx - sx / 2, cy + sy / 2)
    p1 = px(cx + sx / 2, cy - sy / 2)
    d.rectangle([p0[0], p0[1], p1[0], p1[1]], fill=(65, 65, 78))
    label((p0[0] + 3, (p0[1] + p1[1]) / 2 - 7), name, FS, (255, 255, 255), (65, 65, 78))

# car footprint at (1.40, 1.40), facing +Y (north)
cx, cy = 1.40, 1.40
half_w, half_l = 0.135, 0.18
p0 = px(cx - half_w, cy + half_l)
p1 = px(cx + half_w, cy - half_l)
d.rectangle([p0[0], p0[1], p1[0], p1[1]], outline=(0, 90, 230), width=4)
label((p0[0] + 4, p1[1] + 4), "car 0.27 x 0.36", F, (0, 90, 230))

# gap dimension at y=1.40 (both walls present)
y_gap = 1.40
a = px(1.25, y_gap)
b = px(1.55, y_gap)
d.line([a, b], fill=(220, 0, 0), width=4)
for p in (a, b):
    d.line([(p[0], p[1] - 9), (p[0], p[1] + 9)], fill=(220, 0, 0), width=4)
label(((a[0] + b[0]) / 2 - 55, a[1] - 34), "0.30 m gap", FB, (220, 0, 0))

# car width dimension below the car
y_car = 1.20
c = px(1.265, y_car)
e = px(1.535, y_car)
d.line([c, e], fill=(0, 90, 230), width=4)
for p in (c, e):
    d.line([(p[0], p[1] - 8), (p[0], p[1] + 8)], fill=(0, 90, 230), width=4)
label(((c[0] + e[0]) / 2 - 36, c[1] + 10), "0.27 m car", F, (0, 90, 230))

# clearance labels
label((px(1.25, 1.55)[0] + 6, px(1.25, 1.55)[1] - 20), "1.5 cm", FS, (0, 130, 0))
label((px(1.55, 1.55)[0] - 62, px(1.55, 1.55)[1] - 20), "1.5 cm", FS, (0, 130, 0))

# path arrow: north through the gap
start = px(1.40, 1.12)
end = px(1.40, 1.72)
d.line([start, end], fill=(0, 150, 0), width=3)
d.polygon([end, (end[0] - 9, end[1] + 14), (end[0] + 9, end[1] + 14)], fill=(0, 150, 0))
label((end[0] + 12, end[1] + 4), "G -> C path", FS, (0, 120, 0))

label((10, 10), "G->C bottleneck: 0.30 m gap, car 0.27 m -> 1.5 cm per side", FB, (0, 0, 0))
img.save("/mnt/c/Users/ZRT/Desktop/小车/bottleneck.png")
print("saved")
