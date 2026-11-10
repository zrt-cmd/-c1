#!/usr/bin/env python3
"""Draw the map with G, C and the best real progress point."""
import math

from PIL import Image, ImageDraw, ImageFont

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
    ("short_vertical", -0.05, 0.625, 0.1, 0.45),
]

X0, X1 = -3.0, 3.0
Y0, Y1 = -2.3, 2.3
SC = 190
IW, IH = int((X1 - X0) * SC), int((Y1 - Y0) * SC)


def px(x, y):
    return (x - X0) * SC, (Y1 - y) * SC


img = Image.new("RGB", (IW, IH), (250, 250, 250))
d = ImageDraw.Draw(img)

try:
    font = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 26)
    font_s = ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", 20)
except Exception:
    font = ImageFont.load_default()
    font_s = font

for name, cx, cy, sx, sy in walls:
    p0 = px(cx - sx / 2, cy + sy / 2)
    p1 = px(cx + sx / 2, cy - sy / 2)
    d.rectangle([p0[0], p0[1], p1[0], p1[1]], fill=(70, 70, 80))

G = (0.15, 1.70)
C = (-1.80, 1.20)
B = (0.72, 1.63)

# G
p = px(*G)
d.ellipse([p[0] - 9, p[1] - 9, p[0] + 9, p[1] + 9], fill=(0, 80, 220))
d.text((p[0] + 14, p[1] - 30), "G 起点", fill=(0, 80, 220), font=font)

# C
p = px(*C)
d.ellipse([p[0] - 9, p[1] - 9, p[0] + 9, p[1] + 9], fill=(200, 0, 0))
d.text((p[0] - 90, p[1] + 10), "C 目标", fill=(200, 0, 0), font=font)

# travelled segment G -> B
pg, pb = px(*G), px(*B)
d.line([pg, pb], fill=(0, 150, 0), width=5)

# best point
p = px(*B)
d.ellipse([p[0] - 11, p[1] - 11, p[0] + 11, p[1] + 11],
          outline=(255, 120, 0), width=5)
d.text((p[0] + 16, p[1] + 4), "最远点 (0.72, 1.63)  约0.6m", fill=(255, 120, 0), font=font_s)

# highlight the 45cm gap
gx0, gx1 = 0.55, 1.00
gy0, gy1 = 1.375, 1.45
p0 = px(gx0, gy1)
p1 = px(gx1, gy0)
d.rectangle([p0[0], p0[1], p1[0], p1[1]], outline=(220, 0, 160), width=3)
d.text((px(0.78, 1.30)[0] - 40, px(0.78, 1.30)[1] + 8), "45cm 窄口", fill=(220, 0, 160), font=font_s)

# distance label
d.text((40, 30), "当前位置：第一个 45cm 窄口前（G->C 全长约4m）", fill=(40, 40, 40), font=font_s)

img.save("C:/Users/ZRT/Desktop/小车/best_run.png")
print("saved best_run.png")
print("G->B distance %.2f m, B->C straight %.2f m" % (
    math.hypot(B[0] - G[0], B[1] - G[1]),
    math.hypot(C[0] - B[0], C[1] - B[1])))
