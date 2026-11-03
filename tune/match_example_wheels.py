#!/usr/bin/env python3
"""Match the gz example's wheel mass and radius to test lateral drive."""
import re

URDF = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"
s = open(URDF).read()

for link in ["wheel_lf_link", "wheel_lb_link", "wheel_rf_link", "wheel_rb_link"]:
    i = s.find('name="%s"' % link)
    j = s.find("</link>", i)
    body = s[i:j]
    body = re.sub(r'value="[0-9.eE+-]+"', 'value="1.0"', body, count=1)
    body = body.replace('<sphere radius="0.0488" />', '<sphere radius="0.04675" />')
    s = s[:i] + body + s[j:]

open(URDF, "w").write(s)
print("wheel mass 1.0 kg, sphere radius 0.04675")
