#!/usr/bin/env python3
"""Print the axis line of each wheel joint."""
import re

p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"
s = open(p).read()
for name in ["wheel_lf_joint", "wheel_lb_joint", "wheel_rf_joint", "wheel_rb_joint"]:
    i = s.find('name="%s"' % name)
    j = s.find("</joint>", i)
    block = s[i:j]
    m = re.search(r"<axis\s+xyz=\"([^\"]+)\"", block, re.S)
    print(name, "axis =", m.group(1) if m else "??")
