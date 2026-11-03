#!/usr/bin/env python3
"""Make all four wheel joints use axis +Y (gz DiffDrive assumes one convention)."""
import re

p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"
s = open(p).read()

for name in ["wheel_rf_joint", "wheel_rb_joint"]:
    i = s.find('name="%s"' % name)
    j = s.find("</joint>", i)
    block = s[i:j]
    new = re.sub(r'(<axis\s+xyz=")0\s+-1\s+0(")', r"\g<1>0 1 0\g<2>", block)
    if new == block:
        print("no change", name)
    else:
        s = s[:i] + new + s[j:]
        print("fixed", name)

open(p, "w").write(s)
