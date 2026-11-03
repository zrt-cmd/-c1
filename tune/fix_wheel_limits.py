#!/usr/bin/env python3
"""Add effort/velocity limits to the four wheel joints."""
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"
s = open(p).read()

names = ["wheel_lf_joint", "wheel_lb_joint", "wheel_rf_joint", "wheel_rb_joint"]
changed = 0
for name in names:
    key = 'name="%s"' % name
    i = s.find(key)
    if i < 0:
        print("MISSING", name)
        continue
    j = s.find("</joint>", i)
    block = s[i:j]
    if "effort=" in block:
        print("already", name)
        continue
    s = s[:j] + '    <limit effort="20" velocity="30"/>\n  ' + s[j:]
    changed += 1
    print("fixed", name)

open(p, "w").write(s)
print("changed", changed)
