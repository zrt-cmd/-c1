#!/usr/bin/env python3
"""Restore the mecanum wheel mesh as the collision geometry (rollers needed)."""
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"
s = open(p).read()

for name in ["wheel_lf_link", "wheel_lb_link", "wheel_rf_link", "wheel_rb_link"]:
    i = s.find('name="%s"' % name)
    if i < 0:
        print("MISSING", name)
        continue
    c = s.find("<collision>", i)
    e = s.find("</collision>", c)
    if "cylinder" not in s[c:e]:
        print("already mesh", name)
        continue
    block = """<collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://car1_description/meshes/%s.STL" />
      </geometry>
    """ % name
    s = s[:c] + block + s[e:]
    print("restored", name)

open(p, "w").write(s)
