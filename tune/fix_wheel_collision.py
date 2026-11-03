#!/usr/bin/env python3
"""Replace wheel collision meshes with cylinders for stable rolling."""
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"
s = open(p).read()

wheels = ["wheel_lf_link", "wheel_lb_link", "wheel_rf_link", "wheel_rb_link"]
newblock = """<collision>
      <origin xyz="0 0 0" rpy="1.5707963267948966 0 0" />
      <geometry>
        <cylinder radius="0.0488" length="0.052" />
      </geometry>
    """

for name in wheels:
    i = s.find('name="%s"' % name)
    if i < 0:
        print("MISSING", name)
        continue
    c = s.find("<collision>", i)
    e = s.find("</collision>", c)
    if c < 0 or e < 0:
        print("no collision", name)
        continue
    if "cylinder" in s[c:e]:
        print("already", name)
        continue
    s = s[:c] + newblock + s[e:]
    print("replaced", name)

open(p, "w").write(s)
