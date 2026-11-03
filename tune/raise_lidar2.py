#!/usr/bin/env python3
"""Move laser_link above the chassis; drop the ineffective sensor <pose>."""
xacro = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
urdf = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"

s = open(xacro).read()
s = s.replace('\n            <pose>0 0 0.25 0 0 0</pose>', "")
open(xacro, "w").write(s)

u = open(urdf).read()
old = 'xyz="0.146213088049965 0 0.0874000000000005"'
new = 'xyz="0.146213088049965 0 0.3374000000000005"'
if old in u:
    u = u.replace(old, new, 1)
    open(urdf, "w").write(u)
    print("laser_joint moved to z=0.3374")
else:
    print("pattern not found")

for line in open(urdf).read().splitlines():
    if "0.3374000000000005" in line:
        print("   ", line.strip())
