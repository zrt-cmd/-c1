#!/usr/bin/env python3
"""Raise the simulated lidar above the chassis (sim fidelity fix)."""
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(p).read()
old = '<sensor name="lidar" type="gpu_lidar">'
new = old + '\n            <pose>0 0 0.25 0 0 0</pose>'
if "<pose>0 0 0.25" not in s:
    s = s.replace(old, new, 1)
    open(p, "w").write(s)
    print("inserted")
else:
    print("already present")
print("\n".join(open(p).read().splitlines()[88:96]))
