#!/usr/bin/env python3
"""Gravity back on, wheel friction zero (kinematic holonomic base)."""
WORLD = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/worlds/math_room.sdf"
XACRO = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"

w = open(WORLD).read()
w = w.replace("\n    <gravity>0 0 0</gravity>", "")
open(WORLD, "w").write(w)
print("gravity restored:", "<gravity>" not in w)

x = open(XACRO).read()
x = x.replace("<mu1>1.2</mu1>", "<mu1>0.0</mu1>")
x = x.replace("<mu2>1.2</mu2>", "<mu2>0.0</mu2>")
open(XACRO, "w").write(x)
print("wheel friction zeroed")
