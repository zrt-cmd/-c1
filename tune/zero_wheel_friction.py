#!/usr/bin/env python3
"""Zero wheel friction: the kinematic VelocityControl base must not fight the ground."""
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(p).read()
s = s.replace("<mu1>1.2</mu1>", "<mu1>0.0</mu1>")
s = s.replace("<mu2>1.2</mu2>", "<mu2>0.0</mu2>")
open(p, "w").write(s)
print("wheel friction set to 0.0")
