#!/usr/bin/env python3
"""Set wheel contact params to gentler values."""
import re

p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(p).read()
s = re.sub(r"\s*<kp>[^<]*</kp>", "", s)
s = re.sub(r"\s*<kd>[^<]*</kd>", "", s)
s = s.replace("<mu1>2.5</mu1>", "<mu1>1.2</mu1>")
s = s.replace("<mu2>2.5</mu2>", "<mu2>1.2</mu2>")
open(p, "w").write(s)
print("updated")
