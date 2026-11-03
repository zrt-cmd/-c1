#!/usr/bin/env python3
"""Restore the working diff-drive sim: cylinder wheels, friction 1.2, DiffDrive."""
import os
import subprocess

here = os.path.dirname(os.path.abspath(__file__))
for script in ("fix_wheel_collision.py", "revert_diffdrive.py"):
    subprocess.run(["python3", os.path.join(here, script)])

p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(p).read()
s = s.replace("<mu1>0.0</mu1>", "<mu1>1.2</mu1>")
s = s.replace("<mu2>0.0</mu2>", "<mu2>1.2</mu2>")
open(p, "w").write(s)
print("friction restored to 1.2")
