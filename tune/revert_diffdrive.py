#!/usr/bin/env python3
"""Revert the sim to the working diff-drive setup (all wheel axes +Y)."""
import re

xacro = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
urdf = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"

s = open(xacro).read()
start = s.find('<plugin filename="gz-sim-mecanum-drive-system"')
if start >= 0:
    end = s.find("</plugin>", start) + len("</plugin>")
    new = """<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
            <left_joint>wheel_lf_joint</left_joint>
            <left_joint>wheel_lb_joint</left_joint>
            <right_joint>wheel_rf_joint</right_joint>
            <right_joint>wheel_rb_joint</right_joint>
            <wheel_separation>0.216</wheel_separation>
            <wheel_radius>0.04675</wheel_radius>
            <max_linear_acceleration>2.5</max_linear_acceleration>
            <max_angular_acceleration>3.2</max_angular_acceleration>
            <topic>/cmd_vel</topic>
            <odom_topic>/odom</odom_topic>
            <tf_topic>/tf</tf_topic>
            <frame_id>odom</frame_id>
            <child_frame_id>base_footprint</child_frame_id>
            <odom_publish_frequency>50</odom_publish_frequency>
        </plugin>"""
    s = s[:start] + new + s[end:]
    open(xacro, "w").write(s)
    print("reverted to DiffDrive")
else:
    print("diff drive already present")

u = open(urdf).read()
for name in ["wheel_rf_joint", "wheel_rb_joint"]:
    i = u.find('name="%s"' % name)
    j = u.find("</joint>", i)
    block = u[i:j]
    new = re.sub(r'(<axis\s+xyz=")0\s+-1\s+0(")', r"\g<1>0 1 0\g<2>", block)
    if new != block:
        u = u[:i] + new + u[j:]
        print("axis fixed", name)
open(urdf, "w").write(u)
