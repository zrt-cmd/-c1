#!/usr/bin/env python3
"""Mecanum via sphere wheel collisions (the way gz-sim's own example does it)."""
import re

URDF = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"
XACRO = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"

ALL = ["wheel_lf_link", "wheel_lb_link", "wheel_rf_link", "wheel_rb_link"]
R = 0.0488

s = open(URDF).read()
for link in ALL:
    i = s.find('name="%s"' % link)
    c = s.find("<collision>", i)
    e = s.find("</collision>", c)
    block = """<collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <sphere radius="%s" />
      </geometry>
    """ % R
    s = s[:c] + block + s[e:]
open(URDF, "w").write(s)

x = open(XACRO).read()
for plug in ("gz-sim-diff-drive-system", "gz-sim-velocity-control-system"):
    start = x.find('<plugin filename="%s"' % plug)
    if start >= 0:
        end = x.find("</plugin>", start) + len("</plugin>")
        if x.find('<plugin filename="gz-sim-odometry-publisher-system"', end) >= 0:
            o = x.find('<plugin filename="gz-sim-odometry-publisher-system"', end)
            end = x.find("</plugin>", o) + len("</plugin>")
        new = """<plugin filename="gz-sim-mecanum-drive-system" name="gz::sim::systems::MecanumDrive">
            <front_left_joint>wheel_lf_joint</front_left_joint>
            <front_right_joint>wheel_rf_joint</front_right_joint>
            <back_left_joint>wheel_lb_joint</back_left_joint>
            <back_right_joint>wheel_rb_joint</back_right_joint>
            <wheel_separation>0.216</wheel_separation>
            <wheelbase>0.1946</wheelbase>
            <wheel_radius>0.04675</wheel_radius>
            <topic>/cmd_vel</topic>
            <odom_topic>/odom</odom_topic>
            <tf_topic>/tf</tf_topic>
            <frame_id>odom</frame_id>
            <child_frame_id>base_footprint</child_frame_id>
            <odom_publish_frequency>50</odom_publish_frequency>
        </plugin>"""
        x = x[:start] + new + x[end:]
        open(XACRO, "w").write(x)
        print("swapped to MecanumDrive with sphere wheels")
        break
else:
    print("no drive plugin to swap")
