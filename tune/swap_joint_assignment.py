#!/usr/bin/env python3
"""Swap left/right joint assignments inside the MecanumDrive plugin."""
import sys

MODE = sys.argv[1] if len(sys.argv) > 1 else "lr"
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(p).read()

start = s.find('<plugin filename="gz-sim-mecanum-drive-system"')
if start < 0:
    print("mecanum plugin not found")
    raise SystemExit(1)
end = s.find("</plugin>", start) + len("</plugin>")

if MODE == "lr":
    new = """<plugin filename="gz-sim-mecanum-drive-system" name="gz::sim::systems::MecanumDrive">
            <front_left_joint>wheel_rf_joint</front_left_joint>
            <front_right_joint>wheel_lf_joint</front_right_joint>
            <back_left_joint>wheel_rb_joint</back_left_joint>
            <back_right_joint>wheel_lb_joint</back_right_joint>
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
elif MODE == "fb":
    new = """<plugin filename="gz-sim-mecanum-drive-system" name="gz::sim::systems::MecanumDrive">
            <front_left_joint>wheel_lb_joint</front_left_joint>
            <front_right_joint>wheel_rb_joint</front_right_joint>
            <back_left_joint>wheel_lf_joint</back_left_joint>
            <back_right_joint>wheel_rf_joint</back_right_joint>
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
else:
    print("unknown mode")
    raise SystemExit(1)

s = s[:start] + new + s[end:]
open(p, "w").write(s)
print("assignment", MODE, "applied")
