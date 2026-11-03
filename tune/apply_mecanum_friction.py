#!/usr/bin/env python3
"""Anisotropic wheel friction (fdir1) + identity joint assignment, per gz's example."""
import re

XACRO = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(XACRO).read()

fdir = {
    "wheel_lf_link": "1 -1 0",
    "wheel_rf_link": "1 1 0",
    "wheel_lb_link": "1 1 0",
    "wheel_rb_link": "1 -1 0",
}
for link, vec in fdir.items():
    pat = re.compile(
        r'<gazebo reference="%s">.*?</gazebo>' % re.escape(link), re.S)
    block = ('<gazebo reference="%s">\n'
             '        <mu1>1.0</mu1>\n'
             '        <mu2>0.0</mu2>\n'
             '        <fdir1>%s</fdir1>\n'
             '    </gazebo>' % (link, vec))
    if pat.search(s):
        s = pat.sub(block, s, count=1)
    else:
        s = s.replace("<gazebo reference=\"laser_link\">",
                      block + "\n    <gazebo reference=\"laser_link\">", 1)
    print("friction", link, vec)

# identity joint assignment
start = s.find('<plugin filename="gz-sim-mecanum-drive-system"')
if start >= 0:
    end = s.find("</plugin>", start) + len("</plugin>")
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
    s = s[:start] + new + s[end:]
    print("joint assignment: identity")

open(XACRO, "w").write(s)
