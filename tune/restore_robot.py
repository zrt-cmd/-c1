#!/usr/bin/env python3
"""Restore the full car model with the working differential drive."""
import re

URDF = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"
XACRO = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"

LEFT = ["wheel_lf_link", "wheel_lb_link"]
RIGHT = ["wheel_rf_link", "wheel_rb_link"]
ALL = LEFT + RIGHT

s = open(URDF).read()

# reset wheel mesh assignments and flips
for link in ALL:
    i = s.find('name="%s"' % link)
    j = s.find("</link>", i)
    body = s[i:j]
    body = re.sub(r"meshes/wheel_(lf|lb|rf|rb)_link\.STL",
                  "meshes/%s.STL" % link, body)
    body = body.replace('rpy="0 3.14159265 0"', 'rpy="0 0 0"')
    s = s[:i] + body + s[j:]

# wheel joints back to +Y
for name in ["wheel_lf_joint", "wheel_lb_joint", "wheel_rf_joint", "wheel_rb_joint"]:
    i = s.find('name="%s"' % name)
    j = s.find("</joint>", i)
    block = s[i:j]
    block = re.sub(r'(<axis\s+xyz=")0\s+[-+]?1\s+0(")', r"\g<1>0 1 0\g<2>", block)
    s = s[:i] + block + s[j:]

# cylinder collisions (stable rolling for diff drive)
for link in ALL:
    i = s.find('name="%s"' % link)
    c = s.find("<collision>", i)
    e = s.find("</collision>", c)
    block = """<collision>
      <origin xyz="0 0 0" rpy="1.5707963267948966 0 0" />
      <geometry>
        <cylinder radius="0.0488" length="0.052" />
      </geometry>
    """
    s = s[:c] + block + s[e:]
open(URDF, "w").write(s)

x = open(XACRO).read()
# drop kinematic base hint
x = re.sub(r'\s*<gazebo reference="base_footprint">\s*<kinematic>true</kinematic>\s*</gazebo>',
           "", x)
# friction back to 1.2
x = x.replace("<mu1>0.0</mu1>", "<mu1>1.2</mu1>")
x = x.replace("<mu2>0.0</mu2>", "<mu2>1.2</mu2>")

# drive plugin -> DiffDrive
for plug in ("gz-sim-velocity-control-system", "gz-sim-mecanum-drive-system"):
    start = x.find('<plugin filename="%s"' % plug)
    if start >= 0:
        end = x.find("</plugin>", start) + len("</plugin>")
        if x.find('<plugin filename="gz-sim-odometry-publisher-system"', end) >= 0:
            o = x.find('<plugin filename="gz-sim-odometry-publisher-system"', end)
            end = x.find("</plugin>", o) + len("</plugin>")
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
        x = x[:start] + new + x[end:]
        break

open(XACRO, "w").write(x)
print("robot restored: diff drive, cylinders, friction 1.2, axes +Y")
