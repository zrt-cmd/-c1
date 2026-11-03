#!/usr/bin/env python3
"""Configure a mecanum test variant, always starting from the original layout.

Usage: setup_mecanum.py MODE
  0: identity meshes, all wheel axes +Y
  1: swap left/right meshes, axes +Y
  2: identity meshes, right axes -Y
  3: swap left/right meshes, right axes -Y
  4: identity meshes, right meshes rotated 180 deg about Y
  5: identity meshes, left meshes rotated 180 deg about Y
  6: swap L/R + right meshes rotated 180 deg
"""
import re
import sys

MODE = int(sys.argv[1]) if len(sys.argv) > 1 else 0
URDF = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_description/urdf/car1.urdf"
XACRO = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"

LEFT = ["wheel_lf_link", "wheel_lb_link"]
RIGHT = ["wheel_rf_link", "wheel_rb_link"]
ALL = LEFT + RIGHT

s = open(URDF).read()

# --- reset to the original layout first ---
for link in ALL:
    i = s.find('name="%s"' % link)
    j = s.find("</link>", i)
    body = s[i:j]
    # every wheel mesh reference in this link points to its own mesh again
    body = re.sub(r"meshes/wheel_(lf|lb|rf|rb)_link\.STL",
                  "meshes/%s.STL" % link, body)
    body = body.replace('rpy="0 3.14159265 0"', 'rpy="0 0 0"')
    s = s[:i] + body + s[j:]

for name in ALL:
    i = s.find('name="%s"' % name)
    j = s.find("</joint>", i)
    block = s[i:j]
    block = re.sub(r'(<axis\s+xyz=")0\s+[-+]?1\s+0(")', r"\g<1>0 1 0\g<2>", block)
    s = s[:i] + block + s[j:]

# --- apply the mode ---
mesh_map = {n: n for n in ALL}
if MODE in (1, 3, 6):
    mesh_map.update({"wheel_lf_link": "wheel_rf_link", "wheel_rf_link": "wheel_lf_link",
                     "wheel_lb_link": "wheel_rb_link", "wheel_rb_link": "wheel_lb_link"})

flip = set()
if MODE == 4:
    flip = set(RIGHT)
elif MODE == 5:
    flip = set(LEFT)
elif MODE == 6:
    flip = set(RIGHT)

for link in ALL:
    i = s.find('name="%s"' % link)
    j = s.find("</link>", i)
    body = s[i:j]
    body = body.replace("meshes/%s.STL" % link, "meshes/%s.STL" % mesh_map[link])
    if link in flip:
        body = body.replace('rpy="0 0 0"', 'rpy="0 3.14159265 0"')
    s = s[:i] + body + s[j:]

for name in ALL:
    i = s.find('name="%s"' % name)
    j = s.find("</joint>", i)
    block = s[i:j]
    right = name.startswith("wheel_r")
    if MODE in (2, 3) and right:
        block = re.sub(r'(<axis\s+xyz=")0\s+1\s+0(")', r"\g<1>0 -1 0\g<2>", block)
    s = s[:i] + block + s[j:]

open(URDF, "w").write(s)

# --- drive plugin -> mecanum ---
x = open(XACRO).read()
for plug in ("gz-sim-diff-drive-system", "gz-sim-mecanum-drive-system"):
    start = x.find('<plugin filename="%s"' % plug)
    if start >= 0:
        end = x.find("</plugin>", start) + len("</plugin>")
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
            <max_velocity>0.6</max_velocity>
            <max_acceleration>5.0</max_acceleration>
            <max_jerk>10.0</max_jerk>
        </plugin>"""
        x = x[:start] + new + x[end:]
        open(XACRO, "w").write(x)
        break

print("mode", MODE, "flip", sorted(flip))
