#!/usr/bin/env python3
"""Replace the DiffDrive plugin with the Gazebo mecanum drive plugin."""
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(p).read()

start = s.find('<plugin filename="gz-sim-diff-drive-system"')
if start < 0:
    print("diff drive plugin not found (already swapped?)")
    raise SystemExit(0)
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
            <max_velocity>0.6</max_velocity>
            <max_acceleration>2.5</max_acceleration>
            <max_jerk>5.0</max_jerk>
        </plugin>"""

s = s[:start] + new + s[end:]
open(p, "w").write(s)
print("swapped to MecanumDrive")
