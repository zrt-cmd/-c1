#!/usr/bin/env python3
"""Replace VelocityControl+OdometryPublisher with the proven DiffDrive plugin."""
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(p).read()

start = s.find('<plugin filename="gz-sim-velocity-control-system"')
if start < 0:
    print("velocity control plugin not found")
    raise SystemExit(0)
end = s.find("</plugin>", start)
# also drop the odometry publisher that follows it
if s.find('<plugin filename="gz-sim-odometry-publisher-system"', end) >= 0:
    o = s.find('<plugin filename="gz-sim-odometry-publisher-system"', end)
    end = s.find("</plugin>", o)
end += len("</plugin>")

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
open(p, "w").write(s)
print("restored DiffDrive")
