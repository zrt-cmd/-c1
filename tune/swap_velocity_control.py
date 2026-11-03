#!/usr/bin/env python3
"""Holonomic kinematic drive: VelocityControl + OdometryPublisher, no wheel physics."""
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(p).read()

for name in ("gz-sim-diff-drive-system", "gz-sim-mecanum-drive-system"):
    start = s.find('<plugin filename="%s"' % name)
    if start >= 0:
        end = s.find("</plugin>", start) + len("</plugin>")
        new = """<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
            <link_name>base_footprint</link_name>
            <topic>/cmd_vel</topic>
        </plugin>
        <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
            <dimensions>2</dimensions>
            <odom_frame>odom</odom_frame>
            <robot_base_frame>base_footprint</robot_base_frame>
            <odom_topic>/odom</odom_topic>
            <tf_topic>/tf</tf_topic>
            <odom_publish_frequency>50</odom_publish_frequency>
        </plugin>"""
        s = s[:start] + new + s[end:]
        open(p, "w").write(s)
        print("swapped", name, "-> VelocityControl + OdometryPublisher")
        break
else:
    print("no drive plugin found")
