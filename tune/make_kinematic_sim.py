#!/usr/bin/env python3
"""Kinematic holonomic sim: zero gravity + VelocityControl + OdometryPublisher."""
WORLD = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/worlds/math_room.sdf"
XACRO = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"

w = open(WORLD).read()
if "<gravity>" not in w:
    w = w.replace('<world name="math_room">',
                  '<world name="math_room">\n    <gravity>0 0 0</gravity>', 1)
    open(WORLD, "w").write(w)
    print("gravity set to zero")
else:
    print("gravity already set")

x = open(XACRO).read()
for plug in ("gz-sim-diff-drive-system", "gz-sim-mecanum-drive-system"):
    start = x.find('<plugin filename="%s"' % plug)
    if start >= 0:
        end = x.find("</plugin>", start) + len("</plugin>")
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
        x = x[:start] + new + x[end:]
        open(XACRO, "w").write(x)
        print("swapped to VelocityControl + OdometryPublisher")
        break
else:
    print("drive plugin already kinematic")
