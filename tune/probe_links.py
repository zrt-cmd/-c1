#!/usr/bin/env python3
"""Print arm/robot link positions relative to base_link."""
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("links_probe",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
buf = Buffer()
TransformListener(buf, node)

links = ["laser_link", "arm_base_link", "arm_link1", "arm_link2", "arm_link3",
         "arm_link4", "arm_link5", "gripper_left_link", "gripper_right_link"]

t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(node, timeout_sec=0.1)

for link in links:
    try:
        tf = buf.lookup_transform("base_link", link, rclpy.time.Time())
        t = tf.transform.translation
        q = tf.transform.rotation
        import math
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        print("%-20s (%+.3f, %+.3f, %+.3f) yaw=%+.1f" % (
            link, t.x, t.y, t.z, math.degrees(yaw)))
    except Exception as exc:
        print("%-20s error: %s" % (link, str(exc)[:60]))
