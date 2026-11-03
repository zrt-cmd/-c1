#!/usr/bin/env python3
"""Compare AMCL map pose with odom+spawn dead reckoning."""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("pose_probe",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
buf = Buffer()
TransformListener(buf, node)
odom = {}
node.create_subscription(Odometry, "/odom",
                         lambda m: odom.update(x=m.pose.pose.position.x,
                                               y=m.pose.pose.position.y), 10)

t0 = time.time()
while time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.1)

try:
    tf = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
    mx, my = tf.transform.translation.x, tf.transform.translation.y
    q = tf.transform.rotation
    myaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))
    print("AMCL map pose : (%+.3f, %+.3f) yaw=%+.1f deg" % (mx, my, math.degrees(myaw)))
except Exception as exc:
    print("map tf error:", str(exc)[:80])
    mx = my = None

if odom:
    ox, oy = odom["x"] + 0.15, odom["y"] + 1.70
    print("odom+spawn    : (%+.3f, %+.3f)" % (ox, oy))
    if mx is not None:
        print("difference    : (%.3f, %.3f) => %.3f m" % (
            mx - ox, my - oy, math.hypot(mx - ox, my - oy)))
else:
    print("no odom")
