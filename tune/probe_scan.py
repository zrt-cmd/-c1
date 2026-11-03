#!/usr/bin/env python3
"""Print all close laser points in world coords (odom + spawn offset fallback)."""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("scan_probe",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
buf = Buffer()
TransformListener(buf, node)
last = {}


def cb(msg):
    last["msg"] = msg


node.create_subscription(LaserScan, "/scan", cb, 10)

t0 = time.time()
while time.time() - t0 < 5 and "msg" not in last:
    rclpy.spin_once(node, timeout_sec=0.1)
if "msg" not in last:
    print("NO SCAN")
    raise SystemExit(1)

# let TF fill in
t0 = time.time()
while time.time() - t0 < 3:
    rclpy.spin_once(node, timeout_sec=0.1)

msg = last["msg"]
stamp = rclpy.time.Time.from_msg(msg.header.stamp)
lx = ly = 0.0
lyaw = 0.0
for parent, child in (("map", "laser_link"), ("odom", "laser_link"),
                      ("base_footprint", "laser_link")):
    try:
        tf = buf.lookup_transform(parent, child, stamp)
        lx = tf.transform.translation.x
        ly = tf.transform.translation.y
        q = tf.transform.rotation
        lyaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))
        if parent != "map":
            lx += 0.15
            ly += 1.70
        print("frame", parent, "-> laser at (%.3f, %.3f) yaw=%.1fdeg" % (
            lx, ly, math.degrees(lyaw)))
        break
    except Exception:
        continue
else:
    print("no TF, assuming laser at spawn")

pts = []
for i, r in enumerate(msg.ranges):
    if math.isfinite(r) and 0.01 < r < 0.40:
        a = msg.angle_min + i * msg.angle_increment
        wx = lx + r * math.cos(lyaw + a)
        wy = ly + r * math.sin(lyaw + a)
        pts.append((wx, wy, r, math.degrees(a)))
pts.sort(key=lambda p: p[2])
print("close points (<0.40m):", len(pts))
for wx, wy, r, a in pts[:40]:
    print("  world=(%+.3f,%+.3f) r=%.3f ang=%+.1f" % (wx, wy, r, a))
