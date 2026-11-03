#!/usr/bin/env python3
"""Print the local costmap OccupancyGrid around the robot."""
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("local_probe2",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
buf = Buffer()
TransformListener(buf, node)
last = {}
node.create_subscription(OccupancyGrid, "/local_costmap/costmap",
                         lambda m: last.update(m=m), 1)

t0 = time.time()
while time.time() - t0 < 15 and "m" not in last:
    rclpy.spin_once(node, timeout_sec=0.2)
if "m" not in last:
    print("NO COSTMAP")
    sys.exit(1)

m = last["m"]
info = m.info
print("res=%.3f size=%dx%d origin=(%.2f,%.2f)" % (
    info.resolution, info.width, info.height,
    info.origin.position.x, info.origin.position.y))

try:
    tf = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
    rx, ry = tf.transform.translation.x, tf.transform.translation.y
    print("robot at (%.2f, %.2f)" % (rx, ry))
except Exception:
    rx = ry = None

ri = int((rx - info.origin.position.x) / info.resolution) if rx is not None else -1
rj = int((ry - info.origin.position.y) / info.resolution) if ry is not None else -1

for j in range(info.height - 1, -1, -1):
    row = ""
    for i in range(info.width):
        v = m.data[j * info.width + i]
        row += "#" if v >= 99 else ("?" if v < 0 else ("x" if v >= 90 else "."))
    if rx is not None and j == rj:
        row = row[:ri] + "@" + row[ri + 1:]
    print("%5.2f %s" % (info.origin.position.y + j * info.resolution, row))
