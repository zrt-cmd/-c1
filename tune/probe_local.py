#!/usr/bin/env python3
"""Dump the local costmap around the robot."""
import sys

import rclpy
from nav2_msgs.srv import GetCostmap
from rclpy.node import Node

rclpy.init()
node = Node("local_probe",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
cli = node.create_client(GetCostmap, "/local_costmap/get_costmap")
if not cli.wait_for_service(timeout_sec=60):
    print("NO SERVICE")
    sys.exit(1)

req = GetCostmap.Request()
req.specs.resolution = 0.05
req.specs.size_x = 60
req.specs.size_y = 60
req.specs.origin.position.x = 0.0
req.specs.origin.position.y = 0.0
req.specs.origin.orientation.w = 1.0
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=20)
res = fut.result()
if res is None:
    print("NO RESPONSE")
    sys.exit(1)
md = res.map.metadata
data = res.map.data
print("meta res=%.3f size=%dx%d origin=(%.2f,%.2f)" % (
    md.resolution, md.size_x, md.size_y,
    md.origin.position.x, md.origin.position.y))
for j in range(md.size_y - 1, -1, -1):
    row = ""
    for i in range(md.size_x):
        c = data[j * md.size_x + i]
        row += "#" if c >= 254 else ("?" if c == 255 else ("x" if c == 253 else "."))
    print("%5.2f %s" % (md.origin.position.y + j * md.resolution, row))
