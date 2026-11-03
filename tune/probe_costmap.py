#!/usr/bin/env python3
"""Dump global costmap values around the G->C failure area."""
import sys
import time

import rclpy
from nav2_msgs.srv import GetCostmap
from rclpy.node import Node

if "--wait" in sys.argv:
    time.sleep(float(sys.argv[sys.argv.index("--wait") + 1]))

rclpy.init()
node = Node("costmap_probe",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
cli = node.create_client(GetCostmap, "/global_costmap/get_costmap")
if not cli.wait_for_service(timeout_sec=90):
    print("NO SERVICE")
    sys.exit(1)

req = GetCostmap.Request()
req.specs.resolution = 0.05
req.specs.size_x = 108
req.specs.size_y = 80
req.specs.origin.position.x = -2.7
req.specs.origin.position.y = -2.0
req.specs.origin.orientation.w = 1.0

fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
res = fut.result()
if res is None:
    print("NO RESPONSE")
    sys.exit(1)

md = res.map.metadata
data = res.map.data
print("meta res=%.3f size=%dx%d origin=(%.2f,%.2f) n=%d" % (
    md.resolution, md.size_x, md.size_y,
    md.origin.position.x, md.origin.position.y, len(data)))


def cost(x, y):
    i = int(round((x - md.origin.position.x) / md.resolution))
    j = int(round((y - md.origin.position.y) / md.resolution))
    if 0 <= i < md.size_x and 0 <= j < md.size_y:
        return data[j * md.size_x + i]
    return None


pts = [(0.15, 1.70), (0.30, 1.70), (0.42, 1.70), (0.45, 1.70),
       (0.50, 1.65), (0.60, 1.55), (0.75, 1.45), (0.77, 1.41),
       (0.77, 1.35), (0.90, 1.20), (-1.80, 1.20)]
for p in pts:
    print("(%+.2f,%+.2f) = %s" % (p[0], p[1], cost(*p)))

print("\n  full map (every 2 cells, x -2.7..2.7, y 2.0..-2.0)")
for j in range(md.size_y - 1, -1, -2):
    row = ""
    for i in range(0, md.size_x, 2):
        c = data[j * md.size_x + i]
        row += "#" if c >= 254 else ("?" if c == 255 else ("x" if c == 253 else "."))
    print("%5.1f %s" % (md.origin.position.y + j * md.resolution, row))

print("\n  x: 0.20 .. 1.20 m (left to right), y: 2.00 down to 1.20")
x0, x1, y0, y1 = 0.20, 1.20, 1.20, 2.00
i0 = int(round((x0 - md.origin.position.x) / md.resolution))
i1 = int(round((x1 - md.origin.position.x) / md.resolution))
j0 = max(0, int(round((y0 - md.origin.position.y) / md.resolution)))
j1 = min(md.size_y - 1, int(round((y1 - md.origin.position.y) / md.resolution)))
for j in range(j1, j0 - 1, -1):
    row = ""
    for i in range(i0, i1 + 1):
        c = data[j * md.size_x + i]
        if c >= 254:
            row += "#"
        elif c == 255:
            row += "?"
        elif c == 253:
            row += "x"
        else:
            row += "."
    print("%5.2f %s" % (md.origin.position.y + j * md.resolution, row))
