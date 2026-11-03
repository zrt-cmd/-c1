#!/usr/bin/env python3
"""Send a goal and log ground-truth / odom / AMCL poses side by side."""
import math
import os
import re
import subprocess
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

env = os.environ.copy()
env["GZ_PARTITION"] = os.environ.get("GZ_PARTITION", "d2")


def gzpose():
    out = subprocess.run(["gz", "model", "-m", "car1", "-p"],
                         capture_output=True, text=True, env=env).stdout
    m = re.search(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
    return (float(m.group(1)), float(m.group(2))) if m else None


rclpy.init()
node = Node("monitor",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
buf = Buffer()
TransformListener(buf, node)
odom = {}
node.create_subscription(
    Odometry, "/odom",
    lambda m: odom.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y), 10)

cli = ActionClient(node, NavigateToPose, "navigate_to_pose")
if not cli.wait_for_server(timeout_sec=90):
    print("no action server")
    raise SystemExit(1)

# bt_navigator's action server exists while the node is still configuring;
# wait for the lifecycle state to be active before sending a goal.
deadline = time.time() + 120
while time.time() < deadline:
    try:
        out = subprocess.run(["ros2", "lifecycle", "get", "/bt_navigator"],
                             capture_output=True, text=True, timeout=5).stdout
        if out.strip().lower().startswith("active"):
            print("bt_navigator active")
            break
    except Exception:
        pass
    time.sleep(2)
else:
    print("bt_navigator never active")
    raise SystemExit(1)

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.header.stamp = node.get_clock().now().to_msg()
goal.pose.pose.position.x = -1.80
goal.pose.pose.position.y = 1.20
goal.pose.pose.orientation.w = 1.0
fut = cli.send_goal_async(goal)
rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
gh = fut.result()
print("goal accepted:", gh is not None and gh.accepted)
res_fut = gh.get_result_async()

t0 = time.time()
last_print = 0
while time.time() - t0 < 60:
    rclpy.spin_once(node, timeout_sec=0.1)
    if time.time() - last_print >= 3.0:
        last_print = time.time()
        g = gzpose()
        o = (odom.get("x", 0) + 0.15, odom.get("y", 0) + 1.70)
        try:
            tf = buf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            a = (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            a = None
        print("[%4.1fs] gz=(%+.2f,%+.2f) odom=(%+.2f,%+.2f) amcl=%s" % (
            time.time() - t0, g[0] if g else 0, g[1] if g else 0,
            o[0], o[1], None if a is None else "(%+.2f,%+.2f)" % a), flush=True)
    if res_fut.done():
        r = res_fut.result()
        print("action done status", r.status,
              "error_code", r.result.error_code, r.result.error_msg)
        break
