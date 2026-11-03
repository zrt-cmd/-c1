#!/usr/bin/env python3
"""Execute a stop-and-turn waypoint route with NavigateToPose."""
import json
import math
import subprocess
import sys
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

ROUTE = "/home/zrtzrt/tune/route.json"

rclpy.init()
node = Node("route_runner",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
cli = ActionClient(node, NavigateToPose, "navigate_to_pose")
if not cli.wait_for_server(timeout_sec=90):
    print("no action server")
    sys.exit(1)

deadline = time.time() + 150
while time.time() < deadline:
    try:
        out = subprocess.run(["ros2", "lifecycle", "get", "/bt_navigator"],
                             capture_output=True, text=True, timeout=5).stdout
        if out.strip().lower().startswith("active"):
            print("bt_navigator active", flush=True)
            break
    except Exception:
        pass
    time.sleep(2)
else:
    print("bt_navigator never active")
    sys.exit(1)

route = json.load(open(ROUTE))
print("waypoints:", len(route), flush=True)

for idx, wp in enumerate(route[1:], start=1):
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = wp["x"]
    goal.pose.pose.position.y = wp["y"]
    yaw = wp["yaw"]
    goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("WP %d rejected" % idx, flush=True)
        sys.exit(1)

    t0 = time.time()
    res_fut = gh.get_result_async()
    while rclpy.ok() and not res_fut.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 120:
            print("WP %d timeout" % idx, flush=True)
            gh.cancel_goal_async()
            sys.exit(2)

    r = res_fut.result()
    print("WP %d (%.2f,%.2f) status=%s err=%d %s" % (
        idx, wp["x"], wp["y"], r.status, r.result.error_code,
        r.result.error_msg), flush=True)
    if r.status != 4:   # STATUS_SUCCEEDED
        print("stopping at WP %d" % idx, flush=True)
        sys.exit(3)

print("ROUTE_DONE", flush=True)
