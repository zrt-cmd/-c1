#!/usr/bin/env python3
"""Call ComputePathToPose from several start poses and print error codes."""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node

rclpy.init()
node = Node("plan_probe",
            parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
cli = ActionClient(node, ComputePathToPose, "/compute_path_to_pose")
if not cli.wait_for_server(timeout_sec=60):
    print("NO PLANNER")
    sys.exit(1)


def pose(x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x = x
    p.pose.position.y = y
    import math
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


starts = [(0.15, 1.70), (0.30, 1.70), (0.42, 1.70), (0.45, 1.70),
          (0.55, 1.70), (0.65, 1.60), (0.77, 1.55), (0.77, 1.45)]

use_current = "--current" in sys.argv
if use_current:
    starts = [(0.0, 0.0)]

for sx, sy in starts:
    goal = ComputePathToPose.Goal()
    goal.goal = pose(-1.80, 1.20)
    goal.start = pose(sx, sy)
    goal.planner_id = "GridBased"
    goal.use_start = not use_current

    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("(%+.2f,%+.2f) REJECTED" % (sx, sy))
        continue
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=60)
    res = rf.result()
    if res is None:
        print("(%+.2f,%+.2f) NO RESULT" % (sx, sy))
        continue
    r = res.result
    print("(%+.2f,%+.2f) code=%d n=%d msg=%s" % (
        sx, sy, r.error_code, len(r.path.poses), r.error_msg))
