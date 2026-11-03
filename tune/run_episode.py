#!/usr/bin/env python3
"""Run one G->C episode and write metrics JSON.

If --no-launch is not given, it starts the headless Gazebo + Nav2 launch
with the given params file, runs the episode, then kills the process group.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class EpisodeMonitor(Node):
    def __init__(self, collision_dist):
        super().__init__(
            "episode_monitor",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", value=True)
            ],
        )
        self.collision_dist = collision_dist
        self.min_range = float("inf")
        self.collided = False
        self.collision_pose = None
        self.collision_angle = None
        self.path_len = 0.0
        self.last_odom = None
        self.amcl_pose = None
        self.traj = []
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.amcl_cb, 10
        )

    def scan_cb(self, msg):
        best_r = float("inf")
        best_a = None
        for i, r in enumerate(msg.ranges):
            if math.isfinite(r) and r > 0.01 and r < best_r:
                best_r = r
                best_a = msg.angle_min + i * msg.angle_increment
        if best_a is not None:
            if best_r < self.min_range:
                self.min_range = best_r
            if best_r < self.collision_dist and not self.collided:
                self.collided = True
                self.collision_angle = best_a
                if self.amcl_pose is not None:
                    self.collision_pose = self.amcl_pose
                elif self.last_odom is not None:
                    self.collision_pose = (
                        self.last_odom[0] + 0.15,
                        self.last_odom[1] + 1.70,
                    )

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        if self.last_odom is not None:
            self.path_len += math.hypot(
                p.x - self.last_odom[0], p.y - self.last_odom[1]
            )
        self.last_odom = (p.x, p.y)
        if not self.traj or math.hypot(p.x - self.traj[-1][0],
                                       p.y - self.traj[-1][1]) > 0.10:
            self.traj.append((p.x, p.y))

    def amcl_cb(self, msg):
        p = msg.pose.pose.position
        self.amcl_pose = (p.x, p.y)


def wait_for_nav2(env, timeout=120.0):
    """Wait until the bt_navigator lifecycle node is active."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            out = subprocess.check_output(
                ["ros2", "lifecycle", "get", "/bt_navigator"],
                env=env,
                stderr=subprocess.DEVNULL,
                timeout=5,
            ).decode()
            state = out.strip().split()[0].lower() if out.strip() else ""
            if state == "active":
                return True
        except Exception:
            pass
        time.sleep(2.0)
    return False


def wait_topic(env, topic, field="header", timeout=45.0):
    """Wait until a topic can be read once (map/costmap readiness gate)."""
    start = time.time()
    while time.time() - start < timeout:
        try:
            subprocess.check_output(
                ["ros2", "topic", "echo", topic, "--once", "--field", field],
                env=env,
                stderr=subprocess.DEVNULL,
                timeout=6,
            )
            return True
        except Exception:
            pass
        time.sleep(2.0)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--goal-x", type=float, default=-1.80)
    ap.add_argument("--goal-y", type=float, default=1.20)
    ap.add_argument("--domain", type=int, default=-1)
    ap.add_argument("--partition", default="")
    ap.add_argument("--collision-dist", type=float, default=0.08)
    ap.add_argument("--no-launch", action="store_true")
    ap.add_argument("--launch-retries", type=int, default=1)
    ap.add_argument("--no-cleanup", action="store_true")
    ap.add_argument("--visual", action="store_true")
    args = ap.parse_args()

    if args.domain >= 0:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    if args.partition:
        os.environ["GZ_PARTITION"] = args.partition
    os.environ.setdefault("DISPLAY", ":0")
    os.environ.setdefault("GALLIUM_DRIVER", "d3d12")
    os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    os.environ.pop("WAYLAND_DISPLAY", None)
    env = os.environ.copy()

    def cleanup():
        if args.no_cleanup:
            return
        for pat in ("[g]z sim", "[g]z-sim", "[r]os_gz", "[r]os2 launch",
                    "[c]ontroller_server", "[p]lanner_server", "[b]t_navigator",
                    "[b]ehavior_server", "[v]elocity_smoother", "[m]ap_server",
                    "[a]mcl", "[l]ifecycle_manager"):
            subprocess.run(["pkill", "-9", "-f", pat], env=env, check=False)
        time.sleep(1.0)

    def stop_proc(p):
        if p is None:
            return
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except Exception:
            pass
        time.sleep(2.0)
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass
        time.sleep(1.0)

    proc = None
    nav2_ok = False
    sim_log = args.out + ".sim.log"
    if not args.no_launch:
        cmd = [
            "ros2",
            "launch",
            os.path.expanduser("~/tune/gazebo_nav2.launch.py"),
            f"params_file:={args.params}",
        ]
        if args.visual:
            cmd.append("headless:=false")
        for attempt in range(args.launch_retries + 1):
            cleanup()
            sim_f = open(sim_log, "a")
            sim_f.write(f"\n===== launch attempt {attempt + 1} =====\n")
            sim_f.flush()
            proc = subprocess.Popen(
                cmd,
                env=env,
                stdout=sim_f,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            if wait_for_nav2(env):
                nav2_ok = True
                break
            stop_proc(proc)
            proc = None

    metrics = {
        "success": False,
        "status": "not_started",
        "time": None,
        "path_length": None,
        "min_clearance": None,
        "collided": False,
        "final_dist": None,
        "goal": [args.goal_x, args.goal_y],
        "params": args.params,
    }

    node = None
    try:
        if args.no_launch:
            if not wait_for_nav2(env):
                raise RuntimeError("Nav2 bt_navigator did not become active")
        elif not nav2_ok:
            raise RuntimeError(
                f"Nav2 did not become active after {args.launch_retries + 1} attempts")
        if not wait_topic(env, "/global_costmap/costmap", field="info"):
            raise RuntimeError("global costmap not ready")
        if not wait_topic(env, "/local_costmap/costmap", field="info"):
            raise RuntimeError("local costmap not ready")
        time.sleep(2.0)

        rclpy.init()
        node = EpisodeMonitor(args.collision_dist)
        client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        if not client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("action server not ready")

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        goal.pose.pose.orientation.w = 1.0

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("goal rejected")

        start = time.time()
        result_future = goal_handle.get_result_async()
        status = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.collided:
                goal_handle.cancel_goal_async()
                status = "collision"
                break
            if result_future.done():
                status = result_future.result().status
                break
            if time.time() - start > args.timeout:
                goal_handle.cancel_goal_async()
                status = "timeout"
                break

        elapsed = time.time() - start
        final = node.amcl_pose
        if final is None and node.last_odom is not None:
            # odom origin is the spawn pose (0.15, 1.70) in map frame
            final = (node.last_odom[0] + 0.15, node.last_odom[1] + 1.70)
        final_dist = (
            math.hypot(final[0] - args.goal_x, final[1] - args.goal_y)
            if final
            else None
        )
        success = (
            status == GoalStatus.STATUS_SUCCEEDED
            and not node.collided
            and final_dist is not None
            and final_dist < 0.15
        )
        metrics.update(
            {
                "success": bool(success),
                "status": str(status),
                "time": round(elapsed, 3),
                "path_length": round(node.path_len, 3),
                "min_clearance": (
                    round(node.min_range, 4)
                    if math.isfinite(node.min_range)
                    else None
                ),
                "collided": bool(node.collided),
                "collision_pose": node.collision_pose,
                "collision_angle_deg": (
                    round(math.degrees(node.collision_angle), 1)
                    if node.collision_angle is not None
                    else None
                ),
                "final_dist": None if final_dist is None else round(final_dist, 4),
                "trajectory": [
                    [round(x, 3), round(y, 3)] for x, y in node.traj
                ],
            }
        )
    except Exception as exc:
        metrics["status"] = f"error: {exc}"
    finally:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(metrics, f, indent=2)
        print(json.dumps(metrics, indent=2))
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        if proc is not None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            except Exception:
                pass
            time.sleep(2.0)
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass


if __name__ == "__main__":
    main()
