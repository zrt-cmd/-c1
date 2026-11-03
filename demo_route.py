#!/usr/bin/env python3
"""Simple waypoint driver for the Gazebo avoidance demo.

Modes:
  grasp : corridor -> east side -> north -> face the red cube
  goal  : back away from cube -> around slalom -> red goal marker
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Route(Node):
    def __init__(self):
        super().__init__('demo_route')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.got = False

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.got = True

    def stop(self):
        self.pub.publish(Twist())

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.0)

    def goto(self, tx, ty, tyaw=None, reverse=False, timeout=45.0, tol=0.08):
        self.get_logger().info(
            f'goto ({tx:.2f},{ty:.2f}) yaw={tyaw if tyaw is None else round(tyaw,2)} reverse={reverse}')
        start = time.time()
        last_log = 0.0
        while rclpy.ok() and time.time() - start < timeout:
            self.spin_once()
            dx, dy = tx - self.x, ty - self.y
            dist = math.hypot(dx, dy)

            if dist < tol:
                if tyaw is None:
                    break
                err = wrap(tyaw - self.yaw)
                if abs(err) < 0.06:
                    break
                cmd = Twist()
                cmd.angular.z = max(-0.8, min(0.8, 1.6 * err))
                self.pub.publish(cmd)
                time.sleep(0.05)
                continue

            bearing = math.atan2(dy, dx)
            target_yaw = wrap(bearing + math.pi) if reverse else bearing
            err = wrap(target_yaw - self.yaw)
            cmd = Twist()
            if abs(err) > 0.5:
                cmd.angular.z = max(-0.8, min(0.8, 1.6 * err))
            else:
                speed = max(0.06, min(0.18, 0.6 * dist))
                cmd.linear.x = -speed if reverse else speed
                cmd.angular.z = max(-0.8, min(0.8, 1.5 * err))
            self.pub.publish(cmd)

            now = time.time()
            if now - last_log > 2.0:
                last_log = now
                self.get_logger().info(
                    f'  pose=({self.x:.2f},{self.y:.2f},{self.yaw:.2f}) dist={dist:.2f}')
            time.sleep(0.05)
        self.stop()
        final_dist = math.hypot(tx - self.x, ty - self.y)
        ok = final_dist < tol * 1.6
        self.get_logger().info(f'  done dist={final_dist:.2f} ok={ok}')
        return ok

    def run(self, mode):
        while rclpy.ok() and not self.got:
            self.spin_once()
            time.sleep(0.05)
        self.get_logger().info(f'start mode={mode} pose=({self.x:.2f},{self.y:.2f})')

        if mode == 'grasp':
            self.goto(1.8, 0.0, tyaw=0.0)
            self.goto(1.8, 2.6, tyaw=math.pi / 2)
            self.goto(0.25, 2.6, tyaw=math.pi)
        elif mode == 'goal':
            self.goto(1.2, 2.6, tyaw=math.pi, reverse=True, timeout=25)
            self.goto(1.2, 2.6, tyaw=0.0, timeout=15)
            self.goto(4.2, 2.6, tyaw=0.0)
            self.goto(4.2, 3.0, tyaw=math.pi / 2)
        else:
            self.get_logger().error(f'unknown mode {mode}')

        self.stop()
        self.get_logger().info('route finished')


def main():
    rclpy.init()
    node = Route()
    mode = sys.argv[1] if len(sys.argv) > 1 else 'grasp'
    try:
        node.run(mode)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
