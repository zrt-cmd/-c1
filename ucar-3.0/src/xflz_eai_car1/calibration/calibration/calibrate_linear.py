#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Linear velocity calibration node (reviewed & extended).

Drives the robot forward/backward a known distance, then compares odometry vs
physical measurement (用尺子量实际距离).

判停可切换：默认用 /odom_raw（纯轮式里程），也可用 /odometry/filtered (EKF)。
结束后同时打印 odom_raw 与 EKF 报告位移，方便用尺子量物理真值后三层对比。

注意：系数公式 = 实际距离 / odom报告距离（参照 odom_publisher 的 odom报告=factor×积分），
不要按本文件历史 docstring 的"test_distance/实际距离"算（那是错的）。
"""
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class CalibrateLinear(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        signal.signal(signal.SIGINT, self.shutdown)

        # --- 运动参数 ---
        self.declare_parameter('test_distance', 1.0)     # 目标距离 (m)
        self.declare_parameter('speed', 0.2)             # 行进速度 (m/s)
        self.declare_parameter('direction', 1.0)         # 1=向前, -1=向后
        self.declare_parameter('tolerance', 0.03)        # 判停容差 (m)
        self.declare_parameter('settle_time', 1.0)       # 到位后停车稳定时间 (s)
        # --- 判停/测量参数 ---
        self.declare_parameter('use_ekf', False)         # False=odom_raw 判停, True=EKF 判停
        self.declare_parameter('odom_linear_scale_correction', 1.0)
        self.declare_parameter('start_test', False)
        self.update_param()

        self.cmd_vel = self.create_publisher(Twist, "/controller/cmd_vel", 1)
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)

        self.pos = None          # 最新 (x, y) from /odom_raw
        self.last_pos = None     # 上一帧 (x, y)
        self.ekf_pos = None      # 最新 (x, y) from /odometry/filtered
        self.traveled = 0.0      # 累加行进里程 (判停用)

        self.clock = self.get_clock()
        threading.Thread(target=self.main, daemon=True).start()

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Set start_test:=true to run')

    def update_param(self):
        self.start_test = self.get_parameter('start_test').value
        self.test_distance = self.get_parameter('test_distance').value
        self.speed = self.get_parameter('speed').value
        self.direction = self.get_parameter('direction').value
        self.tolerance = self.get_parameter('tolerance').value
        self.settle_time = self.get_parameter('settle_time').value
        self.use_ekf = self.get_parameter('use_ekf').value
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').value

    def odom_callback(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def ekf_callback(self, msg):
        self.ekf_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _travel(self, start, now):
        if start is None or now is None:
            return 0.0
        return ((now[0] - start[0]) ** 2 + (now[1] - start[1]) ** 2) ** 0.5

    def main(self):
        running = False
        while True:
            self.update_param()
            move_cmd = Twist()

            if self.start_test:
                if self.use_ekf:
                    source = self.ekf_pos
                else:
                    source = self.pos
                if source is None:
                    self.get_logger().warn('Waiting for odometry...')
                    time.sleep(0.1)
                    continue

                if not running:
                    # 开始一次新测试：记录起终点，清零累加
                    running = True
                    self.odom_start = self.pos
                    self.ekf_start = self.ekf_pos
                    self.last_pos = self.pos
                    self.traveled = 0.0

                # 累加这一帧的行进里程（幅度，与方向无关）
                dx = self.pos[0] - self.last_pos[0]
                dy = self.pos[1] - self.last_pos[1]
                self.traveled += (dx * dx + dy * dy) ** 0.5
                self.last_pos = self.pos

                # 判停用 odom_raw 或 EKF（use_ekf=true 时用 EKF 判停）
                if self.use_ekf:
                    calib = self._travel(self.ekf_start, self.ekf_pos) * self.odom_linear_scale_correction
                else:
                    calib = self._travel(self.odom_start, self.pos) * self.odom_linear_scale_correction
                remaining = self.test_distance - calib

                if remaining > self.tolerance:
                    move_cmd.linear.x = self.direction * self.speed
                else:
                    # 到位：先停，等车停稳再读最终里程
                    self.cmd_vel.publish(Twist())
                    time.sleep(self.settle_time)
                    d_odom = self._travel(self.odom_start, self.pos)
                    d_ekf = self._travel(self.ekf_start, self.ekf_pos)
                    running = False
                    self.start_test = rclpy.parameter.Parameter(
                        'start_test', rclpy.Parameter.Type.BOOL, False)
                    self.set_parameters([self.start_test])
                    self.get_logger().info('\033[1;32mfinish: '
                                           'odom_raw=%.3fm  EKF=%.3fm  target=%.3fm '
                                           '(※ 用尺子量实际物理距离, factor=实际/odom报告)\033[0m'
                                           % (d_odom, d_ekf, self.test_distance))
            else:
                running = False
                if self.pos is not None:
                    self.last_pos = self.pos

            self.cmd_vel.publish(move_cmd)
            time.sleep(0.05)

    def shutdown(self, signum, frame):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Stopping the robot...')
        self.start_test = False
        self.cmd_vel.publish(Twist())
        rclpy.shutdown()


def main():
    node = CalibrateLinear('calibrate_linear')
    rclpy.spin(node)


if __name__ == "__main__":
    main()
