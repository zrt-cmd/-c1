#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Angular velocity calibration node (reviewed & extended).

Rotates the robot a known angle, then compares IMU / odom_raw / EKF and the
physical angle. 判停可切换（odom_raw / EKF），结束时同时打印三个航向源的累计角，
这样一次就能定位建图斜角的根因（是 IMU 漂移还是 EKF 融合问题）。

说明：
- odom_raw 是纯轮式开环积分（会因麦轮打滑而欠转）。
- IMU 陀螺独立于轮子，是旋转的"半真值"参考（scale≈1.01 已验证）。
- EKF 是 odom + IMU 融合，slam/导航实际用它。
- 系数公式（angular_correction_factor）= 真实转角 / 命令角，
  但真实转角应以 IMU 或指南针为准，不要信 odom 按钮停值。
"""
import time
import rclpy
import signal
import threading
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from math import copysign, pi, atan2, asin, radians


def qua2rpy(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = asin(2 * (w * y - x * z))
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw


def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res


class CalibrateAngular(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        signal.signal(signal.SIGINT, self.shutdown)

        # --- 运动/判停参数 ---
        self.declare_parameter('test_angle', 90.0)       # 目标转角 (deg)
        self.declare_parameter('speed', 0.3)             # 旋转速度 (rad/s)
        self.declare_parameter('tolerance', 1.5)         # 判停容差 (deg)
        self.declare_parameter('use_ekf', False)         # False=odom_raw判停, True=EKF判停
        self.declare_parameter('odom_angular_scale_correction', 1.0)
        self.declare_parameter('start_test', False)
        self.update_param()

        self.cmd_vel = self.create_publisher(Twist, "/controller/cmd_vel", 1)
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # --- 数据源 ---
        self.odom_yaw = None        # odom_raw yaw (rad)
        self.odom_start = None
        self.ekf_yaw = None         # EKF yaw (rad)
        self.ekf_start = None
        self.imu_ang = None         # 最新 gyro_z (rad/s)
        self.imu_integ = 0.0
        self._last_imu_t = None

        self.last_angle = 0
        self.reverse = 1
        self.calib_angle = 0
        self._running = False

        self.clock = self.get_clock()
        threading.Thread(target=self.main, daemon=True).start()

        self.get_logger().info('\033[1;32m%s\033[0m' % 'Set start_test:=true to run')

    def update_param(self):
        self.start_test = self.get_parameter('start_test').value
        self.test_angle = self.get_parameter('test_angle').value
        self.speed = self.get_parameter('speed').value
        self.tolerance = self.get_parameter('tolerance').value
        self.use_ekf = self.get_parameter('use_ekf').value
        self.odom_angular_scale_correction = self.get_parameter('odom_angular_scale_correction').value

    def odom_callback(self, msg):
        self.odom_yaw = qua2rpy(msg.pose.pose.orientation)[2]

    def ekf_callback(self, msg):
        self.ekf_yaw = qua2rpy(msg.pose.pose.orientation)[2]

    def imu_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_imu_t is not None:
            self.imu_integ += msg.angular_velocity.z * (now - self._last_imu_t)
        self._last_imu_t = now
        self.imu_ang = msg.angular_velocity.z

    def main(self):
        while True:
            self.update_param()
            move_cmd = Twist()

            if self.start_test:
                # 判停信号源
                if self.use_ekf and self.ekf_yaw is not None:
                    src_yaw = self.ekf_yaw
                elif self.odom_yaw is not None:
                    src_yaw = self.odom_yaw
                else:
                    self.get_logger().warn('Waiting for odometry...')
                    time.sleep(0.1)
                    continue

                if not self._running:
                    # 一次测试开始：记录各源起始，清零累加
                    self._running = True
                    self.odom_start = self.odom_yaw
                    self.ekf_start = self.ekf_yaw
                    self.last_angle = src_yaw
                    self.calib_angle = 0.0
                    self.imu_integ = 0.0
                    self._last_imu_t = None   # 重设 IMU 积分起点

                target = radians(abs(self.test_angle))
                step = abs(self.odom_angular_scale_correction *
                           normalize_angle(src_yaw - self.last_angle))
                self.calib_angle += step          # 判停用的累计角
                self.last_angle = src_yaw
                remaining = target - self.calib_angle

                if remaining > radians(self.tolerance):
                    move_cmd.angular.z = copysign(self.speed, self.reverse)
                else:
                    # 到位：先停
                    self.cmd_vel.publish(Twist())
                    time.sleep(1.0)
                    d_odom = abs(normalize_angle(self.odom_yaw - self.odom_start)) if self.odom_yaw and self.odom_start else 0.0
                    d_ekf = abs(normalize_angle(self.ekf_yaw - self.ekf_start)) if self.ekf_yaw and self.ekf_start else 0.0
                    d_imu = abs(self.imu_integ)
                    self._running = False
                    self.start_test = rclpy.parameter.Parameter(
                        'start_test', rclpy.Parameter.Type.BOOL, False)
                    self.set_parameters([self.start_test])
                    self.reverse = -self.reverse
                    self.last_angle = 0.0
                    self.get_logger().info(
                        '\033[1;32mfinish: '
                        'IMU=%.1f°  odom_raw=%.1f°  EKF=%.1f°  target=%.1f° '
                        '(※ 用指南针/IMU量真实转角, angular_factor=真实/命令)\033[0m'
                        % (d_imu * 180.0 / pi, d_odom * 180.0 / pi, d_ekf * 180.0 / pi,
                           self.test_angle))
            else:
                self._running = False
                if self.odom_yaw is not None:
                    self.last_angle = self.odom_yaw
                if self._last_imu_t is None:
                    self.imu_integ = 0.0

            self.cmd_vel.publish(move_cmd)
            time.sleep(0.05)

    def shutdown(self, signum, frame):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'Stopping the robot...')
        self.start_test = False
        self.cmd_vel.publish(Twist())
        rclpy.shutdown()


def main():
    node = CalibrateAngular('calibrate_angular')
    rclpy.spin(node)


if __name__ == "__main__":
    main()
