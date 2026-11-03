#!/usr/bin/env python3
# encoding: utf-8
"""Odometry publisher node: cmd_vel -> motor commands + odometry integration."""
import math
import time
import threading
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Pose2D, PoseWithCovarianceStamped
from car1_base_driver_msgs.msg import MotorsState
from controller.mecanum import MecanumChassis

ODOM_POSE_COVARIANCE = list(map(float,
    [1e-3, 0, 0, 0, 0, 0,
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e3]))

ODOM_POSE_COVARIANCE_STOP = list(map(float,
    [1e-9, 0, 0, 0, 0, 0,
     0, 1e-3, 1e-9, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-9]))

ODOM_TWIST_COVARIANCE = list(map(float,
    [1e-3, 0, 0, 0, 0, 0,
     0, 1e-3, 0, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-2]))

ODOM_TWIST_COVARIANCE_STOP = list(map(float,
    [1e-9, 0, 0, 0, 0, 0,
     0, 1e-3, 1e-9, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-9]))


def rpy2qua(roll, pitch, yaw):
    """Convert roll/pitch/yaw to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Pose()
    q.orientation.w = cy * cp * cr + sy * sp * sr
    q.orientation.x = cy * cp * sr - sy * sp * cr
    q.orientation.y = sy * cp * sr + cy * sp * cr
    q.orientation.z = sy * cp * cr - cy * sp * sr
    return q.orientation


class OdomPublisherNode(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.x = 0.0
        self.y = 0.0
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.pose_yaw = 0.0
        self.last_time = None

        # Parameters
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('linear_correction_factor', 1.00)
        self.declare_parameter('angular_correction_factor', 1.00)
        self.declare_parameter('wheelbase', 0.1946)
        self.declare_parameter('track_width', 0.216)
        self.declare_parameter('wheel_diameter', 0.0935)

        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.linear_factor = self.get_parameter('linear_correction_factor').value
        self.angular_factor = self.get_parameter('angular_correction_factor').value

        wheelbase = self.get_parameter('wheelbase').value
        track_width = self.get_parameter('track_width').value
        wheel_diameter = self.get_parameter('wheel_diameter').value
        self.mecanum = MecanumChassis(wheelbase, track_width, wheel_diameter)

        # Odometry publisher
        self.odom = Odometry()
        self.odom.header.frame_id = self.odom_frame_id
        self.odom.child_frame_id = self.base_frame_id
        self.odom.pose.covariance = ODOM_POSE_COVARIANCE
        self.odom.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 1)

        # Motor command publisher
        self.motor_pub = self.create_publisher(
            MotorsState, 'car1_base_driver/set_motor', 1
        )

        # Subscribers
        # /cmd_vel — external commands (teleop, direct pub): clamped to ±0.2 for safety
        self.create_subscription(Twist, 'cmd_vel', self.app_cmd_vel_callback, 1)
        # /controller/cmd_vel — internal commands (Nav2 via velocity_smoother): no clamp
        self.create_subscription(Twist, 'controller/cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(Pose2D, 'set_odom', self.set_odom_callback, 1)

        # cmd_vel timeout
        self.last_cmd_time = None
        self.create_timer(0.1, self._check_cmd_timeout)

        # Services
        self.create_service(Trigger, '~/init_finish', self._init_finish_callback)
        self.create_service(Trigger, '~/load_calibrate_param', self._load_calibrate_callback)

        # Odometry integration thread (20 Hz — sufficient for 0.4m/s robot,
        # reduces CPU from ~40% to ~16% on RK3576)
        self.clock = self.get_clock()
        threading.Thread(target=self._odom_loop, daemon=True).start()

        self.get_logger().info('\033[1;32modom_publisher started\033[0m')

    def _init_finish_callback(self, request, response):
        response.success = True
        return response

    def _load_calibrate_callback(self, request, response):
        self.linear_factor = self.get_parameter('linear_correction_factor').value
        self.angular_factor = self.get_parameter('angular_correction_factor').value
        self.get_logger().info(f'Calibration reloaded: linear={self.linear_factor}, angular={self.angular_factor}')
        response.success = True
        return response

    def set_odom_callback(self, msg):
        """Reset odometry to given pose."""
        self.x = msg.x
        self.y = msg.y
        self.pose_yaw = msg.theta
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

    def cmd_vel_callback(self, msg):
        """Handle velocity commands from internal controller (Nav2, no clamp)."""
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z
        self.last_cmd_time = time.time()
        speeds = self.mecanum.set_velocity(self.linear_x, self.linear_y, self.angular_z)
        self.motor_pub.publish(speeds)

    def app_cmd_vel_callback(self, msg):
        """Handle external velocity commands (teleop): adjusted limits for SLAM mapping.

        Linear: ±0.3 m/s for stable mapping
        Angular: ±1.0 rad/s for faster rotation during mapping
        """
        msg.linear.x = max(-0.3, min(0.3, msg.linear.x))
        msg.linear.y = max(-0.3, min(0.3, msg.linear.y))
        msg.angular.z = max(-1.0, min(1.0, msg.angular.z))
        self.cmd_vel_callback(msg)

    def _check_cmd_timeout(self):
        """Stop motors if no cmd_vel received for 0.5 seconds."""
        if self.last_cmd_time is None:
            return
        if time.time() - self.last_cmd_time > 0.5:
            if self.linear_x != 0.0 or self.linear_y != 0.0 or self.angular_z != 0.0:
                self.linear_x = 0.0
                self.linear_y = 0.0
                self.angular_z = 0.0
                stop_cmd = self.mecanum.set_velocity(0.0, 0.0, 0.0)
                self.motor_pub.publish(stop_cmd)

    def _odom_loop(self):
        """Integrate odometry at 20 Hz."""
        while rclpy.ok():
            current_time = time.time()
            if self.last_time is None:
                dt = 0.0
            else:
                dt = current_time - self.last_time

            self.odom.header.stamp = self.clock.now().to_msg()

            delta_x = self.linear_x * dt * math.cos(self.pose_yaw)
            delta_y = self.linear_x * dt * math.sin(self.pose_yaw)
            delta_yaw = self.angular_z * dt * self.angular_factor

            self.x += delta_x
            self.y += delta_y
            self.pose_yaw += delta_yaw

            self.odom.pose.pose.position.x = self.linear_factor * self.x
            self.odom.pose.pose.position.y = self.linear_factor * self.y
            self.odom.pose.pose.orientation = rpy2qua(0.0, 0.0, self.pose_yaw)
            # twist 速度同样乘 correction_factor，保证 odom_raw 的速度与位置一致。
            # 否则 EKF（odom0_config 用 vx/vy 融合）拿到的是未修正命令速度，
            # 积分出的 EKF 位置会偏大，导致 SLAM/导航里程计不准。
            self.odom.twist.twist.linear.x = self.linear_factor * self.linear_x
            self.odom.twist.twist.linear.y = self.linear_factor * self.linear_y
            self.odom.twist.twist.angular.z = self.angular_factor * self.angular_z

            if self.linear_x == 0 and self.linear_y == 0 and self.angular_z == 0:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE_STOP
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE_STOP
            else:
                self.odom.pose.covariance = ODOM_POSE_COVARIANCE
                self.odom.twist.covariance = ODOM_TWIST_COVARIANCE

            self.odom_pub.publish(self.odom)
            self.last_time = current_time
            time.sleep(0.05)

    def destroy_node(self):
        """Send stop command before destroying node."""
        # Stop all motors
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        stop_cmd = self.mecanum.set_velocity(0.0, 0.0, 0.0)
        self.motor_pub.publish(stop_cmd)
        self.get_logger().info('Sent stop command to motors')
        time.sleep(0.1)  # Give time for command to be sent
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
