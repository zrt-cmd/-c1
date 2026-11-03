"""
Color object tracking node for mecanum robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from .common import PID


class ObjectTrackingNode(Node):
    """ROS2 node for color object tracking with dual PID control."""

    def __init__(self):
        super().__init__('object_tracking_node')

        # Declare parameters
        self.declare_parameter('kp_yaw', 0.5)
        self.declare_parameter('kd_yaw', 0.1)
        self.declare_parameter('kp_dist', 0.3)
        self.declare_parameter('target_color_lab', [0, 0, 0])
        self.declare_parameter('target_radius', 80)

        # Get parameters
        kp_yaw = self.get_parameter('kp_yaw').value
        kd_yaw = self.get_parameter('kd_yaw').value
        kp_dist = self.get_parameter('kp_dist').value
        target_lab = self.get_parameter('target_color_lab').value
        self.target_radius = self.get_parameter('target_radius').value

        self.target_color_lab = np.array(target_lab, dtype=np.float32)

        # Dual PID controllers
        self.pid_yaw = PID(kp_yaw, 0.0, kd_yaw, limit=1.0)
        self.pid_dist = PID(kp_dist, 0.0, 0.0, limit=0.3)

        # CV Bridge
        self.bridge = CvBridge()

        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/depth_cam/rgb/image_raw',
            self.image_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'controller/cmd_vel',
            10
        )

        self.last_time = self.get_clock().now()

        # Color detection threshold (LAB space distance)
        self.color_threshold = 30

        self.get_logger().info('Object tracking node started.')

    def image_callback(self, msg: Image):
        """Process incoming camera image for object tracking."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        height, width = cv_image.shape[:2]

        # Convert to LAB color space
        lab_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)

        # Create mask based on target LAB color distance
        target = self.target_color_lab.reshape(1, 1, 3)
        diff = np.linalg.norm(lab_image.astype(np.float32) - target, axis=2)
        mask = (diff < self.color_threshold).astype(np.uint8) * 255

        # Apply morphological operations to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist = Twist()

        if contours:
            # Get the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 100:  # Minimum area threshold
                # Get enclosing circle
                (cx, cy), radius = cv2.minEnclosingCircle(largest_contour)

                # Calculate dt
                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds / 1e9
                self.last_time = now

                # Yaw PID: x-offset from image center (normalized to -1..1)
                error_yaw = (cx - width / 2.0) / (width / 2.0)
                angular_z = -self.pid_yaw.update(error_yaw, dt)

                # Distance PID: object radius vs target radius
                # Positive error means object is too small (too far), move forward
                error_dist = (self.target_radius - radius) / self.target_radius
                linear_x = self.pid_dist.update(error_dist, dt)

                twist.linear.x = linear_x
                twist.angular.z = angular_z
            else:
                # Object too small, stop
                self._reset_pids()
        else:
            # No object detected, stop
            self._reset_pids()

        self.cmd_vel_pub.publish(twist)

    def _reset_pids(self):
        """Reset both PID controllers."""
        self.pid_yaw.reset()
        self.pid_dist.reset()
        self.last_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
