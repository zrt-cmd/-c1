"""
Line following node for mecanum robot using color detection.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from .common import PID


class LineFollowingNode(Node):
    """ROS2 node for line following using color detection."""

    def __init__(self):
        super().__init__('line_following_node')

        # Declare parameters
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('linear_speed', 0.1)
        self.declare_parameter('target_color_lab', [0, 0, 0])

        # Get parameters
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.linear_speed = self.get_parameter('linear_speed').value
        target_lab = self.get_parameter('target_color_lab').value

        self.target_color_lab = np.array(target_lab, dtype=np.float32)

        # Initialize PID controller
        self.pid = PID(kp, ki, kd, limit=1.0)

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

        self.get_logger().info('Line following node started.')

    def image_callback(self, msg: Image):
        """Process incoming camera image for line detection."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        height, width = cv_image.shape[:2]

        # Use lower portion of image as ROI for line detection
        roi_top = int(height * 0.6)
        roi = cv_image[roi_top:height, :]

        # Convert ROI to LAB color space
        lab_image = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)

        # Create mask based on target LAB color distance
        target = self.target_color_lab.reshape(1, 1, 3)
        diff = np.linalg.norm(lab_image.astype(np.float32) - target, axis=2)
        mask = (diff < self.color_threshold).astype(np.uint8) * 255

        # Apply morphological operations to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find centroid of the detected line
        moments = cv2.moments(mask)

        twist = Twist()

        if moments['m00'] > 0:
            cx = int(moments['m10'] / moments['m00'])

            # Calculate error: offset from image center (normalized to -1..1)
            error = (cx - width / 2.0) / (width / 2.0)

            # Calculate dt
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            self.last_time = now

            # PID control for angular velocity
            angular_z = -self.pid.update(error, dt)

            twist.linear.x = self.linear_speed
            twist.angular.z = angular_z
        else:
            # No line detected, stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pid.reset()
            self.last_time = self.get_clock().now()

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingNode()
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
