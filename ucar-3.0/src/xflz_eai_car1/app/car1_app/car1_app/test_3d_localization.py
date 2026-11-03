#!/usr/bin/env python3
# encoding: utf-8
"""
Test 3D localization (HSV detection + depth + TF) WITHOUT moving the arm.

Use this to verify camera → base_link coordinate accuracy by:
1. Placing the red cube at known positions (measured with a ruler).
2. Running this node and reading the printed (x, y, z) in base_link.
3. Comparing detected vs actual coordinates to compute systematic errors.

Pre-condition: arm must be at observation pose (z>=0.20, looking down).
You can use visual_grasp's MOVE_TO_OBSERVE step or manually publish a pose.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class Localization3DTest(Node):
    def __init__(self):
        super().__init__('localization_3d_test')

        self.declare_parameter('hsv_lower', [0, 100, 80])
        self.declare_parameter('hsv_upper', [10, 255, 255])
        self.declare_parameter('print_period', 1.0)  # Print at 1 Hz
        hsv_lower = self.get_parameter('hsv_lower').value
        hsv_upper = self.get_parameter('hsv_upper').value
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        self.print_period = self.get_parameter('print_period').value

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rgb_image = None
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.rgb_callback, 10)
        self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, '/ascamera_hp60c/camera_publisher/depth0/camera_info',
            self.info_callback, 10)

        self.create_timer(self.print_period, self.tick)
        self.get_logger().info('3D localization test started.')
        self.get_logger().info(
            'Place red cube at known position (measure with ruler), watch printed coords.')

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conv failed: {e}')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conv failed: {e}')

    def info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}')

    def tick(self):
        if self.rgb_image is None or self.depth_image is None or self.fx is None:
            self.get_logger().warn('Waiting for camera data...')
            return

        if self.rgb_image.mean() < 5.0:
            return

        # HSV detection
        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().info('  [no detection]')
            return

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 100:
            self.get_logger().info(f'  [area too small: {area:.0f}]')
            return

        (u, v), _ = cv2.minEnclosingCircle(largest)

        # Sample depth across whole contour
        h, w = self.depth_image.shape[:2]
        cmask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(cmask, [largest], -1, 255, thickness=cv2.FILLED)
        valid_depths = self.depth_image[(cmask > 0) & (self.depth_image > 0)]

        if len(valid_depths) == 0:
            self.get_logger().warn(
                f'  [no valid depth in contour, area={area:.0f}, '
                f'total contour pixels={int(cmask.sum()/255)}]')
            return

        z_cam_raw = float(np.median(valid_depths))
        z_cam = z_cam_raw / 1000.0 if z_cam_raw > 100 else z_cam_raw

        # Camera frame coords
        x_cam = (u - self.cx) * z_cam / self.fx
        y_cam = (v - self.cy) * z_cam / self.fy

        # TF to base_link
        try:
            pt = PointStamped()
            pt.header.frame_id = 'camera_depth_optical_frame'
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.point.x = x_cam
            pt.point.y = y_cam
            pt.point.z = z_cam

            tf = self.tf_buffer.lookup_transform(
                'base_link', 'camera_depth_optical_frame',
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
            pt_base = do_transform_point(pt, tf)

            depth_valid_pct = 100.0 * len(valid_depths) / max(int(cmask.sum() / 255), 1)
            self.get_logger().info(
                f'pixel=({u:.0f},{v:.0f}) area={area:.0f} '
                f'depth_valid={depth_valid_pct:.0f}% '
                f'cam=[{x_cam:+.3f},{y_cam:+.3f},{z_cam:+.3f}] '
                f'base=[{pt_base.point.x:+.3f},{pt_base.point.y:+.3f},{pt_base.point.z:+.3f}]')
        except Exception as e:
            self.get_logger().error(f'TF failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = Localization3DTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
