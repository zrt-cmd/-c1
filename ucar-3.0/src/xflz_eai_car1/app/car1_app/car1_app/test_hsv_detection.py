#!/usr/bin/env python3
# encoding: utf-8
"""
Test HSV detection only (no IK, no grasp).
Publishes annotated image to /detection_debug_image.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class HSVDetectionTest(Node):
    def __init__(self):
        super().__init__('hsv_detection_test')

        # HSV range for red (same as visual_grasp default). Red wraps around H=0,
        # so we ALSO test the high-H end (160-180) to find which side the cube lands on.
        self.declare_parameter('hsv_lower', [0, 100, 80])
        self.declare_parameter('hsv_upper', [10, 255, 255])
        hsv_lower = self.get_parameter('hsv_lower').value
        hsv_upper = self.get_parameter('hsv_upper').value
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        # High-H red (the other half of the red hue circle)
        self.hsv_lower2 = np.array([160, 80, 60], dtype=np.uint8)
        self.hsv_upper2 = np.array([180, 255, 255], dtype=np.uint8)

        self.bridge = CvBridge()

        # Subscribe to RGB
        self.rgb_sub = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.rgb_callback, 10
        )

        # Publish debug image + masks
        self.debug_pub = self.create_publisher(Image, '/detection_debug_image', 10)
        self.mask_lo_pub = self.create_publisher(Image, '/detection_mask_low', 10)
        self.mask_hi_pub = self.create_publisher(Image, '/detection_mask_high', 10)

        self.get_logger().info(f'HSV detection test started. Range: {hsv_lower} - {hsv_upper}')
        self.get_logger().info('View annotated image: ros2 run rqt_image_view rqt_image_view /detection_debug_image')

    def rgb_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # Skip black frames
        if img.mean() < 5.0:
            return

        # HSV detection — test BOTH red ends
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_lo = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask_hi = cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2)
        mask = cv2.bitwise_or(mask_lo, mask_hi)

        # Morphological ops
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Publish both raw masks so you can SEE which red end catches the cube
        self.mask_lo_pub.publish(self.bridge.cv2_to_imgmsg(mask_lo, encoding='mono8'))
        self.mask_hi_pub.publish(self.bridge.cv2_to_imgmsg(mask_hi, encoding='mono8'))

        lo_px = int(mask_lo.sum() / 255)
        hi_px = int(mask_hi.sum() / 255)

        # Find contours on combined mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        debug_img = img.copy()
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)
            cv2.drawContours(debug_img, [largest], -1, (0, 255, 0), 2)

            # Report the ACTUAL HSV of the pixels inside the largest red blob,
            # so we know how to set the threshold correctly.
            cmask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(cmask, [largest], -1, 255, thickness=cv2.FILLED)
            hpx = hsv[cmask > 0]
            if len(hpx) > 0:
                hmin, hmax = int(hpx[:, 0].min()), int(hpx[:, 0].max())
                smin, smax = int(hpx[:, 1].min()), int(hpx[:, 1].max())
                vmin, vmax = int(hpx[:, 2].min()), int(hpx[:, 2].max())
                self.get_logger().info(
                    f'area={area:.0f} lo_px={lo_px} hi_px={hi_px} | blob HSV '
                    f'H[{hmin}-{hmax}] S[{smin}-{smax}] V[{vmin}-{vmax}]',
                    throttle_duration_sec=1.0)
            txt = f'area={area:.0f} lo={lo_px} hi={hi_px}'
            cv2.putText(debug_img, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            self.get_logger().info(f'No red blob. lo_px={lo_px} hi_px={hi_px}',
                                   throttle_duration_sec=1.0)
            cv2.putText(debug_img, f'No object. lo={lo_px} hi={hi_px}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HSVDetectionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
