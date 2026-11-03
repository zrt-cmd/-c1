"""
Hand gesture control node for mecanum robot using MediaPipe.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

try:
    import mediapipe as mp
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    MEDIAPIPE_AVAILABLE = False


class HandGestureNode(Node):
    """ROS2 node for hand gesture control of mecanum robot."""

    def __init__(self):
        super().__init__('hand_gesture_node')

        if not MEDIAPIPE_AVAILABLE:
            self.get_logger().error(
                'mediapipe is not installed. '
                'Install it with: pip install mediapipe'
            )
            raise SystemExit(1)

        # Declare parameters
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.3)

        # Get parameters
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # CV Bridge
        self.bridge = CvBridge()

        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )

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

        self.get_logger().info('Hand gesture control node started.')

    def _detect_gesture(self, hand_landmarks) -> str:
        """
        Determine gesture from hand landmarks.

        Detects: 'up', 'down', 'left', 'right', 'fist' (stop).

        Args:
            hand_landmarks: MediaPipe hand landmarks

        Returns:
            Gesture string
        """
        landmarks = hand_landmarks.landmark

        # Fingertip and base landmark indices
        fingertip_ids = [
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP,
        ]
        finger_pip_ids = [
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP,
        ]

        # Count extended fingers (tip above pip in y)
        fingers_up = 0
        for tip_id, pip_id in zip(fingertip_ids, finger_pip_ids):
            if landmarks[tip_id].y < landmarks[pip_id].y:
                fingers_up += 1

        # Check thumb
        thumb_tip = landmarks[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = landmarks[self.mp_hands.HandLandmark.THUMB_IP]
        thumb_extended = abs(thumb_tip.x - thumb_ip.x) > 0.04

        # Fist detection: no fingers extended
        if fingers_up == 0 and not thumb_extended:
            return 'fist'

        # If index finger is the only one up, use its direction
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_mcp = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]

        dx = index_tip.x - index_mcp.x
        dy = index_tip.y - index_mcp.y

        angle = math.atan2(-dy, dx)  # Negate dy because y-axis is inverted
        angle_deg = math.degrees(angle)

        # Determine direction based on angle
        if 45 <= angle_deg <= 135:
            return 'up'
        elif -135 <= angle_deg <= -45:
            return 'down'
        elif -45 < angle_deg < 45:
            return 'right'
        else:
            return 'left'

    def image_callback(self, msg: Image):
        """Process incoming camera image for hand gesture detection."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Convert BGR to RGB for MediaPipe
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Process with MediaPipe
        results = self.hands.process(rgb_image)

        twist = Twist()

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            gesture = self._detect_gesture(hand_landmarks)

            if gesture == 'up':
                twist.linear.x = self.linear_speed
            elif gesture == 'down':
                twist.linear.x = -self.linear_speed
            elif gesture == 'left':
                # Mecanum strafe left
                twist.linear.y = self.linear_speed
            elif gesture == 'right':
                # Mecanum strafe right
                twist.linear.y = -self.linear_speed
            elif gesture == 'fist':
                # Stop
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0

            self.get_logger().debug(f'Gesture detected: {gesture}')

        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        """Clean up MediaPipe resources."""
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    if not MEDIAPIPE_AVAILABLE:
        logger = rclpy.logging.get_logger('hand_gesture_node')
        logger.error(
            'mediapipe is not installed. '
            'Install it with: pip install mediapipe'
        )
        rclpy.shutdown()
        return

    node = HandGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        # Stop the robot
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
