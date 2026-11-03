"""
Visual tracking + arm grab node for mecanum robot.
"""

import enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from .common import PID

# Try to import servo controller message
try:
    from car1_servo_controller_msgs.msg import ServosPosition
    SERVO_MSG_AVAILABLE = True
except ImportError:
    SERVO_MSG_AVAILABLE = False


class State(enum.Enum):
    """State machine states for track and grab."""
    SEARCH = 0
    APPROACH = 1
    ALIGN = 2
    GRAB = 3
    LIFT = 4
    DONE = 5


class TrackAndGrabNode(Node):
    """ROS2 node for visual tracking and arm grabbing."""

    def __init__(self):
        super().__init__('track_and_grab_node')

        # Declare parameters
        self.declare_parameter('target_color_lab', [0, 0, 0])
        self.declare_parameter('grab_height', 0.05)

        # Get parameters
        target_lab = self.get_parameter('target_color_lab').value
        self.grab_height = self.get_parameter('grab_height').value

        self.target_color_lab = np.array(target_lab, dtype=np.float32)

        # PID controllers
        self.pid_yaw = PID(kp=0.5, ki=0.0, kd=0.1, limit=0.5)
        self.pid_dist = PID(kp=0.3, ki=0.0, kd=0.05, limit=0.2)

        # CV Bridge
        self.bridge = CvBridge()

        # State machine
        self.state = State.SEARCH
        self.search_direction = 1.0  # 1 or -1 for rotation direction

        # Color detection threshold
        self.color_threshold = 30

        # Approach target thresholds
        self.approach_target_radius = 100  # Desired object radius in pixels
        self.align_tolerance_x = 15  # Pixels from center
        self.align_tolerance_radius = 10  # Pixels from target radius
        self.grab_min_depth = 0.05  # Minimum depth to trigger grab (meters)
        self.grab_max_depth = 0.25  # Maximum depth for grab range (meters)

        # Depth data
        self.current_depth = None

        # Timing
        self.last_time = self.get_clock().now()
        self.state_start_time = self.get_clock().now()

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/depth_cam/rgb/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/depth_cam/depth/image_raw',
            self.depth_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'controller/cmd_vel',
            10
        )

        if SERVO_MSG_AVAILABLE:
            self.servo_pub = self.create_publisher(
                ServosPosition,
                'car1_servo_controller/servo_controller',
                10
            )
        else:
            self.servo_pub = None
            self.get_logger().warn(
                'car1_servo_controller_msgs not available. '
                'Servo commands will not be published.'
            )

        # Timer for state machine processing
        self.timer = self.create_timer(0.05, self.state_machine_tick)

        # Detection results (set by rgb_callback)
        self.detected_cx = None
        self.detected_cy = None
        self.detected_radius = None
        self.image_width = 640
        self.image_height = 480

        self.get_logger().info('Track and grab node started.')

    def depth_callback(self, msg: Image):
        """Process incoming depth image."""
        try:
            self.current_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')

    def rgb_callback(self, msg: Image):
        """Process incoming RGB image for object detection."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        self.image_height, self.image_width = cv_image.shape[:2]

        # Convert to LAB color space
        lab_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)

        # Create mask based on target LAB color distance
        target = self.target_color_lab.reshape(1, 1, 3)
        diff = np.linalg.norm(lab_image.astype(np.float32) - target, axis=2)
        mask = (diff < self.color_threshold).astype(np.uint8) * 255

        # Morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)

            if area > 100:
                (cx, cy), radius = cv2.minEnclosingCircle(largest_contour)
                self.detected_cx = cx
                self.detected_cy = cy
                self.detected_radius = radius
                return

        # No detection
        self.detected_cx = None
        self.detected_cy = None
        self.detected_radius = None

    def _get_depth_at_point(self, x: int, y: int) -> float:
        """
        Get depth value at a specific pixel coordinate.

        Args:
            x: Pixel x coordinate
            y: Pixel y coordinate

        Returns:
            Depth in meters, or -1.0 if unavailable
        """
        if self.current_depth is None:
            return -1.0

        h, w = self.current_depth.shape[:2]
        if 0 <= y < h and 0 <= x < w:
            depth_val = self.current_depth[y, x]
            if np.isfinite(depth_val) and depth_val > 0:
                # Convert mm to meters if needed (depends on camera format)
                if depth_val > 100:
                    return depth_val / 1000.0
                return float(depth_val)

        return -1.0

    def _publish_servo_command(self, positions: dict):
        """
        Publish servo position command.

        Args:
            positions: Dictionary of servo_id -> position_value
        """
        if self.servo_pub is None:
            return

        msg = ServosPosition()
        msg.servo_id = list(positions.keys())
        msg.position = list(positions.values())
        self.servo_pub.publish(msg)

    def _change_state(self, new_state: State):
        """Transition to a new state."""
        self.get_logger().info(
            f'State transition: {self.state.name} -> {new_state.name}'
        )
        self.state = new_state
        self.state_start_time = self.get_clock().now()
        self.pid_yaw.reset()
        self.pid_dist.reset()

    def state_machine_tick(self):
        """Main state machine tick."""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0:
            dt = 0.001

        twist = Twist()

        if self.state == State.SEARCH:
            twist = self._handle_search(dt)

        elif self.state == State.APPROACH:
            twist = self._handle_approach(dt)

        elif self.state == State.ALIGN:
            twist = self._handle_align(dt)

        elif self.state == State.GRAB:
            self._handle_grab()
            return

        elif self.state == State.LIFT:
            self._handle_lift()
            return

        elif self.state == State.DONE:
            twist = Twist()  # Stop

        self.cmd_vel_pub.publish(twist)

    def _handle_search(self, dt: float) -> Twist:
        """Search for the target object by rotating."""
        twist = Twist()

        if self.detected_cx is not None:
            self._change_state(State.APPROACH)
            return twist

        # Rotate to search for object
        twist.angular.z = 0.3 * self.search_direction

        # Reverse search direction periodically
        elapsed = (
            self.get_clock().now() - self.state_start_time
        ).nanoseconds / 1e9
        if elapsed > 6.0:
            self.search_direction *= -1
            self.state_start_time = self.get_clock().now()

        return twist

    def _handle_approach(self, dt: float) -> Twist:
        """Approach the detected object."""
        twist = Twist()

        if self.detected_cx is None:
            # Lost target, go back to search
            self._change_state(State.SEARCH)
            return twist

        cx = self.detected_cx
        radius = self.detected_radius
        center_x = self.image_width / 2.0

        # Yaw control
        error_yaw = (cx - center_x) / center_x
        angular_z = -self.pid_yaw.update(error_yaw, dt)

        # Distance control based on object radius
        error_dist = (
            (self.approach_target_radius - radius) / self.approach_target_radius
        )
        linear_x = self.pid_dist.update(error_dist, dt)

        twist.linear.x = linear_x
        twist.angular.z = angular_z

        # Check if close enough to align
        if (abs(cx - center_x) < self.align_tolerance_x and
                abs(radius - self.approach_target_radius) < self.align_tolerance_radius):
            self._change_state(State.ALIGN)

        return twist

    def _handle_align(self, dt: float) -> Twist:
        """Fine alignment before grabbing."""
        twist = Twist()

        if self.detected_cx is None:
            self._change_state(State.SEARCH)
            return twist

        cx = self.detected_cx
        cy = self.detected_cy
        center_x = self.image_width / 2.0
        center_y = self.image_height / 2.0

        # Fine yaw alignment
        error_x = (cx - center_x) / center_x
        angular_z = -self.pid_yaw.update(error_x, dt)

        # Fine forward/backward alignment using y-position
        error_y = (cy - center_y * 1.2) / center_y  # Target slightly below center
        linear_x = -self.pid_dist.update(error_y, dt) * 0.5

        twist.linear.x = linear_x
        twist.angular.z = angular_z

        # Check depth for grab readiness
        if self.detected_cx is not None and self.detected_cy is not None:
            depth = self._get_depth_at_point(
                int(self.detected_cx), int(self.detected_cy)
            )

            if (self.grab_min_depth < depth < self.grab_max_depth and
                    abs(error_x) < 0.05):
                self._change_state(State.GRAB)
                return Twist()  # Stop before grab

        # Timeout for alignment
        elapsed = (
            self.get_clock().now() - self.state_start_time
        ).nanoseconds / 1e9
        if elapsed > 10.0:
            self.get_logger().warn('Alignment timeout, attempting grab anyway.')
            self._change_state(State.GRAB)
            return Twist()

        return twist

    def _handle_grab(self):
        """Execute grab sequence using servos."""
        # Stop robot
        self.cmd_vel_pub.publish(Twist())

        if SERVO_MSG_AVAILABLE and self.servo_pub is not None:
            # Open gripper
            self.get_logger().info('Opening gripper...')
            self._publish_servo_command({1: 500})  # Gripper open position

            # Move arm down to grab height
            self.get_logger().info('Lowering arm...')
            self._publish_servo_command({
                2: 300,  # Shoulder
                3: 600,  # Elbow
            })

            # Close gripper
            self.get_logger().info('Closing gripper...')
            self._publish_servo_command({1: 150})  # Gripper closed position
        else:
            self.get_logger().warn(
                'Servo messages not available, skipping physical grab.'
            )

        self._change_state(State.LIFT)

    def _handle_lift(self):
        """Lift the grabbed object."""
        # Stop robot
        self.cmd_vel_pub.publish(Twist())

        if SERVO_MSG_AVAILABLE and self.servo_pub is not None:
            # Lift arm up
            self.get_logger().info('Lifting arm...')
            self._publish_servo_command({
                2: 500,  # Shoulder up
                3: 300,  # Elbow up
            })
        else:
            self.get_logger().warn(
                'Servo messages not available, skipping physical lift.'
            )

        self._change_state(State.DONE)
        self.get_logger().info('Grab sequence complete.')


def main(args=None):
    rclpy.init(args=args)
    node = TrackAndGrabNode()
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
