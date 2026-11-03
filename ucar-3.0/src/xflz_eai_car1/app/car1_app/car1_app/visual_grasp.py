#!/usr/bin/env python3
# encoding: utf-8
"""
Visual grasp node for static tabletop grasping.

Uses eye-in-hand camera (mounted on arm_link4) with look-then-move strategy:
1. Move arm to observation pose (elevated, looking down)
2. Detect object via HSV color segmentation
3. Compute 3D position in base_link via depth + TF
4. Execute pick via arm_controller
"""
import enum
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class State(enum.Enum):
    """State machine for visual grasp."""
    IDLE = 0             # Waiting for /visual_grasp/start service call
    INIT = 1
    MOVE_TO_OBSERVE = 2
    WAIT_STABLE = 3
    REFRESH_JOINTS = 4   # Force servo readback so TF reflects real joint angles
    DETECT = 5
    WAIT_CONFIRMATION = 6  # Pause after detection for user review
    COMPUTE_TARGET = 7
    PICK = 8
    DONE = 9
    FAILED = 10


class VisualGraspNode(Node):
    """ROS2 node for visual-guided static grasping."""

    def __init__(self):
        super().__init__('visual_grasp_node')

        # Declare parameters
        # observe_pose pitch convention (matches arm_controller/ik_analytical):
        #   0=up, pi/2=horizontal, pi=down. Observation should look down at table.
        self.declare_parameter('observe_pose', [0.20, 0.0, 0.22, 3.14159])
        # Red wraps around the HSV hue circle, so we need TWO ranges OR'd together:
        # low-H end (0-10) and high-H end (160-180). Measured cube is mostly high-H
        # (H 167-178, S down to 80), so the second range is what actually catches it.
        self.declare_parameter('target_color_hsv_lower', [0, 80, 60])
        self.declare_parameter('target_color_hsv_upper', [10, 255, 255])
        self.declare_parameter('target_color_hsv_lower2', [160, 80, 60])
        self.declare_parameter('target_color_hsv_upper2', [180, 255, 255])
        self.declare_parameter('max_detection_attempts', 5)
        # TF frames for localization. The detected pixel is back-projected into the
        # depth optical frame (standard optical convention: x-right, y-down, z-forward),
        # then transformed into base_link via the live TF tree (camera is eye-in-hand on
        # arm_link4, so this transform changes with the arm pose -- it must be looked up,
        # not hard-coded).
        self.declare_parameter('camera_optical_frame', 'camera_depth_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        # dry_run: detect and print base_link coords, then STOP. Does NOT publish the
        # detected target_pose (which would trigger arm_controller to move) and does NOT
        # call pick. Use this to verify localization accuracy safely (arm only goes UP to
        # the observe pose, never descends toward the object).
        self.declare_parameter('dry_run', False)

        # Get parameters
        observe_pose = self.get_parameter('observe_pose').value
        self.observe_x, self.observe_y, self.observe_z, self.observe_pitch = observe_pose
        hsv_lower = self.get_parameter('target_color_hsv_lower').value
        hsv_upper = self.get_parameter('target_color_hsv_upper').value
        self.hsv_lower = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper = np.array(hsv_upper, dtype=np.uint8)
        hsv_lower2 = self.get_parameter('target_color_hsv_lower2').value
        hsv_upper2 = self.get_parameter('target_color_hsv_upper2').value
        self.hsv_lower2 = np.array(hsv_lower2, dtype=np.uint8)
        self.hsv_upper2 = np.array(hsv_upper2, dtype=np.uint8)
        self.max_attempts = self.get_parameter('max_detection_attempts').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.dry_run = self.get_parameter('dry_run').value

        # Wait for user confirmation after detection (for debugging)
        self.wait_for_confirmation = self.declare_parameter('wait_for_confirmation', True).value

        # Visualization (OpenCV windows)
        self.enable_viz = self.declare_parameter('enable_viz', True).value

        # CV Bridge
        self.bridge = CvBridge()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State machine
        # Start in IDLE — waits for /visual_grasp/start service call to begin grasp.
        # This makes the node long-running: after DONE/FAILED it returns to IDLE
        # so the user can trigger another grasp without restarting the node.
        self.state = State.IDLE
        self.state_start_time = self.get_clock().now()
        self.detection_attempts = 0

        # Camera data
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Detection result
        self.detected_point_base = None

        # Latest detection data (for visualization)
        self._last_detection = None
        self._last_tf_stamp = None

        # Latest joint states (for visualization)
        self._latest_joint_states = None

        # Pick service future (async)
        self._pick_future = None

        # Subscribers
        # NOTE: subscribe to the ascamera driver's NATIVE topic names. The driver
        # gates streaming on the "rgb0"/"depth0" substring in the topic name, so
        # these must NOT be remapped. See depth_camera.launch.py for details.
        self.rgb_sub = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/rgb0/image', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/ascamera_hp60c/camera_publisher/depth0/image_raw', self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/ascamera_hp60c/camera_publisher/depth0/camera_info', self.camera_info_callback, 10
        )

        # Subscribe to joint_states for visualization
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_states_callback, 10
        )

        # Publisher for target pose
        self.target_pose_pub = self.create_publisher(
            PoseStamped, '/arm_controller/target_pose', 10
        )
        # set_pick_target: tells arm_controller the grasp target WITHOUT moving the
        # arm. Using target_pose here would make the arm descend to the object before
        # the pick service runs, which looks like a redundant "first grasp".
        self.set_pick_target_pub = self.create_publisher(
            PoseStamped, '/arm_controller/set_pick_target', 10
        )

        # Debug image publishers: view in rqt to SEE what the camera sees and what
        # HSV catches. /visual_grasp/debug_image = RGB with detected contour drawn;
        # /visual_grasp/mask = raw HSV mask (white = pixels matching the color range).
        self.debug_image_pub = self.create_publisher(Image, '/visual_grasp/debug_image', 1)
        self.mask_pub = self.create_publisher(Image, '/visual_grasp/mask', 1)

        # Visualization publisher: composite image (RGB + Depth + Mask + annotations)
        self.viz_pub = self.create_publisher(Image, '/visual_grasp/visualization', 1)

        # Clients that we wait on inside timer callbacks MUST use a Reentrant
        # callback group, otherwise their responses can't be delivered while
        # the caller (a timer callback) is spinning-wait for the future.
        self._client_cbg = ReentrantCallbackGroup()

        # Service client for pick
        self.pick_client = self.create_client(
            Trigger, '/arm_controller/pick', callback_group=self._client_cbg)

        # Service client for on-demand servo readback. joint_states normally
        # reflects commanded values (fast, avoids serial bus contention); we
        # call this AFTER the arm has settled at the observation pose so TF
        # reflects the real joint angles before we detect the object.
        self.refresh_client = self.create_client(
            Trigger, '/servo_controller/refresh_positions',
            callback_group=self._client_cbg)
        self._refresh_future = None
        self._refresh_attempts = 0
        self._refresh_max_attempts = 3

        # User action state for WAIT_CONFIRMATION (triggered via ROS services)
        self._user_action = None
        # User-facing services must use a ReentrantCallbackGroup on a separate
        # thread from the state machine timer. Without this, ROS 2's default
        # MutuallyExclusiveCallbackGroup would serialize the service handler
        # BEHIND the running timer callback — and since the timer callback
        # calls _refresh_joints_sync (which spins for up to 1.5s), the user's
        # service call would visibly hang and never trigger the state change.
        self._user_cbg = ReentrantCallbackGroup()
        self.create_service(
            Trigger, '/visual_grasp/start', self._start_callback,
            callback_group=self._user_cbg)
        self.create_service(
            Trigger, '/visual_grasp/confirm', self._confirm_callback,
            callback_group=self._user_cbg)
        self.create_service(
            Trigger, '/visual_grasp/skip', self._skip_callback,
            callback_group=self._user_cbg)
        self.create_service(
            Trigger, '/visual_grasp/abort', self._abort_callback,
            callback_group=self._user_cbg)

        # Timer for state machine (10 Hz)
        self.timer = self.create_timer(0.1, self.state_machine_tick)

        # Visualization info (no OpenCV window needed — publishes to ROS topic)
        if self.enable_viz:
            self.get_logger().info('Visualization enabled (publishing to /visual_grasp/visualization)')

        self.get_logger().info('Visual grasp node started.')
        self.get_logger().info(f'Observation pose: [{self.observe_x}, {self.observe_y}, {self.observe_z}, {self.observe_pitch:.3f}]')
        self.get_logger().info(f'HSV range: {hsv_lower} - {hsv_upper}')

    def _joint_states_callback(self, msg):
        """Cache latest joint states for visualization."""
        self._latest_joint_states = msg

    def rgb_callback(self, msg):
        """Cache RGB image."""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion failed: {e}')

    def depth_callback(self, msg):
        """Cache depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')

    def camera_info_callback(self, msg):
        """Extract camera intrinsics."""
        if self.camera_info is None:
            self.camera_info = msg
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(f'Camera intrinsics: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')

    def state_machine_tick(self):
        """Main state machine loop."""
        if self.state == State.IDLE:
            self._handle_idle()
        elif self.state == State.INIT:
            self._handle_init()
        elif self.state == State.MOVE_TO_OBSERVE:
            self._handle_move_to_observe()
        elif self.state == State.WAIT_STABLE:
            self._handle_wait_stable()
        elif self.state == State.REFRESH_JOINTS:
            self._handle_refresh_joints()
        elif self.state == State.DETECT:
            self._handle_detect()
        elif self.state == State.WAIT_CONFIRMATION:
            self._handle_wait_confirmation()
        elif self.state == State.COMPUTE_TARGET:
            self._handle_compute_target()
        elif self.state == State.PICK:
            self._handle_pick()
        elif self.state == State.DONE:
            self._handle_done()
        elif self.state == State.FAILED:
            self._handle_failed()

        # Publish visualization continuously (every tick, ~10Hz)
        if self.enable_viz:
            self._update_visualization_continuous()

    def _format_joint_angles(self):
        """Format current joint angles from latest joint_states."""
        if self._latest_joint_states is None:
            return "N/A"

        js = self._latest_joint_states
        arm_joints = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        angles = []
        for jn in arm_joints:
            if jn in js.name:
                idx = js.name.index(jn)
                angles.append(f"{math.degrees(js.position[idx]):+.0f}°")
            else:
                angles.append("?")

        return ", ".join(angles)

    def _update_visualization_continuous(self):
        """
        Continuously publish visualization (called every tick).
        Shows current camera view even if detection hasn't succeeded yet.
        """
        if self.rgb_image is None:
            return  # Need at least RGB

        try:
            h, w = self.rgb_image.shape[:2]

            # === Column 1: RGB ===
            rgb_viz = self.rgb_image.copy()
            cv2.putText(rgb_viz, "RGB", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # If we have a detection, overlay it with actual contour + bounding box
            if self._last_detection is not None:
                u, v = self._last_detection['centroid']
                contour = self._last_detection.get('contour')
                bbox = self._last_detection.get('bbox')

                # Draw actual detected contour (green outline of the red cube)
                if contour is not None:
                    cv2.drawContours(rgb_viz, [contour], -1, (0, 255, 0), 2)

                # Draw bounding box (yellow rectangle around detected region)
                if bbox is not None:
                    bx, by, bw, bh = bbox
                    cv2.rectangle(rgb_viz, (bx, by), (bx+bw, by+bh), (0, 255, 255), 2)

                # Draw large red cross at centroid (grasp point)
                cv2.drawMarker(rgb_viz, (u, v), (0, 0, 255),
                               cv2.MARKER_CROSS, 30, 3)
                # Label it
                cv2.putText(rgb_viz, "TARGET", (u+20, v-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # === Column 2: Depth ===
            if self.depth_image is not None:
                depth_normalized = cv2.normalize(self.depth_image, None, 0, 255,
                                                  cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_color = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                cv2.putText(depth_color, "Depth", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                if self._last_detection is not None:
                    u, v = self._last_detection['centroid']
                    depth_z = self._last_detection['depth_z']
                    contour = self._last_detection.get('contour')
                    bbox = self._last_detection.get('bbox')

                    # Draw actual detected contour (white outline)
                    if contour is not None:
                        cv2.drawContours(depth_color, [contour], -1, (255, 255, 255), 2)

                    # Draw bounding box
                    if bbox is not None:
                        bx, by, bw, bh = bbox
                        cv2.rectangle(depth_color, (bx, by), (bx+bw, by+bh), (0, 255, 255), 2)

                    # Draw large white cross at centroid
                    cv2.drawMarker(depth_color, (u, v), (255, 255, 255),
                                   cv2.MARKER_CROSS, 30, 3)
                    # Depth value label
                    cv2.putText(depth_color, f"{depth_z*1000:.0f}mm",
                                (u+20, v-25), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (255, 255, 0), 2)
            else:
                # No depth image yet, show placeholder
                depth_color = np.zeros((h, w, 3), dtype=np.uint8)
                cv2.putText(depth_color, "Waiting for depth...", (10, h//2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (128, 128, 128), 2)

            # === Column 3: HSV Mask ===
            # Generate mask on-the-fly (even if detection hasn't run yet)
            hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, self.hsv_lower, self.hsv_upper),
                cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2),
            )
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            cv2.putText(mask_color, "HSV Mask", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            if self._last_detection is not None:
                u, v = self._last_detection['centroid']
                # Draw large green cross
                cv2.drawMarker(mask_color, (u, v), (0, 255, 0),
                               cv2.MARKER_CROSS, 30, 3)
                # Draw circle
                cv2.circle(mask_color, (u, v), 15, (255, 255, 0), 2)

            # === Resize and stack ===
            scale = 0.666
            new_h, new_w = int(h * scale), int(w * scale)
            rgb_small = cv2.resize(rgb_viz, (new_w, new_h))
            depth_small = cv2.resize(depth_color, (new_w, new_h))
            mask_small = cv2.resize(mask_color, (new_w, new_h))
            combined = np.hstack([rgb_small, depth_small, mask_small])

            # === Top info bar ===
            info_bar = np.zeros((60, combined.shape[1], 3), dtype=np.uint8)
            if self._last_detection is not None:
                u, v = self._last_detection['centroid']
                depth_z = self._last_detection['depth_z']
                info_text = (f"State: {self.state.name}  |  "
                             f"Centroid: ({u}, {v}) px  |  "
                             f"Depth: {depth_z*1000:.1f} mm")
            else:
                info_text = f"State: {self.state.name}  |  No detection yet"
            cv2.putText(info_bar, info_text, (10, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

            # === Bottom coordinate bar ===
            coord_bar = np.zeros((100, combined.shape[1], 3), dtype=np.uint8)

            if self._last_detection is not None:
                cam_xyz = self._last_detection['cam_xyz']
                base_xyz = self._last_detection['base_xyz']
                joint_str = self._format_joint_angles()

                tf_age_str = "N/A"
                if self._last_tf_stamp is not None:
                    tf_age = (self.get_clock().now().nanoseconds -
                              self._last_tf_stamp.sec * 1e9 -
                              self._last_tf_stamp.nanosec) / 1e9
                    tf_age_str = f"{tf_age:.3f}s"

                coord_lines = [
                    f"Camera: ({cam_xyz[0]:+.3f}, {cam_xyz[1]:+.3f}, {cam_xyz[2]:+.3f}) m",
                    f"Base:   ({base_xyz[0]:+.3f}, {base_xyz[1]:+.3f}, {base_xyz[2]:+.3f}) m",
                    f"Joints: [{joint_str}]  |  TF age: {tf_age_str}",
                ]
            else:
                coord_lines = [
                    "Camera: N/A",
                    "Base:   N/A",
                    "Waiting for detection...",
                ]

            y_off = 25
            for line in coord_lines:
                cv2.putText(coord_bar, line, (10, y_off),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                y_off += 30

            # === Final composite ===
            final = np.vstack([info_bar, combined, coord_bar])

            # Publish
            viz_msg = self.bridge.cv2_to_imgmsg(final, encoding='bgr8')
            self.viz_pub.publish(viz_msg)

        except Exception as e:
            # Log error but don't crash the node
            self.get_logger().error(f'Visualization failed: {e}', throttle_duration_sec=2.0)

    def _update_visualization(self):
        """
        Update OpenCV visualization window (single window, 3-column layout).
        Shows RGB + Depth + HSV Mask side-by-side with detection overlay.
        """
        if self.rgb_image is None or self.depth_image is None or self._last_detection is None:
            return

        centroid = self._last_detection['centroid']
        depth_z = self._last_detection['depth_z']
        cam_xyz = self._last_detection['cam_xyz']
        base_xyz = self._last_detection['base_xyz']
        mask = self._last_detection['mask']

        u, v = centroid
        h, w = self.rgb_image.shape[:2]

        # === Column 1: RGB + Detection ===
        rgb_viz = self.rgb_image.copy()
        cv2.drawMarker(rgb_viz, (u, v), (0, 0, 255),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.putText(rgb_viz, "RGB", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # === Column 2: Depth (colormap) ===
        # Normalize depth to 0-255 for visualization
        depth_normalized = cv2.normalize(self.depth_image, None, 0, 255,
                                          cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_color = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        # Mark same centroid (verifies D2C alignment)
        cv2.drawMarker(depth_color, (u, v), (255, 255, 255),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.putText(depth_color, f"{depth_z*1000:.0f}mm",
                    (u+25, v-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 2)
        cv2.putText(depth_color, "Depth", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # === Column 3: HSV Mask ===
        mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.drawMarker(mask_color, (u, v), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)
        cv2.putText(mask_color, "HSV Mask", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # === Resize columns for smaller display ===
        # Original 640x480 → resize to 426x320 each column (1278 total width)
        scale = 0.666
        new_h, new_w = int(h * scale), int(w * scale)
        rgb_small = cv2.resize(rgb_viz, (new_w, new_h))
        depth_small = cv2.resize(depth_color, (new_w, new_h))
        mask_small = cv2.resize(mask_color, (new_w, new_h))

        # === Horizontal stack ===
        combined = np.hstack([rgb_small, depth_small, mask_small])

        # === Top info bar (60px) ===
        info_bar = np.zeros((60, combined.shape[1], 3), dtype=np.uint8)
        info_text = (f"State: {self.state.name}  |  "
                     f"Centroid: ({u}, {v}) px  |  "
                     f"Depth: {depth_z*1000:.1f} mm")
        cv2.putText(info_bar, info_text, (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        # === Bottom coordinate bar (100px) ===
        coord_bar = np.zeros((100, combined.shape[1], 3), dtype=np.uint8)

        joint_str = self._format_joint_angles()

        # Calculate TF age if available
        tf_age_str = "N/A"
        if self._last_tf_stamp is not None:
            tf_age = (self.get_clock().now().nanoseconds -
                      self._last_tf_stamp.sec * 1e9 -
                      self._last_tf_stamp.nanosec) / 1e9
            tf_age_str = f"{tf_age:.3f}s"

        coord_lines = [
            f"Camera: ({cam_xyz[0]:+.3f}, {cam_xyz[1]:+.3f}, {cam_xyz[2]:+.3f}) m",
            f"Base:   ({base_xyz[0]:+.3f}, {base_xyz[1]:+.3f}, {base_xyz[2]:+.3f}) m",
            f"Joints: [{joint_str}]  |  TF age: {tf_age_str}",
        ]

        y_off = 25
        for line in coord_lines:
            cv2.putText(coord_bar, line, (10, y_off),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            y_off += 30

        # === Final composite ===
        final = np.vstack([info_bar, combined, coord_bar])

        # Publish as ROS Image message (instead of cv2.imshow)
        try:
            viz_msg = self.bridge.cv2_to_imgmsg(final, encoding='bgr8')
            self.viz_pub.publish(viz_msg)
        except Exception as e:
            self.get_logger().warn(f'Visualization publish failed: {e}')

    def _change_state(self, new_state):
        """Transition to new state."""
        self.get_logger().info(f'State: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_start_time = self.get_clock().now()

    def _handle_init(self):
        """Initial state: wait for camera ready."""
        if self.camera_info is not None:
            self.get_logger().info('Camera ready, moving to observation pose...')
            self._change_state(State.MOVE_TO_OBSERVE)

    def _handle_move_to_observe(self):
        """Move arm to observation pose using go_home service (safer than direct pose)."""
        # Instead of publishing observation pose via target_pose (which sets pick_target),
        # we should use a dedicated service or wait for arm to be manually positioned.
        # For now, we publish but note this will set pick_target to observation pose.
        # TODO: Add a dedicated service for non-grasp movements, or use joint_command.

        # Publish observation pose
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = self.observe_x
        pose_msg.pose.position.y = self.observe_y
        pose_msg.pose.position.z = self.observe_z

        # Convert pitch to quaternion (rotation around Y axis)
        # pitch > 0 means looking down
        qw = math.cos(self.observe_pitch / 2)
        qx = 0.0
        qy = math.sin(self.observe_pitch / 2)
        qz = 0.0
        pose_msg.pose.orientation.w = qw
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz

        self.target_pose_pub.publish(pose_msg)
        self.get_logger().info(f'Moving to observation pose: x={self.observe_x}, y={self.observe_y}, z={self.observe_z}, pitch={math.degrees(self.observe_pitch):.1f}°')
        self.get_logger().warn('Note: This sets pick_target. Real grasp target will be set after detection.')

        self._change_state(State.WAIT_STABLE)

    def _handle_wait_stable(self):
        """Wait for arm to reach observation pose and camera to stabilize."""
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        if elapsed > 5.0:  # Increased from 3s to 5s for camera auto-exposure/white-balance
            self.get_logger().info('Arm settled, refreshing servo positions...')
            self._refresh_future = None
            self._refresh_attempts = 0
            self._change_state(State.REFRESH_JOINTS)

    def _handle_refresh_joints(self):
        """Force servo readback so TF reflects real angles before detection.

        joint_states normally shows the commanded position. If a servo didn't
        actually reach its target (blocked, torque limit, wrong IK solution)
        the commanded values would give us a wrong TF and the object localisation
        would be silently off. Refreshing here catches that.

        Retries up to _refresh_max_attempts; on final failure we abort — better
        to fail visibly than to grasp based on a wrong TF.
        """
        # First call: kick off async request
        if self._refresh_future is None:
            if not self.refresh_client.service_is_ready():
                # Give discovery a moment; treat as one failed attempt after 2s.
                elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
                if elapsed > 2.0:
                    self.get_logger().error(
                        '/servo_controller/refresh_positions not available — TF may be stale')
                    self._change_state(State.FAILED)
                return
            self._refresh_attempts += 1
            self.get_logger().info(
                f'refresh_positions attempt {self._refresh_attempts}/{self._refresh_max_attempts}')
            self._refresh_future = self.refresh_client.call_async(Trigger.Request())
            self._refresh_deadline = self.get_clock().now().nanoseconds / 1e9 + 12.0
            return

        # Waiting for response
        if not self._refresh_future.done():
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec > self._refresh_deadline:
                self.get_logger().warn('refresh_positions client-side timeout')
                self._refresh_future.cancel()
                self._refresh_future = None
                if self._refresh_attempts >= self._refresh_max_attempts:
                    self.get_logger().error(
                        'refresh_positions failed after retries; aborting grasp')
                    self._change_state(State.FAILED)
            return

        # Response arrived
        try:
            resp = self._refresh_future.result()
        except Exception as e:
            self.get_logger().warn(f'refresh_positions raised: {e}')
            resp = None
        self._refresh_future = None

        if resp and resp.success:
            self.get_logger().info(f'refresh OK: {resp.message}')
            # Give robot_state_publisher one tick to consume the fresh joint_states
            # and update TF before we sample it in DETECT.
            time.sleep(0.3)
            self.detection_attempts = 0
            self._change_state(State.DETECT)
            return

        # Partial / total failure
        msg = resp.message if resp else 'no response'
        self.get_logger().warn(f'refresh_positions not fully successful: {msg}')
        if self._refresh_attempts >= self._refresh_max_attempts:
            self.get_logger().error(
                f'refresh_positions failed {self._refresh_max_attempts} times; aborting')
            self._change_state(State.FAILED)
        # else: fall through, next tick retries by leaving self._refresh_future = None

    def _handle_detect(self):
        """Detect object via HSV and depth."""
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().warn('Waiting for camera images...')
            return

        result = self._detect_and_localize()
        if result is not None:
            self.detected_point_base = result
            self.get_logger().info(f'Object detected at base_link: [{result[0]:.3f}, {result[1]:.3f}, {result[2]:.3f}]')

            # Visualization is handled by _update_visualization_continuous (called every tick)
            # No need to call it here explicitly

            if self.dry_run:
                self.get_logger().warn(
                    '[DRY_RUN] Localization only. Measure the cube with a ruler and compare to '
                    f'the base_link coords above [{result[0]:.3f}, {result[1]:.3f}, {result[2]:.3f}] '
                    '(target <1.5cm). NOT publishing target_pose, NOT calling pick. Stopping.')
                self._change_state(State.DONE)
            elif self.wait_for_confirmation:
                # Pause for user to review visualization
                self._change_state(State.WAIT_CONFIRMATION)
            else:
                self._change_state(State.COMPUTE_TARGET)
        else:
            self.detection_attempts += 1
            if self.detection_attempts >= self.max_attempts:
                self.get_logger().error(f'Detection failed after {self.max_attempts} attempts')
                self._change_state(State.FAILED)
            else:
                self.get_logger().warn(f'Detection attempt {self.detection_attempts}/{self.max_attempts} failed, retrying...')

    def _publish_mask(self, mask):
        """Publish the HSV mask as a mono8 image for rqt inspection."""
        try:
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding='mono8'))
        except Exception as e:
            self.get_logger().error(f'mask publish failed: {e}')

    def _publish_debug_image(self, rgb, contour, center=None):
        """Publish RGB with the detected contour + center drawn, for rqt inspection."""
        try:
            img = rgb.copy()
            if contour is not None:
                cv2.drawContours(img, [contour], -1, (0, 255, 0), 2)
            if center is not None:
                cv2.circle(img, center, 6, (255, 0, 0), -1)
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))
        except Exception as e:
            self.get_logger().error(f'debug image publish failed: {e}')

    def _refresh_joints_sync(self):
        """
        Synchronously refresh servo positions before TF lookup.

        Uses non-blocking call and waits with rclpy.spin_once() to keep executor alive.
        Timeout: 1.5s per full 6-joint refresh. If timeout, use stale cache (log warning).

        This is called from within _detect_and_localize (a timer callback), so we must
        NOT block the executor with spin_until_future_complete (would deadlock).
        Instead, we do a short bounded wait — if it doesn't return in time, the previous
        joint_states / TF are used, which is a graceful degradation.
        """
        if not self.refresh_client.service_is_ready():
            self.get_logger().warn(
                'refresh_positions service not ready, TF may be stale',
                throttle_duration_sec=5.0)
            return

        # Fire the request
        future = self.refresh_client.call_async(Trigger.Request())

        # Wait up to 1.5s for response, spinning the node so callbacks can run
        deadline = self.get_clock().now().nanoseconds / 1e9 + 1.5
        while self.get_clock().now().nanoseconds / 1e9 < deadline:
            if future.done():
                break
            time.sleep(0.02)

        if not future.done():
            self.get_logger().warn(
                'refresh_positions timed out — using stale TF',
                throttle_duration_sec=5.0)
            return

        try:
            resp = future.result()
            if resp and resp.success:
                self.get_logger().debug(f'refresh OK: {resp.message}')
            else:
                self.get_logger().warn(
                    f'refresh partial: {resp.message if resp else "no response"}',
                    throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().warn(f'refresh raised: {e}', throttle_duration_sec=5.0)

        # Give robot_state_publisher a moment to consume the new joint_states
        # and update TF (typically 30-50ms is enough)
        time.sleep(0.1)

    def _detect_and_localize(self):
        """
        Detect object and compute 3D position in base_link.
        Returns (x, y, z) in base_link or None if detection fails.
        """
        # Skip black frames (camera's first frame after stream start is all-black,
        # and auto-exposure may need a frame to ramp up).
        if self.rgb_image.mean() < 5.0:
            self.get_logger().warn('RGB frame too dark (likely first frame), skipping')
            return None

        # HSV detection — OR the two red ranges (red wraps around H=0/180)
        hsv = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.hsv_lower, self.hsv_upper),
            cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2),
        )

        # Morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Publish the raw mask so we can SEE in rqt what HSV catches (white pixels).
        self._publish_mask(mask)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn(f'HSV found NO red contour (mask white px={int(mask.sum()/255)})')
            self._publish_debug_image(self.rgb_image, None)
            return None

        # Debug: print all contours
        if len(contours) > 1:
            areas = [cv2.contourArea(c) for c in contours]
            self.get_logger().info(f'Found {len(contours)} contours, areas: {[f"{a:.0f}" for a in sorted(areas, reverse=True)[:5]]}')

        # Use largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 100:  # Min area threshold
            self.get_logger().warn(f'Largest red contour too small: area={area:.0f} (<100)')
            self._publish_debug_image(self.rgb_image, largest)
            return None

        # Center pixel = contour CENTROID (image moments), which is more stable than
        # minEnclosingCircle center for irregular blobs and sits on the object body.
        M = cv2.moments(largest)
        if M['m00'] <= 0:
            self.get_logger().warn('Contour has zero area moment, skipping')
            return None
        u = M['m10'] / M['m00']
        v = M['m01'] / M['m00']
        u_int, v_int = int(round(u)), int(round(v))

        # Debug: print bounding box to understand contour position
        x, y, w_box, h_box = cv2.boundingRect(largest)
        self.get_logger().info(f'Contour: area={area:.0f}, bbox=[x:{x}, y:{y}, w:{w_box}, h:{h_box}], centroid=({u_int},{v_int})')

        # Annotated debug image: RGB with detected contour + centroid marked.
        self._publish_debug_image(self.rgb_image, largest, (u_int, v_int))

        # --- Depth sampling ---
        # Sample depth ONLY at pixels that are BOTH inside the detected contour AND have
        # valid (non-zero) depth. Using the contour mask (not a fixed window) keeps
        # background/table pixels out of the sample even when the centroid sits near an
        # edge. We then reject outliers via median +/- MAD so a few stray readings
        # (depth bleed at object borders) cannot pull the estimate off the object top.
        h, w = self.depth_image.shape[:2]

        # Build a mask of the contour interior, then intersect with valid depth.
        contour_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(contour_mask, [largest], -1, 255, thickness=cv2.FILLED)
        valid = (contour_mask > 0) & (self.depth_image > 0)
        object_depths = self.depth_image[valid].astype(np.float64)

        # Require a minimum number of valid samples; too few means occlusion / FOV edge
        # and the median would be noise (we saw 22 px give a bad z earlier).
        MIN_DEPTH_SAMPLES = 30
        if object_depths.size < MIN_DEPTH_SAMPLES:
            self.get_logger().warn(
                f'Too few valid depth px inside contour: {object_depths.size} '
                f'(<{MIN_DEPTH_SAMPLES}) at centroid=({u_int},{v_int}), area={area:.0f}. '
                'Likely occlusion or object at depth-FOV edge.')
            return None

        # Robust outlier rejection: keep samples within 2.5*MAD of the median.
        med = np.median(object_depths)
        mad = np.median(np.abs(object_depths - med))
        if mad > 1e-6:
            inliers = object_depths[np.abs(object_depths - med) <= 2.5 * mad]
        else:
            inliers = object_depths
        z_raw = float(np.median(inliers))

        # HP60C publishes depth as uint16 millimeters. Convert by dtype, not by a
        # magnitude guess (the old `if z>100` heuristic breaks for objects <0.1 m).
        if np.issubdtype(self.depth_image.dtype, np.integer):
            z_cam = z_raw / 1000.0
        else:
            z_cam = z_raw  # float depth is already in meters

        # Back-project to camera frame
        x_cam = (u - self.cx) * z_cam / self.fx
        y_cam = (v - self.cy) * z_cam / self.fy

        # Debug: print every intermediate value so we can isolate which stage is wrong
        # (HSV pixel / depth / back-projection / TF).
        self.get_logger().info(
            f'[DETECT] centroid=({u_int},{v_int}) area={area:.0f} '
            f'depth_px={object_depths.size}->inliers={inliers.size} '
            f'z_cam={z_cam:.3f}m | cam_frame=({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f})')

        # Transform the back-projected point from the depth optical frame into base_link
        # using the LIVE TF tree. The camera is eye-in-hand (mounted on arm_link4), so this
        # transform depends on the current arm joint angles and must be looked up, not
        # hard-coded. /joint_states carries servo readback (real angles), so the TF tree
        # reflects the arm's actual pose. If the transform is unavailable, return None so
        # the caller retries -- never fall back to a stale/guessed pose.

        # === Refresh joint positions IMMEDIATELY before TF lookup ===
        # This ensures TF reflects the actual physical arm pose (not the commanded pose,
        # which may differ due to IK inaccuracies or servo settling). Critical for accurate
        # object localization in base_link frame.
        self._refresh_joints_sync()

        try:
            # lookup_transform(target_frame, source_frame) returns the transform FROM source
            # TO target. To transform a point from camera_optical_frame to base_frame:
            #   target_frame = base_frame (where we want the point)
            #   source_frame = camera_optical_frame (where the point currently is)
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,              # target
                self.camera_optical_frame,    # source
                rclpy.time.Time(),  # latest available
                timeout=Duration(seconds=0.2),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF {self.camera_optical_frame} -> {self.base_frame} unavailable: {e}')
            return None

        point_optical = PointStamped()
        point_optical.header.frame_id = self.camera_optical_frame
        point_optical.point.x = x_cam
        point_optical.point.y = y_cam
        point_optical.point.z = z_cam

        point_base = do_transform_point(point_optical, tf)
        x_base = point_base.point.x
        y_base = point_base.point.y
        z_base = point_base.point.z

        self.get_logger().info(
            f'[TF] {self.camera_optical_frame} -> {self.base_frame}: '
            f'cam_frame=({x_cam:.3f},{y_cam:.3f},{z_cam:.3f}) → '
            f'base_link=[{x_base:.3f},{y_base:.3f},{z_base:.3f}]')

        # Save detection data for visualization
        self._last_detection = {
            'centroid': (u_int, v_int),
            'depth_z': z_cam,
            'cam_xyz': (x_cam, y_cam, z_cam),
            'base_xyz': (x_base, y_base, z_base),
            'mask': mask,  # Save mask for viz
            'contour': largest,  # Save contour for viz (draws actual shape)
            'bbox': cv2.boundingRect(largest),  # (x, y, w, h) for bounding box
        }
        self._last_tf_stamp = tf.header.stamp

        return (x_base, y_base, z_base)

    def _confirm_callback(self, request, response):
        """Service: user confirms detected target, continue to grasp."""
        self._user_action = 'confirm'
        response.success = True
        response.message = 'Confirmed, will continue to grasp'
        return response

    def _skip_callback(self, request, response):
        """Service: user skips this object."""
        self._user_action = 'skip'
        response.success = True
        response.message = 'Skipped'
        return response

    def _abort_callback(self, request, response):
        """Service: user aborts the grasp."""
        self._user_action = 'abort'
        response.success = True
        response.message = 'Aborted'
        return response

    def _handle_wait_confirmation(self):
        """
        Pause after detection for user to review visualization.
        Continuously re-detect to allow real-time adjustment of target position.

        User triggers next action via ROS services (works over SSH, from launch, etc):
          - Call /visual_grasp/confirm  → continue to grasp
          - Call /visual_grasp/skip     → skip this object
          - Call /visual_grasp/abort    → quit
        """
        if not hasattr(self, '_confirmation_printed'):
            self.get_logger().info('=' * 60)
            self.get_logger().info('=== PAUSED FOR REVIEW ===')
            self.get_logger().info('Check visualization at /visual_grasp/visualization')
            self.get_logger().info('Review:')
            self.get_logger().info('  - RGB/Depth/Mask: Are the crosses on target center?')
            self.get_logger().info('  - Bottom coords: Are base_link (x,y,z) reasonable?')
            self.get_logger().info('  - You can MOVE the target, visualization updates in real-time')
            self.get_logger().info('')
            self.get_logger().info('Trigger next action via ROS services (in another terminal):')
            self.get_logger().info('  Continue grasp: ros2 service call /visual_grasp/confirm std_srvs/srv/Trigger')
            self.get_logger().info('  Skip object:    ros2 service call /visual_grasp/skip    std_srvs/srv/Trigger')
            self.get_logger().info('  Abort:          ros2 service call /visual_grasp/abort   std_srvs/srv/Trigger')
            self.get_logger().info('=' * 60)
            self._confirmation_printed = True
            # Reset user action flag
            self._user_action = None

        # Continuously re-detect to update visualization in real-time
        if self.rgb_image is not None and self.depth_image is not None:
            result = self._detect_and_localize()
            if result is not None:
                self.detected_point_base = result  # Update with latest detection

        # Check if user triggered any service
        if self._user_action == 'confirm':
            self.get_logger().info('User confirmed, continuing to grasp...')
            if self.detected_point_base is not None:
                self.get_logger().info(f'Final target: base_link=[{self.detected_point_base[0]:.3f}, {self.detected_point_base[1]:.3f}, {self.detected_point_base[2]:.3f}]')
            del self._confirmation_printed
            self._user_action = None
            self._change_state(State.COMPUTE_TARGET)
        elif self._user_action == 'skip':
            self.get_logger().warn('User skipped this object')
            del self._confirmation_printed
            self._user_action = None
            self._change_state(State.DONE)
        elif self._user_action == 'abort':
            self.get_logger().warn('User aborted')
            del self._confirmation_printed
            self._user_action = None
            self._change_state(State.FAILED)

    def _handle_compute_target(self):
        """Set arm_controller's pick_target WITHOUT moving the arm."""
        if self.detected_point_base is None:
            self._change_state(State.FAILED)
            return

        x, y, z = self.detected_point_base

        # Publish to set_pick_target (NOT target_pose): this only stores the target in
        # arm_controller; it does NOT move the arm. The pick service will run the full
        # grasp sequence (pre-grasp -> descend -> close -> retract) from this target.
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        # Quaternion for pitch=180° (gripper pointing straight down)
        # Convention matches arm_controller: 0=up, 90=horizontal, 180=down
        pitch = math.pi
        qw = math.cos(pitch / 2)
        qx = 0.0
        qy = math.sin(pitch / 2)
        qz = 0.0
        pose_msg.pose.orientation.w = qw
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz

        self.set_pick_target_pub.publish(pose_msg)
        self.get_logger().info(f'Set pick target (no move): [{x:.3f}, {y:.3f}, {z:.3f}]')

        self._change_state(State.PICK)

    def _handle_pick(self):
        """Call pick service (async, non-blocking - never spin inside a timer callback)."""
        # Already waiting on a previous call?
        if self._pick_future is not None:
            if self._pick_future.done():
                try:
                    response = self._pick_future.result()
                    if response and response.success:
                        self.get_logger().info('Pick completed successfully (arm returned to post-grasp pose)')
                        self._change_state(State.DONE)
                    else:
                        msg = response.message if response else 'no response'
                        self.get_logger().error(f'Pick failed: {msg}')
                        self._change_state(State.FAILED)
                except Exception as e:
                    self.get_logger().error(f'Pick service error: {e}')
                    self._change_state(State.FAILED)
                self._pick_future = None
            return

        # Not started yet: send the request once
        if not self.pick_client.service_is_ready():
            self.get_logger().warn('Waiting for /arm_controller/pick service...')
            return

        self.get_logger().info('Calling /arm_controller/pick...')
        self._pick_future = self.pick_client.call_async(Trigger.Request())

    def _handle_idle(self):
        """
        Idle state: waiting for /visual_grasp/start service call to trigger a grasp.

        Emit a one-shot info log so the user knows the node is ready and waiting.
        The visualization publisher keeps running (see state_machine_tick tail)
        so you can still see the camera feed in rqt_image_view while idle.
        """
        if not hasattr(self, '_idle_printed'):
            self.get_logger().info('=' * 60)
            self.get_logger().info('Ready. Waiting for /visual_grasp/start service call...')
            self.get_logger().info(
                '  Trigger grasp: ros2 service call /visual_grasp/start std_srvs/srv/Trigger')
            self.get_logger().info('=' * 60)
            self._idle_printed = True
        # No auto-transition; user triggers via _start_callback → state = INIT

    def _handle_done(self):
        """Grasp completed successfully — return to IDLE for the next one."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('Visual grasp completed successfully!')
        self.get_logger().info('Returning to IDLE. Call /visual_grasp/start for next grasp.')
        self.get_logger().info('=' * 60)
        self._reset_for_next_run()
        self._change_state(State.IDLE)

    def _handle_failed(self):
        """Grasp failed — log then return to IDLE so the user can retry."""
        self.get_logger().error('=' * 60)
        self.get_logger().error('Visual grasp failed')
        self.get_logger().error('Returning to IDLE. Call /visual_grasp/start to retry.')
        self.get_logger().error('=' * 60)
        self._reset_for_next_run()
        self._change_state(State.IDLE)

    def _reset_for_next_run(self):
        """Clear per-run state so the next grasp starts fresh."""
        self.detected_point_base = None
        self._last_detection = None
        self._last_tf_stamp = None
        self._pick_future = None
        self._refresh_future = None
        self._refresh_attempts = 0
        self.detection_attempts = 0
        self._user_action = None
        # Also clear "one-shot" print flags on stateful handlers
        for attr in ('_confirmation_printed', '_idle_printed'):
            if hasattr(self, attr):
                delattr(self, attr)

    def _start_callback(self, request, response):
        """Service /visual_grasp/start — kick off a grasp attempt.

        Only accepts the trigger when we're in IDLE; otherwise returns busy so
        the caller knows a grasp is already in progress.
        """
        if self.state != State.IDLE:
            response.success = False
            response.message = f'Busy, currently in state {self.state.name}'
            return response

        # Camera info arrival gates INIT → MOVE_TO_OBSERVE, so as long as the
        # camera is publishing, this will proceed. If not, INIT will just wait.
        self.get_logger().info('/visual_grasp/start called — starting grasp sequence')
        if hasattr(self, '_idle_printed'):
            delattr(self, '_idle_printed')
        self._change_state(State.INIT)
        response.success = True
        response.message = 'Started'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisualGraspNode()
    # MultiThreadedExecutor lets service callbacks (start/confirm/skip/abort)
    # be processed while the timer callback is running, and lets
    # _refresh_joints_sync's internal call_async future actually be delivered.
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
