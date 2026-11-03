#!/usr/bin/env python3
# encoding: utf-8
"""
Level 5: Autonomous navigation + vision + grasp + transport.

Task flow:
    IDLE
      -> SET_INITIAL_POSE   (publish initial pose to AMCL)
      -> WAIT_LOCALIZATION  (wait for AMCL to converge)
      -> NAV_TO_PICK        (Nav2 goToPose)
      -> ROTATE_SEARCH      (spin in place, scan angles for red cube)
      -> DETECT_AND_GRASP   (HSV detect + plane projection + set_pick_target + pick)
      -> WAIT_GRASP         (wait for arm_controller pick service to complete)
      -> NAV_TO_PLACE       (Nav2 goToPose back to place location)
      -> PLACE_OBJECT       (open gripper + go_home)
      -> DONE               (idle for 5s -> IDLE)

Design notes:
- Zero modifications to visual_grasp.py / arm_controller.py. All Level 5 logic
  lives in this file. We reuse the arm_controller service API (set_pick_target,
  pick, set_gripper, go_home) but implement our own detection + plane projection.
- Plane projection avoids HP60C's 0.5m near-blind zone: we cast an RGB pixel to
  a 3D ray in the camera optical frame, transform the ray into base_link, and
  intersect with the known table plane (z = table_height). No depth image is
  read.
- Rotation search is closed-loop (read yaw from map->base_footprint TF, P
  controller on /cmd_vel angular.z). Angular clamp on /cmd_vel is ±1.0 rad/s
  (see odom_publisher), well above our target speeds.
"""
import enum
import math
import threading
import time

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    PointStamped,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Twist,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, Trigger
from tf2_geometry_msgs import do_transform_point


class State(enum.Enum):
    IDLE = 0
    SET_INITIAL_POSE = 1
    WAIT_LOCALIZATION = 2
    NAV_TO_PICK = 3
    MOVE_TO_OBSERVE = 4      # move arm to observation pose
    WAIT_ARM_STABLE = 5      # wait for arm to reach + camera to settle
    ROTATE_SEARCH = 6
    DETECT_AND_GRASP = 7     # detect + publish set_pick_target
    CALL_PICK = 8            # NEW: separate state for calling pick service
    WAIT_GRASP = 9
    NAV_TO_PLACE = 10
    PLACE_OBJECT = 11
    DONE = 12
    FAILED = 13


def _yaw_to_quat(yaw):
    """Return (x, y, z, w) quaternion for a rotation of `yaw` about Z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _quat_to_yaw(q):
    """Extract yaw from a quaternion object with .x/.y/.z/.w attributes."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap_pi(a):
    """Wrap angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


class NavigationGraspTransport(Node):

    def __init__(self):
        super().__init__('navigation_grasp_transport')

        # ---------------- Parameters ----------------
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('pick_pose', [1.0, 0.0, 0.0])
        self.declare_parameter('place_pose', [0.0, 0.0, 0.0])

        # Rotation search
        self.declare_parameter('enable_search', True)
        self.declare_parameter('search_angles_deg',
                               [0.0, 30.0, -30.0, 60.0, -60.0, 90.0, -90.0])
        self.declare_parameter('search_angular_speed', 0.4)  # rad/s target
        self.declare_parameter('search_yaw_tolerance', 0.05)  # rad (~3 deg)
        self.declare_parameter('search_settle_sec', 1.5)  # image settle after stop
        self.declare_parameter('red_pixel_threshold', 1000)

        # Observation pose (arm position for camera to look down at table)
        # Format: [x, y, z, pitch] in base_link frame
        # pitch convention: 0=up, pi/2=horizontal, pi=down (matches arm_controller)
        self.declare_parameter('observe_pose', [0.20, 0.0, 0.22, 3.14159])
        # Wait time: visual_grasp uses 5s. If arm is still moving during detect,
        # refresh_positions gives an incomplete pose -> x-offset ~2cm.
        self.declare_parameter('observe_settle_sec', 5.0)

        # Detection stability: repeat detection N times, use LAST result.
        # Mimics visual_grasp's WAIT_CONFIRMATION continuous re-detection.
        self.declare_parameter('detection_repeat_count', 5)
        self.declare_parameter('detection_repeat_interval_sec', 0.3)

        # Timeouts (seconds)
        self.declare_parameter('localization_settle_sec', 3.0)
        self.declare_parameter('detection_timeout_sec', 30.0)
        self.declare_parameter('grasp_timeout_sec', 60.0)
        self.declare_parameter('nav_timeout_sec', 300.0)

        # Plane projection
        self.declare_parameter('object_height', 0.04)  # m, top of object above table
        self.declare_parameter('table_height', 0.0)    # m, table top in base_link z

        # Place target in base_link frame [x, y, z, pitch]
        # Where to release the object relative to the robot after returning to place_pose
        self.declare_parameter('place_target_arm', [0.20, 0.0, 0.08, 3.14159])

        # HSV (matches visual_grasp defaults)
        self.declare_parameter('target_color_hsv_lower', [0, 100, 80])
        self.declare_parameter('target_color_hsv_upper', [15, 255, 255])
        self.declare_parameter('target_color_hsv_lower2', [160, 100, 80])
        self.declare_parameter('target_color_hsv_upper2', [180, 255, 255])

        # Frames / topics
        self.declare_parameter('camera_optical_frame', 'camera_depth_optical_frame')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('rgb_topic',
                               '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.declare_parameter('depth_topic',
                               '/ascamera_hp60c/camera_publisher/depth0/image_raw')
        self.declare_parameter('camera_info_topic',
                               '/ascamera_hp60c/camera_publisher/depth0/camera_info')
        # Prefer letting arm_controller's pick sequence carry the grasp motion.
        # If False, we only publish set_pick_target and call pick service.
        self.declare_parameter('dry_run', False)

        p = self.get_parameter
        self.initial_pose = list(p('initial_pose').value)
        self.pick_pose = list(p('pick_pose').value)
        self.place_pose = list(p('place_pose').value)

        self.enable_search = bool(p('enable_search').value)
        self.search_angles = [math.radians(a) for a in p('search_angles_deg').value]
        self.search_speed = float(p('search_angular_speed').value)
        self.search_yaw_tol = float(p('search_yaw_tolerance').value)
        self.search_settle = float(p('search_settle_sec').value)
        self.red_threshold = int(p('red_pixel_threshold').value)

        observe_pose = list(p('observe_pose').value)
        self.observe_x, self.observe_y, self.observe_z, self.observe_pitch = observe_pose
        self.observe_settle = float(p('observe_settle_sec').value)
        self.detection_repeat_count = int(p('detection_repeat_count').value)
        self.detection_repeat_interval = float(p('detection_repeat_interval_sec').value)

        self.localization_settle = float(p('localization_settle_sec').value)
        self.detection_timeout = float(p('detection_timeout_sec').value)
        self.grasp_timeout = float(p('grasp_timeout_sec').value)
        self.nav_timeout = float(p('nav_timeout_sec').value)

        self.object_height = float(p('object_height').value)
        self.table_height = float(p('table_height').value)

        self.place_target_arm = list(p('place_target_arm').value)

        self.hsv_lower = np.array(p('target_color_hsv_lower').value, dtype=np.uint8)
        self.hsv_upper = np.array(p('target_color_hsv_upper').value, dtype=np.uint8)
        self.hsv_lower2 = np.array(p('target_color_hsv_lower2').value, dtype=np.uint8)
        self.hsv_upper2 = np.array(p('target_color_hsv_upper2').value, dtype=np.uint8)

        self.camera_optical_frame = p('camera_optical_frame').value
        self.base_frame = p('base_frame').value
        self.map_frame = p('map_frame').value
        self.robot_frame = p('robot_frame').value
        self.rgb_topic = p('rgb_topic').value
        self.depth_topic = p('depth_topic').value
        self.camera_info_topic = p('camera_info_topic').value
        self.dry_run = bool(p('dry_run').value)

        # ---------------- Callback groups ----------------
        self._srv_cbg = ReentrantCallbackGroup()
        self._timer_cbg = MutuallyExclusiveCallbackGroup()
        self._client_cbg = ReentrantCallbackGroup()

        # ---------------- State ----------------
        self.state = State.IDLE
        self.state_enter_time = self.get_clock().now()
        self._state_lock = threading.Lock()

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rgb_image = None
        self.depth_image = None
        self.rgb_stamp = None
        self.camera_info = None
        self.fx = self.fy = self.cx = self.cy = None

        # search bookkeeping
        self._search_idx = 0
        self._search_base_yaw = None  # yaw at time NAV_TO_PICK finished; angles offset from this
        self._search_stop_time = None  # when we finished aligning to the current angle
        self._detected_target_base = None  # (x, y, z) in base_link once plane-projected

        # nav
        self.navigator = None
        self._nav_started = False
        self._nav_retry_used = False
        self._nav_goal_desc = ''

        # grasp bookkeeping
        self._pick_future = None
        self._gripper_future = None
        self._home_future = None
        self._place_future = None
        self._work_future = None
        self._work_move_start = None
        self._grasp_wait_start = None

        # ---------------- QoS ----------------
        image_qos = QoSProfile(depth=5)
        image_qos.reliability = ReliabilityPolicy.RELIABLE

        # ---------------- Subscribers ----------------
        self.create_subscription(Image, self.rgb_topic, self._rgb_cb, image_qos)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, image_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self._camera_info_cb, 10)

        # ---------------- Publishers ----------------
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.search_angle_pub = self.create_publisher(Float32,
                                                     '~/current_search_angle', 10)
        self.error_pub = self.create_publisher(String, '~/error_reason', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.set_pick_target_pub = self.create_publisher(
            PoseStamped, '/arm_controller/set_pick_target', 10)
        # For observation pose we use target_pose (moves arm immediately)
        self.arm_target_pose_pub = self.create_publisher(
            PoseStamped, '/arm_controller/target_pose', 10)

        # ---------------- Services (server) ----------------
        self.create_service(Trigger, '~/start', self._start_cb,
                            callback_group=self._srv_cbg)
        self.create_service(Trigger, '~/stop', self._stop_cb,
                            callback_group=self._srv_cbg)
        self.create_service(Trigger, '~/set_pick_pose_from_current',
                            self._set_pick_from_current_cb,
                            callback_group=self._srv_cbg)

        # ---------------- Service clients ----------------
        self.pick_client = self.create_client(Trigger, '/arm_controller/pick',
                                              callback_group=self._client_cbg)
        self.place_client = self.create_client(Trigger, '/arm_controller/place',
                                               callback_group=self._client_cbg)
        self.gripper_client = self.create_client(SetBool, '/arm_controller/set_gripper',
                                                 callback_group=self._client_cbg)
        self.home_client = self.create_client(Trigger, '/arm_controller/go_home',
                                              callback_group=self._client_cbg)
        self.work_client = self.create_client(Trigger, '/arm_controller/go_work',
                                              callback_group=self._client_cbg)
        # Refresh servo positions client (forces reading real angles from bus servos).
        # CRITICAL: without this, /joint_states shows COMMANDED positions, not actual
        # ones. The TF tree (camera_depth_optical_frame → base_link) would then reflect
        # the commanded arm pose, not the physical one, causing localization drift.
        # visual_grasp.py depends on this — we must too for consistent grasping.
        self.refresh_client = self.create_client(
            Trigger, '/servo_controller/refresh_positions',
            callback_group=self._client_cbg)

        # ---------------- Main tick ----------------
        # 10 Hz is plenty; state transitions are event-driven and detection is
        # only run at rest during ROTATE_SEARCH/DETECT_AND_GRASP.
        self.tick_timer = self.create_timer(0.1, self._tick,
                                            callback_group=self._timer_cbg)
        # Publish status at 10 Hz too (matches tick)
        self._last_status_publish = 0.0

        self.get_logger().info('navigation_grasp_transport ready (IDLE)')
        self.get_logger().info(f'  initial_pose: {self.initial_pose}')
        self.get_logger().info(f'  pick_pose:    {self.pick_pose}')
        self.get_logger().info(f'  place_pose:   {self.place_pose}')
        self.get_logger().info(f'  search enabled: {self.enable_search}')

    # ==================== Callbacks ====================

    def _rgb_cb(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.rgb_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f'RGB conversion failed: {e}')

    def _depth_cb(self, msg):
        """Cache depth image (HP60C publishes uint16 millimeters)."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'Depth conversion failed: {e}')

    def _camera_info_cb(self, msg):
        if self.camera_info is not None:
            return
        self.camera_info = msg
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(
            f'Camera info: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f}')

    def _start_cb(self, request, response):
        with self._state_lock:
            if self.state != State.IDLE:
                response.success = False
                response.message = f'Busy in state {self.state.name}'
                return response
            self._enter(State.SET_INITIAL_POSE)
        response.success = True
        response.message = 'Started'
        return response

    def _stop_cb(self, request, response):
        with self._state_lock:
            self._publish_error('Stopped by user')
            self._safe_stop_motion()
            if self.navigator is not None:
                try:
                    self.navigator.cancelTask()
                except Exception:
                    pass
            self._enter(State.FAILED)
        response.success = True
        response.message = 'Stopped'
        return response

    def _set_pick_from_current_cb(self, request, response):
        """Save the robot's CURRENT map pose as the pick_pose (handy for tuning)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
        except Exception as e:
            response.success = False
            response.message = f'TF lookup failed: {e}'
            return response
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = _quat_to_yaw(tf.transform.rotation)
        self.pick_pose = [x, y, yaw]
        response.success = True
        response.message = f'pick_pose = [{x:.3f}, {y:.3f}, {yaw:.3f}]'
        self.get_logger().info(response.message)
        return response

    # ==================== State machine ====================

    def _enter(self, new_state):
        if self.state == new_state:
            return
        self.get_logger().info(f'STATE: {self.state.name} -> {new_state.name}')
        self.state = new_state
        self.state_enter_time = self.get_clock().now()
        # Per-state entry cleanup
        if new_state == State.NAV_TO_PICK or new_state == State.NAV_TO_PLACE:
            self._nav_started = False
            self._nav_retry_used = False
        if new_state == State.ROTATE_SEARCH:
            self._search_idx = 0
            self._search_stop_time = None
            self._search_base_yaw = self._get_current_yaw()
        if new_state == State.DETECT_AND_GRASP:
            self._detected_target_base = None
        if new_state == State.WAIT_GRASP:
            self._grasp_wait_start = time.time()
        if new_state == State.MOVE_TO_OBSERVE or new_state == State.WAIT_ARM_STABLE:
            pass  # arm pose and wait handled in respective handlers
        if new_state == State.PLACE_OBJECT:
            # Reset place/work futures so handler starts fresh
            self._place_future = None
            self._work_future = None
            self._work_move_start = None
        if new_state == State.IDLE:
            # Clean up any pending futures from previous run
            self._pick_future = None
            self._gripper_future = None
            self._home_future = None
            self._place_future = None
            self._work_future = None
            self._work_move_start = None
        self._publish_status_now()

    def _elapsed_in_state(self):
        return (self.get_clock().now() - self.state_enter_time).nanoseconds / 1e9

    def _tick(self):
        try:
            self._tick_body()
        except Exception as e:
            self.get_logger().error(f'Tick exception: {e}')
            self._publish_error(f'tick exception: {e}')
            self._safe_stop_motion()
            self._enter(State.FAILED)

    def _tick_body(self):
        # Periodic status publish
        now = time.time()
        if now - self._last_status_publish > 0.5:
            self._publish_status_now()
            self._last_status_publish = now

        s = self.state
        if s == State.IDLE:
            return
        elif s == State.SET_INITIAL_POSE:
            self._handle_set_initial_pose()
        elif s == State.WAIT_LOCALIZATION:
            self._handle_wait_localization()
        elif s == State.NAV_TO_PICK:
            self._handle_navigate(self.pick_pose, State.MOVE_TO_OBSERVE, 'pick')
        elif s == State.MOVE_TO_OBSERVE:
            self._handle_move_to_observe()
        elif s == State.WAIT_ARM_STABLE:
            self._handle_wait_arm_stable()
        elif s == State.ROTATE_SEARCH:
            self._handle_rotate_search()
        elif s == State.DETECT_AND_GRASP:
            self._handle_detect_and_grasp()
        elif s == State.CALL_PICK:
            self._handle_call_pick()
        elif s == State.WAIT_GRASP:
            self._handle_wait_grasp()
        elif s == State.NAV_TO_PLACE:
            self._handle_navigate(self.place_pose, State.PLACE_OBJECT, 'place')
        elif s == State.PLACE_OBJECT:
            self._handle_place_object()
        elif s == State.DONE:
            if self._elapsed_in_state() > 5.0:
                self._enter(State.IDLE)
        elif s == State.FAILED:
            if self._elapsed_in_state() > 5.0:
                self._enter(State.IDLE)

    # ---------------- Handlers ----------------

    def _handle_set_initial_pose(self):
        # Publish initial pose to AMCL. AMCL subscribes to /initialpose (default).
        x, y, yaw = self.initial_pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        # Modest covariance so AMCL trusts our seed but still refines it.
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.068 # yaw (~15 deg std)
        self.initial_pose_pub.publish(msg)
        self.get_logger().info(f'Sent initial pose ({x:.3f}, {y:.3f}, {yaw:.3f}) to AMCL')
        self._enter(State.WAIT_LOCALIZATION)

    def _handle_wait_localization(self):
        if self._elapsed_in_state() >= self.localization_settle:
            self.get_logger().info('Localization settle done')
            # Ensure the navigator is up before we try to send a goal.
            self._ensure_navigator()
            self._enter(State.NAV_TO_PICK)

    def _handle_move_to_observe(self):
        """Move arm to observation pose so camera can see the table."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = self.observe_x
        pose.pose.position.y = self.observe_y
        pose.pose.position.z = self.observe_z
        # Convert pitch to quaternion (rotation around Y axis)
        pitch = self.observe_pitch
        pose.pose.orientation.w = math.cos(pitch / 2.0)
        pose.pose.orientation.y = math.sin(pitch / 2.0)
        self.arm_target_pose_pub.publish(pose)
        self.get_logger().info(
            f'Moving arm to observation pose: '
            f'x={self.observe_x}, y={self.observe_y}, z={self.observe_z}, '
            f'pitch={math.degrees(pitch):.1f}°')
        self._enter(State.WAIT_ARM_STABLE)

    def _handle_wait_arm_stable(self):
        """Wait for arm to reach observation pose and camera to stabilize."""
        if self._elapsed_in_state() >= self.observe_settle:
            self.get_logger().info('Arm stable, camera ready')
            if self.enable_search:
                self._enter(State.ROTATE_SEARCH)
            else:
                self._enter(State.DETECT_AND_GRASP)

    def _ensure_navigator(self):
        if self.navigator is not None:
            return
        try:
            from nav2_simple_commander.robot_navigator import BasicNavigator
        except ImportError as e:
            self._publish_error(f'nav2_simple_commander unavailable: {e}')
            self._enter(State.FAILED)
            return
        self.navigator = BasicNavigator()

    def _handle_navigate(self, target_pose, next_state, tag):
        """Send Nav2 goal and monitor task; retry once on failure."""
        self._ensure_navigator()
        if self.navigator is None:
            return

        if not self._nav_started:
            x, y, yaw = target_pose
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = self.map_frame
            goal.pose.position.x = float(x)
            goal.pose.position.y = float(y)
            qx, qy, qz, qw = _yaw_to_quat(yaw)
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
            self.get_logger().info(
                f'Nav2 goToPose[{tag}]: ({x:.3f}, {y:.3f}, {yaw:.3f})')
            try:
                self.navigator.goToPose(goal)
            except Exception as e:
                self._publish_error(f'Nav2 goToPose exception: {e}')
                self._enter(State.FAILED)
                return
            self._nav_started = True
            self._nav_goal_desc = tag
            return

        # Poll navigator
        if self.navigator.isTaskComplete():
            try:
                from nav2_simple_commander.robot_navigator import TaskResult
                result = self.navigator.getResult()
            except Exception as e:
                self._publish_error(f'Nav2 getResult exception: {e}')
                self._enter(State.FAILED)
                return
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Nav2 [{tag}] SUCCEEDED')
                self._nav_started = False
                self._nav_retry_used = False
                # brief settle before whatever comes next
                time.sleep(0.5)
                self._enter(next_state)
            else:
                self.get_logger().warn(
                    f'Nav2 [{tag}] result={result} (retry_used={self._nav_retry_used})')
                if not self._nav_retry_used:
                    self._nav_retry_used = True
                    self._nav_started = False
                    return
                self._publish_error(f'Nav2 [{tag}] failed after retry')
                self._enter(State.FAILED)
            return

        # Timeout check
        if self._elapsed_in_state() > self.nav_timeout:
            self._publish_error(f'Nav2 [{tag}] timed out after {self.nav_timeout:.0f}s')
            try:
                self.navigator.cancelTask()
            except Exception:
                pass
            self._enter(State.FAILED)

    def _handle_rotate_search(self):
        """Iterate through search_angles, rotate to each, then check if a red
        blob is visible. First hit above threshold -> DETECT_AND_GRASP."""
        if self.rgb_image is None:
            if self._elapsed_in_state() > 5.0:
                self._publish_error('No RGB image received during search')
                self._enter(State.FAILED)
            return

        if self._search_idx >= len(self.search_angles):
            self._publish_error('All search angles exhausted, no target found')
            self._safe_stop_motion()
            self._enter(State.FAILED)
            return

        target_yaw = _wrap_pi(self._search_base_yaw + self.search_angles[self._search_idx])
        current_yaw = self._get_current_yaw()
        if current_yaw is None:
            # TF not ready; just wait
            return

        err = _wrap_pi(target_yaw - current_yaw)

        self._publish_search_angle(self.search_angles[self._search_idx])

        if abs(err) > self.search_yaw_tol:
            # Still rotating: proportional control on angular.z
            self._search_stop_time = None
            tw = Twist()
            # Simple P with saturation
            tw.angular.z = float(max(-self.search_speed,
                                     min(self.search_speed, 1.5 * err)))
            self.cmd_vel_pub.publish(tw)
            return

        # Aligned. Stop motion + settle before checking image.
        if self._search_stop_time is None:
            self._safe_stop_motion()
            self._search_stop_time = time.time()
            return
        if time.time() - self._search_stop_time < self.search_settle:
            return

        # Now check red pixels
        red_pixels = self._count_red_pixels(self.rgb_image)
        deg = math.degrees(self.search_angles[self._search_idx])
        self.get_logger().info(
            f'Search angle {deg:+.1f} deg (idx {self._search_idx}): red_pixels={red_pixels}')

        if red_pixels >= self.red_threshold:
            self.get_logger().info('Target visible at this angle -> DETECT_AND_GRASP')
            self._enter(State.DETECT_AND_GRASP)
        else:
            self._search_idx += 1
            self._search_stop_time = None  # rotate to next angle

    def _handle_detect_and_grasp(self):
        """
        Detect red blob multiple times, use LAST result as target.

        Mimics visual_grasp's WAIT_CONFIRMATION: after WAIT_ARM_STABLE the
        arm may still be micro-adjusting for the first ~1s. A single-shot
        detection at the wrong moment gives x-offset ~2cm. Running detection
        N times and using the last result lets the pose settle before we
        commit to a grasp target.
        """
        if self._elapsed_in_state() > self.detection_timeout:
            self._publish_error('Detection timed out')
            self._enter(State.FAILED)
            return

        if self.rgb_image is None or self.camera_info is None:
            return

        # If we've already set target + started pick, do nothing here.
        if self._pick_future is not None:
            return

        # Run detection N times, keep the last successful result.
        # This is a BLOCKING operation inside the timer, but each detection
        # takes ~1.5s (dominated by refresh_positions), and we call it 5
        # times, so total ~7.5s. Within detection_timeout (30s default).
        last_target = None
        successful_count = 0
        for i in range(self.detection_repeat_count):
            result = self._detect_and_project()
            if result is not None:
                last_target = result
                successful_count += 1
                self.get_logger().info(
                    f'  Detect [{i+1}/{self.detection_repeat_count}]: '
                    f'({result[0]:.3f}, {result[1]:.3f}, {result[2]:.3f})')
            else:
                self.get_logger().warn(
                    f'  Detect [{i+1}/{self.detection_repeat_count}]: FAILED')
            # Small gap between detections (mimics visual_grasp's ~0.1s tick)
            if i < self.detection_repeat_count - 1:
                time.sleep(self.detection_repeat_interval)

        if last_target is None or successful_count < 2:
            # Need at least 2 successful detections for confidence
            if self._elapsed_in_state() < self.detection_timeout - 1.0:
                return  # try the whole batch again next tick
            self._publish_error(
                f'Detection failed: only {successful_count} successful')
            self._enter(State.FAILED)
            return

        x, y, z = last_target
        self._detected_target_base = last_target
        self.get_logger().info(
            f'FINAL target (from last of {successful_count} detections): '
            f'({x:.3f}, {y:.3f}, {z:.3f})')

        if self.dry_run:
            self.get_logger().warn('dry_run=true, skipping pick call')
            self._enter(State.NAV_TO_PLACE)
            return

        # 1. Publish set_pick_target (doesn't move the arm)
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # pitch = pi (gripper down); ik_analytical convention 0=up, pi=down
        pitch = math.pi
        pose.pose.orientation.w = math.cos(pitch / 2.0)
        pose.pose.orientation.y = math.sin(pitch / 2.0)
        self.set_pick_target_pub.publish(pose)
        self.get_logger().info(
            f'Published set_pick_target: [{x:.3f}, {y:.3f}, {z:.3f}]')

        # 2. Switch to CALL_PICK state. The next tick (100ms later) will call
        # the pick service. This matches visual_grasp.py's pattern:
        #   COMPUTE_TARGET (publish target) -> next tick -> PICK (call service)
        # The gap between ticks gives arm_controller's subscription callback
        # time to update pick_target BEFORE the pick service reads it.
        # (time.sleep inside the same tick doesn't guarantee this because it
        # blocks THIS node's executor; ROS 2 DDS layer routes messages via
        # separate threads, but arm_controller's callback still needs its
        # own executor to schedule it.)
        self._enter(State.CALL_PICK)

    def _handle_call_pick(self):
        """Call pick service. Runs one tick after set_pick_target was published,
        so arm_controller has had time to consume the subscription callback
        and update its internal pick_target."""
        if self._pick_future is not None:
            # Should not happen - but if it does, transition to WAIT_GRASP
            self._enter(State.WAIT_GRASP)
            return
        if not self.pick_client.service_is_ready():
            self.get_logger().warn('Waiting for /arm_controller/pick service...')
            return
        self.get_logger().info('Calling /arm_controller/pick...')
        self._pick_future = self.pick_client.call_async(Trigger.Request())
        self._enter(State.WAIT_GRASP)

    def _handle_wait_grasp(self):
        if self._pick_future is None:
            # Shouldn't happen; sanity guard.
            self._enter(State.FAILED)
            return
        if self._pick_future.done():
            try:
                resp = self._pick_future.result()
            except Exception as e:
                self._publish_error(f'pick service error: {e}')
                self._pick_future = None
                self._enter(State.FAILED)
                return
            self._pick_future = None
            if resp is not None and resp.success:
                self.get_logger().info(f'Pick succeeded: {resp.message}')
                self._enter(State.NAV_TO_PLACE)
            else:
                msg = resp.message if resp is not None else 'no response'
                self._publish_error(f'Pick failed: {msg}')
                self._enter(State.FAILED)
            return

        # Timeout
        if time.time() - self._grasp_wait_start > self.grasp_timeout:
            self._publish_error(f'Pick timed out after {self.grasp_timeout:.0f}s')
            self._pick_future = None
            self._enter(State.FAILED)

    def _handle_place_object(self):
        """Place object at place_target_arm, then return arm to work (observe) pose.

        Sequence:
        1. Set place target (relative to base_link) via /arm_controller/set_pick_target
        2. Call /arm_controller/place service (moves arm to target, opens gripper, retracts)
        3. Move arm back to work pose (observe_pose) for safe navigation
        """
        # Step 1: publish place target + call place service
        # Check ALL three futures are None — otherwise the state machine
        # would loop back to Step 1 during the gripper-close phase (which
        # only touches _gripper_future, leaving _place_future / _work_future
        # both None), causing an infinite pick-place-pick-place cycle.
        if (self._place_future is None and self._work_future is None
                and self._gripper_future is None):
            if not self.place_client.service_is_ready():
                self.get_logger().warn('Waiting for /arm_controller/place...')
                return

            # Set the place target (in base_link frame)
            px, py, pz, ppitch = self.place_target_arm
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.base_frame
            pose_msg.pose.position.x = float(px)
            pose_msg.pose.position.y = float(py)
            pose_msg.pose.position.z = float(pz)
            pose_msg.pose.orientation.w = math.cos(ppitch / 2.0)
            pose_msg.pose.orientation.y = math.sin(ppitch / 2.0)
            self.set_pick_target_pub.publish(pose_msg)
            self.get_logger().info(
                f'Set place target: ({px:.3f}, {py:.3f}, {pz:.3f}), pitch={math.degrees(ppitch):.1f}°')

            # Give arm_controller a tick to receive the target
            time.sleep(0.3)

            self.get_logger().info('Calling /arm_controller/place (release object)...')
            self._place_future = self.place_client.call_async(Trigger.Request())
            return

        # Step 2: wait for place service to complete
        if self._place_future is not None:
            if not self._place_future.done():
                return
            try:
                resp = self._place_future.result()
            except Exception as e:
                self._publish_error(f'place service error: {e}')
                self._place_future = None
                self._enter(State.FAILED)
                return
            if resp is None or not resp.success:
                msg = resp.message if resp is not None else 'no response'
                self._publish_error(f'Place failed: {msg}')
                self._place_future = None
                self._enter(State.FAILED)
                return
            self.get_logger().info(f'Place succeeded: {resp.message}')
            self._place_future = None

            # Step 3: call go_work service to move arm to work-ready pose
            if not self.work_client.service_is_ready():
                self.get_logger().warn('Waiting for /arm_controller/go_work service...')
                return
            self.get_logger().info('Calling /arm_controller/go_work...')
            self._work_future = self.work_client.call_async(Trigger.Request())
            return

        # Step 3 (continued): wait for go_work to complete
        if self._work_future is not None:
            if not self._work_future.done():
                return
            try:
                resp = self._work_future.result()
            except Exception as e:
                self._publish_error(f'go_work failed: {e}')
                self._work_future = None
                self._enter(State.FAILED)
                return
            self._work_future = None
            self.get_logger().info(
                f'go_work done: {resp.message if resp else "ok"}')

            # Step 4: close gripper after reaching work pose (ready for next task)
            if not self.gripper_client.service_is_ready():
                self.get_logger().warn('Waiting for /arm_controller/set_gripper...')
                return
            self.get_logger().info('Closing gripper at work pose (ready state)...')
            req = SetBool.Request()
            # arm_controller.gripper_callback: data=True -> open, data=False -> close
            req.data = False
            self._gripper_future = self.gripper_client.call_async(req)
            return

        # Step 4 (continued): wait for gripper close to complete
        if self._gripper_future is not None:
            if not self._gripper_future.done():
                return
            try:
                _ = self._gripper_future.result()
            except Exception as e:
                self._publish_error(f'set_gripper (close) failed: {e}')
                self._gripper_future = None
                self._enter(State.FAILED)
                return
            self._gripper_future = None
            self.get_logger().info('Gripper closed. Task complete -> DONE')
            self._enter(State.DONE)

    # ==================== Perception ====================

    def _refresh_joints_sync(self):
        """
        Synchronously refresh servo positions before TF lookup.

        Ported from visual_grasp.py — CRITICAL for accurate localization.
        Without this, /joint_states carries the commanded arm position, which
        may differ from the actual physical pose. The TF tree would then be
        based on commanded values, and the camera->base_link transform would
        be wrong, causing detected objects to appear at wrong base_link
        coordinates and the arm to miss when picking.

        Timeout: 1.5s. On timeout, use stale TF (graceful degradation).
        """
        if not self.refresh_client.service_is_ready():
            self.get_logger().warn(
                'refresh_positions service not ready, TF may be stale',
                throttle_duration_sec=5.0)
            return

        future = self.refresh_client.call_async(Trigger.Request())

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
        # and update the TF tree (typically 30-50ms is enough)
        time.sleep(0.1)

    def _count_red_pixels(self, rgb):
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.hsv_lower, self.hsv_upper),
            cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2),
        )
        return int(mask.sum() // 255)

    def _detect_and_project(self):
        """
        Detect red cube and localize in base_link using depth back-projection.
        Follows visual_grasp.py's proven pipeline (not plane projection).
        Returns (x, y, z) in base_link or None if detection/localization fails.
        """
        # Early checks with detailed logging
        if self.rgb_image is None:
            self.get_logger().warn('_detect_and_project: rgb_image is None', throttle_duration_sec=2.0)
            return None
        if self.depth_image is None:
            self.get_logger().warn('_detect_and_project: depth_image is None', throttle_duration_sec=2.0)
            return None
        if self.fx is None:
            self.get_logger().warn('_detect_and_project: camera intrinsics (fx) is None', throttle_duration_sec=2.0)
            return None

        rgb = self.rgb_image
        if rgb.mean() < 5.0:
            self.get_logger().warn(f'RGB frame too dark: mean={rgb.mean():.1f}', throttle_duration_sec=2.0)
            return None

        self.get_logger().info(
            f'_detect_and_project: rgb.shape={rgb.shape}, depth.shape={self.depth_image.shape}, '
            f'rgb.mean={rgb.mean():.1f}')

        # === HSV Detection ===
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.hsv_lower, self.hsv_upper),
            cv2.inRange(hsv, self.hsv_lower2, self.hsv_upper2),
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < 100:
            return None

        # Centroid
        M = cv2.moments(largest)
        if M['m00'] <= 0:
            return None
        u = M['m10'] / M['m00']
        v = M['m01'] / M['m00']
        u_int, v_int = int(round(u)), int(round(v))

        # === Depth Sampling (robust median inside contour) ===
        h, w = self.depth_image.shape[:2]
        contour_mask = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(contour_mask, [largest], -1, 255, thickness=cv2.FILLED)
        valid = (contour_mask > 0) & (self.depth_image > 0)
        object_depths = self.depth_image[valid].astype(np.float64)

        MIN_DEPTH_SAMPLES = 30
        if object_depths.size < MIN_DEPTH_SAMPLES:
            self.get_logger().warn(
                f'Too few valid depth px: {object_depths.size} (<{MIN_DEPTH_SAMPLES})',
                throttle_duration_sec=2.0)
            return None

        # Outlier rejection: median +/- 2.5*MAD
        med = np.median(object_depths)
        mad = np.median(np.abs(object_depths - med))
        if mad > 1e-6:
            inliers = object_depths[np.abs(object_depths - med) <= 2.5 * mad]
        else:
            inliers = object_depths
        z_raw = float(np.median(inliers))

        # HP60C depth is uint16 millimeters
        if np.issubdtype(self.depth_image.dtype, np.integer):
            z_cam = z_raw / 1000.0
        else:
            z_cam = z_raw

        # === Back-project to camera frame ===
        x_cam = (u - self.cx) * z_cam / self.fx
        y_cam = (v - self.cy) * z_cam / self.fy

        self.get_logger().info(
            f'[DETECT] centroid=({u_int},{v_int}) area={area:.0f} '
            f'depth_samples={object_depths.size}->inliers={inliers.size} '
            f'z_cam={z_cam:.3f}m | cam_frame=({x_cam:.3f},{y_cam:.3f},{z_cam:.3f})',
            throttle_duration_sec=1.0)

        # === CRITICAL: refresh joint positions before TF lookup ===
        # This forces the servo controller to read ACTUAL angles from the bus,
        # which robot_state_publisher then uses to update the TF tree.
        # Without this, TF reflects commanded (not physical) arm pose, leading
        # to wrong base_link coordinates → arm misses the target.
        self._refresh_joints_sync()

        # === TF transform to base_link ===
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_optical_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(
                f'TF {self.camera_optical_frame} -> {self.base_frame} unavailable: {e}',
                throttle_duration_sec=2.0)
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
            f'[TF] cam_frame=({x_cam:.3f},{y_cam:.3f},{z_cam:.3f}) → '
            f'base_link=[{x_base:.3f},{y_base:.3f},{z_base:.3f}]',
            throttle_duration_sec=1.0)

        return (x_base, y_base, z_base)

    # ==================== Helpers ====================

    def _get_current_yaw(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except Exception:
            return None
        return _quat_to_yaw(tf.transform.rotation)

    def _safe_stop_motion(self):
        stop = Twist()
        self.cmd_vel_pub.publish(stop)

    def _publish_status_now(self):
        msg = String()
        msg.data = self.state.name
        self.status_pub.publish(msg)

    def _publish_search_angle(self, radians_):
        msg = Float32()
        msg.data = float(math.degrees(radians_))
        self.search_angle_pub.publish(msg)

    def _publish_error(self, reason):
        msg = String()
        msg.data = reason
        self.error_pub.publish(msg)
        self.get_logger().error(reason)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGraspTransport()
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
