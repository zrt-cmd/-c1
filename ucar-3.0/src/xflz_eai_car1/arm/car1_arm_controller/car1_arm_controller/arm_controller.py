#!/usr/bin/env python3
"""
机械臂控制器节点 (Level 3 + Level 4)
提供安全的 IK 分步抓取控制，包括：
- 分步移动（底座先转，臂后动）
- 紧急停止机制
- 关节限位检查
- pick/place 高层接口
- 可配置抓取深度

IK: 复用 Level 3 验证过的 ik_analytical.py (支持 elbow 分支 + auto pitch)
"""

import sys
import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from car1_servo_controller_msgs.msg import ServosPosition, ServoPosition

# Import Level 3 IK solver (already validated)
try:
    from .ik_analytical import solve_analytical_ik_auto_pitch
    from .config import Point3D
except ImportError:
    from ik_analytical import solve_analytical_ik_auto_pitch
    from config import Point3D

# Joint limits (rad) - relaxed to match servo hardware range
# 500~2500 PWM = ±1000 ticks = ±2.356 rad = ±135° for 270° servos
JOINT_LIMITS_RAD = {
    'arm_joint1': (-2.356, 2.356),
    'arm_joint2': (-2.356, 2.356),
    'arm_joint3': (-2.356, 2.356),
    'arm_joint4': (-2.356, 2.356),
    'arm_joint5': (-2.356, 2.356),
}

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameters(namespace='', parameters=[
            ('d1', 0.1005),
            ('a2', 0.12614),
            ('a3', 0.12614),
            ('a4', 0.151),  # joint4 to tcp_link (URDF tcp_joint, 0.0744 + 0.0762)
            ('default_duration', 3.0),
            ('gripper_duration', 1.0),
            # 1.649 rad maps through r_joint orientation=-1 to PWM 800.
            ('gripper_open_angle', 1.649),
            ('gripper_close_angle', 0.0),
            ('gripper_depth', 0.0),  # 0 = tcp_link stops AT detected surface point; gripper straddles object, close to grasp (no descent)
            ('pre_grasp_height', 0.03),
            ('lift_height', 0.05),
            ('grasp_pitch', math.pi),  # 180° = end-effector pointing DOWN (ik_analytical convention)
            # After grasping, lift straight up then move to this pose (avoids dragging
            # the object across the table on a big diagonal move). [x, y, z, pitch].
            ('post_grasp_pose', [0.20, 0.0, 0.22, math.pi]),
        ])

        self.d1 = self.get_parameter('d1').value
        self.a2 = self.get_parameter('a2').value
        self.a3 = self.get_parameter('a3').value
        self.a4 = self.get_parameter('a4').value
        self.default_duration = self.get_parameter('default_duration').value
        self.gripper_duration = self.get_parameter('gripper_duration').value
        self.gripper_open = self.get_parameter('gripper_open_angle').value
        self.gripper_close = self.get_parameter('gripper_close_angle').value
        self.gripper_depth = self.get_parameter('gripper_depth').value
        self.pre_grasp_height = self.get_parameter('pre_grasp_height').value
        self.lift_height = self.get_parameter('lift_height').value
        self.grasp_pitch = self.get_parameter('grasp_pitch').value
        self.post_grasp_pose = self.get_parameter('post_grasp_pose').value

        # No longer need IKSolver instance - using solve_ik function from ik_analytical
        self.current_joints = [0.0] * 5
        self.stop_flag = False
        self._motion_lock = threading.Lock()

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/arm_controller/target_pose', self.pose_callback, 10)
        # set_pick_target: stores pick/place target WITHOUT moving the arm. Used by
        # visual_grasp so publishing the detected position doesn't trigger an extra
        # descent before the pick service runs the real grasp sequence.
        self.set_target_sub = self.create_subscription(
            PoseStamped, '/arm_controller/set_pick_target', self.set_pick_target_callback, 10)
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/arm_controller/joint_command', self.joint_cmd_callback, 10)

        # Publisher
        self.servo_pub = self.create_publisher(ServosPosition, '/servo_controller', 10)

        # Services
        self.gripper_srv = self.create_service(
            SetBool, '/arm_controller/set_gripper', self.gripper_callback,
            callback_group=self.cb_group)
        self.home_srv = self.create_service(
            Trigger, '/arm_controller/go_home', self.home_callback,
            callback_group=self.cb_group)
        self.work_srv = self.create_service(
            Trigger, '/arm_controller/go_work', self.work_callback,
            callback_group=self.cb_group)
        self.stop_srv = self.create_service(
            Trigger, '/arm_controller/emergency_stop', self.emergency_stop_callback,
            callback_group=self.cb_group)
        self.pick_srv = self.create_service(
            Trigger, '/arm_controller/pick', self.pick_callback,
            callback_group=self.cb_group)
        self.place_srv = self.create_service(
            Trigger, '/arm_controller/place', self.place_callback,
            callback_group=self.cb_group)

        # Pick/place target (set via target_pose topic before calling pick/place)
        self.pick_target = None
        self.place_target = None

        self.get_logger().info('arm_controller started (Level 3: step-wise IK)')

    # --- Callbacks ---

    def joint_callback(self, msg):
        arm_joints = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        for i, name in enumerate(arm_joints):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joints[i] = msg.position[idx]

    def _quat_to_pitch(self, q):
        """Extract pitch from quaternion (Y-axis rotation), full range [-pi, pi].
        Returns 0 if quaternion is identity/zero.

        Pitch convention (matches ik_analytical.py / web_server):
          0   = end-effector pointing UP
          pi/2 (90°) = end-effector horizontal forward
          pi (180°)  = end-effector pointing DOWN (typical grasp pose)
        """
        w, x, y, z = q.w, q.x, q.y, q.z
        norm_sq = w*w + x*x + y*y + z*z
        if norm_sq < 0.01:
            return 0.0
        # For pure Y-axis rotation: pitch = 2 * atan2(qy, qw), range [-pi, pi]
        return 2.0 * math.atan2(y, w)

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        pitch = self._quat_to_pitch(msg.pose.orientation)
        self.get_logger().info(
            f'target_pose: ({x:.3f}, {y:.3f}, {z:.3f}), pitch={math.degrees(pitch):.1f} deg')

        self.pick_target = (x, y, z, pitch)
        self.place_target = (x, y, z, pitch)

        # Use ik_analytical (pitch convention: 0=up, 90=horizontal, 180=down)
        solution = solve_analytical_ik_auto_pitch(Point3D(x, y, z), preferred_pitch_rad=pitch)
        if solution.error:
            self.get_logger().error(f'IK failed: {solution.error}')
            return

        joints = [solution.q1, solution.q2, solution.q3, solution.q4, solution.q5]
        self.get_logger().info(
            f'IK solution (rad): j1={joints[0]:.3f}, j2={joints[1]:.3f}, j3={joints[2]:.3f}, j4={joints[3]:.3f}, j5={joints[4]:.3f}')
        self.get_logger().info(
            f'IK solution (deg): j1={math.degrees(joints[0]):.1f}, j2={math.degrees(joints[1]):.1f}, j3={math.degrees(joints[2]):.1f}, j4={math.degrees(joints[3]):.1f}, j5={math.degrees(joints[4]):.1f}')
        actual_pitch = solution.actualPitch if solution.actualPitch is not None else pitch
        self.get_logger().info(f'  branch={solution.branch}, actual_pitch={math.degrees(actual_pitch):.1f} deg, auto={solution.autoPitch}')

        if not self._check_limits(joints):
            return

        self.stop_flag = False
        self._move_stepwise(joints, self.default_duration)

    def set_pick_target_callback(self, msg):
        """Store pick/place target WITHOUT moving the arm (used by visual_grasp)."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        pitch = self._quat_to_pitch(msg.pose.orientation)
        self.pick_target = (x, y, z, pitch)
        self.place_target = (x, y, z, pitch)
        self.get_logger().info(
            f'set pick_target (no move): ({x:.3f}, {y:.3f}, {z:.3f}), pitch={math.degrees(pitch):.1f} deg')

    def joint_cmd_callback(self, msg):
        if len(msg.position) != 5:
            self.get_logger().error('joint_command must have 5 positions')
            return

        joints = list(msg.position)
        duration = self.default_duration
        if hasattr(msg, 'effort') and len(msg.effort) > 0:
            duration = msg.effort[0]

        if not self._check_limits(joints):
            return

        self.stop_flag = False
        self._move_stepwise(joints, duration)

    def gripper_callback(self, request, response):
        angle = self.gripper_open if request.data else self.gripper_close
        action = "open" if request.data else "close"
        self._set_gripper(request.data)
        response.success = True
        response.message = f'gripper {action}'
        return response

    def home_callback(self, request, response):
        self.get_logger().info('go_home')
        self.stop_flag = False
        self._move_joints([0.0, 0.0, 0.0, 0.0, 0.0], self.default_duration)
        response.success = True
        response.message = 'home'
        return response

    def work_callback(self, request, response):
        """Move arm to work-ready pose (arm extended forward, ready for observation/grasp).
        Corresponds to servo PWM [1500, 1100, 2200, 1850, 1500] (see work_ready.yaml).
        """
        # Work pose: joints in radians, derived from PWM [1500,1100,2200,1850,1500]
        # PWM_TO_RAD = 4.712/2000, so (pwm-1500)*PWM_TO_RAD
        WORK_JOINTS_RAD = [0.0, -0.9424, 1.6492, 0.8246, 0.0]
        self.get_logger().info('go_work: moving to work-ready pose')
        self.stop_flag = False
        self._move_stepwise(WORK_JOINTS_RAD, self.default_duration)
        response.success = True
        response.message = 'work'
        return response

    def emergency_stop_callback(self, request, response):
        self.stop_flag = True
        self.get_logger().warn('EMERGENCY STOP')
        response.success = True
        response.message = 'stopped'
        return response

    def pick_callback(self, request, response):
        """Pick at the last target_pose position."""
        if self.pick_target is None:
            response.success = False
            response.message = 'no target set (publish to /arm_controller/target_pose first)'
            return response

        x, y, z, pitch = self.pick_target
        self.get_logger().info(f'pick at ({x:.3f}, {y:.3f}, {z:.3f}), pitch={math.degrees(pitch):.1f}')
        self.stop_flag = False

        try:
            self._execute_pick(x, y, z, pitch)
            response.success = True
            response.message = 'pick done'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'pick failed: {e}')
        return response

    def place_callback(self, request, response):
        """Place at the last target_pose position."""
        if self.place_target is None:
            response.success = False
            response.message = 'no target set (publish to /arm_controller/target_pose first)'
            return response

        x, y, z, pitch = self.place_target
        self.get_logger().info(f'place at ({x:.3f}, {y:.3f}, {z:.3f}), pitch={math.degrees(pitch):.1f}')
        self.stop_flag = False

        try:
            self._execute_place(x, y, z, pitch)
            response.success = True
            response.message = 'place done'
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'place failed: {e}')
        return response

    # --- Internal methods ---

    def _check_stop(self):
        if self.stop_flag:
            raise Exception("emergency stop")

    def _check_limits(self, joints):
        names = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        for i, name in enumerate(names):
            lo, hi = JOINT_LIMITS_RAD[name]
            if joints[i] < lo or joints[i] > hi:
                self.get_logger().error(
                    f'limit violation: {name}={math.degrees(joints[i]):.1f} deg '
                    f'(range [{math.degrees(lo):.1f}, {math.degrees(hi):.1f}])')
                return False
        return True

    def _move_joints(self, joints, duration):
        servo_ids = [0, 1, 2, 3, 4]  # joint1~5 → servo ID 0~4
        msg = ServosPosition()
        msg.duration = float(duration)
        msg.position_unit = 'rad'
        for servo_id, angle in zip(servo_ids, joints):
            p = ServoPosition()
            p.id = servo_id
            p.position = float(angle)
            msg.position.append(p)
        self.servo_pub.publish(msg)

    def _move_stepwise(self, target_joints, duration):
        """Step-wise move: rotate base first, then move arm."""
        with self._motion_lock:
            self._check_stop()

            base_diff = abs(target_joints[0] - self.current_joints[0])
            if base_diff > 0.05:
                base_only = list(self.current_joints)
                base_only[0] = target_joints[0]
                self._move_joints(base_only, duration * 0.4)
                self._wait(duration * 0.4 + 0.3)
                self._check_stop()

            self._move_joints(target_joints, duration * 0.6)
            self._wait(duration * 0.6 + 0.3)

    def _set_gripper(self, open_gripper):
        angle = self.gripper_open if open_gripper else self.gripper_close
        msg = ServosPosition()
        msg.duration = float(self.gripper_duration)
        msg.position_unit = 'rad'
        p = ServoPosition()
        p.id = 5
        p.position = float(angle)
        msg.position.append(p)
        self.servo_pub.publish(msg)

    def _wait(self, seconds):
        """Sleep in small increments, checking stop flag."""
        end = time.time() + seconds
        while time.time() < end:
            if self.stop_flag:
                raise Exception("emergency stop")
            time.sleep(0.05)

    def _solve_or_raise(self, x, y, z, pitch=0.0):
        solution = solve_analytical_ik_auto_pitch(Point3D(x, y, z), preferred_pitch_rad=pitch)
        if solution.error:
            raise Exception(f'IK failed for ({x:.3f}, {y:.3f}, {z:.3f}, pitch={math.degrees(pitch):.1f}): {solution.error}')
        joints = [solution.q1, solution.q2, solution.q3, solution.q4, solution.q5]
        if not self._check_limits(joints):
            raise Exception(f'limit violation for ({x:.3f}, {y:.3f}, {z:.3f})')
        return joints

    def _execute_pick(self, x, y, z, pitch=None):
        if pitch is None:
            pitch = self.grasp_pitch
        depth = self.gripper_depth
        pre_h = self.pre_grasp_height
        lift_h = self.lift_height

        # 1. Open gripper
        self._set_gripper(True)
        self._wait(self.gripper_duration + 0.2)
        self._check_stop()

        # 2. Move above target (gripper pointing down)
        joints = self._solve_or_raise(x, y, z + pre_h, pitch)
        self._move_stepwise(joints, self.default_duration)
        self._check_stop()

        # 3. Descend to grasp point (= detected surface point when gripper_depth=0)
        joints = self._solve_or_raise(x, y, z - depth, pitch)
        self._move_joints(joints, 1.5)
        self._wait(1.8)
        self._check_stop()

        # 4. Close gripper
        self._set_gripper(False)
        self._wait(self.gripper_duration + 0.3)
        self._check_stop()

        # 5. Retract to pre-grasp position (same as step 2, avoids self-collision)
        joints = self._solve_or_raise(x, y, z + pre_h, pitch)
        self._move_joints(joints, 1.5)
        self._wait(1.8)
        self._check_stop()

        # 6. Move directly to the post-grasp (observe) pose. Step 5 already lifted the
        #    object pre_grasp_height above the table, so it clears the surface and there
        #    is no need for an intermediate "straight-up" waypoint. That intermediate
        #    point at (x, y, post_z) was unreachable at pitch=pi, so auto-pitch silently
        #    tilted it (~165deg), then this segment snapped back to pitch=pi -- the wrist
        #    flip dipped the end-effector down. Going straight to the reachable
        #    post_grasp_pose keeps pitch=pi throughout, so no dip.
        px, py, pz, ppitch = self.post_grasp_pose
        joints = self._solve_or_raise(px, py, pz, ppitch)
        self._move_stepwise(joints, self.default_duration)
        self._wait(0.5)

        self.get_logger().info('pick complete (returned to post-grasp pose)')

    def _execute_place(self, x, y, z, pitch=None):
        if pitch is None:
            pitch = self.grasp_pitch
        pre_h = self.pre_grasp_height

        # 1. Move above target (gripper pointing down)
        joints = self._solve_or_raise(x, y, z + pre_h, pitch)
        self._move_stepwise(joints, self.default_duration)
        self._check_stop()

        # 2. Descend (gripper pointing down)
        joints = self._solve_or_raise(x, y, z, pitch)
        self._move_joints(joints, 1.5)
        self._wait(1.8)
        self._check_stop()

        # 3. Open gripper
        self._set_gripper(True)
        self._wait(self.gripper_duration + 0.3)
        self._check_stop()

        # 4. Retract upward (keep gripper pointing down)
        joints = self._solve_or_raise(x, y, z + pre_h + 0.03, pitch)
        self._move_joints(joints, 1.5)
        self._wait(1.8)

        self.get_logger().info('place complete')


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
