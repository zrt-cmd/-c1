#!/usr/bin/env python3
"""
视觉引导抓取演示节点
订阅物体检测结果，使用解析法 IK 求解，执行动态 pick-and-place。

订阅话题：
  /detected_object_pose - 物体在 base_link 下的位姿 (geometry_msgs/PoseStamped)

发布话题：
  /arm_controller/joint_trajectory     - 机械臂关节轨迹
  /gripper_controller/joint_trajectory - 夹爪关节轨迹
"""
import math
import threading
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped


class GraspDemoVision(Node):
    def __init__(self):
        super().__init__('grasp_demo_vision')

        # 使用手动验证的关节角度（来自 grasp_demo_manual_ref.py）
        self.observation_pose = [0.0, 0.2, -1.5, -1.5, 0.0]  # 观察位置
        self.pre_grasp_pose = [0.0, -0.90, -0.90, -1.05, 0.0]  # pre-grasp
        self.grasp_pose = [0.0, -1.05, -1.05, -1.2, 0.0]  # 抓取位置
        self.lift_pose = [0.0, -0.85, -1.05, -1.2, 0.0]  # 提升
        self.place_above_pose = [0.52, -0.85, -1.05, -1.2, 0.0]  # 放置上方（30°）
        self.home_pose = [0.0, 0.0, 0.0, 0.0, 0.0]  # home

        self.gripper_open = 0.8
        self.gripper_close = 0.0

        # 检测到的物体位置（仅用于确认检测到物体，不用于IK）
        self.detected_pos = None
        self.pose_received = False

        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # Subscriber
        self.pose_sub = self.create_subscription(
            PoseStamped, '/detected_object_pose', self.pose_callback, 10)

        self.get_logger().info('视觉抓取节点已启动，等待物体检测...')

        # 独立线程执行抓取
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def pose_callback(self, msg):
        """接收物体检测结果"""
        self.detected_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info(
                f'检测到物体: ({self.detected_pos[0]:.3f}, '
                f'{self.detected_pos[1]:.3f}, {self.detected_pos[2]:.3f})')

    def _run(self):
        """等待检测并执行抓取"""
        time.sleep(8.0)  # 等待 Gazebo + 控制器 + 相机就绪

        # 移动到观察位置（让相机朝向地面前方）
        self.get_logger().info('移动到观察位置...')
        self.send_arm_command(self.observation_pose, duration_sec=3.0)
        time.sleep(3.5)  # 等待运动完成

        self.get_logger().info('等待物体检测...')
        timeout = 30.0
        start = time.time()

        while not self.pose_received and (time.time() - start) < timeout:
            time.sleep(0.5)

        if not self.pose_received:
            self.get_logger().error(f'超时 {timeout}s，未检测到物体')
            return

        # 检测到物体，等待稳定
        time.sleep(2.0)

        if self.detected_pos is None:
            self.get_logger().error('检测结果丢失')
            return

        self.get_logger().info(
            f'检测到物体位置: ({self.detected_pos[0]:.3f}, '
            f'{self.detected_pos[1]:.3f}, {self.detected_pos[2]:.3f})')
        self.get_logger().info('使用手动验证的关节角度执行抓取')

        self.execute_grasp_sequence()

    # ========== IK 求解器（复用 grasp_demo.py） ==========

    def _gripper_offset_in_plane(self, pitch):
        gx, gy, gz = self.gripper_center_local
        sp, cp = math.sin(pitch), math.cos(pitch)
        dr = -cp * gx + sp * gz
        dz = sp * gx + cp * gz
        return (dr, dz)

    def compute_ik(self, target_bf, pitch_deg=None):
        if pitch_deg is None:
            pitch_deg = self.grasp_pitch_deg

        x, y, z = target_bf
        pitch = math.radians(pitch_deg)

        grip_dr, grip_dz = self._gripper_offset_in_plane(pitch)
        theta1 = -math.atan2(y, x)

        r_target = math.sqrt(x**2 + y**2)
        r_j5 = r_target - grip_dr
        z_j5 = z - grip_dz

        r = r_j5 - self.d_h
        z_arm = z_j5 - self.d_v

        r_w = r - self.L4 * math.sin(pitch)
        z_w = z_arm - self.L4 * math.cos(pitch)

        dist = math.sqrt(r_w**2 + z_w**2)
        max_reach = self.L2 + self.L3

        if dist > max_reach * 0.99:
            self.get_logger().error(
                f'目标超出范围: dist={dist:.4f}, max={max_reach:.4f}')
            return None
        if dist < abs(self.L2 - self.L3) * 1.1:
            self.get_logger().error(f'目标太近: dist={dist:.4f}')
            return None

        cos_q3 = (dist**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        q3_local = math.acos(cos_q3)

        alpha = math.atan2(r_w, z_w)
        beta = math.atan2(
            self.L3 * math.sin(q3_local),
            self.L2 + self.L3 * math.cos(q3_local))
        phi2 = alpha - beta

        theta2 = -phi2
        theta3 = -q3_local
        theta4 = -pitch - theta2 - theta3

        for name, val in [('j2', theta2), ('j3', theta3), ('j4', theta4)]:
            if val < self.joint_min or val > self.joint_max:
                self.get_logger().error(
                    f'关节 {name} 超限: {math.degrees(val):.1f}°')
                return None

        return [theta1, theta2, theta3, theta4, 0.0]

    # ========== 运动控制 ==========

    def send_arm_command(self, positions, duration_sec=2.0):
        msg = JointTrajectory()
        msg.joint_names = [
            'arm_joint1', 'arm_joint2', 'arm_joint3',
            'arm_joint4', 'arm_joint5'
        ]
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        msg.points = [point]
        self.arm_pub.publish(msg)
        self.get_logger().info(f'  机械臂: [{", ".join(f"{p:.3f}" for p in positions)}]')

    def send_gripper_command(self, position, duration_sec=1.0):
        msg = JointTrajectory()
        msg.joint_names = ['r_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9))
        msg.points = [point]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'  夹爪: {position:.3f}')

    # ========== 抓取序列 ==========

    def execute_grasp_sequence(self):
        """执行基于手动验证角度的抓取序列"""
        self.get_logger().info('===== 开始视觉抓取 =====')

        # Step 1: 张开夹爪
        self.get_logger().info('[Step 1] 张开夹爪')
        self.send_gripper_command(self.gripper_open)
        time.sleep(1.5)

        # Step 2: Pre-grasp
        self.get_logger().info('[Step 2] Pre-grasp')
        self.send_arm_command(self.pre_grasp_pose)
        time.sleep(3.0)

        # Step 3: 下降到抓取位置（手动验证的角度）
        self.get_logger().info('[Step 3] Grasp（手动验证角度）')
        self.send_arm_command(self.grasp_pose)
        time.sleep(3.0)

        # Step 4: 闭合夹爪
        self.get_logger().info('[Step 4] 闭合夹爪')
        self.send_gripper_command(self.gripper_close)
        time.sleep(2.0)

        # Step 5: 提升
        self.get_logger().info('[Step 5] 提升')
        self.send_arm_command(self.lift_pose)
        time.sleep(3.0)

        # Step 6: 移动到放置位置上方
        self.get_logger().info('[Step 6] 放置位置上方')
        self.send_arm_command(self.place_above_pose)
        time.sleep(3.0)

        # Step 7: 释放
        self.get_logger().info('[Step 7] 释放')
        self.send_gripper_command(self.gripper_open)
        time.sleep(2.0)

        # Step 8: Home
        self.get_logger().info('[Step 8] Home')
        self.send_arm_command(self.home_pose)
        time.sleep(3.0)

        self.get_logger().info('===== 视觉抓取完成 =====')


def main(args=None):
    rclpy.init(args=args)
    node = GraspDemoVision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
