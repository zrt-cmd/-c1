#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IK 测试节点（用于实机验证）

订阅目标位置，发布关节角度/PWM 值
不直接控制舵机，仅输出计算结果供验证

用法：
  ros2 run car1_kinematics ik_test_node

测试：
  ros2 topic pub /ik_test/target geometry_msgs/PoseStamped \
    "{pose: {position: {x: 0.25, y: 0.0, z: 0.15}}}" --once
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
import math

try:
    from .config import Point3D
    from .ik_analytical import solve_analytical_ik_auto_pitch, solution_to_dict, solution_to_pwm
    from .fk import compute_fk_position_only
except ImportError:
    from config import Point3D
    from ik_analytical import solve_analytical_ik_auto_pitch, solution_to_dict, solution_to_pwm
    from fk import compute_fk_position_only


class IKTestNode(Node):
    """IK 测试节点"""

    def __init__(self):
        super().__init__('ik_test_node')

        # 订阅目标位置
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/ik_test/target',
            self.target_callback,
            10
        )

        # 发布关节角度
        self.joint_pub = self.create_publisher(
            JointState,
            '/ik_test/joint_angles',
            10
        )

        # 发布 PWM 值
        self.pwm_pub = self.create_publisher(
            Int32MultiArray,
            '/ik_test/pwm_values',
            10
        )

        self.get_logger().info('IK 测试节点已启动')
        self.get_logger().info('订阅: /ik_test/target (geometry_msgs/PoseStamped)')
        self.get_logger().info('发布: /ik_test/joint_angles (sensor_msgs/JointState)')
        self.get_logger().info('发布: /ik_test/pwm_values (std_msgs/Int32MultiArray)')

    def target_callback(self, msg: PoseStamped):
        """处理目标位置"""
        pos = msg.pose.position
        target = Point3D(x=pos.x, y=pos.y, z=pos.z)

        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'收到目标: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) m')

        # 求解 IK（自动调整俯仰角）
        solution = solve_analytical_ik_auto_pitch(target, preferred_pitch_rad=0.0)

        if solution.error:
            self.get_logger().error(f'IK 求解失败: {solution.error}')
            return

        self.get_logger().info(f'✓ IK 求解成功: {solution.branch}')
        if solution.autoPitch:
            self.get_logger().info(
                f'  自动调整俯仰角: {math.degrees(solution.actualPitch):.1f}°'
            )

        # 转换为字典
        angles = solution_to_dict(solution)

        # 输出关节角度
        self.get_logger().info('关节角度 (度):')
        for joint_name, angle_rad in angles.items():
            self.get_logger().info(f'  {joint_name}: {math.degrees(angle_rad):+7.2f}°')

        # 转换为 PWM（直接映射）
        pwm_values = solution_to_pwm(solution)
        self.get_logger().info(f'PWM 值: {pwm_values}')

        # FK 验证
        computed_pos = compute_fk_position_only(angles)
        error = math.sqrt(
            (computed_pos.x - target.x)**2 +
            (computed_pos.y - target.y)**2 +
            (computed_pos.z - target.z)**2
        )
        self.get_logger().info(
            f'FK 验证: 误差 {error*1000:.2f} mm '
            f'{"✓" if error*1000 < 1.5 else "⚠"}'
        )

        # 发布关节角度
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        joint_msg.position = [
            angles['arm_joint1'],
            angles['arm_joint2'],
            angles['arm_joint3'],
            angles['arm_joint4'],
            angles['arm_joint5']
        ]
        self.joint_pub.publish(joint_msg)

        # 发布 PWM 值
        pwm_msg = Int32MultiArray()
        pwm_msg.data = pwm_values
        self.pwm_pub.publish(pwm_msg)

        self.get_logger().info(f'{"="*60}\n')


def main(args=None):
    rclpy.init(args=args)
    node = IKTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
