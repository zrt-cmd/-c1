#!/usr/bin/env python3
"""
基于手动参考角度的抓取演示
使用用户手动调试得到的成功抓取角度作为基准，通过微调生成完整抓取序列。
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class GraspDemoManualRef(Node):
    def __init__(self):
        super().__init__('grasp_demo_manual_ref')

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        # 手动参考角度（用户实测能抓到方块）
        self.grasp_angles = [0.0, -1.05, -1.05, -1.2, 0.0]

        # 基于参考角度生成其他位置
        # pre-grasp: joint2/3/4 各减小 0.15 rad（抬高约 3-4cm）
        self.pre_grasp_angles = [0.0, -0.90, -0.90, -1.05, 0.0]

        # lift: 在抓取基础上 joint2 减小 0.2（抬高）
        self.lift_angles = [0.0, -0.85, -1.05, -1.2, 0.0]

        # place: 移动到右前方（joint1 转 30°，其他保持 lift 姿态）
        self.place_above_angles = [0.52, -0.85, -1.05, -1.2, 0.0]  # 30° = 0.52 rad

        # home: 全零位置
        self.home_angles = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.arm_names = ['arm_joint1', 'arm_joint2', 'arm_joint3',
                          'arm_joint4', 'arm_joint5']

    def send_arm(self, positions, duration=2.0):
        msg = JointTrajectory()
        msg.joint_names = self.arm_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        msg.points = [point]
        self.arm_pub.publish(msg)
        self.get_logger().info(f'  发送机械臂: {[f"{p:.3f}" for p in positions]}')

    def send_gripper(self, position, duration=1.0):
        msg = JointTrajectory()
        msg.joint_names = ['r_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        msg.points = [point]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'  发送夹爪: {position:.3f}')

    def run_sequence(self):
        self.get_logger().info('\n========== 开始抓取序列 ==========')
        time.sleep(2.0)

        # 1. 张开夹爪
        self.get_logger().info('[1/8] 张开夹爪')
        self.send_gripper(0.8, duration=1.0)
        time.sleep(2.0)

        # 2. 移动到 pre-grasp
        self.get_logger().info('[2/8] 移动到 pre-grasp 位置')
        self.send_arm(self.pre_grasp_angles, duration=3.0)
        time.sleep(4.0)

        # 3. 下降到抓取位置（使用手动参考角度）
        self.get_logger().info('[3/8] 下降到抓取位置（手动参考角度）')
        self.send_arm(self.grasp_angles, duration=2.0)
        time.sleep(3.0)

        # 4. 闭合夹爪
        self.get_logger().info('[4/8] 闭合夹爪')
        self.send_gripper(0.0, duration=1.0)
        time.sleep(2.0)

        # 5. 提升
        self.get_logger().info('[5/8] 提升物体')
        self.send_arm(self.lift_angles, duration=2.0)
        time.sleep(3.0)

        # 6. 移动到放置位置上方
        self.get_logger().info('[6/8] 移动到放置位置上方')
        self.send_arm(self.place_above_angles, duration=3.0)
        time.sleep(4.0)

        # 7. 张开夹爪释放
        self.get_logger().info('[7/8] 释放物体')
        self.send_gripper(0.8, duration=1.0)
        time.sleep(2.0)

        # 8. 回到 home
        self.get_logger().info('[8/8] 回到 home 位置')
        self.send_arm(self.home_angles, duration=3.0)
        time.sleep(4.0)

        self.get_logger().info('========== 抓取序列完成 ==========\n')


def main():
    rclpy.init()
    node = GraspDemoManualRef()

    # 等待控制器就绪
    node.get_logger().info('等待 2 秒，确保控制器就绪...')
    time.sleep(2.0)

    # 执行抓取序列
    node.run_sequence()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
