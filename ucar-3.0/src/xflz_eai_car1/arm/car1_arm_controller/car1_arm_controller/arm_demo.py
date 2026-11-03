#!/usr/bin/env python3
"""
机械臂控制演示脚本
展示如何使用 arm_controller 控制机械臂
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
import time


class ArmDemo(Node):
    def __init__(self):
        super().__init__('arm_demo')

        # 发布器
        self.pose_pub = self.create_publisher(PoseStamped, '/arm_controller/target_pose', 10)
        self.joint_pub = self.create_publisher(JointState, '/arm_controller/joint_command', 10)

        # 服务客户端
        self.gripper_client = self.create_client(SetBool, '/arm_controller/set_gripper')
        self.home_client = self.create_client(Trigger, '/arm_controller/go_home')

        self.get_logger().info('机械臂演示节点已启动')

    def move_to_pose(self, x, y, z):
        """移动到指定位姿"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        self.get_logger().info(f'移动到位姿: ({x:.3f}, {y:.3f}, {z:.3f})')
        self.pose_pub.publish(msg)

    def move_joints(self, joints, duration=2.0):
        """移动到指定关节角度"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        msg.position = joints
        msg.effort = [duration]  # 使用 effort 字段传递时间

        self.get_logger().info(f'移动关节: {[f"{j:.3f}" for j in joints]}')
        self.joint_pub.publish(msg)

    def set_gripper(self, open_gripper):
        """控制夹爪"""
        req = SetBool.Request()
        req.data = open_gripper

        action = "张开" if open_gripper else "闭合"
        self.get_logger().info(f'夹爪{action}')

        future = self.gripper_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def go_home(self):
        """回到零位"""
        self.get_logger().info('回到零位')
        req = Trigger.Request()
        future = self.home_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)


def main(args=None):
    rclpy.init(args=args)
    demo = ArmDemo()

    try:
        print("\n=== 机械臂控制演示 ===\n")

        # 等待服务就绪
        time.sleep(1.0)

        # 1. 回到零位
        print("1. 回到零位")
        demo.go_home()
        time.sleep(3.0)

        # 2. 张开夹爪
        print("\n2. 张开夹爪")
        demo.set_gripper(True)
        time.sleep(2.0)

        # 3. 移动到抓取位置（使用关节角度）
        print("\n3. 移动到抓取位置")
        demo.move_joints([0.0, 1.05, 1.05, 1.2, 0.0], duration=2.0)
        time.sleep(3.0)

        # 4. 闭合夹爪
        print("\n4. 闭合夹爪")
        demo.set_gripper(False)
        time.sleep(2.0)

        # 5. 提升
        print("\n5. 提升")
        demo.move_joints([0.0, 0.85, 1.05, 1.2, 0.0], duration=2.0)
        time.sleep(3.0)

        # 6. 移动到目标位置（使用 IK）
        print("\n6. 移动到目标位置（IK）")
        # demo.move_to_pose(0.25, 0.04, -0.04)
        demo.move_joints([0.5, 0.85, 1.05, 1.2, 0.0], duration=2.0)
        time.sleep(3.0)

        # 7. 释放
        print("\n7. 释放夹爪")
        demo.set_gripper(True)
        time.sleep(2.0)

        # 8. 回到零位
        print("\n8. 回到零位")
        demo.go_home()
        time.sleep(3.0)
        demo.set_gripper(False)
        time.sleep(2.0)

        print("\n=== 演示完成 ===\n")

    except KeyboardInterrupt:
        print("\n演示中断")
    finally:
        demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
