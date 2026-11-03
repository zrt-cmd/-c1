#!/usr/bin/env python3
"""
手动关节控制工具
在终端中用键盘调整各关节角度，实时发送到 Gazebo。
输入关节角度后回车发送，输入 'q' 退出，输入 'r' 读取当前关节状态。
"""
import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration


class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')

        self.arm_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(
            JointTrajectory, '/gripper_controller/joint_trajectory', 10)

        self.latest_joint_state = None
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 10)

        # 当前目标角度
        self.arm_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.gripper = 0.4  # 默认张开

        self.arm_names = ['arm_joint1', 'arm_joint2', 'arm_joint3',
                          'arm_joint4', 'arm_joint5']

    def _joint_state_cb(self, msg):
        self.latest_joint_state = msg

    def send_arm(self, duration=2.0):
        msg = JointTrajectory()
        msg.joint_names = self.arm_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in self.arm_joints]
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        msg.points = [point]
        self.arm_pub.publish(msg)

    def send_gripper(self, duration=1.0):
        msg = JointTrajectory()
        msg.joint_names = ['r_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(self.gripper)]
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1) * 1e9))
        msg.points = [point]
        self.gripper_pub.publish(msg)

    def print_state(self):
        if self.latest_joint_state:
            names = list(self.latest_joint_state.name)
            positions = list(self.latest_joint_state.position)
            print("\n===== 当前关节状态 =====")
            for n, p in zip(names, positions):
                print(f"  {n}: {p:.4f} rad ({p*57.2958:.1f} deg)")
            print("========================\n")
        else:
            print("  [尚未收到 /joint_states 消息]")


def main():
    rclpy.init()
    node = ManualControl()

    # 先 spin 一下接收 joint_states
    import threading
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    import time
    time.sleep(1.0)

    print("\n" + "="*50)
    print("  机械臂手动控制工具")
    print("="*50)
    print("\n命令:")
    print("  a j1 j2 j3 j4 j5  — 设置机械臂5个关节(弧度)")
    print("  g value            — 设置夹爪 (0=闭合, 0.4=张开)")
    print("  r                  — 读取当前关节状态")
    print("  h                  — 回到 home (全零)")
    print("  q                  — 退出")
    print("\n示例:")
    print("  a 0 -0.5 -0.8 -1.0 0    (向前伸展)")
    print("  a 0 -0.8 -1.2 -1.5 0    (更低位置)")
    print("  g 0.4                    (张开夹爪)")
    print("  g 0                      (闭合夹爪)")
    print()

    try:
        while True:
            try:
                cmd = input(">>> ").strip()
            except EOFError:
                break

            if not cmd:
                continue
            if cmd == 'q':
                break
            elif cmd == 'r':
                node.print_state()
            elif cmd == 'h':
                node.arm_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
                node.send_arm()
                print("  → 回到 home")
            elif cmd.startswith('a '):
                parts = cmd.split()
                if len(parts) != 6:
                    print("  错误: 需要5个关节值, 例如: a 0 -0.5 -0.8 -1.0 0")
                    continue
                try:
                    joints = [float(x) for x in parts[1:]]
                    node.arm_joints = joints
                    node.send_arm()
                    print(f"  → 机械臂: {[f'{j:.3f}' for j in joints]}")
                except ValueError:
                    print("  错误: 无法解析数值")
            elif cmd.startswith('g '):
                parts = cmd.split()
                if len(parts) != 2:
                    print("  错误: 需要1个值, 例如: g 0.4")
                    continue
                try:
                    node.gripper = float(parts[1])
                    node.send_gripper()
                    print(f"  → 夹爪: {node.gripper:.3f}")
                except ValueError:
                    print("  错误: 无法解析数值")
            else:
                print("  未知命令。输入 q 退出, r 读取状态, h 回home")

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    print("\n退出。")


if __name__ == '__main__':
    main()
