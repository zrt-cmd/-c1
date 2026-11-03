#!/usr/bin/env python3
"""
SLAM建图专用键盘控制节点
完全模仿 teleop_twist_keyboard 的实现方式
"""

import sys
import termios
import tty
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


MSG = """
SLAM建图键盘控制
---------------------------
  w: 前进 (0.2 m/s)
  s: 后退 (0.2 m/s)
  a: 左转 (0.5 rad/s)
  d: 右转 (0.5 rad/s)
  空格: 停止
  Ctrl+C: 退出
---------------------------
"""

# 按键到速度的映射 (linear_x, angular_z)
MOVE_BINDINGS = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
}


def get_key(settings):
    """阻塞读取按键（关键：无超时，会等待按键输入）"""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)  # 阻塞读取，按住时键盘重复会持续触发
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    """保存终端设置"""
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    """恢复终端设置"""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    if not sys.stdin.isatty():
        print('错误：必须在交互式终端中运行')
        return

    settings = save_terminal_settings()

    rclpy.init()
    node = rclpy.create_node('slam_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    # 在后台线程中运行ROS spin
    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    linear_speed = 0.2
    angular_speed = 0.5

    x = 0.0
    th = 0.0

    try:
        print(MSG)
        while True:
            key = get_key(settings)

            if key in MOVE_BINDINGS:
                x = MOVE_BINDINGS[key][0]
                th = MOVE_BINDINGS[key][1]
            elif key == ' ':
                # 空格停止
                x = 0.0
                th = 0.0
            else:
                # 其他任意键也停止
                x = 0.0
                th = 0.0
                if key == '\x03':  # Ctrl+C
                    break

            # 发布速度
            twist = Twist()
            twist.linear.x = x * linear_speed
            twist.angular.z = th * angular_speed
            pub.publish(twist)

    except Exception as e:
        print(e)
    finally:
        # 停止机器人
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        rclpy.shutdown()
        spinner.join()
        restore_terminal_settings(settings)


if __name__ == '__main__':
    main()
