#!/usr/bin/env python3
"""Keyboard teleop control for car1 mecanum robot.

Keys:
    w/s - forward/backward
    a/d - turn left/right
    q/e - strafe left/right
    space - stop
    CTRL-C - quit
"""
import sys
import select
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

if os.name == 'nt':
    import msvcrt
    import time
else:
    import tty
    import termios

LIN_VEL = 0.2
ANG_VEL = 0.5

MSG = """
Control Your Car1 Robot (Mecanum)!
----------------------------------
Moving around:
   q    w    e
   a    s    d

w/s : forward/backward
a/d : rotate left/right
q/e : strafe left/right
space : stop
CTRL-C to quit
"""


def get_key(settings):
    if os.name == 'nt':
        timeout = 0.1
        start_time = time.time()
        while True:
            if msvcrt.kbhit():
                return msvcrt.getch().decode()
            elif time.time() - start_time > timeout:
                return ''
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class TeleopKeyControl(Node):
    def __init__(self):
        super().__init__('teleop_key_control')
        self.cmd_vel_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)

    def run(self):
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)
        else:
            settings = None

        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0

        try:
            print(MSG)
            while rclpy.ok():
                key = get_key(settings)
                if key == 'w':
                    linear_x = LIN_VEL
                    linear_y = 0.0
                    angular_z = 0.0
                elif key == 's':
                    linear_x = -LIN_VEL
                    linear_y = 0.0
                    angular_z = 0.0
                elif key == 'a':
                    linear_x = 0.0
                    linear_y = 0.0
                    angular_z = ANG_VEL
                elif key == 'd':
                    linear_x = 0.0
                    linear_y = 0.0
                    angular_z = -ANG_VEL
                elif key == 'q':
                    linear_x = 0.0
                    linear_y = LIN_VEL
                    angular_z = 0.0
                elif key == 'e':
                    linear_x = 0.0
                    linear_y = -LIN_VEL
                    angular_z = 0.0
                elif key == ' ':
                    linear_x = 0.0
                    linear_y = 0.0
                    angular_z = 0.0
                elif key == '\x03':
                    break
                elif key == '':
                    linear_x = 0.0
                    linear_y = 0.0
                    angular_z = 0.0

                twist = Twist()
                twist.linear.x = linear_x
                twist.linear.y = linear_y
                twist.angular.z = angular_z
                self.cmd_vel_pub.publish(twist)
        except Exception as e:
            print(e)
        finally:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    rclpy.init()
    node = TeleopKeyControl()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
