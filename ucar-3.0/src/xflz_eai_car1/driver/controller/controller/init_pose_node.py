#!/usr/bin/env python3
# encoding: utf-8
"""Init pose node: moves arm to initial position on startup."""
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from car1_servo_controller_msgs.msg import ServosPosition, ServoPosition


class InitPoseNode(Node):
    def __init__(self):
        super().__init__(
            'init_pose',
            allow_undeclared_parameters=True,
        )

        namespace = self.get_namespace()
        if namespace == '/':
            namespace = ''

        self.servo_controller_pub = self.create_publisher(
            ServosPosition, '/servo_controller', 10
        )

        # Wait for servo controller to be ready
        self.client = self.create_client(
            Trigger, namespace + '/servo_controller/init_finish'
        )
        self.get_logger().info('Waiting for servo_controller/init_finish...')
        self.client.wait_for_service()

        # Read init pose parameters
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('positions.arm_joint1', 1500)
        self.declare_parameter('positions.arm_joint2', 1500)
        self.declare_parameter('positions.arm_joint3', 1500)
        self.declare_parameter('positions.arm_joint4', 1500)
        self.declare_parameter('positions.arm_joint5', 1500)
        self.declare_parameter('positions.r_joint', 1500)

        duration = self.get_parameter('duration').value

        # Joint name -> servo ID mapping
        joint_servo_map = {
            'arm_joint1': 0,
            'arm_joint2': 1,
            'arm_joint3': 2,
            'arm_joint4': 3,
            'arm_joint5': 4,
            'r_joint': 5,
        }

        self.init_msg = ServosPosition()
        self.init_msg.duration = float(duration)
        self.init_msg.position_unit = 'pulse'

        for joint_name, servo_id in joint_servo_map.items():
            pulse = self.get_parameter(f'positions.{joint_name}').value
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = float(pulse)
            self.init_msg.position.append(servo)

        # Publish init pose command
        self.servo_controller_pub.publish(self.init_msg)

        self.get_logger().info(
            f'Init pose sent (duration={duration}s): '
            + ', '.join(f'{j}={self.get_parameter("positions." + j).value}' for j in joint_servo_map))

        self.create_service(Trigger, '~/init_finish', self._init_finish_callback)
        self.get_logger().info('\033[1;32minit_pose started\033[0m')

    def _init_finish_callback(self, request, response):
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = InitPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
