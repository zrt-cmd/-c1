#!/usr/bin/env python3
# encoding: utf-8
"""Servo manager: handles communication with car1_base_driver for servo control."""
import rclpy
from rclpy.node import Node
from car1_base_driver_msgs.msg import SetBusServoState, BusServoState
from car1_base_driver_msgs.srv import GetBusServoState
from car1_servo_controller_msgs.msg import ServoPosition, ServosPosition
from car1_base_driver_msgs.msg import ServosPosition as DriverServosPosition
from car1_base_driver_msgs.msg import ServoPosition as DriverServoPosition


class ServoState:
    """Tracks the current state of a single servo."""
    def __init__(self, name=''):
        self.name = name
        self.position = 500


class ServoManager:
    """Manages servo positions via car1_base_driver topics/services."""

    def __init__(self, node, connected_ids):
        """
        Args:
            node: Parent ROS2 node for creating publishers/clients.
            connected_ids: Dict mapping servo_id (str) -> joint_name (str).
        """
        self.node = node
        self.connected_ids = connected_ids
        self.servos = {}
        for servo_id, joint_name in connected_ids.items():
            self.servos[servo_id] = ServoState(joint_name)

        namespace = node.get_namespace()
        if namespace == '/':
            namespace = ''

        self.servo_position_pub = node.create_publisher(
            DriverServosPosition,
            namespace + '/car1_base_driver/bus_servo/set_position',
            1
        )
        self.get_state_client = node.create_client(
            GetBusServoState,
            namespace + '/car1_base_driver/bus_servo/get_state'
        )

    def connect(self):
        """Wait for base driver service to be available."""
        self.node.get_logger().info('Waiting for car1_base_driver/bus_servo/get_state service...')
        self.get_state_client.wait_for_service()
        self.node.get_logger().info('Servo manager connected.')

    def get_position(self):
        """Return dict of servo_id -> ServoState."""
        return self.servos

    def set_position(self, duration, positions):
        """
        Send position command to servos.

        Args:
            duration: Movement duration in seconds.
            positions: List of ServoPosition messages (id, position in pulse).
        """
        duration = max(0.02, min(duration, 30.0))
        msg = DriverServosPosition()
        msg.duration = float(duration)
        for p in positions:
            pulse = int(p.position)
            pulse = max(500, min(pulse, 2500))
            self.servos[str(p.id)].position = pulse
            servo_msg = DriverServoPosition()
            servo_msg.id = p.id
            servo_msg.position = pulse
            msg.position.append(servo_msg)
        self.servo_position_pub.publish(msg)
