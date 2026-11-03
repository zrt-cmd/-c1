#!/usr/bin/env python3
# encoding: utf-8
"""Mecanum wheel chassis kinematics for car1 robot."""
import math
from car1_base_driver_msgs.msg import MotorState, MotorsState


class MecanumChassis:
    """4-wheel mecanum chassis inverse/forward kinematics.

    Wheel layout (top view):
                x (forward)
        motor1 |  ^  | motor3
          +y<- |     | ->-y
        motor2 |     | motor4
    """

    def __init__(self, wheelbase=0.1946, track_width=0.216, wheel_diameter=0.0935):
        """Initialize mecanum chassis kinematics.

        Args:
            wheelbase: Distance between front and rear axles (m).
            track_width: Distance between left and right wheels (m).
            wheel_diameter: Wheel diameter (m).
        """
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_to_rps(self, speed):
        """Convert linear speed (m/s) to rotations per second."""
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        Convert cmd_vel to motor speeds.

        Args:
            linear_x: Forward velocity (m/s).
            linear_y: Lateral velocity (m/s), positive = left.
            angular_z: Yaw rate (rad/s), positive = CCW.

        Returns:
            MotorsState message with 4 motor commands.

        Wheel layout (top view):
                    x (forward)
            motor1 |  ^  | motor2
              +y<- |     | ->-y
            motor3 |     | motor4
        """
        k = (self.wheelbase + self.track_width) / 2.0

        # Mecanum inverse kinematics
        fl = linear_x - linear_y - angular_z * k  # front-left (motor1)
        fr = linear_x + linear_y + angular_z * k  # front-right (motor2)
        rl = linear_x + linear_y - angular_z * k  # rear-left (motor3)
        rr = linear_x - linear_y + angular_z * k  # rear-right (motor4)

        # Firmware raw per-wheel mode expects wheel-rim LINEAR velocity (m/s),
        # NOT rps. The firmware feedback RT = ticks * (pi*D*PID_RATE/RESOLUTION)
        # is already in m/s, and its PID drives RT == TG. So send fl/fr/rl/rr
        # (which are m/s here) directly. Converting to rps caused a ~3.98x
        # over-speed (1/(pi*D_firmware)). The msg field is named .rps for
        # historical reasons but carries m/s.
        v_s = [fl, fr, rl, rr]

        data = []
        for i in range(4):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)

        motors_msg = MotorsState()
        motors_msg.data = data
        return motors_msg
