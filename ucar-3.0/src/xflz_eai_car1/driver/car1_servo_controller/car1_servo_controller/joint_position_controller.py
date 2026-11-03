#!/usr/bin/env python3
# encoding: utf-8
"""Joint position controller: pulse <-> radian conversion."""
import math


class JointPositionController:
    """Handles conversion between servo pulse values and joint radians."""

    # Servo range: 500~2500 pulse corresponds to 0~270 degrees (270° servo spec)
    RADIANS_PER_ENCODER_TICK = 270.0 / 360.0 * (math.pi * 2) / 2000.0  # 0.002356 rad/tick
    ENCODER_TICKS_PER_RADIAN = 1.0 / RADIANS_PER_ENCODER_TICK
    ENCODER_RESOLUTION = 2000

    def __init__(self, joint_name, servo_id, init_pos, min_pos, max_pos, orientation):
        self.joint_name = joint_name
        self.servo_id = servo_id
        self.initial_position_raw = init_pos
        self.min_angle_raw = min_pos
        self.max_angle_raw = max_pos
        self.orientation = orientation  # 1 or -1

        self.flipped = (orientation == -1)

        if self.flipped:
            self.min_angle = (self.initial_position_raw - self.max_angle_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.initial_position_raw - self.min_angle_raw) * self.RADIANS_PER_ENCODER_TICK
        else:
            self.min_angle = (self.min_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
            self.max_angle = (self.max_angle_raw - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK

        self.min_pulse = min(self.min_angle_raw, self.max_angle_raw)
        self.max_pulse = max(self.min_angle_raw, self.max_angle_raw)

    def pos_rad_to_pulse(self, pos_rad):
        """Convert radian position to pulse value, clamped to limits."""
        pos_rad = max(self.min_angle, min(pos_rad, self.max_angle))
        angle_raw = pos_rad * self.ENCODER_TICKS_PER_RADIAN
        if self.flipped:
            return int(self.initial_position_raw - angle_raw)
        else:
            return int(self.initial_position_raw + angle_raw)

    def pos_pulse_to_rad(self, pos_pulse):
        """Convert pulse value to radian position. NO clamping - readback must
        reflect the servo's actual position even if it's beyond soft limits
        (the limits only apply when commanding, not when reading)."""
        if self.flipped:
            return (self.initial_position_raw - pos_pulse) * self.RADIANS_PER_ENCODER_TICK
        else:
            return (pos_pulse - self.initial_position_raw) * self.RADIANS_PER_ENCODER_TICK
