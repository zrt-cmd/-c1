#!/usr/bin/env python3
"""
PWM to FK Demo - Validation Tool for Level 3

This script validates the PWM → Angle → FK → tcp_link position calculation chain.
User provides PWM values, script calculates tcp_link position + orientation.

Usage:
    ros2 run car1_kinematics pwm_to_fk_demo
    or
    python3 pwm_to_fk_demo.py [pwm0 pwm1 pwm2 pwm3 pwm4]

Author: Level 3 Implementation
Date: 2026-06-04
"""

import sys
import math
import numpy as np
from transform import pwm_array_to_angles, TCP_OFFSET_Z, print_servo_config
from fk import compute_fk


def pwm_to_tcp_pose(pwm_values):
    """PWM [id0..id4] -> TCP position + orientation.

    Returns dict with:
      angles_deg, x, y, z, roll, pitch, yaw, qx, qy, qz, qw
    """
    angles = pwm_array_to_angles(pwm_values)
    tcp_pos, T = compute_fk({
        'arm_joint1': angles[0], 'arm_joint2': angles[1],
        'arm_joint3': angles[2], 'arm_joint4': angles[3],
        'arm_joint5': angles[4],
    })
    R = T[:3, :3]

    # ZYX extrinsic RPY from rotation matrix
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        pitch = math.degrees(math.atan2(-R[2, 0], sy))
        yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    else:
        roll = math.degrees(math.atan2(-R[1, 2], R[1, 1]))
        pitch = math.degrees(math.atan2(-R[2, 0], sy))
        yaw = 0.0

    # Quaternion from rotation matrix
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1) * 2
        qw, qx, qy, qz = 0.25 * s, (R[2, 1] - R[1, 2]) / s, \
            (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw, qx, qy, qz = (R[2, 1] - R[1, 2]) / s, 0.25 * s, \
            (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw, qx, qy, qz = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, \
            0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw, qx, qy, qz = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, \
            (R[1, 2] + R[2, 1]) / s, 0.25 * s

    # TCP Z axis in base_footprint (rotation matrix 3rd column)
    tcp_z = R[:, 2]
    # arm_pitch: angle from vertical-up [0,0,1] to TCP-Z (0=up, 90=forward, 180=down)
    arm_pitch_rad = math.acos(max(-1.0, min(1.0, tcp_z[2])))

    return {
        'angles_deg': [math.degrees(a) for a in angles],
        'x': tcp_pos.x, 'y': tcp_pos.y, 'z': tcp_pos.z,
        'roll': roll, 'pitch': pitch, 'yaw': yaw,
        'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
        'arm_pitch_deg': math.degrees(arm_pitch_rad),
        'tcp_z_axis': tcp_z.tolist(),
    }


def print_tcp_pose(pwm_values, description=""):
    """Print TCP position + orientation for given PWM."""
    print("\n" + "=" * 60)
    if description:
        print("  " + description)
    print("=" * 60)

    try:
        r = pwm_to_tcp_pose(pwm_values)
    except Exception as e:
        print("Error: %s" % e)
        return

    print("PWM: %s" % pwm_values)
    print("Angles (deg): %s" % [round(a, 2) for a in r['angles_deg']])
    print()
    print("--- TCP Position (base_footprint) ---")
    print("  x = %.5f m (%5.1f cm)" % (r['x'], r['x'] * 100))
    print("  y = %.5f m (%5.1f cm)" % (r['y'], r['y'] * 100))
    print("  z = %.5f m (%5.1f cm)" % (r['z'], r['z'] * 100))
    print("  水平伸展: %.4f m (%.1f cm)" %
          (math.sqrt(r['x']**2 + r['y']**2),
           math.sqrt(r['x']**2 + r['y']**2) * 100))
    print()
    print("--- Orientation (ZYX extrinsic RPY) ---")
    print("  roll  = %.2f deg" % r['roll'])
    print("  pitch = %.2f deg" % r['pitch'])
    print("  yaw   = %.2f deg" % r['yaw'])
    print()
    print("--- Orientation (Quaternion) ---")
    print("  x = %.4f" % r['qx'])
    print("  y = %.4f" % r['qy'])
    print("  z = %.4f" % r['qz'])
    print("  w = %.4f" % r['qw'])
    print()
    print("--- PoseStamped (for arm_controller) ---")
    arm_pitch_rad = math.radians(r['arm_pitch_deg'])
    w2 = math.cos(arm_pitch_rad / 2)
    y2 = math.sin(arm_pitch_rad / 2)
    print("  arm_pitch: %.1f deg (0=up, 90=forward, 180=down)" %
          r['arm_pitch_deg'])
    print("  TCP Z direction: %s" % r['tcp_z_axis'])
    print("  frame_id: base_link")
    print("  position:    x=%.3f  y=%.3f  z=%.3f" % (r['x'], r['y'], r['z']))
    print("  orientation: x=0.0  y=%.4f  z=0.0  w=%.4f" % (y2, w2))
    print("=" * 60)

    return r


def main():
    """Main function. Supports CLI: python3 pwm_to_fk_demo.py [pwm0 pwm1 pwm2 pwm3 pwm4]"""
    # CLI mode
    if len(sys.argv) == 6:
        pwm = [int(x) for x in sys.argv[1:6]]
        print_tcp_pose(pwm)
        return
    elif len(sys.argv) > 1 and len(sys.argv) != 6:
        print("用法: python3 pwm_to_fk_demo.py [pwm0 pwm1 pwm2 pwm3 pwm4]")
        print("示例: python3 pwm_to_fk_demo.py 2150 1200 2200 2275 1500")
        return

    # Interactive mode
    print("\n" + "=" * 60)
    print("PWM -> TCP 位姿计算工具 (含位置 + 姿态)")
    print("=" * 60)

    print_servo_config()

    # Test cases
    print_tcp_pose([1500, 1500, 1500, 1500, 1500], "Home (全部1500, 竖直向上)")
    print_tcp_pose([2150, 1200, 2200, 2275, 1500], "握持姿态")

    print("\n自定义输入 (输入5个PWM值, 空格分隔, Ctrl+C退出)")
    try:
        while True:
            user_input = input("PWM> ").strip()
            if not user_input:
                break
            try:
                pwm = [int(x) for x in user_input.split()]
                if len(pwm) != 5:
                    print("  需要5个PWM值")
                    continue
                print_tcp_pose(pwm)
            except ValueError:
                print("  格式错误, 例如: 2150 1200 2200 2275 1500")
    except KeyboardInterrupt:
        print("\n退出")


if __name__ == '__main__':
    main()
