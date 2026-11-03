#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PWM ↔ Joint Angle Transformation for car1 Robotic Arm

This module provides conversion functions between servo PWM values and joint angles.
All parameters are calibrated for car1's 270° bus servos (ID 0-5).

Calibrated Parameters (2026-06-04):
- All servos: direction = +1 (PWM increase → angle increase, verified by FK test)
- All servos: offset = 0.0 (PWM 1500 → θ=0, vertical up for joints 1-4)
- tcp_link offset: 76.2mm below arm_link5 (measured: 76.25mm)
- Fingertip offset: 33mm below tcp_link

Author: Level 3 Implementation
Date: 2026-06-04
"""
import math

# ============================================================================
# Calibrated Parameters
# ============================================================================

# Servo specifications
CENTER_PWM = 1500           # Center position for all servos
PWM_RANGE = 2000            # 500-2500 (2000 units)
ANGLE_RANGE = 4.712         # 270° in radians (4.712 rad)
PWM_TO_RAD = ANGLE_RANGE / PWM_RANGE  # 0.002356 rad/PWM

# tcp_link offset relative to arm_link5
TCP_OFFSET_Z = 0.0762  # meters (measured: 76.25mm)

# Fingertip offset relative to tcp_link
FINGERTIP_OFFSET_Z = 0.033  # meters (33mm)

# Servo configuration
SERVO_CONFIG = {
    0: {'center': 1500, 'direction': +1, 'offset': 0.0, 'name': 'base_rotation', 'pwm_min': 500, 'pwm_max': 2500},
    1: {'center': 1500, 'direction': +1, 'offset': 0.0, 'name': 'big_arm',       'pwm_min': 500, 'pwm_max': 2500},
    2: {'center': 1500, 'direction': +1, 'offset': 0.0, 'name': 'small_arm',     'pwm_min': 500, 'pwm_max': 2500},
    3: {'center': 1500, 'direction': +1, 'offset': 0.0, 'name': 'wrist',         'pwm_min': 500, 'pwm_max': 2500},
    4: {'center': 1500, 'direction': +1, 'offset': 0.0, 'name': 'end_rotation',  'pwm_min': 500, 'pwm_max': 2500},
    5: {'center': 1500, 'direction': +1, 'offset': 0.0, 'name': 'gripper',       'pwm_min': 1000,'pwm_max': 2000},
}

# Legacy compatibility
SERVO_MIN = 500
SERVO_MAX = 2500
SERVO_CENTER = 1500
GRIPPER_CONFIG = {'servo_id': 5, 'open_pwm': 1500, 'close_pwm': 1800}


# ============================================================================
# Core Conversion Functions
# ============================================================================

def pulse2angle(servo_id, pwm_value):
    """Convert PWM to joint angle (radians)."""
    if servo_id not in SERVO_CONFIG:
        raise ValueError(f"Invalid servo ID: {servo_id}")
    config = SERVO_CONFIG[servo_id]
    pwm_offset = pwm_value - config['center']
    angle_offset = pwm_offset * PWM_TO_RAD
    angle = config['direction'] * angle_offset + config['offset']
    return angle


def angle2pulse_single(servo_id, angle_rad):
    """Convert joint angle (radians) to PWM."""
    if servo_id not in SERVO_CONFIG:
        raise ValueError(f"Invalid servo ID: {servo_id}")
    config = SERVO_CONFIG[servo_id]
    angle_offset = (angle_rad - config['offset']) * config['direction']
    pwm_offset = angle_offset / PWM_TO_RAD
    pwm_value = int(round(config['center'] + pwm_offset))
    pwm_value = max(config['pwm_min'], min(config['pwm_max'], pwm_value))
    return pwm_value


def pwm_array_to_angles(pwm_values):
    """Convert PWM array [id0-4] to angles [θ0-4]."""
    if len(pwm_values) != 5:
        raise ValueError(f"Expected 5 PWM values, got {len(pwm_values)}")
    return [pulse2angle(i, pwm) for i, pwm in enumerate(pwm_values)]


def angles_to_pwm_array(angles):
    """Convert angles [θ0-4] to PWM array [id0-4]."""
    if len(angles) != 5:
        raise ValueError(f"Expected 5 angles, got {len(angles)}")
    return [angle2pulse_single(i, angle) for i, angle in enumerate(angles)]


def check_pwm_limits(servo_id, pwm_value):
    """Check if PWM is within safe limits."""
    if servo_id not in SERVO_CONFIG:
        return False
    config = SERVO_CONFIG[servo_id]
    return config['pwm_min'] <= pwm_value <= config['pwm_max']


def check_angle_limits(servo_id, angle_rad):
    """Check if angle is within safe PWM limits."""
    try:
        pwm = angle2pulse_single(servo_id, angle_rad)
        return check_pwm_limits(servo_id, pwm)
    except:
        return False


# ============================================================================
# Legacy API (backward compatibility)
# ============================================================================

# Old joint config (deprecated, use SERVO_CONFIG instead)
JOINT_CONFIG = {
    'arm_joint1': {'servo_id': 0, 'center': 1500, 'direction': +1, 'min_deg': -135.0, 'max_deg': 135.0},
    'arm_joint2': {'servo_id': 1, 'center': 1500, 'direction': +1, 'min_deg': -135.0, 'max_deg': 135.0},
    'arm_joint3': {'servo_id': 2, 'center': 1500, 'direction': +1, 'min_deg': -135.0, 'max_deg': 135.0},
    'arm_joint4': {'servo_id': 3, 'center': 1500, 'direction': +1, 'min_deg': -135.0, 'max_deg': 135.0},
    'arm_joint5': {'servo_id': 4, 'center': 1500, 'direction': +1, 'min_deg': -135.0, 'max_deg': 135.0},
}


def angle2pulse(joint_angles):
    """Legacy: Convert dict/list of angles to [[servo_id, pwm], ...]."""
    if isinstance(joint_angles, dict):
        angles = joint_angles
    elif isinstance(joint_angles, (list, tuple)):
        names = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        angles = dict(zip(names, joint_angles))
    else:
        raise ValueError("joint_angles must be dict or list")

    result = []
    for joint_name, config in JOINT_CONFIG.items():
        if joint_name not in angles:
            continue
        angle_rad = angles[joint_name]
        servo_id = config['servo_id']
        pwm = angle2pulse_single(servo_id, angle_rad)
        result.append([servo_id, pwm])
    return result


def pulse2angle_legacy(pulses):
    """Legacy: Convert dict/list of PWMs to {'arm_joint1': rad, ...}."""
    if isinstance(pulses, (list, tuple)):
        pulse_dict = {i: p for i, p in enumerate(pulses)}
    elif isinstance(pulses, dict):
        pulse_dict = pulses
    else:
        raise ValueError("pulses must be dict or list")

    result = {}
    for joint_name, config in JOINT_CONFIG.items():
        sid = config['servo_id']
        if sid not in pulse_dict:
            continue
        pwm = pulse_dict[sid]
        result[joint_name] = pulse2angle(sid, pwm)
    return result


def check_limits(joint_angles):
    """Legacy: Check joint limits."""
    if isinstance(joint_angles, (list, tuple)):
        names = ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']
        angles = dict(zip(names, joint_angles))
    else:
        angles = joint_angles

    violations = []
    for joint_name, config in JOINT_CONFIG.items():
        if joint_name not in angles:
            continue
        angle_deg = math.degrees(angles[joint_name])
        if angle_deg < config['min_deg'] or angle_deg > config['max_deg']:
            violations.append(f"{joint_name}: {angle_deg:.1f}° out of [{config['min_deg']}, {config['max_deg']}]")
    return len(violations) == 0, violations


# ============================================================================
# Utility Functions
# ============================================================================

def print_servo_config():
    """Print servo configuration table."""
    print("=" * 80)
    print("car1 Servo Configuration")
    print("=" * 80)
    print(f"{'ID':<4} {'Name':<18} {'Center':<8} {'Direction':<10} {'Offset':<12} {'PWM Range':<15}")
    print("-" * 80)
    for servo_id, config in SERVO_CONFIG.items():
        pwm_range = f"{config['pwm_min']}-{config['pwm_max']}"
        print(f"{servo_id:<4} {config['name']:<18} {config['center']:<8} "
              f"{config['direction']:+<10} {config['offset']:<12.4f} {pwm_range:<15}")
    print("-" * 80)
    print(f"PWM to Rad: {PWM_TO_RAD:.6f} rad/PWM")
    print(f"tcp_link offset: {TCP_OFFSET_Z:.4f} m")
    print(f"Fingertip offset: {FINGERTIP_OFFSET_Z:.4f} m")
    print("=" * 80)


if __name__ == '__main__':
    print("\nTransform Module Test\n")
    print_servo_config()

    # Test 1: Center position
    print("\nTest 1: Center position")
    pwm1 = [1500, 1500, 1500, 1500, 1500]
    angles1 = pwm_array_to_angles(pwm1)
    pwm1_back = angles_to_pwm_array(angles1)
    print(f"  PWM: {pwm1}")
    print(f"  Angles (rad): {[f'{a:.4f}' for a in angles1]}")
    print(f"  Angles (deg): {[f'{math.degrees(a):.2f}' for a in angles1]}")
    print(f"  Back to PWM: {pwm1_back}")
    print(f"  Match: {pwm1 == pwm1_back}")

    # Test 2: User's test posture
    print("\nTest 2: User's test posture")
    pwm2 = [1500, 1850, 2109, 1850, 1500]
    angles2 = pwm_array_to_angles(pwm2)
    pwm2_back = angles_to_pwm_array(angles2)
    print(f"  PWM: {pwm2}")
    print(f"  Angles (rad): {[f'{a:.4f}' for a in angles2]}")
    print(f"  Angles (deg): {[f'{math.degrees(a):.2f}' for a in angles2]}")
    print(f"  Back to PWM: {pwm2_back}")
    print(f"  Match: {pwm2 == pwm2_back}")
