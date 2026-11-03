#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Forward Kinematics for car1 Robotic Arm

Based on the new URDF with corrected joint axes:
- arm_joint2, arm_joint3, arm_joint4: rotate around Y axis (not Z)
- Visual/collision meshes have RPY compensation applied
- Joint origins have XYZ coordinates adjusted

This FK implementation follows the actual URDF structure.

Author: Level 3 Implementation
Date: 2026-06-08
"""
import math
import numpy as np
from typing import Dict, Tuple, List
try:
    from .config import ROBOT_CALIBRATION, GEOM, Point3D
except ImportError:
    from config import ROBOT_CALIBRATION, GEOM, Point3D


def rotation_matrix_z(angle: float) -> np.ndarray:
    """Rotation matrix around Z axis"""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])


def rotation_matrix_y(angle: float) -> np.ndarray:
    """Rotation matrix around Y axis"""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ])


def homogeneous_transform(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """
    Create 4x4 homogeneous transformation matrix

    Args:
        R: 3x3 rotation matrix
        t: 3x1 translation vector

    Returns:
        4x4 transformation matrix
    """
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def compute_fk(joint_angles: Dict[str, float]) -> Tuple[Point3D, np.ndarray]:
    """
    Compute forward kinematics to tcp_link

    Based on new URDF structure:
    - base_footprint → base_link (0, 0, 0.029)
    - base_link → arm_base_link (0.046624, 0, 0.057)
    - arm_base_link → arm_link1 (0.01, 0, 0.0435) [joint1: Z-axis rotation]
    - arm_link1 → arm_link2 (0.000186, 0.025, 0.059467) [joint2: Y-axis rotation]
    - arm_link2 → arm_link3 (-0.0009, 0.000249, 0.12614) [joint3: Y-axis rotation]
    - arm_link3 → arm_link4 (-0.001266, 0.000249, 0.126138) [joint4: Y-axis rotation]
    - arm_link4 → arm_link5 (0.000919, -0.025283, 0.074395) [joint5: Z-axis rotation]
    - arm_link5 → tcp_link (0.000697, 0, 0.076247)

    Args:
        joint_angles: Dictionary with keys 'arm_joint1' through 'arm_joint5' (radians)

    Returns:
        (tcp_position, T_tcp): TCP position and full transformation matrix
    """
    cal = ROBOT_CALIBRATION

    # Extract joint angles (default to 0 if not provided)
    q1 = joint_angles.get('arm_joint1', 0.0)
    q2 = joint_angles.get('arm_joint2', 0.0)
    q3 = joint_angles.get('arm_joint3', 0.0)
    q4 = joint_angles.get('arm_joint4', 0.0)
    q5 = joint_angles.get('arm_joint5', 0.0)

    # Build transformation chain from base_footprint
    T = np.eye(4)

    # base_footprint → base_link (fixed)
    T = T @ homogeneous_transform(np.eye(3), np.array([0, 0, 0.029]))

    # base_link → arm_base_link (fixed)
    T = T @ homogeneous_transform(np.eye(3), np.array([0.046624, 0, 0.057]))

    # arm_base_link → arm_link1 (fixed offset)
    T = T @ homogeneous_transform(np.eye(3), np.array([0.01, 0, 0.0435]))

    # arm_joint1: rotation around Z
    T = T @ homogeneous_transform(rotation_matrix_z(q1), np.array([0, 0, 0]))

    # arm_link1 → arm_link2 (fixed offset)
    T = T @ homogeneous_transform(np.eye(3), np.array([0.000186, 0.025, 0.059467]))

    # arm_joint2: rotation around Y
    T = T @ homogeneous_transform(rotation_matrix_y(q2), np.array([0, 0, 0]))

    # arm_link2 → arm_link3 (fixed offset)
    T = T @ homogeneous_transform(np.eye(3), np.array([-0.0009, 0.000249, 0.12614]))

    # arm_joint3: rotation around Y
    T = T @ homogeneous_transform(rotation_matrix_y(q3), np.array([0, 0, 0]))

    # arm_link3 → arm_link4 (fixed offset)
    T = T @ homogeneous_transform(np.eye(3), np.array([-0.001266, 0.000249, 0.126138]))

    # arm_joint4: rotation around Y
    T = T @ homogeneous_transform(rotation_matrix_y(q4), np.array([0, 0, 0]))

    # arm_link4 → arm_link5 (fixed offset)
    T = T @ homogeneous_transform(np.eye(3), np.array([0.000919, -0.025283, 0.074395]))

    # arm_joint5: rotation around Z (with small RPY offset in URDF)
    # The URDF has rpy="-0.00000367 0.00000367 -3.14038265"
    # For simplicity, we approximate as pure Z rotation with -π offset
    T = T @ homogeneous_transform(rotation_matrix_z(q5 - math.pi), np.array([0, 0, 0]))

    # arm_link5 → tcp_link (fixed offset)
    T = T @ homogeneous_transform(np.eye(3), np.array([0.000697, 0, 0.076247]))

    # Extract TCP position
    tcp_pos = Point3D(x=T[0, 3], y=T[1, 3], z=T[2, 3])

    return tcp_pos, T


def compute_fk_position_only(joint_angles: Dict[str, float]) -> Point3D:
    """
    Compute forward kinematics (position only) to tcp_link

    Args:
        joint_angles: Dictionary with keys 'arm_joint1' through 'arm_joint5' (radians)

    Returns:
        tcp_position: TCP position in base_footprint frame
    """
    tcp_pos, _ = compute_fk(joint_angles)
    return tcp_pos


def compute_fk_from_j1(q2: float, q3: float, q4: float) -> Tuple[float, float]:
    """
    Compute J4 position in J1 local frame (X-Z plane only)

    This is the simplified 2R planar FK used for IK validation.
    Assumes the arm is in the X-Z plane of J1 frame.

    Args:
        q2: Joint 2 angle (radians)
        q3: Joint 3 angle (radians)
        q4: Joint 4 angle (radians)

    Returns:
        (x_j4, z_j4): J4 position in J1 local frame
    """
    cal = ROBOT_CALIBRATION
    g = GEOM

    # Start from J2 offset
    x = cal.J2_offset.x
    z = cal.J2_offset.z

    # Add link1 contribution (rotated by q2)
    x += g.L1 * math.sin(q2 + g.PHI1)
    z += g.L1 * math.cos(q2 + g.PHI1)

    # Add link2 contribution (rotated by q2 + q3)
    theta2 = q2 + g.PHI1 + q3 - g.PHI2
    x += g.L2 * math.sin(theta2 + g.PHI2)
    z += g.L2 * math.cos(theta2 + g.PHI2)

    return x, z


def compute_end_effector_pitch(joint_angles: Dict[str, float]) -> float:
    """
    Compute the pitch angle of the end effector

    Pitch is defined as the angle of the tool vector (J4→TCP)
    relative to the horizontal plane in J1 frame.

    Args:
        joint_angles: Dictionary with keys 'arm_joint2', 'arm_joint3', 'arm_joint4'

    Returns:
        pitch: End effector pitch angle (radians)
    """
    q2 = joint_angles.get('arm_joint2', 0.0)
    q3 = joint_angles.get('arm_joint3', 0.0)
    q4 = joint_angles.get('arm_joint4', 0.0)

    g = GEOM

    # Cumulative angle from horizontal
    # theta1 = q2 + PHI1 (link1 absolute angle)
    # theta2 = theta1 + q3 - PHI2 + PHI2 = theta1 + q3 (link2 absolute angle)
    # theta_tcp = theta2 + q4 - PHI_TCP + PHI_TCP = theta2 + q4

    theta1 = q2 + g.PHI1
    theta2 = theta1 + q3
    theta_tcp = theta2 + q4

    # Pitch is the tool direction angle (theta_tcp + PHI_TCP - PHI_TCP = theta_tcp)
    # But we need to account for PHI_TCP deviation
    pitch = theta_tcp + g.PHI_TCP

    return pitch


def validate_fk_round_trip(target_pos: Point3D, joint_angles: Dict[str, float],
                           tolerance_mm: float = 1.5) -> Tuple[bool, float, Point3D]:
    """
    Validate FK by comparing computed position with target

    Args:
        target_pos: Target position
        joint_angles: Joint angles from IK
        tolerance_mm: Acceptable error in millimeters

    Returns:
        (is_valid, error_mm, computed_pos): Validation result
    """
    computed_pos = compute_fk_position_only(joint_angles)

    error = math.sqrt(
        (computed_pos.x - target_pos.x) ** 2 +
        (computed_pos.y - target_pos.y) ** 2 +
        (computed_pos.z - target_pos.z) ** 2
    )

    error_mm = error * 1000
    is_valid = error_mm < tolerance_mm

    return is_valid, error_mm, computed_pos


if __name__ == '__main__':
    print("\n" + "=" * 80)
    print("Forward Kinematics Test")
    print("=" * 80)

    # Test 1: Zero position
    print("\nTest 1: Zero position (all joints at 0)")
    angles_zero = {
        'arm_joint1': 0.0,
        'arm_joint2': 0.0,
        'arm_joint3': 0.0,
        'arm_joint4': 0.0,
        'arm_joint5': 0.0,
    }
    tcp_zero, T_zero = compute_fk(angles_zero)
    print(f"  TCP position: ({tcp_zero.x:.6f}, {tcp_zero.y:.6f}, {tcp_zero.z:.6f}) m")
    print(f"  TCP position: ({tcp_zero.x*1000:.2f}, {tcp_zero.y*1000:.2f}, {tcp_zero.z*1000:.2f}) mm")

    # Test 2: Vertical up position (q2=pi/2, others 0)
    print("\nTest 2: Vertical up position (q2=π/2)")
    angles_up = {
        'arm_joint1': 0.0,
        'arm_joint2': math.pi / 2,
        'arm_joint3': 0.0,
        'arm_joint4': 0.0,
        'arm_joint5': 0.0,
    }
    tcp_up, T_up = compute_fk(angles_up)
    print(f"  TCP position: ({tcp_up.x:.6f}, {tcp_up.y:.6f}, {tcp_up.z:.6f}) m")
    print(f"  TCP position: ({tcp_up.x*1000:.2f}, {tcp_up.y*1000:.2f}, {tcp_up.z*1000:.2f}) mm")

    # Test 3: Forward reach (approximate)
    print("\nTest 3: Forward reach (q2=0, q3=0, q4=0)")
    angles_forward = {
        'arm_joint1': 0.0,
        'arm_joint2': 0.0,
        'arm_joint3': 0.0,
        'arm_joint4': 0.0,
        'arm_joint5': 0.0,
    }
    tcp_forward, T_forward = compute_fk(angles_forward)
    print(f"  TCP position: ({tcp_forward.x:.6f}, {tcp_forward.y:.6f}, {tcp_forward.z:.6f}) m")
    print(f"  TCP position: ({tcp_forward.x*1000:.2f}, {tcp_forward.y*1000:.2f}, {tcp_forward.z*1000:.2f}) mm")

    print("\n" + "=" * 80)
