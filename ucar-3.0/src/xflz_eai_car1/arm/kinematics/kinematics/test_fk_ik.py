#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FK/IK Round-Trip Test

Validates the FK and IK implementations by:
1. Testing FK with known joint angles
2. Testing IK with known target positions
3. Round-trip test: target → IK → FK → verify position matches

Author: Level 3 Implementation
Date: 2026-06-08
"""
import math
import sys
from typing import List, Tuple

from config import ROBOT_CALIBRATION, GEOM, print_calibration, Point3D
from fk import compute_fk, compute_fk_position_only, validate_fk_round_trip
from ik_analytical import (
    solve_analytical_ik,
    solve_analytical_ik_auto_pitch,
    solve_analytical_ik_ignore_pitch,
    solution_to_dict,
    solution_to_pwm
)
from transform import (
    angles_to_pwm_array,
    pwm_array_to_angles
)


def print_joint_angles(angles: dict, label: str = "Joint angles"):
    """Print joint angles in degrees"""
    print(f"  {label}:")
    for joint_name in ['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']:
        if joint_name in angles:
            deg = math.degrees(angles[joint_name])
            print(f"    {joint_name}: {deg:+7.2f}°")


def print_pwm_values(pwm_values: List[int], label: str = "PWM values"):
    """Print PWM values"""
    print(f"  {label}: {pwm_values}")


def test_fk():
    """Test forward kinematics"""
    print("\n" + "=" * 80)
    print("Forward Kinematics Tests")
    print("=" * 80)

    test_cases = [
        {
            'name': 'Zero position',
            'angles': {'arm_joint1': 0, 'arm_joint2': 0, 'arm_joint3': 0, 'arm_joint4': 0, 'arm_joint5': 0},
        },
        {
            'name': 'Vertical up (q2=90°)',
            'angles': {'arm_joint1': 0, 'arm_joint2': math.pi/2, 'arm_joint3': 0, 'arm_joint4': 0, 'arm_joint5': 0},
        },
        {
            'name': 'Forward reach (q2=45°, q3=0°, q4=-45°)',
            'angles': {'arm_joint1': 0, 'arm_joint2': math.radians(45), 'arm_joint3': 0, 'arm_joint4': math.radians(-45), 'arm_joint5': 0},
        },
    ]

    for i, tc in enumerate(test_cases, 1):
        print(f"\nTest {i}: {tc['name']}")
        print_joint_angles(tc['angles'], "Input")

        tcp_pos, T = compute_fk(tc['angles'])
        print(f"  TCP position: ({tcp_pos.x:.6f}, {tcp_pos.y:.6f}, {tcp_pos.z:.6f}) m")
        print(f"               ({tcp_pos.x*1000:.2f}, {tcp_pos.y*1000:.2f}, {tcp_pos.z*1000:.2f}) mm")


def test_ik():
    """Test inverse kinematics"""
    print("\n" + "=" * 80)
    print("Inverse Kinematics Tests")
    print("=" * 80)

    test_cases = [
        {
            'name': 'Forward reach, horizontal',
            'target': Point3D(x=0.25, y=0.0, z=0.15),
            'pitch': 0.0,
        },
        {
            'name': 'Forward reach, downward 45°',
            'target': Point3D(x=0.22, y=0.0, z=0.12),
            'pitch': math.radians(-45),
        },
        {
            'name': 'Right side reach',
            'target': Point3D(x=0.20, y=0.05, z=0.15),
            'pitch': 0.0,
        },
    ]

    for i, tc in enumerate(test_cases, 1):
        print(f"\nTest {i}: {tc['name']}")
        print(f"  Target: ({tc['target'].x:.3f}, {tc['target'].y:.3f}, {tc['target'].z:.3f}) m")
        print(f"  Pitch: {math.degrees(tc['pitch']):.1f}°")

        solution = solve_analytical_ik(tc['target'], tc['pitch'])

        if solution.error:
            print(f"  ❌ Error: {solution.error}")
        else:
            print(f"  ✓ Solution found: {solution.branch}")
            angles = solution_to_dict(solution)
            print_joint_angles(angles, "Joint angles")


def test_round_trip():
    """Test IK → FK round trip"""
    print("\n" + "=" * 80)
    print("IK → FK Round-Trip Tests")
    print("=" * 80)

    test_positions = [
        Point3D(x=0.25, y=0.00, z=0.15),
        Point3D(x=0.22, y=0.03, z=0.12),
        Point3D(x=0.20, y=-0.03, z=0.18),
        Point3D(x=0.18, y=0.05, z=0.10),
    ]

    tolerance_mm = 1.5
    passed = 0
    failed = 0

    for i, target in enumerate(test_positions, 1):
        print(f"\nTest {i}: Target ({target.x:.3f}, {target.y:.3f}, {target.z:.3f}) m")

        # Solve IK
        solution = solve_analytical_ik_auto_pitch(target, preferred_pitch_rad=0.0)

        if solution.error:
            print(f"  ❌ IK failed: {solution.error}")
            failed += 1
            continue

        print(f"  IK solution: {solution.branch}")
        if solution.autoPitch:
            print(f"    Auto pitch: {math.degrees(solution.actualPitch):.1f}°")

        # Compute FK
        angles = solution_to_dict(solution)
        is_valid, error_mm, computed_pos = validate_fk_round_trip(target, angles, tolerance_mm)

        print(f"  FK position: ({computed_pos.x:.6f}, {computed_pos.y:.6f}, {computed_pos.z:.6f}) m")
        print(f"  Error: {error_mm:.2f} mm (tolerance: {tolerance_mm:.1f} mm)")

        if is_valid:
            print(f"  ✓ PASS")
            passed += 1
        else:
            print(f"  ❌ FAIL")
            failed += 1

    print(f"\n{'='*80}")
    print(f"Round-trip test summary: {passed} passed, {failed} failed")
    print(f"{'='*80}")


def test_pwm_conversion():
    """Test PWM ↔ angle conversion"""
    print("\n" + "=" * 80)
    print("PWM ↔ Angle Conversion Tests")
    print("=" * 80)

    # Test 1: Center position
    print("\nTest 1: Center position (PWM 1500)")
    pwm1 = [1500, 1500, 1500, 1500, 1500]
    angles1 = pwm_array_to_angles(pwm1)
    pwm1_back = angles_to_pwm_array(angles1)
    print_pwm_values(pwm1, "Input PWM")
    print(f"  Angles (deg): [{', '.join([f'{math.degrees(a):+.2f}' for a in angles1])}]")
    print_pwm_values(pwm1_back, "Output PWM")
    print(f"  Match: {'✓ PASS' if pwm1 == pwm1_back else '❌ FAIL'}")

    # Test 2: Test posture
    print("\nTest 2: Test posture (PWM [1500, 1850, 2109, 1850, 1500])")
    pwm2 = [1500, 1850, 2109, 1850, 1500]
    angles2 = pwm_array_to_angles(pwm2)
    pwm2_back = angles_to_pwm_array(angles2)
    print_pwm_values(pwm2, "Input PWM")
    print(f"  Angles (deg): [{', '.join([f'{math.degrees(a):+.2f}' for a in angles2])}]")
    print_pwm_values(pwm2_back, "Output PWM")
    print(f"  Match: {'✓ PASS' if pwm2 == pwm2_back else '❌ FAIL'}")

    # Test 3: IK solution → PWM (direct mapping)
    print("\nTest 3: IK solution → PWM (direct mapping)")
    target3 = Point3D(x=0.25, y=0.0, z=0.04)
    sol3 = solve_analytical_ik(target3, target_pitch_rad=math.radians(180))
    if sol3.error:
        print(f"  ❌ IK failed: {sol3.error}")
    else:
        pwm3 = solution_to_pwm(sol3)
        # Verify: PWM = 1500 + q / PWM_TO_RAD, direct mapping (no transform)
        angles3 = solution_to_dict(sol3)
        print_joint_angles(angles3, "IK angles")
        print_pwm_values(pwm3, "PWM (direct)")
        # Expected from hardware: [1500, 1870, 2094, 1869, 1500]
        expected = [1500, 1870, 2094, 1869, 1500]
        match = all(abs(a - b) <= 1 for a, b in zip(pwm3, expected))
        print(f"  Expected:    {expected}")
        print(f"  Match (±1): {'✓ PASS' if match else '❌ FAIL'}")


def test_workspace_limits():
    """Test workspace boundary conditions"""
    print("\n" + "=" * 80)
    print("Workspace Limit Tests")
    print("=" * 80)

    print(f"\nWorkspace parameters:")
    print(f"  Max reach: {GEOM.maxReach:.4f} m ({GEOM.maxReach*1000:.1f} mm)")
    print(f"  Min reach: {GEOM.minReach:.4f} m ({GEOM.minReach*1000:.1f} mm)")

    test_cases = [
        {
            'name': 'Near max reach',
            'target': Point3D(x=GEOM.maxReach - 0.01, y=0.0, z=ROBOT_CALIBRATION.J1_origin.z),
        },
        {
            'name': 'Beyond max reach',
            'target': Point3D(x=GEOM.maxReach + 0.05, y=0.0, z=ROBOT_CALIBRATION.J1_origin.z),
        },
        {
            'name': 'Near min reach',
            'target': Point3D(x=GEOM.minReach + 0.01, y=0.0, z=ROBOT_CALIBRATION.J1_origin.z),
        },
        {
            'name': 'Below min reach',
            'target': Point3D(x=GEOM.minReach - 0.01, y=0.0, z=ROBOT_CALIBRATION.J1_origin.z),
        },
    ]

    for i, tc in enumerate(test_cases, 1):
        print(f"\nTest {i}: {tc['name']}")
        print(f"  Target: ({tc['target'].x:.3f}, {tc['target'].y:.3f}, {tc['target'].z:.3f}) m")

        solution = solve_analytical_ik(tc['target'], target_pitch_rad=0.0)

        if solution.error:
            print(f"  Expected: Out of reach")
            print(f"  Result: ❌ {solution.error}")
        else:
            print(f"  Expected: Out of reach")
            print(f"  Result: ✓ Solution found (unexpected)")


def main():
    """Run all tests"""
    print("\n" + "=" * 80)
    print("car1 FK/IK Validation Test Suite")
    print("Level 3 Implementation")
    print("=" * 80)

    # Print calibration parameters
    print_calibration()

    # Run tests
    test_fk()
    test_ik()
    test_round_trip()
    test_pwm_conversion()
    test_workspace_limits()

    print("\n" + "=" * 80)
    print("All tests completed")
    print("=" * 80 + "\n")


if __name__ == '__main__':
    main()
