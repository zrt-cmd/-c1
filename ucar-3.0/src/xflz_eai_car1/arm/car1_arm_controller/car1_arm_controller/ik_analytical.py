#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Analytical Inverse Kinematics for car1 Robotic Arm

Based on geometric closed-form IK solver, no iteration required, fast (~0.05ms).
Supports three modes:
  1. Fixed Pitch: User specifies exact pitch angle
  2. Auto Pitch: Automatically finds closest feasible pitch if user's pitch is unreachable
  3. Ignore Pitch: Scans all pitches and elbow branches, selects config with minimum FK error

Port from test/car1_web_server/viewer/kinematics/ik-analytical.js

Author: Level 3 Implementation
Date: 2026-06-08
"""
import math
from typing import Dict, Optional, Tuple, List
from dataclasses import dataclass

try:
    from .config import ROBOT_CALIBRATION, GEOM, Point3D
except ImportError:
    from config import ROBOT_CALIBRATION, GEOM, Point3D


@dataclass
class IKSolution:
    """IK solution result"""
    q1: float  # arm_joint1 (base rotation)
    q2: float  # arm_joint2 (shoulder)
    q3: float  # arm_joint3 (elbow)
    q4: float  # arm_joint4 (wrist)
    q5: float  # arm_joint5 (end rotation)
    branch: str  # 'elbow-up' or 'elbow-down'
    error: Optional[str] = None  # Error message if solution failed
    actualPitch: Optional[float] = None  # Actual pitch used (for auto/ignore modes)
    autoPitch: bool = False  # Whether pitch was auto-adjusted
    pitchDeviation: float = 0.0  # Deviation from preferred pitch
    tcpError: Optional[float] = None  # FK error (for ignore pitch mode)
    elbowZ: Optional[float] = None  # Elbow Z height (for ignore pitch mode)


def solve_analytical_ik(target_pos: Point3D,
                       target_pitch_rad: float = 0.0,
                       config=None,
                       geom=None) -> IKSolution:
    """
    Analytical IK (Fixed Pitch Mode)

    Args:
        target_pos: Target TCP position {x, y, z} in base_footprint frame (meters)
        target_pitch_rad: Target pitch angle (radians), 0 = horizontal forward
        config: Robot calibration parameters (default: ROBOT_CALIBRATION)
        geom: Derived geometry (default: GEOM)

    Returns:
        IKSolution: IK solution with joint angles or error message
    """
    cal = config if config else ROBOT_CALIBRATION
    g = geom if geom else GEOM

    # === Step 1: Translate to J1 origin ===
    dx_world = target_pos.x - cal.J1_origin.x
    dy_world = target_pos.y - cal.J1_origin.y
    dz_world = target_pos.z - cal.J1_origin.z

    # === Step 2: Calculate q1 (base yaw) ===
    # TCP has fixed Y offset Y_TCP in J1 local frame (~0.215mm), cannot use atan2(dy,dx) directly.
    # Geometric relation: horizontal distance r = √(dx²+dy²), TCP offset needs to hit target,
    # so q1 = atan2(dy,dx) - asin(Y_TCP / r)
    r_horiz = math.hypot(dx_world, dy_world)
    q1 = math.atan2(dy_world, dx_world)
    if r_horiz > 1e-9:
        asin_arg = max(-1.0, min(1.0, g.Y_TCP / r_horiz))
        q1 -= math.asin(asin_arg)

    # === Step 3: Rotate to J1 coordinate system ===
    c1 = math.cos(q1)
    s1 = math.sin(q1)
    x_j1 = c1 * dx_world + s1 * dy_world
    y_j1 = -s1 * dx_world + c1 * dy_world
    z_j1 = dz_world

    # === Step 4: Check Y coordinate constraint ===
    # q1 has compensated Y_TCP, ideally y_j1 ≈ Y_TCP, check residual is within tolerance
    if abs(y_j1 - g.Y_TCP) > cal.solver['yPlaneTolerance']:
        return IKSolution(
            q1=q1, q2=0, q3=0, q4=0, q5=0,
            branch='none',
            error=f"Target Y residual={((y_j1-g.Y_TCP)*1000):.1f}mm in J1 frame, exceeds workspace constraint (tolerance ±{(cal.solver['yPlaneTolerance']*1000):.0f}mm)"
        )

    # === Step 5: Solve 2R IK in X-Z plane ===
    # pitch is defined as the J4→TCP offset vector's actual direction angle in J1 frame.
    # Offset vector length = L_TCP, rotates around Y axis by pitch, X component already in sin(pitch).
    toolX = g.L_TCP * math.sin(target_pitch_rad)
    toolZ = g.L_TCP * math.cos(target_pitch_rad)

    x_j4 = x_j1 - toolX
    z_j4 = z_j1 - toolZ

    x_rel = x_j4 - cal.J2_offset.x
    z_rel = z_j4 - cal.J2_offset.z

    distSq = x_rel * x_rel + z_rel * z_rel
    dist = math.sqrt(distSq)

    # === Step 6: Reachability check ===
    if dist > g.maxReach + cal.solver['reachTolerance']:
        return IKSolution(
            q1=q1, q2=0, q3=0, q4=0, q5=0,
            branch='none',
            error=f"Out of reach: target distance {dist:.4f}m > max reach {g.maxReach:.4f}m"
        )
    if dist < g.minReach - cal.solver['reachTolerance']:
        return IKSolution(
            q1=q1, q2=0, q3=0, q4=0, q5=0,
            branch='none',
            error=f"Too close: target distance {dist:.4f}m < min reach {g.minReach:.4f}m"
        )

    # === Step 7: Law of cosines to find relative angle between links ===
    cosQ3rel = (distSq - g.L1*g.L1 - g.L2*g.L2) / (2*g.L1*g.L2)
    cosQ3rel = max(-1.0, min(1.0, cosQ3rel))
    q3rel_mag = math.acos(cosQ3rel)

    # === Step 8: 2R standard formula ===
    alpha = math.atan2(x_rel, z_rel)

    # === Step 9: Enumerate two elbow branches (with PHI angle compensation) ===
    candidates = []

    for sign in [+1, -1]:
        q3_rel = sign * q3rel_mag
        beta = math.atan2(g.L2 * math.sin(q3_rel), g.L1 + g.L2 * math.cos(q3_rel))

        # Link absolute direction angles
        theta1 = alpha - beta
        theta2 = theta1 + q3_rel

        # Joint angles = link direction angle - link's own deviation angle (PHI compensation)
        q2 = theta1 - g.PHI1
        q3 = q3_rel + g.PHI1 - g.PHI2
        q4 = target_pitch_rad - theta2 + g.PHI2 - g.PHI_TCP

        # Check joint limits
        lim = cal.jointLimits
        if (lim['q2'].min <= q2 <= lim['q2'].max and
            lim['q3'].min <= q3 <= lim['q3'].max and
            lim['q4'].min <= q4 <= lim['q4'].max):
            candidates.append(IKSolution(
                q1=q1, q2=q2, q3=q3, q4=q4, q5=0,
                branch='elbow-up' if sign > 0 else 'elbow-down',
                error=None
            ))

    if len(candidates) == 0:
        return IKSolution(
            q1=q1, q2=0, q3=0, q4=0, q5=0,
            branch='none',
            error='Both elbow configurations exceed joint limits'
        )

    # Prefer elbow-up configuration (larger |q3|)
    candidates.sort(key=lambda s: abs(s.q3), reverse=True)

    return candidates[0]


def solve_analytical_ik_auto_pitch(target_pos: Point3D,
                                   preferred_pitch_rad: float = 0.0,
                                   config=None,
                                   geom=None) -> IKSolution:
    """
    Analytical IK (Auto Pitch Mode)

    When user-specified pitch is unreachable, automatically searches for the closest feasible pitch.

    Args:
        target_pos: Target TCP position {x, y, z} (meters)
        preferred_pitch_rad: Preferred pitch angle (radians)
        config: Robot calibration parameters
        geom: Derived geometry

    Returns:
        IKSolution: IK solution with actualPitch, autoPitch, pitchDeviation fields
    """
    cal = config if config else ROBOT_CALIBRATION
    g = geom if geom else GEOM

    # Try user-specified pitch first
    direct = solve_analytical_ik(target_pos, preferred_pitch_rad, cal, g)
    if not direct.error:
        return IKSolution(
            q1=direct.q1, q2=direct.q2, q3=direct.q3, q4=direct.q4, q5=direct.q5,
            branch=direct.branch,
            actualPitch=preferred_pitch_rad,
            autoPitch=False,
            pitchDeviation=0.0,
            error=None
        )

    # Scan all pitches to find closest feasible solution to preferredPitch
    best_pitch = None
    best_dist = float('inf')
    best_sol = None

    for p_deg in range(-180, 181, 1):
        p_rad = math.radians(p_deg)
        sol = solve_analytical_ik(target_pos, p_rad, cal, g)
        if not sol.error:
            d = abs(p_rad - preferred_pitch_rad)
            if d < best_dist:
                best_dist = d
                best_pitch = p_rad
                best_sol = sol

    if best_sol:
        return IKSolution(
            q1=best_sol.q1, q2=best_sol.q2, q3=best_sol.q3, q4=best_sol.q4, q5=best_sol.q5,
            branch=best_sol.branch,
            actualPitch=best_pitch,
            autoPitch=True,
            pitchDeviation=best_dist,
            error=None
        )

    return IKSolution(
        q1=0, q2=0, q3=0, q4=0, q5=0,
        branch='none',
        error='Target point exceeds workspace'
    )


def solve_analytical_ik_ignore_pitch(target_pos: Point3D,
                                    config=None,
                                    geom=None,
                                    fk_callback=None) -> IKSolution:
    """
    Analytical IK (Ignore Pitch Mode)

    Scans all pitch angles and elbow branches, selects configuration with minimum FK error
    and highest elbow position.

    Args:
        target_pos: Target TCP position {x, y, z} (meters)
        config: Robot calibration parameters
        geom: Derived geometry
        fk_callback: FK validation callback function(joint_angles) -> Point3D or None

    Returns:
        IKSolution: Best IK solution with tcpError and elbowZ fields
    """
    cal = config if config else ROBOT_CALIBRATION
    g = geom if geom else GEOM

    def in_limits(v):
        return cal.jointLimits['q2'].min <= v <= cal.jointLimits['q2'].max

    # === Step 1-3: Coordinate transform to J1 frame ===
    dx_world = target_pos.x - cal.J1_origin.x
    dy_world = target_pos.y - cal.J1_origin.y
    dz_world = target_pos.z - cal.J1_origin.z

    # q1 compensates TCP's fixed Y offset (same as solve_analytical_ik)
    r_horiz = math.hypot(dx_world, dy_world)
    q1_base = math.atan2(dy_world, dx_world)
    if r_horiz > 1e-9:
        asin_arg = max(-1.0, min(1.0, g.Y_TCP / r_horiz))
        q1_base -= math.asin(asin_arg)

    c1 = math.cos(q1_base)
    s1 = math.sin(q1_base)
    x_j1 = c1 * dx_world + s1 * dy_world
    y_j1 = -s1 * dx_world + c1 * dy_world
    z_j1 = dz_world

    # === Y constraint check ===
    # q1 has compensated Y_TCP offset, ideally y_j1 ≈ Y_TCP, residual should be within tolerance
    if abs(y_j1 - g.Y_TCP) > cal.solver['yPlaneTolerance']:
        return IKSolution(
            q1=0, q2=0, q3=0, q4=0, q5=0,
            branch='none',
            error=f"Target Y residual={((y_j1-g.Y_TCP)*1000):.1f}mm in J1 frame exceeds workspace"
        )

    best_err = float('inf')
    best_pitch = None
    best_sol = None
    best_elbow_z = float('-inf')

    for p_deg in range(-180, 181, 1):
        p_rad = math.radians(p_deg)
        toolX = g.L_TCP * math.sin(p_rad)
        toolZ = g.L_TCP * math.cos(p_rad)
        x_j4 = x_j1 - toolX
        z_j4 = z_j1 - toolZ
        x_rel = x_j4 - cal.J2_offset.x
        z_rel = z_j4 - cal.J2_offset.z
        distSq = x_rel*x_rel + z_rel*z_rel
        cosQ3 = (distSq - g.L1*g.L1 - g.L2*g.L2) / (2*g.L1*g.L2)
        if abs(cosQ3) > 1:
            continue
        q3_mag = math.acos(cosQ3)
        alpha = math.atan2(x_rel, z_rel)

        for sgn in [+1, -1]:
            q3_rel = sgn * q3_mag
            beta = math.atan2(g.L2*math.sin(q3_rel), g.L1 + g.L2*math.cos(q3_rel))

            theta1 = alpha - beta
            theta2 = theta1 + q3_rel

            q2 = theta1 - g.PHI1
            q3 = q3_rel + g.PHI1 - g.PHI2
            q4 = p_rad - theta2 + g.PHI2 - g.PHI_TCP
            if not (in_limits(q2) and in_limits(q3) and in_limits(q4)):
                continue

            # Call FK callback to validate
            if fk_callback:
                tcp_actual = fk_callback({
                    'arm_joint1': q1_base,
                    'arm_joint2': q2,
                    'arm_joint3': q3,
                    'arm_joint4': q4,
                    'arm_joint5': 0
                })
                if not tcp_actual:
                    continue

                err = math.sqrt(
                    (tcp_actual.x - target_pos.x) ** 2 +
                    (tcp_actual.y - target_pos.y) ** 2 +
                    (tcp_actual.z - target_pos.z) ** 2
                )
            else:
                err = 0.0

            elbow_z = cal.J1_origin.z + cal.J2_offset.z + g.L1 * math.cos(q2)
            score = err - 0.5 * max(0, elbow_z)
            best_score = best_err - 0.5 * max(0, best_elbow_z)

            if score < best_score - 1e-6:
                best_err = err
                best_pitch = p_rad
                best_elbow_z = elbow_z
                best_sol = {
                    'q1': q1_base, 'q2': q2, 'q3': q3, 'q4': q4, 'q5': 0,
                    'branch': 'A' if sgn > 0 else 'B'
                }

    if best_sol:
        return IKSolution(
            q1=best_sol['q1'],
            q2=best_sol['q2'],
            q3=best_sol['q3'],
            q4=best_sol['q4'],
            q5=best_sol['q5'],
            branch=best_sol['branch'],
            actualPitch=best_pitch,
            autoPitch=True,
            pitchDeviation=0.0,
            tcpError=best_err,
            elbowZ=best_elbow_z,
            error=None
        )

    return IKSolution(
        q1=0, q2=0, q3=0, q4=0, q5=0,
        branch='none',
        error='Target point exceeds workspace'
    )


def solution_to_dict(solution: IKSolution) -> Dict[str, float]:
    """
    Convert IKSolution to joint angles dictionary

    Args:
        solution: IK solution

    Returns:
        Dictionary with joint angles
    """
    return {
        'arm_joint1': solution.q1,
        'arm_joint2': solution.q2,
        'arm_joint3': solution.q3,
        'arm_joint4': solution.q4,
        'arm_joint5': solution.q5,
    }


def solution_to_pwm(solution: IKSolution) -> List[int]:
    """
    Convert IKSolution directly to servo PWM values.

    IMPORTANT: The analytical IK solver produces joint angles (q1-q5) that are
    URDF joint angles, which map DIRECTLY to servo physical angles (direction=+1,
    offset=0, center=1500). NO pi/2 or sign transformation is applied here.

    Verification (target x=0.25,y=0,z=0.04,pitch=180°):
      q2=0.8707 rad -> PWM 1500 + 0.8707/0.002356 = 1870  ✓
      q3=1.4000 rad -> PWM 1500 + 1.4000/0.002356 = 2094  ✓
      q4=0.8694 rad -> PWM 1500 + 0.8694/0.002356 = 1869  ✓

    Args:
        solution: IK solution from analytical solver

    Returns:
        list: [ID0_pwm, ID1_pwm, ID2_pwm, ID3_pwm, ID4_pwm]
    """
    try:
        from .transform import angles_to_pwm_array
    except ImportError:
        from transform import angles_to_pwm_array

    angles = [solution.q1, solution.q2, solution.q3, solution.q4, solution.q5]
    return angles_to_pwm_array(angles)


if __name__ == '__main__':
    print("\n" + "=" * 80)
    print("Analytical IK Test")
    print("=" * 80)

    # Test 1: Forward reach position
    print("\nTest 1: Forward reach (x=0.25, y=0.0, z=0.15), pitch=0")
    target1 = Point3D(x=0.25, y=0.0, z=0.15)
    sol1 = solve_analytical_ik(target1, target_pitch_rad=0.0)
    if sol1.error:
        print(f"  Error: {sol1.error}")
    else:
        print(f"  Solution found: {sol1.branch}")
        print(f"    q1={math.degrees(sol1.q1):+.2f}°, q2={math.degrees(sol1.q2):+.2f}°, "
              f"q3={math.degrees(sol1.q3):+.2f}°, q4={math.degrees(sol1.q4):+.2f}°, q5={math.degrees(sol1.q5):+.2f}°")

    # Test 2: Auto pitch mode
    print("\nTest 2: Auto pitch (x=0.20, y=0.0, z=0.10), preferred pitch=-45°")
    target2 = Point3D(x=0.20, y=0.0, z=0.10)
    sol2 = solve_analytical_ik_auto_pitch(target2, preferred_pitch_rad=math.radians(-45))
    if sol2.error:
        print(f"  Error: {sol2.error}")
    else:
        print(f"  Solution found: {sol2.branch}")
        print(f"    Auto pitch: {sol2.autoPitch}, actual pitch: {math.degrees(sol2.actualPitch):.1f}°, "
              f"deviation: {math.degrees(sol2.pitchDeviation):.1f}°")
        print(f"    q1={math.degrees(sol2.q1):+.2f}°, q2={math.degrees(sol2.q2):+.2f}°, "
              f"q3={math.degrees(sol2.q3):+.2f}°, q4={math.degrees(sol2.q4):+.2f}°, q5={math.degrees(sol2.q5):+.2f}°")

    print("\n" + "=" * 80)
