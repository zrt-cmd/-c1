#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Calibration Parameters for car1 Robotic Arm

All geometric parameters are centralized here for easy calibration after physical assembly.
Measurement method: Place the arm at zero position, use calipers/laser rangefinder to measure key points.
After modification, these parameters will be used by FK/IK solvers.

Based on test/car1_web_server/viewer/kinematics/config.js
Adapted for ROS2 Python implementation

Author: Level 3 Implementation
Date: 2026-06-08
"""
import math
from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass
class Point3D:
    """3D point coordinates"""
    x: float
    y: float
    z: float


@dataclass
class JointLimit:
    """Joint angle limits"""
    min: float
    max: float


class RobotCalibration:
    """Robot calibration parameters (all units in meters and radians)"""

    # ----- Base to J1 (yaw joint around Z axis) -----
    # J1 rotation axis position in base_footprint coordinate system
    J1_origin = Point3D(
        x=0.05662368,   # base_link(0.046624) + arm_base_link(0.01)
        y=0.0,
        z=0.129500      # base_joint(0.029) + arm_base(0.057) + arm_link1(0.0435)
    )

    # ----- J1 to J2 (J2 start point in J1 local frame) -----
    J2_offset = Point3D(
        x=0.000186,
        y=0.025,        # +25mm Y offset (cancels with J5's -25mm)
        z=0.059467
    )

    # ----- Link 1 (J2 → J3, in J2 local frame) -----
    link1 = Point3D(
        x=-0.0009,      # X offset (assembly deviation causes link not perfectly along Z)
        y=0.000249,     # Y offset (negligible)
        z=0.126141      # Main length (Z component)
    )

    # ----- Link 2 (J3 → J4, in J3 local frame) -----
    link2 = Point3D(
        x=-0.001266,
        y=0.000249,
        z=0.126138
    )

    # ----- End effector tool (J4 → TCP, through J5) -----
    # Combines J4→J5(0.000919, -0.025283, 0.074395) + J5→TCP(0.000697, 0, 0.076247)
    # and applies J5's fixed RPY(0, 0, -π) rotation
    toolFromJ4 = Point3D(
        x=0.000222,     # 0.000919 + R_z(-π)·0.000697 ≈ 0.000222
        y=-0.025283,    # J5's Y offset
        z=0.150642      # 0.074395 + 0.076247
    )

    # ----- Joint limits (URDF hard limits) -----
    jointLimits = {
        'q1': JointLimit(min=-math.pi, max=math.pi),
        'q2': JointLimit(min=-math.pi, max=math.pi),
        'q3': JointLimit(min=-math.pi, max=math.pi),
        'q4': JointLimit(min=-math.pi, max=math.pi),
        'q5': JointLimit(min=-math.pi, max=math.pi),
    }

    # ----- Solver parameters -----
    solver = {
        'reachTolerance': 1e-5,      # Reachability numerical tolerance (10μm)
        'yPlaneTolerance': 0.01      # Y plane constraint tolerance (10mm)
    }


@dataclass
class DerivedGeometry:
    """Derived geometric quantities from calibration parameters"""
    # Link effective lengths (X-Z plane projection)
    L1: float
    L2: float
    L_TCP: float

    # Link direction angles (deviation angle relative to parent frame +Z axis)
    PHI1: float
    PHI2: float
    PHI_TCP: float

    # TCP fixed Y offset in J1 frame (unaffected by J2/J3/J4 Y-axis rotations)
    Y_TCP: float

    # Workspace
    maxReach: float
    minReach: float


def derive_geometry(cal: RobotCalibration) -> DerivedGeometry:
    """
    Calculate derived geometric quantities from calibration parameters

    Args:
        cal: Robot calibration parameters

    Returns:
        Derived geometric quantities (link lengths, angles, workspace)
    """
    link1 = cal.link1
    link2 = cal.link2
    tool = cal.toolFromJ4

    L1 = math.hypot(link1.x, link1.z)
    L2 = math.hypot(link2.x, link2.z)
    L_TCP = math.hypot(tool.x, tool.z)

    # TCP fixed Y offset in J1 local frame
    # Since J2/J3/J4 all rotate around Y axis, Y component is unaffected by these rotations
    # = J2_offset.y + link1.y + link2.y + toolFromJ4.y
    Y_TCP = cal.J2_offset.y + link1.y + link2.y + tool.y

    return DerivedGeometry(
        # Link effective lengths (X-Z plane projection)
        L1=L1,
        L2=L2,
        L_TCP=L_TCP,

        # Link direction angles (deviation from parent +Z axis, for precise geometric algorithm)
        PHI1=math.atan2(link1.x, link1.z),
        PHI2=math.atan2(link2.x, link2.z),
        PHI_TCP=math.atan2(tool.x, tool.z),

        # TCP fixed Y offset in J1 frame (for q1 compensation, eliminates residual error)
        Y_TCP=Y_TCP,

        # Workspace
        maxReach=L1 + L2 + L_TCP,  # 修复：应包含 TCP 长度
        minReach=abs(L1 - L2)
    )


# Default calibration instance
ROBOT_CALIBRATION = RobotCalibration()

# Default derived geometry (calculated on module load)
GEOM = derive_geometry(ROBOT_CALIBRATION)


def print_calibration():
    """Print calibration parameters"""
    cal = ROBOT_CALIBRATION
    g = GEOM

    print("=" * 80)
    print("car1 Robot Calibration Parameters")
    print("=" * 80)

    print("\n[Base Geometry]")
    print(f"  J1 origin (base_footprint): ({cal.J1_origin.x:.6f}, {cal.J1_origin.y:.6f}, {cal.J1_origin.z:.6f}) m")
    print(f"  J2 offset (J1 local):        ({cal.J2_offset.x:.6f}, {cal.J2_offset.y:.6f}, {cal.J2_offset.z:.6f}) m")

    print("\n[Link Geometry]")
    print(f"  Link1 (J2→J3): ({cal.link1.x:.6f}, {cal.link1.y:.6f}, {cal.link1.z:.6f}) m")
    print(f"  Link2 (J3→J4): ({cal.link2.x:.6f}, {cal.link2.y:.6f}, {cal.link2.z:.6f}) m")
    print(f"  Tool (J4→TCP): ({cal.toolFromJ4.x:.6f}, {cal.toolFromJ4.y:.6f}, {cal.toolFromJ4.z:.6f}) m")

    print("\n[Derived Geometry]")
    print(f"  L1 (effective):     {g.L1:.6f} m")
    print(f"  L2 (effective):     {g.L2:.6f} m")
    print(f"  L_TCP (effective):  {g.L_TCP:.6f} m")
    print(f"  PHI1 (deviation):   {math.degrees(g.PHI1):+.4f}°")
    print(f"  PHI2 (deviation):   {math.degrees(g.PHI2):+.4f}°")
    print(f"  PHI_TCP (deviation):{math.degrees(g.PHI_TCP):+.4f}°")
    print(f"  Y_TCP (fixed):      {g.Y_TCP:.6f} m ({g.Y_TCP*1000:.2f} mm)")

    print("\n[Workspace]")
    print(f"  Max reach:  {g.maxReach:.6f} m ({g.maxReach*1000:.1f} mm)")
    print(f"  Min reach:  {g.minReach:.6f} m ({g.minReach*1000:.1f} mm)")

    print("\n[Joint Limits]")
    for joint_name, limits in cal.jointLimits.items():
        print(f"  {joint_name}: [{math.degrees(limits.min):+.1f}°, {math.degrees(limits.max):+.1f}°]")

    print("\n[Solver Parameters]")
    print(f"  Reach tolerance:   {cal.solver['reachTolerance']*1e6:.1f} μm")
    print(f"  Y plane tolerance: {cal.solver['yPlaneTolerance']*1000:.1f} mm")

    print("=" * 80)


if __name__ == '__main__':
    print_calibration()
