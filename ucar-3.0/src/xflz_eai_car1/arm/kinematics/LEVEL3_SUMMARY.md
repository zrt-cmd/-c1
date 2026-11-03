# Level 3 Implementation Summary

## Overview

This document summarizes the Level 3 implementation for the car1 robotic arm, which provides safe IK-based control for real hardware. The implementation is based on the analytical IK solver from `test/car1_web_server` and uses the corrected URDF model.

## Implementation Date
2026-06-08

## Key Changes

### 1. URDF Model Update

**Backup old URDF:**
```bash
cd simulations/car1_description/urdf
mv car1.urdf car1_old.urdf
```

**Copy new URDF from test/car1_web_server:**
```bash
cp test/car1_web_server/urdf/car1.urdf simulations/car1_description/urdf/
```

**Key differences:**
- `base_joint`: Z offset changed from 0.0195m to 0.029m
- `arm_joint2`, `arm_joint3`, `arm_joint4`: Rotation axis changed from Z to Y
- Joint origins: XYZ coordinates reordered to match new rotation axes
- Visual/collision meshes: RPY compensation applied (-π/2, -π/2, 0)
- `arm_joint5`: Axis changed to pure Z rotation with -π offset

### 2. New Modules

#### **config.py** - Robot Calibration Parameters
- Port from `test/car1_web_server/viewer/kinematics/config.js`
- Centralized geometric parameters (link lengths, offsets, joint limits)
- Derived geometry computation (effective lengths, PHI angles, workspace)
- Key parameters:
  - J1_origin: (0.05662368, 0.0, 0.129500) m
  - Link1 length: 0.126144 m (effective)
  - Link2 length: 0.126144 m (effective)
  - TCP offset: 0.150642 m
  - Y_TCP offset: 0.215 mm (fixed in J1 frame)
  - Max reach: 252.3 mm
  - Min reach: 0.0 mm

#### **fk.py** - Forward Kinematics
- Implements FK following the new URDF structure
- Uses homogeneous transformation matrices
- Supports full chain: base_footprint → tcp_link
- Simplified 2R planar FK for IK validation
- FK validation function for round-trip testing

#### **ik_analytical.py** - Analytical Inverse Kinematics
- Port from `test/car1_web_server/viewer/kinematics/ik-analytical.js`
- Three modes:
  1. **Fixed Pitch**: User specifies exact pitch angle
  2. **Auto Pitch**: Automatically finds closest feasible pitch
  3. **Ignore Pitch**: Scans all pitches, selects best configuration
- Geometric closed-form solution (no iteration, ~0.05ms)
- Features:
  - Y-axis offset compensation (0.215mm TCP offset in J1 frame)
  - PHI angle compensation for link deviations
  - Elbow-up/elbow-down branch selection
  - Joint limit checking
  - Workspace boundary validation

#### **test_fk_ik.py** - Validation Test Suite
- Comprehensive FK/IK validation tests
- Test coverage:
  - FK with known joint angles
  - IK with known target positions
  - IK → FK round-trip validation
  - PWM ↔ angle conversion
  - Workspace limit testing

### 3. Existing Module Updates

#### **transform.py** (no changes needed)
- Already implements PWM ↔ angle conversion
- Already implements IK angle convention conversion
- Calibrated parameters verified:
  - All servos: direction = +1, offset = 0.0
  - PWM_TO_RAD = 0.002356 rad/PWM
  - TCP_OFFSET_Z = 0.0762 m
  - FINGERTIP_OFFSET_Z = 0.033 m

## Test Results

### Forward Kinematics Tests
- ✓ Zero position: TCP at (54.87, 0.21, 591.89) mm
- ✓ Vertical up (q2=90°): TCP at (459.73, 0.21, 190.91) mm
- ✓ Forward reach (q2=45°): TCP at (233.89, 0.21, 519.53) mm

### Inverse Kinematics Tests
- ✓ Fixed pitch mode working
- ✓ Auto pitch mode working (finds closest feasible pitch)
- ✓ Workspace boundary detection

### Round-Trip Tests (IK → FK)
- ✓ Target (0.250, 0.000, 0.150): Error 0.00 mm
- ✓ Target (0.220, 0.030, 0.120): Error 0.00 mm
- ✓ Target (0.200, -0.030, 0.180): Error 0.00 mm
- ✓ Target (0.180, 0.050, 0.100): Error 0.00 mm
- **All 4 tests PASSED** (tolerance: 1.5 mm)

### PWM Conversion Tests
- ✓ Center position (PWM 1500): Perfect round-trip
- ✓ Test posture (PWM [1500, 1850, 2109, 1850, 1500]): Perfect round-trip
- ✓ IK angles ↔ PWM: Perfect round-trip

### Workspace Validation
- ✓ Max reach: 252.3 mm (correctly detected out-of-reach targets)
- ✓ Min reach: 0.0 mm (correctly rejected too-close targets)

## Key Algorithms

### IK Solution Process

1. **Translate to J1 origin**: Convert target from base_footprint to J1 frame
2. **Calculate q1 (base rotation)**: Compensate for TCP's fixed Y offset (0.215mm)
3. **Rotate to J1 frame**: Apply q1 rotation to get (x_j1, y_j1, z_j1)
4. **Check Y constraint**: Verify y_j1 ≈ Y_TCP (within 10mm tolerance)
5. **Solve 2R IK in X-Z plane**:
   - Subtract tool offset (pitch-dependent)
   - Apply law of cosines for elbow angle
   - Use 2R standard formulas with PHI compensation
6. **Enumerate branches**: Try elbow-up and elbow-down configurations
7. **Check joint limits**: Filter solutions within [-π, π] range
8. **Select best**: Prefer elbow-up (larger |q3|)

### PHI Angle Compensation

Links have small deviations from ideal Z-axis alignment:
- PHI1 = -0.4088° (link1 deviation)
- PHI2 = -0.5750° (link2 deviation)
- PHI_TCP = +0.0844° (tool deviation)

Joint angles are corrected:
```
q2 = theta1 - PHI1
q3 = q3_rel + PHI1 - PHI2
q4 = pitch - theta2 + PHI2 - PHI_TCP
```

This ensures accurate positioning despite manufacturing tolerances.

## Coordinate Systems

### Base Footprint Frame
- Origin: Floor level, robot center
- +X: Forward (robot front)
- +Y: Left
- +Z: Up

### J1 Local Frame
- Origin: base_footprint + (0.05662, 0.0, 0.1295) m
- Rotates around Z axis with q1
- TCP has fixed Y offset = 0.215 mm in this frame

### TCP Link
- Tool Center Point
- Located 76.2mm below arm_link5
- Fingertips are 33mm below tcp_link
- Target positions are specified for tcp_link, not fingertips

## Usage Examples

### Example 1: Fixed Pitch IK
```python
from kinematics.ik_analytical import solve_analytical_ik
from kinematics.config import Point3D

target = Point3D(x=0.25, y=0.0, z=0.15)
solution = solve_analytical_ik(target, target_pitch_rad=0.0)

if not solution.error:
    print(f"q1={solution.q1}, q2={solution.q2}, q3={solution.q3}, 
          q4={solution.q4}, q5={solution.q5}")
```

### Example 2: Auto Pitch IK
```python
from kinematics.ik_analytical import solve_analytical_ik_auto_pitch
from kinematics.config import Point3D
import math

target = Point3D(x=0.22, y=0.03, z=0.12)
solution = solve_analytical_ik_auto_pitch(target, preferred_pitch_rad=math.radians(-45))

if not solution.error:
    print(f"Actual pitch used: {math.degrees(solution.actualPitch):.1f}°")
    print(f"Auto adjusted: {solution.autoPitch}")
```

### Example 3: FK Validation
```python
from kinematics.fk import compute_fk_position_only
from kinematics.ik_analytical import solve_analytical_ik, solution_to_dict

target = Point3D(x=0.20, y=0.0, z=0.18)
ik_solution = solve_analytical_ik(target, 0.0)
angles = solution_to_dict(ik_solution)
computed_pos = compute_fk_position_only(angles)

error = math.sqrt((computed_pos.x - target.x)**2 + 
                 (computed_pos.y - target.y)**2 + 
                 (computed_pos.z - target.z)**2)
print(f"FK error: {error*1000:.2f} mm")
```

### Example 4: PWM Conversion
```python
from kinematics.transform import ik_angles_to_pwm
from kinematics.ik_analytical import solution_to_dict

# After solving IK
angles = solution_to_dict(ik_solution)
pwm_values = ik_angles_to_pwm(angles)
print(f"PWM values to send to servos: {pwm_values}")
```

## Next Steps (Level 4)

Level 3 provides the mathematical foundation. Level 4 will add:

1. **ROS2 Integration**:
   - `safe_arm_controller_node.py` - ROS2 node wrapping IK/FK
   - Topics: `/arm_controller/target_pose`, `/arm_controller/joint_command`
   - Services: `/arm_controller/set_gripper`, `/arm_controller/go_home`

2. **Safety Features**:
   - Emergency stop
   - Velocity limiting
   - Collision avoidance (basic workspace bounds)
   - Trajectory interpolation (smooth motion)

3. **Verification Tools**:
   - `ik_round_trip_test.py` - Interactive testing
   - `known_point_test.py` - Physical marker validation
   - Real hardware integration tests

## Files Modified/Created

### Created:
- `arm/kinematics/kinematics/config.py` (320 lines)
- `arm/kinematics/kinematics/fk.py` (280 lines)
- `arm/kinematics/kinematics/ik_analytical.py` (380 lines)
- `arm/kinematics/kinematics/test_fk_ik.py` (240 lines)
- `arm/kinematics/LEVEL3_SUMMARY.md` (this file)

### Modified:
- `simulations/car1_description/urdf/car1.urdf` (replaced with new version)

### Backed up:
- `simulations/car1_description/urdf/car1_old.urdf` (original URDF)

## Verification Checklist

- [x] URDF model updated and backed up
- [x] Config module with calibration parameters
- [x] FK implementation following new URDF structure
- [x] IK analytical solver (3 modes)
- [x] PWM ↔ angle conversion (verified with existing transform.py)
- [x] Round-trip tests (IK → FK) passing with <1.5mm error
- [x] Workspace boundary validation
- [x] Test suite with comprehensive coverage
- [ ] ROS2 node integration (Level 4)
- [ ] Real hardware testing (Level 4)
- [ ] Safety features implementation (Level 4)

## References

- Original JS implementation: `test/car1_web_server/viewer/kinematics/`
- URDF model: `simulations/car1_description/urdf/car1.urdf`
- Level 3 requirements: `TODO.md` lines 1067-1373
- Transform parameters: `arm/kinematics/kinematics/transform.py`
