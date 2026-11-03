#!/usr/bin/env python3
import os
import sys
import math
import numpy as np

def load_urdf():
    from ament_index_python.packages import get_package_share_directory
    pkg_share = get_package_share_directory('car1_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'car1.urdf')
    
    with open(urdf_file, 'r') as f:
        return f.read()

def extract_link_lengths(joints):
    link_lengths = {
        'd1': 0.1005,
        'a2': 0.12614,
        'a3': 0.12614,
        'a4': 0.074395,
        'd5': 0.0
    }
    return link_lengths

def compute_ik(target_pos, target_orient=None, link_lengths=None):
    if link_lengths is None:
        link_lengths = extract_link_lengths(None)

    x, y, z = target_pos

    d1 = link_lengths.get('d1', 0.1005)
    a2 = link_lengths.get('a2', 0.12614)
    a3 = link_lengths.get('a3', 0.12614)
    a4 = link_lengths.get('a4', 0.074395)

    theta1 = math.atan2(y, x)

    r = math.sqrt(x**2 + y**2)
    pz = z - d1

    # Wrist decoupling: end-effector horizontal (pitch=0), so L4 is horizontal
    wr = r - a4
    wz = pz

    # Reachability check
    D = wr**2 + wz**2
    max_reach = a2 + a3
    if math.sqrt(D) > max_reach * 0.98:
        return None, "目标位置超出机械臂可达范围"
    if math.sqrt(D) < abs(a2 - a3) * 1.05:
        return None, "目标位置在机械臂不可达范围内"

    # Two-link IK for wrist position
    cos_theta3 = (D - a2**2 - a3**2) / (2 * a2 * a3)
    if abs(cos_theta3) > 1.0:
        return None, "目标位置超出机械臂可达范围"

    theta3 = math.acos(cos_theta3)

    theta2 = math.atan2(wz, wr) - math.atan2(a3 * math.sin(theta3), a2 + a3 * math.cos(theta3))

    # Keep end-effector horizontal
    theta4 = -(theta2 + theta3)

    return {
        'arm_joint1': theta1,
        'arm_joint2': theta2,
        'arm_joint3': theta3,
        'arm_joint4': theta4,
        'arm_joint5': 0.0
    }, None

def compute_fk_simple(joint_positions, link_lengths=None):
    if link_lengths is None:
        link_lengths = extract_link_lengths(None)
    
    d1 = link_lengths.get('d1', 0.1005)
    a2 = link_lengths.get('a2', 0.12614)
    a3 = link_lengths.get('a3', 0.12614)
    a4 = link_lengths.get('a4', 0.074395)
    d5 = link_lengths.get('d5', 0.0)
    
    theta1 = joint_positions.get('arm_joint1', 0)
    theta2 = joint_positions.get('arm_joint2', 0)
    theta3 = joint_positions.get('arm_joint3', 0)
    theta4 = joint_positions.get('arm_joint4', 0)
    
    L2 = a2
    L3 = a3
    L4 = a4 + d5
    
    x2 = L2 * math.cos(theta2)
    y2 = L2 * math.sin(theta2)
    
    total_theta23 = theta2 + theta3
    x3 = x2 + L3 * math.cos(total_theta23)
    y3 = y2 + L3 * math.sin(total_theta23)
    
    total_theta234 = theta2 + theta3 + theta4
    x4 = x3 + L4 * math.cos(total_theta234)
    y4 = y3 + L4 * math.sin(total_theta234)
    
    x = x4 * math.cos(theta1)
    y = x4 * math.sin(theta1)
    z = y4 + d1
    
    return np.array([x, y, z])

def test_ik():
    print("=" * 60)
    print("Inverse Kinematics Demo for car1 arm (Analytical Solution)")
    print("=" * 60)
    
    link_lengths = extract_link_lengths(None)
    
    print(f"\n连杆参数 (简化模型):")
    print(f"  d1 (底座高度): {link_lengths.get('d1', 0):.4f} m")
    print(f"  a2 (大臂长度): {link_lengths.get('a2', 0):.4f} m")
    print(f"  a3 (小臂长度): {link_lengths.get('a3', 0):.4f} m")
    print(f"  a4 (腕部长度): {link_lengths.get('a4', 0):.4f} m")
    print(f"  d5 (末端偏移): {link_lengths.get('d5', 0):.4f} m")
    total = link_lengths.get('a2', 0) + link_lengths.get('a3', 0) + link_lengths.get('a4', 0)
    print(f"  总臂展: {total:.4f} m")
    
    print("\n" + "=" * 60)
    print("IK 求解测试")
    print("=" * 60)
    
    test_positions = [
        ([0.25, 0.0, 0.20], "正前方"),
        ([0.28, 0.0, 0.18], "前方中位"),
        ([0.20, 0.10, 0.18], "右前方"),
        ([0.20, -0.10, 0.18], "左前方"),
        ([0.15, 0.0, 0.25], "上方"),
    ]
    
    for target_pos, description in test_positions:
        print(f"\n目标位置: {description}")
        print(f"  目标坐标: x={target_pos[0]:.3f}, y={target_pos[1]:.3f}, z={target_pos[2]:.3f}")
        
        joint_solution, error = compute_ik(target_pos, link_lengths=link_lengths)
        
        if joint_solution is not None:
            print(f"  关节角度解:")
            for joint_name, angle in joint_solution.items():
                deg = math.degrees(angle)
                print(f"    {joint_name}: {angle:.4f} rad ({deg:.2f}°)")
            
            fk_pos = compute_fk_simple(joint_solution, link_lengths)
            print(f"  FK 验证位置: x={fk_pos[0]:.4f}, y={fk_pos[1]:.4f}, z={fk_pos[2]:.4f}")
            error_dist = math.sqrt(
                (fk_pos[0] - target_pos[0])**2 +
                (fk_pos[1] - target_pos[1])**2 +
                (fk_pos[2] - target_pos[2])**2
            )
            print(f"  位置误差: {error_dist:.6f} m")
        else:
            print(f"  求解失败: {error}")
    
    print("\n" + "=" * 60)
    print("关节限位检查")
    print("=" * 60)
    
    for target_pos, description in test_positions[:2]:
        joint_solution, error = compute_ik(target_pos, link_lengths=link_lengths)
        if joint_solution is not None:
            print(f"\n目标: {description}")
            for joint_name, angle in joint_solution.items():
                deg = math.degrees(angle)
                limit_rad = 2.09
                limit_deg = math.degrees(limit_rad)
                status = "OK" if abs(deg) <= limit_deg else "超出限位!"
                print(f"  {joint_name}: {deg:7.2f}°  [{-limit_deg:+.1f}°, {limit_deg:.1f}°] {status}")

def main():
    test_ik()

if __name__ == '__main__':
    test_ik()
