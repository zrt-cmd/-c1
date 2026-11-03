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

def parse_joints(robot_desc):
    import xml.etree.ElementTree as ET
    root = ET.fromstring(robot_desc)
    
    joints = []
    for joint in root.findall('.//joint'):
        joint_name = joint.get('name')
        joint_type = joint.get('type')
        
        if joint_type == 'revolute' or joint_type == 'continuous':
            origin = joint.find('origin')
            axis = joint.find('axis')
            limit = joint.find('limit')
            
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            
            if 'arm_joint' in joint_name:
                xyz = [float(x) for x in origin.get('xyz').split()] if origin is not None else [0, 0, 0]
                rpy = [float(x) for x in origin.get('rpy').split()] if origin is not None else [0, 0, 0]
                axis_xyz = [float(x) for x in axis.get('xyz').split()] if axis is not None else [0, 0, 1]
                lower = float(limit.get('lower', -3.14)) if limit is not None else -3.14
                upper = float(limit.get('upper', 3.14)) if limit is not None else 3.14
                
                joints.append({
                    'name': joint_name,
                    'parent': parent,
                    'child': child,
                    'xyz': xyz,
                    'rpy': rpy,
                    'axis': axis_xyz,
                    'lower': lower,
                    'upper': upper
                })
    
    return joints

def euler_to_rotation_matrix(roll, pitch, yaw):
    Rx = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])
    Ry = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])
    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])
    return Rz @ Ry @ Rx

def compute_fk(joint_positions):
    robot_desc = load_urdf()
    joints = parse_joints(robot_desc)
    
    T = np.eye(4)
    
    print("=" * 60)
    print("Forward Kinematics Demo for car1 arm")
    print("=" * 60)
    print(f"\n关节数量: {len(joints)}")
    print("\n关节列表:")
    for i, j in enumerate(joints):
        print(f"  {i+1}. {j['name']}: {j['parent']} -> {j['child']}")
        print(f"     关节限位: [{j['lower']:.2f}, {j['upper']:.2f}] rad")
    
    print("\n" + "=" * 60)
    print("Forward Kinematics Calculation")
    print("=" * 60)
    
    for i, joint in enumerate(joints):
        theta = joint_positions[i] if i < len(joint_positions) else 0.0
        
        Tx = np.eye(4)
        Tx[0, 3] = joint['xyz'][0]
        Tx[1, 3] = joint['xyz'][1]
        Tx[2, 3] = joint['xyz'][2]
        
        R = euler_to_rotation_matrix(joint['rpy'][0], joint['rpy'][1], joint['rpy'][2])
        Tx[0:3, 0:3] = R
        
        Ra = np.eye(4)
        if joint['axis'][0] != 0:
            c, s = math.cos(theta), math.sin(theta)
            Ra[0:3, 0:3] = np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        elif joint['axis'][1] != 0:
            c, s = math.cos(theta), math.sin(theta)
            Ra[0:3, 0:3] = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        elif joint['axis'][2] != 0:
            c, s = math.cos(theta), math.sin(theta)
            Ra[0:3, 0:3] = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        
        T = T @ Tx @ Ra
        
        print(f"\n{joint['name']} (theta={theta:.4f} rad):")
        print(f"  位置: [{T[0,3]:.4f}, {T[1,3]:.4f}, {T[2,3]:.4f}] m")
    
    final_pos = [T[0,3], T[1,3], T[2,3]]
    print("\n" + "=" * 60)
    print(f"末端执行器位置: [{final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f}] m")
    print("=" * 60)
    
    return final_pos

def main():
    print("\n" + "=" * 60)
    print("car1 机械臂运动学正解仿真")
    print("=" * 60)
    print("\nURDF模型: car1.urdf")
    print("机械臂关节: arm_joint1 ~ arm_joint5 (共5个自由度)")
    print("\n" + "-" * 60)
    
    joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    print("\n测试1: 所有关节角度为0")
    compute_fk(joint_positions)
    
    print("\n" + "-" * 60)
    print("\n测试2: 各关节旋转45度")
    joint_positions = [math.pi/4, math.pi/4, math.pi/4, math.pi/4, math.pi/4]
    compute_fk(joint_positions)
    
    print("\n" + "-" * 60)
    print("\n测试3: 模拟实际抓取姿态")
    joint_positions = [0.0, -0.5, -0.8, 0.3, 0.0]
    compute_fk(joint_positions)

if __name__ == '__main__':
    main()
