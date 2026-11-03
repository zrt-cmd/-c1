#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
URDF 校正生成工具

读取手眼标定结果，自动修改 car1_description 包中的 URDF 文件：
  - 更新 camera_mount_joint 的 origin xyz 和 rpy
  - 备份原始 URDF 为 car1.urdf.bak

使用方式：
  ros2 run eye_hand_calibration generate_corrected_urdf
  # 或直接：python3 scripts/generate_corrected_urdf.py
"""

import sys
import os
import re
import glob
import math
import yaml
import shutil
import numpy as np
import argparse
from typing import Optional
import numpy as np


def find_latest_result(base_dir: Optional[str] = None) -> Optional[str]:
    """查找最新的手眼标定结果文件。"""
    if base_dir is None:
        base_dir = os.path.expanduser('~/.ros/eye_hand_calib')
    pattern = os.path.join(base_dir, 'calibration_*', 'results', 'hand_eye_transform.yaml')
    results = sorted(glob.glob(pattern))
    return results[-1] if results else None


def find_urdf(ros_ws: Optional[str] = None) -> Optional[str]:
    """定位 car1.urdf 文件（优先源码，安装目录会被 colcon build 覆盖）。"""
    candidates = []

    # 1. 源码优先（src 下的才是源头）
    ws = ros_ws or os.path.expanduser('~/ros2_ws')
    src_paths = [
        os.path.join(ws, 'src', 'xflz_eai_car1', 'simulations',
                     'car1_description', 'urdf', 'car1.urdf'),
    ]
    for p in src_paths:
        if os.path.exists(p):
            candidates.append(p)

    # 2. 降级到 install 目录（如果源码不存在）
    ament_prefix = os.environ.get('AMENT_PREFIX_PATH', '')
    for prefix in ament_prefix.split(':'):
        p = os.path.join(prefix, 'share', 'car1_description', 'urdf', 'car1.urdf')
        if os.path.exists(p) and p not in candidates:
            candidates.append(p)

    return candidates[0] if candidates else None


def quat_to_rpy(q: list) -> tuple:
    """
    四元数 [x,y,z,w] 转 RPY（roll, pitch, yaw），单位弧度。
    使用 ZYX 欧拉角约定（与 URDF 一致）。
    """
    x, y, z, w = q
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def update_urdf_joint(urdf_content: str,
                      joint_name: str,
                      xyz: list,
                      rpy: list) -> str:
    """
    替换 URDF 中指定 joint 的 origin xyz 和 rpy。
    使用正则精确替换，保留其他内容。
    """
    # 匹配 joint 块中的 origin 行
    # 例：<origin xyz="-0.043626 -0.025127 0.068237" rpy="0 0 -3.14038195" />
    xyz_str = f'{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}'
    rpy_str = f'{rpy[0]:.8f} {rpy[1]:.8f} {rpy[2]:.8f}'

    # 构造目标 joint 块的正则
    # 找到 joint name="..." 后的第一个 <origin ... />
    # 找到 joint name="..." 后的第一个 <origin
    # 捕获: g1=<joint...<origin + g2=空白(换行+空格) + g3=xyz间空白 + g4=结尾空白
    pattern = (
        r'(<joint\s[^>]*name=["\']{}\b[^>]*>.*?<origin)'
        r'(\s*)'         # g2: origin 和 xyz 之间的空白（换行+空格）
        r'xyz="([^"]*)"'  # g3: 旧的xyz值（后面不用，但保持分组计数）
        r'(\s*)'          # g4: xyz 和 rpy 之间的空白
        r'rpy="([^"]*)"'  # g5: 旧的rpy值
        r'(\s*/>)'        # g6: rpy 后的结尾
    ).format(re.escape(joint_name))

    replacement = (
        r'\g<1>'           # 保留 <joint...<origin
        r'\g<2>'           # 保留 origin 后的空格格式
        f'xyz="{xyz_str}"'  # 新 xyz
        r'\g<4>'           # 保留 xyz 和 rpy 间的空格格式
        f'rpy="{rpy_str}"'  # 新 rpy
        r'\g<6>'           # 保留结尾格式
    )

    new_content, count = re.subn(pattern, replacement, urdf_content, flags=re.DOTALL)
    return new_content, count


def main():
    parser = argparse.ArgumentParser(description='根据手眼标定结果更新 URDF')
    parser.add_argument('--result', type=str, default=None,
                        help='hand_eye_transform.yaml 路径（默认：自动查找最新）')
    parser.add_argument('--urdf', type=str, default=None,
                        help='car1.urdf 路径（默认：自动查找）')
    parser.add_argument('--dry-run', action='store_true',
                        help='只显示修改内容，不实际写入')
    args = parser.parse_args()

    print('=' * 60)
    print('  深度相机手眼标定 — URDF 更新工具')
    print('=' * 60)
    print()

    # 查找标定结果
    result_path = args.result or find_latest_result()
    if result_path is None:
        print('[错误] 未找到手眼标定结果！')
        print('  请先运行: ros2 launch eye_hand_calibration compute_calibration.launch.py')
        sys.exit(1)

    print(f'标定结果: {result_path}')

    with open(result_path, 'r') as f:
        result = yaml.safe_load(f)

    calib = result['hand_eye_calibration']
    trans = calib['transform']
    xyz_calib = [trans['translation']['x'],
                 trans['translation']['y'],
                 trans['translation']['z']]
    # 兼容两种字段名：rotation（旧）或 rotation_quat（新）
    rot_data = trans.get('rotation') or trans.get('rotation_quat', {})
    q_calib = [rot_data['x'], rot_data['y'], rot_data['z'], rot_data['w']]
    rpy_calib = list(quat_to_rpy(q_calib))

    print(f'\n标定结果：')
    print(f'  平移: x={xyz_calib[0]:.4f}  y={xyz_calib[1]:.4f}  z={xyz_calib[2]:.4f}  (m)')
    print(f'  RPY:  r={math.degrees(rpy_calib[0]):.2f}°  '
          f'p={math.degrees(rpy_calib[1]):.2f}°  '
          f'y={math.degrees(rpy_calib[2]):.2f}°')

    # 查找 URDF
    urdf_path = args.urdf or find_urdf()
    if urdf_path is None:
        print('\n[错误] 未找到 car1.urdf！')
        print('  请指定路径: --urdf /path/to/car1.urdf')
        sys.exit(1)

    print(f'\nURDF 文件: {urdf_path}')

    with open(urdf_path, 'r') as f:
        content = f.read()

    # 当前值（用于对比）
    print('\n当前 URDF 中的值：')
    m = re.search(
        r'<joint\s[^>]*name=["\']camera_mount_joint["\'][^>]*>.*?'
        r'<origin\s+xyz="([^"]*)"[^/]*/?>',
        content, re.DOTALL)
    if m:
        print(f'  camera_mount_joint xyz: {m.group(1)}')

    # 标定结果: arm_link4 → camera_depth_optical_frame (4×4 矩阵)
    # URDF 链: arm_link4 → camera_mount_joint → camera_mount_link
    #               → camera_depth_joint → camera_depth_link
    #               → camera_depth_optical_joint → camera_depth_optical_frame
    # 我们需要从标定结果中减去 camera_depth_joint 和 camera_depth_optical_joint 的固定变换

    # 读取 URDF 中的固定关节变换
    def parse_joint_origin(pattern, text):
        m = re.search(pattern, text, re.DOTALL)
        if m:
            xyz = [float(v) for v in m.group(1).split()]
            rpy = [float(v) for v in m.group(2).split()]
            return xyz, rpy
        return None, None

    depth_joint_xyz, depth_joint_rpy = parse_joint_origin(
        r'<joint\s[^>]*name=["\']camera_depth_joint["\'][^>]*>.*?'
        r'<origin\s+xyz="([^"]*)"\s+rpy="([^"]*)"',
        content)
    optical_joint_xyz, optical_joint_rpy = parse_joint_origin(
        r'<joint\s[^>]*name=["\']camera_depth_optical_joint["\'][^>]*>.*?'
        r'<origin\s+xyz="([^"]*)"\s+rpy="([^"]*)"',
        content)

    if depth_joint_xyz is None or optical_joint_xyz is None:
        print('[错误] 无法解析 URDF 中的 camera_depth_joint 或 camera_depth_optical_joint')
        sys.exit(1)

    # 构建固定变换矩阵 T_fixed = camera_mount → camera_optical
    def rpy_to_matrix(rpy):
        cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
        cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
        cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])
        return R

    T_calib = np.array(xyz_calib + [1])  # will use matrix_4x4 from result
    # Actually, use the full matrix from the result
    calib_matrix = np.array(result['hand_eye_calibration']['transform']['matrix_4x4'])

    # T_depth (camera_mount → camera_depth)
    R_depth = rpy_to_matrix(depth_joint_rpy)
    t_depth = np.array(depth_joint_xyz)
    T_depth = np.eye(4)
    T_depth[:3,:3] = R_depth
    T_depth[:3,3] = t_depth

    # T_optical (camera_depth → camera_optical)
    R_optical = rpy_to_matrix(optical_joint_rpy)
    t_optical = np.array(optical_joint_xyz)
    T_optical = np.eye(4)
    T_optical[:3,:3] = R_optical
    T_optical[:3,3] = t_optical

    # T_fixed = T_depth * T_optical (camera_mount → camera_optical)
    T_fixed = T_depth @ T_optical

    # 新的 camera_mount_joint = T_calib * inv(T_fixed)
    R_fixed = T_fixed[:3,:3]
    t_fixed = T_fixed[:3,3]
    T_fixed_inv = np.eye(4)
    T_fixed_inv[:3,:3] = R_fixed.T
    T_fixed_inv[:3,3] = -R_fixed.T @ t_fixed

    T_new_mount = calib_matrix @ T_fixed_inv

    new_xyz = [T_new_mount[0,3], T_new_mount[1,3], T_new_mount[2,3]]
    # 旋转矩阵 → RPY
    R_new = T_new_mount[:3,:3]
    # 旋转矩阵 → RPY (ZYX欧拉角)
    sy = math.sqrt(R_new[0,0]**2 + R_new[1,0]**2)
    if sy > 1e-6:
        new_rx = math.atan2(R_new[2,1], R_new[2,2])
        new_ry = math.atan2(-R_new[2,0], sy)
        new_rz = math.atan2(R_new[1,0], R_new[0,0])
    else:
        new_rx = math.atan2(-R_new[1,2], R_new[1,1])
        new_ry = math.atan2(-R_new[2,0], sy)
        new_rz = 0
    new_rpy = [new_rx, new_ry, new_rz]

    print('\n[注意] 更新 camera_mount_joint origin:')
    print(f'  当前 xyz: -0.043626 -0.025127 0.068237')
    print(f'  当前 rpy: 0 0 -3.14038195')
    print(f'  新 xyz: {new_xyz[0]:.6f} {new_xyz[1]:.6f} {new_xyz[2]:.6f}')
    print(f'  新 rpy: {new_rpy[0]:.8f} {new_rpy[1]:.8f} {new_rpy[2]:.8f}')

    xyz_calib = new_xyz
    rpy_calib = new_rpy

    if args.dry_run:
        print('\n[dry-run] 未写入文件')
        sys.exit(0)

    confirm = input('\n确认更新 URDF？(y/n): ').strip().lower()
    if confirm != 'y':
        print('已取消')
        sys.exit(0)

    # 备份原始 URDF
    bak_path = urdf_path + '.bak'
    shutil.copy2(urdf_path, bak_path)
    print(f'\n已备份原始文件: {bak_path}')

    # 更新 URDF
    new_content, count = update_urdf_joint(
        content, 'camera_mount_joint', xyz_calib, rpy_calib)

    if count == 0:
        print('[警告] 未找到 camera_mount_joint，请手动检查 URDF')
    else:
        with open(urdf_path, 'w') as f:
            f.write(new_content)
        print(f'URDF 已更新 ({count} 处修改)')
        print()
        print('下一步：')
        print('  colcon build --packages-select car1_description')
        print('  source install/setup.bash')
        print('  # 然后重启所有 ROS2 节点使新 URDF 生效')


if __name__ == '__main__':
    main()
