#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线手眼标定计算脚本

读取 data_collector 采集的数据，求解：
  1. 手眼变换矩阵（AX=XB）
  2. 深度线性校正参数

结果保存到 session 目录，并自动更新 config/depth_correction.yaml。

使用方式：
  # 使用最新会话（自动查找）
  ros2 run eye_hand_calibration compute_hand_eye_transform

  # 指定会话目录
  ros2 run eye_hand_calibration compute_hand_eye_transform \\
      --session ~/.ros/eye_hand_calib/calibration_20260729_143022
"""

import sys
import os
import argparse
import yaml
import json
import math
import glob
from datetime import datetime
from typing import Optional
import numpy as np


def find_latest_session(base_dir: Optional[str] = None) -> Optional[str]:
    """查找最新的标定会话目录。"""
    if base_dir is None:
        base_dir = os.path.expanduser('~/.ros/eye_hand_calib')
    sessions = sorted(glob.glob(os.path.join(base_dir, 'calibration_*')))
    return sessions[-1] if sessions else None


def load_session_data(session_dir: str):
    """加载会话数据（poses.yaml 和 detections.yaml）。"""
    poses_path = os.path.join(session_dir, 'poses.yaml')
    det_path = os.path.join(session_dir, 'detections.yaml')

    if not os.path.exists(poses_path):
        raise FileNotFoundError(f'未找到 poses.yaml: {poses_path}')
    if not os.path.exists(det_path):
        raise FileNotFoundError(f'未找到 detections.yaml: {det_path}')

    with open(poses_path, 'r') as f:
        poses_data = yaml.safe_load(f)
    with open(det_path, 'r') as f:
        det_data = yaml.safe_load(f)

    return poses_data, det_data


def main():
    parser = argparse.ArgumentParser(description='手眼标定计算工具')
    parser.add_argument('--session', type=str, default=None,
                        help='标定会话目录（默认：自动查找最新）')
    parser.add_argument('--algorithm', type=str, default='tsai',
                        choices=['tsai', 'horaud'],
                        help='求解算法 (默认: tsai)')
    parser.add_argument('--no-depth-correction', action='store_true',
                        help='跳过深度校正拟合')
    parser.add_argument('--pair-mode', type=str, default='all',
                        choices=['consecutive', 'all', 'first-ref'],
                        help='相对变换构造方式: consecutive=相邻对, all=所有非冗余对(默认), first-ref=以首姿态为基准')
    parser.add_argument('--min-rotation', type=float, default=8.0,
                        help='最小旋转角阈值(度)，过滤旋转太小的对（默认8.0）')
    parser.add_argument('--auto-outlier', action='store_true',
                        help='启用自动离群点检测（leave-one-out 交叉验证）')
    parser.add_argument('--outlier-trans', type=float, default=100.0,
                        help='离群平移阈值(mm)，默认100')
    parser.add_argument('--outlier-rot', type=float, default=30.0,
                        help='离群旋转阈值(度)，默认30')
    args = parser.parse_args()

    print('=' * 60)
    print('  深度相机手眼标定 — 标定计算工具')
    print('=' * 60)
    print()

    # 查找会话目录
    session_dir = args.session or find_latest_session()
    if session_dir is None:
        print('[错误] 未找到标定会话数据！')
        print('  请先运行: ros2 launch eye_hand_calibration collect_data.launch.py')
        sys.exit(1)

    print(f'使用会话: {session_dir}')
    print()

    # 加载数据
    try:
        poses_data, det_data = load_session_data(session_dir)
    except FileNotFoundError as e:
        print(f'[错误] {e}')
        sys.exit(1)

    records = poses_data.get('records', [])
    det_records = det_data.get('records', [])
    # 按 index 建索引，合并两个 YAML 的数据
    det_lookup = {r['index']: r for r in det_records}
    print(f'已加载 {len(records)} 组标定数据')

    if len(records) < 3:
        print(f'[错误] 数据不足！需要至少 3 组，当前只有 {len(records)} 组')
        sys.exit(1)

    # 导入求解器
    try:
        from eye_hand_calibration.hand_eye_solver import (
            solve_hand_eye_tsai,
            solve_hand_eye_horaud,
            fit_depth_correction,
            compute_relative_transforms,
            build_relative_motions_all_pairs,
            detect_outlier_poses,
            evaluate_calibration,
        )
        from eye_hand_calibration.utils import (
            make_transform_matrix,
            matrix_to_xyz_quat,
            quat_to_rotation_matrix,
        )
    except ImportError as e:
        print(f'[错误] 无法导入标定模块: {e}')
        print('  请先编译: colcon build --packages-select eye_hand_calibration')
        sys.exit(1)

    # 提取变换矩阵（合并 poses.yaml 和 detections.yaml）
    print('提取变换矩阵...')
    ee_transforms = []
    tag_transforms = []
    measured_depths = []
    reference_depths = []

    for i, rec in enumerate(records):
        try:
            # 机械臂末端变换 base_link → arm_link4（来自 poses.yaml）
            ee_T = np.array(rec['ee_transform']).reshape(4, 4)

            # AprilTag 检测数据（来自 detections.yaml，按 index 对齐）
            det_rec = det_lookup.get(rec.get('index', i), {})
            tag_data = det_rec.get('tag_in_camera')
            if tag_data is None:
                print(f'  [跳过] 第 {i+1} 组：tag_in_camera 为空（检测失败）')
                continue
            tag_T = np.array(tag_data).reshape(4, 4)

            # 有效性检查：标定时相机始终朝下，arm_link4 的 Z 轴 z 分量应为负（指向地面）
            ee_z_dir = ee_T[2, 2]
            if ee_z_dir > -0.3:
                print(f'  [跳过] 第 {i+1} 组：ee_transform 异常（Z轴朝上={ee_z_dir:.3f}，应为负值）')
                continue

            ee_transforms.append(ee_T)
            tag_transforms.append(tag_T)

            # 深度值
            depth_raw = det_rec.get('depth_raw_m')
            depth_pnp = det_rec.get('depth_pnp_m')
            if depth_raw is not None and depth_pnp is not None:
                measured_depths.append(depth_raw)
                reference_depths.append(depth_pnp)

            print(f'  [{i+1:2d}] ee 平移: ({ee_T[0,3]:.3f}, {ee_T[1,3]:.3f}, {ee_T[2,3]:.3f}) '
                  f'| tag 深度: {tag_T[2,3]:.3f}m')
        except Exception as e:
            print(f'  [跳过] 第 {i+1} 组数据格式错误: {e}')

    if len(ee_transforms) < 3:
        print(f'[错误] 有效数据不足 ({len(ee_transforms)} 组)')
        sys.exit(1)

    print(f'\n有效数据: {len(ee_transforms)} 组')

    # 自动离群点检测
    if args.auto_outlier:
        print(f'\n执行离群点检测（leave-one-out 交叉验证）...')
        ee_transforms, tag_transforms, excluded = detect_outlier_poses(
            ee_transforms, tag_transforms,
            method=args.algorithm,
            min_rotation_deg=args.min_rotation,
            threshold_translation_mm=args.outlier_trans,
            threshold_rotation_deg=args.outlier_rot,
        )
        if len(ee_transforms) < 3:
            print(f'[错误] 离群剔除后有效数据不足')
            sys.exit(1)

    # 计算相对变换
    print(f'\n计算相对变换对（模式: {args.pair_mode}）...')
    if args.pair_mode == 'all':
        A_list, B_list = build_relative_motions_all_pairs(
            ee_transforms, tag_transforms,
            min_rotation_deg=args.min_rotation)
    elif args.pair_mode == 'first-ref':
        A_list, B_list = [], []
        from eye_hand_calibration.hand_eye_solver import invert_transform
        for i in range(1, len(ee_transforms)):
            A = invert_transform(ee_transforms[i]) @ ee_transforms[0]
            B = tag_transforms[i] @ invert_transform(tag_transforms[0])
            cos_a = (np.trace(A[:3,:3]) - 1) / 2
            cos_a = np.clip(cos_a, -1.0, 1.0)
            rot = math.degrees(math.acos(cos_a))
            if rot >= args.min_rotation:
                A_list.append(A)
                B_list.append(B)
    else:
        A_list, B_list = compute_relative_transforms(ee_transforms, tag_transforms)
    print(f'生成 {len(A_list)} 对相对变换')

    # 求解手眼变换
    print(f'\n使用 {args.algorithm.upper()} 算法求解手眼变换...')
    if args.algorithm == 'tsai':
        X = solve_hand_eye_tsai(A_list, B_list)
    else:
        X = solve_hand_eye_horaud(A_list, B_list)

    xyz, quat = matrix_to_xyz_quat(X)
    print(f'\n手眼变换结果（arm_link4 → camera_depth_optical_frame）:')
    print(f'  平移: x={xyz[0]:.4f}m  y={xyz[1]:.4f}m  z={xyz[2]:.4f}m')
    print(f'  四元数: x={quat[0]:.4f}  y={quat[1]:.4f}  z={quat[2]:.4f}  w={quat[3]:.4f}')

    # 评估标定质量
    print('\n评估标定质量...')
    eval_result = evaluate_calibration(A_list, B_list, X)
    print(f'  旋转误差 (均值): {eval_result["mean_rotation_error_deg"]:.3f}°')
    print(f'  旋转误差 (最大): {eval_result["max_rotation_error_deg"]:.3f}°')
    print(f'  平移误差 (均值): {eval_result["mean_translation_error_mm"]:.2f}mm')
    print(f'  平移误差 (最大): {eval_result["max_translation_error_mm"]:.2f}mm')

    quality = '优秀 ✓' if eval_result['mean_translation_error_mm'] < 5.0 else \
              '合格 ✓' if eval_result['mean_translation_error_mm'] < 10.0 else '需要重新标定 ✗'
    print(f'  标定质量: {quality}')

    # 深度校正
    depth_result = None
    if not args.no_depth_correction and len(measured_depths) >= 3:
        print('\n拟合深度校正模型...')
        depth_result = fit_depth_correction(measured_depths, reference_depths)
        print(f'  scale  = {depth_result["scale"]:.6f}')
        print(f'  offset = {depth_result["offset"]*1000:.3f} mm')
        print(f'  RMSE   = {depth_result["rmse"]*1000:.3f} mm')
        print(f'  R²     = {depth_result["r_squared"]:.4f}')
    else:
        print('\n跳过深度校正（数据不足或已禁用）')

    # 保存结果
    results_dir = os.path.join(session_dir, 'results')
    os.makedirs(results_dir, exist_ok=True)
    now_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

    # 手眼变换结果
    he_result = {
        'hand_eye_calibration': {
            'algorithm': args.algorithm,
            'transform': {
                'parent_frame': 'arm_link4',
                'child_frame': 'camera_depth_optical_frame',
                'matrix_4x4': X.tolist(),
                'translation': {'x': float(xyz[0]), 'y': float(xyz[1]), 'z': float(xyz[2])},
                'rotation_quat': {
                    'x': float(quat[0]), 'y': float(quat[1]),
                    'z': float(quat[2]), 'w': float(quat[3])
                },
            },
            'evaluation': {
                'num_poses': len(ee_transforms),
                'num_pairs': len(A_list),
                'rotation_error_mean_deg': eval_result['mean_rotation_error_deg'],
                'rotation_error_max_deg': eval_result['max_rotation_error_deg'],
                'translation_error_mean_mm': eval_result['mean_translation_error_mm'],
                'translation_error_max_mm': eval_result['max_translation_error_mm'],
            },
            'generated_at': now_str,
            'session': session_dir,
        }
    }

    he_path = os.path.join(results_dir, 'hand_eye_transform.yaml')
    with open(he_path, 'w') as f:
        yaml.dump(he_result, f, allow_unicode=True, default_flow_style=False)
    print(f'\n手眼变换已保存: {he_path}')

    # 深度校正结果
    if depth_result:
        dc_result = {
            'depth_correction': {
                **depth_result,
                'valid_range': [0.10, 1.50],
                'generated_at': now_str,
                'calibration_session': session_dir,
            }
        }
        dc_path = os.path.join(results_dir, 'depth_correction.yaml')
        with open(dc_path, 'w') as f:
            yaml.dump(dc_result, f, allow_unicode=True, default_flow_style=False)
        print(f'深度校正已保存: {dc_path}')

        # 同步更新 share 目录的配置
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('eye_hand_calibration')
            share_dc = os.path.join(share, 'config', 'depth_correction.yaml')
            with open(share_dc, 'w') as f:
                yaml.dump(dc_result, f, allow_unicode=True, default_flow_style=False)
            print(f'已同步到包配置: {share_dc}')
        except Exception as e:
            print(f'[提示] 无法同步到 share 目录（需要重新编译后生效）: {e}')

    # 生成文字报告
    report_lines = [
        '=' * 60,
        '  手眼标定结果报告',
        f'  生成时间: {now_str}',
        '=' * 60,
        '',
        '【手眼变换矩阵】',
        '  坐标系: arm_link4 → camera_depth_optical_frame',
        f'  平移: x={xyz[0]*1000:.2f}mm  y={xyz[1]*1000:.2f}mm  z={xyz[2]*1000:.2f}mm',
        f'  四元数: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]',
        '',
        '【标定质量】',
        f'  旋转误差: {eval_result["mean_rotation_error_deg"]:.3f}° (均值)',
        f'  平移误差: {eval_result["mean_translation_error_mm"]:.2f}mm (均值)',
        f'  评价: {quality}',
    ]

    if depth_result:
        report_lines += [
            '',
            '【深度校正模型】',
            '  depth_corrected = scale × depth_raw + offset',
            f'  scale  = {depth_result["scale"]:.6f}',
            f'  offset = {depth_result["offset"]*1000:.3f} mm',
            f'  RMSE   = {depth_result["rmse"]*1000:.3f} mm',
            f'  R²     = {depth_result["r_squared"]:.4f}',
        ]

    report_lines += [
        '',
        '【URDF 修正建议】',
        '  运行以下命令生成修正后的 URDF：',
        '  ros2 run eye_hand_calibration generate_corrected_urdf',
        '',
        '=' * 60,
    ]

    report_path = os.path.join(results_dir, 'evaluation_report.txt')
    with open(report_path, 'w') as f:
        f.write('\n'.join(report_lines))

    print()
    print('\n'.join(report_lines))
    print(f'\n完整报告已保存: {report_path}')

    print()
    print('下一步：')
    print('  ros2 run eye_hand_calibration generate_corrected_urdf')
    print('  ros2 launch eye_hand_calibration verify_calibration.launch.py')


if __name__ == '__main__':
    main()
