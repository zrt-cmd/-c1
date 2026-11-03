#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼标定求解器

求解 AX = XB 方程，得到相机坐标系相对机械臂末端的精确变换矩阵 X。

方程含义：
  A_i = T_{ee}^{base}(i+1) * (T_{ee}^{base}(i))^{-1}
        机械臂末端从位姿 i 到位姿 i+1 的相对变换
  B_i = T_{tag}^{cam}(i+1) * (T_{tag}^{cam}(i))^{-1}
        标定板从位姿 i 到位姿 i+1 在相机坐标系的相对变换
  X   = T_{cam}^{ee}
        相机坐标系相对机械臂末端的变换（待求）

支持算法：
  - Tsai-Lenz（默认，经典稳健）
  - Horaud（旋转部分更稳定）

同时拟合深度线性校正模型：
  depth_corrected = scale * depth_raw + offset

使用方式：
  # 采集完数据后，在 session 目录运行
  ros2 run eye_hand_calibration compute_hand_eye_transform \\
      --session ~/.ros/eye_hand_calib/calibration_20260729_143022
"""

import math
import yaml
import os
from typing import List, Tuple, Optional, Dict, Any
import numpy as np

from eye_hand_calibration.utils import (
    quat_to_rotation_matrix,
    rotation_matrix_to_quat,
    make_transform_matrix,
    invert_transform,
    matrix_to_xyz_quat,
    pixel_to_3d,
    reprojection_error,
)


# ---------------------------------------------------------------------------
# 核心求解算法
# ---------------------------------------------------------------------------

def solve_hand_eye_tsai(
    A_list: List[np.ndarray],
    B_list: List[np.ndarray],
) -> np.ndarray:
    """
    Tsai-Lenz 手眼标定算法。

    分两步：
      1. 旋转求解：利用 R_A * R_X = R_X * R_B 建立线性方程组，SVD 求解
      2. 平移求解：在旋转已知的情况下，线性求解平移向量

    Args:
        A_list: 机械臂末端相对变换列表（4×4 矩阵）
        B_list: 相机中标定板相对变换列表（4×4 矩阵）

    Returns:
        X: 4×4 手眼变换矩阵（T_{cam}^{ee}）

    Reference:
        R. Y. Tsai and R. K. Lenz, "A new technique for fully autonomous
        and efficient 3D robotics hand/eye calibration," 1989.
    """
    n = len(A_list)
    assert n == len(B_list), "A 和 B 的数量必须相同"
    assert n >= 3, f"至少需要 3 对数据，当前只有 {n} 对"

    # ---- 步骤 1：求解旋转 ----
    # 对每对 (A_i, B_i) 提取旋转轴角
    # R_A * R_X = R_X * R_B  →  skew(r_A + r_B) * r_x = r_B - r_A
    # 其中 r 为旋转向量的改型（Rodriguez form / 2 * tan(theta/2)）

    M = np.zeros((3*n, 3))
    b_vec = np.zeros(3*n)

    for i, (A, B) in enumerate(zip(A_list, B_list)):
        R_A = A[:3, :3]
        R_B = B[:3, :3]

        # 旋转矩阵 → 轴角向量（Tsai modified Rodrigues）
        r_A = _rotation_to_modified_rodrigues(R_A)
        r_B = _rotation_to_modified_rodrigues(R_B)

        # skew(r_A + r_B)
        r_sum = r_A + r_B
        S = _skew(r_sum)

        M[3*i:3*i+3, :] = S
        b_vec[3*i:3*i+3] = r_B - r_A

    # 最小二乘求解旋转
    r_x, _, _, _ = np.linalg.lstsq(M, b_vec, rcond=None)

    # 转回普通旋转向量（还原缩放）
    r_x_norm = np.linalg.norm(r_x)
    if r_x_norm < 1e-10:
        R_X = np.eye(3)
    else:
        # 从 modified Rodrigues 还原为旋转矩阵
        theta_half = r_x_norm
        theta = 2 * math.atan(theta_half)
        axis = r_x / r_x_norm
        R_X = _axis_angle_to_rotation(axis, theta)

    # ---- 步骤 2：求解平移 ----
    # (R_A - I) * t_X = R_X * t_B - t_A
    C = np.zeros((3*n, 3))
    d_vec = np.zeros(3*n)

    for i, (A, B) in enumerate(zip(A_list, B_list)):
        R_A = A[:3, :3]
        t_A = A[:3, 3]
        t_B = B[:3, 3]

        C[3*i:3*i+3, :] = R_A - np.eye(3)
        d_vec[3*i:3*i+3] = R_X @ t_B - t_A

    t_x, _, _, _ = np.linalg.lstsq(C, d_vec, rcond=None)

    return make_transform_matrix(R_X, t_x)


def solve_hand_eye_horaud(
    A_list: List[np.ndarray],
    B_list: List[np.ndarray],
) -> np.ndarray:
    """
    Horaud 手眼标定算法（四元数法，旋转部分更数值稳定）。

    Args:
        A_list: 机械臂末端相对变换列表
        B_list: 相机中标定板相对变换列表

    Returns:
        X: 4×4 手眼变换矩阵
    """
    n = len(A_list)

    # ---- 旋转：构造 4n×4 矩阵，SVD 求最小特征向量 ----
    N = np.zeros((4*n, 4))

    for i, (A, B) in enumerate(zip(A_list, B_list)):
        R_A = A[:3, :3]
        R_B = B[:3, :3]
        q_A = rotation_matrix_to_quat(R_A)  # [x,y,z,w]
        q_B = rotation_matrix_to_quat(R_B)

        # 构造 (Q_A_left - Q_B_right) 矩阵（4×4 四元数乘法矩阵）
        Q_left = _quat_left_multiply_matrix(q_A)
        Q_right = _quat_right_multiply_matrix(q_B)
        N[4*i:4*i+4, :] = Q_left - Q_right

    # SVD 求最小奇异值对应的向量
    _, _, Vt = np.linalg.svd(N)
    q_x = Vt[-1]                     # [x, y, z, w]
    q_x = q_x / np.linalg.norm(q_x)
    R_X = quat_to_rotation_matrix(q_x)

    # ---- 平移（与 Tsai 相同）----
    C = np.zeros((3*n, 3))
    d_vec = np.zeros(3*n)

    for i, (A, B) in enumerate(zip(A_list, B_list)):
        R_A = A[:3, :3]
        t_A = A[:3, 3]
        t_B = B[:3, 3]
        C[3*i:3*i+3, :] = R_A - np.eye(3)
        d_vec[3*i:3*i+3] = R_X @ t_B - t_A

    t_x, _, _, _ = np.linalg.lstsq(C, d_vec, rcond=None)

    return make_transform_matrix(R_X, t_x)


# ---------------------------------------------------------------------------
# 深度校正拟合
# ---------------------------------------------------------------------------

def fit_depth_correction(
    measured_depths: List[float],
    reference_depths: List[float],
) -> Dict[str, float]:
    """
    拟合线性深度校正模型：depth_corrected = scale * depth_raw + offset

    reference_depths 由 AprilTag PnP 解算得到（更可信），
    measured_depths  由深度图直接读取（有误差）。

    Args:
        measured_depths:   原始深度图读取值（米）
        reference_depths:  AprilTag PnP 解算的真实深度（米）

    Returns:
        dict: scale, offset, rmse, r_squared
    """
    assert len(measured_depths) == len(reference_depths), "数据长度不一致"
    assert len(measured_depths) >= 2, "至少需要 2 个样本"

    x = np.array(measured_depths)
    y = np.array(reference_depths)

    # 最小二乘线性拟合：y = scale * x + offset
    A = np.vstack([x, np.ones_like(x)]).T
    result, _, _, _ = np.linalg.lstsq(A, y, rcond=None)
    scale, offset = result

    # 评估
    y_pred = scale * x + offset
    residuals = y - y_pred
    rmse = float(np.sqrt(np.mean(residuals**2)))

    ss_res = np.sum(residuals**2)
    ss_tot = np.sum((y - np.mean(y))**2)
    r_squared = float(1 - ss_res / ss_tot) if ss_tot > 1e-10 else 1.0

    return {
        'model': 'linear',
        'scale': float(scale),
        'offset': float(offset),
        'rmse': rmse,
        'r_squared': r_squared,
        'num_samples': len(x),
        'depth_range': [float(np.min(x)), float(np.max(x))],
    }


# ---------------------------------------------------------------------------
# 评估工具
# ---------------------------------------------------------------------------

def evaluate_calibration(
    A_list: List[np.ndarray],
    B_list: List[np.ndarray],
    X: np.ndarray,
) -> Dict[str, float]:
    """
    评估手眼标定质量。

    对每对 (A_i, B_i)：
      残差 = ||A_i * X - X * B_i||_F（Frobenius 范数）

    同时分别计算旋转和平移误差。

    Returns:
        dict: 含 mean_error, std_error, max_error, 
              rotation_error_deg, translation_error_mm
    """
    errors = []
    rot_errors_deg = []
    trans_errors_mm = []

    for A, B in zip(A_list, B_list):
        # AX - XB，理想情况下应为零矩阵
        residual = A @ X - X @ B
        fro_err = np.linalg.norm(residual, 'fro')
        errors.append(fro_err)

        # 旋转误差
        R_AX = (A @ X)[:3, :3]
        R_XB = (X @ B)[:3, :3]
        R_diff = R_AX.T @ R_XB
        # 旋转角度（弧度）= arccos((trace(R_diff) - 1) / 2)
        cos_angle = (np.trace(R_diff) - 1) / 2
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        rot_err = math.degrees(math.acos(cos_angle))
        rot_errors_deg.append(rot_err)

        # 平移误差
        t_AX = (A @ X)[:3, 3]
        t_XB = (X @ B)[:3, 3]
        trans_err = np.linalg.norm(t_AX - t_XB) * 1000  # mm
        trans_errors_mm.append(trans_err)

    return {
        'num_pairs': len(errors),
        'mean_frobenius_error': float(np.mean(errors)),
        'std_frobenius_error': float(np.std(errors)),
        'max_frobenius_error': float(np.max(errors)),
        'mean_rotation_error_deg': float(np.mean(rot_errors_deg)),
        'max_rotation_error_deg': float(np.max(rot_errors_deg)),
        'mean_translation_error_mm': float(np.mean(trans_errors_mm)),
        'max_translation_error_mm': float(np.max(trans_errors_mm)),
    }


def build_relative_motions(
    ee_transforms: List[np.ndarray],
    tag_transforms: List[np.ndarray],
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    从绝对变换列表构造相对运动对 (A_i, B_i)，用于 AX=XB 求解。

    约定（与 OpenCV calibrateHandEye 一致）：
      ee_transforms[i]  = T_{base}^{gripper_i}  (gripper 相对 base 的绝对位姿)
      tag_transforms[i] = T_{cam}^{tag_i}        (tag 相对 camera 的位姿，PnP 解算)
      X                 = T_{gripper}^{cam}       (camera 相对 gripper，待求)

    相对运动对：
      A_i = inv(T_{base}^{gripper_{i+1}}) * T_{base}^{gripper_i}
          = T_{gripper_{i+1}}^{gripper_i}  (gripper 从 i 到 i+1 的相对运动)
      B_i = T_{cam}^{tag_{i+1}} * inv(T_{cam}^{tag_i})
          = tag 在 cam 中的相对运动

    满足：A_i * X = X * B_i

    Returns:
        (A_list, B_list)
    """
    A_list = []
    B_list = []
    n = len(ee_transforms)

    for i in range(n - 1):
        T_ee_i    = ee_transforms[i]
        T_ee_next = ee_transforms[i + 1]
        T_tag_i   = tag_transforms[i]
        T_tag_next = tag_transforms[i + 1]

        # A = inv(T_base2gripper_next) * T_base2gripper_i
        A = invert_transform(T_ee_next) @ T_ee_i
        # B = T_cam2tag_next * inv(T_cam2tag_i)
        B = T_tag_next @ invert_transform(T_tag_i)

        A_list.append(A)
        B_list.append(B)

    return A_list, B_list


# ---------------------------------------------------------------------------
# 主求解入口
# ---------------------------------------------------------------------------

def solve_and_evaluate(
    ee_transforms: List[np.ndarray],
    tag_transforms: List[np.ndarray],
    measured_depths: Optional[List[float]] = None,
    reference_depths: Optional[List[float]] = None,
    method: str = 'tsai',
) -> Dict[str, Any]:
    """
    完整的手眼标定求解流程。

    Args:
        ee_transforms:     机械臂末端绝对变换列表（base_link → arm_link4）
        tag_transforms:    AprilTag 在相机坐标系的绝对变换列表
        measured_depths:   原始深度图读取值（可选，用于深度校正）
        reference_depths:  AprilTag PnP 参考深度（可选）
        method:            'tsai' 或 'horaud'

    Returns:
        dict: 含 hand_eye_matrix, xyz, quat, evaluation, depth_correction
    """
    assert len(ee_transforms) == len(tag_transforms)
    assert len(ee_transforms) >= 3, "至少需要 3 个姿态"

    # 1. 构造相对运动对
    A_list, B_list = build_relative_motions(ee_transforms, tag_transforms)

    # 2. 求解手眼变换
    if method == 'horaud':
        X = solve_hand_eye_horaud(A_list, B_list)
    else:
        X = solve_hand_eye_tsai(A_list, B_list)

    xyz, quat = matrix_to_xyz_quat(X)

    # 3. 评估
    eval_result = evaluate_calibration(A_list, B_list, X)

    # 4. 深度校正（可选）
    depth_result = None
    if measured_depths and reference_depths:
        depth_result = fit_depth_correction(measured_depths, reference_depths)

    return {
        'hand_eye_matrix': X,
        'xyz': xyz.tolist(),
        'quat_xyzw': quat.tolist(),
        'evaluation': eval_result,
        'depth_correction': depth_result,
        'method': method,
        'num_poses': len(ee_transforms),
    }


# ---------------------------------------------------------------------------
# 内部辅助函数
# ---------------------------------------------------------------------------

def _rotation_to_modified_rodrigues(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → Tsai 修改版 Rodrigues 向量（tan(theta/2) * axis）。"""
    # 旋转角
    cos_angle = (np.trace(R) - 1) / 2
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    theta = math.acos(cos_angle)

    if abs(theta) < 1e-8:
        return np.zeros(3)

    # 旋转轴
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2 * math.sin(theta))

    # 修改版 Rodrigues: tan(theta/2) * axis
    return math.tan(theta / 2) * axis


def _skew(v: np.ndarray) -> np.ndarray:
    """3-向量 → 反对称矩阵（skew-symmetric matrix）。"""
    x, y, z = v
    return np.array([
        [ 0, -z,  y],
        [ z,  0, -x],
        [-y,  x,  0],
    ])


def _axis_angle_to_rotation(axis: np.ndarray, theta: float) -> np.ndarray:
    """旋转轴 + 角度 → 旋转矩阵（Rodrigues 公式）。"""
    K = _skew(axis)
    R = np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)
    return R


def _quat_left_multiply_matrix(q: np.ndarray) -> np.ndarray:
    """
    构造四元数左乘矩阵 Q_L，使得 q ⊗ p = Q_L * p。
    q = [x, y, z, w]
    """
    x, y, z, w = q
    return np.array([
        [ w, -z,  y,  x],
        [ z,  w, -x,  y],
        [-y,  x,  w,  z],
        [-x, -y, -z,  w],
    ])


def _quat_right_multiply_matrix(q: np.ndarray) -> np.ndarray:
    """
    构造四元数右乘矩阵 Q_R，使得 p ⊗ q = Q_R * p。
    q = [x, y, z, w]
    """
    x, y, z, w = q
    return np.array([
        [ w,  z, -y,  x],
        [-z,  w,  x,  y],
        [ y, -x,  w,  z],
        [-x, -y, -z,  w],
    ])


def build_relative_motions_all_pairs(
    ee_transforms: List[np.ndarray],
    tag_transforms: List[np.ndarray],
    min_rotation_deg: float = 8.0,
    max_rotation_deg: float = 180.0,
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    """
    使用所有非冗余姿态对构造相对运动，而非仅相邻对。
    支持最小/最大旋转角阈值过滤。
    
    Args:
        ee_transforms: 机械臂末端绝对变换列表
        tag_transforms: 相机中标定板绝对变换列表
        min_rotation_deg: 最小旋转角（度），过滤旋转太小（数值不稳定）的对
        max_rotation_deg: 最大旋转角（度），过滤旋转太大（奇异）的对
    """
    A_list, B_list = [], []
    n = len(ee_transforms)

    for i in range(n):
        for j in range(i + 1, n):
            A = invert_transform(ee_transforms[j]) @ ee_transforms[i]
            B = tag_transforms[j] @ invert_transform(tag_transforms[i])

            # 检查旋转角
            cos_a = (np.trace(A[:3, :3]) - 1) / 2
            cos_a = np.clip(cos_a, -1.0, 1.0)
            rot_deg = math.degrees(math.acos(cos_a))

            if min_rotation_deg <= rot_deg <= max_rotation_deg:
                A_list.append(A)
                B_list.append(B)

    return A_list, B_list


def detect_outlier_poses(
    ee_transforms: List[np.ndarray],
    tag_transforms: List[np.ndarray],
    method: str = 'horaud',
    min_rotation_deg: float = 8.0,
    threshold_translation_mm: float = 100.0,
    threshold_rotation_deg: float = 30.0,
    verbose: bool = True,
) -> Tuple[List[np.ndarray], List[np.ndarray], List[int]]:
    """
    使用 leave-one-out 交叉验证检测离群姿态。

    对每个姿态，排除它后重新标定，比较 X 的变化。
    如果排除某个姿态后 X 发生剧烈变化（平移>100mm或旋转>30°），
    判定该姿态为离群点。

    递归剔除最差的离群点，直到没有新的离群点或剩余数据不足。

    Args:
        ee_transforms: 机械臂末端绝对变换列表
        tag_transforms: 相机中标定板绝对变换列表
        method: 求解算法 ('tsai' 或 'horaud')
        min_rotation_deg: 最小旋转角阈值
        threshold_translation_mm: 平移变化阈值(mm)
        threshold_rotation_deg: 旋转变化阈值(度)
        verbose: 是否打印详细信息

    Returns:
        (filtered_ee, filtered_tag, excluded_indices)
    """
    n = len(ee_transforms)
    if n < 5:
        return ee_transforms, tag_transforms, []

    excluded = set()

    for iteration in range(5):  # 最多5轮迭代
        remaining = [i for i in range(n) if i not in excluded]
        if len(remaining) < 5:
            break

        ee_remain = [ee_transforms[i] for i in remaining]
        tag_remain = [tag_transforms[i] for i in remaining]

        # 全量求解
        A_full, B_full = build_relative_motions_all_pairs(
            ee_remain, tag_remain, min_rotation_deg=min_rotation_deg)
        if len(A_full) < 3:
            break

        if method == 'tsai':
            X_full = solve_hand_eye_tsai(A_full, B_full)
        else:
            X_full = solve_hand_eye_horaud(A_full, B_full)

        # Leave-one-out
        worst_idx = None
        worst_score = 0.0

        for idx in remaining:
            loo = [i for i in remaining if i != idx]
            if len(loo) < 4:
                continue
            ee_loo = [ee_transforms[i] for i in loo]
            tag_loo = [tag_transforms[i] for i in loo]

            A_loo, B_loo = build_relative_motions_all_pairs(
                ee_loo, tag_loo, min_rotation_deg=min_rotation_deg)
            if len(A_loo) < 3:
                continue

            if method == 'tsai':
                X_loo = solve_hand_eye_tsai(A_loo, B_loo)
            else:
                X_loo = solve_hand_eye_horaud(A_loo, B_loo)

            # 计算 X 的变化量
            t_diff = np.linalg.norm(X_full[:3, 3] - X_loo[:3, 3]) * 1000  # mm
            R_diff = X_full[:3, :3].T @ X_loo[:3, :3]
            cos_a = (np.trace(R_diff) - 1) / 2
            cos_a = np.clip(cos_a, -1.0, 1.0)
            r_diff = math.degrees(math.acos(cos_a))

            score = t_diff / threshold_translation_mm + r_diff / threshold_rotation_deg

            if t_diff > threshold_translation_mm or r_diff > threshold_rotation_deg:
                if score > worst_score:
                    worst_score = score
                    worst_idx = idx

        if worst_idx is None:
            break  # 没有新的离群点

        excluded.add(worst_idx)
        if verbose:
            print(f'  [离群检测] 排除姿态 {worst_idx}：导致 X 变化过大')

    filtered_ee = [ee_transforms[i] for i in range(n) if i not in excluded]
    filtered_tag = [tag_transforms[i] for i in range(n) if i not in excluded]

    if verbose and excluded:
        print(f'  共排除 {len(excluded)} 个离群姿态: {sorted(excluded)}')
        print(f'  剩余 {len(filtered_ee)} 组有效数据')

    return filtered_ee, filtered_tag, sorted(excluded)


# 别名
compute_relative_transforms = build_relative_motions
