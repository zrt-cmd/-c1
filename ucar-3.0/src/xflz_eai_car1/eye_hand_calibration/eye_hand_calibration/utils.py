#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
公共工具函数模块

包含：
  - 变换矩阵工具（TF、旋转、四元数）
  - 几何计算（投影、反投影）
  - 文件 IO（标定数据读写）
  - 可视化辅助
"""

import os
import math
import datetime
import yaml
import numpy as np
from typing import Optional, Tuple, List, Dict, Any


# ---------------------------------------------------------------------------
# 变换矩阵工具
# ---------------------------------------------------------------------------

def quat_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """
    四元数转旋转矩阵。
    q = [x, y, z, w]（与 ROS 约定一致）
    """
    x, y, z, w = q
    # 归一化
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n

    R = np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),       2*(x*z + y*w)      ],
        [2*(x*y + z*w),        1 - 2*(x*x + z*z),   2*(y*z - x*w)      ],
        [2*(x*z - y*w),        2*(y*z + x*w),        1 - 2*(x*x + y*y)  ],
    ])
    return R


def rotation_matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """
    旋转矩阵转四元数，返回 [x, y, z, w]。
    使用 Shepperd's method，数值稳定。
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def make_transform_matrix(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """构造 4×4 齐次变换矩阵。"""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.flatten()
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    """
    求 4×4 变换矩阵的逆。
    利用 R^{-1} = R^T 避免 np.linalg.inv。
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def ros_transform_to_matrix(transform) -> np.ndarray:
    """
    将 geometry_msgs/TransformStamped 的 transform 字段转为 4×4 矩阵。
    transform: geometry_msgs.msg.Transform
    """
    t = transform.translation
    r = transform.rotation
    q = np.array([r.x, r.y, r.z, r.w])
    R = quat_to_rotation_matrix(q)
    return make_transform_matrix(R, np.array([t.x, t.y, t.z]))


def pose_stamped_to_matrix(pose) -> np.ndarray:
    """
    将 geometry_msgs/PoseStamped.pose 转为 4×4 矩阵。
    pose: geometry_msgs.msg.Pose
    """
    p = pose.position
    o = pose.orientation
    q = np.array([o.x, o.y, o.z, o.w])
    R = quat_to_rotation_matrix(q)
    return make_transform_matrix(R, np.array([p.x, p.y, p.z]))


def matrix_to_xyz_quat(T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    4×4 矩阵 → (xyz, quat=[x,y,z,w])
    """
    xyz = T[:3, 3]
    quat = rotation_matrix_to_quat(T[:3, :3])
    return xyz, quat


# ---------------------------------------------------------------------------
# 相机投影工具
# ---------------------------------------------------------------------------

def pixel_to_3d(u: float, v: float, depth_m: float,
                fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    """
    像素坐标 + 深度 → 相机坐标系下的 3D 点（米）。
    使用标准针孔相机模型（光学坐标系：x右 y下 z前）。

    Args:
        u, v: 像素坐标
        depth_m: 深度值（米）
        fx, fy, cx, cy: 相机内参

    Returns:
        np.ndarray([X, Y, Z]) 相机坐标系下的 3D 坐标
    """
    X = (u - cx) * depth_m / fx
    Y = (v - cy) * depth_m / fy
    Z = depth_m
    return np.array([X, Y, Z])


def project_3d_to_pixel(point_3d: np.ndarray,
                        fx: float, fy: float,
                        cx: float, cy: float) -> Tuple[float, float]:
    """
    3D 点（相机坐标系）→ 像素坐标。
    用于计算重投影误差。
    """
    X, Y, Z = point_3d
    if abs(Z) < 1e-6:
        return cx, cy
    u = fx * X / Z + cx
    v = fy * Y / Z + cy
    return u, v


def reprojection_error(point_3d: np.ndarray,
                       observed_uv: Tuple[float, float],
                       fx: float, fy: float,
                       cx: float, cy: float) -> float:
    """
    计算单点重投影误差（像素）。
    """
    u_proj, v_proj = project_3d_to_pixel(point_3d, fx, fy, cx, cy)
    u_obs, v_obs = observed_uv
    return math.sqrt((u_proj - u_obs)**2 + (v_proj - v_obs)**2)


# ---------------------------------------------------------------------------
# 深度值工具
# ---------------------------------------------------------------------------

def get_depth_at_pixel(depth_image: np.ndarray, u: int, v: int,
                       window: int = 5) -> Optional[float]:
    """
    在深度图中取 (u, v) 附近 window×window 窗口的中值深度（米）。
    忽略 0 值（无效深度）。
    HP60C 深度图单位为 mm（uint16），这里自动转换为米。

    Args:
        depth_image: HxW uint16 深度图（mm）
        u, v: 像素坐标（列, 行）
        window: 邻域大小

    Returns:
        中值深度（米），如果无效则返回 None
    """
    h, w = depth_image.shape[:2]
    half = window // 2

    r0 = max(0, v - half)
    r1 = min(h, v + half + 1)
    c0 = max(0, u - half)
    c1 = min(w, u + half + 1)

    patch = depth_image[r0:r1, c0:c1].astype(np.float32)
    valid = patch[patch > 0]

    if valid.size == 0:
        return None

    # HP60C 单位 mm → 米
    return float(np.median(valid)) / 1000.0


# ---------------------------------------------------------------------------
# 数据目录管理
# ---------------------------------------------------------------------------

def make_session_dir(base_dir: Optional[str] = None) -> str:
    """
    创建带时间戳的标定会话目录。
    默认路径：~/.ros/eye_hand_calib/calibration_YYYYMMDD_HHMMSS/
    """
    if not base_dir:
        base_dir = os.path.expanduser('~/.ros/eye_hand_calib')

    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    session_dir = os.path.join(base_dir, f'calibration_{ts}')

    for sub in ['images', 'results']:
        os.makedirs(os.path.join(session_dir, sub), exist_ok=True)

    return session_dir


def get_latest_session_dir(base_dir: Optional[str] = None) -> Optional[str]:
    """
    获取最新的标定会话目录。
    """
    if not base_dir:
        base_dir = os.path.expanduser('~/.ros/eye_hand_calib')

    if not os.path.isdir(base_dir):
        return None

    sessions = sorted([
        d for d in os.listdir(base_dir)
        if d.startswith('calibration_') and os.path.isdir(os.path.join(base_dir, d))
    ])

    if not sessions:
        return None

    return os.path.join(base_dir, sessions[-1])


# ---------------------------------------------------------------------------
# YAML 读写
# ---------------------------------------------------------------------------

def save_yaml(path: str, data: Dict[str, Any]) -> None:
    """将字典保存为 YAML 文件，确保目录存在。"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, default_flow_style=False,
                  allow_unicode=True, sort_keys=False)


def load_yaml(path: str) -> Dict[str, Any]:
    """读取 YAML 文件，返回字典。"""
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# 变换误差统计
# ---------------------------------------------------------------------------

def rotation_angle_error(R1: np.ndarray, R2: np.ndarray) -> float:
    """
    两个旋转矩阵的角度差（弧度）。
    使用 trace(R1^T R2) 计算。
    """
    R_diff = R1.T @ R2
    trace = np.clip((np.trace(R_diff) - 1.0) / 2.0, -1.0, 1.0)
    return math.acos(trace)


def translation_error(t1: np.ndarray, t2: np.ndarray) -> float:
    """两个平移向量的欧氏距离（米）。"""
    return float(np.linalg.norm(t1 - t2))


def summarize_errors(errors: List[float]) -> Dict[str, float]:
    """计算误差统计量（均值、std、max、min）。"""
    arr = np.array(errors)
    return {
        'mean': float(np.mean(arr)),
        'std': float(np.std(arr)),
        'max': float(np.max(arr)),
        'min': float(np.min(arr)),
        'rmse': float(np.sqrt(np.mean(arr**2))),
    }


# ---------------------------------------------------------------------------
# 格式化输出
# ---------------------------------------------------------------------------

def format_transform(T: np.ndarray, label: str = '') -> str:
    """格式化打印 4×4 变换矩阵。"""
    xyz, q = matrix_to_xyz_quat(T)
    rpy = quat_to_rpy(q)
    lines = []
    if label:
        lines.append(f'--- {label} ---')
    lines.append(f'  平移 (xyz)：{xyz[0]:.4f}  {xyz[1]:.4f}  {xyz[2]:.4f}  (m)')
    lines.append(f'  旋转 (rpy)：{math.degrees(rpy[0]):.2f}°  '
                 f'{math.degrees(rpy[1]):.2f}°  {math.degrees(rpy[2]):.2f}°')
    lines.append(f'  四元数 (xyzw)：{q[0]:.4f}  {q[1]:.4f}  {q[2]:.4f}  {q[3]:.4f}')
    return '\n'.join(lines)


def quat_to_rpy(q: np.ndarray) -> np.ndarray:
    """
    四元数 [x,y,z,w] → Roll-Pitch-Yaw（弧度，XYZ 外旋）。
    """
    x, y, z, w = q
    # Roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = math.asin(sinp)
    # Yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return np.array([roll, pitch, yaw])


# ---------------------------------------------------------------------------
# 标定数据持久化
# ---------------------------------------------------------------------------

def save_calibration_data(session_dir: str, records: List[Dict[str, Any]]) -> None:
    """
    将采集记录保存到会话目录。

    poses.yaml      — 机械臂末端变换（list of 4x4 flatten）
    detections.yaml — AprilTag 检测结果（list of 4x4 flatten + 深度值）
    """
    poses_path = os.path.join(session_dir, 'poses.yaml')
    det_path = os.path.join(session_dir, 'detections.yaml')

    poses_out: Dict[str, Any] = {
        'session': os.path.basename(session_dir),
        'num_records': len(records),
        'records': [],
    }
    det_out: Dict[str, Any] = {
        'session': os.path.basename(session_dir),
        'num_records': len(records),
        'records': [],
    }

    for i, rec in enumerate(records):
        ee_T = rec.get('ee_transform')
        tag_T = rec.get('tag_in_camera')
        depth_raw = rec.get('depth_raw_m')
        depth_pnp = rec.get('depth_pnp_m')

        poses_out['records'].append({
            'index': i,
            'ee_transform': ee_T.flatten().tolist() if ee_T is not None else None,
            'pose_xyz': rec.get('pose_xyz'),
        })
        det_out['records'].append({
            'index': i,
            'tag_in_camera': tag_T.flatten().tolist() if tag_T is not None else None,
            'depth_raw_m': float(depth_raw) if depth_raw is not None else None,
            'depth_pnp_m': float(depth_pnp) if depth_pnp is not None else None,
            'pixel_uv': rec.get('pixel_uv'),
        })


    save_yaml(poses_path, poses_out)
    save_yaml(det_path, det_out)
