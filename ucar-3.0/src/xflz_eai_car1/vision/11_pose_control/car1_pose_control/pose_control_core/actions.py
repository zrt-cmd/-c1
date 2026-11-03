"""体感动作分类：33 点姿态 → 控制指令（vendor 11_整体检测_体感 无控制逻辑，
PLAN exp11 要求"动作分类→控制指令"，规则为本实验设计，物理直觉优先）。

规则（输入 = 原图像素 landmarks 33×4 + 帧尺寸，内部归一化；vis 低的关键点忽略）：
  squat       双髋 y > 0.72        → backward（后退）
  both_up     双腕 y < 对应肩 y-0.08 → forward（前进）
  left_up     左腕高               → turn_left
  right_up    右腕高               → turn_right
  else                             → stop
优先级：squat > both_up > left/right（单手举起但另一手异常低时不判双手）。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# 关键点索引（BlazePose 33 点）
NOSE = 0
L_SHOULDER, R_SHOULDER = 11, 12
L_ELBOW, R_ELBOW = 13, 14
L_WRIST, R_WRIST = 15, 16
L_HIP, R_HIP = 23, 24

SQUAT_HIP_Y = 0.72        # 髋部归一化 y 超过此值 = 下蹲
HAND_UP_DELTA = 0.08      # 腕 y 比肩 y 高此值 = 举手
MIN_VIS = 0.5             # 关键点可见度阈值


@dataclass(frozen=True)
class Action:
    name: str            # stop/forward/backward/turn_left/turn_right
    label: str           # 显示名（ASCII：cv2.putText 无 CJK 字形，中文会显示 ????）


STOP = Action("stop", "Stop")
FORWARD = Action("forward", "Forward")
BACKWARD = Action("backward", "Backward")
TURN_LEFT = Action("turn_left", "Turn Left")
TURN_RIGHT = Action("turn_right", "Turn Right")


def _norm(lm: np.ndarray, h: int, w: int) -> np.ndarray:
    """像素 → 原图归一化 (33,4)，vis 原样。"""
    out = lm.astype(np.float32).copy()
    out[:, 0] = out[:, 0] / max(1, w)
    out[:, 1] = out[:, 1] / max(1, h)
    return out


def classify_action(landmarks_px: np.ndarray | None, h: int, w: int) -> Action:
    """33×4 像素关键点（None = 未检出）→ 控制指令。"""
    if landmarks_px is None:
        return STOP
    lm = _norm(landmarks_px, h, w)
    vis = lm[:, 3]
    if vis[L_HIP] < MIN_VIS or vis[R_HIP] < MIN_VIS:
        return STOP  # 髋部不可见 = 姿态不可信

    hip_y = float(max(lm[L_HIP, 1], lm[R_HIP, 1]))
    if hip_y > SQUAT_HIP_Y:
        return BACKWARD

    l_up = (vis[L_WRIST] >= MIN_VIS and vis[L_SHOULDER] >= MIN_VIS
            and lm[L_WRIST, 1] < lm[L_SHOULDER, 1] - HAND_UP_DELTA)
    r_up = (vis[R_WRIST] >= MIN_VIS and vis[R_SHOULDER] >= MIN_VIS
            and lm[R_WRIST, 1] < lm[R_SHOULDER, 1] - HAND_UP_DELTA)
    if l_up and r_up:
        return FORWARD
    if l_up:
        return TURN_LEFT
    if r_up:
        return TURN_RIGHT
    return STOP
