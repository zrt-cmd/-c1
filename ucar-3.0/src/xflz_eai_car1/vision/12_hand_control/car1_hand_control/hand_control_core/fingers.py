"""手指计数与拇指-食指夹角（vendor 11_手势识别 fingersUp + 12_手指控制 移植）。

vendor 语义（像素坐标）：
  - 拇指伸直：calc_angle(4,3,2) > 150° 且 calc_angle(3,2,1) > 150°（余弦定理，两段直线度）
  - 食指~小指（tip/pip 索引）：tip.y < pip.y（垂直手型假定，vendor 原样）
  - 拇指-食指夹角：calc_angle(4, 0, 8)（顶点 = 腕 0）→ [0,70]° → value [0,255]
"""
from __future__ import annotations

import numpy as np

# tip / pip 索引（食指~小指）
TIP_PIP = [(8, 6), (12, 10), (16, 14), (20, 18)]
STRAIGHT_ANGLE = 150.0
ANGLE_RANGE = (0.0, 70.0)
VALUE_RANGE = (0, 255)


def calc_angle(a: int, b: int, c: int, lm: np.ndarray) -> float:
    """顶点 b 的夹角（余弦定理，像素坐标）→ 度。"""
    x1, y1 = lm[a, 0], lm[a, 1]
    x2, y2 = lm[b, 0], lm[b, 1]
    x3, y3 = lm[c, 0], lm[c, 1]
    v1 = np.array([x1 - x2, y1 - y2], dtype=np.float64)
    v2 = np.array([x3 - x2, y3 - y2], dtype=np.float64)
    n1 = float(np.linalg.norm(v1))
    n2 = float(np.linalg.norm(v2))
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cos = float(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))


def fingers_up(lm: np.ndarray) -> list[bool]:
    """21×3 像素 landmarks → [拇指, 食指, 中指, 无名指, 小指] 伸直标志（vendor 语义）。"""
    out = []
    out.append(calc_angle(4, 3, 2, lm) > STRAIGHT_ANGLE
               and calc_angle(3, 2, 1, lm) > STRAIGHT_ANGLE)
    for tip, pip in TIP_PIP:
        out.append(bool(lm[tip, 1] < lm[pip, 1]))
    return out


def count_fingers(lm: np.ndarray) -> int:
    """伸直手指数（0~5）。"""
    return int(sum(fingers_up(lm)))


def thumb_index_angle(lm: np.ndarray) -> float:
    """拇指-食指夹角（顶点腕 0，vendor 12）→ 度。"""
    return calc_angle(4, 0, 8, lm)


def thumb_index_value(lm: np.ndarray) -> int:
    """夹角 → 控制值 [0,255]（线性映射 [0,70]°）。"""
    angle = thumb_index_angle(lm)
    return int(round(np.interp(angle, ANGLE_RANGE, VALUE_RANGE)))
