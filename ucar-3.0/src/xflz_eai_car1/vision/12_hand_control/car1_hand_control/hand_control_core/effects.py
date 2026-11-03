"""图像特效（vendor 12_手指控制 移植）：拇指-食指夹角控制 5 种效果。

effect 循环：color → thresh → blur → hue → enhance（'f' 键切换，camera_demo）。
value ∈ [0,255] 由 thumb_index_value() 提供（夹角 [0,70]° 线性映射）。
"""
from __future__ import annotations

import cv2
import numpy as np

EFFECTS = ["color", "thresh", "blur", "hue", "enhance"]


def apply_effect(frame: np.ndarray, effect: str, value: int) -> np.ndarray:
    """应用特效到 BGR 帧；返回副本。effect 不在 EFFECTS 中 → 原图。"""
    if effect == "thresh":
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, out = cv2.threshold(gray, int(value), 255, cv2.THRESH_BINARY)
        return cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    if effect == "blur":
        k = int(round(float(np.interp(value, [0, 255], [0, 11]))))
        k = k | 1  # 奇数
        return cv2.GaussianBlur(frame, (21, 21), float(k))
    if effect == "hue":
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV).astype(np.int16)
        hsv[:, :, 0] = (hsv[:, :, 0] + int(value)) % 180
        return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
    if effect == "enhance":
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clip = max(1.0, int(value) / 40.0)
        clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8))
        l2 = clahe.apply(l)
        return cv2.cvtColor(cv2.merge([l2, a, b]), cv2.COLOR_LAB2BGR)
    return frame.copy()  # color（原图）
