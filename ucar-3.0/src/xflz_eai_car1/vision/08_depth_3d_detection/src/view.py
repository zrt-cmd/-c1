"""双栏合成 + 点击标注（构建文档 03；规格 depth_viewer_guide.md）。"""
from __future__ import annotations

import numpy as np
import cv2

STATUS_TEXT = "Click depth image to read depth"


def compose_rgbd(rgb: np.ndarray, depth_norm: np.ndarray,
                 colormap: str = "jet", scale: float = 0.666) -> np.ndarray:
    """左 RGB | 右 Depth 伪彩；顶部状态栏。返回 (H', 2W', 3)。"""
    from depth import pseudocolor

    h, w = rgb.shape[:2]
    dh, dw = depth_norm.shape[:2]
    assert (dh, dw) == (h, w), f"rgb {rgb.shape} vs depth {depth_norm.shape} mismatch"
    small = (int(round(w * scale)), int(round(h * scale)))
    left = cv2.resize(rgb, small, interpolation=cv2.INTER_LINEAR)
    right = cv2.resize(pseudocolor(depth_norm, colormap), small,
                       interpolation=cv2.INTER_LINEAR)
    out = np.hstack([left, right])
    cv2.putText(out, STATUS_TEXT, (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 255), 2)
    return out


def screen_to_pixel(x: int, y: int, scale: float) -> tuple[int, int]:
    """显示坐标 -> 原图坐标（÷ scale）。"""
    return int(round(x / scale)), int(round(y / scale))


def click_to_pixel(x: int, y: int, img_w: int, scale: float) -> tuple[int, int]:
    """窗口点击坐标 -> 原图像素；双栏视图（左 RGB | 右 Depth）点在右栏时减去左栏宽。"""
    left_w = int(round(img_w * scale))
    if x >= left_w:
        x -= left_w
    return screen_to_pixel(x, y, scale)


def draw_click(out: np.ndarray, u: int, v: int, text: str,
               scale: float = 0.666) -> np.ndarray:
    """在原图坐标 (u, v) 处画标记 + 文字（显示坐标空间）。"""
    x, y = int(round(u * scale)), int(round(v * scale))
    cv2.drawMarker(out, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 12, 2)
    cv2.putText(out, text, (10, out.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 255, 0), 2)
    return out
