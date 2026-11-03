"""深度图处理（构建文档 02；规格 depth_viewer_guide.md）。

uint16 mm、0 = 无效；伪彩映射；点击读深度 + 内参反投影：
x = (u - cx) * z / fx, y = (v - cy) * z / fy（z 为米）。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import cv2


@dataclass(frozen=True)
class ClickResult:
    z_mm: int
    z_m: float
    xyz: tuple | None  # (x, y, z) 米（相机系）或 None


def load_depth(path: str | Path) -> np.ndarray:
    """uint16 深度图 -> (H,W) mm 数组。"""
    d = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if d is None:
        raise FileNotFoundError(f"cannot read depth image: {path}")
    return d.astype(np.uint16)


def normalize_depth(depth: np.ndarray, invalid: int = 0) -> np.ndarray:
    """有效像素 min-max -> 0-255 uint8；无效置 0。"""
    out = np.zeros(depth.shape, dtype=np.uint8)
    valid = depth != invalid
    if valid.any():
        lo, hi = depth[valid].min(), depth[valid].max()
        if hi > lo:
            out[valid] = ((depth[valid].astype(np.float32) - lo) /
                          (hi - lo) * 255).astype(np.uint8)
        else:
            out[valid] = 255  # 单值深度 -> 全亮
    return out


def pseudocolor(depth_norm: np.ndarray, colormap: str = "jet") -> np.ndarray:
    """8bit 深度 -> (H,W,3) BGR 伪彩（gray 模式三通道灰阶）。"""
    if colormap == "gray":
        return cv2.cvtColor(depth_norm, cv2.COLOR_GRAY2BGR)
    return cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)


def read_depth(depth: np.ndarray, u: int, v: int, scale: float = 0.001,
               cam: dict | None = None) -> ClickResult:
    """点击 (u, v) -> 深度；cam 提供时附加反投影坐标。"""
    h, w = depth.shape[:2]
    if not (0 <= u < w and 0 <= v < h):
        return ClickResult(0, 0.0, None)
    z_mm = int(depth[v, u])
    if z_mm == 0:
        return ClickResult(0, 0.0, None)
    z_m = z_mm * scale
    xyz = None
    if cam is not None:
        x = (u - cam["cx"]) * z_m / cam["fx"]
        y = (v - cam["cy"]) * z_m / cam["fy"]
        xyz = (float(x), float(y), float(z_m))
    return ClickResult(z_mm, z_m, xyz)
