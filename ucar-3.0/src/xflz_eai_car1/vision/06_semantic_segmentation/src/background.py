"""背景合成：前景掩码 + 背景模式 -> 合成图（构建文档 04）。

课程「清除背景」效果：掩码内保留原图，掩码外替换背景。
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import cv2


def load_background(path: str | Path, size: tuple[int, int]) -> np.ndarray:
    """读背景图并缩放到 (w, h)；失败抛 ValueError。"""
    img = cv2.imread(str(path))
    if img is None:
        raise ValueError(f"cannot read background image: {path}")
    return cv2.resize(img, size, interpolation=cv2.INTER_LINEAR)


def compose(orig_bgr: np.ndarray, fg_mask: np.ndarray, cfg: dict,
            base_dir: str | Path | None = None) -> np.ndarray:
    """合成：返回与 orig_bgr 同尺寸 BGR 图。"""
    if fg_mask.shape != orig_bgr.shape[:2]:
        raise ValueError(
            f"mask shape {fg_mask.shape} != image shape {orig_bgr.shape[:2]}")
    mode = cfg["background"]["mode"]
    out = orig_bgr.copy()
    if mode == "black":
        out[~fg_mask] = 0
    elif mode == "solid":
        out[~fg_mask] = np.asarray(cfg["background"]["color"], dtype=np.uint8)
    elif mode == "image":
        img = cfg["background"]["image"]
        p = Path(img)
        if not p.is_absolute() and base_dir is not None:
            p = Path(base_dir) / img
        bg = load_background(p, (orig_bgr.shape[1], orig_bgr.shape[0]))
        out[~fg_mask] = bg[~fg_mask]
    else:  # pragma: no cover — config schema 已挡
        raise ValueError(f"unknown background mode: {mode}")
    return out
