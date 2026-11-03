"""预处理：letterbox 填充（与 rknn_model_zoo 对齐）。

契约（TDD 断言目标）：
- letterbox(img, size, color) -> LetterboxResult(padded, scale, pad_x, pad_y)
  等比缩放 + 居中填充；scale = min(size/h, size/w)。
- unletterbox(boxes, scale, pad_x, pad_y) -> boxes 原始图像坐标。

约定：pad_color 默认 (0,0,0)，与 rknn_model_zoo / RGA 管线一致；
      训练侧 ultralytics 用 114 填充，但部署推理必须用 0。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import cv2


@dataclass(frozen=True)
class LetterboxResult:
    padded: np.ndarray
    scale: float
    pad_x: int
    pad_y: int


def letterbox(img: np.ndarray, size: int = 640, color: tuple = (0, 0, 0)) -> LetterboxResult:
    """等比缩放并居中填充到 (size, size)。输入 BGR HWC uint8。"""
    h, w = img.shape[:2]
    scale = min(size / h, size / w)
    new_h, new_w = int(round(h * scale)), int(round(w * scale))
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_x = (size - new_w) // 2
    pad_y = (size - new_h) // 2
    padded = np.full((size, size, 3), color, dtype=np.uint8)
    padded[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return LetterboxResult(padded=padded, scale=scale, pad_x=pad_x, pad_y=pad_y)


def unletterbox(boxes: np.ndarray, lb: LetterboxResult) -> np.ndarray:
    """把 letterbox 坐标映射回原始图像坐标。boxes: (N, 4) xyxy。"""
    return (boxes - np.array([lb.pad_x, lb.pad_y, lb.pad_x, lb.pad_y])) / lb.scale


def to_rgb(bgr_img: np.ndarray) -> np.ndarray:
    """BGR -> RGB 通道交换（返回拷贝）。

    依据：本 fork 导出的 rknn 模型量化配置 rgb2bgr=False（见 .rknn 元数据），
    与 rknn_model_zoo 一致——推理输入必须是 RGB；letterbox 输出为 BGR，
    喂给后端前必须交换。
    """
    return bgr_img[..., ::-1].copy()
