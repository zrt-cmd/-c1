"""预处理：letterbox 640（pad 0）+ BGR→RGB（模型元数据 rgb2bgr=False）。

契约（TDD 断言目标）：
- letterbox(img, size=640, color=(0,0,0)) -> LetterboxResult(padded, scale, pad_x, pad_y)
- unletterbox(boxes, scale, pad_x, pad_y) -> boxes（letterbox 空间 -> 原图）
- to_rgb(bgr_img) -> np.ndarray（拷贝交换）
- prepare_input(img, size=640) -> (rgb_padded, lb)：推理输入 + 元信息
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
    """等比缩放 + 居中填充到 (size, size)。输入 BGR HWC uint8。"""
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
    """BGR -> RGB 通道交换（返回拷贝，不改原帧）。"""
    return bgr_img[..., ::-1].copy()


def prepare_input(img: np.ndarray, size: int = 640) -> tuple[np.ndarray, LetterboxResult]:
    """完整预处理链：letterbox -> RGB。返回 (模型输入, letterbox 元信息)。"""
    lb = letterbox(img, size)
    return to_rgb(lb.padded), lb
