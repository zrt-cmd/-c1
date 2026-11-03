"""模型输入预处理：letterbox（pad 0）+ RGB（构建文档 02）。

语义与 exp01/exp02 一致（rknn_model_zoo 对齐）：等比缩放、黑边 0 填充、
RKNN 喂 uint8 NHWC RGB（mean/std 0/255 由模型量化处理）。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import cv2


@dataclass
class LetterboxResult:
    padded: np.ndarray  # (imgsz, imgsz, 3) uint8
    ratio: float
    pad_x: int         # 左右黑边像素
    pad_y: int         # 上下黑边像素

    def unletterbox(self, boxes: np.ndarray) -> np.ndarray:
        """模型坐标 (x1,y1,x2,y2, 640 系) -> 原图坐标。"""
        xy = (boxes[:, :4].astype(np.float64) - np.array([self.pad_x, self.pad_y] * 2)) / self.ratio
        if boxes.shape[1] > 4:
            xy = np.concatenate([xy, boxes[:, 4:]], axis=1)
        return xy


def letterbox(img: np.ndarray, imgsz: int = 640, color=(0, 0, 0)) -> LetterboxResult:
    h, w = img.shape[:2]
    ratio = imgsz / max(h, w)
    new_w, new_h = int(round(w * ratio)), int(round(h * ratio))
    resized = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    pad_x = (imgsz - new_w) // 2
    pad_y = (imgsz - new_h) // 2
    canvas = np.full((imgsz, imgsz, 3), color, dtype=np.uint8)
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return LetterboxResult(padded=canvas, ratio=ratio, pad_x=pad_x, pad_y=pad_y)


def to_rgb(bgr: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)


def prepare_input(bgr: np.ndarray, imgsz: int = 640) -> np.ndarray:
    """-> [1, imgsz, imgsz, 3] uint8 RGB（RKNN data_format='nhwc'）。"""
    lb = letterbox(bgr, imgsz)
    return to_rgb(lb.padded)[None, ...]
