"""ReID 外观特征提取（构建文档 05；vendor 契约）。

输入：目标 crop（BGR）→ resize 64x128 → /255 → ImageNet mean/std（RGB）→
NCHW [1,3,128,64] → 特征向量（dim 由模型决定，cosine 度量维度无关）。
后端可注入：RKNN（板载）/ Fake（测试）。
多线程：见 reidpool.ReIDPool（N worker × 独立 RKNN 实例）。
注：用户 2026-08-05 重导模型为单张输入（batch=1），与本实现一致。
"""
from __future__ import annotations

from typing import Callable

import numpy as np
import cv2

MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def preprocess_crop(crop_bgr: np.ndarray) -> np.ndarray:
    """crop -> [1,3,128,64] float32（vendor feature_extractor 同款）。"""
    im = cv2.resize(crop_bgr.astype(np.float32) / 255.0, (64, 128))
    im = im[..., ::-1].copy()  # BGR -> RGB
    im = (im - MEAN) / STD
    return im.transpose(2, 0, 1)[None, ...]


class ReIDBackend:
    """RKNN ReID 提取器（输入 64x128 ImageNet 归一化，单张 [1,3,128,64]；RKNNLite）。"""

    def __init__(self, verbose: bool = False):
        self._rknn = None
        self._verbose = verbose

    def load(self, path: str) -> None:
        from rknnlite.api import RKNNLite

        rknn = RKNNLite(verbose=self._verbose)
        if rknn.load_rknn(path) != 0:
            raise RuntimeError(f"load_rknn failed: {path}")
        if rknn.init_runtime() != 0:
            raise RuntimeError("init_runtime failed")
        self._rknn = rknn

    def __call__(self, crops: list[np.ndarray]) -> np.ndarray:
        """crops -> (N, dim) float32，逐张推理。"""
        if self._rknn is None:
            raise RuntimeError("reid backend not loaded")
        feats = []
        for c in crops:
            inp = preprocess_crop(c)
            out = self._rknn.inference(inputs=[inp])  # float32 NCHW，默认 layout
            feats.append(np.asarray(out[0], dtype=np.float32).reshape(1, -1))
        return np.concatenate(feats, axis=0)

    def close(self) -> None:
        if self._rknn is not None:
            self._rknn.release()
            self._rknn = None


class FakeReID:
    """确定性测试后端：特征 = crop 亮度哈希；extract 与 ReIDPool 同接口。"""

    def __init__(self, fn: Callable[[list[np.ndarray]], np.ndarray] | None = None):
        self._fn = fn

    def load(self, path: str) -> None:
        pass

    def __call__(self, crops: list[np.ndarray]) -> np.ndarray:
        if self._fn is not None:
            return self._fn(crops)
        out = []
        for c in crops:
            v = np.zeros(128, dtype=np.float32)
            v[0] = float(np.mean(c))
            out.append(v)
        return np.asarray(out, dtype=np.float32)

    def extract(self, crops: list[np.ndarray]) -> np.ndarray:
        return self.__call__(crops)

    def close(self) -> None:
        pass
