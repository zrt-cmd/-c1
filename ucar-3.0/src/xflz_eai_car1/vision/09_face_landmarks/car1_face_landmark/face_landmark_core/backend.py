"""推理后端抽象（复制自实验 07 模式）：协议 + RKNNLite + Fake 实现。

板上部署统一走 **rknnlite.api.RKNNLite**（rknn-toolkit-lite2 2.3.2，
RK3576 专用推理 API，2026-08-06 用户决策：替换所有 rknn.api.RKNN 推理路径）。
完整版 rknn-toolkit2 仅保留在 x86 转换环境（convert_mediapipe.py）。
"""
from __future__ import annotations

from typing import Callable, Protocol

import numpy as np


class Backend(Protocol):
    def load(self, path: str) -> None: ...
    def predict(self, input_arr: np.ndarray) -> list[np.ndarray]: ...
    def close(self) -> None: ...


class RKNNLiteBackend:
    """板上推理后端：rknnlite.api.RKNNLite（RK3576 专用，2.3.2）。

    输入契约（Blaze）：float32 RGB NHWC [0,1]（归一化在调用方完成）。
    init_runtime(core_mask=...) 无 target 参数：lite 只在板上跑 NPU，不走 simulator；
    core_mask 语义 = RKNNLite.NPU_CORE_AUTO(0) / NPU_CORE_0(1) / 1(2) / 2(4)，
    多核并行用 0_1(3) / 0_1_2(7)。
    """
    # 与 rknnlite.api.RKNNLite 常量一致（不 import 也可引用）
    NPU_CORE_AUTO = 0
    NPU_CORE_0 = 1
    NPU_CORE_1 = 2
    NPU_CORE_2 = 4
    NPU_CORE_0_1 = 3
    NPU_CORE_0_1_2 = 7

    def __init__(self, core_mask: int = NPU_CORE_AUTO, verbose: bool = False):
        self._core_mask = core_mask
        self._verbose = verbose
        self._rknn = None

    def load(self, path: str) -> None:
        from rknnlite.api import RKNNLite  # 延迟导入：x86 单测无需 rknnlite

        rknn = RKNNLite(verbose=self._verbose)
        if rknn.load_rknn(path) != 0:
            raise RuntimeError(f"load_rknn failed: {path}")
        if rknn.init_runtime(core_mask=self._core_mask) != 0:
            raise RuntimeError("init_runtime failed")
        self._rknn = rknn

    def predict(self, input_arr) -> list[np.ndarray]:
        """单输入传 np.ndarray；多输入模型（如 gesture_embedder 3 输入）传 list。"""
        if self._rknn is None:
            raise RuntimeError("backend not loaded")
        inputs = input_arr if isinstance(input_arr, (list, tuple)) else [input_arr]
        outs = self._rknn.inference(inputs=inputs, data_format="nhwc")
        return list(outs)

    def close(self) -> None:
        if self._rknn is not None:
            self._rknn.release()
            self._rknn = None


class FakeBackend:
    """确定性测试后端。"""

    def __init__(self, fn: Callable[[np.ndarray], list[np.ndarray]] | None = None):
        self._fn = fn or (lambda img: [img])
        self.loaded = None

    def load(self, path: str) -> None:
        self.loaded = path

    def predict(self, input_arr: np.ndarray) -> list[np.ndarray]:
        return self._fn(input_arr)

    def close(self) -> None:
        self.loaded = None
