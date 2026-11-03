"""推理后端抽象（构建文档 05）：协议 + RKNN + Fake 实现。"""
from __future__ import annotations

from typing import Callable, Protocol

import numpy as np

from preprocess import prepare_input


class Backend(Protocol):
    def load(self, path: str) -> None: ...
    def predict(self, input_img: np.ndarray) -> list[np.ndarray]: ...
    def close(self) -> None: ...


class RKNNBackend:
    """板上推理后端：rknnlite.api.RKNNLite（RK3576 专用，2.3.2；init_runtime 无 target）。"""
    # 与 rknnlite.api.RKNNLite 常量一致（x86 测试无需 rknnlite 即可引用）
    NPU_CORE_AUTO = 0
    NPU_CORE_0 = 1
    NPU_CORE_1 = 2
    NPU_CORE_2 = 4
    NPU_CORE_0_1 = 3
    NPU_CORE_0_1_2 = 7

    def __init__(self, core_mask: int = 0, verbose: bool = False):
        self._core_mask = core_mask  # 0 = NPU_CORE_AUTO（板端实测：None 会报 core mode not supported）
        self._verbose = verbose
        self._rknn = None

    def load(self, path: str) -> None:
        from rknnlite.api import RKNNLite  # 延迟导入：纯函数测试无需 rknn

        rknn = RKNNLite(verbose=self._verbose)
        if rknn.load_rknn(path) != 0:
            raise RuntimeError(f"load_rknn failed: {path}")
        if rknn.init_runtime(core_mask=self._core_mask) != 0:
            raise RuntimeError("init_runtime failed")
        self._rknn = rknn

    def predict(self, input_img: np.ndarray) -> list[np.ndarray]:
        if self._rknn is None:
            raise RuntimeError("backend not loaded")
        # 模型量化 rgb2bgr=False：输入必须为 RGB；uint8 NHWC，运行时自动量化
        inp = prepare_input(input_img, 640)
        outs = self._rknn.inference(inputs=[inp], data_format="nhwc")
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

    def predict(self, input_img: np.ndarray) -> list[np.ndarray]:
        return self._fn(input_img)

    def close(self) -> None:
        self.loaded = None
