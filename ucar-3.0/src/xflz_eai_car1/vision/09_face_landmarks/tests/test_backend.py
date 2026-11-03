"""backend 模块测试（RKNNLiteBackend 契约；x86 无 rknnlite → 懒导入验证）。"""
from __future__ import annotations

import numpy as np
import pytest

from face_landmark_core.backend import (
    RKNNLiteBackend,
    FakeBackend,
)


def test_fake_backend_contract():
    calls = []

    def fn(img):
        calls.append(img.shape)
        return [np.zeros((1, 4), dtype=np.float32)]

    b = FakeBackend(fn=fn)
    b.load("x.rknn")
    assert b.loaded == "x.rknn"
    out = b.predict(np.zeros((1, 8), dtype=np.float32))
    assert len(out) == 1 and out[0].shape == (1, 4)
    assert calls == [(1, 8)]
    b.close()
    assert b.loaded is None


def test_fake_backend_default_fn_returns_input():
    b = FakeBackend()
    arr = np.zeros((2, 3), dtype=np.float32)
    assert b.predict(arr)[0] is arr


def test_rknnlite_predict_before_load_raises():
    b = RKNNLiteBackend()
    with pytest.raises(RuntimeError, match="not loaded"):
        b.predict(np.zeros((1, 128, 128, 3), dtype=np.float32))


def test_rknnlite_close_idempotent():
    b = RKNNLiteBackend()
    b.close()
    b.close()  # 未加载时 close 必须安全


def test_rknnlite_stores_core_mask_and_verbose():
    b = RKNNLiteBackend(core_mask=RKNNLiteBackend.NPU_CORE_0, verbose=True)
    assert b._core_mask == 1
    assert b._verbose is True
    assert b._rknn is None


def test_rknnlite_npu_core_constants():
    # 与 rknnlite.api.RKNNLite 常量对齐（RK3576 三核 NPU）
    assert RKNNLiteBackend.NPU_CORE_AUTO == 0
    assert RKNNLiteBackend.NPU_CORE_0 == 1
    assert RKNNLiteBackend.NPU_CORE_1 == 2
    assert RKNNLiteBackend.NPU_CORE_2 == 4
    assert RKNNLiteBackend.NPU_CORE_0_1 == 3
    assert RKNNLiteBackend.NPU_CORE_0_1_2 == 7


def test_rknnlite_load_is_lazy():
    """构造不 import rknnlite；load 才导入（x86 无包 → ModuleNotFoundError，
    板上有包但路径坏 → RuntimeError）。"""
    import importlib.util

    b = RKNNLiteBackend()
    assert b._rknn is None
    if importlib.util.find_spec("rknnlite") is None:
        with pytest.raises(ModuleNotFoundError):
            b.load("models/face_detection_short_range.rknn")
    else:
        with pytest.raises(RuntimeError, match="load_rknn failed"):
            b.load("models/does_not_exist.rknn")
