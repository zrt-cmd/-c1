"""reidpool 模块 TDD 测试（构建文档 05：多线程保序）。"""
import numpy as np
import pytest

from reid import FakeReID
from reidpool import ReIDPool


def test_extract_keeps_submission_order():
    def fn(crops):
        return np.array([np.full(128, float(np.mean(c)), dtype=np.float32)
                         for c in crops])

    pool = ReIDPool("dummy.rknn", workers=3, factory=lambda: FakeReID(fn))
    try:
        crops = [np.full((40, 20, 3), v, dtype=np.uint8) for v in (10, 250, 80, 150)]
        feats = pool.extract(crops)
        assert feats.shape == (4, 128)
        # 顺序与输入一致：亮度单调
        assert list(feats[:, 0]) == pytest.approx([10.0, 250.0, 80.0, 150.0])
    finally:
        pool.close()


def test_extract_empty_returns_empty():
    pool = ReIDPool("dummy.rknn", workers=2, factory=FakeReID)
    try:
        feats = pool.extract([])
        assert feats.shape == (0, 128)
    finally:
        pool.close()
