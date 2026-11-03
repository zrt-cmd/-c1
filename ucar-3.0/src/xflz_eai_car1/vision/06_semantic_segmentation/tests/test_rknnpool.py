"""rknnpool 模块 TDD 测试（构建文档 05 的 4 条计划）。"""
import queue
import time

import numpy as np
import pytest

from backend import FakeBackend
from rknnpool import RKNNPool, backend_factory


def _fake_factory(echo=True, fail_on=None):
    def fn(img):
        if fail_on is not None and img[0] == fail_on:
            raise RuntimeError("boom")
        return [img]

    return lambda tid: FakeBackend(fn)


def test_pool_put_get_ordered():
    pool = RKNNPool(_fake_factory(), "dummy.rknn", tpes=2,
                    process_fn=lambda b, item: (b, item))
    for i in range(5):
        pool.put(i)
    results = [pool.get(timeout=5)[0] for _ in range(5)]
    assert sorted(r[1] for r in results) == [0, 1, 2, 3, 4]
    pool.release()


def test_pool_worker_exception_returns_false():
    pool = RKNNPool(_fake_factory(fail_on=3), "dummy.rknn", tpes=2,
                    process_fn=lambda b, item: b.predict(np.array([item]))[0][0] == item)
    pool.put(1)
    ok, _ = pool.get(timeout=5)
    assert bool(ok) is True
    pool.put(3)  # 触发异常 worker
    res, ok2 = pool.get(timeout=5)
    assert res is None and ok2 is False
    pool.release()


def test_pool_load_failure():
    def failing_factory(tid):
        class Bad:
            def load(self, path):
                raise RuntimeError("no model")

            def close(self):
                pass

        return Bad()

    pool = RKNNPool(failing_factory, "missing.rknn", tpes=1, process_fn=lambda b, i: i)
    res, ok = pool.get(timeout=5)
    assert res is None and ok is False
    pool.release()


def test_round_robin_factory_pins_cores(monkeypatch):
    seen = []

    class FakeRKNN:
        NPU_CORE_0, NPU_CORE_1, NPU_CORE_2 = 0, 1, 2

        def __init__(self, core_mask):
            seen.append(core_mask)

    # 替换 rknnpool 命名空间里的 RKNNBackend：工厂从类常量读 NPU 核掩码
    monkeypatch.setattr("rknnpool.RKNNBackend", FakeRKNN)
    factory = backend_factory("round_robin")
    for tid in (0, 1, 2, 3, 4):
        factory(tid)
    assert seen == [0, 1, 2, 0, 1]

def test_unknown_core_mask_raises():
    with pytest.raises(ValueError, match="core_mask"):
        backend_factory("bogus")


def test_get_timeout_raises_queue_empty():
    pool = RKNNPool(_fake_factory(), "dummy.rknn", tpes=1, process_fn=lambda b, i: i)
    with pytest.raises(queue.Empty):
        pool.get(timeout=0.2)
    pool.release()
