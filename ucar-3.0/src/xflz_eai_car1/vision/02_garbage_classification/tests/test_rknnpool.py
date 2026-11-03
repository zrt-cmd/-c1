from __future__ import annotations

"""rknnpool 模块 TDD 测试（构建文档 05 的 7 条计划）。"""
import queue
import time

import numpy as np
import pytest

from detector import FakeBackend
from rknnpool import RKNNPool, rknn_backend_factory


class _FakePoolBackend(FakeBackend):
    """记录实例 id 的 FakeBackend。"""

    instance_count = 0

    def __init__(self, outputs=None):
        super().__init__(outputs or [np.zeros((1, 75, 1, 1), dtype=np.float32)])
        _FakePoolBackend.instance_count += 1
        self.instance_id = _FakePoolBackend.instance_count


def _factory(ids_seen: list[int], fail_load: bool = False):
    def factory(tid: int) -> FakeBackend:
        ids_seen.append(tid)

        class B(_FakePoolBackend):
            def load(self, model_path):
                if fail_load:
                    raise RuntimeError("load boom")
                self.loaded = model_path
        return B()
    return factory


def test_put_get_roundtrip():
    ids = []
    pool = RKNNPool(_factory(ids), "m.rknn", tpes=1,
                    process_fn=lambda backend, item: f"done:{item}")
    try:
        pool.put("a")
        result, ok = pool.get(timeout=5)
        assert (result, ok) == ("done:a", True)
    finally:
        pool.release()


def test_multi_task_order_preserved():
    pool = RKNNPool(_factory([]), "m.rknn", tpes=1,
                    process_fn=lambda backend, item: f"done:{item}")
    try:
        for i in range(5):
            pool.put(i)
        for i in range(5):
            result, ok = pool.get(timeout=5)
            assert (result, ok) == (f"done:{i}", True)
    finally:
        pool.release()


def test_factory_called_with_ids_0_to_tpes_minus_1():
    ids = []
    pool = RKNNPool(_factory(ids), "m.rknn", tpes=3,
                    process_fn=lambda backend, item: item)
    try:
        assert ids == [0, 1, 2]
    finally:
        pool.release()


def test_process_fn_error_emits_none_false():
    def boom(backend, item):
        raise ValueError("boom")

    pool = RKNNPool(_factory([]), "m.rknn", tpes=1, process_fn=boom)
    try:
        pool.put(1)
        result, ok = pool.get(timeout=5)
        assert (result, ok) == (None, False)
    finally:
        pool.release()


def test_backend_load_failure_emits_none_false():
    pool = RKNNPool(_factory([], fail_load=True), "m.rknn", tpes=1,
                    process_fn=lambda backend, item: item)
    try:
        result, ok = pool.get(timeout=5)
        assert (result, ok) == (None, False)
    finally:
        pool.release()


def test_release_stops_cleanly():
    pool = RKNNPool(_factory([]), "m.rknn", tpes=2,
                    process_fn=lambda backend, item: item)
    pool.put(1)
    pool.get(timeout=5)
    t0 = time.time()
    pool.release()
    assert time.time() - t0 < 3  # 不挂起


def test_release_closes_backends():
    backends = []

    def factory(tid):
        b = _FakePoolBackend()
        backends.append(b)
        return b

    pool = RKNNPool(factory, "m.rknn", tpes=2, process_fn=lambda b, i: i)
    pool.release()
    assert all(b.closed for b in backends)


def test_get_timeout_raises_empty():
    pool = RKNNPool(_factory([]), "m.rknn", tpes=1,
                    process_fn=lambda backend, item: item)
    try:
        with pytest.raises(queue.Empty):
            pool.get(timeout=1)
    finally:
        pool.release()


def test_unknown_core_mask_raises():
    with pytest.raises(ValueError, match="core_mask"):
        rknn_backend_factory("bogus")
