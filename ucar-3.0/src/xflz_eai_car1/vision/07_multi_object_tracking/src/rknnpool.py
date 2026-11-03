"""RKNN 推理线程池（构建文档 05；参照 exp02 rknnpool 设计改造）。

1. 后端工厂注入：板上统一 rknnlite（`rknnlite.api.RKNNLite`，2.3.2，RK3576 专用）；
   测试注入 FakeBackend。
2. 核轮转：`round_robin` 按线程 id % 3 绑定 NPU_CORE_0/1/2（RK3576 三核）。
"""
from __future__ import annotations

import queue
import threading
from typing import Callable

from backend import Backend, RKNNBackend


def backend_factory(core_mask: str = "auto") -> Callable[[int], Backend]:
    """构造后端工厂；round_robin 时按线程 id % 3 轮转 NPU 核心。"""
    if core_mask not in ("auto", "round_robin"):
        raise ValueError(f"unknown core_mask: {core_mask} (auto|round_robin)")
    if core_mask == "round_robin":
        masks = [RKNNBackend.NPU_CORE_0, RKNNBackend.NPU_CORE_1, RKNNBackend.NPU_CORE_2]
        return lambda tid: RKNNBackend(core_mask=masks[tid % 3])
    return lambda tid: RKNNBackend()


class RKNNPool:
    def __init__(
        self,
        factory: Callable[[int], Backend],
        model_path: str,
        tpes: int,
        process_fn: Callable[[Backend, object], object],
    ):
        self.tpes = tpes
        self.process_fn = process_fn
        self.model_path = model_path
        self.inputs: queue.Queue = queue.Queue()
        self.outputs: queue.Queue = queue.Queue()
        self.running = True
        self.threads = []
        self._backends = []
        for i in range(tpes):
            backend = factory(i)
            self._backends.append(backend)
            t = threading.Thread(target=self._worker, args=(i, backend), daemon=True)
            t.start()
            self.threads.append(t)

    def _worker(self, thread_id: int, backend: Backend) -> None:
        try:
            backend.load(self.model_path)
        except Exception as exc:  # pragma: no cover - 板载环境路径
            print(f"[pool:{thread_id}] load failed: {exc}")
            self.outputs.put((None, False))
            return
        while self.running:
            try:
                item = self.inputs.get(timeout=1)
            except queue.Empty:
                continue
            try:
                result = self.process_fn(backend, item)
                self.outputs.put((result, True))
            except Exception as exc:
                print(f"[pool:{thread_id}] error: {exc}")
                self.outputs.put((None, False))
                break
        backend.close()

    def put(self, item) -> None:
        self.inputs.put(item)

    def get(self, timeout: float | None = None) -> tuple[object, bool]:
        return self.outputs.get(timeout=timeout)

    def release(self) -> None:
        self.running = False
        for t in self.threads:
            t.join(timeout=5)
        for b in self._backends:
            b.close()
