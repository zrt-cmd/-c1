"""推理抽象：后端注入（rknn / fake）+ 检测编排（构建文档 04）。

契约（TDD 断言目标）：
- Backend(Protocol): load(path) / predict(rgb_input) -> list[np.ndarray] / close()
  predict 输入：640×640×3 uint8 RGB（prepare_input 产物）；输出 3 尺度 (1,75,h,w)
- RKNNBackend(core_mask=None)：板上 rknnlite（RKNNLite，无 target）；可指定核掩码（pool 轮转）
- FakeBackend：预设输出、记录 load/close
- Detector(backend, labels, cat_map, cat_names, cfg): detect(bgr_img)
  -> list[CategoryDetection]（原图坐标 + 4 大类）；with 语法支持
- make_rknn_backend() -> RKNNBackend
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from postprocess import Detection, decode_rknn_outputs, map_categories
from preprocess import prepare_input, unletterbox

DEFAULT_ANCHORS = [(10, 13), (16, 30), (33, 23), (30, 61), (62, 45),
                   (59, 119), (116, 90), (156, 198), (373, 326)]


class Backend(Protocol):
    def load(self, model_path: str) -> None: ...
    def predict(self, rgb_input: np.ndarray) -> list[np.ndarray]: ...
    def close(self) -> None: ...


class RKNNBackend:
    """板上推理后端：rknnlite.api.RKNNLite（RK3576 专用，2.3.2；可指定核掩码）。"""
    # 与 rknnlite.api.RKNNLite 常量一致（x86 测试无需 rknnlite 即可引用）
    NPU_CORE_AUTO = 0
    NPU_CORE_0 = 1
    NPU_CORE_1 = 2
    NPU_CORE_2 = 4
    NPU_CORE_0_1 = 3
    NPU_CORE_0_1_2 = 7

    def __init__(self, core_mask: int = 0, verbose: bool = False):
        self._rknn = None
        self._core_mask = core_mask  # 0 = NPU_CORE_AUTO（板端实测：None 会报 core mode not supported）
        self._verbose = verbose

    def load(self, model_path: str) -> None:
        from rknnlite.api import RKNNLite  # 延迟导入：纯函数测试无需 rknnlite

        self._rknn = RKNNLite(verbose=self._verbose)
        if self._rknn.load_rknn(model_path) != 0:
            raise RuntimeError(f"load_rknn failed: {model_path}")
        if self._rknn.init_runtime(core_mask=self._core_mask) != 0:
            raise RuntimeError("init_runtime failed")

    def predict(self, rgb_input: np.ndarray) -> list[np.ndarray]:
        # 模型为 static_shape，输入需 4D NHWC（板端实测：3D 报 need 4dims input）
        outs = self._rknn.inference(inputs=[rgb_input[np.newaxis, ...]], data_format="nhwc")
        if outs is None or len(outs) != 3:
            raise RuntimeError(f"expected 3 outputs, got {None if outs is None else len(outs)}")
        return list(outs)

    def close(self) -> None:
        self._rknn.release()


class FakeBackend:
    """测试用后端：返回预设 3 尺度输出。"""

    def __init__(self, outputs: list[np.ndarray]):
        self.outputs = outputs
        self.loaded = None
        self.closed = False

    def load(self, model_path: str) -> None:
        self.loaded = model_path

    def predict(self, rgb_input: np.ndarray) -> list[np.ndarray]:
        return list(self.outputs)

    def close(self) -> None:
        self.closed = True


@dataclass
class DetectorConfig:
    imgsz: int = 640
    score_thr: float = 0.25
    nms_thr: float = 0.45
    sigmoid: bool = True
    anchors: list = field(default_factory=lambda: list(DEFAULT_ANCHORS))


class Detector:
    def __init__(self, backend: Backend, labels: list[str], cat_map: list[int],
                 cat_names: dict[str, str], cfg: DetectorConfig | None = None):
        self.backend = backend
        self.labels = labels
        self.cat_map = cat_map
        self.cat_names = cat_names
        self.cfg = cfg or DetectorConfig()

    def load(self, model_path: str) -> "Detector":
        self.backend.load(model_path)
        return self

    def detect(self, bgr_img: np.ndarray) -> list:
        """完整链路：letterbox+RGB -> predict -> yolov5 解码 -> unletterbox -> 4 大类。"""
        inp, lb = prepare_input(bgr_img, self.cfg.imgsz)
        outputs = self.backend.predict(inp)
        dets = decode_rknn_outputs(outputs, self.cfg.anchors, self.cfg.imgsz,
                                   self.cfg.score_thr, self.cfg.nms_thr, self.cfg.sigmoid)
        if not dets:
            return []
        boxes = np.array([[d.x1, d.y1, d.x2, d.y2] for d in dets], dtype=np.float32)
        boxes_orig = unletterbox(boxes, lb)
        dets_orig = [
            Detection(float(b[0]), float(b[1]), float(b[2]), float(b[3]), d.score, d.class_id)
            for d, b in zip(dets, boxes_orig)
        ]
        return map_categories(dets_orig, self.cat_map, self.cat_names)

    def close(self) -> None:
        self.backend.close()

    def __enter__(self) -> "Detector":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def make_rknn_backend() -> RKNNBackend:
    """app 入口用后端工厂。"""
    return RKNNBackend()
