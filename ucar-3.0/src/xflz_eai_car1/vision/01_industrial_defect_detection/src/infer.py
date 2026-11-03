"""推理抽象：可注入后端（rknn / onnxruntime）+ 检测器编排。

契约（TDD 断言目标）：
- Backend(Protocol): load(path) / predict(input) -> list[np.ndarray] / close()
  input: letterbox 后的 (imgsz, imgsz, 3) uint8 BGR 帧
  predict 返回模型原始输出列表（每层 3 个张量，见 postprocess.decode_rknn）
- RKNNBackend / ONNXBackend / FakeBackend（测试用）实现该协议
- Detector(backend, num_classes, imgsz, score_thr, iou_thr).detect(bgr_img) -> list[Detection]
  返回**原始图像坐标**的检测；detect 内部完成 letterbox→predict→decode→unletterbox。
- Detector 支持 with 语法（enter/exit 关闭后端）。
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence

import numpy as np

from postprocess import Detection, decode_rknn
from preprocess import letterbox, to_rgb, unletterbox


class Backend(Protocol):
    def load(self, model_path: str) -> None: ...
    def predict(self, input_img: np.ndarray) -> list[np.ndarray]: ...
    def close(self) -> None: ...


class RKNNBackend:
    """板上推理后端：rknnlite.api.RKNNLite（RK3576 专用，2.3.2；init_runtime 无 target）。"""

    def __init__(self, core_mask: int = 0, verbose: bool = False):
        from rknnlite.api import RKNNLite  # 延迟导入：纯函数测试无需 rknnlite

        self._rknn = RKNNLite(verbose=verbose)
        self._core_mask = core_mask  # 0 = NPU_CORE_AUTO（板端实测：None 报 core mode not supported）

    def load(self, model_path: str) -> None:
        if self._rknn.load_rknn(model_path) != 0:
            raise RuntimeError(f"load_rknn failed: {model_path}")
        if self._rknn.init_runtime(core_mask=self._core_mask) != 0:
            raise RuntimeError("init_runtime failed")

    def predict(self, input_img: np.ndarray) -> list[np.ndarray]:
        # 模型量化配置 rgb2bgr=False：推理输入必须是 RGB（模型元数据核实）
        # static_shape 模型需 4D NHWC（板端实测：3D 报 need 4dims input）
        outs = self._rknn.inference(inputs=[to_rgb(input_img)[np.newaxis, ...]],
                                    data_format="nhwc")
        return list(outs)

    def close(self) -> None:
        self._rknn.release()


class ONNXBackend:
    """onnxruntime 后端（板上已装 1.23.2；用于 torch/onnx/rknn 三方对比的中间档）。"""

    def __init__(self, providers: Sequence[str] | None = None):
        import onnxruntime as ort  # 延迟导入

        self._providers = list(providers) if providers else ["CPUExecutionProvider"]
        self._sess = None

    def load(self, model_path: str) -> None:
        import onnxruntime as ort

        self._sess = ort.InferenceSession(model_path, providers=self._providers)
        if not self._sess.get_inputs():
            raise RuntimeError(f"onnx model has no inputs: {model_path}")
        self._input_name = self._sess.get_inputs()[0].name

    def predict(self, input_img: np.ndarray) -> list[np.ndarray]:
        # 模型输入为 NCHW float32 /255（rknn 侧由量化内部处理，onnx 侧手动归一化）
        inp = to_rgb(input_img).astype(np.float32).transpose(2, 0, 1)[None] / 255.0
        outs = self._sess.run(None, {self._input_name: inp})
        return list(outs)

    def close(self) -> None:
        self._sess = None


class FakeBackend:
    """测试用后端：返回预设输出（须与 decode_rknn 输入形状一致）。"""

    def __init__(self, outputs: list[np.ndarray]):
        self.outputs = outputs
        self.loaded = None
        self.closed = False

    def load(self, model_path: str) -> None:
        self.loaded = model_path

    def predict(self, input_img: np.ndarray) -> list[np.ndarray]:
        return self.outputs

    def close(self) -> None:
        self.closed = True


@dataclass
class DetectorConfig:
    imgsz: int = 640
    score_thr: float = 0.25
    iou_thr: float = 0.45
    pad_color: tuple = (0, 0, 0)


class Detector:
    def __init__(self, backend: Backend, num_classes: int, cfg: DetectorConfig | None = None):
        self.backend = backend
        self.num_classes = num_classes
        self.cfg = cfg or DetectorConfig()

    def load(self, model_path: str) -> "Detector":
        self.backend.load(model_path)
        return self

    def detect(self, bgr_img: np.ndarray) -> list[Detection]:
        lb = letterbox(bgr_img, self.cfg.imgsz, color=self.cfg.pad_color)
        outputs = self.backend.predict(lb.padded)
        dets = decode_rknn(outputs, self.num_classes, self.cfg.imgsz,
                           score_thr=self.cfg.score_thr, iou_thr=self.cfg.iou_thr)
        if not dets:
            return []
        boxes = np.array([[d.x1, d.y1, d.x2, d.y2] for d in dets], dtype=np.float32)
        boxes_orig = unletterbox(boxes, lb)
        return [
            Detection(float(b[0]), float(b[1]), float(b[2]), float(b[3]), d.score, d.class_id)
            for d, b in zip(dets, boxes_orig)
        ]

    def close(self) -> None:
        self.backend.close()

    def __enter__(self) -> "Detector":
        return self

    def __exit__(self, *exc) -> None:
        self.close()
