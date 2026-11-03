"""实验 01 共享测试夹具：src 导入路径、样例图像、迷你 YOLO 数据集。"""
import sys
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

NEU_DET = Path("/home/iflytek/Src/AIoT_B2/NEU-DET.v1i.yolov8")


@pytest.fixture
def sample_img() -> np.ndarray:
    """480x640 三通道 BGR 测试图（含不同亮度区域，便于 letterbox 断言）。"""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[100:200, 200:300] = (255, 0, 0)
    img[300:400, 400:500] = (0, 255, 0)
    return img


@pytest.fixture
def mini_yolo(tmp_path) -> Path:
    """构造 3 图/3 标签的迷你 YOLO 数据集（含 1 个非法标签行）。"""
    root = tmp_path / "mini_dataset"
    (root / "train" / "images").mkdir(parents=True)
    (root / "train" / "labels").mkdir(parents=True)
    (root / "data.yaml").write_text("train: ../train/images\nnc: 2\nnames: ['a', 'b']\n", encoding="utf-8")
    for i in range(3):
        (root / "train" / "images" / f"img{i}.jpg").write_bytes(b"\xff\xd8\xff")
        (root / "train" / "labels" / f"img{i}.txt").write_text("0 0.5 0.5 0.4 0.4\n", encoding="utf-8")
    # 越界类别 -> 非法
    (root / "train" / "labels" / "img0.txt").write_text(
        "0 0.5 0.5 0.4 0.4\n5 0.5 0.5 0.4 0.4\n", encoding="utf-8")
    return root


@pytest.fixture
def valid_config(tmp_path) -> Path:
    p = tmp_path / "cfg.yaml"
    p.write_text(
        "dataset:\n"
        "  root: /x\n  data_yaml: data.yaml\n  splits: [train, valid]\n"
        "train:\n  fork: /f\n  model: yolov8n.pt\n  imgsz: 640\n  epochs: 100\n"
        "  batch: 8\n  device: 0\n  project: runs\n  name: n\n"
        "infer:\n  imgsz: 640\n  score_thr: 0.25\n  iou_thr: 0.45\n  pad_color: [0, 0, 0]\n"
        "rknn:\n  target_platform: rk3576\n  quantize: true\n  calib_count: 200\n"
        "  onnx_model: m.onnx\n  rknn_model: m.rknn\n"
        "eval:\n  split: valid\n  iou_thr: 0.5\n",
        encoding="utf-8")
    return p
