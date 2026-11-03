"""共享 fixtures：临时配置工厂。"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
# python 包父目录：car1_pose_landmark/（其下 pose_landmark_core/ 为 python 包）
PKG = ROOT / "car1_pose_landmark"
if str(PKG) not in sys.path:
    sys.path.insert(0, str(PKG))


def _merge(dst, src):
    for k, v in src.items():
        if v is None:
            dst.pop(k, None)
        elif isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        else:
            dst[k] = v


@pytest.fixture
def make_config(tmp_path):
    """生成一份合法配置 yaml；overrides 递归覆盖，None 表示删除键。"""
    def _make(overrides: dict | None = None) -> Path:
        cfg = {
            "model": {
                "pose_model": "models/pose_landmark_full.rknn",
                "min_detection_confidence": 0.5,
            },
            "backend": {"core_mask": 0, "verbose": False},
            "source": {"mode": "image", "rgb_path": "images/pose_demo.jpg"},
            "display": {"show": False, "window_name": "Pose Detection"},
            "server": {"port": 8082, "stream_name": "pose.mjpg"},
        }
        if overrides:
            _merge(cfg, overrides)
        p = tmp_path / "pose_config.yaml"
        p.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
        return p

    return _make
