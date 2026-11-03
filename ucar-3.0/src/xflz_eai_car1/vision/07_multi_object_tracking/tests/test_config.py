"""config 模块 TDD 测试（构建文档 01 的 7 条计划）。"""
import pytest

from config import ConfigError, load_config

VALID_YAML = (
    "model:\n  det_rknn: models/yolov8n_coco.rknn\n  det_onnx: models/yolov8n_coco.onnx\n"
    "  reid_rknn: models/deepsort.rknn\n  imgsz: 640\n"
    "detect:\n  score_thr: 0.25\n  iou_thr: 0.45\n"
    "tracker:\n  mode: deepsort\n  max_dist: 0.2\n  max_iou_distance: 0.7\n"
    "  max_age: 70\n  n_init: 3\n  nn_budget: 100\n  min_confidence: 0.3\n"
    "pool:\n  workers: 3\n  queue_size: 4\n  core_mask: auto\n"
    "camera:\n  index: 0\n  width: 640\n  height: 480\n"
    "display:\n  show: false\n  window_name: deepsort_track\n"
    "server:\n  port: 8080\n  stream_name: track.mjpg\n"
)


@pytest.fixture
def valid_cfg(tmp_path):
    p = tmp_path / "track_config.yaml"
    p.write_text(VALID_YAML, encoding="utf-8")
    return p


def test_load_valid_config(valid_cfg):
    cfg = load_config(valid_cfg)
    assert cfg["tracker"]["mode"] == "deepsort"
    assert cfg["tracker"]["max_age"] == 70
    assert cfg["pool"]["core_mask"] == "auto"


def test_missing_section_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text(VALID_YAML.replace("server:", "xxx:"), encoding="utf-8")
    with pytest.raises(ConfigError, match="server.port"):
        load_config(p)


def test_wrong_type_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad_type.yaml"
    p.write_text(VALID_YAML.replace("max_age: 70", "max_age: many"), encoding="utf-8")
    with pytest.raises(ConfigError, match="tracker.max_age"):
        load_config(p)


def test_invalid_yaml_raises(tmp_path):
    p = tmp_path / "bad_yaml.yaml"
    p.write_text("key: [1, 2\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(p)


def test_bad_mode(valid_cfg, tmp_path):
    p = tmp_path / "bad_mode.yaml"
    p.write_text(VALID_YAML.replace("mode: deepsort", "mode: kcf"), encoding="utf-8")
    with pytest.raises(ConfigError, match="tracker.mode"):
        load_config(p)


def test_nonpositive_max_age(valid_cfg, tmp_path):
    p = tmp_path / "bad_age.yaml"
    p.write_text(VALID_YAML.replace("max_age: 70", "max_age: 0"), encoding="utf-8")
    with pytest.raises(ConfigError, match="positive"):
        load_config(p)


def test_bad_core_mask(valid_cfg, tmp_path):
    p = tmp_path / "bad_core.yaml"
    p.write_text(VALID_YAML.replace("core_mask: auto", "core_mask: 3"), encoding="utf-8")
    with pytest.raises(ConfigError, match="pool.core_mask"):
        load_config(p)
