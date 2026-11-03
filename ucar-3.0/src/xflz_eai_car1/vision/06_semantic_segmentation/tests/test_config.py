"""config 模块 TDD 测试（构建文档 01 的 7 条计划）。"""
import pytest

from config import ConfigError, load_config

VALID_YAML = (
    "model:\n  rknn: models/yolov8n_seg.rknn\n  onnx: models/yolov8n_seg.onnx\n"
    "  imgsz: 640\n  score_thr: 0.25\n  iou_thr: 0.45\n  mask_thr: 0.5\n"
    "  classes: [0]\n"
    "pool:\n  workers: 3\n  queue_size: 4\n  core_mask: auto\n"
    "background:\n  mode: black\n  color: [0, 0, 0]\n  image: \"\"\n"
    "camera:\n  index: 0\n  width: 640\n  height: 480\n"
    "display:\n  show: false\n  window_name: seg_background\n"
    "server:\n  port: 8080\n  stream_name: seg.mjpg\n"
)


@pytest.fixture
def valid_cfg(tmp_path):
    p = tmp_path / "seg_config.yaml"
    p.write_text(VALID_YAML, encoding="utf-8")
    return p


def test_load_valid_config(valid_cfg):
    cfg = load_config(valid_cfg)
    assert cfg["model"]["classes"] == [0]
    assert cfg["pool"]["core_mask"] == "auto"
    assert cfg["background"]["mode"] == "black"


def test_missing_section_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text(VALID_YAML.replace("server:", "xxx:"), encoding="utf-8")
    with pytest.raises(ConfigError, match="server.port"):
        load_config(p)


def test_wrong_type_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad_type.yaml"
    p.write_text(VALID_YAML.replace("mask_thr: 0.5", "mask_thr: high"), encoding="utf-8")
    with pytest.raises(ConfigError, match="model.mask_thr"):
        load_config(p)


def test_invalid_yaml_raises(tmp_path):
    p = tmp_path / "bad_yaml.yaml"
    p.write_text("key: [1, 2\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(p)


def test_bad_background_mode(valid_cfg, tmp_path):
    p = tmp_path / "bad_mode.yaml"
    p.write_text(VALID_YAML.replace("mode: black", "mode: green"), encoding="utf-8")
    with pytest.raises(ConfigError, match="background.mode"):
        load_config(p)


def test_solid_requires_3_color(valid_cfg, tmp_path):
    p = tmp_path / "bad_color.yaml"
    p.write_text(VALID_YAML.replace("mode: black", "mode: solid")
                       .replace("color: [0, 0, 0]", "color: [0, 0]"), encoding="utf-8")
    with pytest.raises(ConfigError, match="background.color"):
        load_config(p)


def test_image_mode_requires_path(valid_cfg, tmp_path):
    p = tmp_path / "bad_image.yaml"
    p.write_text(VALID_YAML.replace("mode: black", "mode: image"), encoding="utf-8")
    with pytest.raises(ConfigError, match="background.image"):
        load_config(p)


def test_bad_core_mask(valid_cfg, tmp_path):
    p = tmp_path / "bad_core.yaml"
    p.write_text(VALID_YAML.replace("core_mask: auto", "core_mask: 3"), encoding="utf-8")
    with pytest.raises(ConfigError, match="pool.core_mask"):
        load_config(p)
