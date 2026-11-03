"""config 模块测试。"""
from __future__ import annotations

import pytest
import yaml

from face_landmark_core.config import ConfigError, load_config


def test_loads_valid_config(make_config):
    cfg = load_config(make_config())
    assert cfg["model"]["num_faces"] == 2
    assert cfg["backend"]["core_mask"] == 0
    assert cfg["server"]["port"] == 8081
    assert cfg["source"]["mode"] == "image"


def test_missing_file_raises(make_config, tmp_path):
    with pytest.raises(ConfigError, match="not found"):
        load_config(tmp_path / "nope.yaml")


def test_invalid_yaml_raises(make_config, tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("a: [unclosed", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(p)


def test_non_mapping_root_raises(make_config, tmp_path):
    p = tmp_path / "list.yaml"
    p.write_text("- 1\n- 2\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="mapping"):
        load_config(p)


@pytest.mark.parametrize("key,path", [
    ("model", None),
    ("backend", None),
    ("source", None),
    ("display", None),
    ("server", None),
])
def test_missing_section_raises(make_config, key, path):
    cfg_path = make_config({key: None})
    with pytest.raises(ConfigError, match="missing required keys"):
        load_config(cfg_path)


@pytest.mark.parametrize("bad", [
    {"model": {"num_faces": "two"}},
    {"model": {"min_detection_confidence": "high"}},
    {"backend": {"core_mask": "auto"}},
    {"backend": {"verbose": "yes"}},
    {"server": {"port": "8081"}},
    {"display": {"show": "yes"}},
])
def test_wrong_type_raises(make_config, bad):
    with pytest.raises(ConfigError, match="expected"):
        load_config(make_config(bad))


def test_num_faces_positive(make_config):
    with pytest.raises(ConfigError, match="num_faces"):
        load_config(make_config({"model": {"num_faces": 0}}))
    with pytest.raises(ConfigError, match="num_faces"):
        load_config(make_config({"model": {"num_faces": -1}}))


@pytest.mark.parametrize("bad", [0.0, 1.5, -0.1])
def test_confidence_range(make_config, bad):
    with pytest.raises(ConfigError, match="confidence"):
        load_config(make_config({"model": {"min_detection_confidence": bad}}))


def test_core_mask_non_negative(make_config):
    with pytest.raises(ConfigError, match="core_mask"):
        load_config(make_config({"backend": {"core_mask": -1}}))


def test_core_mask_multi_core_ok(make_config):
    cfg = load_config(make_config({"backend": {"core_mask": 7}}))
    assert cfg["backend"]["core_mask"] == 7  # NPU_CORE_0_1_2


def test_empty_model_path_raises(make_config):
    with pytest.raises(ConfigError, match="detector_model"):
        load_config(make_config({"model": {"detector_model": ""}}))
    with pytest.raises(ConfigError, match="landmark_model"):
        load_config(make_config({"model": {"landmark_model": ""}}))


@pytest.mark.parametrize("mode", ["file", "stream", ""])
def test_bad_source_mode(make_config, mode):
    with pytest.raises(ConfigError, match="mode"):
        load_config(make_config({"source": {"mode": mode}}))


def test_image_mode_requires_rgb_path(make_config):
    with pytest.raises(ConfigError, match="rgb_path"):
        load_config(make_config({"source": {"mode": "image", "rgb_path": ""}}))


def test_ros_mode_ok_without_rgb_path(make_config):
    cfg = load_config(make_config({"source": {"mode": "ros", "rgb_path": ""}}))
    assert cfg["source"]["mode"] == "ros"


def test_camera_mode_ok_without_rgb_path(make_config):
    cfg = load_config(make_config({"source": {"mode": "camera", "rgb_path": ""}}))
    assert cfg["source"]["mode"] == "camera"


@pytest.mark.parametrize("port", [0, -1, 70000])
def test_port_range(make_config, port):
    with pytest.raises(ConfigError, match="port"):
        load_config(make_config({"server": {"port": port}}))


def test_yaml_content_preserved(make_config):
    cfg_path = make_config({"model": {"num_faces": 1}})
    raw = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
    assert raw["model"]["num_faces"] == 1
