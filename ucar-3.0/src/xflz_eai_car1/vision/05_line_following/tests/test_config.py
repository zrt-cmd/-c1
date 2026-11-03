"""config 模块 TDD 测试（构建文档 01 的 6 条计划）。"""
import pytest

from config import ConfigError, load_config

VALID_YAML = (
    "preprocess:\n  threshold_mode: binary\n  threshold_value: 127\n"
    "  invert: true\n  morph_kernel: 5\n"
    "contour:\n  min_area: 2000\n"
    "polygon:\n  approx_epsilon: 0.02\n  use_hull: true\n"
    "camera:\n  index: 0\n  width: 640\n  height: 480\n"
    "display:\n  show: false\n  window_name: polygon_contour\n"
)


@pytest.fixture
def valid_cfg(tmp_path):
    p = tmp_path / "line_config.yaml"
    p.write_text(VALID_YAML, encoding="utf-8")
    return p


def test_load_valid_config(valid_cfg):
    cfg = load_config(valid_cfg)
    assert cfg["preprocess"]["threshold_mode"] == "binary"
    assert cfg["polygon"]["use_hull"] is True
    assert cfg["contour"]["min_area"] == 2000


def test_missing_section_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text(VALID_YAML.replace("contour:", "xxx:"), encoding="utf-8")
    with pytest.raises(ConfigError, match="contour.min_area"):
        load_config(p)


def test_wrong_type_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad_type.yaml"
    p.write_text(VALID_YAML.replace("min_area: 2000", "min_area: many"), encoding="utf-8")
    with pytest.raises(ConfigError, match="contour.min_area"):
        load_config(p)


def test_invalid_yaml_raises(tmp_path):
    p = tmp_path / "bad_yaml.yaml"
    p.write_text("key: [1, 2\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(p)


def test_bad_threshold_mode(valid_cfg, tmp_path):
    p = tmp_path / "bad_mode.yaml"
    p.write_text(VALID_YAML.replace("binary", "otsu"), encoding="utf-8")
    with pytest.raises(ConfigError, match="threshold_mode"):
        load_config(p)


def test_even_morph_kernel_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad_kernel.yaml"
    p.write_text(VALID_YAML.replace("morph_kernel: 5", "morph_kernel: 4"), encoding="utf-8")
    with pytest.raises(ConfigError, match="odd"):
        load_config(p)
