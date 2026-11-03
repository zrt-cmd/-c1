"""config 模块 TDD 测试（构建文档 01 的 7 条计划）。"""
import pytest

import qrcode

from config import ConfigError, error_correction_level, load_config

VALID_YAML = (
    "generate:\n"
    "  version: 1\n  error_correction: H\n  box_size: 5\n  border: 4\n"
    "  fill_color: [0, 0, 0]\n  back_color: [255, 255, 255]\n"
    "detect:\n  scale: 0.5\n"
    "camera:\n  index: 0\n  width: 640\n  height: 480\n"
    "display:\n  show: false\n  window_name: qrcode\n"
)


@pytest.fixture
def valid_cfg(tmp_path):
    p = tmp_path / "qrcode_config.yaml"
    p.write_text(VALID_YAML, encoding="utf-8")
    return p


def test_load_valid_config(valid_cfg):
    cfg = load_config(valid_cfg)
    assert cfg["generate"]["version"] == 1
    assert cfg["generate"]["error_correction"] == "H"
    assert cfg["detect"]["scale"] == 0.5


def test_missing_section_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text(VALID_YAML.replace("detect:", "xxx:"), encoding="utf-8")
    with pytest.raises(ConfigError, match="detect.scale"):
        load_config(p)


def test_wrong_type_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad_type.yaml"
    p.write_text(VALID_YAML.replace("box_size: 5", "box_size: big"), encoding="utf-8")
    with pytest.raises(ConfigError, match="generate.box_size"):
        load_config(p)


def test_invalid_yaml_raises(tmp_path):
    p = tmp_path / "bad_yaml.yaml"
    p.write_text("key: [1, 2\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(p)


def test_error_correction_levels():
    assert error_correction_level("L") == qrcode.constants.ERROR_CORRECT_L
    assert error_correction_level("M") == qrcode.constants.ERROR_CORRECT_M
    assert error_correction_level("Q") == qrcode.constants.ERROR_CORRECT_Q
    assert error_correction_level("H") == qrcode.constants.ERROR_CORRECT_H
    with pytest.raises(ConfigError, match="error_correction"):
        error_correction_level("Z")


def test_version_out_of_range(valid_cfg, tmp_path):
    p = tmp_path / "bad_version.yaml"
    p.write_text(VALID_YAML.replace("version: 1", "version: 41"), encoding="utf-8")
    with pytest.raises(ConfigError, match="version"):
        load_config(p)


def test_color_out_of_range(valid_cfg, tmp_path):
    p = tmp_path / "bad_color.yaml"
    p.write_text(VALID_YAML.replace("fill_color: [0, 0, 0]", "fill_color: [0, 0, 300]"),
                 encoding="utf-8")
    with pytest.raises(ConfigError, match="out of \\[0,255\\]"):
        load_config(p)
