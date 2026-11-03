"""config 模块 TDD 测试。"""
import pytest

from config import ConfigError, load_config


def test_load_valid_config(valid_config):
    cfg = load_config(valid_config)
    assert cfg["train"]["imgsz"] == 640
    assert cfg["rknn"]["target_platform"] == "rk3576"


def test_missing_file_raises(tmp_path):
    with pytest.raises(ConfigError, match="not found"):
        load_config(tmp_path / "nope.yaml")


def test_missing_section_raises(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("dataset: {}\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="train.fork"):
        load_config(p)


def test_wrong_type_raises(valid_config, tmp_path):
    p = tmp_path / "bad_type.yaml"
    p.write_text(valid_config.read_text(encoding="utf-8").replace("imgsz: 640", "imgsz: big"),
                 encoding="utf-8")
    with pytest.raises(ConfigError, match="train.imgsz"):
        load_config(p)


def test_invalid_yaml_raises(tmp_path):
    p = tmp_path / "bad_yaml.yaml"
    p.write_text("key: [1, 2\n", encoding="utf-8")   # 未闭合括号 -> ScannerError
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(p)
