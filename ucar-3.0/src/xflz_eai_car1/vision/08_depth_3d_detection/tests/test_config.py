"""config 模块 TDD 测试（构建文档 01 的 8 条计划）。"""
import pytest

from config import ConfigError, load_config

VALID_YAML = (
    "depth:\n  scale: 0.001\n  invalid_value: 0\n  colormap: jet\n"
    "  display_scale: 0.666\n"
    "cam3d:\n  fx: 608.39958\n  fy: 611.20714\n  cx: 328.30096\n  cy: 226.46483\n"
    "  width: 640\n  height: 480\n"
    "source:\n  mode: image\n  rgb_path: a.png\n  depth_path: b.png\n"
    "display:\n  show: false\n  window_name: \"RGB | Depth\"\n"
    "server:\n  port: 8080\n  stream_name: depth.mjpg\n"
)


@pytest.fixture
def valid_cfg(tmp_path):
    p = tmp_path / "depth3d_config.yaml"
    p.write_text(VALID_YAML, encoding="utf-8")
    return p


def test_load_valid_config(valid_cfg):
    cfg = load_config(valid_cfg)
    assert cfg["depth"]["colormap"] == "jet"
    assert cfg["cam3d"]["fx"] == pytest.approx(608.39958)
    assert cfg["source"]["mode"] == "image"


def test_missing_section_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text(VALID_YAML.replace("server:", "xxx:"), encoding="utf-8")
    with pytest.raises(ConfigError, match="server.port"):
        load_config(p)


def test_wrong_type_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad_type.yaml"
    p.write_text(VALID_YAML.replace("scale: 0.001", "scale: tiny"), encoding="utf-8")
    with pytest.raises(ConfigError, match="depth.scale"):
        load_config(p)


def test_invalid_yaml_raises(tmp_path):
    p = tmp_path / "bad_yaml.yaml"
    p.write_text("key: [1, 2\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(p)


def test_bad_colormap(valid_cfg, tmp_path):
    p = tmp_path / "bad_cmap.yaml"
    p.write_text(VALID_YAML.replace("colormap: jet", "colormap: rainbow"), encoding="utf-8")
    with pytest.raises(ConfigError, match="colormap"):
        load_config(p)


def test_image_mode_requires_paths(valid_cfg, tmp_path):
    p = tmp_path / "bad_src.yaml"
    p.write_text(VALID_YAML.replace("rgb_path: a.png", "rgb_path: \"\""), encoding="utf-8")
    with pytest.raises(ConfigError, match="rgb_path"):
        load_config(p)


def test_nonpositive_fx(valid_cfg, tmp_path):
    p = tmp_path / "bad_fx.yaml"
    p.write_text(VALID_YAML.replace("fx: 608.39958", "fx: 0"), encoding="utf-8")
    with pytest.raises(ConfigError, match="cam3d.fx/fy"):
        load_config(p)


def test_nonpositive_display_scale(valid_cfg, tmp_path):
    p = tmp_path / "bad_scale.yaml"
    p.write_text(VALID_YAML.replace("display_scale: 0.666", "display_scale: 0"), encoding="utf-8")
    with pytest.raises(ConfigError, match="display_scale"):
        load_config(p)
