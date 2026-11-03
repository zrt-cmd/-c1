"""config 模块 TDD 测试（构建文档 01 的 8 条计划；HSV 契约）。"""
import pytest

from config import ColorSpec, ConfigError, load_colors, load_config


VALID_YAML = (
    "colors:\n"
    "  - name: 红色\n    h: [0, 15]\n    s: [100, 255]\n    v: [80, 255]\n    h2: [160, 180]\n"
    "  - name: 蓝色\n    h: [90, 130]\n    s: [80, 255]\n    v: [80, 255]\n"
    "morph:\n  open_kernel: 5\n  close_kernel: 9\n  open_iter: 1\n  close_iter: 1\n"
    "contour:\n  min_area: 500\n  min_radius: 10\n  max_radius: 400\n"
    "camera:\n  index: 0\n  width: 640\n  height: 480\n"
    "display:\n  show: false\n  window_name: lab_color\n"
)


@pytest.fixture
def valid_cfg(tmp_path):
    p = tmp_path / "lab_config.yaml"
    p.write_text(VALID_YAML, encoding="utf-8")
    return p


def test_load_valid_config(valid_cfg):
    cfg = load_config(valid_cfg)
    assert cfg["morph"]["open_kernel"] == 5
    assert cfg["camera"]["index"] == 0


def test_missing_section_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text(VALID_YAML.replace("contour:", "xxx:"), encoding="utf-8")
    with pytest.raises(ConfigError, match="contour.min_area"):
        load_config(p)


def test_wrong_type_raises(valid_cfg, tmp_path):
    p = tmp_path / "bad_type.yaml"
    p.write_text(VALID_YAML.replace("open_kernel: 5", "open_kernel: big"), encoding="utf-8")
    with pytest.raises(ConfigError, match="morph.open_kernel"):
        load_config(p)


def test_invalid_yaml_raises(tmp_path):
    p = tmp_path / "bad_yaml.yaml"
    p.write_text("key: [1, 2\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(p)


def test_empty_colors_raises(tmp_path):
    p = tmp_path / "no_colors.yaml"
    p.write_text(
        "colors: []\n"
        "morph:\n  open_kernel: 5\n  close_kernel: 9\n  open_iter: 1\n  close_iter: 1\n"
        "contour:\n  min_area: 500\n  min_radius: 10\n  max_radius: 400\n"
        "camera:\n  index: 0\n  width: 640\n  height: 480\n"
        "display:\n  show: false\n  window_name: lab_color\n",
        encoding="utf-8")
    with pytest.raises(ConfigError, match="colors"):
        load_config(p)


def test_load_colors(valid_cfg):
    colors = load_colors(load_config(valid_cfg))
    assert len(colors) == 2
    assert colors[0].name == "红色"
    assert colors[0].s == (100, 255)
    assert colors[0].h2 == (160, 180)  # 红色 H 环形第二段
    assert colors[1].h2 is None
    assert isinstance(colors[0], ColorSpec)


def test_channel_min_gt_max_raises(tmp_path):
    p = tmp_path / "bad_range.yaml"
    p.write_text(VALID_YAML.replace("s: [100, 255]", "s: [255, 100]"), encoding="utf-8")
    with pytest.raises(ConfigError, match="min.*max"):
        load_colors(load_config(p))


def test_channel_out_of_range_raises(tmp_path):
    p = tmp_path / "bad_bounds.yaml"
    p.write_text(VALID_YAML.replace("v: [80, 255]", "v: [80, 300]"), encoding="utf-8")
    with pytest.raises(ConfigError, match="out of \\[0,255\\]"):
        load_colors(load_config(p))


def test_h_out_of_range_raises(tmp_path):
    # H 通道上限 179（OpenCV HSV），非 255
    p = tmp_path / "bad_h.yaml"
    p.write_text(VALID_YAML.replace("h: [0, 15]", "h: [0, 200]"), encoding="utf-8")
    with pytest.raises(ConfigError, match="out of \\[0,179\\]"):
        load_colors(load_config(p))


def test_missing_channel_raises(tmp_path):
    p = tmp_path / "no_channel.yaml"
    p.write_text(VALID_YAML.replace("    v: [80, 255]\n", ""), encoding="utf-8")
    with pytest.raises(ConfigError, match="missing channel"):
        load_colors(load_config(p))
