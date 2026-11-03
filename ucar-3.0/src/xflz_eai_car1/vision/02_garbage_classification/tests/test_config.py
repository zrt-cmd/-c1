"""config 模块 TDD 测试（构建文档 01 的 12 条计划）。"""
import pytest

from config import (
    ConfigError,
    category_name,
    category_of,
    load_anchors,
    load_category_map,
    load_config,
    load_labels,
)
from conftest import ANCHORS_9, CAT_MAP_20, LABELS_20


def test_load_valid_config(valid_config):
    cfg = load_config(valid_config)
    assert cfg["infer"]["imgsz"] == 640
    assert cfg["model"]["category_names"]["1"] == "recyclable"
    assert cfg["pool"]["core_mask"] == "round_robin"


def test_missing_key_raises(valid_config, tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text(valid_config.read_text(encoding="utf-8").replace("imgsz: 640", ""),
                 encoding="utf-8")
    with pytest.raises(ConfigError, match="infer.imgsz"):
        load_config(p)


def test_wrong_type_raises(valid_config, tmp_path):
    p = tmp_path / "bad_type.yaml"
    p.write_text(valid_config.read_text(encoding="utf-8").replace("tpes: 6", "tpes: many"),
                 encoding="utf-8")
    with pytest.raises(ConfigError, match="pool.tpes"):
        load_config(p)


def test_invalid_yaml_raises(tmp_path):
    p = tmp_path / "bad_yaml.yaml"
    p.write_text("key: [1, 2\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="invalid YAML"):
        load_config(p)


def test_category_names_must_cover_1to4(valid_config, tmp_path):
    p = tmp_path / "bad_names.yaml"
    p.write_text(valid_config.read_text(encoding="utf-8")
                 .replace('"3": hazardous_waste\n', ''), encoding="utf-8")
    with pytest.raises(ConfigError, match="category_names"):
        load_config(p)


def test_load_labels(labels_file):
    assert load_labels(labels_file) == LABELS_20
    assert len(load_labels(labels_file)) == 20


def test_empty_labels_raises(tmp_path):
    p = tmp_path / "empty.txt"
    p.write_text("\n\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="empty"):
        load_labels(p)


def test_load_category_map(cat_map_file):
    assert load_category_map(cat_map_file, 20) == CAT_MAP_20


def test_category_map_wrong_count_raises(cat_map_file):
    with pytest.raises(ConfigError, match="!="):
        load_category_map(cat_map_file, 19)


def test_category_map_value_out_of_range(tmp_path):
    p = tmp_path / "bad_map.txt"
    p.write_text("1\n5\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="not in 1..4"):
        load_category_map(p, 2)


def test_load_anchors(anchors_file):
    assert load_anchors(anchors_file) == ANCHORS_9


def test_anchors_wrong_count_raises(tmp_path):
    p = tmp_path / "bad_anchors.txt"
    p.write_text("10,13\n16,30\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="!= 9"):
        load_anchors(p)


def test_anchors_bad_format_raises(tmp_path):
    p = tmp_path / "bad_anchors2.txt"
    p.write_text("10,13\n16,30\n33,23\n30,61\n62,45\n59,119\n116,90\n156,198\nabc\n",
                 encoding="utf-8")
    with pytest.raises(ConfigError, match="bad row"):
        load_anchors(p)


def test_category_of_mapping():
    cm = CAT_MAP_20
    assert category_of(10, cm) == 1   # shuiping -> recyclable
    assert category_of(0, cm) == 2    # bailuobo -> kitchen_waste
    assert category_of(4, cm) == 3    # dianchi -> hazardous_waste
    assert category_of(3, cm) == 4    # cipian -> other_waste
    assert category_of(19, cm) == 4   # zhuankuai -> other_waste


def test_category_of_out_of_range():
    with pytest.raises(ValueError, match="out of range"):
        category_of(20, CAT_MAP_20)
    with pytest.raises(ValueError, match="out of range"):
        category_of(-1, CAT_MAP_20)


def test_category_name():
    names = {"1": "recyclable", "2": "kitchen_waste",
             "3": "hazardous_waste", "4": "other_waste"}
    assert category_name(1, names) == "recyclable"
    assert category_name(4, names) == "other_waste"
    with pytest.raises(ValueError):
        category_name(5, names)


def test_relative_paths_resolved_against_config_dir(tmp_path, labels_file, cat_map_file,
                                                    anchors_file):
    """路径键相对配置文件目录解析（任意 CWD 可用）。"""
    cfg_file = tmp_path / "sub" / "cfg.yaml"
    cfg_file.parent.mkdir()
    cfg_file.write_text(
        "model:\n  path: model.rknn\n  labels: labels.txt\n  category_map: category_map.txt\n"
        "  category_names:\n    \"1\": a\n    \"2\": b\n    \"3\": c\n    \"4\": d\n"
        "infer:\n  imgsz: 32\n  preprocess: letterbox\n  score_thr: 0.25\n"
        "  nms_thr: 0.45\n  sigmoid: true\n  anchors: anchors.txt\n"
        "pool:\n  tpes: 1\n  core_mask: auto\n"
        "server:\n  host: 127.0.0.1\n  port: 0\n  stream_name: s\n  quality: 80\n  fps: 30\n",
        encoding="utf-8")
    (cfg_file.parent / "labels.txt").write_text("\n".join(LABELS_20), encoding="utf-8")
    (cfg_file.parent / "category_map.txt").write_text(
        "\n".join(str(v) for v in CAT_MAP_20), encoding="utf-8")
    (cfg_file.parent / "anchors.txt").write_text(
        "\n".join(f"{w},{h}" for w, h in ANCHORS_9), encoding="utf-8")
    cfg = load_config(cfg_file)
    assert cfg["model"]["labels"] == str((cfg_file.parent / "labels.txt").resolve())
    assert cfg["model"]["path"] == str((cfg_file.parent / "model.rknn").resolve())
    assert cfg["infer"]["anchors"] == str((cfg_file.parent / "anchors.txt").resolve())
