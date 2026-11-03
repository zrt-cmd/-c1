"""实验 02 共享测试夹具：src 导入路径、样例配置与数据文件。"""
import sys
from pathlib import Path

import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC))

LABELS_20 = [
    "bailuobo", "caiye", "chuangkotie", "cipian", "dianchi", "eluanshi",
    "fanqie", "huluobo", "jiaonang", "lajiao", "shuiping", "tizi", "tudo",
    "yaogao", "yaopian", "yaopianbaozhuang", "yaoping", "yilaguan",
    "zhibei", "zhuankuai",
]
CAT_MAP_20 = [2, 2, 3, 4, 3, 4, 2, 2, 3, 2, 1, 2, 2, 3, 3, 3, 3, 1, 1, 4]
ANCHORS_9 = [(10, 13), (16, 30), (33, 23), (30, 61), (62, 45),
             (59, 119), (116, 90), (156, 198), (373, 326)]
CAT_NAMES = {"1": "recyclable", "2": "kitchen_waste",
             "3": "hazardous_waste", "4": "other_waste"}


@pytest.fixture
def labels_file(tmp_path) -> Path:
    p = tmp_path / "labels.txt"
    p.write_text("\n".join(LABELS_20) + "\n", encoding="utf-8")
    return p


@pytest.fixture
def cat_map_file(tmp_path) -> Path:
    p = tmp_path / "category_map.txt"
    p.write_text("\n".join(str(v) for v in CAT_MAP_20) + "\n", encoding="utf-8")
    return p


@pytest.fixture
def anchors_file(tmp_path) -> Path:
    p = tmp_path / "anchors.txt"
    p.write_text("\n".join(f"{w},{h}" for w, h in ANCHORS_9) + "\n", encoding="utf-8")
    return p


@pytest.fixture
def valid_config(tmp_path, labels_file, cat_map_file, anchors_file) -> Path:
    p = tmp_path / "garbage_config.yaml"
    p.write_text(
        f"model:\n  path: rubbish_2_7.rknn\n  labels: {labels_file}\n"
        f"  category_map: {cat_map_file}\n  category_names:\n"
        "    \"1\": recyclable\n    \"2\": kitchen_waste\n"
        "    \"3\": hazardous_waste\n    \"4\": other_waste\n"
        f"infer:\n  imgsz: 640\n  preprocess: letterbox\n  score_thr: 0.25\n"
        f"  nms_thr: 0.45\n  sigmoid: true\n  anchors: {anchors_file}\n"
        "pool:\n  tpes: 6\n  core_mask: round_robin\n"
        "server:\n  host: 0.0.0.0\n  port: 8080\n  stream_name: garbage_cam\n"
        "  quality: 80\n  fps: 30\n",
        encoding="utf-8")
    return p
