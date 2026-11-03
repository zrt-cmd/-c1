"""实验 01 配置加载与校验。

契约（TDD 断言目标）：
- load_config(path) -> dict：YAML 加载 + schema 校验；缺键/类型错误抛 ConfigError。
- 未知键忽略（向前兼容）。
"""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

REQUIRED = {
    "dataset": {"root": (str,), "data_yaml": (str,), "splits": (list,)},
    "train": {
        "fork": (str,),
        "model": (str,),
        "imgsz": (int,),
        "epochs": (int,),
        "batch": (int,),
        "device": (int, str),
        "project": (str,),
        "name": (str,),
    },
    "infer": {"imgsz": (int,), "score_thr": (float, int), "iou_thr": (float, int), "pad_color": (list,)},
    "rknn": {
        "target_platform": (str,),
        "quantize": (bool,),
        "calib_count": (int,),
        "onnx_model": (str,),
        "rknn_model": (str,),
    },
    "eval": {"split": (str,), "iou_thr": (float, int)},
}


class ConfigError(Exception):
    """配置缺失或类型错误。"""


def load_config(path: str | Path) -> dict:
    """加载并校验配置；失败抛 ConfigError。"""
    p = Path(path)
    if not p.exists():
        raise ConfigError(f"config not found: {p}")
    try:
        raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {p}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError(f"config root must be a mapping, got {type(raw).__name__}")

    missing = [f"{k}.{sk}" for k, sub in REQUIRED.items() for sk, types in sub.items()
               if k not in raw or sk not in raw[k]]
    if missing:
        raise ConfigError(f"missing required keys: {', '.join(missing)}")

    for k, sub in REQUIRED.items():
        for sk, types in sub.items():
            val = raw[k][sk]
            if not isinstance(val, types):
                raise ConfigError(f"key {k}.{sk}: expected {'|'.join(t.__name__ for t in types)}, got {type(val).__name__}")
    return raw


def resolve_path(cfg: dict, key: str) -> Path:
    """取配置中的路径键并解析为绝对路径。"""
    parts = key.split(".")
    node = cfg
    for p in parts:
        node = node[p]
    return Path(node).expanduser().resolve()
