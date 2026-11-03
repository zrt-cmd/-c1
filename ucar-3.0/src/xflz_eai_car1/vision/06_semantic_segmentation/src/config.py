"""实验 06 配置加载与校验（语义分割背景替换参数）。"""
from __future__ import annotations

from pathlib import Path

import yaml

REQUIRED = {
    "model": {"rknn": (str,), "onnx": (str,), "imgsz": (int,),
              "score_thr": (float, int), "iou_thr": (float, int),
              "mask_thr": (float, int), "classes": (list,)},
    "pool": {"workers": (int,), "queue_size": (int,), "core_mask": (str,)},
    "background": {"mode": (str,), "color": (list,), "image": (str,)},
    "camera": {"index": (int,), "width": (int,), "height": (int,)},
    "display": {"show": (bool,), "window_name": (str,)},
    "server": {"port": (int,), "stream_name": (str,)},
}


class ConfigError(Exception):
    """配置缺失、类型错误或内容不合法。"""


def load_config(path: str | Path) -> dict:
    p = Path(path)
    if not p.exists():
        raise ConfigError(f"config not found: {p}")
    try:
        raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {p}: {exc}") from exc
    if not isinstance(raw, dict):
        raise ConfigError(f"config root must be a mapping, got {type(raw).__name__}")

    missing = [f"{k}.{sk}" for k, sub in REQUIRED.items() for sk, _ in sub.items()
               if k not in raw or sk not in raw[k]]
    if missing:
        raise ConfigError(f"missing required keys: {', '.join(missing)}")
    for k, sub in REQUIRED.items():
        for sk, types in sub.items():
            val = raw[k][sk]
            if not isinstance(val, types):
                raise ConfigError(f"key {k}.{sk}: expected {'|'.join(t.__name__ for t in types)}, got {type(val).__name__}")
    if raw["background"]["mode"] not in ("black", "solid", "image"):
        raise ConfigError(f"background.mode must be black|solid|image, got {raw['background']['mode']!r}")
    if raw["background"]["mode"] == "solid" and len(raw["background"]["color"]) != 3:
        raise ConfigError("background.color must be [B, G, R] when mode=solid")
    if raw["background"]["mode"] == "image" and not raw["background"]["image"]:
        raise ConfigError("background.image must be set when mode=image")
    cm = raw["pool"]["core_mask"]
    if cm not in ("auto", "round_robin"):
        raise ConfigError(f"pool.core_mask must be auto|round_robin, got {cm!r}")
    return raw
