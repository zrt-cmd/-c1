"""实验 07 配置加载与校验（多目标跟踪参数）。"""
from __future__ import annotations

from pathlib import Path

import yaml

REQUIRED = {
    "model": {"det_rknn": (str,), "det_onnx": (str,), "reid_rknn": (str,),
              "imgsz": (int,)},
    "detect": {"score_thr": (float, int), "iou_thr": (float, int)},
    "tracker": {"mode": (str,), "max_dist": (float, int),
                "max_iou_distance": (float, int), "max_age": (int,),
                "n_init": (int,), "nn_budget": (int,),
                "min_confidence": (float, int)},
    "pool": {"workers": (int,), "queue_size": (int,), "core_mask": (str,)},
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

    if raw["tracker"]["mode"] not in ("sort", "deepsort"):
        raise ConfigError(f"tracker.mode must be sort|deepsort, got {raw['tracker']['mode']!r}")
    if raw["tracker"]["max_age"] <= 0 or raw["tracker"]["n_init"] <= 0:
        raise ConfigError("tracker.max_age/n_init must be positive")
    if raw["tracker"]["nn_budget"] < 0:
        raise ConfigError("tracker.nn_budget must be >= 0")
    cm = raw["pool"]["core_mask"]
    if cm not in ("auto", "round_robin"):
        raise ConfigError(f"pool.core_mask must be auto|round_robin, got {cm!r}")
    return raw
