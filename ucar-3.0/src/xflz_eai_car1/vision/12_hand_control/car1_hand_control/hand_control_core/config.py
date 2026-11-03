"""实验 12 配置加载与校验（手部参数，RKNN 双模型 palm+hand + RKNNLite）。"""
from __future__ import annotations

from pathlib import Path

import yaml

REQUIRED = {
    "model": {"detector_model": (str,), "landmark_model": (str,),
              "embedder_model": (str,), "classifier_model": (str,),
              "max_hands": (int,),
              "min_detection_confidence": (float, int),
              "min_tracking_confidence": (float, int)},
    "backend": {"core_mask": (int,), "verbose": (bool,)},
    "source": {"mode": (str,), "rgb_path": (str,)},
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

    if raw["model"]["max_hands"] <= 0 or raw["model"]["max_hands"] > 2:
        raise ConfigError(f"model.max_hands must be 1..2, got {raw['model']['max_hands']}")
    for key in ("min_detection_confidence", "min_tracking_confidence"):
        if not 0 < raw["model"][key] <= 1:
            raise ConfigError(
                f"model.{key} must be in (0, 1], got {raw['model'][key]}")
    for key in ("detector_model", "landmark_model", "embedder_model",
                "classifier_model"):
        if not raw["model"][key]:
            raise ConfigError(f"model.{key} must not be empty")
    if raw["backend"]["core_mask"] < 0:
        raise ConfigError(f"backend.core_mask must be >= 0, got {raw['backend']['core_mask']}")
    if raw["source"]["mode"] not in ("image", "ros", "camera"):
        raise ConfigError(f"source.mode must be image|ros|camera, got {raw['source']['mode']!r}")
    if raw["source"]["mode"] == "image" and not raw["source"]["rgb_path"]:
        raise ConfigError("source.rgb_path required when mode=image")
    if raw["server"]["port"] <= 0 or raw["server"]["port"] > 65535:
        raise ConfigError(f"server.port out of range, got {raw['server']['port']}")
    return raw
