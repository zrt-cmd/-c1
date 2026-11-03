"""实验 09 配置加载与校验（人脸关键点参数，RKNN Blaze 双模型 + RKNNLite）。"""
from __future__ import annotations

from pathlib import Path

import yaml

REQUIRED = {
    "model": {"detector_model": (str,), "landmark_model": (str,),
              "num_faces": (int,),
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

    if raw["model"]["num_faces"] <= 0:
        raise ConfigError(f"model.num_faces must be > 0, got {raw['model']['num_faces']}")
    for key in ("min_detection_confidence", "min_tracking_confidence"):
        if not 0 < raw["model"][key] <= 1:
            raise ConfigError(f"model.{key} must be in (0, 1], got {raw['model'][key]}")
    if not raw["model"]["detector_model"] or not raw["model"]["landmark_model"]:
        raise ConfigError("model.detector_model/landmark_model must not be empty")
    if raw["backend"]["core_mask"] < 0:
        raise ConfigError(f"backend.core_mask must be >= 0, got {raw['backend']['core_mask']}")
    if raw["source"]["mode"] not in ("image", "ros", "camera"):
        raise ConfigError(f"source.mode must be image|ros|camera, got {raw['source']['mode']!r}")
    if raw["source"]["mode"] == "image" and not raw["source"]["rgb_path"]:
        raise ConfigError("source.rgb_path required when mode=image")
    if raw["server"]["port"] <= 0 or raw["server"]["port"] > 65535:
        raise ConfigError(f"server.port out of range, got {raw['server']['port']}")
    return raw
