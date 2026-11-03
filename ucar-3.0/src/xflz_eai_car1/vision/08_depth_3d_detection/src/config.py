"""实验 08 配置加载与校验（深度查看器参数）。"""
from __future__ import annotations

from pathlib import Path

import yaml

REQUIRED = {
    "depth": {"scale": (float, int), "invalid_value": (int,),
              "colormap": (str,), "display_scale": (float, int)},
    "cam3d": {"fx": (float, int), "fy": (float, int),
              "cx": (float, int), "cy": (float, int),
              "width": (int,), "height": (int,)},
    "source": {"mode": (str,), "rgb_path": (str,), "depth_path": (str,)},
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

    if raw["depth"]["scale"] <= 0:
        raise ConfigError(f"depth.scale must be > 0, got {raw['depth']['scale']}")
    if raw["depth"]["display_scale"] <= 0:
        raise ConfigError(f"depth.display_scale must be > 0, got {raw['depth']['display_scale']}")
    if raw["depth"]["colormap"] not in ("jet", "gray"):
        raise ConfigError(f"depth.colormap must be jet|gray, got {raw['depth']['colormap']!r}")
    if raw["cam3d"]["fx"] <= 0 or raw["cam3d"]["fy"] <= 0:
        raise ConfigError("cam3d.fx/fy must be > 0")
    if raw["source"]["mode"] not in ("image", "ros"):
        raise ConfigError(f"source.mode must be image|ros, got {raw['source']['mode']!r}")
    if raw["source"]["mode"] == "image":
        if not raw["source"]["rgb_path"] or not raw["source"]["depth_path"]:
            raise ConfigError("source.rgb_path/depth_path required when mode=image")
    return raw
