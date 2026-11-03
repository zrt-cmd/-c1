"""实验 05 配置加载与校验（多边形轮廓应用参数）。"""
from __future__ import annotations

from pathlib import Path

import yaml

REQUIRED = {
    "preprocess": {"threshold_mode": (str,), "threshold_value": (int,),
                   "invert": (bool,), "morph_kernel": (int,)},
    "contour": {"min_area": (int,)},
    "polygon": {"approx_epsilon": (float, int), "use_hull": (bool,)},
    "camera": {"index": (int,), "width": (int,), "height": (int,)},
    "display": {"show": (bool,), "window_name": (str,)},
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

    mode = raw["preprocess"]["threshold_mode"]
    if mode not in ("binary", "adaptive"):
        raise ConfigError(f"threshold_mode must be binary|adaptive, got {mode!r}")
    if raw["preprocess"]["morph_kernel"] % 2 == 0:
        raise ConfigError(f"morph_kernel must be odd, got {raw['preprocess']['morph_kernel']}")
    return raw
