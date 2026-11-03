"""实验 04 配置加载与校验（二维码生成/检测参数）。

契约（TDD 断言目标）：
- load_config(path) -> dict：schema 校验；路径键相对配置文件目录解析
- error_correction_level(name) -> int：L/M/Q/H 映射 qrcode.constants，非法 -> ConfigError
- 生成参数校验：version∈[1,40]、box_size≥1、border≥0、颜色 3 元 RGB∈[0,255]
"""
from __future__ import annotations

from pathlib import Path

import yaml

import qrcode

REQUIRED = {
    "generate": {
        "version": (int,), "error_correction": (str,), "box_size": (int,),
        "border": (int,), "fill_color": (list,), "back_color": (list,),
    },
    "detect": {"scale": (float, int)},
    "camera": {"index": (int,), "width": (int,), "height": (int,)},
    "display": {"show": (bool,), "window_name": (str,)},
}

EC_LEVELS = {
    "L": qrcode.constants.ERROR_CORRECT_L,
    "M": qrcode.constants.ERROR_CORRECT_M,
    "Q": qrcode.constants.ERROR_CORRECT_Q,
    "H": qrcode.constants.ERROR_CORRECT_H,
}


class ConfigError(Exception):
    """配置缺失、类型错误或内容不合法。"""


def error_correction_level(name: str) -> int:
    if name not in EC_LEVELS:
        raise ConfigError(f"error_correction must be one of {sorted(EC_LEVELS)}, got {name!r}")
    return EC_LEVELS[name]


def _check_color(name: str, val) -> tuple[int, int, int]:
    if not isinstance(val, (list, tuple)) or len(val) != 3:
        raise ConfigError(f"{name} must be [r, g, b]")
    rgb = tuple(int(v) for v in val)
    if not all(0 <= v <= 255 for v in rgb):
        raise ConfigError(f"{name} values out of [0,255]: {val}")
    return rgb


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

    g = raw["generate"]
    if not 1 <= g["version"] <= 40:
        raise ConfigError(f"version must be 1..40, got {g['version']}")
    if g["box_size"] < 1:
        raise ConfigError(f"box_size must be >= 1, got {g['box_size']}")
    if g["border"] < 0:
        raise ConfigError(f"border must be >= 0, got {g['border']}")
    error_correction_level(g["error_correction"])
    g["fill_color"] = _check_color("fill_color", g["fill_color"])
    g["back_color"] = _check_color("back_color", g["back_color"])
    return raw
