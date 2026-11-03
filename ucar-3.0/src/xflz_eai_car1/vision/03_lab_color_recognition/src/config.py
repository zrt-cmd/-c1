"""实验 03 配置加载与校验（HSV 颜色阈值/形态学/轮廓/相机）。

对应课程：OpenCV 应用——RGB 颜色过滤 + HSV 颜色追踪（vendor 03_lab/03_颜色追踪）。
注意：课程目录名 "lab" = 实验（实验 03），不是 Lab 色彩空间——颜色过滤用 **HSV**。

契约（TDD 断言目标）：
- load_config(path) -> dict：schema 校验
- ColorSpec(name, h, s, v, h2=None)：每通道 [min, max]；h2 = H 环形第二段（红色跨 0/180）
- load_colors(cfg) -> list[ColorSpec]：≥1 色；min<=max；H∈[0,179]，S/V∈[0,255]
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

REQUIRED = {
    "morph": {"open_kernel": (int,), "close_kernel": (int,), "open_iter": (int,), "close_iter": (int,)},
    "contour": {"min_area": (int, float), "min_radius": (int, float), "max_radius": (int, float)},
    "camera": {"index": (int,), "width": (int,), "height": (int,)},
    "display": {"show": (bool,), "window_name": (str,)},
}


class ConfigError(Exception):
    """配置缺失、类型错误或内容不合法。"""


@dataclass(frozen=True)
class ColorSpec:
    name: str
    h: tuple[int, int]
    s: tuple[int, int]
    v: tuple[int, int]
    h2: tuple[int, int] | None = None  # H 环形第二段（红色 0/180 交界）


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
    if "colors" not in raw or not isinstance(raw["colors"], list) or not raw["colors"]:
        raise ConfigError("config must contain non-empty 'colors' list")

    missing = [f"{k}.{sk}" for k, sub in REQUIRED.items() for sk, _ in sub.items()
               if k not in raw or sk not in raw[k]]
    if missing:
        raise ConfigError(f"missing required keys: {', '.join(missing)}")
    for k, sub in REQUIRED.items():
        for sk, types in sub.items():
            val = raw[k][sk]
            if not isinstance(val, types):
                raise ConfigError(f"key {k}.{sk}: expected {'|'.join(t.__name__ for t in types)}, got {type(val).__name__}")
    return raw


def _parse_channel(name: str, ch: str, val, max_val: int | None = None) -> tuple[int, int]:
    if not isinstance(val, (list, tuple)) or len(val) != 2:
        raise ConfigError(f"color {name}: {ch} range must be [min, max]")
    lo, hi = val
    if not all(isinstance(v, (int, float)) for v in (lo, hi)):
        raise ConfigError(f"color {name}: {ch} bounds must be numbers")
    lo, hi = int(lo), int(hi)
    if lo > hi:
        raise ConfigError(f"color {name}: {ch} min({lo}) > max({hi})")
    if max_val is None:
        max_val = 180 if ch == "h2" else (179 if ch == "h" else 255)
    if not (0 <= lo <= max_val and 0 <= hi <= max_val):
        raise ConfigError(f"color {name}: {ch} bounds out of [0,{max_val}]")
    return lo, hi


def load_colors(cfg: dict) -> list[ColorSpec]:
    specs = []
    for i, c in enumerate(cfg["colors"]):
        if not isinstance(c, dict) or "name" not in c or not isinstance(c["name"], str):
            raise ConfigError(f"colors[{i}]: need dict with str 'name'")
        for ch in ("h", "s", "v"):
            if ch not in c:
                raise ConfigError(f"color {c['name']!r}: missing channel '{ch}'")
        h2 = _parse_channel(c["name"], "h2", c["h2"]) if c.get("h2") is not None else None
        specs.append(ColorSpec(
            name=c["name"],
            h=_parse_channel(c["name"], "h", c["h"]),
            s=_parse_channel(c["name"], "s", c["s"]),
            v=_parse_channel(c["name"], "v", c["v"]),
            h2=h2,
        ))
    return specs
