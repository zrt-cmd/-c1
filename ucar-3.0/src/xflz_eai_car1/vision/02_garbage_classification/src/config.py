"""实验 02 配置加载与校验（含 20 细类 / 4 大类映射 / YOLOv5 anchors）。

契约（TDD 断言目标）：
- load_config(path) -> dict：schema 校验（缺键/类型错/坏 YAML -> ConfigError）；
  路径键相对配置文件所在目录解析
- load_labels(path) -> list[str]：每行一类，跳过 # 注释与空行；空文件 -> ConfigError
- load_category_map(path, n_labels) -> list[int]：行数=n_labels、值∈[1,4]
- load_anchors(path) -> list[tuple[float,float]]：9 组 (w,h)
- category_of(class_id, cat_map) -> int；越界 -> ValueError
- category_name(cat, names) -> str
"""
from __future__ import annotations

from pathlib import Path

import yaml

REQUIRED = {
    "model": {"path": (str,), "labels": (str,), "category_map": (str,), "category_names": (dict,)},
    "infer": {"imgsz": (int,), "preprocess": (str,), "score_thr": (float, int), "nms_thr": (float, int), "anchors": (str,), "sigmoid": (bool,)},
    "pool": {"tpes": (int,), "core_mask": (str,)},
    "server": {"host": (str,), "port": (int,), "stream_name": (str,), "quality": (int,), "fps": (int,)},
}

CATEGORY_IDS = ("1", "2", "3", "4")  # 4 大类固定 ID


class ConfigError(Exception):
    """配置缺失、类型错误或内容不合法。"""


def load_config(path: str | Path) -> dict:
    """加载并校验配置；路径键解析为相对配置文件目录的绝对路径。"""
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

    names = raw["model"]["category_names"]
    if set(names) != set(CATEGORY_IDS):
        raise ConfigError(f"category_names must cover {CATEGORY_IDS}, got {sorted(names)}")

    # 路径键相对配置文件所在目录解析（任意 CWD 可用）
    base = p.parent
    for key in ("path", "labels", "category_map"):
        val = raw["model"][key]
        if not Path(val).is_absolute():
            raw["model"][key] = str((base / val).resolve())
    anchors = raw["infer"]["anchors"]
    if not Path(anchors).is_absolute():
        raw["infer"]["anchors"] = str((base / anchors).resolve())
    return raw


def _data_lines(path: Path) -> list[str]:
    """读取数据文件行：去空行、跳过 # 注释。"""
    if not path.exists():
        raise ConfigError(f"file not found: {path}")
    return [ln.strip() for ln in path.read_text(encoding="utf-8").splitlines()
            if ln.strip() and not ln.lstrip().startswith("#")]


def load_labels(path: str | Path) -> list[str]:
    """读取细类标签（每行一个）。"""
    lines = _data_lines(Path(path))
    if not lines:
        raise ConfigError(f"labels file empty: {path}")
    return lines


def load_category_map(path: str | Path, n_labels: int) -> list[int]:
    """读取细类 -> 大类映射（每行 1-4）。"""
    values = []
    for ln in _data_lines(Path(path)):
        try:
            v = int(ln)
        except ValueError as exc:
            raise ConfigError(f"category_map bad value {ln!r} in {path}") from exc
        if v not in (1, 2, 3, 4):
            raise ConfigError(f"category_map value {v} not in 1..4 ({path})")
        values.append(v)
    if len(values) != n_labels:
        raise ConfigError(f"category_map rows({len(values)}) != labels({n_labels})")
    return values


def load_anchors(path: str | Path) -> list[tuple[float, float]]:
    """读取 anchors（每行 w,h；yolov5 为 9 组 = 3 尺度 × 3 锚）。"""
    anchors: list[tuple[float, float]] = []
    for ln in _data_lines(Path(path)):
        parts = ln.replace(" ", "").split(",")
        if len(parts) != 2:
            raise ConfigError(f"anchors bad row {ln!r} in {path}")
        try:
            anchors.append((float(parts[0]), float(parts[1])))
        except ValueError as exc:
            raise ConfigError(f"anchors bad row {ln!r} in {path}") from exc
    if len(anchors) != 9:
        raise ConfigError(f"anchors rows({len(anchors)}) != 9 (3 scales × 3 anchors)")
    return anchors


def category_of(class_id: int, cat_map: list[int]) -> int:
    """细类 -> 大类（1-4）。class_id 越界抛 ValueError。"""
    if not 0 <= class_id < len(cat_map):
        raise ValueError(f"class_id {class_id} out of range [0, {len(cat_map)})")
    return cat_map[class_id]


def category_name(cat: int, names: dict[str, str]) -> str:
    """大类 ID -> 名称。"""
    key = str(cat)
    if key not in names:
        raise ValueError(f"category {cat} not in category_names {sorted(names)}")
    return names[key]
