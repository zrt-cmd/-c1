"""NEU-DET 数据集校验与统计（YOLO 格式）。

契约（TDD 断言目标）：
- class_names(data_yaml) -> list[str]：读取 nc/names。
- validate_split(root, split) -> DatasetStats：图片/标签计数、类别数、
  缺失标签数、非法标签行数、归一化越界记录。
- label_line_valid(line, nc) -> bool：class ∈ [0, nc)，5 个 float 均在 [0,1]。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


@dataclass(frozen=True)
class DatasetStats:
    split: str
    images: int
    labels: int
    num_classes: int
    missing_labels: int
    unparseable: int
    violations: tuple[str, ...]


def class_names(data_yaml: str | Path) -> list[str]:
    """从 data.yaml 读取类别名列表。"""
    raw = yaml.safe_load(Path(data_yaml).read_text(encoding="utf-8"))
    names = raw.get("names")
    if isinstance(names, dict):  # {0: 'crazing', ...}
        names = [names[i] for i in sorted(names)]
    return list(names)


def label_line_valid(line: str, nc: int) -> bool:
    """校验一行 YOLO 标签：cls 5 个值（cls, x, y, w, h），归一化 0~1。"""
    parts = line.split()
    if len(parts) != 5:
        return False
    try:
        cls, x, y, w, h = (float(p) for p in parts)
    except ValueError:
        return False
    if not (0 <= cls < nc) or int(cls) != cls:
        return False
    return all(0.0 <= v <= 1.0 for v in (x, y, w, h))


def validate_split(root: str | Path, split: str) -> DatasetStats:
    """校验一个 split（train/valid/test）的图片-标签对应与格式合法性。"""
    root = Path(root)
    img_dir = root / split / "images"
    lbl_dir = root / split / "labels"
    images = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in IMG_EXTS) if img_dir.exists() else []
    labels = sorted(p for p in lbl_dir.iterdir() if p.suffix.lower() == ".txt") if lbl_dir.exists() else []
    nc = len(class_names(root / "data.yaml"))

    label_stems = {p.stem for p in labels}
    image_stems = {p.stem for p in images}
    missing = len(image_stems - label_stems)

    unparseable = 0
    violations: list[str] = []
    for lp in labels:
        for lineno, line in enumerate(lp.read_text(encoding="utf-8").splitlines(), 1):
            line = line.strip()
            if not line:
                continue
            if not label_line_valid(line, nc):
                unparseable += 1
                violations.append(f"{lp.name}:{lineno}: {line!r}")

    return DatasetStats(
        split=split,
        images=len(images),
        labels=len(labels),
        num_classes=nc,
        missing_labels=missing,
        unparseable=unparseable,
        violations=tuple(violations),
    )
