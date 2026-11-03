"""二维码生成 / 检测 / 标注（构建文档 02）。

生成：qrcode 库（参考 QRcode_Create.py 同款参数语义）。
检测：OpenCV 内置 QRCodeDetector（零新增、无需模型权重，替代参考的 pyzbar——
      pyzbar 需系统库 libzbar0）。
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import cv2

import qrcode

from config import error_correction_level

MAX_DETECT_PASSES = 8  # 一张图最多检出个数（防止死循环）


@dataclass(frozen=True)
class QRResult:
    text: str
    x1: int
    y1: int
    x2: int
    y2: int


def generate_qr(data: str, cfg: dict) -> np.ndarray:
    """生成二维码图像（BGR HWC uint8）。"""
    if not data:
        raise ValueError("data must not be empty")
    g = cfg["generate"]
    qr = qrcode.QRCode(
        version=g["version"],
        error_correction=error_correction_level(g["error_correction"]),
        box_size=g["box_size"],
        border=g["border"],
    )
    qr.add_data(data)
    qr.make(fit=True)
    img = qr.make_image(fill_color=tuple(g["fill_color"]),
                        back_color=tuple(g["back_color"])).convert("RGB")
    return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)


def detect_qr(img: np.ndarray, scale: float = 0.5) -> list[QRResult]:
    """检测并解码图像中的二维码（可多个；返回原图坐标）。"""
    h, w = img.shape[:2]
    work = img
    factor = 1.0
    if 0 < scale < 1.0:
        work = cv2.resize(img, (int(w * scale), int(h * scale)))
        factor = 1.0 / scale

    detector = cv2.QRCodeDetector()
    results: list[QRResult] = []
    for _ in range(MAX_DETECT_PASSES):
        data, points, _ = detector.detectAndDecode(work)
        if not data:
            break
        if points is None or points.size == 0:
            break
        pts = points[0].astype(np.float32)
        xs = pts[:, 0] * factor
        ys = pts[:, 1] * factor
        results.append(QRResult(
            text=data,
            x1=int(xs.min()), y1=int(ys.min()),
            x2=int(xs.max()), y2=int(ys.max()),
        ))
        # 掩掉已识别区域，继续找下一个
        x1 = int(max(0, xs.min() * scale - 2))
        y1 = int(max(0, ys.min() * scale - 2))
        x2 = int(min(work.shape[1], xs.max() * scale + 2))
        y2 = int(min(work.shape[0], ys.max() * scale + 2))
        work[y1:y2, x1:x2] = 255
    return results


def draw_qr(img: np.ndarray, results: list[QRResult]) -> np.ndarray:
    """画框 + 解析文本标注。"""
    out = img.copy()
    for r in results:
        cv2.rectangle(out, (r.x1, r.y1), (r.x2, r.y2), (0, 255, 0), 2)
        label = r.text if len(r.text) <= 40 else r.text[:37] + "..."
        cv2.putText(out, label, (r.x1, max(16, r.y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    return out
