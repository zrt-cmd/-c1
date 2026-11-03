"""HSV 颜色过滤管线（构建文档 02）。

对应课程：OpenCV 应用——RGB 颜色过滤 / HSV 颜色追踪（vendor 03）；效果对齐
车端 visual_grasp 可视化：RGB 栏（绿轮廓+黄框+红质心十字）+ HSV Mask 栏（白匹配区）。
BGR→HSV → inRange 阈值掩膜（H 环形第二段 h2）→ 开/闭运算 → 圆形轮廓；
检测结果携带掩膜，供双栏绘制。纯函数设计，逐级可单测。
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from config import ColorSpec


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float
    area: float


@dataclass(frozen=True)
class ColorDetect:
    color_name: str
    cx: float
    cy: float
    radius: float
    area: float
    mask: np.ndarray = None  # 该颜色开/闭后的掩膜（绘制用）


def bgr_to_hsv(img: np.ndarray) -> np.ndarray:
    """BGR -> HSV（OpenCV 约定：H∈0-179，S/V∈0-255）。"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)


def range_mask(hsv: np.ndarray, spec: ColorSpec) -> np.ndarray:
    """HSV 阈值掩膜；spec.h2 存在时并入 H 环形第二段（红色跨 0/180）。"""
    lower = np.array([spec.h[0], spec.s[0], spec.v[0]], dtype=np.uint8)
    upper = np.array([spec.h[1], spec.s[1], spec.v[1]], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    if spec.h2 is not None:
        lower2 = np.array([spec.h2[0], spec.s[0], spec.v[0]], dtype=np.uint8)
        upper2 = np.array([spec.h2[1], spec.s[1], spec.v[1]], dtype=np.uint8)
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower2, upper2))
    return mask


def morph_open(mask: np.ndarray, kernel: int = 5, iterations: int = 1) -> np.ndarray:
    """开运算：去孤立噪点。kernel 须为奇数。"""
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel, kernel))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=iterations)


def morph_close(mask: np.ndarray, kernel: int = 9, iterations: int = 1) -> np.ndarray:
    """闭运算：填小孔洞。kernel 须为奇数。"""
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel, kernel))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=iterations)


def largest_circle(mask: np.ndarray, min_area: float = 500,
                   min_radius: float = 10, max_radius: float = 400) -> Circle | None:
    """轮廓 → 最小外接圆，按面积/半径过滤后取最大。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best: Circle | None = None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        if not (min_radius <= radius <= max_radius):
            continue
        if best is None or area > best.area:
            best = Circle(x=float(cx), y=float(cy), radius=float(radius), area=float(area))
    return best


def color_mask(hsv: np.ndarray, cfg: dict, colors: list[ColorSpec]) -> np.ndarray:
    """全部颜色掩膜并集（开/闭后）；用于 Mask 栏。"""
    morph = cfg["morph"]
    union = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for spec in colors:
        union = cv2.bitwise_or(union, range_mask(hsv, spec))
    union = morph_open(union, morph["open_kernel"], morph["open_iter"])
    union = morph_close(union, morph["close_kernel"], morph["close_iter"])
    return union


def detect_colors(bgr: np.ndarray, cfg: dict, colors: list[ColorSpec]) -> list[ColorDetect]:
    """每色：掩膜 → 开/闭 → 最大圆。检出携带该色掩膜。"""
    hsv = bgr_to_hsv(bgr)
    morph = cfg["morph"]
    contour = cfg["contour"]
    dets = []
    for spec in colors:
        mask = range_mask(hsv, spec)
        mask = morph_open(mask, morph["open_kernel"], morph["open_iter"])
        mask = morph_close(mask, morph["close_kernel"], morph["close_iter"])
        c = largest_circle(mask, contour["min_area"], contour["min_radius"], contour["max_radius"])
        if c is not None:
            dets.append(ColorDetect(spec.name, c.x, c.y, c.radius, c.area, mask))
    return dets


def draw_detection(bgr: np.ndarray, dets: list[ColorDetect]) -> np.ndarray:
    """RGB 栏（visual_grasp 风格）：绿轮廓 + 黄外接框 + 红质心十字 + 颜色名。"""
    out = bgr.copy()
    for d in dets:
        if d.mask is not None:
            contours, _ = cv2.findContours(d.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(out, contours, -1, (0, 255, 0), 2)          # 绿色轮廓
            if contours:
                x, y, w, h = cv2.boundingRect(contours[0])
                cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)  # 黄框
        cx, cy = int(d.cx), int(d.cy)
        cv2.drawMarker(out, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)   # 红质心十字
        cv2.putText(out, d.color_name, (cx + 20, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(out, "RGB", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    return out


def mask_view(mask: np.ndarray, dets: list[ColorDetect] | None = None) -> np.ndarray:
    """HSV Mask 栏（visual_grasp 风格）：白=匹配区域 + 绿质心十字 + 青圆。"""
    out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    for d in dets or []:
        cx, cy = int(d.cx), int(d.cy)
        cv2.drawMarker(out, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
        cv2.circle(out, (cx, cy), int(d.radius), (255, 255, 0), 2)
    cv2.putText(out, "HSV Mask", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return out


def auto_hsv_range(bgr: np.ndarray, spec: ColorSpec, min_pixels: int = 50,
                   margin: int = 10) -> dict | None:
    """从首帧自动估计目标色 HSV 范围（种子色域内像素 min/max + 余量）。

    有 h2 的颜色（红）取像素更多的那个 H 段做主区间，h2 保持配置值。
    种子像素太少（场景中没看到该色）→ 返回 None，调用方回退配置值。
    """
    hsv = bgr_to_hsv(bgr)
    bands = [spec.h]
    if spec.h2 is not None:
        bands.append(spec.h2)
    best: dict | None = None
    best_n = 0
    for band in bands:
        lo = np.array([band[0], spec.s[0], spec.v[0]], dtype=np.uint8)
        hi = np.array([band[1], 255, 255], dtype=np.uint8)
        ys, xs = np.where(cv2.inRange(hsv, lo, hi) > 0)
        if len(xs) < min_pixels:
            continue
        px = hsv[ys, xs]
        if len(xs) > best_n:
            best_n = len(xs)
            best = {
                "h": [max(0, int(px[:, 0].min()) - margin),
                      min(179, int(px[:, 0].max()) + margin)],
                "s": [max(0, int(px[:, 1].min()) - 30), 255],
                "v": [max(0, int(px[:, 2].min()) - 30), 255],
            }
    return best
