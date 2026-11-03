"""多边形轮廓检测管线（构建文档 02；vendor「OpenCV应用——多边形轮廓」语义）。

二值化 → 轮廓 → convexHull/approxPolyDP 拟合 → 多边形绘制（含顶点数标签）。
支持两种显示模式：
  draw_polygons      — 原图叠加（红色轮廓 + 黄色顶点 + 顶点数标签）
  draw_polygons_dark — 黑底彩色轮廓 + 质心圆点（vendor convex_hull 效果）
"""
from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Optional

import numpy as np
import cv2


@dataclass(frozen=True)
class PolygonResult:
    vertices: np.ndarray  # (N, 1, 2) int32
    area: float
    n_vertices: int


def to_binary(bgr: np.ndarray, cfg: dict) -> np.ndarray:
    """灰度 → 阈值（binary/adaptive）→ 反相 → 形态学开运算。"""
    p = cfg["preprocess"]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    if p["threshold_mode"] == "adaptive":
        mask = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                     cv2.THRESH_BINARY, 11, 2)
    else:
        _, mask = cv2.threshold(gray, p["threshold_value"], 255, cv2.THRESH_BINARY)
    if p["invert"]:
        mask = cv2.bitwise_not(mask)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                  (p["morph_kernel"], p["morph_kernel"]))
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)


def find_contours(mask: np.ndarray, min_area: int = 2000) -> list[np.ndarray]:
    """外部轮廓 + 面积过滤（按面积降序）。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) >= min_area]


def fit_polygon(contour: np.ndarray, epsilon_ratio: float = 0.02,
                use_hull: bool = True) -> np.ndarray:
    """vendor convex_hull + 多边形简化：凸包 → approxPolyDP 拟合规则多边形。

    先凸包填平凹槽，再 approx 简化到少顶点（矩形→4、三角→3）。
    use_hull=False 时直接近似原始轮廓。返回 (N,1,2) int32 顶点。
    """
    shape = contour
    if use_hull:
        shape = cv2.convexHull(contour)
    peri = cv2.arcLength(shape, True)
    return cv2.approxPolyDP(shape, epsilon_ratio * peri, True)


def detect_polygons(bgr: np.ndarray, cfg: dict) -> list[PolygonResult]:
    """全链路：二值化 → 轮廓 → 多边形拟合。按面积降序返回。"""
    mask = to_binary(bgr, cfg)
    results = []
    for c in find_contours(mask, cfg["contour"]["min_area"]):
        poly = fit_polygon(c, cfg["polygon"]["approx_epsilon"], cfg["polygon"]["use_hull"])
        results.append(PolygonResult(
            vertices=poly,
            area=float(cv2.contourArea(c)),
            n_vertices=int(len(poly)),
        ))
    results.sort(key=lambda r: r.area, reverse=True)
    return results


def draw_polygons(img: np.ndarray, results: list[PolygonResult]) -> np.ndarray:
    """绘制凸包多边形 + 顶点标记（黄点）+ 「n边形」标签（vendor convex_hull 效果）。"""
    out = img.copy()
    for r in results:
        cv2.polylines(out, [r.vertices], True, (0, 0, 255), 2)
        for v in r.vertices:
            x, y = v[0]
            cv2.circle(out, (int(x), int(y)), 4, (0, 255, 255), -1)  # 顶点黄点
        x, y = r.vertices[0][0]
        cv2.putText(out, str(r.n_vertices), (int(x), max(16, int(y) - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return out


def draw_polygons_dark(img: np.ndarray, results: list[PolygonResult],
                       seed: Optional[int] = None) -> np.ndarray:
    """黑底彩色多边形轮廓 + 质心圆点（vendor convex_hull 视觉效果）。

    每个轮廓使用独立随机颜色；画布为纯黑，避免背景干扰轮廓感知。
    Args:
        img:     原始 BGR 帧（仅用于确定画布尺寸）。
        results: detect_polygons 返回的 PolygonResult 列表。
        seed:    随机数种子（测试用；None = 每帧随机）。
    Returns:
        黑底 BGR 图像，与 img 同分辨率。
    """
    rng = random.Random(seed)
    canvas = np.zeros_like(img)
    for r in results:
        color = (rng.randint(50, 255), rng.randint(50, 255), rng.randint(50, 255))
        # 绘制多边形轮廓线
        cv2.polylines(canvas, [r.vertices], True, color, 2)
        # 计算质心并绘制实心圆点
        M = cv2.moments(r.vertices)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.circle(canvas, (cx, cy), 6, color, -1)
        # 顶点数标签（与轮廓同色）
        x, y = r.vertices[0][0]
        cv2.putText(canvas, str(r.n_vertices),
                    (int(x), max(16, int(y) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
    return canvas
