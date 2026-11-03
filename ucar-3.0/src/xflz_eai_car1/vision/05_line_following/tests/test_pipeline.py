"""pipeline 模块 TDD 测试（构建文档 02 的 6 条计划）。"""
import numpy as np
import pytest

from pipeline import (
    PolygonResult,
    detect_polygons,
    draw_polygons,
    draw_polygons_dark,
    find_contours,
    fit_polygon,
    to_binary,
)


def _cfg():
    return {
        "preprocess": {"threshold_mode": "binary", "threshold_value": 127,
                       "invert": True, "morph_kernel": 3},
        "contour": {"min_area": 500},
        "polygon": {"approx_epsilon": 0.02, "use_hull": True},
    }


def _dark_square_img(size=200, side=80, x=60, y=60):
    """亮底中央暗方块。"""
    img = np.full((size, size, 3), 255, dtype=np.uint8)
    img[y:y + side, x:x + side] = (0, 0, 0)
    return img


def test_to_binary_invert():
    img = _dark_square_img()
    mask = to_binary(img, _cfg())
    assert mask.dtype == np.uint8
    assert set(np.unique(mask)) <= {0, 255}
    assert mask[100, 100] == 255      # 暗方块 -> 前景
    assert mask[10, 10] == 0          # 亮底 -> 背景


def test_to_binary_no_invert():
    cfg = _cfg()
    cfg["preprocess"]["invert"] = False
    mask = to_binary(_dark_square_img(), cfg)
    assert mask[100, 100] == 0        # 暗方块 -> 背景
    assert mask[10, 10] == 255


def test_to_binary_adaptive_runs():
    cfg = _cfg()
    cfg["preprocess"]["threshold_mode"] = "adaptive"
    mask = to_binary(_dark_square_img(), cfg)
    # adaptive 在均匀区域行为与 local mean 相关，不断言具体像素，只验证二值输出
    assert set(np.unique(mask)) <= {0, 255}


def test_find_contours_filters_noise():
    mask = np.zeros((200, 200), dtype=np.uint8)
    mask[60:140, 60:140] = 255        # 大方块
    mask[10, 10] = 255                # 单点噪
    contours = find_contours(mask, min_area=500)
    assert len(contours) == 1
    assert cv2_contour_area(contours[0]) == pytest.approx(80 * 80, rel=0.05)


def cv2_contour_area(c):
    import cv2
    return cv2.contourArea(c)


def test_fit_polygon_square_four_vertices():
    import cv2
    img = np.zeros((200, 200), dtype=np.uint8)
    cv2.rectangle(img, (60, 60), (140, 140), 255, -1)
    (contours, _) = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    poly = fit_polygon(contours[0], epsilon_ratio=0.02, use_hull=True)
    assert len(poly) == 4


def test_fit_polygon_hull_fills_concavity():
    import cv2
    # 凹形（U 字）：凸包填平凹槽 -> 顶点数少于直接近似
    img = np.zeros((200, 200), dtype=np.uint8)
    cv2.rectangle(img, (40, 40), (160, 120), 255, -1)
    cv2.rectangle(img, (70, 40), (130, 90), 0, -1)   # 上方挖凹槽
    (contours, _) = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hull = fit_polygon(contours[0], 0.02, use_hull=True)
    approx = fit_polygon(contours[0], 0.02, use_hull=False)
    assert len(hull) < len(approx)


def test_detect_polygons_end_to_end():
    img = _dark_square_img()
    results = detect_polygons(img, _cfg())
    assert len(results) == 1
    r = results[0]
    assert isinstance(r, PolygonResult)
    # convex_hull + 多边形简化：方块 → approx 简化为 4 顶点
    assert r.n_vertices == 4
    assert r.area == pytest.approx(80 * 80, rel=0.05)


def test_detect_polygons_filters_small():
    img = _dark_square_img()
    img[5:12, 5:12] = (0, 0, 0)      # 小噪块（面积 49 < 500）
    results = detect_polygons(img, _cfg())
    assert len(results) == 1


def test_draw_polygons_changes_pixels():
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    results = detect_polygons(_dark_square_img(), _cfg())
    out = draw_polygons(img, results)
    assert out.shape == img.shape
    assert not np.array_equal(out, img)
    # 空结果 -> 原图
    assert np.array_equal(draw_polygons(img, []), img)


def test_draw_polygons_dark_black_background():
    """黑底：无轮廓区域保持全黑。"""
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    results = detect_polygons(_dark_square_img(), _cfg())
    out = draw_polygons_dark(img, results, seed=42)
    assert out.shape == img.shape
    # 角落（远离轮廓）应保持黑色
    assert tuple(out[0, 0]) == (0, 0, 0)


def test_draw_polygons_dark_paints_contour():
    """彩色轮廓：结果与全黑图不相同。"""
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    results = detect_polygons(_dark_square_img(), _cfg())
    out = draw_polygons_dark(img, results, seed=42)
    assert not np.array_equal(out, img)


def test_draw_polygons_dark_empty_stays_black():
    """空结果 → 返回全黑画布。"""
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    out = draw_polygons_dark(img, [], seed=0)
    assert np.array_equal(out, img)


def test_draw_polygons_dark_deterministic_with_seed():
    """相同 seed → 相同颜色。"""
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    results = detect_polygons(_dark_square_img(), _cfg())
    a = draw_polygons_dark(img, results, seed=7)
    b = draw_polygons_dark(img, results, seed=7)
    assert np.array_equal(a, b)


def test_draw_polygons_dark_different_seeds_differ():
    """不同 seed → 不同颜色（大概率）。"""
    img = np.zeros((200, 200, 3), dtype=np.uint8)
    results = detect_polygons(_dark_square_img(), _cfg())
    a = draw_polygons_dark(img, results, seed=1)
    b = draw_polygons_dark(img, results, seed=2)
    assert not np.array_equal(a, b)
