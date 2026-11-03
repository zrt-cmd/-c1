"""view 模块 TDD 测试（构建文档 03 的 4 条计划）。"""
import numpy as np

from view import compose_rgbd, draw_click, screen_to_pixel, STATUS_TEXT


def _rgb_and_depth(h=480, w=640):
    rgb = np.zeros((h, w, 3), dtype=np.uint8)
    rgb[..., 2] = 255  # 红
    depth = np.zeros((h, w), dtype=np.uint8)
    depth[100:400, 100:500] = 200
    return rgb, depth


def test_compose_shapes_and_panes():
    rgb, depth = _rgb_and_depth()
    out = compose_rgbd(rgb, depth, scale=0.666)
    sh = round(480 * 0.666)
    sw = round(640 * 0.666)
    assert out.shape == (sh, 2 * sw, 3)
    # 左栏 = RGB 缩放（红通道满值）
    assert out[50, 50, 2] == 255 and out[50, 50, 0] == 0
    # 右栏与左栏不同（伪彩）
    assert not np.array_equal(out[:, :sw], out[:, sw:])


def test_status_bar_present():
    rgb, depth = _rgb_and_depth()
    out = compose_rgbd(rgb, depth, scale=0.666)
    # 状态栏文字区（左上角）应有非纯色像素
    assert out[10:24, 10:400].max() > 0


def test_screen_to_pixel_restores():
    u, v = screen_to_pixel(213, 160, 0.666)
    assert (u, v) == (320, 240)


def test_draw_click_changes_pixels():
    rgb, depth = _rgb_and_depth()
    out = compose_rgbd(rgb, depth, scale=0.666)
    marked = draw_click(out.copy(), 320, 240, "z = 1000 mm", scale=0.666)
    assert not np.array_equal(out, marked)


def test_click_to_pixel_right_pane_offset():
    """双栏视图：点右栏（Depth）时减去左栏宽度再映射原图坐标。"""
    from view import click_to_pixel

    scale = 0.666
    left_w = int(round(640 * scale))  # 426
    # 右栏内点击 x=left_w+320 -> 原图 u≈320/scale
    u, v = click_to_pixel(left_w + 320, 200, 640, scale)
    assert u == int(round(320 / scale))
    # 左栏点击不受影响
    u2, _ = click_to_pixel(200, 100, 640, scale)
    assert u2 == int(round(200 / scale))
