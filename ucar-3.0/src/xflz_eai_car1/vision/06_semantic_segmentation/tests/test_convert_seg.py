"""convert_seg 构建脚本测试：build_calib_list（转换本体依赖 rknn，板上实测）。"""
import numpy as np
import pytest

from convert_seg import build_calib_list


def test_build_calib_list_ordered_and_counted(tmp_path):
    import cv2

    for name in ("b02.jpg", "a01.png", "c03.jpg", "d04.txt"):
        p = tmp_path / name
        if p.suffix in (".jpg", ".png"):
            cv2.imwrite(str(p), np.zeros((8, 8, 3), dtype=np.uint8))
        else:
            p.write_text("x")
    lst = build_calib_list(tmp_path, count=2)
    assert [p.name for p in lst] == ["a01.png", "b02.jpg"]


def test_build_calib_list_missing_dir_empty():
    assert build_calib_list("/nonexistent/dir", 10) == []
