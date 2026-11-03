"""dataset 模块 TDD 测试。"""
import pytest

from dataset import class_names, label_line_valid, validate_split
from conftest import NEU_DET


def test_label_line_valid_ok():
    assert label_line_valid("0 0.5 0.5 0.4 0.4", nc=2)
    assert label_line_valid("1 0.0 1.0 0.0 1.0", nc=2)


def test_label_line_invalid():
    assert not label_line_valid("2 0.5 0.5 0.4 0.4", nc=2)          # 类别越界
    assert not label_line_valid("0 0.5 0.5 0.4", nc=2)              # 少一列
    assert not label_line_valid("0 a 0.5 0.4 0.4", nc=2)            # 非数值
    assert not label_line_valid("0 1.5 0.5 0.4 0.4", nc=2)          # 归一化越界
    assert not label_line_valid("0 0.5 0.5 0.4 0.4 extra", nc=2)    # 多一列


def test_mini_dataset_stats(mini_yolo):
    st = validate_split(mini_yolo, "train")
    assert st.images == 3
    assert st.labels == 3
    assert st.num_classes == 2
    assert st.unparseable == 1          # img0.txt 中的 5 号类别行
    assert st.violations[0].startswith("img0.txt:2:")


def test_class_names(mini_yolo):
    assert class_names(mini_yolo / "data.yaml") == ["a", "b"]


@pytest.mark.skipif(not NEU_DET.exists(), reason="NEU-DET 数据集不在本机（在训练 PC 上运行）")
def test_real_neu_det_splits():
    """真实数据集断言：train 1439 / valid 216 / test 144（实测计数），6 类，无缺失标签、无非法行。"""
    for split, expected in [("train", 1439), ("valid", 216), ("test", 144)]:
        st = validate_split(NEU_DET, split)
        assert st.images == expected, f"{split}: {st.images}"
        assert st.labels == expected
        assert st.num_classes == 6
        assert st.missing_labels == 0
        assert st.unparseable == 0
