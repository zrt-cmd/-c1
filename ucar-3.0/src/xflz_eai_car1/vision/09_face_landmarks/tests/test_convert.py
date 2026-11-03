"""convert_mediapipe 测试（契约表/头部解析/参数校验；真实转换需转换环境）。"""
from __future__ import annotations

import json

import pytest

from face_landmark_core.convert_mediapipe import (
    EXPECTED,
    PLATFORMS,
    check,
    download_tflite,
    main,
    parse_rknn_header,
)


def test_expected_contract_table():
    assert set(EXPECTED) == {"face_detection_short_range", "face_landmark"}
    det = EXPECTED["face_detection_short_range"]
    assert det["input"] == [1, 128, 128, 3]
    assert det["output_names"] == ["regressors", "classificators"]
    assert det["input_name"] == "input"
    lm = EXPECTED["face_landmark"]
    assert lm["input"] == [1, 192, 192, 3]
    assert lm["output_names"] == ["conv2d_21", "conv2d_31"]
    assert lm["input_name"] == "input_1"
    for spec in EXPECTED.values():
        assert spec["url"].startswith("https://storage.googleapis.com/mediapipe-assets/")


def test_platforms_include_rk3576():
    assert "rk3576" in PLATFORMS  # 课程板
    assert PLATFORMS == ("rk3562", "rk3566", "rk3568", "rk3576", "rk3588")


def test_parse_rknn_header_extracts_tensors(tmp_path):
    fake = tmp_path / "fake.rknn"
    payload = (b"RKNN\x00\x01binary" +
               b'{"regressors": {"dtype": "float16", "layout": "NHWC"}, '
               b'"classificators": {"dtype": "float16", "layout": "NHWC"}}' +
               b'{"input": {"dtype": "float16", "layout": "NHWC"}}' +
               b"more-binary-data")
    fake.write_bytes(payload)
    h = parse_rknn_header(fake)
    assert h["regressors"]["dtype"] == "float16"
    assert h["input"]["layout"] == "NHWC"


def test_parse_rknn_header_no_contract_raises(tmp_path):
    fake = tmp_path / "bad.rknn"
    fake.write_bytes(b"no json here at all")
    with pytest.raises(ValueError):
        parse_rknn_header(fake)


def test_download_skips_existing(tmp_path):
    tflite = tmp_path / "face_landmark.tflite"
    tflite.write_bytes(b"existing")
    got = download_tflite("face_landmark", tmp_path)
    assert got == tflite
    assert got.read_bytes() == b"existing"


def test_check_missing_model_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        check("face_landmark", tmp_path)


def test_check_contract_mismatch_raises(tmp_path):
    fake = tmp_path / "face_landmark.rknn"
    fake.write_bytes(json.dumps({"conv2d_21": {"dtype": "float16"},
                                 "input_1": {"dtype": "float16"}}).encode())
    with pytest.raises(AssertionError):
        check("face_landmark", tmp_path)  # 缺 conv2d_31


def test_check_ok(tmp_path):
    fake = tmp_path / "face_landmark.rknn"
    fake.write_bytes(json.dumps(
        {"conv2d_21": {"dtype": "float16"}, "conv2d_31": {"dtype": "float16"},
         "input_1": {"dtype": "float16"}}).encode())
    h = check("face_landmark", tmp_path)
    assert h["conv2d_21"]["dtype"] == "float16"


def test_main_unknown_model(monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["convert", "--name", "nope", "--check"])
    assert main() == 2
    out = capsys.readouterr().out
    assert "unknown model" in out


def test_main_check_failure_reports(monkeypatch, capsys, tmp_path):
    monkeypatch.setattr("face_landmark_core.convert_mediapipe.MODELS_DIR", tmp_path)
    monkeypatch.setattr("sys.argv", ["convert", "--all", "--check"])
    assert main() == 1
    out = capsys.readouterr().out
    assert "FAILED" in out
