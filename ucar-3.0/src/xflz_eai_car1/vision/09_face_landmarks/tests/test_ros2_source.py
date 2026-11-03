"""ros2_source 单元测试：无 rclpy 依赖（惰性导入 + 注入替身）。

契约（与各实验共用，逻辑一致；exp09 包结构 → face_landmark_core 导入）：
- Ros2Camera 构造不导入 rclpy；open() 才惰性导入（x86 无 ROS 可单测逻辑）
- 回调经 cv_bridge 转 BGR8；黑帧（mean < black_thr）跳过（驱动首帧黑）
- read(timeout) 返回 (True, 最新帧) / (False, None)；release 幂等
"""
from __future__ import annotations

import numpy as np
import pytest

from face_landmark_core.ros2_source import Ros2Camera


class FakeMsg:
    """sensor_msgs/Image 最小替身（回调只关心转码结果）。"""


class FakeBridge:
    def __init__(self, out):
        self._out = out
        self.calls = []

    def imgmsg_to_cv2(self, msg, encoding):
        self.calls.append(encoding)
        return self._out


class FakeNode:
    def __init__(self):
        self.subs = []
        self.destroyed = False

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs.append((topic, cb))

    def destroy_node(self):
        self.destroyed = True


def test_default_topic_is_ascamera_rgb():
    assert Ros2Camera.DEFAULT_TOPIC == "/ascamera_hp60c/camera_publisher/rgb0/image"


def test_construction_imports_no_rclpy():
    # 惰性导入契约：构造不触发 rclpy（x86 无 ROS 环境可单测）
    cam = Ros2Camera()
    assert cam._node is None


def test_on_image_converts_and_stores_latest():
    img = np.zeros((4, 5, 3), np.uint8)
    img[0, 0] = 200  # mean >= black_thr，不跳过
    cam = Ros2Camera(bridge=FakeBridge(img))
    cam._on_image(FakeMsg())
    assert cam._bridge.calls == ["bgr8"]
    ok, frame = cam.read(timeout=0.1)
    assert ok
    assert frame.shape == (4, 5, 3)


def test_on_image_skips_black_frame():
    cam = Ros2Camera(bridge=FakeBridge(np.zeros((4, 5, 3), np.uint8)))
    cam._on_image(FakeMsg())
    assert not cam.isOpened()


def test_on_image_skips_none():
    cam = Ros2Camera(bridge=FakeBridge(None))
    cam._on_image(FakeMsg())
    assert not cam.isOpened()


def test_read_timeout_returns_false_none():
    cam = Ros2Camera()
    ok, frame = cam.read(timeout=0.05)
    assert not ok and frame is None


def test_read_returns_latest_frame_after_update():
    img1 = np.full((2, 2, 3), 10, np.uint8)
    img2 = np.full((2, 2, 3), 20, np.uint8)
    cam = Ros2Camera(bridge=FakeBridge(img1))
    cam._on_image(FakeMsg())
    cam._bridge = FakeBridge(img2)
    cam._on_image(FakeMsg())
    ok, frame = cam.read(timeout=0.1)
    assert ok and int(frame[0, 0, 0]) == 20


def test_open_subscribes_and_release_destroys():
    pytest.importorskip("rclpy")  # 板上（已 source ROS）运行；x86 跳过
    node = FakeNode()
    cam = Ros2Camera(create_node=lambda name: node, spin=lambda n: None,
                     bridge=FakeBridge(np.full((2, 2, 3), 200, np.uint8)))
    assert cam.open()
    assert node.subs and node.subs[0][0] == Ros2Camera.DEFAULT_TOPIC
    node.subs[0][1](FakeMsg())  # 订阅回调 → 存帧
    assert cam.isOpened()
    cam.release()
    assert node.destroyed
    cam.release()  # 幂等
