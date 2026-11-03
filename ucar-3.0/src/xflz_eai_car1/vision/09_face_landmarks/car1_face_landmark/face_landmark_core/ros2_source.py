"""ROS2 话题取流（ascamera HP60C 原生话题；借鉴 ros2_topic_view.py）。

板上取流标准：先起相机驱动
    source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
    ros2 launch ascamera hp60c.launch.py        # ROS_DOMAIN_ID 必须与驱动一致（本板 =2）
再以 --ros 运行各实验。本模块把话题帧包装成 cv2.VideoCapture 兼容接口
（read/isOpened/release），主循环与 MJPEG 推流零改动。

rclpy 惰性导入（同 RKNNLiteBackend 模式）：构造/单测不依赖 ROS；
open() 才导入 rclpy + cv_bridge（x86 无 ROS 环境可测回调逻辑）。
"""
from __future__ import annotations

import threading
import time

import numpy as np


class Ros2Camera:
    """订阅 sensor_msgs/Image（默认 BGR8）→ 缓存最新帧。

    - open(): 惰性导入 rclpy；未 init 则 rclpy.init()；daemon 线程 spin
    - 回调：cv_bridge 转 BGR8；黑帧（mean < black_thr）跳过（驱动首帧黑）
    - read(timeout): 阻塞至首帧或超时，返回 (True, 最新帧) / (False, None)
    - release(): destroy node；若本模块 init 的 rclpy 则 shutdown（幂等）
    """

    DEFAULT_TOPIC = "/ascamera_hp60c/camera_publisher/rgb0/image"

    def __init__(self, topic: str = DEFAULT_TOPIC, encoding: str = "bgr8",
                 black_thr: float = 5.0, qos: int = 10,
                 node_name: str = "ros2_camera",
                 bridge=None, create_node=None, spin=None):
        self._topic = topic
        self._encoding = encoding
        self._black_thr = black_thr
        self._qos = qos
        self._node_name = node_name
        self._bridge = bridge          # 测试注入替身
        self._create_node = create_node  # 测试注入节点工厂
        self._spin = spin              # 测试注入 spin
        self._node = None
        self._frame = None
        self._ok = False
        self._inited_here = False
        self._spin_thread = None
        self._lock = threading.Lock()

    def open(self) -> bool:
        """惰性导入 rclpy 并建节点订阅；返回是否成功。"""
        if self._node is not None:
            return True
        import rclpy  # 惰性：x86 单测不依赖
        from cv_bridge import CvBridge
        from sensor_msgs.msg import Image
        if not rclpy.ok():
            rclpy.init()
            self._inited_here = True
        node = self._create_node(self._node_name) if self._create_node \
            else rclpy.create_node(self._node_name)
        if self._bridge is None:
            self._bridge = CvBridge()
        node.create_subscription(Image, self._topic, self._on_image, self._qos)
        spin = self._spin or rclpy.spin
        self._spin_thread = threading.Thread(target=spin, args=(node,), daemon=True)
        self._spin_thread.start()
        self._node = node
        return True

    def _on_image(self, msg):
        """订阅回调：转 BGR8 → 存最新帧；黑帧/空帧跳过。"""
        img = self._bridge.imgmsg_to_cv2(msg, self._encoding)
        if img is None:
            return
        if self._black_thr and float(np.mean(img)) < self._black_thr:
            return  # 驱动按需推流，首帧黑帧跳过
        with self._lock:
            self._frame = img
            self._ok = True

    def read(self, timeout: float = 5.0):
        """阻塞至有帧或超时；返回 (True, 最新帧) / (False, None)。"""
        deadline = time.time() + timeout
        while True:
            with self._lock:
                frame = self._frame
            if frame is not None:
                return True, frame
            if time.time() >= deadline:
                return False, None
            time.sleep(0.01)

    def isOpened(self) -> bool:
        with self._lock:
            return self._ok

    def release(self) -> None:
        # 顺序关键：先 rclpy.shutdown()（spin 线程收到 ExternalShutdownException 退出），
        # 再 destroy_node——反向会 C++ abort（板上实测 terminate called）
        if self._inited_here:
            import rclpy
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self._inited_here = False
        node = self._node
        self._node = None
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2)
