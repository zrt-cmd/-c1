"""手部 ROS2 节点（vendor 12 移植：订阅 RGB → 21 点骨架 → 双栏 → MJPEG）。

话题：/ascamera_hp60c/camera_publisher/rgb0/image（严禁 remap）。
仅板上有 rclpy；节点不在 x86 单测范围（语法/静态校验）。
"""
from __future__ import annotations

import time

import cv2
import numpy as np

from .config import ConfigError, load_config
from .engine import HandEngine, draw_boxes, draw_hands
from .fingers import count_fingers
from .mjpeg_server import MjpegServer, require
from .view import add_fps, compose, put_status


class HandControlNode:
    """rclpy 节点壳：image_callback → process → 双栏 → imshow/MJPEG。"""

    def __init__(self, cfg: dict):
        import rclpy  # 惰性：x86 无 rclpy 也不崩

        self._cfg = cfg
        self._engine = HandEngine(
            detector_model=cfg["model"]["detector_model"],
            landmark_model=cfg["model"]["landmark_model"],
            embedder_model=cfg["model"]["embedder_model"],
            classifier_model=cfg["model"]["classifier_model"],
            max_hands=cfg["model"]["max_hands"],
            min_detection_confidence=cfg["model"]["min_detection_confidence"],
            min_tracking_confidence=cfg["model"]["min_tracking_confidence"],
            core_mask=cfg["backend"]["core_mask"],
        )
        self._fps = 0.0
        self._p_time = time.time()
        self.exit_flag = False
        self.rgb = None

        self._node = rclpy.create_node("hand_control")
        qos = 1
        self._sub = self._node.create_subscription(
            __import__("sensor_msgs.msg", fromlist=["Image"]).Image,
            cfg.get("source", {}).get("ros", {}).get(
                "rgb_topic", "/ascamera_hp60c/camera_publisher/rgb0/image"),
            self._image_callback, qos)
        port = cfg["server"]["port"]
        if port:
            self._mjpeg = MjpegServer(port=port,
                                      stream_name=cfg["server"]["stream_name"])
            self._mjpeg.start()
        else:
            self._mjpeg = None

    def _image_callback(self, msg):
        try:
            import cv_bridge
            bridge = cv_bridge.CvBridge()
            frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        self.rgb = frame
        result = self._engine.process(frame)
        fingers = count_fingers(result.hands[0].landmarks) if result.hands else 0
        gesture = (result.hands[0].gesture_label if result.hands else "None")
        left = draw_boxes(add_fps(frame, self._fps), result)
        canvas = draw_hands(np.zeros_like(frame), result)
        dst = compose(left, canvas)
        now = time.time()
        dt = now - self._p_time
        self._p_time = now
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
        dst = put_status(dst, self._fps, result.num_hands, fingers, "color", gesture)
        if self._mjpeg is not None:
            self._mjpeg.set_frame(dst)
        if self._cfg["display"]["show"]:
            cv2.imshow(self._cfg["display"]["window_name"], dst)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.exit_flag = True

    def run(self):
        import rclpy
        while rclpy.ok() and not self.exit_flag:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            except KeyboardInterrupt:
                break  # Ctrl+C 打断 NPU 推理（KeyboardInterrupt 从 executor 逃逸）→ 干净退出
            except Exception as exc:
                # 单帧处理异常（推理失败等）→ 丢帧继续，不崩节点
                print(f"[run] frame error: {exc}")
        cv2.destroyAllWindows()

    def shutdown(self):
        self._engine.close()
        if self._mjpeg is not None:
            self._mjpeg.stop()
        try:
            self._node.destroy_node()
        except Exception:
            pass


def main():
    import argparse
    import sys
    parser = argparse.ArgumentParser(description="Hand control (exp12)")
    parser.add_argument("--config", default="config/hand_control_config.yaml")

    # ros2 launch 会追加 `--ros-args -r __node:=...`：先剥离 ROS 参数再解析（Humble 标准）
    import rclpy
    import rclpy.utilities
    rclpy.init(args=sys.argv)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    require()  # mjpeg_streamer 缺失 → 提示安装并退出

    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print(f"[hand_control] config error: {exc}")
        return 2
    node = HandControlNode(cfg)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        except Exception:
            pass  # Ctrl+C 后清理失败不掩盖退出
        try:
            rclpy.shutdown()
        except Exception:
            pass  # SIGINT 时 rclpy 已 shutdown context → RCLError，忽略
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
