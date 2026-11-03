"""ROS2 人脸关键点节点（移植 vendor yahboomcar_mediapipe/04_FaceMesh.py）。

vendor 对照：
- 订阅话题不变：/ascamera_hp60c/camera_publisher/rgb0/image（严禁 remap）
- image_callback: bridge -> process -> FPS -> frame_combine -> imshow -> q 退出
- pubFaceMeshPoint 的 mediapipe process + draw -> FaceLandmarkerEngine.process + draw_mesh
- run(): spin_once 循环 + exit_flag
差异：后端 = RKNN Blaze（RKNNLite 板上推理，468 点）；省略 /mediapipe/points
发布（依赖 yahboomcar_msgs，板工作区未含）；新增 MJPEG 流（ssh 无显示器可用）。
"""
from __future__ import annotations

import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from .config import ConfigError, load_config
from .landmark import FaceLandmarkerEngine, draw_boxes, draw_mesh
from .mjpeg_server import MjpegServer, require
from .view import compose, put_status

RGB_TOPIC = "/ascamera_hp60c/camera_publisher/rgb0/image"


class FaceLandmarkNode(Node):
    def __init__(self, cfg: dict, engine: FaceLandmarkerEngine | None = None):
        super().__init__("face_landmark")
        self._cfg = cfg
        # 引擎可注入（测试），默认按配置创建（RKNN Blaze 双模型，懒加载 RKNNLite）
        self._engine = engine or FaceLandmarkerEngine(
            detector_model=cfg["model"]["detector_model"],
            landmark_model=cfg["model"]["landmark_model"],
            num_faces=cfg["model"]["num_faces"],
            min_detection_confidence=cfg["model"]["min_detection_confidence"],
            core_mask=cfg["backend"]["core_mask"],
        )
        from cv_bridge import CvBridge
        self._bridge = CvBridge()
        self._frame_id = 0
        self._p_time = time.time()
        self._fps = 0.0
        self.exit_flag = False

        self._mjpeg = None
        if cfg["server"].get("enabled", True):
            self._mjpeg = MjpegServer(
                port=cfg["server"]["port"],
                stream_name=cfg["server"]["stream_name"],
            )
            self._mjpeg.start()

        self.create_subscription(Image, RGB_TOPIC, self.image_callback, 10)
        self.get_logger().info(
            f"subscribed {RGB_TOPIC}; detector={cfg['model']['detector_model']} "
            f"landmark={cfg['model']['landmark_model']}")

    def image_callback(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        timestamp_ms = self._frame_id * 33  # 单调递增（VIDEO 模式要求）
        self._frame_id += 1

        result = self._engine.process(frame, timestamp_ms)
        det = draw_boxes(frame, result)   # 左栏：检测框（YOLO 风格）
        mesh = draw_mesh(frame, result)   # 右栏：468 点网格
        dst = compose(det, mesh)

        now = time.time()
        dt = now - self._p_time
        self._p_time = now
        if dt > 0:
            self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
        dst = put_status(dst, self._fps, result.num_faces)

        if self._mjpeg is not None:
            self._mjpeg.set_frame(dst)
        if self._cfg["display"]["show"]:
            cv2.imshow(self._cfg["display"]["window_name"], dst)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.exit_flag = True

    def run(self):
        """vendor 对照：spin_once 循环 + exit_flag 退出。"""
        while rclpy.ok() and not self.exit_flag:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
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


def main():
    import argparse
    import sys
    parser = argparse.ArgumentParser(description="Face landmark (exp09)")
    parser.add_argument("--config", default="config/face_config.yaml")

    # ros2 launch 会追加 `--ros-args -r __node:=...`：先剥离 ROS 参数再解析（Humble 标准）
    rclpy.init(args=sys.argv)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    require()  # mjpeg_streamer 缺失 → 提示安装并退出

    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print(f"[face_landmark] config error: {exc}")
        return 2
    node = FaceLandmarkNode(cfg)
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
