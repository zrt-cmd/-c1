"""深度查看器 CLI（构建文档 04；规格 depth_viewer_guide.md）。

用法：
    # 文件模式（本地/离线）：RGB + uint16 深度图对
    python src/app.py --config config/depth3d_config.yaml   # 需在 config 填 rgb/depth 路径
    # 板上 ROS 模式（rclpy 订阅原生话题，严禁 remap）：
    python src/app.py --config ... --ros  （config source.mode: ros）
    # --display 开本地窗口（点击读深度）；MJPEG: http://<IP>:8080/depth.mjpg
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import cv2

from config import load_config
from depth import load_depth, normalize_depth, read_depth
from mjpeg_server import MjpegServer, require
from view import (STATUS_TEXT, click_to_pixel, compose_rgbd, draw_click,
                   screen_to_pixel)


def run_viewer(rgb: np.ndarray, depth: np.ndarray, cfg: dict,
               click_cb=None) -> np.ndarray:
    """合成双栏图；click_cb(x, y) 为显示坐标回调（窗口模式）。"""
    norm = normalize_depth(depth, cfg["depth"]["invalid_value"])
    return compose_rgbd(rgb, norm, cfg["depth"]["colormap"],
                        cfg["depth"]["display_scale"])


def main() -> int:
    ap = argparse.ArgumentParser(description="depth viewer (RGB | Depth)")
    ap.add_argument("--config", default="config/depth3d_config.yaml")
    ap.add_argument("--ros", action="store_true", help="ROS 模式（板上 rclpy）")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--display", action="store_true", help="本地窗口 + 点击读深度")
    args = ap.parse_args()
    click_state = {"u": None, "v": None, "text": ""}
    require()  # mjpeg_streamer 缺失 → 提示安装并退出
    try:
        cfg = load_config(args.config)
    except Exception as exc:
        print(f"config error: {exc}")
        return 2

    if args.ros or cfg["source"]["mode"] == "ros":
        try:
            import rclpy
        except ImportError:
            print("rclpy not available; run file mode instead (source.mode: image)")
            return 2
        return _ros_main(cfg, args, click_state)

    # ---- 文件模式 ----
    try:
        rgb = cv2.imread(cfg["source"]["rgb_path"])
        if rgb is None:
            print(f"cannot read rgb image: {cfg['source']['rgb_path']}")
            return 2
        depth = load_depth(cfg["source"]["depth_path"])
    except Exception as exc:
        print(f"source error: {exc}")
        return 2
    if rgb.shape[:2] != depth.shape[:2]:
        print(f"rgb {rgb.shape[:2]} vs depth {depth.shape[:2]} mismatch")
        return 2

    def on_click(x, y):
        u, v = click_to_pixel(x, y, rgb.shape[1], cfg["depth"]["display_scale"])
        cam = None if args.display is False else cfg["cam3d"]
        r = read_depth(depth, u, v, cfg["depth"]["scale"], cam)
        if r.z_mm == 0:
            msg = f"Click ({u}, {v}): invalid depth"
        else:
            msg = f"Click ({u}, {v}): z = {r.z_mm} mm ({r.z_m:.3f} m)"
            if r.xyz is not None:
                msg += f"  xyz=({r.xyz[0]:.3f}, {r.xyz[1]:.3f}, {r.xyz[2]:.3f}) m"
        print(msg)
        click_state["u"], click_state["v"] = u, v
        click_state["text"] = msg

    server = MjpegServer("0.0.0.0", args.port or cfg["server"]["port"],
                         stream_name=cfg["server"]["stream_name"]).start()
    window = None
    if args.display:
        window = cfg["display"]["window_name"]
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, lambda ev, x, y, f, p: (
            on_click(x, y) if ev == cv2.EVENT_LBUTTONDOWN else None))

    out = run_viewer(rgb, depth, cfg)
    if click_state["u"] is not None:
        out = draw_click(out, click_state["u"], click_state["v"],
                         click_state["text"], cfg["depth"]["display_scale"])
    server.set_frame(out)
    print(f"viewer ready: file mode, MJPEG http://<ip>:{server.port}/{cfg['server']['stream_name']}")
    if window is not None:
        t0 = time.time()
        while time.time() - t0 < 60:  # 文件模式窗口保持 60s
            disp = out.copy()
            if click_state["u"] is not None:
                disp = draw_click(disp, click_state["u"], click_state["v"],
                                  click_state["text"], cfg["depth"]["display_scale"])
            cv2.imshow(window, disp)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        cv2.destroyAllWindows()
    server.stop()
    return 0


def _ros_main(cfg, args, click_state):
    """板上 ROS 模式：订阅原生话题（严禁 remap），回调内读深度。"""
    import rclpy
    from rclpy.node import Node
    from cv_bridge import CvBridge

    class DepthViewer(Node):
        def __init__(self):
            super().__init__("depth_viewer_node")
            self.bridge = CvBridge()
            self.rgb = None
            self.depth = None
            self.cam = None
            self.create_subscription(
                __import__("sensor_msgs.msg", fromlist=["Image"]).Image,
                "/ascamera_hp60c/camera_publisher/rgb0/image", self._on_rgb, 1)
            self.create_subscription(
                __import__("sensor_msgs.msg", fromlist=["Image"]).Image,
                "/ascamera_hp60c/camera_publisher/depth0/image_raw", self._on_depth, 1)
            self.create_subscription(
                __import__("sensor_msgs.msg", fromlist=["CameraInfo"]).CameraInfo,
                "/ascamera_hp60c/camera_publisher/depth0/camera_info", self._on_info, 1)
            self.timer = self.create_timer(0.1, self._tick)

        def _on_rgb(self, msg):
            self.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        def _on_depth(self, msg):
            self.depth = self.bridge.imgmsg_to_cv2(msg, "passthrough").astype(np.uint16)

        def _on_info(self, msg):
            k = msg.k
            self.cam = {"fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5],
                        "width": msg.width, "height": msg.height}

        def _tick(self):
            if self.rgb is None or self.depth is None:
                return
            if float(self.rgb.mean()) < 5.0:  # 黑帧跳过
                return
            out = run_viewer(self.rgb, self.depth, cfg)
            if click_state["u"] is not None:
                out = draw_click(out, click_state["u"], click_state["v"],
                                 click_state["text"], cfg["depth"]["display_scale"])
            server.set_frame(out)

    rclpy.init()
    node = DepthViewer()
    server = MjpegServer("0.0.0.0", args.port or cfg["server"]["port"],
                         stream_name=cfg["server"]["stream_name"]).start()
    print(f"[viewer] ROS mode: MJPEG http://<ip>:{server.port}/{cfg['server']['stream_name']}")

    def on_click(x, y):
        """窗口点击读深度（读当前帧 node.depth；双栏右栏自动减左栏偏移）。"""
        u, v = click_to_pixel(x, y, node.rgb.shape[1],
                              cfg["depth"]["display_scale"])
        r = read_depth(node.depth, u, v, cfg["depth"]["scale"], node.cam)
        if r.z_mm == 0:
            msg = f"Click ({u}, {v}): invalid depth"
        else:
            msg = f"Click ({u}, {v}): z = {r.z_mm} mm ({r.z_m:.3f} m)"
            if r.xyz is not None:
                msg += f"  xyz=({r.xyz[0]:.3f}, {r.xyz[1]:.3f}, {r.xyz[2]:.3f}) m"
        print(msg)
        click_state["u"], click_state["v"] = u, v
        click_state["text"] = msg

    window = None
    if args.display:
        window = cfg["display"]["window_name"]
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, lambda ev, x, y, f, p: (
            on_click(x, y) if ev == cv2.EVENT_LBUTTONDOWN else None))
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if window is not None and node.rgb is not None and node.depth is not None:
                out = run_viewer(node.rgb, node.depth, cfg)
                if click_state["u"] is not None:
                    out = draw_click(out, click_state["u"], click_state["v"],
                                     click_state["text"], cfg["depth"]["display_scale"])
                cv2.imshow(window, out)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if window is not None:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
