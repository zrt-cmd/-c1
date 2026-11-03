#!/usr/bin/env python3
"""
RGBD Camera Web Viewer Server
Subscribes to HP60C camera topics and streams via WebSocket to browser
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from flask import Flask, send_from_directory
from flask_socketio import SocketIO
import threading
import time
import struct
import gzip
import base64
from collections import deque


class CameraWebServer(Node):
    def __init__(self, flask_app, socketio):
        super().__init__('camera_web_server')

        self.bridge = CvBridge()
        self.socketio = socketio

        # Frame storage
        self.latest_rgb = None
        self.latest_depth_visual = None
        self.latest_depth_raw = None  # Raw depth array in meters
        self.latest_pointcloud = None
        self.depth_width = 640
        self.depth_height = 480

        # Client state
        self.tracked_point = None  # (x, y) sticky tracked pixel
        self.enable_pointcloud_push = False  # Only push when 3D tab active

        # FPS tracking (last 50 frames)
        self.rgb_timestamps = deque(maxlen=50)
        self.depth_timestamps = deque(maxlen=50)
        self.pc_timestamps = deque(maxlen=50)

        # Topic names
        rgb_topic = '/ascamera_hp60c/camera_publisher/rgb0/image'
        depth_topic = '/ascamera_hp60c/camera_publisher/depth0/image_raw'
        pc_topic = '/ascamera_hp60c/camera_publisher/depth0/points'

        # Subscribers with sensor QoS (best_effort + keep_last(1) → no queue buildup)
        self.rgb_sub = self.create_subscription(
            Image, rgb_topic, self.rgb_callback, qos_profile_sensor_data)

        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, qos_profile_sensor_data)

        self.pc_sub = self.create_subscription(
            PointCloud2, pc_topic, self.pointcloud_callback, qos_profile_sensor_data)

        self.get_logger().info('Camera Web Server started')
        self.get_logger().info(f'Subscribing to: {rgb_topic}')
        self.get_logger().info(f'Subscribing to: {depth_topic}')
        self.get_logger().info(f'Subscribing to: {pc_topic}')

    def rgb_callback(self, msg):
        """RGB image callback"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 75])
            self.latest_rgb = base64.b64encode(buffer).decode('utf-8')
            self.rgb_timestamps.append(time.time())
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {e}')

    def depth_callback(self, msg):
        """Depth image callback - generate both visual and raw data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_height, self.depth_width = cv_image.shape

            # Convert to meters and store raw
            depth_meters = cv_image.astype(np.float32) / 1000.0
            self.latest_depth_raw = depth_meters

            # Generate colormap visualization (clip to 0-5m)
            depth_norm = np.clip(depth_meters, 0, 5.0) / 5.0
            depth_color = cv2.applyColorMap(
                (depth_norm * 255).astype(np.uint8),
                cv2.COLORMAP_TURBO
            )
            _, buffer = cv2.imencode('.jpg', depth_color, [cv2.IMWRITE_JPEG_QUALITY, 75])
            self.latest_depth_visual = base64.b64encode(buffer).decode('utf-8')

            self.depth_timestamps.append(time.time())
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {e}')

    def pointcloud_callback(self, msg):
        """PointCloud2 callback - parse XYZRGB, downsample 4x4, pack binary"""
        try:
            # Parse field offsets dynamically
            offsets = {f.name: f.offset for f in msg.fields}
            point_step = msg.point_step
            has_rgb = 'rgb' in offsets

            # Interpret raw buffer as structured bytes
            raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            n_points = len(raw) // point_step
            raw = raw[:n_points * point_step].reshape(n_points, point_step)

            # Extract XYZ as float32
            xyz = np.zeros((n_points, 3), dtype=np.float32)
            for i, axis in enumerate(('x', 'y', 'z')):
                off = offsets[axis]
                xyz[:, i] = raw[:, off:off + 4].copy().view(np.float32).ravel()

            # Extract RGB (packed float32 -> bytes b,g,r)
            if has_rgb:
                off = offsets['rgb']
                rgb_bytes = raw[:, off:off + 4]  # B G R A order from pcl
                colors = rgb_bytes[:, :3]  # take BGR
                # pcl packs as BGRA in little-endian float; reorder to RGB
                colors = colors[:, ::-1].copy()  # -> R G B
            else:
                colors = np.full((n_points, 3), 200, dtype=np.uint8)

            # Filter invalid points (NaN / zero depth)
            valid = np.isfinite(xyz).all(axis=1) & (np.abs(xyz).sum(axis=1) > 1e-4)
            xyz = xyz[valid]
            colors = colors[valid]

            # Downsample 4x4 -> keep every 16th point
            xyz = xyz[::16]
            colors = colors[::16]

            count = xyz.shape[0]
            if count == 0:
                return

            # Pack: float32 x,y,z + uint8 r,g,b per point
            buf = bytearray()
            xyz_bytes = xyz.astype('<f4').tobytes()
            rgb_packed = colors.astype(np.uint8).tobytes()
            # interleave on client side; send separately to keep it simple
            payload = struct.pack('<I', count) + xyz_bytes + rgb_packed
            compressed = gzip.compress(payload, compresslevel=1)
            self.latest_pointcloud = base64.b64encode(compressed).decode('utf-8')

            self.pc_timestamps.append(time.time())
        except Exception as e:
            self.get_logger().error(f'PointCloud callback error: {e}')

    @staticmethod
    def _calc_fps(timestamps):
        if len(timestamps) < 2:
            return 0.0
        span = timestamps[-1] - timestamps[0]
        if span <= 0:
            return 0.0
        return (len(timestamps) - 1) / span

    def query_depth(self, x, y):
        """Return depth in meters at pixel (x, y), or None"""
        if self.latest_depth_raw is None:
            return None
        h, w = self.latest_depth_raw.shape
        if 0 <= x < w and 0 <= y < h:
            d = float(self.latest_depth_raw[y, x])
            if np.isfinite(d) and d > 0:
                return d
        return None


# ---------------------------------------------------------------------------
# Flask + SocketIO web server
# ---------------------------------------------------------------------------

import os

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static')
if not os.path.isdir(STATIC_DIR):
    # When installed via colcon, static lives in share/<pkg>/static
    from ament_index_python.packages import get_package_share_directory
    try:
        STATIC_DIR = os.path.join(
            get_package_share_directory('camera_web_viewer'), 'static')
    except Exception:
        pass

flask_app = Flask(__name__, static_folder=None)
socketio = SocketIO(flask_app, cors_allowed_origins='*', async_mode='threading')

ros_node = None  # set in main()


@flask_app.route('/')
def index():
    return send_from_directory(STATIC_DIR, 'index.html')


@flask_app.route('/<path:path>')
def static_files(path):
    return send_from_directory(STATIC_DIR, path)


@socketio.on('set_tracked_point')
def handle_set_tracked_point(data):
    """Client clicked -> set sticky tracked point for continuous depth updates"""
    if ros_node is None:
        return
    x = data.get('x')
    y = data.get('y')
    if x is None or y is None:
        ros_node.tracked_point = None
    else:
        ros_node.tracked_point = (int(x), int(y))


@socketio.on('enable_pointcloud')
def handle_enable_pointcloud(data):
    """Client switched to/from 3D tab -> enable/disable pointcloud push"""
    if ros_node is None:
        return
    ros_node.enable_pointcloud_push = bool(data.get('enabled', False))


def stream_loop():
    """Background thread: push latest frames + FPS to all clients at 15Hz"""
    while True:
        try:
            if ros_node is not None:
                payload = {
                    'rgb': ros_node.latest_rgb,
                    'depth': ros_node.latest_depth_visual,
                    'depth_width': ros_node.depth_width,
                    'depth_height': ros_node.depth_height,
                    'fps': {
                        'rgb': round(ros_node._calc_fps(ros_node.rgb_timestamps), 1),
                        'depth': round(ros_node._calc_fps(ros_node.depth_timestamps), 1),
                        'pointcloud': round(ros_node._calc_fps(ros_node.pc_timestamps), 1),
                    },
                }

                # Include pointcloud only if 3D mode active
                if ros_node.enable_pointcloud_push:
                    payload['pointcloud'] = ros_node.latest_pointcloud
                else:
                    payload['pointcloud'] = None

                # Include tracked depth if a point is being tracked
                if ros_node.tracked_point:
                    x, y = ros_node.tracked_point
                    depth = ros_node.query_depth(x, y)
                    payload['tracked_xy'] = {'x': x, 'y': y}
                    payload['tracked_depth'] = depth
                else:
                    payload['tracked_xy'] = None
                    payload['tracked_depth'] = None

                socketio.emit('frame', payload)
            socketio.sleep(1.0 / 15.0)  # 15 Hz push (lower latency + bandwidth)
        except Exception:
            socketio.sleep(0.1)


def ros_spin():
    rclpy.spin(ros_node)


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = CameraWebServer(flask_app, socketio)

    # Spin ROS in a separate thread
    spin_thread = threading.Thread(target=ros_spin, daemon=True)
    spin_thread.start()

    # Start stream push loop
    socketio.start_background_task(stream_loop)

    ros_node.get_logger().info('Web server: http://0.0.0.0:8000  (LAN accessible)')
    try:
        socketio.run(flask_app, host='0.0.0.0', port=8000,
                     allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
