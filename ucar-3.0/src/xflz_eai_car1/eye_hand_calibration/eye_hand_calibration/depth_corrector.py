#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
深度校正节点

订阅原始深度图，应用线性校正模型后重新发布。
校正参数从 config/depth_correction.yaml 加载（由标定流程生成）。

模型：depth_corrected = scale * depth_raw + offset

话题：
  订阅:
    /ascamera_hp60c/camera_publisher/depth0/image_raw  (sensor_msgs/Image, uint16, mm)
  发布:
    /eye_hand_calib/depth/image_corrected              (sensor_msgs/Image, uint16, mm)
    /eye_hand_calib/depth/correction_active            (std_msgs/Bool)

使用方式：
  ros2 launch eye_hand_calibration apply_correction.launch.py
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import numpy as np
from typing import Optional


class DepthCorrectorNode(Node):
    """
    实时深度图校正节点。

    如果没有标定结果（scale=1.0, offset=0.0），节点仍然正常运行，
    只是直接透传原始深度图，不做任何修改。
    可以通过 /eye_hand_calib/depth/correction_active 话题确认校正是否生效。
    """

    def __init__(self):
        super().__init__('depth_corrector')

        # ---- 参数 ----
        self.declare_parameter('config_path', '')
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('offset', 0.0)    # 米，内部转换为 mm
        self.declare_parameter('valid_range_min', 0.10)
        self.declare_parameter('valid_range_max', 1.50)
        self.declare_parameter('passthrough_on_error', True)

        config_path = self.get_parameter('config_path').value
        self._scale = self.get_parameter('scale').value
        self._offset_m = self.get_parameter('offset').value
        self._range_min = self.get_parameter('valid_range_min').value
        self._range_max = self.get_parameter('valid_range_max').value
        self._passthrough = self.get_parameter('passthrough_on_error').value

        # 尝试从文件加载校正参数（优先于 ROS 参数）
        self._load_correction_params(config_path)

        # offset 单位转换：米 → mm（深度图单位）
        self._offset_mm = self._offset_m * 1000.0
        self._range_min_mm = self._range_min * 1000.0
        self._range_max_mm = self._range_max * 1000.0

        self._bridge = CvBridge()
        self._correction_active = not (
            abs(self._scale - 1.0) < 1e-6 and abs(self._offset_mm) < 0.1
        )

        # ---- 订阅 ----
        self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self._depth_callback, 10)

        # 透传 camera_info（坐标系不变，内参不变）
        self.create_subscription(
            CameraInfo,
            '/ascamera_hp60c/camera_publisher/depth0/camera_info',
            self._camera_info_callback, 10)

        # ---- 发布 ----
        self._depth_pub = self.create_publisher(
            Image, '/eye_hand_calib/depth/image_corrected', 10)
        self._info_pub = self.create_publisher(
            CameraInfo, '/eye_hand_calib/depth/camera_info', 10)
        self._active_pub = self.create_publisher(
            Bool, '/eye_hand_calib/depth/correction_active', 10)

        # 定时发布校正状态
        self.create_timer(2.0, self._publish_status)

        self.get_logger().info(
            f'深度校正节点启动\n'
            f'  scale  = {self._scale:.6f}\n'
            f'  offset = {self._offset_m*1000:.2f} mm\n'
            f'  校正生效: {self._correction_active}\n'
            f'  有效范围: {self._range_min*100:.0f}cm ~ {self._range_max*100:.0f}cm'
        )

    # ------------------------------------------------------------------
    # 参数加载
    # ------------------------------------------------------------------

    def _load_correction_params(self, config_path: str):
        """从 YAML 文件加载校正参数。"""
        # 如果没有指定路径，尝试从包的 share 目录加载
        if not config_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                share = get_package_share_directory('eye_hand_calibration')
                config_path = os.path.join(share, 'config', 'depth_correction.yaml')
            except Exception:
                pass

        if not config_path or not os.path.exists(config_path):
            self.get_logger().warn(
                '未找到深度校正配置文件，使用默认参数（scale=1.0, offset=0.0）\n'
                '  运行标定后执行 compute_hand_eye_transform 生成配置文件'
            )
            return

        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            dc = cfg.get('depth_correction', {})
            self._scale = float(dc.get('scale', self._scale))
            self._offset_m = float(dc.get('offset', self._offset_m))
            valid_range = dc.get('valid_range', [self._range_min, self._range_max])
            self._range_min = float(valid_range[0])
            self._range_max = float(valid_range[1])
            self.get_logger().info(f'已加载深度校正参数: {config_path}')
        except Exception as e:
            self.get_logger().error(f'加载深度校正参数失败: {e}')

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------

    def _depth_callback(self, msg: Image):
        """接收深度图，应用校正后重新发布。"""
        try:
            depth_raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度图解码失败: {e}', throttle_duration_sec=5.0)
            return

        if not self._correction_active:
            # 无校正参数，直接透传
            self._depth_pub.publish(msg)
            return

        try:
            corrected = self._apply_correction(depth_raw)
            out_msg = self._bridge.cv2_to_imgmsg(corrected, encoding='passthrough')
            out_msg.header = msg.header
            self._depth_pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'深度校正失败: {e}', throttle_duration_sec=5.0)
            if self._passthrough:
                self._depth_pub.publish(msg)

    def _camera_info_callback(self, msg: CameraInfo):
        """透传 camera_info（坐标系和内参不变）。"""
        self._info_pub.publish(msg)

    def _publish_status(self):
        """定时发布校正是否生效。"""
        msg = Bool()
        msg.data = self._correction_active
        self._active_pub.publish(msg)

    # ------------------------------------------------------------------
    # 校正核心
    # ------------------------------------------------------------------

    def _apply_correction(self, depth_raw: np.ndarray) -> np.ndarray:
        """
        应用线性深度校正。

        depth_raw: uint16 深度图（单位 mm）
        返回:      uint16 校正后深度图（单位 mm）

        算法：
          1. uint16 → float32（mm）
          2. 应用线性校正：corrected_mm = scale * raw_mm + offset_mm
          3. 裁剪到有效范围
          4. 原始值为 0（无效点）的像素保持为 0
          5. float32 → uint16
        """
        # 转为 float 计算，避免整数溢出
        raw_f = depth_raw.astype(np.float32)

        # 有效像素掩码（0 = 无效，保持为 0）
        valid_mask = raw_f > 0

        # 线性校正
        corrected_f = raw_f * self._scale + self._offset_mm

        # 裁剪到有效范围
        corrected_f = np.clip(corrected_f, self._range_min_mm, self._range_max_mm)

        # 无效像素恢复为 0
        corrected_f[~valid_mask] = 0.0

        return corrected_f.astype(np.uint16)


def main(args=None):
    rclpy.init(args=args)
    node = DepthCorrectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
