#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
标定结果验证工具

在新位置放置 AprilTag，比较标定前后的 3D 位置误差。

验证逻辑：
  1. 用 AprilTag PnP 解算出标签在相机坐标系的位置（参考值）
  2. 用深度图中心点深度 + 内参反投影到相机坐标系（测量值，校正前）
  3. 用校正后的深度重新反投影（测量值，校正后）
  4. 将三种结果都变换到 base_link 坐标系
  5. 计算 PnP 参考值与校正前/后的 3D 误差

话题：
  订阅:
    /eye_hand_calib/apriltag/pose        (geometry_msgs/PoseStamped)
    /ascamera_hp60c/camera_publisher/depth0/image_raw
    /ascamera_hp60c/camera_publisher/depth0/camera_info
  发布:
    /eye_hand_calib/verify/report        (std_msgs/String)  JSON 格式报告
    /eye_hand_calib/verify/debug_image   (sensor_msgs/Image)

使用方式：
  ros2 launch eye_hand_calibration verify_calibration.launch.py
"""

import math
import json
import yaml
import os
from datetime import datetime
from typing import Optional, List, Dict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
import numpy as np
import cv2

from eye_hand_calibration.utils import (
    ros_transform_to_matrix,
    pose_stamped_to_matrix,
    pixel_to_3d,
    get_depth_at_pixel,
    matrix_to_xyz_quat,
    make_transform_matrix,
)


class CalibrationVerifierNode(Node):
    """
    标定质量验证节点。

    在已标定的系统中，将 AprilTag 放在新位置：
    - 若标定精准，PnP 得到的位置与深度图反投影得到的位置应高度一致
    - 误差 < 5mm 为优秀，< 10mm 为合格

    节点会持续运行，每次检测到 AprilTag 时自动输出误差统计。
    """

    def __init__(self):
        super().__init__('calibration_verifier')

        # ---- 参数 ----
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'arm_link4')
        self.declare_parameter('camera_optical_frame', 'camera_depth_optical_frame')
        self.declare_parameter('depth_scale', 1.0)
        self.declare_parameter('depth_offset', 0.0)
        self.declare_parameter('pass_threshold_mm', 5.0)
        self.declare_parameter('publish_debug_image', True)

        self._base_frame = self.get_parameter('base_frame').value
        self._ee_frame = self.get_parameter('ee_frame').value
        self._cam_frame = self.get_parameter('camera_optical_frame').value
        self._depth_scale = self.get_parameter('depth_scale').value
        self._depth_offset_m = self.get_parameter('depth_offset').value
        self._pass_threshold = self.get_parameter('pass_threshold_mm').value
        self._pub_debug = self.get_parameter('publish_debug_image').value

        # 加载深度校正参数
        self._load_depth_correction()

        # ---- 状态 ----
        self._camera_info: Optional[CameraInfo] = None
        self._depth_image: Optional[np.ndarray] = None
        self._bridge = CvBridge()
        self._measurements: List[Dict] = []

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- 订阅 ----
        self.create_subscription(
            PoseStamped,
            '/eye_hand_calib/apriltag/pose',
            self._tag_pose_callback, 10)
        self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self._depth_callback, 10)
        self.create_subscription(
            CameraInfo,
            '/ascamera_hp60c/camera_publisher/depth0/camera_info',
            self._camera_info_callback, 10)

        # ---- 发布 ----
        self._report_pub = self.create_publisher(
            String, '/eye_hand_calib/verify/report', 10)
        if self._pub_debug:
            self._debug_pub = self.create_publisher(
                Image, '/eye_hand_calib/verify/debug_image', 1)

        # 定时统计报告
        self.create_timer(5.0, self._periodic_report)

        self.get_logger().info(
            '标定验证节点启动\n'
            '  请将 AprilTag 放在工作空间内的任意位置\n'
            f'  合格标准：3D 位置误差 < {self._pass_threshold:.0f}mm\n'
            '  查看报告: ros2 topic echo /eye_hand_calib/verify/report'
        )

    # ------------------------------------------------------------------
    # 参数加载
    # ------------------------------------------------------------------

    def _load_depth_correction(self):
        """从标定结果加载深度校正参数。"""
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('eye_hand_calibration')
            path = os.path.join(share, 'config', 'depth_correction.yaml')
            if os.path.exists(path):
                with open(path, 'r') as f:
                    cfg = yaml.safe_load(f)
                dc = cfg.get('depth_correction', {})
                self._depth_scale = float(dc.get('scale', self._depth_scale))
                self._depth_offset_m = float(dc.get('offset', self._depth_offset_m))
                self.get_logger().info(
                    f'已加载深度校正: scale={self._depth_scale:.4f}, '
                    f'offset={self._depth_offset_m*1000:.2f}mm'
                )
        except Exception as e:
            self.get_logger().warn(f'未能加载深度校正参数，使用默认值: {e}')

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------

    def _camera_info_callback(self, msg: CameraInfo):
        if self._camera_info is None:
            self._camera_info = msg

    def _depth_callback(self, msg: Image):
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception:
            pass

    def _tag_pose_callback(self, msg: PoseStamped):
        """每次检测到 AprilTag 时进行一次误差评估。"""
        if self._camera_info is None or self._depth_image is None:
            return

        try:
            self._evaluate(msg)
        except Exception as e:
            self.get_logger().error(f'验证计算失败: {e}')

    # ------------------------------------------------------------------
    # 核心验证逻辑
    # ------------------------------------------------------------------

    def _evaluate(self, tag_pose_msg: PoseStamped):
        """
        对比 PnP 参考值 vs 深度图反投影（校正前/后），输出误差。
        """
        # 1. PnP 参考位置（相机坐标系）
        p = tag_pose_msg.pose.position
        pnp_cam = np.array([p.x, p.y, p.z])

        # 2. 从深度图读取标签中心深度
        fx = self._camera_info.k[0]
        fy = self._camera_info.k[4]
        cx = self._camera_info.k[2]
        cy = self._camera_info.k[5]

        # 标签中心在像素坐标（从 PnP 反投影到像素）
        u_center = int(round(pnp_cam[0] / pnp_cam[2] * fx + cx))
        v_center = int(round(pnp_cam[1] / pnp_cam[2] * fy + cy))

        depth_raw_m = get_depth_at_pixel(self._depth_image, u_center, v_center, window=7)
        if depth_raw_m is None:
            self.get_logger().warn('标签中心深度无效，跳过本次验证')
            return

        # 3. 原始深度反投影
        raw_cam = pixel_to_3d(u_center, v_center, depth_raw_m, fx, fy, cx, cy)

        # 4. 校正后深度反投影
        depth_corr_m = self._depth_scale * depth_raw_m + self._depth_offset_m
        corr_cam = pixel_to_3d(u_center, v_center, depth_corr_m, fx, fy, cx, cy)

        # 5. 变换到 base_link
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self._base_frame,
                tag_pose_msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f'TF 查询失败: {e}', throttle_duration_sec=3.0)
            return

        T_base_cam = ros_transform_to_matrix(tf_stamped.transform)

        def cam_to_base(pt_cam):
            pt_h = np.array([pt_cam[0], pt_cam[1], pt_cam[2], 1.0])
            return (T_base_cam @ pt_h)[:3]

        pnp_base = cam_to_base(pnp_cam)
        raw_base = cam_to_base(raw_cam)
        corr_base = cam_to_base(corr_cam)

        # 6. 计算误差
        err_raw_mm = float(np.linalg.norm(pnp_base - raw_base) * 1000)
        err_corr_mm = float(np.linalg.norm(pnp_base - corr_base) * 1000)
        improvement = err_raw_mm - err_corr_mm

        result = {
            'timestamp': datetime.now().isoformat(),
            'pnp_base_xyz_m': pnp_base.tolist(),
            'raw_base_xyz_m': raw_base.tolist(),
            'corrected_base_xyz_m': corr_base.tolist(),
            'depth_raw_mm': round(depth_raw_m * 1000, 1),
            'depth_corrected_mm': round(depth_corr_m * 1000, 1),
            'error_before_mm': round(err_raw_mm, 2),
            'error_after_mm': round(err_corr_mm, 2),
            'improvement_mm': round(improvement, 2),
            'pass': err_corr_mm < self._pass_threshold,
        }
        self._measurements.append(result)

        # 终端打印
        status = '✓ 合格' if result['pass'] else '✗ 不合格'
        self.get_logger().info(
            f'[验证 #{len(self._measurements)}] {status}\n'
            f'  PnP 参考:  ({pnp_base[0]*100:.1f}, {pnp_base[1]*100:.1f}, {pnp_base[2]*100:.1f}) cm\n'
            f'  校正前误差: {err_raw_mm:.1f} mm\n'
            f'  校正后误差: {err_corr_mm:.1f} mm\n'
            f'  改善量:    {improvement:+.1f} mm'
        )

        # 发布报告
        self._report_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))

    # ------------------------------------------------------------------
    # 定时统计
    # ------------------------------------------------------------------

    def _periodic_report(self):
        """每 5 秒输出累积统计。"""
        if not self._measurements:
            return
        errors = [m['error_after_mm'] for m in self._measurements]
        passed = sum(1 for m in self._measurements if m['pass'])
        self.get_logger().info(
            f'[统计] 共 {len(errors)} 次测量 | '
            f'误差: mean={np.mean(errors):.1f}mm std={np.std(errors):.1f}mm '
            f'max={np.max(errors):.1f}mm | '
            f'合格率: {passed}/{len(errors)} ({100*passed/len(errors):.0f}%)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationVerifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
