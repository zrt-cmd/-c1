#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag 检测节点

订阅 HP60C 相机 RGB 图像，使用 pupil-apriltags 库检测 TAG36H11 标签，
通过 PnP 解算 3D 位姿并发布。

话题：
  订阅:
    /ascamera_hp60c/camera_publisher/rgb0/image         (sensor_msgs/Image)
    /ascamera_hp60c/camera_publisher/depth0/camera_info (sensor_msgs/CameraInfo)
  发布:
    /eye_hand_calib/apriltag/pose        (geometry_msgs/PoseStamped)
    /eye_hand_calib/apriltag/debug_image (sensor_msgs/Image)

使用方式：
  ros2 run eye_hand_calibration apriltag_detector
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, Tuple

try:
    from pupil_apriltags import Detector as ApriltagDetector
    _APRILTAG_BACKEND = 'pupil-apriltags'
except ImportError:
    try:
        from dt_apriltags import Detector as ApriltagDetector
        _APRILTAG_BACKEND = 'dt-apriltags'
    except ImportError:
        _APRILTAG_BACKEND = None
        ApriltagDetector = None

from eye_hand_calibration.utils import (
    rotation_matrix_to_quat,
    make_transform_matrix,
)


class AprilTagDetectorNode(Node):
    """
    ROS2 节点：检测 AprilTag TAG36H11 并发布位姿。

    内参来自 /camera_info 话题，无需手动填写。
    检测到标签后通过 PnP 求解相机坐标系下的 4×4 变换矩阵，
    发布为 PoseStamped（frame_id = camera_depth_optical_frame）。
    """

    def __init__(self):
        super().__init__('apriltag_detector')

        # ---- 参数声明 ----
        self.declare_parameter('rgb_topic',
                               '/ascamera_hp60c/camera_publisher/rgb0/image')
        self.declare_parameter('camera_info_topic',
                               '/ascamera_hp60c/camera_publisher/depth0/camera_info')
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_id', 3)
        # tag_size = 编码区域边长（米），本项目为 25mm
        self.declare_parameter('tag_size', 0.025)
        self.declare_parameter('max_hamming', 0)
        self.declare_parameter('decimate', 2.0)
        self.declare_parameter('blur', 0.0)
        self.declare_parameter('refine_edges', True)
        self.declare_parameter('nthreads', 4)
        self.declare_parameter('publish_debug_image', True)

        # ---- 读取参数 ----
        rgb_topic = self.get_parameter('rgb_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self._tag_family = self.get_parameter('tag_family').value
        self._tag_id = self.get_parameter('tag_id').value
        self._tag_size = self.get_parameter('tag_size').value
        self._max_hamming = self.get_parameter('max_hamming').value
        self._decimate = self.get_parameter('decimate').value
        self._blur = self.get_parameter('blur').value
        self._refine_edges = self.get_parameter('refine_edges').value
        self._nthreads = self.get_parameter('nthreads').value
        self._pub_debug = self.get_parameter('publish_debug_image').value

        # ---- 状态 ----
        self._camera_info: Optional[CameraInfo] = None
        self._fx = self._fy = self._cx = self._cy = None
        self._dist_coeffs = np.zeros((4, 1))
        self._detector = None
        self._bridge = CvBridge()

        # 最新检测结果，供 data_collector 查询
        self._latest_pose: Optional[PoseStamped] = None
        self._latest_corners: Optional[np.ndarray] = None
        self._detection_count = 0

        # ---- 初始化检测器 ----
        self._init_detector()

        # ---- 订阅 ----
        self.create_subscription(CameraInfo, info_topic,
                                 self._camera_info_callback, 10)
        self.create_subscription(Image, rgb_topic,
                                 self._image_callback, 10)

        # ---- 发布 ----
        self._pose_pub = self.create_publisher(
            PoseStamped, '/eye_hand_calib/apriltag/pose', 10)
        self._detected_pub = self.create_publisher(
            Bool, '/eye_hand_calib/apriltag/detected', 10)
        if self._pub_debug:
            self._debug_pub = self.create_publisher(
                Image, '/eye_hand_calib/apriltag/debug_image', 1)
        else:
            self._debug_pub = None

        self.get_logger().info(
            f'AprilTagDetector 启动 | 后端={_APRILTAG_BACKEND} '
            f'| 族={self._tag_family} | ID={self._tag_id} '
            f'| tag_size={self._tag_size*1000:.0f}mm'
        )

    # ------------------------------------------------------------------
    # 初始化检测器
    # ------------------------------------------------------------------

    def _init_detector(self):
        """根据可用后端初始化 AprilTag 检测器。"""
        if _APRILTAG_BACKEND is None or ApriltagDetector is None:
            self.get_logger().warn(
                '==================================================\n'
                '  AprilTag 库未安装！\n'
                '  节点将以降级模式运行（无检测功能）。\n'
                '\n'
                '  安装方法：\n'
                '    sudo pip3 install pupil-apriltags\n'
                '  或在当前 Python 环境：\n'
                '    pip3 install pupil-apriltags\n'
                '==================================================\n'
            )
            self._detector = None
            return

        opts = dict(
            families=self._tag_family,
            nthreads=self._nthreads,
            quad_decimate=self._decimate,
            quad_sigma=self._blur,
            refine_edges=1 if self._refine_edges else 0,
            decode_sharpening=0.25,
        )

        try:
            self._detector = ApriltagDetector(**opts)
            self.get_logger().info(f'AprilTag 检测器初始化成功 (后端: {_APRILTAG_BACKEND})')
        except Exception as e:
            self.get_logger().error(f'检测器初始化失败: {e}')
            self._detector = None

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------

    def _camera_info_callback(self, msg: CameraInfo):
        """保存相机内参（只需接收一次）。"""
        if self._camera_info is not None:
            return
        self._camera_info = msg
        K = msg.k
        self._fx = K[0]
        self._fy = K[4]
        self._cx = K[2]
        self._cy = K[5]
        # 畸变系数
        d = msg.d
        if len(d) >= 4:
            self._dist_coeffs = np.array(d[:4], dtype=np.float64).reshape(-1, 1)
        self.get_logger().info(
            f'内参已接收: fx={self._fx:.1f} fy={self._fy:.1f} '
            f'cx={self._cx:.1f} cy={self._cy:.1f}'
        )

    def _image_callback(self, msg: Image):
        """接收 RGB 图像，运行检测，发布结果。"""
        if self._detector is None or self._camera_info is None:
            return

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {e}', throttle_duration_sec=2.0)
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        pose, corners = self._detect(gray)

        detected_msg = Bool()
        detected_msg.data = (pose is not None)
        self._detected_pub.publish(detected_msg)

        if pose is not None:
            pose.header = msg.header
            pose.header.frame_id = 'camera_depth_optical_frame'
            self._latest_pose = pose
            self._latest_corners = corners
            self._detection_count += 1
            self._pose_pub.publish(pose)

        if self._debug_pub is not None:
            debug_img = self._draw_debug(bgr, pose, corners)
            try:
                self._debug_pub.publish(
                    self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8'))
            except Exception:
                pass

    # ------------------------------------------------------------------
    # 检测核心
    # ------------------------------------------------------------------

    def _detect(self, gray: np.ndarray
                ) -> Tuple[Optional[PoseStamped], Optional[np.ndarray]]:
        """
        在灰度图中检测 AprilTag，PnP 解算位姿。

        Returns:
            (PoseStamped | None, corners | None)
        """
        try:
            results = self._detector.detect(gray)
        except Exception as e:
            self.get_logger().error(f'检测异常: {e}', throttle_duration_sec=2.0)
            return None, None

        # 筛选目标 ID，汉明距离过滤
        for det in results:
            tag_id = getattr(det, 'tag_id', None) or getattr(det, 'id', None)
            hamming = getattr(det, 'hamming', 0)

            if tag_id != self._tag_id:
                continue
            if hamming > self._max_hamming:
                self.get_logger().debug(
                    f'Tag {tag_id} 汉明距离={hamming} 超过阈值，跳过')
                continue

            corners = det.corners.astype(np.float32)  # shape (4, 2)
            pose = self._solve_pnp(corners)
            return pose, corners

        return None, None

    def _solve_pnp(self, corners: np.ndarray) -> Optional[PoseStamped]:
        """
        使用 solvePnP 从角点计算 AprilTag 在相机坐标系下的位姿。

        标签的 3D 角点定义（TAG 坐标系，单位米，顺序与 apriltag 输出一致）：
          角点 0: (-s/2,  s/2, 0)  左上
          角点 1: ( s/2,  s/2, 0)  右上
          角点 2: ( s/2, -s/2, 0)  右下
          角点 3: (-s/2, -s/2, 0)  左下
        其中 s = tag_size = 0.025m
        """
        s = self._tag_size / 2.0
        obj_pts = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0],
        ], dtype=np.float64)

        K = np.array([
            [self._fx,       0, self._cx],
            [      0, self._fy, self._cy],
            [      0,       0,        1],
        ], dtype=np.float64)

        img_pts = corners.astype(np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, K, self._dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            return None

        R, _ = cv2.Rodrigues(rvec)
        q = rotation_matrix_to_quat(R)
        t = tvec.flatten()

        pose = PoseStamped()
        pose.pose.position.x = float(t[0])
        pose.pose.position.y = float(t[1])
        pose.pose.position.z = float(t[2])
        pose.pose.orientation.x = float(q[0])
        pose.pose.orientation.y = float(q[1])
        pose.pose.orientation.z = float(q[2])
        pose.pose.orientation.w = float(q[3])
        return pose

    # ------------------------------------------------------------------
    # 调试可视化
    # ------------------------------------------------------------------

    def _draw_debug(self, bgr: np.ndarray,
                    pose: Optional[PoseStamped],
                    corners: Optional[np.ndarray]) -> np.ndarray:
        """在图像上绘制检测结果。"""
        img = bgr.copy()

        if corners is not None:
            pts = corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(img, [pts], isClosed=True,
                          color=(0, 255, 0), thickness=2)
            # 绘制角点编号
            for i, (u, v) in enumerate(corners.astype(int)):
                cv2.circle(img, (u, v), 5, (0, 0, 255), -1)
                cv2.putText(img, str(i), (u + 6, v - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        if pose is not None:
            p = pose.pose.position
            info = (f'x={p.x*100:.1f}cm '
                    f'y={p.y*100:.1f}cm '
                    f'z={p.z*100:.1f}cm')
            cv2.putText(img, info, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(img, f'Tag ID={self._tag_id} #{self._detection_count}',
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
        else:
            cv2.putText(img, 'TAG NOT FOUND', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        return img

    # ------------------------------------------------------------------
    # 对外接口（供 data_collector 调用）
    # ------------------------------------------------------------------

    @property
    def latest_pose(self) -> Optional[PoseStamped]:
        """最新检测到的 AprilTag 位姿（相机坐标系）。"""
        return self._latest_pose

    def clear_latest(self):
        """清除最新检测结果，用于等待新一帧。"""
        self._latest_pose = None
        self._latest_corners = None

    def is_ready(self) -> bool:
        """相机内参是否已接收。"""
        return self._camera_info is not None


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
