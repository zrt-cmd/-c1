#!/usr/bin/env python3
"""
HSV 颜色检测 + 深度定位节点
从 RGB 图像中检测红色物体，通过深度图获取 3D 坐标，
再通过 TF 变换到 base_link 坐标系。

订阅话题：
  /camera/image        - RGB 图像 (sensor_msgs/Image)
  /camera/depth_image   - 深度图 (sensor_msgs/Image, float32 编码)
  /camera/camera_info  - 相机内参 (sensor_msgs/CameraInfo)

发布话题：
  /detected_object_pose   - 物体在 base_link 下的位姿 (geometry_msgs/PoseStamped)
  /detected_object_marker - 可视化标记 (visualization_msgs/Marker)
  /detection_image        - 检测结果图 (sensor_msgs/Image)
"""
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 相机内参（从 camera_info 获取）
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_info_received = False

        # 深度图缓存
        self.latest_depth = None

        # HSV 阈值（Gazebo 中红色物体: RGB ~0.8-0.9, 0.1, 0.1）
        # 红色在 HSV 空间跨越 0° 和 180°，需要两个区间
        self.hsv_ranges = [
            (np.array([0, 100, 80]),   np.array([10, 255, 255])),   # H: 0-10
            (np.array([170, 100, 80]), np.array([180, 255, 255])),  # H: 170-180
        ]

        # 检测参数
        self.min_contour_area = 100      # 最小轮廓面积（像素）
        self.depth_region_size = 5       # 深度采样区域半径（像素）

        # QoS: 传感器数据用 BEST_EFFORT
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 订阅
        self.color_sub = self.create_subscription(
            Image, '/camera/image', self.color_callback, sensor_qos)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, sensor_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.info_callback, sensor_qos)

        # 发布
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/detected_object_marker', 10)
        self.image_pub = self.create_publisher(Image, '/detection_image', 10)

        self.get_logger().info('物体检测节点已启动，等待相机数据...')

    def info_callback(self, msg):
        """接收相机内参"""
        if self.camera_info_received:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True
        self.get_logger().info(
            f'相机内参: fx={self.fx:.1f}, fy={self.fy:.1f}, '
            f'cx={self.cx:.1f}, cy={self.cy:.1f}')

    def depth_callback(self, msg):
        """缓存深度图"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().warn(f'深度图转换失败: {e}')

    def color_callback(self, msg):
        """RGB 图像回调：执行检测"""
        if not self.camera_info_received:
            return
        if self.latest_depth is None:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'RGB 图像转换失败: {e}')
            return

        # HSV 颜色分割
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self.hsv_ranges:
            mask |= cv2.inRange(hsv, lower, upper)

        # 形态学处理：去噪声
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 轮廓检测
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return

        # 取最大轮廓
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)

        if area < self.min_contour_area:
            return

        # 轮廓中心
        M = cv2.moments(largest)
        if M['m00'] == 0:
            return
        u = int(M['m10'] / M['m00'])  # 像素 x
        v = int(M['m01'] / M['m00'])  # 像素 y

        # 从深度图获取距离（取区域中值）
        h, w = self.latest_depth.shape[:2]
        r = self.depth_region_size
        u_min = max(0, u - r)
        u_max = min(w, u + r + 1)
        v_min = max(0, v - r)
        v_max = min(h, v + r + 1)
        region = self.latest_depth[v_min:v_max, u_min:u_max].astype(np.float32)

        # 过滤无效深度值
        valid = region[region > 0.1]
        if len(valid) < 3:
            return

        z_cam = float(np.median(valid))

        # 像素坐标 → 光学坐标系 3D 坐标（标准 ROS 约定：+Z 沿光轴，+X 右，+Y 下）
        x_optical = (u - self.cx) * z_cam / self.fx
        y_optical = (v - self.cy) * z_cam / self.fy
        z_optical = z_cam

        self.get_logger().info(
            f'检测: 像素({u}, {v}), 深度={z_optical:.3f}m, '
            f'光学坐标=({x_optical:.3f}, {y_optical:.3f}, {z_optical:.3f})')

        # TF 变换: camera_depth_optical_frame → base_link
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_depth_optical_frame',
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
        except TransformException as e:
            self.get_logger().warn(f'TF 查询失败: {e}')
            return

        # 3D 点变换
        t = transform.transform.translation
        r = transform.transform.rotation

        # 四元数 → 旋转矩阵
        qx, qy, qz, qw = r.x, r.y, r.z, r.w
        rot = self._quat_to_matrix(qx, qy, qz, qw)

        # camera_depth_optical_frame 下的点（已经是光学坐标系）
        pt_optical = np.array([x_optical, y_optical, z_optical])
        # 变换到 base_link
        pt_base = rot @ pt_optical + np.array([t.x, t.y, t.z])

        self.get_logger().info(
            f'TF变换: optical→base, 平移=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}), '
            f'结果=({pt_base[0]:.3f}, {pt_base[1]:.3f}, {pt_base[2]:.3f})')

        # 发布位姿
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = pt_base[0]
        pose.pose.position.y = pt_base[1]
        pose.pose.position.z = pt_base[2]
        self.pose_pub.publish(pose)

        # 发布可视化标记
        marker = Marker()
        marker.header = pose.header
        marker.ns = 'detected_object'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = pose.pose.position
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

        # 发布检测结果图（调试用）
        cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
        cv2.circle(frame, (u, v), 5, (0, 0, 255), -1)
        text = f'({pt_base[0]:.3f}, {pt_base[1]:.3f}, {pt_base[2]:.3f})'
        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        try:
            det_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            det_img.header = msg.header
            self.image_pub.publish(det_img)
        except Exception:
            pass

    @staticmethod
    def _quat_to_matrix(qx, qy, qz, qw):
        """四元数转 3x3 旋转矩阵"""
        return np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ])


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
