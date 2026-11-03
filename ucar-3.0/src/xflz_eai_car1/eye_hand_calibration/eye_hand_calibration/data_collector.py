#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据采集节点

流程：
  1. 等待 /eye_hand_calib/collect/start 服务触发
  2. 依次移动机械臂到预定义的 12 个姿态
  3. 每个姿态：
     a. 等待稳定（settle_time 秒）
     b. 调用 /servo_controller/refresh_positions 读回真实关节角
     c. 从 TF 树获取 base_link → arm_link4 变换
     d. 等待 AprilTag 检测成功（最多 detection_timeout 秒）
     e. 从深度图获取标签中心深度
     f. 保存数据到 session 目录
  4. 所有姿态完成后发布 /eye_hand_calib/collect/done

服务：
  /eye_hand_calib/collect/start  (std_srvs/Trigger) — 开始采集
  /eye_hand_calib/collect/status (std_srvs/Trigger) — 查询状态
"""

import os
import sys
import time
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
from car1_servo_controller_msgs.msg import ServosPosition, ServoPosition
import tf2_ros
import cv2
import numpy as np
from typing import Optional, List, Dict, Any
from cv_bridge import CvBridge
from eye_hand_calibration.utils import (
    ros_transform_to_matrix,
    get_depth_at_pixel,
    pixel_to_3d,
    make_session_dir,
    save_calibration_data,
    matrix_to_xyz_quat,
    quat_to_rotation_matrix,
    make_transform_matrix,
)


class DataCollectorNode(Node):
    """
    机械臂标定数据采集节点。

    自动驱动机械臂遍历预设姿态，在每个姿态同步采集：
      - 机械臂末端 TF 变换（base_link → arm_link4）
      - AprilTag 在相机坐标系的 4×4 变换（来自 apriltag_detector）
      - 深度图中标签中心的深度值
      - RGB 和深度图像（可选保存）
    """

    # IK 求解器路径（硬编码，install后__file__路径会变，不能用__file__推导）
    _IK_PATH = '/home/iflytek/ros2_ws/src/xflz_eai_car1/arm/kinematics/kinematics'

    def __init__(self):
        super().__init__('data_collector')

        # ---- 参数 ----
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('detection_timeout', 5.0)
        self.declare_parameter('min_valid_poses', 8)
        self.declare_parameter('save_images', True)
        self.declare_parameter('data_dir', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ee_frame', 'arm_link4')

        self._settle_time = self.get_parameter('settle_time').value
        self._detection_timeout = self.get_parameter('detection_timeout').value
        self._min_valid = self.get_parameter('min_valid_poses').value
        self._save_images = self.get_parameter('save_images').value
        data_dir_param = self.get_parameter('data_dir').value
        self._base_frame = self.get_parameter('base_frame').value
        self._ee_frame = self.get_parameter('ee_frame').value

        # 加载标定姿态
        self._poses = self._load_poses()
        self.get_logger().info(f'已加载 {len(self._poses)} 个标定姿态')

        # ---- 状态 ----
        self._session_dir = make_session_dir(data_dir_param or None)
        self._records: List[Dict[str, Any]] = []
        self._is_running = False
        self._current_pose_idx = 0

        # 相机数据缓存
        self._rgb_image: Optional[np.ndarray] = None
        self._depth_image: Optional[np.ndarray] = None
        self._camera_info: Optional[CameraInfo] = None
        self._bridge = CvBridge()

        # AprilTag 最新检测（从 apriltag_detector 话题接收）
        self._latest_tag_pose: Optional[Dict] = None
        self._tag_pose_stamp: float = 0.0

        # ---- TF ----
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---- 回调组（服务和状态机分离） ----
        self._srv_cbg = ReentrantCallbackGroup()

        # ---- 订阅 ----
        self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self._rgb_callback, 10)
        self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self._depth_callback, 10)
        self.create_subscription(
            CameraInfo,
            '/ascamera_hp60c/camera_publisher/depth0/camera_info',
            self._camera_info_callback, 10)
        self.create_subscription(
            PoseStamped,
            '/eye_hand_calib/apriltag/pose',
            self._tag_pose_callback, 10)

        # ---- 发布到 /servo_controller（和 arm_controller 一样，确保缓存更新）----
        self._servo_pub = self.create_publisher(
            ServosPosition, '/servo_controller', 10)
        self._status_pub = self.create_publisher(
            String, '/eye_hand_calib/collect/status', 10)

        # ---- 服务客户端 ----
        self._refresh_client = self.create_client(
            Trigger, '/servo_controller/refresh_positions',
            callback_group=self._srv_cbg)
        self._arm_home_client = self.create_client(
            Trigger, '/arm_controller/go_home',
            callback_group=self._srv_cbg)

        # ---- 服务（触发采集） ----
        self.create_service(
            Trigger, '/eye_hand_calib/collect/start',
            self._start_callback,
            callback_group=self._srv_cbg)
        self.create_service(
            Trigger, '/eye_hand_calib/collect/status',
            self._status_callback,
            callback_group=self._srv_cbg)

        self.get_logger().info(
            f'数据采集节点就绪 | 会话目录: {self._session_dir}\n'
            '  触发采集: ros2 service call /eye_hand_calib/collect/start std_srvs/srv/Trigger'
        )

    # ------------------------------------------------------------------
    # 姿态加载
    # ------------------------------------------------------------------

    def _load_poses(self) -> List[List[float]]:
        """从 ROS 参数或配置文件加载标定姿态。"""
        # 尝试从 share 目录加载配置
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('eye_hand_calibration')
            cfg_path = os.path.join(share, 'config', 'calibration_poses.yaml')
            with open(cfg_path, 'r') as f:
                cfg = yaml.safe_load(f)
            raw = cfg['data_collector']['ros__parameters']['poses']
            return [list(p) for p in raw]
        except Exception as e:
            self.get_logger().warn(f'无法加载配置文件，使用内置默认姿态: {e}')
            # 内置默认姿态（兜底）
            return [
                [0.25, 0.00,  0.18, math.pi, 0.0],
                [0.25, 0.00,  0.24, math.pi, 0.0],
                [0.30, 0.00,  0.20, math.pi, 0.0],
                [0.30, 0.00,  0.26, math.pi, 0.0],
                [0.35, 0.00,  0.22, math.pi, 0.0],
                [0.25, -0.07, 0.20, math.pi, 0.3],
                [0.30, -0.07, 0.22, math.pi, 0.3],
                [0.25,  0.07, 0.20, math.pi, -0.3],
                [0.30,  0.07, 0.22, math.pi, -0.3],
                [0.27,  0.00, 0.22, 2.80,    0.0],
                [0.27, -0.05, 0.22, 2.80,    0.2],
                [0.27,  0.05, 0.22, 2.80,   -0.2],
            ]

    # ------------------------------------------------------------------
    # 传感器回调
    # ------------------------------------------------------------------

    def _rgb_callback(self, msg: Image):
        try:
            self._rgb_image = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            pass

    def _depth_callback(self, msg: Image):
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception:
            pass

    def _camera_info_callback(self, msg: CameraInfo):
        self._camera_info = msg

    def _tag_pose_callback(self, msg: PoseStamped):
        """接收 apriltag_detector 发布的标签位姿。"""
        p = msg.pose.position
        o = msg.pose.orientation
        self._latest_tag_pose = {
            'x': p.x, 'y': p.y, 'z': p.z,
            'qx': o.x, 'qy': o.y, 'qz': o.z, 'qw': o.w,
            'frame': msg.header.frame_id,
        }
        self._tag_pose_stamp = time.time()

    # ------------------------------------------------------------------
    # 服务回调
    # ------------------------------------------------------------------

    def _start_callback(self, request, response):
        if self._is_running:
            response.success = False
            response.message = '采集已在运行中，请等待完成'
            return response

        self.get_logger().info('收到启动指令，开始数据采集...')
        self._is_running = True
        self._records = []
        self._current_pose_idx = 0

        # 在新线程中运行，避免阻塞服务响应
        import threading
        t = threading.Thread(target=self._run_collection_loop, daemon=True)
        t.start()

        response.success = True
        response.message = f'开始采集，共 {len(self._poses)} 个姿态'
        return response

    def _status_callback(self, request, response):
        if self._is_running:
            response.success = True
            response.message = (
                f'采集中: {self._current_pose_idx}/{len(self._poses)} '
                f'| 有效: {len(self._records)}'
            )
        else:
            response.success = True
            response.message = (
                f'空闲 | 上次采集: {len(self._records)} 条记录'
            )
        return response

    # ------------------------------------------------------------------
    # 采集主循环
    # ------------------------------------------------------------------

    def _run_collection_loop(self):
        """在独立线程中逐姿态采集标定数据。"""
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'标定数据采集开始 | 会话: {self._session_dir}')
        self.get_logger().info(f'共 {len(self._poses)} 个姿态，最少需要 {self._min_valid} 个成功')
        self.get_logger().info('=' * 60)

        for i, pose in enumerate(self._poses):
            self._current_pose_idx = i + 1
            x, y, z, pitch = pose[:4]   # yaw 不需要传给IK：q1由x/y坐标几何关系自动算出
            self.get_logger().info(
                f'\n[{i+1}/{len(self._poses)}] 移动到姿态: '
                f'x={x:.3f} y={y:.3f} z={z:.3f} pitch={math.degrees(pitch):.1f}°'
            )

            # 1. 发送目标位姿
            if not self._move_arm_to(x, y, z, pitch):
                self.get_logger().warn(f'  → 姿态 {i+1} 移动失败，跳过')
                continue

            # 2. 等待稳定
            self.get_logger().info(f'  → 等待稳定 {self._settle_time}s...')
            time.sleep(self._settle_time)

            # 3. 强制读回真实关节角（照搬 visual_grasp 的异步模式）
            self._refresh_joints_sync()
            time.sleep(0.5)  # 等待 TF 传播（比 visual_grasp 的 0.1s 更保守）

            # 4. 获取末端 TF 变换
            ee_transform = self._get_ee_transform()
            if ee_transform is None:
                self.get_logger().warn(f'  → TF 查询失败，跳过姿态 {i+1}')
                continue

            # 有效性检查：标定时相机始终朝下，arm_link4 的 Z 轴 z 分量应为负
            if ee_transform[2, 2] > -0.3:
                self.get_logger().warn(
                    f'  → TF 异常：arm_link4 Z轴朝上（z={ee_transform[2,2]:.3f}），跳过姿态 {i+1}')
                continue

            # 5. 等待并获取 AprilTag 检测结果
            tag_pose = self._wait_for_detection()
            if tag_pose is None:
                self.get_logger().warn(f'  → AprilTag 检测超时，跳过姿态 {i+1}')
                continue

            # 6. 获取标签中心深度
            depth_m = self._get_tag_center_depth(tag_pose)

            # 7. 保存图像（可选）
            img_path = None
            if self._save_images and self._rgb_image is not None:
                img_path = self._save_image(i)

            # 8. 记录数据（将 tag_pose 字典转换为 4x4 矩阵）
            tag_matrix = None
            if tag_pose:
                q = np.array([tag_pose['qx'], tag_pose['qy'], 
                              tag_pose['qz'], tag_pose['qw']])
                R = quat_to_rotation_matrix(q)
                t = np.array([tag_pose['x'], tag_pose['y'], tag_pose['z']])
                tag_matrix = make_transform_matrix(R, t)

            record = {
                'pose_idx': i,
                'pose_cmd': {'x': x, 'y': y, 'z': z, 'pitch': pitch},
                'ee_transform': ee_transform,
                'tag_in_camera': tag_matrix,  # 改为矩阵
                'depth_raw_m': depth_m,
                'depth_pnp_m': tag_pose.get('z') if tag_pose else None,  # PnP深度
                'pixel_uv': None,  # 可选：标签中心像素坐标
                'image_path': img_path,
                'timestamp': time.time(),
            }
            self._records.append(record)
            self.get_logger().info(
                f'  ✓ 姿态 {i+1} 记录成功 | tag z={tag_pose["z"]:.4f}m '
                f'depth={depth_m:.4f}m' if depth_m else
                f'  ✓ 姿态 {i+1} 记录成功 | tag z={tag_pose["z"]:.4f}m'
            )

            self._publish_status(f'{i+1}/{len(self._poses)} 完成')

        # 采集完成
        self._is_running = False
        n = len(self._records)
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'采集完成: {n}/{len(self._poses)} 个姿态成功')

        if n < self._min_valid:
            self.get_logger().error(
                f'有效数据不足（{n} < {self._min_valid}），标定可能不准确！'
            )
        else:
            # 保存数据
            save_path = self._save_records()
            self.get_logger().info(f'数据已保存: {save_path}')
            self.get_logger().info(
                f'\n下一步：\n'
                f'  ros2 launch eye_hand_calibration compute_calibration.launch.py '
                f'data_dir:={self._session_dir}'
            )
        self.get_logger().info('=' * 60)

    # ------------------------------------------------------------------
    # 辅助方法
    # ------------------------------------------------------------------

    def _move_arm_to(self, x: float, y: float, z: float, pitch: float) -> bool:
        """
        直接发底层舵机话题移动机械臂（绕过 arm_controller）。
        IK 求解器由 x/y 坐标自动算出 J1（偏航角），无需外部传入 yaw。
        """
        # 使用硬编码绝对路径导入 IK 求解器
        # （用 __file__ 推导在 colcon install 后路径会变，不可靠）
        try:
            if self._IK_PATH not in sys.path:
                sys.path.insert(0, self._IK_PATH)
            from ik_analytical import solve_analytical_ik_auto_pitch, Point3D
        except ImportError as e:
            self.get_logger().error(f'IK 求解器导入失败: {e}  路径: {self._IK_PATH}')
            return False

        # IK 求解
        solution = solve_analytical_ik_auto_pitch(
            Point3D(x, y, z),
            preferred_pitch_rad=pitch
        )

        if solution.error:
            self.get_logger().error(f'IK 求解失败: {solution.error}')
            return False

        # 关节角度（弧度）
        joints_rad = [solution.q1, solution.q2, solution.q3, solution.q4, solution.q5]

        # 角度 → PWM（与 interactive_ik_test.py 同款转换）
        # PWM = 1500 + 角度(度) * (2000/240)
        # 关节角度（弧度）
        joints_rad = [solution.q1, solution.q2, solution.q3, solution.q4, solution.q5]
        # 构造消息（先设置基本字段，再填充数组）
        msg = ServosPosition()
        msg.duration = 2.0
        msg.position_unit = 'rad'
        msg.position = []  # 初始化数组
        for servo_id, rad in enumerate(joints_rad):
            servo = ServoPosition()
            servo.id = servo_id
            servo.position = float(rad)
            msg.position.append(servo)

        # 发布到 /servo_controller（缓存立刻更新）
        self._servo_pub.publish(msg)
        self.get_logger().info(
            f'  → IK: J1={math.degrees(joints_rad[0]):.1f}° J2={math.degrees(joints_rad[1]):.1f}° '
            f'J3={math.degrees(joints_rad[2]):.1f}° J4={math.degrees(joints_rad[3]):.1f}° J5={math.degrees(joints_rad[4]):.1f}°')

        # 等待运动完成
        time.sleep(2.5)
        return True

    def _refresh_joints_sync(self):
        """
        同步刷新舵机位置（照搬 visual_grasp）。
        使用 call_async + 轮询 done()，不阻塞 executor。
        """
        if not self._refresh_client.service_is_ready():
            self.get_logger().warn('refresh_positions 服务不可用，跳过读回')
            return

        future = self._refresh_client.call_async(Trigger.Request())
        deadline = time.time() + 1.5  # 1.5s 超时

        while time.time() < deadline:
            if future.done():
                try:
                    result = future.result()
                    if result.success:
                        self.get_logger().debug('refresh_positions 成功')
                    else:
                        self.get_logger().warn(f'refresh_positions 失败: {result.message}')
                except Exception as e:
                    self.get_logger().warn(f'refresh_positions 异常: {e}')
                return
            time.sleep(0.02)  # 轮询间隔

        self.get_logger().warn('refresh_positions 超时，使用缓存值')

    def _get_ee_transform(self) -> Optional[np.ndarray]:
        """从 TF 树获取 base_link → arm_link4 变换，返回 4×4 矩阵。"""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, self._ee_frame,
                rclpy.time.Time(), timeout=Duration(seconds=2.0),
            )
            return ros_transform_to_matrix(tf.transform)
        except Exception as e:
            self.get_logger().warn(f'TF 查询失败 ({self._base_frame}→{self._ee_frame}): {e}')
            return None


    def _wait_for_detection(self) -> Optional[Dict]:
        """等待最新的 AprilTag 检测结果（最多 detection_timeout 秒）。"""
        deadline = time.time() + self._detection_timeout
        # 清空过期数据（超过 0.5s 的视为过期）
        if self._tag_pose_stamp > 0 and (time.time() - self._tag_pose_stamp) > 0.5:
            self._latest_tag_pose = None
            self._tag_pose_stamp = 0.0

        while time.time() < deadline:
            if (self._latest_tag_pose is not None and
                    time.time() - self._tag_pose_stamp < 1.0):
                return dict(self._latest_tag_pose)
            time.sleep(0.05)
        return None

    def _get_tag_center_depth(self, tag_pose: Dict) -> Optional[float]:
        """用标签的 Z 坐标作为深度参考（PnP 解算出的深度）。"""
        # tag_pose['z'] 是相机光轴方向的距离（米），即深度
        # 同时用深度图做交叉验证
        if self._depth_image is None or self._camera_info is None:
            return tag_pose.get('z')

        K = self._camera_info.k
        fx, fy, cx, cy = K[0], K[4], K[2], K[5]
        # 标签中心在像素坐标（从 tag_pose 的 x,y,z 反投影）
        z = tag_pose['z']
        if z <= 0:
            return None
        u = int(tag_pose['x'] * fx / z + cx)
        v = int(tag_pose['y'] * fy / z + cy)

        depth_from_image = get_depth_at_pixel(self._depth_image, u, v, window=11)
        return depth_from_image if depth_from_image is not None else z

    def _save_image(self, idx: int) -> Optional[str]:
        """保存当前帧 RGB 图像。"""
        try:
            img_dir = os.path.join(self._session_dir, 'images')
            os.makedirs(img_dir, exist_ok=True)
            path = os.path.join(img_dir, f'rgb_{idx:02d}.png')
            cv2.imwrite(path, self._rgb_image)
            # 同时保存深度图
            if self._depth_image is not None:
                depth_vis = cv2.normalize(
                    self._depth_image.astype(np.float32),
                    None, 0, 255, cv2.NORM_MINMAX
                ).astype(np.uint8)
                cv2.imwrite(
                    os.path.join(img_dir, f'depth_{idx:02d}.png'),
                    cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                )
            return path
        except Exception as e:
            self.get_logger().warn(f'图像保存失败: {e}')
            return None

    def _save_records(self) -> str:
        """将采集记录保存为 YAML 文件。"""
        save_calibration_data(self._session_dir, self._records)
        return self._session_dir


    def _publish_status(self, msg: str):
        m = String()
        m.data = msg
        self._status_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = DataCollectorNode()

    # ReentrantCallbackGroup 需要 MultiThreadedExecutor
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
