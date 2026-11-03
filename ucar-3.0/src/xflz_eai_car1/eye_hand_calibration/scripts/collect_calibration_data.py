#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式数据采集脚本

提供命令行交互，引导用户完成标定数据采集：
  1. 检查系统状态（相机、机械臂、TF）
  2. 触发 data_collector 节点开始采集
  3. 实时显示进度
  4. 采集完成后自动调用标定计算

使用方式：
  ros2 run eye_hand_calibration collect_calibration_data
  # 或直接：python3 scripts/collect_calibration_data.py
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('collect_calibration_data_cli')
    logger = node.get_logger()

    print('=' * 60)
    print('  深度相机手眼标定 — 数据采集工具')
    print('=' * 60)
    print()
    print('【准备工作】')
    print('  1. 将 AprilTag TAG36H11-3 固定在桌面')
    print('     - 位置：机械臂正前方约 25-30cm')
    print('     - 保证光照充足，避免反光')
    print('  2. 确认系统已启动（bringup + arm_controller + depth_camera）')
    print()

    input('  准备好后按 Enter 开始采集...')
    print()

    # 等待 data_collector 节点就绪
    client = node.create_client(Trigger, '/eye_hand_calib/collect/start')
    logger.info('等待数据采集节点...')

    timeout = 10.0
    start = time.time()
    while not client.wait_for_service(timeout_sec=1.0):
        if time.time() - start > timeout:
            print('\n[错误] 数据采集节点未启动！')
            print('  请先运行: ros2 launch eye_hand_calibration collect_data.launch.py')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        print('  等待节点...')

    # 订阅进度话题
    progress_msgs = []

    def status_cb(msg):
        progress_msgs.append(msg.data)
        print(f'  [{time.strftime("%H:%M:%S")}] {msg.data}')

    node.create_subscription(String, '/eye_hand_calib/collect/status', status_cb, 10)

    # 触发采集
    print('正在触发数据采集...')
    req = Trigger.Request()
    future = client.call_async(req)

    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if future.result() is None:
        print('[错误] 服务调用超时')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    resp = future.result()
    if not resp.success:
        print(f'[错误] 采集启动失败: {resp.message}')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f'[成功] {resp.message}')
    print()
    print('机械臂开始移动，请勿触碰...')
    print('（等待采集完成，共 12 个姿态，约 5-10 分钟）')
    print()

    # 等待完成（监听状态话题）
    done = False
    start = time.time()
    max_wait = 600  # 最多等 10 分钟

    while not done and (time.time() - start) < max_wait:
        rclpy.spin_once(node, timeout_sec=0.5)
        for msg in progress_msgs:
            if 'DONE' in msg or '完成' in msg or '失败' in msg or 'FAILED' in msg:
                done = True
                break
        progress_msgs.clear()

    print()
    if done:
        print('=' * 60)
        print('  数据采集完成！')
        print()
        print('  下一步：运行标定计算')
        print('  ros2 launch eye_hand_calibration compute_calibration.launch.py')
        print('=' * 60)
    else:
        print('[超时] 采集未在预期时间内完成，请检查日志')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
