#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PWM值 → TCP位姿计算工具

从舵机PWM值反算机械臂TCP位姿（包含位置和姿态RPY角）
"""
import sys
import math
import numpy as np

sys.path.insert(0, '/home/iflytek/ros2_ws/src/xflz_eai_car1/arm/kinematics/kinematics')

from fk import compute_fk
from config import Point3D
from transform import pulse2angle  # 使用标准的PWM转换函数


# 移除自定义的 pwm_to_rad，改用 transform.pulse2angle
def rotation_matrix_to_rpy(R: np.ndarray) -> tuple:
    """
    旋转矩阵 → Roll-Pitch-Yaw (ZYX欧拉角)

    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    Args:
        R: 3x3 旋转矩阵

    Returns:
        (roll, pitch, yaw) in radians
    """
    # 提取pitch (β)
    sy = math.sqrt(R[0, 0]**2 + R[1, 0]**2)

    singular = sy < 1e-6  # 万向锁检测

    if not singular:
        # 正常情况
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        # 万向锁（pitch ≈ ±90°）
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0

    return roll, pitch, yaw


def main():
    print("=" * 70)
    print("PWM值 → TCP位姿计算（位置 + 姿态RPY）")
    print("=" * 70)

    if len(sys.argv) == 6:
        # 命令行参数模式
        pwm_values = [int(sys.argv[i]) for i in range(1, 6)]
    else:
        # 交互式输入
        print("\n请输入5个舵机的PWM值（ID 0-4）:")
        print("（或按Ctrl+C退出）")
        pwm_values = []
        for i in range(5):
            while True:
                try:
                    pwm = int(input(f"  J{i+1} (ID{i}) PWM值: "))
                    if 500 <= pwm <= 2500:
                        pwm_values.append(pwm)
                        break
                    else:
                        print("    ⚠️ PWM值应在 500-2500 范围内")
                except ValueError:
                    print("    ⚠️ 请输入整数")
                except KeyboardInterrupt:
                    print("\n已取消")
                    sys.exit(0)

    # PWM → 弧度（使用标准转换函数）
    joint_angles = {
        'arm_joint1': pulse2angle(0, pwm_values[0]),
        'arm_joint2': pulse2angle(1, pwm_values[1]),
        'arm_joint3': pulse2angle(2, pwm_values[2]),
        'arm_joint4': pulse2angle(3, pwm_values[3]),
        'arm_joint5': pulse2angle(4, pwm_values[4]),
    }

    # 计算FK
    tcp_pos, T_matrix = compute_fk(joint_angles)

    # 提取旋转矩阵
    R = T_matrix[:3, :3]

    # 计算RPY角
    roll_rad, pitch_rad, yaw_rad = rotation_matrix_to_rpy(R)
    roll_deg = math.degrees(roll_rad)
    pitch_deg = math.degrees(pitch_rad)
    yaw_deg = math.degrees(yaw_rad)

    print("\n" + "=" * 70)
    print("✓ 正向运动学计算完成")
    print("=" * 70)

    print("\n【输入关节角度】")
    for i, pwm in enumerate(pwm_values):
        rad = joint_angles[f'arm_joint{i+1}']
        deg = math.degrees(rad)
        print(f"  J{i+1} (ID{i}): {pwm:4d} PWM  →  {rad:+7.4f} rad  ({deg:+7.2f}°)")

    print("\n【TCP位置】(base_footprint坐标系)")
    print(f"  X: {tcp_pos.x:+.6f} m  ({tcp_pos.x*1000:+8.2f} mm)")
    print(f"  Y: {tcp_pos.y:+.6f} m  ({tcp_pos.y*1000:+8.2f} mm)")
    print(f"  Z: {tcp_pos.z:+.6f} m  ({tcp_pos.z*1000:+8.2f} mm)")

    print("\n【TCP姿态】(Roll-Pitch-Yaw)")
    print(f"  Roll  (r): {roll_rad:+7.4f} rad  ({roll_deg:+8.2f}°)")
    print(f"  Pitch (p): {pitch_rad:+7.4f} rad  ({pitch_deg:+8.2f}°)")
    print(f"  Yaw   (y): {yaw_rad:+7.4f} rad  ({yaw_deg:+8.2f}°)")

    print("\n【姿态解读】")
    # Roll解读
    if abs(roll_deg) < 10:
        roll_desc = "接近水平"
    elif abs(roll_deg - 180) < 10 or abs(roll_deg + 180) < 10:
        roll_desc = "翻转180°"
    elif abs(roll_deg - 90) < 10:
        roll_desc = "向右翻转90°"
    elif abs(roll_deg + 90) < 10:
        roll_desc = "向左翻转90°"
    else:
        roll_desc = f"倾斜{roll_deg:.1f}°"

    # Pitch解读
    if abs(pitch_deg) < 10:
        pitch_desc = "水平向前"
    elif abs(pitch_deg - 90) < 10:
        pitch_desc = "垂直向上"
    elif abs(pitch_deg + 90) < 10:
        pitch_desc = "垂直向下"
    else:
        if pitch_deg > 0:
            pitch_desc = f"向上抬{pitch_deg:.1f}°"
        else:
            pitch_desc = f"向下俯{abs(pitch_deg):.1f}°"

    # Yaw解读
    if abs(yaw_deg) < 10:
        yaw_desc = "正前方"
    elif abs(yaw_deg - 90) < 10:
        yaw_desc = "向左90°"
    elif abs(yaw_deg + 90) < 10:
        yaw_desc = "向右90°"
    elif abs(abs(yaw_deg) - 180) < 10:
        yaw_desc = "向后180°"
    else:
        yaw_desc = f"偏航{yaw_deg:.1f}°"

    print(f"  Roll:  {roll_desc}")
    print(f"  Pitch: {pitch_desc}")
    print(f"  Yaw:   {yaw_desc}")

    # 显示Z轴方向（相机/工具指向）
    tcp_z_axis = R[:, 2]
    print("\n【TCP Z轴方向】(工具/相机朝向)")
    print(f"  向量: ({tcp_z_axis[0]:+.3f}, {tcp_z_axis[1]:+.3f}, {tcp_z_axis[2]:+.3f})")

    # ===== 计算俯仰角 =====
    # interactive_ik_test.py 的约定：
    #   0° = 水平向前（Z轴→+X）
    #   90° = 垂直向上（Z轴→+Z）
    #   180° = 垂直向下（Z轴→-Z）
    #
    # 从 TCP Z轴方向向量计算：
    horizontal_proj = math.sqrt(tcp_z_axis[0]**2 + tcp_z_axis[1]**2)
    elevation = math.atan2(tcp_z_axis[2], horizontal_proj)  # -90° ~ +90°

    # 转换为 0~180° 范围：
    # elevation = +90° (垂直向上) → pitch = 90°
    # elevation = 0° (水平) → pitch = 0° (但这个公式给的是90°)
    # elevation = -90° (垂直向下) → pitch = 180°
    #
    # 实际上应该是：pitch = 90° - elevation
    # 但实测发现需要反向：pitch = 180° - (90° - elevation) = 90° + elevation
    # 不对...让我用实际数据验证：
    #
    # 实际案例：Z轴=(+0.617, 0, -0.787)
    #   elevation = atan2(-0.787, 0.617) = -51.87°
    #   我之前算的: pitch = 90° - (-51.87°) = 141.87°
    #   实际应该是: 38.12°
    #   关系: 180° - 141.87° = 38.13° ✓
    #
    # 所以正确的公式是：pitch = 180° - (90° - elevation) = 90° + elevation
    # 不对，应该是：pitch = 90° + elevation（当elevation为负时）
    # 实测：elevation = -51.87°, pitch = 90° + (-51.87°) = 38.13° ✓

    pitch_rad = math.pi / 2 + elevation  # pitch从水平向前开始计数
    pitch_deg = math.degrees(pitch_rad)

    # calibration_poses.yaml 使用的是另一种约定：
    # 180° = 垂直向下，0° = 垂直向上，90° = 水平
    # 需要转换：pitch_calib = 180° - pitch
    pitch_calib_deg = 180.0 - pitch_deg
    pitch_calib_rad = math.radians(pitch_calib_deg)

    tcp_pitch_for_ik = pitch_rad
    tcp_pitch_for_ik_deg = pitch_deg

    print("\n" + "=" * 70)
    print("【用于 calibration_poses.yaml 的位姿参数】")
    print("=" * 70)
    print(f"  俯仰角 (pitch): {pitch_calib_deg:7.2f}° ({pitch_calib_rad:7.5f} rad)")
    print(f"  说明: 0° = 垂直向上, 90° = 水平, 180° = 垂直向下")
    print()
    print("  复现此位姿的命令：")
    print(f"    python3 src/xflz_eai_car1/arm/kinematics/kinematics/interactive_ik_test.py --direct")
    print(f"    X 坐标: {tcp_pos.x:.4f}")
    print(f"    Y 坐标: {tcp_pos.y:.4f}")
    print(f"    Z 坐标: {tcp_pos.z:.4f}")
    print(f"    俯仰角: {pitch_calib_deg:.2f}")
    print()
    print("  用于 calibration_poses.yaml 的格式：")
    print(f"    [{tcp_pos.x:.5f}, {tcp_pos.y:.5f}, {tcp_pos.z:.5f}, {pitch_calib_rad:.5f}]")
    print("=" * 70)


if __name__ == '__main__':
    main()
