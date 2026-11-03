#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自定义 IK 测试脚本

用法：python3 custom_ik_test.py
"""
import math
from config import Point3D
from ik_analytical import solve_analytical_ik_auto_pitch, solution_to_dict, solution_to_pwm
from fk import compute_fk_position_only

def test_target(x, y, z, pitch_deg=0):
    """测试指定目标点"""
    target = Point3D(x=x, y=y, z=z)
    pitch_rad = math.radians(pitch_deg)

    print(f"\n{'='*60}")
    print(f"目标位置: ({x:.3f}, {y:.3f}, {z:.3f}) m")
    print(f"         ({x*1000:.1f}, {y*1000:.1f}, {z*1000:.1f}) mm")
    print(f"期望俯仰角: {pitch_deg}°")
    print(f"{'='*60}")

    # 求解 IK（自动调整俯仰角模式）
    solution = solve_analytical_ik_auto_pitch(target, pitch_rad)

    if solution.error:
        print(f"❌ IK 求解失败: {solution.error}")
        return False

    print(f"\n✓ IK 求解成功: {solution.branch}")
    if solution.autoPitch:
        print(f"  自动调整俯仰角: {math.degrees(solution.actualPitch):.1f}° (偏差 {math.degrees(solution.pitchDeviation):.1f}°)")

    # 输出关节角度
    angles = solution_to_dict(solution)
    print(f"\n关节角度:")
    print(f"  arm_joint1 (底座旋转): {math.degrees(angles['arm_joint1']):+7.2f}°")
    print(f"  arm_joint2 (大臂):     {math.degrees(angles['arm_joint2']):+7.2f}°")
    print(f"  arm_joint3 (小臂):     {math.degrees(angles['arm_joint3']):+7.2f}°")
    print(f"  arm_joint4 (腕部):     {math.degrees(angles['arm_joint4']):+7.2f}°")
    print(f"  arm_joint5 (末端旋转): {math.degrees(angles['arm_joint5']):+7.2f}°")

    # 转换为 PWM
    # 转换为 PWM（直接映射）
    pwm_values = solution_to_pwm(solution)
    print(f"\nPWM 值（发送到舵机）:")
    print(f"  ID0 (底座): {pwm_values[0]}")
    print(f"  ID1 (大臂): {pwm_values[1]}")
    print(f"  ID2 (小臂): {pwm_values[2]}")
    print(f"  ID3 (腕部): {pwm_values[3]}")
    print(f"  ID4 (末端): {pwm_values[4]}")

    # FK 验证
    computed_pos = compute_fk_position_only(angles)
    error = math.sqrt((computed_pos.x - target.x)**2 +
                     (computed_pos.y - target.y)**2 +
                     (computed_pos.z - target.z)**2)

    print(f"\nFK 验证:")
    print(f"  计算位置: ({computed_pos.x:.6f}, {computed_pos.y:.6f}, {computed_pos.z:.6f}) m")
    print(f"  误差: {error*1000:.2f} mm")

    if error * 1000 < 1.5:
        print(f"  ✓ 精度合格 (< 1.5mm)")
    else:
        print(f"  ⚠ 精度不足 (>= 1.5mm)")

    return True

def main():
    print("\n" + "="*60)
    print("car1 机械臂 IK 自定义测试")
    print("="*60)

    # 测试用例：定义你想测试的目标点
    test_cases = [
        # (x, y, z, pitch_deg)
        (0.25, 0.00, 0.15, 0),      # 正前方，水平
        (0.22, 0.03, 0.12, -30),    # 右前方，向下30度
        (0.20, -0.02, 0.18, 15),    # 左前方，向上15度
        (0.18, 0.05, 0.10, -45),    # 右侧，向下45度
    ]

    passed = 0
    failed = 0

    for x, y, z, pitch in test_cases:
        if test_target(x, y, z, pitch):
            passed += 1
        else:
            failed += 1

    print(f"\n{'='*60}")
    print(f"测试完成: {passed} 成功, {failed} 失败")
    print(f"{'='*60}\n")

if __name__ == '__main__':
    main()
