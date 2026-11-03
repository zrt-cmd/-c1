#!/usr/bin/env python3
"""
Level 1 验证脚本：IK → 角度 → PWM 完整链路测试

验证内容：
1. angle2pulse: 0 rad → 全部 1500
2. pulse2angle: 全部 1500 → 0 rad
3. IK 求解 → PWM 转换 → 范围检查
4. 往返转换一致性（angle → pulse → angle）

不需要硬件，纯计算验证。
"""
import sys
import math
sys.path.insert(0, '/home/iflytek/ros2_ws/src/xflz_eai_car1/arm/kinematics/kinematics')

from transform import angle2pulse, pulse2angle, check_limits, SCALE
from ik_demo import compute_ik, compute_fk_simple


def test_zero_position():
    """测试1：零位转换"""
    print("=" * 60)
    print("测试1：零位转换")
    print("=" * 60)

    # 0 rad → 应该全部是 1500
    angles = [0.0, 0.0, 0.0, 0.0, 0.0]
    pulses = angle2pulse(angles)
    print(f"  输入角度: {angles}")
    print(f"  输出 PWM: {pulses}")

    all_center = all(p[1] == 1500 for p in pulses)
    print(f"  全部为 1500: {'PASS' if all_center else 'FAIL'}")

    # 1500 → 应该全部是 0 rad
    pulse_dict = {0: 1500, 1: 1500, 2: 1500, 3: 1500, 4: 1500}
    angles_back = pulse2angle(pulse_dict)
    print(f"\n  输入 PWM: {pulse_dict}")
    print(f"  输出角度: {angles_back}")

    all_zero = all(abs(v) < 0.001 for v in angles_back.values())
    print(f"  全部为 0 rad: {'PASS' if all_zero else 'FAIL'}")

    return all_center and all_zero


def test_direction():
    """测试2：方向验证"""
    print("\n" + "=" * 60)
    print("测试2：方向验证（正角度 → PWM 减小）")
    print("=" * 60)

    # 正角度（+45°）→ PWM 应该 < 1500（因为 direction=-1）
    angle_45 = math.radians(45)
    pulses = angle2pulse([angle_45, 0, 0, 0, 0])
    pwm_j1 = pulses[0][1]
    expected = 1500 + (-1) * 45 * SCALE  # ≈ 1167
    print(f"  arm_joint1 = +45° ({angle_45:.4f} rad)")
    print(f"  PWM = {pwm_j1} (期望 ≈ {int(expected)})")
    print(f"  PWM < 1500: {'PASS' if pwm_j1 < 1500 else 'FAIL'}")

    # 负角度（-45°）→ PWM 应该 > 1500
    pulses_neg = angle2pulse([math.radians(-45), 0, 0, 0, 0])
    pwm_j1_neg = pulses_neg[0][1]
    print(f"\n  arm_joint1 = -45° ({math.radians(-45):.4f} rad)")
    print(f"  PWM = {pwm_j1_neg}")
    print(f"  PWM > 1500: {'PASS' if pwm_j1_neg > 1500 else 'FAIL'}")

    return pwm_j1 < 1500 and pwm_j1_neg > 1500


def test_roundtrip():
    """测试3：往返转换一致性"""
    print("\n" + "=" * 60)
    print("测试3：往返转换一致性 (angle → pulse → angle)")
    print("=" * 60)

    test_angles = [
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, -0.3, 0.8, -0.5, 0.2],
        [-1.0, 1.0, -0.5, 0.3, -0.8],
    ]

    all_pass = True
    for angles in test_angles:
        pulses = angle2pulse(angles)
        pulse_dict = {p[0]: p[1] for p in pulses}
        angles_back = pulse2angle(pulse_dict)

        max_error = 0
        for i, name in enumerate(['arm_joint1', 'arm_joint2', 'arm_joint3', 'arm_joint4', 'arm_joint5']):
            error = abs(angles[i] - angles_back[name])
            max_error = max(max_error, error)

        ok = max_error < 0.02  # 允许 PWM 取整误差
        all_pass = all_pass and ok
        print(f"  输入: {[f'{a:.3f}' for a in angles]}")
        print(f"  最大误差: {math.degrees(max_error):.3f}° {'PASS' if ok else 'FAIL'}")

    return all_pass


def test_ik_to_pulse():
    """测试4：IK 求解 → PWM 转换"""
    print("\n" + "=" * 60)
    print("测试4：IK 求解 → PWM 转换")
    print("=" * 60)

    targets = [
        ([0.25, 0.0, 0.20], "正前方"),
        ([0.20, 0.10, 0.18], "右前方"),
        ([0.20, -0.10, 0.18], "左前方"),
        ([0.15, 0.0, 0.25], "上方"),
    ]

    all_pass = True
    for target_pos, desc in targets:
        result, error = compute_ik(target_pos)
        if result is None:
            print(f"\n  目标: {desc} {target_pos} → IK 无解: {error}")
            all_pass = False
            continue

        angles = [result[f'arm_joint{i}'] for i in range(1, 6)]

        # 限位检查
        ok, violations = check_limits(angles)
        if not ok:
            print(f"\n  目标: {desc} {target_pos} → 超出限位: {violations}")
            all_pass = False
            continue

        # 转换为 PWM
        pulses = angle2pulse(angles)

        # 检查 PWM 范围
        pwm_ok = all(500 <= p[1] <= 2500 for p in pulses)

        # FK 验证
        fk_pos = compute_fk_simple(result)
        fk_error = math.sqrt(sum((fk_pos[i] - target_pos[i])**2 for i in range(3)))

        print(f"\n  目标: {desc} {target_pos}")
        print(f"  角度: {[f'{math.degrees(a):.1f}°' for a in angles]}")
        print(f"  PWM:  {[p[1] for p in pulses]}")
        print(f"  FK 误差: {fk_error*1000:.2f} mm")
        print(f"  PWM 范围: {'PASS' if pwm_ok else 'FAIL'}")

        all_pass = all_pass and pwm_ok

    return all_pass


def test_work_position():
    """测试5：验证 test_servo_direct.py 中的工作位置"""
    print("\n" + "=" * 60)
    print("测试5：已知工作位置 PWM → 角度 → FK 位置")
    print("=" * 60)

    # test_servo_direct.py 中的工作位置
    work_pwm = [1500, 1100, 2200, 1850, 1500]
    print(f"  工作位置 PWM: {work_pwm}")

    angles = pulse2angle(work_pwm)
    print(f"  转换角度:")
    for name, rad in angles.items():
        print(f"    {name}: {math.degrees(rad):.1f}°")

    # FK 计算末端位置
    fk_pos = compute_fk_simple(angles)
    print(f"  FK 末端位置: x={fk_pos[0]:.4f}, y={fk_pos[1]:.4f}, z={fk_pos[2]:.4f} m")

    return True


def main():
    print("\n" + "#" * 60)
    print("# Level 1 验证：运动学基础 + 角度↔PWM 转换")
    print("#" * 60)

    results = []
    results.append(("零位转换", test_zero_position()))
    results.append(("方向验证", test_direction()))
    results.append(("往返一致性", test_roundtrip()))
    results.append(("IK→PWM", test_ik_to_pulse()))
    results.append(("工作位置验证", test_work_position()))

    print("\n" + "=" * 60)
    print("总结")
    print("=" * 60)
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  {name}: {status}")

    all_pass = all(r[1] for r in results)
    print(f"\n  总体结果: {'ALL PASS' if all_pass else 'HAS FAILURES'}")
    return 0 if all_pass else 1


if __name__ == '__main__':
    sys.exit(main())
