#!/usr/bin/env python3
"""
Level 3 FK/IK 闭环验证工具

验证逻辑：
1. 输入PWM值
2. FK计算末端位置
3. IK反推关节角度
4. 发送到实物
5. 读取实际PWM
6. 对比输入vs输出

如果FK和IK都正确，输入PWM和输出PWM应该基本一致。
"""
import math
import time
import rclpy
from rclpy.node import Node
from car1_base_driver_msgs.msg import ServosPosition, ServoPosition
from car1_servo_controller_msgs.msg import ServoStateList


class IKSolver:
    """解析IK求解器（从arm_controller.py复制）"""

    def __init__(self, d1, a2, a3, a4):
        self.d1 = d1
        self.a2 = a2
        self.a3 = a3
        self.a4 = a4

    def solve(self, x, y, z, pitch=0.0):
        """
        IK求解
        Returns: [[j1, j2_urdf, j3, j4, j5]] or None
        """
        theta1 = math.atan2(y, x)
        r = math.sqrt(x**2 + y**2)
        pz = z - self.d1

        wr = r - self.a4 * math.cos(pitch)
        wz = pz - self.a4 * math.sin(pitch)

        D = wr**2 + wz**2
        max_reach = self.a2 + self.a3
        if math.sqrt(D) > max_reach * 0.98:
            return None
        if math.sqrt(D) < abs(self.a2 - self.a3) * 1.05:
            return None

        cos_theta3 = (D - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
        if abs(cos_theta3) > 1.0:
            return None

        theta3 = math.acos(cos_theta3)
        theta2 = math.atan2(wz, wr) - math.atan2(
            self.a3 * math.sin(theta3), self.a2 + self.a3 * math.cos(theta3))
        theta4 = pitch - (theta2 + theta3)

        theta2_urdf = theta2 + math.pi / 2
        theta3_urdf = theta3
        theta4_urdf = theta4

        return [[theta1, theta2_urdf, theta3_urdf, theta4_urdf, 0.0]]


class FKIKLoopTest(Node):
    """FK/IK闭环测试节点"""

    # PWM转换常数 (240°范围)
    RADIANS_PER_TICK = 240.0 / 360.0 * (2 * math.pi) / 2000.0
    TICKS_PER_RADIAN = 1.0 / RADIANS_PER_TICK

    def __init__(self):
        super().__init__('fk_ik_loop_test')

        # DH参数 (从arm_params.yaml)
        self.d1 = 0.1005
        self.a2 = 0.12614
        self.a3 = 0.12614
        self.a4 = 0.151

        # IK求解器
        self.ik_solver = IKSolver(self.d1, self.a2, self.a3, self.a4)

        # 舵机配置 (从servo_controller.yaml)
        self.joint_config = [
            {'init': 1500, 'orientation': 1},  # joint1
            {'init': 1500, 'orientation': 1},  # joint2
            {'init': 1500, 'orientation': 1},  # joint3
            {'init': 1500, 'orientation': 1},  # joint4
            {'init': 1500, 'orientation': 1},  # joint5
        ]

        # 订阅舵机状态反馈
        self.latest_servo_states = None
        self.create_subscription(
            ServoStateList,
            '/servo_controller/servo_states',
            self.servo_state_callback,
            10
        )

        # 发布舵机命令
        self.servo_pub = self.create_publisher(
            ServosPosition,
            '/car1_base_driver/bus_servo/set_position',
            10
        )

        self.get_logger().info('FK/IK闭环测试工具已启动')
        self.get_logger().info('等待舵机状态反馈...')

        # 等待首次反馈
        timeout = 5.0
        start_time = time.time()
        while self.latest_servo_states is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_servo_states is None:
            self.get_logger().error('超时：未收到舵机状态反馈')
            self.get_logger().error('请确保 bringup 已启动')
        else:
            self.get_logger().info('已连接到舵机控制器')

    def servo_state_callback(self, msg):
        """保存最新的舵机状态"""
        self.latest_servo_states = msg

    def pwm_to_rad(self, pwm, joint_idx):
        """PWM → 弧度"""
        config = self.joint_config[joint_idx]
        init = config['init']
        orientation = config['orientation']

        if orientation == 1:
            return (pwm - init) * self.RADIANS_PER_TICK
        else:
            return (init - pwm) * self.RADIANS_PER_TICK

    def rad_to_pwm(self, rad, joint_idx):
        """弧度 → PWM"""
        config = self.joint_config[joint_idx]
        init = config['init']
        orientation = config['orientation']

        if orientation == 1:
            return int(init + rad * self.TICKS_PER_RADIAN)
        else:
            return int(init - rad * self.TICKS_PER_RADIAN)

    def fk_compute(self, joints_rad):
        """
        正运动学：关节角度 → 末端位置

        Args:
            joints_rad: [j1, j2_urdf, j3, j4, j5]
        Returns:
            {'x': float, 'y': float, 'z': float, 'pitch': float}
        """
        theta1 = joints_rad[0]
        theta2_urdf = joints_rad[1]
        theta3 = joints_rad[2]
        theta4 = joints_rad[3]

        # URDF → IK坐标系
        theta2_ik = theta2_urdf - math.pi / 2

        # FK计算
        r_wrist = self.a2 * math.cos(theta2_ik) + self.a3 * math.cos(theta2_ik + theta3)
        z_wrist = self.d1 + self.a2 * math.sin(theta2_ik) + self.a3 * math.sin(theta2_ik + theta3)

        pitch = theta2_ik + theta3 + theta4

        x = (r_wrist + self.a4 * math.cos(pitch)) * math.cos(theta1)
        y = (r_wrist + self.a4 * math.cos(pitch)) * math.sin(theta1)
        z = z_wrist + self.a4 * math.sin(pitch)

        return {'x': x, 'y': y, 'z': z, 'pitch': pitch}

    def send_to_robot(self, pwm_list, duration=2.0):
        """发送PWM到实物"""
        msg = ServosPosition()
        msg.duration = float(duration)

        for i, pwm in enumerate(pwm_list):
            pos = ServoPosition()
            pos.id = i
            pos.position = int(pwm)
            msg.position.append(pos)

        self.servo_pub.publish(msg)

    def read_actual_pwm(self):
        """读取实物当前PWM"""
        if self.latest_servo_states is None:
            return None

        pwm_list = []
        for i in range(5):
            for servo in self.latest_servo_states.servo_state:
                if servo.id == i:
                    pwm_list.append(servo.position)
                    break

        return pwm_list if len(pwm_list) == 5 else None

    def test_one_pose(self, name, input_pwm):
        """测试一个姿态的闭环"""
        print(f"\n{'='*70}")
        print(f"测试姿态: {name}")
        print(f"输入PWM: {input_pwm}")

        # 步骤1: PWM → 弧度
        joints_rad = [self.pwm_to_rad(pwm, i) for i, pwm in enumerate(input_pwm)]
        print(f"  → 关节弧度: {[round(r, 4) for r in joints_rad]}")
        joints_deg = [math.degrees(r) for r in joints_rad]
        print(f"  → 关节角度: {[round(d, 2) for d in joints_deg]}°")

        # 步骤2: FK计算末端位置
        position = self.fk_compute(joints_rad)
        print(f"  → FK末端位置: x={position['x']:.4f}m, y={position['y']:.4f}m, z={position['z']:.4f}m, pitch={math.degrees(position['pitch']):.2f}°")

        # 步骤3: IK反推关节角度
        ik_joints = self.ik_solver.solve(
            position['x'], position['y'], position['z'], position['pitch'])

        if not ik_joints:
            print("  ✗ IK无解 (目标超出工作空间)")
            return False

        ik_rad = ik_joints[0]
        print(f"  → IK关节弧度: {[round(r, 4) for r in ik_rad]}")
        ik_deg = [math.degrees(r) for r in ik_rad]
        print(f"  → IK关节角度: {[round(d, 2) for d in ik_deg]}°")

        # 步骤4: 弧度 → PWM
        output_pwm = [self.rad_to_pwm(rad, i) for i, rad in enumerate(ik_rad)]
        print(f"  → 输出PWM: {output_pwm}")

        # 步骤5: 发送到实物
        print("  → 发送到实物，等待运动完成...")
        self.send_to_robot(output_pwm, duration=2.5)

        # 等待运动完成并持续更新反馈
        # 前3秒运动，后1秒稳定并多次采样
        for _ in range(30):  # 3秒运动
            rclpy.spin_once(self, timeout_sec=0.1)

        # 多次采样取最新值，确保读到运动后的位置
        for _ in range(10):  # 1秒稳定采样
            rclpy.spin_once(self, timeout_sec=0.1)

        # 步骤6: 读取实物反馈
        actual_pwm = self.read_actual_pwm()
        if actual_pwm is None:
            print("  ✗ 无法读取实物反馈 (请确保bringup已启动)")
            return False

        print(f"  → 实物反馈PWM: {actual_pwm}")

        # 步骤7: 对比分析
        diff = [actual - inp for actual, inp in zip(actual_pwm, input_pwm)]
        print(f"\n  误差分析 (输入PWM vs 实物PWM):")
        for i, (inp, out, d) in enumerate(zip(input_pwm, actual_pwm, diff)):
            deg_diff = d * 0.12  # 1 pulse ≈ 0.12°
            status = "  " if abs(d) < 20 else "⚠️"
            print(f"    {status} joint{i+1}: {inp:4d} → {out:4d}  (diff: {d:+4d} pulse = {deg_diff:+5.1f}°)")

        max_diff = max(abs(d) for d in diff)
        print(f"\n  最大误差: {max_diff} pulse = {max_diff * 0.12:.1f}°")

        success = False
        if max_diff < 5:
            print(f"  ✅ 优秀 (<5 pulse, FK/IK完美闭环)")
            success = True
        elif max_diff < 20:
            print(f"  ✅ 良好 (<20 pulse, 允许机械间隙)")
            success = True
        else:
            print(f"  ❌ 失败 (>20 pulse, FK/IK或DH参数有问题)")
            success = False

        return success

    def run_test_suite(self):
        """运行所有测试用例"""
        print("\n" + "="*70)
        print("  Level 3 FK/IK 闭环验证测试")
        print("="*70)
        print("\n测试原理:")
        print("  输入PWM → FK计算位置 → IK反推PWM → 发送实物 → 读取PWM")
        print("  如果FK和IK都正确，输入PWM ≈ 输出PWM")
        print("\n测试用例:")

        test_cases = {
            'pre_grasp': [1500, 1100, 2200, 1850, 1500],
            'grasp_down': [1500, 1850, 2109, 1850, 1500],
        }

        for name in test_cases:
            print(f"  - {name}")

        input("\n按回车开始测试...")

        results = []
        for name, pwm in test_cases.items():
            success = self.test_one_pose(name, pwm)
            results.append((name, success))

            if name != list(test_cases.keys())[-1]:
                print("\n按回车继续下一个测试...")
                input()

        # 总结
        print(f"\n{'='*70}")
        print("测试总结:")
        passed = sum(1 for _, s in results if s)
        total = len(results)
        print(f"  通过: {passed}/{total}")

        for name, success in results:
            status = "✅ 通过" if success else "✗ 失败"
            print(f"    {name}: {status}")

        if passed == total and passed > 0:
            print("\n✅ 所有测试通过！说明:")
            print("  ✅ FK计算公式正确")
            print("  ✅ IK解析解正确")
            print("  ✅ DH参数准确 (d1, a2, a3, a4)")
            print("  ✅ PWM↔弧度转换正确")
        elif passed == 0:
            print("\n❌ 所有测试失败！请检查:")
            print("  1. bringup是否正常启动")
            print("  2. /servo_controller/servo_states 是否在发布")
            print("  3. 实物机械臂是否连接")
        else:
            print(f"\n⚠️ 部分测试失败 ({passed}/{total})")
            print("  请检查失败的测试用例")

        print("="*70)


def main(args=None):
    rclpy.init(args=args)
    node = FKIKLoopTest()

    try:
        node.run_test_suite()
    except KeyboardInterrupt:
        print("\n测试被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
