#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
交互式 IK 测试工具

类似网页版的交互体验：
1. 输入 TCP 目标 (x, y, z, pitch)
2. 实时显示 IK 解（关节角度 + PWM 值）
3. 确认后发送到真实机械臂

用法：
  python3 interactive_ik_test.py            # ROS2 模式（需先 ros2 launch car1_bringup）
  python3 interactive_ik_test.py --direct   # 直连串口模式（需先停掉 bringup）
"""
import math
import sys
import time
import struct

from config import Point3D, GEOM, ROBOT_CALIBRATION
from ik_analytical import solve_analytical_ik_auto_pitch, solution_to_dict, solution_to_pwm
from fk import compute_fk_position_only

# ---------------------------------------------------------------------------
# 直连串口常量（与 test_servo_direct.py 一致）
# ---------------------------------------------------------------------------
SERIAL_PORT = '/dev/car1_stm32'
SERIAL_BAUD = 1000000
SERVO_MIN = 500
SERVO_MAX = 2500

_CRC8_TABLE = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
]


def _crc8(data):
    check = 0
    for b in data:
        check = _CRC8_TABLE[check ^ b]
    return check & 0xFF


def _send_packet(ser, func, data):
    buf = [0xAA, 0x55, func, len(data)] + data
    buf.append(_crc8(bytes(buf[2:])))
    ser.write(bytes(buf))


def _recv_packet(ser, timeout=1.0, expected_func=None):
    """接收完整数据包，支持按 func 过滤（与 test_servo_direct.py 一致）"""
    start_time = time.time()
    while time.time() - start_time < timeout:
        state, frame, data_len, recv_count = 0, [], 0, 0
        while time.time() - start_time < timeout:
            if ser.in_waiting > 0:
                b = ord(ser.read(1))
                if state == 0:
                    if b == 0xAA:
                        state = 1
                elif state == 1:
                    if b == 0x55:
                        state = 2
                    else:
                        state = 0
                elif state == 2:
                    frame = [b]
                    state = 3
                elif state == 3:
                    data_len = b
                    frame.append(b)
                    recv_count = 0
                    if data_len == 0:
                        state = 5
                    else:
                        state = 4
                elif state == 4:
                    frame.append(b)
                    recv_count += 1
                    if recv_count >= data_len:
                        state = 5
                elif state == 5:
                    expected_crc = _crc8(bytes(frame))
                    if expected_crc == b:
                        func = frame[0]
                        data = bytes(frame[2:]) if len(frame) > 2 else b''
                        if expected_func is not None and func != expected_func:
                            state = 0
                            continue
                        return func, data
                    state = 0
                    break
            else:
                time.sleep(0.001)
    return None


# ---------------------------------------------------------------------------
# ROS2 导入
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.node import Node
    from car1_base_driver_msgs.msg import ServosPosition, ServoPosition
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class InteractiveIKTester:
    """交互式 IK 测试工具"""

    # 舵机 ID → 关节名称映射
    SERVO_JOINT_MAP = {
        0: 'J1-底座旋转',
        1: 'J2-大臂',
        2: 'J3-小臂',
        3: 'J4-腕部',
        4: 'J5-末端旋转',
        5: '夹爪',
    }

    def __init__(self, direct_serial=False, debug=False):
        self.direct = direct_serial
        self.debug = debug
        self.node = None
        self.servo_pub = None
        self.ser = None

        if direct_serial:
            self._init_direct_serial()
        elif ROS_AVAILABLE:
            self._init_ros2()

    def _init_direct_serial(self):
        """直连串口模式（与 test_servo_direct.py 完全一致）"""
        try:
            import serial
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            self.ser.rts = False
            self.ser.dtr = False
            time.sleep(0.2)
            print(f"✓ 直连串口 {SERIAL_PORT} @ {SERIAL_BAUD}")
            print("  (send/read 均走串口，不受 board_sdk queue 丢包影响)")
        except ImportError:
            print("⚠ pyserial 未安装")
            self.ser = None
        except Exception as e:
            print(f"⚠ 串口打开失败: {e}")
            print("  请确认已停止 bringup: pkill -f bringup")
            self.ser = None

    def _init_ros2(self):
        """ROS2 模式"""
        try:
            rclpy.init()
            self.node = Node('interactive_ik_test')
            self.servo_pub = self.node.create_publisher(
                ServosPosition,
                '/car1_base_driver/bus_servo/set_position',
                10
            )
            print("✓ ROS2 已连接，可发送到实机")
        except Exception as e:
            print(f"⚠ ROS2 初始化失败: {e}")
            print("  继续使用计算模式")
            self.node = None

    # ------------------------------------------------------------------
    # 直连串口 — 发送/读取（与 test_servo_direct.py move_servos / read_servo_position 一致）
    # ------------------------------------------------------------------

    def _direct_move_servos(self, positions, duration=2.0):
        """positions: [[servo_id, pwm], ...]"""
        if not self.ser:
            print("❌ 串口未打开")
            return False

        # 检查限位
        for servo_id, pwm in positions:
            if not (SERVO_MIN <= pwm <= SERVO_MAX):
                joint_name = self.SERVO_JOINT_MAP.get(servo_id, f'ID{servo_id}')
                print(f"❌ {joint_name} PWM {pwm} 超限 (范围: {SERVO_MIN}-{SERVO_MAX})")
                return False

        duration_ms = int(duration * 1000)
        data = [0x01, duration_ms & 0xFF, (duration_ms >> 8) & 0xFF, len(positions)]
        for servo_id, pwm in positions:
            data.extend(struct.pack("<BH", servo_id, int(pwm)))
        _send_packet(self.ser, 0x05, data)
        print(f"  {'ID':<4} {'PWM':>6}")
        print(f"  {'-'*12}")
        for servo_id, pwm in positions:
            print(f"  {servo_id:<4} {int(pwm):>6}")
        return True

    def _read_servo_position(self, ser, servo_id, timeout=0.5, retries=3, debug=False):
        """
        读取单个舵机当前位置（与 test_servo_direct.py read_servo_position 一致，增加重试）。
        先清空缓冲区，发送读位置命令，只接收 func=0x05 的包。
        移动刚结束时总线被 IMU 流占满，前几个读取易超时，故重试。
        """
        if not ser:
            return None
        for attempt in range(retries):
            # 1) 清空接收缓冲区
            ser.reset_input_buffer()
            time.sleep(0.05)
            # 2) 发送读取位置命令 (func=0x05, cmd=0x05, servo_id)
            _send_packet(ser, 0x05, [0x05, servo_id])
            # 3) 接收响应，只取 func=0x05 的包（过滤 IMU 等数据）
            result = _recv_packet(ser, timeout=timeout, expected_func=0x05)
            if result:
                func, data = result
                if len(data) >= 5:
                    # 响应格式: req_id(1) + cmd(0x05回显,1) + status(1) + pos(2)
                    req_id, cmd, status, pos = struct.unpack("<BBBH", data[:5])
                    if debug:
                        print(f"    [DBG] ID{servo_id} attempt{attempt+1}: "
                              f"req_id={req_id} cmd=0x{cmd:02X} status={status} pos={pos}")
                    if status == 0:
                        return pos
                elif debug:
                    print(f"    [DBG] ID{servo_id} attempt{attempt+1}: 数据长度不足 ({len(data)})")
            elif debug:
                print(f"    [DBG] ID{servo_id} attempt{attempt+1}: 超时/无 func=0x05 响应")
        return None

    # ------------------------------------------------------------------
    # 公共接口
    # ------------------------------------------------------------------

    def print_header(self):
        print("\n" + "=" * 70)
        print("car1 机械臂交互式 IK 测试工具")
        if self.direct:
            print("[ 直连串口模式 ]")
        else:
            print("[ ROS2 模式 ]")
        print("=" * 70)
        print("\n工作空间参数:")
        print(f"  最大伸展: {GEOM.maxReach*1000:.1f} mm")
        print(f"  最小距离: {GEOM.minReach*1000:.1f} mm")
        print(f"  J1 原点:  ({ROBOT_CALIBRATION.J1_origin.x*1000:.1f}, "
              f"{ROBOT_CALIBRATION.J1_origin.y*1000:.1f}, "
              f"{ROBOT_CALIBRATION.J1_origin.z*1000:.1f}) mm")
        print("\n坐标系:")
        print("  +X: 向前    +Y: 向左    +Z: 向上")
        print("  单位: 米 (m)")
        print("\n俯仰角 (pitch):")
        print("  0°: 水平向前    -90°: 垂直向下    +90°: 垂直向上")
        print("=" * 70 + "\n")

    def get_input(self, prompt, default=None):
        if default is not None:
            prompt_text = f"{prompt} [默认: {default}]: "
        else:
            prompt_text = f"{prompt}: "
        value = input(prompt_text).strip()
        if not value and default is not None:
            return default
        return value

    def parse_float(self, value, name):
        try:
            return float(value)
        except ValueError:
            print(f"❌ 错误: {name} 必须是数字")
            return None

    def get_target_input(self):
        print("\n" + "-" * 70)
        print("输入目标 TCP 位置 (输入 'q' 退出)")
        print("-" * 70)

        x_str = self.get_input("X 坐标 (米)", "0.28")
        if x_str.lower() == 'q':
            return None
        x = self.parse_float(x_str, "X")
        if x is None:
            return None

        y_str = self.get_input("Y 坐标 (米)", "0.0")
        if y_str.lower() == 'q':
            return None
        y = self.parse_float(y_str, "Y")
        if y is None:
            return None

        z_str = self.get_input("Z 坐标 (米)", "0.10")
        if z_str.lower() == 'q':
            return None
        z = self.parse_float(z_str, "Z")
        if z is None:
            return None

        pitch_str = self.get_input("俯仰角 (度)", "180")
        if pitch_str.lower() == 'q':
            return None
        pitch_deg = self.parse_float(pitch_str, "俯仰角")
        if pitch_deg is None:
            return None

        return {'x': x, 'y': y, 'z': z, 'pitch_deg': pitch_deg}

    def compute_ik(self, target_input):
        target = Point3D(
            x=target_input['x'],
            y=target_input['y'],
            z=target_input['z']
        )
        pitch_rad = math.radians(target_input['pitch_deg'])

        print("\n" + "=" * 70)
        print("计算中...")
        print("=" * 70)

        solution = solve_analytical_ik_auto_pitch(target, pitch_rad)
        if solution.error:
            print(f"\n❌ IK 求解失败:")
            print(f"   {solution.error}")
            return None

        angles = solution_to_dict(solution)
        pwm_values = solution_to_pwm(solution)

        computed_pos = compute_fk_position_only(angles)
        error = math.sqrt(
            (computed_pos.x - target.x) ** 2 +
            (computed_pos.y - target.y) ** 2 +
            (computed_pos.z - target.z) ** 2
        )

        return {
            'solution': solution,
            'angles': angles,
            'pwm_values': pwm_values,
            'computed_pos': computed_pos,
            'error_mm': error * 1000
        }

    def display_result(self, target_input, result):
        print("\n" + "=" * 70)
        print("✓ IK 求解成功")
        print("=" * 70)

        solution = result['solution']
        angles = result['angles']
        pwm_values = result['pwm_values']

        print(f"\n目标位置:")
        print(f"  X: {target_input['x']:.3f} m ({target_input['x']*1000:.1f} mm)")
        print(f"  Y: {target_input['y']:.3f} m ({target_input['y']*1000:.1f} mm)")
        print(f"  Z: {target_input['z']:.3f} m ({target_input['z']*1000:.1f} mm)")
        print(f"  俯仰角: {target_input['pitch_deg']:.1f}°")

        if solution.autoPitch:
            print(f"\n⚠ 自动调整俯仰角:")
            print(f"  实际俯仰角: {math.degrees(solution.actualPitch):.1f}°")
            print(f"  偏差: {math.degrees(solution.pitchDeviation):.1f}°")

        print(f"\n解类型: {solution.branch}")

        print(f"\n关节角度 (弧度/角度):")
        joint_names = [
            ('arm_joint1', 'J1-底座旋转'),
            ('arm_joint2', 'J2-大臂'),
            ('arm_joint3', 'J3-小臂'),
            ('arm_joint4', 'J4-腕部'),
            ('arm_joint5', 'J5-末端旋转'),
        ]
        for joint_key, joint_label in joint_names:
            angle_rad = angles[joint_key]
            angle_deg = math.degrees(angle_rad)
            print(f"  {joint_label:12s}: {angle_rad:+7.4f} rad ({angle_deg:+7.2f}°)")

        print(f"\nPWM 值 (发送到舵机):")
        for i, pwm in enumerate(pwm_values):
            joint_name = self.SERVO_JOINT_MAP.get(i, f'ID{i}')
            print(f"  {joint_name:<14} (ID{i}): {pwm:4d}")

        computed_pos = result['computed_pos']
        error_mm = result['error_mm']
        print(f"\nFK 验证:")
        print(f"  计算位置: ({computed_pos.x:.6f}, {computed_pos.y:.6f}, {computed_pos.z:.6f}) m")
        print(f"  误差: {error_mm:.2f} mm")
        if error_mm < 1.5:
            print(f"  ✓ 精度合格 (< 1.5mm)")
        else:
            print(f"  ⚠ 精度不足 (>= 1.5mm)")

        print("=" * 70)

    def confirm_send(self):
        if self.direct:
            if not self.ser:
                print("\n⚠ 串口未打开，无法发送")
                return False, None
        else:
            if not self.servo_pub:
                print("\n⚠ ROS2 未连接，无法发送到实机")
                return False, None

        print("\n" + "-" * 70)
        response = input("是否发送到实机? (y/n) [n]: ").strip().lower()
        if response not in ('y', 'yes'):
            return False, None

        duration_str = input("运动时长 (秒) [默认: 2.0]: ").strip()
        try:
            duration = float(duration_str) if duration_str else 2.0
        except ValueError:
            duration = 2.0
        return True, duration

    def send_to_robot(self, pwm_values, duration=2.0):
        # 检查限位
        for servo_id, pwm in enumerate(pwm_values):
            if not (SERVO_MIN <= pwm <= SERVO_MAX):
                joint_name = self.SERVO_JOINT_MAP.get(servo_id, f'ID{servo_id}')
                print(f"❌ {joint_name} PWM {pwm} 超限 (范围: {SERVO_MIN}-{SERVO_MAX})")
                return False

        if self.direct:
            positions = [[i, int(p)] for i, p in enumerate(pwm_values)]
            ok = self._direct_move_servos(positions, duration)
            if ok:
                print(f"\n✓ 已发送到实机 (运动时长 {duration:.1f}s, 直连串口)")
            return ok

        if not self.servo_pub:
            print("❌ 无法发送: ROS2 未连接")
            return False

        try:
            msg = ServosPosition()
            msg.duration = float(duration)
            msg.position = []
            for servo_id, pwm in enumerate(pwm_values):
                servo = ServoPosition()
                servo.id = servo_id
                servo.position = int(pwm)
                msg.position.append(servo)
            self.servo_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            print(f"\n✓ 已发送到实机 (运动时长 {duration:.1f}s)")
            print("  话题: /car1_base_driver/bus_servo/set_position")
            return True
        except Exception as e:
            print(f"\n❌ 发送失败: {e}")
            return False

    def read_actual_pwm(self, servo_ids=(0, 1, 2, 3, 4, 5)):
        """
        读取舵机当前实际 PWM 位置（统一走直连串口逻辑，与 test_servo_direct.py 一致）。

        直连模式：复用已打开的 self.ser。
        ROS2 模式：bringup 占用串口，无法直接读取，需先停掉 bringup 再读。
                   此处临时打开 /dev/car1_stm32 完成读取后关闭。
        """
        if self.ser:
            return self._read_actual_pwm_direct(self.ser, servo_ids)

        # ROS2 模式：临时打开串口直连读取
        try:
            import serial
        except ImportError:
            print("⚠ 读取失败: pyserial 未安装")
            return None

        try:
            tmp = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
            tmp.rts = False
            tmp.dtr = False
            time.sleep(0.2)
        except Exception as e:
            print(f"⚠ 读取失败: 无法打开 {SERIAL_PORT}: {e}")
            print("  ROS2 模式下 bringup 占用串口，读取实际 PWM 需先停掉 bringup")
            print("  (或改用直连模式: python3 interactive_ik_test.py --direct)")
            return None

        try:
            return self._read_actual_pwm_direct(tmp, servo_ids)
        finally:
            tmp.close()

    def _read_actual_pwm_direct(self, ser, servo_ids):
        """直连串口读取（与 test_servo_direct.py read_all_servos 一致）"""
        print("\n读取舵机位置...")
        # 移动刚结束总线繁忙，先丢弃积压数据让总线稳定
        ser.reset_input_buffer()
        time.sleep(0.1)
        actual = {}
        for sid in servo_ids:
            pos = self._read_servo_position(ser, sid, debug=self.debug)
            if pos is not None:
                actual[sid] = pos
            time.sleep(0.05)
        return actual if actual else None

    def display_actual_pwm(self, target_pwm, actual_pwm):
        print("\n" + "-" * 70)
        print("读取舵机实际位置:")
        print("-" * 70)
        print(f"  {'关节':<14}{'ID':<6}{'目标PWM':<10}{'实际PWM':<10}{'误差':<8}")
        for servo_id in range(6):
            joint_name = self.SERVO_JOINT_MAP.get(servo_id, f'ID{servo_id}')
            target = target_pwm[servo_id] if servo_id < len(target_pwm) else None
            actual = actual_pwm.get(servo_id, None)
            if target is not None and actual is not None:
                diff = actual - target
                print(f"  {joint_name:<14}{servo_id:<6}{target:<10}{actual:<10}{diff:+d}")
            else:
                target_str = str(target) if target is not None else '-'
                actual_str = str(actual) if actual is not None else '读取失败'
                print(f"  {joint_name:<14}{servo_id:<6}{target_str:<10}{actual_str:<10}-")
        print("-" * 70)

    def run(self):
        self.print_header()

        while True:
            target_input = self.get_target_input()
            if target_input is None:
                print("\n退出程序")
                break

            result = self.compute_ik(target_input)
            if result is None:
                continue

            self.display_result(target_input, result)

            should_send, duration = self.confirm_send()
            if should_send:
                sent = self.send_to_robot(result['pwm_values'], duration)
                if sent:
                    resp = input("\n是否读取舵机实际 PWM? (y/n) [y]: ").strip().lower()
                    if resp in ('', 'y', 'yes'):
                        wait_t = duration + 0.5  # 加长等待时间,给舵机减速余量
                        print(f"等待运动完成 ({wait_t:.1f}s)...")
                        if self.direct:
                            time.sleep(wait_t)
                        else:
                            end_time = time.time() + wait_t
                            while time.time() < end_time:
                                rclpy.spin_once(self.node, timeout_sec=0.05)
                        actual = self.read_actual_pwm()
                        if actual is not None:
                            self.display_actual_pwm(result['pwm_values'], actual)
            print()

    def cleanup(self):
        if self.node:
            self.node.destroy_node()
            rclpy.shutdown()
        if self.ser:
            self.ser.close()
            print("串口已关闭")


def main():
    # 处理 --direct / --debug 参数（在 ROS2 解析前剥离）
    direct = '--direct' in sys.argv
    if direct:
        sys.argv.remove('--direct')
    debug = '--debug' in sys.argv
    if debug:
        sys.argv.remove('--debug')

    tester = InteractiveIKTester(direct_serial=direct, debug=debug)
    try:
        tester.run()
    except KeyboardInterrupt:
        print("\n\n程序被中断")
    finally:
        tester.cleanup()


if __name__ == '__main__':
    main()
