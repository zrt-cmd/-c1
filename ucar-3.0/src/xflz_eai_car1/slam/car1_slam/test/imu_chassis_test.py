"""
基于串口协议的 IMU 数据采集 + 麦轮旋转测试脚本

功能：
1. 通过 /dev/car1_stm32 与 STM32 通信 (波特率 1000000)
2. 发送麦轮速度命令，让车旋转
3. 同时接收并解析 STM32 上报的 IMU 数据 (功能字 0x07)
4. 保存到 CSV 文件供后续分析

IMU 帧格式 (来自 STM32 bmi088_read.c):
  AA 55 07 18 [accel_x:4f] [accel_y:4f] [accel_z:4f]
                [gyro_x:4f]  [gyro_y:4f]  [gyro_z:4f] CRC
  - accel: m/s²
  - gyro: rad/s
  - 上报频率: 50 Hz (osDelay 20ms)

注意：ros_controller_protocol.md 中写的 0x09 是错误的，
      实际 packet.h 中 PACKET_FUNC_IMU = 0x07
"""
import serial
import struct
import time
import csv
import threading
from datetime import datetime
from typing import Optional


# ============ 协议常量 ============
SERIAL_PORT = '/dev/car1_stm32'
BAUDRATE = 1000000
HEADER1 = 0xAA
HEADER2 = 0x55

# 功能字 (来自 packet.h enum PACKET_FUNCTION)
FUNC_SYS       = 0x00
FUNC_LED       = 0x01
FUNC_BUZZER    = 0x02
FUNC_MOTOR     = 0x03
FUNC_PWM_SERVO = 0x04
FUNC_BUS_SERVO = 0x05
FUNC_KEY       = 0x06
FUNC_IMU       = 0x07  # ← 实际值
FUNC_GAMEPAD   = 0x08
FUNC_SBUS      = 0x09
FUNC_OLED      = 0x0A
FUNC_RGB       = 0x0B

# IMU 数据长度: 6 个 float = 24 字节
IMU_DATA_LEN = 24
IMU_FRAME_LEN = 2 + 1 + 1 + IMU_DATA_LEN + 1  # 29 字节


# ============ CRC 计算 ============
# CRC8 查表法 (与 STM32 checksum.c 和 ROS2 driver board_sdk.py 一致)
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


def calc_crc(data: bytes) -> int:
    """CRC8 查表法 (与 STM32 checksum_crc8 一致)

    计算范围: 从功能字到数据内容最后一个字节
    多项式: 由查表隐含 (与 STM32 checksum.c 中的 crc8_table 完全一致)
    """
    check = 0
    for b in data:
        check = _CRC8_TABLE[check ^ b]
    return check & 0xFF


# ============ 协议帧构造 ============
def build_motor_multi_frame(speeds: list) -> bytes:
    """
    构造多电机速度控制帧 (CMD=0x01)

    speeds: 长度为 4 的 list, 单位 RPM, 实际范围约 ±1270 (STM32 R_VX_LIMIT=1500)
    电机 ID 范围: 0~3 (M1~M4), STM32 检查 id < 4
    返回: AA 55 03 LEN 01 NUM [ID SP0-3]... CRC
    """
    assert len(speeds) == 4, "speeds must have 4 elements"

    func = FUNC_MOTOR
    cmd = 0x01  # 多电机协同
    num = 4
    data_len = 2 + num * 5  # 22

    payload = bytes([func, data_len, cmd, num])
    for i, speed in enumerate(speeds):
        payload += bytes([i])                # 电机 ID 0~3 (与 STM32 kinematics_set_raw_wheel 一致)
        payload += struct.pack('<f', float(speed))  # float 小端

    crc = calc_crc(payload)
    return bytes([HEADER1, HEADER2]) + payload + bytes([crc])


def build_motor_stop_all() -> bytes:
    """构造多电机停止帧: AA 55 03 02 03 0F CRC
    CMD=0x03 多电机停止, MASK=0x0F (bit0~3 全选, M1~M4)
    """
    payload = bytes([FUNC_MOTOR, 0x02, 0x03, 0x0F])
    crc = calc_crc(payload)
    return bytes([HEADER1, HEADER2]) + payload + bytes([crc])


# ============ IMU 帧解析 ============
def parse_imu_frame(frame: bytes) -> Optional[dict]:
    """
    解析 IMU 数据帧

    帧结构:
      AA 55 07 18 [ax:4f][ay:4f][az:4f][gx:4f][gy:4f][gz:4f] CRC

    返回 dict 或 None
    """
    if len(frame) != IMU_FRAME_LEN:
        return None
    if frame[0] != HEADER1 or frame[1] != HEADER2 or frame[2] != FUNC_IMU:
        return None

    # 校验 CRC (从功能字到数据末尾)
    if calc_crc(frame[2:-1]) != frame[-1]:
        return None

    # 解析 6 个 float (小端)
    try:
        ax, ay, az, gx, gy, gz = struct.unpack('<ffffff', frame[4:28])
    except struct.error:
        return None

    return {
        'timestamp': time.time(),
        'accel_x': ax,
        'accel_y': ay,
        'accel_z': az,
        'gyro_x': gx,
        'gyro_y': gy,
        'gyro_z': gz,
    }


# ============ 串口读取线程 ============
class IMUReader:
    """在后台读取并解析串口数据"""

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.running = False
        self.imu_log = []
        self.latest_imu = None
        self.frame_count = 0
        self.crc_fail_count = 0

    def start(self):
        self.running = True
        t = threading.Thread(target=self._read_loop, daemon=True)
        t.start()

    def stop(self):
        self.running = False

    def _read_loop(self):
        buffer = bytearray()
        while self.running:
            try:
                data = self.ser.read(64)
                if not data:
                    continue
                buffer.extend(data)

                # 搜索帧头并解析
                while len(buffer) >= 2:
                    # 找帧头 AA 55
                    if buffer[0] != HEADER1 or buffer[1] != HEADER2:
                        buffer.pop(0)
                        continue

                    # 需要至少 4 字节才能判断帧长
                    if len(buffer) < 4:
                        break

                    func = buffer[2]
                    data_len = buffer[3]
                    frame_len = 4 + data_len + 1  # 帧头2 + 功能字1 + 长度1 + 数据 + CRC1

                    if len(buffer) < frame_len:
                        break

                    frame = bytes(buffer[:frame_len])
                    del buffer[:frame_len]

                    if func == FUNC_IMU:
                        imu = parse_imu_frame(frame)
                        if imu:
                            self.frame_count += 1
                            self.latest_imu = imu
                            self.imu_log.append(imu)
                        else:
                            self.crc_fail_count += 1
            except Exception as e:
                print(f"[READ ERROR] {e}")
                time.sleep(0.1)


# ============ 主程序 ============
def main():
    # ---- 测试参数 ----
    # 速度单位: m/s (轮缘线速度)
    # 来源: STM32 kinematics_set_raw_wheel(id, speed) 中 speed 直接赋值给 Wheel.TG,
    #   而 Wheel.RT(实测轮速) = encoder_ticks × MEC_WHEEL_SCALE,
    #   MEC_WHEEL_SCALE = π × D × PID_RATE / RES, 单位是 m/s.
    #   PID 比较 TG 和 RT, 所以 STM32 期望的单位是 m/s, 不是 RPS.
    #
    # 原地旋转角速度公式 (麦轮, 左右两轮反向):
    #   ω = 2 × v_wheel / track_width
    #   track_width = 0.216 m (MEC_ACLE_BASE, 实测值, 与固件一致)
    #
    # WHEEL_SPEED=0.212 m/s → ω ≈ 1.96 rad/s
    #                       → 5s 转 1.56 圈 (理论)
    # 判别实验:
    #   - 若 IMU 积分 ≈ 目视圈数 → 低速打滑是主因, IMU 正常
    #   - 若 IMU 积分 ≈ 2× 目视  → IMU 过报告 2×, 与速度无关
    WHEEL_SPEED = 0.212      # 单轮线速度 (m/s)
    DURATION = 5.0           # 旋转持续时间 (秒), 5s ≈ 转 1.56 圈
    SETTLE_TIME = 1.0        # 旋转前后静止采样时间 (秒)
    OUTPUT_CSV = 'imu_test_log.csv'

    # 理论角速度 (用于和 IMU gz 对比, 判断 IMU 缩小倍数)
    TRACK_WIDTH = 0.216
    THEORETICAL_OMEGA = 2.0 * WHEEL_SPEED / TRACK_WIDTH  # ≈ 1.96 rad/s

    print("=" * 50)
    print("  IMU 数据采集 + 麦轮旋转测试")
    print("=" * 50)
    print(f"串口:     {SERIAL_PORT} @ {BAUDRATE}")
    print(f"轮速:     {WHEEL_SPEED} m/s")
    print(f"理论ω:    {THEORETICAL_OMEGA:.3f} rad/s (期望 IMU gz ≈ 该值)")
    print(f"旋转时间: {DURATION}s")
    print(f"静止采样: {SETTLE_TIME}s (前后各一次)")
    print()
    print("[提示] 若需看 STM32 启动日志 (BMI088 Init / Gyro Chip ID / Scale),")
    print("       请按 STM32 复位键后立即运行本脚本。")
    print()

    # ---- 打开串口 ----
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUDRATE,
            bytesize=8,
            stopbits=1,
            parity='N',
            timeout=0.05,
        )
    except serial.SerialException as e:
        print(f"[ERROR] 无法打开串口 {SERIAL_PORT}: {e}")
        print("请检查: 1) 设备是否连接  2) 权限是否足够  3) 端口名是否正确")
        return

    time.sleep(0.5)
    print("[OK] 串口已打开")

    # ---- 启动 IMU 读取 ----
    reader = IMUReader(ser)
    reader.start()
    print("[OK] IMU 读取线程已启动")
    print()

    try:
        # 阶段 1: 静止采样 (基线)
        print(f"[阶段 1] 静止采样 {SETTLE_TIME}s (采集基线)...")
        t0 = time.time()
        while time.time() - t0 < SETTLE_TIME:
            if reader.latest_imu:
                imu = reader.latest_imu
                print(f"\r  静止: gyro_z={imu['gyro_z']:+.4f} rad/s", end='', flush=True)
            time.sleep(0.05)
        print()

        # 阶段 2: 麦轮旋转
        print(f"\n[阶段 2] 麦轮旋转 {DURATION}s...")
        # 麦轮原地旋转: 左轮正转, 右轮反转 (或反向)
        frame = build_motor_multi_frame([
            WHEEL_SPEED,   -WHEEL_SPEED,
            WHEEL_SPEED,   -WHEEL_SPEED,
        ])
        ser.write(frame)
        print(f"  发送电机命令: {frame.hex()}")

        t0 = time.time()
        while time.time() - t0 < DURATION:
            if reader.latest_imu:
                imu = reader.latest_imu
                gz = imu['gyro_z']
                ratio = THEORETICAL_OMEGA / gz if abs(gz) > 1e-4 else float('inf')
                print(f"\r  旋转: gyro_z={gz:+.4f} rad/s  理论={THEORETICAL_OMEGA:+.3f}  "
                      f"缩小≈{ratio:.1f}×  "
                      f"accel=({imu['accel_x']:+.2f},{imu['accel_y']:+.2f},{imu['accel_z']:+.2f})",
                      end='', flush=True)
            time.sleep(0.05)
        print()

        # 阶段 3: 停止
        print(f"\n[阶段 3] 停止电机, 静止采样 {SETTLE_TIME}s...")
        ser.write(build_motor_stop_all())
        print(f"  发送停止命令")

        t0 = time.time()
        while time.time() - t0 < SETTLE_TIME:
            if reader.latest_imu:
                imu = reader.latest_imu
                print(f"\r  停止: gyro_z={imu['gyro_z']:+.4f} rad/s", end='', flush=True)
            time.sleep(0.05)
        print()

    except KeyboardInterrupt:
        print("\n[中断] 用户取消, 停止电机...")
        ser.write(build_motor_stop_all())
    finally:
        reader.stop()
        ser.close()
        print("\n[OK] 串口已关闭")

    # ---- 保存 CSV ----
    print(f"\n{'=' * 50}")
    print(f"  数据保存")
    print(f"{'=' * 50}")
    print(f"收到 IMU 帧: {reader.frame_count}")
    print(f"CRC 失败:    {reader.crc_fail_count}")

    if not reader.imu_log:
        print("[WARNING] 未收到任何 IMU 数据!")
        print("可能原因: 1) STM32 固件未烧录  2) IMU 初始化失败  3) 接线问题")
        return

    with open(OUTPUT_CSV, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'timestamp', 'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
        ])
        writer.writeheader()
        for row in reader.imu_log:
            writer.writerow(row)
    print(f"[OK] 已保存 {len(reader.imu_log)} 条记录到 {OUTPUT_CSV}")

    # ---- 简单统计 ----
    gyro_z = [r['gyro_z'] for r in reader.imu_log]
    print(f"\n--- 角速度 Z (rad/s) 全程统计 ---")
    print(f"  最大值: {max(gyro_z):+.4f}")
    print(f"  最小值: {min(gyro_z):+.4f}")
    print(f"  平均值: {sum(gyro_z)/len(gyro_z):+.4f}")

    # 旋转阶段诊断: |gz| > 30% 理论值 视为旋转中
    spin_threshold = 0.3 * THEORETICAL_OMEGA
    spin_samples = [r['gyro_z'] for r in reader.imu_log if abs(r['gyro_z']) > spin_threshold]
    print(f"\n--- 旋转阶段诊断 (阈值 |gz|>{spin_threshold:.3f}) ---")
    print(f"  理论角速度:    {THEORETICAL_OMEGA:+.4f} rad/s")
    if spin_samples:
        spin_mean = sum(spin_samples) / len(spin_samples)
        ratio = THEORETICAL_OMEGA / spin_mean if abs(spin_mean) > 1e-4 else float('inf')
        print(f"  旋转阶段均值:  {spin_mean:+.4f} rad/s ({len(spin_samples)} 样本)")
        print(f"  缩小倍数:      {ratio:.2f}×  (理论/实测)")
        print()
        print("  === 诊断结论 ===")
        if ratio < 1.5:
            print(f"  ✓ IMU 基本正常 (缩小 {ratio:.2f}×, 在噪声范围内)")
            print("    残余误差可能来自: 轮速 PID 稳态误差 / 轮胎打滑 / track_width 标定")
        elif 1.8 <= ratio <= 2.2:
            print(f"  ✗ 2× 误差 (缩小 {ratio:.2f}×) → 量程寄存器 0x0F 未生效, 回退到 ±2000°/s 默认")
            print("    修复: bmi088.c 软复位后加 30ms 唤醒延时, 再写 0x0F")
        elif ratio > 5:
            print(f"  ✗ {ratio:.1f}× 误差 → 陀螺 Z 通道未正常采样 (非 scale 问题)")
            print("    可能原因: I2C 读取失败被忽略 / 软复位后未唤醒 / 数据撕裂")
            print("    修复: 加读取错误重试 + 软复位延时 + burst 读取 6 字节")
        else:
            print(f"  ? {ratio:.2f}× 误差, 介于 2× 和大倍数之间, 需结合启动日志判断")
    else:
        print(f"  ✗ 未检测到旋转样本 (无 |gz|>{spin_threshold:.3f} 的数据)")
        print("    可能: 车没转 / IMU 完全死掉 / 电机命令未生效")
        print("    建议: 检查 accel 是否有向心加速度变化, 确认车是否真转")

    accel_z = [r['accel_z'] for r in reader.imu_log]
    print(f"\n--- 加速度 Z (m/s²) 统计 ---")
    print(f"  最大值: {max(accel_z):+.4f}")
    print(f"  最小值: {min(accel_z):+.4f}")
    print(f"  平均值: {sum(accel_z)/len(accel_z):+.4f} (应接近 9.8)")


if __name__ == '__main__':
    main()
