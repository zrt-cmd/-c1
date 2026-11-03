#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""直连下位机(STM32)的底盘测试脚本，绕过 ROS。

用途：隔离验证"转动/移动量是否正确"是不是下位机的问题。
严格复刻 board_sdk.py 的电机打包方式 + mecanum.py 的运动学，
这样发出去的字节和正常 ROS 路径完全一致。

协议(ros_controller_protocol.md + board_sdk.py 实测):
  帧: AA 55 | FUNC(0x03) | LEN | DATA... | CRC8
  电机多控子命令: data = [0x01, NUM] + 每个电机 <Bf>(id-1, rps)
  CRC = sum(FUNC..DATA最后) & 0xFF

用法示例:
  # 直行: 0.1 m/s 前进 2 秒
  python3 chassis_serial_test.py forward --speed 0.1 --time 2
  # 后退
  python3 chassis_serial_test.py backward --speed 0.1 --time 2
  # 左/右平移(麦轮)
  python3 chassis_serial_test.py left  --speed 0.1 --time 2
  python3 chassis_serial_test.py right --speed 0.1 --time 2
  # 原地旋转: 角速度 0.5 rad/s 逆时针，按指令转 90 度后停
  python3 chassis_serial_test.py rotate --wz 0.5 --deg 90
  # 原地旋转: 按时间转(数实际圈数用)
  python3 chassis_serial_test.py rotate --wz 0.5 --time 10
  # 直接给 4 个轮子原始 rps(最底层验证)
  python3 chassis_serial_test.py raw --rps 1 1 1 1 --time 2
"""
import sys
import time
import math
import struct
import argparse
import serial

# --- 机器人几何(与 mecanum.py / odom_publisher 默认一致) ---
WHEELBASE = 0.1946
TRACK_WIDTH = 0.2117
WHEEL_DIAMETER = 0.097

PACKET_FUNC_MOTOR = 0x03
HEAD = bytes([0xAA, 0x55])

# CRC8 查表（与 board_sdk._CRC8_TABLE 完全一致）。
# 注意：协议文档写的 sum()&0xFF 是错的，下位机实际用查表 CRC8。
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


def crc8(payload: bytes) -> int:
    """查表 CRC8，与 board_sdk._checksum_crc8 一致。"""
    check = 0
    for b in payload:
        check = _CRC8_TABLE[check ^ b]
    return check & 0xFF


def open_serial(port, baud):
    """与 board_sdk.__init__ 一致：先关 rts/dtr 再打开，避免复位 STM32。"""
    ser = serial.Serial(None, baud, timeout=0.1)
    ser.rts = False
    ser.dtr = False
    ser.setPort(port)
    ser.open()
    time.sleep(0.5)   # 与 board_sdk 一致，等下位机就绪
    return ser


def speed_to_rps(speed_mps: float) -> float:
    """线速度(m/s) -> 转/秒，与 mecanum.speed_to_rps 一致。"""
    return speed_mps / (math.pi * WHEEL_DIAMETER)


def mecanum_wheel_speeds(vx: float, vy: float, wz: float):
    """逆运动学: (vx,vy,wz) -> 4 轮【轮缘线速度 m/s】，与修复后 mecanum.py 一致。

    固件 raw 模式期望 m/s（其 PID 反馈 RT 单位即 m/s），所以不再转 rps。
    返回 [fl, fr, rl, rr] 的 m/s。
    """
    k = (WHEELBASE + TRACK_WIDTH) / 2.0
    fl = vx - vy - wz * k
    fr = vx + vy + wz * k
    rl = vx + vy - wz * k
    rr = vx - vy + wz * k
    return [fl, fr, rl, rr]


def build_motor_frame(wheel_rps):
    """复刻 board_sdk.set_motor_speed: 子命令0x01 + <Bf>(id-1, rps)。"""
    data = [0x01, len(wheel_rps)]
    for i, rps in enumerate(wheel_rps):
        data.extend(struct.pack("<Bf", i, float(rps)))  # id 从 0 开始(id-1)
    payload = bytes([PACKET_FUNC_MOTOR, len(data)]) + bytes(data)
    frame = HEAD + payload + bytes([crc8(payload)])
    return frame


def build_motor_type_frame(motor_type=0x02):
    """复刻 board_sdk.set_motor_type: FUNC_MOTOR + <BB>(0x05, type)。

    电机能动的前提：ROS 路径在 base_driver 启动时发这个(node:54)。
    直连脚本必须先发，否则下位机不驱动电机。
    """
    data = struct.pack("<BB", 0x05, motor_type)
    payload = bytes([PACKET_FUNC_MOTOR, len(data)]) + data
    return HEAD + payload + bytes([crc8(payload)])


def init_board(ser, motor_type=0x02):
    """发送电机类型初始化(发两次确保收到)。"""
    frame = build_motor_type_frame(motor_type)
    print(f"[init] set_motor_type=0x{motor_type:02x}, 帧: {frame.hex(' ')}")
    for _ in range(2):
        ser.write(frame)
        time.sleep(0.05)


def send_wheels(ser, wheel_rps):
    frame = build_motor_frame(wheel_rps)
    ser.write(frame)
    return frame


def stop(ser):
    send_wheels(ser, [0.0, 0.0, 0.0, 0.0])


def run_for(ser, wheels, duration, label):
    print(f"[{label}] wheel_speeds(m/s) = {[round(r,3) for r in wheels]}")
    print(f"[{label}] 持续 {duration}s ... (Ctrl+C 可中断)")
    frame = build_motor_frame(wheels)
    print(f"[{label}] 首帧字节: {frame.hex(' ')}")
    t0 = time.time()
    try:
        while time.time() - t0 < duration:
            ser.write(frame)         # 持续重发，防下位机超时停车
            time.sleep(0.05)
    finally:
        stop(ser)
        print(f"[{label}] 已发送停止指令")


def main():
    p = argparse.ArgumentParser(description="直连下位机底盘测试")
    p.add_argument("mode", choices=["forward", "backward", "left", "right",
                                    "rotate", "raw", "stop"])
    p.add_argument("--port", default="/dev/car1_stm32")
    p.add_argument("--baud", type=int, default=1000000)
    p.add_argument("--speed", type=float, default=0.1, help="线速度 m/s")
    p.add_argument("--wz", type=float, default=0.5, help="角速度 rad/s (rotate)")
    p.add_argument("--time", type=float, default=0.0, help="运行秒数")
    p.add_argument("--deg", type=float, default=0.0,
                   help="rotate: 按指令角速度积分转多少度后停(开环)")
    p.add_argument("--rps", type=float, nargs=4, metavar=("FL", "FR", "RL", "RR"),
                   help="raw 模式: 直接指定 4 轮 rps")
    args = p.parse_args()

    ser = open_serial(args.port, args.baud)

    # 电机类型初始化（电机能动的前提，等价 base_driver 启动时的 set_motor_type）
    if args.mode != "stop":
        init_board(ser)

    try:
        if args.mode == "stop":
            stop(ser)
            print("已停止")
            return

        if args.mode == "raw":
            if not args.rps:
                print("raw 模式需要 --rps FL FR RL RR")
                return
            dur = args.time if args.time > 0 else 2.0
            run_for(ser, args.rps, dur, "raw")
            return

        if args.mode in ("forward", "backward", "left", "right"):
            s = args.speed
            vx = vy = 0.0
            if args.mode == "forward":
                vx = s
            elif args.mode == "backward":
                vx = -s
            elif args.mode == "left":
                vy = s
            elif args.mode == "right":
                vy = -s
            dur = args.time if args.time > 0 else 2.0
            wheels = mecanum_wheel_speeds(vx, vy, 0.0)
            run_for(ser, wheels, dur, args.mode)
            return

        if args.mode == "rotate":
            wz = args.wz
            wheels = mecanum_wheel_speeds(0.0, 0.0, wz)
            if args.deg > 0:
                # 开环: 按"指令角速度"积分到目标角度
                dur = math.radians(args.deg) / abs(wz)
                print(f"[rotate] 指令角速度 {wz} rad/s, 目标 {args.deg}° "
                      f"-> 理论 {dur:.2f}s (这是按指令积分，实际转角请用指南针量)")
                run_for(ser, wheels, dur, "rotate")
            else:
                dur = args.time if args.time > 0 else 5.0
                turns_cmd = abs(wz) * dur / (2 * math.pi)
                print(f"[rotate] 指令角速度 {wz} rad/s × {dur}s "
                      f"-> 指令应转 {turns_cmd:.2f} 圈 (请数实际圈数对比)")
                run_for(ser, wheels, dur, "rotate")
            return
    except KeyboardInterrupt:
        stop(ser)
        print("\n中断，已停止")
    finally:
        stop(ser)
        ser.close()


if __name__ == "__main__":
    main()
