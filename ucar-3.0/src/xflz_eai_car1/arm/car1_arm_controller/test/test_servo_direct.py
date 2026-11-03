#!/usr/bin/env python3
"""
直接通过串口控制机械臂，使用 PWM 脉冲值（500~2500，1500为中位）。

适配下位机固件：STM32 会通过子命令 0x08 持续上报所有总线舵机状态帧
（status/pwm/temp/voltage）。本脚本用一个后台接收线程逐帧完整解析，
0x08 持续帧 → 最新状态缓存并显示；其他应答帧 → 应答队列，供单发读消费。
接收与发送互不阻塞，不再因 0x08 持续帧淹没而丢数据。

使用前请先停止 bringup：
  Ctrl+C 停掉 ros2 launch car1_arm_controller arm_controller.launch.py

然后运行：
  python3 test_servo_direct.py
"""
import os
import struct
import threading
import time
import yaml
import serial

# --- 配置 ---
SERIAL_PORT = '/dev/car1_stm32'
BAUDRATE = 1000000

# 舵机范围
SERVO_MIN = 500
SERVO_MAX = 2500
SERVO_MID = 1500

# 总线舵机功能字 / 子命令
FUNC_BUS_SERVO = 0x05
CMD_ALL_STATES = 0x08           # 下位机持续上报全舵机状态
CMD_READ_POS = 0x05             # 读位置
CMD_DISABLE_TORQUE = 0x0B       # 释放扭力(微阻力)
CMD_ENABLE_TORQUE = 0x0C        # 锁死
CMD_SET_CSM = 0x40              # 设置开机旋转模式(写EEPROM,无应答)
CMD_READ_CSM = 0x42             # 读取开机模式

# 默认工作状态 (J0~J5)
WORK_POSITION = [1500, 1100, 2200, 1850, 1500, 1500]

# CRC8 查表（与 board_sdk.py 一致）
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


def crc8(data):
    check = 0
    for b in data:
        check = _CRC8_TABLE[check ^ b]
    return check & 0xFF


class SerialReader:
    """后台接收线程 + 发送，接收到的 0x08 状态帧与应答帧分离，互不干扰。"""

    def __init__(self, ser):
        self.ser = ser
        self._running = True
        # 0x08 全舵机状态：最新覆盖，防堆积
        self._servo_states = None
        self._states_lock = threading.Lock()
        # 应答帧队列（read 用），只放非 0x08 应答
        self._resp_lock = threading.Lock()
        self._resp_frames = []          # FIFO 应答帧列表
        self._resp_cond = threading.Condition(self._resp_lock)

    # ---- 发送 ----
    def send_packet(self, func, data):
        buf = bytearray([0xAA, 0x55, func, len(data)])
        buf.extend(data)
        buf.append(crc8(bytes(buf[2:])))
        self.ser.write(bytes(buf))

    # ---- 后台接收线程 ----
    def start(self):
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _recv_loop(self):
        state = 0  # 0:AA 1:55 2:func 3:len 4:data 5:crc
        frame = bytearray()
        data_len = 0
        recv_count = 0
        try:
            while self._running:
                if self.ser.in_waiting <= 0:
                    time.sleep(0.001)
                    continue
                for byte in self.ser.read(self.ser.in_waiting):
                    if state == 0:
                        state = 1 if byte == 0xAA else 0
                    elif state == 1:
                        state = 2 if byte == 0x55 else 0
                    elif state == 2:
                        frame = bytearray([byte])
                        state = 3
                    elif state == 3:
                        data_len = byte
                        frame.append(byte)
                        recv_count = 0
                        state = 5 if data_len == 0 else 4
                    elif state == 4:
                        frame.append(byte)
                        recv_count += 1
                        if recv_count >= data_len:
                            state = 5
                    elif state == 5:
                        if crc8(bytes(frame)) == byte:
                            func = frame[0]
                            body = bytes(frame[2:])
                            self._dispatch(func, body)
                        state = 0  # 无论 CRC 是否通过：回到找帧头
        except Exception as e:
            state = 0
            print(f'[_recv] serial error: {e}, will retry')

    def _dispatch(self, func, body):
        # 0x08：全舵机状态帧，更新缓存（不丢）
        if func == FUNC_BUS_SERVO and len(body) >= 2 and body[0] == CMD_ALL_STATES:
            self._update_states(body)
            return
        # 其他应答帧：入应答队列，供单发读消费
        if func == FUNC_BUS_SERVO:
            with self._resp_cond:
                self._resp_frames.append((func, body))
                self._resp_cond.notify_all()

    def stop(self):
        self._running = False

    # ---- 状态解析 ----
    def _parse_states(self, body):
        servo_count = body[1]
        expected = 2 + 6 * servo_count
        if len(body) != expected:
            return None
        states = []
        off = 2
        for sid in range(servo_count):
            status, pwm, temp, volt_raw = struct.unpack_from('<BHBH', body, off)
            states.append({
                'id': sid, 'status': status, 'valid': status == 0x00,
                'pwm': pwm, 'temp': temp, 'voltage': volt_raw / 10.0,
            })
            off += 6
        return states

    def _update_states(self, body):
        states = self._parse_states(body)
        if states is None:
            return
        with self._states_lock:
            self._servo_states = states

    # ---- 读取 ----
    def read_resp(self, timeout=1.0):
        """取出最早一条应答帧，超时返回 None。0x08 帧不会进入此队列。"""
        with self._resp_cond:
            if self._resp_frames:
                return self._resp_frames.pop(0)
            deadline = time.time() + timeout
            while not self._resp_frames:
                remaining = deadline - time.time()
                if remaining <= 0:
                    return None
                self._resp_cond.wait(timeout=remaining)
            return self._resp_frames.pop(0)

    def clear_resp(self):
        with self._resp_lock:
            self._resp_frames.clear()

    def read_position(self, servo_id, timeout=1.0):
        self.send_packet(FUNC_BUS_SERVO, [CMD_READ_POS, servo_id])
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            resp = self.read_resp(timeout=remaining)
            if resp is None:
                return None
            func, body = resp
            # 应答: [cmd][id][status][pos:2]
            if func == FUNC_BUS_SERVO and len(body) >= 5 and body[0] == CMD_READ_POS and body[1] == servo_id:
                status = body[2]
                if status == 0:
                    return struct.unpack_from('<H', body, 3)[0]
                return None
            # 非目标应答/其它帧，继续等

    def read_csm(self, servo_id, timeout=1.0):
        self.send_packet(FUNC_BUS_SERVO, [CMD_READ_CSM, servo_id])
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            resp = self.read_resp(timeout=remaining)
            if resp is None:
                return None
            func, body = resp
            # 应答: [cmd][id][status][mode]
            if func == FUNC_BUS_SERVO and len(body) >= 4 and body[0] == CMD_READ_CSM and body[1] == servo_id:
                return body[3]
            # 其它应答/超时，继续等

    @property
    def servo_states(self):
        with self._states_lock:
            return self._servo_states


def move_servos(reader, positions, duration=2.0):
    duration_ms = int(duration * 1000)
    clamped = []
    for servo_id, pwm in positions:
        pwm = max(SERVO_MIN, min(SERVO_MAX, int(pwm)))
        clamped.append([servo_id, pwm])
    data = [0x01, duration_ms & 0xFF, (duration_ms >> 8) & 0xFF, len(clamped)]
    for servo_id, pwm in clamped:
        data.append(servo_id & 0xFF)
        data.append(pwm & 0xFF)
        data.append((pwm >> 8) & 0xFF)
    reader.send_packet(FUNC_BUS_SERVO, data)
    print(f"  {'ID':<4} {'PWM':>6}")
    print(f"  {'-'*12}")
    for servo_id, pwm in clamped:
        print(f"  {servo_id:<4} {pwm:>6}")


def move_arm(reader, j1, j2, j3, j4, j5, duration=2.0):
    move_servos(reader, [[0, j1], [1, j2], [2, j3], [3, j4], [4, j5]], duration)


def set_gripper(reader, pwm, duration=1.0):
    move_servos(reader, [[5, pwm]], duration)


def release_servo_torque(reader, servo_ids):
    for servo_id in servo_ids:
        reader.send_packet(FUNC_BUS_SERVO, [CMD_DISABLE_TORQUE, servo_id])
        time.sleep(0.02)
    print(f"  释放力矩(PULK): {servo_ids}")


def restore_servo_torque(reader, servo_ids):
    for servo_id in servo_ids:
        pos = reader.read_position(servo_id)
        if pos is not None:
            move_servos(reader, [[servo_id, pos]], duration=0.5)
            time.sleep(0.05)
        else:
            print(f"  读取舵机 {servo_id} 位置失败，跳过")
    print(f"  恢复力矩: {servo_ids}")


# 下位机开机模式映射（对应 servo_protocol.md 的 #<ID>PCSM<模式号>!）
_POWER_ON_MODE_NAMES = {1: '带力矩(PCSM1,默认)', 2: '保持位置(PCSM2)', 3: '无力1(PCSM3)', 4: '无力2(PCSM4)'}


def set_power_on_mode(reader, servo_id, mode):
    """
    持久写入舵机开机模式。mode: 1~4。对应子命令 0x40，与 /opt/xflz/sensor_html_test 的 set_csm 一致。
    写入 EEPROM 后舵机会自动重启（该次应答缺失，等 2~3 秒后再确认）。
    """
    desc = _POWER_ON_MODE_NAMES.get(mode, f'模式{mode}')
    reader.send_packet(FUNC_BUS_SERVO, [CMD_SET_CSM, servo_id, int(mode) & 0xFF])
    time.sleep(0.05)
    print(f"  舵机 {servo_id} 已发送固化为 CSM{mode} ({desc})，舵机将自动重启")


def read_power_on_mode(reader, servo_id):
    mode = reader.read_csm(servo_id)
    if mode is not None:
        desc = _POWER_ON_MODE_NAMES.get(mode, f'模式{mode}')
        print(f"  舵机 {servo_id} 开机模式 = CSM{mode} ({desc})")
        return mode
    print(f"  舵机 {servo_id} 读开机模式失败")
    return None


def read_all_servos(reader, debug=False):
    """打印后台线程最新缓存的 0x08 全舵机状态（持续上报，天然最新）。"""
    print("\n读取舵机位置(0x08上报)...")
    print(f"  {'ID':<4} {'PWM':>6}  {'temp':>5} {'volt':>6} {'ok':>3}")
    print(f"  {'-'*34}")
    states = reader.servo_states
    if not states:
        print("  暂无 0x08 状态，等待下位机上报...")
        return []
    for st in states:
        print(f"  {st['id']:<4} {st['pwm']:>6}  {st['temp']:>5} {st['voltage']:>6.1f} {'Y' if st['valid'] else 'N':>3}")
    return [(st['id'], st['pwm']) for st in states if st['valid']]


def display_states_loop(reader, sleep_s=0.3):
    """在后台打印 0x08 上报的最新舵机状态（不丢弃、控制刷屏频率）。"""
    def _loop():
        while True:
            states = reader.servo_states
            if states:
                text = ' | '.join(
                    f'ID{s["id"]}:pwm={s["pwm"]},temp={s["temp"]}C,volt={s["voltage"]:.1f}V'
                    for s in states
                )
                print(f'\r[SERVOS] {text:<100}', end='')
            time.sleep(sleep_s)
    threading.Thread(target=_loop, daemon=True).start()


def save_current_pose(reader, include_gripper=False):
    print("\n读取当前姿态(0x08上报)...")
    print(f"  {'ID':<4} {'PWM':>6}")
    print(f"  {'-'*12}")
    states = reader.servo_states
    if not states:
        print("  暂无 0x08 状态，无法保存")
        return [], None
    by_id = {s['id']: s['pwm'] for s in states}
    positions = []
    for sid in range(5):
        pwm = by_id.get(sid)
        if pwm is not None:
            positions.append(pwm)
            print(f"  {sid:<4} {pwm:>6}")
        else:
            print(f"  {sid:<4} {'缺失':>6}")
            return [], None
        time.sleep(0.01)
    gripper_pos = None
    if include_gripper:
        gripper_pos = by_id.get(5)
        if gripper_pos is not None:
            print(f"  {5:<4} {gripper_pos:>6}")
        else:
            print("  夹爪数据缺失")
            return [], None
    name = input("\n  保存? 输入名称(回车跳过): ").strip()
    if not name:
        return [], None
    actions = [('arm', positions, name)]
    if include_gripper and gripper_pos is not None:
        actions.append(('gripper', gripper_pos, f"{name}_gripper"))
    print(f"  已保存: {name}")
    return actions, name


def go_work(reader, duration=2.0):
    positions = [[i, pwm] for i, pwm in enumerate(WORK_POSITION)]
    move_servos(reader, positions, duration)


def go_home(reader, duration=2.0):
    move_servos(reader, [[0, 1500], [1, 1500], [2, 1500], [3, 1500], [4, 1500], [5, 1500]], duration)


ACTIONS = [
    ('home', None, '归零状态'),
    ('arm', [1500, 1100, 2200, 1850, 1500], '工作状态')
]


def interactive_mode(reader):
    print("=" * 60)
    print("=== 交互调试模式 (后台接收0x08状态，回答与发送分离) ===")
    print("=" * 60)
    print("命令:")
    print("  j1 j2 j3 j4 j5   - 5个关节PWM值(500~2500)")
    print("  g <pwm>           - 夹爪PWM值")
    print("  s <id> <pwm>      - 单个舵机控制，如: s 2 1200")
    print("  read              - 读取 0x08 上报的所有舵机状态")
    print("  save / save -g    - 从 0x08 状态保存当前姿态")
    print("  free [id]         - 释放力矩(可手掰)")
    print("  load [id]         - 恢复力矩(锁死)")
    print("  csm <模式>        - 持久写入全部6颗舵机的开机模式(1~4)，如: csm 3")
    print("  csm <id> <模式>   - 设置指定舵机n的开机模式，如: csm 2 3")
    print("  csr <n>           - 读取舵机n的开机模式")
    print("  home / work / play / yaml / list / del / quit")
    print("-" * 60)

    actions = []
    while True:
        try:
            cmd = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        if cmd == 'quit':
            break
        elif cmd == 'home':
            go_home(reader)
            time.sleep(2.5)
            name = input("  保存? 输入名称(回车跳过): ").strip()
            if name:
                actions.append(('home', None, name))
        elif cmd == 'work':
            go_work(reader)
            time.sleep(2.5)
            name = input("  保存? 输入名称(回车跳过): ").strip()
            if name:
                actions.append(('arm', WORK_POSITION[:5], name))
        elif cmd.startswith('g '):
            try:
                pwm = int(cmd[2:])
                set_gripper(reader, pwm)
                time.sleep(1.5)
                name = input("  保存? 输入名称(回车跳过): ").strip()
                if name:
                    actions.append(('gripper', pwm, name))
            except ValueError:
                print("  格式错误，示例: g 1800")
        elif cmd.startswith('s '):
            parts = cmd[2:].split()
            if len(parts) == 2:
                try:
                    sid = int(parts[0]); pwm = int(parts[1])
                    move_servos(reader, [[sid, pwm]], duration=1.0)
                    time.sleep(1.5)
                except ValueError:
                    print("  格式错误，示例: s 2 1200")
            else:
                print("  格式错误，示例: s 2 1200")
        elif cmd == 'read':
            read_all_servos(reader)
        elif cmd == 'save' or cmd == 'save -g':
            saved, _name = save_current_pose(reader, include_gripper=('-g' in cmd))
            if saved:
                actions.extend(saved)
        elif cmd == 'free' or cmd.startswith('free '):
            parts = cmd.split()
            ids = [int(parts[1])] if len(parts) > 1 else list(range(6))
            release_servo_torque(reader, ids)
        elif cmd == 'load' or cmd.startswith('load '):
            parts = cmd.split()
            ids = [int(parts[1])] if len(parts) > 1 else list(range(6))
            restore_servo_torque(reader, ids)
        elif cmd.startswith('csm '):
            parts = cmd.split()
            try:
                if len(parts) == 2:
                    mode = int(parts[1])
                    for sid in range(6):
                        set_power_on_mode(reader, sid, mode)
                        time.sleep(0.05)
                elif len(parts) == 3:
                    sid = int(parts[1]); mode = int(parts[2])
                    set_power_on_mode(reader, sid, mode)
                else:
                    print("  格式: csm <mode> 或 csm <id> <mode>")
            except ValueError:
                print("  格式错误，模式1~4: 1带力矩 2保持位置 3无力1 4无力2")
        elif cmd.startswith('csr '):
            parts = cmd.split()
            if len(parts) == 2:
                try:
                    read_power_on_mode(reader, int(parts[1]))
                except ValueError:
                    print("  格式错误，示例: csr 2")
            else:
                print("  格式错误，示例: csr 2")
        elif cmd == 'play':
            if not actions:
                print("  没有已保存的动作")
            else:
                print(f"  回放 {len(actions)} 个动作...")
                for act in actions:
                    print(f"  [{act[2]}]")
                    if act[0] == 'arm':
                        move_arm(reader, *act[1], duration=2.0)
                        time.sleep(3.0)
                    elif act[0] == 'gripper':
                        set_gripper(reader, act[1])
                        time.sleep(2.0)
                    elif act[0] == 'home':
                        go_home(reader, duration=2.0)
                        time.sleep(3.0)
                print("  回放完成")
        elif cmd == 'yaml':
            export_yaml(actions)
        elif cmd == 'list':
            if not actions:
                print("  没有已保存的动作")
            else:
                for i, act in enumerate(actions):
                    if act[0] == 'arm':
                        print(f"  {i+1}. [{act[2]}] arm: {act[1]}")
                    elif act[0] == 'gripper':
                        print(f"  {i+1}. [{act[2]}] gripper: {act[1]}")
                    elif act[0] == 'home':
                        print(f"  {i+1}. [{act[2]}] home")
        elif cmd.startswith('del '):
            try:
                idx = int(cmd[4:]) - 1
                if 0 <= idx < len(actions):
                    removed = actions.pop(idx)
                    print(f"  已删除: {removed[2]}")
                else:
                    print(f"  序号超出范围 (1~{len(actions)})")
            except ValueError:
                print("  格式错误，示例: del 2")
        else:
            parts = cmd.replace(',', ' ').split()
            if len(parts) == 5:
                try:
                    pwms = [int(x) for x in parts]
                    move_arm(reader, *pwms, duration=2.0)
                    time.sleep(2.5)
                    name = input("  保存? 输入名称(回车跳过): ").strip()
                    if name:
                        actions.append(('arm', pwms, name))
                except ValueError:
                    print("  格式错误，示例: 1500 1000 1000 900 1500")
            else:
                print("  无法识别，可用命令同上。例: 1500 1000 1000 900 1500")
    return actions


def export_actions(actions):
    if not actions:
        return
    print("\n" + "=" * 50)
    print("=== 导出动作序列 ===")
    print("=" * 50)
    print("\n将以下内容替换脚本中的 ACTIONS:\n")
    print("ACTIONS = [")
    for act in actions:
        if act[0] == 'arm':
            print(f"    ('arm', {act[1]}, '{act[2]}'),")
        elif act[0] == 'gripper':
            print(f"    ('gripper', {act[1]}, '{act[2]}'),")
        elif act[0] == 'home':
            print(f"    ('home', None, '{act[2]}'),")
    print("]")


def export_yaml(actions, output_path=None):
    if not actions:
        print("  没有已保存的动作，无法导出")
        return
    if not output_path:
        name = input("  动作组名称 (如 pick_front): ").strip()
        if not name:
            print("  取消导出")
            return
        desc = input("  描述 (回车跳过): ").strip() or f"{name} action"
        default_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            'src/xflz_eai_car1/arm/car1_arm_controller/config/actions')
        output_path = os.path.join(default_dir, f'{name}.yaml')
    else:
        name = os.path.splitext(os.path.basename(output_path))[0]
        desc = f"{name} action"
    yaml_actions = []
    for act in actions:
        step = {'name': act[2], 'duration': 2.0}
        if act[0] == 'arm':
            step['servos'] = {i: act[1][i] for i in range(5)}
        elif act[0] == 'gripper':
            step['servos'] = {5: act[1]}
            step['duration'] = 1.0
        elif act[0] == 'home':
            step['servos'] = {i: 1500 for i in range(6)}
            step['duration'] = 3.0
        yaml_actions.append(step)
    data = {'name': name, 'description': desc, 'actions': yaml_actions}
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
    print(f"\n  已导出到: {output_path}")
    print(f"  动作步数: {len(yaml_actions)}")
    print(f"\n  使用方法:")
    print(f"    ros2 run car1_arm_controller action_player --ros-args -p action_file:={name}")


def playback_mode(reader):
    print("=" * 50)
    print("=== 回放模式 ===")
    print("=" * 50)
    print(f"共 {len(ACTIONS)} 个动作\n")
    for i, act in enumerate(ACTIONS):
        print(f"[{i+1}/{len(ACTIONS)}] {act[2]}")
        if act[0] == 'arm':
            move_arm(reader, *act[1], duration=2.0)
            time.sleep(3.0)
        elif act[0] == 'gripper':
            set_gripper(reader, act[1])
            time.sleep(2.0)
        elif act[0] == 'home':
            go_home(reader, duration=2.0)
            time.sleep(3.0)
    print("\n=== 回放完成 ===")


def main():
    print(f"打开串口 {SERIAL_PORT} @ {BAUDRATE}...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        ser.rts = False
        ser.dtr = False
    except Exception as e:
        print(f"串口打开失败: {e}")
        print("请确认：")
        print("  1. 已停止 bringup (Ctrl+C)")
        print("  2. 串口存在: ls -l /dev/car1_stm32 /dev/car1_lidar")
        print("  3. 有权限: udev 规则已设 0666（或 sudo chmod 666 /dev/car1_stm32）")
        return
    print("串口已打开\n")
    time.sleep(0.2)
    reader = SerialReader(ser)
    reader.start()
    print("后台接收线程已启动 (解析 0x08 状态, 应答分离)\n")
    time.sleep(0.2)

    print("选择模式:")
    print("  1. 交互模式 - 手动输入PWM值调试")
    print("  2. 回放模式 - 执行 ACTIONS 列表")
    choice = input("\n输入 1 或 2: ").strip()
    try:
        if choice == '1':
            actions = interactive_mode(reader)
            export_actions(actions)
        else:
            playback_mode(reader)
    except KeyboardInterrupt:
        print("\n\n中断，回到工作位置...")
        go_work(reader, duration=1.5)
        time.sleep(2.0)
    finally:
        reader.stop()
        ser.close()
        print("串口已关闭")


if __name__ == '__main__':
    main()