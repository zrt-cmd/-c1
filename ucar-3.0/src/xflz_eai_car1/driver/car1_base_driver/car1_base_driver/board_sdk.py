#!/usr/bin/env python3
"""
STM32 Board Communication SDK for car1 robot.
Serial protocol: 0xAA 0x55 | Function | Length | Data | CRC8
Adapted from landerpi ros_robot_controller_sdk.
"""
import enum
import time
import queue
import struct
import serial
import threading


class PacketFunction(enum.IntEnum):
    PACKET_FUNC_SYS = 0
    PACKET_FUNC_LED = 1
    PACKET_FUNC_BUZZER = 2
    PACKET_FUNC_MOTOR = 3
    PACKET_FUNC_PWM_SERVO = 4
    PACKET_FUNC_BUS_SERVO = 5
    PACKET_FUNC_KEY = 6
    PACKET_FUNC_IMU = 7
    PACKET_FUNC_GAMEPAD = 8
    PACKET_FUNC_SBUS = 9
    PACKET_FUNC_OLED = 10
    PACKET_FUNC_RGB = 11
    PACKET_FUNC_NONE = 12


class PacketState(enum.IntEnum):
    STARTBYTE1 = 0
    STARTBYTE2 = 1
    FUNCTION = 2
    LENGTH = 3
    DATA = 4
    CHECKSUM = 5


# CRC8 lookup table
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


def _checksum_crc8(data):
    """Compute CRC8 checksum."""
    check = 0
    for b in data:
        check = _CRC8_TABLE[check ^ b]
    return check & 0xFF


BUS_SERVO_CMD_ALL_STATES = 0x08


class Board:
    """Low-level serial communication with STM32 board."""

    def __init__(self, device="/dev/car1_stm32", baudrate=1000000, timeout=10):
        self.enable_recv = False
        self.frame = []
        self.recv_count = 0

        self.port = serial.Serial(None, baudrate, timeout=timeout)
        self.port.rts = False
        self.port.dtr = False
        self.port.setPort(device)
        self.port.open()

        self.state = PacketState.STARTBYTE1
        self.servo_read_lock = threading.Lock()
        self.pwm_servo_read_lock = threading.Lock()

        self.sys_queue = queue.Queue(maxsize=1)
        self.bus_servo_queue = queue.Queue(maxsize=1)
        self.bus_servo_states_queue = queue.Queue(maxsize=1)
        self.pwm_servo_queue = queue.Queue(maxsize=1)
        self.key_queue = queue.Queue(maxsize=1)
        self.imu_queue = queue.Queue(maxsize=1)
        self.gamepad_queue = queue.Queue(maxsize=1)
        self.sbus_queue = queue.Queue(maxsize=1)

        self.parsers = {
            PacketFunction.PACKET_FUNC_SYS: self._report_sys,
            PacketFunction.PACKET_FUNC_KEY: self._report_key,
            PacketFunction.PACKET_FUNC_IMU: self._report_imu,
            PacketFunction.PACKET_FUNC_GAMEPAD: self._report_gamepad,
            PacketFunction.PACKET_FUNC_BUS_SERVO: self._report_bus_servo,
            PacketFunction.PACKET_FUNC_SBUS: self._report_sbus,
            PacketFunction.PACKET_FUNC_PWM_SERVO: self._report_pwm_servo,
        }

        time.sleep(0.5)
        threading.Thread(target=self._recv_task, daemon=True).start()

    # --- Queue report handlers ---

    def _report_sys(self, data):
        try:
            self.sys_queue.put_nowait(data)
        except queue.Full:
            pass

    def _report_key(self, data):
        try:
            self.key_queue.put_nowait(data)
        except queue.Full:
            pass

    def _report_imu(self, data):
        try:
            self.imu_queue.put_nowait(data)
        except queue.Full:
            pass

    def _report_gamepad(self, data):
        try:
            self.gamepad_queue.put_nowait(data)
        except queue.Full:
            pass

    def _report_bus_servo(self, data):
        if len(data) >= 2 and data[0] == BUS_SERVO_CMD_ALL_STATES:
            self._put_latest(self.bus_servo_states_queue, data)
        else:
            self._put_latest(self.bus_servo_queue, data)

    def _put_latest(self, target, data):
        # maxsize=1 队列：满时丢弃【旧】数据，保留【最新】响应
        # （旧实现丢弃新数据，导致正确响应被陈旧响应挤掉）
        try:
            target.put_nowait(data)
        except queue.Full:
            try:
                target.get_nowait()  # 丢弃旧数据
            except queue.Empty:
                pass
            try:
                target.put_nowait(data)  # 放入新数据
            except queue.Full:
                pass

    def _report_pwm_servo(self, data):
        try:
            self.pwm_servo_queue.put_nowait(data)
        except queue.Full:
            pass

    def _report_sbus(self, data):
        try:
            self.sbus_queue.put_nowait(data)
        except queue.Full:
            pass

    # --- Read methods ---

    def get_battery(self):
        """Get battery voltage in mV."""
        if not self.enable_recv:
            return None
        try:
            data = self.sys_queue.get(block=False)
            if data[0] == 0x04:
                return struct.unpack('<H', data[1:])[0]
        except queue.Empty:
            pass
        return None

    def get_button(self):
        """Get button event. Returns (id, state) or None."""
        if not self.enable_recv:
            return None
        try:
            data = self.key_queue.get(block=False)
            key_id = data[0]
            key_event = data[1]
            if key_event == 0x20:  # CLICK
                return key_id, 0
            elif key_event == 0x01:  # PRESSED
                return key_id, 1
        except queue.Empty:
            pass
        return None

    def get_imu(self):
        """Get IMU data: (ax, ay, az, gx, gy, gz)."""
        if not self.enable_recv:
            return None
        try:
            return struct.unpack('<6f', self.imu_queue.get(block=False))
        except queue.Empty:
            return None

    def get_gamepad(self):
        """Get gamepad data. Returns (axes, buttons) or None."""
        if not self.enable_recv:
            return None
        try:
            gamepad_data = struct.unpack("<HB4b", self.gamepad_queue.get(block=False))
            axes = [0.0] * 8
            buttons = [0] * 16
            # Parse buttons and axes from raw gamepad data
            btn_map = {
                0x0100: ('btn', 0), 0x0200: ('btn', 1), 0x0800: ('btn', 3),
                0x1000: ('btn', 4), 0x4000: ('btn', 6), 0x8000: ('btn', 7),
                0x0004: ('btn', 10), 0x0008: ('btn', 11),
                0x0001: ('axis', 5), 0x0002: ('axis', 4),
            }
            for mask, (kind, idx) in btn_map.items():
                if gamepad_data[0] & mask:
                    if kind == 'btn':
                        buttons[idx] = 1
                    else:
                        axes[idx] = 1.0
            # Analog sticks
            for i, ax_idx in enumerate([0, 1, 2, 3]):
                val = gamepad_data[2 + i]
                if val > 0:
                    axes[ax_idx] = -val / 127 if ax_idx in (0, 2) else val / 127
                elif val < 0:
                    axes[ax_idx] = -val / 128 if ax_idx in (0, 2) else val / 128
            return axes, buttons
        except queue.Empty:
            return None

    def get_bus_servo_states(self):
        """Get all bus servo states reported by subcommand 0x08."""
        if not self.enable_recv:
            return None
        try:
            data = self.bus_servo_states_queue.get(block=False)
        except queue.Empty:
            return None

        if len(data) < 2 or data[0] != BUS_SERVO_CMD_ALL_STATES:
            return None

        servo_count = data[1]
        expected_size = 2 + 6 * servo_count
        if len(data) != expected_size:
            return None

        states = []
        offset = 2
        for servo_id in range(servo_count):
            status, pwm, temp, volt_raw = struct.unpack_from("<BHBH", data, offset)
            states.append({
                'id': servo_id,
                'status': status,
                'valid': status == 0x00,
                'pwm': pwm,
                'temp': temp,
                'voltage': volt_raw / 10.0,
            })
            offset += 6
        return states

    # --- Write methods ---

    def _buf_write(self, func, data):
        """Send a packet to the board."""
        buf = [0xAA, 0x55, int(func)]
        buf.append(len(data))
        buf.extend(data)
        buf.append(_checksum_crc8(bytes(buf[2:])))

        self.port.write(bytes(buf))

    def set_led(self, on_time, off_time, repeat=1, led_id=1):
        on_time_ms = int(on_time * 1000)
        off_time_ms = int(off_time * 1000)
        self._buf_write(PacketFunction.PACKET_FUNC_LED,
                        struct.pack("<BHHH", led_id, on_time_ms, off_time_ms, repeat))

    def set_buzzer(self, freq, on_time, off_time, repeat=1):
        on_time_ms = int(on_time * 1000)
        off_time_ms = int(off_time * 1000)
        self._buf_write(PacketFunction.PACKET_FUNC_BUZZER,
                        struct.pack("<HHHH", freq, on_time_ms, off_time_ms, repeat))

    def set_motor_speed(self, speeds):
        """Set motor speeds. speeds: list of [id, rps]."""
        data = [0x01, len(speeds)]
        for motor_id, rps in speeds:
            data.extend(struct.pack("<Bf", int(motor_id - 1), float(rps)))
        self._buf_write(PacketFunction.PACKET_FUNC_MOTOR, data)

    def set_motor_type(self, motor_type=0x02):
        self._buf_write(PacketFunction.PACKET_FUNC_MOTOR,
                        struct.pack("<BB", 0x05, motor_type))

    def set_battery_level(self, battery_level=0x1af4):
        high_byte = (battery_level >> 8) & 0xFF
        low_byte = battery_level & 0xFF
        self._buf_write(PacketFunction.PACKET_FUNC_SYS,
                        struct.pack("BBB", 0x01, low_byte, high_byte))

    def set_rgb(self, pixels):
        """Set RGB LEDs. pixels: list of [index, r, g, b]."""
        data = [0x01, len(pixels)]
        for index, r, g, b in pixels:
            data.extend(struct.pack("<BBBB", int(index - 1), int(r), int(g), int(b)))
        self._buf_write(PacketFunction.PACKET_FUNC_RGB, data)

    def set_oled_text(self, line, text):
        data = [line, len(text)]
        data.extend(bytes(text, encoding='utf-8'))
        self._buf_write(PacketFunction.PACKET_FUNC_OLED, data)

    # --- Bus servo methods ---

    def bus_servo_set_position(self, duration, positions):
        """Set bus servo positions. duration in seconds, positions: [[id, pos], ...]."""
        duration_ms = int(duration * 1000)
        data = [0x01, duration_ms & 0xFF, (duration_ms >> 8) & 0xFF, len(positions)]
        for servo_id, pos in positions:
            data.extend(struct.pack("<BH", servo_id, pos))
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)

    def bus_servo_enable_torque(self, servo_id, enable):
        cmd = 0x0B if enable else 0x0C
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO,
                        struct.pack("<BB", cmd, servo_id))
        time.sleep(0.02)

    def bus_servo_stop(self, servo_ids):
        data = [0x03, len(servo_ids)]
        data.extend(struct.pack("<" + "B" * len(servo_ids), *servo_ids))
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, data)

    def bus_servo_set_id(self, old_id, new_id):
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO,
                        struct.pack("<BBB", 0x10, old_id, new_id))
        time.sleep(0.02)

    def bus_servo_set_offset(self, servo_id, offset):
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO,
                        struct.pack("<BBb", 0x20, servo_id, int(offset)))
        time.sleep(0.02)

    def bus_servo_save_offset(self, servo_id):
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO,
                        struct.pack("<BB", 0x24, servo_id))
        time.sleep(0.02)

    def bus_servo_set_angle_limit(self, servo_id, limit):
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO,
                        struct.pack("<BBHH", 0x30, servo_id, int(limit[0]), int(limit[1])))
        time.sleep(0.02)

    def bus_servo_set_vin_limit(self, servo_id, limit):
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO,
                        struct.pack("<BBHH", 0x34, servo_id, int(limit[0]), int(limit[1])))
        time.sleep(0.02)

    def bus_servo_set_temp_limit(self, servo_id, limit):
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO,
                        struct.pack("<BBb", 0x38, servo_id, int(limit)))
        time.sleep(0.02)

    def _bus_servo_read(self, servo_id, cmd, fmt):
        """
        读取总线舵机数据（支持 ID 校验）

        Args:
            servo_id: 舵机 ID
            cmd: 读取命令（0x05=读位置，0x38=读温度等）
            fmt: 响应数据格式（struct unpack format）

        Returns:
            tuple: 读取到的数据（不含 cmd, servo_id, error_code），失败返回 None
        """
        with self.servo_read_lock:
            # 发送前清空队列中的陈旧响应（上次超时残留的延迟响应）
            # 该队列只存舵机读取响应，IMU/电池等走独立队列，清空不影响其他功能
            while not self.bus_servo_queue.empty():
                try:
                    self.bus_servo_queue.get_nowait()
                except queue.Empty:
                    break

            # 发送读取命令
            self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO, [cmd, servo_id])

            expected_size = struct.calcsize(fmt)
            max_attempts = 2  # 最多尝试 2 次

            for attempt in range(max_attempts):
                try:
                    # 阻塞等待响应（超时 1 秒）
                    data = self.bus_servo_queue.get(block=True, timeout=1.0)
                except Exception:
                    return None

                # 检查数据长度
                if len(data) < expected_size:
                    continue  # 数据长度不正确，继续等待下一个响应

                try:
                    # 解析响应
                    result = struct.unpack(fmt, data[:expected_size])
                    response_id = result[0]
                    response_cmd = result[1]
                    error_code = result[2]

                    # 校验 cmd 和 servo_id 是否匹配
                    if response_cmd != cmd or response_id != servo_id:
                        if attempt < max_attempts - 1:
                            continue
                        else:
                            return None

                    # 检查错误码
                    if error_code != 0:
                        return None

                    return result[3:]
                except Exception:
                    continue

            # 超过最大重试次数
            return None

    def bus_servo_read_position(self, servo_id):
        return self._bus_servo_read(servo_id, 0x05, "<BBBH")

    def bus_servo_read_vin(self, servo_id):
        return self._bus_servo_read(servo_id, 0x07, "<BBbH")

    def bus_servo_read_temp(self, servo_id):
        return self._bus_servo_read(servo_id, 0x09, "<BBbB")

    def bus_servo_read_id(self, servo_id=254):
        return self._bus_servo_read(servo_id, 0x12, "<BBbB")

    def bus_servo_read_offset(self, servo_id):
        return self._bus_servo_read(servo_id, 0x22, "<BBbb")

    def bus_servo_read_angle_limit(self, servo_id):
        return self._bus_servo_read(servo_id, 0x32, "<BBb2H")

    def bus_servo_read_vin_limit(self, servo_id):
        return self._bus_servo_read(servo_id, 0x36, "<BBb2H")

    def bus_servo_read_temp_limit(self, servo_id):
        return self._bus_servo_read(servo_id, 0x3A, "<BBbB")

    def bus_servo_read_torque_state(self, servo_id):
        return self._bus_servo_read(servo_id, 0x0D, "<BBbb")

    def bus_servo_request_all_states(self):
        """Ask the board to report the 0x08 all-servo state frame once."""
        self._buf_write(PacketFunction.PACKET_FUNC_BUS_SERVO,
                        [BUS_SERVO_CMD_ALL_STATES, 0x00])

    # --- PWM servo methods ---

    def pwm_servo_set_position(self, duration, positions):
        duration_ms = int(duration * 1000)
        data = [0x01, duration_ms & 0xFF, (duration_ms >> 8) & 0xFF, len(positions)]
        for servo_id, pos in positions:
            data.extend(struct.pack("<BH", servo_id, pos))
        self._buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, data)

    def pwm_servo_set_offset(self, servo_id, offset):
        self._buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO,
                        struct.pack("<BBb", 0x07, servo_id, int(offset)))

    def pwm_servo_read_position(self, servo_id):
        with self.pwm_servo_read_lock:
            self._buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, [0x05, servo_id])
            data = self.pwm_servo_queue.get(block=True, timeout=1.0)
            _, _, pos = struct.unpack("<BBH", data)
            return pos

    def pwm_servo_read_offset(self, servo_id):
        with self.pwm_servo_read_lock:
            self._buf_write(PacketFunction.PACKET_FUNC_PWM_SERVO, [0x09, servo_id])
            data = self.pwm_servo_queue.get(block=True, timeout=1.0)
            _, _, offset = struct.unpack("<BBb", data)
            return offset

    # --- Reception ---

    def enable_reception(self, enable=True):
        self.enable_recv = enable

    def _recv_task(self):
        """Background thread: parse incoming serial packets.

        Wrapped in a top-level try/except so a serial hiccup (e.g. another
        process briefly opens the same tty) does NOT kill the receive thread.
        If it dies, no bus_servo responses ever arrive again and every service
        call times out — which is very hard to notice because the process
        looks alive.
        """
        while True:
            try:
                if not self.enable_recv:
                    time.sleep(0.01)
                    continue
                recv_data = self.port.read()
                if not recv_data:
                    continue
                for dat in recv_data:
                    if self.state == PacketState.STARTBYTE1:
                        if dat == 0xAA:
                            self.state = PacketState.STARTBYTE2
                    elif self.state == PacketState.STARTBYTE2:
                        if dat == 0x55:
                            self.state = PacketState.FUNCTION
                        else:
                            self.state = PacketState.STARTBYTE1
                    elif self.state == PacketState.FUNCTION:
                        if dat < int(PacketFunction.PACKET_FUNC_NONE):
                            self.frame = [dat, 0]
                            self.state = PacketState.LENGTH
                        else:
                            self.frame = []
                            self.state = PacketState.STARTBYTE1
                    elif self.state == PacketState.LENGTH:
                        self.frame[1] = dat
                        self.recv_count = 0
                        if dat == 0:
                            self.state = PacketState.CHECKSUM
                        else:
                            self.state = PacketState.DATA
                    elif self.state == PacketState.DATA:
                        self.frame.append(dat)
                        self.recv_count += 1
                        if self.recv_count >= self.frame[1]:
                            self.state = PacketState.CHECKSUM
                    elif self.state == PacketState.CHECKSUM:
                        crc8 = _checksum_crc8(bytes(self.frame))
                        if crc8 == dat:
                            func = PacketFunction(self.frame[0])
                            data = bytes(self.frame[2:])
                            if func in self.parsers:
                                self.parsers[func](data)
                        self.state = PacketState.STARTBYTE1
            except Exception as e:
                # Reset the parser state so we don't try to resume mid-frame
                # with corrupt bytes. Log lightly to avoid spam if the port
                # is really gone.
                self.state = PacketState.STARTBYTE1
                self.frame = []
                self.recv_count = 0
                print(f'[_recv_task] serial error: {e}, will retry')
                time.sleep(0.1)
