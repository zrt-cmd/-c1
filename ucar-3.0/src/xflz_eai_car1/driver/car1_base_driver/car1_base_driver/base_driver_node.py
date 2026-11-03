#!/usr/bin/env python3
"""
ROS2 node for car1 STM32 board communication.
Publishes IMU, battery, button data.
Subscribes to motor, servo, LED, buzzer commands.
"""
import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger
from sensor_msgs.msg import Imu, Joy
import yaml
from ament_index_python.packages import get_package_share_directory

from car1_base_driver_msgs.msg import (
    MotorsState, ServosPosition, SetBusServoState,
    SetPWMServoState, BusServoState, BusServoStates,
    ButtonState, BuzzerState, LedState, OLEDState, RGBStates,
)
from car1_base_driver_msgs.srv import GetBusServoState, GetPWMServoState

from car1_base_driver.board_sdk import Board


class BaseDriverNode(Node):
    """ROS2 interface to the STM32 board."""

    def __init__(self):
        super().__init__('car1_base_driver')

        # Parameters
        self.declare_parameter('serial_port', '/dev/car1_stm32')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('motor_type', 0x02)
        self.declare_parameter('battery_level', 0x1af4)
        self.declare_parameter('imu_frame', 'imu_link')

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        motor_type = self.get_parameter('motor_type').value
        battery_level = self.get_parameter('battery_level').value
        self.imu_frame = self.get_parameter('imu_frame').value

        # Initialize board
        try:
            self.board = Board(device=serial_port, baudrate=baud_rate)
            self.board.enable_reception()
            time.sleep(0.1)
            self.board.set_motor_type(motor_type)
            time.sleep(0.1)
            self.board.set_battery_level(battery_level)
            self.get_logger().info(
                f'Board connected: {serial_port} @ {baud_rate}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '~/imu_raw', 10)
        self.battery_pub = self.create_publisher(UInt16, '~/battery', 10)
        self.button_pub = self.create_publisher(ButtonState, '~/button', 10)
        self.joy_pub = self.create_publisher(Joy, '~/joy', 10)
        # 舵机全量上报（0x08 主动推送/查询触发）：位置(已减offset)/温度/电压
        self.bus_servo_states_pub = self.create_publisher(
            BusServoStates, '~/bus_servo/states', 10)

        # Subscribers
        self.create_subscription(
            MotorsState, '~/set_motor', self._set_motor_cb, 10)
        self.create_subscription(
            ServosPosition, '~/bus_servo/set_position', self._set_bus_servo_cb, 10)
        self.create_subscription(
            SetBusServoState, '~/bus_servo/set_state', self._set_bus_servo_state_cb, 10)
        self.create_subscription(
            SetPWMServoState, '~/pwm_servo/set_state', self._set_pwm_servo_state_cb, 10)
        self.create_subscription(
            LedState, '~/set_led', self._set_led_cb, 10)
        self.create_subscription(
            BuzzerState, '~/set_buzzer', self._set_buzzer_cb, 10)
        self.create_subscription(
            OLEDState, '~/set_oled', self._set_oled_cb, 10)
        self.create_subscription(
            RGBStates, '~/set_rgb', self._set_rgb_cb, 10)

        # Services
        self.create_service(
            GetBusServoState, '~/bus_servo/get_state', self._get_bus_servo_state_cb)
        self.create_service(
            GetPWMServoState, '~/pwm_servo/get_state', self._get_pwm_servo_state_cb)
        self.create_service(
            Trigger, '~/init_finish', self._init_finish_cb)

        # Periodic data publishing
        self.create_timer(0.02, self._pub_imu)       # 50 Hz
        self.create_timer(1.0, self._pub_battery)    # 1 Hz
        self.create_timer(0.1, self._pub_button)     # 10 Hz
        self.create_timer(0.02, self._pub_gamepad)   # 50 Hz
        self.create_timer(0.02, self._pub_bus_servo_states)  # 50 Hz 消费 0x08 上报

        # Load servo offsets from config (only read file, no STM32 commands)
        self.servo_offsets = self._load_servo_offsets()

        self.get_logger().info('car1_base_driver node started')

    def _load_servo_offsets(self):
        """Read servo offsets from YAML config file. Does NOT send any commands to STM32."""
        offsets = {}
        try:
            try:
                config_path = os.path.join(
                    get_package_share_directory('calibration'), 'config', 'servo_offsets.yaml')
            except Exception:
                config_path = os.path.expanduser(
                    '~/ros2_ws/src/xflz_eai_car1/calibration/config/servo_offsets.yaml')

            if not os.path.exists(config_path):
                return offsets

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            if not config or 'servo_offsets' not in config:
                return offsets

            for key, offset in config['servo_offsets'].items():
                if key.startswith('servo_'):
                    servo_id = int(key.split('_')[1])
                    offsets[servo_id] = int(offset)

            if offsets:
                self.get_logger().info(f'Loaded servo offsets: {offsets}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load servo offsets: {e}')

        return offsets

    # --- Publisher callbacks ---

    def _pub_imu(self):
        data = self.board.get_imu()
        if data is None:
            return
        ax, ay, az, gx, gy, gz = data
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        # Firmware sends m/s² for accel and rad/s for gyro (BMI088 scale corrected as of 2024)
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        # Covariances MUST be non-zero, otherwise robot_localization EKF cannot
        # invert the measurement covariance and silently drops the IMU data.
        # Diagonal = variance (std² ); off-diagonal = 0.
        msg.angular_velocity_covariance = [
            0.0025, 0.0,    0.0,
            0.0,    0.0025, 0.0,
            0.0,    0.0,    0.0025,
        ]
        msg.linear_acceleration_covariance = [
            0.04, 0.0,  0.0,
            0.0,  0.04, 0.0,
            0.0,  0.0,  0.04,
        ]
        # No fused orientation from raw gyro/accel; mark orientation unavailable.
        msg.orientation_covariance = [
            -1.0, 0.0, 0.0,
            0.0,  0.0, 0.0,
            0.0,  0.0, 0.0,
        ]
        self.imu_pub.publish(msg)

    def _pub_battery(self):
        data = self.board.get_battery()
        if data is not None:
            msg = UInt16()
            msg.data = data
            self.battery_pub.publish(msg)

    def _pub_button(self):
        data = self.board.get_button()
        if data is not None:
            msg = ButtonState()
            msg.id = data[0]
            msg.state = data[1]
            self.button_pub.publish(msg)

    def _pub_gamepad(self):
        data = self.board.get_gamepad()
        if data is None:
            return
        axes, buttons = data
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [float(a) for a in axes]
        msg.buttons = buttons
        self.joy_pub.publish(msg)

    def _pub_bus_servo_states(self):
        """消费下位机 0x08 全量装上(位置/温度/电压)并发布，供矢节/机械臂被动读位使用。

        上报的 PWM 为物理 RAW 值（未减 offset），而下发指令时加分了 offset，
        故这里减 offset 恢复"逻辑位置"，与 /joint_states 及 IK/URDF 保持一致。
        """
        data = self.board.get_bus_servo_states()
        if data is None:
            return
        msg = BusServoStates()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        for s in data:
            sid = s['id']
            raw = s['pwm']
            logical = max(0, min(65535, raw - self.servo_offsets.get(sid, 0)))
            msg.id.append(sid)
            msg.valid.append(s['valid'])
            msg.position.append(logical)
            msg.temperature.append(s['temp'])
            msg.voltage.append(float(s['voltage']))
        self.bus_servo_states_pub.publish(msg)

    # --- Subscriber callbacks ---

    def _set_motor_cb(self, msg):
        speeds = [[m.id, m.rps] for m in msg.data]
        if speeds:
            self.board.set_motor_speed(speeds)

    def _set_bus_servo_cb(self, msg):
        # Apply offset compensation: position_actual = position_target + offset
        positions = [[p.id, p.position + self.servo_offsets.get(p.id, 0)]
                     for p in msg.position]
        if positions:
            self.board.bus_servo_set_position(msg.duration, positions)

    def _set_bus_servo_state_cb(self, msg):
        for state in msg.state:
            sid_list = state.present_id if state.present_id else state.target_id
            for i, sid in enumerate(state.enable_torque):
                target = state.target_id[i] if i < len(state.target_id) else sid
                self.board.bus_servo_enable_torque(target, sid != 0)
            if state.offset and sid_list:
                for i, offset_val in enumerate(state.offset):
                    if i < len(sid_list):
                        self.board.bus_servo_set_offset(sid_list[i], offset_val)
            if state.save_offset and sid_list:
                for i, save_flag in enumerate(state.save_offset):
                    if save_flag and i < len(sid_list):
                        self.board.bus_servo_save_offset(sid_list[i])

    def _set_pwm_servo_state_cb(self, msg):
        for state in msg.state:
            positions = list(zip(state.id, state.position))
            if positions:
                self.board.pwm_servo_set_position(msg.duration, positions)

    def _set_led_cb(self, msg):
        self.board.set_led(msg.on_time, msg.off_time, msg.repeat, msg.id)

    def _set_buzzer_cb(self, msg):
        self.board.set_buzzer(msg.freq, msg.on_time, msg.off_time, msg.repeat)

    def _set_oled_cb(self, msg):
        self.board.set_oled_text(msg.index, msg.text)

    def _set_rgb_cb(self, msg):
        pixels = [[s.index, s.red, s.green, s.blue] for s in msg.states]
        if pixels:
            self.board.set_rgb(pixels)

    # --- Service callbacks ---

    def _get_bus_servo_state_cb(self, request, response):
        response.success = True
        response.state = []
        for cmd in request.cmd:
            state = BusServoState()
            sid = cmd.id
            state.present_id = [sid]
            state.target_id = [sid]
            if cmd.get_position:
                result = self.board.bus_servo_read_position(sid)
                # Subtract the offset that _set_bus_servo_cb adds on the write side
                # (line ~191: target + offset). The servo reports the RAW pulse it is
                # physically at, so to recover the logical position the controller
                # commanded -- and keep /joint_states consistent with IK/URDF -- we must
                # subtract the same offset here. Without this, joint_states (and thus the
                # TF tree used by visual grasp) is biased by the offset (e.g. servo_2=20
                # pulse ~= 0.047 rad on arm_joint3).
                if result:
                    pos = result[0] - self.servo_offsets.get(sid, 0)
                    state.position = [max(0, min(65535, pos))]
                else:
                    state.position = [0]
            if cmd.get_offset:
                result = self.board.bus_servo_read_offset(sid)
                state.offset = [result[0]] if result else [0]
            if cmd.get_voltage:
                result = self.board.bus_servo_read_vin(sid)
                state.voltage = [result[0]] if result else [0]
            if cmd.get_temperature:
                result = self.board.bus_servo_read_temp(sid)
                state.temperature = [result[0]] if result else [0]
            response.state.append(state)
        return response

    def _get_pwm_servo_state_cb(self, request, response):
        response.success = True
        response.state = []
        # Simplified: return position for each requested servo
        return response

    def _init_finish_cb(self, request, response):
        response.success = True
        response.message = 'car1_base_driver ready'
        return response

    def destroy_node(self):
        """发送四轮全零速，避免程序关闭后轮子因保持最后 PWM 而继续转动。

        STM32 为开环控制、无看门狗，若仅靠 odom_publisher 经话题发零速命令，
        关闭时跨节点投递存在时序竞争（base_driver 可能先被销毁来不及消费），
        停止命令会丢失、轮子持续转动。故在此串口层直接再发一次零速兜底。
        """
        try:
            self.board.set_motor_speed([[1, 0.0], [2, 0.0], [3, 0.0], [4, 0.0]])
            self.get_logger().info('Sent zero-speed command to all motors (shutdown)')
        except Exception as e:
            self.get_logger().warn(f'关闭时发送零速命令失败: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
