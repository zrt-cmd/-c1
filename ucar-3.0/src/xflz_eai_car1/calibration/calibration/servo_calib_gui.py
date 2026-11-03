#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂舵机零位校准 GUI 工具。

界面功能（对应教程"零位校准"一节）：
- 5 个关节独立控制面板（joint1 ~ joint5，对应舵机 ID 0~4）
- PWM 滑块（500-2500），拖动滑块实时下发舵机位置
- 实时位置显示（rad / deg，270° 舵机，PWM_TO_RAD = 4.712/2000）
- 全部回中：所有舵机缓慢移动到中心 1500
- 保存偏移：把当前 PWM 与中心 1500 的偏差写入 servo_offsets.yaml

用法：
  ros2 launch calibration servo_calib.launch.py
  # 或单独运行：
  ros2 run calibration servo_calib_gui
"""
import math
import os
import queue
import threading
import time

import tkinter as tk
from tkinter import ttk, messagebox
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from car1_base_driver_msgs.msg import ServosPosition, ServoPosition
from car1_base_driver_msgs.srv import GetBusServoState
from car1_base_driver_msgs.msg import GetBusServoCmd

# ============================================================================
# 常量（与 car1_kinematics/transform.py 保持一致）
# ============================================================================
CENTER_PWM = 1500       # 理论中位
PWM_MIN = 500           # 滑块下限
PWM_MAX = 2500          # 滑块上限
PWM_TO_RAD = 4.712 / 2000.0   # 270° 舵机：rad / PWM
MOVE_DURATION = 1.0     # 单关节移动时长（秒）

# joint1~joint5 对应的舵机名称（来自 SERVO_CONFIG）
JOINT_NAMES = {
    0: 'base_rotation',
    1: 'big_arm',
    2: 'small_arm',
    3: 'wrist',
    4: 'end_rotation',
}


def pwm_to_angle(pwm):
    """PWM -> 关节角（rad），方向 +1、offset 0。"""
    return (pwm - CENTER_PWM) * PWM_TO_RAD


class ServoCalibNode(Node):
    """ROS2 节点：负责发布舵机位置、读取 STM32 实际位置。"""

    def __init__(self, servo_ids=None):
        super().__init__('servo_calib_gui')
        # 声明参数（可由 launch 传入，例如 servo_ids: [0,1,2,3,4]）
        self.declare_parameter('servo_ids', servo_ids or [0, 1, 2, 3, 4])
        self.declare_parameter('center_position', CENTER_PWM)
        servo_ids = self.get_parameter('servo_ids').value
        self.servo_ids = list(servo_ids)
        # 记录各舵机当前命令位置（保存偏移时的 fallback）
        self.positions = {sid: CENTER_PWM for sid in self.servo_ids}

        self.position_pub = self.create_publisher(
            ServosPosition, '/car1_base_driver/bus_servo/set_position', 10)
        # 订阅自己发布的话题，跟踪位置命令
        self.create_subscription(
            ServosPosition, '/car1_base_driver/bus_servo/set_position',
            self._position_cb, 10)
        # 读取 STM32 舵机实际位置的客户端
        self.get_state_client = self.create_client(
            GetBusServoState, '/car1_base_driver/bus_servo/get_state')

        self._publish_lock = threading.Lock()

    def _position_cb(self, msg):
        for pos in msg.position:
            if pos.id in self.positions:
                self.positions[pos.id] = pos.position

    def publish_pwm(self, servo_id, pwm, duration=MOVE_DURATION):
        """发布单个舵机的 PWM 位置。"""
        pwm = int(max(PWM_MIN, min(PWM_MAX, pwm)))
        with self._publish_lock:
            msg = ServosPosition()
            msg.duration = float(duration)
            pos = ServoPosition()
            pos.id = int(servo_id)
            pos.position = int(pwm)   # ServoPosition.position 是 uint16
            msg.position.append(pos)
            self.positions[servo_id] = float(pwm)
            self.position_pub.publish(msg)
        return pwm

    def publish_all_to_center(self, duration=3.0):
        """所有舵机依次缓慢移动到中心。"""
        for sid in self.servo_ids:
            self.publish_pwm(sid, CENTER_PWM, duration=duration)
            time.sleep(duration + 0.3)

    @staticmethod
    def _wait_future(future, timeout_sec=2.0):
        """等待 rclpy Future 完成（Humble 的 result() 不支持 timeout）。"""
        deadline = time.time() + timeout_sec
        while not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.05)
        return True

    def read_position(self, servo_id):
        """从 STM32 读取单个舵机的实际 PWM 位置；失败返回 None。"""
        if not self.get_state_client.wait_for_service(timeout_sec=2.0):
            return None
        req = GetBusServoState.Request()
        cmd = GetBusServoCmd()
        cmd.id = int(servo_id)
        cmd.get_position = 1
        req.cmd.append(cmd)
        future = self.get_state_client.call_async(req)
        if not self._wait_future(future, timeout_sec=2.0):
            return None
        result = future.result()
        if result.state and result.state[0].position:
            pos = int(result.state[0].position[0])
            # base_driver 在板子无应答时返回 0（舵机有效范围是 500~2500），
            # 0 视为读取失败，避免把无效值当成"读到了"。
            if pos == 0:
                return None
            return pos
        return None


class ServoCalibApp:
    """tkinter 图形界面。"""

    def __init__(self, node):
        self.node = node
        self.servo_ids = node.servo_ids
        self.sliders = {}       # sid -> tk.Scale
        self.pwm_labels = {}    # sid -> Label
        self.angle_labels = {}  # sid -> Label
        self.offset_spins = {}  # sid -> Spinbox（offset 输入框）
        # 各关节 offset（从 servo_offsets.yaml 载入，Apply/Save All 使用）
        self.offsets = self._load_offsets()

        self.root = tk.Tk()
        self.root.title('Servo Calibration - 舵机零位校准')
        self.root.resizable(False, False)

        # 线程安全：工作线程把 UI 更新函数放入队列，主线程轮询执行
        self._ui_queue = queue.Queue()
        self._poll_ui_queue()

        self._build_ui()
        self._set_status('读取PWM(读实际值)→取PWM(算offset)→Apply→Save All')

    # ------------------------------------------------------------ offset 载入
    def _load_offsets(self):
        """从 servo_offsets.yaml 读取各关节当前 offset（找不到则全为 0）。"""
        offsets = {sid: 0 for sid in self.servo_ids}
        candidates = [
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'config', 'servo_offsets.yaml'),
        ]
        try:
            candidates.append(os.path.join(
                get_package_share_directory('calibration'), 'config', 'servo_offsets.yaml'))
        except Exception:
            pass
        for path in candidates:
            try:
                with open(path) as f:
                    data = yaml.safe_load(f) or {}
                servo_offsets = data.get('servo_offsets', {})
                for sid in self.servo_ids:
                    key = f'servo_{sid}'
                    if key in servo_offsets:
                        offsets[sid] = int(servo_offsets[key])
                return offsets
            except Exception:
                continue
        return offsets

    # -------------------------------------------------- 线程安全的 UI 更新
    def _schedule_ui(self, func):
        """从任意线程调用：把要在主线程执行的函数放入队列。"""
        self._ui_queue.put(func)

    def _poll_ui_queue(self):
        try:
            while True:
                func = self._ui_queue.get_nowait()
                func()
        except queue.Empty:
            pass
        self.root.after(50, self._poll_ui_queue)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        header = ttk.Frame(self.root, padding=8)
        header.pack(fill='x')
        ttk.Label(header, text='car1 机械臂舵机零位校准（joint1~joint5）',
                  font=('', 12, 'bold')).pack(side='left')

        table = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        table.pack(fill='both', expand=True)

        # 表头
        ttk.Label(table, text='关节', width=10, anchor='w').grid(row=0, column=0, padx=4, pady=2)
        ttk.Label(table, text='PWM 滑块 (500-2500)', anchor='center').grid(row=0, column=1, padx=4)
        ttk.Label(table, text='PWM', width=7, anchor='e').grid(row=0, column=2, padx=4)
        ttk.Label(table, text='角度 (rad / deg)', width=18, anchor='w').grid(row=0, column=3, padx=4)
        ttk.Label(table, text='Offset (=PWM-1500)', width=14, anchor='center').grid(row=0, column=4, padx=4)
        ttk.Label(table, text='操作', width=22).grid(row=0, column=5, padx=4)

        for i, sid in enumerate(self.servo_ids):
            row = i + 1
            name = JOINT_NAMES.get(sid, f'servo_{sid}')
            ttk.Label(table, text=f'joint{i+1}\n({name})', width=12,
                      anchor='w', justify='left').grid(row=row, column=0, padx=4, pady=4, sticky='w')

            scale = tk.Scale(table, from_=PWM_MIN, to=PWM_MAX, orient='horizontal',
                             resolution=1, length=300, showvalue=False,
                             command=lambda v, s=sid: self._on_slider(s, int(float(v))))
            scale.set(CENTER_PWM)
            scale.grid(row=row, column=1, padx=4, pady=4)
            scale.bind('<ButtonRelease-1>', lambda e, s=sid: self._publish_joint(s))
            self.sliders[sid] = scale

            pwm_lbl = ttk.Label(table, text=f'{CENTER_PWM}', width=7, anchor='e')
            pwm_lbl.grid(row=row, column=2, padx=4)
            self.pwm_labels[sid] = pwm_lbl

            angle_lbl = ttk.Label(table, text='', width=18, anchor='w')
            angle_lbl.grid(row=row, column=3, padx=4)
            self.angle_labels[sid] = angle_lbl

            # Offset 输入框（可正可负，范围 -200 ~ 200）
            spin = tk.Spinbox(table, from_=-200, to=200, width=7, justify='center')
            spin.delete(0, 'end')
            spin.insert(0, str(self.offsets.get(sid, 0)))
            spin.grid(row=row, column=4, padx=4, pady=4)
            self.offset_spins[sid] = spin

            # Apply / 取PWM / 读取PWM
            btns_row = ttk.Frame(table)
            btns_row.grid(row=row, column=5, padx=4, pady=4, sticky='w')
            ttk.Button(btns_row, text='Apply', width=6,
                       command=lambda s=sid: self._on_apply_offset(s)).pack(side='left', padx=1)
            ttk.Button(btns_row, text='取PWM', width=6,
                       command=lambda s=sid: self._on_take_pwm(s)).pack(side='left', padx=1)
            ttk.Button(btns_row, text='读取PWM', width=8,
                       command=lambda s=sid: self._on_read_pwm(s)).pack(side='left', padx=1)

            self._refresh_joint_display(sid)

        btns = ttk.Frame(self.root, padding=8)
        btns.pack(fill='x')
        ttk.Button(btns, text='全部回中 (1500)',
                   command=self._on_center_all).pack(side='left', padx=4)
        ttk.Button(btns, text='保存全部偏移 (Save All)',
                   command=self._on_save_all).pack(side='left', padx=4)
        ttk.Button(btns, text='退出', command=self.root.destroy).pack(side='right', padx=4)

        self.status_var = tk.StringVar()
        status = ttk.Label(self.root, textvariable=self.status_var, relief='sunken',
                           anchor='w', padding=4)
        status.pack(fill='x', side='bottom')

    # ------------------------------------------------------------- 交互逻辑
    def _on_slider(self, sid, pwm):
        """滑块移动：实时刷新 PWM / 角度显示（不动舵机，松手才下发）。"""
        self._refresh_joint_display(sid, pwm)

    def _publish_joint(self, sid):
        """滑块松手：下发该关节 PWM。"""
        pwm = int(self.sliders[sid].get())
        self.node.publish_pwm(sid, pwm)
        self._refresh_joint_display(sid, pwm)
        self._set_status(f'joint{self.servo_ids.index(sid)+1} (servo {sid}) -> PWM {pwm}')

    def _refresh_joint_display(self, sid, pwm=None):
        if pwm is None:
            pwm = int(self.sliders[sid].get())
        self.pwm_labels[sid].config(text=f'{pwm}')
        rad = pwm_to_angle(pwm)
        deg = math.degrees(rad)
        self.angle_labels[sid].config(text=f'{rad:.4f} / {deg:.2f}°')

    def _on_center_all(self):
        self._set_status('正在将所有舵机移动到中心 1500，请稍候...')
        self.root.update_idletasks()
        worker = threading.Thread(target=self._center_all_worker, daemon=True)
        worker.start()

    def _center_all_worker(self):
        try:
            self.node.publish_all_to_center()
        except Exception as e:
            self._schedule_ui(lambda err=e: self._set_status(f'回中失败：{err}'))
            return
        for sid in self.servo_ids:
            self._schedule_ui(lambda s=sid: self._set_slider_value(s, CENTER_PWM))
        self._schedule_ui(lambda: self._set_status('全部舵机已回到中心 1500'))

    def _set_slider_value(self, sid, pwm):
        self.sliders[sid].set(pwm)
        self._refresh_joint_display(sid, pwm)

    def _on_take_pwm(self, sid):
        """把当前滑块 PWM 的偏差（PWM-1500）填入 Offset 输入框。"""
        pwm = int(self.sliders[sid].get())
        offset = pwm - CENTER_PWM
        self._set_offset_value(sid, offset)
        self._set_status(
            f'joint{self.servo_ids.index(sid)+1}: 滑块 PWM={pwm} → offset={offset:+d}（填好后点 Apply）')

    def _on_read_pwm(self, sid):
        """读取PWM：从 STM32 读取该舵机实际 PWM，更新滑块/显示。"""
        self._set_status(f'正在读取 joint{self.servo_ids.index(sid)+1} (servo {sid}) 实际 PWM...')
        worker = threading.Thread(target=self._read_pwm_worker, args=(sid,), daemon=True)
        worker.start()

    def _read_pwm_worker(self, sid):
        try:
            pwm = self.node.read_position(sid)
        except Exception as e:
            self._schedule_ui(lambda err=e, s=sid: self._set_status(f'读取 servo {s} 失败：{err}'))
            return
        if pwm is None:
            self._schedule_ui(lambda s=sid: self._set_status(
                f'读取 servo {s} 失败（base_driver 服务无响应），请确认串口通信正常'))
            return
        self._schedule_ui(lambda s=sid, p=pwm: self._apply_read_pwm(s, p))

    def _apply_read_pwm(self, sid, pwm):
        pwm = int(max(PWM_MIN, min(PWM_MAX, pwm)))
        self._set_slider_value(sid, pwm)
        self._set_status(
            f'joint{self.servo_ids.index(sid)+1} (servo {sid}) 实际 PWM={pwm}，'
            f'可点"取PWM"填入 offset')

    def _on_apply_offset(self, sid):
        """Apply：记录该关节 offset，并把滑块定位到机械中位 PWM=1500+offset 供验证。"""
        try:
            offset = int(self.offset_spins[sid].get())
        except ValueError:
            self._set_status(f'joint{self.servo_ids.index(sid)+1}: offset 必须是整数')
            return
        offset = max(-200, min(200, offset))
        self._set_offset_value(sid, offset)
        self.offsets[sid] = offset
        center_pwm = CENTER_PWM + offset
        self._set_slider_value(sid, center_pwm)
        self._set_status(
            f'joint{self.servo_ids.index(sid)+1} (servo {sid}): offset={offset:+d} 已应用，'
            f'滑块定位到机械中位 PWM={center_pwm}（拖动可验证，最后点 Save All）')

    def _set_offset_value(self, sid, offset):
        spin = self.offset_spins[sid]
        spin.delete(0, 'end')
        spin.insert(0, str(offset))

    def _on_save_all(self):
        """Save All：把各行 Offset 输入框的值写入 servo_offsets.yaml。"""
        try:
            offsets = {}
            for sid in self.servo_ids:
                offsets[sid] = int(self.offset_spins[sid].get())
        except ValueError as e:
            self._show_save_error(f'offset 必须是整数：{e}')
            return
        worker = threading.Thread(target=self._save_all_worker, args=(offsets,), daemon=True)
        worker.start()

    def _save_all_worker(self, offsets):
        """后台线程：写入 yaml（src + install 两处）。"""
        try:
            self.offsets.update(offsets)
            saved = self._write_offsets(offsets)
            if not saved:
                raise RuntimeError('没有可写的配置文件路径')

            info_lines = [
                f'  Servo {sid}: offset={offsets[sid]:+d}'
                for sid in self.servo_ids]
            msg = '偏移已保存:\n' + '\n'.join(info_lines) + '\n\n保存位置:\n' + '\n'.join(saved)
            self._schedule_ui(lambda m=msg: self._show_save_result(m))
        except Exception as e:
            self._schedule_ui(lambda err=e: self._show_save_error(str(err)))

    def _write_offsets(self, offsets):
        """写入 src 与 install 两处 servo_offsets.yaml，返回成功路径列表。"""
        config_data = {'servo_offsets': {f'servo_{sid}': offsets[sid] for sid in self.servo_ids}}
        paths = [
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'config', 'servo_offsets.yaml'),
        ]
        try:
            paths.append(os.path.join(
                get_package_share_directory('calibration'), 'config', 'servo_offsets.yaml'))
        except Exception:
            pass

        saved = []
        for path in paths:
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, 'w') as f:
                    yaml.dump(config_data, f, default_flow_style=False)
                saved.append(path)
            except Exception as e:
                self.node.get_logger().warn(f'Could not save to {path}: {e}')
        return saved

    # ------------------------------------------------------------ 状态显示
    def _set_status(self, text):
        self.status_var.set(text)

    def _show_save_result(self, msg):
        self._set_status('偏移已保存')
        messagebox.showinfo('保存偏移', msg)

    def _show_save_error(self, err):
        self._set_status(f'保存失败：{err}')
        messagebox.showerror('保存偏移失败', err)

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = ServoCalibNode(servo_ids=[0, 1, 2, 3, 4])

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = ServoCalibApp(node)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
