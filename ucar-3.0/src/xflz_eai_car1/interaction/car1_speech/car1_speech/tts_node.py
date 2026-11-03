"""
Text-to-Speech (TTS) Node

实时语音合成 + 播放（sherpa-onnx Matcha-TTS 离线模型 + ALSA aplay 管道）。

实现参考 interaction/car1_interaction/tools/voice_interaction.py 的既有方案：
  - OfflineTts(Matcha acoustic + Vocos vocoder) → float32 采样
  - 增益 soft-clip（tanh，>1.0 防削波失真）→ 双声道 S16_LE PCM
  - 持久化 aplay stdin 管道播放（无每句一次进程开销），支持 stop 中断
  - 初始化时用 audio_utils.setup_playback 设置 ES8388 喇叭音量

音量/合成参数全部可由 tts.yaml 配置，也可由 launch 参数覆盖。
"""

import math
import os
import subprocess
import sys
import threading
import time
import wave

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

# sherpa_onnx 依赖可选装于 ros2_venv；把 venv site-packages 提前加进 sys.path
_VENV_SITE = '/home/iflytek/ros2_venv/lib/python3.10/site-packages'
if os.path.isdir(_VENV_SITE) and _VENV_SITE not in sys.path:
    sys.path.insert(0, _VENV_SITE)

import sherpa_onnx  # noqa: E402

# 音箱/录音音量工具（audio_utils：mixer 设置 + 硬要立体声播放）
_AUDIO_UTILS = '/home/iflytek/ros2_ws/src/xflz_eai_car1/interaction/car1_interaction/tools'
if _AUDIO_UTILS not in sys.path:
    sys.path.insert(0, _AUDIO_UTILS)

try:
    from audio_utils import setup_playback, setup_capture, switch_input_to_mic, apply_capture_volume
    _HAS_AUDIO_UTILS = True
except Exception as e:  # pragma: no cover
    _HAS_AUDIO_UTILS = False


class Playback:
    """向持久化 aplay 进程写 S16_LE 双声道 PCM（支持打断）。"""

    @property
    def failed(self):
        return self._failed

    def __init__(self, node, device='plughw:0,0', sample_rate=16000, channels=2):
        self._node = node
        self.device = device
        self.sample_rate = sample_rate
        self.channels = channels
        self._proc = None
        self._lock = threading.Lock()
        self._failed = False
        self._open()
        if self._failed or (self._proc and self._proc.poll() is not None):
            self._kill_conflicting_playback()
            time.sleep(0.5)
            self._open()
            if self._failed or (self._proc and self._proc.poll() is not None):
                self._failed = True
                self._node.get_logger().error(f'无法打开播放设备 {device}，播放禁用')

    def _kill_conflicting_playback(self):
        try:
            subprocess.run('pkill -9 aplay', shell=True, capture_output=True, timeout=2)
        except Exception:
            pass

    def _open(self):
        try:
            self._proc = subprocess.Popen(
                ['aplay', '-t', 'raw', '-f', 'S16_LE',
                 '-c', str(self.channels), '-r', str(self.sample_rate),
                 '-D', self.device, '-q', '-'],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
            time.sleep(0.1)
            if self._proc.poll() is not None:
                self._failed = True
            else:
                self._failed = False
        except Exception as e:
            self._node.get_logger().error(f'aplay 启动失败: {e}')
            self._failed = True

    def write(self, pcm_bytes):
        if self._failed:
            return
        with self._lock:
            try:
                self._proc.stdin.write(pcm_bytes)
                self._proc.stdin.flush()
            except (BrokenPipeError, OSError) as e:
                self._node.get_logger().warn(f'播放失败 ({e})，重启 aplay')
                self._reopen()
                if not self._failed:
                    try:
                        self._proc.stdin.write(pcm_bytes)
                        self._proc.stdin.flush()
                    except Exception as e2:
                        self._node.get_logger().error(f'重启后仍失败: {e2}')
                        self._failed = True

    def _reopen(self):
        with self._lock:
            self._close_proc()
            self._open()

    def _close_proc(self):
        if self._proc:
            try:
                self._proc.stdin.close()
            except Exception:
                pass
            try:
                self._proc.terminate()
            except Exception:
                pass
            self._proc = None

    def interrupt(self):
        with self._lock:
            self._close_proc()

    def close(self):
        self.interrupt()


class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        # ---- 参数 ----
        self.declare_parameter('engine', 'sherpa')   # sherpa=离线 Matcha-TTS
        self.declare_parameter('voice', 'xiaoyan')   # 保留，sherpa 用 sid 表示音色
        # 合成
        self.declare_parameter('model_dir', '/home/iflytek/models/tts')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('gain', 3.0)           # 合成数字增益；>1 用 tanh 软裁（声音更大）
        self.declare_parameter('sid', 0)
        self.declare_parameter('num_threads', 4)
        # 播放
        self.declare_parameter('playback_device', 'plughw:0,0')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('playback_volume', 100)  # 喇叭音量 0-100
        self.declare_parameter('audio_card', 0)
        # 录音音量（仅调 mixer，供降噪/增益参考；不在此节点采集）
        self.declare_parameter('capture_volume', 88)
        self.declare_parameter('mic_boost', True)

        self.engine = self.get_parameter('engine').value
        self.model_dir = self.get_parameter('model_dir').value
        self.speed = float(self.get_parameter('speed').value)
        self.gain = float(self.get_parameter('gain').value)
        self.sid = int(self.get_parameter('sid').value)
        self.num_threads = int(self.get_parameter('num_threads').value)
        self.playback_device = self.get_parameter('playback_device').value
        self.sample_rate = int(self.get_parameter('sample_rate').value)
        self.playback_volume = int(self.get_parameter('playback_volume').value)
        self.audio_card = int(self.get_parameter('audio_card').value)
        self.capture_volume = int(self.get_parameter('capture_volume').value)
        self.mic_boost = self.get_parameter('mic_boost').value

        # ---- 订阅 / 发布 / 服务 ----
        self.text_sub = self.create_subscription(
            String, '/interaction/tts/speak', self._speak_callback, 10)
        self.wav_sub = self.create_subscription(
            String, '/interaction/tts/play_wav', self._play_wav_callback, 10)
        self.is_speaking_pub = self.create_publisher(
            Bool, '/interaction/tts/is_speaking', 10)
        self.stop_srv = self.create_service(
            Trigger, '/interaction/tts/stop', self._stop_callback)

        self._is_speaking = False
        self._stop_requested = threading.Event()
        self._speak_lock = threading.Lock()
        self._playback = None
        self._tts = None

        # ---- 初始化 ----
        self._setup_audio()
        try:
            self._init_tts()
        except Exception as e:
            self.get_logger().error(f'TTS 引擎初始化失败: {e}（播报将不可用）')
            self._tts = None
        self._init_playback()

        self.get_logger().info(
            f'TTS node started. engine={self.engine} voice={self.voice_param} '
            f'speed={self.speed} gain={self.gain} vol={self.playback_volume}')

    @property
    def voice_param(self):
        return self.get_parameter('voice').value

    # ---------- 初始化 ----------

    def _setup_audio(self):
        """设置播放/录音音量（ES8388）。失败不阻塞（仅提示）。"""
        if not _HAS_AUDIO_UTILS:
            self.get_logger().warn('audio_utils 不可用，跳过音量设置')
            return
        try:
            setup_playback(self.audio_card, self.playback_volume)
        except Exception as e:
            self.get_logger().warn(f'设置播放音量失败: {e}')
        try:
            if self.mic_boost:
                setup_capture(self.audio_card, self.capture_volume, mic_boost=True)
            else:
                switch_input_to_mic(self.audio_card)
                apply_capture_volume(self.audio_card, self.capture_volume)
        except Exception as e:
            self.get_logger().warn(f'设置录音音量失败: {e}')

    def _init_tts(self):
        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                    acoustic_model=os.path.join(self.model_dir, 'model-steps-3.onnx'),
                    vocoder=os.path.join(self.model_dir, 'vocos-16khz-univ.onnx'),
                    lexicon=os.path.join(self.model_dir, 'lexicon.txt'),
                    tokens=os.path.join(self.model_dir, 'tokens.txt'),
                    data_dir=os.path.join(self.model_dir, 'espeak-ng-data'),
                ),
                num_threads=self.num_threads,
            ),
            max_num_sentences=1,
            rule_fsts=','.join([
                os.path.join(self.model_dir, 'phone-zh.fst'),
                os.path.join(self.model_dir, 'date-zh.fst'),
                os.path.join(self.model_dir, 'number-zh.fst'),
            ]),
        )
        if not tts_config.validate():
            raise RuntimeError('TTS 配置验证失败（模型文件不完整？）')
        self._tts = sherpa_onnx.OfflineTts(tts_config)
        self.get_logger().info(f'TTS 引擎就绪（Matcha-TTS）: {self.model_dir}')

    def _init_playback(self):
        self._playback = Playback(self, device=self.playback_device,
                                  sample_rate=self.sample_rate, channels=2)

    # ---------- 合成 ----------

    def _text_to_pcm(self, text):
        """合成 → 增益 soft-clip → 双声道 S16_LE PCM bytes。"""
        audio = self._tts.generate(text, sid=self.sid, speed=self.speed)
        samples = np.asarray(audio.samples, dtype=np.float32)
        if self.gain > 1.0:
            samples = np.tanh(samples * self.gain)
        else:
            samples = (samples * self.gain).clip(-1.0, 1.0)
        mono = (samples * 32767).astype(np.int16)
        stereo = np.column_stack([mono, mono]).ravel()
        return stereo.tobytes()

    # ---------- 回调 ----------

    def _speak_callback(self, msg):
        text = msg.data
        if not text:
            return
        self.get_logger().info(f'Speaking: "{text}"')
        self._speak(text)

    def _play_wav_callback(self, msg):
        path = msg.data
        if not path or not os.path.isfile(path):
            self.get_logger().error(f'提示音不存在: "{path}"')
            return
        if self._playback is None or self._playback.failed:
            self.get_logger().warn('TTS 播放不可用，跳过提示音')
            return
        try:
            with wave.open(path, 'rb') as wav:
                if wav.getsampwidth() != 2:
                    self.get_logger().error('提示音必须是 16-bit PCM')
                    return
                if wav.getframerate() != self.sample_rate:
                    self.get_logger().error(
                        f'提示音采样率 {wav.getframerate()} 与播放 {self.sample_rate} 不一致')
                    return
                pcm = np.frombuffer(wav.readframes(wav.getnframes()), dtype=np.int16)
                if wav.getnchannels() == 1:
                    pcm = np.column_stack([pcm, pcm]).ravel()
                elif wav.getnchannels() != 2:
                    self.get_logger().error('提示音必须是 mono 或 stereo')
                    return
        except Exception as e:
            self.get_logger().error(f'读取提示音失败: {e}')
            return

        self.get_logger().info(f'Playing prompt: {os.path.basename(path)}')
        with self._speak_lock:
            self._stop_requested.clear()
            self._set_speaking(True)
            try:
                self._playback.write(pcm.tobytes())
            finally:
                self._set_speaking(False)

    def _speak(self, text):
        if self._tts is None or self._playback is None:
            self.get_logger().warn(f'TTS 不可用，跳过播报: "{text}"')
            return
        with self._speak_lock:
            self._stop_requested.clear()
            self._set_speaking(True)
            try:
                try:
                    pcm = self._text_to_pcm(text)
                except Exception as e:
                    self.get_logger().error(f'合成失败: {e}')
                    return
                if not self._stop_requested.is_set() and pcm:
                    self._playback.write(pcm)
            finally:
                self._set_speaking(False)

    def _set_speaking(self, val):
        self._is_speaking = val
        msg = Bool()
        msg.data = val
        self.is_speaking_pub.publish(msg)

    def _stop_callback(self, request, response):
        if self._is_speaking:
            self.get_logger().info('Stopping TTS playback')
            self._stop_requested.set()
            if self._playback is not None:
                self._playback.interrupt()
            self._set_speaking(False)
            response.success = True
            response.message = 'TTS stopped'
        else:
            response.success = False
            response.message = 'Not speaking'
        return response

    def destroy_node(self):
        if self._playback is not None:
            self._playback.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
