"""Wake Word + Command Detection Node（语音触发任务启动）

流程（对齐 eai_factory voice_node）：
  1. KWS 持续监听唤醒词（小飞小飞）→ 命中发布 /interaction/wake_word/detected（事件通知）
  2. 播放提示音 + 清回声 → ASR 识别下一句命令
  3. LLM 意图分类：把命令文本分类为 start / none（intent_hints 配置驱动）
  4. 命中 start → 调用 /mission_orchestrator/start service 启动任务
  5. 订阅 /car1_brics_task/state：running=true 暂停唤醒，running=false 恢复

引擎：sherpa-onnx KeywordSpotter（zipformer_kws）+ OnlineRecognizer（zipformer_onnx, CPU）
LLM：内嵌启动 flask_server（Qwen3.5-0.8B rkllm）做意图分类
麦克风：ES8388
"""
import json
import logging
import os
import queue
import re
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from queue import Queue, Empty

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(name)s: %(message)s')

# sherpa_onnx / requests 只装在 /home/iflytek/ros2_venv；ros2 run 的 shebang 用系统
# python3，这里把 venv site-packages 提前加进 sys.path，确保 import 成功。
_VENV_SITE = '/home/iflytek/ros2_venv/lib/python3.10/site-packages'
if os.path.isdir(_VENV_SITE) and _VENV_SITE not in sys.path:
    sys.path.insert(0, _VENV_SITE)

import rclpy
import numpy as np
import sherpa_onnx
from rclpy.node import Node
from std_msgs.msg import Bool, String

# 复用麦克风增益/切换工具
_AUDIO_UTILS = Path('/home/iflytek/ros2_ws/src/xflz_eai_car1/interaction/car1_interaction/tools')
if str(_AUDIO_UTILS) not in sys.path:
    sys.path.insert(0, str(_AUDIO_UTILS))

SAMPLE_RATE = 16000
LLM_SERVER_DIR = '/home/iflytek/ros2_ws/src/xflz_eai_car1/interaction/car1_interaction/llm_server'

log = logging.getLogger('wake')
log_kws = logging.getLogger('KWS')
log_asr = logging.getLogger('ASR')
log_llm = logging.getLogger('LLM')


# ---------------------------------------------------------------------------
# ASR 结果有效性过滤（对齐 eai_factory voice_node：丢弃品牌/单字母/短 ASCII
# 误识别，避免拿乱码喂 LLM）
# ---------------------------------------------------------------------------

_ASR_BLACKLIST = {
    'VIVO', 'IVO', 'VI', 'V I', 'V IVO', 'VI VO', 'OPPO', 'OP PO',
    'HUAWEI', 'XIAOMI', 'APPLE', 'HI', 'HELLO', 'HEY', 'BYE',
    'OK', 'YES', 'NO', 'UH', 'UM', 'AH', 'EH', 'OH',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K',
    'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V',
    'W', 'X', 'Y', 'Z',
}


def _is_valid_asr_result(text):
    if not text:
        return False
    stripped = text.strip()
    if len(stripped) < 2:
        return False
    normalized = re.sub(r'\s+', '', stripped).upper()
    if normalized in _ASR_BLACKLIST:
        return False
    is_pure_ascii = all(ord(c) < 128 for c in stripped)
    has_space = ' ' in stripped
    if is_pure_ascii and not has_space and len(stripped) < 6:
        return False
    return True


class LLMClient:
    """OpenAI 兼容流式客户端（请求本地 flask_server）。"""
    def __init__(self, base_url, timeout=120):
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        import requests
        self.session = requests.Session()
        self.session.keep_alive = False

    def chat(self, messages, temperature=0.1, top_p=0.9,
             max_tokens=20, repeat_penalty=1.1) -> str:
        data = {
            'model': 'rkllm',
            'messages': messages,
            'temperature': temperature,
            'top_p': top_p,
            'max_tokens': max_tokens,
            'repeat_penalty': repeat_penalty,
            'stream': False,
        }
        resp = self.session.post(
            f'{self.base_url}/v1/chat/completions',
            json=data,
            headers={'Content-Type': 'application/json'},
            timeout=(10, self.timeout),
        )
        if resp.status_code != 200:
            raise RuntimeError(f'LLM error {resp.status_code}: {resp.text[:200]}')
        data = resp.json()
        return data['choices'][0]['message']['content']


class WakeWordNode(Node):
    def __init__(self):
        super().__init__('wake_word_node')

        # ---- 参数 ----
        self.declare_parameter('wake_word', '小飞小飞')
        self.declare_parameter('sensitivity', 0.25)
        self.declare_parameter('kws_model_dir', '/home/iflytek/models/zipformer_kws')
        self.declare_parameter('asr_model_dir', '/home/iflytek/models/zipformer_onnx')
        self.declare_parameter('keywords_file', '')
        self.declare_parameter('llm.server_url', 'http://127.0.0.1:8080')
        self.declare_parameter('llm.intent_hints', [])
        self.declare_parameter('llm_server.model',
                               '/home/iflytek/models/rkllm/Qwen3.5-0.8B_w4a16_g128_rk3576.rkllm')
        self.declare_parameter('llm_server.platform', 'rk3576')
        self.declare_parameter('llm_server.lib_path', '/usr/lib/librkllmrt.so')
        self.declare_parameter('llm_server.venv_python', '/home/iflytek/ros2_venv/bin/python3')
        self.declare_parameter('start_service', '/mission_orchestrator/start')
        self.declare_parameter('mission_state_topic', '/car1_brics_task/state')
        self.declare_parameter('audio_device_index', -1)
        self.declare_parameter('capture_volume', 88)
        self.declare_parameter('voice_preset', True)
        self.declare_parameter('prompt_wav', '')
        # GTCRN 神经网络降噪（/home/iflytek/models/ns/gtcrn_simple.onnx）
        # 在 KWS/ASR 前对流式音频做语音增强，去除稳态背景噪声（风扇/电机/空调）。
        # 注意：实测降噪后唤醒略灵敏但 ASR 更难识别，故默认关闭。
        self.declare_parameter('onnx_denoise', False)          # 总开关
        self.declare_parameter('onnx_denoise_model',
                               '/home/iflytek/models/ns/gtcrn_simple.onnx')
        self.declare_parameter('onnx_denoise_threads', 2)

        self.wake_word = self.get_parameter('wake_word').value
        self.sensitivity = self.get_parameter('sensitivity').value
        self.kws_model_dir = self.get_parameter('kws_model_dir').value
        self.asr_model_dir = self.get_parameter('asr_model_dir').value
        self.keywords_file = self.get_parameter('keywords_file').value
        self.llm_server_url = self.get_parameter('llm.server_url').value
        self.intent_hints = self.get_parameter('llm.intent_hints').value or []
        self.start_service = self.get_parameter('start_service').value
        self.mission_state_topic = self.get_parameter('mission_state_topic').value
        self.capture_volume = self.get_parameter('capture_volume').value
        self.voice_preset = self.get_parameter('voice_preset').value
        self.prompt_wav = self.get_parameter('prompt_wav').value
        self.onnx_denoise = self.get_parameter('onnx_denoise').value
        self.onnx_denoise_model = self.get_parameter('onnx_denoise_model').value
        self.onnx_denoise_threads = int(self.get_parameter('onnx_denoise_threads').value)

        # GTCRN 降噪器（可选，默认关闭；失败不阻塞）
        self._denoiser = self._init_denoiser()
        if self._denoiser is not None:
            self.get_logger().info(
                f'GTCRN 降噪启用: {self.onnx_denoise_model}')

        # ---- 发布 / 订阅 ----
        self.detected_pub = self.create_publisher(Bool, '/interaction/wake_word/detected', 10)
        self.text_pub = self.create_publisher(String, '/interaction/wake_word/text', 10)
        self.command_pub = self.create_publisher(String, '/interaction/asr/text', 10)
        self.prompt_pub = self.create_publisher(
            String, '/interaction/tts/play_wav', 10)

        from std_srvs.srv import Trigger
        self.start_client = self.create_client(Trigger, self.start_service)

        from car1_brics_msgs.msg import MissionState
        self.create_subscription(
            MissionState, self.mission_state_topic, self._state_cb, 10)

        # ---- 音频设备 ----
        self._alsa_device = self._detect_mic_device()
        self._setup_mic()

        # ---- 引擎 ----
        self._init_kws()
        self._init_asr()
        self._start_llm_server()
        self._wait_for_llm_server()
        self.llm = LLMClient(self.llm_server_url)

        # ---- 状态 ----
        self._running = True
        self._wake_enabled = True      # 任务运行中置 False（暂停唤醒）
        self._kws_stream = self.kws.create_stream()
        self._state = 'IDLE'           # IDLE / LISTENING
        self._llm_proc = None
        self._asr_last_partial = ''    # ASR 上次部分结果（用于去重 log）
        self._audio_queue = Queue(maxsize=100)  # 采集线程 -> 主循环

        # ---- 采集线程（独立录音，避免阻塞状态机）----
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'wake_word_node 就绪。唤醒="{self.wake_word}" 设备={self._alsa_device} '
            f'LLM={self.llm_server_url} 触发 service={self.start_service}')

    # ------------------------------------------------------------------
    # 音频设备
    # ------------------------------------------------------------------

    def _detect_mic_device(self):
        try:
            out = subprocess.run(['arecord', '-l'], capture_output=True, text=True,
                                 check=False).stdout
            for line in out.split('\n'):
                if 'ES8388' in line.upper() or 'RK' in line.upper():
                    m = re.search(r'card\s+(\d+):.*device\s+(\d+)', line)
                    if m:
                        return f'plughw:{m.group(1)},{m.group(2)}'
        except Exception:
            pass
        return 'plughw:0,0'

    def _setup_mic(self):
        try:
            from audio_utils import (
                switch_input_to_mic, apply_capture_volume, enable_alc, setup_speaker)
            card = int(self._alsa_device.split(':')[1].split(',')[0])
            enable_alc(card)
            setup_speaker(card)
            switch_input_to_mic(card)
            apply_capture_volume(card, 88 if self.voice_preset else self.capture_volume)
            # 显式开启 ALC（实测 ALC=On + 降噪=Off 唤醒效果最佳）
            # + 设置增益 + 切麦克风输入
        except Exception as e:
            self.get_logger().warn(f'麦克风配置失败（继续用默认）: {e}')

    # ------------------------------------------------------------------
    # 引擎初始化
    # ------------------------------------------------------------------

    def _init_kws(self):
        kw_file = self.keywords_file
        if not kw_file or not os.path.isfile(kw_file):
            base = _AUDIO_UTILS / 'keywords.txt'
            kw_file = str(base) if base.exists() else self._gen_keywords_file()
        self.kws = sherpa_onnx.KeywordSpotter(
            tokens=os.path.join(self.kws_model_dir, 'tokens.txt'),
            encoder=os.path.join(self.kws_model_dir,
                                 'encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx'),
            decoder=os.path.join(self.kws_model_dir,
                                 'decoder-epoch-13-avg-2-chunk-16-left-64.onnx'),
            joiner=os.path.join(self.kws_model_dir,
                                'joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx'),
            keywords_file=kw_file,
            num_threads=1, sample_rate=SAMPLE_RATE, max_active_paths=4,
            keywords_score=1.0, keywords_threshold=self.sensitivity,
            num_trailing_blanks=2, provider='cpu',
        )
        log_kws.info('KWS 引擎就绪')

    def _gen_keywords_file(self):
        import tempfile
        fd, path = tempfile.mkstemp(suffix='.txt')
        with os.fdopen(fd, 'w') as f:
            f.write('x iǎo f ēi x iǎo f ēi @小飞小飞\n')
        return path

    def _init_asr(self):
        model_dir = self.asr_model_dir
        self.recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=os.path.join(model_dir, 'tokens.txt'),
            encoder=os.path.join(model_dir, 'encoder-epoch-99-avg-1.onnx'),
            decoder=os.path.join(model_dir, 'decoder-epoch-99-avg-1.onnx'),
            joiner=os.path.join(model_dir, 'joiner-epoch-99-avg-1.int8.onnx'),
            provider='cpu', num_threads=1, sample_rate=SAMPLE_RATE, feature_dim=80,
            decoding_method='greedy_search', blank_penalty=0.25,
            enable_endpoint_detection=True,
            rule1_min_trailing_silence=4.5, rule2_min_trailing_silence=1.5,
            rule3_min_utterance_length=300.0,
        )
        self.asr_stream = None
        log_asr.info('ASR 引擎就绪（greedy_search，无热词）')

    def _init_denoiser(self):
        """按需创建 GTCRN 降噪器；失败不阻塞（返回 None）。"""
        if not self.onnx_denoise:
            return None
        try:
            import numpy as np
            ns_cfg = sherpa_onnx.OnlineSpeechDenoiserConfig()
            ns_cfg.model.gtcrn.model = self.onnx_denoise_model
            ns_cfg.model.num_threads = self.onnx_denoise_threads
            ns_cfg.model.provider = 'cpu'
            denoiser = sherpa_onnx.OnlineSpeechDenoiser(ns_cfg)
            self._denoise_shift = denoiser.frame_shift_in_samples
            return denoiser
        except Exception as e:
            self.get_logger().warn(f'GTCRN 降噪初始化失败，跳过: {e}')
            return None

    def _denoise_chunk(self, samples):
        """把一个 int16 sample 块做 GTCRN 流式降噪，返回 int16 块（失败原样返回）。"""
        if self._denoiser is None:
            return samples
        try:
            x = np.asarray(samples, dtype=np.float32) / 32768.0
            out = []
            for i in range(0, len(x) - self._denoise_shift, self._denoise_shift):
                chunk = x[i:i + self._denoise_shift].tolist()
                res = self._denoiser.run(chunk, SAMPLE_RATE)
                clean = res.samples if hasattr(res, 'samples') else res
                out.extend(clean)
            if not out:
                return samples
            import numpy as _np
            y = (np.asarray(out, dtype=np.float32) * 32767.0).clip(-32768, 32767)
            return y.astype(np.int16).tolist()
        except Exception as e:
            self.get_logger().debug(f'降噪块处理异常: {e}')
            return samples

    def _start_llm_server(self):
        flask_script = os.path.join(LLM_SERVER_DIR, 'flask_server.py')
        if not os.path.exists(flask_script):
            self.get_logger().error(f'flask_server.py 不存在: {flask_script}')
            return
        port = int(self.llm_server_url.split(':')[-1].rstrip('/'))
        model = self.get_parameter('llm_server.model').value
        platform = self.get_parameter('llm_server.platform').value
        venv_py = self.get_parameter('llm_server.venv_python').value
        if not os.path.exists(venv_py):
            venv_py = sys.executable

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        port_in_use = (sock.connect_ex(('127.0.0.1', port)) == 0)
        sock.close()
        if port_in_use:
            self.get_logger().info(f'端口 {port} 已占用，跳过 LLM 启动')
            return

        env = os.environ.copy()
        env['PORT'] = str(port)
        cmd = [venv_py, flask_script, '--rkllm_model_path', model,
               '--target_platform', platform]
        err_log = open('/tmp/wake_llm_server.log', 'w')
        self._llm_proc = subprocess.Popen(cmd, cwd=LLM_SERVER_DIR, env=env,
                                          stdout=err_log, stderr=subprocess.STDOUT)
        self.get_logger().info(f'启动 LLM 服务（端口 {port}）')

    def _wait_for_llm_server(self, max_wait=120):
        import requests
        deadline = time.time() + max_wait
        while time.time() < deadline:
            if self._llm_proc and self._llm_proc.poll() is not None:
                self.get_logger().error('LLM 服务进程退出')
                return False
            try:
                r = requests.get(f'{self.llm_server_url}/v1/models', timeout=3)
                if r.status_code == 200:
                    self.get_logger().info('LLM 服务就绪')
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        self.get_logger().error('LLM 启动超时')
        return False

    # ------------------------------------------------------------------
    # 任务状态订阅（暂停/恢复唤醒）
    # ------------------------------------------------------------------

    def _state_cb(self, msg):
        running = msg.running
        if running and self._wake_enabled:
            self._wake_enabled = False
            self.get_logger().info('任务运行中，暂停唤醒（防止误触发）')
        elif not running and not self._wake_enabled:
            self._wake_enabled = True
            self.get_logger().info('任务结束，恢复唤醒')

    # ------------------------------------------------------------------
    # 采集线程（独立录音，对齐 eai_factory：只灌队列，不处理状态机）
    # ------------------------------------------------------------------

    def _capture_loop(self):
        """独立录音线程：持续从 ALSA 读 chunk 放入队列，绝不在读上阻塞主循环。

        sherpa_onnx.Alsa 以内部实际采样率采（KWS 用 actual_sample_rate 能唤醒，
        证明 ALSA 已重采样到 16k），本分支统一按 SAMPLE_RATE 喂 KWS/ASR。
        """
        self.get_logger().info(f'录音线程: {self._alsa_device}')
        try:
            alsa = sherpa_onnx.Alsa(self._alsa_device)
            actual_rate = alsa.actual_sample_rate
            if actual_rate > 0 and actual_rate != SAMPLE_RATE:
                self.get_logger().warn(
                    f'ALSA actual_sample_rate={actual_rate} != {SAMPLE_RATE}，'
                    f'使用 actual_sample_rate 修正（对齐 kws_test.py）')
            effective_rate = actual_rate if actual_rate > 0 else SAMPLE_RATE
            self.get_logger().info(
                f'采样率: expected={SAMPLE_RATE} actual={actual_rate} '
                f'effective={effective_rate}')
            while self._running and rclpy.ok():
                samples = alsa.read(4000)   # ~250ms，对齐 eai_factory chunk_size
                if not samples:
                    continue
                # 只塞原始音频：降噪仅对 KWS 分支生效（_handle_idle 内），
                # ASR 走原始（实测降噪让 ASR 更难识别）。保持 int16。
                if not samples:
                    continue
                try:
                    self._audio_queue.put((samples, effective_rate), timeout=0.5)
                except queue.Full:
                    try:
                        self._audio_queue.get_nowait()
                    except Empty:
                        pass
                    try:
                        self._audio_queue.put_nowait((samples, effective_rate))
                    except Exception:
                        pass
        except Exception as e:
            self.get_logger().error(f'采集循环异常: {e}')

    def run(self):
        """主循环：spin_once 挂 ROS 回调 + 从队列取 chunk 跑状态机。

        状态机：IDLE（KWS 监听）-> LISTENING（ASR）。
        状态处理与录音彻底解耦（eai_factory 架构），避免阻塞漏音。
        """
        def on_signal(signum, frame):
            self._running = False
        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

        while self._running and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                samples, rate = self._audio_queue.get(timeout=0.1)
            except Empty:
                continue
            if not self._wake_enabled:
                continue  # 任务运行中，暂停唤醒
            if self._state == 'IDLE':
                self._handle_idle(samples, rate)
            elif self._state == 'LISTENING':
                self._handle_listening(samples, rate)

    def destroy_node(self):
        self._running = False
        super().destroy_node()

    def _handle_idle(self, samples, rate):
        """KWS 监听唤醒词。降噪仅在此分支（唤醒更灵敏）；ASR 分支不用降噪。"""
        if self._denoiser is not None:
            samples = self._denoise_chunk(samples)
            if not samples:
                return
        self._kws_stream.accept_waveform(rate, samples)
        while self.kws.is_ready(self._kws_stream):
            self.kws.decode_stream(self._kws_stream)
        result = self.kws.get_result(self._kws_stream)
        if result:
            kw = result.split('@')[-1].strip() if '@' in result else result.strip()
            self.kws.reset_stream(self._kws_stream)
            self.get_logger().info(f'[KWS] 唤醒: {kw}')
            self._on_wakeup()

    def _on_wakeup(self):
        """唤醒命中：发布 detected 事件 + 播提示音 + 清回声 + 进 ASR 监听。"""
        self.detected_pub.publish(Bool(data=True))
        self.text_pub.publish(String(data=self.wake_word))

        if self.prompt_wav and os.path.isfile(self.prompt_wav):
            self.prompt_pub.publish(String(data=self.prompt_wav))
            time.sleep(0.8)

        # 清回声：提示音/扬声器尾音若灌进 ASR，会把命令前段当噪声吞掉
        self._drain_audio_queue()
        self.asr_stream = self.recognizer.create_stream()
        self._asr_last_partial = ''
        self._state = 'LISTENING'
        self.get_logger().info('IDLE -> LISTENING（听命令）')

    def _drain_audio_queue(self):
        while True:
            try:
                self._audio_queue.get_nowait()
            except Empty:
                break

    def _handle_listening(self, samples, rate):
        """ASR 识别命令。"""
        if self.asr_stream is None:
            self._state = 'IDLE'
            return
        self.asr_stream.accept_waveform(rate, samples)
        while self.recognizer.is_ready(self.asr_stream):
            self.recognizer.decode_stream(self.asr_stream)
        result = self.recognizer.get_result(self.asr_stream)

        if result and result != self._asr_last_partial:
            log_asr.info(f'[ASR] {result}')
            self._asr_last_partial = result

        if self.recognizer.is_endpoint(self.asr_stream):
            final = result.strip() if result else ''
            self.recognizer.reset(self.asr_stream)
            self.asr_stream = None
            self._asr_last_partial = ''
            self._state = 'IDLE'
            if final and _is_valid_asr_result(final):
                log_asr.info(f'识别: {final}')
                self.command_pub.publish(String(data=final))
                self._process_command(final)
            else:
                log_asr.info(f'结果无效丢弃: "{final}" → IDLE')

    # ------------------------------------------------------------------
    # LLM 意图分类 + 启动
    # ------------------------------------------------------------------

    def _build_prompt(self, user_text):
        lines = []
        for entry in self.intent_hints:
            if isinstance(entry, dict) and 'intent' in entry and 'hints' in entry:
                words = '、'.join(entry['hints'])
                lines.append(f"- {entry['intent']}：{words}")
        lines.append('- none：与上述无关')
        return (
            '请判断下面这句话属于哪种意图，只回答一个英文单词（start/none），不要解释：\n'
            + '\n'.join(lines) + '\n'
            f'用户指令：{user_text}\n答案：'
        )

    def _process_command(self, user_text):
        """LLM 意图分类 → 命中 start 调启动 service。"""
        self.get_logger().info(f'ASR: {user_text}')
        prompt = self._build_prompt(user_text)
        try:
            answer = self.llm.chat(
                [{'role': 'user', 'content': prompt}],
                temperature=0.1, top_p=0.9, max_tokens=20, repeat_penalty=1.1,
            ).strip().lower()
            log_llm.info(f'LLM 分类: "{answer}"')
        except Exception as e:
            self.get_logger().error(f'LLM 意图分类异常: {e}')
            return

        start = 'start' in answer
        if not start:
            # 关键字兜底（LLM 小模型误判的保险）：命中出发/执行/任务/开始等
            start = self._keyword_fallback(user_text)
        if start:
            self.get_logger().info(f'命中启动意图（{user_text}）→ 调 {self.start_service}')
            # 命中即禁用唤醒：不等 mission_state 回调（DDS 有延迟，避免窗口期重复触发）
            self._wake_enabled = False
            self._call_start_service()
        else:
            self.get_logger().info(f'非启动意图: "{user_text}"')

    def _keyword_fallback(self, text):
        """ASR 原文关键字兜底：命中启动相关词即视为 start。"""
        if not text:
            return False
        for kw in ('出发', '执行', '任务', '开始', '启动', '干活', '走吧'):
            if kw in text:
                return True
        return False

    def _call_start_service(self):
        # wait_for_service 由 DDS discovery 事件驱动，短等即返回，优于裸 service_is_ready()
        # （service_is_ready 需已 discovery，唤醒→本次调用间隔短时可能仍为 False）。
        if not self.start_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f'启动 service 未就绪（等待 3s 仍无）: {self.start_service}')
            return
        from std_srvs.srv import Trigger
        req = Trigger.Request()
        self.start_client.call_async(req)
        self.get_logger().info(f'已请求 {self.start_service}')

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WakeWordNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
