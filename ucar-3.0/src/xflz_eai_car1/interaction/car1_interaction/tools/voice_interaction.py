#!/usr/bin/env python3
"""
Voice interaction: KWS → ASR → LLM(streaming) → TTS(streaming playback)

States: IDLE → LISTENING → SPEAKING → IDLE

Architecture:
  - Capture thread: ALSA → audio_queue (always running)
  - Main loop: state machine (KWS in IDLE/SPEAKING, ASR in LISTENING)
  - LLM+TTS worker thread: spawned per utterance, streams LLM → sentence
    segmentation → TTS synthesis → PCM write to persistent playback pipe

Key design decisions (vs previous version):
  - No VAD gating — KWS runs on all audio (per kws_test.py)
  - ASR uses built-in endpoint detection (per asr_test.py)
  - Playback handle stays open (aplay stdin pipe, no file/process per sentence)
  - Prompt sound pre-loaded as PCM for instant playback
  - No ACTIVE_LISTENING / free-talk window — always requires re-wake

Usage:
  python3 voice_interaction.py
  python3 voice_interaction.py --no-server     # connect to external LLM server
  python3 voice_interaction.py --debug
"""

import argparse
import json
import logging
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import wave
from queue import Queue, Empty

import numpy as np
import sherpa_onnx
import yaml

from audio_utils import setup_capture, setup_playback

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)-5s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("Main")
log_kws = logging.getLogger("KWS")
log_asr = logging.getLogger("ASR")
log_llm = logging.getLogger("LLM")
log_tts = logging.getLogger("TTS")


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# ---------------------------------------------------------------------------
# Persistent ALSA playback (aplay stdin pipe — no file/process per sentence)
# ---------------------------------------------------------------------------

class Playback:
    """Write raw PCM to a persistent aplay process. Supports interrupt.

    Auto-cleans conflicting processes on init if playback device is busy.
    """

    def __init__(self, device="plughw:0,0", sample_rate=16000, channels=2):
        self.device = device
        self.sample_rate = sample_rate
        self.channels = channels
        self._proc = None
        self._lock = threading.Lock()
        self._failed = False

        # Try to open playback device
        self._open()

        # If failed, likely due to device busy — kill conflicting processes
        if self._failed or (self._proc and self._proc.poll() is not None):
            log.warning("播放设备被占用，清理冲突进程...")
            self._kill_conflicting_playback()
            time.sleep(0.5)  # Wait for device release
            self._open()  # Retry

            if self._failed or (self._proc and self._proc.poll() is not None):
                log.error(f"无法打开播放设备 {device}，播放将被禁用")

    def _kill_conflicting_playback(self):
        """Kill processes using the playback device."""
        try:
            # Kill all aplay processes (safest — only kills our own tool)
            subprocess.run("pkill -9 aplay", shell=True,
                          capture_output=True, timeout=2)
            log.debug("已清理 aplay 进程")
        except Exception as e:
            log.debug(f"清理进程失败: {e}")

    def _open(self):
        try:
            self._proc = subprocess.Popen(
                ["aplay", "-t", "raw", "-f", "S16_LE",
                 "-c", str(self.channels), "-r", str(self.sample_rate),
                 "-D", self.device, "-q", "-"],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )
            # Wait briefly to see if aplay exits immediately (device busy)
            time.sleep(0.1)
            if self._proc.poll() is not None:
                # Process exited — read stderr to diagnose
                _, stderr = self._proc.communicate(timeout=0.5)
                err_msg = stderr.decode('utf-8', errors='ignore') if stderr else ""
                log.error(f"aplay 启动失败: {err_msg[:200]}")
                self._failed = True
            else:
                self._failed = False
        except Exception as e:
            log.error(f"无法启动 aplay: {e}")
            self._failed = True

    def write(self, pcm_bytes: bytes):
        """Write S16_LE stereo PCM data to playback."""
        if self._failed:
            return  # Silently skip if playback is broken

        with self._lock:
            try:
                self._proc.stdin.write(pcm_bytes)
                self._proc.stdin.flush()
            except (BrokenPipeError, OSError) as e:
                log.warning(f"播放失败 ({e})，尝试重启 aplay")
                self._reopen()
                if not self._failed:
                    try:
                        self._proc.stdin.write(pcm_bytes)
                        self._proc.stdin.flush()
                    except Exception as e2:
                        log.error(f"重启后仍然失败: {e2}")
                        self._failed = True

    def interrupt(self):
        """Kill current playback and reopen for next use."""
        with self._lock:
            if self._proc and not self._failed:
                try:
                    self._proc.kill()
                    self._proc.wait()
                except Exception:
                    pass
            self._open()

    def _reopen(self):
        if self._proc:
            try:
                self._proc.kill()
                self._proc.wait()
            except Exception:
                pass
        self._open()

    def close(self):
        with self._lock:
            if self._proc and not self._failed:
                try:
                    self._proc.stdin.close()
                    self._proc.wait(timeout=2)
                except Exception:
                    self._proc.kill()


# ---------------------------------------------------------------------------
# LLM streaming client (OpenAI-compatible)
# ---------------------------------------------------------------------------

class LLMClient:
    def __init__(self, base_url, timeout=120):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        import requests
        self.session = requests.Session()
        self.session.keep_alive = False
        adapter = requests.adapters.HTTPAdapter(max_retries=3)
        self.session.mount("http://", adapter)

    def chat_stream(self, messages, temperature=0.8, top_p=0.9,
                    max_tokens=4096, repeat_penalty=1.1,
                    interrupt_event=None, stall_timeout=30.0):
        """Yield content tokens. Stops on interrupt or if no token for stall_timeout seconds.

        stall_timeout guards against server hangs mid-stream — if no new token
        arrives within this window, we abandon the request rather than blocking
        the state machine forever.
        """
        data = {
            "model": "rkllm",
            "messages": messages,
            "temperature": temperature,
            "top_p": top_p,
            "max_tokens": max_tokens,
            "repeat_penalty": repeat_penalty,
            "stream": True,
        }
        headers = {"Content-Type": "application/json",
                   "Authorization": "not_required"}
        resp = self.session.post(
            f"{self.base_url}/v1/chat/completions",
            json=data, headers=headers, stream=True,
            timeout=(10, self.timeout),
        )
        if resp.status_code != 200:
            raise RuntimeError(
                f"LLM error {resp.status_code}: {resp.text[:200]}")
        last_token_time = time.time()
        try:
            for line in resp.iter_lines(decode_unicode=True):
                if interrupt_event and interrupt_event.is_set():
                    break
                # Stall detection (server crashed mid-stream)
                if time.time() - last_token_time > stall_timeout:
                    raise TimeoutError(
                        f"LLM 流式响应停滞 >{stall_timeout}s，放弃本次请求")
                if not line:
                    continue
                if not line.startswith("data: "):
                    continue
                payload = line[6:].strip()
                if payload == "[DONE]":
                    break
                try:
                    chunk = json.loads(payload)
                    content = chunk["choices"][0].get(
                        "delta", {}).get("content", "")
                    if content:
                        last_token_time = time.time()
                        yield content
                except (json.JSONDecodeError, KeyError, IndexError):
                    continue
        finally:
            resp.close()


# ---------------------------------------------------------------------------
# VoiceInteraction — simplified state machine
# ---------------------------------------------------------------------------

class VoiceInteraction:
    """KWS → ASR → LLM(streaming) → TTS(streaming) pipeline."""

    # Sentence segmentation for TTS streaming — split on major pauses.
    # Fine-grained splits (including comma) reduce latency: each segment
    # is synthesized and played immediately rather than buffering the full answer.
    SENTENCE_RE = re.compile(
        r'([。！？；，、\n]|[.?!,](?=\s|$))'
    )

    # Symbols to strip before TTS synthesis.
    # - Markdown / code syntax: * # ` _ ~ < > [ ] { } | \ / @ ^ & $ %
    # - Quotes (mispronounced by TTS): " " ' ' 「 」 『 』
    # - Brackets (redundant for speech): ( ) （ ） [ ] 【 】 < > 《 》
    # - Misc: — (em dash), … (ellipsis), · (middle dot)
    # - Emoji, URLs
    #
    # KEEP punctuation that TTS renders as pauses: 。，！？：；、
    _TTS_STRIP_RE = re.compile(
        r'[\*#`_~<>\[\]\{\}\|\\/@\^&\$%]+'                       # markdown
        r'|["""\'\'「」『』]'                                      # quotes
        r'|[()（）\[\]【】<>《》]'                                 # brackets
        r'|[—…·]'                                                  # misc punct
        r'|[\U0001F300-\U0001FAFF\U00002600-\U000027BF]'         # emoji
        r'|https?://\S+'                                          # URLs
    )

    def __init__(self, config_path, auto_start_llm=True):
        self.cfg = load_config(config_path)
        self.script_dir = os.path.dirname(os.path.abspath(config_path))

        cfg_audio = self.cfg["audio"]
        self.sample_rate = cfg_audio["sample_rate"]
        self.chunk_size = cfg_audio["chunk_size"]
        self.capture_device = cfg_audio["capture_device"]
        self.playback_device = cfg_audio["playback_device"]

        # Thread control
        self.running = True
        self.state = "IDLE"
        self.interrupt_event = threading.Event()
        self.speaking_done = threading.Event()
        self.speaking_done.set()
        self.audio_queue = Queue(maxsize=100)

        # Mixer setup
        card = 0
        setup_capture(card, cfg_audio["capture_volume"],
                      cfg_audio.get("mic_boost", True))
        setup_playback(card, cfg_audio["playback_volume"])

        # Persistent playback
        self.playback = Playback(self.playback_device, self.sample_rate)

        # Pre-load prompt sound as raw PCM
        resource_dir = self.cfg["interaction"]["resource_dir"]
        if not os.path.isabs(resource_dir):
            resource_dir = os.path.join(self.script_dir, resource_dir)
        self._prompt_pcm = self._load_wav_as_pcm(
            os.path.join(resource_dir, "kws.wav"))

        # Init models
        self._init_kws()
        self._init_asr()
        self._init_tts()

        # LLM server
        self._llm_proc = None
        self._shutdown_done = False  # Guard against multiple shutdown calls
        if auto_start_llm and self.cfg.get(
                "llm_server", {}).get("auto_start", True):
            self._start_llm_server()
            self._wait_for_llm_server()
        self._init_llm()

        log.info("初始化完成")

    # ------------------------------------------------------------------
    # WAV pre-loading
    # ------------------------------------------------------------------

    def _load_wav_as_pcm(self, path):
        """Load wav as stereo S16_LE PCM for direct pipe write."""
        if not os.path.exists(path):
            log.warning(f"提示音不存在: {path}")
            return b""
        with wave.open(path, "rb") as wf:
            raw = wf.readframes(wf.getnframes())
            ch = wf.getnchannels()
            sw = wf.getsampwidth()
            rate = wf.getframerate()

        dtype_map = {1: np.int8, 2: np.int16, 4: np.int32}
        samples = np.frombuffer(raw, dtype=dtype_map[sw])

        # Convert to int16 if needed
        if sw != 2:
            samples = (samples.astype(np.float32)
                       / np.iinfo(dtype_map[sw]).max * 32767
                       ).astype(np.int16)

        # Resample (nearest) if rate differs
        if rate != self.sample_rate:
            n_frames = len(samples) // ch
            n_out = int(n_frames * self.sample_rate / rate)
            idx = (np.arange(n_out) * rate / self.sample_rate).astype(int)
            idx = np.clip(idx, 0, n_frames - 1)
            if ch == 1:
                samples = samples[idx]
            else:
                samples = samples.reshape(-1, ch)[idx].ravel()

        # Mono → stereo
        if ch == 1:
            samples = np.column_stack(
                [samples, samples]).ravel().astype(np.int16)

        return samples.tobytes()

    # ------------------------------------------------------------------
    # Model init
    # ------------------------------------------------------------------

    def _init_kws(self):
        cfg = self.cfg["kws"]
        model_dir = cfg["model_dir"]
        kw_file = cfg["keywords_file"]
        if not os.path.isabs(kw_file):
            kw_file = os.path.join(self.script_dir, kw_file)

        self.kws = sherpa_onnx.KeywordSpotter(
            tokens=os.path.join(model_dir, "tokens.txt"),
            encoder=os.path.join(
                model_dir,
                "encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx"),
            decoder=os.path.join(
                model_dir,
                "decoder-epoch-13-avg-2-chunk-16-left-64.onnx"),
            joiner=os.path.join(
                model_dir,
                "joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx"),
            keywords_file=kw_file,
            num_threads=4,
            sample_rate=self.sample_rate,
            max_active_paths=4,
            keywords_score=cfg["keywords_score"],
            keywords_threshold=cfg["threshold"],
            num_trailing_blanks=2,
            provider=cfg["provider"],
        )
        self.kws_stream = self.kws.create_stream()

        with open(kw_file, "r", encoding="utf-8") as f:
            for line in f:
                if "@" in line:
                    log_kws.info(f"唤醒词: {line.strip().split('@')[-1]}")

    def _init_asr(self):
        cfg = self.cfg["asr"]
        model_dir = cfg["model_dir"]
        npu_threads = 1 if cfg["provider"] == "rknn" else 4

        # ASR 用 ONNX 模型（zipformer_onnx，provider=cpu）；原 rknn 效果差已弃用
        self.recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=os.path.join(model_dir, "tokens.txt"),
            encoder=os.path.join(model_dir, "encoder-epoch-99-avg-1.int8.onnx"),
            decoder=os.path.join(model_dir, "decoder-epoch-99-avg-1.onnx"),
            joiner=os.path.join(model_dir, "joiner-epoch-99-avg-1.int8.onnx"),
            provider=cfg["provider"],
            num_threads=npu_threads,
            sample_rate=self.sample_rate,
            feature_dim=80,
            decoding_method="greedy_search",
            blank_penalty=cfg.get("blank_penalty", 0.25),
            enable_endpoint_detection=True,
            rule1_min_trailing_silence=cfg.get("rule1_silence", 4.5),
            rule2_min_trailing_silence=cfg.get("rule2_silence", 1.5),
            rule3_min_utterance_length=cfg.get("rule3_min_utt", 300.0),
        )
        self.asr_stream = None
        self._asr_last_partial = ""
        log_asr.info(
            f"provider={cfg['provider']} "
            f"endpoint: r1={cfg.get('rule1_silence', 4.5)}s "
            f"r2={cfg.get('rule2_silence', 1.5)}s")

    def _init_tts(self):
        cfg = self.cfg["tts"]
        model_dir = cfg["model_dir"]

        tts_config = sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(
                matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                    acoustic_model=os.path.join(
                        model_dir, "model-steps-3.onnx"),
                    vocoder=os.path.join(
                        model_dir, "vocos-16khz-univ.onnx"),
                    lexicon=os.path.join(model_dir, "lexicon.txt"),
                    tokens=os.path.join(model_dir, "tokens.txt"),
                    data_dir=os.path.join(model_dir, "espeak-ng-data"),
                ),
                num_threads=cfg["num_threads"],
            ),
            max_num_sentences=1,
            rule_fsts=",".join([
                os.path.join(model_dir, "phone-zh.fst"),
                os.path.join(model_dir, "date-zh.fst"),
                os.path.join(model_dir, "number-zh.fst"),
            ]),
        )
        if not tts_config.validate():
            raise RuntimeError("TTS 配置验证失败")
        self.tts = sherpa_onnx.OfflineTts(tts_config)
        self.tts_speed = cfg["speed"]
        self.tts_gain = cfg.get("gain", 2.0)
        self.tts_sid = cfg["sid"]
        log_tts.info(f"speed={cfg['speed']} gain={self.tts_gain}")

    def _init_llm(self):
        cfg = self.cfg["llm"]
        self.llm = LLMClient(cfg["server_url"], timeout=cfg["timeout"])
        self.llm_cfg = cfg
        log_llm.info(f"server={cfg['server_url']}")

    # ------------------------------------------------------------------
    # LLM server management
    # ------------------------------------------------------------------

    def _start_llm_server(self):
        cfg = self.cfg["llm_server"]
        server_dir = os.path.abspath(
            os.path.join(self.script_dir, "..", "llm_server"))
        flask_script = os.path.join(server_dir, "flask_server.py")
        if not os.path.exists(flask_script):
            log_llm.error(f"flask_server.py 不存在: {flask_script}")
            return

        # Extract port from server_url
        server_url = self.cfg["llm"]["server_url"]
        port = int(server_url.split(":")[-1].rstrip("/"))

        # Check if port is already in use
        import socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        port_in_use = (sock.connect_ex(("127.0.0.1", port)) == 0)
        sock.close()

        if port_in_use:
            log_llm.warning(
                f"端口 {port} 已被占用，检查是否为有效 LLM 服务...")
            # Verify it's actually a working LLM server (not some other process)
            import requests
            try:
                r = requests.get(
                    f"{server_url}/v1/models", timeout=2)
                if r.status_code == 200:
                    log_llm.info("外部 LLM 服务可用，跳过启动")
                    return
            except Exception as e:
                log_llm.warning(f"端口被占用但服务不可用: {e}")

            # Port conflict: auto-kill if configured
            if cfg.get("auto_kill_conflict", True):
                log_llm.info(f"自动清理端口 {port} 上的冲突进程...")
                try:
                    result = subprocess.run(
                        f"lsof -ti :{port} | xargs kill -9",
                        shell=True, capture_output=True, text=True, timeout=5)
                    if result.returncode == 0:
                        log_llm.info("清理完成，继续启动")
                        time.sleep(0.5)  # Wait for port release
                    else:
                        log_llm.warning(
                            f"清理失败，请手动运行: lsof -ti :{port} | xargs kill -9")
                        return
                except Exception as e:
                    log_llm.error(f"清理端口失败: {e}")
                    return
            else:
                log_llm.warning(
                    f"请手动清理: lsof -ti :{port} | xargs kill -9")
                return

        lib_dir = os.path.join(server_dir, "lib")
        os.makedirs(lib_dir, exist_ok=True)
        lib_dst = os.path.join(lib_dir, "librkllmrt.so")
        if not os.path.exists(lib_dst) and os.path.exists(cfg["lib_path"]):
            os.symlink(cfg["lib_path"], lib_dst)

        platform = cfg["platform"]
        fix_script = os.path.join(
            server_dir, f"fix_freq_{platform}.sh")
        if not os.path.exists(fix_script):
            with open(fix_script, "w") as f:
                f.write("#!/bin/bash\nexit 0\n")
            os.chmod(fix_script, 0o755)

        fake_dir = "/tmp/voice_interaction_sudo"
        os.makedirs(fake_dir, exist_ok=True)
        fake_sudo = os.path.join(fake_dir, "sudo")
        with open(fake_sudo, "w") as f:
            f.write("#!/bin/bash\nexit 0\n")
        os.chmod(fake_sudo, 0o755)

        env = os.environ.copy()
        env["PATH"] = f"{fake_dir}:{env.get('PATH', '')}"
        # Pass port via environment variable (flask_server.py doesn't accept --port)
        env["FLASK_RUN_PORT"] = str(port)
        env["PORT"] = str(port)  # Backup env var

        venv_py = cfg.get("venv_python", sys.executable)
        if not os.path.exists(venv_py):
            venv_py = sys.executable

        cmd = [venv_py, flask_script,
               "--rkllm_model_path", cfg["model"],
               "--target_platform", platform]  # No --port flag

        log_llm.info(
            f"启动 LLM: {os.path.basename(cfg['model'])} (端口 {port})")
        self._llm_proc = subprocess.Popen(
            cmd, cwd=server_dir, env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _wait_for_llm_server(self, max_wait=120):
        import requests
        url = self.cfg["llm"]["server_url"]
        deadline = time.time() + max_wait
        while time.time() < deadline:
            if self._llm_proc and self._llm_proc.poll() is not None:
                log_llm.error("LLM 服务进程退出")
                return False
            try:
                r = requests.get(f"{url}/v1/models", timeout=3)
                if r.status_code == 200:
                    log_llm.info("LLM 服务就绪")
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        log_llm.error("LLM 启动超时")
        return False

    # ------------------------------------------------------------------
    # TTS helper
    # ------------------------------------------------------------------

    @classmethod
    def _clean_tts_text(cls, text):
        """Strip symbols that TTS would mispronounce (markdown, emoji, URLs)."""
        cleaned = cls._TTS_STRIP_RE.sub("", text)
        # Collapse consecutive whitespace and trim
        cleaned = re.sub(r"\s+", " ", cleaned).strip()
        return cleaned

    def _tts_to_pcm(self, text):
        """Clean text → synthesize → stereo S16_LE PCM bytes.

        Uses tanh soft-clipping when gain > 1.0 (avoids harsh distortion,
        matches audio_test.py behavior).
        """
        clean = self._clean_tts_text(text)
        if not clean:
            return b""

        audio = self.tts.generate(
            clean, sid=self.tts_sid, speed=self.tts_speed)
        samples = np.array(audio.samples, dtype=np.float32)

        if self.tts_gain > 1.0:
            # Soft clip (tanh) to prevent harsh distortion at boost
            samples = np.tanh(samples * self.tts_gain)
        else:
            samples = (samples * self.tts_gain).clip(-1.0, 1.0)

        mono = (samples * 32767).astype(np.int16)
        stereo = np.column_stack([mono, mono]).ravel()
        return stereo.tobytes()

    # ------------------------------------------------------------------
    # ASR result filtering
    # ------------------------------------------------------------------

    # Known misrecognition artifacts (silence + noise → gibberish English).
    # These appear when the mic captures room noise or prompt echo tail.
    _ASR_BLACKLIST = {
        # Phone brands (common ASR hallucinations)
        "VIVO", "IVO", "VI", "V I", "V IVO", "VI VO",
        "OPPO", "OPPO", "OP PO",
        "HUAWEI", "XIAOMI", "APPLE",
        # Greetings/filler (often from noise)
        "HI", "HELLO", "HEY", "BYE",
        "OK", "YES", "NO", "UH", "UM", "AH", "EH", "OH",
        # Single letters
        "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K",
        "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V",
        "W", "X", "Y", "Z",
    }

    @classmethod
    def _is_valid_asr_result(cls, text):
        """Reject empty / too-short / known-garbage recognitions."""
        if not text:
            return False
        stripped = text.strip()
        if len(stripped) < 2:
            return False

        # Reject known misrecognitions (case-insensitive, whitespace-stripped)
        normalized = re.sub(r"\s+", "", stripped).upper()
        if normalized in cls._ASR_BLACKLIST:
            return False

        # Reject pure ASCII "words" (no space, no Chinese) that are short.
        # Real queries usually mix Chinese or are longer English phrases.
        is_pure_ascii = all(ord(c) < 128 for c in stripped)
        has_space = " " in stripped
        if is_pure_ascii and not has_space and len(stripped) < 6:
            # Single English word < 6 chars is likely noise (hello, ok, vivo)
            return False

        return True

    def _drain_audio_queue(self):
        """Discard all pending audio (used on SPEAKING→IDLE for echo tail)."""
        drained = 0
        while True:
            try:
                self.audio_queue.get_nowait()
                drained += 1
            except Empty:
                break
        if drained:
            log.debug(f"排空 {drained} 帧音频")

    # ------------------------------------------------------------------
    # Capture thread
    # ------------------------------------------------------------------

    def _capture_loop(self):
        log.info(f"录音线程: {self.capture_device}")
        alsa = sherpa_onnx.Alsa(self.capture_device)
        while self.running:
            samples = alsa.read(self.chunk_size)
            if not samples:
                continue
            try:
                self.audio_queue.put(samples, timeout=0.5)
            except queue.Full:
                try:
                    self.audio_queue.get_nowait()
                except Empty:
                    pass
                self.audio_queue.put_nowait(samples)

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _handle_idle(self, chunk):
        """KWS on all audio. Wake word → prompt → LISTENING."""
        self.kws_stream.accept_waveform(self.sample_rate, chunk)
        while self.kws.is_ready(self.kws_stream):
            self.kws.decode_stream(self.kws_stream)

        result = self.kws.get_result(self.kws_stream)
        if result:
            kw = result.split("@")[-1].strip() if "@" in result \
                else result.strip()
            self.kws.reset_stream(self.kws_stream)
            log_kws.info(f"唤醒: {kw}")

            self._play_prompt_then_start_asr()
            log.info("IDLE → LISTENING")

    def _play_prompt_then_start_asr(self):
        """Play prompt, wait for full playback + echo decay, drain queue twice.

        Critical timing to avoid prompt echo contamination:
        1. Write prompt PCM to aplay pipe (non-blocking at Python level)
        2. Sleep for prompt_duration + ALSA_output_latency + room_echo_tail
        3. Drain queue (discard chunks captured during playback)
        4. Sleep again briefly (let capture thread fill queue with any straggler echo)
        5. Drain queue a second time (final cleanup)
        6. Create ASR stream

        This ensures the first chunk fed to ASR is guaranteed post-echo.
        """
        if self._prompt_pcm:
            # Prompt is 0.67s at 16kHz stereo S16_LE
            prompt_dur = len(self._prompt_pcm) / (2 * 2 * self.sample_rate)
            log.debug(f"播放提示音 ({prompt_dur:.2f}s, {len(self._prompt_pcm)} bytes)")
            self.playback.write(self._prompt_pcm)

            # Wait for: prompt playback + ALSA output buffer + room reverb
            # Be conservative: 0.67s prompt + 0.5s ALSA latency + 0.3s echo tail
            wait_time = prompt_dur + 0.8
            time.sleep(wait_time)
        else:
            log.warning("提示音PCM为空，跳过播放")

        # First drain: discard chunks captured during prompt playback
        drained1 = 0
        while True:
            try:
                self.audio_queue.get_nowait()
                drained1 += 1
            except Empty:
                break

        # Critical: sleep briefly to let capture thread fill queue with any
        # trailing echo chunks that arrived during the first drain
        time.sleep(0.2)

        # Second drain: final cleanup
        drained2 = 0
        while True:
            try:
                self.audio_queue.get_nowait()
                drained2 += 1
            except Empty:
                break

        log.debug(f"排空音频队列: 第1轮 {drained1} 帧, 第2轮 {drained2} 帧")

        # Fresh ASR stream — guaranteed to start after prompt echo
        self.asr_stream = self.recognizer.create_stream()
        self._asr_last_partial = ""
        self.state = "LISTENING"

    def _handle_listening(self, chunk):
        """Feed ASR with built-in endpoint detection."""
        # ASR stream should have been created in _play_prompt_then_start_asr
        if self.asr_stream is None:
            log.warning("LISTENING 状态但 ASR stream 未创建，跳过")
            return

        self.asr_stream.accept_waveform(self.sample_rate, chunk)
        while self.recognizer.is_ready(self.asr_stream):
            self.recognizer.decode_stream(self.asr_stream)

        # Partial result
        result = self.recognizer.get_result(self.asr_stream)
        if result and result != self._asr_last_partial:
            pad = max(0, len(self._asr_last_partial) - len(result))
            print(f"\r[ASR] {result}{' ' * pad}", end="", flush=True)
            self._asr_last_partial = result

        # Endpoint
        if self.recognizer.is_endpoint(self.asr_stream):
            final = result.strip() if result else ""
            self.recognizer.reset(self.asr_stream)

            if self._asr_last_partial:
                print()
            self._asr_last_partial = ""

            if self._is_valid_asr_result(final):
                log_asr.info(f"识别: {final}")
                self._start_speaking(final)
            else:
                log_asr.info(f"结果无效丢弃: \"{final}\" → IDLE")
                self.asr_stream = None
                self.state = "IDLE"

    def _handle_speaking(self, chunk):
        """KWS for interrupt detection. Check if TTS done."""
        self.kws_stream.accept_waveform(self.sample_rate, chunk)
        while self.kws.is_ready(self.kws_stream):
            self.kws.decode_stream(self.kws_stream)

        result = self.kws.get_result(self.kws_stream)
        if result:
            kw = result.split("@")[-1].strip() if "@" in result \
                else result.strip()
            self.kws.reset_stream(self.kws_stream)
            log_kws.info(f"打断: {kw}")

            # Interrupt LLM+TTS
            self.interrupt_event.set()
            self.playback.interrupt()

            # Same as wake-up: play prompt, drain echo, start ASR
            self._play_prompt_then_start_asr()
            log.info("SPEAKING → LISTENING (打断)")
            return

        if self.speaking_done.is_set():
            # Drain any TTS echo captured in the mic queue before going idle,
            # otherwise the tail of the TTS audio could trigger a false wake.
            self._drain_audio_queue()
            # Reset KWS stream so any partial score from TTS echo is discarded
            self.kws.reset_stream(self.kws_stream)
            self.state = "IDLE"
            log.info("SPEAKING → IDLE")

    # ------------------------------------------------------------------
    # LLM + TTS worker (spawned per utterance)
    # ------------------------------------------------------------------

    def _start_speaking(self, user_text):
        """Launch LLM+TTS worker. Ensures clean state before starting."""
        # Ensure previous worker finished (should already be done, but guard)
        if not self.speaking_done.is_set():
            log.warning("上一次 LLM+TTS 未完成，强制中断")
            self.interrupt_event.set()
            # Give it a moment to clean up
            self.speaking_done.wait(timeout=1.0)
            if not self.speaking_done.is_set():
                log.error("上一次 worker 无法终止，可能卡死")

        # Clean state before starting new conversation
        self.interrupt_event.clear()
        self.speaking_done.clear()
        self.asr_stream = None

        self.state = "SPEAKING"
        log.info(f"LISTENING → SPEAKING ({user_text})")

        t = threading.Thread(
            target=self._llm_tts_worker, args=(user_text,),
            daemon=True, name="LLM-TTS")
        t.start()

    def _llm_tts_worker(self, user_text):
        """Stream LLM → segment → TTS → playback write.

        MUST always set speaking_done in finally block — otherwise the main
        loop will get stuck in SPEAKING state forever.
        """
        cfg = self.llm_cfg
        messages = [
            {"role": "system", "content": cfg["system_prompt"]},
            {"role": "user", "content": user_text},
        ]

        buffer = ""
        print("\n[LLM] ", end="", flush=True)

        try:
            for token in self.llm.chat_stream(
                messages,
                temperature=cfg["temperature"],
                top_p=cfg["top_p"],
                max_tokens=cfg["max_tokens"],
                repeat_penalty=cfg["repeat_penalty"],
                interrupt_event=self.interrupt_event,
            ):
                if self.interrupt_event.is_set():
                    break
                print(token, end="", flush=True)
                buffer += token

                # Split on sentence boundaries
                while True:
                    m = self.SENTENCE_RE.search(buffer)
                    if not m:
                        break
                    end = m.end()
                    sentence = buffer[:end].strip()
                    buffer = buffer[end:]
                    if sentence and len(sentence) > 1:
                        if self.interrupt_event.is_set():
                            break
                        pcm = self._tts_to_pcm(sentence)
                        if pcm and not self.interrupt_event.is_set():
                            self.playback.write(pcm)

            # Flush remaining text (only if not interrupted)
            if buffer.strip() and not self.interrupt_event.is_set():
                pcm = self._tts_to_pcm(buffer.strip())
                if pcm and not self.interrupt_event.is_set():
                    self.playback.write(pcm)
        except Exception as e:
            log_llm.error(f"LLM/TTS 异常: {e}")
        finally:
            print()
            # Guarantee: always signal done, even on exception/timeout
            self.speaking_done.set()

    # ------------------------------------------------------------------
    # Main loop & lifecycle
    # ------------------------------------------------------------------

    def run(self):
        cap = threading.Thread(
            target=self._capture_loop, daemon=True, name="Capture")
        cap.start()

        log.info("=" * 50)
        log.info("就绪 — 说唤醒词开始对话, Ctrl+C 退出")
        log.info("=" * 50)

        def on_signal(signum, frame):
            print()
            self.running = False
        signal.signal(signal.SIGINT, on_signal)
        signal.signal(signal.SIGTERM, on_signal)

        try:
            while self.running:
                try:
                    chunk = self.audio_queue.get(timeout=0.3)
                except Empty:
                    continue

                if self.state == "IDLE":
                    self._handle_idle(chunk)
                elif self.state == "LISTENING":
                    self._handle_listening(chunk)
                elif self.state == "SPEAKING":
                    self._handle_speaking(chunk)
        finally:
            self.shutdown()

    def shutdown(self):
        """Clean up resources. Safe to call multiple times."""
        if self._shutdown_done:
            return
        self._shutdown_done = True

        log.info("正在退出...")
        self.running = False

        # Close playback
        try:
            self.playback.close()
        except Exception as e:
            log.debug(f"关闭播放失败: {e}")

        # Stop LLM server if we started it
        if self._llm_proc and self._llm_proc.poll() is None:
            log.info("停止 LLM 服务...")
            self._llm_proc.terminate()
            try:
                self._llm_proc.wait(timeout=3)
                log.info("LLM 服务已停止")
            except subprocess.TimeoutExpired:
                log.warning("LLM 服务未响应，强制终止")
                self._llm_proc.kill()
                self._llm_proc.wait()

        log.info("已退出")


# ---------------------------------------------------------------------------
# CLI entry
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="实时语音交互")
    parser.add_argument(
        "--config", "-c",
        default=os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "config.yaml"))
    parser.add_argument(
        "--no-server", action="store_true",
        help="不自动启动 LLM 服务")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    if not os.path.exists(args.config):
        print(f"错误: 配置文件不存在: {args.config}")
        sys.exit(1)

    app = VoiceInteraction(args.config, auto_start_llm=not args.no_server)
    app.run()


if __name__ == "__main__":
    main()
