"""独立的音频降噪模块（GTCRN 语音增强）。

用 sherpa-onnx 的 OnlineSpeechDenoiser 做流式降噪，去稳态背景噪声（风扇/电机/空调）。
可作为独立工具使用，也可 import 到任意工程（ASR 前、唤醒前）。

用法（命令行）:
    python3 denoiser.py input.wav -o output.wav          # 离线降噪一段 wav
    python3 denoiser.py --record-denoise dev.wav         # 麦克风录音并实时降噪存 wav

作为模块接入:
    from denoiser import SpeechDenoise
    d = SpeechDenoise()
    data = d.load_wav('/tmp/in.wav')        # → float32 [-1,1]
    clean = d.process(data)                # → 降噪后 float32
    d.save_wav(clean, '/tmp/out.wav')
"""
import argparse
import sys
import os
import wave
import time

import numpy as np

DEFAULT_MODEL_PATH = '/home/iflytek/models/ns/gtcrn_simple.onnx'
DEFAULT_RATE = 16000


class SpeechDenoise:
    """GTCRN 流式降噪：把稳态背景噪声去掉，保留语音。

    底层 sherpa_onnx.OnlineSpeechDenoiser（GTCRN），实时块流式处理。
    """

    def __init__(self, model_path=DEFAULT_MODEL_PATH, num_threads=2, provider='cpu'):
        import sherpa_onnx
        cfg = sherpa_onnx.OnlineSpeechDenoiserConfig()
        cfg.model.gtcrn.model = model_path
        cfg.model.num_threads = num_threads
        cfg.model.provider = provider
        self._denoiser = sherpa_onnx.OnlineSpeechDenoiser(cfg)
        self.sample_rate = self._denoiser.sample_rate

    def reset(self):
        """清空 denoiser 内部状态（开始新一段录音/语音时调用）。"""
        self._denoiser.reset()

    def load_wav(self, path):
        """读 wav → float32 numpy（归一化到 [-1,1]）。"""
        with wave.open(path, 'rb') as w:
            sr = w.getframerate()
            nch = w.getnchannels()
            data = w.readframes(w.getnframes())
        import numpy as np
        x = np.frombuffer(data, dtype=np.int16).astype(np.float32) / 32768.0
        if nch > 1:
            x = x.reshape(-1, nch).mean(axis=1)  # 多声道取平均
        return x, sr

    def save_wav(self, samples, path, sample_rate=DEFAULT_RATE):
        """float32 [-1,1] → int16 wav 写出。"""
        import numpy as np
        x = np.clip(samples, -1.0, 1.0)
        raw = (x * 32767.0).astype(np.int16)
        with wave.open(path, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(sample_rate)
            w.writeframes(raw.tobytes())

    def process(self, samples, sample_rate=DEFAULT_RATE):
        """流式降噪一段样本（float32）。返回降噪后 float32（长度≈输入）。"""
        import numpy as np
        samples = np.asarray(samples, dtype=np.float32)
        shift = self._denoiser.frame_shift_in_samples
        out_chunks = []
        for i in range(0, len(samples) - shift, shift):
            chunk = samples[i:i + shift].tolist()
            if len(chunk) < 8:
                continue
            res = self._denoiser.run(chunk, sample_rate)
            # DenoisedAudio：取 samples 字段（float 列表）
            clean = res.samples if hasattr(res, 'samples') else res
            out_chunks.append(np.asarray(clean, dtype=np.float32))
        return np.concatenate(out_chunks) if out_chunks else np.array([], dtype=np.float32)


def _cli():
    p = argparse.ArgumentParser(description='GTCRN 语音降噪工具')
    p.add_argument('input', help='输入 wav 路径')
    p.add_argument('-o', '--output', default='/tmp/denoised.wav', help='输出 wav 路径')
    p.add_argument('--model', default=DEFAULT_MODEL_PATH)
    a = p.parse_args()
    d = SpeechDenoise(model_path=a.model)
    x, sr = d.load_wav(a.input)
    clean = d.process(x, sr)
    d.save_wav(clean, a.output, sr)
    import numpy as np
    print(f'输入RMS={np.sqrt(np.mean(x**2)):.4f} → 降噪RMS={np.sqrt(np.mean(clean**2)):.4f} '
          f'(能量降 {1-np.sqrt(np.mean(clean**2))/max(np.sqrt(np.mean(x**2)),1e-9):.0%})')
    print(f'输出: {a.output}')


if __name__ == '__main__':
    _cli()