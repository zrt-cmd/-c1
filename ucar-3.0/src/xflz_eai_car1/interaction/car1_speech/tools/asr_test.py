#!/usr/bin/env python3
"""
ASR streaming test using sherpa-onnx with ALSA microphone + RKNN.

Uses sherpa-onnx built-in endpoint detection (same as RK3588 version).
This preserves the recognizer's streaming context across the entire session,
which is critical for Zipformer accuracy.

Recording defaults (consistent with audio_test.py and kws_test.py):
  - Mic PGA: +18dB (--capture-volume 88, optimized for 30-50cm)
  - ALC: OFF (avoids noise floor pumping during silence)
  - Use --voice-preset for quick setup with recommended gains

VAD endpoint detection (tuned to reduce word loss):
  - rule1_min_trailing_silence: 3.0s (was 2.4s in RK3588)
  - rule2_min_trailing_silence: 2.0s (was 1.2s, increased to prevent tail loss)
  - rule3_min_utterance_length: 2.0s (was 300s, now requires 2s minimum speech)
  - blank_penalty: 0.3 (was 0.0, helps reduce word omissions)

Hotwords (default enabled with hotwords.txt):
  - Used to bias recognition toward specific names/phrases
  - Helps fix "小飞" → "小飞机" type errors
  - Score 2.0 gives strong preference to hotwords

Mimics the behavior of:
  sherpa-onnx-alsa --tokens=~/models/zipformer_onnx/tokens.txt \\
                   --encoder=~/models/zipformer_onnx/encoder-epoch-99-avg-1.int8.onnx \\
                   --decoder=~/models/zipformer_onnx/decoder-epoch-99-avg-1.onnx \\
                   --joiner=~/models/zipformer_onnx/joiner-epoch-99-avg-1.int8.onnx \\
                   --provider=cpu --num-threads=1 \\
                   --decoding-method=greedy_search plughw:0,0

Usage:
  python3 asr_test.py                                       # optimized defaults + hotwords
  python3 asr_test.py --voice-preset                        # explicit ASR preset
  python3 asr_test.py --provider cpu                        # CPU fallback
  python3 asr_test.py --capture-volume 100                  # max mic gain (+24dB)
  python3 asr_test.py --enable-alc                          # enable ALC (not recommended)
  python3 asr_test.py --hotwords-file my_hotwords.txt       # custom hotwords
  python3 asr_test.py --hotwords-file ""                    # disable hotwords
  python3 asr_test.py --rule2-min-trailing-silence 1.5      # faster endpoint (more tail loss)
"""

import argparse
import signal
import sys

import numpy as np
import sherpa_onnx

# Import AudioMixer for consistent gain control with audio_test.py
from audio_mixer import AudioMixer, capture_volume_pct_to_db
import subprocess

# 流式降噪（GTCRN，去风扇/电机稳态噪声）
from denoiser import SpeechDenoise


def make_denoiser_or_none(model=None):
    """按需创建降噪器；失败时不阻塞（返回 None）。默认用 /models/ns/gtcrn_simple.onnx。"""
    try:
        if model is None:
            from denoiser import DEFAULT_MODEL_PATH
            model = DEFAULT_MODEL_PATH
        return SpeechDenoise(model_path=model)
    except Exception as e:
        print(f"[denoise] 降噪器创建失败，跳过降噪: {e}")
        return None


# ---------------------------------------------------------------------------
# Mixer helpers (consistent with audio_test.py and kws_test.py)
# ---------------------------------------------------------------------------

def switch_input_to_mic(card):
    """Switch ES8388 input mux from Line to Mic inputs."""
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "Left Line Mux", "MicL"],
        capture_output=True, check=False)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "Right Line Mux", "MicR"],
        capture_output=True, check=False)
    print("  Input mux: MicL / MicR")


def apply_capture_volume(card, pct):
    """Apply mic capture volume using AudioMixer (same as audio_test.py).

    - 0%:        Mute mic
    - 1-49%:     PGA 0dB, digital cut -30..0dB (attenuation, hurts SNR)
    - 50%:       Neutral (PGA 0dB, digital 0dB)
    - 51-100%:   PGA boost 0..+24dB (analog gain, best SNR)
    """
    mixer = AudioMixer(card=card)
    if pct <= 0:
        mixer.mute_mic()
        print("  Capture volume: MUTED")
        return
    mixer.unmute_mic()
    preamp_db, digital_db = capture_volume_pct_to_db(pct)
    mixer.set_mic_preamp_db(preamp_db)
    mixer.set_capture_digital_db(digital_db)
    print(f"  Capture volume: {pct}% → PGA {preamp_db:+.1f}dB, digital {digital_db:+.1f}dB")


def apply_voice_preset(card):
    """Apply optimized ASR defaults (same as audio_test.py --voice-preset)."""
    mixer = AudioMixer(card=card)
    mixer.apply_voice_preset()
    print("  Voice preset: mic +18dB (88%), ALC off, speaker -3dB")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="ASR streaming test (sherpa-onnx + ALSA + VAD + RKNN)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 asr_test.py                                # optimized defaults (mic +18dB)
  python3 asr_test.py --voice-preset                 # explicit ASR preset
  python3 asr_test.py --provider cpu                 # CPU fallback (no RKNN)
  python3 asr_test.py --capture-volume 100           # max mic gain (+24dB PGA)
  python3 asr_test.py --enable-alc                   # enable ALC (not recommended)
        """,
    )

    # ---- Model files ----
    parser.add_argument("--tokens",
                        default="/home/iflytek/models/zipformer_onnx/tokens.txt")
    parser.add_argument("--encoder",
                        default="/home/iflytek/models/zipformer_onnx/encoder-epoch-99-avg-1.int8.onnx")
    parser.add_argument("--decoder",
                        default="/home/iflytek/models/zipformer_onnx/decoder-epoch-99-avg-1.onnx")
    parser.add_argument("--joiner",
                        default="/home/iflytek/models/zipformer_onnx/joiner-epoch-99-avg-1.int8.onnx")

    # ---- Device ----
    parser.add_argument("-D", "--device", default="plughw:0,0",
                        help="ALSA PCM device (default: plughw:0,0)")

    # ---- Provider & performance ----
    parser.add_argument("--provider", default="cpu",
                        choices=["cpu", "rknn"],
                        help="Execution provider (default: cpu，ONNX 模型)")
    parser.add_argument("--num-threads", type=int, default=1,
                        help="For RKNN, this is NPU core mask (default: 1 = auto)")
    parser.add_argument("--sample-rate", type=int, default=16000)
    parser.add_argument("--denoise", dest="denoise", action="store_true",
                        help="启用 GTCRN 流式降噪（去风扇/电机噪声）")
    parser.add_argument("--no-denoise", dest="denoise", action="store_false",
                        help="禁用流式降噪")
    parser.set_defaults(denoise=True)

    # ---- Decoding ----
    parser.add_argument("--decoding-method", default="greedy_search",
                        choices=["greedy_search", "modified_beam_search"],
                        help="Decoding method (default: greedy_search). "
                             "Use modified_beam_search for hotwords (more partial result updates)")
    parser.add_argument("--blank-penalty", type=float, default=0.25,
                        help="Penalty on blank symbol (default: 0.25). "
                             "Lower = less word duplication, higher = less word omission")

    # ---- Endpoint detection (built-in, replaces external VAD) ----
    parser.add_argument("--rule1-min-trailing-silence", type=float, default=4.5,
                        help="Rule 1: min trailing silence for endpoint (default: 4.5s, "
                             "increased from 3.0s to reduce noise-triggered false endpoints)")
    parser.add_argument("--rule2-min-trailing-silence", type=float, default=1.5,
                        help="Rule 2: min trailing silence after non-empty result (default: 1.5s, "
                             "balanced between responsiveness and tail loss prevention)")
    parser.add_argument("--rule3-min-utterance-length", type=float, default=300.0,
                        help="Rule 3: min utterance length to trigger endpoint (default: 300.0s, "
                             "effectively disabled - use rule1/rule2 for endpoint control)")

    # ---- Feature extraction ----
    parser.add_argument("--feature-dim", type=int, default=80)

    # ---- Hotwords ----
    parser.add_argument("--hotwords-file", type=str, default="",
                        help="Path to hotwords file for biasing (requires --decoding-method=modified_beam_search). "
                             "One phrase per line, tokens separated by space, e.g. '小 飞 你 好'")
    parser.add_argument("--hotwords-score", type=float, default=2.0,
                        help="Hotword score bonus per token (default: 2.0, higher = stronger bias)")

    # ---- Capture volume & mic boost ----
    parser.add_argument("--card", type=int, default=0)
    parser.add_argument("--capture-volume", type=int, default=95,
                        help="Mic capture volume 0-100%% (default: 95 = +22dB PGA, "
                             "balanced between signal strength and noise floor)")
    parser.add_argument("--voice-preset", action="store_true",
                        help="Apply ASR-optimized defaults (mic +18dB, ALC off) — same as audio_test.py")
    parser.add_argument("--enable-alc", action="store_true", default=True,
                        help="Enable ALC with Noise Gate (default: enabled, auto-adjusts gain for varying distance)")
    parser.add_argument("--disable-alc", action="store_true",
                        help="Disable ALC, use fixed PGA gain (predictable but distance-sensitive)")

    # ---- Debug ----
    parser.add_argument("--debug", action="store_true")

    args = parser.parse_args()

    # -----------------------------------------------------------------------
    # Audio mixer setup (consistent with audio_test.py and kws_test.py)
    # -----------------------------------------------------------------------
    print("Audio mixer setup...")

    # CRITICAL: Switch input mux to microphone (ES8388 may default to Line input)
    switch_input_to_mic(args.card)

    if args.voice_preset:
        # Use optimized ASR preset (mic +18dB, ALC off)
        apply_voice_preset(args.card)
    else:
        # Manual capture volume
        pct = max(0, min(100, args.capture_volume))
        apply_capture_volume(args.card, pct)

        # ALC configuration
        mixer = AudioMixer(card=args.card)
        if args.disable_alc:
            mixer.set_alc(False)
            print("  ALC: off (fixed gain)")
        else:
            # ALC on + Noise Gate: auto-adjust gain, but mute during silence
            mixer.set_alc(True, target_pct=80, max_pga_step=5, noise_gate=True)
            print("  ALC: enabled (target=80%, max_pga=5, noise_gate=ON)")

    # -----------------------------------------------------------------------
    # Build recognizer (with built-in endpoint detection, matches RK3588 config)
    # -----------------------------------------------------------------------
    print("Initializing recognizer...")
    print(f"  tokens:   {args.tokens}")
    print(f"  encoder:  {args.encoder}")
    print(f"  decoder:  {args.decoder}")
    print(f"  joiner:   {args.joiner}")
    print(f"  provider: {args.provider}")
    print(f"  threads:  {args.num_threads}")
    print(f"  method:   {args.decoding_method}")
    print(f"  blank_penalty: {args.blank_penalty}")
    print(f"  rule1 silence: {args.rule1_min_trailing_silence}s")
    print(f"  rule2 silence: {args.rule2_min_trailing_silence}s")
    print(f"  rule3 min_utt: {args.rule3_min_utterance_length}s")
    if args.hotwords_file:
        import os
        if os.path.exists(args.hotwords_file):
            print(f"  hotwords: {args.hotwords_file} (score={args.hotwords_score})")
        else:
            print(f"  hotwords: {args.hotwords_file} NOT FOUND (will be ignored)")
    else:
        print(f"  hotwords: none")

    recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        num_threads=args.num_threads,
        sample_rate=args.sample_rate,
        feature_dim=args.feature_dim,
        enable_endpoint_detection=True,
        rule1_min_trailing_silence=args.rule1_min_trailing_silence,
        rule2_min_trailing_silence=args.rule2_min_trailing_silence,
        rule3_min_utterance_length=args.rule3_min_utterance_length,
        decoding_method=args.decoding_method,
        provider=args.provider,
        hotwords_file=args.hotwords_file,
        hotwords_score=args.hotwords_score,
        blank_penalty=args.blank_penalty,
    )

    # -----------------------------------------------------------------------
    # Open ALSA device
    # -----------------------------------------------------------------------
    print(f"\nOpening ALSA device: {args.device}")
    alsa = sherpa_onnx.Alsa(args.device)
    actual_rate = alsa.actual_sample_rate
    expected_rate = alsa.expected_sample_rate
    print(f"  expected sample rate: {expected_rate}")
    print(f"  actual sample rate:   {actual_rate}")
    # sherpa_onnx.Alsa internally resamples to 16kHz — always feed the
    # recognizer at the model's rate, regardless of hardware capture rate.
    sample_rate = args.sample_rate

    # -----------------------------------------------------------------------
    # Streaming recognition loop
    #
    # Architecture (same as RK3588 offline_stt.py):
    #   ALSA → Recognizer (streaming decode + built-in endpoint detection)
    #        → is_endpoint() → reset stream, keep going
    #
    # A SINGLE stream is used across the whole session (only reset on endpoint,
    # not recreated).  This preserves the recognizer's temporal context, which
    # is critical for Zipformer accuracy.
    # -----------------------------------------------------------------------
    samples_per_read = int(0.1 * sample_rate)  # 100 ms per read (same as RK3588)
    print(f"\nsamples per read: {samples_per_read} (~100ms)")
    print(f"\nStarted! Please speak (Ctrl+C to stop)\n")

    running = True

    def on_sigint(signum, frame):
        nonlocal running
        running = False
        print("\n\nStopping...")

    signal.signal(signal.SIGINT, on_sigint)

    stream = recognizer.create_stream()
    last_result = ""
    segment_id = 0

    # 流式降噪器（--denoise 开启时）
    denoiser = make_denoiser_or_none() if args.denoise else None
    if denoiser is None and args.denoise:
        print("[denoise] 降噪未启用（模型加载失败或--no-denoise）")
    elif denoiser is not None:
        print("[denoise] GTCRN 流式降噪已启用")

    try:
        while running:
            samples = alsa.read(samples_per_read)  # blocking read
            if not samples:
                continue

            # GTCRN 流式降噪（去风扇/电机稳态噪声）
            if denoiser is not None:
                samples = denoiser.process(samples, sample_rate)

            stream.accept_waveform(sample_rate, samples)
            while recognizer.is_ready(stream):
                recognizer.decode_stream(stream)

            is_endpoint = recognizer.is_endpoint(stream)
            result = recognizer.get_result(stream)

            # Live partial update — overwrite current line
            if result and (last_result != result):
                last_result = result
                print(f"\r{segment_id}:{result}", end="", flush=True)

            if is_endpoint:
                # Always emit a newline on endpoint so the next segment
                # starts on a fresh line. If no text was produced this
                # segment, still print a marker so we can see endpoint
                # firing in the logs.
                final = result if result else last_result
                if final:
                    print(f"\r{segment_id}:{final}", flush=True)
                    segment_id += 1
                else:
                    print(f"\r[endpoint, no text]", flush=True)
                last_result = ""
                recognizer.reset(stream)

    except KeyboardInterrupt:
        pass

    if last_result:
        sys.stdout.write("\n")
        sys.stdout.flush()
    print("Done.")


if __name__ == "__main__":
    main()
