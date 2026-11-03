#!/usr/bin/env python3
"""
TTS (Text-to-Speech) test using sherpa-onnx Matcha-TTS + ALSA playback.

Uses the Matcha-TTS acoustic model with Vocos vocoder for high-quality
Chinese + English speech synthesis at 16kHz.

Usage:
  python3 tts_test.py --text "你好，欢迎使用语音合成"
  python3 tts_test.py --text "现在时间是下午三点" --speed 1.2
  python3 tts_test.py -i                                    # interactive mode
  python3 tts_test.py --text "你好" -o hello.wav             # save to file
"""

import argparse
import math
import struct
import subprocess
import time
import wave

import numpy as np
import sherpa_onnx


# ---------------------------------------------------------------------------
# Mixer helpers (ES8388 / RK3576)
# ---------------------------------------------------------------------------

def pct_to_db(pct):
    if pct >= 100:
        return 0.0
    if pct <= 0:
        return -96.0
    return 20.0 * math.log10(pct / 100.0)


def setup_playback(card, pct):
    """Set playback mixer — same dB-based method as audio_test.py."""
    db_val = pct_to_db(pct)
    # Output 1/2 + PCM all use dB values, just like audio_test.py
    for ctl in ["Output 1", "Output 2", "PCM"]:
        subprocess.run(
            ["amixer", "-c", str(card), "sset", ctl, "--", f"{db_val:.1f}dB"],
            capture_output=True, check=False)
    # Enable speaker amplifier
    for ctl in ["Speaker", "spk switch"]:
        subprocess.run(
            ["amixer", "-c", str(card), "sset", ctl, "on"],
            capture_output=True, check=False)
    print(f"  Playback volume: {pct}% ({db_val:.1f} dB)")


# ---------------------------------------------------------------------------
# TTS engine
# ---------------------------------------------------------------------------

def build_tts(args):
    config = sherpa_onnx.OfflineTtsConfig(
        model=sherpa_onnx.OfflineTtsModelConfig(
            matcha=sherpa_onnx.OfflineTtsMatchaModelConfig(
                acoustic_model=args.acoustic_model,
                vocoder=args.vocoder,
                lexicon=args.lexicon,
                tokens=args.tokens,
                data_dir=args.data_dir,
            ),
            num_threads=args.num_threads,
            debug=args.debug,
        ),
        max_num_sentences=args.max_num_sentences,
        rule_fsts=args.rule_fsts if args.rule_fsts else "",
    )
    if not config.validate():
        raise ValueError("TTS config validation failed")
    return sherpa_onnx.OfflineTts(config)


def write_wav_stereo(path, samples_float32, sample_rate):
    """Write float32 mono samples as a stereo S16_LE WAV file.

    ES8388 I2S is hard-wired for stereo — mono WAVs play at double speed
    because the byte clock is wrong.  We duplicate the mono channel to both
    left and right, exactly like audio_test.py's loopback pipeline.
    """
    mono = (np.asarray(samples_float32, dtype=np.float32)
             .clip(-1.0, 1.0) * 32767).astype(np.int16)
    stereo = np.column_stack([mono, mono]).ravel()

    with wave.open(path, "wb") as wf:
        wf.setnchannels(2)
        wf.setsampwidth(2)        # S16_LE = 2 bytes
        wf.setframerate(sample_rate)
        wf.writeframes(stereo.tobytes())


def play_audio(audio, gain=1.5, device="plughw:0,0"):
    """Apply gain, write stereo WAV, play with aplay."""
    samples = np.array(audio.samples, dtype=np.float32)
    samples = (samples * gain).clip(-1.0, 1.0)

    tmp_wav = "/tmp/tts_playback.wav"
    write_wav_stereo(tmp_wav, samples, audio.sample_rate)

    subprocess.run(["aplay", "-q", "-D", device, tmp_wav], check=False)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="TTS test (sherpa-onnx Matcha-TTS + ALSA playback)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 tts_test.py --text "你好"
  python3 tts_test.py --text "现在时间是下午三点" --speed 1.2
  python3 tts_test.py -i                                    # interactive mode
  python3 tts_test.py --text "你好" -o hello.wav             # save to file

Volume:
  --gain N               Digital gain, 1.0-4.0 (default 1.5)
  --playback-volume N    Speaker volume 0-100%% (default 100).
                         100%%=0dB, same as audio_test.py.
        """,
    )

    # ---- Model files ----
    model_dir = "/home/iflytek/models/tts"
    parser.add_argument("--acoustic-model",
                        default=f"{model_dir}/model-steps-3.onnx")
    parser.add_argument("--vocoder",
                        default=f"{model_dir}/vocos-16khz-univ.onnx")
    parser.add_argument("--lexicon",
                        default=f"{model_dir}/lexicon.txt")
    parser.add_argument("--tokens",
                        default=f"{model_dir}/tokens.txt")
    parser.add_argument("--data-dir",
                        default=f"{model_dir}/espeak-ng-data")
    parser.add_argument("--rule-fsts",
                        default=f"{model_dir}/phone-zh.fst,"
                                f"{model_dir}/date-zh.fst,"
                                f"{model_dir}/number-zh.fst")

    # ---- Text input ----
    parser.add_argument("--text", "-t", default=None,
                        help="Text to synthesize")
    parser.add_argument("-i", "--interactive", action="store_true",
                        help="Interactive mode")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Speech speed (default: 1.0, range: 0.5-2.0)")
    parser.add_argument("--sid", type=int, default=0)

    # ---- Output ----
    parser.add_argument("--output", "-o", default="/tmp/tts_output.wav")
    parser.add_argument("--no-play", action="store_true",
                        help="Skip playback, only save to file")

    # ---- Performance ----
    parser.add_argument("--num-threads", type=int, default=4)
    parser.add_argument("--max-num-sentences", type=int, default=1)
    parser.add_argument("--debug", action="store_true")

    # ---- Playback ----
    parser.add_argument("--card", type=int, default=0)
    # ---- Volume controls ----
    parser.add_argument("--gain", type=float, default=2.0,
                        help="Digital gain multiplier (default: 2.0)")
    parser.add_argument("--playback-volume", type=int, default=100,
                        help="Speaker volume 0-100%% (default: 100). "
                             "Same as audio_test.py: 100%%=0dB, 50%%=-6dB.")
    parser.add_argument("-D", "--device", default="plughw:0,0",
                        help="ALSA playback device")

    args = parser.parse_args()

    # ---- Mixer setup ----
    setup_playback(args.card, args.playback_volume)

    # ---- Build TTS engine ----
    print("Loading TTS model...")
    try:
        tts = build_tts(args)
    except Exception as e:
        print(f"Error: {e}")
        print()
        print("Model files not found.")
        sys.exit(1)
    print("  Ready.\n")

    # ---- Interactive mode ----
    if args.interactive:
        print("Interactive TTS. Type text and press Enter. quit/exit/q = exit.")
        print()
        speed = args.speed
        while True:
            try:
                line = input("TTS > ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not line:
                continue
            if line.lower() in ("quit", "exit", "q"):
                break
            if line.lower().startswith("vol="):
                try:
                    args.playback_volume = int(line[4:])
                    setup_playback(args.card, args.playback_volume)
                except ValueError:
                    print("  Usage: vol=80")
                continue
            if line.lower().startswith("speed="):
                try:
                    speed = float(line[6:])
                    print(f"  Speed: {speed}")
                except ValueError:
                    print("  Usage: speed=1.2")
                continue

            t0 = time.perf_counter()
            audio = tts.generate(line, sid=args.sid, speed=speed)
            t1 = time.perf_counter()
            sherpa_onnx.write_wave(args.output, audio.samples, audio.sample_rate)

            gen_time = t1 - t0
            audio_dur = len(audio.samples) / audio.sample_rate
            rtf = gen_time / audio_dur

            print(f"  Audio: {audio_dur:.2f}s  "
                  f"RTF: {rtf:.3f}  "
                  f"({audio.sample_rate} Hz, {len(audio.samples)} samples)")

            if not args.no_play:
                play_audio(audio, args.gain, args.device)

    # ---- Batch mode ----
    else:
        text = args.text if args.text else "你好，欢迎使用语音合成系统。"
        print(f"Text:  {text[:80]}{'...' if len(text) > 80 else ''}")
        print(f"Speed: {args.speed}")

        t0 = time.perf_counter()
        audio = tts.generate(text, sid=args.sid, speed=args.speed)
        t1 = time.perf_counter()
        sherpa_onnx.write_wave(args.output, audio.samples, audio.sample_rate)

        gen_time = t1 - t0
        audio_dur = len(audio.samples) / audio.sample_rate
        rtf = gen_time / audio_dur

        print(f"Audio:  {audio_dur:.2f}s  "
              f"({audio.sample_rate} Hz, {len(audio.samples)} samples)")
        print(f"Gen:    {gen_time:.2f}s")
        print(f"RTF:    {rtf:.3f}  "
              f"({'faster' if rtf < 1.0 else 'slower'} than real-time)")
        print(f"Output: {args.output}")

        if not args.no_play:
            print("Playing...")
            play_audio(audio, args.gain, args.device)


if __name__ == "__main__":
    main()
