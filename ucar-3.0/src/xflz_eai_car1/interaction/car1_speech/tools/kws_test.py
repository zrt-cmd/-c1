#!/usr/bin/env python3
"""
Keyword spotting (wake word) test using sherpa-onnx KeywordSpotter with ALSA
microphone input.

The KeywordSpotter continuously listens for configured wake words from the
microphone. When a keyword is detected, it is printed and the stream resets
for the next detection.

Model: sherpa-onnx-kws-zipformer-zh-en-3M-2025-12-20
  - Supports Chinese and English keywords via pinyin + English phoneme tokens
  - ~3 MB, runs on CPU

Wake words (configurable via --keywords-file):
  小飞同学, 你好小飞, 小飞小飞

Recording defaults (consistent with audio_test.py):
  - Mic PGA: +18dB (--capture-volume 88, optimized for 30-50cm)
  - ALC: OFF (avoids noise floor pumping during silence)
  - Use --voice-preset for quick setup with recommended gains

Usage:
  python3 kws_test.py                                        # optimized defaults
  python3 kws_test.py --voice-preset                         # explicit ASR preset
  python3 kws_test.py --keywords-file my_keywords.txt        # custom keywords
  python3 kws_test.py --keywords-threshold 0.3               # more sensitive
  python3 kws_test.py --capture-volume 100                   # max mic gain (+24dB)
  python3 kws_test.py --enable-alc                           # enable ALC (not recommended)
"""

import argparse
import signal
import sys

import sherpa_onnx

# Import AudioMixer for consistent gain control with audio_test.py
from audio_mixer import AudioMixer, capture_volume_pct_to_db
import subprocess


# ---------------------------------------------------------------------------
# Mixer helpers (consistent with audio_test.py)
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
        description="Keyword spotting test (sherpa-onnx + ALSA)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 kws_test.py                                # optimized defaults (mic +18dB)
  python3 kws_test.py --voice-preset                 # explicit ASR preset
  python3 kws_test.py --keywords-file my_keywords.txt
  python3 kws_test.py --keywords-threshold 0.15      # more sensitive (more false positives)
  python3 kws_test.py --keywords-threshold 0.4       # stricter (fewer false positives)
  python3 kws_test.py --capture-volume 100           # max mic gain (+24dB PGA)
  python3 kws_test.py --enable-alc                   # enable ALC (not recommended)
        """,
    )

    # ---- Model files ----
    model_dir = "/home/iflytek/models/zipformer_kws"
    parser.add_argument("--tokens", default=f"{model_dir}/tokens.txt")
    parser.add_argument("--encoder",
                        default=f"{model_dir}/encoder-epoch-13-avg-2-chunk-16-left-64.int8.onnx")
    parser.add_argument("--decoder",
                        default=f"{model_dir}/decoder-epoch-13-avg-2-chunk-16-left-64.onnx")
    parser.add_argument("--joiner",
                        default=f"{model_dir}/joiner-epoch-13-avg-2-chunk-16-left-64.int8.onnx")
    parser.add_argument("--keywords-file",
                        default="keywords.txt",
                        help="Keywords file, one per line in ppinyin format: "
                             "x iǎo f ēi t óng x ué @小飞同学")

    # ---- Device ----
    parser.add_argument("-D", "--device", default="plughw:0,0",
                        help="ALSA PCM device (default: plughw:0,0)")

    # ---- Decoding ----
    parser.add_argument("--keywords-threshold", type=float, default=0.25,
                        help="Detection sensitivity. Lower = more sensitive (more false "
                             "positives). Higher = stricter (more misses). (default: 0.25)")
    parser.add_argument("--keywords-score", type=float, default=1.0,
                        help="Bonus score for each token in keyword (default: 1.0)")
    parser.add_argument("--num-trailing-blanks", type=int, default=2,
                        help="Trailing blanks after keyword (default: 2)")
    parser.add_argument("--max-active-paths", type=int, default=4)

    # ---- Performance ----
    parser.add_argument("--num-threads", type=int, default=4)
    parser.add_argument("--sample-rate", type=int, default=16000)

    # ---- Capture volume & mic boost ----
    parser.add_argument("--card", type=int, default=0)
    parser.add_argument("--capture-volume", type=int, default=88,
                        help="Mic capture volume 0-100%% (default: 88, optimized for 30-50cm)")
    parser.add_argument("--voice-preset", action="store_true",
                        help="Apply ASR-optimized defaults (mic +18dB, ALC off) — same as audio_test.py")
    parser.add_argument("--enable-alc", action="store_true",
                        help="Enable ALC (automatic level control) — NOT recommended, raises noise floor")

    # ---- Read chunk ----
    parser.add_argument("--chunk-size", type=int, default=4000,
                        help="Samples per ALSA read (default: 4000, ~250ms @ 16kHz)")

    # ---- Debug ----
    parser.add_argument("--debug", action="store_true")

    args = parser.parse_args()

    # -----------------------------------------------------------------------
    # Audio mixer setup (consistent with audio_test.py best practices)
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

        # ALC off by default (user must explicitly enable with --enable-alc)
        mixer = AudioMixer(card=args.card)
        if args.enable_alc:
            mixer.set_alc(True)
            print("  ALC: enabled (--enable-alc specified)")
        else:
            mixer.set_alc(False)
            print("  ALC: off (recommended for stable SNR)")

    # -----------------------------------------------------------------------
    # Build keyword spotter
    # -----------------------------------------------------------------------
    print("\nInitializing keyword spotter...")
    print(f"  tokens:    {args.tokens}")
    print(f"  encoder:   {args.encoder}")
    print(f"  decoder:   {args.decoder}")
    print(f"  joiner:    {args.joiner}")
    print(f"  keywords:  {args.keywords_file}")
    print(f"  threshold: {args.keywords_threshold}")
    print(f"  threads:   {args.num_threads}")

    # Show configured keywords
    with open(args.keywords_file) as f:
        for line in f:
            line = line.strip()
            if line and '@' in line:
                print(f"    -> {line.split('@')[-1].strip()}")
    print()

    kws = sherpa_onnx.KeywordSpotter(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        keywords_file=args.keywords_file,
        num_threads=args.num_threads,
        sample_rate=args.sample_rate,
        max_active_paths=args.max_active_paths,
        keywords_score=args.keywords_score,
        keywords_threshold=args.keywords_threshold,
        num_trailing_blanks=args.num_trailing_blanks,
        provider="cpu",
    )

    # -----------------------------------------------------------------------
    # Open ALSA device
    # -----------------------------------------------------------------------
    print(f"Opening ALSA device: {args.device}")
    alsa = sherpa_onnx.Alsa(args.device)
    actual_rate = alsa.actual_sample_rate
    expected_rate = alsa.expected_sample_rate
    print(f"  expected sample rate: {expected_rate}")
    print(f"  actual sample rate:   {actual_rate}")
    sample_rate = actual_rate if actual_rate > 0 else args.sample_rate

    # -----------------------------------------------------------------------
    # Keyword spotting loop
    #
    # Architecture:
    #   ALSA → KeywordSpotter (continuous streaming) → on detection: print + reset
    #
    # The KWS model continuously monitors the audio stream. When a keyword is
    # detected, the stream is reset so it can immediately start listening for
    # the next keyword.
    # -----------------------------------------------------------------------
    print(f"\nListening for wake words... (Ctrl+C to stop)\n")

    running = True

    def on_sigint(signum, frame):
        nonlocal running
        running = False
        print("\n\nStopping...")

    signal.signal(signal.SIGINT, on_sigint)

    stream = kws.create_stream()

    try:
        while running:
            samples = alsa.read(args.chunk_size)
            if not samples:
                continue

            stream.accept_waveform(sample_rate, samples)

            while kws.is_ready(stream):
                kws.decode_stream(stream)

            result = kws.get_result(stream)
            if result:
                # Remove the display-name suffix if present
                keyword = result.split("@")[-1].strip() if "@" in result else result.strip()
                print(f"[Wake] {keyword}")
                kws.reset_stream(stream)

    except KeyboardInterrupt:
        pass

    print("Done.")


if __name__ == "__main__":
    main()
