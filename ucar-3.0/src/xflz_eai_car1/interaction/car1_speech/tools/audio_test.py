#!/usr/bin/env python3
"""
ES8388 audio loopback test tool — record + play simultaneously, or record/play separately.

Modes:
  loopback (default):  record and play at the same time (arecord | aplay pipeline).
                        Optionally save the recording to a file.
  record:              record only, save to a WAV file.
  play:                play an existing WAV file.

Default parameters match ES8388 on RK3576:
  Card:    0 (rockchip-es8388)
  Device:  hw:0,0
  Rate:    16000 Hz
  Format:  S16_LE
  Channels: 1 (mono, board mic is single-channel — use -c 2 for stereo if needed)

Mixer volume is NOT changed unless you pass --playback-volume / --capture-volume.
Gains are applied through audio_mixer.py (hardware-safe caps for 4Ω 3W speaker
and close-talk board mic):
  --capture-volume  0-100  → mic PGA 0..+18dB (50% neutral, digital fixed 0dB)
  --playback-volume 0-100  → Output 1/2 capped at -12dB (100%→-12dB, 67%→-15dB)
Use --list-mixer to inspect current mixer state, or --voice-preset for the
recommended ASR+TTS defaults.

Tuning tips:
  --voice-preset                                 # recommended: mic +12dB / spk -15dB (1.1W)
  --playback-volume 100 --capture-volume 50      # spk max-safe (-12dB, 2.2W), mic neutral
  --capture-volume 100                           # mic PGA +18dB (still safe)
  --list-mixer                                   # inspect all mixer controls
  --check-speaker-safety                         # verify speaker power (requires amp gain measurement)

Usage:
  # Recording (ASR-optimized)
  python3 audio_test.py --voice-preset --mode record -o rec.wav -d 5 --analyze
  python3 audio_test.py --capture-volume 100 --mode record -o rec.wav

  # Playback (max volume by default in play mode)
  python3 audio_test.py --mode play test.wav                     # auto: -12dB (2.15W)
  python3 audio_test.py --play-preset --mode play test.wav       # explicit max
  python3 audio_test.py --playback-volume 50 --mode play test.wav

  # Loopback
  python3 audio_test.py                                          # no mixer change
  python3 audio_test.py -o rec.wav                               # loopback + save

  # Device info
  python3 audio_test.py -l                                       # list PCM devices
  python3 audio_test.py --list-mixer                             # list mixer controls

  # Quality analysis
  python3 audio_test.py --mode record -o test.wav --analyze                 # analyze recording
  python3 audio_test.py --playback-volume 100 --check-speaker-safety        # check speaker safety
  python3 audio_analyzer.py test.wav                                         # analyze existing WAV
  python3 audio_analyzer.py --check-speaker -12.0                            # check speaker power
  python3 audio_analyzer.py --measure-noise                                  # measure noise floor
"""

import argparse
import os
import signal
import subprocess
import sys

from audio_mixer import (
    AudioMixer,
    capture_volume_pct_to_db,
    playback_volume_pct_to_db,
)
from audio_analyzer import (
    analyze_recording,
    check_speaker_safety,
    print_recording_analysis,
)


# ---------------------------------------------------------------------------
# Mixer helpers
# ---------------------------------------------------------------------------

def apply_playback_volume(card, pct):
    """Map 0-100% to Output 1/2 dB, with 0% being hardware mute.

    - 0%:      Mute speaker (hardware-level silence)
    - 1-100%:  Speaker Output -30..-12dB (capped at 2.15W for 4Ω 3W speaker)
    - PCM is always held at 0dB to preserve dynamic range
    """
    mixer = AudioMixer(card=card)
    if pct <= 0:
        mixer.mute_speaker()
        return -96.0  # Effectively muted
    mixer.unmute_speaker()
    db_val = playback_volume_pct_to_db(pct)
    mixer.set_pcm_db(0.0)
    mixer.set_speaker_output_db(db_val)
    return db_val


def apply_capture_volume(card, pct):
    """Map 0-100% to mic gain, with 0% being true mute.

    - 0%:        Mute mic (Left/Right Channel muted, hardware-level silence)
    - 1-49%:     PGA 0dB, digital cut -30..0dB (attenuation, hurts SNR)
    - 50%:       Neutral (PGA 0dB, digital 0dB)
    - 51-100%:   PGA boost 0..+24dB (analog gain, best SNR)
    """
    mixer = AudioMixer(card=card)
    if pct <= 0:
        mixer.mute_mic()
        return 0.0, 0.0  # Return neutral dB (actual state is muted)
    mixer.unmute_mic()
    preamp_db, digital_db = capture_volume_pct_to_db(pct)
    mixer.set_mic_preamp_db(preamp_db)
    mixer.set_capture_digital_db(digital_db)
    return preamp_db, digital_db


def list_mixer_controls(card):
    """Print all mixer controls and their current values."""
    result = subprocess.run(
        ["amixer", "-c", str(card), "scontents"],
        capture_output=True, text=True, check=False)
    print(f"Mixer controls for card {card}:\n")
    print(result.stdout)


# ---------------------------------------------------------------------------
# PCM device listing
# ---------------------------------------------------------------------------

def list_devices():
    print("=== Capture Devices ===")
    try:
        subprocess.run(["arecord", "-l"], check=False)
    except FileNotFoundError:
        print("Error: arecord not found. Install alsa-utils.")
        sys.exit(1)

    print("\n=== Playback Devices ===")
    try:
        subprocess.run(["aplay", "-l"], check=False)
    except FileNotFoundError:
        pass

    print("\nPCM device names (for -D option):")
    try:
        subprocess.run(["arecord", "-L"], check=False)
    except FileNotFoundError:
        pass


# ---------------------------------------------------------------------------
# Command builders
# ---------------------------------------------------------------------------

def build_arecord_cmd(args):
    """Build the arecord command for record / loopback modes."""
    rec_dev = args.rec_device or args.device
    cmd = ["arecord", "-q"]
    cmd.extend(["-D", rec_dev])
    cmd.extend(["-f", args.format])
    cmd.extend(["-r", str(args.rate)])
    cmd.extend(["-c", str(args.channels)])
    cmd.extend(["-t", "wav"])
    if args.period_size:
        cmd.extend(["--period-size", str(args.period_size)])
    if args.buffer_size:
        cmd.extend(["--buffer-size", str(args.buffer_size)])
    if args.duration > 0:
        cmd.extend(["-d", str(args.duration)])
    return cmd


def build_aplay_cmd(args):
    """Build the aplay command for playback / loopback modes (reads from stdin)."""
    play_dev = args.playback_device or args.device
    cmd = ["aplay", "-q"]
    cmd.extend(["-D", play_dev])
    cmd.extend(["-f", args.format])
    cmd.extend(["-r", str(args.rate)])
    cmd.extend(["-c", str(args.channels)])
    cmd.extend(["-t", "wav"])
    if args.period_size:
        cmd.extend(["--period-size", str(args.period_size)])
    if args.buffer_size:
        cmd.extend(["--buffer-size", str(args.buffer_size)])
    cmd.append("-")
    return cmd


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------

def loopback_mode(args):
    """Record and play simultaneously via pipe (arecord | aplay)."""
    arecord_cmd = build_arecord_cmd(args)
    arecord_cmd.append("-")          # stdout, WAV format

    aplay_cmd = build_aplay_cmd(args)

    rec_dev = args.rec_device or args.device
    play_dev = args.playback_device or args.device

    if not args.quiet:
        print(f"Loopback: rec={rec_dev}  play={play_dev}")
        print(f"  rate={args.rate}, fmt={args.format}, channels={args.channels}")
        if args.period_size:
            print(f"  period_size={args.period_size}")
        if args.buffer_size:
            print(f"  buffer_size={args.buffer_size}")
        if args.duration > 0:
            print(f"  duration={args.duration}s")
        else:
            print(f"  duration=unlimited (Ctrl+C to stop)")
        if args.output:
            print(f"  saving to: {args.output}")
        print()

    arecord_proc = subprocess.Popen(
        arecord_cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    aplay_proc = subprocess.Popen(
        aplay_cmd, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)

    out_fh = None
    if args.output:
        out_fh = open(args.output, "wb")

    try:
        chunk = (args.period_size or 1024) * args.channels * 2
        while True:
            data = arecord_proc.stdout.read(chunk)
            if not data:
                break
            if out_fh:
                out_fh.write(data)
            try:
                aplay_proc.stdin.write(data)
                aplay_proc.stdin.flush()
            except BrokenPipeError:
                break
    except KeyboardInterrupt:
        if not args.quiet:
            print("\nStopped by user.")

    if out_fh:
        out_fh.close()

    arecord_proc.stdout.close()
    try:
        arecord_proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        arecord_proc.kill()
        arecord_proc.wait()

    if aplay_proc.poll() is None:
        aplay_proc.stdin.close()
        try:
            aplay_proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            aplay_proc.kill()
            aplay_proc.wait()

    if not args.quiet:
        rc = arecord_proc.returncode
        if rc == 0 or rc == -signal.SIGINT:
            if args.output and os.path.exists(args.output):
                size_kb = os.path.getsize(args.output) / 1024
                print(f"Done: {args.output} ({size_kb:.1f} KB)")
            else:
                print("Done.")
        else:
            print(f"Recording failed (exit code {rc})")
            sys.exit(1)


def record_only(args):
    """Record only, save to file."""
    cmd = build_arecord_cmd(args)
    rec_dev = args.rec_device or args.device

    if not args.quiet:
        cmd.remove("-q")  # show VU meter
        print(f"Recording: device={rec_dev}, rate={args.rate}, "
              f"fmt={args.format}, channels={args.channels}")
        if args.duration > 0:
            print(f"  duration={args.duration}s")
        print(f"  saving to: {args.output}")
        print("Press Ctrl+C to stop early.\n")

    cmd.append(args.output)

    proc = subprocess.Popen(cmd)
    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\nRecording stopped by user.")
        proc.send_signal(signal.SIGINT)
        proc.wait()

    if proc.returncode == 0 or proc.returncode == -signal.SIGINT:
        if os.path.exists(args.output):
            size_kb = os.path.getsize(args.output) / 1024
            print(f"Done: {args.output} ({size_kb:.1f} KB)")
    else:
        print(f"Recording failed (exit code {proc.returncode})")
        sys.exit(1)


def _prepare_playback_wav(src_path, gain=1.0):
    """Prepare a WAV file for ES8388 playback: mono→stereo + optional digital gain.

    ES8388 I2S is hard-wired for stereo, so mono WAVs play at 2x speed on raw
    hw:X,Y. We duplicate the mono channel into L/R.

    Digital gain compensates for low-amplitude source material (e.g., recordings
    made at moderate PGA gain). Uses tanh soft-clipping to prevent hard clipping
    at high gain values.

    Returns (temp_path, was_modified). Caller must delete temp_path if
    was_modified is True.
    """
    import struct
    import tempfile
    import wave

    with wave.open(src_path, "rb") as wf:
        n_channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        frame_rate = wf.getframerate()
        n_frames = wf.getnframes()
        raw_bytes = wf.readframes(n_frames)

    needs_stereo_conv = (n_channels == 1)
    needs_gain = (gain != 1.0)

    if not needs_stereo_conv and not needs_gain:
        return src_path, False

    # Only support 16-bit PCM for gain processing (most common case)
    if needs_gain and sample_width != 2:
        needs_gain = False  # Silently skip gain for non-16-bit

    if needs_gain:
        # Try numpy path first (fast), fall back to pure Python
        try:
            import numpy as np
            samples = np.frombuffer(raw_bytes, dtype=np.int16).astype(np.float32)
            samples = samples / 32768.0  # Normalize to [-1, 1]
            samples = samples * gain
            samples = np.tanh(samples)   # Soft clip
            samples = (samples * 32767).astype(np.int16)
            processed_bytes = samples.tobytes()
        except ImportError:
            # Pure Python fallback
            import math
            n_samples = len(raw_bytes) // 2
            fmt = f"<{n_samples}h"
            samples = list(struct.unpack(fmt, raw_bytes))
            samples = [max(-32767, min(32767, int(math.tanh(s / 32768.0 * gain) * 32767)))
                       for s in samples]
            processed_bytes = struct.pack(fmt, *samples)
    else:
        processed_bytes = raw_bytes

    # Interleave mono → stereo if needed
    if needs_stereo_conv:
        step = sample_width
        stereo_bytes = bytearray(len(processed_bytes) * 2)
        for i in range(0, len(processed_bytes), step):
            sample = processed_bytes[i:i + step]
            j = i * 2
            stereo_bytes[j:j + step] = sample
            stereo_bytes[j + step:j + step * 2] = sample
        final_bytes = bytes(stereo_bytes)
        final_channels = 2
    else:
        final_bytes = processed_bytes
        final_channels = n_channels

    tmp_fd, tmp_path = tempfile.mkstemp(suffix=".wav", prefix="audio_test_playback_")
    os.close(tmp_fd)
    with wave.open(tmp_path, "wb") as wf:
        wf.setnchannels(final_channels)
        wf.setsampwidth(sample_width)
        wf.setframerate(frame_rate)
        wf.writeframes(final_bytes)
    return tmp_path, True


def play_only(args):
    """Play a WAV file with optional digital gain and mono→stereo conversion.

    ES8388 I2S is hard-wired for stereo. Mono WAV files played to hw:X,Y sound
    sped-up (2x) because the byte clock is stereo. Also applies --gain digital
    boost for low-amplitude recordings (with soft-clipping to prevent distortion).
    """
    if not os.path.exists(args.file):
        print(f"Error: File not found: {args.file}")
        sys.exit(1)

    play_dev = args.playback_device or args.device
    play_file = args.file
    tmp_file = None

    try:
        play_file, was_modified = _prepare_playback_wav(args.file, gain=args.gain)
        if was_modified:
            tmp_file = play_file
            if not args.quiet:
                notes = []
                if args.gain != 1.0:
                    notes.append(f"gain={args.gain}x (soft-clipped)")
                notes.append("mono→stereo")
                print(f"[note] preprocessed: {', '.join(notes)}")
    except Exception as e:
        if not args.quiet:
            print(f"[warn] preprocessing failed ({e}), playing as-is")

    cmd = ["aplay"]
    if args.quiet:
        cmd.append("-q")
    cmd.extend(["-D", play_dev])
    cmd.append(play_file)

    if not args.quiet:
        print(f"Playing: {args.file}")
        print(f"  device={play_dev}")

    proc = subprocess.Popen(cmd)
    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\nPlayback stopped by user.")
        proc.terminate()
        proc.wait()
        if tmp_file and os.path.exists(tmp_file):
            os.unlink(tmp_file)
        return

    if tmp_file and os.path.exists(tmp_file):
        os.unlink(tmp_file)

    if proc.returncode != 0:
        print(f"Playback failed (exit code {proc.returncode})")
        sys.exit(1)
    if not args.quiet:
        print("Done.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="ES8388 audio test tool (loopback / record / play)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 audio_test.py                                     # loopback (no save)
  python3 audio_test.py --voice-preset                      # recommended ASR+TTS gains
  python3 audio_test.py --voice-preset --mode record -o rec.wav
  python3 audio_test.py --playback-volume 100               # speaker max-safe (-12dB, 2.2W)
  python3 audio_test.py --capture-volume 50                 # mic neutral (PGA 0dB)
  python3 audio_test.py --capture-volume 83                 # mic PGA ≈ +12dB
  python3 audio_test.py -o rec.wav                          # loopback + save
  python3 audio_test.py --mode record -o rec.wav            # record only
  python3 audio_test.py --mode record -o rec.wav --analyze  # record + analyze
  python3 audio_test.py --mode play test.wav                # play only
  python3 audio_test.py -l                                  # list PCM devices
  python3 audio_test.py --list-mixer                        # list mixer controls
        """,
    )

    # ---- Mode ----
    parser.add_argument(
        "--mode", choices=["loopback", "record", "play"], default="loopback",
        help="loopback (default) | record | play")

    # ---- Card & devices ----
    parser.add_argument("--card", type=int, default=0,
                        help="ALSA card number for mixer controls (default: 0)")
    parser.add_argument("-D", "--device", default="hw:0,0",
                        help="Default PCM device for both record and playback (default: hw:0,0)")
    parser.add_argument("--rec-device", default=None,
                        help="Record PCM device (overrides --device for recording)")
    parser.add_argument("--playback-device", default=None,
                        help="Playback PCM device (overrides --device for playback)")

    # ---- Audio format ----
    parser.add_argument("-r", "--rate", type=int, default=16000,
                        help="Sample rate in Hz (default: 16000)")
    parser.add_argument("-f", "--format", default="S16_LE",
                        help="Sample format (default: S16_LE)")
    parser.add_argument("-c", "--channels", type=int, default=1,
                        help="Number of channels (default: 1, mono board mic)")

    # ---- Duration ----
    parser.add_argument("-d", "--duration", type=int, default=0,
                        help="Duration in seconds (0 = unlimited, Ctrl+C to stop)")

    # ---- Output ----
    parser.add_argument("-o", "--output", default="",
                        help="Output WAV file (record / loopback modes)")

    # ---- Playback file ----
    parser.add_argument("file", nargs="?", default=None,
                        help="WAV file to play (play mode only)")

    # ---- Buffer / latency tuning ----
    parser.add_argument("--period-size", type=int, default=None,
                        help="ALSA period size in frames (smaller = lower latency, may cause xruns)")
    parser.add_argument("--buffer-size", type=int, default=None,
                        help="ALSA buffer size in frames (larger = more stable, higher latency)")

    # ---- Mixer volume ----
    parser.add_argument("--playback-volume", type=int, default=None,
                        help="Playback 0-100%% → Output 1/2 (capped at -12dB, PCM held at 0dB)")
    parser.add_argument("--capture-volume", type=int, default=None,
                        help="Capture 0-100%% → mic PGA 0..+24dB (50%% neutral, digital held at 0dB)")
    parser.add_argument("--voice-preset", action="store_true",
                        help="Apply ASR defaults (mic +18dB=88%%, ALC off, speaker -15dB=67%%)")
    parser.add_argument("--play-preset", action="store_true",
                        help="Apply playback defaults (speaker -12dB=100%%, max safe 2.15W)")
    parser.add_argument("--max-gain", action="store_true",
                        help="录音最大增益：关ALC + PGA +24dB + 数字+6dB（提升麦克风录音音量）")

    # ---- Digital gain (play mode) ----
    parser.add_argument("--gain", type=float, default=2.0,
                        help="Digital gain multiplier for playback (default: 2.0, "
                             "range 0.5-4.0, uses tanh soft-clipping)")

    # ---- Listing ----
    parser.add_argument("-l", "--list", action="store_true",
                        help="List PCM devices and exit")
    parser.add_argument("--list-mixer", action="store_true",
                        help="List mixer controls for --card and exit")

    # ---- Analysis ----
    parser.add_argument("--analyze", action="store_true",
                        help="Analyze recording quality after recording (requires numpy)")
    parser.add_argument("--check-speaker-safety", action="store_true",
                        help="Check speaker power safety when setting playback volume")

    # ---- Misc ----
    parser.add_argument("-q", "--quiet", action="store_true",
                        help="Suppress output messages")

    args = parser.parse_args()

    # ---- Listing actions (exit after) ----
    if args.list:
        list_devices()
        return
    if args.list_mixer:
        list_mixer_controls(args.card)
        return

    # ---- Apply mixer volume (only when explicitly requested) ----
    if args.voice_preset:
        AudioMixer(card=args.card).apply_voice_preset()
        if not args.quiet:
            print("[mixer] voice preset: ALC off, mic +18dB (88%), speaker -3dB (ASR-friendly)")

    if args.max_gain:
        desc = AudioMixer(card=args.card).apply_record_max_gain()
        if not args.quiet:
            print(f"[mixer] record max gain: {desc}")

    if args.play_preset:
        db_val = apply_playback_volume(args.card, 100)  # Max: +4.5dB (ES8388 hardware limit)
        if not args.quiet:
            print(f"[mixer] play preset: speaker {db_val:+.1f}dB "
                  f"(100%, ES8388 hardware max, use with --gain 2.0)")

    if args.playback_volume is not None:
        pct = max(0, min(100, args.playback_volume))
        db_val = apply_playback_volume(args.card, pct)
        if not args.quiet:
            print(f"[mixer] playback {pct}%  →  Output {db_val:.1f} dB (PCM 0dB)")
        if args.check_speaker_safety:
            check_speaker_safety(db_val)

    if args.capture_volume is not None:
        pct = max(0, min(100, args.capture_volume))
        preamp_db, digital_db = apply_capture_volume(args.card, pct)
        if not args.quiet:
            if pct == 0:
                print(f"[mixer] capture 0%  →  MIC MUTED")
            else:
                print(f"[mixer] capture {pct}%  →  PGA {preamp_db:.1f}dB, digital {digital_db:.1f}dB")

    # ---- Run selected mode ----
    if args.mode == "play":
        if args.file is None:
            parser.print_help()
            print("\nError: Please specify a WAV file to play.")
            sys.exit(1)
        # Auto-apply hardware max playback volume if no explicit volume was set
        if (args.playback_volume is None and not args.voice_preset
                and not args.play_preset):
            db_val = apply_playback_volume(args.card, 100)
            if not args.quiet:
                print(f"[mixer] play mode default: speaker {db_val:+.1f}dB "
                      f"(100%, ES8388 hardware max) — use --playback-volume to override")
        play_only(args)
    elif args.mode == "record":
        if not args.output:
            args.output = "record.wav"
        record_only(args)
        # Analyze recording if requested
        if args.analyze and os.path.exists(args.output):
            if not args.quiet:
                print("\nAnalyzing recording...")
            try:
                result = analyze_recording(args.output)
                print_recording_analysis(result)
            except ImportError:
                print("Warning: numpy not available, skipping analysis")
            except Exception as e:
                print(f"Warning: analysis failed: {e}")
    elif args.mode == "loopback":
        loopback_mode(args)
        # Analyze recording if requested and saved
        if args.analyze and args.output and os.path.exists(args.output):
            if not args.quiet:
                print("\nAnalyzing recording...")
            try:
                result = analyze_recording(args.output)
                print_recording_analysis(result)
            except ImportError:
                print("Warning: numpy not available, skipping analysis")
            except Exception as e:
                print(f"Warning: analysis failed: {e}")


if __name__ == "__main__":
    main()
