#!/usr/bin/env python3
"""
Audio utility helpers — mixer setup, ALSA device enumeration, wav playback.

Extracted from car1_speech/tools/*.py so the voice interaction system and
existing test scripts share a single implementation.
"""

import math
import os
import subprocess
import tempfile
import wave


# ---------------------------------------------------------------------------
# Mixer helpers (ES8388 / RK3576)
# ---------------------------------------------------------------------------

def pct_to_db(pct):
    """Convert volume percentage to dB: 100% → 0dB, 50% → -6dB."""
    if pct >= 100:
        return 0.0
    if pct <= 0:
        return -96.0
    return 20.0 * math.log10(pct / 100.0)


def apply_capture_volume(card, pct):
    """Set microphone capture volume.

    Capture path: Left/Right Channel (preamp, 0..+24dB) + Capture Digital (-96..0dB).
    Preamp stays at max (+24dB) for best SNR; Capture Digital handles attenuation.
    """
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "Capture Mute", "on"],
        capture_output=True, check=False)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "Left Channel", "--", "24.0dB"],
        capture_output=True, check=False)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "Right Channel", "--", "24.0dB"],
        capture_output=True, check=False)
    db_val = pct_to_db(pct)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "Capture Digital", "--", f"{db_val:.1f}dB"],
        capture_output=True, check=False)


def enable_alc(card):
    """Enable ALC (Automatic Level Control) for better low-volume pickup."""
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "ALC Capture Function", "Stereo"],
        capture_output=True, check=False)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "ALC Capture Target", "12"],
        capture_output=True, check=False)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "ALC Capture Max PGA", "7"],
        capture_output=True, check=False)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "ALC Capture Attack Time", "5"],
        capture_output=True, check=False)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "ALC Capture Decay Time", "0"],
        capture_output=True, check=False)


def switch_input_to_mic(card):
    """Switch ES8388 input mux from Line to Mic inputs."""
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "Left Line Mux", "MicL"],
        capture_output=True, check=False)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "Right Line Mux", "MicR"],
        capture_output=True, check=False)


def disable_alc(card):
    """Disable ALC (Automatic Level Control) for stable fixed-gain capture.

    ALC auto-adjusts gain and causes noise floor pumping during silence,
    which hurts KWS/ASR accuracy. kws_test.py explicitly disables ALC.
    """
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "ALC Capture Function", "Off"],
        capture_output=True, check=False)


def setup_speaker(card):
    """Ensure speaker amplifier is enabled for ES8388 + NS4150B.

    spk switch defaults to off on some boots, muting all playback.
    Call once at node startup so prompt/TTS are audible without manual amixer.
    """
    for ctl in ["Speaker", "spk switch"]:
        subprocess.run(
            ["amixer", "-c", str(card), "sset", ctl, "on"],
            capture_output=True, check=False)


def setup_capture(card, volume_pct, mic_boost=True):
    """Full capture setup: volume + optional ALC/Mic switching."""
    pct = max(0, min(100, volume_pct))
    apply_capture_volume(card, pct)
    if mic_boost:
        enable_alc(card)
        switch_input_to_mic(card)


def setup_playback(card, volume_pct):
    """Set playback mixer levels and enable speaker amplifier.

    Follows audio_test.py convention:
      - PCM held at 0dB (preserve dynamic range)
      - Output 1/2 mapped to -30..-12dB (capped at -12dB for 4Ω 3W speaker safety)
      - 100% → -12dB, 50% → -21dB, 0% → mute
    """
    pct = max(0, min(100, volume_pct))

    # Set PCM to 0dB (preserve dynamic range)
    subprocess.run(
        ["amixer", "-c", str(card), "sset", "PCM", "--", "0dB"],
        capture_output=True, check=False)

    # Map 0-100% → -30..-12dB (speaker safety cap at -12dB)
    if pct <= 0:
        # Mute speaker
        for ctl in ["Speaker", "spk switch"]:
            subprocess.run(
                ["amixer", "-c", str(card), "sset", ctl, "off"],
                capture_output=True, check=False)
        return

    db_val = -30.0 + (pct / 100.0) * 18.0  # 0% → -30dB, 100% → -12dB
    for ctl in ["Output 1", "Output 2"]:
        subprocess.run(
            ["amixer", "-c", str(card), "sset", ctl, "--", f"{db_val:.1f}dB"],
            capture_output=True, check=False)

    for ctl in ["Speaker", "spk switch"]:
        subprocess.run(
            ["amixer", "-c", str(card), "sset", ctl, "on"],
            capture_output=True, check=False)


# ---------------------------------------------------------------------------
# WAV playback
# ---------------------------------------------------------------------------

def _ensure_stereo(path):
    """Return a stereo wav path suitable for ES8388 I2S (hard-wired stereo).

    If the wav is already stereo, returns the original path.
    If mono, converts to stereo (duplicate L→R) via a temp file.
    """
    with wave.open(path, "rb") as wf:
        if wf.getnchannels() == 2:
            return path
        nchannels = wf.getnchannels()
        sampwidth = wf.getsampwidth()
        framerate = wf.getframerate()
        raw = wf.readframes(wf.getnframes())

    if nchannels != 1:
        return path

    import numpy as np
    dtype = {1: np.int8, 2: np.int16, 4: np.int32}[sampwidth]
    mono = np.frombuffer(raw, dtype=dtype)
    stereo = np.column_stack([mono, mono]).ravel()

    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False, dir="/tmp")
    with wave.open(tmp.name, "wb") as wf:
        wf.setnchannels(2)
        wf.setsampwidth(sampwidth)
        wf.setframerate(framerate)
        wf.writeframes(stereo.tobytes())
    return tmp.name


def play_wav(path, device="plughw:0,0"):
    """Play a wav file via aplay (non-blocking, returns subprocess.Popen).

    Auto-converts mono→stereo for ES8388 I2S compatibility.
    """
    card = 0
    for ctl in ["Speaker", "spk switch"]:
        subprocess.run(
            ["amixer", "-c", str(card), "sset", ctl, "on"],
            capture_output=True, check=False)
    stereo_path = _ensure_stereo(path)
    return subprocess.Popen(
        ["aplay", "-q", "-D", device, stereo_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def play_wav_sync(path, device="plughw:0,0"):
    """Play a wav file via aplay (blocking).

    Auto-converts mono→stereo for ES8388 I2S compatibility.
    """
    card = 0
    for ctl in ["Speaker", "spk switch"]:
        subprocess.run(
            ["amixer", "-c", str(card), "sset", ctl, "on"],
            capture_output=True, check=False)
    stereo_path = _ensure_stereo(path)
    subprocess.run(
        ["aplay", "-q", "-D", device, stereo_path],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
