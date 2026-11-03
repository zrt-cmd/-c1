#!/usr/bin/env python3
"""Audio quality analyzer and speaker protection checker.

Provides:
  - Recording quality analysis (peak, RMS, clipping, noise floor)
  - Speaker power safety check (prevents damage to 4Ω 3W speaker)
  - Recommendations for gain adjustment

Usage:
  from audio_analyzer import analyze_recording, check_speaker_safety

  result = analyze_recording('recording.wav')
  if result['clipped']:
      print("Warning: clipping detected, reduce capture volume")

  check_speaker_safety(output_db=-3.0, amp_gain_db=20.0)
"""

import math
import subprocess
import wave

import numpy as np


# Speaker specifications
SPEAKER_IMPEDANCE_OHM = 4.0
SPEAKER_POWER_MAX_W = 3.0

# ES8388 codec output voltage (typical)
CODEC_OUTPUT_VPP_AT_0DB = 3.3

# Board amplifier gain (measured or estimated)
# TODO: Measure actual amp gain on the board
DEFAULT_AMP_GAIN_DB = 20.0


def analyze_recording(wav_path, silence_threshold_db=-40.0):
    """Analyze recording quality metrics.

    Args:
        wav_path: Path to WAV file
        silence_threshold_db: Threshold below which signal is considered silence

    Returns:
        dict with keys:
            peak_db: Peak level in dB FS
            rms_db: RMS level in dB FS
            noise_floor_db: Noise floor estimate (RMS of quietest 10%)
            clipped: True if signal reached near-full-scale (>95%)
            too_quiet: True if peak is too low (<5%)
            high_noise: True if noise floor is elevated (>-50dB in quiet sections)
            recommendation: String suggestion for gain adjustment
    """
    try:
        with wave.open(wav_path, 'rb') as wf:
            if wf.getsampwidth() != 2:
                return {'error': 'Only 16-bit WAV supported'}

            frames = wf.readframes(wf.getnframes())
            audio = np.frombuffer(frames, dtype=np.int16)

            # Convert to float [-1, 1]
            audio_float = audio.astype(np.float32) / 32768.0
    except Exception as e:
        return {'error': f'Failed to read WAV: {e}'}

    if len(audio_float) == 0:
        return {'error': 'Empty audio file'}

    # Basic metrics
    peak = np.max(np.abs(audio_float))
    rms = np.sqrt(np.mean(audio_float**2))

    # Noise floor: RMS of quietest 10% of frames
    frame_size = 2048
    n_frames = len(audio_float) // frame_size
    if n_frames > 10:
        frame_rms = []
        for i in range(n_frames):
            frame = audio_float[i*frame_size:(i+1)*frame_size]
            frame_rms.append(np.sqrt(np.mean(frame**2)))
        frame_rms = np.array(frame_rms)
        noise_floor = np.percentile(frame_rms, 10)
    else:
        noise_floor = rms * 0.1  # Fallback estimate

    # Convert to dB
    peak_db = 20 * math.log10(peak + 1e-9)
    rms_db = 20 * math.log10(rms + 1e-9)
    noise_floor_db = 20 * math.log10(noise_floor + 1e-9)

    # Quality checks
    clipped = peak > 0.95
    too_quiet = peak < 0.05
    high_noise = noise_floor > 10**(-50.0 / 20.0)  # -50dB threshold

    # Recommendation
    recommendation = []
    if clipped:
        recommendation.append("CLIPPING detected — reduce --capture-volume by 20-30%")
    elif peak > 0.7:
        recommendation.append("Signal near full scale — consider reducing capture volume slightly")
    elif too_quiet:
        recommendation.append("Signal too weak — increase --capture-volume by 50-100%")
    elif peak < 0.2:
        recommendation.append("Signal low — consider increasing --capture-volume")
    else:
        recommendation.append("Signal level OK")

    if high_noise:
        recommendation.append("High noise floor — check if ALC is enabled (should be OFF for ASR)")

    return {
        'peak_db': peak_db,
        'rms_db': rms_db,
        'noise_floor_db': noise_floor_db,
        'clipped': clipped,
        'too_quiet': too_quiet,
        'high_noise': high_noise,
        'recommendation': ' | '.join(recommendation),
    }


def check_speaker_safety(output_db, amp_gain_db=DEFAULT_AMP_GAIN_DB,
                        codec_vpp_at_0db=CODEC_OUTPUT_VPP_AT_0DB,
                        speaker_impedance=SPEAKER_IMPEDANCE_OHM,
                        speaker_power_max=SPEAKER_POWER_MAX_W,
                        verbose=True):
    """Check if speaker output level is safe for hardware.

    Args:
        output_db: ES8388 Output 1/2 setting in dB
        amp_gain_db: Board amplifier gain in dB (measure with scope if unknown)
        codec_vpp_at_0db: Codec output voltage peak-to-peak at 0dB
        speaker_impedance: Speaker impedance in Ohms
        speaker_power_max: Speaker max continuous power in Watts
        verbose: Print detailed calculation

    Returns:
        dict with keys:
            safe: True if power is within spec
            power_w: Estimated power output in Watts
            power_pct: Power as percentage of speaker max
            voltage_vpp: Voltage at speaker terminals (Vpp)
            recommendation: Safety advice
    """
    # Calculate voltage at codec output
    codec_vpp = codec_vpp_at_0db * 10**(output_db / 20.0)

    # Calculate voltage after amplifier
    amp_gain_linear = 10**(amp_gain_db / 20.0)
    speaker_vpp = codec_vpp * amp_gain_linear

    # Calculate power (RMS voltage squared / impedance)
    speaker_vrms = speaker_vpp / (2 * math.sqrt(2))  # Vpp to Vrms for sine wave
    power_w = (speaker_vrms ** 2) / speaker_impedance
    power_pct = (power_w / speaker_power_max) * 100

    # Safety check
    safe = power_w <= speaker_power_max

    if verbose:
        print(f"\n{'='*60}")
        print(f"Speaker Safety Check")
        print(f"{'='*60}")
        print(f"Output 1/2 setting:     {output_db:+.1f} dB")
        print(f"Codec output voltage:   {codec_vpp:.2f} Vpp")
        print(f"Amplifier gain:         {amp_gain_db:+.1f} dB ({amp_gain_linear:.1f}x)")
        print(f"Speaker voltage:        {speaker_vpp:.2f} Vpp ({speaker_vrms:.2f} Vrms)")
        print(f"Speaker impedance:      {speaker_impedance:.1f} Ω")
        print(f"Estimated power:        {power_w:.2f} W ({power_pct:.0f}% of {speaker_power_max}W max)")
        print(f"{'='*60}")

    # Recommendation
    if power_w > speaker_power_max * 1.5:
        recommendation = "⛔ UNSAFE: Power exceeds spec by >50% — will damage speaker"
        status = "⛔ UNSAFE"
    elif power_w > speaker_power_max:
        recommendation = "⚠️  CAUTION: Power exceeds spec — may damage speaker with prolonged use"
        status = "⚠️  CAUTION"
    elif power_w > speaker_power_max * 0.8:
        recommendation = "✓ Near maximum safe power — OK for short bursts (alerts), not continuous"
        status = "✓ MARGINAL"
    else:
        recommendation = "✓ Safe for continuous operation"
        status = "✓ SAFE"

    if verbose:
        print(f"Status: {status}")
        print(f"{recommendation}")
        print(f"{'='*60}\n")

    return {
        'safe': safe,
        'power_w': power_w,
        'power_pct': power_pct,
        'voltage_vpp': speaker_vpp,
        'recommendation': recommendation,
    }


def measure_noise_floor(card=0, duration=3, rate=16000, device='hw:0,0'):
    """Record silence and measure noise floor.

    Useful for checking if ALC is pumping up background noise.

    Returns:
        dict with noise_floor_db and recommendation
    """
    import tempfile
    import os

    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
        tmp_path = tmp.name

    try:
        # Record silence
        cmd = [
            'arecord', '-q',
            '-D', device,
            '-f', 'S16_LE',
            '-r', str(rate),
            '-c', '2',
            '-d', str(duration),
            '-t', 'wav',
            tmp_path
        ]
        result = subprocess.run(cmd, capture_output=True, text=True)

        if result.returncode != 0:
            return {'error': f'Recording failed: {result.stderr.strip()}'}

        # Analyze
        result = analyze_recording(tmp_path)

        if 'error' not in result:
            noise_db = result['noise_floor_db']

            recommendation = []
            if noise_db > -45:
                recommendation.append("High noise floor — check if ALC is enabled")
            elif noise_db > -50:
                recommendation.append("Moderate noise floor — acceptable for ASR")
            else:
                recommendation.append("Low noise floor — excellent")

            result['recommendation'] = ' | '.join(recommendation)

        return result

    finally:
        if os.path.exists(tmp_path):
            os.unlink(tmp_path)


def print_recording_analysis(result):
    """Pretty-print recording analysis results."""
    if 'error' in result:
        print(f"❌ Analysis failed: {result['error']}")
        return

    print(f"\n{'='*60}")
    print(f"Recording Quality Analysis")
    print(f"{'='*60}")
    print(f"Peak level:      {result['peak_db']:>6.1f} dB FS")
    print(f"RMS level:       {result['rms_db']:>6.1f} dB FS")
    print(f"Noise floor:     {result['noise_floor_db']:>6.1f} dB FS")

    # Status indicators
    status = []
    if result['clipped']:
        status.append("⛔ CLIPPED")
    if result['too_quiet']:
        status.append("⚠️  TOO QUIET")
    if result['high_noise']:
        status.append("⚠️  HIGH NOISE")
    if not status:
        status.append("✓ OK")

    print(f"Status:          {' '.join(status)}")
    print(f"{'='*60}")
    print(f"Recommendation: {result['recommendation']}")
    print(f"{'='*60}\n")


if __name__ == '__main__':
    import sys

    if len(sys.argv) < 2:
        print("Usage:")
        print("  python3 audio_analyzer.py <recording.wav>           # analyze recording")
        print("  python3 audio_analyzer.py --check-speaker <db>      # check speaker safety")
        print("  python3 audio_analyzer.py --measure-noise           # measure noise floor")
        sys.exit(1)

    if sys.argv[1] == '--check-speaker':
        if len(sys.argv) < 3:
            print("Error: specify output dB level")
            print("Example: python3 audio_analyzer.py --check-speaker -3.0")
            sys.exit(1)
        output_db = float(sys.argv[2])
        check_speaker_safety(output_db)

    elif sys.argv[1] == '--measure-noise':
        print("Recording 3 seconds of silence...")
        result = measure_noise_floor()
        print_recording_analysis(result)

    else:
        wav_path = sys.argv[1]
        result = analyze_recording(wav_path)
        print_recording_analysis(result)
