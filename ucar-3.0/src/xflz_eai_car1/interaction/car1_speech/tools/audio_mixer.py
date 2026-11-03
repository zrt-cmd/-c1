#!/usr/bin/env python3
"""ES8388 mixer helper for RK3576 board.

Signal chain:
  Mic (MIC2_P/N) → PGA (Left/Right Channel, 0~+24dB) → ADC → Capture Digital
  PCM (0dB fixed) → DAC → Output 1/2 (-45~+4.5dB) → NS4150B amp → 4Ω 3W speaker

Hardware notes (verified by user testing):
  - ES8388 Output 1/2 range: -30dB ~ +4.5dB (hardware limit)
  - NS4150B amplifier: BTL D-class, ~12dB gain (Rin=20K, typical Rf=40K)
  - Playback max unclipped: Output=+4.5dB + digital gain 2.0x (user confirmed)
  - Higher digital gain (>2.5x) causes audible clipping at Output=+4.5dB

Recommended defaults (based on user testing):
  --voice-preset:  mic +18dB, speaker -3dB (medium volume, ASR-friendly)
  --play-preset:   speaker +4.5dB (hardware max, use with --gain 2.0)

Typical use:
    from audio_mixer import AudioMixer
    mixer = AudioMixer(card=0)
    mixer.apply_voice_preset()  # ASR defaults
"""

import math
import subprocess


MIC_PREAMP_MIN_DB = 0.0
MIC_PREAMP_MAX_DB = 24.0

CAPTURE_DIGITAL_MIN_DB = -30.0
CAPTURE_DIGITAL_MAX_DB = 6.0  # ES8388 digital can boost up to +6dB if needed

PCM_MIN_DB = -60.0
PCM_MAX_DB = 0.0

SPEAKER_OUTPUT_MIN_DB = -30.0
SPEAKER_OUTPUT_MAX_DB = 4.5   # ES8388 hardware max, NS4150B @ 6dB gain still safe for 4Ω 3W
SPEAKER_OUTPUT_ALERT_MAX_DB = 4.5

MIC_GAIN_PRESETS = {
    'low': 12.0,    # 75% capture-volume
    'normal': 18.0, # +18dB PGA (88% capture-volume), default for board mic at 30-50cm
    'high': 24.0,   # Max PGA gain (100% capture-volume)
}

SPEAKER_LEVEL_PRESETS = {
    'quiet': -12.0,    # 试听时"小"
    'normal': -3.0,    # 中等音量
    'loud': 0.0,       # 用户实测反馈"还不错"
    'alert': 4.5,      # 用户实测"明显更响，无破音"，ES8388 硬件极限
}


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _run_amixer(card, args):
    subprocess.run(["amixer", "-c", str(card)] + args,
                   capture_output=True, check=False)


def _set_control_db(card, control, db, lo, hi):
    applied = _clamp(db, lo, hi)
    _run_amixer(card, ["sset", control, "--", f"{applied:.1f}dB"])
    return applied


class AudioMixer:
    """Safe gain control for ES8388 codec."""

    def __init__(self, card=0):
        self.card = card

    def set_mic_preamp_db(self, db):
        applied = _clamp(db, MIC_PREAMP_MIN_DB, MIC_PREAMP_MAX_DB)
        _run_amixer(self.card, ["sset", "Left Channel", "--", f"{applied:.1f}dB"])
        _run_amixer(self.card, ["sset", "Right Channel", "--", f"{applied:.1f}dB"])
        return applied

    def set_capture_digital_db(self, db):
        return _set_control_db(self.card, "Capture Digital", db,
                               CAPTURE_DIGITAL_MIN_DB, CAPTURE_DIGITAL_MAX_DB)

    def set_pcm_db(self, db):
        return _set_control_db(self.card, "PCM", db, PCM_MIN_DB, PCM_MAX_DB)

    def set_speaker_output_db(self, db, allow_alert=False):
        hi = SPEAKER_OUTPUT_ALERT_MAX_DB if allow_alert else SPEAKER_OUTPUT_MAX_DB
        applied = _clamp(db, SPEAKER_OUTPUT_MIN_DB, hi)
        _run_amixer(self.card, ["sset", "Output 1", "--", f"{applied:.1f}dB"])
        _run_amixer(self.card, ["sset", "Output 2", "--", f"{applied:.1f}dB"])
        return applied

    def set_mic_gain(self, level):
        if level not in MIC_GAIN_PRESETS:
            raise ValueError(
                f"unknown mic gain {level!r}, expected {list(MIC_GAIN_PRESETS)}")
        db = MIC_GAIN_PRESETS[level]
        self.set_mic_preamp_db(db)
        self.set_capture_digital_db(0.0)
        return db

    def set_speaker_volume(self, level):
        if level not in SPEAKER_LEVEL_PRESETS:
            raise ValueError(
                f"unknown speaker level {level!r}, expected {list(SPEAKER_LEVEL_PRESETS)}")
        db = SPEAKER_LEVEL_PRESETS[level]
        self.set_pcm_db(0.0)
        self.set_speaker_output_db(db, allow_alert=(level == 'alert'))
        return db

    def apply_voice_preset(self):
        """Recommended startup gains for ASR: MIC2 differential + 4Ω 3W speaker.

        - Mic: +18dB PGA (equivalent to --capture-volume 88, good for 30-50cm)
        - Speaker: -3dB Output (中等音量, ASR 使用避免过响串扰麦克风)
        - ALC: OFF (prevents noise floor pumping during silence)
        """
        self.set_alc(False)
        self.set_mic_gain('normal')
        self.set_speaker_volume('normal')

    def apply_record_max_gain(self):
        """录音最大增益：关 ALC + 麦克风 PGA 拉到最高 + 数字增益打到最大。

        用于提升较弱麦克风信号的录音音量。关闭 ALC 以手动固定增益，避免
        自动增益压音。每次录音调用，重启后由下次运行重设（不持久化）。
        """
        self.set_alc(False)                       # 关 ALC，固定增益
        self.set_mic_gain('high')                 # PGA +24dB（最大模拟增益，最佳 SNR）
        self.set_capture_digital_db(CAPTURE_DIGITAL_MAX_DB)  # 数字 +6dB 再补一档
        return 'mic PGA +24dB, digital +6dB, ALC off'

    def mute_speaker(self):
        _run_amixer(self.card, ["sset", "Output 1", "mute"])
        _run_amixer(self.card, ["sset", "Output 2", "mute"])

    def unmute_speaker(self):
        _run_amixer(self.card, ["sset", "Output 1", "unmute"])
        _run_amixer(self.card, ["sset", "Output 2", "unmute"])

    def mute_mic(self):
        _run_amixer(self.card, ["sset", "Left Channel", "mute"])
        _run_amixer(self.card, ["sset", "Right Channel", "mute"])

    def unmute_mic(self):
        _run_amixer(self.card, ["sset", "Left Channel", "unmute"])
        _run_amixer(self.card, ["sset", "Right Channel", "unmute"])

    def set_alc(self, enabled, target_pct=None, max_pga_step=None, noise_gate=None):
        """Configure the ES8388 Automatic Level Control block.

        ALC on: codec auto-adjusts PGA to hit target level. Great for varying
        distance, but pumps up the noise floor during silence.
        ALC off: PGA stays where set_mic_preamp_db put it — predictable SNR.

        target_pct   0-100  ALC target level (only used when enabled).
        max_pga_step 0-7    Upper bound for ALC-driven PGA moves.
        noise_gate   bool   Mute ADC output when signal falls below threshold.
        """
        _run_amixer(self.card, [
            "sset", "ALC Capture Function", "Stereo" if enabled else "Off"])
        if enabled and target_pct is not None:
            step = int(round(_clamp(target_pct, 0, 100) / 100.0 * 15))
            _run_amixer(self.card, ["sset", "ALC Capture Target", str(step)])
        if enabled and max_pga_step is not None:
            step = int(_clamp(max_pga_step, 0, 7))
            _run_amixer(self.card, ["sset", "ALC Capture Max PGA", str(step)])
        if noise_gate is not None:
            _run_amixer(self.card, [
                "sset", "ALC Capture NG", "on" if noise_gate else "off"])


def capture_volume_pct_to_db(pct):
    """0-100% → (preamp_db, digital_db).

    50% = neutral (PGA 0dB, digital 0dB)
    above 50%: PGA boost 0..+24dB
    below 50%: digital cut 0..-30dB (hurts SNR, avoid)
    """
    pct = _clamp(pct, 0, 100)
    if pct >= 50:
        return (pct - 50) / 50.0 * MIC_PREAMP_MAX_DB, 0.0
    return 0.0, (pct - 50) / 50.0 * abs(CAPTURE_DIGITAL_MIN_DB)


def playback_volume_pct_to_db(pct):
    """0-100% → Output dB, capped at -12dB. 100%→-12dB, 67%→-15dB, 50%→-18dB."""
    pct = _clamp(pct, 0, 100)
    if pct <= 0:
        return SPEAKER_OUTPUT_MIN_DB
    return _clamp(SPEAKER_OUTPUT_MAX_DB + 20.0 * math.log10(pct / 100.0),
                  SPEAKER_OUTPUT_MIN_DB, SPEAKER_OUTPUT_MAX_DB)
