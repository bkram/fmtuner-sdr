#!/usr/bin/env python3
import argparse
import json
import math
import wave
from pathlib import Path

import numpy as np


def analyze_wav(path: Path) -> dict:
    with wave.open(str(path), "rb") as wf:
        channels = wf.getnchannels()
        sample_width = wf.getsampwidth()
        sample_rate = wf.getframerate()
        nframes = wf.getnframes()
        pcm = wf.readframes(nframes)

    if sample_width != 2:
        raise ValueError("Only 16-bit PCM WAV is supported")
    if channels not in (1, 2):
        raise ValueError("Only mono/stereo WAV is supported")

    data = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
    if channels == 2:
        data = data.reshape(-1, 2)
        left = data[:, 0]
        right = data[:, 1]
        mono = 0.5 * (left + right)
    else:
        left = right = mono = data

    peak = float(np.max(np.abs(data))) if data.size else 0.0
    clip_ratio = float(np.mean(np.abs(data) >= 0.999)) if data.size else 0.0
    near_clip_ratio = float(np.mean(np.abs(data) >= 0.95)) if data.size else 0.0
    dc_l = float(np.mean(left)) if left.size else 0.0
    dc_r = float(np.mean(right)) if right.size else 0.0
    rms_l = float(np.sqrt(np.mean(left * left))) if left.size else 0.0
    rms_r = float(np.sqrt(np.mean(right * right))) if right.size else 0.0
    rms_m = float(np.sqrt(np.mean(mono * mono))) if mono.size else 0.0
    crest = (peak / max(rms_m, 1e-12)) if mono.size else 0.0

    # Simple spectral brightness proxy: energy above 8 kHz vs below 8 kHz.
    n = min(len(mono), 1 << 17)
    if n >= 4096:
        x = mono[:n]
        win = np.hanning(n).astype(np.float32)
        spec = np.fft.rfft(x * win)
        psd = np.abs(spec) ** 2
        freqs = np.fft.rfftfreq(n, d=1.0 / sample_rate)
        lo = float(np.sum(psd[freqs < 8000.0]))
        hi = float(np.sum(psd[freqs >= 8000.0]))
        bright_db = 10.0 * math.log10(max(hi, 1e-20) / max(lo, 1e-20))
    else:
        bright_db = 0.0

    report = {
        "frames": int(nframes),
        "channels": int(channels),
        "sample_rate": int(sample_rate),
        "peak": peak,
        "clip_ratio": clip_ratio,
        "near_clip_ratio": near_clip_ratio,
        "dc_left": dc_l,
        "dc_right": dc_r,
        "rms_left": rms_l,
        "rms_right": rms_r,
        "crest_factor": crest,
        "brightness_db_hi_vs_lo": bright_db,
    }

    hints = []
    if clip_ratio > 0.001 or near_clip_ratio > 0.02:
        hints.append("Audio path clipping likely: reduce gain/volume or soften limiter behavior.")
    if abs(dc_l) > 0.01 or abs(dc_r) > 0.01:
        hints.append("Noticeable audio DC offset: review post DC blocker and reset paths.")
    if bright_db < -18.0:
        hints.append("Audio may be dull/muffled: check deemphasis and blend aggressiveness.")
    if bright_db > -6.0:
        hints.append("Audio may be too bright/noisy: check IF bandwidth and weak-signal blend tuning.")
    if not hints:
        hints.append("WAV metrics look reasonable for FM broadcast.")
    report["hints"] = hints
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze 16-bit PCM WAV output from fm-tuner-sdr.")
    parser.add_argument("wav_file", type=Path, help="Path to WAV file")
    parser.add_argument("--json", action="store_true", help="Print JSON report")
    args = parser.parse_args()

    report = analyze_wav(args.wav_file)
    if args.json:
        print(json.dumps(report, indent=2))
        return 0

    print(f"WAV file: {args.wav_file}")
    print(f"Frames: {report['frames']} @ {report['sample_rate']} Hz, channels={report['channels']}")
    print(f"Peak: {report['peak']:.4f}")
    print(f"Clip ratio: {report['clip_ratio']:.5f}")
    print(f"Near-clip ratio: {report['near_clip_ratio']:.5f}")
    print(f"DC: L={report['dc_left']:.5f}, R={report['dc_right']:.5f}")
    print(f"RMS: L={report['rms_left']:.5f}, R={report['rms_right']:.5f}")
    print(f"Crest factor: {report['crest_factor']:.2f}")
    print(f"Brightness (>=8kHz vs <8kHz): {report['brightness_db_hi_vs_lo']:.2f} dB")
    print("Hints:")
    for hint in report["hints"]:
        print(f"- {hint}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

