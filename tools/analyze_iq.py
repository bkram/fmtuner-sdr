#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import numpy as np


def db(x: float) -> float:
    return 10.0 * math.log10(max(x, 1e-20))


def read_iq_u8(path: Path, max_samples: int, offset_bytes: int) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.uint8, offset=max(0, offset_bytes))
    if raw.size < 2:
        return np.empty(0, dtype=np.complex64)
    if (raw.size & 1) != 0:
        raw = raw[:-1]
    if max_samples > 0 and (raw.size // 2) > max_samples:
        raw = raw[: max_samples * 2]
    i = (raw[0::2].astype(np.float32) - 127.0) / 127.5
    q = (raw[1::2].astype(np.float32) - 127.0) / 127.5
    return i + 1j * q


def analyze(iq: np.ndarray, sample_rate: int) -> dict:
    if iq.size < 4096:
        raise ValueError("Need at least 4096 IQ samples for analysis")

    i = iq.real
    q = iq.imag
    p = np.abs(iq) ** 2
    mean_p = float(np.mean(p))

    hard_clip = float(np.mean((np.abs(i) >= 0.995) | (np.abs(q) >= 0.995)))
    near_clip = float(np.mean((np.abs(i) >= 0.93) | (np.abs(q) >= 0.93)))

    rms_i = float(np.sqrt(np.mean(i * i)))
    rms_q = float(np.sqrt(np.mean(q * q)))
    gain_imbalance_db = 20.0 * math.log10(max(rms_i, 1e-12) / max(rms_q, 1e-12))

    # Image-rejection proxy via pseudo-covariance.
    z = iq.astype(np.complex64)
    r0 = np.mean(np.abs(z) ** 2)
    p0 = np.mean(z * z)
    rho = min(float(np.abs(p0) / max(r0, 1e-12)), 0.999999)
    irr_db = 10.0 * math.log10((1.0 + rho) / (1.0 - rho))

    nfft = 1 << 16
    if iq.size < nfft:
        nfft = 1 << int(math.floor(math.log2(iq.size)))
    start = (iq.size - nfft) // 2
    seg = iq[start : start + nfft]
    win = np.hanning(nfft).astype(np.float32)
    spec = np.fft.fftshift(np.fft.fft(seg * win))
    psd = np.abs(spec) ** 2
    freqs = np.fft.fftshift(np.fft.fftfreq(nfft, d=1.0 / sample_rate))

    # 99% occupied bandwidth around center.
    cdf = np.cumsum(psd) / np.sum(psd)
    lo_idx = int(np.searchsorted(cdf, 0.005))
    hi_idx = int(np.searchsorted(cdf, 0.995))
    occ_bw_hz = float(abs(freqs[hi_idx] - freqs[lo_idx]))

    # FM discriminator rough audio-chain proxy.
    dphi = np.angle(z[1:] * np.conj(z[:-1]))
    dphi_rms = float(np.sqrt(np.mean(dphi * dphi)))
    dphi_peak = float(np.percentile(np.abs(dphi), 99.9))

    report = {
        "samples": int(iq.size),
        "sample_rate": int(sample_rate),
        "power_dbfs": db(mean_p),
        "dc_i": float(np.mean(i)),
        "dc_q": float(np.mean(q)),
        "rms_i": rms_i,
        "rms_q": rms_q,
        "gain_imbalance_db": gain_imbalance_db,
        "hard_clip_ratio": hard_clip,
        "near_clip_ratio": near_clip,
        "image_rejection_proxy_db": irr_db,
        "occupied_bw_hz_99pct": occ_bw_hz,
        "fm_dphi_rms": dphi_rms,
        "fm_dphi_p999": dphi_peak,
    }

    hints = []
    if near_clip > 0.01 or hard_clip > 0.001:
        hints.append("Front-end may be overdriven: reduce RTL gain / disable aggressive auto-gain.")
    if abs(report["dc_i"]) > 0.01 or abs(report["dc_q"]) > 0.01:
        hints.append("Noticeable DC offset: verify IQ DC-blocking path and tuner bias conditions.")
    if abs(gain_imbalance_db) > 0.75:
        hints.append("I/Q gain imbalance is non-trivial: consider I/Q correction stage.")
    if occ_bw_hz > 220000:
        hints.append("Wide occupied bandwidth: tighten channel BW/filter to reduce adjacent-channel noise.")
    if not hints:
        hints.append("IQ looks reasonable. Focus next on audio post chain (blend/deemphasis/mute behavior).")
    report["hints"] = hints
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze raw interleaved u8 IQ capture from fm-tuner-sdr.")
    parser.add_argument("iq_file", type=Path, help="Path to raw IQ file captured via -i/--iq")
    parser.add_argument("--sample-rate", type=int, default=256000, help="IQ sample rate in Hz (default: 256000)")
    parser.add_argument("--max-samples", type=int, default=2_000_000, help="Max complex samples to analyze")
    parser.add_argument("--offset-bytes", type=int, default=0, help="Skip initial bytes in file")
    parser.add_argument("--json", action="store_true", help="Print JSON report")
    args = parser.parse_args()

    iq = read_iq_u8(args.iq_file, args.max_samples, args.offset_bytes)
    if iq.size == 0:
        print("No IQ data read")
        return 1

    report = analyze(iq, args.sample_rate)
    if args.json:
        print(json.dumps(report, indent=2))
        return 0

    print(f"IQ file: {args.iq_file}")
    print(f"Samples: {report['samples']} @ {report['sample_rate']} Hz")
    print(f"Power: {report['power_dbfs']:.2f} dBFS")
    print(f"DC offset: I={report['dc_i']:.5f}, Q={report['dc_q']:.5f}")
    print(f"RMS: I={report['rms_i']:.5f}, Q={report['rms_q']:.5f}")
    print(f"I/Q gain imbalance: {report['gain_imbalance_db']:.2f} dB")
    print(f"Hard clip ratio: {report['hard_clip_ratio']:.5f}")
    print(f"Near clip ratio: {report['near_clip_ratio']:.5f}")
    print(f"Image-rejection proxy: {report['image_rejection_proxy_db']:.2f} dB")
    print(f"Occupied BW (99%): {report['occupied_bw_hz_99pct']:.0f} Hz")
    print(f"FM dphi RMS: {report['fm_dphi_rms']:.5f}")
    print(f"FM dphi p99.9: {report['fm_dphi_p999']:.5f}")
    print("Hints:")
    for hint in report["hints"]:
        print(f"- {hint}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

