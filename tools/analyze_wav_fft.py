#!/usr/bin/env python3
import argparse
import json
import math
import wave
from pathlib import Path
from typing import List, Tuple

import numpy as np


def load_wav(path: Path) -> Tuple[np.ndarray, int]:
    with wave.open(str(path), "rb") as wf:
        ch = wf.getnchannels()
        sw = wf.getsampwidth()
        sr = wf.getframerate()
        n = wf.getnframes()
        pcm = wf.readframes(n)
    if sw != 2:
        raise ValueError(f"{path}: expected 16-bit PCM")
    x = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0
    if ch == 2:
        x = x.reshape(-1, 2).mean(axis=1)
    elif ch != 1:
        raise ValueError(f"{path}: expected mono/stereo")
    return x, sr


def framesig(x: np.ndarray, nfft: int, hop: int) -> np.ndarray:
    if x.size < nfft:
        return np.empty((0, nfft), dtype=np.float32)
    nfrm = 1 + (x.size - nfft) // hop
    idx = np.arange(nfft)[None, :] + hop * np.arange(nfrm)[:, None]
    return x[idx]


def mad(v: np.ndarray) -> float:
    if v.size == 0:
        return 0.0
    m = np.median(v)
    return float(np.median(np.abs(v - m))) + 1e-12


def group_runs(mask: np.ndarray) -> List[Tuple[int, int]]:
    runs: List[Tuple[int, int]] = []
    i = 0
    n = mask.size
    while i < n:
        if not mask[i]:
            i += 1
            continue
        j = i + 1
        while j < n and mask[j]:
            j += 1
        runs.append((i, j))
        i = j
    return runs


def analyze(path: Path, nfft: int, hop: int) -> dict:
    x, sr = load_wav(path)
    if x.size < max(4096, nfft):
        raise ValueError(f"{path}: too short for FFT analysis")

    fr = framesig(x, nfft, hop)
    win = np.hanning(nfft).astype(np.float32)
    spec = np.fft.rfft(fr * win[None, :], axis=1)
    p = np.abs(spec) ** 2 + 1e-20
    freqs = np.fft.rfftfreq(nfft, 1.0 / sr)

    rms = np.sqrt(np.mean(fr * fr, axis=1) + 1e-20)
    d = np.diff(x, prepend=x[:1])
    dfr = framesig(d, nfft, hop)
    drms = np.sqrt(np.mean(dfr * dfr, axis=1) + 1e-20)
    centroid = np.sum(p * freqs[None, :], axis=1) / np.sum(p, axis=1)
    flatness = np.exp(np.mean(np.log(p), axis=1)) / np.mean(p, axis=1)
    hi = np.sum(p[:, freqs >= 8000.0], axis=1)
    lo = np.sum(p[:, freqs < 8000.0], axis=1)
    hi_ratio = hi / np.maximum(hi + lo, 1e-20)
    flux = np.sum(np.maximum(np.diff(np.abs(spec), axis=0), 0.0), axis=1)
    flux = np.concatenate([[0.0], flux])

    med_rms = float(np.median(rms))
    med_drms = float(np.median(drms))
    med_flat = float(np.median(flatness))
    med_flux = float(np.median(flux))

    dropout_mask = rms < max(1e-6, med_rms * 0.20)
    click_mask = drms > (med_drms + 8.0 * mad(drms))
    spike_mask = (flatness > (med_flat + 6.0 * mad(flatness))) & (flux > (med_flux + 6.0 * mad(flux)))

    dropout_runs = [(a, b) for (a, b) in group_runs(dropout_mask) if (b - a) >= 2]
    click_runs = group_runs(click_mask)
    spike_runs = group_runs(spike_mask)

    t_hop = hop / sr
    def runs_to_events(runs: List[Tuple[int, int]], limit: int = 8):
        ev = []
        for a, b in runs[:limit]:
            ev.append({
                "start_s": round(a * t_hop, 4),
                "end_s": round(b * t_hop, 4),
                "dur_ms": round((b - a) * t_hop * 1000.0, 2),
            })
        return ev

    # Tremolo proxy: frame-RMS modulation peak in 1..30 Hz.
    mod = rms - np.mean(rms)
    fs_mod = 1.0 / t_hop
    if mod.size >= 256:
        nmod = 1 << int(math.floor(math.log2(mod.size)))
        sm = np.fft.rfft(mod[:nmod] * np.hanning(nmod))
        fm = np.fft.rfftfreq(nmod, d=1.0 / fs_mod)
        band = (fm >= 1.0) & (fm <= 30.0)
        if np.any(band):
            mag = np.abs(sm[band])
            idx = int(np.argmax(mag))
            trem_freq = float(fm[band][idx])
            trem_db = 20.0 * math.log10(float(mag[idx]) / (np.mean(np.abs(sm)) + 1e-12))
        else:
            trem_freq = 0.0
            trem_db = -120.0
    else:
        trem_freq = 0.0
        trem_db = -120.0

    report = {
        "file": str(path),
        "duration_s": round(float(x.size / sr), 3),
        "sample_rate": int(sr),
        "nfft": int(nfft),
        "hop": int(hop),
        "rms_median": float(med_rms),
        "drms_median": float(med_drms),
        "centroid_median_hz": float(np.median(centroid)),
        "hi_ratio_median": float(np.median(hi_ratio)),
        "dropout_events": len(dropout_runs),
        "click_events": len(click_runs),
        "spike_events": len(spike_runs),
        "dropout_event_samples": runs_to_events(dropout_runs),
        "click_event_samples": runs_to_events(click_runs),
        "spike_event_samples": runs_to_events(spike_runs),
        "tremolo_peak_hz": round(trem_freq, 3),
        "tremolo_prominence_db": round(trem_db, 2),
    }

    hints = []
    if report["dropout_events"] > 0:
        hints.append("Potential dropouts detected (low-RMS runs).")
    if report["click_events"] > 30:
        hints.append("Frequent micro-click signatures detected (high derivative bursts).")
    if report["spike_events"] > 10:
        hints.append("Repeated broadband spectral spikes detected.")
    if report["tremolo_prominence_db"] > 18.0:
        hints.append("Strong low-rate RMS modulation (tremolo-like) detected.")
    if not hints:
        hints.append("No strong microstutter/click signature detected in FFT metrics.")
    report["hints"] = hints
    return report


def main() -> int:
    ap = argparse.ArgumentParser(description="Deep FFT/stutter analysis for fm-tuner-sdr WAV captures.")
    ap.add_argument("wav_files", nargs="+", type=Path)
    ap.add_argument("--nfft", type=int, default=2048)
    ap.add_argument("--hop", type=int, default=256)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args()

    reports = [analyze(p, args.nfft, args.hop) for p in args.wav_files]
    if args.json:
        print(json.dumps(reports, indent=2))
        return 0

    for r in reports:
        print(f"WAV: {r['file']}")
        print(f"  duration={r['duration_s']}s sr={r['sample_rate']} nfft={r['nfft']} hop={r['hop']}")
        print(f"  centroid_med={r['centroid_median_hz']:.1f}Hz hi_ratio_med={r['hi_ratio_median']:.4f}")
        print(f"  dropouts={r['dropout_events']} clicks={r['click_events']} spikes={r['spike_events']}")
        print(f"  tremolo_peak={r['tremolo_peak_hz']:.3f}Hz prominence={r['tremolo_prominence_db']:.2f}dB")
        print("  hints:")
        for h in r["hints"]:
            print(f"  - {h}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

