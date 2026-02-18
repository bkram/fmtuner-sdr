#!/usr/bin/env python3
import argparse
import bisect
import json
import signal
import statistics
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional


TOOLS_DIR = Path(__file__).resolve().parent
ROOT_DIR = TOOLS_DIR.parent

import sys

sys.path.insert(0, str(TOOLS_DIR))
from analyze_iq import analyze, read_iq_u8  # noqa: E402


@dataclass
class CaptureResult:
    freq_khz: int
    gain_db: int
    ok: bool
    error: str
    metrics: Optional[dict]
    score: float
    valid: bool


def write_temp_config_with_xdr_port(src: Path, dst: Path, xdr_port: int) -> None:
    text = src.read_text(encoding="utf-8")
    lines = text.splitlines()
    out: List[str] = []
    in_xdr = False
    xdr_found = False
    port_written = False

    for line in lines:
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            if in_xdr and not port_written:
                out.append(f"port = {xdr_port}")
                port_written = True
            sec = stripped[1:-1].strip().lower()
            in_xdr = sec == "xdr"
            if in_xdr:
                xdr_found = True
            out.append(line)
            continue
        if in_xdr and stripped.lower().startswith("port"):
            out.append(f"port = {xdr_port}")
            port_written = True
            continue
        out.append(line)

    if in_xdr and not port_written:
        out.append(f"port = {xdr_port}")
        port_written = True

    if not xdr_found:
        out.append("")
        out.append("[xdr]")
        out.append(f"port = {xdr_port}")
        out.append("guest_mode = true")

    dst.write_text("\n".join(out) + "\n", encoding="utf-8")


def parse_csv_ints(raw: str) -> List[int]:
    out = []
    for token in raw.split(","):
        token = token.strip()
        if not token:
            continue
        out.append(int(token))
    return out


def build_freq_list(
    freqs_csv: str,
    full_band: bool,
    freq_start_khz: int,
    freq_end_khz: int,
    freq_step_khz: int,
) -> List[int]:
    if freqs_csv.strip():
        return parse_csv_ints(freqs_csv)
    if not full_band:
        return []
    if freq_step_khz <= 0:
        raise ValueError("freq_step_khz must be > 0")
    out = []
    f = freq_start_khz
    while f <= freq_end_khz:
        out.append(f)
        f += freq_step_khz
    return out


def score_metrics(metrics: dict, hard_max: float, near_max: float) -> (float, bool):
    power = float(metrics["power_dbfs"])
    hard = float(metrics["hard_clip_ratio"])
    near = float(metrics["near_clip_ratio"])
    dc = max(abs(float(metrics["dc_i"])), abs(float(metrics["dc_q"])))

    score = power
    score -= hard * 8000.0
    score -= near * 1200.0
    if dc > 0.01:
        score -= (dc - 0.01) * 60.0
    valid = hard <= hard_max and near <= near_max
    return score, valid


def run_capture(
    binary: Path,
    config: Path,
    freq_khz: int,
    gain_db: int,
    seconds: int,
    iq_file: Path,
    log_file: Path,
) -> bool:
    cmd = [
        str(binary),
        "-c",
        str(config),
        "-f",
        str(freq_khz),
        "-g",
        str(gain_db),
        "-i",
        str(iq_file),
    ]
    with open(log_file, "w", encoding="utf-8") as lf:
        proc = subprocess.Popen(cmd, stdout=lf, stderr=lf)
        time.sleep(seconds)
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=2.0)
    return iq_file.exists() and iq_file.stat().st_size > 0


def analyze_capture(iq_file: Path, sample_rate: int, max_samples: int) -> dict:
    iq = read_iq_u8(iq_file, max_samples=max_samples, offset_bytes=0)
    if iq.size == 0:
        raise RuntimeError("no IQ samples")
    return analyze(iq, sample_rate)


def choose_global_gain(results: List[CaptureResult], gains: List[int], min_hits: int) -> Optional[int]:
    by_gain: Dict[int, List[float]] = {g: [] for g in gains}
    by_gain_valid_hits: Dict[int, int] = {g: 0 for g in gains}
    for r in results:
        if r.ok:
            by_gain[r.gain_db].append(r.score)
            if r.valid:
                by_gain_valid_hits[r.gain_db] += 1

    candidates = []
    for g in gains:
        scores = by_gain[g]
        if len(scores) < min_hits:
            continue
        candidates.append((statistics.median(scores), by_gain_valid_hits[g], -g, g))
    if not candidates:
        return None
    candidates.sort(reverse=True)
    return candidates[0][3]


def choose_per_freq_gain(results: List[CaptureResult], freqs: List[int]) -> Dict[int, int]:
    out: Dict[int, int] = {}
    for f in freqs:
        cands = [r for r in results if r.freq_khz == f and r.ok]
        if not cands:
            continue
        cands.sort(key=lambda r: (1 if r.valid else 0, r.score, -r.gain_db), reverse=True)
        out[f] = cands[0].gain_db
    return out


def fill_reference_map(
    freq_start_khz: int,
    freq_end_khz: int,
    freq_step_khz: int,
    measured: Dict[int, int],
) -> Dict[int, dict]:
    points = []
    f = freq_start_khz
    while f <= freq_end_khz:
        points.append(f)
        f += freq_step_khz

    measured_keys = sorted(measured.keys())
    out: Dict[int, dict] = {}
    if not measured_keys:
        for p in points:
            out[p] = {"gain_db": None, "source": "unavailable"}
        return out

    for p in points:
        if p in measured:
            out[p] = {"gain_db": measured[p], "source": "measured"}
            continue
        idx = bisect.bisect_left(measured_keys, p)
        if idx == 0:
            out[p] = {"gain_db": measured[measured_keys[0]], "source": "nearest"}
        elif idx >= len(measured_keys):
            out[p] = {"gain_db": measured[measured_keys[-1]], "source": "nearest"}
        else:
            lo = measured_keys[idx - 1]
            hi = measured_keys[idx]
            if (p - lo) <= (hi - p):
                out[p] = {"gain_db": measured[lo], "source": "nearest"}
            else:
                out[p] = {"gain_db": measured[hi], "source": "nearest"}
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="Calibrate RTL-SDR gain for FM band using IQ quality metrics.")
    ap.add_argument("--config", type=Path, default=ROOT_DIR / "fm-tuner-sdr.ini")
    ap.add_argument("--binary", type=Path, default=ROOT_DIR / "build" / "fm-tuner-sdr")
    ap.add_argument("--freqs", type=str, default="", help="Comma-separated kHz list (overrides --full-band)")
    ap.add_argument("--full-band", action="store_true", help="Use full FM band range for calibration")
    ap.add_argument("--freq-start-khz", type=int, default=87500)
    ap.add_argument("--freq-end-khz", type=int, default=108000)
    ap.add_argument("--freq-step-khz", type=int, default=100)
    ap.add_argument("--gains", type=str, default="0,7,14,20,28,35,42,49", help="Comma-separated dB list")
    ap.add_argument("--seconds", type=int, default=8, help="Capture seconds per test point")
    ap.add_argument("--sample-rate", type=int, default=256000, help="IQ sample rate for analysis")
    ap.add_argument("--max-samples", type=int, default=800000, help="Max IQ samples to analyze per test point")
    ap.add_argument("--hard-clip-max", type=float, default=0.001)
    ap.add_argument("--near-clip-max", type=float, default=0.02)
    ap.add_argument("--retries", type=int, default=2, help="Retries per point when no IQ captured")
    ap.add_argument("--out-dir", type=Path, default=Path("/tmp") / f"fmtuner_gain_cal_{int(time.time())}")
    ap.add_argument("--xdr-port", type=int, default=17373, help="Temporary XDR port to avoid 7373 conflicts")
    ap.add_argument("--json", action="store_true", help="Print JSON summary")
    args = ap.parse_args()

    if not args.binary.exists():
        print(f"Binary not found: {args.binary}")
        return 1
    if not args.config.exists():
        print(f"Config not found: {args.config}")
        return 1

    freqs = build_freq_list(
        args.freqs,
        args.full_band,
        args.freq_start_khz,
        args.freq_end_khz,
        args.freq_step_khz,
    )
    gains = parse_csv_ints(args.gains)
    if not freqs or not gains:
        print("Need at least one frequency and one gain. Use --freqs or --full-band.")
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)
    run_config = args.out_dir / "calibration_runtime.ini"
    write_temp_config_with_xdr_port(args.config, run_config, args.xdr_port)
    results: List[CaptureResult] = []

    total = len(freqs) * len(gains)
    idx = 0
    for f in freqs:
        for g in gains:
            idx += 1
            point_dir = args.out_dir / f"f{f}_g{g}"
            point_dir.mkdir(parents=True, exist_ok=True)
            iq_file = point_dir / "capture.iq"
            log_file = point_dir / "fmtuner.log"

            ok = False
            err = ""
            for attempt in range(1, args.retries + 1):
                if iq_file.exists():
                    iq_file.unlink()
                print(f"[{idx}/{total}] freq={f} kHz gain={g} dB attempt={attempt}/{args.retries}")
                ok = run_capture(args.binary, run_config, f, g, args.seconds, iq_file, log_file)
                if ok:
                    break
                err = "no IQ captured"
                time.sleep(1.0)

            if not ok:
                results.append(CaptureResult(f, g, False, err, None, -1e9, False))
                print(f"  -> fail: {err}")
                continue

            try:
                m = analyze_capture(iq_file, args.sample_rate, args.max_samples)
                score, valid = score_metrics(m, args.hard_clip_max, args.near_clip_max)
                results.append(CaptureResult(f, g, True, "", m, score, valid))
                print(
                    f"  -> power={m['power_dbfs']:.2f} dBFS near={m['near_clip_ratio']:.5f} "
                    f"hard={m['hard_clip_ratio']:.5f} score={score:.2f} valid={valid}"
                )
            except Exception as exc:
                results.append(CaptureResult(f, g, False, str(exc), None, -1e9, False))
                print(f"  -> fail: {exc}")

    min_hits = max(1, len(freqs) // 2)
    global_gain = choose_global_gain(results, gains, min_hits=min_hits)
    per_freq = choose_per_freq_gain(results, freqs)
    reference_map = fill_reference_map(
        args.freq_start_khz,
        args.freq_end_khz,
        args.freq_step_khz,
        per_freq,
    )

    serializable = {
        "config": str(args.config),
        "runtime_config": str(run_config),
        "binary": str(args.binary),
        "freqs_khz": freqs,
        "gains_db": gains,
        "reference_range": {
            "start_khz": args.freq_start_khz,
            "end_khz": args.freq_end_khz,
            "step_khz": args.freq_step_khz,
        },
        "seconds_per_point": args.seconds,
        "hard_clip_max": args.hard_clip_max,
        "near_clip_max": args.near_clip_max,
        "recommended_global_gain_db": global_gain,
        "recommended_per_freq_gain_db": {str(k): v for k, v in per_freq.items()},
        "reference_gain_db": {str(k): v["gain_db"] for k, v in reference_map.items()},
        "reference_source": {str(k): v["source"] for k, v in reference_map.items()},
        "results": [
            {
                "freq_khz": r.freq_khz,
                "gain_db": r.gain_db,
                "ok": r.ok,
                "error": r.error,
                "score": r.score,
                "valid": r.valid,
                "metrics": r.metrics,
            }
            for r in results
        ],
    }

    report_json = args.out_dir / "calibration.json"
    report_ini = args.out_dir / "recommended_sdr.ini"
    ref_csv = args.out_dir / "gain_reference.csv"
    ref_ini = args.out_dir / "gain_reference.ini"
    ref_json = args.out_dir / "gain_reference.json"
    report_json.write_text(json.dumps(serializable, indent=2), encoding="utf-8")

    lines = [
        "[sdr]",
        f"rtl_gain_db = {global_gain if global_gain is not None else -1}",
        "overload_auto_gain = false",
    ]
    report_ini.write_text("\n".join(lines) + "\n", encoding="utf-8")

    csv_lines = ["freq_khz,gain_db,source"]
    for f in sorted(reference_map.keys()):
        g = reference_map[f]["gain_db"]
        s = reference_map[f]["source"]
        csv_lines.append(f"{f},{'' if g is None else g},{s}")
    ref_csv.write_text("\n".join(csv_lines) + "\n", encoding="utf-8")

    ini_lines = ["[gain_reference]"]
    for f in sorted(reference_map.keys()):
        g = reference_map[f]["gain_db"]
        if g is not None:
            ini_lines.append(f"{f} = {g}")
    ref_ini.write_text("\n".join(ini_lines) + "\n", encoding="utf-8")

    ref_json.write_text(
        json.dumps(
            {
                "start_khz": args.freq_start_khz,
                "end_khz": args.freq_end_khz,
                "step_khz": args.freq_step_khz,
                "entries": {str(k): reference_map[k] for k in sorted(reference_map.keys())},
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    if args.json:
        print(json.dumps(serializable, indent=2))
    else:
        print("")
        print("Calibration summary")
        print(f"- Output dir: {args.out_dir}")
        print(f"- Recommended global rtl_gain_db: {global_gain}")
        print("- Recommended per-frequency gain (kHz -> dB):")
        for f in freqs:
            if f in per_freq:
                print(f"  {f} -> {per_freq[f]}")
            else:
                print(f"  {f} -> n/a")
        print(f"- JSON report: {report_json}")
        print(f"- INI snippet: {report_ini}")
        print(f"- Reference CSV: {ref_csv}")
        print(f"- Reference INI: {ref_ini}")
        print(f"- Reference JSON: {ref_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
