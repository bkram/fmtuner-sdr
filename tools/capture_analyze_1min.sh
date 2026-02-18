#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT_DIR}/build/fm-tuner-sdr"
ANALYZER="${ROOT_DIR}/tools/analyze_iq.py"
WAV_ANALYZER="${ROOT_DIR}/tools/analyze_wav.py"
CONFIG="${ROOT_DIR}/fm-tuner-sdr.ini"
DURATION=60
SAMPLE_RATE=256000
FREQ_KHZ=88600
WITH_AUDIO=0
WITH_WAV=1
GATE_ENABLED=1
GATE_TRY_SECONDS=12
GATE_MAX_ATTEMPTS=5
GATE_MIN_IQ_POWER_DBFS=-45
GATE_MIN_WAV_RMS=0.03
FINAL_CAPTURE_RETRIES=3

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Options:
  -c, --config <file>        Config file (default: ${CONFIG})
  -d, --duration <seconds>   Capture duration (default: ${DURATION})
  -r, --sample-rate <hz>     IQ sample rate for analysis (default: ${SAMPLE_RATE})
  -f, --freq-khz <khz>       Tune frequency in kHz (default: ${FREQ_KHZ} = 88.6 MHz)
  -a, --audio                Also enable speaker output during capture
      --no-wav               Disable WAV capture/analysis
      --no-gate              Disable pre-capture signal gate
      --gate-try <seconds>   Per-attempt gate capture length (default: ${GATE_TRY_SECONDS})
      --gate-attempts <n>    Max gate attempts (default: ${GATE_MAX_ATTEMPTS})
      --gate-iq-dbfs <db>    Min IQ power dBFS to pass gate (default: ${GATE_MIN_IQ_POWER_DBFS})
      --gate-wav-rms <rms>   Min WAV RMS to pass gate (default: ${GATE_MIN_WAV_RMS})
  -h, --help                 Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -c|--config)
      CONFIG="$2"
      shift 2
      ;;
    -d|--duration)
      DURATION="$2"
      shift 2
      ;;
    -r|--sample-rate)
      SAMPLE_RATE="$2"
      shift 2
      ;;
    -f|--freq-khz)
      FREQ_KHZ="$2"
      shift 2
      ;;
    -a|--audio)
      WITH_AUDIO=1
      shift
      ;;
    --no-wav)
      WITH_WAV=0
      shift
      ;;
    --no-gate)
      GATE_ENABLED=0
      shift
      ;;
    --gate-try)
      GATE_TRY_SECONDS="$2"
      shift 2
      ;;
    --gate-attempts)
      GATE_MAX_ATTEMPTS="$2"
      shift 2
      ;;
    --gate-iq-dbfs)
      GATE_MIN_IQ_POWER_DBFS="$2"
      shift 2
      ;;
    --gate-wav-rms)
      GATE_MIN_WAV_RMS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -x "${BIN}" ]]; then
  echo "Binary not found or not executable: ${BIN}" >&2
  echo "Build first: cmake --build ${ROOT_DIR}/build" >&2
  exit 1
fi

if [[ ! -f "${CONFIG}" ]]; then
  echo "Config file not found: ${CONFIG}" >&2
  exit 1
fi

if [[ ! -f "${ANALYZER}" ]]; then
  echo "Analyzer not found: ${ANALYZER}" >&2
  exit 1
fi
if [[ "${WITH_WAV}" -eq 1 && ! -f "${WAV_ANALYZER}" ]]; then
  echo "WAV analyzer not found: ${WAV_ANALYZER}" >&2
  exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="/tmp/fmtuner_iq_${STAMP}"
mkdir -p "${OUT_DIR}"
capture_once() {
  local target_dir="$1"
  local seconds="$2"
  local iq_file="${target_dir}/capture.iq"
  local wav_file="${target_dir}/capture.wav"
  local log_file="${target_dir}/fmtuner.log"
  local cmd=("${BIN}" -c "${CONFIG}" -f "${FREQ_KHZ}" -i "${iq_file}")
  if [[ "${WITH_WAV}" -eq 1 ]]; then
    cmd+=(-w "${wav_file}")
  fi
  if [[ "${WITH_AUDIO}" -eq 1 ]]; then
    cmd+=(-s)
  fi
  echo "[RUN] ${cmd[*]}"
  "${cmd[@]}" >"${log_file}" 2>&1 &
  local pid=$!
  sleep "${seconds}"
  kill -INT "${pid}" >/dev/null 2>&1 || true
  sleep 1
  kill -TERM "${pid}" >/dev/null 2>&1 || true
  wait "${pid}" 2>/dev/null || true
}

gate_passed=1
if [[ "${GATE_ENABLED}" -eq 1 ]]; then
  gate_passed=0
  echo "[INFO] Running signal gate checks..."
  for ((attempt=1; attempt<=GATE_MAX_ATTEMPTS; attempt++)); do
    att_dir="${OUT_DIR}/gate_attempt_${attempt}"
    mkdir -p "${att_dir}"
    echo "[INFO] Gate attempt ${attempt}/${GATE_MAX_ATTEMPTS} (${GATE_TRY_SECONDS}s)"
    capture_once "${att_dir}" "${GATE_TRY_SECONDS}"
    iq_file="${att_dir}/capture.iq"
    if [[ ! -s "${iq_file}" ]]; then
      echo "[WARN] Gate attempt ${attempt}: no IQ captured"
      continue
    fi
    iq_json="${att_dir}/iq.json"
    python3 "${ANALYZER}" "${iq_file}" --sample-rate "${SAMPLE_RATE}" --json > "${iq_json}"
    iq_power="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["power_dbfs"])' "${iq_json}")"
    wav_rms=0
    if [[ "${WITH_WAV}" -eq 1 && -s "${att_dir}/capture.wav" ]]; then
      wav_json="${att_dir}/wav.json"
      python3 "${WAV_ANALYZER}" "${att_dir}/capture.wav" --json > "${wav_json}"
      wav_rms="$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print((d["rms_left"]+d["rms_right"])/2.0)' "${wav_json}")"
    fi
    pass="$(python3 - "${iq_power}" "${wav_rms}" "${GATE_MIN_IQ_POWER_DBFS}" "${GATE_MIN_WAV_RMS}" <<'PY'
import sys
iq=float(sys.argv[1]); wav=float(sys.argv[2]); iq_min=float(sys.argv[3]); wav_min=float(sys.argv[4])
print(1 if (iq >= iq_min and wav >= wav_min) else 0)
PY
)"
    echo "[INFO] Gate metrics: iq_power=${iq_power} dBFS wav_rms=${wav_rms}"
    if [[ "${pass}" == "1" ]]; then
      gate_passed=1
      break
    fi
  done
fi

if [[ "${gate_passed}" -ne 1 ]]; then
  echo "[ERROR] Signal gate did not pass; capture likely noise-only." >&2
  echo "[HINT] Try tuning/antenna change or run with --no-gate." >&2
  exit 1
fi

IQ_FILE="${OUT_DIR}/capture.iq"
LOG_FILE="${OUT_DIR}/fmtuner.log"
WAV_FILE="${OUT_DIR}/capture.wav"
REPORT_TXT="${OUT_DIR}/analysis.txt"
REPORT_JSON="${OUT_DIR}/analysis.json"
WAV_REPORT_TXT="${OUT_DIR}/analysis_wav.txt"
WAV_REPORT_JSON="${OUT_DIR}/analysis_wav.json"

capture_ok=0
for ((attempt=1; attempt<=FINAL_CAPTURE_RETRIES; attempt++)); do
  : > "${IQ_FILE}"
  if [[ "${WITH_WAV}" -eq 1 ]]; then
    : > "${WAV_FILE}"
  fi
  echo "[INFO] Capturing IQ for ${DURATION}s... (attempt ${attempt}/${FINAL_CAPTURE_RETRIES})"
  capture_once "${OUT_DIR}" "${DURATION}"
  if [[ -s "${IQ_FILE}" ]]; then
    capture_ok=1
    break
  fi
  echo "[WARN] Final capture attempt ${attempt} produced no IQ data."
  tail -n 20 "${LOG_FILE}" || true
  sleep 2
done

if [[ "${capture_ok}" -ne 1 ]]; then
  echo "[ERROR] No IQ data captured after ${FINAL_CAPTURE_RETRIES} attempts: ${IQ_FILE}" >&2
  echo "[INFO] Last log lines:" >&2
  tail -n 40 "${LOG_FILE}" >&2 || true
  exit 1
fi

echo "[INFO] Running analyzer..."
python3 "${ANALYZER}" "${IQ_FILE}" --sample-rate "${SAMPLE_RATE}" | tee "${REPORT_TXT}"
python3 "${ANALYZER}" "${IQ_FILE}" --sample-rate "${SAMPLE_RATE}" --json > "${REPORT_JSON}"

if [[ "${WITH_WAV}" -eq 1 && -s "${WAV_FILE}" ]]; then
  echo "[INFO] Running WAV analyzer..."
  python3 "${WAV_ANALYZER}" "${WAV_FILE}" | tee "${WAV_REPORT_TXT}"
  python3 "${WAV_ANALYZER}" "${WAV_FILE}" --json > "${WAV_REPORT_JSON}"
fi

echo
echo "[DONE] Output directory: ${OUT_DIR}"
echo "  IQ capture:   ${IQ_FILE}"
if [[ "${WITH_WAV}" -eq 1 ]]; then
  echo "  WAV capture:  ${WAV_FILE}"
fi
echo "  fm-tuner log: ${LOG_FILE}"
echo "  report text:  ${REPORT_TXT}"
echo "  report json:  ${REPORT_JSON}"
if [[ "${WITH_WAV}" -eq 1 && -f "${WAV_REPORT_TXT}" ]]; then
  echo "  wav report:   ${WAV_REPORT_TXT}"
  echo "  wav json:     ${WAV_REPORT_JSON}"
fi
