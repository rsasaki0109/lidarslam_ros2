#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  smoke_mid360_robot_runbook.sh [options]

Smoke-test the Jetson MID-360 robot runbook without starting SLAM.

Options:
  --work-dir <dir>       Temporary working directory for the synthetic bag
  --output-dir <dir>     Output directory for readiness and run-plan files
  --profile <file>       Robot profile YAML (default: configs/mid360_robot/livox_mid360_default.yaml)
  --keep-work-dir        Do not delete the temporary working directory
  --help                 Show this help

The smoke creates a synthetic rosbag2 with /livox/lidar, /livox/imu, and
/tf_static records, then runs profile validation, recording dry-run,
post-recording check, readiness, and map dry-run. It does not launch ROS or
SLAM. The Python rosbags package is required.
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

WORK_DIR=""
OUTPUT_DIR=""
PROFILE="${REPO_ROOT}/configs/mid360_robot/livox_mid360_default.yaml"
KEEP_WORK_DIR=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      [[ $# -ge 2 ]] || usage
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      [[ $# -ge 2 ]] || usage
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --profile)
      [[ $# -ge 2 ]] || usage
      PROFILE="$2"
      shift 2
      ;;
    --keep-work-dir)
      KEEP_WORK_DIR=true
      shift
      ;;
    --help|-h)
      usage
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      ;;
  esac
done

if [[ -z "$WORK_DIR" ]]; then
  WORK_DIR=$(mktemp -d)
else
  WORK_DIR=$(realpath -m "$WORK_DIR")
  mkdir -p "$WORK_DIR"
fi

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="${WORK_DIR}/output"
else
  OUTPUT_DIR=$(realpath -m "$OUTPUT_DIR")
fi

PROFILE=$(realpath "$PROFILE")
RECORD_ROOT="${WORK_DIR}/recordings"
BAG_DIR="${RECORD_ROOT}/smoke_record"

cleanup() {
  if [[ "$KEEP_WORK_DIR" == "false" ]]; then
    rm -rf "$WORK_DIR"
  fi
}
trap cleanup EXIT

mkdir -p "$RECORD_ROOT" "$OUTPUT_DIR"

cd "$REPO_ROOT"

python3 scripts/generate_mid360_robot_sample_bag.py "$BAG_DIR" \
  --duration-sec 5 \
  --pointcloud-rate-hz 10 \
  --imu-rate-hz 100 \
  --point-count 16 \
  --force >/dev/null

python3 scripts/validate_mid360_robot_profile.py "$PROFILE" >/dev/null

bash scripts/run_mid360_robot_field_session.sh \
  --robot-profile "$PROFILE" \
  --bag-root "$RECORD_ROOT" \
  --run-id smoke_field \
  --duration-sec 5 \
  --output-dir "$OUTPUT_DIR" \
  --skip-host-readiness \
  --dry-run >/dev/null

bash scripts/record_mid360_robot_bag.sh \
  --robot-profile "$PROFILE" \
  --bag-root "$RECORD_ROOT" \
  --run-id smoke_record \
  --duration-sec 5 \
  --dry-run >/dev/null

bash scripts/check_mid360_robot_recording.sh \
  --bag "$BAG_DIR" \
  --robot-profile "$RECORD_ROOT/smoke_record_profile.yaml" \
  --record-plan "$RECORD_ROOT/smoke_record_record_plan.json" \
  --output-dir "$OUTPUT_DIR" >/dev/null

python3 scripts/check_mid360_robot_readiness.py "$BAG_DIR" \
  --robot-profile "$RECORD_ROOT/smoke_record_profile.yaml" \
  --output-dir "$OUTPUT_DIR" \
  --write-manifest >/dev/null

bash scripts/run_mid360_robot_map.sh "$BAG_DIR" \
  --robot-profile "$RECORD_ROOT/smoke_record_profile.yaml" \
  --output-dir "$OUTPUT_DIR" \
  --write-manifest \
  --dry-run >/dev/null

for path in \
  "$OUTPUT_DIR/mid360_robot_readiness.json" \
  "$OUTPUT_DIR/mid360_robot_readiness.md" \
  "$OUTPUT_DIR/mid360_robot_run_plan.json" \
  "$OUTPUT_DIR/mid360_robot_run_plan.md" \
  "$OUTPUT_DIR/mid360_robot_session_dashboard.html" \
  "$OUTPUT_DIR/mid360_robot_field_session.json" \
  "$OUTPUT_DIR/mid360_robot_field_session.md" \
  "$OUTPUT_DIR/mid360_robot_recording_check.json" \
  "$OUTPUT_DIR/mid360_robot_recording_check.md" \
  "$RECORD_ROOT/smoke_field_record_plan.json" \
  "$RECORD_ROOT/smoke_field_profile.yaml" \
  "$RECORD_ROOT/smoke_record_record_plan.json" \
  "$RECORD_ROOT/smoke_record_record_plan.md" \
  "$RECORD_ROOT/smoke_record_profile.yaml"; do
  [[ -f "$path" ]] || { echo "expected output missing: $path" >&2; exit 1; }
done

python3 - "$OUTPUT_DIR/mid360_robot_readiness.json" <<'PY'
from pathlib import Path
import json
import math
import sys

payload = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
if payload.get('status') != 'PASS':
    raise SystemExit(f"readiness status is not PASS: {payload.get('status')}")
diagnostics = payload.get('bag_diagnostics', {}).get('topics', {})
if not math.isclose(
    diagnostics.get('pointcloud', {}).get('metadata_rate_hz', 0.0),
    10.0,
    rel_tol=0.02,
):
    raise SystemExit('pointcloud metadata rate check missing or incorrect')
if not math.isclose(
    diagnostics.get('imu', {}).get('metadata_rate_hz', 0.0),
    100.0,
    rel_tol=0.02,
):
    raise SystemExit('imu metadata rate check missing or incorrect')
PY

python3 - "$OUTPUT_DIR/mid360_robot_recording_check.json" <<'PY'
from pathlib import Path
import json
import sys

payload = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
if payload.get('status') != 'PASS':
    raise SystemExit(f"recording check status is not PASS: {payload.get('status')}")
PY

echo "MID-360 robot runbook smoke: PASS"
echo "  sample_bag: $BAG_DIR"
echo "  output_dir: $OUTPUT_DIR"
