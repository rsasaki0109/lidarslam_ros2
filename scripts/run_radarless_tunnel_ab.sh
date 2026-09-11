#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/run_radarless_tunnel_ab.sh [options]

Options:
  --sequence <label>          tunnel, fog, hilti_exp07, or mid360
  --dataset-root <dir>        NTNU degeneracy dataset root
  --bag <dir>                 Override rosbag2 directory
  --lidar-topic <topic>       PointCloud2 topic
  --imu-topic <topic>         IMU topic
  --base-frame <frame>        RKO-LIO base frame (default: base_link)
  --output-root <dir>         Evidence root
  --setup <setup.bash>        Workspace setup file
  --base-params <yaml>        Sensor/base RKO-LIO parameters
  --candidate-preset <yaml>   Candidate overlay parameters
  --reference-tum <path>      Dense pseudo-GT/reference TUM
  --candidate-param <k:=v>    Extra candidate ROS parameter (repeatable)
  --max-runtime-secs <n>      Hard per-arm timeout (default: 1500)
  --poll-secs <n>             Completion polling interval (default: 10)
  --quiet-polls <n>           Quiet polls after activity before SIGINT (default: 2)
  --ros-domain-base <id>      DDS domain for control; candidate uses +1 (default: 210)
  --resume                    Reuse complete arm outputs
  --dry-run                   Validate inputs and print commands
  --help                      Show this help

The runner executes a default-off control and a candidate arm against the
same bag, stops offline_node with SIGINT after odometry becomes quiet, and
writes trajectory metrics plus a candidate-vs-control comparison. It never
selects the final exp02/exp03/exp21 holdouts.
EOF
}

die() {
  echo "error: $*" >&2
  exit 2
}

require_value() {
  [[ $# -ge 2 && -n "$2" && "$2" != -* ]] || die "$1 requires a value"
}

SEQUENCE=tunnel
DATASET_ROOT=${LIDAR_DEGENERACY_DATASET_ROOT:-/media/sasaki/aiueo/benchmarks/lidar_degeneracy_datasets_v1}
OUTPUT_ROOT=${RADARLESS_TUNNEL_BENCHMARK_ROOT:-${DATASET_ROOT}/runs/radarless_tunnel_adaptive_v1}
BAG=""
LIDAR_TOPIC=/os_cloud_node/points
IMU_TOPIC=/vectornav_node/uncomp_imu
BASE_FRAME=base_link
SETUP_FILE="${REPO_ROOT}/install/setup.bash"
BASE_PARAMS="${REPO_ROOT}/lidarslam/param/rko_lio_lidar_degeneracy.ros.yaml"
CONTROL_PRESET="${REPO_ROOT}/lidarslam/param/presets/degeneracy_off.ros.yaml"
CANDIDATE_PRESET="${REPO_ROOT}/lidarslam/param/presets/tunnel_imu_no_radar.ros.yaml"
REFERENCE_TUM=""
MAX_RUNTIME_SECS=1500
POLL_SECS=10
QUIET_POLLS=2
ROS_DOMAIN_BASE=210
RESUME=false
DRY_RUN=false
CANDIDATE_PARAMS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sequence) require_value "$@"; SEQUENCE="$2"; shift 2 ;;
    --dataset-root) require_value "$@"; DATASET_ROOT=$(realpath -m "$2"); shift 2 ;;
    --bag) require_value "$@"; BAG=$(realpath -m "$2"); shift 2 ;;
    --lidar-topic) require_value "$@"; LIDAR_TOPIC="$2"; shift 2 ;;
    --imu-topic) require_value "$@"; IMU_TOPIC="$2"; shift 2 ;;
    --base-frame) require_value "$@"; BASE_FRAME="$2"; shift 2 ;;
    --output-root) require_value "$@"; OUTPUT_ROOT=$(realpath -m "$2"); shift 2 ;;
    --setup) require_value "$@"; SETUP_FILE=$(realpath -m "$2"); shift 2 ;;
    --base-params) require_value "$@"; BASE_PARAMS=$(realpath -m "$2"); shift 2 ;;
    --candidate-preset) require_value "$@"; CANDIDATE_PRESET=$(realpath -m "$2"); shift 2 ;;
    --reference-tum) require_value "$@"; REFERENCE_TUM=$(realpath -m "$2"); shift 2 ;;
    --candidate-param) require_value "$@"; CANDIDATE_PARAMS+=("$2"); shift 2 ;;
    --max-runtime-secs) require_value "$@"; MAX_RUNTIME_SECS="$2"; shift 2 ;;
    --poll-secs) require_value "$@"; POLL_SECS="$2"; shift 2 ;;
    --quiet-polls) require_value "$@"; QUIET_POLLS="$2"; shift 2 ;;
    --ros-domain-base) require_value "$@"; ROS_DOMAIN_BASE="$2"; shift 2 ;;
    --resume) RESUME=true; shift ;;
    --dry-run) DRY_RUN=true; shift ;;
    --help|-h) usage; exit 0 ;;
    *) die "unknown option: $1" ;;
  esac
done

case "${SEQUENCE}" in
  tunnel|fog|hilti_exp07|mid360) ;;
  exp02|exp03|exp21)
    die "${SEQUENCE} is a reserved final holdout and is intentionally blocked"
    ;;
  *) die "unknown sequence: ${SEQUENCE}" ;;
esac
[[ "${MAX_RUNTIME_SECS}" =~ ^[1-9][0-9]*$ ]] || die "--max-runtime-secs must be positive"
[[ "${POLL_SECS}" =~ ^[1-9][0-9]*$ ]] || die "--poll-secs must be positive"
[[ "${QUIET_POLLS}" =~ ^[1-9][0-9]*$ ]] || die "--quiet-polls must be positive"
[[ "${ROS_DOMAIN_BASE}" =~ ^[0-9]+$ ]] || die "--ros-domain-base must be an integer"
((ROS_DOMAIN_BASE <= 231)) || die "--ros-domain-base must be <= 231"
for override in "${CANDIDATE_PARAMS[@]}"; do
  [[ "${override}" == *":="* ]] || die "--candidate-param expects name:=value: ${override}"
done

if [[ -z "${BAG}" ]]; then
  case "${SEQUENCE}" in
    tunnel|fog) BAG="${DATASET_ROOT}/ros2_radar/${SEQUENCE}" ;;
    *) die "--bag is required for ${SEQUENCE}" ;;
  esac
fi
[[ -f "${BAG}/metadata.yaml" ]] || die "rosbag2 metadata not found: ${BAG}/metadata.yaml"
[[ -f "${SETUP_FILE}" ]] || die "setup file not found: ${SETUP_FILE}"
[[ -f "${BASE_PARAMS}" ]] || die "base parameter file not found: ${BASE_PARAMS}"
[[ -f "${CONTROL_PRESET}" ]] || die "control preset not found: ${CONTROL_PRESET}"
[[ -f "${CANDIDATE_PRESET}" ]] || die "candidate preset not found: ${CANDIDATE_PRESET}"

if [[ -z "${REFERENCE_TUM}" && "${SEQUENCE}" == tunnel ]]; then
  REFERENCE_TUM="${DATASET_ROOT}/runs/tunnel_gravity_alignment_v1/tunnel_gravity_v2_gain02_0/tunnel_gravity_v2_gain02_tum_0.txt"
fi
if [[ -n "${REFERENCE_TUM}" ]]; then
  [[ -f "${REFERENCE_TUM}" ]] || die "reference TUM not found: ${REFERENCE_TUM}"
fi

SEQUENCE_ROOT="${OUTPUT_ROOT}/${SEQUENCE}"

print_command() {
  printf 'DRY_RUN'
  printf ' %q' "$@"
  printf '\n'
}

find_trajectory() {
  local arm_dir="$1"
  local run_name="$2"
  local paths=()
  mapfile -t paths < <(find "${arm_dir}" -type f -name "${run_name}_tum_*.txt" -print 2>/dev/null | sort)
  [[ ${#paths[@]} -eq 1 ]] || return 1
  printf '%s\n' "${paths[0]}"
}

stop_process() {
  local pid="$1"
  if ! kill -0 "${pid}" 2>/dev/null; then
    return
  fi
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 "${pid}" 2>/dev/null || return
    sleep 1
  done
  kill -TERM "${pid}" 2>/dev/null || true
}

run_arm() {
  local label="$1"
  local preset="$2"
  local domain="$3"
  shift 3
  local overrides=("$@")
  local arm_dir="${SEQUENCE_ROOT}/${label}"
  local run_name="${SEQUENCE}_${label}"
  local command=(
    ros2 run rko_lio offline_node --ros-args
    --params-file "${BASE_PARAMS}"
    --params-file "${preset}"
    -p "bag_path:=${BAG}"
    -p "lidar_topic:=${LIDAR_TOPIC}"
    -p "imu_topic:=${IMU_TOPIC}"
    -p "base_frame:=${BASE_FRAME}"
    -p "odom_topic:=/rko_lio/odometry"
    -p "dump_results:=true"
    -p "results_dir:=${arm_dir}"
    -p "run_name:=${run_name}"
  )
  for override in "${overrides[@]}"; do
    command+=(-p "${override}")
  done

  if [[ "${DRY_RUN}" == true ]]; then
    printf 'ROS_DOMAIN_ID=%q ' "${domain}"
    print_command "${command[@]}"
    return
  fi
  if [[ "${RESUME}" == true ]] && find_trajectory "${arm_dir}" "${run_name}" >/dev/null; then
    echo "reuse complete ${label} arm: ${arm_dir}"
    return
  fi
  [[ ! -e "${arm_dir}" ]] || die "arm output already exists (use --resume): ${arm_dir}"
  mkdir -p "${arm_dir}"
  printf '%q ' "${command[@]}" >"${arm_dir}/command.txt"
  printf '\n' >>"${arm_dir}/command.txt"

  echo "=== ${SEQUENCE} ${label} start ==="
  ROS_DOMAIN_ID="${domain}" stdbuf -oL -eL "${command[@]}" \
    >"${arm_dir}/offline_node.log" 2>&1 &
  local pid=$!
  local start_time
  start_time=$(date +%s)
  local saw_odometry=false
  local quiet_count=0
  local timed_out=false
  while kill -0 "${pid}" 2>/dev/null; do
    sleep "${POLL_SECS}"
    local now
    now=$(date +%s)
    if ((now - start_time >= MAX_RUNTIME_SECS)); then
      timed_out=true
      break
    fi
    if ROS_DOMAIN_ID="${domain}" timeout 8 ros2 topic echo /rko_lio/odometry --once \
        >/dev/null 2>&1; then
      saw_odometry=true
      quiet_count=0
    elif [[ "${saw_odometry}" == true ]]; then
      quiet_count=$((quiet_count + 1))
      if ((quiet_count >= QUIET_POLLS)); then
        break
      fi
    fi
  done
  stop_process "${pid}"
  set +e
  wait "${pid}"
  local status=$?
  set -e
  if [[ "${timed_out}" == true ]]; then
    die "${label} arm exceeded ${MAX_RUNTIME_SECS}s; see ${arm_dir}/offline_node.log"
  fi
  if [[ "${status}" -ne 0 && "${status}" -ne 130 && "${status}" -ne 143 ]]; then
    die "${label} arm exited ${status}; see ${arm_dir}/offline_node.log"
  fi
  find_trajectory "${arm_dir}" "${run_name}" >/dev/null \
    || die "${label} arm did not produce exactly one TUM trajectory"
  echo "=== ${SEQUENCE} ${label} done ==="
}

if [[ "${DRY_RUN}" != true ]]; then
  # The workspace setup chains the selected ROS distribution.
  # shellcheck disable=SC1090
  set +u
  source "${SETUP_FILE}"
  set -u
  mkdir -p "${SEQUENCE_ROOT}"
  SOURCE_BASE_PARAMS="${BASE_PARAMS}"
  BASE_PARAMS="${SEQUENCE_ROOT}/base_params.ros.yaml"
  python3 - "${SOURCE_BASE_PARAMS}" "${BASE_PARAMS}" <<'PY'
import shutil
import sys
from pathlib import Path

import yaml

source, output = map(Path, sys.argv[1:])
if source.resolve() == output.resolve():
    raise SystemExit(0)
document = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
if isinstance(document, dict) and any(
    isinstance(value, dict) and "ros__parameters" in value
    for value in document.values()
):
    shutil.copyfile(source, output)
else:
    output.write_text(
        yaml.safe_dump({"/**": {"ros__parameters": document}}, sort_keys=False),
        encoding="utf-8",
    )
PY
fi

run_arm control "${CONTROL_PRESET}" "${ROS_DOMAIN_BASE}"
run_arm candidate "${CANDIDATE_PRESET}" "$((ROS_DOMAIN_BASE + 1))" "${CANDIDATE_PARAMS[@]}"

if [[ "${DRY_RUN}" == true ]]; then
  exit 0
fi

CONTROL_TUM=$(find_trajectory "${SEQUENCE_ROOT}/control" "${SEQUENCE}_control")
CANDIDATE_TUM=$(find_trajectory "${SEQUENCE_ROOT}/candidate" "${SEQUENCE}_candidate")
EXPECTED_ARGS=()
if [[ "${SEQUENCE}" == tunnel ]]; then
  EXPECTED_ARGS=(--expected-endpoint-distance 500)
fi

python3 "${SCRIPT_DIR}/evaluate_degeneracy_trajectory.py" \
  "${CONTROL_TUM}" "${EXPECTED_ARGS[@]}" \
  >"${SEQUENCE_ROOT}/control_metrics.json"
python3 "${SCRIPT_DIR}/evaluate_degeneracy_trajectory.py" \
  "${CANDIDATE_TUM}" "${EXPECTED_ARGS[@]}" \
  --reference-trajectory "${CONTROL_TUM}" --min-reference-reach-m 10 \
  >"${SEQUENCE_ROOT}/candidate_vs_control.json"
if [[ -n "${REFERENCE_TUM}" ]]; then
  python3 "${SCRIPT_DIR}/evaluate_degeneracy_trajectory.py" \
    "${CANDIDATE_TUM}" "${EXPECTED_ARGS[@]}" \
    --reference-trajectory "${REFERENCE_TUM}" --min-reference-reach-m 10 \
    >"${SEQUENCE_ROOT}/candidate_metrics.json"
else
  cp "${SEQUENCE_ROOT}/candidate_vs_control.json" "${SEQUENCE_ROOT}/candidate_metrics.json"
fi

python3 - "${SEQUENCE_ROOT}" "${SEQUENCE}" "${BAG}" "${BASE_PARAMS}" \
  "${CANDIDATE_PRESET}" "${REFERENCE_TUM}" \
  "$(git -C "${REPO_ROOT}" rev-parse HEAD)" \
  "$(git -C "${REPO_ROOT}/Thirdparty/rko_lio" rev-parse HEAD)" \
  "${CANDIDATE_PARAMS[@]}" <<'PY'
import json
import sys
from pathlib import Path

root_text, sequence, bag, base_params, preset, reference, parent_sha, submodule_sha = sys.argv[1:9]
candidate_overrides = sys.argv[9:]
root = Path(root_text)
summary_paths = sorted(
    (root / "candidate").glob("*/kinematic_velocity_blend_summary.json")
)
payload = {
    "schema_version": 1,
    "sequence": sequence,
    "inputs": {
        "bag": bag,
        "base_params": base_params,
        "candidate_preset": preset,
        "candidate_overrides": candidate_overrides,
        "reference_tum": reference or None,
    },
    "git": {
        "lidar_slam_ros2": parent_sha,
        "rko_lio": submodule_sha,
    },
    "control": json.loads((root / "control_metrics.json").read_text()),
    "candidate": json.loads((root / "candidate_metrics.json").read_text()),
    "candidate_vs_control": json.loads(
        (root / "candidate_vs_control.json").read_text()
    ),
    "kinematic_velocity_blend_summary": (
        json.loads(summary_paths[0].read_text())
        if len(summary_paths) == 1
        else None
    ),
}
(root / "comparison.json").write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n",
    encoding="utf-8",
)
PY

echo "comparison: ${SEQUENCE_ROOT}/comparison.json"
