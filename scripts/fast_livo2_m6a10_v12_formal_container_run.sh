#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
set -Eeuo pipefail

# v12 container candidate: callback transport and terminal support context are
# independent mapper authorities.  CSV output is diagnostic only.
EXPECTED_CONTRACT='m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
EXPECTED_PHASE_MODE='unpaced_ack'
EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/fast_livo2_m6a10_v12_formal_ready.yaml'
EXPECTED_PROFILE_SHA256='675996a7a5fd81d59d752de4bbea1d4373487a17d03ee085aa08ae9493a0b28d'
EXPECTED_BAG_PATH='/input/ntu_viral.bag'
EXPECTED_BAG_BYTES='11290464091'
EXPECTED_BAG_SHA256='5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310'
EXPECTED_MESSAGES='236687'
EXPECTED_LIDAR_MESSAGES='5793'
EXPECTED_IMU_MESSAGES='225102'
EXPECTED_IMAGE_MESSAGES='5792'
EXPECTED_END_TIMESTAMP='1623491515.148352'
EXPECTED_END_GAP='0.25'
EXPECTED_MIN_POLL='0.05'
EXPECTED_CALLBACK_LATENCY='0.25'
EXPECTED_TRANSPORT_LIMIT='1'
EXPECTED_SENSOR_DURATION='579.278127298'
EXPECTED_TIMING_CONTRACT='m6a10-online-compute-v3-timing-v1'
EXPECTED_FEEDER_SHA256='869ca54921c86310af5cefc4ef0c4f8626b5fdcc125dcd60e865fdf1e677ddbf'
CALLBACK_CONTRACT='m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
TRANSPORT_CONTRACT='m6a10-v12-callback-ack-transport-outstanding-v1'
TERMINAL_RAW_CONTRACT='m6a10-fast-livo2-consumer-terminal-v1'
FEEDER_PATH='/runner/scripts/fast_livo2_m6a10_feeder.py'

fail() { echo "FAST-LIVO2 v12 formal container gate failed: $*" >&2; return 1; }
OUT_DIR="${OUT_DIR:-/out}"
BAG_PATH="${BAG_PATH:-${M6A10_BAG_PATH:-}}"
PROFILE_SHA256="${M6A10_PROFILE_SHA256:?M6A10_PROFILE_SHA256 is required}"
PROFILE_PATH="${M6A10_PROFILE_PATH:?M6A10_PROFILE_PATH is required}"
M6A10_PHASE_CONTRACT_VERSION="${M6A10_PHASE_CONTRACT_VERSION:?M6A10_PHASE_CONTRACT_VERSION is required}"
M6A10_PHASE_MODE="${M6A10_PHASE_MODE:?M6A10_PHASE_MODE is required}"
M6A10_FAST_FEEDER_SHA256="${M6A10_FAST_FEEDER_SHA256:?M6A10_FAST_FEEDER_SHA256 is required}"
M6A10_CONSUMER_EVIDENCE="${M6A10_CONSUMER_EVIDENCE:-}"
M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE="${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE:-}"
M6A10_ONLINE_TIMING_EVIDENCE="${M6A10_ONLINE_TIMING_EVIDENCE:-}"
M6A10_SENSOR_DURATION_SECONDS="${M6A10_SENSOR_DURATION_SECONDS:-}"
M6A10_TIMING_CONTRACT_VERSION="${M6A10_TIMING_CONTRACT_VERSION:-}"
BAG_BYTES="${M6A10_BAG_BYTES:-${M6A10_EXPECTED_BAG_BYTES:-}}"
BAG_SHA256="${M6A10_BAG_SHA256:-${M6A10_EXPECTED_BAG_SHA256:-}}"
M6A10_FAST_EXPECTED_MESSAGES="${M6A10_FAST_EXPECTED_MESSAGES:-${M6A10_EXPECTED_MESSAGES:-}}"
M6A10_FAST_EXPECTED_LIDAR_MESSAGES="${M6A10_FAST_EXPECTED_LIDAR_MESSAGES:-${M6A10_EXPECTED_LIDAR_MESSAGES:-}}"
M6A10_FAST_EXPECTED_IMU_MESSAGES="${M6A10_FAST_EXPECTED_IMU_MESSAGES:-${M6A10_EXPECTED_IMU_MESSAGES:-}}"
M6A10_FAST_EXPECTED_IMAGE_MESSAGES="${M6A10_FAST_EXPECTED_IMAGE_MESSAGES:-${M6A10_EXPECTED_IMAGE_MESSAGES:-}}"
M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS="${M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS:-${M6A10_REQUIRED_END_TIMESTAMP_SECONDS:-}}"
M6A10_FAST_MAX_END_GAP_SECONDS="${M6A10_FAST_MAX_END_GAP_SECONDS:-${EXPECTED_END_GAP}}"
M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS="${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS:-${EXPECTED_MIN_POLL}}"
M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS="${M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS:-${EXPECTED_CALLBACK_LATENCY}}"
M6A10_FAST_MAX_BACKLOG_MESSAGES="${M6A10_FAST_MAX_BACKLOG_MESSAGES:-${EXPECTED_TRANSPORT_LIMIT}}"
[[ -n "${BAG_PATH}" && -n "${BAG_BYTES}" && -n "${BAG_SHA256}" &&
   -n "${M6A10_FAST_EXPECTED_MESSAGES}" &&
   -n "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" &&
   -n "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" &&
   -n "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" &&
   -n "${M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS}" &&
   -n "${M6A10_CONSUMER_EVIDENCE}" &&
   -n "${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE}" &&
   -n "${M6A10_ONLINE_TIMING_EVIDENCE}" &&
   -n "${M6A10_SENSOR_DURATION_SECONDS}" &&
   -n "${M6A10_TIMING_CONTRACT_VERSION}" ]] || fail 'environment incomplete'
[[ "${M6A10_PHASE_CONTRACT_VERSION}" == "${EXPECTED_CONTRACT}" ]] || fail 'phase contract mismatch'
[[ "${M6A10_PHASE_MODE}" == "${EXPECTED_PHASE_MODE}" ]] || fail 'phase mode mismatch'
[[ "${PROFILE_SHA256}" == "${EXPECTED_PROFILE_SHA256}" ]] || fail 'profile SHA mismatch'
case "${PROFILE_PATH}" in
  "${EXPECTED_PROFILE_PATH}"|"/runner/${EXPECTED_PROFILE_PATH}") ;;
  *) fail 'profile path mismatch' ;;
esac
[[ "${BAG_PATH}" == "${EXPECTED_BAG_PATH}" ]] || fail 'bag path mismatch'
[[ "${BAG_BYTES}" == "${EXPECTED_BAG_BYTES}" ]] || fail 'bag bytes mismatch'
[[ "${BAG_SHA256}" == "${EXPECTED_BAG_SHA256}" ]] || fail 'bag SHA mismatch'
[[ "${M6A10_FAST_EXPECTED_MESSAGES}" == "${EXPECTED_MESSAGES}" ]] || fail 'total count mismatch'
[[ "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" == "${EXPECTED_LIDAR_MESSAGES}" ]] || fail 'lidar count mismatch'
[[ "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" == "${EXPECTED_IMU_MESSAGES}" ]] || fail 'imu count mismatch'
[[ "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" == "${EXPECTED_IMAGE_MESSAGES}" ]] || fail 'image count mismatch'
[[ "${M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS}" == "${EXPECTED_END_TIMESTAMP}" ]] || fail 'required end mismatch'
[[ "${M6A10_FAST_MAX_END_GAP_SECONDS}" == "${EXPECTED_END_GAP}" ]] || fail 'end gap mismatch'
[[ "${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS}" == "${EXPECTED_MIN_POLL}" ]] || fail 'poll interval mismatch'
[[ "${M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS}" == "${EXPECTED_CALLBACK_LATENCY}" ]] || fail 'callback latency mismatch'
[[ "${M6A10_FAST_MAX_BACKLOG_MESSAGES}" == "${EXPECTED_TRANSPORT_LIMIT}" ]] || fail 'transport limit mismatch'
[[ "${M6A10_SENSOR_DURATION_SECONDS}" == "${EXPECTED_SENSOR_DURATION}" ]] || fail 'sensor duration mismatch'
[[ "${M6A10_TIMING_CONTRACT_VERSION}" == "${EXPECTED_TIMING_CONTRACT}" ]] || fail 'timing contract mismatch'
[[ "${M6A10_FAST_FEEDER_SHA256}" == "${EXPECTED_FEEDER_SHA256}" ]] || fail 'feeder SHA mismatch'

mkdir -p "${OUT_DIR}"; OUT_DIR="$(cd "${OUT_DIR}" && pwd -P)"
CALLBACK_EVIDENCE="${M6A10_CONSUMER_EVIDENCE}"
TERMINAL_EVIDENCE="${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE}"
ONLINE_TIMING="${M6A10_ONLINE_TIMING_EVIDENCE}"
[[ "${CALLBACK_EVIDENCE}" == "${OUT_DIR}/callback_consumer_evidence.json" ]] || fail 'callback path mismatch'
[[ "${TERMINAL_EVIDENCE}" == "${OUT_DIR}/consumer_evidence.json" ]] || fail 'terminal path mismatch'
[[ "${CALLBACK_EVIDENCE}" != "${TERMINAL_EVIDENCE}" ]] || fail 'callback/terminal paths alias'
[[ "${ONLINE_TIMING}" == "${OUT_DIR}/online_compute_timing.json" ]] || fail 'timing path mismatch'
for path in "${CALLBACK_EVIDENCE}" "${TERMINAL_EVIDENCE}" "${ONLINE_TIMING}"; do
  [[ ! -e "${path}" && ! -L "${path}" && ! -e "${path}.part" && ! -L "${path}.part" ]] || fail 'output exists or is staging'
done
[[ -f "${FEEDER_PATH}" && ! -L "${FEEDER_PATH}" ]] || fail 'feeder missing'
[[ "$(sha256sum "${FEEDER_PATH}" | awk '{print $1}')" == "${EXPECTED_FEEDER_SHA256}" ]] || fail 'feeder SHA mismatch'

export ROS_MASTER_URI='http://127.0.0.1:11311' ROS_IP='127.0.0.1' ROS_HOSTNAME='127.0.0.1'
export ROS_HOME="${OUT_DIR}/ros_home" ROS_LOG_DIR="${OUT_DIR}/ros_logs"
export M6A10_PHASE_CONTRACT_VERSION M6A10_PHASE_MODE M6A10_PROFILE_SHA256 M6A10_PROFILE_PATH
export M6A10_CONSUMER_EVIDENCE="${CALLBACK_EVIDENCE}"
export M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE="${TERMINAL_EVIDENCE}"
export M6A10_ONLINE_TIMING_EVIDENCE="${ONLINE_TIMING}"
export M6A10_SENSOR_DURATION_SECONDS M6A10_TIMING_CONTRACT_VERSION
export M6A10_FAST_EXPECTED_MESSAGES M6A10_FAST_EXPECTED_LIDAR_MESSAGES
export M6A10_FAST_EXPECTED_IMU_MESSAGES M6A10_FAST_EXPECTED_IMAGE_MESSAGES
export M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS M6A10_FAST_MAX_END_GAP_SECONDS
export M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS
export M6A10_FAST_MAX_BACKLOG_MESSAGES
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

ROSCORE_PID=''; MAPPER_PID=''; ODOM_PID=''; FEEDER_PID=''
stop_group() { local pid="${1:-}"; [[ -n "${pid}" ]] || return 0; kill -TERM -- "-${pid}" >/dev/null 2>&1 || true; kill -TERM "${pid}" >/dev/null 2>&1 || true; }
cleanup() {
  local status=$?; trap - EXIT INT TERM; set +e
  stop_group "${FEEDER_PID}"; stop_group "${ODOM_PID}"; stop_group "${MAPPER_PID}"; stop_group "${ROSCORE_PID}"
  for pid in "${FEEDER_PID}" "${ODOM_PID}" "${MAPPER_PID}" "${ROSCORE_PID}"; do [[ -n "${pid}" ]] && wait "${pid}" >/dev/null 2>&1 || true; done
  exit "${status}"
}
trap cleanup EXIT; trap 'exit 130' INT; trap 'exit 143' TERM

source /opt/ros/noetic/setup.bash
source /opt/fast_livo_ws/devel/setup.bash
setsid roscore >"${OUT_DIR}/roscore.log" 2>&1 & ROSCORE_PID=$!
master_deadline=$((SECONDS + ${M6A10_STARTUP_TIMEOUT_SECONDS:-60})); master_ready=0
while (( SECONDS < master_deadline )); do
  if rosparam list >/dev/null 2>&1; then master_ready=1; break; fi
  sleep 0.1
done
printf '%s\n' "${master_ready}" >"${OUT_DIR}/master_ready.txt"; (( master_ready == 1 )) || fail 'ROS master not ready'
rosparam set use_sim_time true
setsid roslaunch fast_livo mapping_ouster_ntu.launch rviz:=false >"${OUT_DIR}/mapper.log" 2>&1 & MAPPER_PID=$!
required_services=(
  /m6a10/consumer_status /m6a10/consumer_ack /m6a10/consumer_eof
  /m6a10/consumer_finalize /m6a10/terminal_status /m6a10/terminal_eof
  /m6a10/terminal_finalize
)
service_deadline=$((SECONDS + ${M6A10_STARTUP_TIMEOUT_SECONDS:-60})); services_ready=0
while (( SECONDS < service_deadline )); do
  services_ready=1
  for service in "${required_services[@]}"; do
    if ! rosservice list 2>/dev/null | grep -Fxq "${service}"; then services_ready=0; break; fi
  done
  (( services_ready == 1 )) && break
  kill -0 "${MAPPER_PID}" >/dev/null 2>&1 || fail 'mapper exited before services'; sleep 0.2
done
printf '%s\n' "${services_ready}" >"${OUT_DIR}/services_ready.txt"; (( services_ready == 1 )) || fail 'seven services missing'

# The CSV is an artifact only.  No CSV timestamp parser or EOF gate is used.
setsid rostopic echo -p /aft_mapped_to_init >"${OUT_DIR}/odometry.csv" 2>"${OUT_DIR}/odometry.err" & ODOM_PID=$!
INPUT_START_MONOTONIC_NS="$(python3 -c 'import time; print(time.monotonic_ns())')"
set +e
setsid python3 "${FEEDER_PATH}" --bag "${BAG_PATH}" --output "${OUT_DIR}/feeder_receipt.json" \
  --timeout "${M6A10_FEEDER_TIMEOUT_SECONDS:-1200}" --expected-lidar "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" \
  --expected-imu "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" --expected-image "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" \
  >"${OUT_DIR}/feeder.log" 2>&1 & FEEDER_PID=$!
wait "${FEEDER_PID}"; feeder_status=$?; FEEDER_PID=''; set -e
printf '%s\n' "${feeder_status}" >"${OUT_DIR}/feeder_exit_status.txt"; (( feeder_status == 0 )) || fail 'feeder failed'
python3 - "${OUT_DIR}/feeder_receipt.json" "${BAG_PATH}" "${BAG_BYTES}" "${BAG_SHA256}" \
  "${M6A10_FAST_EXPECTED_MESSAGES}" "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" \
  "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" <<'PY'
import json, sys
path, bag, size, digest, total, lidar, imu, image = sys.argv[1:]
with open(path, encoding='utf-8') as stream: value = json.load(stream)
counts = {'lidar': int(lidar), 'imu': int(imu), 'image': int(image)}
if value.get('status') != 'pass' or value.get('single_inflight') is not True: raise SystemExit('feeder status')
if value.get('bag_path') != bag or value.get('bag_bytes') != int(size) or value.get('bag_sha256') != digest: raise SystemExit('feeder identity')
if value.get('published_messages') != int(total) or value.get('expected_topic_counts') != counts or value.get('published_topic_counts') != counts or value.get('acked_topic_counts') != counts: raise SystemExit('feeder counts')
if value.get('ack_backpressure_verified') is not True or value.get('ground_truth_content_opened') is not False or value.get('scorer_invoked') is not False: raise SystemExit('feeder safety')
PY

call_trigger() {
  local service="${1:?service required}" output="${2:?response path required}"
  timeout "${M6A10_RPC_TIMEOUT_SECONDS:-10}" rosservice call "${service}" "{}" >"${output}" 2>"${output}.err" || return 1
  python3 - "${output}" <<'PY'
import sys, yaml
with open(sys.argv[1], encoding='utf-8') as stream: response = yaml.safe_load(stream) or {}
if response.get('success') is not True: raise SystemExit(1)
PY
}

# Callback close is deliberately before terminal close: EOF -> diagnostic
# status -> finalize.  Final callback evidence is checked before terminal EOF.
call_trigger /m6a10/consumer_eof "${OUT_DIR}/consumer_eof.response" || fail 'consumer_eof failed'
call_trigger /m6a10/consumer_status "${OUT_DIR}/consumer_status.response" || fail 'consumer_status failed'
call_trigger /m6a10/consumer_finalize "${OUT_DIR}/consumer_finalize.response" || fail 'consumer_finalize failed'
CALLBACK_EVIDENCE_SHA256="$(python3 - "${CALLBACK_EVIDENCE}" "${M6A10_FAST_EXPECTED_MESSAGES}" \
  "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" <<'PY'
import hashlib, json, sys
from pathlib import Path
path = Path(sys.argv[1]); total = int(sys.argv[2]); expected = {'lidar': int(sys.argv[3]), 'imu': int(sys.argv[4]), 'image': int(sys.argv[5])}
if path.is_symlink() or not path.is_file() or Path(str(path) + '.part').exists(): raise SystemExit('callback regular-file gate')
raw = path.read_bytes(); value = json.loads(raw.decode('utf-8'))
if value.get('schema_version') != 3 or value.get('contract_version') != 'm6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary' or value.get('transport_contract_version') != 'm6a10-v12-callback-ack-transport-outstanding-v1' or value.get('status') != 'pass': raise SystemExit('callback schema/contract/status')
for key in ('input', 'profile_sha256', 'profile_path', 'raw_path', 'raw_sha256', 'published', 'ack', 'binding', 'validation'):
    if key in value: raise SystemExit('callback authority field')
ledger = value.get('consumer')
if not isinstance(ledger, dict) or ledger.get('received_topic_counts') != expected or ledger.get('received_messages') != total or ledger.get('acked_messages') != total: raise SystemExit('callback exact counts')
if ledger.get('ack_exact') is not True or ledger.get('transport_outstanding_at_drain') != 0 or ledger.get('maximum_transport_outstanding_messages') != 1 or ledger.get('maximum_allowed_transport_outstanding_messages') != 1: raise SystemExit('callback transport')
for key in ('mapper_internal_deque_current_messages', 'mapper_internal_deque_peak_messages'):
    if isinstance(ledger.get(key), bool) or not isinstance(ledger.get(key), int) or ledger[key] < 0: raise SystemExit('mapper deque diagnostic')
if ledger.get('eof_observed') is not True or ledger.get('drain_complete') is not True or ledger.get('dropped_messages') != 0 or ledger.get('queue_overflow') != 0 or ledger.get('processing_failures') != 0 or ledger.get('ack_backpressure_verified') is not True: raise SystemExit('callback completion')
if value.get('ground_truth_content_opened') is not False or value.get('scorer_invoked') is not False: raise SystemExit('callback safety')
print(hashlib.sha256(raw).hexdigest())
PY
)"
printf '%s\n' "${CALLBACK_EVIDENCE_SHA256}" >"${OUT_DIR}/callback_consumer_evidence.sha256"

call_trigger /m6a10/terminal_eof "${OUT_DIR}/terminal_eof.response" || fail 'terminal_eof failed'
terminal_status_count=0; terminal_status_successes=0
terminal_status_deadline=$((SECONDS + ${M6A10_TERMINAL_DRAIN_TIMEOUT_SECONDS:-120}))
while (( SECONDS < terminal_status_deadline )); do
  terminal_status_count=$((terminal_status_count + 1))
  if call_trigger /m6a10/terminal_status "${OUT_DIR}/terminal_status_${terminal_status_count}.response"; then
    terminal_status_successes=$((terminal_status_successes + 1))
    if (( terminal_status_successes >= 2 )); then break; fi
    sleep "${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS}"
  else sleep 0.2; fi
done
(( terminal_status_successes >= 2 )) || fail 'two distinct terminal status RPCs missing'
call_trigger /m6a10/terminal_finalize "${OUT_DIR}/terminal_finalize.response" || fail 'terminal_finalize failed'
RAW_EVIDENCE_SHA256="$(python3 - "${TERMINAL_EVIDENCE}" "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" \
  "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" "${M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS}" "${M6A10_FAST_MAX_END_GAP_SECONDS}" <<'PY'
import hashlib, json, math, sys
from pathlib import Path
path = Path(sys.argv[1]); expected = {'lidar': int(sys.argv[2]), 'imu': int(sys.argv[3]), 'image': int(sys.argv[4])}; required = float(sys.argv[5]); maximum = float(sys.argv[6])
if path.is_symlink() or not path.is_file() or Path(str(path) + '.part').exists(): raise SystemExit('terminal regular-file gate')
raw = path.read_bytes(); value = json.loads(raw.decode('utf-8'))
if value.get('schema_version') != 1 or value.get('contract_id') != 'm6a10-fast-livo2-consumer-terminal-v1' or value.get('status') != 'pass': raise SystemExit('terminal schema/contract/status')
for key in ('input', 'profile_sha256', 'profile_path', 'raw_path', 'raw_sha256', 'expected', 'published', 'ack', 'binding', 'validation'):
    if key in value: raise SystemExit('terminal authority field')
if value.get('received_topic_counts') != expected or value.get('ground_truth_content_opened') is not False or value.get('scorer_invoked') is not False: raise SystemExit('terminal counts/safety')
backend = value.get('backend'); boundary = backend.get('completed_boundary') if isinstance(backend, dict) else None
if not isinstance(backend, dict) or backend.get('quiescent') is not True or backend.get('quiescence_observed') is not True or backend.get('in_flight', {}).get('active') is not False or not isinstance(boundary, dict) or boundary.get('observed') is not True: raise SystemExit('terminal backend')
completed = backend.get('completed_counts')
if not isinstance(completed, dict) or set(completed) != set(expected) or any(isinstance(completed.get(key), bool) or not isinstance(completed.get(key), int) or completed[key] < 0 for key in expected): raise SystemExit('terminal completed counts')
buffers = value.get('buffers')
if not isinstance(buffers, dict) or set(buffers) != set(expected): raise SystemExit('terminal buffers')
if any(value.get(key) != {'lidar': 0, 'imu': 0, 'image': 0} for key in ('dropped_counts', 'overflow_counts')) or value.get('processing_failures') != 0: raise SystemExit('terminal failure counters')
support = value.get('terminal_support_context')
if not isinstance(support, dict) or support.get('classification') != 'nonlidar_at_or_after_boundary_support_context' or support.get('by_topic', {}).get('lidar') != 0: raise SystemExit('v5 terminal support classification')
for record in support.get('records', []):
    if record.get('topic') == 'lidar' or record.get('support_context_proven') is not True or record.get('can_form_synchronization_unit') is not False: raise SystemExit('terminal support record')
    equal = record.get('equal_to_completed_boundary', False)
    if equal and record.get('reason_code') != 'equal_to_completed_boundary_nonlidar_support_context': raise SystemExit('equal support reason')
    if not equal and record.get('reason_code') != 'strictly_after_completed_boundary': raise SystemExit('strict support reason')
trajectory = value.get('trajectory'); gap = trajectory.get('end_gap_seconds') if isinstance(trajectory, dict) else None; last = trajectory.get('last_timestamp_seconds') if isinstance(trajectory, dict) else None
if not isinstance(trajectory, dict) or trajectory.get('coverage_verified') is not True or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x)) for x in (gap, last)) or gap < 0 or gap > maximum or abs(required - last - gap) > 1e-9: raise SystemExit('terminal gap')
observation = value.get('terminal_observation')
if not isinstance(observation, dict) or observation.get('eof_observed') is not True or observation.get('stable') is not True or observation.get('identical_snapshot') is not True or observation.get('sync_predicate_evaluated') is not True or observation.get('poll_count', 0) < 2 or observation.get('stable_poll_count', 0) < 2: raise SystemExit('terminal polling')
print(hashlib.sha256(raw).hexdigest())
PY
)"
printf '%s\n' "${RAW_EVIDENCE_SHA256}" >"${OUT_DIR}/consumer_evidence.sha256"

# timing is written only after callback and terminal raw validation/hash.
DRAIN_END_MONOTONIC_NS="$(python3 -c 'import time; print(time.monotonic_ns())')"
(( DRAIN_END_MONOTONIC_NS > INPUT_START_MONOTONIC_NS )) || fail 'timing boundary is not increasing'
python3 - "${ONLINE_TIMING}" "${ONLINE_TIMING}.part" "${INPUT_START_MONOTONIC_NS}" "${DRAIN_END_MONOTONIC_NS}" "${M6A10_SENSOR_DURATION_SECONDS}" "${M6A10_TIMING_CONTRACT_VERSION}" <<'PY'
import json, math, os, sys
from pathlib import Path
final, part = Path(sys.argv[1]), Path(sys.argv[2]); start, end = int(sys.argv[3]), int(sys.argv[4]); sensor = float(sys.argv[5]); contract = sys.argv[6]
if final.exists() or final.is_symlink() or part.exists() or part.is_symlink(): raise SystemExit('timing exists')
if end <= start or not math.isfinite(sensor) or sensor <= 0: raise SystemExit('timing bounds')
duration = (end - start) / 1e9
rtf = duration / sensor
if not math.isfinite(rtf) or rtf < 0: raise SystemExit('timing RTF')
value = {'schema_version': 1, 'contract_version': contract, 'status': 'PASS', 'boundary': 'input_start_to_drain_end', 'input_start_monotonic_ns': start, 'drain_end_monotonic_ns': end, 'start_monotonic_ns': start, 'end_monotonic_ns': end, 'duration_seconds': duration, 'sensor_duration_seconds': sensor, 'online_compute_rtf': rtf, 'ground_truth_content_opened': False, 'scorer_invoked': False}
payload = (json.dumps(value, sort_keys=True, indent=2) + '\n').encode(); part.parent.mkdir(parents=True, exist_ok=True); fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
try:
    with os.fdopen(fd, 'wb') as stream: stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.link(part, final)
finally:
    try: part.unlink()
    except FileNotFoundError: pass
PY

stop_group "${ODOM_PID}"; wait "${ODOM_PID}" >/dev/null 2>&1 || true; ODOM_PID=''
stop_group "${MAPPER_PID}"; wait "${MAPPER_PID}" >/dev/null 2>&1 || true; MAPPER_PID=''
stop_group "${ROSCORE_PID}"; wait "${ROSCORE_PID}" >/dev/null 2>&1 || true; ROSCORE_PID=''
printf '%s\n' 'PASS' >"${OUT_DIR}/container_run_status.txt"
exit 0
