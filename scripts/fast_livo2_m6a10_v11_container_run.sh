#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.

# Inside-container FAST-LIVO2 M6a10 v11 runtime entrypoint.  The host binds
# only the pinned input and output; all application evidence remains owned by
# the mapper or the inherited single-inflight feeder.
set -Eeuo pipefail

EXPECTED_CONTRACT='m6a10-online-compute-v4-terminal-bounded-end-gap'
EXPECTED_PHASE_MODE='unpaced_ack'
EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/fast_livo2_m6a10_v11_formal.yaml'
EXPECTED_PROFILE_SHA256='e8980198e52604bb7dca0d1c1ecb91b6ce4f8b95b1dea3bb80ad4db8cd382296'
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
EXPECTED_BACKLOG='1'
EXPECTED_SENSOR_DURATION='579.278127298'
EXPECTED_TIMING_CONTRACT='m6a10-online-compute-v3-timing-v1'
EXPECTED_FEEDER_SHA256='1ea1c9bb9c625795c51c4b8c4b6aa81364370da17f8cac562f46d00af9691831'
FEEDER_PATH='/runner/scripts/fast_livo2_m6a10_feeder.py'

fail() {
  echo "FAST-LIVO2 v10 container gate failed: $*" >&2
  return 1
}

OUT_DIR="${OUT_DIR:-/out}"
BAG_PATH="${BAG_PATH:-${M6A10_BAG_PATH:-}}"
[[ -n "${BAG_PATH}" ]] || fail 'BAG_PATH is required'
PROFILE_SHA256="${M6A10_PROFILE_SHA256:?M6A10_PROFILE_SHA256 is required}"
PROFILE_PATH="${M6A10_PROFILE_PATH:?M6A10_PROFILE_PATH is required}"
BAG_BYTES="${M6A10_BAG_BYTES:-${M6A10_EXPECTED_BAG_BYTES:-}}"
BAG_SHA256="${M6A10_BAG_SHA256:-${M6A10_EXPECTED_BAG_SHA256:-}}"
M6A10_FAST_EXPECTED_MESSAGES="${M6A10_FAST_EXPECTED_MESSAGES:-${M6A10_EXPECTED_MESSAGES:-}}"
M6A10_FAST_EXPECTED_LIDAR_MESSAGES="${M6A10_FAST_EXPECTED_LIDAR_MESSAGES:-${M6A10_EXPECTED_LIDAR_MESSAGES:-}}"
M6A10_FAST_EXPECTED_IMU_MESSAGES="${M6A10_FAST_EXPECTED_IMU_MESSAGES:-${M6A10_EXPECTED_IMU_MESSAGES:-}}"
M6A10_FAST_EXPECTED_IMAGE_MESSAGES="${M6A10_FAST_EXPECTED_IMAGE_MESSAGES:-${M6A10_EXPECTED_IMAGE_MESSAGES:-}}"
[[ -n "${BAG_BYTES}" && -n "${BAG_SHA256}" &&
   -n "${M6A10_FAST_EXPECTED_MESSAGES}" &&
   -n "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" &&
   -n "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" &&
   -n "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" ]] || fail 'bag/count environment is incomplete'
M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS="${M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS:-${M6A10_REQUIRED_END_TIMESTAMP_SECONDS:-}}"
M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE="${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE:-}"
M6A10_CONSUMER_EVIDENCE="${M6A10_CONSUMER_EVIDENCE:-}"
M6A10_ONLINE_TIMING_EVIDENCE="${M6A10_ONLINE_TIMING_EVIDENCE:-}"
M6A10_SENSOR_DURATION_SECONDS="${M6A10_SENSOR_DURATION_SECONDS:-}"
M6A10_TIMING_CONTRACT_VERSION="${M6A10_TIMING_CONTRACT_VERSION:-}"
M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS="${M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS:-${EXPECTED_CALLBACK_LATENCY}}"
M6A10_FAST_MAX_BACKLOG_MESSAGES="${M6A10_FAST_MAX_BACKLOG_MESSAGES:-${EXPECTED_BACKLOG}}"
[[ -n "${M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS}" &&
   -n "${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE}" &&
   -n "${M6A10_CONSUMER_EVIDENCE}" &&
   -n "${M6A10_ONLINE_TIMING_EVIDENCE}" &&
   -n "${M6A10_SENSOR_DURATION_SECONDS}" &&
   -n "${M6A10_TIMING_CONTRACT_VERSION}" ]] || fail 'end/evidence/timing environment is incomplete'
M6A10_FAST_FEEDER_SHA256="${M6A10_FAST_FEEDER_SHA256:?M6A10_FAST_FEEDER_SHA256 is required}"
M6A10_PHASE_CONTRACT_VERSION="${M6A10_PHASE_CONTRACT_VERSION:?M6A10_PHASE_CONTRACT_VERSION is required}"
M6A10_PHASE_MODE="${M6A10_PHASE_MODE:?M6A10_PHASE_MODE is required}"

[[ "${M6A10_PHASE_CONTRACT_VERSION}" == "${EXPECTED_CONTRACT}" ]] || fail 'wrong phase contract'
[[ "${M6A10_PHASE_MODE}" == "${EXPECTED_PHASE_MODE}" ]] || fail 'wrong phase mode'
[[ "${PROFILE_SHA256}" == "${EXPECTED_PROFILE_SHA256}" ]] || fail 'profile SHA mismatch'
case "${PROFILE_PATH}" in
  "${EXPECTED_PROFILE_PATH}"|"/runner/${EXPECTED_PROFILE_PATH}") ;;
  *) fail 'profile path mismatch' ;;
esac
[[ "${BAG_PATH}" == "${EXPECTED_BAG_PATH}" ]] || fail 'bag path mismatch'
[[ "${BAG_BYTES}" == "${EXPECTED_BAG_BYTES}" ]] || fail 'bag byte count mismatch'
[[ "${BAG_SHA256}" == "${EXPECTED_BAG_SHA256}" ]] || fail 'bag SHA mismatch'
[[ "${M6A10_FAST_EXPECTED_MESSAGES}" == "${EXPECTED_MESSAGES}" ]] || fail 'total count mismatch'
[[ "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" == "${EXPECTED_LIDAR_MESSAGES}" ]] || fail 'LiDAR count mismatch'
[[ "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" == "${EXPECTED_IMU_MESSAGES}" ]] || fail 'IMU count mismatch'
[[ "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" == "${EXPECTED_IMAGE_MESSAGES}" ]] || fail 'image count mismatch'
[[ "${M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS}" == "${EXPECTED_END_TIMESTAMP}" ]] || fail 'required end mismatch'
[[ "${M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS}" == "${EXPECTED_CALLBACK_LATENCY}" ]] ||
  fail 'callback latency bound mismatch'
[[ "${M6A10_FAST_MAX_BACKLOG_MESSAGES}" == "${EXPECTED_BACKLOG}" ]] ||
  fail 'callback backlog bound mismatch'
[[ "${M6A10_SENSOR_DURATION_SECONDS}" == "${EXPECTED_SENSOR_DURATION}" ]] ||
  fail 'sensor duration mismatch'
[[ "${M6A10_TIMING_CONTRACT_VERSION}" == "${EXPECTED_TIMING_CONTRACT}" ]] ||
  fail 'timing contract mismatch'

M6A10_FAST_MAX_END_GAP_SECONDS="${M6A10_FAST_MAX_END_GAP_SECONDS:-${EXPECTED_END_GAP}}"
M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS="${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS:-${EXPECTED_MIN_POLL}}"
[[ "${M6A10_FAST_MAX_END_GAP_SECONDS}" == "${EXPECTED_END_GAP}" ]] || fail 'end gap mismatch'
[[ "${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS}" == "${EXPECTED_MIN_POLL}" ]] || fail 'terminal poll gap mismatch'
[[ "${M6A10_FAST_FEEDER_SHA256}" == "${EXPECTED_FEEDER_SHA256}" ]] || fail 'feeder SHA mismatch'

mkdir -p "${OUT_DIR}"
OUT_DIR="$(cd "${OUT_DIR}" && pwd -P)"
RAW_EVIDENCE="${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE}"
CALLBACK_EVIDENCE="${M6A10_CONSUMER_EVIDENCE}"
ONLINE_TIMING="${M6A10_ONLINE_TIMING_EVIDENCE}"
[[ "${RAW_EVIDENCE}" == "${OUT_DIR}/consumer_evidence.json" ]] || fail 'raw evidence path mismatch'
[[ "${CALLBACK_EVIDENCE}" == "${OUT_DIR}/callback_consumer_evidence.json" ]] ||
  fail 'callback evidence path mismatch'
[[ "${CALLBACK_EVIDENCE}" != "${RAW_EVIDENCE}" ]] || fail 'evidence paths must be distinct'
[[ "${ONLINE_TIMING}" == "${OUT_DIR}/online_compute_timing.json" ]] ||
  fail 'online timing path mismatch'
for evidence_path in "${RAW_EVIDENCE}" "${CALLBACK_EVIDENCE}" "${ONLINE_TIMING}"; do
  if [[ -e "${evidence_path}" || -L "${evidence_path}" ||
        -e "${evidence_path}.part" || -L "${evidence_path}.part" ]]; then
    fail 'evidence output already exists or is staging'
  fi
done
[[ -f "${FEEDER_PATH}" && ! -L "${FEEDER_PATH}" ]] || fail 'feeder missing'
[[ "$(sha256sum "${FEEDER_PATH}" | awk '{print $1}')" == "${EXPECTED_FEEDER_SHA256}" ]] ||
  fail 'feeder bytes mismatch'

export ROS_MASTER_URI='http://127.0.0.1:11311'
export ROS_IP='127.0.0.1' ROS_HOSTNAME='127.0.0.1'
export ROS_HOME="${OUT_DIR}/ros_home" ROS_LOG_DIR="${OUT_DIR}/ros_logs"
export M6A10_PHASE_CONTRACT_VERSION M6A10_PHASE_MODE
export M6A10_CONSUMER_EVIDENCE="${CALLBACK_EVIDENCE}"
export M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE="${RAW_EVIDENCE}"
export M6A10_ONLINE_TIMING_EVIDENCE="${ONLINE_TIMING}"
export M6A10_SENSOR_DURATION_SECONDS
export M6A10_TIMING_CONTRACT_VERSION
export M6A10_FAST_EXPECTED_MESSAGES
export M6A10_FAST_EXPECTED_LIDAR_MESSAGES
export M6A10_FAST_EXPECTED_IMU_MESSAGES
export M6A10_FAST_EXPECTED_IMAGE_MESSAGES
export M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS
export M6A10_FAST_MAX_BACKLOG_MESSAGES
export M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS
export M6A10_FAST_MAX_END_GAP_SECONDS M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

ROSCORE_PID=''
MAPPER_PID=''
ODOM_PID=''
FEEDER_PID=''

stop_group() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] || return 0
  kill -TERM -- "-${pid}" >/dev/null 2>&1 || true
  kill -TERM "${pid}" >/dev/null 2>&1 || true
}

cleanup() {
  local exit_status=$?
  trap - EXIT INT TERM
  set +e
  stop_group "${FEEDER_PID}"
  stop_group "${ODOM_PID}"
  stop_group "${MAPPER_PID}"
  stop_group "${ROSCORE_PID}"
  for child_pid in "${FEEDER_PID}" "${ODOM_PID}" "${MAPPER_PID}" "${ROSCORE_PID}"; do
    [[ -n "${child_pid}" ]] && wait "${child_pid}" >/dev/null 2>&1 || true
  done
  exit "${exit_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

source /opt/ros/noetic/setup.bash
source /opt/fast_livo_ws/devel/setup.bash

setsid roscore >"${OUT_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!
master_deadline=$((SECONDS + ${M6A10_STARTUP_TIMEOUT_SECONDS:-60}))
master_ready=0
while (( SECONDS < master_deadline )); do
  if rosparam list >/dev/null 2>&1; then
    master_ready=1
    break
  fi
  sleep 0.1
done
printf '%s\n' "${master_ready}" >"${OUT_DIR}/master_ready.txt"
(( master_ready == 1 )) || fail 'ROS master did not become ready'
rosparam set use_sim_time true

setsid roslaunch fast_livo mapping_ouster_ntu.launch rviz:=false \
  >"${OUT_DIR}/mapper.log" 2>&1 &
MAPPER_PID=$!

required_services=(
  /m6a10/consumer_status /m6a10/consumer_ack
  /m6a10/terminal_status /m6a10/terminal_eof /m6a10/terminal_finalize
)
service_deadline=$((SECONDS + ${M6A10_STARTUP_TIMEOUT_SECONDS:-60}))
services_ready=0
while (( SECONDS < service_deadline )); do
  services_ready=1
  for service in "${required_services[@]}"; do
    if ! rosservice list 2>/dev/null | grep -Fxq "${service}"; then
      services_ready=0
      break
    fi
  done
  (( services_ready == 1 )) && break
  kill -0 "${MAPPER_PID}" >/dev/null 2>&1 || fail 'mapper exited before services'
  sleep 0.2
done
printf '%s\n' "${services_ready}" >"${OUT_DIR}/services_ready.txt"
(( services_ready == 1 )) || fail 'required services did not become ready'

setsid rostopic echo -p /aft_mapped_to_init \
  >"${OUT_DIR}/odometry.csv" 2>"${OUT_DIR}/odometry.err" &
ODOM_PID=$!

INPUT_START_MONOTONIC_NS="$(python3 -c 'import time; print(time.monotonic_ns())')"
[[ "${INPUT_START_MONOTONIC_NS}" =~ ^[0-9]+$ ]] || fail 'input start timestamp is invalid'
set +e
setsid python3 "${FEEDER_PATH}" \
  --bag "${BAG_PATH}" --output "${OUT_DIR}/feeder_receipt.json" \
  --timeout "${M6A10_FEEDER_TIMEOUT_SECONDS:-1200}" \
  --expected-lidar "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" \
  --expected-imu "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" \
  --expected-image "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" \
  >"${OUT_DIR}/feeder.log" 2>&1 &
FEEDER_PID=$!
wait "${FEEDER_PID}"
feeder_status=$?
FEEDER_PID=''
set -e
printf '%s\n' "${feeder_status}" >"${OUT_DIR}/feeder_exit_status.txt"
(( feeder_status == 0 )) || fail 'single-inflight feeder failed'

python3 - "${OUT_DIR}/feeder_receipt.json" "${BAG_PATH}" "${BAG_BYTES}" "${BAG_SHA256}" \
  "${M6A10_FAST_EXPECTED_MESSAGES}" "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" \
  "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" <<'PY'
import json
import sys

path, bag_path, bag_bytes, bag_sha, total, lidar, imu, image = sys.argv[1:]
with open(path, encoding='utf-8') as stream:
    value = json.load(stream)
expected = {'lidar': int(lidar), 'imu': int(imu), 'image': int(image)}
if value.get('status') != 'pass' or value.get('single_inflight') is not True:
    raise SystemExit('feeder receipt is not single-inflight PASS')
if value.get('bag_path') != bag_path or value.get('bag_bytes') != int(bag_bytes) or \
        value.get('bag_sha256') != bag_sha:
    raise SystemExit('feeder bag identity mismatch')
if value.get('published_messages') != int(total) or \
        value.get('expected_topic_counts') != expected or \
        value.get('published_topic_counts') != expected or \
        value.get('acked_topic_counts') != expected:
    raise SystemExit('feeder count mismatch')
if value.get('ack_backpressure_verified') is not True or \
        value.get('ground_truth_content_opened') is not False or \
        value.get('scorer_invoked') is not False:
    raise SystemExit('feeder safety/ACK contract mismatch')
PY

call_trigger() {
  local service="${1:?service required}"
  local response_path="${2:?response path required}"
  timeout "${M6A10_RPC_TIMEOUT_SECONDS:-10}" rosservice call "${service}" "{}" \
    >"${response_path}" 2>"${response_path}.err" || return 1
  python3 - "${response_path}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    response = yaml.safe_load(stream) or {}
if response.get('success') is not True:
    raise SystemExit(1)
PY
}

call_trigger /m6a10/terminal_eof "${OUT_DIR}/terminal_eof.response" ||
  fail 'terminal EOF service did not acknowledge'

terminal_status_count=0
terminal_status_successes=0
terminal_status_deadline=$((SECONDS + ${M6A10_TERMINAL_DRAIN_TIMEOUT_SECONDS:-120}))
while (( SECONDS < terminal_status_deadline )); do
  terminal_status_count=$((terminal_status_count + 1))
  if call_trigger /m6a10/terminal_status \
      "${OUT_DIR}/terminal_status_${terminal_status_count}.response"; then
    terminal_status_successes=$((terminal_status_successes + 1))
    if (( terminal_status_successes >= 2 )); then
      break
    fi
  fi
  sleep "${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS}"
done
(( terminal_status_successes >= 2 )) || fail 'fewer than two terminal status RPCs succeeded'

call_trigger /m6a10/terminal_finalize "${OUT_DIR}/terminal_finalize.response" ||
  fail 'terminal finalize service did not acknowledge'

RAW_EVIDENCE_SHA256="$(python3 - "${RAW_EVIDENCE}" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
if path.is_symlink() or not path.is_file():
    raise SystemExit('raw evidence is not a regular file')
payload = path.read_bytes()
try:
    value = json.loads(payload.decode('utf-8'))
except (UnicodeError, json.JSONDecodeError) as error:
    raise SystemExit('raw evidence JSON is invalid') from error
if value.get('schema_version') != 1 or \
        value.get('contract_id') != 'm6a10-fast-livo2-consumer-terminal-v1' or \
        value.get('status') != 'pass':
    raise SystemExit('raw evidence schema/contract/status mismatch')
counts = value.get('received_topic_counts')
if not isinstance(counts, dict) or set(counts) != {'lidar', 'imu', 'image'} or \
        any(isinstance(item, bool) or not isinstance(item, int) or item < 0
            for item in counts.values()):
    raise SystemExit('raw evidence received counts are invalid')
if value.get('ground_truth_content_opened') is not False or \
        value.get('scorer_invoked') is not False:
    raise SystemExit('raw evidence safety flags are invalid')
for forbidden in (
        'input', 'profile_sha256', 'expected', 'expected_messages',
        'expected_topic_counts', 'published', 'published_messages',
        'published_topic_counts', 'ack', 'acked', 'acked_messages',
        'acked_topic_counts', 'acknowledged', 'acknowledged_messages',
        'acknowledged_topic_counts'):
    if forbidden in value:
        raise SystemExit('raw evidence contains authority fields')
print(hashlib.sha256(payload).hexdigest())
PY
)"
DRAIN_END_MONOTONIC_NS="$(python3 -c 'import time; print(time.monotonic_ns())')"
[[ "${DRAIN_END_MONOTONIC_NS}" =~ ^[0-9]+$ ]] || fail 'drain end timestamp is invalid'
(( DRAIN_END_MONOTONIC_NS > INPUT_START_MONOTONIC_NS )) ||
  fail 'online timing boundary is not increasing'

python3 - "${ONLINE_TIMING}" "${ONLINE_TIMING}.part" \
  "${INPUT_START_MONOTONIC_NS}" "${DRAIN_END_MONOTONIC_NS}" \
  "${M6A10_SENSOR_DURATION_SECONDS}" "${M6A10_TIMING_CONTRACT_VERSION}" <<'PY'
import json
import math
import os
from pathlib import Path
import sys

final_path = Path(sys.argv[1])
part_path = Path(sys.argv[2])
start = int(sys.argv[3])
end = int(sys.argv[4])
sensor_duration = float(sys.argv[5])
contract = sys.argv[6]
if final_path.exists() or final_path.is_symlink() or \
        part_path.exists() or part_path.is_symlink():
    raise SystemExit('online timing output already exists or is staging')
if end <= start or not math.isfinite(sensor_duration) or sensor_duration <= 0.0:
    raise SystemExit('online timing bounds are invalid')
duration = (end - start) / 1.0e9
rtf = duration / sensor_duration
if not math.isfinite(duration) or duration < 0.0 or not math.isfinite(rtf) or rtf < 0.0:
    raise SystemExit('online timing metric is invalid')
value = {
    'schema_version': 1,
    'contract_version': contract,
    'status': 'PASS',
    'boundary': 'input_start_to_drain_end',
    'input_start_monotonic_ns': start,
    'drain_end_monotonic_ns': end,
    'start_monotonic_ns': start,
    'end_monotonic_ns': end,
    'duration_seconds': duration,
    'sensor_duration_seconds': sensor_duration,
    'online_compute_rtf': rtf,
    'ground_truth_content_opened': False,
    'scorer_invoked': False,
}
payload = (json.JSONEncoder(indent=2, sort_keys=True).encode(value) + '\n').encode('utf-8')
part_path.parent.mkdir(parents=True, exist_ok=True)
try:
    fd = os.open(part_path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
except FileExistsError as error:
    raise SystemExit('online timing staging file already exists') from error
try:
    with os.fdopen(fd, 'wb') as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    try:
        os.link(part_path, final_path)
    except FileExistsError as error:
        raise SystemExit('online timing output appeared during seal') from error
finally:
    try:
        part_path.unlink()
    except FileNotFoundError:
        pass
PY
printf '%s\n' "${RAW_EVIDENCE_SHA256}" >"${OUT_DIR}/consumer_evidence.sha256"

stop_group "${ODOM_PID}"
wait "${ODOM_PID}" >/dev/null 2>&1 || true
ODOM_PID=''
stop_group "${MAPPER_PID}"
wait "${MAPPER_PID}" >/dev/null 2>&1 || true
MAPPER_PID=''
stop_group "${ROSCORE_PID}"
wait "${ROSCORE_PID}" >/dev/null 2>&1 || true
ROSCORE_PID=''

printf '%s\n' 'PASS' >"${OUT_DIR}/container_run_status.txt"
exit 0
