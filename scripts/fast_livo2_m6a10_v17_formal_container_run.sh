#!/usr/bin/env bash
# FAST-LIVO2 M6A10 v17 formal-ready wrapper (unauthorized candidate).
#
# v17 changes only the production synchronization retry boundary.  Empty
# queue retries are non-sticky; an acquired record, explicit discard, or
# estimator failure remains fail-closed.  This wrapper is deliberately
# authorization-gated and is not a replay authorization.
set -Eeuo pipefail

EXPECTED_PHASE_CONTRACT='m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
EXPECTED_PHASE_MODE='unpaced_ack'
EXPECTED_TRANSPORT_CONTRACT='m6a10-v12-callback-ack-transport-outstanding-v1'
EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml'
EXPECTED_PATCH_SHA256='c9254497b21846b2de6a028aa5fd60927a80da641c2fb47943c49ff3636ca001'
EXPECTED_FEEDER_SHA256='6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7'
EXPECTED_MESSAGES='236687'
EXPECTED_LIDAR_MESSAGES='5793'
EXPECTED_IMU_MESSAGES='225102'
EXPECTED_IMAGE_MESSAGES='5792'
EXPECTED_END_TIMESTAMP='1623491515.148352'
EXPECTED_END_GAP='0.25'
EXPECTED_MIN_POLL='0.05'
EXPECTED_SENSOR_DURATION='579.278127298'
EXPECTED_TIMING_CONTRACT='m6a10-online-compute-v3-timing-v1'
EXPECTED_TRANSPORT_LIMIT='1'

fail() { echo "FAST-LIVO2 v17 wrapper fail-closed: $*" >&2; return 1; }
OUT_DIR="${OUT_DIR:-/out}"
PROFILE_PATH="${M6A10_PROFILE_PATH:?M6A10_PROFILE_PATH is required}"
PROFILE_SHA256="${M6A10_PROFILE_SHA256:?M6A10_PROFILE_SHA256 is required}"
M6A10_PHASE_CONTRACT_VERSION="${M6A10_PHASE_CONTRACT_VERSION:?M6A10_PHASE_CONTRACT_VERSION is required}"
M6A10_PHASE_MODE="${M6A10_PHASE_MODE:?M6A10_PHASE_MODE is required}"
M6A10_FAST_FEEDER_SHA256="${M6A10_FAST_FEEDER_SHA256:?M6A10_FAST_FEEDER_SHA256 is required}"
M6A10_CONSUMER_EVIDENCE="${M6A10_CONSUMER_EVIDENCE:-${OUT_DIR}/callback_consumer_evidence.json}"
M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE="${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE:-${OUT_DIR}/consumer_evidence.json}"
M6A10_ONLINE_TIMING_EVIDENCE="${M6A10_ONLINE_TIMING_EVIDENCE:-${OUT_DIR}/online_compute_timing.json}"
M6A10_SENSOR_DURATION_SECONDS="${M6A10_SENSOR_DURATION_SECONDS:-}"
M6A10_TIMING_CONTRACT_VERSION="${M6A10_TIMING_CONTRACT_VERSION:-}"

[[ "${M6A10_PHASE_CONTRACT_VERSION}" == "${EXPECTED_PHASE_CONTRACT}" ]] || fail 'phase contract'
[[ "${M6A10_PHASE_MODE}" == "${EXPECTED_PHASE_MODE}" ]] || fail 'phase mode'
[[ "${PROFILE_PATH}" == "${EXPECTED_PROFILE_PATH}" ||
   "${PROFILE_PATH}" == "/runner/${EXPECTED_PROFILE_PATH}" ]] || fail 'profile path'
PROFILE_FILE="/runner/${EXPECTED_PROFILE_PATH}"
[[ -f "${PROFILE_FILE}" && ! -L "${PROFILE_FILE}" ]] || fail 'profile file'
[[ "$(sha256sum "${PROFILE_FILE}" | awk '{print $1}')" == "${PROFILE_SHA256}" ]] || fail 'profile SHA'
[[ "${M6A10_FAST_FEEDER_SHA256}" == "${EXPECTED_FEEDER_SHA256}" ]] || fail 'feeder SHA'
[[ "${M6A10_SENSOR_DURATION_SECONDS}" == "${EXPECTED_SENSOR_DURATION}" ]] || fail 'sensor duration'
[[ "${M6A10_TIMING_CONTRACT_VERSION}" == "${EXPECTED_TIMING_CONTRACT}" ]] || fail 'timing contract'
[[ "${M6A10_CONSUMER_EVIDENCE}" != "${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE}" ]] || fail 'evidence alias'
[[ "${M6A10_V17_AUTHORIZATION_STATUS:-UNAUTHORIZED_NOT_RUN}" == "AUTHORIZED_FOR_EXACT_ROOT" ]] ||
  fail 'formal authorization is not installed for this candidate'

OUT_DIR="$(cd "${OUT_DIR}" && pwd -P)"
FEEDER_PATH='/runner/scripts/fast_livo2_m6a10_feeder.py'
[[ -f "${FEEDER_PATH}" && ! -L "${FEEDER_PATH}" ]] || fail 'feeder missing'
[[ "$(sha256sum "${FEEDER_PATH}" | awk '{print $1}')" == "${EXPECTED_FEEDER_SHA256}" ]] || fail 'feeder bytes'
for path in "${M6A10_CONSUMER_EVIDENCE}" "${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE}" "${M6A10_ONLINE_TIMING_EVIDENCE}"; do
  [[ ! -e "${path}" && ! -L "${path}" && ! -e "${path}.part" && ! -L "${path}.part" ]] || fail 'evidence overwrite'
done

export ROS_MASTER_URI='http://127.0.0.1:11311' ROS_IP='127.0.0.1' ROS_HOSTNAME='127.0.0.1'
export ROS_HOME="${OUT_DIR}/ros_home" ROS_LOG_DIR="${OUT_DIR}/ros_logs"
export M6A10_PHASE_CONTRACT_VERSION M6A10_PHASE_MODE M6A10_PROFILE_PATH M6A10_PROFILE_SHA256
export M6A10_CONSUMER_EVIDENCE M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE M6A10_ONLINE_TIMING_EVIDENCE
export M6A10_SENSOR_DURATION_SECONDS M6A10_TIMING_CONTRACT_VERSION
export M6A10_FAST_EXPECTED_MESSAGES="${EXPECTED_MESSAGES}"
export M6A10_FAST_EXPECTED_LIDAR_MESSAGES="${EXPECTED_LIDAR_MESSAGES}"
export M6A10_FAST_EXPECTED_IMU_MESSAGES="${EXPECTED_IMU_MESSAGES}"
export M6A10_FAST_EXPECTED_IMAGE_MESSAGES="${EXPECTED_IMAGE_MESSAGES}"
export M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS="${EXPECTED_END_TIMESTAMP}"
export M6A10_FAST_MAX_END_GAP_SECONDS="${EXPECTED_END_GAP}"
export M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS="${EXPECTED_MIN_POLL}"
export M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS='0.25'
export M6A10_FAST_MAX_BACKLOG_MESSAGES="${EXPECTED_TRANSPORT_LIMIT}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

ROSCORE_PID=''; MAPPER_PID=''; FEEDER_PID=''
stop_pid() { local pid="${1:-}"; [[ -n "${pid}" ]] || return 0; kill -TERM "${pid}" >/dev/null 2>&1 || true; }
cleanup() {
  local status=$?; trap - EXIT INT TERM; set +e
  stop_pid "${FEEDER_PID}"; stop_pid "${MAPPER_PID}"; stop_pid "${ROSCORE_PID}"
  for pid in "${FEEDER_PID}" "${MAPPER_PID}" "${ROSCORE_PID}"; do
    [[ -n "${pid}" ]] && wait "${pid}" >/dev/null 2>&1 || true
  done
  exit "${status}"
}
trap cleanup EXIT; trap 'exit 130' INT; trap 'exit 143' TERM

source /opt/ros/noetic/setup.bash
source /opt/fast_livo_ws/devel/setup.bash
roscore >"${OUT_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!
master_deadline=$((SECONDS + 60))
master_ready=0
while (( SECONDS < master_deadline )); do
  if rosparam list >/dev/null 2>&1; then master_ready=1; break; fi
  sleep 0.1
done
(( master_ready == 1 )) || fail 'roscore not ready'
rosparam set use_sim_time true
roslaunch fast_livo mapping_ouster_ntu.launch rviz:=false >"${OUT_DIR}/mapper.log" 2>&1 &
MAPPER_PID=$!

required_services=(
  /m6a10/consumer_status /m6a10/consumer_ack /m6a10/consumer_eof
  /m6a10/consumer_finalize /m6a10/terminal_status /m6a10/terminal_eof
  /m6a10/terminal_finalize)
service_deadline=$((SECONDS + 60)); services_ready=0
while (( SECONDS < service_deadline )); do
  services_ready=1
  for service in "${required_services[@]}"; do
    if ! rosservice list 2>/dev/null | grep -Fxq "${service}"; then services_ready=0; break; fi
  done
  (( services_ready == 1 )) && break
  kill -0 "${MAPPER_PID}" >/dev/null 2>&1 || fail 'mapper exited before services'
  sleep 0.2
done
(( services_ready == 1 )) || fail 'seven services missing'

set +e
python3 "${FEEDER_PATH}" --bag "${M6A10_BAG_PATH:?M6A10_BAG_PATH is required}" \
  --output "${OUT_DIR}/feeder_receipt.json" --timeout 1200 \
  --expected-lidar "${EXPECTED_LIDAR_MESSAGES}" --expected-imu "${EXPECTED_IMU_MESSAGES}" \
  --expected-image "${EXPECTED_IMAGE_MESSAGES}" >"${OUT_DIR}/feeder.log" 2>&1 &
FEEDER_PID=$!
wait "${FEEDER_PID}"; feeder_status=$?; FEEDER_PID=''
set -e
(( feeder_status == 0 )) || fail 'feeder failed'

call_trigger() {
  local service="${1:?service}"; local response="${2:?response}"
  timeout 10 rosservice call "${service}" "{}" >"${response}" 2>"${response}.err" || return 1
  grep -Fq 'success: True' "${response}"
}
call_trigger /m6a10/consumer_eof "${OUT_DIR}/consumer_eof.response" || fail 'consumer EOF'
call_trigger /m6a10/consumer_status "${OUT_DIR}/consumer_status.response" || fail 'consumer status'
call_trigger /m6a10/consumer_finalize "${OUT_DIR}/consumer_finalize.response" || fail 'consumer finalize'
test -s "${M6A10_CONSUMER_EVIDENCE}" || fail 'callback evidence missing'

call_trigger /m6a10/terminal_eof "${OUT_DIR}/terminal_eof.response" || fail 'terminal EOF'
call_trigger /m6a10/terminal_status "${OUT_DIR}/terminal_status_1.response" || fail 'terminal status 1'
sleep "${EXPECTED_MIN_POLL}"
call_trigger /m6a10/terminal_status "${OUT_DIR}/terminal_status_2.response" || fail 'terminal status 2'
call_trigger /m6a10/terminal_finalize "${OUT_DIR}/terminal_finalize.response" || fail 'terminal finalize'
test -s "${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE}" || fail 'terminal evidence missing'
test -s "${M6A10_ONLINE_TIMING_EVIDENCE}" || fail 'timing evidence missing'
