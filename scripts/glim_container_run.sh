#!/usr/bin/env bash
set -uo pipefail

BAG_PATH="${BAG_PATH:?BAG_PATH is required}"
OUT_DIR="${OUT_DIR:-/out}"
GLIM_PROFILE="${GLIM_PROFILE:-hilti2022_cpu}"
OVERRIDE_CONFIG="${OVERRIDE_CONFIG:-/runner/configs/glim/${GLIM_PROFILE}}"

if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
  [[ "${GLIM_PROFILE}" == "ntu_viral_cpu" ]] || {
    echo 'GLIM v2 requires the preregistered ntu_viral_cpu profile' >&2
    exit 2
  }
  [[ "${M6A10_PHASE_MODE:-}" == "paced_1x" ||
     "${M6A10_PHASE_MODE:-}" == "unpaced_ack" ]] || {
    echo 'GLIM v2 requires paced_1x or unpaced_ack' >&2
    exit 2
  }
  : "${M6A10_GLIM_EXPECTED_MESSAGES:?GLIM v2 expected message count is required}"
  : "${M6A10_GLIM_EXPECTED_IMU_MESSAGES:?GLIM v2 expected IMU count is required}"
  : "${M6A10_GLIM_EXPECTED_POINTS_MESSAGES:?GLIM v2 expected points count is required}"
  : "${M6A10_GLIM_EXPECTED_IMAGE_MESSAGES:?GLIM v2 expected image count is required}"
  : "${M6A10_GLIM_REQUIRED_END_TIMESTAMP_SECONDS:?GLIM v2 required end timestamp is required}"
  : "${M6A10_GLIM_MAX_CALLBACK_LATENCY_NS:?GLIM v2 callback latency bound is required}"
  : "${M6A10_GLIM_MAX_CALLBACK_LATENCY_SECONDS:?GLIM v2 callback latency seconds bound is required}"
  : "${M6A10_GLIM_MAX_BACKLOG_MESSAGES:?GLIM v2 backlog bound is required}"
  [[ "${M6A10_GLIM_BUILD_WITH_CV_BRIDGE:-}" == "ON" ]] || {
    echo 'GLIM v2 requires BUILD_WITH_CV_BRIDGE=ON' >&2
    exit 2
  }
  export M6A10_ACK_BACKPRESSURE_ENABLED=1
fi

# The container root is intentionally read-only.  Keep all ROS state inside
# the attempt output mount; no host home or GT path is used.
export ROS_HOME="${ROS_HOME:-${OUT_DIR}/ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${OUT_DIR}/ros_log}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
source /runner/scripts/container_phase_evidence.sh
source /runner/scripts/container_memory_evidence.sh
trap 'm6a5_container_exit_trap "$?"' EXIT
m6a5_install_container_signal_traps
if ! m6a7_start_process_rss_sampler; then
  echo 'GLIM process RSS sampler failed to start' >&2
  exit 11
fi

GLIM_INPUT_DURATION="${M6A10_SENSOR_DURATION_SECONDS:-}"
if [[ -z "${GLIM_INPUT_DURATION}" ]]; then
  GLIM_INPUT_DURATION="$(m6a10_phase_ros2_bag_duration "${BAG_PATH}")" || {
    echo 'GLIM could not determine canonical bag duration for phase evidence' >&2
    exit 12
  }
fi
m6a10_phase_init glim_cpu "${GLIM_INPUT_DURATION}" || exit 12
M6A10_RESOURCE_REPORT="${OUT_DIR}/process_time.txt"
export M6A10_PHASE_EVENTS M6A10_PHASE_EVIDENCE M6A10_RESOURCE_REPORT

if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
  export M6A10_CONSUMER_EVIDENCE="${OUT_DIR}/consumer_evidence.json"
  export M6A10_CONSUMER_EOF_EVIDENCE="${M6A10_CONSUMER_EVIDENCE}.eof.json"
  export M6A10_CHILD_STARTUP_ENV="${OUT_DIR}/child_startup_env.json"
  startup_part="${M6A10_CHILD_STARTUP_ENV}.part"
  printf '%s\n' "{\"schema_version\":2,\"contract_version\":\"${M6A10_PHASE_CONTRACT_VERSION}\",\"phase_mode\":\"${M6A10_PHASE_MODE}\",\"system\":\"glim\",\"build_with_cv_bridge\":\"${M6A10_GLIM_BUILD_WITH_CV_BRIDGE}\"}" >"${startup_part}"
  mv -f "${startup_part}" "${M6A10_CHILD_STARTUP_ENV}"
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/glim_ws/install/setup.bash
set -u

mkdir -p "${OUT_DIR}"
rm -rf /tmp/glim_benchmark_config
cp -a /opt/glim_ws/src/glim/config /tmp/glim_benchmark_config
for override in config.json config_sensors.json config_ros.json config_logging.json; do
  cp "${OVERRIDE_CONFIG}/${override}" "/tmp/glim_benchmark_config/${override}"
done

if [[ "${M6A3_SYNTHETIC_SMOKE:-0}" == "1" ]]; then
  smoke_log="${OUT_DIR}/synthetic_smoke_glim.log"
  setsid ros2 run glim_ros glim_rosbag "${BAG_PATH}" --ros-args \
    -p config_path:=/tmp/glim_benchmark_config \
    -p auto_quit:=false \
    -p dump_path:="${OUT_DIR}/smoke_dump" >"${smoke_log}" 2>&1 &
  smoke_pid=$!
  started=0
  for _ in $(seq 1 "${SMOKE_STARTUP_TIMEOUT_SECS:-20}"); do
    if ! kill -0 "${smoke_pid}" >/dev/null 2>&1; then
      break
    fi
    # ros2 node list can wait indefinitely when DDS discovery is isolated by
    # network=none.  GLIM's own config+bag-open records are the stronger
    # startup contract for this process-level smoke.
    if grep -Fq "config_path: /tmp/glim_benchmark_config" "${smoke_log}" && \
       grep -Fq "opening ${BAG_PATH}" "${smoke_log}" && \
       grep -Fq "Opened database" "${smoke_log}"; then
      started=1
      break
    fi
    sleep 1
  done
  kill -INT -- "-${smoke_pid}" >/dev/null 2>&1 || true
  clean_shutdown=0
  for _ in $(seq 1 "${SMOKE_SHUTDOWN_TIMEOUT_SECS:-10}"); do
    if ! kill -0 "${smoke_pid}" >/dev/null 2>&1; then
      clean_shutdown=1
      break
    fi
    sleep 1
  done
  if [[ "${clean_shutdown}" != "1" ]]; then
    kill -TERM -- "-${smoke_pid}" >/dev/null 2>&1 || true
    sleep 1
  fi
  wait "${smoke_pid}" >/dev/null 2>&1 || true
  if [[ "${started}" != "1" || ! -s "${smoke_log}" ]]; then
    echo 'error: synthetic GLIM process did not reach an observable node' >&2
    tail -n 80 "${smoke_log}" >&2 || true
    exit 13
  fi
  if [[ "${clean_shutdown}" != "1" ]]; then
    echo 'error: synthetic GLIM process did not cleanly stop within timeout' >&2
    exit 13
  fi
  if grep -Fq "/root/.ros/log" "${smoke_log}"; then
    echo 'error: GLIM attempted to write ROS state outside the attempt output' >&2
    exit 13
  fi
  printf '%s\n' '{"status":"pass","startup_verified":true,"input_verified":true,"clean_shutdown":true,"gt_mounted":false,"performance_run":false}' >"${OUT_DIR}/synthetic_smoke_contract.json"
  exit 0
fi

if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" != "m6a10-online-compute-v2" ]]; then
  /usr/bin/time -v -o "${OUT_DIR}/process_time.txt" \
    ros2 run glim_ros glim_rosbag "${BAG_PATH}" --ros-args \
      -p config_path:=/tmp/glim_benchmark_config \
      -p auto_quit:=true \
      -p dump_path:="${OUT_DIR}/dump" \
    >"${OUT_DIR}/glim.log" 2>&1
  status=$?
  printf '%s\n' "${status}" >"${OUT_DIR}/process_exit_status.txt"
  exit "${status}"
fi

setsid bash /runner/scripts/run_with_resource_report.sh \
  "${OUT_DIR}/process_time.txt" \
  ros2 run glim_ros glim_rosbag "${BAG_PATH}" --ros-args \
    -p config_path:=/tmp/glim_benchmark_config \
    -p auto_quit:=true \
    -p dump_path:="${OUT_DIR}/dump" \
  >"${OUT_DIR}/glim.log" 2>&1 &
glim_pid=$!
input_started=0
input_ended=0
eof_sidecar_sha256=""
for _ in $(seq 1 "${GLIM_STARTUP_TIMEOUT_SECS:-120}"); do
  if grep -Fq "Opened database" "${OUT_DIR}/glim.log" 2>/dev/null; then
    m6a10_phase_mark startup_end || true
    m6a10_phase_mark input_start || true
    input_started=1
    break
  fi
  if ! kill -0 "${glim_pid}" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
if [[ "${input_started}" == 1 ]]; then
  if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
    glim_consumer_eof_state() {
      python3 /runner/scripts/benchmark_phase_contract.py consumer-eof-state \
          --input "${M6A10_CONSUMER_EOF_EVIDENCE}" >/dev/null
      return $?
    }
    glim_consumer_state() {
      python3 /runner/scripts/benchmark_phase_contract.py consumer-state \
          --input "${M6A10_CONSUMER_EVIDENCE}" \
          --diagnostic-base "${OUT_DIR}/m6a10_drain_diagnostic.json" \
          --maximum-callback-latency-seconds "${M6A10_GLIM_MAX_CALLBACK_LATENCY_SECONDS}" \
          --maximum-backlog-messages "${M6A10_GLIM_MAX_BACKLOG_MESSAGES}" >/dev/null
      return $?
    }
    # The application-owned atomic evidence, never log text, is the EOF and
    # acknowledgement boundary.  Invalid/timeout evidence is terminal and
    # must not be turned into a successful save.
    while kill -0 "${glim_pid}" >/dev/null 2>&1; do
      glim_consumer_eof_state
      eof_state=$?
      if [[ "${eof_state}" == 0 ]]; then
        current_eof_sha256="$(sha256sum "${M6A10_CONSUMER_EOF_EVIDENCE}" 2>/dev/null | awk '{print $1}')"
        if [[ -z "${current_eof_sha256}" ]]; then
          eof_state=1
        elif [[ -z "${eof_sidecar_sha256}" ]]; then
          eof_sidecar_sha256="${current_eof_sha256}"
        elif [[ "${current_eof_sha256}" != "${eof_sidecar_sha256}" ]]; then
          echo 'GLIM consumer EOF sidecar changed after publication' >&2
          eof_state=1
        fi
        if [[ "${eof_state}" == 0 && "${input_ended}" == 0 ]]; then
          m6a10_phase_mark input_end || true
          m6a10_phase_mark drain_start || true
          input_ended=1
        fi
      fi
      if [[ "${eof_state}" != 2 && "${eof_state}" != 0 ]]; then
        break
      fi
      if [[ "${eof_state}" == 0 ]]; then
        sleep 0.2
        continue
      fi
      sleep 0.2
    done
    while kill -0 "${glim_pid}" >/dev/null 2>&1; do
      glim_consumer_state
      evidence_state=$?
      if [[ "${evidence_state}" -ne 2 ]]; then
        break
      fi
      sleep 0.2
    done
  else
    while kill -0 "${glim_pid}" >/dev/null 2>&1; do
      sleep 0.2
    done
  fi
fi
wait "${glim_pid}"
status=$?
if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
  glim_consumer_eof_state
  eof_state=$?
  glim_consumer_state
  evidence_state=$?
  if [[ "${eof_state}" == 0 && "${input_ended}" == 0 ]]; then
    current_eof_sha256="$(sha256sum "${M6A10_CONSUMER_EOF_EVIDENCE}" 2>/dev/null | awk '{print $1}')"
    if [[ -n "${current_eof_sha256}" && ( -z "${eof_sidecar_sha256}" || "${current_eof_sha256}" == "${eof_sidecar_sha256}" ) ]]; then
      eof_sidecar_sha256="${current_eof_sha256}"
      m6a10_phase_mark input_end || true
      m6a10_phase_mark drain_start || true
      input_ended=1
    else
      eof_state=1
    fi
  fi
  if [[ "${evidence_state}" == 0 && "${eof_state}" == 0 && "${status}" == 0 ]]; then
    m6a10_phase_mark drain_end || true
  elif [[ "${status}" == 0 ]]; then
    status=14
  fi
fi
if [[ "${input_ended}" == 1 && "${status}" == 0 ]]; then
  m6a10_phase_mark postprocess_start || true
  m6a10_phase_mark postprocess_end || true
  m6a10_phase_mark save_start || true
  m6a10_phase_mark save_end || true
  required_end="${M6A10_SENSOR_END_TIMESTAMP_SECONDS:-}"
  if [[ -z "${required_end}" ]]; then
    required_end="$(m6a10_phase_ros2_bag_end "${BAG_PATH}")" || required_end=""
  fi
  if [[ -n "${required_end}" ]]; then
    m6a10_phase_record_trajectory_coverage \
      "${OUT_DIR}/dump/traj_lidar.txt" "${required_end}" || true
  fi
fi
printf '%s\n' "${status}" >"${OUT_DIR}/process_exit_status.txt"
exit "${status}"
