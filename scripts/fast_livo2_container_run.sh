#!/usr/bin/env bash
set -uo pipefail

# Run one official FAST-LIVO2 replay inside the pinned ROS1 container.
BAG_PATH="${BAG_PATH:?BAG_PATH is required}"
OUT_DIR="${OUT_DIR:-/out}"
RATE="${RATE:-1.0}"
SHUTDOWN_GRACE_SECONDS="${SHUTDOWN_GRACE_SECONDS:-5}"
SAVE_MAP="${SAVE_MAP:-0}"
# MERGE: M6a10 opt-in path plus historical defaults.
FAST_PROFILE="${FAST_PROFILE:-hilti22}"
M6A10_FAST_V2=0

# M6a10 is an explicit opt-in path.  It is deliberately stricter than the
# historical FAST wrapper: the source patch owns callback evidence, the
# wrapper owns only the EOF/drain barrier, and no log text is authoritative.
if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
  M6A10_FAST_V2=1
  FAST_PROFILE="${FAST_PROFILE:?FAST_PROFILE=ntu_viral is required for M6a10}"
  [[ "${FAST_PROFILE}" == "ntu_viral" ]] || {
    echo 'M6a10 FAST requires the pinned NTU profile' >&2
    exit 30
  }
  SAVE_MAP=0
  : "${M6A10_PHASE_MODE:?M6a10 phase mode is required}"
  : "${M6A10_FAST_EXPECTED_MESSAGES:?FAST expected total is required}"
  : "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES:?FAST expected LiDAR count is required}"
  : "${M6A10_FAST_EXPECTED_IMU_MESSAGES:?FAST expected IMU count is required}"
  : "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES:?FAST expected image count is required}"
  : "${M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS:?FAST callback bound is required}"
  : "${M6A10_FAST_MAX_BACKLOG_MESSAGES:?FAST backlog bound is required}"
  : "${M6A10_FAST_FEEDER_SHA256:?FAST feeder hash is required}"
  [[ "$(sha256sum /runner/scripts/fast_livo2_m6a10_feeder.py | awk '{print $1}')" == \
      "${M6A10_FAST_FEEDER_SHA256}" ]] || {
    echo 'FAST feeder hash differs from preregistration' >&2
    exit 30
  }
  M6A10_CONSUMER_EVIDENCE="${M6A10_CONSUMER_EVIDENCE:-${OUT_DIR}/consumer_evidence.json}"
  M6A10_PHASE_MAX_CALLBACK_LATENCY_SECONDS="${M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS}"
  M6A10_PHASE_MAX_BACKLOG_MESSAGES="${M6A10_FAST_MAX_BACKLOG_MESSAGES}"
  # The generic phase bridge checks this gate before initializing v2.  The
  # source-level evidence still records the separately observed capability;
  # setting this only enables the barrier path and cannot manufacture PASS.
  M6A10_ACK_BACKPRESSURE_ENABLED="${M6A10_ACK_BACKPRESSURE_ENABLED:-1}"
  export M6A10_CONSUMER_EVIDENCE M6A10_PHASE_MAX_CALLBACK_LATENCY_SECONDS \
    M6A10_PHASE_MAX_BACKLOG_MESSAGES M6A10_ACK_BACKPRESSURE_ENABLED
fi

mkdir -p "${OUT_DIR}"
source /runner/scripts/container_phase_evidence.sh
source /runner/scripts/container_memory_evidence.sh
MAPPING_LAUNCH_PATH="${MAPPING_LAUNCH:-}"
MAPPING_MAP_LAUNCH_PATH="${MAPPING_MAP_LAUNCH:-}"

mkdir -p "${OUT_DIR}"
set +u
source /opt/ros/noetic/setup.bash
source /bench/catkin_ws/devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_HOME="${OUT_DIR}/ros_home"
export ROS_LOG_DIR="${OUT_DIR}/ros_logs"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
if [[ "${SAVE_MAP}" == "1" ]]; then
  mkdir -p /bench/FAST-LIVO2/Log/pcd
fi

write_status() { printf '%s\n' "$2" >"${OUT_DIR}/$1"; }
cleanup() {
  set +e
  for process_group in "${ODOM_PID:-}" "${MAPPER_PID:-}" "${ROSCORE_PID:-}"; do
    if [[ -n "${process_group}" ]]; then
      kill -TERM -- "-${process_group}" >/dev/null 2>&1 || true
      kill -TERM "${process_group}" >/dev/null 2>&1 || true
    fi
  done
# MERGE: M6a10 opt-in path plus historical defaults.
  # Do not use an unscoped `wait` here: the RSS sampler is an intentional
  # background child and is stopped by m6a10_phase_finalize below. Waiting for
  # every child first would deadlock the EXIT trap before evidence is written.
  for child_pid in "${ODOM_PID:-}" "${MAPPER_PID:-}" "${ROSCORE_PID:-}"; do
    if [[ -n "${child_pid}" ]]; then
      wait "${child_pid}" >/dev/null 2>&1 || true
    fi
  done
  local phase_status=0
  m6a10_phase_finalize "${exit_status}" "${M6A10_RESOURCE_REPORT:-}" || \
    phase_status=$?
  # The shared finalizer stops the sampler and publishes evidence after all
  # mapper children have been reaped, so mapper_time.txt is complete.
  m6a5_write_container_memory_evidence "${exit_status}" || true
  if [[ "${exit_status}" -eq 0 && "${phase_status}" -ne 0 && \
        "${M6A3_SYNTHETIC_SMOKE:-0}" != 1 ]]; then
    return "${phase_status}"
  fi
  return "${exit_status}"
}
trap cleanup EXIT
m6a5_install_container_signal_traps
if ! m6a7_start_process_rss_sampler; then
  echo 'FAST process RSS sampler failed to start' >&2
  exit 11
fi

set +u
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
if [[ -f /opt/fast_livo_ws/devel/setup.bash ]]; then
  # Repo-owned pinned image workspace. The /bench mount is reserved for the
  # input asset tree and must not shadow the built image workspace.
  source /opt/fast_livo_ws/devel/setup.bash
elif [[ "${M6A10_FAST_V2}" == 0 && -f /bench/catkin_ws/devel/setup.bash ]]; then
  # Compatibility with the historical externally-built image.
  source /bench/catkin_ws/devel/setup.bash
else
  echo 'FAST-LIVO2 catkin workspace is missing' >&2
  exit 10
fi
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
export ROS_HOSTNAME=127.0.0.1
export ROS_HOME="${OUT_DIR}/ros_home"
export ROS_LOG_DIR="${OUT_DIR}/ros_logs"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
if [[ "${SAVE_MAP}" == "1" && "${M6A10_FAST_V2}" == 0 ]]; then
  mkdir -p /bench/FAST-LIVO2/Log/pcd
fi
  wait >/dev/null 2>&1 || true
trap cleanup EXIT

rosbag info --yaml "${BAG_PATH}" >"${OUT_DIR}/rosbag_info.yaml" 2>"${OUT_DIR}/rosbag_info.err"
write_status rosbag_info_exit_status.txt "$?"
FAST_INPUT_DURATION="${M6A10_SENSOR_DURATION_SECONDS:-}"
if [[ -z "${FAST_INPUT_DURATION}" ]]; then
  FAST_INPUT_DURATION="$(m6a10_phase_ros1_bag_duration "${OUT_DIR}/rosbag_info.yaml")" || {
    echo 'FAST could not determine rosbag duration for phase evidence' >&2
    exit 12
  }
fi
m6a10_phase_init fast_livo2 "${FAST_INPUT_DURATION}" || exit 12
M6A10_RESOURCE_REPORT="${OUT_DIR}/mapper_time.txt"
export M6A10_PHASE_EVENTS M6A10_PHASE_EVIDENCE M6A10_RESOURCE_REPORT \
  M6A10_CONSUMER_EVIDENCE

setsid roscore >"${OUT_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!
master_ready=0
for _ in $(seq 1 50); do
  if rosparam list >/dev/null 2>&1; then master_ready=1; break; fi
  sleep 0.1
done
write_status master_ready.txt "${master_ready}"
if [[ "${master_ready}" != 1 ]]; then exit 20; fi
rosparam set use_sim_time true

# MERGE: profile selection (ours) plus path overrides (develop).
if [[ "${FAST_PROFILE}" == "ntu_viral" ]]; then
  MAPPING_LAUNCH=(fast_livo mapping_ouster_ntu.launch rviz:=false)
elif [[ "${FAST_PROFILE}" == "hilti22" ]]; then
  MAPPING_LAUNCH=(fast_livo mapping_hesaixt32_hilti22.launch rviz:=false)
else
  echo "FAST_PROFILE is unsupported: ${FAST_PROFILE}" >&2
  exit 23
fi
if [[ -n "${MAPPING_LAUNCH_PATH}" ]]; then
  MAPPING_LAUNCH=("${MAPPING_LAUNCH_PATH}")
fi
if [[ "${SAVE_MAP}" == "1" && -n "${MAPPING_MAP_LAUNCH_PATH}" ]]; then
  MAPPING_LAUNCH=("${MAPPING_MAP_LAUNCH_PATH}")
elif [[ "${SAVE_MAP}" == "1" ]]; then
  MAPPING_LAUNCH=(/runner/configs/fast_livo2/mapping_hesaixt32_hilti22_benchmark_map.launch)
fi
setsid bash /runner/scripts/run_with_resource_report.sh \
  "${OUT_DIR}/mapper_time.txt" \
  roslaunch "${MAPPING_LAUNCH[@]}" \
  >"${OUT_DIR}/mapper.log" 2>&1 &
MAPPER_PID=$!
mapper_ready=0
for _ in $(seq 1 300); do
  if ! kill -0 "${MAPPER_PID}" >/dev/null 2>&1; then break; fi
  if rostopic info /aft_mapped_to_init 2>/dev/null | grep -q 'Type: nav_msgs/Odometry'; then
    mapper_ready=1
    break
  fi
  sleep 0.1
done
write_status mapper_ready.txt "${mapper_ready}"
if [[ "${mapper_ready}" != 1 ]]; then
  wait "${MAPPER_PID}"
  write_status mapper_shutdown_exit_status.txt "$?"
  exit 21
fi
m6a10_phase_mark startup_end || true

if [[ "${M6A3_SYNTHETIC_SMOKE:-0}" == "1" ]]; then
  # Consume the synthetic ROS1 bag briefly so this path checks input parsing,
  # loopback master connectivity, mapper startup, and clean shutdown without
  # producing a performance result.
  rosbag play --clock --rate "${RATE}" "${BAG_PATH}" \
    >"${OUT_DIR}/synthetic_smoke_rosbag_play.log" 2>&1 &
  smoke_bag_pid=$!
  sleep "${SMOKE_INPUT_SECONDS:-2}"
  kill -INT "${smoke_bag_pid}" >/dev/null 2>&1 || true
  wait "${smoke_bag_pid}" >/dev/null 2>&1 || true
  if [[ -n "${MAPPER_PID:-}" ]]; then
    kill -INT -- "-${MAPPER_PID}" >/dev/null 2>&1 || true
    wait "${MAPPER_PID}" >/dev/null 2>&1 || true
    MAPPER_PID=""
  fi
  if [[ -n "${ROSCORE_PID:-}" ]]; then
    kill -TERM -- "-${ROSCORE_PID}" >/dev/null 2>&1 || true
    wait "${ROSCORE_PID}" >/dev/null 2>&1 || true
    ROSCORE_PID=""
  fi
  printf '%s\n' '{"status":"pass","startup_verified":true,"input_verified":true,"clean_shutdown":true,"gt_mounted":false,"performance_run":false,"loopback_only":true}' >"${OUT_DIR}/synthetic_smoke_contract.json"
  exit 0
fi

setsid rostopic echo -p /aft_mapped_to_init \
  >"${OUT_DIR}/odometry.csv" 2>"${OUT_DIR}/odometry.err" &
ODOM_PID=$!

if [[ "${M6A10_FAST_V2}" == 1 ]]; then
  # M6a10 uses a single-inflight feeder.  It publishes one rosbag record,
  # waits for the application callback counter, then records one ACK before
  # reading the next record.  rosbag play is intentionally not used here.
  m6a10_phase_mark input_start || exit 125
  python3 /runner/scripts/fast_livo2_m6a10_feeder.py \
    --bag "${BAG_PATH}" \
    --output "${OUT_DIR}/feeder_receipt.json" \
    --timeout "${M6A10_DRAIN_TIMEOUT_SECONDS:-60}" \
    --expected-lidar "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" \
    --expected-imu "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" \
    --expected-image "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" \
    >"${OUT_DIR}/feeder.log" 2>&1
  BAG_EXIT=$?
  write_status bag_exit_status.txt "${BAG_EXIT}"
  if [[ "${BAG_EXIT}" -ne 0 ]]; then
    echo 'FAST single-inflight feeder failed' >&2
    exit 125
  fi
  m6a10_phase_mark input_end || exit 125
  # EOF is an application service barrier.  The source writes the immutable
  # sidecar before the wrapper starts its bounded drain polling interval.
  rosservice call /m6a10/consumer_eof "{}" \
    >"${OUT_DIR}/consumer_eof_service.txt" 2>"${OUT_DIR}/consumer_eof_service.err" || {
      echo 'FAST consumer EOF service failed' >&2
      exit 125
    }
  test -s "${M6A10_CONSUMER_EVIDENCE}.eof.json" || {
    echo 'FAST consumer EOF sidecar is missing' >&2
    exit 125
  }
  m6a10_phase_mark drain_start || exit 125
  python3 /runner/scripts/fast_livo2_m6a10_feeder.py \
    --bag "${BAG_PATH}" --drain --eof-already-observed \
    --output "${OUT_DIR}/drain_receipt.json" \
    --timeout "${M6A10_DRAIN_TIMEOUT_SECONDS:-60}" \
    >"${OUT_DIR}/drain.log" 2>&1 || {
      echo 'FAST internal deque did not drain to zero' >&2
      exit 124
    }
  python3 - "${OUT_DIR}/feeder_receipt.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    feeder = json.load(stream)
if feeder.get("status") != "pass" or feeder.get("single_inflight") is not True:
    raise SystemExit("FAST feeder did not prove single-inflight publication")
if feeder.get("ack_backpressure_verified") is not True:
    raise SystemExit("FAST feeder did not prove callback ACK backpressure")
if feeder.get("published_topic_counts") != feeder.get("expected_topic_counts"):
    raise SystemExit("FAST feeder topic counts do not match preregistration")
if feeder.get("acked_topic_counts") != feeder.get("expected_topic_counts"):
    raise SystemExit("FAST feeder ACK counts do not match preregistration")
if feeder.get("ground_truth_content_opened") or feeder.get("scorer_invoked"):
    raise SystemExit("FAST feeder safety flags are invalid")
PY
else
  m6a10_phase_mark input_start || true
  bash /runner/scripts/run_with_resource_report.sh \
    "${OUT_DIR}/bag_time.txt" \
    rosbag play --clock --rate "${RATE}" "${BAG_PATH}" \
    >"${OUT_DIR}/rosbag_play.log" 2>&1
  BAG_EXIT=$?
  write_status bag_exit_status.txt "${BAG_EXIT}"
  m6a10_phase_mark input_end || true
fi

if [[ "${M6A10_FAST_V2}" == 0 ]]; then
  m6a10_phase_mark drain_start || true
fi
FAST_REQUIRED_END="${M6A10_SENSOR_END_TIMESTAMP_SECONDS:-}"
if [[ -z "${FAST_REQUIRED_END}" ]]; then
  FAST_REQUIRED_END="$(m6a10_phase_ros1_bag_end "${OUT_DIR}/rosbag_info.yaml")" || FAST_REQUIRED_END=""
fi
if [[ -n "${FAST_REQUIRED_END}" ]]; then
  m6a10_phase_wait_for_trajectory "${OUT_DIR}/odometry.csv" \
    "${FAST_REQUIRED_END}" "${DRAIN_TIMEOUT_SECS:-60}" || true
  m6a10_phase_record_trajectory_coverage \
    "${OUT_DIR}/odometry.csv" "${FAST_REQUIRED_END}" || true
fi
if [[ "${M6A10_FAST_V2}" == 1 ]]; then
  rosservice call /m6a10/consumer_finalize "{}" \
    >"${OUT_DIR}/consumer_finalize_service.txt" 2>"${OUT_DIR}/consumer_finalize_service.err" || {
      echo 'FAST consumer finalize service failed' >&2
      exit 125
    }
  test -s "${M6A10_CONSUMER_EVIDENCE}" || {
    echo 'FAST consumer evidence is missing' >&2
    exit 125
  }
  python3 - "${M6A10_CONSUMER_EVIDENCE}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    evidence = json.load(stream)
if evidence.get("schema_version") != 2:
    raise SystemExit("FAST consumer evidence schema mismatch")
if evidence.get("system") != "fast_livo2":
    raise SystemExit("FAST consumer evidence system mismatch")
if evidence.get("status") != "pass":
    raise SystemExit("FAST consumer evidence is fail-closed invalid")
consumer = evidence.get("consumer")
if not isinstance(consumer, dict) or not consumer.get("eof_observed") or not consumer.get("drain_complete"):
    raise SystemExit("FAST consumer EOF/drain contract is incomplete")
if consumer.get("received_messages") != consumer.get("expected_messages"):
    raise SystemExit("FAST consumer total count mismatch")
expected = consumer.get("expected_topic_counts")
observed = consumer.get("received_topic_counts")
if not isinstance(expected, dict) or expected != observed:
    raise SystemExit("FAST consumer per-topic count mismatch")
if consumer.get("acked_messages") != consumer.get("expected_messages"):
    raise SystemExit("FAST consumer ACK count mismatch")
if consumer.get("dropped_messages") != 0 or consumer.get("queue_overflow") != 0:
    raise SystemExit("FAST consumer reported drop/overflow")
if not consumer.get("queue_overflow_observable") or not consumer.get("ack_backpressure_verified"):
    raise SystemExit("FAST consumer capability was not observed")
if consumer.get("backlog_at_drain") != 0:
    raise SystemExit("FAST internal deque was not empty at drain")
PY
  m6a10_phase_mark drain_end || exit 125
  # These are phase boundaries only; map saving is disabled and no map path is
  # mounted in the dedicated host runner.
  m6a10_phase_mark postprocess_start || exit 125
  m6a10_phase_mark postprocess_end || exit 125
  m6a10_phase_mark save_start || exit 125
  m6a10_phase_mark save_end || exit 125
else
  m6a10_phase_mark drain_end || true
  m6a10_phase_mark postprocess_start || true
  m6a10_phase_mark postprocess_end || true
  m6a10_phase_mark save_start || true
  m6a10_phase_mark save_end || true
fi

# The mapper is a continuous node. Record input-completion health separately
# from its response to the benchmark's external shutdown request.
mapper_alive=0
if kill -0 "${MAPPER_PID}" >/dev/null 2>&1 && \
    timeout 10 rosnode ping -c 1 /laserMapping >/dev/null 2>&1; then
  mapper_alive=1
fi
write_status mapper_alive_after_bag.txt "${mapper_alive}"
sleep "${SHUTDOWN_GRACE_SECONDS}"

kill -TERM -- "-${ODOM_PID}" >/dev/null 2>&1 || true
wait "${ODOM_PID}" >/dev/null 2>&1 || true
ODOM_PID=""
if kill -0 "${MAPPER_PID}" >/dev/null 2>&1; then
  # Background helpers inherit SIGINT=ignored from the non-interactive shell;
  # use TERM so run_with_resource_report's supervisor trap can stop the
  # mapper group and flush its report instead of waiting forever.
  kill -TERM -- "-${MAPPER_PID}" >/dev/null 2>&1 || true
fi
wait "${MAPPER_PID}"
MAPPER_EXIT=$?
write_status mapper_shutdown_exit_status.txt "${MAPPER_EXIT}"
MAPPER_PID=""

if [[ "${BAG_EXIT}" -ne 0 || "${mapper_alive}" -ne 1 ]]; then exit 22; fi
exit 0
