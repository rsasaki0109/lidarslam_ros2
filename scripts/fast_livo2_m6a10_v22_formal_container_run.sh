#!/usr/bin/env bash
# FAST-LIVO2 M6A10 v22 additive wrapper candidate.
# It is a production-shaped candidate only; it does not authorize replay.
set -Eeuo pipefail

EXPECTED_CONTRACT='m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary'
EXPECTED_PHASE_MODE='unpaced_ack'
EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/fast_livo2_m6a10_v17_formal_candidate.yaml'
EXPECTED_PROFILE_SHA256='1803ba255adfac96a61e1f677be53e84b196ad70bb367a10ebdb9791cb94128b'
EXPECTED_FEEDER_SHA256='6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7'
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
TRANSPORT_CONTRACT='m6a10-v12-callback-ack-transport-outstanding-v1'
FEEDER_PATH='/runner/scripts/fast_livo2_m6a10_feeder.py'
SYNTHETIC_MODE="${M6A10_V22_SYNTHETIC_MODE:-0}"
if [[ "${SYNTHETIC_MODE}" == '1' ]]; then
  FEEDER_PATH="${M6A10_V22_SYNTHETIC_FEEDER_PATH:?synthetic feeder path is required}"
  EXPECTED_FEEDER_SHA256="${M6A10_V22_SYNTHETIC_FEEDER_SHA256:?synthetic feeder SHA is required}"
fi

OUT_DIR="${OUT_DIR:-/out}"
[[ ! -L "${OUT_DIR}" ]] || { echo 'OUT_DIR symlink refusal' >&2; exit 1; }
[[ -d "${OUT_DIR}" ]] || mkdir -p "${OUT_DIR}"
OUT_DIR="$(cd "${OUT_DIR}" && pwd -P)"
CALLBACK_EVIDENCE="${M6A10_CONSUMER_EVIDENCE:-${OUT_DIR}/callback_consumer_evidence.json}"
TERMINAL_EVIDENCE="${M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE:-${OUT_DIR}/consumer_evidence.json}"
ONLINE_TIMING="${M6A10_ONLINE_TIMING_EVIDENCE:-${OUT_DIR}/online_compute_timing.json}"
FEEDER_EXIT_STATUS="${M6A10_FEEDER_EXIT_STATUS_EVIDENCE:-${OUT_DIR}/feeder_exit_status.txt}"
FAILURE_REASON="${M6A10_FAILURE_REASON_EVIDENCE:-${OUT_DIR}/failure_reason.json}"
ARTIFACT_MANIFEST="${M6A10_ARTIFACT_MANIFEST:-${OUT_DIR}/artifact_manifest.json}"
M6A10_LAST_FAILURE_REASON=''
FEEDER_STATUS=''
MANIFEST_WRITTEN=0

# O_EXCL + O_NOFOLLOW + fsync + link makes each status artifact atomic and
# refuses a pre-existing or symlinked target.
atomic_text() {
  local final="${1:?final}" body="${2:?body}" mode="${3:-444}"
  python3 - "${final}" "${final}.part" "${body}" "${mode}" <<'PY'
import os, pathlib, sys
final, part = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
payload, mode = sys.argv[3].encode(), int(sys.argv[4], 8)
if final.exists() or final.is_symlink() or part.exists() or part.is_symlink():
    raise SystemExit("atomic overwrite refusal")
fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
try:
    with os.fdopen(fd, "wb") as stream:
        stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.link(part, final, follow_symlinks=False); os.chmod(final, mode)
    dfd = os.open(final.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try: os.fsync(dfd)
    finally: os.close(dfd)
finally:
    try: part.unlink()
    except FileNotFoundError: pass
PY
}

write_failure_reason() {
  local reason="${1:-unknown_failure}"
  [[ -e "${FAILURE_REASON}" || -L "${FAILURE_REASON}" ]] && return 0
  python3 - "${FAILURE_REASON}" "${FAILURE_REASON}.part" "${reason}" "${FEEDER_STATUS:-}" <<'PY'
import json, os, pathlib, sys
final, part = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
reason, feeder = sys.argv[3], sys.argv[4]
if final.exists() or final.is_symlink() or part.exists() or part.is_symlink():
    raise SystemExit("failure reason overwrite refusal")
value = {"schema_version": 1,
         "contract_version": "m6a10-v22-failure-reason-v1",
         "status": "FAIL_CLOSED", "reason": reason,
         "feeder_exit_status": int(feeder) if feeder.isdigit() else None,
         "ground_truth_content_opened": False, "scorer_invoked": False,
         "map_saved": False}
payload = (json.dumps(value, sort_keys=True, indent=2) + "\n").encode()
fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
try:
    with os.fdopen(fd, "wb") as stream:
        stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.link(part, final, follow_symlinks=False); os.chmod(final, 0o444)
    dfd = os.open(final.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try: os.fsync(dfd)
    finally: os.close(dfd)
finally:
    try: part.unlink()
    except FileNotFoundError: pass
PY
}

fail() {
  M6A10_LAST_FAILURE_REASON="${1:-unknown_failure}"
  write_failure_reason "${M6A10_LAST_FAILURE_REASON}" || true
  echo "FAST-LIVO2 v22 wrapper fail-closed: ${M6A10_LAST_FAILURE_REASON}" >&2
  return 1
}

write_artifact_manifest() {
  local exit_status="${1:?exit status}"
  if (( MANIFEST_WRITTEN == 1 )); then return 0; fi
  python3 - "${ARTIFACT_MANIFEST}" "${ARTIFACT_MANIFEST}.part" \
    "${OUT_DIR}" "${exit_status}" "${M6A10_LAST_FAILURE_REASON:-}" <<'PY'
import hashlib, json, os, pathlib, sys
final, part, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]), pathlib.Path(sys.argv[3])
exit_status, reason = int(sys.argv[4]), sys.argv[5]
if final.exists() or final.is_symlink() or part.exists() or part.is_symlink():
    raise SystemExit("manifest overwrite refusal")
names = ("feeder_receipt.json", "feeder.log", "feeder_exit_status.txt",
         "callback_consumer_evidence.json", "consumer_evidence.json",
         "online_compute_timing.json", "container_run_status.txt")
files, missing = {}, []
for name in names:
    path = out / name
    if path.is_symlink() or not path.is_file():
        missing.append(name); continue
    raw = path.read_bytes()
    files[name] = {"path": str(path), "bytes": len(raw),
                   "sha256": hashlib.sha256(raw).hexdigest()}
value = {
    "schema_version": 1, "contract_version": "m6a10-v22-artifact-manifest-v1",
    "status": "PASS" if exit_status == 0 and not missing else "FAIL_CLOSED",
    "exit_status": exit_status, "failure_reason": reason or None,
    "files": files, "missing": missing,
    "ground_truth_content_opened": False, "scorer_invoked": False,
    "map_saved": False,
    "safety": {"input_opened": bool(files.get("feeder_receipt")),
               "ground_truth_content_opened": False, "scorer_invoked": False,
               "map_saved": False},
}
payload = (json.dumps(value, sort_keys=True, indent=2) + "\n").encode()
fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
try:
    with os.fdopen(fd, "wb") as stream:
        stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.link(part, final, follow_symlinks=False); os.chmod(final, 0o444)
    dfd = os.open(final.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try: os.fsync(dfd)
    finally: os.close(dfd)
finally:
    try: part.unlink()
    except FileNotFoundError: pass
PY
  MANIFEST_WRITTEN=1
}

ROSCORE_PID=''; MAPPER_PID=''; ODOM_PID=''; FEEDER_PID=''
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
  if (( exit_status != 0 )); then
    write_failure_reason "${M6A10_LAST_FAILURE_REASON:-exit_${exit_status}}" || true
  fi
  write_artifact_manifest "${exit_status}" || true
  stop_group "${FEEDER_PID}"; stop_group "${ODOM_PID}"
  stop_group "${MAPPER_PID}"; stop_group "${ROSCORE_PID}"
  for pid in "${FEEDER_PID}" "${ODOM_PID}" "${MAPPER_PID}" "${ROSCORE_PID}"; do
    [[ -n "${pid}" ]] && wait "${pid}" >/dev/null 2>&1 || true
  done
  exit "${exit_status}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

PROFILE_PATH="${M6A10_PROFILE_PATH:?M6A10_PROFILE_PATH is required}"
PROFILE_SHA256="${M6A10_PROFILE_SHA256:?M6A10_PROFILE_SHA256 is required}"
M6A10_PHASE_CONTRACT_VERSION="${M6A10_PHASE_CONTRACT_VERSION:?M6A10_PHASE_CONTRACT_VERSION is required}"
M6A10_PHASE_MODE="${M6A10_PHASE_MODE:?M6A10_PHASE_MODE is required}"
M6A10_FAST_FEEDER_SHA256="${M6A10_FAST_FEEDER_SHA256:?M6A10_FAST_FEEDER_SHA256 is required}"
M6A10_SENSOR_DURATION_SECONDS="${M6A10_SENSOR_DURATION_SECONDS:?M6A10_SENSOR_DURATION_SECONDS is required}"
M6A10_TIMING_CONTRACT_VERSION="${M6A10_TIMING_CONTRACT_VERSION:?M6A10_TIMING_CONTRACT_VERSION is required}"
BAG_PATH="${M6A10_BAG_PATH:-${BAG_PATH:-}}"
BAG_BYTES="${M6A10_BAG_BYTES:-}"; BAG_SHA256="${M6A10_BAG_SHA256:-}"
M6A10_FAST_EXPECTED_MESSAGES="${M6A10_FAST_EXPECTED_MESSAGES:-}"
M6A10_FAST_EXPECTED_LIDAR_MESSAGES="${M6A10_FAST_EXPECTED_LIDAR_MESSAGES:-}"
M6A10_FAST_EXPECTED_IMU_MESSAGES="${M6A10_FAST_EXPECTED_IMU_MESSAGES:-}"
M6A10_FAST_EXPECTED_IMAGE_MESSAGES="${M6A10_FAST_EXPECTED_IMAGE_MESSAGES:-}"
M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS="${M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS:-}"
M6A10_FAST_MAX_END_GAP_SECONDS="${M6A10_FAST_MAX_END_GAP_SECONDS:-}"
M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS="${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS:-}"
M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS="${M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS:-}"
M6A10_FAST_MAX_BACKLOG_MESSAGES="${M6A10_FAST_MAX_BACKLOG_MESSAGES:-}"
[[ "${M6A10_PHASE_CONTRACT_VERSION}" == "${EXPECTED_CONTRACT}" ]] || fail 'phase contract'
[[ "${M6A10_PHASE_MODE}" == "${EXPECTED_PHASE_MODE}" ]] || fail 'phase mode'
[[ "${PROFILE_PATH}" == "${EXPECTED_PROFILE_PATH}" || "${PROFILE_PATH}" == "/runner/${EXPECTED_PROFILE_PATH}" ]] || fail 'profile path'
[[ "${PROFILE_SHA256}" == "${EXPECTED_PROFILE_SHA256}" ]] || fail 'profile SHA'
[[ "${M6A10_FAST_FEEDER_SHA256}" == "${EXPECTED_FEEDER_SHA256}" ]] || fail 'feeder SHA'
if [[ "${SYNTHETIC_MODE}" == '1' ]]; then
  [[ "${FEEDER_PATH}" == '/runner/synthetic/feeder.py' ]] || fail 'synthetic feeder path'
fi
[[ "${BAG_PATH}" == "${EXPECTED_BAG_PATH}" && "${BAG_BYTES}" == "${EXPECTED_BAG_BYTES}" && "${BAG_SHA256}" == "${EXPECTED_BAG_SHA256}" ]] || fail 'bag identity'
[[ "${M6A10_FAST_EXPECTED_MESSAGES}" == "${EXPECTED_MESSAGES}" && "${M6A10_FAST_EXPECTED_LIDAR_MESSAGES}" == "${EXPECTED_LIDAR_MESSAGES}" && "${M6A10_FAST_EXPECTED_IMU_MESSAGES}" == "${EXPECTED_IMU_MESSAGES}" && "${M6A10_FAST_EXPECTED_IMAGE_MESSAGES}" == "${EXPECTED_IMAGE_MESSAGES}" ]] || fail 'expected counts'
[[ "${M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS}" == "${EXPECTED_END_TIMESTAMP}" && "${M6A10_FAST_MAX_END_GAP_SECONDS}" == "${EXPECTED_END_GAP}" && "${M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS}" == "${EXPECTED_MIN_POLL}" ]] || fail 'terminal boundary'
[[ "${M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS}" == "${EXPECTED_CALLBACK_LATENCY}" && "${M6A10_FAST_MAX_BACKLOG_MESSAGES}" == "${EXPECTED_TRANSPORT_LIMIT}" ]] || fail 'transport bounds'
[[ "${M6A10_SENSOR_DURATION_SECONDS}" == "${EXPECTED_SENSOR_DURATION}" && "${M6A10_TIMING_CONTRACT_VERSION}" == "${EXPECTED_TIMING_CONTRACT}" ]] || fail 'timing contract'
[[ "${CALLBACK_EVIDENCE}" == "${OUT_DIR}/callback_consumer_evidence.json" && "${TERMINAL_EVIDENCE}" == "${OUT_DIR}/consumer_evidence.json" && "${ONLINE_TIMING}" == "${OUT_DIR}/online_compute_timing.json" ]] || fail 'evidence paths'
[[ "${CALLBACK_EVIDENCE}" != "${TERMINAL_EVIDENCE}" ]] || fail 'evidence alias'
[[ "${FEEDER_EXIT_STATUS}" == "${OUT_DIR}/feeder_exit_status.txt" && "${FAILURE_REASON}" == "${OUT_DIR}/failure_reason.json" && "${ARTIFACT_MANIFEST}" == "${OUT_DIR}/artifact_manifest.json" ]] || fail 'durable artifact paths'
for path in "${CALLBACK_EVIDENCE}" "${TERMINAL_EVIDENCE}" "${ONLINE_TIMING}" "${FEEDER_EXIT_STATUS}" "${FAILURE_REASON}" "${ARTIFACT_MANIFEST}"; do
  [[ ! -e "${path}" && ! -L "${path}" && ! -e "${path}.part" && ! -L "${path}.part" ]] || fail 'output overwrite'
done
[[ -f "/runner/${EXPECTED_PROFILE_PATH}" && ! -L "/runner/${EXPECTED_PROFILE_PATH}" ]] || fail 'profile missing'
[[ "$(sha256sum "/runner/${EXPECTED_PROFILE_PATH}" | awk '{print $1}')" == "${EXPECTED_PROFILE_SHA256}" ]] || fail 'profile bytes'
[[ -f "${FEEDER_PATH}" && ! -L "${FEEDER_PATH}" ]] || fail 'feeder missing'
[[ "$(sha256sum "${FEEDER_PATH}" | awk '{print $1}')" == "${EXPECTED_FEEDER_SHA256}" ]] || fail 'feeder bytes'

export ROS_MASTER_URI='http://127.0.0.1:11311' ROS_IP='127.0.0.1' ROS_HOSTNAME='127.0.0.1'
export ROS_HOME="${OUT_DIR}/ros_home" ROS_LOG_DIR="${OUT_DIR}/ros_logs"
export M6A10_PHASE_CONTRACT_VERSION M6A10_PHASE_MODE M6A10_PROFILE_PATH M6A10_PROFILE_SHA256
export M6A10_CONSUMER_EVIDENCE="${CALLBACK_EVIDENCE}" M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE="${TERMINAL_EVIDENCE}"
export M6A10_ONLINE_TIMING_EVIDENCE="${ONLINE_TIMING}" M6A10_FEEDER_EXIT_STATUS_EVIDENCE="${FEEDER_EXIT_STATUS}"
export M6A10_SENSOR_DURATION_SECONDS M6A10_TIMING_CONTRACT_VERSION
export M6A10_FAST_EXPECTED_MESSAGES M6A10_FAST_EXPECTED_LIDAR_MESSAGES M6A10_FAST_EXPECTED_IMU_MESSAGES M6A10_FAST_EXPECTED_IMAGE_MESSAGES
export M6A10_FAST_REQUIRED_END_TIMESTAMP_SECONDS M6A10_FAST_MAX_END_GAP_SECONDS M6A10_FAST_MIN_TERMINAL_POLL_WALL_SECONDS
export M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS M6A10_FAST_MAX_BACKLOG_MESSAGES
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"

source /opt/ros/noetic/setup.bash
source /opt/fast_livo_ws/devel/setup.bash
# ROS setup scripts prepend their own bin directories.  Synthetic gates must
# keep their sealed command surface first; otherwise the image's production
# rosservice/roslaunch talks to an empty mapper and cannot produce the
# synthetic evidence contract.
if [[ "${SYNTHETIC_MODE}" == '1' ]]; then
  export PATH="/runner/fakebin:${PATH}"
fi
roscore >"${OUT_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!
master_deadline=$((SECONDS + ${M6A10_STARTUP_TIMEOUT_SECONDS:-60})); master_ready=0
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
  /m6a10/terminal_finalize
)
service_deadline=$((SECONDS + ${M6A10_STARTUP_TIMEOUT_SECONDS:-60})); services_ready=0
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
rostopic echo -p /aft_mapped_to_init >"${OUT_DIR}/odometry.csv" 2>"${OUT_DIR}/odometry.err" &
ODOM_PID=$!

# The start boundary is immediately before the one feeder process.
INPUT_START_MONOTONIC_NS="$(python3 -c 'import time; print(time.monotonic_ns())')"
set +e
python3 "${FEEDER_PATH}" --bag "${BAG_PATH}" --output "${OUT_DIR}/feeder_receipt.json" \
  --timeout "${M6A10_FEEDER_TIMEOUT_SECONDS:-1200}" \
  --expected-lidar "${EXPECTED_LIDAR_MESSAGES}" --expected-imu "${EXPECTED_IMU_MESSAGES}" \
  --expected-image "${EXPECTED_IMAGE_MESSAGES}" >"${OUT_DIR}/feeder.log" 2>&1 &
FEEDER_PID=$!
wait "${FEEDER_PID}"; feeder_status=$?; FEEDER_PID=''
set -e
FEEDER_STATUS="${feeder_status}"
printf -v feeder_status_payload '%s\n' "${feeder_status}"
# Persist before branching, for both feeder success and failure.
atomic_text "${FEEDER_EXIT_STATUS}" "${feeder_status_payload}" 444 || fail 'feeder exit status persistence'
(( feeder_status == 0 )) || fail 'feeder failed'

python3 - "${OUT_DIR}/feeder_receipt.json" "${BAG_PATH}" "${BAG_BYTES}" "${BAG_SHA256}" \
  "${EXPECTED_MESSAGES}" "${EXPECTED_LIDAR_MESSAGES}" "${EXPECTED_IMU_MESSAGES}" "${EXPECTED_IMAGE_MESSAGES}" <<'PY'
import json, sys
path, bag, size, digest, total, lidar, imu, image = sys.argv[1:]
value = json.loads(open(path, encoding="utf-8").read())
counts = {"lidar": int(lidar), "imu": int(imu), "image": int(image)}
if value.get("status") != "pass" or value.get("single_inflight") is not True: raise SystemExit("feeder contract")
if value.get("bag_path") != bag or value.get("bag_bytes") != int(size) or value.get("bag_sha256") != digest: raise SystemExit("feeder identity")
if value.get("published_messages") != int(total) or value.get("expected_topic_counts") != counts or value.get("published_topic_counts") != counts or value.get("acked_topic_counts") != counts: raise SystemExit("feeder counts")
if value.get("ack_backpressure_verified") is not True or value.get("ground_truth_content_opened") is not False or value.get("scorer_invoked") is not False: raise SystemExit("feeder safety")
PY

call_trigger() {
  local service="${1:?service}" response="${2:?response}"
  timeout "${M6A10_RPC_TIMEOUT_SECONDS:-10}" rosservice call "${service}" "{}" >"${response}" 2>"${response}.err" || return 1
  grep -Fq 'success: True' "${response}"
}
call_trigger /m6a10/consumer_eof "${OUT_DIR}/consumer_eof.response" || fail 'consumer EOF'
call_trigger /m6a10/consumer_status "${OUT_DIR}/consumer_status.response" || fail 'consumer status'
call_trigger /m6a10/consumer_finalize "${OUT_DIR}/consumer_finalize.response" || fail 'consumer finalize'
python3 - "${CALLBACK_EVIDENCE}" "${EXPECTED_LIDAR_MESSAGES}" "${EXPECTED_IMU_MESSAGES}" "${EXPECTED_IMAGE_MESSAGES}" <<'PY'
import json, pathlib, sys
path = pathlib.Path(sys.argv[1])
if path.is_symlink() or not path.is_file() or pathlib.Path(str(path) + ".part").exists(): raise SystemExit("callback file")
value = json.loads(path.read_text(encoding="utf-8"))
expected = {"lidar": int(sys.argv[2]), "imu": int(sys.argv[3]), "image": int(sys.argv[4])}
if value.get("schema_version") != 3 or value.get("contract_version") != "m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary" or value.get("transport_contract_version") != "m6a10-v12-callback-ack-transport-outstanding-v1" or value.get("status") != "pass": raise SystemExit("callback schema")
for key in ("input", "profile_sha256", "profile_path", "raw_path", "raw_sha256", "published", "ack", "binding", "validation"):
    if key in value: raise SystemExit("callback authority field")
ledger = value.get("consumer")
if not isinstance(ledger, dict) or ledger.get("received_topic_counts") != expected or ledger.get("received_messages") != sum(expected.values()) or ledger.get("acked_messages") != sum(expected.values()): raise SystemExit("callback counts")
if ledger.get("ack_exact") is not True or ledger.get("transport_outstanding_at_drain") != 0 or ledger.get("maximum_transport_outstanding_messages") != 1 or ledger.get("maximum_allowed_transport_outstanding_messages") != 1: raise SystemExit("callback transport")
if ledger.get("eof_observed") is not True or ledger.get("drain_complete") is not True or ledger.get("dropped_messages") != 0 or ledger.get("queue_overflow") != 0 or ledger.get("processing_failures") != 0: raise SystemExit("callback completion")
if value.get("ground_truth_content_opened") is not False or value.get("scorer_invoked") is not False: raise SystemExit("callback safety")
PY

call_trigger /m6a10/terminal_eof "${OUT_DIR}/terminal_eof.response" || fail 'terminal EOF'
call_trigger /m6a10/terminal_status "${OUT_DIR}/terminal_status_1.response" || fail 'terminal status 1'
sleep "${EXPECTED_MIN_POLL}"
call_trigger /m6a10/terminal_status "${OUT_DIR}/terminal_status_2.response" || fail 'terminal status 2'
call_trigger /m6a10/terminal_finalize "${OUT_DIR}/terminal_finalize.response" || fail 'terminal finalize'
python3 - "${TERMINAL_EVIDENCE}" "${EXPECTED_LIDAR_MESSAGES}" "${EXPECTED_IMU_MESSAGES}" "${EXPECTED_IMAGE_MESSAGES}" "${EXPECTED_END_TIMESTAMP}" "${EXPECTED_END_GAP}" <<'PY'
import json, math, pathlib, sys
path = pathlib.Path(sys.argv[1])
if path.is_symlink() or not path.is_file() or pathlib.Path(str(path) + ".part").exists(): raise SystemExit("terminal file")
value = json.loads(path.read_text(encoding="utf-8"))
expected = {"lidar": int(sys.argv[2]), "imu": int(sys.argv[3]), "image": int(sys.argv[4])}
if value.get("schema_version") != 1 or value.get("contract_id") != "m6a10-fast-livo2-consumer-terminal-v1" or value.get("status") != "pass": raise SystemExit("terminal schema")
for key in ("input", "profile_sha256", "expected", "published", "ack", "binding", "validation"):
    if key in value: raise SystemExit("terminal authority field")
if value.get("received_topic_counts") != expected or value.get("ground_truth_content_opened") is not False or value.get("scorer_invoked") is not False: raise SystemExit("terminal counts/safety")
support = value.get("terminal_support_context")
if not isinstance(support, dict) or support.get("by_topic", {}).get("lidar") != 0: raise SystemExit("terminal support")
trajectory = value.get("trajectory"); gap = trajectory.get("end_gap_seconds") if isinstance(trajectory, dict) else None; last = trajectory.get("last_timestamp_seconds") if isinstance(trajectory, dict) else None
if not isinstance(trajectory, dict) or trajectory.get("coverage_verified") is not True or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(float(x)) for x in (gap, last)) or gap < 0 or gap > float(sys.argv[6]) or abs(float(sys.argv[5]) - last - gap) > 1e-6: raise SystemExit("terminal trajectory")
observation = value.get("terminal_observation")
if not isinstance(observation, dict) or observation.get("eof_observed") is not True or observation.get("stable") is not True or observation.get("identical_snapshot") is not True or observation.get("poll_count", 0) < 2 or observation.get("stable_poll_count", 0) < 2: raise SystemExit("terminal polling")
PY

# Timing is generated only after terminal_finalize and both raw documents have
# passed validation.  Failure is recorded by fail() and the EXIT trap.
INPUT_START_MONOTONIC_NS="${INPUT_START_MONOTONIC_NS:?input start}"
DRAIN_END_MONOTONIC_NS="$(python3 -c 'import time; print(time.monotonic_ns())')"
if ! python3 - "${ONLINE_TIMING}" "${ONLINE_TIMING}.part" "${INPUT_START_MONOTONIC_NS}" "${DRAIN_END_MONOTONIC_NS}" "${M6A10_SENSOR_DURATION_SECONDS}" "${M6A10_TIMING_CONTRACT_VERSION}" <<'PY'
import json, math, os, pathlib, sys
final, part = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
start, end, sensor, contract = int(sys.argv[3]), int(sys.argv[4]), float(sys.argv[5]), sys.argv[6]
if final.exists() or final.is_symlink() or part.exists() or part.is_symlink() or end <= start or not math.isfinite(sensor) or sensor <= 0: raise SystemExit("timing bounds")
duration = (end - start) / 1.0e9; rtf = duration / sensor
if not math.isfinite(rtf) or rtf < 0: raise SystemExit("timing RTF")
value = {"schema_version": 1, "contract_version": contract, "status": "PASS",
         "boundary": "input_start_to_drain_end", "input_start_monotonic_ns": start,
         "drain_end_monotonic_ns": end, "duration_seconds": duration,
         "sensor_duration_seconds": sensor, "online_compute_rtf": rtf,
         "ground_truth_content_opened": False, "scorer_invoked": False,
         "map_saved": False}
payload = (json.dumps(value, sort_keys=True, indent=2) + "\n").encode()
fd = os.open(part, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
try:
    with os.fdopen(fd, "wb") as stream:
        stream.write(payload); stream.flush(); os.fsync(stream.fileno())
    os.link(part, final, follow_symlinks=False); os.chmod(final, 0o444)
    dfd = os.open(final.parent, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try: os.fsync(dfd)
    finally: os.close(dfd)
finally:
    try: part.unlink()
    except FileNotFoundError: pass
PY
then
  fail 'online timing persistence'
fi
printf -v container_status_payload '%s\n' 'PASS'
atomic_text "${OUT_DIR}/container_run_status.txt" "${container_status_payload}" 444 || fail 'container status persistence'
write_artifact_manifest 0
stop_group "${ODOM_PID}"; wait "${ODOM_PID}" >/dev/null 2>&1 || true; ODOM_PID=''
stop_group "${MAPPER_PID}"; wait "${MAPPER_PID}" >/dev/null 2>&1 || true; MAPPER_PID=''
stop_group "${ROSCORE_PID}"; wait "${ROSCORE_PID}" >/dev/null 2>&1 || true; ROSCORE_PID=''
exit 0
