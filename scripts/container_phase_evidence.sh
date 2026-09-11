#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# BSD 2-Clause Simplified License. See the repository license headers.

# Thin shell bridge shared by the ROS 2/ROS 1 wrappers.  It deliberately does
# not know about a particular SLAM implementation; all validation and atomic
# JSON handling lives in benchmark_phase_contract.py.

M6A10_PHASE_SCRIPT="${M6A10_PHASE_SCRIPT:-/runner/scripts/benchmark_phase_contract.py}"
M6A10_PHASE_EVENTS="${M6A10_PHASE_EVENTS:-}"
M6A10_PHASE_EVIDENCE="${M6A10_PHASE_EVIDENCE:-}"
M6A10_PHASE_CONTRACT_VERSION="${M6A10_PHASE_CONTRACT_VERSION:-m6a10-online-compute-v1}"
M6A10_PHASE_MODE="${M6A10_PHASE_MODE:-paced_1x}"
M6A10_CONSUMER_EVIDENCE="${M6A10_CONSUMER_EVIDENCE:-}"
M6A10_PHASE_STATUS=0
M6A10_PHASE_FINALIZED=0

m6a10_phase_init() {
  local system="${1:?system is required}"
  local input_duration="${2:?input duration is required}"
  [[ -n "${OUT_DIR:-}" ]] || return 1
  if [[ "${M6A10_PHASE_CONTRACT_VERSION}" == m6a10-online-compute-v2 &&
        "${M6A10_PHASE_MODE}" == unpaced_ack &&
        "${M6A10_ACK_BACKPRESSURE_ENABLED:-0}" != 1 ]]; then
    echo 'v2 unpaced_ack requires an application acknowledgement/backpressure hook' >&2
    return 1
  fi
  M6A10_PHASE_EVENTS="${OUT_DIR}/phase_events.json"
  M6A10_PHASE_EVIDENCE="${OUT_DIR}/phase_evidence.json"
  # The ours wrapper delegates the replay to a child shell.  Export the
  # journal paths so that child can mark the same atomic event document;
  # otherwise the parent would finalize an apparently empty phase.
  export M6A10_PHASE_EVENTS M6A10_PHASE_EVIDENCE
  python3 "${M6A10_PHASE_SCRIPT}" init \
    --output "${M6A10_PHASE_EVENTS}" --system "${system}" \
    --input-duration "${input_duration}" \
    --max-end-gap "${M6A10_MAX_END_GAP_SECONDS:-0.25}" \
    --contract-version "${M6A10_PHASE_CONTRACT_VERSION}" \
    --phase-mode "${M6A10_PHASE_MODE}"
}

m6a10_phase_mark() {
  local name="${1:?phase event name is required}"
  [[ -n "${M6A10_PHASE_EVENTS:-}" ]] || return 1
  python3 "${M6A10_PHASE_SCRIPT}" mark \
    --events "${M6A10_PHASE_EVENTS}" --name "${name}"
}

m6a10_phase_record_trajectory_coverage() {
  local trajectory="${1:?trajectory path is required}"
  local required_timestamp="${2:?required end timestamp is required}"
  [[ -n "${M6A10_PHASE_EVENTS:-}" ]] || return 1
  local last_timestamp
  if last_timestamp="$(python3 - "${trajectory}" <<'PY'
import math
import sys

path = sys.argv[1]
last = None
try:
    with open(path, "rb") as stream:
        for raw in stream:
            text = raw.decode("utf-8", errors="replace").strip()
            if (not text or text.startswith("#") or text.startswith("%") or
                    text.lower().startswith("time,")):
                continue
            token = text.replace(",", " ").split()[0]
            value = float(token)
            # ``rostopic echo -p`` writes header stamps as integer nanoseconds;
            # the phase schema stores Unix seconds.  Keep ordinary TUM seconds
            # unchanged while normalizing this ROS1 representation.
            if abs(value) > 1.0e12:
                value /= 1.0e9
            if math.isfinite(value):
                last = value
except (OSError, ValueError, IndexError):
    last = None
if last is None:
    raise SystemExit(1)
print(last)
PY
)"; then
    local coverage_args=(
      coverage --events "${M6A10_PHASE_EVENTS}"
      --mode trajectory_timestamp_coverage --complete
      --last-timestamp "${last_timestamp}"
      --required-timestamp "${required_timestamp}")
    if [[ -n "${M6A10_DROPPED_MESSAGES:-}" ]]; then
      coverage_args+=(--dropped "${M6A10_DROPPED_MESSAGES}")
    fi
    if [[ -n "${M6A10_QUEUE_OVERFLOW:-}" ]]; then
      coverage_args+=(--queue-overflow "${M6A10_QUEUE_OVERFLOW}")
    fi
    python3 "${M6A10_PHASE_SCRIPT}" "${coverage_args[@]}"
  else
    local coverage_args=(
      coverage --events "${M6A10_PHASE_EVENTS}"
      --mode trajectory_timestamp_coverage)
    if [[ -n "${M6A10_DROPPED_MESSAGES:-}" ]]; then
      coverage_args+=(--dropped "${M6A10_DROPPED_MESSAGES}")
    fi
    if [[ -n "${M6A10_QUEUE_OVERFLOW:-}" ]]; then
      coverage_args+=(--queue-overflow "${M6A10_QUEUE_OVERFLOW}")
    fi
    python3 "${M6A10_PHASE_SCRIPT}" "${coverage_args[@]}"
  fi
}

m6a10_phase_finalize() {
  local exit_status="${1:-$?}"
  local resource_report="${2:-}"
  if [[ "${M6A10_PHASE_FINALIZED}" == 1 ]]; then
    return "${M6A10_PHASE_STATUS}"
  fi
  M6A10_PHASE_FINALIZED=1
  [[ -n "${M6A10_PHASE_EVENTS:-}" && -n "${M6A10_PHASE_EVIDENCE:-}" ]] || {
    M6A10_PHASE_STATUS=1
    return 1
  }
  local args=(
    "${M6A10_PHASE_SCRIPT}" finalize
    --events "${M6A10_PHASE_EVENTS}"
    --output "${M6A10_PHASE_EVIDENCE}"
    --exit-status "${exit_status}"
  )
  if [[ -n "${resource_report}" ]]; then
    args+=(--resource-report "${resource_report}")
  fi
  if [[ "${M6A10_PHASE_CONTRACT_VERSION}" == m6a10-online-compute-v2 &&
        -n "${M6A10_CONSUMER_EVIDENCE}" &&
        -f "${M6A10_CONSUMER_EVIDENCE}" ]]; then
    # The application owns this file.  Missing or malformed evidence is not
    # converted into zeroes; finalization remains fail-closed.
    python3 "${M6A10_PHASE_SCRIPT}" consumer \
      --events "${M6A10_PHASE_EVENTS}" \
      --input "${M6A10_CONSUMER_EVIDENCE}" || true
  fi
  if [[ "${M6A10_PHASE_CONTRACT_VERSION}" == m6a10-online-compute-v2 ]]; then
    local phase_max_callback_latency phase_max_backlog
    phase_max_callback_latency="${M6A10_PHASE_MAX_CALLBACK_LATENCY_SECONDS:-${M6A10_GLIM_MAX_CALLBACK_LATENCY_SECONDS:-0.25}}"
    phase_max_backlog="${M6A10_PHASE_MAX_BACKLOG_MESSAGES:-${M6A10_GLIM_MAX_BACKLOG_MESSAGES:-0}}"
    args+=(--maximum-callback-latency-seconds "${phase_max_callback_latency}"
           --maximum-backlog-messages "${phase_max_backlog}")
  fi
  python3 "${args[@]}"
  M6A10_PHASE_STATUS=$?
  return "${M6A10_PHASE_STATUS}"
}

m6a10_phase_ros2_bag_end() {
  local bag_path="${1:?ROS 2 bag path is required}"
  python3 - "${bag_path}" <<'PY'
import sys
from pathlib import Path

import yaml

info = yaml.safe_load((Path(sys.argv[1]) / "metadata.yaml").read_text())[
    "rosbag2_bagfile_information"]
start = info["starting_time"]["nanoseconds_since_epoch"]
duration = info["duration"]["nanoseconds"]
print((start + duration) / 1e9)
PY
}

m6a10_phase_ros2_bag_duration() {
  local bag_path="${1:?ROS 2 bag path is required}"
  python3 - "${bag_path}" <<'PY'
import sys
from pathlib import Path

import yaml

info = yaml.safe_load((Path(sys.argv[1]) / "metadata.yaml").read_text())[
    "rosbag2_bagfile_information"]
print(info["duration"]["nanoseconds"] / 1e9)
PY
}

m6a10_phase_ros1_bag_end() {
  local info_path="${1:?ROS 1 rosbag info YAML is required}"
  python3 - "${info_path}" <<'PY'
import sys

import yaml

doc = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
start = doc.get("start")
duration = doc.get("duration")
if isinstance(start, dict):
    start = start.get("secs", 0) + start.get("nsecs", 0) / 1e9
if isinstance(duration, dict):
    duration = duration.get("secs", 0) + duration.get("nsecs", 0) / 1e9
if start is None or duration is None:
    raise SystemExit("rosbag info lacks start/duration")
print(float(start) + float(duration))
PY
}

m6a10_phase_ros1_bag_duration() {
  local info_path="${1:?ROS 1 rosbag info YAML is required}"
  python3 - "${info_path}" <<'PY'
import sys

import yaml

doc = yaml.safe_load(open(sys.argv[1], encoding="utf-8")) or {}
duration = doc.get("duration")
if isinstance(duration, dict):
    duration = duration.get("secs", 0) + duration.get("nsecs", 0) / 1e9
if duration is None:
    raise SystemExit("rosbag info lacks duration")
print(float(duration))
PY
}

m6a10_phase_wait_for_trajectory() {
  local trajectory="${1:?trajectory path is required}"
  local required_timestamp="${2:?required end timestamp is required}"
  local timeout_seconds="${3:-60}"
  local deadline=$((SECONDS + timeout_seconds))
  while (( SECONDS < deadline )); do
    if python3 - "${trajectory}" "${required_timestamp}" <<'PY'
import math
import sys

last = None
try:
    with open(sys.argv[1], "rb") as stream:
        for raw in stream:
            text = raw.decode("utf-8", errors="replace").strip()
            if (not text or text.startswith("#") or text.startswith("%") or
                    text.lower().startswith("time,")):
                continue
            value = float(text.replace(",", " ").split()[0])
            if abs(value) > 1.0e12:
                value /= 1.0e9
            if math.isfinite(value):
                last = value
except (OSError, ValueError, IndexError):
    last = None
if last is None or last < float(sys.argv[2]) - 0.25:
    raise SystemExit(1)
PY
    then
      return 0
    fi
    sleep 0.2
  done
  return 1
}
