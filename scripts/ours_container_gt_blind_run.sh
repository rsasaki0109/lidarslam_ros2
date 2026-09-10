#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

set -euo pipefail

: "${BAG_PATH:?BAG_PATH is required}"
: "${OUT_DIR:?OUT_DIR is required}"
: "${LIDAR_TOPIC:?LIDAR_TOPIC is required}"
: "${IMU_TOPIC:?IMU_TOPIC is required}"

# ros2 launch writes its own log before the benchmark node starts.  The image
# root is read-only, so keep that state inside the attempt output just like
# the other benchmark wrappers.
export ROS_HOME="${ROS_HOME:-${OUT_DIR}/ros_home}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-${OUT_DIR}/ros_log}"
mkdir -p "${ROS_HOME}" "${ROS_LOG_DIR}"
source /runner/scripts/container_phase_evidence.sh
source /runner/scripts/container_memory_evidence.sh
if [[ "${M6A10_PHASE_CONTRACT_VERSION:-}" == "m6a10-online-compute-v2" ]]; then
  # The phase bridge is consumed by a nested benchmark shell and by the
  # launch-created OfflineNode process.  Keep the values explicitly exported
  # after both helper sources; relying on the export attribute of a Docker
  # environment variable is too easy to break when a helper assigns a
  # default.  Missing values fail before the input reader starts.
  [[ "${M6A10_PHASE_MODE:-}" == "paced_1x" ||
     "${M6A10_PHASE_MODE:-}" == "unpaced_ack" ]] || {
    echo 'v2 phase mode is missing or unsupported' >&2
    exit 12
  }
  # The RKO-LIO offline node owns this application-level evidence.  It is
  # enabled only for the additive v2 benchmark contract; normal replay never
  # receives this environment variable or writes the file.
  export M6A10_CONSUMER_EVIDENCE="${OUT_DIR}/consumer_evidence.json"
  export M6A10_DRAIN_TIMEOUT_SECONDS="${M6A10_DRAIN_TIMEOUT_SECONDS:-30}"
  export M6A10_DRAIN_DIAGNOSTIC="${M6A10_DRAIN_DIAGNOSTIC:-${OUT_DIR}/m6a10_drain_diagnostic.json}"
  case "${M6A10_DRAIN_DIAGNOSTIC}" in
    "${OUT_DIR}"/*) ;;
    *) echo 'v2 drain diagnostic path must stay inside output directory' >&2; exit 12 ;;
  esac
  export M6A10_PHASE_CONTRACT_VERSION M6A10_PHASE_MODE M6A10_CONSUMER_EVIDENCE
  export M6A10_DRAIN_TIMEOUT_SECONDS M6A10_DRAIN_DIAGNOSTIC
  if [[ "${M6A10_PHASE_MODE:-paced_1x}" == "unpaced_ack" ]]; then
    export M6A10_ACK_BACKPRESSURE_ENABLED=1
  fi
fi
# ``--skip-map-save`` must cover loop-triggered map writes as well as the
# service call.  This is an additive benchmark-only marker consumed by the
# source launch file selected by the v2 runner; normal launches never set it.
if [[ "${M6A10_SKIP_MAP_SAVE:-0}" == "1" ]]; then
  export M6A10_BENCHMARK_NO_MAP_ARTIFACTS=1
else
  unset M6A10_BENCHMARK_NO_MAP_ARTIFACTS
fi
trap 'm6a5_container_exit_trap "$?"' EXIT
m6a5_install_container_signal_traps
if ! m6a7_start_process_rss_sampler; then
  echo 'OURS process RSS sampler failed to start' >&2
  exit 11
fi

set +u
source /opt/ros/jazzy/setup.bash
source /opt/ours_ws/install/setup.bash
set -u

# The image recipe must build the pinned RKO-LIO package explicitly.  Check
# the installed package and executable before touching the input bag so a
# broken image fails with an actionable provenance/runtime error.
RKO_PREFIX="$(ros2 pkg prefix rko_lio 2>/dev/null)" || {
  echo 'error: rko_lio is not discoverable in the pinned runtime image' >&2
  exit 12
}
[[ -f "${RKO_PREFIX}/share/rko_lio/package.xml" ]] || {
  echo "error: rko_lio package manifest is missing under ${RKO_PREFIX}" >&2
  exit 12
}
[[ -x "${RKO_PREFIX}/lib/rko_lio/offline_node" ]] || {
  echo "error: rko_lio offline_node is missing under ${RKO_PREFIX}" >&2
  exit 12
}
RKO_CONFIG="${RKO_PARAM:-/runner/configs/hilti2022/rko_lio_hilti2022_pandar.yaml}"
[[ -f "${RKO_CONFIG}" ]] || {
  echo "error: pinned RKO-LIO benchmark config is missing: ${RKO_CONFIG}" >&2
  exit 12
}

OURS_INPUT_DURATION="${M6A10_SENSOR_DURATION_SECONDS:-}"
if [[ -z "${OURS_INPUT_DURATION}" ]]; then
  OURS_INPUT_DURATION="$(m6a10_phase_ros2_bag_duration "${BAG_PATH}")" || {
    echo 'OURS could not determine canonical bag duration for phase evidence' >&2
    exit 12
  }
fi
m6a10_phase_init ours "${OURS_INPUT_DURATION}" || exit 12
M6A10_RESOURCE_REPORT="${OUT_DIR}/compute_time.txt"
export M6A10_PHASE_EVENTS M6A10_PHASE_EVIDENCE M6A10_RESOURCE_REPORT

if [[ "${M6A10_CONSUMER_ONLY:-0}" == "1" ]]; then
  # Synthetic v2 fixtures may contain too few scans to produce a graph
  # trajectory.  Exercise the real RKO BufferableBag/callback consumer path
  # without starting graph_based_slam or map-save; this mode is explicitly
  # benchmark-only and is never selected by the production driver.
  consumer_param="${OUT_DIR}/consumer_only_params.yaml"
  python3 - "${RKO_CONFIG}" "${consumer_param}" "${BAG_PATH}" \
    "${LIDAR_TOPIC}" "${IMU_TOPIC}" <<'PY'
import os
import sys
from pathlib import Path

import yaml

source, target, bag, lidar, imu = map(Path, sys.argv[1:])
data = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
blocks = []
if isinstance(data, dict):
    blocks = [
        value["ros__parameters"] for value in data.values()
        if isinstance(value, dict) and isinstance(
            value.get("ros__parameters"), dict)]
    parameters = blocks[0] if blocks else data
else:
    parameters = {}
parameters.update({
    "bag_path": str(bag),
    "lidar_topic": str(lidar),
    "imu_topic": str(imu),
    "base_frame": "base_link",
    "results_dir": str(target.parent),
    "run_name": "m6a10_consumer_only",
    "dump_results": False,
    "m6a10_phase_contract_version": os.environ.get(
        "M6A10_PHASE_CONTRACT_VERSION", ""),
    "m6a10_phase_mode": os.environ.get("M6A10_PHASE_MODE", ""),
    "m6a10_consumer_evidence_path": os.environ.get(
        "M6A10_CONSUMER_EVIDENCE", ""),
})
if not blocks:
    data = {"/**": {"ros__parameters": parameters}}
target.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
PY
  m6a10_phase_mark startup_end || true
  m6a10_phase_mark input_start || true
  consumer_log="${OUT_DIR}/consumer_only.log"
  set +e
  timeout --signal=TERM --kill-after=5s \
    "${M6A10_CONSUMER_ONLY_TIMEOUT_SECS:-120}" \
    bash /runner/scripts/run_with_resource_report.sh \
    "${M6A10_RESOURCE_REPORT}" \
    ros2 run rko_lio offline_node --ros-args \
    --params-file "${consumer_param}" >"${consumer_log}" 2>&1
  consumer_status=$?
  set -e
  m6a10_phase_mark input_end || true
  m6a10_phase_mark drain_start || true
  m6a10_phase_mark drain_end || true
  m6a10_phase_mark postprocess_start || true
  m6a10_phase_mark postprocess_end || true
  m6a10_phase_mark save_start || true
  m6a10_phase_mark save_end || true
  printf '%s\n' \
    '{"status":"pass","consumer_only":true,"map_save_skipped":true,"gt_mounted":false,"scorer_invoked":false}' \
    >"${OUT_DIR}/consumer_only_smoke_contract.json"
  exit "${consumer_status}"
fi

if [[ "${M6A3_SYNTHETIC_SMOKE:-0}" == "1" ]]; then
  # Synthetic smoke deliberately exercises the real launch graph, but stops
  # before replay/scoring.  It is not a benchmark and never accepts a GT
  # path.  Keep the temporary ROS parameter wrapper inside the attempt dir.
  mkdir -p "${OUT_DIR}"
  smoke_param="${OUT_DIR}/rko_params.yaml"
  python3 - "${RKO_PARAM:-/runner/configs/hilti2022/rko_lio_hilti2022_pandar.yaml}" "${smoke_param}" <<'PY'
import shutil
import sys
from pathlib import Path

import yaml

source = Path(sys.argv[1])
target = Path(sys.argv[2])
data = yaml.safe_load(source.read_text()) or {}
if isinstance(data, dict) and any(
    isinstance(value, dict) and "ros__parameters" in value
    for value in data.values()
):
    shutil.copyfile(source, target)
else:
    target.write_text(yaml.safe_dump({"/**": {"ros__parameters": data}}, sort_keys=False))
PY
  launch_log="${OUT_DIR}/synthetic_smoke_launch.log"
  setsid ros2 launch lidarslam rko_lio_slam.launch.py \
    "main_param_dir:=/runner/lidarslam/param/lidarslam.yaml" \
    "rko_param_file:=${smoke_param}" \
    "bag_path:=${BAG_PATH}" \
    "lidar_topic:=${LIDAR_TOPIC}" \
    "imu_topic:=${IMU_TOPIC}" \
    "base_frame:=base_link" "publish_static_tf:=false" \
    "save_dir:=${OUT_DIR}" "results_dir:=${OUT_DIR}" \
    "run_name:=m6a3_synthetic_smoke" "dump_results:=false" \
    "use_rviz:=false" >"${launch_log}" 2>&1 &
  smoke_pid=$!
  started=0
  for _ in $(seq 1 "${SMOKE_STARTUP_TIMEOUT_SECS:-30}"); do
    if grep -Fq 'RKO LIO Node is up!' "${launch_log}" 2>/dev/null && \
       grep -Fq '[graph_based_slam]: initialization end' "${launch_log}" 2>/dev/null; then
      started=1
      break
    fi
    if ! kill -0 "${smoke_pid}" >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
  kill -INT -- "-${smoke_pid}" >/dev/null 2>&1 || true
  wait "${smoke_pid}" >/dev/null 2>&1 || true
  if [[ "${started}" != "1" ]]; then
    echo 'error: synthetic RKO launch did not reach both startup markers' >&2
    tail -n 80 "${launch_log}" >&2 || true
    exit 13
  fi
  printf '%s\n' '{"status":"pass","startup_verified":true,"input_verified":true,"clean_shutdown":true,"gt_mounted":false,"performance_run":false}' >"${OUT_DIR}/synthetic_smoke_contract.json"
  exit 0
fi

RKO_BENCHMARK_ARGS=(
  --bag "${BAG_PATH}"
  --lidar-topic "${LIDAR_TOPIC}"
  --imu-topic "${IMU_TOPIC}"
  --rko-param "${RKO_CONFIG}"
  --lidarslam-param /runner/lidarslam/param/lidarslam.yaml
  --output-dir "${OUT_DIR}"
  --run-name "${RUN_NAME:-m6a}"
  --offline-timeout-secs "${OFFLINE_TIMEOUT_SECS:-7200}"
  --save-timeout-secs "${SAVE_TIMEOUT_SECS:-600}"
  --completion-end-margin-secs "${COMPLETION_END_MARGIN_SECONDS:-0.25}"
  --gt-blind
)
if [[ "${M6A10_SKIP_MAP_SAVE:-0}" == "1" ]]; then
  # Synthetic v2 fixtures exercise consumer/phase finalization only.  This
  # opt-in keeps map-service behavior out of that contract and is never set
  # by the production GT-blind driver.
  RKO_BENCHMARK_ARGS+=(--skip-map-save)
fi
bash /runner/scripts/run_rko_lio_graph_benchmark.sh "${RKO_BENCHMARK_ARGS[@]}"
status=$?
exit "${status}"
