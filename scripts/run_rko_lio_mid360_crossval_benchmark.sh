#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  run_rko_lio_mid360_crossval_benchmark.sh [options]

Options:
  --bag <dir>                  MID360 rosbag2 directory
  --reference-tum <file>       Reference TUM trajectory (default: output/glim_mid360_reference.tum)
  --lidar-topic <topic>        LiDAR topic (default: /livox/lidar)
  --imu-topic <topic>          IMU topic (default: /livox/imu)
  --base-frame <frame>         Base frame for RKO-LIO (default: livox_frame)
  --odom-frame <frame>         Odom frame (default: odom)
  --lidar-frame <frame>        LiDAR frame override (default: livox_frame)
  --imu-frame <frame>          IMU frame override (default: livox_frame)
  --skip-to-time <s>           Skip this many seconds at bag start (default: 0.0)
  --deskew <bool>              Override RKO-LIO deskew
  --voxel-size <f>             Override RKO-LIO voxel_size (default: 0.5)
  --max-range <f>              Override RKO-LIO max_range (default: 80.0)
  --min-range <f>              Override RKO-LIO min_range
  --initialization-phase <bool>
                               Override RKO-LIO initialization_phase
  --lidarslam-param <file>     graph_based_slam YAML (default: lidarslam/param/lidarslam_mid360_rko_graph.yaml)
  --rko-param <file>           RKO-LIO YAML (default: lidarslam/param/rko_lio_mid360.yaml)
  --threshold-loop-closure-score <f>
                               Override threshold_loop_closure_score
  --scan-context-loop-closure-score-threshold <f>
                               Override scan_context_loop_closure_score_threshold
  --distance-loop-closure <f>  Override distance_loop_closure
  --use-scan-context <bool>    Override use_scan_context
  --scan-context-threshold <f> Override scan_context_threshold
  --use-bev-descriptor <bool>  Override use_bev_descriptor
  --bev-descriptor-threshold <f>
                               Override bev_descriptor_threshold
  --bev-descriptor-sequence-window <n>
                               Override bev_descriptor_sequence_window
  --bev-descriptor-sequence-threshold <f>
                               Override bev_descriptor_sequence_threshold
  --bev-descriptor-pose-consistency-threshold-m <f>
                               Override bev_descriptor_pose_consistency_threshold_m
  --bev-descriptor-max-euclidean-distance-m <f>
                               Override bev_descriptor_max_euclidean_distance_m
  --bev-descriptor-rerank-weight-m <f>
                               Override bev_descriptor_rerank_weight_m
  --use-solid-descriptor <bool> Override use_solid_descriptor
  --solid-descriptor-min-similarity <f>
                               Override solid_descriptor_min_similarity
  --solid-descriptor-sequence-window <n>
                               Override solid_descriptor_sequence_window
  --solid-descriptor-sequence-min-similarity <f>
                               Override solid_descriptor_sequence_min_similarity
  --solid-descriptor-pose-consistency-threshold-m <f>
                               Override solid_descriptor_pose_consistency_threshold_m
  --solid-descriptor-max-euclidean-distance-m <f>
                               Override solid_descriptor_max_euclidean_distance_m
  --prefer-scan-context-candidates <bool>
                               Override prefer_scan_context_candidates
  --use-3d-bbs-for-scan-context <bool>
                               Override use_3d_bbs_for_scan_context
  --use-pcd-cache <bool>       Override use_pcd_cache
  --loop-max-translation-delta <f>
                               Override loop_max_translation_delta in YAML
  --loop-max-rotation-delta-deg <f>
                               Override loop_max_rotation_delta_deg in YAML
  --loop-edge-info-weight <f>  Override loop_edge_info_weight in YAML
  --loop-edge-dedup-index-window <n>
                               Override loop_edge_dedup_index_window in YAML
  --max-loop-candidate-count <n>
                               Override max_loop_candidate_count in YAML
  --search-submap-num <n>      Override search_submap_num in YAML
  --range-of-searching-loop-closure <f>
                               Override range_of_searching_loop_closure in YAML
  --output-dir <dir>           Output directory for logs and artifacts
  --run-name <name>            Run name tag
  --startup-timeout-secs <s>   Timeout waiting for node startup (default: 30)
  --wall-timeout-secs <s>      Timeout waiting for the offline run to finish (default: 1800)
  --quiescence-secs <s>        Stable period used to treat the run as complete (default: 20)
  --save-timeout-secs <s>      Timeout for /map_save and corrected trajectory (default: 60)
  --help                       Show this help

This wrapper runs:
  rosbag2 -> RKO-LIO + graph_based_slam -> raw/corrected TUM -> aligned metrics.json
against the GLIM MID360 reference trajectory.
EOF
  exit 1
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
WS_ROOT="${REPO_ROOT}"
if [[ ! -f "${WS_ROOT}/install/setup.bash" && -f "${REPO_ROOT}/../install/setup.bash" ]]; then
  WS_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
fi

DEFAULT_BAG="${REPO_ROOT}/demo_data/glim_mid360/rosbag2_2024_04_16-14_17_01"
DEFAULT_REFERENCE_TUM="${REPO_ROOT}/output/glim_mid360_reference.tum"
DEFAULT_LIDAR_TOPIC="/livox/lidar"
DEFAULT_IMU_TOPIC="/livox/imu"
DEFAULT_BASE_FRAME="livox_frame"
DEFAULT_ODOM_FRAME="odom"
DEFAULT_LIDAR_FRAME="livox_frame"
DEFAULT_IMU_FRAME="livox_frame"
DEFAULT_SKIP_TO_TIME="0.0"
DEFAULT_DESKEW=""
DEFAULT_VOXEL_SIZE="0.5"
DEFAULT_MAX_RANGE="80.0"
DEFAULT_MIN_RANGE=""
DEFAULT_INITIALIZATION_PHASE=""
DEFAULT_LIDARSLAM_PARAM="${REPO_ROOT}/lidarslam/param/lidarslam_mid360_rko_graph.yaml"
DEFAULT_RKO_PARAM="${REPO_ROOT}/lidarslam/param/rko_lio_mid360.yaml"

BAG_PATH="$DEFAULT_BAG"
REFERENCE_TUM="$DEFAULT_REFERENCE_TUM"
LIDAR_TOPIC="$DEFAULT_LIDAR_TOPIC"
IMU_TOPIC="$DEFAULT_IMU_TOPIC"
BASE_FRAME="$DEFAULT_BASE_FRAME"
ODOM_FRAME="$DEFAULT_ODOM_FRAME"
LIDAR_FRAME="$DEFAULT_LIDAR_FRAME"
IMU_FRAME="$DEFAULT_IMU_FRAME"
SKIP_TO_TIME="$DEFAULT_SKIP_TO_TIME"
DESKEW="$DEFAULT_DESKEW"
VOXEL_SIZE="$DEFAULT_VOXEL_SIZE"
MAX_RANGE="$DEFAULT_MAX_RANGE"
MIN_RANGE="$DEFAULT_MIN_RANGE"
INITIALIZATION_PHASE="$DEFAULT_INITIALIZATION_PHASE"
LIDARSLAM_PARAM="$DEFAULT_LIDARSLAM_PARAM"
RKO_PARAM="$DEFAULT_RKO_PARAM"
THRESHOLD_LOOP_CLOSURE_SCORE="15.0"
SCAN_CONTEXT_LOOP_CLOSURE_SCORE_THRESHOLD=""
DISTANCE_LOOP_CLOSURE="100.0"
USE_SCAN_CONTEXT="false"
SCAN_CONTEXT_THRESHOLD=""
USE_BEV_DESCRIPTOR=""
BEV_DESCRIPTOR_THRESHOLD=""
BEV_DESCRIPTOR_SEQUENCE_WINDOW=""
BEV_DESCRIPTOR_SEQUENCE_THRESHOLD=""
BEV_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M=""
BEV_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M=""
BEV_DESCRIPTOR_RERANK_WEIGHT_M=""
USE_SOLID_DESCRIPTOR=""
SOLID_DESCRIPTOR_MIN_SIMILARITY=""
SOLID_DESCRIPTOR_SEQUENCE_WINDOW=""
SOLID_DESCRIPTOR_SEQUENCE_MIN_SIMILARITY=""
SOLID_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M=""
SOLID_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M=""
PREFER_SCAN_CONTEXT_CANDIDATES=""
USE_3D_BBS_FOR_SCAN_CONTEXT=""
USE_PCD_CACHE="true"
LOOP_MAX_TRANSLATION_DELTA=""
LOOP_MAX_ROTATION_DELTA_DEG=""
LOOP_EDGE_INFO_WEIGHT=""
LOOP_EDGE_DEDUP_INDEX_WINDOW=""
MAX_LOOP_CANDIDATE_COUNT=""
SEARCH_SUBMAP_NUM=""
RANGE_OF_SEARCHING_LOOP_CLOSURE=""
OUTPUT_DIR=""
RUN_NAME=""
STARTUP_TIMEOUT_SECS=30
WALL_TIMEOUT_SECS=1800
QUIESCENCE_SECS=20
SAVE_TIMEOUT_SECS=60

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      [[ $# -ge 2 ]] || usage
      BAG_PATH=$(realpath "$2")
      shift 2
      ;;
    --reference-tum)
      [[ $# -ge 2 ]] || usage
      REFERENCE_TUM=$(realpath "$2")
      shift 2
      ;;
    --lidar-topic)
      [[ $# -ge 2 ]] || usage
      LIDAR_TOPIC="$2"
      shift 2
      ;;
    --imu-topic)
      [[ $# -ge 2 ]] || usage
      IMU_TOPIC="$2"
      shift 2
      ;;
    --base-frame)
      [[ $# -ge 2 ]] || usage
      BASE_FRAME="$2"
      shift 2
      ;;
    --odom-frame)
      [[ $# -ge 2 ]] || usage
      ODOM_FRAME="$2"
      shift 2
      ;;
    --lidar-frame)
      [[ $# -ge 2 ]] || usage
      LIDAR_FRAME="$2"
      shift 2
      ;;
    --imu-frame)
      [[ $# -ge 2 ]] || usage
      IMU_FRAME="$2"
      shift 2
      ;;
    --skip-to-time)
      [[ $# -ge 2 ]] || usage
      SKIP_TO_TIME="$2"
      shift 2
      ;;
    --deskew)
      [[ $# -ge 2 ]] || usage
      DESKEW="$2"
      shift 2
      ;;
    --voxel-size)
      [[ $# -ge 2 ]] || usage
      VOXEL_SIZE="$2"
      shift 2
      ;;
    --max-range)
      [[ $# -ge 2 ]] || usage
      MAX_RANGE="$2"
      shift 2
      ;;
    --min-range)
      [[ $# -ge 2 ]] || usage
      MIN_RANGE="$2"
      shift 2
      ;;
    --initialization-phase)
      [[ $# -ge 2 ]] || usage
      INITIALIZATION_PHASE="$2"
      shift 2
      ;;
    --lidarslam-param)
      [[ $# -ge 2 ]] || usage
      LIDARSLAM_PARAM=$(realpath "$2")
      shift 2
      ;;
    --rko-param)
      [[ $# -ge 2 ]] || usage
      RKO_PARAM=$(realpath "$2")
      shift 2
      ;;
    --threshold-loop-closure-score)
      [[ $# -ge 2 ]] || usage
      THRESHOLD_LOOP_CLOSURE_SCORE="$2"
      shift 2
      ;;
    --scan-context-loop-closure-score-threshold)
      [[ $# -ge 2 ]] || usage
      SCAN_CONTEXT_LOOP_CLOSURE_SCORE_THRESHOLD="$2"
      shift 2
      ;;
    --distance-loop-closure)
      [[ $# -ge 2 ]] || usage
      DISTANCE_LOOP_CLOSURE="$2"
      shift 2
      ;;
    --use-scan-context)
      [[ $# -ge 2 ]] || usage
      USE_SCAN_CONTEXT="$2"
      shift 2
      ;;
    --scan-context-threshold)
      [[ $# -ge 2 ]] || usage
      SCAN_CONTEXT_THRESHOLD="$2"
      shift 2
      ;;
    --use-bev-descriptor)
      [[ $# -ge 2 ]] || usage
      USE_BEV_DESCRIPTOR="$2"
      shift 2
      ;;
    --bev-descriptor-threshold)
      [[ $# -ge 2 ]] || usage
      BEV_DESCRIPTOR_THRESHOLD="$2"
      shift 2
      ;;
    --bev-descriptor-sequence-window)
      [[ $# -ge 2 ]] || usage
      BEV_DESCRIPTOR_SEQUENCE_WINDOW="$2"
      shift 2
      ;;
    --bev-descriptor-sequence-threshold)
      [[ $# -ge 2 ]] || usage
      BEV_DESCRIPTOR_SEQUENCE_THRESHOLD="$2"
      shift 2
      ;;
    --bev-descriptor-pose-consistency-threshold-m)
      [[ $# -ge 2 ]] || usage
      BEV_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M="$2"
      shift 2
      ;;
    --bev-descriptor-max-euclidean-distance-m)
      [[ $# -ge 2 ]] || usage
      BEV_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M="$2"
      shift 2
      ;;
    --bev-descriptor-rerank-weight-m)
      [[ $# -ge 2 ]] || usage
      BEV_DESCRIPTOR_RERANK_WEIGHT_M="$2"
      shift 2
      ;;
    --use-solid-descriptor)
      [[ $# -ge 2 ]] || usage
      USE_SOLID_DESCRIPTOR="$2"
      shift 2
      ;;
    --solid-descriptor-min-similarity)
      [[ $# -ge 2 ]] || usage
      SOLID_DESCRIPTOR_MIN_SIMILARITY="$2"
      shift 2
      ;;
    --solid-descriptor-sequence-window)
      [[ $# -ge 2 ]] || usage
      SOLID_DESCRIPTOR_SEQUENCE_WINDOW="$2"
      shift 2
      ;;
    --solid-descriptor-sequence-min-similarity)
      [[ $# -ge 2 ]] || usage
      SOLID_DESCRIPTOR_SEQUENCE_MIN_SIMILARITY="$2"
      shift 2
      ;;
    --solid-descriptor-pose-consistency-threshold-m)
      [[ $# -ge 2 ]] || usage
      SOLID_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M="$2"
      shift 2
      ;;
    --solid-descriptor-max-euclidean-distance-m)
      [[ $# -ge 2 ]] || usage
      SOLID_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M="$2"
      shift 2
      ;;
    --prefer-scan-context-candidates)
      [[ $# -ge 2 ]] || usage
      PREFER_SCAN_CONTEXT_CANDIDATES="$2"
      shift 2
      ;;
    --use-3d-bbs-for-scan-context)
      [[ $# -ge 2 ]] || usage
      USE_3D_BBS_FOR_SCAN_CONTEXT="$2"
      shift 2
      ;;
    --use-pcd-cache)
      [[ $# -ge 2 ]] || usage
      USE_PCD_CACHE="$2"
      shift 2
      ;;
    --loop-max-translation-delta)
      [[ $# -ge 2 ]] || usage
      LOOP_MAX_TRANSLATION_DELTA="$2"
      shift 2
      ;;
    --loop-max-rotation-delta-deg)
      [[ $# -ge 2 ]] || usage
      LOOP_MAX_ROTATION_DELTA_DEG="$2"
      shift 2
      ;;
    --loop-edge-info-weight)
      [[ $# -ge 2 ]] || usage
      LOOP_EDGE_INFO_WEIGHT="$2"
      shift 2
      ;;
    --loop-edge-dedup-index-window)
      [[ $# -ge 2 ]] || usage
      LOOP_EDGE_DEDUP_INDEX_WINDOW="$2"
      shift 2
      ;;
    --max-loop-candidate-count)
      [[ $# -ge 2 ]] || usage
      MAX_LOOP_CANDIDATE_COUNT="$2"
      shift 2
      ;;
    --search-submap-num)
      [[ $# -ge 2 ]] || usage
      SEARCH_SUBMAP_NUM="$2"
      shift 2
      ;;
    --range-of-searching-loop-closure)
      [[ $# -ge 2 ]] || usage
      RANGE_OF_SEARCHING_LOOP_CLOSURE="$2"
      shift 2
      ;;
    --output-dir)
      [[ $# -ge 2 ]] || usage
      OUTPUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --run-name)
      [[ $# -ge 2 ]] || usage
      RUN_NAME="$2"
      shift 2
      ;;
    --startup-timeout-secs)
      [[ $# -ge 2 ]] || usage
      STARTUP_TIMEOUT_SECS="$2"
      shift 2
      ;;
    --wall-timeout-secs)
      [[ $# -ge 2 ]] || usage
      WALL_TIMEOUT_SECS="$2"
      shift 2
      ;;
    --quiescence-secs)
      [[ $# -ge 2 ]] || usage
      QUIESCENCE_SECS="$2"
      shift 2
      ;;
    --save-timeout-secs)
      [[ $# -ge 2 ]] || usage
      SAVE_TIMEOUT_SECS="$2"
      shift 2
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

if [[ "${USE_3D_BBS_FOR_SCAN_CONTEXT}" == "true" ]] && (( QUIESCENCE_SECS < 120 )); then
  QUIESCENCE_SECS=120
fi

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/bench_rko_lio_mid360_$(date +%Y%m%d_%H%M%S)"
fi
if [[ -z "$RUN_NAME" ]]; then
  RUN_NAME="$(basename "$OUTPUT_DIR")"
fi

[[ -d "$BAG_PATH" ]] || { echo "rosbag2 directory not found: $BAG_PATH" >&2; exit 1; }
[[ -f "$BAG_PATH/metadata.yaml" ]] || { echo "metadata.yaml not found under $BAG_PATH" >&2; exit 1; }
[[ -f "$REFERENCE_TUM" ]] || { echo "reference TUM not found: $REFERENCE_TUM" >&2; exit 1; }
[[ -f "$LIDARSLAM_PARAM" ]] || { echo "graph param file not found: $LIDARSLAM_PARAM" >&2; exit 1; }
[[ -f "$RKO_PARAM" ]] || { echo "RKO-LIO param file not found: $RKO_PARAM" >&2; exit 1; }

set +u
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
set -u

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found in PATH" >&2; exit 1; }
command -v python3 >/dev/null 2>&1 || { echo "python3 not found in PATH" >&2; exit 1; }

mkdir -p "$OUTPUT_DIR"
if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${OUTPUT_DIR}/.ros_log"
fi
mkdir -p "$ROS_LOG_DIR"

STARTED_AT="$(date -Iseconds)"
STARTED_AT_UNIX="$(date +%s)"
BENCH_T0="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"

LAUNCH_LOG="${OUTPUT_DIR}/slam.launch.log"
MAP_SAVE_LOG="${OUTPUT_DIR}/map_save.log"
RAW_TUM="${OUTPUT_DIR}/traj_raw.tum"
CORRECTED_TUM="${OUTPUT_DIR}/traj_corrected.tum"
RAW_LOG="${OUTPUT_DIR}/odom_logger.log"
CORRECTED_LOG="${OUTPUT_DIR}/path_logger.log"
RKO_ROS_PARAM_FILE="${OUTPUT_DIR}/rko_params.ros.yaml"
GRAPH_PARAM_FILE="${OUTPUT_DIR}/graph_params.effective.yaml"
RKO_RESULT_TUM="${OUTPUT_DIR}/${RUN_NAME}_0/${RUN_NAME}_tum_0.txt"

LAUNCH_PID=""
LAUNCH_PGID=""
RAW_LOGGER_PID=""
CORRECTED_LOGGER_PID=""

terminate_pid() {
  local pid="${1:-}"
  [[ -n "$pid" ]] || return 0
  kill "$pid" >/dev/null 2>&1 || true
  for _ in {1..10}; do
    if ! kill -0 "$pid" >/dev/null 2>&1; then
      wait "$pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.2
  done
  kill -9 "$pid" >/dev/null 2>&1 || true
  wait "$pid" 2>/dev/null || true
}

terminate_process_group() {
  local pgid="${1:-}"
  local leader_pid="${2:-}"
  [[ -n "$pgid" ]] || return 0
  kill -- "-${pgid}" >/dev/null 2>&1 || true
  for _ in {1..10}; do
    if [[ -n "$leader_pid" ]] && ! kill -0 "$leader_pid" >/dev/null 2>&1; then
      wait "$leader_pid" 2>/dev/null || true
      return 0
    fi
    sleep 0.2
  done
  kill -9 -- "-${pgid}" >/dev/null 2>&1 || true
  if [[ -n "$leader_pid" ]]; then
    wait "$leader_pid" 2>/dev/null || true
  fi
}

cleanup() {
  terminate_pid "$RAW_LOGGER_PID"
  terminate_pid "$CORRECTED_LOGGER_PID"
  if [[ -n "$LAUNCH_PGID" ]]; then
    terminate_process_group "$LAUNCH_PGID" "$LAUNCH_PID"
  else
    terminate_pid "$LAUNCH_PID"
  fi
}
trap cleanup EXIT INT TERM

wait_for_log_pattern() {
  local pattern="$1"
  local timeout_secs="$2"
  local deadline=$((SECONDS + timeout_secs))
  while (( SECONDS < deadline )); do
    if grep -Fq "$pattern" "$LAUNCH_LOG" 2>/dev/null; then
      return 0
    fi
    if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      return 1
    fi
    sleep 1
  done
  return 1
}

wait_for_map_outputs() {
  local timeout_secs="$1"
  local deadline=$((SECONDS + timeout_secs))
  while (( SECONDS < deadline )); do
    if [[ -f "$OUTPUT_DIR/map_projector_info.yaml" && -f "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_corrected_trajectory() {
  local timeout_secs="$1"
  local deadline=$((SECONDS + timeout_secs))
  while (( SECONDS < deadline )); do
    if [[ -s "$CORRECTED_TUM" ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

run_state_signature() {
  python3 - "$RAW_TUM" "$CORRECTED_TUM" "$LAUNCH_LOG" <<'PY'
import os
import sys

parts = []
for path in sys.argv[1:]:
    if not path or not os.path.exists(path):
        parts.append("missing")
        continue
    st = os.stat(path)
    parts.append(f"{int(st.st_mtime)}:{st.st_size}")
print("|".join(parts))
PY
}

offline_completion_recorded() {
  if [[ -s "$RKO_RESULT_TUM" ]]; then
    return 0
  fi
  if grep -Fq "RKO LIO Offline Node took" "$LAUNCH_LOG" 2>/dev/null; then
    return 0
  fi
  if [[ -f "$OUTPUT_DIR/rko_lio.log" ]] && grep -Fq "RKO LIO Offline Node took" "$OUTPUT_DIR/rko_lio.log" 2>/dev/null; then
    return 0
  fi
  return 1
}

call_map_save_with_retry() {
  local deadline=$((SECONDS + SAVE_TIMEOUT_SECS))
  while (( SECONDS < deadline )); do
    if timeout 15 ros2 service call /map_save std_srvs/srv/Empty "{}" >"${MAP_SAVE_LOG}" 2>&1; then
      return 0
    fi
    sleep 2
  done
  return 1
}

wait_for_offline_completion() {
  local timeout_secs="$1"
  local last_signature=""
  local stable_since=0
  local deadline=$((SECONDS + timeout_secs))

  while (( SECONDS < deadline )); do
    if offline_completion_recorded; then
      return 0
    fi
    if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      return 0
    fi

    local current_signature
    current_signature="$(run_state_signature)"
    if [[ "$current_signature" == "$last_signature" ]] && [[ -f "$RAW_TUM" ]]; then
      if (( stable_since == 0 )); then
        stable_since=$SECONDS
      fi
      if (( SECONDS - stable_since >= QUIESCENCE_SECS )); then
        echo "No new trajectory/log updates for ${QUIESCENCE_SECS}s; treating run as complete"
        return 0
      fi
    else
      stable_since=0
      last_signature="$current_signature"
    fi

    sleep 2
  done

  return 1
}

python3 - "$RKO_PARAM" "$RKO_ROS_PARAM_FILE" <<'PY'
import shutil
import sys
from pathlib import Path

import yaml

src_path = Path(sys.argv[1])
dst_path = Path(sys.argv[2])
data = yaml.safe_load(src_path.read_text()) or {}

if isinstance(data, dict) and any(
    isinstance(v, dict) and "ros__parameters" in v for v in data.values()
):
    shutil.copyfile(src_path, dst_path)
    sys.exit(0)

wrapped = {"/**": {"ros__parameters": data}}
dst_path.write_text(yaml.safe_dump(wrapped, sort_keys=False))
PY

python3 - "$LIDARSLAM_PARAM" "$GRAPH_PARAM_FILE" \
  "$LOOP_MAX_TRANSLATION_DELTA" "$LOOP_MAX_ROTATION_DELTA_DEG" \
  "$LOOP_EDGE_INFO_WEIGHT" "$LOOP_EDGE_DEDUP_INDEX_WINDOW" \
  "$MAX_LOOP_CANDIDATE_COUNT" "$SEARCH_SUBMAP_NUM" \
  "$RANGE_OF_SEARCHING_LOOP_CLOSURE" "$SCAN_CONTEXT_THRESHOLD" \
  "$USE_BEV_DESCRIPTOR" "$BEV_DESCRIPTOR_THRESHOLD" \
  "$BEV_DESCRIPTOR_SEQUENCE_WINDOW" "$BEV_DESCRIPTOR_SEQUENCE_THRESHOLD" \
  "$BEV_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M" \
  "$BEV_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M" \
  "$BEV_DESCRIPTOR_RERANK_WEIGHT_M" \
  "$USE_SOLID_DESCRIPTOR" "$SOLID_DESCRIPTOR_MIN_SIMILARITY" \
  "$SOLID_DESCRIPTOR_SEQUENCE_WINDOW" "$SOLID_DESCRIPTOR_SEQUENCE_MIN_SIMILARITY" \
  "$SOLID_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M" \
  "$SOLID_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M" \
  "$PREFER_SCAN_CONTEXT_CANDIDATES" \
  "$SCAN_CONTEXT_LOOP_CLOSURE_SCORE_THRESHOLD" \
  "$USE_3D_BBS_FOR_SCAN_CONTEXT" <<'PY'
import sys
from pathlib import Path

import yaml

src_path = Path(sys.argv[1])
dst_path = Path(sys.argv[2])
loop_max_translation_delta = sys.argv[3]
loop_max_rotation_delta_deg = sys.argv[4]
loop_edge_info_weight = sys.argv[5]
loop_edge_dedup_index_window = sys.argv[6]
max_loop_candidate_count = sys.argv[7]
search_submap_num = sys.argv[8]
range_of_searching_loop_closure = sys.argv[9]
scan_context_threshold = sys.argv[10]
use_bev_descriptor = sys.argv[11]
bev_descriptor_threshold = sys.argv[12]
bev_descriptor_sequence_window = sys.argv[13]
bev_descriptor_sequence_threshold = sys.argv[14]
bev_descriptor_pose_consistency_threshold_m = sys.argv[15]
bev_descriptor_max_euclidean_distance_m = sys.argv[16]
bev_descriptor_rerank_weight_m = sys.argv[17]
use_solid_descriptor = sys.argv[18]
solid_descriptor_min_similarity = sys.argv[19]
solid_descriptor_sequence_window = sys.argv[20]
solid_descriptor_sequence_min_similarity = sys.argv[21]
solid_descriptor_pose_consistency_threshold_m = sys.argv[22]
solid_descriptor_max_euclidean_distance_m = sys.argv[23]
prefer_scan_context_candidates = sys.argv[24]
scan_context_loop_closure_score_threshold = sys.argv[25]
use_3d_bbs_for_scan_context = sys.argv[26]

data = yaml.safe_load(src_path.read_text()) or {}
params = data.setdefault('graph_based_slam', {}).setdefault('ros__parameters', {})

def maybe_float(text: str):
    return float(text) if text != '' else None

def maybe_int(text: str):
    return int(text) if text != '' else None

if loop_max_translation_delta != '':
    params['loop_max_translation_delta'] = maybe_float(loop_max_translation_delta)
if loop_max_rotation_delta_deg != '':
    params['loop_max_rotation_delta_deg'] = maybe_float(loop_max_rotation_delta_deg)
if loop_edge_info_weight != '':
    params['loop_edge_info_weight'] = maybe_float(loop_edge_info_weight)
if loop_edge_dedup_index_window != '':
    params['loop_edge_dedup_index_window'] = maybe_int(loop_edge_dedup_index_window)
if max_loop_candidate_count != '':
    params['max_loop_candidate_count'] = maybe_int(max_loop_candidate_count)
if search_submap_num != '':
    params['search_submap_num'] = maybe_int(search_submap_num)
if range_of_searching_loop_closure != '':
    params['range_of_searching_loop_closure'] = maybe_float(range_of_searching_loop_closure)
if scan_context_threshold != '':
    params['scan_context_threshold'] = maybe_float(scan_context_threshold)
if use_bev_descriptor != '':
    params['use_bev_descriptor'] = use_bev_descriptor.lower() == 'true'
if bev_descriptor_threshold != '':
    params['bev_descriptor_threshold'] = maybe_float(bev_descriptor_threshold)
if bev_descriptor_sequence_window != '':
    params['bev_descriptor_sequence_window'] = maybe_int(bev_descriptor_sequence_window)
if bev_descriptor_sequence_threshold != '':
    params['bev_descriptor_sequence_threshold'] = maybe_float(bev_descriptor_sequence_threshold)
if bev_descriptor_pose_consistency_threshold_m != '':
    params['bev_descriptor_pose_consistency_threshold_m'] = maybe_float(
        bev_descriptor_pose_consistency_threshold_m)
if bev_descriptor_max_euclidean_distance_m != '':
    params['bev_descriptor_max_euclidean_distance_m'] = maybe_float(
        bev_descriptor_max_euclidean_distance_m)
if bev_descriptor_rerank_weight_m != '':
    params['bev_descriptor_rerank_weight_m'] = maybe_float(
        bev_descriptor_rerank_weight_m)
if use_solid_descriptor != '':
    params['use_solid_descriptor'] = use_solid_descriptor.lower() == 'true'
if solid_descriptor_min_similarity != '':
    params['solid_descriptor_min_similarity'] = maybe_float(solid_descriptor_min_similarity)
if solid_descriptor_sequence_window != '':
    params['solid_descriptor_sequence_window'] = maybe_int(solid_descriptor_sequence_window)
if solid_descriptor_sequence_min_similarity != '':
    params['solid_descriptor_sequence_min_similarity'] = maybe_float(
        solid_descriptor_sequence_min_similarity)
if solid_descriptor_pose_consistency_threshold_m != '':
    params['solid_descriptor_pose_consistency_threshold_m'] = maybe_float(
        solid_descriptor_pose_consistency_threshold_m)
if solid_descriptor_max_euclidean_distance_m != '':
    params['solid_descriptor_max_euclidean_distance_m'] = maybe_float(
        solid_descriptor_max_euclidean_distance_m)
if prefer_scan_context_candidates != '':
    params['prefer_scan_context_candidates'] = prefer_scan_context_candidates.lower() == 'true'
if scan_context_loop_closure_score_threshold != '':
    params['scan_context_loop_closure_score_threshold'] = maybe_float(
        scan_context_loop_closure_score_threshold)
if use_3d_bbs_for_scan_context != '':
    params['use_3d_bbs_for_scan_context'] = use_3d_bbs_for_scan_context.lower() == 'true'

dst_path.write_text(yaml.safe_dump(data, sort_keys=False))
PY

echo "Running MID360 RKO-LIO + graph_based_slam cross-validation benchmark"
echo "  bag:            $BAG_PATH"
echo "  reference_tum:  $REFERENCE_TUM"
echo "  lidar_topic:    $LIDAR_TOPIC"
echo "  imu_topic:      $IMU_TOPIC"
echo "  base_frame:     $BASE_FRAME"
echo "  odom_frame:     $ODOM_FRAME"
echo "  skip_to_time:   $SKIP_TO_TIME"
[[ -n "$DESKEW" ]] && echo "  deskew:         $DESKEW"
[[ -n "$VOXEL_SIZE" ]] && echo "  voxel_size:     $VOXEL_SIZE"
[[ -n "$MAX_RANGE" ]] && echo "  max_range:      $MAX_RANGE"
[[ -n "$MIN_RANGE" ]] && echo "  min_range:      $MIN_RANGE"
[[ -n "$INITIALIZATION_PHASE" ]] && echo "  init_phase:     $INITIALIZATION_PHASE"
echo "  lidarslam_yaml: $GRAPH_PARAM_FILE"
[[ -n "$SCAN_CONTEXT_THRESHOLD" ]] && echo "  scan_context_threshold: $SCAN_CONTEXT_THRESHOLD"
[[ -n "$USE_BEV_DESCRIPTOR" ]] && echo "  use_bev_descriptor: $USE_BEV_DESCRIPTOR"
[[ -n "$BEV_DESCRIPTOR_THRESHOLD" ]] && echo "  bev_descriptor_threshold: $BEV_DESCRIPTOR_THRESHOLD"
[[ -n "$BEV_DESCRIPTOR_SEQUENCE_WINDOW" ]] && echo "  bev_descriptor_sequence_window: $BEV_DESCRIPTOR_SEQUENCE_WINDOW"
[[ -n "$BEV_DESCRIPTOR_SEQUENCE_THRESHOLD" ]] && echo "  bev_descriptor_sequence_threshold: $BEV_DESCRIPTOR_SEQUENCE_THRESHOLD"
[[ -n "$BEV_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M" ]] && echo "  bev_descriptor_pose_consistency_threshold_m: $BEV_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M"
[[ -n "$BEV_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M" ]] && echo "  bev_descriptor_max_euclidean_distance_m: $BEV_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M"
[[ -n "$BEV_DESCRIPTOR_RERANK_WEIGHT_M" ]] && echo "  bev_descriptor_rerank_weight_m: $BEV_DESCRIPTOR_RERANK_WEIGHT_M"
[[ -n "$USE_SOLID_DESCRIPTOR" ]] && echo "  use_solid_descriptor: $USE_SOLID_DESCRIPTOR"
[[ -n "$SOLID_DESCRIPTOR_MIN_SIMILARITY" ]] && echo "  solid_descriptor_min_similarity: $SOLID_DESCRIPTOR_MIN_SIMILARITY"
[[ -n "$SOLID_DESCRIPTOR_SEQUENCE_WINDOW" ]] && echo "  solid_descriptor_sequence_window: $SOLID_DESCRIPTOR_SEQUENCE_WINDOW"
[[ -n "$SOLID_DESCRIPTOR_SEQUENCE_MIN_SIMILARITY" ]] && echo "  solid_descriptor_sequence_min_similarity: $SOLID_DESCRIPTOR_SEQUENCE_MIN_SIMILARITY"
[[ -n "$SOLID_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M" ]] && echo "  solid_descriptor_pose_consistency_threshold_m: $SOLID_DESCRIPTOR_POSE_CONSISTENCY_THRESHOLD_M"
[[ -n "$SOLID_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M" ]] && echo "  solid_descriptor_max_euclidean_distance_m: $SOLID_DESCRIPTOR_MAX_EUCLIDEAN_DISTANCE_M"
[[ -n "$PREFER_SCAN_CONTEXT_CANDIDATES" ]] && echo "  prefer_scan_context_candidates: $PREFER_SCAN_CONTEXT_CANDIDATES"
[[ -n "$SCAN_CONTEXT_LOOP_CLOSURE_SCORE_THRESHOLD" ]] && echo "  scan_context_loop_closure_score_threshold: $SCAN_CONTEXT_LOOP_CLOSURE_SCORE_THRESHOLD"
[[ -n "$USE_3D_BBS_FOR_SCAN_CONTEXT" ]] && echo "  use_3d_bbs_for_scan_context: $USE_3D_BBS_FOR_SCAN_CONTEXT"
echo "  rko_yaml:       $RKO_PARAM"
echo "  output_dir:     $OUTPUT_DIR"
echo "  run_name:       $RUN_NAME"

if command -v setsid >/dev/null 2>&1; then
  setsid ros2 launch lidarslam rko_lio_slam.launch.py \
    "main_param_dir:=${GRAPH_PARAM_FILE}" \
    "rko_param_file:=${RKO_ROS_PARAM_FILE}" \
    "bag_path:=${BAG_PATH}" \
    "lidar_topic:=${LIDAR_TOPIC}" \
    "imu_topic:=${IMU_TOPIC}" \
    "base_frame:=${BASE_FRAME}" \
    "odom_frame:=${ODOM_FRAME}" \
    "lidar_frame:=${LIDAR_FRAME}" \
    "imu_frame:=${IMU_FRAME}" \
    "skip_to_time:=${SKIP_TO_TIME}" \
    ${DESKEW:+"deskew:=${DESKEW}"} \
    ${VOXEL_SIZE:+"voxel_size:=${VOXEL_SIZE}"} \
    ${MAX_RANGE:+"max_range:=${MAX_RANGE}"} \
    ${MIN_RANGE:+"min_range:=${MIN_RANGE}"} \
    ${INITIALIZATION_PHASE:+"initialization_phase:=${INITIALIZATION_PHASE}"} \
    "publish_static_tf:=false" \
    "threshold_loop_closure_score:=${THRESHOLD_LOOP_CLOSURE_SCORE}" \
    "distance_loop_closure:=${DISTANCE_LOOP_CLOSURE}" \
    "use_scan_context:=${USE_SCAN_CONTEXT}" \
    "use_pcd_cache:=${USE_PCD_CACHE}" \
    "save_dir:=${OUTPUT_DIR}" \
    "results_dir:=${OUTPUT_DIR}" \
    "run_name:=${RUN_NAME}" \
    "dump_results:=true" \
    "use_rviz:=false" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
  LAUNCH_PGID="$LAUNCH_PID"
else
  ros2 launch lidarslam rko_lio_slam.launch.py \
    "main_param_dir:=${GRAPH_PARAM_FILE}" \
    "rko_param_file:=${RKO_ROS_PARAM_FILE}" \
    "bag_path:=${BAG_PATH}" \
    "lidar_topic:=${LIDAR_TOPIC}" \
    "imu_topic:=${IMU_TOPIC}" \
    "base_frame:=${BASE_FRAME}" \
    "odom_frame:=${ODOM_FRAME}" \
    "lidar_frame:=${LIDAR_FRAME}" \
    "imu_frame:=${IMU_FRAME}" \
    "skip_to_time:=${SKIP_TO_TIME}" \
    ${DESKEW:+"deskew:=${DESKEW}"} \
    ${VOXEL_SIZE:+"voxel_size:=${VOXEL_SIZE}"} \
    ${MAX_RANGE:+"max_range:=${MAX_RANGE}"} \
    ${MIN_RANGE:+"min_range:=${MIN_RANGE}"} \
    ${INITIALIZATION_PHASE:+"initialization_phase:=${INITIALIZATION_PHASE}"} \
    "publish_static_tf:=false" \
    "threshold_loop_closure_score:=${THRESHOLD_LOOP_CLOSURE_SCORE}" \
    "distance_loop_closure:=${DISTANCE_LOOP_CLOSURE}" \
    "use_scan_context:=${USE_SCAN_CONTEXT}" \
    "use_pcd_cache:=${USE_PCD_CACHE}" \
    "save_dir:=${OUTPUT_DIR}" \
    "results_dir:=${OUTPUT_DIR}" \
    "run_name:=${RUN_NAME}" \
    "dump_results:=true" \
    "use_rviz:=false" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
fi

python3 "${SCRIPT_DIR}/odom_to_tum.py" \
  --topic /rko_lio/odometry \
  --output "$RAW_TUM" \
  --use-sim-time false \
  >"${RAW_LOG}" 2>&1 &
RAW_LOGGER_PID="$!"

python3 "${SCRIPT_DIR}/path_to_tum.py" \
  --topic /modified_path \
  --output "$CORRECTED_TUM" \
  --use-sim-time false \
  >"${CORRECTED_LOG}" 2>&1 &
CORRECTED_LOGGER_PID="$!"

if ! wait_for_log_pattern "RKO LIO Node is up!" "$STARTUP_TIMEOUT_SECS"; then
  echo "Timed out waiting for RKO-LIO startup. Recent launch log:" >&2
  tail -n 80 "$LAUNCH_LOG" >&2 || true
  exit 1
fi
if ! wait_for_log_pattern "[graph_based_slam]: initialization end" "$STARTUP_TIMEOUT_SECS"; then
  echo "Timed out waiting for graph_based_slam startup. Recent launch log:" >&2
  tail -n 80 "$LAUNCH_LOG" >&2 || true
  exit 1
fi

echo "SLAM launch is up"
if ! wait_for_offline_completion "$WALL_TIMEOUT_SECS"; then
  echo "Launch terminated before offline node completed. Recent launch log:" >&2
  tail -n 120 "$LAUNCH_LOG" >&2 || true
  exit 1
fi

if [[ -f "$OUTPUT_DIR/map_projector_info.yaml" && -f "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml" && -s "$CORRECTED_TUM" ]]; then
  echo "Map outputs and corrected trajectory already exist; skipping /map_save"
else
  echo "Calling /map_save ..."
  if ! call_map_save_with_retry; then
    if [[ -f "$OUTPUT_DIR/map_projector_info.yaml" && -f "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml" && -s "$CORRECTED_TUM" ]]; then
      echo "map_save service unavailable, but usable outputs already exist; continuing" >&2
    else
      echo "map_save service call failed. Recent launch log:" >&2
      tail -n 120 "$LAUNCH_LOG" >&2 || true
      cat "$MAP_SAVE_LOG" >&2 || true
      exit 1
    fi
  fi

  if ! wait_for_map_outputs "$SAVE_TIMEOUT_SECS"; then
    echo "Timed out waiting for saved map outputs under $OUTPUT_DIR" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    exit 1
  fi

  if ! wait_for_corrected_trajectory "$SAVE_TIMEOUT_SECS"; then
    echo "corrected trajectory not found or empty after /map_save: $CORRECTED_TUM" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    exit 1
  fi
fi

terminate_pid "$RAW_LOGGER_PID"
terminate_pid "$CORRECTED_LOGGER_PID"
RAW_LOGGER_PID=""
CORRECTED_LOGGER_PID=""

if [[ -n "$LAUNCH_PGID" ]]; then
  terminate_process_group "$LAUNCH_PGID" "$LAUNCH_PID"
elif [[ -n "$LAUNCH_PID" ]]; then
  terminate_pid "$LAUNCH_PID"
fi
LAUNCH_PID=""
LAUNCH_PGID=""

BENCH_T1="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"
WALL_SEC="$(python3 - "$BENCH_T0" "$BENCH_T1" <<'PY'
import sys
print(float(sys.argv[2]) - float(sys.argv[1]))
PY
)"

python3 "${SCRIPT_DIR}/write_aligned_trajectory_metrics.py" \
  --out-dir "$OUTPUT_DIR" \
  --bag "$BAG_PATH" \
  --reference-tum "$REFERENCE_TUM" \
  --corrected-tum "$CORRECTED_TUM" \
  --raw-tum "$RAW_TUM" \
  --graph-log "$LAUNCH_LOG" \
  --lidarslam-param "$GRAPH_PARAM_FILE" \
  --parameter-file "$GRAPH_PARAM_FILE" \
  --parameter-file "$RKO_ROS_PARAM_FILE" \
  --benchmark-harness "${BASH_SOURCE[0]}" \
  --runtime-artifact "rko_lio_offline_node=$(ros2 pkg prefix rko_lio)/lib/rko_lio/offline_node" \
  --runtime-artifact "graph_based_slam_node=$(ros2 pkg prefix graph_based_slam)/lib/graph_based_slam/graph_based_slam_node" \
  --points-topic "$LIDAR_TOPIC" \
  --points-frame "$LIDAR_FRAME" \
  --robot-frame "$BASE_FRAME" \
  --odom-frame "$ODOM_FRAME" \
  --reference-source "glim_mid360_reference" \
  --reference-kind "cross_validation" \
  --reference-label "GLIM" \
  --glim-traj "$REFERENCE_TUM" \
  --wall-sec "$WALL_SEC" \
  --started-at "$STARTED_AT" \
  --started-at-unix "$STARTED_AT_UNIX"

echo "MID360 benchmark completed"
echo "  output_dir:  $OUTPUT_DIR"
echo "  metrics:     $OUTPUT_DIR/metrics.json"
