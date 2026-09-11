#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SOURCE_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
if [[ -f "${SOURCE_ROOT}/lidarslam/package.xml" ]]; then
  PACKAGE_SHARE="${SOURCE_ROOT}/lidarslam"
  WORK_ROOT="${SOURCE_ROOT}"
  WORKSPACE_SETUP=""
  if [[ -f "${SOURCE_ROOT}/install/setup.bash" ]]; then
    WORKSPACE_SETUP="${SOURCE_ROOT}/install/setup.bash"
  elif [[ -f "${SOURCE_ROOT}/../install/setup.bash" ]]; then
    WORKSPACE_SETUP="$(cd "${SOURCE_ROOT}/.." && pwd)/install/setup.bash"
  fi
else
  PACKAGE_SHARE=$(cd "${SCRIPT_DIR}/../.." && pwd)
  INSTALL_PREFIX=$(cd "${PACKAGE_SHARE}/../.." && pwd)
  WORK_ROOT="${PWD}"
  WORKSPACE_SETUP="${INSTALL_PREFIX}/setup.bash"
fi

usage() {
  local exit_code="${1:-1}"
  cat <<'EOF' >&2
Usage:
  run_rko_lio_graph_autoware_dogfood.sh [options]

Options:
  --bag <dir>                    rosbag2 directory
  --lidar-topic <topic>          LiDAR topic
  --imu-topic <topic>            IMU topic
  --lidarslam-param <file>       graph_based_slam parameter YAML
  --rko-param <file>             RKO-LIO parameter YAML
  --base-frame <frame>           Robot base frame passed to RKO-LIO
  --lidar-frame <frame>          LiDAR frame override
  --imu-frame <frame>            IMU frame override
  --output-dir <dir>             Directory for SLAM outputs and logs
  --run-name <name>              RKO-LIO run_name
  --save-timeout-secs <sec>      Timeout waiting for saved map files (default: 60)
  --startup-timeout-secs <sec>   Timeout waiting for SLAM node startup (default: 30)
  --offline-quiet-log-secs <sec> Treat an unchanged launch log after first odom/cloud
                                 as offline completion after N seconds (default: 0, disabled)
  --wait-for-offline-completion  Wait for the full offline bag run to finish before saving
  --graph-drain-secs <sec>       After offline completion, wait additional N seconds
                                 with no launch-log changes before calling /map_save.
                                 Use this for long bags where graph_based_slam still
                                 has buffered submaps to process (default: 0, disabled).
  --capture-corrected-path BOOL  Stage final trajectory_optimized.tum as traj_corrected.tum;
                                 custom topics use live capture (default: true).
  --corrected-path-topic TOPIC   Custom nav_msgs/Path live-capture topic
                                 (default: /modified_path uses final optimized output).
  --capture-raw-odometry BOOL    Write complete frontend traj_raw.tum from RKO-LIO's native
                                 dump; custom topics use live capture (default: true).
  --raw-odometry-topic TOPIC     Custom nav_msgs/Odometry live-capture topic
                                 (default: /rko_lio/odometry uses the native dump).
  --generate-lanelet2 BOOL       Generate lanelet2_map.osm from traj_corrected.tum (default: true).
  --origin-lat <deg>             Origin latitude for lanelet2 local coordinates (default: 0.0).
  --origin-lon <deg>             Origin longitude for lanelet2 local coordinates (default: 0.0).
  --lane-width <m>               Lane width for generated lanelet2 map (default: 3.5).
  --reference-tum <file>         Reference TUM. When provided, ape_from_tum.py runs after
                                 /map_save and traj_corrected_ape.txt is written under the
                                 output directory.
  --viewer-run-dir <dir>         Reuse an existing built Autoware map-loader run directory
  --viewer-rebuild               Rebuild the minimal Autoware runtime workspace before viewing
  --auto-exit-secs <sec>         Auto-close RViz after N seconds
  --autoware-core-dir <dir>      autoware_core checkout for the viewer
  --work-dir <dir>               Runtime workspace directory for the viewer
  --keep-launch                  Keep the SLAM launch alive after map save
  --skip-viewer                  Stop after verified map output without opening a viewer
  --help                         Show this help

Defaults target the NTU VIRAL tnp_01 restamped VN100 rosbag2 currently stored in this repository.
The script runs RKO-LIO + graph_based_slam, waits for offline odometry to finish, calls /map_save,
then stages the resulting map for Autoware and opens it in the host's rviz2.
EOF
  exit "$exit_code"
}

fail() {
  echo "error: $1" >&2
  if [[ $# -gt 1 ]]; then
    echo "hint: $2" >&2
  fi
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "$value" || "$value" == --* ]]; then
    fail "option requires a value: ${option}" \
      "run this script with --help for valid options."
  fi
}

parse_bool() {
  local option="$1"
  local value="$2"
  case "${value,,}" in
    true|1|yes) echo true ;;
    false|0|no) echo false ;;
    *) fail "${option} expects true or false." \
      "accepted values: true, false, 1, 0, yes, no." ;;
  esac
}

DEFAULT_BAG="${WORK_ROOT}/demo_data/ntu_viral/tnp_01_points_restamped_vn100_rosbag2"
DEFAULT_LIDAR_TOPIC="/os1_cloud_node1/points"
DEFAULT_IMU_TOPIC="/imu/imu"
DEFAULT_BASE_FRAME="base_link"
DEFAULT_LIDAR_FRAME=""
DEFAULT_IMU_FRAME=""
DEFAULT_LIDARSLAM_PARAM="${PACKAGE_SHARE}/param/lidarslam.yaml"
DEFAULT_RKO_PARAM="${PACKAGE_SHARE}/param/rko_lio_ntu_viral.yaml"
DEFAULT_AUTOWARE_CORE="/tmp/autoware_core"
DEFAULT_WORK_DIR="/tmp/autoware_map_runtime_ws"
PRODUCT_SESSION_OUTPUT="${LIDARSLAM_PRODUCT_SESSION_OUTPUT:-full}"

BAG_PATH="$DEFAULT_BAG"
LIDAR_TOPIC="$DEFAULT_LIDAR_TOPIC"
IMU_TOPIC="$DEFAULT_IMU_TOPIC"
BASE_FRAME="$DEFAULT_BASE_FRAME"
LIDAR_FRAME="$DEFAULT_LIDAR_FRAME"
IMU_FRAME="$DEFAULT_IMU_FRAME"
LIDARSLAM_PARAM="$DEFAULT_LIDARSLAM_PARAM"
RKO_PARAM="$DEFAULT_RKO_PARAM"
OUTPUT_DIR=""
RUN_NAME=""
SAVE_TIMEOUT_SECS=60
STARTUP_TIMEOUT_SECS=30
OFFLINE_QUIET_LOG_SECS=0
GRAPH_DRAIN_SECS=0
VIEWER_RUN_DIR=""
VIEWER_REBUILD=false
AUTO_EXIT_SECS=""
AUTOWARE_CORE_DIR="$DEFAULT_AUTOWARE_CORE"
WORK_DIR="$DEFAULT_WORK_DIR"
KEEP_LAUNCH=false
WAIT_FOR_OFFLINE_COMPLETION=false
SKIP_VIEWER=false
CAPTURE_CORRECTED_PATH=true
CORRECTED_PATH_TOPIC="/modified_path"
CAPTURE_RAW_ODOMETRY=true
RAW_ODOMETRY_TOPIC="/rko_lio/odometry"
GENERATE_LANELET2=true
ORIGIN_LAT="0.0"
ORIGIN_LON="0.0"
LANE_WIDTH="3.5"
REFERENCE_TUM=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag)
      require_value "$1" "${2:-}"
      BAG_PATH=$(realpath -m "$2")
      shift 2
      ;;
    --lidar-topic)
      require_value "$1" "${2:-}"
      LIDAR_TOPIC="$2"
      shift 2
      ;;
    --imu-topic)
      require_value "$1" "${2:-}"
      IMU_TOPIC="$2"
      shift 2
      ;;
    --base-frame)
      require_value "$1" "${2:-}"
      BASE_FRAME="$2"
      shift 2
      ;;
    --lidar-frame)
      require_value "$1" "${2:-}"
      LIDAR_FRAME="$2"
      shift 2
      ;;
    --imu-frame)
      require_value "$1" "${2:-}"
      IMU_FRAME="$2"
      shift 2
      ;;
    --lidarslam-param)
      require_value "$1" "${2:-}"
      LIDARSLAM_PARAM=$(realpath -m "$2")
      shift 2
      ;;
    --rko-param)
      require_value "$1" "${2:-}"
      RKO_PARAM=$(realpath -m "$2")
      shift 2
      ;;
    --output-dir)
      require_value "$1" "${2:-}"
      OUTPUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --run-name)
      require_value "$1" "${2:-}"
      RUN_NAME="$2"
      shift 2
      ;;
    --save-timeout-secs)
      require_value "$1" "${2:-}"
      SAVE_TIMEOUT_SECS="$2"
      shift 2
      ;;
    --startup-timeout-secs)
      require_value "$1" "${2:-}"
      STARTUP_TIMEOUT_SECS="$2"
      shift 2
      ;;
    --offline-quiet-log-secs)
      require_value "$1" "${2:-}"
      OFFLINE_QUIET_LOG_SECS="$2"
      shift 2
      ;;
    --graph-drain-secs)
      require_value "$1" "${2:-}"
      GRAPH_DRAIN_SECS="$2"
      shift 2
      ;;
    --viewer-run-dir)
      require_value "$1" "${2:-}"
      VIEWER_RUN_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --wait-for-offline-completion)
      WAIT_FOR_OFFLINE_COMPLETION=true
      shift
      ;;
    --viewer-rebuild)
      VIEWER_REBUILD=true
      shift
      ;;
    --auto-exit-secs)
      require_value "$1" "${2:-}"
      AUTO_EXIT_SECS="$2"
      shift 2
      ;;
    --autoware-core-dir)
      require_value "$1" "${2:-}"
      AUTOWARE_CORE_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --work-dir)
      require_value "$1" "${2:-}"
      WORK_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --keep-launch)
      KEEP_LAUNCH=true
      shift
      ;;
    --skip-viewer)
      SKIP_VIEWER=true
      shift
      ;;
    --capture-corrected-path)
      require_value "$1" "${2:-}"
      CAPTURE_CORRECTED_PATH=$(parse_bool "$1" "$2")
      shift 2
      ;;
    --corrected-path-topic)
      require_value "$1" "${2:-}"
      CORRECTED_PATH_TOPIC="$2"
      shift 2
      ;;
    --capture-raw-odometry)
      require_value "$1" "${2:-}"
      CAPTURE_RAW_ODOMETRY=$(parse_bool "$1" "$2")
      shift 2
      ;;
    --raw-odometry-topic)
      require_value "$1" "${2:-}"
      RAW_ODOMETRY_TOPIC="$2"
      shift 2
      ;;
    --generate-lanelet2)
      require_value "$1" "${2:-}"
      GENERATE_LANELET2=$(parse_bool "$1" "$2")
      shift 2
      ;;
    --origin-lat)
      require_value "$1" "${2:-}"
      ORIGIN_LAT="$2"
      shift 2
      ;;
    --origin-lon)
      require_value "$1" "${2:-}"
      ORIGIN_LON="$2"
      shift 2
      ;;
    --lane-width)
      require_value "$1" "${2:-}"
      LANE_WIDTH="$2"
      shift 2
      ;;
    --reference-tum)
      require_value "$1" "${2:-}"
      REFERENCE_TUM=$(realpath -m "$2")
      shift 2
      ;;
    --help|-h)
      usage 0
      ;;
    *)
      fail "unknown option: $1" \
        "run this script with --help for valid options."
      ;;
  esac
done

default_bag_missing_hint() {
  cat >&2 <<EOF
Default NTU VIRAL dogfood bag not found: ${DEFAULT_BAG}

Prepare it with:
  bash scripts/download_ntu_viral_tnp01.sh
EOF
}

if [[ -z "$OUTPUT_DIR" ]]; then
  OUTPUT_DIR="${WORK_ROOT}/output/dogfood_rko_lio_autoware_$(date +%Y%m%d_%H%M%S)"
fi

if [[ -z "$RUN_NAME" ]]; then
  RUN_NAME="$(basename "$OUTPUT_DIR")"
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "error: rosbag2 directory not found: $BAG_PATH" >&2
  if [[ "$BAG_PATH" == "$DEFAULT_BAG" ]]; then
    default_bag_missing_hint
  else
    echo "hint: pass the rosbag2 directory that contains metadata.yaml." >&2
  fi
  exit 2
fi
[[ -f "$BAG_PATH/metadata.yaml" ]] ||
  fail "metadata.yaml not found under $BAG_PATH" \
    "pass the rosbag2 directory, not a .db3 file or parent folder."
[[ -f "$LIDARSLAM_PARAM" ]] ||
  fail "lidarslam param file not found: $LIDARSLAM_PARAM"
[[ -f "$RKO_PARAM" ]] ||
  fail "RKO-LIO param file not found: $RKO_PARAM"
if [[ -e "$OUTPUT_DIR" && ! -d "$OUTPUT_DIR" ]]; then
  fail "output directory path is a file, not a directory: $OUTPUT_DIR" \
    "choose a directory path for generated map outputs and logs."
fi
if [[ "$SKIP_VIEWER" == "false" ]]; then
  [[ -d "$AUTOWARE_CORE_DIR" ]] ||
    fail "autoware_core directory not found: $AUTOWARE_CORE_DIR" \
      "pass --autoware-core-dir, or use --skip-viewer for map output only."
  if [[ -n "$VIEWER_RUN_DIR" && ! -d "$VIEWER_RUN_DIR" ]]; then
    fail "viewer run directory not found: $VIEWER_RUN_DIR" \
      "omit --viewer-run-dir to let the viewer build or discover one."
  fi
fi

CALLER_PATH="$PATH"
set +u
if [[ -n "${WORKSPACE_SETUP}" && -f "${WORKSPACE_SETUP}" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_SETUP}"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
set -u
export PATH="${CALLER_PATH}:${PATH}"

command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found in PATH" >&2; exit 1; }

mkdir -p "$OUTPUT_DIR"
if [[ -z "${ROS_LOG_DIR:-}" ]]; then
  export ROS_LOG_DIR="${OUTPUT_DIR}/.ros_log"
fi
mkdir -p "$ROS_LOG_DIR"

LAUNCH_LOG="${OUTPUT_DIR}/slam.launch.log"
MAP_SAVE_LOG="${OUTPUT_DIR}/map_save.log"
RKO_ROS_PARAM_FILE="${OUTPUT_DIR}/rko_params.ros.yaml"
GRAPH_ROS_PARAM_FILE="${OUTPUT_DIR}/graph_params.ros.yaml"
CORRECTED_TUM="${OUTPUT_DIR}/traj_corrected.tum"
CORRECTED_LOG="${OUTPUT_DIR}/path_corrected_logger.log"
CORRECTED_APE_REPORT="${OUTPUT_DIR}/traj_corrected_ape.txt"
RAW_TUM="${OUTPUT_DIR}/traj_raw.tum"
RAW_LOG="${OUTPUT_DIR}/odom_raw_logger.log"
LANELET2_OSM="${OUTPUT_DIR}/lanelet2_map.osm"
PATH_TO_TUM_SCRIPT="${SCRIPT_DIR}/path_to_tum.py"
ODOM_TO_TUM_SCRIPT="${SCRIPT_DIR}/odom_to_tum.py"
APE_FROM_TUM_SCRIPT="${SCRIPT_DIR}/ape_from_tum.py"
LAUNCH_PID=""
LAUNCH_PGID=""
CORRECTED_LOGGER_PID=""
RAW_LOGGER_PID=""
USE_FINAL_OPTIMIZED_TRAJECTORY=false
USE_NATIVE_RAW_TRAJECTORY=false
declare -A EXISTING_RKO_DUMP_DIRS=()
KEEP_RUNNING=0

[[ -n "$REFERENCE_TUM" && ! -f "$REFERENCE_TUM" ]] && { echo "--reference-tum file not found: $REFERENCE_TUM" >&2; exit 1; }

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
cp -f "$LIDARSLAM_PARAM" "$GRAPH_ROS_PARAM_FILE"

cleanup() {
  if [[ "$KEEP_RUNNING" -eq 1 ]]; then
    return
  fi
  if [[ -n "$CORRECTED_LOGGER_PID" ]]; then
    kill "$CORRECTED_LOGGER_PID" >/dev/null 2>&1 || true
    wait "$CORRECTED_LOGGER_PID" 2>/dev/null || true
    CORRECTED_LOGGER_PID=""
  fi
  if [[ -n "$RAW_LOGGER_PID" ]]; then
    kill "$RAW_LOGGER_PID" >/dev/null 2>&1 || true
    wait "$RAW_LOGGER_PID" 2>/dev/null || true
    RAW_LOGGER_PID=""
  fi
  if [[ -n "$LAUNCH_PGID" ]]; then
    kill -- "-${LAUNCH_PGID}" >/dev/null 2>&1 || true
    if [[ -n "$LAUNCH_PID" ]]; then
      wait "$LAUNCH_PID" 2>/dev/null || true
    fi
  elif [[ -n "$LAUNCH_PID" ]]; then
    kill "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  LAUNCH_PID=""
  LAUNCH_PGID=""
}

on_signal() {
  local exit_code="$1"
  trap - INT TERM
  KEEP_RUNNING=0
  cleanup
  trap - EXIT
  exit "$exit_code"
}

trap cleanup EXIT
trap 'on_signal 130' INT
trap 'on_signal 143' TERM

stage_native_raw_trajectory() {
  local run_dir
  local candidate
  local selected=""
  local -a candidates=()

  shopt -s nullglob
  for run_dir in "$OUTPUT_DIR"/"${RUN_NAME}"_[0-9]*; do
    if [[ -n "${EXISTING_RKO_DUMP_DIRS[$run_dir]+present}" ]]; then
      continue
    fi
    candidates=("$run_dir"/"${RUN_NAME}"_tum_*.txt)
    if (( ${#candidates[@]} == 1 )); then
      selected="${candidates[0]}"
      break
    fi
  done
  shopt -u nullglob

  if [[ -z "$selected" ]]; then
    echo "Native RKO-LIO trajectory was not produced for this run under $OUTPUT_DIR." >&2
    return 1
  fi
  cp -- "$selected" "$RAW_TUM"
  echo "Raw trajectory staged from complete native RKO-LIO results: $RAW_TUM ($(wc -l < "$RAW_TUM") poses)"
}

stage_final_optimized_trajectory() {
  local optimized_tum="${OUTPUT_DIR}/trajectory_optimized.tum"
  if [[ ! -s "$optimized_tum" ]]; then
    echo "Final optimized trajectory was not produced: $optimized_tum" >&2
    return 1
  fi
  cp -- "$optimized_tum" "$CORRECTED_TUM"
  echo "Corrected trajectory staged from final graph optimization: $CORRECTED_TUM ($(wc -l < "$CORRECTED_TUM") poses)"
}

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

map_output_snapshot() {
  if [[ ! -f "$OUTPUT_DIR/map_projector_info.yaml" || ! -f "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]]; then
    return 1
  fi

  {
    stat -c 'projector %Y %s' "$OUTPUT_DIR/map_projector_info.yaml"
    stat -c 'metadata %Y %s' "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml"
    find "$OUTPUT_DIR/pointcloud_map" -maxdepth 1 -type f -name '*.pcd' -printf 'pcd %f %T@ %s\n' | sort
  }
}

wait_for_graph_drain() {
  local drain_secs="$1"
  if (( drain_secs <= 0 )); then
    return 0
  fi
  echo "Waiting for graph_based_slam to drain: ${drain_secs}s without new launch-log activity ..."
  local last_log_size
  last_log_size=$(stat -c %s "$LAUNCH_LOG" 2>/dev/null || echo 0)
  local last_change_secs=$SECONDS
  local timeout_deadline=$((SECONDS + drain_secs * 4))
  while (( SECONDS < timeout_deadline )); do
    local current_log_size
    current_log_size=$(stat -c %s "$LAUNCH_LOG" 2>/dev/null || echo 0)
    if [[ "$current_log_size" != "$last_log_size" ]]; then
      last_log_size="$current_log_size"
      last_change_secs=$SECONDS
    elif (( SECONDS - last_change_secs >= drain_secs )); then
      echo "graph_based_slam log quiet for ${drain_secs}s; proceeding to /map_save."
      return 0
    fi
    if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      return 0
    fi
    sleep 2
  done
  echo "Graph drain wait timed out after $((drain_secs * 4))s; proceeding anyway." >&2
  return 0
}

wait_for_offline_completion() {
  local timeout_secs="$1"
  local quiet_secs="$2"
  local deadline=$((SECONDS + timeout_secs))
  local last_snapshot=""
  local last_change_secs=$SECONDS
  local last_log_size=-1
  local last_log_change_secs=$SECONDS

  while (( SECONDS < deadline )); do
    if grep -Fq "RKO LIO offline processing complete" "$LAUNCH_LOG" 2>/dev/null; then
      return 0
    fi

    if grep -Fq "offline-subscriber-barrier-timeout" "$LAUNCH_LOG" 2>/dev/null ||
      grep -Fq "offline-subscriber-barrier-invalid" "$LAUNCH_LOG" 2>/dev/null; then
      return 1
    fi

    if [[ -f "$LAUNCH_LOG" ]]; then
      local current_log_size
      current_log_size=$(stat -c %s "$LAUNCH_LOG" 2>/dev/null || echo 0)
      if [[ "$current_log_size" != "$last_log_size" ]]; then
        last_log_size="$current_log_size"
        last_log_change_secs=$SECONDS
      fi
    fi

    if snapshot=$(map_output_snapshot 2>/dev/null); then
      if [[ "$snapshot" != "$last_snapshot" ]]; then
        last_snapshot="$snapshot"
        last_change_secs=$SECONDS
      elif (( SECONDS - last_change_secs >= quiet_secs )); then
        return 0
      fi
    fi

    if (( OFFLINE_QUIET_LOG_SECS > 0 )) &&
      grep -Fq "First cloud received" "$LAUNCH_LOG" 2>/dev/null &&
      grep -Fq "First odom received" "$LAUNCH_LOG" 2>/dev/null &&
      (( SECONDS - last_log_change_secs >= OFFLINE_QUIET_LOG_SECS )); then
      echo "No launch log changes for ${OFFLINE_QUIET_LOG_SECS}s after first odom/cloud; treating offline processing as quiescent."
      return 0
    fi

    if [[ -n "$LAUNCH_PID" ]] && ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      return 1
    fi
    sleep 2
  done
  return 1
}

if [[ "$PRODUCT_SESSION_OUTPUT" != "concise" ]]; then
  echo "Running end-to-end dogfood pipeline"
  echo "  bag:            $BAG_PATH"
  echo "  lidar_topic:    $LIDAR_TOPIC"
  echo "  imu_topic:      $IMU_TOPIC"
  echo "  base_frame:     $BASE_FRAME"
  echo "  lidar_frame:    $LIDAR_FRAME"
  echo "  imu_frame:      $IMU_FRAME"
  echo "  lidarslam_yaml: $LIDARSLAM_PARAM"
  echo "  rko_yaml:       $RKO_PARAM"
  echo "  output_dir:     $OUTPUT_DIR"
  echo "  run_name:       $RUN_NAME"
  echo "  rko_ros_param:  $RKO_ROS_PARAM_FILE"
  echo "  graph_param:    $GRAPH_ROS_PARAM_FILE"
fi

MIN_ODOM_SUBSCRIBERS=1
if [[ "$CAPTURE_RAW_ODOMETRY" == "true" ]]; then
  if [[ "$RAW_ODOMETRY_TOPIC" == "/rko_lio/odometry" ]]; then
    USE_NATIVE_RAW_TRAJECTORY=true
    shopt -s nullglob
    for run_dir in "$OUTPUT_DIR"/"${RUN_NAME}"_[0-9]*; do
      EXISTING_RKO_DUMP_DIRS["$run_dir"]=1
    done
    shopt -u nullglob
    if [[ "$PRODUCT_SESSION_OUTPUT" != "concise" ]]; then
      echo "Will stage the complete native RKO-LIO trajectory after offline processing"
    fi
  elif [[ ! -f "$ODOM_TO_TUM_SCRIPT" ]]; then
    echo "Warning: $ODOM_TO_TUM_SCRIPT not found; skipping raw odometry capture." >&2
  else
    echo "Preparing custom live capture $RAW_ODOMETRY_TOPIC -> $RAW_TUM before offline playback"
    python3 "$ODOM_TO_TUM_SCRIPT" \
      --topic "$RAW_ODOMETRY_TOPIC" \
      --output "$RAW_TUM" \
      --use-sim-time false \
      >"$RAW_LOG" 2>&1 &
    RAW_LOGGER_PID="$!"
    MIN_ODOM_SUBSCRIBERS=2
  fi
fi

if [[ "$CAPTURE_CORRECTED_PATH" == "true" ]]; then
  if [[ "$CORRECTED_PATH_TOPIC" == "/modified_path" ]]; then
    USE_FINAL_OPTIMIZED_TRAJECTORY=true
    if [[ "$PRODUCT_SESSION_OUTPUT" != "concise" ]]; then
      echo "Will stage the final graph-optimized trajectory after map save"
    fi
  elif [[ ! -f "$PATH_TO_TUM_SCRIPT" ]]; then
    echo "Warning: $PATH_TO_TUM_SCRIPT not found; skipping /modified_path capture." >&2
  else
    echo "Preparing custom live capture $CORRECTED_PATH_TOPIC -> $CORRECTED_TUM before offline playback"
    python3 "$PATH_TO_TUM_SCRIPT" \
      --topic "$CORRECTED_PATH_TOPIC" \
      --output "$CORRECTED_TUM" \
      --use-sim-time false \
      >"$CORRECTED_LOG" 2>&1 &
    CORRECTED_LOGGER_PID="$!"
  fi
fi

LAUNCH_ARGS=(
  "main_param_dir:=${LIDARSLAM_PARAM}"
  "rko_param_file:=${RKO_ROS_PARAM_FILE}"
  "bag_path:=${BAG_PATH}"
  "lidar_topic:=${LIDAR_TOPIC}"
  "imu_topic:=${IMU_TOPIC}"
  "base_frame:=${BASE_FRAME}"
  "save_dir:=${OUTPUT_DIR}"
  "results_dir:=${OUTPUT_DIR}"
  "run_name:=${RUN_NAME}"
  "dump_results:=true"
  "wait_for_output_subscribers:=true"
  "min_odom_subscribers:=${MIN_ODOM_SUBSCRIBERS}"
  "min_deskewed_scan_subscribers:=1"
  "subscriber_wait_timeout_sec:=${STARTUP_TIMEOUT_SECS}"
  "subscriber_settle_polls:=3"
  "use_rviz:=false"
)
if [[ -n "$LIDAR_FRAME" ]]; then
  LAUNCH_ARGS+=("lidar_frame:=${LIDAR_FRAME}")
fi
if [[ -n "$IMU_FRAME" ]]; then
  LAUNCH_ARGS+=("imu_frame:=${IMU_FRAME}")
fi

if command -v setsid >/dev/null 2>&1; then
  setsid ros2 launch lidarslam rko_lio_slam.launch.py \
    "${LAUNCH_ARGS[@]}" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
  LAUNCH_PGID="$LAUNCH_PID"
else
  ros2 launch lidarslam rko_lio_slam.launch.py \
    "${LAUNCH_ARGS[@]}" \
    >"${LAUNCH_LOG}" 2>&1 &
  LAUNCH_PID="$!"
fi

if [[ "$PRODUCT_SESSION_OUTPUT" != "concise" ]]; then
  echo "launch log: $LAUNCH_LOG"
fi

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

if ! wait_for_log_pattern "Offline output subscribers ready" "$STARTUP_TIMEOUT_SECS"; then
  echo "Timed out waiting for the offline output subscriber barrier. Recent launch log:" >&2
  tail -n 100 "$LAUNCH_LOG" >&2 || true
  exit 1
fi

if [[ "$WAIT_FOR_OFFLINE_COMPLETION" == "true" ]]; then
  if [[ "$PRODUCT_SESSION_OUTPUT" == "concise" ]]; then
    echo "Mapping is ready; processing the recorded sensor data ..."
  else
    echo "SLAM launch is up; output subscribers are connected before bag playback"
    echo "Waiting for offline bag playback to finish ..."
  fi
  if ! wait_for_offline_completion 900 15; then
    echo "Timed out waiting for offline completion or quiescent map outputs. Recent launch log:" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    exit 1
  fi
else
  if [[ "$PRODUCT_SESSION_OUTPUT" == "concise" ]]; then
    echo "Mapping is ready; waiting for the first map output ..."
  else
    echo "SLAM launch is up; output subscribers are connected before bag playback"
    echo "Waiting for the first saved Autoware map bundle ..."
  fi
  if ! wait_for_map_outputs "$SAVE_TIMEOUT_SECS"; then
    echo "Timed out waiting for the first saved map outputs under $OUTPUT_DIR" >&2
    tail -n 120 "$LAUNCH_LOG" >&2 || true
    exit 1
  fi
fi

wait_for_graph_drain "$GRAPH_DRAIN_SECS"
sleep 3
echo "Calling /map_save ..."
if ! call_map_save_with_retry; then
  if [[ -f "$OUTPUT_DIR/map_projector_info.yaml" && -f "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]]; then
    echo "Warning: /map_save call failed, but usable map outputs already exist. Proceeding with current bundle." >&2
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

echo "Map outputs saved under $OUTPUT_DIR"

if [[ -n "$CORRECTED_LOGGER_PID" ]]; then
  # Give graph_based_slam a moment to publish its final /modified_path.
  sleep 3
  kill -INT "$CORRECTED_LOGGER_PID" >/dev/null 2>&1 || true
  # SIGKILL fallback: older path_to_tum.py versions had a custom signal
  # handler that masked rclpy's default and wedged rcl_wait in C, so the
  # wait below would hang indefinitely. The current path_to_tum.py is
  # fixed (it lets rclpy install its own handler), but keep this guard in
  # case an old copy is on PATH or the script is reused elsewhere.
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    kill -0 "$CORRECTED_LOGGER_PID" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$CORRECTED_LOGGER_PID" 2>/dev/null; then
    echo "Warning: path_to_tum did not exit on SIGINT, sending SIGKILL." >&2
    kill -KILL "$CORRECTED_LOGGER_PID" >/dev/null 2>&1 || true
  fi
  wait "$CORRECTED_LOGGER_PID" 2>/dev/null || true
  CORRECTED_LOGGER_PID=""
  if [[ -f "$CORRECTED_TUM" ]]; then
    echo "Corrected trajectory written: $CORRECTED_TUM ($(wc -l < "$CORRECTED_TUM") poses)"
  else
    echo "Warning: $CORRECTED_TUM was not produced (no /modified_path messages?)." >&2
  fi
fi

if [[ "$USE_FINAL_OPTIMIZED_TRAJECTORY" == "true" ]]; then
  if ! stage_final_optimized_trajectory; then
    echo "Corrected trajectory capture was requested, so refusing to report a complete run." >&2
    exit 1
  fi
fi

if [[ -n "$RAW_LOGGER_PID" ]]; then
  kill -INT "$RAW_LOGGER_PID" >/dev/null 2>&1 || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    kill -0 "$RAW_LOGGER_PID" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$RAW_LOGGER_PID" 2>/dev/null; then
    echo "Warning: odom_to_tum did not exit on SIGINT, sending SIGKILL." >&2
    kill -KILL "$RAW_LOGGER_PID" >/dev/null 2>&1 || true
  fi
  wait "$RAW_LOGGER_PID" 2>/dev/null || true
  RAW_LOGGER_PID=""
  if [[ -f "$RAW_TUM" ]]; then
    echo "Raw trajectory written: $RAW_TUM ($(wc -l < "$RAW_TUM") poses)"
  else
    echo "Warning: $RAW_TUM was not produced (no $RAW_ODOMETRY_TOPIC messages?)." >&2
  fi
fi

if [[ "$USE_NATIVE_RAW_TRAJECTORY" == "true" ]]; then
  if ! stage_native_raw_trajectory; then
    echo "Raw trajectory capture was requested, so refusing to report a complete run." >&2
    exit 1
  fi
fi

if [[ "$GENERATE_LANELET2" == true ]]; then
  # A stale lanelet2_map.osm from an earlier run into the same output dir would
  # pair a mismatched lanelet2 with this run's pointcloud map, and the generator
  # writes its output before structural validation, so generate into a temp file
  # and only move it in place on success.
  rm -f "$LANELET2_OSM" "${LANELET2_OSM}.tmp"
  if [[ -f "$CORRECTED_TUM" ]]; then
    echo "Generating Lanelet2 map from corrected trajectory ..."
    if python3 "$SCRIPT_DIR/simple_lanelet2_generator.py" \
      --input "$CORRECTED_TUM" \
      --output "${LANELET2_OSM}.tmp" \
      --lane-width "$LANE_WIDTH" \
      --origin-lat "$ORIGIN_LAT" \
      --origin-lon "$ORIGIN_LON"; then
      mv "${LANELET2_OSM}.tmp" "$LANELET2_OSM"
      echo "Lanelet2 map written: $LANELET2_OSM"
    else
      rm -f "${LANELET2_OSM}.tmp"
      echo "Warning: Lanelet2 map generation failed; continuing without lanelet2_map.osm." >&2
    fi
  else
    echo "Warning: Lanelet2 generation skipped because $CORRECTED_TUM does not exist (--capture-corrected-path true is required)." >&2
  fi
fi

echo "Autoware map bundle under $OUTPUT_DIR:"
echo "  pointcloud_map/          $([[ -f "$OUTPUT_DIR/pointcloud_map/pointcloud_map_metadata.yaml" ]] && echo "OK" || echo "MISSING")"
echo "  map_projector_info.yaml  $([[ -f "$OUTPUT_DIR/map_projector_info.yaml" ]] && echo "OK" || echo "MISSING")"
if [[ "$GENERATE_LANELET2" == true ]]; then
  echo "  lanelet2_map.osm         $([[ -f "$LANELET2_OSM" ]] && echo "OK" || echo "MISSING (see warnings above)")"
else
  echo "  lanelet2_map.osm         skipped (--generate-lanelet2 false)"
fi

if [[ -n "$REFERENCE_TUM" && -f "$CORRECTED_TUM" ]]; then
  if [[ -f "$APE_FROM_TUM_SCRIPT" ]]; then
    echo "Computing APE vs $REFERENCE_TUM ..."
    if python3 "$APE_FROM_TUM_SCRIPT" \
        --ref "$REFERENCE_TUM" \
        --est "$CORRECTED_TUM" \
        --out "$CORRECTED_APE_REPORT"; then
      echo "APE report: $CORRECTED_APE_REPORT"
      grep -E '^(rmse|mean|median|max|pairs):' "$CORRECTED_APE_REPORT" || true
    else
      echo "Warning: ape_from_tum.py failed (insufficient pairs?)." >&2
    fi
  else
    echo "Warning: $APE_FROM_TUM_SCRIPT not found; skipping APE." >&2
  fi
fi

if [[ "$KEEP_LAUNCH" == "false" ]]; then
  if [[ -n "$LAUNCH_PGID" ]]; then
    kill -- "-${LAUNCH_PGID}" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  elif [[ -n "$LAUNCH_PID" ]]; then
    kill "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  LAUNCH_PID=""
  LAUNCH_PGID=""
else
  KEEP_RUNNING=1
fi

if [[ "$SKIP_VIEWER" == "true" ]]; then
  exit 0
fi

VIEWER_CMD=(
  bash "$SCRIPT_DIR/run_graph_slam_pointcloud_map_in_autoware.sh"
  "$OUTPUT_DIR"
  --autoware-core-dir "$AUTOWARE_CORE_DIR"
  --work-dir "$WORK_DIR"
)

if [[ -n "$VIEWER_RUN_DIR" ]]; then
  VIEWER_CMD+=(--run-dir "$VIEWER_RUN_DIR")
fi
if [[ "$VIEWER_REBUILD" == "true" ]]; then
  VIEWER_CMD+=(--rebuild)
fi
if [[ -n "$AUTO_EXIT_SECS" ]]; then
  VIEWER_CMD+=(--auto-exit-secs "$AUTO_EXIT_SECS")
fi

"${VIEWER_CMD[@]}"
