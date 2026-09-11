#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
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
  PACKAGE_SHARE="$(cd "${SCRIPT_DIR}/../.." && pwd)"
  INSTALL_PREFIX="$(cd "${PACKAGE_SHARE}/../.." && pwd)"
  WORK_ROOT="${PWD}"
  WORKSPACE_SETUP="${INSTALL_PREFIX}/setup.bash"
fi

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_open_data_gnss_smoke.sh --bag /path/to/rosbag2 [options]

Required:
  --bag PATH                  Main rosbag2 directory containing PointCloud2.

Options:
  --gnss-bag PATH             Optional rosbag2 directory that publishes /gnss/fix.
  --points-topic TOPIC        PointCloud2 topic in the main bag (auto-detects if omitted).
  --imu-topic TOPIC           Imu topic in the main bag (auto-detects if omitted).
  --gnss-topic TOPIC          NavSatFix topic (auto-detects if omitted).
  --param FILE                Base lidarslam parameter YAML (default: lidarslam/param/lidarslam.yaml).
  --save-dir DIR              Output directory (default: output/open_data_gnss_smoke_<timestamp>).
  --rate FLOAT                ros2 bag play rate (default: 1.0).
  --drain-sec SEC             Extra wait before /map_save (default: 15).
  --verify-map                Run verify_autoware_map.py after /map_save.

Notes:
  - The script creates a temporary parameter YAML with graph_based_slam use_gnss:=true.
  - If --gnss-bag is omitted, GNSS is expected to come from the main bag.
  - Sidecar GNSS bags are played without --clock to avoid conflicting /clock publishers.
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

require_rosbags() {
  python3 -c 'import rosbags' >/dev/null 2>&1 || die \
    "the pointcloud_gnss_smoke profile requires the Python package 'rosbags'; see https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/docs/distribution.md#profile-specific-extras"
}

timestamp() {
  date +%Y%m%d_%H%M%S
}

detect_topic_by_type() {
  local bag_path="$1"
  local msg_type="$2"
  python3 - "${bag_path}" "${msg_type}" <<'PY'
from pathlib import Path
import sys

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bag_path = Path(sys.argv[1])
msg_type = sys.argv[2]
best_topic = ''
best_count = -1

with AnyReader([bag_path], default_typestore=get_typestore(Stores.LATEST)) as reader:
    for connection in reader.connections:
        if connection.msgtype != msg_type:
            continue
        message_count = getattr(connection, 'msgcount', 0)
        if message_count > best_count:
            best_count = message_count
            best_topic = connection.topic

if best_topic:
    print(best_topic)
PY
}

detect_first_header_frame() {
  local bag_path="$1"
  local topic="$2"
  python3 - "${bag_path}" "${topic}" <<'PY'
from pathlib import Path
import sys

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bag_path = Path(sys.argv[1])
topic = sys.argv[2]
with AnyReader([bag_path], default_typestore=get_typestore(Stores.LATEST)) as reader:
    connections = [conn for conn in reader.connections if conn.topic == topic]
    if not connections:
        raise SystemExit(1)
    for conn, _, raw in reader.messages(connections=connections):
        msg = reader.deserialize(raw, conn.msgtype)
        header = getattr(msg, 'header', None)
        frame_id = getattr(header, 'frame_id', '')
        if frame_id:
            print(frame_id)
            break
PY
}

create_gnss_enabled_param() {
  local base_param="$1"
  local out_param="$2"
  cp "${base_param}" "${out_param}"
  python3 - "${out_param}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding='utf-8')
needle = '      use_gnss: false'
if needle in text:
    text = text.replace(needle, '      use_gnss: true', 1)
elif '      use_gnss: true' not in text:
    raise SystemExit('could not find graph_based_slam use_gnss parameter in base YAML')
path.write_text(text, encoding='utf-8')
PY
}

call_map_save_with_retry() {
  local log_file="$1"
  for _ in $(seq 1 5); do
    if timeout 20 ros2 service call /map_save std_srvs/srv/Empty "{}" >"${log_file}" 2>&1; then
      return 0
    fi
    sleep 2
  done
  return 1
}

BAG_PATH=""
GNSS_BAG=""
POINTS_TOPIC=""
IMU_TOPIC=""
GNSS_TOPIC=""
PARAM_FILE="${PACKAGE_SHARE}/param/lidarslam.yaml"
SAVE_DIR=""
RATE="1.0"
DRAIN_SEC="15"
VERIFY_MAP="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --bag)
      BAG_PATH="$(realpath "${2:-}")"; shift 2 ;;
    --gnss-bag)
      GNSS_BAG="$(realpath "${2:-}")"; shift 2 ;;
    --points-topic)
      POINTS_TOPIC="${2:-}"; shift 2 ;;
    --imu-topic)
      IMU_TOPIC="${2:-}"; shift 2 ;;
    --gnss-topic)
      GNSS_TOPIC="${2:-}"; shift 2 ;;
    --param)
      PARAM_FILE="$(realpath "${2:-}")"; shift 2 ;;
    --save-dir)
      SAVE_DIR="$(realpath "${2:-}")"; shift 2 ;;
    --rate)
      RATE="${2:-}"; shift 2 ;;
    --drain-sec)
      DRAIN_SEC="${2:-}"; shift 2 ;;
    --verify-map)
      VERIFY_MAP="true"; shift ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

[[ -n "${BAG_PATH}" ]] || { usage; die "--bag is required"; }
[[ -d "${BAG_PATH}" ]] || die "bag not found: ${BAG_PATH}"
[[ -f "${BAG_PATH}/metadata.yaml" ]] || die "metadata.yaml not found under ${BAG_PATH}"
[[ -f "${PARAM_FILE}" ]] || die "param file not found: ${PARAM_FILE}"
if [[ -n "${GNSS_BAG}" ]]; then
  [[ -d "${GNSS_BAG}" ]] || die "gnss bag not found: ${GNSS_BAG}"
  [[ -f "${GNSS_BAG}/metadata.yaml" ]] || die "metadata.yaml not found under ${GNSS_BAG}"
fi

if [[ -z "${SAVE_DIR}" ]]; then
  SAVE_DIR="${WORK_ROOT}/output/open_data_gnss_smoke_$(timestamp)"
fi
mkdir -p "${SAVE_DIR}"

set +u
if [[ -n "${WORKSPACE_SETUP}" && -f "${WORKSPACE_SETUP}" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_SETUP}"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
set -u

command -v ros2 >/dev/null 2>&1 || die "ros2 not found"
require_rosbags

if [[ -z "${POINTS_TOPIC}" ]]; then
  POINTS_TOPIC="$(detect_topic_by_type "${BAG_PATH}" "sensor_msgs/msg/PointCloud2")"
fi
[[ -n "${POINTS_TOPIC}" ]] || die "failed to detect PointCloud2 topic"

if [[ -z "${IMU_TOPIC}" ]]; then
  IMU_TOPIC="$(detect_topic_by_type "${BAG_PATH}" "sensor_msgs/msg/Imu")"
fi
if [[ -z "${IMU_TOPIC}" ]]; then
  IMU_TOPIC="/imu"
fi

if [[ -z "${GNSS_TOPIC}" ]]; then
  GNSS_TOPIC_SOURCE="${BAG_PATH}"
  if [[ -n "${GNSS_BAG}" ]]; then
    GNSS_TOPIC_SOURCE="${GNSS_BAG}"
  fi
  GNSS_TOPIC="$(detect_topic_by_type "${GNSS_TOPIC_SOURCE}" "sensor_msgs/msg/NavSatFix")"
fi
[[ -n "${GNSS_TOPIC}" ]] || die "failed to detect NavSatFix topic"

ROBOT_FRAME_ID="$(detect_first_header_frame "${BAG_PATH}" "${POINTS_TOPIC}")"
[[ -n "${ROBOT_FRAME_ID}" ]] || die "failed to detect frame_id for ${POINTS_TOPIC}"

TMP_PARAM="$(mktemp --suffix=.yaml)"
create_gnss_enabled_param "${PARAM_FILE}" "${TMP_PARAM}"

LAUNCH_LOG="${SAVE_DIR}/lidarslam.launch.log"
MAP_SAVE_LOG="${SAVE_DIR}/map_save.log"
MAIN_PLAY_LOG="${SAVE_DIR}/main_bag_play.log"
GNSS_PLAY_LOG="${SAVE_DIR}/gnss_bag_play.log"
VERIFY_LOG="${SAVE_DIR}/verify_autoware_map.log"

LAUNCH_PID=""
MAIN_PLAY_PID=""
GNSS_PLAY_PID=""
cleanup() {
  for pid in "${GNSS_PLAY_PID}" "${MAIN_PLAY_PID}" "${LAUNCH_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  rm -f "${TMP_PARAM}"
}
trap cleanup EXIT INT TERM

echo "Running open-data GNSS smoke:"
echo "  bag:         ${BAG_PATH}"
echo "  gnss_bag:    ${GNSS_BAG:-<main bag>}"
echo "  points:      ${POINTS_TOPIC}"
echo "  imu:         ${IMU_TOPIC}"
echo "  gnss_topic:  ${GNSS_TOPIC}"
echo "  robot_frame: ${ROBOT_FRAME_ID}"
echo "  save_dir:    ${SAVE_DIR}"

ros2 launch lidarslam lidarslam.launch.py \
  "main_param_dir:=${TMP_PARAM}" \
  "input_cloud:=${POINTS_TOPIC}" \
  "imu_topic:=${IMU_TOPIC}" \
  "gnss_topic:=${GNSS_TOPIC}" \
  "robot_frame_id:=${ROBOT_FRAME_ID}" \
  "global_frame_id:=map" \
  "use_graph_based_slam:=true" \
  "use_sim_time:=true" \
  "publish_static_tf:=false" \
  "save_dir:=${SAVE_DIR}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID="$!"

sleep 3

ros2 bag play "${BAG_PATH}" --clock --rate "${RATE}" >"${MAIN_PLAY_LOG}" 2>&1 &
MAIN_PLAY_PID="$!"

if [[ -n "${GNSS_BAG}" ]]; then
  ros2 bag play "${GNSS_BAG}" --rate "${RATE}" >"${GNSS_PLAY_LOG}" 2>&1 &
  GNSS_PLAY_PID="$!"
fi

wait "${MAIN_PLAY_PID}"
MAIN_PLAY_PID=""
if [[ -n "${GNSS_PLAY_PID}" ]]; then
  wait "${GNSS_PLAY_PID}"
  GNSS_PLAY_PID=""
fi

sleep "${DRAIN_SEC}"

if ! call_map_save_with_retry "${MAP_SAVE_LOG}"; then
  echo "map_save service call failed. Recent launch log:" >&2
  tail -n 80 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

if [[ "${VERIFY_MAP}" == "true" ]]; then
  python3 "${SCRIPT_DIR}/verify_autoware_map.py" "${SAVE_DIR}" >"${VERIFY_LOG}" 2>&1
fi

if [[ -f "${SAVE_DIR}/map_projector_info.yaml" ]]; then
  echo "map_projector_info.yaml:"
  cat "${SAVE_DIR}/map_projector_info.yaml"
fi

echo "done: ${SAVE_DIR}"
