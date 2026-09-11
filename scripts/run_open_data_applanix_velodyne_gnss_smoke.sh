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
  bash scripts/run_open_data_applanix_velodyne_gnss_smoke.sh --bag /path/to/rosbag2 [options]

Required:
  --bag PATH                  Main rosbag2 directory containing VelodyneScan and Applanix GSOF.

Options:
  --packet-topic TOPIC        VelodyneScan topic in the main bag (auto-detect if omitted).
  --gnss-bag PATH             Optional NavSatFix sidecar rosbag2. If omitted, one is generated from GSOF49/50.
  --gnss-topic TOPIC          NavSatFix topic (default: /gnss/fix).
  --use-imu BOOL              Enable IMU sidecar for deskew (default: false).
  --imu-bag PATH              Optional Imu sidecar rosbag2. If omitted and --use-imu=true, one is generated.
  --imu-topic TOPIC           Imu topic for scanmatcher/graph_based_slam (default: /imu).
  --imu-translation-deskew BOOL
                              Enable translational deskew from IMU acceleration (default: false).
  --imu-pose-prediction BOOL  Enable IMU-based pose prior in scanmatcher (default: false).
  --gsof49-topic TOPIC        Applanix GSOF49 topic.
  --gsof50-topic TOPIC        Applanix GSOF50 topic.
  --applanix-msg-dir PATH     Path to applanix_msgs/msg (default: /tmp/applanix/applanix_msgs/msg).
  --velodyne-overlay DIR      Overlay workspace with velodyne_pointcloud (default: /tmp/velodyne_ws).
  --velodyne-model MODEL      Velodyne model for packet conversion (default: VLP16).
  --velodyne-calibration FILE Explicit calibration YAML. If omitted, derived from the model.
  --param FILE                Base lidarslam parameter YAML.
  --save-dir DIR              Output directory (default: output/open_data_gnss_smoke_<timestamp>).
  --rate FLOAT                ros2 bag play rate (default: 5.0).
  --play-wall-sec SEC         Wall-clock playback time before map_save (default: 60).
  --drain-sec SEC             Extra wait before /map_save (default: 8).
  --verify-map                Run verify_autoware_map.py after /map_save.
  --ros-distro DISTRO         ROS 2 distro used for sourcing and overlay build (default: $ROS_DISTRO or jazzy).
  --skip-prepare-overlay      Do not auto-build the velodyne overlay when missing.

Notes:
  - This workflow is meant for real open-data bags that expose:
      * LiDAR as velodyne_msgs/msg/VelodyneScan
      * GNSS quality as Applanix GSOF49/50
      * INS orientation/rates as Applanix GSOF49
  - The script converts raw packets to PointCloud2 with velodyne_pointcloud.
  - Applanix acceleration appears gravity-compensated on Leo Drive, so
    translational deskew stays off by default and only rotational deskew is enabled.
  - The generated/selected GNSS sidecar is played without --clock to avoid conflicting clock publishers.
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

require_rosbags() {
  python3 -c 'import rosbags' >/dev/null 2>&1 || die \
    "the packet_applanix_smoke profile requires the Python package 'rosbags'; see https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/docs/distribution.md#profile-specific-extras"
}

timestamp() {
  date +%Y%m%d_%H%M%S
}

detect_topic_by_type() {
  local bag_path="$1"
  local msg_type="$2"
  local extra_msg_dir="${3:-}"
  local preferred_substring="${4:-}"
  local args=(
    --bag "${bag_path}"
    --msg-type "${msg_type}"
  )
  if [[ -n "${extra_msg_dir}" ]]; then
    args+=(--extra-msg-dir "${extra_msg_dir}")
  fi
  if [[ -n "${preferred_substring}" ]]; then
    args+=(--preferred-substring "${preferred_substring}")
  fi
  python3 "${SCRIPT_DIR}/select_rosbag_topic.py" "${args[@]}"
}

detect_first_header_frame() {
  local bag_path="$1"
  local topic="$2"
  local extra_msg_dir="${3:-}"
  python3 - "${bag_path}" "${topic}" "${extra_msg_dir}" <<'PY'
from pathlib import Path
import sys

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

bag_path = Path(sys.argv[1])
topic = sys.argv[2]
extra_msg_dir = Path(sys.argv[3]) if sys.argv[3] else None
typestore = get_typestore(Stores.LATEST)

if extra_msg_dir is not None:
    package_name = extra_msg_dir.parent.name
    for path in sorted(extra_msg_dir.glob('*.msg')):
        text = path.read_text(encoding='utf-8')
        typestore.register(get_types_from_msg(text, f'{package_name}/msg/{path.stem}'))

with AnyReader([bag_path], default_typestore=typestore) as reader:
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

create_main_param() {
  local base_param="$1"
  local out_param="$2"
  local use_imu="$3"
  local imu_translation_deskew="$4"
  local imu_pose_prediction_enable="$5"
  cp "${base_param}" "${out_param}"
  python3 - "${out_param}" "${use_imu}" "${imu_translation_deskew}" "${imu_pose_prediction_enable}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
use_imu = sys.argv[2].strip().lower() in {'1', 'true', 'yes', 'on'}
imu_translation_deskew = sys.argv[3].strip().lower() in {'1', 'true', 'yes', 'on'}
imu_pose_prediction_enable = sys.argv[4].strip().lower() in {'1', 'true', 'yes', 'on'}
text = path.read_text(encoding='utf-8')
if '      use_gnss: false' in text or '      use_gnss: true' in text:
    text = text.replace('      use_gnss: false', '      use_gnss: true', 1)
    text = text.replace('      use_gnss: true', '      use_gnss: true', 1)
else:
    raise SystemExit('failed to update base YAML parameters: use_gnss')
use_imu_line = f'    use_imu: {"true" if use_imu else "false"}'
if '    use_imu: true' in text or '    use_imu: false' in text:
    text = text.replace('    use_imu: true', use_imu_line, 1)
    text = text.replace('    use_imu: false', use_imu_line, 1)
else:
    raise SystemExit('failed to update base YAML parameters: use_imu')
imu_translation_line = (
    f'    imu_translation_deskew: {"true" if imu_translation_deskew else "false"}'
)
if '    imu_translation_deskew: true' in text or '    imu_translation_deskew: false' in text:
    text = text.replace('    imu_translation_deskew: true', imu_translation_line, 1)
    text = text.replace('    imu_translation_deskew: false', imu_translation_line, 1)
else:
    text = text.replace(use_imu_line, use_imu_line + '\n' + imu_translation_line, 1)
imu_pose_prediction_line = (
    '    imu_pose_prediction_enable: '
    f'{"true" if imu_pose_prediction_enable else "false"}'
)
if (
    '    imu_pose_prediction_enable: true' in text or
    '    imu_pose_prediction_enable: false' in text
):
    text = text.replace(
        '    imu_pose_prediction_enable: true',
        imu_pose_prediction_line,
        1,
    )
    text = text.replace(
        '    imu_pose_prediction_enable: false',
        imu_pose_prediction_line,
        1,
    )
else:
    text = text.replace(
        imu_translation_line,
        imu_translation_line + '\n' + imu_pose_prediction_line,
        1,
    )
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

stage_playback_bag() {
  local bag_path="$1"
  local staging_root="$2"
  local output_variable="$3"
  local playback_path=""
  playback_path="$(python3 "${SCRIPT_DIR}/prepare_rosbag2_playback.py" \
    stage \
    --bag "${bag_path}" \
    --staging-root "${staging_root}")"
  printf -v "${output_variable}" '%s' "${playback_path}"
  if [[ "${playback_path}" != "${bag_path}" ]]; then
    STAGED_PLAYBACK_DIRS+=("${playback_path}")
  fi
}

cleanup_playback_staging() {
  local staging_path=""
  for staging_path in "${STAGED_PLAYBACK_DIRS[@]}"; do
    if [[ -d "${staging_path}" ]]; then
      if ! python3 "${SCRIPT_DIR}/prepare_rosbag2_playback.py" \
        cleanup \
        --path "${staging_path}" \
        --staging-root "${SAVE_DIR}"
      then
        echo "warning: failed to remove playback staging: ${staging_path}" >&2
      fi
    fi
  done
}

ensure_velodyne_overlay() {
  local overlay_dir="$1"
  local ros_distro_name="$2"

  if bash "${SCRIPT_DIR}/prepare_velodyne_pointcloud_overlay.sh" \
    --overlay-dir "${overlay_dir}" \
    --check >/dev/null 2>&1
  then
    return 0
  fi
  bash "${SCRIPT_DIR}/prepare_velodyne_pointcloud_overlay.sh" \
    --overlay-dir "${overlay_dir}" \
    --ros-distro "${ros_distro_name}"
}

resolve_velodyne_msg_dir() {
  local overlay_dir="$1"
  local candidate=""
  for candidate in \
    "${overlay_dir}/src/velodyne/velodyne_msgs/msg" \
    "${overlay_dir}/install/velodyne_msgs/share/velodyne_msgs/msg"
  do
    if [[ -d "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

default_calibration_for_model() {
  local overlay_dir="$1"
  local model="$2"
  case "${model}" in
    VLP16)
      printf '%s\n' "${overlay_dir}/install/velodyne_pointcloud/share/velodyne_pointcloud/params/VLP16db.yaml"
      ;;
    32C|VLP32C)
      printf '%s\n' "${overlay_dir}/install/velodyne_pointcloud/share/velodyne_pointcloud/params/VeloView-VLP-32C.yaml"
      ;;
    VLS128)
      printf '%s\n' "${overlay_dir}/install/velodyne_pointcloud/share/velodyne_pointcloud/params/VLS128.yaml"
      ;;
    *)
      die "unsupported velodyne model: ${model}"
      ;;
  esac
}

BAG_PATH=""
PACKET_TOPIC=""
GNSS_BAG=""
GNSS_TOPIC="/gnss/fix"
USE_IMU="false"
IMU_BAG=""
IMU_TOPIC="/imu"
IMU_TRANSLATION_DESKEW="false"
IMU_POSE_PREDICTION="false"
GSOF49_TOPIC="/lvx_client/gsof/ins_solution_49"
GSOF50_TOPIC="/lvx_client/gsof/ins_solution_rms_50"
APPLANIX_MSG_DIR="/tmp/applanix/applanix_msgs/msg"
VELODYNE_OVERLAY="/tmp/velodyne_ws"
VELODYNE_MODEL="VLP16"
VELODYNE_CALIBRATION=""
PARAM_FILE="${PACKAGE_SHARE}/param/lidarslam.yaml"
SAVE_DIR=""
RATE="5.0"
PLAY_WALL_SEC="60"
DRAIN_SEC="8"
VERIFY_MAP="false"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
PREPARE_OVERLAY="true"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --bag)
      BAG_PATH="$(realpath "${2:-}")"; shift 2 ;;
    --packet-topic)
      PACKET_TOPIC="${2:-}"; shift 2 ;;
    --gnss-bag)
      GNSS_BAG="$(realpath "${2:-}")"; shift 2 ;;
    --gnss-topic)
      GNSS_TOPIC="${2:-}"; shift 2 ;;
    --use-imu)
      USE_IMU="${2:-}"; shift 2 ;;
    --imu-bag)
      IMU_BAG="$(realpath "${2:-}")"; shift 2 ;;
    --imu-topic)
      IMU_TOPIC="${2:-}"; shift 2 ;;
    --imu-translation-deskew)
      IMU_TRANSLATION_DESKEW="${2:-}"; shift 2 ;;
    --imu-pose-prediction)
      IMU_POSE_PREDICTION="${2:-}"; shift 2 ;;
    --gsof49-topic)
      GSOF49_TOPIC="${2:-}"; shift 2 ;;
    --gsof50-topic)
      GSOF50_TOPIC="${2:-}"; shift 2 ;;
    --applanix-msg-dir)
      APPLANIX_MSG_DIR="$(realpath "${2:-}")"; shift 2 ;;
    --velodyne-overlay)
      VELODYNE_OVERLAY="$(realpath -m "${2:-}")"; shift 2 ;;
    --velodyne-model)
      VELODYNE_MODEL="${2:-}"; shift 2 ;;
    --velodyne-calibration)
      VELODYNE_CALIBRATION="$(realpath "${2:-}")"; shift 2 ;;
    --param)
      PARAM_FILE="$(realpath "${2:-}")"; shift 2 ;;
    --save-dir)
      SAVE_DIR="$(realpath -m "${2:-}")"; shift 2 ;;
    --rate)
      RATE="${2:-}"; shift 2 ;;
    --play-wall-sec)
      PLAY_WALL_SEC="${2:-}"; shift 2 ;;
    --drain-sec)
      DRAIN_SEC="${2:-}"; shift 2 ;;
    --verify-map)
      VERIFY_MAP="true"; shift ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:-}"; shift 2 ;;
    --skip-prepare-overlay)
      PREPARE_OVERLAY="false"; shift ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

[[ -n "${BAG_PATH}" ]] || { usage; die "--bag is required"; }
[[ -d "${BAG_PATH}" ]] || die "bag not found: ${BAG_PATH}"
[[ -f "${BAG_PATH}/metadata.yaml" ]] || die "metadata.yaml not found under ${BAG_PATH}"
[[ -f "${PARAM_FILE}" ]] || die "param file not found: ${PARAM_FILE}"

if [[ -z "${SAVE_DIR}" ]]; then
  SAVE_DIR="${WORK_ROOT}/output/open_data_gnss_smoke_$(timestamp)"
fi
mkdir -p "${SAVE_DIR}"

[[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]] || {
  die "ROS setup not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
}

if [[ "${PREPARE_OVERLAY}" == "true" ]]; then
  ensure_velodyne_overlay "${VELODYNE_OVERLAY}" "${ROS_DISTRO_NAME}"
fi

[[ -f "${VELODYNE_OVERLAY}/install/setup.bash" ]] || {
  die "velodyne overlay not found: ${VELODYNE_OVERLAY}/install/setup.bash"
}
VELODYNE_MSG_DIR="$(resolve_velodyne_msg_dir "${VELODYNE_OVERLAY}")" || {
  die "velodyne_msgs definitions not found under ${VELODYNE_OVERLAY}"
}

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
if [[ -n "${WORKSPACE_SETUP}" && -f "${WORKSPACE_SETUP}" ]]; then
  # shellcheck source=/dev/null
  source "${WORKSPACE_SETUP}"
fi
# shellcheck source=/dev/null
source "${VELODYNE_OVERLAY}/install/setup.bash"
set -u

command -v ros2 >/dev/null 2>&1 || die "ros2 not found"
require_rosbags
ros2 pkg executables velodyne_pointcloud | grep -q 'velodyne_transform_node' || {
  die "velodyne_transform_node not available after sourcing ${VELODYNE_OVERLAY}"
}

if [[ -z "${PACKET_TOPIC}" ]]; then
  PACKET_TOPIC="$(detect_topic_by_type \
    "${BAG_PATH}" \
    "velodyne_msgs/msg/VelodyneScan" \
    "${VELODYNE_MSG_DIR}" \
    "/front/")"
fi
[[ -n "${PACKET_TOPIC}" ]] || die "failed to detect VelodyneScan topic"

ROBOT_FRAME_ID="$(detect_first_header_frame \
  "${BAG_PATH}" \
  "${PACKET_TOPIC}" \
  "${VELODYNE_MSG_DIR}")"
[[ -n "${ROBOT_FRAME_ID}" ]] || die "failed to detect frame_id for ${PACKET_TOPIC}"

if [[ -z "${VELODYNE_CALIBRATION}" ]]; then
  VELODYNE_CALIBRATION="$(default_calibration_for_model "${VELODYNE_OVERLAY}" "${VELODYNE_MODEL}")"
fi
[[ -f "${VELODYNE_CALIBRATION}" ]] || die "velodyne calibration not found: ${VELODYNE_CALIBRATION}"

CONVERT_LOG="${SAVE_DIR}/convert_applanix.log"
IMU_CONVERT_LOG="${SAVE_DIR}/convert_applanix_imu.log"
if [[ -z "${GNSS_BAG}" ]]; then
  [[ -d "${APPLANIX_MSG_DIR}" ]] || {
    die "applanix_msgs dir not found: ${APPLANIX_MSG_DIR}"
  }
  GNSS_BAG="${SAVE_DIR}/applanix_navsatfix_sidecar"
  python3 "${SCRIPT_DIR}/convert_applanix_gsof_to_navsatfix_bag.py" \
    --input "${BAG_PATH}" \
    --output "${GNSS_BAG}" \
    --gsof49-topic "${GSOF49_TOPIC}" \
    --gsof50-topic "${GSOF50_TOPIC}" \
    --output-topic "${GNSS_TOPIC}" \
    --applanix-msg-dir "${APPLANIX_MSG_DIR}" \
    --force \
    >"${CONVERT_LOG}" 2>&1
fi
[[ -d "${GNSS_BAG}" ]] || die "gnss bag not found: ${GNSS_BAG}"
[[ -f "${GNSS_BAG}/metadata.yaml" ]] || die "metadata.yaml not found under ${GNSS_BAG}"

if [[ "${USE_IMU,,}" == "true" ]]; then
  if [[ -z "${IMU_BAG}" ]]; then
    [[ -d "${APPLANIX_MSG_DIR}" ]] || {
      die "applanix_msgs dir not found: ${APPLANIX_MSG_DIR}"
    }
    IMU_BAG="${SAVE_DIR}/applanix_imu_sidecar"
    python3 "${SCRIPT_DIR}/convert_applanix_gsof_to_imu_bag.py" \
      --input "${BAG_PATH}" \
      --output "${IMU_BAG}" \
      --gsof49-topic "${GSOF49_TOPIC}" \
      --gsof50-topic "${GSOF50_TOPIC}" \
      --output-topic "${IMU_TOPIC}" \
      --frame-id "${ROBOT_FRAME_ID}" \
      --applanix-msg-dir "${APPLANIX_MSG_DIR}" \
      --force \
      >"${IMU_CONVERT_LOG}" 2>&1
  fi
  [[ -d "${IMU_BAG}" ]] || die "imu bag not found: ${IMU_BAG}"
  [[ -f "${IMU_BAG}/metadata.yaml" ]] || die "metadata.yaml not found under ${IMU_BAG}"
fi

TMP_PARAM="$(mktemp --suffix=.yaml)"
VELODYNE_PARAM="$(mktemp --suffix=.yaml)"
QOS_FILE="$(mktemp --suffix=.yaml)"
create_main_param \
  "${PARAM_FILE}" \
  "${TMP_PARAM}" \
  "${USE_IMU}" \
  "${IMU_TRANSLATION_DESKEW}" \
  "${IMU_POSE_PREDICTION}"

cat >"${VELODYNE_PARAM}" <<EOF
velodyne_transform_node:
  ros__parameters:
    calibration: ${VELODYNE_CALIBRATION}
    model: ${VELODYNE_MODEL}
    min_range: 0.9
    max_range: 200.0
    view_direction: 0.0
    fixed_frame: ""
    target_frame: ""
    organize_cloud: false
EOF

cat >"${QOS_FILE}" <<EOF
${PACKET_TOPIC}:
  reliability: reliable
  durability: volatile
  history: keep_last
  depth: 10
EOF

LAUNCH_LOG="${SAVE_DIR}/lidarslam.launch.log"
MAP_SAVE_LOG="${SAVE_DIR}/map_save.log"
MAIN_PLAY_LOG="${SAVE_DIR}/main_bag_play.log"
GNSS_PLAY_LOG="${SAVE_DIR}/gnss_bag_play.log"
IMU_PLAY_LOG="${SAVE_DIR}/imu_bag_play.log"
VELODYNE_LOG="${SAVE_DIR}/velodyne_transform.log"
VERIFY_LOG="${SAVE_DIR}/verify_autoware_map.log"
POINTS_TOPIC="/open_data/velodyne_points"

LAUNCH_PID=""
MAIN_PLAY_PID=""
GNSS_PLAY_PID=""
IMU_PLAY_PID=""
VELODYNE_PID=""
STAGED_PLAYBACK_DIRS=()
MAIN_PLAY_BAG=""
GNSS_PLAY_BAG=""
IMU_PLAY_BAG=""
cleanup() {
  for pid in "${IMU_PLAY_PID}" "${GNSS_PLAY_PID}" "${MAIN_PLAY_PID}" "${VELODYNE_PID}" "${LAUNCH_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  rm -f "${TMP_PARAM}" "${VELODYNE_PARAM}" "${QOS_FILE}"
  cleanup_playback_staging
}
trap cleanup EXIT INT TERM

stage_playback_bag "${BAG_PATH}" "${SAVE_DIR}" MAIN_PLAY_BAG
stage_playback_bag "${GNSS_BAG}" "${SAVE_DIR}" GNSS_PLAY_BAG
if [[ "${USE_IMU,,}" == "true" ]]; then
  stage_playback_bag "${IMU_BAG}" "${SAVE_DIR}" IMU_PLAY_BAG
fi

echo "Running Applanix + Velodyne GNSS smoke:"
echo "  bag:                 ${BAG_PATH}"
echo "  packet_topic:        ${PACKET_TOPIC}"
echo "  gnss_bag:            ${GNSS_BAG}"
echo "  gnss_topic:          ${GNSS_TOPIC}"
echo "  use_imu:             ${USE_IMU}"
if [[ "${USE_IMU,,}" == "true" ]]; then
  echo "  imu_bag:             ${IMU_BAG}"
  echo "  imu_topic:           ${IMU_TOPIC}"
  echo "  imu_translation_deskew:${IMU_TRANSLATION_DESKEW}"
  echo "  imu_pose_prediction: ${IMU_POSE_PREDICTION}"
fi
echo "  velodyne_model:      ${VELODYNE_MODEL}"
echo "  velodyne_calibration:${VELODYNE_CALIBRATION}"
echo "  robot_frame:         ${ROBOT_FRAME_ID}"
if [[ "${MAIN_PLAY_BAG}" != "${BAG_PATH}" ]]; then
  echo "  playback_staging:    isolated FILE-compressed bag"
fi
echo "  save_dir:            ${SAVE_DIR}"

ros2 run velodyne_pointcloud velodyne_transform_node \
  --ros-args \
  --params-file "${VELODYNE_PARAM}" \
  -r "velodyne_packets:=${PACKET_TOPIC}" \
  -r "velodyne_points:=${POINTS_TOPIC}" \
  >"${VELODYNE_LOG}" 2>&1 &
VELODYNE_PID="$!"

ros2 launch lidarslam lidarslam.launch.py \
  "main_param_dir:=${TMP_PARAM}" \
  "input_cloud:=${POINTS_TOPIC}" \
  "imu_topic:=${IMU_TOPIC}" \
  "gnss_topic:=${GNSS_TOPIC}" \
  "robot_frame_id:=${ROBOT_FRAME_ID}" \
  "base_frame:=${ROBOT_FRAME_ID}" \
  "lidar_frame:=${ROBOT_FRAME_ID}" \
  "global_frame_id:=map" \
  "use_graph_based_slam:=true" \
  "use_sim_time:=true" \
  "publish_static_tf:=false" \
  "save_dir:=${SAVE_DIR}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID="$!"

sleep 5

timeout "${PLAY_WALL_SEC}" ros2 bag play "${MAIN_PLAY_BAG}" \
  --clock \
  --rate "${RATE}" \
  --topics "${PACKET_TOPIC}" \
  --qos-profile-overrides-path "${QOS_FILE}" \
  >"${MAIN_PLAY_LOG}" 2>&1 &
MAIN_PLAY_PID="$!"

timeout "${PLAY_WALL_SEC}" ros2 bag play "${GNSS_PLAY_BAG}" \
  --rate "${RATE}" \
  >"${GNSS_PLAY_LOG}" 2>&1 &
GNSS_PLAY_PID="$!"

if [[ "${USE_IMU,,}" == "true" ]]; then
  timeout "${PLAY_WALL_SEC}" ros2 bag play "${IMU_PLAY_BAG}" \
    --rate "${RATE}" \
    >"${IMU_PLAY_LOG}" 2>&1 &
  IMU_PLAY_PID="$!"
fi

wait "${MAIN_PLAY_PID}" || true
MAIN_PLAY_PID=""
wait "${GNSS_PLAY_PID}" || true
GNSS_PLAY_PID=""
if [[ -n "${IMU_PLAY_PID}" ]]; then
  wait "${IMU_PLAY_PID}" || true
  IMU_PLAY_PID=""
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
