#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="${REPO_ROOT}"
if [[ ! -f "${WS_ROOT}/install/setup.bash" && -f "${REPO_ROOT}/../install/setup.bash" ]]; then
  WS_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
fi

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh --bag /path/to/rosbag2 [options]

Required:
  --bag PATH                  Main rosbag2 directory containing VelodyneScan and Applanix GSOF.

Options:
  --packet-topic TOPIC        VelodyneScan topic in the main bag (auto-detect if omitted).
  --reference-tum FILE        Optional reference TUM trajectory. If omitted, one is extracted from GSOF49.
  --robot-frame-id FRAME      Robot/base frame for scanmatcher (default: auto; packet frame unless IMU+tf-bag is used).
  --gnss-bag PATH             Optional NavSatFix rosbag2. If omitted and --use-gnss=true, the wrapper first
                              prefers a native NavSatFix topic in the main bag, then generates a sidecar if needed.
  --gnss-topic TOPIC          NavSatFix topic (default: /gnss/fix).
  --use-odom-prior BOOL       Enable scanmatcher odom prior from a /tf sidecar (default: false).
  --odom-bag PATH             Optional /tf rosbag2 used when --use-odom-prior=true.
                              If omitted, a sidecar is generated from GSOF49.
  --odom-topic TOPIC          TF topic for odom prior playback (default: /tf).
  --odom-frame-id FRAME       Odom frame for scanmatcher (default: odom).
  --odom-prior-planar BOOL    Generate planar odom prior (zero z/roll/pitch) (default: false).
  --odom-prior-velocity-planar BOOL
                              Generate odom prior by integrating GSOF49 planar velocity
                              instead of absolute LLA poses (default: false).
  --odom-prior-translation-only BOOL
                              Use translation-only odom prior inside scanmatcher (default: false).
  --odom-prior-suspect-recovery-only BOOL
                              Apply odom prior only while scanmatcher is in Suspect/Recovery
                              (default: false).
  --odom-prior-weight FLOAT   Blend weight applied to the odom delta inside scanmatcher
                              (default: 1.0).
  --use-imu BOOL              Enable IMU input for deskew (default: false).
  --imu-bag PATH              Optional Imu rosbag2. If omitted and --use-imu=true, the wrapper first
                              prefers a native Imu topic in the main bag, then generates a sidecar if needed.
  --imu-topic TOPIC           Imu topic for scanmatcher/graph_based_slam (default: /imu).
  --imu-frame-id FRAME        frame_id written into the generated IMU sidecar (default: base_link).
  --imu-translation-deskew BOOL
                              Enable translational deskew from IMU acceleration (default: false).
  --imu-rotation-use-orientation BOOL
                              Use absolute IMU orientation for rotation deskew. If false, use
                              gyro-integrated rotation only (default: true).
  --imu-pose-prediction BOOL  Enable IMU-based pose prior in scanmatcher (default: false).
  --cloud-queue-depth N       scanmatcher input cloud queue depth (default: keep base YAML).
  --debug-cloud-dump-max-frames N
                              Dump scanmatcher clouds for the first N frames (default: 0).
  --debug-cloud-dump-dir DIR  Output directory for scanmatcher debug clouds
                              (default: OUTPUT_DIR/scanmatcher_debug_clouds when enabled).
  --tf-bag PATH               Optional rosbag2 with /tf_static used to extract robot->lidar static TF.
  --tf-static-topic TOPIC     Static TF topic inside --tf-bag (default: /tf_static).
  --gsof49-topic TOPIC        Applanix GSOF49 topic.
  --gsof50-topic TOPIC        Applanix GSOF50 topic.
  --applanix-msg-dir PATH     Path to applanix_msgs/msg (default: /tmp/applanix/applanix_msgs/msg).
  --velodyne-overlay DIR      Overlay workspace with velodyne_pointcloud (default: /tmp/velodyne_ws).
  --velodyne-model MODEL      Velodyne model for packet conversion (default: VLP16).
  --velodyne-calibration FILE Explicit calibration YAML. If omitted, derived from the model.
  --param FILE                Base lidarslam parameter YAML.
  --output-dir DIR            Output directory (default: output/open_data_applanix_velodyne_gnss_benchmark_<timestamp>).
  --rate FLOAT                ros2 bag play rate (default: 1.0).
  --play-wall-sec SEC         Playback timeout. If omitted, derived from bag duration and rate.
  --drain-sec SEC             Extra wait before /map_save (default: 8).
  --use-gnss BOOL             Enable backend GNSS constraints (default: true).
  --verify-map                Run verify_autoware_map.py after /map_save.
  --ros-domain-id ID          Export ROS_DOMAIN_ID for this benchmark run.
  --ros-distro DISTRO         ROS 2 distro used for sourcing and overlay build (default: $ROS_DISTRO or jazzy).
  --skip-prepare-overlay      Do not auto-build the velodyne overlay when missing.

This wrapper runs:
  VelodyneScan + Applanix GSOF49/50 -> PointCloud2 + NavSatFix + Imu -> lidarslam.launch.py
  -> raw/corrected TUM -> aligned metrics.json

Notes:
  - all-sensors Leo Drive bags can stay entirely on real native topics:
      * packet topic from the main bag
      * native sensor_msgs/msg/Imu from the main bag
      * native sensor_msgs/msg/NavSatFix from the main bag
      * GSOF49 extracted from the same bag as cross-validation reference
  - driving bags without native Imu/NavSatFix still fall back to Applanix sidecar generation.
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
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

topic_exists_by_name_and_type() {
  local bag_path="$1"
  local topic="$2"
  local msg_type="$3"
  local extra_msg_dir="${4:-}"
  python3 - "${bag_path}" "${topic}" "${msg_type}" "${extra_msg_dir}" <<'PY'
from pathlib import Path
import sys

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

bag_path = Path(sys.argv[1])
topic = sys.argv[2]
msg_type = sys.argv[3]
extra_msg_dir = Path(sys.argv[4]) if sys.argv[4] else None
typestore = get_typestore(Stores.LATEST)

if extra_msg_dir is not None:
    package_name = extra_msg_dir.parent.name
    for path in sorted(extra_msg_dir.glob('*.msg')):
        text = path.read_text(encoding='utf-8')
        typestore.register(get_types_from_msg(text, f'{package_name}/msg/{path.stem}'))

with AnyReader([bag_path], default_typestore=typestore) as reader:
    for connection in reader.connections:
        if connection.topic == topic and connection.msgtype == msg_type:
            raise SystemExit(0)
raise SystemExit(1)
PY
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

bag_duration_seconds() {
  local bag_path="$1"
  python3 - "${bag_path}" <<'PY'
from pathlib import Path
import sys

metadata = Path(sys.argv[1]) / 'metadata.yaml'
if not metadata.is_file():
    raise SystemExit(1)

lines = metadata.read_text(encoding='utf-8', errors='replace').splitlines()
in_duration = False
for line in lines:
    stripped = line.strip()
    if stripped.startswith('duration:'):
        in_duration = True
        continue
    if in_duration and stripped.startswith('nanoseconds:'):
        nanoseconds = int(stripped.split(':', 1)[1].strip())
        print(nanoseconds / 1e9)
        raise SystemExit(0)
    if in_duration and stripped and not line.startswith(' '):
        break
raise SystemExit(1)
PY
}

compute_play_wall_sec() {
  local bag_path="$1"
  local rate="$2"
  python3 - "${bag_path}" "${rate}" <<'PY'
from pathlib import Path
import math
import sys

metadata = Path(sys.argv[1]) / 'metadata.yaml'
rate = float(sys.argv[2])
if rate <= 0.0:
    raise SystemExit('rate must be > 0')
lines = metadata.read_text(encoding='utf-8', errors='replace').splitlines()
in_duration = False
duration_ns = None
for line in lines:
    stripped = line.strip()
    if stripped.startswith('duration:'):
        in_duration = True
        continue
    if in_duration and stripped.startswith('nanoseconds:'):
        duration_ns = int(stripped.split(':', 1)[1].strip())
        break
if duration_ns is None:
    raise SystemExit('failed to parse bag duration')
duration_sec = duration_ns / 1e9
print(int(math.ceil(duration_sec / rate + 60.0)))
PY
}

create_main_param() {
  local base_param="$1"
  local out_param="$2"
  local use_gnss="$3"
  local use_imu="$4"
  local use_odom="$5"
  local imu_translation_deskew="$6"
  local imu_rotation_use_orientation="$7"
  local imu_pose_prediction_enable="$8"
  local cloud_queue_depth="${9}"
  local debug_cloud_dump_dir="${10}"
  local debug_cloud_dump_max_frames="${11}"
  local odom_prior_planar="${12}"
  local odom_prior_translation_only="${13}"
  local odom_prior_weight="${14}"
  local odom_prior_suspect_recovery_only="${15}"
  cp "${base_param}" "${out_param}"
  python3 - "${out_param}" "${use_gnss}" "${use_imu}" "${use_odom}" "${imu_translation_deskew}" "${imu_rotation_use_orientation}" "${imu_pose_prediction_enable}" "${cloud_queue_depth}" "${debug_cloud_dump_dir}" "${debug_cloud_dump_max_frames}" "${odom_prior_planar}" "${odom_prior_translation_only}" "${odom_prior_weight}" "${odom_prior_suspect_recovery_only}" <<'PY'
from pathlib import Path
import json
import sys

path = Path(sys.argv[1])
use_gnss = sys.argv[2].strip().lower() in {'1', 'true', 'yes', 'on'}
use_imu = sys.argv[3].strip().lower() in {'1', 'true', 'yes', 'on'}
use_odom = sys.argv[4].strip().lower() in {'1', 'true', 'yes', 'on'}
imu_translation_deskew = sys.argv[5].strip().lower() in {'1', 'true', 'yes', 'on'}
imu_rotation_use_orientation = sys.argv[6].strip().lower() in {'1', 'true', 'yes', 'on'}
imu_pose_prediction_enable = sys.argv[7].strip().lower() in {'1', 'true', 'yes', 'on'}
cloud_queue_depth = sys.argv[8].strip()
debug_cloud_dump_dir = sys.argv[9]
debug_cloud_dump_max_frames = max(0, int(sys.argv[10]))
odom_prior_planar = sys.argv[11].strip().lower() in {'1', 'true', 'yes', 'on'}
odom_prior_translation_only = sys.argv[12].strip().lower() in {'1', 'true', 'yes', 'on'}
odom_prior_weight = float(sys.argv[13])
odom_prior_suspect_recovery_only = sys.argv[14].strip().lower() in {'1', 'true', 'yes', 'on'}
text = path.read_text(encoding='utf-8')
if '      use_gnss: true' in text or '      use_gnss: false' in text:
    text = text.replace(
        '      use_gnss: true',
        f'      use_gnss: {"true" if use_gnss else "false"}',
        1,
    )
    text = text.replace(
        '      use_gnss: false',
        f'      use_gnss: {"true" if use_gnss else "false"}',
        1,
    )
else:
    raise SystemExit('could not find graph_based_slam use_gnss parameter in base YAML')
if '    use_imu: true' in text or '    use_imu: false' in text:
    use_imu_line = f'    use_imu: {"true" if use_imu else "false"}'
    text = text.replace(
        '    use_imu: true',
        use_imu_line,
        1,
    )
    text = text.replace(
        '    use_imu: false',
        use_imu_line,
        1,
    )
else:
    raise SystemExit('could not find scan_matcher use_imu parameter in base YAML')
if '    use_odom: true' in text or '    use_odom: false' in text:
    use_odom_line = f'    use_odom: {"true" if use_odom else "false"}'
    text = text.replace(
        '    use_odom: true',
        use_odom_line,
        1,
    )
    text = text.replace(
        '    use_odom: false',
        use_odom_line,
        1,
    )
else:
    raise SystemExit('could not find scan_matcher use_odom parameter in base YAML')
odom_prior_planar_line = f'    odom_prior_planar: {"true" if odom_prior_planar else "false"}'
odom_prior_translation_only_line = (
    '    odom_prior_translation_only: '
    f'{"true" if odom_prior_translation_only else "false"}'
)
odom_prior_weight_line = f'    odom_prior_weight: {odom_prior_weight:.6f}'
odom_prior_suspect_recovery_only_line = (
    '    odom_prior_suspect_recovery_only: '
    f'{"true" if odom_prior_suspect_recovery_only else "false"}'
)
for key, replacement in (
    ('    odom_prior_planar:', odom_prior_planar_line),
    ('    odom_prior_translation_only:', odom_prior_translation_only_line),
    ('    odom_prior_weight:', odom_prior_weight_line),
    ('    odom_prior_suspect_recovery_only:', odom_prior_suspect_recovery_only_line),
):
    lines = text.splitlines()
    replaced = False
    for index, line in enumerate(lines):
        if line.startswith(key):
            lines[index] = replacement
            replaced = True
            break
    if not replaced:
        for index, line in enumerate(lines):
            if line == use_odom_line:
                lines[index:index + 1] = [
                    line,
                    odom_prior_planar_line,
                    odom_prior_translation_only_line,
                    odom_prior_weight_line,
                    odom_prior_suspect_recovery_only_line,
                ]
                replaced = True
                break
    if not replaced:
        raise SystemExit(f'could not find scan_matcher insertion point for {key}')
    text = '\n'.join(lines) + '\n'
imu_translation_line = (
    f'    imu_translation_deskew: {"true" if imu_translation_deskew else "false"}'
)
if (
    '    imu_translation_deskew: true' in text or
    '    imu_translation_deskew: false' in text
):
    text = text.replace(
        '    imu_translation_deskew: true',
        imu_translation_line,
        1,
    )
    text = text.replace(
        '    imu_translation_deskew: false',
        imu_translation_line,
        1,
    )
else:
    text = text.replace(use_odom_line, use_odom_line + '\n' + imu_translation_line, 1)
imu_rotation_line = (
    '    imu_rotation_deskew_use_orientation: '
    f'{"true" if imu_rotation_use_orientation else "false"}'
)
if (
    '    imu_rotation_deskew_use_orientation: true' in text or
    '    imu_rotation_deskew_use_orientation: false' in text
):
    text = text.replace(
        '    imu_rotation_deskew_use_orientation: true',
        imu_rotation_line,
        1,
    )
    text = text.replace(
        '    imu_rotation_deskew_use_orientation: false',
        imu_rotation_line,
        1,
    )
else:
    text = text.replace(
        imu_translation_line,
        imu_translation_line + '\n' + imu_rotation_line,
        1,
    )
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
        imu_rotation_line,
        imu_rotation_line + '\n' + imu_pose_prediction_line,
        1,
    )
debug_dir_line = f'    debug_cloud_dump_dir: {json.dumps(debug_cloud_dump_dir)}'
debug_max_frames_line = f'    debug_cloud_dump_max_frames: {debug_cloud_dump_max_frames}'
lines = [
    line for line in text.splitlines()
    if 'debug_cloud_dump_dir:' not in line and 'debug_cloud_dump_max_frames:' not in line
]
inserted = False
for index, line in enumerate(lines):
    if line in ('    debug_flag: true', '    debug_flag: false'):
        lines[index:index + 1] = [line, debug_dir_line, debug_max_frames_line]
        inserted = True
        break
if not inserted:
    for index, line in enumerate(lines):
        if line == imu_pose_prediction_line:
            lines[index:index + 1] = [line, debug_dir_line, debug_max_frames_line]
            inserted = True
            break
if not inserted:
    raise SystemExit('could not find scan_matcher insertion point for debug cloud dump parameters')
if cloud_queue_depth:
    cloud_queue_line = f'    cloud_queue_depth: {int(cloud_queue_depth)}'
    replaced = False
    for index, line in enumerate(lines):
        if line.startswith('    cloud_queue_depth:'):
            lines[index] = cloud_queue_line
            replaced = True
            break
    if not replaced:
        inserted_queue = False
        for index, line in enumerate(lines):
            if line.startswith('    debug_cloud_dump_max_frames:'):
                lines[index:index + 1] = [line, cloud_queue_line]
                inserted_queue = True
                break
        if not inserted_queue:
            raise SystemExit('could not find scan_matcher insertion point for cloud_queue_depth')
text = '\n'.join(lines) + '\n'
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

wait_for_nonempty_file() {
  local path="$1"
  local timeout_secs="$2"
  local deadline=$((SECONDS + timeout_secs))
  while (( SECONDS < deadline )); do
    if [[ -s "${path}" ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

terminate_pid() {
  local pid="$1"
  [[ -n "${pid}" ]] || return 0

  if kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! kill -0 "${pid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        return 0
      fi
      sleep 0.5
    done
    kill -9 "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
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
        --staging-root "${OUTPUT_DIR}"
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
REFERENCE_TUM=""
ROBOT_FRAME_ID_OVERRIDE=""
GNSS_BAG=""
GNSS_TOPIC="/gnss/fix"
USE_ODOM_PRIOR="false"
ODOM_BAG=""
ODOM_TOPIC="/tf"
ODOM_FRAME_ID="odom"
ODOM_PRIOR_PLANAR="false"
ODOM_PRIOR_VELOCITY_PLANAR="false"
ODOM_PRIOR_TRANSLATION_ONLY="false"
ODOM_PRIOR_WEIGHT="1.0"
ODOM_PRIOR_SUSPECT_RECOVERY_ONLY="false"
USE_IMU="false"
IMU_BAG=""
IMU_TOPIC="/imu"
IMU_FRAME_ID="base_link"
IMU_TRANSLATION_DESKEW="false"
IMU_ROTATION_USE_ORIENTATION="true"
IMU_POSE_PREDICTION="false"
CLOUD_QUEUE_DEPTH=""
DEBUG_CLOUD_DUMP_MAX_FRAMES="0"
DEBUG_CLOUD_DUMP_DIR=""
TF_BAG=""
TF_STATIC_TOPIC="/tf_static"
GSOF49_TOPIC="/lvx_client/gsof/ins_solution_49"
GSOF50_TOPIC="/lvx_client/gsof/ins_solution_rms_50"
APPLANIX_MSG_DIR="/tmp/applanix/applanix_msgs/msg"
VELODYNE_OVERLAY="/tmp/velodyne_ws"
VELODYNE_MODEL="VLP16"
VELODYNE_CALIBRATION=""
PARAM_FILE="${REPO_ROOT}/lidarslam/param/lidarslam.yaml"
OUTPUT_DIR=""
RATE="1.0"
PLAY_WALL_SEC=""
DRAIN_SEC="8"
USE_GNSS="true"
VERIFY_MAP="false"
ROS_DOMAIN_ID_OVERRIDE=""
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
    --reference-tum)
      REFERENCE_TUM="$(realpath -m "${2:-}")"; shift 2 ;;
    --robot-frame-id)
      ROBOT_FRAME_ID_OVERRIDE="${2:-}"; shift 2 ;;
    --gnss-bag)
      GNSS_BAG="$(realpath "${2:-}")"; shift 2 ;;
    --gnss-topic)
      GNSS_TOPIC="${2:-}"; shift 2 ;;
    --use-odom-prior)
      USE_ODOM_PRIOR="${2:-}"; shift 2 ;;
    --odom-bag)
      ODOM_BAG="$(realpath "${2:-}")"; shift 2 ;;
    --odom-topic)
      ODOM_TOPIC="${2:-}"; shift 2 ;;
    --odom-frame-id)
      ODOM_FRAME_ID="${2:-}"; shift 2 ;;
    --odom-prior-planar)
      ODOM_PRIOR_PLANAR="${2:-}"; shift 2 ;;
    --odom-prior-velocity-planar)
      ODOM_PRIOR_VELOCITY_PLANAR="${2:-}"; shift 2 ;;
    --odom-prior-translation-only)
      ODOM_PRIOR_TRANSLATION_ONLY="${2:-}"; shift 2 ;;
    --odom-prior-suspect-recovery-only)
      ODOM_PRIOR_SUSPECT_RECOVERY_ONLY="${2:-}"; shift 2 ;;
    --odom-prior-weight)
      ODOM_PRIOR_WEIGHT="${2:-}"; shift 2 ;;
    --use-imu)
      USE_IMU="${2:-}"; shift 2 ;;
    --imu-bag)
      IMU_BAG="$(realpath "${2:-}")"; shift 2 ;;
    --imu-topic)
      IMU_TOPIC="${2:-}"; shift 2 ;;
    --imu-frame-id)
      IMU_FRAME_ID="${2:-}"; shift 2 ;;
    --imu-translation-deskew)
      IMU_TRANSLATION_DESKEW="${2:-}"; shift 2 ;;
    --imu-rotation-use-orientation)
      IMU_ROTATION_USE_ORIENTATION="${2:-}"; shift 2 ;;
    --imu-pose-prediction)
      IMU_POSE_PREDICTION="${2:-}"; shift 2 ;;
    --cloud-queue-depth)
      CLOUD_QUEUE_DEPTH="${2:-}"; shift 2 ;;
    --debug-cloud-dump-max-frames)
      DEBUG_CLOUD_DUMP_MAX_FRAMES="${2:-}"; shift 2 ;;
    --debug-cloud-dump-dir)
      DEBUG_CLOUD_DUMP_DIR="$(realpath -m "${2:-}")"; shift 2 ;;
    --tf-bag)
      TF_BAG="$(realpath "${2:-}")"; shift 2 ;;
    --tf-static-topic)
      TF_STATIC_TOPIC="${2:-}"; shift 2 ;;
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
    --output-dir)
      OUTPUT_DIR="$(realpath -m "${2:-}")"; shift 2 ;;
    --rate)
      RATE="${2:-}"; shift 2 ;;
    --play-wall-sec)
      PLAY_WALL_SEC="${2:-}"; shift 2 ;;
    --drain-sec)
      DRAIN_SEC="${2:-}"; shift 2 ;;
    --use-gnss)
      USE_GNSS="${2:-}"; shift 2 ;;
    --verify-map)
      VERIFY_MAP="true"; shift ;;
    --ros-domain-id)
      ROS_DOMAIN_ID_OVERRIDE="${2:-}"; shift 2 ;;
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

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/open_data_applanix_velodyne_gnss_benchmark_$(timestamp)"
fi
mkdir -p "${OUTPUT_DIR}"

if (( DEBUG_CLOUD_DUMP_MAX_FRAMES > 0 )) && [[ -z "${DEBUG_CLOUD_DUMP_DIR}" ]]; then
  DEBUG_CLOUD_DUMP_DIR="${OUTPUT_DIR}/scanmatcher_debug_clouds"
fi

[[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]] || {
  die "ROS setup not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
}

if [[ -n "${ROS_DOMAIN_ID_OVERRIDE}" ]]; then
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_OVERRIDE}"
fi

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
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
fi
# shellcheck source=/dev/null
source "${VELODYNE_OVERLAY}/install/setup.bash"
set -u

command -v ros2 >/dev/null 2>&1 || die "ros2 not found"

if [[ -z "${PACKET_TOPIC}" ]]; then
  PACKET_TOPIC="$(detect_topic_by_type \
    "${BAG_PATH}" \
    "velodyne_msgs/msg/VelodyneScan" \
    "${VELODYNE_MSG_DIR}" \
    "/front/")"
fi
[[ -n "${PACKET_TOPIC}" ]] || die "failed to detect VelodyneScan topic"

LIDAR_FRAME_ID="$(detect_first_header_frame \
  "${BAG_PATH}" \
  "${PACKET_TOPIC}" \
  "${VELODYNE_MSG_DIR}")"
[[ -n "${LIDAR_FRAME_ID}" ]] || die "failed to detect frame_id for ${PACKET_TOPIC}"

ROBOT_FRAME_ID="${LIDAR_FRAME_ID}"
if [[ -n "${ROBOT_FRAME_ID_OVERRIDE}" ]]; then
  ROBOT_FRAME_ID="${ROBOT_FRAME_ID_OVERRIDE}"
fi

TF_IN_MAIN="false"
if topic_exists_by_name_and_type "${BAG_PATH}" "${TF_STATIC_TOPIC}" "tf2_msgs/msg/TFMessage"; then
  TF_IN_MAIN="true"
fi

PUBLISH_STATIC_TF="false"
STATIC_TF_X="0"
STATIC_TF_Y="0"
STATIC_TF_Z="0"
STATIC_TF_QX="0"
STATIC_TF_QY="0"
STATIC_TF_QZ="0"
STATIC_TF_QW="1"
PUBLISH_IMU_STATIC_TF="false"
STATIC_IMU_TF_X="0"
STATIC_IMU_TF_Y="0"
STATIC_IMU_TF_Z="0"
STATIC_IMU_TF_QX="0"
STATIC_IMU_TF_QY="0"
STATIC_IMU_TF_QZ="0"
STATIC_IMU_TF_QW="1"
if [[ "${ROBOT_FRAME_ID}" != "${LIDAR_FRAME_ID}" ]]; then
  if [[ "${TF_IN_MAIN}" != "true" ]]; then
    [[ -n "${TF_BAG}" ]] || {
      die "robot_frame_id (${ROBOT_FRAME_ID}) differs from lidar frame (${LIDAR_FRAME_ID}) but neither the main bag nor --tf-bag provides ${TF_STATIC_TOPIC}"
    }
    [[ -d "${TF_BAG}" ]] || die "tf bag not found: ${TF_BAG}"
    read -r STATIC_TF_X STATIC_TF_Y STATIC_TF_Z STATIC_TF_QX STATIC_TF_QY STATIC_TF_QZ STATIC_TF_QW < <(
      python3 "${SCRIPT_DIR}/extract_static_transform_from_bag.py" \
        "${TF_BAG}" \
        --topic "${TF_STATIC_TOPIC}" \
        --source-frame "${ROBOT_FRAME_ID}" \
        --target-frame "${LIDAR_FRAME_ID}"
    )
    PUBLISH_STATIC_TF="true"
  fi
fi
if [[ -z "${VELODYNE_CALIBRATION}" ]]; then
  VELODYNE_CALIBRATION="$(default_calibration_for_model "${VELODYNE_OVERLAY}" "${VELODYNE_MODEL}")"
fi
[[ -f "${VELODYNE_CALIBRATION}" ]] || die "velodyne calibration not found: ${VELODYNE_CALIBRATION}"

REFERENCE_TUM_AUTO="${OUTPUT_DIR}/reference_applanix.tum"
REFERENCE_META="${OUTPUT_DIR}/reference_applanix.json"
if [[ -z "${REFERENCE_TUM}" ]]; then
  REFERENCE_TUM="${REFERENCE_TUM_AUTO}"
  python3 "${SCRIPT_DIR}/extract_applanix_gsof49_reference.py" \
    --input "${BAG_PATH}" \
    --output "${REFERENCE_TUM}" \
    --topic "${GSOF49_TOPIC}" \
    --applanix-msg-dir "${APPLANIX_MSG_DIR}" \
    --meta-out "${REFERENCE_META}" \
    >"${OUTPUT_DIR}/reference_extract.log" 2>&1
fi
[[ -f "${REFERENCE_TUM}" ]] || die "reference TUM not found: ${REFERENCE_TUM}"

CONVERT_LOG="${OUTPUT_DIR}/convert_applanix.log"
ODOM_CONVERT_LOG="${OUTPUT_DIR}/convert_applanix_tf.log"
IMU_CONVERT_LOG="${OUTPUT_DIR}/convert_applanix_imu.log"
GNSS_FROM_MAIN="false"
IMU_FROM_MAIN="false"
if [[ "${USE_GNSS,,}" == "true" ]]; then
  if [[ -n "${GNSS_BAG}" ]]; then
    [[ -d "${GNSS_BAG}" ]] || die "gnss bag not found: ${GNSS_BAG}"
    [[ -f "${GNSS_BAG}/metadata.yaml" ]] || die "metadata.yaml not found under ${GNSS_BAG}"
  elif topic_exists_by_name_and_type \
    "${BAG_PATH}" \
    "${GNSS_TOPIC}" \
    "sensor_msgs/msg/NavSatFix"
  then
    GNSS_FROM_MAIN="true"
  else
    NATIVE_GNSS_TOPIC="$(detect_topic_by_type "${BAG_PATH}" "sensor_msgs/msg/NavSatFix")"
    if [[ -n "${NATIVE_GNSS_TOPIC}" ]]; then
      GNSS_TOPIC="${NATIVE_GNSS_TOPIC}"
      GNSS_FROM_MAIN="true"
    else
      [[ -d "${APPLANIX_MSG_DIR}" ]] || {
        die "applanix_msgs dir not found: ${APPLANIX_MSG_DIR}"
      }
      GNSS_BAG="${OUTPUT_DIR}/applanix_navsatfix_sidecar"
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
  fi
fi

if [[ "${USE_IMU,,}" == "true" ]]; then
  if [[ -n "${IMU_BAG}" ]]; then
    [[ -d "${IMU_BAG}" ]] || die "imu bag not found: ${IMU_BAG}"
    [[ -f "${IMU_BAG}/metadata.yaml" ]] || die "metadata.yaml not found under ${IMU_BAG}"
  elif topic_exists_by_name_and_type \
    "${BAG_PATH}" \
    "${IMU_TOPIC}" \
    "sensor_msgs/msg/Imu"
  then
    IMU_FROM_MAIN="true"
  else
    NATIVE_IMU_TOPIC="$(detect_topic_by_type "${BAG_PATH}" "sensor_msgs/msg/Imu")"
    if [[ -n "${NATIVE_IMU_TOPIC}" ]]; then
      IMU_TOPIC="${NATIVE_IMU_TOPIC}"
      IMU_FROM_MAIN="true"
    else
      [[ -d "${APPLANIX_MSG_DIR}" ]] || {
        die "applanix_msgs dir not found: ${APPLANIX_MSG_DIR}"
      }
      IMU_BAG="${OUTPUT_DIR}/applanix_imu_sidecar"
      python3 "${SCRIPT_DIR}/convert_applanix_gsof_to_imu_bag.py" \
        --input "${BAG_PATH}" \
        --output "${IMU_BAG}" \
        --gsof49-topic "${GSOF49_TOPIC}" \
        --gsof50-topic "${GSOF50_TOPIC}" \
        --output-topic "${IMU_TOPIC}" \
        --frame-id "${IMU_FRAME_ID}" \
        --applanix-msg-dir "${APPLANIX_MSG_DIR}" \
        --force \
        >"${IMU_CONVERT_LOG}" 2>&1
    fi
  fi
fi

if [[ "${USE_ODOM_PRIOR,,}" == "true" ]]; then
  if [[ -n "${ODOM_BAG}" ]]; then
    [[ -d "${ODOM_BAG}" ]] || die "odom bag not found: ${ODOM_BAG}"
    [[ -f "${ODOM_BAG}/metadata.yaml" ]] || die "metadata.yaml not found under ${ODOM_BAG}"
  else
    [[ -d "${APPLANIX_MSG_DIR}" ]] || {
      die "applanix_msgs dir not found: ${APPLANIX_MSG_DIR}"
    }
    ODOM_BAG="${OUTPUT_DIR}/applanix_tf_sidecar"
    python3 "${SCRIPT_DIR}/convert_applanix_gsof49_to_tf_bag.py" \
      --input "${BAG_PATH}" \
      --output "${ODOM_BAG}" \
      --topic "${GSOF49_TOPIC}" \
      --output-topic "${ODOM_TOPIC}" \
      --odom-frame-id "${ODOM_FRAME_ID}" \
      --child-frame-id "${ROBOT_FRAME_ID}" \
      --applanix-msg-dir "${APPLANIX_MSG_DIR}" \
      $([[ "${ODOM_PRIOR_PLANAR,,}" == "true" ]] && printf '%s' '--planar') \
      $([[ "${ODOM_PRIOR_VELOCITY_PLANAR,,}" == "true" ]] && printf '%s' '--integrate-velocity-planar') \
      --force \
      >"${ODOM_CONVERT_LOG}" 2>&1
  fi
fi

if [[ "${USE_IMU,,}" == "true" && "${IMU_FRAME_ID}" != "${ROBOT_FRAME_ID}" ]]; then
  if [[ -z "${TF_BAG}" ]]; then
    TF_BAG="${BAG_PATH}"
  fi
  [[ -d "${TF_BAG}" ]] || die "tf bag not found: ${TF_BAG}"
  read -r STATIC_IMU_TF_X STATIC_IMU_TF_Y STATIC_IMU_TF_Z STATIC_IMU_TF_QX STATIC_IMU_TF_QY STATIC_IMU_TF_QZ STATIC_IMU_TF_QW < <(
    python3 "${SCRIPT_DIR}/extract_static_transform_from_bag.py" \
      "${TF_BAG}" \
      --topic "${TF_STATIC_TOPIC}" \
      --source-frame "${ROBOT_FRAME_ID}" \
      --target-frame "${IMU_FRAME_ID}"
  )
  PUBLISH_IMU_STATIC_TF="true"
fi

if [[ -z "${PLAY_WALL_SEC}" ]]; then
  PLAY_WALL_SEC="$(compute_play_wall_sec "${BAG_PATH}" "${RATE}")"
fi

TMP_PARAM="$(mktemp --suffix=.yaml)"
VELODYNE_PARAM="$(mktemp --suffix=.yaml)"
QOS_FILE="$(mktemp --suffix=.yaml)"
create_main_param \
  "${PARAM_FILE}" \
  "${TMP_PARAM}" \
  "${USE_GNSS}" \
  "${USE_IMU}" \
  "${USE_ODOM_PRIOR}" \
  "${IMU_TRANSLATION_DESKEW}" \
  "${IMU_ROTATION_USE_ORIENTATION}" \
  "${IMU_POSE_PREDICTION}" \
  "${CLOUD_QUEUE_DEPTH}" \
  "${DEBUG_CLOUD_DUMP_DIR}" \
  "${DEBUG_CLOUD_DUMP_MAX_FRAMES}" \
  "${ODOM_PRIOR_PLANAR}" \
  "${ODOM_PRIOR_TRANSLATION_ONLY}" \
  "${ODOM_PRIOR_WEIGHT}" \
  "${ODOM_PRIOR_SUSPECT_RECOVERY_ONLY}"

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

EFFECTIVE_MAIN_PARAM="${OUTPUT_DIR}/lidarslam_params.effective.yaml"
EFFECTIVE_VELODYNE_PARAM="${OUTPUT_DIR}/velodyne_params.effective.yaml"
cp -- "${TMP_PARAM}" "${EFFECTIVE_MAIN_PARAM}"
cp -- "${VELODYNE_PARAM}" "${EFFECTIVE_VELODYNE_PARAM}"

LAUNCH_LOG="${OUTPUT_DIR}/lidarslam.launch.log"
MAP_SAVE_LOG="${OUTPUT_DIR}/map_save.log"
MAIN_PLAY_LOG="${OUTPUT_DIR}/main_bag_play.log"
GNSS_PLAY_LOG="${OUTPUT_DIR}/gnss_bag_play.log"
ODOM_PLAY_LOG="${OUTPUT_DIR}/odom_bag_play.log"
IMU_PLAY_LOG="${OUTPUT_DIR}/imu_bag_play.log"
VELODYNE_LOG="${OUTPUT_DIR}/velodyne_transform.log"
VERIFY_LOG="${OUTPUT_DIR}/verify_autoware_map.log"
RAW_TUM="${OUTPUT_DIR}/traj_raw.tum"
CORRECTED_TUM="${OUTPUT_DIR}/traj_corrected.tum"
RAW_LOG="${OUTPUT_DIR}/path_logger_raw.log"
CORRECTED_LOG="${OUTPUT_DIR}/path_logger_corrected.log"
POINTS_TOPIC="/open_data/velodyne_points"

LAUNCH_PID=""
MAIN_PLAY_PID=""
GNSS_PLAY_PID=""
ODOM_PLAY_PID=""
IMU_PLAY_PID=""
VELODYNE_PID=""
IMU_STATIC_TF_PID=""
RAW_LOGGER_PID=""
CORRECTED_LOGGER_PID=""
STAGED_PLAYBACK_DIRS=()
MAIN_PLAY_BAG=""
GNSS_PLAY_BAG=""
ODOM_PLAY_BAG=""
IMU_PLAY_BAG=""
cleanup() {
  for pid in \
    "${GNSS_PLAY_PID}" \
    "${ODOM_PLAY_PID}" \
    "${IMU_PLAY_PID}" \
    "${MAIN_PLAY_PID}" \
    "${RAW_LOGGER_PID}" \
    "${CORRECTED_LOGGER_PID}" \
    "${IMU_STATIC_TF_PID}" \
    "${VELODYNE_PID}" \
    "${LAUNCH_PID}"
  do
    terminate_pid "${pid}"
  done
  rm -f "${TMP_PARAM}" "${VELODYNE_PARAM}" "${QOS_FILE}"
  cleanup_playback_staging
}
trap cleanup EXIT INT TERM

stage_playback_bag "${BAG_PATH}" "${OUTPUT_DIR}" MAIN_PLAY_BAG
if [[ "${USE_GNSS,,}" == "true" && "${GNSS_FROM_MAIN}" != "true" ]]; then
  stage_playback_bag "${GNSS_BAG}" "${OUTPUT_DIR}" GNSS_PLAY_BAG
fi
if [[ "${USE_ODOM_PRIOR,,}" == "true" ]]; then
  stage_playback_bag "${ODOM_BAG}" "${OUTPUT_DIR}" ODOM_PLAY_BAG
fi
if [[ "${USE_IMU,,}" == "true" && "${IMU_FROM_MAIN}" != "true" ]]; then
  stage_playback_bag "${IMU_BAG}" "${OUTPUT_DIR}" IMU_PLAY_BAG
fi

BENCH_T0="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"
STARTED_AT="$(date -Iseconds)"
STARTED_AT_UNIX="$(date +%s)"

echo "Running Applanix + Velodyne GNSS benchmark:"
echo "  bag:                 ${BAG_PATH}"
echo "  packet_topic:        ${PACKET_TOPIC}"
echo "  reference_tum:       ${REFERENCE_TUM}"
echo "  use_gnss:            ${USE_GNSS}"
if [[ "${USE_GNSS,,}" == "true" ]]; then
  if [[ "${GNSS_FROM_MAIN}" == "true" ]]; then
    echo "  gnss_source:         main bag"
  else
    echo "  gnss_source:         sidecar bag"
    echo "  gnss_bag:            ${GNSS_BAG}"
  fi
  echo "  gnss_topic:          ${GNSS_TOPIC}"
fi
echo "  use_odom_prior:      ${USE_ODOM_PRIOR}"
if [[ "${USE_ODOM_PRIOR,,}" == "true" ]]; then
  echo "  odom_source:         sidecar bag"
  echo "  odom_bag:            ${ODOM_BAG}"
  echo "  odom_topic:          ${ODOM_TOPIC}"
  echo "  odom_frame_id:       ${ODOM_FRAME_ID}"
  echo "  odom_prior_planar:   ${ODOM_PRIOR_PLANAR}"
  echo "  odom_prior_velocity_planar: ${ODOM_PRIOR_VELOCITY_PLANAR}"
  echo "  odom_prior_translation_only: ${ODOM_PRIOR_TRANSLATION_ONLY}"
  echo "  odom_prior_weight:   ${ODOM_PRIOR_WEIGHT}"
  echo "  odom_prior_suspect_recovery_only: ${ODOM_PRIOR_SUSPECT_RECOVERY_ONLY}"
fi
echo "  use_imu:             ${USE_IMU}"
if [[ "${USE_IMU,,}" == "true" ]]; then
  if [[ "${IMU_FROM_MAIN}" == "true" ]]; then
    echo "  imu_source:          main bag"
  else
    echo "  imu_source:          sidecar bag"
    echo "  imu_bag:             ${IMU_BAG}"
  fi
  echo "  imu_topic:           ${IMU_TOPIC}"
  echo "  imu_frame:           ${IMU_FRAME_ID}"
  echo "  imu_translation_deskew:${IMU_TRANSLATION_DESKEW}"
  echo "  imu_rotation_use_orientation:${IMU_ROTATION_USE_ORIENTATION}"
  echo "  imu_pose_prediction: ${IMU_POSE_PREDICTION}"
fi
if [[ -n "${CLOUD_QUEUE_DEPTH}" ]]; then
  echo "  cloud_queue_depth:   ${CLOUD_QUEUE_DEPTH}"
fi
echo "  debug_cloud_dump_max_frames:${DEBUG_CLOUD_DUMP_MAX_FRAMES}"
if (( DEBUG_CLOUD_DUMP_MAX_FRAMES > 0 )); then
  echo "  debug_cloud_dump_dir:${DEBUG_CLOUD_DUMP_DIR}"
fi
echo "  rate:                ${RATE}"
echo "  play_wall_sec:       ${PLAY_WALL_SEC}"
if [[ -n "${ROS_DOMAIN_ID_OVERRIDE}" ]]; then
  echo "  ros_domain_id:       ${ROS_DOMAIN_ID_OVERRIDE}"
fi
echo "  velodyne_model:      ${VELODYNE_MODEL}"
echo "  velodyne_calibration:${VELODYNE_CALIBRATION}"
echo "  robot_frame:         ${ROBOT_FRAME_ID}"
echo "  lidar_frame:         ${LIDAR_FRAME_ID}"
echo "  tf_in_main_bag:      ${TF_IN_MAIN}"
if [[ "${MAIN_PLAY_BAG}" != "${BAG_PATH}" ]]; then
  echo "  playback_staging:    isolated FILE-compressed bag"
fi
if [[ "${PUBLISH_STATIC_TF}" == "true" ]]; then
  echo "  tf_bag:              ${TF_BAG}"
  echo "  static_tf:           ${STATIC_TF_X} ${STATIC_TF_Y} ${STATIC_TF_Z} ${STATIC_TF_QX} ${STATIC_TF_QY} ${STATIC_TF_QZ} ${STATIC_TF_QW}"
fi
if [[ "${PUBLISH_IMU_STATIC_TF}" == "true" ]]; then
  echo "  imu_static_tf:       ${STATIC_IMU_TF_X} ${STATIC_IMU_TF_Y} ${STATIC_IMU_TF_Z} ${STATIC_IMU_TF_QX} ${STATIC_IMU_TF_QY} ${STATIC_IMU_TF_QZ} ${STATIC_IMU_TF_QW}"
fi
echo "  output_dir:          ${OUTPUT_DIR}"

ros2 run velodyne_pointcloud velodyne_transform_node \
  --ros-args \
  --params-file "${VELODYNE_PARAM}" \
  -r "velodyne_packets:=${PACKET_TOPIC}" \
  -r "velodyne_points:=${POINTS_TOPIC}" \
  >"${VELODYNE_LOG}" 2>&1 &
VELODYNE_PID="$!"

if [[ "${PUBLISH_IMU_STATIC_TF}" == "true" ]]; then
  ros2 run tf2_ros static_transform_publisher \
    "${STATIC_IMU_TF_X}" \
    "${STATIC_IMU_TF_Y}" \
    "${STATIC_IMU_TF_Z}" \
    "${STATIC_IMU_TF_QX}" \
    "${STATIC_IMU_TF_QY}" \
    "${STATIC_IMU_TF_QZ}" \
    "${STATIC_IMU_TF_QW}" \
    "${ROBOT_FRAME_ID}" \
    "${IMU_FRAME_ID}" \
    >"${OUTPUT_DIR}/imu_static_tf.log" 2>&1 &
  IMU_STATIC_TF_PID="$!"
fi

ros2 launch lidarslam lidarslam.launch.py \
  "main_param_dir:=${TMP_PARAM}" \
  "input_cloud:=${POINTS_TOPIC}" \
  "imu_topic:=${IMU_TOPIC}" \
  "gnss_topic:=${GNSS_TOPIC}" \
  "robot_frame_id:=${ROBOT_FRAME_ID}" \
  "odom_frame_id:=${ODOM_FRAME_ID}" \
  "base_frame:=${ROBOT_FRAME_ID}" \
  "lidar_frame:=${LIDAR_FRAME_ID}" \
  "global_frame_id:=map" \
  "use_graph_based_slam:=true" \
  "use_sim_time:=true" \
  "publish_static_tf:=${PUBLISH_STATIC_TF}" \
  "static_tf_x:=${STATIC_TF_X}" \
  "static_tf_y:=${STATIC_TF_Y}" \
  "static_tf_z:=${STATIC_TF_Z}" \
  "static_tf_qx:=${STATIC_TF_QX}" \
  "static_tf_qy:=${STATIC_TF_QY}" \
  "static_tf_qz:=${STATIC_TF_QZ}" \
  "static_tf_qw:=${STATIC_TF_QW}" \
  "save_dir:=${OUTPUT_DIR}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID="$!"

python3 "${SCRIPT_DIR}/path_to_tum.py" \
  --topic /path \
  --output "${RAW_TUM}" \
  --use-sim-time true \
  >"${RAW_LOG}" 2>&1 &
RAW_LOGGER_PID="$!"

python3 "${SCRIPT_DIR}/path_to_tum.py" \
  --topic /modified_path \
  --output "${CORRECTED_TUM}" \
  --use-sim-time true \
  >"${CORRECTED_LOG}" 2>&1 &
CORRECTED_LOGGER_PID="$!"

sleep 5

MAIN_PLAY_TOPICS=("${PACKET_TOPIC}")
if [[ "${TF_IN_MAIN}" == "true" ]]; then
  MAIN_PLAY_TOPICS+=("${TF_STATIC_TOPIC}")
fi
if [[ "${GNSS_FROM_MAIN}" == "true" ]]; then
  MAIN_PLAY_TOPICS+=("${GNSS_TOPIC}")
fi
if [[ "${IMU_FROM_MAIN}" == "true" ]]; then
  MAIN_PLAY_TOPICS+=("${IMU_TOPIC}")
fi

timeout "${PLAY_WALL_SEC}" ros2 bag play "${MAIN_PLAY_BAG}" \
  --clock \
  --rate "${RATE}" \
  --topics "${MAIN_PLAY_TOPICS[@]}" \
  --qos-profile-overrides-path "${QOS_FILE}" \
  >"${MAIN_PLAY_LOG}" 2>&1 &
MAIN_PLAY_PID="$!"

if [[ "${USE_GNSS,,}" == "true" && "${GNSS_FROM_MAIN}" != "true" ]]; then
  timeout "${PLAY_WALL_SEC}" ros2 bag play "${GNSS_PLAY_BAG}" \
    --rate "${RATE}" \
    >"${GNSS_PLAY_LOG}" 2>&1 &
  GNSS_PLAY_PID="$!"
fi
if [[ "${USE_ODOM_PRIOR,,}" == "true" ]]; then
  timeout "${PLAY_WALL_SEC}" ros2 bag play "${ODOM_PLAY_BAG}" \
    --rate "${RATE}" \
    >"${ODOM_PLAY_LOG}" 2>&1 &
  ODOM_PLAY_PID="$!"
fi
if [[ "${USE_IMU,,}" == "true" && "${IMU_FROM_MAIN}" != "true" ]]; then
  timeout "${PLAY_WALL_SEC}" ros2 bag play "${IMU_PLAY_BAG}" \
    --rate "${RATE}" \
    >"${IMU_PLAY_LOG}" 2>&1 &
  IMU_PLAY_PID="$!"
fi

wait "${MAIN_PLAY_PID}" || true
MAIN_PLAY_PID=""
if [[ -n "${GNSS_PLAY_PID}" ]]; then
  wait "${GNSS_PLAY_PID}" || true
  GNSS_PLAY_PID=""
fi
if [[ -n "${ODOM_PLAY_PID}" ]]; then
  wait "${ODOM_PLAY_PID}" || true
  ODOM_PLAY_PID=""
fi
if [[ -n "${IMU_PLAY_PID}" ]]; then
  wait "${IMU_PLAY_PID}" || true
  IMU_PLAY_PID=""
fi

sleep "${DRAIN_SEC}"

if ! call_map_save_with_retry "${MAP_SAVE_LOG}"; then
  echo "map_save service call failed. Recent launch log:" >&2
  tail -n 120 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

if ! wait_for_nonempty_file "${RAW_TUM}" 60; then
  echo "raw trajectory not found or empty: ${RAW_TUM}" >&2
  tail -n 120 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi
if ! wait_for_nonempty_file "${CORRECTED_TUM}" 60; then
  echo "corrected trajectory not found or empty: ${CORRECTED_TUM}" >&2
  tail -n 120 "${LAUNCH_LOG}" >&2 || true
  exit 1
fi

if [[ "${VERIFY_MAP}" == "true" ]]; then
  python3 "${REPO_ROOT}/scripts/verify_autoware_map.py" "${OUTPUT_DIR}" >"${VERIFY_LOG}" 2>&1
fi

terminate_pid "${RAW_LOGGER_PID}"
RAW_LOGGER_PID=""
terminate_pid "${CORRECTED_LOGGER_PID}"
CORRECTED_LOGGER_PID=""
terminate_pid "${VELODYNE_PID}"
VELODYNE_PID=""
terminate_pid "${LAUNCH_PID}"
LAUNCH_PID=""

BENCH_T1="$(python3 - <<'PY'
import time
print(time.monotonic())
PY
)"
WALL_SEC="$(python3 - "${BENCH_T0}" "${BENCH_T1}" <<'PY'
import sys
print(float(sys.argv[2]) - float(sys.argv[1]))
PY
)"

python3 "${SCRIPT_DIR}/write_aligned_trajectory_metrics.py" \
  --out-dir "${OUTPUT_DIR}" \
  --bag "${BAG_PATH}" \
  --reference-tum "${REFERENCE_TUM}" \
  --corrected-tum "${CORRECTED_TUM}" \
  --raw-tum "${RAW_TUM}" \
  --graph-log "${LAUNCH_LOG}" \
  --lidarslam-param "${TMP_PARAM}" \
  --parameter-file "${EFFECTIVE_MAIN_PARAM}" \
  --parameter-file "${EFFECTIVE_VELODYNE_PARAM}" \
  --benchmark-harness "${BASH_SOURCE[0]}" \
  --runtime-artifact "velodyne_transform_node=${VELODYNE_OVERLAY}/install/velodyne_pointcloud/lib/velodyne_pointcloud/velodyne_transform_node" \
  --runtime-artifact "scanmatcher_node=$(ros2 pkg prefix scanmatcher)/lib/scanmatcher/scanmatcher_node" \
  --runtime-artifact "graph_based_slam_node=$(ros2 pkg prefix graph_based_slam)/lib/graph_based_slam/graph_based_slam_node" \
  --points-topic "${POINTS_TOPIC}" \
  --points-frame "${LIDAR_FRAME_ID}" \
  --robot-frame "${ROBOT_FRAME_ID}" \
  --reference-source "applanix_gsof49_reference" \
  --reference-kind "cross_validation" \
  --reference-label "Applanix_GSOF49" \
  --wall-sec "${WALL_SEC}" \
  --started-at "${STARTED_AT}" \
  --started-at-unix "${STARTED_AT_UNIX}" \
  >"${OUTPUT_DIR}/metrics_path.txt"

if [[ -f "${OUTPUT_DIR}/map_projector_info.yaml" ]]; then
  echo "map_projector_info.yaml:"
  cat "${OUTPUT_DIR}/map_projector_info.yaml"
fi
echo "metrics_json: ${OUTPUT_DIR}/metrics.json"
echo "done: ${OUTPUT_DIR}"
