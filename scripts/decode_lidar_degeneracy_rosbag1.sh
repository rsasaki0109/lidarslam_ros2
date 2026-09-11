#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage: decode_lidar_degeneracy_rosbag1.sh INPUT.bag METADATA.json OUTPUT.bag

Decode an NTNU LiDAR Degeneracy packet bag with the authors' trigger-aware
Ouster driver. The output is an intermediate ROS1 bag containing PointCloud2,
the original VN100 IMU, FMCW radar cloud, and trigger_2. Run
normalize_lidar_degeneracy_rosbag.py afterward to reschedule sensor records
onto their header clock.
EOF
}

if [[ $# -ne 3 ]]; then
  usage
  exit 2
fi

INPUT=$(realpath "$1")
METADATA=$(realpath "$2")
OUTPUT=$(realpath -m "$3")
IMAGE=${OUSTER_SYNC_IMAGE:-ouster-sync-replay:noetic-e6531d0}

[[ -f "$INPUT" ]] || { echo "error: missing input: $INPUT" >&2; exit 2; }
[[ -f "$METADATA" ]] || { echo "error: missing metadata: $METADATA" >&2; exit 2; }
[[ ! -e "$OUTPUT" ]] || { echo "error: refusing to overwrite: $OUTPUT" >&2; exit 2; }
docker image inspect "$IMAGE" >/dev/null

INPUT_DIR=$(dirname "$INPUT")
OUTPUT_DIR=$(dirname "$OUTPUT")
mkdir -p "$OUTPUT_DIR"
INPUT_NAME=$(basename "$INPUT")
METADATA_NAME=$(basename "$METADATA")
OUTPUT_NAME=$(basename "$OUTPUT")
OUTPUT_STEM=${OUTPUT_NAME%.bag}

if [[ "$(dirname "$METADATA")" != "$INPUT_DIR" ]]; then
  echo "error: metadata must be in the input bag directory" >&2
  exit 2
fi

docker run --rm --entrypoint /bin/bash \
  -v "$INPUT_DIR:/input:ro" \
  -v "$OUTPUT_DIR:/output" \
  "$IMAGE" -lc "
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /opt/ouster_sync_ws/devel/setup.bash
roscore >/output/${OUTPUT_STEM}_roscore.log 2>&1 &
core=\$!
recorder=''
cleanup() {
  if [[ -n \"\$recorder\" ]]; then
    kill -INT \"\$recorder\" 2>/dev/null || true
    wait \"\$recorder\" 2>/dev/null || true
  fi
  kill -INT \"\$core\" 2>/dev/null || true
  wait \"\$core\" 2>/dev/null || true
}
trap cleanup EXIT
sleep 2
rosbag record -O /output/${OUTPUT_NAME} \
  /os_cloud_node/points \
  /vectornav_node/uncomp_imu \
  /radar/cloud \
  /sensor_sync_node/trigger_2 \
  >/output/${OUTPUT_STEM}_record.log 2>&1 &
recorder=\$!
sleep 1
roslaunch ouster_ros replay.launch \
  metadata:=/input/${METADATA_NAME} \
  bag_file:=/input/${INPUT_NAME} \
  proc_mask:=PCL viz:=false \
  >/output/${OUTPUT_STEM}_replay.log 2>&1
sleep 2
cleanup
trap - EXIT
chmod a+rw /output/${OUTPUT_NAME} /output/${OUTPUT_STEM}_*.log
"

echo "$OUTPUT"
