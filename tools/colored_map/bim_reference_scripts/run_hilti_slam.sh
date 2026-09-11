#!/bin/bash
# Run scanmatcher SLAM on a HILTI sequence -> world /map -> PLY.
# HILTI has no tf, so run SLAM directly in the LiDAR frame (robot_frame=PandarXT-32).
set -e
SLAM_PID=""
CAP_PID=""
cleanup() {
  trap - EXIT INT TERM
  [ -z "$CAP_PID" ] || kill -INT "$CAP_PID" 2>/dev/null || true
  [ -z "$SLAM_PID" ] || kill -INT "$SLAM_PID" 2>/dev/null || true
  sleep 1
  [ -z "$CAP_PID" ] || kill -9 "$CAP_PID" 2>/dev/null || true
  [ -z "$SLAM_PID" ] || kill -9 "$SLAM_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM
source /opt/ros/jazzy/setup.bash
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd -- "$SCRIPT_DIR/../../.." && pwd)
source "$REPO_ROOT/install/setup.bash"
OUT=${HILTI_SLAM_OUT:-/media/sasaki/aiueo/lidarslam_work/output/bim_maps}
BAG=${1:-/media/sasaki/aiueo/datasets/hilti2022/exp07_ros2}
PREFIX=${2:-exp07}
DUR=${3:-40}
RATE=${4:-0.5}
mkdir -p "$OUT"
rm -f "$OUT/${PREFIX}_slam.log" "$OUT/${PREFIX}_map.ply" "$OUT/${PREFIX}_cap.log"
export RCUTILS_LOGGING_BUFFERED_STREAM=0

# indoor NDT: small resolution/voxels, short map-update trigger, range cap
stdbuf -oL -eL ros2 run scanmatcher scanmatcher_node --ros-args \
  -p use_sim_time:=true \
  -p global_frame_id:=map -p robot_frame_id:=PandarXT-32 -p odom_frame_id:=odom \
  -p registration_method:=NDT -p ndt_resolution:=1.0 -p ndt_num_threads:=4 \
  -p trans_for_mapupdate:=0.5 -p vg_size_for_input:=0.1 -p vg_size_for_map:=0.05 \
  -p use_min_max_filter:=true -p scan_min_range:=0.3 -p scan_max_range:=40.0 \
  -p map_publish_period:=3.0 -p use_imu:=false \
  -p reject_nonconverged_pose_update:=false \
  -p reject_fitness_ratio:=1000000000.0 -p reject_fitness_only_ratio:=1000000000.0 \
  -p reject_trans_jump:=1000000000.0 -p reject_trans_jump_ratio:=1000000000.0 \
  -p motion_gate_enable:=false -p reject_warmup_scans:=5 \
  -p set_initial_pose:=true \
  -p initial_pose_x:=0.0 -p initial_pose_y:=0.0 -p initial_pose_z:=0.0 \
  -p initial_pose_qx:=0.0 -p initial_pose_qy:=0.0 -p initial_pose_qz:=0.0 -p initial_pose_qw:=1.0 \
  -r input_cloud:=/hesai/pandar \
  > "$OUT/${PREFIX}_slam.log" 2>&1 &
SLAM_PID=$!

stdbuf -oL -eL python3 "$SCRIPT_DIR/capture_map.py" "$OUT" "$PREFIX" > "$OUT/${PREFIX}_cap.log" 2>&1 &
CAP_PID=$!
sleep 4

PLAYARGS="--clock --rate $RATE --topics /hesai/pandar"
if [ "$DUR" != "full" ]; then PLAYARGS="$PLAYARGS --playback-duration $DUR"; fi
ros2 bag play "$BAG" $PLAYARGS > "$OUT/${PREFIX}_play.log" 2>&1

sleep 4
kill -INT "$CAP_PID" 2>/dev/null || true
sleep 2
kill -INT "$SLAM_PID" 2>/dev/null || true
sleep 1
kill -9 "$SLAM_PID" "$CAP_PID" 2>/dev/null || true
wait 2>/dev/null || true
SLAM_PID=""
CAP_PID=""

echo "===== SLAM log (tail) ====="; tail -10 "$OUT/${PREFIX}_slam.log"
echo "===== capture ====="; cat "$OUT/${PREFIX}_cap.log"
echo "===== map ply ====="; ls -la "$OUT/${PREFIX}_map.ply" 2>&1
