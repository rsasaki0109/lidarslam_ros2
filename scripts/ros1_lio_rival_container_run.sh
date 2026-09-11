#!/usr/bin/env bash
set -uo pipefail

SYSTEM="${SYSTEM:?SYSTEM is required}"
BAG_PATH="${BAG_PATH:?BAG_PATH is required}"
CONFIG_PATH="${CONFIG_PATH:-/input/config.yaml}"
OUT_DIR="${OUT_DIR:-/out}"
BAG_RATE="${BAG_RATE:-1.0}"
DRAIN_SECONDS="${DRAIN_SECONDS:-5}"
FINALIZE_TIMEOUT_SECONDS="${FINALIZE_TIMEOUT_SECONDS:-600}"
MAPPER_SHUTDOWN_TIMEOUT_SECONDS="${MAPPER_SHUTDOWN_TIMEOUT_SECONDS:-120}"
SEQUENCE="${SEQUENCE:-sequence}"
EXPECTED_TRAJECTORY_END="${EXPECTED_TRAJECTORY_END:-}"
VISUAL_SHADOW_REPORT="${VISUAL_SHADOW_REPORT:-}"
VISUAL_SHADOW_TOPIC="${VISUAL_SHADOW_TOPIC:-/voxel_slam/visual_longitudinal_shadow}"

set +u
source /opt/ros/noetic/setup.bash
source /opt/lio_ws/devel/setup.bash
set -u

mkdir -p "${OUT_DIR}"

mapper_pid=''
recorder_pid=''
roscore_pid=''
visual_shadow_pid=''

stop_process_group() {
  local signal="$1"
  local pid="$2"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "-${signal}" -- "-${pid}" 2>/dev/null || true
  fi
}

process_running() {
  local pid="$1"
  local state
  kill -0 "${pid}" 2>/dev/null || return 1
  state="$(ps -o stat= -p "${pid}" 2>/dev/null || true)"
  [[ "${state}" != Z* ]]
}

trajectory_reached_end() {
  [[ -n "${EXPECTED_TRAJECTORY_END}" ]] || return 1
  [[ -s "${OUT_DIR}/odometry.csv" ]] || return 1
  awk -F, -v target="${EXPECTED_TRAJECTORY_END}" '
    NR == 1 {
      for (i = 1; i <= NF; ++i) {
        if ($i == "field.header.stamp") stamp_column = i
      }
      next
    }
    NF > 1 && stamp_column { stamp = $stamp_column }
    END {
      if (!stamp_column || stamp == "") exit 1
      if (stamp > 1.0e12) stamp *= 1.0e-9
      exit !(stamp >= target - 0.25)
    }
  ' "${OUT_DIR}/odometry.csv"
}

terminate_process_group() {
  local pid="$1"
  local deadline
  [[ -n "${pid}" ]] || return 0
  stop_process_group INT "${pid}"
  deadline=$((SECONDS + MAPPER_SHUTDOWN_TIMEOUT_SECONDS))
  while process_running "${pid}" && [[ ${SECONDS} -lt ${deadline} ]]; do
    sleep 1
  done
  if process_running "${pid}"; then
    stop_process_group TERM "${pid}"
    for _ in $(seq 1 20); do
      process_running "${pid}" || break
      sleep 0.1
    done
  fi
  if process_running "${pid}"; then
    stop_process_group KILL "${pid}"
  fi
  wait "${pid}" 2>/dev/null
}

shutdown_mapper() {
  local deadline
  # The upstream FAST-LIO2 signal handler can consume SIGINT without making
  # ros::ok() false. Ask ROS to shut the node down first so its normal epilogue
  # writes the configured map; retain the process-group path as a fallback.
  rosnode kill "${mapper_node}" >"${OUT_DIR}/mapper_shutdown.log" 2>&1 || true
  deadline=$((SECONDS + 30))
  while process_running "${mapper_pid}" && [[ ${SECONDS} -lt ${deadline} ]]; do
    sleep 1
  done
  if process_running "${mapper_pid}"; then
    terminate_process_group "${mapper_pid}"
  else
    wait "${mapper_pid}" 2>/dev/null
  fi
}

cleanup() {
  stop_process_group INT "${visual_shadow_pid}"
  stop_process_group INT "${recorder_pid}"
  stop_process_group INT "${mapper_pid}"
  stop_process_group TERM "${roscore_pid}"
}
trap cleanup EXIT TERM INT

case "${SYSTEM}" in
  fast_lio2)
    mapper=(/opt/lio_ws/devel/lib/fast_lio/fastlio_mapping)
    odometry_topic=/Odometry
    mapper_node=/laserMapping
    trajectory_path=''
    ;;
  point_lio)
    mapper=(/opt/lio_ws/devel/lib/point_lio/pointlio_mapping)
    odometry_topic=/aft_mapped_to_init
    mapper_node=/laserMapping
    trajectory_path=''
    ;;
  voxel_slam)
    mapper=(/opt/lio_ws/devel/lib/voxel_slam/voxelslam)
    # Keep the complete online estimate even when upstream starts a new
    # alidarState directory after an internal Reset. Voxel-SLAM publishes its
    # current pose as camera_init -> aft_mapped on /tf; the final state file is
    # still retained for its optimized map chunks.
    odometry_topic=/tf
    mapper_node=/cmn_voxel
    trajectory_root="${OUT_DIR}/voxel"
    mkdir -p "${OUT_DIR}/voxel"
    ;;
  *)
    echo "unsupported SYSTEM: ${SYSTEM}" >&2
    exit 64
    ;;
esac

setsid roscore >"${OUT_DIR}/roscore.log" 2>&1 &
roscore_pid=$!
for _ in $(seq 1 100); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  if ! kill -0 "${roscore_pid}" 2>/dev/null; then
    echo 'roscore exited during startup' >&2
    exit 70
  fi
  sleep 0.1
done
if ! rosparam list >/dev/null 2>&1; then
  echo 'ROS master did not become ready' >&2
  exit 70
fi

if [[ "${SYSTEM}" == point_lio ]]; then
  rosparam load "${CONFIG_PATH}" /laserMapping
else
  rosparam load "${CONFIG_PATH}"
fi
rosparam set /use_sim_time true
setsid /usr/bin/time -v -o "${OUT_DIR}/mapper_process_time.txt" \
  "${mapper[@]}" >"${OUT_DIR}/mapper.log" 2>&1 &
mapper_pid=$!

for _ in $(seq 1 300); do
  if rosnode list 2>/dev/null | grep -Fxq "${mapper_node}"; then
    break
  fi
  if ! kill -0 "${mapper_pid}" 2>/dev/null; then
    wait "${mapper_pid}" || true
    echo 'mapper exited during startup' >&2
    exit 71
  fi
  sleep 0.1
done
if ! rosnode list 2>/dev/null | grep -Fxq "${mapper_node}"; then
  echo 'mapper node did not become ready' >&2
  exit 71
fi

if [[ -n "${odometry_topic}" ]]; then
  setsid stdbuf -oL rostopic echo -p "${odometry_topic}" \
    >"${OUT_DIR}/odometry.csv" 2>"${OUT_DIR}/odometry_recorder.log" &
  recorder_pid=$!
fi

if [[ -n "${VISUAL_SHADOW_REPORT}" ]]; then
  if [[ "${SYSTEM}" != voxel_slam ]]; then
    echo 'VISUAL_SHADOW_REPORT requires SYSTEM=voxel_slam' >&2
    exit 72
  fi
  setsid python3 /runner/scripts/replay_visual_weak_axis_shadow.py \
    --report "${VISUAL_SHADOW_REPORT}" --topic "${VISUAL_SHADOW_TOPIC}" \
    >"${OUT_DIR}/visual_shadow_replay.log" 2>&1 &
  visual_shadow_pid=$!
fi

date +%s%N >"${OUT_DIR}/measurement_started_ns.txt"
/usr/bin/time -f '%e' -o "${OUT_DIR}/replay_wall_seconds.txt" \
  rosbag play --clock --quiet -r "${BAG_RATE}" "${BAG_PATH}" \
  >"${OUT_DIR}/rosbag.log" 2>&1
replay_status=$?
printf '%s\n' "${replay_status}" >"${OUT_DIR}/replay_exit_status.txt"

if [[ "${SYSTEM}" == voxel_slam ]]; then
  rosparam set /finish true
  deadline=$((SECONDS + FINALIZE_TIMEOUT_SECONDS))
  while ! find "${trajectory_root}" -mindepth 2 -maxdepth 2 \
      -type f -name alidarState.txt -size +0c -print -quit | grep -q .; do
    [[ ${SECONDS} -lt ${deadline} ]] || break
    if ! kill -0 "${mapper_pid}" 2>/dev/null; then
      break
    fi
    sleep 1
  done
else
  date +%s%N >"${OUT_DIR}/drain_started_ns.txt"
  deadline=$((SECONDS + DRAIN_SECONDS))
  reached_end=false
  while [[ ${SECONDS} -lt ${deadline} ]]; do
    if trajectory_reached_end; then
      reached_end=true
      break
    fi
    sleep 1
  done
  printf '%s\n' "${reached_end}" >"${OUT_DIR}/drain_reached_end.txt"
  date +%s%N >"${OUT_DIR}/drain_finished_ns.txt"
fi
date +%s%N >"${OUT_DIR}/measurement_finished_ns.txt"

if [[ -n "${recorder_pid}" ]]; then
  terminate_process_group "${recorder_pid}" || true
fi
if [[ -n "${visual_shadow_pid}" ]]; then
  terminate_process_group "${visual_shadow_pid}" || true
fi
shutdown_mapper
mapper_status=$?
printf '%s\n' "${mapper_status}" >"${OUT_DIR}/mapper_exit_status.txt"

if [[ ${replay_status} -ne 0 ]]; then
  exit "${replay_status}"
fi
if [[ "${SYSTEM}" == voxel_slam ]]; then
  [[ "$(find "${trajectory_root}" -mindepth 2 -maxdepth 2 \
      -type f -name alidarState.txt -size +0c | wc -l)" -eq 1 ]]
else
  test -s "${OUT_DIR}/odometry.csv"
fi
