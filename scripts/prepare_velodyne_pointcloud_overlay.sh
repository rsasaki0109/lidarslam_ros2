#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash scripts/prepare_velodyne_pointcloud_overlay.sh [options]

Options:
  --overlay-dir DIR    Target overlay workspace (default: /tmp/velodyne_ws).
  --ros-distro DISTRO  ROS 2 distro to source (default: $ROS_DISTRO or jazzy).
  --branch NAME        Git branch for upstream repos (default: ros2).
  --check              Exit successfully only when the overlay is complete.

This clones the minimum upstream repos needed to run
`velodyne_pointcloud/velodyne_transform_node` without sudo and builds:

  - angles
  - diagnostic_updater
  - velodyne_msgs
  - velodyne_pointcloud
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

overlay_is_ready() {
  local overlay_dir="$1"

  [[ -f "${overlay_dir}/install/setup.bash" ]] &&
    [[ -f "${overlay_dir}/install/velodyne_msgs/share/velodyne_msgs/msg/VelodyneScan.msg" ]] &&
    [[ -x "${overlay_dir}/install/velodyne_pointcloud/lib/velodyne_pointcloud/velodyne_transform_node" ]] &&
    [[ -f "${overlay_dir}/install/velodyne_pointcloud/share/velodyne_pointcloud/params/VLP16db.yaml" ]]
}

OVERLAY_DIR="/tmp/velodyne_ws"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
UPSTREAM_BRANCH="ros2"
CHECK_ONLY="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --overlay-dir)
      OVERLAY_DIR="$(realpath -m "${2:-}")"
      shift 2
      ;;
    --ros-distro)
      ROS_DISTRO_NAME="${2:-}"
      shift 2
      ;;
    --branch)
      UPSTREAM_BRANCH="${2:-}"
      shift 2
      ;;
    --check)
      CHECK_ONLY="true"
      shift
      ;;
    *)
      die "unknown arg: $1"
      ;;
  esac
done

if [[ "${CHECK_ONLY}" == "true" ]]; then
  overlay_is_ready "${OVERLAY_DIR}" || die "velodyne overlay is incomplete: ${OVERLAY_DIR}"
  echo "overlay_ready: ${OVERLAY_DIR}"
  exit 0
fi

[[ -n "${ROS_DISTRO_NAME}" ]] || die "ROS distro is empty"
[[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]] || {
  die "ROS setup not found: /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
}

mkdir -p "${OVERLAY_DIR}/src"

if [[ ! -d "${OVERLAY_DIR}/src/velodyne/.git" ]]; then
  git clone \
    --depth=1 \
    --branch "${UPSTREAM_BRANCH}" \
    https://github.com/ros-drivers/velodyne.git \
    "${OVERLAY_DIR}/src/velodyne"
fi

if [[ ! -d "${OVERLAY_DIR}/src/diagnostics/.git" ]]; then
  git clone \
    --depth=1 \
    --branch "${UPSTREAM_BRANCH}" \
    https://github.com/ros/diagnostics.git \
    "${OVERLAY_DIR}/src/diagnostics"
fi

if [[ ! -d "${OVERLAY_DIR}/src/angles/.git" ]]; then
  git clone \
    --depth=1 \
    --branch "${UPSTREAM_BRANCH}" \
    https://github.com/ros/angles.git \
    "${OVERLAY_DIR}/src/angles"
fi

set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

colcon --log-base "${OVERLAY_DIR}/log" build \
  --base-paths \
    "${OVERLAY_DIR}/src/angles" \
    "${OVERLAY_DIR}/src/diagnostics" \
    "${OVERLAY_DIR}/src/velodyne" \
  --packages-select angles diagnostic_updater velodyne_msgs velodyne_pointcloud \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --build-base "${OVERLAY_DIR}/build" \
  --install-base "${OVERLAY_DIR}/install"

echo "overlay_dir: ${OVERLAY_DIR}"
echo "setup_bash: ${OVERLAY_DIR}/install/setup.bash"
