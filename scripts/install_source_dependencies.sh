#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  echo "error: execute this helper with bash; do not source it." >&2
  return 2
fi

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash scripts/install_source_dependencies.sh [--workspace <path>] [--repo-only]

Prepare a lidar_slam_ros2 source workspace on a supported ROS 2 installation.
The helper initializes rosdep when needed, refreshes an empty rosdep cache,
updates the APT package index, installs workspace dependencies, and verifies
that rosdep can resolve the completed workspace.

Options:
  --workspace <path>  Workspace containing src/ (auto-detected by default)
  --repo-only         Resolve dependencies from this repository only
  --help              Show this help

Before running, source either /opt/ros/humble/setup.bash on Ubuntu 22.04 or
/opt/ros/jazzy/setup.bash on Ubuntu 24.04. APT operations may ask for sudo.
EOF
}

fail() {
  echo "error: $*" >&2
  echo "hint: run this helper with --help for supported usage." >&2
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" || "${value}" == -* ]]; then
    fail "option requires a value: ${option}"
  fi
}

WORKSPACE_ROOT=""
REPO_ONLY=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      require_value "$1" "${2:-}"
      WORKSPACE_ROOT="$2"
      shift 2
      ;;
    --repo-only)
      REPO_ONLY=true
      shift
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      fail "unknown option: $1"
      ;;
  esac
done

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

if [[ -z "${WORKSPACE_ROOT}" ]]; then
  REPO_PARENT=$(dirname "${REPO_ROOT}")
  if [[ "$(basename "${REPO_PARENT}")" == "src" ]]; then
    WORKSPACE_ROOT=$(dirname "${REPO_PARENT}")
  elif [[ -d "${PWD}/src" ]]; then
    WORKSPACE_ROOT="${PWD}"
  else
    fail "could not find the workspace; pass --workspace <path>"
  fi
fi

[[ -d "${WORKSPACE_ROOT}" ]] || fail "workspace does not exist: ${WORKSPACE_ROOT}"
WORKSPACE_ROOT=$(cd "${WORKSPACE_ROOT}" && pwd)
[[ -d "${WORKSPACE_ROOT}/src" ]] || fail "workspace has no src directory: ${WORKSPACE_ROOT}"

DEPENDENCY_ROOT="${WORKSPACE_ROOT}/src"
if [[ "${REPO_ONLY}" == true ]]; then
  DEPENDENCY_ROOT="${REPO_ROOT}"
fi

MISSING_SOURCE_FILES=()
for required_source_file in \
  Thirdparty/ndt_omp_ros2/package.xml \
  Thirdparty/rko_lio/package.xml; do
  if [[ ! -f "${REPO_ROOT}/${required_source_file}" ]]; then
    MISSING_SOURCE_FILES+=("${required_source_file}")
  fi
done
if [[ ${#MISSING_SOURCE_FILES[@]} -gt 0 ]]; then
  echo "error: required source submodules are missing:" >&2
  printf '  %s\n' "${MISSING_SOURCE_FILES[@]}" >&2
  printf 'hint: git -C %q submodule update --init --recursive\n' \
    "${REPO_ROOT}" >&2
  exit 2
fi

case "${ROS_DISTRO:-}" in
  humble|jazzy)
    ;;
  "")
    fail "ROS_DISTRO is unset; source the Humble or Jazzy setup.bash first"
    ;;
  *)
    fail "ROS_DISTRO=${ROS_DISTRO} is unsupported; expected humble or jazzy"
    ;;
esac

for command_name in rosdep apt-get; do
  command -v "${command_name}" >/dev/null 2>&1 ||
    fail "required command is unavailable: ${command_name}"
done

ROOT_PREFIX=()
if [[ "${EUID}" -ne 0 ]]; then
  command -v sudo >/dev/null 2>&1 ||
    fail "sudo is required for APT operations when not running as root"
  ROOT_PREFIX=(sudo)
fi

ROSDEP_SOURCES="${LIDARSLAM_ROSDEP_SOURCES_FILE:-/etc/ros/rosdep/sources.list.d/20-default.list}"
if [[ -n "${ROS_HOME:-}" ]]; then
  ROS_HOME_DIR="${ROS_HOME}"
elif [[ -n "${HOME:-}" ]]; then
  ROS_HOME_DIR="${HOME}/.ros"
else
  fail "HOME and ROS_HOME are unset; cannot locate the rosdep cache"
fi
ROSDEP_CACHE="${ROS_HOME_DIR}/rosdep/sources.cache"

if [[ ! -f "${ROSDEP_SOURCES}" ]]; then
  echo "==> Initializing rosdep"
  "${ROOT_PREFIX[@]}" rosdep init
fi

if [[ ! -f "${ROSDEP_CACHE}/index" ]]; then
  echo "==> Updating rosdep metadata for ${ROS_DISTRO}"
  rosdep update --rosdistro "${ROS_DISTRO}"
fi

echo "==> Updating the APT package index"
"${ROOT_PREFIX[@]}" env DEBIAN_FRONTEND=noninteractive apt-get update

echo "==> Installing source dependencies for ${ROS_DISTRO}"
rosdep install \
  --from-paths "${DEPENDENCY_ROOT}" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO}" \
  -r -y

echo "==> Verifying source dependencies"
rosdep check \
  --from-paths "${DEPENDENCY_ROOT}" \
  --ignore-src \
  --rosdistro "${ROS_DISTRO}"

echo "Source dependencies are ready for ROS 2 ${ROS_DISTRO}."
