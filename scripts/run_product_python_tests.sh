#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/run_product_python_tests.sh [options] [-- <pytest args>]

Options:
  --suite <name>       Test all, graph_based_slam, or lidarslam (default: all)
  --ros-setup <path>   Source this ROS setup file before preflight and tests
  --help               Show this help

Examples:
  bash scripts/run_product_python_tests.sh
  bash scripts/run_product_python_tests.sh --suite lidarslam
  bash scripts/run_product_python_tests.sh --suite graph_based_slam -- -k map_bundle

The two package test directories intentionally run in separate pytest
processes. They contain one legacy duplicate module basename, while an
unscoped repository-root pytest also collects optional Thirdparty tests.
Both maintained suites include ROS bag fixtures and therefore require
rosbag2_py from a supported Humble or Jazzy environment.
EOF
}

fail() {
  echo "error: $*" >&2
  echo "hint: run 'bash scripts/run_product_python_tests.sh --help' for valid options." >&2
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" || "${value}" == -* ]]; then
    fail "option requires a value: ${option}"
  fi
}

source_setup() {
  local setup_path="$1"
  set +u
  # shellcheck source=/dev/null
  source "${setup_path}"
  set -u
}

SUITE="all"
ROS_SETUP=""
PYTEST_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --suite)
      require_value "$1" "${2:-}"
      SUITE="$2"
      shift 2
      ;;
    --ros-setup)
      require_value "$1" "${2:-}"
      ROS_SETUP="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      PYTEST_ARGS=("$@")
      break
      ;;
    *)
      fail "unknown option: $1"
      ;;
  esac
done

case "${SUITE}" in
  all|graph_based_slam|lidarslam)
    ;;
  *)
    fail "unsupported suite '${SUITE}'; expected all, graph_based_slam, or lidarslam"
    ;;
esac

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
cd "${REPO_ROOT}"

PYTHON_BIN="${PYTHON:-python3}"
if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
  fail "Python executable not found: ${PYTHON_BIN}"
fi
if ! "${PYTHON_BIN}" -c 'import pytest' >/dev/null 2>&1; then
  echo "error: pytest is unavailable to ${PYTHON_BIN}" >&2
  echo "hint: install it with '${PYTHON_BIN} -m pip install pytest'." >&2
  exit 2
fi

RUN_GRAPH=false
RUN_LIDARSLAM=false
case "${SUITE}" in
  all)
    RUN_GRAPH=true
    RUN_LIDARSLAM=true
    ;;
  graph_based_slam)
    RUN_GRAPH=true
    ;;
  lidarslam)
    RUN_LIDARSLAM=true
    ;;
esac

if [[ -n "${ROS_SETUP}" ]]; then
  if [[ ! -f "${ROS_SETUP}" ]]; then
    fail "ROS setup file does not exist: ${ROS_SETUP}"
  fi
  source_setup "${ROS_SETUP}"
fi

if [[ "${RUN_GRAPH}" == "true" || "${RUN_LIDARSLAM}" == "true" ]] &&
  ! "${PYTHON_BIN}" -c 'import rosbag2_py' >/dev/null 2>&1; then
  AUTO_ROS_SETUP=""
  if [[ -z "${ROS_SETUP}" &&
    ("${ROS_DISTRO:-}" == "humble" || "${ROS_DISTRO:-}" == "jazzy") &&
    -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    AUTO_ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [[ -z "${ROS_SETUP}" && -f /opt/ros/jazzy/setup.bash ]]; then
    AUTO_ROS_SETUP="/opt/ros/jazzy/setup.bash"
  elif [[ -z "${ROS_SETUP}" && -f /opt/ros/humble/setup.bash ]]; then
    AUTO_ROS_SETUP="/opt/ros/humble/setup.bash"
  fi
  if [[ -n "${AUTO_ROS_SETUP}" ]]; then
    echo "==> Sourcing ROS environment: ${AUTO_ROS_SETUP}"
    source_setup "${AUTO_ROS_SETUP}"
  fi
fi

if [[ "${RUN_GRAPH}" == "true" || "${RUN_LIDARSLAM}" == "true" ]] &&
  ! "${PYTHON_BIN}" -c 'import rosbag2_py' >/dev/null 2>&1; then
  echo "error: rosbag2_py is unavailable for the selected product suite: ${SUITE}" >&2
  echo "hint: source /opt/ros/humble/setup.bash or /opt/ros/jazzy/setup.bash," >&2
  echo "      or pass --ros-setup with a supported ROS setup file." >&2
  exit 2
fi

if [[ ("${RUN_GRAPH}" == "true" || "${RUN_LIDARSLAM}" == "true") &&
  -n "${ROS_DISTRO:-}" &&
  "${ROS_DISTRO}" != "humble" && "${ROS_DISTRO}" != "jazzy" ]]; then
  echo "error: ROS_DISTRO=${ROS_DISTRO} is outside the Humble/Jazzy support contract" >&2
  echo "hint: rerun after sourcing a supported ROS setup file, or pass --ros-setup." >&2
  exit 2
fi

REQUIRED_MODULES=()
if [[ "${RUN_GRAPH}" == "true" ]]; then
  REQUIRED_MODULES+=(numpy yaml jsonschema imageio PIL rosbags)
fi
if [[ "${RUN_LIDARSLAM}" == "true" ]]; then
  REQUIRED_MODULES+=(numpy yaml jsonschema scipy)
fi

declare -A CHECKED_MODULES=()
MISSING_MODULES=()
for module_name in "${REQUIRED_MODULES[@]}"; do
  if [[ -n "${CHECKED_MODULES[${module_name}]:-}" ]]; then
    continue
  fi
  CHECKED_MODULES["${module_name}"]=true
  if ! "${PYTHON_BIN}" -c "import ${module_name}" >/dev/null 2>&1; then
    MISSING_MODULES+=("${module_name}")
  fi
done

if [[ ${#MISSING_MODULES[@]} -gt 0 ]]; then
  echo "error: Python product-test prerequisites unavailable: ${MISSING_MODULES[*]}" >&2
  echo "hint: install the repository's declared ROS dependencies with rosdep." >&2
  if [[ " ${MISSING_MODULES[*]} " == *" rosbags "* ]]; then
    echo "hint: in the development Python environment, install the CI-pinned extra:" >&2
    echo "      ${PYTHON_BIN} -m pip install 'rosbags==0.11.0'" >&2
  fi
  exit 2
fi

FAILED_SUITES=()
run_suite() {
  local suite_name="$1"
  local suite_path="$2"
  echo "==> Running ${suite_name} Python product suite"
  if ! PYTHONDONTWRITEBYTECODE=1 "${PYTHON_BIN}" -m pytest -q \
    -p no:cacheprovider "${suite_path}" "${PYTEST_ARGS[@]}"; then
    FAILED_SUITES+=("${suite_name}")
  fi
}

if [[ "${RUN_GRAPH}" == "true" ]]; then
  run_suite "graph_based_slam" "graph_based_slam/test"
fi
if [[ "${RUN_LIDARSLAM}" == "true" ]]; then
  run_suite "lidarslam" "lidarslam/test"
fi

if [[ ${#FAILED_SUITES[@]} -gt 0 ]]; then
  echo "error: Python product suite failed: ${FAILED_SUITES[*]}" >&2
  exit 1
fi

echo "==> Python product suite PASS: ${SUITE}"
