#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/run_default_ci_checks.sh [options]

Options:
  --build-only                 Build the default workflow packages without running tests
  --cmake-build-type <type>    CMake build type (default: Release)
  --help                       Show this help

This script verifies the default permissive-license workflow for this repository:
  - build: registration contracts/plugins, ndt_omp_ros2, lidarslam_msgs,
            scanmatcher, graph_based_slam, lidarslam, rko_lio
  - test:  registration contracts/plugins, lidarslam_msgs, scanmatcher,
            graph_based_slam, lidarslam
EOF
}

fail() {
  echo "error: $*" >&2
  echo "hint: run 'bash scripts/run_default_ci_checks.sh --help' for valid options." >&2
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" || "${value}" == -* ]]; then
    fail "option requires a value: ${option}"
  fi
  OPTION_VALUE="${value}"
}

require_command() {
  local command_name="$1"
  local install_hint="$2"
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "error: required command not found: ${command_name}" >&2
    echo "hint: ${install_hint}" >&2
    exit 2
  fi
}

BUILD_ONLY=false
CMAKE_BUILD_TYPE="Release"
OPTION_VALUE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-only)
      BUILD_ONLY=true
      shift
      ;;
    --cmake-build-type)
      require_value "$1" "${2:-}"
      CMAKE_BUILD_TYPE="${OPTION_VALUE}"
      shift 2
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
cd "${REPO_ROOT}"

require_command colcon "install colcon, for example: sudo apt install python3-colcon-common-extensions"

if ! command -v ros2 >/dev/null 2>&1; then
  for candidate in jazzy humble rolling iron; do
    if [[ -f "/opt/ros/${candidate}/setup.bash" ]]; then
      set +u
      # shellcheck source=/dev/null
      source "/opt/ros/${candidate}/setup.bash"
      set -u
      break
    fi
  done
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "error: required command not found: ros2" >&2
  echo "hint: source a ROS 2 setup file, for example: source /opt/ros/humble/setup.bash" >&2
  exit 2
fi

BUILD_TARGETS=(
  lidarslam_plugin_interfaces
  lidarslam_default_plugins
  lidarslam_fake_registration_plugins
  lidarslam_registration_loader
  scanmatcher
  lidarslam
  rko_lio
)

TEST_TARGETS=(
  lidarslam_plugin_interfaces
  lidarslam_default_plugins
  lidarslam_fake_registration_plugins
  lidarslam_registration_loader
  scanmatcher
  lidarslam_msgs
  graph_based_slam
  lidarslam
)

echo "==> Building default workflow packages"
echo "==> Build targets: ${BUILD_TARGETS[*]}"
if ! colcon build \
  --event-handlers console_direct+ \
  --packages-up-to "${BUILD_TARGETS[@]}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
    -DRKO_LIO_FETCH_CONTENT_DEPS=ON; then
  echo "error: colcon build failed for default workflow packages" >&2
  exit 1
fi

if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "${REPO_ROOT}/install/setup.bash"
  set -u
fi

echo "==> Validating clean-prefix product CLI install"
CLI_INSTALL_TEST_ROOT=$(mktemp -d)
if ! colcon build \
  --base-paths "${REPO_ROOT}/lidarslam" \
  --build-base "${CLI_INSTALL_TEST_ROOT}/build" \
  --install-base "${CLI_INSTALL_TEST_ROOT}/prefix" \
  --merge-install \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"; then
  echo "error: clean-prefix lidarslam install build failed" >&2
  exit 1
fi
if ! python3 "${REPO_ROOT}/scripts/check_installed_product_cli.py" \
  --prefix "${CLI_INSTALL_TEST_ROOT}/prefix" \
  --expected-source-revision "$(
    git -c "safe.directory=${REPO_ROOT}" \
      -C "${REPO_ROOT}" rev-parse HEAD
  )"; then
  echo "error: clean-prefix product CLI validation failed" >&2
  exit 1
fi
rm -rf -- "${CLI_INSTALL_TEST_ROOT}"

if [[ "${BUILD_ONLY}" == "true" ]]; then
  echo "==> Build-only mode completed"
  exit 0
fi

echo "==> Running default workflow tests"
echo "==> Test targets: ${TEST_TARGETS[*]}"
if ! colcon test \
  --event-handlers console_direct+ \
  --return-code-on-test-failure \
  --packages-select "${TEST_TARGETS[@]}"; then
  echo "error: colcon test failed for default workflow packages" >&2
  exit 1
fi

echo "==> Test results"
if ! colcon test-result --verbose; then
  echo "error: colcon test-result reported failures" >&2
  exit 1
fi
