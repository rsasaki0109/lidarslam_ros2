#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  echo "error: execute this quickstart with bash; do not source it." >&2
  return 2
fi

set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  bash scripts/source_quickstart.sh [options]

From a cloned lidar_slam_ros2 repository, prepare only this repository's ROS 2
packages, build them, and run the fixed verified first-map demo. The command may
ask for sudo when system packages are missing.

Options:
  --workspace <path>       Workspace used for build/, install/, log/, and demo
                           output (auto-detected when the repo is under src/)
  --ros-distro <name>      humble or jazzy (auto-detected by default)
  --demo-dir <path>        Demo data/output root (default: workspace)
  --viewer <mode>          browser or none (default: browser)
  --build-only             Prepare and build without downloading/running demo
  --dry-run                Print the exact plan without network, APT, or writes
  --json                   With --dry-run, print the plan as JSON
  --help                   Show this help

Supported hosts are Ubuntu 22.04 with ROS 2 Humble and Ubuntu 24.04 with ROS 2
Jazzy. ROS itself must already be installed under /opt/ros; this helper does not
curl remote scripts or install a ROS distribution.
EOF
}

fail() {
  echo "error: $*" >&2
  echo "hint: run this quickstart with --help for supported usage." >&2
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
REQUESTED_ROS_DISTRO=""
DEMO_ROOT=""
VIEWER="browser"
BUILD_ONLY=false
DRY_RUN=false
JSON_OUTPUT=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      require_value "$1" "${2:-}"
      WORKSPACE_ROOT="$2"
      shift 2
      ;;
    --ros-distro)
      require_value "$1" "${2:-}"
      REQUESTED_ROS_DISTRO="$2"
      shift 2
      ;;
    --demo-dir)
      require_value "$1" "${2:-}"
      DEMO_ROOT="$2"
      shift 2
      ;;
    --viewer)
      require_value "$1" "${2:-}"
      VIEWER="$2"
      shift 2
      ;;
    --build-only)
      BUILD_ONLY=true
      shift
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --json)
      JSON_OUTPUT=true
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

if [[ "${JSON_OUTPUT}" == true && "${DRY_RUN}" != true ]]; then
  fail "--json requires --dry-run; run source_quickstart.sh --dry-run --json"
fi

case "${VIEWER}" in
  browser|none) ;;
  *) fail "--viewer must be browser or none" ;;
esac
case "${REQUESTED_ROS_DISTRO}" in
  ""|humble|jazzy) ;;
  *) fail "--ros-distro must be humble or jazzy" ;;
esac

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
DEPENDENCY_HELPER="${SCRIPT_DIR}/install_source_dependencies.sh"
EXPECTED_SOURCE_PACKAGES=(
  graph_based_slam
  lidarslam
  lidarslam_msgs
  ndt_omp_ros2
  rko_lio
  scanmatcher
)

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
[[ -f "${DEPENDENCY_HELPER}" ]] || fail "dependency helper is missing: ${DEPENDENCY_HELPER}"

if [[ -z "${DEMO_ROOT}" ]]; then
  DEMO_ROOT="${WORKSPACE_ROOT}"
elif [[ "${DEMO_ROOT}" != /* ]]; then
  DEMO_ROOT="${PWD}/${DEMO_ROOT}"
fi
DEMO_PARENT=$(dirname "${DEMO_ROOT}")
[[ -d "${DEMO_PARENT}" ]] || fail "demo parent directory does not exist: ${DEMO_PARENT}"
DEMO_ROOT=$(cd "${DEMO_PARENT}" && pwd)/$(basename "${DEMO_ROOT}")

SELECTED_ROS_DISTRO="${REQUESTED_ROS_DISTRO}"
if [[ -z "${SELECTED_ROS_DISTRO}" && -n "${ROS_DISTRO:-}" ]]; then
  case "${ROS_DISTRO}" in
    humble|jazzy) SELECTED_ROS_DISTRO="${ROS_DISTRO}" ;;
    *) fail "ROS_DISTRO=${ROS_DISTRO} is unsupported; expected humble or jazzy" ;;
  esac
fi

OS_RELEASE_FILE="${LIDARSLAM_OS_RELEASE_FILE:-/etc/os-release}"
if [[ -z "${SELECTED_ROS_DISTRO}" && -f "${OS_RELEASE_FILE}" ]]; then
  OS_VERSION_ID=$(awk -F= '$1 == "VERSION_ID" {print substr($0, index($0, "=") + 1); exit}' "${OS_RELEASE_FILE}")
  OS_VERSION_ID="${OS_VERSION_ID#\"}"
  OS_VERSION_ID="${OS_VERSION_ID%\"}"
  OS_VERSION_ID="${OS_VERSION_ID#\'}"
  OS_VERSION_ID="${OS_VERSION_ID%\'}"
  case "${OS_VERSION_ID}" in
    22.04) SELECTED_ROS_DISTRO="humble" ;;
    24.04) SELECTED_ROS_DISTRO="jazzy" ;;
  esac
fi
[[ -n "${SELECTED_ROS_DISTRO}" ]] ||
  fail "could not select ROS 2; pass --ros-distro humble or --ros-distro jazzy"
if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "${SELECTED_ROS_DISTRO}" ]]; then
  fail "the current ROS_DISTRO=${ROS_DISTRO} conflicts with ${SELECTED_ROS_DISTRO}; use a fresh terminal"
fi

ROS_PREFIX_ROOT="${LIDARSLAM_ROS_PREFIX_ROOT:-/opt/ros}"
ROS_SETUP="${ROS_PREFIX_ROOT}/${SELECTED_ROS_DISTRO}/setup.bash"
[[ -f "${ROS_SETUP}" ]] || {
  echo "error: ROS 2 ${SELECTED_ROS_DISTRO} is not installed at ${ROS_SETUP}" >&2
  echo "next: install ROS 2 ${SELECTED_ROS_DISTRO}, then re-run this command." >&2
  exit 2
}

REQUIRED_SUBMODULE_FILES=(
  Thirdparty/ndt_omp_ros2/package.xml
  Thirdparty/rko_lio/package.xml
)
MISSING_SUBMODULE_FILES=()
for required_file in "${REQUIRED_SUBMODULE_FILES[@]}"; do
  if [[ ! -f "${REPO_ROOT}/${required_file}" ]]; then
    MISSING_SUBMODULE_FILES+=("${required_file}")
  fi
done

MISSING_TOOL_PACKAGES=()
command -v rosdep >/dev/null 2>&1 || MISSING_TOOL_PACKAGES+=(python3-rosdep)
command -v colcon >/dev/null 2>&1 || MISSING_TOOL_PACKAGES+=(python3-colcon-common-extensions)
APT_ROOT_PREFIX=()
if [[ "${EUID}" -ne 0 ]]; then
  APT_ROOT_PREFIX=(sudo)
fi

print_command() {
  printf '  '
  printf '%q ' "$@"
  printf '\n'
}

build_plan_commands() {
  PLAN_COMMAND_FRAMED=()
  local -a plan_command
  if [[ ${#MISSING_SUBMODULE_FILES[@]} -gt 0 ]]; then
    plan_command=(git -C "${REPO_ROOT}" submodule update --init --recursive)
    PLAN_COMMAND_FRAMED+=("${#plan_command[@]}" "${plan_command[@]}")
  fi
  if [[ ${#MISSING_TOOL_PACKAGES[@]} -gt 0 ]]; then
    plan_command=(
      "${APT_ROOT_PREFIX[@]}"
      env DEBIAN_FRONTEND=noninteractive apt-get update
    )
    PLAN_COMMAND_FRAMED+=("${#plan_command[@]}" "${plan_command[@]}")
    plan_command=(
      "${APT_ROOT_PREFIX[@]}"
      env DEBIAN_FRONTEND=noninteractive apt-get install -y
      "${MISSING_TOOL_PACKAGES[@]}"
    )
    PLAN_COMMAND_FRAMED+=("${#plan_command[@]}" "${plan_command[@]}")
  fi
  plan_command=(
    bash "${DEPENDENCY_HELPER}" --workspace "${WORKSPACE_ROOT}" --repo-only
  )
  PLAN_COMMAND_FRAMED+=("${#plan_command[@]}" "${plan_command[@]}")
  plan_command=(
    colcon build
    --base-paths "${REPO_ROOT}"
    --packages-select "${EXPECTED_SOURCE_PACKAGES[@]}"
    --symlink-install
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  )
  PLAN_COMMAND_FRAMED+=("${#plan_command[@]}" "${plan_command[@]}")
  if [[ "${BUILD_ONLY}" == false ]]; then
    plan_command=(lidarslam-map demo "${DEMO_ROOT}" --viewer "${VIEWER}")
    PLAN_COMMAND_FRAMED+=("${#plan_command[@]}" "${plan_command[@]}")
  fi
}

print_json_plan() {
  command -v python3 >/dev/null 2>&1 ||
    fail "--json requires python3 on the host"
  build_plan_commands
  python3 - \
    "${REPO_ROOT}" \
    "${WORKSPACE_ROOT}" \
    "${DEMO_ROOT}" \
    "${SELECTED_ROS_DISTRO}" \
    "${ROS_SETUP}" \
    "${VIEWER}" \
    "${BUILD_ONLY}" \
    "${#EXPECTED_SOURCE_PACKAGES[@]}" \
    "${EXPECTED_SOURCE_PACKAGES[@]}" \
    "${#MISSING_SUBMODULE_FILES[@]}" \
    "${MISSING_SUBMODULE_FILES[@]}" \
    "${#MISSING_TOOL_PACKAGES[@]}" \
    "${MISSING_TOOL_PACKAGES[@]}" \
    "${#PLAN_COMMAND_FRAMED[@]}" \
    "${PLAN_COMMAND_FRAMED[@]}" <<'PY'
import json
import sys


argv = sys.argv
repository = argv[1]
workspace = argv[2]
demo_root = argv[3]
ros_distro = argv[4]
ros_setup = argv[5]
viewer = argv[6]
build_only = argv[7] == 'true'
index = 8


def read_counted() -> list[str]:
    global index
    count = int(argv[index])
    index += 1
    values = argv[index:index + count]
    if len(values) != count:
        raise SystemExit('internal error: source plan argument framing mismatch')
    index += count
    return values


packages = read_counted()
missing_submodules = read_counted()
missing_tools = read_counted()
framed_commands = read_counted()
if index != len(argv):
    raise SystemExit('internal error: source plan argument framing mismatch')

commands = []
command_index = 0
while command_index < len(framed_commands):
    argument_count = int(framed_commands[command_index])
    command_index += 1
    command = framed_commands[command_index:command_index + argument_count]
    if len(command) != argument_count:
        raise SystemExit('internal error: source plan command framing mismatch')
    command_index += argument_count
    commands.append(command)
if command_index != len(framed_commands):
    raise SystemExit('internal error: source plan command framing mismatch')

plan = {
    'schema_version': 1,
    'schema_uri': (
        'https://rsasaki0109.github.io/lidar_slam_ros2/'
        'schemas/source-quickstart-plan-v1.schema.json'
    ),
    'plan_mode': 'dry_run',
    'status': 'ready',
    'ready': True,
    'repository': {
        'path': repository,
        'packages': packages,
        'package_count': len(packages),
    },
    'workspace': {
        'path': workspace,
        'build_path': f'{workspace}/build',
        'install_path': f'{workspace}/install',
        'log_path': f'{workspace}/log',
        'demo_root': demo_root,
    },
    'ros': {
        'distribution': ros_distro,
        'setup_path': ros_setup,
    },
    'options': {
        'build_only': build_only,
        'viewer': viewer,
    },
    'preflight': {
        'submodules': {
            'required': [
                'Thirdparty/ndt_omp_ros2/package.xml',
                'Thirdparty/rko_lio/package.xml',
            ],
            'missing': missing_submodules,
        },
        'tools': {
            'required': ['rosdep', 'colcon'],
            'missing': missing_tools,
        },
        'package_inventory': {
            'expected': packages,
            'verified_during_live_run': False,
        },
    },
    'planned_actions': {
        'initialize_submodules': bool(missing_submodules),
        'install_tools': bool(missing_tools),
        'install_repository_dependencies': True,
        'build_workspace': True,
        'run_fixed_demo': not build_only,
    },
    'commands': commands,
    'side_effects': {
        'network_accessed': False,
        'apt_executed': False,
        'submodule_checkout': False,
        'workspace_build_executed': False,
        'demo_executed': False,
        'filesystem_writes': False,
    },
}
print(json.dumps(plan, indent=2, sort_keys=True))
PY
}

if [[ "${JSON_OUTPUT}" != true ]]; then
  echo "Source quickstart plan"
  echo "  Repository: ${REPO_ROOT}"
  echo "  Workspace: ${WORKSPACE_ROOT}"
  echo "  ROS 2: ${SELECTED_ROS_DISTRO} (${ROS_SETUP})"
  echo "  Build scope: this repository only (${#EXPECTED_SOURCE_PACKAGES[@]} ROS packages)"
  echo "  Packages: ${EXPECTED_SOURCE_PACKAGES[*]}"
  if [[ "${BUILD_ONLY}" == true ]]; then
    echo "  Finish: build only"
  else
    echo "  Demo root: ${DEMO_ROOT}"
    echo "  Viewer: ${VIEWER}"
  fi
  echo "Stages"
  if [[ ${#MISSING_SUBMODULE_FILES[@]} -gt 0 ]]; then
    echo "  source: initialize pinned git submodules"
  else
    echo "  source: pinned submodules present"
  fi
  if [[ ${#MISSING_TOOL_PACKAGES[@]} -gt 0 ]]; then
    echo "  tools: install ${MISSING_TOOL_PACKAGES[*]}"
  else
    echo "  tools: rosdep and colcon present"
  fi
  echo "  dependencies: rosdep install/check for this repository"
  echo "  build: Release, symlink install, tests disabled"
  echo "  launcher: direct installed command auto-activates this workspace"
  if [[ "${BUILD_ONLY}" == false ]]; then
    echo "  demo: fixed public MID-360 bag -> verified Autoware map"
  fi
fi

if [[ "${DRY_RUN}" == true ]]; then
  if [[ "${JSON_OUTPUT}" == true ]]; then
    print_json_plan
  else
    echo "Commands (--dry-run; nothing executed)"
    if [[ ${#MISSING_SUBMODULE_FILES[@]} -gt 0 ]]; then
      print_command git -C "${REPO_ROOT}" submodule update --init --recursive
    fi
    if [[ ${#MISSING_TOOL_PACKAGES[@]} -gt 0 ]]; then
      print_command "${APT_ROOT_PREFIX[@]}" env DEBIAN_FRONTEND=noninteractive apt-get update
      print_command "${APT_ROOT_PREFIX[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y "${MISSING_TOOL_PACKAGES[@]}"
    fi
    print_command bash "${DEPENDENCY_HELPER}" --workspace "${WORKSPACE_ROOT}" --repo-only
    print_command colcon build --base-paths "${REPO_ROOT}" --packages-select "${EXPECTED_SOURCE_PACKAGES[@]}" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
    if [[ "${BUILD_ONLY}" == false ]]; then
      print_command lidarslam-map demo "${DEMO_ROOT}" --viewer "${VIEWER}"
    fi
  fi
  exit 0
fi

RETRY_OPTIONS=(--workspace "${WORKSPACE_ROOT}" --ros-distro "${SELECTED_ROS_DISTRO}")
if [[ "${BUILD_ONLY}" == true ]]; then
  RETRY_OPTIONS+=(--build-only)
else
  RETRY_OPTIONS+=(--demo-dir "${DEMO_ROOT}" --viewer "${VIEWER}")
fi
CURRENT_STAGE="preflight"
on_error() {
  local status=$?
  trap - ERR
  echo "error: source quickstart stopped during ${CURRENT_STAGE} (exit ${status})." >&2
  printf 'retry: bash %q ' "$0" >&2
  printf '%q ' "${RETRY_OPTIONS[@]}" >&2
  printf '\n' >&2
  exit "${status}"
}
trap on_error ERR

if [[ ${#MISSING_SUBMODULE_FILES[@]} -gt 0 ]]; then
  CURRENT_STAGE="source checkout"
  command -v git >/dev/null 2>&1 || fail "git is required to initialize source submodules"
  echo "==> Source checkout: initializing pinned submodules"
  git -C "${REPO_ROOT}" submodule update --init --recursive
  for required_file in "${REQUIRED_SUBMODULE_FILES[@]}"; do
    [[ -f "${REPO_ROOT}/${required_file}" ]] ||
      fail "required submodule file is still missing: ${required_file}"
  done
fi

# Source the base setup in this process only. The final message always prints
# the command needed by the user's current shell.
CURRENT_STAGE="ROS setup"
# ROS setup files from supported distributions may reference unset variables
# while they are being sourced.  Keep nounset for this helper and the user
# command, but do not let the base distribution setup turn a clean shell into
# an onboarding failure.
# shellcheck disable=SC1090
set +u
source "${ROS_SETUP}"
set -u
[[ "${ROS_DISTRO:-}" == "${SELECTED_ROS_DISTRO}" ]] ||
  fail "${ROS_SETUP} did not activate ROS_DISTRO=${SELECTED_ROS_DISTRO}"

if [[ ${#MISSING_TOOL_PACKAGES[@]} -gt 0 ]]; then
  CURRENT_STAGE="base tool installation"
  command -v apt-get >/dev/null 2>&1 || fail "apt-get is required to install build tools"
  if [[ "${EUID}" -ne 0 ]]; then
    command -v sudo >/dev/null 2>&1 || fail "sudo is required to install missing build tools"
  fi
  echo "==> Build tools: installing ${MISSING_TOOL_PACKAGES[*]}"
  "${APT_ROOT_PREFIX[@]}" env DEBIAN_FRONTEND=noninteractive apt-get update
  "${APT_ROOT_PREFIX[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install -y "${MISSING_TOOL_PACKAGES[@]}"
fi
for command_name in rosdep colcon; do
  command -v "${command_name}" >/dev/null 2>&1 ||
    fail "required command remains unavailable after bootstrap: ${command_name}"
done

CURRENT_STAGE="source package inventory"
if ! DISCOVERED_PACKAGE_TEXT=$(colcon list \
  --base-paths "${REPO_ROOT}" \
  --names-only); then
  fail "could not inspect the repository package inventory"
fi
mapfile -t DISCOVERED_SOURCE_PACKAGES < <(
  printf '%s\n' "${DISCOVERED_PACKAGE_TEXT}" |
    sed '/^[[:space:]]*$/d' |
    LC_ALL=C sort -u
)
PACKAGE_INVENTORY_MATCH=true
if [[ ${#DISCOVERED_SOURCE_PACKAGES[@]} -ne ${#EXPECTED_SOURCE_PACKAGES[@]} ]]; then
  PACKAGE_INVENTORY_MATCH=false
else
  for index in "${!EXPECTED_SOURCE_PACKAGES[@]}"; do
    if [[ "${DISCOVERED_SOURCE_PACKAGES[index]}" != "${EXPECTED_SOURCE_PACKAGES[index]}" ]]; then
      PACKAGE_INVENTORY_MATCH=false
      break
    fi
  done
fi
if [[ "${PACKAGE_INVENTORY_MATCH}" != true ]]; then
  echo "error: [source-package-inventory-mismatch] expected the maintained six-package source set." >&2
  echo "expected: ${EXPECTED_SOURCE_PACKAGES[*]}" >&2
  echo "found: ${DISCOVERED_SOURCE_PACKAGES[*]:-(none)}" >&2
  exit 2
fi
echo "==> Source packages: exact maintained inventory confirmed"

CURRENT_STAGE="dependency installation"
echo "==> Dependencies: resolving this repository only"
bash "${DEPENDENCY_HELPER}" --workspace "${WORKSPACE_ROOT}" --repo-only

CURRENT_STAGE="workspace build"
echo "==> Build: compiling the 6 repository packages"
(
  cd "${WORKSPACE_ROOT}"
  colcon build \
    --base-paths "${REPO_ROOT}" \
    --packages-select "${EXPECTED_SOURCE_PACKAGES[@]}" \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
)

INSTALL_SETUP="${WORKSPACE_ROOT}/install/setup.bash"
[[ -f "${INSTALL_SETUP}" ]] || fail "build finished without ${INSTALL_SETUP}"
# shellcheck disable=SC1090
set +u
source "${INSTALL_SETUP}"
set -u
PRODUCT_COMMAND_PATH=$(command -v lidarslam-map || true)
[[ -n "${PRODUCT_COMMAND_PATH}" ]] ||
  fail "build finished but lidarslam-map is not available after sourcing ${INSTALL_SETUP}"
case "${PRODUCT_COMMAND_PATH}" in
  "${WORKSPACE_ROOT}/install/"*) ;;
  *)
    fail "build activated an unexpected lidarslam-map outside this workspace: ${PRODUCT_COMMAND_PATH}"
    ;;
esac
[[ -x "${PRODUCT_COMMAND_PATH}" ]] ||
  fail "the built lidarslam-map is not executable: ${PRODUCT_COMMAND_PATH}"

if [[ "${BUILD_ONLY}" == false ]]; then
  CURRENT_STAGE="verified first-map demo"
  echo "==> Demo: building and verifying the fixed public MID-360 map"
  lidarslam-map demo "${DEMO_ROOT}" --viewer "${VIEWER}"
fi

trap - ERR
echo "Source quickstart: COMPLETE"
echo "Run this build from any directory in a new terminal (no activation step):"
print_command "${PRODUCT_COMMAND_PATH}"
echo "Optional: activate the short lidarslam-map command in the current terminal:"
print_command source "${INSTALL_SETUP}"
if [[ "${BUILD_ONLY}" == true ]]; then
  echo "Try the fixed public demo:"
  print_command "${PRODUCT_COMMAND_PATH}" demo "${DEMO_ROOT}" --viewer "${VIEWER}"
fi
echo "Map your own compatible bag:"
print_command "${PRODUCT_COMMAND_PATH}" start /path/to/rosbag2
