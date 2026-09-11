#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  echo "error: execute this Docker launcher with bash; do not source it." >&2
  return 2
fi

set -Eeuo pipefail

LIDARSLAM_DOCKER_LAUNCHER_VERSION="development"
LIDARSLAM_DOCKER_LAUNCHER_REVISION="working-tree"

usage() {
  cat <<'EOF'
Usage:
  lidarslam-map-docker [options] ROSBAG2_DIR [-- START_OPTIONS...]

Repository checkout compatibility:
  bash scripts/docker_map_bag.sh [options] ROSBAG2_DIR [-- START_OPTIONS...]

Map one local rosbag2 directory with the published lidar_slam_ros2 image. The
bag is mounted read-only, the container runs as the current user, and the
high-level `lidarslam-map start` workflow detects, configures, maps, verifies,
and writes a reopenable session under one host output directory.

Options:
  --output-dir <path>    Empty host output directory
                         (default: ./lidarslam_output/<bag>-map)
  --ros-distro <name>    humble or jazzy (default: humble)
  --image <reference>    Override the corresponding published image reference
  --dry-run              Print the exact plan without Docker, network, or writes
  --json                 With --dry-run, print the plan as JSON
  --version              Print launcher version and source revision
  --help                 Show this help

Everything after `--` is passed to `lidarslam-map start`. The launcher owns
--output-dir, --map-output-dir, and --viewer so all writes stay in the mounted
host directory and the browser-independent session page remains accessible.

Examples:
  lidarslam-map-docker /absolute/path/to/rosbag2
  lidarslam-map-docker --ros-distro jazzy /absolute/path/to/rosbag2
  lidarslam-map-docker --dry-run /absolute/path/to/rosbag2
  lidarslam-map-docker --dry-run --json /absolute/path/to/rosbag2
  lidarslam-map-docker /absolute/path/to/rosbag2 -- --editable
  lidarslam-map-docker /absolute/path/to/rosbag2 -- \
    --lidar-to-base 0,0,0,1,0.10,0,0.20 \
    --imu-to-base 0,0,0,1,0,0,0

For non-interactive execution, review a dry-run first, then pass -- --yes plus
the required calibration confirmation or measured transforms. Published images
currently support Linux x86_64 hosts.
EOF
}

print_version() {
  printf 'lidarslam-map-docker %s (%s)\n' \
    "${LIDARSLAM_DOCKER_LAUNCHER_VERSION}" \
    "${LIDARSLAM_DOCKER_LAUNCHER_REVISION}"
}

die() {
  local status="$1"
  local code="$2"
  local message="$3"
  local next_action="$4"
  echo "error: [${code}] ${message}" >&2
  echo "Next: ${next_action}" >&2
  exit "${status}"
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" || "${value}" == -* ]]; then
    die 2 invalid-usage "option requires a value: ${option}" \
      "Run lidarslam-map-docker --help."
  fi
}

print_command() {
  printf '  '
  printf '%q ' "$@"
  printf '\n'
}

path_contains() {
  local parent="$1"
  local child="$2"
  [[ "${child}" == "${parent}" || "${child}" == "${parent}/"* ]]
}

ROS_DISTRO=humble
IMAGE=""
OUTPUT_INPUT=""
DRY_RUN=false
JSON_OUTPUT=false
BAG_INPUT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      require_value "$1" "${2:-}"
      OUTPUT_INPUT="$2"
      shift 2
      ;;
    --ros-distro)
      require_value "$1" "${2:-}"
      ROS_DISTRO="$2"
      shift 2
      ;;
    --image)
      require_value "$1" "${2:-}"
      IMAGE="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    --json)
      JSON_OUTPUT=true
      shift
      ;;
    --version)
      print_version
      exit 0
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      die 2 bag-required "ROSBAG2_DIR must appear before --" \
        "Pass the directory containing metadata.yaml before start options."
      ;;
    -*)
      die 2 invalid-usage "unknown launcher option: $1" \
        "Put lidarslam-map start options after --."
      ;;
    *)
      BAG_INPUT="$1"
      shift
      break
      ;;
  esac
done

if [[ "${JSON_OUTPUT}" == true && "${DRY_RUN}" != true ]]; then
  die 2 json-requires-dry-run "--json requires --dry-run" \
    "Run lidarslam-map-docker --dry-run --json with the bag directory."
fi

[[ -n "${BAG_INPUT}" ]] ||
  die 2 bag-required "ROSBAG2_DIR is required" \
    "Pass the directory containing metadata.yaml."

START_ARGS=()
if [[ $# -gt 0 ]]; then
  [[ "$1" == "--" ]] ||
    die 2 invalid-usage "unexpected argument after ROSBAG2_DIR: $1" \
      "Put lidarslam-map start options after --."
  shift
  START_ARGS=("$@")
fi

case "${ROS_DISTRO}" in
  humble|jazzy) ;;
  *)
    die 2 unsupported-ros-distro \
      "--ros-distro must be humble or jazzy, not ${ROS_DISTRO}" \
      "Choose --ros-distro humble or --ros-distro jazzy."
    ;;
esac

if [[ -z "${IMAGE}" ]]; then
  IMAGE_TAG="${ROS_DISTRO}"
  if [[ "${LIDARSLAM_DOCKER_LAUNCHER_VERSION}" != "development" ]]; then
    IMAGE_TAG="${LIDARSLAM_DOCKER_LAUNCHER_VERSION}-${ROS_DISTRO}"
  fi
  IMAGE="ghcr.io/rsasaki0109/lidar_slam_ros2:${IMAGE_TAG}"
fi
if [[ ! "${IMAGE}" =~ ^[A-Za-z0-9][A-Za-z0-9._/@:-]*$ ]]; then
  die 2 unsafe-image-reference "image reference contains unsafe characters" \
    "Use a registry image tag or immutable digest without whitespace."
fi

[[ "$(uname -s)" == "Linux" ]] ||
  die 2 unsupported-host "the Docker own-bag launcher supports Linux hosts" \
    "Use the expanded Docker command in docs/getting-started.md on this host."
case "$(uname -m)" in
  x86_64|amd64) ;;
  *)
    die 2 unsupported-host-architecture \
      "published own-bag images currently target amd64" \
      "Use a supported x86_64 host or the documented Jetson evaluation path."
    ;;
esac

[[ -d "${BAG_INPUT}" ]] ||
  die 2 bag-not-found "rosbag2 directory does not exist: ${BAG_INPUT}" \
    "Pass the directory containing metadata.yaml, not a .db3 or .mcap file."
BAG_DIR=$(realpath -e -- "${BAG_INPUT}") ||
  die 2 bag-not-found "cannot resolve rosbag2 directory" \
    "Pass an existing local rosbag2 directory."
[[ -f "${BAG_DIR}/metadata.yaml" && ! -L "${BAG_DIR}/metadata.yaml" ]] ||
  die 2 metadata-missing "${BAG_DIR} does not contain metadata.yaml" \
    "Pass the whole rosbag2 directory with a regular metadata.yaml file."

case "${BAG_DIR}" in
  *$'\n'*|*,*)
    die 2 unsupported-path \
      "the rosbag2 path contains a newline or comma unsupported by Docker --mount" \
      "Move or link the bag through a path without commas or newlines."
    ;;
esac

if [[ -z "${OUTPUT_INPUT}" ]]; then
  BAG_STEM=$(basename "${BAG_DIR}")
  SAFE_STEM=$(printf '%s' "${BAG_STEM}" | LC_ALL=C tr -cs 'A-Za-z0-9._-' '_')
  [[ -n "${SAFE_STEM}" ]] || SAFE_STEM=rosbag2
  OUTPUT_INPUT="${PWD}/lidarslam_output/${SAFE_STEM}-map"
elif [[ "${OUTPUT_INPUT}" != /* ]]; then
  OUTPUT_INPUT="${PWD}/${OUTPUT_INPUT}"
fi
if [[ -L "${OUTPUT_INPUT}" ]]; then
  die 2 output-symlink "output directory must not be a symbolic link" \
    "Choose a new real directory."
fi
OUTPUT_DIR=$(realpath -m -- "${OUTPUT_INPUT}") ||
  die 2 output-path-invalid "cannot resolve output directory" \
    "Choose a new local --output-dir."

case "${OUTPUT_DIR}" in
  *$'\n'*|*,*)
    die 2 unsupported-path \
      "the output path contains a newline or comma unsupported by Docker --mount" \
      "Choose an output path without commas or newlines."
    ;;
esac
[[ "${OUTPUT_DIR}" != "/" && "${OUTPUT_DIR}" != "${HOME:-}" ]] ||
  die 2 unsafe-output "refusing broad output directory: ${OUTPUT_DIR}" \
    "Choose a new dedicated output directory."
if path_contains "${BAG_DIR}" "${OUTPUT_DIR}" ||
   path_contains "${OUTPUT_DIR}" "${BAG_DIR}"; then
  die 2 path-overlap "input and output directories must not overlap" \
    "Choose an output directory outside the read-only rosbag2 tree."
fi
if [[ -e "${OUTPUT_DIR}" && ! -d "${OUTPUT_DIR}" ]]; then
  die 2 output-not-directory "output path is not a directory" \
    "Choose a new directory path."
fi
if [[ -d "${OUTPUT_DIR}" ]]; then
  [[ -r "${OUTPUT_DIR}" && -w "${OUTPUT_DIR}" && -x "${OUTPUT_DIR}" ]] ||
    die 2 output-unusable "output directory is not readable and writable" \
      "Choose a writable new --output-dir."
  OUTPUT_ENTRY=$(find "${OUTPUT_DIR}" -mindepth 1 -print -quit) ||
    die 2 output-inspection-failed "cannot inspect output directory" \
      "Choose a readable new --output-dir."
  if [[ -n "${OUTPUT_ENTRY}" ]]; then
    die 2 output-not-empty "output directory is not empty: ${OUTPUT_DIR}" \
      "Choose a new --output-dir; existing map evidence is never overwritten."
  fi
  OUTPUT_STATUS=empty
else
  OUTPUT_STATUS=absent
fi

HAS_YES=false
for argument in "${START_ARGS[@]}"; do
  case "${argument}" in
    --yes) HAS_YES=true ;;
    -h|--help|--help-all|--dry-run|--json|--output-dir|--output-dir=*|--map-output-dir|--map-output-dir=*|--viewer|--viewer=*|--verification|--verification=*)
      die 2 managed-option \
        "${argument%%=*} is managed by the Docker launcher" \
        "Remove it and use the launcher's --output-dir before ROSBAG2_DIR."
      ;;
  esac
done

DOCKER_TTY_ARGS=()
INTERACTIVE=false
if [[ -t 0 && -t 1 ]]; then
  INTERACTIVE=true
  DOCKER_TTY_ARGS=(-it)
fi

build_docker_command() {
  local runtime_image="$1"
  DOCKER_COMMAND=(
    docker run --rm --pull=never --network none
    "${DOCKER_TTY_ARGS[@]}"
    --user "$(id -u):$(id -g)"
    -e HOME=/output/home
    -e ROS_LOG_DIR=/output/ros-logs
    --mount "type=bind,src=${BAG_DIR},dst=/input,readonly"
    --mount "type=bind,src=${OUTPUT_DIR},dst=/output"
    "${runtime_image}"
    lidarslam-map start /input
    --output-dir /output/setup
    --map-output-dir /output/map
    "${START_ARGS[@]}"
    --viewer none
  )
}

print_plan() {
  local runtime_image="$1"
  echo "Docker own-bag plan"
  echo "  Input:  ${BAG_DIR} (read-only)"
  echo "  Output: ${OUTPUT_DIR}"
  echo "  Image:  ${IMAGE}"
  if [[ "${runtime_image}" != "${IMAGE}" ]]; then
    echo "  Resolved image ID: ${runtime_image}"
  fi
  echo "  Route:  lidarslam-map start (detect -> configure -> map -> verify)"
  if [[ "${INTERACTIVE}" == true ]]; then
    echo "  Mode:   interactive calibration review"
  elif [[ "${HAS_YES}" == true ]]; then
    echo "  Mode:   reviewed non-interactive confirmation"
  else
    echo "  Mode:   interactive terminal required for a live run"
  fi
  echo "Command"
  print_command "${DOCKER_COMMAND[@]}"
}

print_json_plan() {
  local runtime_image="$1"
  command -v python3 >/dev/null 2>&1 ||
    die 70 json-unavailable \
      "--json requires python3 on the host" \
      "Install python3 or use --dry-run for the human-readable plan."
  python3 - \
    "${BAG_DIR}" \
    "${OUTPUT_DIR}" \
    "${IMAGE}" \
    "${ROS_DISTRO}" \
    "${OUTPUT_STATUS}" \
    "${INTERACTIVE}" \
    "${HAS_YES}" \
    "${runtime_image}" \
    "${LIDARSLAM_DOCKER_LAUNCHER_VERSION}" \
    "${LIDARSLAM_DOCKER_LAUNCHER_REVISION}" \
    "${#START_ARGS[@]}" \
    "${START_ARGS[@]}" \
    "${#DOCKER_COMMAND[@]}" \
    "${DOCKER_COMMAND[@]}" <<'PY'
import json
import sys


argv = sys.argv
bag_dir = argv[1]
output_dir = argv[2]
image = argv[3]
ros_distro = argv[4]
output_status = argv[5]
interactive = argv[6] == 'true'
has_yes = argv[7] == 'true'
runtime_image = argv[8]
launcher_version = argv[9]
launcher_revision = argv[10]
index = 11
start_count = int(argv[index])
index += 1
start_args = argv[index:index + start_count]
index += start_count
docker_count = int(argv[index])
index += 1
docker_command = argv[index:index + docker_count]
if len(docker_command) != docker_count or index + docker_count != len(argv):
    raise SystemExit('internal error: Docker plan argument framing mismatch')

if interactive:
    confirmation = 'interactive_terminal'
elif has_yes:
    confirmation = 'reviewed_yes'
else:
    confirmation = 'requires_interactive_or_yes'

plan = {
    'schema_version': 1,
    'schema_uri': (
        'https://rsasaki0109.github.io/lidar_slam_ros2/'
        'schemas/docker-map-bag-plan-v1.schema.json'
    ),
    'plan_mode': 'dry_run',
    'status': 'ready',
    'ready': True,
    'launcher': {
        'version': launcher_version,
        'revision': launcher_revision,
    },
    'input': {
        'path': bag_dir,
        'metadata': 'metadata.yaml',
        'mount': '/input',
        'read_only': True,
    },
    'output': {
        'path': output_dir,
        'mount': '/output',
        'status': output_status,
        'created': False,
        'must_be_empty': True,
    },
    'ros_distro': ros_distro,
    'image': {
        'reference': image,
        'resolved_id': None,
        'identity_bound': False,
        'resolution': 'deferred_until_live_run',
        'contract_preflight_performed': False,
    },
    'route': {
        'command': ['lidarslam-map', 'start', '/input'],
        'start_args': start_args,
        'viewer': 'none',
    },
    'execution': {
        'interactive': interactive,
        'confirmation': confirmation,
        'network_mode': 'none',
    },
    'docker_command': docker_command,
    'side_effects': {
        'docker_called': False,
        'network_accessed': False,
        'filesystem_writes': False,
        'input_mount_read_only': True,
    },
}
print(json.dumps(plan, indent=2, sort_keys=True))
PY
}

if [[ "${DRY_RUN}" == true ]]; then
  build_docker_command "${IMAGE}"
  if [[ "${JSON_OUTPUT}" == true ]]; then
    print_json_plan "${IMAGE}"
  else
    print_plan "${IMAGE}"
    echo "Dry run: complete; Docker, network, and filesystem writes were not used."
  fi
  exit 0
fi

if [[ "${INTERACTIVE}" != true && "${HAS_YES}" != true ]]; then
  die 2 confirmation-required \
    "a live run needs a terminal or an explicit reviewed --yes" \
    "Run --dry-run, then use an interactive terminal or append -- --yes."
fi
command -v docker >/dev/null 2>&1 ||
  die 70 docker-unavailable "docker is not installed or not on PATH" \
    "Install Docker, then rerun the same command."
if ! docker info >/dev/null 2>&1; then
  die 70 docker-unavailable "the local Docker daemon is unavailable" \
    "Start Docker and verify 'docker info', then rerun the same command."
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  if ! docker pull "${IMAGE}" >/dev/null; then
    die 70 image-pull-failed "the selected image is unavailable locally and could not be pulled" \
      "Check the image reference and registry access, then rerun."
  fi
fi
RUNTIME_IMAGE=$(docker image inspect --format '{{.Id}}' "${IMAGE}") ||
  die 70 image-identity-missing "Docker did not return the selected image ID" \
    "Inspect the image with docker image inspect, then rerun."
if [[ ! "${RUNTIME_IMAGE}" =~ ^sha256:[0-9a-f]{64}$ ]]; then
  die 70 image-identity-invalid "Docker returned an invalid image ID" \
    "Inspect the selected image and do not run an unbound candidate."
fi
set +e
docker run --rm --pull=never --network none "${RUNTIME_IMAGE}" \
  lidarslam-map start --help >/dev/null 2>&1
image_contract_status=$?
set -e
if [[ "${image_contract_status}" -ne 0 ]]; then
  if [[ "${image_contract_status}" -eq 2 ]]; then
    die 70 image-contract-missing \
      "the selected image does not expose lidarslam-map start" \
      "Pull a current image or use one built from this reviewed revision."
  fi
  die 70 image-preflight-failed \
    "the selected image could not complete its start-contract preflight" \
    "Inspect the selected image locally, then rerun."
fi
build_docker_command "${RUNTIME_IMAGE}"
print_plan "${RUNTIME_IMAGE}"

if ! mkdir -p -- "${OUTPUT_DIR}"; then
  die 70 output-create-failed "cannot create output directory: ${OUTPUT_DIR}" \
    "Choose a writable new --output-dir."
fi
if [[ -L "${OUTPUT_DIR}" || ! -d "${OUTPUT_DIR}" ]]; then
  die 70 output-race-detected \
    "output path changed while the image was being prepared" \
    "Choose a new --output-dir and rerun."
fi
CREATED_OUTPUT_DIR=$(realpath -e -- "${OUTPUT_DIR}") ||
  die 70 output-race-detected "cannot resolve the created output directory" \
    "Choose a new --output-dir and rerun."
if [[ "${CREATED_OUTPUT_DIR}" != "${OUTPUT_DIR}" ]]; then
  die 70 output-race-detected \
    "output path changed while the image was being prepared" \
    "Choose a new --output-dir and rerun."
fi
if [[ ! -r "${OUTPUT_DIR}" || ! -w "${OUTPUT_DIR}" || ! -x "${OUTPUT_DIR}" ]]; then
  die 70 output-unusable "created output directory is not readable and writable" \
    "Choose a writable new --output-dir."
fi
OUTPUT_ENTRY=$(find "${OUTPUT_DIR}" -mindepth 1 -print -quit) ||
  die 70 output-inspection-failed "cannot inspect the created output directory" \
    "Choose a readable new --output-dir."
if [[ -n "${OUTPUT_ENTRY}" ]]; then
  die 70 output-race-detected \
    "output directory became non-empty while the image was being prepared" \
    "Choose a new --output-dir; existing files were not overwritten."
fi

set +e
"${DOCKER_COMMAND[@]}"
status=$?
set -e
if [[ "${status}" -ne 0 ]]; then
  echo "error: [docker-map-failed] mapping exited with status ${status}" >&2
  echo "Output retained: ${OUTPUT_DIR}" >&2
  if [[ -f "${OUTPUT_DIR}/setup/session.html" && \
        ! -L "${OUTPUT_DIR}/setup/session.html" ]]; then
    echo "Next: review ${OUTPUT_DIR}/setup/session.html, then rerun with a new output directory." >&2
  else
    echo "Next: review the container diagnosis above, then rerun with a new output directory." >&2
  fi
  exit "${status}"
fi

REQUIRED_RESULTS=(
  "${OUTPUT_DIR}/setup/session.json"
  "${OUTPUT_DIR}/setup/session.html"
  "${OUTPUT_DIR}/map/run_manifest.json"
  "${OUTPUT_DIR}/map/first_map_validation_receipt.json"
)
MISSING_RESULTS=()
for result_path in "${REQUIRED_RESULTS[@]}"; do
  if [[ ! -f "${result_path}" || -L "${result_path}" ]]; then
    MISSING_RESULTS+=("${result_path#${OUTPUT_DIR}/}")
  fi
done
if [[ ${#MISSING_RESULTS[@]} -gt 0 ]]; then
  echo "error: [docker-result-missing] the container exited successfully without a complete verified session" >&2
  printf '  missing: %s\n' "${MISSING_RESULTS[@]}" >&2
  echo "Output retained: ${OUTPUT_DIR}" >&2
  echo "Next: review the container output above, then rerun with a new output directory." >&2
  exit 1
fi

echo "Docker own-bag map: COMPLETE"
echo "  Session page: ${OUTPUT_DIR}/setup/session.html"
echo "  Map output:   ${OUTPUT_DIR}/map"
