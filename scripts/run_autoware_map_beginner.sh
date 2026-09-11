#!/usr/bin/env bash
set -euo pipefail

usage() {
  local exit_code="${1:-1}"
  cat <<'EOF' >&2
Usage:
  run_autoware_map_beginner.sh <rosbag2_dir> [options]

Inspect a rosbag2 directory, choose the shortest supported
Autoware-compatible map workflow, and write the result under output/ by default.

The input must be the rosbag2 directory that contains metadata.yaml.

Options:
  --preflight-only             Print the bag preflight report and exit
  --foxglove                    Open the saved map in the Foxglove path after the run
  --autoware                    Open the saved map in the Dockerized Autoware viewer after the run
  --no-viewer                   Do not open a viewer after the run (default)
  --dry-run                     Print the selected command without executing it
  --resume                      Resume terminal post-processing; never rerun SLAM
  --help                        Show this help

Common forwarded options:
  --output-dir <dir>            Write outputs to a specific directory
  --profile <id>                Force a compatible workflow profile
  --verification <mode>         Verification mode: required (default) or off
  --no-verify-map               Deprecated alias for --verification off
  --viewer-rebuild              Rebuild viewer runtime before opening
  --autoware-core-dir <dir>     autoware_core checkout for the Dockerized viewer
  --work-dir <dir>              Runtime workspace for Autoware/Foxglove viewers
  --viewer-run-dir <dir>        Existing built viewer runtime to reuse
  --auto-exit-secs <seconds>    Close the viewer after a timeout

Expected successful outputs:
  pointcloud_map/
  map_projector_info.yaml
  verify_autoware_map.log
  autoware_map_diagnosis.md
  run_manifest.json

Examples:
  bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2
  bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2 --preflight-only
  bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2 --foxglove
  bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2 --output-dir output/my_map
  bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2 --output-dir output/my_map --resume
EOF
  exit "$exit_code"
}

fail() {
  echo "error: $1" >&2
  if [[ $# -gt 1 ]]; then
    echo "hint: $2" >&2
  fi
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "$value" || "$value" == --* ]]; then
    fail "option requires a value: ${option}" \
      "run 'bash scripts/run_autoware_map_beginner.sh --help' for valid options."
  fi
}

set_viewer() {
  local selected="$1"
  local option="$2"
  if [[ "$VIEWER" != "none" && "$selected" != "none" && "$VIEWER" != "$selected" ]]; then
    fail "viewer already set to ${VIEWER}; cannot also use ${option}" \
      "choose one of --foxglove, --autoware, or --no-viewer."
  fi
  VIEWER="$selected"
}

if [[ $# -eq 1 && ( "$1" == "--help" || "$1" == "-h" ) ]]; then
  usage 0
fi

if [[ $# -lt 1 ]]; then
  fail "rosbag2_dir is required." \
    "pass the rosbag2 directory that contains metadata.yaml."
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
RUNNER="${SCRIPT_DIR}/run_autoware_map_from_bag.py"
PREFLIGHT="${SCRIPT_DIR}/preflight_autoware_map_bag.py"
BAG_PATH=""
VIEWER=none
PREFLIGHT_ONLY=false
FORWARDED_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --foxglove)
      set_viewer foxglove "$1"
      shift
      ;;
    --autoware)
      set_viewer autoware "$1"
      shift
      ;;
    --no-viewer)
      set_viewer none "$1"
      shift
      ;;
    --preflight-only)
      PREFLIGHT_ONLY=true
      shift
      ;;
    --help|-h)
      usage 0
      ;;
    --dry-run|--resume|--no-verify-map|--viewer-rebuild)
      FORWARDED_ARGS+=("$1")
      shift
      ;;
    --profile|--output-dir|--verification|--auto-exit-secs)
      require_value "$1" "${2:-}"
      FORWARDED_ARGS+=("$1" "$2")
      shift 2
      ;;
    --autoware-core-dir|--work-dir|--viewer-run-dir)
      require_value "$1" "${2:-}"
      FORWARDED_ARGS+=("$1" "$2")
      shift 2
      ;;
    -*)
      fail "unknown option: $1" \
        "run 'bash scripts/run_autoware_map_beginner.sh --help' for valid options."
      ;;
    *)
      if [[ -z "$BAG_PATH" ]]; then
        BAG_PATH="$1"
        shift
      else
        fail "unexpected positional argument: $1" \
          "pass exactly one rosbag2 directory."
      fi
      ;;
  esac
done

if [[ -z "$BAG_PATH" ]]; then
  fail "rosbag2_dir is required." \
    "pass the rosbag2 directory that contains metadata.yaml."
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "error: rosbag2_dir does not exist or is not a directory: $BAG_PATH" >&2
  echo "hint: pass the rosbag2 directory, not a .db3 file." >&2
  exit 2
fi

if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
  echo "error: metadata.yaml not found under: $BAG_PATH" >&2
  echo "hint: pass the rosbag2 directory that contains metadata.yaml." >&2
  exit 2
fi

if [[ "$PREFLIGHT_ONLY" == "true" ]]; then
  python3 "$PREFLIGHT" "$BAG_PATH"
  exit 0
fi

python3 "$RUNNER" "$BAG_PATH" --viewer "$VIEWER" "${FORWARDED_ARGS[@]}"
