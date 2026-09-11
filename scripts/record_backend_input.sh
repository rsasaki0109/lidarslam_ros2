#!/usr/bin/env bash
# Record deterministic graph-backend inputs while running a frontend command.
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
SETUP_FILE="${REPO_ROOT}/../install/setup.bash"
if [[ ! -f "${SETUP_FILE}" ]]; then
  SETUP_FILE="${REPO_ROOT}/install/setup.bash"
fi
OUTPUT_DIR=""
FLUSH_TIMEOUT=60
COMMAND=()
RECORDER_PID=""

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/record_backend_input.sh --output-dir <dir> [options] -- <command...>

Options:
  --setup <setup.bash>       ROS/workspace setup override (active environment is accepted)
  --flush-timeout <seconds>  Recorder SIGINT flush timeout (default: 60)

The output directory must not already contain a rosbag. The wrapped command and
recorder share the current ROS_DOMAIN_ID. The capture is accepted only when
/rko_lio/odometry and /rko_lio/frame both contain messages.
EOF
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir) [[ $# -ge 2 ]] || usage; OUTPUT_DIR=$(realpath -m "$2"); shift 2 ;;
    --setup) [[ $# -ge 2 ]] || usage; SETUP_FILE=$(realpath -m "$2"); shift 2 ;;
    --flush-timeout) [[ $# -ge 2 ]] || usage; FLUSH_TIMEOUT="$2"; shift 2 ;;
    --) shift; COMMAND=("$@"); break ;;
    --help|-h) usage ;;
    *) echo "unknown option: $1" >&2; usage ;;
  esac
done

[[ -n "${OUTPUT_DIR}" && ${#COMMAND[@]} -gt 0 ]] || usage
[[ "${FLUSH_TIMEOUT}" =~ ^[1-9][0-9]*$ ]] || usage
if [[ ! -f "${SETUP_FILE}" ]]; then
  if [[ -n "${ROS_DISTRO:-}" ]] && command -v ros2 >/dev/null 2>&1; then
    SETUP_FILE=""
  else
    echo "setup file not found and no active ROS environment is available: ${SETUP_FILE}" >&2
    exit 2
  fi
fi
if [[ -e "${OUTPUT_DIR}" ]]; then
  echo "output already exists; refusing to mix captures: ${OUTPUT_DIR}" >&2
  exit 2
fi

if [[ -n "${SETUP_FILE}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${SETUP_FILE}"
  set -u
fi

stop_recorder() {
  [[ -n "${RECORDER_PID}" ]] || return 0
  if kill -0 "${RECORDER_PID}" 2>/dev/null; then
    kill -INT "${RECORDER_PID}" 2>/dev/null || true
    local deadline=$((SECONDS + FLUSH_TIMEOUT))
    while kill -0 "${RECORDER_PID}" 2>/dev/null && ((SECONDS < deadline)); do
      sleep 1
    done
    if kill -0 "${RECORDER_PID}" 2>/dev/null; then
      echo "recorder flush timed out; sending SIGKILL" >&2
      kill -KILL "${RECORDER_PID}" 2>/dev/null || true
    fi
  fi
  wait "${RECORDER_PID}" 2>/dev/null || true
  RECORDER_PID=""
}
trap stop_recorder EXIT INT TERM

mkdir -p "$(dirname "${OUTPUT_DIR}")"
RECORD_LOG="${OUTPUT_DIR}.record.log"
QOS_OVERRIDES="${REPO_ROOT}/configs/rosbag2/backend_capture_qos.yaml"
echo "recording: ${OUTPUT_DIR}"
printf 'command:'
printf ' %q' "${COMMAND[@]}"
printf '\n'
# Bash starts asynchronous commands with SIGINT ignored in non-interactive
# mode. GNU env restores the default dispositions immediately before exec.
/usr/bin/env --default-signal=INT --default-signal=TERM \
  ros2 bag record --storage mcap --storage-preset-profile fastwrite \
  --max-cache-size 1073741824 --include-unpublished-topics \
  --qos-profile-overrides-path "${QOS_OVERRIDES}" -o "${OUTPUT_DIR}" \
  /rko_lio/odometry /rko_lio/frame >"${RECORD_LOG}" 2>&1 &
RECORDER_PID=$!
sleep 2
if ! kill -0 "${RECORDER_PID}" 2>/dev/null; then
  echo "rosbag recorder exited during startup" >&2
  tail -80 "${RECORD_LOG}" >&2 || true
  exit 1
fi

set +e
"${COMMAND[@]}"
COMMAND_STATUS=$?
set -e
stop_recorder

if [[ ! -f "${OUTPUT_DIR}/metadata.yaml" ]]; then
  echo "metadata missing after flush; reindexing capture" >&2
  ros2 bag reindex "${OUTPUT_DIR}" -s mcap
fi
INFO_FILE="${OUTPUT_DIR}.info.txt"
ros2 bag info "${OUTPUT_DIR}" | tee "${INFO_FILE}"
python3 - "${OUTPUT_DIR}/metadata.yaml" <<'PY'
import sys
from pathlib import Path

import yaml

document = yaml.safe_load(Path(sys.argv[1]).read_text())
topics = document['rosbag2_bagfile_information']['topics_with_message_count']
counts = {row['topic_metadata']['name']: int(row['message_count']) for row in topics}
required = ('/rko_lio/odometry', '/rko_lio/frame')
missing = {name: counts.get(name, 0) for name in required if counts.get(name, 0) <= 0}
if missing:
    raise SystemExit(f'backend capture lacks required messages: {missing}')
print('backend input counts: ' + ', '.join(f'{name}={counts[name]}' for name in required))
PY

if ((COMMAND_STATUS != 0)); then
  echo "wrapped command failed with status ${COMMAND_STATUS}" >&2
  exit "${COMMAND_STATUS}"
fi
echo "backend input ready: ${OUTPUT_DIR}"
