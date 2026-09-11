#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
STORAGE_HELPER="${SCRIPT_DIR}/attached_storage.py"

usage() {
  cat >&2 <<'EOF'
Usage:
  bash scripts/download_ntu_viral_tnp01.sh [options]

Options:
  --dest DIR         Destination root directory (default: ./demo_data/ntu_viral)
  --dest-device DEV  Mounted /dev path; use its ntu_viral subdirectory
  --dry-run          Print the local acquisition and disk-space plan; write nothing
  --keep-zip         Keep the downloaded zip file
  --no-convert       Skip rosbag1 -> rosbag2 conversion
  --no-restamp       Skip creation of the default RKO-LIO rosbag2
  -h, --help         Show this help

This script downloads the official NTU VIRAL tnp_01 sequence referenced from
the GLIM supplementary pages, extracts it, and optionally converts the ROS1 bag
to rosbag2 format using rosbags-convert. By default it also writes the
pointcloud+IMU rosbag2 expected by `run_autoware_quickstart.sh` and
`run_rko_lio_graph_benchmark.sh`.
EOF
}

fail() {
  echo "error: $*" >&2
  echo "hint: run 'bash scripts/download_ntu_viral_tnp01.sh --help' for valid options." >&2
  exit 2
}

die() {
  echo "error: $*" >&2
  exit 1
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

human_bytes() {
  awk -v bytes="$1" 'BEGIN { printf "%.1f GB", bytes / 1000000000 }'
}

shell_join() {
  local argument
  local joined=""
  local quoted
  for argument in "$@"; do
    printf -v quoted '%q' "${argument}"
    joined+="${joined:+ }${quoted}"
  done
  printf '%s' "${joined}"
}

filesystem_available_bytes() {
  local probe_path="$1"
  while [[ ! -e "${probe_path}" ]]; do
    local parent_path
    parent_path="$(dirname "${probe_path}")"
    [[ "${parent_path}" != "${probe_path}" ]] || break
    probe_path="${parent_path}"
  done
  df -PB1 -- "${probe_path}" | awk 'NR == 2 { print $4 }'
}

verify_archive_identity() {
  local archive_path="$1"
  local actual_size
  local actual_md5
  actual_size="$(stat -c '%s' -- "${archive_path}")"
  if [[ "${actual_size}" != "${EXPECTED_ARCHIVE_BYTES}" ]]; then
    die "official archive size mismatch: expected ${EXPECTED_ARCHIVE_BYTES} bytes, got ${actual_size}: ${archive_path}"
  fi
  actual_md5="$(md5sum -- "${archive_path}" | awk '{ print $1 }')"
  if [[ "${actual_md5}" != "${EXPECTED_ARCHIVE_MD5}" ]]; then
    die "official archive checksum mismatch: expected MD5 ${EXPECTED_ARCHIVE_MD5}, got ${actual_md5}: ${archive_path}"
  fi
  echo "archive identity: PASS (${actual_size} bytes, MD5 ${actual_md5})"
}

DEST_DIR="${REPO_ROOT}/demo_data/ntu_viral"
DEST_EXPLICIT="false"
DEST_DEVICE=""
DRY_RUN="false"
KEEP_ZIP="false"
DO_CONVERT="true"
DO_RESTAMP="true"
OPTION_VALUE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dest)
      require_value "$1" "${2:-}"
      if [[ -n "${DEST_DEVICE}" ]]; then
        fail "--dest and --dest-device are mutually exclusive"
      fi
      DEST_DIR="${OPTION_VALUE}"
      DEST_EXPLICIT="true"
      shift 2
      ;;
    --dest-device)
      require_value "$1" "${2:-}"
      if [[ "${DEST_EXPLICIT}" == "true" ]]; then
        fail "--dest and --dest-device are mutually exclusive"
      fi
      DEST_DEVICE="${OPTION_VALUE}"
      shift 2
      ;;
    --dry-run)
      DRY_RUN="true"; shift ;;
    --keep-zip)
      KEEP_ZIP="true"; shift ;;
    --no-convert)
      DO_CONVERT="false"; shift ;;
    --no-restamp)
      DO_RESTAMP="false"; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      fail "unknown option: $1" ;;
  esac
done

if [[ -n "${DEST_DEVICE}" ]]; then
  require_command python3 \
    "install python3, for example: sudo apt install python3"
  [[ -f "${STORAGE_HELPER}" ]] || fail \
    "attached-storage helper is missing: ${STORAGE_HELPER}"
  if ! DEST_MOUNTPOINT="$(
    python3 "${STORAGE_HELPER}" mountpoint --device "${DEST_DEVICE}"
  )"; then
    echo "hint: mount the device, then retry the same --dest-device command." >&2
    exit 2
  fi
  DEST_DIR="${DEST_MOUNTPOINT}/ntu_viral"
fi

DEST_DIR="$(realpath -m "${DEST_DIR}")"

SEQ_NAME="tnp_01"
ZIP_PATH="${DEST_DIR}/${SEQ_NAME}.zip"
EXTRACT_DIR="${DEST_DIR}/${SEQ_NAME}"
ROS2_DIR="${DEST_DIR}/${SEQ_NAME}_rosbag2"
RESTAMPED_DIR="${DEST_DIR}/${SEQ_NAME}_points_restamped_vn100_rosbag2"
URL="https://researchdata.ntu.edu.sg/api/access/datafile/98195"
# Exact public identity reported by the official DR-NTU file metadata API:
# https://researchdata.ntu.edu.sg/api/files/98195
EXPECTED_ARCHIVE_BYTES=8736253605
EXPECTED_ARCHIVE_MD5="82588ea4f29e311447f3d716865a022b"
# Conservative planning size for each extracted/converted bag generation. The
# maintained canonical rosbag2 is 11.3 GB; 12 GB leaves format overhead before
# the additional 10% filesystem reserve below.
EXPECTED_BAG_PHASE_BYTES=12000000000

if [[ -e "${DEST_DIR}" && ! -d "${DEST_DIR}" ]]; then
  fail "destination path is not a directory: ${DEST_DIR}"
fi

if [[ "${DO_CONVERT}" != "true" && "${DO_RESTAMP}" == "true" && ! -f "${ROS2_DIR}/metadata.yaml" ]]; then
  fail "--no-convert requires existing rosbag2 metadata when restamp is enabled: ${ROS2_DIR}/metadata.yaml"
fi

EXISTING_BAG_PATH="$(find "${EXTRACT_DIR}" -maxdepth 3 -name '*.bag' -print -quit 2>/dev/null || true)"
ADDITIONAL_BYTES=0
ARCHIVE_REQUIRED="false"
ARCHIVE_ACTION="not-needed"
EXTRACT_ACTION="reuse"
CONVERT_ACTION="skip"
RESTAMP_ACTION="skip"

if [[ -z "${EXISTING_BAG_PATH}" ]]; then
  ARCHIVE_REQUIRED="true"
  ARCHIVE_ACTION="reuse"
  if [[ ! -f "${ZIP_PATH}" ]]; then
    ARCHIVE_ACTION="download"
    ADDITIONAL_BYTES=$((ADDITIONAL_BYTES + EXPECTED_ARCHIVE_BYTES))
  fi
  EXTRACT_ACTION="extract"
  ADDITIONAL_BYTES=$((ADDITIONAL_BYTES + EXPECTED_BAG_PHASE_BYTES))
fi
if [[ "${DO_CONVERT}" == "true" ]]; then
  CONVERT_ACTION="reuse"
  if [[ ! -f "${ROS2_DIR}/metadata.yaml" ]]; then
    CONVERT_ACTION="convert"
    ADDITIONAL_BYTES=$((ADDITIONAL_BYTES + EXPECTED_BAG_PHASE_BYTES))
  fi
fi
if [[ "${DO_RESTAMP}" == "true" ]]; then
  RESTAMP_ACTION="reuse"
  if [[ ! -f "${RESTAMPED_DIR}/metadata.yaml" ]]; then
    RESTAMP_ACTION="restamp"
    ADDITIONAL_BYTES=$((ADDITIONAL_BYTES + EXPECTED_BAG_PHASE_BYTES))
  fi
fi

SPACE_RESERVE_BYTES=0
if (( ADDITIONAL_BYTES > 0 )); then
  SPACE_RESERVE_BYTES=$((ADDITIONAL_BYTES / 10))
  if (( SPACE_RESERVE_BYTES < 1000000000 )); then
    SPACE_RESERVE_BYTES=1000000000
  fi
fi
REQUIRED_BYTES=$((ADDITIONAL_BYTES + SPACE_RESERVE_BYTES))
AVAILABLE_BYTES="$(filesystem_available_bytes "${DEST_DIR}")"
if [[ ! "${AVAILABLE_BYTES}" =~ ^[0-9]+$ ]]; then
  die "failed to determine available disk space for: ${DEST_DIR}"
fi

echo "NTU VIRAL tnp_01 acquisition plan"
echo "source:      ${URL}"
echo "identity:    ${EXPECTED_ARCHIVE_BYTES} bytes, MD5 ${EXPECTED_ARCHIVE_MD5}"
echo "dest:        ${DEST_DIR}"
echo "archive:     ${ARCHIVE_ACTION}"
echo "rosbag1:     ${EXTRACT_ACTION}"
echo "rosbag2:     ${CONVERT_ACTION}"
echo "RKO-LIO bag: ${RESTAMP_ACTION}"
echo "space:       $(human_bytes "${REQUIRED_BYTES}") additional required (includes 10% reserve; ${REQUIRED_BYTES} bytes)"
echo "available:   $(human_bytes "${AVAILABLE_BYTES}") (${AVAILABLE_BYTES} bytes)"

if (( AVAILABLE_BYTES < REQUIRED_BYTES )); then
  SHORTFALL_BYTES=$((REQUIRED_BYTES - AVAILABLE_BYTES))
  STORAGE_CANDIDATE=()
  if command -v python3 >/dev/null 2>&1 && [[ -f "${STORAGE_HELPER}" ]]; then
    mapfile -t STORAGE_CANDIDATE < <(
      python3 "${STORAGE_HELPER}" discover \
        --minimum-bytes "${REQUIRED_BYTES}" --format lines 2>/dev/null || true
    )
  fi
  echo "status:      BLOCKED_INSUFFICIENT_SPACE"
  echo "shortfall:   $(human_bytes "${SHORTFALL_BYTES}") (${SHORTFALL_BYTES} bytes)"
  if [[ ${#STORAGE_CANDIDATE[@]} -ge 2 ]]; then
    CANDIDATE_DEVICE="${STORAGE_CANDIDATE[0]}"
    CANDIDATE_SUMMARY="${STORAGE_CANDIDATE[1]}"
    RECOVERY_COMMAND=(
      bash scripts/download_ntu_viral_tnp01.sh
      --dest-device "${CANDIDATE_DEVICE}"
    )
    if [[ "${KEEP_ZIP}" == "true" ]]; then
      RECOVERY_COMMAND+=(--keep-zip)
    fi
    if [[ "${DO_CONVERT}" != "true" ]]; then
      RECOVERY_COMMAND+=(--no-convert)
    fi
    if [[ "${DO_RESTAMP}" != "true" ]]; then
      RECOVERY_COMMAND+=(--no-restamp)
    fi
    PREFLIGHT_COMMAND=("${RECOVERY_COMMAND[@]}" --dry-run)
    echo "attached:    ${CANDIDATE_SUMMARY}; unmounted, free space unverified"
    if command -v udisksctl >/dev/null 2>&1; then
      MOUNT_COMMAND=(udisksctl mount -b "${CANDIDATE_DEVICE}")
      echo "mount:       $(shell_join "${MOUNT_COMMAND[@]}")"
      echo "next:        $(shell_join "${MOUNT_COMMAND[@]}")"
    else
      echo "next:        mount attached filesystem ${CANDIDATE_DEVICE}"
    fi
    echo "preflight:   $(shell_join "${PREFLIGHT_COMMAND[@]}")"
    echo "after READY: $(shell_join "${RECOVERY_COMMAND[@]}")"
  else
    echo "next:        choose a filesystem with at least $(human_bytes "${REQUIRED_BYTES}") free and pass it with --dest DIR"
  fi
  if [[ "${DRY_RUN}" == "true" ]]; then
    echo "dry-run:     no files, network requests, conversions, or downloads were started"
    exit 0
  fi
  echo "error: insufficient free space for the planned NTU VIRAL acquisition" >&2
  echo "hint: rerun with --dry-run or choose a larger external filesystem with --dest DIR" >&2
  exit 2
fi

echo "status:      READY"
if [[ "${DRY_RUN}" == "true" ]]; then
  echo "dry-run:     no files, network requests, conversions, or downloads were started"
  exit 0
fi

if [[ "${ARCHIVE_ACTION}" == "download" ]]; then
  require_command wget "install wget, for example: sudo apt install wget"
fi
ARCHIVE_VERIFIED="false"
if [[ "${ARCHIVE_REQUIRED}" == "true" ]]; then
  require_command md5sum "install GNU coreutils, for example: sudo apt install coreutils"
  if [[ -f "${ZIP_PATH}" ]]; then
    verify_archive_identity "${ZIP_PATH}"
    ARCHIVE_VERIFIED="true"
  fi
  require_command unzip "install unzip, for example: sudo apt install unzip"
fi
if [[ "${CONVERT_ACTION}" == "convert" ]]; then
  require_command rosbags-convert "install rosbags, for example: python3 -m pip install rosbags"
fi
if [[ "${RESTAMP_ACTION}" == "restamp" ]]; then
  require_command python3 "install python3, for example: sudo apt install python3"
fi

mkdir -p "${DEST_DIR}" || die "failed to create destination directory: ${DEST_DIR}"

echo "sequence:   ${SEQ_NAME}"
echo "source:     ${URL}"
echo "dest:       ${DEST_DIR}"
echo "extract:    ${EXTRACT_DIR}"
echo "rosbag2:    ${ROS2_DIR}"
echo "restamped:  ${RESTAMPED_DIR}"

if [[ "${ARCHIVE_REQUIRED}" == "true" ]]; then
  if [[ ! -f "${ZIP_PATH}" ]]; then
    echo "downloading zip..."
    if ! wget -c -O "${ZIP_PATH}" "${URL}"; then
      die "download failed from ${URL}"
    fi
  else
    echo "zip already exists: ${ZIP_PATH}"
  fi

  if [[ "${ARCHIVE_VERIFIED}" != "true" ]]; then
    verify_archive_identity "${ZIP_PATH}"
  fi

  mkdir -p "${EXTRACT_DIR}"
  echo "extracting zip..."
  if ! unzip -q -o "${ZIP_PATH}" -d "${EXTRACT_DIR}"; then
    die "failed to extract zip: ${ZIP_PATH}"
  fi
else
  echo "bag already extracted under: ${EXTRACT_DIR}"
fi

BAG_PATH="$(find "${EXTRACT_DIR}" -maxdepth 3 -name '*.bag' | head -n 1 || true)"
[[ -n "${BAG_PATH}" ]] || die "failed to locate extracted .bag file under ${EXTRACT_DIR}"

echo "rosbag1:    ${BAG_PATH}"
echo "topics:"
echo "  points: /os1_cloud_node1/points"
echo "  imu:    /imu/imu"

if [[ "${DO_CONVERT}" == "true" ]]; then
  if [[ ! -e "${ROS2_DIR}/metadata.yaml" ]]; then
    echo "converting rosbag1 -> rosbag2..."
    rm -rf "${ROS2_DIR}"
    if ! rosbags-convert --src "${BAG_PATH}" --dst "${ROS2_DIR}"; then
      die "rosbag1 to rosbag2 conversion failed"
    fi
  else
    echo "rosbag2 already exists: ${ROS2_DIR}"
  fi
  echo "rosbag2 dir: ${ROS2_DIR}"
fi

if [[ "${DO_RESTAMP}" == "true" ]]; then
  [[ -f "${ROS2_DIR}/metadata.yaml" ]] || die "restamp requires rosbag2 at ${ROS2_DIR}"
  if [[ ! -e "${RESTAMPED_DIR}/metadata.yaml" ]]; then
    echo "creating restamped rosbag2 for RKO-LIO..."
    if ! python3 "${SCRIPT_DIR}/restamp_rosbag2_topics.py" \
      --input "${ROS2_DIR}" \
      --output "${RESTAMPED_DIR}" \
      --topic /os1_cloud_node1/points \
      --copy-topic /imu/imu \
      --force; then
      die "failed to create restamped rosbag2: ${RESTAMPED_DIR}"
    fi
  else
    echo "restamped rosbag2 already exists: ${RESTAMPED_DIR}"
  fi
  echo "restamped rosbag2 dir: ${RESTAMPED_DIR}"
fi

if [[ "${ARCHIVE_REQUIRED}" == "true" && "${KEEP_ZIP}" != "true" ]]; then
  echo "removing zip..."
  rm -f "${ZIP_PATH}"
fi

echo "done"
