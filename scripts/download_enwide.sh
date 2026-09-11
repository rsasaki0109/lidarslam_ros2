#!/usr/bin/env bash
# Download selected ENWIDE sequences from the official ETH Research Collection
# public share. Dataset license: CC BY 4.0.
#
# Usage:
#   bash scripts/download_enwide.sh --sequence tunnel_d --dest datasets/enwide
#   bash scripts/download_enwide.sh --sequence all --convert
#   bash scripts/download_enwide.sh --sequence tunnel_s --metadata-only
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

DEST_DIR="${REPO_ROOT}/datasets/enwide"
SEQUENCE="tunnel_d"
DO_CONVERT=false
DROP_BAG1=false
METADATA_ONLY=false

SHARE_TOKEN="TaWP9QcSnR2Pz9Z"
BASE_URL="https://libdrive.ethz.ch/public.php/webdav"

usage() {
  cat <<'EOF'
Usage: download_enwide.sh [options]

Options:
  --sequence tunnel_s|tunnel_d|all  Sequence to download (default: tunnel_d)
  --dest PATH                       Destination root (default: datasets/enwide)
  --convert                         Convert rosbag1 to rosbag2 with rosbags-convert
  --drop-bag1                       Remove rosbag1 after successful conversion
  --metadata-only                   Download metadata and ground truth only
  -h, --help                        Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sequence) SEQUENCE="$2"; shift 2 ;;
    --dest) DEST_DIR=$(realpath -m "$2"); shift 2 ;;
    --convert) DO_CONVERT=true; shift ;;
    --drop-bag1) DROP_BAG1=true; shift ;;
    --metadata-only) METADATA_ONLY=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "${SEQUENCE}" in
  tunnel_s) SEQUENCES=(tunnel_s) ;;
  tunnel_d) SEQUENCES=(tunnel_d) ;;
  all) SEQUENCES=(tunnel_s tunnel_d) ;;
  *)
    echo "unknown sequence: ${SEQUENCE} (known: tunnel_s, tunnel_d, all)" >&2
    exit 2
    ;;
esac

mkdir -p "${DEST_DIR}"

download_file() {
  local remote_path="$1"
  local output_path="$2"
  local expected_bytes="$3"

  mkdir -p "$(dirname "${output_path}")"
  local actual_bytes=0
  [[ ! -f "${output_path}" ]] || actual_bytes=$(stat -c '%s' "${output_path}")
  if [[ "${actual_bytes}" -ne "${expected_bytes}" ]]; then
    curl -fL --retry 8 --retry-all-errors --retry-delay 5 --continue-at - \
      -u "${SHARE_TOKEN}:" \
      -o "${output_path}" \
      "${BASE_URL}/${remote_path}"
  fi

  actual_bytes=$(stat -c '%s' "${output_path}")
  if [[ "${actual_bytes}" -ne "${expected_bytes}" ]]; then
    echo "incomplete file: ${output_path}" >&2
    echo "expected ${expected_bytes} bytes, got ${actual_bytes}" >&2
    exit 1
  fi
}

download_file "readme.md" "${DEST_DIR}/readme.md" 1745
download_file "os_enwide.json" "${DEST_DIR}/os_enwide.json" 10587
download_file "prism_imu_extrinsics.txt" \
  "${DEST_DIR}/prism_imu_extrinsics.txt" 161

for sequence in "${SEQUENCES[@]}"; do
  case "${sequence}" in
    tunnel_s)
      BAG_NAME="2023-08-08-17-12-37-tunnel_s.bag"
      EXPECTED_BAG_BYTES=14983936757
      EXPECTED_BAG_ETAG="672ca9aa6fa170b8c4498974cea6a561"
      EXPECTED_GT_BYTES=9408508
      EXPECTED_GT_ETAG="640b8aa9844eb5db1ebb396ca69144ed"
      ;;
    tunnel_d)
      BAG_NAME="2023-08-08-17-50-31-tunnel_d.bag"
      EXPECTED_BAG_BYTES=7485669675
      EXPECTED_BAG_ETAG="f6afd377894e90a85322e425172f7b89"
      EXPECTED_GT_BYTES=4678234
      EXPECTED_GT_ETAG="f012aef67efd0e14261342fac1ac233f"
      ;;
  esac

  SEQUENCE_DIR="${DEST_DIR}/${sequence}"
  BAG1="${SEQUENCE_DIR}/${BAG_NAME}"
  GT_FILE="${SEQUENCE_DIR}/gt-${sequence}.csv"
  ROS2_DIR="${SEQUENCE_DIR}/ros2"
  MANIFEST="${SEQUENCE_DIR}/input_manifest.json"

  download_file "${sequence}/gt-${sequence}.csv" \
    "${GT_FILE}" "${EXPECTED_GT_BYTES}"

  if [[ "${METADATA_ONLY}" != "true" ]]; then
    download_file "${sequence}/${BAG_NAME}" \
      "${BAG1}" "${EXPECTED_BAG_BYTES}"
  fi

  if [[ "${DO_CONVERT}" == "true" && "${METADATA_ONLY}" != "true" && \
        ! -e "${ROS2_DIR}/metadata.yaml" ]]; then
    command -v rosbags-convert >/dev/null 2>&1 || {
      echo "rosbags-convert not found (pip install rosbags)" >&2
      exit 1
    }
    CONVERT_PARENT=$(mktemp -d "${SEQUENCE_DIR}/.ros2-convert.XXXXXX")
    if rosbags-convert --src "${BAG1}" --dst "${CONVERT_PARENT}/ros2"; then
      [[ -f "${CONVERT_PARENT}/ros2/metadata.yaml" ]] || {
        echo "conversion completed without metadata.yaml: ${CONVERT_PARENT}" >&2
        exit 1
      }
      mv "${CONVERT_PARENT}/ros2" "${ROS2_DIR}"
      rmdir "${CONVERT_PARENT}"
    else
      echo "conversion failed; partial evidence kept at ${CONVERT_PARENT}" >&2
      exit 1
    fi
  fi

  SEQUENCE="${sequence}" \
  BAG_NAME="${BAG_NAME}" \
  EXPECTED_BAG_BYTES="${EXPECTED_BAG_BYTES}" \
  EXPECTED_BAG_ETAG="${EXPECTED_BAG_ETAG}" \
  EXPECTED_GT_BYTES="${EXPECTED_GT_BYTES}" \
  EXPECTED_GT_ETAG="${EXPECTED_GT_ETAG}" \
  BAG1="${BAG1}" \
  GT_FILE="${GT_FILE}" \
  ROS2_DIR="${ROS2_DIR}" \
  METADATA_ONLY="${METADATA_ONLY}" \
  MANIFEST="${MANIFEST}" \
  python3 - <<'PY'
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    digest = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        digest.update(candidate.relative_to(path).as_posix().encode())
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


bag = Path(os.environ['BAG1'])
gt = Path(os.environ['GT_FILE'])
ros2 = Path(os.environ['ROS2_DIR'])
metadata_only = os.environ['METADATA_ONLY'] == 'true'
document = {
    'schema_version': 1,
    'dataset': 'ENWIDE',
    'sequence': os.environ['SEQUENCE'],
    'source': 'https://doi.org/10.3929/ethz-c-000787551',
    'license': 'CC-BY-4.0',
    'topics': {'points': '/ouster/points', 'imu': '/ouster/imu'},
    'ground_truth': {
        'path': str(gt),
        'bytes': gt.stat().st_size,
        'sha256': sha256(gt),
        'official_etag': os.environ['EXPECTED_GT_ETAG'],
        'position_only': True,
    },
    'rosbag1': None,
    'rosbag2': None,
}
if not metadata_only:
    document['rosbag1'] = {
        'path': str(bag),
        'bytes': bag.stat().st_size,
        'sha256': sha256(bag),
        'official_etag': os.environ['EXPECTED_BAG_ETAG'],
    }
if (ros2 / 'metadata.yaml').is_file():
    document['rosbag2'] = {
        'path': str(ros2),
        'tree_sha256': sha256_tree(ros2),
        'converter': {
            'name': 'rosbags',
            'version': importlib.metadata.version('rosbags'),
        },
    }
manifest = Path(os.environ['MANIFEST'])
manifest.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
PY

  if [[ "${DROP_BAG1}" == "true" && -e "${ROS2_DIR}/metadata.yaml" ]]; then
    rm -f "${BAG1}"
  fi

  echo "ENWIDE ${sequence} ready"
  echo "  manifest: ${MANIFEST}"
  echo "  ground truth: ${GT_FILE}"
  [[ ! -e "${ROS2_DIR}/metadata.yaml" ]] || echo "  rosbag2: ${ROS2_DIR}"
done
