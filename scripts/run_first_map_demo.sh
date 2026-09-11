#!/usr/bin/env bash
# One-command first-map demo: download the fixed public Livox MID-360 driving
# bag (Zenodo 14841855, Koide, CC-BY 4.0) and run the installed product CLI
# with the maintained MID-360 profile. Docker and source workspaces use this
# same implementation and write the same versioned output contract.
#
# Run after sourcing a built source workspace, or as the default command of
# the ghcr.io/rsasaki0109/lidar_slam_ros2 image:
#
#   source install/setup.bash
#   bash scripts/run_first_map_demo.sh
#
# Environment overrides:
#   DEMO_DATA_DIR    dataset cache directory
#                    (default: <checkout-or-cwd>/datasets/mid360_public)
#   DEMO_OUTPUT_DIR  output directory
#                    (default: <checkout-or-cwd>/output/mid360_demo)
#   DEMO_RESUME      1 resumes safe terminal post-processing only (default: 0)
#   LIDARSLAM_HOST_UID/GID
#                    optional numeric owner for the output mount; set both
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_PARENT="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ -f "${SCRIPT_PARENT}/Dockerfile" && -f "${SCRIPT_PARENT}/lidarslam/package.xml" ]]; then
  WORK_ROOT="${SCRIPT_PARENT}"
else
  WORK_ROOT="${PWD}"
fi
DATA_DIR="${DEMO_DATA_DIR:-${WORK_ROOT}/datasets/mid360_public}"
OUT_DIR="${DEMO_OUTPUT_DIR:-${WORK_ROOT}/output/mid360_demo}"
BAG_NAME="rosbag2_2024_04_16-14_17_01"
DATASET_ID="driving_slam_mid360"
INTAKE_MANIFEST="${DATA_DIR}/${DATASET_ID}/mid360_robot_public_dataset_intake.json"
HOST_UID="${LIDARSLAM_HOST_UID:-}"
HOST_GID="${LIDARSLAM_HOST_GID:-}"
DEMO_RESUME="${DEMO_RESUME:-0}"

if [[ "${DEMO_RESUME}" != "0" && "${DEMO_RESUME}" != "1" ]]; then
  echo "error: DEMO_RESUME must be 0 or 1" >&2
  exit 2
fi

if [[ -n "${HOST_UID}" || -n "${HOST_GID}" ]]; then
  if [[ -z "${HOST_UID}" || -z "${HOST_GID}" ]]; then
    echo "error: set both LIDARSLAM_HOST_UID and LIDARSLAM_HOST_GID" >&2
    exit 2
  fi
  if [[ ! "${HOST_UID}" =~ ^[0-9]+$ || ! "${HOST_GID}" =~ ^[0-9]+$ ]]; then
    echo "error: LIDARSLAM_HOST_UID/GID must be numeric" >&2
    exit 2
  fi
fi

if ! command -v lidarslam-map >/dev/null 2>&1; then
  echo "error: lidarslam-map is not available in PATH" >&2
  echo "source the built workspace first: source install/setup.bash" >&2
  exit 127
fi

restore_output_ownership() {
  local run_status="$1"
  local output_name
  local output_root
  local target
  trap - EXIT

  if [[ -z "${HOST_UID}" ]]; then
    exit "${run_status}"
  fi

  output_root="$(dirname -- "${OUT_DIR}")"
  output_name="$(basename -- "${OUT_DIR}")"
  if [[ -z "${output_root}" || "${output_root}" == "." || "${output_root}" == "/" ]]; then
    echo "error: refusing ownership update for unsafe output root: ${output_root}" >&2
    exit 1
  fi

  if [[ "${EUID}" -eq 0 ]]; then
    if [[ -d "${output_root}" ]]; then
      chown "${HOST_UID}:${HOST_GID}" "${output_root}"
    fi
    for target in \
      "${OUT_DIR}" \
      "${OUT_DIR}.partial" \
      "${output_root}/.${output_name}.postprocess.lock"; do
      if [[ -e "${target}" ]]; then
        chown -R "${HOST_UID}:${HOST_GID}" "${target}"
      fi
    done
    echo "Output ownership: ${HOST_UID}:${HOST_GID}"
  elif [[ "$(id -u)" == "${HOST_UID}" && "$(id -g)" == "${HOST_GID}" ]]; then
    echo "Output ownership already matches ${HOST_UID}:${HOST_GID}"
  else
    echo "error: cannot set output ownership without container root privileges" >&2
    exit 1
  fi
  exit "${run_status}"
}

trap 'restore_output_ownership $?' EXIT

print_demo_artifacts() {
  echo
  echo "== first-map artifacts =="
  echo "outputs under ${OUT_DIR}:"
  echo "  map.pcd                           downsampled point-cloud map"
  echo "  pointcloud_map/                   Autoware map tiles (+ metadata)"
  echo "  map_projector_info.yaml           Autoware projector info (local)"
  echo "  traj_corrected.tum                loop-closed trajectory (TUM format)"
  echo "  run_manifest.json                 versioned execution manifest"
  echo "  verify_autoware_map.log           Autoware map verifier result"
  echo "  autoware_map_diagnosis.json       machine-readable diagnosis"
  echo "  autoware_map_diagnosis.md         operator diagnosis"
  echo "  first_map_validation_receipt.json privacy-bounded first-map receipt"
  echo "  first_map_validation_receipt.md   reviewable first-map receipt"
}

echo "== Demo stage 1/3: prepare and verify public data =="
echo "   (Koide, Zenodo DOI 10.5281/zenodo.14841855, CC-BY 4.0)"
echo "   verifying the registered archive and extracted bag identities..."
python3 "${SCRIPT_DIR}/download_mid360_robot_public_dataset.py" \
  --dataset "${DATASET_ID}" --dataset-root "${DATA_DIR}" --json >/dev/null
bag_dir="$(python3 - "${INTAKE_MANIFEST}" "${DATA_DIR}" "${BAG_NAME}" <<'PY'
import json
from pathlib import Path
import sys

manifest_path = Path(sys.argv[1])
data_dir = Path(sys.argv[2]).expanduser().resolve()
bag_name = sys.argv[3]
try:
    report = json.loads(manifest_path.read_text(encoding='utf-8'))
except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
    raise SystemExit(f'error: invalid public-dataset intake manifest: {exc}')

expected_bag = (
    data_dir / 'driving_slam_mid360' / 'extracted' / bag_name / bag_name
)
download = report.get('download') or {}
extraction = report.get('extraction') or {}
extracted_files = extraction.get('files') or []
archive_sha256 = (
    'f8f89eebf2aaf9cc1d465bfa5451bbb5'
    '99cd92d079b59949104bb4e5cb619bdd'
)
expected_files = {
    f'{bag_name}/metadata.yaml': (
        5590,
        '65d66875f49248e38ff14d80e6e749fb'
        '50606f6f80bd4be337160e3752691e9a',
    ),
    f'{bag_name}/{bag_name}_0.db3': (
        1468932096,
        '3bbd390a97e57af47ad6699baa36eb4c'
        '5f39f61b35275505ecaf221c126354f5',
    ),
}
actual_files = {item.get('path'): item for item in extracted_files}
valid = (
    report.get('status') == 'READY'
    and (report.get('dataset') or {}).get('id') == 'driving_slam_mid360'
    and download.get('status') == 'VERIFIED'
    and download.get('expected_size_bytes') == 517088133
    and download.get('size_bytes') == 517088133
    and download.get('expected_sha256') == archive_sha256
    and download.get('sha256') == archive_sha256
    and download.get('size_verified') is True
    and download.get('sha256_verified') is True
    and extraction.get('status') == 'VERIFIED'
    and set(actual_files) == set(expected_files)
    and all(
        actual_files[path].get('expected_size_bytes') == identity[0]
        and actual_files[path].get('size_bytes') == identity[0]
        and actual_files[path].get('expected_sha256') == identity[1]
        and actual_files[path].get('sha256') == identity[1]
        and actual_files[path].get('size_verified') is True
        and actual_files[path].get('sha256_verified') is True
        for path, identity in expected_files.items()
    )
    and Path(report.get('selected_bag_path') or '').resolve() == expected_bag
)
if not valid:
    raise SystemExit(
        'error: public demo intake did not produce the expected '
        'archive- and member-verified bag'
    )
print(expected_bag)
PY
)"
[[ -n "${bag_dir}" && -f "${bag_dir}/metadata.yaml" ]] || {
  echo "error: demo bag not found under ${DATA_DIR}" >&2
  exit 1
}
echo "   bag: ${bag_dir}"

if [[ "${DEMO_RESUME}" == "1" ]]; then
  echo "== Demo stage 2/3: resume terminal map verification and evidence =="
else
  echo "== Demo stage 2/3: build and verify the map (headless, offline) =="
fi
set +e
if [[ "${DEMO_RESUME}" == "1" ]]; then
  lidarslam-map run "${bag_dir}" \
    --profile rko_lio_graph_mid360_preset \
    --output-dir "${OUT_DIR}" \
    --resume
else
  lidarslam-map run "${bag_dir}" \
    --profile rko_lio_graph_mid360_preset \
    --output-dir "${OUT_DIR}"
fi
run_status=$?
set -e

if [[ "${run_status}" -ne 0 ]]; then
  echo "error: first-map run failed with exit code ${run_status}" >&2
  if [[ -d "${OUT_DIR}.partial" ]]; then
    echo "preserved failure output: ${OUT_DIR}.partial" >&2
  fi
  exit "${run_status}"
fi

echo "== Demo stage 3/3: verified output ready =="
print_demo_artifacts
