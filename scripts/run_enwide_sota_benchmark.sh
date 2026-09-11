#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

SEQUENCE_DIR=""
OUTPUT_DIR=""
RUNS=3
MIN_MATCHED_FRACTION=0.98
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"

usage() {
  cat <<'EOF'
Usage: run_enwide_sota_benchmark.sh --sequence-dir PATH --output-dir PATH [options]

Options:
  --sequence-dir PATH  ENWIDE tunnel_s or tunnel_d directory
  --output-dir PATH    New directory for all repetitions and the summary
  --runs N             Repetitions (default: 3; official contract requires 3)
  -h, --help           Show this help

The dataset topics, sensor configuration, alignment, and scoring policy are
fixed by degenerate_lio_sota_v1. They are intentionally not CLI options.
EOF
}

require_value() {
  [[ $# -ge 2 && -n "$2" && "$2" != -* ]] || {
    echo "option requires a value: $1" >&2
    exit 2
  }
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sequence-dir)
      require_value "$1" "${2:-}"
      SEQUENCE_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --output-dir)
      require_value "$1" "${2:-}"
      OUTPUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --runs)
      require_value "$1" "${2:-}"
      RUNS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

[[ -n "${SEQUENCE_DIR}" && -n "${OUTPUT_DIR}" ]] || {
  usage >&2
  exit 2
}
[[ "${RUNS}" =~ ^[1-9][0-9]*$ ]] || {
  echo "--runs must be a positive integer" >&2
  exit 2
}
[[ ! -e "${OUTPUT_DIR}" ]] || {
  echo "output already exists: ${OUTPUT_DIR}" >&2
  exit 2
}

BAG="${SEQUENCE_DIR}/ros2"
MANIFEST="${SEQUENCE_DIR}/input_manifest.json"
SEQUENCE=$(basename "${SEQUENCE_DIR}")
GT="${SEQUENCE_DIR}/gt-${SEQUENCE}.csv"
PROFILE="${REPO_ROOT}/configs/slam_benchmark_profiles/degenerate_lio_sota_v1.yaml"
RKO_CONFIG="${REPO_ROOT}/configs/enwide/rko_lio_os0_degenerate_sota_v1.yaml"
REFERENCE_META="${REPO_ROOT}/configs/enwide/os_imu_to_prism.json"
GRAPH_CONFIG="${REPO_ROOT}/lidarslam/param/lidarslam.yaml"
EXPECTED_RKO_REVISION="add71ee46322a09139fa95e187d2816ed2c36295"

for required in \
  "${BAG}/metadata.yaml" "${MANIFEST}" "${GT}" "${PROFILE}" \
  "${RKO_CONFIG}" "${REFERENCE_META}" "${GRAPH_CONFIG}"; do
  [[ -f "${required}" ]] || {
    echo "required input not found: ${required}" >&2
    exit 2
  }
done
[[ "$(git -C "${REPO_ROOT}/Thirdparty/rko_lio" rev-parse HEAD)" == \
    "${EXPECTED_RKO_REVISION}" ]] || {
  echo "RKO-LIO revision does not match the preregistered baseline" >&2
  exit 2
}

python3 - "${MANIFEST}" "${BAG}" "${GT}" "${SEQUENCE}" <<'PY'
import hashlib
import json
from pathlib import Path
import sys


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path):
    digest = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        digest.update(candidate.relative_to(path).as_posix().encode())
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


manifest_path, bag, gt = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
sequence = sys.argv[4]
manifest = json.loads(manifest_path.read_text())
failures = []
if manifest.get('sequence') != sequence:
    failures.append('sequence')
if (manifest.get('ground_truth') or {}).get('sha256') != sha256(gt):
    failures.append('ground_truth_sha256')
if (manifest.get('rosbag2') or {}).get('tree_sha256') != sha256_tree(bag):
    failures.append('rosbag2_tree_sha256')
if failures:
    raise SystemExit('input manifest mismatch: ' + ', '.join(failures))
PY

mkdir -p "${OUTPUT_DIR}"
cp "${MANIFEST}" "${OUTPUT_DIR}/input_manifest.json"
cp "${PROFILE}" "${OUTPUT_DIR}/benchmark_profile.yaml"
cp "${RKO_CONFIG}" "${OUTPUT_DIR}/rko_config.yaml"
git -C "${REPO_ROOT}" rev-parse HEAD >"${OUTPUT_DIR}/lidarslam_revision.txt"
git -C "${REPO_ROOT}/Thirdparty/rko_lio" rev-parse HEAD \
  >"${OUTPUT_DIR}/rko_lio_revision.txt"
python3 "${SCRIPT_DIR}/capture_benchmark_machine_fingerprint.py" \
  --output "${OUTPUT_DIR}/machine.json"

for ((index = 1; index <= RUNS; ++index)); do
  run_dir=$(printf '%s/run_%02d' "${OUTPUT_DIR}" "${index}")
  run_name=$(printf 'enwide_%s_%02d' "${SEQUENCE}" "${index}")
  echo "ENWIDE ${SEQUENCE} repetition ${index}/${RUNS}"
  set +e
  /usr/bin/time -v -o "${OUTPUT_DIR}/time_${index}.txt" \
    bash "${SCRIPT_DIR}/run_rko_lio_graph_benchmark.sh" \
      --bag "${BAG}" \
      --reference-tum "${GT}" \
      --reference-meta "${REFERENCE_META}" \
      --lidar-topic /ouster/points \
      --imu-topic /ouster/imu \
      --base-frame os_imu \
      --lidarslam-param "${GRAPH_CONFIG}" \
      --rko-param "${RKO_CONFIG}" \
      --output-dir "${run_dir}" \
      --run-name "${run_name}" \
      --skip-reference-gen \
      --skip-map-save \
      --reference-source enwide_leica_ms60_position_gt \
      --completion-end-margin-secs 1.0 \
      --offline-timeout-secs 3600
  exit_status=$?
  set -e
  printf '%s\n' "${exit_status}" >"${run_dir}.exit_status"
  if [[ -s "${run_dir}/traj_raw_prism.tum" ]]; then
    if ! python3 "${SCRIPT_DIR}/score_position_only_trajectory.py" \
        --reference "${GT}" \
        --estimate "${run_dir}/traj_raw_prism.tum" \
        --output "${run_dir}/position_score.json" \
        --max-time-gap 0.11; then
      echo "warning: position-only scoring failed for repetition ${index}" >&2
    fi
  fi
done

python3 - \
  "${OUTPUT_DIR}" "${RUNS}" "${SEQUENCE}" "${ROS_DOMAIN_ID}" \
  "${MIN_MATCHED_FRACTION}" <<'PY'
import json
from pathlib import Path
import re
import statistics
import sys


output, requested, sequence = Path(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
ros_domain_id = int(sys.argv[4])
minimum_matched_fraction = float(sys.argv[5])
runs = []
for index in range(1, requested + 1):
    run_dir = output / f'run_{index:02d}'
    exit_status = int((output / f'run_{index:02d}.exit_status').read_text())
    score_path = run_dir / 'position_score.json'
    score = json.loads(score_path.read_text()) if score_path.is_file() else None
    metrics_path = run_dir / 'metrics.json'
    metrics = json.loads(metrics_path.read_text()) if metrics_path.is_file() else {}
    time_path = output / f'time_{index}.txt'
    time_text = time_path.read_text(errors='replace') if time_path.is_file() else ''
    match = re.search(r'Maximum resident set size \(kbytes\): (\d+)', time_text)
    matched_fraction = (
        (score.get('association') or {}).get('matched_ground_truth_fraction')
        if score is not None else None
    )
    runs.append({
        'run_index': index,
        'process_exit_status': exit_status,
        'trajectory_complete': (
            exit_status == 0
            and matched_fraction is not None
            and matched_fraction >= minimum_matched_fraction
        ),
        'runtime': {
            'processing_realtime_factor': (
                (metrics.get('lidarslam') or {}).get('rtf')
            ),
            'peak_rss_mb': int(match.group(1)) / 1024.0 if match else None,
        },
        'position_score': score,
    })
ates = [
    run['position_score']['trajectory']['ate_rmse_m']
    for run in runs if run['position_score'] is not None
]
rtes = [
    run['position_score']['trajectory']['rte_translation_percent_10m']
    for run in runs if run['position_score'] is not None
]
document = {
    'schema_version': 1,
    'profile': 'degenerate_lio_sota_v1',
    'sequence': sequence,
    'provenance': {
        'lidarslam_revision': (
            output / 'lidarslam_revision.txt'
        ).read_text().strip(),
        'rko_lio_revision': (
            output / 'rko_lio_revision.txt'
        ).read_text().strip(),
        'ros_domain_id': ros_domain_id,
    },
    'requested_repetitions': requested,
    'completed_repetitions': sum(run['trajectory_complete'] for run in runs),
    'minimum_matched_ground_truth_fraction': minimum_matched_fraction,
    'valid_for_sequence_comparison': (
        requested == 3
        and all(run['trajectory_complete'] for run in runs)
    ),
    'sota_claim_allowed': False,
    'aggregate': {
        'ate_rmse_median_m': statistics.median(ates) if ates else None,
        'rte_translation_percent_median': statistics.median(rtes) if rtes else None,
    },
    'runs': runs,
}
(output / 'summary.json').write_text(json.dumps(document, indent=2) + '\n')
print(json.dumps(document['aggregate'], indent=2))
raise SystemExit(0 if document['completed_repetitions'] == requested else 2)
PY
