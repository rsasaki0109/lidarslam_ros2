#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG=""
EXTRINSIC=""
OUTPUT_DIR=""
FRONTEND_RUNS=2
PARAMS="${REPO_ROOT}/lidarslam/param/lidarslam_lo.yaml"
LOCALIZATION_ZOO="${LOCALIZATION_ZOO:-}"

usage() {
  cat <<'EOF'
Usage: run_aist_ouster_rgb_map_benchmark.sh --bag DIR --extrinsic calib.json [options]

Builds a CPU-only real-RGB point-cloud map from an AIST Ouster calibration bag:
deterministic LiDAR-only odometry -> posed RGB images -> robust coloured map ->
colour, held-out RGB, map-geometry, runtime, and memory reports.

Options:
  --bag DIR              AIST rosbag2 containing /points, /image, /camera_info.
  --extrinsic FILE       Official vlcal calib.json (LiDAR <- camera).
  --output-dir DIR       Artifact root (default: output/aist_ouster_rgb_map_<time>).
  --frontend-runs N      Byte-determinism runs (default: 2).
  --params FILE          scanmatcher/graph parameter YAML.
  --localization-zoo DIR Localization Zoo checkout (auto-detected when nearby).
  --help                 Show this help.

The map intentionally uses --no-deskew: scanmatcher provides one registration
pose per complete scan, not a continuous-time body trajectory. Interpolating its
scan-to-scan corrections across Ouster's per-point t field regresses geometry.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; BAG="$2"; shift 2 ;;
    --extrinsic) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; EXTRINSIC="$2"; shift 2 ;;
    --output-dir) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; OUTPUT_DIR="$2"; shift 2 ;;
    --frontend-runs) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; FRONTEND_RUNS="$2"; shift 2 ;;
    --params) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; PARAMS="$2"; shift 2 ;;
    --localization-zoo) [[ $# -ge 2 ]] || { usage >&2; exit 2; }; LOCALIZATION_ZOO="$2"; shift 2 ;;
    --help|-h) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -d "${BAG}" && -f "${BAG}/metadata.yaml" ]] || {
  echo "--bag must be a rosbag2 directory with metadata.yaml" >&2; exit 2; }
[[ -f "${EXTRINSIC}" ]] || { echo "--extrinsic file not found" >&2; exit 2; }
[[ -f "${PARAMS}" ]] || { echo "--params file not found" >&2; exit 2; }
[[ "${FRONTEND_RUNS}" =~ ^[1-9][0-9]*$ ]] || {
  echo "--frontend-runs must be a positive integer" >&2; exit 2; }
if [[ -z "${LOCALIZATION_ZOO}" ]]; then
  for candidate in \
    "${REPO_ROOT}/../localization_zoo" \
    "${REPO_ROOT}/../../loc_zoo_ws/localization_zoo"; do
    if [[ -f "${candidate}/evaluation/scripts/evaluate_external_tum.py" ]]; then
      LOCALIZATION_ZOO="${candidate}"
      break
    fi
  done
fi
[[ -f "${LOCALIZATION_ZOO}/evaluation/scripts/evaluate_external_tum.py" ]] || {
  echo "Localization Zoo not found; pass --localization-zoo DIR" >&2; exit 2; }
[[ -f "${REPO_ROOT}/install/setup.bash" ]] || {
  echo "install/setup.bash not found; build the workspace first" >&2; exit 2; }
# shellcheck disable=SC1091
set +u
source "${REPO_ROOT}/install/setup.bash"
set -u
if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${REPO_ROOT}/output/aist_ouster_rgb_map_$(date +%Y%m%d_%H%M%S)"
fi
OUTPUT_DIR="$(realpath -m "${OUTPUT_DIR}")"
mkdir -p "${OUTPUT_DIR}"

FRONTEND_DIR="${OUTPUT_DIR}/frontend"
MAP_DIR="${OUTPUT_DIR}/colored_map"
TRAJECTORY="${FRONTEND_DIR}/run1/trajectory_frontend.tum"
mkdir -p "${MAP_DIR}"

echo "[1/6] deterministic scanmatcher trajectory"
bash "${SCRIPT_DIR}/run_frontend_determinism_check.sh" \
  --bag "${BAG}" --cloud-topic /points --params "${PARAMS}" \
  --runs "${FRONTEND_RUNS}" --output-dir "${FRONTEND_DIR}"

echo "[2/6] posed RGB images + rigid-scan coloured map"
/usr/bin/time -v -o "${MAP_DIR}/process_time.txt" \
  python3 "${REPO_ROOT}/tools/colored_map/colored_map_pipeline.py" \
  "${BAG}" "${TRAJECTORY}" "${MAP_DIR}" \
  --extrinsic "${EXTRINSIC}" \
  --points-topic /points --camera-topic /image --camera-info-topic /camera_info \
  --time-offset auto --time-offset-adjustment 0.01 \
  --image-stride 1 --scan-stride 1 \
  --voxel 0.15 --max-points 300000 \
  --min-neighbors 2 --sparse-voxel 0.20 --min-range 1.0 --max-range 80.0 \
  --no-deskew

python3 "${SCRIPT_DIR}/write_runtime_report.py" \
  --time-file "${MAP_DIR}/process_time.txt" \
  --bag-metadata "${BAG}/metadata.yaml" \
  --out "${MAP_DIR}/runtime.json"

echo "[3/6] real-RGB coverage and chroma"
python3 "${SCRIPT_DIR}/analyze_colored_point_cloud.py" \
  --input "${MAP_DIR}/colored_map.ply" \
  --output "${MAP_DIR}/colored_map_report.json"

echo "[4/6] held-out camera colour"
python3 "${SCRIPT_DIR}/evaluate_heldout_point_colors.py" \
  --pointcloud "${MAP_DIR}/colored_map.ply" \
  --transforms "${MAP_DIR}/posed_images/transforms.json" \
  --out "${MAP_DIR}/heldout_point_colors.json"

echo "[5/6] map geometry"
bash "${SCRIPT_DIR}/run_map_quality_check.sh" \
  --input "${MAP_DIR}/colored_map.ply" \
  --output-dir "${MAP_DIR}/map_quality" --runs 1 --downsample 0.2

echo "[6/6] shared public-suite evidence"
python3 "${SCRIPT_DIR}/run_cross_repo_slam_benchmark.py" \
  --localization-zoo "${LOCALIZATION_ZOO}" \
  --dataset aist_ouster_rgb \
  --geometry-report "${MAP_DIR}/map_quality/run1/map_quality_report.yaml" \
  --alignment-report "${MAP_DIR}/heldout_point_colors.json" \
  --colour-report "${MAP_DIR}/colored_map_report.json" \
  --runtime-report "${MAP_DIR}/runtime.json" \
  --raw-artifact "aist_rosbag=${BAG}" \
  --raw-artifact "official_extrinsic=${EXTRINSIC}" \
  --raw-artifact "frontend_trajectory=${TRAJECTORY}" \
  --raw-artifact "coloured_map=${MAP_DIR}/colored_map.ply" \
  --out-dir "${OUTPUT_DIR}/shared_evidence"

echo "AIST RGB map benchmark complete: ${OUTPUT_DIR}"
