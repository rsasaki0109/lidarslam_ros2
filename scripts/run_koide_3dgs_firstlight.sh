#!/usr/bin/env bash
# Reproduce the koide LiDAR+camera 3DGS first-light end-to-end:
#   rosbag2 -> lidarslam trajectory -> posed images -> gsplat .ply.
#
# Requires: built workspace (install/), a CUDA GPU with torch + gsplat, and the
# local demo_data/koide_lidar_camera_calib bag. See
# docs/research/3dgs-koide-first-light.md and
# docs/research/3dgs-postprocess-map-design.md.
#
# NOTE: the camera extrinsic used here is an *approximate* frame-convention
# transform (no calibrated lever arm); replace it with a
# direct_visual_lidar_calibration result for quality work.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

BAG="${BAG:-demo_data/koide_lidar_camera_calib/livox/rosbag2_2023_03_09-13_42_46}"
OUT_DIR="${OUT_DIR:-output/koide_3dgs_firstlight}"
EXTRINSIC="${EXTRINSIC:-configs/gaussian_splatting/koide_lidar_camera_extrinsic_approx.yaml}"
ITERS="${ITERS:-9000}"              # convergence sweet spot (~25.2dB; see notes)
NUM_INIT="${NUM_INIT:-60000}"
LIDAR_PRIMED="${LIDAR_PRIMED:-1}"   # 1 = seed Gaussians from the LiDAR cloud
DENSIFY="${DENSIFY:-1}"             # 1 = gsplat adaptive density control
SH_DEGREE="${SH_DEGREE:-1}"         # view-dependent colour (1 best for koide)
TUM="${OUT_DIR}/lidarslam/traj_map_livox_frame.tum"

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash

echo "== [1/3] lidarslam frontend -> trajectory =="
rm -rf "${OUT_DIR}/lidarslam"
bash scripts/compare_with_glim.sh \
  --bag "${BAG}" --skip-glim \
  --points-topic /livox/points --imu-topic /livox/imu --no-imu \
  --no-graph-based-slam --param lidarslam/param/lidarslam_mid360_noimu.yaml \
  --robot-frame-id livox_frame --base-frame livox_frame --lidar-frame livox_frame \
  --out-dir "${OUT_DIR}"
test -s "${TUM}" || { echo "ERROR: empty trajectory ${TUM}"; exit 1; }

echo "== [2/3] extract posed images (auto clock alignment) =="
python3 tools/colored_map/extract_posed_images.py \
  --bag "${BAG}" --traj "${TUM}" \
  --camera-topic /image --camera-info-topic /camera_info \
  --extrinsic "${EXTRINSIC}" \
  --time-offset auto --clock-reference-topic /livox/points \
  --max-extrapolation 0.4 \
  --out "${OUT_DIR}/gsplat"

INIT_ARGS=(--num-init "${NUM_INIT}")
if [[ "${LIDAR_PRIMED}" == "1" ]]; then
  echo "== [3/4] build LiDAR-primed init cloud =="
  python3 tools/colored_map/build_lidar_init.py \
    --bag "${BAG}" --traj "${TUM}" --points-topic /livox/points \
    --voxel 0.05 --max-points 200000 \
    --out "${OUT_DIR}/gsplat/lidar_init.ply"
  INIT_ARGS=(--init-ply "${OUT_DIR}/gsplat/lidar_init.ply")
fi

DENSIFY_ARGS=()
[[ "${DENSIFY}" == "1" ]] && DENSIFY_ARGS=(--densify)
[[ -n "${SH_DEGREE}" ]] && DENSIFY_ARGS+=(--sh-degree "${SH_DEGREE}")

echo "== [4/4] train gsplat -> .ply =="
python3 tools/gaussian_splatting/train_gsplat.py \
  --transforms "${OUT_DIR}/gsplat/transforms.json" \
  --out "${OUT_DIR}/gsplat/point_cloud.ply" \
  "${INIT_ARGS[@]}" "${DENSIFY_ARGS[@]}" --iters "${ITERS}"

echo "done: ${OUT_DIR}/gsplat/point_cloud.ply"
