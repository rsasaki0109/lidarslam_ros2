#!/usr/bin/env bash
# Reproduce the 3DGS "view-count lever" experiment on the Autoware Leo Drive
# isuzu bag (color cameras, ground-vehicle traverse, FILE-compressed/zstd):
#   rosbag2 -> lidarslam trajectory -> posed images -> LiDAR-primed gsplat .ply.
#
# This is a documented NEGATIVE result: 21x more views (640 vs koide's 30) does
# NOT beat koide's ~24dB -- capture consistency/sharpness dominates view count.
# See docs/research/3dgs-isuzu-viewcount-notes.md. Requires a CUDA GPU with torch
# + gsplat and the post-process map levers from train_gsplat.py (--ssim-lambda).
#
# Camera extrinsic is composed from /tf_static (no manual calibration); the
# camera_top is side-facing, so the raw frames carry motion blur at speed.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

BAG="${BAG:-demo_data/autoware_leo_drive_isuzu/all-sensors-bag1_compressed}"
OUT_DIR="${OUT_DIR:-output/isuzu_3dgs}"
EXTRINSIC="${EXTRINSIC:-configs/gaussian_splatting/isuzu_camera_top_extrinsic.yaml}"
CAMERA="${CAMERA:-/lucid_vision/camera_0}"
POINTS="${POINTS:-/sensing/lidar/concatenated/pointcloud}"
ITERS="${ITERS:-4000}"
TUM="${OUT_DIR}/lidarslam/traj_map_base_link.tum"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
set -u

echo "== [1/4] lidarslam frontend -> trajectory (concatenated cloud, no deskew) =="
rm -rf "${OUT_DIR}/lidarslam"
bash scripts/compare_with_glim.sh \
  --bag "${BAG}" --skip-glim \
  --points-topic "${POINTS}" --no-imu \
  --no-graph-based-slam --param lidarslam/param/lidarslam_kitti_velodyne.yaml \
  --robot-frame-id base_link --base-frame base_link --lidar-frame base_link \
  --out-dir "${OUT_DIR}"
test -s "${TUM}" || { echo "ERROR: empty trajectory ${TUM}"; exit 1; }

echo "== [2/4] extract posed images (compressed bag, TF extrinsic, undistort) =="
python3 tools/colored_map/extract_posed_images.py \
  --bag "${BAG}" --traj "${TUM}" \
  --camera-topic "${CAMERA}/raw_image" --camera-info-topic "${CAMERA}/camera_info" \
  --extrinsic "${EXTRINSIC}" --undistort --max-extrapolation 0.2 \
  --out "${OUT_DIR}/gsplat"

echo "== [3/4] build LiDAR-primed init cloud =="
python3 tools/colored_map/build_lidar_init.py \
  --bag "${BAG}" --traj "${TUM}" --points-topic "${POINTS}" \
  --voxel 0.15 --max-range 80 --max-points 400000 \
  --out "${OUT_DIR}/gsplat/lidar_init.ply"

echo "== [4/4] train gsplat (SSIM + densify) -> .ply =="
python3 tools/gaussian_splatting/train_gsplat.py \
  --transforms "${OUT_DIR}/gsplat/transforms.json" \
  --init-ply "${OUT_DIR}/gsplat/lidar_init.ply" \
  --densify --ssim-lambda 0.2 --iters "${ITERS}" \
  --out "${OUT_DIR}/gsplat/point_cloud.ply"

echo "done: ${OUT_DIR}/gsplat/point_cloud.ply"
