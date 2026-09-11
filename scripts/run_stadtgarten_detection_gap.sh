#!/usr/bin/env bash
# LiDAR-primed sim2real DETECTION gap on a real outdoor scene with COCO objects
# (RTK-SLAM stadtgarten_seq2). Phase 0 could not exercise the detector layer --
# the LiDAR-primed scenes on hand had no COCO objects. stadtgarten is an outdoor
# city-park sequence that does (parked bicycles, pedestrians), so this closes
# that loop: rosbag2 + RKO-LIO trajectory -> posed images -> LiDAR-primed gsplat
# -> sim2real_gap.py --detector (real-image detections surviving on the render).
#
# Window 225-290 s holds a PARKED bicycle (static -> reconstructs cleanly) plus
# walking pedestrians (dynamic -> ghosted by static 3DGS); the result separates
# the two. Camera timeshift t_imu = t_cam - 0.0206 s -> --time-offset -0.020638.
#
# Requires: a CUDA GPU with torch + gsplat + ultralytics, the RTK-SLAM
# stadtgarten_seq2 rosbag2 (scripts/download_rtk_slam_dataset.py, CC-BY 4.0) and
# a SLAM trajectory TUM (world<-imu) for it.
#
# Usage:
#   BAG=datasets/rtk_slam/ros2/stadtgarten_seq2 \
#   TRAJ=output/rtkslam_stadtgarten_seq2_run/traj_raw.tum \
#   OUT_DIR=output/stadt_3dgs bash scripts/run_stadtgarten_detection_gap.sh
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

BAG="${BAG:?set BAG to the stadtgarten_seq2 rosbag2 directory}"
TRAJ="${TRAJ:?set TRAJ to the SLAM trajectory TUM file (world<-imu)}"
OUT_DIR="${OUT_DIR:-output/stadt_3dgs}"
START="${START:-225}"
END="${END:-290}"
STRIDE="${STRIDE:-5}"
ITERS="${ITERS:-12000}"
DETECTOR="${DETECTOR:-yolov8n.pt}"

echo "== [1/4] extract posed images (CompressedImage, undistort) =="
python3 tools/colored_map/extract_posed_images.py \
  --bag "${BAG}" --traj "${TRAJ}" \
  --camera-topic /camera/image_raw/compressed \
  --intrinsics-yaml configs/gaussian_splatting/rtk_slam_cam0_intrinsics.yaml \
  --extrinsic configs/gaussian_splatting/rtk_slam_cam0_extrinsic.yaml \
  --undistort --time-offset -0.020638 \
  --start-time "${START}" --end-time "${END}" --stride "${STRIDE}" \
  --max-extrapolation 0.2 \
  --out "${OUT_DIR}/gsplat"

echo "== [2/4] LiDAR-primed init cloud =="
python3 tools/colored_map/build_lidar_init.py \
  --bag "${BAG}" --traj "${TRAJ}" --points-topic /livox/points \
  --start-time "${START}" --end-time "${END}" \
  --voxel 0.05 --min-range 1.5 --max-range 60 --max-points 400000 \
  --out "${OUT_DIR}/gsplat/lidar_init.ply"

echo "== [3/4] half-res + vignette crop, then train gsplat =="
python3 - "$OUT_DIR" <<'EOF'
import json
import sys
from pathlib import Path

from PIL import Image

out = Path(sys.argv[1]) / 'gsplat'
doc = json.load(open(out / 'transforms.json'))
scale = 0.5
w, h = int(doc['w'] * scale), int(doc['h'] * scale)
x0, y0, w2, h2 = 100, 80, 600, 440  # crop the lens vignette
(out / 'images_crop').mkdir(exist_ok=True)
for fr in doc['frames']:
    name = Path(fr['file_path']).name
    img = Image.open(out / fr['file_path']).resize((w, h), Image.LANCZOS)
    img.crop((x0, y0, x0 + w2, y0 + h2)).save(out / 'images_crop' / name)
    fr['file_path'] = f'images_crop/{name}'
doc['w'], doc['h'] = w2, h2
doc['fl_x'] *= scale
doc['fl_y'] *= scale
doc['cx'] = doc['cx'] * scale - x0
doc['cy'] = doc['cy'] * scale - y0
json.dump(doc, open(out / 'transforms_crop.json', 'w'))
print(f'training set: {len(doc["frames"])} views @ {w2}x{h2}')
EOF

python3 tools/gaussian_splatting/train_gsplat.py \
  --transforms "${OUT_DIR}/gsplat/transforms_crop.json" \
  --out "${OUT_DIR}/gsplat/point_cloud.ply" \
  --init-ply "${OUT_DIR}/gsplat/lidar_init.ply" \
  --densify --sh-degree 1 --iters "${ITERS}"

echo "== [4/4] sim2real detection gap (real detections surviving on the render) =="
python3 tools/gaussian_splatting/sim2real_gap.py \
  --ply "${OUT_DIR}/gsplat/point_cloud.ply" \
  --transforms "${OUT_DIR}/gsplat/transforms_crop.json" \
  --out "${OUT_DIR}/sim2real" \
  --offsets=-0.5,-0.25,0.25,0.5 --scale 1.0 --views 30 \
  --detector "${DETECTOR}" --det-conf 0.3

echo "done: ${OUT_DIR}/sim2real/metrics.json (recon_det_agree = the detection gap)"
