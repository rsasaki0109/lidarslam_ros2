#!/usr/bin/env bash
# Render the RTK-SLAM walking-trajectory flythrough (mp4 + README GIF):
#   construction_seq1 rosbag2 + RKO-LIO trajectory -> posed images ->
#   LiDAR-primed gsplat -> side-by-side "SLAM map + trajectory | 3DGS"
#   flythrough along the estimated walking path.
#
# Requires: a CUDA GPU with torch + gsplat, the RTK-SLAM construction_seq1
# rosbag2 (scripts/download_rtk_slam_dataset.py, CC-BY 4.0) and a SLAM
# trajectory for it in TUM format (e.g. from scripts/run_rtkslam_multiseq.sh).
# Camera calibration ships in configs/gaussian_splatting/rtk_slam_cam0_*.yaml
# (from the dataset's Kalibr calib; camera stamps are already on the IMU clock,
# so --time-offset stays 0).
#
# The default window (480-545 s) walks ~60 m around the machine hall; the
# flythrough renders the best-observed stretch of training views.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

BAG="${BAG:?set BAG to the construction_seq1 rosbag2 directory}"
TRAJ="${TRAJ:?set TRAJ to the SLAM trajectory TUM file (world<-imu)}"
OUT_DIR="${OUT_DIR:-output/rtkslam_3dgs}"
START="${START:-480}"
END="${END:-545}"
STRIDE="${STRIDE:-5}"
ITERS="${ITERS:-12000}"
FLY_FIRST="${FLY_FIRST:-116}"   # flythrough sub-range of the extracted views
FLY_LAST="${FLY_LAST:-208}"

echo "== [1/4] extract posed images (CompressedImage, undistort) =="
python3 tools/colored_map/extract_posed_images.py \
  --bag "${BAG}" --traj "${TRAJ}" \
  --camera-topic /camera/image_raw/compressed \
  --intrinsics-yaml configs/gaussian_splatting/rtk_slam_cam0_intrinsics.yaml \
  --extrinsic configs/gaussian_splatting/rtk_slam_cam0_extrinsic.yaml \
  --undistort --time-offset 0 \
  --start-time "${START}" --end-time "${END}" --stride "${STRIDE}" \
  --max-extrapolation 0.2 \
  --out "${OUT_DIR}/gsplat"

echo "== [2/4] LiDAR-primed init cloud (min-range cuts the operator ghost) =="
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

echo "== [4/4] side-by-side flythrough (SLAM map + trajectory | 3DGS) =="
python3 - "$OUT_DIR" "$FLY_FIRST" "$FLY_LAST" <<'EOF'
import json
import sys
from pathlib import Path

out = Path(sys.argv[1]) / 'gsplat'
first, last = int(sys.argv[2]), int(sys.argv[3])
doc = json.load(open(out / 'transforms_crop.json'))
doc['frames'] = doc['frames'][first:last + 1]
json.dump(doc, open(out / 'transforms_walk.json', 'w'))
print(f'flythrough path: views {first}-{last}')
EOF

# Left pane = the SLAM point-cloud map (height-coloured) + estimated
# trajectory, right pane = the 3DGS scene, both along the same camera path.
# One-way playback with a fade loop seam plus the top-down minimap (current
# position dot on the full trajectory) reads as travel; ping-pong does not.
python3 tools/gaussian_splatting/render_slam_3dgs_sidebyside.py \
  --ply "${OUT_DIR}/gsplat/point_cloud.ply" \
  --pointcloud "${OUT_DIR}/gsplat/lidar_init.ply" \
  --transforms "${OUT_DIR}/gsplat/transforms_walk.json" \
  --traj-transforms "${OUT_DIR}/gsplat/transforms_crop.json" \
  --frames 240 --fps 30 --smooth-window 5 --scale 1.0 --loop-fade 12 \
  --mp4 "${OUT_DIR}/gsplat/flythrough_sidebyside_master.mp4"

# The dot field in the map pane defeats naive GIF/x264 compression; re-encode
# the master with ffmpeg (palette GIF + crf mp4) for README-sized artifacts.
ffmpeg -y -loglevel error -i "${OUT_DIR}/gsplat/flythrough_sidebyside_master.mp4" \
  -c:v libx264 -crf 28 -preset slow -pix_fmt yuv420p \
  "${OUT_DIR}/gsplat/flythrough_sidebyside.mp4"
ffmpeg -y -loglevel error -i "${OUT_DIR}/gsplat/flythrough_sidebyside_master.mp4" \
  -vf "fps=8,scale=600:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=96[p];[s1][p]paletteuse=dither=bayer:bayer_scale=5" \
  "${OUT_DIR}/gsplat/flythrough_sidebyside.gif"

echo "done: ${OUT_DIR}/gsplat/flythrough_sidebyside.mp4 / flythrough_sidebyside.gif"
