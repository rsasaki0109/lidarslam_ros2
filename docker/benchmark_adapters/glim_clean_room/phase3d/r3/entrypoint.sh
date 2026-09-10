#!/bin/sh
# The only container entrypoint for the opt-in candidate.  It writes no
# success artifact itself: the collector publishes a complete atomic batch or
# one terminal failure record, and the host reopens every byte before sealing.
set -eu

input_root=${PHASE3D_INPUT_ROOT:-/phase3d/input}
calibration_root=${PHASE3D_CALIBRATION_ROOT:-/phase3d/calibration}
config_root=${PHASE3D_CONFIG_ROOT:-/phase3d/config}
output_root=${PHASE3D_OUTPUT_ROOT:-/phase3d/output}
lidar_topic=${PHASE3D_LIDAR_TOPIC:-/points}
imu_topic=${PHASE3D_IMU_TOPIC:-/imu}

test -d "$input_root"
test -d "$calibration_root"
test -d "$config_root"
test -d "$output_root"
test -x /opt/phase3d-r3/collect_phase3d.py

exec python3 /opt/phase3d-r3/collect_phase3d.py \
  --input-root "$input_root" \
  --calibration-root "$calibration_root" \
  --config-root "$config_root" \
  --output-root "$output_root" \
  --lidar-topic "$lidar_topic" \
  --imu-topic "$imu_topic"
