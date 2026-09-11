# Operator Workflows

This page keeps the procedural details that do not need to stay in the top-level
README.

## Build Prerequisites

- `scanmatcher` depends on
  [`ndt_omp_ros2`](https://github.com/rsasaki0109/ndt_omp_ros2)
- clone with submodules:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2
cd ..
bash src/lidar_slam_ros2/scripts/install_source_dependencies.sh
```

- build and run the default checks:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
bash scripts/run_default_ci_checks.sh
```

Optional 3D-BBS support:

- `Thirdparty/3d_bbs` is a small MIT-licensed vendor tree with `COLCON_IGNORE`.
- `graph_based_slam` builds its CPU 3D-BBS sources automatically when
  `GRAPH_BASED_SLAM_ENABLE_3D_BBS=ON` and the vendor headers are present.
- Runtime use is still off by default; enable it with
  `use_3d_bbs_for_scan_context: true` in the graph parameter YAML or with the
  MID360 benchmark wrapper option shown in the benchmarking docs.
- To force-disable the optional build, pass
  `--cmake-args -DGRAPH_BASED_SLAM_ENABLE_3D_BBS=OFF`.

## Main Entry Points

| Goal | Entrypoint |
| --- | --- |
| Fixed public first map | `lidarslam-map demo`; add `--viewer none` for headless use or `--dry-run --json` for a network- and write-free plan. Add `--output PLAN` to retain that plan once without shell redirection. |
| Autoware pointcloud-map quickstart | `bash scripts/run_autoware_quickstart.sh` |
| Full dogfood flow | `bash scripts/run_rko_lio_graph_autoware_dogfood.sh --auto-exit-secs 20` |
| Standard NTU VIRAL benchmark | `bash scripts/run_rko_lio_graph_benchmark.sh` |
| KITTI Odometry small_gicp evaluation | `bash scripts/run_kitti_odometry_benchmark.sh --sequence 00 --small-gicp --force-prepare` |
| KITTI Odometry small_gicp sweep | `bash scripts/sweep_kitti_small_gicp.sh --dataset "$KITTI_ODOMETRY_ROOT" --sequences "00 05 07"` |
| localization_zoo PCD/trajectory → fixed graph bag | `python3 scripts/pcd_sequence_to_rosbag2.py --help` then `bash scripts/run_offline_determinism_check.sh` |
| MID360 cross-validation benchmark | `bash scripts/run_rko_lio_mid360_crossval_benchmark.sh` |
| Offline browser 3D map preview | `lidarslam-map view output/my_map` writes and opens a self-contained HTML preview; use `--no-open` on headless hosts. The lower-level `export_mid360_robot_3d_map_preview.py` remains available for custom sampling limits. |
| Recent map-session history | `lidarslam-map sessions` validates direct child session bundles and opens a local newest-first catalog; use `--status action_required`, `--viewer none`, or read-only `--json` as needed. |
| Evidence-backed session comparison | Select two cards in `sessions.html`, or run `lidarslam-map compare output/session-a output/session-b`; use `--viewer none` or read-only `--json` on headless/automated hosts. |
| Privacy-first maintainer report | Choose **Get support** in `sessions.html`, or run `lidarslam-map support output/session-a`; review the fixed three-member ZIP before attaching it to a public issue. |
| Non-destructive 3D map cleanup | Create an RKO graph map with `run --editable`, select unwanted boxes or accepted loops in `view`, then run the browser-printed `edit` command. Replay inputs are auto-detected; `edit --help-all` provides overrides for older outputs. |
| Multi-session map project | Put the trusted anchor first: `lidarslam-map merge output/day1 output/day2 --output-dir output/site_project`, then inspect separately colored session paths with `lidarslam-map view output/site_project`. |
| Mixed-quality open-data GNSS smoke | `bash scripts/run_open_data_applanix_velodyne_gnss_smoke.sh --bag /path/to/rosbag2 --applanix-msg-dir /tmp/applanix/applanix_msgs/msg --verify-map` |
| Mixed-quality open-data GNSS benchmark | `bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh --bag /path/to/rosbag2 --applanix-msg-dir /tmp/applanix/applanix_msgs/msg --verify-map` |
| Leo Drive classic-path suite | `bash scripts/run_open_data_classic_path_benchmark_suite.sh --applanix-msg-dir /tmp/applanix/applanix_msgs/msg --verify-map` |
| Packet IMU deskew validation matrix | `bash scripts/run_open_data_packet_imu_deskew_validation_matrix.sh --applanix-msg-dir /tmp/applanix/applanix_msgs/msg` |
| Dynamic-object-filter save-map benchmark | `bash scripts/run_dynamic_object_filter_benchmark.sh` |
| MID360 place-recognition comparison | `bash scripts/run_place_recognition_benchmark.sh` |
| Pre-tag reproducible release bundle | `python3 scripts/check_release_bundle_reproducibility.py /tmp/lidarslam_ros2_release_candidate.tar.gz` |
| Release/readiness gate | `bash scripts/run_release_readiness_checks.sh --fail-on-profiles` |

## Required Input Topics

### Public default path: `RKO-LIO + graph_based_slam`

Launch:

```bash
ros2 launch lidarslam rko_lio_slam.launch.py \
  bag_path:=/path/to/rosbag2 \
  lidar_topic:=/os_cloud_node/points \
  imu_topic:=/os_cloud_node/imu
```

Required inputs:

- `lidar_topic`: `sensor_msgs/msg/PointCloud2`
- `imu_topic`: `sensor_msgs/msg/Imu`

Optional inputs:

- `/gnss/fix`: `sensor_msgs/msg/NavSatFix` when `graph_based_slam use_gnss:=true`

Internal wiring in this launch:

- `RKO-LIO` publishes odometry on `/rko_lio/odometry`
- `RKO-LIO` publishes submap source clouds on `/rko_lio/frame`
- `graph_based_slam` consumes those via `odom_input` and `cloud_input`

Not currently supported in the public path:

- wheel odometry / vehicle speed topic fusion

GNSS note:

- GNSS is added as translation-only pose-graph constraints in the backend
- when covariance is present, edge weight is scaled from `position_covariance`
- `NavSatFix` does not standardize RTK fix status, so `graph_based_slam`
  treats low horizontal covariance as `RTK-like`
- default threshold: `gnss_rtk_fix_max_horizontal_stddev_m = 0.3`

### Classic path: `scanmatcher + graph_based_slam`

Launch:

```bash
ros2 launch lidarslam lidarslam.launch.py \
  input_cloud:=/points_raw \
  imu_topic:=/imu
```

Required inputs:

- `input_cloud`: `sensor_msgs/msg/PointCloud2`
- TF from `robot_frame_id` to the LiDAR frame

Optional inputs:

- `imu_topic`: `sensor_msgs/msg/Imu` when `scanmatcher use_imu:=true`
- odom TF into `odom_frame_id` when `scanmatcher use_odom:=true`
- `/gnss/fix`: `sensor_msgs/msg/NavSatFix` when backend `use_gnss:=true`

Internal wiring in this launch:

- `scanmatcher` publishes `lidarslam_msgs/msg/MapArray` on `map_array`
- `graph_based_slam` subscribes to `map_array`

Voxel-grid safety:

- every classic scanmatcher PCL VoxelGrid call is preflighted against PCL's
  signed 32-bit index/layout limit;
- a `VOXEL_GRID_*` warning rejects only the named stage instead of passing an
  unfiltered cloud downstream or terminating the node;
- use `vg_size_for_input` for `input_scan`, `registration_target`, and
  `recovery_target` warnings;
- use `vg_size_for_map` for `initial_map` and `map_update` warnings;
- inspect coordinate units and outliers before increasing a leaf size. The
  node never changes map resolution automatically.

See the
[VoxelGrid refusal contract](operational-reliability.md#classic-scanmatcher-voxelgrid-refusal-boundary)
for every reason code, preserved state, and the bounded issue #69 regression.

### Adapting another PointCloud2 LiDAR

Use this checklist when adapting another LiDAR that publishes
`sensor_msgs/msg/PointCloud2`. It establishes readiness for one controlled
first run; it does not validate accuracy, make a vendor part of the supported
matrix, or select universal tuning values. Run the fixed public demo first so
that an installation problem is not confused with a sensor-adaptation problem.

Replace every `<PLACEHOLDER>` below with an observed value before running a
command. If a value is unknown, stop at that check instead of guessing it.

1. **Confirm the topic and message contract.**

   ```bash
   ros2 topic list -t
   ros2 topic type <POINTCLOUD_TOPIC>
   ros2 topic echo --once --field header.frame_id <POINTCLOUD_TOPIC>
   ros2 topic echo --once --field fields <POINTCLOUD_TOPIC>
   ```

   Expected: `<POINTCLOUD_TOPIC>` is listed as
   `sensor_msgs/msg/PointCloud2`, `header.frame_id` is non-empty, and the
   `fields` output contains FLOAT32 `x`, `y`, and `z`. For the RKO-LIO path,
   also identify a supported per-point time field named `t`, `timestamp`,
   `time`, or `stamps`; a header timestamp alone does not satisfy that path.
   If any required field is absent, fix the driver or use a conversion layer
   before launching SLAM.

2. **Check timestamp order and rate.**

   For a rosbag2 input, run the product preflight first:

   ```bash
   lidarslam-map doctor /path/to/rosbag2 --json
   ```

   Review the selected topic's timestamp findings and keep `sampled` distinct
   from a full-bag proof. For a live topic, observe both a timestamp and the
   publication rate:

   ```bash
   timeout 5s ros2 topic echo --once --field header.stamp <POINTCLOUD_TOPIC>
   ros2 topic hz --window 20 <POINTCLOUD_TOPIC>
   ```

   Expected: timestamps advance and the rate remains positive. Repair the
   publisher clock, rosbag playback clock, or timestamp conversion when they
   do not. Do not hide timestamp warnings by increasing a timeout.

3. **Measure the frame relationship; never invent an extrinsic.**

   Use the non-empty frame observed in check 1 as `<LIDAR_FRAME>` and verify
   the directed transform to the robot base while the source is live or being
   played:

   ```bash
   ros2 run tf2_ros tf2_echo <BASE_FRAME> <LIDAR_FRAME>
   ```

   Expected: repeated transforms in the same parent-to-child direction as the
   configured launch. If the path is missing or the measured translation or
   rotation is unknown, stop and repair the broadcaster or calibration. An
   identity transform is valid only when it is the measured mounting
   relationship; guessing an extrinsic can produce a plausible but invalid
   map.

4. **Record the sensor period and valid range in a profile.**

   The classic path uses these fields in its `main_param_dir` YAML:

   ```yaml
   scan_matcher:
     ros__parameters:
       scan_period: <SECONDS_PER_SCAN>
       scan_min_range: <MIN_RANGE_M>
       scan_max_range: <MAX_RANGE_M>
   ```

   The RKO-LIO path uses `min_range` and `max_range` launch arguments or its
   `rko_param_file`; its per-point timestamps determine the scan timing. Set
   the values from the sensor specification or a bounded measurement, and
   record the source in the profile. Do not copy a value from another vendor
   merely because the topic type matches.

5. **Run one explicit, reviewable launch.**

   For the classic path, the public remap and frame arguments are:

   ```bash
   ros2 launch lidarslam lidarslam.launch.py \
     input_cloud:=<POINTCLOUD_TOPIC> \
     imu_topic:=<IMU_TOPIC> \
     robot_frame_id:=<BASE_FRAME> \
     base_frame:=<BASE_FRAME> \
     lidar_frame:=<LIDAR_FRAME> \
     main_param_dir:=/path/to/custom-lidarslam.yaml \
     publish_static_tf:=false
   ```

   Set `publish_static_tf:=true` only when the seven static-transform values
   (`static_tf_x`, `static_tf_y`, `static_tf_z`, `static_tf_qx`,
   `static_tf_qy`, `static_tf_qz`, and `static_tf_qw`) come from the measured
   calibration. For RKO-LIO, use the corresponding public arguments and put
   the measured extrinsics in `rko_param_file`:

   ```bash
   ros2 launch lidarslam rko_lio_slam.launch.py \
     bag_path:=/path/to/rosbag2 \
     lidar_topic:=<POINTCLOUD_TOPIC> \
     imu_topic:=<IMU_TOPIC> \
     base_frame:=<BASE_FRAME> \
     lidar_frame:=<LIDAR_FRAME> \
     rko_param_file:=/path/to/measured-rko.yaml
   ```

   A controlled first run means that the input, frames, timestamps, period,
   ranges, and exact profile are recorded before mapping. It is not an
   accuracy or hardware-support claim. If the checklist exposes a sensor
   question that the existing contract cannot answer, use the
   [sensor-support issue form](https://github.com/rsasaki0109/lidar_slam_ros2/issues/new?template=sensor-support.yml)
   with sanitized observations; do not attach raw bags, map geometry, or
   location-bearing logs.

### KITTI / LiDAR-only evaluation path

KITTI Odometry Velodyne sequences do not include IMU. Use this path for
LiDAR-only evaluation and frontend tuning, not as the public default workflow.

Download and run one sequence:

```bash
bash scripts/download_kitti_odometry.sh --velodyne
export KITTI_ODOMETRY_ROOT="$PWD/datasets/KITTI_odometry"
bash scripts/run_kitti_odometry_benchmark.sh --sequence 00 --small-gicp --force-prepare
```

Sweep several `small_gicp` parameter sets:

```bash
bash scripts/sweep_kitti_small_gicp.sh \
  --dataset "$KITTI_ODOMETRY_ROOT" \
  --sequences "00 05 07"
```

The KITTI wrappers write prepared rosbag2 data and benchmark artifacts under
`output/` by default. The raw KITTI dataset belongs under `datasets/`, which is
local-only and ignored by Git.

For a trajectory already evaluated by `localization_zoo`, the converter also
accepts `--estimate-matrices` and writes exact-timestamp
`/rko_lio/odometry` + cloud pairs for the deterministic backend runner.
Localization Zoo's `csv_lidar_pose` / `*_evaluated_gt.txt` matrices are already
LiDAR-frame poses: keep the default `--gt-frame lidar` and do not pass
`--calib`. For official KITTI camera-frame pose matrices, pass both
`--gt-frame camera` and KITTI `calib.txt` with `--calib`; the explicit pair
prevents accidental double conversion. The frozen sequence-00 experiment and
adoption decision are in
[kitti00-localization-zoo-graph-loop-2026-07.md](research/kitti00-localization-zoo-graph-loop-2026-07.md).

When multiple ROS overlays exist, pass
`--setup /path/to/install/setup.bash` to
`run_offline_determinism_check.sh`. The wrapper resolves the runner from that
overlay and records its SHA-256, params and bag-metadata hashes, and every
`--param` override in `offline_determinism_summary.md`. For a fast
information-weight ablation, pass
`--param fixed_loop_edges_path:=/path/to/loop_edges.csv`; descriptor search is
skipped while the frozen constraints are replayed. Experimental Scan Context
runs can reduce proposal cost deterministically with
`scan_context_query_stride` (default 1, preserving existing behavior).

### Backend only: `graph_based_slam`

Launch:

```bash
ros2 launch graph_based_slam graphbasedslam.launch.py
```

Default required input:

- `map_array`: `lidarslam_msgs/msg/MapArray`

Optional backend aids:

- `/imu`: `sensor_msgs/msg/Imu` when `use_imu_preintegration:=true`
- `/gnss/fix`: `sensor_msgs/msg/NavSatFix` when `use_gnss:=true`

Alternative direct-input mode used by the RKO-LIO launch:

- `odom_input`: `nav_msgs/msg/Odometry`
- `cloud_input`: `sensor_msgs/msg/PointCloud2`

Useful GNSS weighting parameters:

- `gnss_topic`
- `gnss_info_weight`
- `gnss_use_covariance_weighting`
- `gnss_covariance_min_variance_m2`
- `gnss_covariance_max_variance_m2`
- `gnss_rtk_fix_max_horizontal_stddev_m`
- `gnss_rtk_fix_weight_scale`
- `gnss_non_rtk_weight_scale`

Optional save-time dynamic-object filter:

- affects only the map written by `/map_save`
- does not change live odometry, loop closure, or the published working map
- useful when repeated passes observe parked/static structure consistently but
  transient objects appear only once

Parameters:

- `use_dynamic_object_filter`
- `dynamic_object_filter_voxel_size`
- `dynamic_object_filter_min_observations`
- `dynamic_object_filter_temporal_window`
- `dynamic_object_filter_max_range_from_sensor_m`

Inspect a bag before enabling GNSS weighting:

```bash
python3 scripts/inspect_navsatfix_covariance.py /path/to/rosbag2 --topic /gnss/fix
```

For Leo Drive open-data driving bags that only expose Applanix raw GNSS status,
inspect `GSOF50` instead:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
python3 scripts/inspect_applanix_gsof50_quality.py /path/to/rosbag2 \
  --topic /lvx_client/gsof/ins_solution_rms_50 \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg
```

If the bag has `GSOF49/50` but no `/gnss/fix`, generate a sidecar rosbag2 that
publishes only `sensor_msgs/msg/NavSatFix`:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
python3 scripts/convert_applanix_gsof_to_navsatfix_bag.py \
  --input /path/to/rosbag2 \
  --output /tmp/applanix_navsatfix_bag \
  --gsof49-topic /lvx_client/gsof/ins_solution_49 \
  --gsof50-topic /lvx_client/gsof/ins_solution_rms_50 \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --force
```

If you want to test the same Applanix raw messages as `sensor_msgs/msg/Imu`,
generate an IMU sidecar too:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
python3 scripts/convert_applanix_gsof_to_imu_bag.py \
  --input /path/to/rosbag2 \
  --output /tmp/applanix_imu_bag \
  --gsof49-topic /lvx_client/gsof/ins_solution_49 \
  --gsof50-topic /lvx_client/gsof/ins_solution_rms_50 \
  --output-topic /imu \
  --frame-id base_link \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --force
```

Then play the original bag together with the generated sidecar bag:

```bash
ros2 bag play /path/to/rosbag2 --clock
ros2 bag play /tmp/applanix_navsatfix_bag
```

For an end-to-end open-data GNSS smoke with automatic `/map_save`, use:

```bash
bash scripts/run_open_data_gnss_smoke.sh \
  --bag /path/to/rosbag2 \
  --verify-map
```

`run_open_data_gnss_smoke.sh` auto-detects the `NavSatFix` topic from
`--gnss-bag` when provided, otherwise from `--bag`.

For Leo Drive driving bags that expose LiDAR as
`velodyne_msgs/msg/VelodyneScan` and GNSS quality as Applanix `GSOF49/50`,
use the packet-to-PointCloud2 wrapper instead:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_smoke.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

That wrapper will:

- prefer same-bag native `sensor_msgs/msg/NavSatFix` / `sensor_msgs/msg/Imu`
  topics when they exist
- otherwise generate a `NavSatFix` sidecar bag from `GSOF49/50`
- optionally generate an `Imu` sidecar bag from `GSOF49/50`
- extract a local `TUM` reference from `GSOF49` with `extract_applanix_gsof49_reference.py`
- build a minimal `velodyne_pointcloud` overlay on demand with
  `bash scripts/prepare_velodyne_pointcloud_overlay.sh`
- convert `VelodyneScan` packets into `sensor_msgs/msg/PointCloud2`
- run `lidarslam.launch.py`, call `/map_save`, and optionally verify the output

For rosbag2 `compression_mode: FILE` inputs, the smoke and benchmark wrappers
stage a private playback view under the output directory. Any database that
ROS 2 decompresses during playback is removed from that private view on exit,
so the source bag directory is not left with a multi-gigabyte temporary file.

To benchmark the same `driving_30_kmh` bag as a four-way classic-path
comparison, use:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_classic_path_benchmark_suite.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

That suite writes:

- `classic_path_report.md`
- `classic_path_report.json`
- `classic_path_report.svg`

To rerun the current MID360 place-recognition comparison entrypoint, use:

```bash
bash scripts/run_place_recognition_benchmark.sh
```

That wrapper reruns the distance-only baseline and a `Scan Context` candidate,
then emits:

- `place_recognition_report.md`
- `place_recognition_report.json`

For packet IMU deskew, the important caveat is runtime sensitivity. On the real
Leo Drive `all-sensors-bag1` and `all-sensors-bag6` front-lidar cases, native
`/sensing/imu/imu_data` works when the packet benchmark runs at `rate=1.0`.
Current reference numbers are:

- `bag1_front`, `no_imu`: `APE RMSE 0.248 m`
- `bag1_front`, `imu`: `APE RMSE 0.251 m`
- `bag6_front`, `no_imu`: `APE RMSE 0.422 m`
- `bag6_front`, `imu`: `APE RMSE 0.365 m`

The benchmark wrapper defaults to `rate=1.0` for all runs and deterministically
prefers a `/front/` packet stream when several Velodyne topics exist. To
validate the same A/B automatically on the default front-lidar cases, run:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_packet_imu_deskew_validation_matrix.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg
```

That matrix runs both `no_imu` and `imu` at `rate=1.0` for determinism and
writes per-case outputs plus:

- `packet_imu_deskew_validation.md`
- `packet_imu_deskew_validation.json`

The default acceptance criteria are:

- `no_imu` path coverage >= `0.95`
- `imu` path coverage >= `0.95`
- `imu_rmse / no_imu_rmse <= 1.10`
- `imu_matched_poses / no_imu_matched_poses >= 0.80`
deskew and `34.089 m` with `--imu-rotation-use-orientation false`. That is why
the public packet path still keeps `--use-imu false` by default.

If a bag carries NavSatFix messages whose header stamps do not track ROS time,
the backend now falls back to receive time when the skew exceeds
`gnss_header_stamp_max_skew_sec` (default `30 s`). That makes `all-sensors-bag6`
attach GNSS edges again, but its native `/gnss/fix` still disagrees with the
`GSOF49` reference enough that it is better suited to georeferenced smoke tests
than to clean GNSS cross-validation.

If you still want to test packet-based IMU deskew with a real static TF, use:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --use-imu true \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --robot-frame-id base_link \
  --imu-frame-id base_link \
  --verify-map
```

That path uses:

- `convert_applanix_gsof_to_imu_bag.py`
- `extract_static_transform_from_bag.py`
- `PointCloud2.time`-based deskew in `scanmatcher`
- `--imu-rotation-use-orientation false` for the gyro-only rotation variant

To turn the same real open-data path into a benchmark artifact with
`traj_raw.tum`, `traj_corrected.tum`, and `metrics.json`, use:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

### Odometry and TF: two separate contracts

An `nav_msgs/msg/Odometry` message contains a parent frame in
`header.frame_id` and a child frame in `child_frame_id`. Publishing those
fields does not, by itself, guarantee that the matching transform is present
in the `/tf` tree. Start with the read-only bag check:

```bash
lidarslam-map doctor /path/to/rosbag2
```

When the bag contains Odometry, doctor scans the highest-count Odometry topic
and all recorded TF topics, with a 100,000-message bound per topic. It reports
empty or inconsistent frame IDs, no connecting path, or a path containing only
`/tf_static`. A multi-hop path such as
`odom -> base_footprint -> base_link` is accepted when at least one edge comes
from dynamic `/tf`.

When a selected PointCloud2 topic and that dynamic path both exist, doctor then
makes a second bounded pass in bag record order. At each cloud record it checks
whether every required dynamic edge has already appeared and whether the
latest stamp observed on each edge is at least the cloud's `header.stamp`. It
reports startup gaps and every positive future-TF gap; there is no arbitrary
millisecond tolerance because the exact-stamp lookup can reject any request
newer than the latest buffered transform. A clean result is necessary
recorded-bag evidence, not proof of live executor scheduling, DDS delay, clock
alignment, TF buffer history, or interpolation at every sensor timestamp. Use
the checks below for the running system and replace every angle-bracket
placeholder first.

1. **Check the Odometry message frames**

   First confirm that `<ODOM_TOPIC>` is the intended
   `nav_msgs/msg/Odometry` topic, then sample both frame fields:

   ```bash
   timeout 5s ros2 topic echo --once --field header.frame_id <ODOM_TOPIC>
   timeout 5s ros2 topic echo --once --field child_frame_id <ODOM_TOPIC>
   ```

   Expected: both outputs are non-empty and identify the intended
   `<ODOM_FRAME>` parent and `<BASE_FRAME>` child. If either is empty or
   unexpected, correct the Odometry publisher or launch remap before checking
   TF; do not invent frame names in a viewer.

2. **Check that the directed TF path exists**

   ```bash
   ros2 run tf2_ros tf2_echo <ODOM_FRAME> <BASE_FRAME>
   ```

   Expected: repeated `At time ...` transforms in the same parent-to-child
   direction as the sampled message. If the transform is unavailable, the
   Odometry topic is not sufficient: enable the supported TF broadcaster or
   static-extrinsic configuration for the robot, then repeat this check. Do
   not silence TF warnings or copy a robot-specific broadcaster as a fix.

3. **Check transform freshness separately**

   ```bash
   ros2 run tf2_ros tf2_monitor <ODOM_FRAME> <BASE_FRAME>
   ```

   Expected: the monitor reports a live publisher and bounded delay for the
   path. A missing path is a broadcaster/configuration problem; a large delay,
   future extrapolation, or stale timestamp is a timing problem. Align the
   clocks and message/TF timestamps or repair the actual publisher rate, then
   rerun doctor and the monitor. Increasing a lookup timeout alone does not
   repair stale data. Silencing the warning or substituting a stale transform
   does not repair it either.

## Run `RKO-LIO + graph_based_slam`

The main launch entrypoint is:

```bash
ros2 launch lidarslam rko_lio_slam.launch.py \
  bag_path:=/path/to/rosbag2 \
  lidar_topic:=/os_cloud_node/points \
  imu_topic:=/os_cloud_node/imu
```

Useful parameter files:

- default graph backend: `graph_based_slam/param/graphbasedslam.yaml`
- default scanmatcher frontend: `lidarslam/param/lidarslam.yaml`
- NTU VIRAL RKO-LIO profile: `lidarslam/param/rko_lio_ntu_viral.yaml`
- MID360 tuned profile: `lidarslam/param/lidarslam_mid360_rko_graph.yaml`

## Save Maps

Save the current map at any time with:

```bash
ros2 service call /map_save std_srvs/srv/Empty
```

Typical outputs:

- `map.pcd`
- `pose_graph.g2o`
- `pointcloud_map/pointcloud_map_metadata.yaml`
- `pointcloud_map/*.pcd`
- `map_projector_info.yaml`

## Autoware Map Output Notes

`graph_based_slam` always writes `map_projector_info.yaml`.

- without GNSS: `projector_type: Local`
- with GNSS and a stable origin: `projector_type: LocalCartesian` plus
  `map_origin`

To stage an existing run into an Autoware map bundle:

```bash
bash scripts/prepare_autoware_map_from_graph_slam.sh \
  output/bench_rko_lio_ntu_viral_loopgate_20260324 \
  /tmp/autoware_maps/ntu_viral_loopgate
```

To open the staged map through Autoware's map loaders:

```bash
bash scripts/run_autoware_pointcloud_map_viewer_docker.sh \
  /tmp/autoware_maps/ntu_viral_loopgate \
  /tmp/autoware_core \
  /tmp/autoware_map_runtime_ws
```

For the short supported path, use
`bash scripts/run_autoware_quickstart.sh` instead.

## Loop Closure Notes

`graph_based_slam` supports two loop-candidate sources:

- distance-based revisit search
- built-in GPL-free Scan Context place recognition

The backend validates candidates geometrically before adding a loop edge and
keeps only the best local edge inside the configured dedup window.

Scan Context normally excludes the 50 most recent submaps. Experimental short
sequences may override `scan_context_exclude_recent`, but the value must remain
positive and the default stays 50. Treat a smaller window as an ablation: it
can expose aliased descriptors that the geometric verifier must reject.

To regenerate the README loop-area zoom figure used for visual inspection of
closing-segment duplication:

```bash
python3 scripts/generate_readme_loop_zoom_figure.py
```

## Benchmark And Dataset Pointers

Recommended public benchmark:

```bash
bash scripts/download_ntu_viral_tnp01.sh --dry-run
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_rko_lio_graph_benchmark.sh
```

The dry run performs no write or network request and reports the remaining
phases, official archive identity, and conservative destination-space gate.

Current MID360 cross-validation path:

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh
```

The public benchmark and release-report flow is documented in
[benchmarking.md](benchmarking.md).

## Related Docs

- [Autoware Quickstart](autoware-quickstart.md)
- [Benchmarking And Release Gate](benchmarking.md)
- [Comparison](comparison.md)
