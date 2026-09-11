# Jetson MID-360 Robot Runbook

This runbook is the standard operator path for a Jetson-class computer, a Livox
MID-360, and a quadruped or biped robot. It assumes the goal is offline mapping
from a rosbag2 log using the tracked `RKO-LIO + graph_based_slam` MID-360
presets.

For scope boundaries, see
[Jetson MID-360 Robot Scope](jetson-mid360-robot-scope.md). For mount
measurement, see
[Jetson MID-360 Static TF Worksheet](jetson-mid360-static-tf-worksheet.md).

## 0. Prepare The Robot Profile

To smoke-test this runbook without launching SLAM:

```bash
bash scripts/smoke_mid360_robot_runbook.sh
```

The smoke creates a metadata-only fake MID-360 bag, validates the default
profile, runs the recording dry-run, runs the post-recording check, runs
readiness, runs the map dry-run, and checks the generated reports.

If the robot is not available yet, use public MID-360 data instead of a
synthetic bag for the first realistic intake pass:

```bash
python3 scripts/download_mid360_robot_public_dataset.py --list

python3 scripts/download_mid360_robot_public_dataset.py \
  --dataset driving_slam_mid360 \
  --dataset-root datasets/mid360_public \
  --check
```

The recommended first target is the Zenodo `Driving SLAM Test with Livox
MID360` bag (`10.5281/zenodo.14841855`). The downloader writes:

- `datasets/mid360_public/driving_slam_mid360/mid360_robot_public_dataset_intake.json`
- `datasets/mid360_public/driving_slam_mid360/mid360_robot_public_dataset_intake.md`
- `datasets/mid360_public/driving_slam_mid360/driving_slam_mid360_profile.yaml`
- `output/mid360_public/driving_slam_mid360/mid360_robot_recording_check.json`

The recommended archive is pinned by exact size, SHA-256, and legacy MD5.
Interrupted downloads remain as `.zip.part` and resume with a validated HTTP
Range response. If the server ignores Range, the downloader restarts safely
instead of appending duplicate bytes. Existing archives are re-hashed, and ZIP
members are preflighted before transaction-safe extraction. Use `--force` only
to restart a bad partial or deliberately replace an existing extraction;
`--skip-md5` never disables a registered SHA-256 or size check.

Use `--dataset hard_pointcloud_mid360_outdoor_kidnap_a` after the first bag
passes to exercise more difficult outdoor MID-360 failure cases from the Hard
Point Cloud Localization Dataset (`10.5281/zenodo.10122133`).

Before downloading additional multi-GB split bags for loop evaluation, inspect
the public GT trajectories:

```bash
python3 scripts/analyze_mid360_robot_public_loop_candidates.py \
  --download-gt \
  --write
```

This writes:

- `output/mid360_public/loop_candidates/mid360_robot_public_loop_candidates.json`
- `output/mid360_public/loop_candidates/mid360_robot_public_loop_candidates.md`

The GT-only pass identifies Hard Point Cloud `outdoor_kidnap` as the public
MID-360 loop-evaluation sequence. It requires both split bags,
`outdoor_kidnap_a` and `outdoor_kidnap_b`, because the loop closes near the end
of the combined sequence, not inside `outdoor_kidnap_a` alone.

To verify that the public loop is a real cloud-overlap loop before tuning SLAM,
run the GT loop cloud analyzer. It reads the real PointCloud2 windows around a
GT loop candidate, transforms both windows into the GT trajectory frame, and
reports nearest-neighbor overlap:

```bash
python3 scripts/analyze_mid360_robot_public_loop_cloud.py
```

This writes:

- `output/mid360_public/outdoor_kidnap_loop_cloud/mid360_robot_public_loop_cloud_analysis.json`
- `output/mid360_public/outdoor_kidnap_loop_cloud/mid360_robot_public_loop_cloud_analysis.md`

The continuous RKO-LIO kidnap-recovery path uses the same merged public bag, but
with the tracked kidnap-tolerant RKO config:

```bash
bash scripts/run_rko_lio_graph_autoware_dogfood.sh \
  --bag datasets/mid360_public_loops/outdoor_kidnap_raw/rosbag2 \
  --lidar-topic /livox/points \
  --imu-topic /livox/imu \
  --lidarslam-param lidarslam/param/lidarslam_mid360_rko_graph.yaml \
  --rko-param configs/mid360_robot/rko_lio_mid360_kidnap_tolerant.yaml \
  --base-frame base_link \
  --lidar-frame livox_frame \
  --imu-frame livox_frame \
  --output-dir output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final \
  --run-name outdoor_kidnap_ab_rko_kidnap_relocalization_final \
  --wait-for-offline-completion \
  --skip-viewer
```

After the run, verify the saved map, run loop alignment, export the browser
preview, and build the continuous relocalization gate:

```bash
python3 scripts/verify_autoware_map.py \
  output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final

python3 scripts/analyze_mid360_robot_loop_alignment.py \
  output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final \
  --trajectory output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final/outdoor_kidnap_ab_rko_kidnap_relocalization_final_0/outdoor_kidnap_ab_rko_kidnap_relocalization_final_tum_0.txt \
  --write

python3 scripts/export_mid360_robot_3d_map_preview.py \
  output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final \
  --trajectory output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final/outdoor_kidnap_ab_rko_kidnap_relocalization_final_0/outdoor_kidnap_ab_rko_kidnap_relocalization_final_tum_0.txt \
  --loop-alignment output/mid360_public/outdoor_kidnap_ab_rko_kidnap_relocalization_final/mid360_robot_loop_alignment.json

python3 scripts/run_mid360_robot_public_continuous_relocalization_gate.py
```

Current public evidence for this path is `PASS`: 2896 RKO poses over 553.801 s,
one global kidnap relocalization event, Autoware map verify PASS, loop alignment
PASS with 20 candidates and nearest revisit 0.162 m, public loop endpoint
distance 2.515 m at the GT start/end stamps, and tracked recovery config
matching the run config. The endpoint check is intentional: a generic revisit
candidate alone can pass while the public loop endpoint is still wrong.

The reset-based segment path remains as fallback and independent evidence. If
continuous RKO-LIO regresses, plan reset-based segment runs for the GT loop
endpoints:

```bash
python3 scripts/plan_mid360_robot_public_loop_segment_reset.py
```

This writes:

- `output/mid360_public/outdoor_kidnap_segment_reset_plan/mid360_robot_public_loop_segment_reset.json`
- `output/mid360_public/outdoor_kidnap_segment_reset_plan/mid360_robot_public_loop_segment_reset.md`
- `output/mid360_public/outdoor_kidnap_segment_reset_plan/mid360_robot_public_bag_segments.json`

The report identifies which RKO-safe scan segment contains the loop start and
which segment contains the loop end, then prints exact clip and per-segment RKO
commands. Use this when map verify passes but the trajectory stops before the
actual loop revisit.

After the start/end segment maps are generated, align the two independent reset
maps and gate their cloud overlap:

```bash
python3 scripts/analyze_mid360_robot_public_segment_map_cloud_alignment.py
```

This writes:

- `output/mid360_public/outdoor_kidnap_segment_reset_alignment/mid360_robot_public_segment_map_cloud_alignment.json`
- `output/mid360_public/outdoor_kidnap_segment_reset_alignment/mid360_robot_public_segment_map_cloud_alignment.md`
- `output/mid360_public/outdoor_kidnap_segment_reset_alignment/mid360_robot_public_segment_map_cloud_alignment.ply`

This is the CloudAnalyzer path for checking whether reset-based loop maps are
spatially consistent instead of only checking that each segment saved an
Autoware-compatible map. The session dashboard and production-candidate bundle
recognize the alignment JSON/Markdown/PLY as optional evidence; missing
alignment artifacts do not fail bundle verification on their own.

To summarize the public-data completion state without rerunning SLAM, build the
completion gate:

```bash
python3 scripts/run_mid360_robot_public_completion_gate.py
```

This writes:

- `output/mid360_public/completion_gate/mid360_robot_public_completion_gate.json`
- `output/mid360_public/completion_gate/mid360_robot_public_completion_gate.md`

The completion gate is scoped to the public MID-360 segment-reset loop path. It
checks the GT loop cloud overlap, reset plan, both segment RKO map outputs,
Autoware map compatibility for both segment maps, segment-map CloudAnalyzer
alignment, public RKO adoption/config match, dashboard presence, and the
session/dashboard/bundle/import/release entrypoints.

To include this evidence in the release-readiness wrapper as a hard gate:

```bash
bash scripts/run_release_readiness_checks.sh \
  --skip-default-ci \
  --skip-benchmark-summary \
  --public-mid360-completion
```

Once both split archives are extracted, merge the two raw sqlite rosbag2
directories so RKO-LIO sees the loop in a single replay. This path does not
deserialize messages, so it preserves both the public PointCloud2 stream
(`/livox/points`) and the Livox CustomMsg stream (`/livox/lidar`):

```bash
python3 scripts/merge_mid360_robot_public_split_bags.py \
  --input-bag datasets/mid360_public/hard_pointcloud_mid360_outdoor_kidnap_a/extracted/outdoor_kidnap_a/outdoor_kidnap_a \
  --input-bag datasets/mid360_public/hard_pointcloud_mid360_outdoor_kidnap_a/extracted/outdoor_kidnap_b/outdoor_kidnap_b \
  --output-bag datasets/mid360_public_loops/outdoor_kidnap_raw/rosbag2 \
  --force
```

This writes:

- `datasets/mid360_public_loops/outdoor_kidnap_raw/rosbag2/mid360_robot_public_split_bag_merge.json`
- `datasets/mid360_public_loops/outdoor_kidnap_raw/rosbag2/mid360_robot_public_split_bag_merge.md`

The merger preserves the original message timestamps from each input bag,
preserves the sqlite schema row, reports the inter-bag gap, and rejects
overlapping bags. If a smaller focused loop bag is needed later, use
`build_mid360_robot_public_loop_bag.py` against this raw merged source or the
two extracted split bags with `--time-window-sec <start> <end>`.

If RKO-LIO reports `Received LiDAR scan with X seconds delta to previous scan`
errors after running on the merged bag, the upstream PointCloud2 `header.stamp`
field is non-monotonic. Rewrite it to match the rosbag2 receive time first:

```bash
python3 scripts/rewrite_mid360_robot_bag_stamps.py \
  --input-bag datasets/mid360_public_loops/outdoor_kidnap_raw/rosbag2 \
  --output-bag datasets/mid360_public_loops/outdoor_kidnap_stamped/rosbag2 \
  --force
```

This writes:

- `datasets/mid360_public_loops/outdoor_kidnap_stamped/mid360_robot_bag_stamp_rewriter.json`
- `datasets/mid360_public_loops/outdoor_kidnap_stamped/mid360_robot_bag_stamp_rewriter.md`

The rewriter defaults to rewriting `sensor_msgs/msg/PointCloud2` and
`sensor_msgs/msg/Imu`. Override with `--rewrite-msgtype` / `--rewrite-topic` if
only one stream needs the fix. All other topics pass through unchanged.

After running one or more public dataset intakes, generate the comparison page:

```bash
python3 scripts/generate_mid360_robot_public_dataset_report.py
```

This writes:

- `output/mid360_public/mid360_robot_public_dataset_report.json`
- `output/mid360_public/mid360_robot_public_dataset_report.md`
- `output/mid360_public/mid360_robot_public_dataset_report.html`

After a focused RKO-LIO sweep has produced verified map artifacts, include that
evidence in the standard comparison report:

```bash
python3 scripts/generate_mid360_robot_public_dataset_report.py \
  --map-sweep output/mid360_public/rko_sweep/mid360_robot_public_rko_sweep.json
```

The report adds `map_validation`, `map_verified_cases`, and the source
`map_sweep_manifest` per dataset. A `MAP_VERIFIED` row means the sweep saved an
Autoware-compatible pointcloud map and `verify_autoware_map.py` passed for at
least one real-data case, with no failed verification cases in the selected
manifest.

Select launch-ready map candidates without starting SLAM:

```bash
python3 scripts/run_mid360_robot_public_dataset_map_candidates.py
```

This writes:

- `output/mid360_public/mid360_robot_public_dataset_map_candidates.json`
- `output/mid360_public/mid360_robot_public_dataset_map_candidates.md`

The selector accepts `PASS` and `WARN` rows by default when
`ready_for_mid360_launch` is true. Use `--pass-only` for a strict gate, or
`--datasets hard_pointcloud_mid360_outdoor_kidnap_a` to run one public bag.
The manifest includes bag duration, bag size, estimated runtime, estimated map
output size, free space, and existing map-output collisions under each
candidate's `safety` block, including `estimated_runtime_sec` and
`estimated_output_bytes`. Only pass `--run` when the listed commands and output
paths are acceptable.

Useful safety options:

```bash
python3 scripts/run_mid360_robot_public_dataset_map_candidates.py \
  --min-free-gb 10 \
  --runtime-scale 2.0

python3 scripts/run_mid360_robot_public_dataset_map_candidates.py \
  --allow-existing-map-output

python3 scripts/run_mid360_robot_public_dataset_map_candidates.py \
  --datasets hard_pointcloud_mid360_outdoor_kidnap_a \
  --run \
  --run-timeout-sec 900
```

After a public map run, diagnose the run artifacts before retrying a failed
dataset:

```bash
python3 scripts/diagnose_mid360_robot_public_map_run.py \
  --datasets hard_pointcloud_mid360_outdoor_kidnap_a \
  --write
```

This writes:

- `output/mid360_public/mid360_robot_public_map_run_diagnosis.json`
- `output/mid360_public/mid360_robot_public_map_run_diagnosis.md`

The diagnosis records whether RKO-LIO and `graph_based_slam` started, whether
offline completion and `/map_save` were reached, LiDAR scan timestamp delta
error counts, dropped-keypoint counts, partial TUM trajectory files, and map
output presence.

For datasets that fail with RKO-LIO LiDAR scan timestamp deltas, find the
contiguous PointCloud2 spans that stay within the RKO-LIO `1.0 s` adjacent-scan
limit:

```bash
python3 scripts/analyze_mid360_robot_public_bag_segments.py \
  --datasets hard_pointcloud_mid360_outdoor_kidnap_a \
  --write
```

This writes:

- `output/mid360_public/mid360_robot_public_bag_segments.json`
- `output/mid360_public/mid360_robot_public_bag_segments.md`

The segment report records scan counts, detected gap locations, the recommended
real-data segment, clip start/end timestamps, and the segment duration. Use it
before cutting a short public rosbag for a focused RKO-LIO retry.

Clip the recommended segment into a smaller rosbag2 directory:

```bash
python3 scripts/clip_mid360_robot_public_bag_segment.py \
  --dataset hard_pointcloud_mid360_outdoor_kidnap_a \
  --force
```

The clipper writes a real-data subset under:

- `datasets/mid360_public_segments/hard_pointcloud_mid360_outdoor_kidnap_a/segment_002/rosbag2`
- `datasets/mid360_public_segments/hard_pointcloud_mid360_outdoor_kidnap_a/segment_002/mid360_robot_public_bag_segment_clip.json`
- `datasets/mid360_public_segments/hard_pointcloud_mid360_outdoor_kidnap_a/segment_002/mid360_robot_public_bag_segment_clip.md`

The default clip margin is `0.0 s` so the bag stays inside the RKO-safe span
reported by segment analysis. Add `--margin-sec` only when the adjacent scans
have also been checked.

If the clipped real-data bag starts cleanly but stalls with
`Keypoints for ICP registration = 0`, run the focused RKO-LIO frontend sweep:

```bash
python3 scripts/run_mid360_robot_public_rko_sweep.py \
  --bag datasets/mid360_public_segments/hard_pointcloud_mid360_outdoor_kidnap_a/segment_002/rosbag2 \
  --output-dir output/mid360_public/rko_sweep \
  --run \
  --run-timeout-sec 90
```

This writes:

- `output/mid360_public/rko_sweep/mid360_robot_public_rko_sweep.json`
- `output/mid360_public/rko_sweep/mid360_robot_public_rko_sweep.md`
- one `rko_sweep.yaml` and `slam.launch.log` under each per-case output
  directory
- one `verify_autoware_map.log` under each per-case output directory when map
  outputs are saved

The default cases compare lower `voxel_size`, lower `min_range`,
`deskew=false`, and `double_downsample=false` variants against the clipped
public bag. Use `--case name:voxel_size=0.5,min_range=1.0,double_downsample=false,deskew=false`
to pin an additional candidate.

Generate the quality dashboard from the sweep manifest:

```bash
python3 scripts/generate_mid360_robot_public_rko_quality_report.py \
  --sweep output/mid360_public/rko_sweep/mid360_robot_public_rko_sweep.json
```

This writes:

- `output/mid360_public/rko_sweep/mid360_robot_public_rko_quality_report.json`
- `output/mid360_public/rko_sweep/mid360_robot_public_rko_quality_report.md`
- `output/mid360_public/rko_sweep/mid360_robot_public_rko_quality_report.html`

The dashboard ranks cases using Autoware map verification, offline completion,
trajectory path length, pointcloud tile coverage, point count, density, runtime,
and RKO-LIO runtime error signatures. Use it to pick a tracked config only after
the case is `MAP_VERIFIED`.

Check that the tracked RKO-LIO config is still backed by the top-ranked
gate-pass public real-data case:

```bash
python3 scripts/check_mid360_robot_rko_config_adoption.py \
  --quality-report output/mid360_public/rko_sweep/mid360_robot_public_rko_quality_report.json \
  --config configs/mid360_robot/rko_lio_mid360_low_voxel_no_deskew.yaml \
  --require-best
```

This writes:

- `output/mid360_public/rko_sweep/mid360_robot_rko_config_adoption.json`
- `output/mid360_public/rko_sweep/mid360_robot_rko_config_adoption.md`

The command exits non-zero when the YAML is missing required RKO-LIO parameters,
does not match any swept real-data case, matches a case that failed the quality
gate, or no longer matches the top-ranked gate-pass case when `--require-best`
is used.

For the normal adoption path, run the combined gate instead of invoking the
quality and config-adoption steps separately:

```bash
python3 scripts/run_mid360_robot_public_rko_adoption_gate.py \
  --from-existing \
  --sweep output/mid360_public/rko_sweep/mid360_robot_public_rko_sweep.json \
  --config configs/mid360_robot/rko_lio_mid360_low_voxel_no_deskew.yaml
```

This writes:

- `output/mid360_public/rko_sweep/mid360_robot_public_rko_adoption_gate.json`
- `output/mid360_public/rko_sweep/mid360_robot_public_rko_adoption_gate.md`

Use `--run` only when the public clipped bag and per-case output directories are
ready for a full RKO-LIO sweep. In `--run` mode the same command executes the
sweep first, then regenerates quality, config-adoption, and final gate reports.

After a gate-pass public RKO map run, check whether loop revisits show obvious
trajectory/cloud split risk:

```bash
python3 scripts/analyze_mid360_robot_loop_alignment.py \
  output/mid360_public/rko_sweep/voxel_0p50_min_1p00_dd_on \
  --write
```

This writes:

- `output/mid360_public/rko_sweep/voxel_0p50_min_1p00_dd_on/mid360_robot_loop_alignment.json`
- `output/mid360_public/rko_sweep/voxel_0p50_min_1p00_dd_on/mid360_robot_loop_alignment.md`

The analyzer looks for non-adjacent trajectory revisits, measures their revisit
distance, and checks whether the nearby pointcloud tiles split into too many
2D connected components. Treat this as loop-alignment risk evidence for a
CloudCompare/Foxglove follow-up, not as a ground-truth drift metric.

`--offline-quiet-log-secs 15` remains available as a fallback for older builds
where RKO-LIO produces a complete TUM trajectory but does not print the normal
offline completion marker before staying alive. The current offline node should
finish without this fallback.

The current known-good public clipped-bag frontend config is
`configs/mid360_robot/rko_lio_mid360_low_voxel_no_deskew.yaml`
(`voxel_size: 0.5`, `deskew: false`, `double_downsample: true`). The focused
run that established this config produced an Autoware-compatible pointcloud map
with `verify_autoware_map.py` `PASS`.

If no network access and no captured bag is available, generate a readable local
PointCloud2/Imu/TF rosbag2 only to test report plumbing:

```bash
python3 scripts/generate_mid360_robot_sample_bag.py \
  /tmp/mid360_sample_bag \
  --duration-sec 5 \
  --force

python3 scripts/check_mid360_robot_readiness.py /tmp/mid360_sample_bag \
  --robot-profile configs/mid360_robot/livox_mid360_default.yaml \
  --output-dir output/mid360_robot_sample \
  --write-manifest
```

Use this sample bag to test local tooling and report generation only. It is not
a substitute for a stationary and walking bag from the actual robot.

To exercise the full reporting path in one command, generate a sample session:

```bash
python3 scripts/run_mid360_robot_sample_session.py \
  --bag-root /tmp/mid360_sample_session/bags \
  --output-dir /tmp/mid360_sample_session/out \
  --duration-sec 5 \
  --force
```

This writes a synthetic rosbag2, recording plan sidecars, the post-recording
check, readiness report, map dry-run manifest, and
`mid360_robot_session_dashboard.html`.

Use `--scenario low-rate`, `--scenario frame-mismatch`, or
`--scenario missing-tf` to exercise warning and failure dashboards before a real
robot bag exists.

To run all sample-session QA scenarios and verify their expected statuses:

```bash
python3 scripts/run_mid360_robot_sample_session_matrix.py \
  --bag-root /tmp/mid360_sample_matrix/bags \
  --output-dir /tmp/mid360_sample_matrix/out \
  --duration-sec 5 \
  --force
```

This writes `mid360_robot_sample_session_matrix.json`,
`mid360_robot_sample_session_matrix.md`, and
`mid360_robot_sample_session_matrix.html` plus per-scenario dashboards under the
matrix output directory.

Copy the default profile and edit it for the robot:

```bash
cp configs/mid360_robot/livox_mid360_default.yaml \
  configs/mid360_robot/<robot_name>.yaml
```

Set these fields before recording the target route:

- `base_frame`
- `lidar_frame`
- `imu_frame`
- `expected_pointcloud_topic`
- `expected_imu_topic`
- `mount.xyz`
- `mount.q_xyzw`

Validate the profile:

```bash
python3 scripts/validate_mid360_robot_profile.py \
  configs/mid360_robot/<robot_name>.yaml
```

## 0.5. Check The Jetson Host

Before field recording, write a host-readiness snapshot:

```bash
python3 scripts/check_jetson_mid360_host_readiness.py \
  --bag-dir /path/to/bag_storage \
  --output-dir output/mid360_robot_test
```

This writes:

- `output/mid360_robot_test/jetson_mid360_host_readiness.json`
- `output/mid360_robot_test/jetson_mid360_host_readiness.md`

The host report checks:

- Jetson/NVIDIA model metadata and CPU architecture
- `ros2`, `colcon`, `tegrastats`, `nvpmodel`, and `jetson_clocks` availability
- output and bag-storage free space
- thermal-zone temperatures
- available memory
- CPU governor state, when readable

## 1. Record A Short Bag

For a standard field session, use the runner first:

```bash
bash scripts/run_mid360_robot_field_session.sh \
  --robot-profile configs/mid360_robot/<robot_name>.yaml \
  --bag-root /path/to/bag_storage \
  --run-id stand_01 \
  --duration-sec 30 \
  --output-dir output/mid360_robot_test \
  --dry-run
```

When the dry-run command looks correct, run it without `--dry-run`. The standard
field runner records the bag, runs the post-recording check, and leaves mapping
as a generated dry-run plan. It writes:

- `output/mid360_robot_test/mid360_robot_field_session.json`
- `output/mid360_robot_test/mid360_robot_field_session.md`
- `output/mid360_robot_test/mid360_robot_session_dashboard.html`

Mapping does not run unless `--run-map` is passed explicitly.

Open or regenerate the session dashboard:

```bash
python3 scripts/generate_mid360_robot_session_dashboard.py \
  output/mid360_robot_test
```

The dashboard shows the field-session timeline, artifact status, topic/frame
summary, metadata rates, checks, commands, and a route sketch for:
recording, post-check, map dry-run, and map run. When a production-candidate
report exists in the same directory, the dashboard switches to that timeline
and shows host readiness, public RKO adoption, production readiness, and map
diagnosis artifacts.

For a production-candidate session, dry-run the full chain before the robot is
allowed to spend a long recording window:

```bash
bash scripts/run_mid360_robot_production_candidate_session.sh \
  --robot-profile configs/mid360_robot/<robot_name>.yaml \
  --bag-root /path/to/bag_storage \
  --run-id production_candidate_01 \
  --duration-sec 600 \
  --output-dir output/mid360_robot_test \
  --dry-run
```

After the commands and paths are correct, pass `--run`. The production
candidate runner executes Jetson host readiness, recording, post-recording
check, mapping with `autoware_map_diagnosis.json`, the public RKO adoption gate
from `mid360_robot_public_rko_sweep.json`, and the production readiness gate. It
writes:

- `output/mid360_robot_test/mid360_robot_production_candidate_session.json`
- `output/mid360_robot_test/mid360_robot_production_candidate_session.md`
- `output/mid360_robot_test/mid360_robot_session_dashboard.html`
- `output/mid360_robot_test/mid360_robot_production_readiness.json`

Use `--record-only` for a storage/topic rehearsal, `--skip-map` when only the
recording artifacts are needed, or `--skip-public-gate --adoption-gate <json>`
when reusing an already-approved public RKO adoption gate.

To re-run only the production gate and dashboard after an existing artifact
directory has been copied back from the Jetson, use:

```bash
bash scripts/run_mid360_robot_production_candidate_session.sh \
  --robot-profile configs/mid360_robot/<robot_name>.yaml \
  --bag-root /path/to/bag_storage \
  --run-id production_candidate_01 \
  --duration-sec 600 \
  --output-dir output/mid360_robot_test \
  --segment-map-alignment output/mid360_public/outdoor_kidnap_segment_reset_alignment/mid360_robot_public_segment_map_cloud_alignment.json \
  --from-existing-artifacts \
  --run
```

This mode reuses `jetson_mid360_host_readiness.json`,
`mid360_robot_recording_check.json`, `mid360_robot_readiness.json`,
`autoware_map_diagnosis.json`, and
`public_rko_adoption_gate/mid360_robot_public_rko_adoption_gate.json`, then
rewrites `mid360_robot_production_readiness.json` and the session dashboard.
When `--segment-map-alignment` is provided, the dashboard surfaces that public
reset-loop CloudAnalyzer evidence and the bundle manifest carries the
JSON/Markdown/PLY as optional review artifacts.

Export a portable production-candidate bundle before moving artifacts off the
Jetson:

```bash
python3 scripts/export_mid360_robot_production_candidate_bundle.py \
  output/mid360_robot_test \
  --output /tmp/production_candidate_01.tar.gz \
  --verify
```

The bundle writes `mid360_robot_production_candidate_bundle.json` and includes
the production candidate JSON/Markdown, host readiness, recording check,
readiness, map diagnosis, public RKO adoption gate, production readiness,
dashboard HTML, profile snapshot, and record plan. Optional loop-alignment,
segment-map CloudAnalyzer, and 3D map preview artifacts are included when they
exist, but they are not required for bundle verification. It does not include the
full rosbag2 directory; the manifest keeps the original `bag_path` and a
`--from-existing-artifacts --run` recheck command for replay on a development
machine after extraction.

On a development machine, import the tarball and re-run the production gate
from the bundled artifacts:

```bash
python3 scripts/import_mid360_robot_production_candidate_bundle.py \
  /tmp/production_candidate_01.tar.gz \
  --output-dir output/rechecked_candidate_01 \
  --recheck \
  --verify
```

This writes `mid360_robot_production_candidate_bundle_import.json`, updates the
extracted `mid360_robot_production_candidate_bundle.json` with `last_import`,
rewrites `artifacts/mid360_robot_production_readiness.json`, and regenerates
`artifacts/mid360_robot_session_dashboard.html`. The importer rejects unsafe
tar members and treats missing required artifacts as a failed import.

Dry-run the recording command first:

```bash
bash scripts/record_mid360_robot_bag.sh \
  --robot-profile configs/mid360_robot/<robot_name>.yaml \
  --bag-root /path/to/bag_storage \
  --run-id stand_01 \
  --duration-sec 30 \
  --dry-run
```

When the command and storage path look correct, run the same command without
`--dry-run`:

```bash
bash scripts/record_mid360_robot_bag.sh \
  --robot-profile configs/mid360_robot/<robot_name>.yaml \
  --bag-root /path/to/bag_storage \
  --run-id stand_01 \
  --duration-sec 30
```

Check the metadata:

```bash
ros2 bag info /path/to/bag_storage/stand_01
```

The bag should contain at least:

- `sensor_msgs/msg/PointCloud2`
- `sensor_msgs/msg/Imu`
- `/tf` or `/tf_static` when the robot publishes static transforms

The recording helper writes sidecar files next to the bag root:

- `/path/to/bag_storage/stand_01_record_plan.json`
- `/path/to/bag_storage/stand_01_record_plan.md`
- `/path/to/bag_storage/stand_01_profile.yaml`

Run the post-recording check before moving to mapping:

```bash
bash scripts/check_mid360_robot_recording.sh \
  --bag /path/to/bag_storage/stand_01 \
  --robot-profile /path/to/bag_storage/stand_01_profile.yaml \
  --output-dir output/mid360_robot_test
```

This writes:

- `output/mid360_robot_test/mid360_robot_recording_check.json`
- `output/mid360_robot_test/mid360_robot_recording_check.md`
- `output/mid360_robot_test/mid360_robot_readiness.json`
- `output/mid360_robot_test/mid360_robot_run_plan.json`

`ros2 bag info` is only the first pass. The readiness step below also computes
metadata message rates and, when the bag storage is readable from Python,
samples the selected point cloud, IMU, and TF topics to check frame IDs and TF
connectivity.

## 2. Run Readiness

Run the pre-run gate before SLAM:

```bash
python3 scripts/check_mid360_robot_readiness.py /path/to/rosbag2 \
  --robot-profile configs/mid360_robot/<robot_name>.yaml \
  --output-dir output/mid360_robot_test \
  --write-manifest
```

This writes:

- `output/mid360_robot_test/mid360_robot_readiness.json`
- `output/mid360_robot_test/mid360_robot_readiness.md`
- `output/mid360_robot_test/mid360_robot_run_plan.json`
- `output/mid360_robot_test/mid360_robot_run_plan.md`

The readiness report includes:

- selected point cloud and IMU metadata rate in Hz
- sampled point cloud and IMU `header.frame_id`, when messages are readable
- sampled TF connectivity between `base_frame` and the configured LiDAR/IMU
  frames, when TF messages are readable

### PASS

Proceed to dry-run or mapping. Required topics and profile expectations matched.

### WARN

Review the warning before mapping. Typical warnings:

- no `/tf` or `/tf_static` metadata in the bag
- no Livox/MID-360 preset recommendation from the generic preflight
- low metadata message rate for the selected point cloud or IMU topic
- sampled TF does not connect the configured robot and sensor frames

Mapping can still be valid if frames are passed explicitly and the bag is known
to be a MID-360 log.

### FAIL

Do not run mapping yet. Typical failures:

- missing `PointCloud2`
- missing `Imu`
- invalid robot profile
- expected topic mismatch between profile and bag
- sampled point cloud or IMU `header.frame_id` does not match the configured
  profile frame

Fix the bag or profile and rerun readiness.

## 3. Dry-Run The Map Command

Dry-run the map runner to inspect the exact SLAM command:

```bash
bash scripts/run_mid360_robot_map.sh /path/to/rosbag2 \
  --robot-profile configs/mid360_robot/<robot_name>.yaml \
  --output-dir output/mid360_robot_test \
  --write-manifest \
  --dry-run
```

Confirm:

- `--lidar-topic` matches the MID-360 point cloud topic
- `--imu-topic` matches the IMU topic
- `--base-frame`, `--lidar-frame`, and `--imu-frame` match the profile
- `lidarslam_mid360_rko_graph.yaml` is used
- `rko_lio_mid360.yaml` is used

## 4. Run Mapping

Run the offline mapping path:

```bash
bash scripts/run_mid360_robot_map.sh /path/to/rosbag2 \
  --robot-profile configs/mid360_robot/<robot_name>.yaml \
  --output-dir output/mid360_robot_test \
  --write-manifest \
  --write-diagnosis
```

The runner launches `lidarslam rko_lio_slam.launch.py`, waits for offline
completion, calls `/map_save`, and then writes diagnosis files.
Internally, the post-run diagnosis uses:

```bash
python3 scripts/diagnose_autoware_map_run.py \
  output/mid360_robot_test \
  --bag /path/to/rosbag2 \
  --write
```

## 5. Inspect Outputs

Expected outputs:

- `slam.launch.log`
- `map_save.log`
- `mid360_robot_run_plan.json`
- `mid360_robot_run_plan.md`
- `autoware_map_diagnosis.json`
- `autoware_map_diagnosis.md`
- `map_projector_info.yaml`
- `pointcloud_map/pointcloud_map_metadata.yaml`
- `pointcloud_map/*.pcd`
- `pose_graph.g2o`
- `map.pcd`

Open the diagnosis:

```bash
less output/mid360_robot_test/autoware_map_diagnosis.md
```

If map verification artifacts exist, also run:

```bash
python3 scripts/verify_autoware_map.py \
  output/mid360_robot_test/pointcloud_map
```

## 6. Production Readiness Gate

After the Jetson host check, real robot recording check, readiness check,
mapping run, map diagnosis, and public RKO adoption gate all exist, run the
production gate:

```bash
python3 scripts/check_mid360_robot_production_readiness.py \
  --artifact-dir output/mid360_robot_test \
  --map-diagnosis output/mid360_robot_test/autoware_map_diagnosis.json \
  --adoption-gate output/mid360_public/rko_sweep/mid360_robot_public_rko_adoption_gate.json
```

This writes:

- `output/mid360_robot_test/mid360_robot_production_readiness.json`
- `output/mid360_robot_test/mid360_robot_production_readiness.md`

The gate is intentionally stricter than the public-data development gate. It
requires a Jetson host-readiness pass, a real robot bag instead of public or
synthetic evidence, recording and readiness passes, a verified map diagnosis,
the public RKO adoption gate, stable expected frames, MID-360 topic rates, and a
bag duration of at least `600 s` by default. A public dataset can validate the
development path, but it cannot make a robot deployment production ready.

The same gate can be driven by the production-candidate runner:

```bash
bash scripts/run_mid360_robot_production_candidate_session.sh \
  --robot-profile configs/mid360_robot/<robot_name>.yaml \
  --bag-root /path/to/bag_storage \
  --run-id production_candidate_01 \
  --duration-sec 600 \
  --output-dir output/mid360_robot_test \
  --run
```

That report keeps the command log and artifact pointers in
`mid360_robot_production_candidate_session.json`, so a failed production gate
has a single place to inspect before rerunning only the broken step.

## 7. Open A Viewer

Foxglove path:

```bash
bash scripts/run_graph_slam_pointcloud_map_in_autoware_foxglove.sh \
  output/mid360_robot_test
```

Autoware Docker viewer path:

```bash
bash scripts/run_graph_slam_pointcloud_map_in_autoware.sh \
  output/mid360_robot_test
```

## 8. Field Notes

Keep these files with each field run:

- robot profile YAML
- readiness JSON and Markdown
- run-plan JSON and Markdown
- diagnosis JSON and Markdown
- sensor mount measurement notes
- any launch overrides used in the field

Do not replace the robot profile silently between readiness and mapping. If the
profile changes, rerun readiness.
