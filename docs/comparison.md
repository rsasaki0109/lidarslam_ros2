# Comparison

This page is the public comparison snapshot for `lidarslam_ros2 v0.2.2` and
the in-flight `v0.3` track on `develop`.

It is intentionally scoped to workflows that are actually exercised in this
repository. It is not trying to be a universal ranking of every LiDAR SLAM
system.

## Release Track vs Research Track

`v0.3` introduced per-dataset release profiles so the gate stops squashing
heterogeneous datasets onto a single APE threshold. `v0.4` then **graduated the
former research-track profiles to blocking** (decision 2026-06-07,
`docs/roadmap/v0.4.md`):

- **Release track (blocking)** — a FAIL blocks the release. As of v0.5 this is:
  `Newer College math-hard` (ground truth), `NTU VIRAL tnp_01` (ground truth),
  the two `mid360_gt_rtkslam_construction_*` profiles (**total-station ground
  truth**, graduated in v0.5), the Leo Drive applanix/velodyne open-data
  cross-validation, and the KITTI Odometry 00/05/07 LO baseline comparison
  (non-regression).
- **Report-only** — `MID-360` vs GLIM was demoted in v0.5 (decision D-GT-2,
  `docs/roadmap/v0.5.md`): cross-validation against another SLAM estimate
  measures agreement, not accuracy, and the same sensor is now gated on real
  total-station checkpoints. It stays as a regression canary. The Leo Drive
  profile keeps its cross-validation caveat (separate track). New profiles
  introduced mid-cycle (e.g. the outdoor Stadtgarten pair) soak as report-only
  before graduating.

The distinction is exercised by `scripts/run_release_readiness_checks.sh`,
which evaluates each profile in `scripts/release_profiles.yaml` and emits
`PASS` / `FAIL` / `WARN` / `TARGET_MET` / `NO_DATA` per dataset.

## Strategic Position

This repository is deliberately positioned as:

- a ROS 2 pointcloud-map authoring stack
- a benchmarkable mapping workflow
- a non-GPL public path for reusable map artifacts

It is not primarily positioned as:

- the smallest possible LiDAR odometry package
- a localization reliability research platform
- a universal winner on every SLAM benchmark

The intended differentiation is operational:

- generate pointcloud maps
- keep map metadata and georeference outputs usable
- verify saved bundles
- compare runs with tracked metrics and reports
- standardize submission artifacts for repeatable evaluation

That is the product layer this repository is trying to own.

## Capability Comparison

| Workflow | Role in this repo | License stance in the public path | Frontend / backend shape | Loop closure in the documented path | Pointcloud-map authoring / verification |
| --- | --- | --- | --- | --- | --- |
| `lidarslam_ros2` default | recommended public workflow | non-GPL default | `RKO-LIO` frontend + `graph_based_slam` backend | yes | yes |
| `RKO-LIO` raw | odometry baseline | non-GPL default | LIO frontend only | no | no |
| `KISS-ICP` baseline | comparison baseline | external comparison only | LiDAR odometry only | no | no |
| `LIO-SAM` | research reference | excluded from the default release path | tightly coupled factor-graph SLAM | yes | no supported path in this repo |

## Differentiators

The public differentiators currently exercised in this repository are:

- non-GPL default workflow
- saved-map verification tooling
- GNSS-aware `map_projector_info.yaml` export
- save-time dynamic-object cleanup
- tracked benchmark/report artifacts
- real open-data packet-path evidence
- a focused `map_authoring_report` that summarizes benchmark, georeference,
  cleanup, and fallback-path evidence in one place
- a standard submission-bundle helper that collects `pointcloud_map/`,
  `map_projector_info.yaml`, `metrics.json`, trajectories, logs, focused reports,
  and a generated `map_qa_summary.md`

Those are stronger differentiators for map authoring and evaluation than for
pure odometry novelty.

## Local Benchmark Snapshot

These numbers come from generated benchmark artifacts under local `output/`
directories. `output/` is ignored by git; use the commands in
[Benchmarking And Release Gate](benchmarking.md) to regenerate the reports.

### Release-track datasets

As of v0.4 every profile below is a blocking release-track profile. The current
numbers all sit under their `pass` thresholds, so graduation flips their status
from `WARN` to `PASS` without breaking the gate.

| Dataset | Configuration | Reference kind | APE RMSE (m) | Profile gate | Notes |
| --- | --- | --- | --- | --- | --- |
| `NTU VIRAL tnp_01` | current default | `ground_truth` | `0.952` | `PASS` (pass ≤ 1.00, target 0.30) | outdoor long-loop GT |
| `NTU VIRAL tnp_01` | best observed   | `ground_truth` | `0.870` | `PASS` (same)                     | loop-gated backend run |
| `MID-360` RTK-SLAM Construction Hall 2 | indoor default | `ground_truth` (total station, 16 chkpt) | `0.154` (median 0.061) | `PASS` (pass ≤ 0.30, target 0.15) | dense odometry scored, `--match-tolerance 2.0` |
| `MID-360` RTK-SLAM Construction Hall 1 | indoor default | `ground_truth` (total station, 16 chkpt) | `0.403` (median 0.263) | `PASS` (pass ≤ 0.55, target 0.30) | hardest indoor hall (published baselines ~0.22) |
| `MID-360` RTK-SLAM Stadtgarten 2 | outdoor config | `ground_truth` (total station, 19 chkpt) | `0.835` (median 0.327) | report-only soak (pass ≤ 1.20) | outdoor park; `double_downsample: false` (see methodology note) |
| `MID-360` RTK-SLAM Stadtgarten 1 | outdoor config | `ground_truth` (total station, 36 chkpt) | `1.666` (median 1.511) | report-only soak (pass ≤ 2.20) | 26 min / ~1 km park loop; raw odometry drift, no GNSS / loop closure |
| `MID-360` | current default                  | `cross_validation` vs GLIM | `3.641` | report-only since v0.5 (D-GT-2)   | solid-state LiDAR, non-360° FOV |
| `MID-360` | best observed                    | `cross_validation` vs GLIM | `3.590` | report-only (same)                | rerun with same tuned backend family |
| `MID-360` | Scan Context candidate           | `cross_validation` vs GLIM | `3.816` | report-only                       | fair current-code comparison; still opt-in |
| `MID-360` | experimental BEV-assisted rerank | `cross_validation` vs GLIM | `3.607` | report-only                       | sensor-agnostic rerank of distance candidates; still opt-in |
| Leo Drive (applanix/velodyne) | current default | `cross_validation` vs Applanix GSOF49 | varies per bag | `PASS` (pass ≤ 1.50, target 0.50) | open-data Velodyne packet path |

The Newer College `math-hard` profile (ground truth) is the tightest gate
(pass ≤ 0.10); its numbers are not checked in to this repo and are reported
separately on the long-form benchmark notes. The KITTI Odometry 00/05/07 LO
baseline comparison is wired through `scripts/run_kitti_00_05_07_report.sh` and
emits a non-regression report under
`output/kitti_dev_<timestamp>/kitti_dev_report.md`.

As of v0.5 the MID-360 release evidence is **real ground truth**: the two
RTK-SLAM Construction Hall profiles block on SE(3)-aligned total-station
checkpoint RMSE (`ape_rmse_gt_m`), the same metric family as NTU VIRAL and
Newer College. `MID-360` vs GLIM is report-only (regression canary, D-GT-2);
Leo Drive still blocks on its cross-validation threshold. Dataset, metric
definition and attribution:
`docs/research/rtkslam-total-station-gt-methodology.md`.

Source artifacts:

- `output/benchmark_summary.md` (generated locally)
- `output/latest_report.html` (generated locally)
- `output/stress_validation_report_<YYYYMMDD>.md` (generated locally)
- `scripts/release_profiles.yaml` (profile definitions)
- `output/kitti_dev_<timestamp>/kitti_dev_report.md` (generated locally, KITTI LO baseline)

## Same-input HILTI 2022 exp04 vs GLIM CPU

This is the strongest directly comparable OSS result for the current
competitive `lidarslam_ros2` profile. Both systems used the same HILTI 2022
`exp04` ROS 2 bag bytes, LiDAR--IMU calibration, CPU-only host, and the same six
surveyed checkpoints. GLIM's trajectory starts after its three-second IMU
initialization, so the earlier first checkpoint was excluded for both systems
instead of being clamped or extrapolated. APE is SE(3)-aligned position RMSE;
all aggregate values below are from three complete runs.

| System | APE RMSE median (range), m | Processing RTF median | Peak RSS maximum, MB | Completion |
| --- | ---: | ---: | ---: | ---: |
| **lidarslam_ros2** | **0.056536** (identical across runs) | 0.993 | **586.83** | 3/3 |
| GLIM CPU | 0.086624 (0.084876--0.086702) | **0.2437** | 690.88 | 3/3 |

The `lidarslam_ros2` trajectory error is 34.7% lower and its maximum peak RSS
is 15.0% lower. GLIM is substantially faster. Our individual processing RTFs
were `1.094`, `0.993`, and `0.977`, so the median meets 1.0 while the worst run
does not. GLIM completed with 1,228 poses per run; `lidarslam_ros2` completed
with 1,258 poses and ended within 0.0173 seconds of the bag end.

The result does not establish complete map-quality superiority. At 0.1 m
common downsampling, `lidarslam_ros2` had better mean plane thickness
(`0.06081` vs `0.07911` m), but GLIM had slightly better p95 thickness
(`0.12169` vs `0.12504` m) and substantially greater planar coverage
(`0.43030` vs `0.17949`). Consequently this page claims a trajectory and
peak-memory win on this sequence, not a universal system-level victory.

This is a scoped win on HILTI `exp04`: `lidarslam_ros2` wins trajectory APE
and peak memory, while GLIM wins runtime. It is not an overall-SOTA claim.
The table preserves the completed three-run historical comparison; normal
development needs only one new run.

In a separate `n=1` development measurement, task-local correspondence
reduction kept the 1,258-pose frontend trajectory byte-identical while mean ICP
time moved from 21.60 ms to 20.92 ms and frontend wall time from 47.51 s to
46.55 s. These timing deltas are directional optimization evidence, not a
statistical or cross-system claim.

Reproducibility identifiers:

- HILTI bag SHA-256: `d1117a4c6e4c3626a3039e48719ec6e39af34b0a95f5a9807163bc717229c8ee`
- common six-checkpoint SHA-256: `537b7e0f13f223a07f8329cc2b7080e33dc2765a13293d67a7eb3312090db1b8`
- GLIM core v1.2.2: `faa264a1bce1bda406f73457e35511f56cdc2eaa`
- GLIM ROS 2: `4a9e7a4cb084967c8525a1be529ad3ba2a118ae7`
- runner and complete research record:
  [`competitive-slam-plan-2026-07.md`](research/competitive-slam-plan-2026-07.md#glim-cpu-exp04-baseline)

## Voxel-SLAM v17 research candidate vs pinned OSS rivals

This separate experiment evaluates a weak-axis/bounded-map Voxel-SLAM
derivative. It is not the default `RKO-LIO + graph_based_slam` release path.
The comparison used the common exposed SOTA-v5 references and frozen
interpolating scorer. Values are median per-sequence APE RMSE in metres from
three complete repetitions; the final column is the geometric mean across the
three datasets.

| System | NavINST Indoor02 | Oxford Spires Keble 05 | UrbanNav HK Tunnel 1 | Geometric mean |
| --- | ---: | ---: | ---: | ---: |
| **v17 candidate** | 0.17486 | 0.13949 | **488.22566** | **2.2836** |
| GLIM | 1.52860 | **0.11316** | 757.80951 | 5.0779 |
| Point-LIO | 0.65142 | 0.13626 | 638.93868 | 3.8388 |
| FAST-LIO2 | 0.84663 | 0.52470 | 757.42790 | 6.9576 |
| fixed Voxel-SLAM | **0.16664** | 0.14671 | 818.79944 | 2.7160 |

Relative to the pinned rivals, v17's geometric-mean APE is 55.0% lower than
GLIM, 40.5% lower than Point-LIO, 67.2% lower than FAST-LIO2, and 15.9% lower
than fixed Voxel-SLAM. All nine v17 runs completed; maximum processing RTF was
`0.83623`, maximum peak RSS was `274.50 MB`, and the frontend-only experiment
accepted zero loop edges.

Important limitations:

- These datasets became exposed development data after the SOTA-v5 evaluation;
  this is repeatable research evidence, not a fresh blind benchmark.
- v17 loses Oxford to GLIM by about 23% and narrowly loses NavINST to fixed
  Voxel-SLAM. The aggregate win must not be presented as a per-sequence sweep.
- The hundreds-of-metres UrbanNav APE values mean every compared system still
  struggles badly with the tunnel's longitudinal weak axis.
- v17 is a research frontend with its built-in loop closure/HBA disabled. It
  has not passed the map-geometry and fresh-holdout gates required for a SOTA
  claim and is not shipped as the default public workflow.

The reproducible candidate is fixed Voxel-SLAM revision
`70fc8a28d63823d5989ff184daeea0787b672398` plus
`weak_axis_bounded_map.patch` (SHA-256
`62f6e1c5d055106b08b4037267f6b6ac7d8b1c06757719b763b7107c47795b25`).
The frozen run configuration used replay rate `1.2` and CPU set `2-7`.

## Current Default Position

The public `v0.2.2` position is:

- default workflow: `RKO-LIO + graph_based_slam`
- public Autoware entrypoint: `bash scripts/run_autoware_quickstart.sh`
- release gate (legacy): `bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10`
- release gate (`v0.3`): `bash scripts/run_release_readiness_checks.sh --fail-on-profiles`
  using `scripts/release_profiles.yaml` (per-dataset pass/target thresholds)
- map-cleanup benchmark: `bash scripts/run_dynamic_object_filter_benchmark.sh`
- classic-path suite: `bash scripts/run_open_data_classic_path_benchmark_suite.sh`
- place-recognition suite: `bash scripts/run_place_recognition_benchmark.sh`
- KITTI Odometry dev split: `bash scripts/run_kitti_00_05_07_report.sh`
- research-track MID360 default tuning (kept for parity with prior numbers):
  `voxel_size=0.5`, `max_range=80.0`, `search_submap_num=5`,
  `loop_edge_dedup_index_window=20`, `loop_edge_info_weight=200`

## Interpretation

Safe claims:

- the default path is benchmarked on `NTU VIRAL` and reports on `MID-360`
- the pointcloud-map flow is dogfooded into Autoware end-to-end (map loaders)
- the repository already provides reusable comparison artifacts for
  dynamic-filtering, classic-path open-data runs, and place-recognition
- the release gate is now data-aware (per-dataset pass/target thresholds) so
  hard datasets (`MID-360`, `NTU VIRAL`) can be reported without being
  forced to one global APE threshold
- the built-in GPL-free `Scan Context` path is now benchmarked and improves the
  fair current-code `MID-360` rerun baseline, but it is still documented as
  opt-in
- the experimental submap-BEV path currently works better as a
  distance-candidate rerank than as a standalone loop source

Unsafe claims:

- that this repo is already the universal winner on every dataset
- that this repo should be judged primarily as a localization-research stack
- that the current default path is fully validated against every aggressive
  motion edge case
- that the `MID-360` research-track number (3.5–4.0 m vs GLIM) is anywhere
  near production accuracy on solid-state LiDAR

## Release Scope Reminder

`v0.2.2` is a public `v2 beta` release for:

- ROS 2 pointcloud-map generation
- non-GPL default workflow
- Autoware pointcloud-map loading

`v0.3` (in flight on `develop`) extends this with:

- Autoware-compatible lanelet2 auto-generation + multi-segment routing
  validation (`scripts/simple_lanelet2_generator.py --validate-structure`)
- dataset-profile release gate (`scripts/release_profiles.yaml`)
- KITTI Odometry t_rel / r_rel drift metric and 00/05/07 dev-split aggregator
- opt-in NIS-driven auto-scale for `adjacent_edge_info_weight`

`MID-360` and other solid-state LiDAR datasets are explicitly research track
until `v0.4`; they are reported but do not block release.
