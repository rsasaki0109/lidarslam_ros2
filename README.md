# lidarslam_ros2

[![CI](https://github.com/rsasaki0109/lidar_slam_ros2/actions/workflows/main.yml/badge.svg?branch=develop)](https://github.com/rsasaki0109/lidar_slam_ros2/actions/workflows/main.yml)
[![License: BSD-2-Clause](https://img.shields.io/badge/License-BSD--2--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![ROS 2: Humble | Jazzy](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy-22314E?logo=ros&logoColor=white)](#support-and-license)
[![GitHub stars](https://img.shields.io/github/stars/rsasaki0109/lidar_slam_ros2?style=flat&logo=github)](https://github.com/rsasaki0109/lidar_slam_ros2/stargazers)

**Turn a rosbag into a map you can actually drive on.**

ROS 2 LiDAR SLAM that outputs an Autoware-ready map bundle — `pointcloud_map/`,
`map_projector_info.yaml`, and auto-generated lanelet2. Frontend is `RKO-LIO` (MIT), backend is
`graph_based_slam` (BSD-2). No GPL components on the default workflow.

![Point cloud map built by this stack (Shinjuku demo bag)](lidarslam/images/map.png)

*Shinjuku point cloud map built from a demo rosbag with this stack — start at the
[Quickstart](#quickstart). `develop` is the default branch; latest release notes:
[v0.6.0](docs/releases/v0.6.0.md).*

## Why lidarslam_ros2

Most LiDAR SLAM stacks stop at a trajectory and a point cloud. This one ships the
artifacts you need downstream:

- **Autoware-ready output** — `pointcloud_map/` + `map_projector_info.yaml` open
  directly in Autoware map loaders; `verify_autoware_map.py` prints
  `map_verify: PASS` on every saved bundle.
- **lanelet2 auto-generation** — drivable lanelets from the SLAM trajectory,
  validated for multi-segment Autoware routing.
- **Surveyed ground truth** — releases are gated in CI by per-dataset APE
  thresholds, including total-station checkpoints on a Livox MID-360
  ([accuracy](#accuracy)).
- **Loop closure, GPL-free** — opt-in built-in Scan Context, BEV / SOLiD /
  STD/BTC-style Triangle descriptors, and 3D-BBS verification.
- **Deterministic offline mapping** — `graph_slam_offline_runner` (backend,
  recorded odometry bag) and `scan_matcher_offline_runner` (frontend, raw bag)
  produce *byte-identical* trajectories, loop edges and submaps; the release
  gate enforces both.
- **Globally refined, quality-gated maps** — clean-room plane bundle adjustment
  refines submap poses under holdout-validated quality thresholds
  ([evidence](docs/research/map-quality-baseline.md)).
- **GNSS georeferencing** — optional GNSS constraints and projector metadata for
  real-world coordinates.
- **Camera-coloured point-cloud maps** — synchronized images are projected onto
  registered LiDAR scans with calibration-aware, occlusion-resistant colouring.

```mermaid
flowchart LR
    bag(["rosbag2"]) --> rko["RKO-LIO<br/>LiDAR-inertial odometry"]
    rko --> gbs["graph_based_slam<br/>loop closure + graph optimization"]
    gbs --> bundle["Autoware map bundle<br/>pointcloud_map · lanelet2 · projector info"]
```

## Quickstart

For the shortest automatic path, run `bash scripts/run_first_map.sh`; it checks the
environment and selects source or Docker. See [Getting Started](docs/getting-started.md), or add `--dry-run`.

### Try it with Docker (one command, no build)

```bash
docker run --rm -v "$PWD/lidarslam_output:/lidarslam_ws/output" \
  ghcr.io/rsasaki0109/lidar_slam_ros2:humble
```

This downloads a public 517 MB Livox MID-360 bag and runs the full headless
pipeline. The Autoware bundle and `traj_corrected.tum` appear under
`./lidarslam_output/mid360_demo/`; the [quickstart](docs/getting-started.md)
shows caching and interactive-shell options.

### Build from source

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/rsasaki0109/lidar_slam_ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

If you cloned without `--recursive`: `git -C src/lidar_slam_ros2 submodule update --init --recursive`.

Then run one public dataset end to end — NTU VIRAL `tnp_01` (~580 s outdoor bag)
through RKO-LIO + graph_based_slam into an Autoware-loadable map:

```bash
cd src/lidar_slam_ros2
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_autoware_quickstart.sh
python3 scripts/verify_autoware_map.py output/.../pointcloud_map
```

## Use your own bag

```bash
bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2
```

One command turns the bag into a complete Autoware map bundle:
`pointcloud_map/` tiles, `map_projector_info.yaml`, and a `lanelet2_map.osm`
generated from the loop-closed trajectory.

Direct launch commands, required topics, GNSS / IMU pre-integration, and
dynamic-object filtering are in [docs/workflows.md](docs/workflows.md).

![Autoware map loaders rendering a pointcloud_map authored by this stack](lidarslam/images/autoware_map_loader_proof.png)

## Camera-coloured point-cloud maps

The pipeline registers LiDAR scans with the corrected trajectory, then projects
synchronized camera pixels onto that geometry. This RTK-SLAM Construction Hall
1 result follows the full estimated 60 m walking loop.

![Camera-coloured SLAM point-cloud map and its estimated trajectory](lidarslam/images/map_flythrough_rtkslam.webp)

The sequence is from the RTK-SLAM dataset (CC-BY 4.0). Its total-station
checkpoints are also used by the [accuracy gate](#accuracy).

If graph optimization outputs sparse keyframes, the coloured-map pipeline can
propagate their corrections onto the dense SLAM pose stream automatically:

```bash
python3 tools/gaussian_splatting/colored_map_pipeline.py \
  <bag> output/<run>/traj_corrected.tum output/<run>/colored_map \
  --raw-traj output/<run>/traj_raw.tum \
  --extrinsic configs/gaussian_splatting/<lidar_camera_extrinsic>.yaml
```

The pipeline caches `dense_corrected_trajectory.tum` and rebuilds stale
downstream artifacts; use `--force-trajectory` for an explicit refresh.

### Cross-repository SLAM benchmark

`public_suite_v1.yaml` applies trajectory, geometry, RGB, runtime, and memory
gates to frozen OFF/ON manifests from MID-360, HILTI, and RTK-SLAM. Reproduction
commands and both accepted and rejected experiments are in
[Benchmarking and release gate](docs/benchmarking.md#slam-candidate-regression).

## Open-source benchmark results

On the same HILTI 2022 `exp04` LiDAR/IMU input and CPU-only host, the competitive
profile recorded **34.7% lower median APE RMSE than GLIM** over three runs:

| System | Median APE RMSE | Median processing RTF | Maximum peak RSS |
| --- | ---: | ---: | ---: |
| **lidarslam_ros2** | **0.0565 m** | 0.993 | **586.83 MB** |
| GLIM CPU | 0.0866 m | **0.244** | 690.88 MB |

Reproduce one `lidarslam_ros2` sample with:

```bash
python3 scripts/run_ours_competitive_benchmark.py \
  --bag <hilti-exp04-ros2-bag> \
  --reference-tum <common-six-checkpoints.tum> \
  --reference-meta <common-six-checkpoints.json> \
  --rko-param configs/hilti2022/rko_lio_hilti2022_pandar_competitive_v2.yaml \
  --lidarslam-param configs/hilti2022/lidarslam_competitive_v2.yaml \
  --output output/hilti_exp04_ours_n1 --runs 1
```

The GLIM CPU counterpart is:

```bash
python3 scripts/run_glim_benchmark.py \
  --bag <hilti-exp04-ros2-bag> \
  --output output/hilti_exp04_glim_n1 --runs 1
```

Exact scoring rules, revisions, checkpoint policy, map-quality limitations, and
the historical evidence are documented in
[Comparison](docs/comparison.md#same-input-hilti-2022-exp04-vs-glim-cpu).

The separate Voxel-SLAM `v17` research candidate also achieved the lowest
geometric-mean APE across NavINST, Oxford, and UrbanNav: **2.2836 m**, versus
GLIM `5.0779`, Point-LIO `3.8388`, FAST-LIO2 `6.9576`, and fixed Voxel-SLAM
`2.7160` — **55.0% lower than GLIM**. It is not the default release path, does
not win every sequence, and is not fresh-blind evidence. Exact revisions,
per-sequence results, input hashes, resource results, map limitations, and
reproduction notes are in [Comparison](docs/comparison.md#voxel-slam-v17-research-candidate-vs-pinned-oss-rivals).

<!-- BEGIN GENERATED COMPETITIVE CLAIM PUBLICATION -->
<!-- END GENERATED COMPETITIVE CLAIM PUBLICATION -->

## Accuracy

Release-gate thresholds ([Benchmarking](docs/benchmarking.md#release-gate-accuracy-snapshot)) block every release in CI.

## Docs

- **Getting started**: [Getting Started](docs/getting-started.md) · [Autoware quickstart](docs/autoware-quickstart.md) · [Operator workflows](docs/workflows.md) · [Autoware Foxglove](docs/autoware-foxglove.md)
- **Pipelines**: [Autoware-compatible map authoring](docs/autoware-map-authoring.md)
- **Benchmarking**: [Benchmarking and release gate](docs/benchmarking.md) · [Comparison](docs/comparison.md)
- **Project**: [v0.2.2 release notes](docs/releases/v0.2.2.md) · [Contributing](CONTRIBUTING.md) · [Changelog](CHANGELOG.md) · [Releasing](RELEASING.md)

Preview the doc site locally: `python3 -m mkdocs serve`.

## Support and license

| ROS 2 distro | Ubuntu | Scope |
| --- | --- | --- |
| Humble | 22.04 | default workflow build + package tests in CI |
| Jazzy  | 24.04 | default workflow build + package tests in CI; Autoware dogfood exercised locally |

`graph_based_slam` is BSD-2-Clause; `RKO-LIO`, `DLIO`, and the optional vendored
`3D-BBS` are MIT; `FAST_GICP` is BSD-3-Clause; built-in Scan Context is local. GPL-only
components (`Thirdparty/lio-sam`, `Thirdparty/3d_bbs`) are excluded via `COLCON_IGNORE`.

## Quality gates

```bash
bash scripts/run_default_ci_checks.sh
bash scripts/run_release_readiness_checks.sh --ape-threshold 0.10
```

Reference commands and parameter pointers live in [docs/workflows.md](docs/workflows.md).

---

If this project saves you mapping time, a ⭐ helps others find it.
