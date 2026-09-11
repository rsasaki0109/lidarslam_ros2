# Autoware Quickstart

This is the advanced viewer/dogfood compatibility path from `lidarslam_ros2` to
an Autoware pointcloud map shown in `rviz2`. For the beginner first-map result,
use the fixed, headless [Getting Started](getting-started.md#2-run-the-fixed-first-map-demo)
command; Docker and source use the same MID-360 dataset and output contract.

If you want the product-level overview first, see
[Autoware-Compatible Map Authoring](autoware-map-authoring.md).
If you are choosing between Docker, source build, or your own bag, start with
[Getting Started](getting-started.md).
For the optional browser-based viewer path, see
[Autoware Foxglove](autoware-foxglove.md).

For a normal own-bag map, use the product entrypoint:

```bash
lidarslam-map start /path/to/rosbag2
```

It inspects the bag, confirms and saves the selected inputs, verifies the map,
and opens the offline browser result. RKO-LIO additionally requires explicit
calibration confirmation. PointCloud2+GNSS and packet+Applanix bags use the
same entrypoint. The commands below are retained for advanced compatibility
and debugging.

To run only the bag preflight:

```bash
python3 scripts/preflight_autoware_map_bag.py /path/to/rosbag2
```

If you want the repository to pick and run the shortest supported path from the
same preflight result, use:

```bash
python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2
```

For Livox/MID360-style bags, this runner automatically switches to the tracked
MID360 preset and writes `verify_autoware_map.log` plus a diagnosis report into
the output directory.

The advanced public entrypoint for this viewer/dogfood flow is:

```bash
bash scripts/run_autoware_quickstart.sh
```

## Scope

This quickstart covers pointcloud maps only.

- `lanelet2_map.osm` is out of scope.
- Without GNSS the generated `map_projector_info.yaml` uses
  `projector_type: Local`.
- With GNSS enabled and a stable origin, the same file uses
  `projector_type: LocalCartesian` plus `map_origin`.

## Prerequisites

- ROS 2 workspace builds successfully.
- Host has `docker`, `ros2`, `rviz2`, and a working X11 `DISPLAY`.
- For the Docker viewer flow, an `autoware_core` checkout is available.
- For the bundled NTU VIRAL dogfood path, prepare the sample bag once with
  `bash scripts/download_ntu_viral_tnp01.sh`.

Recommended build check:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
bash scripts/run_default_ci_checks.sh
```

## Fastest Paths

### 1. Run the advanced fixed public quickstart

If you need the legacy Autoware viewer/dogfood route:

```bash
bash scripts/download_ntu_viral_tnp01.sh --dry-run
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_autoware_quickstart.sh
```

This runs the bundled NTU VIRAL dogfood path with a bounded viewer lifetime.
Under the hood it forwards to `run_rko_lio_graph_autoware_dogfood.sh`.
The first command writes nothing and stops a fresh multi-gigabyte preparation
from starting on an undersized filesystem; use its suggested external
`--dest` when needed.

### 2. Open an existing graph_based_slam output in Autoware

If you already have a saved `graph_based_slam` run under `output/...`:

```bash
bash scripts/run_autoware_quickstart.sh \
  output/bench_rko_lio_ntu_viral_loopgate_20260324
```

This stages the bundle under `/tmp/autoware_maps/<run_name>`, verifies it, and
opens the pointcloud map through Autoware's Dockerized map loaders. The wrapper
for this path is `run_graph_slam_pointcloud_map_in_autoware.sh`.

### 3. Use the explicit dogfood subcommand

If you want to stay on the public wrapper but pass dogfood-specific options:

```bash
bash scripts/run_autoware_quickstart.sh dogfood \
  --wait-for-offline-completion \
  --auto-exit-secs 20
```

### 4. Stage a run manually and smoke-test the map loaders

If you want to inspect the staged bundle before opening `rviz2`:

```bash
bash scripts/prepare_autoware_map_from_graph_slam.sh \
  output/bench_rko_lio_ntu_viral_loopgate_20260324 \
  /tmp/autoware_maps/ntu_viral_loopgate \
  --smoke \
  --autoware-core-dir /tmp/autoware_core \
  --work-dir /tmp/autoware_map_runtime_ws
```

Then open the viewer:

```bash
bash scripts/run_autoware_pointcloud_map_viewer_docker.sh \
  /tmp/autoware_maps/ntu_viral_loopgate \
  /tmp/autoware_core \
  /tmp/autoware_map_runtime_ws
```

## What Success Looks Like

You should see all of the following:

- `python3 scripts/verify_autoware_map.py <map_dir>` reports `RESULT: PASS`
- Docker-side Autoware map loaders start without early exit
- host-side `rviz2` subscribes to `/map/pointcloud_map`
- `frame_id` is `map`

Useful outputs:

- staged map bundle: `/tmp/autoware_maps/<name>`
- saved SLAM run: `output/dogfood_rko_lio_autoware_<timestamp>/`
- Autoware smoke logs: inside the chosen runtime work directory

## Troubleshooting

### RViz opens but no pointcloud appears

- Confirm `DISPLAY` works on the host.
- Confirm Docker can reach the host X server.
- Re-run with `--viewer-rebuild` if the local runtime image is stale.

### Map bundle fails verification

Run:

```bash
python3 scripts/verify_autoware_map.py /path/to/pointcloud_map
```

Common causes:

- float tile coordinates in `pointcloud_map_metadata.yaml`
- missing `map_projector_info.yaml`
- orphan or missing PCD tiles

### GNSS map origin does not appear

This is expected when GNSS is disabled. The map stays valid for Autoware with:

```yaml
projector_type: Local
```

### A run finished but the result still looks suspicious

Write a short diagnosis report from the saved output directory:

```bash
python3 scripts/diagnose_autoware_map_run.py output/<run_dir> --write
```

This summarizes launch status, verify results, projector metadata, and common
failure hints such as TF issues or missing GNSS edges.

## Related Commands

- bag preflight: `python3 scripts/preflight_autoware_map_bag.py /path/to/rosbag2`
- beginner wrapper: `bash scripts/run_autoware_map_beginner.sh /path/to/rosbag2`
- one-shot runner: `python3 scripts/run_autoware_map_from_bag.py /path/to/rosbag2`
- public Autoware entrypoint: `bash scripts/run_autoware_quickstart.sh`
- benchmark path: `bash scripts/run_rko_lio_graph_benchmark.sh`
- release gate: `bash scripts/run_release_readiness_checks.sh --fail-on-profiles`
- map-only verify: `python3 scripts/verify_autoware_map.py <pointcloud_map_dir>`
- run diagnosis: `python3 scripts/diagnose_autoware_map_run.py <output_dir>`
