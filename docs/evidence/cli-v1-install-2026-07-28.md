# CLI v1 installed-prefix validation — 2026-07-28

## Decision

The CLI v1 option changes passed a non-symlinked, merged-prefix install check
on both supported ROS distributions. This validates the installed command
layout and delegated Python resources before pull-request CI.

It is installation evidence for the CLI change, not a full clean-machine
source build or a graphical viewer acceptance test.

## Frozen inputs

| Input | Value |
| --- | --- |
| Source revision | `48eb02d4a207d449b49f29531aca37ea4e75c247` |
| Source mount | Read-only at `/repo` |
| Build scope | `lidarslam` package against the dependencies installed in each published product image |
| Install shape | `colcon build --merge-install`, without `--symlink-install` |
| Humble environment | `ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:9db1a467c99d69bd3a6d8d7a71e6555874f2a0e1e6f7d062ab2297dd7828c061` |
| Humble environment revision | `1fe7e20ce4f1cc60b07eb5066d1347e96abe2bf6` |
| Jazzy environment | `ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:7b27bdc109c25a7881a884128a91708c2a3e431e776c02b066ec7e33d04b0f1c` |
| Jazzy environment revision | `589b0a303d7bc45f914e8da06ae40ba733e409b2` |

The validation command inside each pinned image was:

```bash
source /lidarslam_ws/install/setup.bash
mkdir -p /tmp/cli_ws
cd /tmp/cli_ws
colcon --log-base log build \
  --merge-install \
  --base-paths /repo/lidarslam \
  --build-base build \
  --install-base install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
python3 /repo/scripts/check_installed_product_cli.py \
  --prefix /tmp/cli_ws/install
```

## Results

| Distribution | Package build | Installed CLI validation | Result |
| --- | ---: | ---: | --- |
| Humble | 16.4 s | Passed | Pass |
| Jazzy | 17.3 s | Passed | Pass |

The installed-prefix validator checked:

- `lidarslam-map`, the `lidarslam-cli` ROS shim, and the historical
  `lidarslam` node as distinct executables;
- the curated runtime manifest and every delegated script, including
  `view_autoware_map.py`;
- `--version` through the PATH command and ROS shim;
- `doctor --json` against a generated PointCloud2 + Imu rosbag2;
- an own-bag `run --dry-run` from an unrelated working directory;
- `inspect --json`;
- `view --help` and rejection of an incomplete output before viewer launch;
- installed Bash-completion presence and syntax.

Jazzy emitted existing developer warnings for CMake policies CMP0144 and
CMP0072 and a PCL deprecated-header note. They did not change the build or
validation result.

## Follow-up: operator and full help modes

Commit `9d13735d69c5d2148b68f13b10980a10a70ba0df` organized the same
compatible option inventory into two stable help modes:

- `<command> --help` shows routine stable operator options;
- `<command> --help-all` also shows advanced viewer-runtime controls and
  deprecated compatibility aliases.

The source was mounted read-only at `/repo` and the `lidarslam` package was
again installed into a new non-symlinked merged prefix inside each pinned
image listed above. The extended validator proved that normal `view` help
does not expose `--runtime-dir`, full help does expose it, and both modes work
from the installed command.

| Distribution | Package build | Extended installed validator | Result |
| --- | ---: | ---: | --- |
| Humble | 16.6 s | Passed | Pass |
| Jazzy | 17.4 s | Passed | Pass |

The repository checks also matched both rendered help modes against
`docs/contracts/cli-v1.json`, validated Bash completion, passed 59 focused CLI
tests and 9 documentation tests, and built the MkDocs site in strict mode.
The Jazzy `lidarslam` package test completed 146 tests with zero failures and
one skip, including ament flake8, lint, PEP 257 and XML checks.

## Limitations

- Only the changed `lidarslam` package was rebuilt. The published images
  supplied already-built dependency packages.
- The test validated viewer command dispatch and preflight behavior, but did
  not launch RViz or Foxglove in the headless containers.
- Pull-request CI must still build the repository matrix and run the complete
  default and Docker checks.
- This maintainer-run evidence does not count as independent-user first-map
  validation.
