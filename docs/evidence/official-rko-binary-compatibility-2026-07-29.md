# Official RKO-LIO binary compatibility — 2026-07-29

Status: **PASS**

The release-shaped product was built without the RKO-LIO source submodule
and run against the official `0.3.2-1` testing binaries on both supported
ROS distributions. Each job used the pinned public MID-360 dataset and the
maintained `rko_lio_graph_mid360_preset` golden path.

| ROS distro | Official Debian build | E2E checks | Runtime | Raw / corrected poses | Tiles |
| --- | --- | ---: | ---: | ---: | ---: |
| Humble | `0.3.2-1jammy.20260728.162644` | 18 / 18 | 90.271 s | 2569 / 574 | 360 |
| Jazzy | `0.3.2-1noble.20260728.162143` | 18 / 18 | 94.139 s | 2572 / 575 | 358 |

The gate proved that `offline_node` resolved from `/opt/ros/<distro>`,
recorded the executable SHA-256, reached terminal success, passed the
Autoware map diagnosis with 8 passes and 0 failures, and met all trajectory
and pointcloud evidence thresholds. No map geometry, trajectories, or point
clouds were uploaded.

Permanent machine-readable evidence is stored in
[`official-rko-binary-compatibility-2026-07-29.json`](official-rko-binary-compatibility-2026-07-29.json).
The originating public workflow is
[run 30412938777](https://github.com/rsasaki0109/lidar_slam_ros2/actions/runs/30412938777).

This closes official-binary compatibility for the maintained MID-360
golden path only. Fork-only research profiles remain evaluation scope.
Normal apt installation still waits for `0.3.2` to sync from ROS testing to
main and for `ndt_omp_ros2` to be released.
