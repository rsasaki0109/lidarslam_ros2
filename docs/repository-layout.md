# Repository layout

The repository root is reserved for project metadata and primary entry-point
documents. Runtime code, generated evidence, and one-off recovery utilities do
not belong at the root.

| Path | Contents |
| --- | --- |
| `lidarslam/` | Integrated ROS 2 launch package |
| `scanmatcher/` | LiDAR odometry frontend |
| `graph_based_slam/` | Loop closure and pose-graph backend |
| `lidarslam_msgs/` | ROS 2 interfaces |
| `lidarslam_*_plugins/` | Registration plugin interfaces and implementations |
| `scripts/` | Maintained command-line workflows and automation |
| `scripts/recovery/` | Historical or incident-specific recovery commands |
| `tools/` | Offline processing libraries and specialized tooling |
| `configs/` | Versioned runtime and benchmark configurations |
| `docs/research/` | Research records and historical plans |
| `docs/research/artifacts/` | Small, reviewable research-result artifacts |
| `tests/` | Repository-level research-contract tests |

Build products such as `build/`, `install/`, `log/`,
`symlink_install_manifest.txt`, generated maps, and benchmark run directories
must remain untracked. New Python programs belong under `scripts/`, `tools/`,
or a Python package; JSON evidence belongs beside its owning contract or under
`docs/research/artifacts/`.
