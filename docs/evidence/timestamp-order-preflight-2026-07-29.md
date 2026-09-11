# Timestamp-order preflight evidence — 2026-07-29

Status: **PASS for clean-input acceptance and derived reversal rejection**.

This execution validates preflight schema v3 against the pinned public
MID-360 product input and a small bag derived from its real serialized
PointCloud2/Imu records. It was run from clean implementation commit
`85296ad85ffc0a0088761d026a22618bb0539883`.

## Environment and input identity

| Field | Value |
| --- | --- |
| Host | Ubuntu 24.04.4 LTS, amd64 |
| Python | 3.12.3 |
| Reader | `rosbags` 0.11.0 fallback |
| Schema validator | `jsonschema` 4.10.3 |
| Clean dataset contract | `driving_slam_mid360_v1` |
| Clean metadata SHA-256 | `65d66875f49248e38ff14d80e6e749fb50606f6f80bd4be337160e3752691e9a` |
| Clean sqlite3 SHA-256 | `3bbd390a97e57af47ad6699baa36eb4c5f39f61b35275505ecaf221c126354f5` |
| Derived metadata SHA-256 | `52dc731d5677d1e32ef59b473445aaaa286f9a22ae65a3a5fefcc94d7ab794d0` |
| Derived sqlite3 SHA-256 | `ca634fdd8544a4451829be93a224e4b90004c7cc686c83ec1fa7cc674aa2e79d` |

The derived fixture retains 12 `/livox/lidar` PointCloud2 records and 120
`/livox/imu` records from the clean input. Its sixth LiDAR record keeps the
original rosbag2 receive timestamp but replaces `header.stamp` with the
previous LiDAR header stamp minus 0.75 seconds. This isolates the mapper-visible
header reversal from storage order.

## Results

| Assertion | Clean public bag | Derived reversal bag |
| --- | ---: | ---: |
| Preflight schema | v3 PASS | v3 PASS |
| Duration | 1.256551 s | 0.054486 s |
| LiDAR records | 2,772 / 2,772 complete | 12 / 12 complete |
| IMU records | 55,435 / 55,435 complete | 120 / 120 complete |
| Reversals | 0 | 1 on `/livox/lidar` |
| Largest backward jump | 0 ns | 750,000,000 ns |
| Invalid stamps | 0 | 0 |
| Timestamp status | `passed` | `failed` |
| Recommended profile | `rko_lio_graph_public_path` | none |

The clean run produced no missing requirements. The derived run emitted an
actionable requirement naming `/livox/lidar`, one reversal and the exact
0.750000000-second maximum backward jump. No ROS mapping process was started.

## Cross-reader revalidation

Clean commit `05fefe8c798eb20d24183d66d769265b66a6b062` was then run with the
Jazzy `rosbag2_py` reader against the same pinned clean bag. Schema v3
validation passed in 2.705229 seconds, selected
`rko_lio_graph_public_path`, and reproduced the complete 2,772-record LiDAR
plus 55,435-record IMU result with zero reversals and zero invalid stamps.
This verifies that the installed ROS reader and documented pure-Python
fallback agree on the named real input.

## Contract covered

- the selected PointCloud2 and Imu streams are inspected, rather than every
  research topic;
- non-decreasing header stamps pass, while a decreasing stamp fails;
- the 100,000-record per-topic bound is explicit in the machine contract;
- complete and bounded-sample results remain distinguishable;
- `failed`, `error`, and `unavailable` inspection states fail closed for
  timestamp-dependent profiles;
- preflight v1 and v2 remain published for existing automation.

## Limits

- The first clean and derived executions used the pure-Python fallback.
  Jazzy `rosbag2_py` was revalidated on the clean input; Humble behavior
  remains covered by installed-CLI CI rather than this named local execution.
- A `sampled` result cannot prove monotonicity after the 100,000th selected
  record. It is intentionally not reported as `passed`.
- The derived input proves detection and pre-launch refusal, not automatic
  repair. Operators must correct their source or run the documented stamp
  rewriter and repeat `doctor`.
