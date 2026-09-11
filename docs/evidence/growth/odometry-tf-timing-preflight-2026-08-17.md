# Odometry TF timing preflight — 2026-08-17

## Decision

Extend the existing read-only `lidarslam-map doctor <rosbag2_dir>` contract
with replay-order timing evidence. Do not change scan matching, suppress TF
warnings, increase a lookup timeout, or substitute a stale transform.

This increment addresses the diagnosis burden in still-open
[Issue #64](https://github.com/rsasaki0109/lidar_slam_ros2/issues/64). The
reported example requested a transform at 20.364 s while the latest transform
was at 20.346 s. A later reporter eliminated the warning and path drift only
after raising the *actual* Odometry/TF update rate. The issue and its comments
were read only; no GitHub issue state was changed.

## Preflight v6 contract

Preflight schema v6 keeps v1–v5 immutable. The existing bounded connectivity
scan now publishes the selected path edges and whether each is dynamic. When a
PointCloud2 topic and a usable dynamic Odometry path both exist, doctor makes
one additional sequential bag pass, bounded to 100,000 records per selected
topic. It:

- preserves rosbag reader order as the playback-order sample basis;
- tracks the greatest valid `header.stamp` observed so far for every required
  dynamic path edge;
- counts clouds seen before all dynamic edges as startup gaps;
- counts every cloud stamp newer than the limiting latest TF stamp as a future
  extrapolation gap, without an arbitrary time threshold; and
- reports the first affected cloud, largest gap, limiting edge, bounded record
  counts, and completeness.

The Issue #64 regression fixes the values at 20.346 s and 20.364 s and requires
an exact maximum gap of 18,000,000 ns. Multi-hop coverage verifies that the
oldest required dynamic edge limits the result; static path edges are ignored
for this timing comparison.

Stable findings keep a compatible maintained mapping recommendation visible:

- `odometry-tf-startup-gap`
- `odometry-tf-future-gap`
- `odometry-tf-timing-invalid`
- `odometry-tf-timing-inspection-unavailable`

## Claim boundary

This is recorded-bag, reader-order, header-stamp evidence. A clean result says
that no sampled cloud was newer than the latest required dynamic TF stamp seen
earlier in that bag pass. It does not prove live executor order, DDS latency,
clock alignment, TF buffer duration, past-side availability, interpolation,
publisher rate, calibration, mapping success, trajectory accuracy, or support
for a new robot. Operators must still rerun `tf2_monitor` on the live system.

The command remains read-only: it does not modify a bag, message, parameter,
publisher, transform, launch file, issue, or external service.

## Local verification

Exact implementation `4bdd7ec86b4cfa6e566d6beda41f5c5fe28537f9`
passed:

- 74 focused preflight, diagnosis, and docs-entrypoint regressions, including
  both serialized Jazzy rosbag2 reader cases;
- 1,489 graph tests with 13 known skips and 11 existing ImageIO warnings;
- 1,040 lidarslam tests, for 2,529 passed package tests in total;
- strict MkDocs and changed-file Jazzy `ament_flake8`;
- G0 `HOLD`, v1 readiness 8 / 10, and `PLAN_VALID_LOCAL_ONLY` with 321
  paths, seven slices, and digest
  `f378fd654e54595dce5da1e5763374f3f56db1dcd87513ab3c5e4de6afcbdbd5`;
- two byte-identical and reverified v0.9.1 candidate bundles with 271 files,
  11,965,449 bytes, and SHA-256
  `735a3683be43cfb2e2638e466b02b140339beb9da573bc5642872219090fc6a9`;
  and
- the exact-head strict release-profile gate failing closed with the five
  expected `NO_DATA` rows: Newer College Maths, NTU VIRAL TnP 01, RTK-SLAM
  Construction Seq2, RTK-SLAM Construction Seq1, and Leo Drive.

These values are commit-bound local evidence. A later evidence-sync commit and
public Draft head must rerun exact-head checks rather than inherit this bundle
identity.
