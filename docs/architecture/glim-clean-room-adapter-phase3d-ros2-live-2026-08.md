# GLIM clean-room adapter Phase 3d: Jazzy live ROS 2 boundary

This is an additive, opt-in transport candidate.  It is not part of the
active benchmark profile and it does not change the retained Phase 0--3c
receipts or the r2 selection.  Humble is not claimed: this host has Jazzy
underlay support, while `/opt/ros/humble` is absent.

## Boundary and ownership

`phase3d/src/phase3d_node.cpp` is the only ROS transport implementation.  It
uses the host-owned `ros2_conversion.hpp` and `pointcloud2_parser` and the
Phase 3b `GlimCoreSession`; it does not use an upstream ROS bridge.  The
production source guard scans Phase 3d as well as the earlier clean-room
surfaces and rejects bridge provenance tokens.

The node owns copied message payloads in a bounded table.  The ordering queue
contains only `(signed int64 nanoseconds, host arrival ordinal, kind)` keys.
Messages are sorted by timestamp and then the host-assigned ordinal.  A
nonzero `reorder_window_ns` is a bounded holdback window; EOF is the only
operation that force-flushes the remainder.  A late item behind an already
submitted timestamp, queue exhaustion, duplicate ordinal, invalid timestamp,
payload-byte bound, allocation failure, or post-EOF input latches a terminal
error.  There is no silent application-level drop.  Input QoS is an explicit
parameter and defaults to reliable; best-effort is available only as an
explicit degraded transport setting because DDS loss cannot be detected
without sequence metadata.

`max_cloud_bytes` bounds one copied PointCloud2 and
`max_pending_payload_bytes` bounds all retained copied payloads.  Accounting
is checked before copying and released only after serialized processing.

## Lifecycle and output contract

The `std_srvs/Trigger` finalize service is the sole EOF API.  It performs one
request, bounded force-drain, exact-core `close`, output validation, and one
internal trajectory/map seal.  A second request is rejected.  Callbacks and
core exceptions are translated into a terminal latch; destruction is
idempotent and never detaches a worker.  Invalid frames, non-finite data,
calibration/frame mismatch, non-monotonic output, non-contiguous output order,
ROS stamp range overflow, quaternion shape errors, and PointCloud2 row-step
overflow fail before publication.

The internal seal is atomic: both required output messages are fully converted
before either publication call.  ROS 2 delivery of two separate topics is not
transactional; therefore the completion diagnostic is emitted only after both
`publish` calls return, and a consumer must use that diagnostic plus the
latched counters to distinguish transport delivery from internal completion.
Trajectory is published as `nav_msgs/Path`; the map is one bounded,
little-endian `sensor_msgs/PointCloud2` with explicit x/y/z/intensity/ring/time
fields.  Both outputs use the contract frame, and the map completion stamp is
the final trajectory stamp.

## Parameters that must be fixed before input

`config_directory`, `sequence_id`, `lidar_frame`, `imu_frame`,
`trajectory_frame`, `map_frame`, `point_time_unit`, both calibration transforms,
all six PointCloud2 field names/types, `require_map_output`, the QoS depth and
reliability, reorder window, payload limits, and all Phase 3b ledger/map
bounds are node parameters.  Phase 3b preflights the config directory as
regular, non-symlink files with relative, contained JSON component names;
mapping files are required only when `require_map_output=true`.

## Synthetic Jazzy evidence

The reproducible smoke uses only synthetic ROS messages (two LiDAR frames,
three IMU messages), the pinned Phase 1 exact-core prefixes, one CMake worker,
and `/opt/ros/jazzy`.  It checks equal-stamp ordering, bounded holdback,
negative/overflow ROS stamps, trajectory quaternion validation, row-step
overflow, exact output frames/stamps, one path plus one map publication,
double-finalize rejection, and post-EOF rejection.  No bag, dataset, GT,
scorer, benchmark, or active profile is used.

Final immutable evidence (the source manifest covers the Phase 3d tree and
source guard; documentation is kept outside that manifest to avoid a
self-hash cycle) is:

| mode | root | configure | build | CTest | install | node binary |
|---|---|---|---|---|---|---|
| normal | `/tmp/glim-clean-room-phase3d-final-normal.KTzM3F` | `d1b8645398685a747c9c838ac5a77e6453e21911635ce4be0651c194f5426896` | `8da45f7e7988fe5786eacc12170ea7d822034c2416772b53144b97e72009bf0f` | `6f6f97f390a9e650461ed3726d2b68180121ec96daf3bad1478b01343f89ae00` | `a2af85cf5e63760155db7397f22ffbfb129dbcafee036ad56f3b9b0df1a113e0` | `9a7d3bac00243b8caa61287917d5ac420e78e2617ea5c6b997cc8e8c3d51d7d0` |
| ASAN/UBSAN | `/tmp/glim-clean-room-phase3d-final-sanitize.jtcp6L` | `edf8adbd2fd6386cd6006469cd2f7586ccc437eac1d65827975600c04d006030` | `8da45f7e7988fe5786eacc12170ea7d822034c2416772b53144b97e72009bf0f` | `d2e4147160799a85261ff22552b398742e15b6f63ea46b7c7fb8936f45573645` | `7af766f0c634990de2d430f825a32a7c29842c96a3353727121589eeebe21ef0` | `c8a579d896af66f032e0bb99ea397989dea2ac6f2d91ef01779c28df23887a24` |

Both CTest runs are 3/3 with zero failures (10 gtest cases: 9 ingress/output
contract cases and 1 synthetic ROS graph case), and both installs contain the
node plus the nine host-owned headers.  The normal install audit (including
`ldd` and bridge-token scan) is `69cacdfb6288b1e9c8563f4e44a01de3b1e66c1537cfeb658915db6246e04c97`.
The deterministic source-manifest digest is
`9f809c32336bb97fbaad5665c8602752a28526062a444e94254fbca0b424cb0e`; normal
and sanitizer install-manifest digests are respectively
`e006ea60a5375ee35a001886111ad6514ff91f94b3002f7db483006f8e6542f5` and
`2a9ebdc1bfb7d53c9b12366c5941cec9993b085cbe6534da77c465e4538d3e24`.

The manifest is path-independent because it is generated from the repository
root with sorted relative paths and a second hash over the resulting
`sha256sum` lines (generated files are outside the selected paths):

```sh
cd /path/to/lidar_slam_ros2
{
  find docker/benchmark_adapters/glim_clean_room/phase3d -type f \
    -not -path '*/__pycache__/*' -printf '%p\n'
  printf '%s\n' docker/benchmark_adapters/glim_clean_room/tests/source_guard.py
} | LC_ALL=C sort | while IFS= read -r path; do sha256sum "$path"; done \
  > /tmp/phase3d-root/source-manifest.sha256
sha256sum /tmp/phase3d-root/source-manifest.sha256
```

A prior sanitizer root `/tmp/glim-clean-room-phase3d-sanitize.ijkRb2` is
retained as a superseded failure: CTest binaries were not linked with the
sanitizer runtime.  The CMake fix was limited to adding the same ASAN/UBSAN
flags to both test targets; the final sanitizer root above is the corrected
run.  A passing focused test is necessary but not sufficient for benchmark
eligibility; this candidate remains `NOT_READY` for a live production claim
and is not added to the active benchmark profile.

The install surface deliberately exports the node executable and all
host-owned headers, while keeping the exact-core static link targets internal
to this opt-in recipe.  A downstream process consumes the installed node;
downstream C++ linkage requires the separately pinned GLIM/GTSAM prefixes and
must be configured explicitly rather than inheriting stale build-tree paths.
