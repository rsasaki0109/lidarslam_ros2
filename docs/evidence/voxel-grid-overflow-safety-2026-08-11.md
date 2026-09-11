# Classic scanmatcher VoxelGrid overflow safety — 2026-08-11

> Status: **PUBLIC_HUMBLE_JAZZY_CI_PASS_ISSUE_AND_RELEASE_PENDING**
>
> Runtime safety commit:
> `a2368c486fc35c0edcac6d9dbf2f9cb89475c820`
>
> Component recovery proof commit:
> `bce5a9dd2f8f1333b92eba5a0ace98f45db58f3b`
>
> Rejected-map-update threshold recovery commit:
> `99cce93a07a7cc136eb925c446dd705bdcd7b37c`
>
> Public issue: [#69 — scanmatcher_node-1 process has died](https://github.com/rsasaki0109/lidar_slam_ros2/issues/69)
>
> Integration target: [Draft PR #427](https://github.com/rsasaki0109/lidar_slam_ros2/pull/427)
>
> Issue or release mutations made by this follow-up: **none**

## Decision

The classic scanmatcher must never use PCL's unfiltered-copy fallback as its
overflow behavior. Every explicit PCL `VoxelGrid` call is now behind one
fail-closed wrapper. Unsafe input clears the candidate output, emits a stable
reason and recovery action, and returns control without replacing the last
valid map or registration target. Valid input still uses PCL with the same
effective float leaf size; the bounded parity test produces exactly equal
XYZ/intensity points.

This closes the local code hazard and both local component-continuation gates.
The asynchronous follow-up also snapshots the triggering scan's distance for
the worker, locks shared map diagnostics, and joins an outstanding worker before
component destruction. The S1 review now commits the movement baseline only
after a map update succeeds. A rejected VoxelGrid layout or contained worker
exception therefore cannot consume the threshold needed by the next safe scan.
It does not by itself close public issue #69: the
historical private rosbag was not retained or replayed, the reviewed public CI
revision must carry both component cases plus the commit-state regression, and
a named release plus an accurate issue response are still required.

## Upstream behavior being contained

The official PCL
[1.12.1 implementation](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.12.1/filters/include/pcl/filters/impl/voxel_grid.hpp)
and
[1.14.1 implementation](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.14.1/filters/include/pcl/filters/impl/voxel_grid.hpp)
both calculate three grid dimensions, compare their product with signed
32-bit maximum, warn on overflow, assign the input cloud to the output, and
return. The warning is therefore not a filtered or rejected result.

Issue #69 records that warning immediately before scanmatcher exited with
`-8` (`SIGFPE`). The original parameters included `vg_size_for_map: 0.1` and a
`200 m` scan range; increasing the leaf size mitigated at least one later user
report but did not provide a safe runtime contract.

The wrapper is deliberately more conservative than the upstream span-only
check. It also rejects:

- a requested leaf that is non-finite, non-positive, not representable as a
  positive finite float, or has a non-finite float inverse;
- a cloud marked `is_dense=true` that contains non-finite XYZ;
- an absolute `floor(coordinate / leaf)` outside signed 32-bit range;
- the exact floored voxel-division product above `2,147,483,647`.

All multiplication is bounded before it occurs, so the preflight does not
introduce another signed-overflow path.

## Call-site behavior

Commit `a2368c4` removes every direct `pcl::VoxelGrid` instance from
`scanmatcher_component.cpp` and routes all five stages through
`filterVoxelGridSafely`.

| Stage | Leaf parameter | Rejection behavior | Preserved state |
| --- | --- | --- | --- |
| `initial_map` | `vg_size_for_map` | return `false`; wait for another usable scan | node, configuration, and publishers remain active |
| `input_scan` | `vg_size_for_input` | skip the scan before registration | current pose, path, map, and registration target |
| `map_update` | `vg_size_for_map` | return from synchronous or asynchronous update | existing map array and registration target |
| `registration_target` | `vg_size_for_input` | do not install the unsafe newly built target | previously installed registration target |
| `recovery_target` | `vg_size_for_input` | return `false` from target refresh | existing target and recovery state |

Recurring runtime warnings are throttled to one per five seconds at each macro
site. Initial-map refusal is not throttled because it is the immediate reason
mapping cannot initialize. No path automatically increases a leaf size or
clips coordinates; both would silently change map resolution or data.

## Privacy-safe bounded reproducer

The regression fixture contains only these two synthetic points:

```text
[-200.0, -200.0, -10.0]
[ 200.0,  200.0,  10.0]
```

With an effective `0.1 m` leaf, the conservative divisions are
`4001 x 4001 x 201`, or `3,217,608,201` cells. This exceeds PCL's signed
32-bit `2,147,483,647` limit and produces
`VOXEL_GRID_LAYOUT_OVERFLOW`. The wrapper empties a pre-populated output cloud
instead of copying these input points through.

The test is a failure-class reproducer, not a claim that these were the exact
bounds in the unavailable historical bag. No bag, map geometry, issue author,
comment author, local path, or private sensor data was copied into the
repository.

## Automated coverage

The new `test_voxel_grid_safety` suite has 11 cases:

1. valid output parity with direct PCL, including intensity;
2. the bounded issue #69 overflow class and empty rejected output;
3. the largest cubic integer layout below the PCL limit;
4. the first adjacent cubic layout above the limit;
5. negative voxel indices;
6. absolute signed-32-bit index overflow;
7. zero, negative, NaN, infinite, and float-underflow leaf sizes;
8. inconsistent dense/non-finite input;
9. valid non-dense input with PCL dropping non-finite XYZ;
10. empty, null, and all-non-finite input;
11. actionable diagnostic content and stable reason code.

Every classic component call site uses the tested wrapper. The component no
longer contains a direct `pcl::VoxelGrid` construction.

The separate `test_scanmatcher_voxel_grid_recovery` suite exercises the real
ROS 2 component rather than calling the wrapper directly. Its first case sends
the bounded issue-class cloud before initialization and requires
`VOXEL_GRID_LAYOUT_OVERFLOW` at `initial_map` with no map output. The same
instance then receives a 245-point valid cloud and must publish both its map and
pose while `rclcpp::ok()` remains true.

The second case initializes with a valid cloud, then sends a cloud that is safe
at `vg_size_for_input=0.5` but unsafe at `vg_size_for_map=0.1`. It requires the
asynchronous `map_update` stage to reject that cloud without adding a map-array
entry. Both that scan and its safe retry contain the same translated geometry;
the unsafe scan crosses a positive 0.02 m update threshold. The safe retry must
publish without further travel, proving that the rejection did not consume the
movement baseline. The case then destroys the component without one more input
callback. A separate pure-state regression binds failed versus successful
commit behavior, while the component case covers the original issue
discussion's asynchronous suspect, immediate recovery, and joining a still
joinable completed worker instead of terminating during destruction.

The positive-threshold asynchronous case at `99cce93` passed ten independent
Jazzy process executions. The prior unsafe-then-safe asynchronous case passed
ten independent processes on both supported distributions. Independent
processes are required because the
intentional five-second logging throttle retains call-site state within one
process. The repetition is a local DDS/lifecycle stability check, not a claim
about the unavailable historical bag.

## Supported-distribution execution

| Environment | Exact substrate | Build | Boundary suite | Component recovery | Complete scanmatcher CTest |
| --- | --- | --- | --- | --- | --- |
| Humble | immutable local image `ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:f1a894d81b5cb7b4e2e55a7b3fc17e538722b59c07b0bec066f2ad499a5e8447`; PCL `1.12.1+dfsg-3build1`; GCC `11.4.0`; installed `lidarslam_msgs 0.9.0` and `ndt_omp_ros2 0.1.0` underlay | prior clean read-only build PASS; exact public `7b3cb99` default workflow PASS | prior 11 / 11 PASS | exact-public `test_scanmatcher_voxel_grid_recovery` target PASS in 2.84 s; prior async 10 / 10 independent-process PASS | exact-public 10 / 10 scanmatcher PASS; complete default workflow 4,241 cases / 0 errors / 0 failures / 151 skips |
| Jazzy | Ubuntu 24.04 host; PCL `1.14.0+dfsg-1`; GCC `13.3.0`; installed `lidarslam_msgs 0.9.1` and `ndt_omp_ros2 0.1.0` underlay | local exact implementation and public `7b3cb99` default workflow PASS | 11 / 11 PASS | exact-public `test_scanmatcher_voxel_grid_recovery` target PASS in 2.87 s; local positive-threshold retry 10 / 10 independent-process PASS | exact-public 10 / 10 scanmatcher PASS; complete default workflow 4,355 cases / 0 errors / 0 failures / 151 skips; current clean-checkout S1 command 3,076 / 126 skips |

The complete CTest set includes lidar undistortion, math utilities, odometry
prior, pose prediction, pose acceptance, IMU processing, map-update policy,
point colorization, the boundary safety suite, and the component recovery
suite. Humble emitted only the existing PCL CMake policy warning. Jazzy
additionally emitted the existing PCL 1.14 deprecated-Boost-header notice;
neither build emitted a new-code diagnostic.

The Jazzy asynchronous case also passed a GCC ThreadSanitizer build with no
candidate-code race or lock report. TSan required address randomization to be
disabled and one library-name-only suppression for an unrelated
`libOpenNI2.so` static-lifecycle mutex warning; the unsuppressed run identified
only that third-party warning.

Formatting and documentation checks also passed:

- `ament_uncrustify` and `ament_cpplint` on the expanded component test;
- the touched legacy component/header retained the same 55 cpplint findings as
  the audited base, and `CMakeLists.txt` retained the same five lint findings;
  the follow-up adds no selected lint debt;
- `mkdocs build --strict`;
- `git diff --check`.

At exact implementation `99cce93`, the complete maintained Python gate passes
2,486 tests with 13 skips and 11 existing ImageIO warnings. The historical S1
carrier reported 4,253 test cases with 0 errors, 0 failures, and 127 skips. The
2026-08-17 R1 review then reran the self-contained command from a clean
checkout, building all tested packages first, and reported 3,076 test cases
with 0 errors, 0 failures, and 126 skips.
Two byte-identical candidate-bundle rehearsals contain 261 files, total
11,931,414 bytes, and have SHA-256
`5f8429e4038ca6567b2bbdb0bb00e36e5c08160631ad30ff77c7422f5080f345`.

## Remaining public gate

Public Draft `7b3cb99` now satisfies the public-revision and supported-CI
requirements: both default workflows execute and pass the component recovery
target. Issue #69 should still remain open until both remaining actions are
complete:

1. the public issue response explains the two leaf parameters and reason codes
   without claiming the unavailable historical bag was exactly reproduced;
2. the fix is included in a named release or the issue explicitly states the
   first release expected to contain it.

Until then the honest state is public Humble/Jazzy component recovery PASS,
issue response and named release pending. No issue label, comment, state,
image, or release was changed during this follow-up.
