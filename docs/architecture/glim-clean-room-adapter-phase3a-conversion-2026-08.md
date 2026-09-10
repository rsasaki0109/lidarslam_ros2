# GLIM clean-room adapter Phase 3a exact-type conversion (2026-08)

Phase 3a is an additive, conversion-only target.  It does not run GLIM core
processing, consume a sensor/data/GT/scorer path, run a benchmark, or change an
active profile, selection, receipt, or Docker recipe.  The Phase 0 public host
ABI remains GLIM-free; exact GLIM types appear only in the opt-in
`phase3a/` target.

## Exact-core evidence

The preserved core checkout is
`/tmp/competitive-rival-closure.sXE92h/glim` at immutable commit
`faa264a1bce1bda406f73457e35511f56cdc2eaa`.  The preserved Phase 1 install is
`/tmp/glim-clean-room-phase1.mwzqdq6t/install/*`; its public
`glim/include/glim` manifest contains 54 files and has SHA-256
`937097c5e1f6fb8aecc02e152560a1db114539929ad12ce8b6b4e4a14dad4ba9`.
No core source or installed file was modified.

The exact core source confirms that `RawPoints::points` are consumed as
homogeneous points and that mapping validates `abs(point.w() - 1.0) < 1e-3`.
Phase 3a therefore writes `w = 1.0` from the host's Cartesian x/y/z fields;
it never accepts or infers an incoming fourth coordinate.  Core preprocessing
copies `RawPoints::times` directly and sorts by those values, so the host
conversion supplies checked seconds and retains the original integer event
identity separately.

## Conversion boundary

The implementation is split from the Phase 0 ABI:

```text
phase3a/include/glim_clean_room/phase3a_conversion.hpp
phase3a/src/phase3a_conversion.cpp
phase3a/CMakeLists.txt
```

`phase3a_conversion.hpp` is included only by the opt-in target configured with
the preserved exact-core install.  The root `include/glim_clean_room` headers
remain host-only and do not expose `glim::RawPoints` or
`glim::EstimationFrame`.

### LiDAR

`to_core_raw_points(frame, contract, ledger)` first validates the complete
sequence/frame contract.  It then:

* splits signed int64 nanoseconds into integer seconds and remainder before a
  checked, finite conversion to core `double` seconds;
* converts each authoritative `relative_time_raw` using the explicitly bound
  `PointTimeUnit`, rejecting non-finite/negative/non-representable values;
* preserves point count, intensity, and ring vectors;
* writes homogeneous `(x, y, z, 1.0)` points and does not put frame identity in
  a field that the core type does not have; and
* binds the event to the ledger only after the host payload has passed
  validation and local construction.

### IMU

`to_core_imu(sample, expected_frame)` returns the typed
`(double stamp_seconds, Eigen::Vector3d acceleration, Eigen::Vector3d angular)`
record expected by the public core methods.  The host contract must already
bind SI units and frame identity; conversion rechecks those bindings and all
stamp/vector values for finite representability.  No arbitrary physical
threshold is invented where the profile has not declared one.

### StampLedger

`StampLedger` is bounded and terminal-latched.  Each input entry stores the
authoritative `EventStamp` and the exact double produced by the split
conversion.  Output matching uses deterministic IEEE-754 bit ordering and a
maximum one-ULP distance; it never uses `llround` or reverse-multiplies a
double to guess nanoseconds.

The ledger rejects non-increasing global input order, timestamp regression,
capacity exhaustion, non-finite output, unmatched output, already-consumed
output, and ambiguous matches.  Global order gaps are intentional: a
LiDAR-only ledger may bind event orders 0 and 2 when event order 1 was an IMU
sample handled by the separate typed IMU path.  The ledger's insertion/binding
sequence is therefore independent of global `EventStamp::order`.

If two distinct large-epoch integer stamps are equal or ULP-near in the core
double domain, the second bind fails closed before any core call.  Output
recovery is two-phase: `lookup_output()` identifies the unique unconsumed
entry, conversion validates the complete output, and `commit_output()` marks
that entry emitted.  A successful trajectory conversion returns the original
int64 nanoseconds but assigns an independent contiguous trajectory output
order.  `emitted_count()` and `all_emitted()` provide a non-destructive drain
completeness check; an empty ledger is complete, while any bound-but-unemitted
entry fails closed.  All ledger errors latch permanently; later calls return
the same failure.

### Trajectory

`TrajectoryConverter` consumes a public
`glim::EstimationFrame::ConstPtr` only after checking a finite rigid
`T_world_lidar`: homogeneous last row, orthonormal rotation, determinant +1,
and quaternion normalization tolerance.  The converter is constructed from
the complete `SequenceContract`, so it binds the precommitted trajectory frame
and contract validity rather than accepting a free-form frame string.  It
recovers the exact timestamp through the ledger, assigns its own next
contiguous trajectory output order, and emits translation plus canonical
`(qx, qy, qz, qw)`.

Quaternion sign is deterministic: positive `w`; for an exact `w == 0`
half-turn, lexicographically positive `(x, y, z)`.  The converter has its own
terminal latch, so a bad transform, wrong order, duplicate output, or invalid
core frame cannot be followed by a successful output.

Map conversion is explicitly `NOT_IMPLEMENTED` through
`map_conversion_not_implemented()`.  No GLIM map type is accepted or silently
discarded in this phase.

## Adversarial coverage

The synthetic host-only test executable covers eight groups:

* negative and minimum int64 epochs, split seconds, point time conversion,
  intensity/ring/count preservation, and homogeneous `w`;
* NaN/Inf point and IMU values, wrong frame/unit, and time failure;
* ledger order regression, bounded capacity, terminal latch, and large-epoch
  duplicate-double collision;
* one-ULP output matching, unmatched output, reused output, and wrong order;
* mixed global order (`LiDAR 0`, `IMU 1`, `LiDAR 2`) with independent
  trajectory output order and exact nanosecond recovery;
* trajectory translation, exact integer identity recovery, and canonical
  half-turn quaternion sign;
* invalid homogeneous row, non-finite/bad rotation, and converter latch; and
* explicit map `NOT_IMPLEMENTED` behavior.

No test constructs a preprocessor/odometry/mapping object or feeds a core
input stream.  It only constructs the public value types needed to prove the
conversion ABI.

## Evidence roots and results

The first normal attempt is preserved at
`/tmp/glim-clean-room-phase3a-normal.K0scFE`; compile/link and source guard
passed, while the synthetic expected-time assertion exposed a test fixture
field-order error.  The fixture was corrected without weakening assertions.

The earlier corrected normal and sanitizer roots
`/tmp/glim-clean-room-phase3a-final2.Rg0Yuc` and
`/tmp/glim-clean-room-phase3a-sanitize-final2.rr2DSh` remain preserved as
historical evidence, but they predate the Phase3a.1 mixed-order case and are
superseded by the fresh roots below.  The final corrected normal run, including
the raw-shape guard, mixed global-order case, and contract-bound trajectory
converter, is `/tmp/glim-clean-room-phase3a-final-normal2.FPSGBz`:

| log | SHA-256 | result |
| --- | --- | --- |
| configure.log | `7726b987c592ef5f242ae5dadcb36eca61387e13beeb31a81f73f445875f7858` | PASS |
| build.log | `e5c0084a8c3acd33fa5a2a8ef722caf70efdae8d2b75cc04eb50e3de69e81785` | PASS |
| ctest.log | `cf0ffc0f30e402b97bf09ce8e15a812749e32cb96bf351b53d6db8885aa6e26b` | 2/2 PASS; executable reports 8 groups |
| direct executable | `87e3c40b199351679271c23776079d53f6c80034cef094c280b356f775c6663a` | 8 groups PASS |

The final fresh AddressSanitizer/UndefinedBehaviorSanitizer run is
`/tmp/glim-clean-room-phase3a-final-sanitize2.G5Syh6`:

| log | SHA-256 | result |
| --- | --- | --- |
| configure.log | `64d1da0ff7fda9a0a198500e8ca465dc2108cd1dcd2029867047667e81d240f3` | PASS |
| build.log | `950ae585578b62ed18cd747cf92865e601b71ac7736996e5acd158289aeea102` | PASS |
| ctest.log | `2b77dd030b6227f2377d600a1fb820ef96776b09fb85bd1e2c4d09208dfbb231` | ASAN/UBSAN 2/2 PASS; executable reports 8 groups |
| direct executable | `87e3c40b199351679271c23776079d53f6c80034cef094c280b356f775c6663a` | ASAN/UBSAN; 8 groups PASS |

The final Phase 0 regression run is preserved at
`/tmp/glim-clean-room-phase3a-final-phase02.EDqqSL` and passed both contract
tests and source guard (2/2).  Its configure/build/CTest log SHA-256 values
are respectively
`9c4bf2fc3c6e23a4462b2ef5649f9e12c0fe1a60e04d6406bd22d3db4ecb25d6`,
`cd136a8b1afbe2a75114b8f267d19832710744db2d8d470fffa9848823d7df98`, and
`9fd138abb438295704855cf70bf1fa816ada49332e83649c10984a3e72eaef51`.
Phase 0's optional exact-core smoke remained NOT_RUN in that run by design.

The additive adapter tree now has 31 regular files (generated bytecode
excluded), 4,296 canonical manifest bytes, and current manifest SHA-256
`7edeb61286fd4d398172731976332a65b170b51008691c2c4e73e1f90092c6d3`.
The exact path-independent command is the two-stage command in the Phase 0
document: sorted regular files below `docker/benchmark_adapters/glim_clean_room/`,
excluding `__pycache__` and `.pyc`, with each repo-relative POSIX path, NUL,
lowercase file SHA-256, and LF.  It currently reports
`target_files=31`, `manifest_bytes=4296`.
The Phase 3a source guard checked 21 production files.  Focused Phase 1
regression tests remained 12 passed plus 2 subtests; the recipe validator
remained PASS.

All profiles, selections, receipts, and Dockerfiles remain unchanged.  Phase
3a is conversion evidence only; core processing, map conversion, ROS runtime,
benchmark eligibility, and formal results remain **NOT_READY**.
