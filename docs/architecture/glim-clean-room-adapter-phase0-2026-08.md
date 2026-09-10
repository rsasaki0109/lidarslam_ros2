# GLIM clean-room adapter: Phase 0/1 status (2026-08)

This note records the additive host-owned adapter boundary for the pinned MIT
GLIM core revision
`faa264a1bce1bda406f73457e35511f56cdc2eaa`.  It does not modify the active
benchmark profile, selection, receipts, or r2 rival closure.  It also does not
claim a runtime result or a SOTA result.

## Scope and source boundary

The implementation is isolated under
`docker/benchmark_adapters/glim_clean_room/`.  Its Phase 0 target has no ROS
or GLIM link dependency.  The optional ROS 2 conversion header is host-owned,
header-only, and only enabled by `GLIM_CLEAN_ROOM_WITH_ROS2`; it maps message
bytes into the local typed view and does not import an upstream bridge.

The only Phase 1 upstream inputs accepted by the CMake smoke are an explicitly
provided include directory containing the pinned public
`glim/util/raw_points.hpp` header, an existing pinned library, and explicitly
provided dependency libraries.  The expected commit is validated as a
40-character lower-case hexadecimal identity.  The adapter never fetches,
builds, or substitutes a GLIM revision.  If those inputs are absent, CMake
reports Phase 1 `NOT_RUN` and still builds only the Phase 0 contract tests.

## Typed boundary

`contract.hpp` and `pointcloud2_parser.hpp` define the host-owned contract for:

- PointCloud2 x/y/z/intensity/ring/relative-time fields, explicit units,
  endian-safe decoding, finite values, and frame/stamp identity;
- IMU acceleration in m/s² and angular velocity in rad/s with explicit frame
  and stamp identity;
- explicit calibration convention, LiDAR/IMU frames, and finite extrinsics;
- contiguous input event order, nondecreasing timestamps, EOF and drain
  transitions, ordered trajectory/map output, and consumer counters; and
- artifact roles and paths, with GT/scorer/metric roles and path-like aliases
  rejected before a runner can receive them.

`core_engine.hpp` is intentionally only the host-owned Phase 0 sink/engine
boundary.  A core-backed implementation is gated on the exact public-header
and library smoke; no GLIM bridge implementation is copied into this tree.

## Phase 0 evidence

The preserved first configure failure was
`/tmp/glim-clean-room-phase0.bMUNpX` (unsupported CMake `{40}` regex).  That
root and its logs remain unchanged.  After replacing that check with explicit
length and non-hex validation, the bounded smoke was run once in the distinct
root `/tmp/glim-clean-room-phase0.L3LRnf`:

| Check | Result |
| --- | --- |
| CMake configure | PASS |
| C++ contract executable | PASS, 8 contract cases |
| clean-room source guard | PASS, 7 production files |
| CTest | PASS, 2/2 tests |
| Phase 1 public API compile/link | NOT_RUN; no pinned built GLIM library was supplied |

Evidence log SHA-256 values:

| File | SHA-256 |
| --- | --- |
| `configure.log` | `e1419f13afcdc3d452aac699a87034e6792c548b81f0de0a360a499b38fa47e1` |
| `build.log` | `8e3638ac073a208aba0cff9caaf3f67823bba2b3159fe1f4bcce8e6c9fdf5028` |
| `ctest.log` | `48665d6eabaf125c466e23a674a06ab1364ea222174ab5521f2237ac37de52cd` |
| contract test executable | `45d0f3843c4d7e5eeb4f2491c911a7f43463a3bfa1fb759cfd90841ba589094a` |

## Gates before Phase 1 or benchmark eligibility

1. Keep the current profile, receipts, and r2 closure immutable; this adapter
   is not evidence for the existing benchmark.
2. Supply a reviewable build of the exact pinned GLIM core and all dependency
   identities, then configure the opt-in public-header compile/link smoke.
3. Reopen and validate the installed core/API provenance, conversion units,
   calibration, ordering, EOF/drain, trajectory/map outputs, and consumer
   counters in a clean install.  A missing, stale, or mismatched identity is a
   fail-closed result.
4. Only after independent runtime and dataset/GT-blind gates are complete may
   any adapter-backed benchmark be considered.  Phase 0 contract PASS alone
   does not change the active claim or runtime readiness state.

## Phase 0.2 hardening

The boundary now retains authoritative event stamps as signed 64-bit integer
nanoseconds.  ROS `sec`/`nanosec` conversion is checked for component range and
signed overflow; conversion to floating-point seconds is exposed only through
the explicit core-facing helper.  The PointCloud2 parser requires a sequence-
bound field mapping (name, datatype, count, and relative-time unit), rejects
duplicate/overlapping/out-of-range fields, preserves raw relative time, and
checks finite non-negative raw and seconds values, including padded rows and
big-endian payloads.

Calibration validation requires explicit forward and inverse conventions,
distinct IMU/LiDAR frames, homogeneous last rows, orthonormal rotation with
determinant `+1`, and mutually inverse transforms.  Input frame IDs must match
the calibration contract.  Trajectory/map output frames, zero-start contiguous
orders, monotonic trajectory stamps, unit quaternions, and required output
presence before drain completion are enforced.

Every validation, ordering, state, counter-overflow, and core callback failure
sets a terminal fault latch.  Subsequent input/output/lifecycle calls are
rejected without invoking the sink.  Submitted, accepted, processed, and
rejected counters are separate for LiDAR, IMU, trajectory, and map channels;
core callback exceptions are translated to `kCoreFailure`.

The Phase 0.2 smoke uses warnings-as-errors and was run in two fresh roots:

| Check | Result |
| --- | --- |
| normal CMake/build/CTest | PASS, 2/2 CTest; 12 contract groups |
| ASAN + UBSAN CMake/build/CTest | PASS, 2/2 CTest |
| Phase 1 public API compile/link | NOT_RUN; no pinned built GLIM library supplied |

The first hardening compile root
`/tmp/glim-clean-room-phase02-compile.jJJdqe` is preserved as a warning
failure.  The corrected normal root is
`/tmp/glim-clean-room-phase02-rerun.Nfvpzo`; the sanitizer root is
`/tmp/glim-clean-room-phase02-sanitize.xxG9DQ`.  These are contract evidence
only and do not alter the active profile or benchmark receipts.

## Phase 0.3 counter-overflow hardening

All production counter and internal order increments now pass through checked
capacity helpers.  Multi-counter transitions preflight every required slot
before changing counters or invoking a sink/core callback.  Input acceptance
preflights accepted/received/processed/rejected/failure capacity; EOF and
drain preflight lifecycle and failure capacity; output paths preflight order,
accepted, processed, output, rejected, and failure capacity.  A failed
preflight latches `kCounterOverflow` and invokes no sink/core callback.

Rejection precedence is deterministic: if the failures counter or the
channel's rejected counter is saturated while reporting any semantic/state/
core error, `kCounterOverflow` wins, neither counter is partially updated, and
the boundary remains terminally failed.  A saturated failures counter never
wraps.  Test-only counter/order seeding is compiled only under
`GLIM_CLEAN_ROOM_TESTING`; it is absent from the production/install ABI and
exercises public input, EOF, drain, trajectory, and map paths.

Phase 0.3 evidence uses fresh roots (the previous Phase 0/0.2 roots remain
preserved):

| Check | Result |
| --- | --- |
| normal CMake/build/CTest | PASS, 2/2 CTest; 14 contract groups |
| ASAN + UBSAN CMake/build/CTest | PASS, 2/2 CTest |
| warnings-as-errors | PASS |
| source guard and diff check | PASS |
| Phase 1 public API compile/link | NOT_RUN; no pinned built GLIM library supplied |

Normal root: `/tmp/glim-clean-room-phase03-closure.KeBDjA`.
Sanitizer root: `/tmp/glim-clean-room-phase03-closure-sanitize.pm3u6v`.

## Reproducible evidence closure

The canonical source-tree manifest covers exactly the regular files below
`docker/benchmark_adapters/glim_clean_room/`.  It excludes generated
`__pycache__` directories and `.pyc` files, stores repo-relative POSIX paths,
sorts those paths lexicographically, and performs two SHA-256 stages: first
each file's bytes, then the exact manifest bytes formed as
`relative_path + NUL + lowercase_hex_file_sha256 + LF` for every sorted file.
The architecture document is deliberately outside this source-tree hash, so
recording the hash here cannot create a self-reference.

Run from the repository root:

```sh
python3 - <<'PY'
from pathlib import Path
import hashlib

repo = Path.cwd().resolve()
root = repo / "docker/benchmark_adapters/glim_clean_room"
files = sorted(
    p for p in root.rglob("*")
    if p.is_file()
    and "__pycache__" not in p.parts
    and p.suffix != ".pyc"
)
manifest = bytearray()
for path in files:
    relative_path = path.relative_to(repo).as_posix()
    file_sha256 = hashlib.sha256(path.read_bytes()).hexdigest()
    manifest.extend(relative_path.encode("utf-8"))
    manifest.extend(b"\0")
    manifest.extend(file_sha256.encode("ascii"))
    manifest.extend(b"\n")
print(f"target_files={len(files)}")
print(f"manifest_sha256={hashlib.sha256(bytes(manifest)).hexdigest()}")
print(f"manifest_bytes={len(manifest)}")
PY
```

Current recomputation after the Phase 3c source changes:
`target_files=31`, `manifest_bytes=4296`,
`manifest_sha256=7edeb61286fd4d398172731976332a65b170b51008691c2c4e73e1f90092c6d3`.
The earlier combined adapter-plus-document working-tree digest
`5cf5977d64a87d42dbec91d96d17a286dcd4b2be6559e37c78b14bd243ab3eb6` is
historical and is superseded by this path-independent, non-self-referential
source-tree manifest.

The retained final-run log SHA-256 values are:

| Root | configure.log | build.log | ctest.log |
| --- | --- | --- | --- |
| `/tmp/glim-clean-room-phase03-closure.KeBDjA` | `7b8e2f07af47c0370865a87ebc2b3c93a35f3bf3446d3a865991a3d19127bcba` | `8e3638ac073a208aba0cff9caaf3f67823bba2b3159fe1f4bcce8e6c9fdf5028` | `39f300d9097cb4d532b0a57167dd4ae584bd2a61a2550d839f4142d162b7bdb0` |
| `/tmp/glim-clean-room-phase03-closure-sanitize.pm3u6v` | `c003151dbbaa5db5ec438c44bb89a4d682d5d6239fbe76af653f4dcc9b3c6bb8` | `8e3638ac073a208aba0cff9caaf3f67823bba2b3159fe1f4bcce8e6c9fdf5028` | `ac927aa20f813b1df10ff249dd269ef14f8f85df879a3d76684943d3cc9491cd` |

## Phase 1 exact-core public-API candidate recipe (prepared, not run)

The additive Phase 1 candidate is under
`docker/benchmark_adapters/glim_clean_room/phase1/`.  It is deliberately
separate from every active benchmark profile, selection, receipt, and the
existing GLIM benchmark Dockerfile.  The machine-readable closure is
`source_closure.json`, its schema is `source_closure.schema.json`, and the
offline validator is `validate_phase1_recipe.py`.  The host launcher is
`phase1_build.py`; it never edits a source checkout or benchmark metadata and
rejects a reused output root.

The v1 recipe identity and host-only receipt below are historical evidence;
the container recipe is superseded by the additive Boost-source r2 section at
the end of this document.  Neither revision is benchmark evidence.

The only source components named by this candidate are official,
commit-addressed GitHub archives/checkouts:

| component | commit | archive SHA-256 | source-tree SHA-256 | license evidence |
| --- | --- | --- | --- | --- |
| GLIM core | `faa264a1bce1bda406f73457e35511f56cdc2eaa` | `d8176e85199a2297269d34fcfb57e4cb2c2d53e593439f665f614cae57972c25` | `2394e58c0c7fe218770b6db20cdab71395c885608783966ae94491e85899260e` | `LICENSE` MIT, `e491c5c12eef41e3a5f673fa0b1942d575fbd1bbd8c4dfa73e4c9d464c4a9ba4` |
| GTSAM | `2f3e56c0ddbd3a1aa54ed043643b553d26a069f6` | `50bd99ddbb363f03f145d814995df234c83ad38f867080fba5b60f6c151b348a` | `f43f0991fcb580b421afd30eb191a12dbea9c92d2dfa355a182932dd0048da38` | `LICENSE`, `LICENSE.BSD`, and recorded third-party notices |
| gtsam_points | `9d32e7dbecf6015560d84b4901d6b0a6f483ec46` | `dc977a8b2a6aeda5d48107920e2cdd7e0e45e1478c3590e45f053335eb975479` | `f3b3ac9b29a49a28e3f9e79fd36cc764caf2e46e961f229283507ff1cc8ffc00` | `LICENSE` MIT, `e491c5c12eef41e3a5f673fa0b1942d575fbd1bbd8c4dfa73e4c9d464c4a9ba4` |

These candidate tree hashes are recomputed from the exact archive/checkouts
with sorted relative POSIX paths and file bytes, excluding only `.git`
metadata.  They are candidate-local identities; no historical profile hash is
rewritten or used to upgrade an active receipt.

The pinned base is the locally available `ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f`
(Jazzy, linux/amd64).  The recipe is Release, bounded to two build workers,
disables `-march=native`, CUDA, viewer, OpenCV, tests, examples, timing,
Python, MATLAB, and TBB, and links the smoke through the exported `glim::glim`
target.  The Dockerfile verifies every downloaded archive and recorded license
artifact with SHA-256 before compilation, and records installed-file hashes in
the image.  A missing base
dependency is a hard failure; it cannot trigger a package-manager or alternate
source fallback.  No dataset, ground truth, benchmark, or scorer path is
present in the candidate.

The read-only preparation checks used the retained clean checkouts under
`/tmp/competitive-rival-closure.sXE92h/` and the three recorded archives:

```sh
python3 docker/benchmark_adapters/glim_clean_room/phase1/validate_phase1_recipe.py --json
python3 docker/benchmark_adapters/glim_clean_room/tests/source_guard.py docker/benchmark_adapters/glim_clean_room
python3 docker/benchmark_adapters/glim_clean_room/phase1/phase1_build.py \
  --validate-only \
  --glim-source /tmp/competitive-rival-closure.sXE92h/glim \
  --gtsam-source /tmp/competitive-rival-closure.sXE92h/gtsam \
  --gtsam-points-source /tmp/competitive-rival-closure.sXE92h/gtsam_points \
  --archive-dir /tmp/competitive-rival-closure.sXE92h
```

Those checks PASS; the raw `source_closure.json` file SHA-256 is
`c34c6a0a7342c281f820c0f429a245e79d50ba00cb7a9e2da09542da24c55e6b`, and the
validator's canonical manifest identity is
`782a29098ed5dfa6cb6440738a34810f503335a8dd6936281537c0f7b77dfc8c`.
The Phase 1 Docker build and host
compile/link launcher are **NOT_RUN in this preparation task**.  Future
execution must use a fresh root and preserve a failure receipt, for example:

```sh
python3 docker/benchmark_adapters/glim_clean_room/phase1/phase1_build.py \
  --glim-source /ABSOLUTE/clean/glim \
  --gtsam-source /ABSOLUTE/clean/gtsam \
  --gtsam-points-source /ABSOLUTE/clean/gtsam_points \
  --archive-dir /ABSOLUTE/official-archives \
  --output-parent /ABSOLUTE/fresh-evidence-parent
docker build --pull=false -f docker/benchmark_adapters/glim_clean_room/phase1/Dockerfile \
  docker/benchmark_adapters/glim_clean_room
```

The Docker build command is also **NOT_RUN** here.  Any later receipt must
separately report image ID/digest, source/archive/license identities,
installed headers/libraries, compile/link smoke and all logs; this candidate
cannot upgrade the existing benchmark's readiness.

## Phase 1 host exact-core compile/link evidence

The authorized single host attempt used the verified local closure and the
launcher command below.  It used a fresh root, two bounded Make workers, and
did not execute a Docker build, benchmark, dataset, ground-truth, or scorer
operation:

```sh
python3 docker/benchmark_adapters/glim_clean_room/phase1/phase1_build.py \
  --glim-source /tmp/competitive-rival-closure.sXE92h/glim \
  --gtsam-source /tmp/competitive-rival-closure.sXE92h/gtsam \
  --gtsam-points-source /tmp/competitive-rival-closure.sXE92h/gtsam_points \
  --archive-dir /tmp/competitive-rival-closure.sXE92h \
  --output-parent /tmp --jobs 2
```

Result: `PASS`, root
`/tmp/glim-clean-room-phase1.mwzqdq6t`, sealed receipt
`phase1.receipt.json` SHA-256
`0689c8fa30648abd03d1b1f83ed003abb33014b273472957f690d79aece5ec4f`.
The receipt sidecar agrees with that hash.  All 12 configure/build/install
steps returned zero.  The receipt records the exact recipe hashes, source
closure, archive/license hashes, compile-command hashes, and installed-file
inventory.

The adapter CTest result is 3/3 PASS:

| test | result |
| --- | --- |
| `glim_clean_room_contract_tests` | PASS |
| `glim_clean_room_source_guard` | PASS |
| `glim_core_public_api_link_smoke` | PASS |

The CTest log SHA-256 is
`5a24b70ac5b5c5acb68d132dfa25e2930508bab4b7234e5f0b2cddf0271ef1b0`.
The smoke executable SHA-256 is
`53db6dcd134ef6088bd71d12625fac021fac5e1b38d11a4d26b791e3caed7243`;
the installed GLIM, GTSAM, GTSAM unstable, and gtsam_points library hashes
are respectively
`5e1ed6dc8438ba2daa99ebff3862772868f6a0a3f1cda4a4f36ecd2e0e6b5cbc`,
`afbbd80c5e36e0eabc64ddbc8daa3ddad27618edc9a5a112999d0c54bfe0d8af`,
`5a070fc712eb315a823771277d3cc72f737f1a29af2f1a20ab5ef66b4954b6f3`, and
`a57e6dcfa58e422260c96bb6716d840c171f5a1f4f8034fa7ffd5c775a755012`.
The canonical installed artifact manifest SHA-256 is
`deb4f3c65dd7f83d50978f198f02fd364181309a188487ad4a432abe5d03c4ed`.
The inventory contains 65 GLIM entries, 515 GTSAM entries, and 149
gtsam_points entries (including install symlinks).

The installed exported target was reopened from
`install/glim/lib/cmake/glim/glim-targets.cmake`; its SHA-256 is
`1b646db7d79baf0080ee6bf7ad1a6531173625431d2b2fad93a8e74d1d6c2f19`, and it
contains `glim::glim` with the pinned GTSAM/gtsam_points dependency targets.
The smoke and GLIM DSO `ldd` logs contain no unresolved dependency and their
SHA-256 values are `780b85f7535db4f4bc1840c220de4d08dca3461d481b211b379699f1603fa9aa`
and `8a6ddbf467eae34e2060b40ceee89362c95dfc9e05dc000bee3a7e1685da820c`.
The source revalidation output SHA-256 is
`72c9dfc8468b5ccf295ef4214ce87cec999a5cda9b97e04bb69883dda245aeaa`.

The post-build audit is sealed at
`/tmp/glim-clean-room-phase1.mwzqdq6t/postbuild_audit.json` with SHA-256
`14168ce9e36b29dffa3f8b9d40f42478e31390dddfb3da780e6b2ac8ed47cf63`.
Its provenance scan found no external bridge source, compile-command, binary,
or linker reference.  The exact upstream GLIM core install contains one
case-insensitive `GLIM_ROS2` optional macro in
`glim/util/ros_cloud_converter.hpp`; this is an upstream core header, not a
cloned/copied bridge package, and it is not present in compile commands or
DSO dependencies.  The compiler/machine supplement is
`postbuild_toolchain_identity.json`, SHA-256
`e1ac17e8ddab4c5fd199b48a36cebb8ba03729000a45749975585bf9595dce5e`,
recording GCC 13.3.0, CMake 3.28.3, x86_64, and the host kernel identity.

This is functional compile/link evidence only.  The candidate remains
`benchmark_eligible: false`; Docker image construction and all benchmark/data
execution remain NOT_RUN/FORBIDDEN.

## Phase 1 container candidate r2: Boost source closure (recipe preparation)

The additive container recipe was revised as candidate
`glim-clean-room-phase1-core-public-api-v2`.  The active benchmark profile,
selection, receipts, and existing GLIM benchmark Dockerfile remain unchanged.
The pinned Jazzy base image was preflighted read-only before this revision:
the base lacks the Boost serialization header and serialization, graph, and
filesystem libraries, so the preflight is retained as `FAIL_CLOSED` rather
than treating the host-only compile result as container evidence.

The preserved preflight receipt is
`/tmp/glim-phase1-base-preflight.0pITMy/base_dependency_preflight.json`,
SHA-256
`8a3d3fe7c64b87443d9203560255c49e481bfec16c54d5bb16db1d57f085d5f1`;
its sidecar SHA-256 is
`19245c16f2439b0f738533f536ec5cf8c9bc4ab71f9b9bd85b150a0e42c4e9b7`.
The four observed missing paths are recorded in that machine-readable receipt.

Candidate r2 obtains Boost only from the immutable official release archive:

| field | value |
| --- | --- |
| version | `1.83.0` |
| URL | `https://archives.boost.io/release/1.83.0/source/boost_1_83_0.tar.bz2` |
| archive bytes | `122892751` |
| archive SHA-256 | `6478edfe2f3305127cffe8caf73ea0176c53769f4bf1585be237eb30798c3b8e` |
| extracted tree SHA-256 | `feb033ff2277bff1befa57ca65ede54a2c7a1fd4f6498d238daae8470dd2fbb0` |
| tree hash kind | `relative_path_content_sha256_v1_excluding_git_metadata` |
| extracted file count | `76253` |
| license | `LICENSE_1_0.txt`, Boost Software License 1.0 |
| license SHA-256 | `c9bff75738922193e67fa726fa225535870d2aa1059f91452c411736284ad566` |

The recipe validates the archive bytes, safe extracted tree, and license before
running `bootstrap.sh` and `b2`.  It builds only serialization, graph,
filesystem, thread, program_options, date_time, timer, chrono, regex, and
system as Release shared libraries with C++17, two workers, multi-threading,
shared runtime, fixed `/opt/install/boost` prefix, and no native CPU tuning.
GTSAM, gtsam_points, GLIM, and the adapter smoke each receive that prefix with
system Boost fallback disabled; the smoke stage copies the Boost installation
and adds its library directory to `LD_LIBRARY_PATH`.

The base stage now performs a CMake configure/compile/link probe for the
remaining Eigen3, spdlog, OpenMP, compiler, CMake, Make, and Python3 closure.
No apt/host-artifact fallback is present.  Docker build, benchmark, dataset,
ground-truth, and scorer execution are **NOT_RUN/FORBIDDEN** for this recipe
revision.  The candidate remains `benchmark_eligible: false`.

The r2 candidate source tree contains 18 regular files (excluding generated
`__pycache__`/`.pyc` files).  Its path-plus-content tree SHA-256 is
`b2d15a0148848a80f50138cd063e1f5a233fcffc042b42007af42c99b41d5697`.
The Boost archive/tree verification output is preserved under
`/tmp/glim-phase1-boost-closure.8ZLviX/`; the canonical tree verification JSON
SHA-256 is `1f38adc9ab5b7dae036ad5f187cfe65497236c28bb6a526e427b064e8d7ca98d`.
The immutable-archive/member/license closure receipt is
`boost_closure_receipt.json`, SHA-256
`9adaae2d76b19f5303222a9c828ef4f1c349edf0deb7b620de9e305ddf40b7e5`, with
sidecar SHA-256
`cbe1f03ecfdf39de2b00e68a55a36623f673ac9634fe103ef8b27f8ebf274ff4`.
The recipe-preparation checks completed without a container build: recipe validator PASS,
JSON Schema PASS, clean-room source guard PASS (13 files), adversarial recipe
tests PASS (5/5), and fresh Phase 0 CTest PASS (2/2).  The final Phase 0 CTest
root is `/tmp/glim-phase1-r2-phase0-final.Ygdnaa`; its configure, build, and CTest log
SHA-256 values are respectively
`2d368441a3e11e326f0336b89b9ca24b3d69a2e50b4ab644b1bddefe5f1c9995`,
`cd136a8b1afbe2a75114b8f267d19832710744db2d8d470fffa9848823d7df98`, and
`31f5d8e04b00df916b98ce2ba69086c09e8848ed3aa6a7eb7d4cf2a9f2beb239`.

## Phase 1 container candidate r2: one-attempt container evidence

The later authorized container attempt was executed exactly once, with no
retry and no source/profile/selection/receipt mutation.  Its preserved root is
`/tmp/glim-clean-room-phase1-container-r2.EbkpQf`; the exact tag was
`glim-clean-room-phase1-r2:20260825-0630-b7c91a`.  The pinned base was
`ros@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f`.
The build exited `0` and produced image ID/digest
`sha256:29521501ad80b77e5f5ba90bbd1dac2e4a820ea267ac7ad47e3026669841a909`.
The image is explicitly labeled `org.opencontainers.image.benchmark_eligible=false`.

The image's in-build and network-disabled post-run checks agree:

| check | result |
| --- | --- |
| full CTest | 3/3 PASS |
| public API link smoke | 1/1 PASS |
| installed regular files | 16,428; installed hash manifest verified |
| dynamic-link audit | 26 targets, 0 unresolved |
| benchmark/dataset/GT/scorer | FORBIDDEN; not executed |

The build log SHA-256 is
`d913db5d50a9c05bc49a29475484ff456ddb1bc5f82cc17f2d640fc0a3ca3118`, and the
post-run log SHA-256 is
`8dd8cc391460d7c25c1443bc507885bbff18dbef10663101bfc6aeeca05561a6`.
The authoritative corrected receipt is
`/tmp/glim-clean-room-phase1-container-r2.EbkpQf/attempt.receipt.corrected.json`,
SHA-256
`0f1600aecadd38df5e2f792c8c06e05a7b19b544b640202f2d37e817b4747269`, with
sidecar SHA-256
`85f75340eec3bbec8451a0b1da6911c946624d7579ec237840ed87f5d18b9222`.

The original receipt remains preserved at
`attempt.receipt.json` (SHA-256
`8e300b0498f006cf269dbf044adbacecbe95f20e1148affb924c305a87a38250`) and is
not authoritative: its first summary counted the literal `set -x` command
containing `grep -c 'not found'` after the final LDD marker.  The correction
receipt is `CORRECTED_SEALED`, names that exact superseded SHA, and records the
marker-bounded LDD result of zero.  No build or container run was repeated to
make that correction.

The production parser and receipt contract are additive:
`docker/benchmark_adapters/glim_clean_room/phase1/container_evidence.py` and
`container_receipt.schema.json`.  The parser accepts only lines between the
exact `LDD_REPORT` and final `LDD_UNRESOLVED=` markers, so shell tracing after
the final marker cannot create a false unresolved dependency.  Its CLI is
limited to `validate-receipt` and `parse-postrun`; it has no build/run mode.
`CORRECTED_SEALED` receipts require a path, SHA-256, and reason in
`supersedes_receipt`; PASS receipts require the forbidden execution roles,
benchmark-ineligible image label, zero exits, CTest 3/3 plus 1/1, and zero
unresolved dependencies.  The current corrected receipt validates, while the
preserved original fails closed as intended.
