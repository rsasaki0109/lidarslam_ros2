# GLIM clean-room adapter Phase 3b runtime (2026-08)

Phase 3b is an additive, opt-in exact-core trajectory runtime vertical slice.
It is not an active benchmark runner and does not consume bags, datasets, GT,
scorer inputs, or active profiles.  The Phase 0 host contract remains
GLIM-free; exact GLIM headers and objects are confined to `phase3b/` behind
the `GlimCoreSession::Impl` boundary.  The complete concrete mapping extension
is documented separately in [Phase 3c map output](glim-clean-room-adapter-phase3c-map-2026-08.md);
neither phase makes any benchmark or SOTA claim eligible.

## Immutable core and build surface

The runtime links only the preserved public install derived from GLIM
`faa264a1bce1bda406f73457e35511f56cdc2eaa` at
`/tmp/competitive-rival-closure.sXE92h/glim` and
`/tmp/glim-clean-room-phase1.mwzqdq6t/install/*`.  The target is
`docker/benchmark_adapters/glim_clean_room/phase3b/`; it does not include,
copy, or link any ROS bridge source.  Exact-core configuration is an explicit
directory containing `config.json`; there is no working-directory or source
tree fallback.  The CMake target requires the exact pinned GLIM vendored
`thirdparty/json/include` directory because the preserved install does not
export that public header; the include path is explicit and configuration
fails closed when it is absent.

Before `GlobalConfig::instance()` is called, a host-owned preflight parses
`config.json` and requires a JSON object with a JSON-object `global` section
and nonempty relative filenames for `config_logging`, `config_sensors`,
`config_preprocess`, and `config_odometry`; map-enabled sessions additionally
require `config_sub_mapping` and `config_global_mapping`.  Absolute paths, `.`/`..`
traversal, lexical or canonical escapes, root/component symlinks, missing or
nonregular files, malformed JSON, and non-object component JSON are rejected.
The component paths are checked both lexically and canonically, so a symlink
cannot redirect a configuration outside the explicit root.  This preflight
constructs no GLIM object and runs before the process-global configuration
lease.

The target uses one C++17 host adapter library and a synthetic fixture test.
It is bounded to one preprocessing and one odometry worker.  The process-wide
GLIM `GlobalConfig` singleton is protected by a held lease, but the lease is
acquired with `try_lock`: a second same-process configured session fails
immediately with `kCoreFailure` instead of blocking.  GLIM configuration
replacement and callback slots are process-global; safe concurrent distinct
configurations therefore require isolated processes/containers.  The slice
does not claim coexistence with arbitrary GLIM users in one process.

## Runtime mapping and lifecycle

```text
typed host EventStamp/LidarFrame/ImuSample
    -> AdapterBoundary (serialized counters/order/terminal latch)
    -> Phase 3a checked RawPoints/IMU conversion + StampLedger
    -> CloudPreprocessor
    -> public glim::OdometryEstimationCPU insert_imu/insert_frame
    -> staged EstimationFrame conversion
    -> one EOF + one get_remaining_frames drain
    -> atomic trajectory batch seal (map-enabled sessions use the Phase 3c
       trajectory+map batch seal)
    -> complete_drain / take_trajectory
```

`submit_lidar`, `submit_imu`, EOF, drain, output retrieval, and close are
serialized by a recursive session mutex.  A core call has an active-call
guard: a callback that re-enters the session is rejected.  The outer call
re-checks the terminal latch after the exact core method returns, even if the
callback ignored the nested failure; `processed` therefore cannot be counted
after a reentrant fault.  Core exceptions, callback exceptions, invalid core
outputs, conversion failures, and counter overflow all become permanent
terminal failures.  `close()` is idempotent after a successful drain; a
synchronous exact-core operation is not hard-interruptible, so bounded hard
cancellation remains `NOT_READY`.

GLIM's public odometry API has no host EOF method.  EOF is therefore a host
boundary event, followed exactly once by `get_remaining_frames()`.  The
current return value of `insert_frame()` is deliberately ignored: publishing
it and then publishing the same frame from a drain remainder can duplicate a
trajectory sample.  The runtime collects only the `marginalized_frames`
output parameter during processing and the drain remainder vector.  The
Phase3b fixture starts the NAIVE initializer at nonzero timestamps, submits
two initialized LiDAR frames (2 s and 3 s), and verifies that the two drain
remainders recover both stamps exactly once.  If any bound LiDAR stamp
remains un-emitted, drain fails closed with `kStampLedgerUnmatched`; it never
fabricates a trajectory sample.

The `StampLedger` retains authoritative int64 nanoseconds, counts emitted
entries, and requires `emitted_count == bound_count` at drain.  It is separate
from global input order and trajectory output order.  Phase 3a's lookup/commit
two-phase conversion prevents a failed conversion from consuming an entry.

## Transactional publication and counters

Core outputs are first staged in a vector bounded by the explicit
`max_pending_trajectory` option; the check does not rely on a container's
implementation-defined `capacity()`.  Once the boundary enters `Draining`,
the drain callback validates every staged frame, frame convention, contiguous
trajectory order, timestamp monotonicity, and all counter/order capacity
before changing any counter.  `record_trajectory_batch()` then seals all
outputs as one transaction.  A validation or overflow failure leaves no
trajectory output counter or order partially updated.

Normal `record_trajectory` and `record_map_chunk` success paths increment only
submitted/accepted/processed/output/order counters.  `trajectory_rejected`,
`map_rejected`, and `failures` remain zero for successful outputs; those slots
are preflighted only so their error paths can fail closed.  If a nested batch
already latched the boundary, the outer callback wrapper preserves that first
terminal error and does not double-count `failures`.

## Callback and ownership boundary

The exact core exposes process-global callback slots without synchronization
or a clear operation.  The session installs a weak lifetime fence for the
update-frame slot and removes it during teardown; the fence has no output side
effect.  All synchronous callback exceptions are caught at the session/core
boundary and latch failure.  This does not make arbitrary retained callbacks
or asynchronous callbacks thread-safe: callback-slot mutation, async mapping,
and same-process untrusted GLIM callback ownership remain `NOT_READY` and
must use an isolated process until a versioned upstream synchronization API is
available.

Phase 2's nine runtime gaps are therefore accounted for by the trajectory
slice as follows (the map rows are closed by Phase 3c):

| Phase 2 gap | Phase 3b status |
| --- | --- |
| transactional activation/rollback | closed for typed input and staged trajectory seal; no partial output publication |
| cancellation and bounded shutdown | `NOT_READY`; synchronous core calls have no hard interrupt |
| callback exception/lifetime handling | synchronous exceptions and session re-entry latch; arbitrary global-slot mutation/async callbacks remain `NOT_READY` |
| common EOF/drain protocol | closed at the host boundary with one EOF, one drain-vector read, and one finalize |
| idempotent close/unload | close is idempotent after drain; lease/resource destruction is ordered, but process-global GLIM coexistence requires isolation |
| sensor/frame/order ownership | closed by the Phase 3a typed contract, ledger, and independent trajectory order |
| ABI/provenance and install identity | exact public installed targets and pinned prefixes are required by CMake; distro/toolchain compatibility remains an external rebuild gate |
| complete trajectory/map ordering | trajectory completeness/order is closed and fail-closed; complete concrete map ordering/coverage/export is closed by Phase 3c |
| global map save/status | status-less `save(path)` remains unused; concrete synchronous `GlobalMapping::export_points()` is proven by Phase 3c, while abstract/plugin map modules remain `NOT_READY` |

This trajectory-only Phase 3b evidence does not itself exercise a map object.
For the concrete map path and its frame-coverage/point-count proof, use the
separate Phase 3c evidence document.  Abstract mapping modules and
status-less persistence remain fail-closed there.

## Synthetic adversarial coverage

The Phase3b executable has the baseline groups listed below; the current
combined Phase3b/3c executable adds map success, bounds, coverage, and mapping
callback groups as documented by Phase 3c:

* config preflight rejection for missing keys, malformed JSON, absolute and
  traversing filenames, and symlink escape, before core construction;
* exact public-core LiDAR/IMU vertical slice, ordered two-frame input, one
  EOF, two drain remainders, ledger completeness, trajectory batch seal,
  idempotent close, and exact nanosecond recovery;
* callback exception translated to a terminal fault with no publication;
* reentrant callback whose nested failure is ignored, proving the outer call
  still fails and `imu_processed` remains zero;
* immediate same-process GlobalConfig lease rejection; and
* counter-overflow fault injection and mapping-contract lifecycle checks.

Phase 0 contract tests additionally prove that successful trajectory/map
outputs leave rejected/failure counters at zero, while the Phase 3a tests
prove emitted-count/all-emitted ledger accounting and mixed global order.
Fixtures are synthetic temporary config/value files only.

## Evidence

The superseded final2 and pre-preflight roots remain preserved, but are not
authoritative for the current combined target.  The historical Phase3b-only
normal evidence root is
`/tmp/glim-clean-room-phase3b-config2-final-normal.big6xu`; the immutable
historical ASAN/UBSAN root is
`/tmp/glim-clean-room-phase3b-config2-final-sanitize.oBhbeC`.  Each root
contains one configure/build/CTest/direct/ldd sequence from the final source
state; no generated output is part of the source manifest.  Normal and
ASAN/UBSAN CTest both report 2/2 tests (runtime and source guard), the direct
executable reports the historical six trajectory groups, and `LDD_UNRESOLVED=0`.

| root | configure.log | build.log | ctest.log | direct.log | ldd.log | test binary |
| --- | --- | --- | --- | --- | --- | --- |
| normal | `40afbd479c05b5b822abc61d81de7c19c793ce1258f66169d162e1239cb2be10` | `85416515a309030168bf734b8ab51a4f5d174922984b02586e247c6b9a63944f` | `20f0c85cafeb2ec59588046786dd276e60162bfbcede7ac2dba2e2831f25a981` | `ba66f85e6dce502b033e689dadf3c69ebc2357639eddc02b273a49faac71de4b` | `8325d505b3d14f62230cacaf7558afd08f00dd0a88cc171c5acdf28295e0eb55` | `8a58ac756bd331a7fa7e9391770b4cda800a88f71129a4d136de2b75230c9578` |
| ASAN/UBSAN | `b46d50f2dc87aced9340e8b93b6d45a20d7b6e35f3109b3b10f8bd6a0584fd8e` | `b2db2f9790a205294254454bb010ab50dc2a1014161d533726630fdef7f2edbc` | `2649049170bb8a5ab5dfc7d61448b90fa1cb37690a2026cf248f5d0d50fae255` | `27a171233885102ba15b4a7a51b4558eba79c588e971d95520f87416b46b1fa0` | `94ff7e8c6b18659752e60445a0ec1febdc0f0b645a482f764a85b50e5c1b3de0` | `051f865dec42b23025414e5c6731bd4ee142bd9f351640332a11c842dd9c2a75` |

The adapter-tree canonical manifest (generated files excluded) is
`target_files=31`, `manifest_bytes=4296`, SHA-256
`7edeb61286fd4d398172731976332a65b170b51008691c2c4e73e1f90092c6d3`.
ThreadSanitizer was not run: the pinned core's process-global callback slots
have no synchronization, and this vertical slice explicitly requires an
isolated process for that unsupported same-process concurrency case.

The post-preflight host regressions are preserved under
`/tmp/glim-clean-room-phase3b-config2-regressions.NBjD9O`.  Each Phase 0 and
Phase 3a normal/sanitizer CTest run reports 2/2:

| run | configure.log | build.log | ctest.log |
| --- | --- | --- | --- |
| Phase 0 normal | `3a7567d8479a3951776516b0af19931eeabd4d52cc76f02f3cd99f101840baca` | `dbfa4bcbc5f0d981abbde29742454568716f893ea552c0ba7f2a286759edff85` | `da0b007212ba3d3001511c59e46966ca2251473fe67e21f8ef2ea9a385d225d1` |
| Phase 0 ASAN/UBSAN | `7f330d6ab6cc72426fe6ce3b5ef77d583153427c9015ac7906addad4266d7f19` | `85620a82a7d950aafd484b96d523b5a1da81248c865f6800730c8f408785c4cc` | `84be7ee8c7a2938845bde575cb708e195b170966a78df6eea58bbeb478fc5a0b` |
| Phase 3a normal | `153b3c831724fb45f64c0873208e12d92a0d0ca723eb8cd018173f638afb1488` | `4b0548bd2dd0209e8a08ff198f798d58cc36223178e1cd267d73bf9a51b4941d` | `b4a823f42d24afb6e38ac0301e90d46fbdcddb0368665011398d9702273b75ae` |
| Phase 3a ASAN/UBSAN | `7c6a860732b9f20a017f7c3d0e37616301c4a46465c7df862fd4b6962a5dc52e` | `48e9cd483425ab26a86a8ba0ed36319ac25a244c8fda879829b0fb4371f38c15` | `d18a9bf0422550bf4ad7ffbab7e73f5f350b64ae9bb6a02c72df13f55402655f` |

This phase remains runtime-only evidence.  ROS distro integration, live
frontend/backend wiring, asynchronous cancellation, abstract/plugin map extraction,
same-process callback isolation, benchmark data, and formal claim eligibility
remain `NOT_READY`.
