# GLIM clean-room adapter Phase 3c map output (2026-08)

Phase 3c is an additive, opt-in exact-core map-output vertical slice.  It is
implemented by the existing `phase3b::GlimCoreSession` target when
`SequenceContract::require_map_output` is true; the Phase 0 installed host
surface remains GLIM-free.  No benchmark profile, bag, dataset, GT, scorer,
or ROS bridge is involved.  This document records the mapping proof boundary;
it is not a benchmark or SOTA claim.

## Audited public API and ownership

The audit uses only the pinned GLIM public install/source at commit
`faa264a1bce1bda406f73457e35511f56cdc2eaa` and the matching installed
`gtsam_points` headers.  The concrete APIs used are:

| Role | Public API | Proof used by the adapter |
| --- | --- | --- |
| Submapping | `glim::SubMappingPassthrough::insert_frame`, `get_submaps`, `submit_end_of_sequence` | Host stages each finalized `EstimationFrame` once, drains the destructive queue, submits EOS once, and rejects null/duplicate/empty/non-rigid submaps. |
| Submap ownership | `glim::SubMap::Ptr`, `frame`, `frames`, `odom_frames`, `id`, `T_world_origin` | The host retains the shared pointer until global insertion/export; pointer and ID sets reject aliases. `save(path)` is never used. |
| Global mapping | `glim::GlobalMapping::insert_submap`, `optimize`, `export_points` | The concrete implementation (not the base default) is selected. Every accepted submap is inserted once, optimize is called synchronously once, and export is reopened before host conversion. |
| Point ownership | `gtsam_points::PointCloud` and concrete `PointCloudCPU` returned by GLIM | Exported raw pointers are consumed before the owning `PointCloud::Ptr` leaves scope; size, pointer, finite values, homogeneous `w=1`, and optional intensity storage are checked. |

The abstract `GlobalMappingBase::export_points()` default (`nullptr`) is not
treated as a successful export.  A mapping plugin loaded through an unproven
abstract module path remains outside this phase and must fail closed.  The
public `save(path)` methods have no completion/status result and are therefore
not used.

## Transaction and ordering

```text
finalized EstimationFrames (host staged, identity checked)
    -> SubMappingPassthrough::insert_frame (exactly once each)
    -> get_submaps (destructive queue read)
    -> submit_end_of_sequence (exactly once)
    -> residual get_submaps proof
    -> GlobalMapping::insert_submap (exactly once each)
    -> GlobalMapping::optimize (synchronous)
    -> GlobalMapping::export_points
    -> bounded finite PointCloud -> one MapChunk
    -> AdapterBoundary::record_output_batch(trajectory + map)
    -> complete_drain
```

No output counter or output vector is published before the complete map has
passed all checks.  `record_output_batch` validates both trajectory and map
vectors, contiguous independent output orders, frame conventions, timestamp
monotonicity, and all counter capacity before changing a counter.  A map
failure therefore cannot expose a trajectory, and a boundary overflow cannot
expose a partial map batch.

The host bounds pending frames, submaps, exported points, and map chunks with
`RuntimeOptions::{max_map_frames,max_map_submaps,max_map_points,max_map_chunks}`.
The limits are explicit policy, not derived from `std::vector::capacity()`.
The exported map is one `MapChunk` with order zero and the contract's map
frame; point relative times and rings are deliberately zero because the
exported global map has no per-point sensor-time/ring semantics.  The map
contract does not infer a frame or silently reinterpret these fields.

The session's process-global GLIM configuration lease and callback limitations
remain unchanged: same-process arbitrary GLIM users and asynchronous callback
mutation require an isolated process.  Synchronous mapping exceptions, null
objects, duplicate identity/IDs, malformed transforms, missing points,
non-finite values, excessive counts, repeated EOS/finalization, and failed
export all latch a terminal error.  `close()` remains idempotent only after a
successful drain; failed sessions never claim a partial result.

## Configuration preflight

Before `GlobalConfig::instance()` or any mapping constructor, the host parses
`config.json` and always requires four nonempty relative component files:
`config_logging`, `config_sensors`, `config_preprocess`, and `config_odometry`.
When map output is requested it additionally requires
`config_sub_mapping` and `config_global_mapping`.  Every file must be a
regular non-symlink JSON object, and lexical/canonical containment is checked
against the explicit config root.  Mapping constructors therefore cannot
silently fall back to an absent component file.  The CMake target continues to
require the exact pinned nlohmann/json include directory explicitly.

## Synthetic adversarial coverage

The runtime fixture covers:

* complete two-frame trajectory plus one submap/global-map export, exact
  trajectory/map counters, one EOS, one drain, idempotent close, and one-shot
  output retrieval;
* a one-point `max_map_points` bound, proving mapping failure occurs before
  either trajectory or map publication;
* exact submap coverage: every staged estimation-frame ID, timestamp, and
  GLIM frame identity must occur exactly once in paired `odom_frames` and
  `frames`, submap IDs must be contiguous from zero, and the checked sum of
  accepted submap points must equal the exported point-cloud size;
* null/duplicate/empty/non-rigid map-output checks in the production path;
* config missing/malformed/absolute/traversal/symlink rejection before core
  construction;
* callback exception and ignored reentrant callback terminalization;
* same-process GlobalConfig lease rejection; and
* counter overflow/atomic trajectory+map batch accounting, mapping callback
  exception, and ignored reentrant callback failures before publication.

All fixtures are synthetic temporary files and typed sensor values.  No map
is written with GLIM's status-less persistence API.

## Evidence and readiness

The authoritative Phase 3c evidence roots and hashes are recorded below after
the final source state is built from fresh roots.  Previous Phase 3b roots
remain immutable historical evidence and are not silently relabeled.

| run | root | configure SHA-256 | build SHA-256 | CTest/direct/ldd/binary |
| --- | --- | --- | --- | --- |
| normal | `/tmp/glim-clean-room-phase3c-final4-normal.lZnuOV` | `939c5b321568593a1ec4d9a5d7cb4ed3749d9c023b7c399ba12d083b7593c94c` | `f3e5b2c3e8581762e5d6246405d74974737e88b5cbebf8762da25da644e9bec9` | CTest `2808e8c783e1c452f2929dd8c1fcf8644d8fe9c14d14809964288ca65d27767c`, direct `4430957b37a1002221bdb91a73c9a1980f2d6adcd0dd1111d4372e06dafacebc`, ldd `db7576a3f144b166fd76b76d4f9c5ce277df59841aed88575213a8462fcb04f8`, binary `e9b51ebb7eda9b206ea59edce29abc6d6b214310aaf8eff4423cfe2d65927e47`, receipt `16cd493a06bf5b9f8790ba6ef1b1cda6867cc5a0695e0f389d674b92440e4153` |
| ASAN/UBSAN | `/tmp/glim-clean-room-phase3c-final4-sanitize.tdZ586` | `7e2a3e778c057d049294d47b61cea7c131f77df6b8340178b7ab84d6193843db` | `f3e5b2c3e8581762e5d6246405d74974737e88b5cbebf8762da25da644e9bec9` | CTest `c2cd7acc2a5bd029680075d5f3190dcdb3285f9ba7c00e6c06e2773c55d3da9c`, direct `5b569580563929594e419b6c74a04b8f9d56744effbca2131e4a3b5f3ce8710d`, ldd `407b54f0dff8d03ec5bc3441dbd4781aaa6248f3defa3d983b167a7e2a1b50ea`, binary `68ebb24b7d3aadb18d2a3e937c83aa527b362542854d246ecd3979bba4e55f19`, receipt `016158bf2e72656d0d4561160ba8c5f1a9c88083b4562428a473b835ee5c4416` |
| Phase 0/3a regressions | `/tmp/glim-clean-room-phase3c-regressions-final3.uoETBK` | Phase0 normal `c489e94788861e1be1ab4e391c4bf6ce585e34dbc8816e970bf26d812838cb23`, sanitize `634be2407c8e22de784b697fdd18bbb9beed4e4edd9bb338ba47efb5ecc1a907`; Phase3a normal `24cf563605ac86cf0583f58d718a825b4e7fcf7281555788f0d3ae3c8743e5eb`, sanitize `671c65d56961f175a3b5c379cec7e7845dcf862c6c3eba817cd4821b7eb66640` | all four builds PASS | Phase0/3a CTest each 2/2; direct Phase0 each 15 groups, Phase3a each 8 groups |

The final4 roots use `direct.log` and `ldd.log` only for the prefix-loaded
PASS probes, and each `receipt.txt` reopens the required logs, recomputes
their SHA-256 values, checks the CTest 2/2 marker, the 11-group direct marker,
and rejects `not found` or `glim_ros2` in the ldd report.  The normal root also
preserves explicit no-prefix negative probes as
`direct-no-prefix-expected-fail.log` (exit 127, `libgtsam.so.4` absent) and
`ldd-no-prefix-expected-fail.log`; neither is used as positive evidence.

Phase 3c closes the previously unimplemented concrete map-export gap for this
isolated synchronous vertical slice.  ROS distro integration, live frontend
and backend wiring, hard interruption of non-cooperative core calls,
asynchronous/global callback isolation, arbitrary mapping plugins, and
benchmark claim eligibility remain `NOT_READY`.
