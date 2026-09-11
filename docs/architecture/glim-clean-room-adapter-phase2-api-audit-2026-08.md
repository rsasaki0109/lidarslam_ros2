# GLIM clean-room adapter Phase 2 public-core API audit (2026-08)

This is an additive, host-toolchain-only API audit.  It does not implement the
adapter, run a sensor stream, run a benchmark, open a dataset/GT/scorer path,
or change an active profile, selection, receipt, or the existing `r2` recipe.
The Phase 0/1 clean-room boundary remains the only production boundary.

## Immutable audit inputs and scope

The audited GLIM checkout is the preserved official-core source at
`/tmp/competitive-rival-closure.sXE92h/glim`, commit
`faa264a1bce1bda406f73457e35511f56cdc2eaa`.  Its working tree was clean.  The
installed public surface is the preserved Phase 1 prefix
`/tmp/glim-clean-room-phase1.mwzqdq6t/install/*`; no source or installed file
was modified by this audit.  The installed `glim/include/glim` regular-file
manifest has 54 files, 5,092 manifest bytes, and SHA-256
`937097c5e1f6fb8aecc02e152560a1db114539929ad12ce8b6b4e4a14dad4ba9`.

Only public GLIM core headers, the exact GLIM core source checkout, and the
preserved installed libraries were inspected.  ROS bridge headers, bridge
source, bridge patches, and bridge build products were deliberately excluded.
The additive source guard scans the Phase 2 probe and rejects bridge tokens or
unexpected ROS dependency discovery; the final CTest source-guard case passed.

## Public data and processing surface

The following table records what the public API actually says.  “Evidence”
means header/source inspection; it is not a claim that the host adapter has
implemented that operation.

| Area | Exact public surface | Semantics established by the core | Boundary consequence |
| --- | --- | --- | --- |
| Raw points | `RawPoints::stamp`, `times`, `intensities`, homogeneous `points`, optional `colors` and `rings`; `Ptr`/`ConstPtr`; `size()` | `stamp` is the first-point timestamp; `times` are relative to that point. The type itself does not validate units, frame ID, finiteness, or vector lengths. | `SequenceContract` must validate field layout, units, frame, lengths, ordering, and finiteness before constructing this object. |
| Preprocessed frame | `PreprocessedFrame::stamp`, `scan_end_time`, `times`, `intensities`, homogeneous `points`, `k_neighbors`, flat `neighbors`, `raw_points` | Scan start/end and relative point times are doubles. No frame ID or EOF state is carried. | Preserve the host frame/calibration identity outside this type and validate output ordering before a sink callback. |
| Preprocessing | `CloudPreprocessor(const CloudPreprocessorParams&)`; `preprocess(const RawPoints::ConstPtr&) -> PreprocessedFrame::Ptr` | Params include distance limits, global-shutter mode, downsampling/outlier/cropbox controls, `crop_bbox_frame`, `T_imu_lidar`, correspondence count, and thread count. Source applies callbacks, filtering, sorting, and neighbors; invalid crop configuration can throw. | The host must bind params/config and catch exceptions at this boundary. Core preprocessing is not documented as reentrant/thread-safe. |
| Point time | `PerPointTimeSettings`; `TimeKeeper::process(const RawPoints::Ptr&) -> bool`; `validate_imu_stamp(double) -> bool` | Core supports configured absolute/relative input and a scale-to-seconds setting, checks rewind and rough IMU gaps, and may rewrite frame/point stamps. It has no EOF method. | Keep integer nanoseconds and explicit units in the host contract; perform checked conversion to core doubles only after validation. |
| IMU | `IMUIntegration::insert_imu(double, Vector3d, Vector3d)` and range/find/erase methods; `OdometryEstimationBase::insert_imu` and mapping-base `insert_imu` have the same triple | Values are doubles; `IMUIntegration` owns an internal queue/preintegration state. No cancellation, EOF, or unit/frame identity is represented. | Host validates m/s², rad/s, frame, stamps, and terminal state before forwarding. |
| Calibration/frame | `CloudPreprocessorParams::T_imu_lidar`; `OdometryEstimationIMUParams` IMU extrinsic; `EstimationFrame::T_lidar_imu`, `T_world_lidar`, `T_world_imu`, `FrameID {WORLD,LIDAR,IMU}` | Transforms and enum frame IDs are present, but there is no public sensor-frame string or orthonormality/inverse validator. `custom_data` uses unchecked `reinterpret_cast`. | `SequenceContract` remains authoritative for frame names, transform convention, rigidity, inverse, and ownership; do not infer these from a header field. |
| Odometry CPU/IMU | `OdometryEstimationCPU(const OdometryEstimationCPUParams&)`; CPU derives from `OdometryEstimationIMU`; base `insert_imu`, `insert_frame`, `get_remaining_frames` | `insert_frame` may return null while IMU initialization is incomplete. `get_remaining_frames()` is the exposed drain-like operation. The source can catch selected smoother range errors, but the public boundary is not an exception translation boundary. | The host must distinguish submitted/accepted/processed frames and treat null/remaining-frame behavior explicitly. Idempotent drain is not proven. |
| Submapping | `SubMappingBase::insert_imu`, `insert_frame`, `get_submaps`, `submit_end_of_sequence`; concrete `SubMapping` and `SubMappingPassthrough` | EOF can force a final submap in the concrete implementation; `get_submaps()` drains/swaps the result queue. No base terminal-state or idempotence contract is present. | Host owns one ordered EOF event and must reject post-EOF input and duplicate output. |
| Async submapping | `AsyncSubMapping(shared_ptr<SubMappingBase>)`, thread-safe documented `insert_*`, `join()`, `workload()`, `get_results()` | Constructor starts a worker. `join()` is a soft end-of-sequence drain; destructor sets a hard kill flag and joins. | Useful lifecycle primitive, but no host fault latch, callback exception translation, cancellation capability descriptor, or proof of output-once behavior. |
| Global mapping | `GlobalMappingBase::insert_imu`, `insert_submap`, `find_overlapping_submaps(double)`, `optimize`, `recover_graph`, `save(path)`, `export_points()` | Concrete save writes a graph/trajectory/submap directory; export merges submap points. There is no generic EOF method. | `on_drain_complete` must not pretend that `save` is an EOF primitive; the host must sequence join, final submaps, save/export, and receipt sealing explicitly. |
| Async global mapping | `AsyncGlobalMapping(shared_ptr<GlobalMappingBase>, int)`, thread-safe documented input methods, `join`, `workload`, `save`, `export_points` | All exposed methods except `save` are documented thread-safe. `save` is expected after `join`; destructor hard-stops and joins. | Host must serialize save/export after join and treat destructor hard-stop as bounded cleanup, not a guaranteed interrupt of core work. |
| Trajectory | `TrajectoryManager::add_odom(double, Isometry3d, int)` and `update_anchor(double, Isometry3d)`; pose conversion accessors | Uses double stamps, priority and interpolation/slerp. No explicit monotonicity, frame-string, or thread-safety contract is exposed. | Host output validator remains mandatory for contiguous order, monotonic integer event stamps, frame, finite/unit quaternion, and required rows. |
| Callbacks | `PreprocessCallbacks`, `OdometryEstimationCallbacks`, `SubMappingCallbacks`, `GlobalMappingCallbacks`, all built from `CallbackSlot<std::function<...>>` | `CallbackSlot` stores a vector without a mutex; callback exceptions propagate; removal has no bounds check. Some callback docs explicitly warn that cross-thread access is unsafe. | Callbacks must be isolated behind the host sink and translated into a terminal fault; they cannot be treated as an independent thread-safe event bus. |
| Config | `Config(string filename)`, typed `param<T>`, volatile overrides, `save`; `GlobalConfig::instance(path, override_path)` and `get_config_path` | JSON/config lookup is path-based and missing/cast-invalid values can abort/throw in source. | Config path, bytes, schema, and revision must be prebound by the host; no result-dependent config selection is supported. |
| Dynamic loading | `load_module_from_so(so, symbol)` calls `dlsym` for a C factory and wraps the returned raw pointer in `shared_ptr`; factories are `extern "C" Base* create_*()` | No public ABI/version/capability negotiation, destroy function, manifest check, or exception boundary exists. Default `shared_ptr` deletion assumes compatible allocation/deallocation. | This is a compile/link fact only. A future adapter must add preflight provenance and ownership checks without claiming that this raw factory is a complete plugin contract. |

## Mapping to the host contract

The current host-owned contract is in
`docker/benchmark_adapters/glim_clean_room/include/glim_clean_room/contract.hpp`.
Its `AdapterBoundary` is the validation/state/counter boundary and its
`CoreSink` is intentionally still a pure seam.  The exact mapping below is a
design map, not an assertion that Phase 2 implemented the adapter.

| Host method | Candidate GLIM call sequence | Compile proof | Exact unmapped requirement |
| --- | --- | --- | --- |
| `CoreSink::on_lidar(const LidarFrame&)` | Validate `LidarFrame`; construct `RawPoints`; `CloudPreprocessor::preprocess`; pass `PreprocessedFrame::Ptr` to `OdometryEstimationBase::insert_frame`; forward resulting/marginalized frames to `SubMappingBase::insert_frame`. | `preprocess` and `insert_frame` member signatures; `RawPoints`/`PreprocessedFrame` constructibility. | No public atomic transaction across preprocessing, odometry, and mapping; no core cancellation; no frame-ID/units validation; exception and partial-side-effect behavior must be supplied by the host. |
| `CoreSink::on_imu(const ImuSample&)` | Validate stamp/units/frame; call odometry, submapping, global mapping and/or `IMUIntegration::insert_imu` in the configured order. | Exact IMU signatures for all three base surfaces and `IMUIntegration`. | Core accepts doubles and does not expose a shared event transaction or rollback. Ordering and duplicate policy are host policy. |
| `CoreSink::on_eof()` | Stop input; call `SubMappingBase::submit_end_of_sequence`; drain `get_remaining_frames`; consume submaps; request async `join()`. | `submit_end_of_sequence`, `get_remaining_frames`, async construction are asserted. | Odometry and global mapping have no common EOF. Concrete EOF and async join idempotence are not proven; a host terminal latch is required. |
| `CoreSink::on_begin_drain()` | Collect remaining odometry frames, submap results, and async queues; submit final submaps to global mapping. | `get_remaining_frames`, `get_submaps`, `Async*::get_results`, and `insert_submap` signatures are available. | No single drain API or completion token exists; the host must define queue quiescence and output ordering. |
| `CoreSink::on_drain_complete()` | After joins, call global `save(path)` and/or `export_points()`, then convert to host trajectory/map records. | Global save/export and trajectory methods are asserted. | Save has path-side effects and no status return; export may be null/default; GT/scorer paths must remain forbidden. |
| `AdapterBoundary::record_trajectory` | Consume odometry/trajectory callback data or `TrajectoryManager` samples. | `TrajectoryManager` signatures and `EstimationFrame` type are asserted. | Core does not promise contiguous IDs, monotonic stamps, finite/unit quaternions, or one output per input. Host validation is mandatory. |
| `AdapterBoundary::record_map_chunk` | Consume `SubMap::Ptr`/`GlobalMappingBase::export_points()`. | `SubMap` result and global export signatures are asserted. | Core does not define host chunk order, frame string, map completeness, or output sealing. |

The contract's checked counters and terminal states are therefore not
redundant with GLIM.  They provide the missing preflight, rejection, fault,
EOF, drain, output-order, and artifact-role guarantees.  No claim is made that
the current `GlimCoreEngine` is wired to these methods in Phase 2.

## Public factory and lifecycle evidence

The installed CMake export exposes `glim::glim`,
`glim::odometry_estimation_cpu`, `glim::sub_mapping`, and
`glim::global_mapping`.  The preserved DSOs export the exact C factories:

```text
glim::OdometryEstimationBase* create_odometry_estimation_module();
glim::SubMappingBase*         create_sub_mapping_module();
glim::GlobalMappingBase*      create_global_mapping_module();
```

The probe resolves these symbols at link time but deliberately does not call
them.  Calling a factory would construct a configured core object and would
cross from audit into runtime behavior.  The source factory implementations
return `new OdometryEstimationCPU(params)`, `new SubMapping(params)`, and
`new GlobalMapping(params)` respectively.  The public loader's raw-pointer to
`shared_ptr` conversion proves neither allocator compatibility nor unload
lifetime; those remain an explicit future gate.

The probe also proves the public construction shape for CPU odometry,
submapping, passthrough submapping, global mapping, async submapping, and async
global mapping, plus extension/config lifecycle signatures.  It does not
pretend that an abstract `OdometryEstimationIMU` can be instantiated: the IMU
API is proven through the base method signatures and the concrete CPU-derived
type.

## Phase 2 compile/link probe

The additive probe is
`docker/benchmark_adapters/glim_clean_room/phase2/api_compile_probe.cpp`,
with CMake in `phase2/CMakeLists.txt`.  It includes only the preserved public
install headers, links the preserved GLIM/GTSAM/gtsam_points libraries, and
contains 41 `static_assert` checks for:

* raw/preprocessed point and timestamp/preprocessing signatures;
* IMU integration and deskew signatures;
* odometry, submapping, async lifecycle, global mapping, save/export, and
  trajectory signatures;
* callback slot types, extension/config lifecycle types, and all three C
  factory types;
* concrete constructibility and default aggregate/config shapes.

The install's exported target uses an include directory one level below the
public include root, so the probe explicitly supplies the preserved public
include roots.  This records an install-export quirk; it does not modify the
Phase 1 install or claim an adapter fix.

The probe executable only takes factory addresses in `main()`.  It does not
construct a module, feed points/IMU, access a map, or read a data/GT/scorer
path.  CTest also runs the clean-room source guard over the additive adapter
tree.

## Mapped versus unmapped API requirements

The public call signatures needed for the proposed input, odometry,
submapping, global mapping, trajectory, callback, and factory edges are
mapped and compile-proven in the table/probe above.  The following are
intentionally **unmapped**, rather than guessed:

1. A single transactional operation spanning preprocess → odometry →
   submapping → global mapping, with rollback after a downstream exception.
2. A public cancellation token or hard interruption guarantee for CPU
   registration, smoothing, or mapping; async destructor hard-kill is only a
   bounded worker shutdown mechanism.
3. Callback synchronization, callback exception translation, callback
   lifetime/unregistration safety, and callback output-once semantics.
4. A common EOF/drain/flush protocol across odometry, submapping, and global
   mapping; `get_remaining_frames`, `submit_end_of_sequence`, and `join` are
   separate mechanisms.
5. Idempotence of odometry drain, submap EOF, async join, and output retrieval.
6. Sensor frame strings, timestamp units, calibration rigidity/inverse
   conventions, and the host's integer-stamp/order policy.
7. ABI epoch/API capability negotiation, plugin manifest provenance, and a
   safe DSO destroy/unload lease for the raw C factory.
8. Complete trajectory/map output completeness and deterministic ordering.
9. A core status/error return for `save`; the public method returns `void` and
   may write several files.

These gaps are why this document is an audit and not a runtime readiness or
benchmark result.  A future Phase 3 implementation must close them at the
host boundary or record a fail-closed reason; it must not infer semantics from
the signatures alone.

## Reproducible evidence

The final host-only run used the fresh root
`/tmp/glim-clean-room-phase2-api-audit-final.caeh3X` after the complete probe
assertion set (41 assertions) was present.  Its preserved logs were:

| Artifact | SHA-256 | Result |
| --- | --- | --- |
| `configure.log` | `9bcd03ea69f8bb4ca824e5e8032efa9c194b54d97ee390baa99fe56d8966cd7e` | configure PASS |
| `build.log` | `0cceb72577163cd5dbfc837208a12c1e660783b7e615c98b1390d45e2b08dcd0` | compile/link PASS |
| `ctest.log` | `ba9aa5496b5d6c4a528d9b55d943626f35a51141802d93dc8360ecf90dda17f7` | 2/2 PASS |

The earlier failed roots remain preserved and are not overwritten:

| Root | Failure retained |
| --- | --- |
| `/tmp/glim-clean-room-phase2-api-audit.UMMxjP` | Initial compile failed because the preserved exported target did not supply the public `${prefix}/include` root. |
| `/tmp/glim-clean-room-phase2-api-audit-rerun.bFTwhQ` | After the include fix, the probe first needed preserved-library `LD_LIBRARY_PATH`, and the source guard CTest command needed an explicit Python interpreter; no data was processed. |

The additive adapter tree currently contains 23 regular files (generated
bytecode excluded), with 3,137 canonical manifest bytes and manifest SHA-256
`9856aa17dbfd8a87f64fb026d134795d2e87e0758443e0444783310d559c545b`.  The
manifest is repo-relative and excludes this architecture document, so the
record is not self-referential.  The final Phase 2 source guard checked 17
production files.

## Status and next gate

Phase 2 status is **API signatures mapped / semantic adapter unmapped**:
seven host entry points have an exact candidate public-call mapping above; nine
semantic/lifecycle groups remain explicitly unmapped.
The 2/2 compile/source-guard CTest is host evidence only.  Runtime adapter
implementation, ROS distro integration, dataset execution, GT/scoring,
benchmark eligibility, and active-profile readiness remain **NOT_READY**.
