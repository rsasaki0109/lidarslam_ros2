# Registration-plugin contract matrix (2026-08-24)

This is a source/test audit of the versioned registration boundary used by the
scanmatcher frontend and graph backend/offline shell. It is not a Humble or
Jazzy runtime result and does not promote the external DSO path. “Strong
static” means the current source and synthetic/unit tests establish the
invariant without a distro container; “partial” records a deliberate
remaining contract gap.

| Requirement | Current implementation | Evidence | Status / remaining gap |
| --- | --- | --- | --- |
| Stable API and ABI identity | `ApiVersion`, runtime descriptor, ABI epoch, compiler/libstdc++ tag, and interface-contract digest in `registration.hpp`; loader validates the sidecar before construction and the provider descriptor after configuration | `test_registration_plugin_loader` contract-manifest tests plus provider/source audits | **Strong static, runtime NOT_READY**: the contract scope is explicit, but a raw header/interface digest is not a complete C++ ABI proof and distro/toolchain rebuild evidence remains required. |
| Exact major/minor and capability negotiation | `isApiCompatible()` requires equal major and plugin minor no newer than host; `CapabilityRequirements` validates every requested bit/policy/metric | loader capability/API tests; scanmatcher/backend preflight helpers | **Strong static** for declared fields. Unknown future fields and explicit ABI identity are not negotiated. |
| Class, DSO, and plugin-XML provenance | class ID must match metadata and provider; install-time sidecar binds canonical XML/DSO size and SHA-256, class, schema, API range, capabilities, policies, config schema, and manifest digest | `readAndValidateRegistrationContractManifest`, symlink/missing/byte-drift tests, post-construction descriptor drift tests | **Strong static, installed-prefix and distro-specific ELF/ODR/load evidence still NOT_READY.** |
| Loader lease, lifetime, and unload | `RegistrationPluginSession` owns the pluginlib `ClassLoader` before the plugin pointer; host-adopted PCL objects use the same session lease; transaction destruction/reset order is explicit | external-session lease tests; host-session adoption; activation commit/rollback tests | **Strong static**. No distro runtime unload smoke has been completed. |
| Transactional prepare/validate/commit/rollback | host-owned slots are untouched until validation; commit swaps plugin before session; rollback rejects unexpected concurrent mutation | activation reject, validator-throw, concurrent-mutation, commit/rollback tests | **Strong static**. External DSO constructor/static-initializer side effects remain outside the rollback boundary. |
| Constructor, resource-init, processing, and destructor exceptions | loader catches discovery/create/metadata/config exceptions; startup resource failure rolls back; session wrappers catch `setInputTarget`/`align` exceptions and latch fault | loader constructor/config tests; resource-init tests; session processing exception tests | **Strong static** for declared boundaries. A plugin violating `reset() noexcept` or throwing from its destructor terminates by the C++ contract; it cannot be recovered by the host. |
| Thread safety, reentrancy, and ownership | session serializes `kSerializedOwner`; `kReentrant` is allowed to run concurrently; an in-flight counter and condition-variable barrier prevent reset/unload during callbacks; session owns processing state and the DSO lease | bounded C++14 stress harness (non-TSAN/TSAN modes), source guards, loader concurrency tests | **Strong static, stress evidence when the dedicated test is run**: `kCooperativeCancel` and `kNonInterruptibleAlign` are explicit; a non-interruptible provider may delay shutdown until its current call returns. |
| Deterministic configuration schema | `ParameterMap` is typed and ordered (`std::map`); provider and sidecar bind schema ID/version/SHA before publication | interface parameter tests; synthetic contract-manifest and descriptor-drift tests | **Strong static** for declared schema identities; schema bytes and distro runtime installation remain NOT_READY. |
| Cancellation and shutdown | `cancel()` blocks future admission and performs pre/post checkpoints; cooperative providers receive `requestCancel()`; idempotent `shutdown()` waits for quiescence before `reset() noexcept` | bounded stress harness covers cancel-before/during/after, provider observation, repeated shutdown, fault latch, and timeout bounds | **Strong static with bounded stress characterization**. PCL built-ins remain `kNonInterruptibleAlign`: cancellation cannot hard-interrupt an active call and shutdown may wait for it to return. |
| No host-state leak | startup transaction keeps candidate state private; no ROS resources are exposed before commit; failures preserve the previous host slots; built-in resource-init failure rolls back the adopted session | activation and scanmatcher injection/resource-init tests; source boundary audit | **Strong static** for host-owned state; external static initialization and arbitrary DSO global side effects are explicitly not rollbackable. |
| Backward/forward compatibility | API major/minor rule rejects incompatible major/newer plugin minor; appended failure code preserves existing enum values; old raw accessor remains source-compatible | API compatibility tests; loader mismatch tests | **Partial**: no ABI identity, config-schema negotiation, or explicit capability-version policy for forward extensions. |
| External C++14 SDK/install surface | interface is ROS-free/header-level C++14; consumer/template checks bind source manifest and install metadata | `run_registration_plugin_consumer_check.sh`, template check, matrix audit | **Strong static**. Humble/Jazzy clean install/build/load and external DSO runtime evidence remain NOT_READY. |
| Observability and failure reason | `LoadFailure` has stable reason codes/messages; alignment diagnostics carry detail; wrapper exceptions become structured internal/cancelled failures | loader negative tests; scanmatcher failure-name mapping; wrapper tests | **Strong static**. Production log/receipt schema for processing fault transitions is not yet a distro-runtime artifact. |

## Wiring boundary

Every supported live scanmatcher selector (NDT, GICP, and dependency-enabled
FAST/SMALL variants) is now host-resolved into a typed adapter and committed
through `RegistrationPluginSession`; the historical PCL defaults remain in
their same translation unit and parameter maps. Live and offline graph NDT
and GICP use the same session boundary. The GICP bridge is an explicitly
preconfigured host adoption, never an implicit pluginlib fallback.

`BackendCore` retains its ROS-free `RegistrationPlugin&` API, but live/offline
shells pass a `RegistrationPluginSessionAdapter`, so target and alignment
calls still use the lease, serialized ownership, exception latch, cancellation,
and idempotent shutdown semantics. The source auditor
`scripts/audit_registration_plugin_processing_boundary.py` rejects raw
`align`, target, source, convergence, and fitness calls in the three
production shells. The standalone `small_gicp_odom_node` and
`map_ndt_residual_report_main` are explicit, separately documented analysis /
odometry seams and are not live scanmatcher/backend consumers.

## Runtime disposition

The current runtime status is **NOT_READY**. No Humble/Jazzy container, live
sensor run, bag replay, benchmark, map, ground-truth, scorer, or external-DSO
promotion is inferred from this static slice. The remaining release blockers
are distro-specific C++14 consumer/install proof, ABI identity policy,
plugin-declared configuration schema identity, dependency-enabled DSO/ODR
runtime evidence, and Humble/Jazzy execution of the dedicated stress test.
