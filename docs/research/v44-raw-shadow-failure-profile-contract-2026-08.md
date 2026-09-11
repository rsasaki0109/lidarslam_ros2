# v44g raw-shadow failure-profile contract (2026-08-13)

## Decision

The v44f route remains rejected. The next boundary is now defined and
statically audited as:

`AUTHORIZE_V44G_FAILURE_PROFILE_CONTRACT_DEFINITION_ONLY`

This is authorization to define the diagnostic contract only. It does not
authorize opening a raw bag, invoking the adapter, continuing the remaining
v44e runs, changing the estimator, relaxing the `0.85` processing-RTF or
`330 MiB` RSS limits, opening accuracy/reference-map inputs, or mutating the
primary trajectory/map.

The contract is:

`configs/sota_v6/development/v44g_raw_shadow_failure_profile_contract.json`

SHA-256: `b985b4454908619215d0f41c1c26845d244790f23b6a40d2dda9677e1935c281`.
The static auditor is
`scripts/audit_v44g_raw_shadow_failure_profile.py` with SHA-256
`d03f1ee399815f04401ab7cadaf5ca7234b662dc133357ae3ec9d1a7901a0839`.

## Frozen boundary

The contract binds the completed v44f rejection audit and its aggregate, the
v44e execution contract, and the exact adapter/core hashes. It targets only
the already attempted first failure: `navinst_indoor02`, repetition 1, scan
index 0. A future implementation may not broaden that target, run another
sequence, or consume another scan under this definition.

The required diagnostic phases are fixed and non-overlapping:

1. `decoder`
2. `reorder`
3. `core`

All three phases count toward the same cumulative processing-RTF value used
by v44e. The profile must capture its values before evaluating the unchanged
RSS/RTF gate. The required fields include attempted RSS, attempted processing
RTF, sensor interval, per-phase and cumulative wall time, gate order, and the
terminal reason. A placeholder `0.0` RTF is not valid profile evidence.

The formula remains:

`(cumulative_decoder_wall_ns + cumulative_reorder_wall_ns + cumulative_core_wall_ns) / sensor_elapsed_seconds`

The profile clock is diagnostic-only monotonic nanoseconds; it cannot enter
estimator state or sensor time. Output is limited to four records and 64 KiB,
with no overwrite and diagnostics only.

## Static evidence

Two independent definition audits were run without importing a raw decoder,
opening a bag, or invoking the shadow adapter. Both passed all 15 checks and
produced the same deterministic report payload:

`3a057ba8ad418a64a0639af04f046b98ec51fb8e12821cbcf28668113169d8a7`

The retained evidence is under:

`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44g_failure_profile_contract_20260813/`

The aggregate is SHA-256
`6964094a85c746bee7f4081ee3173d6db21d604fe6ea9ed518d6955f01405610` and its
deterministic aggregate payload is
`83d4d0b742ce44c7149c9e703389b6b206a0affa075ac31b9b60b121372c92ac`.

The focused contract suite passes **12 tests**. The aggregate decision keeps
failure-profile execution, raw replay continuation, accuracy, primary
mutation, and threshold relaxation all false. The next authorized action, if
separately undertaken, must implement this profile around the existing first
failure without changing the gate; this document itself does not authorize
that execution.
