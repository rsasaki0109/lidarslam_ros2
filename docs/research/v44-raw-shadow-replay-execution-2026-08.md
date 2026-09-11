# v44f raw shadow replay execution result (2026-08-10)

## Decision

The first authorized v44e raw-shadow run failed the frozen processing-resource
gate. The route decision is therefore:

`REJECT_V44_STAGE4_RAW_SHADOW_REPLAY_RESOURCE_GATE`

The attempt was `navinst_indoor02`, repetition 1, which is the first item in
the sealed six-run order. The core terminated immediately after scan index 0
with `processing RTF capacity exceeded after scan`. The v44e contract states
that any one failed run rejects the route, so repetitions 2 through 6 were not
started. No threshold was changed and the failed output was not deleted or
overwritten.

The execution-result contract is
`configs/sota_v6/development/v44f_raw_shadow_replay_execution_audit.json`
(SHA-256 `3da6877b93a386fe875bfefc3844d9105e1cb90ec5820ac86ae7364d97f330b7`).
The report-only auditor is
`scripts/audit_v44f_raw_shadow_replay_execution.py` (SHA-256
`9e0292e0055b06663314c5f7fe04cd82af17e2ca203e56265fda913c21882675`).

## First-run evidence

The adapter first verified its v44e authorization chain and the exact NavINST
bag SHA-256. The first scan lacked a complete earlier IMU bracket and was
recorded as the contract-permitted `dropped_unbracketed_prefix`. When the
adapter supplied the mandatory post-scan runtime observation, cumulative
processing RTF exceeded the frozen `0.85` ceiling. The estimator entered its
terminal path before bootstrap or state creation.

The run evidence is retained under:

`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44e_raw_shadow_replay_contract_20260810/raw_replay/navinst_indoor02/run_01/`

- `run.json`: SHA-256
  `1913305fe4b3a8ded99c62aef3026428443643ecaa281a41c7fe0de6a1bd0b4a`
- `diagnostics.jsonl`: SHA-256
  `32a5764f1db2abdd4b46d2eefb48ce4e69a6481ec93433b035f7cf6ced247639`
- adapter report payload: SHA-256
  `1194b8b69b1a35183ba8f33046ec494df9660f6dacc3b76823040b4680a8396a`
- core diagnostic payload: SHA-256
  `cd9f3b847d7169d51b99924f13e5b578737b0477e4b6dc3f665a9112302671bd`

The terminal result has `valid_shadow_result=false`, active state count 0, and
a null state payload SHA-256. Both protected v17 artifacts are byte-identical
before and after:

- state: `81054c8c81af58b6c0e47d85e5f942676addfea2f9a1084e5d9023772a49d64e`
- map: `be95206f6e1c86c2e09e81467f5056a625bb62dd486b267de0d58fd0dae4f00f`

Accuracy/reference-map inputs were not accessed, primary trajectory/map was
not mutated, and no ROS output was published. Because processing stopped early,
the report's completed-stream inventory is null; no claim is made that the
entire bag was consumed.

## Diagnostic limitation

The exact offending RTF numeric value is not present in v44e evidence.
`FixedLagShadowEstimator.record_runtime_observation` checks the limit before it
replaces the scan's placeholder runtime fields. On rejection, the scan retains
`processing_rtf=0.0`, while the terminal reason records the authoritative
capacity failure. This does not make the gate ambiguous—the attempted value was
strictly greater than 0.85—but it prevents attribution of the excess to decoder,
reorder, object construction, or estimator work from this evidence alone.

Any next investigation must first define a bounded failure-profile contract
that records the attempted RSS/RTF and fixed phase timings before invoking the
unchanged gate. It may not continue the remaining raw runs, reinterpret the
placeholder as a measured value, relax 0.85, or open accuracy data.

## Independent execution audit

The v44f auditor does not import `rosbags` or reopen a bag. It verifies the
v44e contract and static authorization hashes, adapter/core hashes, the exact
failed report and diagnostic hashes, canonical two-record diagnostic order,
null state, unchanged v17 hashes, closed forbidden routes, and the complete
raw-output path inventory. The latter proves that only run 1 exists and the
remaining five runs were not attempted.

All 18 checks pass. The focused v44f suite passes 16 tests, and the complete
v40--v44f regression passes 277 tests in 25.38 s. Two standalone audits have
the same deterministic report payload SHA-256
`29ef8cef8237544110e776662fd3e73ddc07f7aedd751c7a581a9c35149711bd`.
Their file SHA-256 values are
`b0b96e016143d5f42d2661deb21994dea14889be21f489b87d643b20842dd323`
and
`3a5fe3b2f1bda2c45d9973782c7aa8d301cf2b2b95ddc5efea657710e040861f`.
The aggregate JSON SHA-256 is
`9b4dbbaede2239ae619594b6c84ee0f33b7fc89e360bfac777a5c37c37a2e2eb`,
with deterministic aggregate payload SHA-256
`f9a9b25e746f49f5b2dc9665dba1a8019c59bfb2b044897f0561750a1da31073`.
Evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44f_raw_shadow_replay_execution_audit_20260810/`.

The audit status is PASS because the rejection evidence is complete and
repeatable. Raw replay continuation, an accuracy screen, primary mutation, and
promotion remain unauthorized. Only definition of the bounded failure-profile
contract described above is authorized next.
