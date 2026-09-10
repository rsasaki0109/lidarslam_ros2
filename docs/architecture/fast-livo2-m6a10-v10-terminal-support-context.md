# FAST-LIVO2 M6a10 v10 terminal support-context contract

M6a10 v10 is an additive, algorithm-neutral observation contract for the
FAST-LIVO2 online-compute phase. It addresses the evidence gap retained by
the v9c closure: all 236,687 input messages published and callback-ACKed, but
the terminal consumer state was not proven quiescent and an aggregate backlog
of 81 remained at the bounded drain deadline. The v9c closure remains
immutable and is referenced by the v10 profile; this document does not turn
that result into a pass.

The v10 source delta is opt-in through
`m6a10-online-compute-v3-terminal-support-context`. It does not modify the
FAST-LIVO2 estimator, synchronization policy, image handling, or map output.
The existing v2 validator and its rules are unchanged. The generic validator
dispatches to v3 only when both schema version 3 and the exact v3 contract
version are present; mixed metadata is rejected.

## Evidence model

The mapper writes an independent consumer document under
`m6a10-fast-livo2-consumer-terminal-v1`. It contains only accepted callback
counts, completed/discarded dispositions, the backend completion boundary,
and the locked terminal-buffer proof. It does not claim feeder publication or
ACK observations. The host compositor in
`scripts/compose_fast_livo2_terminal_evidence.py` binds that document to the
immutable feeder receipt and writes the one schema-3 document containing:

- exact `expected`, `published`, `received`, and `acknowledged` counts for
  lidar, IMU, and image;
- a stable backend `completed_boundary` with timestamp, sequence, source,
  completed counters, quiescence, and `in_flight: {active: false}`;
- an exact per-topic buffer snapshot, with count, oldest/newest timestamps,
  and every residual record timestamped and identified;
- a `terminal_support_context` classification and per-topic counts; and
- the required evaluation end timestamp plus trajectory coverage.

For each topic, the validator enforces the conservation equation

```text
received[topic] = completed_counts[topic] + support_context_counts[topic]
```

with zero drops, zero overflow, and zero processing failures. This is a
statement about evidence boundaries, not an estimator result.

## Residual records

Residual lidar is always invalid because it can participate in a later valid
FAST-LIVO synchronization unit. Only IMU and image residuals may be terminal
support context. Every allowed residual must have:

1. a record identifier and finite timestamp;
2. a timestamp strictly later than the completed backend boundary;
3. `support_context_proven: true` and `post_boundary: true`;
4. `can_form_synchronization_unit: false`; and
5. reason code `strictly_after_completed_boundary`.

Support context is not processed and is not dropped. The contract rejects
processed/dropped labels on residual records. Missing per-topic counts,
missing timestamps, pre-boundary records, a non-quiescent backend, an active
in-flight unit, count-conservation mismatch, or an uncovered required end
always fail closed. A classic zero-backlog snapshot follows the same schema
and passes with zero support-context records.

## Source and safety boundary

`docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch`
adds the opt-in observation helper, mapper member, terminal services, and
source call sites. Accepted callbacks are entered only after the base queue
push; every input FIFO pop and clear has a disposition hook; estimator
completion is recorded only after `stateEstimationAndMapping()` returns; and
the terminal service snapshots all residual records while holding the mapper
buffer lock. Aggregate queue sizes alone remain insufficient. If a source path
cannot provide completed counters or a stable record-level snapshot, the
generated document remains invalid.

The additive source delta was compiled in the pinned v9 workspace and in the
v10 image recipe. A no-input, network-isolated, read-only-root service gate
confirmed that terminal services respond and finalize an explicitly invalid
document with no ground-truth or scorer access. This is an implementation
gate, not a formal replay or runtime PASS claim.

The host compositor rejects missing, stale, duplicate, or hash-mismatched
receipts; checks exact bag/profile identity and topic counts; and always calls
the generic v3 validator before an output can be written. Its output is
atomic and immutable. The required end timestamp is preregistered as
`1623491515.148352` from
`/media/sasaki/aiueo/benchmarks/m6a10_training_20260822/ntu_tnp01_training_validation_summary.json`
with SHA-256
`4217a4b07f5ff85148e7433be1b9fef51e35d843a98bbb287d6f9c010c254177`.

The v10 profile is
`configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml`. It records
the compile/no-input implementation gate and explicitly forbids formal
replay, ground-truth mounts, scorer invocation, commits, and pushes at this
stage. The corrected
input digest is the 64-character SHA-256
`5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310`.

The v9c predecessor references are retained for audit:

- original immutable closure SHA:
  `467d85dfb714a4f5b71c5058c5dd39e1d42d57d6cda3c79ee950535699181682`;
- correction sidecar SHA:
  `d77e26c83abc4fe10f874a43ff45f62e8120f223df919075f4a956ef47569b3b`.

The 1200-second watchdog in the profile is a future supervision bound only;
it is not an algorithm parameter and is not exercised by this implementation
task.
