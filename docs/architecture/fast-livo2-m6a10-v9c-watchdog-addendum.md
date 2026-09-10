# FAST-LIVO2 M6a10 v9c bounded-supervision addendum

This addendum records the v9c supervision-bound derivation and the input
identity correction. It does not change FAST-LIVO2 estimator or mapping
semantics. The global watchdog is a host supervision bound only; it is not an
algorithm parameter and does not alter the v9 image, feeder, callback, queue,
or phase contracts.

## Immutable v9b observation

The v9b single-replay closure is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/fast_livo2_v2c_v9b_formal_replay_20260823T043914Z_agentv9b/closure_receipt.json`
with SHA-256
`0bb37c4dddcc1edc05f955f7414bb5ac5ae2b46f071ce1f7909a55c646a354d1`.
It recorded 226,700 published and acknowledged records in
903.156884 seconds of watchdog supervision, with 9,987 records still
remaining. The container exited 137 after the authorized watchdog action;
Docker reported `oom_killed=false`, so this is retained as a bounded-watchdog
closure rather than an OOM claim.

The observed ACK rate used for the bound is

```
226700 / 903.156884 s = 251.00843941527216 records/s
```

At that observed rate, the complete 236,687-record phase projects to
942.9443908394707 seconds. Adding the preregistered 60-second EOF/drain
timeout gives 1,002.9443908394707 seconds. v9c therefore preregisters the
rounded global watchdog bound of **1,200 seconds**, leaving
197.0556091605293 seconds of completion/drain margin. Natural completion is
still allowed; manual stop and retry remain forbidden.

## Input identity and source-profile correction

The replay input was independently checked before v9c registration:

| Field | Value |
| --- | --- |
| bytes | `11290464091` |
| SHA-256 | `5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310` |
| expected records | `236687` |

The original v9 source profile contained one extra `7` in the input digest
(65 characters). Its immutable pre-correction SHA-256 was
`11ef3175f09712f9810e2c812560319a7d8a51ef369734d9fb042c7ce13c51b8` and it
remains recorded as prior-attempt lineage. The checked-in source profile now
contains the independently verified 64-character digest and the 1,200-second
watchdog; its new SHA-256 is
`f044586aa35fe5136e592aca5eb320bd7e84927916b9ce99b31a78c571b7ed0d`.

The v9c bound retains the v9 image identity and all existing v9 build,
identity, synthetic-handshake, feeder, runner, and safety gates. It uses the
native persisted key `m6a10_fast_livo2_v2c_v9` and the exact generated native
Docker command hash recorded in its unique preregistration.
