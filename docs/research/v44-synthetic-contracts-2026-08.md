# v44 fixed-lag synthetic numerical contracts

Date: 2026-08-10

Active contract: `v44c1-fixed-lag-synthetic-contracts-20260810`

Status: Stage 3 complete; report-only shadow implementation authorized

## Boundary

Stage 3 implements numerical reference oracles for the 20 contracts frozen in
the v44b architecture. It does not implement a streaming estimator and cannot
open a bag, ROS message, trajectory, map, accuracy metric, or reference input.
Its only inputs are hash-bound JSON contracts and deterministic synthetic
arrays.

The active correction overlay is
`configs/sota_v6/development/v44c1_fixed_lag_synthetic_contracts.json`. It
binds the original scenario contract
`configs/sota_v6/development/v44c_fixed_lag_synthetic_contracts.json`, the
v44b architecture, and both earlier aggregate files by SHA-256. Seeds,
tolerances, case ordering, motion, geometry, and all numerical thresholds are
unchanged by the correction.

## Resource preflight correction

The first standalone v44c validation passed all 20 cases at 42.34 MiB RSS, but
the complete pytest regression host already held 131–143 MiB before the v44c
harness began. The original absolute 128 MiB check therefore rejected four
embedded tests before any v44c allocation. That measured unrelated resident
test modules rather than synthetic-harness memory.

The preliminary contract and PASS aggregate remain immutable and are marked
superseded. v44c1 changes only measurement scope:

- A standalone process must remain below 128 MiB absolute RSS.
- A host already above that ceiling before v44c starts is charged only the
  harness increment, capped at 64 MiB.
- Allocation and write failures remain terminal and are checked before the
  operation.

Final standalone repetitions began at 37.10–37.14 MiB, peaked at
42.47–42.55 MiB, and added only 5.34–5.45 MiB. The full combined regression
also passes under the incremental rule.

## Numerical reference implementation

`scripts/validate_v44c_fixed_lag_synthetic_contracts.py` provides only the
following bounded reference primitives:

- SO(3) exponential/logarithm, vector alignment, and right perturbation;
- integer-nanosecond bracketing and linear IMU boundary interpolation;
- midpoint IMU preintegration, covariance propagation, numerical bias
  Jacobian oracle, and 15-dimensional residual evaluation;
- scan-end deskew and fixed `T_BL` point transformation;
- binary point-to-plane residual and analytic source/current Jacobians;
- deterministic SVD observable projector and truncated least-squares solve;
- dynamic constant-acceleration bootstrap oracle without orientation or ZUPT;
- square-root left-nullspace marginalization and immutable FEJ prior;
- bounded fixed-lag eviction, factor ordering, state serialization, event
  ordering, allocation rejection, and protected-output hashing.

These functions are synthetic references for a later implementation. They do
not hold a live estimator state, subscribe, publish, or process raw data.

## Results

| Contract | Frozen challenge | Key result |
| --- | --- | ---: |
| SO(3) direction | `+0.4 rad/s` about z for 1 s | error `9.94e-17 rad` |
| Constant motion | gravity-specific force, nonzero velocity | max residual `7.11e-15` |
| Gravity sign/rebase | tilted negative-z estimate | error `6.85e-17 m/s²` |
| Gyro-bias Jacobian | direct reintegration challenge | rotation error `1.11e-10 rad` |
| Accel-bias Jacobians | direct reintegration challenge | velocity error `3.34e-12 m/s` |
| Covariance | 0.5 s vs 1.0 s | PSD; trace ratio `2.46475` |
| Boundary interpolation | 5–45 ms inside 10 ms samples | 6 samples; error `1.39e-17` |
| Gap rejection | one 60 ms gap vs 50 ms limit | rejected; zero state output |
| Deskew direction | body advances 0.1 m | early point `5.0 -> 4.9 m` |
| Extrinsic direction | z+90° and `[1,0,0] m` | point becomes `[1,1,0] m` |
| LiDAR Jacobians | both poses, right perturbation | max error `4.69e-11` |
| SVD rotation invariance | random 6-DoF basis change | projector error `1.55e-15` |
| Weak-axis removal | singular values ending at `0.1` | rank 5; weak update exactly 0 |
| Dynamic startup | nonzero velocity/acceleration | velocity error `2.00e-15 m/s` |
| Stationary startup | same bootstrap function | no stationary branch or ZUPT |
| Square-root marginalization | 24x8 random system | solution error `6.11e-16` |
| FEJ/gauge | three-state relative chain | gauge error 0; immutable hash |
| Fixed lag | 20 Hz for 4 s, 3 s lag | 61 active; 20 oldest evicted |
| Deterministic payload | two independent shuffles | identical SHA-256 |
| Combined fail-closed | order, duplicate, memory, write | all 5 rejected; zero output |

The first numerical run exposed a real defect before sealing: the SO(3)
logarithm returned twice the correct rotation vector because its skew-vector
extraction and scale were both applied. The implementation was corrected;
the frozen tolerance was not relaxed.

The 46 v44c1 tests include each case independently, direct finite-difference
checks, malformed rotations/timestamps, nonfinite matrices, incorrect
prerequisite hashes, dataset branches, reordered cases, relaxed replay
authority, resource overflow, nonrepeatable reports, and incomplete reports.
The complete v40–v44c1 regression passes with 161 tests.

## Decision

Both final validation reports have deterministic payload SHA-256
`710564cc07242d98e9bddd0170362a20c2741e4318ddb5602f8964699800b86c`.
The combined case payload SHA-256 is
`7c6fd8e106885f90c1fc1627939212bfaf454c6692ae9be15aa1abb06ea593d2`.

The sealed decision is
`AUTHORIZE_V44_STAGE4_REPORT_ONLY_SHADOW_IMPLEMENTATION`.

This permits implementation of the isolated diagnostic-only fixed-lag shadow
estimator. Raw replay remains unauthorized until that source is complete,
statically audited for the v44b boundaries, and given a separate execution
contract. Accuracy/reference-map inputs and all primary trajectory/map
mutation remain forbidden.
