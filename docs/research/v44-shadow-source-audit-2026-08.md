# v44 fixed-lag report-only shadow source gate

Date: 2026-08-10

Contract: `v44d-fixed-lag-shadow-source-audit-20260810`

Status: source implementation and static boundary audit complete

## Authority boundary

Stage v44d implements the estimator core authorized by v44c1 and audits that
source before a raw runtime exists. The core consumes only immutable,
already-decoded LiDAR/IMU records, one fixed `T_BL`, and externally supplied
protected-output/resource observations. It returns in-memory diagnostics.

The source has no command-line interface, bag decoder, ROS subscriber or
publisher, filesystem API, network/process API, wall-clock estimator input,
dataset identity, accuracy input, reference map, or primary-state callback.
There is therefore no path in this stage that can replay a bag or write a
trajectory or map.

The machine-readable boundary is
`configs/sota_v6/development/v44d_fixed_lag_shadow_source_audit.json`. It binds
the v44b architecture, v44c1 synthetic contract and aggregate, and
`scripts/v44_fixed_lag_shadow_estimator.py` by SHA-256. Any source change
invalidates this audit.

## Implemented estimator core

The source implements one global algorithm path:

- signed-int64 sensor-time ordering, IMU-before-LiDAR tie breaking, stable
  source indices, exactly-once consumption, bounded prefix/suffix handling,
  and first-IMU-after-scan-end triggering;
- exact point-time IMU integration and scan-end deskew, fixed `T_BL`, 1--50 m
  filtering, deterministic 0.5 m voxel representatives, source-frame
  surfels, and causal matching against at most eight older active knots;
- one 15-DoF `R,p,v,b_g,b_a` state per accepted scan end;
- midpoint SO(3) preintegration, 9x9 covariance, all five bias Jacobians,
  thresholded deterministic reintegration, and separate ordered bias-random-
  walk factors;
- dynamic bootstrap for both moving and stationary inputs through the same
  path. The first translation/time pair supplies the velocity seed; all
  bootstrap knots, both bias states, and a two-DoF fixed-magnitude S2 gravity
  direction enter one joint solve with the common broad priors. Completion is
  followed by deterministic gravity-to-negative-z, first-position, and
  first-yaw rebasing;
- binary point-to-plane factors connecting source and current poses, Huber
  whitening, deterministic SVD projection into each relative-pose observable
  subspace, and zero measurement information in rejected modes;
- chronological/type factor ordering, bounded block Householder QR,
  rank-revealing SVD of `R`, four fixed line-search scales, and no wall-clock
  stopping rule;
- three-second/64-knot eviction, square-root separator marginalization,
  immutable first-estimate Jacobian snapshots, source-surfel removal, and a
  diagnostic-only full-lag bias Schur-information spectrum;
- pre-allocation checks for message, state, surfel, correspondence, Jacobian,
  dense-solver, diagnostic-output, RSS, and RTF limits; and
- canonical diagnostic/state hashes, protected-output identity checks, and a
  terminal result with zero valid state count and no state hash on failure.

The implementation does not contain the retired direct velocity/bias
corrections, coordinate-specific weak-axis freeze, speed/history latch,
stationary/ZUPT branch, repeated observation, visual feedback, loop closure,
or persistent/global map path.

## Review correction before sealing

The first source draft passed its synthetic stream, but semantic review found
that bootstrap initially optimized knot and bias states while treating gravity
as already fixed. That was incomplete relative to v44b. Before final evidence,
gravity was promoted to an explicit two-DoF S2 variable in the same bootstrap
solve, the four frozen broad priors were wired in, and deterministic gravity
and yaw rebasing was added.

The same review replaced a linear within-scan pose proxy with integration at
every exact point timestamp and added the required diagnostic full-lag bias
Schur information. The first LiDAR translation divided by sensor time now
provides the velocity seed exactly as specified. No architecture threshold or
authority boundary was relaxed; all earlier v44d evidence was regenerated
after these corrections.

## Static and synthetic validation

The AST audit checks 21 classes, 29 module functions, 33 estimator methods,
21,393 AST nodes, and an exact eight-root import allowlist. All 36 checks pass,
including no top-level effects, CLI, raw/ROS adapter, filesystem/network/
subprocess call, wall-clock/parallel surface, dataset-specific symbol, or
forbidden public input field. It also checks the exact state layout, five bias
Jacobians, factor order, dynamic joint bootstrap, point-time deskew,
observability projection, bounded Householder solve, line search, FEJ
marginalization, resource preflight, protected outputs, diagnostic fields, and
terminal zero-output path.

| Synthetic smoke probe | Result |
| --- | ---: |
| SO(3) exp/log round trip | `0 rad` error |
| Bias Jacobian inventory | all five present and finite |
| S2 gravity | 2 local DoF; `0 m/s²` magnitude error |
| Preintegration covariance | minimum eigenvalue `9.99972e-6` |
| Binary LiDAR Jacobian | maximum error `4.68738e-11` |
| Streaming Householder solve | rank 3; error `9.45728e-16` |
| Square-root separator | marginal rank 3; error `8.94472e-16` |
| FEJ payload | immutable; stable SHA-256 |
| Protected outputs | mutation rejected |
| Resource preflight | oversized state rejected before system creation |

The full frozen-configuration synthetic stream uses 11 static wall scans and
100 Hz IMU. Two independent runs produce the same state SHA-256
`928df5580492ecd6471427bcdc2e6e474ce5921753c2f782826b29d46b6353de`
and diagnostic SHA-256
`1512d9971a1853ed3dac6acc53003a2d19abe379d48a0e9f7d2abf1723fd5257`.
It creates 11 states, obtains LiDAR rank 3 on the planar geometry, retains a
62-of-66 full-lag bias-information rank, and ends with exact negative-z
gravity, zero first-position norm, and zero first yaw. A separate shortened-
lag stream deterministically marginalizes the three oldest of seven states,
retains states `3,4,5,6`, removes their old surfels, and preserves an immutable
separator prior.

There are 28 estimator-core tests and 23 source-auditor tests. Negative tests
cover changed prerequisite/source hashes, relaxed replay authority, reordered
checks, forbidden imports and calls, top-level effects, dataset branches,
public-input expansion, changed state dimension/factor order/authority,
missing marginalization or diagnostics, duplicate reports, payload tampering,
resource overflow, malformed geometry, duplicate events, protected-output
mutation, and terminal zero-output behavior. The complete v40--v44d
regression passes with `212 passed in 18.39s`.

## Deterministic evidence

Two standalone validations have identical deterministic report payload
SHA-256
`9a7bf365e5a90e0f0a1b1296b358305277091059d0ea8bb3beece45fdea2b7d2`.
They began at 34.38--34.40 MiB RSS, peaked at 50.45--50.46 MiB, and added
16.06--16.07 MiB, below both audit ceilings.

Evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44d_fixed_lag_shadow_source_audit_20260810/`.
The aggregate JSON SHA-256 is
`b2494f42b07d9fd844f14112da5b6df6fd050007326018967f9f794b0cb5e997`,
and its deterministic aggregate payload SHA-256 is
`2344a9e9b31861fff44c1b457199a2863615a2c06834a6a9c39a710fc53b97b2`.

## Decision

The sealed decision is
`AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_CONTRACT_DEFINITION`.

This authorizes only definition of the next hash-bound execution contract and
its read-only decoder adapter. Raw shadow replay itself is still unauthorized.
The next contract must bind the exact v44a source manifest and all three bags,
write only into a new bounded evidence directory, enforce 330 MiB RSS and 0.85
RTF after every scan, verify protected v17 hashes before and after, and require
two identical state payloads. Accuracy, ground truth, reference maps, ROS
publication, and every primary trajectory/map/bias mutation remain forbidden.
