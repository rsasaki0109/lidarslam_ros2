# GLIM usability scorecard contract — 2026-08-12

> Decision: **LOCAL_SCORECARD_CONTRACT_PASS / PAIRED_PUBLIC_TRIALS_MISSING**
>
> Candidate base: `c2be534bb2dbbc784d93238295614b9d08147472`
>
> Remote mutations performed: **none**

## Why this increment

The product candidate has an interactive home, a read-only doctor, fixed demo
routes, own-bag preflight, verified map artifacts, stable recovery actions, and
a Japanese beginner page. Those features can reduce operator work, but their
existence alone does not show that the public workflow is as convenient as
GLIM for a first-time user.

This increment converts that comparison into a fixed, neutral evidence
contract. It does not import private maintainer experience, local-only product
identities, or feature-count claims as usability results.

## Fixed paired protocol

The scorecard measures six overlapping jobs independently:

1. discover the supported path;
2. run one fixed public demo;
3. inspect the operator's own bag;
4. produce and verify a downstream artifact;
5. understand and recover from one public failure; and
6. repeat the documented contract after a supported upgrade.

The two records must bind exact public product versions and documentation,
clean matching hosts supported by each product's selected docs, one pair and
anonymous cohort, the same input for each task, exact command sequences,
task-specific metrics, transcript hashes, and no undocumented steps. One
product must be attempted first and the other second.
`READY` additionally requires an external first-time operator and all six tasks
to be comparable. A maintainer pair can reach only `PARTIAL`.

The schemas, empty evidence index, human protocol, and fail-closed checker are
versioned together. The checker rejects missing or reordered tasks, incomplete
task measurements, environment or input drift, non-public product identities,
dirty hosts, multiline commands, undocumented assistance, and authority to
make an automatic winner claim.

## Verification

| Check | Result |
| --- | --- |
| checked-in no-argument index | `NOT_READY`; 0 records; 0/6 comparable tasks |
| scorecard checker regressions | `13 passed` |
| documentation entrypoint and release-bundle integration | PASS |
| complete maintained Python gate | graph: `1,430 passed / 13 skipped / 11 existing warnings`; lidar_slam: `707 passed`; `2,137 total` |
| Python style for the checker and regression module | PASS |
| schema/index JSON parsing | PASS |
| remote writes | none |

The complete maintained Python gate and exact candidate inventory are recorded
in the publication-slice evidence after this increment is frozen.

## Paired observation recorder follow-up — 2026-08-16

Implementation tip
`0575fb6d67dc0b2069d9e41029a767bf3608687c` closes the unsafe hand-editing
gap between worksheet preparation and scorecard validation. One command now
accepts exactly one untouched worksheet per product, follows the declared
first/second order, prompts only for direct observations, derives command
counts and outcomes, and publishes the two validated records with one atomic
directory rename. A non-interactive collector can supply the same fixed task
contract as JSON.

Blank values remain `not-recorded`; the checker now treats that marker as an
explicit comparability blocker even when every numeric field happens to be
present. Pair/environment drift, reordered tasks, malformed measurements,
private command paths, reused worksheets, and existing destinations reject the
whole output. `--require-ready` distinguishes a safely retained incomplete pair
(exit `1`) from a structural or privacy error with no published pair (exit
`2`). Neither path performs a network or remote mutation or infers a winner.

Verification at the implementation tip:

| Check | Result |
| --- | --- |
| recorder regressions | `7 passed`; complete, incomplete, drift, privacy, overwrite, non-TTY, and release-bundle boundaries |
| focused scorecard/publication/G0/docs regressions | `73 passed` |
| registered Jazzy CTest after reconfigure | `6 / 6 passed` |
| complete maintained Python gate | graph: `1,442 passed / 13 skipped / 11 existing warnings`; lidar_slam: `982 passed`; `2,424 total` |
| strict documentation and Python style | PASS |
| deterministic v0.9.1 candidate bundle | two identical 252-file bundles; SHA-256 `47d11e32c15a086707169ca03b83877324a642c853e6f3c3dd3ab581377d8ea2` |

This makes the external measurement executable; it is not the measurement.
The reviewed evidence index still contains zero product records and remains
`NOT_READY`.

## Public pair identity preflight follow-up — 2026-08-17

The official paired preparation path no longer accepts caller-asserted
publicity flags. One `--verify-public` option now performs bounded GET-only
checks for both products before local publication. Git identities are fixed to
`rsasaki0109/lidar_slam_ros2` and `koide3/glim`; a full commit SHA must resolve
unchanged and a release tag must match the declared version before being
dereferenced to a full commit SHA. Image identities are fixed to the canonical
GHCR and Docker Hub repositories and require an exact registry digest header.
Documentation requests and redirects are restricted to each product's fixed
GitHub/Pages host set and HTTP 200.

The emitted preparation manifest has its own Draft 7 schema and distinguishes
`NOT_RUN` offline preparation from a two-product `PASS`. It records GitHub,
registry, and documentation request modes separately and fixes every write or
remote-mutation authority to false. Both worksheets are staged before
exclusive publication; a stale identity, malformed response, off-boundary
redirect, existing destination, or second-file race leaves no partial pair.

Focused verification covers offline preparation, exact commit and annotated
tag resolution, canonical image digest resolution, manual-claim rejection,
identity and redirect drift, schema validation, overwrite refusal, and
second-file rollback. The implementation uses no subprocess and exposes only
an HTTP GET request method. These checks prove that the future observation is
bound to readable public inputs; they do not create the external observation,
mark one task comparable, or authorize a parity/winner claim.

The follow-up chain now persists that manifest as a fixed-name preparation
receipt and adds an exact SHA-256 to each worksheet row. Preparation publishes
both worksheets and the receipt as one rollback set. The recorder accepts the
two selected files only when they share that valid receipt directory, their
exact bytes still match, and their public identity metadata is unchanged. Its
atomic recorded session retains the original triplet under `preparation/`.
The final scorecard CLI requires that archived receipt for explicit records
and revalidates the untouched source bytes and stable identity transition;
the checked-in evidence index requires a receipt path whenever records are
present. A pair of completed JSON files alone can no longer report CLI
`READY`.

Verification for this follow-up:

| Check | Result |
| --- | --- |
| paired preparer regressions | `16 passed` |
| paired recorder regressions | `10 passed` |
| scorecard checker regressions | `17 passed` |
| preparer/recorder/checker/publication/G0 regressions | `78 passed` |
| real receipt-chain smoke | `preparation_binding: VALID`, public identities `PASS`, scorecard `NOT_READY` with 0/6 comparable tasks and no fabricated observation |
| complete maintained Python gate | graph: `1,489 passed / 13 skipped / 11 existing warnings`; lidar_slam: `1,094 passed`; `2,583 total` |
| real public GET preflight | PASS: public Draft commit `4b2ab514a4f33b443e2c4283b3114d11a5e44e49`; GLIM `v1.2.2` resolved to `faa264a1bce1bda406f73457e35511f56cdc2eaa`; both documentation URLs returned 200 |
| strict documentation, Python style, JSON, and Draft 7 schemas | PASS |
| external observation or remote mutation | none |

## Honest boundary and next measurement

No paired GLIM and `lidarslam_ros2` trial records exist yet. The exact local
candidate is not public, so it cannot be used as a public product identity.
This evidence therefore proves only that a neutral comparison can be recorded
and validated; it does not prove parity, superiority, or a lower completion
time.

After the exact candidate is published, run both products from their public
landing pages on matching clean machines with one external first-time
operator. A stronger public claim requires a second external operator with the
product order reversed. Publish task-level values, commands, versions, and
limitations without collapsing them into an overall winner.
