# Open-issue triage proposal — 2026-08-11

> Status: **PROPOSED_NOT_APPLIED**
>
> Coverage: **29 / 29 open issues**
>
> Offline validation / read-only live drift check: **PASS / PASS**
>
> GitHub write authorization: **not granted**
>
> Remote mutations performed: **none**

This is the complete G0 disposition proposal for the public
`rsasaki0109/lidar_slam_ros2` issue backlog. The authoritative structured
record is
[`open-issue-triage-proposal-2026-08-11.json`](open-issue-triage-proposal-2026-08-11.json),
validated against
[`issue-triage-proposal-v1.schema.json`](../../schemas/issue-triage-proposal-v1.schema.json).

The audit read every current issue body and all current comments through
GitHub's read-only REST API. It stores no author identity, issue body, or
comment body. Issue numbers, titles, timestamps, labels, and proposed public
actions are retained because they are required to detect drift before any
future write.

No proposal below is a completed GitHub action. Every target must be re-read,
the live drift check must pass, prerequisites must be public, and explicit
write authorization must exist before a label, comment, state, or state reason
is changed.

## Result

| Decision | Count | Issues |
| --- | ---: | --- |
| Keep open | 2 | #69, #422 |
| Request a current supported-version reproduction | 4 | #64, #104, #106, #124 |
| Close as answered | 12 | #83, #92, #94, #96, #98, #100, #101, #108, #111, #112, #115, #122 |
| Close as reporter-confirmed or implementation-resolved | 7 | #30, #89, #93, #95, #103, #110, #118 |
| Close as superseded by a current path | 3 | #53, #102, #105 |
| Close as not planned for the v1 product boundary | 1 | #116 |

Priority is about the next maintainer action, not the age or importance of the
original user's work:

- P1: #69 crash safety and #422 external first-map adoption;
- P2: #64, #104, #106, and #124 need bounded current reproductions;
- P3: 23 answered, superseded, support-boundary, or broad-scope items.

## Complete proposal

| Issue | Theme / priority | Proposed labels | Decision | Apply gate |
| --- | --- | --- | --- | --- |
| [#30](https://github.com/rsasaki0109/lidar_slam_ros2/issues/30) | quality / P3 | `documentation`, `enhancement` | close resolved: odom weighting and GNSS constraints now exist | current docs |
| [#53](https://github.com/rsasaki0109/lidar_slam_ros2/issues/53) | quality / P3 | `documentation`, `question` | close superseded: replace 2022 open-ended run with soak/diagnosis contract | current docs |
| [#64](https://github.com/rsasaki0109/lidar_slam_ros2/issues/64) | TF / P2 | `bug`, `question` | request Humble/Jazzy timestamp-extrapolation reproduction | none |
| [#69](https://github.com/rsasaki0109/lidar_slam_ros2/issues/69) | reliability / P1 | `bug`, `help wanted` | keep open: VoxelGrid overflow can still terminate scanmatcher | bounded fixture and fix |
| [#83](https://github.com/rsasaki0109/lidar_slam_ros2/issues/83) | platform / P3 | `question` | close answered: native Windows is outside support; Docker is maintained | current docs |
| [#89](https://github.com/rsasaki0109/lidar_slam_ros2/issues/89) | sensor / P3 | `documentation`, `question` | close resolved: reporter confirmed horizontal placement fixed XY failure | current docs |
| [#92](https://github.com/rsasaki0109/lidar_slam_ros2/issues/92) | quality / P3 | `documentation`, `question` | close answered: historical tuning and IMU-model discussion | current docs |
| [#93](https://github.com/rsasaki0109/lidar_slam_ros2/issues/93) | TF / P3 | `documentation`, `question` | close resolved: reporter stated the issue no longer existed | current docs |
| [#94](https://github.com/rsasaki0109/lidar_slam_ros2/issues/94) | quality / P3 | `enhancement`, `question` | close answered: historical long-loop limitation was explained | current docs |
| [#95](https://github.com/rsasaki0109/lidar_slam_ros2/issues/95) | sensor / P3 | `documentation`, `question` | close resolved: Ouster adaptation worked and reporter confirmed understanding | current docs |
| [#96](https://github.com/rsasaki0109/lidar_slam_ros2/issues/96) | sensor / P3 | `documentation`, `question` | close answered: scan period and extrinsic requirements were answered | current docs |
| [#98](https://github.com/rsasaki0109/lidar_slam_ros2/issues/98) | platform / P3 | `question` | close answered: unvalidated Unitree L1 rig needs the public sensor route | starter C4 |
| [#100](https://github.com/rsasaki0109/lidar_slam_ros2/issues/100) | platform / P3 | `question` | close answered: unsupported bipedal vibration scenario | current docs |
| [#101](https://github.com/rsasaki0109/lidar_slam_ros2/issues/101) | advanced / P3 | `documentation`, `question` | close answered: lower-rate modified path is the submap-pose design | current docs |
| [#102](https://github.com/rsasaki0109/lidar_slam_ros2/issues/102) | TF / P3 | `documentation`, `question` | close superseded by empty-frame preflight and no-map card | starters C2 + C5 |
| [#103](https://github.com/rsasaki0109/lidar_slam_ros2/issues/103) | TF / P3 | `documentation`, `question` | close resolved: reporter fixed a leading-slash frame mismatch | current docs |
| [#104](https://github.com/rsasaki0109/lidar_slam_ros2/issues/104) | reliability / P2 | `bug` | request three current repeated runs under the determinism contract | none |
| [#105](https://github.com/rsasaki0109/lidar_slam_ros2/issues/105) | sensor / P3 | `question` | close superseded by measured MID-360 path and support boundary | starter C4 |
| [#106](https://github.com/rsasaki0109/lidar_slam_ros2/issues/106) | TF / P2 | `bug`, `question` | request current no-map diagnosis and exact input/TF evidence | starter C2 |
| [#108](https://github.com/rsasaki0109/lidar_slam_ros2/issues/108) | install / P3 | `documentation`, `question` | close answered: Foxy is EOL and source-clone advice is obsolete | starter C1 |
| [#110](https://github.com/rsasaki0109/lidar_slam_ros2/issues/110) | install / P3 | `documentation`, `question` | close resolved: build passed; launch used the wrong package name | current Docker docs |
| [#111](https://github.com/rsasaki0109/lidar_slam_ros2/issues/111) | sensor / P3 | `documentation`, `question` | close answered with generic PointCloud2 adaptation contract | starter C4 |
| [#112](https://github.com/rsasaki0109/lidar_slam_ros2/issues/112) | TF / P3 | `documentation`, `question` | close answered: Odometry messages do not guarantee TF publication | starter C3 |
| [#115](https://github.com/rsasaki0109/lidar_slam_ros2/issues/115) | sensor / P3 | `documentation`, `question` | close answered without claiming validated RS-16 support | starter C4 |
| [#116](https://github.com/rsasaki0109/lidar_slam_ros2/issues/116) | advanced / P3 | `enhancement` | close not planned: continuous corrected-map feedback is outside v1 | product contract |
| [#118](https://github.com/rsasaki0109/lidar_slam_ros2/issues/118) | advanced / P3 | `question` | close resolved: localization-only package met the stated need | current scope |
| [#122](https://github.com/rsasaki0109/lidar_slam_ros2/issues/122) | install / P3 | `documentation`, `question` | close answered: unpinned g2o on Foxy is outside support | starter C1 |
| [#124](https://github.com/rsasaki0109/lidar_slam_ros2/issues/124) | quality / P2 | `bug`, `question` | request current HDL-32E pose-drift reproduction and diagnostics | none |
| [#422](https://github.com/rsasaki0109/lidar_slam_ros2/issues/422) | adoption / P1 | unchanged | keep open until three accepted independent first maps | public receipts |

## Why six issues remain open

### #69 — VoxelGrid overflow crash

This is the highest technical-reliability item in the old backlog. As of
2026-08-17, runtime fix `a2368c4`, component proof `bce5a9d`, and rejected-map
update recovery `99cce93` are ancestors of public Draft PR #427 head
`4b2ab514`. Its
[bounded evidence](../voxel-grid-overflow-safety-2026-08-11.md) routes all five
classic scanmatcher `VoxelGrid` stages through a fail-closed signed-32-bit
index/layout preflight. The public Humble and Jazzy workflows pass, including
the component recovery target. Valid clouds retain direct-PCL output parity;
an unsafe timestamp is rejected without replacing valid state, and a later
safe timestamp can proceed.

The issue remains open because no named release contains the fix. The prepared
response explains `vg_size_for_map` versus `vg_size_for_input`, asks users to
check units and outliers first, and labels a larger leaf as a
resolution-changing workaround rather than the fix. Parameter advice alone is
not a sufficient crash contract, and the unavailable historical private bag is
not described as reproduced.

### #422 — independent first-map validation

This is the active v1 adoption gate. It remains open at 0/3 and must not receive
private step-by-step help that would invalidate independence. Only reviewed,
privacy-bounded public receipts can advance it.

### #64, #104, #106, and #124 — current reproduction requests

These symptoms remain plausible on the supported product surface, but their
existing evidence predates the current diagnostics or is incomplete. Each gets
one bounded request with exact revision, supported ROS distribution, command,
input identity, relevant diagnosis, and a 30-day recheck. No response means the
historical report can be closed without claiming the underlying failure is
impossible; a new complete reproduction can always reopen the product decision.

## Application protocol

1. Run the offline checker:

   ```bash
   python3 scripts/check_issue_triage_proposal.py --json
   ```

2. Immediately before any remote action, run the read-only drift check:

   ```bash
   python3 scripts/check_issue_triage_proposal.py --live --json
   ```

3. Generate a bounded review packet for one issue. This repeats the live
   GET-only issue check, re-hashes every repository evidence file, and prints
   only to standard output. For #69 it also reuses the GET-only product-Draft
   audit to require PR #427 at exact head `4b2ab514`, still open, Draft, and
   mergeable, with 10 successful checks, 4 intentional skips, and no pending
   or failing check. A second source-bound audit requires latest stable
   `v0.9.0` at exact commit `0df0c4a`, 52 commits behind reviewed fix
   `a2368c4`, while both the planned `v0.9.1` tag and release remain absent:

   ```bash
   python3 scripts/prepare_issue_triage_application.py --live --issue 69
   python3 scripts/prepare_issue_triage_application.py \
     --live --issue 69 --json
   ```

   A live #69 packet requires `live_check: PASS` plus both source claims inside
   `linked_check: PASS`; an offline #69 packet says `linked_check: NOT_RUN`
   instead of presenting dated Draft/CI/release facts as current. The packet
   contains a prepared response draft but no mutation command. It reports
   `PREPARED_NOT_AUTHORIZED`; it cannot post, label, or close anything. Rows
   with starter dependencies remain `DEPENDENCY_REVIEW_REQUIRED`, and #422
   remains `MONITOR_ONLY` so independent validation is not contaminated.
4. Stop if an issue was edited, relabeled, closed, reopened, added, or removed.
   Refresh the proposal from the new state; never apply by issue number alone.
5. Complete starter C1–C5 before applying any row that names one as a gate.
6. Obtain explicit authorization for the exact issue write scope. Packet
   generation and live GET access are not that authorization.
7. Re-read each target and post its issue-specific explanation before changing
   state. For #69, stop unless both source-bound Draft/CI and stable-release
   claims are `PASS`; head, state, Draft flag, mergeability, check totals,
   latest stable identity/ancestry, candidate tag/release presence, or
   GET-only authority drift invalidates the packet. Other dated claims not
   represented by a linked check still require a separate maintainer recheck.
   Do not use a context-free bulk-close message.
8. Apply P1 and reproduction-request rows first, then review support load before
   any closure batch.
9. Capture a fresh aggregate growth snapshot after application. Do not store
   author identities or comment bodies in the repository.

The checker and application-packet generator have no write mode. Authorization
does not change that property; a future write operation must use a separate,
explicitly approved workflow.
