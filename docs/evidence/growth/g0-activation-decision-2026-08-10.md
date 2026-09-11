# G0 activation decision — 2026-08-10

## Decision

The largest current G0 activation blocker is **the absence of a current,
clean, zero-undocumented-step first-map baseline across the promised Docker
and source paths**.

The repository has discovery activity, a stable v0.9.0 release, and strong
machine-verifiable output contracts. It does not yet have a comparable trial
for any row of the Humble/Jazzy Docker/source matrix, or an accepted
independent-user first map. Therefore the next intervention is to execute and
repair the onboarding matrix—not add another algorithm route or increase
promotion.

This decision does not assert that 299 visitors attempted the workflow or that
all eight bundle downloads were prospective users. GitHub traffic is not a
funnel cohort. It shows only that discovery is non-zero while the project has
no accepted evidence at the first-map boundary.

## Baseline evidence

The aggregate values below come from the privacy-bounded
[2026-08-10 growth snapshot](2026-08-10.json). Trial coverage comes from an
inventory of tracked onboarding evidence against the
[v1 trial contract](../../onboarding-trials.md).

| Signal | 2026-08-10 baseline | Decision use |
| --- | ---: | --- |
| GitHub Stars | 837 / 1,000 | Lagging discovery milestone; 163 remain. |
| Unique repository views, 14 days | 299 | Discovery is present, but not attributable to trial attempts. |
| Unique clones, 14 days | 259 | Do not interpret as users or successful installs. |
| Autoware/TIER IV top-referrer unique sum | 6 | Qualified discovery is small but non-zero. |
| v0.9.0 primary-bundle downloads | 8 | Some release acquisition occurred; intent and success are unknown. |
| Accepted independent first maps | 0 / 3 | The activation and remaining v1 adoption gate is empty. |
| Comparable clean onboarding trials | 0 / 4 | No current Docker/source row can yet be used as a UX baseline. |
| Measured current matrix outcomes | 2 / 4 | Humble and Jazzy Docker are product `PASS`, measurement `INCOMPLETE`. |
| v1 readiness | 8 / 10 | Distribution and external adoption remain incomplete. |
| External PRs, trailing 90 days | 1 | The contributor loop is not yet self-sustaining. |
| External merged contributors, trailing 180 days | 0 | No recent external maintainer-independent proof loop. |
| Open / untriaged issues | 29 / 16 | Triage load can hide recurring onboarding findings. |

The historical
[Docker first-map trial](../docker-first-map-2026-07-28.md) proves that one
maintainer-operated Humble image eventually produced a verified map after two
defects were fixed. It predates the current receipt and trial contract, lacks
the complete active-time and peak-disk measurement set, and covers neither
Jazzy nor the source route. It remains valid historical evidence but cannot be
upgraded to a comparable G0 trial by filling missing values from memory.

## Why this blocker wins

- **Discovery is not selected first:** traffic and release acquisition are
  already non-zero. More promotion before measuring first success would make
  failures harder to diagnose and increase support load.
- **Algorithm expansion is not selected:** the v1 audit already marks first
  success, diagnosis, reproducibility, reliability, and CLI compatibility
  complete. There is no evidence that another SLAM route is the missing first
  action.
- **Distribution remains important but separate:** the rosdistro/package
  manager gate depends partly on external repository state. Digest-pinned
  Docker and source trials can expose and repair onboarding defects now.
- **Issue triage is a parallel G0 obligation:** the 16 untriaged issues may
  contain useful findings, but they do not substitute for controlled first-map
  trials.

## Execution gate

Complete the following in order:

1. Run all four fixed matrix rows from clean environments against an immutable
   image digest or Git commit. Record failures as failures; provide no private
   recovery instruction to the operator.
2. Rank findings by number of affected rows, earliest failed stage, and active
   operator cost. Fix the highest-ranked blocker and rerun the same row from a
   clean start.
3. Require at least one Docker and one source row to produce a comparable
   `PASS`, with zero undocumented steps, before changing the landing page or
   starting broad promotion.
4. Select the lower-burden passing path as the first independent-user route.
   Start the public three-user cohort and retain every failed report as a
   product finding.
5. By the tenth independent attempt, require at least 80% first-run completion
   and median active operator time below ten minutes. If either threshold is
   missed, keep promotion paused and repair the largest repeated finding.

Target date for the four-row matrix and first blocker rerun: **2026-09-06**,
the end of the roadmap's G0 Sprint 2 window. The next scorecard decision must
replace `0 / 4` with measured PASS/FAIL/INCOMPLETE rows; it must not infer
success from CI or historical evidence.

## First repair from the route audit

The first static route audit found that the source beginner path still used
the older NTU VIRAL dogfood wrapper and a separate verifier instead of the
Docker path's versioned `lidarslam-map` manifest, diagnosis, and first-map
receipt contract. Commit `74fe625ab2ee1dc9a0d55ce69bd705d22bac5d76`
replaced that split with one shared `run_first_map_demo.sh`: Docker and source
now use the same MID-360 input, maintained profile, headless runner, and
success artifacts. It also documents the equivalent Jazzy Docker route.

This is a product repair, not trial evidence. Comparable coverage remains
`0 / 4` until clean Humble/Jazzy Docker/source attempts are measured with the
onboarding contract. CI and stubbed regression tests cannot promote a row to
PASS.

## First measured execution update

The [2026-08-10 Docker machine probes](../onboarding/docker-machine-probes-2026-08-10.md)
replaced two unknown rows with measured outcomes:

| Row | Product outcome | Measurement status | Wall time | Workflow RX | Undocumented steps |
| --- | --- | --- | ---: | ---: | ---: |
| Docker Humble | `PASS` | `INCOMPLETE` | 1,440.865 s | 1,770,636,344 B | 0 |
| Docker Jazzy | `PASS` | `INCOMPLETE` | 1,140.525 s | 1,906,809,522 B | 0 |

Both rows reached the manifest, diagnosis, Autoware-verifier, and receipt
`PASS` gates with the exact v0.9.0 image digest and fixed 517,088,133-byte
input. They remain non-comparable because a container on the shared host did
not provide an isolated peak-disk scope, and the automation did not observe a
human active-time stopwatch or human-submitted commands. Those three values
are `null`, not estimates; the helper's internal Docker invocation does not
count as an operator command under the trial contract.

This removes an unknown Docker-product defect from the top of the blocker
list. The immediate four-row-matrix blocker is now publication of the reviewed
source candidate: GitHub cannot yet resolve commit
`74fe625ab2ee1dc9a0d55ce69bd705d22bac5d76`, so clean source trials would be
preflight failures. Publication requires a separate review/push decision; a
local-path clone would invalidate the evidence.

The largest observed Docker activation cost is different: cold first success
took 19–24 minutes and received 1.77–1.91 GB. The next product experiment
should evaluate a smaller onboarding fixture without weakening or replacing
the full MID-360 proof route. Broad promotion remains paused until at least one
Docker and one source row become comparable and the independent-user cohort
has started.

## Smaller-fixture experiment update

The [50-second MID-360 fixture pilot](../onboarding/mid360-onboarding-fixture-pilot-2026-08-10.md)
completed the local experiment. Two clean builds produced the same
98,873,952-byte ZIP and manifest, and the extracted candidate produced a
seven-of-seven `PASS` first-map receipt. This reduces the dataset transfer by
80.879%, but arithmetic decomposition of the Docker probes leaves more than
1.25 GB outside the current dataset download. Runtime-image slimming is
therefore the next independent transfer intervention.

The candidate has not been uploaded or made the default. It is not a
comparable onboarding row because no cold VM, network, human active-time, or
isolated peak-disk measurement was taken. The full 277-second gate remains
unchanged.

## Machine-readable matrix gate update — 2026-08-11

The fixed matrix now has a fail-closed aggregate checker in
`scripts/check_onboarding_trial_matrix.py`. It first applies the existing
single-row schema and semantic audit, then requires the exact Docker/source,
Humble/Jazzy, Ubuntu, architecture, revision-kind, product-version, source
commit, and full-dataset pairings. Missing rows stay `MISSING`; a valid FAIL is
not converted into coverage.

Running it against the two tracked machine probes reports:

- matrix status: `INCOMPLETE`;
- present outcomes: `2 / 4`;
- product PASS outcomes: `2 / 4`;
- comparable rows: `0 / 4`;
- missing rows: source Humble and source Jazzy; and
- activation gate: `FAIL`.

This is an audit of existing evidence, not a new trial. Both Docker rows still
lack active operator time, command count, and isolated peak disk. The source
candidate is still not publicly resolvable, and no branch, image, fixture, or
release was published to change that state. The next evidence-producing action
remains a reviewed publication decision followed by fresh dedicated-VM rows.
