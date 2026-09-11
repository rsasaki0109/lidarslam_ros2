# Canonical `ndt_omp` upstream PR packet — 2026-08-12

> Status: **LOCAL_COMMIT_READY / NOT_PUBLISHED**
>
> GitHub write authorization: **not granted**
>
> Remote mutations performed: **none**

This packet turns the v1 distribution gate's `ndt_omp` lineage blocker into
one reviewable upstream change. It does not authorize a branch push, pull
request, rosdistro reply, tag, Bloom run, or package publication.

## Decision

Submit the four lidarslam-required APIs to canonical
[`koide3/ndt_omp`](https://github.com/koide3/ndt_omp), then switch both direct
lidarslam consumers to that package. Do not merge the current
`ndt_omp_ros2` Bloom registrations: despite a distinct ROS package name, they
overlap Humble's existing `ndt_omp` headers, C++ namespace, library filename,
and SONAME.

The fully isolated-fork option remains a fallback only if upstream declines
the APIs. It would require distinct package, namespace, include, library,
SONAME, CMake target, executable, version, tag, Bloom-track, and rosdistro
identities.

## Exact source identity

| Field | Value |
| --- | --- |
| Upstream repository | `koide3/ndt_omp` |
| Upstream branch | `master` |
| Exact upstream base | `5495fd9214945afcb4b35d5a1da385e405c52bf9` |
| Local candidate commit | `618f02f6b50a8590b81f48b4fee5b6cfc8d3f3ea` |
| Local commit subject | `Add optional priors and correspondence diagnostics` |
| Patch SHA-256 | `7b641c32ec4f30faa302e60aaa89765bb9acf67f3f0feb85f9e4e11e88b4dc9f` |
| Patch scope | 5 files, +342 / -1 |
| Proposed fork | `rsasaki0109/ndt_omp_ros2` (GitHub fork of `koide3/ndt_omp`) |
| Proposed branch | `lidarslam-priors-and-correspondence-diagnostics` |

The 2026-08-15 read-only duplicate refresh again found the fork branch absent
and upstream `master` at the exact base above. The four open upstream PRs do
not overlap this API work, and searches containing `prior`, `correspondence
distance`, or `regularization` found no matching PR;
[`koide3/ndt_omp#45`](https://github.com/koide3/ndt_omp/issues/45) is about
score interpretation and is not an implementation duplicate.

The local commit contains exactly the checked-in patch artifact:

```text
CMakeLists.txt
include/pclomp/ndt_omp.h
include/pclomp/ndt_omp_impl.hpp
package.xml
test/test_ndt_regularization.cpp
```

`git diff --cached --binary | sha256sum` before the local commit produced the
same SHA-256 as
`packaging/ndt_omp/koide3-ndt-omp-lidarslam-priors.patch`.

## Proposed pull request

Title:

> Add optional NDT priors and correspondence diagnostics

Body:

> ## Summary
>
> - add opt-in per-axis rotation and translation priors to NDT;
> - add an optional maximum voxel-mean correspondence distance;
> - report the accepted correspondence count and mean distance after
>   alignment; and
> - add four focused tests for priors, correspondence filtering/statistics,
>   and invalid optional configuration.
>
> ## Motivation
>
> A downstream ROS 2 consumer currently carries these APIs in a fork for IMU
> and translation priors plus adaptive correspondence filtering. That fork
> installs the same `pclomp` headers and `libndt_omp.so` as this package, so
> releasing both packages is not a safe long-term solution. Upstreaming the
> small algorithm/API delta lets downstream users return to the canonical
> package instead of publishing an overlapping implementation.
>
> ## Compatibility
>
> Existing behavior is unchanged by default. Priors and distance filtering
> are disabled until explicitly configured. Non-finite or otherwise invalid
> optional values fail back to the disabled state. Correspondence statistics
> are observational and use the existing per-input-point reduction order.
>
> ## Validation
>
> - clean ROS 2 Jazzy / PCL 1.14 build and install;
> - 4/4 focused GTests pass;
> - the same patch previously built and passed the four tests on clean Humble
>   and Jazzy environments; and
> - both downstream consumers were separately built against the canonical
>   package with the prepared dependency-transition patch.
>
> I am happy to split the API groups or adjust naming if a smaller upstream
> surface is preferred.

The PR should be opened as a draft first. The description deliberately names
the downstream fork and file collision; it must not imply that a separate
`ndt_omp_ros2` package is an independent algorithm.

The exact title, body, and Draft requirement are also bound in
`docs/contracts/canonical-ndt-convergence-v1.json`. The prose above remains
the human review copy; the contract is the machine source used by the strict
preflight handoff, so a copy change cannot silently diverge from the checked
candidate.

## Current validation

The exact local candidate commit was checked from a clean detached worktree on
ROS 2 Jazzy with GCC 13 and PCL 1.14:

- clean `colcon build`: PASS;
- install/export: PASS;
- `test_ndt_regularization`: 4/4 test cases PASS;
- generated build, install, and log data used `/dev/shm` and was removed;
- source worktree after validation: clean at exact commit
  `618f02f6b50a8590b81f48b4fee5b6cfc8d3f3ea`.

The repository convergence checker first reported
`READY_FOR_UPSTREAM_REVIEW`, 20/20 checks, against a clean detached checkout
of exact upstream base `5495fd9`. Its stricter read-only publication mode now
also verifies the exact candidate commit/parent/subject/diff hash, current
upstream head, fork parent, proposed-branch absence, and semantic duplicate
PR search:

```bash
GITHUB_TOKEN="$(gh auth token)" \
python3 scripts/check_canonical_ndt_convergence.py \
  --upstream-checkout /path/to/clean/koide3-ndt_omp-at-5495fd9 \
  --candidate-checkout /path/to/clean/ndt_omp-at-618f02f \
  --online \
  --require-ready-for-draft-pr
```

The 2026-08-15 live strict run reported `READY_FOR_DRAFT_PR`, 30/30 checks:
upstream `master` remained exact `5495fd9`, the proposed fork remained the
expected child of `koide3/ndt_omp`, the proposed branch was absent, and none
of four open upstream PRs matched the candidate branch or API terms. The JSON
report omits both local checkout paths and keeps GitHub write authority false.
Earlier Humble/Jazzy build and downstream consumer evidence remains documented
in `docs/evidence/ndt-omp-release-review-2026-08-12.md`.

The same strict online gate was repeated on 2026-08-16 from fresh clean
temporary worktrees and remained `READY_FOR_DRAFT_PR`, 30/30. Upstream
`master`, fork identity, candidate commit, proposed-branch absence, four-PR
duplicate search, and no-write authority all remained unchanged.

The 2026-08-17 strict rerun again passed 30/30 from fresh detached worktrees.
The schema-v1 JSON now emits `draft_pr_handoff` only in this exact green state.
It binds `koide3/ndt_omp:master`, the expected fork and absent create-only
branch, exact base and candidate SHAs, exact title/body, four ordered steps,
and Draft/non-force/create-only constraints. Push, PR creation, force-push,
mark-ready, and merge authority all remain false, and `writes_performed`
remains false. Every non-ready or blocked report must carry
`draft_pr_handoff: null`.

The two existing rosdistro PRs are not green publication candidates. Their
exact heads each have 5/6 passing check runs and one failed
`rosdistro / rosdep checks (3.8)` run. The failure is the old OpenEmbedded
`libpcre@openembedded-core` mapping fixed by
[ros/rosdistro#52858](https://github.com/ros/rosdistro/pull/52858), not the
NDT registration diff, but the old heads remain red. They must neither merge
as-is nor be described as green; collision-free convergence comes first, then
a current-base generated PR and a complete passing suite.

## Publication sequence

Each external step needs its own explicit scope and a fresh drift check.

1. Run the strict command above immediately before publication and require
   `READY_FOR_DRAFT_PR`, 30/30 checks, zero remote errors, the exact candidate
   commit, an absent proposed branch, zero duplicate PRs, and a non-null
   `CREATE_CANONICAL_NDT_DRAFT_PR` handoff.
2. After a separate exact-candidate, create-only decision, follow that
   handoff: non-force push exact commit `618f02f6...` to the proposed fork
   branch and open a Draft PR to `koide3/ndt_omp:master` with its exact
   title/body. Abort on any identity drift; do not force-push, mark ready, or
   merge.
3. Replace `<UPSTREAM_PR_URL>` in the two prepared rosdistro replies with the
   verified Draft URL, then reply to #52950 with the full lineage/collision
   answer and to #52949 with the matching concise answer. Do not close either
   PR until the reviewer confirms whether to hold or supersede it.
4. Address upstream review with focused commits; do not silently expand the
   public API or change default alignment behavior.
5. After upstream acceptance, release/update canonical `ndt_omp`, apply the
   five-file lidarslam consumer transition, and run Humble/Jazzy package and
   installed-consumer gates. Any replacement rosdistro PR must be generated
   from current `master` and pass every check; do not carry forward the old
   stale-base red state.
6. Close or supersede the colliding `ndt_omp_ros2` Bloom registrations, wait
   for main-channel sync, then run the exact package-manager E2E required by
   the v1 distribution gate.

## Authority boundary

This packet is evidence and copy-ready text only. It authorizes no GitHub
write. In particular, the local candidate commit has not been pushed, no
upstream PR exists, the prepared rosdistro response has not been posted, and
the two existing Bloom PRs remain open and unchanged. A
`READY_FOR_DRAFT_PR` report proves technical identity and current read-only
remote state; it does not change that authority boundary.
