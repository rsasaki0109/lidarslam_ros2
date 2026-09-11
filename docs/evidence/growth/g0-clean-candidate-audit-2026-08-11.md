# G0 clean candidate audit — 2026-08-11

> Decision: **REVIEWABLE_LOCAL_CANDIDATE / G0_HOLD**
>
> Audited product revision:
> `6a8727a9014aea1ecfe8ea9c65d6f10cffb87cd3`
>
> Exact local and live public `develop` base:
> `86fa9b610c07ccf4d2b0f10939e17c129d34b40a`
>
> Remote mutations performed: **none**

## Outcome

The refreshed G0 product line is a clean, linear, locally reviewable
candidate. It contains 42 commits and 116 product/evidence paths relative to
the unchanged public `develop` base. It contains no changed research output,
generated map or dataset, archive, model, or binary path. Current read-only
GitHub inspection finds neither the candidate commit nor a remote branch with
the proposed local branch name.

The candidate may be presented for a branch plus Draft pull request after
explicit E1 source-publication authorization. It is not release-ready and G0
cannot advance: supported public CI has not run this revision, the onboarding
matrix remains incomplete, the fixture host and upload remain undecided,
GitHub issue mutations are unauthorized, v1 readiness is `8 / 10`, and the
existing `v0.9.0` tag correctly prevents version reuse.

## Evidence-carrier boundary

This record binds the product audit to fixed revision `6a8727a`. A commit
cannot contain its own Git hash, so the reviewed update to this audit and its
decision packet is carried by one later documentation-only commit. That
carrier does not silently become the product revision.

Immediately before any E1 operation:

1. resolve the exact carrier tip and compare it with `6a8727a`;
2. require the carrier delta to contain only the two G0 audit documents;
3. rerun worktree, ancestry, merge, object, whitespace, remote-base,
   remote-branch, and public-commit checks on that exact tip; and
4. obtain explicit approval naming that final carrier hash.

If any other path or commit appears, this audit is stale and E1 must stop.

## Lineage and worktree

| Check | Observation | Result |
| --- | --- | --- |
| Local base | `origin/develop` = `86fa9b610c07ccf4d2b0f10939e17c129d34b40a` | PASS |
| Live remote base | read-only `git ls-remote origin refs/heads/develop` returned the same hash | PASS |
| Product candidate | `6a8727a9014aea1ecfe8ea9c65d6f10cffb87cd3` | recorded |
| History | 42 commits ahead; 0 merge commits | PASS |
| Worktree | no modified, staged, or untracked path at audit time | PASS |
| Git object graph | `git fsck --no-dangling --no-progress` | PASS |
| Diff whitespace | `git diff --check origin/develop...6a8727a` | PASS |
| Base ancestry | `origin/develop` is an ancestor of the candidate | PASS |
| Remote branch/upstream | no upstream and no `agent/product-g0-guided-ux` remote branch | LOCAL_ONLY |
| Public commit lookup | GitHub returned `No commit found for SHA` (HTTP 422) | UNRESOLVABLE |

The fixture generator `0f91452c`, fixture review `eae8547`, VoxelGrid fix
`a2368c4`, component recovery proof `bce5a9d`, and contributor test
entrypoint `96d0763` are ancestors of the candidate. Publishing a reviewed
carrier would make those source revisions inspectable; it would not publish
the fixture ZIP or authorize an issue, release, package, or image mutation.

## Diff inventory

The product candidate changes 116 paths: 66 added and 50 modified, with
22,539 lines added and 298 removed.

| Top-level area | Changed paths |
| --- | ---: |
| `docs/` | 48 |
| `scripts/` | 23 |
| `lidarslam/` | 16 |
| `graph_based_slam/` | 9 |
| `scanmatcher/` | 7 |
| `docker/` | 3 |
| `.github/` | 3 |
| root and configuration surface | 7 |

The path and object audit found:

- no changed `build/`, `install/`, `log/`, `output/`, dataset, bag, map,
  PCD, PLY, LAS/LAZ, archive, model, private, secret, raw, or generated path;
- no changed `docs/research/`, 3DGS asset, or `lidarslam/images/` path;
- no binary diff and no changed symlink;
- no deleted path; and
- largest added or modified blob: `scanmatcher_component.cpp`, 75,559 bytes.

The repository still contains older multi-megabyte research and visual assets
in the public base history. They were not added or changed by this candidate.
Repository-history reduction remains a separate decision; rewriting public
history is not part of G0.

## Candidate contents

The 42 commits form five review themes:

1. guided own-bag UX, shared Docker/source first-map paths, and comparable
   onboarding-trial contracts;
2. slim installed runtime images, resumable and safe public-data intake, and
   an immutable-host fixture audit;
3. privacy-bounded growth/community operations, complete issue disposition,
   and the 2026–2029 operating plan;
4. fail-closed classic scanmatcher VoxelGrid safety plus real-component
   unsafe-then-safe recovery for issue #69; and
5. one dependency-aware contributor Python test entrypoint, with Humble/Jazzy
   full-suite proof and a cross-version rosbag fixture repair.

No SOTA-v6 implementation, consumed research dataset, generated benchmark
output, or local research evidence is required to build or operate the product
candidate.

## Verification ledger

| Gate | Exact current evidence | Candidate interpretation |
| --- | --- | --- |
| Exact-tip Jazzy Python product suites | graph: 1,406 passed, 13 skipped, 11 pre-existing ImageIO warnings; lidarslam: 484 passed | PASS |
| Humble Python product suites | code revision `e2a4dfc`: graph 1,382 passed, 37 skipped; lidarslam 484 passed; `e2a4dfc..6a8727a` changes only four evidence/navigation documents | PASS lineage; public CI pending |
| Contributor entrypoint regressions | contract 10 passed; entrypoint/default-CI/docs group 30 passed | PASS |
| Issue proposal checker | 29 / 29 coverage; read-only live drift PASS; 18 tests passed; 0 mutations | PASS, not authorized to apply |
| Humble scanmatcher | immutable v0.9 Humble image, PCL 1.12; component recovery 10 consecutive PASS; complete CTest 10 / 10 | PASS |
| Jazzy scanmatcher | PCL 1.14/GCC 13.3; component recovery 10 consecutive PASS; complete CTest 10 / 10 | PASS |
| Scanmatcher lineage | no `scanmatcher/` change after component proof `bce5a9d` | PASS |
| Documentation | `mkdocs build --strict` | PASS with pre-existing Material/nav notices |
| C++ new-file style | `ament_uncrustify`; `ament_cpplint` with package-consistent copyright filter | PASS |
| v1 live readiness | 8 / 10; distribution incomplete; external adoption 0 / 3; stable v0.9 `PUBLISHED` | HOLD |
| Onboarding matrix | 2 / 4 present, 2 product PASS, 0 comparable; both source rows missing | HOLD |
| Fixture packet | 13 / 13 local checks; 98,873,952-byte ZIP; host unset; upload unauthorized | HOLD |
| Release-bundle rehearsal | exact candidate refused because `v0.9.0` names `0df0c4a`, not `6a8727a` | EXPECTED_BLOCK |

The Humble full-suite row is bound to the last code revision, not relabeled as
an exact-tip container run. The exact product candidate adds only the four
documents listed by `git diff --name-only e2a4dfc..6a8727a`; exact-tip Jazzy
and the focused documentation/entrypoint regressions pass. Supported public
Humble and Jazzy CI on a public carrier remains an E1 review requirement.

The release refusal is also correct. `VERSION` remains `0.9.0`, and the
historical tag must not be moved or reused. A future release decision must
choose a new semantic version, update all versioned records, and rerun the
two-build bundle and image proofs from a separately reviewed revision.

## G0 exit audit

| G0 exit | Current state | Required transition |
| --- | --- | --- |
| P1 #69 safe rejection | local implementation and dual-distro component recovery PASS; current candidate has no later scanmatcher change | publish reviewed carrier, pass supported public CI, and name the carrying release |
| Reviewable product line | fixed product candidate PASS; evidence carrier pending its final hash audit | explicit E1 authorization, non-force branch push, Draft PR, CI/diff review |
| Current issue triage | 29 / 29 proposal valid and live-current | explicit E3 authorization before any label, comment, or closure |
| Public fixture identity | local packet PASS | E2 host/upload decision, fresh readiness, and remote re-download audit |
| Four-row onboarding matrix | 2 / 4 outcomes, 0 / 4 comparable | public source/fixture identity, then fresh Humble/Jazzy Docker/source rows |
| v1 readiness | 8 / 10 | distribution completion and three independent first maps |

The phase decision is `HOLD`, not `ADVANCE`. Local quality and
reviewability do not prove public reproducibility, distribution, or adoption.

## Next bounded action

Use the separate
[G0 external-action decision packet](g0-external-action-decision-packet-2026-08-11.md).
If E1 is approved for the final named evidence-carrier hash, push only that
branch without force, open a Draft PR against `develop`, and wait for public
CI and review. E2 fixture publication, E3 community mutations, and E4 release
publication remain independent choices.

If E1 is deferred, continue safe local reliability and release-readiness work;
do not create local-path source trials or claim matrix comparability.
