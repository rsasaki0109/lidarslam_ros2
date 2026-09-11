# ndt_omp release-review audit — 2026-08-12

This is a read-only audit of the dependency blocking the normal
`apt install ros-<distro>-lidarslam` path. It corrects the earlier
wait-only interpretation of the two generated rosdistro pull requests.

## Result

The latest authenticated run of
`python3 scripts/check_ndt_omp_release_readiness.py --json` reports
`BLOCKED`: both generated PRs have an unanswered lineage review and one
failed check run at their exact heads.

- [Humble PR #52949](https://github.com/ros/rosdistro/pull/52949) is open,
  non-draft, and mergeable at
  `c375b1c8e92d14e58a4c10e023920763645fe5c7`.
- [Jazzy PR #52950](https://github.com/ros/rosdistro/pull/52950) is open,
  non-draft, and mergeable at
  `ef7e147af917eee64f4569a528dd98400004cadd`.
- The latest unanswered human review asks how this package relates to the
  existing `ndt_omp` package:
  [direct question](https://github.com/ros/rosdistro/pull/52950#pullrequestreview-4857894506),
  [matching Humble question](https://github.com/ros/rosdistro/pull/52949#pullrequestreview-4857900792).
- The source tag and Bloom release repository already exist. Recreating the
  tag or rerunning Bloom does not answer the review.
- Each exact head has six check runs: four successful, one neutral, and one
  failed `rosdistro / rosdep checks (3.8)` run. Neutral is non-blocking, so
  the machine report records 5/6 passing and 1/6 failing for each PR.

No GitHub comment, PR, tag, repository, or distribution file was changed by
this audit.

## Read-only state refresh — 2026-08-15

An authenticated, thread-aware read-only rerun found no inline review threads
on either PR. The actionable feedback is instead a human review body on Jazzy
asking how the package relates to `ndt_omp`; the Humble review points to that
same question. Both remain unanswered, both PRs remain open at the same exact
heads above, and GitHub currently reports `mergeable=true`. Mergeability only
records the current base-conflict calculation; it does not grant reviewer
approval or make the colliding packages release-ready.

The checker now sends an optional `GITHUB_TOKEN` only to `api.github.com`,
binds every check run to the PR's exact head SHA, prints check totals beside
mergeability, and fails closed for failed, pending, missing, or truncated
check-run evidence. A CI blocker does not hide a pending human-review action.
Focused tests cover authenticated request scoping, explicit conflicts,
unknown mergeability, all check-run gate states, retained review actions, and
readable output. No external state was changed by the refresh.

### Exact-head CI classification

| PR | Exact-head check result | Failing job |
| --- | --- | --- |
| Humble #52949 | 5/6 passing, 0 pending, 1 failing | [`rosdistro / rosdep checks (3.8)`](https://github.com/ros/rosdistro/actions/runs/30493787110/job/92098241682) |
| Jazzy #52950 | 5/6 passing, 0 pending, 1 failing | [`rosdistro / rosdep checks (3.8)`](https://github.com/ros/rosdistro/actions/runs/30494032954/job/92098296862) |

Both failed logs report 1 failed and 13 passed tests. The only failure is the
OpenEmbedded lookup for `libpcre@openembedded-core`; it does not inspect or
reject the 15-line `ndt_omp_ros2` registration itself. ros/rosdistro
[PR #52858](https://github.com/ros/rosdistro/pull/52858) subsequently moved
that key to `libpcre@meta-ros-common`, current `master` contains the corrected
rule, and a later
[`Validate rosdistro` run](https://github.com/ros/rosdistro/actions/runs/31842406951)
passes. Each old NDT branch has one candidate commit absent from `master`,
while current `master` has 307 commits absent from that branch.

It is therefore a supported inference that these two failures come from the
stale branch base and global rosdep state, not the NDT YAML delta. This does
not turn either exact head green. Do not spend effort refreshing the colliding
registration as-is: select the canonical or fully isolated path first, then
refresh or recreate the generated PR from current rosdistro `master` and
require a complete green suite.

## Relationship to the existing package

`ndt_omp_ros2` is a downstream ROS 2 port/fork of
[koide3/ndt_omp](https://github.com/koide3/ndt_omp), not an independent
registration implementation. The fork README says this explicitly, and its
history contains the original `pclomp` implementation.

The current upstream package already supports ROS 2 and exports the same
algorithm family. Humble rosdistro registers and releases `ndt_omp` version
`0.0.0-1`; Jazzy does not currently register it. The upstream Hessian
correction is also already merged as
[koide3/ndt_omp#46](https://github.com/koide3/ndt_omp/pull/46).

The product-specific delta that `scanmatcher` currently consumes is fork
commit `d435e32`:

- rotation-prior setters and clearing;
- translation-prior setters and clearing;
- maximum NDT correspondence-distance filtering;
- mean correspondence-distance reporting after alignment.

`scanmatcher/src/scanmatcher_component.cpp` calls all four API groups for its
opt-in IMU NDT prior, z prior, and adaptive correspondence threshold.
Buildfarm metadata, target export, installed-consumer tests, and Humble/Jazzy
CI are release-quality improvements, but they do not make the implementation
a separate algorithm.

## Co-installation audit

The current fork is not safely co-installable with Humble's released
`ndt_omp`, even though its ROS package and CMake package are named
`ndt_omp_ros2`.

| Surface | Existing `ndt_omp` | Candidate `ndt_omp_ros2` | Result |
| --- | --- | --- | --- |
| ROS package | `ndt_omp` | `ndt_omp_ros2` | distinct |
| CMake package | `ndt_omp` | `ndt_omp_ros2` | distinct |
| C++ namespace | `pclomp` | `pclomp` | collision |
| installed headers | `include/pclomp/*` | `include/pclomp/*` | file-ownership collision |
| shared library | `lib/libndt_omp.so` | `lib/libndt_omp.so` | file/SONAME collision |
| algorithm lineage | koide3 implementation | downstream fork | overlapping implementation |

The two Debian packages can therefore claim the same files under
`/opt/ros/humble`. A unique repository key alone is not a sufficient
co-installation boundary. The current rosdistro PRs must not be described as
merge-ready until this is resolved with the reviewer.

## Convergence decision

The preferred durable path is upstream convergence:

1. Submit the four product-required APIs with focused tests to
   `koide3/ndt_omp`.
2. Change both direct consumers, `scanmatcher` and `graph_based_slam`, to
   depend on the canonical `ndt_omp` package and exported target.
3. Coordinate a Humble update and first Jazzy release of that canonical
   package.
4. Close or supersede the current `ndt_omp_ros2` registrations; never install
   overlapping headers and libraries under a second Debian package name.

If upstream explicitly declines the project-specific APIs, the fallback is a
fully isolated fork. That requires a new package identity plus distinct C++
namespace, include directory, library/SONAME, CMake target, executable path,
version/tag, Bloom tracks, and rosdistro PRs. Merely renaming `package.xml` is
not enough.

This decision prioritizes one maintained implementation and normal ROS package
resolution over a project-specific duplicate. Source and Docker onboarding
remain available while that external coordination proceeds.

## Prepared and verified convergence implementation

The upstream-first path is no longer only a proposal. A local patch was
prepared against exact `koide3/ndt_omp` commit
`5495fd9214945afcb4b35d5a1da385e405c52bf9`:

- artifact:
  `packaging/ndt_omp/koide3-ndt-omp-lidarslam-priors.patch`;
- SHA-256: `7b641c32ec4f30faa302e60aaa89765bb9acf67f3f0feb85f9e4e11e88b4dc9f`;
- payload: rotation and per-axis translation priors, correspondence-distance
  filtering, accepted-correspondence diagnostics, input sanitization, and
  four focused GTests;
- implementation detail: correspondence statistics use the upstream
  per-input-point reduction order, preserving its deterministic summation
  rather than transplanting the fork's thread-local accumulator.

The checked-in contract and read-only checker bind the exact upstream commit,
patch hash, five-path inventory, API markers, test names, parent-consumer
inventory, and no-write authority boundary:

```bash
python3 scripts/check_canonical_ndt_convergence.py --json
python3 scripts/check_canonical_ndt_convergence.py \
  --upstream-checkout /path/to/clean/koide3-ndt_omp \
  --require-ready-for-upstream-review
GITHUB_TOKEN="$(gh auth token)" \
python3 scripts/check_canonical_ndt_convergence.py \
  --upstream-checkout /path/to/clean/koide3-ndt_omp-at-5495fd9 \
  --candidate-checkout /path/to/clean/ndt_omp-at-618f02f \
  --online \
  --require-ready-for-draft-pr
```

The strict command reported `READY_FOR_UPSTREAM_REVIEW` with 20/20 checks
against a clean detached checkout of that exact commit. The report intentionally
omits the checkout path. This state means the local bundle is reviewable; it
does not authorize or claim an upstream PR. The stronger online command then
reported `READY_FOR_DRAFT_PR` with 30/30 checks, binding exact candidate commit
`618f02f`, its exact parent/subject/diff hash, current upstream `master`, fork
identity, proposed-branch absence, and a zero-match search across four open
upstream PRs. It omits both checkout paths and still reports
`github_writes_authorized=false` and `remote_mutations_performed=false`.

The exact patch was also fixed as local upstream commit
`618f02f6b50a8590b81f48b4fee5b6cfc8d3f3ea` and rebuilt from a clean Jazzy
worktree. The copy-ready Draft PR title/body, duplicate check, exact fork
branch, validation result, publication order, and no-write boundary are in the
[upstream PR packet](growth/canonical-ndt-upstream-pr-packet-2026-08-12.md).
The commit is local-only and is not a publicly resolvable revision.

The parent boundary was then tested with a copied `scanmatcher` package that
depends on canonical `ndt_omp`. That first exposed two additional downstream
compatibility deltas: the fork-only `setOulierRatio` misspelling and direct
use of Boost shared pointers after modern PCL changed its `Ptr` alias. The
parent source now uses `pcl::Registration::Ptr`, `pcl::make_shared`, and
`pcl::dynamic_pointer_cast` in both frontend and loop-registration paths;
the existing `ndt_omp_ros2` build remains compatible with that modernization.

The remaining canonical switch is captured separately as
`packaging/ndt_omp/lidarslam-canonical-ndt-transition.patch` (SHA-256
`c090b8f2228b21dcf30650114f9638f38497ca5a0214e3e6063a53aa7bef66b1`).
The earlier three-file draft was incomplete: `graph_based_slam` also directly
declared canonical-provider requirements in its manifest and seven CMake
locations. The corrected five-file patch replaces all eleven direct
dependency references across both consumers and corrects the fork-only
outlier-setter spelling. The fail-closed checker compares those counts with
the live parent files and rejects omission or drift.

Verification completed locally:

- canonical upstream `ndt_omp`: Humble and Jazzy build/install passed; all
  4/4 new GTests passed on both distributions;
- complete canonical transition: `ndt_omp`, `lidarslam_msgs`, `scanmatcher`,
  and `graph_based_slam` built and installed from the same staged source on
  network-isolated immutable Humble and Jazzy images;
- installed manifests and both consumer CMake caches resolve `ndt_omp` from
  the new prefix; no `ndt_omp_ros2` package/share or dependency remains;
- the convergence checker has 11/11 regressions passing, including artifact
  drift, graph-consumer omission, dirty/wrong upstream checkout, apply
  failure, private-path exclusion, create-only output, and authority rejection;
- the complete maintained product Python gate passes 2,098 tests
  (`graph_based_slam` 1,428 and `lidarslam` 670), with 13 known skips and no
  failures;
- the earlier canonical `scanmatcher` test run remains 109/109 passing;
- current fork + modernized parent (`ndt_omp_ros2`, `scanmatcher`, and
  `graph_based_slam`): all three packages built and installed from clean
  build directories;
- the first full parent run passed 227/232 CTest targets. The five failures
  were two free-space safety gates after temporary builds reduced free space
  below 5 GiB, plus three pre-existing lint findings in newly added map-edit
  files. The generated build/install directories were removed, the lint
  findings were corrected, and all five failed targets passed their focused
  reruns (15/15 functional cases plus copyright, cpplint, and flake8).

These artifacts were prepared but not submitted. No upstream PR, review
reply, branch push, tag, Bloom run, or rosdistro mutation was performed.

## Prepared reviewer responses

These responses are prepared but were not posted. The Jazzy response is the
full answer to the direct question:

> Thanks for catching this. `ndt_omp_ros2` is a downstream ROS 2 fork of
> `koide3/ndt_omp`, not an independent implementation. It carries four APIs
> used by lidarslam's optional IMU/translation priors and adaptive
> correspondence threshold, plus buildfarm packaging work. However, the
> current candidate still installs the same `include/pclomp/*` headers and
> `libndt_omp.so` as Humble's released `ndt_omp`, so the differently named
> Debian packages are not safely co-installable. I do not want these PRs
> merged as-is. My preferred correction is to upstream the required APIs and
> consume/release the canonical `ndt_omp` package for Humble and Jazzy. The
> focused upstream work is tracked in Draft PR `<UPSTREAM_PR_URL>`. If those
> project-specific APIs are declined upstream, I will instead fully namespace
> the fork's package, headers, symbols, library, and CMake target, then replace
> these Bloom registrations. I will report the selected collision-free path
> here before requesting another merge review. The current red rosdep check
> also remains a hard gate; any replacement registration will be generated
> from current rosdistro `master` and must be fully green.

The Humble response deliberately points to the same resolution instead of
duplicating a divergent explanation:

> The same lineage and co-installation issue applies here as in Jazzy
> #52950. `ndt_omp_ros2` is a downstream fork that overlaps the canonical
> `ndt_omp` headers and library, so please do not merge this registration
> as-is. I am pursuing the collision-free canonical path in Draft PR
> `<UPSTREAM_PR_URL>` and will return with the accepted resolution before
> requesting another rosdistro review. Any replacement PR will be generated
> from current rosdistro `master` and must pass its complete check suite.

Do not post either response while the literal `<UPSTREAM_PR_URL>` placeholder
remains, or unless that URL resolves to the Draft upstream PR containing exact
candidate commit `618f02f6b50a8590b81f48b4fee5b6cfc8d3f3ea` (or a deliberately
reviewed successor).

Posting either response or changing either external PR requires an explicit
maintainer publication decision.

## Fail-closed response packet gate — 2026-08-16

The release checker now turns the prepared prose above into a copy-ready
packet only after a second read-only identity gate. It requires all of the
following at the same time:

- the online release audit completed without remote-inspection errors;
- both rosdistro PRs remain open at exact heads `c375b1c...` and
  `ef7e147...`, with the exact unanswered review URLs recorded above;
- the canonical URL resolves to an open Draft PR targeting
  `koide3/ndt_omp:master` from the expected fork;
- the Draft head is exact candidate commit `618f02f6...`; and
- no rosdistro response has already superseded the recorded review state.

If any condition fails, packet status is `BLOCKED`, both response bodies are
`null`, and the human output says `No copy-ready response was emitted.` A red
rosdistro check does not suppress the lineage response because that response
explicitly preserves green CI as a separate hard gate. It still prevents the
registration itself from being described as release-ready.

Current no-PR inspection is intentionally blocked:

```bash
python3 scripts/check_ndt_omp_release_readiness.py \
  --review-response-packet \
  --json
```

After the canonical Draft exists, a maintainer can bind and inspect the exact
copy without posting it:

```bash
GITHUB_TOKEN="$(gh auth token)" \
python3 scripts/check_ndt_omp_release_readiness.py \
  --review-response-packet \
  --upstream-pr-url "${verified_upstream_pr}" \
  --require-review-response-ready \
  --json
```

The packet always reports `github_writes_authorized=false` and
`remote_mutations_performed=false`. Focused regression coverage is 25/25,
including exact-target unlock, missing URL, rosdistro-head drift, non-Draft
upstream state, wrong repository URLs, and strict-gate refusal. The live
canonical preflight was also refreshed on 2026-08-16 from clean temporary
worktrees and remains `READY_FOR_DRAFT_PR`, 30/30 checks, with the proposed
branch absent and zero semantic duplicate PRs. No GitHub state was changed.
