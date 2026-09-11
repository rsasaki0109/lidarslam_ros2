# v1.0 Readiness

`lidarslam_ros2` uses a fail-closed, machine-readable audit for the complete
v1.0 product goal. It covers the nine readiness dimensions in the
[v0.9 roadmap](roadmap/v0.9.md#1-measurable-v10-readiness-targets) plus the
stable v0.9 release-candidate publication gate.

Run the audit from a full git checkout:

```bash
python3 scripts/check_v1_readiness.py
```

For automation:

```bash
python3 scripts/check_v1_readiness.py --json
python3 scripts/check_v1_readiness.py --live --json
python3 scripts/check_v1_readiness.py --require-complete
```

The normal command reports an honest snapshot and exits zero when the
contract is valid, even while product work remains. It is deterministic and
does not contact remote services. `--live` additionally runs the read-only
NDT rosdistro, exact Humble/Jazzy main-channel package-manager, and
stable-release publication audits. `--require-complete` exits 1 unless every
gate is complete; if local evidence could otherwise produce `READY`, it
automatically runs those live audits before accepting the result. An invalid
contract, schema, evidence path, adoption ledger, version, git-tag query, or
untrustworthy live inspection exits 2.

For its GitHub API reads, the live audit uses an explicit `GITHUB_TOKEN` when
provided, otherwise it non-interactively reuses the active `gh auth`
credential. If neither is available, it keeps the anonymous read path and
fails closed when that quota is insufficient. Credentials are attached only
to the exact `https://api.github.com` origin, are never included in output,
and grant no publication authority; every request remains GET-only. A valid
incomplete child state remains product evidence rather than becoming an audit
error:
package-manager status is preserved as `SOURCE_REF_MISSING`, `NOT_RUN`,
`RUNNING`, `FAILED`, `READY`, or `BLOCKED` in both JSON and the human status
tuple.

## Authoritative inputs

- [`contracts/v1-readiness.json`](contracts/v1-readiness.json) records the
  reviewed state, evidence paths, and explicit blockers for every gate.
- [`schemas/v1-readiness-contract-v1.schema.json`](schemas/v1-readiness-contract-v1.schema.json)
  prevents a gate, blocker, or evidence field from silently disappearing.
- [`schemas/v1-readiness-report-v1.schema.json`](schemas/v1-readiness-report-v1.schema.json)
  defines the generated JSON report.
- [`evidence/external-first-map-validations.json`](evidence/external-first-map-validations.json)
  remains the source of truth for the independent three-user gate.
- Root `VERSION` and local immutable git tags are checked for the v0.9
  publication gate.
- The distribution gate includes the read-only
  `scripts/check_ndt_omp_release_readiness.py` preflight and its
  [`ndt-omp-release-readiness-v2.schema.json`](schemas/ndt-omp-release-readiness-v2.schema.json)
  contract. It distinguishes a locally reviewed candidate from
  `READY_TO_TAG`, partial publication, unanswered human review, exact-head
  check failure, and a completed rosdistro release.
  The initial and current reviewed results are preserved in
  [`ndt-omp-release-preflight-2026-07-29.md`](evidence/ndt-omp-release-preflight-2026-07-29.md)
  and
  [`ndt-omp-release-progress-2026-07-30.md`](evidence/ndt-omp-release-progress-2026-07-30.md).
  The later
  [release-review audit](evidence/ndt-omp-release-review-2026-08-12.md)
  records the existing-package collision and upstream-first convergence plan.
  The exact patch/base/consumer inventory is independently checked by
  `scripts/check_canonical_ndt_convergence.py` against the versioned
  [`canonical-ndt-convergence-v1.json`](contracts/canonical-ndt-convergence-v1.json)
  contract. Its `READY_FOR_UPSTREAM_REVIEW` result covers local review; the
  stronger `READY_FOR_DRAFT_PR` result additionally binds the exact candidate
  commit and current upstream/fork/branch/duplicate state. Only that fully
  green state emits a schema-bound `draft_pr_handoff` containing the exact
  create-only branch, base/head SHAs, Draft title/body, and post-publication
  GET-only route; all lesser states emit `null`. Push, PR creation,
  force-push, mark-ready, and merge authority remain false. Both statuses are
  technical evidence, not GitHub write authority.
- The package-manager distribution evidence has a versioned
  [`package-manager-install-v1.schema.json`](schemas/package-manager-install-v1.schema.json)
  contract and Humble/Jazzy clean-install/upgrade workflow. It remains an
  open gate until actual ROS testing/main packages produce passing artifacts.
  The dependency-only
  `scripts/check_ros_apt_dependency_readiness.py` preflight first distinguishes
  missing testing binaries from dependencies that have synced to the normal
  channel.
  The live
  `scripts/check_package_manager_release_readiness.py` audit independently
  requires a successful workflow dispatch named for the exact immutable
  source tag, product version, `main` apt channel, and `clean-install` mode.
  It resolves lightweight or annotated tags to a commit, binds the run head to
  that commit, and reports missing refs, running attempts, and failed attempts
  separately instead of collapsing them into `NOT_RUN`.
  The parent v1 live audit accepts all six schema-valid child states and
  exposes the exact one as `publication_audits.package_manager_release_status`.
  Only `READY` can close distribution; every other valid state produces an
  actionable `NOT_READY` report instead of exit 2.
  Its
  [`package-manager-release-readiness-v1.schema.json`](schemas/package-manager-release-readiness-v1.schema.json)
  report requires both named-distro jobs to pass.
- Release publication and first rollback evidence use the read-only
  `scripts/check_published_release.py` audit and
  [`published-release-v1.schema.json`](schemas/published-release-v1.schema.json).
  The audit cross-checks the stable release, tag commit, six attached assets,
  every file hash in the release bundle, and the current Humble/Jazzy GHCR
  version-tag digests.

The checker verifies that all ten expected gate IDs are present exactly once,
that evidence paths remain inside the repository and exist, that semantic
versions and tags align, and that the independent-user ledger passes its own
schema and uniqueness rules. Live evaluation additionally requires
`ndt_omp_ros2` to report `RELEASED` and the exact main-channel Humble/Jazzy
package-manager run to report `READY` before the distribution gate can
complete. It also requires the stable release to report `PUBLISHED` before
the reliability or release-publication gates can complete. A tracked
`complete` state is therefore an audited attestation backed by named
evidence—not a substitute for rerunning the wider CI, real-data, or release
workflows cited by that evidence.

## Current snapshot

The tracked state is **NOT_READY: 8/10 gates complete**.

The authenticated 2026-08-17 live audit also reports 8/10 with exact child
status tuple `BLOCKED / SOURCE_REF_MISSING / PUBLISHED` for NDT,
package-manager, and the current stable release. This is a valid incomplete
state, not malformed evidence.

| Open gate | Remaining proof |
| --- | --- |
| Distribution | `ndt_omp_ros2` is `BLOCKED`: answer the lineage question and converge on canonical `ndt_omp` or a fully isolated fork before replacing rosdistro PRs [#52949](https://github.com/ros/rosdistro/pull/52949) and [#52950](https://github.com/ros/rosdistro/pull/52950); their current heads each have one failed stale-base rosdep check, so require a current-base green replacement, then wait for Jazzy RKO-LIO 0.3.2 main sync and run package-manager E2E |
| External adoption | Accept three distinct independent first-map validations; current ledger is 0/3 |

The reliability and release-publication gates closed when the immutable
[`v0.9.0` release](https://github.com/rsasaki0109/lidar_slam_ros2/releases/tag/v0.9.0)
and its six recovery assets passed the
[published-release audit](evidence/published-release-v0.9.0-2026-07-31.md).

This table is explanatory. The generated report and contract are
authoritative. Update a gate to `complete` only in the same reviewed change
that adds its final public evidence. Never remove an unmet gate or mark an
external action complete based on intent.
