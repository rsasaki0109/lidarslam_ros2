# G0 readiness dashboard

Use the read-only dashboard when reviewing the current 1,000-Star roadmap
gates. It composes the existing checkers; it does not replace them, run a
trial, publish an artifact, or write GitHub/community state.

```bash
python3 scripts/check_g0_readiness.py
```

The default command is local-only. It reports the exact publication inventory,
four privacy-safe capability review lanes,
the four-row Docker/source onboarding matrix, the independent first-map cohort,
v1 readiness, and the current action packet. It prints exactly one next action
with a safe read-only boundary. A `HOLD` is an honest gate state, not a checker
failure.

To bind the current checkout to public Draft PR #427 and its exact-head CI,
opt in to the GitHub GET-only product audit:

```bash
python3 scripts/check_g0_readiness.py \
  --include-product-draft
```

For GitHub API reads, the dashboard and its child audits use an explicit
`GITHUB_TOKEN` when provided, otherwise they non-interactively reuse the
active `gh auth` credential. If neither is available, they keep the anonymous
read path and fail closed when its quota or permissions are insufficient.
Credentials are attached only to exact `https://api.github.com` GET requests,
are never retained in reports, and add no write authority.

The audit requires the canonical repository, PR number and URL, `develop`
base, `agent/product-g0-guided-ux` head branch, and full local/public commit to
agree. An open PR must be mergeable. The latest run for each required check
must contain ten successes and the four expected non-publication skips; a
missing, pending, failed, truncated, malformed, or mismatched result is
`BLOCKED`. Additional checks are accepted only when terminal and non-failing.
The report distinguishes `DRAFT_REVIEW_REQUIRED`,
`READY_FOR_SEPARATE_MERGE_REVIEW`, and `MERGED`, while keeping
`merge_authorized: false` in every state.

When the full local and public heads differ, the dashboard does not tell the
operator to repeat the same audit. It first checks the two commits in the
local object database. A verified fast-forward produces one structured
`NON_FORCE_PR_BRANCH_UPDATE` handoff containing the exact public head, exact
local tip, canonical PR branch, separate authority requirement, and post-update
GET-only verification command. The handoff deliberately contains no push
command and keeps `push_authorized`, `force_push_authorized`, and
`writes_performed` false. Divergent history instead selects a read-only
merge-base inspection; unavailable local history selects a bounded fetch plus
ancestry check from the canonical GitHub repository URL instead of trusting a
checkout-specific `origin`. Neither path authorizes a push, PR state change,
or merge.

The same branch-update action also renders a canonical reviewer-facing PR
description from the clean exact local tip. The `REFRESH_EXACT_DRAFT_DESCRIPTION`
handoff binds the desired head, observed and desired body SHA-256, exact body,
whole-PR and P2 commit/path budgets, current evidence counts, and a separate
description-edit authority. It requires the branch update and GET-only
exact-head check first, keeps the PR Draft, contains no edit command, and keeps
description update, review submission, mark-ready, merge, and writes false.
This prevents a new branch tip from retaining an old PR summary.
The body is directly navigable on GitHub: three generated compare links bind
P0, P1, and P2 to their validated start/end commits, while a seven-row S1–S7
map names each focus, path count, verification count, and publication gate.
Four generated R1–R4 rows then group those slices by required capability:
runtime safety, operator UX, distribution, and integration/publication. The
advisory target is two reviewers, not a merge gate. The routing contract stores
no username, email, or organization and cannot request a reviewer, submit a
review, mark ready, or merge.
The dashboard accepts that map only when phase lineage is contiguous, phase
commits compose the whole PR, slice paths compose P2, titles cannot inject
Markdown, and every
source overview authority remains false. The final local-tip link becomes
publicly resolvable only after the separately authorized branch update.

Role selection is not review evidence by itself. An optional append-only,
identity-free ledger can be included after it has been prepared outside the
repository and bound to the same clean exact tip:

```bash
python3 scripts/check_g0_readiness.py \
  --product-draft-review-ledger \
  /tmp/lidarslam-pr427-review-ledger.json
```

The dashboard retains the ledger SHA-256, current PASS/BLOCKED/not-reviewed
lane counts, and open blocker count, but not its filesystem path or a reviewer
identity. Historical events remain in the ledger while the dashboard reports
only each lane's latest state. Even `COMPLETE_LOCAL_REVIEW` runs no check and
grants no reviewer request, submitted review, mark-ready, merge, or remote
write authority.

Dependency order is explicit. A green Draft points first to the bounded
overview and then to the seven-slice local review plan only when the observed
PR-description digest also matches the canonical clean-tip body. A stale or
missing description selects the same no-write description-refresh handoff
before review. The schema-bound
handoff fixes the exact public/local head, 396-path / three-phase / seven-slice
coverage, overview command, slice template, and four-step review sequence while
keeping command execution, review submission, mark-ready, merge, and all writes
false. A non-Draft open PR still requires a separate maintainer merge
decision. Repository-environment work cannot become the next action until the
exact PR is observed as merged. If an environment audit is requested without
the product audit, the dashboard asks for the missing product audit first
instead of suggesting a settings change from incomplete evidence.

Head equality is insufficient when the checkout is dirty. If the publication
plan reports any uncommitted path, the dashboard selects `git status --short`
instead of constructing an exact-head review handoff. It does not clean,
discard, commit, or publish those bytes.

The selected seven-slice review card is copy-ready from an ordinary terminal.
Before drilling into one slice, the local-only overview makes the large Draft
budget visible without pasting every path:

```bash
python3 scripts/check_publication_slice_plan.py --overview
```

It binds whole-PR and three-phase path identity to Git numstat, then reports
text additions/deletions, binary counts, and the largest textual path for each
phase and slice. Each exact slice card expands to three hotspots and names any
binary paths requiring manifest/content review. The budget is a navigation aid,
not evidence that review occurred, and it grants no GitHub write authority.

ROS-dependent commands source the caller's `ROS_DISTRO` installation and
default to Jazzy when it is unset. Pytest commands disable cache writes, and
the two package test roots run in separate processes to avoid their known
duplicate module basename. Plan validation rejects a missing ROS prelude,
mixed-package pytest command, cache-producing pytest command, or remote-write
CLI form recognized by the checker before displaying the card.

To include the stable-release audit, which performs network reads but no
remote writes, opt in explicitly:

```bash
python3 scripts/check_g0_readiness.py \
  --include-published-release \
  --published-release-version 0.9.1
```

To inspect the protected candidate environment through GET requests only,
pass `--include-candidate-environment`. Authenticated inspection avoids
mistaking an inaccessible endpoint for an absent environment:

```bash
python3 scripts/check_g0_readiness.py \
  --include-product-draft \
  --include-candidate-environment
```

The environment check first requires one complete repository-environment
inventory, then reads the exact `candidate-images` environment and its complete
deployment-policy list. It reports `ABSENT`, `MISCONFIGURED`, or `BLOCKED`
separately. `READY` requires one to six reviewers, **Prevent self-review**, no
unknown protection rule, and exactly one custom `develop` branch policy. Even
`READY` means only `READY_FOR_SEPARATE_E2_REVIEW`: environment writes and the
digest-publication dispatch remain unauthorized.

To take one complete read-only snapshot of the current external G0 gates, use:

```bash
python3 scripts/check_g0_readiness.py \
  --include-public-transition \
  --published-release-version 0.9.1
```

`--include-public-transition` is the user-facing alias for the exact product
Draft, protected `candidate-images` environment, and published-release audits.
It performs those bounded network reads together so a partial audit cannot
send the operator back to the same version-alignment command. The three
long-form flags remain available for focused diagnosis.

When this optional gate becomes the selected next action, the human card and
JSON contract carry the same bounded operator handoff. An absent or
misconfigured environment includes only the trusted repository-settings URL,
the exact reviewer/self-review/develop-only checklist, required external
authority, and the read-only verification command. An inaccessible audit does
not expose a settings URL or suggest mutation; it asks the operator to restore
read access and retry. The dashboard labels the handoff **not executed** and
rechecks that `writes_performed` remains false before displaying it.

For automation, use `--json`. The output follows the
[`g0-readiness-report-v1`](schemas/g0-readiness-report-v1.schema.json)
contract. `--require-ready` exits with status 1 while any summarized gate is
not ready and status 2 if a source checker or the dashboard contract is
invalid.

The dashboard deliberately does not turn a product `PASS` into a comparable
onboarding row. When Docker and source rows use different product versions,
its next action shows two structured, no-write choices: continue the current
candidate (which needs the protected `candidate-images` environment, a
separately authorized digest-only E2 dispatch, and a remote candidate-set
audit; the gate itself does not authorize publication),
or intentionally rebuild all four rows against one already-published version.
The second choice requires a fresh source preflight and fresh records; old
mixed-version measurements must never be reused. After one public identity is
selected and all rows are rebuilt or re-recorded against it, the next action
moves to the measurement gate. Human active time, submitted command count,
isolated disk measurements, and the external first-map acceptance gates remain
evidence requirements. Recruitment, release, image, issue, label, review, and
package actions remain separate decisions.

The same next action now includes a schema-bound
`READ_ONLY_PUBLIC_PRODUCT_TRANSITION` handoff. Before publication it reports
`AUDIT_REQUIRED`, `PUBLICATION_REQUIRED`, or `AUDIT_BLOCKED`, selects the
complete transition audit above, and marks fresh-packet generation ineligible.
Only a matching `PUBLISHED` report produces
`READY_FOR_FRESH_MATRIX_PACKET` and selects the fail-closed release-report →
observer-packet pipeline. The handoff fixes the target version, all three audit
IDs, both commands, and false GitHub-write/remote-mutation authority. Unsafe
version text or a child report for a different version fails before any
copy-ready command is rendered.

For the published-release choice, the dashboard now prints a copy-ready
pipeline from `check_published_release.py` into
`prepare_onboarding_matrix_packet.py --published-release-report -`. The packet
derives the tag commit and both image digests from the same schema-valid report
bytes and retains their SHA-256. Before a clean-host row starts,
`check_published_onboarding_identity.py` repeats the bounded network audit and
requires the live commit and both digests to match exactly. This removes four
manual identity fields without treating packet preparation as a release or a
trial.

When v1 is incomplete, the card and JSON report also expose each incomplete
gate's recorded detail and blocker list. This keeps distribution blockers such
as unresolved `ndt_omp` lineage, missing apt synchronization, or a missing
package-manager run visible without performing any external write. The
blockers are evidence for the next decision, not proof that an external action
has been taken.

When the independent first-map cohort is waiting for public gates, the card
also lists each pending launch prerequisite, such as comparable Docker/source
rows and the canonical documentation/runtime identity. The documentation gate
is byte-bound rather than URL-only. After a reviewed Pages deployment, audit
the selected route with:

```bash
python3 scripts/check_public_docs_deployment.py \
  --expected-revision <exact-40-character-public-commit> \
  --expected-product-version 0.9.1 \
  --route source-quickstart \
  --json
```

`VERIFIED` requires the deployment manifest revision, rendered page size and
SHA-256, product version, and selected route fragment to agree. `NOT_READY` or
`BLOCKED` keeps the cohort closed. The audit performs bounded network reads and
no writes. This makes the closed
cohort state actionable without rendering recruitment text or authorizing a
community write. Machine-readable JSON keeps the stable gate IDs; the human
card adds the concrete evidence required for each one, including the seven
measurements and immutable runtime identity. Unknown future IDs remain visible
and fail safe with a pointer to the cohort contract.

The contributor queue applies the same boundary to the already-published
tracking issue. `python3 scripts/contributor_starter_queue.py --next` does not
treat an open `good first issue` label as sufficient evidence. It evaluates
the declared cohort dependency locally and recommends #422 only when the
derived operating state is exactly `READY_FOR_NEXT_ATTEMPT`; otherwise it
reports the issue as blocked and points the maintainer back to the cohort
checker. Its JSON is validated against
[`contributor-next-action-v1`](schemas/contributor-next-action-v1.schema.json),
performs only bounded GitHub GETs, and cannot authorize recruitment or mutate
an issue.

The current packet is
[`g0-current-action-packet-2026-08-14.md`](evidence/growth/g0-current-action-packet-2026-08-14.md).
It supersedes the historical action snapshot for present handoff decisions
without authorizing remote mutation.
