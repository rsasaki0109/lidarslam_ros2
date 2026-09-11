# Weekly growth scorecard

The 1,000-Star roadmap treats Stars as a lagging signal. The weekly scorecard
records the smaller set of aggregate signals needed to tell whether people can
discover the project, produce a verified first map, and join the contributor
loop.

## Capture one snapshot

Authenticate the GitHub CLI with an account that can read repository traffic,
then run:

```bash
gh auth status
python3 scripts/collect_growth_snapshot.py \
  --captured-at 2026-08-10T00:00:00Z \
  --annotation "v0.9 onboarding baseline" \
  --output docs/evidence/growth/2026-08-10.json
```

Omit `--captured-at` for the current UTC time. The output path is created with
exclusive-create semantics: an existing weekly snapshot is never overwritten.
Without `--output`, the validated JSON is printed to stdout.

The first tracked aggregate is the
[2026-08-10 G0 baseline](evidence/growth/2026-08-10.json).
Its first product decision is the
[G0 activation decision](evidence/growth/g0-activation-decision-2026-08-10.md):
measure and repair the clean Docker/source first-map matrix before expanding
promotion.

The next privacy-bounded weekly snapshot is
[2026-08-13](evidence/growth/2026-08-13.json). It records 837 Stars, 321 unique
clones, 281 unique views, and 13 downloads of the primary v0.9.0 release
bundle. The product gates remain unchanged at 0/3 accepted independent maps
and 8/10 v1 readiness; the snapshot is a measurement update, not an adoption
claim.

The previous read-only snapshot is
[2026-08-15](evidence/growth/2026-08-15.json), captured at
`2026-08-14T19:00:23Z`. It records 837 Stars, 353 unique clones, 267 unique
views, 5 unique Autoware/TIER IV referrals, and 18 downloads of the primary
v0.9.0 release bundle. It still records 0/3 accepted independent maps and
8/10 v1 readiness. The snapshot contains aggregate metrics only; it does not
write stargazer identities, raw GitHub records, or product telemetry.

The current read-only snapshot is
[2026-08-17](evidence/growth/2026-08-17.json), captured at
`2026-08-16T20:52:07Z`. It records 839 Stars, 450 unique clones, 238 unique
views, 3 unique Autoware/TIER IV referrals, and 21 downloads of the primary
v0.9.0 release bundle. Relative to the previous rolling snapshot, Stars rose
by 2, unique clones by 97, and bundle downloads by 3, while unique views fell
by 29 and qualified referrals by 2. First-map adoption remains 0/3, external
merged contributors remain 0, untriaged issues remain 16, and v1 readiness
remains 8/10. This mixed funnel does not justify a promotion or roadmap-phase
transition: activation, qualified referral, distribution, and community gates
remain open even though discovery and artifact interest increased.

The parallel community decision is the
[2026-08-11 contributor backlog](evidence/growth/community-contributor-backlog-2026-08-11.md).
A read-only audit grouped all 29 open issues and prepared five tasks with exact
files, non-goals, acceptance criteria, and focused checks. The candidates have
not been published as GitHub issues and no labels were changed.

The external-adoption operating loop is prepared in the
[first-map validator cohort packet](evidence/growth/first-map-validator-cohort-launch-packet-2026-08-12.md).
Its machine contract fixes a five-attempt first batch, two-attempt review WIP,
the ten-attempt 80%/ten-minute decision gate, privacy and independence rules,
service levels, and stop/repair conditions. Recruitment text remains
fail-closed until one comparable Docker and source row, a canonical public
path, an exact public revision, and the copy-ready handoff are all public.
No community post or GitHub write is authorized by that packet.
Its anonymous operating state now makes the stop/repair loop executable: one
command cross-checks accepted IDs against the adoption ledger and derives
capacity, repeated-blocker, initial/hard-cap, completion-rate, and median-time
decisions without storing participant handles. The tracked empty state remains
an honest zero-attempt record and cannot render recruitment text. Operational
signals expire after 48 hours, and accepted attempts must also match the
ledger's public report, route, and immutable product identity.

The follow-up
[complete triage proposal](evidence/growth/open-issue-triage-proposal-2026-08-11.md)
covers all 29 open issues with a label, priority, disposition, evidence, and
application gate. Its checker validates offline coverage and can fail closed on
live GitHub drift using GET requests only. The proposal remains unapplied and
unauthorized. A 2026-08-17 live read-only audit still reports `PASS`: 29 issues,
23 close proposals, and 6 keep-open or current-reproduction proposals. This
confirms proposal freshness without treating it as completed triage.
The companion application-packet generator turns either one row or all 29 rows
into deterministic, evidence-hashed maintainer review cards. It has no output
file or GitHub write mode, keeps #422 monitor-only, and leaves nine
starter-dependent rows blocked for content review and separate authorization.
The current community-queue follow-up at
`e4ce0aa6eb7d53423423a02af65f923472708f44` keeps that cohort issue blocked for
contributors while allowing one duplicate-free independent C5 body to become
the maintainer's local preview. C6–C9 remain queued, and no issue, label,
assignment, comment, or pull request is created or authorized.
Exact follow-up `0a34e724875d53b8ef74acd8a51fd500ce014ff5` binds that preview's
title, sorted labels, heading-free body, and task/queue/body digests into a
schema-valid local publication handoff. The live handoff remains explicitly
write-required, confirmation-required, and issue-creation-unauthorized.
Exact follow-up `5dc04198549341b33becdeb2bc058117db9fe78f` closes the remaining
public-base gap. Publication review now requires PR #427 merged and the exact
queue present with matching canonical SHA-256 on public `develop`. The current
GET-only state is `WAITING_FOR_PRODUCT_MERGE` plus public queue `ABSENT`, so the
maintainer may prepare C5 for post-merge review but cannot be told to publish
it. Seventy-one queue and 331 S6 integration regressions pass; no GitHub write
or publication authority is added.
The P1 #69 card is synchronized to the public Draft and supported CI as of
2026-08-17, explains both leaf parameters and the resolution tradeoff, and
keeps the unavailable historical bag and missing carrying release explicit.
Its live mode now source-binds that dated statement to PR #427 head
`4b2ab514`, open/Draft/mergeable state, 10 successful checks, 4 intentional
skips, and zero pending or failing checks. A second GET-only claim binds latest
stable `v0.9.0` to commit `0df0c4a`, 52 commits behind fix `a2368c4`, and
requires both the `v0.9.1` tag and release to remain absent. Any Draft, CI,
stable ancestry, or candidate-publication drift produces no review packet.

The current
[G0 clean-candidate audit](evidence/growth/g0-clean-candidate-audit-2026-08-11.md)
binds the reviewed local product revision to the unchanged public `develop`
base, diff inventory, current gates, and dual-distro #69 verification. Its
[decision packet](evidence/growth/g0-external-action-decision-packet-2026-08-11.md)
keeps source push, fixture upload, issue mutations, and release publication as
four separate approvals. The candidate is reviewable locally; G0 remains
`HOLD`.

The [current capture-bound action packet](evidence/growth/g0-current-action-packet-2026-08-14.md)
supersedes the historical snapshot for present handoff decisions. It binds
Draft PR #427, the reviewed product-candidate tip
`3d64ed556aca8a680f09e0f7e8c12a3c8d3e6a6d`; this is the reviewed code-bearing
tip, followed by later docs-only handoff synchronization and product UX
follow-ups, capture-time public Draft baseline
`3ed632e6f6aa1e3ca7f32d893773de1079086ffb` with 10 successful checks and 4
intentional non-publication skips, the current 349-path local plan plus
complete 396-path / three-phase whole-PR review coverage, the
bounded one-card PR review overview and seven exact drill-down slices with
Git-derived text budgets, top-three hotspots, and named binary review paths, the
four role-based capability lanes with a two-reviewer advisory target and no
stored identities or reviewer-request authority, the optional append-only
exact-tip lane ledger with identity-free PASS/BLOCKED events and no GitHub
review authority, the
one-command candidate-session tip `8bc5ea4`, and its guided read-only host
readiness follow-up `a286c65` while the v0.9.1/image state remains unpublished.
The optional authenticated environment preflight at `adecca6` now proves from
a complete live inventory that only `github-pages` exists and
`candidate-images` remains `ABSENT`; it performs no settings write and cannot
authorize E2. Its human and G0 cards now turn that stable blocker into one
bounded administrator handoff: trusted settings URL, exact protected-policy
checklist, independent review, and a copy-ready GET-only verification command.
The handoff is schema-bound to `writes_performed: false`; it is guidance, not
evidence that repository settings changed. The exact operator-handoff and
parse-safety tip is `e2d916a66a57146b0efe4c74e57218d56342ed37`; the later
content-verified GLIM comparison tip is
`3de7f84bb51acd2bd1c2b40724529be9c281d2fe`; the byte-bound public-docs
deployment-provenance implementation tip is
`602c7ad5a64332eb458d98b1a53783ba4f3cdecb`, followed by public evidence refresh
`3ed632e6f6aa1e3ca7f32d893773de1079086ffb`. The current local release packet
also derives its commit and both Docker digests from one published-release
audit at implementation tip
`289f7675a242b00f342528483cde3e5f602a11fc`, then repeats the exact live
identity check before a clean-host row. The paired usability recorder
follow-up at
`0575fb6d67dc0b2069d9e41029a767bf3608687c` removes manual scorecard JSON
editing while preserving missing observations as non-comparable. It still has
no external paired record and makes no GLIM parity claim. The current local
doctor recovery tip
`a83bbfeaea8196a19513c7a26772d500fe8419b8` also turns a reproduced
five-finding unconfigured-shell result into one dependency-ordered **Do this
now** action while retaining all stable finding codes and performing no
network access or write. This is local activation evidence, not an external
first-attempt or GLIM parity result. The current local
paired-preparation follow-up removes manual public-resolvability Booleans: one
GET-only preflight must resolve both fixed canonical commit/tag or registry
identities and approved documentation redirects before either worksheet is
published. A fixed-name receipt SHA-binds both exact worksheet files; all three
outputs roll back together. The recorder requires and archives that untouched
triplet, and the checker rejects explicit completed records without the
preparation chain. Offline preparation remains explicitly non-public. This
makes the handoff safer but still creates no human observation or comparative
claim. The public-docs
read-only audit keeps the
cohort closed while the deployed Pages manifest is absent or its exact page
bytes, route, version, and source revision do not match.
It continues to keep E2 artifact hosting, E3 community mutation, and E4
release publication
separate and unauthorized. The earlier 219-path value remains historical
capture-time evidence only.

The [G0 readiness dashboard](g0-readiness.md) is the single read-only entry
point for rechecking those local gates:

```bash
python3 scripts/check_g0_readiness.py
```

Use `--include-product-draft`, `--include-published-release`, or
`--include-candidate-environment` only when the corresponding network-read
audit is wanted. The exact-head product audit now puts Draft review and a
separate merge decision ahead of candidate-environment administration; it
never turns green CI into merge authority. A clean exact green Draft now
produces one schema-bound overview → three-phase → seven-slice review sequence;
a dirty checkout stops at read-only status inspection so local bytes are not
mistaken for the public review.
Branch drift now carries a second, separately authorized no-write handoff for
the PR description. It renders one canonical exact-head body, hashes the
observed and desired descriptions, replaces stale commit/path scope with the
current whole-PR and P2 review budgets, requires the branch update first, and
keeps Draft, review, mark-ready, merge, and all write authority false. An exact
green head with a mismatched body is stopped at that refresh before review.
The canonical body now removes one more reviewer lookup step: P0–P2 are exact
GitHub compare links and S1–S7 are a bounded focus/path/check/gate table. Link
lineage, commit composition, slice composition, safe titles, clean-tip
identity, and no-write source authority are machine checked before that body
can be emitted.
Exact follow-up `4404f877263157d09ae6c451dae55f5ddbbd03af` removes the
mixed-version partial-audit loop. One `--include-public-transition` option now
reads the Draft, protected environment, and v0.9.1 publication together and
returns a schema-bound transition handoff. The live GET-only result finds the
Draft behind the clean local tip but fast-forwardable, `candidate-images`
absent, and v0.9.1 tag/Release/both images absent; its dependency-ordered next
action is therefore exact non-force Draft-update review, not fresh measurement.
Only a matching published identity can enable a new four-row packet, and old
mixed-version measurements remain explicitly non-reusable.
Exact local discovery follow-up
`8a8876a2d26c09cc92ad330b99d1fa217db1bd8d` reduces the GitHub README choice
to three explicit goals: stable Docker demo, read-only own-bag diagnosis before
`start`, or candidate source dry-run. Docker is the default when unsure and
each row states its version, write, ROS, disk, and time boundary. The README
remains 219 lines; 29 focused entrypoint regressions, the 36-test S6
docs/product command, and strict MkDocs pass. This changes no release identity,
collects no telemetry, and is not a paired GLIM result.
Exact follow-up `1e56431161d3dea417d5d800bb1eedc3cdb51907` applies the same
three-goal decision to canonical Getting Started, while preserving seven
installed or post-success actions in a rendered collapsed section. It also
corrects the Docs Home v0.9.0 label from release candidate to stable release.
Thirty focused entrypoint tests, the 37-test S6 docs/product command, and
strict MkDocs pass. No runtime, public identity, release, recruitment, or
GitHub state changes.
Exact follow-up `da99c7ef82136449727ef97a58c1a2db4ffd6955` removes a
feedback-loop contradiction in the first-map issue form. PASS still requires
one reviewed privacy-bounded receipt; FAIL can now truthfully attest that no
receipt was produced and no file was attached. All three privacy checks remain
required. Thirty-one focused and 38 S6 docs/product regressions plus strict
MkDocs pass; this creates no accepted validation, cohort attempt, upload,
recruitment, or GitHub write.
Exact follow-up `a0aaadc80b952d92074f45499c29a5103a2ad479` makes first-map
reporting redaction-first. The public form and read-only handoff preserve the
command executable, options, and non-private values while requiring literal
`REDACTED` placeholders for credentials, private paths, host or user names,
and precise locations; map geometry remains prohibited. The handoff now gives
four field-by-field completion lines from safe environment hints. Thirty-four
docs and 25 support/installed-contract regressions pass without changing the
v1 schema, uploading evidence, creating an issue, accepting a validation, or
performing a GitHub write.
Exact follow-up `4b1707cdbc2dc41f3d7b52aa8c598841fc925767` fixes the same
contradiction for ordinary bug reports. Session-backed reports still require
one reviewed support ZIP; a pre-session failure instead supplies required
doctor output, an explicit no-session statement, and the first actionable
finding without claiming a nonexistent ZIP review. All four checklist items
remain required. Thirty-two focused and 39 S6 docs/product regressions plus
strict MkDocs pass; this performs no issue, attachment, or GitHub write.
Exact follow-up `51496ca576b668d9e7dc0e7fda39ebdc21b7e1c8` removes a
location-disclosure risk from Autoware issue intake. Required diagnostics now
use a projector summary and redacted command/verifier evidence; precise
latitude, longitude, altitude, MGRS, grid, origin, and private paths become
`REDACTED`. Map bundles, pointcloud/lanelet geometry, bags, raw private logs,
and private-place screenshots are prohibited, with three required privacy
attestations and matching SUPPORT/map-authoring guidance. Thirty-three focused
and 40 S6 docs/product regressions plus strict MkDocs pass; this performs no
attachment, issue, acceptance, or GitHub write.
Exact follow-up `940195b75ea6aff648171b120539a9ab01a0248e` makes benchmark
reporting redaction-first. Public datasets retain canonical identity, source,
license, configuration shape, and key metrics; private or custom bags expose
only redacted sensor/environment/duration metadata. Commands and configuration
replace private values with literal `REDACTED`, and three required attestations
prohibit bags, maps, trajectories, raw logs/data, local paths, complete custom
YAML, and private-site evidence. One reviewed `metrics.json` or public
aggregate report remains optional. Forty-two S6 docs/product regressions and
strict MkDocs pass; no upload, issue, benchmark execution, acceptance, or
GitHub write occurs.
Exact follow-up `6a1dd87e85b966492ebefea84e87a46917885669` makes public bug
evidence path-free by construction. `doctor <bag> --public-json` projects the
private local preflight into a strict type/count/check/profile/finding-code
schema without bag paths, topic/frame names, local commands, raw data/logs, or
free-text messages. Unreadable input returns the same schema with stable
`bag-preflight-input-error`; users still review before sharing. Thirty-one
preflight regressions pass with two dependency skips, plus 12 doctor, 21
option-contract, 42 docs/product, 25 support/installed, 331 broad S6
regressions, and strict MkDocs. No upload, issue, network access, or GitHub
write occurs.
Exact follow-up `71cbf7e776664d40c59157fbfbad4d5611ceae03` makes that safe
handoff visible at the failure point. Every ready or action-required human bag
report keeps the full report local and displays one shell-safe exact-input
`--public-json` command through both the source script and top-level product
wrapper. Thirty-two preflight regressions pass with two dependency skips, plus
42 docs/product, 25 support/installed, 21 option-contract, and 331 broad S6
regressions, with strict MkDocs. No upload, issue, network access, or GitHub
write occurs.
Exact follow-up `387a002dc7826be267fe600db906f80460e6f270` removes the next
own-bag decision burden. A ready report reached through the product CLI keeps
the selected profile and reasons but replaces lower-level scripts and
compatible-path alternatives with one shell-safe exact-input `start`. A report
with findings withholds start and returns to the exact-input `doctor` after the
first finding. Direct preflight and JSON contracts remain detailed and
unchanged. Thirty-two preflight regressions pass with two dependency skips,
plus 42 docs/product, 25 support/installed, 21 option-contract, 331 broad S6,
changed-code `ament_flake8`, and strict MkDocs. No mapping, upload, network
access, issue, or GitHub write occurs.
Exact follow-up `fc87cf86cabba5f55fec47316c6a9a3a4e4cb90f` bounds that
one-action path to a compact product card. A ready card is at most 26 lines and
shows status, duration/count, input types without topic/frame names, selected
profile, check statuses, and one `start`; a finding card keeps only the first
message/action plus remaining stable codes and exact retry. One shell-safe
private `--json` command retains complete detail, and direct preflight retains
the expert report. Thirty-two preflight regressions pass with two dependency
skips, plus 42 docs/product, 25 support/installed, 21 option-contract, 331 broad
S6, changed-code `ament_flake8`, and strict MkDocs. No mapping, upload, network
access, issue, or GitHub write occurs.
Exact follow-up `90c508eef4c6ce6868582bda80e684f14223ea4a` removes the next
own-bag copy-paste detour. Interactive RKO `start` shows calibration once and
directs the operator to the fail-closed confirmation immediately below; it no
longer presents a second `--yes` command in the same process. Non-interactive
`start`, `setup`, and dry-run retain the exact reviewed rerun command. Thirty-five
sensor-setup, 42 docs/product, 25 support/installed, 21 option-contract, and 331
broad S6 regressions pass, with changed-code `ament_flake8` and strict MkDocs.
No real mapping, upload, network access, issue, or GitHub write occurs.
Exact follow-up `2d0bb84a447e29b940adda4bd432e3d5725c9cc0` removes the
remaining confirmed-start repetition. A confirmed live `start` now skips the
full READY setup card and enters the existing start/progress card directly;
setup-only, dry-run, and unconfirmed non-RKO review retain full input,
calibration, and command detail. Thirty-six sensor-setup regressions, exact S3
lifecycle 71 and edit/merge 15, plus 42 docs/product, 25 support/installed, 21
option-contract, and 331 broad S6 regressions pass, with changed-code
`ament_flake8` and strict MkDocs. No real mapping, upload, network access,
issue, or GitHub write occurs.
Exact follow-up `8a620e54a121f5ac45913791b40b5239a59f5885` unifies terminal
success. One VERIFIED or UNVERIFIED card now carries map output, verification,
viewer, session index/page, evidence paths, and exactly one `Next`; viewer
failure makes it the view retry without changing map status, and derived-index
failure keeps one completed fallback card. Thirty-seven sensor-setup
regressions, exact S3 lifecycle 72 and edit/merge 15, plus 42 docs/product, 25
support/installed, 21 option-contract, and 331 broad S6 regressions pass, with
changed-code `ament_flake8` and strict MkDocs. No real mapping, upload, network
access, issue, or GitHub write occurs.
Exact follow-up `14081ea101744b868b80d900bb5a1c42b4ad5046` gives failed maps
one repair action. The default ACTION REQUIRED card shows the first reason,
remaining stable codes, one exact `Next`, and one detail path; all finding
actions, safe retry, inspect alternatives, and evidence paths remain in recovery
JSON and session evidence. Thirty-eight sensor-setup regressions, exact S3
lifecycle 73 and edit/merge 15, plus 42 docs/product, 25 support/installed, 21
option-contract, and 331 broad S6 regressions pass, with changed-code
`ament_flake8` and strict MkDocs. No real mapping, upload, network access,
issue, or GitHub write occurs.
Exact follow-up `e2043c0f324ba8fb855b3a723cb670acd40cb2ad` keeps long stages
visibly active without inventing progress. An unchanged non-complete stage
prints at most one heartbeat every 30 seconds with its existing label and
monotonic elapsed time; only durable stage changes rewrite session artifacts,
and no percentage, ETA, or delegated forward-progress claim is emitted.
Thirty-nine sensor-setup regressions, exact S3 lifecycle 74 and edit/merge 15,
plus 42 docs/product, 25 support/installed, 21 option-contract, and 331 broad S6
regressions pass, with changed-code `ament_flake8`, strict MkDocs, and plan
validation. This is a mocked monitor-boundary result, not a real long mapping
run, clean-host timing result, paired GLIM observation, or parity claim. No
upload, network access, issue, or GitHub write occurs.
Original follow-up `6d1249e…` made the lower boundary interrupt-safe, while a
real ROS trial exposed an outer-dispatcher gap. Correction `0301a0d…` makes the
stable CLI wait for `start` and signals the complete isolated runner group. On
the exact corrected tip, one Ctrl-C after `workflow_running` returned 130 in
1.5 seconds, reaped every descendant, sealed `interrupted / complete`, and
rendered one ACTION REQUIRED / Next / Details with no traceback or success
card. Four schemas and 17 artifact checksums pass, plus 51 CLI/sensor, S3 77,
lower runner 48, docs/product 42, support/installed 25, option 21, and broad S6
332 regressions. The controlled public-fixture trial used a 0.5 GiB floor with
1.58 GiB free; the default storage gate remains unsatisfied. This is not a
complete map, clean-host result, paired GLIM observation, or parity claim. No
upload, network access, issue, or GitHub write occurs.
Follow-up `181b251…` separates expected SIGINT/SIGTERM from genuine timeout
diagnosis. A second exact-tip ROS trial entered the real offline wait and kept
the same 130, descendant-reap, interrupted evidence, and one-action results,
while stderr fell from 9,223 to 1,845 bytes (80.00%) with zero false timeout
labels or launch-log dumps. Fourteen dogfood, 48 lower-runner, 42 docs/product,
25 installed/support, 21 option-contract, and 332 broad S6 regressions pass;
real timeout diagnostics remain unchanged. The same low-storage and controlled-
overlay limits apply, and no remote write occurs.
Exact-installed evidence carrier `edff76d…` then closes the low-storage limit:
the top-level command omitted the override, passed the unchanged 7.01 GiB free
/ 5.00 GiB required preflight, observed live RKO-LIO and graph nodes, and one
Ctrl-C returned 130 in 1.43 seconds with every descendant absent. Four schemas
and all eight artifacts sealed before this earlier interruption revalidate;
the one-action card has no timeout, traceback, success card, or `map.pcd`.
The controlled mixed overlay still leaves clean-host/package-manager, complete-
map quality, paired GLIM, and parity claims pending. No remote write occurs.
Exact implementation `8370ac5…` removes the remaining post-stop duplication
without classifying exit code alone. Only a sealed manifest whose interrupted
status, complete stage, execution/runner codes, and signal error all agree gets
one direct-run stop/evidence line; ordinary 130 and inconsistent evidence keep
full failure diagnostics. In the exact-installed default-storage ROS trial,
one Ctrl-C returned 130 in 1.38 seconds and reaped every descendant. Generic
map failure, repeated failed command, generic session error, false timeout, log
dump, traceback, and success card all fell to zero; stderr fell 1,971 → 1,200
bytes (39.12%). Four schemas, eight artifact checksums, 49 runner, S3 77 + 15,
dogfood 14, docs/product 42, installed/support 25, option 21, and broad S6 332
checks pass. Clean-host and comparison limits remain; no remote write occurs.
Exact implementation and evidence tip `d0e3361…` centralizes all four native
bag-reader opens behind a product-only `rosbag2_storage` WARN boundary. It
changes only the previously-unset logger during open, restores it afterward,
and preserves direct use, explicit expert levels, and every WARN/ERROR. The
direct/product real-bag probe returned 0 with identical 5,661-byte stdout;
direct stderr retained two storage INFO lines while product stderr was empty.
In the exact-installed default-storage ROS stop trial, both live nodes were
observed and one Ctrl-C returned 130 in 1.545 seconds with every descendant
absent. Six routine storage lines fell to zero and stderr fell 1,200 → 180
bytes (85.00%); one stop/evidence pair and ACTION REQUIRED / Next / Details
remain. Four schemas, 18 checksums, preflight 36, runner 49, S3 77 + 15,
dogfood 14, docs/product 42, installed/support 25, option 21, and broad S6 332
checks pass. Clean-host and comparison limits remain; no remote write occurs.
Exact implementation `3dcca0c…` gives guided `start` ownership of the product
story while leaving direct expert `run`, dogfood output, durable evidence, and
warning/error/timeout detail complete. Its child-only environment boundary
hides delegated commands, output/YAML/topic/frame internals, staging promises,
lifecycle duplication, and the launch-log location; the parent retains the
profile, unchanged storage decision, honest progress, and one terminal card.
In a fresh exact-revision non-symlink install, profile/storage appeared before
mapping readiness, both real nodes were observed, and one Ctrl-C returned 130
in 1.543 seconds with every descendant absent. Guided stdout fell 3,679 →
1,448 bytes (60.64%) to 23 lines; stderr is only the stop/evidence pair. Four
schemas, 18 checksums, runner 50, S3 77 + 15, sensor setup 42, dogfood 15,
docs/product 42, installed/support 25, option 21, and broad S6 332 checks pass.
Clean-host and paired-comparison limits remain; no remote write occurs.
Exact implementation `8e67ab7…` closes the successful-completion half of the
same guided UX boundary. An exact-installed 50-second MID360 `start` completed
in 23.45 seconds with `succeeded / complete / 0 / 0`, 7 / 7 receipt PASS, a
4,015,933-byte `map.pcd`, 92 tiles, and 124 checksums. Terminal output fell
51 lines / 3,791 bytes → 23 lines / 1,399 bytes (63.10%); all 16 hidden
post-process lines remain in checksum-bound `map_workflow.log`, while normal
failure replays a bounded tail. Baseline and corrected map, Lanelet2, raw, and
corrected trajectory bytes match. A second real-node Ctrl-C returned 130 in
1.317 seconds, reaped every descendant, and sealed five schemas plus 19
checksums. Runner 54, sensor 42, dogfood 15, S3 77 + 15, S2 43 + 43, and broad
S6 332 checks pass. Clean-host, public-fixture, external-user, and paired-GLIM
limits remain; no remote write occurs.
Exact implementation `cb2218f…` makes the post-success action explicit:
`Report:` / **Prepare a first-map report** replaces user-facing `Share`, while
the structured compatibility key remains unchanged. A fresh exact install
reproduced the 50-second MID360 map in 23.42 seconds with the same map,
Lanelet2, raw, and corrected trajectory hashes. The displayed read-only command
returned a 22-line / 1,279-byte handoff with four schemas and 124 checksums
passing. Human and JSON reads left the complete session path/size/mtime tree
unchanged, created no archive, and made no network syscall under `strace`.
Focused and broad regressions, strict docs, and style pass. This prepares a
reviewed report; it does not upload, create an issue, or count an external map.
Exact implementation `e15ddab…` makes that action a first-class
`lidarslam-map report SESSION` command. The old `support SESSION --first-map`
spelling remains byte-identical, while `report --help` contains no ZIP output
option. A fresh exact install passes the complete product validator; human and
JSON outputs match the legacy command, the 22-line human report is 1,327 bytes,
the fixture tree is unchanged, and `strace` records no network syscall. S3
78 + 15, docs/product 42, support/installed 26, broad S6 333, cohort 33, plan
34, G0 25, strict docs, and style pass. This reduces command discovery and
typing only; it does not map, upload, open a browser, or accept a report.
The dashboard reports one next action and never interprets missing human
measurements, public identity, release, or community evidence as complete.
Its seven slice cards are now executable from an ordinary terminal: ROS state
is sourced explicitly, duplicate package-test basenames stay in separate
pytest processes, caches are disabled, and recognized direct remote-write CLI
forms fail plan validation before a command is displayed.

Local descendant `bce5a9d` additionally passes the real-component #69
unsafe-then-safe recovery sequence ten consecutive times and the full 10-suite
scanmatcher CTest on Humble and Jazzy. This closes a local reliability-evidence
gap; it does not satisfy the public-source, public-CI, fixture, matrix, or
release gates.

The parallel
[contributor Python test entrypoint](evidence/growth/contributor-python-test-entrypoint-2026-08-11.md)
now gives prepared contributors one package-scoped command with dependency
preflight and focused-test forwarding. Final local revision `e2a4dfc` passes
both complete Python product suites on Humble and Jazzy. This improves a
contribution leading signal; it does not increment the external-contributor
metric, and the revision still awaits E1 publication and public CI.

The first execution addendum is the
[2026-08-10 Docker machine-probe summary](evidence/onboarding/docker-machine-probes-2026-08-10.md).
Humble and Jazzy both reached a canonical-route product `PASS`; both remain
measurement `INCOMPLETE` because active operator time, human command count,
and isolated peak disk were deliberately left `null`. This changes measured
matrix coverage from `0 / 4` to `2 / 4`, but comparable coverage remains
`0 / 4`. The weekly GitHub snapshot is not overwritten to insert later trial
observations.

The 2026-08-13 source follow-up adds valid Humble and Jazzy product PASS
records, so the current machine-checked matrix is `4 / 4` present and `0 / 4`
comparable. The source records intentionally omit human active-time
observation, and they are `0.9.1` evidence beside the frozen Docker `0.9.0`
rows; neither condition advances the activation gate.

The collector fails closed if authentication is unavailable, an API response
is malformed, pagination would be incomplete, the local first-map ledger or
anonymous cohort state is invalid, cohort counts/rates/status disagree, or the
v1 readiness audit cannot be trusted. Its output is validated against
[`growth-snapshot-v1.schema.json`](schemas/growth-snapshot-v1.schema.json).

## Metric definitions

| Metric | Exact definition |
| --- | --- |
| Stars, forks, subscribers | Current aggregate fields from the repository API. |
| Views and clones | GitHub's 14-day aggregate totals and unique counts. Unique clones are not interpreted as users. |
| Autoware/TIER IV referrals | Sum of the unique counts for top referrers whose names contain `autoware` or `tier4`. GitHub exposes only its top referrers, and the sum is not a cross-referrer deduplicated user count. |
| Primary-bundle downloads | `download_count` for the single `lidarslam_ros2_v*_release_bundle.tar.gz` asset on the latest stable release. |
| Untriaged open issues | Open issues, excluding pull requests, with no labels. Applying a meaningful label is the minimum triage action. |
| External PRs, 90 days | Pull requests created in the trailing 90 days by non-maintainer, non-bot accounts. |
| External merged contributors, 180 days | Distinct non-maintainer, non-bot authors whose pull request was merged in the trailing 180 days. Only the count is retained. |
| First public response | Time from an external, non-bot issue opening to the first maintainer comment in the trailing 90-day cohort. The scorecard includes eligible, responded, unanswered, and responded-only median values. |
| External first maps | Aggregate accepted/required/remaining counts from the reviewed first-map ledger, plus the anonymous cohort's attempted/terminal/active/review-WIP/successful/accepted counts, completion rate, median active minutes, operational state, and fixed stop conditions. Historical snapshots without the optional cohort extension remain valid and are never rewritten. |
| v1 readiness | Complete/incomplete gate counts from the fail-closed local v1 audit. |

Pass each co-maintainer with `--maintainer LOGIN`; otherwise the default is
`rsasaki0109`. Maintainer logins are used in memory to classify responses and
contributions, but no author list is written.

## Privacy boundary

The collector never writes stargazer identities, issue authors, pull-request
authors, comment authors, comment bodies, issue text, issue/comment/profile
URLs, or raw referrer records. It writes only aggregate counts, durations,
public release metadata, contract identifiers, and short operator-supplied
annotations. Do not put names, private support details, or unpublished
campaign data in an annotation.

The cohort extension copies only bounded aggregate counts, rates, durations,
fixed state names, and fixed stop-condition IDs. It does not copy attempt IDs,
report/blocker URLs, accepted validation IDs, or participant handles into the
weekly snapshot.

The JSON contract fixes both privacy flags to `false`:

```json
{
  "personal_identifiers_written": false,
  "raw_records_written": false
}
```

Raw GitHub responses are held only in process memory. They are not cached by
this script.

## Reproduce without live API access

Use `--fixture-dir DIR` to run the same aggregation and schema validation from
reviewed JSON fixtures. The directory contains:

- `repository.json`
- `traffic-views.json`, `traffic-clones.json`, and
  `traffic-referrers.json`
- `latest-release.json`
- `pulls.json` and `issues.json`
- `issue-comments-N.json` for each external issue `N` in the 90-day response
  cohort, including an empty array when it has no comments

Fixtures are a test and audit interface. Do not commit fixtures copied from the
live API because they contain identities and raw records.

## Weekly review

Capture on the same weekday and approximate UTC time. Compare four-week
rolling values and annotate only real interventions such as releases,
documentation launches, talks, or community posts.

Use the decision rules from the
[1,000-Star roadmap](roadmap/1000-stars.md):

- flat qualified traffic means repair distribution and positioning;
- rising traffic without first-map receipts means stop promotion and repair
  onboarding;
- rising first maps without durable discovery means improve the proof summary
  and post-success invitation;
- rising support load means close diagnosis and documentation gaps before
  opening another channel.

At quarter and phase boundaries, use the
[2026–2029 operating plan](roadmap/1000-stars-2026-2029.md) and its
[health-review template](roadmap/1000-stars-quarterly-review-template.md).
The quarterly review selects one largest funnel constraint, records external
authorization separately from local readiness, and limits the next portfolio
to one product slice, one community slice, and at most one research slice.

The scorecard is evidence for a product decision, not a target to game.
