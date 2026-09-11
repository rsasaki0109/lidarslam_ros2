# 1,000 Stars roadmap — useful, trusted, sustainable

> Status: **active from 2026-08-10**
>
> Last planning audit: **2026-08-17**
>
> Baseline: **837 GitHub Stars**
>
> Latest weekly snapshot: **839 GitHub Stars** at
> `2026-08-16T20:52:07Z`; **161 remain**
>
> Target: **1,000 Stars by 2027-06-30**
>
> Stretch target: **1,000 Stars by 2027-03-31**
>
> Product outcome: a third party reaches a verified Autoware-compatible map
> from a rosbag without reading the implementation or receiving private setup
> help.

This roadmap turns the existing product and research work into a sustainable
adoption loop. It complements the [v0.9 product roadmap](v0.9.md), the
[v1.0 readiness gate](../v1-readiness.md), and the evidence-driven SOTA
research plans. It does not weaken any accuracy, license, compatibility, or
release gate.

The companion
[2026–2029 operating plan](1000-stars-2026-2029.md) extends this milestone
roadmap through compatibility policy, ownership transfer, succession practice,
and a deliberate 2030 continue/narrow/transfer/archive decision. This document
remains the source for the 1,000-Star definition and G0–G5 milestone gates.

The 1,000-Star count is a discovery milestone, not the product's purpose. The
project succeeds only when people can install it, obtain a useful map, verify
the result, recover from failure, and contribute improvements.

## 1. Definition of success

The growth goal is complete only after all of the following are true:

- GitHub reports at least 1,000 current Stars in two weekly snapshots;
- the v1.0 readiness audit reports `10 / 10` complete;
- at least three independent users have accepted first-map validations;
- a current stable release is less than 90 days old;
- at least three non-maintainer contributors have had a contribution merged in
  the preceding 180 days;
- no supported-product P0 issue has waited more than seven days for a public
  disposition;
- every public performance or compatibility claim links to reproducible
  evidence and states its limits.

Crossing 1,000 before these quality conditions is a Star milestone, but not
completion of this roadmap.

### Guardrails

- Do not buy, exchange, automate, or otherwise manufacture Stars.
- Do not mass-message users, forks, issue authors, or unrelated communities.
- Do not make a SOTA, hardware-support, or Autoware-endorsement claim without
  the corresponding evidence.
- Do not add anonymous product telemetry. Use aggregate GitHub metrics and
  voluntary, privacy-bounded validation receipts.
- Do not let research artifacts, generated logs, or benchmark datasets enter
  the supported runtime package or release bundle.
- Do not ship features solely for attention. Every roadmap item must improve
  activation, trust, distribution, or maintainability.

## 2. Baseline on 2026-08-10

GitHub REST traffic data covers the preceding 14-day window. Stargazer dates
describe Stars that still exist today, so they are useful for trend estimates
but are not an exact historical net-growth ledger.

| Signal | Baseline | Meaning |
| --- | ---: | --- |
| Current Stars | 837 | 163 remain |
| Current stargazers added in last 30 days | 11 | recent organic pace |
| Current stargazers added in last 90 days | 35 | about 11.8 per 30.4 days |
| Forks / subscribers | 172 / 12 | strong latent usage, weak ongoing project followership |
| Repository views | 870 total / 299 unique | 14-day discovery baseline |
| Repository clones | 2,042 total / 259 unique | totals include likely automation; use uniques cautiously |
| Main v0.9.0 bundle downloads | 8 | GitHub asset count; GHCR pulls are not included |
| Open issues | 29 | 28 were opened before 2026 and need deliberate triage |
| Open `good first issue` tasks | 1 | the contributor queue is not yet self-sustaining |
| Pull requests | 333 total / 324 merged | high maintainer throughput |
| Historical external PRs | 13 submitted / 10 merged | contribution path exists but is quiet |
| External PRs in the last 90 days | 1 | contributor growth is a primary constraint |
| API-visible contributors | 1 | the project still appears maintainer-dependent |
| Independent first-map validations | 0 / 3 | current v1.0 adoption blocker |
| v1.0 readiness | 8 / 10 | distribution and external adoption remain open |
| Repository size | 172,506 KiB | discovery and clone cost need an artifact/history audit |
| GitHub Discussions | disabled | usage questions remain mixed into the issue backlog |

The strongest 14-day referrers were Google (104 unique visitors) and GitHub
(74). `tier4.github.io` and `autowarefoundation.github.io` contributed only
six unique visitors combined. Search already finds the project; the larger
opportunity is to make the landing page easier to understand and to earn more
qualified Autoware referrals.

The original research worktree remains an operational risk: it is 229 commits
ahead of `origin/develop` and still mixes dirty product, research, and generated
work. Separation has nevertheless advanced. At audited revision `9b3db89`, the
clean G0 product line contained 31 linear commits on exact public
`origin/develop` revision `86fa9b6`, changed 92 paths, and had no merge commit or
dirty file. The 2026-08-11 planning refresh at `33be15b` contains 33 linear
commits and changes 99 paths from the same base, including the complete
fail-closed issue-triage proposal. That line is still local-only. Reviewable
separation is therefore implemented, but source visibility, pull-request
review, fixture hosting, and publication authorization remain open gates rather
than assumed outcomes.

At the current 90-day pace, another 163 Stars would take roughly 14 months,
placing the unassisted projection around October–November 2027. The base target
requires about 15.2 net new Stars per month, approximately 1.3 times the recent
pace. The March stretch target requires about 21 per month.

## 3. Positioning: win a narrower and more useful job

The project should not present itself as a generic replacement for every LiDAR
SLAM framework. FAST-LIO owns a widely cited efficient-LIO position; GLIM owns
a versatile, extensible mapping-framework position. Their current GitHub
counts are useful context, not goals to imitate.

| Project | Public position | Stars on 2026-08-10 |
| --- | --- | ---: |
| `lidar_slam_ros2` | rosbag2-to-verified Autoware map authoring | 837 |
| `koide3/glim` | versatile and extensible 3D mapping framework | 1,727 |
| `hku-mars/FAST_LIO` | efficient and robust LiDAR-inertial odometry | 5,043 |

The product wedge is:

> **The shortest trustworthy path from a rosbag2 recording to a verified
> Autoware-compatible map bundle.**

The three primary audiences are:

1. Autoware mapping teams that need a loadable point-cloud map and projector
   metadata, not only a trajectory;
2. ROS 2 field-robotics integrators who need a diagnosable offline workflow for
   their own sensor data;
3. researchers and students who need reproducible baselines and evidence
   contracts without turning research controls into product defaults.

Advanced coloured mapping, tunnel/fog resilience, and SOTA research remain
valuable proof and extension tracks. They belong below the first successful
map in the information hierarchy and must not obscure the primary job.

## 4. User-friendly means measured, including against GLIM

GLIM already offers maintained documentation, [PPA binary installation](https://koide3.github.io/glim/installation.html),
Docker images, broad sensor support, a direct rosbag executor, visual examples,
and interactive correction. We should not declare victory from a longer feature
list. Once per stable release, run a clean-machine usability scorecard for both
public workflows on equivalent supported hardware.

Measure these tasks independently:

| Task | Measurement |
| --- | --- |
| Discover the supported path | time until the user identifies the correct install/run command |
| Run a fixed demo | commands, download bytes, wall time, active operator time, failure count |
| Inspect an own bag | whether topics, frames, timestamps, and profile choice are explained before execution |
| Produce a downstream artifact | whether the result is a verified Autoware bundle without manual file assembly |
| Understand a failure | whether the public error links directly to one safe recovery action |
| Repeat or upgrade | whether the same command and output contract survive a supported release upgrade |

The six rows now have a versioned
[neutral usability scorecard contract](../usability-scorecard.md). It requires
exact public product identities, one paired clean environment, the same input
per task, task-specific complete measurements, transcript hashes, and an
external first-attempt operator before reporting `READY`. It never infers an
overall winner. The checked-in index is intentionally `NOT_READY` with both
product records absent; local feature evidence is not substituted for a paired
GLIM trial. The paired recorder now removes the remaining JSON-editing step:
one command follows product order, derives command totals and outcomes from
direct observations, preserves blanks as `not-recorded`, validates both
records, and atomically publishes neither or both. It does not create the
still-missing external observation. Pair preparation now also replaces its two
manual public-identity Booleans with one optional GET-only preflight. Both
products' fixed canonical GitHub commit/tag or registry digest and approved
documentation host must resolve before either worksheet can be written; any
identity, redirect, or second-file publication failure leaves no half pair.
Offline preparation remains explicitly `NOT_RUN` and non-public. This proves
only public input identity, not a completed task or comparative result. The
preflight receipt now persists beside the worksheets and SHA-binds their exact
bytes. The recorder requires that receipt, preserves the untouched source
triplet under `preparation/`, and the final checker refuses explicit records
without revalidating that archive. This prevents a later JSON rewrite or
record-only handoff from becoming a CLI `READY` result while still inventing
no observation. The
[2026-08-12 local contract evidence](../evidence/growth/glim-usability-scorecard-contract-2026-08-12.md)
records the enforced protocol and the still-missing public trials.

Only compare overlapping tasks, publish the exact versions and commands, and
record where the products intentionally solve different jobs. The desired
outcome is not “more user-friendly” as an unsupported slogan. It is:

- one obvious beginner path;
- an installed no-argument terminal home that reduces the first decision to
  installation check, demo, own bag, or retained sessions without changing
  automation behavior;
- one bag-optional, read-only doctor that reports stable recovery actions
  before a user commits time or data to mapping;
- no undocumented manual step before `map_verify: PASS`;
- a measured first-run completion rate of at least 80% by the tenth independent
  attempt;
- median active operator time below ten minutes for the fixed demo;
- every own-bag rejection includes a stable reason code and next action.

## 5. Growth model and leading indicators

Stars are a lagging signal. The working model is:

```text
qualified discovery
  -> first verified map
  -> confidence in evidence
  -> useful public report or contribution
  -> recommendation and Stars
```

The operational north-star metric is **independent verified first maps per
month**. It connects product utility to sustainable discovery more directly
than page views or repository activity.

### Scorecard

| Dimension | Baseline | 2026-11 target | 2027-06 target |
| --- | ---: | ---: | ---: |
| GitHub Stars | 837 | 900 | 1,000 |
| 14-day unique repository visitors | 299 | 375 | 500 |
| 14-day unique Autoware/TIER IV referrals | 6 | 15 | 30 |
| Primary release-bundle downloads per release | 8 | 20 | 50 |
| Accepted independent first maps | 0 | 3 | 10 cumulative |
| External merged contributors, trailing 180 days | 0 | 2 | 5 |
| External PRs, trailing 90 days | 1 | 3 | 6 |
| Untriaged open issues | 16 | 0 | 0 |
| Median first public response to new supported-product issues | not tracked | <= 5 days | <= 72 hours |
| v1.0 readiness | 8 / 10 | 10 / 10 | maintained at 10 / 10 |

The response-time values are internal operating targets, not a contractual
support SLA. If maintainer capacity cannot sustain them, reduce concurrent
work and document the actual support boundary instead of hiding the delay.

### Measurement rules

- Capture Stars, forks, subscribers, unique views/clones, referrers, release
  downloads, issue age, and contributor counts once each week.
- Keep compact weekly aggregates; do not commit user identities or a raw list
  of stargazers.
- Review four-week rolling values because release days and automated clones
  make daily totals noisy.
- Annotate releases, documentation launches, talks, and community posts so an
  increase can be associated with a real intervention.
- Never interpret clone totals as users; prefer unique clones, verified maps,
  and release artifacts.
- Re-estimate the target date monthly. Do not weaken quality gates to preserve
  a calendar forecast.

## 6. Workstreams

### A. First-map activation

- Reduce the README's first screen to one promise, one visual proof, one Docker
  command, and one own-bag command; move advanced evidence into clear links.
- Exact local follow-up `8a8876a2d26c09cc92ad330b99d1fa217db1bd8d`
  turns the first Quickstart decision into three bounded goals: stable Docker
  demo, read-only own-bag diagnosis before `start`, or candidate source dry-run.
  Docker is the explicit default when unsure, while each row states its host
  write, version, ROS, storage, and time boundary. The README remains below its
  220-line budget; 29 entrypoint and 36 integrated docs/product regressions plus
  strict MkDocs pass. This is a discovery repair, not a release, measurement,
  promotion, or GLIM parity claim.
- Exact follow-up `1e56431161d3dea417d5d800bb1eedc3cdb51907`
  removes the remaining canonical-entry drift: Getting Started no longer asks
  a first-time visitor to choose among 12 mixed beginner and advanced actions.
  It shows the same three bounded goals as README and retains seven installed
  or post-success actions inside a rendered collapsed section. The Docs Home
  now also calls published v0.9.0 a stable release rather than a release
  candidate. Thirty focused and 37 integrated regressions plus strict MkDocs
  pass; no workflow, release identity, telemetry, recruitment, or GitHub state
  changes.
- Keep `lidarslam-map start BAG` as the canonical own-bag route and package it
  in every supported distribution.
- The current local product candidate adds `lidarslam-map demo [work_dir]` as
  the packaged public-data route over the existing Docker/source
  implementation. Its network- and write-free JSON plan exposes dataset
  identity, attribution, cache/output state, storage, and exact steps; live
  execution re-hashes the pinned archive and extracted bag members, reuses
  only receipt-rebuilt PASS output, and prints map, verifier, receipt, review,
  and diagnosis actions together. Its next local increment adds durable named
  progress, exact failure actions, and `demo --resume` for terminal
  post-processing without ever restarting mapping. This is a product repair,
  not a comparable onboarding row or publication event.
- The next local activation increment reduces the source path to
  `bash scripts/source_quickstart.sh`: it detects the installed Humble/Jazzy
  base, previews all actions without writes, initializes pinned submodules,
  installs missing base tools and repository-only dependencies, builds only the
  six repository packages, and runs the verified demo. The README now places
  the promise, visual proof, Docker command, and own-bag command before the
  feature inventory. A fail-closed disposable-host probe now verifies the exact
  public commit and route, measures disk/network/wall time, preserves private
  logs, and emits the existing bounded trial record. Clean Humble/Jazzy VM rows
  still require an exact published candidate revision before this can close a
  comparable source row.
- The source install no longer ends at a remembered shell-activation step. Its
  printed absolute `lidarslam-map` launcher now activates only the matching
  aggregate workspace environment for its child process. Fresh Jazzy isolated
  and merged installs plus a network-disabled Humble merged install selected
  the maintained RKO-LIO profile and reached the calibration-review dry-run
  with all ROS variables removed; both distributions passed the complete
  clean-prefix checker. The
  [2026-08-12 activation evidence](../evidence/source-launcher-activation-2026-08-12.md)
  keeps package-manager and complete cold-source claims pending.
- The no-install own-bag path now has a one-command Linux host launcher:
  `bash scripts/docker_map_bag.sh BAG`. It validates the bag and fresh output
  before Docker, exposes a Docker/network/write-free plan, mounts input
  read-only, disables external networking for the live run, runs as the host
  UID/GID, and delegates to the same sensor-review, mapping, verification, and
  session-page `lidarslam-map start` contract used after installation. This
  removes the prior eight-line mount recipe and its
  lower-level `run --guided` fork while preserving explicit calibration review.
  Image capability is checked before output creation, the run is rebound to an
  immutable local image ID, and a zero exit cannot claim completion without the
  session, manifest, and receipt artifacts. Existing public v0.9.0 images
  correctly fail this new contract until a candidate image is published.
- The canonical Autoware map-authoring landing page now follows that same
  product contract instead of presenting lower-level shell and Python helpers
  as competing beginner paths. It starts with one published fixed-demo route,
  one source-candidate route, and one own-bag route; then keeps `doctor`,
  `start`, `run`, session evidence, and recovery in task order. A regression
  test rejects the six retired beginner entrypoints on this page, and the page
  explicitly separates published v0.9.0 images from the unpublished v0.9.1
  candidate. The docs-home primary action now points to this canonical page and
  labels the older viewer/dogfood quickstart as advanced compatibility. This is
  a local documentation repair, not a release or onboarding measurement.
- The v0.9.1 release handoff now records its reviewed implementation carrier
  and complete 2,468-test result instead of the stale 2,432-test snapshot. The
  exact Leo Drive follow-up at `92bb524…` records 0.139152 m Applanix
  cross-validation APE, meets the 0.500 m target, and isolates the 2.5 GB
  rosbag2 FILE decompression view from the source dataset. Four exact-head
  blocking `NO_DATA` profiles remain. The maintainer
  checklist runs the complete product suite, strict docs, and canonical fixed
  demo rather than treating the older Autoware dogfood route as first-map
  evidence. Because the notes file becomes the public release body verbatim,
  the tag workflow now stops before image publication when the candidate-only
  banner or `Release decision: HOLD` remains. This makes the current HOLD
  explicit and fail-closed; it does not authorize or perform a release.
- The canonical NTU ground-truth benchmark acquisition now exposes a
  write- and network-free plan before its multi-gigabyte work. It reports each
  remaining phase, pins the official archive byte count and checksum, checks a
  conservative peak working set on the selected filesystem, fails before
  download when space is insufficient, and recommends an explicit external
  `--dest`. Exact attached-storage implementation `8a856f5` now closes the
  remaining path-substitution burden: it discovers the connected unmounted 2 TB
  SanDisk filesystem without probing it, selects one mount action, and emits
  `--dest-device` preflight/live commands that preserve requested phases and
  recheck real free bytes after mounting. The curated bundle now includes both
  the documented NTU entrypoint and its resolver. This makes missing exact-head
  release evidence actionable without weakening the four remaining blocking
  profiles or treating historical runs as current.
- The two blocking RTK-SLAM Construction profiles now have the same actionable
  acquisition boundary. All four official ROS2 inputs use immutable exact
  byte/SHA-256 identities, the checkpoint repository uses a detached pinned
  commit, resumable bytes reduce the capacity plan, and text/JSON dry-runs are
  network- and write-free. A real normal invocation stopped before side effects
  with a root-filesystem shortfall. Exact attached-storage follow-up `0c3f588`
  then found the connected unmounted 2 TB SanDisk partition, selected one
  authorization-respecting mount action, and emitted copy-ready
  `--dest-device` dry-run/live commands that resolve the actual mount path. It
  never mounts, probes contents, or treats partition size as verified free
  space. This removes another preventable user failure; the profiles remain
  `NO_DATA` until fresh exact-candidate runs exist.
- The next release pipeline now turns that repo-independent script into a
  direct attested `lidarslam-map-docker` asset. Its exact tag and source commit
  are embedded deterministically, its default image is the matching immutable
  `v<VERSION>-<distro>` tag, and the read-only publication audit requires this
  seventh asset from v0.9.1 while preserving the historical six-asset v0.9.0
  audit. This closes the local clone-free delivery implementation, not the
  publication or clean-machine trial: no asset, image, tag, or release was
  created by this work.
- The verified-session page now closes the local-success-to-public-learning
  gap with **Share this verified first map**. Its existing-command route,
  `lidarslam-map support SESSION --first-map`, performs no write or network
  request, revalidates receipt-bound PASS evidence, and prints the copy-ready
  summary, reviewed JSON attachment, and canonical issue form. Failed sessions
  receive the privacy-first support ZIP action instead. This reduces the
  independent-validation handoff without claiming any of the still-missing
  0/3 external acceptances.
- Exact follow-up `da99c7ef82136449727ef97a58c1a2db4ffd6955` repairs the
  corresponding failure-report path. The issue form no longer tells a FAIL
  reporter to leave a missing receipt empty and simultaneously requires them
  to claim that they reviewed an attachment. Its required privacy statement
  now accepts exactly one reviewed receipt or FAIL/no-receipt/no-file, while
  PASS still requires the receipt and every privacy checkbox remains required.
  Thirty-one focused and 38 integrated tests plus strict MkDocs pass. This
  enables honest product findings; it creates no validation, recruitment, or
  GitHub write.
- Exact follow-up `a0aaadc80b952d92074f45499c29a5103a2ad479` removes the
  remaining command-sharing ambiguity from first-map reports. The public form
  requests a redacted command shape, keeps executable/options/non-private
  values, requires literal `REDACTED` placeholders for credentials, private
  paths, host or user names, and precise locations, and still prohibits map
  geometry. The read-only handoff now prints four field-by-field completion
  lines using safe environment hints. Thirty-four docs and 25
  support/installed-contract regressions pass without changing the v1 handoff,
  uploading evidence, creating an issue, or accepting a validation.
- Exact follow-up `4b1707cdbc2dc41f3d7b52aa8c598841fc925767`
  applies the same repair to normal bug intake. A startup or preflight failure
  no longer needs an impossible support ZIP or a false attachment-review
  claim: it must provide doctor output, state that no session was created, and
  paste the first actionable finding. Session-backed reports still require one
  reviewed ZIP, diagnostics remain required, and all four privacy/scope checks
  remain mandatory. Thirty-two focused and 39 integrated tests plus strict
  MkDocs pass; no issue, upload, or GitHub write is performed.
- Exact follow-up `51496ca576b668d9e7dc0e7fda39ebdc21b7e1c8` closes a
  location-disclosure risk in Autoware issue intake. The form still requires
  environment, command shape, verifier status, GNSS state, projector summary,
  observed behavior, and all three privacy attestations, but precise projector
  coordinates and private paths must use `REDACTED`. Map bundles,
  pointcloud/lanelet geometry, bags, raw private logs, and private-place
  screenshots are prohibited. SUPPORT and canonical map-authoring guidance
  agree; 33 focused and 40 integrated tests plus strict MkDocs pass. This
  performs no attachment, issue, acceptance, or GitHub write.
- Exact follow-up `940195b75ea6aff648171b120539a9ab01a0248e` makes public
  benchmark intake redaction-first. Public datasets retain canonical identity,
  source, license, configuration shape, and key metrics; a private or custom
  bag exposes only redacted sensor/environment/duration metadata. Commands and
  configuration use literal `REDACTED` placeholders, and three mandatory
  attestations prohibit bags, maps, trajectories, raw logs/data, local paths,
  complete custom YAML, and private-site evidence. One reviewed `metrics.json`
  or public aggregate report remains optional. Forty-two integrated tests plus
  strict MkDocs pass; no upload, issue, benchmark execution, acceptance, or
  GitHub write is performed.
- Exact follow-up `6a1dd87e85b966492ebefea84e87a46917885669` makes public
  bug evidence path-free by construction. `doctor <bag> --public-json`
  projects local preflight v6 into strict type/count/check/profile/finding-code
  evidence without bag paths, topic/frame names, local commands, raw
  data/logs, or free-text messages. Unreadable input returns the same schema
  with stable `bag-preflight-input-error`, while the issue form still requires
  review before sharing. Thirty-one preflight regressions pass with two
  dependency skips, plus 12 doctor, 21 option-contract, 42 docs/product, 25
  support/installed, 331 broad S6 regressions, and strict MkDocs. No upload,
  issue, network access, or GitHub write is performed.
- Exact follow-up `71cbf7e776664d40c59157fbfbad4d5611ceae03` makes the safe
  support route visible at the failure point. Every ready or action-required
  human bag report keeps the full report local and displays one shell-safe
  exact-input `--public-json` command through both the source script and
  top-level product wrapper. Thirty-two preflight regressions pass with two
  dependency skips, plus 42 docs/product, 25 support/installed, 21
  option-contract, and 331 broad S6 regressions, with strict MkDocs. No upload,
  issue, network access, or GitHub write is performed.
- Exact follow-up `387a002dc7826be267fe600db906f80460e6f270` removes the next
  own-bag decision burden. A ready product-dispatched report preserves the
  selected profile and reasons but replaces lower-level scripts and
  compatible-path alternatives with one shell-safe exact-input `start`. A
  report with findings withholds start and returns to the exact-input `doctor`
  after the first finding. Direct preflight and JSON contracts remain detailed
  and unchanged. Thirty-two preflight regressions pass with two dependency
  skips, plus 42 docs/product, 25 support/installed, 21 option-contract, 331
  broad S6, changed-code `ament_flake8`, and strict MkDocs. No mapping, upload,
  network access, issue, or GitHub write is performed.
- Exact follow-up `fc87cf86cabba5f55fec47316c6a9a3a4e4cb90f` bounds that
  one-action path to a compact product card. A ready card is at most 26 lines
  and shows status, duration/count, input types without topic/frame names,
  selected profile, check statuses, and one `start`; a finding card keeps only
  the first message/action plus remaining stable codes and exact retry. One
  shell-safe private `--json` command retains complete detail, while direct
  preflight retains the expert report. Thirty-two preflight regressions pass
  with two dependency skips, plus 42 docs/product, 25 support/installed, 21
  option-contract, 331 broad S6, changed-code `ament_flake8`, and strict
  MkDocs. No mapping, upload, network access, issue, or GitHub write is
  performed.
- Exact follow-up `90c508eef4c6ce6868582bda80e684f14223ea4a` removes the next
  own-bag copy-paste detour. Interactive RKO `start` shows calibration once and
  proceeds to one fail-closed confirmation without presenting a second `--yes`
  command. Non-interactive `start`, `setup`, and dry-run retain the exact
  reviewed rerun command. Thirty-five sensor-setup, 42 docs/product, 25
  support/installed, 21 option-contract, and 331 broad S6 regressions pass,
  with changed-code `ament_flake8` and strict MkDocs. No real mapping, upload,
  network access, issue, or GitHub write is performed.
- Exact follow-up `2d0bb84a447e29b940adda4bd432e3d5725c9cc0` removes the
  remaining confirmed-start repetition. A confirmed live `start` skips the
  full READY setup card and enters the existing start/progress card directly;
  setup-only, dry-run, and unconfirmed non-RKO review retain full input,
  calibration, and command detail. Thirty-six sensor-setup regressions, exact
  S3 lifecycle 71 and edit/merge 15, plus 42 docs/product, 25
  support/installed, 21 option-contract, and 331 broad S6 regressions pass,
  with changed-code `ament_flake8` and strict MkDocs. No real mapping, upload,
  network access, issue, or GitHub write is performed.
- Exact follow-up `8a620e54a121f5ac45913791b40b5239a59f5885` unifies terminal
  success. One VERIFIED or UNVERIFIED card now carries map output,
  verification, viewer, session index/page, evidence paths, and exactly one
  `Next`; viewer failure makes it the view retry without changing map status,
  and derived-index failure keeps one completed fallback card. Thirty-seven
  sensor-setup regressions, exact S3 lifecycle 72 and edit/merge 15, plus 42
  docs/product, 25 support/installed, 21 option-contract, and 331 broad S6
  regressions pass, with changed-code `ament_flake8` and strict MkDocs. No real
  mapping, upload, network access, issue, or GitHub write is performed.
- Exact follow-up `14081ea101744b868b80d900bb5a1c42b4ad5046` gives failed maps
  one repair action. The default ACTION REQUIRED card shows the first reason,
  remaining stable codes, one exact `Next`, and one detail path; all finding
  actions, safe retry, inspect alternatives, and evidence paths remain in
  recovery JSON and session evidence. Thirty-eight sensor-setup regressions,
  exact S3 lifecycle 73 and edit/merge 15, plus 42 docs/product, 25
  support/installed, 21 option-contract, and 331 broad S6 regressions pass,
  with changed-code `ament_flake8` and strict MkDocs. No real mapping, upload,
  network access, issue, or GitHub write is performed.
- Exact follow-up `e2043c0f324ba8fb855b3a723cb670acd40cb2ad` keeps long stages
  visibly active without inventing progress. An unchanged non-complete stage
  prints at most one heartbeat every 30 seconds with its existing label and
  monotonic elapsed time; durable stage changes alone rewrite session
  artifacts, and no percentage, ETA, or delegated forward-progress claim is
  emitted. Thirty-nine sensor-setup regressions, exact S3 lifecycle 74 and
  edit/merge 15, plus 42 docs/product, 25 support/installed, 21 option-contract,
  and 331 broad S6 regressions pass, with changed-code `ament_flake8`, strict
  MkDocs, and plan validation. This is a mocked monitor-boundary result, not a
  real long mapping run, clean-host timing result, paired GLIM observation, or
  parity claim. No upload, network access, issue, or GitHub write is performed.
- Original follow-up `6d1249e…` made the lower boundary interrupt-safe, while a
  real ROS trial exposed an outer-dispatcher gap. Correction `0301a0d…` makes
  the stable CLI wait for `start` and signals the complete isolated runner
  group. On the exact corrected tip, one Ctrl-C after `workflow_running`
  returned 130 in 1.5 seconds, reaped every descendant, sealed `interrupted /
  complete`, and rendered one ACTION REQUIRED / Next / Details with no
  traceback or success card. Four schemas and 17 artifact checksums pass, plus
  51 CLI/sensor, S3 77, lower runner 48, docs/product 42, support/installed 25,
  option 21, and broad S6 332 regressions. The controlled public-fixture trial
  used a 0.5 GiB floor with 1.58 GiB free; the default storage gate remains
  unsatisfied. This is not a complete map, clean-host result, paired GLIM
  observation, or parity claim. No upload, network access, issue, or GitHub
  write is performed.
- Follow-up `181b251…` separates expected SIGINT/SIGTERM from genuine timeout
  diagnosis. A second exact-tip ROS trial entered the real offline wait and kept
  the same 130, descendant-reap, interrupted evidence, and one-action results,
  while stderr fell from 9,223 to 1,845 bytes (80.00%) with zero false timeout
  labels or launch-log dumps. Fourteen dogfood, 48 lower-runner, 42
  docs/product, 25 installed/support, 21 option-contract, and 332 broad S6
  regressions pass; real timeout diagnostics remain unchanged. The same low-
  storage and controlled-overlay limits apply, and no remote write is
  performed.
- Exact-installed evidence carrier `edff76d…` closes that low-storage limit.
  The top-level command omitted the override, passed the unchanged 7.01 GiB
  free / 5.00 GiB required preflight, observed live RKO-LIO and graph nodes,
  and one Ctrl-C returned 130 in 1.43 seconds with every descendant absent.
  Four schemas and all eight artifacts sealed before this earlier interruption
  revalidate; one ACTION REQUIRED / Next / Details remains, with no timeout,
  traceback, success card, or `map.pcd`. The controlled mixed overlay still
  leaves clean-host/package-manager, complete-map quality, paired GLIM, and
  parity claims pending. No remote write is performed.
- Exact implementation `8370ac5…` removes the remaining post-stop duplication
  without classifying exit code alone. Only a sealed manifest whose interrupted
  status, complete stage, execution/runner codes, and signal error all agree
  gets one direct-run stop/evidence line; ordinary 130 and inconsistent
  evidence keep full failure diagnostics. In the exact-installed default-
  storage ROS trial, one Ctrl-C returned 130 in 1.38 seconds and reaped every
  descendant. Generic map failure, repeated failed command, generic session
  error, false timeout, log dump, traceback, and success card all fell to zero;
  stderr fell 1,971 → 1,200 bytes (39.12%). Four schemas, eight checksums, 49
  runner, S3 77 + 15, dogfood 14, docs/product 42, installed/support 25,
  option 21, and broad S6 332 checks pass. Clean-host and comparison limits
  remain; no remote write is performed.
- Exact implementation and evidence tip `d0e3361…` puts all four native bag-
  reader opens behind one product-only `rosbag2_storage` WARN boundary. It
  changes only the previously-unset logger during open, restores it afterward,
  and preserves direct use, explicit expert levels, and every WARN/ERROR. A
  direct/product real-bag probe returned 0 with identical 5,661-byte stdout;
  direct stderr retained two storage INFO lines while product stderr was empty.
  In the exact-installed unchanged-default ROS trial, both live nodes were
  observed and one Ctrl-C returned 130 in 1.545 seconds with every descendant
  absent. Six routine storage lines fell to zero and stderr fell 1,200 → 180
  bytes (85.00%); one stop/evidence pair and ACTION REQUIRED / Next / Details
  remain. Four schemas, 18 checksums, preflight 36, runner 49, S3 77 + 15,
  dogfood 14, docs/product 42, installed/support 25, option 21, and broad S6
  332 checks pass. Clean-host and comparison limits remain; no remote write is
  performed.
- Exact implementation `3dcca0c…` assigns guided `start` one product-owned
  terminal story while preserving complete direct expert `run`, dogfood,
  warning/error/timeout, and durable-evidence behavior. The child-only concise
  boundary removes delegated command, output/YAML/topic/frame, staging,
  lifecycle, and launch-log internals from guided output. A fresh exact-
  revision install showed profile/storage before mapping readiness, observed
  both real nodes, and returned 130 in 1.543 seconds after one Ctrl-C with
  every descendant absent. Guided stdout fell 3,679 → 1,448 bytes (60.64%)
  to 23 lines; stderr retains only the stop/evidence pair. Four schemas, 18
  checksums, runner 50, S3 77 + 15, sensor setup 42, dogfood 15, docs/product
  42, installed/support 25, option 21, and broad S6 332 checks pass. Clean-host
  and paired-comparison limits remain; no remote write is performed.
- Exact implementation `8e67ab7…` closes the successful-completion side of the
  guided UX boundary. An exact-installed 50-second MID360 `start` completed in
  23.45 seconds with `succeeded / complete / 0 / 0`, 7 / 7 receipt PASS, one
  VERIFIED / Next / Share card, a 4,015,933-byte `map.pcd`, 92 tiles, and 124
  checksums. Terminal output fell 51 lines / 3,791 bytes → 23 lines / 1,399
  bytes (63.10%); all 16 hidden post-process lines remain in checksum-bound
  `map_workflow.log`, while normal failure replays a bounded tail. Baseline and
  corrected map, Lanelet2, raw, and corrected trajectory bytes match. A second
  real-node Ctrl-C returned 130 in 1.317 seconds and reaped every descendant.
  Runner 54, sensor 42, dogfood 15, S3 77 + 15, S2 43 + 43, and broad S6 332
  checks pass. Clean-host, public-fixture, external-user, and paired-GLIM
  limits remain; no remote write is performed.
- Exact implementation `cb2218f…` labels the verified post-success action
  `Report:` / **Prepare a first-map report** instead of implying an automatic
  share, while retaining the structured compatibility key. A fresh exact
  install reproduced the 50-second MID360 output in 23.42 seconds with the
  same map, Lanelet2, raw, and corrected trajectory hashes. Its displayed
  read-only command emitted 22 lines / 1,279 bytes; four schemas and 124
  checksums pass, the complete session path/size/mtime tree is unchanged, no
  archive is created, and `strace` records no network syscall. This remains
  local preparation evidence, not an upload, issue, or external first map.
- Exact implementation `e15ddab…` promotes the safe handoff to
  `lidarslam-map report SESSION`. Legacy `support SESSION --first-map` remains
  byte-identical, but new users see no ZIP-only option in `report --help`.
  A fresh exact install passes the complete product validator; its 22-line /
  1,327-byte human output and schema-valid JSON match legacy output, leave the
  fixture tree unchanged, and make no network syscall under `strace`. Focused,
  S3/S6, cohort, plan, G0, strict-doc, and style gates pass. This is a shorter
  local handoff, not mapping, upload, browser opening, issue creation, or
  external acceptance.
- Measure the current 517 MB demo, then provide a smaller onboarding fixture if
  download or run time dominates first success. Keep the full surveyed demo as
  proof rather than silently weakening validation.
- The first [Docker machine probes](../evidence/onboarding/docker-machine-probes-2026-08-10.md)
  measured 19–24 minute cold paths and 1.77–1.91 GB of workflow RX on Humble
  and Jazzy. Both product routes passed; the smaller-fixture experiment is now
  activated, while the full MID-360 route remains the proof path.
- The resulting [50-second MID-360 fixture pilot](../evidence/onboarding/mid360-onboarding-fixture-pilot-2026-08-10.md)
  produced a byte-reproducible 98,873,952-byte ZIP and a verified local map,
  reducing the dataset transfer by 80.879%. It remains unpublished and cannot
  replace the full gate. Its
  [publication review](../evidence/onboarding/mid360-fixture-publication-review-2026-08-11.md)
  now reports 13/13 `LOCAL_ARTIFACT_PASS` checks and a four-item
  `AWAITING_PUBLICATION_DECISION`: two revisions are not publicly resolvable,
  the host is unset, and upload is not authorized.
- The installed
  [public-data acquisition path](../evidence/onboarding/public-dataset-acquisition-hardening-2026-08-11.md)
  now pins size and SHA-256, validates HTTP Range before resume, safely
  restarts Range-ignoring servers, re-hashes cache hits, and transactionally
  extracts preflighted ZIP members. The full 517,088,133-byte source passed a
  read-only identity and member audit. The unpublished fixture has no registry
  URL yet.
- The
  [published-fixture audit gate](../evidence/onboarding/published-fixture-audit-gate-2026-08-11.md)
  now binds an authorized readiness review to GitHub immutable-release metadata
  or a Zenodo version record, then independently re-hashes the downloaded
  artifact. The existing GitHub `v0.9.0` release correctly fails the new
  immutability requirement; no fixture host or upload has been selected.
- The [runtime-image slimming pilot](../evidence/onboarding/runtime-image-slimming-2026-08-11.md)
  reduced Docker's local image-size measurement by 53.6409% on Humble and
  59.6698% on Jazzy while retaining the source-free installed CLI and
  schema-valid first-map result. Exact gzip OCI exports at clean commit
  `ff92f09` then measured 568,999,756 compressed layer bytes on Humble and
  547,456,033 on Jazzy: 53.635066% and 59.674709% below immutable v0.9.0
  baselines, so both pass the 25% gate. Follow-up commits repaired the atomic
  ROS-log link and keyed Docker dependencies on package manifests. Cached
  exports retained identical canonical image graphs. The full public demo,
  clean dedicated-VM trials, and attested release-candidate reproduction
  remain promotion gates; no candidate was pushed.
- Add tested sensor recipes for the most requested families, starting from
  issue evidence rather than an unbounded compatibility matrix.
- Make the final success output show the map path, verifier result, viewer path,
  run receipt, and exact next command in one screen.
- The current candidate now projects that handoff from `session.json`, including
  the headless `--viewer none` path and a read-only `Share:` command for verified
  sessions. This is a local product-contract increment; clean-machine timing
  and external first-map acceptance remain separate gates.
- Retained unverified and action-required sessions now print their compact
  summary before `Next:` in headless history, reducing the return-to-work
  decision without weakening the external measurement gates.

### B. Trust and reproducible proof

- Finish v1.0 only through the existing `10 / 10` fail-closed gate.
- Publish a small human-readable benchmark scorecard whose rows link to exact
  datasets, revisions, commands, limitations, and machine-readable evidence.
- Treat the negative SOTA-v5/v6 outcomes as honest research records. Do not put
  an unresolved SOTA route on the product-release critical path.
- Convert major claims into short visual proof: input, command, output map,
  verifier result, and measured accuracy or runtime.
- Keep comparison language task-specific. Algorithm accuracy, onboarding,
  downstream map authoring, GPU requirements, and sensor breadth are different
  axes.

### C. Distribution and discovery

- Complete the current rosdistro dependency path. As of 2026-08-12,
  `ros/rosdistro` PRs
  [#52949](https://github.com/ros/rosdistro/pull/52949) and
  [#52950](https://github.com/ros/rosdistro/pull/52950) remain open with an
  unanswered question about overlap with the existing `ndt_omp` package.
  Both old exact heads also have one failed stale-base rosdep check; this is
  not caused by the NDT YAML delta, but neither head is green.
  Resolve that collision through upstream convergence or full isolation before
  requesting merge, then refresh or replace the generated registration from
  current rosdistro `master` and require every check to pass. A hash-bound
  upstream patch and complete five-file parent transition now pass exact-base
  checking plus network-isolated Humble/Jazzy four-package builds. The
  read-only Draft publication preflight additionally passes 30/30 checks for
  exact candidate identity, current upstream/fork/branch state, and duplicate
  PR absence. Its schema-bound handoff now fixes the exact create-only branch,
  base/head identities, Draft copy, and GET-only verification route only while
  all 30 checks pass; it grants no push, PR, force-push, ready, or merge
  authority. Publication still requires an explicit maintainer decision.
- Keep the parent v1 live audit aligned with every schema-valid child outcome.
  It now preserves package-manager `SOURCE_REF_MISSING`, `NOT_RUN`, `RUNNING`,
  `FAILED`, `READY`, and `BLOCKED` states in machine and human output. Thus the
  current authenticated tuple `BLOCKED / SOURCE_REF_MISSING / PUBLISHED`
  remains an actionable 8/10 result instead of being misreported as an invalid
  audit. Only the exact `READY` child state can close distribution.
- Exercise clean install and upgrade on Humble and Jazzy after the packages are
  available, then publish v1.0 from the same verified contract.
- Produce one sub-three-minute English demo with captions and one concise
  Japanese companion post. Both point to the same canonical quickstart.
- The local media path now produces a 10.666-second H.264 candidate, four-cue
  English WebVTT, exact-revision Japanese/English copy, and a schema-valid
  SHA-256 manifest from one versioned contract. It rejects the previously
  bundled `v0.2.2` copy, retired commands, local `n/a` metrics, and numerical
  performance claims before rendering. The candidate remains `NOT_PUBLISHED`;
  public posting still follows same-version artifact/docs audits and a
  separate external-action decision.
- Create durable task pages for “ROS 2 LiDAR SLAM quickstart”, “rosbag2 to
  Autoware point-cloud map”, and supported sensor recipes. Avoid duplicate SEO
  pages with no additional operational value.
- Announce only meaningful evidence or releases through the relevant ROS and
  Autoware community channels. One canonical post plus substantive updates is
  preferable to repeated promotion.

### D. Community and contributor loop

- Review all 29 open issues. Label the supported surface, preserve reusable
  answers in docs, close resolved or obsolete reports with a reason, and move
  broad usage discussion only after a suitable public discussion route exists.
- The
  [2026-08-11 read-only community audit](../evidence/growth/community-contributor-backlog-2026-08-11.md)
  groups the complete backlog and defines five copy-ready starter tasks. They
  are `PREPARED_NOT_PUBLISHED`; creating issues or changing labels still needs
  explicit authorization and a current duplicate check.
- The corresponding
  [29/29 issue disposition proposal](../evidence/growth/open-issue-triage-proposal-2026-08-11.md)
  is machine-validated and passes an exact read-only live-drift check. It keeps
  two issues open, requests four current reproductions, and proposes 23 reasoned
  closures, but remains `PROPOSED_NOT_APPLIED`.
- The #69 review card no longer relies on a maintainer remembering two dated
  facts. Its GET-only linked gate requires exact public Draft/CI state and also
  binds latest stable `v0.9.0`/`0df0c4a` as 52 commits behind fix `a2368c4`,
  with no `v0.9.1` tag or release. Any drift suppresses the prepared response;
  this adds no Issue or release authority.
- As of 2026-08-15, the C1 g2o, C2 empty-map, C3 Odometry/TF, and C4 custom
  PointCloud2 documentation gaps are implemented and retired without reusing
  their IDs. The refreshed C5–C9 generation restores five locally `READY`,
  30-minute documentation scopes: the Japanese follow-up handoff, live versus
  optimized pose outputs, mapping/loop/relocalization boundaries, classic IMU
  readiness, and long-route stop triage. A capture-time read-only audit found
  only PR #427 and no matching implementation PR; the queue remains
  `PREPARED_NOT_PUBLISHED` and requires a fresh audit plus a separate community
  publication decision.
- Exact follow-up `e4ce0aa6eb7d53423423a02af65f923472708f44` prevents the
  blocked first-map cohort issue from freezing independent starter preparation.
  A live authenticated GET-only card still keeps #422 unavailable to
  contributors, but after duplicate review it now gives maintainers one C5
  preview action while C6–C9 remain queued. No local task becomes public,
  assigned, or claimable, and every issue/label/write authority remains false.
- Exact follow-up `0a34e724875d53b8ef74acd8a51fd500ce014ff5`
  removes the next manual publication step. The authenticated live card binds
  the selected C5 title, sorted labels, heading-free body, task/queue hashes,
  and body SHA-256 into one schema-valid local handoff. Cross-task or body
  tampering fails closed; maintainer confirmation and a separate external write
  remain required, and issue creation stays unauthorized.
- Exact follow-up `5dc04198549341b33becdeb2bc058117db9fe78f` prevents that
  review-ready handoff from outrunning its public product base. Local task
  publication review now requires both PR #427 merged and the canonical queue
  present with an exact SHA-256 match on public `develop`. While the Draft is
  open and the public queue is absent, the one action is post-merge preparation;
  it cannot recommend issue publication or grant write authority.
- Recruit the first three validators through the existing public validation
  issue and release documentation; do not provide private step-by-step help
  that would invalidate the evidence.
- Maintain five to ten bounded `good first issue` tasks with a fixture, expected
  output, relevant files, and a check command.
- Make the contributor path finish in under 30 minutes for documentation and
  small CLI changes. Provide focused tests instead of requiring every public
  dataset.
- Keep contributor-facing product entrypoints synchronized with the product
  contract. The [2026-08-12 local repair](../evidence/growth/contributor-product-surface-sync-2026-08-12.md)
  replaces a stale three-entrypoint list with the current four workflows and
  regression-checks both the canonical commands and focused test route. Its
  unsourced-shell full gate also auto-selects supported ROS, checks both
  package suites, and enforces that every product pytest remains registered
  with CTest; the current local result is 2,050 passed across the two suites.
- Enable GitHub Discussions only after categories, moderation expectations,
  support boundaries, and a weekly triage slot are ready.
- Invite sustained contributors into review and roadmap discussion before
  expanding merge rights under `GOVERNANCE.md`.

### E. Releaseable architecture and repository hygiene

- Split the 229-commit local delta into product, research, and generated
  evidence inventories. Integrate reviewable product slices from a clean
  `origin/develop` base instead of publishing the whole development branch.
- Keep runtime code, bounded test fixtures, and durable research conclusions in
  Git. Keep bags, build logs, raw replay outputs, generated maps, and repeated
  benchmark artifacts outside the repository or in release evidence storage.
- Audit the 172,506 KiB repository history and release assets before deciding
  whether history migration is justified. Do not rewrite public history merely
  to improve a vanity size number.
- Maintain one product release candidate, one community task, and at most one
  exploratory research candidate in progress at once.
- Keep container verification and publication as separate authority domains.
  Pull requests and manual dispatches build and smoke-test with a contents-read
  token only; moving convenience tags may change only on a `develop` push, and
  immutable matrix-candidate publication uses the separate default-branch
  `repository_dispatch` gate. That gate requires exact-head green CI, a
  maintain/admin repository role, literal E2 digest-only approval, and a
  protected `candidate-images` environment with self-review prevented and
  deployment restricted to `develop`; its package-write job can publish
  untagged digests but cannot create or move a tag. A shared schema-backed
  checker audits the complete live environment inventory and exact policy with
  GET requests only. Its status-specific operator handoff makes the required
  settings URL, reviewer/self-review/develop-only checklist, independent
  review, and read-only recheck copy-ready without performing a write. Gate
  availability is not E2 authorization.
- Budget normal maintainer effort at approximately 60% product/reliability,
  20% distribution/community, and 20% research. Research can use spare capacity
  but cannot delay a failed product, support, or release gate.

### F. Sustainable communication

- Ship a compact monthly “what became easier” update, not a raw commit list.
- Show one before/after workflow, one evidence result, known limitations, and a
  contributor credit in each release note.
- Create case studies only with user permission and privacy-safe artifacts.
- Credit external reports, documentation, fixes, and benchmark submissions,
  not only code.

## 7. Phases and Star checkpoints

Star counts are forecast checkpoints, not phase-exit gates. Quality outcomes
control progression.

| Phase | Window | Quality exit | Star checkpoint |
| --- | --- | --- | ---: |
| G0 — release hygiene | 2026-08-10 to 2026-09-30 | clean product integration path, weekly metrics, all open issues triaged, public v0.9 first-map baseline measured | 860 |
| G1 — v1 activation | 2026-10-01 to 2026-11-30 | v1 audit 10/10, three accepted external first maps, canonical guided path in packaged products | 900 |
| G2 — proof and launch | 2026-12-01 to 2027-02-28 | stable v1 release, public UX/benchmark scorecard, demo video, at least two recent external contributors | 950 |
| G3 — ecosystem | 2027-03-01 to 2027-06-30 | ten cumulative first maps, five recent external contributors, maintained sensor recipes and support targets | 1,000 |
| G4 — sustain | 2027-07-01 to 2027-09-30 | 90-day post-milestone health review, current release, no regression in activation or contributor metrics | maintain >= 1,000 |
| G5 — institutionalize | 2027-10-01 to 2027-12-31 | two repeatable release cycles, one non-maintainer review owner, documented support capacity, and no single private artifact on the release path | maintain >= 1,000 |

If the March stretch target is reached, do not skip G3 quality work. Move
directly into the sustainability review.

### Long-range dependency map

The phases are not independent feature buckets. The durable path to 1,000
Stars has one evidence-gated dependency chain:

```text
smaller runtime image + bounded onboarding fixture
  -> comparable Humble/Jazzy Docker and source trials
  -> public validator cohort + package-manager proof
  -> v1.0 readiness 10 / 10
  -> canonical proof launch
  -> maintained sensor and contributor ecosystem
  -> 1,000-Star milestone + 90-day sustainability audit
  -> repeatable release and review ownership beyond one maintainer
```

Issue triage, weekly aggregate measurement, release maintenance, and bounded
research run beside this chain rather than after it. An external rosdistro
delay must not stop onboarding repair or validator preparation. Conversely, a
research result, traffic spike, or early Star checkpoint cannot bypass a
failed activation, distribution, or adoption gate.

| Horizon | Product and trust | Adoption and community | Distribution and communication | Outcome checkpoint |
| --- | --- | --- | --- | --- |
| 2026 Aug–Sep — G0 | reduce and measure the cold first-map path; preserve full-gate parity | triage the backlog and prepare bounded validator tasks | establish comparable Humble/Jazzy Docker/source evidence | reviewable product line and 860-Star forecast checkpoint |
| 2026 Oct–Nov — G1 | close the 10/10 v1 audit from real package and first-map evidence | accept three independent first maps; keep five to ten bounded contribution tasks | finish package-manager proof and prepare the canonical v1 landing path | verified activation and 900-Star forecast checkpoint |
| 2026 Dec–2027 Feb — G2 | publish a current stable v1 release and reproducible UX/benchmark scorecard | retain at least two recent external contributors and publish reusable user findings | release one short English demo and Japanese companion through relevant channels | proof-led launch and 950-Star base checkpoint; 1,000 stretch only if quality gates already pass |
| 2027 Mar–Jun — G3 | maintain release freshness, claim evidence, and supported sensor recipes | reach ten cumulative first maps and five recent external contributors | grow durable Autoware/ROS task pages and qualified referrals | 1,000 Stars in two weekly snapshots, with all completion conditions still passing |
| 2027 Jul–Sep — G4 | prevent activation, reliability, and upgrade regressions | verify that support and review load remain maintainable | publish a 90-day health review instead of immediately expanding scope | sustain at least 1,000 Stars and a healthy project for 90 days |
| 2027 Oct–Dec — G5 | rehearse two releases from public inputs and documented gates | transfer ownership of one review area and keep the starter queue healthy | publish only maintained proof and a year-end health report | remain useful and releaseable after the milestone campaign ends |

Every six weeks, select the largest measured constraint in the discovery-to-
first-map-to-contribution funnel. Start no more than one product/release slice,
one community slice, and one bounded research slice at once. If the calendar
and a quality gate conflict, revise the forecast rather than the gate.

## 8. First 90 days

### Sprint 1 — separate what can safely ship (weeks 1–2)

1. Inventory the 229 unpublished commits and dirty files into product,
   research, evidence, and generated-output groups.
2. Start a clean product integration line from `origin/develop` and bring over
   only the guided CLI, product contract, packaging, and directly related tests
   in reviewable slices.
3. keep `log-*`, raw bags, maps, and replay evidence outside the source tree;
   add narrow ignore rules only after confirming every target is generated.
4. Run strict docs, supported package tests, release-bundle rehearsal, and the
   fixed Docker first-map path from the exact candidate revision.

Exit: a clean, reviewable candidate exists; no research result or generated
artifact is required to install or use it.

### Sprint 2 — measure and repair first success (weeks 3–4)

1. Run the public v0.9 Docker and source paths on clean Humble and Jazzy
   machines.
2. Record download size, wall time, active operator time, commands, peak disk,
   failure reason, verifier result, and receipt status with the
   [comparable onboarding trial contract](../onboarding-trials.md).
3. Test `--guided` with representative valid and invalid bag metadata.
4. Fix the largest observed activation blocker before changing the landing
   page or creating promotional material.

Exit: the project has a reproducible onboarding baseline and no undocumented
manual step on the fixed demo.

### Sprint 3 — close the two v1 blockers (weeks 5–8)

1. With explicit publication approval, submit the prepared required NDT APIs
   upstream as a Draft only after the strict 30/30 read-only preflight reports
   `READY_FOR_DRAFT_PR` and emits the exact create-only handoff; abort if its
   branch/base/candidate identity drifts. GET-only verify the resulting Draft,
   replace the reply packet's URL placeholder with that verified PR, answer
   the two open `ndt_omp_ros2` rosdistro reviews with the collision analysis,
   and rerun the review/check-aware readiness audit after every external state
   change. If upstream declines, fully isolate the fork.
   Generate the selected replacement from current rosdistro `master` and
   require all checks to pass; do not merge overlapping Debian payloads or a
   red exact head merely to shorten installation.
2. Execute package-manager clean-install and upgrade evidence when the required
   repositories contain the packages.
3. Run a public first-map validation cohort. Treat every failed attempt as a
   product finding, resolve it publicly, and rerun without live guidance.
4. Require the complete v1 audit before tagging v1.0.

Exit: distribution is reproducible and three independent users have reached a
verified map, or the exact remaining external blocker is publicly recorded.

### Sprint 4 — publish proof, not promises (weeks 9–12)

1. Restructure the README around the measured first-map route.
2. Publish the short demo, UX scorecard, benchmark summary, limitations, and
   v1 release evidence from one canonical landing page.
3. Prepare five bounded contributor tasks from onboarding findings and the old
   issue backlog.
4. Announce the release through a small number of relevant channels and review
   qualified traffic, validation, support load, and Stars after two and four
   weeks.

Exit: a new visitor can understand the job, see proof, try it, report a result,
and find a contribution without maintainer guidance.

## 9. Decision rules

- If four-week Star growth is below 12 and qualified traffic is also flat,
  improve distribution and the clarity of the product wedge.
- If traffic rises but first-map receipts do not, stop promotion and repair
  onboarding.
- If first maps rise but Stars do not, improve the proof summary and the
  post-success invitation; do not add unrelated features.
- If support load exceeds the response target for two weeks, reduce launch
  activity and close documentation or diagnosis gaps before enabling another
  channel.
- If external contributors cannot run focused checks, invest in fixtures and
  development setup before creating more `good first issue` labels.
- If an exploratory SOTA route fails its preregistered gate, retain the honest
  result, run only the authorized bounded diagnostic, and remove the route from
  the product critical path.
- If a Star checkpoint is missed while activation and contribution improve,
  keep the quality plan and revise the forecast. If Stars rise while those
  signals decline, treat the growth as non-durable.

## 10. Immediate next development decision

The next project-wide sprint is **G0 release hygiene and product integration**,
not another broad algorithm expansion. Its first local deliverable, the P1
reliability fix for issue #69, now passes through `bce5a9d`: all five classic
scanmatcher VoxelGrid stages fail closed on unsafe integer layouts, 11 boundary
tests pass, valid-cloud PCL output is unchanged, and one real component rejects
the unsafe frame before publishing a map then publishes map and pose for a
later safe frame. The component test passed ten consecutive runs and the full
10-suite scanmatcher CTest passed on both Humble/PCL 1.12 and Jazzy/PCL 1.14. The
[evidence record](../evidence/voxel-grid-overflow-safety-2026-08-11.md) keeps
the public issue open until a reviewed public revision, supported public CI,
and carrying release exist. The S1 review now additionally commits the movement
baseline only after a map update succeeds: an unsafe asynchronous update cannot
consume the threshold needed by the next safe scan, and worker exceptions are
contained without terminating the component. The current v44
research route remains limited to its already-authorized bounded failure-profile
work and cannot block this sprint.

The immediate public-operations dependency is now explicit rather than a
repeated audit loop. When local and public Draft heads differ, the G0 dashboard
checks their local ancestry and emits one exact, schema-bound non-force branch
handoff. It binds the canonical GitHub repository URL, PR branch, public head,
local tip, separate exact-tip authority, and post-update GET-only audit while
keeping push, force-push, and writes false and printing no push command. Missing
history is fetched only from the canonical repository URL, never from an
arbitrary checkout-specific `origin`. This makes the next maintainer decision
copy-ready without turning a local plan into publication authority.
The branch action now also carries a canonical PR-description refresh because
the public body can drift independently from the branch. Its exact body and
SHA-256 are generated from one clean desired head and the current whole-PR,
P2, v1, onboarding, and independent-validation counts. Updating that body is a
separate exact-tip decision after branch verification; the checker prints no
edit command, requires Draft state to remain, and grants no review, mark-ready,
merge, release, or outreach authority. Even an exact green head is held before
review when the observed body digest is stale.
The body now makes the large Draft navigable without another lookup: validated
P0–P2 start/end commits become three exact GitHub compare links, and validated
S1–S7 metadata becomes a compact focus/path/check/publication-gate table.
Contiguous
lineage, whole-PR commit composition, P2 path composition, safe Markdown
labels, and no-write authority are checked before rendering. The desired-tip
compare link is intentionally useful only after the ordered branch update.
Exact follow-up `4404f877263157d09ae6c451dae55f5ddbbd03af` also turns the
public-product transition into one user-facing read. The
`--include-public-transition` alias evaluates Draft → protected environment →
published release together, while its schema-bound handoff distinguishes audit
required, external publication required, blocked audit, and a genuinely
published identity ready for a fresh matrix packet. It rejects unsafe or
mismatched version identity and forbids reuse of old mixed-version measurements.
The live observation exposes the actual order: fast-forward Draft update review
first, then the absent `candidate-images` environment, then the absent v0.9.1
tag/Release/images. It performs and authorizes no external action.

The
[2026-08-10 G0 activation decision](../evidence/growth/g0-activation-decision-2026-08-10.md)
selects the missing clean Docker/source onboarding baseline as the first
measured blocker. Broad promotion remains behind the four-row trial and repair
gate.

The immediate bounded G0 multi-stage runtime-image pilot has passed both its
local Docker-size proxy and exact compressed-layer gate on both supported
distributions. Its
[evidence record](../evidence/onboarding/runtime-image-slimming-2026-08-11.md)
keeps the image and fixture reductions separate. Promotion is still paused
until the full public demo and clean comparable-VM evidence pass from the
reviewed candidate; the final attested release build must reproduce the
compressed gate. The atomic ROS-log link and Docker dependency-cache boundary
have passed exact-revision follow-ups. The 50-second fixture has also passed a
non-publishing local review. The source route is now public at exact commit
`549ef03017c776f23fc968881b346aa685356274`; the active implementation is the
dedicated Humble/Jazzy Docker/source matrix, including confirmation of both
repairs, not a public image retag.

The source side of that matrix now has a local disposable-host executor with a
read-only plan, public-identity preflight, fixed 250 ms disk sampling, isolated
RX measurement, timeout cleanup, receipt verification, and privacy-bounded
record generation. The immutable public-identity prerequisite now passes at
`549ef030`; the source route has since produced valid Humble and Jazzy PASS
records, but the evidence gate remains open because those runtime-image trials
did not observe human active time or a cold dependency-install baseline. They
also belong to product `0.9.1`, while the frozen Docker rows remain `0.9.0`;
the matrix exposes that cross-version evidence but refuses to compare it.

That executor now separates public route readiness from VM execution. A
network-read-only `--public-preflight` requires the exact six maintained source
packages, explicit package selection, repository-only dependency helper,
tests-disabled build, canonical beginner page, and matching `VERSION` from the
same immutable commit. The quickstart independently checks `colcon list` before
rosdep/build and fails inventory drift with a stable code. The former base
`3f4dd70` correctly reported `NOT_READY`; exact public candidate `549ef030` now
reports `READY` for `0.9.1` without writes. The observer never substitutes a
private checkout.

The local technical prerequisite beneath those source rows now passes on both
distributions. Fixed, network-disabled Humble and Jazzy images built all six
maintained packages from the read-only candidate in 302 and 273 seconds,
respectively; the source/install package inventories match, the absolute
fresh-terminal real-bag route and complete installed contract pass, and the
prefix remains free of Python cache writes. The run also found and closed a
package-content leak that copied development `__pycache__` plus a runtime
bytecode-write path. This is deliberately recorded as an all-source overlay
proof, not a comparable onboarding row: system dependencies were already in
the images and the run lacked disposable-host timing. The
[evidence record](../evidence/source-all-packages-install-2026-08-12.md)
keeps those limits explicit.

The installed CLI now also has a local, TTY-only no-argument home for the common
intents: installation check, fixed demo, own rosbag2, and retained sessions. It
delegates to the existing versioned commands, prints the exact command first,
requires an explicit `yes` before demo writes, and leaves non-interactive
no-argument behavior at exit `2`. This directly reduces command-discovery time
in the GLIM usability scorecard without inventing another mapping workflow;
clean-machine timing still belongs to the four-row onboarding trial.
The [2026-08-12 evidence record](../evidence/interactive-home-2026-08-12.md)
binds its source, installed, TTY, safety, documentation, and regression checks.

The local GLIM-parity system-doctor increment now extends `doctor` without
duplicating bag preflight. With no bag it verifies the curated product surface,
matching install, supported ROS environment, bag reader, and demo storage,
returning path-free JSON and stable recovery actions without network or writes.
With a bag it retains the existing input/profile inspection. The
[2026-08-12 system-doctor evidence](../evidence/growth/glim-parity-system-doctor-2026-08-12.md)
keeps the real PPA/package-manager advantage and public clean-machine
comparison pending.

Exact implementation `402c23765fe125a2f42d7fd245d2a1c972a1ab34` moves the
existing Odometry/TF support card into that same bag doctor. A bounded scan now
checks one deterministic
Odometry topic against all recorded TF topics, accepts a dynamic multi-hop
path, and separates invalid frames, no path, static-only evidence, and reader
failure. Findings stay visible beside a compatible maintained mapping path and
do not pretend to prove live freshness, timing, calibration, or accuracy. The
[bounded evidence record](../evidence/growth/odometry-tf-bag-preflight-2026-08-17.md)
keeps those claim and mutation limits explicit; no GitHub issue was changed.

The follow-up preflight v6 increment addresses the still-open Issue #64 timing
burden without changing scan matching. For the selected PointCloud2 topic it
replays recorded TF availability in bag order, reports clouds seen before all
required dynamic edges, and measures every positive future-TF gap against the
limiting path edge. It deliberately does not silence warnings, raise a timeout,
or substitute stale TF. The
[timing evidence record](../evidence/growth/odometry-tf-timing-preflight-2026-08-17.md)
keeps live scheduling, DDS, clock, buffer-history, interpolation, and map
quality claims open for runtime validation.

A read-only 2026-08-16 issue review found the same old file-edit handoff in
[#95](https://github.com/rsasaki0109/lidar_slam_ros2/issues/95),
[#98](https://github.com/rsasaki0109/lidar_slam_ros2/issues/98),
[#103](https://github.com/rsasaki0109/lidar_slam_ros2/issues/103),
[#106](https://github.com/rsasaki0109/lidar_slam_ros2/issues/106),
[#111](https://github.com/rsasaki0109/lidar_slam_ros2/issues/111), and
[#115](https://github.com/rsasaki0109/lidar_slam_ros2/issues/115): people using
Ouster, Velodyne, RoboSense, simulated, or other PointCloud2 inputs were asked
which launch/YAML files to fork before the product could inspect their bag.
Exact implementation `6950764154dfe0a2159f701d8d01cd55ce5907af` replaces that
discovery burden with one bounded message across the TTY home, system doctor,
README, canonical map-authoring page, Japanese quickstart, and release notes:
run `doctor`, then `start`; do not guess remaps, frames, transforms, or tracked
file edits. Unsafe inputs still stop before mapping with a stable reason code,
and PointCloud2 detection is explicitly not a vendor-support or accuracy
claim. The complete maintained Python gate passes 2,469 tests with 13 known
skips, strict MkDocs and changed-file Jazzy `ament_flake8` pass, and the
implementation-carrier candidate bundle remains 261 files.

A second read-only 2026-08-16 issue review found a recurring burden after a run
does finish: users describe maps that rotate, spiral, oscillate, drift, stop
early, look sparse, or fail to appear in
[#89](https://github.com/rsasaki0109/lidar_slam_ros2/issues/89),
[#92](https://github.com/rsasaki0109/lidar_slam_ros2/issues/92),
[#93](https://github.com/rsasaki0109/lidar_slam_ros2/issues/93),
[#94](https://github.com/rsasaki0109/lidar_slam_ros2/issues/94),
[#96](https://github.com/rsasaki0109/lidar_slam_ros2/issues/96),
[#100](https://github.com/rsasaki0109/lidar_slam_ros2/issues/100),
[#101](https://github.com/rsasaki0109/lidar_slam_ros2/issues/101),
[#104](https://github.com/rsasaki0109/lidar_slam_ros2/issues/104),
[#105](https://github.com/rsasaki0109/lidar_slam_ros2/issues/105), and
[#124](https://github.com/rsasaki0109/lidar_slam_ros2/issues/124). Exact
implementation `ee453532a70d2d4b82a6c50c65f19b22d76c239f` extends the existing
retained-run `inspect` command with five bounded, user-reported symptom codes.
It orders sensor, timestamp, calibration, TF, runtime, map-save, and viewer
checks and returns shell-safe product commands without editing parameters,
starting mapping, uploading evidence, or claiming an automatic root cause or
accuracy result. Exact validation carrier
`9f8a2058a3c702f69d159079568ced8433ee3377` passes 2,474 maintained Python
tests with 13 known skips, strict MkDocs, changed-file Jazzy `ament_flake8`,
and a byte-reproducible 261-file candidate bundle. This closes one local
support-navigation gap; it does not prove that any reported map was repaired.

The next issue-driven handoff check found that this bounded symptom was lost
again when an operator ran `support`: maintainers received generic diagnosis
state but not the fixed visual symptom that prompted the report. Exact
implementation `0d102e016717d2def3db3a99525755837461f759` carries only the
five-code enum and `USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED` into the
privacy-bounded JSON and issue body. Symptom titles, checks, commands, free
text, paths, maps, bags, and logs remain excluded; malformed codes, mismatches,
and automatic-cause claims become invalid diagnosis evidence. The focused 56
tests, strict MkDocs, changed-file Jazzy `ament_flake8`, and complete 2,482-test
maintained gate pass. This reduces issue clarification round trips without
claiming that the symptom is a diagnosed cause or that a map was repaired.

The next activation repair closes the largest locally reproduced fixed-demo
rejection gap: low storage no longer leaves the operator to calculate a
shortage, replace `<dir>`, or reconstruct prior options. Both system doctor and
demo plans now expose exact `additional_bytes_required`; human output rounds
up, system JSON remains path-free, and demo recovery retains the complete
shell-quoted command. The 8 GiB safety floor is unchanged. This improves the
local path but does not replace the still-pending public paired observation.

An unconfigured-terminal audit then exposed a smaller but real decision gap in
that same surface: five valid blockers produced five equal-looking recovery
instructions. Exact implementation
`a83bbfeaea8196a19513c7a26772d500fe8419b8` keeps every stable finding and its
machine recovery, but adds one schema-bound top-level `next_action` and one
human **Do this now** card selected in dependency order. Rerunning doctor
reprioritizes the remaining blockers. The exact observed source-checkout run
selected `source-build-required`, retained four follow-up checks, and performed
no network access or write. This is a reduction in first-decision burden, not a
new doctor surface or comparative usability result.

This local UX increment is complete in the reviewed candidate; the remaining
GLIM-parity work is measurement, not another overlapping doctor surface. The
next action is a neutral paired scorecard on exact public identities, using the
six fixed tasks and an external first-attempt operator after the product
release/image gate opens. Until then, local feature checks must not be written
as a comparative usability claim. The pair command can now GET-verify those
exact identities and documentation atomically before handoff, so the external
operator no longer has to make a manual public-resolvability assertion. The
same handoff now carries its content-bound preparation archive through
recording and final validation rather than relying on maintainer memory.

The existing GLIM comparison harness now also fails closed on reference-cache
reuse. Its key binds bag/config/runtime/options and both cache implementations,
while a schema-backed manifest binds the validated TUM bytes. Legacy
path/topic-only files, changed inputs, and contradictory artifacts are misses.
This removes a stale-reference integrity risk in the current measurement
surface; it does not add a new doctor, substitute a fresh GLIM run, or satisfy
the still-pending paired usability scorecard.

The beginner entry now also has a short
[Japanese quickstart](../getting-started-ja.md) linked from the README and Docs
home. It keeps only the canonical doctor, Docker demo, source helper, own-bag,
success, and recovery decisions; a regression binds those commands and safety
boundaries to the release bundle. The page states that PPA/package-manager
installation remains unavailable instead of translating a future promise into
a current path. The
[2026-08-12 local evidence](../evidence/growth/japanese-quickstart-2026-08-12.md)
keeps publication and Japanese first-user timing pending.

The matrix decision is now machine-checked rather than inferred from a table.
The no-argument checker now loads an explicit schema-backed evidence index
instead of falsely reporting zero rows when paths are omitted. Current tracked
evidence reports `BLOCKED`: all four outcomes are present and all four are
product PASS, but zero rows are comparable because human measurements are
still missing, the historical rows do not retain their exact privacy-bounded
first-map validation receipts, and the Docker/source product versions are not
aligned. G0 cannot advance the activation gate until the rows share one
product version and at least one clean Docker row plus one clean source row
has both complete measurements and a SHA-bound, schema-valid retained receipt.
The
stricter target remains all four rows comparable. A source row is not valid
until its exact revision is publicly resolvable, and a shortened-fixture row
must bind the published ZIP checksum rather than a private local copy.

The parallel G0 community slice is also bounded. The live read-only audit found
one current `good first issue` and prepared five additional tasks from the old
support backlog. The task bodies, 30-minute estimates, files, checks, and
non-goals are recorded locally; no GitHub issue or label has been changed.
Source publication and community publication are separate decisions.

Those five tasks now have a schema-backed local queue and a contributor-facing
inspector. Its fixed profiles reject arbitrary commands, verify exact allowed
paths and labels, mark a task stale when its known implementation gap changes,
and retain `PREPARED_NOT_PUBLISHED` plus no-write authority in machine output.
A 2026-08-12 read-only duplicate audit found one open PR (#427) and zero
task-matching open PRs. The 2026-08-15 drift refresh now correctly retires
C1–C4 after their cards were implemented. A second 2026-08-15 read-only audit
then replenishes the ordered generation as C5–C9, with all five exact gap
markers absent and no matching implementation PR. Publication still requires
a fresh duplicate check and a separate maintainer decision; local readiness
does not mean an issue exists, is assigned, or is advertised.

The live queue no longer lets the closed #422 cohort gate suppress unrelated
maintainer preparation. Exact follow-up `e4ce0aa…` keeps the contributor action
at `WAIT_FOR_READY_PUBLISHED_STARTER`, retains #422 and its six blockers, and
selects one duplicate-free local C5 preview for the maintainer. Potential pull
request duplicates still take precedence over every publication preview, and
the preview performs no issue or label mutation. This keeps the community
slice moving without recruiting anyone into unpublished product instructions.
The following `0a34e72…` handoff also prevents a maintainer from manually
recombining a live duplicate decision with stale task bytes: title, labels,
body, and three digests now travel together. It provides no GitHub mutation
command and cannot convert local review readiness into publication authority.
Follow-up `5dc0419…` additionally binds that handoff to the public product
base. Its GET-only gate distinguishes an open/unmerged Draft, a closed-unmerged
PR, a missing queue, queue drift, and the sole `READY` state: merged PR #427
plus a canonical queue match on public `develop`. Current live state is
`WAITING_FOR_PRODUCT_MERGE` with public queue `ABSENT`, so C5 remains a
post-merge preparation artifact rather than a publishable issue.

The
[clean-candidate audit](../evidence/growth/g0-clean-candidate-audit-2026-08-11.md)
and
[external-action packet](../evidence/growth/g0-external-action-decision-packet-2026-08-11.md)
led to public Draft PR `#427`. Exact public route baseline `0c67387` contains
the complete reviewed source path and has a network-read-only `READY` result.
G0 remains `HOLD`: the matrix is 4/4 present and 0/4 comparable, v1 is 8/10,
and the existing v0.9.0 tag prevents version reuse.

The candidate-session command now also has one read-only `--check-readiness`
mode instead of another overlapping doctor surface. The same exact run URL,
row, destination, filesystem, human-measurement mode, and isolation
acknowledgement produce stable host findings plus one shell-safe next command.
It distinguishes a runnable but non-comparable row from a blocked host before
any download, Docker build, APT/source mutation, evidence write, or trial. This
reduces scorecard command discovery and setup failures; it does not replace the
still-missing neutral paired GLIM trial or comparable G0 evidence.

The historical 207-path follow-up from the frozen comparison base has since
been expanded into the current 349-path exact
[dependency-ordered publication slice plan](../evidence/growth/g0-publication-slice-plan-2026-08-12.md).
Its checker gives every path one review owner across runtime safety, first-map
foundation, map lifecycle, source onboarding, distribution, product-shell
integration, and publication control. The plan itself remains local-authority
only even though its reviewed carrier is public; further PR updates require a
clean exact candidate and complete gates.

The later
[whole-PR coverage audit](../evidence/growth/g0-pr-review-coverage-2026-08-17.md)
closes a review-boundary gap between the original 116-path clean-candidate
audit and that follow-up plan. The machine gate now composes the initial audit,
the exact two-commit / 11-path CI bridge, and all seven follow-up slices into
the final 396-path PR inventory. Missing phase paths, changed fixed SHA/digest,
an unapproved bridge path, non-linear history, or an unsafe review record fails
closed before the G0 dashboard can advance. This makes a very large Draft
reviewable without treating a green local plan as push, merge, release, or
community authority.

The same checker now renders one bounded `--overview` card before reviewers
drill into the seven exact `--slice` cards. It shows the three contiguous
commit ranges, 396-path union, overlap/missing/extra results, slice sizes,
dependencies, verification counts, and publication gates without pasting the
complete 349-path follow-up inventory into the PR summary. Human and JSON
forms stay local-only, execute no displayed check, and cannot submit a review
or authorize a push, ready transition, or merge. This targets the current
review bottleneck—an otherwise verified but very large Draft—without adding
another product surface.

The overview and slice cards now derive a review budget from the same exact Git
ranges: textual additions/deletions, binary paths, and up to three largest
textual deltas. The seven slice budgets must compose exactly to the follow-up
budget, every numstat inventory must match its path digest, and malformed or
stale Git output fails closed. Exact slice cards name binary media rather than
hiding it behind a total. These numbers tell a human where to begin; they are
not a risk score, submitted review, correctness claim, or merge authority.

The G0 dashboard now audits that prerequisite directly through bounded GitHub
GETs. It binds local HEAD to Draft PR #427, the canonical branches, mergeable
state, and the latest exact-head check runs. Draft review and the separate
merge decision are selected before any `candidate-images` administration;
green CI never grants merge, environment, E2, E3, or E4 authority.
The shared read-only API layer reuses an explicit token or the existing
`gh auth` login without exposing credentials, and attaches authorization only
to exact `https://api.github.com` GET requests. Wrong hosts, schemes, ports,
userinfo, and write methods fail closed, reducing anonymous-rate-limit false
blockers without adding remote-write authority.
When that exact Draft is green, the dashboard now carries a schema-bound review
handoff: exact head, 396-path / three-phase / seven-slice coverage, overview,
slice template, and a fixed overview → P0/P1/P2 → S1–S7 sequence. It refuses
the handoff when the worktree is dirty and selects read-only status inspection
instead. It also requires the public PR body to match the canonical clean-tip
description digest; stale scope text gets a separately authorized, exact-body,
keep-Draft refresh handoff before review. Neither path executes checks, edits a
PR, submits a review, marks ready, merges, or performs a write.
That exact body also carries three clickable phase diffs and seven bounded
focus/path/check/gate rows, all rebuilt from the validated local overview
rather than copied from prose. A disconnected range, stale count, injected
link label, dirty tip,
or write-authority claim fails before reviewer navigation is emitted.
The same packet now turns seven technical slices into four capability lanes:
runtime safety, operator UX, distribution, and integration/publication. Two
reviewers is an advisory capacity target, never a merge gate; the contract
stores no personal identity and grants no reviewer-request, review, ready, or
merge authority. This gives a potential contributor a bounded way to choose a
useful review area without inventing or collecting a maintainer roster.
An append-only local ledger now closes the next evidence gap. It records only
lane, slice, exact in-scope path, bounded finding category/detail, and a human-
reported PASS or BLOCKED outcome. It stores no reviewer identity or timestamp,
must live outside the repository to avoid changing its own reviewed SHA, keeps
superseded blocker events as history, and refuses stale dependencies. Local
completion remains evidence for a later decision, not a submitted GitHub
review or ready/merge authority.
The seven review cards also carry self-contained, cache-free verification:
ROS-dependent checks source Humble/Jazzy explicitly, package test roots remain
in separate pytest processes, and recognized direct remote-write CLI forms fail
plan validation. Exact follow-up `0633c2a604489538e0f087c02385e7c6467540c3`
extends that boundary to the S6 docs/product-CLI command after exact execution
from an unsourced shell exposed two `rosbag2_py` import failures; the repaired
displayed command now passes all 42 tests and removing its prelude fails
validation.
Evidence carrier `72a8c9e77eba33c1578a3cd9c8afe8fbe6933e33`
passes 2,488 maintained Python tests and a byte-identical 261-file candidate
bundle rehearsal without granting publication authority.
The later RTK-SLAM attached-storage synchronization passes 2,506 maintained
Python tests and retains the exact 314-path plan. Clean evidence carrier
`3a38154e` also passes the canonical two-build rehearsal: both reverified
264-file bundles are byte-identical, with a retained archive size of
11,948,788 bytes and SHA-256
`46b20d1eecaf6ab665c17816db42e937e9cb50c88a8dc4340beda4f2e670cf27`.
That result replaces the older bundle as local evidence, not as a published or
reusable exact-final-head release artifact.
The later NTU attached-storage increment expands the current plan to 317 paths,
adds a shared read-only resolver and the formerly omitted documented NTU helper
to the curated bundle, and reports the exact 49,209,878,965-byte requirement
and root shortfall before any mount, write, or download. Its own exact-head
validation carrier `b01b251` passes 2,517 maintained Python tests and a
byte-identical, twice-reverified 267-file bundle rehearsal: 11,959,011 bytes,
SHA-256
`f963391bf76f67e27828bf0c8eadada484ac8b8b9481e5f904fcec72e1c64bad`.
That evidence is regenerated rather than inferred from the earlier RTK carrier
and grants no mount or publication authority.

The latest local slice also binds the rendered Getting Started bytes, route
fragments, product version, and Pages workflow to one exact source revision.
Its read-only audit keeps the independent cohort closed while the deployed
manifest is absent or stale; a plausible URL alone is no longer launch
evidence.

The release-mode observer packet now derives the release commit and both ROS
distribution image digests from one schema-valid published-release audit
rather than four manually typed values. Its generated Docker preflight repeats
the live tag-commit and digest comparison. This contract is locally validated
at exact tip `289f7675a242b00f342528483cde3e5f602a11fc`. The v0.9.0 identity passes that
check but lacks the source quickstart contract, while v0.9.1 remains
unpublished, so neither state creates a comparable same-version row.

After G0, the next priority is the independent first-map cohort. It is both the
remaining v1 evidence and the fastest honest way to discover whether the
project is genuinely easier for a new user than its alternatives.

The public tracking issue does not bypass that sequence. The live contributor
card binds #422 to the cohort evaluator and keeps it out of the recommended
starter set until the state is exactly `READY_FOR_NEXT_ATTEMPT`. In the current
`WAITING_FOR_PUBLIC_GATES` state, the next work is comparable Docker/source
evidence plus canonical public-documentation provenance and immutable runtime
identity—not recruiting a user into stale deployed instructions. The checked
JSON card is a no-write decision aid; issue edits, Pages deployment, and
community outreach remain separate authority gates.

## 11. Long-term operating cadence

The roadmap is operated as a recurring system, not a one-time launch list.
Each review ends with one explicit constraint, one owner, one evidence artifact,
and one next review date.

| Cadence | Required review | Durable output |
| --- | --- | --- |
| Weekly | capture aggregate growth; label or disposition new issues; inspect support load and blocked reviews | one privacy-bounded growth snapshot and a short decision annotation |
| Every two weeks | inspect one clean first-map attempt or onboarding finding; review the ready contributor queue | one accepted result, one repaired blocker, or one explicit no-change decision |
| Monthly | update the Star forecast, release age, v1 gate, external contribution count, and “what became easier” summary | one compact public progress note after the underlying evidence is public |
| Every six weeks | select the largest measured discovery-to-first-map-to-contribution constraint | one bounded experiment with success and stop criteria |
| Quarterly | rerun supported clean-install/upgrade checks; audit claims, dependencies, licenses, stale issues, and maintainer load | one health review with the next quarter's WIP allocation |
| At 1,000 Stars | start the 90-day sustain audit instead of opening a larger feature campaign | G4 health report followed by the G5 ownership review |

### Capacity and WIP policy

Normal maintainer capacity remains approximately 60% product and reliability,
20% distribution and community, and 20% bounded research. The split is reviewed
quarterly, but the following limits do not change merely because capacity rises:

- at most one product/release slice, one community slice, and one exploratory
  research slice may be active at once;
- at most two starter contributions may wait for substantive maintainer review;
- a supported-product P0, a failed release gate, or a growing support queue
  takes capacity from research first;
- promotional work stops when onboarding or review capacity is failing;
- no stable release may depend on a private bag, unpublished image, local-only
  revision, or maintainer-memory-only procedure.

The project reduces scope before reducing evidence. A smaller maintained sensor
matrix, fewer simultaneous experiments, or a later date is preferable to an
unreviewable release and a stalled contributor queue.

## 12. Star runway and checkpoint budget

The baseline has 163 Stars remaining. These scenarios are forecasts, not quotas
and not permission to manufacture engagement.

| Scenario | Net pace | Approximate crossing date | Operating response |
| --- | ---: | --- | --- |
| Recent organic pace | 11.8 per month | October–November 2027 | keep quality work; improve qualified distribution only after activation passes |
| Base plan | 15.2 per month | by 2027-06-30 | deliver G0–G3 in order and review the forecast monthly |
| Stretch plan | about 21 per month | by 2027-03-31 | accept only if v1, first-map, contributor, and release gates are already healthy |

The phase checkpoints divide the base-plan delta into visible planning units:

| Phase | Start to checkpoint | Net change | Leading outcome that must improve first |
| --- | ---: | ---: | --- |
| G0 | 837 to 860 | +23 | clean public candidate, comparable onboarding evidence, triaged support surface |
| G1 | 860 to 900 | +40 | v1 audit 10/10 and three independent verified first maps |
| G2 | 900 to 950 | +50 | current stable release, proof-led launch, and recent external contributors |
| G3 | 950 to 1,000 | +50 | maintained recipes, ten cumulative first maps, and durable qualified referrals |

Missing a Star checkpoint does not invalidate a phase whose activation and
community outcomes are improving. It triggers a new forecast. Hitting a Star
checkpoint does not advance a phase whose quality exit is incomplete.

## 13. Constraint experiments

Only one experiment in each funnel stage may be promoted at a time. Every
experiment needs a pre-intervention baseline, a bounded cost, and a stop rule.

| Funnel stage | First long-term experiment | Success signal | Stop or redirect signal |
| --- | --- | --- | --- |
| Discovery | one canonical rosbag2-to-Autoware landing page and proof-led README | qualified Autoware/ROS referrals and release-bundle downloads rise over four weeks | traffic stays flat after indexing and one relevant announcement; revisit positioning rather than duplicate pages |
| Activation | published 50-second fixture beside the unchanged full proof route | comparable first-map trials improve active time and completion without weakening verification | users still fail before execution or fixture/public-source identity is not reproducible |
| Trust | public UX and benchmark scorecard with exact revisions and limitations | external first-map reports and evidence-linked citations increase | claims need private context or scorecard rows cannot be independently reproduced |
| Contribution | five bounded starter tasks and a focused-check contributor path | at least three non-maintainer completions with median prepared-environment time at most 30 minutes | two consecutive tasks exceed 45 minutes because of setup or review bottlenecks |
| Referral | one short English demo, one Japanese companion, and consented case studies | qualified referrals, validations, and Stars rise together | views rise while receipts and useful reports stay flat; stop promotion and repair activation |

An experiment is not repeated merely because a social post underperformed.
Repeat only after the underlying product, audience, or distribution condition
has materially changed.

## 14. Risk register and post-milestone ownership

| Risk | Early signal | Mitigation and owner outcome |
| --- | --- | --- |
| Local-only source or fixture blocks reproducibility | a trial cannot name a public immutable revision or checksum | keep G0 closed; obtain separate source and artifact publication decisions |
| Maintainer concentration | releases, reviews, and support stop when one person is unavailable | by G5, transfer one bounded review area and rehearse two checklist-driven release cycles |
| Old support backlog hides current product defects | unlabeled or unanswered supported-path issues remain open | complete reasoned triage, convert reusable answers to docs, and track response aggregates weekly |
| External rosdistro timing slips | package-manager gate remains unchanged across reviews | continue source/Docker activation work; never pretend source proof is binary-package proof |
| Promotion outruns support | traffic rises while receipts, response time, or review throughput worsen | pause announcements and fix diagnosis, docs, or maintainer capacity |
| Research consumes the release path | an exploratory candidate becomes a prerequisite for onboarding | enforce the 20% budget and remove failed routes from the product critical path |
| Unsupported claims damage trust | a comparison omits versions, limits, or reproducible evidence | fail the release/communication review and publish a correction before further promotion |
| Privacy-bearing evidence leaks | a proposed receipt contains maps, paths, identities, or exact locations | reject the artifact and retain only the reviewed aggregate or privacy-bounded receipt |

After 1,000 Stars, growth work changes from acquisition to retention and
ownership. G4 proves that activation, release freshness, response load, and
contribution do not regress for 90 days. G5 then proves that two release cycles
and at least one review domain can operate from public artifacts and written
contracts without relying on one maintainer's private state. Only after those
audits should the project consider a broader sensor matrix, a larger governance
surface, or a new headline research claim.

The [2026–2029 operating plan](1000-stars-2026-2029.md) continues this policy
through G6–G9: versioned compatibility, evidence-backed ecosystem depth,
release/triage resilience, and a 2029 strategic renewal decision. Its
[quarterly template](1000-stars-quarterly-review-template.md) turns the horizon
into a recurring evidence and ownership review.

## 15. Baseline reproduction

The 2026-08-10 snapshot was taken from the authenticated GitHub REST API and
the local readiness audit. Maintainers can refresh the same aggregate sources
without collecting private product telemetry:

```bash
python3 scripts/collect_growth_snapshot.py \
  --output docs/evidence/growth/$(date -u +%F).json

gh api repos/rsasaki0109/lidar_slam_ros2
gh api repos/rsasaki0109/lidar_slam_ros2/traffic/views
gh api repos/rsasaki0109/lidar_slam_ros2/traffic/clones
gh api repos/rsasaki0109/lidar_slam_ros2/traffic/popular/referrers
gh api 'repos/rsasaki0109/lidar_slam_ros2/releases?per_page=20'
python3 scripts/check_v1_readiness.py --json
```

Repository and pull-request aggregates were cross-checked against
`origin/develop` and the GitHub pulls endpoint. The collector now encodes the
weekly aggregation, privacy boundary, and JSON schema; the individual endpoint
commands remain only as a maintainer audit aid.
