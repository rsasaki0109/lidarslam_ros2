# Community contributor backlog — 2026-08-11

> Status: **PREPARED_NOT_PUBLISHED**
>
> Repository snapshot: **837 Stars, 29 open issues, 0 open pull requests**
>
> Publication authority: **not granted**

This is the bounded community work queue for the G0 phase of the
[1,000 Stars roadmap](../../roadmap/1000-stars.md). It converts recurring
support demand into five tasks that a new contributor should be able to finish
in 30 minutes or less. It is not a record of created GitHub issues: no issue,
label, assignment, comment, or repository setting was changed during this
read-only audit.

The snapshot was taken from the public GitHub repository on 2026-08-11. It
retains only aggregate counts and public issue numbers. Author identities and
comment bodies are not copied into this evidence record.

On 2026-08-12, when the live aggregate had moved to 30 open issues, this prose
backlog was promoted to the machine-readable
`docs/contracts/contributor-starter-queue-v1.json` contract. A read-only
GitHub connector audit found one open pull request, #427, and zero matching
open pull requests for each of C1–C5. The local checker validates exact file
scope, a fixed command allowlist, 30-minute estimates, known-gap drift, and the
no-write boundary. It can render copy-ready task bodies, but it does not create
issues, labels, comments, branches, or pull requests. The duplicate audit must
be rerun immediately before any future publication.

```bash
python3 scripts/contributor_starter_queue.py --json
python3 scripts/contributor_starter_queue.py --next
python3 scripts/contributor_starter_queue.py --list
python3 scripts/contributor_starter_queue.py --task starter-C5
```

## Operationalization validation — refreshed 2026-08-16

The checked-in queue now reports `QUEUE_READY_LOCAL_ONLY` for a refreshed
five-task generation, C5–C9. C5 remains the unimplemented Japanese handoff
successor. C6–C9 are four evidence-backed English support cards selected from
the current public backlog after the completed C1–C4 cards were retired. The
retired IDs are not reused, and their canonical markers remain in the docs.

A fresh authenticated GitHub API GET audit at
`2026-08-16T07:21:20+09:00` found one open pull request, #427, and no matching
open pull request for any C5–C9 query. That result is capture-time evidence,
not publication authority, and must be rerun immediately before any issue is
created. The `--next` card repeats bounded GET-only issue and PR reads, directs
contributors only to a live published issue whose declared product dependency
is ready, and gives maintainers one local review command. It rechecks new or
subsequently updated PRs while honoring reviewed non-matches that have not
changed since the recorded audit. An open state and starter label alone are
not readiness evidence.
The default command and list/detail views remain offline and perform no
workspace write. Focused verification uses built-in profiles rather than
executing commands supplied by JSON; docs output goes to a temporary directory
and Python cache writes are disabled.

| Check | Result |
| --- | --- |
| queue/schema/drift/authority/live-next-action regressions | 61 passed |
| English support-card regressions | 22 docs entrypoint tests passed, including C1/C2/C3 contracts |
| C5–C9 queue generation | all five exact gap markers absent; `QUEUE_READY_LOCAL_ONLY` |
| C5–C9 focused profiles | five strict-MkDocs profiles passed; no workspace artifact |
| C1 g2o recovery | implemented and retired by the previous generation; absent from C5–C9 |
| C2 empty-map recovery | canonical card detected and retired by the previous generation; absent from C5–C9 |
| C3 Odometry-versus-TF recovery | canonical card detected and retired by the previous generation; absent from C5–C9 |
| C4 custom-sensor checklist | implemented and retired by the previous generation; absent from C5–C9 |
| contributor runner safety lint | selected fatal flake8 rules passed for runner and test |
| complete maintained Python gate | graph 1,442 passed / 13 skipped; lidar_slam 998 passed; 2,440 total |
| documentation | strict MkDocs build passed with pre-existing notices |
| authority | no issue, label, comment, branch, PR, or other remote mutation |

The C1–C4 rows above are completion history, not current queue output. The
current C5–C9 generation keeps those retired tasks out of its ordered task set.
These checks prove that the queue is bounded and usable by maintainers. They do
not prove a 30-minute external completion and do not authorize publication.
That evidence begins only after a separately approved issue is claimed and a
non-maintainer reports prepared-environment timing.

## Published starter dependency audit — 2026-08-16

The only published `good first issue`,
[#422](https://github.com/rsasaki0109/lidar_slam_ros2/issues/422), is an
independent first-map cohort task rather than an always-open documentation
starter. Its exact public identity is now declared in the queue contract with
the fixed read-only check
`python3 scripts/first_map_validator_cohort.py --json`. Only
`READY_FOR_NEXT_ATTEMPT` makes that issue eligible for the contributor card.

The current live GET-only report is
`PUBLISHED_GOOD_FIRST_ISSUES_BLOCKED`: 29 open issues after excluding the one
open PR, one published `good first issue`, zero eligible published starters,
and one blocked starter. The cohort remains `WAITING_FOR_PUBLIC_GATES` on the
comparable Docker/source rows, canonical documentation path/URL/provenance,
and immutable runtime identity. PR #427 was updated after the stored duplicate
audit and matches the broad C5 query, so it is also retained as one potential
duplicate for human review before any C5 publication.

A read-only public-page check found that the deployed
[Getting Started](https://rsasaki0109.github.io/lidar_slam_ros2/getting-started.html),
[Golden Path CLI](https://rsasaki0109.github.io/lidar_slam_ros2/golden-path-cli.html),
and
[independent-validation](https://rsasaki0109.github.io/lidar_slam_ros2/external-first-map-validation.html)
pages do not yet expose the current reviewed local route consistently. The
deployed beginner page still uses a mutable Docker tag and older source/demo
steps, while the CLI and validation pages still expose the older `run` flow;
the validation page also renders one malformed command prefix. The expected
deployment manifest remains unavailable. These are launch blockers, not
permission to edit #422 or deploy Pages from this packet.

The machine-readable `--next --json` result is validated against
[`contributor-next-action-v1`](../../schemas/contributor-next-action-v1.schema.json).
It retains all published starter identities for audit, separates ready and
blocked lists, emits one safe contributor action and one maintainer action,
and preserves `GET_ONLY` / no-write authority. No issue, label, comment, PR,
Pages deployment, or community post was changed by this audit.

## What the backlog says

Only issue
[#422](https://github.com/rsasaki0109/lidar_slam_ros2/issues/422) currently has
the `good first issue` label. Sixteen of the 29 open issues have no label, and
28 were opened before 2026. The old backlog is therefore a stronger source of
beginner work than speculative feature expansion.

The 29 open issues fall into these mutually exclusive planning groups. The
grouping is an operating aid, not a final disposition of any issue.

| Demand group | Count | Public issue numbers |
| --- | ---: | --- |
| First-map validation and community | 1 | #422 |
| Install and dependency setup | 3 | #108, #110, #122 |
| TF, input, and missing output | 6 | #64, #93, #102, #103, #106, #112 |
| Sensor and robot onboarding | 9 | #83, #89, #95, #96, #98, #100, #105, #111, #115 |
| Mapping quality, reliability, and tuning | 7 | #30, #53, #69, #92, #94, #104, #124 |
| Advanced capability or algorithm scope | 3 | #101, #116, #118 |

The first publication batch should target setup, diagnosis, and documentation.
Relocalization, loop-closure redesign, and broad algorithm work are not starter
tasks because they cannot be bounded honestly to one fixture and one focused
check.

## Starter-task contract

Every published starter issue must contain all of the following:

- one operator-visible outcome;
- an estimate of at most 30 minutes for a prepared contributor environment;
- exact files that may change;
- explicit non-goals;
- acceptance criteria that can be checked without private data or hardware;
- one focused command that normally completes in under five minutes;
- `good first issue`, `help wanted`, and one domain label;
- a maintainer confirmation that no open pull request already implements it.

The estimate begins after the repository and documented development
dependencies are available. Review latency, ROS installation, and first-time
tool downloads are reported separately rather than hidden inside the estimate.

## Candidate C1 — current g2o setup card

Suggested issue title:

> Docs: explain the supported g2o package path and EOL boundary

Why this task exists:

- issues #108 and #122 report `libg2o` resolution or API-version failures;
- issue #110 includes a Humble container installation path;
- the current docs state that Humble and Jazzy packages resolve `libg2o`, but
  the beginner page does not turn the historical errors into one recovery card.

Scope:

- update `docs/getting-started.md` and, only if needed,
  `docs/rosdistro-release.md`;
- distinguish a missing rosdep key from an incompatible source-built g2o;
- show `rosdep resolve libg2o` and the supported binary-package check;
- state that Foxy is outside the maintained product contract;
- recommend a pinned supported package path, not an unpinned source clone.

Acceptance:

- a reader can identify whether the failure is dependency resolution or a C++
  API mismatch;
- Humble and Jazzy are the only maintained distributions claimed;
- the card links to the product support boundary;
- no hardware support or build-success claim is added without evidence.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **20 minutes**. Non-goals: changing CMake, vendoring g2o, or reviving
an EOL ROS distribution.

### C1 local completion — supported g2o recovery

The beginner source-build page now separates a missing `libg2o` rosdep
resolution from an incompatible manual source build. It shows the maintained
Humble/Jazzy environment check, `rosdep resolve libg2o`, the matching binary
package policy/install checks, and the canonical source quickstart recovery.
Foxy and Galactic are explicitly outside the maintained product contract, and
the card rejects CMake patching, vendoring, blind shared-install deletion, and
arbitrary build-success claims.

The canonical C1 marker is now present, so the queue reports C1 as `STALE` and
must not render it as an unclaimed issue. No historical support issue was
changed by this local completion.

## Candidate C2 — no-map three-check card

Suggested issue title:

> Docs: add a three-check recovery card for an empty `/map` or viewer

Why this task exists:

- issues #93, #102, #103, and #106 all describe a missing map or blank viewer;
- the beginner page currently jumps from the symptom to the output directory
  without separating input, frame, runtime, and viewer failures.

Scope:

- extend the `Common First-Run Problems` section in
  `docs/getting-started.md`;
- provide one check each for a live `PointCloud2` input, a non-empty sampled
  `frame_id`, and a connected TF path;
- distinguish “no map messages were produced” from “a map exists but the
  viewer fixed frame or selected topic is wrong”;
- route own-bag users back to `lidarslam-map doctor` and the generated diagnosis.

Acceptance:

- each command includes the expected observation and one next action;
- placeholder topic and frame names are visibly marked for replacement;
- the card never asks a user to upload a map, bag, or location-bearing log;
- the fixed public demo remains the first control experiment.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **25 minutes**. Non-goals: changing ROS nodes, promising support for
an untested sensor, or diagnosing an individual historical bag.

### C2 local completion — empty-map and viewer recovery

The beginner page already contains the full control-demo, live PointCloud2,
sampled non-empty frame, directed TF, map-topic, viewer-frame/topic, local
diagnosis, and no-private-upload path under
`### Empty map or viewer: three-check recovery`. The queue had looked for an
older planned heading and therefore exposed a false READY task. Its probe now
matches the canonical heading and reports C2 as `STALE`.

## Candidate C3 — Odometry message versus TF

Suggested issue title:

> Docs: explain why an Odometry topic does not guarantee `odom -> base_link` TF

Why this task exists:

- issue #112 has an Odometry message whose frame names exist in the message but
  not in the TF tree;
- issue #64 reports future-extrapolation warnings after TF frequency lag;
- these are different failures and should lead to different next actions.

Scope:

- add a short card to `docs/workflows.md`;
- explain that a `nav_msgs/msg/Odometry` publisher does not necessarily
  broadcast the corresponding transform;
- show how to check the Odometry header, `odom -> base_link` TF availability,
  and transform freshness;
- separate a missing transform from an extrapolation/timestamp problem.

Acceptance:

- the card contains no copied third-party broadcaster implementation;
- frame direction and placeholder names are explicit;
- both failures end in a safe, testable next action;
- the text does not recommend suppressing TF warnings as a fix.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **25 minutes**. Non-goals: supplying a robot-specific broadcaster,
changing scanmatcher timing, or tuning a controller.

### C3 local completion — Odometry and TF separation

The workflow guide already checks the Odometry parent/child fields, the
directed `odom -> base` TF path, and transform freshness separately under
`### Odometry and TF: two separate contracts`. It distinguishes a missing
broadcaster from stale or future timestamps and does not recommend suppressing
warnings. The queue's old planned-heading probe has been aligned to this
canonical card, so C3 now reports `STALE` instead of inviting duplicate work.

## Candidate C4 — custom PointCloud2 sensor checklist

Suggested issue title:

> Docs: add the minimal checklist for adapting another PointCloud2 LiDAR

Why this task exists:

- issues #83, #89, #95, #96, #98, #100, #105, #111, and #115 ask how to adapt
  a new sensor or platform;
- most requests need the same input contract before vendor-specific tuning is
  meaningful.

Scope:

- add a compact checklist to `docs/workflows.md`;
- cover the `PointCloud2` topic, non-empty frame, static extrinsic, timestamps,
  scan period, and min/max range;
- show the public launch arguments used to remap the topic and frames;
- link to the sensor-support issue form for evidence that does not fit the
  checklist.

Acceptance:

- every checklist item has one observation command or configuration field;
- vendor names are examples only and are not added to the supported matrix;
- success means “ready for a controlled first run,” not “accuracy validated”;
- unsafe advice such as guessing an extrinsic is explicitly excluded.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: adding a driver, selecting universal
tuning values, or claiming support for hardware that has not passed a recipe.

### C4 local completion — custom PointCloud2 sensor checklist

The checklist is now implemented in the current product candidate at
`docs/workflows.md`. It covers topic/type/field inspection, non-empty frames,
timestamp and rate checks, measured TF/extrinsics, classic and RKO-LIO range
and period configuration, explicit topic/frame remapping, and the
sensor-support issue route. It states that the result is readiness for one
controlled first run rather than an accuracy or hardware-support claim, and it
rejects guessed extrinsics and raw-data uploads.

The queue's existing gap probe therefore reports C4 as `STALE`; the task must
not be rendered or published as if the work were still missing. This is a
local completion only and does not create or edit a GitHub issue.

## Completed C5 — Japanese empty-frame recovery card

Suggested issue title:

> Docs: explain empty `frame_id` recovery in the Japanese first-run card

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese card now states that a sampled `frame_id` must be
non-empty and tells the operator to repair the publisher's
`header.frame_id`, repeat the check, and avoid guessing a viewer frame. The
queue's drift probe deliberately retires this task after the marker appears.

The original reason for the task was:

- issue #102 shows the operator-visible empty-frame symptom;
- the English first-run card now explains that an empty sampled `frame_id` is
  a publisher problem, while the Japanese card still stops after the sample
  command and does not state the repair action;
- the fail-closed preflight implementation is already covered by the public
  candidate, so the next bounded contribution should close the language-path
  gap rather than duplicate the implementation.

Scope:

- extend the three-check recovery card in `docs/getting-started-ja.md`;
- state that a sampled `frame_id` must be non-empty;
- tell the operator to repair the publisher's `header.frame_id`, repeat the
  check, and avoid guessing a viewer frame;
- keep the existing topic and TF commands and the privacy boundary unchanged.

Acceptance:

- the Japanese three-check card states that a sampled `frame_id` must be
  non-empty;
- an empty or timed-out sample tells the operator to repair the publisher
  `header.frame_id` and repeat the check;
- the card tells the operator not to guess a viewer frame and keeps the
  existing topic and TF commands intact;
- no rosbag, hardware, network, or private log is needed by the change.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the preflight implementation,
translating the entire getting-started guide, or claiming support for an
unvalidated sensor.

## Completed C5 — Japanese TF frame substitution

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese card now connects the non-empty frame sampled in
check 2 to `<POINTCLOUD_FRAME>`, identifies the runtime/viewer target frame,
and tells the operator to reuse the same actual frame names when the TF check
fails. The queue's drift probe retires this task after the marker appears.
The preceding C5 topic-selection increment is also retained in the candidate:
it shows `ros2 topic list -t`, selects the
`sensor_msgs/msg/PointCloud2` row, and tells the reader to copy only its topic
name.

## Completed C5 — Japanese headless preview recovery

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese card now explains that the browser preview is a
self-contained local HTML artifact, shows the exact `HTML:` path emitted by
`view`, and gives copy-ready `--no-open` plus `--preview-dir` commands for
headless or browser-failure recovery. It also keeps the map data privacy
boundary visible by directing users to the sanitized support report instead of
uploading the preview, map, bag, or raw log.

The queue's drift probe retires this task after the `--no-open` marker appears.

Suggested issue title:

> Docs: add headless preview recovery to the Japanese first-run guide

Acceptance:

- explain that a browser not opening or a headless machine should use the
  printed self-contained HTML path;
- show `--no-open` and `--preview-dir` as safe preview options;
- keep the existing map verification, diagnosis, TF, topic-selection,
  empty-frame, and privacy guidance intact;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the browser, viewer, or preview
implementation, translating the entire guide, or claiming support for an
unvalidated sensor.

## Completed C5 — Japanese session history and recovery

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese card now explains `lidarslam-map sessions`, the
`--status action_required` filter, `--viewer none`, and read-only `--json`
inspection. It tells the operator to read retained `Details:` and `Next:`
values, follow the exact next command, and check `map_verify` and diagnosis
findings before changing viewer settings. Session history and recovery remain
local-only and preserve the no-private-upload boundary.

The queue's drift probe retires this task after the session marker appears.

Suggested issue title:

> Docs: explain Japanese session history and recovery

Acceptance:

- explain how to list saved sessions with `lidarslam-map sessions`;
- show `--status action_required`, `--viewer none`, and `--json` for focused or
  headless recovery;
- tell the reader to follow the retained `Next` action and keep preview,
  diagnosis, and no-private-upload guidance intact;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing session storage, recovery, or
viewer implementation, translating the entire guide, or claiming support for
an unvalidated sensor.

## Completed C5 — Japanese privacy-first support handoff

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now provides the read-only
`lidarslam-map support /path/to/session_bundle --json` inspection path, explains
that `--first-map` revalidates a receipt-bound PASS without creating a ZIP or
contacting GitHub, and names the three generated files that must be reviewed
before any selective sharing. It keeps maps, bags, raw logs, parameters,
private paths, credential-like command values, and the `--first-map --json`
handoff JSON outside the public attachment path.

The queue's drift probe retires this task after the support command marker
appears. The next task narrows the handoff from privacy review to independent
validation form use; it does not provide live troubleshooting.

Suggested issue title:

> Docs: explain Japanese privacy-first support handoff

Acceptance:

- show `lidarslam-map support /path/to/session_bundle --json` for read-only
  inspection;
- explain that `--first-map` is a read-only verified-first-map handoff and
  never uploads to GitHub;
- tell the reader to review `README.txt`, `issue-body.md`, and
  `support-report.json` before sharing, while preserving session, preview,
  diagnosis, and no-private-upload guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing support-bundle generation, issue
templates, or session implementation, translating the entire guide, or
claiming support for an unvalidated sensor.

## Completed C5 — Japanese independent first-map validation handoff

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now explains that `--first-map` prints the
canonical independent-validation issue form alongside the verification summary
and safe environment hints. It tells the operator to fill the form from their
own run, redact private paths from the command, and attach only the reviewed
first-map receipt. It explicitly excludes the local handoff JSON, receipt path,
map, bag, preview, raw log, trajectory, parameter, and screenshot attachments,
and preserves the no-live-guidance rule for independent validation.

The queue's drift probe retires this task after the independent-validation
marker appears. The next task moves one step earlier in the evidence chain:
recording the exact installed product identity before support or validation.

Suggested issue title:

> Docs: explain Japanese independent first-map validation handoff

Outcome:

A Japanese first-run user can fill the independent-validation form from their
own run and attach only a reviewed first-map receipt, without exposing a map,
bag, preview, raw log, local receipt path, or handoff JSON.

Acceptance:

- explain that `--first-map` prints the verification summary, safe environment
  hints, and the canonical independent-validation issue form;
- tell the operator to fill the form from their own run and attach only a
  reviewed first-map receipt;
- state that the handoff JSON and local receipt path are not public
  attachments, and that the command never uploads to GitHub;
- keep the existing support, session, preview, diagnosis, and no-private-upload
  guidance intact;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing support-bundle generation, issue
templates, or first-map validation schemas, translating the entire guide,
providing live guidance that would invalidate independent validation, or
claiming support for an unvalidated sensor.

## Completed C5 — Japanese version identity and support-boundary check

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now shows the read-only `lidarslam-map
--version` command, distinguishes the published v0.9.0 Docker images from the
unpublished v0.9.1 source candidate, and tells the operator to transfer the
observed version/revision rather than guessing a release identity. The existing
support, privacy, and independent-validation boundaries remain intact.

The queue's drift probe retires this task after the version marker appears. The
next task turns the Japanese failure path into stable-code triage so a user can
choose the correct next action before requesting support.

Suggested issue title:

> Docs: add a Japanese version identity and support-boundary check

Outcome:

A Japanese first-run user can record the installed product identity, match it
to the documented stable or candidate path, and avoid presenting an unpublished
candidate or moving tag as supported release evidence.

Acceptance:

- show `lidarslam-map --version` and tell the reader to record its output before
  support or validation handoff;
- distinguish the immutable published v0.9.0 Docker images from the unpublished
  v0.9.1 source candidate path;
- tell the reader not to use a moving `develop` tag or guess a release identity
  when reporting evidence;
- preserve the existing support, session, preview, diagnosis, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing release metadata, publishing an
image, changing version semantics, translating the entire guide, claiming that
an unpublished candidate is a stable release, or validating an untested sensor.

## Completed C5 — Japanese reason-code and Next-action triage

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now shows the read-only
`lidarslam-map doctor /path/to/rosbag2 --json` diagnosis path, distinguishes the
`findings[].code` field in the doctor report from `reason.code` and
`findings[].code` in start/session recovery JSON, and tells the reader to use
stable codes instead of viewer symptoms or English message text. It also tells
the reader to follow retained `next_action`, `Next:`, or `next_command` values,
separates safe `--resume` post-processing from returning to doctor, and keeps raw
JSON with local paths out of public support attachments.

The queue's drift probe retires this task after the next Japanese dry-run card
marker appears. The next task moves one step earlier in the write boundary:
showing how to inspect an own-bag plan before a session or map is created.

Suggested issue title:

> Docs: explain the Japanese dry-run and write boundary

Outcome:

A Japanese first-run user can inspect an own-bag plan before writes, understand
when a session or map may be created, and choose a controlled next action without
rerunning an unknown input blindly.

Acceptance:

- show `lidarslam-map start /path/to/rosbag2 --yes --dry-run --json` as a
  no-write preflight;
- explain that dry-run creates no session or map output and that confirmation is
  required before mapping writes;
- keep `--viewer none` as the safe choice for headless execution and name the
  retained next command after inspection;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the start or session
implementation, translating the entire guide, asking for a private bag/map/raw-
log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese dry-run and write boundary

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now shows
`lidarslam-map start /path/to/rosbag2 --yes --dry-run --json`, explains that the
dry-run returns a `status: dry_run` plan without leaving a session bundle or map,
and identifies `run.command_shell` as the retained command to review. It also
separates the no-maintained-profile `reason.code`/`findings[].code` path, keeps
the doctor `next_command` actionable, and states that `--viewer none` is the
headless route after the plan is accepted.

The queue's drift probe retires this task after the next Japanese fresh-retry
card marker appears. The next task protects failed-run provenance by making the
fresh output-directory rule visible in the Japanese recovery path.

Suggested issue title:

> Docs: explain Japanese fresh-output retry without overwrite

Outcome:

A Japanese first-run user can retry a failed or incomplete map in a fresh output
directory, preserve the retained setup and evidence, and avoid overwriting an
earlier run.

Acceptance:

- explain that an existing output directory is not overwritten and that a retry
  uses a fresh `--output-dir`;
- distinguish the retained pinned setup and evidence from a new map output
  directory;
- tell the reader to use the retained retry or next command rather than
  reconstructing a command from a viewer symptom;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the start, session, or recovery
implementation, translating the entire guide, asking for a private bag/map/raw-
log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese fresh-output retry without overwrite

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now explains that an existing `--output-dir` is
never overwritten, distinguishes the retained `setup_bundle` and evidence from
the fresh `retry.output_dir`, and tells the reader to use the generated
`retry.command` or `--resume` `next_command` instead of reconstructing a command
from a viewer symptom. The local-only and no-private-upload boundary remains
explicit.

The queue's drift probe retires this task after the next Japanese verification
boundary marker appears. The next task makes the trust boundary visible between
a map that displays and a map whose receipt-backed verification actually passed.

Suggested issue title:

> Docs: explain the Japanese verified-result boundary

Outcome:

A Japanese first-run user can distinguish a displayed map from a verified result,
read the retained receipt status, and avoid sharing or relying on an unverified
output as trusted evidence.

Acceptance:

- distinguish `map_verify: PASS` from a map that merely displays in a viewer;
- name the retained `first_map_validation_receipt.json` and explain `NOT VERIFIED`
  or `UNAVAILABLE` without guessing;
- direct the reader to the retained diagnosis or inspect command before support
  or independent validation;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing verification implementation or
quality thresholds, translating the entire guide, asking for a private
bag/map/raw-log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese verified-result boundary

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now distinguishes a viewer-rendered map from a
trusted result: the same run must show `map_verify: PASS` and retain a
`first_map_validation_receipt.json` whose status is `PASS`. It explains
`NOT VERIFIED` and `UNAVAILABLE` without guessing, directs the reader through
the retained diagnosis and `inspect` command, and keeps receipt review and the
no-private-upload boundary before support or independent validation.

The queue's drift probe retires this task after the next Japanese receipt/session
boundary marker appears. The next task confirms that a PASS receipt belongs to
the same session and output being discussed.

Suggested issue title:

> Docs: explain the Japanese verified-result boundary

Outcome:

A Japanese first-run user can distinguish a displayed map from a verified result,
read the retained receipt status, and avoid sharing or relying on an unverified
output as trusted evidence.

Acceptance:

- distinguish `map_verify: PASS` from a map that merely displays in a viewer;
- name the retained `first_map_validation_receipt.json` and explain `NOT VERIFIED`
  or `UNAVAILABLE` without guessing;
- direct the reader to the retained diagnosis or inspect command before support
  or independent validation;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing verification implementation or
quality thresholds, translating the entire guide, asking for a private
bag/map/raw-log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese receipt/session match boundary

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now binds a receipt to the same session and map
output, explains the `run_id` and `manifest_sha256` provenance fields, and makes
`support --first-map` the read-only revalidation gate. A receipt is a sharing
candidate only after the handoff reports `READY FOR REVIEW`; copied, stale,
mismatched, or failed evidence remains untrusted and the existing privacy
boundary stays in place.

The queue's drift probe retires this task after the next Japanese failed-
revalidation recovery marker appears. The next task explains how to preserve old
evidence and recover safely when that gate rejects a receipt.

Suggested issue title:

> Docs: explain the Japanese receipt/session match boundary

Outcome:

A Japanese first-run user can confirm that a validation receipt belongs to the
same session and output, and can reject copied, stale, mismatched, or failed
evidence as untrusted.

Acceptance:

- tell the reader to match `first_map_validation_receipt.json` to the same
  session/output and read its top-level status;
- treat `status: FAIL`, a missing or malformed receipt, or failed receipt
  revalidation as untrusted rather than inferring PASS from a viewer;
- direct the reader to the retained diagnosis, manifest, verification log, or
  `inspect` command before support or independent validation;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing verification implementation,
receipt schemas, or quality thresholds, translating the entire guide, asking for
a private bag/map/raw-log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese failed receipt revalidation recovery

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now explains that a rejected read-only
`support --first-map` handoff leaves the original map, session, receipt, and
manifest intact. It directs the operator through local-only session and
`map_session_recovery.json` inspection, retained `Details:`/`Next:` values,
`--resume`, the pinned fresh-output `retry.command`, and a separate
verification-required run when neither recovery path is available. The guide
keeps old and new evidence separate, forbids editing or copying receipts, and
requires a new `READY FOR REVIEW` handoff before a reviewed receipt becomes a
sharing candidate.

The queue's drift probe retires this task after the failed-revalidation marker
appears. The next task turns the completed recovery path into a short final
check before any public sharing.

Suggested issue title:

> Docs: explain the Japanese failed receipt revalidation recovery

Outcome:

A Japanese first-run user can respond to a rejected first-map handoff by
preserving the old evidence and using a fresh verification-enabled output
without editing receipts or claiming support.

Acceptance:

- explain that a rejected `support --first-map` revalidation does not destroy
  the old map or justify editing its receipt, manifest, or session;
- direct the reader from a rejected handoff to retained `Details:`, `Next:`,
  `retry.command`, or a fresh verification-enabled output command;
- preserve the non-overwrite, retained-evidence, local-only, and no-private-
  upload boundaries while describing recovery;
- preserve the existing version, support, session, preview, privacy, and
  independent-validation guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing receipt validation, recovery, or
verification implementation, translating the entire guide, asking for a private
bag/map/raw-log upload, or claiming support for an unvalidated sensor.

## Completed C5 — Japanese pre-share verification checklist

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now provides a copy-ready five-item gate for
product version/revision, same-session and output identity, read-only
`READY FOR REVIEW` revalidation, receipt privacy, and the single permitted
public attachment. It explicitly separates the local-only handoff JSON and
paths from the reviewed receipt, requires private-path redaction in the
operator-supplied command, and excludes maps, bags, logs, previews, and the
session bundle from public sharing.

The queue's drift probe retires this task after the five-item checklist marker
appears. The next task turns those confirmed fields into a Japanese public
report template that a validator can fill without copying local evidence.

## Completed C5 — Japanese reviewed-receipt public share template

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now mirrors the public validation form with a
copy-ready block for the documentation path, immutable release/commit/image
digest, environment, private-path-redacted command, PASS verification summary,
findings, and the reviewed receipt attachment. It states that handoff JSON,
receipt paths, session evidence, maps, bags, logs, previews, and other run
artifacts stay local or unshared, while only the reviewed PASS receipt is a
public attachment candidate.

The queue's drift probe retires this task after the public-share-template
marker appears. The next task explains the distinction between a local handoff,
a public report, maintainer review, and accepted ledger evidence.

## Completed C5 — Japanese validation report review status

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now distinguishes local `READY FOR REVIEW`,
public report submission, maintainer review, accepted ledger evidence, and
unresolved or rejected reports. It explicitly states that a local handoff or
public receipt is not accepted validation until public review and ledger
requirements pass, keeps one report/receipt pair, forbids evidence editing or
duplication, and points contributors away from live step-by-step validation
help.

The queue's drift probe retires this task after the validation-report review
status marker appears. The next task supplies a path-free instructional example
without turning an example into accepted evidence.

## Completed C5 — Japanese privacy-safe validation report example

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now includes one path-free, clearly illustrative
report example with an immutable-identity placeholder, redacted command,
receipt-derived verification fields, operator-supplied public fields, and an
explicit `not submitted` / `not maintainer-reviewed` / `not accepted` status.
It warns contributors not to copy example hashes or treat the example as real
evidence, and keeps paths, maps, bags, logs, previews, and session bundles out
of the public example.

The queue's drift probe retires this task after the privacy-safe report-example
marker appears. The next task helps a user recover safely when switching
between the supported first-map routes.

## Completed C5 — Japanese Docker/source route chooser

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now has one route-choice card with a copy-ready
Docker fixed-demo command, a source `--dry-run` command, one stop/check boundary
for each, the published v0.9.0 versus unpublished v0.9.1 identity boundary, and
the unsupported PPA/package-manager boundary. It tells beginners to choose one
route, use fresh output when changing routes, and never mix Docker/source
receipts or session artifacts.

The queue's drift probe retires this task after the Docker/source route-choice
marker appears. The next task separates sanitized support diagnostics from
independent-validation evidence.

## Completed C5 — Japanese fresh-output route-switch recovery card

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now separates same-session `--resume`,
same-pinned-setup `retry.command`, and a changed Docker/source route. It requires
fresh output for a route switch, preserves the old map/session/receipt/manifest,
keeps v0.9.0 Docker and v0.9.1 source identity separate, and directs contributors
to retained `Details:`/`Next:` instructions without rebuilding commands from
viewer appearance.

The queue's drift probe retires this task after the fresh-output route-switch
marker appears. The next task separates sanitized support diagnostics from
independent-validation evidence.

## Completed C5 — Japanese support report versus validation report

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now gives one decision card for ordinary
sanitized support diagnostics and one for an independent validation report. It
separates the `support --json` report, the local-only `--first-map` handoff, the
operator-authored public validation report, and the reviewed receipt; support
diagnostics are explicitly not accepted validation evidence.

The card retains the no-private-upload boundary for local paths, recovery JSON,
maps, bags, logs, previews, and session bundles. It also keeps the published
v0.9.0 Docker identity separate from the unpublished v0.9.1 source candidate and
requires the reporter to record the exact identity rather than infer it from a
viewer.

The queue's drift probe retires this task after the support-versus-validation
marker appears. The next task makes the provenance of every public validation
report field explicit so contributors do not invent a hash, status, or identity.

## Completed C5 — Japanese public validation report field provenance

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now has a field-provenance table that separates
operator-supplied public fields, same-run receipt-derived validation fields, and
review or acceptance status. It binds identity, result, verification summary,
manifest hash, and receipt attachment to the same session and tells the reader
to stop on missing, unavailable, example-only, viewer-only, or mismatched values.

The card retains the no-private-upload boundary for local paths, recovery JSON,
maps, bags, logs, previews, trajectories, parameters, screenshots, and session
bundles. It keeps the published v0.9.0 Docker identity separate from the
unpublished v0.9.1 source candidate and preserves the existing support,
independent-validation, receipt, and review-status guidance.

The queue's drift probe retires this task after the field-provenance marker
appears. The next task makes an operator's public `findings` field actionable
without allowing it to rewrite receipt-derived evidence or disclose private data.

## Completed C5 — Japanese validation-report findings without private data

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now gives a four-field `step`/`expected`/
`observed`/`impact` pattern for one operator observation, distinguishes that
observation from receipt-derived evidence, forbids private artifacts, root-cause
claims, and acceptance claims, and routes maintainer-live-guidance-only
observations to the sanitized support report.

The queue's drift probe retires this task after the findings marker appears. The
next task routes unresolved findings to support or a safe retry without
duplicating evidence or treating follow-up as accepted validation.

## Completed C5 — Japanese validation-report finding follow-up

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now provides a two-route decision table for an
unresolved finding: a sanitized support follow-up using saved `Details:`,
`Next:`, and unedited `retry.command` instructions with fresh output, or a new
independent validation using a fresh run. It keeps the original report, receipt,
manifest hash, and review status unchanged, forbids duplicate issue/session
artifacts, and stops when identity or same-session data is missing.

The card preserves the v0.9.0 Docker versus v0.9.1 source-candidate boundary,
the no-private-upload rule, and the distinction between `READY FOR REVIEW`,
unresolved follow-up, and accepted ledger evidence. The queue's drift probe
retires this task after the finding-follow-up marker appears.

The next task makes the original report/receipt pair and a follow-up summary
easy to audit without creating duplicate evidence.

## Completed C5 — Japanese validation-report follow-up evidence pairing

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now distinguishes the original
`report + reviewed receipt` pair, a follow-up note, and a new
independent-validation report. It gives a copy-ready audit block with route,
`reason.code`, sanitized `Details:`/`Next:`, fresh-output facts, review status,
and a duplicate-artifact check, while keeping the original evidence immutable
and local artifacts private.

The queue's drift probe retires this task after the follow-up-evidence marker
appears. The next task makes the sanitized follow-up summary itself easy to
audit without turning it into a new validation result.

## Completed C5 — Auditable Japanese validation-report follow-up summaries

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now gives a maintainer-facing audit order for
identity, same run/output, route, field provenance, and review status. It
classifies a summary as `MATCHED FOLLOW-UP`, `NEW RUN`, or `STOP`, and keeps the
original report, receipt, hash, and private artifacts outside the public note.

The queue's drift probe retires this task after the follow-up-summary marker
appears. The next task defines safe dispositions for the three audit outcomes
without changing evidence or accepting a note.

## Completed C5 — Auditable Japanese validation-report follow-up dispositions

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now gives a maintainer-facing audit order for
identity, same run/output, route, field provenance, and review status. It
classifies a summary as `MATCHED FOLLOW-UP`, `NEW RUN`, or `STOP`, and keeps the
original report, receipt, hash, and private artifacts outside the public note.

The queue's drift probe retires this task after the follow-up-disposition marker
appears. The next task defines the permitted action and public status for each
classification without editing evidence, duplicating artifacts, or accepting a
note.

## Completed C5 — Safe actions after Japanese validation-report follow-up dispositions

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now defines the permitted action and public
status for `MATCHED FOLLOW-UP`, `NEW RUN`, and `STOP`. It keeps matched support
on the original pair, limits a new run to one new report/receipt pair, and
preserves the support route for `STOP` without editing evidence or accepting a
note.

The queue's drift probe retires this task after the safe-disposition marker
appears. The next task records the actual action result separately, so a
classification cannot be mistaken for proof that an operation was completed.

## Completed C5 — Auditable Japanese validation-report follow-up action results

Implementation status:

This bounded documentation task is implemented in the local product candidate
for PR #427. The Japanese guide now separates the disposition from the action
result and records allowed evidence changes, artifact count, sanitized
`Details:`/`Next:`, and the handoff result. It keeps matched follow-up and new
run boundaries explicit, preserves immutable evidence, and stops on an
out-of-range or mismatched action.

The queue's drift probe retires this task after the action-result marker
appears. The next task makes the receiver-facing handoff reproducible without
granting acceptance or receipt-edit authority.

## Successor C5 — Reproducible Japanese validation-report follow-up handoff

The next prepared C5 task keeps the Japanese language-path scope and lets a
support or maintainer receiver continue a follow-up from a path-free handoff.
It should separate routing from acceptance, preserve immutable evidence,
prevent duplicate artifacts, and avoid maintainer-memory-only instructions.

Suggested issue title:

> Docs: make Japanese validation-report follow-up handoff reproducible

Outcome:

A support or maintainer receiver can continue a Japanese follow-up from a
path-free handoff without rewriting evidence, duplicating artifacts, or
accepting a note.

Acceptance:

- map `NOTE ONLY`, `SUPPORT REQUESTED`, `RETRY STARTED`, `NEW PAIR PREPARED`,
  and `STOPPED` to a support, maintainer-review, or stop handoff with an
  explicit public status;
- provide a handoff block with disposition, action result, target, status,
  owner role, next review, sanitized `Details:`/`Next:`, and no private paths;
- make clear that a handoff does not grant acceptance or receipt-edit
  authority;
- keep the original report, receipt, and hash immutable, permit one
  report/receipt pair per run, and forbid duplicate issue or session artifacts;
- preserve one handoff per action audit and route missing or mismatched
  identity/session/output back to `STOP` and safe support/retry;
- preserve the v0.9.0 Docker versus v0.9.1 source-candidate identity boundary
  and the existing support, session, privacy, independent-validation, and
  review-status guidance;
- require no rosbag, hardware, network, or private log.

Focused check:

```bash
python3 -m mkdocs build --strict
```

Estimate: **30 minutes**. Non-goals: changing the issue template, review
ledger, support or verification implementation, changing the Docker image,
source helper, recovery implementation, or release identity, translating the
entire guide, asking for a private bag/map/raw-log upload, or claiming support
for an unvalidated sensor.

## Refreshed candidates C6–C9

The four replacement scopes come from support demand that remains visible in
the 29-issue backlog and is not covered by the completed C1–C4 cards. They are
documentation tasks, not invitations to redesign SLAM algorithms.

| ID | Public demand | Bounded outcome | Allowed file |
| --- | --- | --- | --- |
| C6 | #101, #116, #124 | distinguish live `current_pose`/`path` from backend `modified_path`, verify message/frame contracts, and avoid describing optimized history as rewritten live odometry | `docs/workflows.md` |
| C7 | #101, #116, #118 | separate mapping, loop closure, prior-map localization, and lost-track relocalization; route each intent to a supported workflow or explicit boundary | `docs/workflows.md` |
| C8 | #89, #96, #100 | make classic `use_imu` readiness copy-ready: topic type, non-empty frame, timestamps/rate, measured base-to-IMU transform, and LiDAR `scan_period` | `docs/workflows.md` |
| C9 | #53, #69, #89, #104 | triage a long-route stop by input, process, registration refusal, compute, storage, and viewer stage without inventing a universal distance limit | `docs/getting-started.md` |

Each task is capped at 30 minutes in a prepared environment, uses the fixed
strict-MkDocs profile, needs no hardware or private data, and has an exact
heading probe that retires it as soon as the planned card exists. C7 may
document the experimental MID-360 relocalization evidence only as bounded
research evidence; it may not turn that experiment into general product
support. C9 may link public benchmark envelopes but may not infer a universal
distance, runtime, memory, or accuracy guarantee.

The copy-ready bodies remain generated from the machine contract:

```bash
python3 scripts/contributor_starter_queue.py --task starter-C6
python3 scripts/contributor_starter_queue.py --task starter-C7
python3 scripts/contributor_starter_queue.py --task starter-C8
python3 scripts/contributor_starter_queue.py --task starter-C9
```

## Publication and review sequence

1. Recheck each candidate against the then-current public `develop` revision.
2. Confirm that no open issue or pull request already implements the exact
   acceptance criteria.
3. Obtain explicit authorization before creating or editing GitHub issues.
4. Keep C1–C4 out of the publication batch because their local cards are now
   complete; do not rename a heading merely to make a retired task look ready.
5. Treat C5–C9 as the current locally ready generation and rerun every live
   duplicate query before any separately authorized publication.
6. Publish no more tasks than the available review capacity can move; local
   `READY` never means an issue exists or a contributor has claimed it.
7. Keep #422 open independently until three accepted external first-map
   validations exist, but do not recommend or recruit through it unless its
   cohort state is exactly `READY_FOR_NEXT_ATTEMPT`.

The old source issues are not automatically closed when a starter task is
published or merged. Each old issue still needs a supported-version check, a
public resolution or support-boundary explanation, and an explicit disposition.
Those issue-specific decisions are recorded in the
[complete read-only triage proposal](open-issue-triage-proposal-2026-08-11.md).

## Success and stop rules

The first five tasks are successful when:

- at least three are completed by non-maintainers;
- median prepared-environment completion time is at most 30 minutes;
- every task is accepted using only its listed focused checks;
- at least two reusable support answers move into public documentation; and
- no task expands into an unbounded hardware or algorithm project.

If two consecutive starter contributions exceed the estimate by more than
15 minutes because of repository setup or test cost, stop publishing more
starter issues and repair the contributor path. If review capacity cannot keep
the published tasks moving, reduce the visible ready queue instead of inviting
more contributors into a stalled path.

## Independent starter review while the cohort is closed — 2026-08-17

> Decision: **LOCAL_COMMUNITY_QUEUE_PROGRESS / ISSUES_NOT_PUBLISHED**
>
> Implementation tip:
> `e4ce0aa6eb7d53423423a02af65f923472708f44`
>
> GitHub issue, label, assignment, comment, or pull request mutations: **none**

An authenticated GET-only live card found one published `good first issue`,
#422, correctly blocked by `WAITING_FOR_PUBLIC_GATES`; five local C5–C9 tasks
were `READY`, and no open pull request matched their current duplicate queries.
The previous maintainer action still selected the blocked cohort gate, so an
issue-specific dependency unintentionally stopped review preparation for all
five independent documentation tasks.

The maintainer action now applies this strict order: review any potential pull
request duplicate first, review one independent publishable local task next,
and inspect the blocked published-issue gate only when no independent task is
ready. Contributor behavior does not change: #422 remains visible but cannot
be recommended, and no unpublished local task is presented as claimable.

At the exact implementation tip, the card keeps status
`PUBLISHED_GOOD_FIRST_ISSUES_BLOCKED` and tells contributors to wait, while the
single maintainer action previews `starter-C5` with:

```bash
python3 scripts/contributor_starter_queue.py --task starter-C5
```

The exact JSON SHA-256 is
`ff5f81d93d1a8f9c5854359b9fede8c5fadac40ab1f287336ab1bedfacf484b2`.
Both C5 and C6 focused strict-MkDocs profiles pass with
`workspace_artifacts_written: false`; 62 queue regressions, the exact
322-test S6 integration command, and 61 publication-plan/routing/G0
regressions pass. Strict MkDocs and ROS lint also pass. The card uses GET-only
GitHub requests and retains false write and remote-mutation authority.
Reviewing the body remains separate from authorizing or creating an issue.

## Exact local publication handoff — 2026-08-17

> Decision: **PUBLICATION_HANDOFF_READY_LOCAL_ONLY / ISSUE_WRITE_UNAUTHORIZED**
>
> Implementation tip:
> `0a34e724875d53b8ef74acd8a51fd500ce014ff5`
>
> Issue creation, label application, assignment, comment, or other GitHub
> mutation: **none**

The live `--next --json` report now binds the selected C5 review to one
tamper-evident `maintainer_publication_handoff`. It contains the exact
repository, title, sorted labels, GitHub body without a duplicated title
heading, canonical task and queue hashes, and the body SHA-256. The handoff is
present exactly when `maintainer_next` selects a local task; duplicate review,
an exhausted queue, or another non-publication action requires `null`.

The exact authenticated GET-only observation retained zero potential task
duplicates and produced:

- body SHA-256:
  `df48cba21d0f3cf4fc2443f4538a8666b55000e550f0dded3898a49f294b65e8`;
- task SHA-256:
  `f32b2d90a251983dc50135a25da75f4dc39ef99bc7ece7d806b3d4c3dd39e558`;
- queue SHA-256:
  `6eca845d333fc506d215c4f2548916037737cecd04e34823789dcb0acef778f0`;
- `external_write_required: true` and
  `maintainer_confirmation_required: true`; and
- `issue_creation_authorized: false`, `writes_performed: false`, and remote
  mutation false.

Body tampering, cross-task linkage, unsorted labels, or a handoff attached to a
non-publication action fails closed. Sixty-three queue regressions, the exact
323-test S6 integration command, 61 plan/routing/G0 regressions, strict MkDocs,
and ROS lint pass. This packet removes title/label/body reconstruction; it does
not authorize or execute issue creation.

## Public-base gate for local starter publication — 2026-08-17

> Decision: **POST_MERGE_PREPARATION_ONLY / PUBLIC_BASE_NOT_READY**
>
> Implementation tip:
> `5dc04198549341b33becdeb2bc058117db9fe78f`
>
> GitHub requests: **GET-only**
>
> Issue, label, assignment, comment, pull-request, or branch mutations:
> **none**

The exact handoff above removed stale title/body reconstruction but could still
say `REVIEW_AND_PUBLISH_LOCAL_TASK` while its product instructions existed only
inside Draft PR #427. The queue now declares a second, independent publication
gate. A local C5–C9 body may reach publication review only after both conditions
are true:

1. PR #427 is merged into its exact `develop` base; and
2. `docs/contracts/contributor-starter-queue-v1.json` exists on public
   `develop` and its canonical JSON SHA-256 matches the local queue.

The authenticated 2026-08-17 GET-only observation finds PR #427 at public head
`4b2ab514a4f33b443e2c4283b3114d11a5e44e49`, `OPEN`, Draft, and not merged.
The canonical queue path returns HTTP 404 on public `develop`. The derived gate
is therefore `WAITING_FOR_PRODUCT_MERGE`, with `public_queue_status: ABSENT`
and blockers `product_pr_not_merged` plus `public_queue_absent`. The one
maintainer action is now `PREPARE_LOCAL_TASK_FOR_POST_MERGE`, not publication.
It retains the exact C5 body for review and gives one GET-only recheck command:

```bash
python3 scripts/contributor_starter_queue.py --next --json
```

The current queue SHA-256 is
`0b3d40e3b43aa6a35026aa53c6610ffd9cf8cc46a0e3d4338da927172b551243`;
the unchanged task and body SHA-256 values are
`f32b2d90a251983dc50135a25da75f4dc39ef99bc7ece7d806b3d4c3dd39e558`
and
`df48cba21d0f3cf4fc2443f4538a8666b55000e550f0dded3898a49f294b65e8`.
The report also carries `public_base_ready: false`,
`issue_creation_authorized: false`, and `writes_performed: false`.

Seventy-one queue regressions cover READY, open Draft, closed-unmerged,
public-file absence, canonical match, drift, wrong-base rejection, wrapped
base64 content, and HTTP 404. The exact S6 integration command passes 331
tests, and strict MkDocs plus changed-file critical Python lint pass. A merge,
public queue match, fresh duplicate audit, maintainer confirmation, and a
separate authorized write remain distinct events.
