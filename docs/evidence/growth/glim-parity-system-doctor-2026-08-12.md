# GLIM parity: bag-optional system doctor — 2026-08-12

> Decision: **LOCAL_UX_INCREMENT_PASS / PUBLIC_COMPARISON_PENDING**
>
> Candidate base: public Draft PR `#427` head `3f4dd70`
>
> Network or files written by `lidarslam-map doctor`: **none**
>
> Remote mutations performed: **none**

## Why this increment

The current GLIM documentation presents four adoption advantages that matter to
a new user: PPA binary packages for Humble/Jazzy, prebuilt Docker images, a
direct rosbag executor, and an offline viewer that supports correction, export,
object removal, and session merging. Its setup documentation also separates
sensor/topic configuration from normal execution.

`lidar_slam_ros2` now overlaps the direct bag, sensor setup, local 3D review,
editing, and session-merge tasks through `start`, `setup`, `view`, `edit`, and
`merge`. GLIM's PPA remains the largest installation advantage. That cannot be
closed honestly by adding another source script: this project's package-manager
path still depends on reviewed NDT ownership and rosdistro publication.

The largest unblocked adoption gap was therefore the moment immediately after
installation. Previously, `doctor` required a bag, so a user could not ask
whether the product surface, ROS environment, and demo storage were ready
before locating data. The no-argument terminal home also had no safe route for
someone whose intent was simply “check this installation.”

Primary comparison sources inspected on 2026-08-12:

- [GLIM installation](https://koide3.github.io/glim/installation.html)
- [GLIM getting started](https://koide3.github.io/glim/quickstart.html)
- [GLIM Docker images](https://koide3.github.io/glim/docker.html)
- [GLIM README](https://github.com/koide3/glim/blob/master/README.md)

## Product change

`lidarslam-map doctor` now has two explicit modes:

```bash
lidarslam-map doctor
lidarslam-map doctor /path/to/rosbag2
```

Without a bag it checks, read-only:

1. the curated runtime-file inventory and current product version;
2. whether a source checkout has a matching installed prefix;
3. Humble/Jazzy activation and `ros2` availability;
4. `rosbag2_py`, required for safe input inspection; and
5. free space for the fixed demo, defaulting to the existing 8 GiB floor.

The report is governed by `system-doctor-v1.schema.json`. It returns `ready` or
`action_required`, stable finding codes, and one copy-ready `next_action` per
finding. JSON intentionally omits checkout, home, install-prefix, and
demo-directory paths. `--demo-dir` chooses another filesystem and
`--min-free-space-gib` can raise the floor. A successful diagnosis exits zero
even when action is required; automation keys on `status` and finding codes.

With a bag, the dispatcher delegates to the existing
`preflight_autoware_map_bag.py` implementation. Topic, PointCloud2 field,
timestamp, and maintained-profile behavior is not forked. System-only storage
options are rejected in bag mode rather than ignored.

The interactive no-argument home adds **Check this installation** before full
help. It prints the exact `lidarslam-map doctor` command and runs immediately
without confirmation because the contract proves there is no network or write.
Demo confirmation and own-bag calibration review remain unchanged.

## Verification

| Check | Result |
| --- | --- |
| source/installed ready reports and schema invariants | PASS |
| missing build, runtime file, ROS, CLI, bag reader, and storage findings | stable-code regressions PASS |
| privacy-bounded JSON | local path exclusion PASS |
| bag-mode exact delegation and option separation | PASS |
| TTY home doctor route and unchanged automation behavior | PASS |
| CLI option/help and machine contract | PASS |
| graph product CLI and documentation contracts | PASS |
| focused lidar_slam tests | 37 passed |
| focused graph tests | 22 passed |
| non-symlinked Jazzy install | build/install PASS; helper, manifest, and schema installed |
| fresh-environment absolute installed launcher | `ready`; 53/53 helpers; Jazzy, `ros2`, and `rosbag2_py` ready |
| non-symlinked Humble overlay install | network-isolated immutable image build/install PASS; helper, manifest, and schema installed |
| Humble installed launcher and complete installed-product gate | `ready`; 53/53 helpers; Humble, `ros2`, and `rosbag2_py` ready; PASS |
| Humble report schema | Draft 2020-12 validation PASS; no local path disclosure |
| installed bytecode state | zero cache artifacts before and after doctor |
| complete maintained Python gate | graph: 1,428 passed / 13 skipped / 11 existing warnings; lidar_slam: 670 passed; 2,098 total |
| Python style/docstrings/copyright | `ament_flake8` 7 files; `ament_pep257` and `ament_copyright` 2 files; PASS |
| documentation | `mkdocs build --strict`: PASS with pre-existing Material/navigation notices |
| machine formats and shell | 89 versioned candidate JSON files parse; shell syntax and `git diff --check` PASS |

The system doctor now has non-symlinked installed proofs on both Jazzy and
Humble. The Humble overlay was built with network access disabled from the
immutable Humble image digest recorded by the distribution evidence, and its
installed-product gate exercised both doctor modes. The complete public CI
matrix still must run on the exact future candidate before it can be proposed
publicly.

## Honest boundary

This increment improves diagnosis and first-command confidence; it does not
create a PPA, Debian package, release, public benchmark, or independent first
map. It also does not prove parity from a feature list. After source publication,
the scorecard must measure command discovery, clean installation, fixed-demo
completion, failure recovery, and active operator time on equivalent Humble and
Jazzy hosts.

## Low-storage recovery follow-up — 2026-08-16

> Decision: **LOCAL_ACTIVATION_REPAIR_PASS / PUBLIC_OBSERVATION_PENDING**
>
> Implementation tip:
> `d01652080485bc68354f354043e4b2e732439223`
>
> Safety floor changed: **no; remains 8 GiB by default**
>
> Network, GitHub, release, or community mutations: **none**

An actual fixed-demo preflight on the Jazzy source candidate reproduced the
remaining recovery gap. With about 6.24 GiB free, system doctor identified low
storage but formerly returned a `<dir>` placeholder; demo dry-run only said to
free space or choose another directory. The operator therefore had to compute
the shortage and reconstruct a command before retrying.

At the implementation tip, both versioned reports expose exact
`additional_bytes_required`. Human output rounds the shortage upward to the
next 0.01 GiB, so it never understates what must be freed. System-doctor JSON
continues to omit the selected local path and returns the placeholder-free
`lidarslam-map doctor` retry. Demo JSON already permits its selected paths and
now retains the complete shell-quoted demo command, including paths with
spaces and all effective storage/viewer options.

The same host then reported:

```text
Demo storage: 6.2 GiB free; 8.0 GiB required; free 1.76 GiB more
additional_bytes_required: 1884504064
Next: Free at least 1.76 GiB ... then run: <complete command>
```

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| doctor/demo focused and schema tests | 28 passed |
| CLI, installed-contract, home, and completion focus | 66 passed |
| complete sourced `lidarslam/test` | 992 passed |
| complete sourced `graph_based_slam/test` | 1,442 passed, 13 skipped, 11 pre-existing ImageIO warnings |
| registered `lidarslam` CTest, including lint | 93 / 93 passed |
| registered `graph_based_slam` CTest, including lint | 232 / 232 passed |
| strict MkDocs and JSON parsing | PASS; only pre-existing Material/navigation notices |
| patch hygiene | `git diff --check` PASS |

This closes one locally observed activation failure. It is not a clean-host
timing result, an independent first map, a paired GLIM scorecard, or evidence
that the unpublished v0.9.1 distribution paths are ready.

## Single recovery action follow-up — 2026-08-17

> Decision: **LOCAL_SINGLE_ACTION_RECOVERY_PASS / EXTERNAL_FIRST_ATTEMPT_PENDING**
>
> Implementation tip:
> `a83bbfeaea8196a19513c7a26772d500fe8419b8`
>
> Network, files, GitHub, release, or community mutations performed by the
> observed doctor run: **none**

A real invocation from an unconfigured source-checkout shell retained five
valid findings: missing source install, ROS environment, `ros2`, `rosbag2_py`,
and fixed-demo storage. The previous human card presented a recovery beside
every finding, leaving a beginner to infer dependency order.

The system report now exposes one required top-level `next_action`. In an
`action_required` report it copies the first dependency-ordered finding into
schema-bound `code`, `reason`, and `action` fields; in a `ready` report it is
exactly `null`. The human card renders that selection once under **Do this
now**, retains every remaining stable finding code as a visible follow-up, and
asks the operator to rerun doctor so the remaining state is reprioritized. The
per-finding JSON recovery text remains intact for automation and detailed
inspection.

At the exact implementation tip, the observed five-finding report selected
`source-build-required` and the existing copy-ready
`source_quickstart.sh --build-only` action. Its JSON SHA-256 was
`08c74e4867d5e6848587fa7c5a69c3a72452a7ef461fe8ee4f776e4601d3b4bd`;
the human card SHA-256 was
`4c4071c3882b32076a499847e34c012124a3d5d9d0b3a49bef9541d7e1fe849d`.
Both reported `network_accessed: false` and `writes_performed: false`.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| doctor, home, option, completion, and installed-CLI focus | 51 passed |
| exact S6 graph docs/product command | 35 passed |
| exact S6 integrated product/growth command | 321 passed |
| G0 dashboard regressions | 21 passed |
| schema status/action coupling and first-finding selection | PASS |
| Jazzy `ament_flake8` and `ament_pep257` | PASS |
| strict MkDocs and patch hygiene | PASS |

This removes one locally reproduced decision burden from the existing doctor;
it does not add another diagnosis surface, perform the selected build, prove a
clean-host completion time, create a paired GLIM observation, or claim parity.

## One-action own-bag handoff follow-up — 2026-08-17

> Decision: **LOCAL_ONE_ACTION_BAG_HANDOFF_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `387a002dc7826be267fe600db906f80460e6f270`
>
> Mapping, network, file, GitHub, release, or community mutations performed by
> the observed doctor command: **none**

The product-dispatched bag report formerly exposed the internal beginner
script, browser variant, raw launch command, and every compatible path even
after selecting a primary profile. A first-time operator therefore still had
to choose which mapping command should follow a successful diagnosis.

At the implementation tip, a ready `lidarslam-map doctor <bag>` report keeps
the selected profile and its reasons, but renders exactly one shell-safe
exact-input `lidarslam-map start <bag>` under **Do this now**. If any finding
remains, the report withholds start, points to the first finding, and renders
the exact-input doctor retry. Direct use of the preflight script retains the
detailed developer commands and alternatives; machine-readable preflight and
path-free public-evidence contracts are unchanged. The dispatcher and renderer
round-trip command and bag paths with shell quoting instead of concatenating
untrusted path text.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| bag preflight and shell-safe ready/finding handoffs | 32 passed / 2 dependency skips |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, and patch hygiene | PASS |

This closes one local command-selection gap without running mapping or hiding
expert evidence. It is not a clean-host timing result, a paired external GLIM
observation, an independent first map, a package-manager release, or a parity
or superiority claim.

## Compact own-bag readiness card follow-up — 2026-08-17

> Decision: **LOCAL_COMPACT_BAG_CARD_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `fc87cf86cabba5f55fec47316c6a9a3a4e4cb90f`
>
> Mapping, network, file, GitHub, release, or community mutations performed by
> the observed doctor command: **none**

The preceding one-action change removed command choice but left the product
report shaped like an expert preflight: nine possible input categories, topic
names, per-check reasons, profile reasons, every finding message/action, and
advisory commands appeared before the operator reached support. That evidence
is useful locally, but it makes the default success path harder to scan.

The product-dispatched card is now bounded to status, bag duration and message
count, detected input types without topic/frame names, selected profile, four
check statuses, and one action. A ready fixture is regression-limited to at
most 26 lines. A finding card prints the first message/action, reduces the
remainder to stable codes, withholds start, and gives the exact-input retry.
The card exposes one shell-safe private `doctor <bag> --json` command for full
local detail and the separate path-free public-evidence command. Direct
preflight continues to render every topic, reason, alternative, finding, and
advisory; no machine schema changed.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| compact/direct bag preflight boundaries | 32 passed / 2 dependency skips |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, and patch hygiene | PASS |

This lowers default reading cost without deleting expert diagnostics or
changing automation. It is not an external first-time observation, a paired
GLIM scorecard result, an independent first map, or a parity/superiority claim.

## Single-prompt own-bag start follow-up — 2026-08-17

> Decision: **LOCAL_SINGLE_PROMPT_START_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `90c508eef4c6ce6868582bda80e684f14223ea4a`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

The compact doctor card now hands a ready bag to `start`, but the next RKO
screen previously displayed an exact second `--yes` command immediately before
the same process asked for confirmation. That made a first-time operator choose
between copying the displayed command and answering the prompt.

The interactive control flow now renders the profile extrinsics once, tells the
operator to review them and answer the fail-closed prompt immediately below,
and omits the redundant `--yes` command. Non-interactive `start`, `setup`, and
dry-run retain the exact reviewed rerun command for automation and copy-paste
use. Decline, EOF, and unreviewed calibration still start no mapping.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 35 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This removes one misleading copy-paste detour without weakening calibration
review or automation. It is not a real first map, clean-host timing result,
paired external GLIM observation, or parity/superiority claim.

## Direct-to-progress confirmed-start follow-up — 2026-08-17

> Decision: **LOCAL_DIRECT_PROGRESS_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `2d0bb84a447e29b940adda4bd432e3d5725c9cc0`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

After the single calibration prompt, confirmed live `start` still rendered the
complete READY setup card and then a second start/progress card. Topics,
transforms, the delegated command, and the setup destination were therefore
repeated after the operator had already made the safety decision.

The confirmed path now skips that repeated review and enters the existing
start/progress card directly. Setup-only and dry-run output retain full selected
inputs, calibration, and delegated command detail because no execution follows.
An unconfirmed non-RKO path also retains that detail before its fail-closed
prompt. The durable `sensor_setup.json`, `session.json`, live progress,
verification, and recovery contracts are unchanged.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 36 passed |
| exact S3 lifecycle command | 71 passed |
| exact S3 edit/merge command | 15 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This removes duplicate terminal reading without hiding a pending decision or
changing automation. It is not a real first map, clean-host timing result,
paired external GLIM observation, or parity/superiority claim.

## Single-card map-completion follow-up — 2026-08-17

> Decision: **LOCAL_SINGLE_COMPLETION_CARD_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `8a620e54a121f5ac45913791b40b5239a59f5885`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

Successful `start` previously printed a completion block with setup, map, and
`Reopen`, then wrote session paths, then printed a second summary whose `Next`
usually repeated the reopen command. A viewer failure added another `Reopen
later` line. The operator therefore had to reconcile two or three terminal
handoffs after the map had already finished.

Terminal success is now one `Map session: VERIFIED` or `UNVERIFIED` card. It
contains the map output, evidence-backed verification state, viewer state,
session index/page, run manifest, first-map receipt, and exactly one recommended
`Next`. A verified card retains the privacy-safe `Share` action. Viewer failure
makes the single `Next` the view retry, emits one warning, and does not replace
verified map success. If the derived session index cannot be written, one
completed fallback card still preserves the map path and view command.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 37 passed |
| exact S3 lifecycle command | 72 passed |
| exact S3 edit/merge command | 15 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This turns map completion into one decision point without changing structured
session, progress, verification, or recovery evidence. It is not a real first
map, clean-host timing result, paired external GLIM observation, or
parity/superiority claim.

## One-action failed-map recovery follow-up — 2026-08-17

> Decision: **LOCAL_ONE_ACTION_RECOVERY_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `14081ea101744b868b80d900bb5a1c42b4ad5046`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

The previous ACTION REQUIRED terminal card printed every finding message and
action, the primary `next_command` again, a safe retry, an inspect alternative,
and multiple evidence paths. Those are useful recovery records, but they made a
failed first map begin with several competing commands.

The default card is now bounded to the first stable reason, remaining finding
codes without secondary prose or actions, exactly one safe `Next`, and one
`Details` path. The detail preference is the human session page, then canonical
recovery JSON, session JSON, and finally the preserved setup bundle. Every
finding/action, retry, inspect command, evidence path, resume condition, and
fresh-output rule remains unchanged in `map_session_recovery.json` and the
derived session handoff.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 38 passed |
| exact S3 lifecycle command | 73 passed |
| exact S3 edit/merge command | 15 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This gives failed mapping one immediate repair step without deleting expert or
machine recovery evidence. It is not a real failure recovery, clean-host timing
result, paired external GLIM observation, or parity/superiority claim.

## Bounded long-stage heartbeat follow-up — 2026-08-17

> Decision: **LOCAL_BOUNDED_HEARTBEAT_PASS / PAIRED_PUBLIC_TRIAL_PENDING**
>
> Implementation tip:
> `e2043c0f324ba8fb855b3a723cb670acd40cb2ad`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

Previously, a delegated stage that legitimately ran for several minutes stayed
silent until its durable run-manifest stage changed. That preserved evidence
correctness but could make a healthy first map look stuck.

An unchanged non-complete stage now emits at most one terminal heartbeat every
30 seconds. It reuses the durable stage label and reports monotonic elapsed time
only. A heartbeat writes neither `session.json` nor `session.html`, and it emits
no percentage, ETA, or assertion that the delegated workflow advanced. Durable
stage transitions remain the sole progress-artifact update boundary.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 39 passed |
| exact S3 lifecycle command | 74 passed |
| exact S3 edit/merge command | 15 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This keeps long mapping visibly active without inventing progress or increasing
artifact churn. The test uses a mocked monitor clock and stage source; it is not
a real long mapping run, clean-host timing result, paired external GLIM
observation, or parity/superiority claim.

## Safe operator interruption follow-up — 2026-08-17

> Decision: **LOCAL_SAFE_INTERRUPTION_PASS / REAL_MAPPING_TRIAL_PENDING**
>
> Implementation tip:
> `6d1249e45a2c91d3b5794f2c6f65eebf19336299`
>
> Real mapping, network, GitHub, release, or community mutations performed:
> **none**

The lower map runner already isolated its ROS workflow, forwarded SIGINT and
SIGTERM, reaped that process group, and wrote an interrupted terminal manifest.
The product-level `start` still delegated through `subprocess.run`, so its own
KeyboardInterrupt could escape before that lower cleanup and evidence sealing
finished.

`start` now supervises the delegated runner in a separate session. One Ctrl-C
is forwarded as SIGINT, then the product waits up to 20 seconds for bounded
cleanup and terminal evidence. If needed, it requests termination, waits 10
more seconds, and force-reaps the delegated runner. SIGTERM enters the same
supervision boundary. The resulting non-zero status flows through the existing
`workflow-interrupted` recovery receipt, session page, one `Next`, and one
`Details` path; no verified success or Python traceback is synthesized.

A real process-level regression starts a synthetic 60-second delegated child,
sends SIGINT only to the product supervisor, and verifies exit 130 plus an
absent child PID. This exercises signal forwarding and reap behavior without
running ROS mapping.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete sensor-setup wizard regressions | 42 passed |
| exact S3 lifecycle command | 77 passed |
| exact S3 edit/merge command | 15 passed |
| complete lower map-runner regressions | 48 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 331 passed |
| changed-code Jazzy `ament_flake8` | PASS |
| strict MkDocs, JSON, bytecode, plan, and patch hygiene | PASS |

This turns one stop request into a bounded evidence-preserving handoff. It is
not a real interrupted mapping trial, clean-host timing result, paired external
GLIM observation, or parity/superiority claim.

## Real ROS interruption correction and trial — 2026-08-17

> Decision: **REAL_ROS_INTERRUPTION_PASS / DEFAULT_STORAGE_AND_CLEAN_HOST_PENDING**
>
> Corrected implementation tip:
> `0301a0d269db4f45b3c8471d3cbf6622e9124337`
>
> Original synthetic-only implementation tip:
> `6d1249e45a2c91d3b5794f2c6f65eebf19336299`
>
> Local ROS mapping performed: **yes, interruption trial only**
>
> Network, GitHub, release, or community mutation performed: **none**

The first real ROS attempt at clean local tip `a24879c…` found a gap that the
synthetic child probe did not cover. Terminal Ctrl-C reached the outer stable
CLI and its `subprocess.run` dispatcher killed the `start` helper. The inner
runner process then received SIGINT without its complete descendant group, so
the ROS workflow survived to a successful map while `session.json` remained
`running`. The terminal contained a Python `KeyboardInterrupt` traceback and no
recovery receipt. The retained red-run hashes bind stdout `63a4f808…`, stderr
`ffb2e720…`, stale session `ac9d2681…`, and successful lower manifest
`f7cb081f…`; that map is test output, not trusted interruption evidence.

Correction `0301a0d…` replaces the stable dispatcher's `subprocess.run` with a
wait-and-forward supervisor. The outermost CLI owns an isolated command group;
nested CLI dispatch waits for the already-signalled group instead of sending a
duplicate signal. The `start` helper now sends SIGINT, SIGTERM, and final
SIGKILL to the complete delegated group rather than only its leader. A new real
dispatcher regression sends SIGINT only to the supervisor and verifies that
both the delegated process and its descendant are reaped without traceback.

The green trial used ROS 2 Jazzy, the same public 50-second MID360 fixture ZIP
(`20e51517…`, 98,873,952 bytes), bag metadata `d866804b…`, and sqlite storage
`0a38fbcc…`. The `lidarslam` copy install was clean exact `0301a0d…`; the
graph/scanmatcher overlay was exact ancestor `d8d2eab…` with no runtime-source
change through the tested tip, and the RKO-LIO source was unchanged from base
`3f4dd70…`. This controlled overlay is not a clean-host package-manager result.

After `workflow_running` and the live ROS process tree were observed, one
terminal Ctrl-C returned 130 in 1.5 seconds. The durable run covers 11.807
seconds from start to sealed manifest and ends `interrupted / complete`, with
`map workflow interrupted by SIGINT`, required verification not completed, and
no completed `map.pcd` or Lanelet2 geometry. All trial descendants were absent.
The final terminal projection contains exactly one `ACTION REQUIRED`, one
`Next`, one `Details`, and one stop request, with no traceback, VERIFIED, or
UNVERIFIED card.

The run manifest, recovery receipt, session index, and first-map receipt each
validate against their tracked schemas. All 17 manifest-bound artifact sizes
and SHA-256 values revalidate. The final stdout, stderr, session, manifest, and
recovery hashes are respectively `ff635663…`, `46d24440…`, `ef17c7b…`,
`8beb0907…`, and `657660d5…`.

Verification on the corrected tip:

| Check | Result |
| --- | --- |
| CLI home plus complete sensor-setup regressions | 51 passed |
| exact S3 lifecycle command | 77 passed |
| complete lower map-runner regressions | 48 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 332 passed |
| four terminal JSON schemas and 17 artifact checksums | PASS |
| changed-code Jazzy `ament_flake8`, JSON, bytecode, and patch hygiene | PASS |

The host had only 1.58 GiB free, so this bounded trial explicitly used a 0.5
GiB floor. The unchanged default storage check correctly remained unsatisfied;
this result is not permission to lower that default. It also is not a complete
map, map-quality or accuracy result, clean-host timing result, independent
first map, paired GLIM observation, or parity/superiority claim.

## Concise expected-stop follow-up — 2026-08-17

> Decision: **REAL_ROS_CONCISE_INTERRUPTION_PASS / DEFAULT_STORAGE_AND_CLEAN_HOST_PENDING**
>
> Implementation tip:
> `181b25121ba328beeb81cc772a60cce0c9a7d82f`
>
> Network, GitHub, release, or community mutation performed: **none**

The corrected `0301a0d…` trial proved safe process and evidence convergence,
but its expected Ctrl-C still produced a false offline-timeout label and a
120-line recent-launch-log dump. That stderr was 9,223 bytes even though the
operator, not a 900-second deadline, ended the run.

The dogfood shell now reserves `EXIT` for ordinary cleanup and handles SIGINT
and SIGTERM through explicit 130 and 143 exits. Its signal handler disables
keep-running behavior, reaps loggers and the isolated ROS launch group, and
exits without returning to the genuine timeout branch. The real startup and
offline-completion timeout diagnostics remain unchanged. Process-level probes
exercise both signals with a fake isolated launch group and verify the group is
absent, `/map_save` is not called, and neither timeout text nor a launch-log
excerpt is emitted.

A second exact-tip Jazzy trial used the same fixture identities and controlled
overlay as the prior trial. After the real RKO-LIO and graph_based_slam process
tree reached `Waiting for offline bag playback to finish ...`, one Ctrl-C again
returned 130 in 1.5 seconds. Every descendant was absent; the manifest remained
`interrupted / complete`, the session and recovery receipt remained
`action_required / workflow-interrupted`, and the terminal retained one stop
request, ACTION REQUIRED, Next, and Details with no success card or traceback.

The expected-stop stderr fell from 9,223 to 1,845 bytes, an 80.00% reduction.
The false timeout label and recent-launch-log dump each fell to zero. The run
manifest, recovery receipt, session index, and first-map receipt validate
against their schemas, and all 17 artifact sizes and SHA-256 values revalidate.
The new stdout, stderr, session, manifest, and recovery hashes are respectively
`ca03803c…`, `93fcb8de…`, `5d2e8354…`, `90a9d49d…`, and `c1c47075…`.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| dogfood shell, including real SIGINT and SIGTERM process groups | 14 passed |
| complete lower map-runner regressions | 48 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 332 passed |
| four terminal JSON schemas and 17 artifact checksums | PASS |
| shell syntax, changed-code `ament_flake8`, strict MkDocs, plan, JSON, and patch hygiene | PASS |

This is a clearer expected-stop path, not weaker timeout diagnosis. The trial
again used an explicit 0.5 GiB floor because the default storage requirement
was unavailable. It is not a complete map, clean-host or package-manager
result, map-quality or accuracy result, paired GLIM observation, or
parity/superiority claim.

## Default-storage interruption follow-up — 2026-08-17

> Decision: **REAL_ROS_DEFAULT_STORAGE_INTERRUPTION_PASS / CLEAN_HOST_PENDING**
>
> Tested installed revision:
> `edff76df06e7a7c86a6adbde454270664ee4d126`
>
> Runtime implementation revision:
> `181b25121ba328beeb81cc772a60cce0c9a7d82f`
>
> Network, GitHub, release, or community mutation performed: **none**

Two clean Git worktrees retained 5.4 GiB of ignored compiler intermediates.
Deleting only those two reproducible `build/` trees raised root free capacity
from 1.9 GiB to 7.3 GiB. Source, branches, installed overlays, and retained
evidence were unchanged. A fresh non-symlink `lidarslam` install then recorded
exact revision `edff76d…`, `dirty=false`; the four interruption-critical
installed scripts were byte-identical to that source revision.

The trial reused the public 50-second MID360 fixture ZIP (`20e51517…`,
98,873,952 bytes), bag metadata `d866804b…`, and sqlite storage `0a38fbcc…`.
The top-level `start` command omitted `--min-free-space-gib`; its generated
plan reported **7.01 GiB free / 5.00 GiB required** and delegated the unchanged
5.0 GiB default. The graph/scanmatcher overlay remained exact ancestor
`d8d2eab…`, with no runtime-source diff through the tested revision, and the
RKO-LIO source remained the existing local base. This is still a controlled
mixed overlay, not a clean-host or package-manager installation.

A preliminary non-overwriting run completed before the delayed stop input and
is excluded from interruption evidence. The accepted fresh-output run observed
both the real RKO-LIO offline node and graph_based_slam node during
`workflow_running`, then received one terminal Ctrl-C. It returned 130 in
1.43 seconds. The sealed manifest spans 14.001 seconds, ends `interrupted /
complete`, records `map workflow interrupted by SIGINT`, and every trial
descendant is absent.

The session and recovery receipt end `action_required / workflow-interrupted`;
required verification is unavailable, the first-map receipt is FAIL as
expected, and no `map.pcd` exists. The terminal contains exactly one stop
request, ACTION REQUIRED, Next, and Details, with zero false timeout labels,
recent-launch-log dumps, tracebacks, or success cards. The run manifest,
recovery receipt, session index, and first-map receipt validate against their
tracked schemas, and all eight artifacts sealed before this earlier
interruption revalidate by size and SHA-256.

The final stdout, stderr, session, manifest, and recovery hashes are
respectively `dd2e6df0…`, `019449f4…`, `78902668…`, `947c779a…`, and
`958d6d15…`. This closes only the unchanged default-storage interruption gate.
It is not a complete-map quality or accuracy result, clean-host timing result,
independent first map, paired GLIM observation, or parity/superiority claim.

## Evidence-bound concise stop summary — 2026-08-17

> Decision: **REAL_ROS_CONCISE_STOP_SUMMARY_PASS / CLEAN_HOST_PENDING**
>
> Implementation and tested installed revision:
> `8370ac511f29eaf3861569103cd5389035c7412a`
>
> Network, GitHub, release, or community mutation performed: **none**

The default-storage trial still left three redundant post-stop diagnostics:
the lower runner's generic failure, its repeated full internal command, and the
session wrapper's generic needs-attention error. They duplicated the preceding
stop request and following schema-backed ACTION REQUIRED card.

The lower runner now shortens output only when the sealed manifest agrees on
all five interruption facts: `status=interrupted`, matching execution and
runner exit codes, `stage=complete`, and the exact terminating signal in
`last_error`. Direct `run` then prints one human stop label and retained-output
path. A process that merely returns 130, or any missing, malformed, incomplete,
or inconsistent manifest, keeps the prior generic error and full failed
command. `start` suppresses only its duplicate generic error for the derived
`workflow-interrupted` reason; every other recovery reason is unchanged.

A fresh non-symlink Jazzy install recorded exact revision `8370ac5…`,
`dirty=false`, and its four interruption-critical scripts matched source
bytes. The run reused the same public fixture identities and controlled
graph/scanmatcher and RKO-LIO overlay. The top-level command again omitted the
storage override and passed **7.01 GiB free / 5.00 GiB required**. After both
the real RKO-LIO offline node and graph_based_slam node were observed during
`workflow_running`, one terminal Ctrl-C returned 130 in 1.38 seconds. The
14.934-second manifest ends `interrupted / complete`; all descendants are
absent.

Post-stop stderr now contains one stop request and one concise
`Map stopped by Ctrl-C; retained evidence: ...` line. Generic map failure,
`failed command`, generic session error, false timeout, recent-launch-log dump,
traceback, and success-card counts are all zero. The complete stderr fell from
1,971 to 1,200 bytes versus the immediately preceding default-storage trial, a
39.12% reduction; versus the original false-timeout trial it is 86.99% lower.
The stdout projection retains exactly one ACTION REQUIRED, Next, and Details.

The run manifest, recovery receipt, session index, and first-map receipt again
validate against their tracked schemas. All eight early-interruption artifact
sizes and SHA-256 values revalidate, required verification remains not
completed, and no `map.pcd` exists. The final stdout, stderr, session,
manifest, and recovery hashes are respectively `178e6632…`, `93f9374a…`,
`0ebdecd5…`, `5633af61…`, and `18678576…`.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete lower map-runner regressions | 49 passed |
| exact S3 lifecycle and edit/merge commands | 77 + 15 passed |
| dogfood shell process/signal regressions | 14 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 332 passed |
| four terminal JSON schemas and eight artifact checksums | PASS |
| changed-code `ament_flake8`, strict MkDocs, plan, bytecode, JSON, and patch hygiene | PASS |

This is an operator-facing stop-summary improvement, not weaker failure or
timeout diagnosis. The controlled mixed overlay still prevents a clean-host or
package-manager claim, and this is not a complete-map quality or accuracy
result, independent first map, paired GLIM observation, or parity/superiority
claim.

## Quiet native bag-reader follow-up — 2026-08-17

> Decision: **REAL_ROS_QUIET_READER_PASS / CLEAN_HOST_PENDING**
>
> Implementation and tested installed revision:
> `d0e33613f4531988ac4fc3ac0687927d164690eb`
>
> Network, GitHub, release, or community mutation performed: **none**

The concise-stop trial's only remaining stderr noise was six native
`rosbag2_storage` INFO lines announcing read-only database opens. Those lines
were routine, but globally redirecting or discarding stderr would also hide
warnings and storage failures.

All four `rosbag2_py.SequentialReader` sites now use one open helper. During a
product-dispatched open only, and only when the exact storage logger has no
explicit level, a scoped boundary raises `rosbag2_storage` to WARN and restores
the previous level immediately after open. A direct preflight does not enter
the boundary; an explicitly configured DEBUG or INFO level is not replaced;
WARN, ERROR, and FATAL remain above the threshold. Focused native-logger tests
prove routine INFO is absent, a warning remains visible, and direct and
explicit levels are unchanged.

A real two-message bag probe made the expert/product distinction measurable.
Both direct preflight and product `doctor --json` returned 0 and emitted the
same 5,661-byte stdout. Direct preflight retained two storage INFO lines in 294
stderr bytes; the product command emitted zero stderr bytes.

A fresh non-symlink Jazzy install then recorded exact revision `d0e3361…`,
`dirty=false`, directly from Git without a revision override. The four critical
installed scripts matched source SHA-256 values, and the complete installed
product validator passed. The run reused the same public 50-second MID360 ZIP
(`20e51517…`), metadata (`d866804b…`), sqlite database (`0a38fbcc…`), fixed
graph/scanmatcher overlay, and existing RKO-LIO base as the prior trial. It is
therefore a controlled mixed overlay, not a clean-host or package-manager run.

The top-level command again omitted `--min-free-space-gib` and reported **7.02
GiB free / 5.00 GiB required**. After the real RKO-LIO offline node and
graph_based_slam node were simultaneously observed at the offline wait, one
Ctrl-C returned 130 in 1.545 seconds. The complete command took 15.665 seconds;
the sealed manifest spans 12.356 seconds and ends `interrupted / complete` with
matching execution and runner exit code 130 and exact SIGINT error. Both
observed PIDs and every trial descendant are absent.

The six routine storage INFO lines fell to zero. Stderr now contains only one
stop request and one concise retained-evidence line: 180 bytes versus 1,200 in
the preceding concise-stop trial, an 85.00% reduction. Generic map failure,
repeated failed command, generic session error, false timeout, recent-log dump,
traceback, and success-card counts remain zero. Stdout retains exactly one
ACTION REQUIRED, Next, and Details.

The run manifest, recovery receipt, session index, and first-map receipt
validate against the installed tracked schemas. All 18 manifest-bound artifact
sizes and SHA-256 values revalidate. Required verification remains incomplete,
the receipt is FAIL as expected, and no `map.pcd` exists. The final stdout,
stderr, session, manifest, and recovery hashes are respectively `8ba436f3…`,
`4ee8f77a…`, `3dd47d8a…`, `89e008a0…`, and `16a35088…`.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| native logging boundary and complete preflight regressions | 36 passed |
| complete lower map-runner regressions | 49 passed |
| exact S3 lifecycle and edit/merge commands | 77 + 15 passed |
| dogfood shell process/signal regressions | 14 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 332 passed |
| four terminal JSON schemas and 18 artifact checksums | PASS |
| exact install provenance/script identity, changed-code `ament_flake8`, strict MkDocs, bytecode, JSON, and patch hygiene | PASS |

This closes only routine native reader noise on the product path. It does not
hide warnings or failures, change direct expert logging, weaken storage or
timeout diagnosis, complete a map, establish clean-host/package-manager timing,
or provide paired GLIM, parity, superiority, quality, or accuracy evidence.

## Concise guided startup follow-up — 2026-08-17

> Decision: **REAL_ROS_CONCISE_START_PASS / CLEAN_HOST_PENDING**
>
> Implementation and tested installed revision:
> `3dcca0c75c25565cc244207711bb45a56beee38a`
>
> Network, GitHub, release, or community mutation performed: **none**

The guided `start` path previously composed three independently useful output
streams: the sensor wizard's product progress, the lower runner's execution
plan and lifecycle, and the dogfood shell's expert launch details. That was
complete but made first use read like an implementation trace.

The sensor wizard now marks only its delegated child with a concise product
session environment. The runner reduces its child summary to the selected map
profile and the unchanged storage decision, and the dogfood layer emits one
friendly mapping-ready message instead of its expert preamble. The parent
continues to own durable progress, the terminal recovery card, and the bounded
Ctrl-C cleanup. Direct `run` and direct dogfood calls retain their complete
expert output. Warnings, errors, real startup/offline-completion timeout
diagnostics, manifests, recovery receipts, and logs are not filtered.

A first exploratory run on `39d7128…` found two measurement defects: its broad
process-name selector could match the barrier shell, and piped stdout exposed
late profile/storage flushes. That run is excluded from final evidence. The
selector was restricted to exact executable `argv[0]` basenames and the two
concise summary lines were explicitly flushed before the final trial.

The final fresh non-symlink Jazzy install recorded exact revision `3dcca0c…`,
`dirty=false`, directly from Git without a revision override. Source and
installed SHA-256 values matched for the sensor wizard (`afee67ca…`), lower
runner (`096bf8be…`), and dogfood shell (`7b3bf6a4…`), and the complete
installed-product validator passed. It reused the same public 50-second
MID360 ZIP (`20e51517…`), metadata (`d866804b…`), sqlite database
(`0a38fbcc…`), fixed graph/scanmatcher overlay, and existing RKO-LIO base as
the preceding trial, so it remains a controlled mixed overlay.

The top-level installed command omitted `--min-free-space-gib` and reported
**6.99 GiB free / 5.00 GiB required**. Map profile and storage appeared before
mapping readiness. After the exact RKO-LIO offline node and
graph_based_slam node were present for three consecutive polls, one SIGINT
returned 130 in 1.543 seconds; total wall time was 15.731 seconds. Both
observed PIDs and every trial descendant are absent.

Guided stdout is **23 lines / 1,448 bytes**, down from 3,679 bytes in the
immediately preceding quiet-reader exact trial: a **60.64% reduction**. It
retains one start card, profile, storage decision, progress stages 1/2/5,
friendly mapping-ready line, diagnosis/receipt links, and exactly one ACTION
REQUIRED / Next / Details card. It omits the delegated command, selected
output and atomic paths, YAML/parameter paths, topics, frames, staging
promises, duplicate lifecycle announcements, old SLAM/offline-wait messages,
and launch-log location. Stderr is 181 bytes and contains only the stop request
and retained-evidence line; routine storage INFO, generic failure, failed
command, generic session error, false timeout, recent-log dump, traceback, and
success cards remain absent.

The manifest is `interrupted / complete`, with matching execution and runner
exit code 130 and exact SIGINT error. The run manifest, recovery receipt,
session index, and first-map receipt validate against the installed tracked
schemas; all 18 manifest-bound artifact sizes and SHA-256 values revalidate.
No `map.pcd` exists. Final stdout, stderr, session, manifest, and recovery
SHA-256 values are respectively `eae14b3e…`, `10b82cf6…`, `030be591…`,
`8968e396…`, and `109a3d50…`.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| guided start / complete sensor-setup regressions | 42 passed |
| complete lower map-runner regressions | 50 passed |
| dogfood shell process/signal regressions | 15 passed |
| exact S3 lifecycle and edit/merge commands | 77 + 15 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 332 passed |
| four terminal JSON schemas and 18 artifact checksums | PASS |
| exact install provenance/script identity, changed-code `ament_flake8`, strict MkDocs, shell/bytecode/JSON/patch hygiene | PASS |

This is a guided-terminal usability improvement, not reduced diagnosis. It
does not change direct expert output, storage policy, runtime behavior, map
quality, or evidence authority, and it does not establish clean-host or
package-manager timing, a complete map, an independent first map, paired GLIM
observation, parity, superiority, quality, or accuracy.

## Complete guided-map follow-up — 2026-08-17

> Decision: **REAL_ROS_GUIDED_COMPLETION_PASS / CLEAN_HOST_PENDING**
>
> Implementation and tested installed revision:
> `8e67ab7f50bb78767b4bc7674137eb4ecdf3e16b`
>
> Network, GitHub, release, or community mutation performed: **none**

The preceding concise-start trial intentionally stopped during mapping. A
complete baseline on exact clean tip `113f588…` then exposed the remaining
success-path gap: mapping and verification passed, but the dogfood shell's map
save, trajectory, Lanelet2, and bundle messages plus the lower runner's old
multi-action completion block returned after the friendly ready line. The
guided terminal expanded to 51 lines / 3,791 bytes even though the mocked
completion card regressions were green.

The lower runner now captures child workflow stdout line by line only for the
parent-owned concise session. It durably flushes every line to
`map_workflow.log`, relays only the two exact mapping-ready status messages,
and leaves stderr live. A normal nonzero child exit replays the most recent 80
stdout lines plus the complete-log path; SIGINT/SIGTERM retain the established
quiet stop contract. Successful lower-runner Next/diagnosis repetition is
also withheld because the parent card already owns those fields. Direct
expert `run` and direct dogfood output are unchanged.

Focused regressions prove successful retention without terminal leakage,
bounded failure replay, parent-owned successful summary, and both ordinary and
concise SIGTERM process-group cleanup. The new workflow log is included in the
manifest checksum set, so convenience does not erase diagnosis.

A fresh non-symlink Jazzy install recorded exact revision `8e67ab7…`,
`dirty=false`, directly from Git without a revision override. Source and
installed SHA-256 matched for the runner (`13f68e9e…`), sensor wizard
(`afee67ca…`), and dogfood shell (`7b3bf6a4…`); the complete installed-product
validator passed. The run reused the exact prior extracted 50-second MID360
bag with metadata `d866804b…` and sqlite `0a38fbcc…`, the fixed
graph/scanmatcher overlay, and the existing RKO-LIO base. It remains a
controlled mixed overlay, not a clean-host or package-manager result.

The installed top-level `start` omitted `--min-free-space-gib`, reported **6.78
GiB free / 5.00 GiB required**, and completed in **23.45 seconds** with peak RSS
134,228 KiB. The terminal is **23 lines / 1,399 bytes**, down from 51 lines /
3,791 bytes in the exact baseline: a **63.10% byte reduction**. Stderr is empty.
Profile/storage precede mapping readiness, and the run ends in exactly one
VERIFIED, Next, and Share. Delegated commands, output/YAML/topic/frame
internals, map-save and trajectory detail, Lanelet2 generator output, bundle
inventory, lifecycle duplication, launch-log location, and the old Next steps
are absent from the terminal.

The checksum-bound `map_workflow.log` retains all 16 hidden post-process lines
in 1,187 bytes. The final manifest is `succeeded / complete`, with execution
and runner exit code 0. The privacy-bounded receipt passes all 7 / 7 checks,
the session index is `verified / PASS`, the partial directory is absent, and
the installed sensor-setup, session-index, run-manifest, and receipt schemas
all validate. Every one of 124 manifest-bound artifact sizes and SHA-256
values revalidates, including the workflow log.

The output contains 500 raw poses, 88 corrected poses, 92 pointcloud tiles
totalling 4,037,472 bytes, a 4,015,933-byte `map.pcd` with SHA-256
`987637f9…`, and a 63,458-byte Lanelet2 map with SHA-256 `25280265…`. The
baseline and corrected runs produced byte-identical `map.pcd`, Lanelet2, raw
trajectory (`9f68345a…`), and corrected trajectory (`8dd2acdf…`) files. The
terminal, retained workflow log, session, manifest, and receipt hashes are
respectively `0cb8e9ba…`, `47084fc8…`, `40448657…`, `eb9ffb24…`, and
`a5ef1f39…`.

The same exact install then exercised interruption after both real nodes and
the friendly ready marker were stable for three polls. One SIGINT returned 130
in **1.317 seconds** and every observed node and trial descendant is absent.
The retained evidence is `interrupted / complete / 130 / 130 / SIGINT`, five
installed schemas and all 19 checksums pass, no `map.pcd` exists, and the
terminal keeps one ACTION REQUIRED / Next / Details with no traceback or false
timeout.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| complete lower map-runner regressions | 54 passed |
| complete sensor-setup regressions | 42 passed |
| dogfood shell process/signal regressions | 15 passed |
| exact S3 lifecycle and edit/merge commands | 77 + 15 passed |
| exact S2 first-map commands | 43 + 43 passed |
| exact S6 graph docs/product command | 42 passed |
| support and installed-product contract | 25 passed |
| option contract | 21 passed |
| broad S6 product/growth command | 332 passed |
| G0 readiness and publication-plan regressions | 25 + 34 passed |
| successful four-schema / 124-checksum and interrupted five-schema / 19-checksum audits | PASS |
| exact install provenance/script identity, changed-code `ament_flake8`, strict MkDocs, bytecode/JSON/patch hygiene | PASS |

This proves a real local guided completion and a regression-safe concise
boundary. It does not make the unpublished local fixture public, establish a
clean-host/package-manager or external-user timing result, replace the full
277-second proof gate, assess map accuracy, or provide paired GLIM,
parity/superiority, adoption, quality, or community evidence.

## Explicit first-map report follow-up — 2026-08-17

> Decision: **READ_ONLY_REPORT_UX_PASS / EXTERNAL_FIRST_MAP_PENDING**
>
> Implementation and tested installed revision:
> `cb2218fc24861088526bd2373bed3376218beb94`
>
> Network, GitHub, release, or community mutation performed: **none**

The verified completion and history UI previously called the local
`support --first-map` preparation command `Share`. Although the command was
already fail-closed and read-only, that label could imply an automatic upload
or issue submission. The user-facing action is now `Report:` / **Prepare a
first-map report**. The internal action kind remains `share` for structured
compatibility, and the `--json` schema is unchanged.

The human report is reduced from the synthetic verified-fixture baseline of
30 lines / 1,606 bytes to 22 lines. It leads with **READY FOR REVIEW**, the
issue URL, and the only attachment after review; then presents the copy fields,
three operator-completed fields, and two concise privacy lines. Redundant
product/source/profile repetition, Markdown fences, and the readable receipt
path are removed from default output without removing structured evidence.

A fresh non-symlink Jazzy install recorded exact revision `cb2218f…`,
`dirty=false`. Using the same controlled graph/scanmatcher overlay and original
50-second MID360 fixture, installed `start` completed in **23.42 seconds** with
peak RSS 135,056 KiB and empty stderr. Its 23-line / 1,388-byte completion has
exactly one VERIFIED, Next, and Report and zero Share labels. The final state is
`succeeded / complete / 0 / 0`, the receipt is 7 / 7 PASS, and session quality
is `verified / pass`.

The map remains 4,015,933 bytes (`987637f9…`), Lanelet2 remains 63,458 bytes
(`25280265…`), and 92 tiles total 4,037,472 bytes. Raw (`9f68345a…`) and
corrected (`8dd2acdf…`) trajectory hashes also match the preceding complete
run, so the reporting-only change did not alter mapping output.

The exact displayed `Report:` command returned 0 with **22 lines / 1,279
bytes** and empty stderr. Human and JSON invocations validate against the
installed first-map handoff, session index, run manifest, and receipt schemas;
all 124 manifest-bound artifact sizes and SHA-256 values revalidate. A complete
session path/type/size/mtime snapshot is byte-identical before and after both
reads, no archive appears, and a network-only `strace` records no socket,
connect, send, or receive syscall. The output identifies the issue form,
reviewed JSON receipt, release ref, environment hints, verification summary,
operator-supplied fields, and privacy boundary without performing the handoff.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| focused support / sensor setup / session history | 17 + 42 + 11 passed |
| exact S3 lifecycle and edit/merge commands | 77 + 15 passed |
| support and installed-product contract | 25 passed |
| exact docs/product and option contract | 42 + 21 passed |
| broad S6 product/growth command | 332 passed |
| publication-plan and G0 readiness regressions | 34 + 25 passed |
| four installed schemas / 124 manifest checksums | PASS |
| session path/type/size/mtime unchanged; no archive or network syscall | PASS |
| changed-code `ament_flake8`, strict MkDocs, bytecode/JSON/patch hygiene | PASS |

This closes a wording and review-density gap only. It does not publish the
fixture, upload the receipt, create an issue, establish an independent external
first map, provide a clean-host/package-manager or paired GLIM result, or grant
remote-write authority.

## First-class first-map report command — 2026-08-17

> Decision: **SHORT_REPORT_COMMAND_PASS / EXTERNAL_FIRST_MAP_PENDING**
>
> Implementation and tested installed revision:
> `e15ddab85d44a9aba7105667ab7b175cd655c271`
>
> Network, GitHub, release, or community mutation performed: **none**

The safe report handoff previously required users to remember that first-map
reporting was a mode of the broader support-ZIP command. Verified completion,
session history, top-level help, Bash completion, English and Japanese guidance,
and the machine-readable CLI contract now expose one purpose-named command:
`lidarslam-map report SESSION`. The older
`lidarslam-map support SESSION --first-map` spelling remains supported.

`report --help` exposes only the session, help, and read-only `--json` option.
ZIP-specific `--output` and the implementation detail `--first-map` are absent.
Internally the stable CLI selects the same support helper under a bounded child
mode, so receipt revalidation, schema, failure codes, privacy text, issue URL,
and no-write behavior have one implementation rather than a fork. The existing
structured action kind remains `share` for compatibility.

A fresh non-symlink Jazzy install records exact revision `e15ddab…` and passes
the complete installed-product CLI validator, including every full-help option
inventory and the new report handoff. On the retained schema-valid verified
visual fixture, installed `report` returns 0 with **22 lines / 1,327 bytes** and
empty stderr. Its human output SHA-256 is `7a199528…`; the schema-valid JSON
SHA-256 is `dcbc1afc…`. Both outputs are byte-identical to the legacy spelling.

A complete fixture path/type/size/mtime snapshot is unchanged across new and
legacy human/JSON reads. No support archive appears. A network-only `strace`
records no socket, connect, send, or receive syscall. This exact test does not
rerun mapping: the command changes dispatch and presentation only, while the
preceding exact real-map evidence remains the mapping-quality authority.

Verification on the implementation tip:

| Check | Result |
| --- | --- |
| combined focused report/setup/history/CLI/docs set | 175 passed |
| exact S3 lifecycle and edit/merge commands | 78 + 15 passed |
| exact S6 docs/product command | 42 passed |
| support and installed-product contract | 26 passed |
| broad S6 product/growth command | 333 passed |
| first-map validator cohort | 33 passed |
| publication-plan and G0 readiness regressions | 34 + 25 passed |
| exact installed full CLI validator / handoff schema | PASS |
| new-versus-legacy human and JSON byte comparison | IDENTICAL |
| fixture unchanged; no archive or network syscall | PASS |
| changed-code `ament_flake8`, strict MkDocs, bytecode/JSON/patch hygiene | PASS |

This closes a command-discovery and typing gap only. It does not publish the
fixture, map a bag, open a browser, upload a receipt, create an issue, establish
an independent external first map, or grant remote-write authority.
