# CLI compatibility and option policy

`lidarslam-map` is the supported product command. This page defines which
parts of its command line are intended to remain compatible through v1 and
how provisional and deprecated options are moved without surprising existing
users.
The machine-readable inventory is
[`contracts/cli-v1.json`](contracts/cli-v1.json).
It records not only option names and visibility, but also every value-taking
option's type, metavar, bounded choices, default behavior, units and numeric
constraints. The contract also describes positional-directory contents.

## Product surface

On a terminal with interactive stdin and stdout, invoking `lidarslam-map`
without a command opens a small home that routes to the existing `demo`,
`start`, `sessions`, or read-only `doctor` workflow. It prints the delegated
command before it can run and requires an explicit `yes` before demo download
or writes. Doctor needs no confirmation because it uses no network and writes
no files. This is a choice-reducing front door, not a new command contract. In
non-interactive use, no arguments remain a usage error printed to stderr with
exit code `2`, so an existing script never starts prompting after an upgrade.

The beginner workflow is one orchestration command:

```bash
lidarslam-map start <rosbag2_dir>
```

Automation retains the three explicit lifecycle commands:

```bash
lidarslam-map doctor [rosbag2_dir]
lidarslam-map run <rosbag2_dir> --output-dir <dir>
lidarslam-map inspect <output_dir>
```

Omitting `rosbag2_dir` checks product runtime files, prefix activation,
Humble/Jazzy, the ROS CLI and bag reader, and fixed-demo storage. Providing the
directory preserves the existing bag compatibility preflight. The system JSON
report is governed by `system-doctor-v1.schema.json` and omits local paths. A
storage rejection includes exact `additional_bytes_required`, a rounded-up
human shortage, and the placeholder-free retry command. Multiple system
findings retain their individual recovery text, while top-level `next_action`
and the human **Do this now** card select exactly one dependency-ordered first
recovery; rerunning doctor reprioritizes the remaining findings. Bag preflight
v6 checks bounded recorded Odometry parent/child frames against the bag's
dynamic TF graph, then reports replay-order startup and future-TF gaps for the
selected PointCloud2 topic. It does not claim that a transform will be fresh or
interpolatable at live runtime.

The full bag preflight retains its path and local commands for local automation.
Use `doctor <rosbag2_dir> --public-json` for a reviewed public issue; its
`public-doctor-evidence-v1` projection keeps only type/count/check/profile and
stable finding-code evidence and returns path-free input-error JSON as well.
The human bag report always ends with that shell-safe command for the exact
input and warns that the full report must remain local. Through the product
CLI, a ready report exposes one exact-input `lidarslam-map start` action and
hides lower-level launch alternatives. A report with findings withholds that
start action and instead prints the exact `doctor` retry after the first
finding is resolved. Direct use of the preflight script retains its detailed
developer commands.

The default product card is bounded to status, bag duration and message count,
input types without topic/frame names, selected profile, four check statuses,
the first finding plus remaining stable codes, and one action. It prints an
exact private `doctor ... --json` command for full local reasons and commands;
that JSON must not be shared. The compact card and detailed JSON are both
read-only and make no network request.

Viewing is an optional post-processing command, not another required mapping
step:

```bash
lidarslam-map view <output_dir> [--viewer autoware|foxglove]
```

Returning to local work and comparing two retained sessions are stable,
non-mutating commands:

```bash
lidarslam-map sessions [sessions_root]
lidarslam-map compare <left_session> <right_session>
```

Non-destructive `edit` and multi-session `merge` are also optional
post-processing commands. They publish new verified outputs and never mutate
the completed maps supplied as inputs.

Research scripts, benchmark runners, ROS launch arguments, and the historical
`ros2 run lidarslam lidarslam` node are outside this CLI contract.

The checkout/release-bundle Docker host launcher is delivery tooling around the
same `start` workflow, not an additional installed CLI command. Its additive
`--json` option is accepted only with `--dry-run` and emits the versioned
`docker-map-bag-plan-v1` plan to stdout without Docker, network, or filesystem
writes. Because that plan includes local paths, wrappers should keep it local
and should not paste raw output into issue reports.

## Stability labels

| Label | Promise |
| --- | --- |
| Stable | The name, accepted value shape, and meaning are v1 compatibility commitments. Additive choices are allowed when they do not change an existing invocation. |
| Provisional | The option works and is tested, but its location or spelling may change before v1. A replacement and compatibility alias must land before removal. |
| Deprecated | A documented replacement exists. The old spelling continues to work during its compatibility window and emits an actionable warning when exercised. |

During v0.9, a stable option cannot be silently renamed, removed, or assigned
a different default meaning. If a security or data-integrity defect requires
a behavior change, the release notes and command output must identify it.

After v1.0, removal of a stable option requires a major release. A deprecated
spelling must continue to work for at least one minor release, emit one
actionable warning to stderr, and preserve its previous exit-code behavior.
Automation must not parse warnings from stdout.

No current deprecated option has a scheduled removal release. The
machine-readable policy records `removal_status: not_scheduled`; choosing a
release later requires a reviewed contract and migration-guide change, not
only a parser edit.

JSON artifact compatibility is governed by the published JSON schemas, not by
this option policy or the repository version.

## Current inventory

| Command | Routine stable options | Advanced stable options | Deprecated options |
| --- | --- | --- | --- |
| `demo` | cache/output paths, `--viewer`, storage floor, `--dry-run`, `--resume`, `--json`, exclusive dry-run plan `--output` | None | None |
| `start` | sensor selection, calibration, output, `--yes`, `--dry-run`, `--editable`, `--viewer`, storage and verification | None | None |
| `setup` | profile, output, calibration and frame options, `--json` | None | None |
| `doctor` | `--json` | None | None |
| `run` | `--profile`, `--output-dir`, `--min-free-space-gib`, `--dry-run`, `--resume`, `--guided`, `--yes`, `--verification` | None | Viewer compatibility options and `--no-verify-map` |
| `inspect` | `--bag`, `--symptom`, `--json`, `--write` | None | None |
| `view` | `--viewer` | `--autoware-core-dir`, `--work-dir`, `--runtime-dir`, `--rebuild`, `--auto-exit-secs` | None |
| `sessions` | `--status`, `--limit`, `--viewer`, `--json` | None | None |
| `compare` | `--output`, `--viewer`, `--json` | None | None |
| `report` | `--json` | None | None |
| `support` | `--output`, `--json`, `--first-map` | None | None |
| `edit` | `--plan`, `--output-dir`, `--dry-run`, `--json` | `--backend-input`, `--params`, `--setup` | None |
| `merge` | output, alignment, acceptance, transform, dry-run, and JSON options | None | None |

Interactive `start` calibration review is one fail-closed prompt: it shows the
profile extrinsics once, then asks whether to continue without presenting a
second `--yes` command. Non-interactive `start`, `setup`, and dry-run review keep
printing the exact reviewed rerun command for automation and copy-paste use.
After confirmation, live `start` skips the repeated setup card and proceeds
directly to map start and durable progress. Setup-only and dry-run output retain
the selected inputs, calibration, and map command because no execution progress
follows them.

For `demo`, cache presence is not proof of integrity. Dry-run reports
`prepared_unverified` or `archive_unverified`; live execution alone promotes
the fixed data after registered archive and extracted-file SHA-256 checks.
`--resume` is narrower than `run --resume`: the demo wrapper exposes it only
for terminal post-processing stages and keeps mapping-active state fail-closed.

`-h`/`--help` and `--help-all` are stable for every command. Top-level
`--version` is also stable. No-argument behavior is mode-sensitive by contract:
TTY input and output select the bounded home, while captured or redirected
execution returns usage exit `2` without reading stdin.

Normal help is the operator view; it contains stable options needed for
routine use. Full help is the compatibility view:

```bash
lidarslam-map run --help
lidarslam-map run --help-all
```

`--help-all` adds advanced runtime controls and deprecated aliases. Hiding a
deprecated option from normal help does not remove it: the parser, completion,
machine-readable inventory and migration documentation retain it throughout
its compatibility window.

The positional names describe directories deliberately:

- `rosbag2_dir` is the directory containing `metadata.yaml`, never an
  individual `.db3` or `.mcap` storage file;
- `output_dir` is a map-run directory or its terminal bundle.

## Option tiers

The map-producing `run` options are ordered by operator intent:

1. **Core:** choose a profile and output directory.
2. **Lifecycle:** plan, reserve storage, or resume post-processing.
3. **Guided onboarding:** show the preflight decision and ask before a long
   human-operated run.
4. **Deprecated compatibility:** forward old viewer requests to `view`.
5. **Verification:** retain the required default or explicitly select the
   diagnostic-only `off` mode.

Viewer construction is not map construction. It is owned by the dedicated
`view` command, so a viewer failure does not make a completed map look like a
mapping failure. Existing `run --viewer ...` invocations route through the new
command and remain warning-emitting compatibility aliases during the
published deprecation window. They appear in `run --help-all`, not the normal
operator help.

The normal `view --help` contains viewer selection. Viewer build and runtime
controls remain stable but advanced, and are listed by `view --help-all`.

`--verification required` is the default. `--verification off` is a diagnostic
escape hatch, not a normal performance option, and emits a visible warning.
The old `--no-verify-map` name remains a warning-emitting compatibility alias
during the deprecation window. An unverified run is never described as a
verified success.

`start` adds only orchestration: it invokes the established sensor setup, map
runner, required verifier, and viewer contracts. Its default browser and
timestamped session directory are beginner defaults; the underlying algorithms
and profile values are unchanged. Calibration is shown before a positive
interactive confirmation, or accepted explicitly with `--yes`.
An incompatible own-bag input returns exit `2` and the versioned
`sensor-setup-rejection-v1` contract. It writes no output, keeps reason and
finding codes stable for automation, and includes human messages plus exact
next actions. A forced incompatible profile uses `profile-incompatible`; no
safe maintained path uses `no-maintained-profile`.
After delegation begins, `start` owns a distinct `map-session-recovery-v1`
handoff for non-zero map results. This additive artifact preserves diagnosis-v1
and run-manifest-v2 compatibility while giving people and automation stable
runtime/map-quality codes, evidence paths, and exact resume or fresh-output
retry commands. It is written beside the unchanged sensor setup manifest and
does not change or overwrite retained map evidence. Viewer-only failures remain
separate because the map workflow has already completed.

The default terminal projection is deliberately smaller than that artifact: it
shows the first reason, remaining stable codes, one exact `Next:`, and one
retained `Details:` path. Per-finding actions, retry and inspect alternatives,
and all evidence paths remain in the JSON and derived session page instead of
competing in the first repair step.

A live `start` also catches one operator Ctrl-C at the stable CLI boundary. The
dispatcher waits for the `start` helper instead of abandoning it, and the
helper signals the delegated runner's complete isolated process group. It waits
up to 20 seconds for bounded cleanup and terminal evidence, requests termination
for at most 10 more seconds when needed, then force-reaps the group if it still
has not stopped. The resulting non-zero runner state flows through the unchanged
one-action recovery contract instead of escaping as a Python traceback;
verified success is never synthesized. An expected SIGINT or SIGTERM is not
relabeled as an offline-completion timeout and does not dump the recent launch
log; genuine startup and completion timeouts keep their diagnostics.

Every delegated `start` also owns an additive `map-session-index-v1` contract:
`session.json` and its derived `session.html` represent `running`, `verified`,
`unverified`, or `action_required` through one stable location. Running progress
mirrors atomic run-manifest-v2 stages and adds no estimated-time guarantee.
An unchanged live stage emits at most one terminal heartbeat every 30 seconds
with monotonic elapsed time. The heartbeat performs no session artifact write
and claims neither a percentage, ETA, nor delegated forward progress.
This does not change the `view` command's verified-map input contract or add a
beginner command. The page is self-contained, escapes operator-controlled text,
and has no network dependencies. Progress generation, browser generation, or
opening is best-effort; run-manifest-v2, validation evidence,
`map_session_recovery.json` when present, and the delegated map exit code remain
authoritative.

After a successful terminal `start`, the command prints one bounded VERIFIED or
UNVERIFIED card projected from that index: map output, verification status,
viewer, session index/page, run manifest, first-map receipt, and one exact
`Next:` command. A verified session adds the read-only `Report:` preparation
handoff. Viewer
failure makes the single `Next:` action the view retry and adds a separate
warning. With `--viewer none`, `Next:` is the copy-ready command for reopening
the map; an unverified or action-required session also prints its retained
one-line summary. No browser is required.

The additive `demo` command is a product orchestrator over the existing fixed
public-data script, not another SLAM route. Its read-only JSON exists only with
`--dry-run` and follows `first-map-demo-plan-v1`. It refuses unsafe path/output
states and low initial storage before delegation. Each volume exposes exact
`additional_bytes_required`; a low-storage finding preserves the complete
shell-quoted retry command. An existing map is reusable
only when its first-map receipt is schema-valid and exactly reproducible from
current retained evidence. Viewer failure cannot replace verified success.

The additive `quality` object does not alter first-map receipt semantics. It
groups the seven required receipt checks into four display cards and preserves
their source IDs. Numeric scoring is intentionally absent. Verification-off
maps to `not_verified`; missing or semantically invalid evidence maps to
`unavailable`, so an older or damaged output cannot silently gain PASS status.

The additive `sessions` command projects existing `map-session-index-v1`
artifacts into a `map-session-catalog-v1` response and local `sessions.html`.
It does not mutate session or map evidence. Discovery is bounded to direct,
non-symlink child bundles and 2 MiB schema-valid session indexes; result count
is capped at 200. `--json` is read-only. Browser rendering escapes every
operator-controlled field and links only regular non-symlink session pages.
Invalid candidates contribute only to `skipped_invalid`.

The additive `compare` command projects two session indexes into the fixed
`map-session-comparison-v1` contract. It treats stale or identity-mismatched
setup evidence as unavailable, compares recorded artifact names without
promoting current-file presence to proof, and emits no numeric score or winner.
Its `--json` path is read-only. HTML output is self-contained and refuses to
replace symlinks or files not generated by the comparison command.

The additive `support` command projects one schema-valid session into the
fixed `support-bundle-v1` contract. Valid setup evidence is identity-bound;
missing, stale, malformed or symlinked evidence remains unavailable. The
three-member ZIP excludes maps, bags, raw logs, parameter contents, local paths
and command credentials, and is marked for human review. `--json` is read-only;
ZIP creation is atomic, refuses replacement, and performs no remote mutation.
The additive `report` command does not create that ZIP. It is a
read-only handoff for a `verified` session, or a fixed Docker/source demo
output without `session.json`, whose PASS receipt remains exactly bound to the
retained manifest, diagnosis, and verification log. It prints a
copy-ready result, exact source commit or product-version fallback,
verification summary, the local privacy-bounded JSON receipt path, the
canonical issue form, and the four fields that still need the operator's own
input. An immutable image user is told to replace the suggested release value
with the digest they actually ran. The command never uploads, opens a browser,
or contacts GitHub. The older `support <session_bundle> --first-map` spelling
remains compatible and resolves to the same implementation and schema.
Missing, stale, malformed, non-PASS, or symlinked evidence fails closed.

`run --guided` remains a compatibility interaction layer: it repeats the existing
preflight, makes the selected profile, topics, checks, and output location
visible, and asks for confirmation before delegating to the same map runner.
`run --guided --yes` is the explicit non-terminal form. Both flags leave the
profile defaults and map algorithm unchanged.

## Naming rules for new options

- Use lowercase kebab case.
- End directory paths with `-dir`.
- Include units in numeric names, such as `-secs` or `-gib`.
- Render option values consistently as `<id>`, `<dir>`, `<file>`,
  `<seconds>`, or an explicit finite choice set.
- Prefer positive behavior and safe defaults. Negative flags are reserved for
  explicit break-glass behavior.
- Reject zero, negative, non-finite, and malformed numeric values during
  argument parsing when the contract requires a positive value.
- Reject invalid combinations with exit code `2`; do not silently ignore an
  option.
- Add automation output only through a versioned JSON contract.
- A beginner-facing orchestration command must measurably reduce submitted
  commands and delegate every technical lifecycle to its existing owner.

Every public addition must update `contracts/cli-v1.json`, command help,
documentation, and tests in the same change. CI compares the manifest with
the exact command and option sets in shell completion, the flags and metavars
rendered by each command, and every finite completion choice. An undocumented
option, extra stale completion, changed value shape, or missing choice fails
the contract test.

Maintained workflow profile IDs and descriptions have one installed registry,
`product_profiles.py`. Doctor help, runner choices and help, the CLI contract,
completion, and release bundles are tested against that registry so a profile
cannot be added to only one command surface.

## Migration sequence

1. `lidarslam-map view <output_dir>` owns optional viewer startup.
2. `run --viewer ...` routes through `view` and emits a deprecation warning
   while preserving its previous combined-command exit behavior.
3. `--verification {required,off}` makes the safety mode explicit;
   `--no-verify-map` remains its warning-emitting compatibility alias.
4. `--help` presents routine stable options; `--help-all` preserves full
   discoverability of advanced and deprecated options.
5. Freeze both help levels, exit codes, JSON schemas, and shell completion in
   the Humble and Jazzy installed-CLI checks.

No deprecated or provisional option is removed merely because a replacement
exists. Removal follows the compatibility window above.
