# Golden-path CLI

The product exposes one command surface over the existing, well-tested
map-authoring tools. After a source build and `source install/setup.bash`, use:

```bash
lidarslam-map
```

On an interactive terminal, the no-argument home offers only the existing
`demo`, `start`, `sessions`, and read-only `doctor` routes, shows the exact
delegated command, and keeps their normal safety checks. Demo execution
requires an explicit `yes`; doctor uses no network and writes no files.
No-argument non-interactive use remains a usage error with exit code `2`, so
automation never waits for a prompt. Direct commands remain stable:

```bash
lidarslam-map demo
lidarslam-map doctor
lidarslam-map start <rosbag2_dir>
lidarslam-map doctor <rosbag2_dir>
lidarslam-map setup <rosbag2_dir>
lidarslam-map run <rosbag2_dir> --guided --editable
lidarslam-map run <rosbag2_dir> --output-dir output/my_map
lidarslam-map inspect output/my_map
lidarslam-map view output/my_map
lidarslam-map edit output/my_map --plan map-edit-plan.json --output-dir output/my_map_edited
lidarslam-map merge output/day1 output/day2 --output-dir output/site_project
```

The equivalent repo-local spelling is:

```bash
./scripts/lidarslam demo
./scripts/lidarslam doctor
./scripts/lidarslam start <rosbag2_dir>
./scripts/lidarslam doctor <rosbag2_dir>
./scripts/lidarslam setup <rosbag2_dir>
./scripts/lidarslam run <rosbag2_dir> --guided --editable
./scripts/lidarslam run <rosbag2_dir> --output-dir output/my_map
./scripts/lidarslam inspect output/my_map
./scripts/lidarslam view output/my_map
./scripts/lidarslam edit output/my_map --plan map-edit-plan.json --output-dir output/my_map_edited
./scripts/lidarslam merge output/day1 output/day2 --output-dir output/site_project
```

`demo` is the no-private-data first-success path. `start` is the human-facing
own-bag mode. It composes the existing setup, run, verifier,
and view tools into one session without changing estimator defaults. It displays
the detected sensor contract and calibration before writing, saves a pinned
setup bundle, places the verified map under `map/`, and opens the browser view.

`demo` delegates map construction to `run_first_map_demo.sh`. The own-bag
commands delegate to
`preflight_autoware_map_bag.py`, `sensor_setup_wizard.py`,
`run_autoware_map_from_bag.py`, and
`diagnose_autoware_map_run.py`; post-processing delegates to the verified
viewer, editor, and multi-session merge engines. Established profile selection
and map verification behavior remain authoritative.

## Commands

### `demo`

Use `lidarslam-map demo [work_dir]` after installation to download the fixed
checksum-pinned public MID-360 dataset, run the maintained verified lifecycle,
and open the offline browser review. Defaults stay under `work_dir`:
`datasets/mid360_public` for cache and `output/mid360_demo` for the map.

Live execution verifies the registered archive SHA-256 plus the known
`metadata.yaml` and sqlite SHA-256 identities even when the files are already
cached. Dry-run labels those bytes `prepared_unverified` because it performs
no expensive hashing.

Progress is reported as durable named stages, without a guessed percentage or
ETA. A safe interruption after mapping is exposed as `resume_ready`; run
`lidarslam-map demo [work_dir] --resume` to continue only verification,
finalization, diagnosis, checksumming, and receipt generation. The command
refuses `initialized` or `workflow_running` state, so resume cannot launch a
second mapping process. Unsafe state receives an inspect command and a fresh,
non-overwriting retry output.

`--dry-run --json` is network- and write-free and emits
[`first-map-demo-plan-v1`](schemas/first-map-demo-plan-v1.schema.json), including
dataset DOI/license/size/SHA-256, exact paths, cache/output state, unique-volume
free-space checks, exact `additional_bytes_required`, steps, findings, and a
copy-ready command. Low-storage recovery keeps the full shell-quoted command
and rounds the displayed shortage upward. Live `--json` is rejected because
mapping emits progress rather than one document.

Use `--output <plan-file>` with `--dry-run` to retain either the human card or
the JSON plan without shell redirection. The file is created exclusively and an
existing path is refused; the command still returns exit code `2` when the
saved plan is not ready. The plan file is only an inspection artifact and does
not create the demo workspace, download data, run mapping, or publish evidence.

The command rejects exact symlink targets, dataset/output overlap, partial
output, unverified existing output, and insufficient initial free space. A
previous output is reused only after its receipt is rebuilt from current
schema-valid manifest, diagnosis, and verifier evidence and matches PASS. The
mapping itself remains owned by `run_first_map_demo.sh`; a successful process
without valid terminal evidence is rejected, while browser failure cannot
replace a verified map result.

### `start`

Use this for a first map or a bag from an unfamiliar installation:

```bash
lidarslam-map start /path/to/rosbag2
lidarslam-map start /path/to/rosbag2 --editable
lidarslam-map start /path/to/rosbag2 --yes --dry-run --json
```

Interactive use requires positive setup confirmation. RKO-LIO defaults to safe
calibration rejection, so no setup or map files exist before its extrinsics are
confirmed. `--yes` is explicit non-terminal confirmation, and live
`start --json` is rejected because a map emits progress rather than one JSON
document. The session keeps the reviewable setup at its root and delegates
mapping to the normal atomic `map/` output. Browser review is the default; use
`--viewer none` for headless use.

An incompatible bag or forced profile returns a non-writing `not_ready` result
with exit code `2`. Human output shows all detected sensor families, stable
finding codes, and an exact recovery command. `--json --dry-run` emits the same
contract under
[`sensor-setup-rejection-v1.schema.json`](schemas/sensor-setup-rejection-v1.schema.json).
Automation keys on `reason.code` and each `findings[].code`; messages and
`next_action` remain operator-facing guidance.

Once the map runner starts, a later non-zero result uses a separate, versioned
[`map-session-recovery-v1.schema.json`](schemas/map-session-recovery-v1.schema.json)
handoff. `start` writes `map_session_recovery.json` beside
`sensor_setup.json`, prints the stable primary reason and all recognized
findings, and lists retained manifest, diagnosis, verifier, and log paths. A
terminal schema-v2 lifecycle that stopped during post-processing receives the
exact identity-preserving `--resume` command. Other terminal failures receive
an exact `inspect` command plus a retry command that reuses the pinned profile,
parameters, frames, and verification mode under a fresh map output. A viewer
failure remains `[viewer-failed]` after a verified map and does not create a map
recovery receipt.
Alongside the detailed recovery receipt, every delegated `start` writes
`session.json` and derives one self-contained `session.html` dashboard. The
[`map-session-index-v1.schema.json`](schemas/map-session-index-v1.schema.json)
contract distinguishes `running`, `verified`, `unverified`, and
`action_required` without changing commands between outcomes. Before delegation
it begins as `running`,
opens in default browser mode, and refreshes every two seconds as atomic
run-manifest stages move through mapping, verification, finalization, and
evidence generation. It shows a six-step lifecycle rather than an unreliable
time estimate. A verified page then links the generated 3D review and evidence.
An unverified page says verification was skipped and puts a fresh
verification-enabled output command first. An action-required page shows the
stable reason, findings, retained evidence, and safe recovery actions. Terminal
browser preview generation uses `view --no-open`; the already-open common page
refreshes to link it. `--viewer none` only retains the page. Operator-controlled
content is escaped, the page loads no network resources, and progress,
page/viewer failure cannot replace the map result or machine-readable evidence.

The page also translates the seven first-map receipt checks into four quality
cards: workflow completion, map output, Autoware verification, and evidence
integrity. Each card retains its source check IDs. No synthetic numeric score is
created. Verification-off is `NOT VERIFIED`; a missing, malformed, duplicated,
or semantically incomplete receipt is `UNAVAILABLE`, never an inferred PASS.

The terminal completion output mirrors the same evidence without requiring the
browser: it prints the verification status, viewer or session-page path, run
manifest, first-map receipt, and one exact `Next:` command. A verified run also
prints the read-only `Report:` preparation handoff. When `--viewer none` is
used, `Next:` is
the copy-ready command for reopening the map.

### `sessions`

Use `lidarslam-map sessions` to reopen recent runs from the default `./output`
directory. The local responsive catalog shows session status, evidence-backed
quality, profile, bag, map path, and the recommended next action. It links back
to each retained `session.html`, so verified review and failure recovery resume
at the same durable page used during mapping.

With `--viewer none`, the terminal projection also prints the retained summary
for an unverified or action-required session, followed by its copy-ready next
action. Newlines in retained session text are compacted so the recovery block
stays one-line-per-field.

The optional positional root changes only the directory being inspected.
`--status` filters one stable session state, `--limit` is bounded from 1 to 200,
and `--viewer none` keeps the generated catalog without opening it. `--json` is
strictly read-only. Discovery is one level deep, does not follow symlinks, and
accepts only bounded schema-valid session indexes.

Each card can be selected for comparison. After exactly two are selected, the
catalog exposes a safely quoted, copy-ready `lidarslam-map compare` command.
Its **Get support** section also provides a safely quoted command for that
single session.

### `compare`

Use `lidarslam-map compare <left_session> <right_session>` to review two local
session bundles side by side. The report has 14 fixed readiness, setup, and
retained-artifact rows. Every row is descriptive: `same`, `different`, or
`unavailable`. It never assigns a numeric score or selects a winner.

Readiness comes only from each schema-valid `session.json`. Setup values are
used only when `sensor_setup.json` still matches the session, bag, output,
profile, and current parameter snapshot hashes. Stale or missing evidence is
shown as unavailable rather than reconstructed from nearby files. `--json` is
read-only; `--viewer none` writes the standalone HTML without opening it.

### `support`

Use `lidarslam-map support <session_bundle>` to create one privacy-first ZIP
for a maintainer issue. The archive always has exactly `README.txt`,
`issue-body.md`, and `support-report.json`. The report projects bounded status,
setup identity, diagnosis state, artifact state and evidence hashes from a
schema-valid session; stale or symlinked setup evidence remains unavailable.

The archive excludes map geometry, bags, raw sensor data, raw logs, parameter
contents, exact local paths and credential-like command values. It is written
atomically to a new `.zip`, never overwrites a path, and performs no upload or
remote mutation. Review all three members before public attachment. `--json`
prints the sanitized schema-valid report without writing a ZIP.

When `inspect --symptom ... --write` retained a visual symptom, `support`
projects only that fixed code and the explicit user-reported evidence basis.
The issue body says it is not an automatically diagnosed root cause; symptom
titles, checks, commands, and free text are excluded.

For a verified first map, run
`lidarslam-map report <session_bundle>` or copy **Prepare a first-map report**
from `session.html`; fixed Docker/source demo outputs can be passed directly
when they contain the receipt but no session page. This command creates no ZIP, writes no
file, and contacts no remote service. It requires receipt-bound quality PASS,
revalidates the retained receipt plus manifest, diagnosis, and verification-log
hashes, then prints one copy-ready verification summary, safe environment
hints, the exact reviewed JSON receipt to attach, and the canonical
independent-validation issue form. Its four-field template asks for the public
documentation path, environment, a command shape with private values replaced
by `REDACTED`, and findings. Add `--json` for a schema-valid,
machine-readable `first-map-handoff-v1` result; it is still read-only and
local-only because it includes the local receipt path. Attach only the
reviewed receipt, never the handoff JSON. The older
`support <session_bundle> --first-map` spelling remains compatible. `report`
exposes only `--json`; ZIP-only `--output` is unavailable, so it cannot create
a second evidence artifact.

### `setup`

Use this to pin one inspected sensor input. It reuses `doctor`'s topic,
PointCloud2 layout, timestamp-order, and profile selection, adds message-frame
detection, and writes a versioned sensor setup manifest plus parameter
snapshots. All four maintained profiles use this entrypoint: the two RKO-LIO
profiles, PointCloud2+NavSatFix, and VelodyneScan+Applanix GSOF49. RKO-LIO
bundles remain unwritten until profile extrinsics are explicitly accepted or
both measured transforms are supplied. GNSS and packet bundles record their
actual navigation inputs and reject RKO-only controls. Existing bundle
directories are never overwritten.

The generated command passes the pinned YAML files and selected frames back to
the normal `run` lifecycle, so atomic output, verification, diagnosis, receipts,
and offline browser review remain unchanged.

### `run --guided`

This compatibility path remains available for existing launchers that want an
explained run without creating a sensor setup session:

```bash
lidarslam-map run /path/to/rosbag2 --guided
lidarslam-map run /path/to/rosbag2 --guided --yes
lidarslam-map run /path/to/rosbag2 --guided --editable --dry-run
```

The guided mode makes the following visible before starting: bag duration, chosen
LiDAR and IMU topics, PointCloud2 field inspection, timestamp-order status,
profile, output directory, and viewer follow-up commands. If no maintained path is safe,
it stops before launching ROS and prints the missing requirement plus the next
`doctor` command. It also checks the local ROS runtime artifacts before asking for
confirmation, so an incomplete build is reported with a copy-ready build/source hint.
After a run it prints map/verification status and copy-ready
`inspect`/`view` commands. Use `run` directly for CI and other automation.

`doctor` without a bag checks the curated runtime surface, matching installed
prefix, Humble/Jazzy environment, ROS CLI, bag reader, and fixed-demo storage.
It emits `ready` or `action_required` plus stable findings, uses no network,
writes no files, and omits local paths from JSON. With a rosbag2 directory it
preserves the existing metadata preflight, reports detected topic capabilities,
and selects a compatible maintained profile. For RKO-LIO profiles it also
reads the first selected `PointCloud2` record and requires FLOAT32 XYZ fields
plus a supported per-point timestamp field (`t`, `timestamp`, `time`, or
`stamps`). If the record cannot be inspected or does not satisfy that layout,
no RKO-LIO profile is recommended. When an Odometry topic is present, doctor
also scans the highest-count Odometry topic and every recorded TF topic up to
100,000 messages per topic. It reports empty or inconsistent parent/child
frames, a missing path, and a static-only path without hiding an otherwise
compatible maintained SLAM profile. When that path contains dynamic edges and
a PointCloud2 topic exists, doctor makes one more bounded pass in bag record
order. For each cloud it compares `header.stamp` with the latest recorded stamp
seen so far on every required dynamic edge. It reports clouds that arrive
before all required edges and positive future-TF gaps, including the largest
gap and limiting edge. Add `--json` for automation. These bounded bag checks do
not certify timestamp units, calibration, live executor or DDS scheduling,
clock alignment, TF buffer history, interpolation, or accuracy.

`run` prints the selected profile and deterministic command before execution.
Use `--dry-run` to inspect the plan without creating the output directory.
For an RKO-LIO graph profile, add `--editable` to retain the deterministic
backend-input bag and exact graph parameters in the output. It consumes extra
disk space but makes accepted-loop repair a later one-command operation.
The runner writes into `<output>.partial` and atomically renames that directory
to the requested output name after the workflow stops. It refuses to start
when either name already exists, so a new run never overwrites prior evidence.
It also refuses to launch when the output filesystem has less than 5 GiB free.
Use `--min-free-space-gib <GiB>` to set a larger reserve after sizing the
expected map; the configured reserve is a refusal boundary, not an output-size
prediction.
Completed, failed, and interrupted workflows retain their artifacts and
terminal state for diagnosis. The runner isolates the workflow in its own
process group. `Ctrl-C` (`SIGINT`) and service/container termination
(`SIGTERM`) are forwarded to that group, bounded by a ten-second graceful
shutdown before forced cleanup. The manifest is then completed with
`interrupted` status, exit `130` or `143`, and the terminating signal in
`lifecycle.last_error`. Once that sealed state exists, direct `run` prints one
concise stop/evidence line instead of a generic failure and repeating the full
internal command. A non-interrupted process that merely returns 130 or 143
retains the normal failure diagnostics.

If the workflow has stopped but verification, finalization, diagnosis, or
checksum generation was interrupted, resume only those terminal
post-processing stages:

```bash
./scripts/lidarslam run <rosbag2_dir> \
  --output-dir output/my_map \
  --resume
```

Resume requires the original bag, software checkout, ROS distribution,
profile, options, and output path. Their checksums and identities must match
the manifest exactly. It accepts either `output/my_map.partial` before the
atomic rename or `output/my_map` after the rename, but never both. A manifest
at `initialized` or `workflow_running` is refused because the original process
may still be active. An advisory lock also prevents two post-processing
processes from owning the same output.

Resume never starts the SLAM workflow again.

Every non-dry run produces a schema-v2 `run_manifest.json`. It records:

- SHA-256 identities for `metadata.yaml`, every referenced rosbag storage file,
  and every output artifact;
- product, core package, Git commit/dirty-state, and ROS distribution identity;
- the selected profile and exact argument vector;
- UTC start/finish timestamps, exit code, terminal status, and diagnosis
  status;
- the durable lifecycle stage, resume count, runner exit code, and last
  post-processing error.

After the manifest reaches its terminal state, the runner also derives
`first_map_validation_receipt.json` and `.md`. The receipt contains only
version/profile identity, status values, and hashes needed for the independent
first-map program; it excludes map geometry, private paths, and the exact
command.

Hashing large bags adds a sequential input read before execution. This is
deliberate: the manifest identifies the data that was actually processed,
rather than only the path where it happened to be stored.

Installed commands read Git revision and dirty state from deterministic
build-time metadata, so moving or deleting the source/build tree does not erase
software provenance. See
[Installed source identity](distribution.md#installed-source-identity).

`execution.exit_code` is always the map-workflow exit code.
`lifecycle.runner_exit_code` is the overall runner result, including
verification and post-processing. `succeeded` means both the workflow and
enabled verification completed successfully. When verification is enabled
(the default), a non-`success` diagnosis changes the manifest to `failed`.
With the diagnostic-only `--verification off`, inspect
`output.diagnosis_status` separately; `succeeded` does not claim that map
verification ran. The old `--no-verify-map` spelling remains a deprecated
compatibility alias.

`inspect` classifies an output as `success`, `map_saved`, `verify_failed`,
`runtime_failed`, or `incomplete`. Add `--write` to create the Markdown and
JSON diagnosis artifacts in the output directory.

`view` validates a completed output before staging and opening it. Map
generation and viewing are deliberately separate, so a standalone viewer
failure does not change the completed run manifest:

```bash
lidarslam-map view output/my_map
lidarslam-map view output/my_map --viewer foxglove
```

The default `browser` viewer exports a self-contained offline HTML preview and
opens it when a desktop session is available. It needs no Autoware checkout,
Foxglove bridge, or network connection. Use `--no-open` for CI/headless hosts;
the command prints the exact generated path.

### `edit`

The browser preview can export a versioned JSON plan containing unwanted XYZ
boxes and accepted loop constraints selected for removal. Apply it to a new
candidate and reopen the result:

```bash
lidarslam-map edit output/my_map \
  --plan map-edit-plan.json \
  --output-dir output/my_map_edited
lidarslam-map view output/my_map_edited
```

`edit` verifies the plan's source hashes before writing, refuses an existing
destination, preserves every PCD field while filtering boxes, and runs both
the map-bundle and Autoware pointcloud-map verifiers before publishing the
candidate atomically. It writes `map_edit_plan.json` and
`map_edit_receipt.json` into the candidate for review and reproduction.

An accepted loop constraint changes optimized poses, not just a line in a
graph file. Create maps that may need loop repair with `run --editable`; then
the normal browser-printed command automatically discovers the retained input
and exact graph parameters from the source output:

```bash
lidarslam-map edit output/my_map \
  --plan map-edit-plan.json \
  --output-dir output/my_map_without_bad_loop
```

Source the built or installed ROS environment first. Replay summaries, runner
logs, and retained constraints are copied into `map_edit_replay_evidence/` in
the candidate. Path overrides for older/manual outputs appear under
`edit --help-all`; missing replay prerequisites fail before output creation.

### `merge`

Use the first completed output as the anchor and add overlapping repeat visits:

```bash
lidarslam-map merge output/day1 output/day2 output/day3 \
  --output-dir output/site_project
lidarslam-map view output/site_project
```

Every source must pass the map-bundle and Autoware pointcloud-map contracts.
Gravity-aligned yaw/translation ICP runs sequentially against the accumulated
project and is gated by trimmed median, p90, and coverage. Projector origin,
frame, tile resolution, and PCD field mismatches fail before publication. Use
`--dry-run` to inspect transforms without writing output. When initialization
is ambiguous, pass `--initial-transform <index:tx,ty,tz,yaw_deg>`.

Sources remain unchanged. Success atomically publishes a deduplicated
full/tiled pointcloud map, per-session transformed trajectories,
`map_project.json`, and `map_merge_receipt.json`. The synthesized G2O artifact
contains trajectory vertices for bundle compatibility but claims no
cross-session graph edges; this limitation is recorded in `map_bundle.yaml`.

## Option tiers

Normal command help groups the stable options needed for routine operation.
Use `--help-all` to also display advanced runtime controls and deprecated
compatibility aliases:

```bash
lidarslam-map run --help
lidarslam-map run --help-all
lidarslam-map view --help-all
lidarslam-map edit --help-all
```

Existing option names remain compatible; the help levels and tiers make the
stable beginner surface distinct from viewer plumbing and safety overrides.
The stability label and migration rules for every option are defined in the
[CLI compatibility and option policy](cli-compatibility.md).

| Tier | Options | Use |
| --- | --- | --- |
| Help | `<command> --help`, `<command> --help-all` | Show routine options or the complete advanced/deprecated inventory |
| Public first map | `demo`, `demo --dry-run --json`, `demo --resume`, `demo --viewer none` | Download fixed public data, verify a map, safely finish interrupted post-processing, reuse trusted output, and optionally open it |
| One-command session | `start`, `start --yes --dry-run`, `start --editable`, `start --viewer` | Pin calibration and configuration, run the verified map lifecycle, then open the result |
| Session return and comparison | `sessions`, `sessions --status`, `compare --viewer`, `compare --json` | Reopen local runs and compare two retained evidence sets without inferred scoring |
| Maintainer or validator report | `support --output`, `support --json`, `report [--json]` | Create a fixed privacy-first ZIP, inspect its sanitized report, or revalidate and hand off one verified first-map receipt without writing |
| Doctor output | `doctor --json`, `doctor <rosbag2_dir> --public-json` | Keep the full versioned preflight local for automation, or emit bounded path-free evidence for a reviewed public issue |
| Guided compatibility | `run --guided`, `run --guided --yes`, `run --guided --dry-run` | Preserve the earlier explained-run path for existing launchers |
| Map selection and output | `run --profile`, `run --output-dir` | Select a maintained profile or an explicit artifact directory |
| Safety and lifecycle | `run --min-free-space-gib`, `run --dry-run`, `run --resume` | Refuse unsafe starts, inspect a plan, or finish terminal post-processing |
| Viewer | `view --viewer {browser,autoware,foxglove}` | Open an existing completed output; defaults to the self-contained browser preview |
| Browser preview | `view --no-open`, `view --preview-dir` | Export without launching a browser, or choose where the portable preview is written |
| Offline map edit | `edit --plan`, `edit --output-dir`, `edit --dry-run` | Create a separate verified candidate from a browser-exported plan |
| Editable mapping | `run --editable` | Retain replay input and exact graph parameters for later accepted-loop repair |
| Loop-edge replay overrides | `edit --backend-input`, `edit --params`, `edit --setup` | Override auto-detected replay inputs for older/manual outputs |
| Multi-session project | `merge --output-dir`, `merge --dry-run`, `merge --initial-transform` | Align overlapping completed maps into a separate verified project |
| Live viewer runtime | `view --autoware-core-dir`, `view --work-dir`, `view --runtime-dir`, `view --rebuild`, `view --auto-exit-secs` | Control Autoware/Foxglove build and runtime details |
| Deprecated viewer compatibility | `run --viewer`, `run --autoware-core-dir`, `run --work-dir`, `run --viewer-run-dir`, `run --viewer-rebuild`, `run --auto-exit-secs` | Preserve existing combined run/view invocations while directing users to `view` |
| Verification | `run --verification {required,off}` | Keep required map verification (default) or explicitly select a diagnostic-only unverified run |
| Deprecated verification alias | `run --no-verify-map` | Compatibility spelling for `--verification off`; emits a warning |
| Inspection context/output | `inspect --bag`, `inspect --symptom`, `inspect --json`, `inspect --write` | Add source-bag context or one user-reported visual symptom, choose machine output, or persist diagnosis files; symptom triage never claims an automatic cause |

Viewer-specific options that would otherwise be ignored are rejected with exit
code `2`. In particular, `--autoware-core-dir` requires
`--viewer autoware`; the other advanced viewer options require either
`--viewer autoware` or `--viewer foxglove`. These checks describe only the
deprecated `run` compatibility options; `view` has no `none` mode.

## Versioned JSON contracts

Automation should select the schema using `schema_version` and `schema_uri`;
it must not infer compatibility from the repository version.

- [Preflight schema v6](schemas/preflight-v6.schema.json) — current; adds
  bounded replay-order PointCloud2-to-dynamic-TF timing evidence
- [Public doctor evidence schema v1](schemas/public-doctor-evidence-v1.schema.json)
  — path/topic-name/frame-name/command/free-text-free
  type/count/check/profile/finding-code projection for reviewed public issues,
  including bounded input-error evidence
- [Preflight schema v5](schemas/preflight-v5.schema.json) — adds bounded
  Odometry parent/child and recorded dynamic-TF connectivity evidence
- [Preflight schema v4](schemas/preflight-v4.schema.json) — adds
  stable rejection finding codes and one concrete next action per finding
- [Preflight schema v3](schemas/preflight-v3.schema.json) — adds
  bounded, per-topic `PointCloud2`/`Imu` `header.stamp` order inspection
- [Preflight schema v2](schemas/preflight-v2.schema.json) — adds record-level
  PointCloud2 field inspection
- [Preflight schema v1](schemas/preflight-v1.schema.json)
- [Diagnosis schema v1](schemas/diagnosis-v1.schema.json)
- [Map session recovery schema v1](schemas/map-session-recovery-v1.schema.json)
  — profile identity, stable runtime/map-quality reasons, retained evidence,
  and copy-ready resume or fresh-output retry actions for `start`
- [Map session index schema v1](schemas/map-session-index-v1.schema.json) — one
  running and terminal landing contract, durable six-step progress, and the
  recommended first terminal action for every delegated `start`
- [Map session catalog schema v1](schemas/map-session-catalog-v1.schema.json)
  — bounded newest-first local history derived from valid session indexes
- [Map session comparison schema v1](schemas/map-session-comparison-v1.schema.json)
  — fixed descriptive rows, evidence availability, and explicit no-score and
  no-winner policy for two local sessions
- [Run manifest schema v1](schemas/run-manifest-v1.schema.json)
- [Run manifest schema v2](schemas/run-manifest-v2.schema.json) — current; adds
  resumable lifecycle state
- [First-map validation receipt schema v1](schemas/first-map-validation-receipt-v1.schema.json)
  — privacy-bounded external onboarding evidence
- [Map edit plan schema v1](schemas/map-edit-plan-v1.schema.json) — source-pinned
  non-destructive region and loop-constraint operations
- [Map edit receipt schema v1](schemas/map-edit-receipt-v1.schema.json) — candidate
  identities, edit counts, replay evidence, and verifier result
- [Map project schema v1](schemas/map-project-v1.schema.json) — source identities,
  transforms, and deterministic merge settings
- [Map merge receipt schema v1](schemas/map-merge-receipt-v1.schema.json) — merged
  point counts, candidate identities, and verifier result

Top-level fields are closed within a published schema. A field addition,
removal, type change, or semantic break requires a new schema file and
migration guidance.

Preflight v1 through v4 and run manifest v1 remain published for existing
artifacts. Preflight v1 only reports metadata-level topic compatibility;
preflight v2 adds PointCloud2 field inspection but predates header timestamp
order evidence. Preflight v3 adds timestamp-order evidence but predates stable,
actionable rejection findings. Preflight v4 adds those findings but predates
Odometry-to-TF bag connectivity evidence. Run manifest v1 predates durable
lifecycle stages, so it can be inspected but cannot be resumed safely.

For archived terminal v1 manifests, create a separate schema-v2 inspection
copy only when an automation consumer requires v2:

```bash
lidarslam-map migrate-manifest output/historical_run \
  --output output/historical_run-v2.json \
  --verification required
```

The verification mode is mandatory because v1 did not record it. The command
accepts only `succeeded`, `failed`, or `interrupted` terminal records with a
finish time and exit code. It never overwrites `run_manifest.json` or an
existing destination, and writes lifecycle stage `complete`; the migrated
copy is therefore never eligible for `run --resume`.

`migrate-manifest` and `rollback-plan` are recovery tools, not beginner
mapping steps. They are intentionally absent from top-level `--help` and are
listed by `lidarslam-map --help-all`. Their flags remain published in the
[CLI v1 contract](contracts/cli-v1.json) and available through explicit
subcommand help.

Preflight v3 scans up to 100,000 records on the selected PointCloud2 and Imu
topics. `passed` means every metadata-declared record on those topics was
checked; `sampled` means the bound was reached without finding disorder.
Preflight v4 preserves the v3 summary and the legacy `missing_requirements`
strings, then adds a closed `findings` record for each rejection. Every finding
contains a stable kebab-case `code`, a human `message`, and one non-empty
`next_action`; automation should key on the code, not parse the message.
`start` and `setup` carry those findings into the versioned
`sensor-setup-rejection-v1.schema.json` NOT READY response without creating a
setup or map directory.
Either result permits a compatible profile. A detected reversal, invalid
header stamp, unreadable selected stream, or inspection error blocks the
affected mapping profile before ROS processes start. The JSON records scanned
counts, completion, first/last stamps, reversal counts and the largest
backward jump, so automation must not treat `sampled` as a full-bag proof.

Each subcommand accepts the same options as its delegated tool:

```bash
./scripts/lidarslam doctor --help
./scripts/lidarslam run --help
./scripts/lidarslam inspect --help
./scripts/lidarslam view --help
./scripts/lidarslam edit --help
./scripts/lidarslam merge --help
./scripts/lidarslam migrate-manifest --help
./scripts/lidarslam rollback-plan --help
```

## Exit-code contract

| Code | Meaning |
| --- | --- |
| `0` | Command completed successfully |
| `2` | Invalid usage, input, profile, or output path |
| `70` | Internal/tooling error prevented the command from starting |
| `130` | Run was interrupted by the operator (`SIGINT`) |
| `143` | Run was terminated by a service or container manager (`SIGTERM`) |
| other non-zero | Map-workflow exit code, propagated unchanged |

## Installed names and compatibility

The ROS package's historical C++ node remains
`ros2 run lidarslam lidarslam`. It is not replaced by the product CLI.

The installed product command is `lidarslam-map`. ROS tooling can invoke the
same command surface through
`ros2 run lidarslam lidarslam-cli <command> ...`. The repo-local
`./scripts/lidarslam` spelling remains useful before installation.

These are aliases for the same own-bag entrypoint, not additional beginner
workflows. See [Distribution and installed CLI](distribution.md) for package
contents, platform support, and the binary-release boundary.
