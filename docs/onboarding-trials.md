# Comparable onboarding trials

An onboarding trial measures how much work a new operator must do before the
product produces a verified first map. It complements the generated first-map
receipt: the receipt proves the result, while the trial record captures time,
commands, transfer size, disk use, and any undocumented intervention.

Use this contract for the fixed Docker and source quickstarts before changing
the landing page or promoting a release. Never reconstruct a missing value
from memory. Store it as `null`; the checker will retain the valid record but
mark its measurements `INCOMPLETE`.

Current bounded evidence is summarized in the
[2026-08-12 onboarding matrix](evidence/growth/onboarding-matrix-index-2026-08-12.md).
All four reviewed Humble/Jazzy Docker and source rows produced a product
`PASS`, but none is a comparable baseline: required human measurements are
missing and the Docker rows are v0.9.0 while the source rows are v0.9.1.

## Fixed G0 trial matrix

| Trial | Clean starting point | Canonical documentation | Fixed input |
| --- | --- | --- | --- |
| Docker Humble, x86_64 | Ubuntu 22.04 with Docker installed; no project image, dataset, or output cache | [Docker First Map](getting-started.md#docker-first-map-no-ros-2-workspace) | MID-360 public demo |
| Docker Jazzy, x86_64 | Ubuntu 24.04 with Docker installed; no project image, dataset, or output cache | [Docker First Map](getting-started.md#docker-first-map-no-ros-2-workspace) with `v0.9.0-jazzy` | MID-360 public demo |
| Source Humble, x86_64 | Ubuntu 22.04 with ROS 2 Humble installed; no checkout, build, dataset, or output | [fixed source first map](getting-started.md#2-run-the-fixed-first-map-demo) | MID-360 public demo |
| Source Jazzy, x86_64 | Ubuntu 24.04 with ROS 2 Jazzy installed; no checkout, build, dataset, or output | [fixed source first map](getting-started.md#2-run-the-fixed-first-map-demo) | MID-360 public demo |

A row with no runnable documented path is a product finding, not a skipped
success. Record it as `FAIL` at the earliest applicable stage. All four rows use
the same fixed MID-360 dataset and the same first-map output contract. Runtime
and output-size values are compared across releases within the same row.
Cross-path values measure onboarding burden only; they are not
algorithm-performance claims.

## Measurement sheet

Record every field in
[`onboarding-trial-v1.schema.json`](schemas/onboarding-trial-v1.schema.json).
The following rules keep separate operators and releases comparable.
The detailed observer procedure is the
[G0 onboarding-trial execution runbook](onboarding-trial-execution.md).
Its disposable-host `run_source_onboarding_probe.py` helper can pin the public
source identity, execute the unchanged headless quickstart, collect the machine
measurements, prompt for the human active-time and command-count observations,
and write the same bounded v1 record. It is observer automation, not a
replacement product path; use the manual protocol when independently auditing
its measurements.
The Docker helper uses the same prompts; on a dedicated disposable VM,
`--disk-scope / --acknowledge-dedicated-filesystem` also records the host
filesystem peak. Omit that option on a shared host, where the Docker record
must remain explicitly non-comparable.
When both human observations are unavailable, both helpers accept
`--record-human-measurements-unknown` to retain explicit `null` values without
typing the two individual unknown flags.
Before provisioning a trial VM, `--public-preflight` checks only the immutable
GitHub source route and returns machine-readable `READY` or `NOT_READY` without
writing files. It requires the exact six-package inventory, dependency helper,
fast build, Getting Started route, and matching product version from one
commit; API/tool failure remains distinct from an unavailable route.
For its GitHub API reads, the observer uses an explicit `GITHUB_TOKEN` when
provided, otherwise it non-interactively reuses the active `gh auth`
credential. If neither is available, it keeps the anonymous read path and
fails closed when that quota is insufficient. The credential is sent only to
exact `https://api.github.com` GET requests and is never stored in the trial
or preflight report.

### Attach measurements without rerunning a completed trial

If the product result and machine evidence already exist but a human
observation was recorded separately, attach only the missing values with the
SHA-bound supplement helper. It never overwrites the original trial record,
does not rerun the product, and rejects a supplement whose base-record hash or
trial ID differs. Use only values observed during the attempt or retained in
the private observer material; do not reconstruct them from memory.

```bash
python3 scripts/complete_onboarding_measurements.py \
  "$TRIAL_RECORD" \
  --prompt-human-measurements

python3 scripts/check_onboarding_trial.py "$TRIAL_RECORD" \
  --supplement "$TRIAL_RECORD.measurements.json" \
  --json --require-comparable
```

The supplement output defaults to `"$TRIAL_RECORD.measurements.json"`; pass
`--output` when a different private path is required. The helper refuses to
overwrite an existing supplement or a known measurement. With `--json`, input
prompts go to stderr so stdout remains valid machine-readable JSON, and the
result includes the exact next validation command.
When `--prompt-human-measurements` is used, stderr first shows a compact
measurement card with the trial identity, wall-time bound, counting rules,
privacy reminder, and that same validation command.

For a dedicated-filesystem Docker observation, add
`--peak-disk-bytes "$PEAK_DISK_BYTES"` to the first command. After review, set
the matching matrix-index row's `measurement_supplement_path` to this
supplement and rerun `check_onboarding_trial_matrix.py`; a supplement is
evidence to review, not authorization to manufacture a comparable row.

| Field | Measurement rule |
| --- | --- |
| `environment.clean_start` | `true` only when the documented prerequisites are present but the project checkout/image, dataset, build, install, and output are absent. Do not pre-pull an image or warm a package, Git, or dataset cache. |
| `environment.revision` | Resolve source to a 40-character Git commit or an image to a `sha256:` digest before acceptance. A release tag may document a trial, but it is not immutable enough for comparison. |
| `input.download_bytes` | Count the bytes newly transferred for the selected dataset from a cold dataset cache. Use the archive's recorded payload size when the downloader exposes it. This field excludes unrelated host traffic and prerequisite installation. |
| `measurements.workflow_download_bytes` | Count all bytes received by the isolated trial environment during the timed documented path, including image layers or source checkout, dependencies, and dataset. Measure a dedicated network interface so unrelated traffic is excluded. This value cannot be smaller than `input.download_bytes`. |
| `measurements.wall_time_sec` | Start immediately before entering the first command on the selected documentation path. Stop when the receipt is written or the attempt reaches its terminal failure. Include downloads and unattended processing. |
| `measurements.active_operator_time_sec` | Accumulate only hands-on time spent entering commands, answering prompts, reading required output, and following documented next actions. Pause while a command runs unattended. Do not include note taking for the study. |
| `measurements.command_count` | Count each command submitted by the operator. A copied multiline shell block is one command; commands invoked internally by a script do not count. Recovery commands count, even when documented. |
| `measurements.peak_disk_bytes` | Measure the largest increase above the clean baseline in the trial's dedicated filesystem scope. Include project images or checkout, dataset, build/install tree, temporary output, and final output. Use the same scope for every row. |
| `measurements.output_bytes` | Measure allocated bytes for the finalized run directory after success, or the retained partial run directory after failure. Do not include the source dataset. |
| `outcome.undocumented_manual_steps` | Count every intervention needed for progress that the selected page did not instruct. A passing comparable baseline requires zero. |
| `outcome.finding_codes` | Use short stable codes such as `image-tag-missing`, `dependency-resolution`, or `mapping-timeout`. Every failed trial needs at least one code and a failure stage. |

Byte values are integer counts, not rounded MB/GB display values. Wall and
active time may contain fractional seconds. Keep the raw stopwatch and disk
observations outside Git if they reveal local details; the committed record
contains only the bounded aggregate fields.

## PASS and comparability gates

A `PASS` record must have all of these outcomes:

- runner exit code `0`;
- succeeded manifest and successful diagnosis;
- verifier and privacy-bounded receipt both report `PASS`;
- no undocumented manual steps and no failure stage;
- SHA-256 values for the manifest and receipt.

A valid record is a **comparable baseline** only when it also starts clean,
uses an immutable revision, has all seven measurements, and passes. Failed and
incomplete trials remain valuable evidence; they must not be silently removed
from the study.

Validate a record with:

```bash
python3 scripts/check_onboarding_trial.py trial.json \
  --json \
  --require-comparable
```

Exit code `0` means comparable, `1` means the JSON is valid but does not meet
the comparison gate, and `2` means the record violates the contract. Omit
`--require-comparable` when auditing an expected failure.

After validating each row, audit the fixed four-row matrix without inferring
missing success:

```bash
python3 scripts/check_onboarding_trial_matrix.py --json
```

With no record arguments, the checker loads the schema-backed reviewed index
at `docs/contracts/g0-onboarding-matrix-evidence-v1.json`. The index names the
all four checked-in rows explicitly (two Docker and two source); it does not
scan files, infer a latest run, or turn an absent row into a failure or success.
Use explicit paths when reviewing a provisional matrix that has not been added
to that index:

```bash
python3 scripts/check_onboarding_trial_matrix.py \
  docker-humble.json docker-jazzy.json \
  source-humble.json source-jazzy.json \
  --json --require-activation-gate
```

The matrix checker requires the exact x86_64 Ubuntu/ROS pairing, immutable
revision kind, fixed full MID-360 dataset identity, one shared product version,
and one shared source commit. It reports `INCOMPLETE` until all four records
exist. The current no-argument audit truthfully reports four present product
PASS outcomes, zero comparable rows, and a `BLOCKED` decision because
measurements and version alignment remain incomplete. The activation gate
needs at least one comparable Docker row and one comparable source row;
`--require-all-comparable` is the stricter four-row gate. Neither result
substitutes for the other release gates.

## Privacy boundary

The record must not contain operator identity, private filesystem paths, exact
commands, map geometry, bag metadata, hostnames, IP addresses, or raw logs.
Use bounded slugs for trial and dataset identifiers. Review the manifest and
receipt separately before sharing them; only their SHA-256 values belong in
this record.

Summarize completed rows in the
[weekly growth scorecard](growth-scorecard.md), then fix the largest repeated
activation blocker before expanding discovery work.
