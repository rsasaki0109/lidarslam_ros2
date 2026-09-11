# Operational reliability

This page defines how the golden-path runner behaves when an input, workflow,
or post-processing stage fails. It is the Phase 3 evidence ledger; a row is
only marked covered when an automated test exercises the public behavior.

## Failure and recovery matrix

| Failure | Public behavior | Preserved state | Operator action | Coverage |
| --- | --- | --- | --- | --- |
| Missing or corrupt `metadata.yaml`, missing referenced storage file | Exit `2` before workflow launch | Existing output is untouched; a new output is not created | Correct or replace the bag, then rerun `lidarslam-map doctor` | Automated |
| Incompatible topic/profile or PointCloud2 field layout | Exit `2` with available profiles and missing requirements before workflow launch; RKO-LIO requires FLOAT32 XYZ and a supported per-point timestamp field | Existing output is untouched | Choose a compatible profile or fix the input fields | Automated |
| Final or `.partial` output collision | Exit `2`; no overwrite | Both existing paths remain immutable | Inspect the existing run or choose a new output name | Automated |
| Output filesystem below the configured reserve | Exit `2` before workflow launch with required and observed GiB | Existing output is untouched; a new output is not created | Free storage, choose another output filesystem, or set a deliberately sized `--min-free-space-gib` budget | Automated |
| Workflow exits non-zero | Propagate the workflow exit code | Final output, diagnosis, checksums, and schema-v2 manifest with `failed` status | Inspect the diagnosis and launch log; rerun into a new directory after fixing the cause | Automated |
| Storage exhaustion during a manifest or map write | Release a 2 MiB terminal-evidence reserve; return the workflow or runner failure code; diagnose `ENOSPC`, quota, and “No space left on device” signatures | Final failed manifest, diagnosis, logs, and prior durable artifacts; incomplete manifest temp files are removed | Preserve evidence, free storage, and rerun into a new output directory | Automated failure injection and real bounded-filesystem gate |
| Operator `Ctrl-C` (`SIGINT`) | Forward `SIGINT` to the isolated workflow process group; force cleanup after ten seconds; exit `130` | Final output and manifest with `interrupted` status and signal reason | Inspect preserved evidence; rerun into a new directory if a map is still required | Automated |
| Service/container stop (`SIGTERM`) | Forward `SIGTERM` to the isolated workflow process group; force cleanup after ten seconds; exit `143` | Final output, diagnosis, checksums, and manifest with `interrupted`, `143`, and `SIGTERM` | Inspect preserved evidence; rerun into a new directory if a map is still required | Automated failure injection |
| Termination after the workflow result is durable but before post-processing completes | Leave the last durable schema-v2 lifecycle stage in the final or `.partial` directory | Original input/software/command identity and map artifacts | Run the same command and output path with `--resume`; SLAM is not rerun | Automated |
| Ambiguous or unsafe resume state | Exit `2`; refuse concurrent or pre-terminal post-processing | Both candidate directories and manifests remain unchanged | Verify no original process is active; resolve the ambiguity manually before retrying | Automated |
| Historical schema-v1 output needs a v2 reader | Require an explicit verification mode and accept only terminal v1 state; write a separate schema-v2 `complete` record | Source manifest and any existing destination remain byte-for-byte unchanged | Run `migrate-manifest` with a new output filename; use the result for inspection only | Automated |
| Published image must be rolled back | Validate signed release evidence and generate pull, attestation, and CLI smoke commands for the exact digest | Moving and versioned registry tags are never changed | Run `rollback-plan`, verify its commands, then deploy the immutable digest reference | Automated and release-gated |
| Missing TF connectivity observed in ROS logs | Diagnosis is `runtime_failed` with a TF connectivity hint | Launch log and diagnosis artifacts | Correct calibration/frame configuration and rerun | Automated diagnosis fixture |
| Classic scanmatcher VoxelGrid layout or absolute index exceeds PCL's signed 32-bit contract; leaf size is invalid; or a cloud marked dense contains non-finite XYZ | Reject before calling PCL; emit a throttled `VOXEL_GRID_*` reason; skip only the affected input scan, initial map, map update, or registration-target refresh | Process stays active; an existing map and registration target are not replaced; rejected filter output is empty rather than an unfiltered copy | Check coordinate units and outliers first; then adjust `vg_size_for_input` or `vg_size_for_map` only if the intended resolution allows it | Automated boundary, issue-fixture, valid-output parity, and real-component unsafe-then-safe tests |
| Pinned public MID-360 bag → verified map | Nightly Jazzy workflow runs the installed CLI with exact archive/bag identity and bounded output thresholds | Non-geometry evidence report, manifests, diagnosis, verification, and logs | Inspect the failed assertion and retained evidence; do not move the contract without review | Automated real-data E2E |

`run_manifest.json` is authoritative for terminal workflow status. Diagnosis
uses its `failed` or `interrupted` state even when the workflow stopped before
writing a recognizable ROS log signature.

## Classic scanmatcher VoxelGrid refusal boundary

PCL 1.12 and 1.14 check whether the product of the three voxel-grid divisions
exceeds signed 32-bit range. When it does, upstream `VoxelGrid` warns that
integer indices would overflow and returns the original, unfiltered cloud.
That fallback is unsafe as an application contract: downstream registration or
map processing can still receive the unexpectedly dense input. It was followed
by a `SIGFPE` process exit in public issue #69.

Classic scanmatcher now inspects every input before all five of its PCL
`VoxelGrid` call sites. It uses the effective float leaf size that PCL will use
and rejects before filtering when any of these stable reasons applies:

| Reason code | Meaning | First action |
| --- | --- | --- |
| `VOXEL_GRID_INVALID_LEAF_SIZE` | The configured value is non-finite, non-positive, outside float range, or so small that its inverse is non-finite. | Set the named leaf-size parameter to a finite value greater than zero. |
| `VOXEL_GRID_EMPTY_CLOUD` / `VOXEL_GRID_NO_FINITE_POINTS` / `VOXEL_GRID_NULL_INPUT` | No usable XYZ input exists. | Check the topic, PointCloud2 conversion, and range filter. |
| `VOXEL_GRID_NONFINITE_DENSE_CLOUD` | A publisher marked the cloud `is_dense=true` while at least one XYZ value is non-finite. PCL would not skip that point. | Remove the invalid point or publish the correct `is_dense=false` metadata. |
| `VOXEL_GRID_INDEX_OVERFLOW` | At least one absolute `floor(coordinate / leaf)` value cannot fit PCL's signed 32-bit voxel index. | Check units, coordinate origin, and outliers before changing resolution. |
| `VOXEL_GRID_LAYOUT_OVERFLOW` | The conservative voxel division product exceeds `2,147,483,647`. | Check units/outliers; if the span is intentional, increase the parameter named in the warning. |

`vg_size_for_input` controls scan and non-NDT target filtering.
`vg_size_for_map` controls initial-map and map-update filtering. The node does
not silently increase either value because that would change map resolution
and make output depend on an anomalous frame. A rejected input scan does not
advance the pose. A rejected map update leaves the previous map/target usable;
a later safe update can proceed. Warnings are throttled on recurring runtime
paths and include stage, point counts, bounds, divisions when available, the
parameter name, and a recovery action.

The bounded regression fixture uses only two synthetic points at
`[-200, -200, -10]` and `[200, 200, 10]` metres with a `0.1 m` leaf. Its
`4001 x 4001 x 201 = 3,217,608,201` layout reproduces the overflow class
without retaining the historical private rosbag. The wrapper returns empty
output and `VOXEL_GRID_LAYOUT_OVERFLOW`; a separate parity test proves that a
valid cloud produces the same filtered points as direct PCL. A real ROS 2
component regression then sends this unsafe cloud before a valid cloud to one
component instance: no map is published for the rejected timestamp, while the
later timestamp produces both a map and pose. The test passes on Humble/PCL
1.12 and Jazzy/PCL 1.14.

## Output storage boundary

`lidarslam-map run` requires at least 5 GiB free on the filesystem that will
contain the output directory. It checks the nearest existing parent before
workflow launch and prints the probe path, required GiB, and observed GiB.
This reserve is a deterministic disk-pressure refusal boundary, not an
output-size prediction: large bags and dense maps can require substantially
more.

Override the reserve only after sizing the expected map:

```bash
lidarslam-map run /path/to/rosbag2 \
  --output-dir /data/maps/session-01 \
  --min-free-space-gib 20
```

The value must be finite and greater than zero. When capacity is below the
budget, no output or `.partial` directory is created. Atomic manifest updates
remove an incomplete temporary file on write failure, so the previous durable
manifest is not replaced by truncated JSON.

After initialization, the runner allocates a hidden 2 MiB file with real
filesystem blocks. It removes that file immediately after the delegated map
workflow exits, before finalization. This small emergency reserve is not map
capacity: it exists so a filesystem that becomes completely full can still
record the terminal manifest and diagnosis. The reserve is removed from both
successful and failed outputs.

## Real bounded-filesystem exhaustion

The `bounded filesystem exhaustion` scheduled workflow complements synthetic
failure injection with the pinned public MID-360 bag and the installed Jazzy
product. It mounts the input read-only and confines every map artifact to a
32 MiB Docker tmpfs. The normal pinned output is substantially larger, so PCL
reaches the kernel-backed capacity limit during `/map_save` without consuming
the host filesystem.

The harness requires Docker, PyYAML and `python3-jsonschema`. It deliberately
requires `--image`; build that image from the same clean commit instead of
silently using a stale convenience image.

Run the same gate on a named Docker host:

```bash
python3 scripts/run_bounded_filesystem_exhaustion.py \
  /data/bags/driving-slam-mid360 \
  --image lidarslam-enospc:<exact-revision> \
  --tmpfs-mib 32 \
  --timeout-secs 600 \
  --hardware-label lab-amd64-jazzy \
  --evidence-dir output/bounded-filesystem/evidence
```

The harness exports only logs, manifests and diagnosis files to the unbounded
evidence directory; pointcloud geometry is never copied. Its
[`bounded-filesystem-exhaustion-v1` schema](schemas/bounded-filesystem-exhaustion-v1.schema.json)
requires one clean harness revision and an image built from that exact
revision, a nonzero product exit, the real PCL
`raw_fallocate ... returned 28` signature,
at most 10% free space on the 32 MiB tmpfs, a terminal failed manifest, a
storage-exhaustion
diagnosis and proof that no success was claimed. A ten-minute process deadline
and Docker stop/kill fallback bound a wedged failure path.

The exact-revision local execution and its input, image, capacity, terminal
state and evidence hashes are recorded in the
[bounded-filesystem exhaustion evidence ledger](evidence/bounded-filesystem-exhaustion-2026-07-29.md).
The first public post-integration workflow artifact passed on merge commit
`4151bf5`; the weekly recurrence remains enabled.

PCL reports the POSIX `ENOSPC` value as `raw_fallocate ... returned 28`; some
versions then print an unrelated `errno: 2` string. Diagnosis therefore keys
on the fallocate operation and return value instead of trusting that secondary
string.

## Termination boundary

The runner starts the delegated workflow in a separate POSIX process group.
For `SIGINT` and `SIGTERM`, it forwards the same signal to the entire group,
waits up to ten seconds for the group leader, then sends `SIGKILL` to any
remaining group members even if the leader exited first. The leader is always
reaped before terminal post-processing begins.

An external `SIGKILL`, kernel panic, power loss, or storage device loss cannot
be converted into a terminal manifest by user-space code. Such a run may
remain at `workflow_running`; `--resume` deliberately refuses it because an
original process could still own the artifacts. Confirm that no workflow
process is alive, retain the directory as evidence, and start a new output.

## Soak profiles

The operator-only soak harness exercises the same `lidarslam-map run` contract
as the golden path. It is deliberately not a fourth beginner entrypoint.
Choose a fixed one-hour or eight-hour profile and supply budgets tied to a
named machine. The `lidarslam` package declares GNU `time` as a runtime
dependency because it supplies the peak-RSS evidence:

```bash
python3 scripts/run_map_soak.py /data/bags/mid360 \
  --output-root /data/soak/one-hour-20260727 \
  --soak-profile one-hour \
  --hardware-label 'lab-amd64-7950x-64GiB' \
  --map-profile mid360_livox_smoke \
  --max-peak-rss-mib 4096 \
  --max-output-gib 40 \
  --max-dropped-inputs 0 \
  --max-iteration-secs 300 \
  --min-free-space-gib 100 \
  --telemetry-interval-secs 30
```

The harness repeats complete, collision-safe map runs until the profile
duration is reached. Each iteration records its command, exit code, GNU time
wall duration and peak RSS, output size, remaining free space, console log and
raw GNU time report. `--max-iteration-secs` is a required positive wall-time
budget for one complete map run. While an iteration is still running, the
harness samples free space and cumulative output size every 30 seconds by
default. The interval must be positive and cannot exceed 60 seconds.

Every sample is appended to `telemetry_samples` and atomically checkpoints
`soak_report.json`. If free space falls below `--min-free-space-gib` or output
growth exceeds `--max-output-gib`, the harness stops the entire timed process
group with `SIGTERM` and a bounded `SIGKILL` fallback, then attempts to
finalize a failed report. The last successful sample remains available even
when the iteration never produces normal GNU time evidence.

- [`soak-report-v4.schema.json`](schemas/soak-report-v4.schema.json) — current;
  adds the required per-iteration wall-time threshold, maximum observed
  duration, and `iteration_duration_within_budget` terminal check;
- [`soak-report-v3.schema.json`](schemas/soak-report-v3.schema.json)
  — published compatibility schema containing a non-secret machine
  fingerprint, harness revision and checksums, and the exact input/software
  identity copied from each successful product run;
- [`soak-report-v2.schema.json`](schemas/soak-report-v2.schema.json)
  — published compatibility schema for periodic telemetry reports;
- [`soak-report-v1.schema.json`](schemas/soak-report-v1.schema.json)
  — published compatibility schema for iteration-only reports.

Schemas v1, v2 and v3 remain published and immutable, so archived reports
retain a resolvable contract.

A passing v4 report requires both `provenance_recorded` and
`iteration_duration_within_budget`. The first successful
iteration fixes the input and software identities for the whole soak; any
later iteration with a different identity terminates the run as failed. The
machine ID is a SHA-256 fingerprint: private machine identifiers contribute
to stability but are never written to the report. Hostnames, usernames and
network addresses are not collected.

Operator `Ctrl-C` and service `SIGTERM` use the same process-group cleanup and
return `130` and `143`, respectively. Their terminal v4 report has
`interrupted` status. If the filesystem refuses the final atomic update, the
last successfully written running-state sample remains as recovery evidence.

When an iteration reaches `--max-iteration-secs`, the harness records a final
sample, terminates the whole timed process group, reaps its leader, and writes
a terminal v4 report with `failed` status. This converts a live player or
mapper stall into bounded, machine-readable evidence instead of allowing a
named one-hour or eight-hour profile to hang indefinitely.

The drop counter is a conservative count of documented console signatures for
message-filter drops, scan drops and queue/buffer overflow. One line can match
more than one signature, so the total can over-count; a zero only proves those
signatures were absent from the captured logs. A failed iteration, unreadable
telemetry, or exceeded RSS/output/drop budget stops the profile immediately
and preserves failed evidence. The duration gate passes only after the full
3,600 or 28,800 seconds have elapsed.

## Named-hardware execution evidence

The one-hour and eight-hour profiles were executed consecutively on
2026-07-27/28 using one clean merged revision, one fixed rosbag2 identity and
the named Jazzy machine
`sasaki-laptop-i5-1145G7-32GiB-jazzy-native`. Both v4 reports passed all eight
terminal checks.

| Profile | Iterations | Wall time | Longest iteration | Peak RSS | Output | Drops |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| one-hour | 86 / 86 | 3,638.645 s | 43.164 s / 300 s | 209.320 MiB / 1,024 MiB | 1,889,888,063 B / 5 GiB | 0 / 0 |
| eight-hour | 671 / 671 | 28,834.115 s | 45.732 s / 300 s | 210.871 MiB / 1,024 MiB | 14,553,359,036 B / 30 GiB | 0 / 0 |

The [auditable evidence ledger](evidence/real-data-soak-2026-07-28.md)
records the exact commit, input and report hashes, thresholds, free-space
minimums, reproduction commands and scope limitations. This closes the named
one-hour/eight-hour execution row for this hardware/input/profile combination;
it does not establish a universal performance or map-quality guarantee.

## Timestamp reversal and disorder before launch

`lidarslam-map doctor` and the `run` planner publish preflight schema v3 and
inspect the selected `PointCloud2` and `Imu` `header.stamp` streams before
starting ROS processes. This makes timestamp reversal a pre-launch diagnosis
instead of a mapper-time surprise. The scan is bounded at 100,000 records per
topic:

- `passed` means every metadata-declared selected record was checked;
- `sampled` means the bound was reached with no observed disorder;
- `failed` identifies unreadable, invalid or decreasing timestamps and records
  the reversal count and largest backward jump.

`failed`, `error`, and `unavailable` fail closed for timestamp-dependent
profiles. Correct the source timestamps or use the documented MID-360 stamp
rewriter, then rerun `doctor`; do not treat a sampled result as proof about
records beyond the scan bound.

The named real-bag and derived reversal execution is recorded in
[timestamp-order preflight evidence](evidence/timestamp-order-preflight-2026-07-29.md).

## Historical output migration and image rollback

Schema-v1 run manifests do not identify a safe resumable lifecycle state or
the original verification mode. `lidarslam-map migrate-manifest` therefore
fails closed unless the record is terminal and the operator supplies
`--verification required` or `--verification off`. Its exclusive atomic
output is a separate schema-v2 record at lifecycle stage `complete`; it is
only a compatibility view for inspection and automation.

Each release image now produces a schema-validated
`release-image-<distro>.json` and `rollback-plan-<distro>.json`. A locally
downloaded release-image record can be revalidated with
`lidarslam-map rollback-plan`; every generated registry command uses the
immutable `@sha256:` reference and reports that no moving tag is mutated.
The release publisher itself now pushes candidates by digest, verifies both
distro records, and uses an idempotent promotion contract: an absent version
tag is created, the same digest is reused, and a conflicting digest is refused
before any tag creation. The deterministic source bundle embeds a checksummed
`release-bundle-manifest-v1` inventory, while `release-promotion-v1` records
the pairwise promotion result.
The source and clean-install execution is recorded in the
[recovery command contract evidence](evidence/recovery-command-contract-2026-07-29.md).

## Published recovery evidence

The first tagged release published the rollback assets and passed
`scripts/check_published_release.py --require-published`. The exact tag,
workflow, assets, and audit result are recorded in the
[published v0.9.0 release evidence](evidence/published-release-v0.9.0-2026-07-31.md).

That read-only audit validates the stable GitHub Release, immutable tag
commit, both image records, both digest-only rollback plans, applied
two-image promotion, and the deterministic bundle's complete embedded hash
inventory. It independently resolves both live GHCR version tags and rejects
publication evidence if either tag no longer names its recorded digest. Its
report follows
[`published-release-v1.schema.json`](schemas/published-release-v1.schema.json).
A missing release is distinct from a network/API or registry failure, and a
release with one missing, malformed, duplicated, cross-version, or moved-image
asset is `BLOCKED`.

See the [pinned real-data E2E contract](real-data-e2e.md) and the
[v1.0 readiness audit](v1-readiness.md) for the remaining distribution and
independent-adoption gates.
