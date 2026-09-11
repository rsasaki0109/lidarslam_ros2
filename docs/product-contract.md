# Product Contract

`lidarslam_ros2` is an offline ROS 2 map-authoring product. Its maintained
product promise is:

> Given a compatible rosbag2 and an explicit sensor configuration, produce a
> diagnosable, reproducible point-cloud map bundle that can be checked before
> it is loaded by Autoware-compatible map tooling.

This contract separates the supported product surface from the repository's
larger research surface. A feature existing in the repository does not by
itself make that feature part of the product contract.

## Supported outcome

A successful product-path run produces an output directory containing:

- `pointcloud_map/` and `pointcloud_map_metadata.yaml`;
- `map_projector_info.yaml`;
- `traj_corrected.tum`;
- `verify_autoware_map.log`;
- `autoware_map_diagnosis.md`;
- `autoware_map_diagnosis.json`;
- `run_manifest.json`, with input/output checksums and execution identity;
- `first_map_validation_receipt.json` and `.md`, a privacy-bounded,
  issue-ready proof summary derived from the finalized run evidence;
- `lanelet2_map.osm` when lanelet generation is enabled and succeeds.

`verify_autoware_map.py` must report `PASS` before the point-cloud bundle is
treated as valid. Generated lanelets are a starting point for map authoring,
not surveyed road semantics; review them before use.

The versioned machine-readable contracts are published with the
[golden-path CLI documentation](golden-path-cli.md#versioned-json-contracts).
Existing output directories are immutable to the runner: operators must choose
a new name, and in-progress work is isolated under a `.partial` sibling. The
only exception is explicit `--resume` of an incomplete schema-v2 lifecycle:
the runner revalidates the original execution identity and completes terminal
post-processing without rerunning SLAM or replacing map artifacts.
Signal handling, preserved failure state, recovery actions, and still-open
Phase 3 gates are tracked in
[Operational reliability](operational-reliability.md).
The flagship installed workflow is also exercised by the
[pinned real-data E2E gate](real-data-e2e.md).

## Official entrypoints

These are the only four beginner-facing product workflows. Other scripts and ROS
launch files are advanced, benchmark, migration, or research interfaces.

Running `lidarslam-map` without arguments on an interactive terminal opens a
choice-reducing home, not another mapping workflow. It offers only the existing
fixed demo, own-bag `start`, previous-session catalog, read-only installation
`doctor`, and complete help. It shows the exact delegated command first,
requires an explicit `yes` before demo download or writes, and retains `start`
calibration review. Doctor uses no network and writes no files, so it runs
without an unnecessary confirmation. Non-interactive no-argument use preserves
stderr help and exit code `2`; EOF, cancellation, and ambiguous confirmation
delegate nothing.

| Goal | Official command | Contract |
| --- | --- | --- |
| Check an installation before finding a bag | `lidarslam-map doctor` | Checks curated runtime files, matching installed prefix, supported ROS, bag-reader availability, and demo storage; returns path-free JSON and stable recovery actions without network or writes |
| Try the fixed public demo | [Docker First Map](getting-started.md#docker-first-map-no-ros-2-workspace) without ROS, or `lidarslam-map demo [work_dir]` after a source install | Downloads the tracked MID-360 demo with progress and writes a verified map; the installed command can open the offline review |
| Map your own compatible rosbag2 | `lidarslam-map start <rosbag2_dir>` for people; `lidarslam-map run <rosbag2_dir> --output-dir <dir>` for automation | Detects and pins consumed sensor topics, requires setup confirmation and RKO calibration confirmation when applicable, runs the normal atomic lifecycle, verifies and diagnoses the map, then opens the offline browser view |
| Return to and compare local sessions | `lidarslam-map sessions`; select two cards or run `lidarslam-map compare <left> <right>` | Reopens schema-valid session pages and compares recorded readiness, setup, and artifacts without inferring a score or winner |
| Prepare a maintainer or first-map report | `lidarslam-map support <session_bundle>` for maintainer support; `lidarslam-map report <session_bundle-or-demo-output>` after a verified run | Creates a review-before-sharing support ZIP, or revalidates PASS evidence and prints the exact independent-validation handoff without writing or uploading |

`bash scripts/docker_map_bag.sh <rosbag2_dir>` is the no-install delivery
wrapper for the existing “Map your own compatible rosbag2” workflow, not a
fifth product workflow. It owns read-only/writable mounts, user identity, and
external-network isolation, then invokes `lidarslam-map start /input` inside
the selected published image.
Its dry-run performs no Docker call or write, and forwarded product arguments
cannot replace the launcher-owned output, viewer, or verification paths. A
live run creates no output until the image passes `start --help`; the launcher
then uses that image's local immutable ID and refuses a zero exit without the
session index, run manifest, and validation receipt.
The additive `--json` option is valid only with `--dry-run` and emits the
versioned `docker-map-bag-plan-v1` contract to stdout without Docker, network,
or filesystem writes. It reports the input as read-only, does not create the
output directory, and leaves image identity and contract preflight deferred to
the live run; the payload contains local paths and is not an issue-reporting
format.

Beginning with the first finalized release after v0.9.0, that exact host
launcher is also a direct `lidarslam-map-docker` release asset. The generated
asset is bound to the release tag and source commit and defaults to the
matching immutable versioned Humble/Jazzy image tag. The source-tree launcher
continues to identify itself as `development (working-tree)` and uses the
moving development image only when no explicit `--image` is supplied. A local
source script must never be mistaken for an attested release asset.

A verified terminal session adds the compatibility-keyed `share` action,
displayed as **Prepare a first-map report**, whose command is
`lidarslam-map report <session_bundle>`. Fixed Docker/source demos may pass
their output directory instead when no `session.json` page is created. The older
`support <session_bundle> --first-map` spelling remains compatible. This is not a fifth
workflow or an acceptance claim. It re-hashes the receipt-bound manifest,
diagnosis, and verification log, then prints only the copy-ready verification
summary, reviewed receipt path, and canonical issue form. It performs no write,
browser open, upload, or remote mutation. Recovery sessions instead add a
`support` action for the existing privacy-bounded ZIP.

After a successful `start`, the terminal prints a bounded **Session summary**
from the retained session index: evidence-backed verification status, the
offline viewer or session-page path, run manifest, first-map receipt, and one
safe `Next:` command. Verified sessions additionally print the read-only
`Report:` preparation handoff. `--viewer none` reports the retained session page and the
exact command needed to reopen the map; it does not weaken verification.

For automation, add `--json` to the first-map support command. It returns the
same read-only handoff as the schema-valid
[`first-map-handoff-v1`](schemas/first-map-handoff-v1.schema.json) object,
including safe environment hints and the four fields the operator must still
complete. The structured handoff is a local-only helper and contains the
session's local receipt path; attach only the reviewed receipt it names.
`--output` remains rejected in this mode.

For automation, use an explicit `--output-dir`; run `lidarslam-map doctor`
before a long run. The repo-local `./scripts/lidarslam` wrapper and installed
`ros2 run lidarslam lidarslam-cli` shim expose the same own-bag contract and do
not add beginner workflows. Installation details are in
[Distribution and installed CLI](distribution.md).

The source-tree onboarding helper also has a machine-readable preview:
`bash scripts/source_quickstart.sh --dry-run --json` emits the versioned
[`source-quickstart-plan-v1`](schemas/source-quickstart-plan-v1.schema.json)
contract only with `--dry-run`. It performs no network, package-manager,
submodule, build, demo, or filesystem write and contains local paths, so the
raw payload is for local automation rather than issue reports.

The absolute installed `lidarslam-map` launcher and a repo-local wrapper with a
matching built workspace activate only their own aggregate `setup.bash` before
delegation. This makes the first command in a fresh terminal independent of a
remembered shell setup while preserving normal explicit activation for the
short command name and other ROS tools. Source-layout discovery is accepted
only when the install tree contains the matching curated CLI resource; an
unrelated parent workspace is never sourced.

`lidarslam-map demo [work_dir]` is the installed public-data activation path.
It pins the Zenodo dataset identity and attribution, checks the configured
free-space floor before download, re-hashes the archive and the two
security-relevant extracted bag files on every live run, and delegates to the same
`run_first_map_demo.sh` implementation used by Docker. `--dry-run --json`
writes nothing and follows `first-map-demo-plan-v1`; live `--json` is rejected.
Its storage projection records exact `additional_bytes_required` per unique
volume. A refusal rounds the shortage upward and returns the complete
shell-quoted retry command instead of asking the operator to rebuild it.
With `--dry-run`, `--output PLAN` may retain the human card or JSON plan using
exclusive creation; an existing plan path is refused and the demo workspace
remains untouched.
A retained output is reused only when its schema-valid first-map receipt can be
rebuilt byte-for-byte from current manifest, diagnosis, and verifier evidence.
A schema-valid terminal interruption is resumed with `demo --resume`; this
finishes post-processing only and refuses any lifecycle stage where mapping
may still be active. Failure output names the last durable stage and provides
copy-ready inspect, resume, or fresh-output recovery without overwriting
evidence.
A viewer failure remains separate from a verified map result.

The `start` command is the recommended human path: it keeps calibration, the
exact command, and profile selection visible before execution. Add `--yes` only
after review for a non-terminal launcher, or use `run` directly for automation.
When no maintained profile is safe, it returns a schema-bound NOT READY report
with stable reason/finding codes, concrete next actions, and no output files.
After mapping has started, a non-zero result instead preserves the setup and
available run evidence and writes `map_session_recovery.json`. Its stable reason
code distinguishes storage, interruption, ROS/TF/map-save, GNSS, verification,
post-processing, and unknown workflow failures. It gives an exact safe resume
when manifest-v2 permits one; otherwise it keeps an inspect command and retries
the unchanged pinned setup only into a fresh output directory.
Every delegated `start` writes a schema-bound `session.json` and derives one
offline `session.html` landing page. Its status is exactly `running`,
`verified`, `unverified`, or `action_required`. While running, the page opens
before runner delegation, refreshes itself, and advances only from the atomic
run-manifest lifecycle; no estimated completion percentage is invented. For a
terminal state, the first action is the recommended copy-ready command.
Verified sessions link the generated 3D review, unverified sessions state that
verification was skipped and offer a fresh verified output, and failed sessions
retain `map_session_recovery.json` as their detailed source. The page is not a
second source of truth, loads no network resources, and opens best-effort in
browser mode. Progress, page, or viewer failure never replaces the map result
or authoritative JSON evidence.

Its map-quality summary is likewise derived, not scored heuristically. Four
operator-facing cards map back to the seven required checks in a schema-valid
first-map validation receipt. A diagnostic verification-off run is explicitly
`not_verified`; a missing, malformed, or semantically incomplete receipt is
`unavailable`. Neither state may be promoted to quality PASS from file presence
alone.

For a run that completed but looks wrong, `lidarslam-map inspect --symptom`
adds one bounded, user-reported visual symptom to the same diagnosis contract.
The five stable codes cover a spinning/spiralling map, drifting/oscillating
pose, early map termination, sparse map, and missing viewer output. The card
checks sensor/time/calibration/TF contracts before tuning, retains only
shell-safe product commands, and never edits parameters or starts mapping.
Its evidence basis is explicitly
`USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED`: it is not a root-cause, hardware
support, or accuracy claim, and the original session remains immutable.

`lidarslam-map sessions [sessions_root]` is the return-to-work path. It derives
a newest-first local catalog from direct child bundles containing a
schema-valid `session.json`; it does not infer status from directory names.
Bundle, JSON, page, and catalog-output symlinks are not followed, each input is
bounded to 2 MiB, and at most 200 entries are shown. The generated
`sessions.html` has no network dependencies and links only regular local
session pages. `--json` performs no write and opens no browser. Invalid entries
are counted but their untrusted contents are not rendered.

`lidarslam-map compare <left_session> <right_session>` derives a fixed 14-row
report from two schema-valid session indexes. Readiness comes from
`session.json`; setup fields come from `sensor_setup.json` only while its
session, bag, output, profile, and parameter-snapshot identities still match.
Missing, stale, malformed, or mismatched evidence is `unavailable`, never
inferred from file presence. Results are only `same`, `different`, or
`unavailable`; there is no numeric score and no selected winner. The generated
HTML is self-contained, refuses symlink/non-comparison replacement, and links
only regular retained session pages. `--json` is read-only, and browser-open
failure does not change a successful comparison result.

`lidarslam-map support <session_bundle>` projects one valid session into the
`support-bundle-v1` contract and a fixed three-member ZIP. Setup evidence is
included only while its identities still match; artifact symlinks and paths
outside the session evidence roots are never read. Map geometry, raw sensor
data, raw logs, parameter contents, local paths and command credentials are
excluded. A retained visual symptom is projected only as its fixed code and
explicit user-reported evidence basis; its title, checks, commands, and free
text remain local. The issue body never presents that code as an automatically
diagnosed root cause. ZIP creation is atomic and refuses existing or symlink
targets;
`--json` is read-only. The command never uploads or changes remote state, and
every result requires human review before public sharing.

`run_autoware_quickstart.sh` remains an advanced viewer/dogfood compatibility
route for the older NTU VIRAL and Autoware runtime flow. It is intentionally
separate from the fixed first-map path above.

After a successful run, `lidarslam-map view <output_dir>` can open the
completed output in Autoware or Foxglove. Viewing is optional post-processing,
and its failure does not alter the map-run manifest.

## Input contract

The general product path accepts a rosbag2 directory containing
`metadata.yaml`. `start`, `setup`, and `run` share four maintained input
profiles:

- `PointCloud2 + Imu` through generic or MID-360 RKO-LIO graph profiles;
- `PointCloud2 + NavSatFix` through the GNSS smoke profile;
- `VelodyneScan + Applanix GSOF49` through the packet conversion profile.

The primary RKO-LIO path additionally requires:

- `sensor_msgs/msg/PointCloud2`;
- `sensor_msgs/msg/Imu`;
- valid, monotonic timestamps;
- a known transform between LiDAR, IMU and base frames;
- enough free disk space for the input bag, intermediate clouds and output map;
  `run` enforces a 5 GiB output-filesystem reserve by default and accepts a
  deliberately sized `--min-free-space-gib` override.

Topic presence is necessary but not sufficient. A bag with the right message
types can still require a sensor-specific point-time field, calibration,
frame override or parameter profile. Before recommending RKO-LIO, the preflight
reads the first record on the selected `PointCloud2` topic and verifies
FLOAT32 `x`, `y`, and `z` plus a supported per-point timestamp field named
`t`, `timestamp`, `time`, or `stamps`. It does not certify timestamp units,
calibration, or TF connectivity. Before launch it also checks up to 100,000
`PointCloud2` and `Imu` records per selected topic for invalid or decreasing
`header.stamp` values. A bounded clean sample is not a full-bag monotonicity
proof, and the machine report distinguishes `sampled` from `passed`.

## Support tiers

| Tier | Scope | Evidence and expectation |
| --- | --- | --- |
| Validated | Tracked MID-360 Docker demo; NTU VIRAL Ouster/VN-100 quickstart | Fixed public input, parameters and expected artifacts; release documentation records measured accuracy |
| Maintained-compatible | Generic record-verified `PointCloud2 + Imu` through the beginner wrapper on ROS 2 Humble and Jazzy | Built and tested in CI; the operator supplies correct timestamp units, calibration, frames and sensor parameters |
| Evaluation | GNSS/Applanix packet paths, radar degeneracy presets, HILTI, coloured maps and optional loop detectors | Reproducible research or benchmark evidence exists, but it is not a universal plug-and-play hardware guarantee |
| Out of scope | Safety-certified localization, autonomous-driving validation, arbitrary proprietary bags, Windows native builds | No product support promise |

See [Support](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/SUPPORT.md)
for reporting requirements and
[Operator Workflows](workflows.md) for advanced configuration.

## Compatibility and change policy

- Humble on Ubuntu 22.04 and Jazzy on Ubuntu 24.04 are the supported source
  build targets.
- The latest tagged `0.x` release and `develop` receive fixes. Older `0.x`
  releases may require upgrading before a fix is provided.
- Product entrypoint flags and output filenames must not be removed silently.
  Deprecations are documented in release notes for at least one tagged release.
- New algorithms begin default-off. Promotion to a maintained preset requires
  paired real-data evidence and explicit negative checks.
- Offline determinism claims apply only to the runners and artifacts named in
  the corresponding release evidence; they do not imply bitwise-identical
  live ROS scheduling.

## Non-goals

The product contract does not promise:

- universal best accuracy on every dataset;
- automatic calibration of an unknown sensor rig;
- production localization or fail-operational autonomy;
- surveyed lanelet semantics;
- compatibility with every research script or optional dependency;
- that a verified file bundle is safe for deployment without operator review.

## Evidence and escalation

Accuracy claims and gates live in
[Benchmarking and Release Gate](benchmarking.md) and
[Comparison](comparison.md). Security-sensitive reports follow
[SECURITY.md](https://github.com/rsasaki0109/lidar_slam_ros2/security/policy);
usage questions and reproducible defects follow
[SUPPORT.md](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/SUPPORT.md).
Independent onboarding evidence follows the
[first-map validation contract](external-first-map-validation.md); its tracked
ledger remains separate from maintainer-operated demos and CI evidence.
The cross-phase completion claim is generated by the
[v1.0 readiness audit](v1-readiness.md), which fails closed over every product
dimension and the external ledger. Tagged publication and recovery assets are
separately verified by the read-only published-release audit; a GitHub Release
or image tag existing without its complete, cross-consistent evidence set is
not sufficient.
