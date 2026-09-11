# Benchmarking And Release Gate

This page describes the recommended benchmark path and the release/readiness
gate used for the default permissive workflow.

> **Reading guide (2447 lines).** Start here, then jump to what you need:
>
> - New to benchmarking? Read [Recommended Benchmark](#recommended-benchmark)
>   and [FAST-LIVO2 head-to-head](#fast-livo2-head-to-head) only.
> - Making a competitive claim? Read [Claim-eligible evidence
>   bundles](#claim-eligible-evidence-bundles) and [Competitive victory
>   evidence (schema v2)](#competitive-victory-evidence-schema-v2).
> - Running M6a10 runtime gates? See [Runtime phase contract
>   (M6a10-v1 compatibility)](#runtime-phase-contract-m6a10-v1-compatibility)
>   and the M6a10 subsections below it.
> - Fresh-holdout / GT-blind procedure? See [Fresh-holdout authorization and
>   GT-blind claim gate](#fresh-holdout-authorization-and-gt-blind-claim-gate).
>
> Section map: Claim-eligible evidence bundles → Recommended Benchmark →
> FAST-LIVO2 head-to-head → Runtime phase contract (M6a10-v1…) → Competitive
> victory evidence (schema v2) → Fresh-holdout authorization…

## Claim-eligible evidence bundles

The final competitive suite gate accepts a claim only when the evidence
declares one canonical bundle root and a deterministic
`competitive_slam_evidence_bundle` manifest.  The verifier is
[`scripts/verify_competitive_evidence_bundle.py`](../scripts/verify_competitive_evidence_bundle.py),
and its manifest shape is registered in
[`competitive_evidence_bundle_v1.schema.json`](../configs/slam_benchmark_profiles/competitive_evidence_bundle_v1.schema.json).
It reopens the required `input`, `result`, `config`, `calibration`,
`revision`, `scorer`, `trajectory`, `resource`, `map_metric`, and `failure`
artifacts, checking normalized paths, root containment, regular-file and
single-link identity, size/SHA-256, and deterministic sidecars.  Missing,
extra, aliased, symlinked, hard-linked, or drifted files fail closed.
The checked-in v1 manifest contract is intentionally strict: legacy v1
manifests written before the per-run index existed are not claim-compatible
and are rejected; there is no fallback that treats one role artifact as
campaign-wide coverage.

Claim manifests must also contain a non-empty `run_artifact_bindings` index.
Each `(system, dataset, run_index)` identity appears exactly once and names
separate trajectory, map, resource, and non-GT score-record files with their
own size/SHA-256 and sidecar.  The v2 suite constructs the expected index from
every complete scored run and passes those opaque hashes into the verifier;
missing, extra, duplicate, or byte-drifted run entries therefore fail before
claim promotion.  The score record is the canonical JSON
`competitive_run_score_v1` projection of the completion, APE, runtime, and
map values used by the gate (canonical JSON plus one newline); its expected
SHA is computed before bundle verification, so editing a metric without
regenerating the sealed score bytes is rejected.
The index is part of the canonical manifest and all referenced files are
included in the root inventory, so one shared trajectory or resource cannot
silently cover a whole campaign.  The verifier does not parse those bytes or
ground truth; it only checks hashes and sidecars.  Numeric metric semantics
remain the separate evaluator/scorer contract, while the sealed score-record
hash binds the exact non-GT values consumed by this gate.

The verifier hashes bytes and checks identity metadata only; it does not parse
ground truth, trajectories, maps, or scorer output.  Ground-truth roles and
path components are rejected before any listed artifact is opened.  Scoring is
therefore still a separate authorized stage.  Existing benchmark receipts
remain `NOT_READY`/fail-closed until a real bundle passes both this verifier
and the metric suite gate; adding a verifier does not promote historical
evidence.

### Deterministic bundle composition

Already-scored, non-GT evidence can be materialized with
[`scripts/compose_competitive_evidence_bundle.py`](../scripts/compose_competitive_evidence_bundle.py).
The composition spec must provide the evidence receipt, all ten global artifact
roles, and an explicit `run_artifact_bindings` entry for every
`(system, dataset, run_index)`.  Each source descriptor names an absolute,
non-symlink source root, a normalized relative path, byte count, and SHA-256;
the composer reopens every source with no-follow descriptors, rejects
hard-links/path traversal/GT components/hash drift, and refuses an existing
output root.  The revision JSON is checked against the declared system pins,
and each run's canonical non-GT score projection must match its pre-existing
`score_artifact_sha256`, so edited metrics cannot be silently resealed.

The composer never reads GT or invokes a scorer.  It copies verified bytes into
a fresh staging root, writes deterministic per-file sidecars, score JSON, the
canonical manifest, and an immutable composition receipt before atomically
sealing the new root.  Without an independently authorized holdout chain the
result is intentionally `NOT_READY` and `claim_eligible: false`:

The machine-readable input contract is
[`competitive_evidence_bundle_composer_v1.schema.json`](../configs/slam_benchmark_profiles/competitive_evidence_bundle_composer_v1.schema.json);
the output manifest remains governed by
[`competitive_evidence_bundle_v1.schema.json`](../configs/slam_benchmark_profiles/competitive_evidence_bundle_v1.schema.json).

```bash
PYTHONDONTWRITEBYTECODE=1 env -u PYTHONPATH \
  python3 scripts/compose_competitive_evidence_bundle.py \
  --spec /path/to/composition-spec.json \
  --output /path/to/new-bundle-root \
  --receipt /path/to/composition-receipt.json
```

`--allow-claim` only requests the existing authorization verifier; it does not
fabricate an attestation and still fails closed when the external authorization
is absent.  Candidate roots and receipts are not benchmark evidence until the
separate claim-time verifier, metric gate, and authorization chain pass.
Failed compositions retain a sealed sibling staging root with
`composition.failure.json` for diagnosis; no partial root is promoted or
overwritten.

### Post-score handoff from real campaign results

The older
[`scripts/compose_competitive_result.py`](../scripts/compose_competitive_result.py)
is an aggregate/report writer.  Its legacy result does not identify every
`(system, dataset, run_index)` trajectory, map, resource receipt, or canonical
`competitive_run_score_v1` byte record, so it is not a claim-bundle input.
After an independently authorized scorer has finished, the additive
[`scripts/prepare_competitive_evidence_bundle_handoff.py`](../scripts/prepare_competitive_evidence_bundle_handoff.py)
converts the already-scored v2 evidence into the missing handoff without
running the scorer.  Its request schema is
[`competitive_evidence_bundle_handoff_v1.schema.json`](../configs/slam_benchmark_profiles/competitive_evidence_bundle_handoff_v1.schema.json).

The request must contain an explicit source descriptor (absolute root,
normalized relative path, expected byte count, and SHA-256) for all ten global
roles and for trajectory/map/resource artifacts of every complete run.  The
`expected_runs` list is also precommitted; its exact set of
`(system, dataset, run_index)` keys must equal the scored evidence and source
index, with no duplicate, missing, or extra run.
generator reopens those files, rejects aliases, symlinks, hard links,
traversal, drift, incomplete or duplicate coverage, and computes each score
digest from the same canonical projection used by the suite gate.  It emits a
sanitized evidence file, composition spec, and sealed handoff receipt; source
bytes are not copied and no dataset or GT path/content is opened.  A separate
GT-safe post-score attestation receipt is required.  Its declared input
evidence SHA must match the bytes read by the handoff; the generated spec also
binds the derived sanitized-evidence SHA that the composer consumes.  Only
the receipt's opaque SHA, scorer fingerprint, and these byte identities enter
the handoff; this local tool does not verify an external signature.  The output remains
`claim_eligible: false`/`NOT_READY`; fresh-holdout authorization, bundle
composition, and the final metric verifier are still independent gates.

For a synthetic or authorized post-score root, invoke:

```bash
PYTHONDONTWRITEBYTECODE=1 env -u PYTHONPATH \
  python3 scripts/prepare_competitive_evidence_bundle_handoff.py \
  --request /path/to/handoff-request.json \
  --output /path/to/new-handoff-root
```

The checked-in campaign has no such complete, authorized handoff and remains
`NOT_READY`; synthetic tests cover the positive digest/coverage path and
GT/path/source-drift rejection.

To require the bundle at the final v2 gate, use `--claim-eligible` with
`evaluate_competitive_suite_gate.py`; report-only invocations remain compatible
and explicitly record `claim_eligible: false`.

### Claim-time execution freeze revalidation

The v2 suite gate performs a second, claim-only reopen of the registered
execution-selection receipt through
[`scripts/check_competitive_execution_selection.py`](../scripts/check_competitive_execution_selection.py).
The receipt path and SHA-256 registration check alone is not an execution
freeze proof.  This preflight must also validate the bytes and identities for
every pinned input/calibration/config/runner/scorer/resource file, system
revision, Release/toolchain/container digest, machine/thread policy, rival and
dataset source closures, and the GT-blind/resource lineage.  Any missing,
stale, or mismatched identity is fail-closed for a claim.  Report-only v2
receipts deliberately record this check as `NOT_REQUESTED`; they cannot become
claim-eligible without the claim-time preflight, fresh-holdout authorization,
and canonical bundle verifier all passing.

## Recommended Benchmark

The standard benchmark path for this repository is:

```bash
bash scripts/download_ntu_viral_tnp01.sh --dry-run
bash scripts/download_ntu_viral_tnp01.sh
bash scripts/run_rko_lio_graph_benchmark.sh
```

The dry run performs no write or network request. It shows which download,
extraction, conversion, and restamping phases remain; pins the official archive
size and checksum; and compares the conservative additional working-set
estimate with the destination filesystem. A fresh full preparation currently
needs about 49 GB free because the official archive, ROS 1 bag, converted
rosbag2, and RKO-LIO restamped bag coexist until completion. If the repository
filesystem is smaller, the planner looks only for sufficiently large
attached-but-unmounted Linux hotplug/USB filesystems. It reports partition size
as `UNVERIFIED_UNTIL_MOUNTED`, never treats that size as free capacity, and
selects one mount action without mounting or probing the device itself.

After mounting the reported device, use the device identity directly:

```bash
udisksctl mount -b /dev/sda1
bash scripts/download_ntu_viral_tnp01.sh \
  --dest-device /dev/sda1 \
  --dry-run
```

`--dest-device` resolves exactly one current mountpoint, appends `ntu_viral`,
and reruns the real free-space check. Remove only `--dry-run` after `READY`.
The discovery path creates no directory, starts no network request, does not
request or bypass authorization, and preserves the selected conversion options
in its copy-ready preflight and live commands.

For a manually managed filesystem, keep the data out of the checkout with:

```bash
bash scripts/download_ntu_viral_tnp01.sh \
  --dest /path/on/large-disk/ntu_viral
bash scripts/run_rko_lio_graph_benchmark.sh \
  --bag /path/on/large-disk/ntu_viral/tnp_01_points_restamped_vn100_rosbag2 \
  --reference-bag /path/on/large-disk/ntu_viral/tnp_01_rosbag2
```

The live acquisition fails before starting `wget` when the destination cannot
hold its remaining phases. A cached archive is never extracted until its exact
official byte count and MD5 identity pass. The shared storage helper and NTU
acquisition script are both included in the curated release bundle.

## GLIM cross-validation

Use the existing comparison harness when the same rosbag2 input should be run
through both products:

```bash
bash scripts/compare_with_glim.sh \
  --bag /path/to/rosbag2 \
  --out-dir output/compare_glim
```

GLIM is a cross-validation reference in this workflow, not ground truth. A
fresh GLIM trajectory is cached only after its TUM structure and timestamps
pass validation. If a later fresh GLIM run does not produce a trajectory, the
harness accepts a fallback only when all of the following match:

- the complete rosbag2 directory bytes and relative file layout;
- the effective GLIM configuration bytes;
- the Docker image ID, or the selected local GLIM runtime artifacts;
- topics, mode, preset, IMU/viewer/OMP options; and
- the comparison harness and cache-helper implementations.

Each entry has a schema-validated manifest that binds this path-free identity
to the trajectory SHA-256, byte count, and pose count. A missing, contradictory,
symlinked, malformed, or modified artifact is a cache miss; the old
path/topic-only cache format is never imported. The run records
`glim.cache.status` and the key in `metrics.json`, and writes the observed
identity to `glim_cache_identity.json` under the run directory.

Use `--no-glim-cache` when a fresh GLIM execution is mandatory. A verified
cache hit may support technical continuity during a failed fresh run, but it
does not prove current GLIM installation usability, current runtime success, a
new benchmark result, or a comparative winner.

## Newer College Maths-Hard

`newer_college_math_hard` is the tightest blocking release profile. It refers
to the Multi-Camera Newer College **Maths-Hard** sequence: Ouster OS0-128,
243.7 seconds, and approximately 320.6 m. Its reference is the official 10 Hz
LiDAR ground-truth trajectory produced by registering each Ouster cloud to the
survey-grade Leica BLK360 prior map. It is not prism ground truth.

The dataset is CC BY-NC-SA 4.0 and the maintainers distribute it through the
[official request form](https://ori-drs.github.io/newer-college-dataset/download/).
The repository must not silently scrape, rehost, or relabel those
non-commercial assets. Before running this profile, request and retain all
three collection-specific inputs:

- the Collection 3 `Maths-Hard` ROS bag;
- the LiDAR ground-truth CSV, whose columns are epoch seconds, nanoseconds,
  position xyz, and quaternion xyzw;
- the matching Collection 3 calibration files.

The expected sensor topics are `/os_cloud_node/points` and
`/os_cloud_node/imu`. Convert the delivered ROS 1 bag without changing message
timestamps:

```bash
rosbags-convert \
  --src /path/to/maths-hard.bag \
  --dst /path/to/math_hard_rosbag2
```

Generate immutable TUM and metadata artifacts from the official CSV. The
body-to-reference translation must come from the matching calibration; all
three values are required so the command cannot silently assume identity:

```bash
python3 scripts/generate_newer_college_reference.py \
  --csv /path/to/maths-hard-lidar-gt.csv \
  --calibration /path/to/collection3-calibration.yaml \
  --output /path/to/math_hard_gt.tum \
  --metadata /path/to/math_hard_reference.json \
  --body-to-reference-x <metres> \
  --body-to-reference-y <metres> \
  --body-to-reference-z <metres>
```

Create the matching RKO-LIO parameter YAML from the same calibration, then run:

```bash
bash scripts/run_rko_lio_graph_benchmark.sh \
  --bag /path/to/math_hard_rosbag2 \
  --reference-tum /path/to/math_hard_gt.tum \
  --reference-meta /path/to/math_hard_reference.json \
  --lidar-topic /os_cloud_node/points \
  --imu-topic /os_cloud_node/imu \
  --base-frame base \
  --rko-param /path/to/rko_lio_math_hard.yaml \
  --lidarslam-param graph_based_slam/param/graphbasedslam_indoor.yaml \
  --output-dir /path/to/benchmarks/newer_college_math_hard_<commit> \
  --run-name newer_college_math_hard_<commit> \
  --skip-reference-gen \
  --reference-source newer_college_math_hard_icp_map_gt
```

Do not invent an identity extrinsic or a reference-frame offset. The
collection calibration must determine the RKO-LIO LiDAR/IMU-to-`base`
parameters and the `body_to_reference_translation_m` recorded in the reference
metadata. A run without those exact inputs is useful for diagnosis but is not
eligible for the 0.10 m release gate. `--fail-on-profiles` prints this
remediation path when exact candidate-commit evidence is absent.

The maintained Maths-Hard profile also sets:

```yaml
double_downsample: true
legacy_voxel_downsample: true
```

This is a measured dataset-specific compatibility setting, not the product
default. The RKO-LIO v0.3 hash-sorted first pass regressed this sequence from
0.081 m to 0.141 m APE. Compatibility mode restores the complete pre-v0.3
two-pass sampler; omitting it does not reproduce the blocking profile. The
modern default remains `false` and is separately covered by the NTU VIRAL and
RTK-SLAM release profiles. See the
[clean-candidate evidence](evidence/rko-voxel-compatibility-2026-07-31.md).

### Degenerate-LIO SOTA track

The preregistered public degeneracy track is defined in
`configs/slam_benchmark_profiles/degenerate_lio_sota_v1.yaml`. It begins with
ENWIDE TunnelS/TunnelD and forbids radar, wheel odometry, GNSS, cameras,
per-sequence tuning, and scale alignment. Download exact official inputs with:

```bash
bash scripts/download_enwide.sh \
  --sequence tunnel_d \
  --dest datasets/enwide \
  --convert
```

The profile remains report-only until all ENWIDE and GEODE degenerate
sequences, pinned rivals, and the hidden holdout are complete. See
`docs/research/enwide-sota-benchmark-plan-2026-07.md` for the claim policy.
Use `scripts/run_enwide_sota_benchmark.sh` for the fixed three-repetition
candidate run; sensor and scoring choices are deliberately not command-line
options.

### Radar-less tunnel frontend A/B

The radar-less tunnel research track has a frontend-only control/candidate
runner. It uses isolated DDS domains, stops `offline_node` with `SIGINT` after
odometry becomes quiet, and records the exact parameter layers, Git SHAs, TUM
trajectories, and comparison metrics:

```bash
bash scripts/run_radarless_tunnel_ab.sh \
  --sequence tunnel \
  --output-root /media/<ssd>/benchmarks/radarless_tunnel_adaptive_v1
```

Use `--candidate-param name:=value` for a focused override and `--dry-run` to
freeze the commands without starting ROS. Run `--sequence fog` as the first
negative check. HILTI exp07 and MID-360 are also supported with explicit
`--bag`, topic, base-parameter, and optional reference arguments. The runner
intentionally rejects exp02, exp03, and exp21 because they are reserved final
holdouts.

`comparison.json` includes endpoint/path metrics, time-aligned reach-ratio
quantiles after the first 10 m of reference motion, and an SE(3)-aligned
translation delta. It also records candidate overrides and the velocity-blend
diagnostic summary when present. For an existing trajectory, the same evaluator
can be run directly:

```bash
python3 scripts/evaluate_degeneracy_trajectory.py <candidate.tum> \
  --reference-trajectory <dense-reference.tum>
```

## RTK-SLAM exact acquisition

Plan the smallest official ROS2 sequence and pinned surveyed-checkpoint assets
before committing disk space or network time:

```bash
python3 scripts/download_rtk_slam_dataset.py \
  --sequence construction_seq2 \
  --eval-assets \
  --dest /mnt/large/rtk_slam \
  --dry-run
```

This standalone acquisition helper is included in the curated release bundle,
so the command also works from its extracted `release_bundle/` directory. The
measured accuracy suite below must run from the exact source checkout in a
compatible built ROS workspace; the curated bundle is an audit and acquisition
packet, not a replacement for that runtime workspace.

`--dry-run` performs no download, Git fetch, directory creation, or other
write. It reports the immutable dataset revision, exact size and SHA-256 of
each DB3 and metadata file, already-present resumable bytes, remaining payload,
filesystem reserve, observed free bytes, exact shortfall, and a copy-ready
external-destination recovery. When Linux exposes a sufficiently large
attached-but-unmounted hotplug filesystem, the plan lists its device,
filesystem, partition size, and optional model/label, then makes mounting it
the single next action. It never mounts or probes the filesystem itself, and
marks free space unknown until the user mounts it. The follow-up command reruns
`--dry-run --dest-device /dev/...`; the helper resolves the actual mount path
and appends `rtk_slam`, so no mount-path placeholder needs editing. It shows
the matching live command only after that exact filesystem reports `READY`.
Add `--json` for the same structured plan. Use `--list` to inspect all four
exact sequence identities without network access.

When the mounted-path plan reports `READY`, remove only `--dry-run`. The live
command checks
capacity before its first write or network request, resumes a smaller regular
file, and verifies exact size plus SHA-256 before accepting it. A same-size
wrong file, oversized file, non-regular path, or symlink fails closed with a
recovery action. Evaluation assets are fetched at commit
`f2921a58caf5a87c1f4f73b48c6f2a5e35f92924`, never a moving default branch.

After acquisition, validate and preview the measured suite without starting
ROS:

```bash
python3 scripts/run_rtk_slam_accuracy_suite.py \
  --dataset-root /mnt/large/rtk_slam \
  --sequence construction_seq2 \
  --dry-run
```

Repeat with `construction_seq1`, or pass `--sequence all` for the complete
four-sequence suite. Acquisition readiness is not benchmark evidence: the two
blocking release rows require fresh exact-candidate outputs from Construction
Seq2 and Construction Seq1.

## FAST-LIVO2 head-to-head

Use the exact same bag, sensor messages, calibration, trajectory reference, and
evaluation alignment for both systems. Record each result in this compact JSON
shape (unknown metrics should be omitted, never estimated):

```json
{
  "system": "lidarslam_ros2",
  "dataset": "hilti2022_exp04",
  "trajectory": {"ape_rmse_m": 0.07146},
  "geometry": {
    "plane_thickness_mean_m": 0.0599,
    "planar_coverage": 0.5355
  },
  "colour": {
    "heldout_rgb_l2_median": 36.37,
    "heldout_rgb_inlier_20": 0.3536
  }
}
```

Add RPE and runtime keys only after measuring them. Create a second manifest
with `"system": "FAST-LIVO2"` and run:

```bash
python3 scripts/compare_fast_livo2.py \
  --ours output/head_to_head/lidarslam_ros2.json \
  --fast-livo2 output/head_to_head/fast_livo2.json \
  --out output/head_to_head/comparison.json
```

The command also writes `comparison.md`. It scores APE, RPE, real-time factor,
peak memory, plane thickness, planar coverage, held-out RGB error, and held-out
RGB inlier rate. A metric only counts when both systems provide it; values
within 1% are ties. This prevents an attractive map image or a single trajectory
number from being presented as an overall win.

## Runtime phase contract (M6a10-v1 compatibility)

Competitive replay uses the preregistered `runtime.online_compute_rtf` as its
primary realtime metric. It covers input consumption through required drain,
divided by sensor duration; container startup, map save/postprocess, and fixed
shutdown grace are excluded. `runtime.wall_realtime_factor` is retained as a
diagnostic. Every wrapper must emit an atomic `phase_evidence.json` with
monotonic boundaries, CPU/IO counters, and a fail-closed trajectory timestamp
or exact message-count coverage proof. See
[`M6a10 phase contract`](architecture/benchmark-phase-contract-m6a10.md).

The additive v2 contract is not satisfied by publisher or process-exit counts.
The current ours-only implementation records its application-owned
`consumer_evidence.json` from `rko_lio::ros::OfflineNode::run`; a replay is
still required before it can be considered measured. GLIM and FAST-LIVO2 have
not been changed in this slice, so a three-system v2 gate remains incomplete.

The previous campaign4 wall-time result remains an immutable failure lineage
(FAST exceeded the old `<=1.0` wall gate); it is not retroactively converted
to the new metric. A GT-blind NTU Viral `tnp_01` public/training replay is
recorded at
`/media/sasaki/aiueo/benchmarks/m6a10_training_20260822/ntu_tnp01_training_validation_summary.json`
(SHA-256
`4217a4b07f5ff85148e7433be1b9fef51e35d843a98bbb287d6f9c010c254177`).
It remains `INCOMPLETE`/`FAIL_CLOSED`: ours lacks authoritative drop/queue
counters, GLIM lacks an explicit EOF marker, and FAST's successful attempts
are just over the preregistered online RTF limit. No ground-truth content or
scorer was accessed, so this is not an accuracy or SOTA result. A future
validation must supply authoritative consumer counters/EOF proof before the
online metric can authorize a competitive gate.

This section describes the immutable v1 compatibility receipt. New
preregistered runs use the additive v2 followability/acknowledgement contract
below; v2 does not rewrite the v1 receipt or retroactively reinterpret its
wall-time result.

### M6a10-v2 preregistration

The additive v2 contract separates 1x paced followability from unpaced
acknowledgement throughput. Paced runs require exact consumer
expected/received/processed counts, observed EOF, zero drops/overflow, empty
drain backlog, 250 ms timestamp/latency bounds, and independently verified
1.0x pacing; wall/online RTF is diagnostic. Unpaced runs additionally require
an implementation-owned synchronous ack/backpressure hook and use the
acknowledgement interval RTF (`<=1.0`) as the throughput gate. A publisher
count or a higher rosbag playback rate is never an acknowledgement proof.
The preregistration is in `competitive_slam_v1.yaml` under
`runtime_policy.phase_contract_v2`; the next training replay must populate
the application-owned `M6A10_CONSUMER_EVIDENCE` file for each wrapper or
remain fail-closed.

### M6a10-v2b GLIM consumer hook (fixed10-v2 replay fail-closed)

GLIM's opt-in v2b image build and read-only installed-image verification are
`PASS`. One functional replay was executed once, but its overall closure is
`INVALID_SAFETY`; it is retained and not promoted. The pinned source revisions are GLIM
`faa264a1bce1bda406f73457e35511f56cdc2eaa` and `glim_ros2`
`4a9e7a4cb084967c8525a1be529ad3ba2a118ae7`; the benchmark-only patches are
`docker/patches/glim.m6a10-v2b.patch` and
`docker/patches/glim_ros2.m6a10-v2b.patch`. The recipe verifies both patch
SHA-256 values and builds with `BUILD_WITH_CV_BRIDGE=ON`. The NTU `tnp_01`
contract is fixed at 5793 LiDAR, 225102 IMU, 5792 image, and 236687 total
messages. The dedicated host entry point is
`scripts/run_glim_benchmark.py --phase-contract v2`; it rejects any bag path
or canonical tree hash other than the preregistered input and supplies every
GLIM expected-count, callback-latency, and backlog-bound environment value.
The preregistered host runner SHA-256 is
`6924b111b88f5354a17f1d7ede693e8547e1ea5196fcdfd5503fa7dd65aba6dd`; its v2
binding also requires the NTU config tree SHA
`842be775f7ee4b555f60957cf9f4cc8c35eb790e3ffc3bca767170c6415bb942` and
matching image labels for both patch SHAs and `BUILD_WITH_CV_BRIDGE=ON`.
The verified image is
`m6a10-v2b-20260823-glim-cpu-benchmark:competitive-v1` with immutable ID
`sha256:010c0019a077116edf4d1e7462dfa28561c4fb17db3b5db52e3652c8a875eb41`.
The read-only build/identity receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-glim-closure-20260823/build_receipt.json`
with SHA-256
`fe4d7e3b3b3fec28185f5faa016b19dfb30bd9a52928b099a68d6679e0a09df2`.
The system-level execution identity was then rechecked without opening a bag
or starting a benchmark. Its PASS receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-preflight-20260823-v2/execution_identity_preflight.json`
with SHA-256
`0ae210d6df457fb6bebf7c23ee33cb4e9299bcfe5f0ddde2ad009f79b1dd5179`.
It binds the current runner/wrapper, recipe, image ID, all OCI labels, the
system-container toolchain fingerprint, and the read-only/network-none
probe. The first generator receipt is retained as superseded because its
config tree hash used the wrong algorithm; the v2 receipt uses the pinned
`relative_path_size_content_sha256_v1` definition. This remains identity
preflight only: replay, GT access, scorer invocation, and performance claims
are absent from the preflight itself.
The fixed10-v2 evidence root is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/glim_m6a10_v2b_unpaced_ack_fixed10_v2`.
The immutable closure receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-closure-20260823/closure_receipt.json`
(SHA-256
`72d8ef9cd2c8e26d4c2120463fdbff3057a75e3f81203863049c11028baa41d5`),
and its output tree SHA-256 is
`f0db95c9c700b8c9650e9ce2751eb716c2cc271c664ee971f337ec6c01da2552`.
The wrapper evidence itself passed with exact expected/received/processed
`236687`, zero drops/overflow/failures, EOF, empty drain, maximum callback
latency `0.0149106` seconds, process-tree RSS `1334648832` bytes,
`memory.max=max`, and OOM delta zero; online acknowledgement RTF
`0.29150756632664043` remains diagnostic. However, the direct first launch
mounted the release parent directory, making the sibling name `ntuviral_gt`
reachable even though no GT file was opened. Strict GT-unreachable closure is
therefore false, retry is zero, and a future attempt must mount only the
canonical input directory. The synchronized profile canonical SHA is
`800f07184b728623710375c778624a4f623bc706a1f8c05b19b544847d6e3830`, and the
execution-selection file SHA bound by that profile is
`9038c02be377a9ac9cc6fc15a34e9f23fc0181b57f31a5053d8f7c1a4aa00db2`; both are
recorded in the machine-readable receipts;
this attempt does not authorize scoring, accuracy, performance comparison, or
M6b.

The application writes an atomic EOF sidecar immediately after reader
`has_next()` ends and before `GlimROS::wait()`, then writes final generic
consumer-state evidence before save. The wrapper treats those files as
authoritative, verifies sidecar immutability, passes the preregistered
high-water bound to the common validator, and never infers EOF from logs.
`ack_source_kind` is `consumer_callback`; `single_message_buffer_verified`
is explicitly false because the asynchronous queue is observable but not a
single-message buffer. Queue drops, overflow, unsupported topics, callback
latency, nonempty final queue, malformed/missing sidecars, and any image build
flag other than `ON` fail closed. The result and runtime evidence paths are now
immutable fail-closed evidence; GT and scorer remain untouched. The host
runner's legacy tree-hash path was not invoked because it conflicts with the
preregistered materialization hash kind; the direct wrapper invocation is
recorded explicitly rather than relabeled as a host-runner execution.

### M6a10-v2b GLIM fixed10-v3 narrow-mount replay (2026-08-23)

Fixed10-v3 is retained as immutable `INVALID_SAFETY` with retry `0`. The
runner now uses the materializer's
`relative_path_size_content_sha256_v1` tree hash and `--pull=never`; its
SHA-256 is `c1f347f5af3751d19ba89ca087e37e32cd9b135108e742f7f3e9e044a205cb24`.
The v3 identity preflight is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-preflight-20260823-v3/execution_identity_preflight.json`
(SHA-256
`9b8cc1d089a73113930557c6558a32a5ded3725d386b20a78c61fc14e740e190`). It
binds only the canonical ROS2 input directory to `/data:ro`; the release
parent and its `ntuviral_gt` sibling are absent from the Docker mount graph.
The image was not rebuilt or pulled.

The single v3 run is under
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/glim_m6a10_v2b_unpaced_ack_fixed10_v3`.
Its consumer evidence passed exact expected/received/processed `236687`,
zero drops/overflow/failures, EOF, final drain backlog `0`, and maximum
callback latency `0.025782` seconds. Process-tree RSS evidence passed at
`1313624064` bytes; cgroup memory peak was `1393958912` bytes with
`memory.max=max` and OOM delta zero. The output tree SHA-256 is
`3ab59eed2f20c3bf179bbfa05d0b5a4ffed723137ea81ebc2427619fd26306c5`, with
no `.part` or unauthorized map artifacts. However, the generic phase
finalizer revalidated the observed high-water backlog `249` using its default
bound `0` instead of the preregistered `100000`, so `phase_evidence.json` is
invalid (`consumer_backlog_bound_exceeded`) despite the authoritative consumer
evidence passing. The immutable closure receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-closure-20260823-v3/closure_receipt.json`
(SHA-256
`492d39b1f7c4a1a2e86893dd979b0ed57b3f37267cb7f21849b990d95bcc32ff`). This
attempt authorizes no scoring, accuracy, performance comparison, SOTA claim,
or M6b progression. The synchronized profile canonical SHA is
`d83a3b96ea224f3f4673dd48a6252af488834dc272f735c5d5e147ce6a7a0ec4`, and
the bound execution-selection file SHA is
`6ddcc24b5d221ef96ac746a7f0b4c6c3e261f5c002d115d51d19eb5fa8e072f4`.

### M6a10-v2b GLIM fixed10-v4 closure (2026-08-23)

Fixed10-v4 is the single authorized GLIM retry after the immutable v3
`INVALID_SAFETY` lineage. The v3 closure remains unchanged at SHA
`492d39b1f7c4a1a2e86893dd979b0ed57b3f37267cb7f21849b990d95bcc32ff`; its
failure was the generic finalizer using backlog bound `0` while the consumer
contract allowed `100000`. The common finalizer now receives that bound
explicitly and rejects consumer/phase bound mismatches.

The v4 identity preflight is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-preflight-20260823-v4/execution_identity_preflight.json`
(SHA-256
`fda87bce9af6c60a60fcc3523e3f64f4b0fba288f2eea53988432085ab95af8a`), and
the single quiescence receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/glim_m6a10_v2b_unpaced_ack_fixed10_v4/quiescence.json`
(SHA-256
`de015c5b7530c66ba222d1c9fcfdc20d486fbde8ee8cfec36a01c0a677814894`). Both
passed before the runner started. The runner mounted only the canonical input
directory as `/data:ro` and used `--network none --read-only --pull=never`.

The v4 run root is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/glim_m6a10_v2b_unpaced_ack_fixed10_v4`;
its output tree SHA-256 is
`85de6ad890b3009cedac7720b5e795f764eafa2cde3cfb13bae3ebee7c04bc1c`. The
independent closure receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a10-v2b-execution-closure-20260823-v4/closure_receipt.json`
(SHA-256
`f33953c4126939dfce841b030cdb2e5560615d42963ae4aaaf4ab0aad7a7f6c1`) and
records `PASS` with retry `0`. Consumer and phase evidence agree on exact
`236687` expected/received/processed messages (LiDAR `5793`, IMU `225102`,
image `5792`), zero drops/overflow/failures, observed EOF, empty final drain,
maximum backlog `275 <= 100000`, and callback maximum `0.014094 <= 0.25 s`.
The unpaced acknowledgement gate passed; online acknowledgement RTF was
`0.28196761482694743` and remains a functional diagnostic, not a cross-system
performance claim. Aggregate process-tree RSS was `1323241472` bytes, cgroup
total peak was `1443680256` bytes, `memory.max=max`, and all OOM deltas were
zero. Runtime outputs are hash-recorded, no `.part` or forbidden map paths
were present, and GT content/scoring were not accessed. This closure does not
authorize accuracy, SOTA, or M6b claims. The synchronized profile canonical
SHA is `f58858e033424c72f2d010fb03bd30846f648ae83a07c3af850af588358f0a82`
and the bound execution-selection file SHA is
`bd4a25a7b47e601ea6260397b84287c7be85d474d729e97ed19b99071bdaef9f`.

### M6a10 fixed10-v2 failure lineage and quiescence preregistration lineage

The one fixed10-v2 ours replay is immutable `FAIL_CLOSED`; it is not a
performance comparison and was not retried. Its root is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v2`
(root tree SHA-256
`868dc7369494eb7f6f7e09ab9363c408024c8bdeb31d7055113f1d9c6d5d7418`, output
tree SHA-256
`28772ce7cec189fbaa3a151f966cc6e8714d57cdaea86c26652b01ddbf987af3`). The
consumer hook itself reported exact `230895` received/processed messages,
zero drops/overflow/failures, EOF, empty backlog, and status `PASS`; that
sub-result is retained but cannot promote the attempt. The phase receipt was
invalid because `input_end` was missing and the process exit was `125`. The
maximum callback latency was `0.371609411` s against the preregistered `0.25`
s bound, and the process-RSS sampler was invalid with `139.5325092%` jitter
against its `100%` bound. Ground-truth content and scoring remained false;
retry count is zero. These independent failures are recorded in the profile
and selection receipt rather than relaxed after the run.

The replacement contract is `m6a10-v2a-ours-rko-unpaced-ack-fixed10-v3` and is
`preregistered_not_executed`, not a result. It keeps the same pinned image,
input tree, message counts, no-map-artifact requirement, and `online RTF <=
1.0` contract. Before a runner can start, the new read-only
`scripts/check_m6a10_quiescence.py` receipt must be atomically produced at
the v3 output root, have status `PASS`, and match its recorded SHA-256. The
preflight samples `/proc` for five seconds, limits CPU busy ratio to 5% and
load1-per-CPU to 0.5, and rejects active compiler/build/docker-build/colcon/
cmake/cargo processes while excluding its own ancestry. No preflight or v3
bag replay has been run; `performance_comparison`, ground-truth access, and
scoring remain false.

The single v3 quiescence preflight is retained as `FAIL_CLOSED`; it did not
start a runner and was not retried. Its receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v3/quiescence.json`
(SHA-256
`74732fbdeb9bb8da094bacde3518cc2475eaeb29a3f4fec5c39d7972dc97a5a6`). The
five-second observation recorded CPU busy `98.72340425531915%` against `5%`,
load1-per-CPU `0.805` against `0.5`, and eight active compiler processes;
`runner_start_allowed` was false. GT/scorer remained false and retry count was
zero. Fixed10-v4 is now preregistered with the same image, input, consumer,
no-map, and quiescence thresholds, with a null receipt result and the v3
preflight failure as its explicit predecessor. Its one preflight is retained
as an immutable `FAIL_CLOSED`: CPU busy was `98.12312312312312%` against
`5%`, load1-per-CPU was `0.86` against `0.5`, and seven compiler processes
were present. The v4 receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v4/quiescence.json`
(SHA-256
`b8c7b38debd07586f3af060e51b83a5fa94108ba2c80bf27f481ac16f291fb51`). It
did not start a runner and was not retried.

Fixed10-v5 records the next one-time preflight under the same unchanged
functional contract. Its receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v5/quiescence.json`
(SHA-256
`dbf54a1a5a1fd9d527fef280f633e3710b9a79d04ac482bb13ac1278516b4dee`). This
five-second observation is also immutable `FAIL_CLOSED`: CPU busy was
`25.717884130982366%` against `5%`; load1-per-CPU was `0.2575` (within its
`0.5` bound), but one active compiler process remained. `runner_start_allowed`
was false, retry count was zero, and ground-truth/scorer access remained
false. Fixed10-v6 preserved the same preregistered image, input, consumer,
no-map, and quiescence contract. Its one quiescence receipt did pass (CPU
busy `3.0264817150063053%`, load1-per-CPU `0.03875`, no forbidden processes),
but the preflight-to-run continuity window was lost before a runner was
started. It is therefore closed `FAIL_CLOSED` as
`runner_not_started_after_preflight`; the runner was not retried, and no bag
replay, ground-truth access, or scoring occurred. The PASS preflight receipt
is `/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v6/quiescence.json`
(SHA-256
`d9cf50b07157a7da8655d672d7955a768676361735967e35e64367b6272cf3fa`). The
immutable closure receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v6/runner_not_started_after_preflight.json`
(SHA-256
`ef04937fd7c17b2ed017902985502aa02b9349ccb63961dcd206c0370aea79ad`).

Fixed10-v7 attempted the single-process launcher intended to close the v6
preflight-to-run continuity gap. It is
`m6a10-v2a-ours-rko-unpaced-ack-fixed10-v7`, now closed
`FAIL_CLOSED` before quiescence/runner start because the launcher-observed
input tree SHA
`bcbb4c86f568125104565fca3882695fab0b501a7ca9d9aa9aaa643f1b8ee6eb`
did not match the preregistered
`0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb926`.
The independent profile-defined `relative_path_size_content_sha256_v1`
calculation still matches `0a45497a…e38cb926`; the failure is therefore a
launcher hash-contract mismatch (its observed diagnostic omitted the file
size field), not evidence that the managed input bytes changed.
The immutable closure is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v7/closure_receipt.json`
(SHA-256
`fffba96d21721c5b811fb7710d3d77889e81e0c8bfb4ffcb8957f6c94bce57da`),
with retry count zero; the root contains no quiescence receipt, launch
receipt, Docker output, time report, or `.part` file. The immutable v6
`runner_not_started_after_preflight` receipt remains its predecessor. The
launcher
is `scripts/run_m6a10_fixed10_v7.py` (SHA-256
`47ab6b059bc9736da5fa69933d7a1d1024db186b78b1c5d30b987544fb0d20d4`), with
contract tests in
`graph_based_slam/test/test_m6a10_fixed10_v7_launcher.py` (SHA-256
`ddfd8d23917b049b870aa8e98598f4c63cfc58c3f5ccc59bd8fb73aa5b825698`). It
reserves and validates a new empty
attempt root, binds the input/source/image identities, invokes the
quiescence checker exactly once, strictly validates its PASS receipt, and
starts the digest-pinned Docker argv from the same process without shell
interpolation or taskset/cpuset. A two-second monotonic preflight-to-run gap
is checked immediately before `Popen` and after child start. The fixed argv
records `/usr/bin/time -v -o time-v.txt`, and immutable marker,
start-attempt, started, and closure receipts distinguish preflight failure,
runner-start failure, signal/exception, nonzero exit, and completion-contract
failure. A zero exit is not completion: host GNU-time fields, phase and
consumer evidence, process RSS/memory evidence, trajectory files, no-map
artifacts, and GT/scorer-blind proof must all validate before a `COMPLETED`
closure is written. Because this attempt failed before those gates, v7 has
not launched Docker or replayed the input; it authorizes no performance,
accuracy, or SOTA claim. The observed hash-kind mismatch is retained for a
future contract correction rather than repaired by retrying this attempt.

Fixed10-v8 was attempted once as the next versioned contract; it does not reuse
the v7 attempt root or alter v7's failed record. Its launcher
`scripts/run_m6a10_fixed10_v8.py` (SHA-256
`65ad02d63be843db100198610fba3602f18b3f034d637635cce6b6a3e719e6d8`) binds
the pinned v7 implementation SHA
`47ab6b059bc9736da5fa69933d7a1d1024db186b78b1c5d30b987544fb0d20d4` and
reuses its identity, immediate-start, two-second gap, host-time, completion,
no-map, and GT-blind gates without changing the v7 source. Its tree identity
uses the materializer's exact
`relative_path_size_content_sha256_v1` helper (path, byte size, NUL, content)
and the corrected full input SHA
`0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263`.
The v8 contract tests are
`graph_based_slam/test/test_m6a10_fixed10_v8_launcher.py` (SHA-256
`35685930524bed8390fcad42fe48b4d342d046331e361380c7f105559a2105e3`). The
single v8 attempt failed closed during identity preflight before quiescence or
Docker: the adapter imported the malformed v7 image digest
`sha256:385b6eeda3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69`,
while read-only Docker inspection returned the pinned full image ID
`sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69`.
This is an adapter/global-binding identity defect, not a tag race. The v8
marker SHA is
`e062814e61d373e5bef77608b0c5a578cb7e7375d25b88b44d161a155493e889` and its
closure SHA is
`c302a50665e24ad1a7f2bd18b1e77a0cb2d2f0382ad18971d3040ce59ebd3d9e`.
`runner_start_attempted=false`, quiescence was not started, and no Docker,
replay, GT access, or scoring occurred. The v8 record is now
`FAIL_CLOSED` with retry count zero; no follow-up preregistration is made.

Fixed10-v9 is the independent successor preregistration. It leaves v7/v8
launchers and attempt roots immutable, keeps the same canonical input identity,
and binds the complete local image ID
`sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69`
in the public `LaunchConfig`, adapter globals, and fixed Docker argv. Its
launcher is `scripts/run_m6a10_fixed10_v9.py` (SHA-256
`879fa1c44aa093b158e405806276935bc41bf3bf4d9134a4534cd4922312254f`) and its
tests are `graph_based_slam/test/test_m6a10_fixed10_v9_launcher.py` (SHA-256
`b26a0580db1eac88e6be4ba9620731533cd98b25ddeea7033bedf5bd1a0ea1b8`). Before
preregistration, the exact v9 `validate_preflight_identity` path passed a
read-only real-input/source/local-image check. The external identity receipt
is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v9_identity_preflight_v2/identity_receipt.json`
(SHA-256
`042af3a3f791400b7c642e0d5aec06357527cc09e4590999bcc75c7d96a7973c`);
it records the canonical input tree, all pinned source hashes, Docker labels,
and `runner_start_attempted=false`, `quiescence_started=false`, and
`docker_run_started=false`. The one v9 launcher attempt then reached its
owned quiescence check and failed closed because load1/CPU was `0.72375 >
0.5` (CPU busy was `2.4439405391786346%`, with no forbidden process). Its
marker, quiescence, and closure receipts are recorded in the profile; no
runner or Docker process started, and retry remains zero. The earlier receipt at the v1
identity-preflight path remains immutable as `superseded_not_promoted` because
it was bound to the pre-final launcher SHA `f6e10e...`; only the v2 receipt is
the active v9 identity binding. No replay, GT access, or scoring has occurred.

Fixed10-v10 is an independent successor to the v9 `PREFLIGHT_FAIL_CLOSED`
record. It does not alter any v1-v9 source, root, or evidence. Its single
authorized GT-blind functional run completed at
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v10`;
there was no retry, GT mount/access, scorer invocation, or performance
comparison. The launcher is
`scripts/run_m6a10_fixed10_v10.py` (SHA-256
`f1d7844eaf8f2d5c22431fed5abcc84ae07f9f34cbd92de3b515c61d20ce78ed`) and its
updated contract test is
`graph_based_slam/test/test_m6a10_fixed10_v10_launcher.py` (SHA-256
`b89afc720fec4cc0af2d0736367407cbb6dc4b8789a26e3a0a3adf56530b2283`).
The launcher bound the complete local image ID
`sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69`,
the canonical input tree
`0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb926`, and the
immutable v9 source SHA
`879fa1c44aa093b158e405806276935bc41bf3bf4d9134a4534cd4922312254f`.
The read-only identity receipt remains
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v10_identity_preflight/identity_receipt.json`
(SHA-256
`2211744e3691df3fc71f7b0a81248a2e75daa4c5b15a23c8a0f8acab4b360ab2`).
The run returned zero and passed the unpaced consumer/phase contract:
`230895` expected/received/processed, zero drops/overflow/failures, EOF and
empty drain, maximum callback latency `0.109221778` s, and online compute RTF
`0.126860008821509` (the unpaced throughput gate is the primary gate; wall RTF
remains diagnostic). Aggregate process-tree RSS was `868278272` bytes, cgroup
total peak was `2750537728` bytes with `memory.max=max` and OOM delta zero, and
raw/corrected trajectory hashes were both
`9e20cb96a4326eb41e26d20171a664133b4038e473c3ca0f89f1748892c323f7`.
The closure receipt SHA is
`e7224eb29dc547a5119cb7ecc1d51d009ced43cfca8e23c6c6d2fc1977e2e82b`; host
`time -v` SHA is
`2a35f07125c14cb9f11a514c0ff57ad5b97ab6c4061b19e45f17f5e50e3470a9`.
The regular-file output tree diagnostic is
`ef2d9b9c0d59b9884ea45ca5443f47c71fc93c3653257d458c375f284855cf7b` under
`output_regular_file_content_sha256_v1`; canonical materializer hashing was
not claimed because the output contains the runtime symlink `out/ros_log/latest`.
This is functional validation only: no accuracy, map-quality, performance
superiority, SOTA claim, or M6b authorization follows.

### M6a10-v2a synchronized-tail input preflight

RKO-LIO's offline processing requires a strict synchronized tail. Before any
conversion or replay, run the read-only analyzer against the canonical NTU
training input:

```bash
python3 scripts/analyze_m6a10_synchronized_tail.py \
  --bag /media/sasaki/aiueo/datasets/ntu_viral_release/tnp_01_canonical_header_order_ros2 \
  --output /media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/synchronized_tail_dryrun.iD9dIn/receipt.json \
  --lidar-topic /os1_cloud_node1/points \
  --imu-topic /imu/imu \
  --dry-run
```

The fixed PointCloud2 contract is one little-endian `UINT32` (`datatype: 6`)
field `t`, interpreted as nanoseconds relative to `header.stamp`. A LiDAR
message is eligible only when its point-level maximum timestamp is strictly
less than the final IMU header timestamp; equality is ineligible. The
decision is independent of arrival order and system name, and any
nonterminal ineligible pattern or unsupported timestamp schema fails closed.
The analyzer writes only an atomic receipt outside the bag, does not create a
trimmed bag, and never opens GT or invokes a scorer.

The pinned dry-run receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/synchronized_tail_dryrun.iD9dIn/receipt.json`
(SHA-256
`31072befe0ee816cfe09ee600f0ed604fd361a26e863e9bc51e5edab6f5f66d3`). It
reported 225102 IMU messages, 5794 LiDAR messages, 5793 eligible scans and
one terminal scan requiring exclusion, with a proposed end duration of
579.277931825 s. This is an input preflight receipt, not a replay or accuracy
result.

### M6a10-v2a synchronized-tail materialization

The deterministic materializer is verified against the NTU `tnp_01` canonical
ROS2 input. The competitive profile status is
`ros2_materialized_verified_ros1_verified`. The fixed10 external receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/synchronized_tail_materialization_v1_fixed10/receipt.json`
(SHA-256
`62defcf7a1b5cdadee666de44b09f71f8e011551ea93e37eec51b64a333cae9f`), and the
published ROS2 tree SHA-256 is
`0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263`.
The receipt records 236688 input records, 236687 output records, 5793
eligible LiDAR scans, 225102 IMU records, 5792 image records, and one dropped
terminal LiDAR payload (SHA-256
`a11e441e679c424d31a43755d96328f94e73a1d2a52e48a1c808ac51f7443830`). Its
independent raw-stream verification passed; this is an input-integrity
materialization result, not an accuracy, replay, or performance result.

The generator SHA-256 is
`caddcf0ae85d74444ae65ea85ed33d5e561a2569dc8ef180b2496a9d87c132c9` and the
contract-test SHA-256 is
`f0c58ddbdfeaae2399125bfffc0ebef5ef5c57aba4dca72e7e8238ec0c3175d4`. The
analyzer receipt remains the sole source of terminal LiDAR selection. The
output is written to a new sibling `.staging` container using the final bag
basename, independently verified before atomic rename, and checked after
publication by output tree hash only. Legacy `.part` staging, internal
`.part.db3` names, overlaps/symlinks, stale outputs, duplicate connections,
and tampering fail closed. The receipt is external to the bag tree to avoid a
hash cycle. The previous fixed10 predecessor remains immutable as
`superseded_not_promoted` because of its contract-ID mismatch and internal
`.part.db3` filename.

The one-time ROS1 equivalence conversion completed with status `PASS`. Its
external receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ros1_equivalence_v1/receipt.json`
(SHA-256
`b71e7f5aa8f82a5711e6b25e42532caa8248a3fcf062d9b3b15a5c86460438ec`), the
semantic report is `semantic.json` (SHA-256
`0a64efb618c9bdbba4e762c350f21e36fb1244b4e2c2ffa17b3d7604cdd318ec`), and
the `/usr/bin/time -v` report is `time-v.txt` (SHA-256
`080dfbc2d50e016359702f58554f428b9aea1f18e974bc8275c2f2104b07459a`). The
published ROS1 bag SHA-256 is
`5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310` with
11290464091 bytes and 236687 messages. Independent inspection found exactly
the three expected topic types and counts: PointCloud2 5793, Imu 225102, and
Image 5792. The semantic report has `all_topics_equal: true`; the sibling
`.staging` container and legacy `.part` artifacts are absent.
The timed wrapper wall time was 207.57 s and maximum resident set size was
153252 KiB; these are conversion/equivalence diagnostics, not SLAM runtime or
accuracy measurements.

The fixed converter command was:

```text
rosbags-convert --src <canonical_ros2> \
  --dst <ros1_staging_container>/<ros1_final_basename>.bag \
  --compress none --src-typestore ros2_humble --dst-typestore ros1_noetic
```

The observed converter is `rosbags-convert` 0.11.0 (executable SHA-256
`83f210e4fd135eb81c12191b20fe06443eab0344398a5e0712283a538749bfc2`, help
SHA-256 `5fc5a415d32b0ccc953cc2b2f4f5213c1c201a9338b4b53eb7bdc1cebc23e4aa`).
The semantic comparator (SHA-256
`7464d0e64ba1cafbdbb7a2e162bd2735b7288d9ea6e3a2dc1e7757bdab5fcf41`) must
report `all_topics_equal` for exactly `/os1_cloud_node1/points`, `/imu/imu`,
and `/left/image_raw`. Ground-truth content and scoring remain false by
contract; this is a transport/semantic-equivalence result, not an accuracy,
performance, or SOTA claim.

## Competitive victory evidence (schema v2)

`scripts/run_cross_repo_slam_benchmark.py` and the legacy `--gate` mode of
`scripts/evaluate_competitive_suite_gate.py` remain report-only compatibility
paths. They do not authorize a system-level SOTA claim. The explicit v2 gate
is fail-closed and consumes one machine-readable evidence document:

```bash
python3 scripts/evaluate_competitive_suite_gate.py \
  --evidence <competitive_evidence_v2.yaml> \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --output <out>/competitive_evidence_v2.json \
  --yaml-output <out>/competitive_evidence_v2.yaml
```

The document must declare `schema_version: 2`,
`evidence_kind: competitive_slam_victory_evidence`, a profile-matched
`fresh_holdout_slots` selection receipt, and all three required systems
(`ours`, `glim`, and `fast_livo2`). Fresh slots are separate from the exposed
historical `holdout_slots` (`exp02`, `exp03`, and `exp21`); those historical
sequences can never be relabelled as fresh. Every profile fresh slot must be
`frozen_unopened`/`frozen` with a selection-receipt, input-manifest,
ground-truth, and calibration SHA-256, and the evidence must match all of
them. The profile records the reviewed, deep-verified Exp14, Exp16, and Exp18
identities in
`configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml` with
status `frozen_unopened`. The receipt fixes the official source revision, bag
sizes/SHA-256 values, opaque GT identities, calibration tree, canonical ROS2
and semantic/input-manifest identities, exposure audit, and blind-order
policy. The execution-identity receipt/preflight is ready/PASS before first
run; a real v2 evidence receipt remains `INCOMPLETE` until all required runs
exist, and no self-declared evidence can promote it.

The evidence contract has explicit `partitions.historical` (role
`regression`) and `partitions.fresh` (role `primary_fresh`) blocks. Every
system must provide exactly three complete runs for every dataset in both
partitions. Historical rows are not used for the aggregate victory APE/CI, but
they remain mandatory regression coverage.

Each system contains pinned provenance (`revision`, container digest, and
64-hex toolchain/config fingerprints) plus common scorer fingerprint,
input/reference/calibration, hardware, machine, thread-policy, and exact
`Release` identity. The thread policy must include `cpu_affinity`,
`max_threads`, `omp_num_threads`, `openblas_num_threads`, `mkl_num_threads`,
`tbb_num_threads`, and `accelerator_policy`; all systems must match the
canonical mapping hash. Each run repeats the dataset identity hashes, so a
global input hash cannot hide a cross-dataset mismatch. Historical identity
uses the profile's `input_manifest_sha256`, `ground_truth_sha256`, and
`calibration_archive_sha256`; fresh identity additionally includes the
selection receipt hash. Failed runs remain records and force `FAIL`; an
omitted run is `INCOMPLETE`. A complete run must include finite positive APE,
processing RTF at most 1.0, peak RSS, all map metrics, completion/exit status,
zero catastrophic failures/verified false loops, and trajectory/map SHA-256
artifacts.

The accuracy gate selects the lowest aggregate APE rival and requires ours to
improve by at least 10%. Its 95% superiority interval is a true two-stage
hierarchical bootstrap: fresh datasets are resampled as clusters with fixed
seed `20260821`, then ours and each rival's three runs inside every selected
dataset are independently resampled before their means are compared (10,000
draws). Runs are explicitly not treated as pseudo-independent datasets. All
rivals must have a positive lower CI bound. Each dataset also has a 2%
primary-APE regression limit against that sequence's best rival, and map
non-regression is checked separately for every dataset/rival pair before any
suite aggregation. The command writes matching JSON and YAML receipts with
`PASS`, `FAIL`, `INCOMPLETE`, or `INVALID`, including profile/evidence SHA-256
identities; missing real competitor evidence therefore cannot become a
victory by aggregation.

The scorer fingerprint is not a hand-entered label: the preflight checker
sorts scorer entries by name and hashes canonical JSON containing each entry's
name, repository-relative path, measured file SHA-256, and declared policy.
The receipt fingerprint must equal that recomputed digest. The checker also
requires every system's revision/container/toolchain status to be `ready`,
`frozen`, or (for source revisions only) `pinned`, rejects dirty worktrees,
and requires clean tracked-diff/untracked-content provenance before a run.
Pending status values remain `INCOMPLETE` even if a placeholder digest is
present.

Before any competitor run, freeze the execution identity with the separate
preflight receipt:

```bash
python3 scripts/check_competitive_execution_selection.py \
  --receipt configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --output <out>/competitive_execution_preflight.json \
  --yaml-output <out>/competitive_execution_preflight.yaml
```

The profile records the receipt path and full-file SHA-256. The checked-in
receipt is now `ready`: the ours clean revision, machine fingerprint,
eight-thread policy, all three pinned container/toolchain identities, and the
deep-verified fresh input identities are recorded. Missing values remain
`INCOMPLETE`; malformed or changed paths/digests are `INVALID`. This check is
read-only and performs no container build, dataset download, ground-truth
inspection, or benchmark run. The identity records exact `Release`,
revision/config/container/toolchain/scorer/machine/thread fields, plus the
modality/calibration policy: GLIM CPU is lidar+IMU, FAST-LIVO2 is
lidar+IMU+five-camera visual, and ours is the lidar+IMU track. These are
fairness constraints, not performance evidence. Before any run, enforce the
recorded policy with `taskset` and the matching Docker `--cpuset-cpus` setting,
and explicitly export `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`,
`MKL_NUM_THREADS`, and `TBB_NUM_THREADS` as recorded; this receipt update does
not change the benchmark runners.

The profile/receipt registration uses the non-cyclic
`canonical_profile_sha256_v1` contract. It parses the complete
`competitive_slam_profile` YAML mapping, removes only
`evidence_gate_v2.execution_selection_receipt_sha256`, then serializes the
mapping as UTF-8 JSON with sorted keys, compact separators, and
`ensure_ascii=true` before hashing with SHA-256. The receipt stores that value
as `common_identity.profile_sha256` plus its hash kind. The profile continues
to store the raw full-file SHA of the receipt, so YAML formatting changes are
visible there without creating a mutual-hash cycle. Any other profile field
mutation changes the canonical profile hash; missing, wrong-kind, or mismatched
values remain fail-closed (`INCOMPLETE` for unresolved pending data and
`INVALID` for malformed/tampered data).

To refresh the pending identity without touching the reviewed receipt, create
an observation artifact and then finalize it against the same receipt:

```bash
python3 scripts/capture_competitive_execution_identity.py capture \
  --receipt configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --output <out>/execution_identity_capture.json \
  --yaml-output <out>/execution_identity_capture.yaml
python3 scripts/capture_competitive_execution_identity.py finalize \
  --receipt configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --capture <out>/execution_identity_capture.yaml \
  --output <out>/execution_identity_finalize.json \
  --yaml-output <out>/execution_identity_finalize.yaml
```

Both commands are read-only with respect to the receipt. Capture records the
current worktree provenance, machine fingerprint, OpenMP-related environment,
and locally available Docker image IDs. For an explicitly bound local image,
capture also runs bounded `--pull=never --network none --read-only` probes for
compiler/linker/ROS/PCL/Eigen/OpenMP and binds the result to the inspected
image digest; a source checkout binding supplies Git provenance only. It does
not pull/build images or open fresh bags/GT. Finalize refuses a capture
from another receipt and cannot promote `pending` to `ready`/`frozen`. The
current worktree therefore produces `INCOMPLETE`, as required; only a later
reviewed clean revision with system-container toolchain identities and a
complete equal thread policy can be explicitly frozen.

### NTU VIRAL second-family preregistration

The dataset-source closure now carries a second distinct GT family as a
metadata-only preregistration:
[`ntu_viral_selection_2026-08.yaml`](../configs/slam_benchmark_profiles/ntu_viral_selection_2026-08.yaml)
and its strict schema
[`ntu_viral_selection_v1.schema.json`](../configs/slam_benchmark_profiles/ntu_viral_selection_v1.schema.json).
The selection is deliberately `NOT_READY`; adding a family declaration does
not count it as a PASS family. Claim eligibility still requires at least two
families whose every evaluation-eligible sequence has recorded and
mount-identity-bound, byte-revalidated input, calibration, and GT identities.

The official NTU VIRAL project page documents two 3-D lidars, two
time-synchronized cameras, multiple IMUs, and UWB nodes, and names the chosen
sequences and their environments/durations: `eee_01` (EEE central carpark,
398.7 s), `nya_01` (inside Nanyang Auditorium, 396.3 s), and `spms_01` (SPMS
facade, 446 s). The project page is the primary citation for those facts:
<https://ntu-aris.github.io/ntu_viral_dataset/>. Its official dataset source
repository is <https://github.com/ntu-aris/ntu_viral_dataset>, and the official
CSV GT repository is <https://github.com/ntu-aris/ntuviral_gt>. The evaluation
tutorial documents the Leica prism/body offset and nanosecond timestamp
handling: <https://ntu-aris.github.io/ntu_viral_dataset/evaluation_tutorial.html>.
The published dataset terms are CC BY-NC-SA 4.0 for non-commercial academic
use; the terms page is <https://creativecommons.org/licenses/by-nc-sa/4.0/>.

The official page points to NTU Data Repository and an OneDrive fallback. Those
are moving references, so no immutable source revision, archive digest, exact
byte size, or SHA-256 is invented here. Before any execution, the selection
requires the materialized input, GT, and calibration SHA-256/byte-size values,
the exact source/archive identity, calibration and sensor-topic/frame/time
revalidation, and support receipts for `ours`, `glim`, and `fast_livo2`.
Expected roles are the official sequence archive/bag, GT in the bag or the
official CSV export, `calib_stereo.zip`, `calib_stereo_imu.bag`, and the
canonical calibration/metadata tree.

`tnp_01` is explicitly represented only in the development profile group as
`development_training_exposed`, because existing m6a10 work has exposed it.
The selection receipt marks it `evaluation_eligible: false` and
`fresh_eligible: false`; it cannot supply the second family or be relabelled as
fresh. The closure auditor also rejects guessed byte identities, reordered
selection bindings, and any attempt to re-enable `tnp_01`.

#### NTU VIRAL acquisition and pin review sequence

The NTU materialization has two deliberately separate stages. Stage A reads
the preregistered selection and profile, verifies the exact evidence mount
(`/media/sasaki/aiueo1`, UUID `3b5dc9b7-c4de-4cf2-a892-00b2c063f34e`, ext4,
label `aiueo`, read-write), and reserves one previously absent candidate root.
It accepts only the exact allowlisted official HTTPS role URLs and expected
filenames in the selection. Redirect chains and final URLs are recorded and
must remain on the allowlist; archives are inspected without extraction and
reject traversal, symlink, hardlink, duplicate, and special members. A
failure after reservation seals `FAIL_CLOSED` evidence, including any partial
download, and that root is never reused or overwritten.

Once exact official URLs, filenames, and source identities have been reviewed,
the production entry point is:

```bash
python3 scripts/acquire_ntu_viral_candidate.py \
  --selection configs/slam_benchmark_profiles/ntu_viral_selection_2026-08.yaml \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --evidence-root /media/sasaki/aiueo1
```

The current selection intentionally contains moving-reference metadata only
(`url: null` and no byte identities), so this command must fail before any
reservation until a separately reviewed selection revision supplies the exact
official role URLs and expected member paths. Synthetic tests inject a local
fixture transport and mount observation; they never use `/media`, network, or
benchmark data.

Stage B is offline. It reopens the sealed candidate receipt and every
candidate byte, verifies receipt/sidecar/source/role/selection/profile hashes,
and writes only a deterministic proposal:

```bash
python3 scripts/authorize_ntu_viral_pin.py \
  --selection configs/slam_benchmark_profiles/ntu_viral_selection_2026-08.yaml \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --candidate-root /media/sasaki/aiueo1/datasets/ntu_viral_candidates/ntu-viral-historical-selection-2026-08-v1 \
  --output /tmp/ntu_viral_pin_proposal.json
```

The proposal is always `PROPOSED_REVIEW_REQUIRED` and
`claim_eligible: false`; the authorizer never edits the profile. Ground-truth
artifacts remain role-separated and the runner input manifest contains no GT
path. A separate reviewer must install an immutable `REVIEWED` profile pin,
its sidecar, and `REVALIDATED` input/GT/calibration identities. Dataset
preflight accepts that reviewed profile pin only; it never accepts a Stage A
candidate receipt or Stage B proposal. Until that review and byte
revalidation exist, the NTU family remains `NOT_READY` and cannot make the
two-family or competitive claim gate pass.

### Competitive RSS claim gate

The v2 suite and sequence evaluators retain the legacy aggregate RSS value as
report-only diagnostics until a producer declares claim eligibility or a
resource receipt.  A claim path then reopens every expected system/sequence/
repetition row and requires a distinct, positive, finite
`aggregate_process_tree_peak_rss_bytes` receipt.  The receipt must be
authoritative (`benchmark_process_rss_authoritative_v1`); functional-only
registration-plugin records, contaminated timing authority, and incomplete
resource rows are ineligible.

Each receipt binds the m6a7 resource-tool revision and sampler/helper SHA-256s,
configuration, thread-policy hash, machine, hardware, and exact `Release`
identity.  The gate requires at least three matched complete runs for every
system and sequence, checks every pinned rival (including the
profile-selected best rival), and deterministically aggregates the maximum
valid peak over repetitions and sequences.  The single canonical
`max_peak_rss_ratio_vs_best_rival: 1.20` profile value is used for the
aggregate, every-rival, and per-sequence ratio checks; the per-sequence
ceiling is bound to that source and cannot be relaxed independently.  The
95% bootstrap/CI requirement is scoped to APE only; RSS makes no CI claim and
uses the preregistered deterministic maximum.  The policy shape is registered
in
[`competitive_memory_gate_v1.schema.json`](../configs/slam_benchmark_profiles/competitive_memory_gate_v1.schema.json).
These checks do not open ground truth or invoke a scorer, and existing
benchmark evidence is not upgraded by this code.

### Fresh-holdout authorization and GT-blind claim gate

The fresh partition has a second, claim-only authorization contract in
addition to the input and execution identities above. Before any ground-truth
availability or access, a sealed authorization mapping must precommit every
fresh slot's immutable dataset/input/calibration/GT identity (GT is exposed
only as SHA-256 plus byte size), every required system's pinned revision,
config, hardware, thread policy, and exact `Release`, the run count, scorer
revision/config/fingerprint, profile hash, and metric-gate hash. A GT path,
URL, URI, decoded content, or content-derived value is forbidden in that
pre-GT manifest.

The authorization receipt chain uses canonical JSON SHA-256 receipts with
strictly increasing UTC timestamps and predecessor links. Its required order
is `precommit`, `holdout_seal`, `replay_authorization`, `leakage_audit`, and
`failure_record`; reordered timestamps, missing/duplicated chain entries, or
revision/config drift fail closed. The replay process is `replay_only` with no
GT mount, GT open, or scorer invocation. Scoring is a distinct
`scoring_only` process and may record exactly one authorized event for a
sealed bundle, or an explicit invalidation; overwrite/retry and duplicate
scoring events are forbidden. The leakage audit must document no prior GT
access, development tuning, result-dependent selection, reused holdout, or
GT path exposure, and the failure ledger must be complete.

The local verifier is
`scripts/competitive_holdout_authorization.py`, integrated into
`scripts/verify_competitive_evidence_bundle.py` and the v2 suite claim gate.
Its schema is
[`competitive_fresh_holdout_authorization_v1.schema.json`](../configs/slam_benchmark_profiles/competitive_fresh_holdout_authorization_v1.schema.json).
The profile-side trust-store shape is specified by
[`competitive_external_attestation_trust_store_v1.schema.json`](../configs/slam_benchmark_profiles/competitive_external_attestation_trust_store_v1.schema.json).
Fresh immutable dataset IDs/hashes are also checked disjoint from historical,
bring-up, development, and regression partitions; shared calibration hashes
are intentionally not treated as dataset overlap.

When the policy requires an attestation, `external_attestation.status: PASS`
alone is insufficient. The profile must carry a versioned `trust_store` with
a canonical store SHA, unique key IDs, Ed25519 public-key bytes and SHA-256,
ACTIVE/REVOKED state, and UTC validity intervals. The verifier trusts only
that precommitted store; it never trusts a public key or key ID self-declared
by the attestation. A detached canonical-base64 Ed25519 signature covers a
domain-separated canonical payload containing the complete authorization
mapping except `external_attestation`, so chain, scoring, leakage, failure,
GT identity, and precommit edits invalidate it. Payload SHA, algorithm, key
identity, validity, revocation, and signature are all checked; a separate
`external_attestation_binding_sha256` in the signed authorization binds the
attestation metadata without circularly signing its own signature/payload
fields. Missing
trust-store material or the Ed25519 dependency is `NOT_READY`; malformed,
unknown, revoked, expired, or cryptographically invalid material is
`FAIL_CLOSED`. Key rotation is represented by multiple versioned anchors and
explicitly revoked/expired entries. The profile also pins the verification
backend name, exact `python-cryptography` version, and Ed25519 implementation
symbol; the verifier records that backend identity and verification timestamp
in its result. No private key is stored in the repository. Attestation expiry
is intentionally wall-clock based: a later re-check can become `FAIL_CLOSED`
after expiry, so the sealed receipt must retain the original verification
time and the profile must define the allowed validity window.

The checked-in profile currently records
`fresh_holdout_authorization.status: NOT_READY`: no independent external
custodian trust anchor or attestation is present. This is not replaced by a
runner or scorer signature. Before a claim can become eligible, an independent custodian must
provide a locally verifiable signature over the sealed authorization chain
(including its chain head and bundle identity), with signer/public-key
identity recorded outside the runner/scorer trust domain. Until that external
attestation is supplied and the profile is reviewed, the actual benchmark
evidence remains `NOT_READY`; no GT content is opened by these checks.

For a measured local checkout or image, bindings are explicit and repeatable;
they never clone, build, or download anything:

```bash
python3 scripts/capture_competitive_execution_identity.py capture \
  --source ours=/path/to/ours \
  --source glim=/path/to/glim \
  --source fast_livo2=/path/to/FAST-LIVO2 \
  --image glim=glim-cpu-benchmark:competitive-v1 \
  --image fast_livo2=fast-livo2-benchmark:ros1-pinned \
  --receipt <receipt.yaml> --profile <profile.yaml> \
  --output <out>/capture.json
```

When a rival checkout or local image is not bound, the observation contains a
machine-readable probe manifest with the exact compiler/linker/ROS/PCL/Eigen/
OpenMP commands still required; it does not infer readiness. A complete
synthetic or clean ready/frozen contract can return `PASS`; any receipt left
pending remains `INCOMPLETE` until an operator explicitly reviews and updates
it. The current M5d execution receipt is ready, while the evidence gate still
awaits benchmark run records.

### M6a0 GT-blind execution plan (2026-08-22)

Before any fresh-holdout replay, generate the read-only 27-attempt plan:

```bash
python3 scripts/run_competitive_gt_blind_benchmark.py \
  --input-root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --output-root /media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_20260822 \
  --profile configs/slam_benchmark_profiles/competitive_slam_v1.yaml \
  --receipt configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --dry-run --plan-output /tmp/m6a_gt_blind_plan.json
```

The driver does not launch containers in `--dry-run`. `--preflight` adds
immutable image inspection and frozen raw/canonical input hashing. M6a1
rebuilt the ours image from algorithm revision
`866f733677e92ecb08d67126e463da99dd140d46`; its immutable image ID/digest is
`sha256:0680ae359deb2da45ff16ecf1c5d92c0510dc51d48bd06c0fcd93ce1d33ff3fb`.
The separate GT-blind harness/orchestrator revision is
`4701f0084d6b0fff475a62bec7eeb6d807561821`, not an algorithm revision.
Receipt/profile hashes were resynchronized without changing the canonical
profile identity. Read-only dry-run and preflight passed all 27 scheduled
attempts; M6a2 subsequently exercised `--execute` once, and its incomplete
GT-blind completion manifest is recorded below. Ours and GLIM
mount only the canonical ROS 2 directory; FAST-LIVO2 mounts only the selected
raw ROS 1 bag and records the frozen raw/canonical semantic-equivalence hash.
No GT path, calibration tree, scorer, APE, or map-quality input is passed to a
container by this harness. Preflight hashes each frozen slot once despite the
repeated system/repetition schedule.

### M6a2 GT-blind execution result (2026-08-22)

The one permitted `--execute` pass covered all 27 scheduled attempts in the
managed results root. Completion is `INCOMPLETE` (manifest SHA256
`a5abafdeb420619a1460737a3cf91862fa41ce7930c041324b1f38c00b16f002`):
ours had nine exit-1 RKO-LIO startup failures, GLIM had nine exit-250
read-only ROS-log failures, and FAST-LIVO2 had nine exit-20 ROS-master
self-connect failures under the network-disabled container policy. Attempt
directories are immutable and no `.part` directory remains. The completion
manifest proves GT content was not opened and no scorer was invoked, but it
contains no valid trajectory/performance evidence. Repair and smoke-test these
three runtime contracts before any rerun; do not infer accuracy, performance,
map quality, or SOTA superiority from this incomplete attempt.

### M6a3 GT-blind remediation and preflight closure (2026-08-22)

The failed M6a2 campaign is immutable and remains the parent failure record;
its completion manifest SHA-256 is
`a5abafdeb420619a1460737a3cf91862fa41ce7930c041324b1f38c00b16f002`.
It is an infrastructure-failed campaign, not a replacement or a benchmark
result. The remediation keeps the algorithm revision
`866f733677e92ecb08d67126e463da99dd140d46` unchanged:

- ours now builds and installs the pinned `rko_lio` runtime, checks its package,
  executable, config, archive SHA, and writes ROS launch state only below the
  attempt output;
- GLIM writes ROS state below the attempt output and its synthetic smoke uses
  process-group SIGINT for bounded clean shutdown;
- FAST-LIVO2 keeps `network=none` and uses only loopback
  `ROS_MASTER_URI`/`ROS_IP`/`ROS_HOSTNAME`.

The rebuilt ours image is
`m6a3c-lidarslam-ours:jazzy@sha256:18198c17627459e96c574b1bf3093064c9c092f4fc2f89594b7e4b14705288bd`.
Synthetic ROS2/ROS1 fixtures exercised startup, bag open/input parsing, and
clean shutdown for all three wrappers. The normalized evidence is under
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a3_20260822/smoke/normalized`;
the ours, GLIM, and FAST tree SHA-256 values are respectively
`12e56407c0750bf47ff3aae307c4a96e78e056f84121421de10ed6d9825216ec2`,
`2e74d9659c04007af20e4fb5999087f1a6cbaac4a4cf3df47f6eacc8562edbed`, and
`8293e1f1771f9b60a8d11a43998f538e6b965341bc3399b4aeda81c1a209ce17`.
All three contract markers report `performance_run: false` and
`gt_mounted: false`.

The disclosed rerun uses the separate results root
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_campaign2_20260822`.
Its read-only dry-run has 27/27 attempts and plan SHA-256
`465e856846eb4bcf15e125285a914d2defda72413b5016e2965b884cda2f8bd8`.
The preflight plan is `preflight_ready`, has 27/27 attempts, plan SHA-256
`7aeac10cdeb1c8356e282b53a62f4f3a05621f37942d325d58302a8f6464cd43`,
and records profile canonical SHA
`5a8b81b7483ce9921fdfb4393ed006390fc5f0b2553b5e29976de96725a4da39`,
selection SHA
`2bfc541a8d6127599f7a36e66c08da44488a08a55a4d9c4709703223be8bdd2b`,
and execution-receipt SHA
`d25057bc3660424f9ea852143a215610b8395dddfee0ed1e8c64a5acd1cde0ab`.
At the M6a3 checkpoint GT content and scorer remained untouched and no
`--execute` had been run. The later M6a4 partial record below is the complete
execution status; neither checkpoint authorizes accuracy, performance, map, or
SOTA claims.

### M6a4 partial GT-blind campaign closure (2026-08-22)

The disclosed campaign2 execute was started once after the fixed preflight.
Attempts 001--018 are immutable final attempts: ours and GLIM each completed
all three frozen slots with three repetitions (18/18, exit 0, complete output,
and GT/scorer-free proofs). FAST attempt 019 produced its wrapper status and
odometry artifacts, but the driver failed while finalizing the attempt because
`attempt_019.part/ros_home/rospack_cache_15823137030970321179` was mode 600,
owned by root, and unreadable to the driver user. The `.part` directory is
preserved unchanged; no attempt 019 final manifest exists, and attempts
020--027 were not started. The exact partial record is the external atomic
manifest
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_campaign2_20260822/partial_campaign_manifest.json`
(SHA-256
`df91a0f0852911790f3fabb5b5638938b8e1222208943be01678a99fbd062978`). Its
non-circular root-tree projection SHA is
`ceafd30fbfe4f0099ba32e4a79390f8dd5cc0e6ea428254c8d021c11231bd16e`.

The partial campaign is `INCOMPLETE`, not a benchmark result. The host
`/usr/bin/time -v` values recorded around `docker run` are client-process RSS,
not container/cgroup peak memory, so RSS evidence is invalid for a performance
gate. GT content and scorers were never accessed. The immutable M6a2 parent
completion SHA remains
`a5abafdeb420619a1460737a3cf91862fa41ce7930c041324b1f38c00b16f002`.
A planned campaign3 must first provide resilient attempt finalization and
container/cgroup memory accounting; no M6b or README/SOTA claim is authorized.

### M6a5 container-memory contract and campaign3 preflight (2026-08-22)

M6a5 closes the measurement-accounting gap without starting another benchmark.
The comparison RSS field is exclusively `container_cgroup_peak_bytes` from
the container's cgroup-v2 `memory.peak`; the host `/usr/bin/time -v` value is
retained only as the diagnostic `docker_client_peak_rss_kb`.  A cgroup-v2
`memory.max` value of `max` is valid and is recorded as an unlimited limit.
The helper writes an atomic, host-readable evidence file and includes all
children in the container scope.  Missing, malformed, non-atomic, or
unreadable evidence is fail-closed.

The external smoke summary is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a5_20260822/memory-max-unlimited/summary.json`
(SHA-256
`e2df6b70dcec703406a406ed7a4c45aaac87b69987d87966f4d10bceee1c85bb`).  The
known-allocation check increased the cgroup peak by `138006528` bytes while
the Docker-client RSS remained approximately constant; this validates the
measurement scope, not SLAM performance.

Campaign3 is a disclosed preflight-only successor at
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_campaign3_20260822`.
Its final evidence directory is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a5_20260822/campaign3-preflight-final`:
the dry-run plan SHA is
`bef6184b506b852288cf07b94772442cdc47c6c58e2cef45da5140249729d082`, the
preflight plan SHA is
`362ad49f85491bfd74ddda181ec0a334c972e4be2818374a05b0db3c5724087b`, and
the summary SHA is
`9ae7115d10dee09f5287317b5e9a3912580f7ae31cfd93e8686162149e108bf0`.
All 27 scheduled attempts are preflight-ready, but the campaign has zero
attempts, `execute_started=false`, and both GT content access and scorer use
are false.  The historical M6a5 checkpoint recorded execution-receipt raw
SHA
`d89d30d9e516f7d7211536bd1ea4f837ae95c4f7aa4527af60a5f7699c3d677f` and
profile canonical SHA
`5a8b81b7483ce9921fdfb4393ed006390fc5f0b2553b5e29976de96725a4da39`;
M6a7 later resynchronized these identities after adding its audited metric
contract (the current values are recorded in the M6a7 section below).
Campaign1 and campaign2 remain immutable incomplete lineage records.  No
M6b scoring, accuracy, performance, map-quality, or README/SOTA claim is
authorized by this preflight.

### M6a6 campaign3 GT-blind closure (2026-08-22)

Campaign3 was executed once from the fixed M6a5 preflight and then closed
without opening GT content or invoking a scorer.  The completion manifest is
27/27 final attempts with no `.part` directories.  Ours completed 9/9 and
GLIM 9/9.  FAST-LIVO2 completed 6/9; attempts 022--024 (FAST-LIVO2,
`fresh_2`) are immutable formal failures with exit status 22 and
`complete=false`.  The completion manifest SHA is
`31df60ff2775d2f2a699e7559c16cc5d732a91e175f12af1db06bc47e4b8cd5b`.
The read-only GT-blind integrity summary is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a6_20260822/completion/integrity-summary.json`
(SHA-256
`af27ae2c7d019db790cace3b38b250c2a5dcd3340e58fa5a91158293b2bf2024`).
The closure manifest is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a6_20260822/closure/m6a6-campaign3-closure.json`
(SHA-256
`44d50669e33dbb105300b2b3f926c99f891341373e6a1dfa71ae9c85acf954b8`).

This remains `INCOMPLETE`, not a benchmark result.  The recorded
`container_cgroup_peak_bytes` is cgroup total footprint (including page
cache), not process RSS; several runs reached the configured 4-GiB cap, and
the cold/warm spread (for example, attempt 001 versus 002) means reclaim and
cache effects cannot be separated.  `docker_client_peak_rss_kb` is retained
only as a diagnostic.  `memory.events` was not recorded by the M6a5 evidence
schema, so no pressure or reclaim conclusion is inferred.  Consequently RTF,
RSS, accuracy, map-quality, and SOTA gates are invalid/insufficient here.
Campaign lineage is campaign1 immutable failure -> campaign2 immutable
partial -> campaign3 incomplete -> planned campaign4.  M6b is not
authorized.

### M6a7 process-tree RSS audit and campaign4 gate (2026-08-22)

Before planning campaign4, the M6a7 evidence was re-audited read-only with the
correct GNU `time` parser (including indented `Exit status:` lines) and strict
schedule/mode validation.  The final audit is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a7_20260822/v3_final_audit_tool_final_20260822/v3_run_audit.json`
(SHA-256
`bd7f57cd2cb6fe8b93a9c28d7b193968d3e865180f2985fb66c44c736e7cd818`), and its
normalized receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a7_20260822/v3_final_audit_tool_final_20260822/m6a7_v3_final_receipt.json`
(SHA-256
`06985d4c4900dbe1aa27687c023c4b6c2ddd3c362ed6782c92a048f5e1eae62e`).
The audit accepts exactly 20 AB/BA pairs (40 runs), verifies every run's
Docker/host exit status and mode, and rejects missing, duplicate, or unexpected
directories.  Earlier failed audit attempts remain immutable lineage evidence;
they are not overwritten by this PASS audit.

The comparable memory metric for the next campaign is the aggregate
process-tree peak RSS, `aggregate_process_tree_peak_rss_bytes`, defined as the
sum of each process's peak `VmRSS` (shared pages may be counted once per
process).  Sampling is 250 ms with scheduler nice 10, cgroup v2
`memory.max=max`, and zero OOM/`oom_kill` delta required.  Cgroup total memory
and host Docker-client RSS are diagnostics only; the latter is explicitly not
comparable.  M6a7's measured overhead gate passed (median absolute 1.8513%,
bootstrap 95% upper 4.1914%), as did signal, allocation/cache-separation, and
all three synthetic wrapper smoke contracts.  This is measurement-contract
evidence, not SLAM accuracy or SOTA evidence.

The checked-in execution receipt and profile bind the audit/receipt/summary
file SHAs using the existing `canonical_profile_sha256_v1` projection.  The
profile excludes only its registered execution-receipt file SHA; no mtime or
self-referential receipt hash is used.  At this M6a7 checkpoint, campaign4
was a separate GT-blind, scorer-free dry-run/preflight gate with a fresh
results root and zero attempts; no performance execution was authorized by
that checkpoint.

The campaign4 read-only plans were then generated against a fresh, disjoint
results root
`/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_campaign4_20260822`.
The final deterministic dry-run plan SHA is
`1ffa6836abc9aa94bb41e310de4ff2e50c9f336335b23779f0ccfa1236ea09f2`; a
repeat dry-run produced the same bytes and SHA.  The final 27/27 preflight
plan SHA is
`a8c953e59b6a7dd70891fcdf3c57c791f9735ea6173dda114cd365e188562e4e`.
Both final plans contain the fixed 27-attempt schedule and the M6a7
process-tree RSS contract; at that checkpoint preflight status was
`preflight_ready` and the results root was empty (`attempts=0`).  The plans bind selection SHA
`2bfc541a8d6127599f7a36e66c08da44488a08a55a4d9c4709703223be8bdd2b`,
algorithm revision `866f733677e92ecb08d67126e463da99dd140d46`, all three
immutable image digests, canonical profile SHA
`cbb093233b2740e0624fbd348ac293a705fd69e7fa1825723a4e7e493736cc25`, and
execution-receipt SHA
`c58d11881f88dd7ea6fef05f1e91edf901ca5eaa23076ff673306580885d14f7`.
They are GT-blind and scorer-free; `--execute` remains a separate, explicitly
unauthorized action in this checkpoint.

### M6a8 campaign4 GT-blind completion (2026-08-22)

The fixed campaign4 root was run only after the third quiescence window
passed.  The pre-run snapshot is recorded at
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/m6a8_20260822/campaign4-run-start-20260822/pre_run_snapshot.json`
with SHA-256
`07f97ffc4241be205a0678fa58bd5defc66fa6bc818d83716d901e3177ec2341`.
Two earlier driver starts failed before any attempt because the existing empty
root was rejected by the no-overwrite contract; their immutable evidence is
`4bda0f3811e7bf3f9f4d9e38ea91b6f9c9be965e8272824c1862bb955fab9bd4` and
`26873c3a35b5c70894ebccc24dd694c05060abc450195d7930423c9f6f9119fd4`.
After verifying that root was empty, it was removed and the driver-owned mkdir
recovery created the actual campaign root.  No container or attempt was
created by either failed start.

Campaign4 completed the registered 27-attempt schedule (three systems × three
fresh slots × three repetitions) with exit status zero and complete output
contracts.  The immutable external artifacts are:

- results root: `/media/sasaki/aiueo1/benchmarks/competitive_results/m6a_gt_blind_campaign4_20260822`;
- `completion_manifest.json`, SHA-256
  `f63b14f52c0b22b957f897568aa38f5a2fce26fea47bd2ca5917f6fec43c1c74`;
- `integrity_manifest.json`, SHA-256
  `f28ba05ea5c8fce0a6944ddcc7353d3dd5ea38dca927a7dcef7e4131983b6e5b`;
- `closure_manifest.json`, SHA-256
  `f2b1b6e943f1c30cd54636c9221bd1eb38d18192cb1cce1f583fd17bf8b1c59a`.

All attempts are GT-blind: `ground_truth_content_opened=false`,
`ground_truth_reachable=false`, and `scorer_invoked=false`.  There are no
`.part` files or residual Docker/sampler processes.  Every attempt passed the
aggregate process-tree RSS evidence contract, used cgroup v2
`memory.max=max`, and had zero OOM/`oom_kill` delta.  Docker-client RSS and
cgroup total memory are retained as diagnostics only.

This is a completion and integrity receipt, not a victory result.  The
closure status is `VALID_COMPLETE_GT_BLIND_RTF_GATE_NOT_PASSED`: all finite
processing RTFs were derived from the opaque bag metadata, but FAST-LIVO2's
maximum was `1.173445611`, exceeding the registered `maximum_processing_rtf`
limit of `1.0`.  No GT was opened, no scorer or accuracy/map-quality metric was
run, and no SOTA or M6b claim is permitted.  The fixed canonical profile and
execution identity remain bound to canonical SHA
`cbb093233b2740e0624fbd348ac293a705fd69e7fa1825723a4e7e493736cc25` and
receipt SHA
`c58d11881f88dd7ea6fef05f1e91edf901ca5eaa23076ff673306580885d14f7`.

### M5c fresh-holdout download checkpoint (2026-08-21)

Fresh input acquisition is a separate, opaque-hash-only checkpoint. The
selection receipt is reviewed independently first; this tool never edits the
selection receipt or competitive profile. Use an explicit destination on the
benchmark storage volume. The read-only `plan` action is the first step and
does not access the network or dataset contents:

```bash
python3 scripts/freeze_competitive_fresh_holdouts.py plan \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --output /tmp/fresh-holdout-plan.json
```

After a separate review of that plan and selection receipt, run the actions in
this order:

```bash
python3 scripts/freeze_competitive_fresh_holdouts.py download \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821
# If a transfer is interrupted, use this instead of repeating download:
python3 scripts/freeze_competitive_fresh_holdouts.py download --resume \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821
python3 scripts/freeze_competitive_fresh_holdouts.py verify \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821
```

Before `finalize`, prepare the canonical ROS 2 tree and semantic report from
the verified raw bags. The preparation command is sequence-scoped or can cover
all managed manifests; it requires `rosbags==0.11.0` and uses the fixed
`rosbags-convert` command recorded in its preparation receipt:

```bash
python3 scripts/prepare_competitive_fresh_ros_inputs.py \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --all
# Or, for one slot (and --resume only after an interrupted preparation):
python3 scripts/prepare_competitive_fresh_ros_inputs.py \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --sequence exp14 --resume
```

It rechecks only the raw-bag byte count/SHA from each
`downloaded_hashed` manifest and never opens the manifest's GT path. Each
conversion is written to `slots/<seq>/canonical_ros2.part`, checked for the
seven-topic contract, compared against the raw ROS 1 bag with
`compare_rosbag_semantic_inputs.py`, then atomically published as
`canonical_ros2/`, `semantic_equivalence.json`, and
`preparation_receipt.json`. The receipt binds the plan SHA, manifest/raw
identity, Python/NumPy/rosbags versions, converter/comparator script hashes,
exact argv, ROS 2 tree hash, and semantic report hash. Existing output is
accepted only when that receipt and all hashes still match; the final receipt
is the commit marker. A crash after conversion, comparison, or either of the
first two atomic renames is resumable only when each artifact has exactly one
of its `.part`/final forms; a staged receipt must validate its full identity,
while a converter/comparator partial without a receipt is only accepted after
its safe tree/report validation. Mixed or symlinked output fails closed. After
this step, pass
`slots/<seq>/canonical_ros2` and `slots/<seq>/semantic_equivalence.json` to
the downloader's `finalize` command. The external managed root used for this
checkpoint has been converted and deep-verified; its fresh slots are now
`frozen_unopened`. Receipt/profile updates are a separate reviewed operation.

Only after preparation and its separate review, publish the downloader's
final state:

```bash
python3 scripts/freeze_competitive_fresh_holdouts.py finalize \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --ros2-root exp14=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp14/canonical_ros2 \
  --ros2-root exp16=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp16/canonical_ros2 \
  --ros2-root exp18=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp18/canonical_ros2 \
  --semantic-report exp14=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp14/semantic_equivalence.json \
  --semantic-report exp16=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp16/semantic_equivalence.json \
  --semantic-report exp18=/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821/slots/exp18/semantic_equivalence.json
```

`--resume` is only for a managed, identity-matching staging directory; a
complete final slot is re-verified and skipped, while a stale or mixed final/
staging tree fails closed. The plan and managed-root marker bind the selection
receipt SHA and the runtime SHA-256 of this producer script, so changing the
producer or selection contract cannot reuse an old download. Raw bags are
checked by expected byte count and official LFS SHA-256. Ground truth is never
parsed or printed: only its opaque byte count/SHA-256 is recorded. Calibration
files are checked by bytes, SHA-256, Git blob identity, and a canonical logical
tree hash; storage paths are kept separate from logical paths. `finalize`
verifies every manifest before calculating the canonical ROS 2 input identity,
and publishes each state atomically. The output root is
`/media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821` for the
current preregistration. The M5d review deep-verified this managed root and
recorded selection/profile/execution identities. Ground truth remains opaque
and no benchmark/metric was run; no README/SOTA claim follows from acquisition
metadata.

After a reviewed `frozen_unopened` tree exists, run the independent deep
verifier before using it in a benchmark:

```bash
python3 scripts/verify_competitive_frozen_holdouts.py \
  --root /media/sasaki/aiueo1/benchmarks/competitive_holdouts/fresh_20260821 \
  --selection configs/slam_benchmark_profiles/fresh_holdout_selection_2026-08.yaml \
  --output <out>/frozen_holdouts_deep_verification.json
```

The verifier requires exactly Exp14/16/18 and rechecks the managed marker,
selection/plan identities, official bag byte/LFS-SHA identity, calibration
byte/Git-blob identity, and every final manifest. The bag's preregistered
Git-blob OID is the immutable Git LFS pointer provenance; it is format-checked
and reported, never compared with the downloaded bag's content blob. Ground
truth remains an opaque stream: no GT path, text, trajectory, or score is printed.
The verifier recomputes the safe
canonical ROS 2 metadata/tree hash, seven-topic semantic report hash,
input-manifest payload hash, and the preparation receipt's deterministic
pre-finalization manifest **file** hash (canonical compact JSON plus its
trailing newline; distinct from the newline-free payload hash). Its JSON
summary includes each manifest and
preparation-receipt file SHA. Missing slots, symlink/path traversal, stale
receipt/runtime/argv identity, or artifact tampering are hard failures. The
M5d invocation passed against the managed root and records the enriched
selection SHA plus its explicit committed preregistration anchor; no GT
content or metric was parsed. The selection/profile/execution identity update
was reviewed separately from the freezer.

### Pinned benchmark image recipes

The checked-in execution receipt now names a repo-owned build recipe and build
entrypoint for each system. Run the entrypoint only after the source revision
and execution identity have been reviewed:

```bash
bash scripts/build_competitive_benchmark_images.sh --system all
```

The recipes use immutable `sha256` base-image references, pin the ours/GLIM/
FAST-LIVO2 source revisions, and set the recorded CPU-only thread environment.
The ours recipe receives a Docker context containing only its Dockerfile. It
clones the public `lidar_slam_ros2` repository at `OURS_REVISION`, verifies the
`ndt_omp_ros2` gitlink and initializes only that build-required submodule, then
checks the detached HEAD, clean status, and initialized submodule status before
rosdep or compilation. The pinned `rko_lio` gitlink is supplied as an exact
local archive whose SHA is checked against the pinned gitlink; no public-mirror
substitution is used. This prevents a dirty host checkout or a source archive
with missing submodule contents from entering the image.
GLIM's CPU path does not consume PCL; its receipt explicitly records `pcl` as
`not_applicable`, and the container probe fingerprints that sentinel rather
than installing an unused package or falling back to the host. The capture
tool only permits this exception for GLIM/PCL; compiler, linker, ROS, Eigen,
OpenMP, and all ours/FAST fields remain mandatory and fail closed.
The FAST-LIVO2 recipe builds its ROS 1 workspace under `/opt/fast_livo_ws`;
its pinned image and system-container toolchain probe are now observed ready;
it does not depend on the historical undocumented
`fast-livo2-benchmark:noetic` or `hdl_localization_noetic:local` images. Its
legacy Sophus compatibility commit is an explicit full-length build-time pin.
FAST's upstream HILTI22 configuration is now bound to the exact
commit-addressed FAST archive member `config/HILTI22.yaml`, with SHA-256
`efae9e702c71c770b19002b6e19d4e1b6f46c67df3727e984981d932258f0b4a`. The
previous external-container path is no longer a recipe input. This proves
reproducibility of the pinned upstream bytes, but does not resolve the
remaining component-license provenance or permit image redistribution.
Fresh execution inputs remain pending.
`--pull=false` is
intentional: a missing base image or source ref must fail rather than silently
changing the identity.

This recipe wiring is provenance infrastructure, not benchmark evidence. The
three pinned image/toolchain observations and the fresh-input identity are now
marked ready after clean builds, read-only probes, and deep verification. The
checked-in receipt is ready for a first run; this does not constitute benchmark
evidence or an accuracy/SOTA claim.

The checked-in synthetic tests cover the exact 10% boundary, missing and
failed runs, old schema, false freshness, pending slots, dataset hash
mismatches, invalid fingerprints, identity/RTF failures, within-run CI
variance, sequence collapse, per-dataset map regression, and all-rival
bootstrap superiority. No real fresh-slot competitor receipt currently
satisfies this contract; existing exp02/exp21 assets with failures or RTF
above one remain negative evidence. README superiority claims are unchanged.

## SLAM candidate regression

Run plane-revisit OFF/ON with the same backend input and reference:

```bash
bash scripts/run_plane_revisit_candidate_benchmark.sh \
  --dataset mid360_public --bag <backend-input-bag> \
  --reference-tum <reference.tum> --fixed-loop-edges <verified.csv> \
  --output-dir /media/<ssd>/benchmarks/phase7/mid360
```

Repeat with `hilti_exp04` and `rtkslam_construction_seq2`. Construction Seq2 is
the required second positive sequence; its total-station checkpoints contain
positions but no surveyed orientations, so rotational RPE is forbidden.
`--dry-run` prints the pipeline without starting ROS. For submap-rate backend
poses, supply the matching frontend trajectory with `--dense-raw-tum`.

To create a deterministic backend input from the original sensor bag:

```bash
ROS_DOMAIN_ID=210 ROS_LOCALHOST_ONLY=1 \
bash scripts/record_backend_input.sh --output-dir <work>/backend_input -- \
  bash scripts/run_rko_lio_graph_benchmark.sh \
    --bag <construction_seq2> --lidar-topic /livox/points \
    --imu-topic /livox/imu --skip-reference-gen \
    --reference-tum <construction_seq2_gt.tum> \
    --reference-meta <construction_seq2_reference.json> \
    --rko-param configs/mid360_robot/rko_lio_mid360_low_voxel_no_deskew.yaml \
    --lidarslam-param lidarslam/param/lidarslam.yaml \
    --output-dir <work>/source_run --offline-timeout-secs 5400
```

The recorder refuses overwrite, flushes MCAP on exit, and requires non-empty
`/rko_lio/odometry` and `/rko_lio/frame` topics. When `--dense-raw-tum` is
supplied to the candidate runner, its timestamp span is used as the runtime
denominator; backend bags use processing time, not original sensor time.
Compare the three dataset pairs:

```bash
python3 scripts/evaluate_slam_candidate_regression.py \
  --baseline <mid360-off>/cross_repo_benchmark.json \
  --baseline <hilti-off>/cross_repo_benchmark.json \
  --baseline <construction-seq2-off>/cross_repo_benchmark.json \
  --candidate <mid360-on>/cross_repo_benchmark.json \
  --candidate <hilti-on>/cross_repo_benchmark.json \
  --candidate <construction-seq2-on>/cross_repo_benchmark.json \
  --output output/phase7/candidate_regression.json --require-pass
```

Promotion requires complete reports, matching inputs, bounded runtime and map
quality, and independently improved MID-360 and Construction Seq2 trajectories.

When aggregate ATE hides which surveyed positions changed, generate a
checkpoint-level JSON and Markdown report:

```bash
python3 scripts/analyze_sparse_checkpoint_errors.py \
  --reference-tum <surveyed-positions.tum> \
  --reference-csv <checkpoint-labels.csv> \
  --estimate raw=<raw.tum> --estimate baseline=<off-dense.tum> \
  --estimate candidate=<on-dense.tum> --baseline-label baseline \
  --output <suite>/checkpoint_errors.json
```

Each trajectory is independently SE(3)-aligned without scale before per-point
errors are compared, matching the position-only public-suite ATE semantics.

That wrapper:

- uses the bundled NTU VIRAL `rosbag2`
- selects the validated `lidarslam/param/lidarslam_ntu_viral.yaml` graph profile
- uses the official-calibration `rko_lio_ntu_viral.yaml` frontend profile with
  the bounded tnp_01 voxel/gravity tuning and offline output backpressure
- runs `RKO-LIO + graph_based_slam`
- waits for graph ingestion to become quiescent before the final map save
- saves raw and corrected trajectories
- computes APE against the Leica prism reference
- verifies the Autoware map bundle when present
- writes `metrics.json` for the reporting pipeline

### Cross-repository suite (Localization Zoo)

`public_suite_v1.yaml` connects Localization Zoo trajectories to trajectory,
geometry, real-RGB, runtime, and memory gates:

```bash
python3 scripts/run_cross_repo_slam_benchmark.py \
  --localization-zoo ../loc_zoo_ws/localization_zoo \
  --dataset <profile> --gt-tum <gt.tum> --raw-tum <raw.tum> \
  --corrected-tum <graph.tum> --runtime-report <runtime.json> \
  --out-dir <benchmark-dir>
```

Candidate promotion compares frozen OFF/ON manifests from MID-360, the HILTI
position-only holdout, and RTK-SLAM Construction Seq2 surveyed checkpoints.
The gate never invents rotational RPE for position-only references. The
initial result is recorded in the
[Phase 7 regression note](research/phase7-plane-revisit-regression-2026-07.md);
the second-positive rejection is in the
[Phase 8 RTK-SLAM note](research/phase8-rtkslam-plane-revisit-2026-07.md).

## KITTI / LiDAR-Only Evaluation

The public default benchmark remains `RKO-LIO + graph_based_slam`. For KITTI
Odometry, use the separate LiDAR-only path because the Velodyne dataset does
not provide IMU messages.

```bash
bash scripts/download_kitti_odometry.sh --velodyne
export KITTI_ODOMETRY_ROOT="$PWD/datasets/KITTI_odometry"
bash scripts/run_kitti_odometry_benchmark.sh --sequence 00 --small-gicp --force-prepare
```

For frontend tuning, run the sweep wrapper:

```bash
bash scripts/sweep_kitti_small_gicp.sh \
  --dataset "$KITTI_ODOMETRY_ROOT" \
  --sequences "00 05 07"
```

The LO and `small_gicp` wrappers generate a rosbag2 QoS override so PointCloud2
playback uses `best_effort`, matching the frontend sensor-data subscriptions.

## Optional 3D-BBS Verification

`graph_based_slam` can build MIT-licensed 3D-BBS support from
`Thirdparty/3d_bbs`. This is an optional verifier for Scan Context loop
candidates, not part of the default public benchmark path.

Build behavior:

- enabled at build time when `GRAPH_BASED_SLAM_ENABLE_3D_BBS=ON` and the vendor
  headers are present
- disabled at runtime unless `use_3d_bbs_for_scan_context: true` is set
- force-disabled with
  `colcon build --symlink-install --cmake-args -DGRAPH_BASED_SLAM_ENABLE_3D_BBS=OFF`

MID-360 wrapper example (research track, `report_only_until: v0.4` in
`scripts/release_profiles.yaml`):

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh \
  --use-3d-bbs-for-scan-context true
```

Typical outputs are written under:

- `output/bench_rko_lio_ntu_viral_<name>/traj_raw_prism.tum`
- `output/bench_rko_lio_ntu_viral_<name>/traj_corrected_sparse.tum`
- `output/bench_rko_lio_ntu_viral_<name>/traj_corrected_prism.tum`
- `output/bench_rko_lio_ntu_viral_<name>/ape_raw_vs_gt.txt`
- `output/bench_rko_lio_ntu_viral_<name>/ape_corrected_vs_gt.txt`
- `output/bench_rko_lio_ntu_viral_<name>/metrics.json`

`traj_corrected_sparse.tum` preserves the optimized graph-node poses emitted
by `/modified_path`. The canonical `traj_corrected.tum` and
`traj_corrected_prism.tum` propagate those corrections onto every raw pose, so
the corrected APE and `metrics.json` describe a full-rate trajectory rather
than sparse nearest-neighbour samples.

## Loop Cloud-Overlap Gate

After registration, the backend can require a fraction of aligned source
points to have target-cloud support. `loop_min_overlap_ratio: 0.0` keeps the
gate disabled for backward compatibility; `loop_overlap_max_distance_m`
defines the nearest-neighbor support radius. The cheap fitness and correction
gates run first, so rejected registrations do not pay the KD-tree cost.

Construction Seq2 validated the following dataset-specific candidate:

```yaml
loop_min_overlap_ratio: 0.76
loop_overlap_max_distance_m: 0.5
```

The ratio rejected the harmful `57 -> 123` revisit and its adjacent
substitutes while retaining five beneficial loop edges. Do not promote this
threshold to a general default until it passes the other release datasets.

For a cross-sensor candidate, the source-overlap threshold can be explicitly
relaxed only when registration applies a large translation correction:

```yaml
loop_min_overlap_ratio: 0.76
loop_min_overlap_ratio_large_correction: 0.70
loop_overlap_large_correction_translation_m: 1.0
loop_overlap_max_distance_m: 0.5
```

The effective threshold is 0.76 below 1.0 m correction and 0.70 at or above
it. Leaving either large-correction parameter at zero disables the override.
The candidate preserved the established MID-360 loop, rejected the HILTI
exp04 false loop, retained Construction Seq2's five verified edges, and
preserved KITTI 00's `28 -> 176` loop (source overlap 0.864002) with
byte-identical edge and trajectory artifacts. The generic YAML default remains
disabled while broader release validation is pending. Reverse and harmonic
overlap are emitted in debug logs for diagnosis, but are not acceptance gates
because target aggregation extent biases them.

Accepted candidates and debug attempts also report `support_rmse_m` and
`support_p90_m`. These are nearest-neighbour distances for source points that
fall within `loop_overlap_max_distance_m`; they reuse the overlap KD-tree and
do not launch another search. Treat them as diagnostics, not gates: repeated
geometry can produce low support residuals at the wrong longitudinal offset.
The p90 calculation uses linear-time selection rather than sorting all
supported points.

HILTI exp01/exp07 can be frozen and compared end to end with one command. Raw
bags and generated backend MCAPs stay on the external SSD by default:

```bash
bash scripts/run_hilti_overlap_crossval.sh --sequence all --runs 2
```

Use `--dry-run` to inspect every command, `--record-only` to stop after input
capture, or `--offline-only --resume` to reuse an existing capture. Each
sequence writes `comparison.json` and `comparison.md` beside `gate_off/` and
`gate_adaptive/`. The capture stage generates a parameter snapshot that keeps
submap publication active but disables expensive live loop registration; both
offline variants then consume the exact same odometry/cloud pairs.

## Summaries And HTML Report

To summarize all collected runs:

```bash
python3 scripts/benchmark_summary.py \
  --root output \
  --write-md output/benchmark_summary.md \
  --write-csv output/benchmark_summary.csv
```

To generate the static HTML report:

```bash
python3 scripts/generate_html_report.py \
  --root output \
  --out output/latest_report.html
```

To generate a short public-beta readiness report from the current local
artifacts:

```bash
python3 scripts/generate_v2_beta_readiness_report.py
```

By default this writes:

- `output/v2_beta_readiness_<YYYYMMDD>.md`

To generate a short public-facing map-authoring positioning report from the
tracked benchmark, GNSS, dynamic-filter, and classic-path artifacts:

```bash
python3 scripts/generate_map_authoring_report.py \
  --out output/map_authoring_report_$(date +%Y%m%d).md \
  --write-json output/map_authoring_report_$(date +%Y%m%d).json
```

To stage a reusable submission-style bundle from an existing run directory:

```bash
bash scripts/create_map_authoring_submission_bundle.sh \
  output/bench_rko_lio_ntu_viral_fresh_20260324 \
  output/submission_bundle_ntu_viral_fresh \
  --report output/map_authoring_report_$(date +%Y%m%d).md \
  --verify-map
```

That bundle standardizes:

- `pointcloud_map/`
- `map_projector_info.yaml`
- `metrics.json` when present
- trajectories and key logs when present
- focused reports under `reports/`, with sibling `json/svg` copied automatically when present
- `map_qa_summary.md`
- `manifest.json`

To generate a separate stress-validation report that distinguishes the current
default path from older long-loop and hard-dataset evidence:

```bash
python3 scripts/generate_stress_validation_report.py
```

By default this writes:

- `output/stress_validation_report_<YYYYMMDD>.md`

To summarize dynamic-object-filter behavior across the tracked Leo Drive
save-time benchmarks:

```bash
python3 scripts/generate_dynamic_object_filter_validation_report.py \
  --out output/dynamic_object_filter_validation_report_$(date +%Y%m%d).md \
  --write-json output/dynamic_object_filter_validation_report_$(date +%Y%m%d).json \
  --write-svg output/dynamic_object_filter_validation_report_$(date +%Y%m%d).svg
```

The default report compares the tracked `bag1` and `bag6` dynamic-filter
benchmarks, so point reduction and voxel-removal behavior can be discussed as
cross-dataset evidence rather than a single-case anecdote. It also reports
coarse tile-footprint preservation via shared metadata tiles, tile jaccard,
and filtered-tile overlap ratio.

To promote an already-recorded aligned cross-validation run such as the MID360
long-loop check into `metrics.json` so it appears in `benchmark_summary.md` and
`latest_report.html`:

```bash
python3 scripts/write_aligned_trajectory_metrics.py \
  --out-dir output/bench_rko_lio_mid360_v3 \
  --bag demo_data/glim_mid360/rosbag2_2024_04_16-14_17_01 \
  --reference-tum output/glim_mid360_reference.tum \
  --corrected-tum output/bench_rko_lio_mid360_v3/traj_corrected.tum \
  --raw-tum output/bench_rko_lio_mid360_v3/traj_raw.tum \
  --graph-log output/bench_rko_lio_mid360_v3/graph_slam.log \
  --parameter-file output/bench_rko_lio_mid360_v3/graph_params.effective.yaml \
  --benchmark-harness scripts/run_rko_lio_mid360_crossval_benchmark.sh \
  --runtime-artifact rko_lio_offline_node=install/rko_lio/lib/rko_lio/offline_node \
  --runtime-artifact graph_based_slam_node=install/graph_based_slam/lib/graph_based_slam/graph_based_slam_node \
  --reference-source glim_mid360_reference \
  --reference-kind cross_validation \
  --reference-label GLIM \
  --points-topic /livox/lidar \
  --points-frame livox_frame \
  --robot-frame livox_frame
```

The summary/report pipeline now exposes the reference kind, so `ground_truth`
and `cross_validation` runs do not appear as if they were the same type of APE.
The writer also hashes rosbag2 metadata and storage, the reference trajectory,
effective parameters, benchmark harness, metrics writer, and every declared
runtime artifact. It records the source commit and dirty state. Every shipped
release profile requires this complete provenance from a clean revision;
legacy, incomplete, or dirty evidence evaluates as `NO_DATA` and therefore
cannot satisfy a blocking release profile. “Clean” includes untracked files,
because an untracked source or build input can otherwise alter a binary without
changing the recorded commit. The release-profile table's `evidence` column
distinguishes “no matching run” from candidate runs rejected for incomplete
provenance or a dirty revision.

For a public-facing snapshot built on top of these artifacts, see
`docs/comparison.md` and `docs/releases/v0.2.2.md`.

To rerun the current MID360 cross-validation benchmark end-to-end:

```bash
bash scripts/run_rko_lio_mid360_crossval_benchmark.sh
```

This MID360 wrapper defaults to a tuned `RKO-LIO + graph_based_slam` profile
with `voxel_size=0.5`, `max_range=80.0`, `search_submap_num=5`,
`loop_edge_dedup_index_window=20`, and `loop_edge_info_weight=200`.

To benchmark the real open-data Leo Drive `driving_30_kmh` bag with mixed
RTK/non-RTK GNSS quality:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

That wrapper writes a local `Applanix_GSOF49` reference trajectory,
`traj_raw.tum`, `traj_corrected.tum`, and `metrics.json` so the run appears in
`benchmark_summary.md` and `latest_report.html`.

For rosbag2 `compression_mode: FILE` inputs, the wrapper plays a private view
inside the output directory. ROS 2 may decompress the storage file while it
plays, but that temporary database is removed with the private view when the
run exits; the source bag directory remains unchanged.

When the main bag already contains native `sensor_msgs/msg/NavSatFix` or
`sensor_msgs/msg/Imu`, the same wrapper now prefers those real topics before it
falls back to Applanix sidecar generation.

Current Leo Drive packet-path evidence is:

- `driving_30_kmh`, GNSS-only classic path: `APE RMSE 195.285 m`
- `bag1_front`, default GNSS-only path: `APE RMSE 0.139 m`
- `bag1_front`, native `/sensing/imu/imu_data`: `APE RMSE 0.251 m`
- `bag6_front`, `no_imu`: `APE RMSE 0.422 m`
- `bag6_front`, native `/sensing/imu/imu_data`: `APE RMSE 0.365 m`

The important result is that packet IMU deskew is usable on the native
`all-sensors` bags, but only when the benchmark is replayed conservatively.
The benchmark now defaults to `rate=1.0` for every configuration and
deterministically prefers a `/front/` packet topic when a bag contains several
Velodyne streams. The exact-revision
[bag1 evidence](evidence/leo-drive-packet-benchmark-2026-07-30.md) records the
input, software, and output hashes. The earlier `20m+` regressions were
runtime-sensitivity and sensor-selection artifacts, not proof that the deskew
math itself was fundamentally broken. To reproduce the current experimental
IMU result on the driving bag:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/driving_30_kmh_2022_06_10-15_47_42_compressed \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --use-imu true \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --robot-frame-id base_link \
  --imu-frame-id base_link \
  --verify-map
```

To compare the same packet path on `all-sensors-bag6` while isolating IMU
deskew from GNSS:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --packet-topic /sensing/lidar/front/velodyne_packets \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --use-gnss false \
  --verify-map

bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --packet-topic /sensing/lidar/front/velodyne_packets \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --use-gnss false \
  --use-imu true \
  --verify-map

bash scripts/run_open_data_applanix_velodyne_gnss_benchmark.sh \
  --bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --packet-topic /sensing/lidar/left/velodyne_packets \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --tf-bag demo_data/autoware_leo_drive_isuzu/all-sensors-bag6_compressed \
  --use-gnss false \
  --use-imu true \
  --imu-rotation-use-orientation false \
  --verify-map
```

To summarize the current cross-dataset odom-prior validation evidence after the
classic-path runs have been recorded:

```bash
python3 scripts/generate_odom_prior_validation_report.py \
  --out output/odom_prior_validation_report_$(date +%Y%m%d).md \
  --write-json output/odom_prior_validation_report_$(date +%Y%m%d).json \
  --write-svg output/odom_prior_validation_report_$(date +%Y%m%d).svg
```

This report intentionally compares `driving_30_kmh` and `bag6_front` side by
side, because the current velocity-based prior helps the fallback classic path
on one dataset and hurts or helps differently on another.

To validate packet IMU deskew as a repeatable matrix on real open data, use:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_packet_imu_deskew_validation_matrix.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg
```

That matrix compares `no_imu` and native-IMU runs for the default `bag1_front`
and `bag6_front` cases at `rate=1.0` and emits:

- `packet_imu_deskew_validation.md`
- `packet_imu_deskew_validation.json`

The report is generated by `generate_packet_imu_deskew_validation_report.py`
and fails if any case violates the configured path-coverage, RMSE-regression,
or matched-pose thresholds.

The same bag also exposes native `/gnss/fix`. The backend now falls back to
receive time when the NavSatFix header stamp is far from ROS time
(`gnss_header_stamp_max_skew_sec`, default `30 s`), which lets the graph attach
GNSS edges on `all-sensors-bag6`. In practice that native `/gnss/fix` still
disagrees with the `GSOF49` reference enough to degrade the cross-validation
APE, so `all-sensors-bag6` is useful for georeferenced smoke tests but not a
clean GNSS benchmark source.

To compare place-recognition behavior on MID360, rerun the same benchmark with
and without an optional descriptor family and then render the short report:

```bash
bash scripts/run_place_recognition_benchmark.sh
```

To compare the current experimental BEV-assisted distance rerank instead:

```bash
bash scripts/run_place_recognition_benchmark.sh --candidate-mode bev_rerank
```

The report shows:

- runtime `use_scan_context`
- accepted/attempted loop counts
- accepted loop source counts
- observed `ScanContext loop candidate` count
- observed `BEV rerank hint` count
- observed `SOLiD rerank candidate` count
- `APE RMSE` delta between the two runs
- optional JSON summary via `--write-json`
- optional SVG summary via `--write-svg`

The report is generated by `generate_place_recognition_report.py`.

Current checked-in evidence is:

- fair current-code baseline rerun:
  `output/bench_rko_lio_mid360_current_default_rerun_20260326/metrics.json`
  (`APE RMSE 4.096 m`)
- current best checked-in Scan Context candidate with DB/index fix,
  aggregated descriptor/registration cloud, and `scan_context_threshold=0.55`:
  `output/bench_rko_lio_mid360_sc055_yawguess_scagg_screg_20260326/metrics.json`
  (`APE RMSE 3.568 m`)
- current experimental BEV-assisted distance rerank:
  `output/bench_rko_lio_mid360_20260326_202840/metrics.json`
  (`APE RMSE 3.607 m`)
- best observed BEV-assisted distance rerank:
  `output/bench_rko_lio_mid360_20260326_202119/metrics.json`
  (`APE RMSE 3.533 m`)
- short comparison report:
  `output/place_recognition_report_20260326.md`

That candidate currently beats both the fair rerun baseline and the published
`3.641 m` default artifact, but the accepted loop still comes from the
distance-based path. Treat `use_scan_context=true` as an opt-in tuning path
rather than the repository default.

The BEV path is now more useful as a sensor-agnostic distance-candidate rerank
than as a standalone loop source. It has shown better-than-baseline runs, but
its rerun variance is still too large for a default-on setting.

To summarize the current stop/go decisions for place recognition and the
classic fallback path in one short report:

```bash
python3 scripts/generate_exploration_closeout_report.py \
  --out output/exploration_closeout_report_$(date +%Y%m%d).md \
  --write-json output/exploration_closeout_report_$(date +%Y%m%d).json
```

A local snapshot can be written to:

- `output/exploration_closeout_report_20260327.md`

That report fixes the current repository position in one place:

- public default place recognition remains the distance-based path
- `Scan Context` stays opt-in
- `BEV-assisted rerank` stays experimental
- `SOLiD` stays experimental/off by default
- the classic path remains a fallback workflow rather than the main public path

## Dynamic Object Filter Benchmark

The dynamic-object filter is save-time only. It does not change live odometry
or loop closure, so the right comparison is the saved map output with the same
bag and the same backend settings.

Run the paired comparison on the open-data bag6 smoke path:

```bash
bash scripts/run_dynamic_object_filter_benchmark.sh
```

That wrapper:

- runs `run_open_data_gnss_smoke.sh` twice on the same bag
- saves `no_filter/` and `dynamic_filter/` outputs under one root
- renders `dynamic_object_filter_report.md`,
  `dynamic_object_filter_report.json`, and `dynamic_object_filter_report.svg`

The report is generated by `generate_dynamic_object_filter_report.py` and
tracks:

- Autoware map verify result for both runs
- projector type
- saved grid cell count
- metadata tile count
- total saved point count
- filter candidate/kept/removed voxel counts
- saved-point reduction ratio

The current checked-in evidence is:

- baseline smoke:
  `output/open_data_gnss_smoke_bag6_autodetect_throttled_20260325`
- filtered smoke:
  `output/open_data_gnss_smoke_bag6_dynamic_filter_20260326`
- benchmark report bundle:
  `output/dynamic_object_filter_benchmark_bag6_20260326`

In that checked run, the saved map went from `138732` to `87861` points while
keeping `verify_autoware_map.py` at `PASS`.

## Leo Drive Classic Path Benchmark

To compare the current classic `scanmatcher + graph_based_slam` path on the
mixed-quality Leo Drive `driving_30_kmh` open-data bag, run:

```bash
git clone --depth=1 https://github.com/autowarefoundation/applanix.git /tmp/applanix
bash scripts/run_open_data_classic_path_benchmark_suite.sh \
  --applanix-msg-dir /tmp/applanix/applanix_msgs/msg \
  --verify-map
```

This wrapper emits:

- `classic_path_report.md`
- `classic_path_report.json`
- `classic_path_report.svg`

The report is generated by `generate_classic_path_report.py`.

The checked-in snapshot is:

- `output/classic_path_report_20260327.md`

Current evidence is:

- `no GNSS`: `APE RMSE 313.695 m`
- `GNSS only`: `APE RMSE 195.285 m`
- `GNSS + velocity-based planar odom prior`: `APE RMSE 175.732 m`
- `GNSS + IMU`: `APE RMSE 271.144 m`

So the classic path still needs work, but the direction is clearer now:
backend GNSS helps substantially, and a velocity-based planar odom prior helps
further on `driving_30_kmh`, while the current packet IMU path is still not a
default recommendation.

## Release/Readiness Gate

To run the local readiness gate in one command:

```bash
bash scripts/run_release_readiness_checks.sh --fail-on-profiles
```

That wrapper can run:

- default build and package tests
- benchmark summary generation
- HTML report generation
- optional public MID-360 segment-reset completion gate
- standalone public MID-360 continuous kidnap-relocalization gate
- optional Autoware dogfood

The release command uses the per-dataset profile thresholds. With
`--fail-on-profiles`, it exits non-zero when a blocking profile exceeds its
threshold or has no matching run. A passing synthetic fixture therefore
checks reporting mechanics but cannot count as release evidence.

The wrapper resolves the current repository `HEAD` and binds every blocking
profile to that exact 40-character commit. Clean benchmark evidence from an
older revision is reported as a candidate-commit mismatch and evaluates as
`NO_DATA`; it cannot authorize the current release candidate. Profiles marked
`report_only_until` remain useful as historical comparisons and are not
commit-bound. When invoking `benchmark_summary.py` directly as a hard gate,
pass both `--fail-on-profiles` and
`--required-git-commit "$(git rev-parse HEAD)"`.

For a one-off uniform threshold check, `--ape-threshold <metres>` is also
hard:

- it exits non-zero if `--benchmark-root` contains no `metrics.json` evidence
- it exits non-zero if any selected run is missing APE
- it exits non-zero if any selected run exceeds the threshold
- by default `run_release_readiness_checks.sh` applies that hard gate only to
  `ground_truth` runs; `cross_validation` runs stay visible in reports without
  blocking release

`--fail-on-profiles` requires an active, existing release-profile YAML and an
exact candidate commit (automatically supplied by the wrapper).
Profiles marked `report_only_until` remain non-blocking even when their data
is absent. Neither hard benchmark gate can be combined with
`--skip-benchmark-summary`. Without `--ape-threshold` or
`--fail-on-profiles`, an empty benchmark root remains report-only and the
wrapper records that benchmark reporting was skipped.

With `--fail-on-profiles`, an empty benchmark root remains a hard failure but
is no longer a dead end. The output directory retains a Markdown/CSV summary
that marks every profile `NO_DATA`, distinguishes the five blocking profiles
from report-only canaries, and prints the tracked dataset acquisition or rerun
instruction for each blocker. The process still exits 2 and cannot authorize a
release without exact-commit evidence.

For the public MID-360 segment-reset completion evidence, add:

```bash
bash scripts/run_release_readiness_checks.sh \
  --skip-default-ci \
  --skip-benchmark-summary \
  --public-mid360-completion
```

That hook runs `scripts/run_mid360_robot_public_completion_gate.py` as a hard
gate and writes its JSON/Markdown under the release-readiness output directory.

### Paired map-quality non-regression

When a candidate map has a like-for-like baseline report, the release wrapper
can add a fail-closed paired check without changing the existing absolute
profile gate:

```bash
bash scripts/run_release_readiness_checks.sh \
  --skip-default-ci \
  --skip-benchmark-summary \
  --map-quality-pcd /path/to/candidate/map_refined.pcd@configs/map_quality_profiles/indoor_construction.yaml \
  --map-quality-baseline-report /path/to/baseline/map_quality_report.yaml \
  --map-quality-max-regression-percent 2.0
```

`run_map_quality_check.sh` evaluates the candidate's run-1 report against the
named baseline with `scripts/check_map_quality_regression.py`. The five paired
metrics are plane thickness mean/p95 (lower is better), planar coverage and
mean-map-entropy valid fraction (higher is better), and entropy value (higher,
or less negative, is worse). Reports must have finite values, meaningful
planes, and identical extraction settings; a missing field, zero baseline
denominator, or mismatch is invalid and fails closed. The paired budget never
relaxes `indoor_construction.yaml` or any other absolute profile. The command
writes `paired_regression_verdict.yaml` and `.json` beside the map-quality
summary, plus human-readable rows in `paired_regression_verdict.txt`.

The HILTI exp04 current-vs-old map reports used in the M4c diagnostic pass this
2% paired check. Both reports independently violate the indoor profile's
`mme_valid_fraction_min` threshold, so that absolute-profile result remains a
separate applicability issue rather than being hidden by the paired pass.

M4c also closed the fixed backend regression receipts for two HILTI inputs:

- exp04: `/tmp/lidarslam-m4c-hilti-exp04-gate.pzufEB`, three-run artifact
  identity and old optimized trajectory exact, wall `2.71/2.90/2.73 s`,
  maximum RTF `0.010347643`, peak RSS `272.167968750 MiB`, and wall CV
  `3.066357758%`.
- exp07: `/tmp/lidarslam-m4c-hilti-exp07-gate.zCELMb`, three-run artifact
  identity and old optimized trajectory exact, historical interpolated APE
  `0.6186851452574647 m` from 5/6 sparse GT points, wall `1.90/1.90/1.97 s`,
  maximum RTF `0.050166829`, peak RSS `201.136718750 MiB`, and wall CV
  `1.715683698%`.

Both receipts pass the fixed RTF/RSS/CV gates and record the canonical host NDT
`backend_loop` receipt with `target_cell_cache_capacity=3`. These are named
input compatibility/resource gates, not official dense-GT or SOTA comparisons;
the M5 fresh-holdout and competitor protocol remains pending. The old/current
indoor absolute-profile violation is reported separately and is not relaxed.

For the continuous RKO-LIO kidnap-relocalization evidence, run:

```bash
python3 scripts/run_mid360_robot_public_continuous_relocalization_gate.py
```

That gate checks the merged public `outdoor_kidnap_a+b` run for full-duration
RKO output, at least one global relocalization event, loop-alignment PASS,
public loop endpoint closure at the GT start/end stamps, Autoware map verify
PASS, offline completion, and tracked kidnap recovery config matching the run
config. The endpoint closure check prevents a local revisit from being counted
as continuous kidnap relocalization.

## CI Coverage

CI exercises the reporting path in two ways:

- a passing synthetic benchmark fixture must generate summary and HTML report
- a failing synthetic benchmark fixture must trip the threshold gate with
  exit code `2`

The fixture generator is:

```bash
python3 scripts/generate_sample_benchmark_metrics.py \
  --root /tmp/ci_fixture \
  --profile passing
```

Use `--profile failing` to create a negative-path fixture.

### M6a10 fixed10 ours no-map replay boundary

The first fixed10-v1 ours replay is an immutable `FAIL_CLOSED` functional
attempt: consumer and online phase evidence passed, but 18 map artifacts were
observed under `--skip-map-save`. The independent receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v1/evidence/independent_verification_final.json`
(SHA-256
`ce0211f2e55e09ed2e8cb9f0692daf2dc5ebdf0f6707942292734bd542f35a0e`), with
output tree SHA-256
`508df8568e9db1d51e8f167a12349cec33d04f61564b6154bf86244ae5f52624`.
No retry was made, and no GT/scorer data was opened.

The failure was diagnosed from the graph source and launch parameters:
accepted loop edges call `doPoseAdjustment(..., use_save_map_in_loop_)`, whose
normal default is true; its save branch writes the grid-divided map, bundle,
degeneracy report, and pose-graph output. The runner's old skip branch only
omitted the `/map_save` service request, so it could not promise map-free
output. Fixed10-v2 is preregistered, not executed: the benchmark-only wrapper
marker sets `use_save_map_in_loop=false` and an empty pose-graph save path via
the source launch, and the runner verifies the forbidden map paths after the
launch exits. `dump_results:=true` remains so the full offline trajectory and
consumer evidence are preserved. The v2 output root and exact wrapper/runner/
launch hashes are pinned in both machine-readable receipts. The v2 image is
`m6a10-v2a-fixed10-v2-lidarslam-ours:jazzy` at immutable digest
`sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69`.
Its installed launch path and OCI label carry SHA
`d45545717f90f6877b5f281fc5623df04b824f7a236d2fe73b91c2dd3714371c`; the
recipe SHA is `99daea2172ae64f048557a9b069f48cd4b462de1581efd0b12457618dd360330`
and its build entrypoint SHA is
`2249b168cebaa640c657d095743a11d82b356123ad433806a443745a1f694b96`.
It copies only this explicitly hashed overlay, never a dirty host tree.
Its status remains `preregistered_not_executed` until a separately authorized
replay.

### FAST-LIVO2 M6a10-v2c retry-v2 closure

The FAST-LIVO2 fixed10 retry-v2 was closed `FAIL_CLOSED` after its single
authorized launch attempt. The read-only quiescence receipt passed (CPU busy
`3.7616763443574857%`, load1/CPU `0.415`, and no forbidden processes), but the
host runner rejected the profile before starting Docker because the required
`safety.ground_truth_mount_exposed` field was absent. The attempt therefore
started no container and replayed no bag; this is a runner contract-validation
failure, not a performance result. Its immutable closure receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/fast_livo2_v2c_fixed10_v2/closure_receipt.json`
(SHA-256
`c76ce0a5d782336012f6c9ae897f9683792799ef92eba00a429fc024958ffdf0`). The
host `time-v.txt` and the passing quiescence receipt remain alongside it, with
no `run_01`, container, partial artifact, map artifact, GT access, scorer, or
retry. The earlier compiler/load failure is retained as an immutable
predecessor; no FAST-LIVO2 accuracy or performance claim follows.

The fixed10-v3 successor made one new quiescence attempt after the explicit
`ground_truth_mount_exposed: false` schema repair. It failed closed before the
runner because an unrelated external build was active: CPU busy was
`98.275%`, load1/CPU was `0.69375`, and seven C++ compiler processes plus one
`rustc` were observed. Its immutable closure is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/fast_livo2_v2c_fixed10_v3/closure_receipt.json`
(SHA-256
`eed5aa5072048ea77007a2f38a1440ca60af743d389a561b9aa89b5503bbc399`), with
quiescence receipt SHA
`d27cefb4923848eed622829a9c802adb378f4e2d2f1078884593e246a8702cf7`. No
container, bag replay, GT access, scorer, or retry was started.

### FAST-LIVO2 fixed10-v4 correction and v5 observability

The fixed10-v4 FAST attempt is `FAIL_CLOSED`, not a result. Its immutable
closure SHA is `1292cbadf0eff2575f2df3014a73e54822d8d57f6703cba5832e3db91065bfd1`.
The host stopped its own container after bounded no-progress supervision; the
container was force-removed after the 30-second stop grace period. OOM was not
observed. Because v4 had no progress checkpoint, it cannot establish that a
first callback ACK was absent. The correction SHA
`861585b554cee8240bd4cae811dbec46c7256b83ba40a30c4e28a91ca690c93f` records
this limitation explicitly. The v4 attempt did not access GT or invoke a
scorer.

The v5 instrumentation is preregistered but not executed. It adds atomic,
non-authoritative feeder progress (first ACK and every 100 records),
line-buffered diagnostics, a stable Docker name/cidfile, low-frequency Docker
stats, and host-side reconnectable lifecycle/inspect snapshots. It does not
change the input,
algorithm, mount graph, or fairness contract. The v5 image must be rebuilt to
bind the new feeder label before any replay; no v5 quiescence or replay has
started. Planned output root:
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/fast_livo2_v2c_fixed10_v5`.
The preregistered source hashes are feeder
`bde0631d29dbb18575fe0fd2ce4bc339e738d14e1b47a1ccc8a77aa348f5622d`, host
runner `54db99e7f9da588baff33973490efb5474d86ca9888b1dae9578621810c38d48`,
and recipe
`a42686414c8400d9dba91cd7103840a703d77514cb33475a3c9a035fb100de1e`.
The old v4 `*map*` detector remains part of its immutable historical record;
v5 preregisters an explicit map-artifact denylist instead (`*.pcd`,
`map.pcd`, `map_bundle.yaml`, `map_projector_info.yaml`,
`degeneracy_report.yaml`, `pose_graph.g2o`, `trajectory_optimized.tum`,
`loop_edges.csv`, `pointcloud_map`, `*.bag`, `*.db3`, and `*.part`). The required
`mapper.log` and `mapper_ready.txt` files are diagnostic-only and explicitly
excluded from the map-artifact denylist.
The synchronized canonical profile identity is
`3904791e1dff0b6841cf40133c7c96327254faf9797dc1827badf6f0d9477d4c`; the
execution-selection file SHA is
`ecca86bca67704ccd653da450344de3fd6ba52ad23baeddfaae9ae3569e4e7f2`.

### Dataset source closure and partition provenance

The claim gate also audits every dataset/sequence declared under the profile's
`bringup`, `development`, `regression_only`, historical, and fresh partitions.
The metadata contract is
`evidence_gate_v2.dataset_source_closure` and its schema is
`configs/slam_benchmark_profiles/competitive_dataset_source_closure_v1.schema.json`;
the offline auditor is
`scripts/check_competitive_dataset_source_closure.py`.  Selection preflight
and the schema-v2 claim gate both invoke it.

Each descriptor must bind an official primary project/version and immutable
download references, license/terms and citation, sensor topics/formats/time
basis, calibration identity/convention, ground-truth method/frame/time basis,
input/calibration/GT hashes and byte counts, sequence duration/message counts,
and support from every required runner.  Dataset IDs, sequence IDs, input
identities, and GT identities must be disjoint across partitions; shared
calibration identities are not treated as dataset overlap.  A sequence is not
counted as a second dataset family: the claim policy requires at least two
distinct GT families and a fresh partition.

The auditor is metadata-only and never opens GT or data files.  Recorded
hashes are explicitly `RECORDED_ONLY` until a verified evidence-volume mount
identity and revalidation receipt are supplied; recorded bytes are never
reported as revalidated bytes.  Missing official version/license/terms,
calibration or GT identity, moving URLs, unsupported runners, and frame/time
basis mismatches fail closed.  The checked-in profile currently remains
`NOT_READY`: Hilti is recorded-only, while NTU VIRAL is a second
precommitted-but-unacquired family with no PASS sequence.  The auditor reports
declared families separately from `passed_families`, so a preregistered NTU
family cannot satisfy the two-family claim requirement.

### Immutable OSS rival source closure

Claim eligibility requires the pinned `glim` and `fast_livo2` rivals to carry
an offline-verifiable source-closure record.  The record is
`evidence_gate_v2.rival_source_closure` in
`configs/slam_benchmark_profiles/competitive_slam_v1.yaml`; its schema is
`configs/slam_benchmark_profiles/competitive_rival_source_closure_v1.schema.json`
and the production audit is
`scripts/check_competitive_rival_source_closure.py`.

Every direct and recipe-cloned upstream source must use an official HTTPS
repository, an exact 40-hex commit, a commit-addressed archive URL with the
downloaded archive SHA-256, a deterministic source-tree SHA-256, an explicit
recursive submodule list, license identity/file hashes, and an official commit
citation.  License evidence is component-scoped: a package metadata
declaration is recorded separately from an upstream license text artifact and
cannot substitute for one.  Ordered local patch hashes, Dockerfile/build-
script/runner/wrapper hashes, configs, the immutable base-image digest, and
expected build options are checked against the current repository and the
versioned selection sidecar
`configs/slam_benchmark_profiles/competitive_execution_selection_2026-08-r2.yaml`.
Branches, `latest`, tag-only references, missing license or archive proof,
hardlinks/symlinks, external configs, recipe drift, and rivals omitted from all
declared tracks fail closed.

The current closure is
`competitive-rival-source-closure-2026-08-r2` (revision 2) and remains
`NOT_READY`.  The pinned FAST-LIVO2 root
`LICENSE` (GPL-2.0-only) and its conflicting `package.xml` BSD declaration are
now represented as separate component evidence, while each `rpg_vikit`,
Sophus, and the GLIM ROS 2 bridge component is explicitly
`NOT_READY_LEGAL_PROVENANCE` when the pinned upstream tree contains no
reopenable license text artifact.  The historical execution-selection receipt
is explicitly `SUPERSEDED_RECIPE_REVISION` and ineligible; it is not rewritten.
The legal policy permits source fetching for reproducibility only and blocks
source, binary, and image redistribution until provenance is resolved. These
are provenance blockers, not benchmark results or a claim.

## Recommended Artifacts To Publish

For a run based only on public, licensed input, first review every file for
credentials, private paths, host or user names, precise locations, and private
geometry. The safe publication set is:

- `metrics.json`
- `benchmark_summary.md`
- `benchmark_summary.csv`
- `latest_report.html`
- a tracked/public parameter preset plus a redacted list of changed arguments,
  not a complete custom parameter YAML
- `docs/comparison.md` when publishing the current positioning of the repo
- `docs/releases/v0.2.2.md` when publishing the current public beta scope
- `v2_beta_readiness_<YYYYMMDD>.md` when preparing a public beta snapshot
- `stress_validation_report_<YYYYMMDD>.md` when discussing long-loop or
  aggressive-motion evidence

For a private or custom bag, use the Benchmark report form and share only its
redacted metadata and key-metric fields. Do not publish the bag, map, trajectory,
APE/raw logs, raw sensor data, private-site images, local/output paths, or precise
coordinates. An optional `metrics.json` or public aggregate report is safe only
after review confirms that none of those values is present.

## Related Commands

- Autoware quickstart: `docs/autoware-quickstart.md`
- public Autoware entrypoint: `bash scripts/run_autoware_quickstart.sh`
- public comparison page: `docs/comparison.md`
- end-to-end dogfood: `bash scripts/run_rko_lio_graph_autoware_dogfood.sh --auto-exit-secs 20`

## Release-gate accuracy snapshot

Current numbers from the release-gate profiles (`scripts/release_profiles.yaml`).
Every release is blocked in CI by these per-dataset thresholds.

| Dataset | Sensor | Reference | APE RMSE | Gate (pass) |
| --- | --- | --- | --- | --- |
| NTU VIRAL `tnp_01` (outdoor, ~580 s) | Ouster OS1-16 + VN-100 | Leica prism ground truth | **0.95 m** (best 0.87) | ≤ 1.00 m |
| RTK-SLAM Construction Hall 2 (indoor, ~600 s) | Livox MID-360 | total-station checkpoints¹ | **0.154 m** (median 0.061) | ≤ 0.30 m |
| RTK-SLAM Construction Hall 1 (indoor, ~741 s) | Livox MID-360 | total-station checkpoints¹ | **0.403 m** (median 0.263) | ≤ 0.55 m |
| RTK-SLAM Stadtgarten 2 (outdoor park, ~876 s) | Livox MID-360 | total-station checkpoints¹ | **0.835 m** (median 0.327) | report-only² |
| RTK-SLAM Stadtgarten 1 (outdoor park, ~1 km loop) | Livox MID-360 | total-station checkpoints¹ | **1.666 m** (median 1.511) | report-only² |
| Newer College `math-hard` (~320 m loop) | Ouster OS0-128 | prism ground truth | reported separately | ≤ 0.10 m |

¹ Surveyed checkpoints from the public RTK-SLAM dataset (CC-BY 4.0), scored like
its published baselines (dense odometry trajectory).
² Outdoor profiles soak as report-only before graduating; the former GLIM
cross-validation gate is also report-only since v0.5. Methodology and
caveats: [Comparison](comparison.md).

Reproduce locally:
```bash
bash scripts/run_rko_lio_graph_benchmark.sh
bash scripts/run_release_readiness_checks.sh --fail-on-profiles
```
