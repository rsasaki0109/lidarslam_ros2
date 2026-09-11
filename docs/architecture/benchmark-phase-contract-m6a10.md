# M6a10 benchmark phase contract

## v2 followability and acknowledgement contract

M6a10-v1 remains immutable evidence (including its `FAIL_CLOSED` training
summary); v2 is an additive contract and does not reinterpret campaign4 or
v1 results. The preregistered v2 schema is
`m6a10-online-compute-v2` in `competitive_slam_v1.yaml`.

It has two explicit modes:

- `paced_1x`: primary success is followability. The implementation must report
  exact expected/received/processed consumer counts, zero drops and queue
  overflow, an observed EOF marker, zero backlog at drain, timestamp coverage
  within 250 ms, and maximum callback latency within 250 ms. Playback pacing
  must be independently verified at exactly 1.0x. Wall/online RTF is retained
  as a diagnostic and is not the primary gate.
- `unpaced_ack`: the same consumer acknowledgement proof is required, plus a
  synchronous acknowledgement/backpressure hook and an acknowledgement
  interval RTF of at most 1.0. Raising rosbag `RATE` is not an implementation
  of this mode; without the application hook the wrapper fails closed.

The counter source must name the processing callback/entry/exit boundary and
must set `publisher_count_used: false`. Publisher counts, bag metadata counts,
or inferred process-exit completion cannot satisfy v2. Missing fields remain
missing rather than becoming zero. The v2 shell bridge accepts an explicit
application-produced `M6A10_CONSUMER_EVIDENCE` JSON file and atomically folds
it into `phase_evidence.json`. The ours/RKO-LIO wrapper now wires this path to
the offline consumer hook, but it has not been replayed on the pinned training
bag; GLIM and FAST-LIVO2 still have no authoritative v2 consumer hook.

M6a10-v1 remains the compatibility contract for the immutable training
summary: its online-compute RTF gate is `<=1.0`, while the earlier campaign4
wall-time result remains diagnostic lineage. The new v2 contract is additive;
it does not reopen campaign4 or reinterpret the v1 summary. For v2,
`paced_1x` followability is primary and `unpaced_ack` throughput uses its
acknowledgement-interval RTF gate. Neither v2 mode is measured yet: the ours
hook is implementation-only evidence at this point, while GLIM/FAST still
lack the required application-owned consumer evidence.

## Measurement boundary

Each wrapper emits one atomic `phase_evidence.json`. Its monotonic event
sequence is:

```
startup -> input_start -> input_end -> required_drain_end
          -> postprocess -> map_save -> shutdown
```

Online compute is:

```
(required_drain_end - input_start) / sensor_duration
```

Container/runner startup, map save, postprocessing, and a fixed shutdown
grace are outside that interval. The complete phase document also records
wall time (diagnostic only), CPU user/system time, GNU-time filesystem
input/output operation counters (not byte counts), and each phase duration. A
missing boundary, non-monotonic clock,
non-finite value, nonzero exit, or missing resource report is invalid rather
than estimated.

Input completeness is fail-closed. A wrapper must provide either exact
processed/expected message counts or a trajectory timestamp coverage proof
whose end gap is at most 0.25 seconds, plus measured zero dropped messages and
zero queue-overflow reports. The shell bridge does not invent zero values when
the implementation has no counters, so such a run remains invalid until its
wrapper supplies those measurements. The shared validator is
`scripts/benchmark_phase_contract.py`; the shell bridge is
`scripts/container_phase_evidence.sh`. Updates use a staging file and atomic
rename, so partial JSON cannot be accepted as a PASS.

For v2, the trajectory-only alternative is insufficient: the application must
also provide the `consumer` object through `M6A10_CONSUMER_EVIDENCE`. Its
processing-boundary source, exact counts, EOF, empty drain backlog, latency,
and pacing/acknowledgement proof are validated independently. A wrapper may
retain trajectory coverage as a diagnostic cross-check, but it cannot promote
publisher counts or process exit into a consumer acknowledgement.

## Wrapper boundaries and ROS differences

- **ours/RKO-LIO (ROS 2):** startup is the existing RKO and graph
  initialization markers. The existing offline trajectory-end/quiescence
  check supplies the v1 timestamp coverage proof. `/map_save` and map output
  waits are explicitly after the online interval. For v2a, the
  `OfflineNode::run` dispatch records expected topic metadata counts, callback
  entry/return counts, callback latency, EOF from
  `BufferableBag::finished()`, and drain state in an atomic
  `consumer_evidence.json`. In `unpaced_ack`, `BufferableBag` uses a
  benchmark-only single-message buffer, so the next input is not supplied
  until the prior callback returns. Registration-queue depth remains a
  separate diagnostic and queue overflow/drop counters remain hard gates.
- **GLIM (ROS 2):** the wrapper records the bag-open marker and requires an
  explicit EOF marker before accepting an input end. Process exit alone is not
  treated as proof because GLIM may still dump/postprocess after input EOF.
  If the pinned GLIM log has no recognized EOF marker, phase evidence is
  invalid and the run remains a recorded failure.
- **FAST-LIVO2 (ROS 1):** `rosbag play` starts the input interval; the wrapper
  waits for odometry timestamps to reach the ROS 1 bag end before ending the
  required drain. The ROS master is loopback-only and the raw ROS 1 bag is not
  silently treated as a ROS 2 bag. Its canonical ROS semantic-equivalence
  receipt remains a separate input identity; ROS1 publisher/odometry counts do
  not satisfy the v2 consumer acknowledgement requirement.

The wrappers keep their existing process-tree RSS, cgroup/OOM, signal, and
GT-blind contracts. The driver requires `phase_evidence.json` in every
attempt and records `runtime.online_compute_rtf`; the legacy
`runtime.processing_realtime_factor`/wall measurement remains diagnostic
lineage. No replay or scoring result is implied by this implementation.

## Evidence status and limitations

The contract and runner parser are covered by synthetic tests for ordering,
RTF calculation, exact coverage, drop/overflow rejection, signal-like
nonzero exits, and atomic output. A GT-blind NTU Viral public/training replay
was then attempted using the identity receipt
`/media/sasaki/aiueo/benchmarks/m6a10_training_20260822/ntu_tnp01_identity.json`
(raw ROS 1 bag SHA-256
`817dad98fc922832c539d41c8f45583cc2674471e6d4772801ac0c40daa9fa71`,
canonical ROS 2 tree SHA-256
`c694c720d9f925a2094658891485016f40cfcfd84ea8c626dee80d9bca01e73e`).
The machine summary is
`/media/sasaki/aiueo/benchmarks/m6a10_training_20260822/ntu_tnp01_training_validation_summary.json`
(SHA-256
`4217a4b07f5ff85148e7433be1b9fef51e35d843a98bbb287d6f9c010c254177`).

That summary is deliberately `INCOMPLETE`/`FAIL_CLOSED`: ours has three
exit-zero attempts but no measured consumer drop/queue counters; GLIM has no
recognized input-EOF marker in the pinned log; FAST has no measured drop/queue
counters and its successful attempts measure online RTF about `1.0022`--`1.0025`
(above the preregistered `<=1.0` gate). Infrastructure failures are retained
as `.part` attempts. No fresh holdout, campaign4 payload, ground-truth content,
scorer, or accuracy result was accessed. The new metric therefore remains
validation evidence only, not a competitive or SOTA claim; a later run must
add authoritative consumer counters/EOF proof before any gate can pass.

## Fixed10-v2 failure and fixed10-v3 preflight

The single fixed10-v2 ours attempt is retained as immutable `FAIL_CLOSED` at
the external output root recorded by the profile. Consumer evidence was
otherwise complete (`230895` exact received/processed, no drop/overflow/
failure, EOF and empty backlog), but this is not sufficient to accept the
attempt: the phase evidence is invalid for missing `input_end` and exits
`125`, callback latency is `0.371609411` s versus the `0.25` s bound, and the
process-RSS evidence is invalid for `139.5325092%` jitter versus `100%`.
The root/output tree hashes and every evidence-file hash are recorded in the
profile and selection receipt. The attempt has no retry, no GT/scorer access,
and no promotion to a comparison result.

`m6a10-v2a-ours-rko-unpaced-ack-fixed10-v3` is a new, preregistered contract
with the same image, synchronized input, exact consumer counts, strict
unpaced acknowledgement requirement, and no-map-artifact contract. It adds a
read-only quiescence preflight. `scripts/check_m6a10_quiescence.py` records a
five-second `/proc` observation and rejects CPU busy ratio above 5%, load1 per
CPU above 0.5, or active compiler/build/docker-build/colcon/cmake/cargo
classes. Its atomic receipt SHA is required to match the v3 contract and
status `PASS` before runner start; missing, stale, overwritten, or failed
receipts are fail-closed. The v3 receipt is currently
`preregistered_not_run`, and no v3 bag replay has started.

The one v3 quiescence observation is immutable `FAIL_CLOSED` rather than a
permission to retry: receipt SHA-256 is
`74732fbdeb9bb8da094bacde3518cc2475eaeb29a3f4fec5c39d7972dc97a5a6`, CPU
busy was `98.72340425531915%`, load1-per-CPU was `0.805`, and eight compiler
processes were detected. `runner_start_allowed` was false, with no GT/scorer
access. Fixed10-v4 is the next preregistered contract; it preserves the v3
image/input/count/no-map identity and records the v3 preflight failure as its
predecessor. Its quiescence receipt is null until a future independent
preflight; no v4 replay has started.

## FAST-LIVO2 v2c execution preflight closure

The FAST-LIVO2 v2c image/identity preflight passed without mounting the raw
bag. The authoritative image is
`m6a10-v2c-20260823-fast-livo2-fixed2:competitive-v1` with ID
`sha256:89f6baeca8bc3691ed8465d0e19fd962cce708053cd28c8a4eebabb5b841f0ea`;
the execution preflight receipt is
`dcf53b48f0d3b2aab90a20ce345e08331a9512e6feefbc12e8333d9ea399b1c5`.

The single fixed10 consumer attempt was then fail-closed before runner start.
Its five-second quiescence receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/fast_livo2_v2c_fixed10_20260823/quiescence.json`,
SHA-256
`b3dc7ed143452e605f6315361fa7f3a909051bb05894b5e20c27737ffb754476`.
The observed CPU busy ratio was `98.04853640230172%` against a `5%` limit;
load1 per CPU was `1.135` against `0.5`, and eight compiler processes were
present. `runner_start_allowed` was false, retry remained zero, and no bag,
GT, scorer, Docker runner, or result artifact was started. This receipt is an
immutable infrastructure preflight failure, not a consumer or performance
result; a future attempt requires a separately preregistered root and a new
passing quiescence receipt.

## v2a synchronized-tail input preflight

Before an RKO-LIO replay, the canonical NTU `tnp_01` ROS 2 bag may be checked
without creating a derived bag. The read-only analyzer is
`scripts/analyze_m6a10_synchronized_tail.py`. Its schema is intentionally
narrow: `/os1_cloud_node1/points` must be
`sensor_msgs/msg/PointCloud2` with one little-endian `UINT32` (`datatype: 6`)
field named `t`. The field is contract-bound to nanoseconds relative to
`header.stamp`; PointCloud2 does not carry a unit declaration, so a different
datatype, endianness, field name, or alternate timestamp field fails closed.

For every scan the analyzer computes the point-level maximum
`header_stamp_ns + t_max_ns`. A scan is eligible only when

```text
point_timestamp_max_ns < last_imu_header_timestamp_ns
```

The comparison is strict because RKO-LIO requires the IMU buffer back to be
newer than the LiDAR point maximum. Input arrival order and system name do not
affect the decision. Ineligible scans must form a contiguous terminal suffix;
an ineligible scan followed by an eligible scan is a fail-closed error. The
receipt reports topic counts, the last IMU header timestamp, every terminal
scan's point min/max, eligible/ineligible counts, and the proposed cutoff and
duration. It writes only an atomic JSON receipt outside the input tree and
never opens GT or invokes a scorer.

The preregistered profile pin is
`competitive_slam_v1.yaml`'s `m6a10-v2a-synchronized-tail-v1` contract. The
NTU dry-run receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/synchronized_tail_dryrun.iD9dIn/receipt.json`
(SHA-256
`31072befe0ee816cfe09ee600f0ed604fd361a26e863e9bc51e5edab6f5f66d3`). It
passed with 225102 IMU messages, 5794 LiDAR messages, 5793 eligible scans,
one terminal ineligible scan, and a proposed end duration of 579.277931825 s.
This is an input-integrity preflight only; no output bag, replay, accuracy
metric, or performance claim is implied.

## v2a synchronized-tail materialization (ROS2 verified; ROS1 pending)

The generator is
`scripts/materialize_m6a10_synchronized_tail.py` (SHA-256
`caddcf0ae85d74444ae65ea85ed33d5e561a2569dc8ef180b2496a9d87c132c9`). Its
contract test is
`graph_based_slam/test/test_materialize_m6a10_synchronized_tail.py` (SHA-256
`f0c58ddbdfeaae2399125bfffc0ebef5ef5c57aba4dca72e7e8238ec0c3175d4`). The
status is `ros2_materialized_verified_ros1_pending`.

The fixed10 receipt is external to the bag at
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/synchronized_tail_materialization_v1_fixed10/receipt.json`
(SHA-256
`62defcf7a1b5cdadee666de44b09f71f8e011551ea93e37eec51b64a333cae9f`). It
records a 236688-record input and a 236687-record output, with 225102 IMU,
5792 image, and 5793 eligible LiDAR records. The one dropped terminal LiDAR
payload has SHA-256
`a11e441e679c424d31a43755d96328f94e73a1d2a52e48a1c808ac51f7443830`; the
output ordered payload stream is
`1512e3946d013b8543f09146b81474745ad9e718910b5f1322bda2823546e101`. The
published ROS2 tree SHA-256 is
`0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263`.
The independent raw-stream reread passed, including connection metadata
equality and the AnyReader order contract. This is an input-materialization
integrity result only; it is not a replay, accuracy, or performance claim.

The materializer streams the input with `rosbags.AnyReader`, drops only the
exact terminal-row identity in the analyzer receipt, and writes raw serialized
payloads, storage timestamps, and AnyReader chronological playback order
(including deterministic timestamp ties), plus connection metadata by
connection ID. Counts and ordered payload hashes are bounded-memory
accumulators. It verifies the staged stream before publication, then atomically
renames an inner final-basename bag from a sibling `.staging` container; a
legacy `.part` staging path, internal `.part.db3` name, overlap/symlink, stale
output, duplicate connection signature, or tampering fails closed. After
publication it performs only the output tree-hash check, avoiding a second
11-GB source replay. The receipt is a required external atomic JSON file and
never part of the output tree.

The previous artifact remains immutable but is marked
`superseded_not_promoted`: receipt SHA-256
`c6f0cec83956b3405f841242db0e8d5b71630477701728add74af2b3115c64a9` and tree
SHA-256
`4b4500450306aa60bf4b9daa79dc29a2cae8743a2b5ace197413bd06544fee9e`, due to
the old analyzer/materialization contract-ID mismatch and an internal
`.part.db3` filename.

FAST-LIVO2 remains a separate ROS 1 transport and has not been converted. Its
preregistered destination is
`/media/sasaki/aiueo1/datasets/ntu_viral_release/tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag`.
Conversion must stage the exact final `.bag` basename inside the sibling
`.staging` container, so the pinned command is:

```text
rosbags-convert --src <canonical_ros2> \
  --dst <ros1_staging_container>/<ros1_final_basename>.bag \
  --compress none --src-typestore ros2_humble --dst-typestore ros1_noetic
```

The observed converter is `rosbags-convert` 0.11.0, executable SHA-256
`83f210e4fd135eb81c12191b20fe06443eab0344398a5e0712283a538749bfc2`, help
SHA-256 `5fc5a415d32b0ccc953cc2b2f4f5213c1c201a9338b4b53eb7bdc1cebc23e4aa`.
Conversion is explicitly pending. Before any FAST run,
`scripts/compare_rosbag_semantic_inputs.py` (SHA-256
`7464d0e64ba1cafbdbb7a2e162bd2735b7288d9ea6e3a2dc1e7757bdab5fcf41`) must
report `all_topics_equal` for exactly the three NTU `tnp_01` topics
`/os1_cloud_node1/points`, `/imu/imu`, and `/left/image_raw`. No GT content or
scorer is opened by either checkpoint.

### Fixed10 v1 no-map failure and fixed10 v2 preregistration

The one fixed10-v1 ours replay is retained as immutable `FAIL_CLOSED`. Its
consumer and phase evidence were `PASS`, but the run output contained 18
forbidden map artifacts despite `--skip-map-save`; the independent
verification receipt is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/ours_m6a10_v2a_unpaced_ack_fixed10_v1/evidence/independent_verification_final.json`
(SHA-256
`ce0211f2e55e09ed2e8cb9f0692daf2dc5ebdf0f6707942292734bd542f35a0e`) and the
output tree SHA-256 is
`508df8568e9db1d51e8f167a12349cec33d04f61564b6154bf86244ae5f52624`.
There was no retry; GT content and the scorer remained unopened.

The cause is in the existing graph lifecycle, not the map-save service call:
`graph_based_slam_component.cpp` invokes
`doPoseAdjustment(..., use_save_map_in_loop_)` after accepted loop edges, and
the normal parameter default is `use_save_map_in_loop: true`. With
`do_save_map=true`, that path calls `saveGridDividedMap`,
`writeMapBundleArtifacts`, and `writeDegeneracyReport`; the launch also gave
pose-graph optimization a `save_dir/pose_graph.g2o` target. The runner's
`--skip-map-save` branch only skipped `/map_save`, while `dump_results:=true`
was intentionally retained for the trajectory dump. Thus skipping the service
was insufficient to establish a no-map output contract.

Fixed10-v2 is preregistered but not replayed. When the wrapper receives the
benchmark-only `M6A10_SKIP_MAP_SAVE=1` opt-in, it exports
`M6A10_BENCHMARK_NO_MAP_ARTIFACTS=1`. The source launch then appends the last
parameter overrides `use_save_map_in_loop=false` and
`save_pose_graph_path=''`; ordinary launches and YAML defaults are unchanged.
After launch termination, the runner fail-closes if any of `map.pcd`,
`pointcloud_map`, `map_bundle.yaml`, `map_projector_info.yaml`,
`degeneracy_report.yaml`, `pose_graph.g2o`, `trajectory_optimized.tum`, or
`loop_edges.csv` exists. Full offline and trajectory/consumer evidence remain
required. The v2 contract and output root are recorded in both benchmark
profile and execution-selection receipt with status
`preregistered_not_executed`. Its fixed10-v2 image-bound launch overlay is
SHA `d45545717f90f6877b5f281fc5623df04b824f7a236d2fe73b91c2dd3714371c` in
image `m6a10-v2a-fixed10-v2-lidarslam-ours:jazzy`, digest
`sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69`;
the installed launch path and OCI label are independently inspected before
replay. The recipe and build entrypoint are pinned by SHA
`99daea2172ae64f048557a9b069f48cd4b462de1581efd0b12457618dd360330` and
`2249b168cebaa640c657d095743a11d82b356123ad433806a443745a1f694b96`,
respectively.

### FAST-LIVO2 v2c single-inflight feeder (static, not executed)

The v2c implementation adds an opt-in, benchmark-only path. The pinned
`scripts/fast_livo2_m6a10_feeder.py` reads the fixed ROS 1 bag in bag order,
publishes exactly one record, waits for the patched mapper's
`/m6a10/consumer_status` callback counter to advance by exactly one, and then
calls `/m6a10/consumer_ack` before reading the next record. This is the
acknowledgement/backpressure proof; publisher counts and log text are never
used. The mapper reports callback acceptance, per-topic/global counts,
callback latency, processing failures, `std::deque` high-water, and final
backlog. `/m6a10/consumer_eof` atomically writes the EOF sidecar before the
bounded drain poll, and `/m6a10/consumer_finalize` writes final evidence only
after the internal deques reach zero.

The dedicated runner binds only
`/media/sasaki/aiueo1/datasets/ntu_viral_release/tnp_01_m6a10_v2a_sync_materialization_v1_ros1.bag`
to `/input/raw_input.bag:ro`; the input parent, calibration, GT, scorer, and
map output are not mounted. The source patch uses subscriber capacity one in
the opt-in path and keeps the mapper's internal `std::deque` buffers
unbounded. Exact counts are LiDAR 5,793, IMU 225,102, image 5,792 (236,687
total), and all three topics are required. Map saving is skipped in the
benchmark path, while ordinary FAST-LIVO2 behavior is unchanged when the
phase environment is absent.

This remains a static, non-executed benchmark contract. The pinned patch has
now been built once under a unique tag and its OCI labels, installed source
markers, and toolchain were independently inspected in a read-only,
network-isolated container. The observed image is recorded as
`build_passed_not_executed`; this does not authorize bag replay, GT access,
scoring, or a performance result.

The v2c static identity is pinned as follows: patch
`docker/patches/fast_livo2.m6a10-v2c.patch` SHA-256
`33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297`, feeder
`scripts/fast_livo2_m6a10_feeder.py` SHA-256
`bde0631d29dbb18575fe0fd2ce4bc339e738d14e1b47a1ccc8a77aa348f5622d`, runner
`scripts/run_fast_livo2_benchmark.py` SHA-256
`54db99e7f9da588baff33973490efb5474d86ca9888b1dae9578621810c38d48`, and
container wrapper `scripts/fast_livo2_container_run.sh` SHA-256
`f4236ec79659becacb12dc5f76c2d7ef41bc83be7616df88c1ee6ebc60e65d20`.
The reproducible Docker recipe (ordinary `git apply` semantics) is SHA-256
`a42686414c8400d9dba91cd7103840a703d77514cb33475a3c9a035fb100de1e`, and
the build entrypoint is SHA-256
`db740f2944b64ac56a8cd63aa9119075927ce12e627b8237708c178994db33a9`.
The observed image is
`m6a10-v2c-20260823-fast-livo2-fixed2:competitive-v1`, ID/digest
`sha256:89f6baeca8bc3691ed8465d0e19fd962cce708053cd28c8a4eebabb5b841f0ea`.
Its immutable build receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/`
`m6a10-v2c-fast-livo2-build-20260823-fixed2/build_receipt.json` (SHA-256
`db29eab240852d6f7fd926f51aefc66547c72fb0e8a1193b978e3d34b65c8926`).
No bag replay, GT access, scoring, or runtime observation is authorized by
this build gate.

The separate execution-identity preflight also passed without mounting the
bag. Its immutable receipt is
`/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/`
`m6a10-v2c-execution-preflight-20260823-final/identity_preflight.json` (SHA-256
`dcf53b48f0d3b2aab90a20ce345e08331a9512e6feefbc12e8333d9ea399b1c5`). The
preflight producer is `scripts/preflight_fast_livo2_m6a10_v2c.py` (SHA-256
`92437c92246f47acc80abaf4e33588298cc4522b87052c1f8af4b2dfedce6a12`). The
probe used `--network none`, a read-only rootfs, and only temporary writable
directories; the planned raw-bag bind is `/input/raw_input.bag:ro`, while the
input parent, GT, scorer, and map outputs remain forbidden. This PASS is an
identity/preflight observation only and does not authorize replay.

### FAST-LIVO2 v2c fixed10-v4 bounded-attempt correction and v5 observability

The fixed10-v4 attempt is retained as an immutable `FAIL_CLOSED` diagnostic,
not as a performance result. Its quiescence receipt is
`0f0110b203364b5e2c9bdfeb343bd96416c8d8d232bcfe2298e701f525606556`; the
attempt closure is
`1292cbadf0eff2575f2df3014a73e54822d8d57f6703cba5832e3db91065bfd1`.
The host runner was stopped after bounded no-progress supervision, and the
container was subsequently force-removed after its 30-second stop grace
period. There is no OOM evidence, and the absence of a first callback ACK is
not proven because v4 had no progress checkpoint. The correction receipt
`861585b554cee8240bd4cae811dbec46c7256b83ba40a30c4e28a91ca690c93f`
therefore records `no_progress_evidence_under_bounded_supervision`, rather
than claiming an unsupported callback stall. The v4 raw attempt and all
predecessor receipts remain unchanged; no GT or scorer was accessed.

The preregistered v5 instrumentation keeps the algorithm, input, mount graph,
and fairness contract unchanged. The feeder now emits line-buffered, atomic,
non-authoritative `feeder_progress.json` checkpoints before a publish, after
the first callback ACK, and every 100 acknowledged records. The host runner
uses a stable container name and cidfile, and atomically records a
  reconnectable `host_lifecycle.json` with non-destructive Docker inspect
  snapshots at a two-second interval. Low-frequency `docker stats` snapshots
  are diagnostic only. The feeder checkpoint is written before the first
  publish, after the first callback ACK, and every 100 ACKs; it is not written
  or printed once per message. Drain progress uses a separate file and cannot
  erase feeder progress. These diagnostics are never accepted as completion
  evidence; authoritative consumer/phase receipts remain the only completion
  gate. The v5 attached container is retained long enough to inspect its exit
  state and is removed only after its run receipt is persisted; an interrupted
  supervisor does not signal or remove it, so another supervisor can reconnect
  by name/cidfile. The v5 output root is
`/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/fast_livo2_v2c_fixed10_v5`;
its status is `preregistered_not_executed` and its image is explicitly marked
`rebuild_required_for_observability_label`. No v5 quiescence or replay has
been started.
The current source bindings are feeder
`bde0631d29dbb18575fe0fd2ce4bc339e738d14e1b47a1ccc8a77aa348f5622d`, host
runner `54db99e7f9da588baff33973490efb5474d86ca9888b1dae9578621810c38d48`,
and Docker recipe
`a42686414c8400d9dba91cd7103840a703d77514cb33475a3c9a035fb100de1e`. The
recipe adds the v5 observability OCI label, so the existing image digest is
not silently reused for v5.
The immutable v4 record retains its historical broad `*map*` detector. The
new v5 preregistration uses a narrower, explicit map-artifact denylist
(`*.pcd`, `map.pcd`, `map_bundle.yaml`, `map_projector_info.yaml`,
`degeneracy_report.yaml`, `pose_graph.g2o`, `trajectory_optimized.tum`,
`loop_edges.csv`, `pointcloud_map`, `*.bag`, `*.db3`, and `*.part`). Required
non-map diagnostics `mapper.log` and `mapper_ready.txt` are explicitly
allowed and are never interpreted as map output.
The profile/selection identity chain was resynchronized after this
preregistration: canonical profile SHA
`3904791e1dff0b6841cf40133c7c96327254faf9797dc1827badf6f0d9477d4c`,
selection file SHA
`ecca86bca67704ccd653da450344de3fd6ba52ad23baeddfaae9ae3569e4e7f2`.
