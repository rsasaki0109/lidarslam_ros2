# Changelog

## 0.9.1 - 2026-08-12

A reliability and first-map usability release candidate. It keeps the v0.9.0
publication immutable while making the supported Docker and source journeys
more guided, recoverable, and reviewable.

### Highlights

- **One coherent beginner surface** — the no-argument `lidarslam-map` home,
  bag-optional doctor, guided `start`, Japanese quickstart, and fixed MID-360
  demo share the same profiles, preflight, verification, and recovery contract.
- **Crash-safe point-cloud handling** — classic scanmatcher VoxelGrid paths and
  graph point-cloud conversions fail closed on unsafe dimensions, overflow,
  invalid leaves, or inconsistent dense clouds while preserving the process and
  its last valid map.
- **Recoverable map lifecycle** — atomic outputs, finalization-only resume,
  session history and comparison, source-preserving edit/merge plans, local 3D
  preview, and privacy-bounded support/first-map receipts keep provenance and
  next actions visible.
- **Fresh-terminal onboarding** — Docker and the exact six-package Humble/Jazzy
  source route now converge on one verified first-map outcome, with installed
  resource checks and public-readiness probes.
- **One-command candidate trial session** — a dedicated observer can start from
  one exact Actions run URL and retain the authenticated handoff, selected
  Docker/source execution, and a hash-bound session receipt under one atomic
  directory. Docker rows bootstrap a content-addressed, recipe-labelled local
  observer image before timing instead of requiring a copied build command.
- **Guided trial readiness** — the same session command now checks the exact
  destination, Ubuntu/ROS row, x86_64 host, measured filesystem, Docker/source
  runtime, source RX counter, human measurements, and isolation confirmation
  without network access or writes. Stable findings distinguish a blocked host
  from runnable non-comparable evidence and print one shell-safe next command.
- **Release identity hardening** — release-bundle CI checks out the exact PR
  head, fetches immutable tags, and refuses historical version reuse. Candidate
  metadata is aligned at 0.9.1 without moving or recreating `v0.9.0`.
- **Reviewable candidate operations** — the GET-only protected-environment
  audit and G0 card now provide a status-specific, copy-ready administrator
  handoff while keeping settings writes, independent approval, and E2 dispatch
  outside the tool.
- **Parse-safe GLIM comparison** — restore the Docker timeout branch in
  `compare_with_glim.sh` and make every tracked shell script pass `bash -n` in
  the maintained docs/product gate.
- **Content-verified GLIM fallback** — replace the path/topic-only trajectory
  cache with a schema-backed identity over bag, effective config, runtime,
  options, and harness bytes; malformed, modified, legacy, or contradictory
  entries now fail closed instead of becoming cross-validation evidence.
- **Exact public-docs identity** — bind the deployed Getting Started bytes,
  canonical route fragments, product version, and Pages workflow to one source
  commit, and keep first-time cohorts closed when that manifest is absent,
  stale, or tampered.
- **Copy-safe clean-host identity** — derive the source commit and both Docker
  digests from one validated published-release report, then recheck those exact
  live identities before a comparable onboarding row can start.
- **Honest adoption evidence** — onboarding and GLIM usability comparisons are
  schema-backed, environment-matched, and fail closed when rows are missing;
  no overall winner is inferred from incomplete evidence.

### Remaining v1.0 gates

- complete the reviewed NDT ownership/rosdistro and main-channel package path;
- capture complete Humble/Jazzy Docker/source onboarding measurements; and
- accept three independent first-map validations. The tracked count remains
  0/3.

## 0.9.0 - 2026-07-30

The stable product-foundation release candidate. It keeps the v0.7 golden
path and adds fail-closed, machine-readable gates for the remaining v1.0
distribution, publication, and external-adoption work.

### Highlights

- **Cross-phase v1 readiness audit** — one schema-validated contract reports
  all ten product gates, verifies evidence paths, release version/tag state,
  and the independent-user ledger, and refuses to describe incomplete work
  as ready.
- **Release-ready binary dependencies** — the official PRBonn RKO-LIO
  `0.3.2-1` binaries passed the pinned Humble/Jazzy installed golden-path E2E,
  and the product now requires `rko_lio >= 0.3.2`. The pinned
  `ndt_omp_ros2 0.1.0` candidate has a read-only preflight that distinguishes
  local readiness, ready-to-tag, partial publication, and completed
  Humble/Jazzy rosdistro state.
- **Package-manager evidence gate** — a manual Humble/Jazzy workflow installs
  exact ROS apt versions, validates every installed CLI/runtime resource,
  detects stale paths across main-to-testing upgrades, and runs the pinned
  real MID-360 map contract. It remains evidence-open until public
  lidarslam packages exist. The live readiness audit now verifies the exact
  immutable-tag/main-channel run identity and both named-distro jobs, so a
  workflow definition or hand-edited gate cannot stand in for execution.
- **Operationally honest distribution** — docs distinguish ROS testing from
  the normal main repository, preserve exact dependency and candidate state,
  and withhold the beginner apt command until main-channel installation
  evidence passes. A post-publication audit validates the stable release,
  tag commit, all image/rollback/promotion records, and every archived bundle
  file hash before publication evidence is accepted.
- **Pre-tag bundle proof** — main CI builds the curated release bundle twice
  from the clean candidate commit, reuses the complete publication verifier
  against both archives, requires byte-identical output, and retains the
  verified rehearsal artifact before an immutable release tag can be created.
- **Actionable benchmark blockers** — every blocking release profile carries
  a tracked remediation command that is printed for `NO_DATA`/`FAIL`. The
  Newer College row now names the official form-gated Maths-Hard sequence and
  its ICP-to-survey-map ground truth instead of incorrectly describing it as
  prism ground truth.
- **Idempotent publication guidance** — the live `ndt_omp_ros2` release audit
  now points at the already-generated Humble/Jazzy rosdistro PRs and explicitly
  forbids recreating the source tag or rerunning Bloom while those PRs remain
  current.
- **Complete default pytest coverage** — every Python test module in
  `graph_based_slam` and `lidarslam` is now registered with CTest, and CI rejects
  future modules that are added without a matching registration.
- **Extensible degeneracy research boundary** — the oriented appearance
  matcher exposes separate intensity/height diagnostics behind default-off
  adapters. Fog and tunnel holdouts did not justify automatic channel
  selection, so no experimental policy became a product default.

### Remaining v1.0 gates

- publish `ndt_omp_ros2 0.1.0`, then the four lidarslam packages, into Humble
  and Jazzy rosdistro;
- wait for required dependencies and lidarslam packages to reach the normal
  ROS apt repository, then capture package-manager install/upgrade evidence;
- publish and verify the first immutable release carrying rollback assets;
- accept three independent first-map validations. The tracked count remains
  0/3.

## 0.7.0 - 2026-07-29

The productization release candidate. It turns the research-oriented workspace
into a documented offline rosbag2-to-Autoware-map product while retaining the
deterministic and map-quality foundations delivered after v0.6.

### Highlights

- **One installed product CLI** — `lidarslam-map doctor`, `run`, `inspect`,
  and optional `view` now drive the same source and installed workflow.
  Machine-readable option contracts, Bash completion, stable exit behavior,
  preflight-v3, diagnosis-v1, and resumable run-manifest-v2 records are tested
  on Humble and Jazzy. Path and duration metavars use one consistent
  vocabulary, completion must exactly match the option contract, and invalid
  names, values, or combinations consistently exit with status 2. Doctor,
  runner help, parser choices, completion, and release bundles share one
  installed maintained-profile registry.
- **Fail-closed map authoring** — output collision, low free space, real
  bounded-filesystem exhaustion, timestamp reversal, process termination, and
  partial-run recovery preserve terminal manifests and actionable diagnosis
  instead of claiming success.
- **Reproducible distribution** — clean-prefix fresh/upgrade parity, versioned
  Humble/Jazzy image automation, SBOM, BuildKit and GitHub provenance,
  digest-only rollback planning, and immutable two-image release promotion
  are release-gated.
- **Real-data operations evidence** — the pinned public MID-360 E2E gate,
  one-hour and eight-hour named-hardware soaks, periodic resource telemetry,
  bounded iteration timeouts, and a real 32 MiB ENOSPC gate publish
  schema-validated evidence.
- **Independent adoption contract** — a structured first-map issue form,
  schema-validated public ledger, and strict readiness command reject
  duplicate reporters, failed runs, and unreviewed evidence while preserving
  an honest 0/3 status before v1.0. Product runs now emit a privacy-bounded
  receipt that binds the final manifest, diagnosis, and map-verifier log
  without publishing geometry or private paths. Receipt parsing accepts the
  verifier's canonical explanatory `RESULT` line and rejects malformed status
  lookalikes.
- **Map-quality and architecture work** — deterministic offline frontend and
  backend runners, event-driven loop search as the only backend path,
  map-quality metrics, offline refinement, stronger Graph SLAM ownership
  boundaries, and opt-in degeneracy/colouring research remain available
  without expanding the beginner surface.

### Current boundaries

- ROS buildfarm packages are not yet released because `ndt_omp_ros2` remains
  outside rosdistro. Official RKO-LIO release entries and binaries now exist
  for Humble and Jazzy, but testing/main versions differ and clean
  installed-E2E compatibility with the maintained fork remains a release
  gate.
- amd64 is the tested image platform; arm64/Jetson remains evaluation tier.
- This is a prerelease candidate. The v1.0 external-adoption gate still
  requires three independent users to complete first-map validation.

## 0.6.0 - 2026-06-12

The deterministic core / ROS shell refactor (roadmap `docs/roadmap/v0.6.md`,
PRs #237–#262). Same bag + same config now provably produces the same map:
the offline runners replay recorded inputs through the same cores the live
nodes use and the release gate enforces byte-identical results across runs.

### Highlights

- **Deterministic offline mapping (backend)** — `graph_slam_offline_runner`
  replays a recorded odometry bag through submap creation, event-driven
  BackendCore loop search and pose-graph optimization with no executor and
  no wall clock; 3 runs are *byte-identical* (loop edges + optimized
  trajectory) on both gate substrates (MID-360: APE 7.262851264566504 to the
  last digit; NTU VIRAL: 9 loop edges, Leica-GT APE 0.8460511516520233).
  Before the refactor the same input produced different loop edges every run.
- **Deterministic offline mapping (frontend)** — `scan_matcher_offline_runner`
  drives the real `ScanMatcherComponent` in lockstep over a raw sensor bag
  (intra-process, drained executor, synchronous map update); 3 runs over the
  full NTU tnp_01 bag are byte-identical (5795 poses, 133 submaps) — the
  first determinism coverage for the scanmatcher (NDT) frontend.
- **Event-driven loop search by default** — the backend now searches loops on
  submap arrival instead of a wall-clock timer; the v0.4
  `deterministic_loop_scheduling` experiment is retired (its negative result
  motivated this architecture). The legacy timer path stays available behind
  `event_driven_loop_search: false` for one release.
- **Release gate hard-enforces determinism** — opt-in
  `--offline-determinism-bag` / `--frontend-determinism-bag` stages in
  `run_release_readiness_checks.sh` fail the gate on any byte-level mismatch.
- **Real bug fixes surfaced by the refactor** — silent ~6% input drop under
  load (QoS KeepLast(1) + best-effort), IMU/GNSS pose-graph edges weighting
  the wrong error blocks (GNSS georeferencing never actually pulled), missing
  tie-breaks in candidate ranking, GNSS yaw alignment with gauge release
  (first georeferenced maps).
- **God objects decomposed with characterization tests** —
  `graph_based_slam_component.cpp` 3421 → 2265 lines behind 19 tested pure
  headers; `scanmatcher_component.cpp` 2140 → 1694 lines with 4 new tested
  pure headers (pose prediction, pose acceptance incl. the tracking state
  machine, IMU processing, map-update policy). Package test count grew to
  1100+.

## 0.5.0 - 2026-06-11

First version aligned across `VERSION`, the four core `package.xml` files and
the release docs, prepared for the first rosdistro (bloom) binary release.
Covers the v0.4 and v0.5 roadmap milestones on top of 0.3.0.

### Highlights

- **Real ground truth on Livox MID-360 (v0.5 milestone)** — all four RTK-SLAM
  sequences measured against total-station checkpoints; the two indoor
  Construction Hall profiles now block releases (pass 0.30 m / 0.55 m), the
  outdoor Stadtgarten pair soaks as report-only with a dedicated outdoor
  preset (`double_downsample: false`: 3.9 m → 0.835 m), and the
  `mid360_vs_glim` cross-validation gate is demoted to a report-only canary.
- **Deterministic loop scheduling (v0.4 milestone)** — opt-in
  `deterministic_loop_scheduling` parameter (default off) that deterministically
  catches up unqueried submaps instead of latest-only wall-clock scheduling;
  the research-track release profiles graduated to blocking gates.
- **One-command everything** — ghcr Docker demo image
  (`docker run ... ghcr.io/rsasaki0109/lidar_slam_ros2:humble` downloads a
  public MID-360 bag and produces the map headless), the beginner bag-to-map
  runner, and the Autoware bundle now emits `lanelet2_map.osm` alongside
  `pointcloud_map/` + `map_projector_info.yaml`.
- **Photoreal map deliverables** — camera-colored point-cloud map flythrough
  (README hero GIF) with occlusion-aware robust colorization, plus the
  documented optional 3DGS pipeline (gsplat, Apache-2.0).
- **rosdistro release prep** — per-package `CHANGELOG.rst`, SPDX
  `BSD-2-Clause` license tags, versions aligned at 0.5.0, and a bloom runbook
  (`docs/rosdistro-release.md`).

## 0.3.0 - 2026-05-25

Public `v3` release. Carries forward the non-GPL default workflow from
`v0.2.x` and adds the Livox MID-360 operator
toolkit, an opt-in STD/BTC-style triangle descriptor research stack with
variance-validated defaults, dogfood wrapper measurement plumbing, and a
user-friendly README.

### Highlights

- **Livox MID-360 operator toolkit** — Jetson-class robot-side recording →
  SLAM → Autoware map workflow with host preflight, bag stamp rewriter,
  recording / production-candidate sessions, public-dataset map runner, and
  dashboards
- **Opt-in STD/BTC-style triangle descriptor stack** — BSD-2 implementation
  with `edge_3d` keypoint extractor for narrow-FOV / indoor, fine-grained ROS
  params, a diagnostic `triangle_descriptor_skip_ransac` flag, and an
  empirically validated MID-360 default of `max_pairs: 16`; default
  `use_triangle_descriptor: false` on every preset
- **Dogfood wrapper measurement plumbing** — frame overrides,
  quiescence-based offline completion, graph-drain wait, corrected-path
  capture, and reference-TUM APE evaluation
- **User-friendly README** — 5-minute "try the public default" path, badges,
  grouped feature lists, progressive disclosure of research tracks

### Research closeout

- 3-dataset variance evidence (NTU / MID-360 / Newer College) shows the
  triangle stack does not yet provide a reproducible APE win — it is opt-in
  research only
- single-run APE claims on MID-360 triangle ablations are unreliable
  (variance can exceed observed |Δ| by 8x); all reports in this release ran
  ≥3 runs with mean ± std + `|Δ|/σ`
- MID-360-specific `+1 m` APE drift was traced to RANSAC compute cost, not
  the act of enabling the pipeline; `max_pairs: 32 → 16` on the MID-360 yaml
  eliminates it (U-shaped sweep; `=16` is the empirical sweet spot)
- full narrative: `docs/research/triangle-stack-2026-05-summary.md`

### Notes

- the recommended public workflow remains `RKO-LIO + graph_based_slam` with
  distance-based loop closure
- the default release path remains non-GPL and focused on pointcloud-map
  generation for Autoware-compatible workflows
- The triangle stack is additive — it does not change the default
  public path

## 0.2.2 - 2026-03-28

Public `v2 beta` patch release focused on release stability and cross-distro CI
consistency.

### Highlights

- fixed Humble/Jazzy style and include-path mismatches that appeared after the
  `0.2.1` release-prep refresh
- kept the public `RKO-LIO + graph_based_slam` workflow, reports, and release
  metadata aligned on `develop`
- validated the patched release scope with green `docs`, `humble`, `jazzy`,
  `release readiness`, and threshold-guard workflows

### Notes

- this is a patch release over `0.2.1`, not a scope expansion
- public defaults and known limits remain unchanged from `0.2.1`

## 0.2.1 - 2026-03-28

Public `v2 beta` refresh focused on map-authoring workflow hardening and
clearer fallback-path positioning.

### Highlights

- GNSS-aware graph optimization now uses covariance-based weighting and has
  real open-data validation for both direct `NavSatFix` bags and Applanix
  sidecar conversion
- packet-path IMU deskew was hardened around `PointCloud2.time` handling and
  validated on real open data with a repeatable matrix report
- save-time dynamic-object filtering now has cross-dataset validation on Leo
  Drive `bag1` and `bag6`, with roughly `50%` saved-point reduction while
  keeping verification `PASS`
- classic-path fallback benchmarking now includes GNSS-only, IMU, and
  velocity-based odom-prior comparisons, with a tracked validation report that
  keeps dataset-specific knobs out of the public default
- exploratory place-recognition work is now explicitly closed out: distance
  remains the public default, while Scan Context, BEV rerank, and SOLiD stay
  opt-in or experimental
- map-authoring reporting and submission-bundle tooling were extended so maps,
  metrics, logs, and focused reports can be packaged and compared more
  repeatably

### Notes

- the recommended public workflow is still `RKO-LIO + graph_based_slam`
- the default release path remains non-GPL and focused on pointcloud-map
  generation rather than full production autonomy stacks

## 0.2.0 - 2026-03-25

Public beta candidate for the `v2` release line.

### Highlights

- recommended default workflow narrowed to permissive-license components
- `RKO-LIO + graph_based_slam` established as the dogfooded default path
- graph backend improved with better adjacent edges, loop dedup, robust kernels,
  multi-candidate validation, and safer state handling
- Autoware-compatible pointcloud-map export hardened with
  `map_projector_info.yaml` and bundle verification
- end-to-end Autoware dogfood flow added:
  `rosbag2 -> SLAM -> map save -> Autoware map loaders -> rviz2`
- benchmark reporting, HTML report generation, and release/readiness gate added
- CI expanded with default workflow checks and release-readiness fixture jobs
- contribution guide, Autoware quickstart, benchmarking guide, and issue
  templates added for external reports
- fixed public Autoware entrypoint added: `scripts/run_autoware_quickstart.sh`
- comparison page and checked-in release notes added for public `v2 beta`
- MID360 current default tuned to `voxel_size=0.5`, `max_range=80.0`,
  `search_submap_num=5`, `loop_edge_dedup_index_window=20`,
  `loop_edge_info_weight=200`

### Notes

- this release is suitable for public beta / developer preview distribution
- the default workflow remains focused on pointcloud-map generation for
  Autoware; lanelet generation is intentionally out of scope
