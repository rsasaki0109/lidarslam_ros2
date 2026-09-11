# HILTI degeneracy baseline (v0.8 Phase 0)

Status: recorded 2026-07-05, before any degeneracy-aware detection or
intervention code exists. This is the "the optimizer must not be allowed to
move its own judge" freeze for `docs/roadmap/v0.8.md` — the same discipline
`docs/research/map-quality-baseline.md` used for v0.7 Phase 1. Every later
v0.8 phase gate (Phase 1 classification accuracy, Phase 2 recovery targets,
Phase 3 default-on decision) is judged against the numbers in this document,
not the other way around.

Two independent things are frozen here:

1. **Real-substrate raw-odometry baseline** — HILTI 2022 exp01/exp07 (already
   measured, v0.7 Phase 4) plus exp04 (measured 2026-07-05 in this pass, §3).
2. **Synthetic degenerate-scenario oracle** — three deterministic fixtures
   (corridor / box / single-plane) with recorded point-to-plane Gauss-Newton
   `H` eigenvalue signatures, the fixture data Phase 1's planned
   `localizability_analysis.hpp` classifier will be unit-tested against.

## 1. Real-substrate raw-odometry baseline (HILTI 2022)

Source: exp01/exp07 from v0.7 Phase 4 (`docs/research/project-plan-archive.md` §2.4, PR #280/#282),
reconfirmed here unchanged; exp04 measured 2026-07-05 in this pass (§3).
Sensor: Hesai PandarXT-32 + Alphasense IMU (handheld Phasma
platform). License: CC BY-NC-SA 3.0, evaluation-only, not redistributed.
Config: `configs/hilti2022/rko_lio_hilti2022_pandar.yaml` (voxel 0.25,
deskew on). Scoring: dense RKO-LIO raw odometry vs the sparse
total-station control points, `ape_from_tum.py --interpolate` (the dense
trajectory is scored, not the sparse submap/corrected trajectory — see
"Methodology note" below).

| sequence | environment | RKO-LIO raw RMSE (m) | median (m) | path | pairs | loops |
|---|---|---:|---:|---|---:|---:|
| exp01 | construction ground level (feature-rich) | **0.066** | 0.053 | 904 m one-shot sweep | 13 | 0 |
| exp07 | long corridor (degenerate) | **0.318** | 0.329 | 124 m one-shot sweep | 6 | 0 |
| exp04 | construction upper level (feature-rich) | **0.071** | 0.055 | 82 m one-shot sweep (125.8 s) | 7 | 0 |

Per-axis residuals, exp07 (the frozen evidence motivating v0.8) vs exp04
(measured 2026-07-05, same SE(3)-Umeyama-aligned interpolate scoring):

| axis | exp07 residual (m) | exp04 residual (m) |
|---|---:|---:|
| x | 0.145 | 0.040 |
| y | 0.235 | 0.039 |
| z | 0.157 | 0.045 |

On exp07, one axis (empirically the largest of the three, the long axis of
the corridor) carries substantially more error than the other two — the
signature of a *directionally* under-constrained registration problem, not
a uniformly noisy one (`docs/roadmap/v0.8.md` §0). exp04's residuals are
near-isotropic (0.039–0.045 m across all three axes) — the feature-rich
control case showing what a well-constrained sweep looks like on the same
sensor/config, mirroring exp01.

All three sequences contribute **zero loop edges** from `graph_based_slam`,
so this is confirmed a raw-odometry accuracy problem, not a
backend/loop-closure problem (corrected == raw at every submap timestamp:
0.000 m on exp01/exp07, v0.7 Phase 4 finding; re-verified 0.0000 m across
all 42 submap poses on exp04). exp01/exp07 are non-revisiting single
sweeps. exp04 *does* physically revisit its start (the first and last of
its 7 control points are the same coordinates), but its 82 m path never
clears the 100 m `distance_loop_closure` gate, so no loop candidate is
ever eligible — the pose graph holds only the adjacent-window constraints
(max edge `|i-j|` = 5), same passthrough behaviour as exp01.

**Methodology note** (carried over from v0.7 Phase 4, `docs/research/project-plan-archive.md` §2.4): score
the *dense* RKO-LIO odometry trajectory, not the sparse
submap/pose-graph-corrected trajectory. HILTI's control points are captured
while the platform is stationary; a sparse corrected/submap trajectory's
linear interpolation across those stationary dwells introduces a scoring
artifact (observed ~0.89 m on exp01's corrected trajectory, versus 0.066 m
scored correctly on the dense trajectory) — this is an interpolation
artifact of sparse-trajectory scoring, not real degradation. Use
`ape_from_tum.py --interpolate` against the dense trajectory (as the
commands below do), or `--sparse-match` if a sparse trajectory is ever the
only one available.

## 2. Release-gate wiring (`scripts/release_profiles.yaml`)

Three new **report-only** rows, `report_only_until: v0.8`, added in this
Phase 0 pass (no HILTI profile existed in `release_profiles.yaml` before
this):

| profile | match | metric | pass (frozen) |
|---|---|---|---:|
| `hilti2022_exp01` | `points_topic: /hesai/pandar`, `reference_kind: ground_truth`, `reference_source_contains: hilti2022_exp01_control_points_gt`, `min_ape_pairs: 10` | `ape_rmse_gt_m` | 0.066 |
| `hilti2022_exp07` | `points_topic: /hesai/pandar`, `reference_kind: ground_truth`, `reference_source_contains: hilti2022_exp07_control_points_gt`, `min_ape_pairs: 4` | `ape_rmse_gt_m` | 0.318 |
| `hilti2022_exp04` | `points_topic: /hesai/pandar`, `reference_kind: ground_truth`, `reference_source_contains: hilti2022_exp04_control_points_gt`, `min_ape_pairs: 5` | `ape_rmse_gt_m` | 0.072 |

Unlike prior soak-then-graduate report-only rows (e.g.
`mid360_gt_rtkslam_stadtgarten_*`), `pass` here is pinned to the *exact*
frozen baseline rather than a looser graduation threshold — these rows
exist as a regression canary against this document's numbers, not as a
target to clear. (`hilti2022_exp04`'s 0.072 is the measured 0.0715 rounded
*up* to 3 dp so the frozen run itself evaluates PASS; rounding down to
0.071 would make the baseline run WARN against its own freeze. Verified
PASS at 0.071 against the 2026-07-05 run via `benchmark_summary.py
--release-profile`.) No `target` is set (no improvement number is chosen yet;
`docs/roadmap/v0.8.md` §9 item 3 explicitly defers the Phase 3 improvement
margin until Phase 2 data exists). Because these rows only evaluate when a
matching local run under `--root` exists, and HILTI is CC BY-NC-SA /
manually fetched (no guaranteed CI access), "blocking" for this substrate
is a manual `RELEASING.md` pre-release checklist item, not a
`report_only_until` flip (`docs/roadmap/v0.8.md` §6).

`scripts/download_hilti2022.sh`'s suggested `--reference-source` label was
corrected in this pass from `hilti2022_${SEQUENCE}_control_points` to
`hilti2022_${SEQUENCE}_control_points_gt` — `benchmark_summary.py`'s
`_infer_reference_kind` only classifies a reference as `ground_truth` when
its source label contains `gt` or `ground_truth`; without the suffix the
profiles above would never match a real run (`reference_kind:
ground_truth` would never be satisfied).

## 3. exp04 — measured 2026-07-05

exp04 (`exp04_construction_upper_level`, "construction upper level"): bag
downloaded and converted via `scripts/download_hilti2022.sh --sequence
exp04` (13.5 GB rosbag2, 125.8 s, 1258 `/hesai/pandar` PointCloud2 + 50198
`/alphasense/imu` msgs; kept off-repo under an external datasets
directory), 7 total-station control points. Run 2026-07-05 on ROS 2 Jazzy
(Ubuntu 24.04), workspace built with `colcon build --symlink-install
--cmake-args -DCMAKE_BUILD_TYPE=Release` (note: the `rko_lio` Thirdparty
submodule additionally needs `-DRKO_LIO_FETCH_CONTENT_DEPS=ON` on a
machine without a system Sophus, per `Thirdparty/rko_lio/README.md`).

**Frozen result** (dense raw odometry vs 7 control points,
`ape_from_tum.py --interpolate --max-time-diff 3.0`, SE(3) Umeyama):

- RMSE **0.0715 m**, median 0.0546 m, mean 0.0665 m, min/max
  0.0355/0.1141 m, 7/7 paired (0 rejected, max interpolation bracket
  0.100 s)
- per-axis RMSE x 0.040 / y 0.039 / z 0.045 m — near-isotropic (§1)
- path 82 m, 1258 raw poses @ ~10 Hz, wall 169 s (RTF 1.34 incl. 60 s
  quiescence), zero loop edges (82 m < the 100 m `distance_loop_closure`
  gate despite the start-revisit in the GT; §1), corrected == raw to
  0.0000 m at all 42 submap poses, Autoware map verify 8/8 PASS

Commands as actually run (bag + outputs on an external disk; repo root
otherwise as in §5):

```bash
# 1. Download + convert (run once; ~13.5 GB rosbag2)
bash scripts/download_hilti2022.sh --sequence exp04

# 2. Raw-odometry benchmark run (same config family as exp01/exp07).
#    --reference-meta points at a `{}` placeholder JSON: see the harness
#    caveat below.
bash scripts/run_rko_lio_graph_benchmark.sh \
  --bag <datasets>/hilti2022/exp04_ros2 \
  --lidar-topic /hesai/pandar --imu-topic /alphasense/imu \
  --rko-param configs/hilti2022/rko_lio_hilti2022_pandar.yaml \
  --reference-tum <datasets>/hilti2022/exp04_construction_upper_level_gt.txt \
  --reference-meta <work>/hilti2022_exp04_reference_meta.json \
  --skip-reference-gen --reference-source hilti2022_exp04_control_points_gt \
  --quiescence-secs 60 --offline-timeout-secs 5400 \
  --output-dir <work>/output/hilti2022_exp04_run

# 3. Score dense raw odometry vs the sparse stationary control points
python3 scripts/ape_from_tum.py --interpolate --max-time-diff 3.0 \
  --ref <datasets>/hilti2022/exp04_construction_upper_level_gt.txt \
  --est <work>/output/hilti2022_exp04_run/traj_raw.tum \
  --out <work>/output/hilti2022_exp04_run/ape_raw_interpolate_vs_gt.txt

# 4. Fold into the release-gate summary (hilti2022_exp04 row added in this
#    pass; evaluated PASS at 0.071 <= 0.072 against this run)
python3 scripts/benchmark_summary.py --root <work>/output \
  --release-profile scripts/release_profiles.yaml --write-md summary.md
```

Harness caveats found while running this (pre-existing
`run_rko_lio_graph_benchmark.sh` behaviour, worked around — not fixed in
this pass; candidates for a separate PR):

1. `--skip-reference-gen` does not fully skip reference generation: the
   guard also regenerates whenever the reference TUM/meta files are
   missing, and the default `--reference-meta` is the NTU-VIRAL-specific
   `output/ntu_viral_tnp01_reference.json`. On a HILTI run without that
   file present, the script calls the NTU-only
   `generate_ntu_viral_tnp01_reference.py` and dies on a missing
   `leica_prism.yaml`. Workaround: pass `--reference-meta` explicitly,
   pointing at a `{}` placeholder JSON (safe — the metrics writer treats
   every field as optional and the prism offset defaults to zero, which is
   correct for HILTI's IMU-frame control points).
2. The script's built-in corrected-trajectory APE uses default
   nearest-neighbour association (0.05 s): on exp04 all 7 stationary
   control points fall > 0.05 s from the sparse 42-pose submap trajectory,
   `ape_from_tum.py` exits 1, and `set -e` aborts the script *before*
   `metrics.json` is written (the raw APE and all trajectories/maps are
   already on disk at that point). `metrics.json` was therefore written by
   invoking `scripts/write_rko_lio_benchmark_metrics.py` directly with the
   same arguments the script would have passed, feeding the §3-step-3
   dense-trajectory interpolate APE report as the canonical APE — honest
   here because corrected == raw to 0.0000 m (backend passthrough, §1) and
   the methodology note (§1) mandates dense-trajectory scoring; a
   sparse-match score of the 42-pose submap trajectory against stationary
   control points reads 1.55 m purely from temporal staleness (up to
   5.04 s), the same artifact class as exp01's ~0.89 m.

exp04 is the designated second freeze substrate and Phase 3 holdout
companion to exp01 (`docs/roadmap/v0.8.md` §5, §6) — it must **not** be
touched again once Phase 2 threshold-tuning starts (holdout hygiene,
`docs/roadmap/v0.8.md` §5 Phase 3).

## 4. Synthetic degenerate-scenario oracle

Deterministic fixture generator: `graph_based_slam/include/graph_based_slam/synthetic_degeneracy_fixtures.hpp`
(clean-room; only the published point-to-plane ICP residual/Jacobian
`r = n^T(Rp + t - q)`, `J = [n^T, (p x n)^T]` is used — no GPL reference
implementation was read). Unit tests:
`graph_based_slam/test/test_synthetic_degeneracy_fixtures.cpp` (6 cases, all
passing as of this recording — see §5).

Three scenarios (`docs/roadmap/v0.8.md` §4.3, §6), all built from a fixed
point lattice (no randomness in point placement) with optional
along-plane-normal thickness noise (fixed-seed `std::mt19937`, applied
strictly along each correspondence's own normal — this perturbs the
residual `b` but *never* the Jacobian's moment arm `p x n`, so it leaves
every `H` eigenvalue bit-identical; verified by
`AlongNormalNoiseChangesResidualNotHessian`):

- **corridor**: parallel side walls (`y = ±1 m`, 2 m wide) + floor/ceiling
  (`z ∈ {0, 2.5 m}`), extruded 20 m along `x` (0.5 m grid, 902
  correspondences). No correspondence's plane normal has an `x` component
  anywhere, so `H`'s along-corridor-translation (`tx`) row/column is
  *exactly* zero by construction — an algebraic degeneracy, not merely a
  small numeric one.
- **box**: the corridor cross-section plus two end walls (`x ∈ {0, 20 m}`,
  normal `±x`), i.e. a fully closed six-plane room (962 correspondences).
- **single_plane**: a floor only, same footprint as the corridor's own
  floor/ceiling extended to a 20 m × 2 m patch (205 correspondences); the
  open-field degenerate case (`docs/roadmap/v0.8.md` §4.3's "single-plane /
  open-field").

### Eigenvalue signatures (ascending; `H` from the default fixture configs, identity pose)

| scenario | λ0 | λ1 | λ2 | λ3 | λ4 | λ5 | classification |
|---|---:|---:|---:|---:|---:|---:|---|
| corridor | **0.000000000** | 89.563973183 | 105.715130718 | 791.640728099 | 55654.284869282 | 67363.295298718 | along-axis degenerate (λ0 exact zero, eigenvector = pure `tx`; λ1..λ5 well-conditioned) |
| box | 59.250234388 | 89.641377404 | 107.115865582 | 791.987407460 | 55791.133900030 | 67392.871215136 | fully observable (all six eigenvalues bounded away from zero; min/max condition number 1137.4) |
| single_plane | **0.000000000** | **0.000000000** | **0.000000000** | 52.857565359 | 102.500000000 | 27827.142434641 | non-observable in three directions (`tx`, `ty`, yaw/`rz` exactly zero; only `tz`, pitch/`rx`, roll/`ry` constrained) |

`H` column norms (`[tx, ty, tz, rx, ry, rz]`), the exact-zero-column proof
underlying the eigenvalue table:

| scenario | tx | ty | tz | rx | ry | rz |
|---|---:|---:|---:|---:|---:|---:|
| corridor | **0** | 4982.638758730 | 4120.449004660 | 6322.679910449 | 55501.644119792 | 66885.314531667 |
| box | 96.046863561 | 4982.638758730 | 4120.449004660 | 6322.679910449 | 55638.819912450 | 66915.105917872 |
| single_plane | **0** | **0** | 2060.224502330 | 102.500000000 | 27750.822059896 | **0** |

Reading: corridor's only zero column is `tx` (the along-corridor axis) —
matching the exp07 evidence that one specific axis, not a rotation, carries
the drift (`docs/roadmap/v0.8.md` §0). Adding the two end walls (box) makes
every column nonzero and lifts the minimum eigenvalue from exact zero to
59.25 — a clean before/after proof that closing the corridor's ends removes
exactly the one missing constraint direction. single_plane's three zero
columns (`tx`, `ty`, `rz`) are exactly the directions a single horizontal
plane cannot constrain (in-plane translation and yaw about the normal);
`rx`/`ry` (tilt) and `tz` (height) remain observable from the plane's own
points — the fixture correctly lands on "non-observable in three
directions", not a false "well-conditioned" reading, which is the specific
failure mode Phase 1's detector gate (`docs/roadmap/v0.8.md` §5 Phase 1)
must avoid.

These are the frozen oracle values Phase 1's `localizability_analysis.hpp`
classifier must reproduce (majority-of-frames along-axis-degenerate on
corridor, all-well-conditioned on box, correct non-observable — not
well-conditioned — on single_plane) before any real-substrate
classification claim is made.

## 5. Reproduction

Real substrates (exp01/exp07 measured in v0.7 Phase 4; exp04 measured
2026-07-05, exact commands and harness caveats in §3):

```bash
bash scripts/download_hilti2022.sh --sequence exp01   # or exp07
bash scripts/run_rko_lio_graph_benchmark.sh \
  --bag datasets/hilti2022/exp01_ros2 \
  --lidar-topic /hesai/pandar --imu-topic /alphasense/imu \
  --rko-param configs/hilti2022/rko_lio_hilti2022_pandar.yaml \
  --reference-tum datasets/hilti2022/exp01_construction_ground_level_gt.txt \
  --skip-reference-gen --reference-source hilti2022_exp01_control_points_gt \
  --quiescence-secs 60 --output-dir output/hilti2022_exp01_run
python3 scripts/ape_from_tum.py --interpolate --max-time-diff 3.0 \
  --ref datasets/hilti2022/exp01_construction_ground_level_gt.txt \
  --est output/hilti2022_exp01_run/traj_raw.tum --out ape.txt
python3 scripts/benchmark_summary.py --root output \
  --release-profile scripts/release_profiles.yaml --write-md summary.md
```

Synthetic fixtures + eigenvalue signatures (unit tests; no bag, no ROS
runtime needed):

```bash
colcon build --packages-select graph_based_slam --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select graph_based_slam \
  --ctest-args -R test_synthetic_degeneracy_fixtures
colcon test-result --verbose
```

Release-profile YAML regression tests (must stay green; see §6 for the two
pre-existing, unrelated failures already present before this Phase 0 pass):

```bash
python3 -m pytest -q lidarslam/test/test_benchmark_summary_profiles.py
```

## 6. Test status at recording time (2026-07-05)

- `test_synthetic_degeneracy_fixtures` (new, 6 cases): **6/6 pass**
  (`CorridorGenerationIsDeterministic`,
  `CorridorHasExactAlongAxisDegeneracy`, `BoxIsFullyObservable`,
  `SinglePlaneIsNonObservableInThreeDirections`,
  `AlongNormalNoiseChangesResidualNotHessian`,
  `EigenSignatureCanonicalizationIsDeterministic`).
- Full `graph_based_slam` gtest + pytest suite (`colcon test-result
  --verbose` after `colcon test --packages-select graph_based_slam`):
  **1378 tests, 0 errors, 0 failures, 74 skipped** — no regression from the
  new fixtures/tests or the CMakeLists wiring.
- `lidarslam/test/test_benchmark_summary_profiles.py`: **12 passed, 2
  failed**. The 2 failures
  (`test_research_track_profiles_graduated_to_blocking`,
  `test_graduated_profile_fails_gate_on_regression`) are **pre-existing**
  and unrelated to this Phase 0 change — they assert `mid360_vs_glim` has
  no `report_only_until`, which stopped being true when `mid360_vs_glim`
  was demoted to report-only under decision D-GT-2 (v0.5); this predates
  and is independent of the HILTI rows added here. Confirmed by
  running the same test file against `scripts/release_profiles.yaml`
  before this pass's edits: identical 12 passed / 2 failed split.
  Re-run after adding the `hilti2022_exp04` row (§3): still exactly
  **12 passed, 2 failed** — the exp04 row introduces no new failure.

---

(Related: `docs/roadmap/v0.8.md` — the phase-gate discipline this document
freezes metrics for; `docs/research/map-quality-baseline.md` — the v0.7
Phase 1 precedent this document's structure follows; `docs/research/project-plan-archive.md` §2.4 — the
original HILTI exp01/exp07 measurement this document reconfirms and
freezes; `docs/research/map-refinement-clean-room-design.md` — the
synthetic-fixture and clean-room discipline reused for the corridor/box/
single-plane generator.)
