# Coloured-map spatiotemporal calibration (2026-07)

## Motivation

Camera-coloured point clouds turn small timing and extrinsic errors into colour
halos at walls, pillars, and object boundaries. The previous `--time-offset
auto` aligns camera and LiDAR clock domains, while the alignment evaluator's
optional correction searched only a 6DoF camera pose. Neither recomposed every
camera pose from the continuous SLAM trajectory while jointly searching the
residual clock offset.

## Architecture

`evaluate_lidar_camera_alignment.py --optimize-spatiotemporal` now optimizes
seven bounded parameters:

1. residual camera-to-trajectory time offset;
2. local optical-frame x/y/z extrinsic translation;
3. local optical-frame roll/pitch/yaw extrinsic rotation.

For every candidate it interpolates `world <- body` from the dense TUM
trajectory and composes it with the refined `body <- camera` transform. The
objective is the coverage-guarded distance from projected LiDAR depth
discontinuities to strong image gradients. Image edge masks are cached across
a deterministic image pyramid. Each coordinate sweep tests both one- and
two-step moves so pixel-quantisation plateaus do not trap the search. Bounds
that are closer than the next search step are expanded and searched again.

The user-facing pipeline is deliberately staged as:

```text
posed images -> uncoloured calibration geometry -> 7DoF calibration
             -> corrected camera poses -> final robust RGB map -> quality gates
```

The feature is opt-in. Without `--refine-spatiotemporal-calibration`, command
composition and output paths are unchanged.

## Safety and validation gates

- Held-out views are selected deterministically across travelled-distance
  segments and static/translation/rotation motion strata.
- Training loss must decrease and held-out loss must decrease independently.
- A configurable minimum number of LiDAR depth-edge pixels is required in both
  partitions.
- A nearly static trajectory is rejected because time offset is unobservable.
- Time, translation, and rotation corrections have independent hard bounds.
- Source camera poses must reproduce one static extrinsic within 1 mm and 0.05
  degrees; otherwise their timestamps/conventions are treated as inconsistent.
- Parameters closer to a search bound than the next search step are listed in
  `boundary_axes`; a production candidate cannot be promoted while any remain.
- The observability audit runs at the finest pyramid resolution. Five samples
  per axis provide one- and two-step centred curvatures; both must be positive
  and the estimated stationary point must remain local.
- Three 3x3 quadratic clock/translation surfaces must be positive definite.
  The normalized Hessian condition number and time/translation correlations
  are bounded, and covariance is emitted only for a positive-definite Hessian.
- Rejected candidates export the original camera poses, never an unvalidated
  correction, and record the rejection reason in JSON.
- Frame timestamps and original images are preserved in the corrected
  `transforms_spatiotemporal.json` dataset.

## RTK-SLAM Construction Seq1 production smoke result

The production CPU smoke used the existing K3 4.84 M-point map,
deterministically sampled to 200,000 points, 13 of 260 camera views, pyramid
scales 0.25/0.5/1.0, and two search rounds per scale. Nine views were training
data and four were held out by the travelled-distance/motion split. Initial
bounds were 40 ms / 4 cm / 0.5 degrees and expanded only for axes within one
search step of a bound.

| metric | initial | refined |
| --- | ---: | ---: |
| training mean edge distance | 6.539 px | 5.937 px |
| training out-of-range fraction | 0.250 | 0.217 |
| held-out mean edge distance | 6.522 px | 6.010 px |
| held-out median edge distance | 6.000 px | 5.000 px |
| held-out out-of-range fraction | 0.241 | 0.216 |

Training and held-out mean loss improved by 9.20% and 7.85%, respectively. No
axis remained near its final bound. All seven normalized Hessian eigenvalues
were positive (0.0369 to 0.0827), the condition number was 2.24, and maximum
absolute clock/translation correlation was 0.105. The correction therefore
passed the production acceptance gate and was exported. This validates the
calibration stage on Seq1; independent Seq2 and cross-rig promotion remain a
separate dataset-level gate.

The regression suite covers deterministic pattern search, step-aware bounds,
multi-scale curvature and undefined-correlation handling, stratified splitting,
continuous-time pose recomposition, static-motion degeneracy, independent
held-out rejection, pipeline staging, cache reuse, and unchanged opt-in command
composition.

## K5 residual diagnostics

An aggregate edge-distance score can hide whether a poor map is caused by one
constant camera correction or by view-dependent timing and pose errors. The
alignment evaluator therefore has an optional diagnostic output:

```bash
python3 scripts/evaluate_lidar_camera_alignment.py \
  --pointcloud coloured.ply --transforms posed/transforms.json \
  --out alignment.json --diagnostics-dir alignment_diagnostics \
  --worst-views 10
```

Each selected view records the median signed x/y displacement from projected
LiDAR depth edges to their nearest strong image edges. The diagnostic directory
contains JSON, the worst-view overlays, and a contact sheet. Image edges are
green; LiDAR edges progress from cyan through yellow to red as residual grows,
and unmatched edges are magenta. A stable signed direction across views points
to a static extrinsic error. Large changes between views instead point to clock,
motion distortion, rolling shutter, or trajectory error. These images are
diagnostic evidence, not a replacement for independent held-out acceptance.

The first full-resolution K4 audit used all 4,906,133 geometry points and 26
views. The ten worst views had 36.5% to 54.2% unmatched depth edges, despite a
12 px search radius. Across all matched edges, the weighted direction was only
(-0.027, -0.126) px and direction coherence was 0.032. The overlays show broad,
scene-dependent residuals on shelves, ceilings, and object boundaries rather
than one consistent translation. This rules out treating K4 as a simple static
extrinsic nudge. Timing, motion distortion, and trajectory-conditioned residuals
must therefore be tested independently.

### Surface-supported edge ablation

The evaluator also provides an opt-in same-surface support filter. A projected
depth-edge pixel is retained only when nearby finite depths agree within an
absolute and range-relative tolerance. Reports always include the raw edge
count and retained fraction so filtering cannot improve a score merely by
discarding difficult observations. Calibration additionally supports a minimum
retained-fraction rejection gate; the pipeline uses 25% when this filter is
enabled.

The full-density K4 `radius=2, min_neighbors=4` ablation retained 51.42% of raw
edges. Median residual improved only from 7.759 to 7.234 px, 2 px inliers from
22.31% to 23.12%, and out-of-range residuals from 34.96% to 32.79%; p90 remained
saturated at 13 px. With the production 300,000-point calibration subsample,
only 7.62% survived. Its apparently lower 4.59 px median is selection bias and
fails the 25% retention gate. This filter is useful for visual diagnosis but is
not a K5 calibration candidate. The next objective needs correspondence support
that remains meaningful under sparse geometry rather than image-plane density
alone.

### Fixed full-density 3D contours

The next candidate extracts visible depth-edge winner IDs from the complete
4.91 M-point geometry before applying `--max-points`. Each view retains a
deterministically image-distributed cap of those world-space points. During
7DoF search the same 3D points are reprojected for every candidate, so neither
the 300,000-point calibration subsample nor candidate-dependent edge detection
can change the objective's population. Pipeline support is opt-in through
`--calibration-fixed-contours` and `--alignment-fixed-contours`.

Image-associated contour selection is diagnostic-only. An ablation selecting
points initially within 12 px of an image edge made zero correction the exact
optimum: both training and held-out loss changed by 0%. The CLI now rejects
that mode during optimization. Production calibration requires geometry-only
selection (`--contour-association-distance 0`).

A geometry-only smoke used 13 stratified views and 20,000 fixed contour points
per view. All 260,000 points survived the 300,000-point calibration condition.
The candidate reduced training loss by 2.25%, but held-out loss by only 0.76%
(7.3723 to 7.3160 px), below the required 2%. The observability audit also
rejected a stationary point outside the local neighbourhood. No corrected pose
was adopted. This establishes density-independent evidence and safe rejection,
but does not yet improve K4 colour registration; fixed nearest-image-edge
distance remains too weak and ambiguous in the cluttered warehouse.

### Per-pixel orientation-aware correspondence

Fixed contours can optionally carry the unit normal of their originating depth
discontinuity. `--orientation-max-angle-deg` then requires the unoriented depth
normal and image-gradient normal to agree as well as satisfying pixel distance.
Angle-rejected contours remain saturated residuals rather than disappearing
from the population. The option is default-off and requires fixed contours.

The 30-degree production smoke retained all 260,000 fixed contour points but
left 61.24% without a valid match. Training loss improved only 1.09% and
held-out loss 0.52%, both worse than distance-only contours. The solution also
reached search bounds and failed observability because of insufficient and
unstable curvature, an unobservable time/translation pair, an out-of-local
stationary point, and ill-conditioning. It was rejected and original poses were
exported.

Pixel normals from a sparse depth raster are not stable enough around shelves,
corners, and thin structures. The next candidate should group contour pixels
into supported line segments and estimate one robust tangent per segment,
rather than loosening this per-pixel gate until it becomes distance-only again.
