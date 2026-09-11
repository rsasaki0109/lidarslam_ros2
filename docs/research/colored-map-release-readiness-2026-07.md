# Camera-coloured map release readiness (2026-07)

## Decision

The camera-coloured map work is ready to ship as an opt-in, quality-gated
pipeline. The selected public example is now the K4 Construction Seq1 map,
which adds opt-in pose-aware dynamic cleaning to the accepted K3 colour path.
Spatiotemporal calibration, dynamic masks, calibration-uncertainty propagation,
and dataset-specific geometry margins are available without changing legacy
defaults. Geometry boundary margins are not promoted for Construction Seq1.

This distinction matters: the K4 artifact passed paired full-density
validation, while the cleaner remains explicit and default-off for other
datasets. The release does not exchange surface consistency for a better
aggregate colour score.

## Selected map and media

The K4 map contains 4,906,133 points. Of those, 3,761,250 receive camera colour
for recolour coverage 0.766642. It uses pose-aware `fusion` cleaning followed
by overlap RGB balancing, view confidence, a
120-pixel image margin, vignette gain limit 2.5, and at least three camera
observations. The report-only Construction Seq1 profile has 11 checks and no
violations:

- held-out RGB L2 median 40.54 (limit 42.0) and inlier-20 fraction 0.2909
  (minimum 0.28);
- global roughness median/p90 5.20/19.98 (limits 6.0/22.0);
- planar roughness median/p90 6.40/23.52 (limits 7.6/25.0);
- chroma retention 1.0051 and appearance coverage 0.7666.

The README animation is rendered from K4 with the CPU surface-splat and
cinematic-camera path. The 800x600 supersampled source has temporal-flicker p90
0.00524, then is Lanczos-downsampled to the existing README dimensions. Six
points across the final encoded loop and eight paired K3/K4 source views were
visually checked.

| asset | dimensions | rate | frames | duration |
| --- | ---: | ---: | ---: | ---: |
| `map_flythrough_rtkslam.mp4` | 600x450 | 30 fps | 240 | 8 s |
| `map_flythrough_rtkslam.gif` | 480x360 | 10 fps | 80 | 8 s |
| `map_flythrough_rtkslam.webp` | 600x450 | 15 fps | 120 | 8 s |

WebP is the README primary; MP4 and GIF are linked fallbacks. The source
sequence is RTK-SLAM Construction Hall 1, distributed under CC-BY 4.0.

## Architecture and promotion evidence

1. The 7DoF spatiotemporal calibration recomposes camera poses from the dense
   trajectory and accepts a correction only when training and held-out edge
   loss improve, bounds and observability pass, and covariance is valid. The
   Seq1 smoke improved training/held-out mean loss by 9.20%/7.85%.
2. Geometry-aware fusion applies z-buffer silhouette, depth-edge, dynamic-mask,
   and uncertainty-aware guards before robust colour selection. Diagnostics
   retain separate rejection causes.
3. Full-density candidate N improved coverage and held-out colour relative to
   its corrected-pose baseline O, but worsened planar roughness to 8.67/28.02.
   It was rejected against the unchanged 7.6/25.0 limits. Boundary margins
   therefore remain zero by default.
4. Edge-aware sampling no longer materializes an `Mx4x3` corner stack. The
   one-million-coordinate kernel is 63.0% faster with 11.6% lower process RSS;
   a paired 484k-point/260-image screen is 25.0% faster with an identical
   report and byte-identical PLY SHA-256.
5. K4 supplies world-frame deskewed scans and same-pose sensor origins to
   `dynamic-object-removal` 0.5.0. Its 128 evidence scans remove 14.28% before
   the common cap/filter. The reconstructed K3 baseline is XYZ-identical, K4
   passes all 11 checks, and its planar roughness improves rather than trading
   static surface consistency for point-count reduction.

Detailed evidence:

- [spatiotemporal calibration](colored-map-spatiotemporal-calibration-2026-07.md)
- [geometry-aware fusion and paired A/B](colored-map-geometry-aware-fusion-2026-07.md)
- [performance and output equivalence](colored-map-fusion-performance-2026-07.md)
- [pose-aware dynamic cleaning and K3/K4 comparison](colored-map-dynamic-cleaning-2026-07.md)

## Compatibility and safe defaults

All new correction and rejection behavior is opt-in. Without the corresponding
flags, pipeline staging, output selection, sampling behavior, and public CLI
defaults remain compatible. The geometry path requires a one-pixel z-buffer;
dynamic exclusion requires a complete, dimension-checked mask set; uncertainty
propagation requires an accepted calibration with valid seven-parameter
uncertainty and strictly increasing timestamps.

The following are not release claims:

- automatic semantic segmentation (the core consumes external PNG masks);
- point-wise dynamic-removal precision/recall on Construction Seq1, which has
  no moving/static ground-truth labels;
- universal geometry-margin values across cameras, rigs, or datasets;
- independent Seq2 or cross-rig calibration promotion;
- a completed world-map quality result from the camera-only realtime bag,
  which did not contain a finished map topic.

## Verification

- dynamic-cleaning/pipeline focused Python suite: 102 passed;
- full local `graph_based_slam/test`: 1,255 passed, 13 skipped; two source-only
  RKO-LIO checks were unavailable because this worktree's submodule is not
  initialized;
- focused coloured-map Python suite: 134 passed, 4 skipped;
- ament flake8: passed;
- paired output-equivalence benchmark: passed;
- GitHub CI: Humble, Jazzy, docs/release metadata, release readiness, and the
  negative threshold guard passed on run `29662475090`.
