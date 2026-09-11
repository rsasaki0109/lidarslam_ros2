# Geometry-aware camera-colour fusion (2026-07)

## Problem

Robust RGB medoids, exposure balancing, and view confidence improve the colour
chosen from valid camera observations. They cannot repair a candidate set that
already contains the wrong surface: a background LiDAR point projected beside
a foreground silhouette, either side of a depth discontinuity, or a moving
object observed in only part of the sequence. Calibration uncertainty also
makes a fixed pixel guard inconsistent across range and vehicle motion.

## Architecture

The robust projector now constructs a full-resolution per-view z-buffer and
applies four opt-in guards before colour sampling:

1. **Occlusion silhouette:** compare projected depth with the minimum finite
   depth in a circular pixel neighbourhood.
2. **Depth edge:** reject both sides when local finite depth range exceeds an
   absolute plus range-relative threshold.
3. **Dynamic image mask:** reject a pixel when any excluded mask pixel lies in
   its circular neighbourhood.
4. **Calibration uncertainty:** convert accepted 7DoF uncertainty into a
   per-observation pixel radius and add it to enabled geometry guards. The
   translation term scales by focal length over range. Rotation is range
   independent. Residual time uncertainty is multiplied by the camera's local
   linear and angular speeds before projection.

The final radius is configurable and clipped. Larger uncertainty also lowers
the observation quality used by bounded sample selection. The previous robust
fusion path is byte-compatible at default values because all new margins and
the uncertainty multiplier are zero at the library boundary.

## Dynamic-mask contract

Segmentation inference is deliberately outside the mapping core. The adapter
accepts PNG files named after each posed image stem, validates their dimensions,
and writes `dynamic_mask_path` into each frame. Non-zero means excluded. It also
records `dynamic-image-mask-v1` provenance with frame coverage, masked-pixel
fraction, and SHA-256 for every attached file.

The production pipeline stages data as:

```text
posed images -> validated dynamic-mask manifest -> optional 7DoF calibration
             -> geometry-aware robust RGB map -> quality reports
```

Calibration consumes the mask-attached transforms document and preserves the
frame metadata in its corrected output. Cache invalidation includes the source
transforms, mask directory, and individual PNG modification times. Dynamic
exclusion requires a complete mask set; partial manifests are allowed only for
non-exclusion dataset preparation.

## Safety and diagnostics

- Geometry-aware fusion requires a one-pixel z-buffer; coarse bins are rejected.
- Negative radii, thresholds, and uncertainty multipliers are rejected.
- Uncertainty propagation requires finite, strictly increasing frame timestamps
  and an accepted calibration with seven finite non-negative uncertainty values.
- Rejected calibration metadata is never silently used.
- Per-run diagnostics count projected samples, same-pixel z-buffer rejects,
  additional silhouette-margin rejects, their aggregate occlusion rejects,
  depth-edge rejects, dynamic-mask rejects, and accepted samples.
- Existing colour options and output selection are unchanged unless explicitly
  enabled.

## Validation status

Unit and integration coverage includes variable-radius depth neighbourhoods,
silhouette rejection, two-sided edge rejection, mask dilation, timestamped
camera motion, uncertainty expansion, manifest hashes and completeness, loader
metadata retention, end-to-end mask consumption, pipeline ordering, cache
invalidation, and legacy opt-in behavior. Production threshold selection and
real-data A/B promotion are intentionally the next dataset-level stage.

## Construction Seq1 guard screening

The first full-density candidate used 2 px silhouette and depth-edge margins
with 0.15 m + 2% discontinuity tolerance. It was rejected: coverage fell from
the K3 reference 0.7476 to 0.3519, held-out RGB L2 median rose from 41.17 to
51.87, and planar roughness median rose from 7.23 to 10.79.

Subsequent candidates used the same deterministic every-tenth-point subset of
the 4,840,318-point K3 map and the accepted corrected camera poses. This is a
paired parameter screen, not a substitute for full-density promotion.

| candidate | guard | coverage | held-out median | inlier 20 |
| --- | --- | ---: | ---: | ---: |
| S0 | corrected-pose baseline | 0.77784 | 43.372 | 0.26297 |
| S1 | silhouette 1 px | 0.75823 | 43.181 | 0.26489 |
| S2 | edge 1 px, 0.5 m + 5% | 0.75653 | 42.922 | 0.26758 |
| S3 | S1 + S2 | 0.74858 | 42.846 | 0.26862 |
| S4 | edge 1 px, 1.0 m + 10% | 0.76650 | 42.958 | 0.26659 |
| S5 | S4, minimum 2 samples | 0.82875 | 42.924 | 0.26717 |
| S6 | edge 1 px, 2.0 m + 20% | 0.77313 | 43.013 | 0.26563 |

S3 had the best held-out score but missed the 0.75 screening coverage floor.
S4 retained about one additional coverage point over S2 for nearly the same
held-out improvement and was promoted to a full-density trial. Full-density S4
improved held-out median from the K3 reference 41.17 to 40.04 and inlier 20 from
0.2863 to 0.2995, but coverage fell to 0.7060 and planar roughness worsened from
7.23/23.89 to 8.27/25.01. It was rejected.

S5 tests whether the geometry guard makes the previous three-observation floor
unnecessarily strict. It recovered screening coverage above the corrected-pose
baseline while retaining most of S4's held-out improvement. S6 traded less
coverage for less improvement and was rejected. S5 was promoted to the final
full-density trial.

Full-density S5 (candidate N) recovered coverage to 0.7712 and further improved
held-out median/inlier 20 to 39.94/0.3007. Global roughness was 5.75/24.61 and
chroma retention was 0.9964, but planar roughness was 8.67/28.02 and therefore
failed the existing 7.6/25.0 profile. N cannot be promoted against K3 alone
because it also includes the newly accepted camera calibration. A full-density
corrected-pose baseline without geometry guards is required to separate the
calibration effect from the guard effect.

That baseline (candidate O) measured coverage 0.7408, held-out median/inlier 20
40.23/0.2928, global roughness 5.20/20.33, and planar roughness 7.72/22.65.
Compared directly with O, N improved coverage and held-out colour fidelity but
worsened planar roughness to 8.67/28.02. The boundary guard was therefore not
promoted and the existing quality threshold was not relaxed. CLI boundary
margins default to zero; dynamic masks, uncertainty propagation, and explicitly
validated dataset-specific margins remain available independently.
