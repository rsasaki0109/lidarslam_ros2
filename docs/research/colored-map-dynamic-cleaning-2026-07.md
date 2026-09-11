# Pose-aware dynamic cleaning for camera-coloured maps (2026-07)

## Decision

Construction Seq1 candidate K4 is promoted over K3 for the README media. K4
uses `dynamic-object-removal` 0.5.0 `fusion` on the accumulated LiDAR geometry,
then applies the unchanged K3 camera-colour settings. The cleaner is optional
and default-off. Dataset-independent defaults are not claimed by this result.

## Placement and provenance

The offline pipeline records every deskewed scan after transforming it with the
same RKO-LIO trajectory used to accumulate the map. Each evidence item contains
world-frame scan points and its world-frame sensor origin. Cleaning runs after
the cross-scan voxel merge and before the final point cap, density filter, and
camera projection:

```text
deskew + pose -> per-scan voxel -> accumulated geometry
              -> pose-aware dynamic cleaning -> density filter -> RGB fusion
```

The optional dependency is imported only when a cleaner is selected. The JSON
report records algorithm and implementation version, evidence scan count,
worker count, thresholds, and input/kept/removed counts. The old path returns
the same point object when the feature is disabled.

## Frozen paired run

Both branches used 639 deskewed `/livox/points` scans from seconds 480--545,
the same corrected TUM trajectory, 0.015 m map voxel, 1.5--60 m range, 5 M point
cap, and identical density filter. Rebuilding the baseline produced 4,840,318
XYZ points byte-for-byte equal to K3's geometry. K4 used every fifth scan as
cleaning evidence (128 scans), four workers, free-vote fraction 0.9, free-vote
floor 2, and void minimum 11 scans.

The cleaner classified 884,580 of 6,193,579 pre-cap points (14.28%) as dynamic
evidence. After the common cap and density filter K4 contains 4,906,133 points.
On a fixed 750,000-point nearest-neighbour sample, 85.89% of K3 points are
within 3 cm of K4 and 94.33% of K4 points are within 3 cm of K3. This is a
structure-support proxy, not point-wise dynamic-object ground truth.

## Full-density promotion gate

K4 was recoloured from the original K3 camera poses with image margin 120,
vignette gain limit 2.5, overlap RGB balance, view confidence, 0.12 m normal
voxel, view-score power 1, and minimum three observations.

| metric | K3 | K4 | limit |
| --- | ---: | ---: | ---: |
| alignment median px | 7.477 | 7.759 | <= 8.0 |
| alignment inlier 2 px | 0.2271 | 0.2231 | >= 0.20 |
| held-out RGB median | 41.173 | 40.539 | <= 42.0 |
| held-out inlier 20 | 0.2863 | 0.2909 | >= 0.28 |
| colour coverage | 0.7476 | 0.7666 | >= 0.70 |
| roughness median / p90 | 5.43 / 20.67 | 5.20 / 19.98 | <= 6 / 22 |
| planar roughness median / p90 | 7.23 / 23.89 | 6.40 / 23.52 | <= 7.6 / 25 |
| chroma retention | 1.0016 | 1.0051 | >= 0.90 |

All 11 report-only profile checks pass. Eight paired cinematic views were also
inspected with the same camera path, display voxel, splat, and point-size
settings before replacing the README media.

## Limits

Construction Seq1 has no point-wise moving/static labels, so this result does
not measure removal precision or recall and does not prove that every removed
point was dynamic. Promotion is based on same-pose provenance, exact baseline
reconstruction, bidirectional structure support, improved colour/roughness
scores, and visual review. Other sensors and scenes must repeat the paired raw
versus cleaned evaluation; point-count reduction alone is never sufficient.

