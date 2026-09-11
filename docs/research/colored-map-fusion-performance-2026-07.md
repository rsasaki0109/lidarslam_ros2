# Camera-colour fusion performance (2026-07)

## Profile

Construction Seq1 full-density recolouring showed two useful operating points:

- corrected-pose baseline O processed about 173.0 million accepted camera
  samples in roughly 14 minutes;
- guarded candidate N processed about 111.6 million in roughly 9 minutes.

A one-view profile over all 4,840,318 points attributed about 46% of the robust
projection call to edge-aware sub-pixel sampling. The implementation built an
`Mx4x3` corner tensor solely to compute each query's local RGB range, after
already gathering the same four corner arrays for bilinear interpolation.

## Optimization

The optimized sampler gathers the top two corners, initializes element-wise
minimum and maximum arrays, then gathers the bottom pair and updates those
arrays in place. Corner arrays are released as soon as their interpolation row
is complete. This removes the `Mx4x3` stack and its `ptp` reduction while
preserving the same bilinear arithmetic, edge threshold, nearest-pixel fallback,
and tie behavior.

A randomized reference test compares 1,000 sub-pixel queries against the old
pairwise-corner formula exactly, including out-of-frame clamping and strong-edge
fallback.

## Results

The fixed microbenchmark used one deterministic 1080x1440 RGB image, one
million deterministic floating-point pixel coordinates, six edge-aware calls,
and the same process environment.

| implementation | median kernel | max RSS |
| --- | ---: | ---: |
| pairwise corner stack | 1.402 s | 270,092 KiB |
| in-place min/max | 0.519 s | 238,664 KiB |

Kernel time fell by 63.0% and process peak RSS by 11.6%.

The paired end-to-end screen used every tenth point of the 4,840,318-point K3
map, all 260 corrected camera poses, overlap RGB balance, view confidence,
120-pixel image margin, vignette gain 2.5, and minimum three samples. Runs were
serial on the same host.

| implementation | wall time | max RSS | coverage |
| --- | ---: | ---: | ---: |
| pairwise corner stack | 130.53 s | 1,848,356 KiB | 0.777837 |
| in-place min/max | 97.91 s | 1,848,584 KiB | 0.777837 |

End-to-end time fell by 25.0%. Whole-process RSS was unchanged because point,
normal, image, and bounded-sample arrays dominate outside the sampling kernel.
Both reports matched after removing their output paths, and the two output PLY
files had the identical SHA-256
`f3edb9e62847961340bd37ee51bd4b59d3b6562a3c9eb500177a7c98dc0f502c`.
