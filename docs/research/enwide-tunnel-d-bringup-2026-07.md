# ENWIDE TunnelD public benchmark bring-up (2026-07-29)

## Decision

The first public ENWIDE TunnelD run is a successful pipeline bring-up and a
failed accuracy baseline. It is not valid evidence for a SOTA claim.

Keep the preregistered geometry/intensity v1 configuration as the reproducible
baseline. Reject the exploratory v2 intensity-disagreement candidate: it
improves global ATE but worsens local RTE, runtime, memory, and estimated path
length.

The first alias-aware reflectivity milestone is implemented in RKO-LIO commits
`b4a8937ab13bbb3dfbffe76365752c77bcdca678` and
`415375106ef0bc706e307ae507d47d9970b6d0ba`. Raw peak observations are exported
by `f285c9e97e6e4425a1b8cf2d5891624448922d8e` and made policy-faithful by
`9579b775b82daf19b764041564661b6b51a3cc96`. The one-dimensional NCC result
now reports a best-versus-second-peak margin, can reject ambiguous peaks, and
records aggregate margin diagnostics. Its default margin is zero, preserving
the historic result until a threshold is selected from diagnostics rather
than TunnelD accuracy.

Peak selection is a sensor- and dimensionality-independent core API:
correlation implementations emit scored offset candidates, while a reusable
policy selects the best supported candidate, excludes the configurable local
peak radius, and reports an explicit rejection reason. The current 1D profile
is an adapter over that API. A later local oriented reflectivity/height
matcher can reuse the selector without adding another gate directly to
`LIO::register_scan`.

Every attempted correlation now emits timestamp, source, best and second-best
score, margin, overlap support, base qualification, ambiguity, and acceptance
to `intensity_peak_diagnostics.csv`. Schema v4 also records the texture- and
ICP-implied longitudinal/lateral velocities, their disagreement magnitude,
streak length, candidate correction magnitude, applied correction vector, and
whether a correction was applied. Base qualification records whether score
and support passed before the alias-margin policy is applied, so low-quality
candidates cannot contaminate the alias distribution.

The selection-independent summarizer consumes one or more of these CSVs and
writes input paths and SHA-256 hashes, qualified row/source counts, fixed
quantiles, and fixed threshold counts:

```bash
python3 scripts/summarize_intensity_peak_diagnostics.py \
  run-a/intensity_peak_diagnostics.csv \
  run-b/intensity_peak_diagnostics.csv \
  --output intensity_peak_summary.json
```

It has no accuracy-metric input. Benchmark analysis can therefore aggregate
repetitions or holdouts and test a future selection policy from raw evidence
without replaying the multi-gigabyte bags or selecting against TunnelD ATE.

The 20-second Tunnel smoke test produced 83 raw observations and 25
base-qualified observations. Its qualified median margin was 0.1465; 20% were
below 0.05. This validates the dump and summarizer only. The sample is too
short and too sequence-specific to select a production threshold.

The first threshold-selection diagnostic attempt ran the full NTNU tunnel
once and the distinct fog sequence three times, without computing accuracy:

| sequence | runs | qualified | p01 | p50 | below 0.005 |
| --- | ---: | ---: | ---: | ---: | ---: |
| tunnel | 1 | 1,322 | 0.00628 | 0.1756 | 0.91% |
| fog | 3 | 732/run | 0.05459 | 0.4318 | 0.00% |

All three fog diagnostic CSVs were byte-identical, with SHA-256
`4958c9dbce9a1d3b9726ff4765ddd9f38c80f175adb73eb431b6fe5741479e64`.
The tunnel CSV SHA-256 was
`b8ab4bb3e56ed0d58e03da478422b8b69b57216cdbe51a696570f52cf143aa40`.

That attempt exposed a diagnostic fidelity bug during the first post-freeze
accuracy run. Peak policy used unclamped profile scores, but the CSV clamped
best and second-best independently to `[-1, 1]` before recomputing the
reported margin. Partial-overlap profile scores can exceed one, so 81 accepted
rows appeared to be below the active margin. The trajectory policy was
correct; the exported evidence was not faithful to it.

The `0.005` candidate in
`configs/enwide/rko_lio_os0_intensity_alias_v3.yaml` remains as an immutable
record of the attempted preregistration, but its threshold evidence is
invalidated and it is not a production candidate. Its single TunnelD run
reported 21.6422 m ATE and 65.8713% RTE, which also fails the local-accuracy
criterion.

Diagnostic schema v2 exports the exact raw policy scores and margin plus an
explicit `has_competing_peak` flag. Offline margin distributions include only
base-qualified observations with a competing peak. Threshold selection must
be repeated on schema-v2 data before another nonzero candidate is evaluated.

The completed schema-v2 full-sequence diagnostic set, still with margin zero
and without accuracy scoring, produced:

| sequence | runs | qualified/run | competing/run | p01 | p50 | below 0.005 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| NTNU tunnel | 3 | 1,311 mean | 1,307 mean | 0.00532 pooled | 0.1761 pooled | 0.94% |
| NTNU fog | 3 | 732 | 699 | 0.05436 | 0.4246 | 0.14% |

The tunnel diagnostic SHA-256 hashes are
`0be8c4a06f5bc2a2b4b1cea00d36fab52c97fd431bc6fe50144e6d1a33a70074`,
`1d9839d47ed45193cdd2aa4e0238887d004f6fa2195259c5029ff6328442f2a9`,
and
`3bc95b08e73737ec4a30c51d96938601eb9fc20ae8df03fe1b8364c4cd59ef40`.
The fog diagnostic SHA-256 is
`3ba265faa3220ec524c84cd7b3c42774a07ed7fa45912743ba0f9cb3da9ccfa6`
for all three runs. The three tunnel CSVs differ,
but their per-run p01 range is narrow (`0.00528` to `0.00648`) and their
fraction below `0.005` ranges from `0.92%` to `0.99%`. Across all six runs,
6,019 observations have a competing peak, pooled p01 is `0.00830`, and
`0.66%` are below `0.005`.

Candidate v4 is frozen at `0.004` before its accuracy is evaluated. The
selection rule is fixed and accuracy-independent: take 80% of the minimum
per-run p01 across the six schema-v2 diagnostics and round down to 0.001.
This leaves a safety margin below the worst observed first percentile and
does not reuse the already-evaluated v3 threshold. The candidate file is
`configs/enwide/rko_lio_os0_intensity_alias_v4.yaml`; only the margin differs
from exploratory v2.

## Frozen public input

- sequence: ENWIDE `tunnel_d`
- source bag bytes: `7485669675`
- source bag SHA-256:
  `afa448cd2ee32921cd514bb7d4c2e139f642bb164f66b1d556cf48c0c798406e`
- ground-truth SHA-256:
  `25c7a20513b3c41e7a5f517119ff41bcf07329b6d87f3aeb8f5ed7725f5c922e`
- converted rosbag2 tree SHA-256:
  `4faf394304f087af616debd86140fc8bcadeb06426f3d8e612302286ddec34ec`
- duration: `119.013327769 s`
- messages: 1,189 PointCloud2 and 11,861 IMU
- position ground-truth path length: `179.707055 m`

The source PointCloud2 layout matches the runner contract: relative `t` is
`uint32` nanoseconds, `reflectivity` is `uint16`, and the frames are
`os_sensor` and `os_imu`.

## Results

The first three rows are single-run exploratory results on the same machine.
V4 is the first completed three-repetition candidate.

| candidate | ATE RMSE (m) | 10 m RTE (%) | matched GT | RTF | peak RSS (MiB) |
| --- | ---: | ---: | ---: | ---: | ---: |
| preregistered v1 | 23.7841 | 64.5694 | 99.579% | 2.421 | 454.7 |
| intensity disagreement v2 | 22.0945 | 65.2899 | 99.579% | 2.780 | 642.8 |
| v2 + alias diagnostics, margin disabled | 22.5956 | 64.6661 | 99.579% | 2.765 | 513.4 |
| schema-v2 alias v4, three-run median | 20.8877 | 65.1782 | 99.579% | 2.750 | 614.1 |
| overlap-local Pearson v5, early-stop run | 22.5613 | 63.9480 | 99.579% | 2.678 | 512.8 |
| oriented reflectivity/height grid v6, early-stop run | 22.3123 | 63.7724 | 99.579% | 2.643 | 642.9 |

V4 completed all three runs. ATE ranged from 20.3902 to 20.9202 m, while RTE
ranged from 63.3531% to 66.7671%. It rejected 17 or 18 ambiguous shifts and
corrected 683 to 693 scans per run. The summary receipt SHA-256 is
`582568422f9621a8d7a65fb771f54155169496156046c25045ef85253e918b8d`.
The lower ATE is encouraging, but the large local error and run-to-run RTE
spread reject v4 as a production or SOTA candidate.

V4 analysis exposed a deeper matcher error: the reported "NCC" standardized
each complete profile once, then averaged products over a shift-dependent
partial overlap. The overlap subset does not retain zero mean and unit
variance, so scores could exceed one and favor a shift for distribution
changes rather than texture agreement.

RKO-LIO `5c09ba20ab881158d779801daa9eeaa79949ca45` replaces that calculation
with Pearson correlation normalized inside each candidate overlap. Scores are
bounded to `[-1, 1]`, and zero-variance overlaps are rejected. The generic
peak selector and LIO gate API are unchanged. A 20-second Tunnel smoke test
completed successfully; its observed score range was 0.314 to 0.953 and
base-qualified observations dropped from 25 to 19. Candidate v5 freezes the
same v2 parameters with margin zero in
`configs/enwide/rko_lio_os0_intensity_pearson_v5.yaml`.

V5's first TunnelD run accepted 961 base-qualified correlations and corrected
673 scans. Its RTE is 1.23 percentage points below the v4 median, but its ATE
is at least 1.64 m above every v4 run. The pre-run early-stop rule required
both metrics to improve before spending two more full runs, so v5
is rejected as an accuracy candidate after one run. The bounded Pearson
calculation remains as a matcher-correctness fix; this result rejects the
current 1D disagreement strategy, not overlap-local normalization.

The next matcher core is implemented in RKO-LIO
`b928368948136a7d17c09c725cf4963a4c23036b`. It builds a world-oriented
longitudinal/lateral grid with mean reflectivity and height channels, scores
each 2D translation using the shared overlap-local Pearson accumulator, and
passes candidates through the same peak policy. Peak exclusion now understands
up to three offset coordinates while preserving the 1D API.

Synthetic tests recover a known 2D translation, use height to disambiguate
periodic reflectivity, and reject collinear axes. Core tests pass 56/56 and the
ROS package builds. This commit is intentionally matcher-only: it does not yet
alter `LIO::register_scan` or any default parameter. The next milestone is a
default-off LIO adapter and raw holdout diagnostics before accuracy evaluation.

RKO-LIO `84a2007e881a5e7c6148af24157c5ac91c544574` connects that matcher to the
existing intensity-disagreement gate behind `intensity_oriented_grid=false`.
It reuses all existing longitudinal, score, margin, support, streak, and blend
parameters; only half-width, lateral search range, and height weight are new.
The 1D and 2D stored states are separate and mutually reset.

With the adapter enabled, the 20-second Tunnel smoke test produced 84
oriented-grid attempts, all base-qualified and valid, with 78 vector
corrections. Peak margin ranged from 0.0846 to 0.3311. The run completed in
7.65 seconds and exported `source=oriented_grid`. Candidate v6 freezes the
default-off adapter parameters in
`configs/enwide/rko_lio_os0_oriented_grid_v6.yaml`. Full non-accuracy
tunnel/fog diagnostics are required before any public accuracy run.

RKO-LIO `db691a390fda76f7887c004d24f18492303c6305` extends diagnostic schema v3
with longitudinal and lateral shift values. The full fog diagnostic recorded
1,653 accepted grid shifts and 1,243 corrected scans. Median shifts were
−0.0099 m longitudinal and approximately zero lateral. Only one lateral result
reached the search boundary, and the runtime saturation guard excluded it
from correction. Peak margin p01 was 0.0815. This validates grid peak behavior
in fog but also shows a high correction duty cycle; a full tunnel diagnostic
is required before accuracy can be consumed.

The full tunnel schema-v3 diagnostic then recorded 3,082 accepted and valid
grid shifts with no search-boundary result. Median shift was −0.0092 m
longitudinal and −0.00064 m lateral; observed ranges were −0.275 to 0.248 m
and −0.090 to 0.137 m respectively. The gate corrected 2,025 scans. Runtime
was 414.1 seconds, 4.9% above the comparable 1D diagnostic. The raw diagnostic
SHA-256 is
`5fe8e268dd142f77b73254e26a3b15c71a4babf2de14aed51a1f5671ad202570`.

Together with the fog run, these results satisfy the pre-accuracy diagnostic
gate: both sequences complete, shifts remain well inside the search window,
and the added runtime is bounded. V6 may proceed to one public TunnelD
early-stop run using its already-frozen parameters.

The v6 public early-stop run accepted 1,170 grid shifts, rejected four
search-boundary results from correction, and corrected 1,001 scans. Unlike
the NTNU diagnostics, ENWIDE reached both longitudinal and lateral search
limits; the saturation guard operated as intended. V6 improves both metrics
over v5 but does not beat v4's best run (20.3902 m ATE and 63.3531% RTE), so
the pre-run rule stops evaluation after one repetition. The reusable 2D
matcher remains, default-off, as an extension boundary; the current
high-duty-cycle disagreement policy is rejected as a SOTA candidate.

The next analysis step is therefore policy diagnosis rather than another
TunnelD parameter sweep. RKO-LIO
`fac09a7b8cb70a0b4c3c64a84128bc0f2827415c` adds schema-v4 diagnostics that
make correction duty cycle and the signed longitudinal/lateral velocity gaps
observable on holdouts. They also distinguish the correction proposed by the
configured weight from the correction actually applied after the persistence
gate. This evidence is intended to support a bounded, information-weighted
fusion policy without using public TunnelD accuracy to select its thresholds.

Three schema-v4 fog repetitions measured 1,653 usable velocity disagreements
per run. Correction duty cycle was 67.70%, 70.84%, and 69.51% despite
byte-distinct outputs, confirming that the high activation rate is robust to
the known parallel nondeterminism. Across the three runs, median velocity
disagreement was 0.543 m/s and median applied correction was 0.0697 m per
scan; applied-correction p99 was 0.209 m and the maximum was 0.312 m. The
signed longitudinal correction was strongly negative (median −0.0545 m per
corrected scan). Input SHA-256 hashes are recorded in
`intensity_grid_v6_diag_schema_v4/fog_summary.json`; no accuracy metric was
consumed.

One full tunnel schema-v4 run then measured 3,106 usable disagreements and
2,066 corrections, a 66.52% duty cycle. Median velocity disagreement was
0.543 m/s, median applied correction was 0.0738 m, p99 was 0.279 m, and the
maximum was 0.437 m. Its diagnostic SHA-256 is
`d29945ae21e47aef282012b356b33c23019e66ef4f23209d07683b4e248fec6c`.
The fog and tunnel correction distributions are therefore too similar for a
velocity-gap threshold or fixed correction cap to classify whether texture
is physically trustworthy. Temporal smoothness does not rescue this policy:
fog correlations are stronger and smoother than tunnel correlations, so a
generic temporal-confidence gate would preferentially trust the harmful fog
signal.

The next matcher diagnostic must expose the intensity and height channel
scores separately at the selected 2D peak. A fixed scene should retain
geometric height consistency while aerosol reflectivity may not. Until that
hypothesis is tested on both holdouts, the oriented-grid correction remains
default-off and no new correction threshold is justified.

RKO-LIO `add71ee46322a09139fa95e187d2816ed2c36295` implements that diagnostic
boundary. The matcher result now retains the intensity and height Pearson
scores at the selected integer-bin peak while its existing combined score
continues to drive selection and sub-bin refinement. CSV schema v5 exports
both channels, and the selection-independent summarizer reports their
individual distributions and `height - intensity` gap. Missing or
zero-variance channels are counted explicitly rather than treated as valid
correlations.

Schema-v5 holdout runs computed both channel scores for every qualified
oriented-grid row: 1,652/1,652 in fog and 3,108/3,108 in tunnel. Fog's median
intensity and height correlations were 0.966 and 0.929; tunnel's were 0.912
and 0.958. Consequently, median `height - intensity` was −0.024 in fog and
+0.014 in tunnel. The diagnostic SHA-256 hashes are
`d2b83b8ce320c1cbdab311373c5cbbea71b7bccb295e3eb099284379bf590920`
for fog and
`c84fbdba29925d2abac45ae4f6ff1474b81bc9862a58a5b021597343fdab20b7`
for tunnel.

The median sign is promising only as a dataset-level observation, not as an
online safety classifier. With fixed non-overlapping 50-scan windows, a
positive median channel gap occurs in 87.1% of tunnel windows but also 30.3%
of harmful fog windows. Longer windows do not remove that false-positive
rate. A channel-gap gate would therefore still authorize substantial harmful
fog correction, so the hypothesis is rejected for automatic activation. No
accuracy metric was consumed. The reusable channel diagnostics remain, while
the oriented-grid correction and its scene classifier remain default-off.

The published COIN-LIO TunnelD reference is 0.487 m ATE and 1.59% RTE. These
numbers are only an external reference because the local scorer has not yet
reproduced COIN-LIO on the identical converted input.

## Failure analysis

The source timestamps and calibration path are internally consistent. The v1
degeneracy intervention did not fire:

- persistent-prior attempts: 0
- persistent-prior applications: 0
- confirmed persistent weak directions: 0

The estimated v1 path is 261.51 m and its per-step speed p95 is 5.40 m/s,
versus 179.71 m and 2.79 m/s for the position ground truth. This is not a
single constant scale error; it contains local correspondence spikes and ends
36.29 m from its own starting point even though the reference is a return
trajectory.

The exploratory v2 changes only four existing reflectivity-gate parameters.
It makes 998 correlation attempts, accepts 897 shifts, and corrects 721 scans.
Its endpoint separation improves to 8.66 m, but its total path grows to
321.70 m and speed p95 to 7.22 m/s. The better global ATE therefore hides
worse local motion. The 10 m RTE correctly rejects it.

The margin-disabled diagnostic run records 913 peak-margin samples with mean
0.0942 and minimum 0.0. Its correction acceptance logic is identical to v2,
but its trajectory SHA-256 differs and its score moves by about 0.5 m ATE.
RKO's parallel scan processing is therefore not byte-deterministic across
runs. This is further evidence that the required three repetitions are a
measurement requirement, not just a reporting convention. No peak-margin
threshold may be selected from either single-run TunnelD score.

The validated open-tunnel inertial preset is also inapplicable without a new
classifier. On TunnelD, the median fraction of points below 3 m is 0.780 and
the median fraction above 10 m is 0.0075, so every scan fails that preset's
open-scene gate. Its thresholds must not be relaxed specifically for this
sequence.

## Evaluation infrastructure correction

The first score attempt exposed a benchmark bug rather than a SLAM failure.
The 10 Hz estimate has normal timestamp jitter up to roughly 0.106 s, while
the scorer used a 0.100 s interpolation bracket. That fragmented the matched
ground truth into 276 short blocks and left no complete 10 m segment.

The frozen ENWIDE bracket is now 0.11 s. It matches 23,202 of 23,300
ground-truth poses in six blocks while still splitting the real larger gaps.
The runner also emits its summary receipt when position scoring fails, so a
metric failure cannot erase the process result.
