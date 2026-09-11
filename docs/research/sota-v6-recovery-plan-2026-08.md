# SOTA-v6 recovery plan after the SOTA-v5 blind result

The SOTA-v5 evidence bundle is complete and negative. The result is not eligible
for a SOTA claim: trajectory, runtime, memory, and map-geometry gates failed.
Completion and the verified-false-loop gate passed. The independent visual
track also failed and remains separate from the primary LiDAR–IMU claim.

The exact machine-readable plan is
`configs/slam_benchmark_profiles/sota_v6_development_recovery_plan.yaml`.
Its evidence source is the SHA-256-bound SOTA-v5 bundle at
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v5_evidence_bundle_6284e1d6`.

## What the result says

The candidate geometric-mean APE was `24.9877 m`; Voxel-SLAM was the strongest
eligible suite rival at `2.7160 m`, for `-820.02%` improvement. NavINST missed
real time at `RTF 1.13475`. Candidate peak RSS was `916.48 MB`; Point-LIO used
`291.75 MB`, so the limiting ratio was `3.1413`, above `1.20`.

Postblind stage scoring rules out graph correction as the trajectory-error
source. On repetition 1, raw frontend and final corrected APE were respectively
`2.81947/2.81947 m` on NavINST, `6.97955/6.97953 m` on Oxford, and
`792.83954/792.83954 m` on UrbanNav. Frontend accuracy is therefore the first
workstream. Conversely, the Oxford process-tree peak attributed `744864 KiB`
of an `798.49 MB` run peak to the graph node, making backend storage and queue
state the resource workstream.

## Development and promotion policy

NavINST Indoor02, Oxford Spires Keble 05, and UrbanNav HK Tunnel 1 are now
development/regression data. They must never be described as blind in a later
claim. Development uses one global candidate configuration, three repetitions,
and the same trajectory, completion, loop, resource, and map checks.

Frontend promotion requires `GM APE <= 2.63451 m`, 100% completion, and no
single sequence more than 10% worse than its best fixed rival. Backend promotion
requires `peak RSS <= 330 MB`, `RTF <= 0.85` on every development sequence,
zero verified false loops, and no trajectory change above `1e-6 m` for
resource-only changes.

Only after both workstreams pass do we audit the complete dataset exposure
history and choose at least three fresh cross-dataset holdouts. Their sensor and
ground-truth identities, candidate/rival revisions, thresholds, machine, and
run order are preregistered before execution. Ground truth remains closed until
all three-repetition runs are terminal and raw outputs are hash-sealed.

The `leo-drive/color-point-cloud` project is retained as a later reference for
calibrated color-map visualization in the independent visual track. It does not
affect the primary trajectory claim or the closed SOTA-v5 result.

## Initial NavINST development screen

These are single-repetition, map-save-skipped development screens, not promotion
evidence. The common frozen reference and interpolating scorer were used after
each runtime-only run completed.

| Global case | APE RMSE (m) | RTF | peak RSS (MB) | Decision |
|---|---:|---:|---:|---|
| v5 plain solver | 2.81585 | 1.09813 | 519.42 | reject |
| older balanced profile | 3.18072 | 1.21282 | 968.72 | reject |
| 0.35 m voxel, 20 points/voxel, v5 budget | 3.26439 | 1.08259 | 739.52 | reject |
| 0.35 m voxel, 8 points/voxel, v5 budget | 3.17611 | 1.10097 | 565.37 | reject |
| 0.35 m voxel, 4 points/voxel, v5 budget | 2.98495 | 1.09504 | 510.26 | cross-dataset screen, then reject |
| 0.35 m voxel, 1 point/voxel, v5 budget | 3.18896 | 1.05518 | 467.94 | resource signal only |

The initial table previously mixed the scorer's mean column with the
preregistered RMSE primary metric. The table above is corrected to RMSE. None
of the retention cases improves the SOTA-v5 NavINST RMSE (`2.81947 m`). The
8-point case does reduce the fine-voxel case peak by about 23.5%, but still
fails the development resource gates. Its peak snapshot assigns
approximately `357 MB` to RKO-LIO and `137 MB` to the graph node, so frontend
voxel retention and graph input-cloud size must be reduced independently.

The unchanged 4-point configuration was screened on all three development
datasets. RMSE was `2.98495/5.80056/780.74836 m` on NavINST/Oxford/UrbanNav,
respectively: `-5.87%/+16.89%/+1.53%` against the v5 candidate. Its geometric
mean improved only `4.67%`, from `24.98771` to `23.82169 m`, while remaining
far from the fixed rivals and regressing NavINST. The configuration is rejected.

## Symmetric loop-verification screen

The relaxed 8 m correction-cap diagnostic showed that source-only overlap was
unsafe: five nonlocal edges entered the final graph, but four failed the frozen
reference-relative residual limits. The backend now offers an opt-in harmonic
forward/reverse overlap gate. The historical source-only behavior remains the
default. The trajectory runner was also corrected to retain a `/modified_path`
published by a loop event when map saving is skipped; previously it silently
replaced that corrected path with raw odometry.

On the same NavINST 4-point frontend, the symmetric `0.60` gate produced two
accepted proposals that deduplicated to one final nonlocal edge, `140 -> 421`.
Its reference-relative residual was `0.61465 m / 3.79443 deg`, within the
frozen `1 m / 15 deg` limits, so the final graph had zero verified false edges.
The raw trajectory was byte-identical to the source-overlap run. The corrected
APE RMSE improved from `2.98495 m` to `2.90371 m` (`2.72%`), with `577/577`
submaps processed, `RTF 0.82812`, and `510.32 MB` peak RSS.

This validates the symmetric gate as a backend safety improvement, but not the
4-point configuration as a promotion candidate: it remains `2.99%` worse than
the SOTA-v5 candidate on NavINST and exceeds the `330 MB` backend development
memory gate. No fresh blind data may be opened from this result.

A follow-up combined the same symmetric backend with the more accurate plain
frontend. It completed `574/574` submaps at `RTF 0.82920` and `525.00 MB`, and
its raw trajectory was byte-identical to the earlier plain screen. No loop
passed the mutual `0.60` floor, so corrected and raw RMSE were both
`2.81585 m`. The closest rejected candidate measured `0.599791`. The threshold
is not relaxed post hoc on exposed development data; this combination is also
rejected.

## Plain-frontend resource screen

The accurate plain frontend was then held byte-fixed while resource costs were
removed one at a time. Limiting the lossless offline LiDAR read-ahead buffer to
four frames reduced the `525.00 MB` mutual-backend run to `497.54 MB`. Spilling
graph submaps to a run-isolated PCD cache and disabling the unused benchmark
static-TF helper reduced the combined peak to `417.34 MB`. Both runs retained
the same raw trajectory SHA-256,
`3bfd377131cd7a15bd479453a9ba77e1f18b31cde177a70c4d76c951610c70fd`.
The exact frontend input to these runs is now checked in as
`configs/sota_v6/development/rko_lio_plain_memory.yaml`; its SHA-256 is
`426ee9e45be23ab24cac35a0c53361fb2d1d6bd9d7945b2422537309d1e355c1`.

RKO-LIO previously reserved the full configured 20-point capacity on the first
point inserted into every voxel. Starting each vector at one point and allowing
normal vector growth preserves the accepted points and their insertion order.
The NavINST run remained byte-identical at APE RMSE `2.815850 m`, completed
`574/574` submaps with no loop event, and measured `RTF 0.82645` and
`354.91 MB`. This is a `32.40%` peak reduction from the mutual plain baseline.
At the peak, launch/RKO-LIO/graph contributed `40.71/226.46/87.73 MB`.

This is a useful resource-only improvement, but it still fails the preregistered
`330 MB` development gate and is not promotable. An inline one-point voxel
representation increased the peak to `382.20 MB` and was reverted. Accuracy-
changing shortcuts were also rejected: 50 m range reached `263.80 MB` but raw
RMSE regressed to `3.15948 m` and introduced one verified false edge; 75 m range
reached `306.43 MB` but RMSE regressed to `3.05200 m`; and four points per voxel
reached `304.48 MB` but raw RMSE regressed to `3.21283 m`. No fresh blind data
is opened from this screen.

## Cross-dataset driving-bridge screen

A single-repetition frontend-only screen tested whether the default-off
anchor-decayed inertial bridge could recover UrbanNav's tunnel distance without
changing the global configuration per dataset. These are exposed development
results, not promotion evidence. The first 12.5 m/s vehicle-envelope run reduced
UrbanNav RMSE from `792.83954 m` to `129.46159 m`, but the same configuration
misclassified the walking NavINST sequence and regressed its RMSE to
`40.54768 m`. Raising the minimum speed to `7 m/s` preserved NavINST exactly
(`2.81947 m`, zero bridge attempts) but established UrbanNav anchors too late
and regressed it to `864.58873 m`.

The bridge now has a default-compatible minimum-IMU-density gate. Unit coverage
confirms that a 30-sample minimum rejects a 13-sample interval and accepts
30/40-sample intervals; observed initialization intervals were 13 on NavINST
and 40 on UrbanNav/Oxford. This is retained as a general safety mechanism; the
default of one sample preserves historical behavior. It did not by itself make
the vehicle bridge globally applicable: the corresponding UrbanNav/Oxford
single runs measured
`179.10298 m` on UrbanNav and `229.77000 m` on Oxford, versus the v5 Oxford
baseline of `6.97955 m`.

Additional range-scene applicability screens were negative. A `0.12`
near-return fraction plus 30 consecutive trusted scans preserved Oxford at
`6.97824 m` with zero bridge attempts, but its 60-second cooldown also disabled
UrbanNav (`792.83613 m`). Removing the cooldown improved UrbanNav only to
`416.75242 m`; a `0.15` threshold measured `428.60424 m`. Thirty-scan moving
averages measured `499.67152 m` at `0.12` and `713.94928 m` at `0.18`.
One-shot initial-scene latching rejected UrbanNav at both `0.12` and `0.18` and
was reverted together with the moving-average implementation. Horizontal-only
bridge corrections (`216.92329 m`) and gravity-window alignment (`646.307 m`)
were also worse than the initial vehicle-envelope run and were reverted.

The checked-in
`configs/sota_v6/development/rko_lio_driving_inertial_bridge.yaml` is therefore
an explicitly retired reproduction control, not a candidate. No fresh blind
holdout may be opened from this screen. The next frontend candidate must solve
applicability without using dataset identity and must pass all three exposed
development datasets before the preregistered three-repetition promotion gate.

## Texture and physical-observability frontend screens

Two further default-off frontend directions were evaluated on the same
consumed development inputs. Neither is a promotion candidate, and no fresh
blind holdout was opened.

The oriented reflectivity/height disagreement matcher had valid UrbanNav
input (`intensity` float32 and `reflectivity` uint16), but failed its
pre-accuracy diagnostic gate. At full weight it corrected 3,243/3,964 scans,
produced 160 search-boundary shifts, raised peak RSS from 221,928 KiB to
472,860 KiB, and shortened the estimated path from 1,263.53 m to 1,121.41 m.
A zero-weight observation run preserved the baseline path but still measured
a negative longitudinal intensity-minus-ICP velocity median in every 500-scan
segment, with 2,674 negative versus 481 positive disagreements. It also used
394,236 KiB. The signal therefore points opposite the required tunnel-motion
recovery and violates the memory gate; accuracy was intentionally not scored.

A point-to-plane residual exposed the geometric null space hidden by the
historical point-to-point translation block and improved UrbanNav APE RMSE
from 792.83613 m to 596.12606 m (24.8%) at RTF 0.205 and 218,660 KiB RSS.
It was not globally applicable: NavINST regressed from approximately 2.82 m
to 42.78942 m. Separating the objectives—retaining point-to-point pose
optimization and using point-to-plane only for observability—reduced that
NavINST regression to 3.17056 m but did not remove it. Weight 0.10 measured
3.14657 m. A 120-scan persistence gate still admitted one NavINST intervention
and measured 3.18856 m. Raising it to 150 rejected UrbanNav too: zero
interventions and 793.09148 m RMSE. Weak-axis/motion alignment did not provide
a separator (median cosine 0.914 UrbanNav versus 0.935 NavINST). The entire
experimental implementation was therefore removed rather than retained as a
dataset-tuned switch.

## Covariance-aware registration screen

An opt-in scan-to-model GICP backend based on pinned `small_gicp` source was
then screened on the consumed UrbanNav development input while preserving
RKO-LIO deskew, IMU pose prediction, state handling, and the legacy map and
recovery paths. The best sparse setting (`0.5 m` correspondence distance,
1,500 keypoints) completed all 3,982 poses and improved the baseline RMSE from
approximately `792.84 m` to `707.59656 m` (10.75%), at `RTF 0.247` and
`304,676 KiB` peak RSS. It was nevertheless substantially worse than the
already rejected point-to-plane screen (`596.12606 m`) and remained far outside
the accuracy required for promotion.

Two bounded follow-up screens were negative. Increasing the correspondence
distance to `1.0 m` measured `720.73303 m` RMSE, `RTF 0.248`, and `309,464 KiB`;
dense sampling with 3,000 keypoints measured `723.20938 m`, `RTF 0.317`, and
`311,860 KiB`. The candidate therefore failed the exposed-development
pre-accuracy gate before NavINST or Oxford evaluation. No fresh blind holdout
was opened, and the backend, dependency, tests, and development configuration
were removed rather than retained as repository complexity.

## Independent scan-to-scan diagnostic

A default-off diagnostic then integrated a second trajectory by registering
each deskewed scan only against its immediate predecessor. It never fed back
into the primary estimate, and was intended to test whether scan-to-map
self-locking—not the scan geometry itself—caused the UrbanNav failure. Both
trajectories completed all 3,982 poses. The unchanged primary trajectory
measured `792.83613 m` RMSE and a `1,263.53 m` path; the independent
scan-to-scan trajectory measured `790.30935 m` RMSE (only 0.32% better) and an
even shorter `1,126.17 m` path. The tunnel-axis ambiguity therefore persists
without the accumulated map. This pre-accuracy diagnostic was removed, and no
NavINST, Oxford, or fresh blind input was opened.

## Translational inertial-state screen

A bounded 9-state translation/velocity/body-accelerometer-bias filter was
screened next. It used IMU propagation and point-to-plane translation
information, projecting its correction only into weak LiDAR axes while leaving
the established ICP orientation and strong translation axes untouched. The
first UrbanNav run completed all 3,982 poses at `RTF 0.151` and `212,180 KiB`
peak RSS. RMSE improved from `792.83613 m` to `459.94448 m`, and estimated path
length recovered from `1,263.53 m` toward the `3,152.15 m` reference at
`3,215.53 m`. However, the result depended on a nonphysical final residual
accelerometer bias of approximately `[-2.89, -0.56, 0.34] m/s^2` and accumulated
a `288.58 m` z-span versus `31.79 m` in the reference.

Gravity-axis rejection improved RMSE to `398.52652 m`, but retained a
`270.49 m` z-span and a `3.88 m/s^2` horizontal bias magnitude. Projecting each
accepted correction exactly onto the world horizontal plane measured
`378.43759 m`, but map feedback still produced a `320.21 m` z-span and the
estimated bias grew to `5.38 m/s^2`. Finally, enforcing a physically bounded
`0.5 m/s^2` residual-bias norm caused runaway (`909.67430 m` RMSE,
`1,371.89 m` z-span). The apparent accuracy gain was therefore inseparable
from an invalid state estimate. The implementation, tests, parameters, and
development configuration were removed; NavINST, Oxford, and fresh blind data
were not opened.

This result closes the lightweight RKO-LIO post-ICP fusion route. The next
frontend candidate should start from a tightly coupled estimator that already
meets the exposed NavINST/Oxford accuracy regime, then add a globally applicable
Urban tunnel observability treatment and the existing verified-loop backend.

## Voxel-SLAM degeneracy-reset diagnostic

The fixed Voxel-SLAM revision `70fc8a28d63823d5989ff184daeea0787b672398`
already contains a tightly coupled iterated error-state Kalman filter. Its
point-to-plane normal matrix is also used for a degeneracy test: the odometry
thread increments a counter when the smallest normal-information eigenvalue is
below `14` and calls `system_reset()` after the configured bound. On all three
SOTA-v5 UrbanNav repetitions, the first reset occurred at the same sensor time
(`1621322564.65`, within milliseconds), followed by exactly `156` resets while
the moving-platform initialization repeatedly failed.

A bounded, single-repetition development diagnostic changed only the global
odometry `degrade_bound` from `100` to `100000`, leaving the pinned image and
all sensor parameters unchanged. It reproduced the fixed run exactly through
the original reset instant (pose `[-65.629, 355.006, -41.457] m`) and confirmed
that reset suppression did not recover the weak direction. The estimate then
ran away to `[-12236.830, 34774.239, -28525.708] m`, accumulated a
`47,638.646 m` path, and terminated by `SIGSEGV` after `3,010/3,980` poses.
Peak RSS was `1,705.19 MB`; the incomplete-overlap APE diagnostic was
`11,603.25 m` RMSE with `96/395` reference poses necessarily rejected. The
original fixed run's full online trajectory path was `1,976.977 m`.

This establishes that the reset is a downstream safety response, not the
primary accuracy fault. The diagnostic configuration is removed rather than
retained as a candidate. A viable Voxel-SLAM derivative must constrain or
project the LiDAR update in weakly observable eigendirections before state and
map corruption, while preserving IMU propagation; simply disabling recovery
is rejected. No NavINST, Oxford, or fresh blind data was opened.

The first such bounded derivative projected the six-dimensional LiDAR normal
equations onto translation eigenvectors whose normal-information eigenvalue
met the same upstream `14` threshold. It retained rotation measurement terms,
the full propagated IMU prior, and the original reset path whenever fewer than
two translation axes remained observable. The exact development source and
image were SHA-256-bound as `cd3af167...897212` and `fc6be7bf...fc1bc1`,
respectively, against the same fixed base revision.

This instantaneous hard projection also failed its UrbanNav pre-accuracy
screen. It crossed the original reset time with zero resets, but by sensor time
`1621322613.47` the estimate had already reached approximately
`[-585, 2640, -844] m`. The run was deliberately stopped once failure was
unambiguous: after `2,291` poses its endpoint was
`[-1046.171, 10885.765, -5631.733] m` and its accumulated path was
`12,424.977 m`, still with zero resets. Because the run was intentionally
terminated, no completion, APE, or resource result is claimed. The failure
shows that a per-scan eigenbasis rotates with the already-corrupted map and
does not provide a stable world-frame weak-axis constraint. This derivative is
rejected before cross-dataset evaluation; no fresh blind input was opened.

A subsequent code audit found that LocalBA optimizes all 15 state dimensions
(`R/p/v/bg/ba`) but upstream copies only `R/p` from the terminal window state
back to online `x_curr`. A one-change diagnostic replaced that partial copy
with the complete state assignment. The exact patched source and development
image were bound as `1ca86c50...e3` and `063b6a27...160d`. Although this closes
a real state-handoff inconsistency, it did not prevent the UrbanNav failure:
the first reset moved only from sensor time `1621322564.65` to
`1621322565.45`. Immediately before the original reset time, the diagnostic
had a `366.765 m` path and endpoint `[-65.880, 356.027, -41.393] m`, versus
`365.718 m` and `[-65.629, 355.006, -41.457] m` for the fixed run. The run was
stopped after reset onset, so no completion or APE result is claimed. Complete
LocalBA state handoff is insufficient by itself and is not promoted or tested
on another dataset.

## Voxel-SLAM weak-axis bridge screens

Four follow-up derivatives used Voxel-SLAM's own normal-information spectrum
as a dataset-independent applicability signal. After ten consecutive scans
with exactly one translation eigenvalue below `14`, the bridge retained the
two observable LiDAR directions and propagated the weak direction from a
causal entry state. Fewer than two observable translation axes retained the
upstream reset response. These are single-repetition UrbanNav development
screens, not promotion evidence.

The first body-stable form fixed the weak axis in the sensor body at entry and
therefore followed subsequent IMU rotation. It completed all `3,980` poses
with zero resets and improved RMSE from fixed Voxel-SLAM's `818.80010 m` to
`496.55993 m`; it also beat fixed Point-LIO's `638.93870 m` on this sequence.
If Voxel-SLAM's exposed NavINST/Oxford results were preserved, the three-
dataset geometric mean would be `2.29897 m`, about `15.35%` better than the
fixed Voxel-SLAM geometric mean. This is the first material accuracy signal
from the recovery work, but it is not promotable: body-axis gravity leakage
produced a `5,386.55 m` path and `1,855.99 m` z-span versus reference
`3,152.15/31.79 m`. RTF was `1.061` and peak RSS was `3,953.46 MB`; the map
contained `17,635,922` points from `3,972` chunks.

Projecting the bridge axis into the gravity-orthogonal plane (v2) did not
stabilize the independent vertical estimator. It completed with one late
reset, `1,034.60956 m` RMSE, `2,432.77 MB` peak RSS, and `RTF 1.005`, and is
rejected. Hard-fixing gravity-axis position and velocity during degeneracy
(v3) held the pre-reset z-span to `31.03 m`, but its instantaneous entry speed
over-integrated distance: at its first reset (`1621322659.97`) its path was
`2,243.54 m` versus `1,432.10 m` in the reference. It was stopped after reset
onset and has no completion or APE claim.

The fourth form estimated bridge speed from a causal 20-second odometry
history. It remained reset-free and gravity-stable through the earlier v3
failure, but releasing the hard constraints after apparent observability
recovery caused a late map/vertical runaway. It completed with `705.66333 m`
RMSE, `RTF 1.255`, and `8,761.91 MB` peak RSS. Although this still improves the
fixed Voxel-SLAM UrbanNav RMSE, it fails accuracy, real-time, resource, and map
geometry gates. Exact v4 development source/image hashes are
`3f97972a...22cb0` and `014a21c3...2dd8b`.

The bridge family therefore remains development-only. The next bounded step
must preserve the v1 distance signal while replacing hard gravity and recovery
transitions with uncertainty-weighted factors, and must cap active-map/map-
export retention before any NavINST or Oxford run. No fresh blind input was
opened.

## Bounded-map bridge convergence (v5-v17)

The next screens kept the v4 causal 20-second entry-speed estimate but made the
weak-axis and gravity constraints sticky across apparent recovery. Built-in
Voxel-SLAM loop closure and HBA were disabled so that only the already verified
external loop backend can eventually own loop acceptance. To bound memory while
preserving revisit information, scans were written to the existing PCD cache,
old active octrees were reduced to exact float plane summaries after 25 m of
estimated travel, and the nearest eligible historical scan was reloaded for
LocalBA. These remain exposed-development results, not blind evidence.

The progression was:

| Version | UrbanNav RMSE (m) | RTF | peak RSS (MB) | Disposition |
|---|---:|---:|---:|---|
| v5 | 665.569 | 1.253 | 8,248 | sticky bridge; resource failure |
| v6 | 665.575 | 1.002 | 1,754 | built-in global backend disabled |
| v7 | 831.172 | — | 1,133 | 100 m ordinary eviction starved map |
| v8 | — | — | — | invalid any-child frozen correspondence; stopped |
| v9 | 745.050 | — | 1,203 | exact-octant frozen planes |
| v10 | 488.216 | — | 842.5 | 25 m active map plus dense disk reload |
| v11 | 756.880 | — | 948 | stride-5 archive rejected |
| v12 | 488.226 | — | 832 | dense archive plus allocator controls |
| v13 | 488.223 | — | 832 | immediate descendant deletion |
| v14 | 488.221 | — | 352.65 | removed eager sliding-window reserve |
| v15 | 897.085 | — | 362 | symmetric covariance compression rejected |
| v16 | — | — | — | 20 m retention crashed near pose 1,650 |
| v17 | 488.219 | 1.002 | 272.44 | reclaim sliding-window vector capacity |

v17 completed all `3,980` UrbanNav poses with zero resets. Its `4,572.28 m`
path and `34.90 m` z-span are materially closer to the reference
(`3,152.15 m` and `31.79 m`) than the earlier bridge forms; endpoint
displacement is `1,501.50 m`. The full
`17,711,712`-point map is byte-identical to v10 and v14, and v17 trajectory XYZ
is identical to v14; the memory reduction is therefore not an accuracy/map
trade. It improves fixed Point-LIO's UrbanNav RMSE (`638.93870 m`) by `23.59%`.

Single-repetition cross-dataset screens then measured `0.17477256 m` on
NavINST and `0.13843821 m` on Oxford, both complete with zero resets and peak
RSS of `129.09 MB` and `103.98 MB`. NavINST is `4.80%` worse than fixed
Voxel-SLAM, Oxford is `5.58%` better than fixed Voxel-SLAM, and neither exceeds
the preregistered per-sequence `+10%` regression bound against the best fixed
rival. Together with UrbanNav, v17's exposed geometric-mean RMSE is
`2.277445 m`, a `16.15%` improvement over fixed Voxel-SLAM's `2.71599 m`.

ROS 1 bag replay at rate 1.0 imposes an approximately `RTF 1.002` wall-clock
floor: fixed FAST-LIO2 and Point-LIO show the same effect on all three exposed
datasets. A fail-closed, preblind-development-only runner option was therefore
added to test accelerated replay without changing the frozen blind default.
At rate 1.2, Oxford v17 completed with `0.13951432 m` RMSE (`+0.78%` versus its
rate-1.0 result), `RTF 0.83520`, and `103.75 MB` peak RSS. A formal blind run
may use a faster common replay rate only after that rate is separately frozen
for every ROS 1 system; this screen itself cannot be promoted to blind
evidence.

The repository-built image then reproduced the rate-1.2 result on the other
two exposed datasets. NavINST measured `0.17452525 m` RMSE (`-0.14%` versus
rate 1.0), `RTF 0.83632`, and `129.87 MB` peak RSS over `3,485` complete poses.
UrbanNav measured `488.22862 m` (`+0.002%`), `RTF 0.83592`, and `272.53 MB`
over `3,979` complete poses. All runs exited cleanly. The resulting rate-1.2
single-repetition geometric mean is `2.282269 m`, still `15.97%` better than
fixed Voxel-SLAM. Thus accelerated replay has now passed the exposed
cross-dataset screen, but two additional repetitions per dataset and a common
rival-rate freeze remain mandatory before blind execution.

The reproducible v17 candidate is the fixed Voxel-SLAM revision plus
`docker/patches/voxel_slam_v17/weak_axis_bounded_map.patch` (SHA-256
`62f6e1c5...95b25`). The patched `voxelslam.cpp` and `voxel_map.hpp` hashes are
`6f8a5121...10c11` and `e3416197...fbe3f`; the clean repository build produced
local image ID `sha256:8d1fae97...130a52`. Promotion remains blocked on three
rate-stability repetitions per exposed dataset, verified external-loop backend
integration and false-loop audit, and map-geometry non-inferiority. No fresh
blind holdout has been opened.

## External verified-loop compatibility screen

The offline verified-loop backend consumes exact-time ROS 2 odometry/cloud
pairs, whereas Voxel-SLAM emits final poses and body-frame PCD chunks.
`scripts/voxel_run_to_graph_bag.py` now performs that deterministic,
lossless adaptation. It was exercised on all three exposed v17 outputs:
NavINST contained `3,477` pairs and `11,727,179` points, Oxford `5,823`
pairs and `30,901,030` points, and UrbanNav `3,972` pairs and
`17,711,712` points.

The existing 8 m mutual-overlap backend profile is not safe for this frontend.
On NavINST it accepted nine edges; reference audit found one verified false
edge because its robust registration fitness was `0.6901 m`, above the
frozen `0.50 m` audit limit. The other edges met the `1.0 m / 15 deg`
reference residual limits. Propagating the graph correction back to all dense
poses changed RMSE from `0.174525 m` to `0.182022 m` (`+4.30%`).

On Oxford it accepted 12 edges, of which nine were verified false. Maximum
reference-relative residuals were `19.769 m` and `25.309 deg`. Although the
dense corrected trajectory measured `0.135089 m`, that apparent improvement
is inadmissible because it depends on false edges. Reducing the permitted
registration correction from 8 m to 1 m produced the identical 12-edge set:
the repeated structures were already aligned in the drifted estimated map and
required almost no registration correction. Geometry overlap and correction
magnitude therefore cannot distinguish these false revisits. UrbanNav
accepted zero edges.

This external-backend profile is rejected rather than tuned against exposed
ground truth. v17 remains frontend-only with built-in Voxel-SLAM loop/HBA
disabled, so its accepted-loop count is exactly zero and its verified-false
count is zero without post-hoc edge selection. The adapter and complete
rejected screens are retained to make that decision reproducible. Candidate
promotion must explicitly freeze the no-loop configuration and demonstrate
zero accepted edges in every repetition; a future nonzero-loop backend needs
a new preregistered, ground-truth-independent discriminator before evaluation.

## Explicit no-loop v17 three-repetition result

The repository image, rate 1.2, CPU set `2-7`, and candidate-specific
configuration hashes were then held fixed for three repetitions on every
exposed dataset. Each candidate configuration differs from its fixed-rival
sensor configuration only by explicit `Loop/enable: 0`. All nine runs
completed and all mapper logs contained zero accepted-loop markers.

| Dataset | median RMSE (m) | min-max RMSE (m) | population std (m) | max RTF | max RSS (MB) |
|---|---:|---:|---:|---:|---:|
| NavINST | 0.17486253 | 0.17469468-0.17495904 | 0.00010923 | 0.83623 | 130.98 |
| Oxford | 0.13948932 | 0.13939858-0.13957591 | 0.00007240 | 0.83517 | 104.95 |
| UrbanNav | 488.22565935 | 488.22543934-488.22576725 | 0.00013645 | 0.83608 | 274.50 |

The three-dataset median geometric mean is `2.28359687 m`, a `15.92%`
improvement over fixed Voxel-SLAM's `2.71599132 m`. Per-sequence change
against the best fixed rival is `+4.85%` on NavINST, `+7.50%` on Oxford,
and `-23.59%` on UrbanNav, so every sequence remains inside the
preregistered `+10%` regression bound.

Map construction was byte-deterministic across all three repetitions:
NavINST produced `11,727,179` points with SHA-256 `be95206f...4f00f`,
Oxford `30,901,030` points with `2d088933...9e4b4`, and UrbanNav
`17,711,712` points with `87caad98...8cbe7`. All materialized exposed
sensor bags were hash-verified and removed after their blocks. These results
close exposed trajectory/runtime/repetition stability; map-geometry audit and
the formal common-rate rival freeze remain before any fresh holdout is opened.

## v17 identical-message map-geometry rejection

The exposed v17 trajectory result was not promoted on accuracy and runtime
alone. `scripts/reconstruct_v17_exposed_standardized_maps.py` reprojected the
three v17 trajectories through the exact frozen canonical LiDAR messages,
sealed SOTA-v5 common time interval, frame transform, and 0.10 m occupied-voxel
representation. The already reconstructed fixed Voxel-SLAM maps were reused
only after their hashes and scan counts were verified. Every candidate/fixed
pair used identical scan counts: `3,459` NavINST, `5,803` Oxford, and `3,952`
UrbanNav. The reconstruction manifest SHA-256 is `36f5391e...bc621`; all three
temporarily materialized inputs were archive-verified and removed afterward.

The strict 0.10 m candidate/fixed occupied-voxel ratios were `1.0005`,
`0.9398`, and `3.0029`, respectively. The frozen intrinsic gates were then run
at 0.10, 0.25, and 0.50 m with one worker to respect the previously recorded
memory bound. NavINST and Oxford passed every scale. UrbanNav passed 0.10 and
0.25 m, but failed at 0.50 m: its occupied-voxel ratio was `7.2217`, while its
worst-candidate versus best-fixed entropy regression was `+0.15601 nats`,
above the frozen `+0.05` limit. The complete intrinsic report SHA-256 is
`ecb948fc...421f`.

The independent official-reference-map gate agreed that v17 is not yet
promotable. NavINST passed with candidate/fixed median-distance ratio
`0.99969`; Oxford failed with ratio `1.10425`, above the frozen `1.05` limit.
The complete reference-distance report SHA-256 is `eb6063a8...1c6d`. These
failures are retained rather than masked by changing a threshold. v17 is
therefore rejected as the fresh-holdout candidate despite its strong exposed
trajectory/runtime result.

UrbanNav diagnosis shows why the intrinsic gate matters. Over the common
interval the reference travels `3,152.1 m`; v17 travels `4,417.2 m` in the
same sampled support. It is accurate through the first 30% of the sequence,
then its cumulative-distance ratio grows to `1.401` by the end, where the
last-decile speeds are `22.39 m/s` estimated versus `6.34 m/s` reference.
The current weak-axis bridge freezes entry speed and therefore cannot follow a
later sustained slowdown. The next candidate must add a causal, bounded
weak-axis deceleration update and pass this complete map audit before fresh
holdout access.

For later evidence visualization only, the user-provided
`https://github.com/leo-drive/color-point-cloud` repository is retained as a
possible colorized-point-cloud reference. It must not alter geometric inputs,
metrics, or candidate selection.

## v18 causal-deceleration bridge rejection

v18 tested the preregistered next mechanism without opening fresh holdout
data. It extends v17 only while the weak axis is unobservable: negative
IMU-propagated acceleration is low-pass filtered with a 2 s time constant,
activated below `-0.15 m/s^2`, bounded at `-3 m/s^2`, and integrated with a
trapezoidal position update. Positive acceleration is ignored because this
mechanism cannot distinguish it from bias drift. The incremental patch is
`docker/patches/voxel_slam_v18/causal_deceleration_bridge.patch` (SHA-256
`e15a15c...eadd`), and the built image ID is
`sha256:30df7c43...499bb`.

The single-repetition consumed UrbanNav development screen completed all
`3,980` poses with process status zero, `RTF 0.83599`, and `209.95 MB` peak
RSS. Accuracy nevertheless regressed to `827.09342 m` ATE RMSE and
`239.67276%` 10 m translational RTE, versus v17's three-repetition median
`488.22566 m`. The trajectory SHA-256 is `36ddd6c3...0715`; the canonical
input was dematerialized after its SHA-256 was verified.

Distance diagnosis rules out promoting or threshold-tuning this candidate.
On the common support, reference, v17, and v18 travel `3,152.15`, `4,417.03`,
and `2,982.95 m`. v18 matches the reference through 30%, but its cumulative
distance ratios at 40%, 50%, 60%, and 70% are only `0.530`, `0.313`, `0.223`,
and `0.172`; it then reaches `0.946` at the end with a last-decile speed of
`25.61 m/s` versus `6.28 m/s` reference. The causal negative-only integrator
therefore treats sustained inertial bias as braking, nearly stops the bridge,
and cannot represent the later reacceleration. v18 is rejected before map
reconstruction or additional repetitions. A subsequent mechanism must obtain
an independent motion constraint or explicit bias observability rather than
retuning these acceleration thresholds. Fresh holdout remains unopened.

## v19 pre-entry bias-calibrated bridge rejection

v19 returned to v17 and tested one explicit-bias-observability mechanism. In
the observable 20 s before bridge entry, it retained body-frame velocity
innovations between the IMU prediction and LiDAR-updated state. At entry it
projected their median onto the prospective weak axis, froze that residual
acceleration correction, and integrated the corrected weak-axis acceleration
with a physical `[-3, +3] m/s^2` bound. It did not use dataset identity,
reference poses, or fresh holdout data. The incremental patch SHA-256 is
`c2b15552...9ee26`, the patched source SHA-256 is `7af27e8c...a42b4`, and the
built image ID is `sha256:3e24030e...cf165`.

The consumed UrbanNav single repetition completed all `3,980` poses with a
clean exit, `RTF 0.83591`, and `243.31 MB` peak RSS. Its 183 pre-entry samples
produced a frozen correction of `-0.294048 m/s^2`. The resulting ATE RMSE was
`558.83867 m`, 10 m translational RTE was `209.85573%`, full trajectory length
was `6,772.41 m`, and z-span was `60.05 m`. It therefore regressed v17's
`488.22566 m` median ATE rather than qualifying for further repetitions or map
reconstruction. The trajectory and score SHA-256 values are
`cfa5a574...65390` and `d30663c5...8103`.

On the common sampled support, v19 travels `6,742.40 m` versus `3,152.15 m`
reference and `4,417.03 m` v17. Its cumulative-distance ratio is already
`1.482` at 40% and grows to `2.139` at the end; last-decile speed is
`25.49 m/s` versus `6.28 m/s` reference. The pre-entry velocity innovation is
not a transportable accelerometer-bias observation: it also contains the
registration/map update dynamics. Treating it as a constant bias therefore
causes overtravel. v19 is rejected without threshold tuning. A credible next
step requires an actually independent longitudinal motion measurement; the
available LiDAR-IMU signals have now failed both direct integration and
pre-entry innovation calibration. Fresh holdout remains unopened.

## v20 turn-dynamics speed constraint pre-implementation diagnostic

The next candidate mechanism under consideration was a turn-dynamics speed
constraint: during a turn the centripetal relation `a_lat = v * omega_z`
recovers speed `v = a_lat / omega_z` independently of the weak-axis LiDAR
signal, which could re-anchor the frozen v17 bridge speed. Because v18 and
v19 showed that direct integration and bias calibration both fail, this
mechanism was screened on the exposed UrbanNav sensor stream **before any
implementation**, and rejected without opening fresh holdout data.

`scripts/v20_turn_dynamics_diagnostic.py` re-inspects the exact canonical
ROS 1 bag (`/imu/data`, 159,459 samples over the full 398.64 s window; the
identity extrinsic puts the body at the Xsens IMU frame) and compares the
IMU-only speed estimate against the exposed common reference
(`common_reference.tum`, 3,152.15 m path). It applies no gravity subtraction
to the body `y` axis, because gravity projects onto the body `z` axis in the
level-turn model, and gates on the lateral-to-gravity ratio to exclude
roll-contaminated samples.

The tunnel is effectively straight. The reference trajectory has a maximum
yaw rate of only `3.68 deg/s`. The IMU reports `omega_z` with mean
`-0.762 deg/s`, std `2.938 deg/s`, and an absolute maximum of
`23.450 deg/s`, but only `5.74%` of samples exceed `5 deg/s` and only
`2.55%` exceed `10 deg/s`. Just two sustained (>= 2 s) strong-turn segments
exist in the whole sequence; in both, the IMU `a_y / omega_z` estimate
(`3.4` and `3.0 m/s`) is roughly one third to one half of the reference
speed (`8.9` and `4.2 m/s`). The signed correlation between `a_y` and
`v_reference * omega_z` is `-0.05` for the positive convention and `+0.05`
for the negative convention, i.e. no relationship. During strong turns the
sign of `a_y / omega_z` is stable in only 46% of samples. The gravity-based
roll estimate peaks at about `8 deg` during those turns, so banked-curve
gravity contamination corrupts the lateral specific force exactly when the
constraint would be needed.

The conclusion is a `NO-GO` before implementation: turn dynamics do not
provide a usable independent speed anchor in this tunnel. The mechanism is
rejected without a patch, build, or repetition; no map or score artifact is
claimed. A future credible step still needs an independent longitudinal
motion measurement, but the available LiDAR-IMU signals have now also failed
the turn-constraint route. Fresh holdout remains unopened.

## Oxford v17 map-geometry investigation and GBA re-enablement screen

The v17 three-repetition trajectory result passed its exposed gates, but the
official-reference map gate failed on Oxford (`median distance ratio 1.10425`
against the frozen `1.05` limit). The map-level difference was traced to the
v17 design decision to disable the built-in Voxel-SLAM global backend
(`Loop/enable: 0` disables both `thd_loop_closure` and `thd_globalmapping`),
not to the trajectory itself.

On the identical frozen canonical LiDAR input (`3.436e8` finite points,
`5,803` scans), the 0.10 m occupied-voxel reconstruction produced
`60.64M` points for v17 versus `64.53M` for fixed Voxel-SLAM, and at 0.25 m
`16.59M` versus `15.59M` occupied voxels. The v17 map is therefore not
"thicker" but spread over more horizontal cells: `130,008` XY columns versus
`115,003` (`+13.0%`), and `40.1%` of its 0.25 m voxels lie beyond `5 m` from
the official reference versus `37.0%` for fixed. Both systems' SE(3)
trajectory alignments were applied; swapping them changes each median by less
than `0.002 m`, so the gap is genuine map content, not an alignment artifact.
The v17 trajectory and fixed trajectory agree to `0.04 m` mean position
residual after SE(3), so the map spread is a global-alignment consequence of
disabling GBA rather than a raw-odometry accuracy failure.

A bounded re-enablement screen (v21) re-introduced the global HBA backend
without loop closure. The no-loop `thd_loop_closure` was extended to generate
keyframes and populate `multimap_keyframes`, and `thd_globalmapping` was
re-enabled. The single UrbanNav screen ran to completion with
`GBA/total_max_iter: 1` and a `5 m` keyframe interval, and the gtsam pass
processed `3,972` poses with `5,817` edges. It did not improve the result:
UrbanNav ATE RMSE changed from `488.23 m` to `499.58 m`, and the build
aborted under `gtsam::IndeterminantLinearSystemException` when odometry edges
were dropped to avoid keyframe/scan index conflicts.

The root incompatibility is structural: Voxel-SLAM's global HBA assumes a
one-to-one keyframe-to-scan correspondence. The no-loop candidate streams all
scans into `scanPoses` while keyframes are subsampled by distance, so HBA
edge ids no longer map to the scan indices used by `build_graph`'s pose
initialization and write-back. Fixing it requires either a full keyframe-id
to scan-index remap inside the global backend or a one-to-one keyframe
generation policy, both of which are invasive changes to a pinned rival
revision. The v21 source and image are retained in the workspace for
reproducibility but are not a promotion candidate. Fresh holdout remains
unopened; Oxford remains a pending map-geometry blocker for the v17
configuration rather than a trajectory defect.

## v17 against the fixed suite rivals (SOTA-v5 holdout set)

For the exposed SOTA-v5 holdout trajectories, the v17 candidate already
outperforms GLIM overall while the raw-odometry gap on Oxford remains a
promotion blocker rather than a claim failure. Median per-sequence APE RMSE
(metres) on the common SOTA-v5 reference, using the frozen interpolating
scorer:

| System | NavINST | Oxford | UrbanNav | GM |
|---|---:|---:|---:|---:|
| v17 | 0.17486 | 0.13949 | 488.22566 | 2.2836 |
| GLIM | 1.52860 | 0.11316 | 757.80951 | 5.0779 |
| Point-LIO | 0.65142 | 0.13626 | 638.93868 | 3.8388 |
| FAST-LIO2 | 0.84663 | 0.52470 | 757.42790 | 6.9576 |
| fixed Voxel-SLAM | 0.16664 | 0.14671 | 818.79944 | 2.7160 |

v17 beats GLIM by `88.6%` on NavINST and `35.6%` on UrbanNav; the three-
dataset geometric mean is `55%` lower (`2.28 m` versus `5.08 m`). The one
remaining loss is Oxford, where GLIM (`0.113 m`) and Point-LIO (`0.136 m`)
are both below v17 (`0.139 m`), so the per-sequence "no worse than the best
fixed rival by more than 10%" promotion bound is violated on Oxford
(`+23%` versus GLIM).

The Oxford loss is a vertical (z) accuracy issue, not horizontal or loop
behavior. On the common reference, v17's 3D RMSE is `0.1395 m` split into
horizontal `0.0625 m` and vertical `0.1247 m`, whereas GLIM is horizontal
`0.0450 m` and vertical `0.1038 m`. The vertical error is concentrated in
two multi-minute windows (`t ~ 1710257709-7722`, peak `0.195 m`, and
`t ~ 1710257930-7973`, peak `0.317 m`), both on sharp curves with large
heading change. It is not explained by plane-normal z-constraint weakness
(the error-window normal |z| equals straight sections), by roll rate, or by
slope magnitude (the steepest `4.05 m` climb has low error), and the raw
odometry z profile matches the reference within `5 cm`; the residual is a
localised 3D offset of about `0.27 m` that SE(3) alignment cannot absorb.
The v17 patch does not cause it: fixed Voxel-SLAM shows the same z std
(`0.137 m`). This is retained as a Voxel-SLAM raw-odometry accuracy issue
for a future candidate rather than a v17-specific defect, and it does not
change the conclusion that v17 already outperforms GLIM on the exposed
holdout set.

## v22 vertical-velocity soft-constraint screen (Oxford)

A bounded single-repetition screen (v22) tried to close the Oxford vertical
gap inside Voxel-SLAM's own odometry. In the error windows the vertical
velocity carries a local bias of about `+6 mm/s` (whole-run bias is
`-0.01 mm/s`), which accumulates `0.16 m` of height error over `25 s` even
though the per-scan z information (`HTH(5,5) ~ 1e5`) is strong. The screen
added a soft constraint that blends the gravity-axis velocity back toward
the propagated IMU velocity whenever the z information weight fell below
`1e-3`, gated on the weak-axis bridge being inactive and behind a new
`Odometry/z_constraint_enabled` parameter (default off).

The screen did not fire: the measured `HTH(5,5)` stays at `1e5..1.6e6`
throughout Oxford, so the z measurement weight never drops below the gate.
The resulting Oxford APE RMSE was `0.1411 m` (v17: `0.1395 m`), i.e. no
improvement and a slight regression. The residual conclusion is that the
Oxford vertical error is a velocity-bias accumulation driven by the IMU
vertical-acceleration / correspondence integration inside Voxel-SLAM's
estimator, not by weak per-scan z geometry; the `HTH(5,5)`-based gate was an
invalid proxy. The v22 source, Dockerfile, and development configuration are
retained in the workspace for reproducibility but are not a promotion
candidate. Fresh holdout remains unopened.

## Fresh holdout candidate audit (2026-08)

The recovery-plan gate for fresh cross-dataset holdouts requires at least
three dataset families absent from every prior training, development,
validation, and holdout ledger. A bounded audit of the mounted dataset
inventory (`/media/sasaki/aiueo/datasets`) and the repository history
narrowed the candidate space.

Every LiDAR-IMU dataset already present is consumed:
- SOTA holdouts: `urbannav`, `navinst`, `oxford_spires`, `boreas`, `mcd`,
  `ntu_viral`, `geode`, `fusionportable_v2`, `botanic_garden`, `rellis_3d`,
  `mag4d`, `edi_slam`, plus the `excluded_consumed_families` list
  (`nclt`, `newer_college`, `hilti`, `kitti`, `rtk_slam`, `enwide`,
  `mun_frl`).
- `koide_hard_localization`: Livox LiDAR-IMU with TUM ground truth and no
  SOTA-ledger record, but `runs/` under the mounted copy contains a large
  set of `glil_*`/`g2_*`/`lidarloc_*`/`rko_lio_*` executions from 2026-07,
  so it was used by the lidarloc/GLIL development track and is excluded as
  well.
- `PPC-Dataset` is GNSS/IMU only (no LiDAR).

A development-only ROS1 normalization screen for koide-hard was implemented
and committed (`scripts/normalize_koide_hard_ros1.py`): it converts
`/livox/points` (PointCloud2) and `/livox/imu` into a canonical ROS1 bag,
keeping the Livox `t` field as float32 seconds. The Voxel-SLAM v17 image
consumes the result and completes the outdoor_hard_01a sequence (exit 0,
~2,630 poses), confirming sensor compatibility, though raw APE is large
(`~92 m`) because the Livox extrinsic and per-point timing are not yet
tuned for this screen. This converter is reusable for a future Livox-based
candidate.

Because the mounted inventory contains no eligible family, the fresh
holdout selection is blocked on acquiring at least three new public
LiDAR-IMU datasets (candidates include UrbanLoco HK/SF, Velodyne-based
urban sequences with RTK ground truth). No selection was preregistered and
no ground truth was opened; fresh holdout remains unopened.

## Oxford vertical-error screens (v22-v35)

The Oxford `+23.3%` trajectory gap versus GLIM is dominated by vertical
(z) error. A sequence of development-only screens tried to close it inside
Voxel-SLAM's estimator; all were rejected and the v17 configuration is
retained unchanged.

- **v22** `Odometry/z_constraint_enabled` vertical-velocity soft constraint
  gated on `HTH(5,5) < 1e-3`: the z information weight stays at
  `1e5..1.6e6` throughout Oxford, so the gate never fires. APE `0.1411`.
- **v23** vertical-velocity correction clamp: no effect, APE `0.1402`.
- **v24** `rdw_acc 1e-5` (10x lower accelerometer-bias random walk):
  APE `0.1370`, vertical `0.1220` (from `0.1247`).
- **v27** `cov_acc 0.1` + `rdw_acc 1e-5`: best single-sequence APE
  `0.1366`, vertical `0.1215`.
- **v29** add `rdw_gyr 1e-5`: `0.1378`, worse.
- **v30** `cov_gyr 0.001`: diverges (`344 m`).
- **v28** `cov_acc 0.01`: diverges (`53 m`).
- **v31** horizontal-plane normal-z emphasis (2x weight for `|nz|>0.7`):
  `0.1379`, worse.
- **v32** vertical-velocity consistency update from observed z displacement:
  `0.1366`, equal to v27 but no additional gain.

The best IMU-noise setting (`cov_acc 0.1`, `rdw_acc 1e-5`) improves Oxford
by only `2.1%` and **regresses NavINST by `22%`** (`0.1749 -> 0.2134`), so
it is not a valid common configuration under the per-sequence-tuning
prohibition. The Oxford vertical error is a deep Voxel-SLAM estimator
property (vertical-bias accumulation in open, sparsely-constrained
sections) that resists config and shallow estimator changes.

### v33 causal short-window velocity-bias consistency screen

v33 implemented the next estimator-level screen without changing the
default rival behavior. It keeps a causal three-second history of
gravity-axis position and velocity, forms robust median/MAD innovations from
each position interval versus its midpoint velocity, and applies a bounded
confidence-weighted velocity correction. The feature is enabled only in the
v33 development configurations; no dataset-specific threshold was changed.

The repository-built v33 image completed the exposed Oxford bag at rate 1.2
with `5,830` trajectory samples, `0.0087 s` end gap, `RTF 0.83517`, and
`104.50 MB` peak RSS. Against the same exposed v17 common reference, APE
RMSE was `0.1393752 m` versus the rate-1.2 v17 screen's `0.1395143 m`
(about `0.10%` better). Horizontal RMSE was unchanged (`0.06254 m` versus
`0.06252 m`), while vertical RMSE moved only from `0.12472 m` to
`0.12456 m`; the principal late error window slightly worsened. The gain is
therefore within the v17 repetition variation and does not close the
Oxford-vs-GLIM gap. v33 is retained as a reproducible development screen,
but is rejected as a promotion candidate; no NavINST/UrbanNav claim or fresh
holdout was opened.

v17 remains the promotion baseline; the v22-v35 sources and v24-v30 configs
are retained in the workspace for reproducibility but are not promotion
candidates. Fresh holdout remains unopened.

### v34 causal gravity-axis accelerometer-bias state-update screen

v34 moved the causal consistency signal one state below v33. It keeps an
eight-second gravity-axis position/velocity history, forms robust median/MAD
interval innovations, infers a bounded constant acceleration-bias error, and
updates the EKF accelerometer bias state `x_curr.ba` in the body gravity axis.
The update is causal, confidence-weighted, rate-limited, and enabled only in
the v34 development configurations; v17 default behavior is unchanged.

The repository-built v34 image completed the same exposed Oxford bag at rate
1.2 with `5,831` trajectory samples, `0.0082 s` end gap, `RTF 0.83522`, and
`104.00 MB` peak RSS. Against the same v17 common reference, APE RMSE was
`0.1391933 m` versus `0.1395143 m` for the v17 screen (`0.23%` better) and
`0.1393752 m` for v33 (`0.13%` better). Horizontal RMSE was
`0.062480 m` versus `0.062519 m` for v17; vertical RMSE was `0.124383 m`
versus `0.124722 m`. This is a small but directionally consistent Oxford
signal, not promotion evidence: it is one exposed repetition and has not yet
shown cross-dataset applicability. No fresh holdout or SOTA claim was opened.

### v34 three-dataset applicability screen

The same v34 image and globally identical opt-in configuration were then
screened once on all three exposed development datasets. All runs completed
with zero process failures, but the Oxford gain did not transfer:

| Sequence | v17 APE RMSE (m) | v34 APE RMSE (m) | Delta | v34 RTF | v34 peak RSS (MB) | Decision |
|---|---:|---:|---:|---:|---:|---|
| NavINST Indoor02 | 0.1745253 | 0.1761811 | +0.95% | 0.83618 | 129.73 | reject |
| Oxford Spires Keble 05 | 0.1395143 | 0.1391933 | -0.23% | 0.83522 | 104.00 | screen only |
| UrbanNav HK Tunnel 1 | 488.2286 | 815.3552 | +67.00% | 0.83589 | 342.29 | reject |

UrbanNav also exceeded the `330 MB` development peak-RSS gate. v34 is
therefore rejected as a promotion candidate and retained only as a
reproducible estimator screen; v17 remains the promotion baseline. The
causal accelerometer-bias update is not globally safe for the weak-axis
driving sequence, so no per-dataset retuning is accepted and no fresh holdout
was opened.

### v35 observability-gated accelerometer-bias screen

v35 kept the v34 state update but added a causal observability gate using the
existing surface-normal eigenvalues. Each gravity-axis history sample records
whether both `evalue[0]` and `evalue[1]` meet the existing `14` degeneracy
threshold, and `ba` is updated only when the complete eight-second window is
observable. The gate is global, default-off in source, and does not use
dataset identity or ground truth. Diagnostic prints expose accepted updates
and gate skips.

The v35 image completed all three exposed screens, but the gate reacted too
late for UrbanNav. The diagnostic log reached update count `200` before its
first gate skip; this did not prevent the weak-axis trajectory failure:

| Sequence | v17 APE RMSE (m) | v35 APE RMSE (m) | Delta | v35 RTF | v35 peak RSS (MB) | Decision |
|---|---:|---:|---:|---:|---:|---|
| NavINST Indoor02 | 0.1745253 | 0.1764067 | +1.08% | 0.83624 | 130.27 | reject |
| Oxford Spires Keble 05 | 0.1395143 | 0.1393970 | -0.08% | 0.83527 | 104.66 | screen only |
| UrbanNav HK Tunnel 1 | 488.2286 | 881.8388 | +80.62% | 0.83585 | 349.35 | reject |

UrbanNav also exceeded the `330 MB` peak-RSS gate. v35 is rejected as a
promotion candidate and as a sufficient weak-axis safeguard; threshold-only
iteration is not justified. The next estimator work must detect the onset of
weak-axis unobservability before the eight-second residual window accumulates,
or avoid feeding that residual into the shared inertial state. No fresh
holdout was opened.

## Remaining blockers and next steps (2026-08-09)

The v17 candidate is frozen and already beats GLIM on the exposed holdout
geometric mean (`2.28 m` vs `5.08 m`, `-55%`). It is not yet promotable
because three exposed-development gates are unmet. Each blocker and its
confirmed status are listed below so the next workstream is unambiguous.

1. **Oxford trajectory `+23.3%` versus best fixed rival (GLIM)**.
   Vertical-error dominated; fourteen screens (v22-v35: IMU-noise tuning,
   matching-structure emphasis, EKF velocity-consistency updates) improved
   Oxford by at most `2.1%` while regressing NavINST by `22%`, so no common
   configuration is valid under the per-sequence-tuning prohibition. Root
   cause is a deep Voxel-SLAM estimator property (vertical-bias
   accumulation in open sections). v33's direct velocity correction moved the
   Oxford screen by only `0.10%`; v34's accelerometer-bias state update moved
   Oxford by `0.23%` but regressed NavINST by `0.95%` and UrbanNav by `67.00%`.
   v35's surface-eigenvalue gate reacted too late, regressing UrbanNav by
   `80.62%` and exceeding the memory gate. Both mechanisms are rejected as
   global fixes. The viable path is a substantial estimator redesign that
   predicts weak-axis onset before residual accumulation, or accepting the
   limitation and keeping v17 for a future candidate revision.

2. **Oxford map geometry** `median distance ratio 1.104` (> `1.05`).
   The v17 map spreads horizontally because the built-in global HBA is
   disabled. Re-enabling it (v21) fails because Voxel-SLAM's global HBA
   assumes a one-to-one keyframe-to-scan correspondence that the no-loop
   candidate does not provide. Next viable path: a full keyframe-id to
   scan-index remap inside the global backend, or a one-to-one keyframe
   policy, both invasive; or an accepted map-quality regression for a
   future revision.

3. **UrbanNav weak-axis speed** (v17 travels `4.4 km` vs `3.15 km`
   reference). The tunnel-axis speed is unobservable from LiDAR-IMU alone;
   direct integration (v18), pre-entry bias calibration (v19), and
   turn-dynamics constraint (v20) all failed. Next viable path: an
   independent longitudinal signal (e.g. camera) or an explicit
   observability-based estimator change.

4. **Fresh holdout selection** is blocked on acquiring at least three new
   public LiDAR-IMU dataset families with ground truth, since the mounted
   inventory is fully consumed (including koide-hard via the lidarloc/GLIL
   track). The koide-hard ROS1 converter is reusable for a future
   Livox-based candidate.

Recommended order: resolve (1) or (3) with a genuinely new estimator
mechanism, then (2), then acquire fresh holdouts (4). Until a mechanism
passes all exposed-development gates, no blind holdout may be opened and no
SOTA claim is made.

## Development structure refactor (2026-08-09)

The repeated single-file screens (v23/v31/v32/zconstraint/gba) had been
kept as whole-source copies under `voxel_slam_sota_v6_*`, which made diffs
and reproducibility opaque. The structure is now patch-based:

- The frozen v17 source
  (`voxel_slam_sota_v6_weak_axis_bridge/voxelslam.cpp`) is the single
  development base.
- Each experiment is materialized as a unified patch under
  `docker/patches/voxel_slam_dev/<name>.patch` that applies to the base
  source (repository-relative `a/VoxelSLAM/src/voxelslam.cpp` headers).
- `scripts/voxel_dev_patch.py` regenerates and validates patches:
  `write --name <n> --source <path>` creates the patch and
  `check` verifies every patch applies to the base in a temp tree.
- `tests/test_voxel_dev_patch.py` locks the contract (patch existence,
  header form, apply-to-base) for `v23`, `v31`, `v32`, `zconstraint`, and
  `gba`.

New screens should be created as a patch over the base, not as a new source
copy, so the experiment lineage stays explicit and reproducible from one
revision plus one patch.

## Source structure refactor (2026-08-09)

The single `voxelslam.cpp` (3,037 lines) mixed four responsibilities. The
auxiliary classes were split into separate headers under
`voxel_slam_sota_v6_refactor/`, keeping the `VOXEL_SLAM` class and `main`
in `voxelslam.cpp`:

- `voxel_output.hpp` — `ResultOutput` (publishing)
- `voxel_io.hpp` — `FileReaderWriter` (I/O)
- `voxel_initialization.hpp` — `Initialization` (align_gravity,
  motion_blur, motion_init)
- `voxelslam.cpp` — `VOXEL_SLAM` + `main`, includes the three headers

`scripts/split_voxel_slam_auxiliary.py` performs the split by line range.
The refactored image builds cleanly and reproduces the v17 trajectory on
Oxford (`99.61%` of matched poses byte-identical; remaining difference is
thread-timing nondeterminism, APE unchanged at `~0.140 m`). This is a
pure structural move: it changes no estimator behavior. The split source is
retained as `voxel_slam_sota_v6_refactor/` for future work; the frozen v17
source remains the patch base.

A further split of the `VOXEL_SLAM` class body (matching / odometry /
backend members into separate translation units) was attempted with a
brace-matching automation script. The automation produced malformed output
(e.g. `VOXEL_SLAM::if(...)` and truncated method bodies), because the class
is tightly coupled and method-body extraction via brace matching is not
reliable for this code shape. The automation was discarded rather than
committed; the `VOXEL_SLAM` class stays in a single translation unit as a
deliberate boundary. The auxiliary-class split and patch-based experiment
management remain the supported extension mechanism.

## v36 behavior-preserving observability diagnostic (2026-08-09)

v36 was a shadow-only diagnostic over the frozen v17 base. It wrote the
candidate innovation, eigenvalue, weak-direction, and velocity-projection
signals to `v36_observability.csv`, but did not write `x_curr.ba`, change
`x_curr.v`, or alter the existing weak-axis bridge. The current diagnostic
patch is SHA-256 `be5c7e0550fa7ea727f27d3b424dda9c0c1c434a3c6e2d23560c4fa827d6b2e4`
and the built image is
`sota-voxel-slam-v36:repo-v36@sha256:8fa2c5dfb649f044493cc8b39137f4f0f962213f768f0942adb58d06ae19a1d0`.

The fixed, dataset-independent onset rule selected from the diagnostic was:

- `evalue[0] / evalue[1] < 0.2`;
- gravity-orthogonal weak-direction norm `>= 0.9`;
- absolute velocity projection `>= 3.0 m/s`; and
- five consecutive scans.

On the precise UrbanNav diagnostic CSV this rule first held at scan `38`,
sensor time `1621322433.3933244`, while the v34-style residual candidate first
became valid at scan `61`. The rule held for `1,146` scans in UrbanNav and for
zero scans in both NavINST and Oxford. The later v35 eigenvalue-14 bridge
signal remained too late. This is sufficient causal evidence for an isolated
state quarantine, but not for applying a correction to the shared inertial
state without an isolation boundary. No fresh holdout was opened.

## v37 isolated vertical-bias quarantine screen and final gate decision

v37 implements the next step only because v36 found the early UrbanNav signal.
It freezes the four global thresholds above, latches after five scans, and
accumulates the bounded candidate in the isolated scalar
`vertical_accel_bias_quarantine_shadow`. The quarantine branch deliberately
does not write `x_curr.ba`; NavINST and Oxford retain the normal v34 opt-in
path, while UrbanNav's quarantined candidate cannot contaminate shared state.
The v37 patch is SHA-256
`31a9ab38bbac99240a2a70df67b6b0cf75541555d28f09092dd0ea44301d7db1`. The
repository-built image is
`sota-voxel-slam-v37:repo-v37@sha256:a8f9c82d7d0614fb18c9eedbba8edb4d325fcc6fcca871b0ba9f0bb09fe1e23e`,
based on the frozen v17 revision `70fc8a28d63823d5989ff184daeea0787b672398`.

The same global v37 configuration, rate `1.2`, CPU set `2-7`, and no-loop
profile were run three times on each exposed development sequence. Accuracy
was scored only after all runtime runs were terminal against each sequence's
frozen common reference:

| Sequence | v17 median APE (m) | v37 median APE (m) | v37 min-max (m) | v37 delta | max RTF | max RSS (MB) | quarantine trigger / shadow updates per run |
|---|---:|---:|---:|---:|---:|---:|---:|
| NavINST Indoor02 | 0.17486253 | 0.17610831 | 0.17610359-0.17617364 | +0.71% | 0.83649 | 130.42 | 0 / 0 |
| Oxford Spires Keble 05 | 0.13948932 | 0.13942029 | 0.13941517-0.13951789 | -0.05% | 0.83535 | 104.68 | 0 / 0 |
| UrbanNav HK Tunnel 1 | 488.22565935 | 488.22794703 | 488.22556244-488.22908408 | +0.0005% | 0.83607 | 274.58 | 1 / 200 |

The v37 geometric mean is `2.28863301 m` versus v17's `2.28359687 m`
(`+0.22%`), below the frozen `2.63451414 m` frontend threshold. All nine
trajectories completed, all nine container/mapper/replay exits were clean, and
the no-loop configurations produced no accepted-loop markers. The backend
accuracy/resource gates therefore pass: every RTF is `<=0.85`, every peak RSS
is `<=330 MB`, and the UrbanNav logs show the quarantine marker plus shadow
updates without a shared `v35 vertical ba` update marker. The source contract
also locks that the quarantine branch cannot assign `x_curr.ba`.

The required map-geometry gate was not counted as a pass. A fixed-support
standardized-map reconstruction generated the three NavINST v37 maps, then
stopped while rebuilding Oxford's canonical ROS1-to-ROS2 input with
`OSError(28) No space left on device`. The archive/recovery path removed the
temporary input and restored the mounted volume; no ground-truth identity or
fresh holdout was opened. The preregistered policy says an inconclusive required
metric is a claim failure. In addition, the unresolved frozen v17 map audit
already fails UrbanNav at `0.50 m` (entropy regression `+0.15601 nats` versus
the `+0.05` limit) and the Oxford official-reference distance ratio is
`1.10425` versus the `1.05` limit. v37 therefore is **rejected for promotion**
and v17 remains the promotion baseline; the quarantine implementation is
retained as a reproducible development screen, not a SOTA claim.

Contract verification completed after the screen: `15 passed`, every tracked
development patch applied cleanly to the v17 base, Docker target
`voxel_slam_v37` was built successfully, and `git diff --check` was clean.
Fresh holdout remains unopened.

## Next workstream: independent visual longitudinal observable (2026-08-10)

The sensor inventory leaves one plausible independent observable: the camera
stream in the already frozen `lidar_imu_visual` inputs. No selected primary or
visual input contains wheel speed, GNSS, or an odometry topic. The visual
inventory is:

| Sequence | Camera messages | LiDAR messages | IMU messages | Canonical camera |
|---|---:|---:|---:|---|
| NavINST Indoor02 | 1,520 | 3,489 | 43,999 | 1280x720 mono8 |
| Oxford Spires Keble 05 | 11,544 | 5,834 | 232,128 | 1440x1080 mono8 |
| UrbanNav HK Tunnel 1 | 5,970 | 3,982 | 159,459 | 672x376 bgr8 |

The existing RKO-LIV direct visual frontend is a negative control for the
next design, not an implementation to retune. In the archived visual-track
run, UrbanNav had 245 valid direct-visual solves and 46 confidence-gated
priors, but zero fused weak directions; 178 directions were rejected as
visually unobservable. NavINST likewise fused zero directions. Repeating the
existing full 6DoF visual-prior gate or changing its thresholds is therefore
out of scope.

The candidate mechanism is a one-dimensional metric visual speed observation:

1. Time-match camera frames to LiDAR scans and use the existing calibrated
   camera model and camera--base extrinsic.
2. Track image motion with a robust 1D forward-motion/flow solve. Use
   projected sparse LiDAR depth only to make the image-derived translation
   metric; do not use the LiDAR registration residual as the speed observation.
3. Project the resulting camera-frame velocity onto the current gravity-
   orthogonal weak direction and expose only that scalar observation.
4. Keep the observation in an isolated shadow state. A bounded, stale-checked
   velocity correction may affect the weak component of the output state, but
   the route must never write `x_curr.ba` and must not replace the existing
   full pose prior.

Before implementing the estimator, a sensor-only feasibility gate must be
passed on all three exposed visual sequences with one global configuration:

- enough tracked/inlier image support for a nonzero sequence of valid metric
  1D observations;
- a bounded camera--LiDAR time residual and a stable weak-axis projection for
  at least five consecutive scan updates;
- no valid observation on a well-conditioned LiDAR direction, and no use of
  dataset identity or trajectory/reference data; and
- UrbanNav must produce sustained nonzero weak-axis observations. If it
  remains at zero, the camera route is rejected before a patch or accuracy
  screen is run.

If the feasibility gate passes, implementation and evaluation proceed in this
order: source/unit contract, one-repetition visual-track screen on all three
exposed sequences, three repetitions with the existing frontend/runtime/
resource/loop gates, then the required map-geometry audit. The visual result
remains an independent LiDAR--IMU--visual track and cannot alter the v17
primary baseline. Fresh holdout selection remains forbidden until the full
exposed gate set passes; the Oxford keyframe-to-scan map remap is a later
separate workstream.

## Sensor-only visual motion feasibility result (2026-08-10)

The first diagnostic implementation is now recorded in
[`scripts/diagnose_visual_longitudinal_observable.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/scripts/diagnose_visual_longitudinal_observable.py:1), with three synthetic contract tests in
[`tests/test_diagnose_visual_longitudinal_observable.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/tests/test_diagnose_visual_longitudinal_observable.py:1). It streams the original ROS1/ROS2 bags, decodes only the selected camera and LiDAR topics, and writes no derived bag or trajectory. The fixed configuration used `stride=1`, `max_sync_sec=0.08`, 1.5 px essential RANSAC threshold, 500 projected features, and 2,000 deterministic LiDAR points per scan. `stride=1` is required here because `stride=10` produced approximately one-second camera gaps in NavINST and invalidated the KLT assumption; this is a sampling contract, not threshold tuning.

The scale solver was corrected before the full run. The original robust Gauss--Newton update could move from a lower-cost approximately `0.3 m` solution to its artificial `0.001 m` lower bound. The replacement uses a bounded coarse global search followed by a one-dimensional projective scale equation with IRLS. The contract test suite passed `3 passed`; Python compilation and `git diff --check` also passed.

The full frozen visual inputs produced this sensor-motion result with the same configuration:

| Sequence | Camera / LiDAR records | Pairs | Valid | Valid fraction | Max streak | Direction coherence | Median scale (m) | Median speed (m/s) | Median residual | Decision |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| NavINST Indoor02 | 1,520 / 3,489 | 1,208 | 1,106 | 91.6% | 95 | 0.995 | 0.34375 | 3.438 | 0.01220 | `GO_SENSOR_MOTION` |
| Oxford Spires Keble 05 | 11,544 / 5,834 | 9,177 | 8,285 | 90.3% | 4,240 | 0.973 | 0.06873 | 1.310 | 0.00351 | `GO_SENSOR_MOTION` |
| UrbanNav HK Tunnel 1 | 5,970 / 3,982 | 4,789 | 2,191 | 45.8% | 280 | 0.849 | 0.31250 | 4.145 | 0.01286 | `GO_SENSOR_MOTION` |

The accepted pairs are all within the fixed 80 ms camera--LiDAR matching bound; rejected records are retained in the report as time-mismatch, track, or pose-inlier counts. No ground-truth, identity, reference trajectory, or map metric was opened. The small JSON reports are under `/media/sasaki/aiueo/benchmarks/sota_v5/diagnostics/visual_longitudinal_v38_preflight/`.

This closes only the independent metric camera-motion sub-gate. The full weak-axis gate remains **pending**: the script currently reports the motion direction in the calibrated base frame, but does not yet join each pair to the runtime gravity-orthogonal LiDAR weak eigenvector and its state attitude. Existing v36 sensor-only diagnostics provide the next join target: the fixed v36 onset rule had zero qualifying rows in NavINST and Oxford and 1,146 rows with a maximum 127-row streak in UrbanNav. UrbanNav's visual motion direction was predominantly base `y` (median absolute component `0.991`), which is a promising nonzero signal but is not counted as a completed weak-axis projection until the timestamped attitude join is implemented.

Next action is therefore a report-only timestamp/attitude join using the existing v36 runtime diagnostic and IMU/state frame, checking five consecutive nonzero gravity-orthogonal weak-axis projections and rejecting observations during well-conditioned LiDAR periods. Only if that gate passes will the isolated scalar shadow observation source contract be implemented. The v17 primary baseline, Voxel state, fresh holdout, and accuracy screen remain untouched.

## Runtime weak-axis join result and implementation boundary (2026-08-10)

The report-only join is implemented in
[`scripts/join_visual_weak_axis_diagnostic.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/scripts/join_visual_weak_axis_diagnostic.py:1), with contract coverage in
[`tests/test_join_visual_weak_axis_diagnostic.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/tests/test_join_visual_weak_axis_diagnostic.py:1). It uses runtime odometry attitude, the v36 weak-eigen diagnostic, and an initial five-second IMU gravity estimate. It never opens a reference trajectory or ground-truth file. A v36 NavINST CSV had timestamp precision collapsed to one value; the join detected this and used an explicit runtime-position-aligned scan-order offset of `9`, with `3.2e-9 m` median alignment error. Oxford and UrbanNav used precise timestamp joins.

The same weak-axis definition (`evalue0/evalue1 < 0.2`, gravity-orthogonal weak norm `>=0.9`, pair/runtime join `<=80 ms`) gave:

| Sequence | Joined valid visual pairs | Weak-eligible pairs | Well-conditioned pairs suppressed | Nonzero weak-axis scan streak | Strong `>=3 m/s` streak | Decision |
|---|---:|---:|---:|---:|---:|---|
| NavINST Indoor02 | 1,106 | 0 | 1,106 | 0 | 0 | no-op |
| Oxford Spires Keble 05 | 8,285 | 2 | 8,283 | 2 | 0 | no-op |
| UrbanNav HK Tunnel 1 | 2,190 | 775 | 1,415 | 89 | 34 | `GO_WEAK_AXIS_PROJECTION` |

For UrbanNav, the weak-axis projection absolute median is `4.115 m/s`, p10 `1.315 m/s`, and p90 `7.985 m/s`; the maximum join residual is `75.6 ms` and p95 is `29.0 ms`. The nonzero threshold was `0.1 m/s`; the separate `3.0 m/s` threshold is the frozen v36 strong-motion reference. The result supports a weak-axis-only observation: NavINST and Oxford remain unchanged because their runtime diagnostics are well-conditioned or too short-lived, while UrbanNav has the required sustained nonzero signal. These are sensor-only results, not accuracy claims.

The full sensor feasibility boundary is therefore **passed for a conditional weak-axis route**: metric visual motion is available on all three tracks, the visual observation is suppressed on well-conditioned directions, and UrbanNav supplies the sustained weak-axis case. The next authorized implementation step is only the source/unit contract for an isolated scalar shadow observation with bounded/stale checks. It must not replace the full visual pose prior, write `x_curr.ba`, or affect the v17 primary path until a separate one-repetition screen is explicitly completed. The join reports are under `/media/sasaki/aiueo/benchmarks/sota_v5/diagnostics/visual_longitudinal_v38_preflight/`.

## Isolated scalar shadow source/unit contract (2026-08-10)

The conditional route now has a source-side contract in
[`docker/patches/voxel_slam_dev/v38_visual_longitudinal_shadow.patch`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/docker/patches/voxel_slam_dev/v38_visual_longitudinal_shadow.patch:1). The patch is derived from the frozen v17 source and is disabled by default. It adds only an isolated scalar shadow observation record containing `stamp_sec`, signed metric `velocity_mps`, and normalized `confidence`; it does not alter the inertial pose, velocity, position, or bias state.

The receiver accepts an observation only when the feature is enabled, all values are finite, confidence is in `[0,1]`, speed is within the configured bound (`20 m/s` default), and the measurement timestamp is finite, not in the future, and no older than the configured `0.2 s` maximum age. Reset clears the shadow validity and counter. No ROS topic, camera producer, estimator correction, or output consumer is wired yet; this is intentionally a source/unit boundary, not a candidate runtime or accuracy experiment.

The patch checker and initial contract tests pass (`12 passed` across the v38 candidate, patch, visual diagnostic, and weak-axis join suites); compilation and `git diff --check` pass. The source-contract patch SHA at this checkpoint was `8a2b1cf6cc46ea5a9d853ac77504ca6f020b63be700b7827c126a4d1657f2ee4`; the later opt-in receiver/consumer revision is recorded below. The next gate is an isolated report-only producer adapter that converts the already validated weak-axis visual pair into this scalar contract, followed by one authorized one-repetition screen. Until that screen is approved and passes, v17 remains the primary path and no fresh holdout is opened.

## Report-only scalar producer result (2026-08-10)

The producer adapter is implemented in
[`scripts/emit_visual_weak_axis_shadow.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/scripts/emit_visual_weak_axis_shadow.py:1), and the join now exposes deterministic per-scan observations in
[`scripts/join_visual_weak_axis_diagnostic.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/scripts/join_visual_weak_axis_diagnostic.py:1). The adapter consumes only the ground-truth-free pair report and the passed weak-axis join report. It emits the exact receiver fields `stamp_sec` (seconds), signed `velocity_mps` (m/s), and normalized `confidence` (`[0,1]`), plus provenance metadata; it does not publish a ROS topic or call the estimator.

Using the existing UrbanNav reports, the detailed join contained `675` weak-eligible per-scan observations. With the receiver speed bound `20 m/s` and producer confidence floor `0.2`, `534` observations were emitted and `141` were rejected at the confidence bound. The emitted confidence range was `0.2000`--`0.7039` with median `0.3850`. The generated report is `/media/sasaki/aiueo/benchmarks/sota_v5/diagnostics/visual_longitudinal_v38_preflight/urbannav_visual_weak_axis_shadow_v38.json`; the detailed join is `/media/sasaki/aiueo/benchmarks/sota_v5/diagnostics/visual_longitudinal_v38_preflight/urbannav_weak_join_v3.json`.

The adapter contract tests, join tests, visual diagnostic tests, v38 patch tests, and patch checker pass (`14 passed` in the combined Python suite); compilation and `git diff --check` pass. This completes the report-only producer boundary. A runtime camera/ROS bridge, Voxel correction consumer, and one-repetition accuracy screen are still intentionally not executed because the v38 source patch remains a disabled shadow receiver with no estimator-state mutation. v17 remains primary and no fresh holdout or ground-truth input has been opened.

## Visual transform correction and final v38 candidate screen (2026-08-10)

Before the candidate screen, a source-contract audit found two issues in the initial report chain. The visual diagnostic had reused the LiDAR-to-camera rotation when expressing camera-center displacement in the base frame; it now keeps LiDAR-to-camera projection and camera-to-base motion rotation separate. The weak-axis join also now resolves arbitrary eigenvector sign using the runtime world velocity, matching the frozen Voxel bridge convention that orients its weak direction toward the current velocity. The original v38 preflight/join/payload files and the first two candidate runs are retained as superseded artifacts, not final evidence.

The corrected fixed-configuration sensor-only results are:

| Sequence | Valid visual pairs | Direction coherence | Median scale (m) | Median speed (m/s) | Corrected weak-eligible pairs | Nonzero streak | Strong `>=3 m/s` streak | Decision |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| NavINST Indoor02 | 1,106 | 0.995 | 0.34375 | 3.438 | 0 | 0 | 0 | no-op |
| Oxford Spires Keble 05 | 8,285 | 0.973 | 0.06873 | 1.310 | 2 | 2 | 0 | no-go |
| UrbanNav HK Tunnel 1 | 2,191 | 0.849 | 0.31250 | 4.145 | 775 | 89 | 40 | `GO_WEAK_AXIS_PROJECTION` |

The corrected UrbanNav join has projection absolute median `4.137 m/s`, p10 `1.315 m/s`, p90 `8.035 m/s`, join maximum `75.6 ms`, p95 `29.0 ms`, and positive weak-axis/runtime-velocity alignment median `6.127 m/s`. The corrected producer again emits `534` observations from `675`, with `141` rejected below confidence `0.2`. The final report is `/media/sasaki/aiueo/benchmarks/sota_v5/diagnostics/visual_longitudinal_v38_preflight/urbannav_visual_weak_axis_shadow_v41.json` and the join is `/media/sasaki/aiueo/benchmarks/sota_v5/diagnostics/visual_longitudinal_v38_preflight/urbannav_weak_join_v41_signed.json`.

The candidate-only source patch was expanded to an opt-in `geometry_msgs/Vector3Stamped` receiver and a bounded weak-axis-speed consumer: source timestamp in the header, signed m/s in `vector.x`, confidence in `vector.y`, gain `0.25`, and maximum weak-speed change `0.5 m/s` per update. The receiver remains disabled by default; only the v38 UrbanNav config enables it. The image built successfully as `sota-voxel-slam-v38:repo-v38@sha256:3b8fb3c48bbc3e1d3eca0dbbc9a1cfa2da6097aa0939815da7af242c382b5d42`, with patch SHA `fe9c313a9cacaffce333f9518f4c0c26cf955741dc9c44961ade9995b6a6e3b8`.

The final corrected-payload runtime was one UrbanNav replay at rate `1.2` using the frozen `dev_v34` canonical ROS1 bag. It completed cleanly (`replay=0`, `mapper=0`); the replay bridge validated/published `534/534` observations with zero late skips, and the mapper logged `600` applied shadow updates. The runtime RTF was approximately `0.844`, but peak RSS was `342,604 kB = 334.57 MB`, exceeding the frozen `330 MB` limit. After the run completed, the existing UrbanNav common reference was used for the single accuracy screen: APE RMSE was `834.3353 m`, versus the retained v17/v37 one-run reference `488.2263 m` (`+70.89%`). No new reference or holdout was opened.

The corrected v38 route therefore **fails the one-repetition accuracy and resource gates** and is rejected for promotion. The map-geometry gate was not run after this failure. v17 remains the primary baseline; the v38 source/config/replay artifacts are retained for diagnosis only. The combined contract suite now passes `20` tests, the full patch checker passes, C++ compilation passes, and `git diff --check` is clean.

## v38 causal audit and v39 output-only visual rejection (2026-08-10)

The v38 failure was reproduced without opening any reference trajectory. The
ground-truth-free audit in
[`scripts/audit_visual_shadow_runtime.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/scripts/audit_visual_shadow_runtime.py:1)
compares only the behavior-preserving v37 output, the rejected v38 output, the
frozen visual payload, and the v38 source patch. It found all four prohibited
runtime properties: wall-clock rather than estimator-state timing, repeated
consumption of one observation, a scalar projected onto a producer-side frozen
axis, and correction feedback into mapper state. The two trajectories were
identical until the first correction application, `122.802 s` after the first
payload observation. Their position difference then crossed `1/10/100/500 m`
after `124.002/126.708/132.211/220.987 s`, reached a maximum of
`1,823.210 m`, and increased path length from `4,572.247 m` to `5,113.177 m`.
The audit decision is `FAIL_VISUAL_SHADOW_RUNTIME_CONTRACT`; its retained
report is
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_v39_output_shadow_20260810/v38_runtime_audit.json`.

v39 tests the narrowest safe alternative: a deterministic output-only shadow
trajectory. The mapper, map, inertial state, and baseline orientation are
immutable. The producer in
[`scripts/emit_visual_velocity_vector_shadow.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/scripts/emit_visual_velocity_vector_shadow.py:1)
keeps the complete calibrated base-frame velocity vector. The consumer in
[`scripts/compose_visual_longitudinal_shadow_trajectory.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/scripts/compose_visual_longitudinal_shadow_trajectory.py:1)
projects each observation against the timestamp-matched behavior-preserving
state, consumes it at most once, and changes only the weak component of a new
output trajectory. The same world-frame translation is applied to the
reference-point output while its orientation is preserved. The fixed global
configuration is
[`configs/voxel_slam_v39/output_only_visual_shadow.yaml`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/configs/voxel_slam_v39/output_only_visual_shadow.yaml:1):
eigenvalue ratio `<0.2`, horizontal weak norm `>=0.9`, baseline weak speed
`>=3 m/s`, five consecutive scans, `80 ms` join, `200 ms` maximum age, gain
`0.25`, and maximum change `0.5 m/s` per accepted observation. No dataset
identity or ground truth is an input to either stage.

The hash-sealed pre-accuracy result was:

| Sequence | Vector observations | Applied once | Rejected join / gate | Output decision | Baseline preservation |
|---|---:|---:|---:|---|---|
| NavINST Indoor02 | 0 | 0 | 0 / 0 | `NO_OP_OUTPUT_SHADOW` | raw and reference files byte-identical |
| Oxford Spires Keble 05 | 0 | 0 | 0 / 0 | `NO_OP_OUTPUT_SHADOW` | raw and reference files byte-identical |
| UrbanNav HK Tunnel 1 | 533 | 346 | 6 / 181 | `GO_OUTPUT_ONLY_SHADOW_SCREEN` | orientation unchanged; map and mapper untouched |

UrbanNav path length changed from `4,572.2474 m` to `4,493.5887 m`; maximum
and final position differences from the baseline were `158.1319 m` and
`157.4931 m`. A second independent composition produced byte-identical raw and
reference-point trajectories (SHA-256
`d125a42a9a7d3d1ff5e9c2f4bbde3d71cc93974ba95650e4dc3c95c581d351d9`
and
`bbff9328e2f650d919ccbc82b1f39e0a1e8f9ceb21c9c7b0a33a86a8f3f71671`).
The measured offline composition cost was `0.36 s` wall time and `57,984 kB`
peak RSS; it does not run concurrently with or feed back into the mapper.

Only after those outputs were sealed was the existing UrbanNav common
reference opened for the one authorized accuracy screen. With the same 395
interpolated pairs and zero rejected reference points, the behavior-preserving
baseline measured `488.2306509 m` APE RMSE and v39 measured `520.8089392 m`:
`+32.5782883 m`, or `+6.6727%`. v39 therefore **fails the one-repetition
accuracy gate and is rejected**. No gain, threshold, sign, or timing parameter
is retuned after this result; no three-repetition or map-geometry gate is run,
and no fresh holdout is opened. v17 remains the primary baseline. All v39
evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_v39_output_shadow_20260810/`.

The visual weak-axis route is now a retired negative control on these consumed
development sequences. Its remaining value is architectural: output isolation
prevents the catastrophic nonlinear map feedback seen in v38, but the camera
velocity itself is not accurate enough to improve the frozen trajectory.

## v40 full-scan GBA graph audit and sealed Oxford rejection (2026-08-10)

The initial remap premise was disproved before implementation. In the frozen
source, `Keyframe::id` is already the source scan index, and HBA edges preserve
that namespace. The v21 failure came from initializing GTSAM with only sparse
keyframe IDs while declaring and writing back a dense range through the last
keyframe. It also omitted the dense odometry chain and anchor prior. Changing
the ID namespace would therefore have hidden the defect rather than fixed it.

The ground-truth-free audit in
[`scripts/audit_v40_gba_graph_contract.py`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/scripts/audit_v40_gba_graph_contract.py:1)
replayed the fixed `window=10`, `stride=1`, `minimum distance=5 m` selection
against behavior-preserving trajectories. All IDs were unique, strictly
increasing, and within the source-scan range:

| Sequence | Scans | Sparse keyframes | First-last scan ID | v21 missing initialized keys | Trailing scans outside v21 range | Full-graph odometry edges | Decision |
|---|---:|---:|---:|---:|---:|---:|---|
| NavINST Indoor02 | 3,485 | 141 | 9-3,289 | 3,149 | 195 | 3,484 | `GO_FULL_SCAN_GRAPH_RESTORATION_NO_ID_REMAP` |
| Oxford Spires Keble 05 | 5,831 | 123 | 9-5,759 | 5,637 | 71 | 5,830 | `GO_FULL_SCAN_GRAPH_RESTORATION_NO_ID_REMAP` |
| UrbanNav HK Tunnel 1 | 3,980 | 313 | 9-3,979 | 3,667 | 0 | 3,979 | `GO_FULL_SCAN_GRAPH_RESTORATION_NO_ID_REMAP` |

The reports are retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_v40_gba_graph_audit_20260810/`.
Their SHA-256 values are `33237d60...` (NavINST), `6ad4352d...`
(Oxford), and `267929cb...` (UrbanNav). Synthetic contracts cover contiguous,
dropped/noncontiguous, duplicate, and out-of-range IDs without mutating a
trajectory or map.

The opt-in/default-off
[`docker/patches/voxel_slam_dev/v40.patch`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/docker/patches/voxel_slam_dev/v40.patch:1)
(SHA-256 `890c351d05938921c427c577ce89f348c5ee39f393bf6a9e890724c1a6b3b4e6`)
restores every scan as a graph variable, adds `N-1` odometry factors and a
prior on scan zero, and fails closed on invalid keyframe IDs or variances. The
three configurations differ from v17 only in the global-GBA contract. The
patched `voxelslam.cpp` SHA-256 is
`185be253dcf42a2d1917739fdc643b20b02a84358cdc4c31cef3171a3a48f86f`;
the source compiles and links in the `voxel_slam_v40` Docker target.

One Oxford runtime-only screen was then sealed before any accuracy or map
reference was opened. It used canonical bag SHA-256
`8e7660e600f1f52758e0301f861a2174500ef38f60df5ac9f9d7198df9c54217`,
configuration SHA-256
`6844604a99781fe0e86d7665227f4be6495022a272274012b33e30ee358e7800`,
rate `1.2`, CPUs `2-7`, and execution image ID
`sha256:48d5f6fe57e2bb69f16f3e1562d91d7a06e59ae1823983d3dbbc9f23cd65871a`.
Bag replay exited zero and the official `/tf` stream produced 5,831 valid
samples through the input endpoint. The global backend did not finish within
the 600 s finalization allowance, however: mapper exit was `143`, container
exit was `1`, `trajectory_complete=false`, no `alidarState.txt` or map output
was finalized, and no GBA completion marker was flushed. The 5,823 frontend
PCD chunks were retained, but without the state file the runner could not
assemble a world map. Total processing wall
time was `1,085.6503 s` for a `580.9839 s` bag (`RTF 1.86864`). A live process
sample reached `374,840 kB = 366.05 MiB`, already above the `330 MiB` gate;
because the forced shutdown left `mapper_process_time.txt` empty, the formal
run record correctly leaves `peak_rss_mb=null` rather than presenting the
sample as a persisted peak.

The retained runtime evidence is under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v40/oxford_spires_keble_05/runtime_v40_full_scan_gba/`.
The sealed SHA-256 values are `bc96b13b...` for `summary.json`, `23a924e7...`
for `run.json`, `470597c3...` for the raw trajectory, and `52bbea4e...` for
`mapper.log`. `accuracy_ground_truth_accessed=false` and
`accuracy_metrics_present=false`; neither an accuracy scorer nor a map
geometry scorer was run.

The execution image inherited a stale v17 `voxelslam.cpp` label even though
its patch label was correct. This provenance-only defect was fixed after the
failed run and locked by a test. The same compiled source was rebuilt as image
ID `sha256:5d07f7b2edd05c1e1eb38e5c30399f7f6dfb07b4792f069ce49bbe1fbb19f8d7`;
its source file and label both report `185be253...`. The failed execution is
still attributed to its original image ID and was not rerun or reclassified.

v40 therefore **fails completion, runtime, and resource gates before
accuracy** and is rejected. No parameter was tuned from this result, no global
three-sequence gate is authorized, and the fresh holdout remains unopened.
v17 remains the primary baseline.

## v41 cancellable/resource-bounded HBA rejection (2026-08-10)

The v40 source audit found a more precise completion cause than raw optimizer
cost. Its standalone worker could observe `is_finish && gba_flag == 0` and
exit before the producer set the request. The producer would subsequently set
`gba_flag=1` and enter `while(gba_flag);`, with no worker left to acknowledge
it and no sleep or ROS shutdown check. This request-before-exit race explains
the silent one-core busy-wait and forced v40 termination.

v41 replaces that handshake with atomic request and producer-done state. A
standalone worker may exit normally only after producer-done and a cleared
request; the producer wait yields and checks ROS shutdown. The default-off
runtime guard adds flushed stage/RSS/HWM markers, a global 330 MiB ceiling,
and a 30 s backend deadline. Cancellation skips GTSAM writeback and map-topic
publication, then saves the unchanged state so the runtime harness can close
normally. The patch is
[`docker/patches/voxel_slam_dev/v41.patch`](/home/sasaki/workspace/old_~2026/lidarslam_ws/lidar_slam_ros2_kaizen_review/docker/patches/voxel_slam_dev/v41.patch:1)
(SHA-256 `69dcfede4d80fcabc4bc04d8846d4297f116798eb68ed8f2fc9d098a0d776c77`),
the patched source SHA-256 is
`e6516064abe6a16876af2b9b3e7cfb61519d992562b96f64bec484a5f963d66e`,
and the Release image is
`sha256:a72cacc930adf576e6572e1063e54a4948620c352ea10a947bf8aba30d645427`.
The image label and in-image source digest agree.

The ground-truth-free lifecycle audit covers success, RSS cancellation before
a request, and deadline cancellation after a request. It requires worker
acknowledgement before writeback and proves that both cancellation paths exit
without writeback or map publication. Its decision is
`GO_V41_RUNTIME_ONLY_DIAGNOSTIC`; the retained report SHA-256 is
`c3afbaf79bab3350426d7a657a29d40dbd1a9a5534421afef23d44c1cd81d366`.

The single Oxford runtime-only execution used the same bag, rate `1.2`, and
CPUs `2-7`, with config SHA-256
`e5055d4c4ce47cbe6ac44237bceba3f68d78b21ae592048a9dc5c1464d6e1f10`.
It completed cleanly: replay, mapper, and container all exited zero; all 5,831
trajectory samples were present with a `0.00264 s` end gap. Processing took
`485.2646 s` for `RTF 0.835246`, inside the `0.85` runtime limit.

The resource result rejects the backend. After 90 keyframes and 17 completed
bottom-up windows, the seventeenth local cloud merge raised HWM to
`352,172 kB = 343.91797 MiB`. The guard emitted `cancel_rss_limit` at source
scan 4,360, followed by `worker_exit_cancelled`; current RSS fell from
`337,508 kB` to `253,956 kB`. The frontend then completed all 5,823 saved
states/chunks and emitted `backend_no_writeback` and
`state_saved_unmodified`. No `worker_request`, ISAM, writeback-complete,
map-publish-complete, or optimized-state marker exists. Thus the fixed RSS
gate is exceeded during bottom-up submap construction, before the restored
full-scan graph or top-down optimizer is even requested.

The cancellation fallback is exactly behavior-preserving. Its
`alidarState.txt` SHA-256
`7f531a4121fc76afda19e3ac3b7cb7d075cc4d45884b64fc3ccf6c53ca92b522`
is byte-identical to all three retained v17 Oxford repetitions. The assembled
30,901,030-point map SHA-256
`2d0889336140638d3b6c87f2e4d0cf26dfe4b951d8a1ec8c7138032b55b9e4b4`
is also byte-identical to v17. It is not an optimized v41 map and is not sent
to an accuracy or geometry scorer.

The sealed v41 summary/run/log SHA-256 values are `7c591c52...`,
`ab7ebe51...`, and `3f72a8ed...`. The independent runtime audit verifies clean
completion, cancellation/no-writeback, resource failure, config binding, and
v17 fallback identity; its decision is
`REJECT_V41_RESOURCE_GATE_RETIRE_BUILTIN_HBA` and its report SHA-256 is
`34a1e57c9af83fdb748fd0e05689b7268075488ca3f662951f94f166e54cf90c`.
All evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v41/` and
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_v41_gba_lifecycle_audit_20260810/`.

v41 therefore fixes shutdown, observability, and fail-closed behavior, but
**fails the RSS gate and produces no map improvement**. The built-in HBA route
is retired for this candidate family; its keyframe distance, stride, voxel
size, and memory threshold will not be tuned from Oxford. v17 remains primary,
and no accuracy reference, reference map, or fresh holdout was opened.

## v42 preregistered streaming external pose-graph feasibility

The remaining Oxford map blocker must now be addressed outside the retired
in-process HBA. The next stage is report-only and fixed in this order:

1. Inventory the retained v17 scan chunks and poses on all three consumed
   sequences, then identify temporally separated revisit candidates from the
   candidate trajectory only. Ground truth and reference maps are prohibited.
2. Build fixed-budget submaps by streaming chunks from disk. Geometry-verify
   revisit pairs with one global set of distance, overlap, and residual gates;
   retain rejected pairs and peak working-set measurements. No trajectory or
   map may be changed in this stage.
3. Add synthetic contracts for transform direction, frame identity, duplicate
   edges, disconnected graphs, invalid covariance, deterministic ordering,
   and fail-closed memory exhaustion.
4. Only a report with repeatable, nonconflicting verified constraints on all
   applicable sequences authorizes an opt-in external sparse pose graph over
   the frozen v17 odometry chain. The online estimator remains untouched.
5. A built candidate must first pass one Oxford runtime/RSS/completion screen.
   Accuracy and map geometry remain closed until that screen passes; one
   scored failure ends v42 without threshold tuning.

## v42 streaming external pose-graph feasibility result (2026-08-10)

The five-stage order above was executed through its report-only authorization
gate. The global contract is
`configs/sota_v6/development/v42_streaming_pose_graph_audit.json` (SHA-256
`c43475b61f87d621efd5528e034149b66e1c473f93e1206f4b3b1d7e21e446fb`),
and the implementation is
`scripts/audit_v42_streaming_pose_graph_feasibility.py` (SHA-256
`c266e10a7334cafdd6487519a2fc11a51fd6f5d5d58063db08299d75a3ec4253`).
The contract contains no dataset-specific branch. It reconstructs the exact
1.5 m v17 anchor policy, requires 100 m travel separation, searches a 20 m
candidate radius, and tests a five-submap sequence at offsets
`[-6,-3,0,3,6]`. Each anchor-local submap streams at most three chunks, is
0.5 m voxelized and capped at 6,000 points, and uses a three-entry LRU. The
process RSS ceiling is 256 MiB.

The new discriminator combines a yaw-invariant 20-ring/60-sector place
descriptor with candidate-episode uniqueness, independent constrained ICP at
all five offsets, mutual overlap/residual limits, and a requirement that one
world-frame correction explain the full sequence within 0.5 m and 2 deg.
The complete edge set from the previously rejected 8 m profile is retained as
a fail-closed challenge set: surviving its eight-anchor dedup neighborhood is
an automatic rejection, not an accuracy label. The script has no ground-truth,
accuracy-reference, reference-map, or optimized-trajectory input surface.

Six real-data audits completed, two per consumed sequence:

| Sequence | Anchors | Episode queries | Descriptor-qualified | Geometry pass before dedup | Verified constraints | Legacy survivors | Peak RSS range | Decision hash |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| NavINST indoor02 | 575 | 302 | 36 | 0 | 0 | 0 | 79.90–80.16 MiB | `1336b00a...` |
| Oxford spires_keble_05 | 443 | 193 | 21 | 15 | 3 | 3 | 81.86–81.97 MiB | `ce111e88...` |
| UrbanNav HK tunnel 1 | 2,005 | 204 | 65 | 0 | 0 | 0 | 95.98–95.99 MiB | `c2c731ef...` |

Each sequence's two deterministic payload SHA-256 values are byte-identical;
wall times were 18.46/19.87 s, 16.80/17.51 s, and 50.07/47.04 s,
respectively. The 16 synthetic contracts pass and cover transform direction,
`map <- base_link` identity, duplicate constraints, disconnected graphs,
invalid covariance, deterministic ordering, strict PCD/state parsing, and
fail-closed memory exhaustion.

The Oxford result is decisive. Its three deduplicated constraints were
`6 -> 303`, `70 -> 365`, and `164 -> 425`, with minimum five-sequence mutual
overlap `0.8613`, `0.8869`, and `0.8698`. Despite that strong local geometry,
all three match clusters from the already rejected edge set: respectively
`{3 -> 300, 14 -> 311}`, `{66 -> 362, 76 -> 371}`, and
`{165 -> 427}`. Long-sequence geometric consistency therefore still cannot
identify the Oxford repeated structures. There are zero new verified
constraints across all three sequences.

The report-only invariant also passed. State/map SHA-256 pairs remained
`4162962d...`/`be95206f...`, `7f531a41...`/`2d088933...`, and
`d91e54f0...`/`87caad98...` for NavINST, Oxford, and UrbanNav. No trajectory,
pose graph, or map output was written. The aggregate report is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v42_streaming_pose_graph_audit_20260810/`;
its file SHA-256 is
`b00ccca9d286d9239ff76b0a67e6ab8d1fdfa508b65b9c0a04764a0647d156cd`
and its deterministic payload SHA-256 is
`63c7176f74d3963936b2be3bfe2a954140e58d9261fc2f7363d840ffa9fd865a`.

The sealed decision is **`REJECT_V42_LEGACY_AMBIGUITY_SURVIVED`**.
External sparse pose-graph implementation is not authorized, so the Oxford
runtime/RSS candidate gate and all accuracy/map scoring remain unopened.
v42 thresholds are retired without tuning, the online estimator remains
untouched, and v17 remains the primary behavior.

## v43 preregistered independent place-identity feasibility

Another geometry-only loop gate is not justified. Any v43 revisit proposal
must first add an independent place-identity signal, such as temporally
separated camera appearance or calibrated LiDAR reflectivity, and demonstrate
that it rejects every retained v42 Oxford ambiguity before producing an edge.
The first stage remains report-only, globally configured, fixed-memory, and
forbidden from opening accuracy/reference-map inputs. If the retained sensor
inputs cannot supply an independent signal on all applicable sequences, the
Oxford global-correction route closes and development returns to a new local
frontend estimator rather than tuning loop thresholds.

## v43 independent place-identity availability result (2026-08-10)

The retained-signal gate was implemented before any new matcher or edge
screen. The global contract is
`configs/sota_v6/development/v43_place_identity_audit.json` (SHA-256
`39fa10345406b3a1019c5acfe6540705c11fdee3cc47f40c755b25720f427eca`),
and the bounded streaming implementation is
`scripts/audit_v43_place_identity_feasibility.py` (SHA-256
`88475a67bac7cbf51ae42d7c2ff22bb83789961ca6ee0ffa35ed77c44b348087`).
The contract has no dataset branch. It requires finite, nonconstant XYZI
intensity with at least 1% nonzero support, 16 distinct values, and 90% of
chunks varying, or an explicit sequence-bound camera manifest. It reads one
PCD at a time under a 128 MiB RSS ceiling. Geometry, loop edges, trajectories,
accuracy references, and reference maps are not inputs.

The complete retained chunk set was scanned twice for each sequence:

| Sequence | Chunks | Intensity points | Nonzero | Dynamic range | Distinct values | Varying chunks | Camera retained | Peak RSS range | Decision hash |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | ---: | --- |
| NavINST indoor02 | 3,477 | 11,727,179 | 0 | 0 | 1 | 0 | no | 38.05–38.07 MiB | `63f72511...` |
| Oxford spires_keble_05 | 5,823 | 30,901,030 | 0 | 0 | 1 | 0 | no | 40.31–40.34 MiB | `f5d72be1...` |
| UrbanNav HK tunnel 1 | 3,972 | 17,711,712 | 0 | 0 | 1 | 0 | no | 38.93–38.94 MiB | `65f89f37...` |

All `60,339,921` retained intensity values are finite but exactly `0.0`.
The per-sequence intensity-payload SHA-256 values are `38c4b1d3...`,
`6634efba...`, and `710dd1a3...`; each decision payload is byte-identical
across the two runs. No camera manifest or image payload is retained in any
v17 run, and the original canonical LiDAR/IMU bag paths recorded by the runs
are not currently mounted. The 12 synthetic contracts pass for zero and
valid signals, nonfinite values, malformed/oversized PCDs, state ordering,
camera sequence binding, memory exhaustion, protected-input identity, and
repeatability.

State/map SHA-256 pairs remained `4162962d...`/`be95206f...`,
`7f531a41...`/`2d088933...`, and `d91e54f0...`/`87caad98...`. No trajectory,
map, matcher, or pose-graph output was produced. Evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v43_place_identity_audit_20260810/`.
The aggregate file SHA-256 is
`a43885ffa7ab8d2ca8859e76a0cbcb67895a44433b690856a4ade521ebb92596`,
and its deterministic payload SHA-256 is
`246beb418b62a3dbe57739f25fe940d0eac3ddb0a712dc900393987e341e7b2d`.

The v43 decision recorded at that stage was (superseded by v43b below)
**`CLOSE_OXFORD_GLOBAL_CORRECTION_ROUTE_NO_INDEPENDENT_PLACE_IDENTITY`**.
A reflectivity/appearance challenge matcher is not authorized because there
is no independent signal to consume. This closes the v17 Oxford global-loop,
external pose-graph, and map-refinement branch without threshold tuning.
v17 remains primary; accuracy, map scoring, and fresh holdout stay closed.

## v43b exact-raw correction and place-identity challenge (2026-08-10)

The v43 retained-PCD measurement remains valid, but its source-availability
inference is superseded. A later read-only inventory found exact-byte copies
of all three canonical ROS1 bags under
`/media/sasaki/aiueo/benchmarks/quarantine/opencode_inputs_20260810/`.
Their SHA-256 values exactly match the original run bindings:
`b8afd9649a310669...` for NavINST, `8e7660e600f1f527...` for Oxford, and
`95524232d9a4c278...` for UrbanNav. Thus the saved v17 PCD chunks do contain
only zero intensity, but they are not the only retained source from which an
independent signal can be recovered. No v43 evidence was rewritten.

The corrective gate was frozen before reading any Oxford identity score. Its
global contract is
`configs/sota_v6/development/v43b_raw_intensity_identity_audit.json`
(SHA-256 `28e5a6b8216f33e1d8a38ca2773b41cbb647696c3283be4e7c295269031e0c1f`),
the exact source binding is
`configs/sota_v6/development/v43b_raw_intensity_sources_20260810.json`
(SHA-256 `c24f4443367e32507ed2af81f88a5dc9b0c95d775fc0b2fddcfed80f3d3b6f2a`),
and the report-only implementation is
`scripts/audit_v43b_raw_intensity_identity.py`
(SHA-256 `59377380bb245185d297294c25fd4d25bace2f472fa8964d36d54edee2bdf717`).
The contract has no dataset branch. It requires the exact common 48-byte
PointCloud2 layout, header-to-state matching within 1 ms, at least 90%
nonzero intensity, a 16-unit dynamic range, 32 distinct values, and 90% of
scans varying. Raw points are transformed by the frozen v17 convention
`p_body = R_body_lidar p_lidar + t_body_lidar`, range filtered to 1–50 m,
0.5 m voxelized, and bounded to 12,000 points per selected scan and 6,000
points per three-scan submap under a 256 MiB RSS ceiling.

Every retained v42 geometry pass is then challenged before edge creation.
Each of its five registered submap pairs must have at least 128 reciprocal
0.5 m correspondences, 5% support, nonconstant overlap intensity,
overlap-local Pearson correlation at least 0.6, and a correlation peak margin
of at least 0.08 over four fixed 1 m spatial decoys. Ground truth, reference
maps, accuracy trajectories, optimized maps, and graph writeback are absent
from the interface.

The exact bags were scanned twice per sequence:

| Sequence | Raw scans | Raw intensity points | Nonzero fraction | Dynamic range | Distinct sampled values | Varying scans | Peak RSS range | Decision hash |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| NavINST indoor02 | 3,489 | 89,292,327 | 99.0613% | 255 | 240 | 3,489/3,489 | 67.39–67.41 MiB | `ba6f0d73...` |
| Oxford spires_keble_05 | 5,834 | 345,360,040 | 99.999993% | 255 | 118 | 5,834/5,834 | 151.14–151.29 MiB | `9a934fbb...` |
| UrbanNav HK tunnel 1 | 3,982 | 246,291,863 | 97.9559% | 255 | 254 | 3,982/3,982 | 95.08–95.14 MiB | `4e0a4eb3...` |

All `680,944,230` values are finite, and each complete intensity payload and
decision payload is byte-identical across repetitions. Oxford required 115
raw scans for its 15 geometry-qualified candidates; all 115 matched the v17
state clock, with maximum absolute error 0.307016 ms. The selected-cloud
payload SHA-256 is `f6f18abe...`.

Oxford's 75 pair scores explain why raw intensity does not safely rescue the
global route. Selected-alignments have high Pearson correlation
(`0.8687` minimum, `0.9071` median, `0.9483` maximum) and strong reciprocal
support (`2,619` minimum), but the same intensity texture remains locally
ambiguous. Only 39/75 pairs meet the frozen 0.08 spatial-peak margin; the
margin range is `0.0240–0.1257`. Every one of the 15 five-pair candidates
therefore fails at least one independent identity pair. This rejects all
three old ambiguity clusters before an edge exists: `6 -> 303`, the
`68--75 -> 363--370` cluster, and the `158--164 -> 419--425` cluster.
There are zero legacy survivors and zero new verified constraints.

The 18 v43b synthetic contracts and the complete v40–v43b regression set
pass (`70 passed`). The aggregate evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v43b_raw_intensity_identity_audit_20260810/`.
Its file SHA-256 is
`f6bd29ec58b78079751bcd19fd676ddafc28a1fc04eb80aef5ef939654900c65`,
and its deterministic aggregate payload SHA-256 is
`9cb9e874f879b337439a9b761235898ace128751cb6f86401a63a62e05f9496b`.

The corrected sealed decision is
**`CLOSE_V43B_GLOBAL_CORRECTION_ROUTE_NO_NEW_UNAMBIGUOUS_CONSTRAINT_SET`**.
Independent raw intensity is available, so v43's availability conclusion is
retired. The global-correction conclusion remains closed for the stronger
reason that the preregistered identity challenge removes every ambiguous
candidate and leaves no replacement constraint. External pose-graph and map
refinement implementation remain unauthorized; v17 remains primary and no
accuracy/reference-map input was opened.

## Next workstream: v44 local-estimator architecture boundary

The next candidate must be local and causal rather than another global
correction. The exact-hash raw-input prerequisite is now satisfied by the
v43b source manifest. v44 proceeds in this fixed order:

1. Perform a report-only LiDAR/IMU clock, rate, gap, covariance, unit, frame,
   and startup inventory on all three exact bags. Freeze one common source,
   timestamp, and memory contract without accuracy or map references.
2. Define one fixed-lag shadow estimator with explicit IMU preintegration,
   gyro/accelerometer bias state, gravity handling, per-factor observability,
   bounded marginalization, and no loop closure or global map correction.
3. Add synthetic contracts for preintegration direction, bias Jacobians,
   timestamp gaps, marginalization consistency, weak-axis observability,
   deterministic ordering, and fail-closed resource exhaustion.
4. Replay in shadow mode first: consume the exact raw stream and export only
   diagnostics while v17 remains the published trajectory. Require bounded
   RSS, completion, deterministic state payloads, and no protected-output
   writeback on all consumed sequences.
5. Only after that runtime gate passes may one frozen v44 candidate open the
   three-sequence accuracy and map screens. One scored failure ends the
   candidate without threshold tuning; fresh holdout remains last.

The model must address Oxford vertical-bias accumulation and UrbanNav
weak-axis speed in one global architecture rather than reusing the retired
direct bias/velocity threshold variants from v22–v38. The immediate next
implementation is therefore the stage-1 raw LiDAR/IMU contract and inventory,
not another loop matcher.

## v44a stage-1 raw LiDAR/IMU readiness result (2026-08-10)

The report-only stage-1 inventory is complete. The active global contract is
`configs/sota_v6/development/v44_raw_lidar_imu_readiness_audit.json`
(contract ID `v44a-raw-lidar-imu-readiness-20260810`, SHA-256
`a71d9271d200e86537cb6f8b25e21662ffe120258e749e6dff17a77e6d1a162b`),
the exact bag/topic/frame/provenance binding is
`configs/sota_v6/development/v44_raw_lidar_imu_sources_20260810.json`
(SHA-256
`002cc82fcd86a70fb774e1e3f576b41cbe86b849eabcfc6406e98cb26232ff1d`),
and the bounded one-pass implementation is
`scripts/audit_v44_raw_lidar_imu_readiness.py` (SHA-256
`db32ce9b65c4ab54825585e801ebbb26984b56fad86a430d7b27c84dacae7500`).
The contract has no dataset branch, consumes no accuracy trajectory or
reference map, writes no trajectory or map, and enforces a 128 MiB RSS ceiling.

One preregistration correction is retained explicitly. The first NavINST
preflight used contract SHA-256
`2336c29dce06e38f4be3dd4f6d9495ca42e2bd4bef68015f05540dbfae8771ea`
and rejected readiness solely because only 94.898252% of scans contained a
physical return at exactly `t=0`. All its other LiDAR, IMU, provenance,
normalization-digest, and synchronization checks passed. A point timestamp
defined as unsigned nanoseconds from scan start need only be a non-negative
offset; it does not imply that the sensor observed a return at the exact scan
origin. The rejected contract is preserved byte-for-byte as
`configs/sota_v6/development/v44_raw_lidar_imu_readiness_rejected_preflight.json`,
and the original report remains under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44_raw_lidar_imu_readiness_20260810/`.
Before reading any Oxford or UrbanNav score, v44a replaced that invalid
invariant with the exact `uint32` non-negative-offset schema plus mandatory
equality to each sealed normalization timing digest. No observed performance
threshold was relaxed.

Each exact bag was then scanned twice under v44a:

| Sequence | LiDAR rate / max gap | IMU rate / max gap | Fully bracketed | Max boundary distance | Orientation / covariance | Startup | Peak RSS range |
| --- | ---: | ---: | ---: | ---: | --- | --- | ---: |
| NavINST indoor02 | 9.9147 Hz / 101.121 ms | 125.0091 Hz / 7.999 ms | 99.9713% | 7.999 ms | absent / absent | stationary | 67.66–67.76 MiB |
| Oxford spires_keble_05 | 10.0403 Hz / 101.636 ms | 399.5339 Hz / 2.534 ms | 99.9829% | 2.520 ms | absent / absent | dynamic | 118.08–118.14 MiB |
| UrbanNav HK tunnel 1 | 9.9914 Hz / 105.746 ms | 400.0048 Hz / 9.227 ms | 99.9749% | 4.332 ms | provided / provided | stationary | 97.44–97.55 MiB |

All six runs pass. Every LiDAR payload has the exact required 48-byte schema,
all point-time normalization digests match their sealed source reports, all
IMU measurements are finite and within the frozen norm bounds, and each
sequence has only one unbracketed prefix scan with zero interior or suffix
coverage holes. The complete deterministic payload SHA-256 is identical
between repetitions for each sequence: `6c611320...`, `90b1cba5...`, and
`6604d0c5...`. The 18 v44a synthetic contracts and the complete v40–v44
regression set pass (`88 passed`).

Aggregate evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44a_raw_lidar_imu_readiness_20260810/`.
The aggregate file SHA-256 is
`532b62ad534cb533c1ecfaa735a844f68dfb38a0a4c5184ef598ab98f3b959fb`,
and its deterministic aggregate payload SHA-256 is
`57c14d126beeacc741471864bc6828869e2f7f3980edfba2b3e62f93c5b437c0`.

The sealed decision is
**`AUTHORIZE_V44_FIXED_LAG_ARCHITECTURE_DEFINITION`**. This authorizes only
the stage-2 architecture contract. Shadow-estimator implementation remains
unauthorized. Because orientation and covariance are not universal and the
Oxford startup window is dynamic, that architecture must use one explicit
noise model, orientation-independent initialization, and dynamic-startup
initialization across all datasets. Dataset-specific algorithm thresholds,
loop closure, and global map correction remain forbidden. Accuracy and map
screens remain closed until the architecture, synthetic contracts, and
report-only shadow runtime gates have passed in order.

## v44b stage-2 fixed-lag architecture result (2026-08-10)

The architecture-definition stage is complete. The machine-readable global
contract is
`configs/sota_v6/development/v44b_fixed_lag_shadow_architecture.json`
(contract ID `v44b-fixed-lag-shadow-architecture-20260810`, SHA-256
`74a80cca0e3dc7de9a10950d50c4fea7518dfe3d4fed8c5e1aeec77c3efee8e7`).
Its readable design and failure analysis are in
`docs/research/v44-fixed-lag-shadow-architecture-2026-08.md` (SHA-256
`3a2267b5ae0a1d3115b4900e55b593de28278e904762ff39d6e712ff64811edc`).
The static validator is
`scripts/validate_v44b_fixed_lag_architecture.py` (SHA-256
`43576fb83f6011f0724b672f6671a849302b1862ff9c74f1ad9a6d1e1b37a232`).
It verifies the exact v44a contract/source/aggregate hashes before accepting
the architecture and has no bag, estimator, accuracy, trajectory, or map
execution interface.

The single architecture uses one 15-DoF state at every accepted scan end:
`R_WB`, `p_WB`, `v_WB`, gyroscope bias, and accelerometer bias. Dynamic
initialization jointly estimates all bootstrap states, both biases, and a
fixed-magnitude S2 gravity direction over a 2–5 second window. Message
orientation, stationary assumptions, zero-velocity factors, and message
covariance are not used. After deterministic gravity/yaw rebasing, streaming
uses `g_W = [0, 0, -9.80665] m/s²` and the exact fixed sensor extrinsics.

Adjacent states use bias-linearized SO(3) midpoint IMU preintegration with all
five required bias Jacobians and an explicit common continuous-time noise
model. LiDAR contributes only causal binary point-to-plane factors between
active knots. Each whitened factor is projected by deterministic SVD onto its
six-DoF relative-pose observable subspace. Rejected modes preserve process
uncertainty; they do not freeze an axis or directly overwrite velocity or
bias. Thus the architecture does not claim to invent UrbanNav tunnel-axis
speed, but it prevents weak geometry from becoming false state information.

The smoother has a three-second, 64-knot fixed lag. This covers the v44a
maximum permitted 20 Hz LiDAR rate (`62` required slots), bounds the active
state dimension at `960`, and bounds active LiDAR correspondences at
`768,000`. Optimization is single-threaded, fixed-order, and limited to four
Gauss–Newton iterations with streaming square-root QR. Marginalization uses a
square-root separator prior with first-estimate Jacobians; prior reset and
ad-hoc covariance inflation are forbidden. The limits remain `330 MiB` RSS
and `0.85` processing RTF.

The exact v44a startup coverage policy is inherited rather than retuned: at
most two unbracketed prefix scans may be dropped before the first state, while
any interior boundary gap after startup fails closed. Sensor time is integer
nanoseconds, every observation is consumed once, and wall clock is never an
estimator input. v17 files are hash-checked before and after; ROS publication,
primary state/map/bias writeback, loop closure, global correction, camera, and
visual-speed feedback are all prohibited.

The architecture explicitly removes the failed v22–v39 mechanisms: raw
coordinate `HTH` gates, direct velocity or accelerometer-bias corrections,
weak-axis speed/history latches, producer-frozen axes, wall-clock age,
repeated observations, and visual feedback. It also keeps the v40–v43 global
correction routes closed.

The contract pre-registers 20 stage-3 synthetic contracts covering SO(3)
direction, gravity sign, bias Jacobians, covariance propagation, timestamp
gaps, deskew and extrinsic direction, binary LiDAR Jacobians, weak-subspace
projection, dynamic initialization, square-root/FEJ marginalization,
fixed-lag eviction, deterministic payloads, resource exhaustion, and protected
outputs. The 27 architecture-boundary tests pass, and the complete v40–v44b
regression set passes (`115 passed`).

Static validation was executed twice. Both reports bind the same contract and
validator hashes and have the identical deterministic payload SHA-256
`2ba8e5d7f5191658d4ced36aecf56514f10210ec78f2b2c380eca22585d3db4f`.
Evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44b_fixed_lag_architecture_20260810/`.
The aggregate file SHA-256 is
`9bc723886d3faa067de979e203eb9e49d534b2252fb733e05f08de386fb64407`,
and its deterministic aggregate payload SHA-256 is
`7794446259eb915e967294e3c515495b23abef57a19398cc2f1beac8b8ec6d52`.

The sealed decision is
**`AUTHORIZE_V44_STAGE3_SYNTHETIC_CONTRACT_IMPLEMENTATION`**. Only the
numerical synthetic harness is now authorized. Fixed-lag estimator
implementation, raw shadow replay, accuracy/reference-map inputs, and primary
trajectory or map mutation remain unauthorized. The immediate next work is to
implement the preintegration, LiDAR Jacobian/observability, initialization,
and square-root marginalization primitives only inside synthetic tests, then
aggregate two deterministic stage-3 validation runs before considering a
shadow estimator implementation.

## v44c1 stage-3 fixed-lag synthetic-contract result (2026-08-10)

The synthetic numerical-contract stage is complete. The original 20-case
scenario contract is
`configs/sota_v6/development/v44c_fixed_lag_synthetic_contracts.json`
(contract ID `v44c-fixed-lag-synthetic-contracts-20260810`, SHA-256
`0f6c57303f9889b0fd1a21b92ef80bf7d14a632010ad68222a7b6efd816e326f`).
The active resource-scope correction is
`configs/sota_v6/development/v44c1_fixed_lag_synthetic_contracts.json`
(contract ID `v44c1-fixed-lag-synthetic-contracts-20260810`, SHA-256
`e5e8a8a3ae8d7e6b17ae8c7c6580751ac3bee8ff914ba80274222617aedb6c92`).
It binds the original scenarios, the v44b architecture, and prior aggregate
evidence by hash. The validator is
`scripts/validate_v44c_fixed_lag_synthetic_contracts.py` (SHA-256
`ce5e5c5c69849838c7c0a7b6167d3bb67738b78ff106ffa63dbfd4ec1847ad66`),
and the readable contract report is
`docs/research/v44-synthetic-contracts-2026-08.md` (SHA-256
`65f3b43bfa8bbc0fd3ed6952623acb5eaba92625c8fe788a17340bb20b2bd6da`).
The implementation has no bag, ROS, trajectory, map, accuracy, or reference
input interface and cannot mutate the primary estimator.

The first standalone v44c run passed all 20 numerical cases at 42.34 MiB RSS.
However, the full pytest host already occupied 131--143 MiB before the
synthetic harness began, so the original 128 MiB absolute ceiling rejected
embedded tests before any v44c allocation. The preliminary evidence remains
preserved under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44c_fixed_lag_synthetic_contracts_20260810/`;
its aggregate file SHA-256 is
`f3cb78a04efad5615bc37d9ee32242b64238375c71bb21b7a52f1401d6e0a55f`
and its aggregate payload SHA-256 is
`c89f1cf5fcece0799afd7d897423bf17ee850b85156706f2fc34cd68c00e4c6f`.
This was a resource-measurement modeling defect, not a numerical failure.
v44c1 therefore changes only measurement scope: a standalone process retains
the absolute 128 MiB ceiling, while an already-loaded host receives a 64 MiB
incremental ceiling. No scenario, seed, ordering rule, motion, geometry,
numerical threshold, or tolerance changed.

The reference implementation now covers SO(3) exponential/logarithm and
right perturbations, integer-nanosecond IMU bracketing, midpoint
preintegration with covariance and bias Jacobian oracles, scan-end deskew,
fixed extrinsics, analytic binary point-to-plane Jacobians, deterministic SVD
observable projection, dynamic initialization, square-root marginalization,
immutable first-estimate Jacobian priors, fixed-lag eviction, deterministic
serialization, resource rejection, and protected-output hashing. These are
bounded synthetic oracles, not a streaming estimator.

All 20 frozen challenges pass. Representative results are `9.94e-17 rad`
SO(3) direction error, `7.11e-15` maximum constant-motion residual,
`1.11e-10 rad` gyro-bias reintegration error, `3.34e-12 m/s` accelerometer
bias velocity error, positive-semidefinite covariance with a `2.46475` trace
growth ratio, `4.69e-11` maximum binary LiDAR Jacobian error, and `1.55e-15`
SVD projector rotation-invariance error. The weak-axis case retains rank five
and exactly zero weak-mode update/information while preserving the `0.75`
process increment. Dynamic startup recovers nonzero initial velocity without
an orientation, stationary, or ZUPT branch. Square-root marginalization
matches the batch solution to `6.11e-16`; FEJ gauge error is zero; a four-second
20 Hz stream leaves 61 active and evicts the 20 oldest states. Timestamp-gap,
ordering, duplicate, resource, and write failures all reject with zero state
output and unchanged protected outputs.

The first numerical execution also exposed a real implementation defect: the
SO(3) logarithm returned twice the intended rotation vector because both the
skew-vector extraction and scale supplied a factor of two. The implementation
was corrected without relaxing the frozen tolerance. The 46 focused tests
pass, including finite-difference, malformed-input, prerequisite-hash,
authority, allocation, repeatability, and incomplete-report negatives. The
complete v40--v44c1 regression passes (`161 passed in 2.60s`).

Two final standalone validations both produced report payload SHA-256
`710564cc07242d98e9bddd0170362a20c2741e4318ddb5602f8964699800b86c`
and combined case payload SHA-256
`7c6fd8e106885f90c1fc1627939212bfaf454c6692ae9be15aa1abb06ea593d2`.
They began at 37.10--37.14 MiB RSS, peaked at 42.47--42.55 MiB, and added
5.34--5.45 MiB. Evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44c1_fixed_lag_synthetic_contracts_20260810/`.
The run file SHA-256 values are
`33825a159c396bf8a62d6001e1afdd07044add65b2f51204d7da5c96f1a4c6db`
and
`036ad5d4dc636681b94eb6155bee76a2fff0a12407dbb7f1721faec37be555d9`.
The aggregate JSON SHA-256 is
`18dc483ae4a924608d16ed3f47b7dd5bb812f6baea027d931d7f2010287bf663`,
and its deterministic aggregate payload SHA-256 is
`c89f1cf5fcece0799afd7d897423bf17ee850b85156706f2fc34cd68c00e4c6f`.

The sealed decision is
**`AUTHORIZE_V44_STAGE4_REPORT_ONLY_SHADOW_IMPLEMENTATION`**. This authorizes
only implementation of an isolated, diagnostic-only fixed-lag shadow
estimator. Raw shadow replay is still unauthorized. Before replay can be
considered, the source must be complete, pass a static audit against every
v44b boundary, and receive a separate execution contract. Accuracy or
reference-map inputs, primary trajectory/map/bias writeback, loop closure,
global correction, and any production-path mutation remain forbidden. The
immediate next work is stage 4 source implementation plus static source audit;
it is not raw-data execution.

## v44d stage-4 fixed-lag shadow-source gate (2026-08-10)

The isolated estimator-core implementation and pre-replay source audit are
complete. The machine-readable gate is
`configs/sota_v6/development/v44d_fixed_lag_shadow_source_audit.json`
(contract ID `v44d-fixed-lag-shadow-source-audit-20260810`, SHA-256
`462d34fb984a45f4a15a5482d2d0e9f61e45c68491e2ddefa9a7fdaa3348df5e`).
The report-only estimator core is
`scripts/v44_fixed_lag_shadow_estimator.py` (SHA-256
`c353cce1cde17e45ea1244871da8a7077f10b28055f8c8cdfae49a5bddaf8ee9`),
the AST/static validator is
`scripts/audit_v44d_fixed_lag_shadow_source.py` (SHA-256
`8d2644e9b05102029743f6cff7701913c88bacaa61fb4cf231b9da026e14bd06`),
and the readable source-gate report is
`docs/research/v44-shadow-source-audit-2026-08.md` (SHA-256
`7bf2bae05d9f96aa152826525bd77f5f76098fd3eabb4435e53bfb10336c5eb4`).

The core has no CLI, raw-bag decoder, ROS subscriber/publisher, filesystem,
network, subprocess, wall-clock estimator input, dataset identity, accuracy
input, reference map, or primary-state callback. Its public sensor boundary is
limited to immutable body-frame IMU records, LiDAR points/scans with integer
nanosecond time, one fixed `T_BL`, and externally supplied resource/protected-
output observations. It returns in-memory diagnostic records only. No raw bag
was opened and no runtime adapter exists in this stage.

The implementation follows the single v44b path: exact point-time IMU
integration and scan-end deskew; deterministic range/voxel/surfel handling;
one 15-DoF state per accepted scan; midpoint SO(3) preintegration with 9x9
covariance and all five bias Jacobians; binary point-to-plane factors; Huber
whitening; per-factor relative-pose SVD projection; chronological/type factor
ordering; bounded block Householder QR and rank solve; fixed line-search work;
three-second/64-knot eviction; square-root FEJ separator priors; source-surfel
removal; and diagnostic-only full-lag bias Schur information. Every capacity is
checked before the relevant allocation or output, and a terminal failure has
zero valid state count and no state payload hash.

Bootstrap uses the same dynamic path for moving and stationary inputs. The
first LiDAR translation divided by sensor time is only the velocity seed. All
bootstrap knots, both biases, and a fixed-magnitude two-DoF S2 gravity
direction enter one joint solve under the single broad prior set. The result is
rebased deterministically to negative-z gravity, first position at the origin,
and first yaw zero. Message orientation, stationary detection, ZUPT, and direct
state-component correction are absent.

Semantic review before sealing found that the first draft had kept gravity
fixed during bootstrap even though knot and bias states were jointly solved.
That incomplete implementation was corrected by adding the S2 gravity variable
and broad priors. The same review replaced a linear within-scan pose proxy with
integration at every exact point timestamp, changed the velocity seed to the
specified first translation/time pair, and added the full-lag bias-information
diagnostic. No threshold, input authority, or replay authority was relaxed;
all final evidence was regenerated after the corrections.

The static validator parses 21,393 AST nodes and checks 21 classes, 29 module
functions, 33 estimator methods, and the exact eight-root import allowlist. All
36 source-boundary checks pass. Ten independent in-memory smoke probes also
pass: zero-error SO(3) round trip; all five finite bias Jacobians; two-DoF S2
gravity with zero magnitude error; PSD covariance with minimum eigenvalue
`9.99972e-6`; binary LiDAR Jacobian error `4.68738e-11`; Householder dense-
solution error `9.45728e-16`; separator-solution error `8.94472e-16`;
immutable FEJ payload; protected-output mutation rejection; and pre-allocation
resource rejection.

The full frozen-configuration synthetic stream runs 11 planar scans with
100 Hz IMU twice. Both runs produce state payload SHA-256
`928df5580492ecd6471427bcdc2e6e474ce5921753c2f782826b29d46b6353de`
and diagnostic payload SHA-256
`1512d9971a1853ed3dac6acc53003a2d19abe379d48a0e9f7d2abf1723fd5257`.
It reaches LiDAR observable rank three, reports 62 retained modes in the 66-
dimensional full-lag bias-information system, and finishes with exact gravity,
origin, and yaw gauges. A shortened-lag integration test evicts the three
oldest of seven states, leaves knot IDs `3,4,5,6`, removes old surfels, and
retains an immutable separator prior. The 28 core tests and 23 audit tests
pass, and the complete v40--v44d regression passes
(`212 passed in 18.39s`).

Two final static validations have identical deterministic report payload
SHA-256
`9a7bf365e5a90e0f0a1b1296b358305277091059d0ea8bb3beece45fdea2b7d2`.
They began at 34.38--34.40 MiB RSS, peaked at 50.45--50.46 MiB, and added
16.06--16.07 MiB. Evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44d_fixed_lag_shadow_source_audit_20260810/`.
The two report file SHA-256 values are
`1b6aeba274d75329ac69d58bba0f1991309aee93966d10dcc7cb089718be333d`
and
`8f6ee2eb85209f27c3155778478de7a11bdd4139f09a04f5510f63b28b4aa06e`.
The aggregate JSON SHA-256 is
`b2494f42b07d9fd844f14112da5b6df6fd050007326018967f9f794b0cb5e997`,
and its deterministic aggregate payload SHA-256 is
`2344a9e9b31861fff44c1b457199a2863615a2c06834a6a9c39a710fc53b97b2`.

The sealed decision is
**`AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_CONTRACT_DEFINITION`**. This does not
authorize raw replay. It authorizes only definition of a separately hash-bound
execution contract and a read-only decoder adapter outside the estimator
core. That next contract must bind the exact v44a source manifest and all three
bags, write only into a new bounded evidence directory, enforce 330 MiB RSS
and 0.85 RTF after every scan, compare protected v17 hashes before and after,
and require completion plus identical state payloads across two repetitions.
Accuracy, ground truth, reference maps, ROS publication, primary trajectory/
map/bias writeback, loop closure, and global correction remain forbidden.

## v44e raw-shadow replay execution-contract gate (2026-08-10)

The separately hash-bound raw replay contract and read-only decoder adapter are
complete. The machine-readable contract is
`configs/sota_v6/development/v44e_raw_shadow_replay_execution_contract.json`
(contract ID `v44e-raw-shadow-replay-execution-20260810`, SHA-256
`08d7f8850f5081fd18d58a860cae3d9566846605460dadb37660b3c3111d1692`).
The adapter is `scripts/v44_raw_shadow_replay_adapter.py` (SHA-256
`6fbf16d811a5bcf26c2cac9190f0557844cd3596cf7704cfdc016c7080ddf0ee`),
the static auditor is
`scripts/audit_v44e_raw_shadow_replay_contract.py` (SHA-256
`8fa736d6c59b17eb3364c50cad0a9ec88cf01ffe867864e9558387db37909389`),
and the readable contract report is
`docs/research/v44-raw-shadow-replay-contract-2026-08.md` (SHA-256
`990a80dff5adafd086ae132050772696c25823c46132efc4093e57d7b8f0db68`).

The contract explicitly repeats and verifies all three v44a bag SHA-256 values,
sizes, topics, frames, message/point counts, and serialized LiDAR/IMU stream
digests. The three sensor-adapter hashes and exact `T_BL` matrices are sealed as
source calibration rather than algorithm branches. The estimator still sees
one global v44b configuration and receives no sequence ID, dataset name, bag
path, orientation, covariance, protected path, accuracy input, or output
callback.

Bag receive order is converted into the architecture's sensor-time order by a
bounded watermark heap. The exact observed maximum receive-minus-header delays
are 3,573,549 ns for NavINST, 135,957,579 ns for Oxford, and 136,013,359 ns for
UrbanNav. The heap is limited to 128 messages and 32 MiB, emits by header time
with IMU before LiDAR and stable source index, and terminates on a larger delay,
receive-time reversal, duplicate/out-of-order output, or capacity overflow.
PointCloud2 remains the exact 48-byte little-endian v44a schema; IMU decoding
uses only header time, angular velocity, and linear acceleration.

The retained v17 `run_01/trajectory.tum` and
`run_01/pcd/voxel_global.pcd` files are individually path/size/hash-bound for
each sequence. Before and after a raw shadow run, the adapter rehashes both and
passes only the digest dictionary to the core protection guard. The core cannot
open those paths. A changed v17 state or map is terminal and cannot yield a
valid shadow result.

The runtime CLI exposes only contract, sequence ID, and repetition. Bag,
output, trajectory, map, and calibration paths cannot be supplied. Output is
restricted to a new nonsymlinked
`raw_replay/<sequence>/run_NN/{diagnostics.jsonl,run.json}` directory under the
v44e evidence root, opened without overwrite and capped at 256 MiB. RSS and
cumulative processing RTF are passed to the core after every completed scan;
the frozen ceilings remain 330 MiB and 0.85. A complete run must also match the
bound message, point, and serialized-stream identities.

The activation chain is fail closed. Before importing `rosbags` or opening a
bag, the adapter verifies every prerequisite/source hash, its own hash, the
contract hash, and the v44e aggregate decision. It then reopens and rehashes
both static source reports listed by that aggregate, requiring repetitions one
and two, identical deterministic payloads, and explicit proof that neither
static pass opened or replayed raw data. Thus an aggregate filename alone is
not sufficient authorization.

All 29 static checks and 10 synthetic adapter probes pass. The focused v44e
suite passes (`49 passed in 4.89s`), and the complete v40--v44e regression
passes (`261 passed in 24.69s`). Two final standalone static validations have
identical deterministic report payload SHA-256
`b98739d779a6e79b2027f100172230f48700b4045dc4b8610b3b54e5fe503d3f`.
Their report SHA-256 values are
`27d98bb4fe182efa5c38ce97f5ee8f876e588ca353e0a393be4339d8cf26bae9`
and
`30a9ba4a02399278e2682e9acdd45daeab82f038eb8b1cebb30763d5491082a4`.
They began at 34.63--34.73 MiB RSS, peaked at 43.16--43.23 MiB, and added
8.50--8.53 MiB. Evidence is retained under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44e_raw_shadow_replay_contract_20260810/`.
The aggregate JSON SHA-256 is
`5d3516edcadea924c5481688666edddec2db7b2be61eaa5cf42f7c52cc278181`,
and its deterministic aggregate payload SHA-256 is
`7550b7303021d8440b749dc2b19279f3cdb66c8c6dfade6f3d2a27a64c91c10e`.

The sealed decision is
**`AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_EXECUTION`**. No raw bag was opened in
this gate. The next action is exactly six report-only runs: two repetitions of
each bound sequence. Every run must complete with a valid non-null state
payload, preserve both v17 hashes, remain under RSS/RTF after every scan, and
match its exact stream identities; each sequence's two state payload SHA-256
values must be identical. Any one failure rejects the route. Accuracy, ground
truth, reference maps, fresh holdout, ROS publication, primary writeback, and
promotion remain closed even if all six raw runs pass; opening an accuracy
screen requires another separately sealed contract.

## v44f fail-fast raw-shadow execution result (2026-08-10)

The first authorized v44e raw-shadow attempt was executed and failed the frozen
resource gate. The ordered first run was `navinst_indoor02`, repetition 1. The
exact v44e authorization chain passed, the NavINST bag SHA-256 was verified,
and scan index 0 was recorded as the permitted unbracketed prefix drop. The
mandatory post-scan runtime observation then caused the estimator to terminate
with `processing RTF capacity exceeded after scan`, meaning the attempted
cumulative RTF was strictly above the unchanged 0.85 ceiling.

The failed adapter report is retained at
`sota_v6_dev_v44e_raw_shadow_replay_contract_20260810/raw_replay/navinst_indoor02/run_01/run.json`
(SHA-256
`1913305fe4b3a8ded99c62aef3026428443643ecaa281a41c7fe0de6a1bd0b4a`),
and its two-record canonical diagnostic is beside it (SHA-256
`32a5764f1db2abdd4b46d2eefb48ce4e69a6481ec93433b035f7cf6ced247639`).
The adapter report payload SHA-256 is
`1194b8b69b1a35183ba8f33046ec494df9660f6dacc3b76823040b4680a8396a`.
The core returned FAIL, zero active states, `valid_shadow_result=false`, and a
null state payload hash. NavINST's protected v17 state and map hashes are
identical before and after. Accuracy/reference-map inputs, ROS publication,
primary writeback, and every valid-state output remained closed.

The v44e execution contract defines any single run failure as rejection. The
remaining NavINST repetition and both Oxford/UrbanNav repetitions were
therefore not started. The failed directory was retained without overwrite,
and no threshold was relaxed. Since execution stopped before stream completion,
the report has no completed full-stream inventory and makes no claim that the
whole bag was consumed.

The exact offending RTF number was not persisted. The core checks RTF before
replacing the scan diagnostic's placeholder runtime fields, so the dropped
scan retains `processing_rtf=0.0` even though the terminal reason proves an
over-limit attempted value. This is an observability limitation, not a pass or
an unknown gate result. It prevents phase attribution from this run and must
not be interpreted as measured zero RTF.

The post-execution audit contract is
`configs/sota_v6/development/v44f_raw_shadow_replay_execution_audit.json`
(contract ID `v44f-raw-shadow-replay-execution-audit-20260810`, SHA-256
`3da6877b93a386fe875bfefc3844d9105e1cb90ec5820ac86ae7364d97f330b7`).
The auditor is `scripts/audit_v44f_raw_shadow_replay_execution.py` (SHA-256
`9e0292e0055b06663314c5f7fe04cd82af17e2ca203e56265fda913c21882675`),
and the readable result is
`docs/research/v44-raw-shadow-replay-execution-2026-08.md` (SHA-256
`9b2586b1bdcde9bf3bcb0ea52f99be8f89bdbff5e0882003592a81a50af14567`).

All 18 evidence checks pass, the focused v44f suite passes (`16 passed in
0.88s`), and the complete v40--v44f regression passes (`277 passed in 25.38s`).
Two standalone audits have identical deterministic report payload SHA-256
`29ef8cef8237544110e776662fd3e73ddc07f7aedd751c7a581a9c35149711bd`.
Their report SHA-256 values are
`b0b96e016143d5f42d2661deb21994dea14889be21f489b87d643b20842dd323`
and
`3a5fe3b2f1bda2c45d9973782c7aa8d301cf2b2b95ddc5efea657710e040861f`.
The aggregate JSON SHA-256 is
`9b4dbbaede2239ae619594b6c84ee0f33b7fc89e360bfac777a5c37c37a2e2eb`,
and its deterministic aggregate payload SHA-256 is
`f9a9b25e746f49f5b2dc9665dba1a8019c59bfb2b044897f0561750a1da31073`.
Evidence is under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44f_raw_shadow_replay_execution_audit_20260810/`.

The sealed route decision is
**`REJECT_V44_STAGE4_RAW_SHADOW_REPLAY_RESOURCE_GATE`**. Raw replay
continuation, accuracy, ground truth, reference maps, fresh holdout, primary
mutation, and promotion are unauthorized. The only next authorized work is to
define a bounded failure-profile contract that records the attempted RSS/RTF
and fixed decoder/reorder/core phase timings before invoking the same unchanged
0.85 gate. That diagnostic contract cannot run another full replay, tune from
accuracy, or relax the resource threshold.

## v44g definition-only failure-profile contract (2026-08-13)

The bounded failure-profile boundary is now defined and statically audited.
The machine-readable contract is
`configs/sota_v6/development/v44g_raw_shadow_failure_profile_contract.json`
(SHA-256
`b985b4454908619215d0f41c1c26845d244790f23b6a40d2dda9677e1935c281`), the
auditor is
`scripts/audit_v44g_raw_shadow_failure_profile.py` (SHA-256
`d03f1ee399815f04401ab7cadaf5ca7234b662dc133357ae3ec9d1a7901a0839`), and
the readable contract report is
`docs/research/v44-raw-shadow-failure-profile-contract-2026-08.md` (SHA-256
`ed7f82fe4444b807977cc30f204117a346624b42cdfa1f14008b524631d203ab`).

The definition binds the completed v44f reject audit, the v44e execution
contract, and the exact adapter/core hashes. Its target is only the already
attempted `navinst_indoor02`, repetition 1, scan 0 failure. The required fixed
diagnostic phases are `decoder -> reorder -> core`; all count toward the same
v44e cumulative processing RTF. The profile must capture attempted RSS and
processing RTF, sensor interval, phase timings, gate order, and terminal reason
before comparing against the unchanged `330 MiB` / `0.85` limits. A placeholder
`0.0` RTF is explicitly invalid.

Two definition audits passed all 15 checks with identical deterministic report
payload SHA-256
`3a057ba8ad418a64a0639af04f046b98ec51fb8e12821cbcf28668113169d8a7`.
The retained aggregate is under
`/home/sasaki/workspace/old_~2026/lidarslam_ws/sota_v6_dev_v44g_failure_profile_contract_20260813/`,
has file SHA-256
`6964094a85c746bee7f4081ee3173d6db21d604fe6ea9ed518d6955f01405610`, and
deterministic aggregate payload SHA-256
`83d4d0b742ce44c7149c9e703389b6b206a0affa075ac31b9b60b121372c92ac`.
The focused v44g suite passes (`12 passed`). The sealed decision is
**`AUTHORIZE_V44G_FAILURE_PROFILE_CONTRACT_DEFINITION_ONLY`**: profile
execution, raw replay continuation, accuracy, primary mutation, and threshold
relaxation remain unauthorized.
