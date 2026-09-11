# Sliding-window gravity alignment study (2026-07-26)

## Decision

Adopt `gravity_window_alignment` (rko_lio fork PR #5) as a **preset-scoped
opt-in**: enabled with `gravity_alignment_gain: 0.2` in `tunnel_radar` and
`corridor_fog_radar` presets and the two `radar_continuous` NTNU configs.
Default remains off, and the exp07/MID-360 holdouts below rule out any
default-on promotion.

First degeneracy intervention in this track that improves both NTNU
sequences simultaneously.

## Root cause: pitch drift the existing regularization cannot see

The NTNU tunnel map (`runs/tunnel_full_slam_map_v1`) sagged parabolically to
z = -33.45 m over the ~500 m level traverse. Projecting raw IMU accelerometer
interval means through the estimated orientation shows the world-frame
gravity residual growing monotonically 0.5 -> 6.3 deg; integrating that tilt
reproduces the sag almost exactly, so the failure is attitude drift, not
translation error.

The existing `min_beta` orientation regularization cannot correct this by
construction. Its reference comes from the body-acceleration Kalman filter,
whose measurement is `avg_imu_accel + R_est^-1 * gravity()` -- built with the
*current, drifting* rotation estimate -- so the reference tracks the drift
instead of observing it. Measured (min_beta sweep, tunnel, radar continuous
config): 200 -> 50 -> 20 -> 5 leaves the final-60s tilt at 6.28 -> 6.36 ->
6.39 -> 6.75 deg and end-z at -33.45 -> -33.89 -> -34.21 -> -35.90 m. A 40x
stronger weight makes the sag slightly worse, because it pulls harder toward
the drifted reference. Sweep artifacts: `runs/tunnel_minbeta_sweep_v1`.

## Mechanism

`gravity_alignment.hpp` + the accumulation block in `LIO::register_scan`:
each scan interval's raw (gravity-inclusive) accelerometer mean, rotated by
that scan's optimized orientation, enters a ~20 s sliding window. Over a
quasi-steady traversal the body-acceleration content averages out (measured
window |mean| ~9.93 m/s^2 throughout the tunnel), so the window mean is an
absolute up reference. The residual tilt toward +z is fed back as a
left-multiplicative, roll/pitch-only correction of at most
`gain * tilt`, hard-capped per scan (0.002 rad). Yaw is untouched.

Guards, each locked in by a measured failure:

1. **Yaw-rate sample gate** (0.05 rad/s, world-frame z): sustained turning
   adds centripetal acceleration that does not average out. Without it fog
   regressed 11.21 -> 16.57 m; with it fog *improves* to 9.63 m. Threshold
   chosen from measured separation: fog loop walk p50 yaw rate 0.10 rad/s
   vs tunnel straight traversal p90 0.04 rad/s.
2. **Magnitude gate** (|mean| within 5% of g): rejects sustained
   acceleration. This is also what kept the corrector fully dormant on the
   MID-360 driving holdout.
3. **Plausibility bound** (45 deg, permissive): a 10 deg bound was tried and
   rejected -- it blocked fog's clutter-lock segments, where large measured
   tilts carry genuine attitude error, and fog degraded to 13.79 m. The
   per-scan cap is the primary safety mechanism.

## A/B results (gain 0.2, on top of the radar continuous-fusion configs)

| sequence | metric | OFF | ON |
|---|---|---:|---:|
| tunnel | end-z sag | -33.45 m | **-5.22 m** |
| tunnel | transverse RMS | 3.50 m | **1.79 m** |
| tunnel | final-60s gravity tilt | 6.28 deg | **0.19 deg** |
| tunnel | reach (true ~500 m) | 505.03 m | 504.59 m |
| fog | start-end drift (loop GT 0) | 11.21 m | **9.63 m** |

Full-pipeline map with the corrector
(`runs/tunnel_full_slam_map_v2_gravity`): backend reach 504.53 m (+0.91%),
end-z -4.73 m, transverse RMS 1.34 m; map quality 713k points, MME
-1.367, plane thickness RMS mean 6.5 cm / p95 11.3 cm, planar coverage
0.819 (report-only, no tunnel profile exists).

Artifacts: `runs/tunnel_gravity_alignment_v1`, `runs/fog_gravity_alignment_v1`
(v1 = no yaw gate, v3 = over-tight 10 deg plausibility bound, v2/v4 = adopted
shape; `fog_gravity_v4_gain02_1` is the adopted-binary rerun after a stray
`colcon build` inside `Thirdparty/rko_lio` silently left the workspace binary
stale -- verify the build tree when a rerun is bit-identical to the previous
candidate).

## Holdouts (2026-07-27)

Frontend-only, `configs/hilti2022/rko_lio_hilti2022_pandar.yaml` /
`lidarslam/param/rko_lio_mid360.yaml` wrapped as ROS params, OFF vs ON with
gain 0.2. Artifacts: `/media/sasaki/aiueo/benchmarks/gravity_alignment_holdout_20260726`.

| holdout | result |
|---|---|
| MID-360 driving (2,772 timestamp-matched poses) | delta RMSE **0.000000 m** (byte-identical; 2,670 windows attempted, 0 applied -- magnitude gate) |
| HILTI exp07 (6 mm control points, SE3 Umeyama) | 0.208 -> 0.264 m APE RMSE (corrector applied on 1,098/1,219 scans) |

Interpretation: on platforms/motions outside the target envelope the
corrector either goes fully dormant (MID-360: the strongest possible safety
outcome, unmatched by any previous candidate in this track) or mildly hurts
(exp07 handheld walk with frequent turns). This fixes the adoption scope:
straight-corridor/tunnel and fog presets only, never default-on, and not in
general handheld configs.

## Relation to prior studies

- `lidar-degeneracy-radar-intensity-ab-2026-07.md`: radar continuous fusion
  fixed along-axis reach (98.7 -> 505 m); this study removes the remaining
  height-sag term. The two compose (all tunnel numbers above are on top of
  the radar config).
- `degeneracy-solve-cross-sensor-gates-2026-07.md` asked for "an
  independently validated uncertainty-aware prior" richer than Hessian-axis
  persistence; the long-window accelerometer mean is exactly such an
  independent reference, and the same cross-sensor discipline (dev on NTNU,
  MID-360 hard safety holdout, exp07 negative check) was applied.
