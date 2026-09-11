# Radar-less tunnel anchor-decayed velocity blend (2026-07-27)

> Update: the adopted preset now includes automatic scene and speed
> applicability gates. See
> [adaptive applicability gate](radarless-tunnel-adaptive-applicability-gate-2026-07.md)
> for the superseding validation.

## Decision

Adopt as a default-off, straight-walking-tunnel preset only:
`lidarslam/param/presets/tunnel_imu_no_radar.ros.yaml`.

The conservative adopted arm reaches 294.72 m of the ~500 m NTNU
Fyllingsdalen tunnel, versus 102.3 m plain RKO-LIO and 153.8 m for the older
intensity-disagreement fallback. It never exceeds the radar-derived pseudo-GT
scale (maximum reach ratio 0.983).

## Failure mechanism

The tunnel failure is not a gentle scale error. At about 40 m, point-to-point
ICP locks onto self-similar wall correspondences and reports zero motion for
roughly 160 s. Its translation Hessian block remains approximately `N * I`,
so Hessian-derived confidence cannot detect the lock.

The implemented prior independently integrates gravity-compensated IMU
acceleration from the last scan where 3D ICP velocity agreed with propagation.
ICP uses a fixed information scale. Propagation information decays
exponentially with anchor age. Agreement scans refresh time but do not feed ICP
velocity back into the independently propagated state.

Four guards were required:

1. 3D ICP innovation is norm-capped before fusion; otherwise an arbitrarily
   large ICP outlier leaks through any finite Gaussian weight.
2. The propagated pseudo-sensor speed is capped; otherwise accelerometer bias
   eventually makes the prior itself diverge.
3. Expected cross-axis velocity is zero relative to the previous motion axis;
   otherwise IMU lateral drift steers the trajectory.
4. Sustained yaw clears the anchor. Five consecutive intervals above
   0.05 rad/s reject fog/handheld turns without reacting to isolated tunnel
   vibration.

## Adopted parameters

- ICP information 1, propagation information 9 (fresh weight 0.9)
- decay time 100 s
- anchor agreement 0.05 m/s in 3D
- ICP innovation cap 1.0 m/s
- propagated speed cap 2.0 m/s (dataset pace ~1.7 m/s)
- minimum speed 0.3 m/s
- yaw gate 0.05 rad/s for 5 consecutive scans
- full map insertion

## A/B and holdouts

| sequence | control | adopted candidate | result |
| --- | ---: | ---: | --- |
| NTNU tunnel reach | 102.3 m | 294.72 m | 1.92x, max pseudo-GT ratio 0.983 |
| NTNU fog start/end reach | 32.80 m | 32.88 m | effectively neutral |
| HILTI exp07 SE(3) APE | 0.2082 m | 0.2264 m | +1.8 cm |
| MID-360 driving | reference run | 0.101 m aligned delta RMSE | six corrected scans; not byte-identical |

Artifacts:

- `/media/sasaki/aiueo/benchmarks/lidar_degeneracy_datasets_v1/runs/tunnel_velocity_blend_v1`
- `/media/sasaki/aiueo/benchmarks/lidar_degeneracy_datasets_v1/runs/fog_velocity_blend_v1`
- `/media/sasaki/aiueo/benchmarks/velocity_blend_holdout_20260727`

## Rejected branches

- Hard kinematic clamp with map insertion: 3.1x self-confirming runaway.
- Hard clamp with map skip: startup reversal and registration death.
- Linear soft blend fed from corrected LIO velocity: kilometre-scale positive
  feedback.
- Independent IMU state without robust ICP innovation: finite ICP weight still
  admitted huge outliers.
- Robust ICP without propagated-speed cap: accelerometer bias made the prior
  diverge.
- No yaw gate: 517.18 m tunnel reach (ratio 1.025) but fog reach 87.28 m.
- Yaw gate persistence 10 scans: 415.59 m tunnel and exp07 0.2133 m, but fog
  reach worsened to 36.17 m.
- Localizability weighting combination: tunnel endpoint improved to 499.89 m,
  but MID-360 changed by 1.424 m aligned RMSE; rejected.

The speed cap is an explicit platform-envelope calibration. Re-tune it for a
different rig only against trusted distance ground truth, and repeat fog,
turning-handheld, and driving negative checks.
