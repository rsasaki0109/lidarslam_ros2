# Radar-less tunnel adaptive applicability gate (2026-07-27)

> Update: retaining and rotating the trusted velocity prior through short yaw
> intervals raises time-consistent tunnel reach to 499.93 m while preserving
> all negative checks and limiting post-stop motion to 0.011 m. See
> [yaw-rotated velocity prior](radarless-tunnel-yaw-rotated-prior-2026-07.md).

## Decision

Adopt the scene- and speed-gated candidate in the default-off
`lidarslam/param/presets/tunnel_imu_no_radar.ros.yaml`.

The candidate reaches 415.59 m in the ~500 m NTNU Fyllingsdalen tunnel,
versus 102.26 m for default-off RKO-LIO. Its final time-aligned reach ratio
against the radar-derived pseudo-GT is 0.824, with a maximum of 0.983; it does
not overshoot the reference scale.

Paired candidate/control trajectories are byte-identical on NTNU fog, HILTI
exp07, and MID-360 driving. The gates therefore retain the tunnel improvement
without changing any tested negative-check trajectory.

## Applicability signal

The original velocity blend had enough information to detect yaw but not to
distinguish a long open tunnel from near-field clutter or driving. The adopted
gate measures two inexpensive fractions from each filtered scan:

- points nearer than 3 m must be no more than 50%;
- points farther than 10 m must be at least 5%.

Sampled scan distributions motivated those limits:

| sequence | near-fraction p50 | far-fraction p50 | scene gate |
| --- | ---: | ---: | --- |
| NTNU tunnel | 0.108 | 0.079 | accepts |
| NTNU fog | 0.926 | 0.000 | rejects while fog is present |
| HILTI exp07 | 0.869 | 0.025 | rejects |
| MID-360 driving | 0.000 | 0.673 | accepts; speed gate rejects |

At least 100 valid points are required. A raw scene rejection starts a 60 s
cooldown so an isolated clear scan in fog or clutter cannot immediately
re-arm the anchor.

The independent speed gate rejects activation above 2.5 m/s after three
consecutive scans, then enforces a 60 s cooldown. The existing 2.0 m/s limit
still caps the propagated pseudo-sensor itself. The higher activation
threshold accommodates noisy per-scan ICP velocity around the validated
~1.7 m/s walking pace; it is not permission to use the prior for driving.

## Adopted parameters

- ICP information 1, propagation information 9, decay time 100 s
- anchor agreement 0.05 m/s, ICP innovation cap 1.0 m/s
- propagated speed cap 2.0 m/s, minimum speed 0.3 m/s
- yaw gate 0.05 rad/s for 10 consecutive scans
- activation speed limit 2.5 m/s for 3 consecutive scans
- scene gate: near 3 m / maximum 0.50, far 10 m / minimum 0.05
- scene and speed re-enable cooldowns 60 s
- full map insertion

All new library defaults are disabled or behavior-preserving. Only the tunnel
preset opts into the new gates.

## A/B and holdouts

| sequence | paired control | adopted candidate | decision evidence |
| --- | ---: | ---: | --- |
| NTNU tunnel endpoint | 102.26 m | 415.59 m | 4.06x control; max pseudo-GT ratio 0.983 |
| NTNU fog endpoint | 35.61 m | 35.61 m | byte-identical TUM |
| HILTI exp07 SE(3) APE | 0.20823 m | 0.20823 m | byte-identical TUM |
| MID-360 driving | reference run | zero trajectory delta | byte-identical TUM |

The fresh paired fog control reaches 35.61 m. The older 32.80 m result in the
initial velocity-blend note came from an earlier run configuration and is not
used as the control for this decision.

The candidate diagnostics explain why each negative check is neutral:

- fog: no blend attempts; 1,364 raw scene rejections plus 359 cooldown
  rejections;
- HILTI exp07: no attempts; 1,262 raw scene rejections plus 58 cooldown
  rejections;
- MID-360: no corrections; 2,117 raw speed rejections plus 597 cooldown
  rejections.

Artifacts:

- `/media/sasaki/aiueo/benchmarks/lidar_degeneracy_datasets_v1/runs/radarless_tunnel_scene_gate_v1`

## Rejected variants

- A 2.0 m/s activation limit without cooldown rejected 100 tunnel scans and
  reduced endpoint reach to 248.29 m.
- The scene gate without re-enable cooldown admitted 119 fog corrections and
  shifted the endpoint from 35.61 m to 30.93 m.
- The speed gate without re-enable cooldown admitted six MID-360 corrections
  and changed the trajectory by 0.101 m aligned RMSE.
- Yaw persistence alone at 10 scans reached 415.59 m but previously changed
  fog and holdout trajectories, so it was not sufficient for adoption.

The preset is still deliberately narrow. Re-tune its speed envelope only
against trusted distance ground truth, then repeat fog, handheld-turning, and
driving negative checks.
