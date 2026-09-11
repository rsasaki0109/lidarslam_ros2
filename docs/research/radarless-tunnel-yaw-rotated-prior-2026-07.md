# Radar-less tunnel yaw-rotated velocity prior (2026-07-29)

## Decision

Adopt the yaw-rotated inertial velocity prior in the default-off
`lidarslam/param/presets/tunnel_imu_no_radar.ros.yaml`.

The candidate reaches 499.93 m on the approximately 500 m NTNU
Fyllingsdalen sequence, versus 415.59 m for the previous scene-gated preset
and 102.26 m for default-off RKO-LIO. Its final and maximum time-aligned reach
ratios against the radar-derived pseudo-GT are 0.991 and 0.996. Aligned
translation RMSE improves from 48.74 m to 8.32 m.

An independent persistent-inactivity gate prevents the prior from converting
a real stop into false forward motion. After the reference stops at about
310 s, the candidate moves 0.011 m over the remaining 14 s.

The trajectory evaluator also projects six representative sensor-frame points
at 5, 10, and 20 m through each candidate/reference pose after global SE(3)
alignment. This measures the world-placement error that directly blurs an
accumulated point-cloud map. At 10 m range, projection RMSE improves from
49.10 m to 10.44 m; at 20 m it improves from 50.16 m to 15.11 m.

Fog, HILTI exp07, and MID-360 driving candidate trajectories remain
byte-identical to their paired default-off controls.

## Failure mechanism

The scene-gated velocity blend still stalls between approximately 190 s and
250 s. Raw IMU inspection and the new diagnostics identify the sequence:

1. sustained yaw crosses the straight-corridor safety gate at 190.9 s;
2. the gate clears the propagation anchor;
3. ICP enters the tunnel zero-motion attractor before yaw settles;
4. `kinematic_blend_min_speed` then prevents a new anchor.

Discarding the full velocity prior on yaw loses useful speed information even
though IMU-supported orientation remains observable.

## Adopted behavior

During sustained yaw, translation correction remains suspended. Instead of
clearing the trusted velocity prior, the implementation rotates that prior by
the registered orientation delta. When yaw settles, the prior therefore has
the new world direction and retains its pre-turn speed.

A low LiDAR speed may keep using the prior only while all of these hold:

- a trusted anchor and independently propagated velocity already exist;
- within-scan raw acceleration-magnitude variance is at least 0.03
  `(m/s^2)^2`;
- scene and platform-speed gates still accept the scan.

The threshold separates the validation sequence cleanly: walking intervals,
including the 180-250 s ICP lock, remain above 0.03; the stationary interval
after 310 s remains below 0.01. Five consecutive inactive scans clear the
prior regardless of the corrected LiDAR velocity. The library default is zero,
so activity-based bridging and inactivity rejection are disabled unless a
preset opts in.

Scene rejection, excessive platform speed, low speed without inertial
activity, and anchor expiry still clear the prior.

## A/B and holdouts

| sequence | control / previous | adopted candidate | result |
| --- | ---: | ---: | --- |
| NTNU tunnel endpoint | 415.59 m previous preset | 499.93 m | +84.34 m |
| NTNU tunnel aligned RMSE | 48.74 m | 8.32 m | -82.9% |
| NTNU tunnel max reach ratio | 0.983 | 0.996 | no runaway |
| motion after reference stop | 40.65 m without stop gate | 0.011 m | stopped |
| 10 m point projection RMSE | 49.10 m | 10.44 m | -78.7% |
| 20 m point projection RMSE | 50.16 m | 15.11 m | -69.9% |
| NTNU fog | paired control | byte-identical | no corrections |
| HILTI exp07 | paired control | byte-identical | no corrections |
| MID-360 driving | paired control | byte-identical | no corrections |

Artifacts:

- `/media/sasaki/aiueo/benchmarks/lidar_degeneracy_datasets_v1/runs/radarless_tunnel_stop_gate_speed21_v1`

## Rejected variants

- Keeping the first agreement-anchor axis through low speed was stopped at
  100.5 s: reach was 42.87 m versus 146.84 m reference, with 6.4 m vertical
  error. The stale axis fixed the wrong direction.
- Re-anchoring from the last low LiDAR velocity after yaw reached only
  305.59 m. Five re-anchors occurred, but the retained velocity had already
  collapsed toward the zero-motion solution.
- Activity gating without preserving the anchor was behavior-identical to the
  415.59 m preset because yaw had already removed the state before low-speed
  activity could retain it.
- Yaw-rotated prior without an independent inactivity gate appeared to reach
  506.38 m, but 40.65 m of that motion occurred after the reference had
  stopped. Endpoint agreement alone hid the temporal failure.
- A 200 s decay time reduced endpoint reach to 449.28 m and worsened aligned
  RMSE to 22.25 m; longer authority retained turn-direction error.
- A 2.2 m/s propagated-speed cap reduced translation RMSE to 6.83 m, but
  overshot the reference by 4.8% and worsened 10/20 m point-placement RMSE
  relative to the adopted 2.1 m/s cap.

The final exp02, exp03, and exp21 holdouts remain untouched.
