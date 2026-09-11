# Degeneracy Resilience Guide

RKO-LIO's degeneracy-resilience feature set is ~30 expert-tuned parameters
spread across several files. This page turns that into four presets under
[`lidarslam/param/presets/`](https://github.com/rsasaki0109/lidar_slam_ros2/tree/develop/lidarslam/param/presets) plus a symptom
lookup so you don't have to read the research notes to pick a config.

Everything here is opt-in and default-off; unmodified `rko_lio` behavior is
unchanged unless you layer one of these preset files in. The validated
numbers below come from
[`docs/research/lidar-degeneracy-radar-intensity-ab-2026-07.md`](research/lidar-degeneracy-radar-intensity-ab-2026-07.md)
(NTNU LiDAR Degeneracy Datasets fog/tunnel sequences, plus a HILTI 2022
exp07 negative-result check). Read that page for the full methodology.

## Is your odometry degenerating?

| Symptom | Root cause | Preset | Sensors needed |
| --- | --- | --- | --- |
| Trajectory reach is much shorter than the true traveled distance in a long, straight, self-similar corridor/tunnel | ICP can enter a confident zero-motion attractor: nearest-neighbour wall correspondences confirm "no motion" even while the platform moves | [`tunnel_radar.ros.yaml`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/lidarslam/param/presets/tunnel_radar.ros.yaml) if you have radar; [`tunnel_imu_no_radar.ros.yaml`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/lidarslam/param/presets/tunnel_imu_no_radar.ros.yaml) for a mostly-straight, known-speed LiDAR+IMU traverse; the older [`tunnel_intensity_no_radar.ros.yaml`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/lidarslam/param/presets/tunnel_intensity_no_radar.ros.yaml) only when reflectivity is distinctive | LiDAR + IMU (+ radar, strongly preferred) |
| Pose freezes in fog/dust/smoke despite real motion: point count drops, IMU still shows walking/driving vibration, reported velocity goes to ~0 | "Clutter lock" -- aerosol returns travel with the sensor, so ICP sees a **healthy, well-conditioned** Hessian and confidently reports near-zero motion. Eigenvalue-based weak-direction gates never fire in this mode -- only a second, disagreeing sensor can catch it | [`corridor_fog_radar.ros.yaml`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/lidarslam/param/presets/corridor_fog_radar.ros.yaml) | LiDAR + IMU + radar (required -- no radar-less fix is validated for this failure mode) |
| The whole traverse bends smoothly downward (or upward): a long, actually-level tunnel maps as a parabola-in-z (tens of meters of height sag over a few hundred meters) even though reach and lateral tracking look fine | Slow pitch/roll drift relative to gravity. The built-in `min_beta` orientation regularization cannot correct this by construction: its reference comes from a body-acceleration Kalman filter whose measurement is built with the *current* rotation estimate, so the reference drifts along with the estimate (measured on the NTNU tunnel: a 0.5 -> 6.3 deg monotonic gravity tilt that survives a 40x stronger `min_beta` weight) | `gravity_window_alignment: true` (included in [`tunnel_radar.ros.yaml`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/lidarslam/param/presets/tunnel_radar.ros.yaml) and [`corridor_fog_radar.ros.yaml`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/lidarslam/param/presets/corridor_fog_radar.ros.yaml)): the ~20 s window mean of raw accelerometer samples, rotated into the world frame, is an absolute up reference; the residual tilt is fed back as a small, capped, roll/pitch-only correction. Verify with `gravity_alignment_summary.json` (`dump_results: true`) -- `last_tilt_deg` should stay small. Preset-scoped on purpose: on a general handheld walk with frequent turns (HILTI exp07) it slightly *worsened* control-point APE (0.208 -> 0.264 m), and on a driving robot (MID-360) the magnitude gate kept it fully dormant (byte-identical trajectory) | LiDAR + IMU (accelerometer only -- no extra sensor) |
| Otherwise-good tracking but consistent drift along just one axis (e.g. lateral creep down a flat-walled corridor, or vertical creep under a low, featureless ceiling) | A single Hessian eigenvalue is persistently weak along that one world-frame axis. Confirm with `degeneracy_persistence.csv` (dumped when `dump_results: true`; columns include `confirmed`, `axis_tx..axis_rz`) before reaching for a preset -- it tells you which axis and how often it's confirmed | Same as the corridor/tunnel row if an external sensor (radar or a distinctively textured surface) observes that axis; otherwise `degeneracy_aware_solve` alone (in `tunnel_radar.ros.yaml` / `tunnel_intensity_no_radar.ros.yaml`, minus the radar/intensity gates) still blends a geometric/IMU prior into the confirmed weak direction, with no cross-sensor validation behind it | LiDAR + IMU, cross-sensor gate only if the weak axis is externally observable |

If none of these match -- registration is failing outright, or you were
"kidnapped" (jumped/lost track) -- see
[Honest limitations](#honest-limitations) below; this guide is about
graceful *degradation*, not recovery from a lost track.

## How to enable

Preset files under `lidarslam/param/presets/` contain **only** the
degeneracy-feature parameters (Hessian gates, radar fusion, intensity
gates). They deliberately do not set extrinsics, `voxel_size`,
`min_range`/`max_range`, or other base sensor parameters -- those stay in
your own sensor config. Layer a preset as a **second** `--params-file`,
after your sensor config, so its values win:

```bash
# Offline (rosbag) -- fog / clutter-lock example
ros2 run rko_lio offline_node --ros-args \
  --params-file <your_sensor_config>.ros.yaml \
  --params-file lidarslam/param/presets/corridor_fog_radar.ros.yaml \
  -p bag_path:=/path/to/bag \
  -p imu_topic:=/imu -p lidar_topic:=/lidar/points -p base_frame:=base_link \
  -p radar_topic:=/radar/cloud \
  -p dump_results:=true -p results_dir:=results -p run_name:=fog_run
```

```bash
# Online (live topics) -- tunnel example
ros2 run rko_lio online_node --ros-args \
  --params-file <your_sensor_config>.ros.yaml \
  --params-file lidarslam/param/presets/tunnel_radar.ros.yaml \
  -p imu_topic:=/imu -p lidar_topic:=/lidar/points -p base_frame:=base_link \
  -p radar_topic:=/radar/cloud
```

You can also drive this through
[`lidarslam/launch/rko_lio_slam.launch.py`](https://github.com/rsasaki0109/lidar_slam_ros2/blob/develop/lidarslam/launch/rko_lio_slam.launch.py)
by passing `rko_param_file:=<merged_yaml>` (that launch file accepts only
one RKO-LIO param file argument, so merge your sensor config and the chosen
preset into one file first if you use it instead of `ros2 run`).

Every preset (including `degeneracy_off.ros.yaml`, the documented no-op
baseline) has TODO-USER-marked knobs where the value is placeholder or
rig-specific -- search each file for `TODO-USER` before running it.

`offline_node` hangs at end-of-bag by design (it waits forever for a scan
past the last IMU sample); stop it with `timeout -s INT <seconds> ros2 run
...` so `dump_results` still fires on shutdown.

## How to verify it's working

With `dump_results: true`, `rko_lio` writes summary JSON files to
`<results_dir>/<run_name>_<index>/` on shutdown:

- **`radar_velocity_fusion_summary.json`** (written whenever
  `radar_velocity_fusion: true`):
  - `prior_attempt_count` -- scans where a radar ego-velocity estimate was
    computed and offered as a prior.
  - `fused_scan_count` -- scans where the Hessian-weak-direction prior
    actually got blended in. In the tunnel run this fired on 1981 scans; in
    fog it fired on only 84 scans and did not help by itself (fog's
    degeneracy isn't a Hessian weak-direction problem -- see the symptom
    table).
  - `disagreement_corrected_scan_count` -- scans where
    `radar_disagreement_gate` overrode ICP's translation. This is the
    number that matters for fog/clutter-lock and for the tunnel's
    remaining gap: healthy runs show hundreds of corrected scans over a
    fog/tunnel-length sequence, not a handful.
- **`intensity_constraint_summary.json`** (written whenever
  `intensity_constraint: true` or `intensity_disagreement_gate: true`):
  - `prior_attempt_count` / `prior_applied_count` -- persistence-gated
    profile-prior activity (`intensity_constraint`).
  - `disagreement_attempt_count` / `disagreement_valid_shift_count` --
    expect roughly 40% of attempts to yield a valid shift correlation; this
    is the normal hit rate for scan-to-scan reflectivity correlation, not a
    fault.
  - `disagreement_corrected_scan_count` -- scans where the gate overrode
    ICP. The validated radar-less tunnel recovery corrected 635 scans.
  - `disagreement_exceeded_threshold_count` -- **a high count here is not
    automatically good news.** On HILTI exp07's self-similar corridor, 52%
    of attempts exceeded the disagreement threshold and were "corrected" --
    and every one of those corrections made the trajectory worse (APE
    0.318 m -> 0.94-2.39 m). A busy counter means the gate is firing, not
    that it's helping; you still need to A/B the actual trajectory error or
    reach distance against `degeneracy_off.ros.yaml` before trusting a new
    environment.
- **`kinematic_velocity_blend_summary.json`** (written whenever
  `kinematic_velocity_blend: true`):
  - `anchor_refresh_count` -- scans where 3D ICP velocity agreed closely
    enough with independent IMU propagation to refresh trust.
  - `corrected_scan_count` and `mean_correction_m` -- how often and how far
    the optimized translation was moved.
  - `mean_propagation_weight`, `max_propagation_weight`, and
    `max_anchor_age_sec` -- whether the run spent long periods bridged by an
    old prior. These are diagnostics, not a universal pass threshold.
- `degeneracy_persistence.csv` -- per-scan diagnostics for the
  Hessian-eigenvalue gates (`confirmed`, `consecutive_scans`,
  `matched_absolute_cosine`, `intervention_count`, and the weak-axis
  vector). Use it to check *which* axis is degenerate before picking a
  preset, per the third symptom-table row.

There is no single "PASS" threshold -- these counters tell you the gates
are engaging, not that engagement is correct for your data. Always compare
against a ground truth or a known-good baseline run when moving to a new
environment.

## Honest limitations

- **Intensity gate applicability is narrow and unforgiving.** It only helps
  when geometry is self-similar but reflectivity texture is distinctive
  (the NTNU tunnel). It is *harmful* when the texture is also
  self-similar/periodic (HILTI exp07: 2.9-7.5x worse APE, at every
  threshold tried) and harmful when the texture is fake/sensor-coupled
  (fog aerosol: worse than baseline). There is no automatic detector for
  which case you're in yet -- you have to look at the data and A/B it.
- **Radar speed is systematically underestimated** by narrow-FOV
  single-chip radar hardware (observed ~0.62 speed ratio vs. LIO in the fog
  cross-check). `radar_velocity_scale` corrects this, but the correct value
  is a rig/geometry-specific calibration (1.05 for the NTNU tunnel radar,
  1.10 already overshoots), not a constant you can copy between rigs or
  even between environments on the same rig -- the same 1.05 that helps the
  tunnel makes fog slightly *worse*. Only tune it against a sequence with
  trustworthy distance ground truth.
- **Radar-less IMU velocity blending is straight-motion and speed-envelope
  specific.** `tunnel_imu_no_radar.ros.yaml` improves the radar-less NTNU
  tunnel reach from 102.3 m to 415.6 m; its maximum time-aligned pseudo-GT
  reach ratio is 0.983. Its 2.0 m/s propagated-speed cap is calibrated for a
  1.7 m/s walk, not a universal value. Sustained-yaw, walking-speed, and
  near/far range-distribution gates clear the anchor when the scene leaves the
  validated envelope. Sixty-second speed and scene cooldowns prevent isolated
  scans from rearming it. Paired runs on fog, HILTI exp07, and MID-360 were
  byte-identical to default-off controls, but the feature remains
  preset-scoped and default-off. There is still no validated radar-less fix
  for fog-style clutter lock.
- **Kidnap/registration-failure handling is stop-and-split, not
  relocalize, for mapping workflows.** All presets in this repo ship with
  `enable_kidnap_relocalization` and `reset_on_registration_failure` left
  at their defaults (off) in your own sensor config -- these presets don't
  touch them. Automatic global relocalization after a lost track has been
  observed to converge to a plausible-looking but wrong global pose in
  earlier sweeps on this codebase; treat a registration failure or long
  scan gap as a hard segment break to review and stitch manually (e.g. in
  the pose graph) rather than trusting an automatic global relocalization
  to silently recover the right pose.
