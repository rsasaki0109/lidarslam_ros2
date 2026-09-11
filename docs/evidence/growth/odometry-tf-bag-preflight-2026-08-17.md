# Odometry-to-TF bag preflight — 2026-08-17

## Decision

Extend the existing read-only `lidarslam-map doctor <rosbag2_dir>` path instead
of adding a broadcaster, launch fork, or second diagnosis command. This closes
the repeated first-contact gap behind open issues #112 and #64: an Odometry
message can name `odom` and `base_link` without publishing the corresponding
transform.

The implementation adds preflight schema v5. Versions v1–v4 remain immutable
and usable for archived output.

## Bounded contract

When one or more `nav_msgs/msg/Odometry` topics are present, doctor:

- selects the highest-message-count Odometry topic deterministically;
- scans it and every `tf2_msgs/msg/TFMessage` topic up to 100,000 messages per
  topic;
- rejects empty, identical, or changing Odometry parent/child frame IDs;
- accepts a deterministic multi-hop TF path when at least one edge was recorded
  on dynamic `/tf`;
- distinguishes no path from a static-only path; and
- keeps a compatible mapping recommendation visible because the Odometry/TF
  finding does not prove that a maintained profile consumes that Odometry.

The JSON result includes bounded record counts, completeness, unique frame IDs,
TF topic counts, the selected frame path, and whether the path contains a
dynamic edge. Findings retain stable codes and one concrete next action:

- `odometry-frame-invalid`
- `odometry-tf-missing`
- `odometry-tf-static-only`
- `odometry-tf-inspection-unavailable`

## Claim boundary

This is recorded-bag connectivity evidence only. It does not prove live TF
freshness, transform interpolation at each sensor timestamp, clock alignment,
publisher frequency, calibration, map accuracy, or support for a new robot.
The workflow card keeps `tf2_echo` and `tf2_monitor` as the live checks. No bag,
parameter, transform, issue, or external state is modified.

## Regression surface

The focused suite covers a compatible mapping path with a visible missing-TF
finding, deterministic dynamic multi-hop acceptance, static-only rejection,
inconsistent Odometry frames, and serialized rosbag2 reading when ROS bindings
are available. The full validation command remains:

```bash
python3 -m pytest -q graph_based_slam/test/test_autoware_map_preflight.py
python3 -m pytest -q graph_based_slam/test/test_autoware_map_run_diagnosis.py
python3 -m pytest -q graph_based_slam/test/test_docs_entrypoints.py
python3 -m mkdocs build --strict
```

Exact-head release, bundle, publication-plan, and CI evidence must be recorded
after the implementation commit; this document grants no push, merge, release,
issue-edit, or community-post authority.

## Local implementation carrier

Exact implementation `402c23765fe125a2f42d7fd245d2a1c972a1ab34` passed:

- 68 focused preflight, diagnosis, and docs-entrypoint regressions, including a
  serialized Jazzy rosbag2 fixture;
- 1,483 graph tests with 13 known skips and 11 existing ImageIO warnings;
- 1,040 lidarslam tests;
- strict MkDocs and changed-file Jazzy `ament_flake8`; and
- `PLAN_VALID_LOCAL_ONLY` with 319 paths, seven slices, digest
  `fa6cd94d350c7ebf68c26ba8bb2c035b344543c374ea5123c367c4de4d8a437e`,
  and no remote mutation.

This carrier result is local and commit-bound. A later final candidate must
rerun bundle rehearsal, exact-head gates, and public CI rather than inheriting
the carrier result.
