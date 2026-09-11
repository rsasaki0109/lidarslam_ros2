^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package graph_based_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.1 (2026-08-12)
------------------
* Reject unsafe or inconsistent point-cloud layouts before conversion and
  preserve deterministic backend behavior after a rejected input.
* Add fail-closed first-map preflight, diagnosis, verification, local preview,
  and atomic finalization/recovery support for the maintained workflows.
* Add source-preserving map edit and multi-session merge plans with
  machine-readable receipts and focused regression coverage.
* Contributors: Ryohei Sasaki

0.9.0 (2026-07-30)
------------------
* Preserve event-driven deterministic backend behavior under the v0.9
  installed and package-manager golden-path contracts.
* Carry versioned readiness, real-map, recovery, and distribution evidence
  without changing the default loop-closure or map-output policy.
* Contributors: Ryohei Sasaki

0.7.0 (2026-07-29)
------------------
* Serialize event-driven backend work independently of the ROS executor, so
  composable deployments cannot concurrently mutate ``BackendCore`` or lose a
  submap notification that arrives while loop search is finishing.
* Move authoritative map/edge ownership into ``GraphStateStore``. PCD cache
  writes are staged before atomic state commits, and one immutable snapshot is
  reused across a batch of loop-search queries instead of deep-copying the
  complete ``MapArray`` for every query.
* Hide ``BackendCore``, registration, voxel filtering, and 3D-BBS state behind
  an implementation-only workspace, keeping g2o, pclomp, and descriptor
  database headers out of the public ROS component interface.
* Complete the component PImpl boundary: ROS subscriptions, configuration,
  graph state, and sensor/cache implementation details now live in a private
  source header, reducing the installed component header from 518 to 78 lines.
* Centralize all 143 ROS parameters in a typed, source-private
  ``GraphSlamConfig`` loader with startup validation, keeping parameter
  declaration separate from live graph and sensor state.
* Add pure composition builders for descriptor, loop-search, pose-graph,
  filtering, grid, and GNSS configuration, and freeze the validated startup
  snapshot before runtime initialization begins.
* Remove the legacy wall-clock loop-search path, add deterministic map-quality
  and offline-refinement gates, and preserve failure evidence during real
  output-filesystem exhaustion.
* Contributors: Ryohei Sasaki

0.6.0 (2026-06-12)
------------------
* Deterministic core / ROS shell refactor (v0.6 roadmap): the loop-closure
  pipeline (descriptor ingestion, candidate generation/rerank, verification,
  accepted-edge set, pose-graph optimization, map saving) now lives in
  tested pure headers assembled by a ROS-free ``BackendCore``.
* Event-driven loop search is the default (searches on submap arrival);
  the ``deterministic_loop_scheduling`` parameter is retired and the legacy
  wall-clock timer path stays behind ``event_driven_loop_search: false``
  for one release.
* New ``graph_slam_offline_runner``: replays a recorded odometry bag through
  the same core with no executor and no wall clock; three runs produce
  byte-identical loop edges and optimized trajectories on both release-gate
  substrates.
* Input path hardened: odometry+cloud subscriptions synchronized
  (``message_filters``) with a deep queue — fixes a silent ~6% frame drop —
  and IMU/GNSS graph edges now weight the correct EdgeSE3 error blocks.
* GNSS yaw alignment with gauge release: maps are georeferenced in the
  projector's local ENU frame when GNSS is enabled.
* Contributors: Ryohei Sasaki

0.5.0 (2026-06-11)
------------------
* First release prepared for the ROS 2 buildfarm (Humble / Jazzy).
* SPDX license tag (BSD-2-Clause).
* ``/map_save`` writes the Autoware map bundle: tiled ``pointcloud_map/`` with
  metadata plus ``map_projector_info.yaml`` (``local``, or ``LocalCartesian``
  when GNSS is enabled).
* Loop closure: NIS-driven adjacent-edge auto scaling, fitness-weighted loop
  edges, opt-in deterministic loop scheduling
  (``deterministic_loop_scheduling``, default off), and an opt-in BSD-2
  triangle-descriptor candidate source (``use_triangle_descriptor``, default
  off).
* Save-time dynamic-object cleanup and optional GNSS constraints.
* Contributors: Ryohei Sasaki
