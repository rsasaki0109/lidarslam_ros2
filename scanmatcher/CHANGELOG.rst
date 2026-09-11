^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scanmatcher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.9.1 (2026-08-12)
------------------
* Reject unsafe PCL VoxelGrid layouts, absolute voxel-index overflow, invalid
  leaf sizes, and inconsistent dense clouds before filtering. The affected
  scan, map update, or registration-target update is skipped with an actionable
  reason code while the node and last valid map/target remain active.
* Preserve point-for-point XYZ/intensity PCL output for valid clouds through a
  shared safety wrapper used by every classic scanmatcher VoxelGrid call site.
* Exercise the real ROS 2 component with an unsafe-then-safe cloud sequence;
  the overflow is rejected without output, then the same process publishes a
  map and pose for the later valid scan on Humble and Jazzy.
* Cover the asynchronous map-update path and safe component shutdown. Each
  worker uses the triggering scan's distance snapshot, shared diagnostics stay
  locked, and destruction waits for an outstanding map update to finish.
* Commit the map-update movement baseline only after the worker succeeds, so a
  rejected VoxelGrid layout does not consume the distance threshold needed by
  the next safe scan. Worker exceptions remain inside the component boundary.
* Contributors: Ryohei Sasaki

0.9.0 (2026-07-30)
------------------
* Prepare the NDT frontend for ROS buildfarm installation through the pinned,
  release-ready ``ndt_omp_ros2 0.1.0`` dependency.
* Keep oriented intensity/height appearance diagnostics and adapters
  default-off after fog and tunnel holdouts did not justify automatic channel
  selection.
* Preserve deterministic offline and installed product behavior across the
  Humble/Jazzy package-manager evidence contract.
* Contributors: Ryohei Sasaki

0.7.0 (2026-07-29)
------------------
* Preserve the deterministic offline frontend contract across Humble/Jazzy
  clean installs and the product CLI release path.
* Harden input validation and architecture boundaries without changing the
  default NDT/FastGICP/SmallGICP selection contract.
* Contributors: Ryohei Sasaki

0.6.0 (2026-06-12)
------------------
* Deterministic core / ROS shell refactor (v0.6 roadmap): pose prediction,
  pose acceptance (including the Tracking/Suspect/Recovery state machine),
  IMU orientation processing and the map-update/local-map policy moved into
  tested pure headers; the component shell keeps registration and pub/sub
  I/O only.
* New ``scan_matcher_offline_runner``: drives the component in lockstep over
  a raw sensor bag (intra-process pub/sub, drained single-threaded executor,
  synchronous map update); three runs over the full NTU VIRAL tnp_01 bag are
  byte-identical — the first determinism coverage for this frontend.
* Contributors: Ryohei Sasaki

0.5.0 (2026-06-11)
------------------
* First release prepared for the ROS 2 buildfarm (Humble / Jazzy).
* SPDX license tag (BSD-2-Clause).
* NDT (ndt_omp_ros2) scan-matching frontend; FastGICP / SmallGICP registration
  backends are enabled automatically when those optional packages are present
  at build time.
* IMU pre-integration and initial-pose options on the classic
  scanmatcher-frontend workflow.
* Contributors: Ryohei Sasaki
