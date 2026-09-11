# CAD/BIM export — handoff

## Quick start (user-facing)

The whole HILTI bag -> SLAM map -> IFC flow now has one entry point. Existing
maps/IFCs are reused by default, so rerunning it is safe and fast:

```bash
python3 tools/colored_map/bim_pipeline.py exp01
python3 tools/colored_map/bim_pipeline.py /path/to/map.ply
```

Use `--dry-run` to preview, `--force-map` to rerun SLAM, and `--force-ifc` to
rerun only BIM extraction. Run `bim_pipeline.py --help` for all options.

Every HTML report also writes a sibling `*_metrics.json`. Gate the synthetic,
exp01 and exp07 evidence cases with the checked-in quality profile:

```bash
python3 scripts/evaluate_bim_regression_gate.py \
  --profile tools/colored_map/bim_regression_profile_v1.json \
  --case synthetic_closed_room=builtin:closed-room \
  --case exp01=/path/to/exp01_bim_metrics.json \
  --case exp07=/path/to/exp07_bim_metrics.json \
  --output /path/to/bim_regression_gate.json
```

The profile deliberately expects exp01 to have no accepted room: its open
construction walls are a negative test against false wall completion.

Point-cloud → CAD/BIM pipeline built on the "coloured-cloud hub" (`map_export` GIS
/ `mesh_export` / `las_export` / **`bim_export`**). This note hands the BIM branch
off so it can be picked up cold.

作った人からの引き継ぎ。`bim_export.py` = 点群から IFC(建物モデル) を作る枝。
純 numpy のロジック + gtest、重い依存(ifcopenshell)は writer 内で lazy import、という
既存 export 群の流儀を踏襲している。

## Files

- `tools/colored_map/bim_export.py` — extraction, topology, QA and IFC writer.
- `graph_based_slam/test/test_bim_export.py` — **72 tests**, all pure-numpy except
  4 IFC-writer tests (`pytest.importorskip('ifcopenshell')`).
- Run: `python3 -m pytest graph_based_slam/test/test_bim_export.py -q`
- Export suite plus the regression-gate tests currently **102 passed**.

## Pipeline / public API

Everything is orientation- and gravity-aware around world `up=(0,0,1)`; the SLAM
maps used here come out roughly z-up.

Pure geometry (numpy only, all unit-tested):
- `refit_plane(points) -> (normal, d)` — PCA plane.
- `fit_plane_ransac(points, *, threshold, iterations, min_inliers, orient, up, horiz_dot, vert_dot, seed)`
  → `(normal, d, mask)` | `None`. **`orient='vertical'|'horizontal'`** restricts the
  candidate normal orientation — this is how walls are found without per-point normals.
- `classify_plane(normal) -> 'horizontal'|'vertical'|'other'` (|n·up| ≥0.85 / ≤0.35).
- `plane_basis`, `oriented_rectangle`, `box_from_rectangle` — plane frame + rectangle + solid box.
- `largest_plane_patch(points, normal, cell)` — largest 4-connected patch; kills the
  spurious "horizontal slice through 3-D clutter" planes that raw RANSAC-by-count loves.
- `voxel_density_filter(points, voxel, min_count)` — drop sparse floaters (SLAM denoise).
- `_binary_dilate/_binary_erode/_binary_close(grid, iters)` — 2-D morphology.
- `detect_openings(wall_inliers, normal, *, cell, min_width, min_height, door_min_height, fill_ratio, close_iter)`
  → `[{kind:'door'|'window', corners(4,3), size, sill}]`. Empty grid regions enclosed on
  both sides + top; open at the base ⇒ door. `close_iter` bridges patchy scans.
- `principal_axis(walls) -> radians` and `regularize_walls(planes, points)` — snap wall
  normals to the building Manhattan grid (0/90° to the principal axis), re-fit rectangles.
- `reconstruct_rooms(planes, *, cell, close_iter, min_cells)` → `[{corners(4,3), height, area}]`
  — rasterise wall footprints, **dilate** to seal doorways, enclosed empty regions = rooms.
- `build_wall_graph` / `extract_room_cycles` — wall intersections to a planar
  graph, then deterministic half-edge cycle extraction (non-Manhattan rooms work).
- `optimize_wall_topology` — snaps or extends only when a closed cycle retains
  at least 85% observed boundary evidence; open scans are not force-closed.
- `evaluate_element_fit` — per-element Coverage, point-to-plane Distance
  RMSE/P95, and spatial Distribution. Values are written to IFC Psets, HTML,
  and the machine-readable metrics JSON.

Extraction:
- `extract_planes(points, *, colors, threshold, max_planes, min_inliers, min_remaining, patch_cell, find_openings, opening_close, orient, ...)`
  → list of plane dicts `{normal, d, indices, centroid, kind, corners, size, thickness, [color], [openings]}`.
  `indices` index into `points`.
- `extract_building(points, *, floor_planes, wall_planes, ...)` — **horizontal pass →
  remove those points → vertical pass on the remainder.** Needed because a plain
  vertical-orient RANSAC on the full cloud almost never samples 3 wall points when floors
  dominate (multi-storey). This is the function that makes walls appear.

IFC (lazy `ifcopenshell` 0.8.5):
- `write_ifc(planes, path, *, min_thickness, rooms)` — Project/Site/Building/Storey,
  `IfcSlab`/`IfcWall`/`IfcBuildingElementProxy` per plane, `IfcStyledItem` colour,
  `IfcOpeningElement`+`IfcDoor`/`IfcWindow`, `IfcSpace` rooms.
- `extract_and_export(xyz, path, *, rgb, thin_voxel, denoise_voxel, denoise_min_count,
  building, regularize, find_openings, opening_close, rooms, room_close, ...)` — the
  high-level entry. Returns `(path, planes)`.
- CLI: `bim_export.py in.ply out.ifc [--building --regularize --openings --rooms
  --denoise-voxel 0.15 --thin-voxel 0.1 --threshold 0.1 --min-inliers 600 ...]`.

## Non-obvious gotchas (these cost real time)

1. **ifcopenshell 0.8.5 renamed the `void` API to `feature`**:
   `run('feature.add_feature', model, feature=opening, element=wall)` (voids), and
   `run('feature.add_filling', model, opening=opening, element=door)` (fills).
   `IfcSpace` goes into the storey via `aggregate.assign_object`, not `spatial.assign_container`.
2. **Walls drown under floors** with plain RANSAC → always use `building=True` for indoor.
3. **Openings need face-on wall coverage.** exp07 (long corridor) is scanned at a grazing
   angle → ~40% coverage in a diagonal band, no *enclosed* holes → 0 openings (correct, it's
   a data limit, not a bug). exp01 (rooms scanned face-on) → real windows in aligned rows.
4. **Room reconstruction uses dilation-only, not full closing** — erosion re-severs a
   freshly-bridged thin wall line. Needs *closed* wall loops; construction sites (exp01)
   have open/unfinished walls, so real rooms barely form. Synthetic closed rooms work.
5. `np.ptp(arr)` — `arr.ptp()` was removed in this numpy.
6. Colours flow via `colors=`/`rgb=` and are carried through `thin_voxel`/`denoise` thinning.

## Data + how the real maps were made

- **HILTI 2022**: `/media/sasaki/aiueo/datasets/hilti2022` (`exp01/exp04/exp07`).
  **The external SSD must be mounted first** — `/dev/sda1` (label `aiueo`, ext4). Claude/Codex
  cannot mount it (polkit/sudo needs interactive auth); the user runs, in their own terminal,
  `udisksctl mount -b /dev/sda1`. See memory `hilti-dataset-location`.
  Topics: `/hesai/pandar` (PointCloud2, frame `PandarXT-32`, XYZI+ring), 5 grayscale fisheye
  cams (not useful for colour), IMU. **No `/tf`.** GT files are sparse survey control points,
  not a dense trajectory → must run SLAM for a map.
- **SLAM on HILTI** (`scanmatcher_node`): `robot_frame_id=PandarXT-32` (no tf → run in the
  LiDAR frame), `set_initial_pose=true`, indoor NDT (`ndt_resolution 1.0`). **Disable the
  car-tuned pose-acceptance gates** or every frame is rejected:
  `reject_fitness_ratio / reject_fitness_only_ratio / reject_trans_jump / reject_trans_jump_ratio = 1e9`,
  `motion_gate_enable=false`, `reject_nonconverged_pose_update=false`.
- **Capturing `/map`**: subscribe and write the PLY **on every callback** — the SIGINT-save
  pattern silently fails under rclpy's own SIGINT handler.
- Reference scripts are checked in at `tools/colored_map/bim_reference_scripts/`
  (`run_hilti_slam.sh`, `capture_map.py`) — they encode the working SLAM params above.
  **Their `OUT=` path still points at the old session scratchpad; edit it before running.**
  `exp07_map.ply` / `exp01_map.ply` were their SLAM outputs (in the scratchpad, may be gone).
- Outdoor colouring bag (separate branch): `/home/sasaki/autoware_data/all-sensors-bag1`.

## State of results (2026-07-13 real-data gate)

- exp01 (5,133,433 points): slabs 6, observed walls 20, synthetic walls 0,
  accepted rooms 0, rejected candidates 1, windows 1. This is intentionally a
  negative room-closure case. Mean fit: Coverage 0.218, Distribution 0.959,
  Distance RMSE 0.059 m.
- exp07 (6,274,297 points): slabs 6, observed walls 20, synthetic walls 1,
  accepted rooms 2, windows 5. Mean fit: Coverage 0.195, Distribution 0.964,
  Distance RMSE 0.057 m, P95 0.095 m.
- Synthetic closed room: four observed walls, one confirmed 24 m² room, zero
  synthetic walls and zero dangling ends.
- All three pass `bim_regression_profile_v1.json`; the gate result is a small
  JSON artifact suitable for CI or benchmark publication.

## Plane-revisit upstream A/B completed

The keyframe plane-revisit pose-graph factor now has a recorded MID-360 OFF/ON
gate. With the safe opt-in defaults, trajectory cross-validation RMSE improves
32.1%, planar thickness mean/P95 improve 0.20%/0.21%, BIM Coverage improves
25.0%, and BIM Distance RMSE/P95 improve 8.7%/8.0%. Distribution remains within
0.001 absolute. All nine combined checks pass, both trajectory and map-quality
reports pass three-run byte identity, and HILTI exp04 inserts zero constraints
as the no-revisit negative case. See
`docs/research/plane-revisit-pose-graph-2026-07.md` and run
`scripts/evaluate_plane_revisit_ab.py` for the machine-readable verdict.

The BIM CLI now reads the offline runner's ASCII/binary `.pcd` directly, so no
PLY conversion step is needed.

## Remaining dataset expansion

- Add a finished-building public sequence as a second positive room-topology
  holdout; exp01 remains valuable as the open-wall negative case.

Related memory: `pointcloud-coloring-export-hub` (full roadmap + every decision),
`hilti-dataset-location`, `agents-md-rtk-prefix-broken`.
