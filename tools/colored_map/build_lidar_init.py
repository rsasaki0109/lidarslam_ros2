#!/usr/bin/env python3
"""Build a LiDAR-primed Gaussian init cloud from a bag + SLAM trajectory.

Accumulates PointCloud2 scans into the world (map) frame using the SLAM TUM
trajectory, voxel-downsamples, and writes a PLY that ``train_gsplat.py`` seeds
Gaussians from. This is the geometric prior at the heart of "LiDAR-primed 3DGS"
(``docs/research/3dgs-postprocess-map-design.md``): the metric LiDAR geometry
removes the need for COLMAP SfM and gives the optimiser correct positions.

The point cloud's ``header.stamp`` is already on the trajectory clock (the
trajectory is logged from the same scans), so no ``--time-offset`` is needed.
The transform/accumulation maths is delegated to ``posed_images`` /
``pointcloud_io`` (pure, tested); only the bag/PointCloud2 reading needs ROS.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Optional, Sequence

import numpy as np

# train_gsplat (the transforms.json loader) stays on the 3DGS side; make its
# directory importable regardless of which directory hosts the caller.
_GS_DIR = Path(__file__).resolve().parents[1] / 'gaussian_splatting'
if str(_GS_DIR) not in sys.path:
    sys.path.append(str(_GS_DIR))

import pointcloud_io as pcio  # noqa: E402
import posed_images as pi  # noqa: E402


def transform_points(points: np.ndarray, world_T_body: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to ``points`` (N,3)."""
    pts = np.asarray(points, dtype=np.float64)
    return (pts @ world_T_body[:3, :3].T) + world_T_body[:3, 3]


def compose_world_lidar(world_T_body: np.ndarray,
                        body_T_lidar: np.ndarray) -> np.ndarray:
    """Compose ``world <- LiDAR`` from a body trajectory and rig extrinsic."""
    return (np.asarray(world_T_body, dtype=np.float64) @
            np.asarray(body_T_lidar, dtype=np.float64))


def _read_pointcloud_xyz_time(msg) -> tuple[np.ndarray, Optional[np.ndarray]]:
    """Extract finite XYZ and optional absolute per-point timestamps."""
    from sensor_msgs_py import point_cloud2

    field_names = {field.name for field in msg.fields}
    time_name = next((name for name in ('timestamp', 'time', 't', 'offset_time')
                      if name in field_names), None)
    if time_name is None:
        pts = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        return pts[np.isfinite(pts).all(axis=1)], None

    records = point_cloud2.read_points(
        msg, field_names=('x', 'y', 'z', time_name), skip_nans=False)
    pts = np.stack([records['x'], records['y'], records['z']], axis=1)
    raw_time = np.asarray(records[time_name], dtype=np.float64)
    finite = np.isfinite(pts).all(axis=1) & np.isfinite(raw_time)
    pts = pts[finite].astype(np.float32)
    raw_time = raw_time[finite]
    header_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    # HILTI stores Unix seconds; common ROS drivers instead store seconds from
    # the scan header; Livox drivers store integer nanoseconds from the scan
    # header as ``offset_time``. Unknown large integer tick formats safely
    # fall back to a rigid scan rather than guessing their unit.
    if raw_time.size == 0:
        return pts, None
    if time_name == 'offset_time':
        if float(np.max(np.abs(raw_time))) > 2.0e9:
            return pts, None
        timestamps = header_time + raw_time * 1.0e-9
    elif float(np.median(np.abs(raw_time))) > 1.0e8:
        timestamps = raw_time
    elif float(np.max(np.abs(raw_time))) <= 10.0:
        timestamps = header_time + raw_time
    else:
        return pts, None
    if float(np.ptp(timestamps)) > 1.0:
        return pts, None
    return pts, timestamps


def deskew_points(points: np.ndarray, timestamps: np.ndarray,
                  samples: Sequence[pi.TrajectorySample],
                  body_T_lidar: np.ndarray, *, bin_seconds: float = 0.001,
                  max_extrapolation: float = 0.1) -> np.ndarray:
    """Transform points at acquisition time using binned trajectory poses."""
    pts = np.asarray(points, dtype=np.float64)
    stamps = np.asarray(timestamps, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or stamps.shape != (len(pts),):
        raise ValueError('points must be Nx3 and timestamps must be length N')
    if bin_seconds <= 0.0:
        raise ValueError('bin_seconds must be > 0')
    if len(pts) == 0:
        return np.empty((0, 3), dtype=np.float32)
    bins = np.floor((stamps - stamps.min()) / bin_seconds).astype(np.int64)
    world = np.empty_like(pts)
    for bin_id in np.unique(bins):
        selected = bins == bin_id
        stamp = float(np.mean(stamps[selected]))
        world_T_body = pi.interpolate_pose(
            samples, stamp, max_extrapolation=max_extrapolation)
        world_T_lidar = compose_world_lidar(world_T_body, body_T_lidar)
        world[selected] = transform_points(pts[selected], world_T_lidar)
    return world.astype(np.float32)


def clean_dynamic_map(
        world: np.ndarray,
        scans: Sequence[tuple[np.ndarray, np.ndarray]], *,
        algorithm: str = 'none', workers: int = 1,
        evidence_stride: int = 1,
        free_votes_fraction: float = 0.9, free_votes_floor: int = 2,
        void_min_scans: int = 11, cleaner_module: Any = None
        ) -> tuple[np.ndarray, dict]:
    """Optionally remove dynamic map points with pose-aware scan evidence.

    ``dynamic-object-removal`` stays an optional dependency.  Importing it is
    delayed until a cleaner is explicitly selected, so the default mapping
    path and ROS package dependencies remain unchanged.
    """
    points = np.asarray(world)
    if algorithm == 'none':
        return points, {
            'enabled': False, 'algorithm': 'none',
            'input_points': int(len(points)), 'kept_points': int(len(points)),
            'removed_points': 0, 'removed_ratio': 0.0,
        }
    if algorithm not in ('fusion', 'range', 'scan_ratio'):
        raise ValueError(f'unsupported dynamic cleaner: {algorithm}')
    if workers < 1:
        raise ValueError('dynamic cleaner workers must be at least one')
    if evidence_stride < 1:
        raise ValueError('dynamic cleaner evidence stride must be at least one')
    if not 0.0 < free_votes_fraction <= 1.0:
        raise ValueError('dynamic cleaner free-votes fraction must be in (0, 1]')
    if free_votes_floor < 1 or void_min_scans < 1:
        raise ValueError('dynamic cleaner vote thresholds must be positive')
    if cleaner_module is None:
        try:
            import dynamic_object_removal as cleaner_module
        except ImportError as exc:
            raise RuntimeError(
                'dynamic map cleaning requires: '
                'pip install "dynamic-object-removal>=0.5"') from exc
    scan_evidence = [(np.asarray(scan), np.asarray(origin))
                     for scan, origin in scans]
    if not scan_evidence:
        raise ValueError('dynamic map cleaning requires pose-aligned scans')
    if algorithm == 'fusion':
        _, keep = cleaner_module.clean_map_by_fusion(
            points, scan_evidence, workers=workers,
            free_votes_fraction=free_votes_fraction,
            free_votes_floor=free_votes_floor,
            void_min_scans=void_min_scans)
    elif algorithm == 'range':
        ground_z = float(np.percentile(points[:, 2], 2))
        _, keep = cleaner_module.clean_map_by_visibility(
            points, scan_evidence, ground_z=ground_z)
    else:
        _, keep = cleaner_module.clean_map_by_scan_ratio(points, scan_evidence)
    keep = np.asarray(keep, dtype=bool)
    if keep.shape != (len(points),):
        raise RuntimeError('dynamic cleaner returned an invalid keep mask')
    kept = points[keep]
    removed = int(len(points) - len(kept))
    return kept, {
        'enabled': True, 'algorithm': algorithm,
        'implementation': 'dynamic-object-removal',
        'implementation_version': str(
            getattr(cleaner_module, '__version__', 'unknown')),
        'scans': int(len(scan_evidence)),
        'input_points': int(len(points)), 'kept_points': int(len(kept)),
        'removed_points': removed,
        'removed_ratio': float(removed / len(points)) if len(points) else 0.0,
        'workers': int(workers),
        'evidence_stride': int(evidence_stride),
        'free_votes_fraction': float(free_votes_fraction),
        'free_votes_floor': int(free_votes_floor),
        'void_min_scans': int(void_min_scans),
    }


def build(args: argparse.Namespace) -> dict:
    """Accumulate scans into world points and write the init PLY."""
    if args.dynamic_map_cleaner_evidence_stride < 1:
        raise ValueError('dynamic cleaner evidence stride must be at least one')
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    samples = pi.read_tum_trajectory(args.traj)
    body_T_lidar = np.eye(4)
    if args.lidar_calibration:
        from extract_posed_images import load_parented_sensor_extrinsic
        body_T_lidar = load_parented_sensor_extrinsic(
            args.lidar_calibration, args.lidar_key)

    import rosbag2_py
    # Reuse the extractor's reader factory so FILE-compressed (zstd) bags work.
    from extract_posed_images import _open_reader
    reader = _open_reader(args.bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.points_topic]))

    chunks: list[np.ndarray] = []
    dynamic_scans: list[tuple[np.ndarray, np.ndarray]] = []
    used = 0
    deskewed = 0
    skipped = 0
    seen = 0
    t0 = None
    while reader.has_next():
        tname, raw, bagt = reader.read_next()
        if tname != args.points_topic:
            continue
        rel_t = 0.0 if t0 is None else (bagt * 1e-9 - t0)
        if t0 is None:
            t0 = bagt * 1e-9
        if args.end_time >= 0 and rel_t > args.end_time:
            break
        if rel_t < args.start_time:
            continue
        if args.stride > 1 and seen % args.stride != 0:
            seen += 1
            continue
        seen += 1
        msg = deserialize_message(raw, PointCloud2)
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            world_T_body = pi.interpolate_pose(
                samples, ts, max_extrapolation=args.max_extrapolation)
        except ValueError:
            skipped += 1
            continue
        pts, point_timestamps = _read_pointcloud_xyz_time(msg)
        if args.max_range > 0 or args.min_range > 0:
            rng = np.linalg.norm(pts, axis=1)
            keep = rng >= args.min_range
            if args.max_range > 0:
                keep &= rng <= args.max_range
            pts = pts[keep]
            if point_timestamps is not None:
                point_timestamps = point_timestamps[keep]
        # PointCloud2 XYZ is expressed in the LiDAR frame, while the SLAM TUM
        # trajectory is world <- body/IMU. Compose both transforms explicitly;
        # treating raw LiDAR points as body-frame points rotates every scan
        # around the wrong axes on rigs such as HILTI's PandarXT-32.
        world_T_lidar = compose_world_lidar(world_T_body, body_T_lidar)
        if args.deskew and point_timestamps is not None:
            world_pts = deskew_points(
                pts, point_timestamps, samples, body_T_lidar,
                bin_seconds=args.deskew_bin_ms * 1e-3,
                max_extrapolation=args.max_extrapolation)
            deskewed += 1
        else:
            world_pts = transform_points(pts, world_T_lidar).astype(np.float32)
        # Downsample each scan before accumulating so peak memory is bounded by
        # the downsampled cloud, not the sum of every raw scan (a long bag is
        # tens of millions of points before any reduction).
        world_pts, _ = pcio.voxel_downsample(world_pts, args.voxel)
        chunks.append(world_pts)
        if (args.dynamic_map_cleaner != 'none' and
                used % args.dynamic_map_cleaner_evidence_stride == 0):
            dynamic_scans.append((world_pts, world_T_lidar[:3, 3].copy()))
        used += 1

    if not chunks:
        raise RuntimeError('no scans accumulated; check --points-topic / trajectory')
    world = np.concatenate(chunks, axis=0)
    # Final pass dedups points that fell in the same voxel across scan boundaries.
    world, _ = pcio.voxel_downsample(world, args.voxel)
    world, dynamic_cleaning = clean_dynamic_map(
        world, dynamic_scans, algorithm=args.dynamic_map_cleaner,
        workers=args.dynamic_map_cleaner_workers,
        evidence_stride=args.dynamic_map_cleaner_evidence_stride,
        free_votes_fraction=args.dynamic_map_cleaner_free_votes_fraction,
        free_votes_floor=args.dynamic_map_cleaner_free_votes_floor,
        void_min_scans=args.dynamic_map_cleaner_void_min_scans)
    chunks.clear()
    if args.max_points > 0 and world.shape[0] > args.max_points:
        rng = np.random.default_rng(0)
        world = world[rng.choice(world.shape[0], args.max_points, replace=False)]
    if args.min_neighbors > 0:
        keep = pcio.drop_sparse_points(world, args.min_neighbors, args.sparse_voxel)
        world = world[keep]
    rgb = None
    colored = 0
    fusion_diagnostics = None
    if args.color_transforms:
        color_result = _colorize(
            world, args.color_transforms, robust=args.color_robust,
            normalize_exposure=args.color_normalize_exposure,
            exposure_scale_limit=args.color_exposure_scale_limit,
            max_samples=args.color_max_samples,
            image_margin=args.color_image_margin,
            vignette_gain_limit=args.color_vignette_gain_limit,
            overlap_color_balance=args.color_overlap_balance,
            view_confidence=args.color_view_confidence,
            normal_voxel=args.color_normal_voxel,
            min_view_cosine=args.color_min_view_cosine,
            min_projected_scale=args.color_min_projected_scale,
            view_score_power=args.color_view_score_power,
            min_samples=args.color_min_samples,
            geometry_aware=args.color_geometry_aware,
            occlusion_margin_px=args.color_occlusion_margin_px,
            depth_edge_margin_px=args.color_depth_edge_margin_px,
            depth_edge_tolerance=args.color_depth_edge_tolerance,
            depth_edge_relative_tolerance=(
                args.color_depth_edge_relative_tolerance),
            dynamic_exclusion=args.color_dynamic_exclusion,
            dynamic_mask_margin_px=args.color_dynamic_mask_margin_px,
            calibration_sigma_multiplier=(
                args.color_calibration_sigma_multiplier),
            maximum_uncertainty_margin_px=(
                args.color_max_uncertainty_margin_px),
            return_diagnostics=args.color_geometry_aware)
        if args.color_geometry_aware:
            rgb, seen, fusion_diagnostics = color_result
        else:
            rgb, seen = color_result
        colored = int(seen.sum())
    out = pcio.write_ply(args.out, world, rgb)
    if args.dynamic_map_cleaner_report:
        report_path = Path(args.dynamic_map_cleaner_report)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(
            json.dumps(dynamic_cleaning, indent=2) + '\n', encoding='utf-8')
    return {'scans_used': used, 'scans_deskewed': deskewed,
            'scans_skipped': skipped, 'points': int(world.shape[0]),
            'colored': colored, 'fusion_diagnostics': fusion_diagnostics,
            'dynamic_cleaning': dynamic_cleaning,
            'out': str(out)}


def _colorize(world: np.ndarray, transforms_path: str, *, robust: bool = False,
              normalize_exposure: bool = True,
              exposure_scale_limit: float = 1.5,
              max_samples: int = 12,
              image_margin: int = 0,
              vignette_gain_limit: float = 1.0,
              overlap_color_balance: bool = False,
              view_confidence: bool = False,
              normal_voxel: float = 0.12,
              min_view_cosine: float = 0.0,
              min_projected_scale: float = 0.0,
              view_score_power: float = 0.0,
              min_samples: int = 1,
              geometry_aware: bool = False,
              occlusion_margin_px: int = 0,
              depth_edge_margin_px: int = 0,
              depth_edge_tolerance: float = 1.0,
              depth_edge_relative_tolerance: float = 0.10,
              dynamic_exclusion: bool = False,
              dynamic_mask_margin_px: int = 2,
              calibration_sigma_multiplier: float = 0.0,
              maximum_uncertainty_margin_px: int = 8,
              return_diagnostics: bool = False):
    """Project ``world`` points into the posed images of a transforms.json."""
    import imageio as iio
    import train_gsplat as tg

    ds = tg.load_transforms(transforms_path)
    images = [np.asarray(iio.imread(p)) for p in ds['image_paths']]
    if not robust:
        return pcio.colorize_by_projection(
            world, ds['viewmats'], ds['K'], images, ds['width'], ds['height'])
    normals = (pcio.estimate_voxel_normals(world, voxel=normal_voxel)
               if view_confidence else None)
    if dynamic_exclusion and not geometry_aware:
        raise ValueError('dynamic exclusion requires geometry-aware fusion')
    exclusion_masks = None
    if geometry_aware and dynamic_exclusion:
        mask_paths = ds['dynamic_mask_paths']
        if any(path is None for path in mask_paths):
            raise ValueError('dynamic exclusion requires dynamic_mask_path for '
                             'every transforms frame')
        exclusion_masks = [
            np.asarray(iio.imread(path)) != 0 for path in mask_paths]
        exclusion_masks = [
            np.any(mask, axis=2) if mask.ndim == 3 else mask
            for mask in exclusion_masks]
    calibration = (ds['spatiotemporal_calibration']
                   if geometry_aware and calibration_sigma_multiplier > 0.0
                   else None)
    result = pcio.colorize_by_projection_robust(
        world, ds['viewmats'], ds['K'], images, ds['width'], ds['height'],
        normalize_exposure=normalize_exposure,
        exposure_scale_limit=exposure_scale_limit,
        max_samples=max_samples, image_margin=image_margin,
        vignette_gain_limit=vignette_gain_limit,
        overlap_color_balance=overlap_color_balance,
        point_normals=normals,
        min_view_cosine=min_view_cosine,
        min_projected_scale=min_projected_scale,
        view_score_power=view_score_power,
        occlusion_margin_px=occlusion_margin_px if geometry_aware else 0,
        depth_edge_margin_px=depth_edge_margin_px if geometry_aware else 0,
        depth_edge_tolerance=depth_edge_tolerance,
        depth_edge_relative_tolerance=depth_edge_relative_tolerance,
        exclusion_masks=exclusion_masks,
        dynamic_mask_margin_px=dynamic_mask_margin_px,
        calibration=calibration,
        view_timestamps=(ds['timestamps'] if calibration is not None else None),
        calibration_sigma_multiplier=(
            calibration_sigma_multiplier if geometry_aware else 0.0),
        maximum_uncertainty_margin_px=maximum_uncertainty_margin_px,
        return_counts=True, return_diagnostics=return_diagnostics)
    if return_diagnostics:
        rgb, seen, counts, diagnostics = result
    else:
        rgb, seen, counts = result
    if min_samples > 1:
        # Colours confirmed by too few camera observations are unreliable
        # (occlusion-fringe or specular one-offs that pepper flat surfaces);
        # demote them to unseen so RGB consumers drop them.
        low = seen & (counts < min_samples)
        rgb[low] = (128, 128, 128)
        seen = seen & ~low
    return (rgb, seen, diagnostics) if return_diagnostics else (rgb, seen)


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--bag', required=True)
    p.add_argument('--traj', required=True, help='SLAM trajectory (TUM, world<-body)')
    p.add_argument('--points-topic', default='/livox/points')
    p.add_argument('--lidar-calibration', default=None,
                   help='HILTI-style sensor-tree YAML containing parent<-LiDAR')
    p.add_argument('--lidar-key', default='PandarXT-32',
                   help='sensor key inside --lidar-calibration')
    p.add_argument('--out', required=True, help='output init .ply')
    p.add_argument('--voxel', type=float, default=0.1, help='voxel size (m)')
    p.add_argument('--max-range', type=float, default=80.0, help='drop points beyond (m)')
    p.add_argument('--min-range', type=float, default=0.0,
                   help='drop points closer than this (m); cuts the operator/'
                        'rig ghost that a handheld scanner sweeps along the '
                        'whole trajectory (unseen by the camera, it survives '
                        'training as a fog tube on the camera path)')
    p.add_argument('--max-points', type=int, default=300000, help='cap final point count')
    p.add_argument('--dynamic-map-cleaner',
                   choices=('none', 'fusion', 'range', 'scan_ratio'),
                   default='none',
                   help='optional pose-aware dynamic-object removal; requires '
                        'dynamic-object-removal>=0.5')
    p.add_argument('--dynamic-map-cleaner-workers', type=int, default=1)
    p.add_argument('--dynamic-map-cleaner-evidence-stride', type=int,
                   default=1,
                   help='use every Nth accumulated scan as cleaning evidence')
    p.add_argument('--dynamic-map-cleaner-free-votes-fraction', type=float,
                   default=0.9)
    p.add_argument('--dynamic-map-cleaner-free-votes-floor', type=int,
                   default=2)
    p.add_argument('--dynamic-map-cleaner-void-min-scans', type=int,
                   default=11)
    p.add_argument('--dynamic-map-cleaner-report', default=None,
                   help='write dynamic point-removal provenance JSON')
    p.add_argument('--max-extrapolation', type=float, default=0.1)
    p.add_argument('--no-deskew', action='store_false', dest='deskew',
                   help='ignore per-point timestamps and transform each scan '
                        'at its header time')
    p.add_argument('--deskew-bin-ms', type=float, default=1.0,
                   help='pose bin width for per-point deskew (default 1 ms)')
    p.add_argument('--stride', type=int, default=1, help='use every Nth scan')
    p.add_argument('--start-time', type=float, default=0.0,
                   help='use scans at/after this many seconds from bag start')
    p.add_argument('--end-time', type=float, default=-1.0,
                   help='use scans up to this many seconds from bag start (-1 = all)')
    p.add_argument('--color-transforms', default=None,
                   help='transforms.json (+ images) to colour the init cloud by '
                        'projection; seeds Gaussian colour instead of flat grey')
    p.add_argument('--color-robust', action='store_true',
                   help='use the occlusion-aware / exposure-normalised / median '
                        'colorizer instead of the plain all-view average (slower; '
                        'much cleaner colours for map flythroughs)')
    p.add_argument('--color-no-normalize-exposure', action='store_false',
                   dest='color_normalize_exposure',
                   help='keep camera RGB values unchanged instead of applying '
                        'per-view global-median exposure gains')
    p.add_argument('--color-exposure-scale-limit', type=float, default=1.5,
                   help='maximum robust-color exposure gain and reciprocal loss')
    p.add_argument('--color-max-samples', type=int, default=12,
                   help='nearest valid camera observations retained per point')
    p.add_argument('--color-min-samples', type=int, default=1,
                   help='demote robust colours confirmed by fewer surviving '
                        'camera samples than this to unseen (1 keeps all)')
    p.add_argument('--color-image-margin', type=int, default=0,
                   help='ignore colour samples within this many pixels of the '
                        'image border (skips lens-vignette darkening; 0 keeps '
                        'the full frame)')
    p.add_argument('--color-vignette-gain-limit', type=float, default=1.0,
                   help='estimate and apply a shared radial luminance gain, '
                        'clamped to this value (1 disables correction)')
    p.add_argument('--color-overlap-balance', action='store_true',
                   help='solve per-frame exposure and white balance from RGB '
                        'of shared visible 3D points')
    p.add_argument('--color-view-confidence', action='store_true',
                   help='prefer observations with strong incidence angle and '
                        'projected resolution using voxel normals')
    p.add_argument('--color-normal-voxel', type=float, default=0.12)
    p.add_argument('--color-min-view-cosine', type=float, default=0.0)
    p.add_argument('--color-min-projected-scale', type=float, default=0.0)
    p.add_argument('--color-view-score-power', type=float, default=1.0)
    p.add_argument('--color-geometry-aware', action='store_true',
                   help='enable depth-edge, silhouette, dynamic-mask, and '
                        'calibration-uncertainty guards')
    p.add_argument('--color-occlusion-margin-px', type=int, default=0)
    p.add_argument('--color-depth-edge-margin-px', type=int, default=0)
    p.add_argument('--color-depth-edge-tolerance', type=float, default=1.0)
    p.add_argument('--color-depth-edge-relative-tolerance', type=float,
                   default=0.10)
    p.add_argument('--color-dynamic-exclusion', action='store_true',
                   help='exclude per-frame dynamic_mask_path pixels')
    p.add_argument('--color-dynamic-mask-margin-px', type=int, default=2)
    p.add_argument('--color-calibration-sigma-multiplier', type=float,
                   default=0.0)
    p.add_argument('--color-max-uncertainty-margin-px', type=int, default=8)
    p.add_argument('--min-neighbors', type=int, default=2,
                   help='drop points whose 3x3x3 voxel neighbourhood (see '
                        '--sparse-voxel) holds fewer points; default 2 requires '
                        'one supporting neighbour, 0 disables')
    p.add_argument('--sparse-voxel', type=float, default=0.1,
                   help='voxel size (m) for the --min-neighbors density filter')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    summary = build(args)
    print(f"accumulated {summary['scans_used']} scans "
          f"({summary['scans_deskewed']} deskewed, "
          f"{summary['scans_skipped']} skipped) -> "
          f"{summary['points']} points "
          f"({summary['colored']} coloured) -> {summary['out']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
