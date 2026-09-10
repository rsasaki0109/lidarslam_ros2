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
from typing import Optional, Sequence

import numpy as np

import lidarslam_benchmark_tools.gaussian_splatting.pointcloud_io as pcio
import lidarslam_benchmark_tools.gaussian_splatting.posed_images as pi


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
    time_name = next((name for name in ('timestamp', 'time', 't')
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
    # the scan header. Unknown large integer tick formats safely fall back to a
    # rigid scan rather than guessing their unit.
    if raw_time.size == 0:
        return pts, None
    if float(np.median(np.abs(raw_time))) > 1.0e8:
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


def build(args: argparse.Namespace) -> dict:
    """Accumulate scans into world points and write the init PLY."""
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2

    samples = pi.read_tum_trajectory(args.traj)
    body_T_lidar = np.eye(4)
    if args.lidar_calibration:
        from lidarslam_benchmark_tools.gaussian_splatting.extract_posed_images import load_parented_sensor_extrinsic
        body_T_lidar = load_parented_sensor_extrinsic(
            args.lidar_calibration, args.lidar_key)

    import rosbag2_py
    # Reuse the extractor's reader factory so FILE-compressed (zstd) bags work.
    from lidarslam_benchmark_tools.gaussian_splatting.extract_posed_images import _open_reader
    reader = _open_reader(args.bag)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.points_topic]))

    chunks: list[np.ndarray] = []
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
        if args.deskew and point_timestamps is not None:
            world_pts = deskew_points(
                pts, point_timestamps, samples, body_T_lidar,
                bin_seconds=args.deskew_bin_ms * 1e-3,
                max_extrapolation=args.max_extrapolation)
            deskewed += 1
        else:
            world_T_lidar = compose_world_lidar(world_T_body, body_T_lidar)
            world_pts = transform_points(pts, world_T_lidar).astype(np.float32)
        # Downsample each scan before accumulating so peak memory is bounded by
        # the downsampled cloud, not the sum of every raw scan (a long bag is
        # tens of millions of points before any reduction).
        world_pts, _ = pcio.voxel_downsample(world_pts, args.voxel)
        chunks.append(world_pts)
        used += 1

    if not chunks:
        raise RuntimeError('no scans accumulated; check --points-topic / trajectory')
    world = np.concatenate(chunks, axis=0)
    # Final pass dedups points that fell in the same voxel across scan boundaries.
    world, _ = pcio.voxel_downsample(world, args.voxel)
    if args.max_points > 0 and world.shape[0] > args.max_points:
        rng = np.random.default_rng(0)
        world = world[rng.choice(world.shape[0], args.max_points, replace=False)]
    if args.min_neighbors > 0:
        keep = pcio.drop_sparse_points(world, args.min_neighbors, args.sparse_voxel)
        world = world[keep]
    rgb = None
    colored = 0
    if args.color_transforms:
        rgb, seen = _colorize(
            world, args.color_transforms, robust=args.color_robust,
            normalize_exposure=args.color_normalize_exposure,
            exposure_scale_limit=args.color_exposure_scale_limit,
            max_samples=args.color_max_samples)
        colored = int(seen.sum())
    out = pcio.write_ply(args.out, world, rgb)
    return {'scans_used': used, 'scans_deskewed': deskewed, 'scans_skipped': skipped,
            'points': int(world.shape[0]), 'colored': colored, 'out': str(out)}


def _colorize(world: np.ndarray, transforms_path: str, *, robust: bool = False,
              normalize_exposure: bool = True,
              exposure_scale_limit: float = 1.5,
              max_samples: int = 12):
    """Project ``world`` points into the posed images of a transforms.json."""
    import imageio.v3 as iio
    import lidarslam_benchmark_tools.gaussian_splatting.train_gsplat as tg

    ds = tg.load_transforms(transforms_path)
    images = [np.asarray(iio.imread(p)) for p in ds['image_paths']]
    fn = (pcio.colorize_by_projection_robust if robust
          else pcio.colorize_by_projection)
    kwargs = {}
    if robust:
        kwargs = {
            'normalize_exposure': normalize_exposure,
            'exposure_scale_limit': exposure_scale_limit,
            'max_samples': max_samples,
        }
    return fn(
        world, ds['viewmats'], ds['K'], images, ds['width'], ds['height'],
        **kwargs)


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
