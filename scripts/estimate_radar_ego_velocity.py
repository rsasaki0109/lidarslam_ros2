#!/usr/bin/env python3
"""Estimate radar ego-velocity from Doppler PointCloud2 scans in a rosbag.

Each radar scan reports, per point, a 3D position (radar frame) and a radial
(Doppler) velocity.  For a rigid sensor moving through a mostly-static scene,
the radial velocity of every static point is (up to a sign convention) the
projection of the sensor's own velocity onto that point's line-of-sight
direction:

    v_r_i = sign * (d_i . v_ego),   d_i = p_i / ||p_i||

This script solves that linear system per scan with a 3-point-minimal-sample
RANSAC (to reject moving targets and noise), then a least-squares refit over
the inlier set, and writes one row per scan to a CSV file.

The sign convention is sensor/vendor specific and is deliberately left as a
CLI parameter (`--sign`) rather than hard-coded; use
`scripts/validate_radar_ego_velocity.py` to determine it empirically against
a reference trajectory.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any

import numpy as np


def _named_field(msg: Any, name: str) -> Any:
    for field in getattr(msg, 'fields', []) or []:
        if field.name == name:
            return field
    raise ValueError(f'PointCloud2 missing field: {name}')


def _numpy_dtype(datatype: int, is_bigendian: bool) -> Any:
    endian = '>' if is_bigendian else '<'
    mapping = {
        1: 'i1',
        2: 'u1',
        3: f'{endian}i2',
        4: f'{endian}u2',
        5: f'{endian}i4',
        6: f'{endian}u4',
        7: f'{endian}f4',
        8: f'{endian}f8',
    }
    if datatype not in mapping:
        raise ValueError(f'unsupported PointCloud2 datatype: {datatype}')
    return np.dtype(mapping[datatype])


def _point_field_array(msg: Any, name: str) -> np.ndarray:
    field = _named_field(msg, name)
    dtype = _numpy_dtype(int(field.datatype), bool(getattr(msg, 'is_bigendian', False)))
    height = int(getattr(msg, 'height', 0))
    width = int(getattr(msg, 'width', 0))
    if height <= 0 or width <= 0:
        return np.empty((0,), dtype=dtype)
    data = getattr(msg, 'data')
    buffer = data if hasattr(data, '__array_interface__') else bytes(data)
    values = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=buffer,
        offset=int(field.offset),
        strides=(int(msg.row_step), int(msg.point_step)),
    )
    return values.reshape(height * width).astype(np.float64, copy=True)


def stamp_sec(msg: Any) -> float:
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def load_scan_points(msg: Any) -> tuple[np.ndarray, np.ndarray]:
    """Return (directions [N,3] unit vectors, radial_velocity [N]) for finite,
    non-degenerate (range > 0) points in one radar scan."""
    x = _point_field_array(msg, 'x')
    y = _point_field_array(msg, 'y')
    z = _point_field_array(msg, 'z')
    v = _point_field_array(msg, 'velocity')
    points = np.column_stack((x, y, z))
    ranges = np.linalg.norm(points, axis=1)
    valid = np.isfinite(ranges) & np.isfinite(v) & (ranges > 1e-3)
    points = points[valid]
    v = v[valid]
    ranges = ranges[valid]
    directions = points / ranges[:, None]
    return directions, v


def ransac_fit_velocity(
    directions: np.ndarray,
    radial_velocity: np.ndarray,
    *,
    iters: int,
    inlier_threshold: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray | None, np.ndarray, float]:
    """Fit v_ego solving directions @ v_ego = radial_velocity (sign=+1
    convention baked into the linear model; the caller applies --sign as a
    post-hoc scalar on the returned vector, which is valid since the model
    is linear and homogeneous in v_ego).

    Returns (v_ego or None, inlier_mask, residual_rms). v_ego is None if no
    RANSAC sample produced a usable fit.
    """
    n = directions.shape[0]
    best_inliers = np.zeros(n, dtype=bool)
    best_count = -1
    idx_all = np.arange(n)
    for _ in range(iters):
        sample = rng.choice(idx_all, size=3, replace=False)
        d_samp = directions[sample]
        t_samp = radial_velocity[sample]
        try:
            v_samp = np.linalg.solve(d_samp, t_samp)
        except np.linalg.LinAlgError:
            continue
        residuals = radial_velocity - directions @ v_samp
        inliers = np.abs(residuals) < inlier_threshold
        count = int(np.sum(inliers))
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_count < 3:
        return None, best_inliers, float('nan')

    d_in = directions[best_inliers]
    t_in = radial_velocity[best_inliers]
    v_fit, *_ = np.linalg.lstsq(d_in, t_in, rcond=None)
    residuals = t_in - d_in @ v_fit
    residual_rms = float(np.sqrt(np.mean(residuals ** 2))) if len(residuals) else float('nan')
    return v_fit, best_inliers, residual_rms


def process_bag(
    bag: Path,
    topic: str,
    *,
    min_points: int,
    ransac_iters: int,
    inlier_threshold: float,
    sign: float,
    seed: int,
) -> list[dict[str, Any]]:
    from rosbags.highlevel import AnyReader

    rng = np.random.default_rng(seed)
    rows: list[dict[str, Any]] = []
    with AnyReader([bag]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            raise ValueError(f'topic not found in bag: {topic}')
        for connection, _, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, connection.msgtype)
            timestamp = stamp_sec(msg)
            directions, radial_velocity = load_scan_points(msg)
            n_points = int(len(radial_velocity))
            if n_points < min_points or n_points < 3:
                rows.append({
                    'timestamp': timestamp,
                    'vx': float('nan'),
                    'vy': float('nan'),
                    'vz': float('nan'),
                    'n_points': n_points,
                    'n_inliers': 0,
                    'residual_rms': float('nan'),
                })
                continue
            v_fit, inliers, residual_rms = ransac_fit_velocity(
                directions, radial_velocity,
                iters=ransac_iters, inlier_threshold=inlier_threshold, rng=rng)
            if v_fit is None:
                rows.append({
                    'timestamp': timestamp,
                    'vx': float('nan'),
                    'vy': float('nan'),
                    'vz': float('nan'),
                    'n_points': n_points,
                    'n_inliers': int(np.sum(inliers)),
                    'residual_rms': float('nan'),
                })
                continue
            v_out = sign * v_fit
            rows.append({
                'timestamp': timestamp,
                'vx': float(v_out[0]),
                'vy': float(v_out[1]),
                'vz': float(v_out[2]),
                'n_points': n_points,
                'n_inliers': int(np.sum(inliers)),
                'residual_rms': residual_rms,
            })
    return rows


def write_csv(rows: list[dict[str, Any]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = ['timestamp', 'vx', 'vy', 'vz', 'n_points', 'n_inliers', 'residual_rms']
    with output.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                'timestamp': f"{row['timestamp']:.9f}",
                'vx': _fmt(row['vx']),
                'vy': _fmt(row['vy']),
                'vz': _fmt(row['vz']),
                'n_points': row['n_points'],
                'n_inliers': row['n_inliers'],
                'residual_rms': _fmt(row['residual_rms']),
            })


def _fmt(value: float) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return 'nan'
    return f'{value:.6f}'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path, help='ROS1 or ROS2 bag path')
    parser.add_argument('--topic', default='/radar/cloud', help='PointCloud2 radar topic')
    parser.add_argument('--output', required=True, type=Path, help='output CSV path')
    parser.add_argument('--min-points', type=int, default=6,
                         help='minimum valid points in a scan to attempt an estimate')
    parser.add_argument('--ransac-iters', type=int, default=200, help='RANSAC iterations per scan')
    parser.add_argument('--inlier-threshold', type=float, default=0.15,
                         help='RANSAC inlier residual threshold in m/s')
    parser.add_argument('--sign', type=float, default=1.0,
                         help='sign applied to the fitted velocity vector '
                              '(model solves directions @ v = radial_velocity, sign=+1 baked in; '
                              'pass --sign -1 to flip the whole vector). '
                              'Determine empirically with validate_radar_ego_velocity.py')
    parser.add_argument('--seed', type=int, default=0, help='RANSAC RNG seed (for determinism)')
    args = parser.parse_args()

    bag = args.bag.expanduser().resolve()
    output = args.output.expanduser().resolve()
    rows = process_bag(
        bag, args.topic,
        min_points=args.min_points,
        ransac_iters=args.ransac_iters,
        inlier_threshold=args.inlier_threshold,
        sign=args.sign,
        seed=args.seed,
    )
    write_csv(rows, output)

    valid = [r for r in rows if not math.isnan(r['vx'])]
    print(f'scans={len(rows)} valid={len(valid)} output={output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
