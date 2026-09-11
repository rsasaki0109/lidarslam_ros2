#!/usr/bin/env python3
"""Validate a radar ego-velocity CSV against a reference (LIO) TUM trajectory.

Given a CSV produced by ``estimate_radar_ego_velocity.py`` (radar-frame
velocity per scan, assuming the v_r = +1 * (d . v) Doppler convention) and a
reference TUM trajectory (world<-base poses), this script:

1. Differentiates the reference trajectory into body(base)-frame velocity
   (finite difference over a ~0.3-0.5s window, rotated by R_wb^T).
2. Time-aligns radar scans to the reference velocity by linear interpolation
   of the two bracketing reference samples.
3. Determines the Doppler sign convention (+1 or -1) that best matches the
   reference, using a candidate radar->base rotation (published extrinsic by
   default, i.e. the NTNU dataset's "Radar extrinsics with respect to IMU").
4. Independently fits a radar->base rotation via the (vector) Orthogonal
   Procrustes / Wahba solution on the same correspondences (restricted to
   samples where the reference speed exceeds --min-speed), as a sanity check
   against the published extrinsic.
5. Reports per-axis correlation, speed RMS error, mean inlier count, and the
   fraction of scans with a "valid" estimate (>= --min-inliers inliers).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


# Published radar->IMU (base) extrinsic, NTNU LiDAR Degeneracy Datasets repo:
# https://github.com/ntnu-arl/lidar_degeneracy_datasets (README.md, "Extrinsics" section,
# "All extrinsics are given with respect to the IMU").
PUBLISHED_RADAR_TRANS = (0.07771, 0.02141, -0.03631)
PUBLISHED_RADAR_QUAT_XYZW = (0.953717, 0.0, -0.3007058, 0.0)


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def matrix_to_quat_xyzw(rotation: np.ndarray) -> tuple[float, float, float, float]:
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (m[2, 1] - m[1, 2]) / s
        qy = (m[0, 2] - m[2, 0]) / s
        qz = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        qw = (m[2, 1] - m[1, 2]) / s
        qx = 0.25 * s
        qy = (m[0, 1] + m[1, 0]) / s
        qz = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        qw = (m[0, 2] - m[2, 0]) / s
        qx = (m[0, 1] + m[1, 0]) / s
        qy = 0.25 * s
        qz = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        qw = (m[1, 0] - m[0, 1]) / s
        qx = (m[0, 2] + m[2, 0]) / s
        qy = (m[1, 2] + m[2, 1]) / s
        qz = 0.25 * s
    return float(qx), float(qy), float(qz), float(qw)


def rotation_angle_deg(r_a: np.ndarray, r_b: np.ndarray) -> float:
    r_diff = r_a @ r_b.T
    cos_angle = (np.trace(r_diff) - 1.0) / 2.0
    cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
    return math.degrees(math.acos(cos_angle))


def read_tum(path: Path) -> list[tuple[float, np.ndarray, np.ndarray]]:
    """Return list of (t, xyz, quat_xyzw) sorted by time."""
    poses = []
    with path.open('r', encoding='utf-8', errors='replace') as stream:
        for line in stream:
            text = line.strip()
            if not text or text.startswith('#'):
                continue
            parts = text.split()
            if len(parts) < 8:
                continue
            values = [float(v) for v in parts[:8]]
            t = values[0]
            xyz = np.array(values[1:4], dtype=np.float64)
            quat = np.array(values[4:8], dtype=np.float64)
            poses.append((t, xyz, quat))
    poses.sort(key=lambda item: item[0])
    return poses


def compute_body_velocity(
    poses: list[tuple[float, np.ndarray, np.ndarray]],
    smooth_window_sec: float,
) -> list[tuple[float, np.ndarray, np.ndarray]]:
    """Central-difference world velocity (smoothed over ~smooth_window_sec),
    rotated into the base frame at each sample. Returns list of
    (t, v_base, speed_world)."""
    times = np.array([p[0] for p in poses], dtype=np.float64)
    xyz = np.array([p[1] for p in poses], dtype=np.float64)
    half = smooth_window_sec / 2.0
    n = len(poses)
    results = []
    for i in range(n):
        t_i = times[i]
        # earliest index with times[before] <= t_i - half (fallback: 0)
        before = i
        while before > 0 and (t_i - times[before - 1]) < half:
            before -= 1
        if before == i and before > 0:
            before -= 1
        after = i
        while after < n - 1 and (times[after + 1] - t_i) < half:
            after += 1
        if after == i and after < n - 1:
            after += 1
        if before == after:
            continue
        dt = times[after] - times[before]
        if dt <= 1e-6:
            continue
        v_world = (xyz[after] - xyz[before]) / dt
        quat = poses[i][2]
        r_wb = quat_to_matrix(*quat)
        v_base = r_wb.T @ v_world
        results.append((t_i, v_base, float(np.linalg.norm(v_world))))
    return results


def interpolate_velocity(
    body_velocity: list[tuple[float, np.ndarray, np.ndarray]],
    t_query: float,
    time_tol: float,
) -> np.ndarray | None:
    times = [item[0] for item in body_velocity]
    idx = int(np.searchsorted(times, t_query))
    if idx <= 0:
        candidate = body_velocity[0]
        return candidate[1] if abs(candidate[0] - t_query) <= time_tol else None
    if idx >= len(body_velocity):
        candidate = body_velocity[-1]
        return candidate[1] if abs(candidate[0] - t_query) <= time_tol else None
    t0, v0, _ = body_velocity[idx - 1]
    t1, v1, _ = body_velocity[idx]
    if t_query - t0 > time_tol and t1 - t_query > time_tol:
        return None
    if t1 <= t0:
        return v0
    alpha = (t_query - t0) / (t1 - t0)
    alpha = min(max(alpha, 0.0), 1.0)
    return v0 + alpha * (v1 - v0)


def read_radar_csv(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open('r', newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            rows.append({
                'timestamp': float(row['timestamp']),
                'vx': float(row['vx']),
                'vy': float(row['vy']),
                'vz': float(row['vz']),
                'n_points': int(row['n_points']),
                'n_inliers': int(row['n_inliers']),
                'residual_rms': float(row['residual_rms']),
            })
    return rows


def pearson_corr(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) < 2 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return float('nan')
    return float(np.corrcoef(a, b)[0, 1])


def wahba_fit(radar_vecs: np.ndarray, base_vecs: np.ndarray) -> np.ndarray:
    """Solve for R minimizing sum||base_i - R @ radar_i||^2 (Orthogonal
    Procrustes / vector Wahba problem, no centroid subtraction)."""
    m = base_vecs.T @ radar_vecs  # sum_i b_i a_i^T
    u, _, vt = np.linalg.svd(m)
    d = np.sign(np.linalg.det(u @ vt))
    correction = np.diag([1.0, 1.0, d])
    return u @ correction @ vt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--radar-csv', required=True, type=Path)
    parser.add_argument('--baseline-tum', required=True, type=Path)
    parser.add_argument('--output-json', required=True, type=Path)
    parser.add_argument('--smooth-window-sec', type=float, default=0.4,
                         help='reference finite-difference smoothing window (0.3-0.5s recommended)')
    parser.add_argument('--time-tol', type=float, default=0.06,
                         help='max time gap (s) to accept a radar<->reference association')
    parser.add_argument('--min-speed', type=float, default=0.3,
                         help='reference speed threshold (m/s) for sign/rotation determination')
    parser.add_argument('--min-inliers', type=int, default=8,
                         help='inlier count threshold defining a "valid" scan estimate')
    parser.add_argument('--radar-trans', type=float, nargs=3, default=list(PUBLISHED_RADAR_TRANS))
    parser.add_argument('--radar-quat-xyzw', type=float, nargs=4, default=list(PUBLISHED_RADAR_QUAT_XYZW))
    parser.add_argument('--extrinsic-source', default=(
        'https://github.com/ntnu-arl/lidar_degeneracy_datasets (README.md, "Extrinsics" '
        'section: "All extrinsics are given with respect to the IMU", Radar block)'))
    args = parser.parse_args()

    radar_rows = read_radar_csv(args.radar_csv.expanduser().resolve())
    poses = read_tum(args.baseline_tum.expanduser().resolve())
    body_velocity = compute_body_velocity(poses, args.smooth_window_sec)

    r_base_radar_published = quat_to_matrix(*args.radar_quat_xyzw)

    # --- Time-align + gather correspondences ---
    matched_t: list[float] = []
    matched_radar: list[np.ndarray] = []
    matched_base: list[np.ndarray] = []
    matched_base_speed: list[float] = []
    n_scans = len(radar_rows)
    n_scan_valid_fit = 0  # rows with a non-nan velocity estimate
    inlier_counts: list[int] = []
    valid_estimate_count = 0

    for row in radar_rows:
        inlier_counts.append(row['n_inliers'])
        if row['n_inliers'] >= args.min_inliers:
            valid_estimate_count += 1
        if math.isnan(row['vx']):
            continue
        n_scan_valid_fit += 1
        v_base = interpolate_velocity(body_velocity, row['timestamp'], args.time_tol)
        if v_base is None:
            continue
        matched_t.append(row['timestamp'])
        matched_radar.append(np.array([row['vx'], row['vy'], row['vz']], dtype=np.float64))
        matched_base.append(v_base)
        matched_base_speed.append(float(np.linalg.norm(v_base)))

    matched_radar_arr = np.array(matched_radar, dtype=np.float64) if matched_radar else np.zeros((0, 3))
    matched_base_arr = np.array(matched_base, dtype=np.float64) if matched_base else np.zeros((0, 3))
    matched_base_speed_arr = np.array(matched_base_speed, dtype=np.float64)

    # --- Determine sign convention using published rotation ---
    # predicted radar-frame velocity = R_base_radar^T @ v_base
    predicted_radar = (r_base_radar_published.T @ matched_base_arr.T).T if len(matched_base_arr) else matched_base_arr
    dot_sum_plus = float(np.sum(matched_radar_arr * predicted_radar)) if len(matched_radar_arr) else 0.0
    sign = 1.0 if dot_sum_plus >= 0 else -1.0
    measured_radar_signed = sign * matched_radar_arr

    # --- Metrics using published rotation + determined sign ---
    corr_x = pearson_corr(measured_radar_signed[:, 0], predicted_radar[:, 0]) if len(matched_radar_arr) else float('nan')
    corr_y = pearson_corr(measured_radar_signed[:, 1], predicted_radar[:, 1]) if len(matched_radar_arr) else float('nan')
    corr_z = pearson_corr(measured_radar_signed[:, 2], predicted_radar[:, 2]) if len(matched_radar_arr) else float('nan')

    measured_speed = np.linalg.norm(measured_radar_signed, axis=1) if len(measured_radar_signed) else np.zeros(0)
    predicted_speed = np.linalg.norm(predicted_radar, axis=1) if len(predicted_radar) else np.zeros(0)
    speed_rms_error = float(np.sqrt(np.mean((measured_speed - predicted_speed) ** 2))) if len(measured_speed) else float('nan')
    vector_rms_error = float(np.sqrt(np.mean(np.sum((measured_radar_signed - predicted_radar) ** 2, axis=1)))) if len(measured_radar_signed) else float('nan')

    # --- Independent Wahba/Procrustes rotation fit (speed > min_speed subset) ---
    speed_mask = matched_base_speed_arr > args.min_speed
    n_wahba_samples = int(np.sum(speed_mask))
    wahba_quat = None
    wahba_matrix_list = None
    angle_to_published_deg = None
    if n_wahba_samples >= 3:
        radar_subset = measured_radar_signed[speed_mask]
        base_subset = matched_base_arr[speed_mask]
        r_wahba = wahba_fit(radar_subset, base_subset)
        wahba_quat = matrix_to_quat_xyzw(r_wahba)
        wahba_matrix_list = r_wahba.tolist()
        angle_to_published_deg = rotation_angle_deg(r_wahba, r_base_radar_published)

    report: dict[str, Any] = {
        'schema_version': 1,
        'radar_csv': str(args.radar_csv.expanduser().resolve()),
        'baseline_tum': str(args.baseline_tum.expanduser().resolve()),
        'smooth_window_sec': args.smooth_window_sec,
        'time_tol_sec': args.time_tol,
        'min_speed_mps': args.min_speed,
        'min_inliers': args.min_inliers,
        'sign_convention': {
            'determined_sign': sign,
            'model': 'measured_radial_velocity = sign * (direction . v_ego); '
                     'apply as final_radar_frame_velocity = determined_sign * radar_csv_velocity '
                     '(determined_sign=+1 means the CSV as given already uses the correct '
                     'physical convention; -1 means it must be negated)',
            'dot_product_sum_with_sign_plus1': dot_sum_plus,
        },
        'extrinsic_rotation_radar_to_base': {
            'published': {
                'quat_xyzw': list(args.radar_quat_xyzw),
                'translation_xyz': list(args.radar_trans),
                'source': args.extrinsic_source,
                'note': 'rotation used for validation predictions (R_base_radar); '
                        'translation not applied (lever-arm/angular-velocity term ignored, '
                        'per task spec: baseline body velocity is pure translational finite difference)',
            },
            'wahba_estimated': {
                'quat_xyzw': list(wahba_quat) if wahba_quat else None,
                'rotation_matrix': wahba_matrix_list,
                'n_samples_used': n_wahba_samples,
                'speed_threshold_mps': args.min_speed,
                'angle_to_published_deg': angle_to_published_deg,
                'note': 'orthogonal Procrustes / vector Wahba fit of R_base_radar solving '
                        'v_base ~= R @ (sign * v_radar_csv), sanity check only; '
                        'published rotation is used for the reported metrics below',
            },
        },
        'metrics': {
            'n_scans_total': n_scans,
            'n_scans_with_velocity_estimate': n_scan_valid_fit,
            'n_scans_time_aligned_to_baseline': len(matched_t),
            'fraction_scans_valid_min_inliers': (valid_estimate_count / n_scans) if n_scans else float('nan'),
            'mean_inlier_count': float(np.mean(inlier_counts)) if inlier_counts else float('nan'),
            'mean_n_points': float(np.mean([r['n_points'] for r in radar_rows])) if radar_rows else float('nan'),
            'correlation_vx': corr_x,
            'correlation_vy': corr_y,
            'correlation_vz': corr_z,
            'speed_rms_error_mps': speed_rms_error,
            'vector_rms_error_mps': vector_rms_error,
            'mean_measured_speed_mps': float(np.mean(measured_speed)) if len(measured_speed) else float('nan'),
            'mean_baseline_speed_mps': float(np.mean(matched_base_speed_arr)) if len(matched_base_speed_arr) else float('nan'),
        },
    }

    output = args.output_json.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=False) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2, sort_keys=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
