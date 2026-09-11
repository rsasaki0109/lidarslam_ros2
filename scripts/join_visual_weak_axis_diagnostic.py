#!/usr/bin/env python3
"""Join sensor-only visual motion with a frozen runtime weak-axis diagnostic.

This is a report-only join.  It uses runtime odometry attitude, a runtime
LiDAR weak-eigenvector CSV, and IMU acceleration only to express the visual
base-frame direction in the runtime world frame and project it into the
gravity-orthogonal weak direction.  It never reads a reference trajectory or
ground-truth file and never changes a bag or a SLAM state.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader


def quaternion_xyzw_rotation(values: list[float] | tuple[float, ...]) -> np.ndarray:
    """Return the rotation represented by an xyzw quaternion."""
    x, y, z, w = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm((x, y, z, w)))
    if norm <= 1.0e-12:
        raise ValueError('quaternion norm must be positive')
    x, y, z, w = np.asarray((x, y, z, w), dtype=np.float64) / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def nearest_index(times: np.ndarray, value: float) -> int:
    """Return the index of the nearest sorted timestamp."""
    index = int(np.searchsorted(times, value))
    index = min(max(index, 0), len(times) - 1)
    if index and abs(times[index - 1] - value) < abs(times[index] - value):
        index -= 1
    return index


def max_true_streak(values: np.ndarray) -> tuple[int, int | None]:
    """Return maximum true streak and the one-based end index of its first 5-run."""
    best = 0
    streak = 0
    first_five: int | None = None
    for index, value in enumerate(values):
        streak = streak + 1 if bool(value) else 0
        best = max(best, streak)
        if streak == 5 and first_five is None:
            first_five = index + 1
    return best, first_five


def read_weak_csv(path: Path) -> tuple[list[dict[str, str]], np.ndarray]:
    """Load the runtime weak-axis rows and their timestamps."""
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    required = {
        't', 'evalue0', 'evalue1', 'weak_eigen_x', 'weak_eigen_y',
        'weak_eigen_z', 'weak_horizontal_norm',
        'position_x', 'position_y', 'position_z',
        'velocity_x', 'velocity_y', 'velocity_z',
    }
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f'weak CSV lacks required columns: {path}')
    times = np.asarray([float(row['t']) for row in rows], dtype=np.float64)
    if np.any(np.diff(times) < 0.0):
        raise ValueError(f'weak CSV timestamps are not ordered: {path}')
    return rows, times


def read_odometry_csv(
        path: Path,
        ) -> tuple[np.ndarray, list[np.ndarray], np.ndarray]:
    """Load runtime body-to-world attitudes from the recorded odometry CSV."""
    with path.open(newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    required = {
        '%time',
        'field.transforms0.transform.rotation.x',
        'field.transforms0.transform.rotation.y',
        'field.transforms0.transform.rotation.z',
        'field.transforms0.transform.rotation.w',
        'field.transforms0.transform.translation.x',
        'field.transforms0.transform.translation.y',
        'field.transforms0.transform.translation.z',
    }
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f'odometry CSV lacks required columns: {path}')
    times = np.asarray([
        float(row['%time']) * 1.0e-9 for row in rows
    ], dtype=np.float64)
    if np.any(np.diff(times) < 0.0):
        raise ValueError(f'odometry timestamps are not ordered: {path}')
    rotations = [quaternion_xyzw_rotation([
        float(row[f'field.transforms0.transform.rotation.{axis}'])
        for axis in 'xyzw']) for row in rows]
    positions = np.asarray([[
        float(row[f'field.transforms0.transform.translation.{axis}'])
        for axis in 'xyz'] for row in rows], dtype=np.float64)
    return times, rotations, positions


def determine_weak_join_mode(
        weak_rows: list[dict[str, str]], weak_times: np.ndarray,
        odometry_positions: np.ndarray,
        ) -> tuple[str, int, float]:
    """Choose precise timestamp matching or an explicit scan-order fallback."""
    unique_times = len(np.unique(weak_times))
    if unique_times >= max(10, len(weak_times) // 2) and np.ptp(weak_times) > 1.0:
        return 'timestamp', 0, 0.0
    weak_positions = np.asarray([[
        float(row[f'position_{axis}']) for axis in 'xyz'
    ] for row in weak_rows], dtype=np.float64)
    max_offset = len(odometry_positions) - len(weak_positions)
    if max_offset < 0:
        raise ValueError('weak diagnostic has more rows than runtime odometry')
    sample_count = min(200, len(weak_positions))
    sample = np.linspace(0, sample_count - 1, sample_count, dtype=np.int64)
    candidates: list[tuple[float, int]] = []
    for offset in range(max_offset + 1):
        error = np.linalg.norm(
            weak_positions[sample] - odometry_positions[offset + sample], axis=1)
        candidates.append((float(np.median(error)), offset))
    error, offset = min(candidates)
    if error > 0.2:
        raise ValueError(
            f'weak diagnostic timestamp precision is unusable and position '
            f'alignment error is {error:.3f} m')
    return 'scan_order_position_aligned', offset, error


def estimate_world_gravity_axis(
        source: Path, imu_topic: str, odometry_times: np.ndarray,
        odometry_rotations: list[np.ndarray], window_sec: float,
        ) -> tuple[np.ndarray, int]:
    """Estimate the world gravity axis from the initial IMU window.

    The runtime convention stores gravity as the negative mean specific force.
    The sign is immaterial for a gravity-orthogonal projection, but retaining
    this convention makes the provenance explicit.
    """
    start = float(odometry_times[0])
    samples: list[np.ndarray] = []
    with AnyReader([source]) as reader:
        connections = [
            connection for connection in reader.connections
            if connection.topic == imu_topic
        ]
        if not connections:
            raise ValueError(f'IMU topic is absent: {imu_topic}')
        for connection, timestamp_ns, raw in reader.messages(connections=connections):
            timestamp = timestamp_ns * 1.0e-9
            if timestamp < start:
                continue
            if timestamp > start + window_sec:
                break
            message = reader.deserialize(raw, connection.msgtype)
            acceleration = np.array([
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
            ], dtype=np.float64)
            if not np.isfinite(acceleration).all():
                continue
            odometry_index = nearest_index(odometry_times, timestamp)
            samples.append(-(odometry_rotations[odometry_index] @ acceleration))
    if len(samples) < 10:
        raise ValueError('fewer than ten finite IMU samples in gravity window')
    gravity = np.median(np.asarray(samples), axis=0)
    norm = float(np.linalg.norm(gravity))
    if norm <= 1.0e-12:
        raise ValueError('initial IMU gravity estimate is degenerate')
    return gravity / norm, len(samples)


def load_visual_pairs(path: Path) -> list[dict[str, Any]]:
    """Load pair-level records emitted by the visual diagnostic."""
    document = json.loads(path.read_text(encoding='utf-8'))
    if document.get('accuracy_ground_truth_accessed') is not False:
        raise ValueError('visual pair report is not ground-truth-free')
    pairs = document.get('pairs')
    if not isinstance(pairs, list):
        raise ValueError('visual pair report has no pairs list')
    return pairs


def project_visual_pairs(
        pairs: list[dict[str, Any]], weak_rows: list[dict[str, str]],
        weak_times: np.ndarray, odometry_times: np.ndarray,
        odometry_rotations: list[np.ndarray], gravity_axis: np.ndarray,
        max_join_sec: float, eigen_ratio_max: float,
        horizontal_norm_min: float, min_projection_mps: float,
        strong_projection_mps: float, weak_join_mode: str,
        weak_odometry_offset: int,
        ) -> dict[str, Any]:
    """Join valid visual pairs and aggregate one value per runtime weak row."""
    valid_pairs = [
        (pair_index, pair) for pair_index, pair in enumerate(pairs)
        if bool(pair.get('valid')) and pair.get('direction_base') is not None
    ]
    per_scan: dict[int, list[dict[str, float | int]]] = {}
    matched_pairs = 0
    weak_eligible_pairs = 0
    well_conditioned_suppressed = 0
    join_errors: list[float] = []
    for pair_index, pair in valid_pairs:
        timestamp = float(pair['lidar_timestamp_sec'])
        odometry_index = nearest_index(odometry_times, timestamp)
        odometry_delta = abs(float(odometry_times[odometry_index] - timestamp))
        if weak_join_mode == 'timestamp':
            weak_index = nearest_index(weak_times, timestamp)
            weak_delta = abs(float(weak_times[weak_index] - timestamp))
        else:
            weak_index = odometry_index - weak_odometry_offset
            weak_delta = 0.0
            if weak_index < 0 or weak_index >= len(weak_rows):
                continue
        if weak_delta > max_join_sec or odometry_delta > max_join_sec:
            continue
        matched_pairs += 1
        join_errors.append(max(weak_delta, odometry_delta))
        row = weak_rows[weak_index]
        eigen_ratio = float(row['evalue0']) / max(float(row['evalue1']), 1.0e-12)
        horizontal_norm = float(row['weak_horizontal_norm'])
        weak = np.asarray([
            float(row[f'weak_eigen_{axis}']) for axis in 'xyz'
        ], dtype=np.float64)
        horizontal = weak - gravity_axis * float(gravity_axis @ weak)
        horizontal_length = float(np.linalg.norm(horizontal))
        eligible = (
            eigen_ratio < eigen_ratio_max and
            horizontal_norm >= horizontal_norm_min and
            horizontal_length > 1.0e-9)
        if not eligible:
            well_conditioned_suppressed += 1
            continue
        horizontal /= horizontal_length
        runtime_velocity = np.asarray([
            float(row[f'velocity_{axis}']) for axis in 'xyz'
        ], dtype=np.float64)
        if not np.isfinite(runtime_velocity).all():
            continue
        # Eigenvector signs are arbitrary.  Voxel's weak bridge orients its
        # horizontal weak direction to have a nonnegative dot product with
        # the current world velocity before storing the signed speed.  Apply
        # the same convention before producing a signed visual scalar.
        if float(runtime_velocity @ horizontal) < 0.0:
            horizontal = -horizontal
        alignment_mps = float(runtime_velocity @ horizontal)
        direction_base = np.asarray(pair['direction_base'], dtype=np.float64)
        direction_world = odometry_rotations[odometry_index] @ direction_base
        projection = float(pair['speed_mps']) * float(direction_world @ horizontal)
        per_scan.setdefault(weak_index, []).append({
            'pair_index': pair_index,
            'projection_mps': projection,
            'join_error_sec': max(weak_delta, odometry_delta),
            'odometry_timestamp_sec': float(odometry_times[odometry_index]),
            'inliers': int(pair.get('inliers', 0)),
            'tracks': int(pair.get('tracks', 0)),
            'residual_norm': float(pair.get('residual_norm', float('inf'))),
            'speed_mps': float(pair.get('speed_mps', 0.0)),
            'alignment_mps': alignment_mps,
        })
        weak_eligible_pairs += 1

    scan_projection = np.full(len(weak_rows), np.nan, dtype=np.float64)
    observations: list[dict[str, Any]] = []
    for index, records in sorted(per_scan.items()):
        values = np.asarray([
            float(record['projection_mps']) for record in records
        ], dtype=np.float64)
        scan_projection[index] = float(np.median(values))
        row = weak_rows[index]
        observations.append({
            'weak_row_index': index,
            'stamp_sec': float(np.median([
                float(record['odometry_timestamp_sec']) for record in records
            ])),
            'velocity_mps': float(scan_projection[index]),
            'pair_count': len(records),
            'pair_indices': [int(record['pair_index']) for record in records],
            'join_error_sec': float(max(
                float(record['join_error_sec']) for record in records)),
            'median_inliers': float(np.median([
                float(record['inliers']) for record in records
            ])),
            'median_tracks': float(np.median([
                float(record['tracks']) for record in records
            ])),
            'median_residual_norm': float(np.median([
                float(record['residual_norm']) for record in records
            ])),
            'median_pair_speed_mps': float(np.median([
                float(record['speed_mps']) for record in records
            ])),
            'eigen_ratio': float(row['evalue0']) / max(
                float(row['evalue1']), 1.0e-12),
            'weak_horizontal_norm': float(row['weak_horizontal_norm']),
            'weak_horizontal_length': float(np.linalg.norm(
                np.asarray([
                    float(row[f'weak_eigen_{axis}']) for axis in 'xyz'
                ], dtype=np.float64) - gravity_axis * float(
                    gravity_axis @ np.asarray([
                        float(row[f'weak_eigen_{axis}']) for axis in 'xyz'
                    ], dtype=np.float64)))),
            'weak_axis_velocity_alignment_mps': float(np.median([
                float(record['alignment_mps']) for record in records
            ])),
        })
    nonzero = np.isfinite(scan_projection) & (
        np.abs(scan_projection) >= min_projection_mps)
    strong = np.isfinite(scan_projection) & (
        np.abs(scan_projection) >= strong_projection_mps)
    nonzero_streak, nonzero_first_five = max_true_streak(nonzero)
    strong_streak, strong_first_five = max_true_streak(strong)
    absolute = np.abs(scan_projection[np.isfinite(scan_projection)])
    return {
        'schema_version': 1,
        'status': 'sensor_only_runtime_weak_axis_join',
        'accuracy_ground_truth_accessed': False,
        'weak_axis_definition': {
            'eigen_ratio_max': eigen_ratio_max,
            'horizontal_norm_min': horizontal_norm_min,
            'max_join_sec': max_join_sec,
            'join_mode': weak_join_mode,
            'weak_to_odometry_offset': weak_odometry_offset,
        },
        'projection_definition': {
            'min_nonzero_projection_mps': min_projection_mps,
            'strong_projection_mps': strong_projection_mps,
        },
        'counts': {
            'visual_pairs_valid': len(valid_pairs),
            'visual_pairs_joined': matched_pairs,
            'weak_eligible_pairs': weak_eligible_pairs,
            'well_conditioned_pairs_suppressed': well_conditioned_suppressed,
            'runtime_weak_rows_with_visual': int(np.isfinite(scan_projection).sum()),
            'runtime_weak_rows_nonzero': int(nonzero.sum()),
            'runtime_weak_rows_strong': int(strong.sum()),
        },
        'quality': {
            'gravity_axis_world': gravity_axis.tolist(),
            'join_max_sec': max(join_errors) if join_errors else None,
            'join_p95_sec': float(np.percentile(join_errors, 95))
            if join_errors else None,
            'projection_abs_p10_mps': float(np.percentile(absolute, 10))
            if len(absolute) else None,
            'projection_abs_median_mps': float(np.median(absolute))
            if len(absolute) else None,
            'projection_abs_p90_mps': float(np.percentile(absolute, 90))
            if len(absolute) else None,
            'max_nonzero_streak_scans': nonzero_streak,
            'first_nonzero_five_scan_index': nonzero_first_five,
            'max_strong_streak_scans': strong_streak,
            'first_strong_five_scan_index': strong_first_five,
        },
        'observations': observations,
        'decision': 'GO_WEAK_AXIS_PROJECTION' if (
            nonzero_streak >= 5 and strong_streak >= 5) else 'NO_GO_WEAK_AXIS_PROJECTION',
        'decision_checks': {
            'five_consecutive_nonzero_scans': nonzero_streak >= 5,
            'five_consecutive_strong_scans': strong_streak >= 5,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--visual-pairs', type=Path, required=True)
    parser.add_argument('--weak-csv', type=Path, required=True)
    parser.add_argument('--odometry-csv', type=Path, required=True)
    parser.add_argument('--imu-source', type=Path, required=True)
    parser.add_argument('--imu-topic', default='/imu/data')
    parser.add_argument('--gravity-window-sec', type=float, default=5.0)
    parser.add_argument('--max-join-sec', type=float, default=0.08)
    parser.add_argument('--eigen-ratio-max', type=float, default=0.2)
    parser.add_argument('--horizontal-norm-min', type=float, default=0.9)
    parser.add_argument('--min-projection-mps', type=float, default=0.1)
    parser.add_argument('--strong-projection-mps', type=float, default=3.0)
    parser.add_argument('--output', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    if args.gravity_window_sec <= 0.0 or args.max_join_sec <= 0.0:
        raise ValueError('gravity window and join bound must be positive')
    pairs = load_visual_pairs(args.visual_pairs)
    weak_rows, weak_times = read_weak_csv(args.weak_csv)
    odometry_times, odometry_rotations, odometry_positions = read_odometry_csv(
        args.odometry_csv)
    weak_join_mode, weak_odometry_offset, position_alignment_error = (
        determine_weak_join_mode(weak_rows, weak_times, odometry_positions))
    gravity_axis, imu_samples = estimate_world_gravity_axis(
        args.imu_source, args.imu_topic, odometry_times, odometry_rotations,
        args.gravity_window_sec)
    report = project_visual_pairs(
        pairs, weak_rows, weak_times, odometry_times, odometry_rotations,
        gravity_axis, args.max_join_sec, args.eigen_ratio_max,
        args.horizontal_norm_min, args.min_projection_mps,
        args.strong_projection_mps, weak_join_mode, weak_odometry_offset)
    report['inputs'] = {
        'visual_pairs': str(args.visual_pairs.resolve()),
        'weak_csv': str(args.weak_csv.resolve()),
        'odometry_csv': str(args.odometry_csv.resolve()),
        'imu_source': str(args.imu_source.resolve()),
        'imu_topic': args.imu_topic,
    }
    report['gravity_estimation'] = {
        'window_sec': args.gravity_window_sec,
        'imu_samples': imu_samples,
    }
    report['weak_join_alignment'] = {
        'mode': weak_join_mode,
        'weak_to_odometry_offset': weak_odometry_offset,
        'median_position_error_m': position_alignment_error,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
