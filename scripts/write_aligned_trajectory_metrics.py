#!/usr/bin/env python3

"""Write metrics.json from an aligned TUM-vs-TUM comparison."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import sys
from typing import Any

import numpy as np

from benchmark_provenance import bag_identity, file_identity, software_identity


def _load_tum(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    if not path.is_file():
        return rows
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        parts = line.strip().split()
        if len(parts) < 8:
            continue
        try:
            t, x, y, z, qx, qy, qz, qw = map(float, parts[:8])
        except ValueError:
            continue
        rows.append(
            {
                't': t,
                'x': x,
                'y': y,
                'z': z,
                'qx': qx,
                'qy': qy,
                'qz': qz,
                'qw': qw,
            },
        )
    rows.sort(key=lambda row: row['t'])
    return rows


def _path_length(rows: list[dict[str, float]]) -> float:
    return sum(
        math.dist((a['x'], a['y'], a['z']), (b['x'], b['y'], b['z']))
        for a, b in zip(rows, rows[1:])
    )


def _match_rows(
    ref_rows: list[dict[str, float]],
    est_rows: list[dict[str, float]],
    tolerance: float,
) -> list[tuple[dict[str, float], dict[str, float]]]:
    est_times = [row['t'] for row in est_rows]
    pairs: list[tuple[dict[str, float], dict[str, float]]] = []
    for ref in ref_rows:
        idx = 0
        while idx < len(est_times) and est_times[idx] < ref['t']:
            idx += 1
        candidates: list[dict[str, float]] = []
        if idx < len(est_rows):
            candidates.append(est_rows[idx])
        if idx > 0:
            candidates.append(est_rows[idx - 1])
        best = None
        best_dt = None
        for cand in candidates:
            dt = abs(cand['t'] - ref['t'])
            if best is None or best_dt is None or dt < best_dt:
                best = cand
                best_dt = dt
        if best is not None and best_dt is not None and best_dt <= tolerance:
            pairs.append((ref, best))
    return pairs


def _rigid_align(
    pairs: list[tuple[dict[str, float], dict[str, float]]],
) -> tuple[np.ndarray, np.ndarray]:
    ref = np.array([[a['x'], a['y'], a['z']] for a, _ in pairs], dtype=float)
    est = np.array([[b['x'], b['y'], b['z']] for _, b in pairs], dtype=float)
    ref_centroid = ref.mean(axis=0)
    est_centroid = est.mean(axis=0)
    cov = (est - est_centroid).T @ (ref - ref_centroid)
    u, _, vt = np.linalg.svd(cov)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T
    trans = ref_centroid - rot @ est_centroid
    return rot, trans


def _apply_alignment(
    rows: list[dict[str, float]],
    rot: np.ndarray,
    trans: np.ndarray,
) -> list[dict[str, float]]:
    out: list[dict[str, float]] = []
    for row in rows:
        xyz = np.array([row['x'], row['y'], row['z']], dtype=float)
        aligned = rot @ xyz + trans
        out.append(
            {
                **row,
                'x': float(aligned[0]),
                'y': float(aligned[1]),
                'z': float(aligned[2]),
            },
        )
    return out


def _match_with_tolerance(
    ref_rows: list[dict[str, float]],
    est_rows: list[dict[str, float]],
    match_tolerance: float | None,
) -> list[tuple[dict[str, float], dict[str, float]]]:
    # Default (match_tolerance is None): the historical 0.05 -> 0.15 s cascade,
    # right for a dense estimate sampled near the reference. A single explicit
    # tolerance is for sparse references (e.g. RTK-SLAM total-station
    # checkpoints, or HILTI-style submap-rate corrected trajectories) scored
    # against a downsampled trajectory, and to reproduce the dataset's own
    # max_dt; it suppresses the silent dropping of checkpoints that have no
    # estimate pose within 0.15 s (see docs/research/project-plan-archive.md Sec. 2.4 / 11.3: an
    # under-tolerant match silently drops sparse reference points and biases
    # the reported error).
    if match_tolerance is not None:
        return _match_rows(ref_rows, est_rows, tolerance=match_tolerance)
    pairs = _match_rows(ref_rows, est_rows, tolerance=0.05)
    if len(pairs) < 10:
        pairs = _match_rows(ref_rows, est_rows, tolerance=0.15)
    return pairs


def _match_diagnostics(
    ref_rows: list[dict[str, float]],
    pairs: list[tuple[dict[str, float], dict[str, float]]],
) -> dict[str, Any]:
    """Diagnostics that make silent-drop bias visible (docs/research/project-plan-archive.md Sec. 2.4 /
    11.3): how many reference points were actually paired, how many were
    rejected (no estimate pose within tolerance), and how stale the worst
    matched pair is in time.
    """
    matched_gaps = [abs(ref['t'] - est['t']) for ref, est in pairs]
    return {
        'total_ref_points': len(ref_rows),
        'rejected_ref_points': len(ref_rows) - len(pairs),
        'max_time_gap_s': max(matched_gaps) if matched_gaps else None,
    }


def _ape_metrics(
    ref_rows: list[dict[str, float]],
    est_rows: list[dict[str, float]],
    match_tolerance: float | None = None,
) -> dict[str, Any]:
    pairs = _match_with_tolerance(ref_rows, est_rows, match_tolerance)
    if len(pairs) < 3:
        raise RuntimeError('not enough matched poses for alignment')

    rot, trans = _rigid_align(pairs)
    est_aligned = _apply_alignment(est_rows, rot, trans)
    aligned_pairs = _match_with_tolerance(ref_rows, est_aligned, match_tolerance)
    if len(aligned_pairs) < 3:
        raise RuntimeError('not enough matched poses after alignment')

    errors = [
        math.dist((ref['x'], ref['y'], ref['z']), (est['x'], est['y'], est['z']))
        for ref, est in aligned_pairs
    ]
    mean = sum(errors) / len(errors)
    variance = sum((err - mean) ** 2 for err in errors) / len(errors)
    sorted_errors = sorted(errors)
    median = sorted_errors[len(sorted_errors) // 2]
    if len(sorted_errors) % 2 == 0:
        median = 0.5 * (
            sorted_errors[len(sorted_errors) // 2 - 1]
            + sorted_errors[len(sorted_errors) // 2]
        )

    diagnostics = _match_diagnostics(ref_rows, aligned_pairs)
    return {
        'alignment': 'se3_umeyama',
        'pairs': len(aligned_pairs),
        'rmse': math.sqrt(sum(err * err for err in errors) / len(errors)),
        'mean': mean,
        'median': median,
        'max': max(errors),
        'min': min(errors),
        'std': math.sqrt(variance),
        'path_length_est_m': _path_length(est_rows),
        'path_length_est_aligned_m': _path_length(est_aligned),
        'path_length_ref_m': _path_length(ref_rows),
        **diagnostics,
    }


def _bag_duration_seconds(metadata_path: Path) -> float | None:
    if not metadata_path.is_file():
        return None
    lines = metadata_path.read_text(encoding='utf-8', errors='replace').splitlines()
    in_duration = False
    for line in lines:
        stripped = line.strip()
        if stripped.startswith('duration:'):
            in_duration = True
            continue
        if in_duration and stripped.startswith('nanoseconds:'):
            try:
                nanoseconds = int(stripped.split(':', 1)[1].strip())
            except ValueError:
                return None
            return nanoseconds / 1e9
        if in_duration and stripped and not line.startswith(' '):
            break
    return None


def _read_pose_count(path: Path) -> int:
    if not path.is_file():
        return 0
    return sum(
        1
        for line in path.read_text(encoding='utf-8', errors='replace').splitlines()
        if line.strip() and not line.lstrip().startswith('#')
    )


def _extract_loop_info(log_path: Path) -> dict[str, Any]:
    if not log_path.is_file():
        return {}
    text = log_path.read_text(encoding='utf-8', errors='replace')
    distances = re.findall(
        r'Odom input:\s+\d+\s+submaps,\s+distance:\s+([0-9]+(?:\.[0-9]+)?)m',
        text,
    )
    attempted_loop_pairs: list[tuple[int, int]] = []
    accepted_loop_pairs: list[tuple[int, int]] = []
    pending_pair: tuple[int, int] | None = None
    pending_rejected = False
    for line in text.splitlines():
        match = re.search(r'id_loop_point 1:(\d+)\s+id_loop_point 2:(\d+)', line)
        if match:
            if pending_pair is not None and not pending_rejected:
                accepted_loop_pairs.append(pending_pair)
            pending_pair = (int(match.group(1)), int(match.group(2)))
            attempted_loop_pairs.append(pending_pair)
            pending_rejected = False
            continue
        if pending_pair is not None and 'loop edge skipped as redundant or lower quality' in line:
            pending_rejected = True
    if pending_pair is not None and not pending_rejected:
        accepted_loop_pairs.append(pending_pair)

    loop_info: dict[str, Any] = {
        'graph_log_path': str(log_path),
        'loop_count': len(accepted_loop_pairs),
        'loop_count_attempted': len(attempted_loop_pairs),
    }
    if distances:
        loop_info['max_loop_search_distance_m'] = max(float(item) for item in distances)
    if accepted_loop_pairs:
        last = accepted_loop_pairs[-1]
        loop_info['last_loop_edge'] = {
            'from_index': int(last[0]),
            'to_index': int(last[1]),
        }
    if attempted_loop_pairs:
        last_attempted = attempted_loop_pairs[-1]
        loop_info['last_loop_edge_attempted'] = {
            'from_index': int(last_attempted[0]),
            'to_index': int(last_attempted[1]),
        }
    return loop_info


def _infer_reference_kind(reference_source: str, explicit_kind: str) -> str:
    if explicit_kind:
        return explicit_kind
    lowered = reference_source.strip().lower()
    if 'gt' in lowered or 'ground_truth' in lowered:
        return 'ground_truth'
    if 'glim' in lowered or 'cross' in lowered:
        return 'cross_validation'
    return 'unknown'


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Write metrics.json from an aligned trajectory comparison.',
    )
    parser.add_argument('--out-dir', required=True, help='Run output directory')
    parser.add_argument('--bag', required=True, help='rosbag2 directory used for the run')
    parser.add_argument('--reference-tum', required=True, help='Reference TUM trajectory')
    parser.add_argument('--corrected-tum', required=True, help='Estimated corrected TUM trajectory')
    parser.add_argument('--raw-tum', default='', help='Optional raw TUM trajectory')
    parser.add_argument('--graph-log', default='', help='Optional graph_slam.log path')
    parser.add_argument(
        '--lidarslam-param',
        default='lidarslam/param/lidarslam.yaml',
        help='Parameter file label stored in metrics.json',
    )
    parser.add_argument(
        '--points-topic',
        default='/points_raw',
        help='LiDAR topic used for the run',
    )
    parser.add_argument(
        '--points-frame',
        default='',
        help='PointCloud2 frame id stored in metrics.json',
    )
    parser.add_argument(
        '--robot-frame',
        default='base_link',
        help='Robot frame id stored in metrics.json',
    )
    parser.add_argument(
        '--odom-frame',
        default='odom',
        help='Odom frame id stored in metrics.json',
    )
    parser.add_argument(
        '--global-frame',
        default='map',
        help='Global frame id stored in metrics.json',
    )
    parser.add_argument(
        '--reference-source',
        default='aligned_reference',
        help='Reference source label stored in metrics.json',
    )
    parser.add_argument(
        '--reference-kind',
        default='',
        help='Reference kind label, for example ground_truth or cross_validation',
    )
    parser.add_argument(
        '--match-tolerance',
        type=float,
        default=-1.0,
        help='Single timestamp-match tolerance in seconds for reference/estimate '
             'pairing. <= 0 keeps the default 0.05 -> 0.15 s cascade. Set a larger '
             'value (e.g. 2.0) for sparse references such as RTK-SLAM total-station '
             'checkpoints scored against a downsampled trajectory.',
    )
    parser.add_argument(
        '--reference-label',
        default='reference',
        help='Human-readable reference label stored in metrics.json',
    )
    parser.add_argument(
        '--glim-traj',
        default='',
        help='Optional reference trajectory path exposed to the HTML report',
    )
    parser.add_argument(
        '--wall-sec',
        type=float,
        default=None,
        help='Optional measured wall time for the run',
    )
    parser.add_argument(
        '--started-at',
        default='',
        help='Optional ISO-8601 start timestamp',
    )
    parser.add_argument(
        '--started-at-unix',
        type=int,
        default=None,
        help='Optional unix start timestamp',
    )
    parser.add_argument(
        '--metrics-out',
        default='',
        help='Output metrics path (default: <out-dir>/metrics.json)',
    )
    parser.add_argument(
        '--parameter-file',
        action='append',
        default=[],
        help='Effective parameter file to identify; repeat for multiple files',
    )
    parser.add_argument(
        '--runtime-artifact',
        action='append',
        default=[],
        metavar='LABEL=PATH',
        help='Runtime executable/library to identify; repeat for multiple artifacts',
    )
    parser.add_argument(
        '--benchmark-harness',
        default='',
        help='Benchmark wrapper/script to identify (default: this metrics writer)',
    )
    args = parser.parse_args()

    out_dir = Path(args.out_dir).expanduser().resolve()
    bag_path = Path(args.bag).expanduser().resolve()
    reference_tum = Path(args.reference_tum).expanduser().resolve()
    corrected_tum = Path(args.corrected_tum).expanduser().resolve()
    raw_tum = Path(args.raw_tum).expanduser().resolve() if args.raw_tum else None
    graph_log = Path(args.graph_log).expanduser().resolve() if args.graph_log else None
    lidarslam_param = Path(args.lidarslam_param).expanduser().resolve()
    glim_traj = (
        Path(args.glim_traj).expanduser().resolve()
        if args.glim_traj else reference_tum
    )
    metrics_path = (
        Path(args.metrics_out).expanduser().resolve()
        if args.metrics_out else out_dir / 'metrics.json'
    )
    parameter_files = [
        Path(path).expanduser().resolve()
        for path in (args.parameter_file or [args.lidarslam_param])
    ]
    runtime_artifacts: list[tuple[str, Path]] = []
    for value in args.runtime_artifact:
        label, separator, path = value.partition('=')
        if not separator or not label or not path:
            parser.error('--runtime-artifact must be LABEL=PATH')
        runtime_artifacts.append((label, Path(path).expanduser().resolve()))
    if not runtime_artifacts:
        parser.error('at least one --runtime-artifact LABEL=PATH is required')
    writer_path = Path(__file__).resolve()
    harness_path = (
        Path(args.benchmark_harness).expanduser().resolve()
        if args.benchmark_harness else writer_path
    )

    ref_rows = _load_tum(reference_tum)
    corrected_rows = _load_tum(corrected_tum)
    if not ref_rows:
        raise SystemExit(f'reference trajectory not found or empty: {reference_tum}')
    if not corrected_rows:
        raise SystemExit(f'corrected trajectory not found or empty: {corrected_tum}')

    match_tolerance = args.match_tolerance if args.match_tolerance > 0 else None
    corrected_ape = _ape_metrics(ref_rows, corrected_rows, match_tolerance)
    if corrected_ape.get('rejected_ref_points', 0) > 0:
        print(
            "warning: corrected trajectory match rejected "
            f"{corrected_ape['rejected_ref_points']}/{corrected_ape['total_ref_points']} "
            "reference point(s) with no estimate pose within tolerance "
            f"(max_time_gap_s={corrected_ape.get('max_time_gap_s')}); "
            "see 'rejected_ref_points' / 'max_time_gap_s' in metrics.json",
            file=sys.stderr,
        )
    raw_ape = None
    if raw_tum and raw_tum.is_file():
        raw_rows = _load_tum(raw_tum)
        if raw_rows:
            raw_ape = _ape_metrics(ref_rows, raw_rows, match_tolerance)
            if raw_ape.get('rejected_ref_points', 0) > 0:
                print(
                    "warning: raw trajectory match rejected "
                    f"{raw_ape['rejected_ref_points']}/{raw_ape['total_ref_points']} "
                    "reference point(s) with no estimate pose within tolerance "
                    f"(max_time_gap_s={raw_ape.get('max_time_gap_s')}); "
                    "see 'rejected_ref_points' / 'max_time_gap_s' in metrics.json",
                    file=sys.stderr,
                )

    bag_duration_sec = _bag_duration_seconds(bag_path / 'metadata.yaml')
    wall_sec = args.wall_sec
    rtf = None
    if wall_sec is not None and bag_duration_sec and bag_duration_sec > 0.0:
        rtf = wall_sec / bag_duration_sec

    reference_kind = _infer_reference_kind(args.reference_source, args.reference_kind)
    graph_metrics = _extract_loop_info(graph_log) if graph_log else {}
    metrics: dict[str, Any] = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/benchmark-metrics-v1.schema.json'
        ),
        'started_at': args.started_at or None,
        'started_at_unix': args.started_at_unix,
        'out_dir': str(out_dir),
        'bag_path': str(bag_path),
        'bag_duration_sec': bag_duration_sec,
        'points_topic': args.points_topic,
        'frames': {
            'global_frame_id': args.global_frame,
            'odom_frame_id': args.odom_frame,
            'robot_frame_id': args.robot_frame,
            'points_frame_id': args.points_frame,
        },
        'reference': {
            'source': args.reference_source,
            'kind': reference_kind,
            'label': args.reference_label,
            'tum_path': str(reference_tum),
            'topic': '',
            'meta_path': '',
            'source_bag': str(bag_path),
        },
        'lidarslam': {
            'success': True,
            'wall_sec': wall_sec,
            'rtf': rtf,
            'tum_path': str(corrected_tum),
            'tum_lines': _read_pose_count(corrected_tum),
            'log_path': str(graph_log) if graph_log and graph_log.is_file() else '',
            'param_path': str(lidarslam_param),
            'out_dir': str(out_dir),
        },
        'glim': {
            'available': glim_traj.is_file(),
            'success': glim_traj.is_file(),
            'reference_source': args.reference_source,
            'traj_path': str(glim_traj) if glim_traj.is_file() else '',
            'wall_sec': None,
            'rtf': None,
        },
        'graph_based_slam': {
            'corrected_path_available': True,
            **graph_metrics,
        },
        'evo': {
            'ape_log_path': '',
            'ape': {
                key: corrected_ape[key]
                for key in (
                    'alignment', 'pairs', 'rmse', 'mean', 'median', 'max', 'min', 'std',
                    'rejected_ref_points', 'total_ref_points', 'max_time_gap_s',
                )
            },
            'raw_ape_log_path': '',
            'raw_ape': (
                {
                    key: raw_ape[key]
                    for key in (
                        'alignment', 'pairs', 'rmse', 'mean', 'median', 'max', 'min', 'std',
                        'rejected_ref_points', 'total_ref_points', 'max_time_gap_s',
                    )
                }
                if raw_ape is not None else None
            ),
        },
        'cross_validation': {
            'matched_poses': corrected_ape['pairs'],
            'alignment': corrected_ape['alignment'],
            'reference_path_length_m': corrected_ape['path_length_ref_m'],
            'estimated_path_length_m': corrected_ape['path_length_est_m'],
            'estimated_aligned_path_length_m': corrected_ape['path_length_est_aligned_m'],
            'reference_label': args.reference_label,
            'rejected_ref_points': corrected_ape['rejected_ref_points'],
            'total_ref_points': corrected_ape['total_ref_points'],
            'max_time_gap_s': corrected_ape['max_time_gap_s'],
        },
        'provenance': {
            'input': {
                'bag': bag_identity(bag_path),
                'reference_trajectory': file_identity(reference_tum),
            },
            'software': software_identity(
                Path(__file__).resolve().parents[1],
                parameter_files=parameter_files,
                runtime_artifacts=runtime_artifacts,
                benchmark_harness=harness_path,
                metrics_writer=writer_path,
            ),
        },
    }
    if raw_tum and raw_tum.is_file():
        metrics['lidarslam']['raw_tum_path'] = str(raw_tum)
        metrics['lidarslam']['raw_tum_lines'] = _read_pose_count(raw_tum)

    metrics_path.parent.mkdir(parents=True, exist_ok=True)
    metrics_path.write_text(
        json.dumps(metrics, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    print(metrics_path)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
