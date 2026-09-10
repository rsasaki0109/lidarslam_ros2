#!/usr/bin/env python3
"""Cloud alignment gate for reset-based public MID-360 segment maps."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from lidarslam_benchmark_tools.mid360_robot_loop_alignment_analyzer import (
    load_tum_trajectory,
    load_pointcloud_map_points,
    resolve_pointcloud_map_dir,
    resolve_trajectory_path,
)
from lidarslam_benchmark_tools.mid360_robot_public_loop_cloud_analyzer import cloud_overlap_metrics, voxel_downsample
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_JSON = 'mid360_robot_public_segment_map_cloud_alignment.json'
PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_MARKDOWN = 'mid360_robot_public_segment_map_cloud_alignment.md'
PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_PLY = 'mid360_robot_public_segment_map_cloud_alignment.ply'


@dataclass(frozen=True)
class SegmentMapCloudAlignmentOptions:
    """Options for comparing two reset-based public MID-360 segment maps."""

    start_run_dir: Path
    end_run_dir: Path
    output_dir: Path
    start_pointcloud_map_dir: Path | None = None
    end_pointcloud_map_dir: Path | None = None
    start_trajectory_path: Path | None = None
    end_trajectory_path: Path | None = None
    start_center_stamp: float | None = None
    end_center_stamp: float | None = None
    crop_radius_m: float = 0.0
    voxel_size_m: float = 0.5
    max_points_per_tile: int = 4000
    max_points_per_cloud: int = 30000
    icp_max_iterations: int = 40
    icp_trim_fraction: float = 0.70
    icp_yaw_samples: int = 36
    icp_convergence_translation_m: float = 1e-3
    icp_convergence_rotation_deg: float = 0.05
    pass_median_nn_m: float = 1.0
    pass_p90_nn_m: float = 2.5
    pass_coverage_within_1m: float = 0.35
    min_cloud_points: int = 200
    ply_max_points_per_cloud: int = 15000


class PublicSegmentMapCloudAlignmentAnalyzer:
    """Align and gate two independent reset segment maps."""

    def analyze(self, options: SegmentMapCloudAlignmentOptions) -> dict[str, Any]:
        """Run cloud alignment and write JSON, Markdown, and PLY artifacts."""
        start_run_dir = options.start_run_dir.expanduser().resolve()
        end_run_dir = options.end_run_dir.expanduser().resolve()
        output_dir = options.output_dir.expanduser().resolve()
        start_map_dir = resolve_pointcloud_map_dir(
            start_run_dir,
            options.start_pointcloud_map_dir,
        )
        end_map_dir = resolve_pointcloud_map_dir(
            end_run_dir,
            options.end_pointcloud_map_dir,
        )
        start_trajectory_path = _resolve_analysis_trajectory_path(
            start_run_dir,
            options.start_trajectory_path,
        )
        end_trajectory_path = _resolve_analysis_trajectory_path(
            end_run_dir,
            options.end_trajectory_path,
        )
        start_cloud = load_pointcloud_map_points(
            start_map_dir,
            max_points_per_tile=max(1, int(options.max_points_per_tile)),
            max_total_points=max(1, int(options.max_points_per_cloud)),
        )
        end_cloud = load_pointcloud_map_points(
            end_map_dir,
            max_points_per_tile=max(1, int(options.max_points_per_tile)),
            max_total_points=max(1, int(options.max_points_per_cloud)),
        )
        start_points_raw = _as_points(start_cloud.get('points') or [])
        end_points_raw = _as_points(end_cloud.get('points') or [])
        crop = _crop_payload(
            start_points_raw=start_points_raw,
            end_points_raw=end_points_raw,
            start_trajectory_path=start_trajectory_path,
            end_trajectory_path=end_trajectory_path,
            options=options,
        )
        start_points_for_analysis = crop['start_points']
        end_points_for_analysis = crop['end_points']
        start_points = voxel_downsample(
            start_points_for_analysis,
            options.voxel_size_m,
            max(1, int(options.max_points_per_cloud)),
        )
        end_points = voxel_downsample(
            end_points_for_analysis,
            options.voxel_size_m,
            max(1, int(options.max_points_per_cloud)),
        )
        initial_metrics = cloud_overlap_metrics(start_points, end_points)
        alignment = estimate_rigid_alignment(
            start_points,
            end_points,
            max_iterations=max(1, int(options.icp_max_iterations)),
            trim_fraction=max(0.05, min(1.0, float(options.icp_trim_fraction))),
            yaw_samples=max(1, int(options.icp_yaw_samples)),
            convergence_translation_m=max(0.0, float(options.icp_convergence_translation_m)),
            convergence_rotation_deg=max(0.0, float(options.icp_convergence_rotation_deg)),
        )
        aligned_start = transform_points(
            start_points,
            np.asarray(alignment['rotation'], dtype=np.float64),
            np.asarray(alignment['translation'], dtype=np.float64),
        )
        aligned_metrics = cloud_overlap_metrics(aligned_start, end_points)
        checks = _checks(
            start_points=start_points,
            end_points=end_points,
            alignment=alignment,
            metrics=aligned_metrics,
            options=options,
        )
        status = _overall_status(checks)
        report = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'start_run_dir': str(start_run_dir),
            'end_run_dir': str(end_run_dir),
            'output_dir': str(output_dir),
            'start_pointcloud_map_dir': str(start_map_dir),
            'end_pointcloud_map_dir': str(end_map_dir),
            'start_trajectory_path': str(start_trajectory_path) if start_trajectory_path else '',
            'end_trajectory_path': str(end_trajectory_path) if end_trajectory_path else '',
            'options': _options_payload(options),
            'crop': crop['summary'],
            'clouds': {
                'start': _cloud_summary(
                    start_cloud,
                    start_points_raw,
                    start_points_for_analysis,
                    start_points,
                ),
                'end': _cloud_summary(
                    end_cloud,
                    end_points_raw,
                    end_points_for_analysis,
                    end_points,
                ),
            },
            'initial_overlap': initial_metrics,
            'aligned_overlap': aligned_metrics,
            'transform_start_to_end': _transform_payload(alignment),
            'icp': _icp_payload(alignment),
            'checks': checks,
            'artifacts': {
                'json': str(output_dir / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_JSON),
                'markdown': str(output_dir / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_MARKDOWN),
                'ply': str(output_dir / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_PLY),
            },
            'next_actions': _next_actions(status),
        }
        write_segment_map_cloud_alignment_report(
            report=report,
            output_dir=output_dir,
            aligned_start=aligned_start,
            end_points=end_points,
            max_points_per_cloud=max(1, int(options.ply_max_points_per_cloud)),
        )
        return report


def estimate_rigid_alignment(
    source: np.ndarray,
    target: np.ndarray,
    *,
    max_iterations: int = 40,
    trim_fraction: float = 0.70,
    yaw_samples: int = 36,
    convergence_translation_m: float = 1e-3,
    convergence_rotation_deg: float = 0.05,
) -> dict[str, Any]:
    """Estimate the rigid transform that maps source points into target frame."""
    source = _finite_points(source)
    target = _finite_points(target)
    if len(source) < 3 or len(target) < 3:
        return _empty_alignment('not enough points')

    rotation, translation, initial = _best_centroid_yaw_initialization(
        source,
        target,
        yaw_samples=max(1, int(yaw_samples)),
    )
    history = []
    converged = False
    reason = 'max_iterations'
    min_matches = min(len(source), len(target), max(3, int(len(source) * trim_fraction)))

    for iteration in range(max(1, int(max_iterations))):
        transformed = transform_points(source, rotation, translation)
        distances, indices = nearest_distances_and_indices(transformed, target)
        order = np.argsort(distances)
        keep_count = max(3, min(len(order), min_matches))
        keep = order[:keep_count]
        matched_source = transformed[keep]
        matched_target = target[indices[keep]]
        delta_rotation, delta_translation = rigid_transform(matched_source, matched_target)
        next_rotation = delta_rotation @ rotation
        next_translation = delta_rotation @ translation + delta_translation
        delta_angle_deg = _rotation_angle_deg(delta_rotation)
        delta_move_m = float(np.linalg.norm(delta_translation))
        kept_distances = distances[keep]
        history.append({
            'iteration': iteration,
            'matches': int(keep_count),
            'median_nn_m': float(np.median(kept_distances)),
            'p90_nn_m': float(np.percentile(kept_distances, 90)),
            'delta_translation_m': delta_move_m,
            'delta_rotation_deg': delta_angle_deg,
        })
        rotation = next_rotation
        translation = next_translation
        if (
            delta_move_m <= float(convergence_translation_m)
            and delta_angle_deg <= float(convergence_rotation_deg)
        ):
            converged = True
            reason = 'converged'
            break

    final_distances, _ = nearest_distances_and_indices(transform_points(source, rotation, translation), target)
    return {
        'status': 'PASS',
        'reason': reason,
        'converged': converged,
        'rotation': rotation.tolist(),
        'translation': translation.tolist(),
        'yaw_deg': _yaw_deg(rotation),
        'initialization': initial,
        'iterations': len(history),
        'history': history[-10:],
        'final_median_nn_m': float(np.median(final_distances)),
        'final_p90_nn_m': float(np.percentile(final_distances, 90)),
    }


def rigid_transform(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return R, t such that source @ R.T + t approximates target."""
    if len(source) != len(target) or len(source) < 3:
        raise ValueError('rigid transform requires at least three paired points')
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    source_centered = source - source_centroid
    target_centered = target - target_centroid
    covariance = source_centered.T @ target_centered
    u_matrix, _, vh_matrix = np.linalg.svd(covariance)
    rotation = vh_matrix.T @ u_matrix.T
    if np.linalg.det(rotation) < 0:
        vh_matrix[-1, :] *= -1.0
        rotation = vh_matrix.T @ u_matrix.T
    translation = target_centroid - rotation @ source_centroid
    return rotation.astype(np.float64), translation.astype(np.float64)


def transform_points(points: np.ndarray, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """Apply a row-vector rigid transform to XYZ points."""
    points = _finite_points(points)
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.float64)
    return points @ rotation.T + translation.reshape(1, 3)


def nearest_distances_and_indices(
    source: np.ndarray,
    target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return nearest-neighbor distances and indices from source to target."""
    source = _finite_points(source)
    target = _finite_points(target)
    if len(source) == 0 or len(target) == 0:
        return np.full((len(source),), math.inf), np.zeros((len(source),), dtype=np.int64)
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        return _nearest_distances_and_indices_chunked(source, target)
    distances, indices = cKDTree(target).query(source, k=1)
    return distances.astype(np.float64, copy=False), indices.astype(np.int64, copy=False)


def render_segment_map_cloud_alignment_markdown(report: dict[str, Any]) -> str:
    """Render a segment-map cloud alignment report as Markdown."""
    clouds = report.get('clouds') or {}
    start = clouds.get('start') or {}
    end = clouds.get('end') or {}
    initial = report.get('initial_overlap') or {}
    aligned = report.get('aligned_overlap') or {}
    transform = report.get('transform_start_to_end') or {}
    crop = report.get('crop') or {}
    lines = [
        '# MID-360 Public Segment Map Cloud Alignment',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- crop_radius_m: `{crop.get('crop_radius_m', 0.0):.3f}`",
        f"- start_points: `{start.get('analysis_points', 0)}`",
        f"- end_points: `{end.get('analysis_points', 0)}`",
        f"- initial_median_nn_m: `{initial.get('symmetric_median_nn_m', 0.0):.3f}`",
        f"- aligned_median_nn_m: `{aligned.get('symmetric_median_nn_m', 0.0):.3f}`",
        f"- aligned_p90_nn_m: `{aligned.get('symmetric_p90_nn_m', 0.0):.3f}`",
        f"- aligned_coverage_within_1m: `{aligned.get('coverage_within_1m', 0.0):.3f}`",
        f"- start_to_end_translation_m: `{transform.get('translation_norm_m', 0.0):.3f}`",
        f"- start_to_end_yaw_deg: `{transform.get('yaw_deg', 0.0):.3f}`",
        '',
        '## Checks',
        '',
    ]
    for check in report.get('checks') or []:
        lines.append(f"- `{check.get('status', '')}` `{check.get('id', '')}`: {check.get('message', '')}")
    steps = report.get('next_actions') or []
    if steps:
        lines.extend(['', '## Next Actions', ''])
        lines.extend(f'- {step}' for step in steps)
    return '\n'.join(lines).rstrip()


def write_segment_map_cloud_alignment_report(
    *,
    report: dict[str, Any],
    output_dir: Path,
    aligned_start: np.ndarray,
    end_points: np.ndarray,
    max_points_per_cloud: int,
) -> dict[str, Path]:
    """Write JSON, Markdown, and colored PLY alignment artifacts."""
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_JSON
    markdown_path = output_dir / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_MARKDOWN
    ply_path = output_dir / PUBLIC_SEGMENT_MAP_CLOUD_ALIGNMENT_PLY
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(
        render_segment_map_cloud_alignment_markdown(report) + '\n',
        encoding='utf-8',
    )
    ply_path.write_text(
        _render_alignment_ply(
            _limit_rows(aligned_start, max_points_per_cloud),
            _limit_rows(end_points, max_points_per_cloud),
        ),
        encoding='utf-8',
    )
    return {'json': json_path, 'markdown': markdown_path, 'ply': ply_path}


def _best_centroid_yaw_initialization(
    source: np.ndarray,
    target: np.ndarray,
    *,
    yaw_samples: int,
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    best_rotation = np.eye(3, dtype=np.float64)
    best_translation = target_centroid - source_centroid
    best_yaw = 0.0
    best_median = math.inf
    yaws = [0.0] if yaw_samples <= 1 else np.linspace(-math.pi, math.pi, yaw_samples, endpoint=False)
    probe_source = _limit_rows(source, min(len(source), 4000))
    for yaw in yaws:
        rotation = _yaw_rotation(float(yaw))
        translation = target_centroid - rotation @ source_centroid
        transformed = transform_points(probe_source, rotation, translation)
        distances, _ = nearest_distances_and_indices(transformed, target)
        median = float(np.median(distances))
        if median < best_median:
            best_median = median
            best_yaw = float(yaw)
            best_rotation = rotation
            best_translation = translation
    return best_rotation, best_translation, {
        'method': 'centroid_yaw_grid',
        'yaw_samples': int(yaw_samples),
        'selected_yaw_deg': math.degrees(best_yaw),
        'median_nn_m': best_median,
    }


def _checks(
    *,
    start_points: np.ndarray,
    end_points: np.ndarray,
    alignment: dict[str, Any],
    metrics: dict[str, Any],
    options: SegmentMapCloudAlignmentOptions,
) -> list[dict[str, str]]:
    checks = []
    checks.append({
        'id': 'start_cloud_points',
        'status': 'PASS' if len(start_points) >= int(options.min_cloud_points) else 'FAIL',
        'message': f'points={len(start_points)} min={int(options.min_cloud_points)}',
    })
    checks.append({
        'id': 'end_cloud_points',
        'status': 'PASS' if len(end_points) >= int(options.min_cloud_points) else 'FAIL',
        'message': f'points={len(end_points)} min={int(options.min_cloud_points)}',
    })
    checks.append({
        'id': 'icp_alignment',
        'status': 'PASS' if alignment.get('status') == 'PASS' else 'FAIL',
        'message': f"reason={alignment.get('reason', '')} iterations={alignment.get('iterations', 0)}",
    })
    median = float(metrics.get('symmetric_median_nn_m', math.inf))
    p90 = float(metrics.get('symmetric_p90_nn_m', math.inf))
    coverage = float(metrics.get('coverage_within_1m', 0.0))
    checks.append({
        'id': 'median_overlap',
        'status': 'PASS' if median <= float(options.pass_median_nn_m) else 'FAIL',
        'message': f'median={median:.3f}m max={float(options.pass_median_nn_m):.3f}m',
    })
    checks.append({
        'id': 'p90_overlap',
        'status': 'PASS' if p90 <= float(options.pass_p90_nn_m) else 'FAIL',
        'message': f'p90={p90:.3f}m max={float(options.pass_p90_nn_m):.3f}m',
    })
    checks.append({
        'id': 'coverage_within_1m',
        'status': 'PASS' if coverage >= float(options.pass_coverage_within_1m) else 'FAIL',
        'message': f'coverage={coverage:.3f} min={float(options.pass_coverage_within_1m):.3f}',
    })
    return checks


def _overall_status(checks: list[dict[str, str]]) -> str:
    if all(check.get('status') == 'PASS' for check in checks):
        return 'PASS'
    if any(check.get('status') == 'PASS' for check in checks):
        return 'WARN'
    return 'FAIL'


def _cloud_summary(
    cloud: dict[str, Any],
    raw_points: np.ndarray,
    cropped_points: np.ndarray,
    analysis_points: np.ndarray,
) -> dict[str, Any]:
    return {
        'map_dir': cloud.get('map_dir', ''),
        'metadata_path': cloud.get('metadata_path', ''),
        'tile_count': int(cloud.get('tile_count', 0)),
        'raw_sampled_points': int(len(raw_points)),
        'cropped_points': int(len(cropped_points)),
        'analysis_points': int(len(analysis_points)),
        'unsupported_tiles': cloud.get('unsupported_tiles') or [],
    }


def _transform_payload(alignment: dict[str, Any]) -> dict[str, Any]:
    translation = np.asarray(alignment.get('translation') or [0.0, 0.0, 0.0], dtype=np.float64)
    return {
        'rotation': alignment.get('rotation') or np.eye(3).tolist(),
        'translation': translation.tolist(),
        'translation_norm_m': float(np.linalg.norm(translation)),
        'yaw_deg': float(alignment.get('yaw_deg', 0.0)),
    }


def _icp_payload(alignment: dict[str, Any]) -> dict[str, Any]:
    return {
        'status': alignment.get('status', ''),
        'reason': alignment.get('reason', ''),
        'converged': bool(alignment.get('converged', False)),
        'iterations': int(alignment.get('iterations', 0)),
        'initialization': alignment.get('initialization') or {},
        'final_median_nn_m': float(alignment.get('final_median_nn_m', math.inf)),
        'final_p90_nn_m': float(alignment.get('final_p90_nn_m', math.inf)),
        'history_tail': alignment.get('history') or [],
    }


def _options_payload(options: SegmentMapCloudAlignmentOptions) -> dict[str, Any]:
    payload = asdict(options)
    for key in (
        'start_run_dir',
        'end_run_dir',
        'output_dir',
        'start_pointcloud_map_dir',
        'end_pointcloud_map_dir',
        'start_trajectory_path',
        'end_trajectory_path',
    ):
        value = payload.get(key)
        payload[key] = str(value) if value is not None else None
    return payload


def _crop_payload(
    *,
    start_points_raw: np.ndarray,
    end_points_raw: np.ndarray,
    start_trajectory_path: Path | None,
    end_trajectory_path: Path | None,
    options: SegmentMapCloudAlignmentOptions,
) -> dict[str, Any]:
    radius = max(0.0, float(options.crop_radius_m))
    start_center = _trajectory_center(start_trajectory_path, options.start_center_stamp)
    end_center = _trajectory_center(end_trajectory_path, options.end_center_stamp)
    start_points = start_points_raw
    end_points = end_points_raw
    if radius > 0.0 and start_center is not None:
        start_points = _crop_radius(start_points_raw, start_center['xyz'], radius)
    if radius > 0.0 and end_center is not None:
        end_points = _crop_radius(end_points_raw, end_center['xyz'], radius)
    return {
        'start_points': start_points,
        'end_points': end_points,
        'summary': {
            'crop_radius_m': radius,
            'start_center': start_center or {},
            'end_center': end_center or {},
            'start_raw_points': int(len(start_points_raw)),
            'end_raw_points': int(len(end_points_raw)),
            'start_cropped_points': int(len(start_points)),
            'end_cropped_points': int(len(end_points)),
        },
    }


def _trajectory_center(path: Path | None, stamp: float | None) -> dict[str, Any] | None:
    if path is None or stamp is None:
        return None
    poses = load_tum_trajectory(path)
    if not poses:
        return None
    selected = min(poses, key=lambda pose: abs(float(pose.stamp) - float(stamp)))
    return {
        'trajectory_path': str(path),
        'requested_stamp': float(stamp),
        'matched_stamp': float(selected.stamp),
        'stamp_error_sec': abs(float(selected.stamp) - float(stamp)),
        'pose_index': int(selected.index),
        'xyz': [float(selected.x), float(selected.y), float(selected.z)],
    }


def _resolve_analysis_trajectory_path(run_dir: Path, explicit_path: Path | None) -> Path | None:
    if explicit_path is not None:
        return resolve_trajectory_path(run_dir, explicit_path)
    candidates = [
        path for path in run_dir.rglob('*_tum_*.txt')
        if path.is_file()
    ]
    if candidates:
        return max(candidates, key=_trajectory_line_count)
    return resolve_trajectory_path(run_dir, None)


def _trajectory_line_count(path: Path) -> int:
    try:
        return sum(1 for line in path.read_text(encoding='utf-8', errors='replace').splitlines() if line.strip())
    except Exception:
        return 0


def _crop_radius(points: np.ndarray, center_xyz: list[float], radius: float) -> np.ndarray:
    if len(points) == 0:
        return points
    center = np.asarray(center_xyz, dtype=np.float64).reshape(1, 3)
    distances = np.linalg.norm(points - center, axis=1)
    return points[distances <= float(radius)]


def _next_actions(status: str) -> list[str]:
    if status == 'PASS':
        return [
            'Use this report as the reset-segment loop drift gate before promoting public-data runs.',
            'Feed the aligned segment-map evidence into the production-candidate dashboard and bundle.',
        ]
    return [
        'Inspect the colored PLY to see whether the reset maps overlap after ICP.',
        'Tighten or loosen ICP voxel/trim settings only after confirming both segment maps have enough points.',
    ]


def _as_points(points: list[Any]) -> np.ndarray:
    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return _finite_points(np.asarray(points, dtype=np.float64).reshape(-1, 3))


def _finite_points(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64)
    if points.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    points = points.reshape(-1, 3)
    return points[np.all(np.isfinite(points), axis=1)]


def _empty_alignment(reason: str) -> dict[str, Any]:
    return {
        'status': 'FAIL',
        'reason': reason,
        'converged': False,
        'rotation': np.eye(3).tolist(),
        'translation': [0.0, 0.0, 0.0],
        'yaw_deg': 0.0,
        'initialization': {},
        'iterations': 0,
        'history': [],
        'final_median_nn_m': math.inf,
        'final_p90_nn_m': math.inf,
    }


def _nearest_distances_and_indices_chunked(
    source: np.ndarray,
    target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    distances = []
    indices = []
    for start in range(0, len(source), 512):
        chunk = source[start:start + 512]
        delta = chunk[:, None, :] - target[None, :, :]
        squared = np.sum(delta * delta, axis=2)
        chunk_indices = np.argmin(squared, axis=1)
        chunk_distances = np.sqrt(squared[np.arange(len(chunk)), chunk_indices])
        distances.append(chunk_distances)
        indices.append(chunk_indices)
    return np.concatenate(distances), np.concatenate(indices).astype(np.int64, copy=False)


def _yaw_rotation(yaw: float) -> np.ndarray:
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return np.array([
        [cos_yaw, -sin_yaw, 0.0],
        [sin_yaw, cos_yaw, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def _yaw_deg(rotation: np.ndarray) -> float:
    return math.degrees(math.atan2(float(rotation[1, 0]), float(rotation[0, 0])))


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    trace = float(np.trace(rotation))
    value = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return math.degrees(math.acos(value))


def _limit_rows(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    step = max(1, int(math.ceil(len(points) / float(max_points))))
    return points[::step][:max_points]


def _render_alignment_ply(aligned_start: np.ndarray, end_points: np.ndarray) -> str:
    rows = []
    for point in aligned_start:
        rows.append((point, (49, 130, 206)))
    for point in end_points:
        rows.append((point, (235, 150, 54)))
    lines = [
        'ply',
        'format ascii 1.0',
        f'element vertex {len(rows)}',
        'property float x',
        'property float y',
        'property float z',
        'property uchar red',
        'property uchar green',
        'property uchar blue',
        'end_header',
    ]
    for point, color in rows:
        red, green, blue = color
        lines.append(
            f'{float(point[0]):.6f} {float(point[1]):.6f} {float(point[2]):.6f} '
            f'{red} {green} {blue}'
        )
    return '\n'.join(lines) + '\n'
