#!/usr/bin/env python3
"""Verified multi-session map projects for lidar_slam map bundles."""

from __future__ import annotations

import csv
import io
import json
import math
import os
import shutil
import struct
import tempfile
from contextlib import redirect_stdout
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

import yaml

from map_edit import (
    MapEditError,
    PcdLayout,
    _binary_records,
    _bundle_artifact_path,
    _retile_full_map,
    _write_binary_pcd,
    _xyz_from_record,
    resolve_bundle_dir,
    sha256_directory,
    sha256_file,
)

from verify_autoware_map import MapVerifier

from verify_map_bundle import BundleVerifier


PROJECT_NAME = 'map_project.json'
RECEIPT_NAME = 'map_merge_receipt.json'
LOOP_HEADER = ['from', 'to', 'fitness', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw']
FORMAT_MAP = {
    ('F', 4): 'f', ('F', 8): 'd',
    ('U', 1): 'B', ('U', 2): 'H', ('U', 4): 'I',
    ('I', 1): 'b', ('I', 2): 'h', ('I', 4): 'i',
}


class MapMergeError(ValueError):
    """A multi-session project cannot be published safely."""


@dataclass(frozen=True)
class MergeOptions:
    """Stable controls for deterministic multi-session alignment."""

    merge_voxel_size_m: float = 0.20
    alignment_voxel_size_m: float = 0.50
    max_alignment_points: int = 12000
    icp_max_iterations: int = 50
    icp_trim_fraction: float = 0.50
    icp_yaw_samples: int = 72
    max_overlap_median_m: float = 1.0
    max_overlap_p90_m: float = 2.5
    min_overlap_within_1m: float = 0.15


@dataclass
class SessionData:
    """Validated source artifacts loaded for one project session."""

    index: int
    bundle_dir: Path
    manifest: dict[str, Any]
    layout: PcdLayout
    records: list[bytes]
    trajectory: list[list[float]]
    identity: dict[str, Any]


def merge_map_sessions(
    *,
    source_dirs: list[Path],
    output_dir: Path,
    options: MergeOptions = MergeOptions(),
    initial_transforms: dict[int, tuple[float, float, float, float]] | None = None,
    dry_run: bool = False,
) -> dict[str, Any]:
    """Align two or more map bundles and atomically publish one project."""
    _validate_options(options)
    if len(source_dirs) < 2:
        raise MapMergeError('merge requires an anchor map and at least one additional session')
    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists():
        raise MapMergeError(f'output already exists: {output_dir}')
    sessions = [_load_session(index, path) for index, path in enumerate(source_dirs)]
    _validate_compatibility(sessions)
    for session in sessions:
        try:
            output_dir.relative_to(session.bundle_dir)
        except ValueError:
            continue
        raise MapMergeError(
            'output directory must not be inside a source map bundle: '
            f'{session.bundle_dir}'
        )

    layouts = [_layout_signature(item.layout) for item in sessions]
    if any(layout != layouts[0] for layout in layouts[1:]):
        raise MapMergeError(
            'source PCD layouts differ; merge requires identical fields, sizes, types, '
            'and counts so intensity and other attributes remain meaningful'
        )

    transformed_records: list[list[bytes]] = [list(sessions[0].records)]
    transformed_trajectories: list[list[list[float]]] = [sessions[0].trajectory]
    anchor_points = _sample_records(
        sessions[0].layout,
        transformed_records[0],
        options.max_alignment_points,
    )
    anchor_points = _voxel_downsample(
        anchor_points,
        options.alignment_voxel_size_m,
        options.max_alignment_points,
    )
    alignments = [_identity_alignment(sessions[0])]
    supplied = initial_transforms or {}

    for session in sessions[1:]:
        source_points = _sample_records(
            session.layout,
            session.records,
            options.max_alignment_points,
        )
        source_points = _voxel_downsample(
            source_points,
            options.alignment_voxel_size_m,
            options.max_alignment_points,
        )
        initial = supplied.get(session.index)
        alignment = _estimate_alignment(
            source_points,
            anchor_points,
            options=options,
            initial_transform=initial,
        )
        metrics = _partial_overlap_metrics(
            _transform_points(
                source_points,
                np.asarray(alignment['rotation'], dtype=np.float64),
                np.asarray(alignment['translation'], dtype=np.float64),
            ),
            anchor_points,
            trim_fraction=options.icp_trim_fraction,
        )
        checks = _alignment_checks(metrics, options)
        if any(item['status'] != 'PASS' for item in checks):
            details = '; '.join(item['message'] for item in checks if item['status'] != 'PASS')
            raise MapMergeError(
                f'session {session.index} alignment failed safety gates: {details}. '
                'Use overlapping map sessions or provide an initial transform.'
            )
        rotation = np.asarray(alignment['rotation'], dtype=np.float64)
        translation = np.asarray(alignment['translation'], dtype=np.float64)
        records = _transform_records(session.layout, session.records, rotation, translation)
        trajectory = _transform_trajectory(session.trajectory, rotation, translation)
        transformed_records.append(records)
        transformed_trajectories.append(trajectory)
        aligned_sample = _transform_points(source_points, rotation, translation)
        anchor_points = _voxel_downsample(
            np.vstack([anchor_points, aligned_sample]),
            options.alignment_voxel_size_m,
            options.max_alignment_points,
        )
        alignments.append({
            'session_index': session.index,
            'source_dir': str(session.bundle_dir),
            'method': 'trimmed_yaw_icp',
            'initial_transform_supplied': initial is not None,
            'rotation': alignment['rotation'],
            'translation': alignment['translation'],
            'yaw_deg': _yaw_deg(rotation),
            'translation_norm_m': float(np.linalg.norm(translation)),
            'iterations': alignment['iterations'],
            'converged': alignment['converged'],
            'metrics': metrics,
            'checks': checks,
        })

    all_records = [record for records in transformed_records for record in records]
    merged_records = _deduplicate_records(
        sessions[0].layout,
        transformed_records,
        options.merge_voxel_size_m,
    )
    project = _project_payload(sessions, alignments, options)
    summary = {
        'schema_version': 1,
        'status': 'DRY_RUN' if dry_run else 'PASS',
        'output_dir': str(output_dir),
        'session_count': len(sessions),
        'source_points': len(all_records),
        'merged_points': len(merged_records),
        'duplicate_points_removed': len(all_records) - len(merged_records),
        'alignments': alignments,
        'sources': [item.identity for item in sessions],
        'options': _options_payload(options),
    }
    if dry_run:
        return summary

    output_dir.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(
        prefix=f'.{output_dir.name}.merging-',
        dir=output_dir.parent,
    ) as temporary:
        staging = Path(temporary) / 'candidate'
        staging.mkdir()
        _write_candidate(
            staging=staging,
            sessions=sessions,
            transformed_trajectories=transformed_trajectories,
            merged_records=merged_records,
            project=project,
            options=options,
        )
        validation = _validate_candidate(staging)
        receipt = {
            **summary,
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'PASS',
            'project_sha256': sha256_file(staging / PROJECT_NAME),
            'candidate_map_bundle_sha256': sha256_file(staging / 'map_bundle.yaml'),
            'candidate_full_map_sha256': sha256_file(staging / 'map.pcd'),
            'candidate_pointcloud_map_sha256': sha256_directory(
                staging / 'pointcloud_map'
            ),
            'validation': validation,
        }
        (staging / RECEIPT_NAME).write_text(
            json.dumps(receipt, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        os.replace(staging, output_dir)
    return receipt


def _validate_options(options: MergeOptions) -> None:
    positive = {
        'merge_voxel_size_m': options.merge_voxel_size_m,
        'alignment_voxel_size_m': options.alignment_voxel_size_m,
        'max_alignment_points': options.max_alignment_points,
        'icp_max_iterations': options.icp_max_iterations,
        'icp_yaw_samples': options.icp_yaw_samples,
        'max_overlap_median_m': options.max_overlap_median_m,
        'max_overlap_p90_m': options.max_overlap_p90_m,
        'min_overlap_within_1m': options.min_overlap_within_1m,
    }
    invalid = [name for name, value in positive.items() if not math.isfinite(value) or value <= 0]
    if invalid:
        raise MapMergeError('merge options must be finite and positive: ' + ', '.join(invalid))
    if not 0.05 <= options.icp_trim_fraction <= 1.0:
        raise MapMergeError('icp_trim_fraction must be between 0.05 and 1.0')
    if options.min_overlap_within_1m > 1.0:
        raise MapMergeError('min_overlap_within_1m must not exceed 1.0')


def _load_session(index: int, source_dir: Path) -> SessionData:
    bundle_dir = resolve_bundle_dir(source_dir)
    bundle_verifier = BundleVerifier(bundle_dir)
    if not bundle_verifier.run():
        raise MapMergeError(
            f'session {index} map bundle is invalid: ' + '; '.join(bundle_verifier.failures)
        )
    map_verifier = MapVerifier(
        _bundle_artifact_path(bundle_dir, 'pointcloud_map'),
        check_bounds=True,
        verbose=False,
    )
    with redirect_stdout(io.StringIO()):
        map_ok = map_verifier.run()
    if not map_ok:
        details = '; '.join(map_verifier.failures)
        raise MapMergeError(
            f'session {index} Autoware pointcloud map is invalid: {details}'
        )
    manifest = yaml.safe_load((bundle_dir / 'map_bundle.yaml').read_text(encoding='utf-8'))
    full_map = _bundle_artifact_path(bundle_dir, 'full_map')
    layout, records = _binary_records(full_map)
    trajectory_path = _bundle_artifact_path(bundle_dir, 'trajectory')
    trajectory = _load_trajectory(trajectory_path)
    projector = _bundle_artifact_path(bundle_dir, 'projector_info')
    identity = {
        'session_index': index,
        'source_dir': str(bundle_dir),
        'map_bundle_sha256': sha256_file(bundle_dir / 'map_bundle.yaml'),
        'full_map_sha256': sha256_file(full_map),
        'pointcloud_map_sha256': sha256_directory(
            _bundle_artifact_path(bundle_dir, 'pointcloud_map')
        ),
        'trajectory_sha256': sha256_file(trajectory_path),
        'projector_info_sha256': sha256_file(projector),
        'point_count': len(records),
        'trajectory_pose_count': len(trajectory),
    }
    return SessionData(index, bundle_dir, manifest, layout, records, trajectory, identity)


def _validate_compatibility(sessions: list[SessionData]) -> None:
    anchor = sessions[0]
    anchor_frame = anchor.manifest.get('frame_id')
    anchor_projector = _projector_document(anchor.bundle_dir)
    anchor_metadata = _pointcloud_metadata(anchor.bundle_dir)
    for session in sessions[1:]:
        if session.manifest.get('frame_id') != anchor_frame:
            raise MapMergeError(
                f'session {session.index} frame_id differs from anchor: '
                f'{session.manifest.get("frame_id")!r} != {anchor_frame!r}'
            )
        if _projector_document(session.bundle_dir) != anchor_projector:
            raise MapMergeError(
                f'session {session.index} map_projector_info.yaml differs from anchor; '
                'georeferenced maps must use the same projector and origin'
            )
        metadata = _pointcloud_metadata(session.bundle_dir)
        resolution = (float(metadata['x_resolution']), float(metadata['y_resolution']))
        anchor_resolution = (
            float(anchor_metadata['x_resolution']),
            float(anchor_metadata['y_resolution']),
        )
        if resolution != anchor_resolution:
            raise MapMergeError(
                f'session {session.index} tile resolution differs from anchor'
            )


def _projector_document(bundle_dir: Path) -> Any:
    path = _bundle_artifact_path(bundle_dir, 'projector_info')
    try:
        return yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise MapMergeError(f'cannot read projector info {path}: {exc}') from exc


def _pointcloud_metadata(bundle_dir: Path) -> dict[str, Any]:
    path = _bundle_artifact_path(bundle_dir, 'pointcloud_map') / 'pointcloud_map_metadata.yaml'
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise MapMergeError(f'cannot read pointcloud metadata {path}: {exc}') from exc
    if not isinstance(document, dict):
        raise MapMergeError(f'pointcloud metadata must contain a mapping: {path}')
    return document


def _layout_signature(layout: PcdLayout) -> tuple[Any, ...]:
    return (
        tuple(layout.fields), tuple(layout.sizes), tuple(layout.types),
        tuple(layout.counts), layout.row_size,
    )


def _load_trajectory(path: Path) -> list[list[float]]:
    rows = []
    for number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), start=1):
        if not line.strip():
            continue
        try:
            row = [float(value) for value in line.split()]
        except ValueError as exc:
            raise MapMergeError(f'invalid TUM trajectory row {number}: {path}') from exc
        if len(row) != 8 or not all(math.isfinite(value) for value in row):
            raise MapMergeError(f'invalid TUM trajectory row {number}: {path}')
        rows.append(row)
    if not rows:
        raise MapMergeError(f'trajectory is empty: {path}')
    return rows


def _sample_records(layout: PcdLayout, records: list[bytes], maximum: int) -> np.ndarray:
    if not records:
        return np.empty((0, 3), dtype=np.float64)
    count = min(len(records), max(1, int(maximum)))
    indices = np.linspace(0, len(records) - 1, count, dtype=np.int64)
    return np.asarray([_xyz_from_record(layout, records[index]) for index in indices])


def _voxel_downsample(points: np.ndarray, voxel_size: float, maximum: int) -> np.ndarray:
    points = _finite_points(points)
    if len(points) == 0:
        return points
    keys = np.floor(points / float(voxel_size)).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    sampled = points[np.sort(indices)]
    if len(sampled) <= maximum:
        return sampled
    selection = np.linspace(0, len(sampled) - 1, maximum, dtype=np.int64)
    return sampled[selection]


def _estimate_alignment(
    source: np.ndarray,
    target: np.ndarray,
    *,
    options: MergeOptions,
    initial_transform: tuple[float, float, float, float] | None,
) -> dict[str, Any]:
    source = _finite_points(source)
    target = _finite_points(target)
    if len(source) < 3 or len(target) < 3:
        raise MapMergeError('alignment requires at least three finite points per session')
    if initial_transform is not None:
        tx, ty, tz, yaw_deg = initial_transform
        rotation = _yaw_rotation(math.radians(yaw_deg))
        translation = np.asarray([tx, ty, tz], dtype=np.float64)
    else:
        rotation, translation = _best_initialization(
            source,
            target,
            yaw_samples=options.icp_yaw_samples,
            trim_fraction=options.icp_trim_fraction,
        )
    history = []
    converged = False
    keep_count = max(3, int(math.ceil(len(source) * options.icp_trim_fraction)))
    keep_count = min(keep_count, len(source))
    for iteration in range(options.icp_max_iterations):
        transformed = _transform_points(source, rotation, translation)
        distances, indices = _nearest_distances_and_indices(transformed, target)
        keep = np.argsort(distances)[:keep_count]
        delta_rotation, delta_translation = _rigid_yaw_transform(
            transformed[keep], target[indices[keep]]
        )
        rotation = delta_rotation @ rotation
        translation = delta_rotation @ translation + delta_translation
        move = float(np.linalg.norm(delta_translation))
        angle = _rotation_angle_deg(delta_rotation)
        history.append({
            'iteration': iteration,
            'matches': int(keep_count),
            'median_nn_m': float(np.median(distances[keep])),
            'delta_translation_m': move,
            'delta_rotation_deg': angle,
        })
        if move <= 1e-3 and angle <= 0.05:
            converged = True
            break
    return {
        'rotation': rotation.tolist(),
        'translation': translation.tolist(),
        'iterations': len(history),
        'converged': converged,
        'history_tail': history[-10:],
    }


def _best_initialization(
    source: np.ndarray,
    target: np.ndarray,
    *,
    yaw_samples: int,
    trim_fraction: float,
) -> tuple[np.ndarray, np.ndarray]:
    source_centroid = np.median(source, axis=0)
    target_centroid = np.median(target, axis=0)
    candidates = [(np.eye(3), np.zeros(3))]
    for yaw in np.linspace(-math.pi, math.pi, max(1, yaw_samples), endpoint=False):
        rotation = _yaw_rotation(float(yaw))
        translation = target_centroid - rotation @ source_centroid
        candidates.append((rotation, translation))
    probe_count = min(len(source), 4000)
    probe = source[np.linspace(0, len(source) - 1, probe_count, dtype=np.int64)]
    keep_count = max(3, min(probe_count, int(math.ceil(probe_count * trim_fraction))))
    best = candidates[0]
    best_score = math.inf
    for rotation, translation in candidates:
        distances, _ = _nearest_distances_and_indices(
            _transform_points(probe, rotation, translation), target
        )
        score = float(np.mean(np.sort(distances)[:keep_count]))
        if score < best_score:
            best = (rotation, translation)
            best_score = score
    return best


def _partial_overlap_metrics(
    source: np.ndarray,
    target: np.ndarray,
    *,
    trim_fraction: float,
) -> dict[str, Any]:
    source_distances, _ = _nearest_distances_and_indices(source, target)
    target_distances, _ = _nearest_distances_and_indices(target, source)
    source_overlap = int(max(3, math.ceil(len(source_distances) * trim_fraction)))
    target_overlap = int(max(3, math.ceil(len(target_distances) * trim_fraction)))
    trimmed = np.concatenate([
        np.sort(source_distances)[:source_overlap],
        np.sort(target_distances)[:target_overlap],
    ])
    return {
        'trim_fraction': trim_fraction,
        'trimmed_median_nn_m': float(np.median(trimmed)),
        'trimmed_p90_nn_m': float(np.percentile(trimmed, 90)),
        'source_coverage_within_1m': float(np.mean(source_distances <= 1.0)),
        'target_coverage_within_1m': float(np.mean(target_distances <= 1.0)),
        'overlap_within_1m': float(max(
            np.mean(source_distances <= 1.0),
            np.mean(target_distances <= 1.0),
        )),
    }


def _alignment_checks(metrics: dict[str, Any], options: MergeOptions) -> list[dict[str, str]]:
    values = (
        (
            'trimmed_median_nn', metrics['trimmed_median_nn_m'],
            options.max_overlap_median_m, '<=', 'm',
        ),
        (
            'trimmed_p90_nn', metrics['trimmed_p90_nn_m'],
            options.max_overlap_p90_m, '<=', 'm',
        ),
        (
            'overlap_within_1m', metrics['overlap_within_1m'],
            options.min_overlap_within_1m, '>=', '',
        ),
    )
    checks = []
    for name, observed, threshold, operator, unit in values:
        passed = observed <= threshold if operator == '<=' else observed >= threshold
        checks.append({
            'id': name,
            'status': 'PASS' if passed else 'FAIL',
            'message': (
                f'{name}={observed:.4f}{unit} requires '
                f'{operator}{threshold:.4f}{unit}'
            ),
        })
    return checks


def _nearest_distances_and_indices(
    source: np.ndarray,
    target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        return _nearest_chunked(source, target)
    distances, indices = cKDTree(target).query(source, k=1)
    return distances.astype(np.float64, copy=False), indices.astype(np.int64, copy=False)


def _nearest_chunked(
    source: np.ndarray,
    target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    distances = []
    indices = []
    for start in range(0, len(source), 256):
        delta = source[start:start + 256, None, :] - target[None, :, :]
        squared = np.sum(delta * delta, axis=2)
        selected = np.argmin(squared, axis=1)
        distances.append(np.sqrt(squared[np.arange(len(selected)), selected]))
        indices.append(selected)
    return np.concatenate(distances), np.concatenate(indices)


def _rigid_yaw_transform(
    source: np.ndarray,
    target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate yaw and XYZ translation while preserving the gravity axis."""
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    covariance = (
        (source[:, :2] - source_centroid[:2]).T
        @ (target[:, :2] - target_centroid[:2])
    )
    left, _, right = np.linalg.svd(covariance)
    rotation_xy = right.T @ left.T
    if np.linalg.det(rotation_xy) < 0:
        right[-1] *= -1.0
        rotation_xy = right.T @ left.T
    rotation = np.eye(3, dtype=np.float64)
    rotation[:2, :2] = rotation_xy
    translation = target_centroid - rotation @ source_centroid
    return rotation, translation


def _transform_points(
    points: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    return _finite_points(points) @ rotation.T + translation.reshape(1, 3)


def _finite_points(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return points[np.all(np.isfinite(points), axis=1)]


def _yaw_rotation(yaw: float) -> np.ndarray:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.asarray([
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def _yaw_deg(rotation: np.ndarray) -> float:
    return math.degrees(math.atan2(float(rotation[1, 0]), float(rotation[0, 0])))


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    value = max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) * 0.5))
    return math.degrees(math.acos(value))


def _field_offset(layout: PcdLayout, field: str) -> tuple[int, str]:
    offset = 0
    for name, size, data_type, count in zip(
        layout.fields, layout.sizes, layout.types, layout.counts
    ):
        if name == field:
            if count != 1 or (data_type, size) not in FORMAT_MAP:
                raise MapMergeError(f'PCD field {field!r} cannot be transformed safely')
            return offset, FORMAT_MAP[(data_type, size)]
        offset += size * count
    raise MapMergeError(f'PCD field is missing: {field}')


def _write_record_vector(
    layout: PcdLayout,
    record: bytearray,
    names: tuple[str, str, str],
    vector: np.ndarray,
) -> None:
    for name, value in zip(names, vector):
        offset, data_format = _field_offset(layout, name)
        try:
            struct.pack_into('<' + data_format, record, offset, float(value))
        except (struct.error, OverflowError) as exc:
            raise MapMergeError(f'transformed {name} cannot be represented in the PCD layout') from exc


def _transform_records(
    layout: PcdLayout,
    records: list[bytes],
    rotation: np.ndarray,
    translation: np.ndarray,
) -> list[bytes]:
    normal_names = ('normal_x', 'normal_y', 'normal_z')
    normal_count = sum(name in layout.fields for name in normal_names)
    if normal_count not in (0, 3):
        raise MapMergeError('PCD contains a partial normal vector; refusing unsafe transform')
    transformed = []
    for source in records:
        record = bytearray(source)
        xyz = np.asarray(_xyz_from_record(layout, source), dtype=np.float64)
        _write_record_vector(layout, record, ('x', 'y', 'z'), rotation @ xyz + translation)
        if normal_count == 3:
            normal = np.asarray([
                _read_record_scalar(layout, source, name) for name in normal_names
            ])
            _write_record_vector(layout, record, normal_names, rotation @ normal)
        transformed.append(bytes(record))
    return transformed


def _read_record_scalar(layout: PcdLayout, record: bytes, field: str) -> float:
    offset, data_format = _field_offset(layout, field)
    return float(struct.unpack_from('<' + data_format, record, offset)[0])


def _transform_trajectory(
    rows: list[list[float]],
    rotation: np.ndarray,
    translation: np.ndarray,
) -> list[list[float]]:
    result = []
    for stamp, x, y, z, qx, qy, qz, qw in rows:
        position = rotation @ np.asarray([x, y, z]) + translation
        orientation = rotation @ _quaternion_matrix(qx, qy, qz, qw)
        out_qx, out_qy, out_qz, out_qw = _matrix_quaternion(orientation)
        result.append([
            stamp, float(position[0]), float(position[1]), float(position[2]),
            out_qx, out_qy, out_qz, out_qw,
        ])
    return result


def _quaternion_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise MapMergeError('trajectory contains a zero-norm quaternion')
    x, y, z, w = qx / norm, qy / norm, qz / norm, qw / norm
    return np.asarray([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _matrix_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale
    quaternion = np.asarray([qx, qy, qz, qw], dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0:
        quaternion *= -1.0
    return tuple(float(value) for value in quaternion)


def _deduplicate_records(
    layout: PcdLayout,
    session_records: list[list[bytes]],
    voxel_size: float,
) -> list[bytes]:
    selected: dict[tuple[int, int, int], bytes] = {}
    for records in session_records:
        for record in records:
            xyz = _xyz_from_record(layout, record)
            key = tuple(math.floor(value / voxel_size) for value in xyz)
            selected.setdefault(key, record)
    return [selected[key] for key in sorted(selected)]


def _project_payload(
    sessions: list[SessionData],
    alignments: list[dict[str, Any]],
    options: MergeOptions,
) -> dict[str, Any]:
    return {
        'schema_version': 1,
        'project_type': 'gravity_aligned_multi_session_pointcloud_map',
        'anchor_session_index': 0,
        'session_count': len(sessions),
        'sources': [session.identity for session in sessions],
        'transforms_to_anchor': alignments,
        'options': _options_payload(options),
        'receipt': RECEIPT_NAME,
    }


def _options_payload(options: MergeOptions) -> dict[str, Any]:
    return {
        name: getattr(options, name)
        for name in options.__dataclass_fields__
    }


def _identity_alignment(session: SessionData) -> dict[str, Any]:
    return {
        'session_index': session.index,
        'source_dir': str(session.bundle_dir),
        'method': 'anchor_identity',
        'initial_transform_supplied': False,
        'rotation': np.eye(3).tolist(),
        'translation': [0.0, 0.0, 0.0],
        'yaw_deg': 0.0,
        'translation_norm_m': 0.0,
        'iterations': 0,
        'converged': True,
        'metrics': {},
        'checks': [],
    }


def _write_candidate(
    *,
    staging: Path,
    sessions: list[SessionData],
    transformed_trajectories: list[list[list[float]]],
    merged_records: list[bytes],
    project: dict[str, Any],
    options: MergeOptions,
) -> None:
    anchor = sessions[0]
    _write_binary_pcd(staging / 'map.pcd', anchor.layout, merged_records)
    metadata = _pointcloud_metadata(anchor.bundle_dir)
    pointcloud_dir = staging / 'pointcloud_map'
    pointcloud_dir.mkdir()
    (pointcloud_dir / 'pointcloud_map_metadata.yaml').write_text(
        yaml.safe_dump({
            'x_resolution': float(metadata['x_resolution']),
            'y_resolution': float(metadata['y_resolution']),
        }, sort_keys=False),
        encoding='utf-8',
    )
    preliminary_manifest = {
        'format_version': 1,
        'frame_id': anchor.manifest.get('frame_id', 'map'),
        'submap_count': sum(len(rows) for rows in transformed_trajectories),
        'loop_edge_count': 0,
        'artifacts': {
            'full_map': 'map.pcd',
            'pointcloud_map': 'pointcloud_map',
            'trajectory': 'trajectory_optimized.tum',
            'pose_graph': 'pose_graph.g2o',
            'loop_edges': 'loop_edges.csv',
            'projector_info': 'map_projector_info.yaml',
        },
    }
    (staging / 'map_bundle.yaml').write_text(
        yaml.safe_dump(preliminary_manifest, sort_keys=False), encoding='utf-8'
    )
    try:
        _retile_full_map(staging)
    except MapEditError as exc:
        raise MapMergeError(f'cannot retile merged map safely: {exc}') from exc
    shutil.copy2(
        _bundle_artifact_path(anchor.bundle_dir, 'projector_info'),
        staging / 'map_projector_info.yaml',
    )
    sessions_dir = staging / 'sessions'
    sessions_dir.mkdir()
    merged_trajectory = []
    for session, trajectory in zip(sessions, transformed_trajectories):
        name = f'{session.index:03d}_{_safe_name(session.bundle_dir.name)}'
        directory = sessions_dir / name
        directory.mkdir()
        _write_trajectory(directory / 'trajectory_transformed.tum', trajectory)
        (directory / 'source_identity.json').write_text(
            json.dumps(session.identity, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        merged_trajectory.extend(trajectory)
    _write_trajectory(staging / 'trajectory_optimized.tum', merged_trajectory)
    _write_pose_graph(staging / 'pose_graph.g2o', merged_trajectory)
    with (staging / 'loop_edges.csv').open('w', newline='', encoding='utf-8') as stream:
        csv.DictWriter(stream, fieldnames=LOOP_HEADER).writeheader()
    (staging / PROJECT_NAME).write_text(
        json.dumps(project, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    manifest = {
        'format_version': 1,
        'frame_id': anchor.manifest.get('frame_id', 'map'),
        'submap_count': len(merged_trajectory),
        'loop_edge_count': 0,
        'map_leaf_size_m': options.merge_voxel_size_m,
        'grid_size_x_m': float(metadata['x_resolution']),
        'grid_size_y_m': float(metadata['y_resolution']),
        'artifacts': {
            'full_map': 'map.pcd',
            'pointcloud_map': 'pointcloud_map',
            'trajectory': 'trajectory_optimized.tum',
            'pose_graph': 'pose_graph.g2o',
            'loop_edges': 'loop_edges.csv',
            'projector_info': 'map_projector_info.yaml',
        },
        'merge': {
            'schema_version': 1,
            'project': PROJECT_NAME,
            'receipt': RECEIPT_NAME,
            'session_count': len(sessions),
            'pose_graph_semantics': 'session_trajectories_without_cross_session_edges',
        },
    }
    (staging / 'map_bundle.yaml').write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8'
    )


def _safe_name(value: str) -> str:
    rendered = ''.join(character if character.isalnum() or character in '-_' else '_' for character in value)
    return rendered.strip('_') or 'session'


def _write_trajectory(path: Path, rows: list[list[float]]) -> None:
    path.write_text(
        ''.join(' '.join(f'{value:.17g}' for value in row) + '\n' for row in rows),
        encoding='utf-8',
    )


def _write_pose_graph(path: Path, rows: list[list[float]]) -> None:
    lines = [
        f"VERTEX_SE3:QUAT {index} {' '.join(f'{value:.17g}' for value in row[1:])}"
        for index, row in enumerate(rows)
    ]
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def _validate_candidate(staging: Path) -> dict[str, Any]:
    bundle = BundleVerifier(staging)
    bundle_ok = bundle.run()
    verifier = MapVerifier(staging / 'pointcloud_map', check_bounds=True, verbose=False)
    with redirect_stdout(io.StringIO()):
        map_ok = verifier.run()
    if not bundle_ok or not map_ok:
        details = '; '.join(bundle.failures + verifier.failures)
        raise MapMergeError(
            f'merged candidate validation failed: {details}'
        )
    return {
        'map_bundle': 'PASS',
        'autoware_pointcloud_map': 'PASS',
        'map_bundle_checks': bundle.passes,
        'autoware_pass_count': len(verifier.passes),
        'autoware_warning_count': len(verifier.warnings),
    }
