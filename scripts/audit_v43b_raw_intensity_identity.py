#!/usr/bin/env python3
"""Audit exact raw LiDAR intensity and challenge every v42 geometry pass."""

from __future__ import annotations

import argparse
from collections import OrderedDict
import csv
import hashlib
import json
import math
from pathlib import Path
import resource
import time
from typing import Any, Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


class ContractError(ValueError):
    """Raised when an input violates the frozen v43b contract."""


class MemoryBudgetError(RuntimeError):
    """Raised before a frozen resource bound is exceeded."""


def canonical_json(payload: Any) -> str:
    return json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False)


def payload_sha256(payload: Any) -> str:
    return hashlib.sha256(canonical_json(payload).encode('utf-8')).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def current_rss_mib() -> float:
    status = Path('/proc/self/status')
    if status.is_file():
        for line in status.read_text(encoding='utf-8').splitlines():
            if line.startswith('VmRSS:'):
                return float(line.split()[1]) / 1024.0
    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


class MemoryGuard:
    def __init__(self, maximum_rss_mib: float) -> None:
        self.maximum_rss_mib = float(maximum_rss_mib)
        self.peak_rss_mib = current_rss_mib()

    def check(self, label: str) -> None:
        rss = current_rss_mib()
        self.peak_rss_mib = max(self.peak_rss_mib, rss)
        if rss > self.maximum_rss_mib:
            raise MemoryBudgetError(
                f'RSS {rss:.3f} MiB exceeds {self.maximum_rss_mib:.3f} MiB at {label}')


def load_contract(path: Path) -> tuple[dict[str, Any], str]:
    contract = json.loads(path.read_text(encoding='utf-8'))
    required = {
        'schema_version', 'contract_id', 'frames', 'raw_pointcloud',
        'intensity_health', 'extraction', 'anchor', 'submap', 'identity',
        'legacy_challenge', 'memory', 'decision',
    }
    missing = sorted(required - set(contract))
    if missing:
        raise ContractError(f'contract missing keys: {missing}')
    if contract['schema_version'] != 1:
        raise ContractError('unsupported contract schema_version')
    if contract['frames'] != {
            'world': 'map', 'body': 'base_link',
            'lidar_to_body_convention':
            'p_body=R_body_lidar*p_lidar+t_body_lidar'}:
        raise ContractError('v43b frame or extrinsic convention changed')
    raw = contract['raw_pointcloud']
    if raw['timestamp_source'] != 'header.stamp':
        raise ContractError('v43b requires PointCloud2 header timestamps')
    names = [item['name'] for item in raw['required_fields']]
    if names != [
            'x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring',
            'ambient', 'range']:
        raise ContractError('v43b canonical PointCloud2 schema changed')
    offsets = [int(value) for value in contract['identity']['sequence_offsets']]
    if offsets != [-6, -3, 0, 3, 6]:
        raise ContractError('v43b must challenge the exact v42 five-pair sequence')
    health = contract['intensity_health']
    if int(health['minimum_distinct_values']) > int(health['distinct_value_cap']):
        raise ContractError('distinct value cap is below its pass threshold')
    if not 0.0 <= float(contract['identity']['minimum_overlap_local_pearson']) <= 1.0:
        raise ContractError('Pearson threshold must be in [0, 1]')
    for field in ('maximum_message_bytes', 'maximum_selected_cloud_bytes'):
        if int(contract['memory'][field]) <= 0:
            raise ContractError(f'{field} must be positive')
    if float(contract['memory']['maximum_rss_mib']) <= 0.0:
        raise ContractError('maximum_rss_mib must be positive')
    return contract, sha256_file(path)


def _resolve_source_path(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def load_source_binding(path: Path, sequence_id: str,
                        verify_hashes: bool = True) -> tuple[dict[str, Any], str]:
    manifest = json.loads(path.read_text(encoding='utf-8'))
    if manifest.get('schema_version') != 1:
        raise ContractError('unsupported source manifest schema_version')
    matches = [item for item in manifest.get('sequences', [])
               if item.get('sequence_id') == sequence_id]
    if len(matches) != 1:
        raise ContractError(f'exactly one source binding required for {sequence_id!r}')
    source = dict(matches[0])
    path_fields = (
        'bag_path', 'voxel_dir', 'voxel_config_path', 'v42_report_path',
        'legacy_edges_path')
    for field in path_fields:
        source[field] = _resolve_source_path(source[field]).resolve()
    files = {
        'bag': (source['bag_path'], source['bag_sha256']),
        'voxel_config': (source['voxel_config_path'], source['voxel_config_sha256']),
        'v42_report': (source['v42_report_path'], source['v42_report_sha256']),
        'legacy_edges': (source['legacy_edges_path'], source['legacy_edges_sha256']),
        'state': (source['voxel_dir'] / 'alidarState.txt', source['state_sha256']),
    }
    for label, (bound_path, expected_sha) in files.items():
        if not bound_path.is_file():
            raise FileNotFoundError(bound_path)
        if verify_hashes:
            actual = sha256_file(bound_path)
            if actual != expected_sha:
                raise ContractError(
                    f'{label} SHA-256 {actual} differs from binding {expected_sha}')
    if source['bag_path'].stat().st_size != int(source['bag_size_bytes']):
        raise ContractError('raw bag size differs from source binding')
    source['source_set_id'] = manifest['source_set_id']
    source['source_manifest_sha256'] = sha256_file(path)
    return source, source['source_manifest_sha256']


def quaternion_rotation(values: Iterable[float]) -> np.ndarray:
    qx, qy, qz, qw = (float(value) for value in values)
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ContractError('pose has an invalid quaternion')
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def pose_matrix(pose: Iterable[float]) -> np.ndarray:
    x, y, z, qx, qy, qz, qw = (float(value) for value in pose)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = quaternion_rotation((qx, qy, qz, qw))
    result[:3, 3] = (x, y, z)
    return result


def load_states(path: Path) -> tuple[np.ndarray, np.ndarray]:
    stamps: list[float] = []
    transforms: list[np.ndarray] = []
    previous = -math.inf
    for line_number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
        if not line.strip():
            continue
        fields = line.split()
        if len(fields) < 8:
            raise ContractError(f'{path}:{line_number}: expected at least 8 fields')
        try:
            values = [float(value) for value in fields[:8]]
        except ValueError as error:
            raise ContractError(f'{path}:{line_number}: non-numeric pose') from error
        if not all(math.isfinite(value) for value in values):
            raise ContractError(f'{path}:{line_number}: non-finite pose')
        if values[0] <= previous:
            raise ContractError(f'{path}:{line_number}: timestamps are not increasing')
        stamps.append(values[0])
        transforms.append(pose_matrix(values[1:]))
        previous = values[0]
    if not stamps:
        raise ContractError(f'{path}: no states')
    return np.asarray(stamps, dtype=np.float64), np.stack(transforms)


def select_anchor_indices(transforms: np.ndarray,
                          minimum_translation_m: float) -> list[int]:
    if len(transforms) == 0:
        return []
    threshold = float(minimum_translation_m)
    if not math.isfinite(threshold) or threshold <= 0.0:
        raise ContractError('anchor translation threshold must be positive')
    selected = [0]
    for index in range(1, len(transforms)):
        distance = np.linalg.norm(
            transforms[index, :3, 3] - transforms[selected[-1], :3, 3])
        if distance >= threshold:
            selected.append(index)
    return selected


def load_extrinsic(source: dict[str, Any]) -> tuple[np.ndarray, dict[str, Any]]:
    try:
        import yaml
    except ModuleNotFoundError as error:
        raise ContractError('PyYAML is required to bind the v17 extrinsic') from error
    document = yaml.safe_load(source['voxel_config_path'].read_text(encoding='utf-8'))
    general = document.get('General', {})
    if general.get('lid_topic') != source['lidar_topic']:
        raise ContractError('v17 LiDAR topic differs from source binding')
    rotation = np.asarray(general.get('extrinsic_rota'), dtype=np.float64)
    translation = np.asarray(general.get('extrinsic_tran'), dtype=np.float64)
    if rotation.size != 9 or translation.shape != (3,):
        raise ContractError('v17 extrinsic must contain a 3x3 rotation and translation')
    rotation = rotation.reshape(3, 3)
    if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
        raise ContractError('v17 extrinsic is non-finite')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ContractError('v17 LiDAR-to-body rotation is not orthonormal')
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
        raise ContractError('v17 LiDAR-to-body rotation determinant is not one')
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform, {
        'convention': 'p_body=R_body_lidar*p_lidar+t_body_lidar',
        'rotation': rotation.tolist(),
        'translation_m': translation.tolist(),
    }


def load_legacy_edges(path: Path) -> list[tuple[int, int]]:
    rows = list(csv.DictReader(path.open(newline='', encoding='utf-8')))
    edges = []
    for row in rows:
        source, target = int(row['from']), int(row['to'])
        if source >= target:
            raise ContractError(f'{path}: legacy edge is not historical->query')
        edges.append((source, target))
    return sorted(set(edges))


def matching_legacy_edges(source: int, target: int,
                          legacy: list[tuple[int, int]], window: int
                          ) -> list[list[int]]:
    return [[old_source, old_target] for old_source, old_target in legacy
            if abs(source - old_source) <= window
            and abs(target - old_target) <= window]


def validate_se3(transform: np.ndarray) -> None:
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ContractError('v42 target-from-source transform is not finite 4x4')
    if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
        raise ContractError('v42 target-from-source homogeneous row is invalid')
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise ContractError('v42 target-from-source rotation is not orthonormal')
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
        raise ContractError('v42 target-from-source determinant is not one')


def load_v42_geometry_candidates(source: dict[str, Any], sequence_id: str,
                                 anchors: list[int], contract: dict[str, Any]
                                 ) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    report = json.loads(source['v42_report_path'].read_text(encoding='utf-8'))
    if report.get('status') != 'PASS' or report.get('sequence_id') != sequence_id:
        raise ContractError('v42 report is not a passing report for this sequence')
    deterministic = report.get('deterministic', {})
    if report.get('deterministic_payload_sha256') != payload_sha256(deterministic):
        raise ContractError('v42 deterministic payload hash is invalid')
    if deterministic.get('input', {}).get('state_sha256') != source['state_sha256']:
        raise ContractError('v42 state binding differs from v43b state binding')
    anchor = deterministic.get('anchor', {})
    if (int(anchor.get('count', -1)) != len(anchors)
            or anchor.get('state_indices_sha256') != payload_sha256(anchors)):
        raise ContractError('v43b anchor reconstruction differs from v42')
    candidate_section = deterministic.get('candidate', {})
    candidates = [item for item in candidate_section.get('records', [])
                  if bool(item.get('geometry', {}).get('passed'))]
    if len(candidates) != int(candidate_section.get(
            'geometry_pass_count_before_dedup', -1)):
        raise ContractError('v42 geometry-pass count is inconsistent')
    expected_offsets = [int(value) for value in contract['identity']['sequence_offsets']]
    for candidate in candidates:
        pairs = candidate.get('geometry', {}).get('pairs', [])
        if [int(pair.get('offset')) for pair in pairs] != expected_offsets:
            raise ContractError('v42 geometry candidate does not have the frozen offsets')
        for pair in pairs:
            source_anchor = int(pair['source_anchor'])
            target_anchor = int(pair['target_anchor'])
            if not (0 <= source_anchor < len(anchors)
                    and 0 <= target_anchor < len(anchors)):
                raise ContractError('v42 candidate anchor is out of range')
            validate_se3(np.asarray(pair['target_from_source_matrix'], dtype=np.float64))
    return report, candidates


def required_state_indices(candidates: list[dict[str, Any]], anchors: list[int],
                           anchor_radius: int, maximum_count: int) -> list[int]:
    required: set[int] = set()
    radius = int(anchor_radius)
    for candidate in candidates:
        for pair in candidate['geometry']['pairs']:
            for anchor_index in (int(pair['source_anchor']), int(pair['target_anchor'])):
                begin = max(0, anchor_index - radius)
                end = min(len(anchors), anchor_index + radius + 1)
                required.update(anchors[index] for index in range(begin, end))
    result = sorted(required)
    if len(result) > int(maximum_count):
        raise MemoryBudgetError(
            f'{len(result)} selected scans exceed fixed limit {maximum_count}')
    return result


class IntensityAccumulator:
    """Exact full-channel digest plus bounded health statistics."""

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.scan_count = 0
        self.point_count = 0
        self.finite_count = 0
        self.nonzero_count = 0
        self.varying_scan_count = 0
        self.minimum = math.inf
        self.maximum = -math.inf
        self.total = 0.0
        self.total_squared = 0.0
        self.distinct: set[int] = set()
        self.digest = hashlib.sha256()

    def add(self, intensity: np.ndarray) -> None:
        values = np.asarray(intensity, dtype='<f4')
        self.scan_count += 1
        self.point_count += int(len(values))
        self.digest.update(values.tobytes(order='C'))
        finite = values[np.isfinite(values)]
        self.finite_count += int(len(finite))
        if not len(finite):
            return
        minimum = float(np.min(finite))
        maximum = float(np.max(finite))
        self.minimum = min(self.minimum, minimum)
        self.maximum = max(self.maximum, maximum)
        self.nonzero_count += int(np.count_nonzero(
            np.abs(finite) > float(self.config['nonzero_epsilon'])))
        if maximum - minimum >= float(self.config['minimum_dynamic_range']):
            self.varying_scan_count += 1
        self.total += float(np.sum(finite, dtype=np.float64))
        as_double = finite.astype(np.float64)
        self.total_squared += float(np.dot(as_double, as_double))
        cap = int(self.config['distinct_value_cap'])
        if len(self.distinct) < cap:
            count = min(int(self.config['distinct_samples_per_scan']), len(finite))
            indices = np.linspace(0, len(finite) - 1, count, dtype=np.int64)
            for value in finite[indices].view(np.uint32):
                self.distinct.add(int(value))
                if len(self.distinct) >= cap:
                    break

    def result(self) -> dict[str, Any]:
        finite_fraction = self.finite_count / max(1, self.point_count)
        nonzero_fraction = self.nonzero_count / max(1, self.finite_count)
        varying_fraction = self.varying_scan_count / max(1, self.scan_count)
        dynamic_range = self.maximum - self.minimum if self.finite_count else 0.0
        mean = self.total / max(1, self.finite_count)
        variance = max(
            0.0, self.total_squared / max(1, self.finite_count) - mean*mean)
        checks = {
            'finite_fraction': finite_fraction >= float(
                self.config['minimum_finite_fraction']),
            'nonzero_fraction': nonzero_fraction >= float(
                self.config['minimum_nonzero_fraction']),
            'dynamic_range': dynamic_range >= float(
                self.config['minimum_dynamic_range']),
            'distinct_values': len(self.distinct) >= int(
                self.config['minimum_distinct_values']),
            'varying_scan_fraction': varying_fraction >= float(
                self.config['minimum_varying_scan_fraction']),
        }
        return {
            'available': all(checks.values()),
            'checks': checks,
            'scan_count': self.scan_count,
            'point_count': self.point_count,
            'finite_count': self.finite_count,
            'finite_fraction': finite_fraction,
            'nonzero_count': self.nonzero_count,
            'nonzero_fraction': nonzero_fraction,
            'minimum': self.minimum if self.finite_count else None,
            'maximum': self.maximum if self.finite_count else None,
            'dynamic_range': dynamic_range,
            'mean': mean if self.finite_count else None,
            'standard_deviation': math.sqrt(variance) if self.finite_count else None,
            'distinct_values_observed_up_to_cap': len(self.distinct),
            'varying_scan_count': self.varying_scan_count,
            'varying_scan_fraction': varying_fraction,
            'intensity_payload_sha256': self.digest.hexdigest(),
        }


def point_dtype(config: dict[str, Any]) -> np.dtype:
    formats = {2: 'u1', 4: '<u2', 6: '<u4', 7: '<f4'}
    fields = config['required_fields']
    return np.dtype({
        'names': [item['name'] for item in fields],
        'formats': [formats[int(item['datatype'])] for item in fields],
        'offsets': [int(item['offset']) for item in fields],
        'itemsize': int(config['point_step_bytes']),
    })


def pointcloud_view(message: Any, raw_config: dict[str, Any],
                    expected_frame: str, maximum_raw_points: int,
                    maximum_message_bytes: int) -> np.ndarray:
    if int(message.height) != int(raw_config['height']):
        raise ContractError('PointCloud2 height differs from contract')
    if bool(message.is_bigendian) == bool(raw_config['little_endian']):
        raise ContractError('PointCloud2 endianness differs from contract')
    if int(message.point_step) != int(raw_config['point_step_bytes']):
        raise ContractError('PointCloud2 point_step differs from contract')
    width = int(message.width)
    if width < 0 or width > int(maximum_raw_points):
        raise MemoryBudgetError('PointCloud2 width exceeds fixed scan limit')
    expected_bytes = width * int(message.point_step)
    if int(message.row_step) != expected_bytes or len(message.data) != expected_bytes:
        raise ContractError('PointCloud2 byte count or row_step is inconsistent')
    if expected_bytes > int(maximum_message_bytes):
        raise MemoryBudgetError('PointCloud2 payload exceeds maximum_message_bytes')
    actual_fields = [
        {'name': field.name, 'offset': int(field.offset),
         'datatype': int(field.datatype), 'count': int(field.count)}
        for field in message.fields]
    if actual_fields != raw_config['required_fields']:
        raise ContractError('PointCloud2 fields differ from exact canonical schema')
    if message.header.frame_id != expected_frame:
        raise ContractError(
            f'PointCloud2 frame {message.header.frame_id!r} differs from {expected_frame!r}')
    raw = np.asarray(message.data, dtype=np.uint8)
    if not raw.flags.c_contiguous:
        raise ContractError('PointCloud2 data is not contiguous')
    return np.ndarray(shape=(width,), dtype=point_dtype(raw_config), buffer=raw)


def header_stamp_ns(message: Any) -> int:
    return (int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec))


def nearest_required_index(required_stamps_ns: np.ndarray, stamp_ns: int
                           ) -> int | None:
    if not len(required_stamps_ns):
        return None
    position = int(np.searchsorted(required_stamps_ns, stamp_ns))
    candidates = [index for index in (position - 1, position)
                  if 0 <= index < len(required_stamps_ns)]
    return min(candidates,
               key=lambda index: (abs(int(required_stamps_ns[index]) - stamp_ns), index))


def transform_xyz(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def voxel_mean_xyzi(points: np.ndarray, voxel_size_m: float,
                    maximum_points: int) -> np.ndarray:
    points = np.asarray(points, dtype=np.float32)
    if not len(points):
        return np.empty((0, 4), dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 4 or not np.isfinite(points).all():
        raise ContractError('voxel input must be finite XYZI')
    keys = np.floor(points[:, :3] / float(voxel_size_m)).astype(np.int32)
    order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
    sorted_keys = keys[order]
    sorted_points = points[order].astype(np.float64)
    changes = np.flatnonzero(np.any(sorted_keys[1:] != sorted_keys[:-1], axis=1)) + 1
    starts = np.concatenate((np.array([0], dtype=np.int64), changes))
    ends = np.concatenate((changes, np.array([len(points)], dtype=np.int64)))
    sums = np.add.reduceat(sorted_points, starts, axis=0)
    means = sums / (ends - starts)[:, None]
    result = means.astype(np.float32)
    limit = int(maximum_points)
    if len(result) > limit:
        keep = np.linspace(0, len(result) - 1, limit, dtype=np.int64)
        result = result[keep]
    return np.ascontiguousarray(result)


def extract_selected_cloud(points: np.ndarray, lidar_to_body: np.ndarray,
                           config: dict[str, Any]) -> np.ndarray:
    xyzi = np.column_stack((
        points['x'], points['y'], points['z'], points['intensity'])).astype(
            np.float32, copy=False)
    finite = np.isfinite(xyzi).all(axis=1)
    body_xyz = transform_xyz(xyzi[:, :3].astype(np.float64), lidar_to_body)
    ranges = np.linalg.norm(body_xyz, axis=1)
    keep = finite & np.isfinite(body_xyz).all(axis=1)
    keep &= ranges >= float(config['minimum_range_m'])
    keep &= ranges <= float(config['maximum_range_m'])
    body = np.empty((int(np.count_nonzero(keep)), 4), dtype=np.float32)
    body[:, :3] = body_xyz[keep]
    body[:, 3] = xyzi[keep, 3]
    return voxel_mean_xyzi(
        body, float(config['voxel_size_m']),
        int(config['maximum_points_per_selected_scan']))


def stream_raw_bag(*, source: dict[str, Any], stamps: np.ndarray,
                   required_states: list[int], lidar_to_body: np.ndarray,
                   contract: dict[str, Any], memory: MemoryGuard
                   ) -> tuple[dict[str, Any], dict[int, np.ndarray]]:
    try:
        from rosbags.highlevel import AnyReader
    except ModuleNotFoundError as error:
        raise ContractError('rosbags is required for exact ROS1 raw audit') from error
    raw_config = contract['raw_pointcloud']
    extraction = contract['extraction']
    maximum_match_ns = int(raw_config['maximum_state_match_error_ns'])
    required_stamps = np.rint(stamps[required_states] * 1e9).astype(np.int64)
    clouds: dict[int, np.ndarray] = {}
    match_errors: dict[int, int] = {}
    selected_bytes = 0
    accumulator = IntensityAccumulator(contract['intensity_health'])
    previous_stamp = -1
    first_stamp = None
    last_stamp = None
    bag_path = source['bag_path']
    stat_before = bag_path.stat()
    with AnyReader([bag_path]) as reader:
        connections = [connection for connection in reader.connections
                       if connection.topic == source['lidar_topic']]
        if len(connections) != 1:
            raise ContractError('exactly one bound LiDAR connection is required')
        connection = connections[0]
        if connection.msgtype != raw_config['message_type']:
            raise ContractError('bound LiDAR topic is not PointCloud2')
        for scan_index, (current, _, serialized) in enumerate(
                reader.messages(connections=connections)):
            message = reader.deserialize(serialized, current.msgtype)
            stamp = header_stamp_ns(message)
            if (raw_config['require_monotonic_header_timestamps']
                    and stamp <= previous_stamp):
                raise ContractError('PointCloud2 header timestamps are not increasing')
            previous_stamp = stamp
            first_stamp = stamp if first_stamp is None else first_stamp
            last_stamp = stamp
            points = pointcloud_view(
                message, raw_config, source['lidar_frame'],
                int(extraction['maximum_raw_points_per_scan']),
                int(contract['memory']['maximum_message_bytes']))
            accumulator.add(points['intensity'])
            required_position = nearest_required_index(required_stamps, stamp)
            if required_position is not None:
                error_ns = stamp - int(required_stamps[required_position])
                if abs(error_ns) <= maximum_match_ns:
                    state_index = required_states[required_position]
                    previous_error = match_errors.get(state_index)
                    if previous_error is None or abs(error_ns) < abs(previous_error):
                        cloud = extract_selected_cloud(points, lidar_to_body, extraction)
                        old = clouds.get(state_index)
                        selected_bytes -= 0 if old is None else old.nbytes
                        selected_bytes += cloud.nbytes
                        if selected_bytes > int(
                                contract['memory']['maximum_selected_cloud_bytes']):
                            raise MemoryBudgetError(
                                'selected raw clouds exceed fixed byte budget')
                        clouds[state_index] = cloud
                        match_errors[state_index] = error_ns
            if scan_index % 64 == 0:
                memory.check('raw_bag_stream')
    missing = sorted(set(required_states) - set(clouds))
    if missing:
        raise ContractError(
            f'{len(missing)} required states have no raw scan within timestamp gate')
    stat_after = bag_path.stat()
    if (stat_before.st_ino, stat_before.st_size, stat_before.st_mtime_ns) != (
            stat_after.st_ino, stat_after.st_size, stat_after.st_mtime_ns):
        raise ContractError('raw bag metadata changed during report-only audit')
    cloud_digest = hashlib.sha256()
    for state_index in sorted(clouds):
        cloud_digest.update(state_index.to_bytes(8, 'little', signed=False))
        cloud_digest.update(clouds[state_index].astype('<f4', copy=False).tobytes())
    errors = list(match_errors.values())
    return {
        'topic': source['lidar_topic'],
        'message_type': raw_config['message_type'],
        'frame_id': source['lidar_frame'],
        'first_header_stamp_ns': first_stamp,
        'last_header_stamp_ns': last_stamp,
        'intensity': accumulator.result(),
        'required_state_count': len(required_states),
        'matched_state_count': len(clouds),
        'maximum_absolute_match_error_ns': max(map(abs, errors), default=None),
        'minimum_signed_match_error_ns': min(errors, default=None),
        'maximum_signed_match_error_ns': max(errors, default=None),
        'selected_cloud_bytes': selected_bytes,
        'selected_cloud_payload_sha256': cloud_digest.hexdigest(),
        'bag_metadata_unchanged': True,
    }, clouds


class RawSubmapLoader:
    def __init__(self, transforms: np.ndarray, anchors: list[int],
                 clouds: dict[int, np.ndarray], config: dict[str, Any],
                 memory: MemoryGuard) -> None:
        self.transforms = transforms
        self.anchors = anchors
        self.clouds = clouds
        self.config = config
        self.memory = memory
        self.cache: OrderedDict[int, np.ndarray] = OrderedDict()
        self.cache_bytes = 0
        self.peak_cache_bytes = 0
        self.peak_submap_bytes = 0

    def build(self, anchor_index: int) -> np.ndarray:
        if anchor_index in self.cache:
            value = self.cache.pop(anchor_index)
            self.cache[anchor_index] = value
            return value
        radius = int(self.config['anchor_radius'])
        center_state = self.anchors[anchor_index]
        center_from_world = np.linalg.inv(self.transforms[center_state])
        pieces = []
        begin = max(0, anchor_index - radius)
        end = min(len(self.anchors), anchor_index + radius + 1)
        for neighbor in range(begin, end):
            state_index = self.anchors[neighbor]
            if state_index not in self.clouds:
                raise ContractError('required raw cloud was not retained')
            center_from_body = center_from_world @ self.transforms[state_index]
            cloud = self.clouds[state_index]
            transformed = np.empty_like(cloud)
            transformed[:, :3] = transform_xyz(
                cloud[:, :3].astype(np.float64), center_from_body)
            transformed[:, 3] = cloud[:, 3]
            pieces.append(transformed)
        merged = np.concatenate(pieces) if pieces else np.empty((0, 4), np.float32)
        result = voxel_mean_xyzi(
            merged, float(self.config['voxel_size_m']),
            int(self.config['maximum_points']))
        self.peak_submap_bytes = max(self.peak_submap_bytes, merged.nbytes, result.nbytes)
        limit = int(self.config['maximum_cache_bytes'])
        entries = int(self.config['cache_entries'])
        while self.cache and (len(self.cache) >= entries
                              or self.cache_bytes + result.nbytes > limit):
            _, evicted = self.cache.popitem(last=False)
            self.cache_bytes -= evicted.nbytes
        if result.nbytes > limit:
            raise MemoryBudgetError('one raw submap exceeds maximum_cache_bytes')
        self.cache[anchor_index] = result
        self.cache_bytes += result.nbytes
        self.peak_cache_bytes = max(self.peak_cache_bytes, self.cache_bytes)
        self.memory.check('raw_submap_build')
        return result


def nearest_distances_indices(source: np.ndarray, target: np.ndarray,
                              target_tree: Any | None = None
                              ) -> tuple[np.ndarray, np.ndarray]:
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError as error:
        if len(source) * len(target) > 1_000_000:
            raise ContractError('scipy is required for bounded real-data matching') from error
        delta = source[:, None, :] - target[None, :, :]
        squared = np.sum(delta*delta, axis=2)
        indices = np.argmin(squared, axis=1)
        return np.sqrt(squared[np.arange(len(source)), indices]), indices
    tree = target_tree if target_tree is not None else cKDTree(target)
    distances, indices = tree.query(source, k=1, workers=1)
    return (distances.astype(np.float64, copy=False),
            indices.astype(np.int64, copy=False))


def overlap_local_pearson(first: np.ndarray, second: np.ndarray) -> float | None:
    first = np.asarray(first, dtype=np.float64)
    second = np.asarray(second, dtype=np.float64)
    if len(first) != len(second) or len(first) < 2:
        return None
    first = first - np.mean(first)
    second = second - np.mean(second)
    denominator = float(np.linalg.norm(first) * np.linalg.norm(second))
    if not math.isfinite(denominator) or denominator <= 1e-12:
        return None
    return float(np.clip(np.dot(first, second) / denominator, -1.0, 1.0))


def score_alignment(source: np.ndarray, target: np.ndarray, transform: np.ndarray,
                    config: dict[str, Any], target_tree: Any | None = None
                    ) -> dict[str, Any]:
    aligned = transform_xyz(source[:, :3].astype(np.float64), transform)
    forward_distances, forward_indices = nearest_distances_indices(
        aligned, target[:, :3], target_tree)
    reverse_distances, reverse_indices = nearest_distances_indices(
        target[:, :3], aligned)
    limit = float(config['maximum_correspondence_m'])
    source_indices = np.arange(len(source), dtype=np.int64)
    mutual = ((forward_distances <= limit)
              & (reverse_indices[forward_indices] == source_indices))
    selected_source = np.flatnonzero(mutual)
    selected_target = forward_indices[selected_source]
    first = source[selected_source, 3].astype(np.float64)
    second = target[selected_target, 3].astype(np.float64)
    support = len(first)
    fraction = support / max(1, min(len(source), len(target)))
    first_range = float(np.ptp(first)) if support else 0.0
    second_range = float(np.ptp(second)) if support else 0.0
    first_std = float(np.std(first)) if support else 0.0
    second_std = float(np.std(second)) if support else 0.0
    correlation = overlap_local_pearson(first, second)
    checks = {
        'support_count': support >= int(config['minimum_mutual_correspondences']),
        'support_fraction': fraction >= float(
            config['minimum_mutual_correspondence_fraction']),
        'source_dynamic_range': first_range >= float(
            config['minimum_overlap_intensity_dynamic_range']),
        'target_dynamic_range': second_range >= float(
            config['minimum_overlap_intensity_dynamic_range']),
        'source_standard_deviation': first_std >= float(
            config['minimum_overlap_intensity_standard_deviation']),
        'target_standard_deviation': second_std >= float(
            config['minimum_overlap_intensity_standard_deviation']),
        'pearson_defined': correlation is not None,
    }
    return {
        'valid': all(checks.values()),
        'checks': checks,
        'mutual_correspondence_count': support,
        'mutual_correspondence_fraction': fraction,
        'source_overlap_dynamic_range': first_range,
        'target_overlap_dynamic_range': second_range,
        'source_overlap_standard_deviation': first_std,
        'target_overlap_standard_deviation': second_std,
        'overlap_local_pearson': correlation,
        'support_rmse_m': (float(np.sqrt(np.mean(forward_distances[selected_source]**2)))
                           if support else None),
    }


def challenge_pair(source: np.ndarray, target: np.ndarray, transform: np.ndarray,
                   config: dict[str, Any]) -> dict[str, Any]:
    try:
        from scipy.spatial import cKDTree
        target_tree = cKDTree(target[:, :3])
    except ModuleNotFoundError:
        target_tree = None
    selected = score_alignment(source, target, transform, config, target_tree)
    shift = float(config['spatial_decoy_translation_m'])
    decoys = []
    for dx, dy in ((-shift, 0.0), (shift, 0.0), (0.0, -shift), (0.0, shift)):
        candidate = transform.copy()
        candidate[:3, 3] += (dx, dy, 0.0)
        score = score_alignment(source, target, candidate, config, target_tree)
        decoys.append({
            'translation_xy_m': [dx, dy],
            'valid': score['valid'],
            'mutual_correspondence_count': score['mutual_correspondence_count'],
            'overlap_local_pearson': score['overlap_local_pearson'],
        })
    valid_decoys = [item['overlap_local_pearson'] for item in decoys
                    if item['valid'] and item['overlap_local_pearson'] is not None]
    best_decoy = max(valid_decoys, default=None)
    correlation = selected['overlap_local_pearson']
    margin = (correlation - best_decoy
              if correlation is not None and best_decoy is not None else None)
    reasons = []
    if not selected['valid']:
        reasons.append('invalid_selected_overlap')
    if correlation is None or correlation < float(config['minimum_overlap_local_pearson']):
        reasons.append('pearson')
    if len(valid_decoys) < int(config['minimum_valid_spatial_decoys']):
        reasons.append('valid_spatial_decoys')
    if margin is None or margin < float(config['minimum_spatial_peak_margin']):
        reasons.append('spatial_peak_margin')
    return {
        'passed': not reasons,
        'reasons': sorted(set(reasons)),
        'selected': selected,
        'spatial_decoys': decoys,
        'valid_spatial_decoy_count': len(valid_decoys),
        'best_spatial_decoy_pearson': best_decoy,
        'spatial_peak_margin': margin,
    }


def challenge_candidate(candidate: dict[str, Any], loader: RawSubmapLoader,
                        config: dict[str, Any]) -> dict[str, Any]:
    pairs = []
    for v42_pair in candidate['geometry']['pairs']:
        source_anchor = int(v42_pair['source_anchor'])
        target_anchor = int(v42_pair['target_anchor'])
        source_cloud = loader.build(source_anchor)
        target_cloud = loader.build(target_anchor)
        transform = np.asarray(v42_pair['target_from_source_matrix'], dtype=np.float64)
        result = challenge_pair(source_cloud, target_cloud, transform, config)
        result.update({
            'offset': int(v42_pair['offset']),
            'source_anchor': source_anchor,
            'target_anchor': target_anchor,
            'source_points': len(source_cloud),
            'target_points': len(target_cloud),
            'v42_target_from_source_sha256': payload_sha256(
                v42_pair['target_from_source_matrix']),
        })
        pairs.append(result)
        loader.memory.check('raw_identity_pair')
    require_every = bool(config['require_every_sequence_pair'])
    passed_count = sum(bool(pair['passed']) for pair in pairs)
    reasons = []
    if require_every and passed_count != len(pairs):
        reasons.append('sequence_pair_gate')
    elif not require_every and passed_count == 0:
        reasons.append('sequence_pair_gate')
    correlations = [pair['selected']['overlap_local_pearson'] for pair in pairs
                    if pair['selected']['overlap_local_pearson'] is not None]
    margins = [pair['spatial_peak_margin'] for pair in pairs
               if pair['spatial_peak_margin'] is not None]
    return {
        'passed': not reasons,
        'reasons': reasons,
        'passing_pair_count': passed_count,
        'pair_count': len(pairs),
        'minimum_pair_pearson': min(correlations, default=None),
        'median_pair_pearson': (float(np.median(correlations))
                                if correlations else None),
        'minimum_spatial_peak_margin': min(margins, default=None),
        'pairs': pairs,
    }


def deduplicate_new_constraints(records: list[dict[str, Any]], window: int
                                ) -> list[dict[str, Any]]:
    eligible = [record for record in records
                if record['identity']['passed'] and not record['matching_legacy_edges']]
    eligible.sort(key=lambda record: (
        -float(record['identity']['minimum_pair_pearson']),
        -float(record['identity']['minimum_spatial_peak_margin']),
        int(record['source_anchor']), int(record['target_anchor'])))
    selected = []
    for record in eligible:
        duplicate = next((other for other in selected
                          if abs(record['source_anchor'] - other['source_anchor']) <= window
                          and abs(record['target_anchor'] - other['target_anchor']) <= window), None)
        if duplicate is None:
            selected.append(record)
    return [{
        'source_anchor': int(record['source_anchor']),
        'target_anchor': int(record['target_anchor']),
        'minimum_pair_pearson': record['identity']['minimum_pair_pearson'],
        'minimum_spatial_peak_margin':
            record['identity']['minimum_spatial_peak_margin'],
    } for record in sorted(selected,
                           key=lambda item: (item['source_anchor'], item['target_anchor']))]


def audit_sequence(*, contract_path: Path, source_manifest_path: Path,
                   sequence_id: str, repetition: int, output: Path) -> dict[str, Any]:
    started = time.monotonic()
    contract, contract_sha = load_contract(contract_path)
    memory = MemoryGuard(float(contract['memory']['maximum_rss_mib']))
    report: dict[str, Any] = {
        'schema_version': 1,
        'audit': 'v43b_raw_intensity_place_identity',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'sequence_id': sequence_id,
        'repetition': int(repetition),
        'status': 'FAIL',
    }
    try:
        source, source_manifest_sha = load_source_binding(
            source_manifest_path, sequence_id, verify_hashes=True)
        state_path = source['voxel_dir'] / 'alidarState.txt'
        protected_before = {
            'state': sha256_file(state_path),
            'voxel_config': sha256_file(source['voxel_config_path']),
            'v42_report': sha256_file(source['v42_report_path']),
            'legacy_edges': sha256_file(source['legacy_edges_path']),
        }
        stamps, transforms = load_states(state_path)
        anchors = select_anchor_indices(
            transforms, float(contract['anchor']['minimum_translation_m']))
        lidar_to_body, extrinsic_report = load_extrinsic(source)
        v42_report, candidates = load_v42_geometry_candidates(
            source, sequence_id, anchors, contract)
        required_states = required_state_indices(
            candidates, anchors, int(contract['submap']['anchor_radius']),
            int(contract['extraction']['maximum_selected_scan_count']))
        raw_report, clouds = stream_raw_bag(
            source=source, stamps=stamps, required_states=required_states,
            lidar_to_body=lidar_to_body, contract=contract, memory=memory)
        legacy = load_legacy_edges(source['legacy_edges_path'])
        if (payload_sha256(legacy) != v42_report['deterministic']
                ['legacy_challenge']['edges_sha256']):
            raise ContractError('legacy edge payload differs from v42 challenge')
        identity_records = []
        loader = RawSubmapLoader(
            transforms, anchors, clouds, contract['submap'], memory)
        if raw_report['intensity']['available']:
            window = int(contract['legacy_challenge']['deduplication_index_window'])
            for candidate in candidates:
                identity = challenge_candidate(candidate, loader, contract['identity'])
                source_anchor = int(candidate['source_anchor'])
                target_anchor = int(candidate['target_anchor'])
                identity_records.append({
                    'source_anchor': source_anchor,
                    'target_anchor': target_anchor,
                    'direction': candidate['direction'],
                    'v42_disposition': candidate['disposition'],
                    'matching_legacy_edges': matching_legacy_edges(
                        source_anchor, target_anchor, legacy, window),
                    'identity': identity,
                    'disposition': ('PASS_RAW_INTENSITY_IDENTITY'
                                    if identity['passed']
                                    else 'REJECT_RAW_INTENSITY_IDENTITY'),
                })
        challenge_complete = (
            len(identity_records) == len(candidates)
            if contract['legacy_challenge']['require_every_v42_geometry_pass']
            else True)
        legacy_survivors = [
            {'source_anchor': record['source_anchor'],
             'target_anchor': record['target_anchor'],
             'matching_legacy_edges': record['matching_legacy_edges']}
            for record in identity_records
            if record['identity']['passed'] and record['matching_legacy_edges']]
        new_constraints = deduplicate_new_constraints(
            identity_records,
            int(contract['legacy_challenge']['deduplication_index_window']))
        protected_after = {
            'state': sha256_file(state_path),
            'voxel_config': sha256_file(source['voxel_config_path']),
            'v42_report': sha256_file(source['v42_report_path']),
            'legacy_edges': sha256_file(source['legacy_edges_path']),
        }
        if protected_before != protected_after:
            raise ContractError('protected input changed during report-only audit')
        deterministic = {
            'sequence_id': sequence_id,
            'contract_id': contract['contract_id'],
            'contract_sha256': contract_sha,
            'source': {
                'source_set_id': source['source_set_id'],
                'source_manifest_sha256': source_manifest_sha,
                'bag_path': str(source['bag_path']),
                'bag_size_bytes': int(source['bag_size_bytes']),
                'bag_sha256': source['bag_sha256'],
                'state_path': str(state_path),
                'state_count': len(stamps),
                'state_sha256': source['state_sha256'],
                'voxel_config_path': str(source['voxel_config_path']),
                'voxel_config_sha256': source['voxel_config_sha256'],
                'v42_report_path': str(source['v42_report_path']),
                'v42_report_sha256': source['v42_report_sha256'],
                'v42_deterministic_payload_sha256':
                    v42_report['deterministic_payload_sha256'],
                'legacy_edges_path': str(source['legacy_edges_path']),
                'legacy_edges_sha256': source['legacy_edges_sha256'],
            },
            'extrinsic': extrinsic_report,
            'anchor': {
                'count': len(anchors),
                'state_indices_sha256': payload_sha256(anchors),
            },
            'raw': raw_report,
            'challenge': {
                'v42_geometry_pass_count': len(candidates),
                'challenged_count': len(identity_records),
                'complete': challenge_complete,
                'identity_pass_count_before_dedup': sum(
                    record['identity']['passed'] for record in identity_records),
                'legacy_geometry_candidate_count': sum(
                    bool(record['matching_legacy_edges']) for record in identity_records),
                'legacy_survivors': legacy_survivors,
                'new_verified_constraints': new_constraints,
                'records': identity_records,
            },
            'submap': {
                'peak_cache_bytes': loader.peak_cache_bytes,
                'peak_submap_bytes': loader.peak_submap_bytes,
            },
            'protected_inputs_unchanged': True,
        }
        report['deterministic'] = deterministic
        report['deterministic_payload_sha256'] = payload_sha256(deterministic)
        report['status'] = 'PASS'
    except Exception as error:
        report['error_type'] = type(error).__name__
        report['error'] = str(error)
    finally:
        try:
            memory.check('audit_finalize')
        except MemoryBudgetError as error:
            report['status'] = 'FAIL'
            report['error_type'] = type(error).__name__
            report['error'] = str(error)
        report['runtime'] = {
            'wall_seconds': time.monotonic() - started,
            'peak_rss_mib': memory.peak_rss_mib,
            'maximum_rss_mib': memory.maximum_rss_mib,
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + '\n',
            encoding='utf-8')
    return report


def aggregate_reports(*, contract_path: Path, reports: list[Path],
                      expected_sequences: list[str], output: Path,
                      markdown_output: Path | None = None) -> dict[str, Any]:
    contract, contract_sha = load_contract(contract_path)
    loaded = [(path, json.loads(path.read_text(encoding='utf-8'))) for path in reports]
    groups: dict[str, list[tuple[Path, dict[str, Any]]]] = {
        sequence: [] for sequence in expected_sequences}
    for path, report in loaded:
        sequence = report.get('sequence_id')
        if sequence not in groups:
            raise ContractError(f'unexpected sequence {sequence!r} in {path}')
        groups[sequence].append((path, report))
    required = int(contract['decision']['required_repetitions'])
    complete = True
    repeatable = True
    unavailable = []
    incomplete_challenges = []
    applicable_without_new = []
    total_legacy_survivors = 0
    total_new_constraints = 0
    sequence_results = []
    for sequence in expected_sequences:
        items = sorted(groups[sequence], key=lambda item: int(item[1].get('repetition', 0)))
        hashes = [item[1].get('deterministic_payload_sha256') for item in items]
        integrity = [
            report.get('status') == 'PASS'
            and report.get('contract_sha256') == contract_sha
            and report.get('deterministic_payload_sha256') ==
            payload_sha256(report.get('deterministic', {}))
            for _, report in items]
        item_complete = len(items) == required and all(integrity)
        item_repeatable = item_complete and len(set(hashes)) == 1
        complete &= item_complete
        repeatable &= item_repeatable
        deterministic = items[0][1].get('deterministic', {}) if items else {}
        intensity = deterministic.get('raw', {}).get('intensity', {})
        challenge = deterministic.get('challenge', {})
        available = bool(intensity.get('available'))
        challenge_complete = bool(challenge.get('complete'))
        candidates = int(challenge.get('v42_geometry_pass_count', 0))
        new_constraints = len(challenge.get('new_verified_constraints', []))
        legacy_survivors = len(challenge.get('legacy_survivors', []))
        if not available:
            unavailable.append(sequence)
        if not challenge_complete:
            incomplete_challenges.append(sequence)
        if (candidates > 0 and new_constraints == 0
                and contract['decision']['require_new_constraint_per_applicable_sequence']):
            applicable_without_new.append(sequence)
        total_new_constraints += new_constraints
        total_legacy_survivors += legacy_survivors
        sequence_results.append({
            'sequence_id': sequence,
            'complete': item_complete,
            'repeatable': item_repeatable,
            'deterministic_payload_sha256': hashes[0] if item_repeatable else None,
            'raw_intensity_available': available,
            'raw_scan_count': int(intensity.get('scan_count', 0)),
            'raw_intensity_point_count': int(intensity.get('point_count', 0)),
            'raw_intensity_nonzero_fraction': float(
                intensity.get('nonzero_fraction', 0.0)),
            'raw_intensity_dynamic_range': float(intensity.get('dynamic_range', 0.0)),
            'v42_geometry_pass_count': candidates,
            'challenged_count': int(challenge.get('challenged_count', 0)),
            'identity_pass_count_before_dedup': int(
                challenge.get('identity_pass_count_before_dedup', 0)),
            'legacy_survivor_count': legacy_survivors,
            'new_verified_constraint_count': new_constraints,
        })
    if not complete or not repeatable:
        decision = 'REJECT_V43B_INCOMPLETE_OR_NONREPEATABLE_AUDIT'
    elif (contract['decision']['require_raw_intensity_on_every_sequence']
          and unavailable):
        decision = 'CLOSE_V43B_GLOBAL_CORRECTION_ROUTE_RAW_INTENSITY_UNAVAILABLE'
    elif incomplete_challenges:
        decision = 'REJECT_V43B_INCOMPLETE_LEGACY_CHALLENGE'
    elif total_legacy_survivors > int(
            contract['decision']['maximum_legacy_challenge_survivors']):
        decision = 'REJECT_V43B_LEGACY_AMBIGUITY_SURVIVED'
    elif (total_new_constraints < int(
            contract['decision']['minimum_total_new_verified_constraints'])
          or applicable_without_new):
        decision = 'CLOSE_V43B_GLOBAL_CORRECTION_ROUTE_NO_NEW_UNAMBIGUOUS_CONSTRAINT_SET'
    else:
        decision = 'AUTHORIZE_V43B_EXTERNAL_SPARSE_POSE_GRAPH_IMPLEMENTATION'
    aggregate = {
        'schema_version': 1,
        'audit': 'v43b_raw_intensity_place_identity_aggregate',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'implementation_sha256': sha256_file(Path(__file__)),
        'status': 'PASS' if complete and repeatable else 'FAIL',
        'decision': decision,
        'external_sparse_pose_graph_authorized': decision.startswith('AUTHORIZE_'),
        'sequences_without_raw_intensity': unavailable,
        'sequences_with_incomplete_challenge': incomplete_challenges,
        'applicable_sequences_without_new_constraint': applicable_without_new,
        'total_legacy_challenge_survivors': total_legacy_survivors,
        'total_new_verified_constraints': total_new_constraints,
        'sequence_results': sequence_results,
        'source_reports': [
            {'path': str(path.resolve()), 'sha256': sha256_file(path)}
            for path, _ in loaded],
    }
    aggregate['aggregate_payload_sha256'] = payload_sha256({
        key: value for key, value in aggregate.items() if key != 'source_reports'})
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(aggregate, indent=2, sort_keys=True) + '\n',
                      encoding='utf-8')
    if markdown_output:
        lines = [
            '# v43b raw-intensity place-identity challenge', '',
            f"- decision: `{decision}`",
            '- external sparse pose graph authorized: '
            f"`{str(aggregate['external_sparse_pose_graph_authorized']).lower()}`",
            f'- legacy challenge survivors: `{total_legacy_survivors}`',
            f'- new verified constraints: `{total_new_constraints}`', '',
            '## Sequences', '',
        ]
        for item in sequence_results:
            lines.append(
                f"- `{item['sequence_id']}`: repeatable="
                f"{str(item['repeatable']).lower()}, scans={item['raw_scan_count']}, "
                f"points={item['raw_intensity_point_count']}, "
                f"nonzero={item['raw_intensity_nonzero_fraction']:.6f}, "
                f"geometry={item['v42_geometry_pass_count']}, "
                f"identity_pass={item['identity_pass_count_before_dedup']}, "
                f"legacy_survivors={item['legacy_survivor_count']}, "
                f"new={item['new_verified_constraint_count']}")
        markdown_output.parent.mkdir(parents=True, exist_ok=True)
        markdown_output.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return aggregate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    audit = subparsers.add_parser('audit')
    audit.add_argument('--contract', type=Path, required=True)
    audit.add_argument('--source-manifest', type=Path, required=True)
    audit.add_argument('--sequence-id', required=True)
    audit.add_argument('--repetition', type=int, required=True)
    audit.add_argument('--output', type=Path, required=True)
    aggregate = subparsers.add_parser('aggregate')
    aggregate.add_argument('--contract', type=Path, required=True)
    aggregate.add_argument('--report', type=Path, action='append', required=True)
    aggregate.add_argument('--expected-sequence', action='append', required=True)
    aggregate.add_argument('--output', type=Path, required=True)
    aggregate.add_argument('--markdown-output', type=Path)
    return parser.parse_args()


def main() -> int:
    options = parse_args()
    if options.command == 'audit':
        report = audit_sequence(
            contract_path=options.contract,
            source_manifest_path=options.source_manifest,
            sequence_id=options.sequence_id,
            repetition=options.repetition,
            output=options.output,
        )
        return 0 if report['status'] == 'PASS' else 2
    aggregate = aggregate_reports(
        contract_path=options.contract,
        reports=options.report,
        expected_sequences=options.expected_sequence,
        output=options.output,
        markdown_output=options.markdown_output,
    )
    return 0 if aggregate['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
