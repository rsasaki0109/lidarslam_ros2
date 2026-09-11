#!/usr/bin/env python3
"""Audit exact raw LiDAR/IMU readiness for a bounded fixed-lag architecture."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import resource
import time
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


class ContractError(ValueError):
    """Raised when an input violates the frozen v44 source contract."""


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
        'schema_version', 'contract_id', 'frames', 'messages', 'timing',
        'imu', 'startup_classifier', 'memory', 'decision',
    }
    missing = sorted(required - set(contract))
    if missing:
        raise ContractError(f'contract missing keys: {missing}')
    if contract['schema_version'] != 1:
        raise ContractError('unsupported contract schema_version')
    if contract['frames'] != {
            'world': 'map', 'estimator_body': 'base_link',
            'source_frame_policy': 'exact_manifest_binding'}:
        raise ContractError('v44 frame policy changed')
    messages = contract['messages']
    if messages['lidar_type'] != 'sensor_msgs/msg/PointCloud2':
        raise ContractError('v44 LiDAR message type changed')
    if messages['imu_type'] != 'sensor_msgs/msg/Imu':
        raise ContractError('v44 IMU message type changed')
    if messages['point_time_semantics'] != 'uint32_nanoseconds_from_scan_start':
        raise ContractError('v44 point-time semantics changed')
    names = [item['name'] for item in messages['required_lidar_fields']]
    if names != [
            'x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring',
            'ambient', 'range']:
        raise ContractError('v44 canonical PointCloud2 layout changed')
    timing = contract['timing']
    if not 0.0 <= float(timing['minimum_fully_bracketed_lidar_fraction']) <= 1.0:
        raise ContractError('bracket fraction must be in [0, 1]')
    if int(timing['minimum_scan_time_span_ns']) <= 0:
        raise ContractError('minimum scan span must be positive')
    if (int(timing['minimum_scan_time_span_ns'])
            > int(timing['maximum_scan_time_span_ns'])):
        raise ContractError('scan span bounds are reversed')
    if int(timing['minimum_point_time_offset_ns']) < 0:
        raise ContractError('minimum point-time offset must be non-negative')
    if not bool(timing['require_exact_normalization_timing_digest']):
        raise ContractError('exact normalization timing digest must remain required')
    if float(contract['memory']['maximum_rss_mib']) <= 0.0:
        raise ContractError('maximum_rss_mib must be positive')
    if bool(contract['decision']['published_trajectory_or_map_writes_allowed']):
        raise ContractError('v44 stage 1 must remain report-only')
    if bool(contract['decision']['accuracy_or_reference_map_inputs_allowed']):
        raise ContractError('v44 stage 1 must not open accuracy references')
    return contract, sha256_file(path)


def _resolve_path(value: str) -> Path:
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
        'bag_path', 'sensor_manifest_path', 'normalization_report_path',
        'sensor_adapter_path')
    for field in path_fields:
        source[field] = _resolve_path(source[field]).resolve()
    files = {
        'bag': (source['bag_path'], source['bag_sha256']),
        'sensor_manifest': (
            source['sensor_manifest_path'], source['sensor_manifest_sha256']),
        'normalization_report': (
            source['normalization_report_path'], source['normalization_report_sha256']),
        'sensor_adapter': (
            source['sensor_adapter_path'], source['sensor_adapter_sha256']),
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
    validate_provenance(source)
    return source, source['source_manifest_sha256']


def validate_provenance(source: dict[str, Any]) -> None:
    sensor = json.loads(source['sensor_manifest_path'].read_text(encoding='utf-8'))
    normalization = json.loads(
        source['normalization_report_path'].read_text(encoding='utf-8'))
    if sensor.get('accuracy_ground_truth_accessed') is not False:
        raise ContractError('sensor manifest is not pre-accuracy')
    if sensor.get('base_frame') != source['base_frame']:
        raise ContractError('sensor manifest base frame differs from binding')
    if sensor.get('topics') != [source['lidar_topic'], source['imu_topic']]:
        raise ContractError('sensor manifest topics differ from binding')
    if sensor.get('canonical_ros1_bag', {}).get('sha256') != source['bag_sha256']:
        raise ContractError('sensor manifest bag hash differs from binding')
    if normalization.get('accuracy_ground_truth_accessed') is not False:
        raise ContractError('normalization report accessed accuracy ground truth')
    if normalization.get('navigation_topics_accessed') is not False:
        raise ContractError('normalization report accessed navigation topics')
    if normalization.get('canonical_ros1_bag', {}).get('sha256') != source['bag_sha256']:
        raise ContractError('normalization report bag hash differs from binding')
    expected_counts = {
        source['lidar_topic']: int(source['expected_lidar_messages']),
        source['imu_topic']: int(source['expected_imu_messages']),
    }
    if normalization.get('message_counts_by_source_topic') != expected_counts:
        raise ContractError('normalization message counts differ from binding')
    point_filtering = normalization.get('lidar_point_filtering', {})
    if int(point_filtering.get('output_point_count', -1)) != int(
            source['expected_lidar_points']):
        raise ContractError('normalization point count differs from binding')
    lidar = normalization.get('lidar_normalization', {})
    if (lidar.get('output_time_unit') != 'relative_nanoseconds_from_scan_start'
            or lidar.get('output_time_field') != 't'
            or int(lidar.get('output_point_step', -1)) != 48):
        raise ContractError('normalization timing/layout semantics differ')
    try:
        import yaml
    except ModuleNotFoundError as error:
        raise ContractError('PyYAML is required to bind the sensor adapter') from error
    adapter = yaml.safe_load(source['sensor_adapter_path'].read_text(encoding='utf-8'))
    interface = adapter.get('sensor_interface', {})
    if interface != {
            'lidar_topic': source['lidar_topic'],
            'imu_topic': source['imu_topic'],
            'base_frame': source['base_frame']}:
        raise ContractError('sensor adapter interface differs from binding')
    parameters = adapter.get('rko_parameters', {})
    if (float(parameters.get('lidar_timestamps.multiplier_to_seconds', 0.0))
            != 1e-9
            or parameters.get('lidar_timestamps.force_relative') is not True):
        raise ContractError('sensor adapter point-time semantics differ')
    for field in ('extrinsic_imu2base_quat_xyzw_xyz',
                  'extrinsic_lidar2base_quat_xyzw_xyz'):
        values = np.asarray(parameters.get(field), dtype=np.float64)
        if values.shape != (7,) or not np.isfinite(values).all():
            raise ContractError(f'sensor adapter {field} is not finite length seven')


def point_dtype(messages: dict[str, Any]) -> np.dtype:
    formats = {2: 'u1', 4: '<u2', 6: '<u4', 7: '<f4'}
    fields = messages['required_lidar_fields']
    return np.dtype({
        'names': [item['name'] for item in fields],
        'formats': [formats[int(item['datatype'])] for item in fields],
        'offsets': [int(item['offset']) for item in fields],
        'itemsize': int(messages['lidar_point_step_bytes']),
    })


def pointcloud_view(message: Any, messages: dict[str, Any], expected_frame: str,
                    maximum_message_bytes: int) -> np.ndarray:
    if int(message.height) != int(messages['lidar_height']):
        raise ContractError('PointCloud2 height differs from contract')
    if bool(message.is_bigendian) == bool(messages['lidar_little_endian']):
        raise ContractError('PointCloud2 endianness differs from contract')
    if int(message.point_step) != int(messages['lidar_point_step_bytes']):
        raise ContractError('PointCloud2 point_step differs from contract')
    expected_bytes = int(message.width) * int(message.point_step)
    if int(message.row_step) != expected_bytes or len(message.data) != expected_bytes:
        raise ContractError('PointCloud2 byte count or row_step is inconsistent')
    if expected_bytes > int(maximum_message_bytes):
        raise MemoryBudgetError('PointCloud2 payload exceeds maximum_message_bytes')
    actual_fields = [
        {'name': field.name, 'offset': int(field.offset),
         'datatype': int(field.datatype), 'count': int(field.count)}
        for field in message.fields]
    if actual_fields != messages['required_lidar_fields']:
        raise ContractError('PointCloud2 fields differ from exact canonical schema')
    if message.header.frame_id != expected_frame:
        raise ContractError('PointCloud2 frame differs from exact source binding')
    raw = np.asarray(message.data, dtype=np.uint8)
    if not raw.flags.c_contiguous:
        raise ContractError('PointCloud2 data is not contiguous')
    return np.ndarray(
        shape=(int(message.width),), dtype=point_dtype(messages), buffer=raw)


def header_stamp_ns(message: Any) -> int:
    return (int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec))


def quantile_summary(values: np.ndarray) -> dict[str, Any]:
    values = np.asarray(values)
    if not len(values):
        return {
            'count': 0, 'minimum': None, 'p01': None, 'median': None,
            'p95': None, 'p99': None, 'maximum': None, 'mean': None}
    as_double = values.astype(np.float64)
    return {
        'count': len(values),
        'minimum': float(np.min(as_double)),
        'p01': float(np.percentile(as_double, 1)),
        'median': float(np.percentile(as_double, 50)),
        'p95': float(np.percentile(as_double, 95)),
        'p99': float(np.percentile(as_double, 99)),
        'maximum': float(np.max(as_double)),
        'mean': float(np.mean(as_double)),
    }


class VectorStats:
    def __init__(self) -> None:
        self.count = 0
        self.finite_count = 0
        self.minimum = np.full(3, math.inf, dtype=np.float64)
        self.maximum = np.full(3, -math.inf, dtype=np.float64)
        self.total = np.zeros(3, dtype=np.float64)
        self.total_squared = np.zeros(3, dtype=np.float64)

    def add(self, values: np.ndarray) -> bool:
        values = np.asarray(values, dtype=np.float64)
        self.count += 1
        if values.shape != (3,) or not np.isfinite(values).all():
            return False
        self.finite_count += 1
        self.minimum = np.minimum(self.minimum, values)
        self.maximum = np.maximum(self.maximum, values)
        self.total += values
        self.total_squared += values*values
        return True

    def result(self) -> dict[str, Any]:
        denominator = max(1, self.finite_count)
        mean = self.total / denominator
        variance = np.maximum(0.0, self.total_squared / denominator - mean*mean)
        return {
            'count': self.count,
            'finite_count': self.finite_count,
            'finite_fraction': self.finite_count / max(1, self.count),
            'minimum': self.minimum.tolist() if self.finite_count else None,
            'maximum': self.maximum.tolist() if self.finite_count else None,
            'dynamic_range': ((self.maximum - self.minimum).tolist()
                              if self.finite_count else None),
            'mean': mean.tolist() if self.finite_count else None,
            'standard_deviation': np.sqrt(variance).tolist() if self.finite_count else None,
        }


def classify_covariance(values: Any, config: dict[str, Any],
                        allow_unknown_sentinel: bool) -> str:
    covariance = np.asarray(values, dtype=np.float64)
    if covariance.shape != (9,) or not np.isfinite(covariance).all():
        return 'invalid'
    sentinel = float(config['unknown_orientation_covariance_sentinel'])
    if (allow_unknown_sentinel and covariance[0] == sentinel
            and np.count_nonzero(covariance[1:]) == 0):
        return 'unknown_sentinel'
    if np.count_nonzero(covariance) == 0:
        return 'zero_unavailable'
    matrix = covariance.reshape(3, 3)
    tolerance = float(config['covariance_symmetry_tolerance'])
    if not np.allclose(matrix, matrix.T, atol=tolerance, rtol=0.0):
        return 'invalid'
    eigenvalues = np.linalg.eigvalsh((matrix + matrix.T) * 0.5)
    if float(np.min(eigenvalues)) < -float(
            config['covariance_negative_eigenvalue_tolerance']):
        return 'invalid'
    return 'provided_psd'


class StartupAccumulator:
    def __init__(self, first_stamp_ns: int, config: dict[str, Any]) -> None:
        self.end_stamp_ns = first_stamp_ns + int(float(config['window_sec']) * 1e9)
        self.config = config
        self.count = 0
        self.gyro_sum_squared_norm = 0.0
        self.accel_sum = np.zeros(3, dtype=np.float64)
        self.accel_sum_squared_norm = 0.0

    def add(self, stamp_ns: int, gyro: np.ndarray, accel: np.ndarray) -> None:
        if stamp_ns > self.end_stamp_ns:
            return
        self.count += 1
        self.gyro_sum_squared_norm += float(np.dot(gyro, gyro))
        self.accel_sum += accel
        self.accel_sum_squared_norm += float(np.dot(accel, accel))

    def result(self) -> dict[str, Any]:
        count = max(1, self.count)
        mean_accel = self.accel_sum / count
        gyro_rms = math.sqrt(self.gyro_sum_squared_norm / count)
        residual_variance = max(
            0.0, self.accel_sum_squared_norm / count - float(np.dot(mean_accel, mean_accel)))
        accel_residual_rms = math.sqrt(residual_variance)
        gravity_norm = float(np.linalg.norm(mean_accel))
        checks = {
            'sample_count': self.count >= int(self.config['minimum_imu_samples']),
            'gyro_rms': gyro_rms <= float(self.config['maximum_gyro_rms_rad_s']),
            'gravity_magnitude': abs(
                gravity_norm - float(self.config['gravity_magnitude_m_s2'])) <= float(
                    self.config['maximum_gravity_magnitude_error_m_s2']),
            'acceleration_residual_rms': accel_residual_rms <= float(
                self.config['maximum_acceleration_residual_rms_m_s2']),
        }
        return {
            'stationary_candidate': all(checks.values()),
            'checks': checks,
            'sample_count': self.count,
            'gyro_rms_rad_s': gyro_rms,
            'mean_linear_acceleration_m_s2': mean_accel.tolist(),
            'mean_linear_acceleration_norm_m_s2': gravity_norm,
            'linear_acceleration_residual_rms_m_s2': accel_residual_rms,
        }


def interval_summary(stamps_ns: np.ndarray) -> dict[str, Any]:
    stamps_ns = np.asarray(stamps_ns, dtype=np.int64)
    intervals = np.diff(stamps_ns)
    duration_ns = int(stamps_ns[-1] - stamps_ns[0]) if len(stamps_ns) > 1 else 0
    return {
        'message_count': len(stamps_ns),
        'first_header_stamp_ns': int(stamps_ns[0]) if len(stamps_ns) else None,
        'last_header_stamp_ns': int(stamps_ns[-1]) if len(stamps_ns) else None,
        'duration_sec': duration_ns / 1e9,
        'mean_rate_hz': ((len(stamps_ns) - 1) * 1e9 / duration_ns
                         if duration_ns > 0 else 0.0),
        'interval_ns': quantile_summary(intervals),
    }


def unbracketed_regions(mask: np.ndarray) -> dict[str, int]:
    mask = np.asarray(mask, dtype=bool)
    count = len(mask)
    prefix = 0
    while prefix < count and mask[prefix]:
        prefix += 1
    suffix = 0
    while suffix < count - prefix and mask[count - suffix - 1]:
        suffix += 1
    return {
        'total': int(np.count_nonzero(mask)),
        'prefix': prefix,
        'interior': int(np.count_nonzero(mask[prefix:count-suffix if suffix else count])),
        'suffix': suffix,
    }


def synchronization_summary(lidar_stamps: np.ndarray, scan_ends: np.ndarray,
                            imu_stamps: np.ndarray,
                            config: dict[str, Any]) -> dict[str, Any]:
    lidar_stamps = np.asarray(lidar_stamps, dtype=np.int64)
    scan_ends = np.asarray(scan_ends, dtype=np.int64)
    imu_stamps = np.asarray(imu_stamps, dtype=np.int64)
    full = np.zeros(len(lidar_stamps), dtype=bool)
    maximum_boundary_distances = []
    samples_inside = np.zeros(len(lidar_stamps), dtype=np.int64)
    for index, (start, end) in enumerate(zip(lidar_stamps, scan_ends)):
        start_after = int(np.searchsorted(imu_stamps, start, side='right'))
        end_after = int(np.searchsorted(imu_stamps, end, side='right'))
        start_before = start_after - 1
        end_before = end_after - 1
        bracketed = (start_before >= 0 and start_after < len(imu_stamps)
                     and end_before >= 0 and end_after < len(imu_stamps))
        full[index] = bracketed
        samples_inside[index] = (
            int(np.searchsorted(imu_stamps, end, side='right'))
            - int(np.searchsorted(imu_stamps, start, side='left')))
        if bracketed:
            maximum_boundary_distances.append(max(
                int(start - imu_stamps[start_before]),
                int(imu_stamps[start_after] - start),
                int(end - imu_stamps[end_before]),
                int(imu_stamps[end_after] - end)))
    unbracketed = unbracketed_regions(~full)
    full_fraction = float(np.mean(full)) if len(full) else 0.0
    full_samples = samples_inside[full]
    boundary = np.asarray(maximum_boundary_distances, dtype=np.int64)
    overlap_start = max(int(lidar_stamps[0]), int(imu_stamps[0]))
    overlap_end = min(int(scan_ends[-1]), int(imu_stamps[-1]))
    checks = {
        'fully_bracketed_fraction': full_fraction >= float(
            config['minimum_fully_bracketed_lidar_fraction']),
        'unbracketed_prefix': unbracketed['prefix'] <= int(
            config['maximum_unbracketed_prefix_scans']),
        'unbracketed_interior': unbracketed['interior'] <= int(
            config['maximum_unbracketed_interior_scans']),
        'unbracketed_suffix': unbracketed['suffix'] <= int(
            config['maximum_unbracketed_suffix_scans']),
        'boundary_bracket_distance': (
            len(boundary) > 0 and int(np.max(boundary)) <= int(
                config['maximum_imu_boundary_bracket_distance_ns'])),
        'imu_samples_per_scan': (
            len(full_samples) > 0 and int(np.min(full_samples)) >= int(
                config['minimum_imu_samples_per_fully_bracketed_scan'])),
    }
    return {
        'ready': all(checks.values()),
        'checks': checks,
        'fully_bracketed_scan_count': int(np.count_nonzero(full)),
        'fully_bracketed_scan_fraction': full_fraction,
        'unbracketed_scans': unbracketed,
        'maximum_boundary_bracket_distance_ns': (
            int(np.max(boundary)) if len(boundary) else None),
        'boundary_bracket_distance_ns': quantile_summary(boundary),
        'imu_samples_inside_scan': quantile_summary(full_samples),
        'overlap_duration_sec': max(0, overlap_end - overlap_start) / 1e9,
        'fully_bracketed_mask_sha256': hashlib.sha256(
            full.astype(np.uint8).tobytes()).hexdigest(),
    }


def _vector(message_vector: Any) -> np.ndarray:
    return np.asarray(
        [message_vector.x, message_vector.y, message_vector.z], dtype=np.float64)


def stream_inventory(source: dict[str, Any], contract: dict[str, Any],
                     memory: MemoryGuard) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
    except ModuleNotFoundError as error:
        raise ContractError('rosbags is required for exact ROS1 raw audit') from error
    messages = contract['messages']
    imu_config = contract['imu']
    expected_lidar = int(source['expected_lidar_messages'])
    expected_imu = int(source['expected_imu_messages'])
    maximum_stamps = int(contract['memory']['maximum_stored_timestamps'])
    if expected_lidar + expected_imu > maximum_stamps:
        raise MemoryBudgetError('bound message count exceeds timestamp storage limit')
    lidar_stamps = np.empty(expected_lidar, dtype=np.int64)
    lidar_receive_offsets = np.empty(expected_lidar, dtype=np.int64)
    scan_ends = np.empty(expected_lidar, dtype=np.int64)
    point_counts = np.empty(expected_lidar, dtype=np.int64)
    time_minima = np.empty(expected_lidar, dtype=np.int64)
    time_maxima = np.empty(expected_lidar, dtype=np.int64)
    time_monotonic = np.empty(expected_lidar, dtype=bool)
    imu_stamps = np.empty(expected_imu, dtype=np.int64)
    imu_receive_offsets = np.empty(expected_imu, dtype=np.int64)
    gyro_norms = np.empty(expected_imu, dtype=np.float64)
    accel_norms = np.empty(expected_imu, dtype=np.float64)
    gyro_stats = VectorStats()
    accel_stats = VectorStats()
    orientation_status = Counter()
    orientation_covariance = Counter()
    gyro_covariance = Counter()
    accel_covariance = Counter()
    lidar_digest = hashlib.sha256()
    imu_digest = hashlib.sha256()
    timing_digest = hashlib.sha256()
    lidar_index = 0
    imu_index = 0
    startup: StartupAccumulator | None = None
    bag_path = source['bag_path']
    stat_before = bag_path.stat()
    with AnyReader([bag_path]) as reader:
        lidar_connections = [connection for connection in reader.connections
                             if connection.topic == source['lidar_topic']]
        imu_connections = [connection for connection in reader.connections
                           if connection.topic == source['imu_topic']]
        if len(lidar_connections) != 1 or len(imu_connections) != 1:
            raise ContractError('exactly one LiDAR and one IMU connection are required')
        lidar_connection = lidar_connections[0]
        imu_connection = imu_connections[0]
        if lidar_connection.msgtype != messages['lidar_type']:
            raise ContractError('bound LiDAR topic has the wrong message type')
        if imu_connection.msgtype != messages['imu_type']:
            raise ContractError('bound IMU topic has the wrong message type')
        if (int(lidar_connection.msgcount) != expected_lidar
                or int(imu_connection.msgcount) != expected_imu):
            raise ContractError('bag connection counts differ from source binding')
        selected = [lidar_connection, imu_connection]
        for event_index, (connection, receive_stamp, serialized) in enumerate(
                reader.messages(connections=selected)):
            message = reader.deserialize(serialized, connection.msgtype)
            stamp = header_stamp_ns(message)
            if connection.topic == source['lidar_topic']:
                if lidar_index >= expected_lidar:
                    raise ContractError('more LiDAR messages than source binding')
                if lidar_index and stamp <= int(lidar_stamps[lidar_index - 1]):
                    raise ContractError('LiDAR header timestamps are not increasing')
                points = pointcloud_view(
                    message, messages, source['lidar_frame'],
                    int(contract['memory']['maximum_message_bytes']))
                if len(points) == 0:
                    raise ContractError('empty PointCloud2 scan')
                times = points['t']
                minimum = int(np.min(times))
                maximum = int(np.max(times))
                lidar_stamps[lidar_index] = stamp
                lidar_receive_offsets[lidar_index] = int(receive_stamp) - stamp
                scan_ends[lidar_index] = stamp + maximum
                point_counts[lidar_index] = len(points)
                time_minima[lidar_index] = minimum
                time_maxima[lidar_index] = maximum
                time_monotonic[lidar_index] = bool(np.all(times[1:] >= times[:-1]))
                timing_digest.update(stamp.to_bytes(8, 'big', signed=False))
                timing_digest.update(times.astype('<u4', copy=False).tobytes())
                lidar_digest.update(serialized)
                lidar_index += 1
            else:
                if imu_index >= expected_imu:
                    raise ContractError('more IMU messages than source binding')
                if imu_index and stamp <= int(imu_stamps[imu_index - 1]):
                    raise ContractError('IMU header timestamps are not increasing')
                if message.header.frame_id != source['imu_frame']:
                    raise ContractError('IMU frame differs from exact source binding')
                gyro = _vector(message.angular_velocity)
                accel = _vector(message.linear_acceleration)
                gyro_finite = gyro_stats.add(gyro)
                accel_finite = accel_stats.add(accel)
                gyro_norms[imu_index] = float(np.linalg.norm(gyro))
                accel_norms[imu_index] = float(np.linalg.norm(accel))
                quaternion = np.asarray([
                    message.orientation.x, message.orientation.y,
                    message.orientation.z, message.orientation.w], dtype=np.float64)
                if not np.isfinite(quaternion).all():
                    orientation_status['invalid_nonfinite'] += 1
                else:
                    norm = float(np.linalg.norm(quaternion))
                    if norm <= 1e-12:
                        orientation_status['zero_unavailable'] += 1
                    elif abs(norm - 1.0) <= float(imu_config['orientation_norm_tolerance']):
                        orientation_status['provided_unit'] += 1
                    else:
                        orientation_status['invalid_norm'] += 1
                orientation_covariance[classify_covariance(
                    message.orientation_covariance, imu_config, True)] += 1
                gyro_covariance[classify_covariance(
                    message.angular_velocity_covariance, imu_config, False)] += 1
                accel_covariance[classify_covariance(
                    message.linear_acceleration_covariance, imu_config, False)] += 1
                imu_stamps[imu_index] = stamp
                imu_receive_offsets[imu_index] = int(receive_stamp) - stamp
                if startup is None:
                    startup = StartupAccumulator(stamp, contract['startup_classifier'])
                if gyro_finite and accel_finite:
                    startup.add(stamp, gyro, accel)
                imu_digest.update(serialized)
                imu_index += 1
            if event_index % 4096 == 0:
                memory.check('raw_lidar_imu_stream')
    if lidar_index != expected_lidar or imu_index != expected_imu:
        raise ContractError('streamed message count differs from source binding')
    if startup is None:
        raise ContractError('no IMU startup samples')
    stat_after = bag_path.stat()
    if (stat_before.st_ino, stat_before.st_size, stat_before.st_mtime_ns) != (
            stat_after.st_ino, stat_after.st_size, stat_after.st_mtime_ns):
        raise ContractError('raw bag metadata changed during report-only audit')
    point_total = int(np.sum(point_counts, dtype=np.int64))
    normalization = json.loads(
        source['normalization_report_path'].read_text(encoding='utf-8'))
    expected_timing_digest = normalization['lidar_normalization'][
        'stamp_and_point_times_sha256']
    observed_timing_digest = timing_digest.hexdigest()
    lidar_time = interval_summary(lidar_stamps)
    imu_time = interval_summary(imu_stamps)
    scan_spans = time_maxima - time_minima
    zero_fraction = float(np.mean(time_minima == 0))
    monotonic_fraction = float(np.mean(time_monotonic))
    timing_config = contract['timing']
    lidar_checks = {
        'message_count': lidar_index == expected_lidar,
        'point_count': point_total == int(source['expected_lidar_points']),
        'duration': lidar_time['duration_sec'] >= float(
            timing_config['minimum_stream_duration_sec']),
        'rate': (float(timing_config['minimum_lidar_rate_hz'])
                 <= lidar_time['mean_rate_hz']
                 <= float(timing_config['maximum_lidar_rate_hz'])),
        'maximum_gap': lidar_time['interval_ns']['maximum'] <= int(
            timing_config['maximum_lidar_gap_ns']),
        'minimum_scan_time_span': int(np.min(scan_spans)) >= int(
            timing_config['minimum_scan_time_span_ns']),
        'maximum_scan_time_span': int(np.max(scan_spans)) <= int(
            timing_config['maximum_scan_time_span_ns']),
        'point_time_offset_lower_bound': int(np.min(time_minima)) >= int(
            timing_config['minimum_point_time_offset_ns']),
        'normalization_timing_digest': (
            not bool(timing_config['require_exact_normalization_timing_digest'])
            or observed_timing_digest == expected_timing_digest),
    }
    gyro_result = gyro_stats.result()
    accel_result = accel_stats.result()
    invalid_orientation = sum(
        value for key, value in orientation_status.items() if key.startswith('invalid'))
    invalid_covariance = (
        orientation_covariance['invalid'] + gyro_covariance['invalid']
        + accel_covariance['invalid'])
    imu_checks = {
        'message_count': imu_index == expected_imu,
        'minimum_message_count': imu_index >= int(imu_config['minimum_message_count']),
        'duration': imu_time['duration_sec'] >= float(
            timing_config['minimum_stream_duration_sec']),
        'rate': (float(timing_config['minimum_imu_rate_hz'])
                 <= imu_time['mean_rate_hz']
                 <= float(timing_config['maximum_imu_rate_hz'])),
        'maximum_gap': imu_time['interval_ns']['maximum'] <= int(
            timing_config['maximum_imu_gap_ns']),
        'finite_measurements': (
            gyro_result['finite_fraction'] >= float(
                imu_config['minimum_finite_measurement_fraction'])
            and accel_result['finite_fraction'] >= float(
                imu_config['minimum_finite_measurement_fraction'])),
        'maximum_angular_velocity_norm': float(np.max(gyro_norms)) <= float(
            imu_config['maximum_angular_velocity_norm_rad_s']),
        'maximum_linear_acceleration_norm': float(np.max(accel_norms)) <= float(
            imu_config['maximum_linear_acceleration_norm_m_s2']),
        'median_linear_acceleration_norm': (
            float(imu_config['minimum_median_linear_acceleration_norm_m_s2'])
            <= float(np.median(accel_norms))
            <= float(imu_config['maximum_median_linear_acceleration_norm_m_s2'])),
        'angular_velocity_dynamic_range': min(gyro_result['dynamic_range']) >= float(
            imu_config['minimum_angular_velocity_axis_dynamic_range_rad_s']),
        'linear_acceleration_dynamic_range': min(accel_result['dynamic_range']) >= float(
            imu_config['minimum_linear_acceleration_axis_dynamic_range_m_s2']),
        'orientation_valid_when_present': invalid_orientation == 0,
        'covariance_valid_when_present': invalid_covariance == 0,
    }
    synchronization = synchronization_summary(
        lidar_stamps, scan_ends, imu_stamps, timing_config)
    startup_result = startup.result()
    orientation_available = orientation_status['provided_unit'] == imu_index
    gyro_covariance_available = gyro_covariance['provided_psd'] == imu_index
    accel_covariance_available = accel_covariance['provided_psd'] == imu_index
    return {
        'ready': all(lidar_checks.values()) and all(imu_checks.values())
        and synchronization['ready'],
        'lidar': {
            'topic': source['lidar_topic'],
            'frame_id': source['lidar_frame'],
            'message_type': messages['lidar_type'],
            'checks': lidar_checks,
            'timing': lidar_time,
            'receive_minus_header_ns': quantile_summary(lidar_receive_offsets),
            'point_count': point_total,
            'points_per_scan': quantile_summary(point_counts),
            'point_time_minimum_ns': quantile_summary(time_minima),
            'point_time_maximum_ns': quantile_summary(time_maxima),
            'scan_time_span_ns': quantile_summary(scan_spans),
            'zero_origin_scan_fraction': zero_fraction,
            'nondecreasing_point_time_scan_fraction': monotonic_fraction,
            'stamp_and_point_times_sha256': observed_timing_digest,
            'serialized_payload_sha256': lidar_digest.hexdigest(),
        },
        'imu': {
            'topic': source['imu_topic'],
            'frame_id': source['imu_frame'],
            'message_type': messages['imu_type'],
            'angular_velocity_unit': messages['angular_velocity_unit'],
            'linear_acceleration_unit': messages['linear_acceleration_unit'],
            'checks': imu_checks,
            'timing': imu_time,
            'receive_minus_header_ns': quantile_summary(imu_receive_offsets),
            'angular_velocity': gyro_result,
            'angular_velocity_norm_rad_s': quantile_summary(gyro_norms),
            'linear_acceleration': accel_result,
            'linear_acceleration_norm_m_s2': quantile_summary(accel_norms),
            'orientation_status_counts': dict(sorted(orientation_status.items())),
            'orientation_universally_available': orientation_available,
            'orientation_covariance_status_counts': dict(
                sorted(orientation_covariance.items())),
            'angular_velocity_covariance_status_counts': dict(
                sorted(gyro_covariance.items())),
            'linear_acceleration_covariance_status_counts': dict(
                sorted(accel_covariance.items())),
            'angular_velocity_covariance_universally_available':
                gyro_covariance_available,
            'linear_acceleration_covariance_universally_available':
                accel_covariance_available,
            'serialized_payload_sha256': imu_digest.hexdigest(),
        },
        'synchronization': synchronization,
        'startup': startup_result,
        'architecture_requirements': {
            'explicit_noise_model_required': not (
                gyro_covariance_available and accel_covariance_available),
            'orientation_independent_initialization_required': not orientation_available,
            'dynamic_startup_initialization_required': not startup_result[
                'stationary_candidate'],
        },
        'bag_metadata_unchanged': True,
    }


def audit_sequence(*, contract_path: Path, source_manifest_path: Path,
                   sequence_id: str, repetition: int, output: Path) -> dict[str, Any]:
    started = time.monotonic()
    contract, contract_sha = load_contract(contract_path)
    memory = MemoryGuard(float(contract['memory']['maximum_rss_mib']))
    report: dict[str, Any] = {
        'schema_version': 1,
        'audit': 'v44_raw_lidar_imu_readiness',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'sequence_id': sequence_id,
        'repetition': int(repetition),
        'status': 'FAIL',
    }
    try:
        source, source_manifest_sha = load_source_binding(
            source_manifest_path, sequence_id, verify_hashes=True)
        protected_paths = {
            'sensor_manifest': source['sensor_manifest_path'],
            'normalization_report': source['normalization_report_path'],
            'sensor_adapter': source['sensor_adapter_path'],
        }
        protected_before = {
            key: sha256_file(value) for key, value in protected_paths.items()}
        inventory = stream_inventory(source, contract, memory)
        protected_after = {
            key: sha256_file(value) for key, value in protected_paths.items()}
        if protected_before != protected_after:
            raise ContractError('protected source metadata changed during audit')
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
                'sensor_manifest_path': str(source['sensor_manifest_path']),
                'sensor_manifest_sha256': source['sensor_manifest_sha256'],
                'normalization_report_path': str(
                    source['normalization_report_path']),
                'normalization_report_sha256':
                    source['normalization_report_sha256'],
                'sensor_adapter_path': str(source['sensor_adapter_path']),
                'sensor_adapter_sha256': source['sensor_adapter_sha256'],
                'base_frame': source['base_frame'],
            },
            'inventory': inventory,
            'ready_for_fixed_lag_architecture_definition': inventory['ready'],
            'protected_inputs_unchanged': True,
            'published_trajectory_or_map_written': False,
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
    not_ready = []
    explicit_noise = False
    orientation_independent = False
    dynamic_startup = False
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
        inventory = deterministic.get('inventory', {})
        ready = bool(deterministic.get('ready_for_fixed_lag_architecture_definition'))
        requirements = inventory.get('architecture_requirements', {})
        explicit_noise |= bool(requirements.get('explicit_noise_model_required'))
        orientation_independent |= bool(
            requirements.get('orientation_independent_initialization_required'))
        dynamic_startup |= bool(
            requirements.get('dynamic_startup_initialization_required'))
        if not ready:
            not_ready.append(sequence)
        lidar = inventory.get('lidar', {})
        imu = inventory.get('imu', {})
        sync = inventory.get('synchronization', {})
        startup = inventory.get('startup', {})
        sequence_results.append({
            'sequence_id': sequence,
            'complete': item_complete,
            'repeatable': item_repeatable,
            'deterministic_payload_sha256': hashes[0] if item_repeatable else None,
            'ready': ready,
            'lidar_messages': int(lidar.get('timing', {}).get('message_count', 0)),
            'lidar_rate_hz': float(lidar.get('timing', {}).get('mean_rate_hz', 0.0)),
            'maximum_lidar_gap_ns': lidar.get('timing', {}).get(
                'interval_ns', {}).get('maximum'),
            'imu_messages': int(imu.get('timing', {}).get('message_count', 0)),
            'imu_rate_hz': float(imu.get('timing', {}).get('mean_rate_hz', 0.0)),
            'maximum_imu_gap_ns': imu.get('timing', {}).get(
                'interval_ns', {}).get('maximum'),
            'fully_bracketed_lidar_fraction': float(
                sync.get('fully_bracketed_scan_fraction', 0.0)),
            'unbracketed_scans': sync.get('unbracketed_scans', {}),
            'maximum_boundary_bracket_distance_ns': sync.get(
                'maximum_boundary_bracket_distance_ns'),
            'orientation_universally_available': bool(
                imu.get('orientation_universally_available')),
            'message_covariance_universally_available': bool(
                imu.get('angular_velocity_covariance_universally_available'))
            and bool(imu.get('linear_acceleration_covariance_universally_available')),
            'stationary_startup_candidate': bool(startup.get('stationary_candidate')),
        })
    if not complete or not repeatable:
        decision = 'REJECT_V44_INCOMPLETE_OR_NONREPEATABLE_READINESS_AUDIT'
    elif (contract['decision']['require_every_sequence_ready'] and not_ready):
        decision = 'BLOCK_V44_FIXED_LAG_ARCHITECTURE_RAW_STREAM_NOT_READY'
    else:
        decision = 'AUTHORIZE_V44_FIXED_LAG_ARCHITECTURE_DEFINITION'
    architecture_requirements = {
        'explicit_noise_model_required': explicit_noise,
        'orientation_independent_initialization_required': orientation_independent,
        'dynamic_startup_initialization_required': dynamic_startup,
        'dataset_specific_algorithm_thresholds_allowed': False,
        'loop_closure_or_global_map_correction_allowed': False,
    }
    aggregate = {
        'schema_version': 1,
        'audit': 'v44_raw_lidar_imu_readiness_aggregate',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'implementation_sha256': sha256_file(Path(__file__)),
        'status': 'PASS' if complete and repeatable else 'FAIL',
        'decision': decision,
        'fixed_lag_architecture_definition_authorized': decision.startswith('AUTHORIZE_'),
        'shadow_estimator_implementation_authorized': False,
        'sequences_not_ready': not_ready,
        'architecture_requirements': architecture_requirements,
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
            '# v44 raw LiDAR/IMU readiness', '',
            f"- decision: `{decision}`",
            '- fixed-lag architecture definition authorized: '
            f"`{str(aggregate['fixed_lag_architecture_definition_authorized']).lower()}`",
            '- shadow estimator implementation authorized: `false`',
            f'- architecture requirements: `{canonical_json(architecture_requirements)}`',
            '', '## Sequences', '',
        ]
        for item in sequence_results:
            lines.append(
                f"- `{item['sequence_id']}`: repeatable="
                f"{str(item['repeatable']).lower()}, ready={str(item['ready']).lower()}, "
                f"LiDAR={item['lidar_rate_hz']:.3f} Hz, "
                f"IMU={item['imu_rate_hz']:.3f} Hz, "
                f"bracketed={item['fully_bracketed_lidar_fraction']:.6f}, "
                f"stationary_startup={str(item['stationary_startup_candidate']).lower()}")
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
