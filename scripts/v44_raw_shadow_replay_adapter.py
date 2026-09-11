#!/usr/bin/env python3
"""Read-only, hash-bound raw LiDAR/IMU adapter for the v44 shadow core.

The adapter is deliberately outside the estimator core.  It can be activated
only by the separately generated v44e static-audit aggregate, accepts no bag or
output path from the command line, and writes diagnostic evidence only.  It
never publishes ROS messages and never reads accuracy or reference-map data.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import hashlib
import heapq
import importlib.util
import json
import math
from pathlib import Path
import resource
import sys
import time
from typing import Any, Mapping, Sequence

import numpy as np


EXECUTION_CONTRACT_ID = 'v44e-raw-shadow-replay-execution-20260810'
ADAPTER_STAGE = 'report_only_raw_shadow_replay_adapter'
REQUIRED_AUTHORIZATION_DECISION = (
    'AUTHORIZE_V44_STAGE4_RAW_SHADOW_REPLAY_EXECUTION')
ADAPTER_AUTHORITY = {
    'raw_shadow_replay': True,
    'accuracy_or_reference_map_inputs': False,
    'primary_trajectory_or_map_mutation': False,
    'primary_bias_writeback': False,
    'ros_publication': False,
    'loop_or_global_correction': False,
    'diagnostic_evidence_output_only': True,
}
EVENT_KIND_ORDER = {'imu': 0, 'lidar': 1}
DIAGNOSTIC_FILENAME = 'diagnostics.jsonl'
RUN_REPORT_FILENAME = 'run.json'


class ContractError(ValueError):
    """A runtime input differs from the exact execution contract."""


class AuthorizationError(ContractError):
    """The separately sealed static gate does not authorize execution."""


class CapacityError(RuntimeError):
    """A bounded runtime resource would be exceeded."""


def require(condition: bool, message: str) -> None:
    if not bool(condition):
        raise ContractError(message)


def canonical_json(value: Any) -> str:
    return json.dumps(
        value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def payload_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode('utf-8')).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    def no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ContractError(f'duplicate JSON key: {key}')
            result[key] = value
        return result

    try:
        value = json.loads(
            path.read_text(encoding='utf-8'), object_pairs_hook=no_duplicates)
    except (OSError, UnicodeError, json.JSONDecodeError) as error:
        raise ContractError(f'cannot read JSON: {path}') from error
    if not isinstance(value, dict):
        raise ContractError(f'JSON root must be an object: {path}')
    return value


def exact_path(value: str) -> Path:
    path = Path(value)
    require(path.is_absolute(), 'contract-bound path must be absolute')
    require(path == path.resolve(), 'contract-bound path must be canonical')
    return path


def file_identity(path: Path) -> tuple[int, int, int, int, int]:
    status = path.stat()
    return (
        int(status.st_dev), int(status.st_ino), int(status.st_size),
        int(status.st_mtime_ns), int(status.st_mode))


def verify_bound_file(
        path: Path, expected_sha256: str, expected_bytes: int | None,
        label: str) -> str:
    require(path.is_file(), f'{label} is absent')
    require(not path.is_symlink(), f'{label} must not be a symlink')
    if expected_bytes is not None:
        require(path.stat().st_size == int(expected_bytes),
                f'{label} byte size differs')
    observed = sha256_file(path)
    require(observed == expected_sha256, f'{label} SHA-256 differs')
    return observed


def current_rss_mib() -> float:
    status_path = Path('/proc/self/status')
    if status_path.is_file():
        for line in status_path.read_text(encoding='utf-8').splitlines():
            if line.startswith('VmRSS:'):
                return float(line.split()[1]) / 1024.0
    maximum = float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
    return maximum / 1024.0


def header_stamp_ns(message: Any) -> int:
    try:
        stamp = message.header.stamp
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError) as error:
        raise ContractError('message has no valid header stamp') from error
    require(0 <= nanoseconds < 1_000_000_000,
            'header nanoseconds are outside [0, 1e9)')
    value = seconds * 1_000_000_000 + nanoseconds
    require(-(2 ** 63) <= value < 2 ** 63,
            'header timestamp is outside signed int64')
    return value


def _vector3(value: Any, label: str) -> tuple[float, float, float]:
    try:
        result = (float(value.x), float(value.y), float(value.z))
    except (AttributeError, TypeError, ValueError) as error:
        raise ContractError(f'{label} is not a three-vector') from error
    require(all(math.isfinite(item) for item in result),
            f'{label} contains a non-finite value')
    return result


def point_dtype(message_contract: Mapping[str, Any]) -> np.dtype:
    formats = {2: 'u1', 4: '<u2', 6: '<u4', 7: '<f4'}
    fields = message_contract['required_lidar_fields']
    try:
        return np.dtype({
            'names': [item['name'] for item in fields],
            'formats': [formats[int(item['datatype'])] for item in fields],
            'offsets': [int(item['offset']) for item in fields],
            'itemsize': int(message_contract['lidar_point_step_bytes']),
        })
    except (KeyError, TypeError, ValueError) as error:
        raise ContractError('unsupported canonical PointCloud2 schema') from error


def pointcloud_view(
        message: Any, message_contract: Mapping[str, Any], expected_frame: str,
        serialized_size_bytes: int, maximum_message_bytes: int) -> np.ndarray:
    require(0 < int(serialized_size_bytes) <= int(maximum_message_bytes),
            'PointCloud2 serialized size exceeds the bound')
    require(message.header.frame_id == expected_frame,
            'PointCloud2 frame differs from source binding')
    require(int(message.height) == int(message_contract['lidar_height']),
            'PointCloud2 height differs from canonical schema')
    require(bool(message.is_bigendian) is not bool(
        message_contract['lidar_little_endian']),
        'PointCloud2 endianness differs from canonical schema')
    point_step = int(message.point_step)
    width = int(message.width)
    height = int(message.height)
    require(point_step == int(message_contract['lidar_point_step_bytes']),
            'PointCloud2 point_step differs from canonical schema')
    require(width > 0 and height > 0, 'PointCloud2 dimensions must be positive')
    expected_fields = [
        (item['name'], int(item['offset']), int(item['datatype']),
         int(item['count']))
        for item in message_contract['required_lidar_fields']]
    try:
        observed_fields = [
            (item.name, int(item.offset), int(item.datatype), int(item.count))
            for item in message.fields]
    except (AttributeError, TypeError, ValueError) as error:
        raise ContractError('PointCloud2 field inventory is malformed') from error
    require(observed_fields == expected_fields,
            'PointCloud2 fields differ from canonical schema')
    row_step = int(message.row_step)
    require(row_step == width * point_step,
            'PointCloud2 rows must be contiguous')
    try:
        data = memoryview(message.data).cast('B')
    except (TypeError, ValueError) as error:
        raise ContractError('PointCloud2 data is not a byte buffer') from error
    require(len(data) == row_step * height,
            'PointCloud2 data size differs from dimensions')
    view = np.frombuffer(data, dtype=point_dtype(message_contract),
                         count=width * height)
    require(len(view) == width * height, 'PointCloud2 decode is incomplete')
    return view


def decode_imu_message(
        message: Any, source: Mapping[str, Any], core: Any,
        source_index: int, serialized_size_bytes: int,
        maximum_message_bytes: int) -> Any:
    require(0 < int(serialized_size_bytes) <= int(maximum_message_bytes),
            'IMU serialized size exceeds the bound')
    require(message.header.frame_id == source['imu_frame'],
            'IMU frame differs from source binding')
    return core.ImuSample(
        timestamp_ns=header_stamp_ns(message),
        angular_velocity_B_rad_s=_vector3(
            message.angular_velocity, 'IMU angular velocity'),
        linear_acceleration_B_m_s2=_vector3(
            message.linear_acceleration, 'IMU linear acceleration'),
        source_index=int(source_index),
        serialized_size_bytes=int(serialized_size_bytes))


def decode_lidar_message(
        message: Any, source: Mapping[str, Any], message_contract: Mapping[str, Any],
        core: Any, scan_index: int, serialized_size_bytes: int,
        maximum_message_bytes: int) -> Any:
    points = pointcloud_view(
        message, message_contract, source['lidar_frame'], serialized_size_bytes,
        maximum_message_bytes)
    records = tuple(
        core.LidarPoint(
            point_L_m=(float(point['x']), float(point['y']), float(point['z'])),
            offset_ns=int(point['t']), ring=int(point['ring']),
            source_index=index)
        for index, point in enumerate(points))
    return core.LidarScan(
        scan_index=int(scan_index), header_stamp_ns=header_stamp_ns(message),
        points=records, source_index=int(scan_index),
        serialized_size_bytes=int(serialized_size_bytes))


@dataclass(order=True)
class BufferedSensorMessage:
    """One decoded ROS message ordered only by the frozen sensor-time key."""

    sort_key: tuple[int, int, int]
    receive_stamp_ns: int = field(compare=False)
    kind: str = field(compare=False)
    source_index: int = field(compare=False)
    serialized_size_bytes: int = field(compare=False)
    message: Any = field(compare=False)


class DeterministicEventReorder:
    """Bounded receive-time watermark restoring exact header-time order."""

    def __init__(
            self, maximum_receive_delay_ns: int, maximum_buffered_messages: int,
            maximum_buffered_bytes: int) -> None:
        self.maximum_receive_delay_ns = int(maximum_receive_delay_ns)
        self.maximum_buffered_messages = int(maximum_buffered_messages)
        self.maximum_buffered_bytes = int(maximum_buffered_bytes)
        require(self.maximum_receive_delay_ns >= 0,
                'receive-delay bound must be nonnegative')
        require(self.maximum_buffered_messages > 0,
                'reorder message capacity must be positive')
        require(self.maximum_buffered_bytes > 0,
                'reorder byte capacity must be positive')
        self._heap: list[BufferedSensorMessage] = []
        self._buffered_bytes = 0
        self._last_receive_stamp_ns: int | None = None
        self._last_emitted_key: tuple[int, int, int] | None = None
        self.peak_messages = 0
        self.peak_bytes = 0

    def _emit(self) -> BufferedSensorMessage:
        event = heapq.heappop(self._heap)
        self._buffered_bytes -= int(event.serialized_size_bytes)
        if (self._last_emitted_key is not None
                and event.sort_key <= self._last_emitted_key):
            raise ContractError('reordered sensor event is duplicate or out of order')
        self._last_emitted_key = event.sort_key
        return event

    def push(self, event: BufferedSensorMessage) -> list[BufferedSensorMessage]:
        require(event.kind in EVENT_KIND_ORDER, 'unknown sensor event kind')
        require(event.sort_key == (
            int(event.sort_key[0]), EVENT_KIND_ORDER[event.kind],
            int(event.source_index)), 'sensor event sort key differs')
        receive = int(event.receive_stamp_ns)
        header = int(event.sort_key[0])
        if (self._last_receive_stamp_ns is not None
                and receive < self._last_receive_stamp_ns):
            raise ContractError('bag receive timestamps are not monotonic')
        delay = receive - header
        require(0 <= delay <= self.maximum_receive_delay_ns,
                'message receive-minus-header delay exceeds binding')
        next_messages = len(self._heap) + 1
        next_bytes = self._buffered_bytes + int(event.serialized_size_bytes)
        if next_messages > self.maximum_buffered_messages:
            raise CapacityError('reorder message capacity exceeded')
        if next_bytes > self.maximum_buffered_bytes:
            raise CapacityError('reorder byte capacity exceeded')
        heapq.heappush(self._heap, event)
        self._buffered_bytes = next_bytes
        self._last_receive_stamp_ns = receive
        self.peak_messages = max(self.peak_messages, len(self._heap))
        self.peak_bytes = max(self.peak_bytes, self._buffered_bytes)
        watermark = receive - self.maximum_receive_delay_ns
        ready: list[BufferedSensorMessage] = []
        while self._heap and self._heap[0].sort_key[0] < watermark:
            ready.append(self._emit())
        return ready

    def finish(self) -> list[BufferedSensorMessage]:
        result: list[BufferedSensorMessage] = []
        while self._heap:
            result.append(self._emit())
        return result


def validate_authorization_payload(
        aggregate: Mapping[str, Any], contract: Mapping[str, Any],
        contract_sha256: str, adapter_sha256: str) -> None:
    authorization = contract['authorization']
    require(aggregate.get('audit') == authorization['required_aggregate_audit'],
            'authorization aggregate audit ID differs')
    require(aggregate.get('contract_id') == contract['contract_id'],
            'authorization aggregate contract ID differs')
    require(aggregate.get('contract_sha256') == contract_sha256,
            'authorization aggregate contract SHA-256 differs')
    require(aggregate.get('adapter_sha256') == adapter_sha256,
            'authorization aggregate adapter SHA-256 differs')
    require(aggregate.get('auditor_sha256') == contract['static_auditor']['sha256'],
            'authorization aggregate auditor SHA-256 differs')
    if aggregate.get('status') != 'PASS':
        raise AuthorizationError('authorization aggregate did not pass')
    require(aggregate.get('decision') == REQUIRED_AUTHORIZATION_DECISION,
            'authorization aggregate decision differs')
    require(aggregate.get('aggregate_payload_sha256') == payload_sha256(
        aggregate.get('deterministic')),
        'authorization aggregate payload SHA-256 differs')
    require(aggregate.get('raw_shadow_replay_execution_authorized') is True,
            'raw shadow replay is not authorized')
    require(aggregate.get('raw_replay_executed') is False,
            'static authorization aggregate must not contain a replay')
    for key in (
            'accuracy_or_reference_map_inputs_authorized',
            'primary_trajectory_or_map_mutation_authorized',
            'ros_publication_authorized'):
        require(aggregate.get(key) is False,
                f'authorization aggregate unexpectedly opens {key}')


def validate_authorization_source_reports(
        aggregate: Mapping[str, Any], contract: Mapping[str, Any],
        contract_sha256: str, adapter_sha256: str) -> None:
    """Revalidate the two hash-listed static reports behind authorization."""
    require(contract['authorization'][
        'source_reports_revalidated_at_runtime'] is True,
        'runtime source-report revalidation policy differs')
    reports = aggregate.get('source_reports')
    require(isinstance(reports, list) and len(reports) == 2,
            'authorization source-report inventory differs')
    evidence_root = exact_path(contract['output']['evidence_root'])
    expected_paths = [
        evidence_root / 'run_01.json', evidence_root / 'run_02.json']
    expected_payload = aggregate['deterministic']['report_payload_sha256']
    repetitions = []
    for binding, expected_path in zip(reports, expected_paths):
        require(isinstance(binding, dict)
                and set(binding) == {'path', 'sha256'},
                'authorization source-report binding differs')
        report_path = exact_path(binding['path'])
        require(report_path == expected_path,
                'authorization source-report path differs')
        verify_bound_file(
            report_path, binding['sha256'], None,
            'v44e authorization source report')
        report = load_json(report_path)
        require(report.get('audit') ==
                'v44e_raw_shadow_replay_contract_static_validation',
                'authorization source-report audit ID differs')
        require(report.get('contract_id') == contract['contract_id']
                and report.get('contract_sha256') == contract_sha256,
                'authorization source-report contract binding differs')
        require(report.get('adapter_sha256') == adapter_sha256
                and report.get('auditor_sha256') ==
                contract['static_auditor']['sha256'],
                'authorization source-report implementation binding differs')
        require(report.get('status') == 'PASS'
                and report.get('decision') ==
                REQUIRED_AUTHORIZATION_DECISION,
                'authorization source report did not pass')
        deterministic = report.get('deterministic')
        require(isinstance(deterministic, dict)
                and report.get('report_payload_sha256') ==
                payload_sha256(deterministic) == expected_payload,
                'authorization source-report payload differs')
        require(deterministic.get('raw_replay_executed') is False
                and deterministic.get('raw_bag_opened') is False,
                'authorization source report opened or replayed raw data')
        require(report.get('raw_shadow_replay_execution_authorized') is True,
                'authorization source report keeps replay closed')
        repetitions.append(int(report.get('repetition', 0)))
    require(repetitions == [1, 2],
            'authorization source-report repetitions differ')


def validate_contract_for_runtime(
        contract: Mapping[str, Any], contract_path: Path) -> None:
    require(contract.get('schema_version') == 1,
            'unsupported execution contract schema')
    require(contract.get('contract_id') == EXECUTION_CONTRACT_ID,
            'execution contract ID differs')
    required_path = exact_path(contract['execution']['required_contract_path'])
    require(contract_path.resolve() == required_path,
            'execution contract path differs from sealed path')
    require(contract['authority'] == ADAPTER_AUTHORITY,
            'adapter authority differs from fail-closed contract')
    require(contract['execution']['raw_bag_open_allowed_only_after_authorization']
            is True, 'authorization-before-open policy differs')
    require(contract['execution']['required_repetitions_per_sequence'] == 2,
            'runtime repetition count differs')
    require(contract['output']['filenames'] == [
        DIAGNOSTIC_FILENAME, RUN_REPORT_FILENAME],
        'diagnostic output filename inventory differs')
    require(contract['output']['overwrite_allowed'] is False,
            'diagnostic overwrite must remain forbidden')
    require(contract['source_binding']['required_sequence_count'] == 3,
            'exactly three source bindings are required')
    sources = contract['source_binding']['sequences']
    require(len(sources) == 3, 'exactly three source records are required')
    require(len({item['sequence_id'] for item in sources}) == 3,
            'source sequence IDs must be unique')


def load_runtime_context(
        contract_path: Path) -> tuple[dict[str, Any], str, dict[str, Any], Any]:
    contract_path = contract_path.resolve()
    contract = load_json(contract_path)
    validate_contract_for_runtime(contract, contract_path)
    contract_digest = sha256_file(contract_path)
    adapter_path = Path(__file__).resolve()
    adapter_digest = sha256_file(adapter_path)
    require(adapter_digest == contract['adapter']['sha256'],
            'executing adapter SHA-256 differs from contract')
    require(adapter_path == exact_path(contract['adapter']['path']),
            'executing adapter path differs from contract')
    auditor_path = exact_path(contract['static_auditor']['path'])
    verify_bound_file(
        auditor_path, contract['static_auditor']['sha256'], None,
        'v44e static auditor')
    for item in contract['prerequisites'].values():
        if isinstance(item, dict) and {'path', 'sha256'} <= set(item):
            verify_bound_file(
                exact_path(item['path']), item['sha256'], item.get('bytes'),
                item.get('label', 'prerequisite'))
    aggregate_path = exact_path(
        contract['authorization']['required_aggregate_path'])
    aggregate = load_json(aggregate_path)
    validate_authorization_payload(
        aggregate, contract, contract_digest, adapter_digest)
    validate_authorization_source_reports(
        aggregate, contract, contract_digest, adapter_digest)
    core_binding = contract['estimator_core']
    core_path = exact_path(core_binding['path'])
    core_digest = verify_bound_file(
        core_path, core_binding['sha256'], core_binding.get('bytes'),
        'v44 estimator core')
    spec = importlib.util.spec_from_file_location(
        f'v44_shadow_core_{core_digest[:12]}', core_path)
    if spec is None or spec.loader is None:
        raise ContractError('cannot load v44 estimator core')
    core = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = core
    spec.loader.exec_module(core)
    return contract, contract_digest, aggregate, core


def source_binding(
        contract: Mapping[str, Any], sequence_id: str) -> dict[str, Any]:
    matches = [dict(item) for item in
               contract['source_binding']['sequences']
               if item['sequence_id'] == sequence_id]
    require(len(matches) == 1,
            'sequence ID is not one exact source binding')
    return matches[0]


def body_from_lidar(source: Mapping[str, Any], core: Any) -> Any:
    calibration = source['calibration']['body_from_lidar']
    rotation = tuple(tuple(float(value) for value in row)
                     for row in calibration['rotation'])
    translation = tuple(float(value)
                        for value in calibration['translation_m'])
    return core.BodyFromLidar(
        rotation_BL=rotation, translation_BL_m=translation)


def hash_protected_artifacts(source: Mapping[str, Any]) -> dict[str, str]:
    result: dict[str, str] = {}
    for item in source['protected_v17_artifacts']:
        name = str(item['name'])
        require(name not in result, 'protected artifact name is duplicated')
        result[name] = verify_bound_file(
            exact_path(item['path']), item['sha256'], int(item['bytes']),
            f'protected v17 {name}')
    require(set(result) == {'v17_state', 'v17_map'},
            'protected v17 artifact inventory differs')
    return dict(sorted(result.items()))


class BoundedEvidenceWriter:
    """Exclusive writer restricted to one exact diagnostic run directory."""

    def __init__(self, run_directory: Path, evidence_root: Path,
                 maximum_bytes: int) -> None:
        self.run_directory = run_directory
        self.evidence_root = evidence_root
        self.maximum_bytes = int(maximum_bytes)
        self.written_bytes = 0
        require(self.maximum_bytes > 0, 'evidence byte capacity must be positive')
        require(self.evidence_root.is_absolute(),
                'evidence root must be absolute')
        require(self.run_directory.is_absolute(),
                'run directory must be absolute')
        require(self.run_directory.parent.parent.parent == self.evidence_root,
                'run directory is outside the exact evidence layout')
        require(self.evidence_root.is_dir()
                and not self.evidence_root.is_symlink(),
                'evidence root must be an existing real directory')
        raw_root = self.evidence_root / 'raw_replay'
        for directory in (raw_root, self.run_directory.parent):
            if directory.exists():
                require(directory.is_dir() and not directory.is_symlink(),
                        'evidence parent must be a real directory')
            else:
                directory.mkdir()
        try:
            self.run_directory.mkdir(exist_ok=False)
        except FileExistsError as error:
            raise ContractError('diagnostic run directory already exists') from error

    def _write_exclusive(self, filename: str, payload: bytes) -> None:
        require(filename in {DIAGNOSTIC_FILENAME, RUN_REPORT_FILENAME},
                'diagnostic filename is not allowlisted')
        require(len(payload) <= self.maximum_bytes - self.written_bytes,
                'diagnostic evidence byte capacity exceeded')
        path = self.run_directory / filename
        try:
            with path.open('xb') as stream:
                stream.write(payload)
        except FileExistsError as error:
            raise ContractError('diagnostic output overwrite was refused') from error
        self.written_bytes += len(payload)

    def write_diagnostics(self, records: Sequence[Mapping[str, Any]]) -> None:
        payload = b''.join(
            (canonical_json(dict(record)) + '\n').encode('utf-8')
            for record in records)
        self._write_exclusive(DIAGNOSTIC_FILENAME, payload)

    def write_report(self, report: Mapping[str, Any]) -> None:
        payload = (json.dumps(
            dict(report), indent=2, sort_keys=True, allow_nan=False)
            + '\n').encode('utf-8')
        self._write_exclusive(RUN_REPORT_FILENAME, payload)


def expected_run_directory(
        contract: Mapping[str, Any], sequence_id: str, repetition: int) -> Path:
    root = exact_path(contract['output']['evidence_root'])
    required = int(contract['execution']['required_repetitions_per_sequence'])
    require(1 <= int(repetition) <= required,
            'repetition is outside the execution contract')
    require('/' not in sequence_id and sequence_id not in {'.', '..'},
            'sequence ID is not path-safe')
    return root / 'raw_replay' / sequence_id / f'run_{int(repetition):02d}'


def _new_scan_records(
        estimator: Any, observed: set[int]) -> list[dict[str, Any]]:
    result = []
    for record in estimator.diagnostics:
        if record.get('record_type') != 'scan':
            continue
        index = int(record['scan_index'])
        if index not in observed:
            result.append(record)
    return result


def record_runtime_after_scans(
        estimator: Any, observed: set[int], processing_started: float,
        sensor_origin_ns: int) -> None:
    for record in _new_scan_records(estimator, observed):
        scan_index = int(record['scan_index'])
        sensor_duration = (int(record['scan_end_ns']) - int(sensor_origin_ns)) / 1e9
        require(sensor_duration > 0.0,
                'per-scan sensor duration must be positive')
        estimator.record_runtime_observation(
            scan_index=scan_index, rss_mib=current_rss_mib(),
            processing_seconds=time.perf_counter() - processing_started,
            sensor_duration_seconds=sensor_duration)
        observed.add(scan_index)


def consume_buffered_event(
        event: BufferedSensorMessage, source: Mapping[str, Any],
        message_contract: Mapping[str, Any], core: Any, estimator: Any) -> int:
    maximum = int(message_contract['maximum_message_bytes'])
    if event.kind == 'imu':
        estimator.consume_imu(decode_imu_message(
            event.message, source, core, event.source_index,
            event.serialized_size_bytes, maximum))
        return 0
    scan = decode_lidar_message(
        event.message, source, message_contract, core, event.source_index,
        event.serialized_size_bytes, maximum)
    estimator.consume_lidar(scan)
    return len(scan.points)


def stream_raw_bag(
        contract: Mapping[str, Any], source: Mapping[str, Any], core: Any,
        estimator: Any) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
    except ModuleNotFoundError as error:
        raise ContractError('rosbags is required for authorized raw replay') from error
    bag_path = exact_path(source['bag']['path'])
    bag_identity_before = file_identity(bag_path)
    bag_digest = verify_bound_file(
        bag_path, source['bag']['sha256'], int(source['bag']['bytes']),
        'raw bag')
    reorder_contract = contract['reorder_buffer']
    reorder = DeterministicEventReorder(
        int(source['maximum_receive_minus_header_ns']),
        int(reorder_contract['maximum_buffered_messages']),
        int(reorder_contract['maximum_buffered_serialized_bytes']))
    message_contract = contract['messages']
    expected_counts = {
        'lidar': int(source['expected_lidar_messages']),
        'imu': int(source['expected_imu_messages']),
    }
    expected_digests = source['serialized_stream_sha256']
    counts = {'lidar': 0, 'imu': 0}
    point_count = 0
    digests = {'lidar': hashlib.sha256(), 'imu': hashlib.sha256()}
    observed_scans: set[int] = set()
    sensor_origin_ns: int | None = None
    processing_started = time.perf_counter()

    with AnyReader([bag_path]) as reader:
        connections: dict[str, Any] = {}
        for kind, topic, message_type in (
                ('lidar', source['lidar_topic'], message_contract['lidar_type']),
                ('imu', source['imu_topic'], message_contract['imu_type'])):
            matches = [item for item in reader.connections
                       if item.topic == topic]
            require(len(matches) == 1,
                    f'exactly one {kind} connection is required')
            require(matches[0].msgtype == message_type,
                    f'{kind} message type differs')
            require(int(matches[0].msgcount) == expected_counts[kind],
                    f'{kind} connection count differs')
            connections[kind] = matches[0]
        connection_kinds = {
            int(connection.id): kind
            for kind, connection in connections.items()}
        for connection, receive_stamp, serialized in reader.messages(
                connections=list(connections.values())):
            kind = connection_kinds[int(connection.id)]
            index = counts[kind]
            if index >= expected_counts[kind]:
                raise ContractError(f'too many {kind} messages')
            serialized_size = len(serialized)
            if serialized_size <= 0 or serialized_size > int(
                    message_contract['maximum_message_bytes']):
                raise CapacityError(f'{kind} serialized message exceeds bound')
            digests[kind].update(serialized)
            message = reader.deserialize(serialized, connection.msgtype)
            header = header_stamp_ns(message)
            if sensor_origin_ns is None:
                sensor_origin_ns = header
            else:
                sensor_origin_ns = min(sensor_origin_ns, header)
            event = BufferedSensorMessage(
                sort_key=(header, EVENT_KIND_ORDER[kind], index),
                receive_stamp_ns=int(receive_stamp), kind=kind,
                source_index=index, serialized_size_bytes=serialized_size,
                message=message)
            counts[kind] += 1
            for ready in reorder.push(event):
                point_count += consume_buffered_event(
                    ready, source, message_contract, core, estimator)
                record_runtime_after_scans(
                    estimator, observed_scans, processing_started,
                    int(sensor_origin_ns))
        for ready in reorder.finish():
            point_count += consume_buffered_event(
                ready, source, message_contract, core, estimator)
            record_runtime_after_scans(
                estimator, observed_scans, processing_started,
                int(sensor_origin_ns))

    require(counts == expected_counts, 'raw stream message counts differ')
    require(point_count == int(source['expected_lidar_points']),
            'raw stream LiDAR point count differs')
    observed_digests = {
        kind: digest.hexdigest() for kind, digest in digests.items()}
    require(observed_digests == expected_digests,
            'raw serialized stream SHA-256 differs')
    require(file_identity(bag_path) == bag_identity_before,
            'raw bag metadata changed during replay')
    require(sensor_origin_ns is not None, 'raw sensor stream is empty')
    return {
        'bag_sha256': bag_digest,
        'message_counts': counts,
        'lidar_point_count': point_count,
        'serialized_stream_sha256': observed_digests,
        'sensor_origin_ns': int(sensor_origin_ns),
        'processing_wall_seconds_before_finalize': (
            time.perf_counter() - processing_started),
        'reorder_peak_messages': reorder.peak_messages,
        'reorder_peak_serialized_bytes': reorder.peak_bytes,
        'runtime_observation_count': len(observed_scans),
        'peak_rss_mib_before_finalize': current_rss_mib(),
    }


def run_replay(
        contract_path: Path, sequence_id: str, repetition: int) -> dict[str, Any]:
    contract, contract_digest, aggregate, core = load_runtime_context(contract_path)
    source = source_binding(contract, sequence_id)
    output_directory = expected_run_directory(
        contract, sequence_id, repetition)
    writer = BoundedEvidenceWriter(
        output_directory, exact_path(contract['output']['evidence_root']),
        int(contract['output']['maximum_bytes_per_run']))
    protected_before = hash_protected_artifacts(source)
    configuration = core.FixedLagShadowConfig.from_architecture(
        load_json(exact_path(contract['prerequisites']['architecture']['path'])))
    estimator = core.FixedLagShadowEstimator(
        configuration, body_from_lidar(source, core),
        core.ProtectedOutputGuard(protected_before))
    stream_result: dict[str, Any] | None = None
    runtime_error: str | None = None
    try:
        stream_result = stream_raw_bag(contract, source, core, estimator)
    except Exception as error:  # retained in the bounded terminal evidence
        runtime_error = f'{type(error).__name__}: {error}'
        if not estimator.failed:
            estimator._fail(runtime_error)
    protected_after = hash_protected_artifacts(source)
    core_result = estimator.finalize(protected_after)
    status = 'PASS' if (
        runtime_error is None and core_result.get('status') == 'PASS'
        and protected_before == protected_after) else 'FAIL'
    deterministic = {
        'sequence_id': sequence_id,
        'repetition': int(repetition),
        'source_set_id': contract['source_binding']['source_set_id'],
        'bag_sha256': source['bag']['sha256'],
        'protected_v17_before': protected_before,
        'protected_v17_after': protected_after,
        'protected_v17_unchanged': protected_before == protected_after,
        'core_status': core_result.get('status'),
        'core_state_payload_sha256': core_result.get('state_payload_sha256'),
        'core_diagnostic_payload_sha256': core_result.get(
            'diagnostic_payload_sha256'),
        'core_valid_shadow_result': core_result.get('valid_shadow_result'),
        'runtime_error': runtime_error,
        'stream_identity': None if stream_result is None else {
            'message_counts': stream_result['message_counts'],
            'serialized_stream_sha256': stream_result[
                'serialized_stream_sha256'],
        },
        'authority': dict(ADAPTER_AUTHORITY),
    }
    report = {
        'schema_version': 1,
        'stage': ADAPTER_STAGE,
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_digest,
        'authorization_aggregate_sha256': sha256_file(exact_path(
            contract['authorization']['required_aggregate_path'])),
        'authorization_decision': aggregate['decision'],
        'adapter_sha256': sha256_file(Path(__file__).resolve()),
        'sequence_id': sequence_id,
        'repetition': int(repetition),
        'status': status,
        'raw_replay_executed': True,
        'accuracy_or_reference_map_inputs_accessed': False,
        'primary_trajectory_or_map_mutated': False,
        'ros_output_published': False,
        'deterministic': deterministic,
        'report_payload_sha256': payload_sha256(deterministic),
        'stream': stream_result,
        'core_result': core_result,
        'evidence_bytes_before_report': writer.written_bytes,
    }
    writer.write_diagnostics(estimator.diagnostics)
    report['evidence_bytes_before_report'] = writer.written_bytes
    writer.write_report(report)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    replay = subparsers.add_parser(
        'replay', help='run one authorized report-only shadow repetition')
    replay.add_argument('--contract', required=True, type=Path)
    replay.add_argument('--sequence-id', required=True)
    replay.add_argument('--repetition', required=True, type=int)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    report = run_replay(
        arguments.contract, arguments.sequence_id, arguments.repetition)
    print(json.dumps({
        'status': report['status'], 'sequence_id': report['sequence_id'],
        'repetition': report['repetition'],
        'report_payload_sha256': report['report_payload_sha256'],
    }, sort_keys=True))
    return 0 if report['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
