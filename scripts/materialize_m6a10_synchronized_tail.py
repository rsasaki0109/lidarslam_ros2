#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Materialize a synchronized ROS 2 input from a read-only analyzer receipt.

The analyzer is the only component allowed to select terminal LiDAR records.
This command copies raw serialized records in their original order, drops only
the exact terminal-row identities in that receipt, and publishes a new bag by
an atomic directory rename.  The receipt is deliberately outside the output
bag so its hash cannot be part of the bag-tree hash.  Ground truth and scoring
are never opened or invoked.

ROS 1/FAST-LIVO2 is intentionally not converted here.  A prior pinned
``rosbags-convert`` run must provide a passing semantic-equivalence report;
``--ros1-semantic-report`` validates that report without opening either bag.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, is_dataclass
from enum import Enum
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import sys
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from lidarslam_benchmark_tools.analyze_m6a10_synchronized_tail import (  # noqa: E402
    CONTRACT_ID as ANALYZER_CONTRACT_ID,
    summarize_lidar_message,
    SUPPORTED_LIDAR_TYPE,
    SynchronizedTailError,
)


SCHEMA_VERSION = 1
RECEIPT_KIND = 'm6a10_v2a_synchronized_tail_generation'
MATERIALIZATION_CONTRACT_ID = 'm6a10-v2a-synchronized-tail-materialization-v1'
ORDER_DEFINITION = (
    'anyreader_chronological_playback_order_including_deterministic_ties')
SEMANTIC_REPORT_SCHEMA = 1
ROSBAGS_CONVERT = 'rosbags-convert'
ROSBAGS_CONVERT_VERSION = '0.11.0'
SEMANTIC_COMPARATOR = SCRIPTS / 'compare_rosbag_semantic_inputs.py'
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
TAIL_KEY_FIELDS = (
    'header_timestamp_ns',
    'point_timestamp_min_ns',
    'point_timestamp_max_ns',
    'storage_timestamp_ns',
    'payload_sha256',
)


class MaterializationError(ValueError):
    """Raised when the synchronized-tail generation contract is not proven."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_file(path: Path) -> str:
    """Public file hash helper used by tests and receipts."""
    return _sha256_file(path)


def sha256_tree(root: Path) -> dict[str, Any]:
    """Hash every regular file without following symlinks."""
    root = root.resolve(strict=True)
    if not root.is_dir() or root.is_symlink():
        raise MaterializationError('tree_not_directory', f'not a regular tree: {root}')
    digest = hashlib.sha256()
    files = 0
    total_bytes = 0
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise MaterializationError('symlink_tree', f'symlink in tree: {path}')
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix().encode('utf-8')
        size = path.stat().st_size
        digest.update(relative)
        digest.update(b'\0')
        digest.update(str(size).encode('ascii'))
        digest.update(b'\0')
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        files += 1
        total_bytes += size
    if files == 0:
        raise MaterializationError('empty_tree', f'tree has no files: {root}')
    return {'sha256': digest.hexdigest(), 'file_count': files, 'total_bytes': total_bytes}


def _jsonable(value: Any) -> Any:
    """Convert rosbags namedtuples/enums to a deterministic JSON value."""
    if is_dataclass(value):
        return _jsonable(asdict(value))
    if isinstance(value, tuple):
        return [_jsonable(item) for item in value]
    if isinstance(value, list):
        return [_jsonable(item) for item in value]
    if isinstance(value, dict):
        return {str(key): _jsonable(item) for key, item in sorted(value.items())}
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if hasattr(value, '_asdict'):
        return _jsonable(value._asdict())
    raise MaterializationError('non_json_metadata', f'unsupported metadata: {type(value)!r}')


def _require_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
        raise MaterializationError('invalid_sha256', f'{label} must be a 64-hex SHA-256')
    return value.lower()


def _resolve_regular(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.exists():
        raise MaterializationError('unsafe_path', f'{label} is missing or a symlink: {path}')
    return path.resolve(strict=True)


def _is_under(path: Path, root: Path) -> bool:
    try:
        path.resolve(strict=False).relative_to(root.resolve(strict=False))
    except ValueError:
        return False
    return True


def assert_output_paths(input_root: Path, output_root: Path, receipt_path: Path) -> None:
    """Reject collisions, containment, symlinks, and stale staging artifacts."""
    input_root = _resolve_regular(input_root, 'input bag')
    output_root = output_root.expanduser().resolve(strict=False)
    receipt_path = receipt_path.expanduser().resolve(strict=False)
    if output_root.exists() or output_root.is_symlink():
        raise MaterializationError('output_exists', f'output already exists: {output_root}')
    legacy_stage = output_root.with_name(output_root.name + '.part')
    if legacy_stage.exists() or legacy_stage.is_symlink():
        raise MaterializationError(
            'output_part_exists', f'legacy output staging exists: {legacy_stage}')
    staging_container = output_root.with_name(output_root.name + '.staging')
    if staging_container.exists() or staging_container.is_symlink():
        raise MaterializationError(
            'output_staging_exists',
            f'output staging container exists: {staging_container}')
    if receipt_path.exists() or receipt_path.is_symlink():
        raise MaterializationError('receipt_exists', f'receipt already exists: {receipt_path}')
    receipt_part = receipt_path.with_name(receipt_path.name + '.part')
    if receipt_part.exists() or receipt_part.is_symlink():
        raise MaterializationError(
            'receipt_part_exists', f'receipt staging exists: {receipt_part}')
    if _is_under(output_root, input_root) or _is_under(input_root, output_root):
        raise MaterializationError('path_overlap', 'input and output trees overlap')
    if _is_under(receipt_path, input_root) or _is_under(receipt_path, output_root):
        raise MaterializationError(
            'receipt_inside_tree', 'receipt must be outside input/output trees')
    if output_root.parent == input_root or _is_under(output_root.parent, input_root):
        # A sibling output under the input directory can be hidden from the
        # source tree hash and is therefore not an acceptable destination.
        raise MaterializationError(
            'output_parent_inside_input', 'output parent is inside input tree')
    if not output_root.parent.exists():
        output_root.parent.mkdir(parents=True, exist_ok=True)
    if not output_root.parent.is_dir() or output_root.parent.is_symlink():
        raise MaterializationError(
            'output_parent_unsafe', 'output parent is not a regular directory')


def _tail_key(row: dict[str, Any]) -> tuple[Any, ...]:
    values = tuple(row.get(field) for field in TAIL_KEY_FIELDS)
    if not all(isinstance(value, int) for value in values[:4]):
        raise MaterializationError(
            'invalid_terminal_row', 'terminal timestamp identity is invalid')
    if not isinstance(values[4], str) or SHA256_RE.fullmatch(values[4]) is None:
        raise MaterializationError('invalid_terminal_row', 'terminal payload identity is invalid')
    return values


def _tail_key_json(key: tuple[Any, ...]) -> dict[str, Any]:
    return dict(zip(TAIL_KEY_FIELDS, key))


def validate_analyzer_receipt(
    receipt: dict[str, Any],
    input_root: Path,
    *,
    analyzer_path: Path | None = None,
) -> dict[str, Any]:
    """Validate analyzer provenance and return the exact drop-key set."""
    if not isinstance(receipt, dict):
        raise MaterializationError('analyzer_schema', 'analyzer receipt must be an object')
    if receipt.get('schema_version') != 1 or receipt.get('contract_id') != ANALYZER_CONTRACT_ID:
        raise MaterializationError('analyzer_schema', 'analyzer contract is not supported')
    if receipt.get('status') != 'PASS' or receipt.get('mode') != 'dry_run':
        raise MaterializationError('analyzer_status', 'analyzer receipt is not a PASS dry run')
    safety = receipt.get('safety')
    if not isinstance(safety, dict) or any(
            safety.get(field) is not False for field in (
                'ground_truth_content_opened', 'scorer_invoked', 'output_bag_written')):
        raise MaterializationError('analyzer_safety', 'analyzer safety flags are not blind')
    input_info = receipt.get('input')
    if not isinstance(input_info, dict):
        raise MaterializationError('analyzer_input', 'analyzer input identity is missing')
    input_root = _resolve_regular(input_root, 'input bag')
    if Path(str(input_info.get('path', ''))).expanduser().resolve(strict=False) != input_root:
        raise MaterializationError(
            'analyzer_input', 'analyzer input path differs from generator input')
    tree = input_info.get('tree')
    if not isinstance(tree, dict):
        raise MaterializationError('analyzer_input', 'analyzer input tree identity is missing')
    expected_tree = _require_sha(tree.get('sha256'), 'analyzer input tree sha256')
    observed_tree = sha256_tree(input_root)
    if observed_tree['sha256'] != expected_tree:
        raise MaterializationError('analyzer_input_changed', 'input tree changed after analysis')
    generated = receipt.get('generated_by')
    if not isinstance(generated, dict):
        raise MaterializationError('analyzer_provenance', 'analyzer generated_by is missing')
    actual_analyzer = analyzer_path or (ROOT / 'scripts' / 'analyze_m6a10_synchronized_tail.py')
    actual_analyzer = _resolve_regular(actual_analyzer, 'analyzer script')
    if generated.get('path') != 'scripts/analyze_m6a10_synchronized_tail.py':
        raise MaterializationError('analyzer_provenance', 'unexpected analyzer path')
    if generated.get('sha256') != _sha256_file(actual_analyzer):
        raise MaterializationError('analyzer_provenance', 'analyzer script hash changed')
    evaluation = receipt.get('tail_evaluation')
    if not isinstance(evaluation, dict) or evaluation.get('comparator') != (
            'point_timestamp_max_ns < last_imu_header_timestamp_ns'):
        raise MaterializationError(
            'analyzer_contract', 'strict synchronized-tail comparator missing')
    if evaluation.get('comparison_is_strict') is not True:
        raise MaterializationError('analyzer_contract', 'strict comparison flag is false')
    if evaluation.get('ineligible_lidar_count', 0) < 0:
        raise MaterializationError('analyzer_contract', 'invalid ineligible count')
    terminal = evaluation.get('terminal_lidar')
    if not isinstance(terminal, list):
        raise MaterializationError('analyzer_contract', 'terminal_lidar is not a list')
    keys: list[tuple[Any, ...]] = []
    for row in terminal:
        if not isinstance(row, dict):
            raise MaterializationError('analyzer_contract', 'terminal row is not an object')
        key = _tail_key(row)
        if key in keys:
            raise MaterializationError('duplicate_terminal_row', 'terminal rows are ambiguous')
        keys.append(key)
    if len(keys) != evaluation.get('ineligible_lidar_count'):
        raise MaterializationError(
            'analyzer_contract', 'terminal row count does not match receipt')
    return {
        'drop_keys': frozenset(keys),
        'drop_rows': tuple(_tail_key_json(key) for key in keys),
        'input_tree': observed_tree,
        'input_tree_sha256': observed_tree['sha256'],
        'analyzer_sha256': _sha256_file(actual_analyzer),
        'contract_id': receipt['contract_id'],
    }


def select_records(
    records: Iterable[dict[str, Any]],
    drop_keys: Iterable[tuple[Any, ...]],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Select records using only precomputed analyzer terminal identities.

    This pure helper is used by synthetic tests.  A LiDAR record must carry a
    ``tail_key`` produced by the analyzer; no timestamp policy is inferred
    here.  Every requested key must be seen exactly once by the caller's
    source stream.
    """
    requested = set(drop_keys)
    seen: set[tuple[Any, ...]] = set()
    kept: list[dict[str, Any]] = []
    dropped: list[dict[str, Any]] = []
    for record in records:
        key = record.get('tail_key')
        if key in requested:
            if key in seen:
                raise MaterializationError(
                    'duplicate_terminal_record', 'terminal row occurred twice')
            seen.add(key)
            dropped.append(record)
        else:
            kept.append(record)
    missing = requested.difference(seen)
    if missing:
        raise MaterializationError(
            'terminal_record_missing', 'analyzer terminal row was not found')
    return kept, dropped


def _record_fields(record: dict[str, Any]) -> tuple[Any, ...]:
    return (
        record.get('topic'), record.get('msgtype'), record.get('timestamp_ns'),
        bytes(record.get('raw', b'')),
    )


def _update_ordered_digest(digest: Any, record: dict[str, Any]) -> None:
    topic = str(record.get('topic', '')).encode('utf-8')
    msgtype = str(record.get('msgtype', '')).encode('utf-8')
    timestamp = int(record.get('timestamp_ns'))
    raw = bytes(record.get('raw', b''))
    digest.update(struct.pack('<Q', len(topic)))
    digest.update(topic)
    digest.update(struct.pack('<Q', len(msgtype)))
    digest.update(msgtype)
    digest.update(struct.pack('<qQ', timestamp, len(raw)))
    digest.update(raw)


class _StreamAccumulator:
    """Bounded-memory ordered payload/count accumulator."""

    def __init__(self) -> None:
        self.count = 0
        self.aggregate = hashlib.sha256()
        self.per_topic: dict[str, dict[str, Any]] = {}

    def add(self, record: dict[str, Any]) -> None:
        _update_ordered_digest(self.aggregate, record)
        topic = str(record['topic'])
        topic_state = self.per_topic.setdefault(
            topic, {'count': 0, 'aggregate': hashlib.sha256()})
        _update_ordered_digest(topic_state['aggregate'], record)
        topic_state['count'] += 1
        self.count += 1

    def document(self) -> dict[str, Any]:
        return {
            topic: {
                'count': state['count'],
                'ordered_payload_stream_sha256': state['aggregate'].hexdigest(),
            }
            for topic, state in sorted(self.per_topic.items())
        }

    def digest(self) -> str:
        return self.aggregate.hexdigest()


def verify_record_stream(
    expected: Iterable[dict[str, Any]],
    actual: Iterable[dict[str, Any]],
    *,
    connection_id_map: dict[int, int] | None = None,
) -> None:
    """Fail closed on record order, timestamp, topic/type, or raw payload drift."""
    expected_iter = iter(expected)
    actual_iter = iter(actual)
    index = 0
    while True:
        try:
            left = next(expected_iter)
        except StopIteration:
            try:
                next(actual_iter)
            except StopIteration:
                return
            raise MaterializationError('record_count_mismatch', 'output has extra records')
        try:
            right = next(actual_iter)
        except StopIteration as exc:
            raise MaterializationError(
                'record_count_mismatch', 'output record count changed') from exc
        if _record_fields(left) != _record_fields(right):
            raise MaterializationError(
                'record_stream_mismatch', f'record mismatch at index {index}')
        if connection_id_map is not None:
            if connection_id_map.get(left.get('connection_id')) != right.get('connection_id'):
                raise MaterializationError(
                    'connection_stream_mismatch', f'connection mismatch at index {index}')
        index += 1


def ordered_payload_stream_sha256(records: Iterable[dict[str, Any]]) -> str:
    """Hash ordered topic/type/timestamp/raw payload tuples unambiguously."""
    digest = hashlib.sha256()
    for record in records:
        _update_ordered_digest(digest, record)
    return digest.hexdigest()


def _stats(records: Iterable[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    accumulator = _StreamAccumulator()
    for record in records:
        accumulator.add(record)
    return accumulator.document()


def _connection_identity(connection: Any, *, include_id: bool = True) -> dict[str, Any]:
    ext = getattr(connection, 'ext', None)
    serialization = getattr(ext, 'serialization_format', None)
    qos = getattr(ext, 'offered_qos_profiles', None)
    msgdef = getattr(connection, 'msgdef', None)
    msgdef_data = getattr(msgdef, 'data', None)
    if not isinstance(connection.topic, str) or not isinstance(connection.msgtype, str):
        raise MaterializationError('connection_metadata', 'connection topic/type is invalid')
    if not isinstance(serialization, str) or not isinstance(qos, (list, tuple)):
        raise MaterializationError('connection_metadata', 'rosbag2 QoS metadata is unavailable')
    if not isinstance(msgdef_data, str) or not isinstance(connection.digest, str):
        raise MaterializationError('connection_metadata', 'message definition/hash is unavailable')
    identity = {
        'topic': connection.topic,
        'msgtype': connection.msgtype,
        'serialization_format': serialization,
        'offered_qos_profiles': _jsonable(qos),
        'rihs01': connection.digest,
        'msgdef_sha256': hashlib.sha256(msgdef_data.encode('utf-8')).hexdigest(),
    }
    if include_id:
        identity['connection_id'] = int(connection.id)
    return identity


def connection_identity(connection: Any) -> dict[str, Any]:
    """Public connection metadata identity helper."""
    return _connection_identity(connection)


def _connection_signature(identity: dict[str, Any]) -> str:
    payload = dict(identity)
    payload.pop('connection_id', None)
    return json.dumps(payload, sort_keys=True, separators=(',', ':'))


def _assert_unique_connections(connections: Iterable[Any]) -> list[dict[str, Any]]:
    identities = [_connection_identity(connection) for connection in connections]
    signatures = [_connection_signature(identity) for identity in identities]
    if len(signatures) != len(set(signatures)):
        raise MaterializationError(
            'duplicate_connection_signature',
            'rosbag2 Writer cannot preserve duplicate identical connection metadata',
        )
    return identities


def validate_ros1_semantic_report(path: Path) -> dict[str, Any]:
    """Validate a prior ROS1→ROS2 semantic report without opening a bag."""
    path = _resolve_regular(path, 'ROS1 semantic report')
    try:
        report = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise MaterializationError(
            'semantic_report_invalid', 'semantic report is unreadable') from exc
    if not isinstance(report, dict) or report.get('schema_version') != SEMANTIC_REPORT_SCHEMA:
        raise MaterializationError(
            'semantic_report_invalid', 'semantic report schema is unsupported')
    topics = report.get('topics')
    if report.get('all_topics_equal') is not True or not isinstance(topics, list) or not topics:
        raise MaterializationError(
            'semantic_report_mismatch', 'ROS1/ROS2 semantic report is not PASS')
    if any(not isinstance(topic, dict) or topic.get('equal') is not True for topic in topics):
        raise MaterializationError(
            'semantic_report_mismatch', 'a semantic topic comparison failed')
    return {
        'status': 'PASS',
        'path': str(path),
        'sha256': _sha256_file(path),
        'schema_version': SEMANTIC_REPORT_SCHEMA,
    }


def build_ros1_conversion_argv(ros2_input: Path, ros1_stage: Path) -> list[str]:
    """Return the pinned ROS 2→ROS 1 conversion command without executing it.

    FAST-LIVO2 keeps its ROS 1 transport as a separate, explicit input.  The
    command is a pure builder so callers can bind it into a preregistered
    receipt and run it in an isolated staging directory later.  This slice
    intentionally does not invoke the converter or create the staging path.
    """
    ros2_input = Path(ros2_input).expanduser().resolve(strict=False)
    ros1_stage = Path(ros1_stage).expanduser().resolve(strict=False)
    if ros1_stage.exists() or ros1_stage.is_symlink():
        raise MaterializationError('conversion_output_exists', 'ROS1 staging path already exists')
    if _is_under(ros1_stage, ros2_input) or _is_under(ros2_input, ros1_stage):
        raise MaterializationError('conversion_path_overlap', 'ROS1 conversion paths overlap')
    return [
        ROSBAGS_CONVERT,
        '--src', str(ros2_input),
        '--dst', str(ros1_stage),
        '--compress', 'none',
        '--src-typestore', 'ros2_humble',
        '--dst-typestore', 'ros1_noetic',
    ]


def build_semantic_comparator_argv(
    ros1_input: Path,
    ros2_input: Path,
    topics: Iterable[str],
    output_report: Path,
) -> list[str]:
    """Return the existing cross-transport semantic comparator command."""
    ros1_input = Path(ros1_input).expanduser().resolve(strict=False)
    ros2_input = Path(ros2_input).expanduser().resolve(strict=False)
    output_report = Path(output_report).expanduser().resolve(strict=False)
    if output_report.exists() or output_report.is_symlink():
        raise MaterializationError('semantic_output_exists', 'semantic report path already exists')
    topic_list = sorted({str(topic) for topic in topics})
    if not topic_list:
        raise MaterializationError(
            'semantic_topics_missing', 'at least one semantic topic is required')
    return [
        sys.executable,
        str(SEMANTIC_COMPARATOR),
        '--left', str(ros1_input),
        '--right', str(ros2_input),
        *sum((['--topic', topic] for topic in topic_list), []),
        '--output', str(output_report),
    ]


def _atomic_write_json(path: Path, document: dict[str, Any]) -> None:
    path = path.expanduser().resolve(strict=False)
    if path.exists() or path.is_symlink():
        raise MaterializationError('receipt_exists', f'receipt already exists: {path}')
    part = path.with_name(path.name + '.part')
    if part.exists() or part.is_symlink():
        raise MaterializationError('receipt_part_exists', f'receipt staging exists: {part}')
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (
        json.dumps(document, sort_keys=True, separators=(',', ':')) + '\n'
    ).encode('utf-8')
    with part.open('xb') as stream:
        stream.write(encoded)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(part, path)
    directory_fd = os.open(path.parent, os.O_RDONLY)
    try:
        os.fsync(directory_fd)
    finally:
        os.close(directory_fd)


def _metadata_version(input_root: Path) -> int:
    try:
        import yaml
        metadata = yaml.safe_load((input_root / 'metadata.yaml').read_text(encoding='utf-8'))
        version = metadata['rosbag2_bagfile_information']['version']
        version = int(version)
    except (KeyError, OSError, TypeError, ValueError, ImportError) as exc:
        raise MaterializationError(
            'metadata_invalid', 'rosbag2 metadata version is unavailable') from exc
    if version not in (8, 9):
        raise MaterializationError(
            'metadata_invalid', f'unsupported rosbag2 metadata version: {version}')
    return version


def _record_from_message(connection: Any, timestamp: int, raw: bytes) -> dict[str, Any]:
    return {
        'connection_id': int(connection.id),
        'topic': connection.topic,
        'msgtype': connection.msgtype,
        'timestamp_ns': int(timestamp),
        'raw': bytes(raw),
    }


def _lidar_tail_key(connection: Any, timestamp: int, raw: bytes, reader: Any) -> tuple[Any, ...]:
    try:
        message = reader.deserialize(raw, connection.msgtype)
        summary = summarize_lidar_message(
            message, int(timestamp), serialized_payload=bytes(raw), source_index=0)
    except SynchronizedTailError as exc:
        raise MaterializationError('lidar_schema', str(exc)) from exc
    return _tail_key(summary)


def _verify_bag_stream(
    input_root: Path,
    output_root: Path,
    lidar_topic: str,
    drop_keys: frozenset[tuple[Any, ...]],
) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as exc:  # pragma: no cover - runtime dependency
        raise MaterializationError('rosbags_missing', 'rosbags is required') from exc
    dropped: set[tuple[Any, ...]] = set()
    source_connections: list[dict[str, Any]]
    output_connections: list[dict[str, Any]]
    with AnyReader([input_root], default_typestore=get_typestore(Stores.LATEST)) as source_reader:
        source_connections_raw = list(source_reader.connections)
        source_connections = _assert_unique_connections(source_connections_raw)
        with AnyReader(
                [output_root], default_typestore=get_typestore(Stores.LATEST)) as output_reader:
            output_connections_raw = list(output_reader.connections)
            output_connections = _assert_unique_connections(output_connections_raw)
            source_signatures = [_connection_signature(item) for item in source_connections]
            output_signatures = [_connection_signature(item) for item in output_connections]
            if source_signatures != output_signatures:
                raise MaterializationError(
                    'connection_metadata_mismatch',
                    'output connection metadata changed')
            source_to_output = {
                int(source['connection_id']): int(output['connection_id'])
                for source, output in zip(source_connections, output_connections)
            }
            output_accumulator = _StreamAccumulator()
            output_messages = iter(output_reader.messages())
            for index, (connection, timestamp, raw) in enumerate(
                    source_reader.messages()):
                record = _record_from_message(connection, timestamp, raw)
                if connection.topic == lidar_topic:
                    key = _lidar_tail_key(connection, timestamp, raw, source_reader)
                    if key in drop_keys:
                        if key in dropped:
                            raise MaterializationError(
                                'duplicate_terminal_record', 'terminal row repeated')
                        dropped.add(key)
                        continue
                    record['tail_key'] = key
                try:
                    actual_connection, actual_timestamp, actual_raw = next(output_messages)
                except StopIteration as exc:
                    raise MaterializationError(
                        'record_count_mismatch',
                        f'output ended before source record {index}') from exc
                actual = _record_from_message(actual_connection, actual_timestamp, actual_raw)
                if _record_fields(record) != _record_fields(actual):
                    raise MaterializationError(
                        'record_stream_mismatch', f'record mismatch at index {index}')
                if source_to_output.get(record['connection_id']) != actual['connection_id']:
                    raise MaterializationError(
                        'connection_stream_mismatch',
                        f'connection mismatch at index {index}')
                output_accumulator.add(actual)
            try:
                next(output_messages)
            except StopIteration:
                pass
            else:
                raise MaterializationError('record_count_mismatch', 'output has extra records')
            if dropped != set(drop_keys):
                raise MaterializationError(
                    'terminal_record_missing',
                    'not every analyzer drop row was found')
    return {
        'output_records': output_accumulator.count,
        'output_topics': output_accumulator.document(),
        'output_ordered_payload_stream_sha256': output_accumulator.digest(),
        'connection_id_map': source_to_output,
        'connections': source_connections,
        'output_connections': output_connections,
        'dropped_count': len(dropped),
    }


def materialize_bag(
    input_root: Path,
    output_root: Path,
    receipt_path: Path,
    analyzer_receipt_path: Path,
    *,
    ros1_semantic_report: Path | None = None,
) -> dict[str, Any]:
    """Materialize and independently verify a synchronized ROS2 bag."""
    input_root = _resolve_regular(input_root, 'input bag')
    analyzer_receipt_path = _resolve_regular(analyzer_receipt_path, 'analyzer receipt')
    receipt_path = receipt_path.expanduser().resolve(strict=False)
    output_root = output_root.expanduser().resolve(strict=False)
    assert_output_paths(input_root, output_root, receipt_path)
    try:
        analyzer_receipt = json.loads(analyzer_receipt_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise MaterializationError(
            'analyzer_unreadable', 'analyzer receipt is unreadable') from exc
    analyzer_identity = validate_analyzer_receipt(analyzer_receipt, input_root)
    lidar_topic = analyzer_receipt['topics']['lidar']['topic']
    if analyzer_receipt['topics']['lidar']['type'] != SUPPORTED_LIDAR_TYPE:
        raise MaterializationError('lidar_type_contract', 'analyzer lidar type is unsupported')
    semantic_identity = None
    if ros1_semantic_report is not None:
        semantic_identity = validate_ros1_semantic_report(ros1_semantic_report)
    version = _metadata_version(input_root)
    staging_container = output_root.with_name(output_root.name + '.staging')
    stage = staging_container / output_root.name
    staging_container.mkdir()
    input_accumulator = _StreamAccumulator()
    output_accumulator = _StreamAccumulator()
    dropped_rows: list[dict[str, Any]] = []
    dropped_keys_seen: set[tuple[Any, ...]] = set()
    source_connections: list[dict[str, Any]] = []
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import StoragePlugin, Writer
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as exc:  # pragma: no cover - runtime dependency
        raise MaterializationError('rosbags_missing', 'rosbags is required') from exc
    with AnyReader([input_root], default_typestore=get_typestore(Stores.LATEST)) as reader:
        connections = list(reader.connections)
        source_connections = _assert_unique_connections(connections)
        connection_map: dict[int, Any] = {}
        with Writer(stage, version=version, storage_plugin=StoragePlugin.SQLITE3) as writer:
            for connection in connections:
                ext = connection.ext
                connection_map[connection.id] = writer.add_connection(
                    connection.topic,
                    connection.msgtype,
                    msgdef=connection.msgdef.data,
                    rihs01=connection.digest,
                    serialization_format=ext.serialization_format,
                    offered_qos_profiles=list(ext.offered_qos_profiles),
                )
            for connection, timestamp, raw in reader.messages():
                raw = bytes(raw)
                record = {
                    'connection_id': int(connection.id),
                    'topic': connection.topic,
                    'msgtype': connection.msgtype,
                    'timestamp_ns': int(timestamp),
                    'raw': raw,
                }
                input_accumulator.add(record)
                if connection.topic == lidar_topic:
                    key = _lidar_tail_key(connection, timestamp, raw, reader)
                    if key in analyzer_identity['drop_keys']:
                        if key in dropped_keys_seen:
                            raise MaterializationError(
                                'duplicate_terminal_record', 'terminal row repeated')
                        record['tail_key'] = key
                        dropped_keys_seen.add(key)
                        dropped_rows.append(_tail_key_json(key))
                        continue
                    record['tail_key'] = key
                writer.write(connection_map[connection.id], int(timestamp), raw)
                output_accumulator.add(record)
        if dropped_keys_seen != set(analyzer_identity['drop_keys']):
            raise MaterializationError(
                'terminal_record_missing', 'analyzer terminal row was not found')
        # Verify the staged bag before publishing it.  This rereads raw CDR and
        # metadata, and therefore catches Writer metadata/payload drift.
        staged_verification = _verify_bag_stream(
            input_root, stage, lidar_topic, analyzer_identity['drop_keys'])
        if (staged_verification['output_records'] != output_accumulator.count or
                staged_verification['output_topics'] != output_accumulator.document() or
                staged_verification['output_ordered_payload_stream_sha256'] !=
                output_accumulator.digest()):
            raise MaterializationError(
                'verification_accumulator_mismatch',
                'streaming write and independent verification statistics differ')
        staged_tree = sha256_tree(stage)
        os.replace(stage, output_root)
        output_tree = sha256_tree(output_root)
        if output_tree != staged_tree:
            raise MaterializationError(
                'publish_tree_mismatch',
                'atomic rename changed the independently verified output tree')
    os.rmdir(staging_container)
    input_tree = analyzer_identity['input_tree']
    analyzer_file_sha = _sha256_file(analyzer_receipt_path)
    generator_path = Path(__file__).resolve()
    receipt: dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'receipt_kind': RECEIPT_KIND,
        'status': 'PASS',
        'contract_id': MATERIALIZATION_CONTRACT_ID,
        'input': {
            'path': str(input_root),
            'tree': input_tree,
            'tree_hash_kind': 'relative_path_size_content_sha256_v1',
            'record_count': input_accumulator.count,
            'ordered_payload_stream_sha256': input_accumulator.digest(),
            'per_topic': input_accumulator.document(),
        },
        'output': {
            'path': str(output_root),
            'tree': output_tree,
            'tree_hash_kind': 'relative_path_size_content_sha256_v1',
            'record_count': output_accumulator.count,
            'ordered_payload_stream_sha256': output_accumulator.digest(),
            'per_topic': output_accumulator.document(),
        },
        'connections': {
            'source': source_connections,
            'output': staged_verification['output_connections'],
            'output_order_preserved': True,
            'connection_id_semantics': 'source_connection_id_bound_by_ordered_metadata_signature',
        },
        'dropped_lidar': {
            'count': len(dropped_rows),
            'rows': dropped_rows,
            'selection_source': 'analyzer_receipt_only',
        },
        'analyzer': {
            'path': 'scripts/analyze_m6a10_synchronized_tail.py',
            'sha256': analyzer_identity['analyzer_sha256'],
            'receipt_path': str(analyzer_receipt_path),
            'receipt_sha256': analyzer_file_sha,
            'contract_id': analyzer_identity['contract_id'],
        },
        'generator': {
            'path': 'scripts/materialize_m6a10_synchronized_tail.py',
            'sha256': _sha256_file(generator_path),
        },
        'semantic_equivalence': semantic_identity or {
            'status': 'preregistered_not_provided_for_ros2_only_generation',
            'required_before_fast_livo2_run': True,
            'comparator': 'scripts/compare_rosbag_semantic_inputs.py',
        },
        'verification': {
            'status': 'PASS',
            'method': 'independent_raw_stream_reread',
            'output_record_count': staged_verification['output_records'],
            'dropped_count': staged_verification['dropped_count'],
            'ordered_payload_stream_sha256': (
                staged_verification['output_ordered_payload_stream_sha256']),
            'connection_metadata_equal': (
                [_connection_signature(item)
                 for item in staged_verification['connections']] ==
                [_connection_signature(item)
                 for item in staged_verification['output_connections']]),
        },
        'order_definition': ORDER_DEFINITION,
        'safety': {
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'input_modified': False,
            'output_bag_written': True,
            'atomic_publish': True,
            'receipt_external_to_output': True,
            'raw_serialized_payloads_preserved': True,
            'input_record_order_preserved_except_declared_terminal_suffix': True,
        },
        # The path is safe to bind inside the receipt; the receipt's own file
        # SHA is intentionally returned to the caller only to avoid a cycle.
        'receipt_path': str(receipt_path),
    }
    _atomic_write_json(receipt_path, receipt)
    receipt['receipt_path'] = str(receipt_path)
    receipt['receipt_sha256'] = _sha256_file(receipt_path)
    return receipt


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    parser.add_argument('--analyzer-receipt', type=Path, required=True)
    parser.add_argument('--ros1-semantic-report', type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        receipt = materialize_bag(
            args.input, args.output, args.receipt, args.analyzer_receipt,
            ros1_semantic_report=args.ros1_semantic_report)
    except (MaterializationError, OSError, RuntimeError, TypeError, KeyError) as exc:
        code = exc.code if isinstance(exc, MaterializationError) else type(exc).__name__.lower()
        print(f'FAIL_CLOSED {code}: {exc}', file=sys.stderr)
        return 2
    print(json.dumps({
        'status': receipt['status'],
        'output': receipt['output']['path'],
        'receipt': receipt['receipt_path'],
        'dropped_lidar_count': receipt['dropped_lidar']['count'],
    }, sort_keys=True))
    return 0


if __name__ == '__main__':  # pragma: no cover
    raise SystemExit(main())
