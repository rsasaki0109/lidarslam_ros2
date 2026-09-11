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

"""Read-only synchronized-tail analysis for the M6a10 input contract.

The RKO-LIO offline reader cannot safely process a LiDAR scan whose last point
timestamp is newer than the last available IMU sample.  This tool identifies
the terminal suffix of such scans before any conversion or replay is started.
It deliberately writes only an atomic JSON receipt; it never writes a bag,
opens a ground-truth file, or invokes a scorer.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any, Iterable


CONTRACT_ID = 'm6a10-v2a-synchronized-tail-v1'
POINTFIELD_UINT32 = 6
SUPPORTED_LIDAR_TYPE = 'sensor_msgs/msg/PointCloud2'
SUPPORTED_IMU_TYPE = 'sensor_msgs/msg/Imu'
TIME_FIELD_NAME = 't'
TIME_UNIT = 'nanoseconds'
TIME_RELATION = 'header.stamp + relative point offset'
SCHEMA_VERSION = 1
_TIME_FIELD_ALIASES = frozenset({'timestamp', 'time', 'stamps'})


class SynchronizedTailError(ValueError):
    """Raised when an input cannot be proven to satisfy the contract."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _sha256_tree(root: Path) -> dict[str, Any]:
    """Hash a bag tree without following symlinks or reading directories."""
    digest = hashlib.sha256()
    files = 0
    total_bytes = 0
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise SynchronizedTailError('symlink_input', f'symlink in input tree: {path}')
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
        raise SynchronizedTailError('empty_input', f'input tree has no files: {root}')
    return {
        'sha256': digest.hexdigest(),
        'file_count': files,
        'total_bytes': total_bytes,
    }


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _header_stamp_ns(message: Any) -> int:
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    sec = getattr(stamp, 'sec', None)
    nanosec = getattr(stamp, 'nanosec', None)
    if not _is_int(sec) or not _is_int(nanosec):
        raise SynchronizedTailError(
            'invalid_header_stamp',
            'message header stamp must contain integer sec/nanosec',
        )
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        raise SynchronizedTailError('invalid_header_stamp', 'message header stamp is out of range')
    return sec * 1_000_000_000 + nanosec


def _point_field_descriptor(field: Any) -> dict[str, Any]:
    name = getattr(field, 'name', None)
    offset = getattr(field, 'offset', None)
    datatype = getattr(field, 'datatype', None)
    count = getattr(field, 'count', None)
    if not isinstance(name, str) or not name:
        raise SynchronizedTailError('invalid_point_field', 'PointField name is missing')
    if not _is_int(offset) or offset < 0:
        raise SynchronizedTailError(
            'invalid_point_field', f'PointField {name!r} offset is invalid')
    if not _is_int(datatype) or not _is_int(count) or count <= 0:
        raise SynchronizedTailError(
            'invalid_point_field', f'PointField {name!r} type/count is invalid')
    return {'name': name, 'offset': offset, 'datatype': datatype, 'count': count}


def validate_pointcloud_schema(message: Any) -> dict[str, Any]:
    """Validate the exact NTU Ouster timestamp schema and return its identity."""
    height = getattr(message, 'height', None)
    width = getattr(message, 'width', None)
    point_step = getattr(message, 'point_step', None)
    row_step = getattr(message, 'row_step', None)
    if not all(_is_int(value) for value in (height, width, point_step, row_step)):
        raise SynchronizedTailError(
            'invalid_pointcloud_shape', 'PointCloud2 shape fields are invalid')
    if height <= 0 or width <= 0 or point_step <= 0 or row_step < width * point_step:
        raise SynchronizedTailError(
            'invalid_pointcloud_shape', 'PointCloud2 shape is not processable')
    if bool(getattr(message, 'is_bigendian', False)):
        raise SynchronizedTailError(
            'unsupported_pointcloud_endianness',
            'big-endian PointCloud2 is unsupported')

    fields = [_point_field_descriptor(field) for field in (getattr(message, 'fields', None) or [])]
    names = [field['name'] for field in fields]
    if len(names) != len(set(names)):
        raise SynchronizedTailError(
            'duplicate_point_field', 'PointCloud2 contains duplicate field names')
    timestamp_aliases = _TIME_FIELD_ALIASES.intersection(names)
    if timestamp_aliases:
        raise SynchronizedTailError(
            'ambiguous_point_timestamp',
            f'unsupported alternate point timestamp fields: {sorted(timestamp_aliases)}',
        )
    timestamp_fields = [field for field in fields if field['name'] == TIME_FIELD_NAME]
    if len(timestamp_fields) != 1:
        raise SynchronizedTailError(
            'unsupported_point_timestamp_schema',
            "PointCloud2 must contain exactly one UINT32 't' field",
        )
    timestamp_field = timestamp_fields[0]
    if timestamp_field['datatype'] != POINTFIELD_UINT32 or timestamp_field['count'] != 1:
        raise SynchronizedTailError(
            'unsupported_point_timestamp_schema',
            "PointCloud2 't' must be one UINT32 field",
        )
    if timestamp_field['offset'] + 4 > point_step:
        raise SynchronizedTailError(
            'invalid_point_timestamp_offset', "PointCloud2 't' exceeds point_step")

    data = bytes(getattr(message, 'data', b''))
    expected_bytes = row_step * height
    if len(data) != expected_bytes:
        raise SynchronizedTailError(
            'invalid_pointcloud_data_size',
            f'PointCloud2 data size {len(data)} != row_step*height {expected_bytes}',
        )
    return {
        'field_name': TIME_FIELD_NAME,
        'datatype': POINTFIELD_UINT32,
        'datatype_name': 'UINT32',
        'count': 1,
        'unit': TIME_UNIT,
        'relative_to': 'header.stamp',
        'relation': TIME_RELATION,
        'endianness': 'little',
        'point_step': point_step,
        'row_step': row_step,
        'height': height,
        'width': width,
        'fields': fields,
    }


def _point_time_min_max(message: Any, schema: dict[str, Any]) -> tuple[int, int]:
    """Read the uint32 timestamp field using PointCloud2 row/point strides."""
    try:
        import numpy as np
    except ModuleNotFoundError as exc:  # pragma: no cover - environment contract
        raise SynchronizedTailError(
            'numpy_missing', 'numpy is required for PointCloud2 analysis') from exc
    data = bytes(getattr(message, 'data', b''))
    values = np.ndarray(
        shape=(schema['height'], schema['width']),
        dtype=np.dtype('<u4'),
        buffer=data,
        offset=next(
            field['offset'] for field in schema['fields'] if field['name'] == TIME_FIELD_NAME),
        strides=(schema['row_step'], schema['point_step']),
    )
    return int(values.min()), int(values.max())


def summarize_lidar_message(
    message: Any,
    storage_timestamp_ns: int,
    *,
    serialized_payload: bytes = b'',
    source_index: int = 0,
) -> dict[str, Any]:
    """Validate and summarize one PointCloud2 without retaining its payload."""
    schema = validate_pointcloud_schema(message)
    header_ns = _header_stamp_ns(message)
    raw_min, raw_max = _point_time_min_max(message, schema)
    if raw_min < 0:  # uint32 cannot normally reach this; retain explicit contract check.
        raise SynchronizedTailError('negative_relative_timestamp', "PointCloud2 't' is negative")
    payload_sha = hashlib.sha256(bytes(serialized_payload)).hexdigest()
    return {
        'source_index': int(source_index),
        'storage_timestamp_ns': int(storage_timestamp_ns),
        'header_timestamp_ns': header_ns,
        'raw_point_timestamp_min_ns': raw_min,
        'raw_point_timestamp_max_ns': raw_max,
        'point_timestamp_min_ns': header_ns + raw_min,
        'point_timestamp_max_ns': header_ns + raw_max,
        'payload_sha256': payload_sha,
        'point_count': schema['height'] * schema['width'],
        'schema': schema,
    }


def evaluate_synchronized_tail(
    lidar_rows: Iterable[dict[str, Any]],
    *,
    last_imu_header_timestamp_ns: int,
    first_sensor_header_timestamp_ns: int | None = None,
) -> dict[str, Any]:
    """Evaluate a sorted-independent terminal suffix of ineligible scans."""
    if not _is_int(last_imu_header_timestamp_ns) or last_imu_header_timestamp_ns < 0:
        raise SynchronizedTailError('invalid_last_imu_timestamp', 'last IMU timestamp is invalid')
    rows = sorted(
        (dict(row) for row in lidar_rows),
        key=lambda row: (
            int(row['header_timestamp_ns']),
            int(row['point_timestamp_max_ns']),
            str(row.get('payload_sha256', '')),
            int(row['storage_timestamp_ns']),
            int(row['point_timestamp_min_ns']),
        ),
    )
    if not rows:
        raise SynchronizedTailError('no_lidar_messages', 'no LiDAR messages were analyzed')
    for index, row in enumerate(rows):
        for key in ('header_timestamp_ns', 'point_timestamp_min_ns', 'point_timestamp_max_ns'):
            if not _is_int(row.get(key)):
                raise SynchronizedTailError(
                    'invalid_lidar_summary', f'LiDAR row missing integer {key}')
        row['message_index'] = index
        # RKO-LIO's processing condition is strict: the IMU buffer back must
        # be newer than the scan's final point timestamp.  Equality therefore
        # remains in the terminal suffix and is never silently trimmed in.
        row['eligible'] = row['point_timestamp_max_ns'] < last_imu_header_timestamp_ns

    first_ineligible = next(
        (index for index, row in enumerate(rows) if not row['eligible']), None)
    if first_ineligible is not None and any(
            row['eligible'] for row in rows[first_ineligible + 1:]):
        raise SynchronizedTailError(
            'nonterminal_ineligible_pattern',
            'an ineligible LiDAR message is followed by an eligible message',
        )
    eligible_count = len(rows) if first_ineligible is None else first_ineligible
    terminal_rows = rows[eligible_count:]
    if first_sensor_header_timestamp_ns is None:
        first_sensor_header_timestamp_ns = min(row['header_timestamp_ns'] for row in rows)
    if not _is_int(first_sensor_header_timestamp_ns) or first_sensor_header_timestamp_ns < 0:
        raise SynchronizedTailError(
            'invalid_first_sensor_timestamp', 'first sensor timestamp is invalid')
    cutoff_ns = rows[eligible_count - 1]['point_timestamp_max_ns'] if eligible_count else None
    end_duration = (
        None if cutoff_ns is None else (
            cutoff_ns - first_sensor_header_timestamp_ns) / 1_000_000_000.0
    )
    if end_duration is not None and end_duration < 0:
        raise SynchronizedTailError(
            'negative_proposed_duration', 'proposed synchronized duration is negative')
    return {
        'status': 'PASS' if eligible_count else 'FAIL_CLOSED',
        'failure_reason': None if eligible_count else 'no_eligible_lidar_messages',
        'comparator': 'point_timestamp_max_ns < last_imu_header_timestamp_ns',
        'comparison_is_strict': True,
        'last_imu_header_timestamp_ns': last_imu_header_timestamp_ns,
        'first_sensor_header_timestamp_ns': first_sensor_header_timestamp_ns,
        'lidar_count': len(rows),
        'eligible_lidar_count': eligible_count,
        'ineligible_lidar_count': len(terminal_rows),
        'terminal_lidar': [
            {
                'message_index': row['message_index'],
                'header_timestamp_ns': row['header_timestamp_ns'],
                'point_timestamp_min_ns': row['point_timestamp_min_ns'],
                'point_timestamp_max_ns': row['point_timestamp_max_ns'],
                'storage_timestamp_ns': row['storage_timestamp_ns'],
                'payload_sha256': row.get('payload_sha256', ''),
            }
            for row in terminal_rows
        ],
        'proposed_cutoff_timestamp_ns': cutoff_ns,
        'proposed_end_duration_seconds': end_duration,
        'proposed_keep_lidar_count': eligible_count,
        'proposed_output_bag': None,
    }


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path = path.expanduser().resolve()
    if path.exists() or path.is_symlink():
        raise SynchronizedTailError('output_exists', f'output already exists: {path}')
    part = path.with_name(path.name + '.part')
    if part.exists() or part.is_symlink():
        raise SynchronizedTailError(
            'output_part_exists', f'output staging file already exists: {part}')
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (
        json.dumps(payload, sort_keys=True, separators=(',', ':'), ensure_ascii=True) + '\n'
    ).encode()
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


def _ensure_disjoint_paths(input_root: Path, output_path: Path) -> None:
    input_root = input_root.resolve()
    output_path = output_path.resolve(strict=False)
    try:
        common = os.path.commonpath((str(input_root), str(output_path)))
        if common == str(input_root):
            raise SynchronizedTailError(
                'output_inside_input', 'receipt output must be outside the input bag')
        if common == str(output_path):
            raise SynchronizedTailError(
                'input_inside_output', 'output root must not contain the input bag')
    except ValueError as exc:
        raise SynchronizedTailError(
            'path_resolution_error',
            'input/output paths have no common filesystem') from exc


def _deserialize_bag(bag_root: Path, lidar_topic: str, imu_topic: str) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as exc:  # pragma: no cover - environment contract
        raise SynchronizedTailError(
            'rosbags_missing', 'rosbags is required for dry-run analysis') from exc

    try:
        with AnyReader([bag_root], default_typestore=get_typestore(Stores.LATEST)) as reader:
            connections = list(reader.connections)
            lidar_connections = [
                connection for connection in connections if connection.topic == lidar_topic]
            imu_connections = [
                connection for connection in connections if connection.topic == imu_topic]
            if len(lidar_connections) != 1:
                raise SynchronizedTailError(
                    'lidar_topic_contract', 'exactly one LiDAR connection is required')
            if len(imu_connections) != 1:
                raise SynchronizedTailError(
                    'imu_topic_contract', 'exactly one IMU connection is required')
            lidar_connection = lidar_connections[0]
            imu_connection = imu_connections[0]
            if lidar_connection.msgtype != SUPPORTED_LIDAR_TYPE:
                raise SynchronizedTailError(
                    'lidar_type_contract',
                    'LiDAR topic is not sensor_msgs/msg/PointCloud2')
            if imu_connection.msgtype != SUPPORTED_IMU_TYPE:
                raise SynchronizedTailError(
                    'imu_type_contract', 'IMU topic is not sensor_msgs/msg/Imu')

            topic_counts = {
                connection.topic: int(connection.msgcount)
                for connection in connections
                if _is_int(getattr(connection, 'msgcount', None))
            }
            imu_count = 0
            last_imu = None
            first_imu = None
            lidar_rows: list[dict[str, Any]] = []
            first_lidar = None
            wanted = [lidar_connection, imu_connection]
            for connection, storage_ns, raw in reader.messages(connections=wanted):
                message = reader.deserialize(raw, connection.msgtype)
                if connection.id == imu_connection.id:
                    stamp_ns = _header_stamp_ns(message)
                    imu_count += 1
                    last_imu = stamp_ns if last_imu is None else max(last_imu, stamp_ns)
                    first_imu = stamp_ns if first_imu is None else min(first_imu, stamp_ns)
                else:
                    row = summarize_lidar_message(
                        message,
                        int(storage_ns),
                        serialized_payload=bytes(raw),
                        source_index=len(lidar_rows),
                    )
                    schema = row['schema']
                    if lidar_rows and schema != lidar_rows[0]['schema']:
                        raise SynchronizedTailError(
                            'changing_pointcloud_schema',
                            'LiDAR schema changed mid-bag')
                    lidar_rows.append(row)
                    first_lidar = row['header_timestamp_ns'] if first_lidar is None else min(
                        first_lidar,
                        row['header_timestamp_ns'],
                    )
            if imu_count != topic_counts.get(imu_topic, imu_count):
                raise SynchronizedTailError(
                    'imu_count_mismatch',
                    'IMU metadata count differs from observed count')
            if len(lidar_rows) != topic_counts.get(lidar_topic, len(lidar_rows)):
                raise SynchronizedTailError(
                    'lidar_count_mismatch',
                    'LiDAR metadata count differs from observed count')
            if last_imu is None or first_imu is None:
                raise SynchronizedTailError('no_imu_messages', 'no IMU messages were analyzed')
            result = evaluate_synchronized_tail(
                lidar_rows,
                last_imu_header_timestamp_ns=last_imu,
                first_sensor_header_timestamp_ns=min(first_imu, first_lidar),
            )
            result.update({
                'topic_counts': topic_counts,
                'imu_message_count_observed': imu_count,
                'lidar_schema': lidar_rows[0]['schema'],
                'imu_topic': imu_topic,
                'lidar_topic': lidar_topic,
            })
            return result
    except SynchronizedTailError:
        raise
    except Exception as exc:
        raise SynchronizedTailError(
            'rosbag_read_error', f'rosbag read failed: {type(exc).__name__}') from exc


def analyze_bag(bag_root: Path, lidar_topic: str, imu_topic: str) -> dict[str, Any]:
    bag_root = bag_root.expanduser().resolve(strict=True)
    if not bag_root.is_dir():
        raise SynchronizedTailError('input_not_directory', 'ROS2 bag input must be a directory')
    metadata = bag_root / 'metadata.yaml'
    if not metadata.is_file() or metadata.is_symlink():
        raise SynchronizedTailError('metadata_missing', 'ROS2 bag metadata.yaml is missing')
    result = _deserialize_bag(bag_root, lidar_topic, imu_topic)
    result['input'] = {
        'path': str(bag_root),
        'metadata_sha256': _sha256_file(metadata),
        'tree': _sha256_tree(bag_root),
    }
    return result


def _failure_receipt(error: SynchronizedTailError, bag: Path, output: Path) -> dict[str, Any]:
    return {
        'schema_version': SCHEMA_VERSION,
        'contract_id': CONTRACT_ID,
        'status': 'FAIL_CLOSED',
        'failure_code': error.code,
        'failure_type': type(error).__name__,
        'failure_message': str(error),
        'mode': 'dry_run',
        'input_path': str(bag),
        'output_receipt_path': str(output),
        'safety': {
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'output_bag_written': False,
        },
    }


def build_receipt(result: dict[str, Any], output: Path) -> dict[str, Any]:
    script = Path(__file__).resolve()
    lidar_topic = result.pop('lidar_topic')
    imu_topic = result.pop('imu_topic')
    topic_counts = result['topic_counts']
    lidar_schema = result.pop('lidar_schema')
    return {
        'schema_version': SCHEMA_VERSION,
        'contract_id': CONTRACT_ID,
        'status': result['status'],
        'mode': 'dry_run',
        'generated_by': {
            'path': 'scripts/analyze_m6a10_synchronized_tail.py',
            'sha256': _sha256_file(script),
        },
        'input': result.pop('input'),
        'topics': {
            'lidar': {
                'topic': lidar_topic,
                'type': SUPPORTED_LIDAR_TYPE,
                'count': topic_counts.get(lidar_topic, result['lidar_count']),
                'schema': lidar_schema,
            },
            'imu': {
                'topic': imu_topic,
                'type': SUPPORTED_IMU_TYPE,
                'count': topic_counts.get(imu_topic, result['imu_message_count_observed']),
                'last_header_timestamp_ns': result['last_imu_header_timestamp_ns'],
            },
            'counts': result.pop('topic_counts'),
        },
        'tail_evaluation': result,
        'safety': {
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'output_bag_written': False,
            'receipt_output_path': str(output.resolve()),
        },
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--lidar-topic', default='/os1_cloud_node1/points')
    parser.add_argument('--imu-topic', default='/imu/imu')
    parser.add_argument('--dry-run', action='store_true', required=True)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    bag = args.bag.expanduser().resolve(strict=False)
    output = args.output.expanduser().resolve(strict=False)
    error: SynchronizedTailError | None = None
    try:
        if not bag.exists():
            raise SynchronizedTailError('input_missing', f'input bag does not exist: {bag}')
        _ensure_disjoint_paths(bag, output)
        result = analyze_bag(bag, args.lidar_topic, args.imu_topic)
        receipt = build_receipt(result, output)
    except SynchronizedTailError as exc:
        error = exc
        receipt = _failure_receipt(exc, bag, output)
    try:
        _atomic_write_json(output, receipt)
    except SynchronizedTailError as exc:
        print(f'FAIL_CLOSED {exc.code}: {exc}', file=sys.stderr)
        return 2
    if error is not None:
        print(f'FAIL_CLOSED {error.code}: receipt={output}', file=sys.stderr)
        return 2
    print(json.dumps({
        'status': receipt['status'],
        'receipt': str(output),
        'eligible_lidar_count': receipt['tail_evaluation']['eligible_lidar_count'],
        'ineligible_lidar_count': receipt['tail_evaluation']['ineligible_lidar_count'],
        'proposed_end_duration_seconds': (
            receipt['tail_evaluation']['proposed_end_duration_seconds']),
    }, sort_keys=True))
    return 0 if receipt['status'] == 'PASS' else 2


if __name__ == '__main__':  # pragma: no cover
    raise SystemExit(main())
