#!/usr/bin/env python3
"""Compare ROS1/ROS2 bags by canonicalized message content.

Bag container bytes and ROS wire serialization differ across ROS1 and ROS2.
This tool deserializes both representations with rosbags, hashes every field
(including ndarray payloads), and requires identical record timestamps,
message order, counts, and semantic hashes for each selected topic.
"""

from __future__ import annotations

import argparse
from dataclasses import fields, is_dataclass
import hashlib
import json
from pathlib import Path
import struct
import sys
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader


def _token(digest: Any, tag: bytes, payload: bytes = b'') -> None:
    digest.update(tag)
    digest.update(struct.pack('<Q', len(payload)))
    digest.update(payload)


def _update_numeric_dataclass_sequence(digest: Any, value: list[Any]) -> bool:
    """Hash a homogeneous numeric ROS struct array without per-scalar tokens."""
    if not value or not is_dataclass(value[0]):
        return False
    value_fields = [field for field in fields(value[0])
                    if field.name != '__msgtype__']
    numeric_fields: list[tuple[Any, Any]] = []
    for field in value_fields:
        sample = getattr(value[0], field.name)
        if isinstance(sample, (bool, np.bool_)):
            dtype = np.dtype('?')
        elif isinstance(sample, (int, np.integer)):
            dtype = np.dtype('<i8')
        elif isinstance(sample, (float, np.floating)):
            dtype = np.dtype('<f8')
        else:
            return False
        numeric_fields.append((field, dtype))
    expected_type = type(value[0])
    if any(type(item) is not expected_type for item in value):
        return False
    _token(digest, b'Q')
    update_canonical(digest, len(value))
    for field, dtype in numeric_fields:
        _token(digest, b'K', field.name.encode('utf-8'))
        array = np.fromiter(
            (getattr(item, field.name) for item in value),
            dtype=dtype, count=len(value))
        update_canonical(digest, array)
    _token(digest, b'E')
    return True


def update_canonical(digest: Any, value: Any) -> None:
    """Append an unambiguous, representation-independent value encoding."""
    if is_dataclass(value):
        _token(digest, b'D')
        value_fields = fields(value)
        field_names = {field.name for field in value_fields}
        is_ros_header = {'stamp', 'frame_id'}.issubset(field_names)
        for field in value_fields:
            # std_msgs/Header.seq existed only in ROS1 and is transport-era
            # metadata, not sensor content. ROS2 intentionally removed it.
            if ((is_ros_header and field.name == 'seq') or
                    field.name == '__msgtype__'):
                continue
            _token(digest, b'K', field.name.encode('utf-8'))
            update_canonical(digest, getattr(value, field.name))
        _token(digest, b'E')
    elif isinstance(value, np.ndarray):
        array = np.ascontiguousarray(value)
        if array.dtype.byteorder == '>' or (
                array.dtype.byteorder == '=' and sys.byteorder == 'big'):
            array = array.astype(array.dtype.newbyteorder('<'), copy=False)
        _token(digest, b'A', array.dtype.str.lstrip('<>=|').encode('ascii'))
        update_canonical(digest, tuple(int(item) for item in array.shape))
        _token(digest, b'B', array.tobytes(order='C'))
    elif isinstance(value, (list, tuple)):
        if isinstance(value, list) and _update_numeric_dataclass_sequence(
                digest, value):
            return
        _token(digest, b'L')
        update_canonical(digest, len(value))
        for item in value:
            update_canonical(digest, item)
        _token(digest, b'E')
    elif isinstance(value, (bytes, bytearray, memoryview)):
        _token(digest, b'B', bytes(value))
    elif isinstance(value, str):
        _token(digest, b'S', value.encode('utf-8'))
    elif isinstance(value, (bool, np.bool_)):
        _token(digest, b'T' if bool(value) else b'F')
    elif isinstance(value, (int, np.integer)):
        _token(digest, b'I', str(int(value)).encode('ascii'))
    elif isinstance(value, (float, np.floating)):
        _token(digest, b'R', struct.pack('<d', float(value)))
    elif value is None:
        _token(digest, b'N')
    else:
        raise TypeError(f'unsupported message field type: {type(value)!r}')


def message_hash(topic: str, timestamp: int, message: Any) -> str:
    digest = hashlib.sha256()
    update_canonical(digest, topic)
    update_canonical(digest, timestamp)
    update_canonical(digest, message)
    return digest.hexdigest()


def digest_bag(path: Path, topics: set[str]) -> dict[str, Any]:
    per_topic: dict[str, dict[str, Any]] = {
        topic: {'count': 0, 'aggregate': hashlib.sha256(), 'messages': []}
        for topic in topics}
    with AnyReader([path]) as reader:
        connections = [c for c in reader.connections if c.topic in topics]
        present = {connection.topic for connection in connections}
        missing = topics.difference(present)
        if missing:
            raise ValueError(f'{path} lacks topics: {sorted(missing)}')
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            value = message_hash(connection.topic, timestamp, message)
            row = per_topic[connection.topic]
            row['count'] += 1
            row['aggregate'].update(bytes.fromhex(value))
            row['messages'].append(value)
    return per_topic


def compare(left: dict[str, Any], right: dict[str, Any]) -> dict[str, Any]:
    topics = []
    all_equal = True
    for topic in sorted(set(left) | set(right)):
        lhs, rhs = left.get(topic), right.get(topic)
        count_left = 0 if lhs is None else lhs['count']
        count_right = 0 if rhs is None else rhs['count']
        aggregate_left = None if lhs is None else lhs['aggregate'].hexdigest()
        aggregate_right = None if rhs is None else rhs['aggregate'].hexdigest()
        first_mismatch = None
        if lhs is not None and rhs is not None:
            for index, (a, b) in enumerate(zip(lhs['messages'], rhs['messages'])):
                if a != b:
                    first_mismatch = index
                    break
            if first_mismatch is None and count_left != count_right:
                first_mismatch = min(count_left, count_right)
        equal = count_left == count_right and aggregate_left == aggregate_right
        all_equal &= equal
        topics.append({
            'topic': topic, 'equal': equal,
            'message_count_left': count_left, 'message_count_right': count_right,
            'aggregate_sha256_left': aggregate_left,
            'aggregate_sha256_right': aggregate_right,
            'first_mismatch_index': first_mismatch,
        })
    return {'schema_version': 1, 'all_topics_equal': all_equal, 'topics': topics}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--left', type=Path, required=True)
    parser.add_argument('--right', type=Path, required=True)
    parser.add_argument('--topic', action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    topics = set(args.topic)
    result = compare(digest_bag(args.left.resolve(), topics),
                     digest_bag(args.right.resolve(), topics))
    result['left'] = str(args.left.resolve())
    result['right'] = str(args.right.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['all_topics_equal'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
