#!/usr/bin/env python3
"""Extract and verify the latched Ouster metadata from a ROS1 packet bag."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


METADATA_TOPIC = '/os_cloud_node/metadata'


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def extract_metadata(source: Path) -> tuple[str, dict[str, object]]:
    typestore = get_typestore(Stores.ROS1_NOETIC)
    values: list[str] = []
    stamps: list[int] = []
    with Reader(source) as reader:
        connections = [connection for connection in reader.connections
                       if connection.topic == METADATA_TOPIC]
        if not connections:
            raise ValueError(f'{source} lacks {METADATA_TOPIC}')
        for connection, timestamp, raw in reader.messages(
                connections=connections):
            message = typestore.deserialize_ros1(raw, connection.msgtype)
            values.append(message.data)
            stamps.append(timestamp)
    if not values or not values[0].strip():
        raise ValueError(f'{source} contains empty Ouster metadata')
    if any(value != values[0] for value in values[1:]):
        raise ValueError(f'{source} contains inconsistent Ouster metadata')
    # Parsing here catches truncated strings before the decoder is launched.
    parsed = json.loads(values[0])
    encoded = (values[0].rstrip() + '\n').encode('utf-8')
    report: dict[str, object] = {
        'schema_version': 1,
        'source': str(source.resolve()),
        'metadata_topic': METADATA_TOPIC,
        'metadata_message_count': len(values),
        'first_record_stamp_ns': stamps[0],
        'last_record_stamp_ns': stamps[-1],
        'metadata_sha256': sha256_bytes(encoded),
        'sensor_serial': parsed.get('sensor_info', {}).get('prod_sn'),
        'lidar_mode': parsed.get('config_params', {}).get('lidar_mode'),
    }
    return encoded.decode('utf-8'), report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args()
    for path in (args.output, args.report):
        if path.exists():
            raise ValueError(f'refusing to overwrite: {path}')
    metadata, report = extract_metadata(args.input.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(metadata, encoding='utf-8')
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, TypeError, json.JSONDecodeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
