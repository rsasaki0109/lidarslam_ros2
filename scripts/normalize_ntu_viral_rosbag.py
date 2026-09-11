#!/usr/bin/env python3
"""Create a deterministic, topic-filtered NTU-VIRAL ROS1 benchmark bag.

The dataset authors document jitter in the Ouster header stamps. Their public
restamp utility linearly spaces the stamps between the first and last scan.
This implementation applies that published operation only to the horizontal
Ouster cloud, selects the intersection of the three sensor header-time ranges,
and records every transformation parameter for freezing.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer


DEFAULT_TOPICS = (
    '/os1_cloud_node1/points', '/imu/imu', '/left/image_raw')
LIDAR_TOPIC = '/os1_cloud_node1/points'


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def regularized_stamps(first: int, last: int, count: int) -> list[int]:
    if count < 2 or last <= first:
        raise ValueError('at least two increasing LiDAR stamps are required')
    span = last - first
    # Integer arithmetic avoids platform-dependent floating-point rounding.
    return [first + (span * index + (count - 1) // 2) // (count - 1)
            for index in range(count)]


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def set_stamp_ns(message: Any, value: int) -> None:
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        int(value), 1_000_000_000)


def normalize(source: Path, destination: Path,
              topics: tuple[str, ...] = DEFAULT_TOPICS) -> dict[str, Any]:
    if destination.exists():
        raise ValueError(f'refusing to overwrite output: {destination}')
    selected = set(topics)
    with AnyReader([source]) as reader:
        connections = [c for c in reader.connections if c.topic in selected]
        present = {c.topic for c in connections}
        missing = selected.difference(present)
        if missing:
            raise ValueError(f'input bag lacks topics: {sorted(missing)}')
        input_stamps: dict[str, list[int]] = {topic: [] for topic in selected}
        for connection, _, raw in reader.messages(connections=connections):
            input_stamps[connection.topic].append(stamp_ns(
                reader.deserialize(raw, connection.msgtype)))
        lidar_stamps = input_stamps[LIDAR_TOPIC]
        replacement = regularized_stamps(
            lidar_stamps[0], lidar_stamps[-1], len(lidar_stamps))
        common_start = max(stamps[0] for stamps in input_stamps.values())
        common_end = min(stamps[-1] for stamps in input_stamps.values())
        if common_end <= common_start:
            raise ValueError('selected sensor topics have no common time range')
        connection_map = {}
        counts = {topic: 0 for topic in selected}
        skipped = {topic: 0 for topic in selected}
        with Writer(destination) as writer:
            for connection in connections:
                connection_map[connection.id] = writer.add_connection(
                    connection.topic, connection.msgtype,
                    typestore=reader.typestore,
                    callerid=connection.ext.callerid,
                    latching=connection.ext.latching)
            lidar_index = 0
            for connection, _, raw in reader.messages(
                    connections=connections):
                output_raw = raw
                message = reader.deserialize(raw, connection.msgtype)
                message_stamp = stamp_ns(message)
                if connection.topic == LIDAR_TOPIC:
                    message_stamp = replacement[lidar_index]
                    set_stamp_ns(message, message_stamp)
                    output_raw = reader.typestore.serialize_ros1(
                        message, connection.msgtype)
                    lidar_index += 1
                if not common_start <= message_stamp <= common_end:
                    skipped[connection.topic] += 1
                    continue
                # Schedule every selected message by the same header-clock
                # convention. Sensor payload and non-LiDAR headers are intact.
                output_timestamp = message_stamp + 1_000_000
                writer.write(connection_map[connection.id], output_timestamp,
                             output_raw)
                counts[connection.topic] += 1
    return {
        'schema_version': 1,
        'operation': 'ntu_viral_author_linear_ouster_header_restamp',
        'source': str(source.resolve()),
        'destination': str(destination.resolve()),
        'source_sha256': sha256_file(source),
        'destination_sha256': sha256_file(destination),
        'topics': counts,
        'skipped_outside_common_range': skipped,
        'common_start_stamp_ns': common_start,
        'common_end_stamp_ns': common_end,
        'lidar_topic': LIDAR_TOPIC,
        'lidar_count': len(replacement),
        'original_first_stamp_ns': lidar_stamps[0],
        'original_last_stamp_ns': lidar_stamps[-1],
        'normalized_first_stamp_ns': replacement[0],
        'normalized_last_stamp_ns': replacement[-1],
        'normalized_interval_numerator_ns': lidar_stamps[-1] - lidar_stamps[0],
        'normalized_interval_denominator': len(lidar_stamps) - 1,
        'record_time_offset_ns': 1_000_000,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args()
    if args.report.exists():
        raise ValueError(f'refusing to overwrite report: {args.report}')
    report = normalize(args.input.resolve(), args.output.resolve())
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n',
                           encoding='utf-8')
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
