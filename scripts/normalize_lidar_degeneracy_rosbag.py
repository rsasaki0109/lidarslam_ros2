#!/usr/bin/env python3
"""Normalize a decoded LiDAR Degeneracy ROS1 bag onto its sensor clock.

The official packet decoder publishes correctly trigger-stamped PointCloud2
messages, but a separate ``rosbag record`` process timestamps records with its
wall clock.  This pass keeps the sensor payloads byte-for-byte intact, filters
to the common sensor interval, and schedules LiDAR, IMU, and optional radar
records by header time.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from contextlib import ExitStack
from pathlib import Path
import sys
from typing import Any

from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer


LIDAR_TOPIC = '/os_cloud_node/points'
IMU_TOPIC = '/vectornav_node/uncomp_imu'
RADAR_TOPIC = '/radar/cloud'
REQUIRED_TOPICS = (LIDAR_TOPIC, IMU_TOPIC)
OPTIONAL_TOPICS = (RADAR_TOPIC,)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def normalize(source: Path, destination: Path,
              radar_source: Path | None = None) -> dict[str, Any]:
    if destination.exists():
        raise ValueError(f'refusing to overwrite: {destination}')
    with ExitStack() as stack:
        reader = stack.enter_context(AnyReader([source]))
        selected_topics = set(REQUIRED_TOPICS + OPTIONAL_TOPICS)
        connections = [connection for connection in reader.connections
                       if connection.topic in selected_topics]
        present = {connection.topic for connection in connections}
        missing = set(REQUIRED_TOPICS) - present
        if missing:
            raise ValueError(f'missing required topics: {missing}')

        radar_reader = None
        radar_connections: list[Any] = []
        if radar_source is not None:
            if RADAR_TOPIC in present:
                raise ValueError(
                    f'{RADAR_TOPIC} already present in --input; '
                    'do not also pass --radar-input')
            radar_reader = stack.enter_context(AnyReader([radar_source]))
            radar_connections = [connection for connection in radar_reader.connections
                                  if connection.topic == RADAR_TOPIC]
            if not radar_connections:
                raise ValueError(f'{radar_source} has no {RADAR_TOPIC} topic')
            present = present | {RADAR_TOPIC}

        topics = tuple(topic for topic in REQUIRED_TOPICS + OPTIONAL_TOPICS
                       if topic in present)
        stamps: dict[str, list[int]] = {topic: [] for topic in topics}
        for connection, _, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            stamps[connection.topic].append(stamp_ns(message))
        for connection, _, raw in radar_reader.messages(connections=radar_connections) \
                if radar_reader is not None else ():
            message = radar_reader.deserialize(raw, connection.msgtype)
            stamps[connection.topic].append(stamp_ns(message))
        for topic, values in stamps.items():
            if len(values) < 2 or any(a >= b for a, b in zip(values, values[1:])):
                raise ValueError(f'{topic} header stamps are not strictly increasing')
        # The common interval is defined by the required LiDAR+IMU topics only;
        # an optional radar stream never widens or narrows it. Radar records
        # outside this window are dropped like any other out-of-range record.
        common_start = max(stamps[topic][0] for topic in REQUIRED_TOPICS)
        common_end = min(stamps[topic][-1] for topic in REQUIRED_TOPICS)
        if common_end <= common_start:
            raise ValueError('LiDAR and IMU have no common header-time range')

        connection_map = {}
        counts = {topic: 0 for topic in topics}
        skipped = {topic: 0 for topic in topics}
        with Writer(destination) as writer:
            for connection in connections:
                connection_map[connection.id] = writer.add_connection(
                    connection.topic, connection.msgtype,
                    typestore=reader.typestore,
                    callerid=connection.ext.callerid,
                    latching=connection.ext.latching)
            for connection, _, raw in reader.messages(connections=connections):
                message = reader.deserialize(raw, connection.msgtype)
                timestamp = stamp_ns(message)
                if not common_start <= timestamp <= common_end:
                    skipped[connection.topic] += 1
                    continue
                writer.write(connection_map[connection.id], timestamp, raw)
                counts[connection.topic] += 1

            if radar_reader is not None:
                radar_connection_map = {}
                for connection in radar_connections:
                    radar_connection_map[connection.id] = writer.add_connection(
                        connection.topic, connection.msgtype,
                        typestore=radar_reader.typestore,
                        callerid=connection.ext.callerid,
                        latching=connection.ext.latching)
                for connection, _, raw in radar_reader.messages(connections=radar_connections):
                    message = radar_reader.deserialize(raw, connection.msgtype)
                    timestamp = stamp_ns(message)
                    if not common_start <= timestamp <= common_end:
                        skipped[connection.topic] += 1
                        continue
                    writer.write(radar_connection_map[connection.id], timestamp, raw)
                    counts[connection.topic] += 1

    return {
        'schema_version': 1,
        'operation': 'header_clock_reschedule_and_common_interval_filter',
        'source': str(source.resolve()),
        'destination': str(destination.resolve()),
        'source_sha256': sha256_file(source),
        'destination_sha256': sha256_file(destination),
        'radar_source': str(radar_source.resolve()) if radar_source else None,
        'radar_source_sha256': sha256_file(radar_source) if radar_source else None,
        'topics': counts,
        'skipped_outside_common_range': skipped,
        'common_start_stamp_ns': common_start,
        'common_end_stamp_ns': common_end,
        'record_clock': 'sensor_header_stamp',
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--radar-input', type=Path, default=None,
                        help=f'optional bag to pull the {RADAR_TOPIC} connection from')
    args = parser.parse_args()
    if args.report.exists():
        raise ValueError(f'refusing to overwrite: {args.report}')
    radar_input = args.radar_input.resolve() if args.radar_input else None
    report = normalize(args.input.resolve(), args.output.resolve(), radar_input)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(
        json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
