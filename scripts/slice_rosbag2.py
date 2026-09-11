#!/usr/bin/env python3
"""Copy a deterministic time/topic slice of a rosbag2 without reserializing messages."""

from __future__ import annotations

import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import get_typestore, Stores, TypesysError


SUPPORTED_TYPESTORES = (
    get_typestore(Stores.ROS2_HUMBLE),
    get_typestore(Stores.ROS2_JAZZY),
)


def _portable_msgdef(msgtype: str, source_digest: str) -> tuple[str, str]:
    """Return a definition shared by Humble/Jazzy or reject ambiguity."""
    definitions = []
    for typestore in SUPPORTED_TYPESTORES:
        try:
            msgdef, _ = typestore.generate_msgdef(msgtype, ros_version=2)
            digest = typestore.hash_rihs01(msgtype)
        except (KeyError, TypesysError) as exc:
            raise ValueError(
                f'{msgtype} has no embedded definition and is not available '
                'in both supported ROS 2 typestores'
            ) from exc
        definitions.append((msgdef, digest))
    if len(set(definitions)) != 1:
        raise ValueError(
            f'{msgtype} has no embedded definition and differs between '
            'supported ROS 2 typestores'
        )
    msgdef, digest = definitions[0]
    if source_digest and source_digest != digest:
        raise ValueError(
            f'{msgtype} source type hash mismatch: '
            f'expected {source_digest}, generated {digest}'
        )
    return msgdef, digest


def slice_bag(source: Path, destination: Path, duration_seconds: float,
              topics: set[str], start_offset_seconds: float = 0.0) -> dict[str, int]:
    if destination.exists():
        raise ValueError(f'destination already exists: {destination}')
    if duration_seconds <= 0.0 or start_offset_seconds < 0.0:
        raise ValueError('duration must be positive and start offset non-negative')

    counts: dict[str, int] = {}
    typestore = SUPPORTED_TYPESTORES[0]
    with AnyReader([source], default_typestore=typestore) as reader:
        selected = [connection for connection in reader.connections
                    if not topics or connection.topic in topics]
        missing = topics - {connection.topic for connection in selected}
        if missing:
            raise ValueError(f'topics absent from source bag: {sorted(missing)}')
        start_ns = reader.start_time + round(start_offset_seconds * 1e9)
        end_ns = start_ns + round(duration_seconds * 1e9)
        with Writer(destination, version=8) as writer:
            outputs = {}
            for connection in selected:
                kwargs = {
                    'serialization_format': connection.ext.serialization_format,
                    'offered_qos_profiles': connection.ext.offered_qos_profiles,
                }
                if getattr(connection.msgdef, 'data', ''):
                    if not connection.digest:
                        raise ValueError(
                            f'{connection.msgtype} embeds a definition without '
                            'a type hash'
                        )
                    kwargs['msgdef'] = connection.msgdef.data
                    kwargs['rihs01'] = connection.digest
                else:
                    msgdef, digest = _portable_msgdef(
                        connection.msgtype, connection.digest
                    )
                    kwargs['msgdef'] = msgdef
                    kwargs['rihs01'] = digest
                outputs[connection.id] = writer.add_connection(
                    connection.topic,
                    connection.msgtype,
                    **kwargs,
                )
            for connection, timestamp, rawdata in reader.messages(
                    connections=selected, start=start_ns, stop=end_ns):
                writer.write(outputs[connection.id], timestamp, rawdata)
                counts[connection.topic] = counts.get(connection.topic, 0) + 1
    return counts


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--duration-seconds', type=float, required=True)
    parser.add_argument('--start-offset-seconds', type=float, default=0.0)
    parser.add_argument('--topic', action='append', default=[])
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    counts = slice_bag(
        args.source.resolve(), args.destination.resolve(), args.duration_seconds,
        set(args.topic), args.start_offset_seconds)
    print(f'output: {args.destination.resolve()}')
    for topic, count in sorted(counts.items()):
        print(f'{topic}: {count}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
