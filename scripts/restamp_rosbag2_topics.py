#!/usr/bin/env python3

from __future__ import annotations

import argparse
import shutil
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore


@dataclass
class TopicStats:
    count: int = 0
    first_stamp_ns: int = 0
    last_stamp_ns: int = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Create a slim rosbag2 containing only selected topics, with message "
            "header stamps and bag timestamps regularized to a linear timeline."
        ),
    )
    parser.add_argument("--input", required=True, help="Input rosbag2 directory")
    parser.add_argument("--output", required=True, help="Output rosbag2 directory")
    parser.add_argument(
        "--topic",
        action="append",
        required=True,
        dest="topics",
        help="Topic to copy and regularize. Repeat for multiple topics.",
    )
    parser.add_argument(
        "--copy-topic",
        action="append",
        default=[],
        dest="copy_topics",
        help="Topic to copy without changing message/header timestamps. Repeat for multiple topics.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Remove output directory if it already exists.",
    )
    return parser.parse_args()


def sec_nsec_from_ns(stamp_ns: int) -> tuple[int, int]:
    sec = stamp_ns // 1_000_000_000
    nanosec = stamp_ns % 1_000_000_000
    return sec, nanosec


def regularized_stamp(stats: TopicStats, index: int) -> int:
    if stats.count <= 1:
        return stats.first_stamp_ns
    span = stats.last_stamp_ns - stats.first_stamp_ns
    return stats.first_stamp_ns + round(span * index / (stats.count - 1))


def main() -> int:
    args = parse_args()
    input_path = Path(args.input).expanduser().resolve()
    output_path = Path(args.output).expanduser().resolve()
    restamp_topics = list(dict.fromkeys(args.topics))
    copy_topics = list(dict.fromkeys(args.copy_topics))
    all_topics = list(dict.fromkeys(restamp_topics + copy_topics))

    if not input_path.is_dir():
        raise SystemExit(f"input bag not found: {input_path}")
    if output_path.exists():
        if not args.force:
            raise SystemExit(f"output already exists: {output_path}")
        shutil.rmtree(output_path)

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    stats: dict[str, TopicStats] = defaultdict(TopicStats)

    with AnyReader([input_path], default_typestore=typestore) as reader:
        connections = {conn.topic: conn for conn in reader.connections if conn.topic in all_topics}
        missing = [topic for topic in all_topics if topic not in connections]
        if missing:
            raise SystemExit(f"missing topics: {', '.join(missing)}")

        restamp_connections = [connections[topic] for topic in restamp_topics]
        for conn, _, raw in reader.messages(restamp_connections):
            msg = reader.deserialize(raw, conn.msgtype)
            header = getattr(msg, "header", None)
            if header is None:
                raise SystemExit(f"topic has no header: {conn.topic}")
            stamp_ns = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
            topic_stats = stats[conn.topic]
            if topic_stats.count == 0:
                topic_stats.first_stamp_ns = stamp_ns
            topic_stats.last_stamp_ns = stamp_ns
            topic_stats.count += 1

    for topic in restamp_topics:
        topic_stats = stats[topic]
        print(
            f"{topic}: count={topic_stats.count} "
            f"first={topic_stats.first_stamp_ns} last={topic_stats.last_stamp_ns}",
        )

    with AnyReader([input_path], default_typestore=typestore) as reader, Writer(
        output_path,
        version=8,
    ) as writer:
        input_connections = {conn.topic: conn for conn in reader.connections if conn.topic in all_topics}
        output_connections = {}
        for topic in all_topics:
            conn = input_connections[topic]
            output_connections[topic] = writer.add_connection(
                topic,
                conn.msgtype,
                typestore=typestore,
                msgdef=conn.msgdef.data,
                rihs01=conn.digest,
                serialization_format=conn.ext.serialization_format,
                offered_qos_profiles=conn.ext.offered_qos_profiles,
            )

        topic_indices = defaultdict(int)
        for conn, _, raw in reader.messages(input_connections.values()):
            if conn.topic in copy_topics:
                writer.write(output_connections[conn.topic], _, raw)
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            idx = topic_indices[conn.topic]
            topic_indices[conn.topic] += 1
            stamp_ns = regularized_stamp(stats[conn.topic], idx)
            sec, nanosec = sec_nsec_from_ns(stamp_ns)
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
            writer.write(
                output_connections[conn.topic],
                stamp_ns,
                typestore.serialize_cdr(msg, conn.msgtype),
            )

    print(f"wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
