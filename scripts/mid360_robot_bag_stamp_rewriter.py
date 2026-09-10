#!/usr/bin/env python3
"""Rewrite PointCloud2 / Imu header.stamp in a rosbag2 to match receive time.

Some public MID-360 datasets have PointCloud2 messages whose `header.stamp`
field is corrupted or non-monotonic, which makes RKO-LIO drop frames with
errors like "Received LiDAR scan with 522.4 seconds delta to previous scan."
This module reads a rosbag2 directory and writes a new one where the
header.stamp of selected topics has been overwritten with the rosbag2
receive timestamp (`timestamp_ns`). All other messages are passed through
unchanged.
"""

from __future__ import annotations

import shutil
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


BAG_STAMP_REWRITER_JSON = 'mid360_robot_bag_stamp_rewriter.json'
BAG_STAMP_REWRITER_MARKDOWN = 'mid360_robot_bag_stamp_rewriter.md'

DEFAULT_STAMP_MSGTYPES = (
    'sensor_msgs/msg/PointCloud2',
    'sensor_msgs/msg/Imu',
)


@dataclass(frozen=True)
class BagStampRewriterOptions:
    """Options for the bag stamp rewriter."""

    input_bag: Path
    output_bag: Path
    rewrite_msgtypes: tuple[str, ...] = DEFAULT_STAMP_MSGTYPES
    rewrite_topics: tuple[str, ...] = ()
    force: bool = False


class BagStampRewriter:
    """Rewrite header.stamp of selected messages to match the bag receive time."""

    def rewrite(self, options: BagStampRewriterOptions) -> dict[str, Any]:
        """Rewrite the bag and return a summary dict."""
        input_bag = options.input_bag.expanduser().resolve()
        output_bag = options.output_bag.expanduser().resolve()
        if not input_bag.exists():
            raise FileNotFoundError(f'input bag not found: {input_bag}')
        _prepare_output(output_bag, options.force)
        result = _rewrite_bag(
            input_bag=input_bag,
            output_bag=output_bag,
            rewrite_msgtypes=set(options.rewrite_msgtypes),
            rewrite_topics=set(options.rewrite_topics),
        )

        summary: dict[str, Any] = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'PASS' if result['total_messages'] > 0 else 'FAIL',
            'input_bag': str(input_bag),
            'output_bag': str(output_bag),
            'rewrite_msgtypes': list(options.rewrite_msgtypes),
            'rewrite_topics': list(options.rewrite_topics),
            'result': result,
        }
        _write_summary(output_bag.parent, summary)
        return summary


def render_bag_stamp_rewriter_markdown(summary: dict[str, Any]) -> str:
    """Render the rewriter summary as Markdown."""
    result = summary.get('result') or {}
    lines = [
        '# MID-360 Bag Stamp Rewriter',
        '',
        f"- status: `{summary.get('status', '')}`",
        f"- created_at: `{summary.get('created_at', '')}`",
        f"- input_bag: `{summary.get('input_bag', '')}`",
        f"- output_bag: `{summary.get('output_bag', '')}`",
        f"- total_messages: `{result.get('total_messages', 0)}`",
        f"- rewritten_messages: `{result.get('rewritten_messages', 0)}`",
        f"- passthrough_messages: `{result.get('passthrough_messages', 0)}`",
        f"- max_stamp_delta_sec: `{result.get('max_stamp_delta_sec', 0.0):.6f}`",
        '',
        '## Rewrite Targets',
        '',
    ]
    targets = summary.get('rewrite_msgtypes') or []
    if targets:
        for item in targets:
            lines.append(f'- msgtype `{item}`')
    topics = summary.get('rewrite_topics') or []
    if topics:
        for item in topics:
            lines.append(f'- topic `{item}`')

    lines.extend(['', '## Per-Topic Counts', ''])
    for topic, payload in (result.get('per_topic') or {}).items():
        lines.append(
            f"- `{topic}` ({payload.get('msgtype', '')}): "
            f"rewritten=`{payload.get('rewritten', 0)}`, "
            f"passthrough=`{payload.get('passthrough', 0)}`, "
            f"max_delta=`{payload.get('max_stamp_delta_sec', 0.0):.6f}` s"
        )
    return '\n'.join(lines)


def _rewrite_bag(
    *,
    input_bag: Path,
    output_bag: Path,
    rewrite_msgtypes: set[str],
    rewrite_topics: set[str],
) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError('rosbags is required to rewrite bag stamps') from exc

    typestore = get_typestore(Stores.LATEST)
    rewritten = 0
    passthrough = 0
    total = 0
    max_delta_sec = 0.0
    per_topic: dict[str, dict[str, Any]] = {}

    with AnyReader([input_bag], default_typestore=typestore) as reader, Writer(
        output_bag,
        version=9,
    ) as writer:
        output_connections: dict[str, Any] = {}
        for conn in reader.connections:
            kwargs = {
                'typestore': typestore,
                'serialization_format': conn.ext.serialization_format,
                'offered_qos_profiles': conn.ext.offered_qos_profiles,
            }
            if conn.digest and getattr(conn.msgdef, 'data', ''):
                kwargs['msgdef'] = conn.msgdef.data
                kwargs['rihs01'] = conn.digest
            output_connections[conn.topic] = writer.add_connection(
                conn.topic, conn.msgtype, **kwargs,
            )

        for conn, timestamp_ns, raw in reader.messages():
            total += 1
            topic_payload = per_topic.setdefault(
                conn.topic,
                {
                    'msgtype': conn.msgtype,
                    'rewritten': 0,
                    'passthrough': 0,
                    'max_stamp_delta_sec': 0.0,
                },
            )
            should_rewrite = (
                conn.msgtype in rewrite_msgtypes
                or conn.topic in rewrite_topics
            )
            if should_rewrite:
                msg = typestore.deserialize_cdr(raw, conn.msgtype)
                original_stamp_ns = _stamp_ns_from_msg(msg)
                new_sec = int(timestamp_ns // 1_000_000_000)
                new_nanosec = int(timestamp_ns % 1_000_000_000)
                if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                    msg.header.stamp.sec = new_sec
                    msg.header.stamp.nanosec = new_nanosec
                raw = typestore.serialize_cdr(msg, conn.msgtype)
                rewritten += 1
                topic_payload['rewritten'] += 1
                if original_stamp_ns is not None:
                    delta_sec = abs(timestamp_ns - original_stamp_ns) / 1_000_000_000.0
                    if delta_sec > topic_payload['max_stamp_delta_sec']:
                        topic_payload['max_stamp_delta_sec'] = delta_sec
                    if delta_sec > max_delta_sec:
                        max_delta_sec = delta_sec
            else:
                passthrough += 1
                topic_payload['passthrough'] += 1
            writer.write(output_connections[conn.topic], timestamp_ns, raw)

    return {
        'total_messages': total,
        'rewritten_messages': rewritten,
        'passthrough_messages': passthrough,
        'max_stamp_delta_sec': max_delta_sec,
        'per_topic': per_topic,
    }


def _stamp_ns_from_msg(msg: Any) -> int | None:
    header = getattr(msg, 'header', None)
    if header is None:
        return None
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return None
    sec = getattr(stamp, 'sec', None)
    nanosec = getattr(stamp, 'nanosec', None)
    if sec is None or nanosec is None:
        return None
    return int(sec) * 1_000_000_000 + int(nanosec)


def _prepare_output(output_bag: Path, force: bool) -> None:
    if output_bag.exists():
        if not force:
            raise FileExistsError(f'output bag exists (use --force): {output_bag}')
        shutil.rmtree(output_bag)
    output_bag.parent.mkdir(parents=True, exist_ok=True)


def _write_summary(parent_dir: Path, summary: dict[str, Any]) -> dict[str, Path]:
    parent_dir.mkdir(parents=True, exist_ok=True)
    json_path = parent_dir / BAG_STAMP_REWRITER_JSON
    markdown_path = parent_dir / BAG_STAMP_REWRITER_MARKDOWN
    json_path.write_text(payload_to_json(summary) + '\n', encoding='utf-8')
    markdown_path.write_text(
        render_bag_stamp_rewriter_markdown(summary) + '\n',
        encoding='utf-8',
    )
    return {'json': json_path, 'markdown': markdown_path}


def options_payload(options: BagStampRewriterOptions) -> dict[str, Any]:
    """Convert options to a JSON-friendly dict."""
    payload = asdict(options)
    payload['input_bag'] = str(options.input_bag)
    payload['output_bag'] = str(options.output_bag)
    return payload
