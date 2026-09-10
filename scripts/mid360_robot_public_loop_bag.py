#!/usr/bin/env python3
"""Build a focused loop rosbag2 from multiple public MID-360 split bags.

Hard Point Cloud Localization `outdoor_kidnap` ships as two zip archives
(`outdoor_kidnap_a.zip` and `outdoor_kidnap_b.zip`); loop closure is only
visible when both halves are replayed in order. This module reads the
input bags sequentially and writes one merged rosbag2 with the original
timestamps, optionally filtered to a focused window around the loop.
"""

from __future__ import annotations

import shutil
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_LOOP_BAG_JSON = 'mid360_robot_public_loop_bag.json'
PUBLIC_LOOP_BAG_MARKDOWN = 'mid360_robot_public_loop_bag.md'


@dataclass(frozen=True)
class PublicLoopBagOptions:
    """Options for merging public split bags into one focused loop bag."""

    input_bags: tuple[Path, ...]
    output_bag: Path
    topics: tuple[str, ...] = ()
    include_tf: bool = True
    time_window_sec: tuple[float, float] | None = None
    force: bool = False
    enforce_monotonic: bool = True
    extra_metadata: dict[str, Any] = field(default_factory=dict)


class PublicLoopBagBuilder:
    """Read N input bags in order and write a single merged rosbag2 directory."""

    def build(self, options: PublicLoopBagOptions) -> dict[str, Any]:
        """Build the merged loop bag and return a summary dict."""
        if not options.input_bags:
            raise ValueError('PublicLoopBagOptions.input_bags must not be empty')
        for bag in options.input_bags:
            if not bag.exists():
                raise FileNotFoundError(f'input bag not found: {bag}')

        output_bag = options.output_bag.expanduser().resolve()
        _prepare_output(output_bag, options.force)

        copy_summary = _copy_bags(
            input_bags=tuple(bag.expanduser().resolve() for bag in options.input_bags),
            output_bag=output_bag,
            requested_topics=list(options.topics),
            include_tf=options.include_tf,
            time_window_sec=options.time_window_sec,
            enforce_monotonic=options.enforce_monotonic,
        )

        summary: dict[str, Any] = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': _status(copy_summary),
            'input_bags': [str(bag) for bag in options.input_bags],
            'output_bag': str(output_bag),
            'requested_topics': list(options.topics),
            'include_tf': options.include_tf,
            'enforce_monotonic': options.enforce_monotonic,
            'time_window_sec': list(options.time_window_sec) if options.time_window_sec else None,
            'copy': copy_summary,
        }
        if options.extra_metadata:
            summary['metadata'] = dict(options.extra_metadata)
        _write_summary(output_bag.parent, summary)
        return summary


def render_public_loop_bag_markdown(summary: dict[str, Any]) -> str:
    """Render a multi-bag loop summary as Markdown."""
    copy_summary = summary.get('copy') or {}
    lines = [
        '# MID-360 Public Loop Bag',
        '',
        f"- status: `{summary.get('status', '')}`",
        f"- created_at: `{summary.get('created_at', '')}`",
        f"- output_bag: `{summary.get('output_bag', '')}`",
        f"- total_messages: `{copy_summary.get('total_messages', 0)}`",
        f"- per_bag_messages: `{copy_summary.get('per_bag_total') or []}`",
        '',
        '## Input Bags',
        '',
    ]
    for bag in summary.get('input_bags') or []:
        lines.append(f'- `{bag}`')
    lines.extend(['', '## Topics', ''])
    for topic, count in (copy_summary.get('message_counts') or {}).items():
        lines.append(f'- `{topic}`: `{count}`')
    window = summary.get('time_window_sec')
    if window:
        lines.extend([
            '',
            '## Time Window (relative to first message)',
            '',
            f'- start_sec: `{window[0]}`',
            f'- end_sec: `{window[1]}`',
        ])
    metadata = summary.get('metadata')
    if metadata:
        lines.extend(['', '## Metadata', ''])
        for key, value in metadata.items():
            lines.append(f'- `{key}`: `{value}`')
    return '\n'.join(lines)


def _copy_bags(
    *,
    input_bags: tuple[Path, ...],
    output_bag: Path,
    requested_topics: list[str],
    include_tf: bool,
    time_window_sec: tuple[float, float] | None,
    enforce_monotonic: bool,
) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError('rosbags is required to build the loop bag') from exc

    typestore = get_typestore(Stores.LATEST)
    wanted = _resolved_topic_set(requested_topics, include_tf)

    message_counts: dict[str, int] = {}
    per_bag_total: list[int] = []
    first_time_ns: int | None = None
    last_time_ns: int | None = None
    bag_first_last: list[dict[str, Any]] = []

    with Writer(output_bag, version=9) as writer:
        output_connections: dict[tuple[str, str], Any] = {}

        for bag_index, input_bag in enumerate(input_bags):
            with AnyReader([input_bag], default_typestore=typestore) as reader:
                bag_first: int | None = None
                bag_last: int | None = None
                bag_total = 0

                input_connections = [
                    conn for conn in reader.connections
                    if (not wanted or conn.topic in wanted)
                ]
                for conn in input_connections:
                    key = (conn.topic, conn.msgtype)
                    if key in output_connections:
                        continue
                    kwargs = {
                        'typestore': typestore,
                        'serialization_format': conn.ext.serialization_format,
                        'offered_qos_profiles': conn.ext.offered_qos_profiles,
                    }
                    if conn.digest and getattr(conn.msgdef, 'data', ''):
                        kwargs['msgdef'] = conn.msgdef.data
                        kwargs['rihs01'] = conn.digest
                    output_connections[key] = writer.add_connection(
                        conn.topic, conn.msgtype, **kwargs,
                    )

                for conn, timestamp_ns, raw in reader.messages(
                    connections=input_connections,
                ):
                    timestamp_ns = int(timestamp_ns)
                    if first_time_ns is None:
                        first_time_ns = timestamp_ns
                    if not _passes_window(timestamp_ns, first_time_ns, time_window_sec):
                        continue
                    if (
                        enforce_monotonic
                        and last_time_ns is not None
                        and timestamp_ns < last_time_ns
                    ):
                        raise ValueError(
                            'input bags overlap in time; pass enforce_monotonic=False to '
                            f'allow rewinding (bag {input_bag}, ts {timestamp_ns} < last {last_time_ns})',
                        )
                    writer.write(
                        output_connections[(conn.topic, conn.msgtype)],
                        timestamp_ns,
                        raw,
                    )
                    message_counts[conn.topic] = message_counts.get(conn.topic, 0) + 1
                    bag_total += 1
                    bag_first = timestamp_ns if bag_first is None else min(bag_first, timestamp_ns)
                    bag_last = timestamp_ns if bag_last is None else max(bag_last, timestamp_ns)
                    last_time_ns = (
                        timestamp_ns if last_time_ns is None else max(last_time_ns, timestamp_ns)
                    )

                per_bag_total.append(bag_total)
                bag_first_last.append({
                    'bag_index': bag_index,
                    'bag_path': str(input_bag),
                    'first_time_ns': bag_first,
                    'last_time_ns': bag_last,
                    'message_count': bag_total,
                })

    return {
        'message_counts': message_counts,
        'per_bag_total': per_bag_total,
        'per_bag_summary': bag_first_last,
        'total_messages': sum(message_counts.values()),
        'first_time_ns': first_time_ns,
        'last_time_ns': last_time_ns,
        'duration_sec': (
            (last_time_ns - first_time_ns) / 1_000_000_000.0
            if first_time_ns is not None and last_time_ns is not None
            else 0.0
        ),
    }


def _resolved_topic_set(requested_topics: list[str], include_tf: bool) -> set[str]:
    if not requested_topics:
        return set()
    topics = set(requested_topics)
    if include_tf:
        topics.update({'/tf_static', '/tf'})
    return topics


def _passes_window(
    timestamp_ns: int,
    first_time_ns: int,
    time_window_sec: tuple[float, float] | None,
) -> bool:
    if not time_window_sec:
        return True
    start_sec, end_sec = time_window_sec
    offset_sec = (timestamp_ns - first_time_ns) / 1_000_000_000.0
    if start_sec is not None and offset_sec < start_sec:
        return False
    if end_sec is not None and offset_sec > end_sec:
        return False
    return True


def _status(copy_summary: dict[str, Any]) -> str:
    return 'PASS' if copy_summary.get('total_messages', 0) > 0 else 'FAIL'


def _prepare_output(output_bag: Path, force: bool) -> None:
    if output_bag.exists():
        if not force:
            raise FileExistsError(f'output bag exists (use --force): {output_bag}')
        shutil.rmtree(output_bag)
    output_bag.parent.mkdir(parents=True, exist_ok=True)


def _write_summary(parent_dir: Path, summary: dict[str, Any]) -> dict[str, Path]:
    parent_dir.mkdir(parents=True, exist_ok=True)
    json_path = parent_dir / PUBLIC_LOOP_BAG_JSON
    markdown_path = parent_dir / PUBLIC_LOOP_BAG_MARKDOWN
    json_path.write_text(payload_to_json(summary) + '\n', encoding='utf-8')
    markdown_path.write_text(
        render_public_loop_bag_markdown(summary) + '\n',
        encoding='utf-8',
    )
    return {'json': json_path, 'markdown': markdown_path}


def options_payload(options: PublicLoopBagOptions) -> dict[str, Any]:
    """Convert PublicLoopBagOptions to a JSON-friendly dict."""
    payload = asdict(options)
    payload['input_bags'] = [str(bag) for bag in options.input_bags]
    payload['output_bag'] = str(options.output_bag)
    return payload
