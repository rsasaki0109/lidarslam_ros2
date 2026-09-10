#!/usr/bin/env python3
"""Clip public MID-360 bag segments for focused retries."""

from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_public_bag_segments import PUBLIC_BAG_SEGMENTS_JSON
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_BAG_SEGMENT_CLIP_JSON = 'mid360_robot_public_bag_segment_clip.json'
PUBLIC_BAG_SEGMENT_CLIP_MARKDOWN = 'mid360_robot_public_bag_segment_clip.md'


@dataclass(frozen=True)
class PublicBagSegmentClipOptions:
    """Options for clipping one public bag segment."""

    dataset_id: str
    segment_id: str = ''
    output_root: Path = Path('datasets/mid360_public_segments')
    margin_sec: float = 0.0
    force: bool = False
    include_tf: bool = True


class PublicBagSegmentClipper:
    """Clip one segment selected by the public bag segment report."""

    def __init__(self, segments_path: Path) -> None:
        self._segments_path = segments_path.expanduser().resolve()

    def clip(self, options: PublicBagSegmentClipOptions) -> dict[str, Any]:
        """Write a clipped rosbag2 directory and sidecar summary."""
        report = _load_json(self._segments_path)
        row = _find_dataset(report, options.dataset_id)
        segment = _find_segment(row, options.segment_id)
        output_dir = _clip_output_dir(options, row, segment)
        bag_path = output_dir / 'rosbag2'
        _prepare_output_dir(output_dir, bag_path, options.force)

        start_ns, end_ns = _clip_window_ns(segment, options.margin_sec)
        topic_names = _clip_topics(row, options.include_tf)
        copy_summary = _copy_bag_window(
            input_bag=Path(str(row.get('selected_bag_path') or '')).expanduser().resolve(),
            output_bag=bag_path,
            topics=topic_names,
            start_ns=start_ns,
            end_ns=end_ns,
        )
        summary = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': _clip_status(copy_summary, row),
            'segments_path': str(self._segments_path),
            'dataset_id': row.get('dataset_id', ''),
            'segment_id': segment.get('segment_id', ''),
            'source_bag_path': row.get('selected_bag_path', ''),
            'output_dir': str(output_dir),
            'output_bag_path': str(bag_path),
            'margin_sec': max(0.0, float(options.margin_sec)),
            'clip_window': {
                'start_time_ns': start_ns,
                'end_time_ns': end_ns,
                'duration_sec': (end_ns - start_ns) / 1_000_000_000.0,
            },
            'segment': segment,
            'topics': topic_names,
            'copy': copy_summary,
            'suggested_next_steps': _suggested_next_steps(bag_path, output_dir, row),
        }
        _write_summary(output_dir, summary)
        return summary


def render_public_bag_segment_clip_markdown(summary: dict[str, Any]) -> str:
    """Render a clipped segment summary as Markdown."""
    copy_summary = summary.get('copy') or {}
    lines = [
        '# MID-360 Public Bag Segment Clip',
        '',
        f"- status: `{summary.get('status', '')}`",
        f"- created_at: `{summary.get('created_at', '')}`",
        f"- dataset_id: `{summary.get('dataset_id', '')}`",
        f"- segment_id: `{summary.get('segment_id', '')}`",
        f"- source_bag_path: `{summary.get('source_bag_path', '')}`",
        f"- output_bag_path: `{summary.get('output_bag_path', '')}`",
        f"- margin_sec: `{summary.get('margin_sec', '')}`",
        f"- copied_messages: `{copy_summary.get('total_messages', 0)}`",
        '',
        '## Topics',
        '',
    ]
    for topic, count in (copy_summary.get('message_counts') or {}).items():
        lines.append(f"- `{topic}`: `{count}`")

    steps = summary.get('suggested_next_steps') or []
    if steps:
        lines.extend(['', '## Suggested Next Commands', ''])
        for step in steps:
            lines.append(f'- `{step}`')
    return '\n'.join(lines)


def _copy_bag_window(
    input_bag: Path,
    output_bag: Path,
    topics: list[str],
    start_ns: int,
    end_ns: int,
) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError('rosbags is required to clip public bag segments') from exc

    typestore = get_typestore(Stores.LATEST)
    wanted = set(topics)
    message_counts = {topic: 0 for topic in topics}
    first_time_ns: int | None = None
    last_time_ns: int | None = None

    with AnyReader([input_bag], default_typestore=typestore) as reader, Writer(
        output_bag,
        version=9,
    ) as writer:
        input_connections = [conn for conn in reader.connections if conn.topic in wanted]
        output_connections = {}
        for conn in input_connections:
            kwargs = {
                'typestore': typestore,
                'serialization_format': conn.ext.serialization_format,
                'offered_qos_profiles': conn.ext.offered_qos_profiles,
            }
            if conn.digest and getattr(conn.msgdef, 'data', ''):
                kwargs['msgdef'] = conn.msgdef.data
                kwargs['rihs01'] = conn.digest
            output_connections[conn.topic] = writer.add_connection(
                conn.topic,
                conn.msgtype,
                **kwargs,
            )

        for conn, timestamp_ns, raw in reader.messages(connections=input_connections):
            timestamp_ns = int(timestamp_ns)
            if timestamp_ns < start_ns or timestamp_ns > end_ns:
                continue
            writer.write(output_connections[conn.topic], timestamp_ns, raw)
            message_counts[conn.topic] = message_counts.get(conn.topic, 0) + 1
            first_time_ns = timestamp_ns if first_time_ns is None else min(first_time_ns, timestamp_ns)
            last_time_ns = timestamp_ns if last_time_ns is None else max(last_time_ns, timestamp_ns)

    return {
        'input_bag': str(input_bag),
        'output_bag': str(output_bag),
        'requested_topics': topics,
        'message_counts': message_counts,
        'total_messages': sum(message_counts.values()),
        'first_time_ns': first_time_ns,
        'last_time_ns': last_time_ns,
    }


def _clip_status(copy_summary: dict[str, Any], row: dict[str, Any]) -> str:
    counts = copy_summary.get('message_counts') or {}
    topics = row.get('selected_topics') or {}
    required = [str(topics.get('pointcloud') or ''), str(topics.get('imu') or '')]
    if any(not topic or counts.get(topic, 0) <= 0 for topic in required):
        return 'FAIL'
    return 'PASS'


def _write_summary(output_dir: Path, summary: dict[str, Any]) -> dict[str, Path]:
    json_path = output_dir / PUBLIC_BAG_SEGMENT_CLIP_JSON
    markdown_path = output_dir / PUBLIC_BAG_SEGMENT_CLIP_MARKDOWN
    json_path.write_text(payload_to_json(summary) + '\n', encoding='utf-8')
    markdown_path.write_text(render_public_bag_segment_clip_markdown(summary) + '\n',
                             encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def _find_dataset(report: dict[str, Any], dataset_id: str) -> dict[str, Any]:
    for row in report.get('datasets') or []:
        if row.get('dataset_id') == dataset_id:
            return row
    raise ValueError(f'dataset not found in {PUBLIC_BAG_SEGMENTS_JSON}: {dataset_id}')


def _find_segment(row: dict[str, Any], segment_id: str) -> dict[str, Any]:
    if not segment_id:
        segment = row.get('recommended_segment') or {}
        if segment:
            return segment
        raise ValueError(f'dataset has no recommended segment: {row.get("dataset_id", "")}')
    for segment in row.get('segments') or []:
        if segment.get('segment_id') == segment_id:
            return segment
    raise ValueError(f'segment not found for {row.get("dataset_id", "")}: {segment_id}')


def _clip_output_dir(
    options: PublicBagSegmentClipOptions,
    row: dict[str, Any],
    segment: dict[str, Any],
) -> Path:
    return (
        options.output_root.expanduser().resolve()
        / str(row.get('dataset_id') or '')
        / str(segment.get('segment_id') or '')
    )


def _prepare_output_dir(output_dir: Path, bag_path: Path, force: bool) -> None:
    if output_dir.exists():
        if not force:
            raise FileExistsError(f'clip output exists (use --force): {output_dir}')
        shutil.rmtree(output_dir)
    bag_path.parent.mkdir(parents=True, exist_ok=True)


def _clip_window_ns(segment: dict[str, Any], margin_sec: float) -> tuple[int, int]:
    margin_ns = int(max(0.0, float(margin_sec)) * 1_000_000_000)
    start_ns = int(segment['clip_start_time_ns']) - margin_ns
    end_ns = int(segment['clip_end_time_ns']) + margin_ns
    return start_ns, end_ns


def _clip_topics(row: dict[str, Any], include_tf: bool) -> list[str]:
    selected = row.get('selected_topics') or {}
    topics = [
        str(selected.get('pointcloud') or ''),
        str(selected.get('imu') or ''),
    ]
    if include_tf:
        topics.extend(['/tf_static', '/tf'])
    return [topic for index, topic in enumerate(topics) if topic and topic not in topics[:index]]


def _suggested_next_steps(
    bag_path: Path,
    output_dir: Path,
    row: dict[str, Any],
) -> list[str]:
    profile_hint = ''
    dataset_id = str(row.get('dataset_id') or '')
    if dataset_id:
        profile_hint = (
            ' --robot-profile '
            f'datasets/mid360_public/{dataset_id}/{dataset_id}_profile.yaml'
        )
    return [
        f'ros2 bag info {bag_path}',
        (
            f'python3 scripts/check_mid360_robot_readiness.py {bag_path}'
            f'{profile_hint} --output-dir {output_dir} --write-manifest'
        ),
    ]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))
