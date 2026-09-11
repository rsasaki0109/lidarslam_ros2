#!/usr/bin/env python3
"""Merge split public MID-360 rosbag2 sqlite bags without deserializing messages."""

from __future__ import annotations

import shutil
import sqlite3
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_SPLIT_BAG_MERGE_JSON = 'mid360_robot_public_split_bag_merge.json'
PUBLIC_SPLIT_BAG_MERGE_MARKDOWN = 'mid360_robot_public_split_bag_merge.md'


@dataclass(frozen=True)
class SplitBagMergeOptions:
    """Options for raw sqlite rosbag2 split-bag merging."""

    input_bags: tuple[Path, ...]
    output_bag: Path
    force: bool = False
    output_db_name: str = 'merged_0.db3'


class SplitBagMerger:
    """Merge rosbag2 sqlite directories into one output bag."""

    def merge(self, options: SplitBagMergeOptions) -> dict[str, Any]:
        """Merge split bags and return a report."""
        input_bags = tuple(path.expanduser().resolve() for path in options.input_bags)
        output_bag = options.output_bag.expanduser().resolve()
        if len(input_bags) < 2:
            raise ValueError('at least two input bags are required')
        _prepare_output(output_bag, options.force)

        input_infos = [_read_bag_info(path) for path in input_bags]
        inter_bag_gaps = _validate_monotonic_ranges(input_infos)
        schema = _validate_schema(input_infos)
        topics = _merge_topics(input_infos)
        topic_id_by_signature = {
            _topic_signature(topic): index + 1
            for index, topic in enumerate(topics)
        }

        output_db = output_bag / options.output_db_name
        copy_summary = _copy_messages(
            output_db=output_db,
            input_infos=input_infos,
            topic_id_by_signature=topic_id_by_signature,
            topics=topics,
            schema=schema,
        )
        metadata = _metadata_payload(
            db_name=options.output_db_name,
            topics=topics,
            copy_summary=copy_summary,
        )
        (output_bag / 'metadata.yaml').write_text(
            yaml.safe_dump(metadata, sort_keys=False),
            encoding='utf-8',
        )
        report = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'PASS' if copy_summary['total_messages'] > 0 else 'FAIL',
            'input_bags': [str(path) for path in input_bags],
            'output_bag': str(output_bag),
            'output_db': str(output_db),
            'schema': schema,
            'inter_bag_gaps': inter_bag_gaps,
            'topics': [
                {
                    'name': topic['name'],
                    'type': topic['type'],
                    'serialization_format': topic['serialization_format'],
                    'message_count': copy_summary['message_counts'].get(topic['name'], 0),
                }
                for topic in topics
            ],
            'copy': copy_summary,
            'metadata_yaml': str(output_bag / 'metadata.yaml'),
            'suggested_next_steps': [
                f'ros2 bag info {output_bag}',
                f'bash scripts/run_mid360_robot_map.sh {output_bag} --dry-run',
            ],
        }
        _write_report(output_bag, report)
        return report


def render_split_bag_merge_markdown(report: dict[str, Any]) -> str:
    """Render split-bag merge report as Markdown."""
    copy = report.get('copy') or {}
    lines = [
        '# MID-360 Public Split Bag Merge',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- output_bag: `{report.get('output_bag', '')}`",
        f"- total_messages: `{copy.get('total_messages', 0)}`",
        f"- start_time_ns: `{copy.get('start_time_ns', '')}`",
        f"- end_time_ns: `{copy.get('end_time_ns', '')}`",
        f"- duration_sec: `{copy.get('duration_sec', 0):.3f}`",
        '',
        '## Input Bags',
        '',
    ]
    lines.extend(f'- `{path}`' for path in report.get('input_bags') or [])
    gaps = report.get('inter_bag_gaps') or []
    if gaps:
        lines.extend(['', '## Inter-Bag Gaps', ''])
        for gap in gaps:
            lines.append(
                f"- `{gap.get('previous_bag', '')}` -> `{gap.get('current_bag', '')}`: "
                f"`{gap.get('gap_sec', 0.0):.6f}` s"
            )
    lines.extend(['', '## Topics', ''])
    for topic in report.get('topics') or []:
        lines.append(
            f"- `{topic.get('name', '')}` `{topic.get('type', '')}`: "
            f"`{topic.get('message_count', 0)}`"
        )
    lines.extend(['', '## Suggested Next Steps', ''])
    steps = report.get('suggested_next_steps') or []
    if steps:
        lines.extend(f'- `{step}`' for step in steps)
    else:
        lines.append('- none')
    return '\n'.join(lines)


def _prepare_output(output_bag: Path, force: bool) -> None:
    if output_bag.exists():
        if not force:
            raise FileExistsError(f'output bag already exists: {output_bag}')
        shutil.rmtree(output_bag)
    output_bag.mkdir(parents=True)


def _read_bag_info(path: Path) -> dict[str, Any]:
    metadata_path = path / 'metadata.yaml'
    if not metadata_path.is_file():
        raise FileNotFoundError(f'metadata.yaml not found: {path}')
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    info = metadata.get('rosbag2_bagfile_information') or {}
    db_paths = [path / item for item in info.get('relative_file_paths') or []]
    if not db_paths:
        db_paths = sorted(path.glob('*.db3'))
    if len(db_paths) != 1:
        raise ValueError(f'exactly one sqlite db per input bag is supported: {path}')
    db_path = db_paths[0]
    if not db_path.is_file():
        raise FileNotFoundError(f'sqlite db not found: {db_path}')
    start_ns, end_ns, count = _message_range(db_path)
    topics = _topics(db_path)
    return {
        'bag_path': path,
        'db_path': db_path,
        'metadata': info,
        'schema': _schema_info(db_path),
        'topics': topics,
        'start_time_ns': start_ns,
        'end_time_ns': end_ns,
        'message_count': count,
    }


def _message_range(db_path: Path) -> tuple[int, int, int]:
    with sqlite3.connect(db_path) as connection:
        start_ns, end_ns, count = connection.execute(
            'select min(timestamp), max(timestamp), count(*) from messages'
        ).fetchone()
    if count <= 0:
        return 0, 0, 0
    return int(start_ns), int(end_ns), int(count)


def _topics(db_path: Path) -> list[dict[str, Any]]:
    with sqlite3.connect(db_path) as connection:
        rows = connection.execute(
            'select id, name, type, serialization_format, offered_qos_profiles '
            'from topics order by id'
        ).fetchall()
    return [
        {
            'id': int(row[0]),
            'name': str(row[1]),
            'type': str(row[2]),
            'serialization_format': str(row[3]),
            'offered_qos_profiles': str(row[4] or ''),
        }
        for row in rows
    ]


def _schema_info(db_path: Path) -> dict[str, Any]:
    with sqlite3.connect(db_path) as connection:
        row = connection.execute(
            'select schema_version, ros_distro from schema order by schema_version limit 1'
        ).fetchone()
    if row is None:
        return {'schema_version': 0, 'ros_distro': ''}
    return {
        'schema_version': int(row[0]),
        'ros_distro': str(row[1]),
    }


def _validate_schema(input_infos: list[dict[str, Any]]) -> dict[str, Any]:
    schemas = [info['schema'] for info in input_infos]
    first = schemas[0]
    for schema in schemas[1:]:
        if schema != first:
            raise ValueError(f'input bag sqlite schema differs: {first} vs {schema}')
    return dict(first)


def _validate_monotonic_ranges(input_infos: list[dict[str, Any]]) -> list[dict[str, Any]]:
    gaps: list[dict[str, Any]] = []
    for previous, current in zip(input_infos, input_infos[1:]):
        if previous['end_time_ns'] > current['start_time_ns']:
            raise ValueError(
                'input bag timestamps overlap or are not sorted: '
                f"{previous['bag_path']} then {current['bag_path']}"
            )
        gap_ns = int(current['start_time_ns']) - int(previous['end_time_ns'])
        gaps.append({
            'previous_bag': str(previous['bag_path']),
            'current_bag': str(current['bag_path']),
            'previous_end_time_ns': int(previous['end_time_ns']),
            'current_start_time_ns': int(current['start_time_ns']),
            'gap_ns': gap_ns,
            'gap_sec': gap_ns / 1_000_000_000.0,
        })
    return gaps


def _merge_topics(input_infos: list[dict[str, Any]]) -> list[dict[str, Any]]:
    topics_by_signature: dict[tuple[str, str, str], dict[str, Any]] = {}
    for info in input_infos:
        for topic in info['topics']:
            signature = _topic_signature(topic)
            existing = topics_by_signature.get(signature)
            if existing is None:
                topics_by_signature[signature] = topic
                continue
            if existing['offered_qos_profiles'] != topic['offered_qos_profiles']:
                raise ValueError(f'topic QoS differs across split bags: {topic["name"]}')
    return sorted(topics_by_signature.values(), key=lambda item: item['name'])


def _topic_signature(topic: dict[str, Any]) -> tuple[str, str, str]:
    return (
        str(topic['name']),
        str(topic['type']),
        str(topic['serialization_format']),
    )


def _copy_messages(
    *,
    output_db: Path,
    input_infos: list[dict[str, Any]],
    topic_id_by_signature: dict[tuple[str, str, str], int],
    topics: list[dict[str, Any]],
    schema: dict[str, Any],
) -> dict[str, Any]:
    output_db.parent.mkdir(parents=True, exist_ok=True)
    message_counts = {topic['name']: 0 for topic in topics}
    source_counts = []
    start_time_ns: int | None = None
    end_time_ns: int | None = None
    next_id = 1
    with sqlite3.connect(output_db) as output:
        _create_schema(output)
        for topic in topics:
            output.execute(
                'insert into topics(id, name, type, serialization_format, offered_qos_profiles) '
                'values (?, ?, ?, ?, ?)',
                (
                    topic_id_by_signature[_topic_signature(topic)],
                    topic['name'],
                    topic['type'],
                    topic['serialization_format'],
                    topic['offered_qos_profiles'],
                ),
            )
        topic_name_by_output_id = {
            topic_id_by_signature[_topic_signature(topic)]: topic['name']
            for topic in topics
        }
        for info in input_infos:
            topic_id_map = {
                topic['id']: topic_id_by_signature[_topic_signature(topic)]
                for topic in info['topics']
            }
            copied = 0
            with sqlite3.connect(info['db_path']) as source:
                rows = source.execute(
                    'select topic_id, timestamp, data from messages order by timestamp, id'
                )
                for topic_id, timestamp, data in rows:
                    mapped_topic_id = topic_id_map[int(topic_id)]
                    output.execute(
                        'insert into messages(id, topic_id, timestamp, data) values (?, ?, ?, ?)',
                        (next_id, mapped_topic_id, int(timestamp), data),
                    )
                    next_id += 1
                    copied += 1
                    topic_name = topic_name_by_output_id[mapped_topic_id]
                    message_counts[topic_name] += 1
                    start_time_ns = int(timestamp) if start_time_ns is None else min(start_time_ns, int(timestamp))
                    end_time_ns = int(timestamp) if end_time_ns is None else max(end_time_ns, int(timestamp))
            source_counts.append({
                'bag_path': str(info['bag_path']),
                'messages': copied,
                'start_time_ns': info['start_time_ns'],
                'end_time_ns': info['end_time_ns'],
            })
        output.execute(
            'insert into schema(schema_version, ros_distro) values (?, ?)',
            (int(schema['schema_version']), str(schema['ros_distro'])),
        )
        output.execute('create index timestamp_idx on messages (timestamp ASC)')
        output.commit()
    total = next_id - 1
    start_time_ns = start_time_ns or 0
    end_time_ns = end_time_ns or start_time_ns
    return {
        'total_messages': total,
        'message_counts': message_counts,
        'source_counts': source_counts,
        'start_time_ns': start_time_ns,
        'end_time_ns': end_time_ns,
        'duration_sec': (end_time_ns - start_time_ns) / 1_000_000_000.0,
    }


def _create_schema(connection: sqlite3.Connection) -> None:
    connection.execute(
        'create table topics('
        'id integer primary key,'
        'name text not null,'
        'type text not null,'
        'serialization_format text not null,'
        'offered_qos_profiles text not null)'
    )
    connection.execute(
        'create table messages('
        'id integer primary key,'
        'topic_id integer not null,'
        'timestamp integer not null,'
        'data blob not null)'
    )
    connection.execute(
        'create table schema('
        'schema_version integer primary key,'
        'ros_distro text not null)'
    )
    connection.execute(
        'create table metadata('
        'id integer primary key,'
        'metadata_version integer not null,'
        'metadata text not null)'
    )


def _metadata_payload(
    *,
    db_name: str,
    topics: list[dict[str, Any]],
    copy_summary: dict[str, Any],
) -> dict[str, Any]:
    start_time_ns = int(copy_summary['start_time_ns'])
    duration_ns = int(copy_summary['end_time_ns']) - start_time_ns
    return {
        'rosbag2_bagfile_information': {
            'version': 5,
            'storage_identifier': 'sqlite3',
            'duration': {'nanoseconds': duration_ns},
            'starting_time': {'nanoseconds_since_epoch': start_time_ns},
            'message_count': int(copy_summary['total_messages']),
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': topic['name'],
                        'type': topic['type'],
                        'serialization_format': topic['serialization_format'],
                        'offered_qos_profiles': topic['offered_qos_profiles'],
                    },
                    'message_count': int(copy_summary['message_counts'].get(topic['name'], 0)),
                }
                for topic in topics
            ],
            'compression_format': '',
            'compression_mode': '',
            'relative_file_paths': [db_name],
            'files': [
                {
                    'path': db_name,
                    'starting_time': {'nanoseconds_since_epoch': start_time_ns},
                    'duration': {'nanoseconds': duration_ns},
                    'message_count': int(copy_summary['total_messages']),
                }
            ],
        }
    }


def _write_report(output_bag: Path, report: dict[str, Any]) -> None:
    (output_bag / PUBLIC_SPLIT_BAG_MERGE_JSON).write_text(
        payload_to_json(report) + '\n',
        encoding='utf-8',
    )
    (output_bag / PUBLIC_SPLIT_BAG_MERGE_MARKDOWN).write_text(
        render_split_bag_merge_markdown(report) + '\n',
        encoding='utf-8',
    )


if __name__ == "__main__":
    raise SystemExit("mid360_robot_public_split_bag_merge is a library module; import it instead of running it directly.")
