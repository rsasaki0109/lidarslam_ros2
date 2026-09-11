#!/usr/bin/env python3
"""Preflight a rosbag2 for Autoware-compatible map authoring workflows."""

import argparse
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import shlex
import sys
import textwrap
from typing import Any, Callable

import yaml

try:
    from product_profiles import PROFILE_HELP
except ModuleNotFoundError as exc:
    if exc.name != 'product_profiles':
        raise
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    try:
        from product_profiles import PROFILE_HELP
    finally:
        sys.path.pop(0)


POINTCLOUD2 = 'sensor_msgs/msg/PointCloud2'
IMU = 'sensor_msgs/msg/Imu'
NAVSATFIX = 'sensor_msgs/msg/NavSatFix'
ODOMETRY = 'nav_msgs/msg/Odometry'
VELODYNE_SCAN = 'velodyne_msgs/msg/VelodyneScan'
TFMESSAGE = 'tf2_msgs/msg/TFMessage'
GSOF49 = 'applanix_msgs/msg/NavigationSolutionGsof49'
GSOF50 = 'applanix_msgs/msg/NavigationPerformanceGsof50'
VELOCITY_REPORT = 'autoware_auto_vehicle_msgs/msg/VelocityReport'
SCHEMA_VERSION = 6
SCHEMA_URI = 'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/preflight-v6.schema.json'
PUBLIC_EVIDENCE_SCHEMA_VERSION = 1
PUBLIC_EVIDENCE_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/public-doctor-evidence-v1.schema.json'
)
POINT_FIELD_UINT32 = 6
POINT_FIELD_FLOAT32 = 7
POINT_FIELD_FLOAT64 = 8
RKO_TIMESTAMP_FIELDS = ('t', 'timestamp', 'time', 'stamps')
RKO_TIMESTAMP_DATATYPES = (
    POINT_FIELD_UINT32,
    POINT_FIELD_FLOAT32,
    POINT_FIELD_FLOAT64,
)
TIMESTAMP_SCAN_MSGTYPES = (POINTCLOUD2, IMU)
MAX_TIMESTAMP_RECORDS_PER_TOPIC = 100_000
MAX_ODOMETRY_TF_RECORDS_PER_TOPIC = 100_000
MAX_ODOMETRY_TF_TIMING_RECORDS_PER_TOPIC = 100_000
ROSBAG_STORAGE_LOGGER = 'rosbag2_storage'


@contextmanager
def _product_rosbag_open_logging():
    """Hide routine storage INFO during product opens, preserving diagnostics."""
    if not os.environ.get('LIDARSLAM_CLI_COMMAND'):
        yield
        return

    try:
        from rclpy.logging import (
            LoggingSeverity,
            get_logger_level,
            set_logger_level,
        )
    except ImportError:
        yield
        return

    previous_level = get_logger_level(ROSBAG_STORAGE_LOGGER)
    changed = previous_level == LoggingSeverity.UNSET
    if changed:
        set_logger_level(ROSBAG_STORAGE_LOGGER, LoggingSeverity.WARN)
    try:
        yield
    finally:
        if changed:
            set_logger_level(ROSBAG_STORAGE_LOGGER, previous_level)


def _open_rosbag_reader(rosbag2_py, bag_path: Path, storage_id: str):
    """Open one native reader with the product-only storage log boundary."""
    reader = rosbag2_py.SequentialReader()
    with _product_rosbag_open_logging():
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(bag_path),
                storage_id=storage_id,
            ),
            rosbag2_py.ConverterOptions('', ''),
        )
    return reader


def _finding(code: str, message: str, next_action: str) -> dict[str, str]:
    """Build one stable, actionable preflight rejection finding."""
    return {
        'code': code,
        'message': message,
        'next_action': next_action,
    }


@dataclass(frozen=True)
class TopicRecord:
    """Topic metadata relevant to bag preflight."""

    name: str
    msg_type: str
    message_count: int


def _safe_quote(value: str) -> str:
    return shlex.quote(value)


def _topic_from_entry(entry: dict[str, Any]) -> TopicRecord:
    topic_metadata = entry.get('topic_metadata', {}) or {}
    return TopicRecord(
        name=str(topic_metadata.get('name', '')),
        msg_type=str(topic_metadata.get('type', '')),
        message_count=int(entry.get('message_count', 0)),
    )


def load_bag_metadata(bag_path: Path) -> dict[str, Any]:
    """Load rosbag2 metadata.yaml."""
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        raise FileNotFoundError(
            f'metadata.yaml not found under {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )
    try:
        data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    except yaml.YAMLError as exc:
        raise ValueError(f'failed to parse {metadata_path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError(
            f'{metadata_path} must contain a rosbag2 metadata YAML mapping.'
        )
    bag_info = data.get('rosbag2_bagfile_information', {}) or {}
    if not isinstance(bag_info, dict) or not bag_info:
        raise ValueError(
            f'rosbag2_bagfile_information missing in {metadata_path}. '
            'Use a rosbag2 metadata.yaml file, not a topic list or arbitrary YAML file.'
        )
    return bag_info


def _duration_seconds(bag_info: dict[str, Any]) -> float | None:
    duration = bag_info.get('duration', {}) or {}
    nanoseconds = duration.get('nanoseconds')
    if nanoseconds is None:
        return None
    return float(nanoseconds) / 1e9


def _collect_topics(bag_info: dict[str, Any]) -> list[TopicRecord]:
    topics = []
    for entry in bag_info.get('topics_with_message_count', []) or []:
        if not isinstance(entry, dict):
            continue
        record = _topic_from_entry(entry)
        if record.name and record.msg_type:
            topics.append(record)
    return topics


def _topic_group(records: list[TopicRecord], msg_type: str) -> list[TopicRecord]:
    return sorted(
        [record for record in records if record.msg_type == msg_type],
        key=lambda item: (-item.message_count, item.name),
    )


def _best_topic(records: list[TopicRecord], msg_type: str) -> TopicRecord | None:
    grouped = _topic_group(records, msg_type)
    return grouped[0] if grouped else None


def assess_pointcloud_fields(fields: list[dict[str, Any]]) -> dict[str, Any]:
    """Assess whether PointCloud2 fields satisfy RKO-LIO's reader contract."""
    normalized = [
        {
            'name': str(field.get('name', '')),
            'datatype': int(field.get('datatype', 0)),
            'count': int(field.get('count', 0)),
        }
        for field in fields
    ]
    by_name = {field['name']: field for field in normalized}

    invalid_xyz = [
        name
        for name in ('x', 'y', 'z')
        if name not in by_name
        or by_name[name]['datatype'] != POINT_FIELD_FLOAT32
        or by_name[name]['count'] <= 0
    ]
    if invalid_xyz:
        return {
            'rko_lio_compatible': False,
            'timestamp_field': None,
            'reason': (
                'RKO-LIO requires x, y, and z PointCloud2 fields with FLOAT32 '
                f'datatype and positive count; invalid or missing: {", ".join(invalid_xyz)}.'
            ),
        }

    timestamp_candidates = [
        by_name[name] for name in RKO_TIMESTAMP_FIELDS if name in by_name
    ]
    for field in timestamp_candidates:
        if (
            field['datatype'] in RKO_TIMESTAMP_DATATYPES
            and field['count'] > 0
        ):
            return {
                'rko_lio_compatible': True,
                'timestamp_field': field['name'],
                'reason': (
                    f'RKO-LIO-compatible per-point timestamp field '
                    f'{field["name"]!r} was found.'
                ),
            }

    expected = '/'.join(RKO_TIMESTAMP_FIELDS)
    if timestamp_candidates:
        return {
            'rko_lio_compatible': False,
            'timestamp_field': None,
            'reason': (
                f'PointCloud2 timestamp field must be one of {expected}, have '
                'positive count, and use UINT32, FLOAT32, or FLOAT64.'
            ),
        }
    return {
        'rko_lio_compatible': False,
        'timestamp_field': None,
        'reason': (
            'PointCloud2 has no per-point timestamp field required for '
            f'RKO-LIO deskewing (expected {expected}).'
        ),
    }


def inspect_pointcloud_record(
    bag_path: Path,
    topic: str,
    storage_id: str,
) -> dict[str, Any]:
    """Read the first selected PointCloud2 record and inspect its fields."""
    base = {
        'topic': topic,
        'frame_id': None,
        'fields': [],
        'rko_lio_compatible': None,
        'timestamp_field': None,
    }
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        return _inspect_pointcloud_record_with_rosbags(
            bag_path,
            topic,
            ros_import_error=exc,
        )

    try:
        reader = _open_rosbag_reader(rosbag2_py, bag_path, storage_id)
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        while reader.has_next():
            record_topic, serialized, _ = reader.read_next()
            if record_topic != topic:
                continue
            message = deserialize_message(serialized, PointCloud2)
            fields = [
                {
                    'name': field.name,
                    'datatype': field.datatype,
                    'count': field.count,
                }
                for field in message.fields
            ]
            assessment = assess_pointcloud_fields(fields)
            return {
                **base,
                **assessment,
                'status': 'inspected',
                'frame_id': message.header.frame_id.strip() or None,
                'fields': fields,
            }
    except Exception as exc:  # rosbag2 storage plugins expose backend-specific errors
        return {
            **base,
            'status': 'error',
            'reason': f'Failed to inspect PointCloud2 record on {topic}: {exc}',
        }

    return {
        **base,
        'status': 'empty',
        'reason': f'No PointCloud2 record was found on selected topic {topic}.',
    }


def _timestamp_scan_state(
    topics: list[TopicRecord],
    max_records_per_topic: int,
) -> dict[str, dict[str, Any]]:
    return {
        topic.name: {
            'topic': topic.name,
            'msg_type': topic.msg_type,
            'expected_records': topic.message_count,
            'records_scanned': 0,
            'complete': False,
            'first_stamp_ns': None,
            'last_stamp_ns': None,
            'reversal_count': 0,
            'invalid_stamp_count': 0,
            'max_backward_jump_ns': 0,
            '_frame_ids': set(),
            '_limit': max_records_per_topic,
        }
        for topic in topics
    }


def _stamp_ns_from_message(message: Any) -> int | None:
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    sec = getattr(stamp, 'sec', None)
    nanosec = getattr(stamp, 'nanosec', None)
    if sec is None or nanosec is None:
        return None
    sec = int(sec)
    nanosec = int(nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nanosec


def _record_timestamp_sample(
    state: dict[str, Any],
    stamp_ns: int | None,
    frame_id: str | None = None,
) -> None:
    if state['records_scanned'] >= state['_limit']:
        return
    state['records_scanned'] += 1
    if frame_id and frame_id.strip():
        state['_frame_ids'].add(frame_id.strip())
    if stamp_ns is None:
        state['invalid_stamp_count'] += 1
        return
    if state['first_stamp_ns'] is None:
        state['first_stamp_ns'] = stamp_ns
    previous = state['last_stamp_ns']
    if previous is not None and stamp_ns < previous:
        state['reversal_count'] += 1
        state['max_backward_jump_ns'] = max(
            state['max_backward_jump_ns'],
            previous - stamp_ns,
        )
    state['last_stamp_ns'] = stamp_ns


def _timestamp_scan_satisfied(state: dict[str, Any]) -> bool:
    expected = state['expected_records']
    goal = min(
        expected if expected > 0 else state['_limit'],
        state['_limit'],
    )
    return state['records_scanned'] >= goal


def _finalize_timestamp_scan(
    states: dict[str, dict[str, Any]],
    *,
    exhausted: bool,
    max_records_per_topic: int,
) -> dict[str, Any]:
    topics = []
    failed_topics = []
    for state in states.values():
        expected = state['expected_records']
        state['complete'] = (
            exhausted
            or (expected > 0 and state['records_scanned'] >= expected)
        )
        state['readable'] = state['records_scanned'] > 0
        frame_ids = sorted(state.pop('_frame_ids'))
        state['frame_id'] = frame_ids[0] if len(frame_ids) == 1 else None
        state['frame_ids'] = frame_ids
        state.pop('_limit')
        if (
            not state['readable']
            or state['reversal_count']
            or state['invalid_stamp_count']
        ):
            failed_topics.append(state['topic'])
        topics.append(state)

    if failed_topics:
        status = 'failed'
        reason = (
            'Unreadable, non-monotonic, or invalid header timestamps '
            'were detected on: '
            + ', '.join(failed_topics)
            + '. Correct or rewrite the bag before mapping.'
        )
    elif all(topic['complete'] for topic in topics):
        status = 'passed'
        reason = 'All selected PointCloud2/Imu header timestamps are monotonic.'
    else:
        status = 'sampled'
        reason = (
            'No reversal was found in the bounded header timestamp sample; '
            'at least one selected topic was not scanned completely.'
        )
    return {
        'status': status,
        'timestamp_source': 'header.stamp',
        'max_records_per_topic': max_records_per_topic,
        'failed_topics': failed_topics,
        'topics': topics,
        'reason': reason,
    }


def inspect_timestamp_order(
    bag_path: Path,
    topics: list[TopicRecord],
    storage_id: str,
    max_records_per_topic: int = MAX_TIMESTAMP_RECORDS_PER_TOPIC,
) -> dict[str, Any]:
    """Boundedly inspect PointCloud2/Imu header timestamps before launch."""
    selected = [
        topic for topic in topics
        if topic.msg_type in TIMESTAMP_SCAN_MSGTYPES
    ]
    if not selected:
        return {
            'status': 'not_applicable',
            'timestamp_source': 'header.stamp',
            'max_records_per_topic': max_records_per_topic,
            'failed_topics': [],
            'topics': [],
            'reason': 'No PointCloud2 or Imu topic was selected for inspection.',
        }
    if max_records_per_topic <= 0:
        raise ValueError('max_records_per_topic must be positive')

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu, PointCloud2
    except ImportError as exc:
        return _inspect_timestamp_order_with_rosbags(
            bag_path,
            selected,
            max_records_per_topic,
            ros_import_error=exc,
        )

    message_types = {
        POINTCLOUD2: PointCloud2,
        IMU: Imu,
    }
    states = _timestamp_scan_state(selected, max_records_per_topic)
    try:
        reader = _open_rosbag_reader(rosbag2_py, bag_path, storage_id)
        reader.set_filter(
            rosbag2_py.StorageFilter(topics=list(states))
        )
        exhausted = True
        while reader.has_next():
            if all(_timestamp_scan_satisfied(state) for state in states.values()):
                exhausted = False
                break
            topic, serialized, _ = reader.read_next()
            state = states.get(topic)
            if state is None or _timestamp_scan_satisfied(state):
                continue
            message = deserialize_message(
                serialized,
                message_types[state['msg_type']],
            )
            _record_timestamp_sample(
                state,
                _stamp_ns_from_message(message),
                getattr(getattr(message, 'header', None), 'frame_id', None),
            )
        return _finalize_timestamp_scan(
            states,
            exhausted=exhausted,
            max_records_per_topic=max_records_per_topic,
        )
    except Exception as exc:  # storage plugins expose backend-specific errors
        return {
            'status': 'error',
            'timestamp_source': 'header.stamp',
            'max_records_per_topic': max_records_per_topic,
            'failed_topics': [],
            'topics': [],
            'reason': f'Failed to inspect header timestamp order: {exc}',
        }


def _inspect_timestamp_order_with_rosbags(
    bag_path: Path,
    topics: list[TopicRecord],
    max_records_per_topic: int,
    ros_import_error: ImportError,
) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as rosbags_error:
        return {
            'status': 'unavailable',
            'timestamp_source': 'header.stamp',
            'max_records_per_topic': max_records_per_topic,
            'failed_topics': [],
            'topics': [],
            'reason': (
                'Header timestamp inspection is unavailable: ROS 2 Python '
                f'bindings failed to import ({ros_import_error}); the rosbags '
                f'fallback also failed to import ({rosbags_error}).'
            ),
        }

    states = _timestamp_scan_state(topics, max_records_per_topic)
    try:
        typestore = get_typestore(Stores.LATEST)
        with AnyReader(
            [bag_path],
            default_typestore=typestore,
        ) as reader:
            connections = [
                connection
                for connection in reader.connections
                if connection.topic in states
                and connection.msgtype == states[connection.topic]['msg_type']
            ]
            exhausted = True
            for connection, _, serialized in reader.messages(
                connections=connections,
            ):
                if all(
                    _timestamp_scan_satisfied(state)
                    for state in states.values()
                ):
                    exhausted = False
                    break
                state = states[connection.topic]
                if _timestamp_scan_satisfied(state):
                    continue
                message = reader.deserialize(
                    serialized,
                    connection.msgtype,
                )
                _record_timestamp_sample(
                    state,
                    _stamp_ns_from_message(message),
                    getattr(getattr(message, 'header', None), 'frame_id', None),
                )
        return _finalize_timestamp_scan(
            states,
            exhausted=exhausted,
            max_records_per_topic=max_records_per_topic,
        )
    except Exception as exc:
        return {
            'status': 'error',
            'timestamp_source': 'header.stamp',
            'max_records_per_topic': max_records_per_topic,
            'failed_topics': [],
            'topics': [],
            'reason': (
                'Failed to inspect header timestamp order with rosbags: '
                f'{exc}'
            ),
        }


def _inspect_pointcloud_record_with_rosbags(
    bag_path: Path,
    topic: str,
    ros_import_error: ImportError,
) -> dict[str, Any]:
    """Use the pure-Python rosbags reader when ROS Python bindings are absent."""
    base = {
        'topic': topic,
        'frame_id': None,
        'fields': [],
        'rko_lio_compatible': None,
        'timestamp_field': None,
    }
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as rosbags_error:
        return {
            **base,
            'status': 'unavailable',
            'reason': (
                'PointCloud2 record inspection is unavailable: ROS 2 Python '
                f'bindings failed to import ({ros_import_error}); the rosbags '
                f'fallback also failed to import ({rosbags_error}). Source ROS 2 '
                'or install the Python rosbags package before selecting an '
                'RKO-LIO profile.'
            ),
        }

    try:
        typestore = get_typestore(Stores.LATEST)
        with AnyReader(
            [bag_path],
            default_typestore=typestore,
        ) as reader:
            connections = [
                connection
                for connection in reader.connections
                if connection.topic == topic and connection.msgtype == POINTCLOUD2
            ]
            for connection, _, serialized in reader.messages(
                connections=connections,
            ):
                message = reader.deserialize(serialized, connection.msgtype)
                fields = [
                    {
                        'name': field.name,
                        'datatype': int(field.datatype),
                        'count': int(field.count),
                    }
                    for field in message.fields
                ]
                assessment = assess_pointcloud_fields(fields)
                return {
                    **base,
                    **assessment,
                    'status': 'inspected',
                    'frame_id': str(message.header.frame_id).strip() or None,
                    'fields': fields,
                }
    except Exception as exc:
        return {
            **base,
            'status': 'error',
            'reason': (
                f'Failed to inspect PointCloud2 record on {topic} with rosbags: '
                f'{exc}'
            ),
        }

    return {
        **base,
        'status': 'empty',
        'reason': f'No PointCloud2 record was found on selected topic {topic}.',
    }


def _odometry_tf_state(
    odometry_topic: TopicRecord,
    tf_topics: list[TopicRecord],
    max_records_per_topic: int,
) -> dict[str, Any]:
    """Build bounded mutable state for one Odometry-to-TF inspection."""
    return {
        'odometry_topic': odometry_topic.name,
        'expected_records': odometry_topic.message_count,
        'records_scanned': 0,
        '_frame_ids': set(),
        '_child_frame_ids': set(),
        'invalid_frame_count': 0,
        '_tf_topics': {
            topic.name: {
                'topic': topic.name,
                'expected_records': topic.message_count,
                'records_scanned': 0,
                'complete': False,
                'readable': False,
            }
            for topic in tf_topics
        },
        '_edges': {},
        'invalid_transform_count': 0,
        '_limit': max_records_per_topic,
    }


def _bounded_topic_satisfied(records_scanned: int, expected: int, limit: int) -> bool:
    goal = min(expected if expected > 0 else limit, limit)
    return records_scanned >= goal


def _odometry_tf_scan_satisfied(state: dict[str, Any]) -> bool:
    limit = state['_limit']
    if not _bounded_topic_satisfied(
        state['records_scanned'],
        state['expected_records'],
        limit,
    ):
        return False
    return all(
        _bounded_topic_satisfied(
            topic['records_scanned'],
            topic['expected_records'],
            limit,
        )
        for topic in state['_tf_topics'].values()
    )


def _record_odometry_frames(state: dict[str, Any], message: Any) -> None:
    """Record one Odometry parent/child pair without trusting empty IDs."""
    if state['records_scanned'] >= state['_limit']:
        return
    state['records_scanned'] += 1
    header = getattr(message, 'header', None)
    parent = str(getattr(header, 'frame_id', '')).strip()
    child = str(getattr(message, 'child_frame_id', '')).strip()
    if not parent or not child or parent == child:
        state['invalid_frame_count'] += 1
        return
    state['_frame_ids'].add(parent)
    state['_child_frame_ids'].add(child)


def _record_tf_transforms(
    state: dict[str, Any],
    topic: str,
    message: Any,
) -> None:
    """Record one TFMessage and whether each undirected edge is dynamic."""
    topic_state = state['_tf_topics'].get(topic)
    if topic_state is None or topic_state['records_scanned'] >= state['_limit']:
        return
    topic_state['records_scanned'] += 1
    topic_state['readable'] = True
    dynamic = _is_dynamic_tf_topic(topic)
    for transform in getattr(message, 'transforms', []):
        header = getattr(transform, 'header', None)
        parent = str(getattr(header, 'frame_id', '')).strip()
        child = str(getattr(transform, 'child_frame_id', '')).strip()
        if not parent or not child or parent == child:
            state['invalid_transform_count'] += 1
            continue
        edge = tuple(sorted((parent, child)))
        state['_edges'][edge] = state['_edges'].get(edge, False) or dynamic


def _is_dynamic_tf_topic(topic: str) -> bool:
    """Treat every TFMessage topic except a tf_static basename as dynamic."""
    return topic.rstrip('/').split('/')[-1] != 'tf_static'


def find_tf_path(
    edges: dict[tuple[str, str], bool],
    source: str,
    target: str,
    *,
    require_dynamic: bool = False,
) -> tuple[list[str], bool] | None:
    """Return a deterministic simple TF path and its dynamic-edge state."""
    graph: dict[str, dict[str, bool]] = {}
    for (left, right), dynamic in edges.items():
        graph.setdefault(left, {})[right] = dynamic
        graph.setdefault(right, {})[left] = dynamic
    queue: list[tuple[str, list[str]]] = [(source, [source])]
    visited = {source}
    while queue:
        node, path = queue.pop(0)
        if node == target:
            used_dynamic = any(
                graph[left][right]
                for left, right in zip(path, path[1:])
            )
            if used_dynamic or not require_dynamic:
                return path, used_dynamic
            return None
        for neighbor in sorted(graph.get(node, {})):
            if neighbor in visited:
                continue
            visited.add(neighbor)
            queue.append((neighbor, [*path, neighbor]))
    return None


def _finalize_odometry_tf_scan(
    state: dict[str, Any],
    *,
    exhausted: bool,
) -> dict[str, Any]:
    """Turn bounded Odometry/TF scan state into the public preflight result."""
    limit = state.pop('_limit')
    expected = state['expected_records']
    odometry_complete = exhausted or (
        expected > 0 and state['records_scanned'] >= expected
    )
    frame_ids = sorted(state.pop('_frame_ids'))
    child_frame_ids = sorted(state.pop('_child_frame_ids'))
    tf_topics = []
    for topic in state.pop('_tf_topics').values():
        topic['complete'] = exhausted or (
            topic['expected_records'] > 0
            and topic['records_scanned'] >= topic['expected_records']
        )
        tf_topics.append(topic)
    tf_topics.sort(key=lambda item: item['topic'])
    readable = state['records_scanned'] > 0
    parent = frame_ids[0] if len(frame_ids) == 1 else None
    child = child_frame_ids[0] if len(child_frame_ids) == 1 else None
    edges = state.pop('_edges')
    tf_path: list[str] = []
    path_edges: list[dict[str, Any]] = []
    dynamic_path: bool | None = None

    if not readable:
        status = 'failed'
        reason = 'No readable Odometry record was found on the selected topic.'
    elif (
        state['invalid_frame_count']
        or len(frame_ids) != 1
        or len(child_frame_ids) != 1
    ):
        status = 'failed'
        reason = (
            'Odometry parent/child frame IDs are empty, identical, or '
            'inconsistent within the bounded bag scan.'
        )
    else:
        dynamic_result = find_tf_path(
            edges,
            parent,
            child,
            require_dynamic=True,
        )
        any_result = dynamic_result or find_tf_path(edges, parent, child)
        if dynamic_result is not None:
            tf_path, dynamic_path = dynamic_result
            all_complete = odometry_complete and all(
                topic['complete'] for topic in tf_topics
            )
            status = 'passed' if all_complete else 'sampled'
            reason = (
                'A TF path with at least one dynamic /tf edge connects the '
                'Odometry parent and child frames.'
            )
        elif any_result is not None:
            tf_path, dynamic_path = any_result
            status = 'failed'
            reason = (
                'The bounded bag scan found only a static TF path between the '
                'Odometry parent and child frames.'
            )
        else:
            status = 'failed'
            reason = (
                'No TF path in the bounded bag scan connects the Odometry '
                'parent and child frames.'
            )

    path_edges = [
        {
            'from_frame': left,
            'to_frame': right,
            'dynamic': edges[tuple(sorted((left, right)))],
        }
        for left, right in zip(tf_path, tf_path[1:])
    ]
    return {
        'status': status,
        'odometry_topic': state['odometry_topic'],
        'expected_records': expected,
        'records_scanned': state['records_scanned'],
        'complete': odometry_complete,
        'readable': readable,
        'header_frame_id': parent,
        'child_frame_id': child,
        'frame_ids': frame_ids,
        'child_frame_ids': child_frame_ids,
        'invalid_frame_count': state['invalid_frame_count'],
        'tf_topics': tf_topics,
        'tf_records_scanned': sum(
            topic['records_scanned'] for topic in tf_topics
        ),
        'invalid_transform_count': state['invalid_transform_count'],
        'tf_path': tf_path,
        'path_edges': path_edges,
        'dynamic_path': dynamic_path,
        'max_records_per_topic': limit,
        'reason': reason,
    }


def _odometry_tf_error_result(
    odometry_topic: TopicRecord,
    tf_topics: list[TopicRecord],
    max_records_per_topic: int,
    status: str,
    reason: str,
) -> dict[str, Any]:
    return {
        'status': status,
        'odometry_topic': odometry_topic.name,
        'expected_records': odometry_topic.message_count,
        'records_scanned': 0,
        'complete': False,
        'readable': False,
        'header_frame_id': None,
        'child_frame_id': None,
        'frame_ids': [],
        'child_frame_ids': [],
        'invalid_frame_count': 0,
        'tf_topics': [
            {
                'topic': topic.name,
                'expected_records': topic.message_count,
                'records_scanned': 0,
                'complete': False,
                'readable': False,
            }
            for topic in sorted(tf_topics, key=lambda item: item.name)
        ],
        'tf_records_scanned': 0,
        'invalid_transform_count': 0,
        'tf_path': [],
        'path_edges': [],
        'dynamic_path': None,
        'max_records_per_topic': max_records_per_topic,
        'reason': reason,
    }


def inspect_odometry_tf(
    bag_path: Path,
    odometry_topic: TopicRecord,
    tf_topics: list[TopicRecord],
    storage_id: str,
    max_records_per_topic: int = MAX_ODOMETRY_TF_RECORDS_PER_TOPIC,
) -> dict[str, Any]:
    """Boundedly verify that recorded Odometry frame IDs have dynamic TF."""
    if max_records_per_topic <= 0:
        raise ValueError('max_records_per_topic must be positive')
    try:
        import rosbag2_py
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        return _inspect_odometry_tf_with_rosbags(
            bag_path,
            odometry_topic,
            tf_topics,
            max_records_per_topic,
            ros_import_error=exc,
        )

    state = _odometry_tf_state(
        odometry_topic,
        tf_topics,
        max_records_per_topic,
    )
    selected_topics = [odometry_topic.name, *state['_tf_topics']]
    try:
        reader = _open_rosbag_reader(rosbag2_py, bag_path, storage_id)
        reader.set_filter(rosbag2_py.StorageFilter(topics=selected_topics))
        exhausted = True
        while reader.has_next():
            if _odometry_tf_scan_satisfied(state):
                exhausted = False
                break
            topic, serialized, _ = reader.read_next()
            if topic == odometry_topic.name:
                message = deserialize_message(serialized, Odometry)
                _record_odometry_frames(state, message)
            elif topic in state['_tf_topics']:
                message = deserialize_message(serialized, TFMessage)
                _record_tf_transforms(state, topic, message)
        return _finalize_odometry_tf_scan(state, exhausted=exhausted)
    except Exception as exc:  # storage plugins expose backend-specific errors
        return _odometry_tf_error_result(
            odometry_topic,
            tf_topics,
            max_records_per_topic,
            'error',
            f'Failed to inspect Odometry/TF connectivity: {exc}',
        )


def _inspect_odometry_tf_with_rosbags(
    bag_path: Path,
    odometry_topic: TopicRecord,
    tf_topics: list[TopicRecord],
    max_records_per_topic: int,
    ros_import_error: ImportError,
) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as rosbags_error:
        return _odometry_tf_error_result(
            odometry_topic,
            tf_topics,
            max_records_per_topic,
            'unavailable',
            'Odometry/TF inspection is unavailable: ROS 2 Python bindings '
            f'failed to import ({ros_import_error}); the rosbags fallback '
            f'also failed to import ({rosbags_error}).',
        )

    state = _odometry_tf_state(
        odometry_topic,
        tf_topics,
        max_records_per_topic,
    )
    try:
        typestore = get_typestore(Stores.LATEST)
        with AnyReader(
            [bag_path],
            default_typestore=typestore,
        ) as reader:
            selected_topics = {odometry_topic.name, *state['_tf_topics']}
            connections = [
                connection
                for connection in reader.connections
                if connection.topic in selected_topics
                and connection.msgtype in {ODOMETRY, TFMESSAGE}
            ]
            exhausted = True
            for connection, _, serialized in reader.messages(
                connections=connections,
            ):
                if _odometry_tf_scan_satisfied(state):
                    exhausted = False
                    break
                message = reader.deserialize(serialized, connection.msgtype)
                if connection.msgtype == ODOMETRY:
                    _record_odometry_frames(state, message)
                else:
                    _record_tf_transforms(state, connection.topic, message)
        return _finalize_odometry_tf_scan(state, exhausted=exhausted)
    except Exception as exc:
        return _odometry_tf_error_result(
            odometry_topic,
            tf_topics,
            max_records_per_topic,
            'error',
            'Failed to inspect Odometry/TF connectivity with rosbags: '
            f'{exc}',
        )


def _no_odometry_tf_result(
    max_records_per_topic: int,
) -> dict[str, Any]:
    return {
        'status': 'not_applicable',
        'odometry_topic': None,
        'expected_records': 0,
        'records_scanned': 0,
        'complete': True,
        'readable': False,
        'header_frame_id': None,
        'child_frame_id': None,
        'frame_ids': [],
        'child_frame_ids': [],
        'invalid_frame_count': 0,
        'tf_topics': [],
        'tf_records_scanned': 0,
        'invalid_transform_count': 0,
        'tf_path': [],
        'path_edges': [],
        'dynamic_path': None,
        'max_records_per_topic': max_records_per_topic,
        'reason': 'No Odometry topic was found.',
    }


def _public_dynamic_path_edges(
    path_edges: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    """Return normalized dynamic edges while preserving TF path direction."""
    return [
        {
            'from_frame': str(edge['from_frame']),
            'to_frame': str(edge['to_frame']),
            'dynamic': True,
        }
        for edge in path_edges
        if edge.get('dynamic') is True
    ]


def _no_odometry_tf_timing_result(
    pointcloud_topic: TopicRecord | None,
    max_records_per_topic: int,
    reason: str,
) -> dict[str, Any]:
    """Build an explicit no-scan result for an inapplicable timing check."""
    return {
        'status': 'not_applicable',
        'sample_basis': 'bag_record_order_and_header_stamp',
        'pointcloud_topic': (
            pointcloud_topic.name if pointcloud_topic is not None else None
        ),
        'expected_pointcloud_records': (
            pointcloud_topic.message_count if pointcloud_topic is not None else 0
        ),
        'pointcloud_records_scanned': 0,
        'complete': True,
        'readable': False,
        'dynamic_path_edges': [],
        'tf_topics': [],
        'tf_records_scanned': 0,
        'invalid_pointcloud_stamp_count': 0,
        'invalid_transform_stamp_count': 0,
        'clouds_before_all_dynamic_tf': 0,
        'future_extrapolation_count': 0,
        'first_future_cloud_stamp_ns': None,
        'max_future_gap_ns': 0,
        'max_future_gap_edge': None,
        'latest_required_tf_stamp_ns_at_max_gap': None,
        'max_records_per_topic': max_records_per_topic,
        'reason': reason,
    }


def _odometry_tf_timing_state(
    pointcloud_topic: TopicRecord,
    tf_topics: list[TopicRecord],
    path_edges: list[dict[str, Any]],
    max_records_per_topic: int,
) -> dict[str, Any]:
    dynamic_edges = _public_dynamic_path_edges(path_edges)
    return {
        'pointcloud_topic': pointcloud_topic.name,
        'expected_pointcloud_records': pointcloud_topic.message_count,
        'pointcloud_records_scanned': 0,
        'readable': False,
        'dynamic_path_edges': dynamic_edges,
        '_latest_dynamic_stamps': {
            tuple(sorted((edge['from_frame'], edge['to_frame']))): None
            for edge in dynamic_edges
        },
        '_edge_by_key': {
            tuple(sorted((edge['from_frame'], edge['to_frame']))): edge
            for edge in dynamic_edges
        },
        '_tf_topics': {
            topic.name: {
                'topic': topic.name,
                'expected_records': topic.message_count,
                'records_scanned': 0,
                'complete': False,
                'readable': False,
            }
            for topic in tf_topics
        },
        'invalid_pointcloud_stamp_count': 0,
        'invalid_transform_stamp_count': 0,
        'clouds_before_all_dynamic_tf': 0,
        'future_extrapolation_count': 0,
        'first_future_cloud_stamp_ns': None,
        'max_future_gap_ns': 0,
        'max_future_gap_edge': None,
        'latest_required_tf_stamp_ns_at_max_gap': None,
        '_limit': max_records_per_topic,
    }


def _odometry_tf_timing_scan_satisfied(state: dict[str, Any]) -> bool:
    limit = state['_limit']
    if not _bounded_topic_satisfied(
        state['pointcloud_records_scanned'],
        state['expected_pointcloud_records'],
        limit,
    ):
        return False
    return all(
        _bounded_topic_satisfied(
            topic['records_scanned'],
            topic['expected_records'],
            limit,
        )
        for topic in state['_tf_topics'].values()
    )


def _record_odometry_tf_timing_transform(
    state: dict[str, Any],
    topic: str,
    message: Any,
) -> None:
    """Advance latest stamps for required dynamic TF edges in replay order."""
    topic_state = state['_tf_topics'].get(topic)
    if topic_state is None or topic_state['records_scanned'] >= state['_limit']:
        return
    topic_state['records_scanned'] += 1
    topic_state['readable'] = True
    if not _is_dynamic_tf_topic(topic):
        return
    latest = state['_latest_dynamic_stamps']
    for transform in getattr(message, 'transforms', []):
        header = getattr(transform, 'header', None)
        parent = str(getattr(header, 'frame_id', '')).strip()
        child = str(getattr(transform, 'child_frame_id', '')).strip()
        edge_key = tuple(sorted((parent, child)))
        if not parent or not child or parent == child or edge_key not in latest:
            continue
        stamp_ns = _stamp_ns_from_message(transform)
        if stamp_ns is None:
            state['invalid_transform_stamp_count'] += 1
            continue
        previous = latest[edge_key]
        if previous is None or stamp_ns > previous:
            latest[edge_key] = stamp_ns


def _record_odometry_tf_timing_pointcloud(
    state: dict[str, Any],
    message: Any,
) -> None:
    """Compare one cloud stamp with TF stamps available earlier in the bag."""
    if state['pointcloud_records_scanned'] >= state['_limit']:
        return
    state['pointcloud_records_scanned'] += 1
    state['readable'] = True
    cloud_stamp_ns = _stamp_ns_from_message(message)
    if cloud_stamp_ns is None:
        state['invalid_pointcloud_stamp_count'] += 1
        return

    latest = state['_latest_dynamic_stamps']
    if any(stamp_ns is None for stamp_ns in latest.values()):
        state['clouds_before_all_dynamic_tf'] += 1
        return

    limiting_key, limiting_stamp_ns = min(
        latest.items(),
        key=lambda item: (item[1], item[0]),
    )
    if cloud_stamp_ns <= limiting_stamp_ns:
        return

    gap_ns = cloud_stamp_ns - limiting_stamp_ns
    state['future_extrapolation_count'] += 1
    if state['first_future_cloud_stamp_ns'] is None:
        state['first_future_cloud_stamp_ns'] = cloud_stamp_ns
    if gap_ns > state['max_future_gap_ns']:
        state['max_future_gap_ns'] = gap_ns
        state['max_future_gap_edge'] = dict(state['_edge_by_key'][limiting_key])
        state['latest_required_tf_stamp_ns_at_max_gap'] = limiting_stamp_ns


def _finalize_odometry_tf_timing_scan(
    state: dict[str, Any],
    *,
    exhausted: bool,
) -> dict[str, Any]:
    """Turn replay-order timing state into bounded public evidence."""
    limit = state.pop('_limit')
    expected = state['expected_pointcloud_records']
    pointcloud_complete = exhausted or (
        expected > 0 and state['pointcloud_records_scanned'] >= expected
    )
    tf_topics = []
    for topic in state.pop('_tf_topics').values():
        topic['complete'] = exhausted or (
            topic['expected_records'] > 0
            and topic['records_scanned'] >= topic['expected_records']
        )
        tf_topics.append(topic)
    tf_topics.sort(key=lambda item: item['topic'])
    complete = pointcloud_complete and all(
        topic['complete'] for topic in tf_topics
    )

    if not state['readable']:
        status = 'failed'
        reason = 'No readable PointCloud2 record was found on the selected topic.'
    elif (
        state['invalid_pointcloud_stamp_count']
        or state['invalid_transform_stamp_count']
        or state['clouds_before_all_dynamic_tf']
        or state['future_extrapolation_count']
    ):
        status = 'failed'
        reason = (
            'The bounded replay-order scan found missing, invalid, or future '
            'dynamic TF availability for at least one PointCloud2 record.'
        )
    elif complete:
        status = 'passed'
        reason = (
            'Every scanned PointCloud2 header stamp had all required dynamic '
            'TF edges available at the same or a later header stamp.'
        )
    else:
        status = 'sampled'
        reason = (
            'No dynamic TF timing gap was found in the bounded replay-order '
            'sample; at least one selected topic was not scanned completely.'
        )

    state.pop('_latest_dynamic_stamps')
    state.pop('_edge_by_key')
    return {
        'status': status,
        'sample_basis': 'bag_record_order_and_header_stamp',
        'pointcloud_topic': state['pointcloud_topic'],
        'expected_pointcloud_records': expected,
        'pointcloud_records_scanned': state['pointcloud_records_scanned'],
        'complete': complete,
        'readable': state['readable'],
        'dynamic_path_edges': state['dynamic_path_edges'],
        'tf_topics': tf_topics,
        'tf_records_scanned': sum(
            topic['records_scanned'] for topic in tf_topics
        ),
        'invalid_pointcloud_stamp_count': (
            state['invalid_pointcloud_stamp_count']
        ),
        'invalid_transform_stamp_count': (
            state['invalid_transform_stamp_count']
        ),
        'clouds_before_all_dynamic_tf': state['clouds_before_all_dynamic_tf'],
        'future_extrapolation_count': state['future_extrapolation_count'],
        'first_future_cloud_stamp_ns': state['first_future_cloud_stamp_ns'],
        'max_future_gap_ns': state['max_future_gap_ns'],
        'max_future_gap_edge': state['max_future_gap_edge'],
        'latest_required_tf_stamp_ns_at_max_gap': (
            state['latest_required_tf_stamp_ns_at_max_gap']
        ),
        'max_records_per_topic': limit,
        'reason': reason,
    }


def _odometry_tf_timing_error_result(
    pointcloud_topic: TopicRecord,
    tf_topics: list[TopicRecord],
    path_edges: list[dict[str, Any]],
    max_records_per_topic: int,
    status: str,
    reason: str,
) -> dict[str, Any]:
    result = _no_odometry_tf_timing_result(
        pointcloud_topic,
        max_records_per_topic,
        reason,
    )
    result.update({
        'status': status,
        'complete': False,
        'dynamic_path_edges': _public_dynamic_path_edges(path_edges),
        'tf_topics': [
            {
                'topic': topic.name,
                'expected_records': topic.message_count,
                'records_scanned': 0,
                'complete': False,
                'readable': False,
            }
            for topic in sorted(tf_topics, key=lambda item: item.name)
        ],
    })
    return result


def inspect_odometry_tf_timing(
    bag_path: Path,
    pointcloud_topic: TopicRecord,
    tf_topics: list[TopicRecord],
    path_edges: list[dict[str, Any]],
    storage_id: str,
    max_records_per_topic: int = MAX_ODOMETRY_TF_TIMING_RECORDS_PER_TOPIC,
) -> dict[str, Any]:
    """Inspect exact-cloud-stamp TF availability in bounded bag replay order."""
    if max_records_per_topic <= 0:
        raise ValueError('max_records_per_topic must be positive')
    dynamic_edges = _public_dynamic_path_edges(path_edges)
    if not dynamic_edges:
        return _no_odometry_tf_timing_result(
            pointcloud_topic,
            max_records_per_topic,
            'The Odometry TF path has no dynamic edge to inspect.',
        )
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import PointCloud2
        from tf2_msgs.msg import TFMessage
    except ImportError as exc:
        return _inspect_odometry_tf_timing_with_rosbags(
            bag_path,
            pointcloud_topic,
            tf_topics,
            path_edges,
            max_records_per_topic,
            ros_import_error=exc,
        )

    state = _odometry_tf_timing_state(
        pointcloud_topic,
        tf_topics,
        path_edges,
        max_records_per_topic,
    )
    selected_topics = [pointcloud_topic.name, *state['_tf_topics']]
    try:
        reader = _open_rosbag_reader(rosbag2_py, bag_path, storage_id)
        reader.set_filter(rosbag2_py.StorageFilter(topics=selected_topics))
        exhausted = True
        while reader.has_next():
            if _odometry_tf_timing_scan_satisfied(state):
                exhausted = False
                break
            topic, serialized, _ = reader.read_next()
            if topic == pointcloud_topic.name:
                message = deserialize_message(serialized, PointCloud2)
                _record_odometry_tf_timing_pointcloud(state, message)
            elif topic in state['_tf_topics']:
                message = deserialize_message(serialized, TFMessage)
                _record_odometry_tf_timing_transform(state, topic, message)
        return _finalize_odometry_tf_timing_scan(state, exhausted=exhausted)
    except Exception as exc:  # storage plugins expose backend-specific errors
        return _odometry_tf_timing_error_result(
            pointcloud_topic,
            tf_topics,
            path_edges,
            max_records_per_topic,
            'error',
            f'Failed to inspect Odometry TF timing: {exc}',
        )


def _inspect_odometry_tf_timing_with_rosbags(
    bag_path: Path,
    pointcloud_topic: TopicRecord,
    tf_topics: list[TopicRecord],
    path_edges: list[dict[str, Any]],
    max_records_per_topic: int,
    ros_import_error: ImportError,
) -> dict[str, Any]:
    try:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore
    except ImportError as rosbags_error:
        return _odometry_tf_timing_error_result(
            pointcloud_topic,
            tf_topics,
            path_edges,
            max_records_per_topic,
            'unavailable',
            'Odometry TF timing inspection is unavailable: ROS 2 Python '
            f'bindings failed to import ({ros_import_error}); the rosbags '
            f'fallback also failed to import ({rosbags_error}).',
        )

    state = _odometry_tf_timing_state(
        pointcloud_topic,
        tf_topics,
        path_edges,
        max_records_per_topic,
    )
    try:
        typestore = get_typestore(Stores.LATEST)
        with AnyReader(
            [bag_path],
            default_typestore=typestore,
        ) as reader:
            selected_topics = {pointcloud_topic.name, *state['_tf_topics']}
            connections = [
                connection
                for connection in reader.connections
                if connection.topic in selected_topics
                and (
                    (
                        connection.topic == pointcloud_topic.name
                        and connection.msgtype == POINTCLOUD2
                    )
                    or (
                        connection.topic in state['_tf_topics']
                        and connection.msgtype == TFMESSAGE
                    )
                )
            ]
            exhausted = True
            for connection, _, serialized in reader.messages(
                connections=connections,
            ):
                if _odometry_tf_timing_scan_satisfied(state):
                    exhausted = False
                    break
                message = reader.deserialize(serialized, connection.msgtype)
                if connection.topic == pointcloud_topic.name:
                    _record_odometry_tf_timing_pointcloud(state, message)
                else:
                    _record_odometry_tf_timing_transform(
                        state,
                        connection.topic,
                        message,
                    )
        return _finalize_odometry_tf_timing_scan(state, exhausted=exhausted)
    except Exception as exc:
        return _odometry_tf_timing_error_result(
            pointcloud_topic,
            tf_topics,
            path_edges,
            max_records_per_topic,
            'error',
            'Failed to inspect Odometry TF timing with rosbags: '
            f'{exc}',
        )


def summarize_bag(bag_path: Path) -> dict[str, Any]:
    """Summarize a rosbag2 in terms of map-authoring inputs."""
    bag_info = load_bag_metadata(bag_path)
    topic_records = _collect_topics(bag_info)
    summary = {
        'bag_path': str(bag_path),
        'duration_sec': _duration_seconds(bag_info),
        'message_count': int(bag_info.get('message_count', 0)),
        'topics': {
            'pointcloud2': [asdict(item) for item in _topic_group(topic_records, POINTCLOUD2)],
            'imu': [asdict(item) for item in _topic_group(topic_records, IMU)],
            'navsatfix': [asdict(item) for item in _topic_group(topic_records, NAVSATFIX)],
            'odometry': [asdict(item) for item in _topic_group(topic_records, ODOMETRY)],
            'velodyne_scan': [asdict(item) for item in _topic_group(topic_records, VELODYNE_SCAN)],
            'applanix_gsof49': [asdict(item) for item in _topic_group(topic_records, GSOF49)],
            'applanix_gsof50': [asdict(item) for item in _topic_group(topic_records, GSOF50)],
            'tf': [asdict(item) for item in _topic_group(topic_records, TFMESSAGE)],
            'velocity_report': [
                asdict(item)
                for item in _topic_group(topic_records, VELOCITY_REPORT)
            ],
        },
    }

    summary['capabilities'] = {
        'has_pointcloud2': bool(summary['topics']['pointcloud2']),
        'has_imu': bool(summary['topics']['imu']),
        'has_navsatfix': bool(summary['topics']['navsatfix']),
        'has_odometry': bool(summary['topics']['odometry']),
        'has_velodyne_scan': bool(summary['topics']['velodyne_scan']),
        'has_applanix_gsof49': bool(summary['topics']['applanix_gsof49']),
        'has_applanix_gsof50': bool(summary['topics']['applanix_gsof50']),
        'has_tf': bool(summary['topics']['tf']),
        'has_velocity_report': bool(summary['topics']['velocity_report']),
    }
    return summary


def build_recommendations(summary: dict[str, Any]) -> list[dict[str, Any]]:
    """Build compatible workflow recommendations for the bag."""
    bag_path = summary['bag_path']
    bag_q = _safe_quote(bag_path)
    capabilities = summary['capabilities']
    recommendations: list[dict[str, Any]] = []

    best_pointcloud = (
        summary['topics']['pointcloud2'][0]
        if summary['topics']['pointcloud2'] else None
    )
    best_imu = summary['topics']['imu'][0] if summary['topics']['imu'] else None
    best_navsat = summary['topics']['navsatfix'][0] if summary['topics']['navsatfix'] else None
    best_packet = (
        summary['topics']['velodyne_scan'][0]
        if summary['topics']['velodyne_scan'] else None
    )
    best_gsof49 = (
        summary['topics']['applanix_gsof49'][0]
        if summary['topics']['applanix_gsof49'] else None
    )
    best_gsof50 = (
        summary['topics']['applanix_gsof50'][0]
        if summary['topics']['applanix_gsof50'] else None
    )
    bag_path_lower = bag_path.lower()

    def looks_like_livox_mid360() -> bool:
        topic_names = []
        for key in ('pointcloud2', 'imu'):
            topic_names.extend(item['name'].lower() for item in summary['topics'][key])
        return 'mid360' in bag_path_lower or any('livox' in name for name in topic_names)

    pointcloud_inspection = summary['pointcloud_inspection']
    timestamp_order_ready = (
        summary['timestamp_order']['status'] in {'passed', 'sampled'}
    )
    failed_timestamp_topics = set(
        summary['timestamp_order']['failed_topics']
    )
    if (
        capabilities['has_pointcloud2']
        and capabilities['has_imu']
        and pointcloud_inspection['rko_lio_compatible'] is True
        and timestamp_order_ready
        and best_pointcloud['name'] not in failed_timestamp_topics
        and best_imu['name'] not in failed_timestamp_topics
    ):
        command = textwrap.dedent(
            f"""\
            ros2 launch lidarslam rko_lio_slam.launch.py \\
              bag_path:={bag_q} \\
              lidar_topic:={_safe_quote(best_pointcloud['name'])} \\
              imu_topic:={_safe_quote(best_imu['name'])}"""
        )
        notes = []
        if capabilities['has_navsatfix']:
            notes.append(
                'GNSS is present in the bag. Inspect covariance before enabling backend '
                'GNSS weighting.'
            )
        recommendations.append({
            'id': 'rko_lio_graph_public_path',
            'priority': 100,
            'label': 'RKO-LIO + graph_based_slam public path',
            'why': [
                f"PointCloud2 is available on {best_pointcloud['name']}",
                f"Imu is available on {best_imu['name']}",
                (
                    'The first PointCloud2 record has RKO-LIO-compatible '
                    f"per-point timestamps in {pointcloud_inspection['timestamp_field']!r}."
                ),
                'This is the main maintained map-authoring path in the repository.',
            ],
            'command': command,
            'notes': notes,
        })

        if looks_like_livox_mid360():
            tuned_command = textwrap.dedent(
                f"""\
                ros2 launch lidarslam rko_lio_slam.launch.py \\
                  main_param_dir:=lidarslam/param/lidarslam_mid360_rko_graph.yaml \\
                  rko_param_file:=lidarslam/param/rko_lio_mid360.yaml \\
                  bag_path:={bag_q} \\
                  lidar_topic:={_safe_quote(best_pointcloud['name'])} \\
                  imu_topic:={_safe_quote(best_imu['name'])}"""
            )
            recommendations.append({
                'id': 'rko_lio_graph_mid360_preset',
                'priority': 95,
                'label': 'RKO-LIO + graph_based_slam MID360/Livox preset',
                'why': [
                    (
                        f"PointCloud2 topic {best_pointcloud['name']} looks like "
                        'a Livox/MID360 source'
                    ),
                    (
                        'The repository tracks a tuned MID360 graph/backend YAML '
                        'for this sensor family.'
                    ),
                ],
                'command': tuned_command,
                'notes': [
                    (
                        'Use this when the bag is a Livox/MID360-style dataset and '
                        'you want the tracked tuned preset instead of the generic '
                        'default.'
                    ),
                ],
            })

    if (
        capabilities['has_pointcloud2']
        and capabilities['has_navsatfix']
        and timestamp_order_ready
        and best_pointcloud['name'] not in failed_timestamp_topics
    ):
        command = (
            'bash scripts/run_open_data_gnss_smoke.sh '
            f'--bag {bag_q} '
            f'--points-topic {_safe_quote(best_pointcloud["name"])} '
            f'--gnss-topic {_safe_quote(best_navsat["name"])}'
        )
        if capabilities['has_imu']:
            command += f' --imu-topic {_safe_quote(best_imu["name"])}'
        recommendations.append({
            'id': 'pointcloud_gnss_smoke',
            'priority': 80,
            'label': 'PointCloud2 + GNSS smoke path',
            'why': [
                f"PointCloud2 is available on {best_pointcloud['name']}",
                f"NavSatFix is available on {best_navsat['name']}",
                (
                    'This wrapper produces a verified pointcloud map with '
                    'GNSS-enabled backend constraints.'
                ),
            ],
            'command': command,
            'notes': [],
        })

    if capabilities['has_velodyne_scan'] and capabilities['has_applanix_gsof49']:
        command = (
            'bash scripts/run_open_data_applanix_velodyne_gnss_smoke.sh '
            f'--bag {bag_q} '
            f'--packet-topic {_safe_quote(best_packet["name"])}'
        )
        if best_gsof49 is not None:
            command += f' --gsof49-topic {_safe_quote(best_gsof49["name"])}'
        if best_gsof50 is not None:
            command += f' --gsof50-topic {_safe_quote(best_gsof50["name"])}'
        recommendations.append({
            'id': 'packet_applanix_smoke',
            'priority': 70,
            'label': 'Velodyne packet + Applanix smoke path',
            'why': [
                f"VelodyneScan is available on {best_packet['name']}",
                f"Applanix GSOF49 is available on {best_gsof49['name']}",
                (
                    'This wrapper converts packet and Applanix data into the '
                    'maintained pointcloud-map path.'
                ),
            ],
            'command': command,
            'notes': [],
        })

    return sorted(recommendations, key=lambda item: item['priority'], reverse=True)


def build_preflight_payload(
    bag_path: Path,
    pointcloud_inspector: Callable[[Path, str, str], dict[str, Any]] | None = None,
    timestamp_inspector: Callable[
        [Path, list[TopicRecord], str, int],
        dict[str, Any],
    ] | None = None,
    odometry_tf_inspector: Callable[
        [Path, TopicRecord, list[TopicRecord], str, int],
        dict[str, Any],
    ] | None = None,
    odometry_tf_timing_inspector: Callable[
        [
            Path,
            TopicRecord,
            list[TopicRecord],
            list[dict[str, Any]],
            str,
            int,
        ],
        dict[str, Any],
    ] | None = None,
    max_timestamp_records_per_topic: int = MAX_TIMESTAMP_RECORDS_PER_TOPIC,
    max_odometry_tf_records_per_topic: int = MAX_ODOMETRY_TF_RECORDS_PER_TOPIC,
    max_odometry_tf_timing_records_per_topic: int = (
        MAX_ODOMETRY_TF_TIMING_RECORDS_PER_TOPIC
    ),
) -> dict[str, Any]:
    """Create the machine-readable preflight result."""
    summary = summarize_bag(bag_path)
    bag_info = load_bag_metadata(bag_path)
    topic_records = _collect_topics(bag_info)
    storage_id = str(bag_info.get('storage_identifier', ''))
    pointcloud_topic = _best_topic(topic_records, POINTCLOUD2)
    if pointcloud_topic is not None:
        inspector = pointcloud_inspector or inspect_pointcloud_record
        summary['pointcloud_inspection'] = inspector(
            bag_path,
            pointcloud_topic.name,
            storage_id,
        )
    else:
        summary['pointcloud_inspection'] = {
            'status': 'not_applicable',
            'topic': None,
            'fields': [],
            'rko_lio_compatible': None,
            'timestamp_field': None,
            'reason': 'No PointCloud2 topic was found.',
        }
    timestamp_topics = []
    for msg_type in TIMESTAMP_SCAN_MSGTYPES:
        topic = _best_topic(topic_records, msg_type)
        if topic is not None:
            timestamp_topics.append(topic)
    order_inspector = timestamp_inspector or inspect_timestamp_order
    summary['timestamp_order'] = order_inspector(
        bag_path,
        timestamp_topics,
        storage_id,
        max_timestamp_records_per_topic,
    )
    odometry_topic = _best_topic(topic_records, ODOMETRY)
    tf_topics = _topic_group(topic_records, TFMESSAGE)
    if odometry_topic is None:
        summary['odometry_tf'] = _no_odometry_tf_result(
            max_odometry_tf_records_per_topic,
        )
    else:
        odometry_inspector = odometry_tf_inspector or inspect_odometry_tf
        summary['odometry_tf'] = odometry_inspector(
            bag_path,
            odometry_topic,
            tf_topics,
            storage_id,
            max_odometry_tf_records_per_topic,
        )
    odometry_tf = summary['odometry_tf']
    if pointcloud_topic is None:
        summary['odometry_tf_timing'] = _no_odometry_tf_timing_result(
            None,
            max_odometry_tf_timing_records_per_topic,
            'No PointCloud2 topic was found.',
        )
    elif odometry_tf['status'] not in {'passed', 'sampled'}:
        summary['odometry_tf_timing'] = _no_odometry_tf_timing_result(
            pointcloud_topic,
            max_odometry_tf_timing_records_per_topic,
            'A usable dynamic Odometry TF path was not established.',
        )
    elif not _public_dynamic_path_edges(odometry_tf['path_edges']):
        summary['odometry_tf_timing'] = _no_odometry_tf_timing_result(
            pointcloud_topic,
            max_odometry_tf_timing_records_per_topic,
            'The usable Odometry TF path has no dynamic edge to inspect.',
        )
    else:
        timing_inspector = (
            odometry_tf_timing_inspector or inspect_odometry_tf_timing
        )
        summary['odometry_tf_timing'] = timing_inspector(
            bag_path,
            pointcloud_topic,
            tf_topics,
            odometry_tf['path_edges'],
            storage_id,
            max_odometry_tf_timing_records_per_topic,
        )
    recommendations = build_recommendations(summary)
    bag_q = _safe_quote(summary['bag_path'])
    advisory = []
    if summary['capabilities']['has_navsatfix']:
        navsat_topic = summary['topics']['navsatfix'][0]['name']
        advisory.append({
            'label': 'Inspect NavSatFix covariance',
            'command': (
                'python3 scripts/inspect_navsatfix_covariance.py '
                f'{_safe_quote(summary["bag_path"])} --topic {_safe_quote(navsat_topic)}'
            ),
        })
    if summary['capabilities']['has_applanix_gsof50']:
        gsof50_topic = summary['topics']['applanix_gsof50'][0]['name']
        advisory.append({
            'label': 'Inspect Applanix GSOF50 quality',
            'command': (
                'python3 scripts/inspect_applanix_gsof50_quality.py '
                f'{_safe_quote(summary["bag_path"])} --topic {_safe_quote(gsof50_topic)}'
            ),
        })

    findings = []
    if (
        not summary['capabilities']['has_pointcloud2']
        and not summary['capabilities']['has_velodyne_scan']
    ):
        findings.append(_finding(
            'range-input-missing',
            'No PointCloud2 or VelodyneScan topic was found.',
            'Record or convert a sensor_msgs/msg/PointCloud2 or '
            'velodyne_msgs/msg/VelodyneScan topic, then rerun '
            'lidarslam-map doctor <rosbag2_dir>.',
        ))
    inspection = summary['pointcloud_inspection']
    if (
        summary['capabilities']['has_pointcloud2']
        and summary['capabilities']['has_imu']
        and inspection['rko_lio_compatible'] is not True
    ):
        if inspection['status'] in {'error', 'unavailable'}:
            findings.append(_finding(
                'pointcloud-inspection-unavailable',
                f"PointCloud2 topic {inspection['topic']} could not be inspected: "
                f"{inspection['reason']}",
                'Run ros2 bag info <rosbag2_dir>; repair invalid metadata or '
                'install the reported storage/message support, then rerun '
                'lidarslam-map doctor <rosbag2_dir>.',
            ))
        elif inspection['status'] == 'empty':
            findings.append(_finding(
                'pointcloud-record-missing',
                f"PointCloud2 topic {inspection['topic']} had no readable record: "
                f"{inspection['reason']}",
                'Record at least one readable PointCloud2 message, then rerun '
                'lidarslam-map doctor <rosbag2_dir>.',
            ))
        else:
            findings.append(_finding(
                'pointcloud-layout-incompatible',
                f"PointCloud2 topic {inspection['topic']} is not verified as compatible "
                f"with RKO-LIO: {inspection['reason']}",
                'Rewrite the selected PointCloud2 with x/y/z fields and a supported '
                'per-point timestamp field (t, timestamp, time, or stamps), then '
                'rerun lidarslam-map doctor <rosbag2_dir>.',
            ))
    timestamp_order = summary['timestamp_order']
    if timestamp_order['status'] in {'error', 'unavailable'}:
        findings.append(_finding(
            'timestamp-inspection-unavailable',
            'Header timestamp order could not be inspected before launch: '
            f"{timestamp_order['reason']}",
            'Run ros2 bag info <rosbag2_dir>; repair invalid metadata or install '
            'the reported storage/message support, then rerun '
            'lidarslam-map doctor <rosbag2_dir>.',
        ))
    if timestamp_order['status'] == 'failed':
        for topic in timestamp_order['topics']:
            if not topic['readable']:
                findings.append(_finding(
                    'timestamp-unreadable',
                    f"Header timestamps on {topic['topic']} could not be read. "
                    'Correct or rewrite the bag before mapping.',
                    f"Repair or rewrite header.stamp on {topic['topic']}, then "
                    'rerun lidarslam-map doctor <rosbag2_dir>.',
                ))
            elif topic['reversal_count'] or topic['invalid_stamp_count']:
                findings.append(_finding(
                    'timestamp-order-invalid',
                    f"Header timestamp disorder on {topic['topic']}: "
                    f"{topic['reversal_count']} reversal(s), "
                    f"{topic['invalid_stamp_count']} invalid stamp(s), "
                    'maximum backward jump '
                    f"{topic['max_backward_jump_ns'] / 1e9:.9f} s. "
                    'Correct or rewrite the bag before mapping.',
                    f"Rewrite header.stamp on {topic['topic']} so it is valid and "
                    'monotonic, then rerun lidarslam-map doctor <rosbag2_dir>.',
                ))
    odometry_tf = summary['odometry_tf']
    if odometry_tf['status'] in {'error', 'unavailable'}:
        findings.append(_finding(
            'odometry-tf-inspection-unavailable',
            'Odometry-to-TF connectivity could not be inspected: '
            f"{odometry_tf['reason']}",
            'Run ros2 bag info <rosbag2_dir>; repair invalid metadata or install '
            'the reported storage/message support, then rerun '
            'lidarslam-map doctor <rosbag2_dir>.',
        ))
    elif odometry_tf['status'] == 'failed':
        if (
            not odometry_tf['readable']
            or odometry_tf['invalid_frame_count']
            or len(odometry_tf['frame_ids']) != 1
            or len(odometry_tf['child_frame_ids']) != 1
        ):
            findings.append(_finding(
                'odometry-frame-invalid',
                f"Odometry topic {odometry_tf['odometry_topic']} has invalid or "
                f"inconsistent parent/child frame IDs: {odometry_tf['reason']}",
                'Correct header.frame_id and child_frame_id on the Odometry '
                'publisher or bag, then rerun lidarslam-map doctor '
                '<rosbag2_dir>.',
            ))
        elif odometry_tf['tf_path'] and odometry_tf['dynamic_path'] is False:
            findings.append(_finding(
                'odometry-tf-static-only',
                f"Odometry frames {odometry_tf['header_frame_id']} -> "
                f"{odometry_tf['child_frame_id']} have only a static path in "
                f"the bounded bag scan: {' -> '.join(odometry_tf['tf_path'])}.",
                'Configure or record a dynamic /tf broadcaster for the Odometry '
                'motion, then rerun lidarslam-map doctor <rosbag2_dir>.',
            ))
        else:
            findings.append(_finding(
                'odometry-tf-missing',
                f"Odometry frames {odometry_tf['header_frame_id']} -> "
                f"{odometry_tf['child_frame_id']} have no connecting TF path "
                'in the bounded bag scan. An Odometry message does not publish '
                'that transform by itself.',
                'Configure or record the matching dynamic /tf broadcaster, then '
                'rerun lidarslam-map doctor <rosbag2_dir>.',
            ))
    odometry_tf_timing = summary['odometry_tf_timing']
    if odometry_tf_timing['status'] in {'error', 'unavailable'}:
        findings.append(_finding(
            'odometry-tf-timing-inspection-unavailable',
            'Replay-order Odometry TF timing could not be inspected: '
            f"{odometry_tf_timing['reason']}",
            'Run ros2 bag info <rosbag2_dir>; repair invalid metadata or '
            'install the reported storage/message support, then rerun '
            'lidarslam-map doctor <rosbag2_dir>.',
        ))
    elif odometry_tf_timing['status'] == 'failed':
        if (
            not odometry_tf_timing['readable']
            or odometry_tf_timing['invalid_pointcloud_stamp_count']
            or odometry_tf_timing['invalid_transform_stamp_count']
        ):
            findings.append(_finding(
                'odometry-tf-timing-invalid',
                'Odometry TF timing evidence contains unreadable records or '
                'invalid header stamps: '
                f"{odometry_tf_timing['reason']}",
                'Repair PointCloud2 and required dynamic TF header stamps, then '
                'rerun lidarslam-map doctor <rosbag2_dir>.',
            ))
        if odometry_tf_timing['clouds_before_all_dynamic_tf']:
            findings.append(_finding(
                'odometry-tf-startup-gap',
                f"{odometry_tf_timing['clouds_before_all_dynamic_tf']} "
                'PointCloud2 record(s) arrived before every required dynamic '
                'Odometry TF edge had been observed in bag replay order.',
                'Start and record the dynamic TF broadcaster before sensor '
                'playback, then rerun lidarslam-map doctor <rosbag2_dir>.',
            ))
        if odometry_tf_timing['future_extrapolation_count']:
            edge = odometry_tf_timing['max_future_gap_edge']
            edge_text = (
                f"{edge['from_frame']} -> {edge['to_frame']}"
                if edge is not None else 'unknown dynamic edge'
            )
            findings.append(_finding(
                'odometry-tf-future-gap',
                f"{odometry_tf_timing['future_extrapolation_count']} "
                'PointCloud2 record(s) requested a future dynamic TF in bag '
                'replay order; maximum gap '
                f"{odometry_tf_timing['max_future_gap_ns'] / 1e6:.3f} ms "
                f'on {edge_text}.',
                'Align PointCloud2 and TF clocks/header stamps, raise the actual '
                'dynamic TF publication rate if needed, then rerun '
                'lidarslam-map doctor <rosbag2_dir> and tf2_monitor. Do not '
                'silence the warning or substitute a stale transform.',
            ))
    if not recommendations:
        if not summary['capabilities']['has_imu']:
            findings.append(_finding(
                'imu-input-missing',
                'No Imu topic was found for the main RKO-LIO public path.',
                'Record a sensor_msgs/msg/Imu topic synchronized with the '
                'PointCloud2 input, then rerun lidarslam-map doctor '
                '<rosbag2_dir>.',
            ))
        if not summary['capabilities']['has_navsatfix']:
            findings.append(_finding(
                'navsatfix-input-missing',
                'No NavSatFix topic was found for the PointCloud2 + GNSS smoke path.',
                'If using the GNSS smoke path, record a sensor_msgs/msg/NavSatFix '
                'topic, then rerun lidarslam-map doctor <rosbag2_dir>.',
            ))
        if not summary['capabilities']['has_applanix_gsof49']:
            findings.append(_finding(
                'applanix-gsof49-input-missing',
                'No Applanix GSOF49 topic was found for the packet + Applanix path.',
                'If using the packet path, record an Applanix GSOF49 navigation '
                'topic, then rerun lidarslam-map doctor <rosbag2_dir>.',
            ))

    missing = [finding['message'] for finding in findings]

    beginner_commands = []
    if recommendations:
        beginner_commands = [
            {
                'label': 'Beginner one-command path',
                'command': f'bash scripts/run_autoware_map_beginner.sh {bag_q}',
            },
            {
                'label': 'Beginner path with Foxglove viewer',
                'command': f'bash scripts/run_autoware_map_beginner.sh {bag_q} --foxglove',
            },
            {
                'label': 'Beginner dry-run to inspect the chosen public path',
                'command': f'bash scripts/run_autoware_map_beginner.sh {bag_q} --dry-run',
            },
        ]

    return {
        'schema_version': SCHEMA_VERSION,
        'schema_uri': SCHEMA_URI,
        'summary': summary,
        'recommendations': recommendations,
        'recommended_profile_id': recommendations[0]['id'] if recommendations else None,
        'beginner_commands': beginner_commands,
        'advisory': advisory,
        'findings': findings,
        'missing_requirements': missing,
    }


def _public_topic_type_counts(summary: dict[str, Any]) -> list[dict[str, Any]]:
    """Aggregate topic evidence without publishing topic names."""
    totals: dict[str, dict[str, Any]] = {}
    for records in summary['topics'].values():
        for record in records:
            msg_type = str(record['msg_type'])
            current = totals.setdefault(msg_type, {
                'msg_type': msg_type,
                'topic_count': 0,
                'message_count': 0,
            })
            current['topic_count'] += 1
            current['message_count'] += int(record['message_count'])
    return [totals[msg_type] for msg_type in sorted(totals)]


def build_public_preflight_evidence(
    payload: dict[str, Any],
) -> dict[str, Any]:
    """Project preflight-v6 into bounded evidence safe for a public issue."""
    summary = payload['summary']
    pointcloud = summary['pointcloud_inspection']
    timestamp_order = summary['timestamp_order']
    timestamp_topics = timestamp_order['topics']
    odometry_tf = summary['odometry_tf']
    odometry_tf_timing = summary['odometry_tf_timing']
    recommendations = payload['recommendations']
    findings = payload['findings']
    if findings:
        status = 'action_required'
    elif recommendations:
        status = 'ready'
    else:
        status = 'unsupported'

    return {
        'schema_version': PUBLIC_EVIDENCE_SCHEMA_VERSION,
        'schema_uri': PUBLIC_EVIDENCE_SCHEMA_URI,
        'mode': 'bag_public_evidence',
        'status': status,
        'input': {
            'duration_sec': summary['duration_sec'],
            'message_count': summary['message_count'],
            'capabilities': {
                key: bool(value)
                for key, value in sorted(summary['capabilities'].items())
            },
            'topic_type_counts': _public_topic_type_counts(summary),
        },
        'checks': {
            'pointcloud': {
                'status': pointcloud['status'],
                'field_names': sorted({
                    str(field['name']) for field in pointcloud['fields']
                }),
                'rko_lio_compatible': pointcloud['rko_lio_compatible'],
                'timestamp_field': pointcloud['timestamp_field'],
            },
            'timestamp_order': {
                'status': timestamp_order['status'],
                'topic_type_count': len({
                    str(item['msg_type']) for item in timestamp_topics
                }),
                'records_scanned': sum(
                    int(item['records_scanned']) for item in timestamp_topics
                ),
                'reversal_count': sum(
                    int(item['reversal_count']) for item in timestamp_topics
                ),
                'invalid_stamp_count': sum(
                    int(item['invalid_stamp_count'])
                    for item in timestamp_topics
                ),
            },
            'odometry_tf': {
                'status': odometry_tf['status'],
                'records_scanned': odometry_tf['records_scanned'],
                'tf_records_scanned': odometry_tf['tf_records_scanned'],
                'readable': odometry_tf['readable'],
                'invalid_frame_count': odometry_tf['invalid_frame_count'],
                'invalid_transform_count': (
                    odometry_tf['invalid_transform_count']
                ),
                'path_edge_count': len(odometry_tf['path_edges']),
                'dynamic_path': odometry_tf['dynamic_path'],
            },
            'odometry_tf_timing': {
                'status': odometry_tf_timing['status'],
                'pointcloud_records_scanned': (
                    odometry_tf_timing['pointcloud_records_scanned']
                ),
                'tf_records_scanned': (
                    odometry_tf_timing['tf_records_scanned']
                ),
                'invalid_pointcloud_stamp_count': (
                    odometry_tf_timing['invalid_pointcloud_stamp_count']
                ),
                'invalid_transform_stamp_count': (
                    odometry_tf_timing['invalid_transform_stamp_count']
                ),
                'clouds_before_all_dynamic_tf': (
                    odometry_tf_timing['clouds_before_all_dynamic_tf']
                ),
                'future_extrapolation_count': (
                    odometry_tf_timing['future_extrapolation_count']
                ),
                'max_future_gap_ns': odometry_tf_timing['max_future_gap_ns'],
            },
        },
        'recommended_profile_id': payload['recommended_profile_id'],
        'compatible_profile_ids': sorted(
            str(item['id']) for item in recommendations
        ),
        'finding_codes': [str(item['code']) for item in findings],
        'first_action_code': (
            str(findings[0]['code']) if findings else None
        ),
        'privacy': {
            'bag_path_included': False,
            'topic_or_frame_names_included': False,
            'local_commands_included': False,
            'raw_sensor_data_included': False,
            'raw_logs_included': False,
            'free_text_messages_included': False,
            'review_before_sharing': True,
        },
        'network_accessed': False,
        'writes_performed': False,
    }


def build_public_preflight_input_error() -> dict[str, Any]:
    """Return path-free public evidence when the local input cannot be read."""
    empty_capabilities = {
        key: False
        for key in (
            'has_applanix_gsof49',
            'has_applanix_gsof50',
            'has_imu',
            'has_navsatfix',
            'has_odometry',
            'has_pointcloud2',
            'has_tf',
            'has_velodyne_scan',
            'has_velocity_report',
        )
    }
    return {
        'schema_version': PUBLIC_EVIDENCE_SCHEMA_VERSION,
        'schema_uri': PUBLIC_EVIDENCE_SCHEMA_URI,
        'mode': 'bag_public_evidence',
        'status': 'input_error',
        'input': {
            'duration_sec': None,
            'message_count': 0,
            'capabilities': empty_capabilities,
            'topic_type_counts': [],
        },
        'checks': {
            'pointcloud': {
                'status': 'error',
                'field_names': [],
                'rko_lio_compatible': None,
                'timestamp_field': None,
            },
            'timestamp_order': {
                'status': 'error',
                'topic_type_count': 0,
                'records_scanned': 0,
                'reversal_count': 0,
                'invalid_stamp_count': 0,
            },
            'odometry_tf': {
                'status': 'error',
                'records_scanned': 0,
                'tf_records_scanned': 0,
                'readable': False,
                'invalid_frame_count': 0,
                'invalid_transform_count': 0,
                'path_edge_count': 0,
                'dynamic_path': None,
            },
            'odometry_tf_timing': {
                'status': 'error',
                'pointcloud_records_scanned': 0,
                'tf_records_scanned': 0,
                'invalid_pointcloud_stamp_count': 0,
                'invalid_transform_stamp_count': 0,
                'clouds_before_all_dynamic_tf': 0,
                'future_extrapolation_count': 0,
                'max_future_gap_ns': 0,
            },
        },
        'recommended_profile_id': None,
        'compatible_profile_ids': [],
        'finding_codes': ['bag-preflight-input-error'],
        'first_action_code': 'bag-preflight-input-error',
        'privacy': {
            'bag_path_included': False,
            'topic_or_frame_names_included': False,
            'local_commands_included': False,
            'raw_sensor_data_included': False,
            'raw_logs_included': False,
            'free_text_messages_included': False,
            'review_before_sharing': True,
        },
        'network_accessed': False,
        'writes_performed': False,
    }


def _format_topic_list(records: list[dict[str, Any]]) -> str:
    if not records:
        return 'none'
    rendered = [f"{record['name']} ({record['message_count']})" for record in records[:3]]
    remaining = len(records) - min(len(records), 3)
    if remaining > 0:
        rendered.append(f'+{remaining} more')
    return ', '.join(rendered)


def _configured_doctor_command() -> list[str] | None:
    """Return a validated product-doctor argv supplied by the dispatcher."""
    configured = os.environ.get('LIDARSLAM_CLI_COMMAND')
    if not configured:
        return None
    try:
        command = shlex.split(configured)
    except ValueError:
        return None
    if len(command) < 2 or command[-1] != 'doctor':
        return None
    return command


def _public_evidence_command(bag_path: str) -> str:
    """Return the shell-safe public-evidence command for this entrypoint."""
    command = _configured_doctor_command()
    if command is None:
        command = ['python3', 'scripts/preflight_autoware_map_bag.py']
    return shlex.join([*command, bag_path, '--public-json'])


def _product_doctor_command(bag_path: str) -> str | None:
    """Return an exact-input product-doctor retry when dispatched by the CLI."""
    command = _configured_doctor_command()
    if command is None:
        return None
    return shlex.join([*command, bag_path])


def _product_start_command(bag_path: str) -> str | None:
    """Return the matching guided start command when dispatched by the CLI."""
    command = _configured_doctor_command()
    if command is None:
        return None
    return shlex.join([*command[:-1], 'start', bag_path])


def _product_detail_command(bag_path: str) -> str | None:
    """Return the exact-input private detail command for the product card."""
    command = _configured_doctor_command()
    if command is None:
        return None
    return shlex.join([*command, bag_path, '--json'])


def _product_input_summary(summary: dict[str, Any]) -> str:
    """Summarize detected input types without listing topic or frame names."""
    labels = (
        ('has_pointcloud2', 'PointCloud2'),
        ('has_imu', 'Imu'),
        ('has_navsatfix', 'NavSatFix'),
        ('has_odometry', 'Odometry'),
        ('has_tf', 'TF'),
        ('has_velodyne_scan', 'VelodyneScan'),
        ('has_applanix_gsof49', 'Applanix GSOF49'),
        ('has_applanix_gsof50', 'Applanix GSOF50'),
        ('has_velocity_report', 'VelocityReport'),
    )
    detected = [
        label
        for key, label in labels
        if summary['capabilities'][key]
    ]
    return ', '.join(detected) if detected else 'none'


def _product_check_lines(summary: dict[str, Any]) -> list[str]:
    """Return bounded check statuses for the compact product card."""
    checks = [
        ('PointCloud2 record', summary['pointcloud_inspection']['status']),
        ('Header timestamp order', summary['timestamp_order']['status']),
        ('Odometry -> TF', summary['odometry_tf']['status']),
        ('Odometry TF timing', summary['odometry_tf_timing']['status']),
    ]
    return [
        f'  {label}: {status}'
        for label, status in checks
        if status != 'not_applicable'
    ]


def _render_product_text_report(payload: dict[str, Any]) -> str:
    """Render one bounded next-action card for the stable product CLI."""
    summary = payload['summary']
    recommendations = payload['recommendations']
    findings = payload.get('findings') or []
    doctor_command = _product_doctor_command(summary['bag_path'])
    start_command = _product_start_command(summary['bag_path'])
    detail_command = _product_detail_command(summary['bag_path'])
    if doctor_command is None or start_command is None or detail_command is None:
        raise RuntimeError('product doctor command is unavailable')

    if findings:
        status = 'ACTION REQUIRED'
    elif recommendations:
        status = 'READY'
    else:
        status = 'UNSUPPORTED'
    duration_sec = summary['duration_sec']
    duration_text = f'{duration_sec:.3f}s' if duration_sec is not None else 'unknown'
    primary = recommendations[0] if recommendations else None
    profile = (
        f"{primary['label']} [{primary['id']}]"
        if primary is not None else
        'none'
    )

    lines = [
        'lidarslam-map doctor — own-bag readiness',
        f'Status:   {status}',
        f"Bag:      {summary['bag_path']}",
        f'Duration: {duration_text}',
        f"Messages: {summary['message_count']}",
        f'Inputs:   {_product_input_summary(summary)}',
        f'Profile:  {profile}',
        'Checks:',
        *_product_check_lines(summary),
    ]

    if findings:
        first = findings[0]
        lines.extend([
            '',
            'Do this now:',
            f"  [{first['code']}] {first['message']}",
            f"  Next: {first['next_action']}",
        ])
        if len(findings) > 1:
            lines.append(
                '  Follow-up finding codes: '
                + ', '.join(item['code'] for item in findings[1:])
            )
        lines.extend([
            'Rerun after that action:',
            textwrap.indent(doctor_command, '  '),
        ])
    elif primary is not None:
        lines.extend([
            '',
            'Do this now:',
            textwrap.indent(start_command, '  '),
            '  This guided command rechecks setup before writing a new output.',
        ])
    else:
        lines.extend([
            '',
            'Do this now:',
            '  No maintained profile is ready. Review the local details below.',
        ])

    lines.extend([
        '',
        'Need local details?',
        (
            '  Keep this JSON local; it contains the bag path, topic/frame '
            'names, and local commands.'
        ),
        textwrap.indent(detail_command, '  '),
        '',
        'Need public support?',
        '  Do not share this card or the detailed JSON; they contain local data.',
        '  Generate bounded evidence and review it before sharing:',
        textwrap.indent(
            _public_evidence_command(summary['bag_path']),
            '  ',
        ),
    ])
    return '\n'.join(lines)


def render_text_report(payload: dict[str, Any]) -> str:
    """Render a human-readable preflight report."""
    if _configured_doctor_command() is not None:
        return _render_product_text_report(payload)

    summary = payload['summary']
    recommendations = payload['recommendations']
    findings = payload.get('findings') or []
    duration_sec = summary['duration_sec']
    duration_text = f'{duration_sec:.3f}s' if duration_sec is not None else 'unknown'

    lines = [
        'Autoware-Compatible Map Preflight',
        f"bag: {summary['bag_path']}",
        f'duration: {duration_text}',
        f"messages: {summary['message_count']}",
        '',
        'Detected inputs:',
        f"  PointCloud2: {_format_topic_list(summary['topics']['pointcloud2'])}",
        f"  Imu: {_format_topic_list(summary['topics']['imu'])}",
        f"  NavSatFix: {_format_topic_list(summary['topics']['navsatfix'])}",
        f"  Odometry: {_format_topic_list(summary['topics']['odometry'])}",
        f"  VelodyneScan: {_format_topic_list(summary['topics']['velodyne_scan'])}",
        f"  Applanix GSOF49: {_format_topic_list(summary['topics']['applanix_gsof49'])}",
        f"  Applanix GSOF50: {_format_topic_list(summary['topics']['applanix_gsof50'])}",
        f"  TF/TF_STATIC: {_format_topic_list(summary['topics']['tf'])}",
        f"  VelocityReport: {_format_topic_list(summary['topics']['velocity_report'])}",
    ]
    inspection = summary['pointcloud_inspection']
    if inspection['status'] != 'not_applicable':
        lines.append(
            '  PointCloud2 record: '
            f"{inspection['status']} ({inspection['reason']})"
        )
    timestamp_order = summary['timestamp_order']
    lines.append(
        '  Header timestamp order: '
        f"{timestamp_order['status']} ({timestamp_order['reason']})"
    )
    odometry_tf = summary['odometry_tf']
    if odometry_tf['status'] != 'not_applicable':
        lines.append(
            '  Odometry -> TF: '
            f"{odometry_tf['status']} ({odometry_tf['reason']})"
        )
    odometry_tf_timing = summary['odometry_tf_timing']
    if odometry_tf_timing['status'] != 'not_applicable':
        lines.append(
            '  Odometry TF timing: '
            f"{odometry_tf_timing['status']} "
            f"({odometry_tf_timing['reason']})"
        )

    if recommendations:
        primary = recommendations[0]
        lines.extend([
            '',
            f"Recommended path: {primary['label']}",
            f"Recommended profile: {primary['id']}",
            'Why:',
        ])
        for reason in primary['why']:
            lines.append(f'  - {reason}')
        lines.extend([
            'Beginner command:',
            textwrap.indent(
                payload['beginner_commands'][0]['command'],
                '  ',
            ),
            'Beginner command with browser viewer:',
            textwrap.indent(
                payload['beginner_commands'][1]['command'],
                '  ',
            ),
            'Next command:',
            textwrap.indent(primary['command'], '  '),
        ])

        if len(recommendations) > 1:
            lines.append('')
            lines.append('Other compatible paths:')
            for alternative in recommendations[1:]:
                lines.append(f"  - {alternative['label']} [{alternative['id']}]")
                lines.append(textwrap.indent(alternative['command'], '    '))
    else:
        lines.extend([
            '',
            'Recommended path: none',
        ])

    if findings:
        lines.extend(['', 'Findings:'])
        for finding in findings:
            lines.append(f"  [{finding['code']}] {finding['message']}")
            lines.append(f"    Next: {finding['next_action']}")
    elif not recommendations and payload['missing_requirements']:
        lines.extend(['', 'Findings:'])
        for item in payload['missing_requirements']:
            lines.append(f'  - {item}')

    if payload['advisory']:
        lines.append('')
        lines.append('Advisory commands:')
        for item in payload['advisory']:
            lines.append(f"  - {item['label']}")
            lines.append(textwrap.indent(item['command'], '    '))

    lines.extend([
        '',
        'Need public support?',
        (
            '  Keep this full report local; it can contain the bag path, '
            'topic/frame names, and local commands.'
        ),
        '  Generate bounded evidence and review it before sharing:',
        textwrap.indent(
            _public_evidence_command(summary['bag_path']),
            '  ',
        ),
    ])

    return '\n'.join(lines)


def validate_bag_path(bag_path: Path) -> None:
    """Validate that a CLI input points at a rosbag2 directory."""
    if bag_path.is_file():
        if bag_path.suffix == '.db3':
            raise FileNotFoundError(
                f'rosbag2 path points to a .db3 file: {bag_path}. '
                'Pass the rosbag2 directory that contains metadata.yaml, not the .db3 file.'
            )
        raise FileNotFoundError(
            f'rosbag2 path is a file, not a directory: {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )
    if not bag_path.exists():
        raise FileNotFoundError(
            f'rosbag2 directory does not exist: {bag_path}. '
            'Pass the directory that contains metadata.yaml.'
        )
    if not bag_path.is_dir():
        raise FileNotFoundError(
            f'rosbag2 path is not a directory: {bag_path}. '
            'Pass the directory that contains metadata.yaml.'
        )
    if not (bag_path / 'metadata.yaml').is_file():
        raise FileNotFoundError(
            f'metadata.yaml not found under {bag_path}. '
            'Pass the rosbag2 directory that contains metadata.yaml.'
        )


def _profile_help_text() -> str:
    lines = ['Profiles this tool can recommend:']
    for profile_id, description in PROFILE_HELP:
        lines.append(f'  {profile_id}: {description}')
    return '\n'.join(lines)


def _help_epilog() -> str:
    command = os.environ.get(
        'LIDARSLAM_CLI_COMMAND',
        'python3 scripts/preflight_autoware_map_bag.py',
    )
    product_command = (
        command.rsplit(' ', 1)[0]
        if 'LIDARSLAM_CLI_COMMAND' in os.environ
        else 'lidarslam-map'
    )
    return '\n'.join([
        'The input must be the rosbag2 directory that contains metadata.yaml.',
        'Pass /path/to/rosbag2, not /path/to/rosbag2_0.db3.',
        '',
        _profile_help_text(),
        '',
        'Typical commands:',
        f'  {command} /path/to/rosbag2',
        f'  {command} /path/to/rosbag2 --json',
        f'  {command} /path/to/rosbag2 --public-json',
        f'  {product_command} run /path/to/rosbag2 --dry-run',
    ])


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Inspect a rosbag2 directory and suggest the shortest supported '
            'Autoware-compatible map workflow.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_help_epilog(),
    )
    parser.add_argument(
        'bag',
        metavar='rosbag2_dir',
        help='Directory containing metadata.yaml.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all options (this command has no advanced options).',
    )
    output_group = parser.add_mutually_exclusive_group()
    output_group.add_argument(
        '--json',
        action='store_true',
        help='Emit machine-readable JSON instead of the human report.',
    )
    output_group.add_argument(
        '--public-json',
        action='store_true',
        help=(
            'Emit path-free, topic/frame-name-free evidence for a reviewed '
            'public issue instead of the local report.'
        ),
    )
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    bag_path = Path(args.bag).expanduser().resolve()
    try:
        validate_bag_path(bag_path)
        payload = build_preflight_payload(bag_path)
    except (OSError, ValueError) as exc:
        if args.public_json:
            print(json.dumps(
                build_public_preflight_input_error(),
                indent=2,
                sort_keys=True,
            ))
            return 2
        print(f'error: {exc}', file=sys.stderr)
        return 2

    if args.public_json:
        print(json.dumps(
            build_public_preflight_evidence(payload),
            indent=2,
            sort_keys=True,
        ))
    elif args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        print(render_text_report(payload))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
