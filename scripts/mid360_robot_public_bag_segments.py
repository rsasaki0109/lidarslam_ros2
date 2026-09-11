#!/usr/bin/env python3
"""Find contiguous RKO-LIO-safe segments in public MID-360 bags."""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_BAG_SEGMENTS_JSON = 'mid360_robot_public_bag_segments.json'
PUBLIC_BAG_SEGMENTS_MARKDOWN = 'mid360_robot_public_bag_segments.md'
RKO_LIO_MAX_SCAN_DELTA_SEC = 1.0


@dataclass(frozen=True)
class PublicBagSegmentOptions:
    """Options for public bag segment analysis."""

    max_scan_gap_sec: float = RKO_LIO_MAX_SCAN_DELTA_SEC
    min_segment_duration_sec: float = 5.0
    max_scans: int = 0
    min_keypoints: int = 10
    voxel_size: float = 1.0
    min_range: float = 1.0
    max_range: float = 100.0


class RosbagPointCloudTimingReader:
    """Read PointCloud2 scan timestamp ranges from a rosbag2 directory."""

    def read_scan_timings(
        self,
        bag_path: Path,
        pointcloud_topic: str,
        max_scans: int = 0,
        *,
        voxel_size: float = 1.0,
        min_range: float = 1.0,
        max_range: float = 100.0,
    ) -> list[dict[str, Any]]:
        """Return RKO-LIO processed timestamp ranges for one PointCloud2 topic."""
        try:
            from rosbags.highlevel import AnyReader
            from rosbags.typesys import Stores, get_typestore
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError('rosbags is required for bag segment analysis') from exc

        bag_path = bag_path.expanduser().resolve()
        typestore = get_typestore(Stores.LATEST)
        timings: list[dict[str, Any]] = []
        with AnyReader([bag_path], default_typestore=typestore) as reader:
            connections = [conn for conn in reader.connections if conn.topic == pointcloud_topic]
            if not connections:
                raise ValueError(f'pointcloud topic not found in bag: {pointcloud_topic}')
            for connection, timestamp_ns, raw in reader.messages(connections=connections):
                msg = reader.deserialize(raw, connection.msgtype)
                timings.append(
                    pointcloud_scan_timing(
                        len(timings),
                        int(timestamp_ns),
                        msg,
                        voxel_size=voxel_size,
                        min_range=min_range,
                        max_range=max_range,
                    )
                )
                if max_scans > 0 and len(timings) >= max_scans:
                    break
        return timings


class PublicBagSegmentReportBuilder:
    """Build public dataset bag segment reports from a map-candidate manifest."""

    def __init__(
        self,
        manifest_path: Path,
        output_dir: Path,
        timing_reader: Any | None = None,
    ) -> None:
        self._manifest_path = manifest_path.expanduser().resolve()
        self._output_dir = output_dir.expanduser().resolve()
        self._timing_reader = timing_reader or RosbagPointCloudTimingReader()

    def build(
        self,
        dataset_ids: tuple[str, ...] = (),
        options: PublicBagSegmentOptions | None = None,
    ) -> dict[str, Any]:
        """Build a segment report for selected public datasets."""
        options = options or PublicBagSegmentOptions()
        manifest = _load_json(self._manifest_path)
        wanted_ids = set(dataset_ids)
        rows = []
        skipped = []
        for candidate in manifest.get('candidates') or []:
            dataset_id = str(candidate.get('dataset_id') or '')
            if wanted_ids and dataset_id not in wanted_ids:
                skipped.append({'dataset_id': dataset_id, 'skip_reason': 'dataset not selected'})
                continue
            rows.append(self._build_row(candidate, options))

        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': _overall_status(rows),
            'manifest_path': str(self._manifest_path),
            'output_dir': str(self._output_dir),
            'selection': {'dataset_ids': list(dataset_ids)},
            'options': {
                'max_scan_gap_sec': options.max_scan_gap_sec,
                'min_segment_duration_sec': options.min_segment_duration_sec,
                'max_scans': options.max_scans,
                'min_keypoints': options.min_keypoints,
                'voxel_size': options.voxel_size,
                'min_range': options.min_range,
                'max_range': options.max_range,
            },
            'datasets': rows,
            'skipped': skipped,
            'counts': _counts(rows, skipped),
        }

    def write(self, report: dict[str, Any]) -> dict[str, Path]:
        """Write segment report JSON and Markdown."""
        self._output_dir.mkdir(parents=True, exist_ok=True)
        json_path = self._output_dir / PUBLIC_BAG_SEGMENTS_JSON
        markdown_path = self._output_dir / PUBLIC_BAG_SEGMENTS_MARKDOWN
        json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
        markdown_path.write_text(render_public_bag_segments_markdown(report) + '\n',
                                 encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    def _build_row(
        self,
        candidate: dict[str, Any],
        options: PublicBagSegmentOptions,
    ) -> dict[str, Any]:
        dataset_id = str(candidate.get('dataset_id') or '')
        topics = candidate.get('selected_topics') or {}
        pointcloud_topic = str(topics.get('pointcloud') or '')
        bag_path = Path(str(candidate.get('selected_bag_path') or '')).expanduser()
        try:
            timings = self._timing_reader.read_scan_timings(
                bag_path=bag_path,
                pointcloud_topic=pointcloud_topic,
                max_scans=options.max_scans,
                voxel_size=options.voxel_size,
                min_range=options.min_range,
                max_range=options.max_range,
            )
            split = split_contiguous_scan_segments(timings, options)
            recommended = recommend_segment(split['segments'])
            status = 'PASS' if recommended else ('FAIL' if not split['segments'] else 'WARN')
            error = ''
        except Exception as exc:
            timings = []
            split = {'segments': [], 'gaps': []}
            recommended = {}
            status = 'FAIL'
            error = str(exc)

        return {
            'dataset_id': dataset_id,
            'title': candidate.get('title', ''),
            'status': status,
            'error': error,
            'selected_bag_path': str(bag_path),
            'selected_topics': {
                'pointcloud': pointcloud_topic,
                'imu': str(topics.get('imu') or ''),
            },
            'scan_count': len(timings),
            'segments': split['segments'],
            'gaps': split['gaps'],
            'recommended_segment': recommended,
            'suggested_next_steps': _suggested_next_steps(dataset_id, recommended),
        }


def pointcloud_scan_timing(
    scan_index: int,
    receive_time_ns: int,
    msg: Any,
    *,
    voxel_size: float = 1.0,
    min_range: float = 1.0,
    max_range: float = 100.0,
) -> dict[str, Any]:
    """Build one scan timing row using RKO-LIO's timestamp processing rules."""
    field = _timestamp_field(msg)
    raw_min, raw_max = _point_time_min_max(msg, field)
    header_stamp_ns = _header_stamp_ns(msg)
    processed = process_rko_lio_timestamps(
        raw_min=raw_min,
        raw_max=raw_max,
        header_stamp_ns=header_stamp_ns,
    )
    quality = pointcloud_preprocess_quality(
        msg,
        voxel_size=voxel_size,
        min_range=min_range,
        max_range=max_range,
    )
    return {
        'scan_index': int(scan_index),
        'receive_time_ns': int(receive_time_ns),
        'header_stamp_ns': header_stamp_ns,
        'point_count': int(getattr(msg, 'height', 0)) * int(getattr(msg, 'width', 0)),
        'timestamp_field': field.name,
        'timestamp_datatype': int(field.datatype),
        'raw_time_min': raw_min,
        'raw_time_max': raw_max,
        **quality,
        **processed,
    }


def pointcloud_preprocess_quality(
    msg: Any,
    *,
    voxel_size: float = 1.0,
    min_range: float = 1.0,
    max_range: float = 100.0,
) -> dict[str, Any]:
    """Estimate RKO-LIO range filtering and keypoint voxel counts."""
    try:
        import numpy as np
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError('numpy is required for PointCloud2 quality analysis') from exc

    x = _point_field_array(np, msg, _named_field(msg, 'x')).astype(np.float64, copy=False)
    y = _point_field_array(np, msg, _named_field(msg, 'y')).astype(np.float64, copy=False)
    z = _point_field_array(np, msg, _named_field(msg, 'z')).astype(np.float64, copy=False)
    ranges = np.sqrt((x * x) + (y * y) + (z * z))
    mask = np.isfinite(ranges) & (ranges > float(min_range)) & (ranges < float(max_range))
    clipped_count = int(np.count_nonzero(mask))
    keypoint_count = _voxel_keypoint_count(
        np,
        x[mask],
        y[mask],
        z[mask],
        max(1e-6, float(voxel_size) * 1.5),
    )
    return {
        'clipped_point_count': clipped_count,
        'keypoint_count': keypoint_count,
    }


def process_rko_lio_timestamps(
    raw_min: float,
    raw_max: float,
    header_stamp_ns: int,
    multiplier_to_seconds: float = 0.0,
    force_absolute: bool = False,
    force_relative: bool = False,
) -> dict[str, Any]:
    """Apply the same timestamp mode inference used by RKO-LIO."""
    scan_duration = abs(float(raw_max) - float(raw_min))
    multiplier = float(multiplier_to_seconds)
    if multiplier < 1e-12:
        multiplier = 1e-9 if scan_duration > 100.0 else 1.0

    min_sec = float(raw_min) * multiplier
    max_sec = float(raw_max) * multiplier
    header_sec = int(header_stamp_ns) / 1_000_000_000.0

    absolute = (
        bool(force_absolute)
        or abs(header_sec - min_sec) < 0.001
        or abs(header_sec - max_sec) < 0.010
    )
    if absolute:
        mode = 'absolute'
        covered = True
    else:
        relative = bool(force_relative) or abs(min_sec) < 0.001 or abs(max_sec) < 0.010
        if relative:
            min_sec += header_sec
            max_sec += header_sec
            mode = 'relative'
            covered = True
        else:
            mode = 'unsupported'
            covered = False

    return {
        'timestamp_mode': mode,
        'timestamp_multiplier': multiplier,
        'rko_timestamp_supported': covered,
        'processed_min_sec': min_sec,
        'processed_max_sec': max_sec,
    }


def split_contiguous_scan_segments(
    timings: list[dict[str, Any]],
    options: PublicBagSegmentOptions,
) -> dict[str, list[dict[str, Any]]]:
    """Split scans when RKO-LIO would see a LiDAR timestamp delta above the limit."""
    if not timings:
        return {'segments': [], 'gaps': []}

    max_gap = max(0.0, float(options.max_scan_gap_sec))
    min_duration = max(0.0, float(options.min_segment_duration_sec))
    gaps: list[dict[str, Any]] = []
    segments: list[dict[str, Any]] = []
    start_index = _next_valid_scan_index(timings, 0, options)
    internal_gaps: list[float] = []
    bag_start_receive_ns = int(timings[0]['receive_time_ns'])
    if start_index >= len(timings):
        return {
            'segments': [],
            'gaps': [_quality_gap_row(scan, options) for scan in timings],
        }

    for index in range(start_index + 1, len(timings)):
        if not _scan_quality_ok(timings[index], options):
            gaps.append(_quality_gap_row(timings[index], options))
            if start_index < index:
                segments.append(
                    _segment_row(
                        segment_index=len(segments),
                        scans=timings[start_index:index],
                        max_internal_gap_sec=max(internal_gaps, default=0.0),
                        options=options,
                        min_duration_sec=min_duration,
                        bag_start_receive_ns=bag_start_receive_ns,
                    )
                )
            start_index = _next_valid_scan_index(timings, index + 1, options)
            internal_gaps = []
            continue
        if start_index >= index:
            continue
        delta = float(timings[index]['processed_max_sec']) - float(
            timings[index - 1]['processed_max_sec']
        )
        if abs(delta) > max_gap:
            gaps.append(_gap_row(timings, index, delta))
            segments.append(
                _segment_row(
                    segment_index=len(segments),
                    scans=timings[start_index:index],
                    max_internal_gap_sec=max(internal_gaps, default=0.0),
                    options=options,
                    min_duration_sec=min_duration,
                    bag_start_receive_ns=bag_start_receive_ns,
                )
            )
            start_index = index
            internal_gaps = []
        else:
            internal_gaps.append(abs(delta))

    if start_index < len(timings):
        segments.append(
            _segment_row(
                segment_index=len(segments),
                scans=timings[start_index:],
                max_internal_gap_sec=max(internal_gaps, default=0.0),
                options=options,
                min_duration_sec=min_duration,
                bag_start_receive_ns=bag_start_receive_ns,
            )
        )
    return {'segments': segments, 'gaps': gaps}


def recommend_segment(segments: list[dict[str, Any]]) -> dict[str, Any]:
    """Return the longest segment that meets the minimum duration gate."""
    ready = [segment for segment in segments if segment.get('ready_for_clip')]
    if not ready:
        return {}
    return max(
        ready,
        key=lambda item: (
            float(item.get('duration_sec') or 0.0),
            int(item.get('scan_count') or 0),
        ),
    )


def render_public_bag_segments_markdown(report: dict[str, Any]) -> str:
    """Render public bag segment report as Markdown."""
    counts = report.get('counts') or {}
    lines = [
        '# MID-360 Public Bag Segments',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- created_at: `{report.get('created_at', '')}`",
        f"- manifest_path: `{report.get('manifest_path', '')}`",
        f"- total: `{counts.get('total', 0)}`",
        f"- pass: `{counts.get('pass', 0)}`",
        f"- warn: `{counts.get('warn', 0)}`",
        f"- fail: `{counts.get('fail', 0)}`",
        '',
        '## Dataset Summary',
        '',
        (
            '| Dataset | Status | Scans | Segments | Gaps | Recommended | Duration | '
            'Start Offset | End Offset |'
        ),
        '| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: |',
    ]
    for row in report.get('datasets') or []:
        recommended = row.get('recommended_segment') or {}
        lines.append(
            '| '
            + ' | '.join([
                f"`{row.get('dataset_id', '')}`",
                f"`{row.get('status', '')}`",
                str(row.get('scan_count', 0)),
                str(len(row.get('segments') or [])),
                str(len(row.get('gaps') or [])),
                f"`{recommended.get('segment_id', '')}`" if recommended else '',
                _fmt_seconds(recommended.get('duration_sec')),
                _fmt_seconds(recommended.get('clip_start_offset_sec')),
                _fmt_seconds(recommended.get('clip_end_offset_sec')),
            ])
            + ' |'
        )

    for row in report.get('datasets') or []:
        lines.extend([
            '',
            f"## {row.get('dataset_id', '')}",
            '',
            f"- status: `{row.get('status', '')}`",
            f"- selected_bag_path: `{row.get('selected_bag_path', '')}`",
            f"- pointcloud_topic: `{(row.get('selected_topics') or {}).get('pointcloud', '')}`",
            f"- imu_topic: `{(row.get('selected_topics') or {}).get('imu', '')}`",
            f"- scan_count: `{row.get('scan_count', 0)}`",
        ])
        if row.get('error'):
            lines.append(f"- error: `{row.get('error')}`")

        recommended = row.get('recommended_segment') or {}
        if recommended:
            lines.extend([
                '',
                '### Recommended Segment',
                '',
                f"- segment_id: `{recommended.get('segment_id', '')}`",
                f"- scan_range: `{recommended.get('start_scan_index')}..{recommended.get('end_scan_index')}`",
                f"- scan_count: `{recommended.get('scan_count')}`",
                f"- duration_sec: `{recommended.get('duration_sec')}`",
                f"- clip_start_time_ns: `{recommended.get('clip_start_time_ns')}`",
                f"- clip_end_time_ns: `{recommended.get('clip_end_time_ns')}`",
                f"- max_internal_gap_sec: `{recommended.get('max_internal_gap_sec')}`",
                f"- min_keypoint_count: `{recommended.get('min_keypoint_count')}`",
            ])

        gaps = row.get('gaps') or []
        lines.extend(['', '### First Gaps', ''])
        if gaps:
            for gap in gaps[:10]:
                lines.append(_gap_markdown_line(gap))
        else:
            lines.append('- none')

        steps = row.get('suggested_next_steps') or []
        if steps:
            lines.extend(['', '### Suggested Next Commands', ''])
            for step in steps:
                lines.append(f'- `{step}`')

    skipped = report.get('skipped') or []
    lines.extend(['', '## Skipped', ''])
    if skipped:
        for item in skipped:
            lines.append(f"- `{item.get('dataset_id', '')}`: {item.get('skip_reason', '')}")
    else:
        lines.append('- none')
    return '\n'.join(lines)


def _timestamp_field(msg: Any) -> Any:
    for field in getattr(msg, 'fields', []) or []:
        if field.name in ('t', 'timestamp', 'time', 'stamps') and int(field.count) != 0:
            return field
    raise ValueError('PointCloud2 has no RKO-LIO timestamp field')


def _named_field(msg: Any, name: str) -> Any:
    for field in getattr(msg, 'fields', []) or []:
        if field.name == name:
            return field
    raise ValueError(f'PointCloud2 missing field: {name}')


def _point_time_min_max(msg: Any, field: Any) -> tuple[float, float]:
    try:
        import numpy as np
    except ModuleNotFoundError as exc:
        raise ModuleNotFoundError('numpy is required for PointCloud2 timing analysis') from exc

    values = _point_field_array(np, msg, field)
    return float(np.min(values)), float(np.max(values))


def _point_field_array(np: Any, msg: Any, field: Any) -> Any:
    dtype = _numpy_dtype(np, int(field.datatype), bool(getattr(msg, 'is_bigendian', False)))
    height = int(getattr(msg, 'height', 0))
    width = int(getattr(msg, 'width', 0))
    if height <= 0 or width <= 0:
        raise ValueError('PointCloud2 has no points')
    data = getattr(msg, 'data')
    buffer = data if hasattr(data, '__array_interface__') else bytes(data)
    values = np.ndarray(
        shape=(height, width),
        dtype=dtype,
        buffer=buffer,
        offset=int(field.offset),
        strides=(int(msg.row_step), int(msg.point_step)),
    )
    return values.reshape(height * width)


def _voxel_keypoint_count(np: Any, x: Any, y: Any, z: Any, voxel_size: float) -> int:
    if len(x) == 0:
        return 0
    voxels = np.floor(np.column_stack((x, y, z)) / float(voxel_size)).astype(np.int64, copy=False)
    return int(np.unique(voxels, axis=0).shape[0])


def _numpy_dtype(np: Any, datatype: int, is_bigendian: bool) -> Any:
    endian = '>' if is_bigendian else '<'
    mapping = {
        1: 'i1',
        2: 'u1',
        3: f'{endian}i2',
        4: f'{endian}u2',
        5: f'{endian}i4',
        6: f'{endian}u4',
        7: f'{endian}f4',
        8: f'{endian}f8',
    }
    if datatype not in mapping:
        raise ValueError(f'unsupported PointCloud2 timestamp datatype: {datatype}')
    return np.dtype(mapping[datatype])


def _header_stamp_ns(msg: Any) -> int:
    header = getattr(msg, 'header', None)
    stamp = getattr(header, 'stamp', None)
    sec = getattr(stamp, 'sec', 0)
    nanosec = getattr(stamp, 'nanosec', 0)
    return int(sec) * 1_000_000_000 + int(nanosec)


def _segment_row(
    segment_index: int,
    scans: list[dict[str, Any]],
    max_internal_gap_sec: float,
    options: PublicBagSegmentOptions,
    min_duration_sec: float,
    bag_start_receive_ns: int,
) -> dict[str, Any]:
    first = scans[0]
    last = scans[-1]
    first_receive_ns = int(first['receive_time_ns'])
    duration_sec = max(
        0.0,
        float(last['processed_max_sec']) - float(first['processed_min_sec']),
    )
    mean_period = duration_sec / max(1, len(scans) - 1)
    ready = (
        len(scans) >= 2
        and duration_sec >= min_duration_sec
        and max_internal_gap_sec <= float(options.max_scan_gap_sec)
        and all(scan.get('rko_timestamp_supported') for scan in scans)
        and all(_scan_quality_ok(scan, options) for scan in scans)
    )
    keypoint_counts = [int(scan.get('keypoint_count', 0)) for scan in scans]
    return {
        'segment_id': f'segment_{segment_index:03d}',
        'start_scan_index': int(first['scan_index']),
        'end_scan_index': int(last['scan_index']),
        'scan_count': len(scans),
        'duration_sec': duration_sec,
        'mean_scan_period_sec': mean_period,
        'max_internal_gap_sec': max_internal_gap_sec,
        'min_keypoint_count': min(keypoint_counts) if keypoint_counts else 0,
        'max_keypoint_count': max(keypoint_counts) if keypoint_counts else 0,
        'ready_for_clip': ready,
        'clip_start_time_ns': first_receive_ns,
        'clip_end_time_ns': int(last['receive_time_ns']),
        'clip_start_offset_sec': (
            first_receive_ns - int(bag_start_receive_ns)
        ) / 1_000_000_000.0,
        'clip_end_offset_sec': (
            int(last['receive_time_ns']) - int(bag_start_receive_ns)
        ) / 1_000_000_000.0,
        'rko_start_sec': float(first['processed_min_sec']),
        'rko_end_sec': float(last['processed_max_sec']),
    }


def _gap_row(timings: list[dict[str, Any]], index: int, delta_sec: float) -> dict[str, Any]:
    previous = timings[index - 1]
    current = timings[index]
    return {
        'previous_scan_index': int(previous['scan_index']),
        'current_scan_index': int(current['scan_index']),
        'delta_sec': float(delta_sec),
        'previous_receive_time_ns': int(previous['receive_time_ns']),
        'current_receive_time_ns': int(current['receive_time_ns']),
        'reason': 'scan timestamp delta exceeds limit',
    }


def _quality_gap_row(scan: dict[str, Any], options: PublicBagSegmentOptions) -> dict[str, Any]:
    return {
        'previous_scan_index': int(scan.get('scan_index', 0)) - 1,
        'current_scan_index': int(scan.get('scan_index', 0)),
        'delta_sec': None,
        'previous_receive_time_ns': None,
        'current_receive_time_ns': int(scan.get('receive_time_ns', 0)),
        'reason': 'scan keypoint count below RKO-LIO minimum',
        'keypoint_count': int(scan.get('keypoint_count', 0)),
        'min_keypoints': int(options.min_keypoints),
    }


def _scan_quality_ok(scan: dict[str, Any], options: PublicBagSegmentOptions) -> bool:
    return int(scan.get('keypoint_count', int(options.min_keypoints))) >= int(options.min_keypoints)


def _next_valid_scan_index(
    timings: list[dict[str, Any]],
    start: int,
    options: PublicBagSegmentOptions,
) -> int:
    for index in range(start, len(timings)):
        if _scan_quality_ok(timings[index], options):
            return index
    return len(timings)


def _gap_markdown_line(gap: dict[str, Any]) -> str:
    reason = gap.get('reason', '')
    if gap.get('delta_sec') is None:
        return (
            f"- scan `{gap.get('current_scan_index')}`: {reason}, "
            f"keypoints `{gap.get('keypoint_count')}`"
        )
    return (
        f"- after scan `{gap.get('previous_scan_index')}`: "
        f"`{gap.get('delta_sec')}` sec, {reason}"
    )


def _suggested_next_steps(dataset_id: str, recommended: dict[str, Any]) -> list[str]:
    if not recommended:
        return []
    return [
        (
            'python3 scripts/analyze_mid360_robot_public_bag_segments.py '
            f'--datasets {dataset_id} --write'
        ),
    ]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _overall_status(rows: list[dict[str, Any]]) -> str:
    if not rows:
        return 'EMPTY'
    statuses = [str(row.get('status') or '') for row in rows]
    if any(status == 'FAIL' for status in statuses):
        return 'FAIL'
    if any(status == 'WARN' for status in statuses):
        return 'WARN'
    return 'PASS'


def _counts(rows: list[dict[str, Any]], skipped: list[dict[str, Any]]) -> dict[str, int]:
    statuses = [str(row.get('status') or '') for row in rows]
    return {
        'total': len(rows),
        'pass': sum(1 for status in statuses if status == 'PASS'),
        'warn': sum(1 for status in statuses if status == 'WARN'),
        'fail': sum(1 for status in statuses if status == 'FAIL'),
        'skipped': len(skipped),
    }


def _fmt_seconds(value: Any) -> str:
    if value is None:
        return ''
    try:
        return f'{float(value):.1f}s'
    except Exception:
        return str(value)


if __name__ == "__main__":
    raise SystemExit("mid360_robot_public_bag_segments is a library module; import it instead of running it directly.")
