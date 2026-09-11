#!/usr/bin/env python3
"""Cloud-overlap analysis for public MID-360 GT loop candidates."""

from __future__ import annotations

import bisect
import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Protocol
from zipfile import ZipFile

import numpy as np

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_LOOP_CLOUD_ANALYSIS_JSON = 'mid360_robot_public_loop_cloud_analysis.json'
PUBLIC_LOOP_CLOUD_ANALYSIS_MARKDOWN = 'mid360_robot_public_loop_cloud_analysis.md'


@dataclass(frozen=True)
class Pose:
    """One TUM-style lidar pose."""

    stamp: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class ScanPoints:
    """PointCloud2 points sampled from one scan."""

    stamp: float
    receive_time_ns: int
    points: np.ndarray


class PointCloudWindowReader(Protocol):
    """Protocol for reading PointCloud2 scans in a time window."""

    def read_window(
        self,
        bag_path: Path,
        topic: str,
        start_stamp: float,
        end_stamp: float,
        *,
        max_scans: int,
        max_points_per_scan: int,
        min_range_m: float,
        max_range_m: float,
    ) -> list[ScanPoints]:
        """Read sampled scans in the requested window."""


@dataclass(frozen=True)
class LoopCloudAnalysisOptions:
    """Options for public loop cloud-overlap analysis."""

    loop_candidates_json: Path
    gt_zip: Path
    bag_path: Path
    output_dir: Path
    sequence_id: str = 'outdoor_kidnap'
    pointcloud_topic: str = '/livox/points'
    candidate_index: int = 0
    window_sec: float = 1.0
    voxel_size_m: float = 0.5
    max_scans_per_window: int = 30
    max_points_per_scan: int = 4000
    max_points_per_cloud: int = 30000
    min_range_m: float = 1.0
    max_range_m: float = 80.0
    pass_median_nn_m: float = 1.0
    pass_coverage_within_1m: float = 0.30


class RosbagPointCloudWindowReader:
    """Read PointCloud2 XYZ samples from a rosbag2 window."""

    def read_window(
        self,
        bag_path: Path,
        topic: str,
        start_stamp: float,
        end_stamp: float,
        *,
        max_scans: int,
        max_points_per_scan: int,
        min_range_m: float,
        max_range_m: float,
    ) -> list[ScanPoints]:
        """Read sampled PointCloud2 scans with header stamps inside a window."""
        try:
            from rosbags.highlevel import AnyReader
            from rosbags.typesys import Stores, get_typestore
        except ModuleNotFoundError as exc:
            raise ModuleNotFoundError('rosbags is required for loop cloud analysis') from exc

        typestore = get_typestore(Stores.LATEST)
        scans: list[ScanPoints] = []
        with AnyReader([bag_path.expanduser().resolve()], default_typestore=typestore) as reader:
            connections = [conn for conn in reader.connections if conn.topic == topic]
            if not connections:
                raise ValueError(f'pointcloud topic not found in bag: {topic}')
            for connection, timestamp_ns, raw in reader.messages(connections=connections):
                msg = reader.deserialize(raw, connection.msgtype)
                stamp = _stamp_sec(msg)
                if stamp < start_stamp:
                    continue
                if stamp > end_stamp:
                    if scans:
                        break
                    continue
                points = _pointcloud_xyz(msg, min_range_m=min_range_m, max_range_m=max_range_m)
                points = _limit_rows(points, max_points_per_scan)
                scans.append(
                    ScanPoints(stamp=stamp, receive_time_ns=int(timestamp_ns), points=points)
                )
                if max_scans > 0 and len(scans) >= max_scans:
                    break
        return scans


class PublicLoopCloudAnalyzer:
    """Analyze local cloud overlap at a public GT loop candidate."""

    def __init__(self, reader: PointCloudWindowReader | None = None) -> None:
        self._reader = reader or RosbagPointCloudWindowReader()

    def analyze(self, options: LoopCloudAnalysisOptions) -> dict[str, Any]:
        """Run cloud-overlap analysis and return a JSON-friendly report."""
        candidates_payload = _load_json(options.loop_candidates_json)
        sequence = _select_sequence(candidates_payload, options.sequence_id)
        candidates = sequence.get('loop_candidates') or []
        if not candidates:
            raise ValueError(f'sequence has no loop candidates: {options.sequence_id}')
        if options.candidate_index < 0 or options.candidate_index >= len(candidates):
            raise ValueError(f'candidate index out of range: {options.candidate_index}')

        candidate = candidates[options.candidate_index]
        poses = load_gt_trajectory_from_zip(options.gt_zip, str(sequence['trajectory_file']))
        pose_index = PoseIndex(poses)
        start_stamp = float(candidate['start_stamp'])
        end_stamp = float(candidate['end_stamp'])
        half_window = max(0.0, float(options.window_sec))
        start_scans = self._reader.read_window(
            options.bag_path,
            options.pointcloud_topic,
            start_stamp - half_window,
            start_stamp + half_window,
            max_scans=max(1, int(options.max_scans_per_window)),
            max_points_per_scan=max(1, int(options.max_points_per_scan)),
            min_range_m=float(options.min_range_m),
            max_range_m=float(options.max_range_m),
        )
        end_scans = self._reader.read_window(
            options.bag_path,
            options.pointcloud_topic,
            end_stamp - half_window,
            end_stamp + half_window,
            max_scans=max(1, int(options.max_scans_per_window)),
            max_points_per_scan=max(1, int(options.max_points_per_scan)),
            min_range_m=float(options.min_range_m),
            max_range_m=float(options.max_range_m),
        )
        start_cloud = _build_world_cloud(start_scans, pose_index)
        end_cloud = _build_world_cloud(end_scans, pose_index)
        start_cloud = voxel_downsample(start_cloud, options.voxel_size_m, options.max_points_per_cloud)
        end_cloud = voxel_downsample(end_cloud, options.voxel_size_m, options.max_points_per_cloud)
        metrics = cloud_overlap_metrics(start_cloud, end_cloud)
        status = _status(metrics, options)

        report = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'loop_candidates_json': str(options.loop_candidates_json.expanduser().resolve()),
            'gt_zip': str(options.gt_zip.expanduser().resolve()),
            'bag_path': str(options.bag_path.expanduser().resolve()),
            'output_dir': str(options.output_dir.expanduser().resolve()),
            'sequence_id': options.sequence_id,
            'pointcloud_topic': options.pointcloud_topic,
            'candidate_index': int(options.candidate_index),
            'candidate': candidate,
            'options': _options_payload(options),
            'windows': {
                'start': _window_summary(start_scans, start_cloud),
                'end': _window_summary(end_scans, end_cloud),
            },
            'overlap': metrics,
            'checks': _checks(metrics, options, start_scans, end_scans),
            'next_actions': _next_actions(status),
        }
        write_public_loop_cloud_report(report, options.output_dir)
        return report


class PoseIndex:
    """Nearest-neighbor lookup over sorted poses by timestamp."""

    def __init__(self, poses: list[Pose]) -> None:
        if not poses:
            raise ValueError('GT trajectory is empty')
        self._poses = sorted(poses, key=lambda pose: pose.stamp)
        self._stamps = [pose.stamp for pose in self._poses]

    def nearest(self, stamp: float) -> Pose:
        """Return the nearest pose by timestamp."""
        index = bisect.bisect_left(self._stamps, stamp)
        if index <= 0:
            return self._poses[0]
        if index >= len(self._poses):
            return self._poses[-1]
        before = self._poses[index - 1]
        after = self._poses[index]
        return before if abs(before.stamp - stamp) <= abs(after.stamp - stamp) else after


def load_gt_trajectory_from_zip(zip_path: Path, member_name: str) -> list[Pose]:
    """Load a TUM trajectory from a public GT zip file."""
    with ZipFile(zip_path.expanduser().resolve()) as archive:
        lines = archive.read(member_name).decode('utf-8').splitlines()
    poses: list[Pose] = []
    for line in lines:
        text = line.strip()
        if not text or text.startswith('#'):
            continue
        parts = text.split()
        if len(parts) < 8:
            continue
        poses.append(Pose(*(float(value) for value in parts[:8])))
    if not poses:
        raise ValueError(f'no poses loaded from {member_name}')
    return poses


def render_public_loop_cloud_markdown(report: dict[str, Any]) -> str:
    """Render loop cloud analysis as Markdown."""
    overlap = report.get('overlap') or {}
    start = (report.get('windows') or {}).get('start') or {}
    end = (report.get('windows') or {}).get('end') or {}
    lines = [
        '# MID-360 Public Loop Cloud Analysis',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- sequence_id: `{report.get('sequence_id', '')}`",
        f"- candidate_index: `{report.get('candidate_index', '')}`",
        f"- pointcloud_topic: `{report.get('pointcloud_topic', '')}`",
        f"- start_scans: `{start.get('scan_count', 0)}`",
        f"- end_scans: `{end.get('scan_count', 0)}`",
        f"- start_cloud_points: `{start.get('world_points', 0)}`",
        f"- end_cloud_points: `{end.get('world_points', 0)}`",
        f"- median_nn_m: `{overlap.get('symmetric_median_nn_m', 0.0):.3f}`",
        f"- p90_nn_m: `{overlap.get('symmetric_p90_nn_m', 0.0):.3f}`",
        f"- coverage_within_1m: `{overlap.get('coverage_within_1m', 0.0):.3f}`",
        '',
        '## Checks',
        '',
    ]
    for check in report.get('checks') or []:
        lines.append(f"- `{check.get('status', '')}` `{check.get('id', '')}`: {check.get('message', '')}")
    steps = report.get('next_actions') or []
    if steps:
        lines.extend(['', '## Next Actions', ''])
        lines.extend(f'- {step}' for step in steps)
    return '\n'.join(lines)


def write_public_loop_cloud_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write JSON and Markdown artifacts."""
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / PUBLIC_LOOP_CLOUD_ANALYSIS_JSON
    markdown_path = output_dir / PUBLIC_LOOP_CLOUD_ANALYSIS_MARKDOWN
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(render_public_loop_cloud_markdown(report) + '\n', encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def voxel_downsample(points: np.ndarray, voxel_size_m: float, max_points: int) -> np.ndarray:
    """Voxel-downsample XYZ points and cap the result deterministically."""
    if points.size == 0:
        return points.reshape(0, 3)
    voxel_size = max(1e-6, float(voxel_size_m))
    keys = np.floor(points / voxel_size).astype(np.int64, copy=False)
    _, first_indices = np.unique(keys, axis=0, return_index=True)
    downsampled = points[np.sort(first_indices)]
    return _limit_rows(downsampled, max_points)


def cloud_overlap_metrics(start_cloud: np.ndarray, end_cloud: np.ndarray) -> dict[str, Any]:
    """Compute symmetric nearest-neighbor overlap metrics between two clouds."""
    if len(start_cloud) == 0 or len(end_cloud) == 0:
        return {
            'status': 'FAIL',
            'reason': 'empty cloud',
            'start_to_end': {},
            'end_to_start': {},
            'symmetric_median_nn_m': math.inf,
            'symmetric_p90_nn_m': math.inf,
            'coverage_within_0p5m': 0.0,
            'coverage_within_1m': 0.0,
            'coverage_within_2m': 0.0,
        }
    start_dist = _nearest_distances(start_cloud, end_cloud)
    end_dist = _nearest_distances(end_cloud, start_cloud)
    both = np.concatenate([start_dist, end_dist])
    return {
        'status': 'PASS',
        'start_to_end': _distance_summary(start_dist),
        'end_to_start': _distance_summary(end_dist),
        'symmetric_median_nn_m': float(np.median(both)),
        'symmetric_p90_nn_m': float(np.percentile(both, 90)),
        'coverage_within_0p5m': float(np.mean(both <= 0.5)),
        'coverage_within_1m': float(np.mean(both <= 1.0)),
        'coverage_within_2m': float(np.mean(both <= 2.0)),
    }


def _build_world_cloud(scans: list[ScanPoints], pose_index: PoseIndex) -> np.ndarray:
    clouds = []
    for scan in scans:
        pose = pose_index.nearest(scan.stamp)
        clouds.append(transform_points(scan.points, pose))
    if not clouds:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack(clouds)


def transform_points(points: np.ndarray, pose: Pose) -> np.ndarray:
    """Transform lidar-frame points into the GT trajectory frame."""
    rotation = _quat_to_rotation_matrix(pose.qx, pose.qy, pose.qz, pose.qw)
    translation = np.array([pose.x, pose.y, pose.z], dtype=np.float64)
    return points @ rotation.T + translation


def _quat_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def _nearest_distances(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    try:
        from scipy.spatial import cKDTree
    except ModuleNotFoundError:
        return _nearest_distances_chunked(source, target)
    distances, _ = cKDTree(target).query(source, k=1)
    return distances.astype(np.float64, copy=False)


def _nearest_distances_chunked(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    chunks = []
    for start in range(0, len(source), 512):
        chunk = source[start:start + 512]
        delta = chunk[:, None, :] - target[None, :, :]
        chunks.append(np.sqrt(np.sum(delta * delta, axis=2)).min(axis=1))
    return np.concatenate(chunks)


def _distance_summary(distances: np.ndarray) -> dict[str, float]:
    return {
        'count': int(len(distances)),
        'mean_m': float(np.mean(distances)),
        'median_m': float(np.median(distances)),
        'p90_m': float(np.percentile(distances, 90)),
        'p95_m': float(np.percentile(distances, 95)),
        'max_m': float(np.max(distances)),
    }


def _pointcloud_xyz(msg: Any, *, min_range_m: float, max_range_m: float) -> np.ndarray:
    x = _point_field_array(msg, 'x').astype(np.float64, copy=False)
    y = _point_field_array(msg, 'y').astype(np.float64, copy=False)
    z = _point_field_array(msg, 'z').astype(np.float64, copy=False)
    points = np.column_stack((x, y, z))
    ranges = np.sqrt(np.sum(points * points, axis=1))
    mask = (
        np.isfinite(ranges)
        & (ranges > float(min_range_m))
        & (ranges < float(max_range_m))
    )
    return points[mask]


def _point_field_array(msg: Any, name: str) -> np.ndarray:
    field = _named_field(msg, name)
    dtype = _numpy_dtype(int(field.datatype), bool(getattr(msg, 'is_bigendian', False)))
    height = int(getattr(msg, 'height', 0))
    width = int(getattr(msg, 'width', 0))
    if height <= 0 or width <= 0:
        return np.empty((0,), dtype=dtype)
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


def _named_field(msg: Any, name: str) -> Any:
    for field in getattr(msg, 'fields', []) or []:
        if field.name == name:
            return field
    raise ValueError(f'PointCloud2 missing field: {name}')


def _numpy_dtype(datatype: int, is_bigendian: bool) -> Any:
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
        raise ValueError(f'unsupported PointCloud2 datatype: {datatype}')
    return np.dtype(mapping[datatype])


def _stamp_sec(msg: Any) -> float:
    stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
    return float(getattr(stamp, 'sec', 0)) + float(getattr(stamp, 'nanosec', 0)) * 1e-9


def _limit_rows(points: np.ndarray, max_points: int) -> np.ndarray:
    if max_points <= 0 or len(points) <= max_points:
        return points
    step = max(1, int(math.ceil(len(points) / float(max_points))))
    return points[::step][:max_points]


def _select_sequence(payload: dict[str, Any], sequence_id: str) -> dict[str, Any]:
    for sequence in payload.get('sequences') or []:
        if sequence.get('sequence_id') == sequence_id:
            return sequence
    raise ValueError(f'sequence not found: {sequence_id}')


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.expanduser().resolve().read_text(encoding='utf-8'))


def _window_summary(scans: list[ScanPoints], cloud: np.ndarray) -> dict[str, Any]:
    return {
        'scan_count': len(scans),
        'raw_points': int(sum(len(scan.points) for scan in scans)),
        'world_points': int(len(cloud)),
        'start_stamp': min((scan.stamp for scan in scans), default=None),
        'end_stamp': max((scan.stamp for scan in scans), default=None),
    }


def _checks(
    metrics: dict[str, Any],
    options: LoopCloudAnalysisOptions,
    start_scans: list[ScanPoints],
    end_scans: list[ScanPoints],
) -> list[dict[str, str]]:
    checks = []
    checks.append({
        'id': 'start_window_scans',
        'status': 'PASS' if start_scans else 'FAIL',
        'message': f'scans={len(start_scans)}',
    })
    checks.append({
        'id': 'end_window_scans',
        'status': 'PASS' if end_scans else 'FAIL',
        'message': f'scans={len(end_scans)}',
    })
    median = float(metrics.get('symmetric_median_nn_m', math.inf))
    coverage = float(metrics.get('coverage_within_1m', 0.0))
    checks.append({
        'id': 'median_overlap',
        'status': 'PASS' if median <= float(options.pass_median_nn_m) else 'FAIL',
        'message': f'median={median:.3f}m max={options.pass_median_nn_m:.3f}m',
    })
    checks.append({
        'id': 'coverage_within_1m',
        'status': 'PASS' if coverage >= float(options.pass_coverage_within_1m) else 'FAIL',
        'message': f'coverage={coverage:.3f} min={options.pass_coverage_within_1m:.3f}',
    })
    return checks


def _status(metrics: dict[str, Any], options: LoopCloudAnalysisOptions) -> str:
    median = float(metrics.get('symmetric_median_nn_m', math.inf))
    coverage = float(metrics.get('coverage_within_1m', 0.0))
    if median <= float(options.pass_median_nn_m) and coverage >= float(options.pass_coverage_within_1m):
        return 'PASS'
    if math.isfinite(median):
        return 'WARN'
    return 'FAIL'


def _next_actions(status: str) -> list[str]:
    if status == 'PASS':
        return ['Use this loop pair as a cloud-level reference gate for SLAM loop-closure tuning.']
    return [
        'Increase --window-sec or inspect the selected candidate if the loop overlap is weak.',
        'Compare against RKO trajectory loop analysis after a SLAM run reaches the same timestamps.',
    ]


def _options_payload(options: LoopCloudAnalysisOptions) -> dict[str, Any]:
    return {
        'window_sec': options.window_sec,
        'voxel_size_m': options.voxel_size_m,
        'max_scans_per_window': options.max_scans_per_window,
        'max_points_per_scan': options.max_points_per_scan,
        'max_points_per_cloud': options.max_points_per_cloud,
        'min_range_m': options.min_range_m,
        'max_range_m': options.max_range_m,
        'pass_median_nn_m': options.pass_median_nn_m,
        'pass_coverage_within_1m': options.pass_coverage_within_1m,
    }


if __name__ == "__main__":
    raise SystemExit("mid360_robot_public_loop_cloud_analyzer is a library module; import it instead of running it directly.")
