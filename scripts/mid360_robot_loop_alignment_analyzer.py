#!/usr/bin/env python3
"""Loop-alignment cloud analyzer for MID-360 map outputs."""

from __future__ import annotations

import math
import struct
from collections import deque
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


LOOP_ALIGNMENT_JSON = 'mid360_robot_loop_alignment.json'
LOOP_ALIGNMENT_MARKDOWN = 'mid360_robot_loop_alignment.md'


@dataclass(frozen=True)
class LoopAlignmentThresholds:
    """Thresholds for loop-alignment evidence."""

    loop_search_radius_m: float = 2.0
    max_loop_distance_m: float = 1.0
    min_index_separation: int = 50
    min_time_separation_sec: float = 20.0
    min_loop_candidates: int = 1
    cloud_radius_m: float = 8.0
    voxel_size_m: float = 0.5
    min_local_points: int = 80
    max_connected_components: int = 3
    min_largest_component_ratio: float = 0.6


@dataclass(frozen=True)
class LoopAlignmentOptions:
    """Inputs and limits for loop-alignment analysis."""

    run_dir: Path
    pointcloud_map_dir: Path | None = None
    trajectory_path: Path | None = None
    output_dir: Path | None = None
    thresholds: LoopAlignmentThresholds = LoopAlignmentThresholds()
    max_points_per_tile: int = 5000
    max_total_points: int = 200000
    max_loop_candidates: int = 20


@dataclass(frozen=True)
class Pose:
    """One TUM-style trajectory pose."""

    index: int
    stamp: float
    x: float
    y: float
    z: float


def load_tum_trajectory(path: Path) -> list[Pose]:
    """Load a TUM-style trajectory from disk."""
    return _load_trajectory(path)


def parse_tum_trajectory_lines(lines: list[str]) -> list[Pose]:
    """Parse TUM-style trajectory lines."""
    return _parse_trajectory_lines(lines)


def find_loop_candidates(
    poses: list[Pose],
    thresholds: LoopAlignmentThresholds,
    *,
    max_candidates: int,
) -> list[dict[str, Any]]:
    """Find non-adjacent trajectory revisits within the loop search radius."""
    return _find_loop_candidates(poses, thresholds, max_candidates=max_candidates)


def find_nearest_revisit(
    poses: list[Pose],
    thresholds: LoopAlignmentThresholds,
) -> dict[str, Any] | None:
    """Find the nearest non-adjacent trajectory revisit, even outside the gate radius."""
    return _find_nearest_revisit(poses, thresholds)


def resolve_pointcloud_map_dir(run_dir: Path, pointcloud_map_dir: Path | None = None) -> Path:
    """Resolve a run directory to its Autoware pointcloud_map directory."""
    return _resolve_map_dir(run_dir.expanduser().resolve(), pointcloud_map_dir)


def resolve_trajectory_path(run_dir: Path, trajectory_path: Path | None = None) -> Path | None:
    """Resolve a run directory to a likely TUM trajectory path."""
    return _resolve_trajectory_path(run_dir.expanduser().resolve(), trajectory_path)


def load_pointcloud_map_points(
    map_dir: Path,
    *,
    max_points_per_tile: int,
    max_total_points: int,
) -> dict[str, Any]:
    """Load sampled XYZ points from an Autoware pointcloud_map directory."""
    return _load_cloud_points(
        map_dir.expanduser().resolve(),
        max_points_per_tile=max_points_per_tile,
        max_total_points=max_total_points,
    )


class Mid360LoopAlignmentAnalyzer:
    """Analyze trajectory loop candidates and local cloud split risk."""

    def analyze(self, options: LoopAlignmentOptions) -> dict[str, Any]:
        """Build a loop-alignment report."""
        run_dir = options.run_dir.expanduser().resolve()
        map_dir = _resolve_map_dir(run_dir, options.pointcloud_map_dir)
        trajectory_path = _resolve_trajectory_path(run_dir, options.trajectory_path)
        poses = _load_trajectory(trajectory_path)
        cloud = _load_cloud_points(
            map_dir,
            max_points_per_tile=max(1, options.max_points_per_tile),
            max_total_points=max(1, options.max_total_points),
        )
        candidates = _find_loop_candidates(
            poses,
            options.thresholds,
            max_candidates=max(1, options.max_loop_candidates),
        )
        nearest_revisit = _find_nearest_revisit(poses, options.thresholds)
        local_cloud = [
            _analyze_candidate_cloud(candidate, cloud['points'], options.thresholds)
            for candidate in candidates[: max(1, options.max_loop_candidates)]
        ]
        checks = _build_checks(
            poses=poses,
            cloud=cloud,
            candidates=candidates,
            nearest_revisit=nearest_revisit,
            local_cloud=local_cloud,
            thresholds=options.thresholds,
        )
        status = _overall_status(checks)
        output_dir = (options.output_dir.expanduser().resolve()
                      if options.output_dir else run_dir)
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'run_dir': str(run_dir),
            'pointcloud_map_dir': str(map_dir),
            'trajectory_path': str(trajectory_path) if trajectory_path else '',
            'output_dir': str(output_dir),
            'thresholds': asdict(options.thresholds),
            'trajectory': _trajectory_summary(poses),
            'cloud': {key: value for key, value in cloud.items() if key != 'points'},
            'nearest_revisit': nearest_revisit,
            'loop_candidates': candidates,
            'local_cloud_checks': local_cloud,
            'checks': checks,
            'counts': _count_checks(checks),
            'next_actions': _next_actions(checks),
        }


def write_loop_alignment_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write loop-alignment JSON and Markdown artifacts."""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / LOOP_ALIGNMENT_JSON
    markdown_path = output_dir / LOOP_ALIGNMENT_MARKDOWN
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(render_loop_alignment_markdown(report) + '\n', encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def render_loop_alignment_markdown(report: dict[str, Any]) -> str:
    """Render a concise loop-alignment report."""
    trajectory = report.get('trajectory') or {}
    cloud = report.get('cloud') or {}
    lines = [
        '# MID-360 Loop Alignment Cloud Analysis',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- run_dir: `{report.get('run_dir', '')}`",
        f"- trajectory_path: `{report.get('trajectory_path', '')}`",
        f"- pointcloud_map_dir: `{report.get('pointcloud_map_dir', '')}`",
        f"- poses: `{trajectory.get('poses', 0)}`",
        f"- path_length_m: `{_fmt(trajectory.get('path_length_m'))}`",
        f"- sampled_cloud_points: `{cloud.get('sampled_points', 0)}`",
        f"- nearest_revisit_distance_m: "
        f"`{_fmt((report.get('nearest_revisit') or {}).get('distance_m'))}`",
        f"- loop_candidates: `{len(report.get('loop_candidates') or [])}`",
        '',
        '## Checks',
        '',
    ]
    for check in report.get('checks') or []:
        lines.append(
            f"- `{check.get('status', '')}` `{check.get('id', '')}`: "
            f"{check.get('message', '')}"
        )
    lines.extend(['', '## Loop Candidates', ''])
    candidates = report.get('loop_candidates') or []
    if candidates:
        for item in candidates[:10]:
            lines.append(
                f"- distance `{_fmt(item.get('distance_m'))}` m "
                f"between pose `{item.get('start_index')}` and `{item.get('end_index')}`"
            )
    else:
        lines.append('- none')
    lines.extend(['', '## Next Actions', ''])
    actions = report.get('next_actions') or []
    if actions:
        lines.extend(f'- {action}' for action in actions)
    else:
        lines.append('- none')
    return '\n'.join(lines)


def _resolve_map_dir(run_dir: Path, pointcloud_map_dir: Path | None) -> Path:
    if pointcloud_map_dir is not None:
        return pointcloud_map_dir.expanduser().resolve()
    if (run_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml').is_file():
        return run_dir / 'pointcloud_map'
    if (run_dir / 'pointcloud_map_metadata.yaml').is_file():
        return run_dir
    return run_dir / 'pointcloud_map'


def _resolve_trajectory_path(run_dir: Path, trajectory_path: Path | None) -> Path | None:
    if trajectory_path is not None:
        return trajectory_path.expanduser().resolve()
    patterns = (
        '*tum*.txt',
        '*.tum',
        'traj_*.txt',
        'trajectory*.txt',
    )
    candidates: list[Path] = []
    for pattern in patterns:
        candidates.extend(path for path in run_dir.rglob(pattern) if path.is_file())
    if not candidates:
        return None
    return sorted(candidates, key=lambda path: (len(path.parts), str(path)))[0]


def _load_trajectory(path: Path | None) -> list[Pose]:
    if path is None or not path.is_file():
        return []
    return _parse_trajectory_lines(path.read_text(encoding='utf-8', errors='replace').splitlines())


def _parse_trajectory_lines(lines: list[str]) -> list[Pose]:
    poses = []
    for line in lines:
        parts = line.split()
        if len(parts) < 4:
            continue
        try:
            stamp = float(parts[0])
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
        except ValueError:
            continue
        poses.append(Pose(index=len(poses), stamp=stamp, x=x, y=y, z=z))
    return poses


def _load_cloud_points(
    map_dir: Path,
    *,
    max_points_per_tile: int,
    max_total_points: int,
) -> dict[str, Any]:
    metadata_path = map_dir / 'pointcloud_map_metadata.yaml'
    metadata = _load_yaml(metadata_path)
    referenced = [
        str(key) for key in metadata
        if str(key).endswith('.pcd')
    ]
    if referenced:
        pcd_paths = [map_dir / name for name in referenced]
    else:
        pcd_paths = sorted(map_dir.glob('*.pcd'))
    # Reserve a fair quota for every spatial tile. Stopping after the global
    # cap biases the sample toward metadata order (typically negative X) and
    # can leave the actual loop area with zero points.
    fair_tile_quota = max(
        1,
        min(max_points_per_tile, max_total_points // max(1, len(pcd_paths))),
    )
    points: list[tuple[float, float, float]] = []
    tile_summaries = []
    unsupported = []
    for path in pcd_paths:
        tile = _read_pcd_xyz(path, max_points=fair_tile_quota)
        tile_summaries.append({
            'name': path.name,
            'points': tile['points'],
            'sampled_points': len(tile['xyz']),
            'data': tile['data'],
            'error': tile['error'],
        })
        if tile['error']:
            unsupported.append(f'{path.name}: {tile["error"]}')
        points.extend(tile['xyz'])
    points = _sample_points(points, max_total_points)
    return {
        'map_dir': str(map_dir),
        'metadata_path': str(metadata_path) if metadata_path.is_file() else '',
        'tile_count': len(pcd_paths),
        'sampled_points': len(points),
        'unsupported_tiles': unsupported[:20],
        'tiles': tile_summaries[:50],
        'points': points,
    }


def _read_pcd_xyz(path: Path, *, max_points: int) -> dict[str, Any]:
    if not path.is_file():
        return {'points': 0, 'data': '', 'xyz': [], 'error': 'missing'}
    try:
        from lidarslam_benchmark_tools.verify_autoware_map import parse_pcd_header, read_xyz_from_pcd

        header = parse_pcd_header(str(path))
        data_kind = str(header.get('data') or '')
        if data_kind == 'binary':
            xyz = read_xyz_from_pcd(str(path), header)
        elif data_kind == 'ascii':
            xyz = _read_ascii_xyz(path, int(header.get('header_bytes') or 0))
        elif data_kind == 'binary_compressed':
            xyz = _read_binary_compressed_xyz(path, header)
        else:
            return {
                'points': int(header.get('points') or 0),
                'data': data_kind,
                'xyz': [],
                'error': f'unsupported PCD DATA {data_kind}',
            }
        return {
            'points': int(header.get('points') or len(xyz)),
            'data': data_kind,
            'xyz': _sample_points(xyz, max_points),
            'error': '',
        }
    except Exception as exc:
        return {'points': 0, 'data': '', 'xyz': [], 'error': str(exc)}


def _read_binary_compressed_xyz(
    path: Path,
    header: dict[str, Any],
) -> list[tuple[float, float, float]]:
    fields = [str(item).lower() for item in header.get('fields') or []]
    sizes = [int(item) for item in header.get('size') or []]
    types = [str(item).upper() for item in header.get('type') or []]
    counts = [int(item) for item in header.get('count') or []]
    if not counts and fields:
        counts = [1] * len(fields)
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError('invalid PCD field metadata')
    if 'x' not in fields or 'y' not in fields:
        return []

    n_points = int(header.get('points') or 0)
    if n_points <= 0:
        n_points = int(header.get('width') or 0) * int(header.get('height') or 1)
    if n_points <= 0:
        return []

    with path.open('rb') as file_obj:
        file_obj.seek(int(header.get('header_bytes') or 0))
        sizes_blob = file_obj.read(8)
        if len(sizes_blob) != 8:
            raise ValueError('missing binary_compressed size header')
        compressed_size = int.from_bytes(sizes_blob[:4], byteorder='little', signed=False)
        uncompressed_size = int.from_bytes(sizes_blob[4:], byteorder='little', signed=False)
        compressed = file_obj.read(compressed_size)
    if len(compressed) != compressed_size:
        raise ValueError('truncated binary_compressed payload')
    data = _decompress_lzf(compressed, uncompressed_size)

    components = _compressed_field_components(fields, sizes, types, counts, n_points)
    x_component = _first_component(components, 'x')
    y_component = _first_component(components, 'y')
    z_component = _first_component(components, 'z')
    if x_component is None or y_component is None:
        return []

    points = []
    for index in range(n_points):
        x = _unpack_component(data, x_component, index)
        y = _unpack_component(data, y_component, index)
        z = _unpack_component(data, z_component, index) if z_component else 0.0
        points.append((float(x), float(y), float(z)))
    return points


def _compressed_field_components(
    fields: list[str],
    sizes: list[int],
    types: list[str],
    counts: list[int],
    n_points: int,
) -> list[dict[str, Any]]:
    components = []
    offset = 0
    for field, size, data_type, count in zip(fields, sizes, types, counts):
        for component_index in range(count):
            components.append({
                'field': field,
                'component_index': component_index,
                'size': size,
                'type': data_type,
                'offset': offset,
            })
            offset += size * n_points
    return components


def _first_component(
    components: list[dict[str, Any]],
    field: str,
) -> dict[str, Any] | None:
    for component in components:
        if component['field'] == field and int(component['component_index']) == 0:
            return component
    return None


def _unpack_component(data: bytes, component: dict[str, Any], point_index: int) -> float:
    fmt_map = {
        ('F', 4): 'f',
        ('F', 8): 'd',
        ('U', 1): 'B',
        ('U', 2): 'H',
        ('U', 4): 'I',
        ('I', 1): 'b',
        ('I', 2): 'h',
        ('I', 4): 'i',
    }
    size = int(component['size'])
    data_type = str(component['type']).upper()
    fmt = fmt_map.get((data_type, size))
    if fmt is None:
        raise ValueError(f'unsupported PCD component type {data_type}{size}')
    offset = int(component['offset']) + point_index * size
    return struct.unpack_from('<' + fmt, data, offset)[0]


def _decompress_lzf(compressed: bytes, expected_size: int) -> bytes:
    try:
        import lzf  # type: ignore[import-not-found]

        data = lzf.decompress(compressed, expected_size)
        if data is not None and len(data) == expected_size:
            return data
    except Exception:
        pass
    return _decompress_lzf_pure_python(compressed, expected_size)


def _decompress_lzf_pure_python(compressed: bytes, expected_size: int) -> bytes:
    output = bytearray()
    index = 0
    while index < len(compressed):
        control = compressed[index]
        index += 1
        if control < 32:
            length = control + 1
            if index + length > len(compressed):
                raise ValueError('truncated LZF literal run')
            output.extend(compressed[index:index + length])
            index += length
        else:
            length = control >> 5
            reference_offset = (control & 0x1F) << 8
            if length == 7:
                if index >= len(compressed):
                    raise ValueError('truncated LZF extended length')
                length += compressed[index]
                index += 1
            if index >= len(compressed):
                raise ValueError('truncated LZF back-reference')
            reference_offset += compressed[index]
            index += 1
            length += 2
            reference = len(output) - reference_offset - 1
            if reference < 0:
                raise ValueError('invalid LZF back-reference')
            for _ in range(length):
                output.append(output[reference])
                reference += 1
        if len(output) > expected_size:
            raise ValueError('LZF payload expanded beyond expected size')
    if len(output) != expected_size:
        raise ValueError(
            f'LZF payload expanded to {len(output)} bytes, expected {expected_size}'
        )
    return bytes(output)


def _read_ascii_xyz(path: Path, header_bytes: int) -> list[tuple[float, float, float]]:
    text = path.read_bytes()[header_bytes:].decode('ascii', errors='replace')
    points = []
    for line in text.splitlines():
        parts = line.split()
        if len(parts) < 2:
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2]) if len(parts) >= 3 else 0.0
        except ValueError:
            continue
        points.append((x, y, z))
    return points


def _sample_points(
    points: list[tuple[float, float, float]],
    max_points: int,
) -> list[tuple[float, float, float]]:
    if len(points) <= max_points:
        return points
    stride = max(1, math.ceil(len(points) / max_points))
    return points[::stride][:max_points]


def _find_loop_candidates(
    poses: list[Pose],
    thresholds: LoopAlignmentThresholds,
    *,
    max_candidates: int,
) -> list[dict[str, Any]]:
    candidates = []
    for i, start in enumerate(poses):
        for end in poses[i + thresholds.min_index_separation:]:
            if abs(end.stamp - start.stamp) < thresholds.min_time_separation_sec:
                continue
            distance = _distance_pose(start, end)
            if distance <= thresholds.loop_search_radius_m:
                candidates.append({
                    'start_index': start.index,
                    'end_index': end.index,
                    'start_stamp': start.stamp,
                    'end_stamp': end.stamp,
                    'distance_m': distance,
                    'midpoint': [
                        (start.x + end.x) * 0.5,
                        (start.y + end.y) * 0.5,
                        (start.z + end.z) * 0.5,
                    ],
                })
    candidates.sort(key=lambda item: float(item['distance_m']))
    return candidates[:max_candidates]


def _find_nearest_revisit(
    poses: list[Pose],
    thresholds: LoopAlignmentThresholds,
) -> dict[str, Any] | None:
    nearest: dict[str, Any] | None = None
    nearest_distance = math.inf
    for i, start in enumerate(poses):
        for end in poses[i + thresholds.min_index_separation:]:
            if abs(end.stamp - start.stamp) < thresholds.min_time_separation_sec:
                continue
            distance = _distance_pose(start, end)
            if distance >= nearest_distance:
                continue
            nearest_distance = distance
            nearest = {
                'start_index': start.index,
                'end_index': end.index,
                'start_stamp': start.stamp,
                'end_stamp': end.stamp,
                'distance_m': distance,
                'midpoint': [
                    (start.x + end.x) * 0.5,
                    (start.y + end.y) * 0.5,
                    (start.z + end.z) * 0.5,
                ],
            }
    return nearest


def _analyze_candidate_cloud(
    candidate: dict[str, Any],
    points: list[tuple[float, float, float]],
    thresholds: LoopAlignmentThresholds,
) -> dict[str, Any]:
    cx, cy, _ = candidate['midpoint']
    radius = thresholds.cloud_radius_m
    radius2 = radius * radius
    local = [
        point for point in points
        if (point[0] - cx) ** 2 + (point[1] - cy) ** 2 <= radius2
    ]
    grid = _occupied_grid(local, thresholds.voxel_size_m)
    components = _connected_components(grid)
    largest = max(components, default=0)
    ratio = largest / len(grid) if grid else 0.0
    status = 'PASS'
    messages = []
    if len(local) < thresholds.min_local_points:
        status = 'WARN'
        messages.append(f'local points {len(local)} < {thresholds.min_local_points}')
    if len(components) > thresholds.max_connected_components:
        status = 'FAIL'
        messages.append(
            f'connected components {len(components)} > {thresholds.max_connected_components}'
        )
    if grid and ratio < thresholds.min_largest_component_ratio:
        status = 'FAIL'
        messages.append(
            f'largest component ratio {ratio:.3f} < {thresholds.min_largest_component_ratio}'
        )
    return {
        'candidate': candidate,
        'status': status,
        'message': '; '.join(messages) if messages else 'local loop cloud is connected enough',
        'local_points': len(local),
        'occupied_voxels': len(grid),
        'connected_components': len(components),
        'largest_component_voxels': largest,
        'largest_component_ratio': ratio,
    }


def _occupied_grid(points: list[tuple[float, float, float]], voxel: float) -> set[tuple[int, int]]:
    if voxel <= 0.0:
        voxel = 0.5
    return {
        (math.floor(point[0] / voxel), math.floor(point[1] / voxel))
        for point in points
    }


def _connected_components(grid: set[tuple[int, int]]) -> list[int]:
    remaining = set(grid)
    components = []
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
    while remaining:
        start = remaining.pop()
        queue = deque([start])
        count = 1
        while queue:
            x, y = queue.popleft()
            for dx, dy in neighbors:
                nxt = (x + dx, y + dy)
                if nxt in remaining:
                    remaining.remove(nxt)
                    queue.append(nxt)
                    count += 1
        components.append(count)
    return sorted(components, reverse=True)


def _build_checks(
    *,
    poses: list[Pose],
    cloud: dict[str, Any],
    candidates: list[dict[str, Any]],
    nearest_revisit: dict[str, Any] | None,
    local_cloud: list[dict[str, Any]],
    thresholds: LoopAlignmentThresholds,
) -> list[dict[str, str]]:
    nearest_msg = ''
    if nearest_revisit:
        nearest_msg = f" nearest={float(nearest_revisit['distance_m']):.3f}m"
    checks = [
        _check('trajectory_present', bool(poses), f'poses={len(poses)}'),
        _check('cloud_points_present', int(cloud.get('sampled_points') or 0) > 0,
               f"sampled_points={cloud.get('sampled_points', 0)}"),
        _check(
            'loop_candidates_present',
            len(candidates) >= thresholds.min_loop_candidates,
            f'candidates={len(candidates)} min={thresholds.min_loop_candidates}'
            f' radius={thresholds.loop_search_radius_m}m{nearest_msg}',
        ),
    ]
    if candidates:
        max_distance = max(float(item['distance_m']) for item in candidates)
        checks.append(_check(
            'loop_distance_gate',
            max_distance <= thresholds.max_loop_distance_m,
            f'max_loop_distance_m={max_distance:.3f} threshold={thresholds.max_loop_distance_m}',
        ))
    for index, item in enumerate(local_cloud[:5]):
        checks.append(_check(
            f'local_cloud_connected_{index}',
            item.get('status') != 'FAIL',
            item.get('message', ''),
            warn=item.get('status') == 'WARN',
        ))
    return checks


def _check(check_id: str, passed: bool, message: str, *, warn: bool = False) -> dict[str, str]:
    if passed:
        status = 'WARN' if warn else 'PASS'
    else:
        status = 'FAIL'
    return {'id': check_id, 'status': status, 'message': message}


def _overall_status(checks: list[dict[str, str]]) -> str:
    if any(check['status'] == 'FAIL' for check in checks):
        return 'FAIL'
    if any(check['status'] == 'WARN' for check in checks):
        return 'WARN'
    return 'PASS'


def _count_checks(checks: list[dict[str, str]]) -> dict[str, int]:
    return {
        'pass': sum(1 for check in checks if check['status'] == 'PASS'),
        'warn': sum(1 for check in checks if check['status'] == 'WARN'),
        'fail': sum(1 for check in checks if check['status'] == 'FAIL'),
    }


def _next_actions(checks: list[dict[str, str]]) -> list[str]:
    failed = {check['id'] for check in checks if check['status'] == 'FAIL'}
    actions = []
    if 'trajectory_present' in failed:
        actions.append('Provide a TUM trajectory from the map run with --trajectory.')
    if 'cloud_points_present' in failed:
        actions.append('Provide readable ASCII or binary PCD tiles under pointcloud_map/.')
    if 'loop_candidates_present' in failed:
        actions.append('Use a longer route with a real revisit, or raise --loop-search-radius-m for diagnosis.')
    if 'loop_distance_gate' in failed:
        actions.append('Inspect loop closure around the reported candidate poses; trajectory revisit distance is high.')
    if any(item.startswith('local_cloud_connected') for item in failed):
        actions.append('Inspect the local loop cloud in CloudCompare/Foxglove; the map may contain split components.')
    return actions[:6]


def _trajectory_summary(poses: list[Pose]) -> dict[str, Any]:
    path_length = 0.0
    for prev, cur in zip(poses, poses[1:]):
        path_length += _distance_pose(prev, cur)
    return {
        'poses': len(poses),
        'duration_sec': poses[-1].stamp - poses[0].stamp if len(poses) >= 2 else 0.0,
        'path_length_m': path_length,
        'bounds': _pose_bounds(poses),
    }


def _pose_bounds(poses: list[Pose]) -> dict[str, float]:
    if not poses:
        return {}
    return {
        'min_x': min(pose.x for pose in poses),
        'max_x': max(pose.x for pose in poses),
        'min_y': min(pose.y for pose in poses),
        'max_y': max(pose.y for pose in poses),
        'min_z': min(pose.z for pose in poses),
        'max_z': max(pose.z for pose in poses),
    }


def _distance_pose(a: Pose, b: Pose) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _fmt(value: Any) -> str:
    try:
        return f'{float(value):.3f}'
    except Exception:
        return ''


if __name__ == "__main__":
    raise SystemExit("mid360_robot_loop_alignment_analyzer is a library module; import it instead of running it directly.")
