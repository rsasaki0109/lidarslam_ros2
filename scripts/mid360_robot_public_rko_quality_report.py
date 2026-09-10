#!/usr/bin/env python3
"""Quality reports for public MID-360 RKO-LIO sweep outputs."""

from __future__ import annotations

import html
import json
import math
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


RKO_QUALITY_JSON = 'mid360_robot_public_rko_quality_report.json'
RKO_QUALITY_MARKDOWN = 'mid360_robot_public_rko_quality_report.md'
RKO_QUALITY_HTML = 'mid360_robot_public_rko_quality_report.html'


@dataclass(frozen=True)
class RkoQualityGateThresholds:
    """Minimum acceptance thresholds for a real-data MID-360 RKO-LIO case."""

    min_trajectory_poses: int = 200
    min_trajectory_duration_sec: float = 60.0
    min_path_length_m: float = 10.0
    max_step_m: float = 5.0
    min_map_points: int = 100000
    min_map_tiles: int = 10
    max_runtime_sec: float = 120.0


class RkoQualityReportBuilder:
    """Build a case-by-case quality report from a public RKO sweep manifest."""

    def __init__(
        self,
        sweep_path: Path,
        thresholds: RkoQualityGateThresholds | None = None,
    ) -> None:
        self._sweep_path = sweep_path.expanduser().resolve()
        self._thresholds = thresholds or RkoQualityGateThresholds()

    def build_report(self) -> dict[str, Any]:
        """Build the quality report payload."""
        manifest = _load_json(self._sweep_path)
        cases = [_build_case_quality(row) for row in manifest.get('diagnostics') or []]
        for row in cases:
            row['quality_gate'] = _quality_gate(row, self._thresholds)
        ranked = _rank_cases(cases)
        rank_by_case = {
            str(row.get('case_id') or ''): index + 1 for index, row in enumerate(ranked)
        }
        for row in cases:
            row['rank'] = rank_by_case.get(str(row.get('case_id') or ''), 0)
        best = ranked[0] if ranked else {}
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': _overall_status(cases),
            'sweep_path': str(self._sweep_path),
            'sweep_status': manifest.get('status', ''),
            'bag_path': manifest.get('bag_path', ''),
            'output_dir': manifest.get('output_dir', ''),
            'gate_thresholds': asdict(self._thresholds),
            'counts': _counts(cases),
            'best_case': _best_case_summary(best),
            'cases': sorted(cases, key=lambda item: int(item.get('rank') or 9999)),
        }


def write_rko_quality_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write JSON, Markdown, and HTML quality reports."""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / RKO_QUALITY_JSON
    markdown_path = output_dir / RKO_QUALITY_MARKDOWN
    html_path = output_dir / RKO_QUALITY_HTML
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(render_rko_quality_markdown(report) + '\n', encoding='utf-8')
    html_path.write_text(render_rko_quality_html(report), encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path, 'html': html_path}


def render_rko_quality_markdown(report: dict[str, Any]) -> str:
    """Render the quality report as Markdown."""
    counts = report.get('counts') or {}
    best = report.get('best_case') or {}
    lines = [
        '# MID-360 Public RKO-LIO Quality Report',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- created_at: `{report.get('created_at', '')}`",
        f"- sweep_path: `{report.get('sweep_path', '')}`",
        f"- bag_path: `{report.get('bag_path', '')}`",
        f"- cases: `{counts.get('cases', 0)}`",
        f"- map_verified: `{counts.get('map_verified', 0)}`",
        f"- gate_pass: `{counts.get('gate_pass', 0)}`",
        f"- total_map_points: `{counts.get('total_map_points', 0)}`",
        f"- best_case: `{best.get('case_id', '')}`",
        '',
        '## Case Quality',
        '',
    ]
    cases = report.get('cases') or []
    if not cases:
        lines.append('- none')
        return '\n'.join(lines)

    lines.extend([
        (
            '| Rank | Case | Status | Verify | Voxel | Min Range | Runtime s | '
            'Trajectory Poses | Duration s | Path m | Max Step m | Map Points | Tiles | '
            'Point Density | Gate | Score |'
        ),
        (
            '| ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | '
            '---: | ---: | ---: | --- | ---: |'
        ),
    ])
    for row in cases:
        params = row.get('parameters') or {}
        runtime = row.get('runtime') or {}
        trajectory = row.get('trajectory') or {}
        map_quality = row.get('map_quality') or {}
        verification = row.get('verification') or {}
        gate = row.get('quality_gate') or {}
        lines.append(
            '| '
            + ' | '.join([
                str(row.get('rank', '')),
                f"`{row.get('case_id', '')}`",
                f"`{row.get('status', '')}`",
                f"`{verification.get('result', '')}`",
                _fmt_float(params.get('voxel_size')),
                _fmt_float(params.get('min_range')),
                _fmt_float(runtime.get('duration_sec')),
                _fmt_int(trajectory.get('poses')),
                _fmt_float(trajectory.get('duration_sec')),
                _fmt_float(trajectory.get('path_length_m')),
                _fmt_float(trajectory.get('max_step_m')),
                _fmt_int(map_quality.get('point_count')),
                _fmt_int(map_quality.get('tile_count')),
                _fmt_float(map_quality.get('point_density_per_m2')),
                f"`{gate.get('status', '')}`",
                _fmt_int(row.get('quality_score')),
            ])
            + ' |'
        )

    lines.extend(['', '## Best Case', ''])
    if best:
        lines.extend([
            f"- case: `{best.get('case_id', '')}`",
            f"- status: `{best.get('status', '')}`",
            f"- gate: `{best.get('gate_status', '')}`",
            f"- score: `{best.get('quality_score', 0)}`",
            f"- output_dir: `{best.get('output_dir', '')}`",
        ])
    else:
        lines.append('- none')
    return '\n'.join(lines)


def render_rko_quality_html(report: dict[str, Any]) -> str:
    """Render the quality report as self-contained HTML."""
    rows = report.get('cases') or []
    status = str(report.get('status') or 'UNKNOWN').upper()
    return '\n'.join([
        '<!doctype html>',
        '<html lang="en">',
        '<head>',
        '<meta charset="utf-8">',
        '<meta name="viewport" content="width=device-width, initial-scale=1">',
        '<title>MID-360 Public RKO-LIO Quality Report</title>',
        '<style>',
        _css(),
        '</style>',
        '</head>',
        '<body>',
        '<main>',
        '<section class="hero">',
        '<div>',
        '<p class="eyebrow">MID-360 Public RKO-LIO</p>',
        '<h1>Quality Report</h1>',
        f'<p class="subtle">{_h(report.get("sweep_path", ""))}</p>',
        '</div>',
        f'<div class="status {status.lower()}">{_h(status)}</div>',
        '</section>',
        _metric_cards(report),
        _best_panel(report),
        _case_table(rows),
        '</main>',
        '</body>',
        '</html>',
    ])


def _build_case_quality(row: dict[str, Any]) -> dict[str, Any]:
    output_dir = Path(str(row.get('output_dir') or '')).expanduser().resolve()
    trajectory = _trajectory_quality(row, output_dir)
    map_quality = _map_quality(output_dir / 'pointcloud_map')
    runtime = row.get('run_result') or {}
    case = {
        'case_id': row.get('case_id', ''),
        'label': row.get('label', ''),
        'status': row.get('status', ''),
        'parameters': row.get('parameters') or {},
        'output_dir': str(output_dir),
        'runtime': {
            'duration_sec': _maybe_float(runtime.get('duration_sec')),
            'returncode': runtime.get('returncode'),
            'timed_out': bool(runtime.get('timed_out')),
            'offline_completed': bool((row.get('runtime') or {}).get('offline_completed')),
            'keypoint_drop_count': int(
                (row.get('runtime') or {}).get('keypoints_too_few_count') or 0
            ),
            'lidar_delta_error_count': int(
                (row.get('runtime') or {}).get('lidar_delta_error_count') or 0
            ),
            'buffer_throttle_count': int(
                (row.get('runtime') or {}).get('buffer_throttle_count') or 0
            ),
        },
        'outputs': row.get('outputs') or {},
        'verification': row.get('verification') or {},
        'trajectory': trajectory,
        'map_quality': map_quality,
    }
    case['quality_score'] = _quality_score(case)
    return case


def _trajectory_quality(row: dict[str, Any], output_dir: Path) -> dict[str, Any]:
    files = _trajectory_paths(row, output_dir)
    summaries = [_trajectory_file_quality(path) for path in files]
    valid = [item for item in summaries if int(item.get('poses') or 0) > 0]
    bounds = _merge_bounds([item.get('bounds') or {} for item in valid])
    return {
        'file_count': len(files),
        'poses': sum(int(item.get('poses') or 0) for item in summaries),
        'invalid_lines': sum(int(item.get('invalid_lines') or 0) for item in summaries),
        'duration_sec': sum(float(item.get('duration_sec') or 0.0) for item in valid),
        'path_length_m': sum(float(item.get('path_length_m') or 0.0) for item in valid),
        'max_step_m': max((float(item.get('max_step_m') or 0.0) for item in valid), default=0.0),
        'bounds': bounds,
        'files': summaries,
    }


def _trajectory_paths(row: dict[str, Any], output_dir: Path) -> list[Path]:
    trajectory = (row.get('outputs') or {}).get('trajectory') or {}
    paths: list[Path] = []
    for item in trajectory.get('files') or []:
        value = item.get('path') if isinstance(item, dict) else item
        if value:
            paths.append(Path(str(value)).expanduser().resolve())
    if paths:
        return sorted(path for path in paths if path.is_file())
    if not output_dir.is_dir():
        return []
    patterns = ('*_tum_*.txt', 'traj_*.tum', '*.tum')
    found: set[Path] = set()
    for pattern in patterns:
        found.update(path.resolve() for path in output_dir.rglob(pattern) if path.is_file())
    return sorted(found)


def _trajectory_file_quality(path: Path) -> dict[str, Any]:
    poses: list[tuple[float, float, float, float]] = []
    invalid = 0
    for line in _read_text(path).splitlines():
        text = line.strip()
        if not text:
            continue
        parts = text.split()
        if len(parts) < 4:
            invalid += 1
            continue
        try:
            poses.append((float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3])))
        except ValueError:
            invalid += 1
    path_length = 0.0
    max_step = 0.0
    for prev, cur in zip(poses, poses[1:]):
        step = math.sqrt(
            (cur[1] - prev[1]) ** 2
            + (cur[2] - prev[2]) ** 2
            + (cur[3] - prev[3]) ** 2
        )
        path_length += step
        max_step = max(max_step, step)
    timestamps = [item[0] for item in poses]
    return {
        'path': str(path),
        'poses': len(poses),
        'invalid_lines': invalid,
        'duration_sec': max(timestamps) - min(timestamps) if len(timestamps) >= 2 else 0.0,
        'path_length_m': path_length,
        'max_step_m': max_step,
        'bounds': _pose_bounds(poses),
    }


def _map_quality(pointcloud_map_dir: Path) -> dict[str, Any]:
    metadata_path = pointcloud_map_dir / 'pointcloud_map_metadata.yaml'
    metadata = _load_yaml(metadata_path)
    x_res = _maybe_float(metadata.get('x_resolution'))
    y_res = _maybe_float(metadata.get('y_resolution'))
    tiles = _metadata_tiles(metadata)
    pcd_files = sorted(path for path in pointcloud_map_dir.glob('*.pcd') if path.is_file())
    referenced_names = {str(name) for name in tiles}
    tile_stats = [_pcd_stats(pointcloud_map_dir / name) for name in sorted(referenced_names)]
    existing_referenced = [item for item in tile_stats if item.get('exists')]
    point_count = sum(int(item.get('points') or 0) for item in existing_referenced)
    missing = [item.get('name') for item in tile_stats if not item.get('exists')]
    orphans = [path.name for path in pcd_files if path.name not in referenced_names]
    tile_bounds = _tile_bounds(tiles, x_res, y_res)
    area = _bounds_area(tile_bounds)
    return {
        'map_dir': str(pointcloud_map_dir) if pointcloud_map_dir.is_dir() else '',
        'metadata_path': str(metadata_path) if metadata_path.is_file() else '',
        'x_resolution': x_res,
        'y_resolution': y_res,
        'tile_count': len(tiles),
        'pcd_file_count': len(pcd_files),
        'referenced_pcd_count': len(referenced_names),
        'missing_referenced_pcd_count': len(missing),
        'orphan_pcd_count': len(orphans),
        'empty_tile_count': sum(
            1 for item in existing_referenced if int(item.get('points') or 0) == 0
        ),
        'point_count': point_count,
        'total_bytes': sum(int(item.get('size_bytes') or 0) for item in existing_referenced),
        'min_points_per_tile': min(
            (int(item.get('points') or 0) for item in existing_referenced), default=0,
        ),
        'max_points_per_tile': max(
            (int(item.get('points') or 0) for item in existing_referenced), default=0,
        ),
        'mean_points_per_tile': (
            point_count / len(existing_referenced) if existing_referenced else 0.0
        ),
        'tile_bounds': tile_bounds,
        'tile_area_m2': area,
        'point_density_per_m2': point_count / area if area > 0.0 else 0.0,
        'missing_referenced_pcd': missing[:20],
        'orphan_pcd': orphans[:20],
    }


def _metadata_tiles(metadata: dict[str, Any]) -> dict[str, tuple[float, float]]:
    tiles: dict[str, tuple[float, float]] = {}
    for key, value in metadata.items():
        if key in ('x_resolution', 'y_resolution'):
            continue
        if not str(key).endswith('.pcd'):
            continue
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            continue
        x = _maybe_float(value[0])
        y = _maybe_float(value[1])
        if x is None or y is None:
            continue
        tiles[str(key)] = (x, y)
    return tiles


def _pcd_stats(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {'name': path.name, 'exists': False, 'points': 0, 'size_bytes': 0, 'error': ''}
    try:
        from lidarslam_benchmark_tools.verify_autoware_map import parse_pcd_header

        header = parse_pcd_header(str(path))
        return {
            'name': path.name,
            'exists': True,
            'points': int(header.get('points') or 0),
            'size_bytes': path.stat().st_size,
            'data': header.get('data', ''),
            'error': '',
        }
    except Exception as exc:
        return {
            'name': path.name,
            'exists': True,
            'points': 0,
            'size_bytes': path.stat().st_size if path.is_file() else 0,
            'data': '',
            'error': str(exc),
        }


def _quality_score(case: dict[str, Any]) -> int:
    status = str(case.get('status') or '')
    verification = case.get('verification') or {}
    runtime = case.get('runtime') or {}
    trajectory = case.get('trajectory') or {}
    map_quality = case.get('map_quality') or {}
    score = 0.0
    if status == 'MAP_VERIFIED':
        score += 30
    elif status == 'MAP_SAVED':
        score += 20
    if verification.get('result') == 'PASS':
        score += 20
    if runtime.get('offline_completed'):
        score += 10
    if (
        int(runtime.get('keypoint_drop_count') or 0) == 0
        and int(runtime.get('lidar_delta_error_count') or 0) == 0
    ):
        score += 10
    score += min(10.0, float(trajectory.get('path_length_m') or 0.0) / 10.0)
    point_count = int(map_quality.get('point_count') or 0)
    if point_count > 0:
        score += min(10.0, math.log10(point_count + 1.0) * 2.0)
    score += min(5.0, float(map_quality.get('point_density_per_m2') or 0.0))
    duration = _maybe_float(runtime.get('duration_sec'))
    if duration is not None:
        score += max(0.0, min(5.0, (25.0 - duration) / 3.0))
    score -= min(12, int(runtime.get('keypoint_drop_count') or 0) * 2)
    score -= min(12, int(runtime.get('lidar_delta_error_count') or 0) * 2)
    if runtime.get('timed_out'):
        score -= 20
    if int(map_quality.get('missing_referenced_pcd_count') or 0) > 0:
        score -= 15
    if int(map_quality.get('empty_tile_count') or 0) > 0:
        score -= 5
    return int(round(max(0.0, min(100.0, score))))


def _quality_gate(
    case: dict[str, Any],
    thresholds: RkoQualityGateThresholds,
) -> dict[str, Any]:
    runtime = case.get('runtime') or {}
    trajectory = case.get('trajectory') or {}
    map_quality = case.get('map_quality') or {}
    verification = case.get('verification') or {}
    checks = [
        _gate_check(
            'map_verified',
            str(case.get('status') or '') == 'MAP_VERIFIED'
            and verification.get('result') == 'PASS',
            f"status={case.get('status', '')} verify={verification.get('result', '')}",
        ),
        _gate_check(
            'offline_completed',
            bool(runtime.get('offline_completed')),
            'RKO-LIO offline completion marker was observed.',
        ),
        _gate_check(
            'no_runtime_timeout',
            not bool(runtime.get('timed_out')),
            f"timed_out={runtime.get('timed_out')}",
        ),
        _gate_check(
            'no_keypoint_or_lidar_delta_errors',
            int(runtime.get('keypoint_drop_count') or 0) == 0
            and int(runtime.get('lidar_delta_error_count') or 0) == 0,
            (
                f"keypoint_drop_count={runtime.get('keypoint_drop_count', 0)} "
                f"lidar_delta_error_count={runtime.get('lidar_delta_error_count', 0)}"
            ),
        ),
        _gate_check(
            'trajectory_poses',
            int(trajectory.get('poses') or 0) >= thresholds.min_trajectory_poses,
            f"poses={trajectory.get('poses', 0)} min={thresholds.min_trajectory_poses}",
        ),
        _gate_check(
            'trajectory_duration',
            float(trajectory.get('duration_sec') or 0.0)
            >= thresholds.min_trajectory_duration_sec,
            (
                f"duration_sec={_fmt_float(trajectory.get('duration_sec'))} "
                f"min={thresholds.min_trajectory_duration_sec}"
            ),
        ),
        _gate_check(
            'trajectory_path_length',
            float(trajectory.get('path_length_m') or 0.0) >= thresholds.min_path_length_m,
            (
                f"path_length_m={_fmt_float(trajectory.get('path_length_m'))} "
                f"min={thresholds.min_path_length_m}"
            ),
        ),
        _gate_check(
            'trajectory_max_step',
            float(trajectory.get('max_step_m') or 0.0) <= thresholds.max_step_m,
            f"max_step_m={_fmt_float(trajectory.get('max_step_m'))} max={thresholds.max_step_m}",
        ),
        _gate_check(
            'map_points',
            int(map_quality.get('point_count') or 0) >= thresholds.min_map_points,
            f"point_count={map_quality.get('point_count', 0)} min={thresholds.min_map_points}",
        ),
        _gate_check(
            'map_tiles',
            int(map_quality.get('tile_count') or 0) >= thresholds.min_map_tiles,
            f"tile_count={map_quality.get('tile_count', 0)} min={thresholds.min_map_tiles}",
        ),
        _gate_check(
            'map_integrity',
            int(map_quality.get('missing_referenced_pcd_count') or 0) == 0
            and int(map_quality.get('orphan_pcd_count') or 0) == 0
            and int(map_quality.get('empty_tile_count') or 0) == 0,
            (
                f"missing={map_quality.get('missing_referenced_pcd_count', 0)} "
                f"orphan={map_quality.get('orphan_pcd_count', 0)} "
                f"empty={map_quality.get('empty_tile_count', 0)}"
            ),
        ),
        _gate_check(
            'runtime_budget',
            _runtime_within_budget(runtime.get('duration_sec'), thresholds.max_runtime_sec),
            (
                f"duration_sec={_fmt_float(runtime.get('duration_sec'))} "
                f"max={thresholds.max_runtime_sec}"
            ),
        ),
    ]
    failures = [item for item in checks if item['status'] == 'FAIL']
    return {
        'status': 'PASS' if not failures else 'FAIL',
        'fail_count': len(failures),
        'checks': checks,
    }


def _gate_check(check_id: str, passed: bool, message: str) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'message': message,
    }


def _runtime_within_budget(value: Any, max_runtime_sec: float) -> bool:
    duration = _maybe_float(value)
    if duration is None:
        return False
    return duration <= max_runtime_sec


def _rank_cases(cases: list[dict[str, Any]]) -> list[dict[str, Any]]:
    def key(row: dict[str, Any]) -> tuple[int, int, int, float, float, str]:
        trajectory = row.get('trajectory') or {}
        runtime = row.get('runtime') or {}
        gate = row.get('quality_gate') or {}
        return (
            0 if gate.get('status') == 'PASS' else 1,
            -int(row.get('quality_score') or 0),
            -int(trajectory.get('poses') or 0),
            -float(trajectory.get('path_length_m') or 0.0),
            float(runtime.get('duration_sec') or 999999.0),
            str(row.get('case_id') or ''),
        )

    return sorted(cases, key=key)


def _overall_status(cases: list[dict[str, Any]]) -> str:
    if not cases:
        return 'EMPTY'
    if any((row.get('quality_gate') or {}).get('status') == 'PASS' for row in cases):
        return 'PASS'
    if any((row.get('map_quality') or {}).get('point_count') for row in cases):
        return 'WARN'
    return 'INCOMPLETE'


def _counts(cases: list[dict[str, Any]]) -> dict[str, Any]:
    return {
        'cases': len(cases),
        'map_verified': sum(
            1 for row in cases if str(row.get('status') or '') == 'MAP_VERIFIED'
        ),
        'gate_pass': sum(
            1 for row in cases if (row.get('quality_gate') or {}).get('status') == 'PASS'
        ),
        'map_saved': sum(
            1 for row in cases if (row.get('outputs') or {}).get('map_saved')
        ),
        'trajectories': sum(
            1 for row in cases
            if int((row.get('trajectory') or {}).get('poses') or 0) > 0
        ),
        'total_map_points': sum(
            int((row.get('map_quality') or {}).get('point_count') or 0) for row in cases
        ),
        'total_tiles': sum(
            int((row.get('map_quality') or {}).get('tile_count') or 0) for row in cases
        ),
    }


def _best_case_summary(row: dict[str, Any]) -> dict[str, Any]:
    if not row:
        return {}
    return {
        'case_id': row.get('case_id', ''),
        'status': row.get('status', ''),
        'gate_status': (row.get('quality_gate') or {}).get('status', ''),
        'quality_score': row.get('quality_score', 0),
        'output_dir': row.get('output_dir', ''),
    }


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        payload = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _read_text(path: Path) -> str:
    if not path.is_file():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _pose_bounds(poses: list[tuple[float, float, float, float]]) -> dict[str, float]:
    if not poses:
        return {}
    xs = [item[1] for item in poses]
    ys = [item[2] for item in poses]
    zs = [item[3] for item in poses]
    return {
        'min_x': min(xs),
        'max_x': max(xs),
        'min_y': min(ys),
        'max_y': max(ys),
        'min_z': min(zs),
        'max_z': max(zs),
    }


def _merge_bounds(bounds_list: list[dict[str, Any]]) -> dict[str, float]:
    valid = [item for item in bounds_list if item]
    if not valid:
        return {}
    keys = ('min_x', 'min_y', 'min_z')
    max_keys = ('max_x', 'max_y', 'max_z')
    result: dict[str, float] = {}
    for key in keys:
        result[key] = min(float(item[key]) for item in valid if key in item)
    for key in max_keys:
        result[key] = max(float(item[key]) for item in valid if key in item)
    return result


def _tile_bounds(
    tiles: dict[str, tuple[float, float]],
    x_res: float | None,
    y_res: float | None,
) -> dict[str, float]:
    if not tiles or x_res is None or y_res is None:
        return {}
    xs = [coord[0] for coord in tiles.values()]
    ys = [coord[1] for coord in tiles.values()]
    return {
        'min_x': min(xs),
        'max_x': max(xs) + x_res,
        'min_y': min(ys),
        'max_y': max(ys) + y_res,
    }


def _bounds_area(bounds: dict[str, Any]) -> float:
    if not bounds:
        return 0.0
    width = float(bounds.get('max_x') or 0.0) - float(bounds.get('min_x') or 0.0)
    height = float(bounds.get('max_y') or 0.0) - float(bounds.get('min_y') or 0.0)
    return max(0.0, width) * max(0.0, height)


def _maybe_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def _fmt_float(value: Any) -> str:
    number = _maybe_float(value)
    return '' if number is None else f'{number:.2f}'


def _fmt_int(value: Any) -> str:
    if value is None:
        return ''
    try:
        return str(int(value))
    except Exception:
        return ''


def _metric_cards(report: dict[str, Any]) -> str:
    counts = report.get('counts') or {}
    cards = [
        _metric_card('Cases', counts.get('cases', 0)),
        _metric_card('Map Verified', counts.get('map_verified', 0)),
        _metric_card('Gate Pass', counts.get('gate_pass', 0)),
        _metric_card('Map Points', counts.get('total_map_points', 0)),
    ]
    return '<section class="metrics">' + ''.join(cards) + '</section>'


def _metric_card(label: str, value: Any) -> str:
    return f'<article class="metric"><h2>{_h(label)}</h2><p>{_h(value)}</p></article>'


def _best_panel(report: dict[str, Any]) -> str:
    best = report.get('best_case') or {}
    if not best:
        return '<section><p class="empty">No sweep cases found.</p></section>'
    return (
        '<section class="best">'
        '<h2>Best Case</h2>'
        f'<p><strong>{_h(best.get("case_id", ""))}</strong> '
        f'<span class="pill {_h(str(best.get("status", "")).lower())}">'
        f'{_h(best.get("status", ""))}</span></p>'
        f'<p>Gate: <strong>{_h(best.get("gate_status", ""))}</strong> '
        f'Score: <strong>{_h(best.get("quality_score", 0))}</strong></p>'
        f'<p class="subtle">{_h(best.get("output_dir", ""))}</p>'
        '</section>'
    )


def _case_table(rows: list[dict[str, Any]]) -> str:
    if not rows:
        return ''
    body = []
    for row in rows:
        params = row.get('parameters') or {}
        runtime = row.get('runtime') or {}
        trajectory = row.get('trajectory') or {}
        map_quality = row.get('map_quality') or {}
        verification = row.get('verification') or {}
        gate = row.get('quality_gate') or {}
        status = str(row.get('status') or '').upper()
        gate_status = str(gate.get('status') or '').upper()
        body.append(
            '<tr>'
            f'<td>{_h(row.get("rank", ""))}</td>'
            f'<td><code>{_h(row.get("case_id", ""))}</code></td>'
            f'<td><span class="pill {status.lower()}">{_h(status)}</span></td>'
            f'<td>{_h(verification.get("result", ""))}</td>'
            f'<td>{_h(_fmt_float(params.get("voxel_size")))}</td>'
            f'<td>{_h(_fmt_float(params.get("min_range")))}</td>'
            f'<td>{_h(_fmt_float(runtime.get("duration_sec")))}</td>'
            f'<td>{_h(_fmt_int(trajectory.get("poses")))}</td>'
            f'<td>{_h(_fmt_float(trajectory.get("duration_sec")))}</td>'
            f'<td>{_h(_fmt_float(trajectory.get("path_length_m")))}</td>'
            f'<td>{_h(_fmt_int(map_quality.get("point_count")))}</td>'
            f'<td>{_h(_fmt_int(map_quality.get("tile_count")))}</td>'
            f'<td>{_h(_fmt_float(map_quality.get("point_density_per_m2")))}</td>'
            f'<td><span class="pill gate-{gate_status.lower()}">{_h(gate_status)}</span></td>'
            f'<td><strong>{_h(row.get("quality_score", 0))}</strong></td>'
            '</tr>'
        )
    return (
        '<section>'
        '<h2>Case Quality</h2>'
        '<table>'
        '<thead><tr>'
        '<th>Rank</th><th>Case</th><th>Status</th><th>Verify</th>'
        '<th>Voxel</th><th>Min Range</th><th>Runtime s</th><th>Poses</th>'
        '<th>Trajectory s</th><th>Path m</th><th>Map Points</th><th>Tiles</th><th>Density</th>'
        '<th>Gate</th><th>Score</th>'
        '</tr></thead><tbody>'
        + ''.join(body)
        + '</tbody></table></section>'
    )


def _h(value: Any) -> str:
    return html.escape('' if value is None else str(value))


def _css() -> str:
    return """
:root {
  color-scheme: light;
  font-family: Inter, ui-sans-serif, system-ui, -apple-system,
    BlinkMacSystemFont, "Segoe UI", sans-serif;
  background: #f7f8fb;
  color: #1f2937;
}
body { margin: 0; }
main { max-width: 1220px; margin: 0 auto; padding: 28px; }
.hero {
  display: flex;
  align-items: flex-start;
  justify-content: space-between;
  gap: 20px;
  padding: 28px 0 22px;
  border-bottom: 1px solid #d7dee8;
}
.eyebrow {
  margin: 0 0 8px;
  font-size: 12px;
  font-weight: 800;
  letter-spacing: 0;
  text-transform: uppercase;
  color: #506074;
}
h1, h2, p { margin-top: 0; }
h1 { margin-bottom: 8px; font-size: 34px; line-height: 1.12; }
h2 { margin: 28px 0 12px; font-size: 19px; }
.subtle { color: #5b6778; overflow-wrap: anywhere; }
.status, .pill {
  border-radius: 6px;
  font-weight: 800;
  text-align: center;
}
.status { min-width: 104px; padding: 10px 14px; }
.pill { display: inline-block; min-width: 76px; padding: 4px 8px; font-size: 12px; }
.pass, .map_verified, .gate-pass { background: #dcfce7; color: #166534; }
.warn, .map_saved { background: #fef3c7; color: #92400e; }
.fail, .gate-fail { background: #fee2e2; color: #991b1b; }
.incomplete, .empty, .no_run { background: #e5e7eb; color: #374151; }
.metrics {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 12px;
  margin: 20px 0;
}
.metric, .best {
  background: #ffffff;
  border: 1px solid #d7dee8;
  border-radius: 8px;
  padding: 16px;
}
.metric h2 { margin: 0 0 8px; font-size: 13px; color: #5b6778; }
.metric p { margin: 0; font-size: 28px; font-weight: 800; }
table {
  width: 100%;
  border-collapse: collapse;
  background: #ffffff;
  border: 1px solid #d7dee8;
}
th, td {
  padding: 10px 12px;
  border-bottom: 1px solid #e5eaf1;
  text-align: left;
  vertical-align: top;
  font-size: 14px;
}
th { color: #506074; font-size: 12px; text-transform: uppercase; letter-spacing: 0; }
code {
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: 12px;
  overflow-wrap: anywhere;
}
@media (max-width: 820px) {
  main { padding: 18px; }
  .hero { display: block; }
  .status { display: inline-block; margin-top: 8px; }
  .metrics { grid-template-columns: repeat(2, minmax(0, 1fr)); }
  table { display: block; overflow-x: auto; white-space: nowrap; }
}
"""
