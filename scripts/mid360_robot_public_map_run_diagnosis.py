#!/usr/bin/env python3
"""Diagnose public MID-360 map-run artifacts."""

from __future__ import annotations

import json
import re
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_MAP_RUN_DIAGNOSIS_JSON = 'mid360_robot_public_map_run_diagnosis.json'
PUBLIC_MAP_RUN_DIAGNOSIS_MARKDOWN = 'mid360_robot_public_map_run_diagnosis.md'

_ANSI_RE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
_LIDAR_DELTA_RE = re.compile(
    r'Received LiDAR scan with\s+([0-9]+(?:\.[0-9]+)?)\s+seconds delta'
)
_KEYPOINTS_RE = re.compile(r'Keypoints for ICP registration\s*=\s*(\d+)')
_BAG_MESSAGE_COUNT_RE = re.compile(r'Bag reader initialized with total message count:\s*(\d+)')
_COMMON_MAP_OUTPUTS = (
    'map_projector_info.yaml',
    'map.pcd',
    'pose_graph.g2o',
    'pointcloud_map/pointcloud_map_metadata.yaml',
)


class PublicDatasetMapRunDiagnosisBuilder:
    """Build diagnosis rows from a public map-candidate manifest."""

    def __init__(self, manifest_path: Path, output_dir: Path) -> None:
        self._manifest_path = manifest_path.expanduser().resolve()
        self._output_dir = output_dir.expanduser().resolve()

    def build(self, dataset_ids: tuple[str, ...] = ()) -> dict[str, Any]:
        """Build a diagnosis report for selected datasets."""
        manifest = _load_json(self._manifest_path)
        wanted_ids = set(dataset_ids)
        runs_by_id = {
            str(item.get('dataset_id') or ''): item
            for item in manifest.get('runs') or []
        }
        rows = []
        skipped = []
        for candidate in manifest.get('candidates') or []:
            dataset_id = str(candidate.get('dataset_id') or '')
            if wanted_ids and dataset_id not in wanted_ids:
                skipped.append({'dataset_id': dataset_id, 'skip_reason': 'dataset not selected'})
                continue
            rows.append(self._diagnose_candidate(candidate, runs_by_id.get(dataset_id, {})))

        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': _overall_status(rows),
            'manifest_path': str(self._manifest_path),
            'output_dir': str(self._output_dir),
            'selection': {'dataset_ids': list(dataset_ids)},
            'datasets': rows,
            'skipped': skipped,
            'counts': _counts(rows, skipped),
        }

    def write(self, report: dict[str, Any]) -> dict[str, Path]:
        """Write diagnosis JSON and Markdown files."""
        self._output_dir.mkdir(parents=True, exist_ok=True)
        json_path = self._output_dir / PUBLIC_MAP_RUN_DIAGNOSIS_JSON
        markdown_path = self._output_dir / PUBLIC_MAP_RUN_DIAGNOSIS_MARKDOWN
        json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
        markdown_path.write_text(render_public_map_run_diagnosis_markdown(report) + '\n',
                                 encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    def _diagnose_candidate(
        self,
        candidate: dict[str, Any],
        run_result: dict[str, Any],
    ) -> dict[str, Any]:
        dataset_id = str(candidate.get('dataset_id') or '')
        output_dir = _candidate_output_dir(candidate)
        launch_log_path = _find_first(output_dir, ('slam.launch.log', 'lidarslam.launch.log'))
        map_save_log_path = _find_first(output_dir, ('map_save.log',))
        launch_log = _strip_ansi(_read_text(launch_log_path))
        runtime = _parse_runtime_signature(launch_log, run_result)
        outputs = _summarize_outputs(output_dir)
        status = _dataset_status(runtime, outputs, run_result, output_dir.exists())
        hints = _problem_hints(runtime, outputs, run_result, status)

        return {
            'dataset_id': dataset_id,
            'title': candidate.get('title', ''),
            'status': status,
            'selected_bag_path': candidate.get('selected_bag_path', ''),
            'output_dir': str(output_dir),
            'files': {
                'launch_log': str(launch_log_path) if launch_log_path else '',
                'map_save_log': str(map_save_log_path) if map_save_log_path else '',
            },
            'run_result': _public_run_result(run_result),
            'runtime': runtime,
            'outputs': outputs,
            'problem_hints': hints,
            'suggested_next_steps': _suggested_next_steps(
                dataset_id=dataset_id,
                selected_bag_path=str(candidate.get('selected_bag_path') or ''),
                output_dir=output_dir,
                launch_log_path=launch_log_path,
                status=status,
            ),
        }


def render_public_map_run_diagnosis_markdown(report: dict[str, Any]) -> str:
    """Render a public map-run diagnosis report as Markdown."""
    counts = report.get('counts') or {}
    lines = [
        '# MID-360 Public Map Run Diagnosis',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- created_at: `{report.get('created_at', '')}`",
        f"- manifest_path: `{report.get('manifest_path', '')}`",
        f"- output_dir: `{report.get('output_dir', '')}`",
        f"- total: `{counts.get('total', 0)}`",
        f"- fail: `{counts.get('fail', 0)}`",
        f"- incomplete: `{counts.get('incomplete', 0)}`",
        '',
        '## Dataset Summary',
        '',
        (
            '| Dataset | Status | Timed Out | Return | LiDAR Delta Errors | '
            'Max Delta | Keypoint Drops | Trajectory Lines | Map Saved |'
        ),
        '| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- |',
    ]
    for row in report.get('datasets') or []:
        runtime = row.get('runtime') or {}
        run_result = row.get('run_result') or {}
        outputs = row.get('outputs') or {}
        trajectory = outputs.get('trajectory') or {}
        lines.append(
            '| '
            + ' | '.join([
                f"`{row.get('dataset_id', '')}`",
                f"`{row.get('status', '')}`",
                '`yes`' if run_result.get('timed_out') else '`no`',
                _fmt_value(run_result.get('returncode')),
                _fmt_value(runtime.get('lidar_delta_error_count')),
                _fmt_seconds(runtime.get('lidar_delta_max_sec')),
                _fmt_value(runtime.get('keypoints_too_few_count')),
                _fmt_value(trajectory.get('total_lines')),
                '`yes`' if outputs.get('map_saved') else '`no`',
            ])
            + ' |'
        )

    for row in report.get('datasets') or []:
        runtime = row.get('runtime') or {}
        outputs = row.get('outputs') or {}
        trajectory = outputs.get('trajectory') or {}
        lines.extend([
            '',
            f"## {row.get('dataset_id', '')}",
            '',
            f"- status: `{row.get('status', '')}`",
            f"- selected_bag_path: `{row.get('selected_bag_path', '')}`",
            f"- output_dir: `{row.get('output_dir', '')}`",
            f"- launch_log: `{(row.get('files') or {}).get('launch_log', 'missing') or 'missing'}`",
            f"- rko_started: `{runtime.get('rko_started')}`",
            f"- graph_initialized: `{runtime.get('graph_initialized')}`",
            f"- offline_completed: `{runtime.get('offline_completed')}`",
            f"- map_save_called: `{runtime.get('map_save_called')}`",
            f"- bag_message_count: `{runtime.get('bag_message_count')}`",
            f"- lidar_delta_first_values_sec: `{runtime.get('lidar_delta_first_values_sec')}`",
            f"- trajectory_files: `{trajectory.get('file_count', 0)}`",
            f"- trajectory_total_lines: `{trajectory.get('total_lines', 0)}`",
            '',
            '### Hints',
            '',
        ])
        hints = row.get('problem_hints') or []
        if hints:
            for hint in hints:
                lines.append(f'- {hint}')
        else:
            lines.append('- No public-run failure signature was detected.')

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


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _candidate_output_dir(candidate: dict[str, Any]) -> Path:
    safety = candidate.get('safety') or {}
    output_dir = safety.get('output_dir') or candidate.get('output_dir') or ''
    return Path(str(output_dir)).expanduser().resolve()


def _find_first(root: Path, names: tuple[str, ...]) -> Path | None:
    for name in names:
        path = root / name
        if path.is_file():
            return path
    return None


def _read_text(path: Path | None) -> str:
    if path is None or not path.is_file():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _strip_ansi(text: str) -> str:
    return _ANSI_RE.sub('', text)


def _parse_runtime_signature(
    launch_log: str,
    run_result: dict[str, Any],
) -> dict[str, Any]:
    combined = '\n'.join([
        launch_log,
        str(run_result.get('stdout') or ''),
        str(run_result.get('stderr') or ''),
    ])
    deltas = [float(match.group(1)) for match in _LIDAR_DELTA_RE.finditer(combined)]
    keypoints = [int(match.group(1)) for match in _KEYPOINTS_RE.finditer(combined)]
    bag_count_match = _BAG_MESSAGE_COUNT_RE.search(combined)
    return {
        'rko_started': 'RKO LIO Node is up!' in combined,
        'graph_initialized': '[graph_based_slam]: initialization end' in combined,
        'first_cloud_received': 'First cloud received' in combined,
        'first_odom_received': 'First odom received' in combined,
        'offline_completed': 'RKO LIO Offline Node took' in combined,
        'map_save_called': 'Calling /map_save' in combined or 'map_save service call' in combined,
        'process_died': 'process has died' in combined,
        'wrapper_timeout': bool(run_result.get('timed_out')),
        'bag_message_count': int(bag_count_match.group(1)) if bag_count_match else None,
        'lidar_delta_error_count': len(deltas),
        'lidar_delta_min_sec': min(deltas) if deltas else None,
        'lidar_delta_max_sec': max(deltas) if deltas else None,
        'lidar_delta_first_values_sec': deltas[:5],
        'lidar_delta_last_values_sec': deltas[-5:] if deltas else [],
        'keypoints_too_few_count': len(keypoints),
        'keypoints_min': min(keypoints) if keypoints else None,
        'intensity_missing_count': combined.count("Failed to find match for field 'intensity'"),
        'loop_search_count': combined.count('searching Loop'),
        'buffer_throttle_count': combined.count('throttling the bag reading thread'),
    }


def _summarize_outputs(output_dir: Path) -> dict[str, Any]:
    trajectory_files = _trajectory_files(output_dir)
    trajectory_rows = [_trajectory_file_summary(path, output_dir) for path in trajectory_files]
    existing_map_outputs = [
        str((output_dir / relative).resolve())
        for relative in _COMMON_MAP_OUTPUTS
        if (output_dir / relative).exists()
    ]
    pointcloud_map_dir = output_dir / 'pointcloud_map'
    pointcloud_tile_count = 0
    if pointcloud_map_dir.is_dir():
        pointcloud_tile_count = sum(1 for path in pointcloud_map_dir.glob('*.pcd') if path.is_file())
    return {
        'map_saved': bool(existing_map_outputs),
        'existing_map_outputs': existing_map_outputs,
        'pointcloud_tile_count': pointcloud_tile_count,
        'trajectory': {
            'file_count': len(trajectory_rows),
            'total_lines': sum(int(item['line_count']) for item in trajectory_rows),
            'files': trajectory_rows,
        },
    }


def _trajectory_files(output_dir: Path) -> list[Path]:
    if not output_dir.is_dir():
        return []
    names = ('*_tum_*.txt', 'traj_*.tum', '*.tum')
    paths: set[Path] = set()
    for pattern in names:
        paths.update(path for path in output_dir.rglob(pattern) if path.is_file())
    return sorted(paths)


def _trajectory_file_summary(path: Path, root: Path) -> dict[str, Any]:
    text = _read_text(path)
    lines = [line for line in text.splitlines() if line.strip()]
    return {
        'path': str(path.resolve()),
        'relative_path': str(path.resolve().relative_to(root.resolve())),
        'line_count': len(lines),
        'size_bytes': path.stat().st_size if path.is_file() else 0,
    }


def _dataset_status(
    runtime: dict[str, Any],
    outputs: dict[str, Any],
    run_result: dict[str, Any],
    output_dir_exists: bool,
) -> str:
    if outputs.get('map_saved'):
        return 'MAP_SAVED'
    if run_result and run_result.get('returncode') not in (None, 0):
        return 'FAIL'
    if runtime.get('wrapper_timeout'):
        return 'FAIL'
    if runtime.get('offline_completed') and not outputs.get('map_saved'):
        return 'MAP_SAVE_MISSING'
    if (
        output_dir_exists
        and (
            runtime.get('rko_started')
            or runtime.get('graph_initialized')
            or runtime.get('lidar_delta_error_count')
        )
    ):
        return 'INCOMPLETE'
    return 'NO_RUN'


def _public_run_result(run_result: dict[str, Any]) -> dict[str, Any]:
    if not run_result:
        return {
            'present': False,
            'returncode': None,
            'timed_out': False,
            'timeout_sec': 0,
            'duration_sec': None,
        }
    return {
        'present': True,
        'returncode': run_result.get('returncode'),
        'timed_out': bool(run_result.get('timed_out')),
        'timeout_sec': run_result.get('timeout_sec', 0),
        'duration_sec': run_result.get('duration_sec'),
    }


def _problem_hints(
    runtime: dict[str, Any],
    outputs: dict[str, Any],
    run_result: dict[str, Any],
    status: str,
) -> list[str]:
    hints: list[str] = []
    if run_result.get('timed_out'):
        hints.append('The wrapper timeout fired before the map run reached completion.')
    if int(runtime.get('lidar_delta_error_count') or 0) > 0:
        hints.append(
            'RKO-LIO dropped many LiDAR frames because scan timestamp deltas exceed '
            'its expected contiguous scan interval.'
        )
    if int(runtime.get('keypoints_too_few_count') or 0) > 0:
        hints.append(
            'Some scans produced too few ICP keypoints; this can indicate sparse/cropped '
            'input or a voxel size that is too aggressive for the bag.'
        )
    if int(runtime.get('intensity_missing_count') or 0) > 0:
        hints.append(
            'The point cloud lacks a classic intensity field; graph_based_slam continued, '
            'but the bag does not match every classic LiDAR assumption.'
        )
    if runtime.get('rko_started') and not runtime.get('offline_completed'):
        hints.append('RKO-LIO started, but the offline node did not report completion.')
    if not runtime.get('map_save_called') and not outputs.get('map_saved'):
        hints.append('/map_save was not reached, so no Autoware pointcloud map was written.')
    trajectory = outputs.get('trajectory') or {}
    if int(trajectory.get('total_lines') or 0) > 0 and not outputs.get('map_saved'):
        hints.append(
            'Partial TUM trajectory output exists, so the run passed startup and failed '
            'later in offline processing or save orchestration.'
        )
    if status == 'NO_RUN':
        hints.append('No launch log or run result was found for this candidate yet.')
    return hints


def _suggested_next_steps(
    dataset_id: str,
    selected_bag_path: str,
    output_dir: Path,
    launch_log_path: Path | None,
    status: str,
) -> list[str]:
    steps: list[str] = []
    if launch_log_path:
        steps.append(f'tail -n 120 {launch_log_path}')
    if selected_bag_path:
        steps.append(f'ros2 bag info {selected_bag_path}')
    if status in ('FAIL', 'INCOMPLETE', 'MAP_SAVE_MISSING'):
        steps.append(
            'python3 scripts/diagnose_mid360_robot_public_map_run.py '
            f'--datasets {dataset_id} --write'
        )
    if output_dir.is_dir():
        steps.append(f'find {output_dir} -maxdepth 3 -type f | sort')
    return steps[:5]


def _overall_status(rows: list[dict[str, Any]]) -> str:
    if not rows:
        return 'EMPTY'
    statuses = {str(row.get('status') or '') for row in rows}
    if 'FAIL' in statuses:
        return 'FAIL'
    if 'MAP_SAVE_MISSING' in statuses or 'INCOMPLETE' in statuses:
        return 'INCOMPLETE'
    if statuses == {'MAP_SAVED'}:
        return 'PASS'
    if 'NO_RUN' in statuses:
        return 'INCOMPLETE'
    return 'WARN'


def _counts(rows: list[dict[str, Any]], skipped: list[dict[str, Any]]) -> dict[str, int]:
    statuses = [str(row.get('status') or '') for row in rows]
    return {
        'total': len(rows),
        'map_saved': sum(1 for status in statuses if status == 'MAP_SAVED'),
        'fail': sum(1 for status in statuses if status == 'FAIL'),
        'incomplete': sum(
            1 for status in statuses if status in ('INCOMPLETE', 'MAP_SAVE_MISSING', 'NO_RUN')
        ),
        'skipped': len(skipped),
    }


def _fmt_value(value: Any) -> str:
    if value is None:
        return ''
    return str(value)


def _fmt_seconds(value: Any) -> str:
    if value is None:
        return ''
    try:
        return f'{float(value):.1f}s'
    except Exception:
        return str(value)
