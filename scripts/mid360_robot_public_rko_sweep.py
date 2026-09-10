#!/usr/bin/env python3
"""Generate and optionally run RKO-LIO parameter sweeps on public MID-360 bags."""

from __future__ import annotations

import io
import os
import re
import signal
import shlex
import subprocess
import time
from contextlib import redirect_stdout
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


RKO_SWEEP_JSON = 'mid360_robot_public_rko_sweep.json'
RKO_SWEEP_MARKDOWN = 'mid360_robot_public_rko_sweep.md'
CASE_CONFIG_NAME = 'rko_sweep.yaml'
VERIFY_LOG_NAME = 'verify_autoware_map.log'

_ANSI_RE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
_LIDAR_DELTA_RE = re.compile(
    r'Received LiDAR scan with\s+([0-9]+(?:\.[0-9]+)?)\s+seconds delta'
)
_KEYPOINTS_RE = re.compile(r'Keypoints for ICP registration\s*=\s*(\d+)')
_BAG_MESSAGE_COUNT_RE = re.compile(r'Bag reader initialized with total message count:\s*(\d+)')
_MAP_OUTPUTS = (
    'map_projector_info.yaml',
    'pointcloud_map/pointcloud_map_metadata.yaml',
    'map.pcd',
    'pose_graph.g2o',
)
_RUNTIME_COLLISION_NAMES = (
    'slam.launch.log',
    'map_save.log',
    'map_projector_info.yaml',
    'pointcloud_map',
    'map.pcd',
    'pose_graph.g2o',
    'rko_params.ros.yaml',
    VERIFY_LOG_NAME,
)


@dataclass(frozen=True)
class RkoSweepCase:
    """One RKO-LIO frontend parameter candidate."""

    label: str
    voxel_size: float
    min_range: float
    max_range: float = 100.0
    deskew: bool = False
    double_downsample: bool = True
    initialization_phase: bool = False

    @property
    def case_id(self) -> str:
        if self.label:
            return _safe_id(self.label)
        parts = [
            f'voxel_{_num_id(self.voxel_size)}',
            f'min_{_num_id(self.min_range)}',
            f'max_{_num_id(self.max_range)}',
            f'dd_{_bool_id(self.double_downsample)}',
            f'deskew_{_bool_id(self.deskew)}',
        ]
        return '_'.join(parts)

    def parameters(self) -> dict[str, Any]:
        return {
            'voxel_size': float(self.voxel_size),
            'min_range': float(self.min_range),
            'max_range': float(self.max_range),
            'deskew': bool(self.deskew),
            'double_downsample': bool(self.double_downsample),
            'initialization_phase': bool(self.initialization_phase),
        }


@dataclass(frozen=True)
class RkoSweepOptions:
    """Inputs shared by all RKO-LIO sweep cases."""

    repo_root: Path
    bag_path: Path
    output_dir: Path
    base_rko_param: Path
    lidarslam_param: Path
    lidar_topic: str = '/livox/points'
    imu_topic: str = '/livox/imu'
    base_frame: str = 'base_link'
    lidar_frame: str = 'livox_frame'
    imu_frame: str = 'livox_frame'
    save_timeout_secs: int = 60
    startup_timeout_secs: int = 30
    offline_quiet_log_secs: int = 0
    allow_existing_output: bool = False
    limit: int = 0


@dataclass(frozen=True)
class RkoSweepRunOptions:
    """Execution options for a parameter sweep."""

    timeout_sec: int = 90


class RkoSweepBuilder:
    """Build RKO-LIO parameter sweep manifests and execute selected cases."""

    def __init__(self, options: RkoSweepOptions, cases: tuple[RkoSweepCase, ...]) -> None:
        self._options = options
        self._cases = cases

    def build(
        self, run: bool = False, run_options: RkoSweepRunOptions | None = None,
    ) -> dict[str, Any]:
        """Build a sweep manifest, executing runnable cases only when requested."""
        selected_cases = self._selected_cases()
        case_rows = [self._build_case_row(case) for case in selected_cases]
        runnable = [row for row in case_rows if (row.get('safety') or {}).get('can_run')]
        blocked = [row for row in case_rows if not (row.get('safety') or {}).get('can_run')]
        run_config = run_options or RkoSweepRunOptions()
        runs = self._run_cases(runnable, run_config) if run else []
        runs_by_id = {str(item.get('case_id') or ''): item for item in runs}
        diagnostics = [
            diagnose_rko_sweep_case(row, runs_by_id.get(str(row.get('case_id') or ''), {}))
            for row in case_rows
        ]

        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': _overall_status(case_rows, blocked, runs, diagnostics, run),
            'mode': 'RUN' if run else 'PLAN',
            'bag_path': str(self._options.bag_path.expanduser().resolve()),
            'output_dir': str(self._options.output_dir.expanduser().resolve()),
            'base_rko_param': str(self._options.base_rko_param.expanduser().resolve()),
            'lidarslam_param': str(self._options.lidarslam_param.expanduser().resolve()),
            'topics': {
                'pointcloud': self._options.lidar_topic,
                'imu': self._options.imu_topic,
            },
            'frames': {
                'base_frame': self._options.base_frame,
                'lidar_frame': self._options.lidar_frame,
                'imu_frame': self._options.imu_frame,
            },
            'run_options': {
                'timeout_sec': run_config.timeout_sec,
                'save_timeout_secs': self._options.save_timeout_secs,
                'startup_timeout_secs': self._options.startup_timeout_secs,
                'offline_quiet_log_secs': self._options.offline_quiet_log_secs,
            },
            'cases': case_rows,
            'blocked': blocked,
            'runs': runs,
            'diagnostics': diagnostics,
            'counts': {
                'cases': len(case_rows),
                'runnable': len(runnable),
                'blocked': len(blocked),
                'run': len(runs),
                'failed': sum(1 for item in runs if item.get('returncode') != 0),
                'map_saved': sum(
                    1 for item in diagnostics if (item.get('outputs') or {}).get('map_saved')
                ),
                'map_verified': sum(
                    1
                    for item in diagnostics
                    if (item.get('verification') or {}).get('result') == 'PASS'
                ),
                'verify_failed': sum(
                    1
                    for item in diagnostics
                    if (item.get('verification') or {}).get('result') == 'FAIL'
                ),
                'keypoint_drop_cases': sum(
                    1
                    for item in diagnostics
                    if int((item.get('runtime') or {}).get('keypoints_too_few_count') or 0) > 0
                ),
                'lidar_delta_cases': sum(
                    1
                    for item in diagnostics
                    if int((item.get('runtime') or {}).get('lidar_delta_error_count') or 0) > 0
                ),
            },
        }

    def write(self, manifest: dict[str, Any]) -> dict[str, Path]:
        """Write sweep JSON and Markdown artifacts."""
        output_dir = self._options.output_dir.expanduser().resolve()
        output_dir.mkdir(parents=True, exist_ok=True)
        json_path = output_dir / RKO_SWEEP_JSON
        markdown_path = output_dir / RKO_SWEEP_MARKDOWN
        json_path.write_text(payload_to_json(manifest) + '\n', encoding='utf-8')
        markdown_path.write_text(render_rko_sweep_markdown(manifest) + '\n', encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    def _selected_cases(self) -> tuple[RkoSweepCase, ...]:
        if self._options.limit <= 0:
            return self._cases
        return self._cases[:self._options.limit]

    def _build_case_row(self, case: RkoSweepCase) -> dict[str, Any]:
        case_dir = self._options.output_dir.expanduser().resolve() / case.case_id
        config_path = case_dir / CASE_CONFIG_NAME
        case_dir.mkdir(parents=True, exist_ok=True)
        write_rko_case_config(self._options.base_rko_param, config_path, case)
        command = build_rko_sweep_command(self._options, case, config_path, case_dir)
        row = {
            'case_id': case.case_id,
            'label': case.label or case.case_id,
            'parameters': case.parameters(),
            'config_path': str(config_path),
            'output_dir': str(case_dir),
            'command': command,
            'command_shell': shlex.join(command),
        }
        return {**row, 'safety': _build_case_safety(self._options, case_dir)}

    def _run_cases(
        self,
        rows: list[dict[str, Any]],
        run_options: RkoSweepRunOptions,
    ) -> list[dict[str, Any]]:
        results = []
        for row in rows:
            results.append(
                _run_command(
                    case_id=str(row['case_id']),
                    command=[str(item) for item in row['command']],
                    command_shell=str(row['command_shell']),
                    timeout_sec=max(0, int(run_options.timeout_sec)),
                    cwd=self._options.repo_root.expanduser().resolve(),
                )
            )
        return results


def default_rko_sweep_cases() -> tuple[RkoSweepCase, ...]:
    """Return a small real-data sweep focused on sparse MID-360 keypoint drops."""
    return (
        RkoSweepCase(
            label='voxel_0p50_min_1p00_dd_on',
            voxel_size=0.5,
            min_range=1.0,
            deskew=False,
            double_downsample=True,
        ),
        RkoSweepCase(
            label='voxel_0p30_min_1p00_dd_on',
            voxel_size=0.3,
            min_range=1.0,
            deskew=False,
            double_downsample=True,
        ),
        RkoSweepCase(
            label='voxel_0p50_min_0p50_dd_on',
            voxel_size=0.5,
            min_range=0.5,
            deskew=False,
            double_downsample=True,
        ),
        RkoSweepCase(
            label='voxel_0p50_min_1p00_dd_off',
            voxel_size=0.5,
            min_range=1.0,
            deskew=False,
            double_downsample=False,
        ),
    )


def parse_rko_sweep_case(spec: str) -> RkoSweepCase:
    """Parse a CLI case spec.

    Supported form:
      label:voxel_size=0.5,min_range=1.0,double_downsample=true,deskew=false
    """
    text = spec.strip()
    if not text:
        raise ValueError('empty case spec')
    label = ''
    body = text
    if ':' in text:
        label, body = text.split(':', 1)
        label = label.strip()
    values: dict[str, str] = {}
    for token in body.split(','):
        item = token.strip()
        if not item:
            continue
        if '=' not in item:
            raise ValueError(f'case item must be key=value: {item}')
        key, value = item.split('=', 1)
        values[_canonical_key(key.strip())] = value.strip()

    return RkoSweepCase(
        label=label,
        voxel_size=float(_required(values, 'voxel_size')),
        min_range=float(values.get('min_range', '1.0')),
        max_range=float(values.get('max_range', '100.0')),
        deskew=_parse_bool(values.get('deskew', 'false')),
        double_downsample=_parse_bool(values.get('double_downsample', 'true')),
        initialization_phase=_parse_bool(values.get('initialization_phase', 'false')),
    )


def write_rko_case_config(base_config_path: Path, output_path: Path, case: RkoSweepCase) -> None:
    """Write an RKO-LIO YAML by applying a sweep case to a base config."""
    data = _load_yaml_dict(base_config_path)
    data.update(case.parameters())
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')


def build_rko_sweep_command(
    options: RkoSweepOptions,
    case: RkoSweepCase,
    config_path: Path,
    case_dir: Path,
) -> list[str]:
    """Build the dogfood wrapper command for one sweep case."""
    command = [
        'bash',
        str(options.repo_root / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh'),
        '--bag',
        str(options.bag_path.expanduser().resolve()),
        '--lidar-topic',
        options.lidar_topic,
        '--imu-topic',
        options.imu_topic,
        '--lidarslam-param',
        str(options.lidarslam_param.expanduser().resolve()),
        '--rko-param',
        str(config_path),
        '--base-frame',
        options.base_frame,
        '--lidar-frame',
        options.lidar_frame,
        '--imu-frame',
        options.imu_frame,
        '--output-dir',
        str(case_dir),
        '--run-name',
        case.case_id,
        '--save-timeout-secs',
        str(max(1, int(options.save_timeout_secs))),
        '--startup-timeout-secs',
        str(max(1, int(options.startup_timeout_secs))),
        '--offline-quiet-log-secs',
        str(max(0, int(options.offline_quiet_log_secs))),
        '--wait-for-offline-completion',
        '--skip-viewer',
    ]
    return command


def diagnose_rko_sweep_case(
    case_row: dict[str, Any], run_result: dict[str, Any],
) -> dict[str, Any]:
    """Summarize runtime signatures and output files for one sweep case."""
    output_dir = Path(str(case_row.get('output_dir') or '')).expanduser().resolve()
    launch_log_path = output_dir / 'slam.launch.log'
    launch_log = _strip_ansi(_read_text(launch_log_path))
    runtime = _parse_runtime_signature(launch_log, run_result)
    outputs = _summarize_outputs(output_dir)
    verification = (
        _verify_autoware_map_output(output_dir)
        if outputs.get('map_saved') else _no_verify()
    )
    status = _case_status(runtime, outputs, verification, run_result, output_dir.exists())
    return {
        'case_id': case_row.get('case_id', ''),
        'label': case_row.get('label', ''),
        'status': status,
        'parameters': case_row.get('parameters') or {},
        'output_dir': str(output_dir),
        'files': {
            'launch_log': str(launch_log_path) if launch_log_path.is_file() else '',
            'verify_log': verification.get('log_path', ''),
        },
        'run_result': _public_run_result(run_result),
        'runtime': runtime,
        'outputs': outputs,
        'verification': verification,
        'problem_hints': _problem_hints(runtime, outputs, verification, run_result, status),
    }


def render_rko_sweep_markdown(manifest: dict[str, Any]) -> str:
    """Render an RKO-LIO public sweep manifest as Markdown."""
    counts = manifest.get('counts') or {}
    lines = [
        '# MID-360 Public RKO-LIO Sweep',
        '',
        f"- status: `{manifest.get('status', '')}`",
        f"- mode: `{manifest.get('mode', '')}`",
        f"- created_at: `{manifest.get('created_at', '')}`",
        f"- bag_path: `{manifest.get('bag_path', '')}`",
        f"- output_dir: `{manifest.get('output_dir', '')}`",
        f"- cases: `{counts.get('cases', 0)}`",
        f"- runnable: `{counts.get('runnable', 0)}`",
        f"- blocked: `{counts.get('blocked', 0)}`",
        f"- map_verified: `{counts.get('map_verified', 0)}`",
        '',
        '## Case Summary',
        '',
    ]
    diagnostics = manifest.get('diagnostics') or []
    if diagnostics:
        lines.extend([
            (
                '| Case | Status | Voxel | Min Range | Double Downsample | Deskew | '
                'Timed Out | Return | Keypoint Drops | Min Keypoints | LiDAR Delta Errors | '
                'Offline | Map Saved | Verify |'
            ),
            (
                '| --- | --- | ---: | ---: | --- | --- | --- | ---: | ---: | ---: | ---: | '
                '--- | --- | --- |'
            ),
        ])
        for row in diagnostics:
            params = row.get('parameters') or {}
            runtime = row.get('runtime') or {}
            run_result = row.get('run_result') or {}
            outputs = row.get('outputs') or {}
            verification = row.get('verification') or {}
            lines.append(
                '| '
                + ' | '.join([
                    f"`{row.get('case_id', '')}`",
                    f"`{row.get('status', '')}`",
                    _fmt_float(params.get('voxel_size')),
                    _fmt_float(params.get('min_range')),
                    f"`{params.get('double_downsample')}`",
                    f"`{params.get('deskew')}`",
                    '`yes`' if run_result.get('timed_out') else '`no`',
                    _fmt_value(run_result.get('returncode')),
                    _fmt_value(runtime.get('keypoints_too_few_count')),
                    _fmt_value(runtime.get('keypoints_min')),
                    _fmt_value(runtime.get('lidar_delta_error_count')),
                    '`yes`' if runtime.get('offline_completed') else '`no`',
                    '`yes`' if outputs.get('map_saved') else '`no`',
                    f"`{verification.get('result', 'SKIP')}`",
                ])
                + ' |'
            )
    else:
        lines.append('- none')

    best = _best_case(diagnostics)
    if best:
        lines.extend([
            '',
            '## Current Best',
            '',
            f"- case: `{best.get('case_id', '')}`",
            f"- status: `{best.get('status', '')}`",
            f"- output_dir: `{best.get('output_dir', '')}`",
        ])

    lines.extend(['', '## Commands', ''])
    cases = manifest.get('cases') or []
    if cases:
        for row in cases:
            lines.extend([
                f"### {row.get('case_id', '')}",
                '',
                f"- config_path: `{row.get('config_path', '')}`",
                f"- output_dir: `{row.get('output_dir', '')}`",
                '',
                '```bash',
                str(row.get('command_shell', '')),
                '```',
                '',
            ])
    else:
        lines.append('- none')

    blocked = manifest.get('blocked') or []
    lines.extend(['', '## Blocked', ''])
    if blocked:
        for row in blocked:
            safety = row.get('safety') or {}
            failures = '; '.join(str(item) for item in safety.get('failures') or [])
            lines.append(f"- `{row.get('case_id', '')}`: {failures}")
    else:
        lines.append('- none')

    runs = manifest.get('runs') or []
    if runs:
        lines.extend(['', '## Runs', ''])
        for row in runs:
            timeout = ' timed out' if row.get('timed_out') else ''
            lines.append(
                f"- `{row.get('case_id', '')}` returncode `{row.get('returncode')}`"
                f"{timeout} duration `{_fmt_seconds(row.get('duration_sec'))}`"
            )

    for row in diagnostics:
        hints = row.get('problem_hints') or []
        if not hints:
            continue
        lines.extend(['', f"## Hints: {row.get('case_id', '')}", ''])
        for hint in hints:
            lines.append(f'- {hint}')

    return '\n'.join(lines)


def _load_yaml_dict(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.expanduser().read_text(encoding='utf-8')) or {}
    if not isinstance(data, dict):
        raise ValueError(f'RKO config must be a YAML mapping: {path}')
    return data


def _build_case_safety(options: RkoSweepOptions, case_dir: Path) -> dict[str, Any]:
    failures: list[str] = []
    warnings: list[str] = []
    bag_path = options.bag_path.expanduser().resolve()
    metadata_path = bag_path / 'metadata.yaml'
    base_config = options.base_rko_param.expanduser().resolve()
    lidarslam_param = options.lidarslam_param.expanduser().resolve()
    collisions = _runtime_collisions(case_dir)

    if not bag_path.is_dir():
        failures.append(f'bag directory does not exist: {bag_path}')
    if not metadata_path.is_file():
        failures.append(f'rosbag2 metadata.yaml is missing: {metadata_path}')
    if not base_config.is_file():
        failures.append(f'base RKO config does not exist: {base_config}')
    if not lidarslam_param.is_file():
        failures.append(f'lidarslam param does not exist: {lidarslam_param}')
    if collisions and not options.allow_existing_output:
        failures.append('runtime output collision detected')
    if not options.lidar_topic:
        failures.append('lidar topic is empty')
    if not options.imu_topic:
        warnings.append('imu topic is empty')

    status = 'FAIL' if failures else ('WARN' if warnings else 'OK')
    return {
        'status': status,
        'can_run': status != 'FAIL',
        'failures': failures,
        'warnings': warnings,
        'existing_runtime_outputs': [str(path) for path in collisions],
    }


def _runtime_collisions(case_dir: Path) -> list[Path]:
    if not case_dir.exists():
        return []
    return [
        (case_dir / relative).resolve()
        for relative in _RUNTIME_COLLISION_NAMES
        if (case_dir / relative).exists()
    ]


def _run_command(
    case_id: str,
    command: list[str],
    command_shell: str,
    timeout_sec: int,
    cwd: Path,
) -> dict[str, Any]:
    started = time.monotonic()
    process = subprocess.Popen(
        command,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    timed_out = False
    try:
        stdout, stderr = process.communicate(
            timeout=timeout_sec if timeout_sec > 0 else None
        )
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        _terminate_process_group(process.pid)
        stdout, stderr = process.communicate()
        stdout = _decode_timeout_output(exc.stdout) + (stdout or '')
        stderr = _decode_timeout_output(exc.stderr) + (stderr or '')

    return {
        'case_id': case_id,
        'command': command,
        'command_shell': command_shell,
        'returncode': 124 if timed_out else process.returncode,
        'timed_out': timed_out,
        'timeout_sec': timeout_sec,
        'duration_sec': time.monotonic() - started,
        'stdout': stdout or '',
        'stderr': stderr or '',
    }


def _terminate_process_group(pid: int) -> None:
    try:
        os.killpg(pid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            os.killpg(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.2)
    try:
        os.killpg(pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            os.killpg(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.2)
    try:
        os.killpg(pid, signal.SIGKILL)
    except ProcessLookupError:
        return


def _decode_timeout_output(value: Any) -> str:
    if value is None:
        return ''
    if isinstance(value, bytes):
        return value.decode(errors='replace')
    return str(value)


def _strip_ansi(text: str) -> str:
    return _ANSI_RE.sub('', text)


def _read_text(path: Path) -> str:
    if not path.is_file():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _parse_runtime_signature(launch_log: str, run_result: dict[str, Any]) -> dict[str, Any]:
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
        'keypoints_too_few_count': len(keypoints),
        'keypoints_min': min(keypoints) if keypoints else None,
        'deskew_disabled': 'Deskewing is disabled' in combined,
        'deskew_enabled': 'Deskewing is enabled' in combined,
        'buffer_throttle_count': combined.count('throttling the bag reading thread'),
    }


def _summarize_outputs(output_dir: Path) -> dict[str, Any]:
    existing_map_outputs = [
        str((output_dir / relative).resolve())
        for relative in _MAP_OUTPUTS
        if (output_dir / relative).exists()
    ]
    trajectory_files = _trajectory_files(output_dir)
    return {
        'map_saved': bool(existing_map_outputs),
        'existing_map_outputs': existing_map_outputs,
        'trajectory': {
            'file_count': len(trajectory_files),
            'total_lines': sum(_line_count(path) for path in trajectory_files),
            'files': [str(path.resolve()) for path in trajectory_files],
        },
    }


def _verify_autoware_map_output(output_dir: Path) -> dict[str, Any]:
    pointcloud_map_dir = output_dir / 'pointcloud_map'
    log_path = output_dir / VERIFY_LOG_NAME
    if not pointcloud_map_dir.is_dir():
        return {
            'result': 'SKIP',
            'reason': 'pointcloud_map directory is missing',
            'log_path': '',
            'counts': None,
        }
    try:
        from lidarslam_benchmark_tools.verify_autoware_map import MapVerifier

        stream = io.StringIO()
        verifier = MapVerifier(str(pointcloud_map_dir), check_bounds=False, verbose=False)
        with redirect_stdout(stream):
            success = verifier.run()
        text = stream.getvalue()
        log_path.write_text(text, encoding='utf-8')
        return {
            'result': 'PASS' if success else 'FAIL',
            'reason': '',
            'log_path': str(log_path),
            'map_dir': str(pointcloud_map_dir.resolve()),
            'counts': {
                'pass': len(verifier.passes),
                'warn': len(verifier.warnings),
                'fail': len(verifier.failures),
            },
            'warnings': verifier.warnings[:10],
            'failures': verifier.failures[:10],
        }
    except Exception as exc:
        log_path.write_text(f'Autoware map verification failed to run: {exc}\n', encoding='utf-8')
        return {
            'result': 'FAIL',
            'reason': str(exc),
            'log_path': str(log_path),
            'map_dir': str(pointcloud_map_dir.resolve()),
            'counts': None,
            'warnings': [],
            'failures': [str(exc)],
        }


def _no_verify() -> dict[str, Any]:
    return {
        'result': 'SKIP',
        'reason': 'map outputs are missing',
        'log_path': '',
        'counts': None,
        'warnings': [],
        'failures': [],
    }


def _trajectory_files(output_dir: Path) -> list[Path]:
    if not output_dir.is_dir():
        return []
    paths: set[Path] = set()
    for pattern in ('*_tum_*.txt', 'traj_*.tum', '*.tum'):
        paths.update(path for path in output_dir.rglob(pattern) if path.is_file())
    return sorted(paths)


def _line_count(path: Path) -> int:
    return sum(1 for line in _read_text(path).splitlines() if line.strip())


def _case_status(
    runtime: dict[str, Any],
    outputs: dict[str, Any],
    verification: dict[str, Any],
    run_result: dict[str, Any],
    output_dir_exists: bool,
) -> str:
    if outputs.get('map_saved'):
        if verification.get('result') == 'PASS':
            return 'MAP_VERIFIED'
        if verification.get('result') == 'FAIL':
            return 'VERIFY_FAILED'
        return 'MAP_SAVED'
    if run_result and run_result.get('returncode') not in (None, 0):
        return 'FAIL'
    if runtime.get('wrapper_timeout'):
        return 'FAIL'
    if runtime.get('offline_completed'):
        return 'MAP_SAVE_MISSING'
    if output_dir_exists and (runtime.get('rko_started') or runtime.get('graph_initialized')):
        return 'INCOMPLETE'
    if output_dir_exists:
        return 'PLAN'
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
    verification: dict[str, Any],
    run_result: dict[str, Any],
    status: str,
) -> list[str]:
    hints: list[str] = []
    if run_result.get('timed_out'):
        hints.append('The wrapper timeout fired before the case reached completion.')
    if int(runtime.get('keypoints_too_few_count') or 0) > 0:
        hints.append(
            'RKO-LIO reported too few ICP keypoints; compare lower voxel_size or '
            'double_downsample=false cases.'
        )
    if int(runtime.get('lidar_delta_error_count') or 0) > 0:
        hints.append(
            'LiDAR timestamp-delta errors remain in this case; retry on a tighter '
            'contiguous bag segment before tuning ICP parameters.'
        )
    if runtime.get('rko_started') and not runtime.get('offline_completed'):
        hints.append('RKO-LIO started but did not report offline completion.')
    if runtime.get('offline_completed') and not outputs.get('map_saved'):
        hints.append('Offline odometry completed but no Autoware map output was saved.')
    if verification.get('result') == 'FAIL':
        failures = verification.get('failures') or []
        if failures:
            hints.append(f"Autoware map verification failed: {failures[0]}")
        else:
            hints.append('Autoware map verification failed.')
    if status == 'PLAN':
        hints.append('Case was planned but not executed.')
    return hints


def _overall_status(
    cases: list[dict[str, Any]],
    blocked: list[dict[str, Any]],
    runs: list[dict[str, Any]],
    diagnostics: list[dict[str, Any]],
    run: bool,
) -> str:
    if not cases:
        return 'EMPTY'
    if len(blocked) == len(cases):
        return 'BLOCKED'
    if any(row.get('status') == 'VERIFY_FAILED' for row in diagnostics):
        return 'FAIL'
    if any(row.get('status') == 'MAP_VERIFIED' for row in diagnostics):
        return 'PASS' if not blocked else 'PARTIAL'
    if any(row.get('status') == 'MAP_SAVED' for row in diagnostics):
        return 'WARN' if not blocked else 'PARTIAL'
    if not run:
        return 'PARTIAL' if blocked else 'READY'
    if any(item.get('returncode') != 0 for item in runs):
        return 'FAIL'
    if blocked:
        return 'PARTIAL'
    return 'INCOMPLETE'


def _best_case(diagnostics: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not diagnostics:
        return None

    def score(row: dict[str, Any]) -> tuple[int, int, int, int]:
        runtime = row.get('runtime') or {}
        outputs = row.get('outputs') or {}
        trajectory = outputs.get('trajectory') or {}
        return (
            0 if row.get('status') == 'MAP_VERIFIED' else (
                1 if row.get('status') == 'MAP_SAVED' else 2
            ),
            int(runtime.get('keypoints_too_few_count') or 0),
            int(runtime.get('lidar_delta_error_count') or 0),
            -int(trajectory.get('total_lines') or 0),
        )

    return sorted(diagnostics, key=score)[0]


def _canonical_key(key: str) -> str:
    aliases = {
        'voxel': 'voxel_size',
        'min': 'min_range',
        'max': 'max_range',
        'dd': 'double_downsample',
        'double': 'double_downsample',
        'init': 'initialization_phase',
        'initialization': 'initialization_phase',
    }
    return aliases.get(key, key)


def _required(values: dict[str, str], key: str) -> str:
    if key not in values:
        raise ValueError(f'case spec is missing {key}')
    return values[key]


def _parse_bool(value: str) -> bool:
    lowered = value.strip().lower()
    if lowered in ('true', '1', 'yes', 'on'):
        return True
    if lowered in ('false', '0', 'no', 'off'):
        return False
    raise ValueError(f'not a boolean: {value}')


def _safe_id(value: str) -> str:
    text = value.strip().lower().replace('.', 'p')
    text = re.sub(r'[^a-z0-9_+-]+', '_', text)
    text = text.replace('+', 'p').replace('-', 'm')
    return re.sub(r'_+', '_', text).strip('_') or 'case'


def _num_id(value: float) -> str:
    return f'{float(value):g}'.replace('-', 'm').replace('.', 'p')


def _bool_id(value: bool) -> str:
    return 'on' if value else 'off'


def _fmt_float(value: Any) -> str:
    if value is None:
        return ''
    try:
        return f'{float(value):.2f}'
    except Exception:
        return str(value)


def _fmt_seconds(value: Any) -> str:
    if value is None:
        return ''
    try:
        return f'{float(value):.1f}s'
    except Exception:
        return str(value)


def _fmt_value(value: Any) -> str:
    if value is None:
        return ''
    return str(value)
