#!/usr/bin/env python3
"""Select and optionally run public MID-360 dataset map candidates."""

from __future__ import annotations

import json
import os
import signal
import shlex
import shutil
import subprocess
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_DATASET_MAP_CANDIDATES_JSON = 'mid360_robot_public_dataset_map_candidates.json'
PUBLIC_DATASET_MAP_CANDIDATES_MARKDOWN = 'mid360_robot_public_dataset_map_candidates.md'
DEFAULT_MIN_FREE_BYTES = 5 * 1024 * 1024 * 1024
MAP_OUTPUT_COLLISION_PATHS = (
    'map_projector_info.yaml',
    'pointcloud_map',
    'map.pcd',
    'map_save.log',
)


@dataclass(frozen=True)
class PublicDatasetMapSelectionOptions:
    """Selection options for public dataset map candidates."""

    dataset_ids: tuple[str, ...] = ()
    allow_warn: bool = True
    limit: int = 0

    @property
    def allowed_statuses(self) -> tuple[str, ...]:
        return ('PASS', 'WARN') if self.allow_warn else ('PASS',)


@dataclass(frozen=True)
class PublicDatasetMapSafetyOptions:
    """Runtime and storage safety options for public dataset map runs."""

    min_free_bytes: int = DEFAULT_MIN_FREE_BYTES
    runtime_scale: float = 1.5
    output_size_ratio: float = 1.25
    allow_existing_map_output: bool = False


@dataclass(frozen=True)
class PublicDatasetMapRunOptions:
    """Execution options for selected public dataset map commands."""

    timeout_sec: int = 0


class PublicDatasetMapCandidateSelector:
    """Select runnable map candidates from a public dataset report."""

    def select(
        self,
        report: dict[str, Any],
        options: PublicDatasetMapSelectionOptions,
    ) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        """Return selected candidates and skipped rows."""
        selected: list[dict[str, Any]] = []
        skipped: list[dict[str, Any]] = []
        wanted_ids = set(options.dataset_ids)
        for row in report.get('datasets') or []:
            candidate = self._candidate_from_row(row)
            skip_reason = self._skip_reason(candidate, wanted_ids, options)
            if skip_reason:
                skipped.append({**candidate, 'skip_reason': skip_reason})
                continue
            selected.append(candidate)
            if options.limit > 0 and len(selected) >= options.limit:
                break
        return selected, skipped

    @staticmethod
    def _candidate_from_row(row: dict[str, Any]) -> dict[str, Any]:
        return {
            'dataset_id': row.get('dataset_id', ''),
            'title': row.get('title', ''),
            'source_status': str(row.get('status') or '').upper(),
            'ready_for_mid360_launch': bool(row.get('ready_for_mid360_launch')),
            'selected_bag_path': row.get('selected_bag_path', ''),
            'selected_topics': row.get('selected_topics') or {},
            'rates_hz': row.get('rates_hz') or {},
            'sampled_frames': row.get('sampled_frames') or {},
            'warning_count': len(row.get('warnings') or []),
            'warnings': row.get('warnings') or [],
            'output_dir': PublicDatasetMapCandidateSelector._output_dir_from_row(row),
            'command_shell': row.get('map_command_shell', ''),
        }

    @staticmethod
    def _skip_reason(
        candidate: dict[str, Any],
        wanted_ids: set[str],
        options: PublicDatasetMapSelectionOptions,
    ) -> str:
        dataset_id = str(candidate.get('dataset_id') or '')
        if wanted_ids and dataset_id not in wanted_ids:
            return 'dataset not selected'
        status = str(candidate.get('source_status') or '').upper()
        if status not in options.allowed_statuses:
            return f'status {status or "UNKNOWN"} is not allowed'
        if not candidate.get('ready_for_mid360_launch'):
            return 'run plan is not ready_for_mid360_launch'
        if not candidate.get('command_shell'):
            return 'map command is missing'
        if not candidate.get('selected_bag_path'):
            return 'selected bag path is missing'
        return ''

    @staticmethod
    def _output_dir_from_row(row: dict[str, Any]) -> str:
        artifact_paths = row.get('artifact_paths') or {}
        run_plan_json = artifact_paths.get('run_plan_json')
        if run_plan_json:
            return str(Path(str(run_plan_json)).expanduser().parent)
        return _command_option_value(str(row.get('map_command_shell') or ''), '--output-dir')


class PublicDatasetMapRunner:
    """Build manifests and optionally execute selected map commands."""

    def __init__(
        self,
        report_path: Path,
        output_dir: Path,
        selector: PublicDatasetMapCandidateSelector | None = None,
    ) -> None:
        self._report_path = report_path.expanduser().resolve()
        self._output_dir = output_dir.expanduser().resolve()
        self._selector = selector or PublicDatasetMapCandidateSelector()

    def build_manifest(
        self,
        options: PublicDatasetMapSelectionOptions,
        safety_options: PublicDatasetMapSafetyOptions | None = None,
        run_options: PublicDatasetMapRunOptions | None = None,
        run: bool = False,
    ) -> dict[str, Any]:
        """Build a candidate manifest, running commands only when requested."""
        report = self._load_report()
        candidates, skipped = self._selector.select(report, options)
        safety = safety_options or PublicDatasetMapSafetyOptions()
        candidates = [self._attach_safety(candidate, safety) for candidate in candidates]
        runnable = [
            candidate for candidate in candidates
            if (candidate.get('safety') or {}).get('can_run')
        ]
        blocked = [
            candidate for candidate in candidates
            if not (candidate.get('safety') or {}).get('can_run')
        ]
        run_config = run_options or PublicDatasetMapRunOptions()
        runs = self._run_candidates(runnable, run_config) if run else []
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': self._status(candidates, blocked, runs, run),
            'mode': 'RUN' if run else 'PLAN',
            'source_report_path': str(self._report_path),
            'output_dir': str(self._output_dir),
            'selection': {
                'dataset_ids': list(options.dataset_ids),
                'allow_warn': options.allow_warn,
                'allowed_statuses': list(options.allowed_statuses),
                'limit': options.limit,
            },
            'safety_options': {
                'min_free_bytes': safety.min_free_bytes,
                'runtime_scale': safety.runtime_scale,
                'output_size_ratio': safety.output_size_ratio,
                'allow_existing_map_output': safety.allow_existing_map_output,
            },
            'run_options': {
                'timeout_sec': run_config.timeout_sec,
            },
            'candidates': candidates,
            'blocked': blocked,
            'skipped': skipped,
            'runs': runs,
            'counts': {
                'candidates': len(candidates),
                'runnable': len(runnable),
                'blocked': len(blocked),
                'skipped': len(skipped),
                'run': len(runs),
                'failed': sum(1 for item in runs if item.get('returncode') != 0),
            },
        }

    def write_manifest(self, manifest: dict[str, Any]) -> dict[str, Path]:
        """Write candidate manifest JSON and Markdown."""
        self._output_dir.mkdir(parents=True, exist_ok=True)
        json_path = self._output_dir / PUBLIC_DATASET_MAP_CANDIDATES_JSON
        markdown_path = self._output_dir / PUBLIC_DATASET_MAP_CANDIDATES_MARKDOWN
        json_path.write_text(payload_to_json(manifest) + '\n', encoding='utf-8')
        markdown_path.write_text(render_map_candidates_markdown(manifest) + '\n', encoding='utf-8')
        return {'json': json_path, 'markdown': markdown_path}

    def _load_report(self) -> dict[str, Any]:
        return json.loads(self._report_path.read_text(encoding='utf-8'))

    def _run_candidates(
        self,
        candidates: list[dict[str, Any]],
        run_options: PublicDatasetMapRunOptions,
    ) -> list[dict[str, Any]]:
        results = []
        for candidate in candidates:
            command = shlex.split(str(candidate['command_shell']))
            results.append(
                _run_command(
                    dataset_id=str(candidate['dataset_id']),
                    command=command,
                    command_shell=str(candidate['command_shell']),
                    timeout_sec=run_options.timeout_sec,
                )
            )
        return results

    def _attach_safety(
        self,
        candidate: dict[str, Any],
        safety_options: PublicDatasetMapSafetyOptions,
    ) -> dict[str, Any]:
        safety = build_candidate_safety(candidate, safety_options)
        return {**candidate, 'safety': safety}

    @staticmethod
    def _status(
        candidates: list[dict[str, Any]],
        blocked: list[dict[str, Any]],
        runs: list[dict[str, Any]],
        run: bool,
    ) -> str:
        if not candidates:
            return 'EMPTY'
        if len(blocked) == len(candidates):
            return 'BLOCKED'
        if not run:
            if blocked:
                return 'PARTIAL'
            return 'READY'
        if any(item.get('returncode') != 0 for item in runs):
            return 'FAIL'
        if blocked:
            return 'PARTIAL'
        return 'PASS'


def build_candidate_safety(
    candidate: dict[str, Any],
    options: PublicDatasetMapSafetyOptions,
) -> dict[str, Any]:
    """Build runtime/storage safety checks for a selected candidate."""
    bag_path = Path(str(candidate.get('selected_bag_path') or '')).expanduser()
    output_dir = _candidate_output_dir(candidate)
    duration_sec = _bag_duration_sec(bag_path)
    bag_size_bytes = _path_size_bytes(bag_path)
    existing_outputs = _existing_map_outputs(output_dir)
    free_bytes = _free_bytes_for_path(output_dir)
    estimated_runtime_sec = _estimated_runtime_sec(duration_sec, options.runtime_scale)
    estimated_output_bytes = int(max(0, bag_size_bytes) * options.output_size_ratio)
    capacity_required_bytes = estimated_output_bytes + max(0, int(options.min_free_bytes))
    capacity_ok = free_bytes is not None and free_bytes >= capacity_required_bytes
    collision = bool(existing_outputs) and not options.allow_existing_map_output
    failures = []
    warnings = []

    if not bag_path.is_dir():
        failures.append(f'bag directory does not exist: {bag_path}')
    if duration_sec is None:
        warnings.append('bag duration is unknown')
    if output_dir is None:
        failures.append('output directory could not be resolved')
    if collision:
        failures.append('map output collision detected')
    if not capacity_ok:
        failures.append('free space is below estimated output plus reserve')
    if bag_size_bytes <= 0:
        warnings.append('bag size is zero or unavailable')

    status = 'FAIL' if failures else ('WARN' if warnings else 'OK')
    return {
        'status': status,
        'can_run': status != 'FAIL',
        'failures': failures,
        'warnings': warnings,
        'bag_duration_sec': duration_sec,
        'bag_size_bytes': bag_size_bytes,
        'estimated_runtime_sec': estimated_runtime_sec,
        'estimated_output_bytes': estimated_output_bytes,
        'min_free_bytes': int(options.min_free_bytes),
        'capacity_required_bytes': capacity_required_bytes,
        'free_bytes': free_bytes,
        'capacity_ok': capacity_ok,
        'output_dir': str(output_dir) if output_dir else '',
        'existing_map_outputs': [str(path) for path in existing_outputs],
        'map_output_collision': collision,
    }


def _run_command(
    dataset_id: str,
    command: list[str],
    command_shell: str,
    timeout_sec: int,
) -> dict[str, Any]:
    started = time.monotonic()
    process = subprocess.Popen(
        command,
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

    duration_sec = time.monotonic() - started
    return {
        'dataset_id': dataset_id,
        'command': command,
        'command_shell': command_shell,
        'returncode': 124 if timed_out else process.returncode,
        'timed_out': timed_out,
        'timeout_sec': timeout_sec,
        'duration_sec': duration_sec,
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


def render_map_candidates_markdown(manifest: dict[str, Any]) -> str:
    """Render public dataset map candidates as Markdown."""
    lines = [
        '# MID-360 Public Dataset Map Candidates',
        '',
        f"- status: `{manifest.get('status', '')}`",
        f"- mode: `{manifest.get('mode', '')}`",
        f"- created_at: `{manifest.get('created_at', '')}`",
        f"- source_report_path: `{manifest.get('source_report_path', '')}`",
        f"- output_dir: `{manifest.get('output_dir', '')}`",
        '',
        '## Candidates',
        '',
    ]
    candidates = manifest.get('candidates') or []
    if candidates:
        lines.extend([
            (
                '| Dataset | Source Status | Safety | PointCloud2 | IMU | PointCloud Hz | '
                'IMU Hz | Est Runtime | Est Output | Free | Warnings |'
            ),
            '| --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |',
        ])
        for candidate in candidates:
            topics = candidate.get('selected_topics') or {}
            rates = candidate.get('rates_hz') or {}
            safety = candidate.get('safety') or {}
            lines.append(
                '| '
                + ' | '.join([
                    f"`{candidate.get('dataset_id', '')}`",
                    f"`{candidate.get('source_status', '')}`",
                    f"`{safety.get('status', '')}`",
                    f"`{topics.get('pointcloud', '')}`",
                    f"`{topics.get('imu', '')}`",
                    _fmt_rate(rates.get('pointcloud')),
                    _fmt_rate(rates.get('imu')),
                    _fmt_seconds(safety.get('estimated_runtime_sec')),
                    _fmt_bytes(safety.get('estimated_output_bytes')),
                    _fmt_bytes(safety.get('free_bytes')),
                    str(candidate.get('warning_count', 0)),
                ])
                + ' |'
            )
        lines.extend(['', '### Commands', ''])
        for candidate in candidates:
            lines.extend([
                f"#### {candidate.get('dataset_id', '')}",
                '',
                '```bash',
                str(candidate.get('command_shell', '')),
                '```',
                '',
                _safety_lines(candidate.get('safety') or {}),
                '',
            ])
    else:
        lines.append('- none')

    blocked = manifest.get('blocked') or []
    lines.extend(['', '## Blocked', ''])
    if blocked:
        for item in blocked:
            safety = item.get('safety') or {}
            failures = '; '.join(str(value) for value in safety.get('failures') or [])
            lines.append(f"- `{item.get('dataset_id', '')}`: {failures}")
    else:
        lines.append('- none')

    skipped = manifest.get('skipped') or []
    lines.extend(['', '## Skipped', ''])
    if skipped:
        for item in skipped:
            lines.append(
                f"- `{item.get('dataset_id', '')}`: {item.get('skip_reason', '')}"
            )
    else:
        lines.append('- none')

    runs = manifest.get('runs') or []
    if runs:
        lines.extend(['', '## Runs', ''])
        for item in runs:
            timeout = ' timed out' if item.get('timed_out') else ''
            lines.append(
                f"- `{item.get('dataset_id', '')}` returncode `{item.get('returncode')}`"
                f"{timeout} duration `{_fmt_seconds(item.get('duration_sec'))}`"
            )
    return '\n'.join(lines)


def _fmt_rate(value: Any) -> str:
    try:
        return f'{float(value):.2f}'
    except Exception:
        return ''


def _fmt_seconds(value: Any) -> str:
    try:
        return f'{float(value):.0f}s'
    except Exception:
        return ''


def _fmt_bytes(value: Any) -> str:
    try:
        number = float(value)
    except Exception:
        return ''
    units = ('B', 'KiB', 'MiB', 'GiB', 'TiB')
    index = 0
    while number >= 1024.0 and index < len(units) - 1:
        number /= 1024.0
        index += 1
    return f'{number:.1f} {units[index]}'


def _safety_lines(safety: dict[str, Any]) -> str:
    failures = safety.get('failures') or []
    warnings = safety.get('warnings') or []
    lines = [
        f"- safety: `{safety.get('status', '')}`",
        f"- output_dir: `{safety.get('output_dir', '')}`",
        f"- bag_duration_sec: `{safety.get('bag_duration_sec')}`",
        f"- estimated_runtime_sec: `{safety.get('estimated_runtime_sec')}`",
        f"- estimated_output_bytes: `{safety.get('estimated_output_bytes')}`",
        f"- free_bytes: `{safety.get('free_bytes')}`",
    ]
    if failures:
        lines.append(f"- safety_failures: `{'; '.join(str(item) for item in failures)}`")
    if warnings:
        lines.append(f"- safety_warnings: `{'; '.join(str(item) for item in warnings)}`")
    return '\n'.join(lines)


def _candidate_output_dir(candidate: dict[str, Any]) -> Path | None:
    output_dir = str(candidate.get('output_dir') or '').strip()
    if output_dir:
        return Path(output_dir).expanduser().resolve()
    command_output_dir = _command_option_value(str(candidate.get('command_shell') or ''), '--output-dir')
    if command_output_dir:
        return Path(command_output_dir).expanduser().resolve()
    return None


def _command_option_value(command_shell: str, option: str) -> str:
    if not command_shell:
        return ''
    try:
        tokens = shlex.split(command_shell)
    except ValueError:
        return ''
    prefix = option + '='
    for index, token in enumerate(tokens):
        if token == option and index + 1 < len(tokens):
            return tokens[index + 1]
        if token.startswith(prefix):
            return token[len(prefix):]
    return ''


def _bag_duration_sec(bag_path: Path) -> float | None:
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        return None
    try:
        data = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
        info = data.get('rosbag2_bagfile_information') or {}
        duration = info.get('duration') or {}
        nanoseconds = duration.get('nanoseconds')
        if nanoseconds is None:
            return None
        return int(nanoseconds) / 1_000_000_000.0
    except Exception:
        return None


def _path_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    if path.is_file():
        return path.stat().st_size
    total = 0
    for item in path.rglob('*'):
        if item.is_file():
            total += item.stat().st_size
    return total


def _existing_map_outputs(output_dir: Path | None) -> list[Path]:
    if output_dir is None:
        return []
    return [
        (output_dir / relative).resolve()
        for relative in MAP_OUTPUT_COLLISION_PATHS
        if (output_dir / relative).exists()
    ]


def _free_bytes_for_path(path: Path | None) -> int | None:
    if path is None:
        return None
    target = path if path.exists() else path.parent
    while not target.exists() and target != target.parent:
        target = target.parent
    try:
        return int(shutil.disk_usage(target).free)
    except Exception:
        return None


def _estimated_runtime_sec(duration_sec: float | None, runtime_scale: float) -> float | None:
    if duration_sec is None:
        return None
    return max(0.0, float(duration_sec)) * max(0.1, float(runtime_scale)) + 90.0


if __name__ == "__main__":
    raise SystemExit("mid360_robot_public_dataset_map_runner is a library module; import it instead of running it directly.")
