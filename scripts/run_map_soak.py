#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Run the product map command repeatedly under a bounded soak contract."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import signal
import subprocess
import time
from typing import Callable


ROOT = Path(__file__).resolve().parents[1]
PROFILE_DURATIONS = {
    'one-hour': 3600.0,
    'eight-hour': 28800.0,
}
REPORT_NAME = 'soak_report.json'
RUN_MANIFEST_NAME = 'run_manifest.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/soak-report-v4.schema.json'
)
DEFAULT_TELEMETRY_INTERVAL_SECS = 30.0
PROCESS_SHUTDOWN_GRACE_SECS = 10.0
DROP_SIGNATURES = {
    'message_filter': re.compile(r'Message Filter.*dropp', re.IGNORECASE),
    'scan_drop': re.compile(r'dropp(?:ed|ing).*scan', re.IGNORECASE),
    'queue_overflow': re.compile(r'(?:queue|buffer).*(?:overflow|full)', re.IGNORECASE),
}
GNU_TIME_KEYS = {
    'elapsed': 'Elapsed (wall clock) time (h:mm:ss or m:ss)',
    'peak_rss': 'Maximum resident set size (kbytes)',
    'exit_status': 'Exit status',
}


class TerminationSignal(RuntimeError):
    """Represent a service termination delivered while an iteration runs."""

    def __init__(self, signum: int):
        self.signum = signum
        super().__init__(signal.Signals(signum).name)


def _raise_termination_signal(signum: int, _frame: object) -> None:
    raise TerminationSignal(signum)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def capture_machine_fingerprint() -> dict[str, object]:
    """Capture a stable non-secret identity and public capacity fields."""
    private_values = []
    for path in (
        Path('/etc/machine-id'),
        Path('/sys/class/dmi/id/product_uuid'),
        Path('/sys/class/dmi/id/board_serial'),
    ):
        try:
            value = path.read_text(encoding='utf-8').strip()
        except OSError:
            value = ''
        if value:
            private_values.append(value)

    cpu_fields = {}
    try:
        for line in Path('/proc/cpuinfo').read_text(
            encoding='utf-8',
            errors='replace',
        ).splitlines():
            if ':' not in line:
                continue
            name, value = line.split(':', 1)
            normalized_name = name.strip().lower()
            if normalized_name in ('model name', 'model', 'hardware'):
                cpu_fields.setdefault(normalized_name, value.strip())
    except OSError:
        pass
    cpu_model = next(
        (
            cpu_fields[name]
            for name in ('model name', 'model', 'hardware')
            if cpu_fields.get(name)
        ),
        '',
    )

    memory_total_kb = None
    try:
        for line in Path('/proc/meminfo').read_text(
            encoding='utf-8',
            errors='replace',
        ).splitlines():
            if line.startswith('MemTotal:'):
                memory_total_kb = int(line.split()[1])
                break
    except (OSError, ValueError, IndexError):
        pass

    os_release = ''
    try:
        for line in Path('/etc/os-release').read_text(
            encoding='utf-8',
            errors='replace',
        ).splitlines():
            if line.startswith('PRETTY_NAME='):
                os_release = line.split('=', 1)[1].strip().strip('"')
                break
    except OSError:
        pass

    hardware = {
        'architecture': platform.machine(),
        'cpu_model': cpu_model,
        'logical_cpu_count': os.cpu_count(),
        'memory_total_kb': memory_total_kb,
    }
    environment = {
        'kernel_release': platform.release(),
        'os_release': os_release,
    }
    identity_payload = {
        'private_identifiers': private_values,
        **hardware,
    }
    machine_id = hashlib.sha256(
        json.dumps(
            identity_payload,
            sort_keys=True,
            separators=(',', ':'),
        ).encode('utf-8')
    ).hexdigest()
    return {'machine_id': machine_id, **hardware, **environment}


def _git_state() -> dict[str, object]:
    try:
        commit = subprocess.run(
            ['git', 'rev-parse', 'HEAD'],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
        status = subprocess.run(
            ['git', 'status', '--porcelain', '--untracked-files=no'],
            cwd=ROOT,
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return {'git_commit': None, 'git_dirty': None}
    return {
        'git_commit': commit.stdout.strip() if commit.returncode == 0 else None,
        'git_dirty': (
            bool(status.stdout.strip()) if status.returncode == 0 else None
        ),
    }


def _harness_identity(cli: Path) -> dict[str, object]:
    version_path = ROOT / 'VERSION'
    if not version_path.is_file():
        raise ValueError(f'product VERSION not found: {version_path}')
    product_version = version_path.read_text(encoding='utf-8').strip()
    if not product_version:
        raise ValueError(f'product VERSION is empty: {version_path}')
    return {
        'product_version': product_version,
        **_git_state(),
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'runner_sha256': _sha256(Path(__file__).resolve()),
        'product_cli_sha256': _sha256(cli),
    }


def _load_iteration_provenance(
    run_dir: Path,
) -> tuple[dict[str, object], dict[str, object]]:
    manifest_path = run_dir / RUN_MANIFEST_NAME
    if not manifest_path.is_file():
        raise ValueError(
            f'successful iteration lacks {RUN_MANIFEST_NAME}: {run_dir}'
        )
    try:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(
            f'iteration manifest is not readable JSON: {manifest_path}: {exc}'
        ) from exc
    if not isinstance(manifest, dict):
        raise ValueError(f'iteration manifest root is not an object: {manifest_path}')
    if manifest.get('status') != 'succeeded':
        raise ValueError(
            'successful iteration manifest status is not succeeded: '
            f'{manifest.get("status")!r}'
        )
    input_identity = manifest.get('input')
    software_identity = manifest.get('software')
    if not isinstance(input_identity, dict) or not isinstance(
        software_identity,
        dict,
    ):
        raise ValueError(
            'successful iteration manifest lacks input/software identity'
        )
    return input_identity, software_identity


def _positive_finite(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a positive number') from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError('must be finite and greater than zero')
    return parsed


def _telemetry_interval(value: str) -> float:
    parsed = _positive_finite(value)
    if parsed > 60.0:
        raise argparse.ArgumentTypeError('must not exceed 60 seconds')
    return parsed


def _nonnegative_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a non-negative integer') from exc
    if parsed < 0:
        raise argparse.ArgumentTypeError('must be a non-negative integer')
    return parsed


def _nonempty(value: str) -> str:
    parsed = value.strip()
    if not parsed:
        raise argparse.ArgumentTypeError('must not be empty')
    return parsed


def _elapsed_seconds(value: str) -> float:
    fields = value.strip().split(':')
    if len(fields) == 2:
        return float(fields[0]) * 60.0 + float(fields[1])
    if len(fields) == 3:
        return (
            float(fields[0]) * 3600.0
            + float(fields[1]) * 60.0
            + float(fields[2])
        )
    raise ValueError(f'invalid GNU time elapsed value: {value!r}')


def parse_gnu_time(text: str) -> dict[str, float | int]:
    """Parse the stable fields used by the soak evidence contract."""
    fields: dict[str, str] = {}
    for line in text.splitlines():
        stripped = line.strip()
        for name, key in GNU_TIME_KEYS.items():
            prefix = f'{key}:'
            if stripped.startswith(prefix):
                fields[name] = stripped[len(prefix):].strip()
    missing = sorted(set(GNU_TIME_KEYS) - set(fields))
    if missing:
        raise ValueError(f'GNU time report lacks fields: {", ".join(missing)}')
    return {
        'wall_time_sec': _elapsed_seconds(fields['elapsed']),
        'peak_rss_mib': float(fields['peak_rss']) / 1024.0,
        'exit_code': int(fields['exit_status']),
    }


def count_dropped_inputs(text: str) -> dict[str, int]:
    """Count documented log signatures without treating silence as proof."""
    return {
        name: sum(1 for line in text.splitlines() if pattern.search(line))
        for name, pattern in DROP_SIGNATURES.items()
    }


def directory_size_bytes(path: Path) -> int:
    total = 0
    if not path.exists():
        return total
    for candidate in path.rglob('*'):
        if candidate.is_file():
            try:
                total += candidate.stat().st_size
            except FileNotFoundError:
                continue
    return total


def _write_report_atomic(output_root: Path, report: dict[str, object]) -> None:
    destination = output_root / REPORT_NAME
    temporary = output_root / f'.{REPORT_NAME}.tmp'
    try:
        temporary.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        os.replace(temporary, destination)
    except OSError:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


def evaluate_report(report: dict[str, object]) -> dict[str, bool]:
    thresholds = report['thresholds']
    metrics = report['metrics']
    return {
        'target_duration_reached': (
            metrics['wall_time_sec'] >= report['target_duration_sec']
        ),
        'all_iterations_succeeded': (
            metrics['iterations_completed'] > 0
            and metrics['iterations_failed'] == 0
        ),
        'peak_rss_within_budget': (
            metrics['peak_rss_mib'] <= thresholds['max_peak_rss_mib']
        ),
        'output_size_within_budget': (
            metrics['output_size_bytes'] <= thresholds['max_output_bytes']
        ),
        'dropped_inputs_within_budget': (
            metrics['dropped_inputs_total'] <= thresholds['max_dropped_inputs']
        ),
        'free_space_within_budget': (
            metrics['minimum_free_space_bytes'] is not None
            and metrics['minimum_free_space_bytes']
            >= thresholds['minimum_free_space_bytes']
        ),
        'iteration_duration_within_budget': (
            metrics['max_observed_iteration_sec']
            <= thresholds['max_iteration_sec']
        ),
        'provenance_recorded': (
            report['input_identity'] is not None
            and report['software_identity'] is not None
        ),
    }


def _resolve_cli(value: str | None) -> Path:
    if value:
        candidate = Path(value).expanduser().resolve()
    else:
        installed = shutil.which('lidarslam-map')
        candidate = Path(installed) if installed else ROOT / 'scripts/lidarslam'
    if not candidate.is_file():
        raise ValueError(f'product CLI not found: {candidate}')
    if not os.access(candidate, os.X_OK):
        raise ValueError(f'product CLI is not executable: {candidate}')
    return candidate


def _run_iteration(
    command: list[str],
    log_path: Path,
    time_path: Path,
    *,
    telemetry_interval_sec: float,
    max_iteration_sec: float,
    on_sample: Callable[[float], None],
    clock: Callable[[], float] = time.monotonic,
) -> tuple[int, dict[str, float | int], dict[str, int]]:
    gnu_time = Path('/usr/bin/time')
    if not gnu_time.is_file():
        raise ValueError('GNU /usr/bin/time is required for soak telemetry')
    environment = os.environ.copy()
    environment['LC_ALL'] = 'C'
    timed_command = [
        str(gnu_time),
        '--verbose',
        '--output',
        str(time_path),
        *command,
    ]
    with log_path.open('w', encoding='utf-8') as stream:
        process = subprocess.Popen(
            timed_command,
            stdout=stream,
            stderr=subprocess.STDOUT,
            env=environment,
            start_new_session=True,
        )
        previous_sigterm_handler = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGTERM, _raise_termination_signal)
        iteration_start = clock()
        last_sample_elapsed: float | None = None
        try:
            on_sample(0.0)
            last_sample_elapsed = 0.0
            while True:
                elapsed = clock() - iteration_start
                remaining = max_iteration_sec - elapsed
                if remaining <= 0:
                    on_sample(elapsed)
                    raise TimeoutError(
                        'iteration exceeded max_iteration_sec '
                        f'({max_iteration_sec:g})'
                    )
                try:
                    process.wait(timeout=min(telemetry_interval_sec, remaining))
                    break
                except subprocess.TimeoutExpired:
                    elapsed = clock() - iteration_start
                    on_sample(elapsed)
                    last_sample_elapsed = elapsed
                    if elapsed >= max_iteration_sec:
                        raise TimeoutError(
                            'iteration exceeded max_iteration_sec '
                            f'({max_iteration_sec:g})'
                        )
            final_elapsed = clock() - iteration_start
            if last_sample_elapsed is None or final_elapsed > last_sample_elapsed:
                on_sample(final_elapsed)
        except BaseException:
            _terminate_process_group(process)
            raise
        finally:
            signal.signal(signal.SIGTERM, previous_sigterm_handler)
    log_text = log_path.read_text(encoding='utf-8', errors='replace')
    metrics = parse_gnu_time(time_path.read_text(encoding='utf-8'))
    return process.returncode, metrics, count_dropped_inputs(log_text)


def _terminate_process_group(process: subprocess.Popen) -> None:
    """Stop the timed process group and reap its leader."""
    process_group_id = process.pid
    try:
        os.killpg(process_group_id, signal.SIGTERM)
    except ProcessLookupError:
        pass

    try:
        process.wait(timeout=PROCESS_SHUTDOWN_GRACE_SECS)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process_group_id, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def _initial_report(args: argparse.Namespace, cli: Path) -> dict[str, object]:
    return {
        'schema_version': 4,
        'schema_uri': SCHEMA_URI,
        'status': 'running',
        'last_error': None,
        'active_iteration_id': None,
        'profile': args.soak_profile,
        'target_duration_sec': PROFILE_DURATIONS[args.soak_profile],
        'telemetry_interval_sec': args.telemetry_interval_secs,
        'started_at': _utc_now(),
        'completed_at': None,
        'hardware_label': args.hardware_label,
        'hardware': capture_machine_fingerprint(),
        'harness': _harness_identity(cli),
        'bag_path': str(Path(args.bag).expanduser().resolve()),
        'map_profile': args.map_profile,
        'product_cli': str(cli),
        'input_identity': None,
        'software_identity': None,
        'thresholds': {
            'max_peak_rss_mib': args.max_peak_rss_mib,
            'max_output_bytes': math.ceil(args.max_output_gib * 1024**3),
            'max_dropped_inputs': args.max_dropped_inputs,
            'max_iteration_sec': args.max_iteration_secs,
            'minimum_free_space_bytes': math.ceil(
                args.min_free_space_gib * 1024**3
            ),
        },
        'metrics': {
            'wall_time_sec': 0.0,
            'iterations_completed': 0,
            'iterations_failed': 0,
            'peak_rss_mib': 0.0,
            'output_size_bytes': 0,
            'dropped_inputs_total': 0,
            'max_observed_iteration_sec': 0.0,
            'minimum_free_space_bytes': None,
        },
        'checks': {},
        'telemetry_samples': [],
        'iterations': [],
    }


def run_soak(
    args: argparse.Namespace,
    *,
    clock: Callable[[], float] = time.monotonic,
    run_iteration: Callable[
        ...,
        tuple[int, dict[str, float | int], dict[str, int]],
    ] = _run_iteration,
    load_iteration_provenance: Callable[
        [Path],
        tuple[dict[str, object], dict[str, object]],
    ] = _load_iteration_provenance,
) -> tuple[int, dict[str, object]]:
    """Execute iterations until the profile duration or first failed run."""
    cli = _resolve_cli(args.cli)
    bag = Path(args.bag).expanduser().resolve()
    if not (bag / 'metadata.yaml').is_file():
        raise ValueError(f'rosbag2 metadata.yaml not found under: {bag}')
    output_root = Path(args.output_root).expanduser().resolve()
    if output_root.exists():
        raise ValueError(f'soak output already exists: {output_root}')
    if run_iteration is _run_iteration and not Path('/usr/bin/time').is_file():
        raise ValueError('GNU /usr/bin/time is required for soak telemetry')
    output_root.mkdir(parents=True)
    logs_dir = output_root / 'logs'
    runs_dir = output_root / 'runs'
    logs_dir.mkdir()
    runs_dir.mkdir()

    report = _initial_report(args, cli)
    target = report['target_duration_sec']
    start = clock()
    _write_report_atomic(output_root, report)
    iteration_number = 0
    while clock() - start < target:
        iteration_number += 1
        iteration_id = f'iteration-{iteration_number:04d}'
        run_dir = runs_dir / iteration_id
        log_path = logs_dir / f'{iteration_id}.log'
        time_path = logs_dir / f'{iteration_id}.time'
        command = [
            str(cli),
            'run',
            str(bag),
            '--output-dir',
            str(run_dir),
            '--min-free-space-gib',
            str(args.min_free_space_gib),
        ]
        if args.map_profile:
            command.extend(['--profile', args.map_profile])
        iteration_started = _utc_now()
        report['active_iteration_id'] = iteration_id
        _write_report_atomic(output_root, report)
        iteration_base_elapsed = report['metrics']['wall_time_sec']

        def record_sample(iteration_elapsed_sec: float) -> None:
            free_bytes = shutil.disk_usage(output_root).free
            output_size = directory_size_bytes(runs_dir)
            soak_elapsed_sec = iteration_base_elapsed + iteration_elapsed_sec
            report['telemetry_samples'].append({
                'captured_at': _utc_now(),
                'iteration_id': iteration_id,
                'iteration_elapsed_sec': iteration_elapsed_sec,
                'soak_elapsed_sec': soak_elapsed_sec,
                'output_size_bytes': output_size,
                'free_space_bytes': free_bytes,
            })
            metrics = report['metrics']
            metrics['wall_time_sec'] = soak_elapsed_sec
            metrics['max_observed_iteration_sec'] = max(
                metrics['max_observed_iteration_sec'],
                iteration_elapsed_sec,
            )
            metrics['output_size_bytes'] = output_size
            previous_minimum = metrics['minimum_free_space_bytes']
            metrics['minimum_free_space_bytes'] = (
                free_bytes
                if previous_minimum is None
                else min(previous_minimum, free_bytes)
            )
            report['checks'] = evaluate_report(report)
            _write_report_atomic(output_root, report)
            if free_bytes < report['thresholds']['minimum_free_space_bytes']:
                raise RuntimeError(
                    f'{iteration_id} fell below minimum_free_space_bytes'
                )
            if output_size > report['thresholds']['max_output_bytes']:
                raise RuntimeError(
                    f'{iteration_id} failed output_size_within_budget'
                )

        try:
            return_code, resources, dropped = run_iteration(
                command,
                log_path,
                time_path,
                telemetry_interval_sec=args.telemetry_interval_secs,
                max_iteration_sec=args.max_iteration_secs,
                on_sample=record_sample,
            )
            if return_code == 0:
                input_identity, software_identity = load_iteration_provenance(
                    run_dir,
                )
                if (
                    report['input_identity'] is not None
                    and report['input_identity'] != input_identity
                ):
                    raise ValueError(
                        f'{iteration_id} input identity differs from prior iterations'
                    )
                if (
                    report['software_identity'] is not None
                    and report['software_identity'] != software_identity
                ):
                    raise ValueError(
                        f'{iteration_id} software identity differs from prior iterations'
                    )
                report['input_identity'] = input_identity
                report['software_identity'] = software_identity
        except (KeyboardInterrupt, TerminationSignal) as exc:
            if isinstance(exc, TerminationSignal):
                runner_exit_code = 128 + exc.signum
                reason = signal.Signals(exc.signum).name
            else:
                runner_exit_code = 130
                reason = 'SIGINT'
            report['metrics']['wall_time_sec'] = clock() - start
            report['metrics']['iterations_failed'] += 1
            report['status'] = 'interrupted'
            report['last_error'] = f'{iteration_id} interrupted by {reason}'
            report['active_iteration_id'] = None
            report['completed_at'] = _utc_now()
            report['checks'] = evaluate_report(report)
            _write_report_atomic(output_root, report)
            return runner_exit_code, report
        except (OSError, RuntimeError, ValueError) as exc:
            report['metrics']['wall_time_sec'] = clock() - start
            report['metrics']['iterations_failed'] += 1
            report['status'] = 'failed'
            report['last_error'] = (
                f'{iteration_id} telemetry or execution failed: {exc}'
            )
            report['active_iteration_id'] = None
            report['completed_at'] = _utc_now()
            report['checks'] = evaluate_report(report)
            _write_report_atomic(output_root, report)
            return 1, report
        elapsed = clock() - start
        output_size = directory_size_bytes(run_dir)
        free_bytes = shutil.disk_usage(output_root).free
        dropped_total = sum(dropped.values())
        iteration = {
            'id': iteration_id,
            'started_at': iteration_started,
            'completed_at': _utc_now(),
            'command': command,
            'command_display': shlex.join(command),
            'exit_code': return_code,
            'wall_time_sec': resources['wall_time_sec'],
            'peak_rss_mib': resources['peak_rss_mib'],
            'output_size_bytes': output_size,
            'free_space_bytes': free_bytes,
            'dropped_input_signatures': dropped,
            'log': str(log_path.relative_to(output_root)),
            'time_report': str(time_path.relative_to(output_root)),
        }
        report['iterations'].append(iteration)
        report['active_iteration_id'] = None
        metrics = report['metrics']
        metrics['wall_time_sec'] = elapsed
        metrics['iterations_completed'] += int(return_code == 0)
        metrics['iterations_failed'] += int(return_code != 0)
        metrics['peak_rss_mib'] = max(
            metrics['peak_rss_mib'],
            resources['peak_rss_mib'],
        )
        metrics['max_observed_iteration_sec'] = max(
            metrics['max_observed_iteration_sec'],
            resources['wall_time_sec'],
        )
        metrics['output_size_bytes'] = directory_size_bytes(runs_dir)
        metrics['dropped_inputs_total'] += dropped_total
        previous_minimum = metrics['minimum_free_space_bytes']
        metrics['minimum_free_space_bytes'] = (
            free_bytes
            if previous_minimum is None
            else min(previous_minimum, free_bytes)
        )
        report['checks'] = evaluate_report(report)
        _write_report_atomic(output_root, report)
        if return_code != 0:
            report['last_error'] = (
                f'{iteration_id} exited with code {return_code}'
            )
            break
        budget_checks = (
            'peak_rss_within_budget',
            'output_size_within_budget',
            'dropped_inputs_within_budget',
            'free_space_within_budget',
            'iteration_duration_within_budget',
        )
        failed_budget = next(
            (name for name in budget_checks if not report['checks'][name]),
            None,
        )
        if failed_budget is not None:
            report['last_error'] = f'{iteration_id} failed {failed_budget}'
            break

    report['completed_at'] = _utc_now()
    report['checks'] = evaluate_report(report)
    report['status'] = (
        'passed' if all(report['checks'].values()) else 'failed'
    )
    _write_report_atomic(output_root, report)
    return (0 if report['status'] == 'passed' else 1), report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', help='rosbag2 directory containing metadata.yaml')
    parser.add_argument('--output-root', required=True)
    parser.add_argument(
        '--soak-profile',
        choices=tuple(PROFILE_DURATIONS),
        required=True,
    )
    parser.add_argument(
        '--hardware-label',
        type=_nonempty,
        required=True,
        help='Stable operator-defined hardware identity recorded in evidence',
    )
    parser.add_argument('--map-profile')
    parser.add_argument('--cli', help='Product CLI path (default: lidarslam-map)')
    parser.add_argument(
        '--max-peak-rss-mib',
        type=_positive_finite,
        required=True,
    )
    parser.add_argument(
        '--max-output-gib',
        type=_positive_finite,
        required=True,
    )
    parser.add_argument(
        '--max-dropped-inputs',
        type=_nonnegative_int,
        required=True,
    )
    parser.add_argument(
        '--max-iteration-secs',
        type=_positive_finite,
        required=True,
        help=(
            'Terminate the timed process group when one complete map run '
            'exceeds this wall-time budget'
        ),
    )
    parser.add_argument(
        '--min-free-space-gib',
        type=_positive_finite,
        default=5.0,
    )
    parser.add_argument(
        '--telemetry-interval-secs',
        type=_telemetry_interval,
        default=DEFAULT_TELEMETRY_INTERVAL_SECS,
        help=(
            'Checkpoint free-space and output telemetry while an iteration '
            f'is running (default: {DEFAULT_TELEMETRY_INTERVAL_SECS:g}; '
            'maximum: 60 seconds).'
        ),
    )
    return parser.parse_args()


def main() -> int:
    try:
        exit_code, report = run_soak(parse_args())
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=os.sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
