#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Run repeated, provenance-checked FAST-LIVO2 benchmarks in Docker."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time
from typing import Any, Sequence

import yaml

try:
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        current_rival_source_closure_identity)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    source_root = Path(__file__).resolve().parents[1]
    if str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (  # type: ignore[no-redef]
        current_rival_source_closure_identity)

try:
    from lidarslam_benchmark_tools import package_root
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]


ROOT = package_root()
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'

# M6a10 profiles are selected by a persisted key, never by a transient
# phase adapter.  A profile may contain more than one historical
# attempt, so callers of the phase runner must bind the exact key explicitly.
M6A10_PROFILE_SCHEMA = 'explicit_profile_key_v1'
M6A10_PROFILE_KEYS = {
    'm6a10_fast_livo2_v2c_v7',
    'm6a10_fast_livo2_v2c_v8',
    'm6a10_fast_livo2_v2c_v9',
    'm6a10_fast_livo2_v2c',
}


def current_competitive_closure_identity(
        contract: dict[str, Any]) -> dict[str, Any] | None:
    """Require the current versioned rival recipe closure for production runs."""
    policy = contract.get('evidence_gate_v2', {}).get(
        'rival_source_closure', {})
    if not isinstance(policy, dict) or policy.get('required') is not True:
        return None
    return current_rival_source_closure_identity(
        {'competitive_slam_profile': contract}, root=ROOT)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _is_relative_to(path: Path, parent: Path) -> bool:
    """Python 3.8-compatible replacement for ``Path.is_relative_to``."""
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def benchmark_machine_fingerprint() -> dict[str, Any]:
    """Return a stable, non-secret fingerprint of the benchmark host."""
    private_values = []
    for path in (Path('/etc/machine-id'),
                 Path('/sys/class/dmi/id/product_uuid'),
                 Path('/sys/class/dmi/id/board_serial')):
        try:
            value = path.read_text().strip()
        except OSError:
            value = ''
        if value:
            private_values.append(value)

    cpu_model = ''
    try:
        for line in Path('/proc/cpuinfo').read_text().splitlines():
            if line.lower().startswith('model name') and ':' in line:
                cpu_model = line.split(':', 1)[1].strip()
                break
    except OSError:
        pass
    memory_total_kb = None
    try:
        for line in Path('/proc/meminfo').read_text().splitlines():
            if line.startswith('MemTotal:'):
                memory_total_kb = int(line.split()[1])
                break
    except (OSError, ValueError, IndexError):
        pass

    public = {
        'architecture': platform.machine(),
        'cpu_model': cpu_model,
        'logical_cpu_count': os.cpu_count(),
        'memory_total_kb': memory_total_kb,
    }
    identity_payload = {
        'private_identifiers': private_values,
        **public,
    }
    machine_id = hashlib.sha256(json.dumps(
        identity_payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return {'machine_id': machine_id, **public}


def command_output(command: list[str]) -> str:
    return subprocess.run(command, check=True, text=True,
                          capture_output=True).stdout.strip()


def atomic_json_replace(path: Path, payload: dict[str, Any]) -> None:
    """Atomically replace a host-only diagnostic JSON document."""
    path = path.resolve()
    if path.is_symlink():
        raise RuntimeError(f'diagnostic path is a symlink: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(
        f'.{path.name}.{os.getpid()}.{time.monotonic_ns()}.part')
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(payload, stream, sort_keys=True, separators=(',', ':'))
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def docker_container_snapshot(container_name: str) -> dict[str, Any]:
    """Read a best-effort Docker snapshot without changing container state."""
    try:
        completed = subprocess.run(
            ['docker', 'inspect', container_name], check=False, text=True,
            capture_output=True, timeout=2)
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'status': 'inspect_error', 'error': str(error)}
    if completed.returncode != 0:
        stderr = completed.stderr or ''
        status = ('not_found' if 'no such object' in stderr.lower() or
                  'no such container' in stderr.lower() else 'inspect_error')
        return {'status': status, 'returncode': completed.returncode,
                'stderr_sha256': hashlib.sha256(stderr.encode()).hexdigest()}
    try:
        inspected = json.loads(completed.stdout)
    except json.JSONDecodeError:
        return {'status': 'invalid_inspect_json', 'stdout_sha256': hashlib.sha256(
            completed.stdout.encode()).hexdigest()}
    if not isinstance(inspected, list) or not inspected:
        return {'status': 'empty_inspect'}
    item = inspected[0]
    state = item.get('State') or {}
    return {
        'status': 'present',
        'id': item.get('Id'),
        'name': item.get('Name'),
        'status_text': state.get('Status'),
        'running': state.get('Running'),
        'started_at': state.get('StartedAt'),
        'finished_at': state.get('FinishedAt'),
        'exit_code': state.get('ExitCode'),
        'oom_killed': state.get('OOMKilled'),
    }


def _memory_usage_bytes(value: str) -> int | None:
    """Parse the first Docker stats memory value without trusting display text."""
    match = re.match(r'^\s*([0-9]+(?:\.[0-9]+)?)\s*([kmgt]?i?b)\b',
                     value or '', re.IGNORECASE)
    if not match:
        return None
    units = {
        'b': 1, 'kb': 1000, 'kib': 1024, 'mb': 1000 ** 2,
        'mib': 1024 ** 2, 'gb': 1000 ** 3, 'gib': 1024 ** 3,
        'tb': 1000 ** 4, 'tib': 1024 ** 4,
    }
    multiplier = units.get(match.group(2).lower())
    return None if multiplier is None else int(float(match.group(1)) * multiplier)


def docker_stats_snapshot(container_name: str) -> dict[str, Any]:
    """Collect low-frequency diagnostic stats without changing container state."""
    try:
        completed = subprocess.run(
            ['docker', 'stats', '--no-stream', '--format', '{{json .}}',
             container_name], check=False, text=True, capture_output=True,
            timeout=3)
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'status': 'stats_error', 'error': str(error)}
    raw = completed.stdout.strip()
    if completed.returncode != 0:
        return {'status': 'stats_unavailable', 'returncode': completed.returncode,
                'stderr_sha256': hashlib.sha256(
                    completed.stderr.encode()).hexdigest()}
    try:
        document = json.loads(raw.splitlines()[0])
    except (IndexError, json.JSONDecodeError):
        return {'status': 'invalid_stats_json', 'stdout_sha256': hashlib.sha256(
            raw.encode()).hexdigest()}
    memory_usage = document.get('MemUsage', '')
    return {
        'status': 'present',
        'cpu_percent': document.get('CPUPerc'),
        'memory_usage': memory_usage,
        'memory_usage_bytes': _memory_usage_bytes(memory_usage),
        'pids': document.get('PIDs'),
        'raw_sha256': hashlib.sha256(raw.encode()).hexdigest(),
    }


def docker_top_snapshot(container_name: str) -> dict[str, Any]:
    """Capture the container process table for timeout diagnostics."""
    try:
        completed = subprocess.run(
            ['docker', 'top', container_name, '-eo',
             'pid,ppid,stat,etime,pcpu,pmem,args'], check=False, text=True,
            capture_output=True, timeout=3)
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'status': 'top_error', 'error': str(error)}
    raw = completed.stdout
    if completed.returncode != 0:
        return {'status': 'top_unavailable', 'returncode': completed.returncode,
                'stderr_sha256': hashlib.sha256(
                    completed.stderr.encode()).hexdigest()}
    return {
        'status': 'present',
        'stdout': raw,
        'stdout_sha256': hashlib.sha256(raw.encode()).hexdigest(),
    }


def docker_stop(container_name: str, grace_seconds: float) -> dict[str, Any]:
    """Ask Docker for a graceful stop and retain only hashed command output."""
    timeout = max(1, int(math.ceil(grace_seconds)))
    try:
        completed = subprocess.run(
            ['docker', 'stop', '--time', str(timeout), container_name],
            check=False, text=True, capture_output=True,
            timeout=max(timeout + 5, 10))
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'status': 'stop_error', 'error': str(error)}
    return {
        'status': 'requested', 'returncode': completed.returncode,
        'stdout_sha256': hashlib.sha256(completed.stdout.encode()).hexdigest(),
        'stderr_sha256': hashlib.sha256(completed.stderr.encode()).hexdigest(),
        'grace_seconds': grace_seconds,
    }


def docker_kill(container_name: str) -> dict[str, Any]:
    """Force-stop a still-running container after its TERM grace period."""
    try:
        completed = subprocess.run(
            ['docker', 'kill', container_name], check=False, text=True,
            capture_output=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'status': 'kill_error', 'error': str(error)}
    return {
        'status': 'requested', 'returncode': completed.returncode,
        'stdout_sha256': hashlib.sha256(completed.stdout.encode()).hexdigest(),
        'stderr_sha256': hashlib.sha256(completed.stderr.encode()).hexdigest(),
    }


def _watchdog_timeout_diagnostics(container_name: str) -> dict[str, Any]:
    """Collect inspect/stats/top before any watchdog stop action."""
    return {
        'captured_at': dt.datetime.now(dt.timezone.utc).isoformat(),
        'inspect': docker_container_snapshot(container_name),
        'stats': docker_stats_snapshot(container_name),
        'top': docker_top_snapshot(container_name),
    }


def remove_exited_container(container_name: str) -> dict[str, Any]:
    """Remove only an exited named container after its lifecycle is recorded."""
    before = docker_container_snapshot(container_name)
    if before.get('status') == 'not_found':
        return {'status': 'not_present', 'before': before}
    if before.get('status') != 'present' or before.get('running'):
        return {'status': 'not_removed_running_or_unverified', 'before': before}
    try:
        completed = subprocess.run(
            ['docker', 'rm', container_name], check=False, text=True,
            capture_output=True, timeout=10)
    except (OSError, subprocess.TimeoutExpired) as error:
        return {'status': 'remove_error', 'before': before, 'error': str(error)}
    after = docker_container_snapshot(container_name)
    return {
        'status': 'removed' if completed.returncode == 0 and
        after.get('status') == 'not_found' else 'remove_failed',
        'returncode': completed.returncode,
        'stdout_sha256': hashlib.sha256(completed.stdout.encode()).hexdigest(),
        'stderr_sha256': hashlib.sha256(completed.stderr.encode()).hexdigest(),
        'before': before,
        'after': after,
    }


def ensure_container_name_available(container_name: str) -> None:
    """Reject stale or unverifiable Docker names before launching a run."""
    snapshot = docker_container_snapshot(container_name)
    if snapshot.get('status') != 'not_found':
        raise RuntimeError(
            'FAST M6a10 stable container name is not verified absent: '
            f'{container_name} ({snapshot.get("status")})')


def supervise_container(process: Any, *, container_name: str,
                        cidfile: Path, lifecycle_path: Path,
                        interval_seconds: float = 2.0,
                        stats_interval_seconds: float = 5.0,
                        max_runtime_seconds: float | None = None,
                        term_grace_seconds: float = 15.0
                        ) -> tuple[int, dict[str, Any]]:
    """Poll a Docker client while retaining a reconnectable lifecycle trail.

    This supervisor never sends a signal.  If the host-side process is
    interrupted, the stable name/cidfile in the lifecycle document lets an
    independent supervisor reconnect with ``docker inspect`` rather than
    assuming that the container stopped.
    """
    if interval_seconds <= 0:
        raise ValueError('supervision interval must be positive')
    if stats_interval_seconds <= 0:
        raise ValueError('stats interval must be positive')
    if max_runtime_seconds is not None and max_runtime_seconds <= 0:
        raise ValueError('watchdog max runtime must be positive')
    if term_grace_seconds <= 0:
        raise ValueError('watchdog TERM grace must be positive')
    started = dt.datetime.now(dt.timezone.utc).isoformat()
    started_monotonic = time.monotonic()
    samples = 0
    stats_samples = 0
    max_memory_usage_bytes = 0
    next_stats_at = time.monotonic()
    last_snapshot: dict[str, Any] = {'status': 'not_sampled'}
    last_stats: dict[str, Any] = {'status': 'not_sampled'}
    lifecycle: dict[str, Any] = {
        'schema_version': 1,
        'contract_id': 'm6a10-fast-livo2-host-supervision-v1',
        'status': 'running',
        'container_name': container_name,
        'host_pid': getattr(process, 'pid', None),
        'cidfile': str(cidfile),
        'reconnect_command': ['docker', 'inspect', container_name],
        'started_at': started,
        'supervision_interval_seconds': interval_seconds,
        'stats_interval_seconds': stats_interval_seconds,
        'samples': samples,
        'stats_samples': stats_samples,
        'max_memory_usage_bytes': max_memory_usage_bytes,
        'last_snapshot': last_snapshot,
        'last_stats': last_stats,
        'watchdog': {
            'enabled': max_runtime_seconds is not None,
            'max_runtime_seconds': max_runtime_seconds,
            'term_grace_seconds': term_grace_seconds,
            'triggered': False,
            'action': None,
            'diagnostics': None,
        },
    }
    atomic_json_replace(lifecycle_path, lifecycle)
    try:
        while process.poll() is None:
            if (max_runtime_seconds is not None and
                    time.monotonic() - started_monotonic >= max_runtime_seconds):
                diagnostics = _watchdog_timeout_diagnostics(container_name)
                lifecycle.update({
                    'status': 'watchdog_timeout',
                    'watchdog': {
                        'enabled': True,
                        'max_runtime_seconds': max_runtime_seconds,
                        'term_grace_seconds': term_grace_seconds,
                        'triggered': True,
                        'action': 'capture_then_term_then_kill_if_running',
                        'diagnostics': diagnostics,
                    },
                    'watchdog_triggered_at': dt.datetime.now(
                        dt.timezone.utc).isoformat(),
                })
                atomic_json_replace(lifecycle_path, lifecycle)
                stop_result = docker_stop(container_name, term_grace_seconds)
                after_stop = docker_container_snapshot(container_name)
                kill_result = None
                if after_stop.get('status') == 'present' and after_stop.get('running'):
                    kill_result = docker_kill(container_name)
                lifecycle['watchdog'].update({
                    'stop': stop_result,
                    'after_stop': after_stop,
                    'kill': kill_result,
                    'after_kill': docker_container_snapshot(container_name)
                    if kill_result is not None else after_stop,
                })
                try:
                    try:
                        return_code = process.wait(
                            timeout=max(term_grace_seconds + 5.0, 10.0))
                    except TypeError:
                        # Small injected process surrogates may expose only
                        # wait(); production Popen supports timeout=.
                        return_code = process.wait()
                except subprocess.TimeoutExpired as error:
                    lifecycle['watchdog']['wait_timeout'] = repr(error)
                    try:
                        process.kill()
                    except (AttributeError, OSError):
                        pass
                    try:
                        return_code = process.wait(timeout=5.0)
                    except (AttributeError, OSError, subprocess.TimeoutExpired):
                        return_code = 124
                except BaseException as error:
                    return_code = 124
                    lifecycle['watchdog']['wait_error'] = repr(error)
                lifecycle.update({
                    'status': 'watchdog_fail_closed',
                    'returncode': return_code,
                    'finished_at': dt.datetime.now(dt.timezone.utc).isoformat(),
                    'last_snapshot': docker_container_snapshot(container_name),
                    'last_stats': docker_stats_snapshot(container_name),
                })
                atomic_json_replace(lifecycle_path, lifecycle)
                return return_code, lifecycle
            cid = None
            try:
                cid = cidfile.read_text(encoding='utf-8').strip() or None
            except OSError:
                pass
            last_snapshot = docker_container_snapshot(container_name)
            samples += 1
            now_monotonic = time.monotonic()
            if now_monotonic >= next_stats_at:
                last_stats = docker_stats_snapshot(container_name)
                stats_samples += 1
                memory_usage = last_stats.get('memory_usage_bytes')
                if isinstance(memory_usage, int):
                    max_memory_usage_bytes = max(max_memory_usage_bytes,
                                                 memory_usage)
                next_stats_at = now_monotonic + stats_interval_seconds
            lifecycle.update({
                'samples': samples,
                'stats_samples': stats_samples,
                'max_memory_usage_bytes': max_memory_usage_bytes,
                'cid': cid,
                'last_snapshot': last_snapshot,
                'last_stats': last_stats,
                'last_observed_at': dt.datetime.now(dt.timezone.utc).isoformat(),
            })
            atomic_json_replace(lifecycle_path, lifecycle)
            time.sleep(interval_seconds)
    except KeyboardInterrupt:
        lifecycle.update({
            'status': 'host_supervision_interrupted',
            'interrupted_at': dt.datetime.now(dt.timezone.utc).isoformat(),
            'last_snapshot': docker_container_snapshot(container_name),
            'last_stats': docker_stats_snapshot(container_name),
        })
        atomic_json_replace(lifecycle_path, lifecycle)
        raise
    return_code = process.wait()
    last_snapshot = docker_container_snapshot(container_name)
    lifecycle.update({
        'status': 'exited',
        'returncode': return_code,
        'finished_at': dt.datetime.now(dt.timezone.utc).isoformat(),
        'last_snapshot': last_snapshot,
        'last_stats': docker_stats_snapshot(container_name),
        'stats_samples': stats_samples,
        'max_memory_usage_bytes': max_memory_usage_bytes,
    })
    atomic_json_replace(lifecycle_path, lifecycle)
    return return_code, lifecycle


def atomic_json_create(path: Path, payload: dict[str, Any]) -> None:
    """Create a diagnostic JSON file exactly once, never replacing it."""
    path = path.resolve()
    if path.exists() or path.is_symlink():
        return
    part = path.with_name(path.name + '.part')
    if part.exists() or part.is_symlink():
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, sort_keys=True, separators=(',', ':')) + '\n')
    try:
        with part.open('x', encoding='utf-8') as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(str(part), str(path))
            part.unlink()
        except FileExistsError:
            # A parent supervisor won the race.  Keep its immutable run.json.
            part.unlink()
    except FileExistsError:
        return
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass


def _detached_watchdog_loop(*, container_name: str, cidfile: Path,
                            lifecycle_path: Path, run_json_path: Path,
                            command_sha256: str, input_path: str,
                            input_sha256: str, contract_id: str,
                            max_runtime_seconds: float,
                            term_grace_seconds: float,
                            interval_seconds: float,
                            stats_interval_seconds: float) -> int:
    """Independent watchdog that survives loss of the invoking agent.

    It intentionally owns only lifecycle/timeout diagnostics.  A live parent
    host runner writes the authoritative normal ``run.json``; this process
    creates that file only for a watchdog closure when the parent disappears.
    """
    started_at = dt.datetime.now(dt.timezone.utc).isoformat()
    started_monotonic = time.monotonic()
    lifecycle: dict[str, Any] = {
        'schema_version': 2,
        'contract_id': 'm6a10-fast-livo2-host-watchdog-v2',
        'status': 'watchdog_running',
        'container_name': container_name,
        'cidfile': str(cidfile),
        'command_sha256': command_sha256,
        'started_at': started_at,
        'max_runtime_seconds': max_runtime_seconds,
        'term_grace_seconds': term_grace_seconds,
        'supervision_interval_seconds': interval_seconds,
        'stats_interval_seconds': stats_interval_seconds,
        'samples': 0,
        'stats_samples': 0,
        'last_snapshot': None,
        'last_stats': None,
        'watchdog': {'triggered': False},
    }
    try:
        atomic_json_replace(lifecycle_path, lifecycle)
        next_stats = time.monotonic()
        while True:
            snapshot = docker_container_snapshot(container_name)
            lifecycle['samples'] += 1
            lifecycle['last_snapshot'] = snapshot
            if time.monotonic() >= next_stats:
                stats = docker_stats_snapshot(container_name)
                lifecycle['last_stats'] = stats
                lifecycle['stats_samples'] += 1
                next_stats = time.monotonic() + stats_interval_seconds
            lifecycle['last_observed_at'] = dt.datetime.now(
                dt.timezone.utc).isoformat()
            if (snapshot.get('status') == 'not_found' or
                    (snapshot.get('status') == 'present' and
                     snapshot.get('running') is False)):
                # Give the live parent a short hand-off window to publish its
                # authoritative report.  If the outer agent vanished, this
                # child still leaves an explicit fail-closed run.json.
                time.sleep(min(max(interval_seconds, 0.1), 5.0))
                lifecycle.update({
                    'status': 'container_exited',
                    'finished_at': dt.datetime.now(dt.timezone.utc).isoformat(),
                })
                atomic_json_replace(lifecycle_path, lifecycle)
                atomic_json_create(run_json_path, {
                    'schema_version': 2,
                    'contract_id': contract_id,
                    'system': 'fast_livo2',
                    'status': 'FAIL_CLOSED',
                    'failure': 'outer_host_runner_missing_before_run_receipt',
                    'input': {'path': input_path, 'sha256': input_sha256},
                    'execution': {
                        'container_name': container_name,
                        'command_sha256': command_sha256,
                        'container_exit_status': snapshot.get('exit_code'),
                        'host_lifecycle_path': str(lifecycle_path),
                        'host_lifecycle': lifecycle,
                        'gt_content_opened': False,
                        'scorer_invoked': False,
                    },
                })
                return int(snapshot.get('exit_code') or 0)
            if time.monotonic() - started_monotonic >= max_runtime_seconds:
                diagnostics = _watchdog_timeout_diagnostics(container_name)
                lifecycle.update({
                    'status': 'watchdog_timeout',
                    'watchdog': {
                        'triggered': True,
                        'action': 'capture_then_term_then_kill_if_running',
                        'diagnostics': diagnostics,
                    },
                    'watchdog_triggered_at': dt.datetime.now(
                        dt.timezone.utc).isoformat(),
                })
                atomic_json_replace(lifecycle_path, lifecycle)
                stop = docker_stop(container_name, term_grace_seconds)
                after_stop = docker_container_snapshot(container_name)
                kill = None
                if after_stop.get('status') == 'present' and after_stop.get('running'):
                    kill = docker_kill(container_name)
                final_snapshot = docker_container_snapshot(container_name)
                lifecycle['watchdog'].update({
                    'stop': stop, 'after_stop': after_stop, 'kill': kill,
                    'after_kill': final_snapshot,
                })
                lifecycle.update({
                    'status': 'watchdog_fail_closed',
                    'finished_at': dt.datetime.now(dt.timezone.utc).isoformat(),
                    'last_snapshot': final_snapshot,
                })
                atomic_json_replace(lifecycle_path, lifecycle)
                atomic_json_create(run_json_path, {
                    'schema_version': 2,
                    'contract_id': contract_id,
                    'system': 'fast_livo2',
                    'status': 'FAIL_CLOSED',
                    'failure': 'global_watchdog_timeout',
                    'input': {'path': input_path, 'sha256': input_sha256},
                    'execution': {
                        'container_name': container_name,
                        'command_sha256': command_sha256,
                        'container_exit_status': final_snapshot.get('exit_code'),
                        'host_lifecycle_path': str(lifecycle_path),
                        'host_lifecycle': lifecycle,
                        'gt_content_opened': False,
                        'scorer_invoked': False,
                    },
                })
                return 124
            atomic_json_replace(lifecycle_path, lifecycle)
            time.sleep(interval_seconds)
    except BaseException as error:
        # A diagnostic writer must not leave an unhandled exception that hides
        # the container.  The caller's own supervision can reconnect by name.
        lifecycle.update({'status': 'watchdog_exception', 'error': repr(error)})
        try:
            atomic_json_replace(lifecycle_path, lifecycle)
        except (OSError, RuntimeError):
            pass
        return 125


def _start_detached_watchdog(*, container_name: str, cidfile: Path,
                             lifecycle_path: Path, run_json_path: Path,
                             command_sha256: str, input_path: str,
                             input_sha256: str, contract_id: str,
                             max_runtime_seconds: float,
                             term_grace_seconds: float,
                             interval_seconds: float,
                             stats_interval_seconds: float,
                             log_path: Path) -> Any:
    """Launch the independent watchdog in a new session."""
    command = [
        sys.executable, str(Path(__file__).resolve()), '--watchdog-child',
        '--watchdog-container-name', container_name,
        '--watchdog-cidfile', str(cidfile),
        '--watchdog-lifecycle', str(lifecycle_path),
        '--watchdog-run-json', str(run_json_path),
        '--watchdog-command-sha256', command_sha256,
        '--watchdog-input-path', input_path,
        '--watchdog-input-sha256', input_sha256,
        '--watchdog-contract-id', contract_id,
        '--watchdog-max-runtime', str(max_runtime_seconds),
        '--watchdog-term-grace', str(term_grace_seconds),
        '--watchdog-interval', str(interval_seconds),
        '--watchdog-stats-interval', str(stats_interval_seconds),
    ]
    stream = log_path.open('x', encoding='utf-8')
    try:
        process = subprocess.Popen(
            command, stdin=subprocess.DEVNULL, stdout=stream, stderr=stream,
            close_fds=True, start_new_session=True)
        return process
    except BaseException:
        raise
    finally:
        stream.close()


def read_int(path: Path) -> int | None:
    try:
        return int(path.read_text().strip())
    except (FileNotFoundError, ValueError):
        return None


def parse_time_report(path: Path) -> dict[str, float | int]:
    if not path.exists():
        return {}
    text = path.read_text(errors='replace')
    result: dict[str, float | int] = {}
    rss = re.search(r'Maximum resident set size \(kbytes\):\s*(\d+)', text)
    elapsed = re.search(
        r'Elapsed \(wall clock\) time \(h:mm:ss or m:ss\):\s*([0-9:.]+)', text)
    if rss:
        result['peak_rss_kb'] = int(rss.group(1))
        result['peak_rss_mb'] = int(rss.group(1)) / 1024.0
    if elapsed:
        parts = [float(value) for value in elapsed.group(1).split(':')]
        result['wall_seconds'] = sum(
            value * 60 ** index for index, value in enumerate(reversed(parts)))
    return result


def parse_bag_bounds(path: Path) -> tuple[float | None, float | None]:
    if not path.exists():
        return None, None
    document = yaml.safe_load(path.read_text()) or {}
    try:
        start = float(document['start'])
        duration = float(document['duration'])
    except (KeyError, TypeError, ValueError):
        return None, None
    return start, start + duration


def processing_rtf_upper_bound(replay_wall_seconds: float | None,
                               drain_seconds: float,
                               sensor_duration: float | None) -> float | None:
    """Conservative processing bound from accelerated replay plus drain.

    This is only evidence when the accelerated run retains the baseline pose
    count and trajectory accuracy; the scorer records it but the gate checker
    is responsible for that cross-run validation.
    """
    if not replay_wall_seconds or not sensor_duration or sensor_duration <= 0.0:
        return None
    return (replay_wall_seconds + max(0.0, drain_seconds)) / sensor_duration


def first_present(row: dict[str, str], names: list[str]) -> float:
    for name in names:
        value = row.get(name)
        if value not in (None, ''):
            return float(value)
    raise KeyError(', '.join(names))


def odometry_csv_to_tum(source: Path, destination: Path) -> dict[str, Any]:
    """Convert ROS1 ``rostopic echo -p`` nav_msgs/Odometry output to TUM."""
    count, malformed = 0, 0
    first_stamp = last_stamp = None
    with destination.open('w', encoding='utf-8') as output:
        if not source.exists():
            return {'samples': 0, 'malformed_rows': 0,
                    'first_stamp': None, 'last_stamp': None}
        with source.open(newline='', errors='replace') as stream:
            for row in csv.DictReader(stream):
                try:
                    stamp = first_present(row, [
                        'field.header.stamp', 'field.header.stamp.secs'])
                    if row.get('field.header.stamp.nsecs'):
                        stamp += float(row['field.header.stamp.nsecs']) * 1e-9
                    elif stamp > 1.0e12:
                        # ROS1 ``rostopic echo -p`` serializes a Time field as
                        # one integer count of nanoseconds on Noetic.
                        stamp *= 1e-9
                    values = [first_present(row, [name]) for name in (
                        'field.pose.pose.position.x', 'field.pose.pose.position.y',
                        'field.pose.pose.position.z', 'field.pose.pose.orientation.x',
                        'field.pose.pose.orientation.y', 'field.pose.pose.orientation.z',
                        'field.pose.pose.orientation.w')]
                except (KeyError, TypeError, ValueError):
                    malformed += 1
                    continue
                output.write(f'{stamp:.9f} ' + ' '.join(
                    f'{value:.12g}' for value in values) + '\n')
                first_stamp = stamp if first_stamp is None else first_stamp
                last_stamp, count = stamp, count + 1
    return {'samples': count, 'malformed_rows': malformed,
            'first_stamp': first_stamp, 'last_stamp': last_stamp}


def git_state(source: Path) -> dict[str, Any]:
    return {
        'revision': command_output(['git', '-C', str(source), 'rev-parse', 'HEAD']),
        'tracked_dirty': bool(command_output([
            'git', '-C', str(source), 'status', '--porcelain',
            '--untracked-files=no'])),
    }


def load_contract(profile_path: Path) -> dict[str, Any]:
    return yaml.safe_load(profile_path.read_text())['competitive_slam_profile']


def _default_m6a10_profile_key(
        profile_path: Path, contract: dict[str, Any]) -> str:
    """Infer a key only when the profile has one unambiguous M6a10 phase."""
    candidates = sorted(
        key for key, value in contract.items()
        if key in M6A10_PROFILE_KEYS and isinstance(value, dict))
    if len(candidates) == 1:
        return candidates[0]
    # Legacy v2c profiles predate the explicit schema and contain one of the
    # two old names.  Keep those files runnable, while refusing ambiguity.
    basename = profile_path.name
    suffix_key = {
        '_v7_formal.yaml': 'm6a10_fast_livo2_v2c_v7',
        '_v8_formal.yaml': 'm6a10_fast_livo2_v2c_v8',
        '_v9_formal.yaml': 'm6a10_fast_livo2_v2c_v9',
    }
    for suffix, key in suffix_key.items():
        if basename.endswith(suffix) and key in contract:
            return key
    raise ValueError(
        'M6a10 profile selection is ambiguous; pass --m6a10-profile-key '
        'with the persisted contract key')


def load_m6a10_fast_contract(
        profile_path: Path, profile_key: str | None = None) -> dict[str, Any]:
    """Load one persisted M6a10 phase by explicit profile key."""
    contract = load_contract(profile_path)
    selected_key = profile_key or _default_m6a10_profile_key(profile_path, contract)
    if selected_key not in M6A10_PROFILE_KEYS:
        raise ValueError(f'unsupported M6a10 profile key: {selected_key}')
    phase = contract.get(selected_key)
    if not isinstance(phase, dict):
        raise ValueError(f'profile lacks selected M6a10 key: {selected_key}')
    declared_key = phase.get('profile_key')
    if declared_key is not None and declared_key != selected_key:
        raise ValueError(
            f'M6a10 profile key declaration mismatch: {selected_key} != {declared_key}')
    if phase.get('schema_version', 1) >= 2 and declared_key != selected_key:
        raise ValueError(
            'M6a10 schema v2 requires a persisted profile_key field')
    if phase.get('status') not in {
            'preregistered_not_built', 'preregistered_not_executed',
            'build_passed_not_executed'}:
        raise ValueError(
            'FAST M6a10 phase is not preregistered/build_passed_not_executed')
    if phase.get('result') is not None:
        raise ValueError('FAST M6a10 result must remain null before execution')
    if phase.get('contract_id') not in {
            'm6a10-v2c-fast-livo2-single-inflight-v1',
            'm6a10-v2c-fast-livo2-single-inflight-v2-watchdog-v1',
            'm6a10-v2c-fast-livo2-service-queue-v8',
            'm6a10-v2c-fast-livo2-wallrate-v9'}:
        raise ValueError('FAST M6a10 contract id mismatch')
    feeder = phase.get('feeder', {})
    if feeder.get('path') != 'scripts/fast_livo2_m6a10_feeder.py':
        raise ValueError('FAST M6a10 feeder path is not pinned')
    if not isinstance(feeder.get('sha256'), str) or len(feeder['sha256']) != 64:
        raise ValueError('FAST M6a10 feeder hash is not pinned')
    watchdog = phase.get('watchdog')
    if not isinstance(watchdog, dict):
        raise ValueError('FAST M6a10 global watchdog is not preregistered')
    for name in ('max_runtime_seconds', 'term_grace_seconds',
                 'poll_interval_seconds', 'stats_interval_seconds'):
        value = watchdog.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)) or value <= 0:
            raise ValueError(f'FAST M6a10 watchdog field is invalid: {name}')
    return phase


def build_m6a10_fast_command(
        args: argparse.Namespace, phase: dict[str, Any], run_dir: Path,
        bag: Path, profile_key: str | None = None) -> list[str]:
    """Build the input-only, network-isolated v2 command.

    This function is pure command construction.  It never mounts the asset
    parent (and therefore cannot expose a sibling GT tree); the caller must
    validate the immutable bag identity before invoking it.
    """
    input_path = '/input/raw_input.bag'
    expected = phase['consumer']['expected_topic_counts']
    execution = phase['execution']
    container_name = 'm6a10-fast-' + re.sub(
        r'[^a-zA-Z0-9_.-]', '-', run_dir.name)
    cidfile = run_dir / 'container.cid'
    # Use the inspected immutable image ID for execution.  The human-readable
    # tag remains metadata only and is used by the caller during inspection.
    image = execution.get('image_digest') or execution['image_tag']
    watchdog = phase.get('watchdog')
    command = [
        'docker', 'run', '--pull=never', '--init', '--name',
        container_name, '--cidfile', str(cidfile), '--network', 'none',
        '--read-only', '--tmpfs', '/tmp:rw,noexec,nosuid,size=1024m',
        '--shm-size', '512m',
        '-e', 'ROS_MASTER_URI=http://127.0.0.1:11311',
        '-e', 'ROS_IP=127.0.0.1', '-e', 'ROS_HOSTNAME=127.0.0.1',
        '-e', 'ROS_DOMAIN_ID=229',
        '-e', 'OMP_NUM_THREADS=8', '-e', 'OPENBLAS_NUM_THREADS=8',
        '-e', 'MKL_NUM_THREADS=8', '-e', 'TBB_NUM_THREADS=8',
        '-e', 'M6A10_PHASE_CONTRACT_VERSION=m6a10-online-compute-v2',
        '-e', f'M6A10_PROFILE_KEY={profile_key or phase.get("profile_key", "")}',
        '-e', f'M6A10_PHASE_MODE={args.phase_mode}',
        '-e', f'M6A10_DRAIN_TIMEOUT_SECONDS={phase["phase"]["drain_timeout_seconds"]}',
        '-e', 'M6A10_SKIP_MAP_SAVE=1', '-e', 'SAVE_MAP=0',
        '-e', f'M6A10_FAST_EXPECTED_MESSAGES={phase["consumer"]["expected_messages"]}',
        '-e', f'M6A10_FAST_EXPECTED_LIDAR_MESSAGES={expected["lidar"]}',
        '-e', f'M6A10_FAST_EXPECTED_IMU_MESSAGES={expected["imu"]}',
        '-e', f'M6A10_FAST_EXPECTED_IMAGE_MESSAGES={expected["image"]}',
        '-e', 'M6A10_FAST_MAX_CALLBACK_LATENCY_SECONDS='
        f'{phase["consumer"]["maximum_callback_latency_seconds"]}',
        '-e', f'M6A10_FAST_MAX_BACKLOG_MESSAGES={phase["consumer"]["maximum_backlog_messages"]}',
        '-e', f'M6A10_FAST_FEEDER_SHA256={phase["feeder"]["sha256"]}',
        '-e', 'M6A10_FAST_PROGRESS_PATH=/out/feeder_progress.json',
        '-e', 'M6A10_FAST_PROGRESS_EVERY=100',
        '-e', 'M6A10_FAST_QUEUE_CAPACITY_MESSAGES=1',
        '-e', 'FAST_PROFILE=ntu_viral', '-e', f'BAG_PATH={input_path}',
        '-e', 'OUT_DIR=/out', '-e', 'LIDAR_TOPIC=/os1_cloud_node1/points',
        '-e', 'IMU_TOPIC=/imu/imu', '-e', 'IMAGE_TOPIC=/left/image_raw',
        '-e', 'M6A10_CONSUMER_EVIDENCE=/out/consumer_evidence.json',
        '-v', f'{bag}:{input_path}:ro', '-v', f'{ROOT}:/runner:ro',
        '-v', f'{run_dir}:/out:rw', '--entrypoint', '/bin/bash', image,
        '/runner/scripts/fast_livo2_container_run.sh']
    if isinstance(watchdog, dict) and watchdog.get('max_runtime_seconds') is not None:
        insert_at = command.index('M6A10_DRAIN_TIMEOUT_SECONDS=' + str(
            phase['phase']['drain_timeout_seconds'])) + 1
        command[insert_at:insert_at] = [
            '-e', f'M6A10_GLOBAL_WATCHDOG_SECONDS={watchdog["max_runtime_seconds"]}']
    return command


def validate_m6a10_image_identity(phase: dict[str, Any]) -> dict[str, Any]:
    """Fail closed unless the local image carries the v2c source labels."""
    execution = phase['execution']
    raw = command_output([
        'docker', 'image', 'inspect', execution['image_tag'], '--format', '{{json .}}'])
    try:
        document = json.loads(raw)
    except json.JSONDecodeError as error:
        raise RuntimeError('FAST M6a10 image inspect was not JSON') from error
    observed_id = document.get('Id')
    if observed_id != execution['image_digest']:
        raise RuntimeError(
            f'FAST M6a10 image id mismatch expected={execution["image_digest"]} '
            f'actual={observed_id}')
    labels = document.get('Config', {}).get('Labels') or {}
    required = {
        'benchmark.fast_livo2.m6a10_patch_sha256': phase['image_labels']['patch_sha256'],
        'benchmark.fast_livo2.m6a10_feeder_sha256': phase['feeder']['sha256'],
        'benchmark.fast_livo2.m6a10_runner_sha256': phase['runner']['sha256'],
        'benchmark.fast_livo2.m6a10_consumer_contract': phase['contract_id'],
        'benchmark.fast_livo2.m6a10_queue_overflow_capability':
            'single_inflight_exact_count_observed',
    }
    optional_label_bindings = {
        'variant': 'benchmark.fast_livo2.m6a10_variant',
        'base_patch_sha256': 'benchmark.fast_livo2.m6a10_base_patch_sha256',
        'v8_delta_patch_sha256': 'benchmark.fast_livo2.m6a10_v8_delta_patch_sha256',
        'v9_delta_patch_sha256': 'benchmark.fast_livo2.m6a10_v9_delta_patch_sha256',
        'service_handshake_contract':
            'benchmark.fast_livo2.m6a10_service_handshake_contract',
    }
    for field, label in optional_label_bindings.items():
        expected = phase.get('image_labels', {}).get(field)
        if expected is not None:
            required[label] = expected
    watchdog = phase.get('watchdog')
    if isinstance(watchdog, dict) and watchdog.get('contract_id'):
        required['benchmark.fast_livo2.m6a10_watchdog_contract'] = watchdog[
            'contract_id']
    for key, expected in required.items():
        if labels.get(key) != expected:
            raise RuntimeError(f'FAST M6a10 image label mismatch: {key}')
    return {'id': observed_id, 'labels': labels}


def run_m6a10_phase(args: argparse.Namespace) -> int:
    full_contract = load_contract(args.profile)
    closure_identity = current_competitive_closure_identity(full_contract)
    requested_profile_key = getattr(args, 'm6a10_profile_key', None)
    phase = (load_m6a10_fast_contract(args.profile, requested_profile_key)
             if requested_profile_key is not None else
             load_m6a10_fast_contract(args.profile))
    selected_profile_key = requested_profile_key or phase.get('profile_key')
    input_contract = phase['input']
    bag = args.bag.resolve()
    configured = Path(input_contract['path']).resolve()
    if bag != configured:
        raise ValueError(f'FAST M6a10 input path mismatch: {bag} != {configured}')
    if not bag.is_file():
        raise ValueError(f'FAST M6a10 input is not a file: {bag}')
    actual_hash = sha256(bag)
    if actual_hash != input_contract['sha256']:
        raise ValueError('FAST M6a10 input SHA256 differs from preregistration')
    if bag.stat().st_size != input_contract['bytes']:
        raise ValueError('FAST M6a10 input size differs from preregistration')
    if not phase['safety']['ground_truth_mount_exposed'] is False:
        raise ValueError('FAST M6a10 safety contract permits a GT mount')
    if phase['consumer']['queue_overflow_observable'] is not True:
        raise RuntimeError(
            'FAST M6a10 remains fail-closed: sequential queue-drop capability '
            'is not preregistered')
    if (args.phase_mode == 'unpaced_ack' and
            phase['consumer']['ack_backpressure_verified'] is not True):
        raise RuntimeError(
            'FAST M6a10 remains fail-closed: callback acknowledgement '
            'mechanism is not preregistered')
    image_digest = phase['execution'].get('image_digest')
    if not isinstance(image_digest, str) or not image_digest.startswith('sha256:'):
        raise RuntimeError(
            'FAST M6a10 image is not built and labelled; execution remains fail-closed')
    feeder_path = ROOT / phase['feeder']['path']
    if sha256(feeder_path) != phase['feeder']['sha256']:
        raise RuntimeError('FAST M6a10 feeder hash differs from preregistration')
    runner_contract = phase.get('runner') or {}
    runner_path = ROOT / runner_contract.get('path', 'scripts/run_fast_livo2_benchmark.py')
    runner_hash = runner_contract.get('sha256')
    if not isinstance(runner_hash, str) or len(runner_hash) != 64:
        raise RuntimeError('FAST M6a10 runner hash is not pinned')
    if sha256(runner_path) != runner_hash:
        raise RuntimeError('FAST M6a10 runner hash differs from preregistration')
    image_identity = validate_m6a10_image_identity(phase)
    if not args.output.exists():
        args.output.mkdir(parents=True)
    run_dir = args.output / 'run_01'
    if run_dir.exists():
        raise ValueError(f'FAST M6a10 output already exists: {run_dir}')
    run_dir.mkdir()
    lifecycle_path = run_dir / 'host_lifecycle.json'
    cidfile = run_dir / 'container.cid'
    if lifecycle_path.exists() or lifecycle_path.is_symlink() or \
            cidfile.exists() or cidfile.is_symlink():
        raise ValueError('FAST M6a10 lifecycle artifacts already exist')
    command = build_m6a10_fast_command(
        args, phase, run_dir, bag, profile_key=selected_profile_key)
    container_name = next(
        command[index + 1] for index, value in enumerate(command)
        if value == '--name')
    ensure_container_name_available(container_name)
    started = dt.datetime.now(dt.timezone.utc)
    watchdog_process = None
    with (run_dir / 'container_stdout.log').open('w') as stdout, \
            (run_dir / 'container_stderr.log').open('w') as stderr:
        process = subprocess.Popen(command, stdout=stdout, stderr=stderr)
        watchdog = phase['watchdog']
        supervision_interval = float(watchdog['poll_interval_seconds'])
        stats_interval = float(watchdog['stats_interval_seconds'])
        # The detached child keeps diagnostics alive if the outer host agent
        # disappears.  Tests may disable it explicitly without changing the
        # production default.
        if os.environ.get('M6A10_FAST_DETACHED_WATCHDOG', '1') != '0':
            command_sha = hashlib.sha256(json.dumps(
                command, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
            try:
                watchdog_process = _start_detached_watchdog(
                    container_name=container_name, cidfile=cidfile,
                    lifecycle_path=lifecycle_path, run_json_path=run_dir / 'run.json',
                    command_sha256=command_sha,
                    input_path=str(bag), input_sha256=actual_hash,
                    contract_id=phase['contract_id'],
                    max_runtime_seconds=float(watchdog['max_runtime_seconds']),
                    term_grace_seconds=float(watchdog['term_grace_seconds']),
                    interval_seconds=supervision_interval,
                    stats_interval_seconds=stats_interval,
                    log_path=run_dir / 'watchdog.log')
            except BaseException as error:
                stop = docker_stop(container_name, float(watchdog['term_grace_seconds']))
                after = docker_container_snapshot(container_name)
                if after.get('status') == 'present' and after.get('running'):
                    docker_kill(container_name)
                atomic_json_replace(lifecycle_path, {
                    'schema_version': 2,
                    'contract_id': 'm6a10-fast-livo2-host-watchdog-v2',
                    'status': 'watchdog_start_failure',
                    'container_name': container_name,
                    'watchdog': {'triggered': True, 'action': 'stop_after_start_failure',
                                 'stop': stop, 'after': after, 'error': repr(error)},
                })
                atomic_json_create(run_dir / 'run.json', {
                    'schema_version': 2, 'contract_id': phase['contract_id'],
                    'system': 'fast_livo2', 'status': 'FAIL_CLOSED',
                    'failure': 'watchdog_start_failure',
                    'execution': {'container_name': container_name,
                                  'gt_content_opened': False,
                                  'scorer_invoked': False},
                })
                raise
        try:
            completed_status, lifecycle = supervise_container(
                process,
                container_name=container_name,
                cidfile=cidfile,
                lifecycle_path=lifecycle_path,
                interval_seconds=supervision_interval,
                stats_interval_seconds=stats_interval,
                max_runtime_seconds=float(watchdog['max_runtime_seconds']),
                term_grace_seconds=float(watchdog['term_grace_seconds']))
        finally:
            if watchdog_process is not None and watchdog_process.poll() is None:
                watchdog_process.terminate()
            if watchdog_process is not None:
                try:
                    watchdog_process.wait(timeout=10)
                except (OSError, subprocess.TimeoutExpired):
                    watchdog_process.kill()
                    watchdog_process.wait()
    finished = dt.datetime.now(dt.timezone.utc)
    report = {
        'schema_version': 2, 'contract_id': phase['contract_id'],
        'profile_key': selected_profile_key,
        'rival_source_closure': closure_identity,
        'system': 'fast_livo2', 'phase_mode': args.phase_mode,
        'started_at': started.isoformat(), 'finished_at': finished.isoformat(),
        'input': {'path': str(bag), 'bytes': bag.stat().st_size,
                  'sha256': actual_hash},
        'execution': {'container_exit_status': completed_status,
                      'image_identity': image_identity,
                      'host_lifecycle_path': str(lifecycle_path),
                      'host_lifecycle': lifecycle,
                      'global_watchdog': {
                          'max_runtime_seconds': phase['watchdog'][
                              'max_runtime_seconds'],
                          'term_grace_seconds': phase['watchdog'][
                              'term_grace_seconds'],
                          'poll_interval_seconds': phase['watchdog'][
                              'poll_interval_seconds'],
                          'stats_interval_seconds': phase['watchdog'][
                              'stats_interval_seconds'],
                          'detached_watchdog_used': watchdog_process is not None,
                      },
                      'gt_content_opened': False, 'scorer_invoked': False},
        'command': command,
    }
    atomic_json_replace(run_dir / 'run.json', report)
    cleanup = remove_exited_container(container_name)
    report['execution']['container_cleanup'] = cleanup
    atomic_json_replace(run_dir / 'run.json', report)
    cleanup_ok = cleanup['status'] in {'removed', 'not_present'}
    return 0 if completed_status == 0 and cleanup_ok else 2


def bag_container_binding(bag: Path, asset_root: Path) -> tuple[str, list[str]]:
    """Return the in-container bag path and any additional read-only mount."""
    if _is_relative_to(bag, asset_root):
        return '/bench/' + bag.relative_to(asset_root).as_posix(), []
    return '/input/input.bag', ['-v', f'{bag}:/input/input.bag:ro']


def fast_log_binding(run_dir: Path, save_map: bool) -> list[str]:
    """Isolate official trajectory logs, and maps when requested, per run."""
    destination = run_dir / 'fast_log'
    (destination / 'result').mkdir(parents=True, exist_ok=True)
    if save_map:
        (destination / 'pcd').mkdir(parents=True, exist_ok=True)
    return ['-v', f'{destination}:/bench/FAST-LIVO2/Log']


def select_fast_map(run_dir: Path) -> Path | None:
    """Select the official downsampled map, never a partial interval PCD."""
    candidates = (
        run_dir / 'fast_log/pcd/all_downsampled_points.pcd',
        run_dir / 'fast_log/pcd/all_raw_points.pcd',
    )
    return next((path for path in candidates
                 if path.is_file() and path.stat().st_size > 0), None)


def evaluate_map_quality(map_path: Path, run_dir: Path) -> dict[str, Any] | None:
    """Run the common geometry evaluator and return its normalized payload."""
    quality_dir = run_dir / 'map_quality'
    command = [
        'bash', str(ROOT / 'scripts/run_map_quality_check.sh'),
        '--input', str(map_path), '--output-dir', str(quality_dir),
        '--runs', '1', '--downsample', '0.1',
        '--setup', str(ROOT / 'install/setup.bash')]
    completed = subprocess.run(
        command, cwd=ROOT, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, check=False)
    report = quality_dir / 'run1/map_quality_report.yaml'
    if completed.returncode != 0 or not report.is_file():
        return None
    payload = yaml.safe_load(report.read_text())['map_quality_report']
    payload['source_map'] = {
        'path': str(map_path.resolve()), 'sha256': sha256(map_path),
        'bytes': map_path.stat().st_size}
    return payload


def tum_trajectory_info(path: Path) -> dict[str, Any]:
    count = malformed = 0
    first_stamp = last_stamp = None
    if not path.is_file():
        return {'samples': 0, 'malformed_rows': 0,
                'first_stamp': None, 'last_stamp': None}
    for line in path.read_text(errors='replace').splitlines():
        fields = line.split()
        try:
            if len(fields) != 8:
                raise ValueError
            stamp = float(fields[0])
            values = [float(value) for value in fields[1:]]
            if not all(value == value for value in [stamp, *values]):
                raise ValueError
        except ValueError:
            malformed += 1
            continue
        first_stamp = stamp if first_stamp is None else first_stamp
        last_stamp, count = stamp, count + 1
    return {'samples': count, 'malformed_rows': malformed,
            'first_stamp': first_stamp, 'last_stamp': last_stamp}


def collect_official_state_trajectory(run_dir: Path) -> tuple[dict[str, Any], str | None]:
    """Collect FAST-LIVO2's exact LiDAR-update-time IMU state trajectory."""
    candidates = sorted(path for path in (run_dir / 'fast_log/result').glob('*.txt')
                        if path.stat().st_size > 0)
    destination = run_dir / 'trajectory_imu.tum'
    if len(candidates) != 1:
        return tum_trajectory_info(destination), None
    shutil.copyfile(candidates[0], destination)
    return tum_trajectory_info(destination), str(candidates[0])


def load_reference_offset(path: Path, source_frame: str) -> tuple[float, float, float]:
    metadata = json.loads(path.read_text())
    key = f'{source_frame}_to_prism_translation_m'
    offset = metadata.get(key)
    if not isinstance(offset, dict) or any(axis not in offset for axis in 'xyz'):
        raise ValueError(f'reference metadata lacks {key}')
    return tuple(float(offset[axis]) for axis in 'xyz')


def apply_tum_translation_offset(source: Path, destination: Path,
                                 offset: tuple[float, float, float]) -> None:
    """Move each TUM pose origin by a fixed offset expressed in its local frame."""
    tx, ty, tz = offset
    lines = []
    for line in source.read_text(errors='replace').splitlines():
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f'invalid TUM line: {line}')
        stamp = fields[0]
        px, py, pz, qx, qy, qz, qw = map(float, fields[1:])
        norm = (qw * qw + qx * qx + qy * qy + qz * qz) ** 0.5
        if norm <= 0.0:
            raise ValueError('zero-norm TUM quaternion')
        qw, qx, qy, qz = (value / norm for value in (qw, qx, qy, qz))
        rx = ((1.0 - 2.0 * (qy * qy + qz * qz)) * tx +
              2.0 * (qx * qy - qz * qw) * ty +
              2.0 * (qx * qz + qy * qw) * tz)
        ry = (2.0 * (qx * qy + qz * qw) * tx +
              (1.0 - 2.0 * (qx * qx + qz * qz)) * ty +
              2.0 * (qy * qz - qx * qw) * tz)
        rz = (2.0 * (qx * qz - qy * qw) * tx +
              2.0 * (qy * qz + qx * qw) * ty +
              (1.0 - 2.0 * (qx * qx + qy * qy)) * tz)
        lines.append(
            f'{stamp} {px + rx:.9f} {py + ry:.9f} {pz + rz:.9f} '
            f'{fields[4]} {fields[5]} {fields[6]} {fields[7]}\n')
    destination.write_text(''.join(lines))


def validate_frozen_input_manifest(path: Path | None, hash_key: str,
                                   actual_hash: str) -> dict[str, Any] | None:
    if path is None:
        return None
    document = json.loads(path.read_text())
    if document.get('status') != 'frozen':
        raise ValueError(f'input manifest is not frozen: {path}')
    expected = document.get('hashes', {}).get(hash_key)
    if expected != actual_hash:
        raise ValueError(
            f'input hash differs from frozen manifest: {hash_key} '
            f'expected={expected} actual={actual_hash}')
    return {'path': str(path.resolve()), 'sha256': sha256(path),
            'slot': document.get('slot'), 'sequence': document.get('sequence')}


def run_once(args: argparse.Namespace, asset_root: Path, output: Path,
             run_index: int, shared: dict[str, Any]) -> dict[str, Any]:
    run_dir = output / f'run_{run_index:02d}'
    run_dir.mkdir(parents=True, exist_ok=False)
    bag_inside, bag_mount = bag_container_binding(args.bag, asset_root)
    log_mount = fast_log_binding(run_dir, args.save_map)
    launch_mounts: list[str] = []
    launch_environment: list[str] = []
    if args.mapping_launch is not None:
        launch_mounts += ['-v', f'{args.mapping_launch}:/benchmark_launch.launch:ro']
        launch_environment += ['-e', 'MAPPING_LAUNCH=/benchmark_launch.launch']
    if args.mapping_map_launch is not None:
        launch_mounts += [
            '-v', f'{args.mapping_map_launch}:/benchmark_map_launch.launch:ro']
        launch_environment += [
            '-e', 'MAPPING_MAP_LAUNCH=/benchmark_map_launch.launch']
    command = [
        'docker', 'run', '--rm', '--init', '--name',
        f'fast-livo2-bench-{run_index}-{os.getpid()}',
        '-e', f'BAG_PATH={bag_inside}', '-e', f'RATE={args.rate}',
        '-e', f'SHUTDOWN_GRACE_SECONDS={args.shutdown_grace_seconds}',
        '-e', f'SAVE_MAP={1 if args.save_map else 0}',
        *launch_environment,
        '-v', f'{asset_root}:/bench', *bag_mount, *log_mount, *launch_mounts,
        '-v', f'{ROOT}:/runner:ro',
        '-v', f'{run_dir}:/out', '--entrypoint', '/bin/bash', args.image,
        '/runner/scripts/fast_livo2_container_run.sh']
    started = dt.datetime.now(dt.timezone.utc)
    with (run_dir / 'container_stdout.log').open('w') as stdout, \
            (run_dir / 'container_stderr.log').open('w') as stderr:
        completed = subprocess.run(command, stdout=stdout, stderr=stderr,
                                   check=False)
    finished = dt.datetime.now(dt.timezone.utc)
    legacy_odometry = odometry_csv_to_tum(
        run_dir / 'odometry.csv', run_dir / 'odometry_now_stamp.tum')
    trajectory, official_path = collect_official_state_trajectory(run_dir)
    scoring_path = run_dir / 'trajectory_prism.tum'
    if trajectory['samples'] > 0:
        apply_tum_translation_offset(
            run_dir / 'trajectory_imu.tum', scoring_path, args.frame_offset)
    scoring_trajectory = tum_trajectory_info(scoring_path)
    bag_start, bag_end = parse_bag_bounds(run_dir / 'rosbag_info.yaml')
    end_gap = (None if bag_end is None or trajectory['last_stamp'] is None
               else bag_end - trajectory['last_stamp'])
    bag_time = parse_time_report(run_dir / 'bag_time.txt')
    mapper_time = parse_time_report(run_dir / 'mapper_time.txt')
    duration = None if bag_start is None or bag_end is None else bag_end - bag_start
    rtf = (float(bag_time['wall_seconds']) / duration
           if duration and bag_time.get('wall_seconds') else None)
    processing_bound = processing_rtf_upper_bound(
        bag_time.get('wall_seconds'), args.shutdown_grace_seconds, duration)
    bag_exit = read_int(run_dir / 'bag_exit_status.txt')
    alive_after_bag = read_int(run_dir / 'mapper_alive_after_bag.txt') == 1
    shutdown_exit = read_int(run_dir / 'mapper_shutdown_exit_status.txt')
    map_path = select_fast_map(run_dir) if args.save_map else None
    map_quality = (evaluate_map_quality(map_path, run_dir)
                   if map_path is not None else None)
    complete = (bag_exit == 0 and alive_after_bag and trajectory['samples'] > 0 and
                scoring_trajectory['samples'] == trajectory['samples'] and
                end_gap is not None and end_gap <= args.maximum_end_gap_seconds)
    if args.save_map and map_quality is None:
        complete = False
    report = {
        'schema_version': 1, 'system': 'fast_livo2', 'run_index': run_index,
        'started_at': started.isoformat(), 'finished_at': finished.isoformat(),
        'provenance': shared,
        'execution': {'container_exit_status': completed.returncode,
                      'bag_exit_status': bag_exit,
                      'mapper_alive_after_bag': alive_after_bag,
                      'mapper_shutdown_exit_status': shutdown_exit},
        'completion': {'trajectory_complete': complete,
                       'trajectory_end_gap_seconds': end_gap,
                       'process_exit_status': shutdown_exit},
        'trajectory': trajectory,
        'scoring_trajectory': scoring_trajectory,
        'trajectory_contract': {
            'source': 'official_FAST_LIVO2_Log_result',
            'source_path': official_path,
            'timestamp': 'LidarMeasures.last_lio_update_time',
            'source_frame': args.trajectory_source_frame,
            'target_frame': 'leica_prism',
            'offset_m': dict(zip('xyz', args.frame_offset)),
            'legacy_now_stamp_odometry': legacy_odometry,
        },
        'runtime': {'bag_duration_seconds': duration,
                    'replay_wall_realtime_factor': rtf,
                    'processing_realtime_factor_upper_bound': processing_bound,
                    'processing_measurement_method': (
                        'accelerated_replay_plus_fixed_drain_requires_'
                        'accuracy_and_count_validation'),
                    'mapper': mapper_time, 'bag_player': bag_time},
    }
    if map_quality is not None:
        report['mapping'] = map_quality
    (run_dir / 'run.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--asset-root', type=Path,
        help='FAST source root for the legacy v1 path; omitted by M6a10 v2')
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, default=DEFAULT_PROFILE)
    parser.add_argument(
        '--mapping-launch', type=Path,
        help='Optional official-source-compatible roslaunch file for this sensor')
    parser.add_argument(
        '--mapping-map-launch', type=Path,
        help='Map-export roslaunch variant used together with --save-map')
    parser.add_argument('--input-manifest', type=Path)
    parser.add_argument('--reference-meta', type=Path, required=True)
    parser.add_argument('--trajectory-source-frame', choices=('imu', 'body', 'lidar'),
                        default='imu')
    parser.add_argument('--image', default='fast-livo2-benchmark:noetic')
    parser.add_argument('--runs', type=int)
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--shutdown-grace-seconds', type=float, default=5.0)
    parser.add_argument('--maximum-end-gap-seconds', type=float, default=0.25)
    parser.add_argument('--save-map', action='store_true')
    parser.add_argument('--phase-contract', default='v1',
                        choices=('v1', 'm6a10-online-compute-v2'))
    parser.add_argument(
        '--m6a10-profile-key', choices=tuple(sorted(M6A10_PROFILE_KEYS)),
        help='explicit persisted M6a10 contract key (required when a profile is ambiguous)')
    parser.add_argument('--phase-mode', default='unpaced_ack',
                        choices=('paced_1x', 'unpaced_ack'))
    return parser.parse_args()


def _watchdog_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--watchdog-child', action='store_true')
    parser.add_argument('--watchdog-container-name', required=True)
    parser.add_argument('--watchdog-cidfile', type=Path, required=True)
    parser.add_argument('--watchdog-lifecycle', type=Path, required=True)
    parser.add_argument('--watchdog-run-json', type=Path, required=True)
    parser.add_argument('--watchdog-command-sha256', required=True)
    parser.add_argument('--watchdog-input-path', required=True)
    parser.add_argument('--watchdog-input-sha256', required=True)
    parser.add_argument('--watchdog-contract-id', required=True)
    parser.add_argument('--watchdog-max-runtime', type=float, required=True)
    parser.add_argument('--watchdog-term-grace', type=float, required=True)
    parser.add_argument('--watchdog-interval', type=float, required=True)
    parser.add_argument('--watchdog-stats-interval', type=float, required=True)
    return parser


def _run_watchdog_child(argv: Sequence[str]) -> int:
    args = _watchdog_parser().parse_args(argv)
    return _detached_watchdog_loop(
        container_name=args.watchdog_container_name,
        cidfile=args.watchdog_cidfile,
        lifecycle_path=args.watchdog_lifecycle,
        run_json_path=args.watchdog_run_json,
        command_sha256=args.watchdog_command_sha256,
        input_path=args.watchdog_input_path,
        input_sha256=args.watchdog_input_sha256,
        contract_id=args.watchdog_contract_id,
        max_runtime_seconds=args.watchdog_max_runtime,
        term_grace_seconds=args.watchdog_term_grace,
        interval_seconds=args.watchdog_interval,
        stats_interval_seconds=args.watchdog_stats_interval)


def main() -> int:
    if '--watchdog-child' in sys.argv[1:]:
        return _run_watchdog_child(sys.argv[1:])
    args = parse_args()
    args.bag = args.bag.resolve()
    args.output = args.output.resolve()
    if args.phase_contract == 'm6a10-online-compute-v2':
        return run_m6a10_phase(args)
    if args.asset_root is None:
        raise ValueError('--asset-root is required for the legacy v1 path')
    args.asset_root = args.asset_root.resolve()
    args.reference_meta = args.reference_meta.resolve()
    if not args.reference_meta.is_file():
        raise ValueError(f'--reference-meta is not a file: {args.reference_meta}')
    args.frame_offset = load_reference_offset(
        args.reference_meta, args.trajectory_source_frame)
    if args.mapping_launch is not None:
        args.mapping_launch = args.mapping_launch.resolve()
        if not args.mapping_launch.is_file():
            raise ValueError(f'--mapping-launch is not a file: {args.mapping_launch}')
    if args.mapping_map_launch is not None:
        args.mapping_map_launch = args.mapping_map_launch.resolve()
        if not args.mapping_map_launch.is_file():
            raise ValueError(
                f'--mapping-map-launch is not a file: {args.mapping_map_launch}')
    if args.save_map and args.mapping_launch is not None and args.mapping_map_launch is None:
        raise ValueError(
            '--mapping-map-launch is required with --save-map when '
            '--mapping-launch is supplied')
    if not args.bag.is_file():
        raise ValueError(f'--bag is not a file: {args.bag}')
    contract = load_contract(args.profile)
    closure_identity = current_competitive_closure_identity(contract)
    runs = contract['repetitions'] if args.runs is None else args.runs
    if runs < 1:
        raise ValueError('--runs must be positive')
    source_state = git_state(args.asset_root / 'FAST-LIVO2')
    expected = contract['rivals']['fast_livo2']['revision']
    if source_state['revision'] != expected or source_state['tracked_dirty']:
        raise RuntimeError(f'FAST-LIVO2 must be clean at {expected}; got {source_state}')
    args.output.mkdir(parents=True, exist_ok=False)
    bag_hash = sha256(args.bag)
    manifest = validate_frozen_input_manifest(
        args.input_manifest, 'raw_rosbag1_sha256', bag_hash)
    shared = {
        'profile': contract['name'], 'profile_sha256': sha256(args.profile),
        'rival_source_closure': closure_identity,
        'machine': benchmark_machine_fingerprint(),
        'source': source_state, 'bag_path': str(args.bag),
        'bag_sha256': bag_hash, 'input_manifest': manifest,
        'container_image': args.image,
        'container_image_id': command_output([
            'docker', 'image', 'inspect', args.image, '--format', '{{.Id}}']),
        'rate': args.rate, 'map_export_enabled': args.save_map,
        'reference_metadata': {
            'path': str(args.reference_meta), 'sha256': sha256(args.reference_meta)},
        'trajectory_source_frame': args.trajectory_source_frame,
        'trajectory_to_prism_offset_m': dict(zip('xyz', args.frame_offset)),
        'mapping_launch': None if args.mapping_launch is None else {
            'path': str(args.mapping_launch), 'sha256': sha256(args.mapping_launch)},
        'mapping_map_launch': None if args.mapping_map_launch is None else {
            'path': str(args.mapping_map_launch),
            'sha256': sha256(args.mapping_map_launch)},
    }
    reports = []
    for index in range(1, runs + 1):
        print(f'FAST-LIVO2 repetition {index}/{runs}', flush=True)
        reports.append(run_once(args, args.asset_root, args.output, index, shared))
    summary = {
        'schema_version': 1, 'system': 'fast_livo2', 'requested_runs': runs,
        'completed_trajectories': sum(
            report['completion']['trajectory_complete'] for report in reports),
        'clean_shutdowns': sum(
            report['completion']['process_exit_status'] == 0 for report in reports),
        'completed_maps': sum('mapping' in report for report in reports),
        'runs': reports,
    }
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: summary[key] for key in (
        'requested_runs', 'completed_trajectories', 'clean_shutdowns')}, indent=2))
    return 0 if summary['completed_trajectories'] == runs else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
