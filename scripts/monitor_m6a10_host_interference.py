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
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Read-only continuous host-interference observation for an M6a10 run.

The monitor is deliberately an observer.  It samples the existing procfs
helper from the same host process, excludes the launcher's ancestry and the
owned runtime PIDs, and never controls another process.  A matching build
process makes ``contaminated`` sticky for the complete run.  The append-only
sample log and its final summary are evidence for a later closure decision;
this module does not make a performance decision by itself.
"""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable, Iterable, Mapping

try:
    from lidarslam_benchmark_tools.check_m6a10_quiescence import (
        ancestor_pids,
        find_forbidden_processes,
    )
except ModuleNotFoundError:  # pragma: no cover - used by direct file imports
    import importlib.util

    _HELPER_PATH = Path(__file__).with_name('check_m6a10_quiescence.py')
    _HELPER_SPEC = importlib.util.spec_from_file_location(
        'm6a10_quiescence_for_interference', _HELPER_PATH)
    if _HELPER_SPEC is None or _HELPER_SPEC.loader is None:
        raise ImportError(f'cannot load {_HELPER_PATH}')
    _HELPER_MODULE = importlib.util.module_from_spec(_HELPER_SPEC)
    _HELPER_SPEC.loader.exec_module(_HELPER_MODULE)
    ancestor_pids = _HELPER_MODULE.ancestor_pids
    find_forbidden_processes = _HELPER_MODULE.find_forbidden_processes


SCHEMA_VERSION = 1
CONTRACT_VERSION = 'm6a10-host-interference-v1'
DEFAULT_INTERVAL_SECONDS = 5.0
MAX_INTERVAL_SECONDS = 5.0
MAX_COVERAGE_GAP_SECONDS = 5.0
COVERAGE_JITTER_FACTOR = 1.25
SUMMARY_MODE = 0o444


class InterferenceError(ValueError):
    """A malformed monitor lifecycle or an unsafe evidence destination."""


def _finite_nonnegative(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise InterferenceError(f'{label} is not numeric')
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise InterferenceError(f'{label} is not finite/nonnegative')
    return result


def _validate_monotonic_ns(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise InterferenceError(f'{label} is not a nonnegative integer')
    return value


def _utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def allowed_pids(
        *, proc_root: Path = Path('/proc'), launcher_pid: int | None = None,
        owned_pids: Iterable[int] = ()) -> set[int]:
    """Return launcher ancestry plus explicitly owned runtime PIDs."""
    root_pid = os.getpid() if launcher_pid is None else launcher_pid
    if isinstance(root_pid, bool) or not isinstance(root_pid, int) or root_pid <= 0:
        raise InterferenceError('launcher_pid must be a positive integer')
    result = set(ancestor_pids(proc_root, pid=root_pid))
    for pid in owned_pids:
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise InterferenceError('owned PIDs must be positive integers')
        result.add(pid)
    return result


def _default_sampler(
        proc_root: Path, excluded_pids: Iterable[int]) -> dict[str, Any]:
    matches, race_skips = find_forbidden_processes(
        proc_root, exclude_pids=excluded_pids)
    # The shared helper already emits only redacted command identity fields.
    redacted = []
    for match in matches:
        redacted.append({
            'pid': int(match['pid']),
            'comm': str(match['comm'])[:128],
            'class': str(match['class']),
            'argv_sha256': str(match['argv_sha256']),
        })
    return {
        'forbidden_processes': redacted,
        'proc_race_count': int(race_skips),
    }


def _normalise_matches(value: Any) -> list[dict[str, Any]]:
    if value is None:
        return []
    if not isinstance(value, (list, tuple)):
        raise InterferenceError('forbidden_processes is not a sequence')
    result: list[dict[str, Any]] = []
    for match in value:
        if not isinstance(match, Mapping):
            raise InterferenceError('forbidden process match is not an object')
        required = ('pid', 'comm', 'class', 'argv_sha256')
        if any(key not in match for key in required):
            raise InterferenceError('forbidden process match is incomplete')
        pid = match['pid']
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise InterferenceError('forbidden process PID is invalid')
        digest = str(match['argv_sha256'])
        if len(digest) != 64 or any(char not in '0123456789abcdef' for char in digest):
            raise InterferenceError('forbidden process argv hash is invalid')
        result.append({
            'pid': pid,
            'comm': str(match['comm'])[:128],
            'class': str(match['class']),
            'argv_sha256': digest,
        })
    return result


def _atomic_create_bytes(path: Path, payload: bytes, *, mode: int = 0o600) -> None:
    """Create one file without replacing a prior file or following a symlink."""
    if os.path.lexists(path):
        raise InterferenceError(f'refusing to overwrite evidence: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + '.part')
    if os.path.lexists(part):
        raise InterferenceError(f'stale evidence staging file exists: {part}')
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    no_follow = getattr(os, 'O_NOFOLLOW', 0)
    fd = os.open(part, flags | no_follow, mode)
    try:
        with os.fdopen(fd, 'wb', closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(part, path, follow_symlinks=False)
        except FileExistsError as error:
            raise InterferenceError(f'refusing to overwrite evidence: {path}') from error
    finally:
        try:
            part.unlink()
        except FileNotFoundError:
            pass
    os.chmod(path, mode, follow_symlinks=False)


def _append_jsonl(path: Path, value: Mapping[str, Any]) -> None:
    if path.is_symlink() or not path.exists():
        raise InterferenceError(f'sample log is not prepared: {path}')
    payload = (json.dumps(value, sort_keys=True, separators=(',', ':')) + '\n').encode()
    flags = os.O_WRONLY | os.O_APPEND
    no_follow = getattr(os, 'O_NOFOLLOW', 0)
    fd = os.open(path, flags | no_follow)
    try:
        with os.fdopen(fd, 'ab', closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except OSError as error:
        raise InterferenceError(f'cannot append sample log: {path}') from error


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


Sampler = Callable[[set[int]], Mapping[str, Any]]


class HostInterferenceMonitor:
    """Threaded, same-process, read-only host interference monitor."""

    def __init__(
            self, samples_path: Path, *, proc_root: Path = Path('/proc'),
            interval_seconds: float = DEFAULT_INTERVAL_SECONDS,
            sampler: Sampler | None = None,
            clock: Callable[[], int] = time.monotonic_ns,
            utc_clock: Callable[[], str] = _utc_now,
            launcher_pid: int | None = None,
            owned_pids: Iterable[int] = ()) -> None:
        interval = _finite_nonnegative(interval_seconds, 'interval_seconds')
        if interval <= 0 or interval > MAX_INTERVAL_SECONDS:
            raise InterferenceError('interval_seconds must be >0 and <=5 seconds')
        self.samples_path = Path(samples_path)
        self.proc_root = Path(proc_root)
        self.interval_seconds = interval
        self._sampler = sampler
        self._clock = clock
        self._utc_clock = utc_clock
        self._allowed_pids = allowed_pids(
            proc_root=self.proc_root, launcher_pid=launcher_pid, owned_pids=owned_pids)
        self._lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._started = False
        self._stopped = False
        self._finalized = False
        self._order = 0
        self._samples: list[dict[str, Any]] = []
        self._pending_pid_events: list[dict[str, Any]] = []
        self._pid_events: list[dict[str, Any]] = []
        self._contaminated = False
        self._invalid = False
        self._invalid_reasons: list[str] = []

    @property
    def allowed_pid_set(self) -> frozenset[int]:
        return frozenset(self._allowed_pids)

    @property
    def contaminated(self) -> bool:
        return self._contaminated

    @property
    def invalid(self) -> bool:
        return self._invalid

    @property
    def samples(self) -> tuple[dict[str, Any], ...]:
        return tuple(self._samples)

    def allow_owned_pid(self, pid: int) -> None:
        """Allow one already-owned runtime PID for subsequent snapshots.

        The runner PID is often known only after the start boundary.  This
        method changes only the monitor's read-only exclusion set and records
        the event in the next sample and in the final summary.
        """
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise InterferenceError('owned PID must be a positive integer')
        with self._lock:
            if not self._started or self._stopped or self._finalized:
                raise InterferenceError('owned PID can only be added while running')
            stamp = _validate_monotonic_ns(self._clock(), 'owned PID event monotonic_ns')
            event = {
                'event': 'owned_pid_allowed',
                'pid': pid,
                'monotonic_ns': stamp,
                'event_order': len(self._pid_events),
            }
            self._allowed_pids.add(pid)
            self._pending_pid_events.append(event)
            self._pid_events.append(event)

    def _prepare_samples_file(self) -> None:
        if os.path.lexists(self.samples_path):
            raise InterferenceError(f'refusing to overwrite sample log: {self.samples_path}')
        self.samples_path.parent.mkdir(parents=True, exist_ok=True)
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        no_follow = getattr(os, 'O_NOFOLLOW', 0)
        fd = os.open(self.samples_path, flags | no_follow, 0o600)
        os.close(fd)

    def _mark_invalid(self, reason: str) -> None:
        self._invalid = True
        if reason not in self._invalid_reasons:
            self._invalid_reasons.append(reason)

    def _sample_locked(self, phase: str) -> None:
        stamp = _validate_monotonic_ns(self._clock(), 'sample monotonic_ns')
        if self._samples and stamp <= self._samples[-1]['monotonic_ns']:
            self._mark_invalid('non_monotonic_sample_interval')
        error: str | None = None
        matches: list[dict[str, Any]] = []
        race_count = 0
        try:
            raw = (_default_sampler(self.proc_root, self._allowed_pids)
                   if self._sampler is None else self._sampler(self._allowed_pids))
            if not isinstance(raw, Mapping):
                raise InterferenceError('sample callback did not return an object')
            matches = _normalise_matches(raw.get('forbidden_processes', []))
            race_count_value = raw.get('proc_race_count', raw.get('proc_race_skips', 0))
            if (isinstance(race_count_value, bool) or
                    not isinstance(race_count_value, int) or race_count_value < 0):
                raise InterferenceError('proc race count is invalid')
            race_count = race_count_value
        except Exception as caught:  # sticky invalid; keep an auditable sample
            self._mark_invalid('sample_callback_exception')
            error = f'{type(caught).__name__}: {caught}'[:512]
        if matches:
            self._contaminated = True
        sample: dict[str, Any] = {
            'schema_version': SCHEMA_VERSION,
            'contract_version': CONTRACT_VERSION,
            'order': self._order,
            'phase': phase,
            'monotonic_ns': stamp,
            'captured_at_utc': self._utc_clock(),
            'forbidden_processes': matches,
            'proc_race_count': race_count,
            'sample_valid': error is None,
        }
        if self._pending_pid_events:
            sample['owned_pid_events'] = list(self._pending_pid_events)
        if error is not None:
            sample['error'] = error
        _append_jsonl(self.samples_path, sample)
        self._samples.append(sample)
        self._pending_pid_events.clear()
        self._order += 1

    def sample_once(self, phase: str = 'interval') -> None:
        """Capture one deterministic sample; useful for injected test clocks."""
        if phase not in {'run_start', 'interval', 'run_end'}:
            raise InterferenceError(f'unknown sample phase: {phase}')
        with self._lock:
            if not self._started or self._stopped:
                raise InterferenceError('monitor is not running')
            self._sample_locked(phase)

    def _worker(self) -> None:
        while not self._stop_event.wait(self.interval_seconds):
            try:
                with self._lock:
                    if self._stopped:
                        return
                    self._sample_locked('interval')
            except Exception as error:  # preserve natural completion and mark invalid
                with self._lock:
                    self._mark_invalid(f'monitor_sample_exception:{type(error).__name__}')

    def start(self) -> None:
        with self._lock:
            if self._started:
                raise InterferenceError('monitor already started')
            if self._finalized:
                raise InterferenceError('monitor already finalized')
            self._prepare_samples_file()
            self._started = True
            try:
                self._sample_locked('run_start')
            except Exception:
                self._mark_invalid('run_start_capture_failure')
                raise
            self._thread = threading.Thread(
                target=self._worker, name='m6a10-host-interference', daemon=True)
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                raise InterferenceError('monitor is not started')
            if self._stopped:
                raise InterferenceError('monitor already stopped')
            self._stop_event.set()
            self._stopped = True
            self._sample_locked('run_end')
            thread = self._thread
        if thread is not None:
            thread.join()

    def _coverage(self) -> dict[str, Any]:
        if not self._samples:
            raise InterferenceError('zero samples cannot be finalized')
        starts = [item for item in self._samples if item['phase'] == 'run_start']
        ends = [item for item in self._samples if item['phase'] == 'run_end']
        if len(starts) != 1 or len(ends) != 1:
            raise InterferenceError('coverage gap: start/end snapshots are incomplete')
        stamps = [item['monotonic_ns'] for item in self._samples]
        gaps = [right - left for left, right in zip(stamps, stamps[1:])]
        if any(gap <= 0 for gap in gaps):
            raise InterferenceError('invalid interval: sample timestamps are not increasing')
        max_gap_ns = max(gaps) if gaps else 0
        max_gap_seconds = max_gap_ns / 1_000_000_000
        # A cadence may include bounded scheduling jitter, but coverage can
        # never be more than five seconds apart.  A one-second cadence keeps
        # its tighter 1.25-second allowance; a four-second cadence can use the
        # full five-second contract.
        max_allowed_gap_seconds = min(
            MAX_COVERAGE_GAP_SECONDS,
            self.interval_seconds * COVERAGE_JITTER_FACTOR,
        )
        coverage_gap = max_gap_seconds > max_allowed_gap_seconds
        return {
            'start_snapshot': True,
            'end_snapshot': True,
            'interval_snapshot_count': sum(
                item['phase'] == 'interval' for item in self._samples),
            'max_gap_seconds': max_gap_seconds,
            'max_allowed_gap_seconds': max_allowed_gap_seconds,
            'coverage_gap': coverage_gap,
            'run_duration_seconds': (
                ends[0]['monotonic_ns'] - starts[0]['monotonic_ns']) / 1_000_000_000,
        }

    def finalize(self, summary_path: Path) -> dict[str, Any]:
        with self._lock:
            if not self._started or not self._stopped:
                raise InterferenceError('monitor must be stopped before finalization')
            if self._finalized:
                raise InterferenceError('monitor already finalized')
            coverage = self._coverage()
            if coverage['coverage_gap']:
                raise InterferenceError('coverage gap exceeds monitor interval')
            if self.samples_path.resolve() == Path(summary_path).resolve():
                raise InterferenceError('sample log and summary path must differ')
            samples_sha = _sha256_file(self.samples_path)
            os.chmod(self.samples_path, SUMMARY_MODE, follow_symlinks=False)
            status = 'PASS' if not self._contaminated and not self._invalid else 'FAIL_CLOSED'
            summary = {
                'schema_version': SCHEMA_VERSION,
                'contract_version': CONTRACT_VERSION,
                'status': status,
                'contaminated': self._contaminated,
                'invalid': self._invalid,
                'invalid_reasons': list(self._invalid_reasons),
                'sample_count': len(self._samples),
                'forbidden_sample_count': sum(
                    bool(item['forbidden_processes']) for item in self._samples),
                'proc_race_count': sum(item['proc_race_count'] for item in self._samples),
                'allowed_pid_count': len(self._allowed_pids),
                'owned_pid_allow_events': list(self._pid_events),
                'coverage': coverage,
                'samples_path': str(self.samples_path),
                'samples_sha256': samples_sha,
                'finalized_at_utc': self._utc_clock(),
            }
            payload = (json.dumps(summary, indent=2, sort_keys=True) + '\n').encode()
            _atomic_create_bytes(Path(summary_path), payload, mode=SUMMARY_MODE)
            summary_sha = _sha256_file(Path(summary_path))
            sidecar = Path(summary_path).with_name(Path(summary_path).name + '.sha256')
            _atomic_create_bytes(
                sidecar, f'{summary_sha}  {Path(summary_path).name}\n'.encode(),
                mode=SUMMARY_MODE)
            self._finalized = True
            summary['summary_sha256'] = summary_sha
            summary['summary_sidecar_path'] = str(sidecar)
            return summary


Monitor = HostInterferenceMonitor


__all__ = [
    'CONTRACT_VERSION',
    'COVERAGE_JITTER_FACTOR',
    'DEFAULT_INTERVAL_SECONDS',
    'HostInterferenceMonitor',
    'InterferenceError',
    'MAX_INTERVAL_SECONDS',
    'MAX_COVERAGE_GAP_SECONDS',
    'Monitor',
    'SCHEMA_VERSION',
    'allowed_pids',
]


if __name__ == "__main__":
    raise SystemExit("monitor_m6a10_host_interference is a library module; import it instead of running it directly.")
