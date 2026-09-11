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

"""Read-only CPU quiescence preflight for M6a10 benchmark runs.

The checker deliberately does not launch, stop, or reprioritize any process.
It samples procfs, records a short CPU/load observation, and atomically writes
a receipt.  A caller must verify the receipt's SHA-256 and ``status == PASS``
before starting a runner; the receipt itself is not a permission to mutate the
machine or to access benchmark/ground-truth data.
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys
import time
from typing import Any, Iterable


SCHEMA_VERSION = 1
CONTRACT_VERSION = 'm6a10-quiescence-v1'
DEFAULT_SAMPLE_SECONDS = 5.0
DEFAULT_MAX_BUSY_PERCENT = 5.0
DEFAULT_MAX_LOAD_PER_CPU = 0.50

# Match only commands that actively build/compile.  A plain ``cmake`` or
# ``docker ps`` inspection is not a build and is intentionally not rejected.
FORBIDDEN_PROCESS_PATTERNS = {
    'compiler': re.compile(
        r'(^|/)(?:cc|c\+\+|gcc|g\+\+|clang|clang\+\+|nvcc|rustc)(?:\s|$)'),
    'docker_build': re.compile(r'\b(?:docker|podman)\s+build(?:\s|$)'),
    'colcon_build': re.compile(r'\bcolcon\s+build(?:\s|$)'),
    'cmake_build': re.compile(r'\bcmake\s+--build(?:\s|$)'),
    'cargo_build': re.compile(r'\bcargo\s+build(?:\s|$)'),
    'generic_build': re.compile(
        r'\b(?:ninja|bazel)\s+build(?:\s|$)|\bmake(?:\s+-j|\s+all(?:\s|$))'),
}


class QuiescenceError(ValueError):
    """A malformed observation or unsafe preflight condition."""


def _finite_nonnegative(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise QuiescenceError(f'{label} is not numeric')
    result = float(value)
    if not math.isfinite(result) or result < 0:
        raise QuiescenceError(f'{label} is not finite/nonnegative')
    return result


def _read_text(path: Path) -> str:
    return path.read_text(encoding='utf-8', errors='replace')


def read_cpu_jiffies(proc_root: Path = Path('/proc')) -> dict[str, int]:
    """Read aggregate CPU counters from a procfs-compatible tree."""
    for line in _read_text(proc_root / 'stat').splitlines():
        if line.startswith('cpu '):
            fields = line.split()[1:]
            if len(fields) < 4:
                raise QuiescenceError('aggregate /proc/stat CPU line is short')
            try:
                values = [int(item) for item in fields]
            except ValueError as error:
                raise QuiescenceError('aggregate /proc/stat CPU line is invalid') from error
            if any(item < 0 for item in values):
                raise QuiescenceError('aggregate /proc/stat CPU counters are negative')
            return {
                'total_jiffies': sum(values),
                'idle_jiffies': values[3] + (values[4] if len(values) > 4 else 0),
            }
    raise QuiescenceError('aggregate CPU line is missing from /proc/stat')


def cpu_busy_percent(before: dict[str, int], after: dict[str, int]) -> float:
    """Compute aggregate busy percentage between two CPU snapshots."""
    total_delta = after['total_jiffies'] - before['total_jiffies']
    idle_delta = after['idle_jiffies'] - before['idle_jiffies']
    if total_delta <= 0 or idle_delta < 0 or idle_delta > total_delta:
        raise QuiescenceError('CPU counter delta is invalid')
    return 100.0 * (total_delta - idle_delta) / total_delta


def read_loadavg(proc_root: Path = Path('/proc')) -> dict[str, float]:
    """Read the first three load averages without invoking an external tool."""
    fields = _read_text(proc_root / 'loadavg').split()
    if len(fields) < 3:
        raise QuiescenceError('/proc/loadavg is short')
    values = [_finite_nonnegative(float(item), f'loadavg[{index}]')
              for index, item in enumerate(fields[:3])]
    return {'load1': values[0], 'load5': values[1], 'load15': values[2]}


def _pid_dirs(proc_root: Path) -> Iterable[int]:
    for item in proc_root.iterdir():
        if item.name.isdigit() and item.is_dir():
            yield int(item.name)


def _proc_parent(proc_root: Path, pid: int) -> int | None:
    try:
        fields = _read_text(proc_root / str(pid) / 'stat').split()
        # comm may contain spaces, but the final ')' in the kernel field is
        # sufficient for these lightweight ancestry checks.
        stat_line = _read_text(proc_root / str(pid) / 'stat')
        close = stat_line.rfind(')')
        fields = stat_line[close + 2:].split()
        return int(fields[1]) if len(fields) > 1 else None
    except (OSError, ValueError, IndexError):
        return None


def ancestor_pids(proc_root: Path = Path('/proc'), pid: int | None = None) -> set[int]:
    """Return the process and its ancestors, tolerating procfs races."""
    current = os.getpid() if pid is None else pid
    result: set[int] = set()
    while current > 0 and current not in result:
        result.add(current)
        parent = _proc_parent(proc_root, current)
        if parent is None or parent == current:
            break
        current = parent
    return result


def _process_cmdline(proc_root: Path, pid: int) -> tuple[str, str]:
    base = proc_root / str(pid)
    command_bytes = (base / 'cmdline').read_bytes()
    command = command_bytes.replace(b'\0', b' ').decode('utf-8', errors='replace').strip()
    comm = _read_text(base / 'comm').strip()
    return command, comm


def find_forbidden_processes(
        proc_root: Path = Path('/proc'), *,
        exclude_pids: Iterable[int] = ()) -> tuple[list[dict[str, Any]], int]:
    """Find active build processes, returning redacted command fingerprints.

    Full argv is intentionally not put in the receipt: command lines can
    contain host paths or credentials.  The SHA-256 permits later forensic
    comparison without exposing those values.
    """
    excluded = set(exclude_pids)
    matches: list[dict[str, Any]] = []
    race_skips = 0
    for pid in _pid_dirs(proc_root):
        if pid in excluded:
            continue
        try:
            command, comm = _process_cmdline(proc_root, pid)
            stat_line = _read_text(proc_root / str(pid) / 'stat')
            close = stat_line.rfind(')')
            state = stat_line[close + 2:].split()[0]
            if state == 'Z':
                continue
        except OSError:
            race_skips += 1
            continue
        lowered = command.lower()
        for class_name, pattern in FORBIDDEN_PROCESS_PATTERNS.items():
            if pattern.search(lowered):
                digest = hashlib.sha256(command.encode('utf-8')).hexdigest()
                matches.append({
                    'pid': pid,
                    'comm': comm[:128],
                    'class': class_name,
                    'argv_sha256': digest,
                })
                break
    return sorted(matches, key=lambda item: (item['class'], item['pid'])), race_skips


def receipt_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def require_pass_receipt(path: Path, expected_sha256: str | None = None) -> dict[str, Any]:
    """Load a receipt that authorizes a runner, optionally with SHA binding."""
    if not path.is_file():
        raise QuiescenceError(f'quiescence receipt is missing: {path}')
    if expected_sha256 is not None and receipt_sha256(path) != expected_sha256:
        raise QuiescenceError('quiescence receipt SHA-256 does not match')
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise QuiescenceError('quiescence receipt is not valid JSON') from error
    if value.get('schema_version') != SCHEMA_VERSION or \
            value.get('contract_version') != CONTRACT_VERSION or \
            value.get('status') != 'PASS' or \
            value.get('runner_start_allowed') is not True:
        raise QuiescenceError('quiescence receipt is not a PASS authorization')
    return value


def _atomic_write(path: Path, value: dict[str, Any]) -> None:
    if path.exists() or path.with_name(path.name + '.part').exists():
        raise QuiescenceError(f'refusing to overwrite quiescence receipt: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    part = path.with_name(path.name + '.part')
    part.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    os.replace(part, path)


def collect_observation(
        *, proc_root: Path = Path('/proc'), sample_seconds: float = DEFAULT_SAMPLE_SECONDS,
        max_busy_percent: float = DEFAULT_MAX_BUSY_PERCENT,
        max_load_per_cpu: float = DEFAULT_MAX_LOAD_PER_CPU,
        nproc: int | None = None, sleep=time.sleep,
        monotonic=time.monotonic, excluded_pids: Iterable[int] = ()) -> dict[str, Any]:
    sample_seconds = _finite_nonnegative(sample_seconds, 'sample_seconds')
    if sample_seconds <= 0:
        raise QuiescenceError('sample_seconds must be positive')
    max_busy_percent = _finite_nonnegative(max_busy_percent, 'max_busy_percent')
    max_load_per_cpu = _finite_nonnegative(max_load_per_cpu, 'max_load_per_cpu')
    processor_count = int(nproc if nproc is not None else (os.cpu_count() or 0))
    if processor_count <= 0:
        raise QuiescenceError('nproc must be positive')
    before = read_cpu_jiffies(proc_root)
    load = read_loadavg(proc_root)
    started = monotonic()
    sleep(sample_seconds)
    after = read_cpu_jiffies(proc_root)
    elapsed = monotonic() - started
    busy = cpu_busy_percent(before, after)
    forbidden, race_skips = find_forbidden_processes(
        proc_root, exclude_pids=excluded_pids)
    load_per_cpu = load['load1'] / processor_count
    checks = {
        'nproc_positive': processor_count > 0,
        'cpu_busy_within_limit': busy <= max_busy_percent,
        'load1_per_cpu_within_limit': load_per_cpu <= max_load_per_cpu,
        'no_forbidden_processes': not forbidden,
    }
    return {
        'schema_version': SCHEMA_VERSION,
        'contract_version': CONTRACT_VERSION,
        'sample_seconds': sample_seconds,
        'sample_elapsed_seconds': elapsed,
        'nproc': processor_count,
        'cpu': {
            'before': before,
            'after': after,
            'busy_percent': busy,
        },
        'loadavg': {**load, 'load1_per_cpu': load_per_cpu},
        'forbidden_processes': forbidden,
        'proc_race_skips': race_skips,
        'checks': checks,
        'limits': {
            'max_cpu_busy_percent': max_busy_percent,
            'max_load1_per_cpu': max_load_per_cpu,
        },
    }


def build_receipt(observation: dict[str, Any], *, now: str | None = None) -> dict[str, Any]:
    checks = observation['checks']
    passed = all(checks.values())
    return {
        'schema_version': SCHEMA_VERSION,
        'contract_version': CONTRACT_VERSION,
        'captured_at_utc': now or dt.datetime.now(dt.timezone.utc).isoformat(),
        'status': 'PASS' if passed else 'FAIL_CLOSED',
        'runner_start_allowed': passed,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'observation': observation,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sample-seconds', type=float, default=DEFAULT_SAMPLE_SECONDS)
    parser.add_argument('--max-busy-percent', type=float, default=DEFAULT_MAX_BUSY_PERCENT)
    parser.add_argument('--max-load-per-cpu', type=float, default=DEFAULT_MAX_LOAD_PER_CPU)
    parser.add_argument('--proc-root', type=Path, default=Path('/proc'))
    parser.add_argument('--ignore-pid', type=int, action='append', default=[])
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.output.exists() or args.output.with_name(args.output.name + '.part').exists():
        raise QuiescenceError(f'refusing to overwrite quiescence receipt: {args.output}')
    excluded = ancestor_pids(args.proc_root)
    excluded.update(args.ignore_pid)
    try:
        observation = collect_observation(
            proc_root=args.proc_root,
            sample_seconds=args.sample_seconds,
            max_busy_percent=args.max_busy_percent,
            max_load_per_cpu=args.max_load_per_cpu,
            excluded_pids=excluded)
        receipt = build_receipt(observation)
    except (OSError, QuiescenceError, ValueError) as error:
        receipt = {
            'schema_version': SCHEMA_VERSION,
            'contract_version': CONTRACT_VERSION,
            'captured_at_utc': dt.datetime.now(dt.timezone.utc).isoformat(),
            'status': 'FAIL_CLOSED',
            'runner_start_allowed': False,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'error': str(error),
        }
    _atomic_write(args.output, receipt)
    digest = receipt_sha256(args.output)
    print(json.dumps({
        'path': str(args.output),
        'sha256': digest,
        'status': receipt['status'],
        'runner_start_allowed': receipt['runner_start_allowed'],
    }, sort_keys=True))
    return 0 if receipt['status'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, QuiescenceError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)
