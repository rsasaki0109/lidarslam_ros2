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

"""Sample process RSS from the container PID namespace.

The sampler intentionally uses only the Python standard library.  It does not
use the host-side ``/usr/bin/time`` value: every sample is made by walking the
container's own ``/proc`` namespace and summing the RSS of all readable
processes except this sampler.  A final report is published with an atomic
rename so a driver can distinguish a completed measurement from a killed
sampler.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import time
from typing import Any, Iterable


MEASUREMENT_VERSION = 'm6a7-container-process-rss-v1'
MEASUREMENT_SCOPE = 'container_pid_namespace_proc_status'
PRIMARY_METRIC = 'aggregate_process_tree_peak_rss_bytes'
PRIMARY_METRIC_DEFINITION = (
    'sum_of_per_process_vmrss_peaks_shared_pages_may_be_recounted')
STATUS_FIELDS = ('VmRSS', 'RssAnon', 'RssFile', 'RssShmem')
_STATUS_RE = re.compile(r'^([0-9]+)\s+kB$')


class SamplerError(ValueError):
    """Raised for an invalid sampler configuration or summary."""


def _parse_kib(value: str) -> int:
    match = _STATUS_RE.fullmatch(value.strip())
    if match is None:
        raise SamplerError(f'invalid RSS value: {value!r}')
    return int(match.group(1)) * 1024


def parse_proc_status(text: str) -> dict[str, int]:
    """Parse the four RSS counters from one Linux ``/proc/*/status`` file."""
    fields: dict[str, str] = {}
    for line in text.splitlines():
        name, separator, value = line.partition(':')
        if separator and name in STATUS_FIELDS:
            fields[name] = value.strip()
    missing = [name for name in STATUS_FIELDS if name not in fields]
    if missing:
        raise SamplerError(f'missing RSS fields: {",".join(missing)}')
    return {name: _parse_kib(fields[name]) for name in STATUS_FIELDS}


def _pid_directories(proc_root: Path) -> Iterable[tuple[int, Path]]:
    """Yield numeric proc entries without materializing/stat-ing the tree.

    ``/proc`` is a hot path for the sampler.  ``Path.iterdir`` followed by
    ``Path.is_dir`` creates a temporary list and performs a second metadata
    lookup for every entry.  ``os.scandir`` carries the directory-entry type
    information and lets the caller process one PID at a time, preserving the
    same race-safe status/stat reads with substantially less sampler work.
    """
    try:
        with os.scandir(proc_root) as entries:
            for entry in entries:
                if not entry.name.isdecimal():
                    continue
                try:
                    if entry.is_dir(follow_symlinks=False):
                        yield int(entry.name), Path(entry.path)
                except OSError:
                    # A process can disappear while scandir is consuming the
                    # directory.  The status read below records the race when
                    # possible; an entry-level failure is simply skipped.
                    continue
    except OSError:
        return


def _pid_start_time(directory: Path) -> int | None:
    """Read Linux /proc stat field 22 when the fixture/kernel provides it."""
    try:
        text = (directory / 'stat').read_text(encoding='utf-8')
    except (OSError, UnicodeError):
        return None
    closing = text.rfind(')')
    if closing < 0:
        return None
    fields = text[closing + 2:].split()
    if len(fields) <= 19 or not fields[19].isdecimal():
        return None
    return int(fields[19])


def scan_proc(proc_root: Path | str = '/proc', *, exclude_pids: Iterable[int] = (),
              pid_start_times: dict[int, int] | None = None) -> dict[str, Any]:
    """Return one instantaneous process-tree RSS sample.

    A process disappearing between directory enumeration and ``status`` read
    is a normal PID race and is counted separately.  When ``pid_start_times``
    is supplied (as it is by the long-running sampler), one ``/proc/<pid>/stat``
    read per sample detects PID reuse against the previous sample.  This keeps
    the reuse guard while avoiding the old pre/post double stat read on every
    process.  Permission, malformed, or other reads are counted as errors and
    do not contribute a partial RSS value.
    """
    root = Path(proc_root)
    excluded = {int(pid) for pid in exclude_pids}
    excluded.add(os.getpid())
    totals = {name: 0 for name in STATUS_FIELDS}
    process_count = 0
    race_skips = 0
    pid_reuse_skips = 0
    errors = 0
    no_rss_skips = 0
    for pid, directory in _pid_directories(root):
        if pid in excluded:
            continue
        try:
            status_text = (directory / 'status').read_text(
                encoding='utf-8', errors='strict')
            parsed = parse_proc_status(status_text)
        except FileNotFoundError:
            race_skips += 1
            continue
        except SamplerError:
            if re.search(r'^Kthread:\s+1(?:\s|$)', status_text, re.MULTILINE) or \
                    re.search(r'^State:\s+I(?:\s|$)', status_text, re.MULTILINE):
                no_rss_skips += 1
            else:
                errors += 1
            continue
        except (OSError, UnicodeError):
            errors += 1
            continue
        if pid_start_times is not None:
            start_time = _pid_start_time(directory)
            if start_time is None:
                # Tiny proc fixtures used by the contract tests may omit
                # ``stat`` entirely.  A real numeric /proc entry has it; only
                # that case is a failed reuse/race observation.
                if (directory / 'stat').exists():
                    race_skips += 1
                    continue
            else:
                previous_start_time = pid_start_times.get(pid)
                pid_start_times[pid] = start_time
                if previous_start_time is not None and \
                        start_time != previous_start_time:
                    race_skips += 1
                    pid_reuse_skips += 1
                    continue
        process_count += 1
        for name in STATUS_FIELDS:
            totals[name] += parsed[name]
    return {
        'timestamp_monotonic_ns': time.monotonic_ns(),
        'process_count': process_count,
        'vmrss_bytes': totals['VmRSS'],
        'rss_anon_bytes': totals['RssAnon'],
        'rss_file_bytes': totals['RssFile'],
        'rss_shmem_bytes': totals['RssShmem'],
        'pid_race_skips': race_skips,
        'pid_reuse_skips': pid_reuse_skips,
        'no_rss_skips': no_rss_skips,
        'errors': errors,
    }


def _percent(value: float, denominator: float) -> float:
    if denominator <= 0:
        return 0.0
    return value / denominator * 100.0


def summarize_samples(
        samples: list[dict[str, Any]], *, requested_interval_seconds: float,
        sampler_pid: int, sample_errors: int, pid_race_skips: int,
        min_samples: int, max_errors: int, max_race_skips: int,
        max_jitter_percent: float, missed_intervals: int,
        stopped_by_signal: bool = False, scheduler_nice: int = 0) -> dict[str, Any]:
    """Build and validate the machine-readable sampler report."""
    if requested_interval_seconds <= 0 or not math.isfinite(requested_interval_seconds):
        raise SamplerError('requested interval must be finite and positive')
    if min_samples < 1 or max_errors < 0 or max_race_skips < 0 or \
            max_jitter_percent < 0 or not math.isfinite(max_jitter_percent):
        raise SamplerError('sampler thresholds are invalid')
    if isinstance(scheduler_nice, bool) or not isinstance(scheduler_nice, int) or \
            scheduler_nice < 0 or scheduler_nice > 19:
        raise SamplerError('scheduler nice must be an integer in [0, 19]')
    timestamps = [int(sample['timestamp_monotonic_ns']) for sample in samples]
    intervals = [
        (right - left) / 1_000_000_000.0
        for left, right in zip(timestamps, timestamps[1:])
        if right >= left
    ]
    actual = {
        'count': len(intervals),
        'mean_seconds': (sum(intervals) / len(intervals)) if intervals else None,
        'min_seconds': min(intervals) if intervals else None,
        'max_seconds': max(intervals) if intervals else None,
    }
    jitter = max(
        (_percent(abs(value - requested_interval_seconds),
                  requested_interval_seconds) for value in intervals),
        default=0.0)
    peaks = {}
    for field in ('vmrss_bytes', 'rss_anon_bytes', 'rss_file_bytes',
                  'rss_shmem_bytes', 'process_count'):
        peak_sample = max(samples, key=lambda sample: int(sample[field])) \
            if samples else None
        peaks[field] = max((int(sample[field]) for sample in samples), default=0)
        if field == 'vmrss_bytes' and peak_sample is not None:
            peaks['timestamp_monotonic_ns'] = int(
                peak_sample['timestamp_monotonic_ns'])
    valid = bool(
        len(samples) >= max(min_samples, 1) and
        sample_errors <= max_errors and
        pid_race_skips <= max_race_skips and
        jitter <= max_jitter_percent and
        missed_intervals == 0 and
        all(int(sample.get('process_count', -1)) >= 0 for sample in samples))
    reasons = []
    if len(samples) < min_samples:
        reasons.append('too_few_samples')
    if sample_errors > max_errors:
        reasons.append('sample_error_threshold_exceeded')
    if pid_race_skips > max_race_skips:
        reasons.append('pid_race_threshold_exceeded')
    if jitter > max_jitter_percent:
        reasons.append('interval_jitter_threshold_exceeded')
    if missed_intervals:
        reasons.append('missed_interval_detected')
    if not samples:
        reasons.append('no_samples')
    return {
        'schema_version': 1,
        'measurement_version': MEASUREMENT_VERSION,
        'measurement_scope': MEASUREMENT_SCOPE,
        'primary_metric': PRIMARY_METRIC,
        'primary_metric_definition': PRIMARY_METRIC_DEFINITION,
        'status': 'pass' if valid else 'invalid',
        'status_reason': '' if valid else ','.join(reasons),
        'atomic': False,
        'sampler_pid': int(sampler_pid),
        'sampler_excluded': True,
        'scheduler_nice': scheduler_nice,
        'interval_requested_seconds': requested_interval_seconds,
        'interval_actual': actual,
        'interval_jitter_percent': jitter,
        'missed_intervals': int(missed_intervals),
        'sample_count': len(samples),
        'sample_errors': int(sample_errors),
        'pid_race_skips': int(pid_race_skips),
        'thresholds': {
            'min_samples': int(min_samples),
            'max_errors': int(max_errors),
            'max_race_skips': int(max_race_skips),
            'max_jitter_percent': max_jitter_percent,
        },
        'peak': peaks,
        PRIMARY_METRIC: peaks['vmrss_bytes'],
        'first_sample_monotonic_ns': timestamps[0] if timestamps else None,
        'last_sample_monotonic_ns': timestamps[-1] if timestamps else None,
        'stopped_by_signal': bool(stopped_by_signal),
    }


def validate_summary(value: dict[str, Any]) -> tuple[bool, str]:
    """Fail-closed validation used by the shell helper and unit tests."""
    if not isinstance(value, dict):
        return False, 'summary_not_object'
    if value.get('measurement_version') != MEASUREMENT_VERSION or \
            value.get('measurement_scope') != MEASUREMENT_SCOPE:
        return False, 'summary_version_or_scope_invalid'
    if value.get('primary_metric') != PRIMARY_METRIC or \
            value.get('primary_metric_definition') != PRIMARY_METRIC_DEFINITION:
        return False, 'primary_metric_contract_invalid'
    scheduler_nice = value.get('scheduler_nice')
    if isinstance(scheduler_nice, bool) or not isinstance(scheduler_nice, int) or \
            scheduler_nice < 0 or scheduler_nice > 19:
        return False, 'scheduler_nice_invalid'
    effective_nice = value.get('scheduler_nice_effective')
    inherited_nice = value.get('scheduler_nice_inherited')
    if effective_nice is not None:
        if isinstance(effective_nice, bool) or not isinstance(effective_nice, int) or \
                effective_nice < scheduler_nice or effective_nice > 19:
            return False, 'scheduler_nice_effective_invalid'
    if inherited_nice is not None and (
            isinstance(inherited_nice, bool) or not isinstance(inherited_nice, int)):
        return False, 'scheduler_nice_inherited_invalid'
    if value.get('status') != 'pass' or value.get('atomic') is not True:
        return False, str(value.get('status_reason') or 'summary_not_pass')
    thresholds = value.get('thresholds')
    if not isinstance(thresholds, dict):
        return False, 'thresholds_missing'
    sample_count = value.get('sample_count')
    min_samples = thresholds.get('min_samples')
    if isinstance(min_samples, bool) or not isinstance(min_samples, int) or \
            min_samples < 1:
        return False, 'min_samples_threshold_invalid'
    if isinstance(sample_count, bool) or not isinstance(sample_count, int) or \
            sample_count < min_samples:
        return False, 'too_few_samples'
    for name, threshold in (('sample_errors', 'max_errors'),
                            ('pid_race_skips', 'max_race_skips')):
        count = value.get(name)
        limit = thresholds.get(threshold)
        if isinstance(count, bool) or not isinstance(count, int) or \
                isinstance(limit, bool) or not isinstance(limit, int) or \
                count < 0 or limit < 0 or count > limit:
            return False, f'{name}_threshold_exceeded'
    jitter = value.get('interval_jitter_percent')
    jitter_limit = thresholds.get('max_jitter_percent')
    if isinstance(jitter, bool) or not isinstance(jitter, (int, float)) or \
            not math.isfinite(float(jitter)) or jitter < 0 or \
            isinstance(jitter_limit, bool) or \
            not isinstance(jitter_limit, (int, float)) or \
            not math.isfinite(float(jitter_limit)) or jitter_limit < 0 or \
            jitter > jitter_limit:
        return False, 'interval_jitter_threshold_exceeded'
    if value.get('missed_intervals') != 0:
        return False, 'missed_interval_detected'
    peak = value.get('peak')
    if not isinstance(peak, dict) or any(
            isinstance(peak.get(field), bool) or
            not isinstance(peak.get(field), int) or peak.get(field) < 0
            for field in ('vmrss_bytes', 'rss_anon_bytes', 'rss_file_bytes',
                          'rss_shmem_bytes', 'process_count')):
        return False, 'peak_fields_invalid'
    aggregate = value.get(PRIMARY_METRIC)
    if isinstance(aggregate, bool) or not isinstance(aggregate, int) or \
            aggregate < 0 or aggregate != peak['vmrss_bytes']:
        return False, 'primary_metric_value_invalid'
    return True, ''


def write_atomic(path: Path, value: dict[str, Any]) -> None:
    """Write one final report using a same-directory atomic rename."""
    if path.exists():
        raise SamplerError(f'final sampler report already exists: {path}')
    part = path.with_name(path.name + '.part')
    if part.exists():
        raise SamplerError(f'sampler report part already exists: {part}')
    part.parent.mkdir(parents=True, exist_ok=True)
    value = dict(value)
    value['atomic'] = False
    part.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n',
                    encoding='utf-8')
    value['atomic'] = True
    part.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n',
                    encoding='utf-8')
    os.replace(part, path)


def _run(args: argparse.Namespace) -> int:
    try:
        # The sampler is deliberately background priority.  It still wakes at
        # the contractual 250 ms cadence and reads every process, but it does
        # not steal a timeslice from an 8-worker workload on a fully occupied
        # cpuset.  Positive nice is unprivileged; an already lower-priority
        # parent cannot be promoted back to the target without privilege, so
        # retain that effective priority and record both values.  The profile
        # target remains the value used for eligibility checks.
        inherited_nice = os.getpriority(os.PRIO_PROCESS, 0)
        if inherited_nice < args.scheduler_nice:
            os.nice(args.scheduler_nice - inherited_nice)
        actual_nice = os.getpriority(os.PRIO_PROCESS, 0)
    except OSError as error:
        raise SamplerError(f'unable to set sampler scheduler priority: {error}')
    if actual_nice < args.scheduler_nice:
        raise SamplerError(
            f'sampler scheduler priority below target: {actual_nice} < '
            f'{args.scheduler_nice}')
    stop = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    interval = args.interval_ms / 1000.0
    samples: list[dict[str, Any]] = []
    sample_errors = 0
    race_skips = 0
    pid_reuse_skips = 0
    no_rss_skips = 0
    missed = 0
    pid_start_times: dict[int, int] = {}
    next_deadline = time.monotonic()
    while not stop:
        sample = scan_proc(args.proc_root, pid_start_times=pid_start_times)
        samples.append(sample)
        sample_errors += int(sample['errors'])
        race_skips += int(sample['pid_race_skips'])
        pid_reuse_skips += int(sample.get('pid_reuse_skips', 0))
        no_rss_skips += int(sample.get('no_rss_skips', 0))
        next_deadline += interval
        now = time.monotonic()
        if now > next_deadline + interval * 1.5:
            missed += 1
            next_deadline = now
        time.sleep(max(0.0, next_deadline - time.monotonic()))
    report = summarize_samples(
        samples, requested_interval_seconds=interval,
        sampler_pid=os.getpid(), sample_errors=sample_errors,
        pid_race_skips=race_skips, min_samples=args.min_samples,
        max_errors=args.max_errors, max_race_skips=args.max_race_skips,
        max_jitter_percent=args.max_jitter_percent,
        missed_intervals=missed, stopped_by_signal=True,
        scheduler_nice=args.scheduler_nice)
    report['no_rss_skips'] = no_rss_skips
    report['pid_reuse_skips'] = pid_reuse_skips
    report['scheduler_nice_inherited'] = inherited_nice
    report['scheduler_nice_effective'] = actual_nice
    write_atomic(args.output, report)
    return 0 if report['status'] == 'pass' else 2


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--proc-root', type=Path, default=Path('/proc'))
    parser.add_argument('--interval-ms', type=float, default=250.0)
    parser.add_argument('--min-samples', type=int, default=2)
    parser.add_argument('--max-errors', type=int, default=100)
    parser.add_argument('--max-race-skips', type=int, default=100)
    parser.add_argument('--max-jitter-percent', type=float, default=100.0)
    parser.add_argument('--scheduler-nice', type=int, default=10)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    try:
        return _run(parse_args(argv))
    except (OSError, SamplerError, ValueError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
