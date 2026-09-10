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


"""Focused tests for the read-only continuous M6a10 host monitor."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'monitor_m6a10_host_interference.py'
SPEC = importlib.util.spec_from_file_location(
    'm6a10_host_interference', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeClock:
    def __init__(self, *values: int):
        self.values = iter(values)

    def __call__(self) -> int:
        return next(self.values)


def _match(pid: int = 123, class_name: str = 'compiler') -> dict[str, object]:
    return {
        'pid': pid,
        'comm': 'redacted-build-worker',
        'class': class_name,
        'argv_sha256': 'a' * 64,
    }


def _monitor(tmp_path, *, sampler, clock, interval_seconds=5.0, owned_pids=()):
    return MODULE.HostInterferenceMonitor(
        tmp_path / 'samples.jsonl',
        interval_seconds=interval_seconds,
        sampler=sampler,
        clock=clock,
        utc_clock=lambda: '2026-08-24T00:00:00+00:00',
        launcher_pid=101,
        owned_pids=owned_pids,
    )


def _proc_tree(tmp_path, commands):
    proc = tmp_path / 'proc'
    proc.mkdir()
    (proc / 'loadavg').write_text('0.10 0.20 0.30 1/100 1\n')
    (proc / 'stat').write_text('cpu 100 0 0 90 0\n')
    for pid, command in commands.items():
        base = proc / str(pid)
        base.mkdir()
        (base / 'cmdline').write_bytes(command.encode() + b'\0')
        (base / 'comm').write_text(command.split()[0].split('/')[-1] + '\n')
        (base /
         'stat').write_text(f'{pid} (fake) S 1 ' +
                            ' '.join(['0'] *
                                     50) +
                            '\n')
    return proc


def _finish(monitor, tmp_path):
    monitor.start()
    monitor.stop()
    return monitor.finalize(tmp_path / 'summary.json')


def test_allowed_pids_include_launcher_ancestry_and_owned_runtime(tmp_path):
    proc = _proc_tree(tmp_path, {101: 'python3 launcher.py', 100: 'bash'})
    (proc / '101' / 'stat').write_text('101 (fake) S 100 ' +
                                       ' '.join(['0'] * 50) + '\n')
    (proc / '100' / 'stat').write_text('100 (fake) S 1 ' +
                                       ' '.join(['0'] * 50) + '\n')
    allowed = MODULE.allowed_pids(
        proc_root=proc,
        launcher_pid=101,
        owned_pids=(
            202,
            303))
    assert {100, 101, 202, 303}.issubset(allowed)


@pytest.mark.parametrize('command,expected', [
    ('/usr/bin/g++ -c x.cc', 'compiler'),
    ('docker build --tag x .', 'docker_build'),
    ('colcon build --cmake-args', 'colcon_build'),
    ('cmake --build build', 'cmake_build'),
    ('cargo build --release', 'cargo_build'),
    ('ninja build', 'generic_build'),
    ('make -j4', 'generic_build'),
])
def test_late_build_classes_reuse_quiescence_detector(
        tmp_path, command, expected):
    proc = _proc_tree(tmp_path, {123: command})
    observed = MODULE._default_sampler(proc, set())
    assert observed['forbidden_processes'][0]['class'] == expected
    assert 'argv' not in observed['forbidden_processes'][0]


def test_start_end_snapshots_and_summary_hashes_are_immutable(tmp_path):
    monitor = _monitor(
        tmp_path,
        sampler=lambda _allowed: {
            'forbidden_processes': [],
            'proc_race_count': 0},
        clock=FakeClock(
            0,
            1_000_000_000),
    )
    result = _finish(monitor, tmp_path)
    rows = [
        json.loads(line) for line in (
            tmp_path /
            'samples.jsonl').read_text().splitlines()]
    assert [row['phase'] for row in rows] == ['run_start', 'run_end']
    assert [row['order'] for row in rows] == [0, 1]
    assert result['status'] == 'PASS'
    assert result['samples_sha256'] == hashlib.sha256(
        (tmp_path / 'samples.jsonl').read_bytes()).hexdigest()
    sidecar = tmp_path / 'summary.json.sha256'
    summary_sha = hashlib.sha256((tmp_path / 'summary.json').read_bytes()).hexdigest()
    assert sidecar.read_text() == f'{summary_sha}  summary.json\n'
    assert (os.stat(tmp_path / 'samples.jsonl').st_mode & 0o777) == 0o444
    assert (os.stat(tmp_path / 'summary.json').st_mode & 0o777) == 0o444
    assert (os.stat(sidecar).st_mode & 0o777) == 0o444


def test_transient_forbidden_process_is_sticky_and_late_detection_is_preserved(
        tmp_path):
    observations = iter([
        {'forbidden_processes': [], 'proc_race_count': 0},
        {'forbidden_processes': [_match(class_name='docker_build')], 'proc_race_count': 1},
        {'forbidden_processes': [], 'proc_race_count': 0},
    ])
    monitor = _monitor(
        tmp_path,
        sampler=lambda _allowed: next(observations),
        clock=FakeClock(
            0,
            1_000_000_000,
            2_000_000_000,
            3_000_000_000))
    monitor.start()
    monitor.sample_once('interval')
    monitor.sample_once('interval')
    monitor.stop()
    result = monitor.finalize(tmp_path / 'summary.json')
    assert result['contaminated'] is True
    assert result['status'] == 'FAIL_CLOSED'
    assert result['forbidden_sample_count'] == 1
    assert result['proc_race_count'] == 1


def test_callback_exception_is_sticky_invalid_but_natural_completion_is_not_interrupted(
        tmp_path):
    calls = []

    def sampler(_allowed):
        calls.append('sample')
        if len(calls) == 2:
            raise RuntimeError('injected sampler failure')
        return {'forbidden_processes': [], 'proc_race_count': 0}

    monitor = _monitor(tmp_path, sampler=sampler,
                       clock=FakeClock(0, 1_000_000_000, 2_000_000_000))
    monitor.start()
    monitor.sample_once('interval')
    monitor.stop()
    result = monitor.finalize(tmp_path / 'summary.json')
    assert result['invalid'] is True
    assert result['status'] == 'FAIL_CLOSED'
    assert any(row.get('sample_valid') is False for row in monitor.samples)
    assert calls == ['sample', 'sample', 'sample']


def test_proc_race_count_is_aggregated_and_allowed_pid_is_passed(tmp_path):
    seen = []

    def sampler(allowed):
        seen.append(set(allowed))
        return {'forbidden_processes': [], 'proc_race_count': 4}

    monitor = _monitor(
        tmp_path, sampler=sampler, clock=FakeClock(
            0, 1_000_000_000), owned_pids=(
            222,))
    result = _finish(monitor, tmp_path)
    assert result['proc_race_count'] == 8
    assert all(222 in item for item in seen)


def test_late_owned_runner_pid_is_thread_safe_and_audited(tmp_path):
    seen = []

    def sampler(allowed):
        seen.append(set(allowed))
        return {'forbidden_processes': [], 'proc_race_count': 0}

    monitor = _monitor(
        tmp_path,
        sampler=sampler,
        clock=FakeClock(0, 100_000_000, 200_000_000, 300_000_000),
    )
    monitor.start()
    monitor.allow_owned_pid(707)
    monitor.sample_once('interval')
    monitor.stop()
    with pytest.raises(MODULE.InterferenceError):
        monitor.allow_owned_pid(708)
    result = monitor.finalize(tmp_path / 'summary.json')
    assert 707 not in seen[0]
    assert 707 in seen[1] and 707 in seen[2]
    assert result['owned_pid_allow_events'][0]['pid'] == 707
    rows = [
        json.loads(line) for line in (
            tmp_path /
            'samples.jsonl').read_text().splitlines()]
    assert rows[1]['owned_pid_events'][0]['pid'] == 707
    with pytest.raises(MODULE.InterferenceError):
        monitor.allow_owned_pid(709)
    with pytest.raises(MODULE.InterferenceError):
        monitor.allow_owned_pid(0)
    with pytest.raises(MODULE.InterferenceError):
        monitor.allow_owned_pid(True)


def test_coverage_gap_and_invalid_interval_are_fail_closed(tmp_path):
    monitor = _monitor(
        tmp_path,
        sampler=lambda _allowed: {
            'forbidden_processes': [],
            'proc_race_count': 0},
        clock=FakeClock(
            0,
            10_000_000_000),
        interval_seconds=1.0,
    )
    monitor.start()
    monitor.stop()
    with pytest.raises(MODULE.InterferenceError, match='coverage gap'):
        monitor.finalize(tmp_path / 'summary.json')
    assert not (tmp_path / 'summary.json').exists()
    with pytest.raises(MODULE.InterferenceError):
        MODULE.HostInterferenceMonitor(
            tmp_path / 'other.jsonl', interval_seconds=0.0)
    with pytest.raises(MODULE.InterferenceError):
        MODULE.HostInterferenceMonitor(
            tmp_path / 'third.jsonl', interval_seconds=5.1)


def test_coverage_boundary_is_strictly_bounded_and_cadence_is_retained(
        tmp_path):
    accepted = _monitor(
        tmp_path / 'accepted',
        sampler=lambda _allowed: {
            'forbidden_processes': [],
            'proc_race_count': 0},
        clock=FakeClock(
            0,
            5_000_000_000),
        interval_seconds=5.0,
    )
    accepted_result = _finish(accepted, tmp_path / 'accepted')
    assert accepted_result['coverage']['max_gap_seconds'] == 5.0
    assert accepted_result['coverage']['max_allowed_gap_seconds'] == 5.0
    four_second = _monitor(
        tmp_path / 'four',
        sampler=lambda _allowed: {
            'forbidden_processes': [],
            'proc_race_count': 0},
        clock=FakeClock(
            0,
            5_000_000_000),
        interval_seconds=4.0,
    )
    four_result = _finish(four_second, tmp_path / 'four')
    assert four_result['coverage']['max_allowed_gap_seconds'] == 5.0
    rejected = _monitor(
        tmp_path / 'over',
        sampler=lambda _allowed: {
            'forbidden_processes': [],
            'proc_race_count': 0},
        clock=FakeClock(
            0,
            5_000_000_001),
        interval_seconds=5.0,
    )
    rejected.start()
    rejected.stop()
    with pytest.raises(MODULE.InterferenceError, match='coverage gap'):
        rejected.finalize(tmp_path / 'over' / 'summary.json')
    one_second = _monitor(
        tmp_path / 'one',
        sampler=lambda _allowed: {
            'forbidden_processes': [],
            'proc_race_count': 0},
        clock=FakeClock(
            0,
            4_000_000_000),
        interval_seconds=1.0,
    )
    one_second.start()
    one_second.stop()
    with pytest.raises(MODULE.InterferenceError, match='coverage gap'):
        one_second.finalize(tmp_path / 'one' / 'summary.json')


def test_overwrite_and_symlink_destinations_are_rejected(tmp_path):
    existing = tmp_path / 'existing.jsonl'
    existing.write_text('sealed\n')
    with pytest.raises(MODULE.InterferenceError):
        MODULE.HostInterferenceMonitor(
            existing,
            sampler=lambda _allowed: {},
            clock=FakeClock(
                0,
                1)).start()
    target = tmp_path / 'target.jsonl'
    target.write_text('target\n')
    link = tmp_path / 'link.jsonl'
    link.symlink_to(target)
    with pytest.raises(MODULE.InterferenceError):
        MODULE.HostInterferenceMonitor(
            link, sampler=lambda _allowed: {}).start()


def test_monitor_has_no_process_control_or_authority_fields():
    source = SCRIPT.read_text()
    assert 'subprocess' not in source
    assert 'signal' not in source
    assert 'os.kill' not in source
    assert "'ground_truth" not in source
    assert "'scorer" not in source
    assert "'input" not in source
    assert 'os.fsync' in source
    assert 'threading.Thread' in source
