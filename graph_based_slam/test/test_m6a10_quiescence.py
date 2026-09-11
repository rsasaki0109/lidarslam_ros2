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

"""Unit tests for the read-only M6a10 quiescence preflight."""

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_m6a10_quiescence.py'
SPEC = importlib.util.spec_from_file_location('m6a10_quiescence', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _proc_tree(tmp_path, *, cmdlines=None, parents=None, stat_cpu=None,
               loadavg='0.10 0.20 0.30 1/100 1'):
    proc = tmp_path / 'proc'
    proc.mkdir()
    (proc / 'stat').write_text(
        'cpu ' + ' '.join(str(item) for item in (stat_cpu or [100, 0, 0, 90, 0])) + '\n')
    (proc / 'loadavg').write_text(loadavg + '\n')
    for pid, command in (cmdlines or {}).items():
        base = proc / str(pid)
        base.mkdir()
        (base / 'cmdline').write_bytes(command.encode() + b'\0')
        (base / 'comm').write_text(command.split()[0].split('/')[-1] + '\n')
        ppid = (parents or {}).get(pid, 1)
        stat_tail = ' '.join(['0'] * 50)
        (base / 'stat').write_text(
            f'{pid} (fake) S {ppid} {stat_tail}\n')
    return proc


def _no_sleep(_seconds):
    return None


def test_cpu_busy_ratio_and_load_are_strict():
    before = {'total_jiffies': 100, 'idle_jiffies': 90}
    after = {'total_jiffies': 200, 'idle_jiffies': 150}
    assert MODULE.cpu_busy_percent(before, after) == 40.0
    with pytest.raises(MODULE.QuiescenceError):
        MODULE.cpu_busy_percent(before, {'total_jiffies': 100, 'idle_jiffies': 80})


def test_forbidden_build_process_is_detected(tmp_path):
    proc = _proc_tree(tmp_path, cmdlines={123: 'docker build --tag test .'})
    matches, races = MODULE.find_forbidden_processes(proc, exclude_pids={999})
    assert races == 0
    assert matches[0]['class'] == 'docker_build'
    assert 'docker build' not in json.dumps(matches)


def test_compiler_and_colcon_classes_are_detected(tmp_path):
    proc = _proc_tree(
        tmp_path,
        cmdlines={123: '/usr/bin/g++ -c x.cc', 124: 'colcon build --cmake-args'})
    matches, _ = MODULE.find_forbidden_processes(proc)
    assert {item['class'] for item in matches} == {'compiler', 'colcon_build'}


def test_ancestors_are_excluded_to_avoid_self_false_positive(tmp_path):
    proc = _proc_tree(
        tmp_path, cmdlines={10: 'bash -lc docker build .', 11: 'python3 check.py'},
        parents={11: 10})
    matches, _ = MODULE.find_forbidden_processes(proc, exclude_pids={10, 11})
    assert matches == []


def test_observation_passes_with_quiet_fake_proc(tmp_path, monkeypatch):
    proc = _proc_tree(tmp_path, stat_cpu=[100, 0, 0, 90, 0])
    snapshots = iter([
        {'total_jiffies': 100, 'idle_jiffies': 90},
        {'total_jiffies': 200, 'idle_jiffies': 190},
    ])
    monkeypatch.setattr(MODULE, 'read_cpu_jiffies',
                        lambda _proc_root: next(snapshots))
    observation = MODULE.collect_observation(
        proc_root=proc, sample_seconds=1, max_busy_percent=5,
        max_load_per_cpu=0.5, nproc=8,
        sleep=_no_sleep, monotonic=lambda: next(iter([0.0, 1.0])),
        excluded_pids=set())
    assert observation['nproc'] == 8
    assert observation['loadavg']['load1_per_cpu'] < 0.5
    assert observation['checks']['no_forbidden_processes'] is True


def test_observation_rejects_invalid_thresholds(tmp_path):
    proc = _proc_tree(tmp_path)
    with pytest.raises(MODULE.QuiescenceError):
        MODULE.collect_observation(
            proc_root=proc, sample_seconds=0, sleep=_no_sleep)


def test_receipt_is_atomic_and_refuses_overwrite(tmp_path):
    output = tmp_path / 'quiescence.json'
    observation = {
        'checks': {
            'nproc_positive': True,
            'cpu_busy_within_limit': True,
            'load1_per_cpu_within_limit': True,
            'no_forbidden_processes': True,
        },
    }
    receipt = MODULE.build_receipt(observation, now='2026-08-22T00:00:00+00:00')
    MODULE._atomic_write(output, receipt)
    assert json.loads(output.read_text())['status'] == 'PASS'
    with pytest.raises(MODULE.QuiescenceError):
        MODULE._atomic_write(output, receipt)


def test_pass_receipt_requires_contract_and_sha(tmp_path):
    output = tmp_path / 'quiescence.json'
    receipt = MODULE.build_receipt({
        'checks': {
            'nproc_positive': True,
            'cpu_busy_within_limit': True,
            'load1_per_cpu_within_limit': True,
            'no_forbidden_processes': True,
        },
    }, now='2026-08-22T00:00:00+00:00')
    MODULE._atomic_write(output, receipt)
    digest = MODULE.receipt_sha256(output)
    assert MODULE.require_pass_receipt(output, digest)['status'] == 'PASS'
    with pytest.raises(MODULE.QuiescenceError):
        MODULE.require_pass_receipt(output, '0' * 64)


def test_fail_receipt_blocks_runner(tmp_path):
    output = tmp_path / 'quiescence.json'
    receipt = MODULE.build_receipt({
        'checks': {
            'nproc_positive': True,
            'cpu_busy_within_limit': False,
            'load1_per_cpu_within_limit': True,
            'no_forbidden_processes': True,
        },
    }, now='2026-08-22T00:00:00+00:00')
    MODULE._atomic_write(output, receipt)
    with pytest.raises(MODULE.QuiescenceError):
        MODULE.require_pass_receipt(output)
