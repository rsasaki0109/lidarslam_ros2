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


"""
Injected-only tests for the additive v12 formal candidate.

No test opens the benchmark input or starts Docker/ROS.  The lifecycle uses a
fake process, monitor, raw capture, and compositor so ordering and fail-closed
authority are tested entirely on host-generated observations.
"""

from __future__ import annotations

import copy
import hashlib
import importlib.util
from pathlib import Path
import stat
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_fast_livo2_m6a10_v12_formal.py'
SPEC = importlib.util.spec_from_file_location('m6a10_v12_formal_candidate', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _positive_raw():
    completed = {'lidar': 5793, 'imu': 225000, 'image': 5780}
    buffer = {'lidar': 0, 'imu': 102, 'image': 12}
    support_records = [{'topic': 'imu', 'record_id': 'imu-%d' % i} for i in range(102)] + [
        {'topic': 'image', 'record_id': 'image-%d' % i} for i in range(12)
    ]
    return {
        'system': 'fast_livo2',
        'phase_mode': MODULE.PHASE_MODE,
        'received_topic_counts': dict(MODULE.EXPECTED_COUNTS),
        'completed_counts': completed,
        'backend': {'completed_counts': dict(completed)},
        'buffers': {
            topic: {
                'count': buffer[topic],
                'records': (
                    [
                        {'topic': topic, 'record_id': '%s-%d' % (topic, i)}
                        for i in range(buffer[topic])
                    ]
                ),
            }
            for topic in MODULE.EXPECTED_COUNTS
        },
        'terminal_support_context': {
            'by_topic': {'lidar': 0, 'imu': 102, 'image': 12},
            'total_count': 114,
            'records': support_records,
        },
    }


def _config(tmp_path):
    return MODULE.CandidateConfig(root=tmp_path / 'candidate-root', repo_root=ROOT)


def test_candidate_profile_and_all_external_lineage_pins_are_read_only():
    verified = MODULE.verify_candidate_profile()
    assert verified['sha256'] == MODULE.PROFILE_SHA256
    assert verified['profile_key'] == MODULE.PROFILE_KEY
    assert MODULE.READY_PROFILE_PATH.is_file()
    assert (
        hashlib.sha256(MODULE.READY_PROFILE_PATH.read_bytes()).hexdigest()
        == MODULE.READY_PROFILE_SHA256
    )
    assert (
        MODULE.PROFILE_PATH.read_text(encoding='utf-8').find('formal_replay_forbidden: true') >= 0
    )
    assert MODULE.BUILD_RECEIPT_PATH.is_file()
    assert MODULE.ATTEMPT6_RECEIPT_PATH.is_file()
    assert MODULE.HOST_GATE_RECEIPT_PATH.is_file()


def test_default_authorization_fails_before_any_probe_or_root_creation(tmp_path):
    root = tmp_path / 'must-not-be-created'
    calls = []

    def forbidden_probe(_config):
        calls.append('probe')
        raise AssertionError('unauthorized candidate accessed a probe')

    with pytest.raises(MODULE.AuthorizationError) as error:
        MODULE.run_formal(
            MODULE.CandidateConfig(root=root),
            identity_probe=forbidden_probe,
            bag_probe=forbidden_probe,
        )
    assert error.value.kind == 'FORMAL_REPLAY_UNAUTHORIZED'
    assert calls == []
    assert not root.exists()


def test_docker_argv_is_fixed_shell_free_and_has_four_safe_mounts(tmp_path):
    argv = MODULE.build_safe_docker_argv(
        MODULE.CandidateConfig(root=tmp_path / 'root'),
        tmp_path / 'out',
    )
    assert argv[0:3] == ['docker', 'run', '--name']
    assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv
    assert '--pull=never' in argv and '--init' in argv
    assert '--rm' not in argv
    assert '--entrypoint' in argv
    assert argv[-1] == MODULE.IMAGE_ID
    mounts = [argv[i + 1] for i, item in enumerate(argv[:-1]) if item == '--mount']
    assert len(mounts) == 4
    assert any('dst=/input/ntu_viral.bag,readonly' in mount for mount in mounts)
    assert any('dst=/out,readonly=false' in mount for mount in mounts)
    assert any('dst=/runner/v12_runtime.sh,readonly' in mount for mount in mounts)
    assert any(
        'dst=/runner/scripts/fast_livo2_m6a10_feeder.py,readonly' in mount for mount in mounts
    )
    assert all(mount != 'rw' and not mount.endswith(',rw') for mount in mounts)
    assert all('shell' not in item.lower() for item in argv)


def test_topic_conservation_positive_and_explicit_negative_categories():
    raw = _positive_raw()
    result = MODULE.validate_topic_conservation(raw)
    assert result['completed'] == result['backend_completed']
    assert result['buffer'] == {'lidar': 0, 'imu': 102, 'image': 12}
    assert result['support'] == {'lidar': 0, 'imu': 102, 'image': 12}

    mutations = []
    value = copy.deepcopy(raw)
    value['completed_counts']['imu'] += 1
    mutations.append(value)
    value = copy.deepcopy(raw)
    value['backend']['completed_counts']['image'] += 1
    mutations.append(value)
    value = copy.deepcopy(raw)
    value['buffers']['imu']['count'] += 1
    mutations.append(value)
    value = copy.deepcopy(raw)
    value['buffers']['image']['records'].pop()
    mutations.append(value)
    value = copy.deepcopy(raw)
    value['terminal_support_context']['by_topic']['imu'] -= 1
    mutations.append(value)
    value = copy.deepcopy(raw)
    value['buffers']['lidar'] = {'count': 1, 'records': [{'topic': 'lidar'}]}
    mutations.append(value)
    for mutation in mutations:
        with pytest.raises(MODULE.CandidateError):
            MODULE.validate_topic_conservation(mutation)


class _FakeMonitor:
    def __init__(self, events, *, contaminated=False, coverage_gap=False):
        self.events = events
        self.contaminated = contaminated
        self.coverage_gap = coverage_gap
        self._started = False

    def start(self):
        self.events.append('monitor.start')
        self._started = True

    def allow_owned_pid(self, pid):
        self.events.append(('monitor.allow_owned_pid', pid))

    def stop(self):
        self.events.append('monitor.stop')

    def finalize(self, _path):
        self.events.append('monitor.finalize')
        return {
            'status': 'FAIL_CLOSED' if self.contaminated or self.coverage_gap else 'PASS',
            'contaminated': self.contaminated,
            'invalid': False,
            'coverage': {'coverage_gap': self.coverage_gap, 'max_gap_seconds': 4.0},
        }


class _FakeProcess:
    pid = 4242

    def __init__(self, events, returncode=0):
        self.events = events
        self.returncode = returncode

    def wait(self):
        self.events.append('process.wait')
        return self.returncode


def _run_injected(tmp_path, *, contaminated=False, coverage_gap=False, returncode=0):
    events = []
    monitor = _FakeMonitor(events, contaminated=contaminated, coverage_gap=coverage_gap)
    raw = _positive_raw()

    def monitor_factory(_root):
        events.append('monitor.factory')
        return monitor

    def process_factory(argv, cwd):
        events.append(('popen', list(argv), cwd))
        return _FakeProcess(events, returncode=returncode)

    def capture(_config, _root):
        events.append('raw.capture')
        return raw

    def compose(_raw, _config):
        events.append('compose')
        return {'status': 'PASS', 'transport': {'outstanding': 0}}

    result = MODULE.run_formal(
        _config(tmp_path),
        authorization_validator=lambda _config: {'authorized': True, 'injected': True},
        identity_probe=lambda _config: {'image_id': MODULE.IMAGE_ID},
        bag_probe=lambda _config: {'path': MODULE.INPUT_PATH, 'opened': False},
        process_factory=process_factory,
        monitor_factory=monitor_factory,
        raw_capture=capture,
        composer=compose,
        now='2026-08-24T00:00:00+00:00',
    )
    return result, events


def test_injected_lifecycle_has_one_shell_free_start_and_conservation_before_compose(tmp_path):
    result, events = _run_injected(tmp_path)
    assert result['status'] == 'PASS'
    assert result['execution']['one_start'] is True
    assert result['execution']['popen_count'] == 1
    assert events.index('monitor.start') < next(
        i for i, v in enumerate(events) if isinstance(v, tuple) and v[0] == 'popen'
    )
    assert (
        events.index('process.wait')
        < events.index('monitor.finalize')
        < events.index('raw.capture')
        < events.index('compose')
    )
    receipt = Path(result['receipt_path'])
    sidecar = Path(result['sidecar_path'])
    assert stat.S_IMODE(receipt.stat().st_mode) == 0o444
    assert stat.S_IMODE(sidecar.stat().st_mode) == 0o444
    receipt_sha = hashlib.sha256(receipt.read_bytes()).hexdigest()
    assert sidecar.read_text(encoding='ascii') == '%s  %s\n' % (receipt_sha, receipt.name)
    assert result['safety'] == {
        'formal_replay_started': False,
        'input_opened': False,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'map_saved': False,
    }


def test_interference_is_sticky_fail_closed_without_stopping_process(tmp_path):
    result, events = _run_injected(tmp_path, contaminated=True)
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'HOST_INTERFERENCE_CONTAMINATED'
    assert 'process.wait' in events
    assert events.index('raw.capture') < events.index('compose')
    assert 'monitor.stop' in events and 'monitor.finalize' in events


def test_coverage_gap_and_process_failure_are_fail_closed_without_retry(tmp_path):
    result, events = _run_injected(tmp_path, coverage_gap=True)
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'HOST_INTERFERENCE_COVERAGE_INVALID'
    assert events.count('process.wait') == 1

    result, events = _run_injected(tmp_path / 'nonzero', returncode=31)
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'PROCESS_FAILURE'
    assert result['execution']['retry'] is False
    assert events.count('process.wait') == 1


def test_static_candidate_has_no_shell_replay_or_process_control_path():
    text = SCRIPT.read_text(encoding='utf-8').lower()
    assert 'subprocess.popen' in text
    assert 'shell=false' in text
    assert 'monitor.start()' in text
    assert 'monitor.allow_owned_pid' in text
    assert 'monitor.finalize' in text
    assert 'rosbag play' not in text
    assert '.terminate(' not in text
    assert '.kill(' not in text
    assert 'os.kill' not in text
    assert 'signal.' not in text
    assert 'formal_replay_authorized' in text
