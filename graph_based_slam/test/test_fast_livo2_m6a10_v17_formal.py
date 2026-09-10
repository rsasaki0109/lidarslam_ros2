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


"""Injected v17 formal lifecycle tests; no Docker, bag, ROS, GT, or scorer."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'm6a10_v17_formal_test_module', ROOT / 'scripts/run_fast_livo2_m6a10_v17_formal.py'
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _authorization(tmp_path: Path):
    path = tmp_path / 'authorization.receipt.json'
    path.write_text('{"status":"AUTHORIZED"}\n', encoding='utf-8')
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return path, digest


def _callback():
    return json.loads(
        (
            ROOT / 'graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json'
        ).read_text()
    )


def _terminal(*, residual_lidar: bool = False):
    received = dict(MODULE.EXPECTED_COUNTS)
    completed = {'lidar': 5793, 'imu': 225024, 'image': 5789}
    buffers = {
        'lidar': {
            'count': 1 if residual_lidar else 0,
            'records': [{'topic': 'lidar'}] if residual_lidar else [],
        },
        'imu': {'count': 78, 'records': [{'topic': 'imu'}] * 78},
        'image': {'count': 3, 'records': [{'topic': 'image'}] * 3},
    }
    if residual_lidar:
        completed['lidar'] = 5792
    return {
        'status': 'pass',
        'contract_id': MODULE.TERMINAL_CONTRACT,
        'received_topic_counts': received,
        'completed_counts': completed,
        'backend': {'completed_counts': completed},
        'buffers': buffers,
        'terminal_support_context': {
            'by_topic': {'lidar': 0, 'imu': 78, 'image': 3},
            'total_count': 81,
            'records': [{'topic': 'imu'}] * 78 + [{'topic': 'image'}] * 3,
        },
    }


class _Monitor:
    def __init__(self, events):
        self.events = events
        self._started = False

    def start(self):
        self.events.append('monitor_start')
        self._started = True

    def allow_owned_pid(self, pid):
        self.events.append(('allow_owned_pid', pid))

    def stop(self):
        self.events.append('monitor_stop')
        self._started = False

    def finalize(self, path):
        self.events.append('monitor_finalize')
        return {
            'status': 'PASS',
            'contaminated': False,
            'invalid': False,
            'coverage': {'coverage_gap': False},
            'summary_path': str(path),
        }


class _Process:
    pid = 1717

    def __init__(self, events, returncode=0):
        self.events = events
        self.returncode = returncode

    def wait(self):
        self.events.append('wait')
        return self.returncode


def _runtime(tmp_path: Path, *, residual_lidar: bool = False):
    auth_path, auth_sha = _authorization(tmp_path)
    events = []
    terminal = _terminal(residual_lidar=residual_lidar)

    def authorize(config):
        assert config.authorization_path == auth_path
        assert config.authorization_sha256 == auth_sha
        return {'authorized': True, 'formal_execution': True}

    def quiescence(_config):
        return {'status': 'PASS', 'window_count': 3, 'consecutive_passes': 3, 'windows': []}

    def identity(_config):
        events.append('identity')
        return {'image_id': MODULE.IMAGE_ID, 'tag': MODULE.IMAGE_TAG, 'opened': False}

    def bag(_config):
        events.append('bag_probe')
        return {'path': 'pinned-but-not-opened', 'bytes': 0, 'sha256': 'not-read', 'opened': False}

    def process(argv, _cwd):
        events.append(('popen', list(argv)))
        return _Process(events)

    def monitor_factory(_root):
        return _Monitor(events)

    def capture(_config, _root):
        return {
            'terminal': terminal,
            'documents': {
                'feeder': {'schema_version': 1, 'status': 'PASS'},
                'callback': _callback(),
                'terminal': terminal,
                'timing': {'schema_version': 1, 'status': 'PASS', 'rtf': 1.0},
            },
            'bindings': {'callback': {'sha256': 'callback'}, 'terminal': {'sha256': 'terminal'}},
        }

    def compose(raw, _config):
        events.append('compose')
        return {
            'status': 'PASS',
            'contract_id': MODULE.PHASE_CONTRACT,
            'transport': 'callback',
            'terminal_support': raw['terminal']['terminal_support_context'],
        }

    return (
        auth_path,
        auth_sha,
        events,
        {
            'authorization_validator': authorize,
            'quiescence_probe': quiescence,
            'identity_probe': identity,
            'bag_probe': bag,
            'process_factory': process,
            'monitor_factory': monitor_factory,
            'raw_capture': capture,
            'composer': compose,
        },
    )


def test_v17_profile_and_immutable_receipts_are_verified():
    verified = MODULE.verify_candidate_profile()
    assert verified['sha256'] == MODULE.PROFILE_SHA256
    assert verified['image_id'] == MODULE.IMAGE_ID
    assert (
        MODULE.READY_PROFILE_SHA256
        == hashlib.sha256(MODULE.READY_PROFILE_PATH.read_bytes()).hexdigest()
    )
    assert (
        MODULE.BUILD_RECEIPT_SHA256
        == hashlib.sha256(MODULE.BUILD_RECEIPT_PATH.read_bytes()).hexdigest()
    )
    assert (
        MODULE.NO_INPUT_PASS_RECEIPT_SHA256
        == hashlib.sha256(MODULE.NO_INPUT_PASS_RECEIPT_PATH.read_bytes()).hexdigest()
    )


def test_v17_builder_single_call_safe_mounts_and_embedded_profile(tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'attempt', repo_root=MODULE.ROOT)
    original = MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV
    calls = []

    def counted(value, output):
        calls.append((value, output))
        return original(value, output)

    MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = counted
    try:
        argv = MODULE.build_safe_docker_argv(config, tmp_path / 'out')
    finally:
        MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = original
    assert len(calls) == 1
    mounts = [argv[i + 1] for i, item in enumerate(argv[:-1]) if item == '--mount']
    assert {MODULE._mount_destination(spec) for spec in mounts} == {
        '/input/ntu_viral.bag',
        '/out',
        MODULE.WRAPPER_PATH_IN_CONTAINER,
        MODULE.FEEDER_PATH_IN_CONTAINER,
    }
    assert sum(spec.endswith(',dst=/out,readonly=false') for spec in mounts) == 1
    assert MODULE._tmpfs_destinations(argv) == ['/tmp', '/root/.ros']
    assert '/out' not in MODULE._tmpfs_destinations(argv)
    assert argv[-1] == MODULE.IMAGE_ID
    assert '--rm' not in argv and 'rw' not in argv
    env = [argv[i + 1] for i, item in enumerate(argv[:-1]) if item == '--env']
    assert (
        'M6A10_PROFILE_PATH=configs/slam_benchmark_profiles/'
        'fast_livo2_m6a10_v17_formal_candidate.yaml'
        in env
    )
    assert 'M6A10_PROFILE_SHA256=' + MODULE.READY_PROFILE_SHA256 in env
    assert 'M6A10_V17_AUTHORIZATION_STATUS=AUTHORIZED_FOR_EXACT_ROOT' not in env
    assert any(
        item == '--entrypoint' and argv[i + 1] == MODULE.WRAPPER_PATH_IN_CONTAINER
        for i, item in enumerate(argv[:-1])
    )

    authorized = MODULE.v12.CandidateConfig(
        root=tmp_path / 'authorized-attempt',
        repo_root=MODULE.ROOT,
        authorization_path=tmp_path / 'authorization.receipt.json',
        authorization_sha256='a' * 64,
    )
    authorized_argv = MODULE.build_safe_docker_argv(authorized, tmp_path / 'authorized-out')
    authorized_env = [
        authorized_argv[i + 1] for i, item in enumerate(authorized_argv[:-1]) if item == '--env'
    ]
    assert 'M6A10_V17_AUTHORIZATION_STATUS=AUTHORIZED_FOR_EXACT_ROOT' in authorized_env


def test_v17_main_injected_valid_terminal_support_tail_passes_and_seals(tmp_path, capsys):
    root = tmp_path / 'fresh-attempt'
    auth_path, auth_sha, events, runtime = _runtime(tmp_path)
    args = [
        '--root',
        str(root),
        '--authorization',
        str(auth_path),
        '--authorization-sha256',
        auth_sha,
    ]
    assert MODULE.main(args, runtime=runtime) == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed['status'] == 'PASS'
    closure = json.loads((root / 'closure_receipt.json').read_text())
    assert closure['status'] == 'PASS'
    assert closure['v17_contract_version'] == MODULE.CONTRACT_VERSION
    assert closure['v17_v12_contract_snapshot'] == {
        'phase_contract': MODULE.PHASE_CONTRACT,
        'transport_contract': MODULE.TRANSPORT_CONTRACT,
    }
    assert closure['execution']['popen_count'] == 1
    assert closure['execution']['one_start'] is True
    assert closure['safety']['input_opened'] is False
    assert events.count('identity') == 1 and events.count('bag_probe') == 1
    assert events.count('wait') == 1 and events.count('compose') == 1
    assert events.count('monitor_start') == 1 and events.count('monitor_finalize') == 1
    assert len([item for item in events if isinstance(item, tuple) and item[0] == 'popen']) == 1
    sidecar = root / 'closure_receipt.json.sha256'
    assert sidecar.read_bytes() == (
        '%s  closure_receipt.json\n'
        % hashlib.sha256((root / 'closure_receipt.json').read_bytes()).hexdigest()
    ).encode('ascii')


def test_v17_sticky_invalid_terminal_conservation_seals_fail_closed(tmp_path, capsys):
    root = tmp_path / 'sticky-invalid'
    auth_path, auth_sha, events, runtime = _runtime(tmp_path, residual_lidar=True)
    args = [
        '--root',
        str(root),
        '--authorization',
        str(auth_path),
        '--authorization-sha256',
        auth_sha,
    ]
    assert MODULE.main(args, runtime=runtime) == 1
    capsys.readouterr()
    closure = json.loads((root / 'closure_receipt.json').read_text())
    assert closure['status'] == 'FAIL_CLOSED'
    assert closure['failure_kind'] == 'CONSERVATION_RESIDUAL_LIDAR'
    assert closure['execution']['one_start'] is True
    assert closure['execution']['retry'] is False
    assert events.count('wait') == 1 and events.count('compose') == 0


def test_v17_default_main_rejects_unauthorized_before_root(tmp_path):
    root = tmp_path / 'never-created'
    assert MODULE.main(['--root', str(root)]) == 11
    assert not root.exists()


def test_v17_source_has_no_formal_runtime_execution_in_default_test_path():
    text = (
        (ROOT / 'scripts/run_fast_livo2_m6a10_v17_formal.py').read_text(encoding='utf-8').lower()
    )
    assert 'rosbag play' not in text
    assert 'subprocess.run(["docker"' not in text
