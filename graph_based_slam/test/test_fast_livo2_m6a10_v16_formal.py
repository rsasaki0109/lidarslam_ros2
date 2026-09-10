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


"""Injected v16 lifecycle tests; Docker, ROS, bag, GT, scorer, and maps stay unused."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'm6a10_v16_formal_test', ROOT / 'scripts/run_fast_livo2_m6a10_v16_formal.py'
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _mounts(argv):
    return [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == '--mount']


def _authorization(tmp_path):
    path = tmp_path / 'authorization.receipt.json'
    value = {
        'schema_version': 1,
        'contract_version': 'm6a10-v16-test-authorization-v1',
        'status': 'AUTHORIZED',
        'authorized': True,
        'formal_execution': True,
        'formal_replay_started': False,
        'attempt_count': 1,
        'retry': False,
        'manual_stop': False,
    }
    path.write_text(json.dumps(value, sort_keys=True) + '\n', encoding='utf-8')
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    sidecar = path.with_name(path.name + '.sha256')
    sidecar.write_bytes(('%s  %s\n' % (digest, path.name)).encode('ascii'))

    def validator(config):
        assert config.authorization_path == path
        assert hashlib.sha256(path.read_bytes()).hexdigest() == config.authorization_sha256
        assert sidecar.read_bytes() == ('%s  %s\n' % (digest, path.name)).encode('ascii')
        return {'authorized': True, 'formal_execution': True, 'receipt': value}

    return path, digest, validator


class _Monitor:
    def __init__(self, root):
        self.root = root
        self.events = []
        self._started = False

    def start(self):
        self.events.append('start')
        self._started = True

    def allow_owned_pid(self, pid):
        self.events.append(('allow', pid))

    def stop(self):
        self.events.append('stop')
        self._started = False

    def finalize(self, path):
        self.events.append('finalize')
        return {
            'status': 'PASS',
            'contaminated': False,
            'invalid': False,
            'coverage': {'coverage_gap': False},
            'summary_path': str(path),
        }


class _Process:
    pid = 816016

    def __init__(self, events, returncode=0):
        self.events = events
        self.returncode = returncode

    def wait(self):
        self.events.append('wait')
        return self.returncode


def _quiescence(_config):
    return {
        'status': 'PASS',
        'window_count': 3,
        'consecutive_passes': 3,
        'windows': [],
    }


def _terminal_raw():
    counts = dict(MODULE.EXPECTED_COUNTS)
    return {
        'received_topic_counts': counts,
        'completed_counts': counts,
        'backend': {'completed_counts': counts},
        'buffers': {
            'lidar': {'count': 0, 'records': []},
            'imu': {'count': 0, 'records': []},
            'image': {'count': 0, 'records': []},
        },
        'terminal_support_context': {
            'by_topic': {'lidar': 0, 'imu': 0, 'image': 0},
            'total_count': 0,
            'records': [],
        },
    }


def _runtime(tmp_path, *, process_factory=None, events=None, failing=False):
    auth_path, auth_sha, validator = _authorization(tmp_path)
    events = [] if events is None else events
    callback = json.loads(
        (
            ROOT / 'graph_based_slam/test/fixtures/fast_livo2_m6a10_v12_consumer_status_pass.json'
        ).read_text()
    )
    terminal = _terminal_raw()

    def capture(_config, root):
        output = root / 'out'
        output.mkdir(exist_ok=True)
        artifacts = {
            'feeder_receipt.json': {'schema_version': 1, 'status': 'PASS'},
            'callback_consumer_evidence.json': callback,
            'consumer_evidence.json': terminal,
            'online_compute_timing.json': {'schema_version': 1, 'status': 'PASS', 'rtf': 1.0},
        }
        bindings = {}
        for name, value in artifacts.items():
            raw = (json.dumps(value, sort_keys=True) + '\n').encode('utf-8')
            path = output / name
            path.write_bytes(raw)
            bindings[name] = {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()}
        (output / 'feeder.log').write_text('fake feeder stderr\n', encoding='utf-8')
        (output / 'feeder_exit_status.txt').write_text('0\n', encoding='ascii')
        return {
            'terminal': terminal,
            'documents': {
                'feeder': artifacts['feeder_receipt.json'],
                'callback': callback,
                'terminal': terminal,
                'timing': artifacts['online_compute_timing.json'],
            },
            'bindings': bindings,
            'feeder_persistence': {
                'receipt_present': True,
                'stderr_present': True,
                'exit_status_present': True,
            },
        }

    def identity(_config):
        events.append('identity')
        return {
            'image_id': MODULE.IMAGE_ID,
            'tag': MODULE.IMAGE_TAG,
            'opened': False,
            'fake_inspect': True,
        }

    def bag(_config):
        events.append('bag_probe')
        return {'path': 'fake-input-not-opened', 'bytes': 0, 'sha256': 'fake', 'opened': False}

    def process(argv, _cwd):
        events.append(('popen', list(argv)))
        if failing:
            raise RuntimeError('injected Popen failure')
        return _Process(events)

    def compose(raw, _config):
        assert raw['documents']['callback']['schema_version'] == 3
        assert (
            raw['documents']['callback']['transport_contract_version'] == MODULE.TRANSPORT_CONTRACT
        )
        return {'status': 'PASS', 'contract': MODULE.PHASE_CONTRACT, 'fake_composition': True}

    return {
        'authorization_path': auth_path,
        'authorization_sha256': auth_sha,
        'authorization_validator': validator,
        'quiescence_probe': _quiescence,
        'identity_probe': identity,
        'bag_probe': bag,
        'process_factory': process if process_factory is None else process_factory,
        'monitor_factory': _Monitor,
        'raw_capture': capture,
        'composer': compose,
    }


def _runtime_options(runtime):
    return {
        key: value
        for key, value in runtime.items()
        if key not in {'authorization_path', 'authorization_sha256'}
    }


def test_v16_snapshot_uses_only_real_v12_contract_symbols():
    assert not hasattr(MODULE.v12, 'CONTRACT_VERSION')
    snapshot = MODULE._snapshot_v12_state()
    assert 'CONTRACT_VERSION' not in snapshot
    assert 'RECEIPT_NAME' not in snapshot
    assert snapshot['PHASE_CONTRACT'] == MODULE.v12.PHASE_CONTRACT
    assert snapshot['TRANSPORT_CONTRACT'] == MODULE.v12.TRANSPORT_CONTRACT
    assert MODULE.snapshot_v12_contracts() == {
        'phase_contract': MODULE.PHASE_CONTRACT,
        'transport_contract': MODULE.TRANSPORT_CONTRACT,
    }


def test_v16_profile_and_v15_failure_lineage_are_pinned():
    profile = MODULE.verify_candidate_profile()
    assert profile['sha256'] == MODULE.PROFILE_SHA256
    assert profile['phase_contract'] == MODULE.PHASE_CONTRACT
    assert profile['transport_contract'] == MODULE.TRANSPORT_CONTRACT
    assert (
        MODULE.V15_LAUNCHER_SHA256
        == hashlib.sha256(MODULE.V15_LAUNCHER_PATH.read_bytes()).hexdigest()
    )
    assert (
        MODULE.V15_PROFILE_SHA256
        == hashlib.sha256(MODULE.V15_PROFILE_PATH.read_bytes()).hexdigest()
    )


def test_v16_builder_calls_original_once_and_keeps_safe_mount_contract(tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'attempt')
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
    destinations = [MODULE._mount_destination(item) for item in _mounts(argv)]
    assert destinations == [
        '/input/ntu_viral.bag',
        '/out',
        MODULE.WRAPPER_PATH_IN_CONTAINER,
        MODULE.FEEDER_PATH_IN_CONTAINER,
    ]
    assert sum(item.endswith(',dst=/out,readonly=false') for item in _mounts(argv)) == 1
    assert MODULE._tmpfs_destinations(argv) == ['/tmp', '/root/.ros']
    assert argv[-1] == MODULE.IMAGE_ID
    assert '--rm' not in argv and 'rw' not in argv


def test_v16_main_end_to_end_fake_authorization_popen_artifacts_and_closure(tmp_path, capsys):
    root = tmp_path / 'fresh-attempt'
    events = []
    runtime = _runtime(tmp_path, events=events)
    args = [
        '--root',
        str(root),
        '--authorization',
        str(runtime['authorization_path']),
        '--authorization-sha256',
        runtime['authorization_sha256'],
    ]
    assert MODULE.main(args, runtime=_runtime_options(runtime)) == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed['status'] == 'PASS'
    closure = json.loads((root / 'closure_receipt.json').read_text(encoding='utf-8'))
    assert closure['status'] == 'PASS'
    assert closure['v16_contract_version'] == MODULE.CONTRACT_VERSION
    assert closure['v16_v12_contract_snapshot'] == {
        'phase_contract': MODULE.PHASE_CONTRACT,
        'transport_contract': MODULE.TRANSPORT_CONTRACT,
    }
    assert closure['execution']['popen_count'] == 1
    assert closure['execution']['one_start'] is True
    assert closure['safety']['input_opened'] is False
    assert closure['safety']['ground_truth_content_opened'] is False
    assert closure['safety']['scorer_invoked'] is False
    assert closure['safety']['map_saved'] is False
    assert events.count('identity') == 1
    assert events.count('bag_probe') == 1
    assert events.count('wait') == 1
    assert (
        len([event for event in events if isinstance(event, tuple) and event[0] == 'popen']) == 1
    )
    sidecar = root / 'closure_receipt.json.sha256'
    assert sidecar.read_bytes() == (
        '%s  closure_receipt.json\n'
        % hashlib.sha256((root / 'closure_receipt.json').read_bytes()).hexdigest()
    ).encode('ascii')


def test_v16_exception_path_seals_fail_closed_and_restores_v12(tmp_path):
    root = tmp_path / 'exception-attempt'
    events = []
    runtime = _runtime(tmp_path, events=events, failing=True)
    before = MODULE.v12.build_safe_docker_argv
    config = MODULE.v12.CandidateConfig(
        root=root,
        profile_path=MODULE.PROFILE_PATH,
        authorization_path=runtime['authorization_path'],
        authorization_sha256=runtime['authorization_sha256'],
    )
    result = MODULE.run_formal(config, **_runtime_options(runtime))
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'PROCESS_LIFECYCLE_FAILURE'
    assert result['execution']['popen_count'] == 0
    assert (root / 'closure_receipt.json').is_file()
    assert MODULE.v12.build_safe_docker_argv is before


def test_v16_default_main_rejects_missing_authorization_before_root(tmp_path):
    root = tmp_path / 'never-created'
    assert MODULE.main(['--root', str(root)]) == 11
    assert not root.exists()


def test_v16_missing_authorization_receipt_fails_before_root_or_popen(tmp_path):
    root = tmp_path / 'missing-auth-attempt'
    missing = tmp_path / 'missing-auth.json'
    assert (
        MODULE.main(
            [
                '--root',
                str(root),
                '--authorization',
                str(missing),
                '--authorization-sha256',
                '0' * 64,
            ]
        )
        == 11
    )
    assert not root.exists()


def test_v16_preexisting_root_is_rejected_without_popen(tmp_path):
    root = tmp_path / 'already-there'
    root.mkdir()
    runtime = _runtime(tmp_path)
    args = [
        '--root',
        str(root),
        '--authorization',
        str(runtime['authorization_path']),
        '--authorization-sha256',
        runtime['authorization_sha256'],
    ]
    assert MODULE.main(args, runtime=_runtime_options(runtime)) == 11
    assert not (root / 'closure_receipt.json').exists()
