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


"""Focused tests for the v10 formal-launcher preflight skeleton."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'run_fast_livo2_m6a10_v10_formal.py'
SPEC = importlib.util.spec_from_file_location('fast_livo2_m6a10_v10_formal', SCRIPT)
V10 = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = V10
SPEC.loader.exec_module(V10)


def _config(tmp_path: Path) -> object:
    return V10.FormalConfig(root=tmp_path / 'attempt', repo_root=ROOT)


def _quiescence_receipt(path: Path, *, status: str, allowed: bool) -> None:
    path.write_text(
        json.dumps(
            {
                'schema_version': 1,
                'contract_version': V10.QUIESCENCE_CONTRACT_VERSION,
                'status': status,
                'runner_start_allowed': allowed,
                'ground_truth_content_opened': False,
                'scorer_invoked': False,
                'observation': {
                    'sample_seconds': 5.0,
                    'nproc': 8,
                    'proc_race_skips': 0,
                    'forbidden_processes': [],
                    'checks': {'cpu': True, 'load': True},
                    'limits': {'max_cpu_busy_percent': 5.0, 'max_load1_per_cpu': 0.5},
                    'cpu': {'busy_percent': 1.0},
                    'loadavg': {'load1_per_cpu': 0.1},
                },
            }
        ),
        encoding='utf-8',
    )


def test_v10_immutable_constants_and_local_evidence_bindings():
    assert V10.PROFILE_SHA256 == (
        'f706f41b0a985347ff98e4953532590c7a9c8c2d75db355e53cfb558f95bc459'
    )
    assert V10.V10_PATCH_SHA256 == (
        '155bae4e37eac7d6220baedd861b9110b6139ee2b384d5b90db78ac24a6733c6'
    )
    assert V10.IMAGE_ID == (
        'sha256:3e087acc5ef116f03357a73927c18b2059068b093ae1cce6cb41c1baf1fbf759'
    )
    assert V10.BAG_BYTES == 11290464091
    assert V10.BAG_SHA256 == ('5bc7c6a0e5088aa3f733377a3e597705b075b1ec5ca5be272b75636a0b697310')
    assert V10.EXPECTED_MESSAGES == 236687
    assert V10.EXPECTED_TOPIC_COUNTS == {'lidar': 5793, 'imu': 225102, 'image': 5792}
    assert V10.REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS == 1623491515.148352
    assert V10.SENSOR_DURATION_SECONDS == 579.278127298
    assert V10.FEEDER_PATH == Path('scripts/fast_livo2_m6a10_feeder.py')
    assert V10.FEEDER_CONTAINER_PATH == '/runner/scripts/fast_livo2_m6a10_feeder.py'
    assert V10.FEEDER_SHA256 == (
        '1ea1c9bb9c625795c51c4b8c4b6aa81364370da17f8cac562f46d00af9691831'
    )
    assert V10.AUTHORIZATION_SHA256 == (
        '40f585ef747f5915cb20629001eeef176bd5f3d7b6a95feb19b93506b4da596f'
    )
    assert V10.AUTHORIZATION_PATH.name == 'formal_replay_authorization.receipt.json'
    assert V10.AUTHORIZED_ATTEMPT_ROOT.name == (
        'fast_livo2_v2c_v10_formal_replay3_20260823T112240Z_agentv10formal'
    )
    assert V10.EVIDENCE_RECEIPT_SHA256 == {
        'build_identity': 'a92e1d283f6787229c21be96b4f9f87564e13cf6ad8edda019ef096868139b96',
        'synthetic_gates': 'e41a44df9db8ed807aa14c2cc3540ba58f1ba0956e88be3b4151a9c3db4415f7',
        'no_input_handshake': 'a8b1b39fd1ad6807d328063b98c70037d3bca1d1c6bec8b68347c807ef7517a7',
        'binder_compositor': '8d65a05b581f9ddfbefbc7f88f29e0b002828911842a067dc3e0d19c7c778347',
    }


def test_identity_verification_is_source_bound_and_does_not_open_bag(tmp_path):
    config = _config(tmp_path)
    identity = V10.validate_preflight_identity(
        config,
        image_probe=lambda _config: {
            'tag': V10.IMAGE_TAG,
            'id': V10.IMAGE_ID,
            'docker_inspected': False,
        },
        authorization_validator=lambda _config: {
            'status': 'AUTHORIZED',
            'attempt_root': str(config.root),
        },
    )
    assert identity['status'] == 'PASS'
    assert identity['profile']['sha256'] == V10.PROFILE_SHA256
    assert identity['patch']['sha256'] == V10.V10_PATCH_SHA256
    assert identity['image']['id'] == V10.IMAGE_ID
    assert set(identity['evidence_receipts']) == {
        'build_identity',
        'synthetic_gates',
        'no_input_handshake',
        'binder_compositor',
    }
    assert all(item['status'] == 'PASS' for item in identity['evidence_receipts'].values())
    assert all(item['safety_verified'] is True for item in identity['evidence_receipts'].values())
    assert identity['bag']['opened'] is False
    assert identity['docker_inspected'] is False
    assert identity['ground_truth_content_opened'] is False
    assert identity['scorer_invoked'] is False


def test_identity_rejects_wrong_image_before_probe(tmp_path):
    config = V10.FormalConfig(
        root=tmp_path / 'attempt', repo_root=ROOT, image_id='sha256:' + '0' * 64
    )
    with pytest.raises(V10.LaunchError, match='image tag or ID is not pinned'):
        V10.validate_preflight_identity(
            config, image_probe=lambda _config: pytest.fail('probe must not run')
        )


def test_formal_authorization_is_exact_and_excludes_launcher_hash():
    authorization = json.loads(V10.AUTHORIZATION_PATH.read_text(encoding='utf-8'))
    checked = V10.verify_formal_authorization(
        V10.FormalConfig(root=V10.AUTHORIZED_ATTEMPT_ROOT, repo_root=ROOT)
    )
    assert checked['status'] == 'AUTHORIZED'
    assert checked['attempt_root'] == str(V10.AUTHORIZED_ATTEMPT_ROOT)
    assert checked['attempt_index'] == 3
    assert checked['retry_of_same_root'] is False
    assert checked['replay_count'] == 1
    assert checked['runner_start_count'] == 1
    assert 'launcher' not in authorization.get('sources', {})
    assert 'launcher_sha256' not in authorization
    assert authorization['prior_attempt']['closure_sha256'] == (
        'ec9459f4f76bf46873c1bd4e8f90299310e0449b4d82435dd0af3110cc2f5446'
    )
    assert authorization['prior_attempt']['cause'] == 'feeder_missing'
    assert authorization['prior_attempt']['exit_code'] == 1
    assert authorization['prior_attempt']['container_status'] == (
        'container_created_then_natural_exit_cleanup_removed'
    )
    assert authorization['authorized_delta']['output_mount'] == (
        'type=bind,dst=/out,readonly=false'
    )
    assert authorization['authorized_delta']['input_mount'].endswith(',readonly')
    assert authorization['authorized_delta']['wrapper_mount'].endswith(',readonly')
    assert authorization['authorized_delta']['feeder_mount'] == (
        'type=bind,dst=/runner/scripts/fast_livo2_m6a10_feeder.py,readonly'
    )
    assert authorization['sources']['feeder'] == {
        'path': 'scripts/fast_livo2_m6a10_feeder.py',
        'sha256': V10.FEEDER_SHA256,
    }


def _authorization_fixture(tmp_path, monkeypatch, mutate):
    value = json.loads(V10.AUTHORIZATION_PATH.read_text(encoding='utf-8'))
    mutate(value)
    path = tmp_path / 'authorization.receipt.json'
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    digest = V10.file_sha256(path)
    path.with_name(path.name + '.sha256').write_text(f'{digest}  {path.name}\n', encoding='ascii')
    monkeypatch.setattr(V10, 'AUTHORIZATION_PATH', path)
    monkeypatch.setattr(V10, 'AUTHORIZATION_SHA256', digest)


def test_formal_authorization_rejects_receipt_drift(tmp_path, monkeypatch):
    _authorization_fixture(
        tmp_path, monkeypatch, lambda value: value['profile'].__setitem__('sha256', '0' * 64)
    )
    with pytest.raises(V10.LaunchError) as error:
        V10.verify_formal_authorization(
            V10.FormalConfig(root=V10.AUTHORIZED_ATTEMPT_ROOT, repo_root=ROOT)
        )
    assert error.value.kind == 'AUTHORIZATION_FIELD_MISMATCH'


def test_formal_authorization_rejects_root_and_replay_reuse(tmp_path, monkeypatch):
    with pytest.raises(V10.LaunchError) as error:
        V10.verify_formal_authorization(_config(tmp_path))
    assert error.value.kind == 'AUTHORIZATION_ROOT_MISMATCH'

    _authorization_fixture(
        tmp_path, monkeypatch, lambda value: value['execution'].__setitem__('replay_count', 2)
    )
    with pytest.raises(V10.LaunchError) as error:
        V10.verify_formal_authorization(
            V10.FormalConfig(root=V10.AUTHORIZED_ATTEMPT_ROOT, repo_root=ROOT)
        )
    assert error.value.kind == 'AUTHORIZATION_FIELD_MISMATCH'


def test_formal_authorization_rejects_safety_drift(tmp_path, monkeypatch):
    _authorization_fixture(
        tmp_path, monkeypatch, lambda value: value['safety'].__setitem__('scorer_invoked', True)
    )
    with pytest.raises(V10.LaunchError) as error:
        V10.verify_formal_authorization(
            V10.FormalConfig(root=V10.AUTHORIZED_ATTEMPT_ROOT, repo_root=ROOT)
        )
    assert error.value.kind == 'AUTHORIZATION_FIELD_MISMATCH'


def test_formal_authorization_rejects_attempt_lineage_and_mount_drift(tmp_path, monkeypatch):
    mutations = (
        lambda value: value.__setitem__('attempt_index', 1),
        lambda value: value['prior_attempt'].__setitem__('closure_sha256', '0' * 64),
        lambda value: value['prior_attempt'].__setitem__('container_status', 'created'),
        lambda value: value['authorized_delta'].__setitem__(
            'output_mount', 'type=bind,dst=/out,rw'
        ),
        lambda value: value['authorized_delta'].__setitem__(
            'input_mount', 'type=bind,dst=/input/ntu_viral.bag'
        ),
        lambda value: value['authorized_delta'].__setitem__(
            'wrapper_mount', 'type=bind,dst=/runner/v10_runtime.sh,rw'
        ),
        lambda value: value['authorized_delta'].__setitem__(
            'feeder_mount', 'type=bind,dst=/runner/scripts/fast_livo2_m6a10_feeder.py,rw'
        ),
    )
    for index, mutate in enumerate(mutations):
        case = tmp_path / f'case-{index}'
        case.mkdir()
        _authorization_fixture(case, monkeypatch, mutate)
        with pytest.raises(V10.LaunchError) as error:
            V10.verify_formal_authorization(
                V10.FormalConfig(root=V10.AUTHORIZED_ATTEMPT_ROOT, repo_root=ROOT)
            )
        assert error.value.kind == 'AUTHORIZATION_FIELD_MISMATCH'


def test_formal_authorization_is_verified_before_bag_probe(monkeypatch):
    events = []

    def authorization(config):
        events.append('authorization')
        return V10.verify_formal_authorization(config)

    def bag_probe(_config):
        events.append('bag')
        return {
            'path': str(V10.BAG_PATH),
            'bytes': V10.BAG_BYTES,
            'sha256': V10.BAG_SHA256,
            'expected_messages': V10.EXPECTED_MESSAGES,
            'expected_topic_counts': dict(V10.EXPECTED_TOPIC_COUNTS),
            'required_evaluation_end_timestamp_seconds':
                V10.REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
            'opened': True,
        }

    V10.validate_preflight_identity(
        V10.FormalConfig(root=V10.AUTHORIZED_ATTEMPT_ROOT, repo_root=ROOT),
        image_probe=lambda _config: {
            'tag': V10.IMAGE_TAG,
            'id': V10.IMAGE_ID,
            'docker_inspected': False,
        },
        authorization_validator=authorization,
        bag_probe=bag_probe,
    )
    assert events == ['authorization', 'bag']


def test_evidence_receipts_are_verified_before_bag_probe(monkeypatch, tmp_path):
    monkeypatch.setitem(V10.EVIDENCE_RECEIPT_SHA256, 'build_identity', '0' * 64)
    bag_probe_called = []

    def bag_probe(_config):
        bag_probe_called.append(True)
        pytest.fail('bag probe must not run when evidence is unbound')

    with pytest.raises(V10.LaunchError, match='receipt SHA'):
        V10.validate_preflight_identity(_config(tmp_path), bag_probe=bag_probe)
    assert bag_probe_called == []


def test_fixed_safe_docker_argv_is_digest_bound_and_gt_blind(tmp_path):
    output = tmp_path / 'out'
    config = V10.FormalConfig(root=tmp_path / 'attempt', repo_root=ROOT)
    argv = V10.build_safe_docker_argv(config, output)
    assert argv[:2] == ['docker', 'run']
    assert '--name' in argv
    assert '--pull=never' in argv
    assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv
    assert '--init' in argv
    assert '--rm' not in argv
    assert V10.IMAGE_ID in argv
    assert V10.IMAGE_TAG not in argv
    assert argv[-2:] == ['/runner/v10_runtime.sh', V10.IMAGE_ID]
    assert '/bin/bash' not in argv and '/bin/sh' not in argv
    assert '-lc' not in argv and '-c' not in argv
    assert '--privileged' not in argv
    assert '--cap-add' not in argv
    assert '--device' not in argv
    assert argv.count('--mount') == 4
    mounts = [argv[index + 1] for index, value in enumerate(argv) if value == '--mount']
    input_mount = next(mount for mount in mounts if ',dst=/input/ntu_viral.bag,' in mount)
    output_mount = next(mount for mount in mounts if ',dst=/out,' in mount)
    wrapper_mount = next(mount for mount in mounts if ',dst=/runner/v10_runtime.sh,' in mount)
    feeder_mount = next(
        mount for mount in mounts if ',dst=' + V10.FEEDER_CONTAINER_PATH + ',' in mount
    )
    assert input_mount.endswith(',dst=/input/ntu_viral.bag,readonly')
    assert output_mount.endswith(',dst=/out,readonly=false')
    assert wrapper_mount.endswith(',dst=/runner/v10_runtime.sh,readonly')
    assert feeder_mount.endswith(',dst=' + V10.FEEDER_CONTAINER_PATH + ',readonly')
    assert not any(mount.endswith(',rw') for mount in mounts)
    assert not any('dst=/out,rw' in mount for mount in mounts)
    rendered = ' '.join(argv).lower()
    assert 'ground_truth' not in rendered
    assert 'scorer' not in rendered
    assert 'map' not in rendered
    assert 'taskset' not in rendered
    assert '--cpuset' not in rendered
    env = {
        argv[index + 1].split('=', 1)[0]: argv[index + 1].split('=', 1)[1]
        for index, value in enumerate(argv)
        if value == '--env'
    }
    assert env['M6A10_PHASE_CONTRACT_VERSION'] == V10.PHASE_CONTRACT_VERSION
    assert env['M6A10_PROFILE_SHA256'] == V10.PROFILE_SHA256
    assert env['M6A10_BAG_BYTES'] == str(V10.BAG_BYTES)
    assert env['M6A10_BAG_SHA256'] == V10.BAG_SHA256
    assert env['M6A10_FEEDER_TIMEOUT_SECONDS'] == '60'
    assert env['M6A10_FAST_FEEDER_SHA256'] == V10.FEEDER_SHA256
    assert env['M6A10_TERMINAL_DRAIN_TIMEOUT_SECONDS'] == '120'
    assert env['M6A10_CONSUMER_EVIDENCE'] != env['M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE']
    assert env['M6A10_SENSOR_DURATION_SECONDS'] == '579.278127298'
    assert env['M6A10_TIMING_CONTRACT_VERSION'] == V10.TIMING_CONTRACT_VERSION
    assert env['M6A10_ONLINE_TIMING_EVIDENCE'] == '/out/' + V10.TIMING_FILENAME


def test_safe_argv_rejects_feeder_drift_and_path_injection(tmp_path, monkeypatch):
    drifted = tmp_path / 'feeder.py'
    drifted.write_text('drifted feeder\n', encoding='utf-8')
    monkeypatch.setattr(V10, 'FEEDER_PATH', drifted)
    with pytest.raises(V10.LaunchError, match='feeder bytes differ'):
        V10.build_safe_docker_argv(
            V10.FormalConfig(root=tmp_path / 'attempt', repo_root=ROOT), tmp_path / 'out'
        )

    injected = tmp_path / 'feeder\ninjected.py'
    injected.write_bytes((ROOT / 'scripts' / 'fast_livo2_m6a10_feeder.py').read_bytes())
    monkeypatch.setattr(V10, 'FEEDER_PATH', injected)
    with pytest.raises(V10.LaunchError, match='invalid feeder path'):
        V10.build_safe_docker_argv(
            V10.FormalConfig(root=tmp_path / 'attempt2', repo_root=ROOT), tmp_path / 'out2'
        )


def test_safe_argv_rejects_non_pinned_bag_and_existing_output(tmp_path):
    with pytest.raises(V10.LaunchError, match='pinned bag path'):
        V10.build_safe_docker_argv(
            V10.FormalConfig(
                root=tmp_path / 'attempt', repo_root=ROOT, bag_path=tmp_path / 'other.bag'
            ),
            tmp_path / 'out',
        )
    output = tmp_path / 'out'
    output.mkdir()
    with pytest.raises(V10.LaunchError, match='must be fresh'):
        V10.build_safe_docker_argv(
            V10.FormalConfig(root=tmp_path / 'attempt', repo_root=ROOT), output
        )


def test_fresh_root_refuses_overwrite_and_preserves_sentinel(tmp_path):
    root = tmp_path / 'attempt'
    root.mkdir()
    sentinel = root / 'sentinel.txt'
    sentinel.write_text('keep\n', encoding='utf-8')
    with pytest.raises(V10.LaunchError, match='must be fresh') as error:
        V10.reserve_attempt_root(_config(tmp_path))
    assert error.value.kind == 'ROOT_ALREADY_EXISTS'
    assert sentinel.read_text(encoding='utf-8') == 'keep\n'
    assert not (root / 'attempt_root_marker.json').exists()


def test_quiescence_fail_closed_never_calls_runner(tmp_path):
    config = _config(tmp_path)
    calls: list[object] = []

    def failed_quiescence(_config, path):
        _quiescence_receipt(path, status='FAIL_CLOSED', allowed=False)
        return 1

    def should_not_start(*_args):
        calls.append(True)
        pytest.fail('runner hook must not start after quiescence FAIL_CLOSED')

    assert (
        V10.run_formal(
            config,
            identity_validator=lambda _config: {'status': 'PASS', 'bag_opened': False},
            quiescence_runner=failed_quiescence,
            runner_hook=should_not_start,
        )
        == 10
    )
    assert calls == []
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'PREFLIGHT_FAIL_CLOSED'
    assert closure['runner_start_attempted'] is False
    assert closure['bag_opened'] is False
    assert not (config.root / 'runner_hook_invocation.json').exists()


def test_passed_quiescence_exposes_only_injected_runner_hook(tmp_path):
    config = _config(tmp_path)
    calls: list[tuple[tuple[str, ...], object]] = []

    def passed_quiescence(_config, path):
        _quiescence_receipt(path, status='PASS', allowed=True)
        return 0

    def injected_runner(argv, received_config):
        calls.append((tuple(argv), received_config))
        return 17

    assert (
        V10.run_formal(
            config,
            identity_validator=lambda _config: {'status': 'PASS', 'bag_opened': False},
            quiescence_runner=passed_quiescence,
            runner_hook=injected_runner,
        )
        == 17
    )
    assert len(calls) == 1
    argv, received_config = calls[0]
    assert received_config is config
    assert argv[0:2] == ('docker', 'run')
    receipt = json.loads((config.root / 'runner_hook_receipt.json').read_text())
    assert receipt['status'] == 'INJECTED_RUNNER_HOOK_RETURNED'
    assert receipt['docker_run_started'] is False
    assert receipt['bag_opened'] is False


def _production_identity(*, input_opened=False) -> dict:
    value = {
        'status': 'PASS',
        'contract_id': V10.CONTRACT_ID,
        'profile': {'path': str(ROOT / V10.PROFILE_PATH), 'sha256': V10.PROFILE_SHA256},
        'patch': {'path': str(ROOT / V10.V10_PATCH_PATH), 'sha256': V10.V10_PATCH_SHA256},
        'sources': {
            'wrapper': {'path': str(ROOT / V10.WRAPPER_PATH), 'sha256': V10.WRAPPER_SHA256},
            'feeder': {'path': str(ROOT / V10.FEEDER_PATH), 'sha256': V10.FEEDER_SHA256},
            'quiescence': {
                'path': str(ROOT / V10.QUIESCENCE_SCRIPT),
                'sha256': V10.QUIESCENCE_SCRIPT_SHA256,
            },
        },
        'image': {'tag': V10.IMAGE_TAG, 'id': V10.IMAGE_ID, 'docker_inspected': True},
        'bag': {
            'path': str(V10.BAG_PATH),
            'bytes': V10.BAG_BYTES,
            'sha256': V10.BAG_SHA256,
            'expected_messages': V10.EXPECTED_MESSAGES,
            'expected_topic_counts': dict(V10.EXPECTED_TOPIC_COUNTS),
            'required_evaluation_end_timestamp_seconds':
                V10.REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
            'opened': False,
        },
        'evidence_receipts': {},
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    if input_opened:
        value['bag']['probe'] = {
            'path': str(V10.BAG_PATH),
            'bytes': V10.BAG_BYTES,
            'sha256': V10.BAG_SHA256,
            'expected_messages': V10.EXPECTED_MESSAGES,
            'expected_topic_counts': dict(V10.EXPECTED_TOPIC_COUNTS),
            'required_evaluation_end_timestamp_seconds':
                V10.REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
            'opened': True,
        }
    return value


class _FakeProcess:
    def __init__(self, *, returncode=0, running=False, events=None):
        self.pid = 4242
        self.returncode = returncode
        self.running = running
        self.events = events if events is not None else []

    def poll(self):
        self.events.append('poll')
        return None if self.running else self.returncode

    def wait(self, *args, **kwargs):
        self.events.append('wait')
        return self.returncode


def _fake_composed(output_path: Path) -> dict:
    counts = {
        key: dict(V10.EXPECTED_TOPIC_COUNTS)
        for key in ('expected', 'published', 'received', 'acknowledged')
    }
    value = {
        'schema_version': 3,
        'contract_version': V10.PHASE_CONTRACT_VERSION,
        'status': 'pass',
        'counts': counts,
        'required_evaluation_end_timestamp_seconds': V10.REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    output_path.write_text(json.dumps(value), encoding='utf-8')
    return value


def _lifecycle_hooks(tmp_path, *, process_returncode=0, running=False, oom=False, events=None):
    events = events if events is not None else []
    process = _FakeProcess(returncode=process_returncode, running=running, events=events)
    state = {'removed': False, 'oom': oom}

    def factory(_argv, **_kwargs):
        events.append('popen')
        return process

    def inspect(_name):
        events.append('inspect')
        if state['removed']:
            return {'status': 'not_found'}
        return {
            'status': 'present',
            'running': process.running,
            'state': 'running' if process.running else 'exited',
            'exit_code': process.returncode,
            'oom_killed': state['oom'],
        }

    def stats(_name):
        events.append('stats')
        return {'status': 'present', 'memory_usage_bytes': 4096}

    def top(_name):
        events.append('top')
        return {'status': 'present'}

    def stop(_name):
        events.append('stop')
        process.running = False
        return {'status': 'requested', 'returncode': 0}

    def remove(_name):
        events.append('remove')
        state['removed'] = True
        return {'status': 'removed'}

    return process, factory, inspect, stats, top, stop, remove, events


def _write_runtime_evidence(output_dir: Path) -> None:
    (output_dir / 'consumer_evidence.json').write_text('{}', encoding='utf-8')
    (output_dir / 'feeder_receipt.json').write_text('{}', encoding='utf-8')
    start = 1_000_000_000
    end = start + 750_000_000
    sensor_duration = V10.SENSOR_DURATION_SECONDS
    (output_dir / V10.TIMING_FILENAME).write_text(
        json.dumps(
            {
                'schema_version': V10.TIMING_SCHEMA_VERSION,
                'contract_version': V10.TIMING_CONTRACT_VERSION,
                'status': 'PASS',
                'input_start_monotonic_ns': start,
                'drain_end_monotonic_ns': end,
                'start_monotonic_ns': start,
                'end_monotonic_ns': end,
                'duration_seconds': 0.75,
                'sensor_duration_seconds': sensor_duration,
                'online_compute_rtf': 0.75 / sensor_duration,
                'ground_truth_content_opened': False,
                'scorer_invoked': False,
            }
        ),
        encoding='utf-8',
    )


def _fake_binder(raw_path, output_path, profile_path):
    output_path.write_text('{}', encoding='utf-8')
    return {'status': 'PASS', 'raw_path': str(raw_path), 'profile_path': str(profile_path)}


def _fake_compositor(*, profile_path, consumer_path, feeder_path, output_path):
    return _fake_composed(output_path)


def _timing_payload(*, start=1_000_000_000, end=1_750_000_000, **overrides):
    duration = (end - start) / 1.0e9
    value = {
        'schema_version': V10.TIMING_SCHEMA_VERSION,
        'contract_version': V10.TIMING_CONTRACT_VERSION,
        'status': 'PASS',
        'input_start_monotonic_ns': start,
        'drain_end_monotonic_ns': end,
        'start_monotonic_ns': start,
        'end_monotonic_ns': end,
        'duration_seconds': duration,
        'sensor_duration_seconds': V10.SENSOR_DURATION_SECONDS,
        'online_compute_rtf': duration / V10.SENSOR_DURATION_SECONDS,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    value.update(overrides)
    return value


def test_online_timing_positive_is_independent_runtime_authority(tmp_path):
    path = tmp_path / V10.TIMING_FILENAME
    path.write_text(json.dumps(_timing_payload()), encoding='utf-8')
    checked = V10.validate_online_timing(path)
    assert checked['online_compute_rtf'] == pytest.approx(0.75 / V10.SENSOR_DURATION_SECONDS)
    assert checked['sha256'] == V10.file_sha256(path)


@pytest.mark.parametrize(
    'overrides, kind',
    [
        ({'sensor_duration_seconds': 579.0}, 'TIMING_SENSOR_DURATION_MISMATCH'),
        ({'online_compute_rtf': float('nan')}, 'TIMING_RTF_INVALID'),
        ({'drain_end_monotonic_ns': 1_000_000_000}, 'TIMING_INVALID'),
        ({'ground_truth_content_opened': True}, 'TIMING_SAFETY_INVALID'),
    ],
)
def test_online_timing_negative_cases_fail_closed(tmp_path, overrides, kind):
    path = tmp_path / V10.TIMING_FILENAME
    path.write_text(json.dumps(_timing_payload(**overrides)), encoding='utf-8')
    with pytest.raises(V10.LaunchError) as error:
        V10.validate_online_timing(path)
    assert error.value.kind == kind


def test_online_timing_staging_and_overwrite_are_rejected(tmp_path):
    path = tmp_path / V10.TIMING_FILENAME
    path.write_text(json.dumps(_timing_payload()), encoding='utf-8')
    (tmp_path / (V10.TIMING_FILENAME + '.part')).write_text('staging', encoding='utf-8')
    with pytest.raises(V10.LaunchError) as error:
        V10.validate_online_timing(path)
    assert error.value.kind == 'TIMING_STAGING_PRESENT'


def test_default_image_probe_is_read_only_and_digest_bound(monkeypatch):
    calls = []

    class Result:
        returncode = 0
        stderr = ''
        stdout = json.dumps(
            {
                'Id': V10.IMAGE_ID,
                'RepoTags': [V10.IMAGE_TAG],
                'Config': {'Labels': {'portable': 'true'}},
            }
        )

    def fake_run(command, **kwargs):
        calls.append((tuple(command), kwargs))
        return Result()

    monkeypatch.setattr(V10.subprocess, 'run', fake_run)
    image = V10._default_image_probe(_config(Path('/tmp')))
    assert image['id'] == V10.IMAGE_ID
    assert calls[0][0] == ('docker', 'image', 'inspect', '--format', '{{json .}}', V10.IMAGE_ID)
    assert calls[0][1]['check'] is False


def test_default_bag_probe_stats_and_stream_hash_are_exact(monkeypatch, tmp_path):
    bag = tmp_path / 'probe.bag'
    payload = b'v10-test-bag'
    bag.write_bytes(payload)
    monkeypatch.setattr(V10, 'BAG_PATH', bag)
    monkeypatch.setattr(V10, 'BAG_BYTES', len(payload))
    monkeypatch.setattr(V10, 'BAG_SHA256', V10.file_sha256(bag))
    metadata = V10._default_bag_probe(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT, bag_path=bag)
    )
    assert metadata['bytes'] == len(payload)
    assert metadata['sha256'] == V10.file_sha256(bag)
    assert metadata['streaming_sha256'] is True


def test_receipts_are_verified_before_default_bag_probe(monkeypatch, tmp_path):
    calls = []
    original = V10.verify_evidence_receipts

    def fail_receipts():
        calls.append('receipts')
        raise V10.LaunchError('EVIDENCE_RECEIPT_SHA_MISMATCH', 'test drift')

    monkeypatch.setattr(V10, 'verify_evidence_receipts', fail_receipts)
    with pytest.raises(V10.LaunchError, match='test drift'):
        V10.validate_preflight_identity(
            _config(tmp_path),
            image_probe=lambda _config: pytest.fail('image probe must not run'),
            bag_probe=lambda _config: pytest.fail('bag probe must not run'),
        )
    assert calls == ['receipts']
    monkeypatch.setattr(V10, 'verify_evidence_receipts', original)


def test_production_success_popen_once_bind_compose_then_stopped_cleanup(tmp_path):
    events = []
    process, factory, inspect, stats, top, stop, remove, events = _lifecycle_hooks(
        tmp_path, events=events
    )

    def q(_config, path):
        _quiescence_receipt(path, status='PASS', allowed=True)
        events.append('quiescence')
        return 0

    def binder(raw, bound, profile):
        events.append('binder')
        return _fake_binder(raw, bound, profile)

    def compositor(**kwargs):
        events.append('compositor')
        return _fake_compositor(**kwargs)

    def factory_with_output(argv, **kwargs):
        result = factory(argv, **kwargs)
        _write_runtime_evidence(tmp_path / 'run' / 'out')
        return result

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(input_opened=True),
        quiescence_runner=q,
        process_factory=factory_with_output,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_stop=stop,
        container_remove=remove,
        binder=binder,
        compositor=compositor,
    )
    assert rc == 0
    assert events.count('popen') == 1
    assert events.index('binder') < events.index('remove')
    assert events.index('compositor') < events.index('remove')
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['status'] == 'PASS'
    assert closure['failure_kind'] is None
    assert closure['result']['rtf'] == pytest.approx(0.75 / V10.SENSOR_DURATION_SECONDS)
    assert closure['bag_opened'] is False
    assert closure['input_opened'] is False
    assert closure['timing']['sha256']
    assert closure['bindings']['timing']['path'].endswith(V10.TIMING_FILENAME)
    assert closure['command_sha256']
    assert (tmp_path / 'run' / 'closure_receipt.json.sha256').is_file()


def test_process_nonzero_is_fail_closed_and_no_retry(tmp_path):
    process, factory, inspect, stats, top, stop, remove, events = _lifecycle_hooks(
        tmp_path, process_returncode=7
    )
    calls = []

    def factory_once(argv, **kwargs):
        calls.append(argv)
        return factory(argv, **kwargs)

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory_once,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_remove=remove,
    )
    assert rc != 0
    assert len(calls) == 1
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'CONTAINER_PROCESS_FAILURE'
    assert closure['container']['diagnostics_before_cleanup']['state']['exit_code'] == 7


def test_oom_is_distinguished_from_generic_process_failure(tmp_path):
    process, factory, inspect, stats, top, stop, remove, _ = _lifecycle_hooks(
        tmp_path, process_returncode=137, oom=True
    )
    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_remove=remove,
    )
    assert rc != 0
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'CONTAINER_OOM'


def test_watchdog_diagnostics_precede_stop_and_cleanup(tmp_path):
    process, factory, inspect, stats, top, stop, remove, events = _lifecycle_hooks(
        tmp_path, running=True
    )
    clock_values = iter((0.0, 2.0))

    def clock():
        return next(clock_values, 2.0)

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT, watchdog_seconds=1.0),
        identity_validator=lambda _config: _production_identity(),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_stop=stop,
        container_remove=remove,
        clock=clock,
        sleep=lambda _seconds: None,
    )
    assert rc != 0
    assert events.index('top') < events.index('stop') < events.index('remove')
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'CONTAINER_WATCHDOG_TIMEOUT'
    assert closure['container']['diagnostics_before_stop']['oom_killed'] is False


def test_missing_raw_and_feeder_fail_closed_without_binding(tmp_path):
    process, factory, inspect, stats, top, stop, remove, _ = _lifecycle_hooks(tmp_path)
    binder_calls = []

    def binder(*args):
        binder_calls.append(args)
        pytest.fail('binder must not run when raw/feeder evidence is missing')

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_remove=remove,
        binder=binder,
    )
    assert rc != 0
    assert binder_calls == []
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'MISSING_RAW_EVIDENCE'


def test_binder_and_compositor_rejection_are_distinct(tmp_path):
    process, factory, inspect, stats, top, stop, remove, _ = _lifecycle_hooks(tmp_path)

    def factory_with_output(argv, **kwargs):
        result = factory(argv, **kwargs)
        _write_runtime_evidence(tmp_path / 'run' / 'out')
        return result

    def rejecting_binder(*_args):
        raise V10.LaunchError('BINDER_REJECT', 'raw contract rejected')

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory_with_output,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_remove=remove,
        binder=rejecting_binder,
    )
    assert rc != 0
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'BINDER_REJECT'


def test_compositor_rejection_is_fail_closed_after_binding(tmp_path):
    process, factory, inspect, stats, top, stop, remove, events = _lifecycle_hooks(tmp_path)

    def factory_with_output(argv, **kwargs):
        result = factory(argv, **kwargs)
        _write_runtime_evidence(tmp_path / 'run' / 'out')
        return result

    def compositor_reject(**_kwargs):
        raise V10.LaunchError('COMPOSITOR_REJECT', 'schema-v3 rejected')

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory_with_output,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_remove=remove,
        binder=_fake_binder,
        compositor=compositor_reject,
    )
    assert rc != 0
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'COMPOSITOR_REJECT'
    assert events.index('remove') > events.index('inspect')


def test_cleanup_refuses_running_or_unverified_container(tmp_path):
    process, factory, _inspect, stats, top, stop, remove, _ = _lifecycle_hooks(tmp_path)
    calls = []

    def still_running(_name):
        calls.append('inspect')
        return {'status': 'present', 'running': True, 'oom_killed': False}

    def factory_with_output(argv, **kwargs):
        result = factory(argv, **kwargs)
        _write_runtime_evidence(tmp_path / 'run' / 'out')
        return result

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory_with_output,
        container_inspect=still_running,
        container_stats=stats,
        container_top=top,
        container_remove=remove,
        binder=_fake_binder,
        compositor=_fake_compositor,
    )
    assert rc != 0
    assert 'remove' not in calls
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'CLEANUP_FAIL_CLOSED'


def test_missing_feeder_is_distinguished_from_missing_raw(tmp_path):
    process, factory, inspect, stats, top, stop, remove, _ = _lifecycle_hooks(tmp_path)

    def factory_with_raw_only(argv, **kwargs):
        result = factory(argv, **kwargs)
        (tmp_path / 'run' / 'out' / 'consumer_evidence.json').write_text('{}')
        return result

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory_with_raw_only,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_remove=remove,
        binder=_fake_binder,
    )
    assert rc != 0
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'MISSING_FEEDER_EVIDENCE'


def test_quiescence_default_command_is_invoked_once(monkeypatch, tmp_path):
    calls = []

    class Result:
        returncode = 0

    def fake_run(command, **kwargs):
        calls.append((tuple(command), kwargs))
        return Result()

    monkeypatch.setattr(V10.subprocess, 'run', fake_run)
    assert V10._run_quiescence(_config(tmp_path), tmp_path / 'q.json') == 0
    assert len(calls) == 1
    assert calls[0][0][1].endswith('check_m6a10_quiescence.py')


def test_preflight_to_popen_gap_fails_without_start_or_retry(monkeypatch, tmp_path):
    times = iter((0, 3_000_000_001))
    monkeypatch.setattr(V10.time, 'monotonic_ns', lambda: next(times, 3_000_000_001))
    starts = []

    def factory(*args, **kwargs):
        starts.append(True)
        pytest.fail('Popen must not start after a continuity-gap failure')

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory,
    )
    assert rc != 0
    assert starts == []
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'PREFLIGHT_RUN_GAP_FAIL_CLOSED'


def test_popen_invocation_delay_does_not_orphan_started_process(monkeypatch, tmp_path):
    events = []
    process, factory, inspect, stats, top, stop, remove, events = _lifecycle_hooks(
        tmp_path, events=events
    )
    stamps = iter((0, 1_000_000_000, 1_100_000_000, 9_000_000_000))
    monkeypatch.setattr(V10.time, 'monotonic_ns', lambda: next(stamps, 9_000_000_000))

    def factory_with_output(argv, **kwargs):
        value = factory(argv, **kwargs)
        _write_runtime_evidence(tmp_path / 'run' / 'out')
        return value

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(input_opened=True),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory_with_output,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_stop=stop,
        container_remove=remove,
        binder=_fake_binder,
        compositor=_fake_compositor,
    )
    assert rc == 0
    assert events.count('popen') == 1
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] is None
    assert closure['process']['popen_invocation_gap_ns'] == 7_900_000_000


def test_supervision_exception_captures_then_stops_once_and_removes(tmp_path):
    events = []
    process, factory, _inspect, _stats, _top, _stop, remove, events = _lifecycle_hooks(
        tmp_path, running=True, events=events
    )
    inspect_calls = {'count': 0}

    def inspect(name):
        events.append('inspect')
        inspect_calls['count'] += 1
        if inspect_calls['count'] == 1:
            raise RuntimeError('inspect exploded')
        if events.count('remove'):
            return {'status': 'not_found'}
        return {
            'status': 'present',
            'running': process.running,
            'state': 'running' if process.running else 'exited',
            'exit_code': process.returncode,
            'oom_killed': False,
        }

    def stats(name):
        events.append('stats')
        return {'status': 'present', 'memory_usage_bytes': 1}

    def top(name):
        events.append('top')
        return {'status': 'present'}

    def stop(name):
        events.append('stop')
        process.running = False
        return {'status': 'requested'}

    rc = V10.run_formal(
        V10.FormalConfig(root=tmp_path / 'run', repo_root=ROOT),
        identity_validator=lambda _config: _production_identity(input_opened=True),
        quiescence_runner=lambda _config, path: (
            _quiescence_receipt(path, status='PASS', allowed=True) or 0
        ),
        process_factory=factory,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_stop=stop,
        container_remove=remove,
    )
    assert rc == 70
    assert events.count('stop') == 1
    assert events.index('top') < events.index('stop') < events.index('remove')
    closure = json.loads((tmp_path / 'run' / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'CONTAINER_SUPERVISION_FAILURE'
    assert closure['cleanup']['status'] == 'removed'


def test_closure_sidecar_and_overwrite_refusal(tmp_path):
    root = tmp_path / 'run'
    root.mkdir()
    digest = V10._write_closure(root, kind='TEST_FAILURE', message='closed')
    sidecar = root / 'closure_receipt.json.sha256'
    assert digest == V10.file_sha256(root / 'closure_receipt.json')
    assert sidecar.read_text().startswith(digest + '  ')
    with pytest.raises(V10.LaunchError, match='overwrite'):
        V10._write_closure(root, kind='SECOND', message='must refuse')
