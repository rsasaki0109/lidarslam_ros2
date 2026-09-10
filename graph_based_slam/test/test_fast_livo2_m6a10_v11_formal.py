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
Focused v11 formal-launcher gates.

These tests inspect pinned source/receipt contracts and exercise only injected
lifecycle seams.  They never invoke the production main, Docker, ROS, bag
probes, formal replay, GT, scorer, or map code.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_fast_livo2_m6a10_v11_formal.py'
V10_SCRIPT = ROOT / 'scripts/run_fast_livo2_m6a10_v10_formal.py'
SPEC = importlib.util.spec_from_file_location('fast_livo2_m6a10_v11_formal', SCRIPT)
V11 = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = V11
SPEC.loader.exec_module(V11)
AUTHORIZATION_SOURCE_PATH = Path(
    '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260823/'
    'fast_livo2_v2c_v11_formal_authorization_20260823T140331Z/'
    'formal_replay_authorization.receipt.json'
)


def _config(tmp_path: Path) -> Any:
    return V11.FormalConfig(root=tmp_path / 'attempt', repo_root=ROOT)


def _quiescence_receipt(path: Path, *, status: str, allowed: bool) -> None:
    path.write_text(
        json.dumps(
            {
                'schema_version': 1,
                'contract_version': V11.QUIESCENCE_CONTRACT_VERSION,
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


def _identity() -> dict[str, Any]:
    return {
        'status': 'PASS',
        'contract_id': V11.CONTRACT_ID,
        'profile': {'path': str(V11.PROFILE_PATH), 'sha256': V11.PROFILE_SHA256},
        'patch': {'path': str(V11.V11_PATCH_PATH), 'sha256': V11.V11_PATCH_SHA256},
        'image': {'tag': V11.IMAGE_TAG, 'id': V11.IMAGE_ID},
        'bag': {
            'path': str(V11.BAG_PATH),
            'bytes': V11.BAG_BYTES,
            'sha256': V11.BAG_SHA256,
            'expected_messages': V11.EXPECTED_MESSAGES,
            'expected_topic_counts': dict(V11.EXPECTED_TOPIC_COUNTS),
            'required_evaluation_end_timestamp_seconds':
                V11.REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
            'opened': False,
        },
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def test_v11_constants_and_v10_lineage_are_immutable():
    assert V11.PHASE_CONTRACT_VERSION == ('m6a10-online-compute-v4-terminal-bounded-end-gap')
    assert V11.PROFILE_PATH == Path(
        'configs/slam_benchmark_profiles/fast_livo2_m6a10_v11_formal_ready.yaml'
    )
    assert V11.PROFILE_SHA256 == (
        '6f56068a8a283851aa3a3723d2c9b526235bc97ba507a7dee4ea72bcd461bd2d'
    )
    assert V11.V11_PATCH_PATH == Path(
        'docker/patches/fast_livo2.m6a10-v2c-v11-bounded-end-gap.patch'
    )
    assert V11.V11_PATCH_SHA256 == (
        '2cbca3a7bb465981cb5e3072efd1713302e002127d4ae5872413ed5749d80ba2'
    )
    assert V11.WRAPPER_PATH == Path('scripts/fast_livo2_m6a10_v11_formal_container_run.sh')
    assert V11.WRAPPER_SHA256 == (
        'a769d5f81dc52c64dbeed88c4f43525a3f2bac1069d29fbe40967bf40a768aeb'
    )
    assert V11.IMAGE_TAG == (
        'm6a10-v2c-v11-bounded-end-gap-20260823t121749z-fast-livo2-benchmark:' 'ros1-pinned'
    )
    assert V11.IMAGE_ID == (
        'sha256:729a7bba2127fc6517c106d59a12668294aeee1a0b31d04d31f9c25c762f6c3a'
    )
    assert V11.EVIDENCE_RECEIPT_SHA256 == {
        'build_identity': '98ceaba27849e23491e19f8f6c7c3392306cd9ff22cfa87048a1810a7b83c8b6',
        'synthetic_gates': 'eaac94b7572330e09fe889d4868691eb075fbb33c1aabae60d45686055626790',
        'dual_service': '03f29eeb07eb6fde6f93996081333b5cfc54e3a6c920f38168f259790a52ec64',
        'binder_compositor': '19db8b5e7308108443b4df0d1f878996b5acda1d1f9a69356ba4c1ba6edc1fb7',
    }
    assert V11.AUTHORIZATION_SHA256 == (
        '6a6665307d2e44426213b002a625c540a37fbb4ea2d03657f8147843a811700a'
    )
    assert V11.AUTHORIZATION_PATH.name == 'formal_replay_authorization.receipt.json'
    assert V11.AUTHORIZED_ATTEMPT_ROOT.name == (
        'fast_livo2_v2c_v11_formal_replay_20260823T140331Z_agentv11formal'
    )
    assert V11.ENVIRONMENT_PRECONDITION_WINDOW_COUNT == 3
    assert V11.ENVIRONMENT_PRECONDITION_CONSECUTIVE_PASS_COUNT == 3
    assert V11.ENVIRONMENT_PRECONDITION_MAX_BUSY_PERCENT == 5.0
    assert V11.ENVIRONMENT_PRECONDITION_MAX_LOAD1_PER_CPU == 0.5
    assert len(V11.ENVIRONMENT_PRECONDITION_WINDOWS) == 3
    assert V11.AUTHORIZATION_CONTRACT_ID == ('m6a10-v2c-v11-formal-replay-authorization-v1')
    assert V11.AUTHORIZATION_RECEIPT_KIND == ('m6a10_v11_formal_replay_authorization')
    assert V11.file_sha256(V10_SCRIPT) == (
        '09e5833ed0e6d73fc22b99154e0349fa4c8d7acdb81c3b5244b115fa4dc1296d'
    )
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'AUTHORIZED_ATTEMPT_ROOT' in source
    assert 'AUTHORIZATION_PATH' in source
    assert 'AUTHORIZATION_NOT_INSTALLED' not in source
    assert 'def verify_formal_authorization' in source


def test_all_four_actual_receipts_are_strictly_verified():
    checked = V11.verify_evidence_receipts()
    assert set(checked) == {
        'build_identity',
        'synthetic_gates',
        'dual_service',
        'binder_compositor',
    }
    assert all(value['safety_verified'] is True for value in checked.values())
    assert checked['synthetic_gates']['production_gate']['final_status'] == 'PASS'
    assert checked['dual_service']['terminal_fail_closed_expected'] is True


def test_authorization_fails_before_image_or_bag_probe(tmp_path, monkeypatch):
    events: list[str] = []
    monkeypatch.setattr(V11, 'verify_evidence_receipts', lambda: {})
    config = _config(tmp_path)

    def image_probe(_config):
        events.append('image')
        pytest.fail('image probe must not run before missing authorization')

    def bag_probe(_config):
        events.append('bag')
        pytest.fail('bag probe must not run before missing authorization')

    with pytest.raises(V11.LaunchError) as error:
        V11.validate_preflight_identity(config, image_probe=image_probe, bag_probe=bag_probe)
    assert error.value.kind == 'AUTHORIZATION_ROOT_MISMATCH'
    assert events == []


def test_exact_v11_authorization_is_verified_without_bag_or_image_probe():
    checked = V11.verify_formal_authorization(
        V11.FormalConfig(root=V11.AUTHORIZED_ATTEMPT_ROOT, repo_root=ROOT)
    )
    assert checked['status'] == 'AUTHORIZED'
    assert checked['attempt_index'] == 4
    assert checked['attempt_root'] == str(V11.AUTHORIZED_ATTEMPT_ROOT)
    assert checked['retry_of_same_root'] is False
    assert checked['replay_count'] == 1
    assert checked['runner_start_count'] == 1
    assert checked['execution']['mounts']['count'] == 4
    assert checked['sources'].keys() == {'wrapper', 'feeder', 'binder', 'compositor', 'quiescence'}
    assert checked['gate_receipts']['synthetic_gates']['status'] == (
        'PASS_WITH_PREFLIGHT_FAILURES_RESOLVED'
    )
    assert checked['gate_receipts']['dual_service']['status'] == ('FAIL_CLOSED_EXPECTED')
    environment = checked['environment_precondition']
    assert environment['window_count'] == 3
    assert environment['consecutive_pass_count'] == 3
    assert all(window['status'] == 'PASS' for window in environment['windows'])
    assert all(window['runner_start_allowed'] is True for window in environment['windows'])
    assert [window['cpu_busy_percent'] for window in environment['windows']] == [
        2.697252331736829,
        2.5725094577553596,
        3.302243508948828,
    ]
    assert [window['load1_per_cpu'] for window in environment['windows']] == [
        0.05625,
        0.04875,
        0.055,
    ]


def _authorization_fixture(tmp_path, monkeypatch, mutate):
    value = json.loads(AUTHORIZATION_SOURCE_PATH.read_text(encoding='utf-8'))
    mutate(value)
    path = tmp_path / 'formal_replay_authorization.receipt.json'
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    digest = V11.file_sha256(path)
    sidecar = path.with_name(path.name + '.sha256')
    sidecar.write_text(f'{digest}  {path.name}\n', encoding='ascii')
    path.chmod(0o444)
    sidecar.chmod(0o444)
    monkeypatch.setattr(V11, 'AUTHORIZATION_PATH', path)
    monkeypatch.setattr(V11, 'AUTHORIZATION_SHA256', digest)


def test_authorization_rejects_drift_reuse_and_launcher_self_reference(tmp_path, monkeypatch):
    cases = (
        (
            lambda value: value['profile'].__setitem__('sha256', '0' * 64),
            'AUTHORIZATION_FIELD_MISMATCH',
        ),
        (
            lambda value: value['gate_receipts']['dual_service'].__setitem__('status', 'PASS'),
            'AUTHORIZATION_GATE_INVALID',
        ),
        (
            lambda value: value['execution']['mounts'].__setitem__(
                'output', 'type=bind,dst=/out,rw'
            ),
            'AUTHORIZATION_MOUNT_INVALID',
        ),
        (
            lambda value: value['sources'].__setitem__(
                'launcher', {'path': str(SCRIPT), 'sha256': '0' * 64}
            ),
            'AUTHORIZATION_SOURCE_SET_INVALID',
        ),
        (
            lambda value: value['environment_precondition']['windows'][0].__setitem__(
                'cpu_busy_percent', 99.0
            ),
            'AUTHORIZATION_ENVIRONMENT_BINDING_INVALID',
        ),
        (lambda value: value.__setitem__('attempt_index', 2), 'AUTHORIZATION_FIELD_MISMATCH'),
    )
    for index, (mutate, kind) in enumerate(cases):
        case = tmp_path / f'case-{index}'
        case.mkdir()
        _authorization_fixture(case, monkeypatch, mutate)
        with pytest.raises(V11.LaunchError) as error:
            V11.verify_formal_authorization(
                V11.FormalConfig(root=V11.AUTHORIZED_ATTEMPT_ROOT, repo_root=ROOT)
            )
        assert error.value.kind == kind


def test_environment_receipt_tamper_is_rejected(tmp_path, monkeypatch):
    original = V11.ENVIRONMENT_PRECONDITION_WINDOWS[0]
    observed = json.loads(Path(original['path']).read_text(encoding='utf-8'))
    observed['status'] = 'FAIL_CLOSED'
    tampered = tmp_path / 'window_01.receipt.json'
    tampered.write_text(json.dumps(observed, sort_keys=True) + '\n', encoding='utf-8')
    tampered_digest = V11.file_sha256(tampered)
    expected = dict(original)
    expected['path'] = str(tampered)
    expected['sha256'] = tampered_digest
    windows = (expected,) + V11.ENVIRONMENT_PRECONDITION_WINDOWS[1:]
    _authorization_fixture(
        tmp_path,
        monkeypatch,
        lambda value: value['environment_precondition']['windows'].__setitem__(0, expected),
    )
    monkeypatch.setattr(V11, 'ENVIRONMENT_PRECONDITION_WINDOWS', windows)
    with pytest.raises(V11.LaunchError) as error:
        V11.verify_formal_authorization(
            V11.FormalConfig(root=V11.AUTHORIZED_ATTEMPT_ROOT, repo_root=ROOT)
        )
    assert error.value.kind == 'AUTHORIZATION_ENVIRONMENT_FIELD_MISMATCH'


def test_reservation_owns_fresh_root_and_rejects_preexisting_root(tmp_path):
    fresh = tmp_path / 'fresh-attempt'
    config = V11.FormalConfig(root=fresh, repo_root=ROOT)
    marker = V11.reserve_attempt_root(config)
    assert marker['status'] == 'RESERVED_EMPTY_ROOT'
    assert fresh.is_dir()
    with pytest.raises(V11.LaunchError) as error:
        V11.reserve_attempt_root(config)
    assert error.value.kind == 'ROOT_ALREADY_EXISTS'


def test_fresh_reservation_precedes_injected_authorization_and_bag_probe(tmp_path):
    events: list[str] = []
    config = _config(tmp_path)

    def identity(_config):
        events.append('authorization')
        assert config.root.is_dir()
        return _identity()

    def failed_quiescence(_config, path):
        events.append('quiescence')
        _quiescence_receipt(path, status='FAIL_CLOSED', allowed=False)
        return 1

    assert (
        V11.run_formal(
            config,
            identity_validator=identity,
            quiescence_runner=failed_quiescence,
        )
        == 10
    )
    assert events == ['authorization', 'quiescence']


def test_identity_order_is_receipts_authorization_then_probes(tmp_path, monkeypatch):
    events: list[str] = []
    monkeypatch.setattr(
        V11,
        'verify_evidence_receipts',
        lambda: events.append('receipts') or {},
    )

    def authorization(_config):
        events.append('authorization')
        return {'status': 'INJECTED_AUTH'}

    def image_probe(_config):
        events.append('image')
        return {'tag': V11.IMAGE_TAG, 'id': V11.IMAGE_ID}

    def bag_probe(_config):
        events.append('bag')
        return {
            'path': str(V11.BAG_PATH),
            'bytes': V11.BAG_BYTES,
            'sha256': V11.BAG_SHA256,
            'expected_messages': V11.EXPECTED_MESSAGES,
            'expected_topic_counts': dict(V11.EXPECTED_TOPIC_COUNTS),
            'required_evaluation_end_timestamp_seconds':
                V11.REQUIRED_EVALUATION_END_TIMESTAMP_SECONDS,
            'opened': True,
        }

    identity = V11.validate_preflight_identity(
        _config(tmp_path),
        image_probe=image_probe,
        bag_probe=bag_probe,
        authorization_validator=authorization,
    )
    assert events == ['receipts', 'authorization', 'image', 'bag']
    assert identity['image']['id'] == V11.IMAGE_ID
    assert identity['bag']['probe']['opened'] is True


def test_fixed_v11_argv_is_digest_bound_and_has_exact_four_mounts(tmp_path):
    argv = V11.build_safe_docker_argv(_config(tmp_path), tmp_path / 'out')
    assert argv[:2] == ['docker', 'run']
    assert '--pull=never' in argv
    assert '--network' in argv
    assert argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv
    assert '--init' in argv
    assert '--rm' not in argv
    assert argv[-2:] == [V11.WRAPPER_CONTAINER_PATH, V11.IMAGE_ID]
    assert V11.IMAGE_TAG not in argv
    assert V11.IMAGE_ID in argv
    assert argv.count('--mount') == 4
    mounts = [argv[index + 1] for index, value in enumerate(argv) if value == '--mount']
    assert any(mount.endswith(',dst=/input/ntu_viral.bag,readonly') for mount in mounts)
    assert any(mount.endswith(',dst=/out,readonly=false') for mount in mounts)
    assert any(mount.endswith(f',dst={V11.WRAPPER_CONTAINER_PATH},readonly') for mount in mounts)
    assert any(mount.endswith(f',dst={V11.FEEDER_CONTAINER_PATH},readonly') for mount in mounts)
    assert not any(mount.endswith(',rw') for mount in mounts)
    rendered = ' '.join(argv).lower()
    for forbidden in (
        '/bin/sh',
        '/bin/bash',
        'rosbag',
        'ground_truth',
        'scorer',
        'map_save',
        '--privileged',
        '--cap-add',
        '--device',
    ):
        assert forbidden not in rendered
    env = {
        argv[index + 1].split('=', 1)[0]: argv[index + 1].split('=', 1)[1]
        for index, value in enumerate(argv)
        if value == '--env'
    }
    assert env['M6A10_PHASE_CONTRACT_VERSION'] == V11.PHASE_CONTRACT_VERSION
    assert env['M6A10_PROFILE_SHA256'] == V11.PROFILE_SHA256
    assert env['M6A10_FAST_FEEDER_SHA256'] == V11.FEEDER_SHA256
    assert env['M6A10_FEEDER_TIMEOUT_SECONDS'] == '60'
    assert env['M6A10_TERMINAL_DRAIN_TIMEOUT_SECONDS'] == '120'
    assert env['M6A10_SENSOR_DURATION_SECONDS'] == '579.278127298'
    assert env['M6A10_CONSUMER_EVIDENCE'] != (env['M6A10_TERMINAL_SUPPORT_CONTEXT_EVIDENCE'])


def test_argv_rejects_existing_output_and_wrong_bag(tmp_path):
    output = tmp_path / 'out'
    output.mkdir()
    with pytest.raises(V11.LaunchError) as error:
        V11.build_safe_docker_argv(_config(tmp_path), output)
    assert error.value.kind == 'OUTPUT_OVERWRITE'
    with pytest.raises(V11.LaunchError) as error:
        V11.build_safe_docker_argv(
            V11.FormalConfig(
                root=tmp_path / 'other',
                repo_root=ROOT,
                bag_path=tmp_path / 'wrong.bag',
            ),
            tmp_path / 'new-out',
        )
    assert error.value.kind == 'BAG_IDENTITY_MISMATCH'


def test_quiescence_fail_closed_never_starts_runner(tmp_path):
    config = _config(tmp_path)
    calls: list[Any] = []

    def failed_quiescence(_config, path):
        _quiescence_receipt(path, status='FAIL_CLOSED', allowed=False)
        return 1

    def should_not_start(*_args):
        calls.append(True)
        pytest.fail('runner must not start after quiescence failure')

    assert (
        V11.run_formal(
            config,
            identity_validator=lambda _config: _identity(),
            quiescence_runner=failed_quiescence,
            runner_hook=should_not_start,
        )
        == 10
    )
    assert calls == []
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'PREFLIGHT_FAIL_CLOSED'
    assert closure['runner_start_attempted'] is False


def test_injected_runner_starts_once_without_retry(tmp_path):
    config = _config(tmp_path)
    calls: list[tuple[str, ...]] = []

    def passed_quiescence(_config, path):
        _quiescence_receipt(path, status='PASS', allowed=True)
        return 0

    def injected_runner(argv, _config):
        calls.append(tuple(argv))
        return 17

    assert (
        V11.run_formal(
            config,
            identity_validator=lambda _config: _identity(),
            quiescence_runner=passed_quiescence,
            runner_hook=injected_runner,
        )
        == 17
    )
    assert len(calls) == 1
    assert calls[0][-2:] == (V11.WRAPPER_CONTAINER_PATH, V11.IMAGE_ID)
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'INJECTED_RUNNER_HOOK_RETURNED'
    assert closure['retry_count'] == 0


class _FakeProcess:
    def __init__(self, *, returncode: int, running: bool, events: list[str]):
        self.pid = 4242
        self.returncode = returncode
        self.running = running
        self.events = events

    def poll(self):
        self.events.append('poll')
        return None if self.running else self.returncode

    def wait(self, *args, **kwargs):
        self.events.append('wait')
        return self.returncode


def _lifecycle_hooks(
    *,
    returncode: int,
    running: bool,
    events: list[str],
):
    process = _FakeProcess(returncode=returncode, running=running, events=events)
    removed = {'value': False}

    def factory(_argv, **_kwargs):
        events.append('popen')
        return process

    def inspect(_name):
        events.append('inspect')
        if removed['value']:
            return {'status': 'not_found'}
        return {
            'status': 'present',
            'running': process.running,
            'state': 'running' if process.running else 'exited',
            'exit_code': process.returncode,
            'oom_killed': False,
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
        removed['value'] = True
        return {'status': 'removed'}

    return process, factory, inspect, stats, top, stop, remove


def test_process_failure_is_diagnostics_first_and_stopped_only_cleanup(tmp_path):
    config = _config(tmp_path)
    events: list[str] = []
    _, factory, inspect, stats, top, stop, remove = _lifecycle_hooks(
        returncode=7, running=False, events=events
    )

    def passed_quiescence(_config, path):
        _quiescence_receipt(path, status='PASS', allowed=True)
        return 0

    result = V11.run_formal(
        config,
        identity_validator=lambda _config: _identity(),
        quiescence_runner=passed_quiescence,
        process_factory=factory,
        container_inspect=inspect,
        container_stats=stats,
        container_top=top,
        container_stop=stop,
        container_remove=remove,
    )
    assert result == 31
    assert events.count('popen') == 1
    assert 'stop' not in events
    assert events.index('inspect') < events.index('remove')
    closure = json.loads((config.root / 'closure_receipt.json').read_text())
    assert closure['failure_kind'] == 'CONTAINER_PROCESS_FAILURE'
    assert closure['process']['started'] is True
    assert closure['cleanup']['status'] == 'removed'


def test_watchdog_captures_diagnostics_before_stop():
    events: list[str] = []
    process = _FakeProcess(returncode=0, running=True, events=events)
    state = {'now': 0.0}

    def clock():
        state['now'] += 1.1
        return state['now']

    def inspect(_name):
        events.append('inspect')
        return {
            'status': 'present',
            'running': process.running,
            'state': 'running',
            'exit_code': None,
            'oom_killed': False,
        }

    def stats(_name):
        events.append('stats')
        return {'status': 'present', 'memory_usage_bytes': 8192}

    def top(_name):
        events.append('top')
        return {'status': 'present'}

    def stop(_name):
        events.append('stop')
        process.running = False
        return {'status': 'requested'}

    code, lifecycle = V11._supervise_container(
        process,
        container_name='v11-test',
        watchdog_seconds=1.0,
        stats_interval_seconds=1.0,
        inspect=inspect,
        stats=stats,
        top=top,
        stop=stop,
        clock=clock,
        sleep=lambda _seconds: None,
    )
    assert code == 0
    assert lifecycle['status'] == 'watchdog_timeout'
    assert lifecycle['watchdog_triggered'] is True
    assert events.index('top') < events.index('stop')
    assert lifecycle['diagnostics_before_stop']['stats']['memory_usage_bytes'] == 8192


def test_stopped_only_cleanup_refuses_running_container():
    events: list[str] = []

    def inspect(_name):
        events.append('inspect')
        return {'status': 'present', 'running': True}

    def remove(_name):
        events.append('remove')
        return {'status': 'removed'}

    result = V11._remove_verified_exited_container('v11-test', inspect, remove)
    assert result['status'] == 'not_removed_running_or_unverified'
    assert events == ['inspect']
