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


"""Dedicated v19 authorizer and real-v17 production seam tests."""

from __future__ import annotations

import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import authorize_fast_livo2_m6a10_v19_formal as AUTH  # noqa: E402,I100
import fast_livo2_m6a10_v19_base_adapter as ADAPTER  # noqa: E402,I100
import run_fast_livo2_m6a10_v19_formal as LAUNCH  # noqa: E402,I100


def _fake_window(path, *, proc_root, now=None):
    index = int(path.name.split('_')[-1].split('.')[0])
    value = {
        'schema_version': 1,
        'contract_version': AUTH.QUIESCENCE_CONTRACT,
        'window_index': index,
        'status': 'PASS',
        'runner_start_allowed': True,
        'observation': {'forbidden_processes': [], 'cpu_busy_percent': 1.0, 'load1_per_cpu': 0.01},
    }
    digest = AUTH._write_json(path, value)
    return {
        'path': str(path),
        'sha256': digest,
        'status': 'PASS',
        'runner_start_allowed': True,
        'forbidden_processes': [],
    }


def test_v17_surface_and_actual_builder_are_bound_without_starting_docker(tmp_path):
    base = ADAPTER.load_base_surface()
    runtime = ADAPTER.load_v17_runtime_surface(base)
    assert runtime.module_sha256 == AUTH.V17_LAUNCHER_SHA256
    root = tmp_path / 'formal'
    out = root / 'out'
    out.mkdir(parents=True)
    config = LAUNCH.CandidateConfig(
        root=root,
        repo_root=ROOT,
        authorization_path=AUTH.AUTHORIZATION_RECEIPT_PATH,
        authorization_sha256='0' * 64,
    )
    argv = list(runtime.build_safe_docker_argv(config, out))
    assert argv[0:3] == ['docker', 'run', '--name']
    assert argv[-1] == AUTH.V17_IMAGE_ID
    assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv and '--pull=never' in argv
    assert '--rm' not in argv and 'rw' not in argv
    mounts = [argv[i + 1] for i, item in enumerate(argv[:-1]) if item == '--mount']
    destinations = [
        field[4:] for mount in mounts for field in mount.split(',') if field.startswith('dst=')
    ]
    assert len(destinations) == 4 and len(set(destinations)) == 4
    assert '/out' in destinations


def test_authorizer_rejects_wrong_fixed_roots_without_creating_them(tmp_path):
    with pytest.raises(AUTH.AuthorizationError, match='fixed exact roots'):
        AUTH.authorize(
            authorization_root=tmp_path / 'auth',
            attempt_root=tmp_path / 'attempt',
            window_runner=_fake_window,
        )
    assert not (tmp_path / 'auth').exists()
    assert not (tmp_path / 'attempt').exists()


def test_real_window_filename_index_is_numeric_not_json_suffix(tmp_path, monkeypatch):
    monkeypatch.setattr(AUTH.quiescence, 'ancestor_pids', lambda *_args, **_kwargs: set())
    monkeypatch.setattr(
        AUTH.quiescence,
        'collect_observation',
        lambda **_kwargs: {
            'forbidden_processes': [],
            'cpu_busy_percent': 1.0,
            'load1_per_cpu': 0.01,
        },
    )
    monkeypatch.setattr(
        AUTH.quiescence,
        'build_receipt',
        lambda observation, now=None: {
            'schema_version': 1,
            'contract_version': AUTH.QUIESCENCE_CONTRACT,
            'status': 'PASS',
            'runner_start_allowed': True,
            'observation': observation,
        },
    )
    path = tmp_path / 'quiescence_window_01.receipt.json'
    result = AUTH._run_window(path, now='test')
    assert result['status'] == 'PASS'
    assert json.loads(path.read_text(encoding='utf-8'))['window_index'] == 1


def test_authorizer_seals_exact_three_windows_and_shared_verifier_accepts(tmp_path, monkeypatch):
    auth_root = tmp_path / 'auth'
    attempt_root = tmp_path / 'attempt'
    monkeypatch.setattr(AUTH, 'AUTHORIZATION_ROOT', auth_root)
    monkeypatch.setattr(
        AUTH, 'AUTHORIZATION_RECEIPT_PATH', auth_root / 'formal_authorization.receipt.json'
    )
    monkeypatch.setattr(AUTH, 'ATTEMPT_ROOT', attempt_root)
    value = AUTH.authorize(
        authorization_root=auth_root,
        attempt_root=attempt_root,
        repo_root=ROOT,
        window_runner=_fake_window,
        now='test',
    )
    assert value['status'] == 'AUTHORIZED'
    assert value['formal_replay_started'] is False
    assert len(value['windows']) == 3
    receipt = auth_root / 'formal_authorization.receipt.json'
    assert receipt.stat().st_mode & 0o777 == 0o444
    verified = AUTH.verify_authorization(
        receipt, attempt_root, value['receipt_sha256'], repo_root=ROOT
    )
    assert verified['authorized'] is True
    with pytest.raises(AUTH.AuthorizationError, match='exact path/root mismatch'):
        AUTH.verify_authorization(
            receipt, tmp_path / 'other', value['receipt_sha256'], repo_root=ROOT
        )
    first = auth_root / 'quiescence_window_01.receipt.json'
    tampered = json.loads(first.read_text(encoding='utf-8'))
    tampered['runner_start_allowed'] = False
    first.chmod(0o644)
    first.write_text(json.dumps(tampered) + '\n', encoding='utf-8')
    with pytest.raises(AUTH.AuthorizationError):
        AUTH.verify_authorization(receipt, attempt_root, value['receipt_sha256'], repo_root=ROOT)


class _Process:
    pid = 19019

    def wait(self):
        return 0


class _Monitor:
    def start(self):
        return None

    def allow_owned_pid(self, pid):
        assert pid == 19019

    def stop(self):
        return None

    def finalize(self, _path):
        return {
            'status': 'PASS',
            'contaminated': False,
            'invalid': False,
            'coverage': {'coverage_gap': False},
        }


def test_actual_v17_runtime_path_calls_real_builder_once_and_seals_closure(tmp_path, monkeypatch):
    base = ADAPTER.load_base_surface()
    real = ADAPTER.load_v17_runtime_surface(base)
    calls = []

    def builder(config, output):
        calls.append('builder')
        return real.build_safe_docker_argv(config, output)

    runtime = SimpleNamespace(
        module_sha256=AUTH.V17_LAUNCHER_SHA256,
        profile_path=AUTH.V17_PROFILE_PATH,
        profile_sha256=AUTH.V17_PROFILE_SHA256,
        as_identity=lambda: {
            'module': 'run_fast_livo2_m6a10_v17_formal',
            'sha256': AUTH.V17_LAUNCHER_SHA256,
        },
        verify_candidate_profile=lambda *_args: {'status': 'PASS'},
        build_safe_docker_argv=builder,
        production_identity_probe=lambda _config: {'image_id': AUTH.V17_IMAGE_ID, 'opened': False},
        production_bag_probe=lambda _config: {'path': LAUNCH.INPUT_PATH, 'opened': False},
        production_capture=lambda _config, _root: {
            'documents': {'callback': {'status': 'pass'}, 'terminal': {'status': 'pass'}}
        },
        production_compose=lambda _raw, _config: {
            'status': 'PASS',
            'contract_id': LAUNCH.CLOSURE_CONTRACT,
        },
        scoped_runtime=lambda: __import__('contextlib').nullcontext(),
        default_monitor=lambda _root: _Monitor(),
        default_popen=lambda _argv, _cwd: _Process(),
    )
    monkeypatch.setattr(ADAPTER, 'load_v17_runtime_surface', lambda _surface: runtime)
    root = tmp_path / 'fresh'
    root.mkdir()
    result = LAUNCH._run_actual_v17(
        LAUNCH.CandidateConfig(root=root),
        base,
        {'status': 'PASS'},
        {'authorized': True, 'formal_execution': True},
        'test',
    )
    assert result['status'] == 'PASS'
    assert calls == ['builder']
    assert result['execution']['popen_count'] == 1
    assert result['execution']['shell'] is False
    assert result['monitor']['status'] == 'PASS'
    assert (root / 'closure_receipt.json').stat().st_mode & 0o777 == 0o444
