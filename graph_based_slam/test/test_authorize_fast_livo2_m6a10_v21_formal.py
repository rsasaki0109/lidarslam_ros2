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


"""v21 freshness, reservation ordering, and immutable failure gates."""

from __future__ import annotations

from contextlib import nullcontext
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import authorize_fast_livo2_m6a10_v21_formal as AUTH  # noqa: E402,I100
import run_fast_livo2_m6a10_v21_formal as RUN  # noqa: E402,I100


def _fake_quiescence(monkeypatch):
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


def test_parent_and_root_freshness_are_fail_closed(tmp_path):
    fresh = tmp_path / 'fresh' / 'attempt'
    fresh.parent.mkdir()
    AUTH._parent_ready(fresh, 'fresh')
    fresh.mkdir()
    with pytest.raises(AUTH.AuthorizationError, match='already exists'):
        AUTH._parent_ready(fresh, 'existing')

    link_root = tmp_path / 'link-root'
    link_root.symlink_to(fresh, target_is_directory=True)
    with pytest.raises(AUTH.AuthorizationError, match='already exists'):
        AUTH._parent_ready(link_root, 'symlink-root')

    file_root = tmp_path / 'file-root'
    file_root.write_text('not a directory', encoding='utf-8')
    with pytest.raises(AUTH.AuthorizationError, match='already exists'):
        AUTH._parent_ready(file_root, 'non-directory-root')

    link_parent = tmp_path / 'link-parent'
    link_parent.symlink_to(fresh.parent, target_is_directory=True)
    with pytest.raises(AUTH.AuthorizationError, match='symlink'):
        AUTH._parent_ready(link_parent / 'attempt', 'symlink-parent')

    no_write = tmp_path / 'no-write'
    no_write.mkdir()
    old_mode = no_write.stat().st_mode & 0o777
    try:
        no_write.chmod(0o555)
        with pytest.raises(AUTH.AuthorizationError, match='writable'):
            AUTH._parent_ready(no_write / 'attempt', 'no-write')
    finally:
        no_write.chmod(old_mode)


def test_actual_v21_authorizer_uses_absent_attempt_and_seals_three_windows(tmp_path, monkeypatch):
    _fake_quiescence(monkeypatch)
    auth_root = tmp_path / 'authorization'
    attempt_root = tmp_path / 'attempt'
    monkeypatch.setattr(AUTH, 'AUTHORIZATION_ROOT', auth_root)
    monkeypatch.setattr(
        AUTH, 'AUTHORIZATION_RECEIPT_PATH', auth_root / 'formal_authorization.receipt.json'
    )
    monkeypatch.setattr(AUTH, 'ATTEMPT_ROOT', attempt_root)
    assert not auth_root.exists() and not attempt_root.exists()
    value = AUTH.authorize(
        authorization_root=auth_root,
        attempt_root=attempt_root,
        repo_root=ROOT,
        window_runner=AUTH._run_window,
        now='test',
    )
    assert value['status'] == 'AUTHORIZED'
    assert value['attempt_root'] == str(attempt_root)
    assert not attempt_root.exists()
    receipt = auth_root / 'formal_authorization.receipt.json'
    sidecar = receipt.with_name(receipt.name + '.sha256')
    assert receipt.stat().st_mode & 0o777 == 0o444
    assert sidecar.stat().st_mode & 0o777 == 0o444
    assert [window['window_index'] for window in value['windows']] == [1, 2, 3]
    verified = AUTH.verify_authorization(
        receipt, attempt_root, value['receipt_sha256'], repo_root=ROOT
    )
    assert verified['status'] == 'AUTHORIZED'


def test_v21_real_v19_lifecycle_reserves_root_before_out_and_single_popen(tmp_path, monkeypatch):
    base = tmp_path / 'v21'
    base.mkdir()
    root = base / 'attempt'
    monkeypatch.setattr(RUN, 'ATTEMPT_ROOT', root)
    events = []

    class Monitor:
        def start(self):
            events.append(('monitor_start', root.exists(), (root / 'out').exists()))

        def allow_owned_pid(self, pid):
            events.append(('allow_owned_pid', pid))

        def stop(self):
            events.append(('monitor_stop',))

        def finalize(self, _path):
            events.append(('monitor_finalize',))
            return {
                'status': 'PASS',
                'contaminated': False,
                'invalid': False,
                'coverage': {'coverage_gap': False},
            }

    class Process:
        pid = 43210

        def wait(self):
            events.append(('wait',))
            return 0

    class Runtime:
        module_sha256 = '769aaa1b2f818bb8f4eeac1b57f45e4734949e4510cb3a4557d47348d847c686'
        profile_path = Path('unused-profile')
        profile_sha256 = 'unused'

        def as_identity(self):
            return {'module_sha256': self.module_sha256}

        def verify_candidate_profile(self, *_args):
            events.append(('profile',))
            return {}

        def production_identity_probe(self, config):
            events.append(('identity', root.exists(), (root / 'out').exists()))
            return {'opened': False}

        def production_bag_probe(self, _config):
            events.append(('bag',))
            return {'opened': False}

        def build_safe_docker_argv(self, config, output):
            events.append(('argv', root.exists(), output.exists()))
            return ['fake-runtime-argv']

        def scoped_runtime(self):
            return nullcontext()

        def default_monitor(self, _root):
            return Monitor()

        def default_popen(self, argv, _cwd):
            events.append(('popen', root.exists(), (root / 'out').exists(), list(argv)))
            return Process()

        def production_capture(self, _config, _root):
            events.append(('capture',))
            return {'status': 'PASS'}

        def production_compose(self, _raw, _config):
            events.append(('compose',))
            return {'status': 'PASS'}

    monkeypatch.setattr(RUN.adapter, 'load_v17_runtime_surface', lambda _surface: Runtime())
    config = RUN.v19.CandidateConfig(
        root=root, repo_root=ROOT, bag_path='unused', profile_path=RUN.PROFILE_PATH
    )
    assert not root.exists()
    result = RUN.run_formal(
        config,
        authorization_validator=lambda _config: {
            'authorized': True,
            'formal_execution': True,
        },
        now='test',
    )
    assert result['status'] == 'PASS'
    names = [event[0] for event in events]
    assert names.index('identity') < names.index('popen')
    identity = next(event for event in events if event[0] == 'identity')
    popen = next(event for event in events if event[0] == 'popen')
    assert identity[1:] == (True, True)
    assert popen[1:] == (True, True, ['fake-runtime-argv'])
    assert result['execution']['popen_count'] == 1
    assert root.exists() and (root / 'out').is_dir()
    receipt = root / 'closure_receipt.json'
    assert receipt.stat().st_mode & 0o777 == 0o444
    assert (receipt.with_name(receipt.name + '.sha256')).stat().st_mode & 0o777 == 0o444


def test_exception_after_reservation_seals_without_start_or_out(tmp_path, monkeypatch):
    root = tmp_path / 'attempt'
    monkeypatch.setattr(RUN, 'ATTEMPT_ROOT', root)
    config = RUN.v19.CandidateConfig(
        root=root, repo_root=ROOT, bag_path='unused', profile_path=RUN.PROFILE_PATH
    )
    result = RUN.run_formal(
        config,
        authorization_validator=lambda _config: {
            'authorized': False,
            'formal_execution': False,
        },
        now='test',
    )
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'AUTHORIZATION_STATUS'
    assert result['execution']['popen_count'] == 0
    assert result['attempt_root_reserved'] is True
    assert root.is_dir() and not (root / 'out').exists()
    assert (root / 'closure_receipt.json').stat().st_mode & 0o777 == 0o444


def test_state_order_and_no_shell_or_docker_in_v21_source():
    source = (ROOT / 'scripts/run_fast_livo2_m6a10_v21_formal.py').read_text(encoding='utf-8')
    assert source.index('def _reserve_exact_root') < source.index('v19._run_actual_v17')
    assert 'os.mkdir(root' in source
    assert 'subprocess' not in source
    assert 'docker run' not in source
    assert 'shell=True' not in source
