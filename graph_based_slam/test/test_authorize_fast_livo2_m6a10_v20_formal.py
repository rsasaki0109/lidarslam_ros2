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


"""v20 strict filename parser and dry authorization sealing gates."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import authorize_fast_livo2_m6a10_v20_formal as AUTH  # noqa: E402,I100


def _install_fake_quiescence(monkeypatch):
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


def test_window_parser_requires_full_exact_basename(tmp_path, monkeypatch):
    _install_fake_quiescence(monkeypatch)
    good = tmp_path / 'quiescence_window_02.receipt.json'
    result = AUTH._run_window(good, now='test')
    assert result['window_index'] == 2
    assert json.loads(good.read_text(encoding='utf-8'))['window_index'] == 2
    for name in (
        'quiescence_window_2.receipt.json',
        'prefix_quiescence_window_02.receipt.json',
        'quiescence_window_02.receipt.json.bak',
        'quiescence_window_04.receipt.json',
        'quiescence_window_02.receipt.json.link',
    ):
        with pytest.raises(AUTH.AuthorizationError, match='window (filename|index)'):
            AUTH._run_window(tmp_path / name, now='test')


def test_actual_module_dry_authorize_writes_three_ordered_receipts_and_sidecar(
    tmp_path, monkeypatch
):
    _install_fake_quiescence(monkeypatch)
    auth_root = tmp_path / 'authorization'
    attempt_root = tmp_path / 'formal'
    monkeypatch.setattr(AUTH, 'AUTHORIZATION_ROOT', auth_root)
    monkeypatch.setattr(
        AUTH, 'AUTHORIZATION_RECEIPT_PATH', auth_root / 'formal_authorization.receipt.json'
    )
    monkeypatch.setattr(AUTH, 'ATTEMPT_ROOT', attempt_root)
    value = AUTH.authorize(
        authorization_root=auth_root,
        attempt_root=attempt_root,
        repo_root=ROOT,
        window_runner=AUTH._run_window,
        now='test',
    )
    assert value['status'] == 'AUTHORIZED'
    receipt = auth_root / 'formal_authorization.receipt.json'
    sidecar = receipt.with_name(receipt.name + '.sha256')
    assert receipt.stat().st_mode & 0o777 == 0o444
    assert sidecar.stat().st_mode & 0o777 == 0o444
    verified = AUTH.verify_authorization(
        receipt, attempt_root, value['receipt_sha256'], repo_root=ROOT
    )
    assert verified['authorized'] is True
    assert [item['window_index'] for item in value['windows']] == [1, 2, 3]
    assert len({item['sha256'] for item in value['windows']}) == 3
    assert len({item['path'] for item in value['windows']}) == 3


def test_v19_failure_lineage_is_bound_and_attempt_root_is_not_reused(tmp_path, monkeypatch):
    assert AUTH.V19_FAILURE_AUTHORIZATION_PATH.is_file()
    # Copy the immutable predecessor receipt into a local read-only fixture;
    # never make the test depend on whether a sealed external attempt root
    # happens to exist after a real fail-closed run.
    fixture = tmp_path / 'v19-failure-fixture' / 'formal_authorization.receipt.json'
    fixture.parent.mkdir()
    fixture.write_bytes(AUTH.V19_FAILURE_AUTHORIZATION_PATH.read_bytes())
    fixture.chmod(0o444)
    assert AUTH.sha256_file(fixture) == AUTH.V19_FAILURE_AUTHORIZATION_SHA256
    assert AUTH.V19_FAILURE_AUTHORIZATION_PATH.parent.name.endswith('agentv19')

    _install_fake_quiescence(monkeypatch)
    auth_root = tmp_path / 'fresh-authorization'
    attempt_root = tmp_path / 'fresh-attempt'
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
    assert not attempt_root.exists()

    # A second call cannot reuse the sealed authorization root.
    with pytest.raises(AUTH.AuthorizationError, match='roots must be absent'):
        AUTH.authorize(
            authorization_root=auth_root,
            attempt_root=attempt_root,
            repo_root=ROOT,
            window_runner=AUTH._run_window,
            now='test',
        )

    # Existing directory and symlink roots are both rejected before any new
    # authorization root is created.
    existing_auth = tmp_path / 'existing-auth'
    existing_attempt = tmp_path / 'existing-attempt'
    existing_auth.mkdir()
    existing_attempt.mkdir()
    monkeypatch.setattr(AUTH, 'AUTHORIZATION_ROOT', existing_auth)
    monkeypatch.setattr(
        AUTH, 'AUTHORIZATION_RECEIPT_PATH', existing_auth / 'formal_authorization.receipt.json'
    )
    monkeypatch.setattr(AUTH, 'ATTEMPT_ROOT', existing_attempt)
    with pytest.raises(AUTH.AuthorizationError, match='roots must be absent'):
        AUTH.authorize(
            authorization_root=existing_auth,
            attempt_root=existing_attempt,
            repo_root=ROOT,
            window_runner=AUTH._run_window,
            now='test',
        )

    symlink_target = tmp_path / 'symlink-target'
    symlink_target.mkdir()
    symlink_attempt = tmp_path / 'symlink-attempt'
    symlink_attempt.symlink_to(symlink_target, target_is_directory=True)
    symlink_auth = tmp_path / 'symlink-auth'
    monkeypatch.setattr(AUTH, 'AUTHORIZATION_ROOT', symlink_auth)
    monkeypatch.setattr(
        AUTH, 'AUTHORIZATION_RECEIPT_PATH', symlink_auth / 'formal_authorization.receipt.json'
    )
    monkeypatch.setattr(AUTH, 'ATTEMPT_ROOT', symlink_attempt)
    with pytest.raises(AUTH.AuthorizationError, match='roots must be absent'):
        AUTH.authorize(
            authorization_root=symlink_auth,
            attempt_root=symlink_attempt,
            repo_root=ROOT,
            window_runner=AUTH._run_window,
            now='test',
        )
