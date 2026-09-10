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


"""Host-only tests for v12 exact-root authorization."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import stat
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/authorize_fast_livo2_m6a10_v12_formal.py'
SPEC = importlib.util.spec_from_file_location('m6a10_v12_authorize', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _window_runner(path: Path, *, proc_root: Path, now=None):
    del proc_root
    value = {
        'schema_version': 1,
        'contract_version': 'm6a10-quiescence-v1',
        'status': 'PASS',
        'runner_start_allowed': True,
        'observation': {
            'forbidden_processes': [],
            'checks': {
                'no_forbidden_processes': True}},
    }
    path.write_text(json.dumps(value, sort_keys=True) + '\n', encoding='utf-8')
    return {
        'path': str(path),
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'status': 'PASS',
        'runner_start_allowed': True,
    }


def _failing_window(path: Path, *, proc_root: Path, now=None):
    del proc_root, now
    value = {'status': 'FAIL_CLOSED', 'runner_start_allowed': False}
    path.write_text(json.dumps(value) + '\n', encoding='utf-8')
    return {
        'path': str(path),
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'status': 'FAIL_CLOSED',
        'runner_start_allowed': False,
    }


def test_exact_root_authorization_requires_three_windows_and_seals_immutable_receipt(
        tmp_path):
    auth_root = tmp_path / 'authorization'
    attempt_root = tmp_path / 'attempt'
    result = MODULE.authorize(
        auth_root, attempt_root, repo_root=ROOT,
        window_runner=_window_runner, now='2026-08-24T00:00:00+00:00',
    )
    assert result['status'] == 'AUTHORIZED'
    assert result['formal_replay_authorized'] is True
    assert result['attempt_root'] == str(attempt_root)
    assert result['quiescence']['window_count'] == 3
    assert len(result['quiescence']['windows']) == 3
    assert all(item['status'] ==
               'PASS' for item in result['quiescence']['windows'])
    receipt = Path(result['receipt_path'])
    sidecar = Path(result['sidecar_path'])
    assert not attempt_root.exists()
    assert stat.S_IMODE(receipt.stat().st_mode) == 0o444
    assert stat.S_IMODE(sidecar.stat().st_mode) == 0o444
    receipt_sha = hashlib.sha256(receipt.read_bytes()).hexdigest()
    assert sidecar.read_text(
        encoding='ascii') == '%s  %s\n' % (receipt_sha,
                                           receipt.name)
    assert result['safety']['ground_truth_content_opened'] is False
    assert result['safety']['scorer_invoked'] is False


def test_any_quiescence_failure_is_sticky_and_never_authorizes_root(tmp_path):
    calls = []

    def runner(path, *, proc_root, now=None):
        calls.append(path.name)
        return _failing_window(path, proc_root=proc_root, now=now)

    result = MODULE.authorize(
        tmp_path / 'authorization', tmp_path / 'attempt',
        repo_root=ROOT, window_runner=runner,
    )
    assert result['status'] == 'FAIL_CLOSED'
    assert result['formal_replay_authorized'] is False
    assert result['failure_kind'] == 'QUIESCENCE_FAIL_CLOSED'
    assert len(calls) == 3


def test_authorization_and_attempt_root_reuse_are_rejected(tmp_path):
    auth = tmp_path / 'auth'
    attempt = tmp_path / 'attempt'
    first = MODULE.authorize(
        auth,
        attempt,
        repo_root=ROOT,
        window_runner=_window_runner)
    assert first['status'] == 'AUTHORIZED'
    with pytest.raises(MODULE.AuthorizationError, match='authorization root'):
        MODULE.authorize(auth, attempt, repo_root=ROOT,
                         window_runner=_window_runner)
    existing_attempt = tmp_path / 'existing-attempt'
    existing_attempt.mkdir()
    with pytest.raises(MODULE.AuthorizationError, match='attempt root'):
        MODULE.authorize(
            tmp_path / 'auth2',
            existing_attempt,
            repo_root=ROOT,
            window_runner=_window_runner)


def test_authorizer_is_read_only_with_no_runtime_surfaces():
    text = SCRIPT.read_text(encoding='utf-8').lower()
    assert 'docker run' not in text
    assert 'rosbag play' not in text
    assert 'ground_truth' in text
    assert 'scorer_invoked' in text
    assert 'subprocess' not in text
    assert '.terminate(' not in text
    assert '.kill(' not in text
