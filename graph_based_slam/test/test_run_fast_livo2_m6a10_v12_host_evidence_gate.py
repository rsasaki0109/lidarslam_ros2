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


"""Host-only tests for the fixed v12 evidence gate runner."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import stat
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_fast_livo2_m6a10_v12_host_evidence_gate.py'
SPEC = importlib.util.spec_from_file_location('m6a10_v12_host_gate', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class FakeCompleted:
    def __init__(self, returncode=0, stdout=b'pytest ok\n', stderr=b''):
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr


def test_attempt6_fixture_is_strictly_verified_read_only():
    result = MODULE.verify_attempt6()
    assert result['status'] == 'PASS'
    assert result['attempt_index'] == 6
    assert result['receipt_sha256'] == MODULE.ATTEMPT6_RECEIPT_SHA256
    assert result['callback']['sha256'] == MODULE.ATTEMPT6_CALLBACK_SHA256
    assert result['terminal']['sha256'] == MODULE.ATTEMPT6_TERMINAL_SHA256
    assert result['service_journal']['sha256'] == MODULE.ATTEMPT6_JOURNAL_SHA256


def test_fixed_pytest_argv_is_shell_free_and_explicit():
    argv = MODULE.build_pytest_argv(ROOT)
    assert argv[1:4] == ['-m', 'pytest', '-q']
    assert argv[4:] == [str(ROOT / item) for item in MODULE.FIXED_TESTS]
    MODULE.validate_pytest_argv(argv, ROOT)
    digest = MODULE.canonical_command_sha256(argv)
    assert len(digest) == 64
    assert all(
        token not in '\n'.join(argv).lower()
        for token in ('docker', 'rosbag', 'ground_truth', 'scorer', 'map_save')
    )


@pytest.mark.parametrize(
    'mutation',
    [
        lambda argv, root: argv[:4] + ['docker build'] + argv[4:],
        lambda argv, root: argv[:4] + [str(root / 'scorer/results.py')] + argv[4:],
        lambda argv, root: argv[:4] + ['pytest -k x; echo bad'] + argv[4:],
    ],
)
def test_forbidden_command_or_path_is_rejected(mutation):
    argv = mutation(MODULE.build_pytest_argv(ROOT), ROOT)
    with pytest.raises(MODULE.HostEvidenceGateError):
        MODULE.validate_pytest_argv(argv, ROOT)


def test_fake_pytest_success_seals_outputs_and_receipt_once(tmp_path):
    calls = []

    def fake_executor(argv, cwd):
        calls.append((list(argv), cwd))
        return FakeCompleted(stdout=b'50 passed\n', stderr=b'')

    root = tmp_path / 'fresh-success'
    result = MODULE.run_gate(
        root,
        repo_root=ROOT,
        executor=fake_executor,
        now='2026-08-24T00:00:00+00:00',
    )
    assert result['status'] == 'PASS'
    assert len(calls) == 1
    assert calls[0][1] == ROOT
    assert result['pytest']['returncode'] == 0
    assert result['pytest']['stdout_sha256'] == hashlib.sha256(b'50 passed\n').hexdigest()
    receipt = root / MODULE.RECEIPT_NAME
    sidecar = root / (MODULE.RECEIPT_NAME + '.sha256')
    assert receipt.is_file() and sidecar.is_file()
    assert json.loads(receipt.read_text())['status'] == 'PASS'
    receipt_sha = hashlib.sha256(receipt.read_bytes()).hexdigest()
    assert sidecar.read_text() == f'{receipt_sha}  {receipt.name}\n'
    for path in (receipt, sidecar, root / 'pytest.stdout', root / 'pytest.stderr'):
        assert stat.S_IMODE(path.stat().st_mode) == 0o444
    safety = json.loads(receipt.read_text())['safety']
    assert all(value is False for value in safety.values())


def test_fake_pytest_failure_is_sealed_without_retry(tmp_path):
    calls = []

    def fake_executor(argv, cwd):
        calls.append(list(argv))
        return FakeCompleted(returncode=5, stdout=b'failed\n', stderr=b'assertion\n')

    result = MODULE.run_gate(tmp_path / 'fresh-failure', repo_root=ROOT, executor=fake_executor)
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'PYTEST_FAILED'
    assert result['pytest']['returncode'] == 5
    assert len(calls) == 1


def test_fresh_root_overwrite_and_symlink_are_rejected(tmp_path):
    existing = tmp_path / 'existing'
    existing.mkdir()
    with pytest.raises(MODULE.HostEvidenceGateError, match='fresh host evidence root'):
        MODULE.run_gate(existing, repo_root=ROOT, executor=lambda *_: FakeCompleted())
    target = tmp_path / 'target'
    target.mkdir()
    link = tmp_path / 'link'
    link.symlink_to(target, target_is_directory=True)
    with pytest.raises(MODULE.HostEvidenceGateError):
        MODULE.reserve_root(link)


def test_attempt6_negative_drift_is_fail_closed(monkeypatch):
    monkeypatch.setattr(MODULE, 'ATTEMPT6_RECEIPT_SHA256', '0' * 64)
    with pytest.raises(MODULE.HostEvidenceGateError, match='receipt SHA'):
        MODULE.verify_attempt6()


def test_source_static_contract_has_no_runtime_docker_or_input_surfaces():
    text = SCRIPT.read_text(encoding='utf-8').lower()
    assert 'docker run' not in text
    assert 'rosbag play' not in text
    assert 'subprocess.run' in text
    assert 'shell=false' in text
    assert 'formal_replay_started' in text
    assert 'scorer_invoked' in text
