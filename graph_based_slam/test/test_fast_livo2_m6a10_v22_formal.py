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


"""v22 candidate lineage, authorization ordering, and injected lifecycle tests."""

from __future__ import annotations

import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import authorize_fast_livo2_m6a10_v22_formal as AUTH  # noqa: E402,I100
import run_fast_livo2_m6a10_v22_formal as RUN  # noqa: E402,I100


def _argv():
    return [
        'docker',
        'run',
        '--network',
        'none',
        '--read-only',
        '--mount',
        'type=bind,src=/sealed/v22_runtime.sh,dst=/runner/v22_runtime.sh,readonly',
        '--mount',
        'type=bind,src=/fresh/out,dst=/out,readonly=false',
        AUTH.V17_IMAGE_ID,
        '/runner/v22_runtime.sh',
    ]


class _Process:
    def __init__(self, returncode=0):
        self.returncode = returncode
        self.wait_count = 0

    def wait(self):
        self.wait_count += 1
        return self.returncode


def test_real_immutable_v17_v21_v22_lineage_is_accepted():
    value = AUTH.verify_lineage()
    assert value['v22_success']['status'] == 'PASS'
    assert value['v22_failure']['status'] == 'FAIL_CLOSED'
    assert value['v22_success']['execution']['start_count'] == 1
    assert value['v22_failure']['execution']['input_mount_count'] == 0


def test_tampered_gate_hash_is_rejected_without_mutating_receipts(monkeypatch):
    monkeypatch.setattr(AUTH, 'V22_SUCCESS_RECEIPT_SHA256', '0' * 64)
    with pytest.raises(AUTH.AuthorizationError, match='SHA drift'):
        AUTH.verify_lineage()


def test_default_main_fails_before_bag_or_root_creation(tmp_path, capsys):
    root = tmp_path / 'formal-attempt'
    rc = RUN.main(['--root', str(root), '--bag', '/must-not-be-opened.bag'])
    output = json.loads(capsys.readouterr().out)
    assert rc == 11
    assert output['failure_kind'] == 'AUTHORIZATION_NOT_INSTALLED'
    assert not root.exists()


def test_injected_success_has_one_fake_popen_and_immutable_closure(tmp_path):
    root = tmp_path / 'success'
    calls = []
    process = _Process(0)

    def fake_popen(argv, *, cwd, shell):
        calls.append((list(argv), cwd, shell))
        return process

    result = RUN.run_formal(
        root=root,
        runtime={
            'injected': True,
            'authorization_validator': lambda: True,
            'argv': _argv(),
            'popen': fake_popen,
            'artifact_validator': lambda _root: {
                'status': 'PASS',
                'feeder_exit_status': 0,
                'timing': {'status': 'PASS', 'online_compute_rtf': 0.01},
            },
        },
    )
    assert result['status'] == 'PASS'
    assert result['execution']['popen_count'] == 1
    assert len(calls) == 1 and calls[0][2] is False
    assert process.wait_count == 1
    closure = root / 'closure_receipt.json'
    assert closure.stat().st_mode & 0o777 == 0o444
    assert (closure.with_name(closure.name + '.sha256')).stat().st_mode & 0o777 == 0o444
    assert json.loads(closure.read_text())['safety']['input_opened'] is False


def test_main_injected_path_reaches_same_single_popen_closure(tmp_path, capsys):
    root = tmp_path / 'main-success'
    process = _Process(0)
    calls = []

    def fake_popen(argv, *, cwd, shell):
        calls.append((list(argv), cwd, shell))
        return process

    rc = RUN.main(
        ['--root', str(root), '--bag', '/never-opened.bag'],
        runtime={
            'injected': True,
            'authorization_validator': lambda: True,
            'argv': _argv(),
            'popen': fake_popen,
            'artifact_validator': lambda _root: {'status': 'PASS'},
        },
    )
    output = json.loads(capsys.readouterr().out)
    assert rc == 0 and output['status'] == 'PASS'
    assert len(calls) == 1 and calls[0][2] is False and process.wait_count == 1
    assert (root / 'closure_receipt.json').stat().st_mode & 0o777 == 0o444


def test_injected_nonzero_persists_fail_closed_without_retry(tmp_path):
    root = tmp_path / 'failure'
    process = _Process(17)
    calls = []

    def fake_popen(argv, *, cwd, shell):
        calls.append(list(argv))
        return process

    result = RUN.run_formal(
        root=root,
        runtime={
            'injected': True,
            'authorization_validator': lambda: True,
            'argv': _argv(),
            'popen': fake_popen,
            'artifact_validator': lambda _root: {
                'status': 'FAIL_CLOSED',
                'reason': 'feeder nonzero',
            },
        },
    )
    assert result['status'] == 'FAIL_CLOSED'
    assert result['execution']['popen_count'] == 1
    assert len(calls) == 1 and process.wait_count == 1
    assert result['failure_kind'] == 'INJECTED_RUNTIME_FAILURE'


def test_injected_argv_rejects_input_or_second_rw_mount(tmp_path):
    root = tmp_path / 'bad'
    bad = _argv() + ['--mount', 'type=bind,src=/bag,dst=/input/bag,readonly']
    result = RUN.run_formal(
        root=root,
        runtime={
            'injected': True,
            'authorization_validator': lambda: True,
            'argv': bad,
            'popen': lambda *_args, **_kwargs: _Process(0),
        },
    )
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'ARGV_MOUNTS'
    assert root.exists()
    closure = root / 'closure_receipt.json'
    assert closure.is_file()
    assert json.loads(closure.read_text())['status'] == 'FAIL_CLOSED'


def test_formal_source_has_no_real_subprocess_or_bag_probe():
    source = (ROOT / 'scripts/run_fast_livo2_m6a10_v22_formal.py').read_text(encoding='utf-8')
    assert 'import subprocess' not in source
    assert 'shell=True' not in source
    assert 'AUTHORIZATION_NOT_INSTALLED' in source
