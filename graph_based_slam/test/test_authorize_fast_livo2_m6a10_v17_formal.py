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


"""Host-only v17 authorization and immutable lineage tests."""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'm6a10_v17_authorizer_test_module', ROOT / 'scripts/authorize_fast_livo2_m6a10_v17_formal.py'
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _window(path, **_kwargs):
    value = {
        'schema_version': 1,
        'contract_version': 'm6a10-quiescence-v1',
        'status': 'PASS',
        'runner_start_allowed': True,
        'observation': {'forbidden_processes': [], 'cpu_busy_percent': 1.0},
    }
    digest = MODULE._write_json(Path(path), value)
    return {
        'path': str(path),
        'sha256': digest,
        'status': 'PASS',
        'runner_start_allowed': True,
        'forbidden_processes': [],
    }


def test_v17_lineage_and_candidate_source_pass():
    lineage = MODULE._verify_lineage()
    assert lineage['candidate']['sha256'] == MODULE.V17_PROFILE_SHA256
    assert lineage['candidate']['launcher_sha256'] == MODULE.V17_LAUNCHER_SHA256
    assert lineage['image']['id'] == MODULE.IMAGE_ID
    assert set(lineage['receipts']) == {
        'v17_build',
        'v17_no_input_pass',
        'v17_no_input_failure',
        'v16_closure',
        'v16_terminal_raw',
    }
    assert (
        MODULE.V17_LAUNCHER_SHA256
        == hashlib.sha256(MODULE.V17_LAUNCHER_PATH.read_bytes()).hexdigest()
    )


def test_v17_authorize_three_windows_and_verify_exact_fresh_root(tmp_path):
    auth_root = tmp_path / 'authorization'
    attempt_root = tmp_path / 'formal-attempt'
    result = MODULE.authorize(
        auth_root, attempt_root, window_runner=_window, now='2026-08-24T00:00:00+00:00'
    )
    assert result['status'] == 'AUTHORIZED'
    receipt = auth_root / MODULE.RECEIPT_NAME
    sidecar = receipt.with_name(receipt.name + '.sha256')
    assert receipt.stat().st_mode & 0o777 == 0o444
    assert sidecar.stat().st_mode & 0o777 == 0o444
    receipt_sha = hashlib.sha256(receipt.read_bytes()).hexdigest()
    assert receipt_sha == result['receipt_sha256']
    assert sidecar.read_bytes() == (receipt_sha + '  ' + receipt.name + '\n').encode('ascii')
    verified = MODULE.verify_authorization(receipt, attempt_root, receipt_sha)
    assert verified['status'] == 'AUTHORIZED'
    assert verified['attempt_root'] == str(attempt_root)
    assert [window['status'] for window in verified['windows']] == ['PASS', 'PASS', 'PASS']


def test_v17_authorization_rejects_root_reuse_wrong_root_and_sidecar_drift(tmp_path):
    auth_root = tmp_path / 'authorization'
    attempt_root = tmp_path / 'formal-attempt'
    result = MODULE.authorize(auth_root, attempt_root, window_runner=_window)
    receipt = auth_root / MODULE.RECEIPT_NAME
    with pytest.raises(MODULE.AuthorizationError, match='root mismatch'):
        MODULE.verify_authorization(receipt, tmp_path / 'different', result['receipt_sha256'])
    attempt_root.mkdir()
    with pytest.raises(MODULE.AuthorizationError, match='already exists'):
        MODULE.verify_authorization(receipt, attempt_root, result['receipt_sha256'])
    tampered = tmp_path / 'tampered.json'
    tampered.write_bytes(receipt.read_bytes())
    tampered_sidecar = tampered.with_name(tampered.name + '.sha256')
    tampered_sidecar.write_bytes((result['receipt_sha256'] + '  wrong.json\n').encode('ascii'))
    with pytest.raises(MODULE.AuthorizationError):
        MODULE._json_receipt(
            tampered,
            result['receipt_sha256'],
            hashlib.sha256(tampered_sidecar.read_bytes()).hexdigest(),
            'tampered',
        )


def test_v17_authorizer_is_host_only_and_requires_fresh_roots(tmp_path):
    text = (
        (ROOT / 'scripts/authorize_fast_livo2_m6a10_v17_formal.py')
        .read_text(encoding='utf-8')
        .lower()
    )
    assert 'subprocess' not in text
    assert 'docker run' not in text
    assert 'rosbag play' not in text
    existing = tmp_path / 'existing'
    existing.mkdir()
    with pytest.raises(MODULE.AuthorizationError, match='already exists'):
        MODULE.authorize(existing, tmp_path / 'attempt', window_runner=_window)
