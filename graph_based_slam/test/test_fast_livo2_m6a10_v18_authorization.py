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


"""v18 shared authorization verifier and injected lifecycle tests."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


VERIFY = _load(
    'verify_fast_livo2_m6a10_v18_authorization',
    ROOT / 'scripts/verify_fast_livo2_m6a10_v18_authorization.py',
)
AUTHZ = _load(
    'm6a10_v18_authorizer_test', ROOT / 'scripts/authorize_fast_livo2_m6a10_v18_formal.py'
)
LAUNCH = _load('m6a10_v18_launcher_test', ROOT / 'scripts/run_fast_livo2_m6a10_v18_formal.py')


REAL_RECEIPT = VERIFY.AUTHORIZATION_RECEIPT_PATH
REAL_ROOT = Path(
    '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/'
    'fast_livo2_v2c_v17_formal_replay_20260823T230500Z_agentv17formal'
)


def _real_document():
    return json.loads(REAL_RECEIPT.read_text(encoding='utf-8'))


def _fixture(tmp_path: Path, *, mutate=None):
    tmp_path.mkdir(parents=True, exist_ok=True)
    value = _real_document()
    value['attempt_root'] = str(tmp_path / 'attempt')
    for window in value['windows']:
        source = Path(window['path'])
        destination = tmp_path / source.name
        payload = source.read_bytes()
        destination.write_bytes(payload)
        destination.chmod(0o444)
        window['path'] = str(destination)
        window['sha256'] = hashlib.sha256(payload).hexdigest()
    if mutate is not None:
        mutate(value)
    path = tmp_path / 'authorization.receipt.json'
    payload = (json.dumps(value, indent=2, sort_keys=True) + '\n').encode('utf-8')
    path.write_bytes(payload)
    path.chmod(0o444)
    digest = hashlib.sha256(payload).hexdigest()
    sidecar = path.with_name(path.name + '.sha256')
    side_payload = (digest + '  ' + path.name + '\n').encode('ascii')
    sidecar.write_bytes(side_payload)
    sidecar.chmod(0o444)
    return path, hashlib.sha256(side_payload).hexdigest(), digest, Path(value['attempt_root'])


def _verify_fixture(path: Path, sidecar_sha: str, digest: str, attempt: Path):
    return VERIFY.verify_receipt_document(
        path,
        digest,
        attempt,
        expected_receipt_path=path,
        expected_sidecar_sha256=sidecar_sha,
        require_fresh_root=True,
    )


def test_real_sealed_v17_receipt_is_accepted_read_only():
    value = VERIFY.verify_receipt_document(
        REAL_RECEIPT,
        VERIFY.AUTHORIZATION_RECEIPT_SHA256,
        REAL_ROOT,
        expected_receipt_path=REAL_RECEIPT,
        expected_sidecar_sha256=VERIFY.AUTHORIZATION_SIDECAR_SHA256,
        require_fresh_root=False,
    )
    assert value['status'] == 'AUTHORIZED'
    assert value['attempt_root'] == str(REAL_ROOT)
    assert value['lineage']['candidate']['launcher_sha256'] == VERIFY.V17_LAUNCHER_SHA256
    assert value['lineage']['image']['id'] == VERIFY.IMAGE_ID


def test_shared_verifier_is_used_by_authorizer_adapter():
    assert AUTHZ.verify_authorization is VERIFY.verify_authorization
    source = (ROOT / 'scripts/run_fast_livo2_m6a10_v18_formal.py').read_text(encoding='utf-8')
    assert 'authorization.verify_authorization' in source
    assert 'def _verify_authorization' not in source


def test_tampered_authorization_fields_root_and_window_are_rejected(tmp_path):
    def bad_status(value):
        value['status'] = 'FAIL_CLOSED'

    path, side_sha, digest, attempt = _fixture(tmp_path / 'status', mutate=bad_status)
    with pytest.raises(VERIFY.AuthorizationError, match='not executable'):
        _verify_fixture(path, side_sha, digest, attempt)

    def bad_root(value):
        value['attempt_root'] = str(tmp_path / 'different-root')

    path, side_sha, digest, attempt = _fixture(tmp_path / 'root', mutate=bad_root)
    attempt = tmp_path / 'root' / 'attempt'
    with pytest.raises(VERIFY.AuthorizationError, match='exact formal attempt root mismatch'):
        _verify_fixture(path, side_sha, digest, attempt)

    def bad_window(value):
        value['windows'][1]['status'] = 'FAIL_CLOSED'

    path, side_sha, digest, attempt = _fixture(tmp_path / 'window', mutate=bad_window)
    with pytest.raises(VERIFY.AuthorizationError, match='window 2 is not PASS'):
        _verify_fixture(path, side_sha, digest, attempt)


def test_tampered_hash_sidecar_and_mode_are_rejected(tmp_path):
    path, side_sha, digest, attempt = _fixture(tmp_path / 'hash')
    with pytest.raises(VERIFY.AuthorizationError, match='SHA drift'):
        VERIFY.verify_receipt_document(
            path,
            VERIFY.AUTHORIZATION_RECEIPT_SHA256,
            attempt,
            expected_receipt_path=path,
            require_fresh_root=True,
        )

    path, side_sha, digest, attempt = _fixture(tmp_path / 'mode')
    path.chmod(0o644)
    with pytest.raises(VERIFY.AuthorizationError, match='mode 0444'):
        _verify_fixture(path, side_sha, digest, attempt)

    path, side_sha, digest, attempt = _fixture(tmp_path / 'sidecar')
    sidecar = path.with_name(path.name + '.sha256')
    sidecar.chmod(0o644)
    sidecar.write_text('0' * 64 + '  authorization.receipt.json\n', encoding='ascii')
    sidecar.chmod(0o444)
    sidecar.chmod(0o444)
    with pytest.raises(VERIFY.AuthorizationError, match='sidecar SHA drift|sidecar content drift'):
        _verify_fixture(path, side_sha, digest, attempt)


def test_profile_is_unauthorized_and_pins_v17_preflight_failure():
    value = LAUNCH.verify_profile()
    assert value['status'] == 'V18_FORMAL_CANDIDATE_UNAUTHORIZED'
    assert LAUNCH.PROFILE_SHA256 == hashlib.sha256(LAUNCH.PROFILE_PATH.read_bytes()).hexdigest()
    assert (
        VERIFY.V17_PREFLIGHT_CLOSURE_SHA256
        == hashlib.sha256(VERIFY.V17_PREFLIGHT_CLOSURE_PATH.read_bytes()).hexdigest()
    )


class _Process:
    pid = 18018

    def __init__(self, events):
        self.events = events

    def wait(self):
        self.events.append('wait')
        return 0


class _Monitor:
    def __init__(self, events):
        self.events = events

    def start(self):
        self.events.append('monitor_start')

    def allow_owned_pid(self, pid):
        self.events.append(('allow_owned_pid', pid))

    def stop(self):
        self.events.append('monitor_stop')

    def finalize(self, _path):
        self.events.append('monitor_finalize')
        return {
            'status': 'PASS',
            'contaminated': False,
            'invalid': False,
            'coverage': {'coverage_gap': False},
        }


def test_v18_main_injected_real_format_receipt_single_popen_to_closure(tmp_path, capsys):
    auth_path, side_sha, digest, attempt = _fixture(tmp_path / 'run')
    events = []

    def verifier(path, root, expected, **kwargs):
        assert path == auth_path
        assert root == attempt
        assert expected == digest
        return VERIFY.verify_receipt_document(
            path,
            expected,
            root,
            expected_receipt_path=path,
            expected_sidecar_sha256=side_sha,
            require_fresh_root=True,
            repo_root=kwargs.get('repo_root', ROOT),
        )

    def process(argv, cwd):
        events.append(('popen', list(argv), cwd))
        return _Process(events)

    def compose(raw, _config):
        events.append('compose')
        assert raw['returncode'] == 0
        return {'status': 'PASS', 'contract_id': LAUNCH.CONTRACT_VERSION}

    code = LAUNCH.main(
        [
            '--root',
            str(attempt),
            '--authorization',
            str(auth_path),
            '--authorization-sha256',
            digest,
        ],
        runtime={
            'receipt_verifier': verifier,
            'process_factory': process,
            'monitor_factory': lambda root: _Monitor(events),
            'composer': compose,
            'now': '2026-08-24T00:00:00+00:00',
        },
    )
    assert code == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed['status'] == 'PASS'
    assert (
        len([event for event in events if isinstance(event, tuple) and event[0] == 'popen']) == 1
    )
    assert events.count('wait') == 1
    assert events.count('monitor_start') == 1
    assert events.count('monitor_finalize') == 1
    assert events.count('compose') == 1
    closure = attempt / 'closure_receipt.json'
    assert closure.stat().st_mode & 0o777 == 0o444
    value = json.loads(closure.read_text(encoding='utf-8'))
    assert value['status'] == 'PASS'
    assert value['execution']['popen_count'] == 1
    assert value['execution']['retry'] is False
    assert value['safety']['input_opened'] is False
    sidecar = closure.with_name(closure.name + '.sha256')
    closure_sha = hashlib.sha256(closure.read_bytes()).hexdigest()
    assert sidecar.read_bytes() == (closure_sha + '  closure_receipt.json\n').encode('ascii')


def test_v18_default_path_is_execution_inert_and_seals_unauthorized(tmp_path):
    root = tmp_path / 'unauthorized'
    result = LAUNCH.run_formal(LAUNCH.CandidateConfig(root=root))
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'V18_AUTHORIZATION_REQUIRED'
    assert result['execution']['popen_count'] == 0
    assert not any(root.glob('out/*'))
