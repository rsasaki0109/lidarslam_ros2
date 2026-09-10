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


"""v23 shared-authorization and injected lifecycle gates."""

from __future__ import annotations

import ast
import copy
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import fast_livo2_m6a10_v19_base_adapter as ADAPTER  # noqa: E402,I100
import fast_livo2_m6a10_v23_shared_auth as AUTH  # noqa: E402,I100
import run_fast_livo2_m6a10_v23_formal as RUN  # noqa: E402,I100


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
    pid = 23023

    def __init__(self, events, returncode=0):
        self.events = events
        self.returncode = returncode
        self.wait_count = 0

    def wait(self):
        self.wait_count += 1
        self.events.append('wait')
        return self.returncode


def _real_receipt():
    return json.loads(AUTH.V22_AUTHORIZATION_RECEIPT_PATH.read_text(encoding='utf-8'))


def test_actual_v19_v22_modules_and_sealed_v22_authorization_are_bound():
    identity = AUTH.verify_candidate_identity(launcher_path=RUN.V23_LAUNCHER_PATH)
    assert identity['v19_adapter']['sha256'] == AUTH.V19_ADAPTER_SHA256
    assert identity['v22_launcher']['sha256'] == AUTH.V22_LAUNCHER_SHA256
    assert identity['v22_lineage_status']['v22_success'] == 'PASS'
    value = AUTH.verify_v22_authorization()
    assert value['status'] == 'AUTHORIZED'
    assert value['authorized'] is True
    assert value['attempt_root'] == AUTH.V22_ATTEMPT_ROOT
    assert value['formal_replay_started'] is False
    receipt = AUTH.V22_AUTHORIZATION_RECEIPT_PATH
    assert receipt.stat().st_mode & 0o777 == 0o444
    assert receipt.with_name(receipt.name + '.sha256').stat().st_mode & 0o777 == 0o444


def test_shared_path_uses_v19_receipt_reader_and_launcher_has_no_sentinel_or_duplicate_verifier():
    source = (ROOT / 'scripts/fast_livo2_m6a10_v23_shared_auth.py').read_text(encoding='utf-8')
    launcher_source = RUN.V23_LAUNCHER_PATH.read_text(encoding='utf-8')
    assert 'v19_verifier._read_receipt' in source
    assert 'AUTHORIZATION_NOT_INSTALLED' not in launcher_source
    assert 'def verify_authorization' not in launcher_source
    assert 'shared_auth.verify_v22_authorization' in launcher_source
    ADAPTER.assert_launcher_uses_adapter(RUN.V23_LAUNCHER_PATH)
    tree = ast.parse(launcher_source)
    assert not any(
        isinstance(node, ast.FunctionDef) and node.name == 'verify_authorization'
        for node in ast.walk(tree)
    )


@pytest.mark.parametrize(
    'field, value',
    [
        ('attempt_root', '/tmp/not-authorized'),
        (
            'image',
            {'formal_replay_forbidden': False, 'id': 'sha256:bad', 'tag': AUTH.V17_IMAGE_TAG},
        ),
        (
            'safety',
            {
                'formal_replay_started': False,
                'ground_truth_content_opened': False,
                'input_opened': True,
                'map_saved': False,
                'scorer_invoked': False,
            },
        ),
    ],
)
def test_receipt_root_image_safety_tampering_is_rejected(field, value):
    document = _real_receipt()
    document[field] = value
    with pytest.raises(AUTH.SharedAuthorizationError):
        AUTH._validate_v22_document(document, expected_attempt_root=AUTH.V22_ATTEMPT_ROOT)


def test_receipt_window_tampering_is_rejected_without_touching_sealed_fixture():
    document = _real_receipt()
    document['windows'] = copy.deepcopy(document['windows'])
    document['windows'][0]['status'] = 'FAIL_CLOSED'
    with pytest.raises(AUTH.SharedAuthorizationError):
        AUTH._validate_v22_document(document, expected_attempt_root=AUTH.V22_ATTEMPT_ROOT)


def test_wrong_receipt_path_and_symlink_are_rejected(tmp_path):
    copied = tmp_path / 'formal_authorization.receipt.json'
    copied.write_bytes(AUTH.V22_AUTHORIZATION_RECEIPT_PATH.read_bytes())
    with pytest.raises(AUTH.SharedAuthorizationError, match='path drift'):
        AUTH.verify_v22_authorization(path=copied)
    link = tmp_path / 'link.json'
    link.symlink_to(AUTH.V22_AUTHORIZATION_RECEIPT_PATH)
    with pytest.raises(AUTH.SharedAuthorizationError, match='path drift|symlink'):
        AUTH.verify_v22_authorization(path=link)


def test_receipt_and_sidecar_sha_tampering_is_rejected(monkeypatch):
    with pytest.raises(AUTH.SharedAuthorizationError):
        AUTH.verify_v22_authorization(expected_sha256='0' * 64)
    monkeypatch.setattr(AUTH, 'V22_AUTHORIZATION_SIDECAR_SHA256', '0' * 64)
    with pytest.raises(AUTH.SharedAuthorizationError):
        AUTH.verify_v22_authorization()


def test_profile_mutation_is_rejected():
    profile = RUN.verify_profile()
    document = {
        'schema_version': 1,
        'status': 'V23_FORMAL_CANDIDATE_UNAUTHORIZED',
        'formal_replay_forbidden': True,
        'formal_replay_authorized': False,
        'formal_replay_started': False,
        'replay_count': 0,
        'candidate': dict(profile['candidate']),
        'safety': dict(profile['safety']),
    }
    document['candidate']['image_id'] = 'sha256:tampered'
    with pytest.raises(RUN.CandidateError):
        RUN._validate_profile_document(document)


def test_main_without_injected_runtime_fails_before_root_creation(tmp_path, capsys):
    root = tmp_path / 'formal'
    rc = RUN.main(['--root', str(root)])
    output = json.loads(capsys.readouterr().out)
    assert rc == 11
    assert output['failure_kind'] == 'V23_FORMAL_UNAUTHORIZED'
    assert not root.exists()


def test_fake_authorization_bypass_is_rejected_before_root_creation(tmp_path):
    root = tmp_path / 'bypass'
    with pytest.raises(RUN.CandidateError, match='fake authorization'):
        RUN.run_formal(
            root=root,
            runtime={'injected': True, 'authorization_validator': lambda: True, 'argv': _argv()},
        )
    assert not root.exists()


def test_exact_root_is_required_outside_explicit_test_alias(tmp_path):
    root = tmp_path / 'wrong-root'
    with pytest.raises(RUN.CandidateError, match='exact root'):
        RUN.run_formal(
            root=root,
            runtime={
                'injected': True,
                'argv': _argv(),
                'popen': lambda **_kwargs: _Process([]),
                'artifact_validator': lambda _root: {'status': 'PASS'},
            },
        )
    assert not root.exists()


def test_injected_main_reserves_fresh_root_then_runs_one_fake_popen_and_seals_pass(
    tmp_path, capsys
):
    root = tmp_path / 'fresh'
    events = []

    def fake_popen(argv, *, cwd, shell):
        events.append(
            ('popen', Path(cwd).exists(), (Path(cwd) / 'out').is_dir(), shell, list(argv))
        )
        return _Process(events)

    result = RUN.run_formal(
        root=root,
        runtime={
            'injected': True,
            'test_only_root_alias': True,
            'argv': _argv(),
            'popen': fake_popen,
            'artifact_validator': lambda value: {'status': 'PASS', 'root': str(value)},
        },
    )
    assert result['status'] == 'PASS'
    assert events[0][0] == 'popen' and events[0][1:4] == (True, True, False)
    assert events.count('wait') == 1
    receipt = root / 'closure_receipt.json'
    assert receipt.stat().st_mode & 0o777 == 0o444
    assert receipt.with_name(receipt.name + '.sha256').stat().st_mode & 0o777 == 0o444
    value = json.loads(receipt.read_text(encoding='utf-8'))
    assert value['execution']['popen_count'] == 1
    assert value['execution']['retry'] is False
    assert value['safety']['input_opened'] is False
    assert value['test_only_root_alias'] is True


def test_injected_nonzero_seals_fail_closed_without_retry(tmp_path):
    root = tmp_path / 'failure'
    events = []

    def fake_popen(argv, *, cwd, shell):
        events.append((list(argv), cwd, shell))
        return _Process(events, returncode=17)

    result = RUN.run_formal(
        root=root,
        runtime={
            'injected': True,
            'test_only_root_alias': True,
            'argv': _argv(),
            'popen': fake_popen,
            'artifact_validator': lambda _root: {'status': 'FAIL_CLOSED', 'reason': 'nonzero'},
        },
    )
    assert result['status'] == 'FAIL_CLOSED'
    assert result['execution']['popen_count'] == 1
    assert len([item for item in events if isinstance(item, tuple)]) == 1
    assert json.loads((root / 'closure_receipt.json').read_text())['execution']['retry'] is False


def test_popen_exception_seals_preprocess_fail_closed_without_retry(tmp_path):
    root = tmp_path / 'exception'
    calls = []

    def fake_popen(*_args, **_kwargs):
        calls.append('popen')
        raise RuntimeError('synthetic Popen failure')

    result = RUN.run_formal(
        root=root,
        runtime={
            'injected': True,
            'test_only_root_alias': True,
            'argv': _argv(),
            'popen': fake_popen,
            'artifact_validator': lambda _root: {'status': 'PASS'},
        },
    )
    assert result['status'] == 'FAIL_CLOSED'
    assert calls == ['popen']
    value = json.loads((root / 'closure_receipt.json').read_text())
    assert value['execution']['popen_count'] == 0
    assert value['execution']['retry'] is False


def test_shared_runtime_argv_rejects_second_rw_or_input_mount():
    bad = _argv() + ['--mount', 'type=bind,src=/bag,dst=/input/bag,readonly']
    with pytest.raises(AUTH.SharedAuthorizationError):
        AUTH.validate_runtime_argv(bad)
