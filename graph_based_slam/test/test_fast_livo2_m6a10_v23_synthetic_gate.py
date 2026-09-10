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


"""Synthetic-only v23 authorization and actual-argv gates."""

from __future__ import annotations

import copy
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

import fast_livo2_m6a10_v23_shared_auth as AUTH  # noqa: E402,I100
import run_fast_livo2_m6a10_v22_synthetic_gate as V22_GATE  # noqa: E402,I100
import run_fast_livo2_m6a10_v23_formal as FORMAL  # noqa: E402,I100


def test_synthetic_authorization_is_separate_from_formal_domain():
    value = AUTH.verify_synthetic_authorization()
    assert value['status'] == 'AUTHORIZED'
    assert value['gate_kind'] == 'synthetic_runtime_only'
    assert value['runtime_root'] == AUTH.V23_SYNTHETIC_RUNTIME_ROOT
    assert value['formal_fixture']['status'] == 'AUTHORIZED'
    assert value['formal_fixture']['receipt_sha256'] == AUTH.V22_AUTHORIZATION_SHA256
    assert value['formal_replay_started'] is False

    # The formal verifier is path/domain-bound and must reject this receipt.
    with pytest.raises(AUTH.SharedAuthorizationError, match='path drift'):
        AUTH.verify_v22_authorization(
            path=AUTH.V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH,
            expected_sha256=AUTH.V23_SYNTHETIC_AUTHORIZATION_SHA256,
        )


def test_synthetic_receipt_is_immutable_and_contains_three_fixture_windows():
    path = AUTH.V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH
    sidecar = path.with_name(path.name + '.sha256')
    assert path.stat().st_mode & 0o777 == 0o444
    assert sidecar.stat().st_mode & 0o777 == 0o444
    value = json.loads(path.read_text(encoding='utf-8'))
    assert value['formal_replay_authorized'] is False
    assert value['formal_replay_forbidden'] is True
    assert value['gate_kind'] == 'synthetic_runtime_only'
    assert len(value['windows']) == 3
    assert value['execution']['input_mount_count'] == 0


def test_v22_actual_container_argv_is_digest_bound_and_input_free(tmp_path):
    sources = V22_GATE._fake_sources(tmp_path, 'success')
    argv = V22_GATE.build_docker_argv(
        tmp_path, 'success', 'm6a10-v23-static-argv', sources['feeder']['sha256']
    )
    V22_GATE.validate_docker_argv(argv, 'm6a10-v23-static-argv')
    assert argv[-2:] == [V22_GATE.IMAGE_ID, '/runner/v22_runtime.sh']
    assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv and '--rm' not in argv
    mounts = [argv[i + 1] for i, item in enumerate(argv[:-1]) if item == '--mount']
    assert len(mounts) == 4
    assert sum(spec.endswith('dst=/out,readonly=false') for spec in mounts) == 1
    assert not any('dst=/input' in spec for spec in mounts)
    assert V22_GATE.IMAGE_TAG not in argv


def test_v23_profile_pins_synthetic_receipt_and_authorizer():
    profile = FORMAL.verify_profile()
    candidate = profile['candidate']
    assert candidate['synthetic_auth_receipt_sha256'] == AUTH.V23_SYNTHETIC_AUTHORIZATION_SHA256
    assert (
        candidate['synthetic_auth_sidecar_sha256']
        == AUTH.V23_SYNTHETIC_AUTHORIZATION_SIDECAR_SHA256
    )
    assert candidate['synthetic_authorizer_sha256'] == AUTH.V23_SYNTHETIC_AUTHORIZER_SHA256
    assert candidate['synthetic_runtime_root'] == AUTH.V23_SYNTHETIC_RUNTIME_ROOT
    assert profile['status'] == 'V23_FORMAL_CANDIDATE_UNAUTHORIZED'


@pytest.mark.parametrize('field', ['gate_kind', 'runtime_root', 'formal_replay_authorized'])
def test_synthetic_domain_mutation_is_not_accepted(monkeypatch, field):
    value = json.loads(AUTH.V23_SYNTHETIC_AUTHORIZATION_RECEIPT_PATH.read_text(encoding='utf-8'))
    mutated = copy.deepcopy(value)
    mutated[field] = {
        'gate_kind': 'formal',
        'runtime_root': '/tmp/wrong',
        'formal_replay_authorized': True,
    }[field]
    # Keep the immutable fixture untouched and exercise the same production
    # domain predicate used by the file verifier.
    assert mutated[field] != value[field]
    with pytest.raises(AUTH.SharedAuthorizationError):
        AUTH._validate_synthetic_domain_fields(
            mutated, expected_runtime_root=AUTH.V23_SYNTHETIC_RUNTIME_ROOT
        )
