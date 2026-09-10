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


"""Host-only tests for the additive v13 authorized formal candidate."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_fast_livo2_m6a10_v13_formal_authorized.py'
SPEC = importlib.util.spec_from_file_location('m6a10_v13_authorized', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _mounts(argv):
    return [argv[index + 1] for index, item in enumerate(argv[:-1]) if item == '--mount']


def test_profile_and_auth_source_identity_are_stable():
    assert MODULE._verify_profile()['sha256'] == MODULE.PROFILE_SHA256
    assert MODULE.authorizer._sha256(MODULE.AUTHORIZER_PATH) == MODULE.AUTHORIZER_SHA256
    assert (
        MODULE.authorizer._sha256(MODULE.mount_candidate.PARENT_LAUNCHER_PATH)
        == MODULE.mount_candidate.PARENT_LAUNCHER_SHA256
    )
    lineage = MODULE.authorizer._verify_immutable_lineage()
    assert lineage['feeder_root_cause_fixed'] is False
    assert lineage['feeder_underlying_nonzero_cause'] == 'unknown'


def test_v13_argv_is_mount_corrected_and_security_bound(tmp_path):
    config = MODULE.BASE.CandidateConfig(root=tmp_path / 'root')
    argv = MODULE.build_safe_docker_argv(config, tmp_path / 'out')
    mounts = _mounts(argv)
    destinations = [MODULE.mount_candidate._mount_destination(spec) for spec in mounts]
    assert destinations.count('/out') == 1
    assert destinations.count(MODULE.BASE.WRAPPER_PATH_IN_CONTAINER) == 1
    assert not any(
        item == '--tmpfs' and argv[index + 1].startswith('/out:')
        for index, item in enumerate(argv[:-1])
    )
    assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv and '--pull=never' in argv
    assert '--rm' not in argv
    assert any(spec.endswith(',dst=/out,readonly=false') for spec in mounts)
    assert any(spec.endswith(',dst=/runner/v12_runtime.sh,readonly') for spec in mounts)


def test_capture_preserves_feeder_log_and_exit_artifacts(tmp_path):
    root = tmp_path / 'attempt'
    out = root / 'out'
    out.mkdir(parents=True)
    (out / 'feeder.log').write_bytes(b'stderr: synthetic failure\n')
    (out / 'feeder_exit_status.txt').write_bytes(b'31\n')
    (out / 'feeder_receipt.json').write_text('{"status":"pass"}\n', encoding='utf-8')
    captured = MODULE._capture_with_feeder_diagnostics(None, root)
    assert captured['feeder_underlying_nonzero_cause'] == 'unknown'
    assert captured['bindings']['feeder_log']['bytes'] == len(b'stderr: synthetic failure\n')
    assert captured['documents']['feeder_exit_status'] == '31'
    assert 'callback' in captured['missing']
    assert 'terminal' in captured['missing']


def test_no_authorization_defaults_to_fail_closed(tmp_path):
    config = MODULE.BASE.CandidateConfig(root=tmp_path / 'fresh', profile_path=MODULE.PROFILE_PATH)
    with pytest.raises(MODULE.BASE.AuthorizationError) as error:
        MODULE.run_formal(config)
    assert error.value.kind == 'FORMAL_REPLAY_UNAUTHORIZED'
    assert not config.root.exists()
