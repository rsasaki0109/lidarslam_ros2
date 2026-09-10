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


"""Host-only regression tests for the v13 output-mount correction."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_fast_livo2_m6a10_v13_formal.py'
SPEC = importlib.util.spec_from_file_location('m6a10_v13_formal', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _mounts(argv):
    values = []
    for index, item in enumerate(argv[:-1]):
        if item in {'--mount', '--tmpfs'}:
            values.append((item, argv[index + 1]))
    return values


def _destinations(mounts):
    values = []
    for kind, mount in mounts:
        if kind == '--tmpfs':
            values.append(mount.split(':', 1)[0])
        else:
            values.append(next(field[4:] for field in mount.split(',')
                               if field.startswith('dst=')))
    return values


def test_failed_v12_lineage_is_sealed_and_unchanged():
    value = MODULE.verify_v13_lineage()
    assert value['candidate_version'] == 'v13-mount-corrected'
    assert value['failed_attempt']['closure_sha256'] == MODULE.FAILED_CLOSURE_SHA256
    assert set(value['failed_attempt']['diagnostics']) == {
        'container_inspect', 'container_logs', 'container_diff',
    }


def test_v12_duplicate_out_mount_is_removed_without_changing_other_mounts(
        tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'root')
    old = MODULE.v12.build_safe_docker_argv(config, tmp_path / 'out')
    new = MODULE.build_safe_docker_argv(config, tmp_path / 'out')
    old_mounts = _mounts(old)
    new_mounts = _mounts(new)
    assert _destinations(old_mounts).count('/out') == 2
    assert _destinations(new_mounts).count('/out') == 1
    assert old[-1] == new[-1] == MODULE.IMAGE_ID
    assert old[:old.index('--entrypoint')] != []
    assert '--network' in new and new[new.index('--network') + 1] == 'none'
    assert '--read-only' in new and '--pull=never' in new and '--init' in new
    assert '--rm' not in new
    assert all(item != 'rw' for item in new)
    assert any(
        dest == '/input/ntu_viral.bag' for dest in _destinations(new_mounts))
    assert any(
        dest == '/runner/v12_runtime.sh' for dest in _destinations(new_mounts))
    assert any(
        dest == '/runner/scripts/fast_livo2_m6a10_feeder.py' for dest in _destinations(new_mounts))
    assert [
        mount for kind,
        mount in new_mounts if kind == '--mount' and 'dst=/out,' in mount] == [
        'type=bind,src=%s,dst=/out,readonly=false' %
        (tmp_path /
         'out').resolve()]
    assert not any(item == '--tmpfs' and new[index + 1].startswith('/out:')
                   for index, item in enumerate(new[:-1]))


def test_v13_rejects_duplicate_destinations_and_keeps_authorization_fail_closed(
        tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'root')
    output = tmp_path / 'out'
    argv = MODULE.build_safe_docker_argv(config, output)
    duplicate_index = argv.index('--mount')
    duplicate = list(argv)
    duplicate.extend(
        ['--mount', 'type=bind,src=%s,dst=/out,readonly=false' % output.resolve()])
    original = MODULE.v12.build_safe_docker_argv
    MODULE.v12.build_safe_docker_argv = lambda _config, _output: duplicate
    try:
        with pytest.raises(MODULE.v12.CandidateError, match='duplicate'):
            MODULE.build_safe_docker_argv(config, output)
    finally:
        MODULE.v12.build_safe_docker_argv = original
    assert duplicate_index >= 0
    with pytest.raises(MODULE.v12.AuthorizationError) as error:
        MODULE.run_formal(config)
    assert error.value.kind == 'FORMAL_REPLAY_UNAUTHORIZED'
    assert not config.root.exists()


def test_v13_source_has_no_out_tmpfs_and_preserves_shell_free_lifecycle():
    text = SCRIPT.read_text(encoding='utf-8')
    assert 'v13-single-output-bind-no-duplicate-mount-v1' in text
    assert '--tmpfs\", \"/out' not in text
    assert 'duplicate Docker mount destination' in text
    assert 'shell=False' not in text
    assert 'v12.run_formal' in text
