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


"""Host-only v14 scoped-builder and one-start regression tests."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_fast_livo2_m6a10_v14_formal.py'
SPEC = importlib.util.spec_from_file_location('m6a10_v14_formal', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _mounts(argv):
    return [argv[index + 1]
            for index, item in enumerate(argv[:-1]) if item == '--mount']


def test_v14_profile_binds_v13_failure_and_stays_unauthorized():
    profile = MODULE.verify_candidate_profile()
    assert profile['sha256'] == MODULE.hashlib.sha256(
        MODULE.PROFILE_PATH.read_bytes()).hexdigest()
    text = MODULE.PROFILE_PATH.read_text(encoding='utf-8')
    assert 'V14_FORMAL_CANDIDATE_UNAUTHORIZED' in text
    assert 'DOCKER_ARGV_BUILDER_RECURSION' in text
    assert 'feeder_root_cause_fixed: false' in text
    assert 'formal_replay_forbidden: true' in text


def test_original_builder_called_once_and_mount_contract_is_correct(tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'root')
    calls = []
    original = MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV

    def counted(config_value, output_value):
        calls.append((config_value, output_value))
        return original(config_value, output_value)

    MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = counted
    try:
        argv = MODULE.build_safe_docker_argv(config, tmp_path / 'out')
    finally:
        MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = original
    assert len(calls) == 1
    mounts = _mounts(argv)
    destinations = [MODULE._mount_destination(item) for item in mounts]
    assert destinations.count('/out') == 1
    assert destinations.count(MODULE.WRAPPER_DESTINATION) == 1
    assert not any(item == '--tmpfs' and argv[index + 1].startswith('/out:')
                   for index, item in enumerate(argv[:-1]))
    assert any(item.endswith(',dst=/out,readonly=false') for item in mounts)
    assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv and '--init' in argv and '--pull=never' in argv
    assert '--rm' not in argv


def test_injected_popen_exactly_once_and_scope_does_not_leak(tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'root')
    before = MODULE.v12.build_safe_docker_argv
    calls = []

    class FakeProcess:
        pid = 424242

        def wait(self):
            calls.append('wait')
            return 0

    def process_factory(argv, cwd):
        calls.append((list(argv), cwd))
        return FakeProcess()

    result = MODULE.run_injected_once(config, process_factory)
    assert result['start_count'] == 1
    assert result['popen_count'] == 1
    assert result['returncode'] == 0
    assert len([item for item in calls if isinstance(item, tuple)]) == 1
    assert calls[-1] == 'wait'
    assert MODULE.v12.build_safe_docker_argv is before
    second = MODULE.run_injected_once(
        MODULE.v12.CandidateConfig(
            root=tmp_path / 'root2'),
        process_factory)
    assert second['popen_count'] == 1
    assert MODULE.v12.build_safe_docker_argv is before


def test_scope_restores_builder_on_executor_exception(tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'root')
    before = MODULE.v12.build_safe_docker_argv

    def failing_factory(argv, cwd):
        raise RuntimeError('injected Popen failure')

    with pytest.raises(RuntimeError, match='injected Popen failure'):
        MODULE.run_injected_once(config, failing_factory)
    assert MODULE.v12.build_safe_docker_argv is before


def test_argv_rejects_extra_mount_and_capability_flag(tmp_path):
    config = MODULE.v12.CandidateConfig(root=tmp_path / 'root')
    original = MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV
    output = tmp_path / 'out'
    baseline = original(config, output)
    try:
        MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = lambda _config, _output: (
            list(baseline) + ['--mount', 'type=bind,src=/tmp/extra,dst=/extra,readonly']
        )
        with pytest.raises(MODULE.CandidateError, match='allowlist'):
            MODULE.build_safe_docker_argv(config, output)

        MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = lambda _config, _output: (
            list(baseline[:-1]) + ['--privileged', baseline[-1]]
        )
        with pytest.raises(MODULE.CandidateError, match='capability'):
            MODULE.build_safe_docker_argv(config, output)
    finally:
        MODULE.ORIGINAL_V12_BUILD_DOCKER_ARGV = original


def test_default_entrypoint_is_formal_fail_closed():
    with pytest.raises(SystemExit, match='FORMAL_REPLAY_UNAUTHORIZED'):
        MODULE.main([])
