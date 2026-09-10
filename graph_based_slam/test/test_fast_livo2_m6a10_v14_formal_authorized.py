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


"""Injected v14 authorized-lifecycle regressions; no Docker or bag access."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts/run_fast_livo2_m6a10_v14_formal_authorized.py'
SPEC = importlib.util.spec_from_file_location('m6a10_v14_authorized', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _authorization(_config):
    return {'authorized': True, 'formal_execution': False, 'candidate_version': 'test'}


class _Monitor:
    def __init__(self, _root):
        self.events = []

    def start(self):
        self.events.append('start')

    def allow_owned_pid(self, pid):
        self.events.append(('allow', pid))

    def stop(self):
        self.events.append('stop')

    def finalize(self, _path):
        self.events.append('finalize')
        return {
            'status': 'PASS',
            'contaminated': False,
            'invalid': False,
            'coverage': {'coverage_gap': False},
        }


class _Process:
    pid = 900001

    def __init__(self, events):
        self.events = events

    def wait(self):
        self.events.append('wait')
        return 0


def test_injected_authorized_path_builds_once_starts_once_and_restores(tmp_path, monkeypatch):
    config = MODULE.BASE.CandidateConfig(
        root=tmp_path / 'attempt',
        profile_path=MODULE.PROFILE_PATH,
        authorization_path=tmp_path / 'auth.json',
        authorization_sha256='test',
    )
    monkeypatch.setattr(MODULE, '_verify_authorization', _authorization)
    before = MODULE.BASE.build_safe_docker_argv
    original = MODULE.candidate.ORIGINAL_V12_BUILD_DOCKER_ARGV
    builder_calls = []
    process_events = []

    def counted_builder(value, output):
        builder_calls.append((value, output))
        return original(value, output)

    monkeypatch.setattr(MODULE.candidate, 'ORIGINAL_V12_BUILD_DOCKER_ARGV', counted_builder)

    def process_factory(argv, cwd):
        process_events.append((list(argv), cwd))
        return _Process(process_events)

    result = MODULE.run_formal(
        config,
        identity_probe=lambda _config: {'image_id': MODULE.BASE.IMAGE_ID, 'opened': False},
        bag_probe=lambda _config: {
            'path': 'injected',
            'bytes': 0,
            'sha256': 'test',
            'opened': False,
        },
        process_factory=process_factory,
        monitor_factory=_Monitor,
        raw_capture=lambda _config, _root: {'terminal': {}, 'documents': {}, 'bindings': {}},
    )
    # conservation is intentionally absent in this seam
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] in {
        'COUNTS_INVALID',
        'CONSERVATION_INVALID',
        'CONSERVATION_BACKEND',
    }
    assert len(builder_calls) == 1
    assert len([event for event in process_events if isinstance(event, tuple)]) == 1
    assert MODULE.BASE.build_safe_docker_argv is before
    assert not any(item == '--rm' for item in process_events[0][0])
    mounts = [
        process_events[0][0][index + 1]
        for index, item in enumerate(process_events[0][0][:-1])
        if item == '--mount'
    ]
    assert sum('dst=/out,' in item for item in mounts) == 1
    assert not any(
        item == '--tmpfs' and process_events[0][0][index + 1].startswith('/out:')
        for index, item in enumerate(process_events[0][0][:-1])
    )


def test_injected_authorized_path_restores_after_process_exception(tmp_path, monkeypatch):
    config = MODULE.BASE.CandidateConfig(
        root=tmp_path / 'attempt',
        profile_path=MODULE.PROFILE_PATH,
        authorization_path=tmp_path / 'auth.json',
        authorization_sha256='test',
    )
    monkeypatch.setattr(MODULE, '_verify_authorization', _authorization)
    before = MODULE.BASE.build_safe_docker_argv

    def failing_factory(_argv, _cwd):
        raise RuntimeError('injected executor failure')

    result = MODULE.run_formal(
        config,
        identity_probe=lambda _config: {'image_id': MODULE.BASE.IMAGE_ID, 'opened': False},
        bag_probe=lambda _config: {
            'path': 'injected',
            'bytes': 0,
            'sha256': 'test',
            'opened': False,
        },
        process_factory=failing_factory,
        monitor_factory=_Monitor,
    )
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'PROCESS_LIFECYCLE_FAILURE'
    assert MODULE.BASE.build_safe_docker_argv is before


def test_authorizer_and_source_pins_are_explicit():
    assert MODULE.AUTHORIZER_SHA256 != 'PENDING_V14_AUTHORIZER_SHA256'
    assert (
        MODULE.authorizer.CANDIDATE_LAUNCHER_SHA256
        == MODULE.candidate.PROFILE_SHA256[:0]
        + '26610f1de4f0cec2d231e54641c88300dbe3fefb8fb1dfbbec686a2acb266459'
    )
    assert (
        MODULE.authorizer.V13_CLOSURE_SHA256
        == '73252dd828388a5ff026413ec957cb02504dea4ba14b936678257333954b8212'
    )
    assert (
        MODULE.authorizer.V13_CORRECTION_SHA256
        == '24c9ceb1d2b2da8e8ec95c54aa00ad723f1713146ff21a4b235533ebf776fd0e'
    )
