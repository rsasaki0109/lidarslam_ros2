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


"""Injected authorized v16 wrapper tests; no Docker, ROS, or bag access."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'm6a10_v16_authorized_runner_test',
    ROOT / 'scripts/run_fast_livo2_m6a10_v16_formal_authorized.py',
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _window(path, **_kwargs):
    document = {
        'schema_version': 1,
        'contract_version': 'm6a10-quiescence-v1',
        'status': 'PASS',
        'runner_start_allowed': True,
        'observation': {'forbidden_processes': []},
    }
    MODULE.authorization._write_json(Path(path), document)
    return {
        'path': str(path),
        'sha256': MODULE.authorization.sha256_file(Path(path)),
        'status': 'PASS',
        'runner_start_allowed': True,
        'forbidden_processes': [],
    }


class _Monitor:
    def __init__(self, _root):
        self.started = False

    def start(self):
        self.started = True

    def allow_owned_pid(self, _pid):
        assert self.started

    def stop(self):
        self.started = False

    def finalize(self, _path):
        return {
            'status': 'PASS',
            'contaminated': False,
            'invalid': False,
            'coverage': {'coverage_gap': False},
        }


class _Process:
    pid = 816017

    def wait(self):
        return 0


def _terminal():
    counts = dict(MODULE.candidate.EXPECTED_COUNTS)
    return {
        'received_topic_counts': counts,
        'completed_counts': counts,
        'backend': {'completed_counts': counts},
        'buffers': {topic: {'count': 0, 'records': []} for topic in counts},
        'terminal_support_context': {
            'by_topic': {'lidar': 0, 'imu': 0, 'image': 0},
            'total_count': 0,
            'records': [],
        },
    }


def _callback():
    return {
        'schema_version': 3,
        'system': 'fast_livo2',
        'benchmark_only': True,
        'contract_version': MODULE.candidate.PHASE_CONTRACT,
        'transport_contract_version': MODULE.candidate.TRANSPORT_CONTRACT,
        'phase_mode': 'unpaced_ack',
        'status': 'pass',
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def test_authorized_wrapper_runs_candidate_once_and_uses_sealed_windows(tmp_path):
    auth_root = tmp_path / 'authorization'
    attempt_root = tmp_path / 'attempt'
    auth_result = MODULE.authorization.authorize(auth_root, attempt_root, window_runner=_window)
    auth_path = auth_root / MODULE.authorization.RECEIPT_NAME
    auth_sha = auth_result['receipt_sha256']
    config = MODULE.candidate.v12.CandidateConfig(
        root=attempt_root,
        profile_path=MODULE.candidate.PROFILE_PATH,
        authorization_path=auth_path,
        authorization_sha256=auth_sha,
    )
    events = []

    def identity(_config):
        events.append('identity')
        return {'image_id': MODULE.candidate.IMAGE_ID, 'opened': False}

    def bag(_config):
        events.append('bag')
        return {'path': 'fake', 'bytes': 0, 'sha256': 'fake', 'opened': False}

    def process(argv, _cwd):
        events.append(('popen', list(argv)))
        return _Process()

    def capture(_config, root):
        output = root / 'out'
        callback = _callback()
        terminal = _terminal()
        return {
            'terminal': terminal,
            'documents': {'callback': callback},
            'bindings': {'callback': {'path': str(output / 'callback.json'), 'sha256': 'fake'}},
        }

    def compose(_raw, _config):
        return {'status': 'PASS', 'contract': MODULE.candidate.PHASE_CONTRACT}

    result = MODULE.run_authorized(
        config,
        identity_probe=identity,
        bag_probe=bag,
        process_factory=process,
        monitor_factory=_Monitor,
        raw_capture=capture,
        composer=compose,
    )
    assert result['status'] == 'PASS'
    assert result['execution']['popen_count'] == 1
    assert result['preflight_quiescence']['sealed'] is True
    assert events.count('identity') == 1 and events.count('bag') == 1
    assert len([item for item in events if isinstance(item, tuple) and item[0] == 'popen']) == 1
    closure = attempt_root / 'closure_receipt.json'
    assert closure.is_file()
    document = json.loads(closure.read_text(encoding='utf-8'))
    assert document['status'] == 'PASS'


def test_authorized_wrapper_seals_pre_popen_failure(tmp_path):
    root = tmp_path / 'failed-attempt'
    missing_auth = tmp_path / 'missing-auth.json'
    args = [
        '--root',
        str(root),
        '--authorization',
        str(missing_auth),
        '--authorization-sha256',
        '0' * 64,
    ]
    assert MODULE.main(args) == 11
    closure = root / 'closure_receipt.json'
    assert closure.is_file()
    value = json.loads(closure.read_text(encoding='utf-8'))
    assert value['status'] == 'FAIL_CLOSED'
    assert value['pre_popen_failure'] is True
    assert value['execution']['popen_count'] == 0
    assert value['safety']['input_opened'] is False
    assert (root / 'closure_receipt.json.sha256').is_file()
