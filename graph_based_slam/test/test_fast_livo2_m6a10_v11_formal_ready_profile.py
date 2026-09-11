#!/usr/bin/env python3

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


"""
Static gates for the v11 formal-ready profile transition.

This test reads YAML/receipt bytes only.  It never invokes the wrapper,
Docker, ROS, bag input, replay, GT, scorer, or map generation.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess

import yaml

ROOT = Path(__file__).resolve().parents[2]
CANDIDATE_PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v11_formal.yaml'
READY_PROFILE = ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v11_formal_ready.yaml'
CANDIDATE_WRAPPER = ROOT / 'scripts/fast_livo2_m6a10_v11_container_run.sh'
READY_WRAPPER = ROOT / 'scripts/fast_livo2_m6a10_v11_formal_container_run.sh'
READY_PROFILE_SHA256 = '6f56068a8a283851aa3a3723d2c9b526235bc97ba507a7dee4ea72bcd461bd2d'
CANDIDATE_PROFILE_SHA256 = 'e8980198e52604bb7dca0d1c1ecb91b6ce4f8b95b1dea3bb80ad4db8cd382296'
CANDIDATE_WRAPPER_SHA256 = '309ebb310e78f0d28dfa969e505bb0629df277947368433fe60fff009b6f9783'
V10_PROFILE_SHA256 = 'f706f41b0a985347ff98e4953532590c7a9c8c2d75db355e53cfb558f95bc459'
V10_CLOSURE_SHA256 = '16d1ee5bd55588391e3019bcdd7828c85f74c484f314f64a4dcc8a95a300fb05'
DUAL_ROOT = Path(
    '/media/sasaki/aiueo1/benchmarks/20260823/'
    'fast_livo2_v2c_v11_dual_service_20260823T121749Z_agentv11'
)
DUAL_RECEIPT = DUAL_ROOT / 'v11_dual_service.receipt.json'
DUAL_SIDECAR = DUAL_ROOT / 'v11_dual_service.receipt.json.sha256'
IDENTITY_RECEIPT = DUAL_ROOT / 'diagnostics/identity_preflight.receipt.json'
IDENTITY_SIDECAR = DUAL_ROOT / 'diagnostics/identity_preflight.receipt.json.sha256'
DUAL_SHA256 = '03f29eeb07eb6fde6f93996081333b5cfc54e3a6c920f38168f259790a52ec64'
DUAL_SIDECAR_SHA256 = '38dcf45ab17e862dc5a9c8cbb8c3191abdf7d4e398581f6cefd005c4df3d4392'
IDENTITY_SHA256 = '7578ec623e603adb5e6c54c75d655b9979b7b9e0dee78af4dbe1b23b35d308f3'
IDENTITY_SIDECAR_SHA256 = '296f05ab6a8b7bc5c773002ee8c5c804ea4bab97b1713da776a455d11c17fa63'


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding='utf-8'))


def _candidate() -> dict:
    return _yaml(CANDIDATE_PROFILE)['competitive_slam_profile']['m6a10_fast_livo2_v2c_v11']


def _ready() -> dict:
    return _yaml(READY_PROFILE)['competitive_slam_profile'][
        'm6a10_fast_livo2_v2c_v11_formal_ready'
    ]


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _receipt_and_sidecar(path: Path, sidecar: Path, expected: str) -> dict:
    assert path.is_file()
    assert sidecar.is_file()
    assert _sha(path) == expected
    assert (
        _sha(sidecar)
        == {
            DUAL_RECEIPT: DUAL_SIDECAR_SHA256,
            IDENTITY_RECEIPT: IDENTITY_SIDECAR_SHA256,
        }[path]
    )
    sidecar_line = sidecar.read_text(encoding='ascii').strip()
    assert sidecar_line.split()[0] == expected
    assert sidecar_line.endswith(path.name)
    return json.loads(path.read_text(encoding='utf-8'))


def test_ready_profile_supersedes_candidate_and_preserves_v10_lineage():
    assert _sha(CANDIDATE_PROFILE) == CANDIDATE_PROFILE_SHA256
    assert _sha(CANDIDATE_WRAPPER) == CANDIDATE_WRAPPER_SHA256
    assert (
        _sha(ROOT / 'configs/slam_benchmark_profiles/fast_livo2_m6a10_v10_formal.yaml')
        == V10_PROFILE_SHA256
    )

    profile = _yaml(READY_PROFILE)
    preregistration = profile['preregistration']
    ready = _ready()
    assert _sha(READY_PROFILE) == READY_PROFILE_SHA256
    assert preregistration['superseded_candidate_profile'] == (
        'configs/slam_benchmark_profiles/fast_livo2_m6a10_v11_formal.yaml'
    )
    assert preregistration['superseded_candidate_profile_sha256'] == CANDIDATE_PROFILE_SHA256
    assert preregistration['predecessor_profile'] == (
        'configs/slam_benchmark_profiles/fast_livo2_m6a10_v11_formal.yaml'
    )
    assert preregistration['predecessor_profile_sha256'] == CANDIDATE_PROFILE_SHA256
    assert preregistration['v10_predecessor_profile_sha256'] == V10_PROFILE_SHA256
    assert preregistration['v10_predecessor_closure_sha256'] == V10_CLOSURE_SHA256
    assert ready['superseded_candidate_profile']['sha256'] == CANDIDATE_PROFILE_SHA256
    assert ready['predecessor']['profile_sha256'] == CANDIDATE_PROFILE_SHA256
    assert ready['v10_lineage']['profile_sha256'] == V10_PROFILE_SHA256
    assert ready['v10_lineage']['closure_sha256'] == V10_CLOSURE_SHA256


def test_ready_receipts_bind_sidecars_and_expected_fail_closed_handshake():
    ready = _ready()
    source = ready['source']
    execution = ready['execution']
    dual = _receipt_and_sidecar(DUAL_RECEIPT, DUAL_SIDECAR, DUAL_SHA256)
    identity = _receipt_and_sidecar(IDENTITY_RECEIPT, IDENTITY_SIDECAR, IDENTITY_SHA256)

    assert source['dual_service_receipt_sha256'] == DUAL_SHA256
    assert source['dual_service_receipt_sidecar_sha256'] == DUAL_SIDECAR_SHA256
    assert source['identity_receipt_sha256'] == IDENTITY_SHA256
    assert source['identity_receipt_sidecar_sha256'] == IDENTITY_SIDECAR_SHA256
    assert execution['dual_service_receipt_sha256'] == DUAL_SHA256
    assert execution['identity_receipt_sha256'] == IDENTITY_SHA256

    assert dual['status'] == 'FAIL_CLOSED_EXPECTED'
    assert dual['contract'] == 'm6a10-online-compute-v4-terminal-bounded-end-gap'
    assert dual['production_attempt_count'] == 1
    assert dual['retry_count'] == 0
    assert dual['callback']['ack_first_success'] is True
    assert dual['callback']['duplicate_ack_success'] is False
    assert dual['callback']['exactly_once'] is True
    assert dual['callback']['published_messages'] == 1
    assert dual['services_present'] == [
        '/m6a10/consumer_status',
        '/m6a10/consumer_ack',
        '/m6a10/terminal_status',
        '/m6a10/terminal_eof',
        '/m6a10/terminal_finalize',
    ]
    assert dual['terminal']['eof_observed'] is True
    assert dual['terminal']['stable_poll_count'] == 2
    assert dual['terminal']['completed_boundary_observed'] is False
    assert dual['terminal']['evidence_status'] == 'invalid'
    assert dual['safety'] == {
        'formal_replay_started': False,
        'ground_truth_content_opened': False,
        'ground_truth_mounts': [],
        'input_bag_opened': False,
        'input_mounts': [],
        'map_save': False,
        'scorer_invoked': False,
        'scorer_mounts': [],
    }
    assert identity['status'] == 'PASS'
    assert identity['contract'] == 'm6a10-online-compute-v4-terminal-bounded-end-gap'
    assert identity['profile_sha256'] == CANDIDATE_PROFILE_SHA256
    assert identity['formal_replay_forbidden'] is True
    assert identity['ground_truth_present'] is False
    assert identity['scorer_present'] is False


def test_ready_semantic_transition_is_runtime_ready_but_formal_fail_closed():
    candidate = _candidate()
    ready = _ready()
    assert candidate['phase'] == ready['phase']
    assert candidate['input'] == ready['input']
    assert candidate['source']['runtime_ready'] is False
    assert candidate['source']['runtime_ready_for_preregistered_replay'] is False
    assert candidate['source']['dual_service_pending'] is True
    assert ready['status'] == 'build_synthetic_dual_service_validated'
    assert ready['source']['dual_service_no_input_status'] == (
        'PASS_EXPECTED_TERMINAL_FAIL_CLOSED'
    )
    assert ready['source']['dual_service_pending'] is False
    assert ready['source']['runtime_ready'] is True
    assert ready['source']['runtime_ready_for_preregistered_replay'] is True
    assert ready['execution']['dual_service_callbacks'] == 1
    assert ready['execution']['dual_service_ack_first_success'] is True
    assert ready['execution']['dual_service_duplicate_ack_rejected'] is True
    assert ready['execution']['dual_service_services_count'] == 5
    assert ready['execution']['dual_service_terminal_fail_closed_expected'] is True
    root = _yaml(READY_PROFILE)
    assert root['preregistration']['replay_authorized'] is False
    assert root['preregistration']['formal_replay_forbidden'] is True
    assert root['preregistration']['replay_count'] == 0
    assert ready['execution']['replay_count'] == 0
    assert ready['execution']['formal_replay_forbidden'] is True
    for field in (
        'input_opened',
        'input_mount_performed',
        'ground_truth_content_opened',
        'scorer_invoked',
        'map_saved',
    ):
        assert ready['execution'][field] is False


def test_ready_wrapper_is_candidate_copy_with_only_ready_identity_changes():
    candidate = CANDIDATE_WRAPPER.read_text(encoding='utf-8')
    expected = candidate
    replacements = (
        (
            '# Inside-container FAST-LIVO2 M6a10 v11 runtime entrypoint.',
            '# Inside-container FAST-LIVO2 M6a10 v11 formal-ready runtime entrypoint.',
        ),
        (
            "EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/"
            "fast_livo2_m6a10_v11_formal.yaml'",
            "EXPECTED_PROFILE_PATH='configs/slam_benchmark_profiles/"
            "fast_livo2_m6a10_v11_formal_ready.yaml'",
        ),
        (
            "EXPECTED_PROFILE_SHA256='e8980198e52604bb7dca0d1c1ecb91b6ce4f8b95b1dea3bb"
            "80ad4db8cd382296'",
            f"EXPECTED_PROFILE_SHA256='{READY_PROFILE_SHA256}'",
        ),
    )
    for old, new in replacements:
        assert expected.count(old) == 1
        expected = expected.replace(old, new, 1)
    assert READY_WRAPPER.read_text(encoding='utf-8') == expected
    assert "EXPECTED_CONTRACT='m6a10-online-compute-v4-terminal-bounded-end-gap'" in expected
    assert READY_WRAPPER.stat().st_mode & 0o111
    checked = subprocess.run(
        ['bash', '-n', str(READY_WRAPPER)], check=False, capture_output=True, text=True
    )
    assert checked.returncode == 0, checked.stderr
