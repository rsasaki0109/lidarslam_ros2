# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import annotations

import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'evaluate_bim_regression_gate.py'
PROFILE = REPO_ROOT / 'tools' / 'colored_map' / 'bim_regression_profile_v1.json'


def _load():
    spec = importlib.util.spec_from_file_location('evaluate_bim_regression_gate', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


gate = _load()


def test_evaluate_case_reports_exact_min_max_and_missing_metrics():
    metrics = {'summary': {'walls': 4, 'rooms': 1, 'rmse': 0.04}}
    rules = {
        'summary.walls': {'exact': 4},
        'summary.rooms': {'min': 2},
        'summary.rmse': {'max': 0.03},
        'summary.windows': {'min': 1},
    }
    violations = gate.evaluate_case(metrics, rules)
    assert violations == [
        'summary.rooms: 1 < 2',
        'summary.rmse: 0.04 > 0.03',
        'summary.windows: metric is missing',
    ]


def test_suite_requires_every_profile_case():
    profile = {'name': 'test', 'cases': {
        'present': {'rules': {'summary.rooms': {'exact': 1}}},
        'missing': {'rules': {}},
    }}
    result = gate.evaluate_suite(profile, {'present': {'summary': {'rooms': 1}}})
    assert result['passed'] is False
    assert result['cases']['present']['passed'] is True
    assert result['cases']['missing']['violations'] == [
        'required case metrics are missing']


def test_builtin_closed_room_exercises_real_topology_pipeline():
    metrics = gate.load_case('builtin:closed-room')
    assert metrics['summary']['observed_walls'] == 4
    assert metrics['summary']['accepted_rooms'] == 1
    assert metrics['summary']['dangling_wall_ends'] == 0
    assert metrics['rooms'][0]['generation_method'] == 'LiDAR wall topology cycle'


def test_versioned_profile_gates_all_required_evidence_cases():
    profile = json.loads(PROFILE.read_text())
    assert set(profile['cases']) == {'synthetic_closed_room', 'exp01', 'exp07'}
    synthetic = gate.load_case('builtin:closed-room')
    violations = gate.evaluate_case(
        synthetic, profile['cases']['synthetic_closed_room']['rules'])
    assert violations == []


def test_cli_writes_passing_and_failing_gate_reports(tmp_path):
    profile = tmp_path / 'profile.json'
    profile.write_text(json.dumps({'name': 'synthetic', 'cases': {
        'room': {'rules': {
            'summary.observed_walls': {'exact': 4},
            'summary.accepted_rooms': {'exact': 1},
        }}
    }}))
    output = tmp_path / 'gate.json'
    assert gate.main(['--profile', str(profile), '--case',
                      'room=builtin:closed-room', '--output', str(output)]) == 0
    assert json.loads(output.read_text())['passed'] is True

    profile.write_text(json.dumps({'name': 'synthetic', 'cases': {
        'room': {'rules': {'summary.accepted_rooms': {'exact': 2}}}
    }}))
    assert gate.main(['--profile', str(profile), '--case',
                      'room=builtin:closed-room', '--output', str(output)]) == 1
    assert json.loads(output.read_text())['passed'] is False
