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
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
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

"""Tests for the fail-closed v1.0 readiness audit."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_v1_readiness.py'
PACKAGE_MANAGER_REPORT_SCHEMA = (
    ROOT
    / 'docs'
    / 'schemas'
    / 'package-manager-release-readiness-v1.schema.json'
)
CURRENT_VERSION = (ROOT / 'VERSION').read_text(encoding='utf-8').strip()
SPEC = importlib.util.spec_from_file_location('v1_readiness', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
READINESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(READINESS)


def test_tracked_contract_reports_exact_open_product_gates():
    report = READINESS.evaluate_readiness(tags={'v0.9.0'})

    assert report['status'] == 'NOT_READY'
    assert report['product_version'] == CURRENT_VERSION
    assert report['summary'] == {
        'total': 10,
        'complete': 8,
        'incomplete': 2,
    }
    incomplete = {
        gate['id']
        for gate in report['gates']
        if gate['status'] == 'INCOMPLETE'
    }
    assert incomplete == {
        'distribution',
        'external-adoption',
    }
    assert report['external_first_map']['accepted_validations'] == 0
    assert report['release'] == {
        'expected_tag': 'v0.9.0',
        'minimum_version_met': True,
        'tag_present': True,
    }
    assert report['publication_audits'] == {
        'inspected': False,
        'ndt_omp_ros2_status': None,
        'package_manager_release_status': None,
        'lidarslam_release_status': None,
    }


def test_require_complete_exits_one_for_tracked_state():
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--require-complete', '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    assert json.loads(result.stdout)['status'] == 'NOT_READY'
    assert result.stderr == ''


def test_every_complete_gate_can_produce_ready_report(tmp_path):
    contract = json.loads(
        READINESS.DEFAULT_CONTRACT.read_text(encoding='utf-8'))
    for gate in contract['gates']:
        gate['state'] = 'complete'
        gate['blockers'] = []
        for evidence in gate['evidence']:
            path = tmp_path / evidence
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
    (tmp_path / 'VERSION').write_text('0.9.0\n', encoding='utf-8')
    contract_path = tmp_path / 'v1-readiness.json'
    contract_path.write_text(
        json.dumps(contract),
        encoding='utf-8',
    )
    external = {
        'schema_version': 1,
        'status': 'READY',
        'required_validations': 3,
        'accepted_validations': 3,
        'remaining_validations': 0,
        'distinct_reporters': 3,
        'documentation_path_counts': {
            'docker-first-map': 1,
            'source-quickstart': 1,
            'own-bag': 1,
        },
        'validation_ids': ['a', 'b', 'c'],
    }

    report = READINESS.evaluate_readiness(
        repo_root=tmp_path,
        contract_path=contract_path,
        tags={'v0.9.0'},
        external_report=external,
    )

    assert report['status'] == 'READY'
    assert report['summary']['complete'] == 10
    assert all(gate['blockers'] == [] for gate in report['gates'])


@pytest.mark.parametrize(
    ('ndt_status', 'release_status', 'package_manager_status', 'incomplete'),
    [
        ('RELEASED', 'PUBLISHED', 'READY', set()),
        ('IN_PROGRESS', 'PUBLISHED', 'READY', {'distribution'}),
        ('REVIEW_REQUIRED', 'PUBLISHED', 'READY', {'distribution'}),
        (
            'RELEASED',
            'NOT_PUBLISHED',
            'READY',
            {'reliability', 'release-publication'},
        ),
        ('RELEASED', 'PUBLISHED', 'NOT_RUN', {'distribution'}),
        ('RELEASED', 'PUBLISHED', 'SOURCE_REF_MISSING', {'distribution'}),
        ('RELEASED', 'PUBLISHED', 'RUNNING', {'distribution'}),
        ('RELEASED', 'PUBLISHED', 'FAILED', {'distribution'}),
        ('RELEASED', 'PUBLISHED', 'BLOCKED', {'distribution'}),
    ],
)
def test_live_publication_reports_are_required_for_claimed_complete_gates(
    tmp_path,
    ndt_status,
    release_status,
    package_manager_status,
    incomplete,
):
    contract = json.loads(
        READINESS.DEFAULT_CONTRACT.read_text(encoding='utf-8'))
    for gate in contract['gates']:
        gate['state'] = 'complete'
        gate['blockers'] = []
        for evidence in gate['evidence']:
            path = tmp_path / evidence
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
    (tmp_path / 'VERSION').write_text('0.9.0\n', encoding='utf-8')
    contract_path = tmp_path / 'v1-readiness.json'
    contract_path.write_text(json.dumps(contract), encoding='utf-8')
    external = {
        'status': 'READY',
        'accepted_validations': 3,
        'required_validations': 3,
        'remaining_validations': 0,
    }

    report = READINESS.evaluate_readiness(
        repo_root=tmp_path,
        contract_path=contract_path,
        tags={'v0.9.0'},
        external_report=external,
        require_live_publication=True,
        ndt_release_report={'status': ndt_status},
        published_release_report={'status': release_status},
        package_manager_report={'status': package_manager_status},
    )

    observed = {
        gate['id']
        for gate in report['gates']
        if gate['status'] == 'INCOMPLETE'
    }
    assert observed == incomplete
    assert report['status'] == ('NOT_READY' if incomplete else 'READY')
    assert report['publication_audits'] == {
        'inspected': True,
        'ndt_omp_ros2_status': ndt_status,
        'package_manager_release_status': package_manager_status,
        'lidarslam_release_status': release_status,
    }
    rendered = READINESS.render_markdown(report)
    assert (
        f'**{ndt_status} / {package_manager_status} / {release_status}**'
        in rendered
    )


def test_package_manager_status_allowlist_matches_child_report_schema():
    schema = json.loads(PACKAGE_MANAGER_REPORT_SCHEMA.read_text(
        encoding='utf-8'))

    assert set(schema['properties']['status']['enum']) == (
        READINESS.PACKAGE_MANAGER_RELEASE_STATUSES)


def test_unknown_package_manager_status_remains_invalid():
    with pytest.raises(
        READINESS.ReadinessError,
        match='package-manager live distribution report has invalid status',
    ):
        READINESS.evaluate_readiness(
            require_live_publication=True,
            ndt_release_report={'status': 'RELEASED'},
            published_release_report={'status': 'PUBLISHED'},
            package_manager_report={'status': 'UNKNOWN'},
        )


def test_live_evaluation_fails_without_both_publication_reports(tmp_path):
    with pytest.raises(
        READINESS.ReadinessError,
        match='live publication and distribution reports are required',
    ):
        READINESS.evaluate_readiness(
            require_live_publication=True,
            ndt_release_report={'status': 'RELEASED'},
        )


def test_checker_load_failure_is_normalized_as_readiness_error(tmp_path):
    scripts = tmp_path / 'scripts'
    scripts.mkdir()
    checker = scripts / 'broken.py'
    checker.write_text('this is not valid Python !!!\n', encoding='utf-8')

    with pytest.raises(
        READINESS.ReadinessError,
        match='cannot load readiness checker',
    ):
        READINESS._load_checker(tmp_path, 'broken.py', 'broken_for_v1')


def test_require_complete_escalates_a_locally_ready_report_to_live(
    monkeypatch,
    capsys,
):
    reports = [
        {
            'status': 'READY',
            'product_version': CURRENT_VERSION,
            'release': {'expected_tag': 'v0.9.0'},
        },
        {
            'status': 'NOT_READY',
            'publication_audits': {
                'inspected': True,
                'ndt_omp_ros2_status': 'IN_PROGRESS',
                'package_manager_release_status': 'NOT_RUN',
                'lidarslam_release_status': 'NOT_PUBLISHED',
            },
        },
    ]
    evaluation_calls = []
    live_calls = []

    def fake_evaluate(**kwargs):
        evaluation_calls.append(kwargs)
        return reports.pop(0)

    monkeypatch.setattr(READINESS, 'evaluate_readiness', fake_evaluate)

    def fake_live(**kwargs):
        live_calls.append(kwargs)
        return (
            {'status': 'IN_PROGRESS'},
            {'status': 'NOT_PUBLISHED'},
            {'status': 'NOT_RUN'},
        )

    monkeypatch.setattr(
        READINESS,
        'inspect_live_publication',
        fake_live,
    )
    monkeypatch.setattr(
        READINESS,
        'render_markdown',
        lambda report: f"{report['status']}\n",
    )

    result = READINESS.main(['--require-complete'])

    assert result == 1
    assert capsys.readouterr().out == 'NOT_READY\n'
    assert live_calls == [{
        'repo_root': ROOT,
        'product_version': CURRENT_VERSION,
        'published_release_version': '0.9.0',
    }]
    assert len(evaluation_calls) == 2
    assert evaluation_calls[1]['require_live_publication'] is True
    assert evaluation_calls[1]['ndt_release_report']['status'] == 'IN_PROGRESS'
    assert (
        evaluation_calls[1]['published_release_report']['status']
        == 'NOT_PUBLISHED'
    )
    assert (
        evaluation_calls[1]['package_manager_report']['status']
        == 'NOT_RUN'
    )


def test_evidence_cannot_escape_repository(tmp_path):
    contract = json.loads(
        READINESS.DEFAULT_CONTRACT.read_text(encoding='utf-8'))
    contract['gates'][0]['evidence'] = ['../outside']
    contract_path = tmp_path / 'v1-readiness.json'
    contract_path.write_text(json.dumps(contract), encoding='utf-8')
    (tmp_path / 'VERSION').write_text('0.9.0\n', encoding='utf-8')

    with pytest.raises(
        READINESS.ReadinessError,
        match='evidence path escapes repository',
    ):
        READINESS.evaluate_readiness(
            repo_root=tmp_path,
            contract_path=contract_path,
            tags={'v0.9.0'},
            external_report={'status': 'READY'},
        )
