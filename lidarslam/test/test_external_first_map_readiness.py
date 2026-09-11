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

"""Regression tests for the independent first-map adoption gate."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / 'scripts' / 'check_external_first_map_readiness.py'
LEDGER = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'external-first-map-validations.json'
)
SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'external-first-map-validations-v1.schema.json'
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'check_external_first_map_readiness',
        SCRIPT,
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _validation(index: int, path: str = 'docker-first-map') -> dict:
    return {
        'id': f'validation-{index}',
        'reporter': f'@external-user-{index}',
        'issue_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/issues/'
            f'{500 + index}'
        ),
        'submitted_at': f'2026-08-0{index}T12:00:00Z',
        'independent_attestation': True,
        'documentation_path': path,
        'release_ref': 'v0.9.0',
        'environment': {
            'os': 'Ubuntu 24.04',
            'architecture': 'amd64',
            'ros_distro': 'container-managed',
            'install_method': 'ghcr',
        },
        'exact_command': 'docker run --rm ghcr.io/example@sha256:abc',
        'result': 'passed',
        'verification': {
            'manifest_status': 'succeeded',
            'diagnosis_status': 'success',
            'autoware_status': 'PASS',
            'manifest_sha256': f'{index:064x}',
        },
        'findings': [],
        'acceptance': {
            'status': 'accepted',
            'reviewed_by': '@rsasaki0109',
            'reviewed_at': f'2026-08-0{index}T13:00:00Z',
            'review_url': (
                'https://github.com/rsasaki0109/lidar_slam_ros2/issues/'
                f'{500 + index}#issuecomment-1'
            ),
            'findings_status': 'no-findings',
            'resolution_urls': [],
        },
    }


def _ledger(validations: list[dict]) -> dict:
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
            'external-first-map-validations-v1.schema.json'
        ),
        'required_validations': 3,
        'program_url': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'external-first-map-validation.html'
        ),
        'validations': validations,
    }


def _write_ledger(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / 'ledger.json'
    path.write_text(
        json.dumps(payload, indent=2) + '\n',
        encoding='utf-8',
    )
    return path


def test_tracked_ledger_is_valid_and_honestly_not_ready():
    module = _load_module()
    report = module.validate_ledger(LEDGER, SCHEMA)

    assert report['status'] == 'NOT_READY'
    assert report['required_validations'] == 3
    assert report['accepted_validations'] == 0
    assert report['remaining_validations'] == 3
    assert report['distinct_reporters'] == 0
    assert set(report['documentation_path_counts']) == {
        'docker-first-map',
        'source-quickstart',
        'own-bag',
    }


def test_three_distinct_accepted_reports_complete_gate(tmp_path):
    module = _load_module()
    ledger = _write_ledger(
        tmp_path,
        _ledger([
            _validation(1, 'docker-first-map'),
            _validation(2, 'source-quickstart'),
            _validation(3, 'own-bag'),
        ]),
    )

    report = module.validate_ledger(ledger, SCHEMA)

    assert report['status'] == 'READY'
    assert report['accepted_validations'] == 3
    assert report['remaining_validations'] == 0
    assert report['documentation_path_counts'] == {
        'docker-first-map': 1,
        'source-quickstart': 1,
        'own-bag': 1,
    }
    assert module.main([
        '--ledger',
        str(ledger),
        '--schema',
        str(SCHEMA),
        '--require-complete',
    ]) == 0


def test_incomplete_valid_ledger_only_fails_strict_release_mode(
    tmp_path,
    capsys,
):
    module = _load_module()
    ledger = _write_ledger(tmp_path, _ledger([_validation(1)]))
    common = ['--ledger', str(ledger), '--schema', str(SCHEMA)]

    assert module.main([*common, '--json']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['status'] == 'NOT_READY'
    assert report['remaining_validations'] == 2
    assert module.main([*common, '--require-complete']) == 1


@pytest.mark.parametrize(
    'duplicate_field',
    [
        'id',
        'reporter',
        'issue_url',
        'manifest_sha256',
    ],
)
def test_duplicate_evidence_never_counts_as_independent(
    tmp_path,
    duplicate_field,
):
    module = _load_module()
    first = _validation(1)
    second = _validation(2)
    if duplicate_field == 'manifest_sha256':
        second['verification']['manifest_sha256'] = (
            first['verification']['manifest_sha256']
        )
    else:
        second[duplicate_field] = first[duplicate_field]
    ledger = _write_ledger(tmp_path, _ledger([first, second]))

    with pytest.raises(module.LedgerError, match='duplicate'):
        module.validate_ledger(ledger, SCHEMA)


@pytest.mark.parametrize(
    ('path', 'value'),
    [
        (('independent_attestation',), False),
        (('result',), 'failed'),
        (('verification', 'manifest_status'), 'failed'),
        (('verification', 'diagnosis_status'), 'failure'),
        (('verification', 'autoware_status'), 'FAIL'),
        (('acceptance', 'status'), 'pending'),
    ],
)
def test_unaccepted_or_unsuccessful_entry_is_schema_invalid(
    tmp_path,
    path,
    value,
):
    module = _load_module()
    entry = copy.deepcopy(_validation(1))
    target = entry
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    ledger = _write_ledger(tmp_path, _ledger([entry]))

    with pytest.raises(module.LedgerError, match='schema validation failed'):
        module.validate_ledger(ledger, SCHEMA)


def test_cli_returns_two_for_invalid_ledger(tmp_path, capsys):
    module = _load_module()
    ledger = _write_ledger(tmp_path, {'schema_version': 1})

    assert module.main([
        '--ledger',
        str(ledger),
        '--schema',
        str(SCHEMA),
    ]) == 2
    assert 'external first-map ledger invalid' in capsys.readouterr().err


@pytest.mark.parametrize(
    ('findings', 'status', 'urls'),
    [
        (
            ['Docker ownership was unclear.'],
            'no-findings',
            [],
        ),
        (
            ['Docker ownership was unclear.'],
            'resolved',
            [],
        ),
        (
            [],
            'documented',
            [
                'https://github.com/rsasaki0109/lidar_slam_ros2/issues/501',
            ],
        ),
    ],
)
def test_findings_need_consistent_public_disposition(
    tmp_path,
    findings,
    status,
    urls,
):
    module = _load_module()
    entry = _validation(1)
    entry['findings'] = findings
    entry['acceptance']['findings_status'] = status
    entry['acceptance']['resolution_urls'] = urls
    ledger = _write_ledger(tmp_path, _ledger([entry]))

    with pytest.raises(module.LedgerError, match='findings'):
        module.validate_ledger(ledger, SCHEMA)


def test_resolved_finding_with_public_link_is_accepted(tmp_path):
    module = _load_module()
    entry = _validation(1)
    entry['findings'] = ['Docker ownership was unclear.']
    entry['acceptance']['findings_status'] = 'resolved'
    entry['acceptance']['resolution_urls'] = [
        'https://github.com/rsasaki0109/lidar_slam_ros2/issues/501',
    ]
    ledger = _write_ledger(tmp_path, _ledger([entry]))

    report = module.validate_ledger(ledger, SCHEMA)

    assert report['accepted_validations'] == 1
    assert report['status'] == 'NOT_READY'
