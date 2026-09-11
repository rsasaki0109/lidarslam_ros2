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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for comparable, privacy-bounded onboarding trial records."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_onboarding_trial.py'
SPEC = importlib.util.spec_from_file_location('check_onboarding_trial', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
TRIAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TRIAL)

SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/onboarding-trial-v1.schema.json'
)


def _comparable_trial() -> dict[str, object]:
    return {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'trial_id': 'docker-humble-v090-20260810',
        'captured_at': '2026-08-10T03:10:00Z',
        'documentation_path': 'docker-first-map',
        'operator_class': 'maintainer',
        'environment': {
            'clean_start': True,
            'ros_distro': 'humble',
            'architecture': 'x86_64',
            'os_family': 'ubuntu-22.04',
            'product_version': '0.9.0',
            'revision': {
                'kind': 'image-digest',
                'value': 'sha256:' + ('a' * 64),
            },
        },
        'input': {
            'dataset_class': 'fixed-public',
            'dataset_id': 'mid360-public-zenodo-14841855',
            'download_bytes': 517088133,
        },
        'measurements': {
            'workflow_download_bytes': 1800000000,
            'wall_time_sec': 83.53,
            'active_operator_time_sec': 45.0,
            'command_count': 1,
            'peak_disk_bytes': 700000000,
            'output_bytes': 127000000,
        },
        'outcome': {
            'status': 'PASS',
            'runner_exit_code': 0,
            'manifest_status': 'succeeded',
            'diagnosis_status': 'success',
            'verifier_status': 'PASS',
            'receipt_status': 'PASS',
            'undocumented_manual_steps': 0,
            'failure_stage': 'none',
            'finding_codes': [],
        },
        'evidence': {
            'manifest_sha256': 'b' * 64,
            'receipt_sha256': 'c' * 64,
        },
        'privacy': {
            'contains_private_paths': False,
            'contains_exact_command': False,
            'contains_operator_identity': False,
            'review_before_sharing': True,
        },
    }


def _validation_receipt(record: dict[str, object]) -> bytes:
    revision = record['environment']['revision']
    git_commit = (
        revision['value'] if revision['kind'] == 'git-commit' else None
    )
    receipt = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/first-map-validation-receipt-v1.schema.json'
        ),
        'status': 'PASS',
        'run': {
            'run_id': 'bounded-test-run',
            'product_version': record['environment']['product_version'],
            'git_commit': git_commit,
            'profile_id': 'rko_lio_graph_mid360_preset',
        },
        'verification': {
            'manifest_status': 'succeeded',
            'diagnosis_status': 'success',
            'autoware_status': 'PASS',
            'manifest_sha256': record['evidence']['manifest_sha256'],
        },
        'evidence': {
            'manifest': {
                'filename': 'run_manifest.json',
                'sha256': record['evidence']['manifest_sha256'],
            },
            'diagnosis': {
                'filename': 'autoware_map_diagnosis.json',
                'available': True,
                'sha256': 'd' * 64,
            },
            'verify_log': {
                'filename': 'verify_autoware_map.log',
                'available': True,
                'sha256': 'e' * 64,
            },
        },
        'checks': [
            {'id': f'check_{index}', 'passed': True, 'observed': 'pass'}
            for index in range(7)
        ],
        'shareability': {
            'contains_map_geometry': False,
            'contains_private_paths': False,
            'contains_exact_command': False,
            'review_before_sharing': True,
        },
    }
    payload = json.dumps(receipt, sort_keys=True).encode() + b'\n'
    record['evidence']['receipt_sha256'] = hashlib.sha256(payload).hexdigest()
    return payload


def _evaluate_intrinsic(record: dict[str, object]) -> dict[str, object]:
    return TRIAL.evaluate_trial(
        record,
        require_evidence_binding=False,
    )


def test_complete_pass_is_comparable():
    """Intrinsic completeness remains available to trusted probe callers."""
    report = _evaluate_intrinsic(_comparable_trial())

    assert report == {
        'schema_version': 1,
        'trial_id': 'docker-humble-v090-20260810',
        'outcome_status': 'PASS',
        'measurement_status': 'COMPLETE',
        'comparable': True,
        'missing_measurements': [],
        'comparability_blockers': [],
    }


def test_comparability_gate_requires_exact_validation_receipt_bytes():
    """A public comparable claim requires the exact retained receipt."""
    record = _comparable_trial()
    unbound = TRIAL.evaluate_trial(
        record,
        require_evidence_binding=True,
    )
    assert unbound['comparability_blockers'] == [
        'validation_receipt_unbound'
    ]

    receipt = _validation_receipt(record)
    bound = TRIAL.evaluate_trial(
        record,
        validation_receipt_bytes=receipt,
        require_evidence_binding=True,
    )
    assert bound['comparable'] is True

    with pytest.raises(TRIAL.TrialError, match='SHA-256'):
        TRIAL.evaluate_trial(
            record,
            validation_receipt_bytes=receipt + b' ',
            require_evidence_binding=True,
        )


def test_missing_values_are_reported_without_fabricating_measurements():
    record = _comparable_trial()
    record['input']['download_bytes'] = None
    record['measurements']['active_operator_time_sec'] = None

    report = _evaluate_intrinsic(record)

    assert report['measurement_status'] == 'INCOMPLETE'
    assert report['comparable'] is False
    assert report['missing_measurements'] == [
        'input.download_bytes',
        'measurements.active_operator_time_sec',
    ]
    assert report['comparability_blockers'] == ['measurements_incomplete']


@pytest.mark.parametrize(
    ('field', 'value', 'blocker'),
    [
        ('clean_start', False, 'environment_not_clean'),
        ('revision', {'kind': 'release-tag', 'value': 'v0.9.0'},
         'revision_not_immutable'),
    ],
)
def test_mutable_or_prepared_environment_is_not_comparable(
    field, value, blocker,
):
    record = _comparable_trial()
    record['environment'][field] = value

    report = _evaluate_intrinsic(record)

    assert report['comparable'] is False
    assert report['comparability_blockers'] == [blocker]


@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    [
        ('wall_time_sec', 0.0, 'wall_time_sec must be greater than zero'),
        (
            'workflow_download_bytes',
            500000000,
            'workflow_download_bytes cannot be less than input.download_bytes',
        ),
        (
            'active_operator_time_sec',
            100.0,
            'active_operator_time_sec cannot exceed wall_time_sec',
        ),
        (
            'output_bytes',
            800000000,
            'output_bytes cannot exceed peak_disk_bytes',
        ),
    ],
)
def test_impossible_measurements_fail_closed(field, value, message):
    record = _comparable_trial()
    record['measurements'][field] = value

    with pytest.raises(TRIAL.TrialError, match=message):
        _evaluate_intrinsic(record)


def test_pass_requires_the_complete_success_contract():
    record = _comparable_trial()
    record['outcome']['runner_exit_code'] = 1
    record['outcome']['undocumented_manual_steps'] = 1

    with pytest.raises(TRIAL.TrialError, match='runner_exit_code') as exc_info:
        _evaluate_intrinsic(record)

    assert 'undocumented_manual_steps' in str(exc_info.value)


@pytest.mark.parametrize(
    ('failure_stage', 'finding_codes', 'message'),
    [
        ('none', ['mapping-timeout'], 'must identify a failure_stage'),
        ('mapping', [], 'must include at least one finding code'),
    ],
)
def test_failed_trial_requires_actionable_classification(
    failure_stage, finding_codes, message,
):
    record = _comparable_trial()
    record['outcome']['status'] = 'FAIL'
    record['outcome']['failure_stage'] = failure_stage
    record['outcome']['finding_codes'] = finding_codes

    with pytest.raises(TRIAL.TrialError, match=message):
        _evaluate_intrinsic(record)


def test_valid_failed_trial_is_retained_but_not_comparable():
    record = _comparable_trial()
    record['outcome'].update({
        'status': 'FAIL',
        'runner_exit_code': 1,
        'manifest_status': 'failed',
        'diagnosis_status': 'failure',
        'verifier_status': 'NOT_RUN',
        'receipt_status': 'NOT_CREATED',
        'failure_stage': 'mapping',
        'finding_codes': ['mapping-timeout'],
    })
    record['evidence']['receipt_sha256'] = None

    report = _evaluate_intrinsic(record)

    assert report['outcome_status'] == 'FAIL'
    assert report['measurement_status'] == 'COMPLETE'
    assert report['comparability_blockers'] == ['outcome_failed']


def test_evidence_hash_availability_must_match_outcome_status():
    record = _comparable_trial()
    record['evidence']['receipt_sha256'] = None

    with pytest.raises(TRIAL.TrialError, match='receipt status'):
        _evaluate_intrinsic(record)


def test_schema_rejects_paths_and_fractional_byte_counts():
    record = _comparable_trial()
    record['input']['dataset_id'] = '/home/operator/private/demo'

    with pytest.raises(TRIAL.TrialError, match='schema failed'):
        _evaluate_intrinsic(record)

    record = _comparable_trial()
    record['measurements']['output_bytes'] = 1.5
    with pytest.raises(TRIAL.TrialError, match='schema failed'):
        _evaluate_intrinsic(record)


def test_cli_distinguishes_comparable_incomplete_and_invalid_records(
    tmp_path, capsys,
):
    record = _comparable_trial()
    receipt = _validation_receipt(record)
    record_path = tmp_path / 'trial.json'
    record_path.write_text(json.dumps(record), encoding='utf-8')
    receipt_path = tmp_path / 'first-map-validation-receipt.json'
    receipt_path.write_bytes(receipt)

    assert TRIAL.main([
        str(record_path),
        '--validation-receipt', str(receipt_path),
        '--json',
        '--require-comparable',
    ]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['comparable'] is True

    incomplete = _comparable_trial()
    incomplete['measurements']['peak_disk_bytes'] = None
    record_path.write_text(json.dumps(incomplete), encoding='utf-8')
    assert TRIAL.main([str(record_path), '--require-comparable']) == 1
    assert 'Comparable onboarding baseline: **NO**' in capsys.readouterr().out

    invalid = _comparable_trial()
    invalid['trial_id'] = 'private/path'
    record_path.write_text(json.dumps(invalid), encoding='utf-8')
    assert TRIAL.main([str(record_path)]) == 2
    assert 'schema failed' in capsys.readouterr().err
