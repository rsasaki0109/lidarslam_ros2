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

"""Tests for the fixed onboarding trial matrix gate."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import sys
from pathlib import Path  # noqa: I100

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / 'check_onboarding_trial_matrix.py'
SPEC = importlib.util.spec_from_file_location(
    'check_onboarding_trial_matrix', SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
MATRIX = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MATRIX)

TRIAL_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/onboarding-trial-v1.schema.json'
)


def _trial(route: str, distro: str) -> dict[str, object]:
    docker = route == 'docker'
    return {
        'schema_version': 1,
        'schema_uri': TRIAL_SCHEMA_URI,
        'trial_id': f'g0-{route}-{distro}-20260811-a',
        'captured_at': '2026-08-11T03:10:00Z',
        'documentation_path': (
            'docker-first-map' if docker else 'source-quickstart'
        ),
        'operator_class': 'maintainer',
        'environment': {
            'clean_start': True,
            'ros_distro': distro,
            'architecture': 'x86_64',
            'os_family': (
                'ubuntu-22.04' if distro == 'humble'
                else 'ubuntu-24.04'
            ),
            'product_version': '0.9.0',
            'revision': {
                'kind': 'image-digest' if docker else 'git-commit',
                'value': (
                    'sha256:' + (('a' if distro == 'humble' else 'b') * 64)
                    if docker else 'c' * 40
                ),
            },
        },
        'input': {
            'dataset_class': 'fixed-public',
            'dataset_id': MATRIX.DATASET_ID,
            'download_bytes': MATRIX.DATASET_BYTES,
        },
        'measurements': {
            'workflow_download_bytes': 1800000000,
            'wall_time_sec': 1000.0,
            'active_operator_time_sec': 45.0,
            'command_count': 1 if docker else 9,
            'peak_disk_bytes': 7000000000,
            'output_bytes': 130000000,
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
            'manifest_sha256': 'd' * 64,
            'receipt_sha256': 'e' * 64,
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
            'run_id': record['trial_id'],
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
                'sha256': '1' * 64,
            },
            'verify_log': {
                'filename': 'verify_autoware_map.log',
                'available': True,
                'sha256': '2' * 64,
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


def _bind(record: dict[str, object]) -> MATRIX.MatrixRecord:
    payload = _validation_receipt(record)
    if isinstance(record, MATRIX.MatrixRecord):
        record.validation_receipt_bytes = payload
        return record
    return MATRIX.MatrixRecord(
        record,
        validation_receipt_bytes=payload,
    )


def _full_matrix() -> list[dict[str, object]]:
    return [
        _bind(_trial('docker', 'humble')),
        _bind(_trial('docker', 'jazzy')),
        _bind(_trial('source', 'humble')),
        _bind(_trial('source', 'jazzy')),
    ]


def _evidence_index() -> dict[str, object]:
    return json.loads(MATRIX.EVIDENCE_INDEX_PATH.read_text(encoding='utf-8'))


def _make_failed(record: dict[str, object], finding: str) -> None:
    record['outcome'].update({
        'status': 'FAIL',
        'runner_exit_code': None,
        'manifest_status': 'missing',
        'diagnosis_status': 'missing',
        'verifier_status': 'NOT_RUN',
        'receipt_status': 'NOT_CREATED',
        'failure_stage': 'preflight',
        'finding_codes': [finding],
    })
    record['evidence'].update({
        'manifest_sha256': None,
        'receipt_sha256': None,
    })
    if isinstance(record, MATRIX.MatrixRecord):
        record.validation_receipt_bytes = None


def test_all_four_comparable_rows_pass_both_matrix_gates():
    """Four comparable rows satisfy activation and strict matrix gates."""
    report = MATRIX.evaluate_matrix(_full_matrix())

    assert report['decision']['status'] == 'ALL_ROWS_COMPARABLE'
    assert report['summary'] == {
        'required_rows': 4,
        'present_rows': 4,
        'pass_rows': 4,
        'comparable_rows': 4,
        'docker_comparable_rows': 2,
        'source_comparable_rows': 2,
        'product_version_aligned': True,
        'matrix_complete': True,
        'activation_gate': True,
        'all_rows_comparable': True,
    }
    assert report['decision']['actions'] == []


def test_unbound_or_changed_receipt_cannot_open_matrix_comparability():
    """Missing or changed retained bytes keep the fixed matrix closed."""
    records = _full_matrix()
    records[0].validation_receipt_bytes = None

    report = MATRIX.evaluate_matrix(records)

    docker_humble = report['rows'][0]
    assert docker_humble['comparable'] is False
    assert docker_humble['comparability_blockers'] == [
        'validation_receipt_unbound'
    ]

    records = _full_matrix()
    records[0].validation_receipt_bytes += b' '
    with pytest.raises(MATRIX.MatrixError, match='SHA-256'):
        MATRIX.evaluate_matrix(records)


def test_partial_matrix_names_missing_rows_without_inferring_success():
    """Absent source records remain explicit missing rows."""
    records = _full_matrix()[:2]
    for record in records:
        record['measurements']['active_operator_time_sec'] = None
        record['measurements']['command_count'] = None
        record['measurements']['peak_disk_bytes'] = None

    report = MATRIX.evaluate_matrix(records)

    assert report['decision']['status'] == 'INCOMPLETE'
    assert report['summary']['present_rows'] == 2
    assert report['summary']['pass_rows'] == 2
    assert report['summary']['comparable_rows'] == 0
    assert [
        row['row_id'] for row in report['rows'] if not row['present']
    ] == ['source-humble', 'source-jazzy']
    assert report['decision']['actions'][0].endswith(
        'source-humble, source-jazzy'
    )


def test_checked_in_index_reports_all_recorded_rows_without_inference():
    """No-argument evidence exposes source PASS rows and their blockers."""
    records = MATRIX.load_evidence_index()

    report = MATRIX.evaluate_matrix(records)

    assert len(records) == 4
    assert report['decision']['status'] == 'BLOCKED'
    assert report['summary']['present_rows'] == 4
    assert report['summary']['pass_rows'] == 4
    assert report['summary']['comparable_rows'] == 0
    assert report['summary']['product_version_aligned'] is False
    assert report['rows'][2]['outcome_status'] == 'PASS'
    assert report['rows'][3]['outcome_status'] == 'PASS'
    assert records[2]['measurements']['command_count'] is None
    assert records[3]['measurements']['command_count'] is None


def test_evidence_index_applies_sha_bound_measurement_supplement(tmp_path):
    """A reviewed supplement fills nulls without changing the base record."""
    evidence_dir = tmp_path / 'docs' / 'evidence' / 'onboarding'
    evidence_dir.mkdir(parents=True)
    records = _full_matrix()
    record_paths = []
    receipt_paths = []
    for record in records:
        filename = f"{record['trial_id']}.json"
        path = evidence_dir / filename
        raw = (
            json.dumps(record, indent=2, sort_keys=True).encode('utf-8')
            + b'\n'
        )
        path.write_bytes(raw)
        record_paths.append(f'docs/evidence/onboarding/{filename}')
        receipt_filename = f"{record['trial_id']}.receipt.json"
        (evidence_dir / receipt_filename).write_bytes(
            record.validation_receipt_bytes
        )
        receipt_paths.append(
            f'docs/evidence/onboarding/{receipt_filename}'
        )

    source_humble = records[2]
    source_humble['measurements']['active_operator_time_sec'] = None
    source_humble['measurements']['command_count'] = None
    raw = (
        json.dumps(source_humble, indent=2, sort_keys=True).encode('utf-8')
        + b'\n'
    )
    (evidence_dir / f"{source_humble['trial_id']}.json").write_bytes(raw)
    supplement = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/onboarding-measurement-supplement-v1.schema.json'
        ),
        'supplement_id': 'source-humble-measurements-20260814',
        'captured_at': '2026-08-14T00:00:00Z',
        'trial_id': source_humble['trial_id'],
        'base_record_sha256': hashlib.sha256(raw).hexdigest(),
        'measurements': {
            'input_download_bytes': None,
            'workflow_download_bytes': None,
            'wall_time_sec': None,
            'active_operator_time_sec': 45.0,
            'command_count': 9,
            'peak_disk_bytes': None,
            'output_bytes': None,
        },
        'measurement_sources': {
            'input_download_bytes': 'not-supplemented',
            'workflow_download_bytes': 'not-supplemented',
            'wall_time_sec': 'not-supplemented',
            'active_operator_time_sec': 'operator-observation',
            'command_count': 'operator-observation',
            'peak_disk_bytes': 'not-supplemented',
            'output_bytes': 'not-supplemented',
        },
        'privacy': {
            'contains_private_paths': False,
            'contains_exact_command': False,
            'contains_operator_identity': False,
            'review_before_sharing': True,
        },
    }
    supplement_path = evidence_dir / 'source-humble.measurements.json'
    supplement_path.write_text(
        json.dumps(supplement, indent=2, sort_keys=True), encoding='utf-8'
    )

    index = _evidence_index()
    for row, record_path, receipt_path in zip(
        index['rows'],
        record_paths,
        receipt_paths,
    ):
        row['record_path'] = record_path
        row['validation_receipt_path'] = receipt_path
    index['rows'][2]['measurement_supplement_path'] = (
        'docs/evidence/onboarding/source-humble.measurements.json'
    )
    index_path = tmp_path / 'index.json'
    index_path.write_text(json.dumps(index), encoding='utf-8')

    loaded = MATRIX.load_evidence_index(index_path, repo_root=tmp_path)
    report = MATRIX.evaluate_matrix(loaded)

    assert report['decision']['status'] == 'ALL_ROWS_COMPARABLE'
    assert loaded[2]['measurements']['active_operator_time_sec'] == 45.0
    assert loaded[2]['measurements']['command_count'] == 9


def test_no_argument_cli_uses_checked_in_index(capsys):
    """The shortest audit reports all evidence and remains fail-closed."""
    assert MATRIX.main(['--json']) == 0

    report = json.loads(capsys.readouterr().out)
    assert report['summary']['present_rows'] == 4
    assert report['summary']['pass_rows'] == 4
    assert report['decision']['status'] == 'BLOCKED'
    assert report['summary']['product_version_aligned'] is False


def test_evidence_index_order_and_paths_fail_closed(tmp_path):
    """The index cannot shuffle rows or point outside reviewed evidence."""
    reordered = copy.deepcopy(_evidence_index())
    reordered['rows'][0], reordered['rows'][1] = (
        reordered['rows'][1], reordered['rows'][0]
    )
    reordered_path = tmp_path / 'reordered.json'
    reordered_path.write_text(json.dumps(reordered), encoding='utf-8')
    with pytest.raises(MATRIX.MatrixError, match='fixed matrix order'):
        MATRIX.load_evidence_index(reordered_path)

    unsafe = copy.deepcopy(_evidence_index())
    unsafe['rows'][0]['record_path'] = '../private.json'
    unsafe_path = tmp_path / 'unsafe.json'
    unsafe_path.write_text(json.dumps(unsafe), encoding='utf-8')
    with pytest.raises(MATRIX.MatrixError, match='evidence index schema'):
        MATRIX.load_evidence_index(unsafe_path)


def test_evidence_index_rejects_missing_or_mislabeled_record(tmp_path):
    """A path must exist and prove the row named by its index entry."""
    missing = copy.deepcopy(_evidence_index())
    missing['rows'][0]['record_path'] = (
        'docs/evidence/onboarding/not-present.json'
    )
    missing_path = tmp_path / 'missing.json'
    missing_path.write_text(json.dumps(missing), encoding='utf-8')
    with pytest.raises(MATRIX.MatrixError, match='not a regular file'):
        MATRIX.load_evidence_index(missing_path)

    mislabeled = copy.deepcopy(_evidence_index())
    mislabeled['rows'][0]['record_path'] = (
        'docs/evidence/onboarding/'
        'g0-docker-jazzy-20260810-machine-a.json'
    )
    mislabeled_path = tmp_path / 'mislabeled.json'
    mislabeled_path.write_text(json.dumps(mislabeled), encoding='utf-8')
    with pytest.raises(MATRIX.MatrixError, match='different matrix row'):
        MATRIX.load_evidence_index(mislabeled_path)


def test_one_comparable_row_per_route_passes_only_activation_gate():
    """One passing row per route is activation evidence, not full parity."""
    records = _full_matrix()
    _make_failed(records[1], 'jazzy-image-unavailable')
    _make_failed(records[3], 'source-build-failed')

    report = MATRIX.evaluate_matrix(records)

    assert report['decision']['status'] == 'ACTIVATION_GATE_PASS'
    assert report['summary']['activation_gate'] is True
    assert report['summary']['all_rows_comparable'] is False
    assert report['summary']['comparable_rows'] == 2


def test_complete_matrix_without_source_pass_is_blocked():
    """Docker success alone cannot open the documented activation gate."""
    records = _full_matrix()
    _make_failed(records[2], 'source-candidate-not-published')
    _make_failed(records[3], 'source-candidate-not-published')

    report = MATRIX.evaluate_matrix(records)

    assert report['decision']['status'] == 'BLOCKED'
    assert report['summary']['activation_gate'] is False
    assert report['decision']['actions'][-1].startswith(
        'Require at least one comparable Docker PASS'
    )


def test_duplicate_or_mislabeled_rows_fail_closed():
    """Duplicate row claims and wrong OS pairings are invalid evidence."""
    duplicate = _full_matrix()
    duplicate[1] = _trial('docker', 'humble')
    duplicate[1]['trial_id'] = 'different-docker-humble'
    with pytest.raises(MATRIX.MatrixError, match='duplicate matrix row'):
        MATRIX.evaluate_matrix(duplicate)

    wrong_os = _full_matrix()
    wrong_os[0]['environment']['os_family'] = 'ubuntu-24.04'
    with pytest.raises(MATRIX.MatrixError, match='ubuntu-22.04'):
        MATRIX.evaluate_matrix(wrong_os)


def test_mixed_product_or_source_identity_fails_closed():
    """Mixed product versions stay visible but cannot pass comparison gates."""
    mixed_version = _full_matrix()
    mixed_version[3]['environment']['product_version'] = '0.9.1'
    _bind(mixed_version[3])
    report = MATRIX.evaluate_matrix(mixed_version)
    assert report['decision']['status'] == 'BLOCKED'
    assert report['summary']['product_version_aligned'] is False
    assert report['summary']['activation_gate'] is False
    assert report['summary']['all_rows_comparable'] is False
    assert any(
        action.startswith('Align all matrix rows')
        for action in report['decision']['actions']
    )

    mixed_source = _full_matrix()
    mixed_source[3]['environment']['revision']['value'] = 'f' * 40
    _bind(mixed_source[3])
    with pytest.raises(MATRIX.MatrixError, match='source rows disagree'):
        MATRIX.evaluate_matrix(mixed_source)


def test_pass_cannot_silently_use_the_smaller_fixture():
    """The full G0 matrix cannot be passed with the bounded fixture."""
    records = _full_matrix()
    records[0]['input']['download_bytes'] = 98873952

    with pytest.raises(MATRIX.MatrixError, match='full fixed dataset'):
        MATRIX.evaluate_matrix(records)


def test_cli_distinguishes_unmet_gate_and_invalid_input(tmp_path, capsys):
    """The CLI separates valid incompleteness from malformed records."""
    paths = []
    for record in _full_matrix()[:2]:
        path = tmp_path / f'{record["trial_id"]}.json'
        path.write_text(json.dumps(record), encoding='utf-8')
        paths.append(str(path))

    assert MATRIX.main([*paths, '--require-activation-gate']) == 1
    assert 'Matrix status: **INCOMPLETE**' in capsys.readouterr().out

    invalid = tmp_path / 'invalid.json'
    invalid.write_text('[]', encoding='utf-8')
    assert MATRIX.main([str(invalid)]) == 2
    assert 'root is not an object' in capsys.readouterr().err
