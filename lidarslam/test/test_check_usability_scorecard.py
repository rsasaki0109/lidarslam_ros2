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

"""Tests for the neutral GLIM usability scorecard contract."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_usability_scorecard.py'
INDEX = (
    ROOT / 'docs' / 'contracts'
    / 'glim-usability-scorecard-evidence-v1.json'
)
SPEC = importlib.util.spec_from_file_location(
    'check_usability_scorecard_test',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
SCORECARD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCORECARD)


def _measurements() -> dict[str, int]:
    return {
        'wall_time_sec': 120,
        'active_operator_time_sec': 30,
        'command_count': 1,
        'workflow_download_bytes': 1000,
        'peak_disk_bytes': 2000,
        'failure_count': 0,
        'output_bytes': 3000,
    }


def _task(contract: dict, product_id: str) -> dict:
    task_id = contract['task_id']
    return {
        'task_id': task_id,
        'documentation_url': (
            f'https://example.test/{product_id}/{task_id}'
        ),
        'input_id': f'paired-{task_id}',
        'exact_commands': [f'{product_id} {task_id}'],
        'measurements': _measurements(),
        'checks': [
            {'id': check_id, 'passed': True}
            for check_id in contract['checks']
        ],
        'outcome': {
            'status': 'PASS',
            'undocumented_manual_steps': 0,
            'finding_codes': [],
        },
        'evidence': {
            'transcript_sha256': (
                'a' * 64 if product_id == 'lidarslam_ros2' else 'b' * 64
            ),
            'public_url': (
                f'https://example.test/evidence/{product_id}/{task_id}'
            ),
        },
    }


def _trial(
    product_id: str,
    *,
    operator_class: str = 'external',
    first_attempt: bool = True,
) -> dict:
    if product_id == 'lidarslam_ros2':
        version = '0.9.1'
        revision = {'kind': 'git-commit', 'value': 'c' * 40}
        fingerprint = '1' * 64
    else:
        version = '1.2.0'
        revision = {'kind': 'release-tag', 'value': 'v1.2.0'}
        fingerprint = '2' * 64
    return {
        'schema_version': 1,
        'schema_uri': SCORECARD.TRIAL_SCHEMA_URI,
        'trial_id': f'ux-{product_id}-20260812',
        'captured_at': '2026-08-12T06:00:00Z',
        'product': {
            'id': product_id,
            'version': version,
            'revision': revision,
            'documentation_root_url': f'https://example.test/{product_id}',
            'publicly_resolvable': True,
        },
        'operator': {
            'class': operator_class,
            'cohort_id': 'external-paired-operator-a',
            'first_attempt': first_attempt,
            'product_order': (
                'first' if product_id == 'lidarslam_ros2' else 'second'
            ),
        },
        'environment': {
            'comparison_pair_id': 'paired-jazzy-machine-class-a',
            'clean_start': True,
            'supported_by_product_docs': True,
            'ros_distro': 'jazzy',
            'os_family': 'ubuntu-24.04',
            'architecture': 'x86_64',
            'hardware_class': 'eight-core-32gib-x86_64',
            'machine_fingerprint_sha256': fingerprint,
        },
        'tasks': [
            _task(contract, product_id)
            for contract in SCORECARD.TASK_CONTRACTS
        ],
        'privacy': {
            'contains_private_paths': False,
            'contains_operator_identity': False,
            'contains_secrets': False,
            'review_before_sharing': True,
        },
    }


def _index() -> dict:
    return json.loads(INDEX.read_text(encoding='utf-8'))


def _preparation_archive(tmp_path: Path, records: list[dict]) -> Path:
    """Create exact untouched source worksheets and their valid receipt."""
    archive = tmp_path / 'preparation'
    archive.mkdir()
    prepared = []
    files = []
    for record in records:
        worksheet = copy.deepcopy(record)
        for task in worksheet['tasks']:
            task['exact_commands'] = []
            task['measurements'] = {
                key: None for key in task['measurements']
            }
            task['checks'] = [
                {'id': item['id'], 'passed': False}
                for item in task['checks']
            ]
            task['outcome'] = {
                'status': 'FAIL',
                'undocumented_manual_steps': 0,
                'finding_codes': ['not-recorded'],
            }
            task['evidence'] = {
                'transcript_sha256': None,
                'public_url': None,
            }
        prepared.append(worksheet)
        filename = f'{worksheet["trial_id"]}.json'
        payload = (
            json.dumps(worksheet, indent=2, sort_keys=True) + '\n'
        ).encode()
        (archive / filename).write_bytes(payload)
        files.append({
            'filename': filename,
            'product': worksheet['product']['id'],
            'product_order': worksheet['operator']['product_order'],
            'trial_id': worksheet['trial_id'],
            'sha256': hashlib.sha256(payload).hexdigest(),
        })
    receipt = {
        'schema_version': 1,
        'schema_uri': SCORECARD.PREPARATION_SCHEMA_URI,
        'receipt_filename': SCORECARD.PREPARATION_RECEIPT_NAME,
        'status': 'PREPARED_INCOMPLETE',
        'comparison_pair_id': (
            prepared[0]['environment']['comparison_pair_id']
        ),
        'public_identity_check': {
            'performed': True,
            'status': 'PASS',
            'network_reads_performed': True,
            'results': [
                {
                    'product_id': worksheet['product']['id'],
                    'identity_source': SCORECARD.IDENTITY_SOURCES[
                        (worksheet['product']['id'], 'git')
                    ],
                    'revision_kind': worksheet['product']['revision']['kind'],
                    'requested_revision': (
                        worksheet['product']['revision']['value']
                    ),
                    'resolved_revision': (
                        worksheet['product']['revision']['value']
                        if worksheet['product']['revision']['kind']
                        == 'git-commit'
                        else 'd' * 40
                    ),
                    'documentation_url': (
                        worksheet['product']['documentation_root_url']
                    ),
                    'documentation_http_status': 200,
                    'documentation_final_url': (
                        worksheet['product']['documentation_root_url']
                    ),
                }
                for worksheet in prepared
            ],
        },
        'files': files,
        'authority': {
            'github_requests': 'GET_ONLY',
            'registry_requests': 'NONE',
            'documentation_requests': 'GET_ONLY',
            'github_writes_authorized': False,
            'local_worksheets_written': True,
            'local_preparation_receipt_written': True,
            'rollback_on_publication_error': True,
            'remote_mutations_performed': False,
        },
    }
    receipt_path = archive / SCORECARD.PREPARATION_RECEIPT_NAME
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    return receipt_path


def test_checked_in_index_reports_honest_not_ready_state(capsys):
    """No-argument audit must expose both absent product records."""
    records = SCORECARD.load_evidence_index()

    report = SCORECARD.evaluate_scorecard(records)

    assert records == []
    assert report['status'] == 'NOT_READY'
    assert report['summary']['records_present'] == 0
    assert report['summary']['comparable_tasks'] == 0
    assert report['comparison_policy']['overall_winner_inferred'] is False
    assert report['comparison_policy'][
        'preparation_receipt_required_for_published_pair'
    ] is True
    assert SCORECARD.main(['--json']) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload['status'] == 'NOT_READY'


def test_complete_external_first_attempt_pair_is_ready_without_winner():
    """Complete paired evidence is ready without an overall winner."""
    report = SCORECARD.evaluate_scorecard([
        _trial('lidarslam_ros2'),
        _trial('glim'),
    ])

    assert report['status'] == 'READY'
    assert report['summary']['records_present'] == 2
    assert report['summary']['comparable_tasks'] == 6
    assert report['summary']['public_scorecard_ready'] is True
    assert all(task['comparable'] for task in report['tasks'])
    assert 'winner' not in report
    assert 'not inferred' in SCORECARD.render_report(report)


def test_missing_required_metric_makes_only_that_task_partial():
    """One missing task metric must not invalidate measured peer tasks."""
    ours = _trial('lidarslam_ros2')
    rival = _trial('glim')
    ours['tasks'][1]['measurements']['peak_disk_bytes'] = None

    report = SCORECARD.evaluate_scorecard([ours, rival])

    assert report['status'] == 'PARTIAL'
    assert report['summary']['comparable_tasks'] == 5
    row = report['tasks'][1]
    assert row['task_id'] == 'run-fixed-demo'
    assert row['comparable'] is False
    assert any(
        'peak_disk_bytes' in item
        for item in row['comparability_blockers']
    )


def test_maintainer_pair_cannot_become_public_scorecard_ready():
    """Maintainer evidence can compare tasks, not support launch claims."""
    report = SCORECARD.evaluate_scorecard([
        _trial('lidarslam_ros2', operator_class='maintainer'),
        _trial('glim', operator_class='maintainer'),
    ])

    assert report['summary']['comparable_tasks'] == 6
    assert report['summary']['external_first_attempt_pair'] is False
    assert report['summary']['public_scorecard_ready'] is False
    assert report['status'] == 'PARTIAL'
    assert report['decision']['actions'][-1].startswith(
        'Repeat the complete paired scorecard'
    )


def test_environment_and_input_mismatch_are_explicit_blockers():
    """Different hosts or inputs must remain visible comparison blockers."""
    ours = _trial('lidarslam_ros2')
    rival = _trial('glim')
    rival['environment']['hardware_class'] = 'different-machine-class'
    rival['tasks'][2]['input_id'] = 'different-bag'

    report = SCORECARD.evaluate_scorecard([ours, rival])

    assert report['status'] == 'NOT_READY'
    assert report['summary']['comparable_tasks'] == 0
    assert all(
        'environment-hardware_class-mismatch' in row['comparability_blockers']
        for row in report['tasks']
    )
    assert 'input-id-mismatch' in report['tasks'][2]['comparability_blockers']


def test_product_order_is_recorded_for_the_paired_operator():
    """A paired operator must expose which product was attempted first."""
    ours = _trial('lidarslam_ros2')
    rival = _trial('glim')
    rival['operator']['product_order'] = 'first'

    report = SCORECARD.evaluate_scorecard([ours, rival])

    assert report['status'] == 'NOT_READY'
    assert all(
        'operator-product-order-invalid' in row['comparability_blockers']
        for row in report['tasks']
    )


def test_identity_transcript_and_manual_steps_fail_closed():
    """Private identities, missing transcripts, and hidden steps block rows."""
    ours = _trial('lidarslam_ros2')
    rival = _trial('glim')
    ours['product']['publicly_resolvable'] = False
    ours['tasks'][0]['evidence']['transcript_sha256'] = None
    rival['tasks'][0]['outcome']['undocumented_manual_steps'] = 1

    report = SCORECARD.evaluate_scorecard([ours, rival])

    first = report['tasks'][0]['comparability_blockers']
    assert 'lidarslam_ros2-identity-not-public' in first
    assert 'lidarslam_ros2-transcript-missing' in first
    assert 'glim-undocumented-manual-steps' in first


def test_task_order_checks_and_outcomes_are_fixed():
    """Task criteria cannot be reordered or contradicted by PASS."""
    reordered = _trial('glim')
    reordered['tasks'][0], reordered['tasks'][1] = (
        reordered['tasks'][1], reordered['tasks'][0]
    )
    with pytest.raises(
        SCORECARD.ScorecardError,
        match='fixed scorecard order',
    ):
        SCORECARD.validate_trial(reordered)

    wrong_checks = _trial('glim')
    wrong_checks['tasks'][0]['checks'][0]['id'] = 'invented-check'
    with pytest.raises(SCORECARD.ScorecardError, match='checks must be'):
        SCORECARD.validate_trial(wrong_checks)

    false_pass = _trial('glim')
    false_pass['tasks'][0]['checks'][0]['passed'] = False
    with pytest.raises(SCORECARD.ScorecardError, match='PASS has a failed'):
        SCORECARD.validate_trial(false_pass)


def test_commands_preserve_count_and_reject_multiline_private_paths():
    """Published command sequences preserve retries and remain safe."""
    repeated = _trial('glim')
    repeated['tasks'][0]['exact_commands'] = ['glim --help', 'glim --help']
    repeated['tasks'][0]['measurements']['command_count'] = 2
    SCORECARD.validate_trial(repeated)

    zero_commands = _trial('glim')
    zero_commands['tasks'][0]['exact_commands'] = []
    zero_commands['tasks'][0]['measurements']['command_count'] = 0
    SCORECARD.validate_trial(zero_commands)

    mismatched = _trial('glim')
    mismatched['tasks'][0]['exact_commands'] = ['glim --help', 'glim run']
    with pytest.raises(SCORECARD.ScorecardError, match='command_count'):
        SCORECARD.validate_trial(mismatched)

    multiline = _trial('glim')
    multiline['tasks'][0]['exact_commands'] = ['glim --help\nrm example']
    with pytest.raises(SCORECARD.ScorecardError, match='one line'):
        SCORECARD.validate_trial(multiline)

    private = _trial('glim')
    private['tasks'][0]['exact_commands'] = ['glim /home/alice/private-bag']
    with pytest.raises(SCORECARD.ScorecardError, match='private path'):
        SCORECARD.validate_trial(private)


def test_time_and_schema_invariants_are_enforced():
    """Impossible timers and non-exact revisions are invalid evidence."""
    impossible_time = _trial('glim')
    impossible_time['tasks'][0]['measurements']['wall_time_sec'] = 10
    impossible_time['tasks'][0]['measurements'][
        'active_operator_time_sec'
    ] = 11
    with pytest.raises(SCORECARD.ScorecardError, match='exceeds wall time'):
        SCORECARD.validate_trial(impossible_time)

    non_finite = _trial('glim')
    non_finite['tasks'][0]['measurements']['wall_time_sec'] = float('inf')
    with pytest.raises(SCORECARD.ScorecardError, match='must be finite'):
        SCORECARD.validate_trial(non_finite)

    unsupported = _trial('glim')
    unsupported['environment']['supported_by_product_docs'] = False
    with pytest.raises(SCORECARD.ScorecardError, match='schema failed'):
        SCORECARD.validate_trial(unsupported)

    invalid_revision = _trial('glim')
    invalid_revision['product']['revision']['value'] = 'latest'
    with pytest.raises(SCORECARD.ScorecardError, match='trial schema failed'):
        SCORECARD.validate_trial(invalid_revision)


def test_index_order_path_and_product_binding_fail_closed(tmp_path):
    """The evidence index cannot escape the repo or relabel one product."""
    reordered = _index()
    reordered['rows'].reverse()
    reordered_path = tmp_path / 'reordered.json'
    reordered_path.write_text(json.dumps(reordered), encoding='utf-8')
    with pytest.raises(SCORECARD.ScorecardError, match='fixed order'):
        SCORECARD.load_evidence_index(reordered_path, tmp_path)

    unsafe = _index()
    unsafe['rows'][0]['record_path'] = '../private.json'
    unsafe_path = tmp_path / 'unsafe.json'
    unsafe_path.write_text(json.dumps(unsafe), encoding='utf-8')
    with pytest.raises(SCORECARD.ScorecardError, match='schema failed'):
        SCORECARD.load_evidence_index(unsafe_path, tmp_path)

    evidence = tmp_path / 'docs' / 'evidence' / 'usability'
    evidence.mkdir(parents=True)
    record_path = evidence / 'glim.json'
    record_path.write_text(json.dumps(_trial('glim')), encoding='utf-8')
    mislabeled = _index()
    mislabeled['rows'][0]['record_path'] = (
        'docs/evidence/usability/glim.json'
    )
    mislabeled['preparation_receipt_path'] = (
        'docs/evidence/usability/pair/preparation/'
        + SCORECARD.PREPARATION_RECEIPT_NAME
    )
    mislabeled_path = tmp_path / 'mislabeled.json'
    mislabeled_path.write_text(json.dumps(mislabeled), encoding='utf-8')
    with pytest.raises(SCORECARD.ScorecardError, match='another product'):
        SCORECARD.load_evidence_index(mislabeled_path, tmp_path)


def test_explicit_record_cli_reports_ready(tmp_path, capsys):
    """Explicit paired records use the same evaluator as the index route."""
    ours = tmp_path / 'ours.json'
    rival = tmp_path / 'rival.json'
    records = [_trial('lidarslam_ros2'), _trial('glim')]
    ours.write_text(json.dumps(records[0]), encoding='utf-8')
    rival.write_text(json.dumps(records[1]), encoding='utf-8')
    receipt = _preparation_archive(tmp_path, records)

    assert SCORECARD.main([
        '--record', str(ours),
        '--record', str(rival),
        '--preparation-receipt', str(receipt),
        '--json',
    ]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['status'] == 'READY'
    assert report['summary']['comparable_tasks'] == 6
    assert report['preparation_binding']['status'] == 'VALID'


def test_explicit_records_without_preparation_receipt_fail_closed(
    tmp_path,
    capsys,
):
    """A pair of completed JSON records alone cannot produce CLI READY."""
    ours = tmp_path / 'ours.json'
    rival = tmp_path / 'rival.json'
    ours.write_text(json.dumps(_trial('lidarslam_ros2')), encoding='utf-8')
    rival.write_text(json.dumps(_trial('glim')), encoding='utf-8')

    assert SCORECARD.main([
        '--record', str(ours),
        '--record', str(rival),
        '--json',
    ]) == 2
    assert '--preparation-receipt' in capsys.readouterr().err


def test_completed_identity_drift_from_preparation_fails_closed(
    tmp_path,
    capsys,
):
    """A prepared public pair cannot authorize a relabeled completed pair."""
    records = [_trial('lidarslam_ros2'), _trial('glim')]
    receipt = _preparation_archive(tmp_path, records)
    records[0]['product']['version'] = '0.9.2'
    ours = tmp_path / 'ours.json'
    rival = tmp_path / 'rival.json'
    ours.write_text(json.dumps(records[0]), encoding='utf-8')
    rival.write_text(json.dumps(records[1]), encoding='utf-8')

    assert SCORECARD.main([
        '--record', str(ours),
        '--record', str(rival),
        '--preparation-receipt', str(receipt),
        '--json',
    ]) == 2
    assert 'differs from preparation' in capsys.readouterr().err


def test_preparation_archive_hash_tamper_fails_closed(tmp_path, capsys):
    """A changed archived source worksheet cannot retain a valid binding."""
    records = [_trial('lidarslam_ros2'), _trial('glim')]
    receipt = _preparation_archive(tmp_path, records)
    receipt_value = json.loads(receipt.read_text(encoding='utf-8'))
    prepared_path = receipt.parent / receipt_value['files'][0]['filename']
    prepared_path.write_bytes(prepared_path.read_bytes() + b' ')
    ours = tmp_path / 'ours.json'
    rival = tmp_path / 'rival.json'
    ours.write_text(json.dumps(records[0]), encoding='utf-8')
    rival.write_text(json.dumps(records[1]), encoding='utf-8')

    assert SCORECARD.main([
        '--record', str(ours),
        '--record', str(rival),
        '--preparation-receipt', str(receipt),
        '--json',
    ]) == 2
    assert 'hash differs' in capsys.readouterr().err


def test_index_records_require_preparation_receipt(tmp_path):
    """A reviewed index cannot add product rows without their source chain."""
    evidence = tmp_path / 'docs' / 'evidence' / 'usability'
    evidence.mkdir(parents=True)
    index = _index()
    for row, product_id in zip(index['rows'], SCORECARD.PRODUCT_IDS):
        filename = f'{product_id}.json'
        (evidence / filename).write_text(
            json.dumps(_trial(product_id)),
            encoding='utf-8',
        )
        row['record_path'] = f'docs/evidence/usability/{filename}'
    index_path = tmp_path / 'index-without-receipt.json'
    index_path.write_text(json.dumps(index), encoding='utf-8')

    with pytest.raises(
        SCORECARD.ScorecardError,
        match='preparation_receipt_path',
    ):
        SCORECARD.load_evidence_index(index_path, tmp_path)


def test_duplicate_product_and_trial_rows_are_rejected():
    """Every product and trial identity may occur only once."""
    first = _trial('glim')
    second = copy.deepcopy(first)
    second['trial_id'] = 'ux-glim-second'
    with pytest.raises(
        SCORECARD.ScorecardError,
        match='duplicate product row',
    ):
        SCORECARD.evaluate_scorecard([first, second])

    second = _trial('lidarslam_ros2')
    second['trial_id'] = first['trial_id']
    with pytest.raises(SCORECARD.ScorecardError, match='duplicate trial_id'):
        SCORECARD.evaluate_scorecard([first, second])
