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

"""Tests for fail-closed external first-map evidence intake."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = REPO_ROOT / 'scripts'
MODULE_PATH = SCRIPTS / 'prepare_external_first_map_acceptance.py'
LEDGER_PATH = (
    REPO_ROOT / 'docs/evidence/external-first-map-validations.json'
)
LEDGER_SCHEMA_PATH = (
    REPO_ROOT
    / 'docs/schemas/external-first-map-validations-v1.schema.json'
)
RECEIPT_SCHEMA_PATH = (
    REPO_ROOT / 'docs/schemas/first-map-validation-receipt-v1.schema.json'
)
REPORT_SCHEMA_PATH = (
    REPO_ROOT / 'docs/schemas/external-first-map-acceptance-v1.schema.json'
)
MANIFEST_SHA = '1' * 64


def _load_module():
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(
            'prepare_external_first_map_acceptance',
            MODULE_PATH,
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


def _load(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def _entry() -> dict:
    return {
        'id': 'validation-501',
        'reporter': '@external-user',
        'issue_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/issues/501'
        ),
        'submitted_at': '2026-08-01T12:00:00Z',
        'independent_attestation': True,
        'documentation_path': 'docker-first-map',
        'release_ref': 'v0.9.0',
        'environment': {
            'os': 'Ubuntu 24.04',
            'architecture': 'amd64',
            'ros_distro': 'container-managed',
            'install_method': 'ghcr',
        },
        'exact_command': (
            'docker run --rm ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0'
        ),
        'result': 'passed',
        'verification': {
            'manifest_status': 'succeeded',
            'diagnosis_status': 'success',
            'autoware_status': 'PASS',
            'manifest_sha256': MANIFEST_SHA,
        },
        'findings': [],
        'acceptance': {
            'status': 'accepted',
            'reviewed_by': '@rsasaki0109',
            'reviewed_at': '2026-08-01T13:00:00Z',
            'review_url': (
                'https://github.com/rsasaki0109/lidar_slam_ros2/issues/'
                '501#issuecomment-9001'
            ),
            'findings_status': 'no-findings',
            'resolution_urls': [],
        },
    }


def _receipt() -> dict:
    checks = [
        'manifest_succeeded',
        'lifecycle_complete',
        'runner_exit_zero',
        'diagnosis_success',
        'autoware_verification_pass',
        'diagnosis_bound_to_manifest',
        'verify_log_bound_to_manifest',
    ]
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
            'first-map-validation-receipt-v1.schema.json'
        ),
        'status': 'PASS',
        'run': {
            'run_id': 'external-run-501',
            'product_version': '0.9.0',
            'git_commit': 'a' * 40,
            'profile_id': 'rko_lio_graph_mid360_preset',
        },
        'verification': copy.deepcopy(_entry()['verification']),
        'evidence': {
            'manifest': {
                'filename': 'run_manifest.json',
                'sha256': MANIFEST_SHA,
            },
            'diagnosis': {
                'filename': 'autoware_map_diagnosis.json',
                'available': True,
                'sha256': '2' * 64,
            },
            'verify_log': {
                'filename': 'verify_autoware_map.log',
                'available': True,
                'sha256': '3' * 64,
            },
        },
        'checks': [
            {'id': check_id, 'passed': True, 'observed': 'matched'}
            for check_id in checks
        ],
        'shareability': {
            'contains_map_geometry': False,
            'contains_private_paths': False,
            'contains_exact_command': False,
            'review_before_sharing': True,
        },
    }


def _prepare(module, entry=None, receipt=None, ledger=None):
    return module.prepare_acceptance(
        ledger or _load(LEDGER_PATH),
        entry or _entry(),
        receipt or _receipt(),
        _load(LEDGER_SCHEMA_PATH),
        _load(RECEIPT_SCHEMA_PATH),
        _load(REPORT_SCHEMA_PATH),
    )


def _write(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload), encoding='utf-8')


def test_valid_receipt_prepares_one_entry_without_mutating_ledger():
    """Valid evidence should produce a separate one-entry proposal."""
    module = _load_module()
    ledger = _load(LEDGER_PATH)
    original = copy.deepcopy(ledger)

    report, proposal = _prepare(module, ledger=ledger)

    assert ledger == original
    assert report['status'] == 'READY_TO_PROPOSE'
    assert report['current_accepted_validations'] == 0
    assert report['proposed_accepted_validations'] == 1
    assert report['remaining_validations'] == 2
    assert proposal['validations'] == [_entry()]


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (
            lambda entry, receipt: receipt.update(status='FAIL'),
            'receipt status must be PASS',
        ),
        (
            lambda entry, receipt: receipt['verification'].update(
                manifest_sha256='4' * 64
            ),
            'exactly match',
        ),
        (
            lambda entry, receipt: receipt['checks'][0].update(passed=False),
            'failed required checks',
        ),
        (
            lambda entry, receipt: entry['acceptance'].update(
                reviewed_by='@external-user'
            ),
            'known maintainer',
        ),
        (
            lambda entry, receipt: entry.update(reporter='@rsasaki0109'),
            'known maintainer',
        ),
        (
            lambda entry, receipt: entry['acceptance'].update(
                reviewed_at='2026-07-31T13:00:00Z'
            ),
            'cannot precede',
        ),
        (
            lambda entry, receipt: entry['acceptance'].update(
                review_url='https://example.com/review/501'
            ),
            'public repository',
        ),
        (
            lambda entry, receipt: entry['acceptance'].update(
                review_url=(
                    'https://github.com/rsasaki0109/lidar_slam_ros2/issues/'
                    '502#issuecomment-9001'
                )
            ),
            'submitted issue',
        ),
    ],
)
def test_acceptance_blocks_unbound_or_unreviewed_evidence(
    mutation,
    message,
):
    """Any missing evidence binding or review control must fail closed."""
    module = _load_module()
    entry = _entry()
    receipt = _receipt()
    mutation(entry, receipt)

    with pytest.raises(module.AcceptanceError, match=message):
        _prepare(module, entry=entry, receipt=receipt)


def test_duplicate_evidence_is_blocked_by_proposed_ledger_validation():
    """Existing ledger identities must not count twice."""
    module = _load_module()
    ledger = _load(LEDGER_PATH)
    ledger['validations'] = [_entry()]

    with pytest.raises(module.AcceptanceError, match='duplicate'):
        _prepare(module, ledger=ledger)


def test_cli_writes_only_a_new_proposal_file(tmp_path, capsys):
    """The CLI should create once and refuse an existing output file."""
    module = _load_module()
    entry_path = tmp_path / 'entry.json'
    receipt_path = tmp_path / 'receipt.json'
    output_path = tmp_path / 'proposal.json'
    _write(entry_path, _entry())
    _write(receipt_path, _receipt())
    args = [
        '--entry',
        str(entry_path),
        '--receipt',
        str(receipt_path),
        '--output-ledger',
        str(output_path),
    ]

    assert module.main(args) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['proposal_written_to'] == str(output_path)
    assert len(_load(output_path)['validations']) == 1
    assert module.main(args) == 2
    assert 'without overwriting' in capsys.readouterr().err


def test_cli_refuses_to_overwrite_authoritative_ledger(tmp_path, capsys):
    """The authoritative input ledger must never be an output target."""
    module = _load_module()
    entry_path = tmp_path / 'entry.json'
    receipt_path = tmp_path / 'receipt.json'
    ledger_path = tmp_path / 'ledger.json'
    _write(entry_path, _entry())
    _write(receipt_path, _receipt())
    _write(ledger_path, _load(LEDGER_PATH))

    result = module.main([
        '--entry',
        str(entry_path),
        '--receipt',
        str(receipt_path),
        '--ledger',
        str(ledger_path),
        '--output-ledger',
        str(ledger_path),
    ])

    assert result == 2
    assert _load(ledger_path)['validations'] == []
    assert 'authoritative input ledger' in capsys.readouterr().err
