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

"""Tests for the one-command exact candidate onboarding-row runner."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import pathlib
import subprocess
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / 'scripts'
SCRIPT = SCRIPT_DIR / 'run_candidate_trial.py'
sys.path.insert(0, str(SCRIPT_DIR))
SPEC = importlib.util.spec_from_file_location('run_candidate_trial', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)

SOURCE_COMMIT = 'a' * 40
HUMBLE_DIGEST = 'sha256:' + 'b' * 64
JAZZY_DIGEST = 'sha256:' + 'c' * 64
SET_SHA256 = 'd' * 64
BUNDLE_SHA256 = 'e' * 64
RUN_URL = (
    'https://github.com/rsasaki0109/lidar_slam_ros2/'
    'actions/runs/12345'
)


def _row(route: str, distro: str) -> dict[str, object]:
    os_family = 'ubuntu-22.04' if distro == 'humble' else 'ubuntu-24.04'
    if route == 'docker':
        digest = HUMBLE_DIGEST if distro == 'humble' else JAZZY_DIGEST
        identity = {
            'kind': 'image-digest',
            'value': digest,
            'tag': None,
            'immutable_ref': (
                'ghcr.io/rsasaki0109/lidar_slam_ros2@' + digest
            ),
        }
    else:
        identity = {
            'kind': 'git-commit',
            'value': SOURCE_COMMIT,
            'tag': None,
            'immutable_ref': None,
        }
    return {
        'row_id': f'{route}-{distro}',
        'route': route,
        'ros_distro': distro,
        'os_family': os_family,
        'product_version': '0.9.1',
        'identity': identity,
        'preflight_command': 'reviewed preflight',
        'observer_command': 'reviewed probe',
        'required_measurements': [],
    }


def _handoff(
    tmp_path: pathlib.Path,
) -> tuple[pathlib.Path, dict[str, object]]:
    directory = tmp_path / 'candidate-handoff'
    directory.mkdir()
    preparation = {
        'run_id': 12345,
        'source_pr': 427,
        'source_commit': SOURCE_COMMIT,
        'product_version': '0.9.1',
        'workflow_run_url': RUN_URL,
        'candidate_bundle_sha256': BUNDLE_SHA256,
        'candidate_set_sha256': SET_SHA256,
        'artifact_expires_at': '2026-09-14T00:00:00Z',
    }
    preparation_bytes = (
        json.dumps(preparation, sort_keys=True) + '\n'
    ).encode()
    (directory / 'preparation.json').write_bytes(preparation_bytes)
    handoff = {
        'directory': directory,
        'evidence_bundle': {'candidate_set': {'status': 'READY'}},
        'audit': {'status': 'REMOTE_AUDIT_PASS'},
        'packet': {
            'rows': [
                _row('docker', 'humble'),
                _row('docker', 'jazzy'),
                _row('source', 'humble'),
                _row('source', 'jazzy'),
            ],
        },
        'preparation': preparation,
        'metadata_bytes': {'preparation.json': preparation_bytes},
    }
    return directory, handoff


def _record(
    handoff: dict[str, object],
    row: dict[str, object],
    trial_id: str,
    *,
    outcome: str = 'PASS',
    human_complete: bool = False,
) -> dict[str, object]:
    preparation = handoff['preparation']
    identity = row['identity']
    passed = outcome == 'PASS'
    evidence: dict[str, object] = {
        'manifest_sha256': '1' * 64 if passed else None,
        'receipt_sha256': '2' * 64 if passed else None,
    }
    if row['route'] == 'docker':
        evidence['candidate_image_set'] = {
            'sha256': preparation['candidate_set_sha256'],
            'bundle_sha256': preparation['candidate_bundle_sha256'],
            'source_pr': preparation['source_pr'],
            'source_commit': preparation['source_commit'],
            'product_version': preparation['product_version'],
            'workflow_run_url': preparation['workflow_run_url'],
            'immutable_ref': identity['immutable_ref'],
        }
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/onboarding-trial-v1.schema.json'
        ),
        'trial_id': trial_id,
        'captured_at': '2026-08-15T12:01:00Z',
        'documentation_path': (
            'docker-first-map' if row['route'] == 'docker'
            else 'source-quickstart'
        ),
        'operator_class': 'maintainer',
        'environment': {
            'clean_start': True,
            'ros_distro': row['ros_distro'],
            'architecture': 'x86_64',
            'os_family': row['os_family'],
            'product_version': row['product_version'],
            'revision': {
                'kind': identity['kind'],
                'value': identity['value'],
            },
        },
        'input': {
            'dataset_class': 'fixed-public',
            'dataset_id': 'mid360-public-zenodo-14841855',
            'download_bytes': 100,
        },
        'measurements': {
            'workflow_download_bytes': 200,
            'wall_time_sec': 30.0,
            'active_operator_time_sec': 4.0 if human_complete else None,
            'command_count': 1 if human_complete else None,
            'peak_disk_bytes': 400,
            'output_bytes': 300,
        },
        'outcome': {
            'status': outcome,
            'runner_exit_code': 0 if passed else 7,
            'manifest_status': 'succeeded' if passed else 'missing',
            'diagnosis_status': 'success' if passed else 'missing',
            'verifier_status': 'PASS' if passed else 'NOT_RUN',
            'receipt_status': 'PASS' if passed else 'NOT_CREATED',
            'undocumented_manual_steps': 0,
            'failure_stage': 'none' if passed else 'mapping',
            'finding_codes': [] if passed else ['mapping-failed'],
        },
        'evidence': evidence,
        'privacy': {
            'contains_private_paths': False,
            'contains_exact_command': False,
            'contains_operator_identity': False,
            'review_before_sharing': True,
        },
    }


def _validation_receipt(record: dict[str, object]) -> bytes:
    candidate = record['evidence'].get('candidate_image_set')
    git_commit = (
        candidate['source_commit']
        if candidate is not None
        else record['environment']['revision']['value']
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
                'sha256': '3' * 64,
            },
            'verify_log': {
                'filename': 'verify_autoware_map.log',
                'available': True,
                'sha256': '4' * 64,
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


def _remote_audit(status: str = 'REMOTE_AUDIT_PASS') -> dict[str, object]:
    return {
        'status': status,
        'findings': (
            [] if status == 'REMOTE_AUDIT_PASS'
            else ['candidate-artifact-set-incomplete-or-expired']
        ),
    }


def _install_handoff(monkeypatch, handoff):
    monkeypatch.setattr(
        RUNNER,
        'load_candidate_trial_handoff',
        lambda _directory: handoff,
    )


def _probe_writer(
    handoff,
    row,
    calls,
    *,
    outcome='PASS',
    human_complete=False,
    returncode=None,
    retain_receipt=True,
):
    def run(command, **_kwargs):
        calls.append(command)
        trial_id = command[command.index('--trial-id') + 1]
        record_path = pathlib.Path(command[command.index('--record') + 1])
        record = _record(
            handoff,
            row,
            trial_id,
            outcome=outcome,
            human_complete=human_complete,
        )
        if outcome == 'PASS' and retain_receipt:
            receipt_path = pathlib.Path(
                command[
                    command.index('--validation-receipt-output') + 1
                ]
            )
            receipt_path.write_bytes(_validation_receipt(record))
        record_path.write_text(
            json.dumps(record) + '\n',
            encoding='utf-8',
        )
        code = (0 if outcome == 'PASS' else 1)
        return subprocess.CompletedProcess(
            command,
            code if returncode is None else returncode,
        )

    return run


def test_probe_stdout_is_kept_out_of_wrapper_json_stream():
    """Live probe output goes to stderr so --json stdout stays parseable."""
    observed = {}

    def probe(command, **kwargs):
        observed.update(kwargs)
        return subprocess.CompletedProcess(command, 0)

    result = RUNNER._run_probe(['probe'], runner=probe)

    assert result.returncode == 0
    assert observed == {'check': False, 'stdout': sys.stderr}


def test_docker_row_runs_from_structured_identity_and_keeps_unknown_human_data(
    monkeypatch,
    tmp_path,
):
    """Non-interactive auto mode never fabricates observer measurements."""
    directory, handoff = _handoff(tmp_path)
    _install_handoff(monkeypatch, handoff)
    monkeypatch.setattr(
        RUNNER,
        'audit_candidate_bundle',
        lambda *_args, **_kwargs: _remote_audit(),
    )
    row = handoff['packet']['rows'][1]
    calls = []
    output = tmp_path / 'trial-output'

    receipt, exit_code = RUNNER.run_candidate_trial(
        directory,
        'docker-jazzy',
        output,
        acknowledge_dedicated_trial_host=True,
        read_runner=lambda *_args, **_kwargs: None,
        probe_runner=_probe_writer(handoff, row, calls),
        interactive=False,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 0
    assert receipt['status'] == 'TRIAL_RECORDED'
    assert receipt['trial']['outcome_status'] == 'PASS'
    assert receipt['trial']['measurement_status'] == 'INCOMPLETE'
    assert receipt['trial']['comparable'] is False
    assert receipt['measurement_capture']['resolved_mode'] == 'unknown'
    assert 'measurements.active_operator_time_sec' in (
        receipt['trial']['missing_measurements']
    )
    assert output.is_dir()
    assert (output / 'trial-record.json').is_file()
    assert (output / 'trial-audit.json').is_file()
    assert (output / 'execution.json').is_file()
    assert len(calls) == 1
    command = calls[0]
    assert '--record-human-measurements-unknown' in command
    assert '--prompt-human-measurements' not in command
    assert command[command.index('--candidate-image-set-sha256') + 1] == (
        SET_SHA256
    )
    assert command[command.index('--candidate-image-ref') + 1] == (
        row['identity']['immutable_ref']
    )
    assert '--build-observer-image-if-missing' in command
    observer_image = command[command.index('--observer-image') + 1]
    observer_recipe = command[
        command.index('--observer-recipe-sha256') + 1
    ]
    assert observer_image == (
        f'lidarslam-onboarding-trial-host:24.04-{observer_recipe[:12]}'
    )
    assert len(observer_recipe) == 64
    assert not list(tmp_path.glob('.trial-output.running-*'))


def test_interactive_complete_docker_trial_is_comparable(
    monkeypatch,
    tmp_path,
):
    """Prompt mode can delegate human values into a comparable PASS."""
    directory, handoff = _handoff(tmp_path)
    _install_handoff(monkeypatch, handoff)
    monkeypatch.setattr(
        RUNNER,
        'audit_candidate_bundle',
        lambda *_args, **_kwargs: _remote_audit(),
    )
    row = handoff['packet']['rows'][0]
    calls = []

    receipt, exit_code = RUNNER.run_candidate_trial(
        directory,
        'docker-humble',
        tmp_path / 'trial-output',
        acknowledge_dedicated_trial_host=True,
        human_measurements='prompt',
        probe_runner=_probe_writer(
            handoff, row, calls, human_complete=True
        ),
        interactive=True,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 0
    assert receipt['trial']['measurement_status'] == 'COMPLETE'
    assert receipt['trial']['comparable'] is True
    assert '--prompt-human-measurements' in calls[0]
    assert (
        tmp_path / 'trial-output' / 'first-map-validation-receipt.json'
    ).is_file()


def test_complete_pass_without_retained_receipt_stays_non_comparable(
    monkeypatch,
    tmp_path,
):
    """A complete PASS cannot rely on a digest without retained bytes."""
    directory, handoff = _handoff(tmp_path)
    _install_handoff(monkeypatch, handoff)
    monkeypatch.setattr(
        RUNNER,
        'audit_candidate_bundle',
        lambda *_args, **_kwargs: _remote_audit(),
    )
    row = handoff['packet']['rows'][0]

    receipt, exit_code = RUNNER.run_candidate_trial(
        directory,
        'docker-humble',
        tmp_path / 'trial-output',
        acknowledge_dedicated_trial_host=True,
        probe_runner=_probe_writer(
            handoff,
            row,
            [],
            human_complete=True,
            retain_receipt=False,
        ),
        interactive=True,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 0
    assert receipt['trial']['comparable'] is False
    assert receipt['trial']['comparability_blockers'] == [
        'validation_receipt_unbound'
    ]
    assert receipt['outputs']['validation_receipt'] is None


def test_valid_product_failure_is_retained_as_trial_evidence(
    monkeypatch,
    tmp_path,
):
    """A route FAIL is evidence, not a wrapper failure or deleted staging."""
    directory, handoff = _handoff(tmp_path)
    _install_handoff(monkeypatch, handoff)
    monkeypatch.setattr(
        RUNNER,
        'audit_candidate_bundle',
        lambda *_args, **_kwargs: _remote_audit(),
    )
    row = handoff['packet']['rows'][0]
    output = tmp_path / 'trial-output'

    receipt, exit_code = RUNNER.run_candidate_trial(
        directory,
        'docker-humble',
        output,
        acknowledge_dedicated_trial_host=True,
        probe_runner=_probe_writer(
            handoff, row, [], outcome='FAIL', human_complete=True
        ),
        interactive=False,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 1
    assert receipt['status'] == 'TRIAL_RECORDED'
    assert receipt['trial']['outcome_status'] == 'FAIL'
    assert 'outcome_failed' in receipt['trial']['comparability_blockers']
    assert (output / 'trial-record.json').is_file()


def test_failed_remote_preflight_blocks_probe_and_publishes_reason(
    monkeypatch,
    tmp_path,
):
    """A stale candidate identity never reaches privileged trial execution."""
    directory, handoff = _handoff(tmp_path)
    _install_handoff(monkeypatch, handoff)
    monkeypatch.setattr(
        RUNNER,
        'audit_candidate_bundle',
        lambda *_args, **_kwargs: _remote_audit('REMOTE_AUDIT_FAIL'),
    )
    output = tmp_path / 'trial-output'

    receipt, exit_code = RUNNER.run_candidate_trial(
        directory,
        'docker-humble',
        output,
        acknowledge_dedicated_trial_host=True,
        probe_runner=lambda *_args, **_kwargs: pytest.fail('probe ran'),
        interactive=False,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 1
    assert receipt['status'] == 'PREFLIGHT_BLOCKED'
    assert receipt['trial']['attempted'] is False
    assert receipt['preflight']['status'] == 'REMOTE_AUDIT_FAIL'
    assert (output / 'row-preflight.json').is_file()
    assert not (output / 'private').exists()


def test_invalid_probe_record_is_quarantined_and_harness_error_is_retained(
    monkeypatch,
    tmp_path,
):
    """Malformed probe output cannot masquerade as bounded trial evidence."""
    directory, handoff = _handoff(tmp_path)
    _install_handoff(monkeypatch, handoff)
    monkeypatch.setattr(
        RUNNER,
        'audit_candidate_bundle',
        lambda *_args, **_kwargs: _remote_audit(),
    )
    output = tmp_path / 'trial-output'

    def bad_probe(command, **_kwargs):
        record = pathlib.Path(command[command.index('--record') + 1])
        record.write_text('{bad json\n', encoding='utf-8')
        return subprocess.CompletedProcess(command, 2)

    receipt, exit_code = RUNNER.run_candidate_trial(
        directory,
        'docker-humble',
        output,
        acknowledge_dedicated_trial_host=True,
        probe_runner=bad_probe,
        interactive=False,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 2
    assert receipt['status'] == 'HARNESS_ERROR'
    assert receipt['trial']['record_status'] == 'QUARANTINED'
    assert not (output / 'trial-record.json').exists()
    assert (output / 'private' / 'untrusted-trial-record.json').is_file()
    assert (output / 'execution.json').is_file()


def test_valid_record_with_wrong_probe_exit_is_retained_as_harness_error(
    monkeypatch,
    tmp_path,
):
    """A valid record cannot hide a contradictory delegated process result."""
    directory, handoff = _handoff(tmp_path)
    _install_handoff(monkeypatch, handoff)
    monkeypatch.setattr(
        RUNNER,
        'audit_candidate_bundle',
        lambda *_args, **_kwargs: _remote_audit(),
    )
    row = handoff['packet']['rows'][0]
    output = tmp_path / 'trial-output'

    receipt, exit_code = RUNNER.run_candidate_trial(
        directory,
        'docker-humble',
        output,
        acknowledge_dedicated_trial_host=True,
        probe_runner=_probe_writer(
            handoff, row, [], outcome='PASS', returncode=1
        ),
        interactive=False,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 2
    assert receipt['status'] == 'HARNESS_ERROR'
    assert receipt['trial']['record_status'] == 'AVAILABLE'
    assert receipt['trial']['finding_codes'] == [
        'probe-exit-contract-mismatch'
    ]
    assert (output / 'trial-record.json').is_file()
    assert (output / 'trial-audit.json').is_file()


def test_source_row_runs_ready_preflight_and_source_safety_contract(
    monkeypatch,
    tmp_path,
):
    """Source selection derives a source probe without candidate-image args."""
    directory, handoff = _handoff(tmp_path)
    _install_handoff(monkeypatch, handoff)
    row = handoff['packet']['rows'][3]
    read_calls = []

    def read_runner(command, **_kwargs):
        read_calls.append(command)
        report = {
            'schema_version': 1,
            'status': 'READY',
            'repository': (
                'https://github.com/rsasaki0109/lidar_slam_ros2.git'
            ),
            'source_commit': SOURCE_COMMIT,
            'product_version': '0.9.1',
            'network_requested': True,
            'writes_performed': False,
            'finding_codes': [],
            'details': {},
        }
        return subprocess.CompletedProcess(
            command, 0, json.dumps(report), ''
        )

    probe_calls = []
    output = tmp_path / 'trial-output'
    receipt, exit_code = RUNNER.run_candidate_trial(
        directory,
        'source-jazzy',
        output,
        acknowledge_dedicated_trial_host=True,
        human_measurements='unknown',
        network_interface='eth9',
        read_runner=read_runner,
        probe_runner=_probe_writer(handoff, row, probe_calls),
        interactive=False,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 0
    assert receipt['preflight']['status'] == 'READY'
    assert len(read_calls) == 1
    command = probe_calls[0]
    assert '--acknowledge-disposable-host' in command
    assert '--acknowledge-isolated-network' in command
    assert command[command.index('--network-interface') + 1] == 'eth9'
    assert '--candidate-image-ref' not in command
    assert (output / 'private' / 'trial').is_dir()
    assert (output / 'private' / 'observer').is_dir()


def test_source_not_ready_blocks_before_disposable_host_changes(
    monkeypatch,
    tmp_path,
):
    """A public source blocker is retained without starting host mutation."""
    directory, handoff = _handoff(tmp_path)
    _install_handoff(monkeypatch, handoff)

    def read_runner(command, **_kwargs):
        report = {
            'schema_version': 1,
            'status': 'NOT_READY',
            'repository': (
                'https://github.com/rsasaki0109/lidar_slam_ros2.git'
            ),
            'source_commit': SOURCE_COMMIT,
            'product_version': '0.9.1',
            'network_requested': True,
            'writes_performed': False,
            'finding_codes': ['source-candidate-not-published'],
            'detail': 'exact source commit is not publicly resolvable',
        }
        return subprocess.CompletedProcess(
            command, 1, json.dumps(report), ''
        )

    output = tmp_path / 'trial-output'
    receipt, exit_code = RUNNER.run_candidate_trial(
        directory,
        'source-humble',
        output,
        acknowledge_dedicated_trial_host=True,
        read_runner=read_runner,
        probe_runner=lambda *_args, **_kwargs: pytest.fail('probe ran'),
        interactive=False,
        started_at='2026-08-15T12:00:00Z',
    )

    assert exit_code == 1
    assert receipt['status'] == 'PREFLIGHT_BLOCKED'
    assert receipt['preflight']['status'] == 'NOT_READY'
    assert receipt['authority']['source_host_mutation_authorized'] is False
    assert not (output / 'private').exists()


def test_existing_output_and_noninteractive_prompt_fail_before_handoff_read(
    monkeypatch,
    tmp_path,
):
    """Unsafe destination reuse and hanging prompt modes fail locally."""
    directory, _handoff_value = _handoff(tmp_path)
    existing = tmp_path / 'existing'
    existing.mkdir()
    sentinel = existing / 'keep.txt'
    sentinel.write_text('keep\n', encoding='utf-8')
    monkeypatch.setattr(
        RUNNER,
        'load_candidate_trial_handoff',
        lambda _directory: pytest.fail('handoff was read'),
    )

    with pytest.raises(
        RUNNER.CandidateTrialExecutionError,
        match='refusing to overwrite',
    ):
        RUNNER.run_candidate_trial(
            directory,
            'docker-humble',
            existing,
            acknowledge_dedicated_trial_host=True,
            interactive=False,
        )
    assert sentinel.read_text(encoding='utf-8') == 'keep\n'

    with pytest.raises(
        RUNNER.CandidateTrialExecutionError,
        match='interactive terminal',
    ):
        RUNNER.run_candidate_trial(
            directory,
            'docker-humble',
            tmp_path / 'new-output',
            acknowledge_dedicated_trial_host=True,
            human_measurements='prompt',
            interactive=False,
        )


def test_dedicated_host_acknowledgement_is_required_before_handoff_read(
    monkeypatch,
    tmp_path,
):
    """One friendly flag preserves both route-specific safety boundaries."""
    directory, _handoff_value = _handoff(tmp_path)
    monkeypatch.setattr(
        RUNNER,
        'load_candidate_trial_handoff',
        lambda _directory: pytest.fail('handoff was read'),
    )

    with pytest.raises(
        RUNNER.CandidateTrialExecutionError,
        match='acknowledge-dedicated-trial-host',
    ):
        RUNNER.run_candidate_trial(
            directory,
            'docker-humble',
            tmp_path / 'trial-output',
            acknowledge_dedicated_trial_host=False,
        )
