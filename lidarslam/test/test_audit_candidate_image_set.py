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

"""Tests for the read-only immutable candidate-image evidence audit."""

from __future__ import annotations

import importlib.util
import json
import pathlib
import subprocess
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'audit_candidate_image_set.py'


def _module():
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        spec = importlib.util.spec_from_file_location(
            'audit_candidate_image_set', SCRIPT
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPT.parent))


def _request(module):
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/candidate-image-request-v1.schema.json'
        ),
        'status': 'AUTHORIZED',
        'publication_mode': 'digest_only',
        'repository': 'rsasaki0109/lidar_slam_ros2',
        'event_name': 'repository_dispatch',
        'event_action': 'e2-publish-candidate-image',
        'default_branch': 'develop',
        'workflow_branch_ref': 'refs/heads/develop',
        'workflow_gate_commit': 'f' * 40,
        'source_pr': 427,
        'source_commit': 'a' * 40,
        'product_version': '0.9.1',
        'requested_by': 'maintainer',
        'actor_role': 'maintain',
        'required_success_checks': sorted(module.REQUIRED_SUCCESS_CHECKS),
        'observed_successful_checks': sorted(
            module.REQUIRED_SUCCESS_CHECKS
        ),
        'observed_skipped_checks': sorted(module.ALLOWED_SKIPPED_CHECKS),
        'environment': {
            'name': 'candidate-images',
            'required_reviewer_count': 1,
            'prevent_self_review': True,
            'deployment_branch_policy': 'develop_only',
        },
        'authority': {
            'package_write_authorized_for_digest_job': True,
            'tag_creation_authorized': False,
            'moving_tag_mutation_authorized': False,
            'release_mutation_authorized': False,
        },
    }


def _documents(module):
    request = _request(module)
    records = {}
    for distro, digest_character in (('humble', 'b'), ('jazzy', 'c')):
        records[distro] = module.build_candidate_image_record(
            request,
            ros_distro=distro,
            platform='linux/amd64',
            digest='sha256:' + digest_character * 64,
            cli_version='lidarslam_ros2 0.9.1',
            workflow_run_url=(
                'https://github.com/rsasaki0109/lidar_slam_ros2/'
                'actions/runs/12345'
            ),
            evidence_retention_days=30,
        )
    candidate_set = module.verify_candidate_image_set([
        records['humble'], records['jazzy'],
    ])
    return request, records, candidate_set


def _candidate_set(module):
    return _documents(module)[2]


def _write_bundle(module, directory):
    directory.mkdir()
    request, records, candidate_set = _documents(module)
    documents = {
        'candidate-image-request.json': request,
        'candidate-image-humble.json': records['humble'],
        'candidate-image-jazzy.json': records['jazzy'],
        'candidate-image-set.json': candidate_set,
    }
    for filename, document in documents.items():
        (directory / filename).write_text(
            json.dumps(document, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
    return module.load_candidate_evidence_bundle(directory)


def test_local_set_only_audit_remains_offline_and_has_no_write_authority():
    """Compatibility mode binds set bytes but cannot claim a full bundle."""
    module = _module()
    report = module.audit_candidate_set(
        _candidate_set(module),
        candidate_set_sha256='d' * 64,
    )

    assert report['status'] == 'LOCAL_CONTRACT_PASS'
    assert report['candidate_set_sha256'] == 'd' * 64
    assert report['candidate_bundle_sha256'] is None
    assert report['retained_evidence']['status'] == 'SET_ONLY_PASS'
    assert report['authority'] == {
        'network_reads_performed': False,
        'temporary_artifact_copies_used': False,
        'github_writes_authorized': False,
        'registry_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def test_four_file_bundle_rederives_records_set_and_canonical_hash(tmp_path):
    """One directory binds request, records, set, and all exact bytes."""
    module = _module()
    bundle = _write_bundle(module, tmp_path / 'candidate-evidence')
    report = module.audit_candidate_bundle(bundle)

    assert report['status'] == 'LOCAL_CONTRACT_PASS'
    assert report['retained_evidence']['status'] == 'FOUR_FILE_PASS'
    assert report['candidate_bundle_sha256'] == bundle['bundle_sha256']
    assert report['candidate_set_sha256'] == bundle['file_hashes'][
        'candidate-image-set.json'
    ]
    assert len(report['retained_evidence']['files']) == 4
    assert all(
        item['remote_content_status'] == 'NOT_CHECKED'
        for item in report['retained_evidence']['files']
    )


def test_bundle_rejects_cross_file_tampering_and_unexpected_entries(tmp_path):
    """A changed set or ambiguous directory layout fails before any reads."""
    module = _module()
    directory = tmp_path / 'candidate-evidence'
    _write_bundle(module, directory)
    set_path = directory / 'candidate-image-set.json'
    candidate_set = json.loads(set_path.read_text(encoding='utf-8'))
    candidate_set['requested_by'] = 'someone-else'
    set_path.write_text(json.dumps(candidate_set), encoding='utf-8')

    with pytest.raises(
        module.CandidateSetAuditError,
        match='does not derive exactly',
    ):
        module.load_candidate_evidence_bundle(directory)

    set_path.write_text(
        json.dumps(_candidate_set(module), sort_keys=True) + '\n',
        encoding='utf-8',
    )
    (directory / 'notes.txt').write_text('ambiguous', encoding='utf-8')
    with pytest.raises(
        module.CandidateSetAuditError,
        match='must contain exactly',
    ):
        module.load_candidate_evidence_bundle(directory)


def test_semantically_duplicate_distro_fails_even_when_shape_passes():
    """Schema-shaped duplicate distro entries cannot masquerade as a pair."""
    module = _module()
    candidate_set = _candidate_set(module)
    candidate_set['images'][1]['ros_distro'] = 'humble'

    with pytest.raises(
        module.CandidateSetAuditError,
        match='Humble and Jazzy exactly once',
    ):
        module.audit_candidate_set(
            candidate_set,
            candidate_set_sha256='d' * 64,
        )


def _remote_runner(
    module,
    bundle,
    *,
    expire_artifact=False,
    fail_attestation=False,
    mismatch_artifact=None,
    workflow_head_sha=None,
    extra_artifact=False,
):
    calls = []

    def runner(command, **_kwargs):
        calls.append(command)
        if command[:2] == ['gh', 'api'] and command[2].endswith(
            '/actions/runs/12345'
        ):
            payload = {
                'id': 12345,
                'html_url': (
                    'https://github.com/rsasaki0109/lidar_slam_ros2/'
                    'actions/runs/12345'
                ),
                'event': 'repository_dispatch',
                'status': 'completed',
                'conclusion': 'success',
                'head_branch': 'develop',
                'head_sha': (
                    workflow_head_sha
                    or bundle['candidate_set']['workflow_gate_commit']
                ),
                'path': '.github/workflows/candidate-image.yml',
            }
            return subprocess.CompletedProcess(
                command, 0, json.dumps(payload), ''
            )
        if command[:2] == ['gh', 'api'] and 'artifacts?' in command[2]:
            artifacts = [
                    {
                        'id': 1000 + index,
                        'name': name,
                        'expired': expire_artifact and index == 0,
                        'expires_at': '2026-09-14T00:00:00Z',
                        'workflow_run': {
                            'id': 12345,
                            'head_branch': 'develop',
                            'head_sha': (
                                workflow_head_sha
                                or bundle['candidate_set'][
                                    'workflow_gate_commit'
                                ]
                            ),
                        },
                    }
                    for index, name in enumerate(module.EXPECTED_ARTIFACTS)
                ]
            if extra_artifact:
                artifacts.append({
                    'id': 2000,
                    'name': 'unexpected-candidate-evidence',
                    'expired': False,
                    'expires_at': '2026-09-14T00:00:00Z',
                    'workflow_run': {
                        'id': 12345,
                        'head_branch': 'develop',
                        'head_sha': bundle['candidate_set'][
                            'workflow_gate_commit'
                        ],
                    },
                })
            payload = {
                'total_count': len(artifacts),
                'artifacts': artifacts,
            }
            return subprocess.CompletedProcess(
                command, 0, json.dumps(payload), ''
            )
        if command[:3] == ['gh', 'run', 'download']:
            artifact_name = command[command.index('--name') + 1]
            destination = pathlib.Path(command[command.index('--dir') + 1])
            filename = next(
                item[1] for item in module.EVIDENCE_FILES
                if item[0] == artifact_name
            )
            payload = bundle['paths'][filename].read_bytes()
            if artifact_name == mismatch_artifact:
                payload += b'\n'
            (destination / filename).write_bytes(payload)
            return subprocess.CompletedProcess(command, 0, '', '')
        if command[:4] == ['docker', 'buildx', 'imagetools', 'inspect']:
            digest = command[4].split('@', 1)[1]
            return subprocess.CompletedProcess(
                command, 0, json.dumps({'digest': digest}), ''
            )
        if command[:3] == ['gh', 'attestation', 'verify']:
            return subprocess.CompletedProcess(
                command,
                1 if fail_attestation else 0,
                '',
                'unverified' if fail_attestation else '',
            )
        raise AssertionError(f'unexpected command: {command}')

    return runner, calls


def test_remote_audit_byte_compares_artifacts_digests_and_attestations(
    tmp_path,
):
    """Remote PASS requires the exact four retained artifact payloads."""
    module = _module()
    bundle = _write_bundle(module, tmp_path / 'candidate-evidence')
    runner, calls = _remote_runner(module, bundle)

    report = module.audit_candidate_bundle(
        bundle,
        remote=True,
        runner=runner,
    )

    assert report['status'] == 'REMOTE_AUDIT_PASS'
    assert report['workflow']['status'] == 'PASS'
    assert report['artifacts'] == {
        'status': 'PASS',
        'required_names': list(module.EXPECTED_ARTIFACTS),
        'expires_at': '2026-09-14T00:00:00Z',
    }
    assert report['findings'] == []
    assert all(
        item['remote_content_status'] == 'PASS'
        for item in report['retained_evidence']['files']
    )
    assert sum(call[:3] == ['gh', 'run', 'download'] for call in calls) == 4
    download_directories = [
        pathlib.Path(call[call.index('--dir') + 1])
        for call in calls if call[:3] == ['gh', 'run', 'download']
    ]
    assert all(not path.exists() for path in download_directories)
    assert sum(
        call[:3] == ['gh', 'attestation', 'verify'] for call in calls
    ) == 2
    assert report['authority']['temporary_artifact_copies_used'] is True
    assert report['authority']['remote_mutations_performed'] is False


def test_remote_audit_fails_on_byte_mismatch_expiry_or_attestation(tmp_path):
    """Tampered, expired, or unattested evidence keeps trials blocked."""
    module = _module()
    bundle = _write_bundle(module, tmp_path / 'candidate-evidence')
    runner, _calls = _remote_runner(
        module,
        bundle,
        fail_attestation=True,
        mismatch_artifact='candidate-image-set',
    )
    mismatch_report = module.audit_candidate_bundle(
        bundle,
        remote=True,
        runner=runner,
    )
    assert mismatch_report['status'] == 'REMOTE_AUDIT_FAIL'
    assert 'candidate-image-set-content-mismatch' in (
        mismatch_report['findings']
    )
    assert 'humble-attestation-unverified' in mismatch_report['findings']

    wrong_gate_runner, _calls = _remote_runner(
        module,
        bundle,
        workflow_head_sha='e' * 40,
    )
    wrong_gate_report = module.audit_candidate_bundle(
        bundle,
        remote=True,
        runner=wrong_gate_runner,
    )
    assert wrong_gate_report['status'] == 'REMOTE_AUDIT_FAIL'
    assert 'workflow-run-identity-mismatch' in wrong_gate_report['findings']
    assert wrong_gate_report['artifacts']['status'] == 'FAIL'

    extra_runner, _calls = _remote_runner(
        module,
        bundle,
        extra_artifact=True,
    )
    extra_report = module.audit_candidate_bundle(
        bundle,
        remote=True,
        runner=extra_runner,
    )
    assert extra_report['status'] == 'REMOTE_AUDIT_FAIL'
    assert 'candidate-artifact-set-incomplete-or-expired' in (
        extra_report['findings']
    )

    expired_runner, _calls = _remote_runner(
        module,
        bundle,
        expire_artifact=True,
    )
    expired_report = module.audit_candidate_bundle(
        bundle,
        remote=True,
        runner=expired_runner,
    )
    assert expired_report['status'] == 'REMOTE_AUDIT_FAIL'
    assert 'candidate-artifact-set-incomplete-or-expired' in (
        expired_report['findings']
    )


def test_remote_mode_rejects_set_only_before_network_access():
    """Artifact authenticity can never be claimed from one local set file."""
    module = _module()
    called = False

    def runner(*_args, **_kwargs):
        nonlocal called
        called = True
        raise AssertionError('network command must not run')

    with pytest.raises(
        module.CandidateSetAuditError,
        match='all four retained files',
    ):
        module.audit_candidate_set(
            _candidate_set(module),
            candidate_set_sha256='d' * 64,
            remote=True,
            runner=runner,
        )
    assert called is False


def test_cli_hashes_the_exact_four_file_bundle_without_remote_reads(
    tmp_path,
    capsys,
):
    """The primary CLI is one directory and stays offline by default."""
    module = _module()
    directory = tmp_path / 'candidate-evidence'
    bundle = _write_bundle(module, directory)

    assert module.main([
        '--candidate-evidence-dir', str(directory), '--json',
    ]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['candidate_bundle_sha256'] == bundle['bundle_sha256']
    assert report['status'] == 'LOCAL_CONTRACT_PASS'

    assert module.main([
        '--candidate-image-set',
        str(directory / 'candidate-image-set.json'),
        '--remote',
    ]) == 2
    assert 'requires --candidate-evidence-dir' in capsys.readouterr().err
