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

"""Tests for the atomic one-command candidate-trial handoff."""

from __future__ import annotations

import importlib.util
import json
import pathlib
import subprocess
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT_DIR = ROOT / 'scripts'
PREPARATION_SCRIPT = SCRIPT_DIR / 'prepare_candidate_trial.py'
AUDIT_SCRIPT = SCRIPT_DIR / 'audit_candidate_image_set.py'
RUN_URL = (
    'https://github.com/rsasaki0109/lidar_slam_ros2/'
    'actions/runs/12345'
)


def _load_module(name, path):
    sys.path.insert(0, str(SCRIPT_DIR))
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPT_DIR))


def _audit_module():
    return _load_module('candidate_trial_fixture_audit', AUDIT_SCRIPT)


def _preparation_module():
    return _load_module('prepare_candidate_trial_test', PREPARATION_SCRIPT)


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


def _write_source_bundle(directory):
    module = _audit_module()
    directory.mkdir()
    request = _request(module)
    records = {}
    for distro, digest_character in (('humble', 'b'), ('jazzy', 'c')):
        records[distro] = module.build_candidate_image_record(
            request,
            ros_distro=distro,
            platform='linux/amd64',
            digest='sha256:' + digest_character * 64,
            cli_version='lidarslam_ros2 0.9.1',
            workflow_run_url=RUN_URL,
            evidence_retention_days=30,
        )
    candidate_set = module.verify_candidate_image_set([
        records['humble'], records['jazzy'],
    ])
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


def _remote_runner(
    module,
    source_bundle,
    *,
    fail_first_download=None,
    mismatch_second_download=None,
):
    calls = []
    download_counts = {}

    def runner(command, **_kwargs):
        calls.append(command)
        if command[:3] == ['gh', 'run', 'download']:
            artifact_name = command[command.index('--name') + 1]
            destination = pathlib.Path(command[command.index('--dir') + 1])
            download_counts[artifact_name] = (
                download_counts.get(artifact_name, 0) + 1
            )
            if (
                artifact_name == fail_first_download
                and download_counts[artifact_name] == 1
            ):
                return subprocess.CompletedProcess(
                    command, 1, '', 'artifact unavailable'
                )
            filename = next(
                item[1] for item in module.EVIDENCE_FILES
                if item[0] == artifact_name
            )
            payload = source_bundle['paths'][filename].read_bytes()
            if (
                artifact_name == mismatch_second_download
                and download_counts[artifact_name] == 2
            ):
                payload += b'\n'
            (destination / filename).write_bytes(payload)
            return subprocess.CompletedProcess(command, 0, '', '')
        if command[:2] == ['gh', 'api'] and command[2].endswith(
            '/actions/runs/12345'
        ):
            payload = {
                'id': 12345,
                'html_url': RUN_URL,
                'event': 'repository_dispatch',
                'status': 'completed',
                'conclusion': 'success',
                'head_branch': 'develop',
                'head_sha': source_bundle['candidate_set'][
                    'workflow_gate_commit'
                ],
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
                        'expired': False,
                        'expires_at': '2026-09-14T00:00:00Z',
                        'workflow_run': {
                            'id': 12345,
                            'head_branch': 'develop',
                            'head_sha': source_bundle['candidate_set'][
                                'workflow_gate_commit'
                            ],
                        },
                    }
                    for index, name in enumerate(module.EXPECTED_ARTIFACTS)
                ]
            payload = {
                'total_count': len(artifacts),
                'artifacts': artifacts,
            }
            return subprocess.CompletedProcess(
                command, 0, json.dumps(payload), ''
            )
        if command[:4] == ['docker', 'buildx', 'imagetools', 'inspect']:
            digest = command[4].split('@', 1)[1]
            return subprocess.CompletedProcess(
                command, 0, json.dumps({'digest': digest}), ''
            )
        if command[:3] == ['gh', 'attestation', 'verify']:
            return subprocess.CompletedProcess(command, 0, '', '')
        raise AssertionError(f'unexpected command: {command}')

    return runner, calls


def _staging_paths(parent, output_name):
    return list(parent.glob(f'.{output_name}.preparing-*'))


def test_preparation_downloads_audits_and_publishes_one_complete_handoff(
    tmp_path,
):
    """Success publishes only a complete, schema-valid, portable handoff."""
    module = _preparation_module()
    source_bundle = _write_source_bundle(tmp_path / 'source')
    runner, calls = _remote_runner(module, source_bundle)
    output = tmp_path / 'candidate-handoff'

    receipt = module.prepare_candidate_trial(
        RUN_URL,
        output,
        runner=runner,
        prepared_at='2026-08-15T12:00:00Z',
    )

    assert output.is_dir()
    assert sorted(item.name for item in output.iterdir()) == [
        'artifacts',
        'candidate-audit.json',
        'observer-packet.json',
        'observer-packet.md',
        'preparation.json',
    ]
    assert sorted(item.name for item in (output / 'artifacts').iterdir()) == (
        sorted(module.EXPECTED_FILENAMES)
    )
    assert json.loads((output / 'preparation.json').read_text()) == receipt
    audit = json.loads((output / 'candidate-audit.json').read_text())
    packet = json.loads((output / 'observer-packet.json').read_text())
    assert audit['status'] == 'REMOTE_AUDIT_PASS'
    assert packet['status'] == 'READY_FOR_READ_ONLY_PREFLIGHT'
    assert receipt['candidate_bundle_sha256'] == source_bundle[
        'bundle_sha256'
    ]
    assert receipt['authority']['trial_executed'] is False
    assert receipt['authority']['remote_mutations_performed'] is False
    assert receipt['artifact_expires_at'] == '2026-09-14T00:00:00Z'
    assert receipt['outputs']['artifacts_directory'] == 'artifacts'
    module.validate_contract(receipt, module.PREPARATION_SCHEMA)
    loaded = module.load_candidate_trial_handoff(output)
    assert loaded['preparation'] == receipt
    assert loaded['packet'] == packet
    assert loaded['audit'] == audit
    assert receipt['candidate_bundle_sha256'] in (
        output / 'observer-packet.md'
    ).read_text()
    assert sum(call[:3] == ['gh', 'run', 'download'] for call in calls) == 8
    assert sum(call[:2] == ['gh', 'api'] for call in calls) == 2
    assert sum(
        call[:3] == ['gh', 'attestation', 'verify'] for call in calls
    ) == 2
    assert _staging_paths(tmp_path, output.name) == []


def test_handoff_loader_rejects_schema_valid_packet_substitution(tmp_path):
    """A retained packet is rebuilt instead of trusted as a command source."""
    module = _preparation_module()
    source_bundle = _write_source_bundle(tmp_path / 'source')
    runner, _calls = _remote_runner(module, source_bundle)
    output = tmp_path / 'candidate-handoff'
    module.prepare_candidate_trial(RUN_URL, output, runner=runner)
    packet_path = output / 'observer-packet.json'
    packet = json.loads(packet_path.read_text(encoding='utf-8'))
    packet['rows'][0]['observer_command'] += ' --unexpected'
    packet_path.write_text(
        json.dumps(packet, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )

    with pytest.raises(
        module.CandidateTrialCheckError,
        match='does not derive from artifact bytes',
    ):
        module.load_candidate_trial_handoff(output)


def test_handoff_loader_rejects_retained_audit_identity_substitution(
    tmp_path,
):
    """REMOTE_AUDIT_PASS cannot be copied onto a different source identity."""
    module = _preparation_module()
    source_bundle = _write_source_bundle(tmp_path / 'source')
    runner, _calls = _remote_runner(module, source_bundle)
    output = tmp_path / 'candidate-handoff'
    module.prepare_candidate_trial(RUN_URL, output, runner=runner)
    audit_path = output / 'candidate-audit.json'
    audit = json.loads(audit_path.read_text(encoding='utf-8'))
    audit['source_commit'] = 'd' * 40
    audit_path.write_text(
        json.dumps(audit, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )

    with pytest.raises(
        module.CandidateTrialCheckError,
        match='audit identity does not match artifact bytes',
    ):
        module.load_candidate_trial_handoff(output)


def test_handoff_loader_rejects_extra_top_level_entry(tmp_path):
    """A handoff cannot smuggle an unbound script beside reviewed files."""
    module = _preparation_module()
    source_bundle = _write_source_bundle(tmp_path / 'source')
    runner, _calls = _remote_runner(module, source_bundle)
    output = tmp_path / 'candidate-handoff'
    module.prepare_candidate_trial(RUN_URL, output, runner=runner)
    (output / 'run-me.sh').write_text('exit 0\n', encoding='utf-8')

    with pytest.raises(
        module.CandidateTrialCheckError,
        match='must contain exactly',
    ):
        module.load_candidate_trial_handoff(output)


def test_second_download_mismatch_leaves_no_partial_handoff(tmp_path):
    """Independent byte mismatch removes staging and blocks packet output."""
    module = _preparation_module()
    source_bundle = _write_source_bundle(tmp_path / 'source')
    runner, _calls = _remote_runner(
        module,
        source_bundle,
        mismatch_second_download='candidate-image-set',
    )
    output = tmp_path / 'candidate-handoff'

    with pytest.raises(
        module.CandidateTrialCheckError,
        match='candidate-image-set-content-mismatch',
    ):
        module.prepare_candidate_trial(RUN_URL, output, runner=runner)

    assert not output.exists()
    assert _staging_paths(tmp_path, output.name) == []


def test_initial_download_failure_leaves_no_partial_handoff(tmp_path):
    """Artifact acquisition failures never expose an incomplete directory."""
    module = _preparation_module()
    source_bundle = _write_source_bundle(tmp_path / 'source')
    runner, _calls = _remote_runner(
        module,
        source_bundle,
        fail_first_download='candidate-image-record-jazzy',
    )
    output = tmp_path / 'candidate-handoff'

    with pytest.raises(
        module.CandidateTrialCheckError,
        match='candidate-image-record-jazzy',
    ):
        module.prepare_candidate_trial(RUN_URL, output, runner=runner)

    assert not output.exists()
    assert _staging_paths(tmp_path, output.name) == []


def test_downloaded_evidence_must_name_the_requested_run(tmp_path):
    """A valid bundle from another run cannot satisfy the selected URL."""
    module = _preparation_module()
    source_bundle = _write_source_bundle(tmp_path / 'source')
    runner, calls = _remote_runner(module, source_bundle)
    output = tmp_path / 'candidate-handoff'
    other_run_url = RUN_URL.rsplit('/', 1)[0] + '/12346'

    with pytest.raises(
        module.CandidateTrialCheckError,
        match='different workflow run URL',
    ):
        module.prepare_candidate_trial(other_run_url, output, runner=runner)

    assert not output.exists()
    assert sum(call[:3] == ['gh', 'run', 'download'] for call in calls) == 4
    assert not any(call[:2] == ['gh', 'api'] for call in calls)
    assert _staging_paths(tmp_path, output.name) == []


def test_existing_output_is_rejected_before_any_network_read(tmp_path):
    """An existing destination is preserved byte-for-byte and never queried."""
    module = _preparation_module()
    output = tmp_path / 'candidate-handoff'
    output.mkdir()
    sentinel = output / 'keep.txt'
    sentinel.write_text('keep\n', encoding='utf-8')
    calls = []

    def runner(command, **_kwargs):
        calls.append(command)
        raise AssertionError('network command must not run')

    with pytest.raises(
        module.CandidateTrialPreparationError,
        match='refusing to overwrite',
    ):
        module.prepare_candidate_trial(RUN_URL, output, runner=runner)

    assert sentinel.read_text(encoding='utf-8') == 'keep\n'
    assert calls == []


@pytest.mark.parametrize(
    'workflow_run_url',
    [
        'https://github.com/other/repository/actions/runs/12345',
        RUN_URL + '?attempt=1',
        RUN_URL + '/',
        '12345',
    ],
)
def test_malformed_or_ambiguous_run_url_fails_before_network(
    tmp_path,
    workflow_run_url,
):
    """Only one exact repository Actions URL can choose remote evidence."""
    module = _preparation_module()
    calls = []

    def runner(command, **_kwargs):
        calls.append(command)
        raise AssertionError('network command must not run')

    with pytest.raises(
        module.CandidateTrialPreparationError,
        match='exact rsasaki0109',
    ):
        module.prepare_candidate_trial(
            workflow_run_url,
            tmp_path / 'candidate-handoff',
            runner=runner,
        )

    assert calls == []
