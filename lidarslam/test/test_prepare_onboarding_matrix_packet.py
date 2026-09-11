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
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for the exact-identity G0 observer packet."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'prepare_onboarding_matrix_packet.py'


def _module():
    sys.path.insert(0, str(SCRIPT.parent))
    try:
        spec = importlib.util.spec_from_file_location(
            'prepare_onboarding_matrix_packet', SCRIPT
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPT.parent))


def _packet(module):
    return module.build_release_packet_from_report(_release_payload())


def _release_report():
    checks = [
        'tag-commit',
        'release-tag',
        'release-finalized',
        'stable-release-channel',
        'release-url',
        'required-assets',
        'cross-asset-identity',
        'live-image-tag-digests',
    ]
    assets = [
        'lidarslam_ros2_v0.9.1_release_bundle.tar.gz',
        'lidarslam-map-docker',
        'release-image-humble.json',
        'release-image-jazzy.json',
        'release-promotion.json',
        'rollback-plan-humble.json',
        'rollback-plan-jazzy.json',
    ]
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/published-release-v1.schema.json'
        ),
        'status': 'PUBLISHED',
        'repository': 'rsasaki0109/lidar_slam_ros2',
        'expected_version': '0.9.1',
        'expected_tag': 'v0.9.1',
        'remote': {
            'tag_present': True,
            'tag_commit': 'a' * 40,
            'release_present': True,
            'draft': False,
            'prerelease': False,
            'html_url': (
                'https://github.com/rsasaki0109/lidar_slam_ros2/'
                'releases/tag/v0.9.1'
            ),
            'errors': [],
        },
        'checks': [
            {'id': check, 'status': 'PASS', 'detail': 'verified'}
            for check in checks
        ],
        'assets': [
            {
                'name': asset,
                'status': 'PASS',
                'size_bytes': 1,
                'sha256': hashlib.sha256(asset.encode()).hexdigest(),
                'detail': 'verified',
            }
            for asset in assets
        ],
        'images': [
            {
                'tag': (
                    'ghcr.io/rsasaki0109/lidar_slam_ros2:'
                    'v0.9.1-humble'
                ),
                'status': 'PUBLISHED',
                'digest': 'sha256:' + 'b' * 64,
                'detail': 'verified',
            },
            {
                'tag': (
                    'ghcr.io/rsasaki0109/lidar_slam_ros2:'
                    'v0.9.1-jazzy'
                ),
                'status': 'PUBLISHED',
                'digest': 'sha256:' + 'c' * 64,
                'detail': 'verified',
            },
        ],
    }


def _release_payload(report=None):
    value = _release_report() if report is None else report
    return (json.dumps(value, indent=2, sort_keys=True) + '\n').encode()


def _candidate_set():
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/candidate-image-set-v1.schema.json'
        ),
        'status': 'PASS',
        'publication_mode': 'digest_only',
        'repository': 'rsasaki0109/lidar_slam_ros2',
        'source_pr': 427,
        'source_commit': 'a' * 40,
        'product_version': '0.9.1',
        'platform': 'linux/amd64',
        'workflow_run_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/'
            'actions/runs/12345'
        ),
        'workflow_branch_ref': 'refs/heads/develop',
        'workflow_gate_commit': 'f' * 40,
        'requested_by': 'maintainer',
        'images': [
            {
                'ros_distro': 'humble',
                'digest': 'sha256:' + 'b' * 64,
                'immutable_ref': (
                    'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:'
                    + 'b' * 64
                ),
            },
            {
                'ros_distro': 'jazzy',
                'digest': 'sha256:' + 'c' * 64,
                'immutable_ref': (
                    'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:'
                    + 'c' * 64
                ),
            },
        ],
        'tags_created': [],
        'moving_tags_mutated': False,
        'release_mutated': False,
        'registry_retention_status': 'REQUIRES_REMOTE_AUDIT',
        'evidence_retention_days': 30,
    }


def _candidate_request():
    required_checks = [
        'build (humble)',
        'build (jazzy)',
        'docs and release metadata',
        'humble default workflow',
        'humble v0.6.0 to candidate',
        'jazzy default workflow',
        'jazzy v0.6.0 to candidate',
        'release readiness',
        'release readiness threshold guard',
    ]
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
        'required_success_checks': sorted(required_checks),
        'observed_successful_checks': sorted(required_checks),
        'observed_skipped_checks': [
            'authorize immutable candidate request',
            'build and push (${{ matrix.ros_distro }})',
            'publish immutable digest (${{ matrix.ros_distro }})',
            'verify immutable candidate pair',
        ],
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


def _candidate_record(distro, digest_character):
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/candidate-image-v1.schema.json'
        ),
        'status': 'PASS',
        'publication_mode': 'digest_only',
        'repository': 'rsasaki0109/lidar_slam_ros2',
        'source_pr': 427,
        'source_commit': 'a' * 40,
        'product_version': '0.9.1',
        'ros_distro': distro,
        'platform': 'linux/amd64',
        'digest': 'sha256:' + digest_character * 64,
        'immutable_ref': (
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:'
            + digest_character * 64
        ),
        'cli_version': 'lidarslam_ros2 0.9.1',
        'workflow_run_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/'
            'actions/runs/12345'
        ),
        'workflow_branch_ref': 'refs/heads/develop',
        'workflow_gate_commit': 'f' * 40,
        'requested_by': 'maintainer',
        'tags_created': [],
        'moving_tags_mutated': False,
        'release_mutated': False,
        'registry_retention_status': 'REQUIRES_REMOTE_AUDIT',
        'evidence_retention_days': 30,
    }


def _write_candidate_bundle(module, directory):
    directory.mkdir()
    documents = {
        'candidate-image-request.json': _candidate_request(),
        'candidate-image-humble.json': _candidate_record('humble', 'b'),
        'candidate-image-jazzy.json': _candidate_record('jazzy', 'c'),
        'candidate-image-set.json': _candidate_set(),
    }
    for filename, document in documents.items():
        (directory / filename).write_text(
            json.dumps(document, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
    return module.load_candidate_evidence_bundle(directory)


def test_packet_aligns_all_rows_to_one_version_and_exact_identities():
    """Every row shares the requested version and immutable identity."""
    module = _module()
    packet = _packet(module)

    assert packet['status'] == 'READY_FOR_READ_ONLY_PREFLIGHT'
    assert packet['schema_version'] == 3
    assert packet['packet_id'] == 'g0-onboarding-0.9.1-release'
    assert packet['docker_identity_mode'] == 'release'
    assert packet['docker_evidence']['release_report_sha256'] == (
        hashlib.sha256(_release_payload()).hexdigest()
    )
    assert packet['docker_evidence']['release_tag'] == 'v0.9.1'
    assert packet['docker_evidence']['release_commit'] == 'a' * 40
    assert [row['row_id'] for row in packet['rows']] == [
        'docker-humble',
        'docker-jazzy',
        'source-humble',
        'source-jazzy',
    ]
    assert {row['product_version'] for row in packet['rows']} == {'0.9.1'}
    assert packet['rows'][0]['identity']['value'] == 'sha256:' + 'b' * 64
    assert packet['rows'][1]['identity']['value'] == 'sha256:' + 'c' * 64
    assert packet['rows'][2]['identity']['value'] == 'a' * 40
    assert packet['rows'][3]['identity']['value'] == 'a' * 40
    assert all(
        len(row['required_measurements']) == 7 for row in packet['rows']
    )
    assert packet['authority'] == {
        'network_reads_performed': False,
        'trial_executed': False,
        'github_writes_authorized': False,
        'community_posts_authorized': False,
        'remote_mutations_performed': False,
    }


def test_packet_commands_pin_identity_and_keep_paths_as_placeholders():
    """Rendered commands pin identities without leaking local paths."""
    module = _module()
    packet = _packet(module)
    commands = '\n'.join(
        [
            packet['public_checks']['docker']['command'],
            packet['public_checks']['source']['command'],
            *(row['observer_command'] for row in packet['rows']),
        ]
    )

    assert '--version 0.9.1' in commands
    assert 'a' * 40 in commands
    assert 'sha256:' + 'b' * 64 in commands
    assert 'sha256:' + 'c' * 64 in commands
    assert '--prompt-human-measurements' in commands
    assert '<TRIAL_RECORD_OUTSIDE_CHECKOUT>' in commands
    assert '<TRIAL_ROOT_OUTSIDE_CHECKOUT>' in commands
    assert '<OBSERVER_PARENT_OUTSIDE_CHECKOUT>' in commands
    assert '/home/' not in commands
    assert '$HOME' not in commands
    assert packet['public_checks']['docker']['read_only'] is True
    assert packet['public_checks']['source']['read_only'] is True
    assert packet['public_checks']['docker']['command'] == (
        'python3 scripts/check_published_onboarding_identity.py '
        '--version 0.9.1 --source-commit ' + 'a' * 40 + ' '
        '--docker-humble-digest sha256:' + 'b' * 64 + ' '
        '--docker-jazzy-digest sha256:' + 'c' * 64 + ' --json'
    )
    assert packet['public_checks']['docker']['command'].count(
        'check_published_onboarding_identity.py'
    ) == 1


def test_candidate_packet_uses_retained_bundle_without_release_tags(tmp_path):
    """Candidate rows bind all four files and remain tag-free end to end."""
    module = _module()
    bundle = _write_candidate_bundle(module, tmp_path / 'candidate-evidence')
    packet = module.build_candidate_packet(bundle)
    payload = json.dumps(packet, sort_keys=True)
    docker_rows = [
        row for row in packet['rows'] if row['route'] == 'docker'
    ]

    assert packet['packet_id'] == 'g0-onboarding-0.9.1-candidate-12345'
    assert packet['docker_identity_mode'] == 'candidate-image-set'
    assert packet['docker_evidence'] == {
        'mode': 'candidate-image-set',
        'release_report_sha256': None,
        'release_tag': None,
        'release_commit': None,
        'release_url': None,
        'candidate_set_sha256': bundle['file_hashes'][
            'candidate-image-set.json'
        ],
        'candidate_bundle_sha256': bundle['bundle_sha256'],
        'source_pr': 427,
        'source_commit': 'a' * 40,
        'workflow_run_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/'
            'actions/runs/12345'
        ),
        'requested_by': 'maintainer',
        'registry_retention_status': 'REQUIRES_REMOTE_AUDIT',
        'evidence_retention_days': 30,
    }
    assert packet['public_checks']['docker']['required_status'] == (
        'REMOTE_AUDIT_PASS'
    )
    assert packet['public_checks']['docker']['command'] == (
        'python3 scripts/audit_candidate_image_set.py '
        "--candidate-evidence-dir '<CANDIDATE_EVIDENCE_DIR>' --remote --json"
    )
    assert all(row['identity']['tag'] is None for row in docker_rows)
    assert all(
        row['identity']['immutable_ref'].startswith(
            'ghcr.io/rsasaki0109/lidar_slam_ros2@sha256:'
        )
        for row in docker_rows
    )
    assert '--candidate-image-ref' in payload
    assert '--candidate-image-set-sha256 ' + bundle['file_hashes'][
        'candidate-image-set.json'
    ] in payload
    assert '--candidate-evidence-bundle-sha256 ' + (
        bundle['bundle_sha256']
    ) in payload
    assert '--candidate-source-pr 427' in payload
    assert 'v0.9.1-humble' not in payload
    assert 'v0.9.1-jazzy' not in payload
    assert 'check_published_onboarding_identity.py' not in payload


def test_candidate_cli_derives_identity_and_rejects_manual_overrides(
    tmp_path,
    capsys,
):
    """Candidate CLI accepts one retained set and rejects mixed identity."""
    module = _module()
    candidate_directory = tmp_path / 'candidate-evidence'
    bundle = _write_candidate_bundle(module, candidate_directory)

    assert module.main([
        '--candidate-evidence-dir', str(candidate_directory), '--json',
    ]) == 0
    packet = json.loads(capsys.readouterr().out)
    assert packet['docker_evidence']['candidate_set_sha256'] == (
        bundle['file_hashes']['candidate-image-set.json']
    )
    assert packet['docker_evidence']['candidate_bundle_sha256'] == (
        bundle['bundle_sha256']
    )

    assert module.main([
        '--candidate-evidence-dir', str(candidate_directory),
        '--product-version', '0.9.1',
        '--json',
    ]) == 2
    assert 'manual release identity inputs are not accepted' in (
        capsys.readouterr().err
    )


def test_candidate_packet_rejects_a_malformed_retained_set(tmp_path):
    """A schema-shaped but duplicate distro pair fails before rendering."""
    module = _module()
    bundle = _write_candidate_bundle(module, tmp_path / 'candidate-evidence')
    bundle['candidate_set']['images'][1]['ros_distro'] = 'humble'

    with pytest.raises(module.PacketError, match='candidate image evidence'):
        module.build_candidate_packet(bundle)


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('expected_version', 'development'),
        ('tag_commit', 'A' * 40),
        ('humble_digest', 'sha256:' + 'b' * 63),
        ('jazzy_digest', 'sha256:' + 'b' * 64),
    ],
)
def test_packet_rejects_ambiguous_or_malformed_identity(field, value):
    """Malformed or copied report identities fail before packet output."""
    module = _module()
    report = _release_report()
    if field == 'expected_version':
        report[field] = value
    elif field == 'tag_commit':
        report['remote'][field] = value
    elif field == 'humble_digest':
        report['images'][0]['digest'] = value
    else:
        report['images'][1]['digest'] = value

    with pytest.raises(module.PacketError):
        module.build_release_packet_from_report(_release_payload(report))


def test_release_packet_requires_a_complete_published_report():
    """Schema-valid status or check weakening cannot create a packet."""
    module = _module()
    unpublished = _release_report()
    unpublished['status'] = 'IN_PROGRESS'
    unpublished['remote']['release_present'] = False
    unpublished['remote']['draft'] = None
    unpublished['remote']['prerelease'] = None
    unpublished['remote']['html_url'] = None
    with pytest.raises(module.PacketError, match='status PUBLISHED'):
        module.build_release_packet_from_report(_release_payload(unpublished))

    incomplete = _release_report()
    incomplete['checks'] = [
        check for check in incomplete['checks']
        if check['id'] != 'live-image-tag-digests'
    ]
    with pytest.raises(module.PacketError, match='lacks required PASS'):
        module.build_release_packet_from_report(_release_payload(incomplete))


def test_render_is_explicitly_a_plan_not_a_measurement_record():
    """The human card remains a plan and never claims observed evidence."""
    module = _module()
    rendered = module.render_packet(_packet(module))

    assert 'READY_FOR_READ_ONLY_PREFLIGHT' in rendered
    assert 'not a trial record or a release claim' in rendered
    assert 'Published release tag: `v0.9.1`' in rendered
    assert f'Published release commit: `{"a" * 40}`' in rendered
    assert 'Published release audit SHA-256:' in rendered
    assert 'active_operator_time_sec' in rendered
    assert 'command_count' in rendered
    assert 'network_reads_performed' not in rendered


def test_output_is_exclusive_and_does_not_overwrite(tmp_path, capsys):
    """The packet writer refuses a second write to the same path."""
    module = _module()
    output = tmp_path / 'observer-packet.json'
    report = tmp_path / 'published-release.json'
    report.write_bytes(_release_payload())
    args = [
        '--published-release-report',
        str(report),
        '--json',
        '--output',
        str(output),
    ]

    assert module.main(args) == 0
    original = output.read_bytes()
    assert module.main(args) == 2
    assert output.read_bytes() == original
    assert 'File exists' in capsys.readouterr().err


def test_release_cli_rejects_manual_identity_and_symlink(tmp_path, capsys):
    """Release CLI derives identity from regular report bytes only."""
    module = _module()
    assert module.main([
        '--product-version', '0.9.1',
        '--source-commit', 'a' * 40,
        '--docker-humble-digest', 'sha256:' + 'b' * 64,
        '--docker-jazzy-digest', 'sha256:' + 'c' * 64,
        '--json',
    ]) == 2
    assert 'manual release identity inputs are not accepted' in (
        capsys.readouterr().err
    )

    report = tmp_path / 'published-release.json'
    report.write_bytes(_release_payload())
    alias = tmp_path / 'report-link.json'
    alias.symlink_to(report)
    assert module.main([
        '--published-release-report', str(alias), '--json',
    ]) == 2
    assert 'regular non-symlink' in capsys.readouterr().err
