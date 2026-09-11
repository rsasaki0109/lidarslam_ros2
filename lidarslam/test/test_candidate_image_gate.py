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

"""Regression tests for the immutable candidate-image authority gate."""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import jsonschema

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
WORKFLOW = ROOT / '.github' / 'workflows' / 'candidate-image.yml'
SCHEMAS = ROOT / 'docs' / 'schemas'
REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
SOURCE_COMMIT = 'a' * 40
GATE_COMMIT = 'f' * 40


def _load_module(filename: str, name: str):
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(name, SCRIPTS / filename)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


def _request_module():
    return _load_module(
        'validate_candidate_image_request.py',
        'validate_candidate_image_request_test',
    )


def _pull_request(
    *,
    head_repository: str = REPOSITORY,
    head_sha: str = SOURCE_COMMIT,
    mergeable: bool | None = True,
) -> dict:
    return {
        'number': 427,
        'state': 'open',
        'draft': True,
        'mergeable': mergeable,
        'base': {
            'ref': 'develop',
            'sha': 'b' * 40,
            'repo': {'full_name': REPOSITORY},
        },
        'head': {
            'ref': 'agent/product-g0-guided-ux',
            'sha': head_sha,
            'repo': {'full_name': head_repository},
        },
    }


def _check_runs(module, *, extra: list[dict] | None = None) -> dict:
    runs = [
        {
            'name': name,
            'status': 'completed',
            'conclusion': 'success',
        }
        for name in sorted(module.REQUIRED_SUCCESS_CHECKS)
    ]
    runs.append({
        'name': 'build and push (${{ matrix.ros_distro }})',
        'status': 'completed',
        'conclusion': 'skipped',
    })
    runs.extend(extra or [])
    return {'total_count': len(runs), 'check_runs': runs}


def _environment() -> dict:
    return {
        'name': 'candidate-images',
        'protection_rules': [
            {
                'id': 1,
                'type': 'required_reviewers',
                'prevent_self_review': True,
                'reviewers': [
                    {
                        'type': 'User',
                        'reviewer': {'login': 'release-reviewer'},
                    }
                ],
            },
            {'id': 2, 'type': 'branch_policy'},
        ],
        'deployment_branch_policy': {
            'protected_branches': False,
            'custom_branch_policies': True,
        },
    }


def _branch_policies(name: str = 'develop') -> dict:
    return {
        'total_count': 1,
        'branch_policies': [{'id': 1, 'name': name, 'type': 'branch'}],
    }


def _request(module, **overrides):
    values = {
        'event_name': 'repository_dispatch',
        'event_action': 'e2-publish-candidate-image',
        'repository': REPOSITORY,
        'workflow_branch_ref': 'refs/heads/develop',
        'workflow_gate_commit': GATE_COMMIT,
        'default_branch': 'develop',
        'source_pr': 427,
        'source_commit': SOURCE_COMMIT,
        'product_version': '0.9.1',
        'candidate_version': '0.9.1',
        'requested_by': 'rsasaki0109',
        'actor_role': 'admin',
        'approval': 'E2_IMMUTABLE_DIGEST_ONLY',
        'pull_request': _pull_request(),
        'check_runs_document': _check_runs(module),
        'environment': _environment(),
        'branch_policies_document': _branch_policies(),
    }
    values.update(overrides)
    return module.validate_candidate_image_request(**values)


def _image_record(distro: str, digest_character: str) -> dict:
    request_module = _request_module()
    record_module = _load_module(
        'create_candidate_image_record.py',
        f'create_candidate_image_record_{distro}_{digest_character}',
    )
    return record_module.build_candidate_image_record(
        _request(request_module),
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


def test_candidate_contract_schemas_are_valid_draft7():
    """The three persisted authority records must remain valid schemas."""
    for filename in (
        'candidate-image-request-v1.schema.json',
        'candidate-image-v1.schema.json',
        'candidate-image-set-v1.schema.json',
    ):
        schema = json.loads((SCHEMAS / filename).read_text(encoding='utf-8'))
        jsonschema.Draft7Validator.check_schema(schema)


def test_request_authorizes_exact_same_repo_head_with_green_ci():
    """A maintainer can authorize only one exact, fully green PR head."""
    module = _request_module()
    report = _request(module)

    assert report['status'] == 'AUTHORIZED'
    assert report['publication_mode'] == 'digest_only'
    assert report['source_pr'] == 427
    assert report['source_commit'] == SOURCE_COMMIT
    assert report['workflow_gate_commit'] == GATE_COMMIT
    assert set(report['required_success_checks']) == (
        module.REQUIRED_SUCCESS_CHECKS
    )
    assert report['observed_skipped_checks'] == [
        'build and push (${{ matrix.ros_distro }})'
    ]
    assert report['environment'] == {
        'name': 'candidate-images',
        'required_reviewer_count': 1,
        'prevent_self_review': True,
        'deployment_branch_policy': 'develop_only',
    }
    assert report['authority'] == {
        'package_write_authorized_for_digest_job': True,
        'tag_creation_authorized': False,
        'moving_tag_mutation_authorized': False,
        'release_mutation_authorized': False,
    }


@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    [
        ('event_name', 'workflow_dispatch', 'dedicated event'),
        ('workflow_branch_ref', 'refs/heads/topic', 'default branch'),
        ('workflow_gate_commit', 'main', 'lowercase 40-character'),
        ('actor_role', 'write', 'maintain role'),
        ('approval', 'yes', 'approval is missing'),
        ('candidate_version', '0.9.0', 'VERSION does not match'),
        ('source_commit', 'A' * 40, 'lowercase 40-character'),
    ],
)
def test_request_rejects_ambiguous_authority_or_identity(
    field,
    value,
    message,
):
    """Selectable refs, weak actors, and mismatched identity fail closed."""
    module = _request_module()
    with pytest.raises(ValueError, match=message):
        _request(module, **{field: value})


def test_request_rejects_foreign_or_unmergeable_pull_request():
    """Fork code and unresolved mergeability cannot receive package writes."""
    module = _request_module()
    with pytest.raises(ValueError, match='same-repository'):
        _request(
            module,
            pull_request=_pull_request(head_repository='other/fork'),
        )
    with pytest.raises(ValueError, match='must be mergeable'):
        _request(module, pull_request=_pull_request(mergeable=None))


def test_request_rejects_unprotected_candidate_environment():
    """A named but unprotected environment must not look like approval."""
    module = _request_module()
    environment = _environment()
    environment['protection_rules'] = []
    with pytest.raises(ValueError, match='reviewer rule'):
        _request(module, environment=environment)
    with pytest.raises(ValueError, match='allow develop only'):
        _request(
            module,
            branch_policies_document=_branch_policies('release/*'),
        )

    environment = _environment()
    environment['protection_rules'][0]['prevent_self_review'] = False
    with pytest.raises(ValueError, match='prevent self-review'):
        _request(module, environment=environment)


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        ('pending', 'not completed'),
        ('failure', 'not success'),
        ('missing', 'missing required successful checks'),
        ('unexpected_skip', 'unexpected skipped'),
        ('truncated', 'incomplete or truncated'),
    ],
)
def test_request_rejects_incomplete_or_non_green_exact_head_ci(
    mutation,
    message,
):
    """Only complete exact-head success can cross the package-write gate."""
    module = _request_module()
    document = _check_runs(module)
    if mutation == 'pending':
        document['check_runs'][0]['status'] = 'in_progress'
        document['check_runs'][0]['conclusion'] = None
    elif mutation == 'failure':
        document['check_runs'][0]['conclusion'] = 'failure'
    elif mutation == 'missing':
        document['check_runs'].pop(0)
        document['total_count'] -= 1
    elif mutation == 'unexpected_skip':
        document['check_runs'].append({
            'name': 'security review',
            'status': 'completed',
            'conclusion': 'skipped',
        })
        document['total_count'] += 1
    else:
        document['total_count'] += 1

    with pytest.raises(ValueError, match=message):
        _request(module, check_runs_document=document)


def test_candidate_record_is_digest_only_and_retention_honest():
    """Per-distro evidence names no tag and does not invent retention."""
    record = _image_record('humble', 'c')

    assert record['immutable_ref'] == (
        'ghcr.io/rsasaki0109/lidar_slam_ros2@' + record['digest']
    )
    assert record['tags_created'] == []
    assert record['moving_tags_mutated'] is False
    assert record['release_mutated'] is False
    assert record['registry_retention_status'] == 'REQUIRES_REMOTE_AUDIT'
    assert record['evidence_retention_days'] == 30


def test_candidate_pair_requires_distinct_consistent_humble_and_jazzy():
    """A matrix-ready report needs both distinct, exact-source digests."""
    module = _load_module(
        'verify_candidate_image_set.py',
        'verify_candidate_image_set_test',
    )
    humble = _image_record('humble', 'c')
    jazzy = _image_record('jazzy', 'd')
    report = module.verify_candidate_image_set([humble, jazzy])

    assert report['status'] == 'PASS'
    assert [image['ros_distro'] for image in report['images']] == [
        'humble',
        'jazzy',
    ]
    assert report['tags_created'] == []
    assert report['moving_tags_mutated'] is False
    assert report['registry_retention_status'] == 'REQUIRES_REMOTE_AUDIT'

    copied = dict(jazzy, digest=humble['digest'])
    copied['immutable_ref'] = humble['immutable_ref']
    with pytest.raises(ValueError, match='digests must differ'):
        module.verify_candidate_image_set([humble, copied])


def test_candidate_clis_persist_one_complete_set_without_overwrite(
    tmp_path: pathlib.Path,
):
    """Workflow-facing CLIs should write one auditable pair exactly once."""
    request_module = _request_module()
    fixtures = {
        'pull-request.json': _pull_request(),
        'check-runs.json': _check_runs(request_module),
        'environment.json': _environment(),
        'branch-policies.json': _branch_policies(),
    }
    for filename, payload in fixtures.items():
        (tmp_path / filename).write_text(
            json.dumps(payload),
            encoding='utf-8',
        )
    (tmp_path / 'VERSION').write_text('0.9.1\n', encoding='utf-8')
    request_path = tmp_path / 'candidate-image-request.json'
    request_args = [
        '--event-name', 'repository_dispatch',
        '--event-action', 'e2-publish-candidate-image',
        '--repository', REPOSITORY,
        '--workflow-branch-ref', 'refs/heads/develop',
        '--workflow-gate-commit', GATE_COMMIT,
        '--default-branch', 'develop',
        '--source-pr', '427',
        '--source-commit', SOURCE_COMMIT,
        '--product-version', '0.9.1',
        '--candidate-version-file', str(tmp_path / 'VERSION'),
        '--requested-by', 'rsasaki0109',
        '--actor-role', 'admin',
        '--approval', 'E2_IMMUTABLE_DIGEST_ONLY',
        '--pull-request-json', str(tmp_path / 'pull-request.json'),
        '--check-runs-json', str(tmp_path / 'check-runs.json'),
        '--environment-json', str(tmp_path / 'environment.json'),
        '--branch-policies-json', str(tmp_path / 'branch-policies.json'),
        '--output', str(request_path),
    ]
    assert request_module.main(request_args) == 0
    assert request_module.main(request_args) == 2

    record_module = _load_module(
        'create_candidate_image_record.py',
        'create_candidate_image_record_cli_test',
    )
    record_paths = []
    for distro, character in (('humble', 'c'), ('jazzy', 'd')):
        record_path = tmp_path / f'candidate-image-{distro}.json'
        record_paths.append(record_path)
        assert record_module.main([
            '--request', str(request_path),
            '--ros-distro', distro,
            '--platform', 'linux/amd64',
            '--digest', 'sha256:' + character * 64,
            '--cli-version', 'lidarslam_ros2 0.9.1',
            '--workflow-run-url',
            'https://github.com/rsasaki0109/lidar_slam_ros2/'
            'actions/runs/12345',
            '--evidence-retention-days', '30',
            '--output', str(record_path),
        ]) == 0

    set_module = _load_module(
        'verify_candidate_image_set.py',
        'verify_candidate_image_set_cli_test',
    )
    set_path = tmp_path / 'candidate-image-set.json'
    set_args = [
        str(record_paths[0]),
        str(record_paths[1]),
        '--output',
        str(set_path),
    ]
    assert set_module.main(set_args) == 0
    assert set_module.main(set_args) == 2
    assert json.loads(set_path.read_text(encoding='utf-8'))['status'] == 'PASS'


def test_workflow_separates_contract_authorization_and_publication():
    """Only the digest job receives package-write authority."""
    workflow = WORKFLOW.read_text(encoding='utf-8')
    contract = workflow.split('  contract:', 1)[1].split(
        '  authorize:', 1
    )[0]
    authorize = workflow.split('  authorize:', 1)[1].split(
        '  publish:', 1
    )[0]
    publish = workflow.split('  publish:', 1)[1].split(
        '  verify-set:', 1
    )[0]
    verify_set = workflow.split('  verify-set:', 1)[1]

    assert 'workflow_dispatch:' not in workflow
    assert 'repository_dispatch:' in workflow
    assert 'e2-publish-candidate-image' in workflow
    assert 'permissions:\n  contents: read' in workflow
    assert 'packages: write' not in contract
    assert 'packages: write' not in authorize
    assert 'packages: write' not in verify_set
    assert publish.count('packages: write') == 1
    assert 'actions: read' in authorize
    assert 'pull-requests: read' in authorize
    assert workflow.count('ref: ${{ github.sha }}') == 3
    assert 'ref: ${{ github.event.repository.default_branch }}' not in workflow
    assert '--workflow-gate-commit "${GITHUB_SHA}"' in authorize
    assert 'E2_IMMUTABLE_DIGEST_ONLY' in authorize
    assert 'check-runs?filter=latest&per_page=100' in authorize
    assert 'environments/candidate-images' in authorize
    assert 'scripts/check_candidate_environment.py' in workflow
    assert 'deployment-branch-policies?per_page=100' in authorize
    assert 'test_candidate_environment_readiness.py' in contract
    assert 'context: candidate' in publish
    assert 'push-by-digest=true' in publish
    assert 'name-canonical=true' in publish
    assert 'push=true' in publish
    assert '\n          tags:' not in publish
    assert 'docker buildx imagetools create' not in workflow
    assert 'ghcr.io/${{ github.repository }}:humble' not in workflow
    assert 'candidate-images' in publish
    assert 'actions/attest@v4' in publish
    assert 'sbom: true' in publish
    assert 'provenance: mode=max' in publish
    assert 'gate/scripts/create_candidate_image_record.py' in publish
    assert 'gate/scripts/verify_candidate_image_set.py' in verify_set
