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

"""Regression tests for the canonical ndt_omp convergence contract."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import jsonschema
import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_canonical_ndt_convergence.py'
CONTRACT = ROOT / 'docs' / 'contracts' / 'canonical-ndt-convergence-v1.json'
REPORT_SCHEMA = (
    ROOT
    / 'docs'
    / 'schemas'
    / 'canonical-ndt-convergence-readiness-v1.schema.json'
)
SPEC = importlib.util.spec_from_file_location(
    'canonical_ndt_convergence', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


def _contract() -> dict:
    return json.loads(CONTRACT.read_text(encoding='utf-8'))


def _write_contract(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / 'contract.json'
    path.write_text(json.dumps(payload), encoding='utf-8')
    return path


def _publication_state(**overrides) -> dict:
    contract = _contract()
    publication = contract['publication']
    state = {
        'inspected': True,
        'upstream_repository': publication['upstream_repository'],
        'fork_repository': publication['fork_repository'],
        'proposed_branch': publication['proposed_branch'],
        'errors': [],
        'upstream_head_sha': contract['upstream']['base_commit'],
        'fork_is_expected': True,
        'proposed_branch_present': False,
        'open_pr_count': 4,
        'duplicate_prs': [],
    }
    state.update(overrides)
    return state


def test_checked_in_bundle_is_artifact_ready_and_schema_valid():
    report = CHECKER.evaluate()

    assert report['status'] == 'ARTIFACTS_READY'
    assert report['mode'] == 'artifacts'
    assert report['authority'] == {
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    assert report['draft_pr_handoff'] is None
    assert all(
        item['status'] == 'PASS'
        for item in report['checks']
        if not item['id'].startswith('upstream-checkout-')
        and item['id'] != 'upstream-patch-applies'
        and not item['id'].startswith('candidate-checkout-')
        and not item['id'].startswith('publication-')
    )
    assert sum(
        item['status'] == 'NOT_CHECKED' for item in report['checks']) == 13
    schema = json.loads(REPORT_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator(schema).validate(report)


def test_parent_transition_covers_both_direct_consumers_exactly():
    report = CHECKER.evaluate()
    checks = {item['id']: item for item in report['checks']}

    assert checks['consumer-graph_based_slam/CMakeLists.txt'] == {
        'id': 'consumer-graph_based_slam/CMakeLists.txt',
        'status': 'PASS',
        'detail': (
            'current=7, deleted=7, canonical-added=7, '
            'virtual-fork-remaining=0'
        ),
    }
    assert checks['consumer-graph_based_slam/package.xml']['status'] == 'PASS'
    assert checks['consumer-scanmatcher/CMakeLists.txt']['status'] == 'PASS'
    assert checks['consumer-scanmatcher/package.xml']['status'] == 'PASS'
    assert checks['parent-api-spelling-transition']['status'] == 'PASS'


def test_exact_clean_upstream_checkout_promotes_local_review_status(
    monkeypatch,
    tmp_path: Path,
):
    checkout = tmp_path / 'upstream'
    checkout.mkdir()
    real_apply = CHECKER._apply_check

    def fake_git_text(repo, *arguments):
        assert repo == checkout
        if arguments == ('rev-parse', 'HEAD'):
            return _contract()['upstream']['base_commit']
        assert arguments == (
            'status', '--porcelain', '--untracked-files=all')
        return ''

    def fake_apply(repo, patch):
        if repo == checkout:
            return True, 'patch applies without modifying the checkout'
        return real_apply(repo, patch)

    monkeypatch.setattr(CHECKER, '_git_text', fake_git_text)
    monkeypatch.setattr(CHECKER, '_apply_check', fake_apply)

    report = CHECKER.evaluate(upstream_checkout=checkout)

    assert report['status'] == 'READY_FOR_UPSTREAM_REVIEW'
    assert report['upstream_checkout'] == {
        'inspected': True,
        'commit': _contract()['upstream']['base_commit'],
        'clean': True,
        'patch_applies': True,
    }
    assert report['draft_pr_handoff'] is None
    assert str(checkout) not in json.dumps(report)


def test_exact_candidate_and_online_state_promote_draft_pr(
    monkeypatch,
    tmp_path: Path,
):
    upstream = tmp_path / 'upstream'
    candidate = tmp_path / 'candidate'
    upstream.mkdir()
    candidate.mkdir()
    contract = _contract()
    base = contract['upstream']['base_commit']
    candidate_commit = contract['publication']['candidate_commit']
    real_apply = CHECKER._apply_check

    def fake_git_text(repo, *arguments):
        if repo == upstream:
            if arguments == ('rev-parse', 'HEAD'):
                return base
            assert arguments == (
                'status', '--porcelain', '--untracked-files=all')
            return ''
        assert repo == candidate
        if arguments == ('rev-parse', 'HEAD'):
            return candidate_commit
        if arguments == ('show', '-s', '--format=%P', 'HEAD'):
            return base
        if arguments == ('show', '-s', '--format=%s', 'HEAD'):
            return contract['publication']['candidate_subject']
        assert arguments == (
            'status', '--porcelain', '--untracked-files=all')
        return ''

    def fake_apply(repo, patch):
        if repo == upstream:
            return True, 'patch applies without modifying the checkout'
        return real_apply(repo, patch)

    monkeypatch.setattr(CHECKER, '_git_text', fake_git_text)
    monkeypatch.setattr(CHECKER, '_apply_check', fake_apply)
    monkeypatch.setattr(
        CHECKER,
        '_git_diff_sha256',
        lambda repo, _base, _commit: (
            contract['upstream']['patch']['sha256']
            if repo == candidate else pytest.fail('unexpected diff checkout')
        ),
    )

    report = CHECKER.evaluate(
        upstream_checkout=upstream,
        candidate_checkout=candidate,
        online=True,
        publication_state=_publication_state(),
    )

    assert report['status'] == 'READY_FOR_DRAFT_PR'
    assert report['mode'] == 'draft-preflight'
    assert all(item['status'] == 'PASS' for item in report['checks'])
    assert report['candidate_checkout'] == {
        'inspected': True,
        'commit': candidate_commit,
        'parent_commit': base,
        'clean': True,
        'subject': contract['publication']['candidate_subject'],
        'patch_matches': True,
    }
    assert report['publication_preflight']['duplicate_prs'] == []
    handoff = report['draft_pr_handoff']
    assert handoff is not None
    assert handoff['kind'] == 'CREATE_CANONICAL_NDT_DRAFT_PR'
    assert handoff['external_write_required'] is True
    assert handoff['upstream_repository'] == 'koide3/ndt_omp'
    assert handoff['base_branch'] == 'master'
    assert handoff['fork_repository'] == 'rsasaki0109/ndt_omp_ros2'
    assert handoff['head_branch'] == (
        'lidarslam-priors-and-correspondence-diagnostics')
    assert handoff['upstream_base_commit'] == base
    assert handoff['candidate_commit'] == candidate_commit
    assert handoff['candidate_subject'] == (
        contract['publication']['candidate_subject'])
    assert handoff['title'] == contract['publication']['draft_pr']['title']
    assert handoff['body'] == contract['publication']['draft_pr']['body']
    assert handoff['preflight'] == {
        'upstream_base_verified': True,
        'fork_identity_verified': True,
        'branch_absent_verified': True,
        'open_pr_count': 4,
        'duplicate_pr_count': 0,
    }
    assert handoff['constraints'] == {
        'draft_required': True,
        'create_only': True,
        'non_force_only': True,
        'push_authorized': False,
        'pr_creation_authorized': False,
        'force_push_authorized': False,
        'mark_ready_authorized': False,
        'merge_authorized': False,
    }
    assert len(handoff['steps']) == 4
    assert handoff['writes_performed'] is False
    assert report['authority']['github_writes_authorized'] is False
    schema = json.loads(REPORT_SCHEMA.read_text(encoding='utf-8'))
    validator = jsonschema.Draft7Validator(schema)
    validator.validate(report)
    missing_handoff = json.loads(json.dumps(report))
    missing_handoff['draft_pr_handoff'] = None
    with pytest.raises(jsonschema.ValidationError):
        validator.validate(missing_handoff)
    unexpected_handoff = json.loads(json.dumps(report))
    unexpected_handoff['status'] = 'READY_FOR_UPSTREAM_REVIEW'
    with pytest.raises(jsonschema.ValidationError):
        validator.validate(unexpected_handoff)
    summary = CHECKER._summary(report)
    assert (
        'Draft PR handoff: rsasaki0109/ndt_omp_ros2:'
        'lidarslam-priors-and-correspondence-diagnostics -> '
        'koide3/ndt_omp:master'
    ) in summary
    assert f'Exact candidate: {candidate_commit}' in summary
    assert 'authorized by this report: no' in summary
    rendered = json.dumps(report)
    assert str(upstream) not in rendered
    assert str(candidate) not in rendered


@pytest.mark.parametrize(
    ('failure', 'check_id'),
    (
        ('commit', 'candidate-checkout-commit'),
        ('parent', 'candidate-checkout-parent'),
        ('dirty', 'candidate-checkout-clean'),
        ('subject', 'candidate-checkout-subject'),
        ('patch', 'candidate-checkout-patch'),
    ),
)
def test_candidate_checkout_drift_blocks_draft_preflight(
    monkeypatch,
    tmp_path: Path,
    failure: str,
    check_id: str,
):
    candidate = tmp_path / 'candidate'
    candidate.mkdir()
    contract = _contract()
    expected_commit = contract['publication']['candidate_commit']
    expected_parent = contract['upstream']['base_commit']
    expected_subject = contract['publication']['candidate_subject']

    def fake_git_text(repo, *arguments):
        assert repo == candidate
        if arguments == ('rev-parse', 'HEAD'):
            return '0' * 40 if failure == 'commit' else expected_commit
        if arguments == ('show', '-s', '--format=%P', 'HEAD'):
            return '1' * 40 if failure == 'parent' else expected_parent
        if arguments == ('show', '-s', '--format=%s', 'HEAD'):
            return 'Unexpected subject' if failure == 'subject' else (
                expected_subject)
        assert arguments == (
            'status', '--porcelain', '--untracked-files=all')
        return ' M changed.cpp' if failure == 'dirty' else ''

    monkeypatch.setattr(CHECKER, '_git_text', fake_git_text)
    monkeypatch.setattr(
        CHECKER,
        '_git_diff_sha256',
        lambda *_args: (
            '2' * 64 if failure == 'patch'
            else contract['upstream']['patch']['sha256']
        ),
    )

    report = CHECKER.evaluate(candidate_checkout=candidate)

    assert report['status'] == 'BLOCKED'
    assert report['draft_pr_handoff'] is None
    checks = {item['id']: item for item in report['checks']}
    assert checks[check_id]['status'] == 'FAIL'


@pytest.mark.parametrize(
    ('override', 'check_id'),
    (
        ({'errors': ['HTTP 503']}, 'publication-remote-inspection'),
        ({'upstream_head_sha': '0' * 40}, 'publication-upstream-base'),
        ({'fork_is_expected': False}, 'publication-fork-identity'),
        ({'proposed_branch_present': True}, 'publication-branch-absent'),
        ({
            'duplicate_prs': [{
                'number': 88,
                'title': 'Add rotation prior',
                'url': 'https://github.com/koide3/ndt_omp/pull/88',
                'head_label': 'contributor:prior',
                'head_sha': 'b' * 40,
            }],
        }, 'publication-no-duplicate-pr'),
    ),
)
def test_publication_remote_failures_block(
    override: dict,
    check_id: str,
):
    report = CHECKER.evaluate(
        online=True,
        publication_state=_publication_state(**override),
    )

    assert report['status'] == 'BLOCKED'
    assert report['draft_pr_handoff'] is None
    checks = {item['id']: item for item in report['checks']}
    assert checks[check_id]['status'] == 'FAIL'


def test_publication_inspection_detects_semantic_duplicate(monkeypatch):
    contract = _contract()
    base = contract['upstream']['base_commit']
    calls = []

    def fake_request(url, *, allow_404=False):
        calls.append((url, allow_404))
        if '/koide3/ndt_omp/git/ref/heads/master' in url:
            return 200, {'object': {'sha': base}}
        if url.endswith('/repos/rsasaki0109/ndt_omp_ros2'):
            return 200, {
                'fork': True,
                'parent': {'full_name': 'koide3/ndt_omp'},
            }
        if '/rsasaki0109/ndt_omp_ros2/git/ref/heads/' in url:
            assert allow_404 is True
            return 404, None
        if '/koide3/ndt_omp/pulls?' in url:
            return 200, [
                {
                    'number': 77,
                    'title': 'Fix voxel covariance',
                    'body': '',
                    'html_url': 'https://github.com/koide3/ndt_omp/pull/77',
                    'head': {
                        'ref': 'covariance',
                        'label': 'user:covariance',
                        'sha': '7' * 40,
                        'repo': {'full_name': 'user/ndt_omp'},
                    },
                },
                {
                    'number': 88,
                    'title': 'Add translation prior support',
                    'body': 'Optional API work',
                    'html_url': 'https://github.com/koide3/ndt_omp/pull/88',
                    'head': {
                        'ref': 'translation-prior',
                        'label': 'user:translation-prior',
                        'sha': '8' * 40,
                        'repo': {'full_name': 'user/ndt_omp'},
                    },
                },
                {
                    'number': 89,
                    'title': 'Unrelated title',
                    'body': '',
                    'html_url': 'https://github.com/koide3/ndt_omp/pull/89',
                    'head': {
                        'ref': contract['publication']['proposed_branch'],
                        'label': (
                            'rsasaki0109:'
                            f'{contract["publication"]["proposed_branch"]}'
                        ),
                        'sha': '9' * 40,
                        'repo': {
                            'full_name': contract['publication'][
                                'fork_repository'],
                        },
                    },
                },
            ]
        pytest.fail(f'unexpected GitHub URL: {url}')

    monkeypatch.setattr(CHECKER, '_request_json', fake_request)

    state = CHECKER._inspect_publication_state(contract)

    assert state['errors'] == []
    assert state['upstream_head_sha'] == base
    assert state['fork_is_expected'] is True
    assert state['proposed_branch_present'] is False
    assert state['open_pr_count'] == 3
    assert [item['number'] for item in state['duplicate_prs']] == [88, 89]
    assert len(calls) == 4


def test_publication_token_is_scoped_to_github_api(monkeypatch):
    requests = []

    class FakeResponse:
        status = 200

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self):
            return b'{}'

    def fake_urlopen(request, timeout):
        assert timeout == 20
        requests.append(request)
        return FakeResponse()

    monkeypatch.setenv('GITHUB_TOKEN', 'read-only-test-token')
    monkeypatch.setattr(CHECKER.urllib.request, 'urlopen', fake_urlopen)

    status, payload = CHECKER._request_json(
        'https://api.github.com/repos/koide3/ndt_omp')

    assert status == 200
    assert payload == {}
    assert requests[0].get_header('Authorization') == (
        'Bearer read-only-test-token'
    )
    with pytest.raises(CHECKER.ConvergenceError, match='non-GitHub API URL'):
        CHECKER._request_json('https://example.com/not-allowed')
    with pytest.raises(CHECKER.ConvergenceError, match='non-GitHub API URL'):
        CHECKER._request_json(
            'https://api.github.com.evil.example/repos/koide3/ndt_omp'
        )


@pytest.mark.parametrize('failure', ('commit', 'dirty', 'apply'))
def test_upstream_checkout_failures_block_review(
    monkeypatch,
    tmp_path: Path,
    failure: str,
):
    checkout = tmp_path / 'upstream'
    checkout.mkdir()
    expected = _contract()['upstream']['base_commit']
    real_apply = CHECKER._apply_check

    def fake_git_text(repo, *arguments):
        assert repo == checkout
        if arguments == ('rev-parse', 'HEAD'):
            return '0' * 40 if failure == 'commit' else expected
        return ' M changed.cpp' if failure == 'dirty' else ''

    def fake_apply(repo, patch):
        if repo == checkout:
            return (
                (False, 'injected apply failure') if failure == 'apply'
                else (True, 'patch applies without modifying the checkout')
            )
        return real_apply(repo, patch)

    monkeypatch.setattr(CHECKER, '_git_text', fake_git_text)
    monkeypatch.setattr(CHECKER, '_apply_check', fake_apply)

    report = CHECKER.evaluate(upstream_checkout=checkout)

    assert report['status'] == 'BLOCKED'
    assert any(item['status'] == 'FAIL' for item in report['checks'])


def test_artifact_hash_drift_fails_closed(tmp_path: Path):
    payload = _contract()
    payload['upstream']['patch']['sha256'] = '0' * 64

    report = CHECKER.evaluate(contract_path=_write_contract(tmp_path, payload))

    assert report['status'] == 'BLOCKED'
    checks = {item['id']: item for item in report['checks']}
    assert checks['upstream-patch-sha256']['status'] == 'FAIL'


def test_consumer_inventory_drift_fails_closed(tmp_path: Path):
    payload = _contract()
    payload['parent_transition']['consumer_replacements'][0][
        'before_count'] = 6

    report = CHECKER.evaluate(contract_path=_write_contract(tmp_path, payload))

    assert report['status'] == 'BLOCKED'
    checks = {item['id']: item for item in report['checks']}
    assert checks['consumer-graph_based_slam/CMakeLists.txt']['status'] == (
        'FAIL')


def test_remote_write_authority_is_rejected_by_schema(tmp_path: Path):
    payload = _contract()
    payload['authority']['github_writes_authorized'] = True

    with pytest.raises(
        CHECKER.ConvergenceError,
        match='authority.github_writes_authorized',
    ):
        CHECKER.evaluate(contract_path=_write_contract(tmp_path, payload))


def test_draft_pr_copy_drift_is_rejected_by_schema(tmp_path: Path):
    payload = _contract()
    payload['publication']['draft_pr']['title'] = 'Unbound title'

    with pytest.raises(
        CHECKER.ConvergenceError,
        match='publication.draft_pr.title',
    ):
        CHECKER.evaluate(contract_path=_write_contract(tmp_path, payload))


def test_cli_json_is_path_private_and_strict_mode_requires_checkout():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--json',
            '--require-ready-for-upstream-review',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    report = json.loads(result.stdout)
    assert report['status'] == 'ARTIFACTS_READY'
    assert '/home/' not in result.stdout
    assert '/tmp/' not in result.stdout
    assert result.stderr == ''


def test_output_file_is_create_only(tmp_path: Path):
    output = tmp_path / 'readiness.json'
    first = CHECKER.main(['--json', '--output-json', str(output)])
    before = output.read_bytes()
    second = CHECKER.main(['--json', '--output-json', str(output)])

    assert first == 0
    assert second == 2
    assert output.read_bytes() == before
