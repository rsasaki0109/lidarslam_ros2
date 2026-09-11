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

"""Tests for the read-only ndt_omp_ros2 initial-release preflight."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_ndt_omp_release_readiness.py'
SPEC = importlib.util.spec_from_file_location('ndt_release_readiness', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
PREFLIGHT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREFLIGHT)


def _local(ready: bool = True):
    return {
        'ready': ready,
        'gitlink_commit': PREFLIGHT.EXPECTED_COMMIT,
        'head_commit': PREFLIGHT.EXPECTED_COMMIT,
        'package_version': PREFLIGHT.EXPECTED_VERSION,
        'checks': [{
            'id': 'fixture',
            'status': 'PASS' if ready else 'FAIL',
            'detail': 'fixture local candidate state',
        }],
    }


def _pull_request(
    distro: str,
    *,
    response_pending: bool = False,
    state: str = 'open',
    merged: bool = False,
    mergeable: bool | None = True,
    check_state: str = 'passing',
):
    number = PREFLIGHT.ROSDISTRO_PULL_REQUESTS[distro]
    review_url = (
        f'https://github.com/ros/rosdistro/pull/{number}'
        '#pullrequestreview-1'
    ) if response_pending else None
    if check_state == 'missing':
        check_runs = {
            'inspected': True,
            'total_count': 0,
            'passing_count': 0,
            'pending_count': 0,
            'failing_count': 0,
            'runs': [],
        }
    else:
        classification = check_state.upper()
        status = 'completed' if check_state != 'pending' else 'in_progress'
        conclusion = (
            'success' if check_state == 'passing'
            else 'failure' if check_state == 'failing'
            else None
        )
        check_runs = {
            'inspected': True,
            'total_count': 1,
            'passing_count': int(check_state == 'passing'),
            'pending_count': int(check_state == 'pending'),
            'failing_count': int(check_state == 'failing'),
            'runs': [{
                'name': 'fixture check',
                'status': status,
                'conclusion': conclusion,
                'details_url': (
                    'https://github.com/ros/rosdistro/actions/runs/1'
                ),
                'classification': classification,
            }],
        }
    return {
        'number': number,
        'url': f'https://github.com/ros/rosdistro/pull/{number}',
        'state': state,
        'merged': merged,
        'mergeable': mergeable if state == 'open' else None,
        'head_sha': 'a' * 40,
        'updated_at': '2026-08-12T00:00:00Z',
        'latest_actionable_review': {
            'url': review_url,
            'author': 'reviewer' if response_pending else None,
            'created_at': (
                '2026-08-04T00:00:00Z' if response_pending else None
            ),
        },
        'response_pending': response_pending,
        'check_runs': check_runs,
    }


def _remote(
    *,
    tag: bool,
    release_repo: bool,
    humble: bool,
    jazzy: bool,
    pending: tuple[str, ...] = (),
    closed: tuple[str, ...] = (),
    unmergeable: tuple[str, ...] = (),
    unknown_mergeability: tuple[str, ...] = (),
    failing_checks: tuple[str, ...] = (),
    pending_checks: tuple[str, ...] = (),
    missing_checks: tuple[str, ...] = (),
):
    return {
        'errors': [],
        'origin_branch_commit': PREFLIGHT.EXPECTED_COMMIT,
        'source_tag_present': tag,
        'release_repository_present': release_repo,
        'rosdistro': {'humble': humble, 'jazzy': jazzy},
        'pull_requests': {
            distro: _pull_request(
                distro,
                response_pending=distro in pending,
                state='closed' if distro in closed else 'open',
                mergeable=(
                    False if distro in unmergeable else
                    None if distro in unknown_mergeability else True
                ),
                check_state=(
                    'failing' if distro in failing_checks else
                    'pending' if distro in pending_checks else
                    'missing' if distro in missing_checks else 'passing'
                ),
            )
            for distro in PREFLIGHT.DISTROS
        },
    }


def _response_report():
    remote = _remote(
        tag=True,
        release_repo=True,
        humble=False,
        jazzy=False,
        pending=('humble', 'jazzy'),
        failing_checks=('humble', 'jazzy'),
    )
    for distro in PREFLIGHT.DISTROS:
        pull_request = remote['pull_requests'][distro]
        pull_request['head_sha'] = (
            PREFLIGHT.ROSDISTRO_PULL_REQUEST_HEADS[distro])
        pull_request['latest_actionable_review']['url'] = (
            PREFLIGHT.ROSDISTRO_REVIEW_URLS[distro])
    return PREFLIGHT.evaluate_readiness(local=_local(), remote=remote)


def _upstream_pull_request():
    return {
        'number': 72,
        'url': 'https://github.com/koide3/ndt_omp/pull/72',
        'state': 'open',
        'draft': True,
        'merged': False,
        'head_sha': PREFLIGHT.UPSTREAM_CANDIDATE_COMMIT,
        'head_repository': PREFLIGHT.UPSTREAM_FORK_REPOSITORY,
        'base_branch': PREFLIGHT.UPSTREAM_BASE_BRANCH,
        'base_repository': PREFLIGHT.UPSTREAM_REPOSITORY,
    }


def test_tracked_candidate_is_locally_ready_and_schema_valid():
    report = PREFLIGHT.evaluate_readiness(offline=True)

    assert report['status'] == 'LOCAL_READY'
    assert report['local']['ready'] is True
    assert report['local']['gitlink_commit'] == PREFLIGHT.EXPECTED_COMMIT
    assert report['local']['head_commit'] == PREFLIGHT.EXPECTED_COMMIT
    assert report['local']['package_version'] == '0.1.0'
    assert report['schema_version'] == 2
    assert all(
        item['status'] == 'PASS' for item in report['local']['checks'])


def test_initially_absent_remote_artifacts_are_ready_to_tag():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=False,
            release_repo=False,
            humble=False,
            jazzy=False,
        ),
    )

    assert report['status'] == 'READY_TO_TAG'


def test_partial_publication_is_in_progress():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=True,
            jazzy=False,
        ),
    )

    assert report['status'] == 'IN_PROGRESS'
    assert any(
        'PR #52950 (jazzy)' in action
        and 'do not recreate the source tag or rerun Bloom' in action
        for action in report['actions']
    )
    assert not any(
        action.startswith('Bloom-release') for action in report['actions'])


def test_generated_prs_are_waited_on_without_repeating_bloom():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
        ),
    )

    assert report['status'] == 'IN_PROGRESS'
    assert len(report['actions']) == 2
    assert 'PR #52949 (humble)' in report['actions'][0]
    assert 'PR #52950 (jazzy)' in report['actions'][1]
    assert all('rerun Bloom' in action for action in report['actions'])


def test_unanswered_human_review_requires_response_instead_of_waiting():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            pending=('humble', 'jazzy'),
        ),
    )

    assert report['status'] == 'REVIEW_REQUIRED'
    assert len(report['actions']) == 2
    assert all('unanswered human review' in item for item in report['actions'])
    assert all('upstream lineage' in item for item in report['actions'])
    assert all('collision-free convergence plan' in item
               for item in report['actions'])
    assert not any('Wait for' in item for item in report['actions'])


def test_failed_check_run_blocks_and_preserves_review_action():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            pending=('jazzy',),
            failing_checks=('humble',),
        ),
    )

    assert report['status'] == 'BLOCKED'
    assert any(
        'PR #52949 (humble) has 1 failing check run at exact head' in item
        and 'fixture check' in item
        and 'require all checks to pass' in item
        for item in report['actions']
    )
    assert any(
        'unanswered human review' in item and 'PR #52950 (jazzy)' in item
        for item in report['actions']
    )


def test_pending_or_missing_check_run_evidence_fails_closed():
    pending_report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            pending_checks=('humble',),
        ),
    )
    missing_report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            missing_checks=('jazzy',),
        ),
    )

    assert pending_report['status'] == 'BLOCKED'
    assert any('has 1 pending check run' in item
               for item in pending_report['actions'])
    assert missing_report['status'] == 'BLOCKED'
    assert any('has no check-run evidence' in item
               for item in missing_report['actions'])


def test_inconsistent_check_run_counts_fail_closed():
    remote = _remote(
        tag=True,
        release_repo=True,
        humble=False,
        jazzy=False,
    )
    remote['pull_requests']['humble']['check_runs']['total_count'] = 2

    report = PREFLIGHT.evaluate_readiness(local=_local(), remote=remote)

    assert report['status'] == 'BLOCKED'
    assert any('Resolve remote inspection failures' in item
               for item in report['actions'])


def test_closed_unmerged_generated_pr_blocks_replacement_release_state():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            closed=('jazzy',),
        ),
    )

    assert report['status'] == 'BLOCKED'
    assert any(
        'PR #52950 (jazzy) closed without merge' in item
        for item in report['actions']
    )


def test_unmergeable_generated_pr_blocks_and_preserves_review_action():
    """An explicit base conflict blocks without hiding review work."""
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            pending=('humble',),
            unmergeable=('jazzy',),
        ),
    )

    assert report['status'] == 'BLOCKED'
    assert any(
        'PR #52950 (jazzy) is not mergeable at exact head' in item
        and 'Resolve the base conflict' in item
        for item in report['actions']
    )
    assert any(
        'unanswered human review' in item and 'PR #52949 (humble)' in item
        for item in report['actions']
    )


def test_unknown_mergeability_fails_closed_instead_of_waiting():
    """An unresolved GitHub calculation cannot become a wait-only result."""
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            unknown_mergeability=('humble',),
        ),
    )

    assert report['status'] == 'BLOCKED'
    assert any(
        'has not resolved mergeability' in item
        and 'PR #52949 (humble)' in item
        for item in report['actions']
    )
    assert not any('Wait for' in item for item in report['actions'])


def test_complete_publication_is_released():
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=True,
            jazzy=True,
        ),
    )

    assert report['status'] == 'RELEASED'


def test_remote_error_and_branch_drift_fail_closed():
    failed_query = _remote(
        tag=False,
        release_repo=False,
        humble=False,
        jazzy=False,
    )
    failed_query['errors'] = ['GitHub returned HTTP 503']
    drifted = _remote(
        tag=False,
        release_repo=False,
        humble=False,
        jazzy=False,
    )
    drifted['origin_branch_commit'] = '0' * 40

    assert PREFLIGHT.evaluate_readiness(
        local=_local(), remote=failed_query)['status'] == 'BLOCKED'
    assert PREFLIGHT.evaluate_readiness(
        local=_local(), remote=drifted)['status'] == 'BLOCKED'


def test_explicit_404_is_absent_without_hiding_other_remote_state(monkeypatch):
    def fake_request(url):
        if url.endswith('/git/ref/heads/humble'):
            return 200, json.dumps({
                'object': {'sha': PREFLIGHT.EXPECTED_COMMIT},
            })
        if '/git/ref/tags/' in url or url.endswith(
                '/ndt_omp_ros2-release'):
            return 404, ''
        return 200, 'repositories:\n  another_package:\n'

    monkeypatch.setattr(PREFLIGHT, '_request_text', fake_request)

    remote = PREFLIGHT.inspect_remote()
    report = PREFLIGHT.evaluate_readiness(local=_local(), remote=remote)

    assert remote['errors'] == []
    assert remote['source_tag_present'] is False
    assert remote['release_repository_present'] is False
    assert remote['rosdistro'] == {'humble': False, 'jazzy': False}
    assert all(
        item['state'] is None
        for item in remote['pull_requests'].values()
    )
    assert report['status'] == 'READY_TO_TAG'


def test_pull_request_inspection_detects_question_and_author_response(
        monkeypatch):
    author_replied = False

    def fake_request_json(url):
        if url.endswith('/pulls/52950'):
            return {
                'user': {'login': 'rsasaki0109', 'type': 'User'},
                'state': 'open',
                'merged': False,
                'mergeable': True,
                'head': {'sha': 'b' * 40},
                'updated_at': '2026-08-07T00:00:00Z',
                'html_url': 'https://github.com/ros/rosdistro/pull/52950',
            }
        if '/reviews?' in url:
            return [{
                'user': {'login': 'reviewer', 'type': 'User'},
                'body': 'How does this relate to ndt_omp?',
                'state': 'COMMENTED',
                'submitted_at': '2026-08-04T00:00:00Z',
                'html_url': (
                    'https://github.com/ros/rosdistro/pull/52950'
                    '#pullrequestreview-1'
                ),
            }]
        if '/check-runs?' in url:
            return {
                'total_count': 1,
                'check_runs': [{
                    'name': 'rosdistro / rosdep checks (3.8)',
                    'status': 'completed',
                    'conclusion': 'success',
                    'details_url': (
                        'https://github.com/ros/rosdistro/actions/runs/1'
                    ),
                }],
            }
        if '/issues/' in url and author_replied:
            return [{
                'user': {'login': 'rsasaki0109', 'type': 'User'},
                'body': 'Thanks; here is the convergence plan.',
                'created_at': '2026-08-05T00:00:00Z',
                'html_url': (
                    'https://github.com/ros/rosdistro/pull/52950'
                    '#issuecomment-1'
                ),
            }]
        return []

    monkeypatch.setattr(PREFLIGHT, '_request_json', fake_request_json)

    pending = PREFLIGHT._inspect_pull_request('jazzy')
    assert pending['response_pending'] is True
    assert pending['latest_actionable_review']['author'] == 'reviewer'

    author_replied = True
    answered = PREFLIGHT._inspect_pull_request('jazzy')
    assert answered['response_pending'] is False


def test_check_run_inspection_classifies_and_rejects_truncation(monkeypatch):
    payload = {
        'total_count': 4,
        'check_runs': [
            {
                'name': 'success',
                'status': 'completed',
                'conclusion': 'success',
                'details_url': None,
            },
            {
                'name': 'neutral',
                'status': 'completed',
                'conclusion': 'neutral',
                'details_url': None,
            },
            {
                'name': 'failure',
                'status': 'completed',
                'conclusion': 'failure',
                'details_url': None,
            },
            {
                'name': 'running',
                'status': 'in_progress',
                'conclusion': None,
                'details_url': None,
            },
        ],
    }
    monkeypatch.setattr(PREFLIGHT, '_request_json', lambda _url: payload)

    result = PREFLIGHT._inspect_check_runs(
        'https://api.github.com/repos/ros/rosdistro', 'a' * 40, 52950)

    assert result['passing_count'] == 2
    assert result['pending_count'] == 1
    assert result['failing_count'] == 1

    payload['total_count'] = 5
    try:
        PREFLIGHT._inspect_check_runs(
            'https://api.github.com/repos/ros/rosdistro', 'a' * 40, 52950)
    except PREFLIGHT.PreflightError as exc:
        assert 'truncated' in str(exc)
    else:
        raise AssertionError('truncated check-run evidence was accepted')


def test_github_token_is_used_only_for_github_api(monkeypatch):
    """The optional token is scoped to GitHub API requests."""
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
    monkeypatch.setattr(PREFLIGHT.urllib.request, 'urlopen', fake_urlopen)

    PREFLIGHT._request_text('https://api.github.com/repos/owner/repo')
    PREFLIGHT._request_text(
        'https://raw.githubusercontent.com/owner/repo/main/file')

    assert requests[0].get_header('Authorization') == (
        'Bearer read-only-test-token'
    )
    assert requests[1].get_header('Authorization') is None


def test_human_summary_exposes_mergeability():
    """The readable report exposes mergeability beside review state."""
    report = PREFLIGHT.evaluate_readiness(
        local=_local(),
        remote=_remote(
            tag=True,
            release_repo=True,
            humble=False,
            jazzy=False,
            pending=('humble', 'jazzy'),
        ),
    )

    summary = PREFLIGHT._summary(report)
    assert 'state=open merged=False mergeable=True response_pending=True' in (
        summary
    )
    assert 'checks=1/1 passing pending=0 failing=0' in summary


def test_review_response_packet_unlocks_only_exact_verified_targets():
    report = _response_report()
    upstream = _upstream_pull_request()

    packet = PREFLIGHT.build_review_response_packet(
        report,
        upstream['url'],
        upstream_pull_request=upstream,
    )

    assert report['status'] == 'BLOCKED'
    assert packet['status'] == 'READY_FOR_MAINTAINER_POST'
    assert packet['blockers'] == []
    assert packet['authority'] == {
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    for distro in PREFLIGHT.DISTROS:
        target = packet['rosdistro_pull_requests'][distro]
        assert target['head_sha'] == (
            PREFLIGHT.ROSDISTRO_PULL_REQUEST_HEADS[distro])
        assert target['review_url'] == PREFLIGHT.ROSDISTRO_REVIEW_URLS[distro]
        assert upstream['url'] in target['response']
        assert '<UPSTREAM_PR_URL>' not in target['response']
    summary = PREFLIGHT._review_response_summary(packet)
    assert 'READY_FOR_MAINTAINER_POST' in summary
    assert 'posting remains a separate maintainer action' in summary


def test_review_response_packet_emits_no_body_while_blocked():
    packet = PREFLIGHT.build_review_response_packet(
        _response_report(),
        None,
    )

    assert packet['status'] == 'BLOCKED'
    assert any('Draft PR URL is required' in item
               for item in packet['blockers'])
    assert all(
        target['response'] is None
        for target in packet['rosdistro_pull_requests'].values()
    )
    assert 'No copy-ready response was emitted.' in (
        PREFLIGHT._review_response_summary(packet))


def test_review_response_packet_rejects_remote_or_upstream_drift():
    report = _response_report()
    report['remote']['pull_requests']['humble']['head_sha'] = '0' * 40
    upstream = _upstream_pull_request()
    upstream['draft'] = False
    upstream['head_sha'] = 'f' * 40

    packet = PREFLIGHT.build_review_response_packet(
        report,
        upstream['url'],
        upstream_pull_request=upstream,
    )

    assert packet['status'] == 'BLOCKED'
    assert any('humble rosdistro PR identity' in item
               for item in packet['blockers'])
    assert any('not the expected open Draft' in item
               for item in packet['blockers'])
    assert all(
        target['response'] is None
        for target in packet['rosdistro_pull_requests'].values()
    )


def test_upstream_pull_request_inspection_binds_repository_and_commit(
        monkeypatch):
    upstream = _upstream_pull_request()

    def fake_request_json(url):
        assert url.endswith('/repos/koide3/ndt_omp/pulls/72')
        return {
            'html_url': upstream['url'],
            'state': upstream['state'],
            'draft': upstream['draft'],
            'merged': upstream['merged'],
            'head': {
                'sha': upstream['head_sha'],
                'repo': {'full_name': upstream['head_repository']},
            },
            'base': {
                'ref': upstream['base_branch'],
                'repo': {'full_name': upstream['base_repository']},
            },
        }

    monkeypatch.setattr(PREFLIGHT, '_request_json', fake_request_json)

    assert PREFLIGHT._inspect_upstream_pull_request(upstream['url']) == (
        upstream)
    try:
        PREFLIGHT._inspect_upstream_pull_request(
            'https://github.com/another/repo/pull/72')
    except PREFLIGHT.PreflightError as exc:
        assert 'upstream PR URL must match' in str(exc)
    else:
        raise AssertionError('an unrelated upstream PR URL was accepted')


def test_missing_candidate_path_fails_closed(tmp_path):
    report = PREFLIGHT.evaluate_readiness(
        repo_root=tmp_path,
        offline=True,
    )

    assert report['status'] == 'BLOCKED'
    assert report['local']['ready'] is False


def test_offline_strict_gate_refuses_ready_to_tag(tmp_path):
    output = tmp_path / 'report.json'
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--offline',
            '--require-ready-to-tag',
            '--json',
            '--output-json',
            str(output),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    assert json.loads(result.stdout)['status'] == 'LOCAL_READY'
    assert json.loads(output.read_text(encoding='utf-8'))['status'] == (
        'LOCAL_READY'
    )


def test_strict_review_response_gate_fails_without_verified_upstream_pr():
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--offline',
            '--require-review-response-ready',
            '--json',
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    packet = json.loads(result.stdout)
    assert packet['status'] == 'BLOCKED'
    assert all(
        target['response'] is None
        for target in packet['rosdistro_pull_requests'].values()
    )
