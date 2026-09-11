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

"""Tests for the live package-manager release audit."""

from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_package_manager_release_readiness.py'
SPEC = importlib.util.spec_from_file_location(
    'package_manager_release_readiness',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
READINESS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(READINESS)


def _successful_run() -> dict:
    return {
        'id': 123,
        'html_url': 'https://github.com/example/actions/runs/123',
        'display_title': READINESS.expected_run_name('0.9.0'),
        'event': 'workflow_dispatch',
        'status': 'completed',
        'conclusion': 'success',
        'workflow_path': (
            '.github/workflows/package-manager-install-upgrade.yml'
        ),
        'head_sha': 'a' * 40,
        'jobs': [
            {
                'name': 'humble clean-install',
                'status': 'completed',
                'conclusion': 'success',
            },
            {
                'name': 'jazzy clean-install',
                'status': 'completed',
                'conclusion': 'success',
            },
        ],
    }


def _snapshot(*runs: dict, source_sha: str = 'a' * 40) -> dict:
    return {
        'inspected': True,
        'errors': [],
        'source_ref': {
            'ref': 'v0.9.0',
            'resolved': True,
            'commit_sha': source_sha,
        },
        'runs': list(runs),
    }


def test_exact_successful_main_channel_matrix_is_ready():
    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot=_snapshot(_successful_run()),
    )

    assert report['status'] == 'READY'
    assert report['selected_run']['id'] == 123
    assert report['actions'] == []
    assert all(check['status'] == 'PASS' for check in report['checks'])


def test_no_matching_run_is_not_run():
    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot=_snapshot(),
    )

    assert report['status'] == 'NOT_RUN'
    assert report['selected_run'] is None
    assert any(
        'gh workflow run package-manager-install-upgrade.yml'
        in action
        for action in report['actions']
    )


def test_api_failure_is_blocked_not_not_run():
    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot={
            'inspected': False,
            'errors': ['rate limited'],
            'source_ref': {
                'ref': 'v0.9.0',
                'resolved': False,
                'commit_sha': None,
            },
            'runs': [],
        },
    )

    assert report['status'] == 'BLOCKED'
    assert report['remote']['inspected'] is False


def test_github_token_is_scoped_to_api_requests(monkeypatch):
    """The optional token is sent only to bounded GitHub API requests."""
    requests = []

    class FakeHeaders:
        def get(self, _name):
            return None

    class FakeResponse:
        status = 200
        headers = FakeHeaders()

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self, _limit):
            return b'{}'

    def fake_urlopen(request, timeout):
        assert timeout == 20
        requests.append(request)
        return FakeResponse()

    monkeypatch.setenv('GITHUB_TOKEN', 'read-only-test-token')
    monkeypatch.setattr(READINESS.urllib.request, 'urlopen', fake_urlopen)

    READINESS._request_json('https://api.github.com/repos/owner/repo')
    READINESS._request_json(
        'https://raw.githubusercontent.com/owner/repo/main/file')

    assert requests[0].get_header('Authorization') == (
        'Bearer read-only-test-token'
    )
    assert requests[1].get_header('Authorization') is None


def test_both_named_distros_are_required():
    run = _successful_run()
    run['jobs'][1]['conclusion'] = 'failure'

    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot=_snapshot(run),
    )

    assert report['status'] == 'FAILED'
    failed = {
        check['id']
        for check in report['checks']
        if check['status'] == 'FAIL'
    }
    assert failed == {'jazzy-main-clean-install'}


def test_missing_source_ref_blocks_dispatch_instead_of_claiming_not_run():
    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot={
            'inspected': True,
            'errors': [],
            'source_ref': {
                'ref': 'v0.9.0',
                'resolved': False,
                'commit_sha': None,
            },
            'runs': [],
        },
    )

    assert report['status'] == 'SOURCE_REF_MISSING'
    assert report['source_ref']['resolved'] is False
    assert 'Do not dispatch' in report['actions'][0]
    assert 'gh workflow run' not in ' '.join(report['actions'])


def test_run_head_must_match_the_current_immutable_tag_commit():
    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot=_snapshot(_successful_run(), source_sha='b' * 40),
    )

    assert report['status'] == 'BLOCKED'
    failed = {
        check['id']
        for check in report['checks']
        if check['status'] == 'FAIL'
    }
    assert failed == {'workflow-head-identity'}


def test_in_progress_exact_run_is_running_not_not_run():
    run = _successful_run()
    run['status'] = 'in_progress'
    run['conclusion'] = None
    run['jobs'][0]['status'] = 'in_progress'
    run['jobs'][0]['conclusion'] = None

    report = READINESS.evaluate_readiness(
        version='0.9.0',
        snapshot=_snapshot(run),
    )

    assert report['status'] == 'RUNNING'
    assert report['selected_run'] is None
    assert report['actions'] == [
        'Wait for the exact package-manager run to finish: '
        'https://github.com/example/actions/runs/123'
    ]


def test_remote_inspection_resolves_tag_and_does_not_hide_failed_runs(
    monkeypatch,
):
    urls = []

    def fake_request(url, *, allow_not_found=False):
        urls.append((url, allow_not_found))
        if '/git/ref/tags/v0.9.0' in url:
            return {
                'object': {
                    'type': 'commit',
                    'sha': 'a' * 40,
                },
            }
        if '/runs?' in url:
            return {'workflow_runs': []}
        raise AssertionError(url)

    monkeypatch.setattr(READINESS, '_request_json', fake_request)

    snapshot = READINESS.inspect_remote('0.9.0')

    assert snapshot['source_ref'] == {
        'ref': 'v0.9.0',
        'resolved': True,
        'commit_sha': 'a' * 40,
    }
    runs_url = next(url for url, _allowed in urls if '/runs?' in url)
    assert 'event=workflow_dispatch' in runs_url
    assert 'status=' not in runs_url


def test_annotated_source_tag_is_peeled_to_its_commit(monkeypatch):
    tag_object_url = (
        'https://api.github.com/repos/rsasaki0109/lidar_slam_ros2/'
        'git/tags/' + 'b' * 40
    )

    def fake_request(url, *, allow_not_found=False):
        if '/git/ref/tags/v0.9.0' in url:
            assert allow_not_found is True
            return {
                'object': {
                    'type': 'tag',
                    'sha': 'b' * 40,
                    'url': tag_object_url,
                },
            }
        if url == tag_object_url:
            return {
                'object': {
                    'type': 'commit',
                    'sha': 'a' * 40,
                },
            }
        raise AssertionError(url)

    monkeypatch.setattr(READINESS, '_request_json', fake_request)

    assert READINESS._resolve_source_ref('0.9.0') == {
        'ref': 'v0.9.0',
        'resolved': True,
        'commit_sha': 'a' * 40,
    }


def test_workflow_run_name_exposes_exact_audit_identity():
    workflow = (
        ROOT / '.github' / 'workflows'
        / 'package-manager-install-upgrade.yml'
    ).read_text(encoding='utf-8')

    assert (
        'run-name: package-manager / ${{ inputs.source_ref }} / '
        '${{ inputs.target_version }} / ${{ inputs.target_channel }} / '
        '${{ inputs.mode }}'
    ) in workflow
