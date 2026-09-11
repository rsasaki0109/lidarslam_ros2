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

"""Regression tests for the bounded local contributor starter queue."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import jsonschema

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'contributor_starter_queue.py'
QUEUE_PATH = (
    ROOT / 'docs' / 'contracts' / 'contributor-starter-queue-v1.json'
)
SCHEMA_PATH = (
    ROOT / 'docs' / 'schemas' / 'contributor-starter-queue-v1.schema.json'
)
SPEC = importlib.util.spec_from_file_location(
    'contributor_starter_queue', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)


def _load(path: Path) -> dict:
    payload = json.loads(path.read_text(encoding='utf-8'))
    assert isinstance(payload, dict)
    return payload


def _queue() -> dict:
    return copy.deepcopy(_load(QUEUE_PATH))


def _schema() -> dict:
    return copy.deepcopy(_load(SCHEMA_PATH))


def _task(payload: dict, task_id: str) -> dict:
    return next(item for item in payload['tasks'] if item['id'] == task_id)


def _github_issue(
    number: int,
    title: str,
    *,
    labels: tuple[str, ...] = (),
) -> dict:
    return {
        'number': number,
        'title': title,
        'html_url': (
            f'https://github.com/rsasaki0109/lidar_slam_ros2/issues/{number}'
        ),
        'labels': [{'name': item} for item in labels],
    }


def _github_pull(
    number: int,
    title: str,
    *,
    body: str = '',
    updated_at: str = '2026-08-15T22:14:44Z',
) -> dict:
    return {
        'number': number,
        'title': title,
        'body': body,
        'updated_at': updated_at,
        'html_url': (
            f'https://github.com/rsasaki0109/lidar_slam_ros2/pull/{number}'
        ),
    }


def _publication_gate(
    *,
    status: str = 'WAITING_FOR_PUBLIC_GATES',
    eligible: bool = False,
) -> dict:
    return {
        'id': 'first-map-validator-cohort-v1',
        'status': status,
        'eligible': eligible,
        'blocking_reasons': (
            [] if eligible else ['comparable_docker_row']
        ),
        'check_command': [
            'python3',
            'scripts/first_map_validator_cohort.py',
            '--json',
        ],
        'remote_mutations_performed': False,
    }


def _local_task_gate(
    *,
    status: str = 'WAITING_FOR_PRODUCT_MERGE',
    public_queue_status: str = 'ABSENT',
) -> dict:
    queue = _queue()
    eligible = status == 'READY'
    if status == 'READY':
        pull_state = 'CLOSED'
        merged = True
        draft = False
        blocking_reasons = []
        public_queue_status = 'MATCH'
    elif status == 'PRODUCT_PR_CLOSED_UNMERGED':
        pull_state = 'CLOSED'
        merged = False
        draft = False
        blocking_reasons = ['product_pr_not_merged']
    elif status == 'PUBLIC_QUEUE_ABSENT':
        pull_state = 'CLOSED'
        merged = True
        draft = False
        blocking_reasons = ['public_queue_absent']
        public_queue_status = 'ABSENT'
    elif status == 'PUBLIC_QUEUE_DRIFT':
        pull_state = 'CLOSED'
        merged = True
        draft = False
        blocking_reasons = ['public_queue_drift']
        public_queue_status = 'DRIFT'
    else:
        assert status == 'WAITING_FOR_PRODUCT_MERGE'
        pull_state = 'OPEN'
        merged = False
        draft = True
        blocking_reasons = ['product_pr_not_merged']
    if public_queue_status == 'ABSENT':
        if 'public_queue_absent' not in blocking_reasons:
            blocking_reasons.append('public_queue_absent')
        observed_queue_sha256 = None
    elif public_queue_status == 'MATCH':
        observed_queue_sha256 = CHECKER._canonical_sha256(queue)
    else:
        assert public_queue_status == 'DRIFT'
        if 'public_queue_drift' not in blocking_reasons:
            blocking_reasons.append('public_queue_drift')
        observed_queue_sha256 = '0' * 64
    return {
        'id': 'product-draft-public-queue-v1',
        'status': status,
        'eligible': eligible,
        'pull_request_number': 427,
        'pull_request_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/pull/427'
        ),
        'pull_request_state': pull_state,
        'is_draft': draft,
        'merged': merged,
        'head_sha': '4b2ab514a4f33b443e2c4283b3114d11a5e44e49',
        'target_branch': 'develop',
        'public_queue_path': (
            'docs/contracts/contributor-starter-queue-v1.json'
        ),
        'public_queue_status': public_queue_status,
        'expected_queue_sha256': CHECKER._canonical_sha256(queue),
        'observed_queue_sha256': observed_queue_sha256,
        'blocking_reasons': blocking_reasons,
        'recheck_command': CHECKER.LOCAL_TASK_PUBLICATION_RECHECK_COMMAND,
        'github_requests': 'GET_ONLY',
        'remote_mutations_performed': False,
    }


def _product_pull_record(
    *,
    state: str = 'open',
    draft: bool = True,
    merged: bool = False,
) -> dict:
    return {
        'number': 427,
        'state': state,
        'draft': draft,
        'merged': merged,
        'html_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/pull/427'
        ),
        'base': {
            'ref': 'develop',
            'repo': {'full_name': 'rsasaki0109/lidar_slam_ros2'},
        },
        'head': {
            'sha': '4b2ab514a4f33b443e2c4283b3114d11a5e44e49',
        },
    }


EXPECTED_READY_IDS = [f'starter-C{index}' for index in range(5, 10)]
EXPECTED_GAP_MARKERS = {
    'starter-C5': (
        '### 日本語のvalidation reportのfollow-up引き継ぎを再現可能にする'
    ),
    'starter-C6': '### Live and loop-optimized pose outputs',
    'starter-C7': (
        '### Mapping, loop closure, and relocalization are different'
    ),
    'starter-C8': '### Classic-path IMU readiness',
    'starter-C9': '### Long route stopped: find the limiting stage',
}


def _copy_scoped_files(payload: dict, destination: Path) -> None:
    paths = {
        path
        for task in payload['tasks']
        for path in task['allowed_paths']
    }
    for path in paths:
        target = destination / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((ROOT / path).read_bytes())


def test_checked_in_queue_is_schema_valid_and_ready_local_only():
    """The refreshed five-task generation is complete but unpublished."""
    queue, report = CHECKER.evaluate()

    jsonschema.Draft7Validator(_schema()).validate(queue)
    assert report['status'] == 'QUEUE_READY_LOCAL_ONLY'
    assert report['task_count'] == 5
    assert report['ready_task_ids'] == EXPECTED_READY_IDS
    assert report['stale_tasks'] == []
    assert report['remote_duplicate_audit'] == {
        'checked_at': '2026-08-16T07:21:20+09:00',
        'open_pull_request_count': 1,
        'matching_task_pull_requests': 0,
        'recheck_before_publication': True,
    }
    assert report['authority'] == {
        'github_writes_authorized': False,
        'issues_published': False,
        'remote_mutations_performed': False,
    }


def test_all_tasks_are_bounded_to_thirty_minutes_and_exact_paths():
    """Every task stays within the advertised beginner-sized boundary."""
    payload = _queue()

    assert [task['id'] for task in payload['tasks']] == [
        'starter-C5',
        'starter-C6',
        'starter-C7',
        'starter-C8',
        'starter-C9',
    ]
    assert max(task['estimate_minutes'] for task in payload['tasks']) == 30
    assert all(task['estimate_minutes'] <= 30 for task in payload['tasks'])
    assert all(task['allowed_paths'] == sorted(task['allowed_paths'])
               for task in payload['tasks'])
    assert all(not task['private_data_required'] for task in payload['tasks'])
    assert all(not task['hardware_required'] for task in payload['tasks'])


def test_rendered_task_is_copy_ready_but_keeps_coordination_gate():
    """Rendered issue text is complete and clearly remains unpublished."""
    payload = _queue()

    for task in payload['tasks']:
        body = CHECKER.render_task_markdown(task)
        assert body.startswith(f"# {task['title']}\n")
        assert 'prepared locally but is not a published GitHub issue' in body
        assert '## Allowed files' in body
        assert '## Acceptance' in body
        assert '## Non-goals' in body
        assert '## Focused checks' in body
        assert 'No private data or hardware is required.' in body
        for path in task['allowed_paths']:
            assert f'`{path}`' in body
        for label in task['labels']:
            assert f'`{label}`' in body


def test_arbitrary_command_cannot_replace_an_allowlisted_profile():
    """JSON data cannot inject a shell command into the verifier."""
    payload = _queue()
    task = _task(payload, 'starter-C5')
    task['focused_checks'][0]['argv'] = ['bash', '-c', 'touch unexpected']

    with pytest.raises(
        CHECKER.QueueError,
        match='focused checks do not match profile',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_publication_gate_command_cannot_be_replaced_by_queue_data():
    """The live card never executes an arbitrary dependency command."""
    payload = _queue()
    payload['published_issue_dependencies'][0]['check_command'] = [
        'bash',
        '-c',
        'touch unexpected',
    ]

    with pytest.raises(CHECKER.QueueError, match='schema validation failed'):
        CHECKER.validate_queue(payload, _schema())


def test_remote_write_authority_is_rejected_by_schema():
    """The local contract cannot grant itself GitHub write authority."""
    payload = _queue()
    payload['authority']['github_writes_authorized'] = True

    with pytest.raises(
        CHECKER.QueueError,
        match='authority.github_writes_authorized',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_open_pull_request_duplicate_fails_closed():
    """A matching open PR prevents the queue from reporting ready."""
    payload = _queue()
    payload['remote_duplicate_audit']['task_matches'][0][
        'matching_open_pull_requests'
    ] = [999]

    with pytest.raises(
        CHECKER.QueueError,
        match='open pull request duplicates require review',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_duplicate_audit_must_cover_every_task_in_order():
    """The remote duplicate audit cannot omit or shuffle a task."""
    payload = _queue()
    matches = payload['remote_duplicate_audit']['task_matches']
    matches[0], matches[1] = matches[1], matches[0]

    with pytest.raises(
        CHECKER.QueueError,
        match='duplicate audit must cover the ordered task set',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_next_report_blocks_a_published_issue_with_closed_product_gates():
    """A published label cannot bypass the first-map cohort state."""
    queue, local_report = CHECKER.evaluate()
    issue = _github_issue(
        422,
        'Help validate the public first-map path for v1.0',
        labels=('documentation', 'good first issue', 'help wanted'),
    )
    known_pull = _github_pull(
        427,
        'Prepare crash-safe guided mapping for G0 review',
        body='Japanese validation report follow-up handoff',
    )

    report = CHECKER.build_next_report(
        queue,
        local_report,
        [issue],
        [known_pull],
        [_publication_gate()],
        _local_task_gate(),
    )

    assert report['status'] == 'PUBLISHED_GOOD_FIRST_ISSUES_BLOCKED'
    assert report['contributor_next'] == {
        'action': 'WAIT_FOR_READY_PUBLISHED_STARTER',
        'url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/issues?'
            'q=is%3Aissue+is%3Aopen+label%3A%22good+first+issue%22'
        ),
    }
    assert report['maintainer_next'] == {
        'action': 'PREPARE_LOCAL_TASK_FOR_POST_MERGE',
        'task_id': 'starter-C5',
        'gate_status': 'WAITING_FOR_PRODUCT_MERGE',
        'preview_command': [
            'python3',
            'scripts/contributor_starter_queue.py',
            '--task',
            'starter-C5',
        ],
        'recheck_command': [
            'python3',
            'scripts/contributor_starter_queue.py',
            '--next',
            '--json',
        ],
    }
    handoff = report['maintainer_publication_handoff']
    task = _task(queue, 'starter-C5')
    assert handoff['kind'] == 'PREPARE_LOCAL_TASK_FOR_POST_MERGE'
    assert handoff['task_id'] == 'starter-C5'
    assert handoff['repository'] == 'rsasaki0109/lidar_slam_ros2'
    assert handoff['title'] == task['title']
    assert handoff['labels'] == sorted(task['labels'])
    rendered_task = CHECKER.render_task_markdown(task)
    assert handoff['issue_body'] == rendered_task.split('\n\n', 1)[1]
    assert not handoff['issue_body'].startswith('# ')
    assert handoff['title'] not in handoff['issue_body'].splitlines()[0]
    assert handoff['issue_body_sha256'] == CHECKER.hashlib.sha256(
        handoff['issue_body'].encode('utf-8')
    ).hexdigest()
    assert handoff['task_sha256'] == CHECKER._canonical_sha256(task)
    assert handoff['queue_sha256'] == CHECKER._canonical_sha256(queue)
    assert handoff['live_duplicate_count'] == 0
    assert handoff['publication_gate_status'] == (
        'WAITING_FOR_PRODUCT_MERGE'
    )
    assert handoff['public_base_ready'] is False
    assert handoff['external_write_required'] is True
    assert handoff['maintainer_confirmation_required'] is True
    assert handoff['issue_creation_authorized'] is False
    assert handoff['writes_performed'] is False
    assert report['eligible_good_first_issues'] == []
    assert report['blocked_good_first_issues'] == [{
        'number': 422,
        'title': issue['title'],
        'url': issue['html_url'],
        'gate': _publication_gate(),
    }]
    assert report['unpublished_ready_task_ids'] == EXPECTED_READY_IDS
    assert report['potential_pull_duplicates'] == []
    assert report['authority'] == {
        'github_requests': 'GET_ONLY',
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    rendered = CHECKER.render_next_report(report)
    assert issue['html_url'] in rendered
    assert '0 ready, 1 blocked' in rendered
    assert 'contributor_starter_queue.py --task starter-C5' in rendered
    assert 'Publication handoff: exact local body for starter-C5' in rendered
    assert 'Public base ready: no (WAITING_FOR_PRODUCT_MERGE)' in rendered
    assert handoff['issue_body_sha256'] in rendered
    assert 'GitHub issue creation authorized: no' in rendered
    assert 'do not start a blocked cohort task' in rendered
    assert 'no GitHub issue, pull request, or label was changed' in rendered


def test_blocked_issue_gate_is_next_when_no_independent_task_is_ready():
    """A closed issue gate remains actionable after local work is exhausted."""
    queue, local_report = CHECKER.evaluate()
    local_report = {**local_report, 'ready_task_ids': []}
    issue = _github_issue(
        422,
        'Help validate the public first-map path for v1.0',
        labels=('documentation', 'good first issue', 'help wanted'),
    )

    report = CHECKER.build_next_report(
        queue,
        local_report,
        [issue],
        [],
        [_publication_gate()],
        _local_task_gate(),
    )

    assert report['contributor_next']['action'] == (
        'WAIT_FOR_READY_PUBLISHED_STARTER'
    )
    assert report['maintainer_next'] == {
        'action': 'REVIEW_BLOCKED_PUBLISHED_STARTER',
        'issue_number': 422,
        'gate_id': 'first-map-validator-cohort-v1',
        'gate_status': 'WAITING_FOR_PUBLIC_GATES',
        'review_command': [
            'python3',
            'scripts/first_map_validator_cohort.py',
            '--json',
        ],
    }
    assert report['maintainer_publication_handoff'] is None


def test_publication_handoff_rejects_body_and_task_linkage_tampering():
    """A selected task cannot carry stale or cross-task publication bytes."""
    queue, local_report = CHECKER.evaluate()
    report = CHECKER.build_next_report(
        queue,
        local_report,
        [],
        [],
        [_publication_gate()],
        _local_task_gate(),
    )

    stale_body = copy.deepcopy(report)
    stale_body['maintainer_publication_handoff']['issue_body'] += 'changed\n'
    with pytest.raises(CHECKER.QueueError, match='issue body digest is stale'):
        CHECKER._validate_next_report(stale_body)

    cross_task = copy.deepcopy(report)
    cross_task['maintainer_publication_handoff']['task_id'] = 'starter-C6'
    with pytest.raises(CHECKER.QueueError, match='task does not match'):
        CHECKER._validate_next_report(cross_task)


def test_next_report_allows_a_gated_issue_only_when_cohort_is_ready():
    """READY_FOR_NEXT_ATTEMPT turns the existing issue into a valid route."""
    queue, local_report = CHECKER.evaluate()
    issue = _github_issue(
        422,
        'Help validate the public first-map path for v1.0',
        labels=('documentation', 'good first issue', 'help wanted'),
    )
    gate = _publication_gate(
        status='READY_FOR_NEXT_ATTEMPT',
        eligible=True,
    )

    report = CHECKER.build_next_report(
        queue,
        local_report,
        [issue],
        [],
        [gate],
        _local_task_gate(status='READY'),
    )

    assert report['status'] == 'PUBLISHED_GOOD_FIRST_ISSUE_AVAILABLE'
    assert report['eligible_good_first_issues'] == [{
        'number': 422,
        'title': issue['title'],
        'url': issue['html_url'],
    }]
    assert report['blocked_good_first_issues'] == []
    assert report['contributor_next'] == {
        'action': 'REVIEW_PUBLISHED_GOOD_FIRST_ISSUE',
        'issue_number': 422,
        'url': issue['html_url'],
    }
    assert report['maintainer_next']['task_id'] == 'starter-C5'
    assert report['maintainer_next']['action'] == (
        'REVIEW_AND_PUBLISH_LOCAL_TASK'
    )
    assert report['maintainer_publication_handoff']['task_id'] == 'starter-C5'
    assert report['maintainer_publication_handoff'][
        'public_base_ready'
    ] is True


def test_next_report_keeps_dependency_when_the_issue_title_changes():
    """The repository-stable issue number keeps #422 fail closed."""
    queue, local_report = CHECKER.evaluate()
    issue = _github_issue(
        422,
        'Edited public first-map validation title',
        labels=('good first issue',),
    )

    report = CHECKER.build_next_report(
        queue,
        local_report,
        [issue],
        [],
        [_publication_gate()],
        _local_task_gate(),
    )

    assert report['status'] == 'PUBLISHED_GOOD_FIRST_ISSUES_BLOCKED'
    assert report['blocked_good_first_issues'][0]['number'] == 422
    assert report['contributor_next']['action'] == (
        'WAIT_FOR_READY_PUBLISHED_STARTER'
    )


def test_next_report_keeps_an_unrelated_good_first_issue_eligible():
    """A closed cohort gate does not suppress independent starter work."""
    queue, local_report = CHECKER.evaluate()
    issue = _github_issue(
        600,
        'Docs: clarify one bounded command',
        labels=('documentation', 'good first issue'),
    )

    report = CHECKER.build_next_report(
        queue,
        local_report,
        [issue],
        [],
        [_publication_gate()],
        _local_task_gate(),
    )

    assert report['status'] == 'PUBLISHED_GOOD_FIRST_ISSUE_AVAILABLE'
    assert report['eligible_good_first_issues'][0]['number'] == 600
    assert report['blocked_good_first_issues'] == []
    assert report['contributor_next']['issue_number'] == 600


def test_next_report_prefers_an_exact_published_queue_task():
    """An exact live task issue takes priority over a generic starter."""
    queue, local_report = CHECKER.evaluate()
    task = _task(queue, 'starter-C5')
    issue = _github_issue(
        501,
        task['title'],
        labels=('documentation', 'good first issue', 'help wanted'),
    )

    report = CHECKER.build_next_report(
        queue,
        local_report,
        [issue],
        [],
        [_publication_gate()],
        _local_task_gate(),
    )

    assert report['status'] == 'PUBLISHED_QUEUE_TASK_AVAILABLE'
    assert report['contributor_next']['task_id'] == 'starter-C5'
    assert report['contributor_next']['url'] == issue['html_url']
    assert report['unpublished_ready_task_ids'] == EXPECTED_READY_IDS[1:]
    assert report['maintainer_next']['task_id'] == 'starter-C6'
    assert report['maintainer_publication_handoff']['task_id'] == 'starter-C6'


def test_next_report_rechecks_a_known_pull_updated_after_the_audit():
    """A changed known PR cannot inherit an older non-duplicate decision."""
    queue, local_report = CHECKER.evaluate()
    changed_pull = _github_pull(
        427,
        'Prepare crash-safe guided mapping for G0 review',
        body='Japanese validation report follow-up handoff',
        updated_at='2026-08-15T22:23:00Z',
    )

    report = CHECKER.build_next_report(
        queue,
        local_report,
        [],
        [changed_pull],
        [_publication_gate()],
        _local_task_gate(),
    )

    assert report['potential_pull_duplicates'] == [{
        'task_id': 'starter-C5',
        'pull_requests': [{
            'number': 427,
            'title': changed_pull['title'],
            'url': changed_pull['html_url'],
        }],
    }]
    assert report['maintainer_next'] == {
        'action': 'REVIEW_POTENTIAL_PULL_DUPLICATE',
    }
    assert report['maintainer_publication_handoff'] is None


def test_live_reader_uses_get_only(monkeypatch):
    """Remote availability performs only bounded GET requests."""
    commands = []

    def fake_run(command, **kwargs):
        commands.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, '[]', '')

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)

    assert CHECKER._github_get_all(
        'rsasaki0109/lidar_slam_ros2',
        'issues',
    ) == []
    assert len(commands) == 1
    command, kwargs = commands[0]
    assert command[:4] == ['gh', 'api', '--method', 'GET']
    assert 'state=open' in command[-1]
    assert kwargs['check'] is False
    assert kwargs['timeout'] == 60


def test_local_task_gate_waits_for_product_merge_and_public_queue():
    """The current draft state cannot suggest public issue publication."""
    queue = _queue()

    gate = CHECKER.build_local_task_publication_gate(
        queue,
        _product_pull_record(),
        None,
    )

    assert gate['status'] == 'WAITING_FOR_PRODUCT_MERGE'
    assert gate['eligible'] is False
    assert gate['pull_request_state'] == 'OPEN'
    assert gate['is_draft'] is True
    assert gate['merged'] is False
    assert gate['public_queue_status'] == 'ABSENT'
    assert gate['observed_queue_sha256'] is None
    assert gate['blocking_reasons'] == [
        'product_pr_not_merged',
        'public_queue_absent',
    ]
    assert gate['github_requests'] == 'GET_ONLY'
    assert gate['remote_mutations_performed'] is False


def test_local_task_gate_opens_only_after_merge_and_exact_public_queue():
    """A merged product PR and byte-semantic queue match open the gate."""
    queue = _queue()

    gate = CHECKER.build_local_task_publication_gate(
        queue,
        _product_pull_record(state='closed', draft=False, merged=True),
        copy.deepcopy(queue),
    )

    expected_sha256 = CHECKER._canonical_sha256(queue)
    assert gate['status'] == 'READY'
    assert gate['eligible'] is True
    assert gate['public_queue_status'] == 'MATCH'
    assert gate['expected_queue_sha256'] == expected_sha256
    assert gate['observed_queue_sha256'] == expected_sha256
    assert gate['blocking_reasons'] == []


@pytest.mark.parametrize(
    ('pull', 'public_queue_factory', 'expected_status', 'expected_reason'),
    [
        (
            _product_pull_record(state='closed', draft=False, merged=False),
            lambda queue: copy.deepcopy(queue),
            'PRODUCT_PR_CLOSED_UNMERGED',
            'product_pr_not_merged',
        ),
        (
            _product_pull_record(state='closed', draft=False, merged=True),
            lambda queue: None,
            'PUBLIC_QUEUE_ABSENT',
            'public_queue_absent',
        ),
        (
            _product_pull_record(state='closed', draft=False, merged=True),
            lambda queue: {**queue, 'generated_at': '2099-01-01T00:00:00Z'},
            'PUBLIC_QUEUE_DRIFT',
            'public_queue_drift',
        ),
    ],
)
def test_local_task_gate_fails_closed_for_each_public_base_mismatch(
    pull: dict,
    public_queue_factory,
    expected_status: str,
    expected_reason: str,
):
    """Closed-unmerged, missing, and drifted public bases remain distinct."""
    queue = _queue()

    gate = CHECKER.build_local_task_publication_gate(
        queue,
        pull,
        public_queue_factory(queue),
    )

    assert gate['status'] == expected_status
    assert gate['eligible'] is False
    assert expected_reason in gate['blocking_reasons']


def test_local_task_gate_rejects_a_different_base_repository():
    """The bounded PR identity cannot be redirected to another repository."""
    pull = _product_pull_record()
    pull['base']['repo']['full_name'] = 'example/other'

    with pytest.raises(CHECKER.QueueError, match='invalid bounded identity'):
        CHECKER.build_local_task_publication_gate(_queue(), pull, None)


def test_product_pull_reader_uses_one_get_request(monkeypatch):
    """The product dependency reader cannot mutate the pull request."""
    calls = []
    payload = _product_pull_record()

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, json.dumps(payload), '')

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)

    assert CHECKER._github_get_product_pull(
        'rsasaki0109/lidar_slam_ros2',
        427,
    ) == payload
    command, kwargs = calls[0]
    assert command[:4] == ['gh', 'api', '--method', 'GET']
    assert command[-1] == '/repos/rsasaki0109/lidar_slam_ros2/pulls/427'
    assert kwargs['check'] is False


def test_public_queue_reader_decodes_wrapped_base64_and_accepts_only_404(
    monkeypatch,
):
    """Public content is GET-only; a missing file is explicit, not an error."""
    queue = _queue()
    encoded = CHECKER.base64.b64encode(
        json.dumps(queue).encode('utf-8')
    ).decode('ascii')
    wrapped = f'{encoded[:40]}\n{encoded[40:]}'
    responses = [
        subprocess.CompletedProcess(
            [],
            0,
            json.dumps({
                'type': 'file',
                'encoding': 'base64',
                'content': wrapped,
            }),
            '',
        ),
        subprocess.CompletedProcess([], 1, '', 'gh: Not Found (HTTP 404)'),
    ]
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        response = responses.pop(0)
        return subprocess.CompletedProcess(
            command,
            response.returncode,
            response.stdout,
            response.stderr,
        )

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)
    arguments = (
        'rsasaki0109/lidar_slam_ros2',
        'docs/contracts/contributor-starter-queue-v1.json',
        'develop',
    )

    assert CHECKER._github_get_public_queue(*arguments) == queue
    assert CHECKER._github_get_public_queue(*arguments) is None
    assert all(call[0][:4] == ['gh', 'api', '--method', 'GET']
               for call in calls)
    assert all('?ref=develop' in call[0][-1] for call in calls)


def test_publication_gate_collector_uses_read_only_cohort_state(monkeypatch):
    """The dependency adapter accepts only the no-write state report."""
    payload = {
        'status': 'WAITING_FOR_PUBLIC_GATES',
        'next_attempt_permitted_by_state': False,
        'pending_launch_gates': [
            'comparable_docker_row',
            'canonical_documentation_provenance',
        ],
        'stop_conditions': [],
        'community_posts_authorized': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(
            command,
            0,
            json.dumps(payload),
            '',
        )

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)
    dependency = _queue()['published_issue_dependencies'][0]

    report = CHECKER._collect_publication_gate(dependency)

    assert report == {
        'id': 'first-map-validator-cohort-v1',
        'status': 'WAITING_FOR_PUBLIC_GATES',
        'eligible': False,
        'blocking_reasons': [
            'comparable_docker_row',
            'canonical_documentation_provenance',
        ],
        'check_command': dependency['check_command'],
        'remote_mutations_performed': False,
    }
    command, kwargs = calls[0]
    assert command == dependency['check_command']
    assert kwargs['check'] is False
    assert kwargs['timeout'] == 60
    assert kwargs['env']['PYTHONDONTWRITEBYTECODE'] == '1'


def test_publication_gate_collector_rejects_write_authority(monkeypatch):
    """A gate report that claims remote authority fails closed."""
    payload = {
        'status': 'READY_FOR_NEXT_ATTEMPT',
        'next_attempt_permitted_by_state': True,
        'pending_launch_gates': [],
        'stop_conditions': [],
        'community_posts_authorized': True,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }

    def fake_run(command, **kwargs):
        del kwargs
        return subprocess.CompletedProcess(
            command,
            0,
            json.dumps(payload),
            '',
        )

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)
    dependency = _queue()['published_issue_dependencies'][0]

    with pytest.raises(CHECKER.QueueError, match='no-write boundary'):
        CHECKER._collect_publication_gate(dependency)


def test_live_reader_fails_closed_on_an_api_error(monkeypatch):
    """An unavailable API cannot become an empty availability report."""

    def fake_run(command, **kwargs):
        del kwargs
        return subprocess.CompletedProcess(command, 1, '', 'offline')

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)

    with pytest.raises(CHECKER.QueueError, match='offline'):
        CHECKER._github_get_all(
            'rsasaki0109/lidar_slam_ros2',
            'issues',
        )


def test_missing_scoped_file_fails_closed(tmp_path: Path):
    """A missing file invalidates its starter scope."""
    payload = _queue()
    _copy_scoped_files(payload, tmp_path)
    (tmp_path / 'docs' / 'getting-started.md').unlink()

    with pytest.raises(
        CHECKER.QueueError,
        match='allowed path is not a regular file',
    ):
        CHECKER.validate_queue(payload, _schema(), tmp_path)


def test_probe_cannot_escape_task_path_scope():
    """Known-gap probes can inspect only files a task may modify."""
    payload = _queue()
    _task(payload, 'starter-C6')['gap_probes'][0]['path'] = 'README.md'

    with pytest.raises(
        CHECKER.QueueError,
        match='gap probe is outside its path scope',
    ):
        CHECKER.validate_queue(payload, _schema())


def test_replenished_generation_uses_absent_exact_gap_markers():
    """All five scopes are new while retired canonical cards stay present."""
    payload, report = CHECKER.evaluate()

    for task_id, marker in EXPECTED_GAP_MARKERS.items():
        probe = _task(payload, task_id)['gap_probes'][0]
        assert probe['value'] == marker
        assert probe['mode'] == 'not_contains'
        assert marker not in (ROOT / probe['path']).read_text(encoding='utf-8')

    retired_markers = {
        'docs/getting-started.md': (
            '### Recover g2o dependency failures',
            '### Empty map or viewer: three-check recovery',
        ),
        'docs/workflows.md': (
            '### Odometry and TF: two separate contracts',
            '### Adapting another PointCloud2 LiDAR',
        ),
    }
    for path, markers in retired_markers.items():
        source = (ROOT / path).read_text(encoding='utf-8')
        assert all(marker in source for marker in markers)

    assert report['ready_task_ids'] == EXPECTED_READY_IDS
    assert report['stale_tasks'] == []


@pytest.mark.parametrize('task_id', EXPECTED_READY_IDS[1:])
def test_new_english_task_becomes_stale_when_its_marker_appears(
    tmp_path: Path,
    task_id: str,
):
    """A completed refreshed card is removed from the ready set."""
    payload = _queue()
    _copy_scoped_files(payload, tmp_path)
    task = _task(payload, task_id)
    probe = task['gap_probes'][0]
    source = tmp_path / probe['path']
    source.write_text(
        source.read_text(encoding='utf-8') + f"\n{probe['value']}\n",
        encoding='utf-8',
    )

    report = CHECKER.validate_queue(payload, _schema(), tmp_path)

    assert report['ready_task_ids'] == [
        item for item in EXPECTED_READY_IDS if item != task_id
    ]
    assert report['stale_tasks'] == [{
        'id': task_id,
        'status': 'STALE',
        'reasons': [probe['failure_detail']],
    }]


def test_japanese_validation_follow_up_handoff_task_becomes_stale_when_marker_appears(
    tmp_path: Path,
):
    """A successor Japanese handoff card requires reassessment."""
    payload = _queue()
    _copy_scoped_files(payload, tmp_path)
    source = tmp_path / 'docs' / 'getting-started-ja.md'
    source.write_text(
        source.read_text(encoding='utf-8')
        + '\n### 日本語のvalidation reportのfollow-up引き継ぎを再現可能にする\n',
        encoding='utf-8',
    )

    report = CHECKER.validate_queue(payload, _schema(), tmp_path)

    assert report['status'] == 'QUEUE_STALE_LOCAL_ONLY'
    assert [item['id'] for item in report['stale_tasks']] == ['starter-C5']
    assert report['stale_tasks'][0]['reasons'] == [
        'the planned Japanese follow-up-handoff marker already exists',
    ]


def test_japanese_recovery_card_keeps_empty_frame_action():
    """The completed Japanese card retains an actionable frame repair."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'header.frame_id' in source
    assert 'publisherの`header.frame_id`を修正してから再確認' in source
    assert 'viewerのframe名を推測して先に進めません' in source


def test_japanese_recovery_card_explains_pointcloud_topic_selection():
    """The Japanese card makes the topic placeholder copy-ready."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'ros2 topic list -t' in source
    assert '[sensor_msgs/msg/PointCloud2]' in source
    assert '`<POINTCLOUD_TOPIC>`をその名前に置き換えます' in source
    assert 'PointCloud2の行がない場合は' in source


def test_japanese_recovery_card_explains_tf_frame_substitution():
    """The Japanese card connects the TF placeholders to observed frames."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'check 2で出た空でないframe名を`<POINTCLOUD_FRAME>`に入れます' in source
    assert '<TF_TARGET_FRAME>`にはruntimeまたはviewerが基準にするtarget frame' in source
    assert '`livox_frame`なら、`<POINTCLOUD_FRAME>`だけを' in source
    assert 'viewerでframe名を推測したり' in source
    assert '同じ実際の' in source
    assert 'frame名で再確認します' in source


def test_japanese_recovery_card_explains_headless_preview_options():
    """The Japanese card makes local browser recovery copy-ready."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'ブラウザが自動で' in source
    assert 'ヘッドレス環境では' in source
    assert 'self-contained HTML' in source
    assert '`--no-open`を付けるとブラウザを起動せず' in source
    assert 'lidarslam-map view /path/to/output' in source
    assert '--viewer browser' in source
    assert '--no-open' in source
    assert '--preview-dir /path/to/preview' in source
    assert (
        '`HTML: /path/to/output/preview/mid360_robot_3d_map_preview.html`'
        in source
    )
    assert 'サニタイズ済みのsupport' in source


def test_japanese_recovery_card_explains_session_history_and_recovery():
    """The Japanese card makes retained session recovery copy-ready."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '保存したsessionを探して復旧する' in source
    assert 'lidarslam-map sessions' in source
    assert '--status action_required' in source
    assert '--viewer none' in source
    assert '--json' in source
    assert '`Details:`と`Next:`' in source
    assert '`Next:`の行に表示された実際のcommandをそのまま実行' in source
    assert 'map_verify' in source
    assert 'autoware_map_diagnosis.md' in source
    assert 'session履歴、catalog、診断、previewはlocal-only' in source
    assert 'preview\nHTMLをGitHub issueへuploadせず' in source


def test_japanese_recovery_card_explains_privacy_first_support_handoff():
    """The Japanese card makes support sharing reviewable and bounded."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '支援へ共有する前に確認する' in source
    assert 'lidarslam-map support /path/to/session_bundle --json' in source
    assert 'read-onlyで、ZIPを作成せず' in source
    assert 'README.txt' in source
    assert 'issue-body.md' in source
    assert 'support-report.json' in source
    assert 'map geometry、bag、raw log、parameter内容' in source
    assert 'credentialのようなcommand値は含まれません' in source
    assert '`report`はreceipt-boundのPASSを再検証' in source
    assert 'review済みのfirst-map receiptだけ' in source
    assert '`report --json`のhandoff JSONは' in source
    assert '公開添付には使いません' in source


def test_japanese_recovery_card_explains_independent_first_map_handoff():
    """The Japanese card keeps independent validation privacy-bounded."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '独立validator向けのissue form' in source
    assert 'canonical\nindependent-validation issue form' in source
    assert '自分で実行したrunの' in source
    assert 'private pathをredact' in source
    assert 'first-map receiptだけを添付します' in source
    assert 'map、bag、preview、\nraw log、trajectory、parameter' in source
    assert (
        '`report --json`のhandoff JSONとlocal receipt pathはpublic '
        'attachmentではありません' in source
    )
    assert 'maintainerのlive step-by-step guidanceは' in source
    assert '[Independent First-map Validation](external-first-map-validation.md)' in source


def test_japanese_recovery_card_explains_version_identity_boundary():
    """The Japanese card makes product identity copy-ready."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'versionとsupport境界を記録する' in source
    assert 'lidarslam-map --version' in source
    assert '公開済み安定版`v0.9.0-humble`または`v0.9.0-jazzy`' in source
    assert '`v0.9.1`はまだ公開・tag付けされていないsource候補' in source
    assert 'candidateとしてそのversion/revisionを報告' in source
    assert '`develop`の移動tagを使ったり' in source
    assert '`--version`の出力をそのままsupport report' in source


def test_japanese_recovery_card_explains_reason_code_and_next_action_triage():
    """The Japanese card keys diagnosis on stable codes and retained actions."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert 'JSON診断でreason-codeとNext-actionを読む' in source
    assert 'lidarslam-map doctor /path/to/rosbag2 --json' in source
    assert 'read-onlyで読み、ネットワークへ接続せず' in source
    assert '`findings[].code`を安定したキー' in source
    assert '`reason.code`と各項目の' in source
    assert 'viewerで見えた症状' in source
    assert '`next_action`、`Next:`、`next_command`は保持された次の操作' in source
    assert 'raw JSONをissueへ貼り付けません' in source
    assert '安全なterminal post-processingだけを再開する`--resume`' in source
    assert 'mappingやviewerを再試行せず' in source
    assert '失敗したrunを上書きしません' in source


def test_japanese_recovery_card_explains_dry_run_write_boundary():
    """The Japanese card makes the pre-write plan and headless route explicit."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '### 自分のbagを先にdry-runで確認する' in source
    assert 'lidarslam-map start /path/to/rosbag2 \\' in source
    assert '  --yes \\' in source
    assert '  --dry-run \\' in source
    assert '  --json' in source
    assert '`--dry-run`ではsession bundle、`sensor_setup.json`、map outputを残さず' in source
    assert '`status`が\n`dry_run`になり' in source
    assert '`run.command_shell`に表示された保持済みの実行command' in source
    assert '`reason.code`、`findings[].code`、' in source
    assert '`next_command`がdoctorを示すときはそのcommandへ戻り' in source
    assert '確認したplanで実際にmappingへ進むときは`--dry-run`を外し' in source
    assert '`--viewer none`を使います' in source
    assert 'planをレビューした後だけ`--yes --viewer none`を追加' in source
    assert 'raw outputをissueへ貼り付けません' in source


def test_japanese_recovery_card_explains_fresh_retry_without_overwrite():
    """The Japanese card keeps setup/evidence and retry output distinct."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '### 失敗したrunを上書きせず再試行する' in source
    assert '`--output-dir`がすでに存在する場合は`output directory already exists`' in source
    assert '`setup_bundle`:' in source
    assert '`sensor_setup.json`、pinned parameter、session page' in source
    assert '`evidence`と`files_preserved: true`' in source
    assert '`retry.available: true`' in source
    assert '`retry.command`が同じpinned setupを使い' in source
    assert '`retry.output_dir`' in source
    assert '`map.retry`のような新しいdirectory' in source
    assert '`resume.available: true`' in source
    assert 'で`next_command`が`--resume`なら' in source
    assert 'mappingを再実行しません' in source
    assert 'retry.command`をpathやoptionを編集せず' in source
    assert '古いsessionとは別の新しい' in source
    assert '元のsession、map、raw logを削除・uploadせず' in source


def test_japanese_recovery_card_explains_verified_result_boundary():
    """The Japanese card separates a displayed map from trusted evidence."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '### 検証済みmapと表示できるmapを区別する' in source
    assert 'viewerで`pointcloud_map/`やpreviewが開けても' in source
    assert '`map_verify: PASS`' in source
    assert '`first_map_validation_receipt.json`の`status: PASS`' in source
    assert 'trusted resultとして扱えるのは' in source
    assert '`NOT VERIFIED`' in source
    assert '`UNAVAILABLE`はreceiptがない、壊れている' in source
    assert '別sessionからコピーしたreceiptを現在のmapの証拠には使いません' in source
    assert (
        '`autoware_map_diagnosis.md`/JSON、`verify_autoware_map.log`、'
        '`run_manifest.json`' in source
    )
    assert 'lidarslam-map inspect /path/to/output --write' in source
    assert '表示されたmapをverified resultとして\n扱いません' in source


def test_japanese_recovery_card_explains_receipt_session_revalidation():
    """The Japanese card keeps receipt provenance tied to one session."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '### receiptのsessionとstatusを確認する' in source
    assert '`first_map_validation_receipt.json`だけを別のmap directoryへコピーしても' in source
    assert '`artifacts.validation_receipt`のpathと`map_output`' in source
    assert '`run.run_id`が\n同じ`run_manifest.json`の`run_id`と一致' in source
    assert '`verification.manifest_sha256`は、そのsessionの`run_manifest.json`に結び付いた値' in source
    assert '別runのreceipt、古いsessionのreceipt、名前だけ変更したreceiptを混ぜません' in source
    assert 'lidarslam-map sessions ./output --json' in source
    assert 'lidarslam-map report /path/to/session_bundle' in source
    assert '全check、manifest・diagnosis・verification logのhashが一致' in source
    assert '`READY FOR REVIEW`が出た場合だけ' in source
    assert '`report --json`' in source
    assert '書き込みもGitHubへの通信も行いません' in source
    assert 'receipt、`run_manifest.json`、`session.json`を\n手編集してhashを合わせたり' in source
    assert '`retry.command`またはverification-enabledな新しいoutput command' in source


def test_japanese_recovery_card_explains_failed_receipt_revalidation_recovery():
    """The Japanese card preserves rejected evidence and gives safe recovery."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '### receipt再検証に失敗したときの復旧' in source
    assert (
        '`report`がrejectしても、元のmap、session、receipt、manifestは'
        '削除されません。'
    ) in source
    assert 'receipt、`run_manifest.json`、`session.json`の内容やhashを手で\n書き換えて再検証を通そうとしません。' in source
    assert (
        'lidarslam-map sessions ./output --status action_required --viewer none --json'
        in source
    )
    assert '`map_session_recovery.json`、diagnosis、`Details:`、`Next:`' in source
    assert '`resume.available`\nがtrueで`next_command`が`--resume`なら' in source
    assert '`retry.available`がtrueなら、保存された`retry.command`を編集せずに実行します' in source
    assert '`map.retry`または`map.retry-2`のような新しいoutput' in source
    assert 'verification offの診断' in source
    assert '--verification required' in source
    assert 'report`が`READY FOR REVIEW`を返し' in source
    assert '旧証跡、bag、map、raw logは削除・uploadせず' in source


def test_japanese_recovery_card_explains_pre_share_verification_checklist():
    """The Japanese card makes the public-share boundary copy-ready."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '### 公開共有前の5項目チェック' in source
    assert '同じ\nsessionについて次の5項目を確認' in source
    assert '`lidarslam-map --version`' in source
    assert '`run.product_version`と`run.git_commit`' in source
    assert '`run.run_id`、`map_output`、receipt path' in source
    assert '`verification.manifest_sha256`' in source
    assert '`report`が`READY FOR REVIEW`を返し' in source
    assert 'receiptの`status: PASS`と全checkのPASS' in source
    assert 'receiptの`shareability`を読み' in source
    assert 'private pathをredactした' in source
    assert 'handoff JSONとlocal receipt pathはlocal-only' in source
    assert '`first_map_validation_receipt.json`だけ' in source
    assert 'map、bag、raw log、preview HTML、trajectory、parameter、screenshot' in source
    assert 'session bundle全体も添付しません' in source


def test_japanese_recovery_card_explains_public_receipt_report_template():
    """The Japanese card keeps public report fields copy-ready and bounded."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '### 公開共有用のreceiptテンプレート' in source
    assert '`report`が表示するcanonical issue form' in source
    assert '公開ドキュメント経路:' in source
    assert 'release/commit/image digest:' in source
    assert 'environment: <OS> / <architecture> / <ROS> / <install method>' in source
    assert 'exact command (private paths redacted):' in source
    assert 'result: PASS — verified first map completed' in source
    assert 'manifest_status=succeeded' in source
    assert 'diagnosis_status=success' in source
    assert 'autoware_status=PASS' in source
    assert 'manifest_sha256=<64 lowercase hex characters>' in source
    assert 'attachment: first_map_validation_receipt.json (reviewed)' in source
    assert '`release/commit/image digest`はreceiptの`run.product_version`' in source
    assert '`report --json`のhandoff、`receipt_path`、`markdown_path`' in source
    assert 'PASSを確認して内容をreviewした' in source
    assert '`first_map_validation_receipt.json`だけをpublic attachment' in source


def test_japanese_recovery_card_explains_validation_report_review_status():
    """The Japanese card separates handoff, review, and acceptance states."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')

    assert '### validation reportのreview statusを区別する' in source
    assert 'local `READY FOR REVIEW`' in source
    assert 'public report submitted' in source
    assert 'maintainer review' in source
    assert 'accepted ledger evidence' in source
    assert 'unresolved / rejected' in source
    assert 'accepted validationではありません' in source
    assert 'roadmapの\nmatrix evidenceを推測しません' in source
    assert 'liveなstep-by-step validation helpを依頼せず' in source
    assert '複数issueへ重複添付したりしません' in source
    assert 'ledgerにacceptedとして記録された' in source


def test_japanese_recovery_card_has_privacy_safe_report_example():
    """The Japanese example is illustrative and keeps evidence provenance clear."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    example = source.split(
        '### 日本語のprivacy-safe validation report例', 1)[1].split(
            '## 詳細', 1)[0]

    assert '[説明用の架空例 — not a real validation result / not accepted ledger evidence]' in example
    assert (
        'release/commit/image digest: example-image@sha256:'
        '<example-only-64-hex-digest>' in example
    )
    assert 'environment: Ubuntu 22.04 / amd64 / ROS 2 Humble / Docker' in example
    assert 'result: PASS — verified first map completed (example only)' in example
    assert 'manifest_sha256=<example-only; copyしない>' in example
    assert 'operator-supplied public fields' in example
    assert 'receipt-derived fields' in example
    assert 'not submitted / not maintainer-reviewed / not accepted' in example
    assert '/home/' not in example
    assert '/tmp/' not in example


def test_japanese_recovery_card_explains_docker_source_route_choice():
    """The Japanese route card keeps identity and stop boundaries explicit."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語のDockerとsource経路を選ぶ', 1)[1].split(
            '## 1. インストールを確認する', 1)[0]

    assert '最初のfirst-mapでは、Dockerまたはsourceのどちらか1つだけ' in card
    assert 'Docker fixed first-map' in card
    assert 'source quickstart' in card
    assert 'v0.9.0-humble' in card
    assert 'v0.9.0-jazzy' in card
    assert 'v0.9.1' in card
    assert 'docker run --rm' in card
    assert 'bash scripts/source_quickstart.sh --dry-run' in card
    assert 'map_verify: PASS' in card
    assert 'receiptの`status: PASS`' in card
    assert '`dry_run`のplanと`run.command_shell`' in card
    assert 'PPA/package-manager経路は未対応' in card
    assert '出力directoryやsessionを別経路で使い回さず' in card
    assert 'Dockerのreceiptと混ぜない' in card


def test_japanese_recovery_card_explains_fresh_output_route_switch():
    """The Japanese recovery card separates resume, retry, and route changes."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語の経路切替とfresh output復旧', 1)[1].split(
            '## 1. インストールを確認する', 1)[0]

    assert '元のrunを別経路で\n続けません' in card
    assert '`resume.available: true`' in card
    assert '`next_command`の`--resume`を編集せずに実行する' in card
    assert '`retry.available: true`' in card
    assert '`retry.command`を編集せずに実行する' in card
    assert '`retry.output_dir`の新しいoutput' in card
    assert 'Docker/sourceの経路を変える' in card
    assert 'fresh outputで新しいrunを開始する' in card
    assert '古いmap、session、receipt、manifestを新runへコピー・再利用せず' in card
    assert '`--resume`は既存sessionの安全なterminal post-processing用' in card
    assert '`v0.9.0` Dockerまたは\n`v0.9.1` source候補' in card
    assert '旧runと新runのreceiptやhashを混ぜません' in card
    assert 'privacy-bounded support reportだけを使います' in card


def test_japanese_recovery_card_separates_support_and_validation_reports():
    """The Japanese card keeps diagnosis and accepted evidence distinct."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語のsupport reportとvalidation reportを分ける', 1)[1].split(
            '検証済みfirst mapを独立validatorへ渡す場合だけ', 1)[0]

    assert '`support report`と、利用者が公開手順を自分で実行した結果' in card
    assert '`lidarslam-map support /path/to/session_bundle --json`' in card
    assert '`README.txt`、`issue-body.md`、' in card
    assert '`support-report.json`をすべて読み' in card
    assert 'accepted validation evidenceではない' in card
    assert '`report`' in card
    assert '`report --json`のhandoff JSON' in card
    assert 'canonical independent-validation issue form' in card
    assert '`first_map_validation_receipt.json`を内容確認したもの1つだけ' in card
    assert 'maintainer reviewとvalidation ledgerのaccepted記録' in card
    assert 'recovery JSON' in card
    assert '`map_session_recovery.json`' in card
    assert 'session bundle全体は貼りません' in card
    assert '`v0.9.0-humble`または`v0.9.0-jazzy`' in card
    assert '`v0.9.1`候補と' in card
    assert '別runのreceiptやhashを1つの報告へ混ぜず' in card


def test_japanese_recovery_card_explains_report_field_provenance():
    """The Japanese report card binds fields to the correct evidence source."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語の公開reportのfield provenanceを確認する', 1)[1].split(
            '### validation reportのreview statusを区別する', 1)[0]

    assert 'operator-supplied public fields' in card
    assert 'receipt-derived validation fields' in card
    assert '公開ドキュメント経路、`environment`' in card
    assert 'private pathをredactしたexact command' in card
    assert '`release/commit/image digest`、`result`' in card
    assert 'verification summary' in card
    assert '`manifest_sha256`' in card
    assert 'receipt/handoffの値をそのまま照合して転記する' in card
    assert 'accepted validationと書かない' in card
    assert '`run.product_version`・`run.git_commit`' in card
    assert '`run.run_id`' in card
    assert '`map_output`' in card
    assert '`verification.manifest_sha256`' in card
    assert 'missing' in card
    assert '`UNAVAILABLE`' in card
    assert 'mismatch' in card
    assert 'example-only' in card
    assert 'viewer-only' in card
    assert 'Details:' in card
    assert '`v0.9.0-humble`または`v0.9.0-jazzy`' in card
    assert '`v0.9.1`候補とexact commit/revision' in card
    assert '`develop` tag' in card
    assert 'session bundle全体' in card


def test_japanese_recovery_card_explains_safe_validation_findings():
    """The Japanese findings card keeps observations actionable and safe."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語のvalidation reportのfindingsを安全に書く', 1)[1].split(
            '### validation reportのreview statusを区別する', 1)[0]

    assert '`findings`は、自分のrunで気付いた一件の観察' in card
    assert 'operator-supplied field' in card
    assert 'step:' in card
    assert 'expected:' in card
    assert 'observed:' in card
    assert 'impact:' in card
    assert 'Docker First Map' in card
    assert 'source quickstart' in card
    assert 'Session summary' in card
    assert '`reason.code`' in card
    assert 'private path' in card
    assert 'raw log' in card
    assert 'root causeが不明なら`不明`' in card
    assert '4項目のblockを観察ごとに分けます' in card
    assert 'manifest_sha256' in card
    assert 'receipt JSON' in card
    assert '独立validation reportへ' in card
    assert 'support reportとして扱います' in card


def test_japanese_recovery_card_explains_validation_finding_follow_up():
    """The Japanese follow-up card preserves routes and evidence boundaries."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語のvalidation reportのfinding follow-upを安全に行う', 1)[1].split(
            '### validation reportのreview statusを区別する', 1)[0]
    normalized = ' '.join(card.split())

    assert '`unresolved`または`rejected`' in normalized
    assert '保存された`Details:`、`Next:`、`retry.command`' in normalized
    assert 'support follow-up' in normalized
    assert 'new independent validation' in normalized
    assert '同じpinned setupの`retry.command`' in normalized
    assert '新しい`retry.output_dir`のfresh output' in normalized
    assert '元のreport、receipt、manifest hash、review statusを編集する' in normalized
    assert 'support reportをaccepted validationにする' in normalized
    assert '古いmap、session、receipt、hashを新runへコピーする' in normalized
    assert 'maintainerのlive guidanceだけの結果を独立validationと呼ぶ' in normalized
    assert 'follow-up route: support follow-up' in normalized
    assert 'original report/receipt: unchanged' in normalized
    assert '`reason.code`' in normalized
    assert 'private pathを除いた保存済みの説明' in normalized
    assert 'review status: unresolved / retrying — not accepted validation' in normalized
    assert '`v0.9.0-humble`または`v0.9.0-jazzy`' in normalized
    assert '`v0.9.1`候補とexact commit/revision' in normalized
    assert 'identity、session、または 同じrunへの結び付きを確認できない場合は停止' in normalized
    assert '一つのrunにつきreportとreceiptは1組だけ' in normalized
    assert '複数issueへ重複添付しません' in normalized
    assert 'handoff JSON、local receipt pathは公開せず' in normalized


def test_japanese_recovery_card_explains_follow_up_evidence_pairing():
    """The Japanese pairing card makes follow-up evidence auditable."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語のvalidation reportのfollow-up証跡を一組で保つ', 1)[1].split(
            '### validation reportのreview statusを区別する', 1)[0]
    normalized = ' '.join(card.split())
    compact = normalized.replace(' ', '')

    assert '元のrunの`report + reviewed receipt`' in normalized
    assert 'follow-upの要約' in normalized
    assert '別runの independent-validation report' in normalized
    assert '新しいreceiptや 新しいaccepted evidenceではありません' in normalized
    assert '元のreport、receipt、`manifest_sha256`、reviewstatusを変えず' in compact
    assert 'original report/receipt pair' in normalized
    assert 'follow-up note' in normalized
    assert 'new independent-validation report' in normalized
    assert '新しいreceiptを作らず' in normalized
    assert '保存済み`reason.code`' in normalized
    assert 'sanitized `Details:`/`Next:`' in normalized
    assert 'fresh-output facts' in normalized
    assert 'acceptedはledgerの明示記録がある場合だけ書く' in normalized
    assert 'follow-up audit:' in normalized
    assert 'original identity:' in normalized
    assert 'follow-up route:' in normalized
    assert 'reason.code: <saved stable code>' in normalized
    assert 'Details: <sanitized saved explanation>' in normalized
    assert 'Next: <sanitized saved next action>' in normalized
    assert 'review status: <unresolved / retrying / READY FOR REVIEW — not accepted>' in normalized
    assert 'duplicate check: <no duplicate issue or session artifact>' in normalized
    assert '同じrunのsupport follow-up' in normalized
    assert 'fresh-output facts`だけを新しいoutputから追記' in normalized
    assert '`v0.9.0-humble`または`v0.9.0-jazzy`' in normalized
    assert '`v0.9.1`候補とexact commit/revision' in normalized
    assert 'identity、session、output、またはreportとreceiptが同じrunに結び付く' in normalized
    assert 'acceptedや independent validationとも書きません' in normalized
    assert '公開validation evidence の一組に昇格させません' in normalized


def test_japanese_recovery_card_explains_follow_up_audit_dispositions():
    """The Japanese audit card gives evidence-based dispositions."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語のvalidation reportのfollow-up要約を監査可能にする', 1)[1].split(
            '### validation reportのreview statusを区別する', 1)[0]
    normalized = ' '.join(card.split())

    assert 'maintainerがfollow-up要約を受け取ったら' in normalized
    assert '元の `report + reviewed receipt`と照合します' in normalized
    assert 'どれか一つでも確認できない 場合は`STOP`' in normalized
    assert '1. identity' in normalized
    assert '2. same run' in normalized
    assert '3. route' in normalized
    assert '4. provenance' in normalized
    assert '5. review status' in normalized
    assert 'missing/mismatchならSTOP' in normalized
    assert '同じrunを確認できなければSTOP' in normalized
    assert 'noteを新しいreport/receiptに変換せず' in normalized
    assert (
        'receipt-derived result、verification、hash、review statusをfollow-up '
        'factsで上書きしない' in normalized
    )
    assert 'ledgerの明示的なaccepted記録がない限りacceptedと分類しない' in normalized
    assert '`MATCHED FOLLOW-UP`' in normalized
    assert '`NEW RUN`' in normalized
    assert 'identity、 session、output、route、またはfieldの出どころが不明なら`STOP`' in normalized
    assert 'follow-up audit disposition:' in normalized
    assert 'original pair: matched / missing / mismatch' in normalized
    assert 'same run/output: matched / stop' in normalized
    assert 'evidence change: none — original report, receipt, and hash unchanged' in normalized
    assert 'disposition: MATCHED FOLLOW-UP / NEW RUN / STOP' in normalized
    assert '新しいvalidation resultやreceiptではありません' in normalized
    assert '複数issueへ同じnoteやsession artifactを重複添付せず' in normalized
    assert '`STOP`の結果をaccepted、再現成功、またはroadmap evidenceと 書かず' in normalized


def test_japanese_recovery_card_explains_safe_follow_up_actions():
    """The Japanese action card constrains each disposition after review."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語のvalidation reportのfollow-up監査結果を安全に分類する', 1
    )[1].split(
        '### validation reportのreview statusを区別する', 1
    )[0]
    normalized = ' '.join(card.split())
    compact = normalized.replace(' ', '')

    assert '分類の後に何をしてよいかも固定します' in normalized
    assert '`MATCHED FOLLOW-UP`' in normalized
    assert '`NEW RUN`' in normalized
    assert '`STOP`' in normalized
    assert '許可される対応' in normalized
    assert 'original report/receipt pairは変更せず' in normalized
    assert '新しいreceiptを作らない' in normalized
    assert '新しいrunのreport + reviewed receiptを1組だけ作り' in normalized
    assert '古いpairと混ぜない' in normalized
    assert '公開validationnoteを作らない' in compact
    assert 'identity、session、outputのどれかが不足する場合は`STOP`' in compact
    assert 'acceptedは、maintainer reviewとledgerの明示的なaccepted記録がある場合だけ書きます' in normalized
    assert 'follow-up action:' in normalized
    assert 'allowed action: <saved Details:/Next:/retry.command or new-run review>' in normalized
    assert 'original evidence: unchanged' in normalized
    assert 'new evidence: none / one new run report + reviewed receipt / none' in normalized
    assert (
        'duplicate check: <one pair per run; no duplicate issue or session '
        'artifact>' in normalized
    )
    assert '一つのrunにつきreport+reviewedreceiptは一組だけ' in compact
    assert '元のidentity、session、outputに結び付かない場合' in compact


def test_japanese_recovery_card_audits_follow_up_action_results():
    """The Japanese action-audit card separates outcome from disposition."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語のvalidation reportのfollow-up分類後の対応を監査可能にする', 1
    )[1].split(
        '### validation reportのreview statusを区別する', 1
    )[0]
    normalized = ' '.join(card.split())
    compact = normalized.replace(' ', '')

    assert '分類を記録しただけでは、実際に何をしたかは決まりません' in normalized
    assert '`action result`を別に記録' in normalized
    assert 'identity、session、outputを再確認できない場合' in normalized
    assert '`NOTE ONLY`' in normalized
    assert '`SUPPORT REQUESTED`' in normalized
    assert '`RETRY STARTED`' in normalized
    assert '`NEW PAIR PREPARED`' in normalized
    assert '`STOPPED`' in normalized
    assert '変更なし。元のpairを参照' in normalized
    assert '新しいreport、receipt、公開validation noteを作らない' in normalized
    assert 'acceptedは、maintainer reviewとledgerの明示的な' in normalized
    assert 'follow-up action audit:' in normalized
    assert 'allowed evidence change: none / one new report + reviewed receipt / none' in normalized
    assert 'new artifact count: 0 / one report + reviewed receipt / 0' in normalized
    assert 'identity/session/output: matched / missing / mismatch' in normalized
    assert 'audit result: completed / STOPPED — no acceptance' in normalized
    assert '既存のreportやreceiptを書き換えて整合させず' in normalized
    assert '一つのrunにつきreport+reviewedreceiptは一組だけ' in compact
    assert '複数issueへ同じnoteやsessionartifactは重複添付しません' in compact


def test_japanese_recovery_card_explains_reproducible_follow_up_handoff():
    """The Japanese handoff card keeps routing separate from acceptance."""
    source = (ROOT / 'docs' / 'getting-started-ja.md').read_text(
        encoding='utf-8')
    card = source.split(
        '### 日本語のvalidation reportのfollow-up実行結果を安全に引き継ぐ', 1
    )[1].split(
        '### validation reportのreview statusを区別する', 1
    )[0]
    normalized = ' '.join(card.split())
    compact = normalized.replace(' ', '')

    assert 'handoffは、次の担当者へ監査結果を渡すためのrouting' in normalized
    assert 'acceptanceやreceipt編集の許可ではありません' in compact
    assert '`READY FOR SUPPORT`' in normalized
    assert '`READY FOR REVIEW`' in normalized
    assert '`STOP` — not accepted' in normalized
    assert 'handoff target' in normalized
    assert 'owner role' in normalized
    assert 'next review' in normalized
    assert 'live guidanceだけをindependent validationと呼ぶ' in normalized
    assert 'ledger前にaccepted、PASS、roadmap evidenceと書く' in normalized
    assert '公開validation noteや新しいreceiptを作る' in normalized
    assert 'follow-up handoff:' in normalized
    assert 'original audit: <sanitized stable code; no private path>' in normalized
    assert 'handoff status: READY FOR SUPPORT / READY FOR REVIEW' in normalized
    assert 'privacy check: <no private path or artifact>' in normalized
    assert (
        'duplicate check: <one handoff per action audit; no duplicate issue '
        'or session artifact>' in normalized
    )
    assert '一つのactionauditにつきhandoffは一つ' in compact
    assert '一つのrunにつきreport+reviewedreceiptは一組だけ' in compact


def test_stale_task_cannot_be_rendered_copy_ready():
    """Stale task details are not emitted as a copy-ready issue body."""
    report = {
        'stale_tasks': [{
            'id': 'starter-C6',
            'status': 'STALE',
            'reasons': ['planned marker already exists'],
        }],
    }

    with pytest.raises(
        CHECKER.QueueError,
        match='starter-C6 is stale and must be reviewed before rendering',
    ):
        CHECKER._require_render_ready(report, 'starter-C6')


def test_docs_verifier_uses_temp_site_and_never_contract_shell(
    monkeypatch,
):
    """Docs verification writes only to a temporary site directory."""
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, '', '')

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)
    task = _task(_queue(), 'starter-C6')

    report = CHECKER.verify_task(task)

    assert report['status'] == 'FOCUSED_CHECKS_PASSED'
    assert report['workspace_artifacts_written'] is False
    assert len(calls) == 1
    command, kwargs = calls[0]
    assert command[:6] == [
        sys.executable, '-m', 'mkdocs', 'build', '--strict', '--site-dir'
    ]
    assert Path(command[6]).name == 'site'
    assert ROOT not in Path(command[6]).parents
    assert kwargs['env']['PYTHONDONTWRITEBYTECODE'] == '1'


def test_c5_verifier_uses_the_fixed_docs_profile(monkeypatch):
    """C5 verification follows the fixed built-in docs profile."""
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(command, 0, '', '')

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)
    task = _task(_queue(), 'starter-C5')

    report = CHECKER.verify_task(task)

    assert report['status'] == 'FOCUSED_CHECKS_PASSED'
    assert len(calls) == 1
    command, kwargs = calls[0]
    assert command[:6] == [
        sys.executable, '-m', 'mkdocs', 'build', '--strict', '--site-dir'
    ]
    assert Path(command[6]).name == 'site'
    assert ROOT not in Path(command[6]).parents
    assert kwargs['env']['PYTEST_ADDOPTS'] == '-p no:cacheprovider'


def test_code_verifier_stops_after_first_failure(monkeypatch):
    """A failed focused check prevents later checks from running."""
    calls = []

    def fake_run(command, **kwargs):
        calls.append(command)
        return subprocess.CompletedProcess(command, 1, '', '')

    monkeypatch.setattr(CHECKER.subprocess, 'run', fake_run)
    task = _task(_queue(), 'starter-C5')

    report = CHECKER.verify_task(task)

    assert report['status'] == 'FOCUSED_CHECKS_FAILED'
    assert len(calls) == 1
    assert report['checks'][0]['returncode'] == 1


def test_default_cli_json_is_path_private_and_no_write():
    """Default machine output contains no private path or write claim."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    payload = json.loads(result.stdout)
    assert payload['status'] == 'QUEUE_READY_LOCAL_ONLY'
    assert payload['ready_task_ids'] == EXPECTED_READY_IDS
    assert payload['stale_tasks'] == []
    assert payload['authority']['github_writes_authorized'] is False
    assert '/home/' not in result.stdout
    assert '/tmp/' not in result.stdout
    assert result.stderr == ''


def test_list_cli_names_every_task_and_publication_boundary():
    """The list view keeps all five tasks behind the publication gate."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--list'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'PREPARED_NOT_PUBLISHED' in result.stdout
    assert 'Wait for a published issue' in result.stdout
    for task_id in CHECKER.EXPECTED_TASK_IDS:
        assert task_id in result.stdout
    assert result.stderr == ''
