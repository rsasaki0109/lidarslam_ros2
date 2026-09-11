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
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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

"""Tests for the fail-closed issue-triage application packet."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'prepare_issue_triage_application.py'
PROPOSAL_PATH = (
    ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'open-issue-triage-proposal-2026-08-11.json'
)
PROPOSAL_SCHEMA_PATH = (
    ROOT / 'docs' / 'schemas' / 'issue-triage-proposal-v1.schema.json'
)
PACKET_SCHEMA_PATH = (
    ROOT
    / 'docs'
    / 'schemas'
    / 'issue-triage-application-packet-v1.schema.json'
)
PUBLIC_DRAFT_HEAD = '4b2ab514a4f33b443e2c4283b3114d11a5e44e49'
FIX_COMMIT = 'a2368c486fc35c0edcac6d9dbf2f9cb89475c820'
LATEST_STABLE_COMMIT = '0df0c4a86df9f68a894c83f8342e4107c3d23b0f'
SPEC = importlib.util.spec_from_file_location(
    'prepare_issue_triage_application',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
APPLICATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(APPLICATION)


def _load(path: Path) -> dict:
    """Load one tracked JSON object."""
    payload = json.loads(path.read_text(encoding='utf-8'))
    assert isinstance(payload, dict)
    return payload


def _proposal() -> dict:
    """Return the tracked proposal."""
    return _load(PROPOSAL_PATH)


def _proposal_schema() -> dict:
    """Return the tracked proposal schema."""
    return _load(PROPOSAL_SCHEMA_PATH)


def _packet_schema() -> dict:
    """Return the tracked packet schema."""
    return _load(PACKET_SCHEMA_PATH)


def _packet(
    *,
    issue_number=None,
    live_status='NOT_RUN',
    linked_results=None,
) -> dict:
    """Build one packet from tracked sources."""
    return APPLICATION.build_packet(
        _proposal(),
        _proposal_schema(),
        _packet_schema(),
        issue_number=issue_number,
        live_status=live_status,
        linked_results=linked_results,
    )


def _action(packet: dict, issue_number: int) -> dict:
    """Return one issue action from a packet."""
    return next(
        item for item in packet['actions']
        if item['issue_number'] == issue_number
    )


def _live_snapshot(proposal: dict) -> tuple[list[dict], list[str]]:
    """Build the exact API-shaped fields retained by the proposal."""
    issues = [{
        'issue_number': item['issue_number'],
        'title': item['title'],
        'updated_at': item['observed']['updated_at'],
        'labels': list(item['observed']['labels']),
    } for item in proposal['issues']]
    return issues, list(proposal['label_catalog'])


def _product_draft_audit() -> dict:
    """Return the exact bounded result from the existing GET-only audit."""
    return {
        'pull_request': 427,
        'status': 'DRAFT_REVIEW_REQUIRED',
        'remote_head': PUBLIC_DRAFT_HEAD,
        'head_matches_local': True,
        'state': 'OPEN',
        'is_draft': True,
        'mergeable': True,
        'passing_check_count': 10,
        'skipped_check_count': 4,
        'pending_check_count': 0,
        'failing_check_count': 0,
        'required_checks_complete': True,
        'blockers': [],
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'merge_authorized': False,
            'remote_mutations_performed': False,
        },
    }


def _draft_result() -> dict:
    """Return the privacy-bounded packet form of the Draft audit."""
    observed = _product_draft_audit()
    return {
        'id': 'issue-69-public-draft',
        'issue_number': 69,
        'kind': 'product-draft',
        'status': 'PASS',
        'remote_head': observed['remote_head'],
        'state': observed['state'],
        'is_draft': observed['is_draft'],
        'mergeable': observed['mergeable'],
        'passing_check_count': observed['passing_check_count'],
        'skipped_check_count': observed['skipped_check_count'],
        'pending_check_count': observed['pending_check_count'],
        'failing_check_count': observed['failing_check_count'],
        'required_checks_complete': observed['required_checks_complete'],
        'authority': observed['authority'],
    }


def _release_responses() -> dict[str, tuple[int, dict]]:
    """Return exact API-shaped stable-release and candidate-tag reads."""
    repository = 'rsasaki0109/lidar_slam_ros2'
    return {
        f'repos/{repository}/releases/latest': (200, {
            'tag_name': 'v0.9.0',
            'draft': False,
            'prerelease': False,
            'html_url': (
                f'https://github.com/{repository}/releases/tag/v0.9.0'
            ),
        }),
        f'repos/{repository}/commits/v0.9.0': (200, {
            'sha': LATEST_STABLE_COMMIT,
        }),
        f'repos/{repository}/compare/{FIX_COMMIT}...v0.9.0': (200, {
            'status': 'behind',
            'ahead_by': 0,
            'behind_by': 52,
            'total_commits': 0,
            'merge_base_commit': {'sha': LATEST_STABLE_COMMIT},
        }),
        f'repos/{repository}/git/ref/tags/v0.9.1': (404, {
            'message': 'Not Found',
        }),
        f'repos/{repository}/releases/tags/v0.9.1': (404, {
            'message': 'Not Found',
        }),
    }


def _release_result() -> dict:
    """Return the privacy-bounded packet form of the release audit."""
    return {
        'id': 'issue-69-stable-release-absence',
        'issue_number': 69,
        'kind': 'stable-release-absence',
        'status': 'PASS',
        'fix_commit': FIX_COMMIT,
        'latest_stable_tag': 'v0.9.0',
        'latest_stable_commit': LATEST_STABLE_COMMIT,
        'latest_stable_relation': 'behind',
        'latest_stable_behind_by': 52,
        'candidate_tag': 'v0.9.1',
        'candidate_tag_present': False,
        'candidate_release_present': False,
        'authority': dict(APPLICATION.LINKED_CHECK_AUTHORITY),
    }


def _linked_results() -> list[dict]:
    """Return both source-ordered linked results for issue #69."""
    return [_draft_result(), _release_result()]


def test_complete_packet_is_schema_valid_ordered_and_unauthorized():
    """All 29 rows become deterministic review actions without authority."""
    packet = _packet()

    APPLICATION.validate_packet(packet, _packet_schema())
    assert packet['status'] == 'PREPARED_NOT_AUTHORIZED'
    assert packet['selection'] == {
        'requested_issue_number': None,
        'proposal_issue_count': 29,
        'selected_action_count': 29,
    }
    assert packet['summary'] == {
        'close_action_count': 23,
        'reproduction_request_count': 4,
        'dependency_review_count': 9,
        'monitor_only_count': 1,
    }
    assert packet['linked_check'] == {
        'performed': False,
        'status': 'NOT_RUN',
        'claim_count': 2,
        'results': [],
    }
    assert [item['issue_number'] for item in packet['actions'][:6]] == [
        69, 422, 64, 104, 106, 124,
    ]
    assert [item['issue_number'] for item in packet['actions'][6:]] == sorted(
        item['issue_number'] for item in packet['actions'][6:]
    )
    assert packet['authority'] == {
        'github_requests': 'NONE',
        'github_writes_authorized': False,
        'issue_comments_authorized': False,
        'issue_label_changes_authorized': False,
        'issue_state_changes_authorized': False,
        'local_files_written': False,
        'remote_mutations_performed': False,
    }


def test_issue_69_is_a_reviewed_keep_open_help_request():
    """The crash-safety issue is not accidentally closed."""
    action = _action(_packet(), 69)

    assert action['content_status'] == 'READY_FOR_MAINTAINER_REVIEW'
    assert action['proposed_changes'] == {
        'labels_to_add': ['help wanted'],
        'comment_required': True,
        'close_issue': False,
        'state_reason': None,
    }
    assert action['prerequisites']['linked_claim_ids'] == [
        'issue-69-public-draft',
        'issue-69-stable-release-absence',
    ]
    assert len(action['evidence']) == 6
    assert all(
        item['kind'] == 'repository_file' and len(item['sha256']) == 64
        for item in action['evidence']
    )
    response = action['public_response_draft']
    assert 'What we found:' in response
    assert 'Current status: **keeping this issue open**.' in response
    assert 'privacy-safe synthetic points' in response
    assert 'public Draft PR #427' in response
    assert 'passes Humble and Jazzy CI' in response
    assert 'No named stable release contains this reviewed fix commit yet' in (
        response
    )
    assert '`vg_size_for_map`' in response
    assert '`vg_size_for_input`' in response
    assert 'only a workaround' in response
    assert 'original 2 GB bag is not required' in response
    assert 'fix remains local' not in response
    assert 'lacks supported public CI' not in response


def test_issue_422_remains_monitor_only():
    """Independent validation is preserved without a support intervention."""
    action = _action(_packet(), 422)

    assert action['content_status'] == 'MONITOR_ONLY'
    assert action['proposed_changes'] == {
        'labels_to_add': [],
        'comment_required': False,
        'close_issue': False,
        'state_reason': None,
    }
    assert _packet(issue_number=422)['linked_check'] == {
        'performed': False,
        'status': 'NOT_REQUIRED',
        'claim_count': 0,
        'results': [],
    }


def test_dependency_rows_are_explicitly_blocked_for_review():
    """Starter dependencies cannot be flattened into a blind close batch."""
    actions = [
        item for item in _packet()['actions']
        if item['content_status'] == 'DEPENDENCY_REVIEW_REQUIRED'
    ]

    assert [item['issue_number'] for item in actions] == [
        106, 98, 102, 105, 108, 111, 112, 115, 122,
    ]
    assert all(item['prerequisites']['dependency_ids'] for item in actions)


def test_single_issue_selection_is_bounded_and_unknown_issue_fails():
    """A maintainer can review one known issue but cannot invent a row."""
    packet = _packet(issue_number=104)

    assert packet['selection']['selected_action_count'] == 1
    assert [item['issue_number'] for item in packet['actions']] == [104]
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='is not in the proposal',
    ):
        _packet(issue_number=999)


def test_response_drafts_are_issue_specific_and_contain_no_mutation_command():
    """Each draft cites its rationale, next step, and immutable evidence."""
    packet = _packet()
    rendered = APPLICATION.render_packet(packet)

    for action in packet['actions']:
        response = action['public_response_draft']
        assert action['rationale'] in response
        assert action['response_summary'] in response
        assert 'Evidence reviewed:' in response
    assert 'review aid' in rendered
    assert 'Linked PR/CI check: **NOT_RUN**' in rendered
    assert 'Linked claims to recheck: `issue-69-public-draft`' in rendered
    assert 'does not authorize posting, labeling, or closing' in rendered
    assert 'gh issue' not in rendered
    assert 'PATCH ' not in rendered


def test_external_and_repository_evidence_stay_inside_bounded_sources():
    """Accept only tracked files and a clean github.com repository URL."""
    action = _action(_packet(), 118)

    assert action['evidence'][0] == {
        'kind': 'external_url',
        'url': 'https://github.com/rsasaki0109/lidar_localization_ros2',
    }
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='outside the GitHub boundary',
    ):
        APPLICATION._evidence_record('https://example.com/evidence')
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='unavailable',
    ):
        APPLICATION._evidence_record('docs/missing-evidence.md')


def test_packet_retains_no_raw_github_content_or_identity():
    """Live reads do not become a durable raw issue archive."""
    packet = _packet()

    assert packet['privacy'] == {
        'author_identities_retained': False,
        'comment_bodies_requested': False,
        'issue_bodies_retained': False,
        'raw_github_records_written': False,
    }
    serialized = json.dumps(packet, sort_keys=True)
    assert '"author"' not in serialized
    assert '"comments"' not in serialized

    live_packet = _packet(
        issue_number=69,
        live_status='PASS',
        linked_results=_linked_results(),
    )
    linked_serialized = json.dumps(
        live_packet['linked_check'],
        sort_keys=True,
    )
    assert 'tag_name' not in linked_serialized
    assert 'html_url' not in linked_serialized
    assert 'merge_base_commit' not in linked_serialized
    assert 'message' not in linked_serialized


@pytest.mark.parametrize(
    ('mutate', 'message'),
    [
        (
            lambda packet: packet['actions'][1].update(order=1),
            'order must be consecutive',
        ),
        (
            lambda packet: packet['summary'].update(close_action_count=0),
            'summary is inconsistent',
        ),
        (
            lambda packet: (
                packet['actions'][0]['proposed_changes'].update(
                    close_issue=True
                ),
                packet['summary'].update(close_action_count=24),
            ),
            'close decision is inconsistent',
        ),
        (
            lambda packet: packet['actions'][0].update(
                public_response_draft='generic response'
            ),
            'response draft is not issue-specific',
        ),
        (
            lambda packet: packet['actions'][0]['evidence'][0].update(
                sha256='0' * 64
            ),
            'evidence has drifted',
        ),
        (
            lambda packet: packet['authority'].update(
                github_requests='GET_ONLY'
            ),
            'request mode is inconsistent',
        ),
    ],
)
def test_semantic_validator_rejects_packet_drift(mutate, message):
    """Schema-shaped but internally inconsistent packets fail closed."""
    packet = copy.deepcopy(_packet())
    mutate(packet)

    with pytest.raises(APPLICATION.ApplicationPacketError, match=message):
        APPLICATION.validate_packet(packet, _packet_schema())


def test_source_proposal_and_generator_hashes_are_rechecked():
    """A packet cannot detach itself from either implementation input."""
    packet = copy.deepcopy(_packet())
    packet['source']['proposal_canonical_sha256'] = '0' * 64
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='source proposal hash has drifted',
    ):
        APPLICATION.validate_packet(packet, _packet_schema())

    packet = copy.deepcopy(_packet())
    packet['source']['generator_sha256'] = '0' * 64
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='source generator hash has drifted',
    ):
        APPLICATION.validate_packet(packet, _packet_schema())


def test_schema_refuses_any_claimed_write_authority():
    """No Boolean edit can turn this review artifact into write approval."""
    packet = copy.deepcopy(_packet())
    packet['authority']['github_writes_authorized'] = True

    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='packet schema failed',
    ):
        APPLICATION.validate_packet(packet, _packet_schema())


def test_live_main_uses_get_only_snapshot_and_emits_pass(monkeypatch, capsys):
    """An exact live snapshot changes only the request audit mode."""
    proposal = _proposal()
    live_issues, live_labels = _live_snapshot(proposal)
    issue_calls = []
    draft_calls = []
    release_calls = []
    release_responses = _release_responses()

    def fake_fetch(repository):
        issue_calls.append(repository)
        return live_issues, live_labels

    def fake_draft_audit(*, local_head):
        draft_calls.append(local_head)
        return _product_draft_audit()

    def fake_github_json(path):
        release_calls.append(path)
        return release_responses[path]

    monkeypatch.setattr(
        APPLICATION.PROPOSAL_CHECKER,
        'fetch_live_snapshot',
        fake_fetch,
    )
    monkeypatch.setattr(
        APPLICATION.G0_READINESS,
        'audit_product_draft',
        fake_draft_audit,
    )
    monkeypatch.setattr(
        APPLICATION.G0_READINESS,
        '_github_json',
        fake_github_json,
    )

    assert APPLICATION.main(['--live', '--issue', '69', '--json']) == 0
    packet = json.loads(capsys.readouterr().out)
    assert issue_calls == ['rsasaki0109/lidar_slam_ros2']
    assert draft_calls == [PUBLIC_DRAFT_HEAD]
    assert release_calls == list(release_responses)
    assert packet['live_check'] == {'performed': True, 'status': 'PASS'}
    assert packet['linked_check'] == {
        'performed': True,
        'status': 'PASS',
        'claim_count': 2,
        'results': _linked_results(),
    }
    assert packet['authority']['github_requests'] == 'GET_ONLY'
    assert packet['authority']['remote_mutations_performed'] is False


@pytest.mark.parametrize(
    ('field', 'drifted_value'),
    [
        ('remote_head', '1' * 40),
        ('status', 'READY'),
        ('passing_check_count', 9),
        ('authority', {
            'network_reads_performed': True,
            'github_writes_authorized': True,
            'merge_authorized': False,
            'remote_mutations_performed': False,
        }),
    ],
)
def test_linked_draft_claim_drift_fails_closed(field, drifted_value):
    """Head, status, check-count, or authority drift blocks the response."""
    observed = _product_draft_audit()
    observed[field] = drifted_value

    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match=f'linked claim issue-69-public-draft drifted: {field}',
    ):
        APPLICATION._verify_linked_claims(
            [
                issue for issue in _proposal()['issues']
                if issue['issue_number'] == 69
            ],
            auditor=lambda **kwargs: observed,
        )


@pytest.mark.parametrize(
    ('response_key', 'payload_key', 'drifted_value', 'message'),
    [
        ('latest', 'tag_name', 'v0.9.1', 'latest_tag'),
        ('stable_commit', 'sha', '1' * 40, 'stable_commit'),
        ('relation', 'behind_by', 51, 'behind_by'),
        ('candidate_tag', None, 200, 'candidate_tag_http_status'),
        (
            'candidate_release',
            None,
            200,
            'candidate_release_http_status',
        ),
    ],
)
def test_linked_stable_release_drift_fails_closed(
    response_key,
    payload_key,
    drifted_value,
    message,
):
    """A new release or changed stable ancestry invalidates the response."""
    responses = _release_responses()
    path = next(
        item for item in responses
        if (
            response_key == 'latest' and item.endswith('/releases/latest')
        ) or (
            response_key == 'stable_commit'
            and item.endswith('/commits/v0.9.0')
        ) or (
            response_key == 'relation' and '/compare/' in item
        ) or (
            response_key == 'candidate_tag' and '/git/ref/tags/' in item
        ) or (
            response_key == 'candidate_release'
            and item.endswith('/releases/tags/v0.9.1')
        )
    )
    status, payload = responses[path]
    if payload_key is None:
        responses[path] = (drifted_value, payload)
    else:
        payload[payload_key] = drifted_value
        responses[path] = (status, payload)

    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match=(
            'linked claim issue-69-stable-release-absence drifted: '
            f'{message}'
        ),
    ):
        APPLICATION._verify_linked_claims(
            [
                issue for issue in _proposal()['issues']
                if issue['issue_number'] == 69
            ],
            auditor=lambda **kwargs: _product_draft_audit(),
            fetcher=lambda path: responses[path],
        )


def test_linked_drift_main_emits_no_false_packet(monkeypatch, capsys):
    """A changed public Draft head prevents all packet output."""
    proposal = _proposal()
    live_issues, live_labels = _live_snapshot(proposal)
    observed = _product_draft_audit()
    observed['remote_head'] = '1' * 40
    monkeypatch.setattr(
        APPLICATION.PROPOSAL_CHECKER,
        'fetch_live_snapshot',
        lambda repository: (live_issues, live_labels),
    )
    monkeypatch.setattr(
        APPLICATION.G0_READINESS,
        'audit_product_draft',
        lambda **kwargs: observed,
    )

    assert APPLICATION.main(['--live', '--issue', '69', '--json']) == 1
    captured = capsys.readouterr()
    assert captured.out == ''
    assert 'linked claim issue-69-public-draft drifted' in captured.err
    assert 'remote_head' in captured.err


def test_linked_release_drift_main_emits_no_false_packet(
    monkeypatch,
    capsys,
):
    """A newly visible candidate release prevents all packet output."""
    proposal = _proposal()
    live_issues, live_labels = _live_snapshot(proposal)
    responses = _release_responses()
    release_path = next(
        path for path in responses
        if path.endswith('/releases/tags/v0.9.1')
    )
    responses[release_path] = (200, {'tag_name': 'v0.9.1'})
    monkeypatch.setattr(
        APPLICATION.PROPOSAL_CHECKER,
        'fetch_live_snapshot',
        lambda repository: (live_issues, live_labels),
    )
    monkeypatch.setattr(
        APPLICATION.G0_READINESS,
        'audit_product_draft',
        lambda **kwargs: _product_draft_audit(),
    )
    monkeypatch.setattr(
        APPLICATION.G0_READINESS,
        '_github_json',
        lambda path: responses[path],
    )

    assert APPLICATION.main(['--live', '--issue', '69', '--json']) == 1
    captured = capsys.readouterr()
    assert captured.out == ''
    assert 'issue-69-stable-release-absence drifted' in captured.err
    assert 'candidate_release_http_status' in captured.err


def test_linked_packet_result_is_source_bound_and_live_bound():
    """A shaped PASS cannot detach from the proposal or live issue check."""
    packet = _packet(
        issue_number=69,
        live_status='PASS',
        linked_results=_linked_results(),
    )
    packet['linked_check']['results'][0]['remote_head'] = '1' * 40
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='result is inconsistent',
    ):
        APPLICATION.validate_packet(packet, _packet_schema())

    packet = _packet(issue_number=69)
    packet['linked_check'] = {
        'performed': True,
        'status': 'PASS',
        'claim_count': 2,
        'results': _linked_results(),
    }
    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='linked check status is inconsistent',
    ):
        APPLICATION.validate_packet(packet, _packet_schema())


def test_linked_packet_results_cannot_be_reordered():
    """Both shaped results remain ordered by their source proposal claims."""
    results = list(reversed(_linked_results()))

    with pytest.raises(
        APPLICATION.ApplicationPacketError,
        match='linked check results are incomplete or reordered',
    ):
        _packet(
            issue_number=69,
            live_status='PASS',
            linked_results=results,
        )


def test_live_drift_returns_failure_without_a_false_packet(
    monkeypatch,
    capsys,
):
    """Any open-issue drift prevents packet output."""
    proposal = _proposal()
    live_issues, live_labels = _live_snapshot(proposal)
    live_issues.pop()
    monkeypatch.setattr(
        APPLICATION.PROPOSAL_CHECKER,
        'fetch_live_snapshot',
        lambda repository: (live_issues, live_labels),
    )

    assert APPLICATION.main(['--live', '--json']) == 1
    captured = capsys.readouterr()
    assert captured.out == ''
    assert 'open-issue set drifted' in captured.err


def test_cli_prints_json_and_human_review_cards_without_output_option():
    """The CLI is stdout-only and has no file-writing mode."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--issue', '69', '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['actions'][0]['issue_number'] == 69

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--issue', '422'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert 'MONITOR_ONLY' in result.stdout
    assert 'mutation command' in result.stdout

    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--output', 'packet.json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 2
    assert 'unrecognized arguments: --output' in result.stderr


def test_generator_contains_no_remote_write_adapter():
    """The generator delegates only the existing GET-only live snapshot."""
    source = SCRIPT.read_text(encoding='utf-8')

    assert 'fetch_live_snapshot' in source
    assert "'POST'" not in source
    assert "'PATCH'" not in source
    assert "'PUT'" not in source
    assert "'DELETE'" not in source
    assert 'subprocess' not in source


def test_application_packet_is_registered_and_bundled():
    """Ctest and release evidence include the exact review implementation."""
    cmake = (ROOT / 'lidarslam' / 'CMakeLists.txt').read_text(
        encoding='utf-8'
    )
    builder = (ROOT / 'scripts' / 'build_release_bundle.py').read_text(
        encoding='utf-8'
    )

    assert 'test_prepare_issue_triage_application' in cmake
    assert "'scripts/prepare_issue_triage_application.py'" in builder
