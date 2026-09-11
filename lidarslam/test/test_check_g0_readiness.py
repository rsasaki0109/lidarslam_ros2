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
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for the local, read-only G0 readiness dashboard."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import pathlib
import re
import subprocess

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_g0_readiness.py'
SPEC = importlib.util.spec_from_file_location('check_g0_readiness', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
DASHBOARD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DASHBOARD)


def _absent_environment_handoff() -> dict:
    """Return the exact bounded admin handoff expected from the preflight."""
    return {
        'kind': 'CREATE_AND_REVIEW_ENVIRONMENT',
        'authority_required': 'repository-settings-admin',
        'external_write_required': True,
        'settings_url': (
            'https://github.com/rsasaki0109/lidar_slam_ros2/'
            'settings/environments'
        ),
        'steps': [
            'Create an environment named candidate-images.',
            'Add 1–6 trusted User or Team required reviewers.',
            'Enable Prevent self-review.',
            (
                'Select custom deployment branches and allow exactly the '
                'develop branch.'
            ),
            'Have an independent maintainer review the saved settings.',
        ],
        'verification_command': (
            'python3 scripts/check_candidate_environment.py '
            '--json --require-ready'
        ),
        'writes_performed': False,
    }


def _product_pull(
    head: str,
    *,
    body: str | None = '',
    draft: bool = True,
    state: str = 'open',
    merged: bool = False,
    mergeable: bool | None = True,
) -> dict:
    """Return the bounded PR identity consumed by the live audit."""
    return {
        'number': 427,
        'html_url': 'https://github.com/rsasaki0109/lidar_slam_ros2/pull/427',
        'state': state,
        'draft': draft,
        'merged': merged,
        'mergeable': mergeable,
        'body': body,
        'head': {
            'sha': head,
            'ref': 'agent/product-g0-guided-ux',
            'repo': {'full_name': 'rsasaki0109/lidar_slam_ros2'},
        },
        'base': {
            'ref': 'develop',
            'repo': {'full_name': 'rsasaki0109/lidar_slam_ros2'},
        },
    }


def _product_checks(*, failed_name: str | None = None) -> dict:
    """Return the exact expected successful and non-publication check set."""
    runs = []
    run_id = 1
    for name in sorted(DASHBOARD.REQUIRED_SUCCESS_CHECKS):
        runs.append({
            'id': run_id,
            'name': name,
            'status': 'completed',
            'conclusion': 'failure' if name == failed_name else 'success',
        })
        run_id += 1
    for name in sorted(DASHBOARD.REQUIRED_SKIPPED_CHECKS):
        runs.append({
            'id': run_id,
            'name': name,
            'status': 'completed',
            'conclusion': 'skipped',
        })
        run_id += 1
    return {'total_count': len(runs), 'check_runs': runs}


def _audit_product(
    head: str,
    *,
    body: str | None = '',
    draft: bool = True,
    state: str = 'open',
    merged: bool = False,
    mergeable: bool | None = True,
    failed_name: str | None = None,
) -> dict:
    """Run the PR audit against deterministic GET fixtures."""
    def fetcher(path: str):
        if path.endswith('/pulls/427'):
            return 200, _product_pull(
                head,
                body=body,
                draft=draft,
                state=state,
                merged=merged,
                mergeable=mergeable,
            )
        assert path.endswith(f'/commits/{head}/check-runs?per_page=100')
        return 200, _product_checks(failed_name=failed_name)

    return DASHBOARD.audit_product_draft(
        fetcher=fetcher,
        local_head=head,
    )


def _desired_description(reports: dict, head: str) -> str:
    """Render the canonical body from deterministic local fixtures."""
    navigation = DASHBOARD._publication_review_navigation_summary(
        reports['publication_overview'],
        reports['publication_plan'],
    )
    return DASHBOARD._product_draft_description_body(
        reports['publication_plan'],
        navigation,
        DASHBOARD._review_routing_summary(
            reports['product_draft_review_routing'],
            reports['publication_plan'],
            navigation,
        ),
        DASHBOARD._matrix_summary(reports['onboarding_matrix']),
        DASHBOARD._cohort_summary(reports['first_map_cohort']),
        DASHBOARD._v1_summary(reports['v1_readiness']),
        head,
    )


def _bind_review_tip(reports: dict, head: str) -> None:
    """Keep synthetic plan and validated-overview fixtures on one tip."""
    reports['publication_plan']['local_tip_sha'] = head
    reports['publication_overview']['candidate']['local_tip_sha'] = head
    reports['publication_overview']['review_phases'][-1]['end_sha'] = head
    reports['product_draft_review_routing']['exact_head'] = head


def _set_review_worktree_state(
    reports: dict,
    *,
    clean: bool,
    uncommitted_path_count: int,
) -> None:
    """Keep synthetic plan and overview cleanliness evidence aligned."""
    reports['publication_plan']['worktree_clean'] = clean
    reports['publication_plan'][
        'uncommitted_path_count'
    ] = uncommitted_path_count
    reports['publication_overview']['candidate']['worktree_clean'] = clean
    reports['publication_overview']['candidate'][
        'uncommitted_path_count'
    ] = uncommitted_path_count
    routing = reports['product_draft_review_routing']
    routing['status'] = (
        'READY_LOCAL_ONLY' if clean else 'PREPARED_DIRTY_WORKTREE'
    )
    routing['worktree_clean'] = clean
    routing['uncommitted_path_count'] = uncommitted_path_count


def _review_ledger_report(reports: dict, states: list[str]) -> dict:
    """Return one schema-shaped identity-free ledger checker fixture."""
    assert len(states) == 4
    routing = reports['product_draft_review_routing']
    reviewed = sum(state != 'NOT_REVIEWED' for state in states)
    passing = states.count('PASS')
    blocked = states.count('BLOCKED')
    if reviewed == 0:
        status = 'EMPTY_LOCAL_LEDGER'
    elif blocked:
        status = 'BLOCKED_LOCAL_REVIEW'
    elif passing == 4:
        status = 'COMPLETE_LOCAL_REVIEW'
    else:
        status = 'IN_PROGRESS_LOCAL_REVIEW'
    current_lanes = []
    sequence = 0
    for lane, state in zip(routing['lanes'], states):
        if state != 'NOT_REVIEWED':
            sequence += 1
        current_lanes.append({
            'id': lane['id'],
            'order': lane['order'],
            'slice_ids': list(lane['slice_ids']),
            'status': state,
            'verification_status': (
                'NOT_RECORDED'
                if state == 'NOT_REVIEWED'
                else ('PASS' if state == 'PASS' else 'FAIL')
            ),
            'latest_event_sequence': (
                sequence if state != 'NOT_REVIEWED' else None
            ),
            'finding_count': 1 if state == 'BLOCKED' else 0,
            'blocker_count': 1 if state == 'BLOCKED' else 0,
        })
    next_lane_id = next(
        (
            lane['id']
            for lane, state in zip(routing['lanes'], states)
            if state != 'PASS'
        ),
        None,
    )
    return {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
            'product-draft-review-ledger-report-v1.schema.json'
        ),
        'repository': 'rsasaki0109/lidar_slam_ros2',
        'pull_request': 427,
        'scope': 'anonymous-product-draft-review-ledger-report',
        'status': status,
        'exact_head': reports['publication_plan']['local_tip_sha'],
        'ledger_sha256': '3' * 64,
        'routing_contract_sha256': '4' * 64,
        'worktree_clean': True,
        'event_count': reviewed,
        'reviewed_lane_count': reviewed,
        'passing_lane_count': passing,
        'blocked_lane_count': blocked,
        'historical_finding_count': blocked,
        'current_finding_count': blocked,
        'open_blocker_count': blocked,
        'current_lanes': current_lanes,
        'next_lane_id': next_lane_id,
        'authority': {
            'identities_collected': False,
            'review_commands_executed_by_tool': False,
            'github_reviewer_requests_authorized': False,
            'github_reviews_authorized': False,
            'mark_ready_authorized': False,
            'merge_authorized': False,
            'remote_mutations_performed': False,
        },
    }


def test_current_dashboard_preserves_the_tracked_hold_state():
    """The current local evidence remains an honest G0 HOLD."""
    reports = DASHBOARD.collect_checker_reports()
    report = DASHBOARD.build_report(reports)

    assert report['status'] == 'HOLD'
    assert report['authority'] == {
        'network_reads_performed': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    assert report['checks']['publication_plan']['status'] == (
        'PLAN_VALID_LOCAL_ONLY'
    )
    assert report['checks']['publication_plan']['path_count'] == 349
    assert report['checks']['publication_plan'][
        'whole_pr_commit_count'
    ] >= 315
    assert report['checks']['publication_plan'][
        'follow_up_review_commit_count'
    ] >= 271
    assert report['checks']['publication_plan']['whole_pr_path_count'] == 396
    assert report['checks']['publication_plan']['review_phase_count'] == 3
    assert report['checks']['publication_plan'][
        'review_coverage_complete'
    ] is True
    assert report['checks']['publication_plan']['bridge_path_count'] == 11
    navigation = report['checks']['publication_review_navigation']
    assert navigation['status'] == 'READY_LOCAL_ONLY'
    assert navigation['exact_head'] == report['checks'][
        'publication_plan'
    ]['local_tip_sha']
    assert [item['id'] for item in navigation['phases']] == [
        'P0-initial-review',
        'P1-ci-bridge',
        'P2-follow-up-slices',
    ]
    assert all(
        item['compare_url'].startswith(
            'https://github.com/rsasaki0109/lidar_slam_ros2/compare/'
        )
        for item in navigation['phases']
    )
    assert [item['id'] for item in navigation['slices']] == [
        'S1-runtime-safety',
        'S2-first-map-foundation',
        'S3-map-lifecycle',
        'S4-source-onboarding',
        'S5-distribution-readiness',
        'S6-product-shell-integration',
        'S7-publication-control',
    ]
    assert sum(item['path_count'] for item in navigation['slices']) == 349
    assert navigation['commands_executed'] is False
    assert navigation['github_writes_authorized'] is False
    routing = report['checks']['product_draft_review_routing']
    expected_routing_status = (
        'READY_LOCAL_ONLY'
        if report['checks']['publication_plan']['worktree_clean']
        else 'PREPARED_DIRTY_WORKTREE'
    )
    assert routing['status'] == expected_routing_status
    assert routing['worktree_clean'] is report['checks'][
        'publication_plan'
    ]['worktree_clean']
    assert routing['uncommitted_path_count'] == report['checks'][
        'publication_plan'
    ]['uncommitted_path_count']
    assert [item['id'] for item in routing['lanes']] == [
        'R1-runtime-safety',
        'R2-operator-ux',
        'R3-distribution',
        'R4-integration-publication',
    ]
    assert routing['summary'] == {
        'lane_count': 4,
        'slice_count': 7,
        'path_count': 349,
        'verification_count': 34,
        'unassigned_slice_count': 0,
        'duplicate_slice_count': 0,
    }
    assert routing['policy']['advisory_reviewer_target'] == 2
    assert routing['policy']['advisory_target_is_merge_gate'] is False
    assert routing['authority']['github_reviewer_requests_authorized'] is False
    assert routing['authority']['github_reviews_authorized'] is False
    assert report['checks']['product_draft_review_ledger'] == {
        'status': 'NOT_CHECKED',
        'exact_head': None,
        'ledger_sha256': None,
        'routing_contract_sha256': None,
        'worktree_clean': None,
        'event_count': 0,
        'reviewed_lane_count': 0,
        'passing_lane_count': 0,
        'blocked_lane_count': 0,
        'historical_finding_count': 0,
        'current_finding_count': 0,
        'open_blocker_count': 0,
        'current_lanes': [],
        'next_lane_id': None,
        'authority': {
            'identities_collected': False,
            'review_commands_executed_by_tool': False,
            'github_reviewer_requests_authorized': False,
            'github_reviews_authorized': False,
            'mark_ready_authorized': False,
            'merge_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    assert report['checks']['onboarding_matrix']['comparable_rows'] == 0
    assert report['checks']['published_release']['status'] == 'NOT_CHECKED'
    assert report['checks']['product_draft'] == {
        'status': 'NOT_CHECKED',
        'pull_request': 427,
        'url': 'https://github.com/rsasaki0109/lidar_slam_ros2/pull/427',
        'state': 'NOT_CHECKED',
        'is_draft': None,
        'merged': None,
        'mergeable': None,
        'base_ref': 'develop',
        'head_ref': 'agent/product-g0-guided-ux',
        'local_head': None,
        'remote_head': None,
        'head_matches_local': None,
        'non_force_update_possible': None,
        'remote_description_sha256': None,
        'observed_check_count': 0,
        'passing_check_count': 0,
        'skipped_check_count': 0,
        'pending_check_count': 0,
        'failing_check_count': 0,
        'required_checks_complete': None,
        'blockers': [],
        'decision_state': 'NOT_CHECKED',
        'merge_authorized': False,
    }
    assert report['checks']['candidate_environment'] == {
        'status': 'NOT_CHECKED',
        'environment': 'candidate-images',
        'target_present': None,
        'required_reviewer_count': None,
        'prevent_self_review': None,
        'deployment_branch_policy': None,
        'decision_state': 'NOT_CHECKED',
        'blockers': [],
        'operator_handoff': None,
    }
    v1_details = report['checks']['v1_readiness']['incomplete_gate_details']
    assert [item['id'] for item in v1_details] == [
        'distribution',
        'external-adoption',
    ]
    assert any(
        'ndt_omp' in blocker
        for item in v1_details
        for blocker in item['blockers']
    )
    assert report['next_action']['id'] == 'align-public-product-version'
    assert '--include-public-transition' in report['next_action']['command']
    assert 'mixed-version rows' in report['next_action']['reason']
    alternatives = report['next_action']['alternatives']
    assert [item['id'] for item in alternatives] == [
        'continue-current-candidate',
        'rebuild-against-published-version',
    ]
    assert alternatives[0]['status'] == 'REQUIRES_EXTERNAL_PUBLICATION'
    assert alternatives[1]['status'] == 'BLOCKED_UNTIL_PUBLISHED'
    assert '--published-release-report - --render' in (
        alternatives[1]['command']
    )
    transition = report['next_action']['public_transition_handoff']
    assert transition == {
        'kind': 'READ_ONLY_PUBLIC_PRODUCT_TRANSITION',
        'status': 'AUDIT_REQUIRED',
        'target_version': '0.9.1',
        'observed_release_status': 'NOT_CHECKED',
        'audits': [
            'product_draft',
            'candidate_environment',
            'published_release',
        ],
        'audit_command': (
            'python3 scripts/check_g0_readiness.py '
            '--include-public-transition '
            '--published-release-version 0.9.1 --json'
        ),
        'post_publication_packet_command': (
            'python3 scripts/check_published_release.py '
            '--version 0.9.1 --json --require-published | '
            'python3 scripts/prepare_onboarding_matrix_packet.py '
            '--published-release-report - --render'
        ),
        'packet_generation_eligible': False,
        'published_identity_required': True,
        'mixed_version_measurements_reusable': False,
        'network_reads_required': True,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }
    assert report['checks']['first_map_cohort']['pending_launch_gates'] == [
        'comparable_docker_row',
        'comparable_source_row',
        'copy_ready_handoff_public',
    ]

    card = DASHBOARD.render_card(report)
    assert card.count('Next action:') == 1
    assert 'GitHub/community writes: **no**' in card
    assert f'| review roles | {expected_routing_status} |' in card
    assert '4 capability lanes / advisory target 2' in card
    assert 'identities: none; reviewer requests: false' in card
    assert '| review ledger | NOT_CHECKED |' in card
    assert '0 pass / 0 blocked / 0 open blockers' in card
    assert '| product Draft PR #427 | NOT_CHECKED |' in card
    assert 'Choices (no write):' in card
    assert 'never reuse mixed-version measurements' in card
    assert 'Public product transition (not executed):' in card
    assert 'Fresh matrix packet eligible: no' in card
    assert 'Mixed-version measurements reusable: no' in card
    assert 'v1 blockers:' in card
    assert 'ndt_omp' in card
    assert 'first-map cohort blockers:' in card
    assert 'copy_ready_handoff_public' in card
    assert 'copy-ready first-map handoff' in card
    assert (
        'record one clean Docker PASS at that version with all seven '
        'measurements' in card
    )
    assert 'all seven measurements' in card
    assert 'g0-current-action-packet-2026-08-14.md' in card

    packet = (
        ROOT / 'docs' / 'evidence' / 'growth'
        / 'g0-current-action-packet-2026-08-14.md'
    ).read_text(encoding='utf-8')
    match = re.search(
        r'Exact reviewed product-candidate tip: `([0-9a-f]{40})`',
        packet,
    )
    assert match is not None
    ancestor_check = subprocess.run(
        ['git', 'merge-base', '--is-ancestor', match.group(1), 'HEAD'],
        cwd=ROOT,
        check=False,
    )
    assert ancestor_check.returncode == 0

    public_baseline_match = re.search(
        r'Capture-time public Draft baseline: `([0-9a-f]{40})`',
        packet,
    )
    assert public_baseline_match is not None
    public_baseline = public_baseline_match.group(1)
    public_baseline_short = f'{public_baseline[:7]}…'
    assert (
        f'capture-time public baseline `{public_baseline_short}`'
        in packet
    )
    assert (
        f'**PASS** for `{public_baseline_short}`: 10 successful checks plus '
        '4 intentionally skipped non-publication jobs, 0 failures'
        in packet
    )
    assert (
        f'source route `READY` at exact public `{public_baseline_short}`'
        in packet
    )
    assert f'--source-commit {public_baseline}' in packet
    baseline_ancestor_check = subprocess.run(
        ['git', 'merge-base', '--is-ancestor', public_baseline, 'HEAD'],
        cwd=ROOT,
        check=False,
    )
    assert baseline_ancestor_check.returncode == 0
    assert 'e222bc490611e6d429f42a1b37778023d55faeb3' not in packet

    scorecard = (
        ROOT / 'docs' / 'growth-scorecard.md'
    ).read_text(encoding='utf-8')
    scorecard_match = re.search(
        r'It binds\nDraft PR #427, the reviewed product-candidate tip\n'
        r'`([0-9a-f]{40})`',
        scorecard,
    )
    assert scorecard_match is not None
    assert scorecard_match.group(1) == match.group(1)
    assert public_baseline in scorecard
    assert (
        '10 successful checks and 4\nintentional non-publication skips'
        in scorecard
    )
    assert 'current 349-path local plan' in scorecard
    assert (
        'complete 396-path / three-phase whole-PR review coverage'
        in scorecard
    )


def test_dashboard_rejects_incomplete_whole_pr_review_coverage():
    """A valid slice inventory cannot hide an uncovered historical gap."""
    reports = DASHBOARD.collect_checker_reports()
    reports['publication_plan']['review_coverage_complete'] = False

    report = DASHBOARD.build_report(reports)

    assert report['status'] == 'HOLD'
    assert report['next_action']['id'] == 'repair-publication-plan'
    assert 'whole-PR review coverage is not valid' in (
        report['next_action']['reason']
    )
    assert report['authority']['github_writes_authorized'] is False


def test_dashboard_rejects_tampered_review_navigation():
    """Reviewer links and table labels must come from one validated lineage."""
    reports = DASHBOARD.collect_checker_reports()
    broken_lineage = copy.deepcopy(reports)
    broken_lineage['publication_overview']['review_phases'][1][
        'start_sha'
    ] = '0' * 40
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='P1-ci-bridge is invalid',
    ):
        DASHBOARD.build_report(broken_lineage)

    unsafe_title = copy.deepcopy(reports)
    unsafe_title['publication_overview']['review_slices'][0]['title'] = (
        'safe | [forged link](https://example.invalid)'
    )
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='S1-runtime-safety is invalid',
    ):
        DASHBOARD.build_report(unsafe_title)

    unsafe_authority = copy.deepcopy(reports)
    unsafe_authority['publication_overview'][
        'github_writes_authorized'
    ] = True
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='unsafe or incomplete',
    ):
        DASHBOARD.build_report(unsafe_authority)


def test_dashboard_rejects_tampered_review_routing():
    """Capability lanes cannot claim duplicate scope or GitHub authority."""
    reports = DASHBOARD.collect_checker_reports()
    duplicate_scope = copy.deepcopy(reports)
    duplicate_scope['product_draft_review_routing']['lanes'][1][
        'slice_ids'
    ] = ['S1-runtime-safety', 'S4-source-onboarding']
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='R2-operator-ux is invalid',
    ):
        DASHBOARD.build_report(duplicate_scope)

    unsafe_authority = copy.deepcopy(reports)
    unsafe_authority['product_draft_review_routing']['authority'][
        'github_reviewer_requests_authorized'
    ] = True
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='contradicts the exact local plan',
    ):
        DASHBOARD.build_report(unsafe_authority)


def test_dashboard_accepts_optional_anonymous_review_ledger_without_authority():
    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft_review_ledger'] = _review_ledger_report(
        reports,
        ['PASS', 'PASS', 'PASS', 'PASS'],
    )

    report = DASHBOARD.build_report(reports)
    ledger = report['checks']['product_draft_review_ledger']

    assert ledger['status'] == 'COMPLETE_LOCAL_REVIEW'
    assert ledger['exact_head'] == report['checks']['publication_plan'][
        'local_tip_sha'
    ]
    assert ledger['passing_lane_count'] == 4
    assert ledger['blocked_lane_count'] == 0
    assert ledger['next_lane_id'] is None
    assert ledger['authority']['identities_collected'] is False
    assert ledger['authority']['github_reviews_authorized'] is False
    assert report['authority']['github_writes_authorized'] is False
    assert 'ledger_path' not in json.dumps(report)

    card = DASHBOARD.render_card(report)
    assert '| review ledger | COMPLETE_LOCAL_REVIEW |' in card
    assert '4 pass / 0 blocked / 0 open blockers' in card
    assert 'GitHub review submitted: false' in card


def test_dashboard_preserves_blocked_review_ledger_and_rejects_tampering():
    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft_review_ledger'] = _review_ledger_report(
        reports,
        ['PASS', 'BLOCKED', 'NOT_REVIEWED', 'NOT_REVIEWED'],
    )

    report = DASHBOARD.build_report(reports)
    ledger = report['checks']['product_draft_review_ledger']
    assert ledger['status'] == 'BLOCKED_LOCAL_REVIEW'
    assert ledger['passing_lane_count'] == 1
    assert ledger['blocked_lane_count'] == 1
    assert ledger['open_blocker_count'] == 1
    assert ledger['next_lane_id'] == 'R2-operator-ux'

    unsafe = copy.deepcopy(reports)
    unsafe['product_draft_review_ledger']['authority'][
        'github_reviews_authorized'
    ] = True
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='stale, unsafe, or incomplete',
    ):
        DASHBOARD.build_report(unsafe)

    stale = copy.deepcopy(reports)
    stale['product_draft_review_ledger']['exact_head'] = '0' * 40
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='stale, unsafe, or incomplete',
    ):
        DASHBOARD.build_report(stale)

    wrong_scope = copy.deepcopy(reports)
    wrong_scope['product_draft_review_ledger']['current_lanes'][1][
        'slice_ids'
    ] = ['S1-runtime-safety']
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='R2-operator-ux is invalid',
    ):
        DASHBOARD.build_report(wrong_scope)


def test_dashboard_cli_accepts_local_review_ledger_without_retaining_path():
    args = DASHBOARD.parse_args([
        '--product-draft-review-ledger',
        '/tmp/lidarslam-pr427-review-ledger.json',
        '--json',
    ])

    assert args.product_draft_review_ledger == DASHBOARD.Path(
        '/tmp/lidarslam-pr427-review-ledger.json'
    )
    assert args.json is True


def test_public_transition_alias_enables_all_three_remote_reads(
    monkeypatch,
    capsys,
):
    """One option expands to the complete dependency-ordered public audit."""
    observed = {}

    def fake_collect(**kwargs):
        observed.update(kwargs)
        return {}

    monkeypatch.setattr(DASHBOARD, 'collect_checker_reports', fake_collect)
    monkeypatch.setattr(
        DASHBOARD,
        'build_report',
        lambda reports, **kwargs: {'status': 'HOLD'},
    )

    assert DASHBOARD.main([
        '--include-public-transition',
        '--published-release-version',
        '0.9.1',
        '--json',
    ]) == 0

    assert observed['include_product_draft'] is True
    assert observed['include_candidate_environment'] is True
    assert observed['include_published_release'] is True
    assert observed['published_release_version'] == '0.9.1'
    assert json.loads(capsys.readouterr().out) == {'status': 'HOLD'}


def test_dashboard_can_include_a_read_only_release_report_without_writes():
    """An optional release report is represented without adding authority."""
    reports = DASHBOARD.collect_checker_reports()
    reports['published_release'] = {
        'status': 'NOT_PUBLISHED',
        'expected_version': '0.9.1',
        'remote': {'tag_present': False},
        'images': [
            {'tag': 'ghcr.io/example:v0.9.1-humble', 'status': 'ABSENT'},
        ],
    }
    report = DASHBOARD.build_report(
        reports,
        published_release_version='0.9.1',
    )

    assert report['authority']['network_reads_performed'] is True
    assert report['checks']['published_release'] == {
        'status': 'NOT_PUBLISHED',
        'version': '0.9.1',
        'tag_present': False,
        'image_statuses': [
            {'tag': 'ghcr.io/example:v0.9.1-humble', 'status': 'ABSENT'},
        ],
    }
    assert report['authority']['remote_mutations_performed'] is False
    transition = report['next_action']['public_transition_handoff']
    assert transition['status'] == 'PUBLICATION_REQUIRED'
    assert transition['observed_release_status'] == 'NOT_PUBLISHED'
    assert transition['packet_generation_eligible'] is False
    assert report['next_action']['alternatives'][1]['status'] == (
        'BLOCKED_UNTIL_PUBLISHED'
    )


def test_published_identity_selects_one_fresh_matrix_packet_command():
    """A public identity opens packet preparation without reusing old rows."""
    reports = DASHBOARD.collect_checker_reports()
    reports['published_release'] = {
        'status': 'PUBLISHED',
        'expected_version': '0.9.1',
        'remote': {'tag_present': True},
        'images': [],
    }

    report = DASHBOARD.build_report(
        reports,
        published_release_version='0.9.1',
    )

    action = report['next_action']
    transition = action['public_transition_handoff']
    assert action['id'] == 'align-public-product-version'
    assert action['title'] == 'Prepare one fresh same-version matrix packet'
    assert action['command'] == transition[
        'post_publication_packet_command'
    ]
    assert transition['status'] == 'READY_FOR_FRESH_MATRIX_PACKET'
    assert transition['packet_generation_eligible'] is True
    assert transition['mixed_version_measurements_reusable'] is False
    assert action['alternatives'][1]['status'] == 'READY_FOR_FRESH_PACKET'


def test_public_transition_rejects_an_unsafe_version_before_rendering():
    """A copy-ready command cannot include shell metacharacters."""
    reports = DASHBOARD.collect_checker_reports()

    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='safe semantic version',
    ):
        DASHBOARD.build_report(
            reports,
            published_release_version='0.9.1;touch-unexpected',
        )


def test_public_transition_rejects_a_different_observed_release_version():
    """A child report cannot silently redirect the matrix target version."""
    reports = DASHBOARD.collect_checker_reports()
    reports['published_release'] = {
        'status': 'NOT_PUBLISHED',
        'expected_version': '0.9.2',
        'remote': {'tag_present': False},
        'images': [],
    }

    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='does not match the requested transition version',
    ):
        DASHBOARD.build_report(
            reports,
            published_release_version='0.9.1',
        )


def test_exact_green_draft_is_reviewed_before_candidate_environment():
    """Exact green Draft evidence takes priority over repository settings."""
    head = '1' * 40
    reports = DASHBOARD.collect_checker_reports()
    _bind_review_tip(reports, head)
    _set_review_worktree_state(
        reports,
        clean=True,
        uncommitted_path_count=0,
    )
    body = _desired_description(reports, head)
    product = _audit_product(head, body=body)

    assert product['status'] == 'DRAFT_REVIEW_REQUIRED'
    assert product['local_head'] == product['remote_head'] == head
    assert product['head_matches_local'] is True
    assert product['mergeable'] is True
    assert product['passing_check_count'] == 10
    assert product['skipped_check_count'] == 4
    assert product['pending_check_count'] == 0
    assert product['failing_check_count'] == 0
    assert product['required_checks_complete'] is True
    assert product['remote_description_sha256'] == hashlib.sha256(
        body.encode('utf-8')
    ).hexdigest()
    assert product['authority']['merge_authorized'] is False

    reports['product_draft'] = product
    reports['candidate_environment'] = {
        'status': 'ABSENT',
        'environment': 'candidate-images',
        'observed': {'target_present': False, 'target': None},
        'findings': [{
            'id': 'candidate-environment-absent',
            'severity': 'BLOCKER',
            'detail': 'Configure and independently review the environment.',
        }],
        'decision': {
            'state': 'HOLD',
            'dispatch_authorized': False,
            'next_action': 'Configure the environment.',
        },
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'environment_writes_authorized': False,
            'artifact_publication_authorized': False,
            'remote_mutations_performed': False,
        },
        'operator_handoff': _absent_environment_handoff(),
    }
    report = DASHBOARD.build_report(reports)

    assert report['next_action']['id'] == 'review-product-draft'
    assert report['next_action']['command'] == (
        'python3 scripts/check_publication_slice_plan.py --overview'
    )
    assert '10 passing checks and 4 intentional skips' in (
        report['next_action']['reason']
    )
    assert 'marking ready and merging remain separate' in (
        report['next_action']['write_boundary']
    )
    handoff = report['next_action']['product_draft_review_handoff']
    assert handoff == {
        'kind': 'EXACT_DRAFT_REVIEW_SEQUENCE',
        'external_write_required': False,
        'pull_request': 427,
        'url': 'https://github.com/rsasaki0109/lidar_slam_ros2/pull/427',
        'exact_head': head,
        'whole_pr_path_count': 396,
        'review_phase_count': 3,
        'slice_count': 7,
        'overview_command': (
            'python3 scripts/check_publication_slice_plan.py --overview'
        ),
        'slice_command_template': (
            'python3 scripts/check_publication_slice_plan.py --slice <ID>'
        ),
        'steps': [
            'Render the exact overview and confirm a clean matching tip.',
            'Review P0, P1, then P2 using their bounded hotspots.',
            (
                'Review S1 through S7 in dependency order and run each '
                'displayed verification group.'
            ),
            (
                'Record findings separately; review submission, mark-ready, '
                'and merge remain separate GitHub decisions.'
            ),
        ],
        'commands_executed': False,
        'github_review_submitted': False,
        'mark_ready_authorized': False,
        'merge_authorized': False,
        'writes_performed': False,
    }
    card = DASHBOARD.render_card(report)
    assert '| product Draft PR #427 | DRAFT_REVIEW_REQUIRED |' in card
    assert 'checks 10 pass / 4 skip / 0 fail' in card
    assert 'merge authorized: false' in card
    assert 'Draft review sequence (not executed):' in card
    assert f'- Exact head: `{head}`' in card
    assert '- Coverage: 396 paths / 3 phases / 7 slices' in card
    assert (
        'Slice template: `python3 scripts/check_publication_slice_plan.py '
        '--slice <ID>`'
    ) in card
    assert '- GitHub review submitted: no' in card
    assert card.count('Next action:') == 1


def test_exact_green_draft_refreshes_stale_description_before_review():
    """A green exact head cannot carry stale reviewer-facing scope facts."""
    head = '9' * 40
    reports = DASHBOARD.collect_checker_reports()
    _bind_review_tip(reports, head)
    _set_review_worktree_state(
        reports,
        clean=True,
        uncommitted_path_count=0,
    )
    reports['product_draft'] = _audit_product(
        head,
        body='stale 321-path / 77-commit description',
    )

    report = DASHBOARD.build_report(reports)
    action = report['next_action']

    assert action['id'] == 'review-product-draft-description-refresh'
    handoff = action['product_draft_description_refresh_handoff']
    assert handoff['desired_head'] == head
    assert handoff['observed_public_head'] == head
    assert handoff['after_branch_update_required'] is False
    assert handoff['body_matches_observed'] is False
    assert handoff['body_sha256'] == hashlib.sha256(
        handoff['body'].encode('utf-8')
    ).hexdigest()
    assert f'Candidate head: `{head}`' in handoff['body']
    assert 'Whole PR review: **' in handoff['body']
    assert '396 paths / 3 phases**' in handoff['body']
    assert '349 paths / 7 slices**' in handoff['body']
    assert '## Exact review map' in handoff['body']
    assert handoff['body'].count('[Open exact diff](') == 3
    assert (
        f'{DASHBOARD.PRODUCT_GITHUB_URL}/compare/' in handoff['body']
    )
    assert '| `S1` | Runtime point-cloud and VoxelGrid safety | 16 | 3 |' in (
        handoff['body']
    )
    assert '| `S7` | Exact publication inventory and authority boundary |' in (
        handoff['body']
    )
    assert '## Review roles' in handoff['body']
    assert handoff['body'].count('| `R') == 4
    assert '| `R1` | S1, S2 | 50 | 5 |' in handoff['body']
    assert '| `R4` | S6, S7 | 162 | 16 |' in handoff['body']
    assert (
        'Advisory reviewer target: **2** (target only; not a merge gate). '
        'Identities collected: none.'
    ) in handoff['body']
    assert '@' not in handoff['body']
    assert 'username' not in handoff['body'].lower()
    assert '0/4 comparable' in handoff['body']
    assert handoff['description_update_authorized'] is False
    assert handoff['review_submission_authorized'] is False
    assert handoff['mark_ready_authorized'] is False
    assert handoff['merge_authorized'] is False
    assert handoff['writes_performed'] is False
    assert 'gh pr edit' not in repr(action)

    card = DASHBOARD.render_card(report)
    assert 'Draft description refresh handoff (not executed):' in card
    assert 'Exact desired PR description:' in card
    assert handoff['body_sha256'] in card
    assert '- Description updates performed: no' in card


def test_exact_green_draft_refuses_review_handoff_from_dirty_worktree():
    """Uncommitted bytes cannot be mislabeled as the exact public review."""
    head = '2' * 40
    reports = DASHBOARD.collect_checker_reports()
    _set_review_worktree_state(
        reports,
        clean=False,
        uncommitted_path_count=2,
    )
    reports['product_draft'] = _audit_product(head)

    report = DASHBOARD.build_report(reports)

    assert report['next_action'] == {
        'id': 'restore-clean-draft-review-worktree',
        'title': 'Restore a clean worktree before exact Draft review',
        'reason': (
            'The local publication plan has 2 uncommitted paths, so its '
            'rendered review budget would not describe exact public head '
            f'{head}.'
        ),
        'command': 'git status --short',
        'write_boundary': (
            'read-only local inspection; no file cleanup, commit, push, '
            'review submission, mark-ready, or merge is authorized'
        ),
    }
    assert report['authority']['github_writes_authorized'] is False


def test_environment_audit_without_pr_audit_requests_the_missing_dependency():
    """A settings read cannot skip the earlier product merge dependency."""
    reports = DASHBOARD.collect_checker_reports()
    reports['candidate_environment'] = {
        'status': 'ABSENT',
        'environment': 'candidate-images',
        'observed': {'target_present': False, 'target': None},
        'findings': [{
            'id': 'candidate-environment-absent',
            'severity': 'BLOCKER',
            'detail': 'Configure and independently review the environment.',
        }],
        'decision': {
            'state': 'HOLD',
            'dispatch_authorized': False,
            'next_action': 'Configure the environment.',
        },
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'environment_writes_authorized': False,
            'artifact_publication_authorized': False,
            'remote_mutations_performed': False,
        },
        'operator_handoff': _absent_environment_handoff(),
    }

    report = DASHBOARD.build_report(reports)

    assert report['next_action']['id'] == 'inspect-product-draft'
    assert '--include-product-draft' in report['next_action']['command']
    assert 'settings changes' in report['next_action']['write_boundary']


def test_product_audit_fails_closed_on_drift_failed_ci_and_authority():
    """Head drift, failed required CI, and claimed merge authority all stop."""
    head = '2' * 40

    def drift_fetcher(path: str):
        assert path.endswith('/pulls/427')
        return 200, _product_pull('3' * 40)

    drift = DASHBOARD.audit_product_draft(
        fetcher=drift_fetcher,
        local_head=head,
        ancestor_checker=lambda _ancestor, _descendant: False,
    )
    assert drift['status'] == 'BLOCKED'
    assert drift['head_matches_local'] is False
    assert drift['non_force_update_possible'] is False
    assert drift['required_checks_complete'] is False
    assert drift['blockers'] == [
        'Local and public product PR heads do not match.'
    ]

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = drift
    divergence = DASHBOARD.build_report(reports)['next_action']
    assert divergence['id'] == 'inspect-product-draft-divergence'
    assert divergence['command'] == f"git merge-base {'3' * 40} {head}"
    assert divergence['command'] != DASHBOARD.PRODUCT_PR_VERIFY_COMMAND

    failed = _audit_product(
        head,
        failed_name='docs and release metadata',
    )
    assert failed['status'] == 'BLOCKED'
    assert failed['failing_check_count'] == 1
    assert 'required successful checks are not successful' in (
        failed['blockers'][0]
    )

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = _audit_product(head)
    reports['product_draft']['authority']['merge_authorized'] = True
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='unsafe or incomplete authority',
    ):
        DASHBOARD.build_report(reports)

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = _audit_product(head)
    reports['product_draft']['non_force_update_possible'] = False
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='non-force update claim contradicts its state',
    ):
        DASHBOARD.build_report(reports)


def test_product_audit_bounds_and_validates_description_digest():
    """Remote free text is hashed only when it satisfies the size contract."""
    head = 'a' * 40

    def malformed_fetcher(path: str):
        assert path.endswith('/pulls/427')
        pull = _product_pull(head)
        pull['body'] = {'unexpected': 'object'}
        return 200, pull

    malformed = DASHBOARD.audit_product_draft(
        fetcher=malformed_fetcher,
        local_head=head,
    )
    assert malformed['status'] == 'BLOCKED'
    assert malformed['blockers'] == [
        'Product PR description is not text or null.'
    ]
    assert malformed['remote_description_sha256'] is None

    def oversized_fetcher(path: str):
        assert path.endswith('/pulls/427')
        return 200, _product_pull(
            head,
            body='x' * (DASHBOARD.MAX_PRODUCT_DESCRIPTION_BYTES + 1),
        )

    oversized = DASHBOARD.audit_product_draft(
        fetcher=oversized_fetcher,
        local_head=head,
    )
    assert oversized['status'] == 'BLOCKED'
    assert oversized['blockers'] == [
        'Product PR description exceeds the audit size limit.'
    ]

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = _audit_product(head)
    reports['product_draft']['remote_description_sha256'] = 'bad-digest'
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='description digest is invalid',
    ):
        DASHBOARD.build_report(reports)


def test_head_drift_emits_exact_non_force_handoff_without_push_command():
    """Fast-forward drift gets a bounded handoff, not a circular re-audit."""
    local_head = '6' * 40
    public_head = '5' * 40

    def drift_fetcher(path: str):
        assert path.endswith('/pulls/427')
        return 200, _product_pull(public_head)

    product = DASHBOARD.audit_product_draft(
        fetcher=drift_fetcher,
        local_head=local_head,
        ancestor_checker=lambda ancestor, descendant: (
            ancestor == public_head and descendant == local_head
        ),
    )
    assert product['non_force_update_possible'] is True

    reports = DASHBOARD.collect_checker_reports()
    _bind_review_tip(reports, local_head)
    _set_review_worktree_state(
        reports,
        clean=True,
        uncommitted_path_count=0,
    )
    reports['product_draft'] = product
    report = DASHBOARD.build_report(reports)
    action = report['next_action']

    assert action['id'] == 'review-product-draft-branch-update'
    assert action['command'] == (
        f'git merge-base --is-ancestor {public_head} {local_head}'
    )
    assert action['command'] != DASHBOARD.PRODUCT_PR_VERIFY_COMMAND
    handoff = action['product_draft_update_handoff']
    assert handoff['repository_url'] == DASHBOARD.PRODUCT_REPOSITORY_URL
    assert handoff['public_head'] == public_head
    assert handoff['local_head'] == local_head
    assert handoff['fast_forward_verified'] is True
    assert handoff['non_force_only'] is True
    assert handoff['push_authorized'] is False
    assert handoff['force_push_authorized'] is False
    assert handoff['writes_performed'] is False
    assert handoff['verification_command'] == DASHBOARD.PRODUCT_PR_VERIFY_COMMAND
    description = action['product_draft_description_refresh_handoff']
    assert description['observed_public_head'] == public_head
    assert description['desired_head'] == local_head
    assert description['after_branch_update_required'] is True
    assert description['clean_tip_verified'] is True
    assert description['body_matches_observed'] is False
    assert description['body_sha256'] == hashlib.sha256(
        description['body'].encode('utf-8')
    ).hexdigest()
    assert f'Candidate head: `{local_head}`' in description['body']
    assert description['description_update_authorized'] is False
    assert description['mark_ready_authorized'] is False
    assert description['merge_authorized'] is False
    assert description['writes_performed'] is False
    assert 'git push' not in repr(action)

    card = DASHBOARD.render_card(report)
    assert 'Draft branch update handoff (not executed):' in card
    assert f'Public head: `{public_head}`' in card
    assert f'Local tip: `{local_head}`' in card
    assert f'Repository: `{DASHBOARD.PRODUCT_REPOSITORY_URL}`' in card
    assert 'Fast-forward verified: yes' in card
    assert 'Pushes performed: no' in card
    assert 'Draft description refresh handoff (not executed):' in card
    assert f'Desired head: `{local_head}`' in card
    assert 'Branch update required first: yes' in card
    assert '- Description updates performed: no' in card
    assert 'git push' not in card

    dirty_reports = copy.deepcopy(reports)
    _set_review_worktree_state(
        dirty_reports,
        clean=False,
        uncommitted_path_count=1,
    )
    dirty_action = DASHBOARD.build_report(dirty_reports)['next_action']
    assert dirty_action['id'] == 'restore-clean-draft-update-worktree'
    assert 'product_draft_update_handoff' not in dirty_action
    assert 'product_draft_description_refresh_handoff' not in dirty_action
    assert 'no cleanup, commit, push, PR edit' in (
        dirty_action['write_boundary']
    )


def test_missing_lineage_fetches_the_canonical_repository_not_origin():
    """Lineage recovery cannot trust a checkout-specific origin remote."""
    local_head = '8' * 40
    public_head = '7' * 40

    def drift_fetcher(path: str):
        assert path.endswith('/pulls/427')
        return 200, _product_pull(public_head)

    product = DASHBOARD.audit_product_draft(
        fetcher=drift_fetcher,
        local_head=local_head,
        ancestor_checker=lambda _ancestor, _descendant: None,
    )
    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = product
    action = DASHBOARD.build_report(reports)['next_action']

    assert action['id'] == 'restore-product-draft-lineage-evidence'
    assert DASHBOARD.PRODUCT_REPOSITORY_URL in action['command']
    assert ' origin ' not in action['command']
    assert action['command'].endswith(
        f'git merge-base --is-ancestor {public_head} {local_head}'
    )
    assert 'no remote write' in action['write_boundary']


def test_product_review_state_only_clears_after_exact_merge():
    """Ready-for-review stays ahead of settings; a merged PR may proceed."""
    head = '4' * 40
    ready = _audit_product(head, draft=False)
    assert ready['status'] == 'READY_FOR_SEPARATE_MERGE_REVIEW'
    assert ready['decision_state'] == 'READY_FOR_SEPARATE_MERGE_REVIEW'
    assert ready['authority']['merge_authorized'] is False

    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = ready
    report = DASHBOARD.build_report(reports)
    assert report['next_action']['id'] == 'review-product-merge'

    merged = _audit_product(
        head,
        draft=False,
        state='closed',
        merged=True,
        mergeable=None,
    )
    assert merged['status'] == 'MERGED'
    assert merged['decision_state'] == 'MERGED'


def test_dashboard_surfaces_absent_environment_before_candidate_alignment():
    """A requested E2 preflight becomes the one bounded next action."""
    reports = DASHBOARD.collect_checker_reports()
    reports['product_draft'] = _audit_product(
        '5' * 40,
        draft=False,
        state='closed',
        merged=True,
        mergeable=None,
    )
    reports['candidate_environment'] = {
        'status': 'ABSENT',
        'environment': 'candidate-images',
        'observed': {
            'target_present': False,
            'target': None,
        },
        'findings': [{
            'id': 'candidate-environment-absent',
            'severity': 'BLOCKER',
            'detail': 'Configure and independently review the environment.',
        }],
        'decision': {
            'state': 'HOLD',
            'dispatch_authorized': False,
            'next_action': 'Configure the environment.',
        },
        'authority': {
            'network_reads_performed': True,
            'github_writes_authorized': False,
            'environment_writes_authorized': False,
            'artifact_publication_authorized': False,
            'remote_mutations_performed': False,
        },
        'operator_handoff': _absent_environment_handoff(),
    }

    report = DASHBOARD.build_report(reports)

    assert report['authority']['network_reads_performed'] is True
    assert report['checks']['candidate_environment']['status'] == 'ABSENT'
    assert report['next_action']['id'] == 'review-candidate-environment'
    assert '--require-ready' in report['next_action']['command']
    handoff = report['checks']['candidate_environment']['operator_handoff']
    assert handoff == _absent_environment_handoff()
    assert report['next_action']['operator_handoff'] == handoff
    assert 'E2 dispatch remain separate' in (
        report['next_action']['write_boundary']
    )
    card = DASHBOARD.render_card(report)
    assert '| candidate environment | ABSENT |' in card
    assert 'dispatch authorized: false' in card
    assert 'Operator handoff (not executed):' in card
    assert 'Authority required: repository-settings-admin' in card
    assert '/settings/environments' in card
    assert '1. Create an environment named candidate-images.' in card
    assert 'Environment writes performed: no' in card

    unsafe = copy.deepcopy(reports)
    unsafe['candidate_environment']['decision'][
        'dispatch_authorized'
    ] = True
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='remote-write authority',
    ):
        DASHBOARD.build_report(unsafe)

    unsafe = copy.deepcopy(reports)
    unsafe['candidate_environment']['operator_handoff'][
        'writes_performed'
    ] = True
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='operator handoff is unsafe',
    ):
        DASHBOARD.build_report(unsafe)

    unsafe = copy.deepcopy(reports)
    unsafe['candidate_environment']['operator_handoff']['settings_url'] = (
        'https://example.com/settings'
    )
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='untrusted URL',
    ):
        DASHBOARD.build_report(unsafe)

    unsafe = copy.deepcopy(reports)
    unsafe['candidate_environment']['operator_handoff']['kind'] = (
        'REPAIR_AND_REVIEW_ENVIRONMENT'
    )
    with pytest.raises(
        DASHBOARD.G0ReadinessError,
        match='contradicts its status',
    ):
        DASHBOARD.build_report(unsafe)


def test_dashboard_selects_one_next_action_and_ready_state_is_explicit():
    """A fully ready synthetic report gets one explicit review action."""
    reports = DASHBOARD.collect_checker_reports()
    ready = copy.deepcopy(reports)
    ready['onboarding_matrix']['decision']['status'] = 'ACTIVATION_GATE_PASS'
    ready['onboarding_matrix']['summary'].update({
        'activation_gate': True,
        'comparable_rows': 2,
        'docker_comparable_rows': 1,
        'source_comparable_rows': 1,
        'product_version_aligned': True,
    })
    ready['first_map_cohort'].update({
        'status': 'READY_FOR_NEXT_ATTEMPT',
        'launch_status': 'READY_FOR_NEXT_ATTEMPT',
        'pending_launch_gates': [],
    })
    ready['v1_readiness'].update({
        'status': 'READY',
        'summary': {'complete': 10, 'incomplete': 0, 'total': 10},
        'gates': [],
    })
    ready['published_release'] = {
        'status': 'PUBLISHED',
        'expected_version': '0.9.1',
        'remote': {'tag_present': True},
        'images': [],
    }

    report = DASHBOARD.build_report(
        ready,
        published_release_version='0.9.1',
    )

    assert report['status'] == 'READY_FOR_REVIEW'
    assert report['next_action']['id'] == 'review-external-gates'
    assert report['next_action']['write_boundary'].startswith('read-only')


def test_checker_error_is_not_downgraded_to_a_hold():
    """A checker execution error remains an error, never a synthetic HOLD."""
    def failing_runner(*_args, **_kwargs):
        return DASHBOARD.subprocess.CompletedProcess(
            args=[],
            returncode=2,
            stdout='',
            stderr='synthetic checker failure',
        )

    with pytest.raises(DASHBOARD.G0ReadinessError, match='synthetic'):
        DASHBOARD.collect_checker_reports(runner=failing_runner)


def test_product_pr_network_client_is_authenticated_get_only(monkeypatch):
    """The product PR client scopes auth to a bodyless GitHub GET."""
    captured = {}

    class Response:
        status = 200

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self, _limit):
            return b'{"number": 427}'

    def urlopen(request, *, timeout):
        captured['request'] = request
        captured['timeout'] = timeout
        return Response()

    monkeypatch.setenv('GITHUB_TOKEN', 'read-only-test-token')
    monkeypatch.setattr(DASHBOARD.urllib.request, 'urlopen', urlopen)

    status, payload = DASHBOARD._github_json(
        'repos/rsasaki0109/lidar_slam_ros2/pulls/427'
    )

    request = captured['request']
    assert status == 200
    assert payload == {'number': 427}
    assert request.get_method() == 'GET'
    assert request.data is None
    assert request.get_header('Authorization') == (
        'Bearer read-only-test-token'
    )
    assert captured['timeout'] == 30
