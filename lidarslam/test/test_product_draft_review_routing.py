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

"""Tests for privacy-safe product Draft review routing."""

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_product_draft_review_routing.py'
SPEC = importlib.util.spec_from_file_location(
    'check_product_draft_review_routing',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
ROUTING = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ROUTING)


def _contract() -> dict:
    return json.loads(ROUTING.CONTRACT_PATH.read_text(encoding='utf-8'))


def _overview() -> dict:
    slices = [
        ('S1-runtime-safety', 16, 3, []),
        ('S2-first-map-foundation', 34, 2, ['S1-runtime-safety']),
        ('S3-map-lifecycle', 25, 2, ['S2-first-map-foundation']),
        ('S4-source-onboarding', 38, 2, ['S2-first-map-foundation']),
        (
            'S5-distribution-readiness',
            74,
            9,
            ['S1-runtime-safety', 'S4-source-onboarding'],
        ),
        (
            'S6-product-shell-integration',
            155,
            14,
            [
                'S1-runtime-safety',
                'S2-first-map-foundation',
                'S3-map-lifecycle',
                'S4-source-onboarding',
                'S5-distribution-readiness',
            ],
        ),
        (
            'S7-publication-control',
            7,
            2,
            [
                'S1-runtime-safety',
                'S2-first-map-foundation',
                'S3-map-lifecycle',
                'S4-source-onboarding',
                'S5-distribution-readiness',
                'S6-product-shell-integration',
            ],
        ),
    ]
    return {
        'status': 'PR_REVIEW_OVERVIEW_READY_LOCAL_ONLY',
        'candidate': {
            'local_tip_sha': '1' * 40,
            'follow_up_path_count': 349,
            'slice_count': 7,
            'review_coverage_complete': True,
            'merge_commit_count': 0,
            'worktree_clean': True,
            'uncommitted_path_count': 0,
        },
        'review_slices': [
            {
                'id': slice_id,
                'order': order,
                'path_count': path_count,
                'verification_count': verification_count,
                'depends_on': dependencies,
            }
            for order, (
                slice_id,
                path_count,
                verification_count,
                dependencies,
            ) in enumerate(slices, start=1)
        ],
        'commands_executed': False,
        'github_writes_authorized': False,
        'remote_mutations_performed': False,
    }


def test_routing_assigns_every_slice_once_without_collecting_identity():
    report = ROUTING.build_report(_contract(), _overview())

    assert report['status'] == 'READY_LOCAL_ONLY'
    assert report['exact_head'] == '1' * 40
    assert report['worktree_clean'] is True
    assert report['uncommitted_path_count'] == 0
    assert report['advisory_reviewer_target'] == 2
    assert report['advisory_target_is_merge_gate'] is False
    assert report['summary'] == {
        'lane_count': 4,
        'slice_count': 7,
        'path_count': 349,
        'verification_count': 34,
        'unassigned_slice_count': 0,
        'duplicate_slice_count': 0,
    }
    assert [lane['id'] for lane in report['lanes']] == list(
        ROUTING.EXPECTED_LANE_IDS
    )
    assert [lane['path_count'] for lane in report['lanes']] == [
        50,
        63,
        74,
        162,
    ]
    assert report['lanes'][-1]['depends_on_lanes'] == [
        'R1-runtime-safety',
        'R2-operator-ux',
        'R3-distribution',
    ]
    assert report['authority'] == {
        'commands_executed': False,
        'github_reviewer_requests_authorized': False,
        'github_reviews_authorized': False,
        'mark_ready_authorized': False,
        'merge_authorized': False,
        'remote_mutations_performed': False,
    }
    serialized = json.dumps(report)
    assert '@' not in serialized
    assert 'username' not in serialized
    assert 'email' not in serialized

    card = ROUTING.render_card(report)
    assert 'Advisory reviewer target: 2 (target only; not a merge gate)' in card
    assert 'Reviewer identities collected: none' in card
    assert '| `R1-runtime-safety` | S1-runtime-safety,' in card
    assert 'GitHub reviewer requests authorized: no' in card
    assert 'no identity collection, reviewer request' in card


def test_lane_card_is_bounded_to_one_capability():
    report = ROUTING.build_report(_contract(), _overview())

    card = ROUTING.render_card(report, lane_id='R3-distribution')

    assert '## R3-distribution — Distribution and dependency readiness' in card
    assert '--slice S5-distribution-readiness' in card
    assert '--slice S1-runtime-safety' not in card
    assert 'ROS distribution, dependency ownership, and release evidence' in (
        card
    )


def test_routing_rejects_missing_duplicate_and_stale_slice_budgets():
    missing = _contract()
    missing['lanes'][-1]['slice_ids'].remove('S7-publication-control')
    with pytest.raises(ROUTING.ReviewRoutingError, match='missing='):
        ROUTING.build_report(missing, _overview())

    duplicate = _contract()
    duplicate['lanes'][0]['slice_ids'][1] = 'S3-map-lifecycle'
    with pytest.raises(ROUTING.ReviewRoutingError, match='duplicates='):
        ROUTING.build_report(duplicate, _overview())

    stale = _contract()
    stale['lanes'][1]['expected_path_count'] += 1
    with pytest.raises(ROUTING.ReviewRoutingError, match='budget is stale'):
        ROUTING.build_report(stale, _overview())


def test_routing_rejects_dependency_drift_and_bounds_dirty_overview():
    wrong_dependency = _contract()
    wrong_dependency['lanes'][2]['depends_on_lanes'] = [
        'R1-runtime-safety'
    ]
    with pytest.raises(
        ROUTING.ReviewRoutingError,
        match='dependencies are not exact',
    ):
        ROUTING.build_report(wrong_dependency, _overview())

    dirty = _overview()
    dirty['candidate']['worktree_clean'] = False
    dirty['candidate']['uncommitted_path_count'] = 2
    dirty_report = ROUTING.build_report(_contract(), dirty)
    assert dirty_report['status'] == 'PREPARED_DIRTY_WORKTREE'
    assert dirty_report['worktree_clean'] is False
    assert dirty_report['uncommitted_path_count'] == 2
    dirty_card = ROUTING.render_card(dirty_report)
    assert 'Worktree clean: no' in dirty_card
    assert 'do not use this dirty routing packet' in dirty_card

    unsafe = _overview()
    unsafe['github_writes_authorized'] = True
    with pytest.raises(
        ROUTING.ReviewRoutingError,
        match='one bounded, no-write exact overview',
    ):
        ROUTING.build_report(_contract(), unsafe)


def test_contract_schema_refuses_reviewer_request_authority():
    unsafe = _contract()
    unsafe['authority']['github_reviewer_requests_authorized'] = True

    with pytest.raises(
        ROUTING.ReviewRoutingError,
        match='review routing contract failed',
    ):
        ROUTING.build_report(unsafe, _overview())


def test_release_bundle_keeps_review_routing_contract_and_entrypoints():
    bundle_script = ROOT / 'scripts' / 'build_release_bundle.py'
    spec = importlib.util.spec_from_file_location(
        'build_release_bundle_for_review_routing',
        bundle_script,
    )
    assert spec is not None and spec.loader is not None
    bundle = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(ROOT / 'scripts'))
    try:
        spec.loader.exec_module(bundle)
    finally:
        sys.path.remove(str(ROOT / 'scripts'))

    paths = set(bundle.release_bundle_paths(ROOT, 'v0.9.1'))

    assert {
        'docs/contracts/product-draft-review-routing-v1.json',
        'docs/review-routing.md',
        'docs/schemas/product-draft-review-routing-report-v1.schema.json',
        'docs/schemas/product-draft-review-routing-v1.schema.json',
        'scripts/check_product_draft_review_routing.py',
    } <= paths
