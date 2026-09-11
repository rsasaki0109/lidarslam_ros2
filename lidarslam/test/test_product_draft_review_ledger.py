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

"""Tests for the anonymous exact-tip product Draft review ledger."""

from __future__ import annotations

import copy
import importlib.util
import json
import pathlib
import sys

import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'product_draft_review_ledger.py'
SPEC = importlib.util.spec_from_file_location(
    'product_draft_review_ledger',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
LEDGER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LEDGER)


def _context() -> dict:
    return {
        'exact_head': '1' * 40,
        'worktree_clean': True,
        'routing_contract_sha256': '2' * 64,
        'lanes': [
            {
                'id': 'R1-runtime-safety',
                'order': 1,
                'slice_ids': (
                    'S1-runtime-safety',
                    'S2-first-map-foundation',
                ),
                'depends_on_lanes': (),
            },
            {
                'id': 'R2-operator-ux',
                'order': 2,
                'slice_ids': (
                    'S3-map-lifecycle',
                    'S4-source-onboarding',
                ),
                'depends_on_lanes': ('R1-runtime-safety',),
            },
            {
                'id': 'R3-distribution',
                'order': 3,
                'slice_ids': ('S5-distribution-readiness',),
                'depends_on_lanes': (
                    'R1-runtime-safety',
                    'R2-operator-ux',
                ),
            },
            {
                'id': 'R4-integration-publication',
                'order': 4,
                'slice_ids': (
                    'S6-product-shell-integration',
                    'S7-publication-control',
                ),
                'depends_on_lanes': (
                    'R1-runtime-safety',
                    'R2-operator-ux',
                    'R3-distribution',
                ),
            },
        ],
        'paths_by_slice': {
            'S1-runtime-safety': ('graph_based_slam/src/runtime.cpp',),
            'S2-first-map-foundation': ('docs/getting-started.md',),
            'S3-map-lifecycle': ('scripts/session.py',),
            'S4-source-onboarding': ('scripts/source_quickstart.sh',),
            'S5-distribution-readiness': ('package.xml',),
            'S6-product-shell-integration': ('scripts/product_cli.py',),
            'S7-publication-control': ('docs/releases/v0.9.1.md',),
        },
    }


def _pass(
    ledger: dict,
    lane_id: str,
    *,
    findings: list[list[str]] | None = None,
) -> dict:
    return LEDGER.append_event(
        ledger,
        _context(),
        lane_id=lane_id,
        outcome='PASS',
        verification_status='PASS',
        raw_findings=findings or [],
    )


def test_empty_ledger_is_exact_anonymous_and_not_review_evidence():
    ledger = LEDGER.prepare_ledger(_context())
    report = LEDGER.build_report(ledger, _context())

    assert report['status'] == 'EMPTY_LOCAL_LEDGER'
    assert report['exact_head'] == '1' * 40
    assert report['event_count'] == 0
    assert report['reviewed_lane_count'] == 0
    assert report['passing_lane_count'] == 0
    assert report['blocked_lane_count'] == 0
    assert report['next_lane_id'] == 'R1-runtime-safety'
    assert report['authority'] == LEDGER.AUTHORITY
    assert all(
        lane['status'] == 'NOT_REVIEWED'
        for lane in report['current_lanes']
    )
    serialized = json.dumps(ledger)
    assert '@' not in serialized
    assert 'username' not in serialized
    assert 'email' not in serialized

    card = LEDGER.render_card(report, ledger)
    assert 'Reviewer identities collected: none' in card
    assert 'GitHub review submitted: no' in card
    assert '--lane R1-runtime-safety' in card


def test_append_only_events_retain_blocker_history_until_all_lanes_pass():
    ledger = LEDGER.prepare_ledger(_context())
    ledger = _pass(ledger, 'R1-runtime-safety')
    ledger = LEDGER.append_event(
        ledger,
        _context(),
        lane_id='R2-operator-ux',
        outcome='BLOCKED',
        verification_status='FAIL',
        raw_findings=[[
            'BLOCKER',
            'operator-ux-gap',
            'S3-map-lifecycle',
            'scripts/session.py',
            'Retained session choice does not expose the safe recovery step.',
        ]],
    )
    blocked = LEDGER.build_report(ledger, _context())

    assert blocked['status'] == 'BLOCKED_LOCAL_REVIEW'
    assert blocked['passing_lane_count'] == 1
    assert blocked['blocked_lane_count'] == 1
    assert blocked['open_blocker_count'] == 1
    assert blocked['next_lane_id'] == 'R2-operator-ux'
    assert ledger['events'][-1]['findings'][0]['id'] == 'E002-F01'

    ledger = _pass(ledger, 'R2-operator-ux')
    recovered = LEDGER.build_report(ledger, _context())
    assert recovered['status'] == 'IN_PROGRESS_LOCAL_REVIEW'
    assert recovered['historical_finding_count'] == 1
    assert recovered['current_finding_count'] == 0
    assert recovered['open_blocker_count'] == 0
    assert recovered['next_lane_id'] == 'R3-distribution'

    ledger = _pass(ledger, 'R3-distribution')
    ledger = _pass(
        ledger,
        'R4-integration-publication',
        findings=[[
            'NOTE',
            'maintainability-note',
            'S6-product-shell-integration',
            'scripts/product_cli.py',
            'Keep the lane card and dashboard wording synchronized.',
        ]],
    )
    complete = LEDGER.build_report(ledger, _context())

    assert complete['status'] == 'COMPLETE_LOCAL_REVIEW'
    assert complete['passing_lane_count'] == 4
    assert complete['blocked_lane_count'] == 0
    assert complete['next_lane_id'] is None
    assert complete['event_count'] == 5
    assert complete['historical_finding_count'] == 2
    assert complete['current_finding_count'] == 1
    assert complete['open_blocker_count'] == 0


def test_lane_dependencies_and_downstream_freshness_fail_closed():
    ledger = LEDGER.prepare_ledger(_context())
    with pytest.raises(LEDGER.ReviewLedgerError, match='unmet lane dependencies'):
        _pass(ledger, 'R2-operator-ux')

    ledger = _pass(ledger, 'R1-runtime-safety')
    ledger = _pass(ledger, 'R2-operator-ux')
    with pytest.raises(LEDGER.ReviewLedgerError, match='stale downstream lanes'):
        _pass(ledger, 'R1-runtime-safety')


@pytest.mark.parametrize(
    ('finding', 'message'),
    [
        (
            [
                'BLOCKER',
                'correctness-risk',
                'S1-runtime-safety',
                'scripts/session.py',
                'The result is out of lane scope.',
            ],
            'unsafe or out of scope',
        ),
        (
            [
                'BLOCKER',
                'correctness-risk',
                'S1-runtime-safety',
                'graph_based_slam/src/runtime.cpp',
                'Ask @reviewer or reviewer@example.com for local /home/name data.',
            ],
            'unsafe or out of scope',
        ),
    ],
)
def test_findings_reject_out_of_scope_paths_and_identity_bearing_detail(
    finding: list[str],
    message: str,
):
    ledger = LEDGER.prepare_ledger(_context())
    with pytest.raises(LEDGER.ReviewLedgerError, match=message):
        LEDGER.append_event(
            ledger,
            _context(),
            lane_id='R1-runtime-safety',
            outcome='BLOCKED',
            verification_status='FAIL',
            raw_findings=[finding],
        )


def test_pass_and_blocked_outcomes_require_truthful_findings():
    ledger = LEDGER.prepare_ledger(_context())
    with pytest.raises(LEDGER.ReviewLedgerError, match='review ledger failed'):
        LEDGER.append_event(
            ledger,
            _context(),
            lane_id='R1-runtime-safety',
            outcome='PASS',
            verification_status='FAIL',
            raw_findings=[],
        )
    with pytest.raises(LEDGER.ReviewLedgerError, match='review ledger failed'):
        LEDGER.append_event(
            ledger,
            _context(),
            lane_id='R1-runtime-safety',
            outcome='BLOCKED',
            verification_status='FAIL',
            raw_findings=[[
                'NOTE',
                'maintainability-note',
                'S1-runtime-safety',
                'graph_based_slam/src/runtime.cpp',
                'This note alone cannot block the lane.',
            ]],
        )


def test_canonical_atomic_cli_writes_stay_outside_the_repository(
    tmp_path: pathlib.Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    monkeypatch.setattr(LEDGER, 'collect_review_context', _context)
    path = tmp_path / 'review-ledger.json'

    assert LEDGER.main([
        'prepare', '--output', str(path), '--json',
    ]) == 0
    prepared = json.loads(capsys.readouterr().out)
    assert prepared['status'] == 'EMPTY_LOCAL_LEDGER'
    assert path.read_bytes() == LEDGER._canonical_payload(
        json.loads(path.read_text(encoding='utf-8'))
    )

    assert LEDGER.main([
        'record',
        '--ledger', str(path),
        '--lane', 'R1-runtime-safety',
        '--outcome', 'PASS',
        '--verification-status', 'PASS',
        '--json',
    ]) == 0
    recorded = json.loads(capsys.readouterr().out)
    assert recorded['status'] == 'IN_PROGRESS_LOCAL_REVIEW'
    assert recorded['event_count'] == 1
    assert not list(tmp_path.glob('.*.tmp'))

    with pytest.raises(
        LEDGER.ReviewLedgerError,
        match='outside the source repository',
    ):
        LEDGER._write_ledger(
            ROOT / 'forbidden-review-ledger.json',
            LEDGER.prepare_ledger(_context()),
            replace=False,
        )


def test_tampered_authority_and_noncanonical_ledger_are_rejected(
    tmp_path: pathlib.Path,
):
    ledger = LEDGER.prepare_ledger(_context())
    unsafe = copy.deepcopy(ledger)
    unsafe['authority']['github_reviews_authorized'] = True
    with pytest.raises(LEDGER.ReviewLedgerError, match='review ledger failed'):
        LEDGER.build_report(unsafe, _context())

    path = tmp_path / 'ledger.json'
    path.write_text(json.dumps(ledger), encoding='utf-8')
    with pytest.raises(LEDGER.ReviewLedgerError, match='canonical recorder'):
        LEDGER.load_ledger(path)


def test_release_bundle_keeps_review_ledger_schemas_and_entrypoint():
    bundle_script = ROOT / 'scripts' / 'build_release_bundle.py'
    spec = importlib.util.spec_from_file_location(
        'build_release_bundle_for_review_ledger',
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
        'docs/schemas/product-draft-review-ledger-v1.schema.json',
        'docs/schemas/product-draft-review-ledger-report-v1.schema.json',
        'scripts/product_draft_review_ledger.py',
    } <= paths
