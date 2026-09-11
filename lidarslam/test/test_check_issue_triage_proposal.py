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

"""Tests for the fail-closed, read-only issue-triage proposal audit."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'check_issue_triage_proposal.py'
PROPOSAL_PATH = (
    ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'open-issue-triage-proposal-2026-08-11.json'
)
SCHEMA_PATH = (
    ROOT / 'docs' / 'schemas' / 'issue-triage-proposal-v1.schema.json'
)
SPEC = importlib.util.spec_from_file_location(
    'check_issue_triage_proposal',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def _load(path: Path) -> dict:
    """Load one tracked JSON object."""
    payload = json.loads(path.read_text(encoding='utf-8'))
    assert isinstance(payload, dict)
    return payload


def _proposal() -> dict:
    """Return an isolated copy of the tracked proposal."""
    return copy.deepcopy(_load(PROPOSAL_PATH))


def _schema() -> dict:
    """Return an isolated copy of the tracked schema."""
    return copy.deepcopy(_load(SCHEMA_PATH))


def _live_snapshot(proposal: dict) -> tuple[list[dict], list[str]]:
    """Build an exact live-shaped snapshot from observed proposal fields."""
    issues = []
    for item in proposal['issues']:
        issues.append({
            'issue_number': item['issue_number'],
            'title': item['title'],
            'updated_at': item['observed']['updated_at'],
            'labels': list(item['observed']['labels']),
        })
    return issues, list(proposal['label_catalog'])


def _walk_keys(value):
    """Yield every mapping key recursively."""
    if isinstance(value, dict):
        for key, item in value.items():
            yield key
            yield from _walk_keys(item)
    elif isinstance(value, list):
        for item in value:
            yield from _walk_keys(item)


def test_tracked_proposal_is_complete_and_not_authorized():
    """The complete tracked proposal passes without implying a write grant."""
    report = AUDIT.validate_proposal(_proposal(), _schema())

    assert report['status'] == 'PROPOSAL_VALID_NOT_AUTHORIZED'
    assert report['issue_count'] == 29
    assert report['close_proposals'] == 23
    assert report['open_or_reproduction_proposals'] == 6
    assert report['remote_mutations_performed'] is False


def test_tracked_proposal_covers_each_issue_once_and_omits_raw_content():
    """The durable record has complete IDs without raw support content."""
    proposal = _proposal()
    numbers = [item['issue_number'] for item in proposal['issues']]

    assert numbers == sorted(numbers)
    assert len(numbers) == len(set(numbers)) == 29
    assert set(_walk_keys(proposal)).isdisjoint({
        'author',
        'authors',
        'body',
        'comment',
        'comments',
        'user',
    })
    assert proposal['privacy'] == {
        'author_identities_written': False,
        'comment_bodies_written': False,
        'issue_bodies_written': False,
    }


def test_issue_69_linked_public_claim_is_required_and_exclusive():
    """The dated Draft/CI/release claims cannot disappear or move."""
    proposal = _proposal()
    issue_69 = next(
        item for item in proposal['issues'] if item['issue_number'] == 69
    )
    issue_69['linked_claims'].pop()
    with pytest.raises(AUDIT.ProposalError, match='schema validation failed'):
        AUDIT.validate_proposal(proposal, _schema())

    proposal = _proposal()
    issue_69 = next(
        item for item in proposal['issues'] if item['issue_number'] == 69
    )
    claim = issue_69['linked_claims'].pop()
    issue_64 = next(
        item for item in proposal['issues'] if item['issue_number'] == 64
    )
    issue_64['linked_claims'] = claim
    with pytest.raises(AUDIT.ProposalError, match='schema validation failed'):
        AUDIT.validate_proposal(proposal, _schema())


def test_duplicate_issue_is_rejected():
    """Coverage cannot count the same GitHub issue twice."""
    proposal = _proposal()
    proposal['issues'].append(copy.deepcopy(proposal['issues'][-1]))
    proposal['source']['issue_count'] += 1
    proposal['summary']['issue_count'] += 1

    with pytest.raises(AUDIT.ProposalError, match='duplicates'):
        AUDIT.validate_proposal(proposal, _schema())


def test_summary_count_drift_is_rejected():
    """Human summary totals cannot drift from the issue records."""
    proposal = _proposal()
    proposal['summary']['decisions']['keep-open'] += 1

    with pytest.raises(AUDIT.ProposalError, match='decisions counts'):
        AUDIT.validate_proposal(proposal, _schema())


def test_proposal_cannot_silently_remove_an_observed_label():
    """Label proposals are additive unless a later reviewed format says so."""
    proposal = _proposal()
    issue = next(
        item for item in proposal['issues'] if item['issue_number'] == 422)
    issue['proposed_labels'].remove('help wanted')

    with pytest.raises(AUDIT.ProposalError, match='removes an observed label'):
        AUDIT.validate_proposal(proposal, _schema())


def test_unknown_remote_label_is_rejected():
    """The plan cannot assume that an uncreated GitHub label exists."""
    proposal = _proposal()
    issue = proposal['issues'][0]
    issue['proposed_labels'] = sorted(
        issue['proposed_labels'] + ['triage-ready'])

    with pytest.raises(AUDIT.ProposalError, match='unknown labels'):
        AUDIT.validate_proposal(proposal, _schema())


def test_close_decision_requires_matching_state_reason():
    """The GitHub closed-state reason stays bound to the decision."""
    proposal = _proposal()
    issue = next(
        item for item in proposal['issues'] if item['issue_number'] == 116)
    issue['state_reason'] = 'completed'

    with pytest.raises(AUDIT.ProposalError, match='not_planned'):
        AUDIT.validate_proposal(proposal, _schema())


def test_claimed_authorization_requires_a_reference():
    """A bare Boolean can never stand in for explicit write authorization."""
    proposal = _proposal()
    proposal['authority']['github_writes_authorized'] = True

    with pytest.raises(AUDIT.ProposalError, match='authorization_reference'):
        AUDIT.validate_proposal(proposal, _schema())


def test_exact_live_snapshot_passes():
    """An unchanged read-only GitHub snapshot is safe to review."""
    proposal = _proposal()
    issues, labels = _live_snapshot(proposal)

    AUDIT.validate_live_snapshot(proposal, issues, labels)


def test_live_issue_set_drift_is_rejected():
    """A closed, opened, added, or removed issue invalidates the batch."""
    proposal = _proposal()
    issues, labels = _live_snapshot(proposal)
    issues.pop()

    with pytest.raises(AUDIT.ProposalError, match='open-issue set drifted'):
        AUDIT.validate_live_snapshot(proposal, issues, labels)


@pytest.mark.parametrize('field', ['title', 'updated_at', 'labels'])
def test_live_issue_field_drift_is_rejected(field):
    """Title, timestamp, and label changes each require a fresh review."""
    proposal = _proposal()
    issues, labels = _live_snapshot(proposal)
    if field == 'labels':
        issues[0][field] = sorted(issues[0][field] + ['question'])
    else:
        issues[0][field] += '-changed'

    verb = 'have' if field == 'labels' else 'has'
    with pytest.raises(AUDIT.ProposalError, match=f'{field} {verb} drifted'):
        AUDIT.validate_live_snapshot(proposal, issues, labels)


def test_live_label_catalog_drift_is_rejected():
    """Applying nonexistent or newly ambiguous labels is fail-closed."""
    proposal = _proposal()
    issues, labels = _live_snapshot(proposal)
    labels.append('new-label')

    with pytest.raises(AUDIT.ProposalError, match='label catalog has drifted'):
        AUDIT.validate_live_snapshot(proposal, issues, labels)


def test_cli_emits_machine_readable_offline_report():
    """Maintainers can run the default audit without network access."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--json'],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    report = json.loads(result.stdout)
    assert report['status'] == 'PROPOSAL_VALID_NOT_AUTHORIZED'
    assert report['live_check'] == 'NOT_RUN'
    assert report['remote_mutations_performed'] is False


def test_github_adapter_contains_no_write_method():
    """The proposal checker is structurally incapable of remote mutation."""
    source = SCRIPT.read_text(encoding='utf-8')

    assert "'GET'" in source
    assert "'POST'" not in source
    assert "'PATCH'" not in source
    assert "'PUT'" not in source
    assert "'DELETE'" not in source


def test_checker_is_registered_with_package_ctest():
    """The offline audit runs in the normal lidarslam test registration."""
    cmake = (ROOT / 'lidarslam' / 'CMakeLists.txt').read_text(
        encoding='utf-8')

    assert 'test_check_issue_triage_proposal' in cmake


def test_checker_is_in_the_public_release_bundle():
    """Release evidence includes the exact checker used by maintainers."""
    builder = (ROOT / 'scripts' / 'build_release_bundle.py').read_text(
        encoding='utf-8')

    assert "'scripts/check_issue_triage_proposal.py'" in builder
