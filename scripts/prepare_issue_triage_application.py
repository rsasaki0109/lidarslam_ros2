#!/usr/bin/env python3
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

"""Prepare issue-specific triage actions without mutating GitHub."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import sys
import urllib.parse
from pathlib import Path, PurePosixPath
from typing import Any, Sequence

import jsonschema


sys.dont_write_bytecode = True

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROPOSAL = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'open-issue-triage-proposal-2026-08-11.json'
)
DEFAULT_PROPOSAL_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'issue-triage-proposal-v1.schema.json'
)
DEFAULT_PACKET_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'issue-triage-application-packet-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/issue-triage-application-packet-v1.schema.json'
)
CLOSE_DECISIONS = {
    'close-answered',
    'close-not-planned',
    'close-resolved',
    'close-superseded',
}
DECISION_SUMMARIES = {
    'close-answered': 'answered by the current guidance',
    'close-not-planned': 'not planned within the supported scope',
    'close-resolved': 'resolved by the current implementation or workflow',
    'close-superseded': 'superseded by the current supported path',
    'keep-open': 'keeping this issue open',
    'request-current-reproduction': (
        'requesting one current supported-version reproduction'
    ),
}
PRIORITY_ORDER = {'P1': 1, 'P2': 2, 'P3': 3}
LINKED_CHECK_AUTHORITY = {
    'network_reads_performed': True,
    'github_writes_authorized': False,
    'merge_authorized': False,
    'remote_mutations_performed': False,
}


def _load_proposal_checker() -> Any:
    path = REPO_ROOT / 'scripts' / 'check_issue_triage_proposal.py'
    spec = importlib.util.spec_from_file_location(
        '_issue_triage_proposal_checker',
        path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError('cannot load the issue-triage proposal checker')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PROPOSAL_CHECKER = _load_proposal_checker()


def _load_g0_readiness_checker() -> Any:
    path = REPO_ROOT / 'scripts' / 'check_g0_readiness.py'
    spec = importlib.util.spec_from_file_location(
        '_g0_readiness_checker_for_issue_triage',
        path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError('cannot load the G0 readiness checker')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


G0_READINESS = _load_g0_readiness_checker()


class ApplicationPacketError(ValueError):
    """The proposed application packet cannot be trusted."""


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ApplicationPacketError(
            f'cannot read {label} {path}: {exc}'
        ) from exc
    if not isinstance(payload, dict):
        raise ApplicationPacketError(f'{label} must be a JSON object')
    return payload


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _canonical_json_sha256(payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        payload,
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('utf-8')
    return _sha256_bytes(encoded)


def _relative_regular_file(path: Path, label: str) -> tuple[str, Path]:
    try:
        resolved = path.resolve(strict=True)
        relative = resolved.relative_to(REPO_ROOT.resolve())
    except (OSError, ValueError) as exc:
        raise ApplicationPacketError(
            f'{label} must be a repository file: {path}'
        ) from exc
    if path.is_symlink() or not resolved.is_file():
        raise ApplicationPacketError(
            f'{label} must be a regular nonsymlink file: {path}'
        )
    return relative.as_posix(), resolved


def _validate_repo_path(value: str) -> Path:
    candidate = PurePosixPath(value)
    if (
        not value
        or value.startswith('/')
        or '\\' in value
        or '\n' in value
        or '\r' in value
        or '..' in candidate.parts
        or str(candidate) != value
    ):
        raise ApplicationPacketError(
            f'invalid evidence repository path: {value!r}'
        )
    path = REPO_ROOT / value
    try:
        resolved = path.resolve(strict=True)
        resolved.relative_to(REPO_ROOT.resolve())
    except (OSError, ValueError) as exc:
        raise ApplicationPacketError(
            f'evidence path is unavailable: {value}'
        ) from exc
    if path.is_symlink() or not resolved.is_file():
        raise ApplicationPacketError(
            f'evidence path is not a regular nonsymlink file: {value}'
        )
    return resolved


def _evidence_record(value: str) -> dict[str, Any]:
    parsed = urllib.parse.urlparse(value)
    if parsed.scheme or parsed.netloc:
        if (
            parsed.scheme != 'https'
            or parsed.netloc != 'github.com'
            or parsed.username is not None
            or parsed.password is not None
            or parsed.query
            or parsed.fragment
        ):
            raise ApplicationPacketError(
                f'external evidence URL is outside the GitHub boundary: '
                f'{value}'
            )
        return {'kind': 'external_url', 'url': value}
    path = _validate_repo_path(value)
    return {
        'kind': 'repository_file',
        'path': value,
        'sha256': _sha256_bytes(path.read_bytes()),
    }


def _evidence_markdown(evidence: Sequence[dict[str, Any]]) -> list[str]:
    lines = []
    for item in evidence:
        if item['kind'] == 'repository_file':
            lines.append(f"- `{item['path']}`")
        else:
            lines.append(f"- {item['url']}")
    return lines


def _response_draft(
    issue: dict[str, Any],
    evidence: Sequence[dict[str, Any]],
) -> str:
    decision = DECISION_SUMMARIES[issue['decision']]
    lines = [
        'Thank you for reporting this.',
        '',
        'What we found:',
        '',
        issue['rationale'],
        '',
        f'Current status: **{decision}**.',
        '',
        'What happens next:',
        '',
        issue['response_summary'],
        '',
        'Evidence reviewed:',
        '',
        *_evidence_markdown(evidence),
    ]
    return '\n'.join(lines)


def _content_status(
    dependencies: Sequence[str],
    proposed_changes: dict[str, Any],
) -> str:
    mutation_requested = (
        bool(proposed_changes['labels_to_add'])
        or proposed_changes['close_issue']
        or proposed_changes['comment_required']
    )
    if not mutation_requested:
        return 'MONITOR_ONLY'
    if dependencies:
        return 'DEPENDENCY_REVIEW_REQUIRED'
    return 'READY_FOR_MAINTAINER_REVIEW'


def _build_action(issue: dict[str, Any], order: int) -> dict[str, Any]:
    observed_labels = issue['observed']['labels']
    labels_to_add = sorted(
        set(issue['proposed_labels']) - set(observed_labels)
    )
    close_issue = issue['decision'] in CLOSE_DECISIONS
    comment_required = (
        close_issue
        or issue['decision'] == 'request-current-reproduction'
        or bool(labels_to_add)
    )
    proposed_changes = {
        'labels_to_add': labels_to_add,
        'comment_required': comment_required,
        'close_issue': close_issue,
        'state_reason': issue['state_reason'],
    }
    evidence = [_evidence_record(value) for value in issue['evidence']]
    dependencies = list(issue['apply_after'])
    return {
        'order': order,
        'issue_number': issue['issue_number'],
        'title': issue['title'],
        'url': issue['url'],
        'observed': {
            'updated_at': issue['observed']['updated_at'],
            'labels': list(observed_labels),
        },
        'theme': issue['theme'],
        'priority': issue['priority'],
        'decision': issue['decision'],
        'content_status': _content_status(
            dependencies,
            proposed_changes,
        ),
        'prerequisites': {
            'live_recheck_required': True,
            'linked_claim_ids': [
                claim['id'] for claim in issue.get('linked_claims', [])
            ],
            'dependency_ids': dependencies,
            'maintainer_content_review_required': True,
            'explicit_write_authorization_required': True,
        },
        'proposed_changes': proposed_changes,
        'rationale': issue['rationale'],
        'response_summary': issue['response_summary'],
        'public_response_draft': _response_draft(issue, evidence),
        'evidence': evidence,
        'recheck_before_apply': issue['recheck_before_apply'],
    }


def _sort_key(issue: dict[str, Any]) -> tuple[int, int]:
    return PRIORITY_ORDER[issue['priority']], issue['issue_number']


def _selected_issues(
    proposal: dict[str, Any],
    issue_number: int | None,
) -> list[dict[str, Any]]:
    issues = sorted(proposal['issues'], key=_sort_key)
    if issue_number is None:
        return issues
    selected = [
        issue for issue in issues
        if issue['issue_number'] == issue_number
    ]
    if not selected:
        raise ApplicationPacketError(
            f'issue #{issue_number} is not in the proposal'
        )
    return selected


def _linked_claims(
    issues: Sequence[dict[str, Any]],
) -> list[tuple[int, dict[str, Any]]]:
    claims = [
        (issue['issue_number'], claim)
        for issue in issues
        for claim in issue.get('linked_claims', [])
    ]
    claim_ids = [claim['id'] for _, claim in claims]
    if len(claim_ids) != len(set(claim_ids)):
        raise ApplicationPacketError(
            'linked claim IDs must not contain duplicates'
        )
    return claims


def _expected_linked_result(
    issue_number: int,
    claim: dict[str, Any],
) -> dict[str, Any]:
    common = {
        'id': claim['id'],
        'issue_number': issue_number,
        'kind': claim['kind'],
        'status': 'PASS',
    }
    if claim['kind'] == 'product-draft':
        return {
            **common,
            'remote_head': claim['expected_head_sha'],
            'state': claim['expected_state'],
            'is_draft': claim['expected_is_draft'],
            'mergeable': claim['expected_mergeable'],
            'passing_check_count': claim['expected_passing_check_count'],
            'skipped_check_count': claim['expected_skipped_check_count'],
            'pending_check_count': claim['expected_pending_check_count'],
            'failing_check_count': claim['expected_failing_check_count'],
            'required_checks_complete': (
                claim['expected_required_checks_complete']
            ),
            'authority': dict(LINKED_CHECK_AUTHORITY),
        }
    if claim['kind'] == 'stable-release-absence':
        return {
            **common,
            'fix_commit': claim['fix_commit_sha'],
            'latest_stable_tag': claim['expected_latest_stable_tag'],
            'latest_stable_commit': (
                claim['expected_latest_stable_commit']
            ),
            'latest_stable_relation': (
                claim['expected_latest_stable_relation']
            ),
            'latest_stable_behind_by': (
                claim['expected_latest_stable_behind_by']
            ),
            'candidate_tag': claim['expected_candidate_tag'],
            'candidate_tag_present': (
                claim['expected_candidate_tag_present']
            ),
            'candidate_release_present': (
                claim['expected_candidate_release_present']
            ),
            'authority': dict(LINKED_CHECK_AUTHORITY),
        }
    raise ApplicationPacketError(
        f"linked claim {claim['id']} has an unsupported kind"
    )


def _raise_linked_drift(
    claim: dict[str, Any],
    observed: dict[str, Any],
    expected: dict[str, Any],
) -> None:
    drifted = [
        key for key, value in expected.items()
        if observed.get(key) != value
    ]
    if drifted:
        raise ApplicationPacketError(
            f"linked claim {claim['id']} drifted: "
            + ', '.join(sorted(drifted))
        )


def _verify_product_draft_claim(
    issue_number: int,
    claim: dict[str, Any],
    auditor: Any,
) -> dict[str, Any]:
    try:
        observed = auditor(local_head=claim['expected_head_sha'])
    except G0_READINESS.G0ReadinessError as exc:
        raise ApplicationPacketError(
            f"linked claim {claim['id']} could not be checked: {exc}"
        ) from exc
    expected = {
        'pull_request': claim['pull_request'],
        'status': claim['expected_status'],
        'remote_head': claim['expected_head_sha'],
        'head_matches_local': True,
        'state': claim['expected_state'],
        'is_draft': claim['expected_is_draft'],
        'mergeable': claim['expected_mergeable'],
        'passing_check_count': claim['expected_passing_check_count'],
        'skipped_check_count': claim['expected_skipped_check_count'],
        'pending_check_count': claim['expected_pending_check_count'],
        'failing_check_count': claim['expected_failing_check_count'],
        'required_checks_complete': (
            claim['expected_required_checks_complete']
        ),
        'blockers': [],
        'authority': LINKED_CHECK_AUTHORITY,
    }
    _raise_linked_drift(claim, observed, expected)
    return _expected_linked_result(issue_number, claim)


def _verify_stable_release_absence_claim(
    issue_number: int,
    claim: dict[str, Any],
    fetcher: Any,
) -> dict[str, Any]:
    repository = claim['repository']
    stable_tag = claim['expected_latest_stable_tag']
    candidate_tag = claim['expected_candidate_tag']
    fix_commit = claim['fix_commit_sha']
    paths = {
        'latest': f'repos/{repository}/releases/latest',
        'stable_commit': f'repos/{repository}/commits/{stable_tag}',
        'relation': (
            f'repos/{repository}/compare/{fix_commit}...{stable_tag}'
        ),
        'candidate_tag': (
            f'repos/{repository}/git/ref/tags/{candidate_tag}'
        ),
        'candidate_release': (
            f'repos/{repository}/releases/tags/{candidate_tag}'
        ),
    }
    responses = {}
    try:
        for name, path in paths.items():
            responses[name] = fetcher(path)
    except G0_READINESS.G0ReadinessError as exc:
        raise ApplicationPacketError(
            f"linked claim {claim['id']} could not be checked: {exc}"
        ) from exc

    latest_status, latest = responses['latest']
    commit_status, commit = responses['stable_commit']
    relation_status, relation = responses['relation']
    candidate_tag_status, _ = responses['candidate_tag']
    candidate_release_status, _ = responses['candidate_release']
    latest = latest if isinstance(latest, dict) else {}
    commit = commit if isinstance(commit, dict) else {}
    relation = relation if isinstance(relation, dict) else {}
    merge_base = relation.get('merge_base_commit')
    merge_base = merge_base if isinstance(merge_base, dict) else {}
    observed = {
        'latest_http_status': latest_status,
        'latest_tag': latest.get('tag_name'),
        'latest_draft': latest.get('draft'),
        'latest_prerelease': latest.get('prerelease'),
        'latest_url': latest.get('html_url'),
        'stable_commit_http_status': commit_status,
        'stable_commit': commit.get('sha'),
        'relation_http_status': relation_status,
        'relation': relation.get('status'),
        'ahead_by': relation.get('ahead_by'),
        'behind_by': relation.get('behind_by'),
        'total_commits': relation.get('total_commits'),
        'merge_base': merge_base.get('sha'),
        'candidate_tag_http_status': candidate_tag_status,
        'candidate_release_http_status': candidate_release_status,
    }
    expected = {
        'latest_http_status': 200,
        'latest_tag': stable_tag,
        'latest_draft': False,
        'latest_prerelease': False,
        'latest_url': (
            f'https://github.com/{repository}/releases/tag/{stable_tag}'
        ),
        'stable_commit_http_status': 200,
        'stable_commit': claim['expected_latest_stable_commit'],
        'relation_http_status': 200,
        'relation': claim['expected_latest_stable_relation'],
        'ahead_by': 0,
        'behind_by': claim['expected_latest_stable_behind_by'],
        'total_commits': 0,
        'merge_base': claim['expected_latest_stable_commit'],
        'candidate_tag_http_status': (
            200 if claim['expected_candidate_tag_present'] else 404
        ),
        'candidate_release_http_status': (
            200 if claim['expected_candidate_release_present'] else 404
        ),
    }
    _raise_linked_drift(claim, observed, expected)
    return _expected_linked_result(issue_number, claim)


def _verify_linked_claims(
    issues: Sequence[dict[str, Any]],
    *,
    auditor: Any = None,
    fetcher: Any = None,
) -> list[dict[str, Any]]:
    """Verify source-bound PR, CI, and release claims with GitHub GETs."""
    auditor = auditor or G0_READINESS.audit_product_draft
    fetcher = fetcher or G0_READINESS._github_json
    results = []
    for issue_number, claim in _linked_claims(issues):
        if claim['kind'] == 'product-draft':
            result = _verify_product_draft_claim(
                issue_number,
                claim,
                auditor,
            )
        elif claim['kind'] == 'stable-release-absence':
            result = _verify_stable_release_absence_claim(
                issue_number,
                claim,
                fetcher,
            )
        else:
            raise ApplicationPacketError(
                f"linked claim {claim['id']} has an unsupported kind"
            )
        results.append(result)
    return results


def _linked_check(
    claims: Sequence[tuple[int, dict[str, Any]]],
    results: Sequence[dict[str, Any]] | None,
) -> dict[str, Any]:
    if not claims:
        if results:
            raise ApplicationPacketError(
                'linked results exist without selected claims'
            )
        return {
            'performed': False,
            'status': 'NOT_REQUIRED',
            'claim_count': 0,
            'results': [],
        }
    if results is None:
        return {
            'performed': False,
            'status': 'NOT_RUN',
            'claim_count': len(claims),
            'results': [],
        }
    return {
        'performed': True,
        'status': 'PASS',
        'claim_count': len(claims),
        'results': list(results),
    }


def _schema_error_path(error: jsonschema.ValidationError) -> str:
    path = '.'.join(str(item) for item in error.absolute_path)
    return path or '<root>'


def validate_packet(
    packet: dict[str, Any],
    schema: dict[str, Any],
) -> None:
    """Validate schema plus ordering, action, evidence, and authority rules."""
    validator = jsonschema.Draft7Validator(
        schema,
        format_checker=jsonschema.FormatChecker(),
    )
    errors = sorted(
        validator.iter_errors(packet),
        key=lambda item: [str(part) for part in item.absolute_path],
    )
    if errors:
        first = errors[0]
        raise ApplicationPacketError(
            f'packet schema failed at {_schema_error_path(first)}: '
            f'{first.message}'
        )
    if packet['schema_uri'] != SCHEMA_URI:
        raise ApplicationPacketError(
            'packet schema_uri is not the supported v1 URI'
        )

    source = packet['source']
    source_proposal_path = _validate_repo_path(source['proposal_path'])
    source_proposal = _load_json(source_proposal_path, 'source proposal')
    source_proposal_schema = _load_json(
        DEFAULT_PROPOSAL_SCHEMA,
        'source proposal schema',
    )
    PROPOSAL_CHECKER.validate_proposal(
        source_proposal,
        source_proposal_schema,
    )
    if (
        _canonical_json_sha256(source_proposal)
        != source['proposal_canonical_sha256']
    ):
        raise ApplicationPacketError(
            'packet source proposal hash has drifted'
        )
    if source_proposal['captured_at'] != source['proposal_captured_at']:
        raise ApplicationPacketError(
            'packet source proposal capture time has drifted'
        )
    source_generator_path = _validate_repo_path(source['generator_path'])
    if source_generator_path != Path(__file__).resolve():
        raise ApplicationPacketError(
            'packet source generator is not this implementation'
        )
    if (
        _sha256_bytes(source_generator_path.read_bytes())
        != source['generator_sha256']
    ):
        raise ApplicationPacketError(
            'packet source generator hash has drifted'
        )

    actions = packet['actions']
    orders = [action['order'] for action in actions]
    if orders != list(range(1, len(actions) + 1)):
        raise ApplicationPacketError(
            'packet action order must be consecutive from one'
        )
    numbers = [action['issue_number'] for action in actions]
    if len(numbers) != len(set(numbers)):
        raise ApplicationPacketError(
            'packet issue numbers must not contain duplicates'
        )
    selection = packet['selection']
    if selection['selected_action_count'] != len(actions):
        raise ApplicationPacketError(
            'packet selected_action_count is inconsistent'
        )
    requested = selection['requested_issue_number']
    if requested is not None and numbers != [requested]:
        raise ApplicationPacketError(
            'packet requested issue selection is inconsistent'
        )
    if selection['proposal_issue_count'] != len(source_proposal['issues']):
        raise ApplicationPacketError(
            'packet proposal issue count is inconsistent with its source'
        )

    source_issues = _selected_issues(source_proposal, requested)
    source_claims = _linked_claims(source_issues)
    linked_check = packet['linked_check']
    if linked_check['claim_count'] != len(source_claims):
        raise ApplicationPacketError(
            'packet linked claim count is inconsistent with its source'
        )
    if not source_claims:
        expected_linked_status = 'NOT_REQUIRED'
    elif packet['live_check']['status'] == 'PASS':
        expected_linked_status = 'PASS'
    else:
        expected_linked_status = 'NOT_RUN'
    if linked_check['status'] != expected_linked_status:
        raise ApplicationPacketError(
            'packet linked check status is inconsistent'
        )
    expected_claim_ids = [claim['id'] for _, claim in source_claims]
    result_ids = [result['id'] for result in linked_check['results']]
    if linked_check['status'] == 'PASS':
        if result_ids != expected_claim_ids:
            raise ApplicationPacketError(
                'packet linked check results are incomplete or reordered'
            )
        for (issue_number, claim), result in zip(
            source_claims,
            linked_check['results'],
        ):
            expected_result = _expected_linked_result(
                issue_number,
                claim,
            )
            if result != expected_result:
                raise ApplicationPacketError(
                    f"linked claim {claim['id']} result is inconsistent"
                )

    expected_summary = {
        'close_action_count': sum(
            action['proposed_changes']['close_issue']
            for action in actions
        ),
        'reproduction_request_count': sum(
            action['decision'] == 'request-current-reproduction'
            for action in actions
        ),
        'dependency_review_count': sum(
            action['content_status'] == 'DEPENDENCY_REVIEW_REQUIRED'
            for action in actions
        ),
        'monitor_only_count': sum(
            action['content_status'] == 'MONITOR_ONLY'
            for action in actions
        ),
    }
    if packet['summary'] != expected_summary:
        raise ApplicationPacketError('packet summary is inconsistent')

    for action in actions:
        changes = action['proposed_changes']
        observed_labels = action['observed']['labels']
        added_labels = changes['labels_to_add']
        if observed_labels != sorted(observed_labels):
            raise ApplicationPacketError(
                f"issue #{action['issue_number']} observed labels are unsorted"
            )
        if added_labels != sorted(added_labels):
            raise ApplicationPacketError(
                f"issue #{action['issue_number']} added labels are unsorted"
            )
        if set(observed_labels) & set(added_labels):
            raise ApplicationPacketError(
                f"issue #{action['issue_number']} re-adds an observed label"
            )
        expected_close = action['decision'] in CLOSE_DECISIONS
        if changes['close_issue'] is not expected_close:
            raise ApplicationPacketError(
                f"issue #{action['issue_number']} close decision is "
                'inconsistent'
            )
        expected_reason = None
        if action['decision'] in {
            'close-answered',
            'close-resolved',
            'close-superseded',
        }:
            expected_reason = 'completed'
        elif action['decision'] == 'close-not-planned':
            expected_reason = 'not_planned'
        if changes['state_reason'] != expected_reason:
            raise ApplicationPacketError(
                f"issue #{action['issue_number']} state reason is "
                'inconsistent'
            )
        expected_comment = (
            expected_close
            or action['decision'] == 'request-current-reproduction'
            or bool(added_labels)
        )
        if changes['comment_required'] is not expected_comment:
            raise ApplicationPacketError(
                f"issue #{action['issue_number']} comment requirement is "
                'inconsistent'
            )
        expected_status = _content_status(
            action['prerequisites']['dependency_ids'],
            changes,
        )
        if action['content_status'] != expected_status:
            raise ApplicationPacketError(
                f"issue #{action['issue_number']} content status is "
                'inconsistent'
            )
        source_issue = next(
            item for item in source_issues
            if item['issue_number'] == action['issue_number']
        )
        expected_linked_ids = [
            claim['id']
            for claim in source_issue.get('linked_claims', [])
        ]
        if (
            action['prerequisites']['linked_claim_ids']
            != expected_linked_ids
        ):
            raise ApplicationPacketError(
                f"issue #{action['issue_number']} linked claims are "
                'inconsistent'
            )
        response = action['public_response_draft']
        if (
            action['rationale'] not in response
            or action['response_summary'] not in response
        ):
            raise ApplicationPacketError(
                f"issue #{action['issue_number']} response draft is not "
                'issue-specific'
            )
        for evidence in action['evidence']:
            value = (
                evidence['path']
                if evidence['kind'] == 'repository_file'
                else evidence['url']
            )
            if _evidence_record(value) != evidence:
                raise ApplicationPacketError(
                    f"issue #{action['issue_number']} evidence has drifted"
                )

    expected_request_mode = (
        'GET_ONLY' if packet['live_check']['status'] == 'PASS' else 'NONE'
    )
    if packet['authority']['github_requests'] != expected_request_mode:
        raise ApplicationPacketError(
            'packet GitHub request mode is inconsistent with live_check'
        )

    expected_actions = [
        _build_action(issue, order)
        for order, issue in enumerate(source_issues, start=1)
    ]
    if actions != expected_actions:
        raise ApplicationPacketError(
            'packet actions have drifted from the source proposal'
        )


def build_packet(
    proposal: dict[str, Any],
    proposal_schema: dict[str, Any],
    packet_schema: dict[str, Any],
    *,
    proposal_path: Path = DEFAULT_PROPOSAL,
    issue_number: int | None = None,
    live_status: str = 'NOT_RUN',
    linked_results: Sequence[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Build and validate one deterministic no-write application packet."""
    PROPOSAL_CHECKER.validate_proposal(proposal, proposal_schema)
    proposal_relative, _ = _relative_regular_file(
        proposal_path,
        'proposal',
    )
    generator_relative, generator_path = _relative_regular_file(
        Path(__file__),
        'generator',
    )

    issues = _selected_issues(proposal, issue_number)
    claims = _linked_claims(issues)
    actions = [
        _build_action(issue, index)
        for index, issue in enumerate(issues, start=1)
    ]
    packet = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': 'PREPARED_NOT_AUTHORIZED',
        'repository': proposal['repository'],
        'source': {
            'proposal_path': proposal_relative,
            'proposal_canonical_sha256': _canonical_json_sha256(proposal),
            'proposal_captured_at': proposal['captured_at'],
            'generator_path': generator_relative,
            'generator_sha256': _sha256_bytes(generator_path.read_bytes()),
        },
        'selection': {
            'requested_issue_number': issue_number,
            'proposal_issue_count': len(proposal['issues']),
            'selected_action_count': len(actions),
        },
        'live_check': {
            'performed': live_status == 'PASS',
            'status': live_status,
        },
        'linked_check': _linked_check(claims, linked_results),
        'summary': {
            'close_action_count': sum(
                action['proposed_changes']['close_issue']
                for action in actions
            ),
            'reproduction_request_count': sum(
                action['decision'] == 'request-current-reproduction'
                for action in actions
            ),
            'dependency_review_count': sum(
                action['content_status'] == 'DEPENDENCY_REVIEW_REQUIRED'
                for action in actions
            ),
            'monitor_only_count': sum(
                action['content_status'] == 'MONITOR_ONLY'
                for action in actions
            ),
        },
        'actions': actions,
        'authority': {
            'github_requests': (
                'GET_ONLY' if live_status == 'PASS' else 'NONE'
            ),
            'github_writes_authorized': False,
            'issue_comments_authorized': False,
            'issue_label_changes_authorized': False,
            'issue_state_changes_authorized': False,
            'local_files_written': False,
            'remote_mutations_performed': False,
        },
        'privacy': {
            'author_identities_retained': False,
            'comment_bodies_requested': False,
            'issue_bodies_retained': False,
            'raw_github_records_written': False,
        },
    }
    validate_packet(packet, packet_schema)
    return packet


def render_packet(packet: dict[str, Any]) -> str:
    """Render a bounded maintainer review card from the validated packet."""
    authority = packet['authority']
    lines = [
        '# Issue triage application packet',
        '',
        f"- Status: **{packet['status']}**",
        f"- Repository: `{packet['repository']}`",
        (
            '- Live drift check: '
            f"**{packet['live_check']['status']}**"
        ),
        (
            '- Linked PR/CI check: '
            f"**{packet['linked_check']['status']}**"
        ),
        (
            '- Selected actions: '
            f"{packet['selection']['selected_action_count']} / "
            f"{packet['selection']['proposal_issue_count']}"
        ),
        '- GitHub writes authorized: **no**',
        '- Remote mutations performed: **no**',
        '',
        (
            'This card is a review aid. It contains no mutation command and '
            'does not authorize posting, labeling, or closing an issue.'
        ),
    ]
    for action in packet['actions']:
        changes = action['proposed_changes']
        state_reason = changes['state_reason'] or 'null'
        labels = ', '.join(
            f'`{label}`' for label in changes['labels_to_add']
        ) or 'none'
        dependencies = ', '.join(
            f'`{item}`'
            for item in action['prerequisites']['dependency_ids']
        ) or 'none'
        linked_claims = ', '.join(
            f'`{item}`'
            for item in action['prerequisites']['linked_claim_ids']
        ) or 'none'
        lines.extend([
            '',
            (
                f"## {action['order']}. #{action['issue_number']} — "
                f"{action['title']}"
            ),
            '',
            f"- Priority: `{action['priority']}`",
            f"- Decision: `{action['decision']}`",
            f"- Content status: `{action['content_status']}`",
            f'- Labels to add: {labels}',
            f"- Close issue: `{str(changes['close_issue']).lower()}`",
            f'- State reason: `{state_reason}`',
            f'- Dependencies to review: {dependencies}',
            f'- Linked claims to recheck: {linked_claims}',
            '- Live recheck required immediately before any action: `true`',
            '- Explicit write authorization required: `true`',
            '',
            '### Prepared public response draft — review before posting',
            '',
            action['public_response_draft'],
        ])
    if authority['github_writes_authorized']:
        raise ApplicationPacketError('renderer refuses write authority')
    return '\n'.join(lines)


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--proposal', type=Path, default=DEFAULT_PROPOSAL)
    parser.add_argument(
        '--proposal-schema',
        type=Path,
        default=DEFAULT_PROPOSAL_SCHEMA,
    )
    parser.add_argument(
        '--packet-schema',
        type=Path,
        default=DEFAULT_PACKET_SCHEMA,
    )
    parser.add_argument('--issue', type=int, dest='issue_number')
    parser.add_argument(
        '--live',
        action='store_true',
        help='require a current GET-only GitHub drift check',
    )
    parser.add_argument('--json', action='store_true', dest='as_json')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Validate inputs, optionally recheck live state, and print a packet."""
    args = _parse_args(argv)
    try:
        proposal = _load_json(args.proposal, 'proposal')
        proposal_schema = _load_json(
            args.proposal_schema,
            'proposal schema',
        )
        packet_schema = _load_json(args.packet_schema, 'packet schema')
        PROPOSAL_CHECKER.validate_proposal(proposal, proposal_schema)
        live_status = 'NOT_RUN'
        linked_results = None
        if args.live:
            live_issues, live_labels = (
                PROPOSAL_CHECKER.fetch_live_snapshot(
                    proposal['repository']
                )
            )
            PROPOSAL_CHECKER.validate_live_snapshot(
                proposal,
                live_issues,
                live_labels,
            )
            live_status = 'PASS'
            selected_issues = _selected_issues(
                proposal,
                args.issue_number,
            )
            linked_results = _verify_linked_claims(selected_issues)
        packet = build_packet(
            proposal,
            proposal_schema,
            packet_schema,
            proposal_path=args.proposal,
            issue_number=args.issue_number,
            live_status=live_status,
            linked_results=linked_results,
        )
        if args.as_json:
            print(json.dumps(packet, indent=2, sort_keys=True))
        else:
            print(render_packet(packet))
        return 0
    except (
        ApplicationPacketError,
        PROPOSAL_CHECKER.ProposalError,
    ) as exc:
        print(
            f'ISSUE_TRIAGE_APPLICATION_INVALID: {exc}',
            file=sys.stderr,
        )
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
