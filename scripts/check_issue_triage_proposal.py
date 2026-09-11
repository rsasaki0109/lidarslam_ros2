#!/usr/bin/env python3
"""Validate an issue-triage proposal without mutating GitHub."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import urllib.parse
from collections import Counter
from pathlib import Path
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROPOSAL = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'growth'
    / 'open-issue-triage-proposal-2026-08-11.json'
)
DEFAULT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'issue-triage-proposal-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/issue-triage-proposal-v1.schema.json'
)
THEMES = (
    'advanced-scope',
    'install-dependency',
    'quality-reliability',
    'sensor-platform',
    'tf-input-output',
    'validation-community',
)
PRIORITIES = ('P1', 'P2', 'P3')
DECISIONS = (
    'close-answered',
    'close-not-planned',
    'close-resolved',
    'close-superseded',
    'keep-open',
    'request-current-reproduction',
)
OPEN_DECISIONS = {'keep-open', 'request-current-reproduction'}
CLOSE_COMPLETED_DECISIONS = {
    'close-answered',
    'close-resolved',
    'close-superseded',
}
PAGE_SIZE = 100
MAX_PAGES = 10


class ProposalError(ValueError):
    """The proposal or its live source cannot be trusted."""


def _load_json(path: Path, label: str) -> Any:
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ProposalError(f'cannot read {label} {path}: {exc}') from exc


def _schema_error_path(error: jsonschema.ValidationError) -> str:
    path = '.'.join(str(item) for item in error.absolute_path)
    return path or '<root>'


def _count_all(values: list[str], keys: tuple[str, ...]) -> dict[str, int]:
    counts = Counter(values)
    return {key: counts.get(key, 0) for key in keys}


def _require_sorted_unique(values: list[Any], label: str) -> None:
    if values != sorted(values):
        raise ProposalError(f'{label} must be sorted')
    if len(values) != len(set(values)):
        raise ProposalError(f'{label} must not contain duplicates')


def _validate_decision(issue: dict[str, Any]) -> None:
    decision = issue['decision']
    state_reason = issue['state_reason']
    number = issue['issue_number']
    if decision in OPEN_DECISIONS and state_reason is not None:
        raise ProposalError(
            f'issue #{number} open decision must have null state_reason')
    if decision in CLOSE_COMPLETED_DECISIONS and state_reason != 'completed':
        raise ProposalError(
            f'issue #{number} {decision} must use completed state_reason')
    if decision == 'close-not-planned' and state_reason != 'not_planned':
        raise ProposalError(
            f'issue #{number} close-not-planned must use not_planned '
            'state_reason')


def validate_proposal(
    proposal: dict[str, Any],
    schema: dict[str, Any],
) -> dict[str, Any]:
    """Validate schema, coverage, counts, labels, and action invariants."""
    validator = jsonschema.Draft7Validator(
        schema,
        format_checker=jsonschema.FormatChecker(),
    )
    errors = sorted(
        validator.iter_errors(proposal),
        key=lambda item: list(item.absolute_path),
    )
    if errors:
        first = errors[0]
        raise ProposalError(
            f'schema validation failed at {_schema_error_path(first)}: '
            f'{first.message}')
    if proposal['schema_uri'] != SCHEMA_URI:
        raise ProposalError('proposal schema_uri is not the supported v1 URI')

    catalog = proposal['label_catalog']
    _require_sorted_unique(catalog, 'label_catalog')
    catalog_set = set(catalog)
    issues = proposal['issues']
    numbers = [item['issue_number'] for item in issues]
    _require_sorted_unique(numbers, 'issue numbers')

    for issue in issues:
        number = issue['issue_number']
        observed_labels = issue['observed']['labels']
        proposed_labels = issue['proposed_labels']
        _require_sorted_unique(
            observed_labels,
            f'issue #{number} observed labels',
        )
        _require_sorted_unique(
            proposed_labels,
            f'issue #{number} proposed labels',
        )
        if not set(observed_labels).issubset(proposed_labels):
            raise ProposalError(
                f'issue #{number} proposal removes an observed label')
        unknown = set(observed_labels + proposed_labels) - catalog_set
        if unknown:
            raise ProposalError(
                f'issue #{number} uses unknown labels: {sorted(unknown)}')
        expected_url = (
            f"https://github.com/{proposal['repository']}/issues/{number}"
        )
        if issue['url'] != expected_url:
            raise ProposalError(
                f'issue #{number} URL does not match the repository')
        _validate_decision(issue)

    issue_count = len(issues)
    if proposal['source']['issue_count'] != issue_count:
        raise ProposalError('source issue_count does not match issues')
    summary = proposal['summary']
    if summary['issue_count'] != issue_count:
        raise ProposalError('summary issue_count does not match issues')
    expected_unlabeled = sum(
        not item['observed']['labels'] for item in issues)
    if summary['current_unlabeled'] != expected_unlabeled:
        raise ProposalError('summary current_unlabeled is inconsistent')
    expected_good_first = sum(
        'good first issue' in item['observed']['labels'] for item in issues)
    if summary['current_good_first_issue'] != expected_good_first:
        raise ProposalError(
            'summary current_good_first_issue is inconsistent')

    expected_counts = {
        'themes': _count_all(
            [item['theme'] for item in issues], THEMES),
        'priorities': _count_all(
            [item['priority'] for item in issues], PRIORITIES),
        'decisions': _count_all(
            [item['decision'] for item in issues], DECISIONS),
    }
    for key, expected in expected_counts.items():
        if summary[key] != expected:
            raise ProposalError(f'summary {key} counts are inconsistent')

    authority = proposal['authority']
    if authority['proposal_applied']:
        raise ProposalError('an unapplied proposal cannot be marked applied')
    if authority['github_writes_authorized']:
        if not authority['authorization_reference']:
            raise ProposalError(
                'authorized proposal requires authorization_reference')
        status = 'PROPOSAL_VALID_AUTHORIZED_NOT_APPLIED'
    else:
        if authority['authorization_reference'] is not None:
            raise ProposalError(
                'unauthorized proposal must not claim an authorization '
                'reference')
        status = 'PROPOSAL_VALID_NOT_AUTHORIZED'

    close_count = sum(
        item['decision'].startswith('close-') for item in issues)
    return {
        'status': status,
        'repository': proposal['repository'],
        'captured_at': proposal['captured_at'],
        'issue_count': issue_count,
        'close_proposals': close_count,
        'open_or_reproduction_proposals': issue_count - close_count,
        'decision_counts': expected_counts['decisions'],
        'live_check': 'NOT_RUN',
        'remote_mutations_performed': False,
    }


class GhApi:
    """Read-only GitHub REST adapter backed by the authenticated gh CLI."""

    def get(self, endpoint: str, params: dict[str, Any]) -> Any:
        """Return one decoded JSON response from an explicit GET request."""
        query = urllib.parse.urlencode(params)
        target = endpoint if not query else f'{endpoint}?{query}'
        command = [
            'gh',
            'api',
            '--method',
            'GET',
            '-H',
            'Accept: application/vnd.github+json',
            '-H',
            'X-GitHub-Api-Version: 2022-11-28',
            target,
        ]
        try:
            result = subprocess.run(
                command,
                cwd=REPO_ROOT,
                check=False,
                capture_output=True,
                text=True,
                timeout=60,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ProposalError(
                f'cannot execute read-only GitHub request: {exc}') from exc
        if result.returncode != 0:
            detail = result.stderr.strip() or 'gh api returned no error text'
            raise ProposalError(
                f'read-only GitHub request failed for {endpoint}: {detail}')
        try:
            return json.loads(result.stdout)
        except json.JSONDecodeError as exc:
            raise ProposalError(
                f'GitHub returned invalid JSON for {endpoint}: {exc}') from exc

    def get_all(
        self,
        endpoint: str,
        params: dict[str, Any] | None = None,
    ) -> list[Any]:
        """Read every result page and reject an incomplete pagination."""
        records: list[Any] = []
        for page in range(1, MAX_PAGES + 1):
            page_params = dict(params or {})
            page_params.update({'per_page': PAGE_SIZE, 'page': page})
            payload = self.get(endpoint, page_params)
            if not isinstance(payload, list):
                raise ProposalError(
                    f'GitHub response for {endpoint} is not an array')
            records.extend(payload)
            if len(payload) < PAGE_SIZE:
                return records
        raise ProposalError(
            f'GitHub pagination exceeded {MAX_PAGES} pages for {endpoint}')


def _live_label_names(value: Any, label: str) -> list[str]:
    if not isinstance(value, list):
        raise ProposalError(f'{label} labels are not an array')
    names = []
    for index, raw in enumerate(value):
        if isinstance(raw, str):
            name = raw
        elif isinstance(raw, dict):
            name = raw.get('name')
        else:
            name = None
        if not isinstance(name, str) or not name:
            raise ProposalError(f'{label} label[{index}] has no name')
        names.append(name)
    return sorted(names)


def fetch_live_snapshot(
    repository: str,
    api: GhApi | None = None,
) -> tuple[list[dict[str, Any]], list[str]]:
    """Fetch current open issues and label names using GET requests only."""
    api = api or GhApi()
    endpoint = f'/repos/{repository}/issues'
    raw_issues = api.get_all(endpoint, {'state': 'open'})
    issues = []
    for index, raw in enumerate(raw_issues):
        if not isinstance(raw, dict):
            raise ProposalError(f'live issue[{index}] is not an object')
        if 'pull_request' in raw:
            continue
        number = raw.get('number')
        title = raw.get('title')
        updated_at = raw.get('updated_at')
        state = raw.get('state')
        if not isinstance(number, int) or number < 1:
            raise ProposalError(f'live issue[{index}] has invalid number')
        if not isinstance(title, str) or not title:
            raise ProposalError(f'live issue #{number} has invalid title')
        if not isinstance(updated_at, str) or not updated_at:
            raise ProposalError(
                f'live issue #{number} has invalid updated_at')
        if state != 'open':
            raise ProposalError(f'live issue #{number} is not open')
        issues.append({
            'issue_number': number,
            'title': title,
            'updated_at': updated_at,
            'labels': _live_label_names(
                raw.get('labels'),
                f'live issue #{number}',
            ),
        })
    label_records = api.get_all(f'/repos/{repository}/labels')
    labels = _live_label_names(label_records, 'live repository')
    return sorted(issues, key=lambda item: item['issue_number']), labels


def validate_live_snapshot(
    proposal: dict[str, Any],
    live_issues: list[dict[str, Any]],
    live_labels: list[str],
) -> None:
    """Fail if any state used by the proposal has drifted on GitHub."""
    if sorted(live_labels) != proposal['label_catalog']:
        raise ProposalError('live repository label catalog has drifted')
    expected = {
        item['issue_number']: item for item in proposal['issues']
    }
    current = {
        item['issue_number']: item for item in live_issues
    }
    if set(current) != set(expected):
        missing = sorted(set(expected) - set(current))
        extra = sorted(set(current) - set(expected))
        raise ProposalError(
            f'live open-issue set drifted: missing={missing}, extra={extra}')
    for number, item in expected.items():
        live = current[number]
        observed = item['observed']
        if live['title'] != item['title']:
            raise ProposalError(f'live issue #{number} title has drifted')
        if live['updated_at'] != observed['updated_at']:
            raise ProposalError(f'live issue #{number} updated_at has drifted')
        if live['labels'] != observed['labels']:
            raise ProposalError(f'live issue #{number} labels have drifted')


def _render_text(report: dict[str, Any]) -> str:
    lines = [
        report['status'],
        f"repository: {report['repository']}",
        f"captured_at: {report['captured_at']}",
        f"issues: {report['issue_count']}",
        f"close proposals: {report['close_proposals']}",
        (
            'keep/reproduction proposals: '
            f"{report['open_or_reproduction_proposals']}"
        ),
        f"live check: {report['live_check']}",
        'remote mutations performed: false',
    ]
    return '\n'.join(lines)


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Validate a complete issue-triage proposal without writing to '
            'GitHub.'
        ),
    )
    parser.add_argument(
        'proposal',
        nargs='?',
        type=Path,
        default=DEFAULT_PROPOSAL,
    )
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument(
        '--live',
        action='store_true',
        help='compare the snapshot with current GitHub GET responses',
    )
    parser.add_argument('--json', action='store_true', dest='as_json')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run the offline audit and optional read-only live drift check."""
    args = _parse_args(argv)
    try:
        proposal = _load_json(args.proposal, 'proposal')
        schema = _load_json(args.schema, 'schema')
        if not isinstance(proposal, dict) or not isinstance(schema, dict):
            raise ProposalError('proposal and schema must be JSON objects')
        report = validate_proposal(proposal, schema)
        if args.live:
            live_issues, live_labels = fetch_live_snapshot(
                proposal['repository'])
            validate_live_snapshot(proposal, live_issues, live_labels)
            report['live_check'] = 'PASS'
        if args.as_json:
            print(json.dumps(report, indent=2, sort_keys=True))
        else:
            print(_render_text(report))
        return 0
    except ProposalError as exc:
        print(f'ISSUE_TRIAGE_PROPOSAL_INVALID: {exc}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
