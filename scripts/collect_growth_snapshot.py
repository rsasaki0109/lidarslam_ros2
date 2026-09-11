#!/usr/bin/env python3
"""Collect one privacy-bounded weekly project growth snapshot."""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path
import re
from statistics import median
import subprocess
import sys
from typing import Any
import urllib.parse

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
DEFAULT_MAINTAINERS = ('rsasaki0109',)
DEFAULT_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'growth-snapshot-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/growth-snapshot-v1.schema.json'
)
PRIMARY_BUNDLE_PATTERN = re.compile(
    r'^lidarslam_ros2_v[0-9]+\.[0-9]+\.[0-9]+'
    r'(?:[-+][A-Za-z0-9.-]+)?_release_bundle\.tar\.gz$'
)
TRAFFIC_WINDOW_DAYS = 14
PR_WINDOW_DAYS = 90
CONTRIBUTOR_WINDOW_DAYS = 180
RESPONSE_WINDOW_DAYS = 90
STAR_GOAL = 1000
PAGE_SIZE = 100
MAX_PAGES = 20


class SnapshotError(ValueError):
    """The weekly snapshot cannot be collected or trusted."""


def _load_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise SnapshotError(f'cannot read JSON {path}: {exc}') from exc


def _require_object(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise SnapshotError(f'{label} must be a JSON object')
    return value


def _require_list(value: Any, label: str) -> list[Any]:
    if not isinstance(value, list):
        raise SnapshotError(f'{label} must be a JSON array')
    return value


class GhApi:
    """Small read-only GitHub API client backed by the authenticated gh CLI."""

    source = 'live'

    def get(
        self,
        endpoint: str,
        params: dict[str, Any] | None = None,
    ) -> Any:
        query = urllib.parse.urlencode(params or {}, doseq=True)
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
            raise SnapshotError(
                f'cannot execute read-only GitHub request {endpoint}: {exc}'
            ) from exc
        if result.returncode != 0:
            detail = result.stderr.strip() or 'gh api returned no error text'
            raise SnapshotError(f'GitHub request failed for {endpoint}: {detail}')
        try:
            return json.loads(result.stdout)
        except json.JSONDecodeError as exc:
            raise SnapshotError(
                f'GitHub returned invalid JSON for {endpoint}: {exc}'
            ) from exc

    def get_all(
        self,
        endpoint: str,
        params: dict[str, Any] | None = None,
    ) -> list[Any]:
        records: list[Any] = []
        for page in range(1, MAX_PAGES + 1):
            page_params = dict(params or {})
            page_params.update({'per_page': PAGE_SIZE, 'page': page})
            payload = _require_list(
                self.get(endpoint, page_params),
                f'GitHub response for {endpoint}',
            )
            records.extend(payload)
            if len(payload) < PAGE_SIZE:
                return records
        raise SnapshotError(
            f'GitHub pagination exceeded {MAX_PAGES} pages for {endpoint}; '
            'refusing to emit a partial aggregate'
        )


class FixtureApi:
    """Offline API adapter for deterministic audits and tests."""

    source = 'fixture'

    _OBJECT_FILES = {
        'repository': 'repository.json',
        'traffic/views': 'traffic-views.json',
        'traffic/clones': 'traffic-clones.json',
        'traffic/popular/referrers': 'traffic-referrers.json',
        'releases/latest': 'latest-release.json',
    }

    def __init__(self, fixture_dir: Path) -> None:
        self.fixture_dir = fixture_dir

    def _key(self, endpoint: str) -> str:
        if re.fullmatch(r'/repos/[^/]+/[^/]+', endpoint):
            return 'repository'
        for suffix in self._OBJECT_FILES:
            if suffix == 'repository':
                continue
            if endpoint.endswith('/' + suffix):
                return suffix
        raise SnapshotError(f'fixture has no object route for {endpoint}')

    def get(
        self,
        endpoint: str,
        params: dict[str, Any] | None = None,
    ) -> Any:
        del params
        key = self._key(endpoint)
        return _load_json(self.fixture_dir / self._OBJECT_FILES[key])

    def get_all(
        self,
        endpoint: str,
        params: dict[str, Any] | None = None,
    ) -> list[Any]:
        del params
        if endpoint.endswith('/pulls'):
            filename = 'pulls.json'
        elif endpoint.endswith('/issues'):
            filename = 'issues.json'
        else:
            match = re.search(r'/issues/([1-9][0-9]*)/comments$', endpoint)
            if match is None:
                raise SnapshotError(f'fixture has no list route for {endpoint}')
            filename = f'issue-comments-{match.group(1)}.json'
        return _require_list(
            _load_json(self.fixture_dir / filename),
            f'fixture {filename}',
        )


def _parse_timestamp(value: Any, label: str) -> datetime:
    if not isinstance(value, str):
        raise SnapshotError(f'{label} must be an ISO-8601 timestamp')
    try:
        parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError as exc:
        raise SnapshotError(f'{label} is not an ISO-8601 timestamp') from exc
    if parsed.tzinfo is None:
        raise SnapshotError(f'{label} must include a UTC offset')
    return parsed.astimezone(timezone.utc)


def _format_timestamp(value: datetime) -> str:
    return value.astimezone(timezone.utc).replace(microsecond=0).isoformat(
    ).replace('+00:00', 'Z')


def _integer(record: dict[str, Any], key: str, label: str) -> int:
    value = record.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise SnapshotError(f'{label}.{key} must be a non-negative integer')
    return value


def _user(record: dict[str, Any], label: str) -> tuple[str, bool]:
    user = _require_object(record.get('user'), f'{label}.user')
    login = user.get('login')
    if not isinstance(login, str) or not login:
        raise SnapshotError(f'{label}.user.login must be a non-empty string')
    is_bot = user.get('type') == 'Bot' or login.casefold().endswith('[bot]')
    return login.casefold(), is_bot


def _is_external(
    record: dict[str, Any],
    maintainers: set[str],
    label: str,
) -> bool:
    login, is_bot = _user(record, label)
    return not is_bot and login not in maintainers


def _traffic_metric(record: dict[str, Any], label: str) -> dict[str, int]:
    return {
        'total': _integer(record, 'count', label),
        'unique': _integer(record, 'uniques', label),
    }


def _qualified_referrals(records: list[Any]) -> dict[str, int]:
    all_unique_sum = 0
    qualified_unique_sum = 0
    for index, raw in enumerate(records):
        record = _require_object(raw, f'referrer[{index}]')
        referrer = record.get('referrer')
        if not isinstance(referrer, str) or not referrer:
            raise SnapshotError(
                f'referrer[{index}].referrer must be a non-empty string')
        unique = _integer(record, 'uniques', f'referrer[{index}]')
        all_unique_sum += unique
        normalized = referrer.casefold().replace('.', '')
        if 'autoware' in normalized or 'tier4' in normalized:
            qualified_unique_sum += unique
    return {
        'top_referrer_unique_sum': all_unique_sum,
        'autoware_tier4_unique_sum': qualified_unique_sum,
    }


def _release_metric(record: dict[str, Any]) -> dict[str, Any]:
    tag = record.get('tag_name')
    published_at = record.get('published_at')
    if not isinstance(tag, str) or not tag:
        raise SnapshotError('latest release tag_name is missing')
    published = _format_timestamp(
        _parse_timestamp(published_at, 'latest release published_at'))
    assets = _require_list(record.get('assets'), 'latest release assets')
    if len(assets) > 100:
        raise SnapshotError('latest release has more than 100 assets')
    total_downloads = 0
    primary: list[tuple[str, int]] = []
    for index, raw in enumerate(assets):
        asset = _require_object(raw, f'release asset[{index}]')
        name = asset.get('name')
        if not isinstance(name, str) or not name:
            raise SnapshotError(
                f'release asset[{index}].name must be a non-empty string')
        downloads = _integer(
            asset,
            'download_count',
            f'release asset[{index}]',
        )
        total_downloads += downloads
        if PRIMARY_BUNDLE_PATTERN.fullmatch(name):
            primary.append((name, downloads))
    if len(primary) > 1:
        raise SnapshotError(
            'latest release contains multiple primary release bundles')
    primary_name, primary_downloads = (
        primary[0] if primary else (None, 0)
    )
    return {
        'tag': tag,
        'published_at': published,
        'total_asset_downloads': total_downloads,
        'primary_bundle': {
            'name': primary_name,
            'downloads': primary_downloads,
        },
    }


def _community_metrics(
    *,
    api: Any,
    repository: str,
    pulls: list[Any],
    issues: list[Any],
    captured_at: datetime,
    maintainers: set[str],
) -> dict[str, Any]:
    pr_cutoff = captured_at - timedelta(days=PR_WINDOW_DAYS)
    contributor_cutoff = captured_at - timedelta(
        days=CONTRIBUTOR_WINDOW_DAYS)
    response_cutoff = captured_at - timedelta(days=RESPONSE_WINDOW_DAYS)

    external_prs = 0
    merged_contributors: set[str] = set()
    for index, raw in enumerate(pulls):
        pull = _require_object(raw, f'pull[{index}]')
        external = _is_external(pull, maintainers, f'pull[{index}]')
        created = _parse_timestamp(
            pull.get('created_at'),
            f'pull[{index}].created_at',
        )
        if external and pr_cutoff <= created <= captured_at:
            external_prs += 1
        merged_raw = pull.get('merged_at')
        if external and merged_raw is not None:
            merged = _parse_timestamp(
                merged_raw,
                f'pull[{index}].merged_at',
            )
            if contributor_cutoff <= merged <= captured_at:
                login, _ = _user(pull, f'pull[{index}]')
                merged_contributors.add(login)

    open_issues = 0
    untriaged_open_issues = 0
    eligible_responses = 0
    response_hours: list[float] = []
    for index, raw in enumerate(issues):
        issue = _require_object(raw, f'issue[{index}]')
        if 'pull_request' in issue:
            continue
        state = issue.get('state')
        if state not in {'open', 'closed'}:
            raise SnapshotError(
                f'issue[{index}].state must be open or closed')
        labels = _require_list(issue.get('labels'), f'issue[{index}].labels')
        if state == 'open':
            open_issues += 1
            if not labels:
                untriaged_open_issues += 1

        created = _parse_timestamp(
            issue.get('created_at'),
            f'issue[{index}].created_at',
        )
        if not (response_cutoff <= created <= captured_at):
            continue
        if not _is_external(issue, maintainers, f'issue[{index}]'):
            continue
        number = issue.get('number')
        if not isinstance(number, int) or isinstance(number, bool) or number < 1:
            raise SnapshotError(
                f'issue[{index}].number must be a positive integer')
        eligible_responses += 1
        comments = api.get_all(
            f'/repos/{repository}/issues/{number}/comments')
        first_response: datetime | None = None
        for comment_index, comment_raw in enumerate(comments):
            comment = _require_object(
                comment_raw,
                f'issue[{index}] comment[{comment_index}]',
            )
            login, is_bot = _user(
                comment,
                f'issue[{index}] comment[{comment_index}]',
            )
            if is_bot or login not in maintainers:
                continue
            response = _parse_timestamp(
                comment.get('created_at'),
                f'issue[{index}] comment[{comment_index}].created_at',
            )
            if created <= response <= captured_at:
                if first_response is None or response < first_response:
                    first_response = response
        if first_response is not None:
            response_hours.append(
                (first_response - created).total_seconds() / 3600.0)

    responded = len(response_hours)
    return {
        'open_issues': open_issues,
        'untriaged_open_issues': untriaged_open_issues,
        'external_prs_90d': external_prs,
        'external_merged_contributors_180d': len(merged_contributors),
        'issue_first_response_90d': {
            'scope': 'external_non_bot_issues',
            'eligible': eligible_responses,
            'responded': responded,
            'unanswered': eligible_responses - responded,
            'median_hours': (
                round(float(median(response_hours)), 3)
                if response_hours else None
            ),
        },
    }


def _load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise SnapshotError(f'cannot load local metric provider {path}')
    module = importlib.util.module_from_spec(spec)
    module_dir = str(path.parent)
    inserted_path = module_dir not in sys.path
    if inserted_path:
        sys.path.insert(0, module_dir)
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        raise SnapshotError(
            f'cannot load local metric provider {path}: {exc}') from exc
    finally:
        if inserted_path:
            sys.path.remove(module_dir)
    return module


def collect_product_metrics(
    repo_root: Path = REPO_ROOT,
    captured_at: datetime | None = None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Collect accepted maps, cohort operations, and v1 readiness."""
    external = _load_module(
        repo_root / 'scripts' / 'check_external_first_map_readiness.py',
        'growth_external_first_map',
    )
    readiness = _load_module(
        repo_root / 'scripts' / 'check_v1_readiness.py',
        'growth_v1_readiness',
    )
    cohort = _load_module(
        repo_root / 'scripts' / 'first_map_validator_cohort.py',
        'growth_first_map_cohort',
    )
    try:
        first_map_report = external.validate_ledger()
        cohort_report = cohort.evaluate_state(
            _require_object(
                _load_json(
                    repo_root
                    / 'docs'
                    / 'contracts'
                    / 'first-map-validator-cohort-v1.json'
                ),
                'first-map cohort contract',
            ),
            _require_object(
                _load_json(
                    repo_root
                    / 'docs'
                    / 'schemas'
                    / 'first-map-validator-cohort-v1.schema.json'
                ),
                'first-map cohort contract schema',
            ),
            _require_object(
                _load_json(
                    repo_root
                    / 'docs'
                    / 'evidence'
                    / 'growth'
                    / 'first-map-validator-cohort-state.json'
                ),
                'first-map cohort state',
            ),
            _require_object(
                _load_json(
                    repo_root
                    / 'docs'
                    / 'schemas'
                    / 'first-map-validator-cohort-state-v1.schema.json'
                ),
                'first-map cohort state schema',
            ),
            _require_object(
                _load_json(
                    repo_root
                    / 'docs'
                    / 'evidence'
                    / 'external-first-map-validations.json'
                ),
                'accepted first-map ledger',
            ),
            _require_object(
                _load_json(
                    repo_root
                    / 'docs'
                    / 'schemas'
                    / 'external-first-map-validations-v1.schema.json'
                ),
                'accepted first-map schema',
            ),
            now=captured_at or datetime.now(timezone.utc),
        )
        readiness_report = readiness.evaluate_readiness(
            external_report=first_map_report,
        )
    except Exception as exc:
        raise SnapshotError(
            f'cannot collect trusted local product metrics: {exc}') from exc
    return first_map_report, cohort_report, readiness_report


def collect_snapshot(
    *,
    api: Any,
    repository: str = DEFAULT_REPOSITORY,
    captured_at: datetime,
    maintainers: set[str],
    first_map_report: dict[str, Any],
    cohort_report: dict[str, Any],
    readiness_report: dict[str, Any],
    annotations: list[str] | None = None,
    schema_path: Path = DEFAULT_SCHEMA,
) -> dict[str, Any]:
    """Collect, aggregate, validate, and return one weekly snapshot."""
    captured_at = captured_at.astimezone(timezone.utc)
    base = f'/repos/{repository}'
    repository_record = _require_object(
        api.get(base),
        'repository response',
    )
    views = _require_object(api.get(f'{base}/traffic/views'), 'traffic views')
    clones = _require_object(
        api.get(f'{base}/traffic/clones'),
        'traffic clones',
    )
    referrers = _require_list(
        api.get(f'{base}/traffic/popular/referrers'),
        'traffic referrers',
    )
    release = _require_object(
        api.get(f'{base}/releases/latest'),
        'latest release',
    )
    pulls = api.get_all(
        f'{base}/pulls',
        {'state': 'all', 'sort': 'created', 'direction': 'desc'},
    )
    issues = api.get_all(
        f'{base}/issues',
        {'state': 'all', 'sort': 'created', 'direction': 'desc'},
    )

    stars = _integer(repository_record, 'stargazers_count', 'repository')
    first_map_accepted = _integer(
        first_map_report,
        'accepted_validations',
        'first-map report',
    )
    first_map_required = _integer(
        first_map_report,
        'required_validations',
        'first-map report',
    )
    first_map_remaining = _integer(
        first_map_report,
        'remaining_validations',
        'first-map report',
    )
    cohort_status = cohort_report.get('status')
    allowed_cohort_statuses = {
        'WAITING_FOR_PUBLIC_GATES',
        'WAITING_FOR_OPERATIONAL_SIGNALS',
        'PAUSED_REPAIR',
        'TARGET_MET',
        'HARD_CAP_REVIEW',
        'INITIAL_BATCH_REVIEW',
        'CAPACITY_FULL',
        'READY_FOR_NEXT_ATTEMPT',
    }
    if cohort_status not in allowed_cohort_statuses:
        raise SnapshotError('first-map cohort status is unsupported')
    cohort_phase = cohort_report.get('phase')
    if cohort_phase not in {'initial', 'extended'}:
        raise SnapshotError('first-map cohort phase is unsupported')
    completion_rate = cohort_report.get('completion_rate')
    if (
        completion_rate is not None
        and (
            not isinstance(completion_rate, (int, float))
            or isinstance(completion_rate, bool)
            or not 0 <= completion_rate <= 1
        )
    ):
        raise SnapshotError('first-map cohort completion_rate is invalid')
    median_minutes = cohort_report.get('median_active_operator_minutes')
    if (
        median_minutes is not None
        and (
            not isinstance(median_minutes, (int, float))
            or isinstance(median_minutes, bool)
            or median_minutes < 0
        )
    ):
        raise SnapshotError(
            'first-map cohort median_active_operator_minutes is invalid'
        )
    signals_fresh = cohort_report.get('operational_signals_fresh')
    next_attempt = cohort_report.get('next_attempt_permitted_by_state')
    if not isinstance(signals_fresh, bool) or not isinstance(next_attempt, bool):
        raise SnapshotError('first-map cohort boolean state is invalid')
    stop_conditions = cohort_report.get('stop_conditions')
    if not isinstance(stop_conditions, list):
        raise SnapshotError('first-map cohort stop_conditions must be a list')
    if stop_conditions != sorted(set(stop_conditions)):
        raise SnapshotError(
            'first-map cohort stop_conditions must be sorted and unique'
        )
    cohort_attempted = _integer(
        cohort_report,
        'attempt_count',
        'first-map cohort',
    )
    cohort_terminal = _integer(
        cohort_report,
        'terminal_attempt_count',
        'first-map cohort',
    )
    cohort_active = _integer(
        cohort_report,
        'active_attempt_count',
        'first-map cohort',
    )
    cohort_review_wip = _integer(
        cohort_report,
        'review_wip_count',
        'first-map cohort',
    )
    cohort_successful = _integer(
        cohort_report,
        'successful_first_map_count',
        'first-map cohort',
    )
    cohort_accepted = _integer(
        cohort_report,
        'accepted_validations',
        'first-map cohort',
    )
    cohort_target = _integer(
        cohort_report,
        'accepted_target',
        'first-map cohort',
    )
    if cohort_terminal + cohort_active != cohort_attempted:
        raise SnapshotError(
            'first-map cohort terminal and active counts must equal attempted'
        )
    if not cohort_accepted <= cohort_successful <= cohort_terminal:
        raise SnapshotError(
            'first-map cohort accepted/successful/terminal counts conflict'
        )
    if not cohort_active <= cohort_review_wip <= cohort_attempted:
        raise SnapshotError(
            'first-map cohort active/review-WIP/attempted counts conflict'
        )
    if cohort_accepted > first_map_accepted:
        raise SnapshotError(
            'first-map cohort accepted count exceeds the cumulative ledger'
        )
    if cohort_target != first_map_required:
        raise SnapshotError(
            'first-map cohort target differs from the v1 accepted-map gate'
        )
    expected_completion_rate = (
        cohort_successful / cohort_attempted
        if cohort_attempted else None
    )
    if (
        completion_rate != expected_completion_rate
        and not (
            completion_rate is not None
            and expected_completion_rate is not None
            and abs(completion_rate - expected_completion_rate) <= 1e-12
        )
    ):
        raise SnapshotError(
            'first-map cohort completion_rate conflicts with its counts'
        )
    if (cohort_terminal == 0) != (median_minutes is None):
        raise SnapshotError(
            'first-map cohort median time conflicts with terminal count'
        )
    if next_attempt != (cohort_status == 'READY_FOR_NEXT_ATTEMPT'):
        raise SnapshotError(
            'first-map cohort next-attempt decision conflicts with status'
        )
    if cohort_status == 'READY_FOR_NEXT_ATTEMPT' and not signals_fresh:
        raise SnapshotError(
            'first-map cohort cannot be ready with stale operational signals'
        )
    if cohort_status == 'TARGET_MET' and cohort_accepted < cohort_target:
        raise SnapshotError(
            'first-map cohort TARGET_MET lacks accepted evidence'
        )
    if cohort_status == 'PAUSED_REPAIR' and not stop_conditions:
        raise SnapshotError('first-map cohort PAUSED_REPAIR lacks a stop reason')
    if cohort_status == 'CAPACITY_FULL' and cohort_review_wip < 2:
        raise SnapshotError('first-map cohort CAPACITY_FULL lacks full WIP')
    if cohort_status == 'INITIAL_BATCH_REVIEW' and cohort_attempted < 5:
        raise SnapshotError(
            'first-map cohort INITIAL_BATCH_REVIEW lacks five attempts'
        )
    if cohort_status == 'HARD_CAP_REVIEW' and cohort_attempted < 10:
        raise SnapshotError(
            'first-map cohort HARD_CAP_REVIEW lacks ten attempts'
        )
    summary = _require_object(
        readiness_report.get('summary'),
        'v1 readiness summary',
    )
    readiness_status = readiness_report.get('status')
    if readiness_status not in {'READY', 'NOT_READY'}:
        raise SnapshotError('v1 readiness status must be READY or NOT_READY')

    snapshot = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'repository': repository,
        'captured_at': _format_timestamp(captured_at),
        'source': {
            'github': api.source,
            'traffic_window_days': TRAFFIC_WINDOW_DAYS,
            'external_pr_window_days': PR_WINDOW_DAYS,
            'external_contributor_window_days': CONTRIBUTOR_WINDOW_DAYS,
            'response_window_days': RESPONSE_WINDOW_DAYS,
        },
        'goal': {
            'target_stars': STAR_GOAL,
            'stars_to_goal': max(STAR_GOAL - stars, 0),
        },
        'github': {
            'stars': stars,
            'forks': _integer(repository_record, 'forks_count', 'repository'),
            'subscribers': _integer(
                repository_record,
                'subscribers_count',
                'repository',
            ),
            'traffic_14d': {
                'views': _traffic_metric(views, 'traffic views'),
                'clones': _traffic_metric(clones, 'traffic clones'),
                'referrals': _qualified_referrals(referrers),
            },
            'latest_release': _release_metric(release),
            'community': _community_metrics(
                api=api,
                repository=repository,
                pulls=pulls,
                issues=issues,
                captured_at=captured_at,
                maintainers=maintainers,
            ),
        },
        'product': {
            'external_first_maps': {
                'accepted': first_map_accepted,
                'required': first_map_required,
                'remaining': first_map_remaining,
                'cohort': {
                    'id': cohort_report.get('cohort_id'),
                    'status': cohort_status,
                    'phase': cohort_phase,
                    'attempted': cohort_attempted,
                    'terminal': cohort_terminal,
                    'active': cohort_active,
                    'review_wip': cohort_review_wip,
                    'successful': cohort_successful,
                    'accepted': cohort_accepted,
                    'target': cohort_target,
                    'completion_rate': completion_rate,
                    'median_active_operator_minutes': median_minutes,
                    'operational_signals_fresh': signals_fresh,
                    'next_attempt_permitted_by_state': next_attempt,
                    'stop_conditions': list(stop_conditions),
                },
            },
            'v1_readiness': {
                'status': readiness_status,
                'complete': _integer(summary, 'complete', 'v1 summary'),
                'incomplete': _integer(summary, 'incomplete', 'v1 summary'),
                'total': _integer(summary, 'total', 'v1 summary'),
            },
        },
        'annotations': list(annotations or []),
        'privacy': {
            'personal_identifiers_written': False,
            'raw_records_written': False,
        },
    }
    schema = _require_object(_load_json(schema_path), 'snapshot schema')
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(
            schema,
            format_checker=jsonschema.FormatChecker(),
        ).validate(snapshot)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise SnapshotError(
            f'snapshot schema validation failed at '
            f'{location or "<root>"}: {exc.message}'
        ) from exc
    return snapshot


def _write_new_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rendered = json.dumps(payload, indent=2, sort_keys=True) + '\n'
    try:
        with path.open('x', encoding='utf-8') as output:
            output.write(rendered)
    except OSError as exc:
        raise SnapshotError(
            f'cannot create weekly snapshot without overwriting {path}: {exc}'
        ) from exc


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Collect one aggregate weekly GitHub, adoption, and v1 readiness '
            'snapshot without writing user identities or raw API records.'
        ),
    )
    parser.add_argument('--repository', default=DEFAULT_REPOSITORY)
    parser.add_argument(
        '--maintainer',
        action='append',
        dest='maintainers',
        help=(
            'Maintainer login used only to aggregate first-response and '
            'external contribution counts. Repeat for co-maintainers.'
        ),
    )
    parser.add_argument(
        '--captured-at',
        help='ISO-8601 timestamp for a reproducible snapshot; defaults to now.',
    )
    parser.add_argument(
        '--fixture-dir',
        type=Path,
        help='Read deterministic GitHub response fixtures instead of gh api.',
    )
    parser.add_argument(
        '--annotation',
        action='append',
        default=[],
        help='Short public intervention note. Repeat up to ten times.',
    )
    parser.add_argument('--schema', type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument(
        '--output',
        type=Path,
        help='Create a new JSON file; existing snapshots are never overwritten.',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """CLI entrypoint; collection or contract failures exit 2."""
    args = _parse_args(argv)
    try:
        captured_at = (
            _parse_timestamp(args.captured_at, '--captured-at')
            if args.captured_at
            else datetime.now(timezone.utc)
        )
        maintainer_values = (
            *DEFAULT_MAINTAINERS,
            *(args.maintainers or []),
        )
        maintainers = {
            item.casefold() for item in maintainer_values if item
        }
        if not maintainers:
            raise SnapshotError('at least one maintainer login is required')
        if len(args.annotation) > 10:
            raise SnapshotError('at most ten annotations are allowed')
        api = FixtureApi(args.fixture_dir) if args.fixture_dir else GhApi()
        first_map_report, cohort_report, readiness_report = (
            collect_product_metrics(captured_at=captured_at)
        )
        snapshot = collect_snapshot(
            api=api,
            repository=args.repository,
            captured_at=captured_at,
            maintainers=maintainers,
            first_map_report=first_map_report,
            cohort_report=cohort_report,
            readiness_report=readiness_report,
            annotations=args.annotation,
            schema_path=args.schema,
        )
        if args.output is None:
            print(json.dumps(snapshot, indent=2, sort_keys=True))
        else:
            _write_new_json(args.output, snapshot)
            print(args.output)
    except SnapshotError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
