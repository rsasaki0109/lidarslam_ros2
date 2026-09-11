#!/usr/bin/env python3
"""Fail-closed audit of the tracked lidarslam_ros2 v1.0 readiness gates."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = REPO_ROOT / 'docs' / 'contracts' / 'v1-readiness.json'
DEFAULT_CONTRACT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'v1-readiness-contract-v1.schema.json'
)
DEFAULT_REPORT_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'v1-readiness-report-v1.schema.json'
)
DEFAULT_LEDGER = (
    REPO_ROOT
    / 'docs'
    / 'evidence'
    / 'external-first-map-validations.json'
)
DEFAULT_LEDGER_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'external-first-map-validations-v1.schema.json'
)
REPORT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/v1-readiness-report-v1.schema.json'
)
EXPECTED_GATE_IDS = {
    'first-success',
    'official-surface',
    'diagnosability',
    'reproducibility',
    'distribution',
    'reliability',
    'compatibility',
    'oss-operations',
    'external-adoption',
    'release-publication',
}
NDT_RELEASE_STATUSES = {
    'LOCAL_READY',
    'READY_TO_TAG',
    'IN_PROGRESS',
    'REVIEW_REQUIRED',
    'RELEASED',
    'BLOCKED',
}
PUBLISHED_RELEASE_STATUSES = {
    'NOT_PUBLISHED',
    'IN_PROGRESS',
    'PUBLISHED',
    'BLOCKED',
}
PACKAGE_MANAGER_RELEASE_STATUSES = {
    'SOURCE_REF_MISSING',
    'NOT_RUN',
    'RUNNING',
    'FAILED',
    'READY',
    'BLOCKED',
}
SEMVER_PATTERN = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')


class ReadinessError(ValueError):
    """The readiness contract or its evidence is structurally invalid."""


def _load_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReadinessError(f'cannot read JSON object {path}: {exc}') from exc
    if not isinstance(payload, dict):
        raise ReadinessError(f'JSON root must be an object: {path}')
    return payload


def _validate_schema(payload: dict[str, Any], schema_path: Path) -> None:
    schema = _load_object(schema_path)
    try:
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(schema).validate(payload)
    except (jsonschema.SchemaError, jsonschema.ValidationError) as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise ReadinessError(
            f'{schema_path.name} validation failed at '
            f'{location or "<root>"}: {exc.message}'
        ) from exc


def _version_tuple(value: str) -> tuple[int, int, int]:
    if not SEMVER_PATTERN.fullmatch(value):
        raise ReadinessError(
            f'product version must be MAJOR.MINOR.PATCH: {value!r}')
    return tuple(int(part) for part in value.split('.'))  # type: ignore[return-value]


def _load_checker(repo_root: Path, filename: str, module_name: str) -> Any:
    path = repo_root / 'scripts' / filename
    spec = importlib.util.spec_from_file_location(
        module_name,
        path,
    )
    if spec is None or spec.loader is None:
        raise ReadinessError(f'cannot load readiness checker: {path}')
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as exc:
        raise ReadinessError(
            f'cannot load readiness checker {path}: {exc}') from exc
    return module


def _external_checker(repo_root: Path) -> Any:
    return _load_checker(
        repo_root,
        'check_external_first_map_readiness.py',
        'external_first_map_readiness_for_v1',
    )


def inspect_live_publication(
    *,
    repo_root: Path,
    product_version: str,
    published_release_version: str,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Run the authoritative, read-only publication and distribution audits."""
    ndt_checker = _load_checker(
        repo_root,
        'check_ndt_omp_release_readiness.py',
        'ndt_omp_release_readiness_for_v1',
    )
    release_checker = _load_checker(
        repo_root,
        'check_published_release.py',
        'published_release_for_v1',
    )
    package_manager_checker = _load_checker(
        repo_root,
        'check_package_manager_release_readiness.py',
        'package_manager_release_for_v1',
    )
    try:
        ndt_report = ndt_checker.evaluate_readiness(repo_root=repo_root)
        snapshot = release_checker.inspect_remote(published_release_version)
        published_report = release_checker.evaluate_publication(
            version=published_release_version,
            snapshot=snapshot,
        )
        package_manager_snapshot = package_manager_checker.inspect_remote(
            product_version,
        )
        package_manager_report = package_manager_checker.evaluate_readiness(
            version=product_version,
            snapshot=package_manager_snapshot,
        )
    except Exception as exc:
        raise ReadinessError(
            f'live publication audit could not be trusted: {exc}') from exc
    return ndt_report, published_report, package_manager_report


def _git_tags(repo_root: Path) -> set[str]:
    result = subprocess.run(
        ['git', 'tag', '--list'],
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ReadinessError(
            f'cannot list git tags: {result.stderr.strip()}')
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def _missing_evidence(
    repo_root: Path,
    evidence: list[str],
) -> list[str]:
    root = repo_root.resolve()
    missing: list[str] = []
    for relative_text in evidence:
        relative = Path(relative_text)
        if relative.is_absolute():
            raise ReadinessError(
                f'evidence path must be repository-relative: {relative_text}')
        resolved = (root / relative).resolve()
        try:
            resolved.relative_to(root)
        except ValueError as exc:
            raise ReadinessError(
                f'evidence path escapes repository: {relative_text}') from exc
        if not resolved.is_file():
            missing.append(relative_text)
    return missing


def evaluate_readiness(
    *,
    repo_root: Path = REPO_ROOT,
    contract_path: Path = DEFAULT_CONTRACT,
    contract_schema_path: Path = DEFAULT_CONTRACT_SCHEMA,
    report_schema_path: Path = DEFAULT_REPORT_SCHEMA,
    ledger_path: Path = DEFAULT_LEDGER,
    ledger_schema_path: Path = DEFAULT_LEDGER_SCHEMA,
    tags: set[str] | None = None,
    external_report: dict[str, Any] | None = None,
    require_live_publication: bool = False,
    ndt_release_report: dict[str, Any] | None = None,
    published_release_report: dict[str, Any] | None = None,
    package_manager_report: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Validate the contract and return one deterministic readiness report."""
    repo_root = repo_root.resolve()
    contract = _load_object(contract_path)
    _validate_schema(contract, contract_schema_path)

    gate_ids = [gate['id'] for gate in contract['gates']]
    if len(gate_ids) != len(set(gate_ids)):
        raise ReadinessError('readiness gate ids must be unique')
    if set(gate_ids) != EXPECTED_GATE_IDS:
        raise ReadinessError(
            'readiness contract gate ids differ from the v1.0 scope: '
            f'missing={sorted(EXPECTED_GATE_IDS - set(gate_ids))}, '
            f'extra={sorted(set(gate_ids) - EXPECTED_GATE_IDS)}'
        )

    version_path = repo_root / contract['product_version_source']
    try:
        product_version = version_path.read_text(encoding='utf-8').strip()
    except OSError as exc:
        raise ReadinessError(
            f'cannot read product version {version_path}: {exc}') from exc
    product_tuple = _version_tuple(product_version)
    minimum_version = contract['minimum_release_candidate_version']
    minimum_tuple = _version_tuple(minimum_version)
    expected_tag = f'v{minimum_version}'
    available_tags = _git_tags(repo_root) if tags is None else tags
    minimum_version_met = product_tuple >= minimum_tuple
    tag_present = expected_tag in available_tags

    if external_report is None:
        checker = _external_checker(repo_root)
        try:
            external_report = checker.validate_ledger(
                ledger_path,
                ledger_schema_path,
            )
        except checker.LedgerError as exc:
            raise ReadinessError(
                f'external first-map ledger invalid: {exc}') from exc

    if require_live_publication:
        if (
            ndt_release_report is None
            or published_release_report is None
            or package_manager_report is None
        ):
            raise ReadinessError(
                'live publication and distribution reports are required '
                'for strict evaluation')
        ndt_status = ndt_release_report.get('status')
        published_status = published_release_report.get('status')
        package_manager_status = package_manager_report.get('status')
        if ndt_status not in NDT_RELEASE_STATUSES:
            raise ReadinessError(
                f'NDT live publication report has invalid status: {ndt_status!r}')
        if published_status not in PUBLISHED_RELEASE_STATUSES:
            raise ReadinessError(
                'lidarslam live publication report has invalid status: '
                f'{published_status!r}'
            )
        if package_manager_status not in PACKAGE_MANAGER_RELEASE_STATUSES:
            raise ReadinessError(
                'package-manager live distribution report has invalid status: '
                f'{package_manager_status!r}'
            )
    else:
        ndt_status = None
        published_status = None
        package_manager_status = None

    gate_reports: list[dict[str, Any]] = []
    for gate in contract['gates']:
        blockers = list(gate['blockers'])
        missing = _missing_evidence(repo_root, gate['evidence'])
        blockers.extend(
            f'Missing tracked evidence: {path}' for path in missing)
        complete = gate['state'] == 'complete' and not missing
        detail = (
            'Tracked gate attestation and every evidence path are complete.'
            if complete
            else 'Tracked gate remains incomplete.'
        )

        if gate['id'] == 'external-adoption':
            complete = (
                gate['state'] == 'complete'
                and external_report['status'] == 'READY'
                and not missing
            )
            detail = (
                f"{external_report['accepted_validations']} / "
                f"{external_report['required_validations']} accepted "
                'independent validations.'
            )
            if external_report['status'] != 'READY':
                dynamic = (
                    f"{external_report['remaining_validations']} independent "
                    'first-map validation(s) remain.'
                )
                if dynamic not in blockers:
                    blockers.append(dynamic)

        if gate['id'] == 'distribution' and require_live_publication:
            complete = (
                complete
                and ndt_status == 'RELEASED'
                and package_manager_status == 'READY'
            )
            detail = (
                f'Tracked distribution attestation; live ndt_omp_ros2 '
                f'release status={ndt_status}; package-manager main-channel '
                f'status={package_manager_status}.'
            )
            if ndt_status != 'RELEASED':
                dynamic = (
                    'Live ndt_omp_ros2 release audit must report RELEASED; '
                    f'observed {ndt_status}.'
                )
                if dynamic not in blockers:
                    blockers.append(dynamic)
            if package_manager_status != 'READY':
                dynamic = (
                    'Live Humble/Jazzy package-manager main-channel audit must '
                    f'report READY; observed {package_manager_status}.'
                )
                if dynamic not in blockers:
                    blockers.append(dynamic)

        if gate['id'] == 'reliability' and require_live_publication:
            complete = complete and published_status == 'PUBLISHED'
            detail = (
                'Tracked reliability attestation; live stable-release '
                f'status={published_status}.'
            )
            if published_status != 'PUBLISHED':
                dynamic = (
                    'Live stable-release audit must report PUBLISHED; '
                    f'observed {published_status}.'
                )
                if dynamic not in blockers:
                    blockers.append(dynamic)

        if gate['id'] == 'release-publication':
            complete = (
                gate['state'] == 'complete'
                and minimum_version_met
                and tag_present
                and not missing
            )
            if require_live_publication:
                complete = complete and published_status == 'PUBLISHED'
            detail = (
                f'VERSION={product_version}; minimum={minimum_version}; '
                f'expected local tag={expected_tag}; '
                f'tag_present={str(tag_present).lower()}; '
                'live stable-release status='
                f'{published_status if require_live_publication else "not inspected"}.'
            )
            if not minimum_version_met:
                blockers.append(
                    f'VERSION {product_version} is below the stable release '
                    f'candidate floor {minimum_version}.')
            if not tag_present:
                blockers.append(
                    f'Immutable release tag {expected_tag} is not present.')
            if require_live_publication and published_status != 'PUBLISHED':
                dynamic = (
                    'Live stable-release audit must report PUBLISHED; '
                    f'observed {published_status}.'
                )
                if dynamic not in blockers:
                    blockers.append(dynamic)

        gate_reports.append({
            'id': gate['id'],
            'title': gate['title'],
            'status': 'COMPLETE' if complete else 'INCOMPLETE',
            'evidence': gate['evidence'],
            'blockers': blockers,
            'detail': detail,
        })

    complete_count = sum(
        gate['status'] == 'COMPLETE' for gate in gate_reports)
    report = {
        'schema_version': 1,
        'schema_uri': REPORT_SCHEMA_URI,
        'status': (
            'READY'
            if complete_count == len(gate_reports)
            else 'NOT_READY'
        ),
        'product_version': product_version,
        'target_version': contract['target_version'],
        'minimum_release_candidate_version': minimum_version,
        'summary': {
            'total': len(gate_reports),
            'complete': complete_count,
            'incomplete': len(gate_reports) - complete_count,
        },
        'release': {
            'expected_tag': expected_tag,
            'minimum_version_met': minimum_version_met,
            'tag_present': tag_present,
        },
        'publication_audits': {
            'inspected': require_live_publication,
            'ndt_omp_ros2_status': ndt_status,
            'package_manager_release_status': package_manager_status,
            'lidarslam_release_status': published_status,
        },
        'external_first_map': external_report,
        'gates': gate_reports,
    }
    _validate_schema(report, report_schema_path)
    return report


def render_markdown(report: dict[str, Any]) -> str:
    """Render a reviewer-facing readiness summary."""
    lines = [
        '# lidarslam_ros2 v1.0 readiness',
        '',
        f"- Status: **{report['status']}**",
        f"- Product VERSION: `{report['product_version']}`",
        (
            '- Complete gates: '
            f"**{report['summary']['complete']} / "
            f"{report['summary']['total']}**"
        ),
        (
            '- Live publication audits: '
            '**'
            + (
                'inspected'
                if report['publication_audits']['inspected']
                else 'not inspected'
            )
            + '**'
        ),
        (
            '- Live status tuple (NDT / package manager / release): '
            '**'
            + (
                f"{report['publication_audits']['ndt_omp_ros2_status']} / "
                f"{report['publication_audits']['package_manager_release_status']} / "
                f"{report['publication_audits']['lidarslam_release_status']}"
                if report['publication_audits']['inspected']
                else 'not inspected'
            )
            + '**'
        ),
        '',
        '| Gate | Status | Detail |',
        '| --- | --- | --- |',
    ]
    for gate in report['gates']:
        lines.append(
            f"| `{gate['id']}` | **{gate['status']}** | "
            f"{gate['detail']} |"
        )
    lines.extend(['', '## Remaining blockers', ''])
    incomplete = [
        gate for gate in report['gates']
        if gate['status'] == 'INCOMPLETE'
    ]
    if not incomplete:
        lines.append('- None.')
    else:
        for gate in incomplete:
            lines.append(f"### {gate['title']} (`{gate['id']}`)")
            lines.append('')
            lines.extend(f'- {blocker}' for blocker in gate['blockers'])
            lines.append('')
    return '\n'.join(lines).rstrip() + '\n'


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--contract', type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument(
        '--contract-schema',
        type=Path,
        default=DEFAULT_CONTRACT_SCHEMA,
    )
    parser.add_argument(
        '--report-schema',
        type=Path,
        default=DEFAULT_REPORT_SCHEMA,
    )
    parser.add_argument('--ledger', type=Path, default=DEFAULT_LEDGER)
    parser.add_argument(
        '--ledger-schema',
        type=Path,
        default=DEFAULT_LEDGER_SCHEMA,
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print JSON instead of Markdown.',
    )
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--output-markdown', type=Path)
    parser.add_argument(
        '--live',
        action='store_true',
        help=(
            'Run the read-only NDT, package-manager, and stable-release '
            'publication audits.'
        ),
    )
    parser.add_argument(
        '--require-complete',
        action='store_true',
        help=(
            'Exit 1 unless every gate is complete; automatically run live '
            'publication audits before reporting READY.'
        ),
    )
    return parser.parse_args(argv)


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    try:
        report = evaluate_readiness(
            contract_path=args.contract,
            contract_schema_path=args.contract_schema,
            report_schema_path=args.report_schema,
            ledger_path=args.ledger,
            ledger_schema_path=args.ledger_schema,
        )
        if args.live or (args.require_complete and report['status'] == 'READY'):
            (
                ndt_report,
                published_report,
                package_manager_report,
            ) = inspect_live_publication(
                repo_root=REPO_ROOT,
                product_version=report['product_version'],
                published_release_version=(
                    report['release']['expected_tag'].removeprefix('v')
                ),
            )
            report = evaluate_readiness(
                contract_path=args.contract,
                contract_schema_path=args.contract_schema,
                report_schema_path=args.report_schema,
                ledger_path=args.ledger,
                ledger_schema_path=args.ledger_schema,
                require_live_publication=True,
                ndt_release_report=ndt_report,
                published_release_report=published_report,
                package_manager_report=package_manager_report,
            )
    except ReadinessError as exc:
        print(f'v1 readiness audit invalid: {exc}', file=sys.stderr)
        return 2

    json_text = json.dumps(report, indent=2, sort_keys=True) + '\n'
    markdown_text = render_markdown(report)
    if args.output_json:
        _write(args.output_json, json_text)
    if args.output_markdown:
        _write(args.output_markdown, markdown_text)
    print(json_text if args.json else markdown_text, end='')
    if args.require_complete and report['status'] != 'READY':
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
