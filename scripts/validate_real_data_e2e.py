#!/usr/bin/env python3
"""Validate a pinned real-data golden-path run against its release contract."""

from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
RUN_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v2.schema.json'
DIAGNOSIS_SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'diagnosis-v1.schema.json'


def _load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(payload, dict):
        raise ValueError(f'JSON root must be an object: {path}')
    return payload


def _digest(path: Path, algorithm: str) -> str:
    value = hashlib.new(algorithm)
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def _line_count(path: Path) -> int:
    with path.open('rb') as stream:
        return sum(1 for line in stream if line.strip())


def _parse_timestamp(value: str) -> datetime:
    return datetime.fromisoformat(value.replace('Z', '+00:00'))


def validate(
    contract_path: Path,
    intake_path: Path,
    run_dir: Path,
) -> dict[str, Any]:
    """Return a machine-readable PASS/FAIL report."""
    contract = _load_json(contract_path)
    intake = _load_json(intake_path)
    run_manifest = _load_json(run_dir / 'run_manifest.json')
    diagnosis = _load_json(run_dir / 'autoware_map_diagnosis.json')
    checks: list[dict[str, str]] = []

    def check(check_id: str, passed: bool, detail: str) -> None:
        checks.append({
            'id': check_id,
            'status': 'PASS' if passed else 'FAIL',
            'detail': detail,
        })

    check(
        'contract_schema',
        contract.get('schema_version') == 1,
        f"schema_version={contract.get('schema_version')!r}",
    )

    dataset = contract['dataset']
    intake_dataset = intake.get('dataset') or {}
    intake_file = intake.get('file') or {}
    check(
        'dataset_identity',
        intake_dataset.get('id') == dataset['id']
        and intake_file.get('id') == dataset['file_id']
        and intake_file.get('filename') == dataset['filename'],
        (
            f"dataset={intake_dataset.get('id')!r}, "
            f"file={intake_file.get('id')!r}"
        ),
    )
    check(
        'intake_ready',
        intake.get('status') == 'READY',
        f"status={intake.get('status')!r}",
    )
    check(
        'dataset_source',
        intake_dataset.get('source_url') == dataset['source_url']
        and intake_file.get('url', '').startswith(
            dataset['source_url'] + '/files/'
        ),
        f"source={intake_dataset.get('source_url')!r}",
    )

    archive_path = Path(intake.get('archive_path', ''))
    archive_exists = archive_path.is_file()
    archive_size = archive_path.stat().st_size if archive_exists else -1
    archive_md5 = _digest(archive_path, 'md5') if archive_exists else ''
    archive_sha256 = _digest(archive_path, 'sha256') if archive_exists else ''
    check(
        'archive_identity',
        archive_exists
        and archive_size == dataset['size_bytes']
        and archive_md5 == dataset['md5']
        and archive_sha256 == dataset.get('sha256'),
        (
            f'size_bytes={archive_size}, md5={archive_md5 or "missing"}, '
            f'sha256={archive_sha256 or "missing"}'
        ),
    )

    try:
        jsonschema.validate(run_manifest, _load_json(RUN_SCHEMA))
    except jsonschema.ValidationError as exc:
        check('run_manifest_schema', False, exc.message)
    else:
        check('run_manifest_schema', True, 'schema-v2 valid')
    try:
        jsonschema.validate(diagnosis, _load_json(DIAGNOSIS_SCHEMA))
    except jsonschema.ValidationError as exc:
        check('diagnosis_schema', False, exc.message)
    else:
        check('diagnosis_schema', True, 'diagnosis-v1 valid')

    expected_input = contract['input']
    actual_input = run_manifest.get('input') or {}
    metadata_size_matches = (
        actual_input.get('metadata_size_bytes')
        == expected_input['metadata_size_bytes']
    )
    metadata_hash_matches = (
        actual_input.get('metadata_sha256')
        == expected_input['metadata_sha256']
    )
    storage_type_matches = (
        actual_input.get('storage_identifier')
        == expected_input['storage_identifier']
    )
    check(
        'bag_metadata_identity',
        metadata_size_matches
        and metadata_hash_matches
        and storage_type_matches,
        (
            f"size_bytes={actual_input.get('metadata_size_bytes')!r}, "
            f"sha256={actual_input.get('metadata_sha256')!r}"
        ),
    )
    check(
        'bag_storage_identity',
        actual_input.get('storage_files') == expected_input['storage_files'],
        f"storage_files={actual_input.get('storage_files')!r}",
    )

    profile = run_manifest.get('profile') or {}
    execution = run_manifest.get('execution') or {}
    lifecycle = run_manifest.get('lifecycle') or {}
    output = run_manifest.get('output') or {}
    expected_execution = contract['execution']
    check(
        'terminal_success',
        run_manifest.get('status') == 'succeeded'
        and execution.get('exit_code') == 0
        and lifecycle.get('stage') == 'complete'
        and lifecycle.get('runner_exit_code') == 0
        and lifecycle.get('last_error') is None
        and output.get('finalized') is True
        and output.get('diagnosis_status') == 'success',
        (
            f"status={run_manifest.get('status')!r}, "
            f"stage={lifecycle.get('stage')!r}, "
            f"exit={execution.get('exit_code')!r}"
        ),
    )
    check(
        'maintained_profile',
        profile.get('id') == expected_execution['profile_id'],
        f"profile_id={profile.get('id')!r}",
    )
    check(
        'ros_distro',
        (run_manifest.get('software') or {}).get('ros_distro')
        == expected_execution['ros_distro'],
        (
            'ros_distro='
            f"{(run_manifest.get('software') or {}).get('ros_distro')!r}"
        ),
    )
    argv_text = '\n'.join(execution.get('argv') or [])
    missing_argv = [
        fragment
        for fragment in expected_execution['required_argv_fragments']
        if fragment not in argv_text
    ]
    check(
        'execution_contract',
        not missing_argv,
        f'missing_fragments={missing_argv!r}',
    )
    try:
        runtime_sec = (
            _parse_timestamp(execution['finished_at'])
            - _parse_timestamp(execution['started_at'])
        ).total_seconds()
    except (KeyError, TypeError, ValueError) as exc:
        runtime_sec = -1.0
        runtime_detail = str(exc)
    else:
        runtime_detail = f'runtime_sec={runtime_sec:.3f}'
    check(
        'runtime_budget',
        0 < runtime_sec <= expected_execution['maximum_runtime_sec'],
        runtime_detail,
    )

    preflight = diagnosis.get('bag_preflight') or {}
    summary = preflight.get('summary') or {}
    actual_topics = {}
    for category in ('pointcloud2', 'imu'):
        for topic in (summary.get('topics') or {}).get(category, []):
            actual_topics[topic.get('name')] = {
                'type': topic.get('msg_type'),
                'message_count': topic.get('message_count'),
            }
    duration_sec = summary.get('duration_sec')
    duration_ok = isinstance(duration_sec, (int, float)) and abs(
        duration_sec - expected_input['duration_sec']
    ) <= expected_input['duration_tolerance_sec']
    check(
        'preflight_identity',
        summary.get('message_count') == expected_input['message_count']
        and duration_ok
        and all(
            actual_topics.get(name) == expected
            for name, expected in expected_input['topics'].items()
        ),
        (
            f'duration_sec={duration_sec!r}, '
            f"message_count={summary.get('message_count')!r}, "
            f'topics={actual_topics!r}'
        ),
    )

    verify = diagnosis.get('verify') or {}
    verify_counts = verify.get('counts') or {}
    check(
        'diagnosis_success',
        diagnosis.get('status') == 'success'
        and verify.get('result') == 'PASS'
        and verify_counts.get('fail') == 0
        and verify_counts.get('pass', 0)
        >= contract['output']['minimum_verify_passes'],
        (
            f"status={diagnosis.get('status')!r}, "
            f"verify={verify.get('result')!r}, counts={verify_counts!r}"
        ),
    )

    raw_path = run_dir / 'traj_raw.tum'
    corrected_path = run_dir / 'traj_corrected.tum'
    raw_poses = _line_count(raw_path) if raw_path.is_file() else 0
    corrected_poses = (
        _line_count(corrected_path) if corrected_path.is_file() else 0
    )
    check(
        'trajectory_evidence',
        raw_poses >= contract['output']['minimum_raw_poses']
        and corrected_poses >= contract['output']['minimum_corrected_poses'],
        f'raw_poses={raw_poses}, corrected_poses={corrected_poses}',
    )

    tiles = list((run_dir / 'pointcloud_map').glob('*.pcd'))
    pointcloud_bytes = sum(path.stat().st_size for path in tiles)
    check(
        'pointcloud_evidence',
        len(tiles) >= contract['output']['minimum_pointcloud_tiles']
        and pointcloud_bytes >= contract['output']['minimum_pointcloud_bytes'],
        f'tiles={len(tiles)}, bytes={pointcloud_bytes}',
    )

    failures = [row for row in checks if row['status'] == 'FAIL']
    return {
        'schema_version': 1,
        'contract_id': contract.get('id'),
        'status': 'PASS' if not failures else 'FAIL',
        'contract_path': str(contract_path.resolve()),
        'intake_manifest_path': str(intake_path.resolve()),
        'run_dir': str(run_dir.resolve()),
        'checks': checks,
        'summary': {
            'pass': len(checks) - len(failures),
            'fail': len(failures),
            'runtime_sec': runtime_sec,
            'raw_poses': raw_poses,
            'corrected_poses': corrected_poses,
            'pointcloud_tiles': len(tiles),
            'pointcloud_bytes': pointcloud_bytes,
        },
    }


def render_markdown(report: dict[str, Any]) -> str:
    """Render the validation report for Actions summaries and artifacts."""
    lines = [
        '# Real-data E2E validation',
        '',
        f"- contract: `{report['contract_id']}`",
        f"- status: **{report['status']}**",
        f"- run: `{report['run_dir']}`",
        '',
        '| Check | Status | Detail |',
        '| --- | --- | --- |',
    ]
    for row in report['checks']:
        detail = row['detail'].replace('|', '\\|').replace('\n', ' ')
        lines.append(f"| `{row['id']}` | **{row['status']}** | {detail} |")
    lines.extend([
        '',
        '## Evidence summary',
        '',
        f"- runtime: `{report['summary']['runtime_sec']:.3f} s`",
        f"- raw poses: `{report['summary']['raw_poses']}`",
        f"- corrected poses: `{report['summary']['corrected_poses']}`",
        f"- pointcloud tiles: `{report['summary']['pointcloud_tiles']}`",
        f"- pointcloud bytes: `{report['summary']['pointcloud_bytes']}`",
    ])
    return '\n'.join(lines) + '\n'


def main() -> int:
    """Validate CLI inputs, write evidence reports, and return gate status."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--contract', type=Path, required=True)
    parser.add_argument('--intake-manifest', type=Path, required=True)
    parser.add_argument('--run-dir', type=Path, required=True)
    parser.add_argument('--output-json', type=Path)
    parser.add_argument('--output-markdown', type=Path)
    args = parser.parse_args()

    try:
        report = validate(
            args.contract.resolve(),
            args.intake_manifest.resolve(),
            args.run_dir.resolve(),
        )
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(
            f'error: real-data E2E validation could not run: {exc}',
            file=sys.stderr,
        )
        return 2

    payload = json.dumps(report, indent=2, sort_keys=True) + '\n'
    markdown = render_markdown(report)
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(payload, encoding='utf-8')
    if args.output_markdown:
        args.output_markdown.parent.mkdir(parents=True, exist_ok=True)
        args.output_markdown.write_text(markdown, encoding='utf-8')
    print(markdown, end='')
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
