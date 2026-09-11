#!/usr/bin/env python3
"""Build a privacy-bounded receipt for an external first-map report."""

from __future__ import annotations

import hashlib
import json
import re
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 1
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/first-map-validation-receipt-v1.schema.json'
)
RECEIPT_JSON_NAME = 'first_map_validation_receipt.json'
RECEIPT_MARKDOWN_NAME = 'first_map_validation_receipt.md'
MANIFEST_NAME = 'run_manifest.json'
DIAGNOSIS_NAME = 'autoware_map_diagnosis.json'
VERIFY_LOG_NAME = 'verify_autoware_map.log'
VALIDATION_ISSUE_URL = (
    'https://github.com/rsasaki0109/lidar_slam_ros2/issues/new'
    '?template=first-map-validation.yml'
)


class ReceiptError(ValueError):
    """The run artifacts cannot produce a trustworthy validation receipt."""


def _load_object(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReceiptError(
            f'cannot read JSON object {path.name}: {exc}'
        ) from exc
    if not isinstance(payload, dict):
        raise ReceiptError(f'JSON root must be an object: {path.name}')
    return payload


def _sha256(path: Path) -> str:
    try:
        with path.open('rb') as stream:
            digest = hashlib.sha256()
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError as exc:
        raise ReceiptError(f'cannot hash {path.name}: {exc}') from exc
    return digest.hexdigest()


def _optional_sha256(path: Path) -> str | None:
    return _sha256(path) if path.is_file() else None


def _verify_result(path: Path) -> str:
    if not path.is_file():
        return 'missing'
    try:
        text = path.read_text(encoding='utf-8', errors='replace')
    except OSError as exc:
        raise ReceiptError(f'cannot read {path.name}: {exc}') from exc
    match = re.search(
        r'^RESULT:\s*(PASS|FAIL)(?:\s+--[^\r\n]*)?\s*$',
        text,
        re.MULTILINE,
    )
    return match.group(1) if match else 'unknown'


def _artifact_identities(manifest: dict[str, Any]) -> dict[str, str]:
    output = manifest.get('output')
    if not isinstance(output, dict):
        return {}
    identities = output.get('artifact_checksums')
    if not isinstance(identities, list):
        return {}
    result: dict[str, str] = {}
    for identity in identities:
        if not isinstance(identity, dict):
            continue
        path = identity.get('path')
        digest = identity.get('sha256')
        if isinstance(path, str) and isinstance(digest, str):
            result[path] = digest
    return result


def _check(
    check_id: str,
    passed: bool,
    observed: str,
) -> dict[str, object]:
    return {
        'id': check_id,
        'passed': passed,
        'observed': observed,
    }


def build_receipt(run_dir: Path) -> dict[str, Any]:
    """Return a deterministic, geometry-free receipt for one run directory."""
    run_dir = run_dir.expanduser().resolve()
    if not run_dir.is_dir():
        raise ReceiptError(f'run directory does not exist: {run_dir}')

    manifest_path = run_dir / MANIFEST_NAME
    diagnosis_path = run_dir / DIAGNOSIS_NAME
    verify_log_path = run_dir / VERIFY_LOG_NAME
    if not manifest_path.is_file():
        raise ReceiptError(
            f'required run artifact is missing: {manifest_path.name}'
        )

    manifest = _load_object(manifest_path)
    diagnosis = (
        _load_object(diagnosis_path)
        if diagnosis_path.is_file()
        else {}
    )
    manifest_sha256 = _sha256(manifest_path)
    diagnosis_sha256 = _optional_sha256(diagnosis_path)
    verify_log_sha256 = _optional_sha256(verify_log_path)

    run_id = manifest.get('run_id')
    if not isinstance(run_id, str) or not run_id:
        raise ReceiptError('run_manifest.json has no valid run_id')
    software = manifest.get('software')
    profile = manifest.get('profile')
    lifecycle = manifest.get('lifecycle')
    if not isinstance(software, dict):
        raise ReceiptError('run_manifest.json has no software object')
    if not isinstance(profile, dict):
        raise ReceiptError('run_manifest.json has no profile object')
    if not isinstance(lifecycle, dict):
        raise ReceiptError('run_manifest.json has no lifecycle object')

    manifest_status = manifest.get('status')
    diagnosis_status = diagnosis.get('status')
    autoware_status = _verify_result(verify_log_path)
    recorded = _artifact_identities(manifest)

    checks = [
        _check(
            'manifest_succeeded',
            manifest_status == 'succeeded',
            str(manifest_status or 'missing'),
        ),
        _check(
            'lifecycle_complete',
            lifecycle.get('stage') == 'complete',
            str(lifecycle.get('stage') or 'missing'),
        ),
        _check(
            'runner_exit_zero',
            lifecycle.get('runner_exit_code') == 0,
            str(lifecycle.get('runner_exit_code')),
        ),
        _check(
            'diagnosis_success',
            diagnosis_status == 'success',
            str(diagnosis_status or 'missing'),
        ),
        _check(
            'autoware_verification_pass',
            autoware_status == 'PASS',
            str(autoware_status or 'missing'),
        ),
        _check(
            'diagnosis_bound_to_manifest',
            (
                diagnosis_sha256 is not None
                and recorded.get(DIAGNOSIS_NAME) == diagnosis_sha256
            ),
            (
                'matched'
                if (
                    diagnosis_sha256 is not None
                    and recorded.get(DIAGNOSIS_NAME) == diagnosis_sha256
                )
                else 'missing-or-mismatched'
            ),
        ),
        _check(
            'verify_log_bound_to_manifest',
            (
                verify_log_sha256 is not None
                and recorded.get(VERIFY_LOG_NAME) == verify_log_sha256
            ),
            (
                'matched'
                if (
                    verify_log_sha256 is not None
                    and recorded.get(VERIFY_LOG_NAME) == verify_log_sha256
                )
                else 'missing-or-mismatched'
            ),
        ),
    ]
    status = 'PASS' if all(item['passed'] for item in checks) else 'FAIL'

    return {
        'schema_version': SCHEMA_VERSION,
        'schema_uri': SCHEMA_URI,
        'status': status,
        'run': {
            'run_id': run_id,
            'product_version': str(
                software.get('product_version') or 'unknown'
            ),
            'git_commit': (
                str(software['git_commit'])
                if software.get('git_commit') is not None
                else None
            ),
            'profile_id': str(profile.get('id') or 'unknown'),
        },
        'verification': {
            'manifest_status': str(manifest_status or 'missing'),
            'diagnosis_status': str(diagnosis_status or 'missing'),
            'autoware_status': str(autoware_status or 'missing'),
            'manifest_sha256': manifest_sha256,
        },
        'evidence': {
            'manifest': {
                'filename': MANIFEST_NAME,
                'sha256': manifest_sha256,
            },
            'diagnosis': {
                'filename': DIAGNOSIS_NAME,
                'available': diagnosis_sha256 is not None,
                'sha256': diagnosis_sha256,
            },
            'verify_log': {
                'filename': VERIFY_LOG_NAME,
                'available': verify_log_sha256 is not None,
                'sha256': verify_log_sha256,
            },
        },
        'checks': checks,
        'shareability': {
            'contains_map_geometry': False,
            'contains_private_paths': False,
            'contains_exact_command': False,
            'review_before_sharing': True,
        },
    }


def render_markdown(receipt: dict[str, Any]) -> str:
    """Render the issue-form verification block and a compact check table."""
    verification = receipt['verification']
    run = receipt['run']
    lines = [
        '# First-map validation receipt',
        '',
        f"- Receipt status: **{receipt['status']}**",
        f"- Run ID: `{run['run_id']}`",
        f"- Product version: `{run['product_version']}`",
        f"- Git commit: `{run['git_commit'] or 'unknown'}`",
        f"- Profile: `{run['profile_id']}`",
        '',
        '## Verification summary',
        '',
        '```text',
        f"manifest_status={verification['manifest_status']}",
        f"diagnosis_status={verification['diagnosis_status']}",
        f"autoware_status={verification['autoware_status']}",
        f"manifest_sha256={verification['manifest_sha256']}",
        '```',
        '',
        '## Checks',
        '',
        '| Check | Result | Observed |',
        '| --- | --- | --- |',
    ]
    for check in receipt['checks']:
        result = 'PASS' if check['passed'] else 'FAIL'
        lines.append(
            f"| `{check['id']}` | {result} | `{check['observed']}` |"
        )
    lines.extend([
        '',
        'This receipt intentionally excludes map geometry, private paths, and '
        'the exact command. Review the receipt before sharing it and redact '
        'the separately reported command if it contains a private path.',
        '',
        '## Submit this run',
        '',
        f'[Open the Independent First-map Validation issue form]'
        f'({VALIDATION_ISSUE_URL}). Both PASS and FAIL reports are useful.',
        '',
        'After reviewing `first_map_validation_receipt.json`, attach that '
        'JSON file to the issue. Do not attach the manifest, map, logs, bag, '
        'or any other run artifact.',
    ])
    return '\n'.join(lines) + '\n'


def _write_atomic(path: Path, text: str) -> None:
    temporary = path.with_name(f'.{path.name}.tmp')
    try:
        temporary.write_text(text, encoding='utf-8')
        temporary.replace(path)
    except OSError:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


def write_receipt(
    run_dir: Path,
    receipt: dict[str, Any] | None = None,
) -> tuple[dict[str, Any], dict[str, Path]]:
    """Write receipts without exposing run-directory paths."""
    run_dir = run_dir.expanduser().resolve()
    report = receipt if receipt is not None else build_receipt(run_dir)
    json_path = run_dir / RECEIPT_JSON_NAME
    markdown_path = run_dir / RECEIPT_MARKDOWN_NAME
    try:
        _write_atomic(
            json_path,
            json.dumps(report, indent=2, sort_keys=True) + '\n',
        )
        _write_atomic(markdown_path, render_markdown(report))
    except OSError as exc:
        raise ReceiptError(f'cannot write validation receipt: {exc}') from exc
    return report, {
        'json': json_path,
        'markdown': markdown_path,
    }
