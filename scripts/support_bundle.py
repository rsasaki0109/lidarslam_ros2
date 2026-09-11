#!/usr/bin/env python3
"""Create a privacy-bounded support attachment from one map session."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess
import sys
from typing import Any, Sequence
import zipfile


SCRIPT_DIR = Path(__file__).resolve().parent
PRODUCT_ROOT = SCRIPT_DIR.parent
SUPPORT_SCHEMA = 'support-bundle-v1.schema.json'
SUPPORT_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/support-bundle-v1.schema.json'
)
FIRST_MAP_HANDOFF_SCHEMA = 'first-map-handoff-v1.schema.json'
FIRST_MAP_HANDOFF_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/first-map-handoff-v1.schema.json'
)
FIRST_MAP_RECEIPT_NAME = 'first_map_validation_receipt.json'
FIRST_MAP_MARKDOWN_NAME = 'first_map_validation_receipt.md'
MAX_JSON_BYTES = 2 * 1024 * 1024
MAX_HASH_BYTES = 2 * 1024 * 1024
ARCHIVE_MARKER = 'lidarslam-support-bundle-v1'
ARTIFACT_NAMES = (
    'setup_manifest',
    'run_manifest',
    'diagnosis_json',
    'validation_receipt',
    'pointcloud_map',
    'map_preview_html',
    'backend_input',
    'recovery_receipt',
)
EXPECTED_BASENAMES = {
    'setup_manifest': 'sensor_setup.json',
    'run_manifest': 'run_manifest.json',
    'diagnosis_json': 'autoware_map_diagnosis.json',
    'validation_receipt': 'first_map_validation_receipt.json',
    'pointcloud_map': 'pointcloud_map',
    'backend_input': 'backend_input',
    'recovery_receipt': 'map_session_recovery.json',
}
DIRECTORY_ARTIFACTS = {'pointcloud_map', 'backend_input'}
SECRET_OPTION = re.compile(
    r'(?:password|passwd|token|secret|credential|api[-_]?key)',
    re.IGNORECASE,
)
SAFE_IDENTIFIER = re.compile(r'^[A-Za-z0-9_.:-]{1,160}$')
REPORTED_SYMPTOM_BASIS = 'USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED'
SUPPORT_MODE_ENV = 'LIDARSLAM_SUPPORT_MODE'
REPORTED_SYMPTOM_CODES = frozenset({
    'map-spins-or-spirals',
    'pose-drifts-or-oscillates',
    'map-stops-early',
    'map-is-too-sparse',
    'map-is-not-visible',
})


def _load_script_module(script_name: str, module_name: str):
    path = SCRIPT_DIR / script_name
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'failed to load {module_name} from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the privacy-bounded support-bundle command."""
    report_mode = os.environ.get(SUPPORT_MODE_ENV) == 'first-map-report'
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Revalidate a verified first map and prepare its reviewed report '
            'without writing or contacting GitHub.'
            if report_mode else
            'Create a review-before-sharing support ZIP without maps, bags, '
            'raw logs, parameter contents, or local paths.'
        ),
    )
    parser.add_argument(
        'session',
        metavar=(
            'session_bundle_or_demo_output'
            if report_mode else 'session_bundle'
        ),
        help=(
            'Verified session directory, or fixed-demo output containing '
            'first_map_validation_receipt.json.'
        ),
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all options (this command has no hidden options).',
    )
    if not report_mode:
        parser.add_argument(
            '--output',
            metavar='<file>',
            help='New ZIP path (default: beside the session bundle).',
        )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print the sanitized report without writing a ZIP.',
    )
    if not report_mode:
        parser.add_argument(
            '--first-map',
            action='store_true',
            help=(
                'Validate a verified first-map receipt and print the exact '
                'review-and-submit handoff without writing or contacting '
                'GitHub.'
            ),
        )
    args = parser.parse_args(argv)
    if report_mode:
        args.first_map = True
        args.output = None
    return args


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _json_sha256(value: Any) -> str:
    rendered = json.dumps(
        value,
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('utf-8')
    return hashlib.sha256(rendered).hexdigest()


def _read_json(path: Path) -> dict[str, Any] | None:
    if path.is_symlink() or not path.is_file():
        return None
    try:
        if path.stat().st_size > MAX_JSON_BYTES:
            return None
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _safe_identifier(value: Any) -> str:
    rendered = str(value)
    return rendered if SAFE_IDENTIFIER.fullmatch(rendered) else '<redacted>'


def _safe_optional(value: Any, *, maximum: int = 256) -> str | None:
    if value is None:
        return None
    rendered = str(value)
    if not rendered or len(rendered) > maximum:
        return '<redacted>'
    if any(ord(character) < 32 for character in rendered):
        return '<redacted>'
    return rendered


def _product_build_info() -> dict[str, Any]:
    version = None
    try:
        version = (PRODUCT_ROOT / 'VERSION').read_text(
            encoding='utf-8'
        ).strip() or None
    except OSError:
        pass
    revision = None
    dirty = None
    installed = _read_json(PRODUCT_ROOT / 'product-build-info.json')
    if installed is not None:
        candidate = installed.get('revision')
        if isinstance(candidate, str) and re.fullmatch(
            r'[0-9a-f]{40}', candidate
        ):
            revision = candidate
        if isinstance(installed.get('dirty'), bool):
            dirty = installed['dirty']
    elif (PRODUCT_ROOT / '.git').exists():
        try:
            commit = subprocess.run(
                ['git', 'rev-parse', 'HEAD'],
                cwd=PRODUCT_ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            status = subprocess.run(
                ['git', 'status', '--porcelain', '--untracked-files=no'],
                cwd=PRODUCT_ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
        except OSError:
            pass
        else:
            candidate = commit.stdout.strip().lower()
            if commit.returncode == 0 and re.fullmatch(
                r'[0-9a-f]{40}', candidate
            ):
                revision = candidate
            if status.returncode == 0:
                dirty = bool(status.stdout.strip())
    ros_distro = _safe_optional(os.environ.get('ROS_DISTRO'), maximum=64)
    return {
        'version': _safe_optional(version, maximum=64),
        'source_revision': revision,
        'source_dirty': dirty,
        'ros_distro': ros_distro,
        'platform': {
            'system': _safe_optional(platform.system()) or 'unknown',
            'release': _safe_optional(platform.release()) or 'unknown',
            'machine': _safe_optional(platform.machine()) or 'unknown',
        },
    }


def _known_private_values(
    record: dict[str, Any],
    setup_source: dict[str, Any],
) -> set[str]:
    session = record['payload']
    values = {
        str(record['bundle']),
        str(record['session_path']),
        session['bag_path'],
        session['setup_bundle'],
        session['map_output'],
        str(Path.home()),
    }
    values.update(
        value for value in session['artifacts'].values()
        if isinstance(value, str)
    )
    setup = setup_source['payload']
    if setup is not None:
        values.update({
            setup['bundle_path'],
            setup['input']['bag_path'],
            setup['input']['metadata_path'],
            setup['run']['output_dir'],
            setup['run']['command_shell'],
        })
        values.update(item['path'] for item in setup['input']['storage_files'])
        for item in setup['parameters']:
            values.add(item['source_path'])
            values.add(str(record['bundle'] / item['bundle_path']))
        argv = setup['run']['argv']
        for index, item in enumerate(argv[:-1]):
            if SECRET_OPTION.search(item):
                values.add(argv[index + 1])
        for item in argv:
            if '=' in item and SECRET_OPTION.search(item.split('=', 1)[0]):
                values.add(item.split('=', 1)[1])
    return {
        value for value in values
        if isinstance(value, str) and len(value) >= 4
    }


def _replace_known_path(
    value: str,
    replacements: Sequence[tuple[str, str]],
) -> str:
    for private, placeholder in replacements:
        root = private.rstrip('/')
        if value == private or (root and value.startswith(root + '/')):
            return placeholder
    if value.startswith('file://'):
        return '<local-uri>'
    if value.startswith('/'):
        return '<local-path>'
    if '://' in value and '@' in value:
        return '<redacted-uri>'
    return value


def _redacted_command(
    record: dict[str, Any],
    setup: dict[str, Any],
) -> dict[str, str]:
    argv = [str(item) for item in setup['run']['argv']]
    replacements = [
        (setup['input']['metadata_path'], '<bag-metadata>'),
        (setup['input']['bag_path'], '<bag>'),
        (setup['run']['output_dir'], '<map-output>'),
        (setup['bundle_path'], '<setup-bundle>'),
    ]
    for item in setup['input']['storage_files']:
        replacements.append((item['path'], '<bag-storage>'))
    for item in setup['parameters']:
        replacements.extend([
            (item['source_path'], f'<{item["role"]}-source>'),
            (
                str(record['bundle'] / item['bundle_path']),
                f'<{item["role"]}-snapshot>',
            ),
        ])
    replacements.sort(key=lambda item: len(item[0]), reverse=True)
    redacted = []
    hide_next = False
    for item in argv:
        if hide_next:
            redacted.append('<redacted-secret>')
            hide_next = False
            continue
        if item.startswith('-') and '=' in item:
            option, value = item.split('=', 1)
            if SECRET_OPTION.search(option):
                redacted.append(f'{option}=<redacted-secret>')
                continue
            rendered = _replace_known_path(value, replacements)
            redacted.append(f'{option}={rendered}')
            continue
        rendered = _replace_known_path(item, replacements)
        redacted.append(rendered)
        if item.startswith('-') and SECRET_OPTION.search(item):
            hide_next = True
    return {
        'redacted': shlex.join(redacted),
        'argv_sha256': _json_sha256(argv),
    }


def _session_projection(record: dict[str, Any]) -> dict[str, Any]:
    session = record['payload']
    reason_codes = []
    if session['reason'] is not None:
        reason_codes.append(session['reason']['code'])
    reason_codes.extend(item['code'] for item in session['findings'])
    checks = sorted(
        (
            {
                'id': item['id'],
                'status': item['status'],
                'source_checks': sorted(item['source_checks']),
            }
            for item in session['quality']['checks']
        ),
        key=lambda item: item['id'],
    )
    progress = session['progress']
    return {
        'evidence_sha256': _sha256(record['session_path']),
        'created_at': session['created_at'],
        'status': session['status'],
        'runner_exit_code': session['runner_exit_code'],
        'profile_id': _safe_identifier(session['profile']['id']),
        'verification': dict(session['verification']),
        'progress': {
            'phase': progress['phase'],
            'stage': progress['stage'],
            'current_step': progress['current_step'],
            'total_steps': progress['total_steps'],
        },
        'quality': {
            'overall': session['quality']['overall'],
            'source_status': session['quality']['source']['status'],
            'checks': checks,
        },
        'reason_codes': list(dict.fromkeys(reason_codes)),
        'action_kinds': list(dict.fromkeys(
            item['kind'] for item in session['actions']
        )),
    }


def _sanitized_mapping(
    source: dict[str, Any],
    private_values: set[str],
) -> dict[str, Any]:
    sanitized = {}
    for key, value in source.items():
        if isinstance(value, dict):
            sanitized[key] = _sanitized_mapping(value, private_values)
        elif isinstance(value, str):
            rendered = '<redacted>' if value in private_values else value
            sanitized[key] = _safe_optional(rendered)
        elif value is None or isinstance(value, (bool, int, float)):
            sanitized[key] = value
        else:
            sanitized[key] = '<redacted>'
    return sanitized


def _setup_projection(
    record: dict[str, Any],
    source: dict[str, Any],
    private_values: set[str],
) -> dict[str, Any]:
    if source['status'] != 'valid':
        return {
            'status': source['status'],
            'profile_id': None,
            'input_identity': None,
            'topics': None,
            'frames': None,
            'pointcloud': None,
            'timestamp_order_status': None,
            'calibration': None,
            'parameters': [],
            'run_command': None,
        }
    setup = source['payload']
    input_identity = setup['input']
    timestamp_status = setup['timestamp_order'].get('status')
    return {
        'status': 'valid',
        'profile_id': _safe_identifier(setup['profile']['id']),
        'input_identity': {
            'metadata_size_bytes': input_identity['metadata_size_bytes'],
            'metadata_sha256': input_identity['metadata_sha256'],
            'storage_identifier': _safe_optional(
                input_identity.get('storage_identifier')
            ),
            'storage_file_count': len(input_identity['storage_files']),
            'storage_file_sha256': sorted(
                item['sha256'] for item in input_identity['storage_files']
            ),
        },
        'topics': _sanitized_mapping(setup['topics'], private_values),
        'frames': _sanitized_mapping(setup['frames'], private_values),
        'pointcloud': {
            'inspection_status': _safe_identifier(
                setup['pointcloud']['inspection_status']
            ),
            'timestamp_field': _safe_optional(
                setup['pointcloud']['timestamp_field']
            ),
            'field_count': len(setup['pointcloud']['fields']),
        },
        'timestamp_order_status': (
            _safe_identifier(timestamp_status)
            if timestamp_status is not None else None
        ),
        'calibration': {
            'required': setup['calibration']['required'],
            'source': setup['calibration']['source'],
            'lidar_to_base_quat_xyzw_xyz': (
                setup['calibration']['lidar_to_base_quat_xyzw_xyz']
            ),
            'imu_to_base_quat_xyzw_xyz': (
                setup['calibration']['imu_to_base_quat_xyzw_xyz']
            ),
        },
        'parameters': sorted(
            (
                {
                    'role': item['role'],
                    'size_bytes': item['size_bytes'],
                    'sha256': item['sha256'],
                }
                for item in setup['parameters']
            ),
            key=lambda item: item['role'],
        ),
        'run_command': _redacted_command(record, setup),
    }


def _path_has_symlink(path: Path) -> bool:
    cursor = path
    while True:
        if cursor.is_symlink():
            return True
        if cursor.parent == cursor:
            return False
        cursor = cursor.parent


def _inside_any(path: Path, roots: Sequence[Path]) -> bool:
    for root in roots:
        try:
            path.relative_to(root)
        except ValueError:
            continue
        return True
    return False


def _artifact_projection(
    name: str,
    recorded: str | None,
    roots: Sequence[Path],
) -> dict[str, Any]:
    result = {
        'name': name,
        'recorded': recorded is not None,
        'current_state': 'not_recorded',
        'size_bytes': None,
        'sha256': None,
    }
    if recorded is None:
        return result
    requested = Path(recorded).expanduser()
    expected = EXPECTED_BASENAMES.get(name)
    if expected is not None and requested.name != expected:
        result['current_state'] = 'unexpected_name'
        return result
    if name == 'map_preview_html' and requested.suffix.lower() != '.html':
        result['current_state'] = 'unexpected_name'
        return result
    if _path_has_symlink(requested):
        result['current_state'] = 'symlink'
        return result
    resolved = requested.resolve()
    if not _inside_any(resolved, roots):
        result['current_state'] = 'outside_evidence_roots'
        return result
    if not resolved.exists():
        result['current_state'] = 'missing'
        return result
    if name in DIRECTORY_ARTIFACTS:
        result['current_state'] = (
            'directory' if resolved.is_dir() else 'unreadable'
        )
        return result
    if not resolved.is_file():
        result['current_state'] = 'unreadable'
        return result
    try:
        size = resolved.stat().st_size
        result['size_bytes'] = size
        if size > MAX_HASH_BYTES:
            result['current_state'] = 'oversized'
            return result
        result['sha256'] = _sha256(resolved)
    except OSError:
        result['current_state'] = 'unreadable'
        result['size_bytes'] = None
        result['sha256'] = None
        return result
    result['current_state'] = 'regular_file'
    return result


def _artifact_projections(record: dict[str, Any]) -> list[dict[str, Any]]:
    session = record['payload']
    bundle = record['bundle'].resolve()
    map_output = Path(session['map_output']).expanduser().resolve()
    partial = map_output.with_name(f'{map_output.name}.partial')
    roots = (bundle, map_output, partial)
    return [
        _artifact_projection(name, session['artifacts'][name], roots)
        for name in ARTIFACT_NAMES
    ]


def _diagnosis_projection(
    record: dict[str, Any],
    artifacts: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    evidence = next(
        item for item in artifacts if item['name'] == 'diagnosis_json'
    )
    result = {
        'evidence_status': evidence['current_state'],
        'status': None,
        'verify_result': None,
        'projector_type': None,
        'problem_hint_count': 0,
        'suggested_step_count': 0,
        'reported_symptom': None,
    }
    if evidence['current_state'] != 'regular_file':
        return result
    recorded = record['payload']['artifacts']['diagnosis_json']
    payload = _read_json(Path(recorded)) if recorded is not None else None
    allowed_statuses = {
        'success',
        'map_saved',
        'verify_failed',
        'runtime_failed',
        'incomplete',
    }
    if payload is None or payload.get('status') not in allowed_statuses:
        result['evidence_status'] = 'invalid_contract'
        return result
    symptom = payload.get('symptom_triage')
    reported_symptom = None
    if symptom is not None:
        if not (
            isinstance(symptom, dict)
            and symptom.get('symptom') in REPORTED_SYMPTOM_CODES
            and symptom.get('code') == symptom.get('symptom')
            and symptom.get('basis') == REPORTED_SYMPTOM_BASIS
        ):
            result['evidence_status'] = 'invalid_contract'
            return result
        reported_symptom = {
            'code': symptom['code'],
            'basis': symptom['basis'],
        }
    verify = payload.get('verify')
    hints = payload.get('problem_hints')
    steps = payload.get('suggested_next_steps')
    result.update({
        'status': payload['status'],
        'verify_result': (
            _safe_identifier(verify.get('result'))
            if isinstance(verify, dict) and verify.get('result') is not None
            else None
        ),
        'projector_type': _safe_optional(payload.get('projector_type')),
        'problem_hint_count': len(hints) if isinstance(hints, list) else 0,
        'suggested_step_count': len(steps) if isinstance(steps, list) else 0,
        'reported_symptom': reported_symptom,
    })
    return result


def _assert_private_values_absent(
    rendered: str,
    private_values: set[str],
) -> None:
    leaked = sorted(
        value for value in private_values
        if value in rendered
    )
    if leaked:
        raise RuntimeError(
            'privacy audit rejected a local path, command, or credential leak'
        )


def build_support_report(session_bundle: str) -> dict[str, Any]:
    """Build and validate a privacy-bounded session support report."""
    comparison = _load_script_module(
        'session_compare.py',
        'support_bundle_session_evidence',
    )
    product_schema = _load_script_module(
        'product_schema.py',
        'support_bundle_product_schema',
    )
    record = comparison._load_session_bundle(session_bundle, product_schema)
    setup_source = comparison._load_setup_source(record, product_schema)
    private_values = _known_private_values(record, setup_source)
    artifacts = _artifact_projections(record)
    report = {
        'schema_version': 1,
        'schema_uri': SUPPORT_SCHEMA_URI,
        'created_at': _utc_now(),
        'privacy': {
            'contains_map_geometry': False,
            'contains_raw_sensor_data': False,
            'contains_raw_logs': False,
            'contains_parameter_contents': False,
            'local_paths_redacted': True,
            'command_secrets_redacted': True,
            'review_before_sharing': True,
        },
        'product': _product_build_info(),
        'session': _session_projection(record),
        'setup': _setup_projection(
            record,
            setup_source,
            private_values,
        ),
        'diagnosis': _diagnosis_projection(record, artifacts),
        'artifacts': artifacts,
    }
    product_schema.validate_contract(report, SUPPORT_SCHEMA)
    rendered = json.dumps(report, ensure_ascii=False, sort_keys=True)
    _assert_private_values_absent(rendered, private_values)
    return report


def _build_handoff_from_receipt(
    receipt_path: Path,
    product_schema,
) -> dict[str, Any]:
    """Build a first-map handoff from one receipt-bound output directory."""
    requested = receipt_path.expanduser()
    if _path_has_symlink(requested):
        raise ValueError(
            'validation receipt is not a regular, non-symlink file'
        )
    receipt_path = requested.resolve()
    if receipt_path.name != FIRST_MAP_RECEIPT_NAME:
        raise ValueError('output points to an unexpected validation receipt')
    if any(ord(character) < 32 for character in str(receipt_path)):
        raise ValueError('validation receipt path contains control characters')

    wizard = _load_script_module(
        'sensor_setup_wizard.py',
        'first_map_handoff_receipt_validator',
    )
    receipt, checks, source_status, validated_path = (
        wizard._validated_validation_receipt(receipt_path.parent)
    )
    if (
        source_status != 'valid'
        or receipt is None
        or validated_path.resolve() != receipt_path
    ):
        raise ValueError(
            'validation receipt no longer matches the retained run evidence'
        )
    if receipt['status'] != 'PASS' or not all(
        item['passed'] for item in checks.values()
    ):
        raise ValueError('first-map validation receipt is not PASS')

    for evidence in receipt['evidence'].values():
        evidence_path = receipt_path.parent / evidence['filename']
        if _path_has_symlink(evidence_path):
            raise ValueError(
                'receipt-bound evidence contains a symlink: '
                f'{evidence_path.name}'
            )

    receipt_module = _load_script_module(
        'first_map_validation_receipt.py',
        'first_map_handoff_renderer',
    )
    markdown_path = receipt_path.parent / FIRST_MAP_MARKDOWN_NAME
    markdown_available = (
        not _path_has_symlink(markdown_path)
        and markdown_path.is_file()
        and markdown_path.stat().st_size <= MAX_JSON_BYTES
    )
    verification = receipt['verification']
    run = receipt['run']
    product = _product_build_info()
    product_version = _safe_identifier(run['product_version'])
    git_commit = (
        _safe_identifier(run['git_commit'])
        if run['git_commit'] is not None else 'unknown'
    )
    release_ref = (
        git_commit if git_commit != 'unknown' else product_version
    )
    handoff = {
        'schema_version': 1,
        'schema_uri': FIRST_MAP_HANDOFF_SCHEMA_URI,
        'status': 'READY_FOR_REVIEW',
        'receipt_status': 'PASS',
        'receipt_path': str(receipt_path),
        'markdown_path': str(markdown_path) if markdown_available else None,
        'issue_url': receipt_module.VALIDATION_ISSUE_URL,
        'run': {
            'product_version': product_version,
            'git_commit': git_commit,
            'profile_id': _safe_identifier(run['profile_id']),
        },
        'environment_hints': {
            'os_family': product['platform']['system'],
            'architecture': product['platform']['machine'],
            'ros_distro': product['ros_distro'],
        },
        'form_fields': {
            'result': 'PASS — verified first map completed',
            'release_ref': release_ref,
        },
        'verification_summary': '\n'.join([
            'manifest_status='
            + _safe_identifier(verification['manifest_status']),
            'diagnosis_status='
            + _safe_identifier(verification['diagnosis_status']),
            'autoware_status='
            + _safe_identifier(verification['autoware_status']),
            'manifest_sha256=' + verification['manifest_sha256'],
        ]),
        'privacy': {
            'contains_map_geometry': False,
            'contains_private_paths': True,
            'contains_exact_command': False,
            'review_before_sharing': True,
        },
        'operator_supplied_fields': [
            'documentation_path',
            'environment_details',
            'exact_command',
            'findings',
        ],
    }
    product_schema.validate_contract(handoff, FIRST_MAP_HANDOFF_SCHEMA)
    return handoff


def _build_receipt_only_handoff(
    output_dir: str,
    product_schema,
) -> dict[str, Any]:
    """Build a handoff for fixed Docker/source demo outputs without sessions."""
    requested = Path(output_dir).expanduser()
    if requested.is_symlink():
        raise ValueError(f'output directory may not be a symlink: {requested}')
    if _path_has_symlink(requested):
        raise ValueError(
            f'output directory contains a symlinked path: {requested}'
        )
    bundle = requested.resolve()
    if not bundle.is_dir():
        raise ValueError(f'output directory is not a directory: {bundle}')
    return _build_handoff_from_receipt(
        bundle / FIRST_MAP_RECEIPT_NAME,
        product_schema,
    )


def build_first_map_handoff(session_bundle: str) -> dict[str, Any]:
    """Validate one PASS session or fixed-demo output handoff."""
    comparison = _load_script_module(
        'session_compare.py',
        'first_map_handoff_session_evidence',
    )
    product_schema = _load_script_module(
        'product_schema.py',
        'first_map_handoff_product_schema',
    )
    requested = Path(session_bundle).expanduser()
    if requested.is_symlink():
        raise ValueError(f'output directory may not be a symlink: {requested}')
    bundle = requested.resolve()
    if not (bundle / 'session.json').exists() and not (
        bundle / 'session.json'
    ).is_symlink():
        return _build_receipt_only_handoff(session_bundle, product_schema)
    record = comparison._load_session_bundle(session_bundle, product_schema)
    session = record['payload']
    if not (
        session['status'] == 'verified'
        and session['runner_exit_code'] == 0
        and session['verification'] == {'mode': 'required', 'result': 'PASS'}
        and session['quality']['overall'] == 'pass'
        and session['quality']['source']['status'] == 'valid'
    ):
        raise ValueError(
            'first-map handoff requires a verified session with PASS '
            'receipt-bound quality evidence'
        )

    artifacts = {
        item['name']: item for item in _artifact_projections(record)
    }
    receipt_artifact = artifacts['validation_receipt']
    if receipt_artifact['current_state'] != 'regular_file':
        raise ValueError(
            'validation receipt is not a regular in-session file: '
            f"{receipt_artifact['current_state']}"
        )
    recorded_receipt = session['artifacts']['validation_receipt']
    if not isinstance(recorded_receipt, str):
        raise ValueError('session has no validation receipt path')
    return _build_handoff_from_receipt(
        Path(recorded_receipt).expanduser(),
        product_schema,
    )


def render_first_map_handoff(handoff: dict[str, Any]) -> str:
    """Render one concise, copy-ready independent-validation handoff."""
    form_fields = handoff['form_fields']
    environment = ' / '.join([
        handoff['environment_hints']['os_family'],
        handoff['environment_hints']['architecture'],
        'ROS 2 ' + (handoff['environment_hints']['ros_distro'] or 'unknown'),
        '<install method>',
        '<relevant hardware>',
    ])
    verification_lines = [
        f'    {line}' for line in handoff['verification_summary'].splitlines()
    ]
    lines = [
        'First-map report: READY FOR REVIEW',
        f"  Open issue:          {handoff['issue_url']}",
        f"  Attach after review: {handoff['receipt_path']}",
        '',
        'Copy into the issue form:',
        f"  Result: {form_fields['result']}",
        '  Release, commit, or image digest: '
        f"{form_fields['release_ref']}",
        '    Replace it with the immutable image digest if that is what '
        'you ran.',
        f'  Environment: {environment}',
        '  Verification summary:',
        *verification_lines,
    ]
    lines.extend([
        '',
        'Complete before submitting:',
        '  Public documentation path: <Docker First Map | Source quickstart '
        '| Own-bag golden path>',
        '  Redacted command shape: <executable and options; replace every '
        'private value with REDACTED>',
        '  Findings: <what was unclear, slow, surprising, broken, or helpful>',
        '',
        'Privacy: use REDACTED for credentials, private paths, host/user names, '
        'and precise locations.',
        'Attach only the reviewed JSON receipt; never attach the map, bag, '
        'manifest, logs, trajectory, parameters, or private-place screenshots.',
    ])
    return '\n'.join(lines)


def _issue_body(report: dict[str, Any]) -> str:
    session = report['session']
    product = report['product']
    setup = report['setup']
    diagnosis = report['diagnosis']
    command = (
        setup['run_command']['redacted']
        if setup['run_command'] is not None else
        'Unavailable because setup evidence is missing or invalid.'
    )
    reasons = ', '.join(session['reason_codes']) or 'none'
    diagnosis_status = diagnosis['status'] or diagnosis['evidence_status']
    verify_result = diagnosis['verify_result'] or 'unknown'
    reported_symptom = diagnosis['reported_symptom']
    symptom_lines = ['- Reported visual symptom: `not recorded`']
    if reported_symptom is not None:
        symptom_lines = [
            '- Reported visual symptom: '
            f'`{reported_symptom["code"]}`',
            '- Symptom evidence boundary: user-reported; not an '
            'automatically diagnosed root cause',
        ]
    return '\n'.join([
        '# lidarslam support report',
        '',
        '<!-- Review this text and the ZIP before sharing publicly. -->',
        '',
        '## Environment',
        '',
        f'- Version: `{product["version"] or "unknown"}`',
        f'- Source revision: `{product["source_revision"] or "unknown"}`',
        f'- Source dirty: `{product["source_dirty"]}`',
        f'- ROS distribution: `{product["ros_distro"] or "unknown"}`',
        (
            f'- Platform: `{product["platform"]["system"]} '
            f'{product["platform"]["release"]} '
            f'{product["platform"]["machine"]}`'
        ),
        '',
        '## Session evidence',
        '',
        f'- Status: `{session["status"]}`',
        f'- Profile: `{session["profile_id"]}`',
        f'- Verification: `{session["verification"]["result"]}`',
        f'- Quality: `{session["quality"]["overall"]}`',
        f'- Setup evidence: `{setup["status"]}`',
        f'- Diagnosis: `{diagnosis_status}`',
        f'- Verifier diagnosis: `{verify_result}`',
        *symptom_lines,
        f'- Reason codes: `{reasons}`',
        f'- Session evidence SHA-256: `{session["evidence_sha256"]}`',
        '',
        '## Redacted command',
        '',
        '```bash',
        command,
        '```',
        '',
        '## Expected behavior',
        '',
        '<!-- Describe what should have happened. -->',
        '',
        '## Observed behavior',
        '',
        '<!-- Describe what happened. Add separately reviewed logs only when '
        'needed. -->',
        '',
    ])


def _readme() -> str:
    return '\n'.join([
        ARCHIVE_MARKER,
        '',
        'This ZIP was generated by lidarslam-map support.',
        '',
        'It intentionally excludes map geometry, raw sensor data, raw logs,',
        'parameter-file contents, exact local paths, and command credentials.',
        'support-report.json is the machine-readable evidence summary.',
        'issue-body.md is a copy-ready starting point for a GitHub issue.',
        '',
        'Review every member before sharing. Add separately reviewed logs',
        'or a licensed minimal reproduction only when requested.',
        'Do not publish security vulnerabilities; follow SECURITY.md instead.',
        '',
    ])


def support_members(report: dict[str, Any]) -> dict[str, str]:
    """Return the fixed, reviewable archive members."""
    return {
        'README.txt': _readme(),
        'issue-body.md': _issue_body(report),
        'support-report.json': (
            json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True)
            + '\n'
        ),
    }


def _zip_info(name: str, created_at: str) -> zipfile.ZipInfo:
    parsed = datetime.fromisoformat(created_at.replace('Z', '+00:00'))
    info = zipfile.ZipInfo(
        name,
        date_time=(
            parsed.year,
            parsed.month,
            parsed.day,
            parsed.hour,
            parsed.minute,
            parsed.second,
        ),
    )
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o100644 << 16
    return info


def write_support_zip(path: Path, report: dict[str, Any]) -> Path:
    """Atomically create a new support ZIP without replacing any path."""
    requested = path.expanduser()
    if requested.is_symlink():
        raise OSError(f'refusing to replace symlink: {requested}')
    path = requested.resolve()
    if path.exists():
        raise OSError(f'refusing to replace existing path: {path}')
    if path.suffix.lower() != '.zip':
        raise OSError('support output must use the .zip suffix')
    if not path.parent.is_dir():
        raise OSError(f'support output parent does not exist: {path.parent}')
    temporary = path.parent / f'.{path.name}.{os.getpid()}.tmp'
    members = support_members(report)
    try:
        with zipfile.ZipFile(temporary, 'x') as archive:
            for name in sorted(members):
                archive.writestr(
                    _zip_info(name, report['created_at']),
                    members[name].encode('utf-8'),
                )
        os.link(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return path


def _default_output(bundle: Path, report: dict[str, Any]) -> Path:
    stamp = re.sub(r'[^0-9]', '', report['created_at'])[:14]
    status = report['session']['status'].replace('_', '-')
    return bundle.parent / f'lidarslam-support-{status}-{stamp}Z.zip'


def main(argv: Sequence[str] | None = None) -> int:
    """Create one local support attachment or print its report."""
    args = parse_args(argv)
    if args.first_map:
        if args.output is not None:
            print(
                'error: [invalid-usage] --first-map cannot be combined with '
                '--output',
                file=sys.stderr,
            )
            return 2
        try:
            handoff = build_first_map_handoff(args.session)
        except (OSError, RuntimeError, ValueError) as exc:
            print(f'error: [first-map-handoff-invalid] {exc}', file=sys.stderr)
            return 2
        if args.json:
            print(json.dumps(
                handoff,
                indent=2,
                ensure_ascii=False,
                sort_keys=True,
            ))
        else:
            print(render_first_map_handoff(handoff))
        return 0
    try:
        report = build_support_report(args.session)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'error: [support-bundle-invalid] {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False, sort_keys=True))
        return 0
    bundle = Path(args.session).expanduser().resolve()
    output = Path(args.output) if args.output else _default_output(
        bundle,
        report,
    )
    try:
        archive = write_support_zip(output, report)
    except OSError as exc:
        print(f'error: [support-write-failed] {exc}', file=sys.stderr)
        return 2
    print(f'Support bundle: {archive}')
    print(
        'Privacy: no map, bag, raw log, or parameter contents were included.'
    )
    print(
        'Review all three ZIP members before attaching it to a public issue.'
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
