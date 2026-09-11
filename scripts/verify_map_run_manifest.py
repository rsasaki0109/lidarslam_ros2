#!/usr/bin/env python3
"""Reopen and verify a schema-v2 operational map-run manifest."""

from __future__ import annotations

import argparse
import json
import stat
import sys
from pathlib import Path
from typing import Any

import map_run_manifest as manifest_tools


MAX_MANIFEST_BYTES = 16 * 1024 * 1024


class VerificationError(RuntimeError):
    pass


def _regular_file(path: Path, label: str) -> bytes:
    try:
        info = path.lstat()
    except OSError as error:
        raise VerificationError(f'{label} is missing: {path}') from error
    if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise VerificationError(f'{label} must be a single-link regular file: {path}')
    if info.st_size <= 0 or info.st_size > MAX_MANIFEST_BYTES:
        raise VerificationError(f'{label} has invalid size: {info.st_size}')
    return path.read_bytes()


def _expand_command(
    command: list[str], repo_root: Path, bag_dir: Path, output_dir: Path,
) -> list[str]:
    replacements = {
        '${REPO_ROOT}': str(repo_root),
        '${BAG_DIR}': str(bag_dir),
        '${OUTPUT_DIR}': str(output_dir),
    }
    expanded = []
    for argument in command:
        if not isinstance(argument, str) or '\x00' in argument:
            raise VerificationError('workflow command contains an invalid argument')
        value = argument
        for marker, replacement in replacements.items():
            value = value.replace(marker, replacement)
        expanded.append(value)
    return expanded


def verify_manifest(
    *, output_dir: Path, bag_dir: Path, repo_root: Path,
) -> dict[str, Any]:
    output_dir = output_dir.resolve()
    bag_dir = bag_dir.resolve()
    repo_root = repo_root.resolve()
    manifest_path = output_dir / 'map_run_manifest.json'
    try:
        document = json.loads(_regular_file(manifest_path, 'manifest').decode('utf-8'))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise VerificationError(f'manifest is not valid UTF-8 JSON: {error}') from error
    required = {
        'schema_version', 'created_at_utc', 'status', 'workflow', 'source',
        'environment', 'input', 'configs', 'outputs',
    }
    if not isinstance(document, dict) or set(document) != required:
        raise VerificationError('manifest fields are not exact')
    if document.get('schema_version') != 2:
        raise VerificationError('only map-run manifest schema v2 is supported')
    workflow = document.get('workflow')
    if not isinstance(workflow, dict) or set(workflow) != {'profile_id', 'command'}:
        raise VerificationError('workflow binding is invalid')
    command = workflow.get('command')
    if not isinstance(command, list) or not command:
        raise VerificationError('workflow command is invalid')

    checks: dict[str, dict[str, Any]] = {}
    current_input = manifest_tools._input_identity(bag_dir)
    checks['input'] = {
        'status': 'PASS' if document['input'] == current_input else 'MISMATCH',
        'expected_tree_sha256': document.get('input', {}).get('tree_sha256')
            if isinstance(document.get('input'), dict) else None,
        'actual_tree_sha256': current_input['tree_sha256'],
    }
    expanded_command = _expand_command(command, repo_root, bag_dir, output_dir)
    current_configs = manifest_tools._config_identities(expanded_command, repo_root)
    checks['configs'] = {
        'status': 'PASS' if document['configs'] == current_configs else 'MISMATCH',
        'expected_count': len(document['configs']) if isinstance(document['configs'], list) else None,
        'actual_count': len(current_configs),
    }
    current_source = manifest_tools._git_state(repo_root)
    checks['source'] = {
        'status': 'PASS' if document['source'] == current_source else 'MISMATCH',
        'expected': document['source'],
        'actual': current_source,
    }
    current_outputs = manifest_tools._output_inventory(output_dir)
    checks['outputs'] = {
        'status': 'PASS' if document['outputs'] == current_outputs else 'MISMATCH',
        'expected_count': len(document['outputs']) if isinstance(document['outputs'], list) else None,
        'actual_count': len(current_outputs),
    }
    status = 'PASS' if all(item['status'] == 'PASS' for item in checks.values()) else 'FAIL_CLOSED'
    return {
        'schema': 'lidarslam-map-run-manifest-verification-v1',
        'status': status,
        'manifest_schema_version': 2,
        'profile_id': workflow['profile_id'],
        'checks': checks,
        'safety': {'read_only': True, 'network_used': False, 'slam_started': False},
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_dir', type=Path)
    parser.add_argument('--bag-dir', type=Path, required=True)
    parser.add_argument(
        '--repo-root', type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    args = parser.parse_args(argv)
    try:
        result = verify_manifest(
            output_dir=args.output_dir,
            bag_dir=args.bag_dir,
            repo_root=args.repo_root,
        )
    except (VerificationError, ValueError, OSError) as error:
        result = {
            'schema': 'lidarslam-map-run-manifest-verification-v1',
            'status': 'FAIL_CLOSED',
            'error': str(error),
            'safety': {'read_only': True, 'network_used': False, 'slam_started': False},
        }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['status'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
