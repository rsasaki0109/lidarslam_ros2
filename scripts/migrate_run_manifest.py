#!/usr/bin/env python3
"""Migrate a terminal run-manifest v1 into inspect-only schema v2."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
from typing import Any

from product_schema import load_json_object, validate_contract


V1_SCHEMA = 'run-manifest-v1.schema.json'
V2_SCHEMA = 'run-manifest-v2.schema.json'
V2_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/run-manifest-v2.schema.json'
)
MANIFEST_NAME = 'run_manifest.json'
TERMINAL_STATUSES = {'succeeded', 'failed', 'interrupted'}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def migrate_manifest(
    source: dict[str, Any],
    *,
    verification_enabled: bool,
) -> dict[str, Any]:
    """Return a schema-v2 historical manifest that can never be resumed."""
    validate_contract(source, V1_SCHEMA)
    status = source['status']
    execution = source['execution']
    if status not in TERMINAL_STATUSES:
        raise ValueError(
            'only terminal schema-v1 manifests can be migrated; '
            f'found status {status!r}'
        )
    if execution['finished_at'] is None or execution['exit_code'] is None:
        raise ValueError(
            'terminal migration requires execution.finished_at and '
            'execution.exit_code'
        )

    migrated = dict(source)
    migrated['schema_version'] = 2
    migrated['schema_uri'] = V2_SCHEMA_URI
    migrated['lifecycle'] = {
        'stage': 'complete',
        'resume_count': 0,
        'verification_enabled': verification_enabled,
        'runner_exit_code': execution['exit_code'],
        'last_error': (
            None
            if status == 'succeeded'
            else (
                'Historical schema-v1 terminal state migrated for inspection '
                'only; original lifecycle detail is unavailable.'
            )
        ),
    }
    validate_contract(migrated, V2_SCHEMA)
    return migrated


def _write_json_exclusive_atomic(
    destination: Path,
    payload: dict[str, Any],
) -> None:
    if destination.exists() or destination.is_symlink():
        raise FileExistsError(
            f'migration output already exists; refusing overwrite: {destination}'
        )
    if not destination.parent.is_dir():
        raise FileNotFoundError(
            f'migration output parent does not exist: {destination.parent}'
        )
    temporary = destination.with_name(f'.{destination.name}.tmp')
    if temporary.exists() or temporary.is_symlink():
        raise FileExistsError(
            f'migration temporary path already exists: {temporary}'
        )
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        # A same-directory hard link publishes atomically and fails if another
        # process created the destination after the preflight check.
        os.link(temporary, destination)
        temporary.unlink()
        directory_fd = os.open(destination.parent, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
    except Exception:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


def migrate_file(
    run_dir: Path,
    destination: Path,
    *,
    verification_enabled: bool,
) -> dict[str, Any]:
    """Validate, migrate, and atomically write one historical manifest."""
    run_dir = run_dir.expanduser().resolve()
    destination = destination.expanduser().absolute()
    if not run_dir.is_dir():
        raise FileNotFoundError(
            f'run directory does not exist or is not a directory: {run_dir}'
        )
    source_path = run_dir / MANIFEST_NAME
    if not source_path.is_file():
        raise FileNotFoundError(f'run manifest not found: {source_path}')
    if destination.resolve(strict=False) == source_path.resolve():
        raise ValueError(
            'migration output must not replace the source run_manifest.json'
        )

    source = load_json_object(source_path, 'run manifest')
    migrated = migrate_manifest(
        source,
        verification_enabled=verification_enabled,
    )
    source_sha256 = _sha256(source_path)
    _write_json_exclusive_atomic(destination, migrated)
    return {
        'status': 'PASS',
        'source': str(source_path),
        'source_sha256': source_sha256,
        'destination': str(destination),
        'destination_sha256': _sha256(destination),
        'source_schema_version': 1,
        'destination_schema_version': 2,
        'lifecycle_stage': 'complete',
        'resume_allowed': False,
        'verification_enabled': verification_enabled,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Convert a terminal run-manifest v1 into a separate, '
            'inspect-only schema-v2 file without changing the run directory.'
        ),
    )
    parser.add_argument(
        'run_dir',
        metavar='output_dir',
        help='Historical map output directory containing run_manifest.json.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all migration options.',
    )
    parser.add_argument(
        '--output',
        required=True,
        metavar='<file>',
        help='New JSON file; it and its temporary sibling must not exist.',
    )
    parser.add_argument(
        '--verification',
        required=True,
        choices=('required', 'off'),
        metavar='{required,off}',
        help='Explicit historical verification mode; it is never inferred.',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print the migration record as JSON.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        result = migrate_file(
            Path(args.run_dir),
            Path(args.output),
            verification_enabled=args.verification == 'required',
        )
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print('Run manifest migration: PASS')
        print(f"- source: {result['source']}")
        print(f"- destination: {result['destination']}")
        print('- lifecycle: complete (inspection only)')
        print('- resume allowed: no')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
