#!/usr/bin/env python3
"""Create an immutable, verified rollback plan from release image evidence."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import sys
from typing import Any

from product_schema import load_json_object, validate_contract


RELEASE_IMAGE_SCHEMA = 'release-image-v1.schema.json'
ROLLBACK_PLAN_SCHEMA = 'rollback-plan-v1.schema.json'
TAG_PATTERN = re.compile(
    r'^ghcr\.io/(?P<repository>[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+):'
    r'v(?P<version>[0-9]+\.[0-9]+\.[0-9]+)-'
    r'(?P<ros_distro>humble|jazzy)$'
)


def _command(*values: str) -> str:
    return shlex.join(values)


def build_rollback_plan(
    record: dict[str, Any],
    *,
    source_record: Path,
) -> dict[str, Any]:
    """Validate release evidence and return only immutable commands."""
    validate_contract(record, RELEASE_IMAGE_SCHEMA)
    match = TAG_PATTERN.fullmatch(record['tag'])
    if match is None:
        raise ValueError(
            'release image tag must be '
            'ghcr.io/<owner>/<repo>:v<major>.<minor>.<patch>-<distro>'
        )
    if match.group('version') != record['product_version']:
        raise ValueError('release image tag version does not match product_version')
    if match.group('ros_distro') != record['ros_distro']:
        raise ValueError('release image tag distro does not match ros_distro')
    expected_cli = f"lidarslam_ros2 {record['product_version']}"
    if record['cli_version'] != expected_cli:
        raise ValueError(
            f"release image cli_version must be {expected_cli!r}"
        )

    repository = match.group('repository')
    image_name = f'ghcr.io/{repository}'
    immutable_ref = f"{image_name}@{record['digest']}"
    plan = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/rollback-plan-v1.schema.json'
        ),
        'status': 'PASS',
        'source_record': source_record.name,
        'repository': repository,
        'ros_distro': record['ros_distro'],
        'platform': record['platform'],
        'product_version': record['product_version'],
        'git_commit': record['git_commit'],
        'tag': record['tag'],
        'digest': record['digest'],
        'immutable_ref': immutable_ref,
        'moving_tag_mutated': False,
        'commands': {
            'pull': _command('docker', 'pull', immutable_ref),
            'attestation': _command(
                'gh',
                'attestation',
                'verify',
                f'oci://{immutable_ref}',
                '-R',
                repository,
            ),
            'cli_smoke': _command(
                'docker',
                'run',
                '--rm',
                immutable_ref,
                'lidarslam-map',
                '--version',
            ),
        },
    }
    validate_contract(plan, ROLLBACK_PLAN_SCHEMA)
    return plan


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Validate a release-image evidence file and print an immutable '
            'digest rollback plan. This command never moves registry tags.'
        ),
    )
    parser.add_argument(
        'record',
        metavar='release_image_record',
        help='Downloaded release-image-humble.json or release-image-jazzy.json.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all rollback planning options.',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print the versioned rollback plan JSON.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    record_path = Path(args.record).expanduser().resolve()
    try:
        record = load_json_object(record_path, 'release image record')
        plan = build_rollback_plan(record, source_record=record_path)
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(plan, indent=2, sort_keys=True))
    else:
        print('Last-known-good rollback plan: PASS')
        print(f"- image: {plan['immutable_ref']}")
        print(f"- commit: {plan['git_commit']}")
        print('- moving tag mutated: no')
        print('Verify and smoke-test:')
        print(f"  {plan['commands']['pull']}")
        print(f"  {plan['commands']['attestation']}")
        print(f"  {plan['commands']['cli_smoke']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
