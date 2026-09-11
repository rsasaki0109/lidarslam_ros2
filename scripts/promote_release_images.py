#!/usr/bin/env python3
"""Safely create immutable version tags from tested release image digests."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable, Sequence

from create_release_image_record import TAG_PATTERN

from product_schema import load_json_object, validate_contract


RELEASE_IMAGE_SCHEMA = 'release-image-v1.schema.json'
PROMOTION_SCHEMA = 'release-promotion-v1.schema.json'
Runner = Callable[..., subprocess.CompletedProcess[str]]


def _inspect_digest(reference: str, *, runner: Runner) -> str | None:
    completed = runner(
        [
            'docker',
            'buildx',
            'imagetools',
            'inspect',
            reference,
            '--format',
            '{{json .Manifest}}',
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        return None
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f'docker returned invalid manifest JSON for {reference}: {exc}'
        ) from exc
    digest = payload.get('digest') if isinstance(payload, dict) else None
    if not isinstance(digest, str) or not re.fullmatch(
        r'sha256:[0-9a-f]{64}',
        digest,
    ):
        raise ValueError(f'docker returned an invalid digest for {reference}')
    return digest


def _validate_record_set(
    records: Sequence[dict[str, Any]],
    *,
    expected_repository: str,
) -> None:
    if len(records) != 2:
        raise ValueError(
            'release promotion requires exactly two image records'
        )
    for record in records:
        validate_contract(record, RELEASE_IMAGE_SCHEMA)
    by_distro = {record['ros_distro']: record for record in records}
    if set(by_distro) != {'humble', 'jazzy'}:
        raise ValueError('release promotion requires Humble and Jazzy records')
    if len(by_distro) != len(records):
        raise ValueError(
            'release image records contain a duplicate ROS distro'
        )

    for field in ('git_commit', 'platform', 'product_version'):
        if len({record[field] for record in records}) != 1:
            raise ValueError(f'release image records disagree on {field}')
    if len({record['digest'] for record in records}) != 2:
        raise ValueError('Humble and Jazzy records must use distinct digests')

    for record in records:
        match = TAG_PATTERN.fullmatch(record['tag'])
        if match is None:
            raise ValueError('release image record contains an invalid tag')
        if match.group('repository') != expected_repository:
            raise ValueError(
                'release image record repository does not match '
                f'{expected_repository}'
            )
        if match.group('version') != record['product_version']:
            raise ValueError('release image tag version does not match record')
        if match.group('ros_distro') != record['ros_distro']:
            raise ValueError('release image tag distro does not match record')
        if record['cli_version'] != (
            f"lidarslam_ros2 {record['product_version']}"
        ):
            raise ValueError('release image CLI version does not match record')


def promote_release_images(
    records: Sequence[dict[str, Any]],
    *,
    expected_repository: str,
    apply: bool,
    runner: Runner = subprocess.run,
) -> dict[str, Any]:
    """Preflight every digest, then create or reuse version tags safely."""
    _validate_record_set(records, expected_repository=expected_repository)
    ordered = sorted(records, key=lambda record: record['ros_distro'])
    preflight: list[dict[str, Any]] = []
    for record in ordered:
        repository_ref = record['tag'].split(':', 1)[0]
        immutable_ref = f"{repository_ref}@{record['digest']}"
        source_digest = _inspect_digest(immutable_ref, runner=runner)
        if source_digest != record['digest']:
            raise ValueError(
                f'candidate digest is unavailable: {immutable_ref}'
            )
        current_digest = _inspect_digest(record['tag'], runner=runner)
        if current_digest is not None and current_digest != record['digest']:
            raise ValueError(
                f"refusing to move {record['tag']}: "
                f'{current_digest} != {record["digest"]}'
            )
        preflight.append({
            'ros_distro': record['ros_distro'],
            'tag': record['tag'],
            'digest': record['digest'],
            'immutable_ref': immutable_ref,
            'action': 'reuse' if current_digest else 'create',
        })

    created: list[str] = []
    reused: list[str] = []
    if apply:
        for item in preflight:
            if item['action'] == 'reuse':
                reused.append(item['tag'])
                continue
            completed = runner(
                [
                    'docker',
                    'buildx',
                    'imagetools',
                    'create',
                    '--tag',
                    item['tag'],
                    item['immutable_ref'],
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            if completed.returncode != 0:
                raise ValueError(
                    f"failed to create {item['tag']}: "
                    f'{completed.stderr.strip()}'
                )
            observed = _inspect_digest(item['tag'], runner=runner)
            if observed != item['digest']:
                raise ValueError(
                    f"created tag {item['tag']} resolved to {observed}, "
                    f"expected {item['digest']}"
                )
            created.append(item['tag'])

    report = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/release-promotion-v1.schema.json'
        ),
        'status': 'PASS',
        'mode': 'applied' if apply else 'dry_run',
        'repository': expected_repository,
        'product_version': ordered[0]['product_version'],
        'git_commit': ordered[0]['git_commit'],
        'moving_tag_mutated': False,
        'images': preflight,
        'created_tags': created,
        'reused_tags': reused,
    }
    validate_contract(report, PROMOTION_SCHEMA)
    return report


def parse_args() -> argparse.Namespace:
    """Parse release-image promotion command-line arguments."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_RELEASE_COMMAND'),
        description=(
            'Validate Humble/Jazzy release records and safely promote their '
            'tested digests. Dry-run is the default.'
        ),
    )
    parser.add_argument('records', nargs=2)
    parser.add_argument('--repository', required=True)
    parser.add_argument(
        '--apply',
        action='store_true',
        help=(
            'Create missing version tags after every record passes '
            'preflight.'
        ),
    )
    parser.add_argument('--json', action='store_true')
    return parser.parse_args()


def main() -> int:
    """Validate records and optionally apply an immutable promotion."""
    args = parse_args()
    try:
        records = [
            load_json_object(
                Path(path).expanduser().resolve(),
                'release image record',
            )
            for path in args.records
        ]
        report = promote_release_images(
            records,
            expected_repository=args.repository,
            apply=args.apply,
        )
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(f"Release image promotion: {report['status']}")
        print(f"- mode: {report['mode']}")
        print('- moving tag mutated: no')
        for image in report['images']:
            print(
                f"- {image['ros_distro']}: {image['action']} "
                f"{image['tag']} -> {image['digest']}"
            )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
