#!/usr/bin/env python3
"""Create immutable, tag-free candidate-image evidence."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import sys
from typing import Any, Sequence

from product_schema import load_json_object, validate_contract


REQUEST_SCHEMA = 'candidate-image-request-v1.schema.json'
IMAGE_SCHEMA = 'candidate-image-v1.schema.json'
IMAGE_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-image-v1.schema.json'
)
DIGEST_RE = re.compile(r'^sha256:[0-9a-f]{64}$')
RUN_URL_RE = re.compile(
    r'^https://github\.com/([A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+)/actions/'
    r'runs/[1-9][0-9]*$'
)


def build_candidate_image_record(
    request: dict[str, Any],
    *,
    ros_distro: str,
    platform: str,
    digest: str,
    cli_version: str,
    workflow_run_url: str,
    evidence_retention_days: int,
) -> dict[str, Any]:
    """Build one internally consistent digest-only image record."""
    validate_contract(request, REQUEST_SCHEMA)
    if request['status'] != 'AUTHORIZED':
        raise ValueError('candidate request is not authorized')
    if ros_distro not in {'humble', 'jazzy'}:
        raise ValueError('ros_distro must be humble or jazzy')
    if platform != 'linux/amd64':
        raise ValueError('candidate platform must be linux/amd64')
    if DIGEST_RE.fullmatch(digest) is None:
        raise ValueError('candidate digest is malformed')
    expected_cli = f"lidarslam_ros2 {request['product_version']}"
    if cli_version != expected_cli:
        raise ValueError('candidate CLI version does not match the request')
    run_match = RUN_URL_RE.fullmatch(workflow_run_url)
    if run_match is None:
        raise ValueError('workflow run URL is malformed')
    if run_match.group(1) != request['repository']:
        raise ValueError('workflow run repository does not match the request')
    if (
        not isinstance(evidence_retention_days, int)
        or isinstance(evidence_retention_days, bool)
        or not 1 <= evidence_retention_days <= 90
    ):
        raise ValueError('evidence retention must be between 1 and 90 days')

    repository_ref = f"ghcr.io/{request['repository']}"
    record = {
        'schema_version': 1,
        'schema_uri': IMAGE_SCHEMA_URI,
        'status': 'PASS',
        'publication_mode': 'digest_only',
        'repository': request['repository'],
        'source_pr': request['source_pr'],
        'source_commit': request['source_commit'],
        'product_version': request['product_version'],
        'ros_distro': ros_distro,
        'platform': platform,
        'digest': digest,
        'immutable_ref': f'{repository_ref}@{digest}',
        'cli_version': cli_version,
        'workflow_run_url': workflow_run_url,
        'workflow_branch_ref': request['workflow_branch_ref'],
        'workflow_gate_commit': request['workflow_gate_commit'],
        'requested_by': request['requested_by'],
        'tags_created': [],
        'moving_tags_mutated': False,
        'release_mutated': False,
        'registry_retention_status': 'REQUIRES_REMOTE_AUDIT',
        'evidence_retention_days': evidence_retention_days,
    }
    validate_contract(record, IMAGE_SCHEMA)
    return record


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_RELEASE_COMMAND'),
        description=__doc__,
    )
    parser.add_argument('--request', required=True, type=Path)
    parser.add_argument(
        '--ros-distro',
        required=True,
        choices=('humble', 'jazzy'),
    )
    parser.add_argument('--platform', default='linux/amd64')
    parser.add_argument('--digest', required=True)
    parser.add_argument('--cli-version', required=True)
    parser.add_argument('--workflow-run-url', required=True)
    parser.add_argument(
        '--evidence-retention-days',
        type=int,
        default=30,
    )
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Create one candidate record without overwriting evidence."""
    args = _parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(
                f'refusing to overwrite candidate evidence: {args.output}'
            )
        request = load_json_object(args.request, 'candidate request')
        record = build_candidate_image_record(
            request,
            ros_distro=args.ros_distro,
            platform=args.platform,
            digest=args.digest,
            cli_version=args.cli_version,
            workflow_run_url=args.workflow_run_url,
            evidence_retention_days=args.evidence_retention_days,
        )
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(record, stream, indent=2, sort_keys=True)
            stream.write('\n')
    except (OSError, ValueError) as exc:
        print(f'candidate image record error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
