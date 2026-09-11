#!/usr/bin/env python3
"""Verify the complete Humble/Jazzy immutable candidate-image pair."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Sequence

from product_schema import load_json_object, validate_contract


IMAGE_SCHEMA = 'candidate-image-v1.schema.json'
SET_SCHEMA = 'candidate-image-set-v1.schema.json'
SET_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/candidate-image-set-v1.schema.json'
)


def verify_candidate_image_set(
    records: Sequence[dict[str, Any]],
) -> dict[str, Any]:
    """Require one consistent record for each supported ROS distro."""
    if len(records) != 2:
        raise ValueError('candidate set requires exactly two image records')
    for record in records:
        validate_contract(record, IMAGE_SCHEMA)
    by_distro = {record['ros_distro']: record for record in records}
    if set(by_distro) != {'humble', 'jazzy'} or len(by_distro) != 2:
        raise ValueError(
            'candidate set requires Humble and Jazzy exactly once'
        )
    shared_fields = (
        'repository',
        'source_pr',
        'source_commit',
        'product_version',
        'platform',
        'workflow_run_url',
        'workflow_branch_ref',
        'workflow_gate_commit',
        'requested_by',
        'publication_mode',
        'registry_retention_status',
        'evidence_retention_days',
    )
    for field in shared_fields:
        if len({record[field] for record in records}) != 1:
            raise ValueError(f'candidate image records disagree on {field}')
    if len({record['digest'] for record in records}) != 2:
        raise ValueError('Humble and Jazzy candidate digests must differ')
    if any(record['tags_created'] for record in records):
        raise ValueError('candidate image records must not create tags')
    if any(record['moving_tags_mutated'] for record in records):
        raise ValueError('candidate image records mutated a moving tag')
    if any(record['release_mutated'] for record in records):
        raise ValueError('candidate image records mutated a release')

    ordered = [by_distro['humble'], by_distro['jazzy']]
    report = {
        'schema_version': 1,
        'schema_uri': SET_SCHEMA_URI,
        'status': 'PASS',
        'publication_mode': 'digest_only',
        'repository': ordered[0]['repository'],
        'source_pr': ordered[0]['source_pr'],
        'source_commit': ordered[0]['source_commit'],
        'product_version': ordered[0]['product_version'],
        'platform': ordered[0]['platform'],
        'workflow_run_url': ordered[0]['workflow_run_url'],
        'workflow_branch_ref': ordered[0]['workflow_branch_ref'],
        'workflow_gate_commit': ordered[0]['workflow_gate_commit'],
        'requested_by': ordered[0]['requested_by'],
        'images': [
            {
                'ros_distro': record['ros_distro'],
                'digest': record['digest'],
                'immutable_ref': record['immutable_ref'],
            }
            for record in ordered
        ],
        'tags_created': [],
        'moving_tags_mutated': False,
        'release_mutated': False,
        'registry_retention_status': 'REQUIRES_REMOTE_AUDIT',
        'evidence_retention_days': ordered[0]['evidence_retention_days'],
    }
    validate_contract(report, SET_SCHEMA)
    return report


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('records', nargs=2, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Write one exclusive pair report."""
    args = _parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError(
                f'refusing to overwrite candidate set: {args.output}'
            )
        records = [
            load_json_object(path, 'candidate image record')
            for path in args.records
        ]
        report = verify_candidate_image_set(records)
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write('\n')
    except (OSError, ValueError) as exc:
        print(f'candidate image set error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
