#!/usr/bin/env python3
"""Create validated release-image evidence from a tested image digest."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Any

from product_schema import validate_contract


SCHEMA_NAME = 'release-image-v1.schema.json'
TAG_PATTERN = re.compile(
    r'^ghcr\.io/(?P<repository>[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+):'
    r'v(?P<version>[0-9]+\.[0-9]+\.[0-9]+)-'
    r'(?P<ros_distro>humble|jazzy)$'
)


def build_release_image_record(
    *,
    ros_distro: str,
    platform: str,
    tag: str,
    digest: str,
    git_commit: str,
    product_version: str,
    cli_version: str,
) -> dict[str, Any]:
    """Build and validate one internally consistent release-image record."""
    record = {
        'schema_version': 1,
        'status': 'PASS',
        'ros_distro': ros_distro,
        'platform': platform,
        'tag': tag,
        'digest': digest,
        'git_commit': git_commit,
        'product_version': product_version,
        'cli_version': cli_version,
    }
    validate_contract(record, SCHEMA_NAME)
    match = TAG_PATTERN.fullmatch(tag)
    if match is None:
        raise ValueError(
            'release image tag does not match the public contract'
        )
    if match.group('version') != product_version:
        raise ValueError(
            'release image tag version does not match product_version'
        )
    if match.group('ros_distro') != ros_distro:
        raise ValueError(
            'release image tag distro does not match ros_distro'
        )
    if cli_version != f'lidarslam_ros2 {product_version}':
        raise ValueError(
            'release image cli_version does not match product_version'
        )
    return record


def parse_args() -> argparse.Namespace:
    """Parse release-image record command-line arguments."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_RELEASE_COMMAND'),
        description='Create one validated release-image-v1 evidence record.',
    )
    parser.add_argument(
        '--ros-distro',
        required=True,
        choices=('humble', 'jazzy'),
    )
    parser.add_argument('--platform', default='linux/amd64')
    parser.add_argument('--tag', required=True)
    parser.add_argument('--digest', required=True)
    parser.add_argument('--git-commit', required=True)
    parser.add_argument('--product-version', required=True)
    parser.add_argument('--cli-version', required=True)
    parser.add_argument('--output', required=True)
    return parser.parse_args()


def main() -> int:
    """Create one release-image record without overwriting evidence."""
    args = parse_args()
    output = Path(args.output).expanduser().resolve()
    try:
        if output.exists():
            raise ValueError(
                f'refusing to overwrite release evidence: {output}'
            )
        record = build_release_image_record(
            ros_distro=args.ros_distro,
            platform=args.platform,
            tag=args.tag,
            digest=args.digest,
            git_commit=args.git_commit,
            product_version=args.product_version,
            cli_version=args.cli_version,
        )
        output.write_text(
            json.dumps(record, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
    except (OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
