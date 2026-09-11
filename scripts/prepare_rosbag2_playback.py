#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Isolate rosbag2 FILE decompression from the source bag directory."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import tempfile

import yaml


STAGING_PREFIX = '.lidarslam-rosbag2-playback-'
MARKER_NAME = '.lidarslam-rosbag2-playback-v1.json'


def _load_bag_metadata(bag_path: Path) -> tuple[bytes, dict]:
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        raise ValueError(f'metadata.yaml not found under {bag_path}')
    metadata_bytes = metadata_path.read_bytes()
    metadata = yaml.safe_load(metadata_bytes) or {}
    if not isinstance(metadata, dict):
        raise ValueError('rosbag2 metadata root must be an object')
    bag_info = metadata.get('rosbag2_bagfile_information') or {}
    if not isinstance(bag_info, dict):
        raise ValueError('rosbag2_bagfile_information must be an object')
    return metadata_bytes, bag_info


def _safe_relative_storage_path(value: object) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError('rosbag2 relative_file_paths must contain strings')
    path = Path(value)
    if path.is_absolute() or '..' in path.parts:
        raise ValueError(f'unsafe rosbag2 storage path: {value}')
    if path.name in {'', '.', '..'}:
        raise ValueError(f'invalid rosbag2 storage path: {value}')
    return path


def stage_for_playback(bag: Path, staging_root: Path) -> Path:
    """Return the source bag or a private view for FILE-compressed playback."""
    bag = bag.expanduser().resolve(strict=True)
    staging_root = staging_root.expanduser().resolve(strict=True)
    if not bag.is_dir():
        raise ValueError(f'bag is not a directory: {bag}')
    if not staging_root.is_dir():
        raise ValueError(f'staging root is not a directory: {staging_root}')

    metadata_bytes, bag_info = _load_bag_metadata(bag)
    compression_mode = str(bag_info.get('compression_mode') or '').upper()
    if compression_mode != 'FILE':
        return bag

    compression_format = str(bag_info.get('compression_format') or '').strip()
    if not compression_format or any(
        character in compression_format for character in '/\\'
    ):
        raise ValueError(
            'FILE-compressed rosbag2 metadata has an invalid '
            'compression_format'
        )
    relative_paths = bag_info.get('relative_file_paths') or []
    if not isinstance(relative_paths, list) or not relative_paths:
        raise ValueError(
            'FILE-compressed rosbag2 metadata has no relative_file_paths'
        )

    staging_path = Path(tempfile.mkdtemp(
        prefix=STAGING_PREFIX,
        dir=staging_root,
    ))
    try:
        seen: set[Path] = set()
        for value in relative_paths:
            relative_path = _safe_relative_storage_path(value)
            if relative_path in seen:
                raise ValueError(
                    f'duplicate rosbag2 storage path: {relative_path}'
                )
            seen.add(relative_path)
            if not relative_path.name.endswith(f'.{compression_format}'):
                raise ValueError(
                    'FILE-compressed rosbag2 storage path does not end in '
                    f'.{compression_format}: {relative_path}'
                )

            source = (bag / relative_path).resolve(strict=True)
            try:
                source.relative_to(bag)
            except ValueError as exc:
                raise ValueError(
                    f'rosbag2 storage path escapes the bag: {relative_path}'
                ) from exc
            if not source.is_file():
                raise ValueError(f'rosbag2 storage is not a file: {source}')

            destination = staging_path / relative_path
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.symlink_to(source)

        (staging_path / 'metadata.yaml').write_bytes(metadata_bytes)
        marker = {
            'schema_version': 1,
            'source_bag': str(bag),
            'staging_root': str(staging_root),
        }
        (staging_path / MARKER_NAME).write_text(
            json.dumps(marker, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
    except Exception:
        shutil.rmtree(staging_path)
        raise
    return staging_path


def cleanup_staging(staging_path: Path, staging_root: Path) -> None:
    """Remove only a marker-backed staging directory created by this tool."""
    staging_root = staging_root.expanduser().resolve(strict=True)
    staging_path = staging_path.expanduser().resolve(strict=True)
    if staging_path.parent != staging_root:
        raise ValueError(
            f'staging path is not a direct child of {staging_root}: '
            f'{staging_path}'
        )
    if not staging_path.name.startswith(STAGING_PREFIX):
        raise ValueError(
            f'unexpected staging directory name: {staging_path.name}'
        )
    if not staging_path.is_dir() or staging_path.is_symlink():
        raise ValueError(
            f'staging path is not a real directory: {staging_path}'
        )

    marker_path = staging_path / MARKER_NAME
    if not marker_path.is_file() or marker_path.is_symlink():
        raise ValueError(f'staging marker is missing or unsafe: {marker_path}')
    marker = json.loads(marker_path.read_text(encoding='utf-8'))
    if marker.get('schema_version') != 1:
        raise ValueError(f'unsupported staging marker: {marker_path}')
    if marker.get('staging_root') != str(staging_root):
        raise ValueError(f'staging marker root mismatch: {marker_path}')

    shutil.rmtree(staging_path)


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)

    stage_parser = subparsers.add_parser(
        'stage',
        help=(
            'create an isolated playback view when the bag is '
            'FILE-compressed'
        ),
    )
    stage_parser.add_argument('--bag', type=Path, required=True)
    stage_parser.add_argument('--staging-root', type=Path, required=True)

    cleanup_parser = subparsers.add_parser(
        'cleanup',
        help='remove one marker-backed playback staging directory',
    )
    cleanup_parser.add_argument('--path', type=Path, required=True)
    cleanup_parser.add_argument('--staging-root', type=Path, required=True)
    return parser


def main() -> int:
    """Run the staging or cleanup command."""
    args = build_parser().parse_args()
    try:
        if args.command == 'stage':
            print(stage_for_playback(args.bag, args.staging_root))
        else:
            cleanup_staging(args.path, args.staging_root)
            print(f'removed_playback_staging: {args.path.resolve()}')
    except (OSError, ValueError, json.JSONDecodeError, yaml.YAMLError) as exc:
        raise SystemExit(f'error: {exc}') from exc
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
