#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Build a deterministic, attributed MID-360 onboarding rosbag fixture."""

from __future__ import annotations

import argparse
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import sqlite3
import subprocess
import tempfile
from typing import Any
import zipfile
import zlib

from product_schema import load_json_object, validate_contract
from slice_rosbag2 import slice_bag
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = (
    REPO_ROOT / 'configs' / 'real_data_e2e' /
    'driving_slam_mid360_v1.json'
)
SCHEMA_FILENAME = 'mid360-onboarding-fixture-v1.schema.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
    + SCHEMA_FILENAME
)
DEFAULT_FIXTURE_ID = 'mid360_onboarding_50s_v1'
DEFAULT_DURATION_SECONDS = 50.0
DEFAULT_MAX_ARCHIVE_BYTES = 100_000_000
SELECTED_TOPICS = ('/livox/lidar', '/livox/imu')
EXPECTED_TOPIC_TYPES = {
    '/livox/lidar': 'sensor_msgs/msg/PointCloud2',
    '/livox/imu': 'sensor_msgs/msg/Imu',
}
ZIP_MEMBER_TIMESTAMP = (1980, 1, 1, 0, 0, 0)
ZIP_MEMBER_MODE = 0o100644
CHUNK_BYTES = 8 * 1024 * 1024
SLUG_PATTERN = re.compile(r'^[a-z0-9][a-z0-9._-]{2,79}$')


def _file_digests(path: Path, algorithms: tuple[str, ...]) -> dict[str, str]:
    digests = {name: hashlib.new(name) for name in algorithms}
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(CHUNK_BYTES), b''):
            for digest in digests.values():
                digest.update(block)
    return {name: digest.hexdigest() for name, digest in digests.items()}


def _sha256(path: Path) -> str:
    return _file_digests(path, ('sha256',))['sha256']


def _require_relative_file(value: str, label: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if path.is_absolute() or not path.name or '..' in path.parts:
        raise ValueError(f'{label} must be a safe relative file path: {value!r}')
    return path


def _resolve_without_symlinks(path: Path, label: str) -> Path:
    absolute = Path(os.path.abspath(path))
    current = Path(absolute.anchor)
    for part in absolute.parts[1:]:
        current /= part
        if current.is_symlink():
            raise ValueError(f'{label} must not use symlink components: {current}')
    return absolute.resolve()


def _metadata_info(bag: Path) -> dict[str, Any]:
    metadata_path = bag / 'metadata.yaml'
    try:
        document = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
        info = document['rosbag2_bagfile_information']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as exc:
        raise ValueError(f'rosbag2 metadata is not readable: {metadata_path}') from exc
    if not isinstance(info, dict):
        raise ValueError(f'rosbag2 metadata root is invalid: {metadata_path}')
    return info


def _duration_ns(info: dict[str, Any]) -> int:
    try:
        return int(info['duration']['nanoseconds'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('rosbag2 metadata has no valid duration') from exc


def _starting_time_ns(info: dict[str, Any]) -> int:
    try:
        return int(info['starting_time']['nanoseconds_since_epoch'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('rosbag2 metadata has no valid starting time') from exc


def _topic_rows(info: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    try:
        topic_entries = info['topics_with_message_count']
        for entry in topic_entries:
            metadata = entry['topic_metadata']
            rows.append({
                'name': str(metadata['name']),
                'type': str(metadata['type']),
                'message_count': int(entry['message_count']),
            })
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('rosbag2 metadata has invalid topic records') from exc
    rows = sorted(rows, key=lambda row: row['name'])
    if len({row['name'] for row in rows}) != len(rows):
        raise ValueError('rosbag2 metadata contains duplicate topic names')
    return rows


def _storage_rows(bag: Path, info: dict[str, Any]) -> list[dict[str, Any]]:
    try:
        relative_files = list(info['relative_file_paths'])
    except (KeyError, TypeError) as exc:
        raise ValueError('rosbag2 metadata has no storage file list') from exc
    if not relative_files:
        raise ValueError('rosbag2 metadata lists no storage files')
    rows = []
    bag_root = bag.resolve()
    for value in relative_files:
        relative = _require_relative_file(str(value), 'rosbag2 storage path')
        path = bag.joinpath(*relative.parts)
        current = bag
        for part in relative.parts:
            current /= part
            if current.is_symlink():
                raise ValueError(
                    f'rosbag2 storage path uses a symlink: {relative}'
                )
        if not path.is_file() or path.is_symlink():
            raise ValueError(f'rosbag2 storage file is missing or unsafe: {relative}')
        try:
            path.resolve(strict=True).relative_to(bag_root)
        except (OSError, ValueError) as exc:
            raise ValueError(
                f'rosbag2 storage file escapes the bag root: {relative}'
            ) from exc
        rows.append({
            'path': relative.as_posix(),
            'size_bytes': path.stat().st_size,
            'sha256': _sha256(path),
        })
    return rows


def _bag_identity(bag: Path) -> dict[str, Any]:
    info = _metadata_info(bag)
    metadata = bag / 'metadata.yaml'
    try:
        message_count = int(info['message_count'])
        storage_identifier = str(info['storage_identifier'])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError('rosbag2 metadata has invalid bag identity fields') from exc
    return {
        'metadata': {
            'filename': 'metadata.yaml',
            'size_bytes': metadata.stat().st_size,
            'sha256': _sha256(metadata),
        },
        'storage_identifier': storage_identifier,
        'storage_files': _storage_rows(bag, info),
        'starting_time_ns': _starting_time_ns(info),
        'duration_ns': _duration_ns(info),
        'message_count': message_count,
        'topics': _topic_rows(info),
    }


def _expect_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise ValueError(f'{label} mismatch: expected {expected!r}, got {actual!r}')


def _verify_source(
    archive: Path,
    bag: Path,
    contract: dict[str, Any],
) -> dict[str, Any]:
    if not archive.is_file() or archive.is_symlink():
        raise ValueError(f'source archive is missing or unsafe: {archive}')
    if not bag.is_dir() or bag.is_symlink():
        raise ValueError(f'source bag is missing or unsafe: {bag}')
    try:
        dataset = contract['dataset']
        expected_input = contract['input']
        license_info = dataset['license']
    except (KeyError, TypeError) as exc:
        raise ValueError('source contract is missing provenance fields') from exc

    _expect_equal(archive.name, dataset['filename'], 'source archive filename')
    _expect_equal(bag.name, expected_input['bag_directory'], 'source bag directory')
    archive_digests = _file_digests(archive, ('md5', 'sha256'))
    _expect_equal(archive.stat().st_size, dataset['size_bytes'], 'source archive size')
    _expect_equal(archive_digests['md5'], dataset['md5'], 'source archive MD5')
    _expect_equal(
        archive_digests['sha256'], dataset['sha256'], 'source archive SHA-256'
    )

    identity = _bag_identity(bag)
    expected_metadata = {
        'filename': 'metadata.yaml',
        'size_bytes': expected_input['metadata_size_bytes'],
        'sha256': expected_input['metadata_sha256'],
    }
    _expect_equal(identity['metadata'], expected_metadata, 'source metadata identity')
    _expect_equal(
        identity['storage_identifier'],
        expected_input['storage_identifier'],
        'source storage identifier',
    )
    _expect_equal(
        identity['storage_files'],
        expected_input['storage_files'],
        'source storage identity',
    )
    _expect_equal(
        identity['message_count'], expected_input['message_count'],
        'source message count'
    )
    expected_duration_ns = round(float(expected_input['duration_sec']) * 1e9)
    tolerance_ns = round(float(expected_input['duration_tolerance_sec']) * 1e9)
    if abs(identity['duration_ns'] - expected_duration_ns) > tolerance_ns:
        raise ValueError(
            'source duration mismatch: expected '
            f'{expected_duration_ns} +/- {tolerance_ns} ns, '
            f"got {identity['duration_ns']} ns"
        )
    actual_topics = {row['name']: row for row in identity['topics']}
    for name, values in expected_input['topics'].items():
        expected_topic = {'name': name, **values}
        _expect_equal(
            actual_topics.get(name), expected_topic,
            f'source topic identity for {name}'
        )
    _expect_equal(
        sum(row['message_count'] for row in identity['topics']),
        identity['message_count'],
        'source metadata topic-count sum',
    )

    required_dataset_strings = (
        'id', 'title', 'creator', 'citation', 'source_url', 'doi',
        'download_url',
    )
    for key in required_dataset_strings:
        if not isinstance(dataset.get(key), str) or not dataset[key]:
            raise ValueError(f'source contract dataset.{key} must be non-empty')
    for key in ('spdx', 'name', 'url'):
        if not isinstance(license_info.get(key), str) or not license_info[key]:
            raise ValueError(f'source contract dataset.license.{key} must be non-empty')

    return {
        'dataset_id': dataset['id'],
        'title': dataset['title'],
        'creator': dataset['creator'],
        'citation': dataset['citation'],
        'source_url': dataset['source_url'],
        'doi': dataset['doi'],
        'download_url': dataset['download_url'],
        'license_spdx': license_info['spdx'],
        'license_name': license_info['name'],
        'license_url': license_info['url'],
        'archive': {
            'filename': dataset['filename'],
            'size_bytes': archive.stat().st_size,
            **archive_digests,
        },
        'bag': {
            'directory_name': expected_input['bag_directory'],
            **identity,
        },
    }


def _git_identity(repo_root: Path, allow_dirty: bool) -> dict[str, Any]:
    def run(*arguments: str) -> str:
        result = subprocess.run(
            ['git', '-C', str(repo_root), *arguments],
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            raise ValueError(f'git identity command failed: {detail}')
        return result.stdout.strip()

    commit = run('rev-parse', 'HEAD')
    if not re.fullmatch(r'[0-9a-f]{40}', commit):
        raise ValueError(f'git returned an invalid commit: {commit!r}')
    dirty = bool(run('status', '--porcelain', '--untracked-files=all'))
    if dirty and not allow_dirty:
        raise ValueError(
            'repository is dirty; commit the generator and contract first or '
            'use --allow-dirty for a non-publishable pilot'
        )
    return {'git_commit': commit, 'git_dirty': dirty}


def _package_version(distribution: str) -> str:
    try:
        return version(distribution)
    except PackageNotFoundError:
        return 'unknown'


def _attribution(source: dict[str, Any], duration_ns: int) -> str:
    duration_seconds = duration_ns / 1e9
    return f"""# Attribution and license

This ROS 2 bag is a derivative onboarding fixture made from
\"{source['title']}\" by {source['creator']}.

- Original source: {source['source_url']}
- DOI: https://doi.org/{source['doi']}
- Citation: {source['citation']}
- License: {source['license_name']} ({source['license_url']})

Changes made by the lidar_slam_ros2 project: retained the first
{duration_seconds:g} seconds of messages on `/livox/lidar` and `/livox/imu`,
copied their serialized payloads without deserialization, rewrote the rosbag2
container and metadata, and packaged the result as a deterministic ZIP.

This fixture is for onboarding only. It does not replace the full 277-second
real-data release gate. No endorsement by the original creator is implied.
"""


def _zip_members(fixture_dir: Path, fixture_id: str) -> list[dict[str, Any]]:
    rows = []
    for path in sorted(fixture_dir.rglob('*')):
        if not path.is_file():
            continue
        if path.is_symlink():
            raise ValueError(f'ZIP source member must not be a symlink: {path}')
        relative = path.relative_to(fixture_dir).as_posix()
        rows.append({
            'path': f'{fixture_id}/{relative}',
            'size_bytes': path.stat().st_size,
            'sha256': _sha256(path),
        })
    if not rows:
        raise ValueError('fixture contains no files')
    return rows


def _write_deterministic_zip(
    fixture_dir: Path,
    fixture_id: str,
    destination: Path,
) -> list[dict[str, Any]]:
    members = _zip_members(fixture_dir, fixture_id)
    with zipfile.ZipFile(
        destination,
        mode='x',
        compression=zipfile.ZIP_DEFLATED,
        compresslevel=9,
    ) as archive:
        for row in members:
            relative = PurePosixPath(row['path']).relative_to(fixture_id)
            source_path = fixture_dir.joinpath(*relative.parts)
            info = zipfile.ZipInfo(row['path'], ZIP_MEMBER_TIMESTAMP)
            info.compress_type = zipfile.ZIP_DEFLATED
            info.create_system = 3
            info.external_attr = ZIP_MEMBER_MODE << 16
            info._compresslevel = 9
            with source_path.open('rb') as source_stream:
                with archive.open(info, mode='w', force_zip64=False) as output:
                    shutil.copyfileobj(source_stream, output, CHUNK_BYTES)
    return members


def _contract_repo_path(contract_path: Path) -> str:
    try:
        return contract_path.resolve().relative_to(REPO_ROOT).as_posix()
    except ValueError as exc:
        raise ValueError('source contract must be inside the repository') from exc


def build_fixture(
    source_archive: Path,
    source_bag: Path,
    output_dir: Path,
    contract_path: Path = DEFAULT_CONTRACT,
    fixture_id: str = DEFAULT_FIXTURE_ID,
    start_offset_seconds: float = 0.0,
    duration_seconds: float = DEFAULT_DURATION_SECONDS,
    max_archive_bytes: int = DEFAULT_MAX_ARCHIVE_BYTES,
    allow_dirty: bool = False,
) -> dict[str, Any]:
    """Build the ZIP and sidecar manifest, returning the validated manifest."""
    if not SLUG_PATTERN.fullmatch(fixture_id):
        raise ValueError(f'invalid fixture id: {fixture_id!r}')
    if (
        not math.isfinite(start_offset_seconds)
        or not math.isfinite(duration_seconds)
        or start_offset_seconds < 0.0
        or duration_seconds <= 0.0
    ):
        raise ValueError('start offset must be non-negative and duration positive')
    if max_archive_bytes <= 0:
        raise ValueError('maximum archive size must be positive')

    source_archive = _resolve_without_symlinks(
        source_archive, 'source archive'
    )
    source_bag = _resolve_without_symlinks(source_bag, 'source bag')
    contract_path = _resolve_without_symlinks(
        contract_path, 'source contract'
    )
    contract = load_json_object(contract_path, 'source contract')
    source = _verify_source(source_archive, source_bag, contract)
    revision = _git_identity(REPO_ROOT, allow_dirty)
    start_offset_ns = round(start_offset_seconds * 1e9)
    requested_duration_ns = round(duration_seconds * 1e9)

    output_dir.mkdir(parents=True, exist_ok=True)
    if not output_dir.is_dir() or output_dir.is_symlink():
        raise ValueError(f'output directory is missing or unsafe: {output_dir}')
    artifact_path = output_dir / f'{fixture_id}.zip'
    manifest_path = output_dir / f'{fixture_id}.manifest.json'
    for path in (artifact_path, manifest_path):
        if path.exists() or path.is_symlink():
            raise ValueError(f'output already exists: {path}')

    with tempfile.TemporaryDirectory(
        prefix=f'.{fixture_id}.', dir=output_dir
    ) as temporary:
        temporary_root = Path(temporary)
        fixture_dir = temporary_root / fixture_id
        output_bag = fixture_dir / 'bag'
        counts = slice_bag(
            source_bag,
            output_bag,
            duration_seconds,
            set(SELECTED_TOPICS),
            start_offset_seconds,
        )
        expected_source_identity = dict(source['bag'])
        expected_source_identity.pop('directory_name')
        _expect_equal(
            _bag_identity(source_bag),
            expected_source_identity,
            'source bag identity after slicing',
        )
        for topic in SELECTED_TOPICS:
            if counts.get(topic, 0) <= 0:
                raise ValueError(f'generated fixture has no messages on {topic}')
        attribution_path = fixture_dir / 'ATTRIBUTION.md'
        attribution_path.write_text(
            _attribution(source, requested_duration_ns), encoding='utf-8'
        )

        output_identity = _bag_identity(output_bag)
        output_topics = output_identity['topics']
        topic_types = {row['name']: row['type'] for row in output_topics}
        _expect_equal(topic_types, EXPECTED_TOPIC_TYPES, 'generated topic types')
        _expect_equal(
            {row['name']: row['message_count'] for row in output_topics},
            counts,
            'generated topic counts',
        )
        _expect_equal(
            output_identity['message_count'], sum(counts.values()),
            'generated message count'
        )

        temporary_artifact = temporary_root / artifact_path.name
        members = _write_deterministic_zip(
            fixture_dir, fixture_id, temporary_artifact
        )
        artifact_size = temporary_artifact.stat().st_size
        if artifact_size > max_archive_bytes:
            raise ValueError(
                f'archive size gate failed: {artifact_size} > '
                f'{max_archive_bytes} bytes'
            )

        tool_path = Path(__file__).resolve()
        slicer_path = tool_path.with_name('slice_rosbag2.py')
        contract_repo_path = _contract_repo_path(contract_path)
        manifest = {
            'schema_version': 1,
            'schema_uri': SCHEMA_URI,
            'status': 'BUILT',
            'fixture_id': fixture_id,
            'role': 'onboarding',
            'source': source,
            'clip': {
                'start_offset_ns': start_offset_ns,
                'requested_duration_ns': requested_duration_ns,
                'actual_duration_ns': output_identity['duration_ns'],
                'first_message_timestamp_ns': output_identity['starting_time_ns'],
                'raw_message_copy': True,
                'topics': output_topics,
                'message_count': output_identity['message_count'],
                'bag': {
                    'directory_name': 'bag',
                    'metadata': output_identity['metadata'],
                    'storage_identifier': output_identity['storage_identifier'],
                    'storage_files': output_identity['storage_files'],
                },
            },
            'generation': {
                'revision': revision,
                'tool': {
                    'path': 'scripts/build_mid360_onboarding_fixture.py',
                    'sha256': _sha256(tool_path),
                },
                'slicer': {
                    'path': 'scripts/slice_rosbag2.py',
                    'sha256': _sha256(slicer_path),
                },
                'source_contract': {
                    'path': contract_repo_path,
                    'sha256': _sha256(contract_path),
                },
                'toolchain': {
                    'python_version': platform.python_version(),
                    'rosbags_version': _package_version('rosbags'),
                    'sqlite_version': sqlite3.sqlite_version,
                    'zlib_version': zlib.ZLIB_VERSION,
                    'zlib_runtime_version': zlib.ZLIB_RUNTIME_VERSION,
                },
                'archive_recipe': {
                    'compression': 'deflate',
                    'compression_level': 9,
                    'member_timestamp': '1980-01-01T00:00:00Z',
                    'member_mode': '0644',
                    'sorted_members': True,
                },
            },
            'artifact': {
                'filename': artifact_path.name,
                'format': 'zip',
                'size_bytes': artifact_size,
                'sha256': _sha256(temporary_artifact),
                'max_size_bytes': max_archive_bytes,
                'size_gate_pass': True,
                'archive_root': fixture_id,
                'members': members,
            },
            'publication': {
                'onboarding_only': True,
                'replaces_full_real_data_gate': False,
                'contains_map_geometry': True,
                'track_archive_in_git': False,
                'review_before_publishing': True,
                'map_validation_status': 'NOT_RUN',
                'attribution_member': f'{fixture_id}/ATTRIBUTION.md',
            },
        }
        validate_contract(manifest, SCHEMA_FILENAME)

        temporary_manifest = temporary_root / manifest_path.name
        temporary_manifest.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        created_outputs = []
        try:
            for source_path, destination in (
                (temporary_artifact, artifact_path),
                (temporary_manifest, manifest_path),
            ):
                os.link(source_path, destination, follow_symlinks=False)
                identity = os.lstat(destination)
                created_outputs.append(
                    (destination, (identity.st_dev, identity.st_ino))
                )
        except OSError:
            for path, expected_identity in created_outputs:
                try:
                    actual = os.lstat(path)
                except FileNotFoundError:
                    continue
                if (actual.st_dev, actual.st_ino) == expected_identity:
                    path.unlink()
            raise
    return manifest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-archive', type=Path, required=True)
    parser.add_argument('--source-bag', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--contract', type=Path, default=DEFAULT_CONTRACT)
    parser.add_argument('--fixture-id', default=DEFAULT_FIXTURE_ID)
    parser.add_argument(
        '--start-offset-seconds', type=float, default=0.0
    )
    parser.add_argument(
        '--duration-seconds', type=float, default=DEFAULT_DURATION_SECONDS
    )
    parser.add_argument(
        '--max-archive-bytes',
        type=int,
        default=DEFAULT_MAX_ARCHIVE_BYTES,
    )
    parser.add_argument(
        '--allow-dirty',
        action='store_true',
        help='allow a non-publishable pilot from a dirty checkout',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manifest = build_fixture(
        source_archive=args.source_archive,
        source_bag=args.source_bag,
        output_dir=args.output_dir.resolve(),
        contract_path=args.contract,
        fixture_id=args.fixture_id,
        start_offset_seconds=args.start_offset_seconds,
        duration_seconds=args.duration_seconds,
        max_archive_bytes=args.max_archive_bytes,
        allow_dirty=args.allow_dirty,
    )
    artifact = manifest['artifact']
    print(f"status: {manifest['status']}")
    print(f"artifact: {(args.output_dir.resolve() / artifact['filename'])}")
    print(f"size_bytes: {artifact['size_bytes']}")
    print(f"sha256: {artifact['sha256']}")
    print('map_validation_status: NOT_RUN')
    print('review_before_publishing: true')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
