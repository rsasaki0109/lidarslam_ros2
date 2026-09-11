#!/usr/bin/env python3
"""Prepare one verified NTU-VIRAL archive as canonical ROS1/ROS2 inputs."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import subprocess
import sys
import zipfile


ROOT = Path(__file__).resolve().parents[1]
TOPICS = ('/os1_cloud_node1/points', '/imu/imu', '/left/image_raw')


def file_digest(path: Path, algorithm: str) -> str:
    value = hashlib.new(algorithm)
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def tree_digest(path: Path) -> str:
    value = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        value.update(candidate.relative_to(path).as_posix().encode())
        value.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
                value.update(block)
    return value.hexdigest()


def verify_archive(path: Path, expected_size: int,
                   expected_md5: str) -> dict[str, object]:
    if not path.is_file():
        raise ValueError(f'archive is missing: {path}')
    size = path.stat().st_size
    if size != expected_size:
        raise ValueError(
            f'archive size mismatch: expected {expected_size}, got {size}')
    md5 = file_digest(path, 'md5')
    if md5.lower() != expected_md5.lower():
        raise ValueError(f'archive MD5 mismatch: expected {expected_md5}, got {md5}')
    return {
        'path': str(path.resolve()), 'size_bytes': size, 'md5': md5,
        'sha256': file_digest(path, 'sha256')}


def safe_extract_zip(archive: Path, destination: Path) -> None:
    destination = destination.resolve()
    destination.mkdir(parents=True, exist_ok=False)
    with zipfile.ZipFile(archive) as source:
        for member in source.infolist():
            target = (destination / member.filename).resolve()
            if target != destination and destination not in target.parents:
                raise ValueError(f'unsafe ZIP member path: {member.filename}')
            mode = member.external_attr >> 16
            if stat.S_ISLNK(mode):
                raise ValueError(f'ZIP symlink is not permitted: {member.filename}')
        source.extractall(destination)


def require_single_bag(extracted: Path) -> Path:
    bags = sorted(extracted.rglob('*.bag'))
    if len(bags) != 1:
        raise ValueError(
            f'expected exactly one ROS1 bag under {extracted}, got {len(bags)}')
    return bags[0].resolve()


def run(command: list[str]) -> None:
    print(json.dumps({'command': command}, sort_keys=True), flush=True)
    subprocess.run(command, check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sequence', required=True)
    parser.add_argument('--archive', required=True, type=Path)
    parser.add_argument('--expected-size', required=True, type=int)
    parser.add_argument('--expected-md5', required=True)
    parser.add_argument('--output-root', required=True, type=Path)
    args = parser.parse_args()

    archive = args.archive.resolve()
    archive_record = verify_archive(
        archive, args.expected_size, args.expected_md5)
    sequence_root = (args.output_root.resolve() / args.sequence)
    sequence_root.mkdir(parents=True, exist_ok=True)
    extracted = sequence_root / 'extracted'
    extraction_identity = extracted / '.official_archive.json'
    if not extracted.exists():
        temporary = sequence_root / '.extracting'
        if temporary.exists():
            raise ValueError(f'stale extraction directory exists: {temporary}')
        safe_extract_zip(archive, temporary)
        (temporary / '.official_archive.json').write_text(
            json.dumps(archive_record, indent=2, sort_keys=True) + '\n')
        os.replace(temporary, extracted)
    elif not extraction_identity.is_file():
        raise ValueError(
            f'refusing unproven existing extraction directory: {extracted}')
    elif json.loads(extraction_identity.read_text()) != archive_record:
        raise ValueError('existing extraction belongs to a different archive')

    original = require_single_bag(extracted)
    normalized = sequence_root / 'canonical_ros1.bag'
    normalization_report = sequence_root / 'normalization_report.json'
    if normalized.exists() != normalization_report.exists():
        raise ValueError('incomplete normalization artifacts already exist')
    if not normalized.exists():
        run([
            sys.executable, str(ROOT / 'scripts/normalize_ntu_viral_rosbag.py'),
            '--input', str(original), '--output', str(normalized),
            '--report', str(normalization_report)])

    ros2 = sequence_root / 'canonical_ros2'
    if not (ros2 / 'metadata.yaml').is_file():
        if ros2.exists():
            raise ValueError(f'incomplete ROS2 conversion exists: {ros2}')
        converter = shutil.which('rosbags-convert')
        if converter is None:
            raise ValueError('rosbags-convert is not installed')
        run([converter, '--src', str(normalized), '--dst', str(ros2)])

    semantic_report = sequence_root / 'semantic_report.json'
    command = [
        sys.executable, str(ROOT / 'scripts/compare_rosbag_semantic_inputs.py'),
        '--left', str(normalized), '--right', str(ros2),
        '--output', str(semantic_report)]
    for topic in TOPICS:
        command.extend(['--topic', topic])
    run(command)

    normalization = json.loads(normalization_report.read_text())
    semantic = json.loads(semantic_report.read_text())
    if semantic.get('all_topics_equal') is not True:
        raise ValueError('ROS1/ROS2 semantic comparison failed')
    manifest = {
        'schema_version': 1,
        'sequence': args.sequence,
        'status': 'prepared_semantically_verified',
        'archive': archive_record,
        'original_ros1_bag': {
            'path': str(original), 'sha256': file_digest(original, 'sha256')},
        'canonical_ros1_bag': {
            'path': str(normalized), 'sha256': file_digest(normalized, 'sha256')},
        'canonical_ros2': {
            'path': str(ros2), 'tree_sha256': tree_digest(ros2)},
        'normalization_report': normalization,
        'semantic_report': semantic,
        'topics': list(TOPICS),
    }
    manifest_path = sequence_root / 'preparation_manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    print(json.dumps({'output': str(manifest_path), 'status': manifest['status']},
                     sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError,
            zipfile.BadZipFile, subprocess.CalledProcessError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
