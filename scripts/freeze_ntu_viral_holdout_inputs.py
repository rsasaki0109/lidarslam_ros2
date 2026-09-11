#!/usr/bin/env python3
"""Validate and freeze one normalized NTU-VIRAL competitive holdout."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import yaml


REQUIRED_TOPICS = {
    '/os1_cloud_node1/points', '/imu/imu', '/left/image_raw'}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def md5_file(path: Path) -> str:
    digest = hashlib.md5()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def sha256_tree(path: Path) -> str:
    digest = hashlib.sha256()
    for candidate in sorted(item for item in path.rglob('*') if item.is_file()):
        digest.update(candidate.relative_to(path).as_posix().encode())
        digest.update(b'\0')
        with candidate.open('rb') as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
                digest.update(block)
    return digest.hexdigest()


def hash_path(path: Path) -> str:
    if path.is_file():
        return sha256_file(path)
    if path.is_dir():
        return sha256_tree(path)
    raise ValueError(f'required input is missing: {path}')


def metadata_summary(path: Path) -> dict[str, Any]:
    info = yaml.safe_load(path.read_text(encoding='utf-8'))[
        'rosbag2_bagfile_information']
    topics = sorted(row['topic_metadata']['name']
                    for row in info['topics_with_message_count'])
    return {
        'storage_identifier': info['storage_identifier'],
        'duration_nanoseconds': info['duration']['nanoseconds'],
        'starting_time_nanoseconds': info['starting_time']['nanoseconds_since_epoch'],
        'message_count': info['message_count'],
        'topics': topics,
    }


def validate_semantic_report(path: Path) -> None:
    report = json.loads(path.read_text(encoding='utf-8'))
    equal = {row['topic'] for row in report.get('topics', [])
             if row.get('equal') is True}
    if report.get('all_topics_equal') is not True or not REQUIRED_TOPICS <= equal:
        raise ValueError('normalized ROS1/derived ROS2 semantic comparison failed')


def build_manifest(args: argparse.Namespace) -> dict[str, Any]:
    legacy_rko = getattr(args, 'rko_param', None)
    rko_lio_param = getattr(args, 'rko_lio_param', None) or legacy_rko
    rko_liv_param = getattr(args, 'rko_liv_param', None) or legacy_rko
    if rko_lio_param is None or rko_liv_param is None:
        raise ValueError(
            'both LiDAR-IMU and LiDAR-IMU-Visual RKO params are required')
    for path in (args.official_archive, args.original_ros1_bag,
                 args.normalized_ros1_bag,
                 args.normalization_report, args.semantic_report,
                 args.reference, args.reference_metadata,
                 args.candidate_manifest, rko_lio_param, rko_liv_param,
                 args.lidarslam_param, args.fast_mapping_launch,
                 args.fast_mapping_map_launch, args.fast_official_config,
                 args.fast_official_camera_config, args.fast_image_identity):
        if not path.is_file():
            raise ValueError(f'required file is missing: {path}')
    for path in (args.ros2_bag, args.glim_config_dir):
        if not path.is_dir():
            raise ValueError(f'required directory is missing: {path}')
    hash_path(args.calibration)
    summary = metadata_summary(args.ros2_bag / 'metadata.yaml')
    missing = REQUIRED_TOPICS.difference(summary['topics'])
    if missing:
        raise ValueError(f'derived ROS2 bag lacks topics: {sorted(missing)}')
    validate_semantic_report(args.semantic_report)

    archive_size = args.official_archive.stat().st_size
    if archive_size != args.archive_expected_bytes:
        raise ValueError(
            f'official archive size mismatch: expected '
            f'{args.archive_expected_bytes}, got {archive_size}')
    archive_md5 = md5_file(args.official_archive)
    if archive_md5.lower() != args.archive_expected_md5.lower():
        raise ValueError('official archive MD5 mismatch')

    normalization = json.loads(args.normalization_report.read_text())
    original_hash = sha256_file(args.original_ros1_bag)
    normalized_hash = sha256_file(args.normalized_ros1_bag)
    if normalization.get('source_sha256') != original_hash:
        raise ValueError('normalization report source hash mismatch')
    if normalization.get('destination_sha256') != normalized_hash:
        raise ValueError('normalization report destination hash mismatch')
    candidate = json.loads(args.candidate_manifest.read_text())
    if candidate.get('candidate_status') != 'frozen_before_holdout':
        raise ValueError('candidate manifest is not frozen_before_holdout')

    return {
        'schema_version': 1,
        'slot': args.slot,
        'sequence': args.sequence,
        'dataset': 'NTU-VIRAL',
        'status': 'frozen',
        'candidate': {
            'repository_revision': candidate['repository_revision'],
            'source_tree_sha256': candidate['source_tree']['sha256'],
        },
        'representations': {
            'distribution': 'official NTU-VIRAL ZIP archive',
            'source': 'official original ROS1 bag extracted from distribution',
            'canonical_experiment_input': (
                'author-recommended deterministic Ouster-restamped ROS1 bag'),
            'ros2': 'semantic-equivalent conversion of canonical input',
            'semantic_topics': sorted(REQUIRED_TOPICS),
        },
        'hashes': {
            'official_archive_bytes': archive_size,
            'official_archive_md5': archive_md5,
            'official_archive_sha256': sha256_file(args.official_archive),
            'original_rosbag1_sha256': original_hash,
            # Compatibility key used by the FAST-LIVO2 runner.
            'raw_rosbag1_sha256': normalized_hash,
            'canonical_rosbag2_tree_sha256': sha256_tree(args.ros2_bag),
            'normalization_report_sha256': sha256_file(args.normalization_report),
            'semantic_report_sha256': sha256_file(args.semantic_report),
            'reference_sha256': sha256_file(args.reference),
            'reference_metadata_sha256': sha256_file(args.reference_metadata),
            'calibration_sha256': hash_path(args.calibration),
            'candidate_manifest_sha256': sha256_file(args.candidate_manifest),
            # Compatibility alias is the visual-track candidate.
            'rko_param_sha256': sha256_file(rko_liv_param),
            'rko_lio_param_sha256': sha256_file(rko_lio_param),
            'rko_liv_param_sha256': sha256_file(rko_liv_param),
            'lidarslam_param_sha256': sha256_file(args.lidarslam_param),
            'glim_config_tree_sha256': sha256_tree(args.glim_config_dir),
            'fast_mapping_launch_sha256': sha256_file(args.fast_mapping_launch),
            'fast_mapping_map_launch_sha256': sha256_file(
                args.fast_mapping_map_launch),
            'fast_official_config_sha256': sha256_file(args.fast_official_config),
            'fast_official_camera_config_sha256': sha256_file(
                args.fast_official_camera_config),
            'fast_image_identity_sha256': sha256_file(args.fast_image_identity),
        },
        'canonical_rosbag2': summary,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--slot', required=True)
    parser.add_argument('--sequence', required=True)
    parser.add_argument('--archive-expected-bytes', required=True, type=int)
    parser.add_argument('--archive-expected-md5', required=True)
    parser.add_argument('--rko-param', type=Path,
                        help='legacy fallback for both track-specific params')
    parser.add_argument('--rko-lio-param', type=Path)
    parser.add_argument('--rko-liv-param', type=Path)
    for name in ('official-archive', 'original-ros1-bag',
                 'normalized-ros1-bag', 'ros2-bag',
                 'normalization-report', 'semantic-report', 'reference',
                 'reference-metadata', 'calibration', 'candidate-manifest',
                 'lidarslam-param', 'glim-config-dir',
                 'fast-mapping-launch', 'fast-mapping-map-launch',
                 'fast-official-config', 'fast-official-camera-config',
                 'fast-image-identity', 'output'):
        parser.add_argument('--' + name, required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite frozen manifest: {args.output}')
    manifest = build_manifest(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
