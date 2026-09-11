#!/usr/bin/env python3
"""Validate and freeze one RTK-SLAM competitive holdout input bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import yaml


REQUIRED_ROS2_TOPICS = {
    '/camera/image_raw/compressed',
    '/livox/imu',
    '/livox/lidar',
    '/livox/points',
}
SEMANTIC_TOPICS = {
    '/camera/image_raw/compressed',
    '/livox/imu',
    '/livox/lidar',
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
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


def metadata_summary(path: Path) -> dict[str, Any]:
    document = yaml.safe_load(path.read_text(encoding='utf-8'))
    info = document['rosbag2_bagfile_information']
    topics = sorted(
        row['topic_metadata']['name']
        for row in info['topics_with_message_count'])
    return {
        'storage_identifier': info['storage_identifier'],
        'duration_nanoseconds': info['duration']['nanoseconds'],
        'starting_time_nanoseconds': info['starting_time']['nanoseconds_since_epoch'],
        'message_count': info['message_count'],
        'topics': topics,
    }


def checked_file(path: Path, label: str) -> Path:
    if not path.is_file():
        raise ValueError(f'{label} is missing: {path}')
    return path


def checked_directory(path: Path, label: str) -> Path:
    if not path.is_dir():
        raise ValueError(f'{label} is missing: {path}')
    return path


def validate_semantic_report(path: Path) -> dict[str, Any]:
    report = json.loads(path.read_text(encoding='utf-8'))
    if report.get('all_topics_equal') is not True:
        raise ValueError('ROS1/ROS2 semantic comparison did not pass')
    topics = {row['topic'] for row in report.get('topics', [])
              if row.get('equal') is True}
    missing = SEMANTIC_TOPICS.difference(topics)
    if missing:
        raise ValueError(
            f'semantic comparison lacks equal required topics: {sorted(missing)}')
    return report


def build_manifest(args: argparse.Namespace) -> dict[str, Any]:
    ros2_bag = checked_directory(args.ros2_bag, 'canonical ROS2 bag')
    metadata = checked_file(ros2_bag / 'metadata.yaml', 'ROS2 metadata')
    ros1_bag = checked_file(args.ros1_bag, 'derived ROS1 bag')
    required_files = {
        'reference_sha256': args.reference,
        'reference_metadata_sha256': args.reference_metadata,
        'calibration_sha256': args.calibration,
        'semantic_report_sha256': args.semantic_report,
        'candidate_manifest_sha256': args.candidate_manifest,
        'rko_param_sha256': args.rko_param,
        'lidarslam_param_sha256': args.lidarslam_param,
        'fast_livo2_mapping_launch_sha256': args.fast_mapping_launch,
        'fast_livo2_mapping_map_launch_sha256': args.fast_mapping_map_launch,
        'fast_livo2_config_sha256': args.fast_config,
        'fast_livo2_camera_config_sha256': args.fast_camera_config,
        'fast_livo2_image_identity_sha256': args.fast_image_identity,
    }
    for label, path in required_files.items():
        checked_file(path, label)
    checked_directory(args.glim_config_dir, 'GLIM config directory')

    summary = metadata_summary(metadata)
    missing = REQUIRED_ROS2_TOPICS.difference(summary['topics'])
    if missing:
        raise ValueError(f'canonical ROS2 bag lacks topics: {sorted(missing)}')
    semantic = validate_semantic_report(args.semantic_report)
    candidate = json.loads(args.candidate_manifest.read_text(encoding='utf-8'))
    if candidate.get('candidate_status') != 'frozen_before_holdout':
        raise ValueError('candidate manifest is not frozen_before_holdout')

    hashes = {
        key: sha256_file(path) for key, path in required_files.items()
    }
    hashes.update({
        # Runner compatibility: this legacy key names the representation,
        # not its provenance. RTK-SLAM starts as ROS2; this is a verified,
        # topic-filtered ROS1 derivative for FAST-LIVO2 only.
        'raw_rosbag1_sha256': sha256_file(ros1_bag),
        'canonical_rosbag2_tree_sha256': sha256_tree(ros2_bag),
        'glim_config_tree_sha256': sha256_tree(args.glim_config_dir),
    })
    return {
        'schema_version': 1,
        'slot': args.slot,
        'sequence': args.sequence,
        'dataset': 'RTK-SLAM',
        'status': 'frozen',
        'candidate': {
            'source_tree_sha256': candidate['source_tree']['sha256'],
            'repository_revision': candidate['repository_revision'],
        },
        'representations': {
            'canonical': 'downloaded ROS2 bag',
            'fast_livo2': (
                'topic-filtered ROS1 derivative; equality proven by semantic_report'),
            'semantic_topics': sorted(SEMANTIC_TOPICS),
            'semantic_all_topics_equal': semantic['all_topics_equal'],
        },
        'hashes': hashes,
        'canonical_rosbag2': summary,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--slot', required=True)
    parser.add_argument('--sequence', required=True)
    parser.add_argument('--ros2-bag', required=True, type=Path)
    parser.add_argument('--ros1-bag', required=True, type=Path)
    parser.add_argument('--reference', required=True, type=Path)
    parser.add_argument('--reference-metadata', required=True, type=Path)
    parser.add_argument('--calibration', required=True, type=Path)
    parser.add_argument('--semantic-report', required=True, type=Path)
    parser.add_argument('--candidate-manifest', required=True, type=Path)
    parser.add_argument('--rko-param', required=True, type=Path)
    parser.add_argument('--lidarslam-param', required=True, type=Path)
    parser.add_argument('--glim-config-dir', required=True, type=Path)
    parser.add_argument('--fast-mapping-launch', required=True, type=Path)
    parser.add_argument('--fast-mapping-map-launch', required=True, type=Path)
    parser.add_argument('--fast-config', required=True, type=Path)
    parser.add_argument('--fast-camera-config', required=True, type=Path)
    parser.add_argument('--fast-image-identity', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite frozen manifest: {args.output}')
    manifest = build_manifest(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n',
                           encoding='utf-8')
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
