#!/usr/bin/env python3
"""Freeze a complete, internally consistent NTU-VIRAL holdout input suite."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import yaml


CONSISTENT_HASHES = (
    'candidate_manifest_sha256', 'calibration_sha256',
    'rko_lio_param_sha256', 'rko_liv_param_sha256',
    'lidarslam_param_sha256', 'glim_config_tree_sha256',
    'fast_mapping_launch_sha256', 'fast_mapping_map_launch_sha256',
    'fast_official_config_sha256', 'fast_official_camera_config_sha256',
    'fast_image_identity_sha256')


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def freeze(profile_path: Path, preregistration_path: Path,
           manifest_paths: list[Path]) -> dict[str, Any]:
    profile = yaml.safe_load(profile_path.read_text())['competitive_slam_profile']
    slots = profile['datasets']['holdout_slots']
    preregistration_sha = sha256(preregistration_path)
    if preregistration_sha != profile['holdout_selection_preregistration_sha256']:
        raise ValueError('preregistration hash differs from profile')
    documents = [json.loads(path.read_text()) for path in manifest_paths]
    if len(documents) != len(slots):
        raise ValueError('one input manifest per holdout slot is required')
    by_slot = {document.get('slot'): (path, document)
               for path, document in zip(manifest_paths, documents)}
    if set(by_slot) != set(slots) or len(by_slot) != len(documents):
        raise ValueError('input manifest slots do not exactly match profile slots')

    rows = {}
    shared: dict[str, str] = {}
    for slot_name, expected in slots.items():
        path, document = by_slot[slot_name]
        hashes = document.get('hashes') or {}
        if document.get('status') != 'frozen':
            raise ValueError(f'{slot_name}: input manifest is not frozen')
        if document.get('sequence') != expected['sequence']:
            raise ValueError(f'{slot_name}: sequence differs from profile')
        if hashes.get('official_archive_bytes') != expected['archive_expected_bytes']:
            raise ValueError(f'{slot_name}: archive size differs from profile')
        if hashes.get('official_archive_md5') != expected['archive_expected_md5']:
            raise ValueError(f'{slot_name}: archive MD5 differs from profile')
        if hashes.get('reference_sha256') != expected['ground_truth_sha256']:
            raise ValueError(f'{slot_name}: reference differs from profile')
        for key in CONSISTENT_HASHES:
            value = hashes.get(key)
            if not isinstance(value, str) or len(value) != 64:
                raise ValueError(f'{slot_name}: missing valid {key}')
            if key in shared and shared[key] != value:
                raise ValueError(f'{slot_name}: inconsistent shared hash {key}')
            shared[key] = value
        rows[slot_name] = {
            'sequence': document['sequence'],
            'input_manifest_path': str(path.resolve()),
            'input_manifest_sha256': sha256(path),
            'raw_rosbag1_sha256': hashes['raw_rosbag1_sha256'],
            'canonical_rosbag2_tree_sha256': hashes[
                'canonical_rosbag2_tree_sha256'],
            'semantic_equivalence_sha256': hashes['semantic_report_sha256'],
            'reference_sha256': hashes['reference_sha256'],
        }
    return {
        'schema_version': 1,
        'status': 'frozen',
        'profile_path': str(profile_path.resolve()),
        'profile_sha256': sha256(profile_path),
        'preregistration_path': str(preregistration_path.resolve()),
        'preregistration_sha256': preregistration_sha,
        'shared_hashes': shared,
        'holdouts': rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--preregistration', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, action='append', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite frozen suite: {args.output}')
    document = freeze(args.profile, args.preregistration, args.manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2, sort_keys=True) + '\n')
    print(json.dumps(document, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError,
            yaml.YAMLError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
