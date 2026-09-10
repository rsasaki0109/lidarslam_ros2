#!/usr/bin/env python3
"""Freeze source/config hashes before opening competitive holdouts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import yaml

from lidarslam_benchmark_tools.competitive_candidate_provenance import git_revision, source_tree_digest


ROOT = Path(__file__).resolve().parents[1]


def file_record(path: Path) -> dict[str, str]:
    path = path.resolve()
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--rko-param', type=Path, required=True)
    parser.add_argument('--lidarslam-param', type=Path, required=True)
    args = parser.parse_args()
    for name in ('output', 'profile', 'rko_param', 'lidarslam_param'):
        setattr(args, name, getattr(args, name).resolve())

    params = yaml.safe_load(args.lidarslam_param.read_text())
    graph = params['graph_based_slam']['ros__parameters']
    manifest = {
        'schema_version': 1,
        'candidate_status': 'frozen_before_holdout',
        'repository_revision': git_revision(ROOT),
        'rko_lio_revision': git_revision(ROOT / 'Thirdparty/rko_lio'),
        'source_tree': source_tree_digest(ROOT),
        'benchmark_profile': file_record(args.profile),
        'rko_param': file_record(args.rko_param),
        'lidarslam_param': file_record(args.lidarslam_param),
        'map_refinement': {
            key: graph[key] for key in (
                'map_leaf_size', 'use_dynamic_object_filter',
                'use_planar_map_filter', 'planar_map_filter_voxel_size',
                'planar_map_filter_min_neighbors',
                'planar_map_filter_max_small_eigenvalue_ratio',
                'planar_map_filter_min_middle_eigenvalue_ratio',
                'planar_map_filter_min_retained_ratio')},
        'excluded_capabilities': [
            'saved_map_loading', 'localization_mode', 'relocalization'],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + '\n')
    print(json.dumps({
        'output': str(args.output),
        'source_tree_sha256': manifest['source_tree']['sha256'],
        'source_file_count': manifest['source_tree']['file_count']}, indent=2))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
