#!/usr/bin/env python3
"""Freeze repeated FAST-LIVO2 run artifacts into one scored summary."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import statistics
import sys
from typing import Any

import yaml

try:
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (
        current_rival_source_closure_identity)
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    from lidarslam_benchmark_tools.check_competitive_rival_source_closure import (  # type: ignore[no-redef]
        current_rival_source_closure_identity)


try:
    from lidarslam_benchmark_tools import package_root
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]


ROOT = package_root()
DEFAULT_PROFILE = ROOT / 'configs/slam_benchmark_profiles/competitive_slam_v1.yaml'


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def parse_ape(path: Path) -> dict[str, Any]:
    text = path.read_text()
    values: dict[str, Any] = {}
    for key in ('pairs', 'rmse', 'mean', 'median', 'std', 'min', 'max',
                'total_ref_points', 'rejected_ref_points', 'max_time_gap'):
        match = re.search(rf'^{key}:\s*(.+)$', text, re.MULTILINE)
        if not match:
            continue
        value = match.group(1).strip()
        values[key] = (int(value) if key in (
            'pairs', 'total_ref_points', 'rejected_ref_points') else float(value))
    alignment = re.search(r'^alignment:\s*(.+)$', text, re.MULTILINE)
    mode = re.search(r'^mode:\s*(.+)$', text, re.MULTILINE)
    values['alignment'] = alignment.group(1).strip() if alignment else None
    values['association_mode'] = mode.group(1).strip() if mode else None
    if 'rmse' not in values:
        raise ValueError(f'{path}: missing APE RMSE')
    return values


def summarize(benchmark_dir: Path, reference: Path) -> dict[str, Any]:
    run_paths = sorted(benchmark_dir.glob('run_*/run.json'))
    if not run_paths:
        raise ValueError(f'{benchmark_dir}: no run_*/run.json artifacts')
    profile = yaml.safe_load(DEFAULT_PROFILE.read_text(encoding='utf-8'))
    expected_closure = current_rival_source_closure_identity(
        profile, root=ROOT)
    runs = []
    bag_hashes, revisions, image_ids = set(), set(), set()
    closure_identities: list[dict[str, Any]] = []
    for run_path in run_paths:
        report = json.loads(run_path.read_text())
        ape = parse_ape(run_path.parent / 'ape_vs_gt.txt')
        provenance = report['provenance']
        observed_closure = provenance.get('rival_source_closure')
        if observed_closure != expected_closure:
            raise ValueError(
                'run provenance rival source closure is missing or differs '
                'from the current r2 closure')
        closure_identities.append(observed_closure)
        bag_hashes.add(provenance['bag_sha256'])
        revisions.add(provenance['source']['revision'])
        image_ids.add(provenance['container_image_id'])
        runs.append({
            'run': run_path.parent.name,
            'ape': ape,
            'trajectory_complete': report['completion']['trajectory_complete'],
            'process_exit_status': report['completion']['process_exit_status'],
            'trajectory_samples': report['trajectory']['samples'],
            'trajectory_end_gap_seconds': report['completion'][
                'trajectory_end_gap_seconds'],
            'replay_wall_rtf': report['runtime'].get(
                'replay_wall_realtime_factor', report['runtime'].get('realtime_factor')),
            'processing_rtf_upper_bound': report['runtime'].get(
                'processing_realtime_factor_upper_bound'),
            'peak_rss_mb': report['runtime']['mapper'].get('peak_rss_mb'),
        })
    if any(len(values) != 1 for values in (bag_hashes, revisions, image_ids)):
        raise ValueError('run provenance differs across repetitions')
    if any(identity != expected_closure for identity in closure_identities):
        raise ValueError('run provenance differs across repetitions')
    ape_values = [run['ape']['rmse'] for run in runs]
    rtf_values = [run['replay_wall_rtf'] for run in runs]
    rss_values = [run['peak_rss_mb'] for run in runs]
    processing_bounds = [run['processing_rtf_upper_bound'] for run in runs
                         if run['processing_rtf_upper_bound'] is not None]
    valid = all(run['trajectory_complete'] and run['process_exit_status'] == 0
                for run in runs)
    return {
        'schema_version': 1,
        'system': 'fast_livo2',
        'valid_repetitions': len(runs) if valid else 0,
        'provenance': {
            'source_revision': next(iter(revisions)),
            'container_image_id': next(iter(image_ids)),
            'bag_sha256': next(iter(bag_hashes)),
            'reference_path': str(reference.resolve()),
            'reference_sha256': sha256(reference),
            'rival_source_closure': expected_closure,
        },
        'aggregate': {
            'ape_rmse_median_m': statistics.median(ape_values),
            'ape_rmse_min_m': min(ape_values),
            'ape_rmse_max_m': max(ape_values),
            'ape_rmse_population_std_m': statistics.pstdev(ape_values),
            'replay_wall_rtf_median': statistics.median(rtf_values),
            'replay_wall_rtf_max': max(rtf_values),
            'processing_rtf_upper_bound_median': (
                statistics.median(processing_bounds) if processing_bounds else None),
            'processing_rtf_upper_bound_max': (
                max(processing_bounds) if processing_bounds else None),
            'peak_rss_max_mb': max(rss_values),
        },
        'runs': runs,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--benchmark-dir', type=Path, required=True)
    parser.add_argument('--reference-tum', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    output = args.output or args.benchmark_dir / 'scored_summary.json'
    document = summarize(args.benchmark_dir, args.reference_tum)
    output.write_text(json.dumps(document, indent=2) + '\n')
    print(json.dumps(document['aggregate'], indent=2))
    return 0 if document['valid_repetitions'] == len(document['runs']) else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
