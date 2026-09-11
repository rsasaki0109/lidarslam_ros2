#!/usr/bin/env python3
"""Score repeated dense trajectories on one explicit common GT subset."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import statistics
import subprocess
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tum_rows(path: Path) -> list[tuple[float, str]]:
    rows = []
    for raw in path.read_text(errors='replace').splitlines():
        line = raw.strip()
        if line and not line.startswith('#') and len(line.split()) >= 8:
            rows.append((float(line.split()[0]), line))
    return rows


def select_common_reference(reference: Path,
                            trajectories: list[Path]) -> tuple[list[str], list[float]]:
    bounds = []
    for trajectory in trajectories:
        rows = tum_rows(trajectory)
        if not rows:
            raise ValueError(f'{trajectory}: empty trajectory')
        bounds.append((rows[0][0], rows[-1][0]))
    first, last = max(item[0] for item in bounds), min(item[1] for item in bounds)
    selected, excluded = [], []
    for stamp, line in tum_rows(reference):
        (selected if first <= stamp <= last else excluded).append(
            line if first <= stamp <= last else stamp)
    if len(selected) < 3:
        raise ValueError(f'common reference has only {len(selected)} poses')
    return selected, excluded


def parse_ape(path: Path) -> dict[str, Any]:
    text = path.read_text()
    result: dict[str, Any] = {}
    for key in ('pairs', 'rmse', 'mean', 'median', 'std', 'min', 'max',
                'total_ref_points', 'rejected_ref_points', 'max_time_gap'):
        match = re.search(rf'^{key}:\s*(.+)$', text, re.MULTILINE)
        if match:
            result[key] = (int(match.group(1)) if key in (
                'pairs', 'total_ref_points', 'rejected_ref_points')
                           else float(match.group(1)))
    if 'rmse' not in result:
        raise ValueError(f'{path}: missing RMSE')
    return result


def peak_rss_mb(run_report: dict[str, Any]) -> float | None:
    runtime = run_report.get('runtime') or {}
    value = runtime.get('peak_rss_mb')
    if value is None:
        value = (runtime.get('mapper') or {}).get('peak_rss_mb')
    return None if value is None else float(value)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--benchmark-dir', type=Path, required=True)
    parser.add_argument('--reference-tum', type=Path, required=True)
    parser.add_argument('--trajectory-relative-path', required=True)
    parser.add_argument('--common-range-trajectory', type=Path,
                        action='append', default=[])
    parser.add_argument('--max-time-diff', type=float, default=0.25)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    run_dirs = sorted(path.parent for path in args.benchmark_dir.glob('run_*/run.json'))
    trajectories = [run / args.trajectory_relative_path for run in run_dirs]
    range_trajectories = trajectories + [
        path.resolve() for path in args.common_range_trajectory]
    selected, excluded = select_common_reference(
        args.reference_tum, range_trajectories)
    common_reference = args.benchmark_dir / 'common_reference.tum'
    common_reference.write_text('\n'.join(selected) + '\n')
    runs = []
    for run_dir, trajectory in zip(run_dirs, trajectories):
        ape_path = run_dir / 'ape_common_gt.txt'
        subprocess.run([
            sys.executable, str(ROOT / 'scripts/ape_from_tum.py'),
            '--ref', str(common_reference), '--est', str(trajectory),
            '--out', str(ape_path), '--interpolate',
            '--max-time-diff', str(args.max_time_diff)], check=True)
        run_report = json.loads((run_dir / 'run.json').read_text())
        completion = run_report['completion']
        trajectory_info = (run_report.get('scoring_trajectory') or
                           run_report.get('trajectory') or {})
        runs.append({
            'run': run_dir.name,
            'ape': parse_ape(ape_path),
            'completion': completion,
            'trajectory_complete': bool(completion['trajectory_complete']),
            'process_exit_status': completion.get('process_exit_status'),
            'trajectory_samples': int(trajectory_info.get('samples', 0)),
            'runtime': run_report['runtime'],
            'peak_rss_mb': peak_rss_mb(run_report),
        })
    apes = [run['ape']['rmse'] for run in runs]
    valid = all(run['completion']['trajectory_complete'] and
                run['completion']['process_exit_status'] == 0 and
                run['ape']['rejected_ref_points'] == 0 for run in runs)
    rss_values = [run['peak_rss_mb'] for run in runs
                  if run['peak_rss_mb'] is not None]
    document = {
        'schema_version': 1,
        'valid_repetitions': len(runs) if valid else 0,
        'reference': {
            'source_path': str(args.reference_tum.resolve()),
            'source_sha256': sha256(args.reference_tum),
            'common_path': str(common_reference.resolve()),
            'common_sha256': sha256(common_reference),
            'common_poses': len(selected),
            'excluded_timestamps': excluded,
            'range_contract_trajectories': [
                {'path': str(path.resolve()), 'sha256': sha256(path)}
                for path in range_trajectories],
        },
        'aggregate': {
            'ape_rmse_median_m': statistics.median(apes),
            'ape_rmse_min_m': min(apes),
            'ape_rmse_max_m': max(apes),
            'ape_rmse_population_std_m': statistics.pstdev(apes),
            'peak_rss_max_mb': max(rss_values) if rss_values else None,
        },
        'runs': runs,
    }
    output = args.output or args.benchmark_dir / 'scored_summary.json'
    output.write_text(json.dumps(document, indent=2) + '\n')
    print(json.dumps(document['aggregate'], indent=2))
    return 0 if valid else 2


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError,
            subprocess.CalledProcessError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
