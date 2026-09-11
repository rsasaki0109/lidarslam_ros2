#!/usr/bin/env python3
"""Compare two dense trajectories at identical sparse reference checkpoints."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import sys
from typing import Any

from lidarslam_benchmark_tools.ape_from_tum import (
    calc_errors, interpolate_association, read_tum, try_align_umeyama)


def evaluate(reference_path: Path, ours_path: Path, rival_path: Path,
             max_edge_diff: float) -> dict[str, Any]:
    reference = read_tum(reference_path)
    ours = read_tum(ours_path)
    rival = read_tum(rival_path)
    if not reference or not ours or not rival:
        raise ValueError('reference and both trajectories must contain poses')
    ref_ours, sampled_ours, ours_diag = interpolate_association(
        reference, ours, max_edge_diff)
    ref_rival, sampled_rival, rival_diag = interpolate_association(
        reference, rival, max_edge_diff)
    if ours_diag['rejected_ref_points'] or rival_diag['rejected_ref_points']:
        raise ValueError('a trajectory does not cover every reference checkpoint')
    if ref_ours != ref_rival or len(ref_ours) != len(reference):
        raise ValueError('trajectory checkpoint associations differ')
    ours_alignment, aligned_ours = try_align_umeyama(ref_ours, sampled_ours)
    rival_alignment, aligned_rival = try_align_umeyama(ref_rival, sampled_rival)
    ours_errors = calc_errors(ref_ours, aligned_ours)
    rival_errors = calc_errors(ref_rival, aligned_rival)
    rows = []
    for index, ((stamp, ref_xyz), ours_error, rival_error) in enumerate(
            zip(reference, ours_errors, rival_errors)):
        rows.append({
            'index': index, 'timestamp': stamp, 'reference_xyz': list(ref_xyz),
            'ours_error_m': ours_error, 'rival_error_m': rival_error,
            'ours_minus_rival_m': ours_error - rival_error,
            'winner': ('ours' if ours_error < rival_error else
                       'rival' if rival_error < ours_error else 'tie'),
        })
    def rmse(values: list[float]) -> float:
        return math.sqrt(statistics.fmean(value * value for value in values))
    gaps = [row['ours_minus_rival_m'] for row in rows]
    return {
        'schema_version': 1,
        'reference': str(reference_path.resolve()),
        'ours': str(ours_path.resolve()), 'rival': str(rival_path.resolve()),
        'checkpoint_count': len(rows),
        'alignment': {'ours': ours_alignment, 'rival': rival_alignment},
        'aggregate': {
            'ours_ape_rmse_m': rmse(ours_errors),
            'rival_ape_rmse_m': rmse(rival_errors),
            'ours_wins': sum(row['winner'] == 'ours' for row in rows),
            'rival_wins': sum(row['winner'] == 'rival' for row in rows),
            'largest_ours_deficit_m': max(gaps),
            'largest_ours_deficit_checkpoint': max(
                range(len(gaps)), key=gaps.__getitem__),
        },
        'diagnostics': {'ours': ours_diag, 'rival': rival_diag},
        'checkpoints': rows,
    }


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=[
            'index', 'timestamp', 'ours_error_m', 'rival_error_m',
            'ours_minus_rival_m', 'winner'])
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row[key] for key in writer.fieldnames})


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--ours', type=Path, required=True)
    parser.add_argument('--rival', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-edge-diff', type=float, default=3.0)
    args = parser.parse_args()
    result = evaluate(args.reference, args.ours, args.rival, args.max_edge_diff)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n')
    write_csv(args.output.with_suffix('.csv'), result['checkpoints'])
    print(json.dumps(result['aggregate'], indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        sys.exit(2)
