#!/usr/bin/env python3
"""Audit the rejected v38 visual-shadow runtime without opening ground truth.

The audit compares a behavior-preserving baseline trajectory with a candidate
trajectory, locates divergence after the first visual observation, and checks
the candidate source contract for four feedback hazards.  It reads no
reference trajectory and reports no accuracy metric.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
from typing import Any


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_tum(path: Path) -> list[tuple[float, tuple[float, float, float]]]:
    rows: list[tuple[float, tuple[float, float, float]]] = []
    previous = -math.inf
    for line_number, line in enumerate(
            path.read_text(encoding='utf-8', errors='replace').splitlines(), 1):
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        fields = line.split()
        if len(fields) != 8:
            raise ValueError(f'{path}:{line_number}: invalid TUM row')
        values = [float(value) for value in fields]
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f'{path}:{line_number}: non-finite TUM value')
        if values[0] <= previous:
            raise ValueError(f'{path}:{line_number}: timestamps not increasing')
        previous = values[0]
        rows.append((values[0], (values[1], values[2], values[3])))
    if not rows:
        raise ValueError(f'{path}: empty trajectory')
    return rows


def path_length(rows: list[tuple[float, tuple[float, float, float]]]) -> float:
    return sum(
        math.dist(left[1], right[1]) for left, right in zip(rows, rows[1:]))


def matched_position_deltas(
        baseline: list[tuple[float, tuple[float, float, float]]],
        candidate: list[tuple[float, tuple[float, float, float]]],
        max_time_delta_sec: float,
        ) -> list[tuple[float, float, float]]:
    if max_time_delta_sec <= 0.0:
        raise ValueError('maximum time delta must be positive')
    baseline_times = [row[0] for row in baseline]
    result: list[tuple[float, float, float]] = []
    for stamp, position in candidate:
        insertion = bisect_left(baseline_times, stamp)
        candidates = [
            index for index in (insertion - 1, insertion)
            if 0 <= index < len(baseline)]
        if not candidates:
            continue
        index = min(candidates, key=lambda item: abs(
            baseline_times[item] - stamp))
        delta_sec = abs(baseline_times[index] - stamp)
        if delta_sec <= max_time_delta_sec:
            result.append((
                stamp, math.dist(position, baseline[index][1]), delta_sec))
    if not result:
        raise ValueError('baseline and candidate have no matched poses')
    return result


def source_contract(source: str) -> dict[str, bool]:
    """Return explicit source properties; false values are contract failures."""
    return {
        'state_timestamp_controls_application': (
            'apply_visual_longitudinal_shadow(' in source and
            'state_stamp_sec' in source),
        'observation_consumed_at_most_once': (
            'last_consumed_visual' in source or
            'visual_observation_queue.pop_front()' in source),
        'candidate_axis_projection_uses_vector_observation': (
            'velocity_base_mps' in source or
            'linear_velocity_base' in source),
        'mapper_state_is_not_feedback_target': not (
            'weak_axis_speed += correction' in source),
    }


def build_audit(
        baseline_path: Path, candidate_path: Path, payload_path: Path,
        source_patch_path: Path, max_time_delta_sec: float = 0.06,
        ) -> dict[str, Any]:
    baseline = load_tum(baseline_path)
    candidate = load_tum(candidate_path)
    payload = json.loads(payload_path.read_text(encoding='utf-8'))
    if payload.get('accuracy_ground_truth_accessed') is not False:
        raise ValueError('payload is not explicitly ground-truth-free')
    observations = payload.get('observations')
    if not isinstance(observations, list) or not observations:
        raise ValueError('payload has no observations')
    first_observation = float(observations[0]['stamp_sec'])
    if not math.isfinite(first_observation):
        raise ValueError('first observation timestamp is not finite')

    deltas = matched_position_deltas(
        baseline, candidate, max_time_delta_sec)
    after = [row for row in deltas if row[0] >= first_observation]
    if not after:
        raise ValueError('candidate ends before the first observation')
    thresholds: dict[str, Any] = {}
    for threshold in (0.01, 0.1, 1.0, 10.0, 100.0, 500.0):
        crossing = next((row for row in after if row[1] >= threshold), None)
        thresholds[f'{threshold:g}_m'] = None if crossing is None else {
            'stamp_sec': crossing[0],
            'seconds_after_first_observation': crossing[0] - first_observation,
            'position_delta_m': crossing[1],
        }
    maximum = max(after, key=lambda row: row[1])
    contract = source_contract(
        source_patch_path.read_text(encoding='utf-8', errors='replace'))
    failed_contracts = sorted(
        key for key, passed in contract.items() if not passed)
    return {
        'schema_version': 1,
        'status': 'ground_truth_free_visual_shadow_runtime_audit',
        'accuracy_ground_truth_accessed': False,
        'decision': (
            'FAIL_VISUAL_SHADOW_RUNTIME_CONTRACT'
            if failed_contracts or maximum[1] >= 1.0
            else 'PASS_VISUAL_SHADOW_RUNTIME_CONTRACT'),
        'inputs': {
            'baseline_tum': str(baseline_path.resolve()),
            'baseline_sha256': sha256(baseline_path),
            'candidate_tum': str(candidate_path.resolve()),
            'candidate_sha256': sha256(candidate_path),
            'payload': str(payload_path.resolve()),
            'payload_sha256': sha256(payload_path),
            'source_patch': str(source_patch_path.resolve()),
            'source_patch_sha256': sha256(source_patch_path),
        },
        'trajectory': {
            'matched_poses': len(deltas),
            'max_match_delta_sec': max(row[2] for row in deltas),
            'baseline_path_length_m': path_length(baseline),
            'candidate_path_length_m': path_length(candidate),
            'first_observation_stamp_sec': first_observation,
            'first_threshold_crossings': thresholds,
            'maximum_position_delta_after_first_observation_m': maximum[1],
            'maximum_position_delta_stamp_sec': maximum[0],
        },
        'source_contract': contract,
        'failed_source_contracts': failed_contracts,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-tum', required=True, type=Path)
    parser.add_argument('--candidate-tum', required=True, type=Path)
    parser.add_argument('--payload', required=True, type=Path)
    parser.add_argument('--source-patch', required=True, type=Path)
    parser.add_argument('--max-time-delta-sec', type=float, default=0.06)
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    report = build_audit(
        args.baseline_tum, args.candidate_tum, args.payload,
        args.source_patch, args.max_time_delta_sec)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(report, indent=2, sort_keys=True) + '\n'
    args.output.write_text(encoded, encoding='utf-8')
    print(encoded, end='')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError,
            json.JSONDecodeError) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
