#!/usr/bin/env python3
"""Emit a ground-truth-free base-frame visual velocity-vector contract.

Unlike the rejected v38 scalar payload, this report keeps the calibrated
camera-derived velocity vector in the base frame.  A later output-only
consumer projects it against its own timestamped weak axis; no mapper or
inertial state is mutated here.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics
from typing import Any

from emit_visual_weak_axis_shadow import (
    build_shadow_document, load_ground_truth_free_report)


def _pair_velocity_base(pair: dict[str, Any]) -> tuple[float, float, float]:
    if not bool(pair.get('valid')):
        raise ValueError('referenced visual pair is not valid')
    direction = pair.get('direction_base')
    if not isinstance(direction, list) or len(direction) != 3:
        raise ValueError('visual pair has no three-vector direction_base')
    speed = float(pair.get('speed_mps'))
    values = tuple(float(value) * speed for value in direction)
    if not math.isfinite(speed) or not all(math.isfinite(v) for v in values):
        raise ValueError('visual pair velocity is not finite')
    return values


def build_vector_document(
        join_report: dict[str, Any], visual_report: dict[str, Any],
        join_report_path: Path, visual_report_path: Path,
        max_age_sec: float = 0.2, max_speed_mps: float = 20.0,
        min_confidence: float = 0.2, require_go: bool = True,
        ) -> dict[str, Any]:
    join_decision = join_report.get('decision')
    if join_decision != 'GO_WEAK_AXIS_PROJECTION':
        if require_go:
            raise ValueError(f'weak-axis join did not pass: {join_decision}')
        return {
            'schema_version': 1,
            'status': 'report_only_visual_velocity_vector_shadow_source',
            'accuracy_ground_truth_accessed': False,
            'decision': 'NO_GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE',
            'contract': {
                'velocity_field': 'velocity_base_mps',
                'velocity_frame': 'base',
                'velocity_unit': 'm/s',
                'axis_projection_at_consumer': True,
                'mapper_state_mutated': False,
            },
            'inputs': {
                'weak_axis_join': str(join_report_path.resolve()),
                'visual_pairs': str(visual_report_path.resolve()),
                'weak_axis_decision': join_decision,
            },
            'counts': {'join_observations': len(
                join_report.get('observations') or []),
                'emitted_observations': 0},
            'observations': [],
        }
    pairs = visual_report.get('pairs')
    if not isinstance(pairs, list):
        raise ValueError('visual pair report has no pairs list')

    scalar = build_shadow_document(
        join_report, visual_report, join_report_path, visual_report_path,
        max_age_sec, max_speed_mps, min_confidence, require_go=True)
    observations: list[dict[str, Any]] = []
    invalid_vectors = 0
    for scalar_observation in scalar['observations']:
        try:
            pair_indices = scalar_observation['source']['pair_indices']
            vectors = [_pair_velocity_base(pairs[int(index)])
                       for index in pair_indices]
            if not vectors:
                raise ValueError('observation has no visual pair vectors')
            vector = tuple(statistics.median(component)
                           for component in zip(*vectors))
            norm = math.sqrt(sum(value * value for value in vector))
            if not math.isfinite(norm) or norm > max_speed_mps:
                raise ValueError('base-frame velocity exceeds speed bound')
        except (IndexError, KeyError, TypeError, ValueError):
            invalid_vectors += 1
            continue
        observations.append({
            'stamp_sec': float(scalar_observation['stamp_sec']),
            'velocity_base_mps': list(vector),
            'confidence': float(scalar_observation['confidence']),
            'source': {
                **scalar_observation['source'],
                'v38_reference_projection_mps': float(
                    scalar_observation['velocity_mps']),
            },
        })

    return {
        'schema_version': 1,
        'status': 'report_only_visual_velocity_vector_shadow_source',
        'accuracy_ground_truth_accessed': False,
        'decision': (
            'GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE' if observations
            else 'NO_GO_REPORT_ONLY_VECTOR_SHADOW_SOURCE'),
        'contract': {
            'stamp_field': 'stamp_sec',
            'stamp_unit': 's',
            'velocity_field': 'velocity_base_mps',
            'velocity_frame': 'base',
            'velocity_unit': 'm/s',
            'confidence_field': 'confidence',
            'confidence_range': [0.0, 1.0],
            'max_age_sec': max_age_sec,
            'max_speed_mps': max_speed_mps,
            'axis_projection_at_consumer': True,
            'mapper_state_mutated': False,
            'ros_topic_published': False,
        },
        'inputs': {
            'weak_axis_join': str(join_report_path.resolve()),
            'visual_pairs': str(visual_report_path.resolve()),
            'weak_axis_decision': join_decision,
        },
        'gravity_axis_world': list(
            join_report.get('quality', {}).get(
                'gravity_axis_world', [0.0, 0.0, 1.0])),
        'counts': {
            'join_observations': len(join_report['observations']),
            'scalar_contract_observations': len(scalar['observations']),
            'emitted_observations': len(observations),
            'invalid_vector_observations': invalid_vectors,
        },
        'observations': observations,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weak-axis-join', required=True, type=Path)
    parser.add_argument('--visual-pairs', required=True, type=Path)
    parser.add_argument('--max-age-sec', type=float, default=0.2)
    parser.add_argument('--max-speed-mps', type=float, default=20.0)
    parser.add_argument('--min-confidence', type=float, default=0.2)
    parser.add_argument('--allow-no-go', action='store_true')
    parser.add_argument(
        '--print-document', action='store_true',
        help='print the full observation document instead of a short summary')
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args()


def terminal_summary(report: dict[str, Any], output: Path) -> dict[str, Any]:
    counts = report.get('counts') or {}
    return {
        'decision': report.get('decision'),
        'emitted_observations': int(counts.get('emitted_observations', 0)),
        'invalid_vector_observations': int(
            counts.get('invalid_vector_observations', 0)),
        'output': str(output.resolve()),
    }


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    join_report = load_ground_truth_free_report(args.weak_axis_join)
    visual_report = load_ground_truth_free_report(args.visual_pairs)
    report = build_vector_document(
        join_report, visual_report, args.weak_axis_join, args.visual_pairs,
        args.max_age_sec, args.max_speed_mps, args.min_confidence,
        require_go=not args.allow_no_go)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(report, indent=2, sort_keys=True) + '\n'
    args.output.write_text(encoded, encoding='utf-8')
    if args.print_document:
        print(encoded, end='')
    else:
        print(json.dumps(
            terminal_summary(report, args.output), indent=2,
            sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError,
            json.JSONDecodeError) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
