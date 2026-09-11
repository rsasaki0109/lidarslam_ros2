#!/usr/bin/env python3
"""Emit a report-only scalar shadow observation from a validated weak-axis join.

The output is deliberately not a ROS message and is never sent to Voxel.  It
is the producer-side contract for the isolated receiver added by v38:
``stamp_sec`` in seconds, signed ``velocity_mps`` in metres per second, and
normalized ``confidence`` in ``[0, 1]``.  The input join must already have
passed the weak-axis feasibility gate.  No trajectory, reference, or
ground-truth file is read.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
from typing import Any


def load_ground_truth_free_report(path: Path) -> dict[str, Any]:
    """Load a JSON report and require its explicit ground-truth-free marker."""
    document = json.loads(path.read_text(encoding='utf-8'))
    if document.get('accuracy_ground_truth_accessed') is not False:
        raise ValueError(f'report is not ground-truth-free: {path}')
    return document


def _finite_float(value: Any, field: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{field} must be finite')
    return result


def _clamp(value: float, low: float = 0.0, high: float = 1.0) -> float:
    return min(high, max(low, value))


def confidence_components(
        observation: dict[str, Any], visual_config: dict[str, Any],
        join_report: dict[str, Any],) -> dict[str, float]:
    """Return conservative normalized quality components for one observation.

    The confidence is a minimum, rather than an average, so a weak link in
    inliers, residual, synchronization, or axis separation remains visible
    to the receiver.  These components are provenance metadata, not a new
    accuracy metric.
    """
    weak_definition = join_report.get('weak_axis_definition', {})
    min_inliers = max(_finite_float(
        visual_config.get('min_inliers', 15), 'min_inliers'), 1.0)
    max_residual = max(_finite_float(
        visual_config.get('max_residual_norm', 0.02), 'max_residual_norm'),
        1.0e-12)
    max_join = max(_finite_float(
        weak_definition.get('max_join_sec', 0.08), 'max_join_sec'), 1.0e-12)
    eigen_ratio_max = max(_finite_float(
        weak_definition.get('eigen_ratio_max', 0.2), 'eigen_ratio_max'),
        1.0e-12)

    inliers = _finite_float(observation.get('median_inliers', 0.0), 'median_inliers')
    residual = _finite_float(
        observation.get('median_residual_norm', max_residual),
        'median_residual_norm')
    join_error = _finite_float(
        observation.get('join_error_sec', max_join), 'join_error_sec')
    eigen_ratio = _finite_float(
        observation.get('eigen_ratio', eigen_ratio_max), 'eigen_ratio')
    horizontal_norm = _finite_float(
        observation.get('weak_horizontal_norm', 0.0), 'weak_horizontal_norm')

    return {
        'inlier_support': _clamp(inliers / (2.0 * min_inliers)),
        'residual_margin': _clamp(1.0 - residual / max_residual),
        'sync_margin': _clamp(1.0 - join_error / max_join),
        'weak_axis_norm': _clamp(horizontal_norm),
        'weak_axis_separation': _clamp(
            (eigen_ratio_max - eigen_ratio) / eigen_ratio_max),
    }


def build_shadow_document(
        join_report: dict[str, Any], visual_report: dict[str, Any],
        join_report_path: Path, visual_report_path: Path,
        max_age_sec: float = 0.2, max_speed_mps: float = 20.0,
        min_confidence: float = 0.2, require_go: bool = True,
        ) -> dict[str, Any]:
    """Convert accepted weak-axis scan observations into the source contract."""
    if require_go and join_report.get('decision') != 'GO_WEAK_AXIS_PROJECTION':
        raise ValueError(
            'weak-axis join did not pass: '
            f"{join_report.get('decision', '<missing>')}")
    if not isinstance(join_report.get('observations'), list):
        raise ValueError('weak-axis join has no detailed observations')
    if not isinstance(visual_report.get('config'), dict):
        raise ValueError('visual pair report has no config')
    if max_age_sec <= 0.0 or max_speed_mps <= 0.0:
        raise ValueError('receiver age and speed bounds must be positive')
    if not 0.0 <= min_confidence <= 1.0:
        raise ValueError('minimum confidence must be in [0,1]')

    visual_config = visual_report['config']
    observations: list[dict[str, Any]] = []
    rejected: Counter[str] = Counter()
    for source_observation in join_report['observations']:
        if not isinstance(source_observation, dict):
            rejected['observation_not_object'] += 1
            continue
        try:
            stamp_sec = _finite_float(
                source_observation.get('stamp_sec'), 'stamp_sec')
            velocity_mps = _finite_float(
                source_observation.get('velocity_mps'), 'velocity_mps')
            pair_count = int(source_observation.get('pair_count', 0))
            if pair_count < 1:
                raise ValueError('pair_count must be positive')
            if abs(velocity_mps) > max_speed_mps:
                rejected['speed_bound'] += 1
                continue
            components = confidence_components(
                source_observation, visual_config, join_report)
            confidence = min(components.values())
            if confidence < min_confidence:
                rejected['confidence_bound'] += 1
                continue
        except (TypeError, ValueError, KeyError):
            rejected['invalid_observation'] += 1
            continue
        observations.append({
            'stamp_sec': stamp_sec,
            'velocity_mps': velocity_mps,
            'confidence': confidence,
            'source': {
                'weak_row_index': int(source_observation['weak_row_index']),
                'pair_count': pair_count,
                'pair_indices': [
                    int(index) for index in source_observation.get(
                        'pair_indices', [])
                ],
                'join_error_sec': float(source_observation['join_error_sec']),
                'projection_abs_mps': abs(velocity_mps),
                'confidence_components': components,
            },
        })

    return {
        'schema_version': 1,
        'status': 'report_only_visual_weak_axis_shadow_source',
        'accuracy_ground_truth_accessed': False,
        'decision': (
            'GO_REPORT_ONLY_SHADOW_SOURCE' if observations
            else 'NO_GO_REPORT_ONLY_SHADOW_SOURCE'),
        'contract': {
            'receiver_method': 'ingest_visual_longitudinal_shadow',
            'stamp_field': 'stamp_sec',
            'stamp_unit': 's',
            'velocity_field': 'velocity_mps',
            'velocity_unit': 'm/s',
            'confidence_field': 'confidence',
            'confidence_range': [0.0, 1.0],
            'max_age_sec': max_age_sec,
            'max_speed_mps': max_speed_mps,
            'estimator_state_mutated': False,
            'ros_topic_published': False,
        },
        'inputs': {
            'weak_axis_join': str(join_report_path.resolve()),
            'visual_pairs': str(visual_report_path.resolve()),
            'weak_axis_decision': join_report.get('decision'),
        },
        'counts': {
            'join_observations': len(join_report['observations']),
            'emitted_observations': len(observations),
            'rejected_observations': sum(rejected.values()),
        },
        'rejections': dict(sorted(rejected.items())),
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
    parser.add_argument('--output', required=True, type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise ValueError(f'refusing to overwrite: {args.output}')
    join_report = load_ground_truth_free_report(args.weak_axis_join)
    visual_report = load_ground_truth_free_report(args.visual_pairs)
    report = build_shadow_document(
        join_report, visual_report, args.weak_axis_join, args.visual_pairs,
        args.max_age_sec, args.max_speed_mps, args.min_confidence,
        require_go=not args.allow_no_go)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(report, indent=2, sort_keys=True) + '\n'
    args.output.write_text(encoded, encoding='utf-8')
    print(encoded, end='')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
