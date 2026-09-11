#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""Gate trajectory, geometry, calibration, and held-out colour reports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml


METRICS = {
    'ape_rmse_max_m': ('trajectory', ('evo', 'ape', 'rmse'), 'max'),
    'thickness_rms_mean_max_m': (
        'geometry', ('map_quality_report', 'plane_metrics',
                     'thickness_rms_mean_m'), 'max'),
    'thickness_rms_p95_max_m': (
        'geometry', ('map_quality_report', 'plane_metrics',
                     'thickness_rms_p95_m'), 'max'),
    'planar_coverage_min': (
        'geometry', ('map_quality_report', 'plane_metrics',
                     'planar_coverage'), 'min'),
    'alignment_median_max_px': (
        'alignment', ('weighted_median_px',), 'max'),
    'alignment_inlier_2px_min': (
        'alignment', ('weighted_inlier_2px',), 'min'),
    'heldout_rgb_median_max': (
        'colour', ('rgb_l2_median',), 'max'),
    'heldout_rgb_inlier_20_min': (
        'colour', ('rgb_l2_inlier_20',), 'min'),
    'heldout_scored_fraction_min': (
        'colour', ('heldout_scored_fraction',), 'min'),
    'appearance_coverage_min': (
        'appearance', ('coverage',), 'min'),
    'appearance_roughness_median_max': (
        'appearance', ('roughness', 'roughness_median'), 'max'),
    'appearance_roughness_p90_max': (
        'appearance', ('roughness', 'roughness_p90'), 'max'),
    'appearance_chroma_retention_min': (
        'appearance', ('chroma_retention',), 'min'),
    'appearance_planar_roughness_median_max': (
        'appearance', ('planar_roughness', 'roughness_median'), 'max'),
    'appearance_planar_roughness_p90_max': (
        'appearance', ('planar_roughness', 'roughness_p90'), 'max'),
}


class QualityGateError(Exception):
    """A user-facing report or profile error."""


def load_mapping(path: Path) -> dict[str, Any]:
    """Load a JSON or YAML mapping."""
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise QualityGateError(f'cannot read {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise QualityGateError(f'expected mapping in {path}')
    return data


def nested_number(data: dict[str, Any], path: tuple[str, ...]) -> float:
    """Read one finite numeric value from a nested mapping."""
    value: Any = data
    for key in path:
        if not isinstance(value, dict) or key not in value:
            raise QualityGateError(f'missing metric: {".".join(path)}')
        value = value[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise QualityGateError(f'metric {".".join(path)} must be numeric')
    number = float(value)
    if number != number or abs(number) == float('inf'):
        raise QualityGateError(f'metric {".".join(path)} must be finite')
    return number


def evaluate(reports: dict[str, dict[str, Any]],
             profile_document: dict[str, Any]) -> dict[str, Any]:
    """Evaluate all configured thresholds and return a JSON-ready result."""
    profile = profile_document.get('colored_map_quality_profile')
    if not isinstance(profile, dict):
        raise QualityGateError('profile needs colored_map_quality_profile')
    name = profile.get('name')
    enforcement = profile.get('enforcement')
    thresholds = profile.get('thresholds')
    if not isinstance(name, str) or not name:
        raise QualityGateError('profile name must be a non-empty string')
    if enforcement not in ('blocking', 'report_only'):
        raise QualityGateError('enforcement must be blocking or report_only')
    if not isinstance(thresholds, dict) or not thresholds:
        raise QualityGateError('profile thresholds must be a non-empty mapping')
    unknown = sorted(set(thresholds) - set(METRICS))
    if unknown:
        raise QualityGateError(f'unknown thresholds: {", ".join(unknown)}')

    checks = []
    violations = 0
    for key, limit_value in thresholds.items():
        if isinstance(limit_value, bool) or not isinstance(limit_value, (int, float)):
            raise QualityGateError(f'threshold {key} must be numeric')
        report_name, metric_path, comparison = METRICS[key]
        if report_name not in reports:
            raise QualityGateError(
                f'threshold {key} needs --{report_name}-report')
        value = nested_number(reports[report_name], metric_path)
        limit = float(limit_value)
        passed = value <= limit if comparison == 'max' else value >= limit
        violations += int(not passed)
        checks.append({'name': key, 'source': report_name,
                       'metric': '.'.join(metric_path), 'value': value,
                       'comparison': comparison, 'limit': limit,
                       'verdict': 'PASS' if passed else 'VIOLATION'})
    if enforcement == 'report_only':
        overall = 'REPORT_ONLY'
    else:
        overall = 'FAILED' if violations else 'OK'
    return {'profile': name, 'enforcement': enforcement, 'overall': overall,
            'violations': violations, 'checks': checks}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory-report', type=Path)
    parser.add_argument('--geometry-report', type=Path)
    parser.add_argument('--alignment-report', type=Path)
    parser.add_argument('--colour-report', type=Path)
    parser.add_argument('--appearance-report', type=Path, default=None,
                        help='evaluate_colored_map_appearance.py output; '
                             'required when the profile sets appearance_* '
                             'thresholds')
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    try:
        report_paths = {
            'trajectory': args.trajectory_report,
            'geometry': args.geometry_report,
            'alignment': args.alignment_report,
            'colour': args.colour_report,
            'appearance': args.appearance_report,
        }
        reports = {name: load_mapping(path)
                   for name, path in report_paths.items()
                   if path is not None}
        result = evaluate(reports, load_mapping(args.profile))
    except QualityGateError as exc:
        parser.error(str(exc))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    for check in result['checks']:
        operator = '<=' if check['comparison'] == 'max' else '>='
        print(f"{check['verdict']:9} {check['name']}: "
              f"{check['value']:.6g} {operator} {check['limit']:.6g}")
    print(f"COLORED_MAP_QUALITY_{result['overall']}: "
          f"{result['violations']} violation(s)")
    return 1 if result['overall'] == 'FAILED' else 0


if __name__ == '__main__':
    raise SystemExit(main())
