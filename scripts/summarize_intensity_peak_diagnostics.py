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
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
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

"""Summarize raw intensity-correlation peak diagnostics without accuracy data."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
from typing import Any


REQUIRED_FIELDS = {
    'timestamp',
    'source',
    'correlation',
    'second_best_correlation',
    'peak_margin',
    'longitudinal_shift_m',
    'lateral_shift_m',
    'overlap_bins',
    'base_qualified',
    'has_competing_peak',
    'ambiguous',
    'accepted',
    'motion_dt_s',
    'intensity_velocity_longitudinal_mps',
    'intensity_velocity_lateral_mps',
    'icp_velocity_longitudinal_mps',
    'icp_velocity_lateral_mps',
    'velocity_disagreement_mps',
    'candidate_correction_m',
    'applied_correction_longitudinal_m',
    'applied_correction_lateral_m',
    'applied_correction_m',
    'disagreement_streak',
    'disagreement_measured',
    'correction_applied',
    'intensity_channel_correlation',
    'height_channel_correlation',
}
QUANTILES = (
    ('min', 0.00),
    ('p01', 0.01),
    ('p05', 0.05),
    ('p10', 0.10),
    ('p25', 0.25),
    ('p50', 0.50),
    ('p75', 0.75),
    ('p90', 0.90),
    ('p95', 0.95),
    ('p99', 0.99),
    ('max', 1.00),
)
REPORTING_THRESHOLDS = (0.005, 0.01, 0.02, 0.05, 0.1, 0.2)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_bool(value: str, path: Path, line_number: int, field: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {'1', 'true'}:
        return True
    if normalized in {'0', 'false'}:
        return False
    raise ValueError(
        f'{path}:{line_number}: {field} must be 0/1 or true/false')


def _quantile(sorted_values: list[float], probability: float) -> float | None:
    if not sorted_values:
        return None
    position = probability * (len(sorted_values) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return sorted_values[lower]
    fraction = position - lower
    return (
        sorted_values[lower] * (1.0 - fraction)
        + sorted_values[upper] * fraction
    )


def _quantiles(values: list[float]) -> dict[str, float | None]:
    values.sort()
    return {
        name: _quantile(values, probability)
        for name, probability in QUANTILES
    }


def _finite_float(
    row: dict[str, str],
    field: str,
    path: Path,
    line_number: int,
) -> float:
    value = float(row[field])
    if not math.isfinite(value):
        raise ValueError(
            f'{path}:{line_number}: {field} must be finite')
    return value


def summarize(paths: list[Path]) -> dict[str, Any]:
    margins: list[float] = []
    velocity_gaps: list[float] = []
    longitudinal_velocity_gaps: list[float] = []
    lateral_velocity_gaps: list[float] = []
    candidate_corrections: list[float] = []
    applied_corrections: list[float] = []
    applied_longitudinal_corrections: list[float] = []
    applied_lateral_corrections: list[float] = []
    intensity_channel_correlations: list[float] = []
    height_channel_correlations: list[float] = []
    channel_correlation_gaps: list[float] = []
    missing_channel_score_count = 0
    source_counts: Counter[str] = Counter()
    accepted_count = 0
    ambiguous_count = 0
    base_qualified_count = 0
    inputs: list[dict[str, Any]] = []

    for path in paths:
        row_count = 0
        qualified_count = 0
        measured_count = 0
        corrected_count = 0
        channel_score_count = 0
        missing_file_channel_score_count = 0
        with path.open(newline='', encoding='utf-8') as stream:
            reader = csv.DictReader(stream)
            fields = set(reader.fieldnames or ())
            missing = sorted(REQUIRED_FIELDS - fields)
            if missing:
                raise ValueError(
                    f'{path}: missing required columns: {", ".join(missing)}')
            for line_number, row in enumerate(reader, 2):
                row_count += 1
                disagreement_measured = _parse_bool(
                    row['disagreement_measured'],
                    path,
                    line_number,
                    'disagreement_measured',
                )
                correction_applied = _parse_bool(
                    row['correction_applied'],
                    path,
                    line_number,
                    'correction_applied',
                )
                if correction_applied and not disagreement_measured:
                    raise ValueError(
                        f'{path}:{line_number}: correction_applied requires '
                        'disagreement_measured')
                if disagreement_measured:
                    measured_count += 1
                    intensity_longitudinal = _finite_float(
                        row,
                        'intensity_velocity_longitudinal_mps',
                        path,
                        line_number,
                    )
                    intensity_lateral = _finite_float(
                        row,
                        'intensity_velocity_lateral_mps',
                        path,
                        line_number,
                    )
                    icp_longitudinal = _finite_float(
                        row,
                        'icp_velocity_longitudinal_mps',
                        path,
                        line_number,
                    )
                    icp_lateral = _finite_float(
                        row,
                        'icp_velocity_lateral_mps',
                        path,
                        line_number,
                    )
                    velocity_gaps.append(_finite_float(
                        row,
                        'velocity_disagreement_mps',
                        path,
                        line_number,
                    ))
                    longitudinal_velocity_gaps.append(
                        intensity_longitudinal - icp_longitudinal)
                    lateral_velocity_gaps.append(
                        intensity_lateral - icp_lateral)
                    candidate_corrections.append(_finite_float(
                        row,
                        'candidate_correction_m',
                        path,
                        line_number,
                    ))
                if correction_applied:
                    corrected_count += 1
                    applied_corrections.append(_finite_float(
                        row,
                        'applied_correction_m',
                        path,
                        line_number,
                    ))
                    applied_longitudinal_corrections.append(_finite_float(
                        row,
                        'applied_correction_longitudinal_m',
                        path,
                        line_number,
                    ))
                    applied_lateral_corrections.append(_finite_float(
                        row,
                        'applied_correction_lateral_m',
                        path,
                        line_number,
                    ))
                base_qualified = _parse_bool(
                    row['base_qualified'], path, line_number, 'base_qualified')
                if not base_qualified:
                    continue
                source = row['source'].strip()
                if not source:
                    raise ValueError(
                        f'{path}:{line_number}: source must not be empty')
                source_counts[source] += 1
                if source == 'oriented_grid':
                    intensity_channel = _finite_float(
                        row,
                        'intensity_channel_correlation',
                        path,
                        line_number,
                    )
                    height_channel = _finite_float(
                        row,
                        'height_channel_correlation',
                        path,
                        line_number,
                    )
                    intensity_available = (
                        -1.0 <= intensity_channel <= 1.0)
                    height_available = -1.0 <= height_channel <= 1.0
                    if not intensity_available and intensity_channel != -2.0:
                        raise ValueError(
                            f'{path}:{line_number}: oriented_grid channel '
                            'intensity correlation must be in [-1, 1] or -2')
                    if not height_available and height_channel != -2.0:
                        raise ValueError(
                            f'{path}:{line_number}: oriented_grid channel '
                            'height correlation must be in [-1, 1] or -2')
                    if intensity_available and height_available:
                        intensity_channel_correlations.append(
                            intensity_channel)
                        height_channel_correlations.append(height_channel)
                        channel_correlation_gaps.append(
                            height_channel - intensity_channel)
                        channel_score_count += 1
                    else:
                        missing_channel_score_count += 1
                        missing_file_channel_score_count += 1
                qualified_count += 1
                base_qualified_count += 1
                ambiguous_count += _parse_bool(
                    row['ambiguous'], path, line_number, 'ambiguous')
                accepted_count += _parse_bool(
                    row['accepted'], path, line_number, 'accepted')
                has_competing_peak = _parse_bool(
                    row['has_competing_peak'],
                    path,
                    line_number,
                    'has_competing_peak',
                )
                if not has_competing_peak:
                    continue
                margins.append(_finite_float(
                    row, 'peak_margin', path, line_number))
        inputs.append({
            'path': str(path.resolve()),
            'sha256': _sha256(path),
            'rows': row_count,
            'base_qualified_rows': qualified_count,
            'disagreement_measured_rows': measured_count,
            'correction_applied_rows': corrected_count,
            'oriented_grid_channel_score_rows': channel_score_count,
            'oriented_grid_missing_channel_score_rows':
                missing_file_channel_score_count,
            'correction_duty_cycle': (
                corrected_count / measured_count
                if measured_count else None
            ),
        })

    margins.sort()
    qualified_total = len(margins)
    return {
        'schema_version': 5,
        'selection_independent': True,
        'accuracy_metrics_consumed': False,
        'inputs': inputs,
        'rows': {
            'total': sum(item['rows'] for item in inputs),
            'base_qualified': base_qualified_count,
            'with_competing_peak': qualified_total,
            'accepted': accepted_count,
            'ambiguous': ambiguous_count,
            'disagreement_measured': len(velocity_gaps),
            'correction_applied': len(applied_corrections),
        },
        'source_counts': dict(sorted(source_counts.items())),
        'peak_margin_quantiles': _quantiles(margins),
        'disagreement': {
            'correction_duty_cycle': (
                len(applied_corrections) / len(velocity_gaps)
                if velocity_gaps else None
            ),
            'velocity_gap_mps_quantiles': _quantiles(velocity_gaps),
            'longitudinal_velocity_gap_mps_quantiles':
                _quantiles(longitudinal_velocity_gaps),
            'lateral_velocity_gap_mps_quantiles':
                _quantiles(lateral_velocity_gaps),
            'candidate_correction_m_quantiles':
                _quantiles(candidate_corrections),
            'applied_correction_m_quantiles':
                _quantiles(applied_corrections),
            'applied_longitudinal_correction_m_quantiles':
                _quantiles(applied_longitudinal_corrections),
            'applied_lateral_correction_m_quantiles':
                _quantiles(applied_lateral_corrections),
        },
        'oriented_grid_channels': {
            'paired_rows': len(channel_correlation_gaps),
            'missing_rows': missing_channel_score_count,
            'intensity_correlation_quantiles':
                _quantiles(intensity_channel_correlations),
            'height_correlation_quantiles':
                _quantiles(height_channel_correlations),
            'height_minus_intensity_quantiles':
                _quantiles(channel_correlation_gaps),
        },
        'thresholds': {
            format(threshold, 'g'): {
                'below_count': sum(value < threshold for value in margins),
                'below_fraction': (
                    sum(value < threshold for value in margins)
                    / qualified_total
                    if qualified_total else None
                ),
            }
            for threshold in REPORTING_THRESHOLDS
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'diagnostics',
        nargs='+',
        type=Path,
        help='intensity_peak_diagnostics.csv files to aggregate',
    )
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()

    result = summarize(args.diagnostics)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
