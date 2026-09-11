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

"""Tests for intensity peak diagnostic summarization."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'summarize_intensity_peak_diagnostics.py'
SPEC = importlib.util.spec_from_file_location('peak_summary', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
SUMMARY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SUMMARY)

HEADER = (
    'timestamp,source,correlation,second_best_correlation,peak_margin,'
    'longitudinal_shift_m,lateral_shift_m,overlap_bins,base_qualified,'
    'has_competing_peak,ambiguous,accepted,motion_dt_s,'
    'intensity_velocity_longitudinal_mps,'
    'intensity_velocity_lateral_mps,icp_velocity_longitudinal_mps,'
    'icp_velocity_lateral_mps,velocity_disagreement_mps,'
    'candidate_correction_m,applied_correction_longitudinal_m,'
    'applied_correction_lateral_m,applied_correction_m,'
    'disagreement_streak,disagreement_measured,correction_applied,'
    'intensity_channel_correlation,height_channel_correlation\n'
)


def test_combines_inputs_and_ignores_unqualified_rows(tmp_path):
    first = tmp_path / 'first.csv'
    first.write_text(
        HEADER
        + '1.0,prior,0.8,0.79,0.01,0.25,0.0,50,1,1,0,1,'
        '0,0,0,0,0,0,0,0,0,0,0,0,0,-2,-2\n'
        + '2.0,prior,0.4,0.39,0.01,0.0,0.0,10,0,1,0,0,'
        '0,0,0,0,0,0,0,0,0,0,0,0,0,-2,-2\n',
        encoding='utf-8',
    )
    second = tmp_path / 'second.csv'
    second.write_text(
        HEADER
        + '3.0,prior,0.8,0.75,0.05,0.0,0.0,50,true,true,false,true,'
        '0,0,0,0,0,0,0,0,0,0,0,false,false,-2,-2\n'
        + '4.0,oriented_grid,0.8,0.60,0.20,0.5,-0.25,50,1,1,1,0,'
        '0.1,1.0,0.2,0.4,0.1,0.608276,0.0608276,0.06,0.01,'
        '0.0608276,10,true,true,0.7,0.9\n',
        encoding='utf-8',
    )

    result = SUMMARY.summarize([first, second])

    assert result['schema_version'] == 5
    assert result['selection_independent'] is True
    assert result['accuracy_metrics_consumed'] is False
    assert result['rows'] == {
        'total': 4,
        'base_qualified': 3,
        'with_competing_peak': 3,
        'accepted': 2,
        'ambiguous': 1,
        'disagreement_measured': 1,
        'correction_applied': 1,
    }
    assert result['source_counts'] == {
        'oriented_grid': 1,
        'prior': 2,
    }
    assert result['peak_margin_quantiles']['p50'] == pytest.approx(0.05)
    assert result['thresholds']['0.1'] == {
        'below_count': 2,
        'below_fraction': pytest.approx(2.0 / 3.0),
    }
    assert result['disagreement']['correction_duty_cycle'] == 1.0
    assert (
        result['disagreement']['velocity_gap_mps_quantiles']['p50']
        == pytest.approx(0.608276)
    )
    assert (
        result['disagreement'][
            'longitudinal_velocity_gap_mps_quantiles']['p50']
        == pytest.approx(0.6)
    )
    assert len(result['inputs']) == 2
    assert all(len(item['sha256']) == 64 for item in result['inputs'])
    assert result['inputs'][0]['correction_duty_cycle'] is None
    assert result['inputs'][1]['correction_duty_cycle'] == 1.0
    assert result['oriented_grid_channels']['paired_rows'] == 1
    assert result['oriented_grid_channels']['missing_rows'] == 0
    assert (
        result['oriented_grid_channels'][
            'intensity_correlation_quantiles']['p50']
        == pytest.approx(0.7)
    )
    assert (
        result['oriented_grid_channels'][
            'height_minus_intensity_quantiles']['p50']
        == pytest.approx(0.2)
    )


def test_rejects_legacy_csv_without_base_qualified(tmp_path):
    legacy = tmp_path / 'legacy.csv'
    legacy.write_text(
        HEADER.replace('base_qualified,', '')
        + '1.0,prior,0.8,0.7,0.1,0.0,0.0,50,1,0,1,'
        '0,0,0,0,0,0,0,0,0,0,0,0,0,-2,-2\n',
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='base_qualified'):
        SUMMARY.summarize([legacy])
