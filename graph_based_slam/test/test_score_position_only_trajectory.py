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

"""Tests for position-only trajectory scoring."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'score_position_only_trajectory.py'
SPEC = importlib.util.spec_from_file_location('position_scorer', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
SCORER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SCORER)


def _write_trajectory(path: Path, positions: np.ndarray) -> Path:
    lines = [
        f'{index * 0.1:.9f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1'
        for index, (x, y, z) in enumerate(positions)
    ]
    path.write_text('\n'.join(lines) + '\n')
    return path


def _curved_positions() -> np.ndarray:
    parameter = np.linspace(0.0, 25.0, 251)
    return np.column_stack((
        parameter,
        2.0 * np.sin(parameter / 4.0),
        0.2 * np.cos(parameter / 3.0),
    ))


def test_score_is_invariant_to_rigid_coordinate_transform(tmp_path):
    reference = _curved_positions()
    yaw = np.deg2rad(37.0)
    rotation = np.array((
        (np.cos(yaw), -np.sin(yaw), 0.0),
        (np.sin(yaw), np.cos(yaw), 0.0),
        (0.0, 0.0, 1.0),
    ))
    estimate = (rotation @ reference.T).T + np.array((8.0, -3.0, 1.2))
    result = SCORER.score(
        _write_trajectory(tmp_path / 'reference.tum', reference),
        _write_trajectory(tmp_path / 'estimate.tum', estimate),
        segment_length=10.0,
        max_time_gap=0.01,
    )
    assert result['alignment']['type'] == 'se3_no_scale'
    assert result['trajectory']['ate_rmse_m'] < 1e-8
    assert result['trajectory']['rte_translation_percent_10m'] < 1e-8
    assert result['association']['matched_ground_truth_fraction'] == 1.0


def test_score_does_not_hide_scale_drift(tmp_path):
    reference = _curved_positions()
    estimate = reference * 1.01
    result = SCORER.score(
        _write_trajectory(tmp_path / 'reference.tum', reference),
        _write_trajectory(tmp_path / 'estimate.tum', estimate),
        segment_length=10.0,
        max_time_gap=0.01,
    )
    assert result['trajectory']['ate_rmse_m'] > 0.05
    assert 0.9 < result['trajectory']['rte_translation_percent_10m'] < 1.2


def test_association_reports_missing_ground_truth_fraction(tmp_path):
    reference = _curved_positions()
    estimate = reference[25:-25]
    result = SCORER.score(
        _write_trajectory(tmp_path / 'reference.tum', reference),
        _write_trajectory(tmp_path / 'estimate.tum', estimate),
        segment_length=10.0,
        max_time_gap=0.01,
    )
    assert result['association']['matched_poses'] == len(estimate)
    assert result['association']['matched_ground_truth_fraction'] < 0.9


def test_nominal_10_hz_timestamp_jitter_does_not_fragment_rte(tmp_path):
    reference_stamps = np.arange(0.0, 25.0, 0.005)
    reference_positions = np.column_stack((
        reference_stamps,
        np.zeros_like(reference_stamps),
        np.zeros_like(reference_stamps),
    ))
    estimate_stamps = np.arange(0.0, 25.0, 0.10005)
    estimate_positions = np.column_stack((
        estimate_stamps,
        np.zeros_like(estimate_stamps),
        np.zeros_like(estimate_stamps),
    ))
    reference = tmp_path / 'reference.tum'
    estimate = tmp_path / 'estimate.tum'
    reference.write_text('\n'.join(
        f'{stamp:.9f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1'
        for stamp, (x, y, z) in zip(reference_stamps, reference_positions)
    ) + '\n')
    estimate.write_text('\n'.join(
        f'{stamp:.9f} {x:.9f} {y:.9f} {z:.9f} 0 0 0 1'
        for stamp, (x, y, z) in zip(estimate_stamps, estimate_positions)
    ) + '\n')

    result = SCORER.score(
        reference,
        estimate,
        segment_length=10.0,
        max_time_gap=0.11,
    )

    assert result['association']['contiguous_blocks'] == 1
    assert result['association']['matched_ground_truth_fraction'] > 0.99
    assert result['trajectory']['rte_segments'] > 0
