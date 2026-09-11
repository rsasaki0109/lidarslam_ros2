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

"""Tests for production LiDAR-camera calibration policy."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'colored_map'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import spatiotemporal_calibration as stc  # noqa: E402, I100


def _pose(x: float, yaw_deg: float = 0.0) -> np.ndarray:
    angle = np.deg2rad(yaw_deg)
    cosine, sine = np.cos(angle), np.sin(angle)
    result = np.eye(4)
    result[:3, :3] = [[cosine, -sine, 0.0],
                      [sine, cosine, 0.0], [0.0, 0.0, 1.0]]
    result[0, 3] = x
    return result


def test_image_pyramid_and_intrinsics_preserve_projection_scale():
    image = np.arange(8 * 10).reshape(8, 10)
    reduced = stc.downsample_nearest(image, 0.5)
    assert reduced.shape == (4, 5)
    np.testing.assert_array_equal(reduced, image[::2, ::2])
    K = np.array([[100.0, 0.0, 5.0],
                  [0.0, 80.0, 4.0], [0.0, 0.0, 1.0]])
    np.testing.assert_allclose(
        stc.scale_intrinsics(K, 0.5),
        [[50.0, 0.0, 2.5], [0.0, 40.0, 2.0], [0.0, 0.0, 1.0]])


def test_stratified_split_spans_path_and_motion_regimes():
    selected = list(range(16))
    stamps = np.arange(16, dtype=float)
    poses = np.asarray([
        _pose(float(index), yaw_deg=20.0 * (index % 4 == 3))
        for index in selected])
    train, heldout, report = stc.stratified_view_split(
        stamps, poses, selected, holdout_fraction=0.25, spatial_segments=4)
    assert set(train).isdisjoint(heldout)
    assert sorted(train + heldout) == selected
    assert len(heldout) >= 4
    heldout_segments = {index // 4 for index in heldout}
    assert heldout_segments == {0, 1, 2, 3}
    assert report['strategy'] == 'travelled_distance_x_motion'
    assert any(item['motion'] == 'rotation' for item in report['strata'])


def test_stratified_split_is_deterministic_for_stationary_path():
    selected = list(range(10))
    stamps = np.arange(10, dtype=float)
    poses = np.repeat(np.eye(4)[None], len(selected), axis=0)
    first = stc.stratified_view_split(stamps, poses, selected)
    second = stc.stratified_view_split(stamps, poses, selected)
    assert first == second
    assert first[1]


def test_bounded_coordinate_search_converges_and_records_history():
    target = np.array([0.04, -0.02, 0.0])

    def objective(parameters):
        return np.sum((parameters - target) ** 2)

    parameters, loss, report = stc.bounded_coordinate_search(
        objective, np.zeros(3), np.full(3, 0.02), np.full(3, 0.1),
        rounds=2)
    np.testing.assert_allclose(parameters, target, atol=0.01)
    assert loss < objective(np.zeros(3))
    assert report['evaluations'] > 1
    assert report['history'][-1]['loss'] == loss


def test_boundary_axes_names_only_saturated_parameters():
    axes = stc.boundary_axes(
        np.array([0.1, 0.0, -0.2, 0.0, 0.0, 0.3, 0.0]),
        np.array([0.1, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3]))
    assert axes == ['dt', 'ty', 'pitch']

    near_axes = stc.boundary_axes(
        np.array([0.091, 0.18, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([0.1, 0.2, 0.2, 0.2, 0.3, 0.3, 0.3]),
        tolerance=np.array([0.01, 0.01, 0.01, 0.01, 0.02, 0.02, 0.02]))
    assert near_axes == ['dt']


def test_observability_reports_uncertainty_for_positive_quadratic():
    diagonal = np.array([5.0, 4.0, 3.0, 2.0, 6.0, 7.0, 8.0])

    def objective(parameters):
        return 1.0 + 0.5 * np.sum(diagonal * parameters ** 2)

    report = stc.finite_difference_observability(
        objective, np.zeros(7), np.ones(7) * 0.01)
    assert report['observable']
    assert np.isclose(report['condition_number'], 4.0)
    assert all(value > 0.0 for value in
               report['uncertainty_dt_s_xyz_m_rpy_rad'])
    assert np.isclose(
        report['maximum_abs_time_translation_correlation'], 0.0,
        atol=1e-12)


def test_observability_rejects_flat_and_clock_translation_coupling():
    def flat(parameters):
        return 1.0 + parameters[1] ** 2

    flat_report = stc.finite_difference_observability(
        flat, np.zeros(7), np.ones(7) * 0.01)
    assert not flat_report['observable']
    assert 'insufficient_positive_curvature' in flat_report['rejection_reasons']

    def coupled(parameters):
        base = np.sum(parameters[2:] ** 2)
        return 1.0 + base + (parameters[0] + parameters[1]) ** 2 + \
            1e-4 * (parameters[0] - parameters[1]) ** 2

    coupled_report = stc.finite_difference_observability(
        coupled, np.zeros(7), np.ones(7) * 0.01,
        maximum_condition=1e8, maximum_correlation=0.9)
    assert not coupled_report['observable']
    assert 'time_translation_correlation' in \
        coupled_report['rejection_reasons']


def test_observability_does_not_invent_correlation_for_indefinite_pair():
    def saddle(parameters):
        return 2.0 + np.sum(parameters[1:] ** 2) - parameters[0] ** 2

    report = stc.finite_difference_observability(
        saddle, np.zeros(7), np.ones(7) * 0.01)
    assert not report['observable']
    assert report['maximum_abs_time_translation_correlation'] is None
    assert all(item['correlation'] is None for item in
               report['time_translation_correlations'])
    assert 'unobservable_time_translation_pair' in \
        report['rejection_reasons']


def test_observability_five_point_fit_rejects_nonlocal_stationary_point():
    center = np.array([0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def shifted(parameters):
        return 1.0 + np.sum((parameters - center) ** 2)

    report = stc.finite_difference_observability(
        shifted, np.zeros(7), np.ones(7) * 0.01)
    assert not report['observable']
    assert 'stationary_point_outside_local_neighborhood' in \
        report['rejection_reasons']
    assert np.isclose(report['axis_quadratic_fits'][0][
        'stationary_offset_steps'], 2.0)
