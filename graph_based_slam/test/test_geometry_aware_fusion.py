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

"""Tests for geometry-aware RGB fusion guards."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'colored_map'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import geometry_aware_fusion as gaf  # noqa: E402, I100


def test_depth_neighborhood_reports_sparse_discontinuity():
    depth = np.full((5, 5), np.inf)
    depth[2, 2], depth[2, 3] = 2.0, 8.0
    minimum, maximum, support = gaf.neighborhood_depth_statistics(
        depth, np.array([2, 3]), np.array([2, 2]), np.array([1, 1]))
    np.testing.assert_array_equal(minimum, [2.0, 2.0])
    np.testing.assert_array_equal(maximum, [8.0, 8.0])
    np.testing.assert_array_equal(support, [2, 2])


def test_depth_neighborhood_honours_per_query_radius():
    depth = np.full((5, 5), np.inf)
    depth[2, 1], depth[2, 2] = 3.0, 7.0
    minimum, maximum, support = gaf.neighborhood_depth_statistics(
        depth, np.array([2, 2]), np.array([2, 2]), np.array([0, 1]))
    np.testing.assert_array_equal(minimum, [7.0, 3.0])
    np.testing.assert_array_equal(maximum, [7.0, 7.0])
    np.testing.assert_array_equal(support, [1, 2])


def test_mask_neighborhood_dilates_only_requested_queries():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True
    result = gaf.mask_neighborhood_any(
        mask, np.array([2, 3, 4]), np.array([2, 2, 2]),
        np.array([0, 1, 1]))
    assert result.tolist() == [True, True, False]


def test_camera_motion_rates_use_pose_and_timestamp_spacing():
    poses = np.repeat(np.eye(4)[None], 3, axis=0)
    poses[:, 0, 3] = [0.0, 1.0, 3.0]
    linear, angular = gaf.camera_motion_rates(
        np.linalg.inv(poses), [0.0, 1.0, 2.0])
    np.testing.assert_allclose(linear, [1.0, 1.5, 2.0])
    np.testing.assert_allclose(angular, 0.0)


def test_calibration_uncertainty_expands_near_points_more():
    calibration = {
        'accepted': True,
        'uncertainty_dt_s_xyz_m_rpy_rad':
            [0.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0],
    }
    radii = gaf.calibration_pixel_radii(
        np.array([2.0, 10.0]), 100.0, calibration,
        sigma_multiplier=1.0, maximum_radius=10)
    assert radii.tolist() == [1, 1]
    stronger = gaf.calibration_pixel_radii(
        np.array([2.0, 10.0]), 100.0, calibration,
        linear_speed=2.0, sigma_multiplier=2.0, maximum_radius=10)
    assert stronger[0] > stronger[1]


def test_calibration_uncertainty_is_default_off_and_requires_acceptance():
    depth = np.array([5.0])
    assert gaf.calibration_pixel_radii(
        depth, 100.0, None).tolist() == [0]
    with np.testing.assert_raises(ValueError):
        gaf.calibration_pixel_radii(
            depth, 100.0, None, sigma_multiplier=1.0)
    with np.testing.assert_raises(ValueError):
        gaf.calibration_pixel_radii(
            depth, 100.0,
            {'accepted': False,
             'uncertainty_dt_s_xyz_m_rpy_rad': [0.0] * 7},
            sigma_multiplier=1.0)
