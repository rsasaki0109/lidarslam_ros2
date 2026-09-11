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

"""Regression tests for degeneracy trajectory and point-projection metrics."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'evaluate_degeneracy_trajectory.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'evaluate_degeneracy_trajectory',
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _trajectory(yaw_rad: float = 0.0) -> np.ndarray:
    half = 0.5 * yaw_rad
    quaternion = np.array([0.0, 0.0, np.sin(half), np.cos(half)])
    xyz = np.array([
        [0.0, 0.0, 0.0],
        [10.0, 0.0, 0.0],
        [10.0, 10.0, 0.0],
        [20.0, 10.0, 0.0],
    ])
    return np.column_stack((
        np.arange(len(xyz), dtype=float),
        xyz,
        np.tile(quaternion, (len(xyz), 1)),
    ))


def _write_tum(path: Path, trajectory: np.ndarray) -> None:
    np.savetxt(path, trajectory, fmt='%.12f')


def test_identical_trajectory_has_zero_point_projection_error(tmp_path: Path):
    module = _load_module()
    candidate_path = tmp_path / 'candidate.tum'
    reference_path = tmp_path / 'reference.tum'
    trajectory = _trajectory()
    _write_tum(candidate_path, trajectory)
    _write_tum(reference_path, trajectory)

    result = module.evaluate(
        candidate_path,
        expected_endpoint_distance=None,
        reference_path=reference_path,
        min_reference_reach_m=1.0,
    )

    point_error = result['reference']['aligned_point_projection_delta_m']
    assert point_error['ranges_m']['20']['max'] < 1.0e-10


def test_point_projection_error_scales_with_range_for_yaw_error(tmp_path: Path):
    module = _load_module()
    candidate_path = tmp_path / 'candidate.tum'
    reference_path = tmp_path / 'reference.tum'
    _write_tum(candidate_path, _trajectory(yaw_rad=0.1))
    _write_tum(reference_path, _trajectory())

    result = module.evaluate(
        candidate_path,
        expected_endpoint_distance=None,
        reference_path=reference_path,
        min_reference_reach_m=1.0,
    )

    ranges = result['reference']['aligned_point_projection_delta_m']['ranges_m']
    assert ranges['10']['rmse'] > ranges['5']['rmse']
    assert np.isclose(ranges['20']['rmse'], 2.0 * ranges['10']['rmse'])


def test_zero_quaternion_fails_closed(tmp_path: Path):
    module = _load_module()
    candidate_path = tmp_path / 'candidate.tum'
    reference_path = tmp_path / 'reference.tum'
    candidate = _trajectory()
    candidate[:, 4:8] = 0.0
    _write_tum(candidate_path, candidate)
    _write_tum(reference_path, _trajectory())

    with pytest.raises(ValueError, match='zero quaternion'):
        module.evaluate(
            candidate_path,
            expected_endpoint_distance=None,
            reference_path=reference_path,
            min_reference_reach_m=1.0,
        )
