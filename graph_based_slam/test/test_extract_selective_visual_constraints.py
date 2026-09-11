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

"""Tests for selective visual relative-pose constraint extraction."""

import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'visual_constraints', ROOT / 'scripts' /
    'extract_selective_visual_constraints.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_opengl_pose_is_converted_to_opencv_axes():
    frame = {'transform_matrix': np.eye(4).tolist()}
    np.testing.assert_array_equal(
        MODULE.camera_pose_opencv(frame), np.diag([1.0, -1.0, -1.0, 1.0]))


def test_rotation_angle_handles_identity_and_pi():
    assert MODULE.rotation_angle_deg(np.eye(3)) == 0.0
    np.testing.assert_allclose(
        MODULE.rotation_angle_deg(np.diag([-1.0, -1.0, 1.0])), 180.0)


def test_multiple_essential_matrices_are_split_in_stable_order():
    candidates = [np.eye(3), np.diag([1.0, -1.0, -1.0])]
    split = MODULE.essential_candidates(np.vstack(candidates))
    assert len(split) == 2
    np.testing.assert_array_equal(split[0], candidates[0])
    np.testing.assert_array_equal(split[1], candidates[1])


def test_gate_accepts_only_when_every_confidence_check_passes():
    common = {
        'tracks': 100, 'inliers': 60, 'rotation_error_deg': 1.0,
        'translation_cosine': 0.8, 'predicted_translation_m': 0.2,
        'min_tracks': 80, 'min_inliers': 50, 'min_inlier_ratio': 0.2,
        'max_rotation_error_deg': 3.0, 'min_translation_cosine': 0.5,
        'min_translation_m': 0.03, 'max_translation_m': 2.0}
    accepted, reasons = MODULE.evaluate_gate(**common)
    assert accepted and reasons == []
    common['translation_cosine'] = 0.1
    accepted, reasons = MODULE.evaluate_gate(**common)
    assert not accepted
    assert reasons == ['translation_disagreement']


def test_gate_reports_all_failed_checks_deterministically():
    accepted, reasons = MODULE.evaluate_gate(
        tracks=10, inliers=1, rotation_error_deg=8.0,
        translation_cosine=-0.2, predicted_translation_m=0.0,
        min_tracks=80, min_inliers=50, min_inlier_ratio=0.2,
        max_rotation_error_deg=3.0, min_translation_cosine=0.5,
        min_translation_m=0.03, max_translation_m=2.0)
    assert not accepted
    assert reasons == [
        'insufficient_tracks', 'insufficient_inliers', 'low_inlier_ratio',
        'rotation_disagreement', 'translation_disagreement',
        'insufficient_baseline']
