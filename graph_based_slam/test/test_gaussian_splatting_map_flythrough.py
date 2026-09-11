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

"""Tests for the map-flythrough renderer's pure path helpers (CPU only)."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import render_map_flythrough

    return render_map_flythrough


mf = _load()


# --------------------------------------------------------------------------- #
# moving_average_edge
# --------------------------------------------------------------------------- #
def test_moving_average_preserves_constant_and_shape():
    x = np.tile([1.0, -2.0, 3.0], (20, 1))
    out = mf.moving_average_edge(x, 7)
    assert out.shape == x.shape
    assert np.allclose(out, x)


def test_moving_average_window_one_is_identity():
    x = np.random.default_rng(0).normal(size=(10, 3))
    assert np.allclose(mf.moving_average_edge(x, 1), x)


# --------------------------------------------------------------------------- #
# resample_equal_arclength
# --------------------------------------------------------------------------- #
def test_resample_equal_arclength_uniform_steps():
    t = np.linspace(0.0, 1.0, 50)
    pts = np.stack([10.0 * t ** 2, np.zeros_like(t), np.zeros_like(t)], axis=1)
    out, total = mf.resample_equal_arclength(pts, 100)
    step = np.linalg.norm(np.diff(out, axis=0), axis=1)
    assert out.shape == (100, 3)
    assert total == pytest.approx(10.0, abs=1e-6)
    assert step.max() - step.min() < 1e-3


def test_resample_equal_arclength_skips_standstill():
    # 10 m walk, 30 repeated samples standing still, 10 m walk back up.
    walk = np.linspace([0, 0, 0], [10, 0, 0], 20)
    stand = np.tile([10.0, 0.0, 0.0], (30, 1))
    back = np.linspace([10, 0, 0], [10, 10, 0], 20)
    out, total = mf.resample_equal_arclength(np.vstack([walk, stand, back]), 80)
    step = np.linalg.norm(np.diff(out, axis=0), axis=1)
    assert total == pytest.approx(20.0, abs=1e-3)
    # No dwell: every frame advances by roughly total / (frames - 1).
    assert step.min() > 0.5 * total / 79


def test_resample_equal_arclength_rejects_single_point():
    with pytest.raises(ValueError):
        mf.resample_equal_arclength(np.zeros((1, 3)), 10)


def test_resample_equal_arclength_rejects_all_duplicates():
    with pytest.raises(ValueError):
        mf.resample_equal_arclength(np.tile([1.0, 2.0, 3.0], (10, 1)), 10)


def test_resample_equal_arclength_uniform_on_l_shaped_path():
    leg_a = np.linspace([0, 0, 0], [6, 0, 0], 7)
    leg_b = np.linspace([6, 0, 0], [6, 4, 0], 5)[1:]
    out, total = mf.resample_equal_arclength(np.vstack([leg_a, leg_b]), 51)
    step = np.linalg.norm(np.diff(out, axis=0), axis=1)
    assert total == pytest.approx(10.0, abs=1e-9)
    assert np.allclose(step, 0.2, atol=1e-6)
    assert out[-1] == pytest.approx([6.0, 4.0, 0.0])


# --------------------------------------------------------------------------- #
# smooth_tangents
# --------------------------------------------------------------------------- #
def test_smooth_tangents_unit_norm_and_direction():
    path = np.stack([np.linspace(0, 12, 40), np.zeros(40), np.zeros(40)], axis=1)
    t = mf.smooth_tangents(path)
    assert np.allclose(np.linalg.norm(t, axis=1), 1.0)
    assert np.allclose(t, np.tile([1.0, 0.0, 0.0], (40, 1)))


def test_smooth_tangents_repeated_samples_inherit_direction():
    path = np.vstack([np.linspace([0, 0, 0], [5, 0, 0], 10), np.tile([5.0, 0, 0], (10, 1))])
    t = mf.smooth_tangents(path, window=1)
    assert np.allclose(np.linalg.norm(t, axis=1), 1.0)
    assert np.allclose(t[-1], [1.0, 0.0, 0.0])


# --------------------------------------------------------------------------- #
# third_person_path + build_viewmats
# --------------------------------------------------------------------------- #
def test_third_person_path_behind_above_and_looking_at_ride():
    ride = np.stack([np.linspace(0, 30, 60), np.zeros(60), np.full(60, -10.0)], axis=1)
    tangents = mf.smooth_tangents(ride)
    eyes, forwards = mf.third_person_path(ride, tangents, follow_back=5.5, lift=5.5)
    mid = 30
    assert eyes[mid, 0] == pytest.approx(ride[mid, 0] - 5.5, abs=0.2)
    assert eyes[mid, 2] == pytest.approx(ride[mid, 2] + 5.5, abs=0.2)
    to_subject = ride[mid] + mf.WORLD_UP * 0.8 - eyes[mid]
    to_subject /= np.linalg.norm(to_subject)
    assert float(forwards[mid] @ to_subject) == pytest.approx(1.0, abs=1e-6)


def test_build_viewmats_rotation_is_right_handed_and_not_rolled():
    ride = np.stack([np.linspace(0, 30, 60), np.linspace(0, 8, 60),
                     np.full(60, -10.0)], axis=1)
    eyes, forwards = mf.third_person_path(ride, mf.smooth_tangents(ride),
                                          follow_back=5.5, lift=5.5)
    viewmats = mf.build_viewmats(eyes, forwards)
    assert viewmats.shape == (60, 4, 4)
    assert not np.isnan(viewmats).any()
    for vm in viewmats[::10]:
        rot = np.linalg.inv(vm)[:3, :3]
        assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-5)
        assert np.linalg.det(rot) == pytest.approx(1.0, abs=1e-5)
        # Camera "down" (the c2w y column) must point downwards in the world:
        # the reversed cross-product order rolls the camera 180 degrees.
        assert rot[2, 1] < -0.5


def test_build_viewmats_near_vertical_forward_stays_finite():
    eyes = np.tile([0.0, 0.0, 10.0], (5, 1))
    forwards = np.vstack([
        np.tile([1.0, 0.0, 0.0], (2, 1)),       # establish a heading first
        np.tile([0.0, 0.0, -1.0], (3, 1)),      # then look straight down
    ])
    viewmats = mf.build_viewmats(eyes, forwards)
    assert not np.isnan(viewmats).any()
    for vm in viewmats:
        rot = np.linalg.inv(vm)[:3, :3]
        assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-5)
        assert np.linalg.det(rot) == pytest.approx(1.0, abs=1e-5)


def test_build_viewmats_recovers_eye_position():
    ride = np.stack([np.linspace(0, 10, 20), np.zeros(20), np.zeros(20)], axis=1)
    eyes, forwards = mf.third_person_path(ride, mf.smooth_tangents(ride),
                                          follow_back=3.0, lift=4.0)
    viewmats = mf.build_viewmats(eyes, forwards)
    centres = np.stack([np.linalg.inv(vm)[:3, 3] for vm in viewmats])
    assert np.allclose(centres, eyes, atol=1e-4)


# --------------------------------------------------------------------------- #
# enhance_colors + uncolored_mask
# --------------------------------------------------------------------------- #
def test_enhance_colors_stays_in_range_and_boosts_saturation():
    rng = np.random.default_rng(2)
    rgb = rng.uniform(0.2, 0.8, size=(1000, 3))
    out = mf.enhance_colors(rgb)
    assert out.shape == rgb.shape
    assert out.min() >= 0.0 and out.max() <= 1.0
    spread_in = (rgb - rgb.mean(axis=1, keepdims=True)).std()
    spread_out = (out - out.mean(axis=1, keepdims=True)).std()
    assert spread_out > spread_in


def test_enhance_colors_saturation_one_keeps_grey_grey():
    grey = np.full((10, 3), 0.5)
    out = mf.enhance_colors(grey, saturation=1.0)
    # Grey stays channel-balanced whatever the contrast stretch does.
    assert np.allclose(out, out[:, :1])


def test_enhance_colors_near_uniform_input_is_not_overamplified():
    rng = np.random.default_rng(3)
    rgb = 0.5 + rng.uniform(-2e-4, 2e-4, size=(500, 3))
    out = mf.enhance_colors(rgb, saturation=1.0)
    # The percentile stretch must skip a sub-epsilon range instead of blowing
    # quantisation noise up to full black/white.
    assert out.max() - out.min() < 0.01


def test_uncolored_mask_matches_default_grey_only():
    rgb = np.array([[128, 128, 128], [128, 128, 129], [0, 0, 0], [128, 0, 128]],
                   dtype=np.uint8)
    assert mf.uncolored_mask(rgb).tolist() == [True, False, False, False]


# --------------------------------------------------------------------------- #
# ceiling_cut_mask
# --------------------------------------------------------------------------- #
def test_ceiling_cut_follows_local_walking_height():
    # Ramp: walking height descends from z=0 (x=0) to z=-3 (x=30).
    ride = np.stack([np.linspace(0, 30, 40), np.zeros(40),
                     np.linspace(0, -3, 40)], axis=1)
    # One point 1.5 m above each end's local height, one 4 m above.
    xyz = np.array([
        [0.0, 0.0, 1.5], [0.0, 0.0, 4.0],
        [30.0, 0.0, -1.5], [30.0, 0.0, 1.0],
    ])
    keep = mf.ceiling_cut_mask(xyz, ride, 2.3)
    assert keep.tolist() == [True, False, True, False]


def test_ceiling_cut_mask_chunking_consistent():
    rng = np.random.default_rng(1)
    ride = np.stack([np.linspace(0, 20, 30), np.zeros(30), np.zeros(30)], axis=1)
    xyz = rng.uniform([-1, -1, -1], [21, 1, 5], size=(500, 3))
    assert np.array_equal(mf.ceiling_cut_mask(xyz, ride, 2.3, chunk=64),
                          mf.ceiling_cut_mask(xyz, ride, 2.3, chunk=100000))


def test_cinematic_resampling_preserves_endpoints_and_slows_turns():
    points = np.array([[0.0, 0.0, 0.0], [5.0, 0.0, 0.0],
                       [5.0, 5.0, 0.0]])
    equal, _ = mf.resample_cinematic_arclength(points, 41, 0.0)
    slow, total = mf.resample_cinematic_arclength(points, 41, 4.0)
    np.testing.assert_allclose(slow[[0, -1]], points[[0, -1]])
    assert total == pytest.approx(10.0)
    corner = np.array([5.0, 0.0, 0.0])
    assert np.count_nonzero(np.linalg.norm(slow - corner, axis=1) < 1.0) > \
        np.count_nonzero(np.linalg.norm(equal - corner, axis=1) < 1.0)


def test_look_ahead_targets_moves_focus_forward_without_overshoot():
    path = np.column_stack([np.arange(6.0), np.zeros((6, 2))])
    targets = mf.look_ahead_targets(path, 2.0)
    np.testing.assert_allclose(targets[:, 0], [2, 3, 4, 5, 5, 5])


def test_flythrough_metrics_reports_black_and_flicker():
    frames = np.zeros((3, 4, 4, 3), dtype=np.uint8)
    frames[0, :2] = 100
    frames[1, :2] = 140
    frames[2, :2] = 100
    report = mf.flythrough_metrics(frames)
    assert report['black_pixel_fraction'] == 0.5
    assert report['temporal_luminance_delta_median'] > 0.0
    assert report['temporal_flicker_p90'] > 0.0
