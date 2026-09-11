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

"""Tests for the coloured-map appearance metrics (chroma / roughness)."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'


def _load():
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    import evaluate_colored_map_appearance

    return evaluate_colored_map_appearance


app = _load()


def test_channel_range_chroma_grey_is_zero_and_saturated_is_high():
    rgb = np.array([[100, 100, 100], [255, 0, 0], [10, 20, 30]])
    chroma = app.channel_range_chroma(rgb)
    np.testing.assert_allclose(chroma, [0.0, 255.0, 20.0])
    with pytest.raises(ValueError):
        app.channel_range_chroma(np.zeros((3, 4)))


def test_image_chroma_mixes_mono_and_colour():
    colour = np.zeros((8, 8, 3), dtype=np.uint8)
    colour[:, :, 0] = 200  # constant red: channel range 200 everywhere
    mono = np.full((8, 8), 90, dtype=np.uint8)
    assert app.image_chroma([colour]) == pytest.approx(200.0)
    assert app.image_chroma([mono]) == pytest.approx(0.0)
    assert app.image_chroma([colour, mono]) == pytest.approx(100.0)
    with pytest.raises(ValueError):
        app.image_chroma([])
    with pytest.raises(ValueError):
        app.image_chroma([colour], pixel_stride=0)


def test_roughness_zero_for_uniform_surface():
    rng = np.random.default_rng(0)
    xyz = rng.uniform(0.0, 1.0, (500, 3))
    rgb = np.full((500, 3), 120, dtype=np.uint8)
    result = app.voxel_color_roughness(xyz, rgb, voxel=0.25)
    assert result['voxels_scored'] > 0
    assert result['roughness_median'] == pytest.approx(0.0)


def test_roughness_detects_pepper_noise():
    rng = np.random.default_rng(1)
    xyz = rng.uniform(0.0, 1.0, (2000, 3))
    clean = np.full((2000, 3), 120, dtype=np.uint8)
    peppered = clean.copy()
    idx = rng.choice(2000, 200, replace=False)
    peppered[idx] = rng.integers(0, 255, (200, 3))
    quiet = app.voxel_color_roughness(xyz, clean, voxel=0.25)
    noisy = app.voxel_color_roughness(xyz, peppered, voxel=0.25)
    assert noisy['roughness_p90'] > quiet['roughness_p90'] + 10.0


def test_roughness_low_for_organised_texture_gradient():
    # A smooth colour gradient across the cloud is texture, not pepper:
    # within a small voxel the colours barely differ.
    n = 4000
    rng = np.random.default_rng(2)
    xyz = rng.uniform(0.0, 4.0, (n, 3))
    rgb = np.clip(xyz[:, :1] * 60.0, 0, 255).astype(np.uint8).repeat(3, axis=1)
    result = app.voxel_color_roughness(xyz, rgb, voxel=0.1)
    assert result['roughness_median'] is not None
    assert result['roughness_median'] < 5.0


def test_roughness_validation_and_empty():
    with pytest.raises(ValueError):
        app.voxel_color_roughness(np.zeros((1, 3)), np.zeros((1, 3)), voxel=0.0)
    with pytest.raises(ValueError):
        app.voxel_color_roughness(np.zeros((1, 3)), np.zeros((1, 3)),
                                  min_points=1)
    with pytest.raises(ValueError):
        app.voxel_color_roughness(np.zeros((2, 3)), np.zeros((3, 3)))
    empty = app.voxel_color_roughness(np.zeros((0, 3)), np.zeros((0, 3)))
    assert empty['voxels_scored'] == 0
    assert empty['roughness_median'] is None


def test_evaluate_reports_coverage_and_retention():
    xyz = np.array([[0.0, 0.0, 0.0], [0.05, 0.0, 0.0], [5.0, 5.0, 5.0]])
    rgb = np.array([[200, 40, 40], [200, 40, 40], [128, 128, 128]],
                   dtype=np.uint8)  # third point carries the unseen default
    colour_img = np.zeros((8, 8, 3), dtype=np.uint8)
    colour_img[:, :, 0] = 160
    report = app.evaluate(xyz, rgb, images=[colour_img])
    assert report['points'] == 3
    assert report['colored'] == 2
    assert report['coverage'] == pytest.approx(2.0 / 3.0)
    assert report['chroma_mean'] == pytest.approx(160.0)
    assert report['chroma_retention'] == pytest.approx(1.0)


def test_evaluate_mono_images_disable_retention():
    xyz = np.zeros((2, 3))
    rgb = np.array([[10, 10, 10], [20, 20, 20]], dtype=np.uint8)
    mono = np.full((8, 8), 90, dtype=np.uint8)
    report = app.evaluate(xyz, rgb, images=[mono])
    assert report['chroma_retention'] is None


def test_planar_roughness_is_optional_and_detects_planar_pepper():
    rng = np.random.default_rng(7)
    xy = rng.uniform(0.0, 0.07, (80, 2))
    xyz = np.column_stack([xy, rng.normal(0.0, 0.0002, 80)])
    rgb = np.full((80, 3), 100, dtype=np.uint8)
    rgb[:8] = 240
    ordinary = app.evaluate(xyz, rgb, voxel=0.08)
    planar = app.evaluate(
        xyz, rgb, voxel=0.08, planar_roughness=True,
        planar_min_points=10)
    assert 'planar_roughness' not in ordinary
    assert planar['planar_points'] == 80
    assert planar['planar_fraction'] == pytest.approx(1.0)
    assert planar['planar_roughness']['roughness_p90'] > 30.0


def test_planar_roughness_rejects_line_like_voxel():
    xyz = np.column_stack([
        np.linspace(0.0, 0.07, 20), np.zeros(20), np.zeros(20)])
    rgb = np.full((20, 3), 100, dtype=np.uint8)
    report = app.evaluate(
        xyz, rgb, voxel=0.08, planar_roughness=True,
        planar_min_points=10)
    assert report['planar_points'] == 0
    assert report['planar_roughness']['voxels_scored'] == 0
