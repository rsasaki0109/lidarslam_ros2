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

"""Tests for the flythrough renderer's pure helpers (CPU, no torch/gsplat)."""

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
    import render_path
    import train_gsplat

    return render_path, train_gsplat


rp, tg = _load()


def _gaussian_set(n=5, k_rest=0):
    means = np.arange(n * 3, dtype=float).reshape(n, 3) * 0.1
    scales_log = np.linspace(-3.0, -1.0, n * 3).reshape(n, 3)
    quats = np.tile([1.0, 0.0, 0.0, 0.0], (n, 1))
    opac = np.linspace(-2.0, 2.0, n)
    colors = np.linspace(0.05, 0.95, n * 3).reshape(n, 3)
    sh_rest = None
    if k_rest:
        sh_rest = np.arange(n * k_rest * 3, dtype=float).reshape(n, k_rest, 3) * 0.01
    return means, scales_log, quats, opac, colors, sh_rest


# --------------------------------------------------------------------------- #
# INRIA .ply round-trip
# --------------------------------------------------------------------------- #
def test_load_gaussian_ply_roundtrip_band0(tmp_path):
    means, scales_log, quats, opac, colors, _ = _gaussian_set()
    out = tg.export_ply(tmp_path / 'g.ply', means, scales_log, quats, opac, colors)
    got = rp.load_gaussian_ply(out)
    np.testing.assert_allclose(got['means'], means, atol=1e-5)
    np.testing.assert_allclose(got['scales_log'], scales_log, atol=1e-5)
    np.testing.assert_allclose(got['quats'], quats, atol=1e-5)
    np.testing.assert_allclose(got['opacities_logit'], opac, atol=1e-5)
    np.testing.assert_allclose(got['colors_rgb'], colors, atol=1e-5)
    assert got['sh_rest'] is None


def test_load_gaussian_ply_roundtrip_sh_rest(tmp_path):
    means, scales_log, quats, opac, colors, sh_rest = _gaussian_set(k_rest=3)
    out = tg.export_ply(tmp_path / 'g.ply', means, scales_log, quats, opac,
                        colors, sh_rest)
    got = rp.load_gaussian_ply(out)
    np.testing.assert_allclose(got['sh_rest'], sh_rest, atol=1e-5)


# --------------------------------------------------------------------------- #
# SH degree inference
# --------------------------------------------------------------------------- #
def test_infer_sh_degree():
    assert rp.infer_sh_degree(None) is None
    assert rp.infer_sh_degree(np.zeros((2, 3, 3))) == 1
    assert rp.infer_sh_degree(np.zeros((2, 15, 3))) == 3
    with pytest.raises(ValueError):
        rp.infer_sh_degree(np.zeros((2, 4, 3)))


# --------------------------------------------------------------------------- #
# Rotation conversion
# --------------------------------------------------------------------------- #
def test_matrix_to_quat_roundtrip():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import posed_images as pi

    for omega in ([0.0, 0.0, 0.0], [0.3, 0.0, 0.0], [0.0, 3.0, 0.0],
                  [1.0, -2.0, 0.5], [0.0, 0.0, 3.1]):
        rot = tg.axis_angle_to_matrix(np.asarray(omega))
        q = rp.matrix_to_quat_xyzw(rot)
        np.testing.assert_allclose(pi.quat_to_matrix(q), rot, atol=1e-9)


# --------------------------------------------------------------------------- #
# Camera path
# --------------------------------------------------------------------------- #
def _translation_keys(points):
    poses = []
    for xyz in points:
        m = np.eye(4)
        m[:3, 3] = xyz
        poses.append(m)
    return poses


def test_path_endpoints_and_monotonic_x():
    keys = _translation_keys([[0, 0, 0], [1, 0, 0], [2, 0, 0]])
    path = rp.path_through_views(keys, 11, smooth_window=1)
    assert len(path) == 11
    np.testing.assert_allclose(path[0][:3, 3], [0, 0, 0], atol=1e-12)
    np.testing.assert_allclose(path[-1][:3, 3], [2, 0, 0], atol=1e-12)
    xs = [p[:3, 3][0] for p in path]
    assert all(b > a for a, b in zip(xs, xs[1:]))


def test_path_midpoint_lerp_no_smoothing():
    keys = _translation_keys([[0, 0, 0], [4, 2, -2]])
    path = rp.path_through_views(keys, 3, smooth_window=1)
    np.testing.assert_allclose(path[1][:3, 3], [2, 1, -1], atol=1e-12)


def test_path_smoothing_pulls_in_corner():
    keys = _translation_keys([[0, 0, 0], [1, 0, 0], [1, 1, 0]])
    sharp = rp.path_through_views(keys, 21, smooth_window=1)
    smooth = rp.path_through_views(keys, 21, smooth_window=5)
    corner = np.array([1.0, 0.0, 0.0])
    d_sharp = min(np.linalg.norm(p[:3, 3] - corner) for p in sharp)
    d_smooth = min(np.linalg.norm(p[:3, 3] - corner) for p in smooth)
    assert d_smooth > d_sharp  # the box filter rounds the corner


def test_path_rotation_slerp_endpoints():
    rot = tg.axis_angle_to_matrix(np.array([0.0, 0.0, 1.0]))
    a, b = np.eye(4), np.eye(4)
    b[:3, :3] = rot
    b[:3, 3] = [1, 0, 0]
    path = rp.path_through_views([a, b], 5, smooth_window=1)
    np.testing.assert_allclose(path[0][:3, :3], np.eye(3), atol=1e-9)
    np.testing.assert_allclose(path[-1][:3, :3], rot, atol=1e-9)
    half = tg.axis_angle_to_matrix(np.array([0.0, 0.0, 0.5]))
    np.testing.assert_allclose(path[2][:3, :3], half, atol=1e-9)


def test_path_validation_errors():
    keys = _translation_keys([[0, 0, 0], [1, 0, 0]])
    with pytest.raises(ValueError):
        rp.path_through_views(keys[:1], 5)
    with pytest.raises(ValueError):
        rp.path_through_views(keys, 1)
    with pytest.raises(ValueError):
        rp.path_through_views(keys, 5, smooth_window=4)


# --------------------------------------------------------------------------- #
# Ping-pong frame order
# --------------------------------------------------------------------------- #
def test_ping_pong_indices():
    assert rp.ping_pong_indices(4) == [0, 1, 2, 3, 2, 1]
    assert rp.ping_pong_indices(2) == [0, 1]
    assert rp.ping_pong_indices(1) == [0]


# --------------------------------------------------------------------------- #
# Intrinsics scaling + CLI defaults
# --------------------------------------------------------------------------- #
def test_scale_intrinsics_quarter():
    K = np.array([[1000.0, 0.0, 1224.0], [0.0, 1000.0, 1024.0], [0.0, 0.0, 1.0]])
    K2, w, h = rp.scale_intrinsics(K, 2448, 2048, 0.25)
    assert (w, h) == (612, 512)
    np.testing.assert_allclose(K2[0], [250.0, 0.0, 306.0])
    np.testing.assert_allclose(K2[1], [0.0, 250.0, 256.0])
    np.testing.assert_allclose(K2[2], [0.0, 0.0, 1.0])


def test_scale_intrinsics_identity_and_errors():
    K = np.eye(3)
    K2, w, h = rp.scale_intrinsics(K, 640, 480, 1.0)
    assert (w, h) == (640, 480)
    np.testing.assert_allclose(K2, K)
    with pytest.raises(ValueError):
        rp.scale_intrinsics(K, 640, 480, 0.0)


def test_parser_defaults():
    args = rp.build_parser().parse_args(['--ply', 'a.ply', '--transforms', 't.json'])
    assert args.scale == 1.0
    assert args.rotate == 0
    assert args.frames == 240
    assert not args.ping_pong


# --------------------------------------------------------------------------- #
# CPU rasteriser
# --------------------------------------------------------------------------- #
def _point_set(points, colors, size=0.05):
    from render_slam_3dgs_sidebyside import points_to_gaussians

    return points_to_gaussians(np.asarray(points, dtype=float),
                               np.asarray(colors, dtype=float), size)


def _centred_camera():
    K = np.array([[300.0, 0.0, 64.0], [0.0, 300.0, 48.0], [0.0, 0.0, 1.0]])
    return K, np.eye(4)[None], 128, 96


def test_render_frames_cpu_nearest_point_wins():
    gaussians = _point_set([[0.0, 0.0, 2.0], [0.0, 0.0, 5.0]],
                           [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    K, viewmats, width, height = _centred_camera()
    frames = rp.render_frames_cpu(gaussians, viewmats, K, width, height)
    assert frames.shape == (1, height, width, 3)
    np.testing.assert_array_equal(frames[0, 48, 64], [255, 0, 0])


def test_render_frames_cpu_background_is_black():
    gaussians = _point_set([[0.0, 0.0, 2.0]], [[1.0, 1.0, 1.0]])
    K, viewmats, width, height = _centred_camera()
    frames = rp.render_frames_cpu(gaussians, viewmats, K, width, height)
    assert frames[0, 0, 0].tolist() == [0, 0, 0]


def test_render_frames_cpu_behind_camera_is_culled():
    gaussians = _point_set([[0.0, 0.0, -2.0]], [[1.0, 1.0, 1.0]])
    K, viewmats, width, height = _centred_camera()
    frames = rp.render_frames_cpu(gaussians, viewmats, K, width, height)
    assert int(frames.sum()) == 0


def test_render_frames_cpu_far_points_shrink():
    K, viewmats, width, height = _centred_camera()

    def lit_pixels(depth):
        gaussians = _point_set([[0.0, 0.0, depth]], [[1.0, 1.0, 1.0]], size=0.05)
        frames = rp.render_frames_cpu(gaussians, viewmats, K, width, height)
        return int((frames[0].sum(axis=2) > 0).sum())

    assert lit_pixels(1.0) > lit_pixels(8.0) > 0


def test_render_frames_cpu_rejects_sh_and_bad_supersample():
    gaussians = _point_set([[0.0, 0.0, 2.0]], [[1.0, 0.0, 0.0]])
    K, viewmats, width, height = _centred_camera()
    with pytest.raises(ValueError):
        rp.render_frames_cpu(dict(gaussians, sh_rest=np.zeros((1, 3, 3))),
                             viewmats, K, width, height)
    with pytest.raises(ValueError):
        rp.render_frames_cpu(gaussians, viewmats, K, width, height,
                             supersample=0)
    with pytest.raises(ValueError, match='soft_edge_px'):
        rp.render_frames_cpu(gaussians, viewmats, K, width, height,
                             soft_edge_px=-0.1)


def test_render_frames_cpu_soft_edge_expands_with_fade():
    gaussians = _point_set(
        [[0.0, 0.0, 2.0]], [[1.0, 1.0, 1.0]], size=0.03)
    K = np.array([[20.0, 0.0, 8.0],
                  [0.0, 20.0, 8.0],
                  [0.0, 0.0, 1.0]])
    viewmats = np.eye(4)[None]
    opaque = rp.render_frames_cpu(
        gaussians, viewmats, K, 16, 16, supersample=2)
    explicit_off = rp.render_frames_cpu(
        gaussians, viewmats, K, 16, 16, supersample=2,
        soft_edge_px=0.0)
    softened = rp.render_frames_cpu(
        gaussians, viewmats, K, 16, 16, supersample=2,
        soft_edge_px=1.0)
    assert np.count_nonzero(softened) > np.count_nonzero(opaque)
    np.testing.assert_array_equal(explicit_off, opaque)
    assert softened[0, 8, 8, 0] >= opaque[0, 8, 8, 0]
    assert np.any((softened > 0) & (softened < 255))


def test_render_frames_cpu_surface_splat_aligns_ellipse_to_plane():
    gaussians = _point_set(
        [[0.0, 0.0, 2.0]], [[1.0, 1.0, 1.0]], size=0.03)
    gaussians['surface_normals'] = np.array([[1.0, 0.0, 0.0]])
    K = np.array([[40.0, 0.0, 16.0],
                  [0.0, 40.0, 16.0],
                  [0.0, 0.0, 1.0]])
    frame = rp.render_frames_cpu(
        gaussians, np.eye(4)[None], K, 32, 32, supersample=2,
        surface_splat=True, surface_aspect_limit=3.0)[0]
    yy, xx = np.nonzero(frame.sum(axis=2) > 0)
    assert np.ptp(yy) > np.ptp(xx)


def test_render_frames_cpu_surface_splat_validation():
    gaussians = _point_set([[0.0, 0.0, 2.0]], [[1.0, 1.0, 1.0]])
    K, viewmats, width, height = _centred_camera()
    with pytest.raises(ValueError, match='surface_normals'):
        rp.render_frames_cpu(gaussians, viewmats, K, width, height,
                             surface_splat=True)
    with pytest.raises(ValueError, match='surface_aspect_limit'):
        rp.render_frames_cpu(gaussians, viewmats, K, width, height,
                             surface_aspect_limit=0.9)


def test_render_frames_dispatches_cpu_device():
    gaussians = _point_set([[0.0, 0.0, 2.0]], [[0.0, 1.0, 0.0]])
    K, viewmats, width, height = _centred_camera()
    frames = rp.render_frames(gaussians, viewmats, K, width, height,
                              device='cpu')
    np.testing.assert_array_equal(frames[0, 48, 64], [0, 255, 0])
