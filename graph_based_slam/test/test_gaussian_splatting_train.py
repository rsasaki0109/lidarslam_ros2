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

"""Tests for the 3DGS gsplat trainer pure helpers (no torch/CUDA needed)."""

from __future__ import annotations

from pathlib import Path
import struct
import sys

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import train_gsplat

    return train_gsplat


tg = _load()
pi = tg.pi


# --------------------------------------------------------------------------- #
# looks_at_poses
# --------------------------------------------------------------------------- #
def test_looks_at_poses_count_and_radius():
    poses = tg.looks_at_poses(radius=3.0, count=8)
    assert len(poses) == 8
    for c2w in poses:
        assert abs(np.linalg.norm(c2w[:3, 3]) - 3.0) < 1e-6


def test_looks_at_poses_forward_points_at_origin():
    for c2w in tg.looks_at_poses(radius=2.0, count=6, height=0.5):
        eye = c2w[:3, 3]
        forward = c2w[:3, 2]  # OpenCV +z is forward
        to_origin = -eye / np.linalg.norm(eye)
        assert float(np.dot(forward, to_origin)) > 0.9


def test_looks_at_poses_orthonormal_rotation():
    c2w = tg.looks_at_poses(radius=2.0, count=4)[0]
    R = c2w[:3, :3]
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-6)


# --------------------------------------------------------------------------- #
# load_transforms round-trips write_transforms and recovers OpenCV w2c
# --------------------------------------------------------------------------- #
def test_load_transforms_recovers_viewmat(tmp_path):
    intr = pi.CameraIntrinsics(64, 48, 50.0, 50.0, 32.0, 24.0)
    c2w_cv = pi.make_transform([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    (tmp_path / 'images').mkdir()
    (tmp_path / 'images' / '0.png').write_bytes(b'stub')
    frames = [pi.PosedImage('images/0.png', c2w_cv, 0.0)]
    pi.write_transforms(tmp_path / 'transforms.json', intr, frames)

    ds = tg.load_transforms(tmp_path / 'transforms.json')
    assert ds['width'] == 64 and ds['height'] == 48
    np.testing.assert_allclose(ds['K'], [[50, 0, 32], [0, 50, 24], [0, 0, 1]],
                               atol=1e-9)
    # viewmat should be the inverse of the original OpenCV c2w.
    np.testing.assert_allclose(ds['viewmats'][0], np.linalg.inv(c2w_cv), atol=1e-9)
    assert ds['image_paths'][0].name == '0.png'


# --------------------------------------------------------------------------- #
# export_ply
# --------------------------------------------------------------------------- #
def test_export_ply_header_and_roundtrip(tmp_path):
    means = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    scales_log = np.zeros((2, 3))
    quats = np.tile([1.0, 0.0, 0.0, 0.0], (2, 1))
    opac = np.array([0.0, 0.0])
    colors = np.array([[0.5, 0.5, 0.5], [1.0, 0.0, 0.0]])
    out = tg.export_ply(tmp_path / 'g.ply', means, scales_log, quats, opac, colors)

    raw = Path(out).read_bytes()
    header_end = raw.index(b'end_header\n') + len(b'end_header\n')
    header = raw[:header_end].decode('ascii')
    assert 'element vertex 2' in header
    assert 'property float f_dc_0' in header and 'property float rot_3' in header

    n_props = header.count('property float ')
    body = raw[header_end:]
    vals = struct.unpack('<' + 'f' * (2 * n_props), body)
    row0 = vals[:n_props]
    # x, y, z are the first three properties.
    np.testing.assert_allclose(row0[:3], [1.0, 2.0, 3.0], atol=1e-6)
    # grey (0.5) -> f_dc 0 for all three colour channels (indices 6,7,8).
    np.testing.assert_allclose(row0[6:9], [0.0, 0.0, 0.0], atol=1e-6)


def test_axis_angle_identity():
    np.testing.assert_allclose(tg.axis_angle_to_matrix([0, 0, 0]), np.eye(3), atol=1e-12)


def test_axis_angle_90deg_about_z():
    R = tg.axis_angle_to_matrix([0, 0, np.pi / 2])
    np.testing.assert_allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-9)


def test_axis_angle_is_orthonormal():
    R = tg.axis_angle_to_matrix([0.3, -0.7, 1.1])
    np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-9)
    assert abs(np.linalg.det(R) - 1.0) < 1e-9


def test_export_ply_vertex_count(tmp_path):
    n = 5
    out = tg.export_ply(
        tmp_path / 'g.ply', np.zeros((n, 3)), np.zeros((n, 3)),
        np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros(n), np.full((n, 3), 0.5),
    )
    assert f'element vertex {n}'.encode() in Path(out).read_bytes()


def test_export_ply_sh_rest_fields_and_order(tmp_path):
    n, k_rest = 4, 15  # SH degree 3 -> (3+1)^2 - 1 = 15 higher coeffs
    sh_rest = np.arange(n * k_rest * 3, dtype=np.float32).reshape(n, k_rest, 3)
    out = tg.export_ply(
        tmp_path / 'g.ply', np.zeros((n, 3)), np.zeros((n, 3)),
        np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros(n), np.full((n, 3), 0.5),
        sh_rest,
    )
    raw = Path(out).read_bytes()
    header = raw[:raw.index(b'end_header\n')].decode('ascii')
    names = [ln.split()[-1] for ln in header.splitlines()
             if ln.startswith('property float')]
    # 3 f_dc + 45 f_rest, channel-major (all R coeffs, then G, then B).
    assert names.count('f_dc_0') == 1
    assert sum(nm.startswith('f_rest_') for nm in names) == k_rest * 3
    body = np.frombuffer(raw[raw.index(b'end_header\n') + 11:],
                         dtype=np.float32).reshape(n, len(names))
    i0 = names.index('f_rest_0')
    # f_rest_0 = channel 0, coeff 0 ; f_rest_{k_rest} = channel 1, coeff 0
    np.testing.assert_allclose(body[:, i0], sh_rest[:, 0, 0])
    np.testing.assert_allclose(body[:, i0 + k_rest], sh_rest[:, 0, 1])


def test_export_ply_no_sh_rest_is_band0(tmp_path):
    n = 3
    out = tg.export_ply(
        tmp_path / 'g.ply', np.zeros((n, 3)), np.zeros((n, 3)),
        np.tile([1.0, 0, 0, 0], (n, 1)), np.zeros(n), np.full((n, 3), 0.5),
    )
    header = Path(out).read_bytes()
    header = header[:header.index(b'end_header')].decode('ascii')
    assert 'f_rest_' not in header


# --------------------------------------------------------------------------- #
# knn_scale_log (pure numpy/scipy; no torch)
# --------------------------------------------------------------------------- #
def test_knn_scale_log_shape_and_isotropic():
    pts = np.random.default_rng(0).normal(size=(50, 3))
    scales = tg.knn_scale_log(pts)
    assert scales.shape == (50, 3)
    # isotropic: the three columns are identical per point.
    np.testing.assert_allclose(scales[:, 0], scales[:, 1])
    np.testing.assert_allclose(scales[:, 0], scales[:, 2])


def test_knn_scale_log_matches_uniform_spacing():
    # Needs the real k-NN path; without scipy the helper falls back to a
    # global-spacing estimate (CI has no scipy), so skip there.
    pytest.importorskip('scipy.spatial')
    # Points spaced 1.0 apart on a line -> nearest-neighbour distance ~1.0.
    pts = np.stack([np.arange(20.0), np.zeros(20), np.zeros(20)], axis=1)
    scales = tg.knn_scale_log(pts, k=1)
    np.testing.assert_allclose(np.exp(scales[:, 0]).mean(), 1.0, atol=1e-6)


def test_knn_scale_log_dense_smaller_than_sparse():
    dense = np.random.default_rng(1).normal(scale=0.05, size=(100, 3))
    sparse = np.random.default_rng(2).normal(scale=5.0, size=(100, 3))
    assert tg.knn_scale_log(dense).mean() < tg.knn_scale_log(sparse).mean()


def test_knn_scale_log_fallback_is_uniform_isotropic(monkeypatch):
    # Force the scipy-absent branch (the path CI actually runs) and pin its
    # contract: a single global spacing applied to every point, isotropic, and
    # a warning so a silent degrade is visible. Real failures inside the k-NN
    # query are no longer swallowed -- only ImportError reaches this fallback.
    monkeypatch.setitem(sys.modules, 'scipy.spatial', None)
    pts = np.random.default_rng(0).normal(size=(40, 3))
    with pytest.warns(UserWarning, match='scipy.spatial unavailable'):
        scales = tg.knn_scale_log(pts)
    assert scales.shape == (40, 3)
    assert np.all(scales == scales[0])  # one global spacing, every point/axis


# --------------------------------------------------------------------------- #
# make_ssim / _photometric_loss (torch needed; skipped where unavailable)
# --------------------------------------------------------------------------- #
def test_make_ssim_identical_is_one():
    torch = pytest.importorskip('torch')
    img = torch.rand(32, 40, 3)
    ssim = tg.make_ssim(img.device)
    assert float(ssim(img, img)) == pytest.approx(1.0, abs=1e-4)


# --------------------------------------------------------------------------- #
# CLI parser (no torch)
# --------------------------------------------------------------------------- #
def test_parser_quality_flags_default_off():
    args = tg.build_parser().parse_args(['--transforms', 't', '--out', 'o'])
    assert args.sh_degree is None
    assert args.antialiased is False
    assert args.mcmc is False
    assert args.mcmc_cap == 500000


def test_parser_quality_flags_set():
    args = tg.build_parser().parse_args(
        ['--transforms', 't', '--out', 'o', '--sh-degree', '1',
         '--antialiased', '--mcmc', '--mcmc-cap', '300000']
    )
    assert args.sh_degree == 1
    assert args.antialiased is True
    assert args.mcmc is True and args.mcmc_cap == 300000


def test_parser_lidar_depth_lambda():
    args = tg.build_parser().parse_args(['--transforms', 't', '--out', 'o'])
    assert args.lidar_depth_lambda == 0.0
    args = tg.build_parser().parse_args(
        ['--transforms', 't', '--out', 'o', '--lidar-depth-lambda', '0.002'])
    assert args.lidar_depth_lambda == 0.002


def _depth_ds():
    return {'K': np.eye(3), 'width': 8, 'height': 8,
            'viewmats': [np.eye(4)], 'image_paths': [], 'groups': [0]}


def test_lidar_depth_requires_init_points():
    # The guard fires before any torch/GPU work, so it is CPU-testable.
    with pytest.raises(ValueError, match='requires an init point cloud'):
        tg.train_densify(_depth_ds(), init_points=None,
                         lidar_depth_lambda=0.5, iters=1, device='cpu')


def test_lidar_depth_incompatible_with_pose_optimisation():
    pts = np.zeros((4, 3), dtype=np.float32)
    with pytest.raises(ValueError, match='incompatible'):
        tg.train_densify(_depth_ds(), init_points=pts, lidar_depth_lambda=0.5,
                         optimize_extrinsic=True, iters=1, device='cpu')
    with pytest.raises(ValueError, match='incompatible'):
        tg.train_densify(_depth_ds(), init_points=pts, lidar_depth_lambda=0.5,
                         optimize_pose_groups=True, iters=1, device='cpu')


def test_make_ssim_noise_below_one():
    torch = pytest.importorskip('torch')
    a = torch.rand(32, 40, 3)
    b = torch.rand(32, 40, 3)
    ssim = tg.make_ssim(a.device)
    assert float(ssim(a, b)) < 0.5


def test_photometric_loss_mse_when_lambda_zero():
    torch = pytest.importorskip('torch')
    a, b = torch.rand(8, 8, 3), torch.rand(8, 8, 3)
    loss, mse = tg._photometric_loss(a, b, None, 0.0)
    assert float(loss) == pytest.approx(float(mse))
    assert float(mse) == pytest.approx(float(((a - b) ** 2).mean()))


def test_photometric_loss_blends_when_lambda_positive():
    torch = pytest.importorskip('torch')
    a, b = torch.rand(16, 16, 3), torch.rand(16, 16, 3)
    ssim = tg.make_ssim(a.device)
    loss, mse = tg._photometric_loss(a, b, ssim, 0.2)
    # mse is still the true MSE (reported for PSNR), loss is the blended term.
    assert float(mse) == pytest.approx(float(((a - b) ** 2).mean()))
    assert float(loss) != pytest.approx(float(mse))


# --------------------------------------------------------------------------- #
# load_transforms group indices (multi-session 'bag' key)
# --------------------------------------------------------------------------- #
def test_load_transforms_groups_from_bag_key(tmp_path):
    import json

    (tmp_path / 'images').mkdir()
    frames = []
    for i, bag in enumerate(['a', 'a', 'b', 'a', 'c']):
        (tmp_path / 'images' / f'{i}.png').write_bytes(b'stub')
        frames.append({'file_path': f'images/{i}.png',
                       'transform_matrix': np.eye(4).tolist(),
                       'bag': bag})
    doc = {'camera_model': 'OPENCV', 'w': 8, 'h': 6, 'fl_x': 5.0, 'fl_y': 5.0,
           'cx': 4.0, 'cy': 3.0, 'frames': frames}
    (tmp_path / 'transforms.json').write_text(json.dumps(doc))

    ds = tg.load_transforms(tmp_path / 'transforms.json')
    # group ids follow first appearance order of the 'bag' values
    assert ds['groups'] == [0, 0, 1, 0, 2]


def test_load_transforms_groups_default_zero(tmp_path):
    import json

    (tmp_path / 'images').mkdir()
    (tmp_path / 'images' / '0.png').write_bytes(b'stub')
    doc = {'camera_model': 'OPENCV', 'w': 8, 'h': 6, 'fl_x': 5.0, 'fl_y': 5.0,
           'cx': 4.0, 'cy': 3.0,
           'frames': [{'file_path': 'images/0.png',
                       'transform_matrix': np.eye(4).tolist()}]}
    (tmp_path / 'transforms.json').write_text(json.dumps(doc))

    ds = tg.load_transforms(tmp_path / 'transforms.json')
    assert ds['groups'] == [0]


def test_load_transforms_retains_fusion_metadata(tmp_path):
    import json

    (tmp_path / 'images').mkdir()
    (tmp_path / 'masks').mkdir()
    (tmp_path / 'images' / '0.png').write_bytes(b'stub')
    (tmp_path / 'masks' / '0.png').write_bytes(b'mask')
    calibration = {
        'accepted': True,
        'uncertainty_dt_s_xyz_m_rpy_rad': [0.01] * 7,
    }
    document = {
        'w': 8, 'h': 6, 'fl_x': 5.0, 'fl_y': 5.0,
        'cx': 4.0, 'cy': 3.0,
        'spatiotemporal_calibration': calibration,
        'frames': [{
            'file_path': 'images/0.png',
            'dynamic_mask_path': 'masks/0.png', 'timestamp': 1.25,
            'transform_matrix': np.eye(4).tolist(),
        }],
    }
    (tmp_path / 'transforms.json').write_text(json.dumps(document))
    dataset = tg.load_transforms(tmp_path / 'transforms.json')
    assert dataset['timestamps'].tolist() == [1.25]
    assert dataset['dynamic_mask_paths'][0] == (
        tmp_path / 'masks' / '0.png').resolve()
    assert dataset['spatiotemporal_calibration'] == calibration


def test_parser_pose_group_and_exposure_flags():
    args = tg.build_parser().parse_args(['--transforms', 't', '--out', 'o'])
    assert args.optimize_pose_groups is False
    assert args.optimize_exposure is False
    args = tg.build_parser().parse_args(
        ['--transforms', 't', '--out', 'o',
         '--optimize-pose-groups', '--optimize-exposure'])
    assert args.optimize_pose_groups is True
    assert args.optimize_exposure is True
