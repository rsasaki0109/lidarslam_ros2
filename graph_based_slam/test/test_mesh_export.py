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

"""Tests for coloured mesh reconstruction (skipped when open3d is absent)."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:  # pragma: no cover - environment dependent
    o3d = None
    HAS_OPEN3D = False

pytestmark = pytest.mark.skipif(not HAS_OPEN3D, reason='open3d is not installed')

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import mesh_export

    return mesh_export


mesh_export = _load()


def test_cli_parser_accepts_thin_voxel():
    args = mesh_export._build_arg_parser().parse_args(
        ['input.ply', 'output.ply', '--thin-voxel', '0.1'])
    assert args.thin_voxel == pytest.approx(0.1)


def _red_sphere(n=3000):
    """Build a dense red sphere for both reconstructors."""
    sph = o3d.geometry.TriangleMesh.create_sphere(radius=1.0, resolution=20)
    pcd = sph.sample_points_uniformly(number_of_points=n)
    xyz = np.asarray(pcd.points)
    rgb = np.tile([220, 30, 30], (len(xyz), 1)).astype(np.uint8)
    return xyz, rgb


# --------------------------------------------------------------------------- #
# Poisson
# --------------------------------------------------------------------------- #
def test_poisson_builds_coloured_watertight_mesh():
    xyz, rgb = _red_sphere()
    mesh = mesh_export.reconstruct_mesh(xyz, rgb, method='poisson', depth=6)
    assert len(mesh.triangles) > 100
    assert mesh.has_vertex_colors()
    # Colours carried through: mean vertex colour is dominantly red.
    mean = np.asarray(mesh.vertex_colors).mean(axis=0)
    assert mean[0] > mean[1] and mean[0] > mean[2]


def test_poisson_density_trim_shrinks_mesh():
    xyz, rgb = _red_sphere()
    full = mesh_export.reconstruct_mesh(xyz, rgb, method='poisson', depth=6,
                                        density_quantile=0.0)
    trimmed = mesh_export.reconstruct_mesh(xyz, rgb, method='poisson', depth=6,
                                           density_quantile=0.2)
    # Trimming the 20% lowest-density verts removes the extrapolated balloon.
    assert len(trimmed.vertices) < len(full.vertices)


# --------------------------------------------------------------------------- #
# Ball pivoting
# --------------------------------------------------------------------------- #
def test_bpa_builds_surface_mesh_with_colours():
    xyz, rgb = _red_sphere()
    mesh = mesh_export.reconstruct_mesh(xyz, rgb, method='bpa')
    assert len(mesh.triangles) > 100
    assert mesh.has_vertex_colors()
    # BPA never invents geometry, so every vertex sits ~on the unit sphere.
    r = np.linalg.norm(np.asarray(mesh.vertices), axis=1)
    assert np.abs(r - 1.0).max() < 0.15


# --------------------------------------------------------------------------- #
# I/O round-trip + CLI path
# --------------------------------------------------------------------------- #
def test_write_and_read_back_mesh(tmp_path):
    xyz, rgb = _red_sphere()
    mesh = mesh_export.reconstruct_mesh(xyz, rgb, method='bpa')
    out = mesh_export.write_mesh(tmp_path / 'm.ply', mesh)
    assert out.exists()
    back = o3d.io.read_triangle_mesh(str(out))
    assert len(back.triangles) == len(mesh.triangles)
    assert back.has_vertex_colors()


def test_uncoloured_cloud_yields_uncoloured_mesh():
    xyz, _ = _red_sphere()
    mesh = mesh_export.reconstruct_mesh(xyz, None, method='bpa')
    assert len(mesh.triangles) > 100
    assert not mesh.has_vertex_colors()


# --------------------------------------------------------------------------- #
# Validation
# --------------------------------------------------------------------------- #
def test_bad_method_raises():
    xyz, rgb = _red_sphere(n=500)
    with pytest.raises(ValueError):
        mesh_export.reconstruct_mesh(xyz, rgb, method='marching')


def test_rgb_shape_mismatch_raises():
    xyz, _ = _red_sphere(n=500)
    with pytest.raises(ValueError):
        mesh_export.make_cloud(xyz, np.zeros((10, 3), dtype=np.uint8))
