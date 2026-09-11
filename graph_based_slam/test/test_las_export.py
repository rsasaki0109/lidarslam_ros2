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

"""Tests for LAS/LAZ export (skipped when laspy is absent)."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np
import pytest

try:
    import laspy
    HAS_LASPY = True
except ImportError:  # pragma: no cover - environment dependent
    laspy = None
    HAS_LASPY = False

pytestmark = pytest.mark.skipif(not HAS_LASPY, reason='laspy is not installed')

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'


def _load():
    if str(TOOL_DIR) not in sys.path:
        sys.path.insert(0, str(TOOL_DIR))
    import las_export

    return las_export


le = _load()


def _cloud(n=200):
    rng = np.random.default_rng(0)
    xyz = rng.uniform(-10, 10, (n, 3))
    rgb = rng.integers(0, 256, (n, 3)).astype(np.uint8)
    return xyz, rgb


# --------------------------------------------------------------------------- #
# Round-trip: coordinates and colour survive
# --------------------------------------------------------------------------- #
def test_las_roundtrip_xyz_and_rgb(tmp_path):
    xyz, rgb = _cloud()
    out = le.write_las(tmp_path / 'c.las', xyz, rgb, scale=0.001)
    las = laspy.read(str(out))
    assert len(las.x) == len(xyz)
    # mm scale -> sub-mm agreement on coordinates
    np.testing.assert_allclose(np.c_[las.x, las.y, las.z], xyz, atol=1e-3)
    # 16-bit colour maps back to the original 8-bit (stored as *257)
    back = np.c_[las.red, las.green, las.blue] // 257
    np.testing.assert_array_equal(back.astype(np.uint8), rgb)


def test_las_point_format_has_rgb(tmp_path):
    xyz, rgb = _cloud(50)
    out = le.write_las(tmp_path / 'c.las', xyz, rgb)
    las = laspy.read(str(out))
    assert las.header.point_format.id == 2  # XYZ + intensity + RGB


def test_las_without_rgb(tmp_path):
    xyz, _ = _cloud(50)
    out = le.write_las(tmp_path / 'c.las', xyz, None)
    las = laspy.read(str(out))
    assert len(las.x) == 50
    # colour dims exist in format 2 but were never written -> all zero
    assert int(np.asarray(las.red).max()) == 0


# --------------------------------------------------------------------------- #
# Georeferencing origin VLR
# --------------------------------------------------------------------------- #
def test_origin_vlr_roundtrip(tmp_path):
    xyz, rgb = _cloud(50)
    out = le.write_las(tmp_path / 'c.las', xyz, rgb,
                       origin_lat=35.6812, origin_lon=139.7671)
    got = le.read_origin(out)
    assert got is not None
    lat, lon = got
    assert abs(lat - 35.6812) < 1e-9
    assert abs(lon - 139.7671) < 1e-9


def test_no_origin_means_no_vlr(tmp_path):
    xyz, rgb = _cloud(50)
    out = le.write_las(tmp_path / 'c.las', xyz, rgb)
    assert le.read_origin(out) is None


# --------------------------------------------------------------------------- #
# Thinning + LAZ compression
# --------------------------------------------------------------------------- #
def test_thin_voxel_reduces_points(tmp_path):
    a = np.random.default_rng(1).normal(0.5, 0.01, (100, 3))
    b = a + np.array([20.0, 0.0, 0.0])
    xyz = np.vstack([a, b])
    out = le.write_las(tmp_path / 'c.las', xyz, thin_voxel=1.0)
    las = laspy.read(str(out))
    assert len(las.x) == 2


def test_laz_compression_when_backend_present(tmp_path):
    pytest.importorskip('lazrs')
    xyz, rgb = _cloud(200)
    out = le.write_las(tmp_path / 'c.laz', xyz, rgb)
    las = laspy.read(str(out))
    assert len(las.x) == 200
    # LAZ should be smaller than the equivalent LAS for a 200-pt coloured cloud
    las_path = le.write_las(tmp_path / 'c.las', xyz, rgb)
    assert out.stat().st_size < las_path.stat().st_size


# --------------------------------------------------------------------------- #
# Validation
# --------------------------------------------------------------------------- #
def test_bad_xyz_shape_raises(tmp_path):
    with pytest.raises(ValueError):
        le.write_las(tmp_path / 'c.las', np.zeros((4, 2)))


def test_rgb_mismatch_raises(tmp_path):
    with pytest.raises(ValueError):
        le.write_las(tmp_path / 'c.las', np.zeros((4, 3)),
                     np.zeros((3, 3), dtype=np.uint8))
