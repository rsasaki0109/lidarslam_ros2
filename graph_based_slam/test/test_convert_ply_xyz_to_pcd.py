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


"""Tests for the float64 PLY to float32 PCD benchmark adapter."""

import importlib.util
from pathlib import Path
import struct

import numpy as np
import pytest

try:
    import open3d  # noqa: F401
    HAS_OPEN3D = True
except ImportError:  # pragma: no cover - environment dependent
    HAS_OPEN3D = False

pytestmark = pytest.mark.skipif(not HAS_OPEN3D, reason='open3d is not installed')

ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'convert_ply_xyz_to_pcd.py'
SPEC = importlib.util.spec_from_file_location('convert_ply_xyz_to_pcd', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _write_double_xyz_ply(path: Path, points: list[tuple[float, float, float]]):
    header = (
        'ply\nformat binary_little_endian 1.0\n'
        f'element vertex {len(points)}\n'
        'property double x\nproperty double y\nproperty double z\n'
        'end_header\n'
    ).encode('ascii')
    with path.open('wb') as stream:
        stream.write(header)
        for point in points:
            stream.write(struct.pack('<ddd', *point))


def test_double_ply_is_emitted_as_exact_float_xyz_binary_pcd(tmp_path):
    points = [(1.25, -2.5, 3.75), (4.5, 5.25, -6.0)]
    ply = tmp_path / 'input.ply'
    pcd = tmp_path / 'output.pcd'
    _write_double_xyz_ply(ply, points)

    xyz = MODULE.load_xyz(ply)
    MODULE.write_binary_pcd(pcd, xyz)

    payload = pcd.read_bytes()
    header, binary = payload.split(b'DATA binary\n', 1)
    assert b'FIELDS x y z' in header
    assert b'POINTS 2' in header
    assert len(binary) == 2 * 3 * 4
    np.testing.assert_allclose(
        np.frombuffer(binary, dtype='<f4').reshape(-1, 3), points)


def test_non_finite_vertex_is_rejected(tmp_path):
    ply = tmp_path / 'bad.ply'
    _write_double_xyz_ply(ply, [(1.0, float('nan'), 2.0)])
    try:
        MODULE.load_xyz(ply)
    except ValueError as exc:
        assert 'non-finite' in str(exc)
    else:
        raise AssertionError('non-finite PLY vertex was accepted')
