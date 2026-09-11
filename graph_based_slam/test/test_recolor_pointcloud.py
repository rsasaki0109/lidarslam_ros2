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

"""Tests for deterministic paired recolouring subsets."""

from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = REPO_ROOT / 'tools' / 'colored_map'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import recolor_pointcloud as recolor  # noqa: E402, I100


def test_select_paired_subset_keeps_stable_input_indices():
    points = np.arange(30).reshape(10, 3)
    np.testing.assert_array_equal(
        recolor.select_paired_subset(points, 3), points[[0, 3, 6, 9]])
    np.testing.assert_array_equal(
        recolor.select_paired_subset(points, 1), points)


def test_select_paired_subset_rejects_invalid_stride():
    with np.testing.assert_raises(ValueError):
        recolor.select_paired_subset(np.zeros((2, 3)), 0)


def test_parser_exposes_default_off_screening_stride(tmp_path):
    args = recolor.build_parser().parse_args([
        '--input', str(tmp_path / 'in.ply'),
        '--transforms', str(tmp_path / 'transforms.json'),
        '--out', str(tmp_path / 'out.ply'),
    ])
    assert args.point_stride == 1
    assert args.occlusion_margin_px == 0
    assert args.depth_edge_margin_px == 0
