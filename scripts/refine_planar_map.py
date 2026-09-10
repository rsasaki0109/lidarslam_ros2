#!/usr/bin/env python3
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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
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

"""Reduce map thickness by projecting safe planar voxels onto local planes."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'
from lidarslam_benchmark_tools.gaussian_splatting import pointcloud_io as pcio


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--voxel', type=float, default=1.0)
    parser.add_argument('--min-points', type=int, default=10)
    parser.add_argument('--max-planarity-ratio', type=float, default=0.06)
    parser.add_argument('--min-tangent-ratio', type=float, default=0.04)
    parser.add_argument('--max-distance', type=float, default=0.18)
    args = parser.parse_args()

    xyz, rgb = pcio.read_ply_xyz(args.input)
    refined, projected = pcio.project_planar_voxels(
        xyz, args.voxel, min_points=args.min_points,
        max_planarity_ratio=args.max_planarity_ratio,
        min_second_to_first_ratio=args.min_tangent_ratio,
        max_projection_distance=args.max_distance)
    pcio.write_ply(args.output, refined, rgb)
    count = int(projected.sum())
    print(f'projected {count}/{len(xyz)} points '
          f'({100.0 * count / max(1, len(xyz)):.1f}%) -> {args.output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
