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

"""Re-run robust camera colouring on an existing XYZ/RGB PLY map."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Optional, Sequence

# Sibling helpers stay importable regardless of which directory hosts the
# caller (same pattern as build_lidar_init.py).
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.append(str(_HERE))

import build_lidar_init as builder
import pointcloud_io as pcio


def select_paired_subset(xyz, stride: int):
    """Return a deterministic index-strided subset for paired A/B trials."""
    if stride < 1:
        raise ValueError('point stride must be at least one')
    return xyz[::stride]


def build_parser() -> argparse.ArgumentParser:
    """Construct the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True,
                        help='source PLY; XYZ is retained')
    parser.add_argument('--transforms', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--report', default=None)
    parser.add_argument('--point-stride', type=int, default=1,
                        help='deterministically use every Nth input point; '
                             'intended for paired parameter screening')
    parser.add_argument('--exposure-scale-limit', type=float, default=1.5)
    parser.add_argument('--max-samples', type=int, default=12)
    parser.add_argument('--min-samples', type=int, default=1)
    parser.add_argument('--image-margin', type=int, default=0)
    parser.add_argument('--vignette-gain-limit', type=float, default=1.0)
    parser.add_argument('--overlap-balance', action='store_true')
    parser.add_argument('--view-confidence', action='store_true')
    parser.add_argument('--normal-voxel', type=float, default=0.12)
    parser.add_argument('--min-view-cosine', type=float, default=0.0)
    parser.add_argument('--min-projected-scale', type=float, default=0.0)
    parser.add_argument('--view-score-power', type=float, default=1.0)
    parser.add_argument('--geometry-aware', action='store_true')
    parser.add_argument('--occlusion-margin-px', type=int, default=0)
    parser.add_argument('--depth-edge-margin-px', type=int, default=0)
    parser.add_argument('--depth-edge-tolerance', type=float, default=1.0)
    parser.add_argument('--depth-edge-relative-tolerance', type=float,
                        default=0.10)
    parser.add_argument('--dynamic-exclusion', action='store_true')
    parser.add_argument('--dynamic-mask-margin-px', type=int, default=2)
    parser.add_argument('--calibration-sigma-multiplier', type=float,
                        default=0.0)
    parser.add_argument('--max-uncertainty-margin-px', type=int, default=8)
    parser.add_argument('--no-normalize-exposure', action='store_false',
                        dest='normalize_exposure')
    return parser


def run(args: argparse.Namespace) -> dict:
    """Colour one existing map and return a compact coverage report."""
    xyz, _ = pcio.read_ply_xyz(args.input)
    input_points = len(xyz)
    xyz = select_paired_subset(xyz, args.point_stride)
    result = builder._colorize(
        xyz, args.transforms, robust=True,
        normalize_exposure=args.normalize_exposure,
        exposure_scale_limit=args.exposure_scale_limit,
        max_samples=args.max_samples,
        image_margin=args.image_margin,
        vignette_gain_limit=args.vignette_gain_limit,
        overlap_color_balance=args.overlap_balance,
        view_confidence=args.view_confidence,
        normal_voxel=args.normal_voxel,
        min_view_cosine=args.min_view_cosine,
        min_projected_scale=args.min_projected_scale,
        view_score_power=args.view_score_power,
        min_samples=args.min_samples, geometry_aware=args.geometry_aware,
        occlusion_margin_px=args.occlusion_margin_px,
        depth_edge_margin_px=args.depth_edge_margin_px,
        depth_edge_tolerance=args.depth_edge_tolerance,
        depth_edge_relative_tolerance=args.depth_edge_relative_tolerance,
        dynamic_exclusion=args.dynamic_exclusion,
        dynamic_mask_margin_px=args.dynamic_mask_margin_px,
        calibration_sigma_multiplier=args.calibration_sigma_multiplier,
        maximum_uncertainty_margin_px=args.max_uncertainty_margin_px,
        return_diagnostics=args.geometry_aware)
    if args.geometry_aware:
        rgb, seen, diagnostics = result
    else:
        rgb, seen = result
        diagnostics = None
    output = pcio.write_ply(args.out, xyz, rgb)
    report = {
        'input': str(args.input), 'output': str(output),
        'input_points': int(input_points),
        'points': int(len(xyz)), 'colored': int(seen.sum()),
        'point_stride': int(args.point_stride),
        'coverage': float(seen.mean()) if len(seen) else 0.0,
        'overlap_balance': bool(args.overlap_balance),
        'view_confidence': bool(args.view_confidence),
        'image_margin': int(args.image_margin),
        'vignette_gain_limit': float(args.vignette_gain_limit),
        'min_samples': int(args.min_samples),
        'geometry_aware': bool(args.geometry_aware),
        'fusion_diagnostics': diagnostics,
    }
    if args.report:
        path = Path(args.report)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + '\n')
    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    report = run(build_parser().parse_args(argv))
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
