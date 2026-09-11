#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Re-pose a fixed extracted image set with another body trajectory."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'tools' / 'gaussian_splatting'
from lidarslam_benchmark_tools.gaussian_splatting import extract_posed_images as extractor
from lidarslam_benchmark_tools.gaussian_splatting import posed_images as pi


def repose(template_path: Path, trajectory_path: Path,
           extrinsic_path: Path, output_path: Path,
           max_extrapolation: float = 0.05) -> dict:
    """Write template images/intrinsics with poses from ``trajectory_path``."""
    template = json.loads(template_path.read_text())
    intrinsics = pi.CameraIntrinsics(
        width=int(template['w']), height=int(template['h']),
        fx=float(template['fl_x']), fy=float(template['fl_y']),
        cx=float(template['cx']), cy=float(template['cy']),
        distortion=tuple(float(template.get(key, 0.0))
                         for key in ('k1', 'k2', 'p1', 'p2', 'k3')),
    )
    trajectory = pi.read_tum_trajectory(trajectory_path)
    body_T_camera = extractor.load_extrinsic(extrinsic_path)
    frames = []
    for index, source in enumerate(template['frames']):
        stamp = float(source['stamp'])
        world_T_camera = extractor.resolve_world_T_camera(
            stamp, trajectory, body_T_camera,
            max_extrapolation=max_extrapolation)
        if world_T_camera is None:
            raise ValueError(
                f'template frame {index} at {stamp:.9f} is outside trajectory')
        image_path = (template_path.parent / source['file_path']).resolve()
        frames.append(pi.PosedImage(str(image_path), world_T_camera, stamp))
    if output_path.exists():
        raise ValueError(f'refusing to overwrite: {output_path}')
    pi.write_transforms(output_path, intrinsics, frames)
    return {'frames': len(frames), 'output': str(output_path.resolve())}


def main() -> int:
    """Run the CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--template', type=Path, required=True)
    parser.add_argument('--trajectory', type=Path, required=True)
    parser.add_argument('--extrinsic', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-extrapolation', type=float, default=0.05)
    args = parser.parse_args()
    result = repose(
        args.template, args.trajectory, args.extrinsic, args.output,
        args.max_extrapolation)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError,
            json.JSONDecodeError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(2)
