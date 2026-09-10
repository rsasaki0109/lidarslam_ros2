#!/usr/bin/env python3
"""Cut a compact object out of a trained 3DGS scene as a standalone actor .ply.

Phase 3's sprite actor is a flat billboard -- correct from the recorded heading
but it reveals its flatness as the viewpoint swings. A *volumetric* actor needs
real Gaussian geometry. The cheapest source on hand is the scene itself: this
tool selects the Gaussians inside an axis-aligned box and writes them as their
own INRIA ``.ply``, which ``actor_compositing.py --mode ply`` then places and
composites with correct parallax and depth-tested occlusion -- a photoreal,
genuinely 3D actor without any external asset.

The cropping (``crop_gaussians``) and INRIA IO are reused from
``actor_compositing`` / ``train_gsplat`` and are unit tested there; this is a
thin CLI wrapper.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from lidarslam_benchmark_tools.gaussian_splatting.actor_compositing import crop_gaussians, reorient_up_to_z
from lidarslam_benchmark_tools.gaussian_splatting.render_path import load_gaussian_ply
from lidarslam_benchmark_tools.gaussian_splatting.train_gsplat import export_ply


def build_parser() -> argparse.ArgumentParser:
    """Construct the CLI argument parser."""
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--ply', required=True, help='trained INRIA 3DGS scene .ply')
    p.add_argument('--out', required=True, help='output actor .ply')
    p.add_argument('--center', required=True,
                   help='object centre x,y in the scene/world frame (metres)')
    p.add_argument('--half-extent', type=float, default=0.6,
                   help='half-width of the crop box in x and y (metres)')
    p.add_argument('--z-range', default='-1e9,1e9',
                   help='z_min,z_max of the crop box (metres)')
    p.add_argument('--up-axis', default='z', choices=('x', 'y', 'z'),
                   help="scene up axis; reoriented to +z so the actor stands "
                        "upright (y for Tanks&Temples scenes)")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    args = build_parser().parse_args(argv)
    center = [float(v) for v in args.center.split(',')]
    z_range = [float(v) for v in args.z_range.split(',')]
    scene = load_gaussian_ply(args.ply)
    actor = reorient_up_to_z(crop_gaussians(scene, center, args.half_extent,
                                            z_range), args.up_axis)
    out = export_ply(args.out, actor['means'], actor['scales_log'],
                     actor['quats'], actor['opacities_logit'],
                     actor['colors_rgb'], actor['sh_rest'])
    means = np.asarray(actor['means'])
    extent = means.max(axis=0) - means.min(axis=0)
    print(f'wrote {out} ({means.shape[0]} gaussians, '
          f'extent {extent[0]:.2f}x{extent[1]:.2f}x{extent[2]:.2f} m)')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
