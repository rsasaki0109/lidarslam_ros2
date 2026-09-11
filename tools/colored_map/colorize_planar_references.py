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

"""Colour planar map regions from FAST-LIVO2-style reference patches."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

# train_gsplat (the transforms.json loader) stays on the 3DGS side; make its
# directory importable regardless of which directory hosts the caller.
_GS_DIR = Path(__file__).resolve().parents[1] / 'gaussian_splatting'
if str(_GS_DIR) not in sys.path:
    sys.path.append(str(_GS_DIR))

import plane_patch_warp as ppw  # noqa: E402
import pointcloud_io as pcio  # noqa: E402
import train_gsplat as tg  # noqa: E402


def luminance(image: np.ndarray) -> np.ndarray:
    """Convert mono/RGB image data to float32 luminance."""
    source = np.asarray(image, dtype=np.float32)
    if source.ndim == 2:
        return source
    if source.ndim != 3 or source.shape[2] < 3:
        raise ValueError(f'image must be HxW or HxWx3, got {source.shape}')
    return np.tensordot(
        source[:, :, :3], np.array([0.299, 0.587, 0.114], np.float32),
        axes=([-1], [0])).astype(np.float32)


def normalize_exposures(images: list[np.ndarray], limit: float = 1.5
                        ) -> list[np.ndarray]:
    """Scale images to their shared median luminance, matching fallback RGB."""
    medians = np.asarray([pcio._median_luminance(image) for image in images])
    valid = medians > 1.0e-6
    scales = np.ones(len(images), dtype=np.float32)
    if valid.any():
        target = float(np.median(medians[valid]))
        scales[valid] = np.clip(target / medians[valid], 1.0 / limit, limit)
    return [np.clip(np.asarray(image, np.float32) * scale, 0.0, 255.0)
            for image, scale in zip(images, scales)]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pointcloud', type=Path, required=True)
    parser.add_argument('--transforms', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--folds', type=int, default=1,
                        help='use all views by default; set 2 for evaluation')
    parser.add_argument('--fold', type=int, default=0)
    parser.add_argument('--voxel-size', type=float, default=1.0)
    parser.add_argument('--min-points', type=int, default=10)
    parser.add_argument('--max-views', type=int, default=6)
    parser.add_argument('--score-margin', type=float, default=0.12)
    parser.add_argument('--mode', choices=('filter', 'replace'), default='filter')
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.folds < 1 or not 0 <= args.fold < args.folds:
        raise SystemExit('--folds must be >= 1 and --fold must be valid')
    import imageio as iio

    points, _ = pcio.read_ply_xyz(args.pointcloud)
    dataset = tg.load_transforms(args.transforms)
    selected = [index for index in range(len(dataset['image_paths']))
                if index % args.folds == args.fold]
    if len(selected) < 2:
        raise SystemExit('at least two selected camera views are required')
    images = [np.asarray(iio.imread(dataset['image_paths'][index]))
              for index in selected]
    views = np.asarray(dataset['viewmats'], dtype=np.float64)[selected]
    K = np.asarray(dataset['K'], dtype=np.float64)
    normalized = normalize_exposures(images)
    references, view_mask = ppw.select_planar_voxel_references(
        points, [luminance(image) for image in normalized], K, views,
        voxel_size=args.voxel_size, min_points=args.min_points,
        max_views=args.max_views, return_view_mask=True,
        score_margin=args.score_margin)
    fallback, seen = pcio.colorize_by_projection_robust(
        points, views, K, images, dataset['width'], dataset['height'],
        observation_mask=(view_mask if args.mode == 'filter' else None))
    if args.mode == 'replace':
        colours, updated = ppw.apply_reference_colours(
            points, normalized, K, views, references, fallback)
    else:
        colours = fallback
        updated = np.any(~view_mask, axis=1)
    pcio.write_ply(args.out, points, colours)
    print(f'points={len(points)} fallback_seen={seen.mean():.4f} '
          f'planar_selected={(references >= 0).mean():.4f} '
          f'planar_filtered={updated.mean():.4f} mode={args.mode} out={args.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
