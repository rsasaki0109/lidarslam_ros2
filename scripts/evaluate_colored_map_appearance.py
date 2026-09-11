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

"""Score how a coloured map *looks*: washout, pepper noise, and coverage.

The held-out reprojection error (``evaluate_heldout_point_colors.py``)
measures fidelity to individual camera views, but it misses the two defects
that dominate perceived quality: multi-view averaging that washes chroma out
of the whole map, and isolated wrong-colour points ("pepper") sprinkled over
smooth surfaces. This report measures both directly on the map:

- ``chroma``: mean per-point RGB channel range of the coloured points, and,
  when posed source images are given, ``chroma_retention`` — map chroma over
  source-image chroma. Washed-out averaging drives retention well below 1.
- ``roughness``: per-voxel colour standard deviation among the coloured
  points that share a voxel (median / p90 across voxels). Real texture is
  organised across voxels; pepper noise raises the *within*-voxel spread.
- ``coverage``: the coloured fraction, so the two scores above cannot be
  gamed by discarding points.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_DIR = REPO_ROOT / 'tools' / 'gaussian_splatting'
if str(TOOL_DIR) not in sys.path:
    sys.path.insert(0, str(TOOL_DIR))

import pointcloud_io as pcio  # noqa: E402


def channel_range_chroma(rgb: np.ndarray) -> np.ndarray:
    """Per-point chroma as the max-min spread across RGB channels."""
    values = np.asarray(rgb)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError(f'rgb must be Nx3, got {values.shape}')
    return np.ptp(values.astype(np.int16), axis=1).astype(np.float64)


def image_chroma(images, pixel_stride: int = 4) -> float:
    """Mean channel-range chroma over the sampled pixels of all images.

    Mono images (2-D or single-channel) contribute zero chroma, matching how
    they colour a map. ``pixel_stride`` subsamples rows and columns.
    """
    if pixel_stride < 1:
        raise ValueError('pixel_stride must be >= 1')
    totals = []
    for image in images:
        arr = np.asarray(image)
        sub = arr[::pixel_stride, ::pixel_stride]
        if sub.ndim == 2 or sub.shape[2] < 3:
            totals.append(0.0)
            continue
        totals.append(float(np.mean(np.ptp(
            sub[:, :, :3].astype(np.int16), axis=2))))
    if not totals:
        raise ValueError('no images given')
    return float(np.mean(totals))


def voxel_color_roughness(xyz: np.ndarray, rgb: np.ndarray,
                          voxel: float = 0.08, min_points: int = 3) -> dict:
    """Within-voxel colour spread of a coloured cloud.

    Groups points into a ``voxel``-sized grid and, for every voxel holding at
    least ``min_points`` points, computes the per-channel standard deviation
    of its colours averaged over channels. Smooth, correctly coloured
    surfaces score low even when the map as a whole is colourful; occlusion
    fringes and specular one-offs ("pepper") raise the within-voxel spread.
    Returns median and p90 across voxels plus the voxel count.
    """
    if voxel <= 0.0:
        raise ValueError('voxel must be positive')
    if min_points < 2:
        raise ValueError('min_points must be >= 2')
    pts = np.asarray(xyz, dtype=np.float64)
    cols = np.asarray(rgb, dtype=np.float64)
    if len(pts) != len(cols):
        raise ValueError('xyz and rgb must have equal length')
    empty = {'voxels_scored': 0, 'roughness_median': None, 'roughness_p90': None}
    if len(pts) == 0:
        return empty
    keys = np.floor(pts / voxel).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True,
                                   return_counts=True)
    order = np.argsort(inverse, kind='stable')
    grouped = cols[order]
    boundaries = np.concatenate(([0], np.cumsum(counts)[:-1]))
    sums = np.add.reduceat(grouped, boundaries, axis=0)
    sq_sums = np.add.reduceat(grouped ** 2, boundaries, axis=0)
    n = counts.astype(np.float64)[:, None]
    variance = np.maximum(sq_sums / n - (sums / n) ** 2, 0.0)
    stds = np.sqrt(variance).mean(axis=1)
    scored = stds[counts >= min_points]
    if scored.size == 0:
        return empty
    return {
        'voxels_scored': int(scored.size),
        'roughness_median': float(np.median(scored)),
        'roughness_p90': float(np.percentile(scored, 90)),
    }


def evaluate(xyz: np.ndarray, rgb: np.ndarray, *,
             default_rgb=(128, 128, 128), voxel: float = 0.08,
             images=None, pixel_stride: int = 4,
             planar_roughness: bool = False,
             planar_min_points: int = 10,
             planar_max_ratio: float = 0.06,
             planar_min_second_ratio: float = 0.04) -> dict:
    """Assemble the appearance report for one coloured cloud."""
    default = np.asarray(default_rgb, dtype=np.uint8)
    seen = np.any(np.asarray(rgb) != default[None, :], axis=1)
    coloured = np.asarray(rgb)[seen]
    chroma = channel_range_chroma(coloured) if len(coloured) else np.zeros(0)
    report = {
        'points': int(len(xyz)),
        'colored': int(seen.sum()),
        'coverage': float(seen.mean()) if len(seen) else 0.0,
        'chroma_mean': float(chroma.mean()) if chroma.size else 0.0,
        'chroma_p50': float(np.median(chroma)) if chroma.size else 0.0,
        'roughness': voxel_color_roughness(
            np.asarray(xyz)[seen], coloured, voxel=voxel),
        'voxel': voxel,
    }
    if images is not None:
        source = image_chroma(images, pixel_stride)
        report['image_chroma_mean'] = source
        report['chroma_retention'] = (
            float(report['chroma_mean'] / source) if source >= 2.0 else None)
    if planar_roughness:
        coloured_xyz = np.asarray(xyz)[seen]
        _, planar = pcio.project_planar_voxels(
            coloured_xyz, voxel, min_points=planar_min_points,
            max_planarity_ratio=planar_max_ratio,
            min_second_to_first_ratio=planar_min_second_ratio,
            max_projection_distance=float('inf'))
        report['planar_points'] = int(planar.sum())
        report['planar_fraction'] = (
            float(planar.mean()) if len(planar) else 0.0)
        report['planar_roughness'] = voxel_color_roughness(
            coloured_xyz[planar], coloured[planar], voxel=voxel)
        report['planar_parameters'] = {
            'min_points': planar_min_points,
            'max_planarity_ratio': planar_max_ratio,
            'min_second_to_first_ratio': planar_min_second_ratio,
        }
    return report


def main() -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0])
    parser.add_argument('--pointcloud', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--transforms', type=Path, default=None,
                        help='posed_images transforms.json; enables '
                             'chroma_retention against the source images')
    parser.add_argument('--voxel', type=float, default=0.08,
                        help='roughness grouping voxel size (m)')
    parser.add_argument('--view-stride', type=int, default=5,
                        help='sample every Nth source image for image chroma')
    parser.add_argument('--default-rgb', type=int, nargs=3,
                        default=(128, 128, 128))
    parser.add_argument('--planar-roughness', action='store_true',
                        help='also score colour roughness in PCA-planar voxels')
    parser.add_argument('--planar-min-points', type=int, default=10)
    parser.add_argument('--planar-max-ratio', type=float, default=0.06)
    parser.add_argument('--planar-min-second-ratio', type=float, default=0.04)
    args = parser.parse_args()
    if args.view_stride < 1:
        raise SystemExit('--view-stride must be >= 1')

    xyz, rgb = pcio.read_ply_xyz(args.pointcloud)
    if rgb is None:
        raise SystemExit('point cloud has no RGB colours')
    images = None
    if args.transforms is not None:
        import imageio.v3 as iio
        import train_gsplat as tg
        dataset = tg.load_transforms(args.transforms)
        images = [np.asarray(iio.imread(path))
                  for path in dataset['image_paths'][::args.view_stride]]
    report = evaluate(xyz, rgb, default_rgb=tuple(args.default_rgb),
                      voxel=args.voxel, images=images,
                      planar_roughness=args.planar_roughness,
                      planar_min_points=args.planar_min_points,
                      planar_max_ratio=args.planar_max_ratio,
                      planar_min_second_ratio=args.planar_min_second_ratio)
    report['pointcloud'] = str(args.pointcloud.resolve())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
