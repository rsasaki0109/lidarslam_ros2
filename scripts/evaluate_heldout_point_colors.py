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

"""Evaluate camera-coloured points on camera views excluded from colouring."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

from lidarslam_benchmark_tools.gaussian_splatting import pointcloud_io as pcio
from lidarslam_benchmark_tools.gaussian_splatting import train_gsplat as tg


def visible_point_samples(points: np.ndarray, viewmat: np.ndarray,
                          K: np.ndarray, width: int, height: int
                          ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return nearest point id and float pixel coordinates per occupied pixel."""
    pts = np.asarray(points, dtype=np.float64)
    vm = np.asarray(viewmat, dtype=np.float64)
    cam = pts @ vm[:3, :3].T + vm[:3, 3]
    z = cam[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        uf = K[0, 0] * cam[:, 0] / z + K[0, 2]
        vf = K[1, 1] * cam[:, 1] / z + K[1, 2]
    safe_uf = np.nan_to_num(uf, nan=-1.0, posinf=-1.0, neginf=-1.0)
    safe_vf = np.nan_to_num(vf, nan=-1.0, posinf=-1.0, neginf=-1.0)
    u = np.round(safe_uf).astype(np.int64)
    v = np.round(safe_vf).astype(np.int64)
    valid = (np.isfinite(uf) & np.isfinite(vf) & (z > 1e-6) &
             (u >= 0) & (u < width) & (v >= 0) & (v < height))
    ids = np.flatnonzero(valid)
    if ids.size == 0:
        empty = np.zeros(0, dtype=np.float64)
        return ids, empty, empty
    pixels = v[ids] * width + u[ids]
    order = np.lexsort((z[ids], pixels))
    sorted_pixels = pixels[order]
    first = np.concatenate(([True], sorted_pixels[1:] != sorted_pixels[:-1]))
    chosen = ids[order[first]]
    return chosen, uf[chosen], vf[chosen]


def exposure_scales(images: list[np.ndarray], limit: float = 1.5) -> np.ndarray:
    """Return the same clamped median-luminance scales as robust colouring."""
    if limit < 1.0:
        raise ValueError('limit must be >= 1')
    medians = np.asarray([pcio._median_luminance(image) for image in images])
    valid = medians > 1e-6
    scales = np.ones(len(images), dtype=np.float32)
    if valid.any():
        target = float(np.median(medians[valid]))
        scales[valid] = np.clip(target / medians[valid], 1.0 / limit, limit)
    return scales


def score_heldout_view(points: np.ndarray, colors: np.ndarray, seen: np.ndarray,
                       viewmat: np.ndarray, K: np.ndarray, image: np.ndarray,
                       exposure_scale: float = 1.0) -> tuple[np.ndarray, int]:
    """Return per-visible-point RGB Euclidean errors and visible count."""
    height, width = image.shape[:2]
    ids, uf, vf = visible_point_samples(points, viewmat, K, width, height)
    visible_count = int(ids.size)
    keep = seen[ids]
    ids, uf, vf = ids[keep], uf[keep], vf[keep]
    if ids.size == 0:
        return np.zeros(0, dtype=np.float32), visible_count
    observed = pcio._sample_pixels(
        image, uf, vf, width, height, 'edge-aware', 48.0)
    observed = np.clip(observed * exposure_scale, 0.0, 255.0)
    delta = colors[ids].astype(np.float32) - observed
    return np.linalg.norm(delta, axis=1), visible_count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pointcloud', type=Path, required=True)
    parser.add_argument('--transforms', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--folds', type=int, default=2)
    parser.add_argument('--holdout-fold', type=int, default=1)
    parser.add_argument('--view-stride', type=int, default=5)
    parser.add_argument('--use-pointcloud-colors', action='store_true',
                        help='score stored RGB instead of recolouring train views')
    parser.add_argument('--no-normalize-exposure', action='store_false',
                        dest='normalize_exposure',
                        help='compare raw camera RGB without per-view exposure gains')
    parser.add_argument('--exposure-scale-limit', type=float, default=1.5,
                        help='maximum exposure gain and reciprocal loss')
    args = parser.parse_args()
    if args.folds < 2 or not 0 <= args.holdout_fold < args.folds:
        raise SystemExit('--folds must be >= 2 and --holdout-fold must be valid')
    if args.view_stride < 1:
        raise SystemExit('--view-stride must be >= 1')

    import imageio.v3 as iio
    points, stored_colors = pcio.read_point_cloud_xyz(args.pointcloud)
    dataset = tg.load_transforms(args.transforms)
    viewmats = np.asarray(dataset['viewmats'], dtype=np.float64)
    K = np.asarray(dataset['K'], dtype=np.float64)
    images = [np.asarray(iio.imread(path)) for path in dataset['image_paths']]
    holdout = [i for i in range(len(images)) if i % args.folds == args.holdout_fold]
    holdout_set = set(holdout)
    train = [i for i in range(len(images)) if i not in holdout_set]
    if args.use_pointcloud_colors:
        if stored_colors is None:
            raise SystemExit(
                '--use-pointcloud-colors requires RGB in the point cloud')
        colors = stored_colors
        seen = np.ones(len(points), dtype=bool)
    else:
        colors, seen = pcio.colorize_by_projection_robust(
            points, viewmats[train], K,
            [images[i] for i in train], dataset['width'], dataset['height'],
            normalize_exposure=args.normalize_exposure,
            exposure_scale_limit=args.exposure_scale_limit)
    scales = (exposure_scales(images, args.exposure_scale_limit)
              if args.normalize_exposure else np.ones(len(images)))
    errors = []
    visible_total = 0
    per_view = []
    for index in holdout[::args.view_stride]:
        values, visible = score_heldout_view(
            points, colors, seen, viewmats[index], K,
            images[index], float(scales[index]))
        errors.append(values)
        visible_total += visible
        per_view.append({'view_index': index, 'visible_points': visible,
                         'scored_points': int(values.size),
                         'median_rgb_l2': (float(np.median(values))
                                           if values.size else None)})
    combined = np.concatenate(errors) if errors else np.zeros(0)
    if combined.size == 0:
        raise SystemExit('no held-out points could be scored')
    report = {
        'train_views': len(train), 'heldout_views': len(holdout),
        'heldout_views_scored': len(per_view),
        'color_source': ('pointcloud' if args.use_pointcloud_colors else 'train'),
        'normalize_exposure': args.normalize_exposure,
        'exposure_scale_limit': args.exposure_scale_limit,
        'visible_points': visible_total, 'scored_points': int(combined.size),
        'training_coverage': float(seen.mean()),
        'heldout_scored_fraction': float(combined.size / visible_total),
        'rgb_l2_mean': float(np.mean(combined)),
        'rgb_l2_median': float(np.median(combined)),
        'rgb_l2_p90': float(np.percentile(combined, 90)),
        'rgb_l2_inlier_20': float(np.mean(combined <= 20.0)),
        'per_view': per_view,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items()
                      if key != 'per_view'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
