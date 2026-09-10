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

"""Measure LiDAR depth-edge alignment against camera image edges."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys

import numpy as np

from lidarslam_benchmark_tools.gaussian_splatting import pointcloud_io as pcio
from lidarslam_benchmark_tools.gaussian_splatting import posed_images as pi
from lidarslam_benchmark_tools.gaussian_splatting import train_gsplat as tg


def image_edges(image: np.ndarray, percentile: float = 95.0) -> np.ndarray:
    """Return a strong grayscale-gradient edge mask without OpenCV/SciPy."""
    array = np.asarray(image, dtype=np.float32)
    if array.ndim == 3:
        array = array[:, :, :3] @ np.array([0.299, 0.587, 0.114])
    if array.ndim != 2:
        raise ValueError(f'image must be HxW or HxWxC, got {array.shape}')
    if not 0.0 <= percentile <= 100.0:
        raise ValueError('percentile must be between 0 and 100')
    gx = np.zeros_like(array)
    gy = np.zeros_like(array)
    gx[:, 1:-1] = np.abs(array[:, 2:] - array[:, :-2])
    gy[1:-1, :] = np.abs(array[2:, :] - array[:-2, :])
    magnitude = np.hypot(gx, gy)
    nonzero = magnitude[magnitude > 0.0]
    if nonzero.size == 0:
        return np.zeros(array.shape, dtype=bool)
    return magnitude >= np.percentile(nonzero, percentile)


def depth_edges(depth: np.ndarray, absolute: float = 0.25,
                relative: float = 0.02) -> np.ndarray:
    """Return pixels adjacent to a supported LiDAR depth discontinuity."""
    values = np.asarray(depth, dtype=np.float32)
    if values.ndim != 2:
        raise ValueError(f'depth must be HxW, got {values.shape}')
    if absolute < 0.0 or relative < 0.0:
        raise ValueError('depth thresholds must be non-negative')
    valid = np.isfinite(values) & (values > 0.0)
    edges = np.zeros(values.shape, dtype=bool)
    with np.errstate(invalid='ignore'):
        horizontal = (valid[:, :-1] & valid[:, 1:] &
                      (np.abs(values[:, :-1] - values[:, 1:]) >
                       absolute + relative * np.minimum(
                           values[:, :-1], values[:, 1:])))
        vertical = (valid[:-1, :] & valid[1:, :] &
                    (np.abs(values[:-1, :] - values[1:, :]) >
                     absolute + relative * np.minimum(
                         values[:-1, :], values[1:, :])))
    edges[:, :-1] |= horizontal
    edges[:, 1:] |= horizontal
    edges[:-1, :] |= vertical
    edges[1:, :] |= vertical
    return edges


def nearest_edge_distances(query: np.ndarray, target: np.ndarray,
                           max_distance: int = 12) -> np.ndarray:
    """Find target-edge distance for query pixels with a bounded local search."""
    query_y, query_x = np.nonzero(query)
    if query_y.size == 0:
        return np.zeros(0, dtype=np.float32)
    distances = np.full(query_y.size, float(max_distance + 1), dtype=np.float32)
    height, width = target.shape
    offsets = [(dy, dx) for dy in range(-max_distance, max_distance + 1)
               for dx in range(-max_distance, max_distance + 1)
               if dy * dy + dx * dx <= max_distance * max_distance]
    offsets.sort(key=lambda item: item[0] * item[0] + item[1] * item[1])
    for dy, dx in offsets:
        distance = float(np.hypot(dy, dx))
        pending = distances > distance
        y = query_y + dy
        x = query_x + dx
        inside = pending & (y >= 0) & (y < height) & (x >= 0) & (x < width)
        hit = np.zeros(query_y.size, dtype=bool)
        hit[inside] = target[y[inside], x[inside]]
        distances[hit] = distance
    return distances


def projected_depth(points: np.ndarray, viewmat: np.ndarray, K: np.ndarray,
                    width: int, height: int) -> np.ndarray:
    """Project one view into an image containing sparse nearest depths."""
    (pixels, depths), = pcio.project_depth_maps(
        points, np.asarray(viewmat)[None], K, width, height)
    image = np.full((height, width), np.inf, dtype=np.float32)
    image.reshape(-1)[pixels] = depths
    return image


def score_view(points: np.ndarray, viewmat: np.ndarray, K: np.ndarray,
               image: np.ndarray, *, edge_percentile: float = 95.0,
               max_distance: int = 12) -> dict:
    """Score one camera view and return pixel-distance alignment metrics."""
    height, width = image.shape[:2]
    lidar_edges = depth_edges(projected_depth(points, viewmat, K, width, height))
    distances = nearest_edge_distances(
        lidar_edges, image_edges(image, edge_percentile), max_distance)
    if distances.size == 0:
        return {'edge_points': 0, 'median_px': None, 'p90_px': None,
                'inlier_2px': None, 'out_of_range_fraction': None}
    return {'edge_points': int(distances.size),
            'median_px': float(np.median(distances)),
            'p90_px': float(np.percentile(distances, 90)),
            'inlier_2px': float(np.mean(distances <= 2.0)),
            'out_of_range_fraction': float(np.mean(distances > max_distance))}


def correction_matrix(parameters: np.ndarray) -> np.ndarray:
    """Build camera-frame SE(3) correction from xyz metres and xyz radians."""
    tx, ty, tz, rx, ry, rz = np.asarray(parameters, dtype=np.float64).reshape(6)
    sx, cx = np.sin(rx), np.cos(rx)
    sy, cy = np.sin(ry), np.cos(ry)
    sz, cz = np.sin(rz), np.cos(rz)
    rotation_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rotation_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rotation_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rotation_z @ rotation_y @ rotation_x
    result[:3, 3] = [tx, ty, tz]
    return result


def alignment_objective(points: np.ndarray, viewmats: np.ndarray,
                        K: np.ndarray, images: list[np.ndarray],
                        parameters: np.ndarray, *, edge_percentile: float = 95.0,
                        max_distance: int = 12,
                        reference_edge_points: int | None = None) -> tuple[float, dict]:
    """Return coverage-guarded mean edge distance for one SE(3) correction."""
    delta = correction_matrix(parameters)
    chunks = []
    for viewmat, image in zip(viewmats, images):
        height, width = image.shape[:2]
        lidar_edges = depth_edges(projected_depth(
            points, delta @ viewmat, K, width, height))
        chunks.append(nearest_edge_distances(
            lidar_edges, image_edges(image, edge_percentile), max_distance))
    values = np.concatenate([item for item in chunks if item.size]) \
        if any(item.size for item in chunks) else np.zeros(0, np.float32)
    if not values.size:
        return float('inf'), {'edge_points': 0, 'mean_px': None}
    edge_points = int(values.size)
    reference = edge_points if reference_edge_points is None else reference_edge_points
    coverage = edge_points / max(reference, 1)
    penalty = max(0.0, 0.9 - coverage) * max_distance * 4.0
    loss = float(np.mean(values) + penalty)
    return loss, {'edge_points': edge_points, 'mean_px': float(np.mean(values)),
                  'median_px': float(np.median(values)),
                  'out_of_range_fraction': float(np.mean(values > max_distance)),
                  'coverage': coverage}


def optimize_correction(points: np.ndarray, viewmats: np.ndarray, K: np.ndarray,
                        images: list[np.ndarray], *, rounds: int = 3,
                        translation_step: float = 0.02,
                        rotation_step_deg: float = 0.2,
                        edge_percentile: float = 95.0,
                        max_distance: int = 12) -> tuple[np.ndarray, dict, dict]:
    """Coordinate-search a camera-frame correction, coarse to fine."""
    parameters = np.zeros(6, dtype=np.float64)
    base_loss, before = alignment_objective(
        points, viewmats, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance)
    best_loss = base_loss
    reference = before['edge_points']
    steps = np.array([translation_step] * 3 +
                     [np.deg2rad(rotation_step_deg)] * 3)
    for _ in range(rounds):
        for axis in range(6):
            for direction in (-1.0, 1.0):
                candidate = parameters.copy()
                candidate[axis] += direction * steps[axis]
                loss, _ = alignment_objective(
                    points, viewmats, K, images, candidate,
                    edge_percentile=edge_percentile,
                    max_distance=max_distance,
                    reference_edge_points=reference)
                if loss < best_loss:
                    parameters, best_loss = candidate, loss
        steps *= 0.5
    _, after = alignment_objective(
        points, viewmats, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance,
        reference_edge_points=reference)
    before['loss'] = base_loss
    after['loss'] = best_loss
    return parameters, before, after


def write_corrected_transforms(source: Path, output: Path,
                               correction: np.ndarray) -> Path:
    """Write corrected camera poses without modifying the source dataset."""
    source = Path(source).resolve()
    output = Path(output).resolve()
    if source == output:
        raise ValueError('corrected transforms output must differ from source')
    document = json.loads(source.read_text())
    dataset = tg.load_transforms(source)
    if len(document['frames']) != len(dataset['viewmats']):
        raise ValueError('frame and viewmat counts differ')
    for frame, viewmat, image_path in zip(
            document['frames'], dataset['viewmats'], dataset['image_paths']):
        corrected_c2w_cv = np.linalg.inv(correction @ viewmat)
        frame['transform_matrix'] = (
            corrected_c2w_cv @ pi.ROS_OPTICAL_TO_OPENGL).tolist()
        frame['file_path'] = os.path.relpath(image_path, output.parent)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2) + '\n')
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pointcloud', type=Path, required=True)
    parser.add_argument('--transforms', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--view-stride', type=int, default=10)
    parser.add_argument('--edge-percentile', type=float, default=95.0,
                        help='retain this percentile of nonzero image gradients; '
                             '95 focuses the metric on structural edges')
    parser.add_argument('--max-distance', type=int, default=12)
    parser.add_argument('--optimize-extrinsic', action='store_true')
    parser.add_argument('--optimization-rounds', type=int, default=3)
    parser.add_argument('--translation-step', type=float, default=0.02)
    parser.add_argument('--rotation-step-deg', type=float, default=0.2)
    parser.add_argument('--corrected-transforms-out', type=Path)
    args = parser.parse_args()
    if (args.view_stride < 1 or args.max_distance < 1 or
            args.optimization_rounds < 1 or args.translation_step <= 0.0 or
            args.rotation_step_deg <= 0.0):
        raise SystemExit('stride, distance, rounds, and search steps must be > 0')
    import imageio.v3 as iio
    points, _ = pcio.read_ply_xyz(args.pointcloud)
    dataset = tg.load_transforms(args.transforms)
    viewmats = np.asarray(dataset['viewmats'], dtype=np.float64)
    selected = list(range(0, len(viewmats), args.view_stride))
    images = [np.asarray(iio.imread(dataset['image_paths'][index]))
              for index in selected]
    optimization = None
    delta = np.eye(4)
    if args.optimize_extrinsic:
        parameters, before, after = optimize_correction(
            points, viewmats[selected], dataset['K'], images,
            rounds=args.optimization_rounds,
            translation_step=args.translation_step,
            rotation_step_deg=args.rotation_step_deg,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance)
        delta = correction_matrix(parameters)
        optimization = {
            'parameters_xyz_m_rpy_deg': parameters[:3].tolist() +
            np.rad2deg(parameters[3:]).tolist(),
            'camera_correction_matrix': delta.tolist(),
            'before': before, 'after': after,
        }
    per_view = []
    for index, image in zip(selected, images):
        score = score_view(
            points, delta @ viewmats[index], dataset['K'], image,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance)
        score['view_index'] = index
        per_view.append(score)
    valid = [item for item in per_view if item['edge_points'] > 0]
    if not valid:
        raise SystemExit('no LiDAR depth edges were measurable')
    weights = np.asarray([item['edge_points'] for item in valid], dtype=float)
    report = {'pointcloud': str(args.pointcloud.resolve()),
              'transforms': str(args.transforms.resolve()),
              'views_scored': len(valid), 'edge_points': int(weights.sum())}
    for name in ('median_px', 'p90_px', 'inlier_2px',
                 'out_of_range_fraction'):
        report[f'weighted_{name}'] = float(np.average(
            [item[name] for item in valid], weights=weights))
    if optimization is not None:
        report['extrinsic_optimization'] = optimization
        if args.corrected_transforms_out is not None:
            corrected = write_corrected_transforms(
                args.transforms, args.corrected_transforms_out, delta)
            report['corrected_transforms'] = str(corrected)
    elif args.corrected_transforms_out is not None:
        raise SystemExit('--corrected-transforms-out requires --optimize-extrinsic')
    report['per_view'] = per_view
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items()
                      if key != 'per_view'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
