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
from lidarslam_benchmark_tools.gaussian_splatting import spatiotemporal_calibration as stc
from lidarslam_benchmark_tools.gaussian_splatting import train_gsplat as tg


def image_edge_field(image: np.ndarray, percentile: float = 95.0
                     ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return strong edges and unit image-gradient normal components."""
    array = np.asarray(image, dtype=np.float32)
    if array.ndim == 3:
        array = array[:, :, :3] @ np.array([0.299, 0.587, 0.114])
    if array.ndim != 2:
        raise ValueError(f'image must be HxW or HxWxC, got {array.shape}')
    if not 0.0 <= percentile <= 100.0:
        raise ValueError('percentile must be between 0 and 100')
    gx = np.zeros_like(array)
    gy = np.zeros_like(array)
    gx[:, 1:-1] = array[:, 2:] - array[:, :-2]
    gy[1:-1, :] = array[2:, :] - array[:-2, :]
    magnitude = np.hypot(gx, gy)
    nonzero = magnitude[magnitude > 0.0]
    if nonzero.size == 0:
        empty = np.zeros(array.shape, dtype=np.float32)
        return np.zeros(array.shape, dtype=bool), empty, empty
    edges = magnitude >= np.percentile(nonzero, percentile)
    denominator = np.maximum(magnitude, 1e-12)
    return (edges, (gx / denominator).astype(np.float32),
            (gy / denominator).astype(np.float32))


def image_edges(image: np.ndarray, percentile: float = 95.0) -> np.ndarray:
    """Return a strong grayscale-gradient edge mask without OpenCV/SciPy."""
    return image_edge_field(image, percentile)[0]


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


def depth_edge_normal_field(depth: np.ndarray, absolute: float = 0.25,
                            relative: float = 0.02
                            ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return depth edges and unsigned discontinuity-normal components."""
    values = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(values) & (values > 0.0)
    nx = np.zeros(values.shape, dtype=np.float32)
    ny = np.zeros(values.shape, dtype=np.float32)
    with np.errstate(invalid='ignore'):
        horizontal = (valid[:, :-1] & valid[:, 1:] &
                      (np.abs(values[:, :-1] - values[:, 1:]) >
                       absolute + relative * np.minimum(
                           values[:, :-1], values[:, 1:])))
        vertical = (valid[:-1, :] & valid[1:, :] &
                    (np.abs(values[:-1, :] - values[1:, :]) >
                     absolute + relative * np.minimum(
                         values[:-1, :], values[1:, :])))
    with np.errstate(invalid='ignore'):
        horizontal_sign = np.where(
            horizontal, np.sign(values[:, 1:] - values[:, :-1]), 0.0)
        vertical_sign = np.where(
            vertical, np.sign(values[1:, :] - values[:-1, :]), 0.0)
    nx[:, :-1] += horizontal_sign
    nx[:, 1:] += horizontal_sign
    ny[:-1, :] += vertical_sign
    ny[1:, :] += vertical_sign
    magnitude = np.hypot(nx, ny)
    supported = magnitude > 0.0
    nx[supported] /= magnitude[supported]
    ny[supported] /= magnitude[supported]
    return supported, nx, ny


def surface_support_mask(depth: np.ndarray, *, radius: int = 2,
                         min_neighbors: int = 4, absolute: float = 0.10,
                         relative: float = 0.01) -> np.ndarray:
    """Return depth pixels supported by nearby samples on the same surface."""
    values = np.asarray(depth, dtype=np.float32)
    if values.ndim != 2:
        raise ValueError(f'depth must be HxW, got {values.shape}')
    if radius < 1 or min_neighbors < 1:
        raise ValueError('surface support radius and neighbours must be positive')
    if min_neighbors > (2 * radius + 1) ** 2 - 1:
        raise ValueError('surface support neighbours exceed the search window')
    if absolute < 0.0 or relative < 0.0:
        raise ValueError('surface support tolerances must be non-negative')
    valid = np.isfinite(values) & (values > 0.0)
    counts = np.zeros(values.shape, dtype=np.uint16)
    height, width = values.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dy == 0 and dx == 0:
                continue
            target_y = slice(max(0, -dy), min(height, height - dy))
            target_x = slice(max(0, -dx), min(width, width - dx))
            neighbour_y = slice(max(0, dy), min(height, height + dy))
            neighbour_x = slice(max(0, dx), min(width, width + dx))
            centre = values[target_y, target_x]
            neighbour = values[neighbour_y, neighbour_x]
            pair_valid = (valid[target_y, target_x] &
                          valid[neighbour_y, neighbour_x])
            with np.errstate(invalid='ignore'):
                close = pair_valid & (
                    np.abs(centre - neighbour) <=
                    absolute + relative * np.minimum(centre, neighbour))
            counts[target_y, target_x] += close
    return valid & (counts >= min_neighbors)


def supported_depth_edges(depth: np.ndarray,
                          support: dict | None = None) -> tuple[np.ndarray, int]:
    """Return raw or same-surface-supported edges plus the raw edge count."""
    raw = depth_edges(depth)
    raw_count = int(np.sum(raw))
    if support is None or support.get('radius', 0) == 0:
        return raw, raw_count
    mask = surface_support_mask(
        depth, radius=support['radius'],
        min_neighbors=support['min_neighbors'],
        absolute=support['absolute'], relative=support['relative'])
    return raw & mask, raw_count


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


def nearest_edge_correspondences(query: np.ndarray, target: np.ndarray,
                                 max_distance: int = 12) -> dict:
    """Return bounded nearest-edge distances and signed target offsets.

    ``dx_px`` and ``dy_px`` point from each LiDAR depth-edge pixel to the
    matched image-edge pixel. Unmatched pixels retain the historical
    ``max_distance + 1`` distance and have null-direction sentinels encoded as
    NaN. The deterministic offset ordering makes diagnostics reproducible.
    """
    query_y, query_x = np.nonzero(query)
    if query_y.size == 0:
        empty = np.zeros(0, dtype=np.float32)
        return {'query_y': query_y, 'query_x': query_x,
                'distance_px': empty, 'dy_px': empty, 'dx_px': empty}
    distances = np.full(query_y.size, float(max_distance + 1), dtype=np.float32)
    match_dy = np.full(query_y.size, np.nan, dtype=np.float32)
    match_dx = np.full(query_y.size, np.nan, dtype=np.float32)
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
        match_dy[hit] = dy
        match_dx[hit] = dx
    return {'query_y': query_y, 'query_x': query_x,
            'distance_px': distances, 'dy_px': match_dy, 'dx_px': match_dx}


def nearest_oriented_edge_correspondences(
        query: np.ndarray, target: np.ndarray,
        query_nx: np.ndarray, query_ny: np.ndarray,
        target_nx: np.ndarray, target_ny: np.ndarray, *,
        max_distance: int = 12, max_angle_deg: float = 30.0) -> dict:
    """Match nearest edges whose unoriented normal axes agree."""
    query_y, query_x = np.nonzero(query)
    if query_y.size == 0:
        empty = np.zeros(0, dtype=np.float32)
        return {'query_y': query_y, 'query_x': query_x,
                'distance_px': empty, 'dy_px': empty, 'dx_px': empty}
    distances = np.full(query_y.size, float(max_distance + 1), dtype=np.float32)
    match_dy = np.full(query_y.size, np.nan, dtype=np.float32)
    match_dx = np.full(query_y.size, np.nan, dtype=np.float32)
    qnx, qny = query_nx[query_y, query_x], query_ny[query_y, query_x]
    valid_query_normal = np.hypot(qnx, qny) > 0.5
    cosine_limit = float(np.cos(np.deg2rad(max_angle_deg)))
    height, width = target.shape
    offsets = [(dy, dx) for dy in range(-max_distance, max_distance + 1)
               for dx in range(-max_distance, max_distance + 1)
               if dy * dy + dx * dx <= max_distance * max_distance]
    offsets.sort(key=lambda item: item[0] * item[0] + item[1] * item[1])
    for dy, dx in offsets:
        distance = float(np.hypot(dy, dx))
        pending = (distances > distance) & valid_query_normal
        y, x = query_y + dy, query_x + dx
        inside = pending & (y >= 0) & (y < height) & (x >= 0) & (x < width)
        hit = np.zeros(query_y.size, dtype=bool)
        if np.any(inside):
            indices = np.flatnonzero(inside)
            edge_hit = target[y[indices], x[indices]]
            dot = np.abs(
                qnx[indices] * target_nx[y[indices], x[indices]] +
                qny[indices] * target_ny[y[indices], x[indices]])
            hit[indices] = edge_hit & (dot >= cosine_limit)
        distances[hit] = distance
        match_dy[hit] = dy
        match_dx[hit] = dx
    return {'query_y': query_y, 'query_x': query_x,
            'distance_px': distances, 'dy_px': match_dy, 'dx_px': match_dx}


def projected_depth(points: np.ndarray, viewmat: np.ndarray, K: np.ndarray,
                    width: int, height: int) -> np.ndarray:
    """Project one view into an image containing sparse nearest depths."""
    (pixels, depths), = pcio.project_depth_maps(
        points, np.asarray(viewmat)[None], K, width, height)
    image = np.full((height, width), np.inf, dtype=np.float32)
    image.reshape(-1)[pixels] = depths
    return image


def projected_depth_and_ids(points: np.ndarray, viewmat: np.ndarray,
                            K: np.ndarray, width: int,
                            height: int) -> tuple[np.ndarray, np.ndarray]:
    """Project nearest depth and deterministic source point ID per pixel."""
    points = np.asarray(points, dtype=np.float64)
    camera = points @ viewmat[:3, :3].T + viewmat[:3, 3]
    depth = camera[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = np.nan_to_num(K[0, 0] * camera[:, 0] / depth + K[0, 2], nan=-1.0,
                          posinf=-1.0, neginf=-1.0)
        v = np.nan_to_num(K[1, 1] * camera[:, 1] / depth + K[1, 2], nan=-1.0,
                          posinf=-1.0, neginf=-1.0)
    ui = np.round(u).astype(np.int64)
    vi = np.round(v).astype(np.int64)
    inside = ((depth > 1e-6) & (ui >= 0) & (ui < width) &
              (vi >= 0) & (vi < height))
    depth_image = np.full(height * width, np.inf, dtype=np.float64)
    missing_id = np.iinfo(np.int64).max
    id_image = np.full(height * width, missing_id, dtype=np.int64)
    if not np.any(inside):
        id_image.fill(-1)
        return (depth_image.reshape(height, width).astype(np.float32),
                id_image.reshape(height, width))
    ids = np.flatnonzero(inside)
    pixels = vi[inside] * width + ui[inside]
    np.minimum.at(depth_image, pixels, depth[inside])
    winners = depth[inside] == depth_image[pixels]
    np.minimum.at(id_image, pixels[winners], ids[winners])
    id_image[id_image == missing_id] = -1
    return (depth_image.reshape(height, width).astype(np.float32),
            id_image.reshape(height, width))


def projected_point_mask(points: np.ndarray, viewmat: np.ndarray, K: np.ndarray,
                         width: int, height: int) -> np.ndarray:
    """Project fixed 3D contour points into a deterministic binary mask."""
    return projected_contour_field(points, viewmat, K, width, height)[0]


def projected_contour_field(contours: np.ndarray, viewmat: np.ndarray,
                            K: np.ndarray, width: int, height: int
                            ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project fixed xyz+normal contours into a mask and normal field."""
    contours = np.asarray(contours, dtype=np.float64)
    mask = np.zeros((height, width), dtype=bool)
    normal_x = np.zeros((height, width), dtype=np.float32)
    normal_y = np.zeros((height, width), dtype=np.float32)
    if contours.size == 0:
        return mask, normal_x, normal_y
    points = contours[:, :3]
    camera = points @ viewmat[:3, :3].T + viewmat[:3, 3]
    depth = camera[:, 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = np.nan_to_num(K[0, 0] * camera[:, 0] / depth + K[0, 2], nan=-1.0,
                          posinf=-1.0, neginf=-1.0)
        v = np.nan_to_num(K[1, 1] * camera[:, 1] / depth + K[1, 2], nan=-1.0,
                          posinf=-1.0, neginf=-1.0)
    ui = np.round(u).astype(np.int64)
    vi = np.round(v).astype(np.int64)
    inside = ((depth > 1e-6) & (ui >= 0) & (ui < width) &
              (vi >= 0) & (vi < height))
    mask[vi[inside], ui[inside]] = True
    if contours.shape[1] >= 5:
        pixels = vi[inside] * width + ui[inside]
        flat_x, flat_y = normal_x.reshape(-1), normal_y.reshape(-1)
        np.add.at(flat_x, pixels, contours[inside, 3].astype(np.float32))
        np.add.at(flat_y, pixels, contours[inside, 4].astype(np.float32))
        magnitude = np.hypot(normal_x, normal_y)
        valid = magnitude > 1e-6
        normal_x[valid] /= magnitude[valid]
        normal_y[valid] /= magnitude[valid]
    return mask, normal_x, normal_y


def extract_fixed_contours(points: np.ndarray, viewmats: np.ndarray,
                           K: np.ndarray, images: list[np.ndarray], *,
                           edge_percentile: float = 95.0,
                           association_distance: int = 0,
                           max_points_per_view: int = 50000,
                           depth_support: dict | None = None
                           ) -> tuple[list[np.ndarray], dict]:
    """Freeze full-density visible depth edges as per-view 3D point banks."""
    banks = []
    per_view = []
    for ordinal, (viewmat, image) in enumerate(zip(viewmats, images)):
        height, width = image.shape[:2]
        depth, point_ids = projected_depth_and_ids(
            points, viewmat, K, width, height)
        lidar_edges, raw_edges = supported_depth_edges(depth, depth_support)
        _, depth_nx, depth_ny = depth_edge_normal_field(depth)
        edge_ids = point_ids[lidar_edges]
        edge_normals = np.stack(
            [depth_nx[lidar_edges], depth_ny[lidar_edges]], axis=1)
        valid_ids = edge_ids >= 0
        edge_ids = edge_ids[valid_ids]
        edge_normals = edge_normals[valid_ids]
        query = np.zeros_like(lidar_edges)
        query[lidar_edges] = valid_ids
        if association_distance > 0:
            distances = nearest_edge_distances(
                query, image_edges(image, edge_percentile),
                association_distance)
            associated = distances <= association_distance
        else:
            associated = np.ones(len(edge_ids), dtype=bool)
        selected_ids = edge_ids[associated]
        selected_normals = edge_normals[associated]
        # Preserve row-major image order before deterministic thinning so the
        # cap remains spatially distributed instead of following PLY point ID.
        _, first_occurrence = np.unique(selected_ids, return_index=True)
        first_occurrence = np.sort(first_occurrence)
        selected_ids = selected_ids[first_occurrence]
        selected_normals = selected_normals[first_occurrence]
        before_cap = len(selected_ids)
        if len(selected_ids) > max_points_per_view:
            indices = np.linspace(
                0, len(selected_ids) - 1, max_points_per_view,
                dtype=np.int64)
            selected_ids = selected_ids[indices]
            selected_normals = selected_normals[indices]
        banks.append(np.column_stack([
            np.asarray(points[selected_ids], dtype=np.float64),
            selected_normals.astype(np.float64)]))
        per_view.append({
            'ordinal': ordinal,
            'raw_depth_edge_pixels': raw_edges,
            'supported_depth_edge_pixels': int(np.sum(lidar_edges)),
            'candidate_edge_points_before_cap': before_cap,
            'image_associated_edge_points_before_cap': (
                before_cap if association_distance > 0 else None),
            'fixed_contour_points': len(selected_ids),
            'association_fraction': float(
                np.mean(associated) if associated.size else 0.0),
        })
    return banks, {
        'association_distance_px': association_distance,
        'image_association_enabled': association_distance > 0,
        'max_points_per_view': max_points_per_view,
        'depth_support': depth_support,
        'views': per_view,
        'total_fixed_contour_points': int(sum(len(bank) for bank in banks)),
    }


def score_view(points: np.ndarray, viewmat: np.ndarray, K: np.ndarray,
               image: np.ndarray, *, edge_percentile: float = 95.0,
               max_distance: int = 12,
               depth_support: dict | None = None,
               contour_points: np.ndarray | None = None,
               orientation_max_angle_deg: float = 0.0) -> dict:
    """Score one camera view and return pixel-distance alignment metrics."""
    height, width = image.shape[:2]
    if contour_points is None:
        depth = projected_depth(points, viewmat, K, width, height)
        lidar_edges, raw_edge_points = supported_depth_edges(
            depth, depth_support)
        _, query_nx, query_ny = depth_edge_normal_field(depth)
    else:
        lidar_edges, query_nx, query_ny = projected_contour_field(
            contour_points, viewmat, K, width, height)
        raw_edge_points = len(contour_points)
    image_edge_mask, target_nx, target_ny = image_edge_field(
        image, edge_percentile)
    if orientation_max_angle_deg > 0.0:
        correspondences = nearest_oriented_edge_correspondences(
            lidar_edges, image_edge_mask, query_nx, query_ny,
            target_nx, target_ny, max_distance=max_distance,
            max_angle_deg=orientation_max_angle_deg)
    else:
        correspondences = nearest_edge_correspondences(
            lidar_edges, image_edge_mask, max_distance)
    distances = correspondences['distance_px']
    if distances.size == 0:
        return {'edge_points': 0, 'raw_edge_points': raw_edge_points,
                'supported_edge_fraction': 0.0, 'median_px': None,
                'p90_px': None,
                'inlier_2px': None, 'out_of_range_fraction': None,
                'matched_edge_points': 0, 'median_dx_px': None,
                'median_dy_px': None, 'mean_dx_px': None,
                'mean_dy_px': None, 'direction_coherence': None}
    matched = np.isfinite(correspondences['dx_px'])
    dx = correspondences['dx_px'][matched]
    dy = correspondences['dy_px'][matched]
    mean_dx = float(np.mean(dx)) if np.any(matched) else None
    mean_dy = float(np.mean(dy)) if np.any(matched) else None
    mean_radius = float(np.mean(np.hypot(dx, dy))) if np.any(matched) else None
    return {'edge_points': int(distances.size),
            'raw_edge_points': raw_edge_points,
            'supported_edge_fraction': float(
                distances.size / max(raw_edge_points, 1)),
            'median_px': float(np.median(distances)),
            'p90_px': float(np.percentile(distances, 90)),
            'inlier_2px': float(np.mean(distances <= 2.0)),
            'out_of_range_fraction': float(np.mean(distances > max_distance)),
            'matched_edge_points': int(np.sum(matched)),
            'median_dx_px': (float(np.median(correspondences['dx_px'][matched]))
                             if np.any(matched) else None),
            'median_dy_px': (float(np.median(correspondences['dy_px'][matched]))
                             if np.any(matched) else None),
            'mean_dx_px': mean_dx, 'mean_dy_px': mean_dy,
            'direction_coherence': (float(np.hypot(mean_dx, mean_dy) /
                                          max(mean_radius, 1e-12))
                                    if mean_radius is not None else None),
            'orientation_max_angle_deg': orientation_max_angle_deg}


def render_residual_overlay(points: np.ndarray, viewmat: np.ndarray,
                            K: np.ndarray, image: np.ndarray, *,
                            edge_percentile: float = 95.0,
                            max_distance: int = 12,
                            depth_support: dict | None = None,
                            contour_points: np.ndarray | None = None,
                            orientation_max_angle_deg: float = 0.0
                            ) -> np.ndarray:
    """Overlay image edges and colour-coded LiDAR edge residuals."""
    source = np.asarray(image)
    if source.ndim == 2:
        source = np.repeat(source[:, :, None], 3, axis=2)
    source = source[:, :, :3]
    if source.dtype != np.uint8:
        source = np.clip(source, 0, 255).astype(np.uint8)
    output = np.clip(source.astype(np.float32) * 0.55, 0, 255).astype(np.uint8)
    height, width = source.shape[:2]
    camera_edges, target_nx, target_ny = image_edge_field(
        source, edge_percentile)
    if contour_points is None:
        depth = projected_depth(points, viewmat, K, width, height)
        lidar_edges, _ = supported_depth_edges(depth, depth_support)
        _, query_nx, query_ny = depth_edge_normal_field(depth)
    else:
        lidar_edges, query_nx, query_ny = projected_contour_field(
            contour_points, viewmat, K, width, height)
    if orientation_max_angle_deg > 0.0:
        residuals = nearest_oriented_edge_correspondences(
            lidar_edges, camera_edges, query_nx, query_ny,
            target_nx, target_ny, max_distance=max_distance,
            max_angle_deg=orientation_max_angle_deg)
    else:
        residuals = nearest_edge_correspondences(
            lidar_edges, camera_edges, max_distance)

    # Camera edges are green. LiDAR edges run cyan -> yellow -> red as the
    # residual grows; saturated/unmatched edges are magenta.
    output[camera_edges] = [40, 220, 40]
    if residuals['distance_px'].size:
        ratio = np.minimum(residuals['distance_px'], max_distance) / max_distance
        colours = np.stack([
            255.0 * ratio,
            255.0 * (1.0 - np.abs(2.0 * ratio - 1.0)),
            255.0 * (1.0 - ratio),
        ], axis=1).astype(np.uint8)
        unmatched = residuals['distance_px'] > max_distance
        colours[unmatched] = [255, 0, 255]
        y, x = residuals['query_y'], residuals['query_x']
        for dy, dx in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
            yy, xx = y + dy, x + dx
            inside = (yy >= 0) & (yy < height) & (xx >= 0) & (xx < width)
            output[yy[inside], xx[inside]] = colours[inside]
    return output


def write_residual_diagnostics(directory: Path, points: np.ndarray,
                               viewmats: np.ndarray, K: np.ndarray,
                               selected: list[int], images: list[np.ndarray],
                               per_view: list[dict], *, worst_views: int,
                               edge_percentile: float,
                               max_distance: int,
                               depth_support: dict | None = None,
                               contour_points_by_view: list[np.ndarray] | None = None,
                               orientation_max_angle_deg: float = 0.0
                               ) -> dict:
    """Write deterministic worst-view overlays and a compact contact sheet."""
    import imageio as iio

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    image_by_index = dict(zip(selected, images))
    contour_by_index = (dict(zip(selected, contour_points_by_view))
                        if contour_points_by_view is not None else {})
    valid = [item for item in per_view if item['edge_points'] > 0]
    ranked = sorted(
        valid,
        key=lambda item: (item['out_of_range_fraction'], item['p90_px'],
                          item['median_px'], -item['view_index']),
        reverse=True)[:worst_views]
    tiles = []
    entries = []
    for rank, metrics in enumerate(ranked, 1):
        index = metrics['view_index']
        overlay = render_residual_overlay(
            points, viewmats[index], K, image_by_index[index],
            edge_percentile=edge_percentile, max_distance=max_distance,
            depth_support=depth_support,
            contour_points=contour_by_index.get(index),
            orientation_max_angle_deg=orientation_max_angle_deg)
        filename = f'worst_{rank:02d}_view_{index:05d}.png'
        path = directory / filename
        iio.imwrite(path, overlay)
        tile_width = min(400, overlay.shape[1])
        step = max(1, int(np.ceil(overlay.shape[1] / tile_width)))
        tiles.append(overlay[::step, ::step])
        entries.append({'rank': rank, 'view_index': index,
                        'overlay': filename, **metrics})

    contact_name = None
    if tiles:
        columns = min(2, len(tiles))
        tile_height = max(tile.shape[0] for tile in tiles)
        tile_width = max(tile.shape[1] for tile in tiles)
        rows = (len(tiles) + columns - 1) // columns
        contact = np.zeros((rows * tile_height, columns * tile_width, 3),
                           dtype=np.uint8)
        for ordinal, tile in enumerate(tiles):
            row, column = divmod(ordinal, columns)
            contact[row * tile_height:row * tile_height + tile.shape[0],
                    column * tile_width:column * tile_width + tile.shape[1]] = tile
        contact_name = 'worst_views_contact_sheet.png'
        iio.imwrite(directory / contact_name, contact)

    matched = [item for item in valid if item['mean_dx_px'] is not None]
    direction = None
    if matched:
        weights = np.asarray([item['matched_edge_points'] for item in matched],
                             dtype=float)
        direction = {
            'weighted_mean_dx_px': float(np.average(
                [item['mean_dx_px'] for item in matched], weights=weights)),
            'weighted_mean_dy_px': float(np.average(
                [item['mean_dy_px'] for item in matched], weights=weights)),
            'view_mean_dx_std_px': float(np.std(
                [item['mean_dx_px'] for item in matched])),
            'view_mean_dy_std_px': float(np.std(
                [item['mean_dy_px'] for item in matched])),
            'weighted_direction_coherence': float(np.average(
                [item['direction_coherence'] for item in matched],
                weights=weights)),
        }
    result = {'legend': {'camera_edges': 'green',
                         'aligned_lidar_edges': 'cyan',
                         'mid_residual_lidar_edges': 'yellow',
                         'max_residual_lidar_edges': 'red',
                         'unmatched_lidar_edges': 'magenta'},
              'ranking': 'out_of_range_fraction,p90_px,median_px',
              'depth_support': depth_support,
              'fixed_contours': contour_points_by_view is not None,
              'orientation_max_angle_deg': orientation_max_angle_deg,
              'direction_summary': direction,
              'contact_sheet': contact_name, 'worst_views': entries}
    (directory / 'diagnostics.json').write_text(json.dumps(result, indent=2) + '\n')
    return result


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
                        reference_edge_points: int | None = None,
                        image_edge_masks: list[np.ndarray] | None = None,
                        depth_support: dict | None = None,
                        contour_points_by_view: list[np.ndarray] | None = None,
                        orientation_max_angle_deg: float = 0.0
                        ) -> tuple[float, dict]:
    """Return coverage-guarded mean edge distance for one SE(3) correction."""
    if (contour_points_by_view is not None and
            len(contour_points_by_view) != len(images)):
        raise ValueError('fixed contour bank count must match image count')
    delta = correction_matrix(parameters)
    chunks = []
    targets = (image_edge_masks if image_edge_masks is not None else
               [(image_edge_field(image, edge_percentile)
                 if orientation_max_angle_deg > 0.0 else
                 image_edges(image, edge_percentile)) for image in images])
    raw_edge_points = 0
    contours = (contour_points_by_view if contour_points_by_view is not None
                else [None] * len(images))
    for viewmat, image, target, contour_points in zip(
            viewmats, images, targets, contours):
        height, width = image.shape[:2]
        if contour_points is None:
            depth = projected_depth(
                points, delta @ viewmat, K, width, height)
            lidar_edges, raw_count = supported_depth_edges(
                depth, depth_support)
            _, query_nx, query_ny = depth_edge_normal_field(depth)
        else:
            lidar_edges, query_nx, query_ny = projected_contour_field(
                contour_points, delta @ viewmat, K, width, height)
            raw_count = len(contour_points)
        raw_edge_points += raw_count
        if orientation_max_angle_deg > 0.0:
            target_mask, target_nx, target_ny = target
            chunks.append(nearest_oriented_edge_correspondences(
                lidar_edges, target_mask, query_nx, query_ny,
                target_nx, target_ny, max_distance=max_distance,
                max_angle_deg=orientation_max_angle_deg)['distance_px'])
        else:
            chunks.append(nearest_edge_distances(
                lidar_edges, target, max_distance))
    values = np.concatenate([item for item in chunks if item.size]) \
        if any(item.size for item in chunks) else np.zeros(0, np.float32)
    if not values.size:
        return float('inf'), {'edge_points': 0,
                              'raw_edge_points': raw_edge_points,
                              'supported_edge_fraction': 0.0,
                              'mean_px': None}
    edge_points = int(values.size)
    reference = edge_points if reference_edge_points is None else reference_edge_points
    coverage = edge_points / max(reference, 1)
    penalty = max(0.0, 0.9 - coverage) * max_distance * 4.0
    loss = float(np.mean(values) + penalty)
    return loss, {'edge_points': edge_points, 'mean_px': float(np.mean(values)),
                  'median_px': float(np.median(values)),
                  'out_of_range_fraction': float(np.mean(values > max_distance)),
                  'coverage': coverage,
                  'raw_edge_points': raw_edge_points,
                  'supported_edge_fraction': float(
                      edge_points / max(raw_edge_points, 1))}


def optimize_correction(points: np.ndarray, viewmats: np.ndarray, K: np.ndarray,
                        images: list[np.ndarray], *, rounds: int = 3,
                        translation_step: float = 0.02,
                        rotation_step_deg: float = 0.2,
                        edge_percentile: float = 95.0,
                        max_distance: int = 12,
                        depth_support: dict | None = None,
                        contour_points_by_view: list[np.ndarray] | None = None,
                        orientation_max_angle_deg: float = 0.0
                        ) -> tuple[np.ndarray, dict, dict]:
    """Coordinate-search a camera-frame correction, coarse to fine."""
    parameters = np.zeros(6, dtype=np.float64)
    base_loss, before = alignment_objective(
        points, viewmats, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance,
        depth_support=depth_support,
        contour_points_by_view=contour_points_by_view,
        orientation_max_angle_deg=orientation_max_angle_deg)
    best_loss = base_loss
    reference = before['edge_points']
    edge_masks = [(image_edge_field(image, edge_percentile)
                   if orientation_max_angle_deg > 0.0 else
                   image_edges(image, edge_percentile)) for image in images]
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
                    reference_edge_points=reference,
                    image_edge_masks=edge_masks,
                    depth_support=depth_support,
                    contour_points_by_view=contour_points_by_view,
                    orientation_max_angle_deg=orientation_max_angle_deg)
                if loss < best_loss:
                    parameters, best_loss = candidate, loss
        steps *= 0.5
    _, after = alignment_objective(
        points, viewmats, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance,
        reference_edge_points=reference, image_edge_masks=edge_masks,
        depth_support=depth_support,
        contour_points_by_view=contour_points_by_view,
        orientation_max_angle_deg=orientation_max_angle_deg)
    before['loss'] = base_loss
    after['loss'] = best_loss
    return parameters, before, after


def frame_stamps(transforms: Path) -> np.ndarray:
    """Read the camera timestamps retained by ``extract_posed_images``."""
    document = json.loads(Path(transforms).read_text())
    try:
        stamps = np.asarray([frame['stamp'] for frame in document['frames']],
                            dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(
            f'{transforms}: every frame needs a numeric stamp for temporal '
            'calibration') from exc
    if stamps.size == 0 or not np.all(np.isfinite(stamps)):
        raise ValueError(f'{transforms}: frame stamps must be finite and non-empty')
    return stamps


def infer_body_T_camera(samples: list[pi.TrajectorySample], stamps: np.ndarray,
                        viewmats: np.ndarray) -> tuple[np.ndarray, dict]:
    """Recover the static body<-camera transform from extracted camera poses."""
    estimates = []
    for stamp, viewmat in zip(stamps, viewmats):
        world_T_body = pi.interpolate_pose(samples, float(stamp))
        world_T_camera = np.linalg.inv(viewmat)
        estimates.append(np.linalg.inv(world_T_body) @ world_T_camera)
    translations = np.asarray([item[:3, 3] for item in estimates])
    centre = np.median(translations, axis=0)
    representative = estimates[int(np.argmin(
        np.linalg.norm(translations - centre, axis=1)))]
    translation_spread = np.linalg.norm(
        translations - representative[:3, 3], axis=1)
    rotation_spread = []
    for estimate in estimates:
        relative = representative[:3, :3].T @ estimate[:3, :3]
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        rotation_spread.append(np.rad2deg(np.arccos(cosine)))
    consistency = {
        'frames': len(estimates),
        'translation_spread_p95_m': float(np.percentile(translation_spread, 95)),
        'rotation_spread_p95_deg': float(np.percentile(rotation_spread, 95)),
    }
    return representative, consistency


def trajectory_excitation(samples: list[pi.TrajectorySample],
                          stamps: np.ndarray) -> dict:
    """Summarise motion that makes a camera/LiDAR time offset observable."""
    poses = [pi.interpolate_pose(samples, float(stamp)) for stamp in stamps]
    translations = np.asarray([pose[:3, 3] for pose in poses])
    translation_path = float(np.linalg.norm(np.diff(translations, axis=0), axis=1).sum())
    rotation_path = 0.0
    for first, second in zip(poses, poses[1:]):
        relative = first[:3, :3].T @ second[:3, :3]
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        rotation_path += float(np.rad2deg(np.arccos(cosine)))
    return {'translation_path_m': translation_path,
            'rotation_path_deg': rotation_path,
            'time_offset_observable': bool(
                translation_path >= 0.05 or rotation_path >= 1.0)}


def recompose_viewmats(samples: list[pi.TrajectorySample], stamps: np.ndarray,
                       body_T_camera: np.ndarray,
                       parameters: np.ndarray) -> np.ndarray:
    """Compose world-to-camera poses for a time and local SE(3) correction."""
    values = np.asarray(parameters, dtype=np.float64).reshape(7)
    time_offset = float(values[0])
    corrected_extrinsic = body_T_camera @ correction_matrix(values[1:])
    return np.asarray([
        np.linalg.inv(pi.interpolate_pose(samples, float(stamp + time_offset)) @
                      corrected_extrinsic)
        for stamp in stamps
    ])


def spatiotemporal_objective(
        points: np.ndarray, samples: list[pi.TrajectorySample],
        stamps: np.ndarray, body_T_camera: np.ndarray, K: np.ndarray,
        images: list[np.ndarray], parameters: np.ndarray, *,
        edge_percentile: float = 95.0, max_distance: int = 12,
        reference_edge_points: int | None = None,
        image_edge_masks: list[np.ndarray] | None = None,
        depth_support: dict | None = None,
        contour_points_by_view: list[np.ndarray] | None = None,
        orientation_max_angle_deg: float = 0.0
        ) -> tuple[float, dict]:
    """Evaluate a continuous-time pose recomposition against image edges."""
    try:
        viewmats = recompose_viewmats(
            samples, stamps, body_T_camera, parameters)
    except ValueError:
        return float('inf'), {'edge_points': 0, 'mean_px': None,
                              'median_px': None, 'coverage': 0.0}
    return alignment_objective(
        points, viewmats, K, images, np.zeros(6),
        edge_percentile=edge_percentile, max_distance=max_distance,
        reference_edge_points=reference_edge_points,
        image_edge_masks=image_edge_masks, depth_support=depth_support,
        contour_points_by_view=contour_points_by_view,
        orientation_max_angle_deg=orientation_max_angle_deg)


def optimize_spatiotemporal(
        points: np.ndarray, samples: list[pi.TrajectorySample],
        stamps: np.ndarray, body_T_camera: np.ndarray, K: np.ndarray,
        images: list[np.ndarray], *, rounds: int = 3,
        time_step: float = 0.02, translation_step: float = 0.02,
        rotation_step_deg: float = 0.2, max_time_offset: float = 0.1,
        max_translation: float = 0.1, max_rotation_deg: float = 2.0,
        edge_percentile: float = 95.0,
        max_distance: int = 12,
        depth_support: dict | None = None,
        contour_points_by_view: list[np.ndarray] | None = None,
        orientation_max_angle_deg: float = 0.0
        ) -> tuple[np.ndarray, dict, dict]:
    """Bounded deterministic coordinate search over dt plus a local SE(3)."""
    parameters = np.zeros(7, dtype=np.float64)
    base_loss, before = spatiotemporal_objective(
        points, samples, stamps, body_T_camera, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance,
        depth_support=depth_support,
        contour_points_by_view=contour_points_by_view,
        orientation_max_angle_deg=orientation_max_angle_deg)
    reference = before['edge_points']
    edge_masks = [(image_edge_field(image, edge_percentile)
                   if orientation_max_angle_deg > 0.0 else
                   image_edges(image, edge_percentile)) for image in images]
    best_loss = base_loss
    steps = np.array([time_step] + [translation_step] * 3 +
                     [np.deg2rad(rotation_step_deg)] * 3)
    bounds = np.array([max_time_offset] + [max_translation] * 3 +
                      [np.deg2rad(max_rotation_deg)] * 3)
    for _ in range(rounds):
        changed = True
        while changed:
            changed = False
            for axis in range(7):
                for direction in (-1.0, 1.0):
                    candidate = parameters.copy()
                    candidate[axis] += direction * steps[axis]
                    if abs(candidate[axis]) > bounds[axis] + 1e-12:
                        continue
                    loss, _ = spatiotemporal_objective(
                        points, samples, stamps, body_T_camera, K, images,
                        candidate, edge_percentile=edge_percentile,
                        max_distance=max_distance,
                        reference_edge_points=reference,
                        image_edge_masks=edge_masks,
                        depth_support=depth_support,
                        contour_points_by_view=contour_points_by_view,
                        orientation_max_angle_deg=orientation_max_angle_deg)
                    if loss + 1e-12 < best_loss:
                        parameters, best_loss = candidate, loss
                        changed = True
        steps *= 0.5
    _, after = spatiotemporal_objective(
        points, samples, stamps, body_T_camera, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance,
        reference_edge_points=reference, image_edge_masks=edge_masks,
        depth_support=depth_support,
        contour_points_by_view=contour_points_by_view,
        orientation_max_angle_deg=orientation_max_angle_deg)
    before['loss'] = base_loss
    after['loss'] = best_loss
    return parameters, before, after


def optimize_spatiotemporal_production(
        points: np.ndarray, samples: list[pi.TrajectorySample],
        stamps: np.ndarray, body_T_camera: np.ndarray, K: np.ndarray,
        images: list[np.ndarray], *, scales: tuple[float, ...] = (0.25, 0.5, 1.0),
        rounds_per_level: int = 2, time_step: float = 0.02,
        translation_step: float = 0.02, rotation_step_deg: float = 0.2,
        max_time_offset: float = 0.1, max_translation: float = 0.1,
        max_rotation_deg: float = 2.0, auto_bound_expansions: int = 2,
        bound_expansion_factor: float = 2.0, observability_scale: float = 1.0,
        minimum_curvature: float = 1e-6, maximum_condition: float = 1e6,
        maximum_time_translation_correlation: float = 0.98,
        edge_percentile: float = 95.0,
        max_distance: int = 12,
        depth_support: dict | None = None,
        contour_points_by_view: list[np.ndarray] | None = None,
        orientation_max_angle_deg: float = 0.0
        ) -> tuple[np.ndarray, dict, dict, dict]:
    """Optimize a pyramid and audit local identifiability before adoption."""
    if (not scales or any(not 0.0 < scale <= 1.0 for scale in scales) or
            tuple(sorted(scales)) != scales):
        raise ValueError('pyramid scales must be sorted values in (0, 1]')
    if not np.isclose(observability_scale, scales[-1]):
        raise ValueError('observability scale must match the finest pyramid '
                         'scale')
    parameters = np.zeros(7, dtype=np.float64)
    base_steps = np.array([time_step] + [translation_step] * 3 +
                          [np.deg2rad(rotation_step_deg)] * 3)
    bounds = np.array([max_time_offset] + [max_translation] * 3 +
                      [np.deg2rad(max_rotation_deg)] * 3)
    initial_bounds = bounds.copy()
    levels = []

    def level_objective(scale, reference_parameters=None):
        level_images = [stc.downsample_nearest(image, scale)
                        for image in images]
        level_K = stc.scale_intrinsics(K, scale)
        edge_masks = [(image_edge_field(image, edge_percentile)
                       if orientation_max_angle_deg > 0.0 else
                       image_edges(image, edge_percentile))
                      for image in level_images]
        reference_parameters = (parameters if reference_parameters is None
                                else reference_parameters)
        _, reference_metrics = spatiotemporal_objective(
            points, samples, stamps, body_T_camera, level_K, level_images,
            reference_parameters, edge_percentile=edge_percentile,
            max_distance=max(2, int(round(max_distance * scale))),
            image_edge_masks=edge_masks, depth_support=depth_support,
            contour_points_by_view=contour_points_by_view,
            orientation_max_angle_deg=orientation_max_angle_deg)
        reference_edges = reference_metrics['edge_points']

        def objective(candidate):
            loss, _ = spatiotemporal_objective(
                points, samples, stamps, body_T_camera, level_K, level_images,
                candidate, edge_percentile=edge_percentile,
                max_distance=max(2, int(round(max_distance * scale))),
                reference_edge_points=reference_edges,
                image_edge_masks=edge_masks, depth_support=depth_support,
                contour_points_by_view=contour_points_by_view,
                orientation_max_angle_deg=orientation_max_angle_deg)
            return loss
        return objective

    base_loss, before = spatiotemporal_objective(
        points, samples, stamps, body_T_camera, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance,
        depth_support=depth_support,
        contour_points_by_view=contour_points_by_view,
        orientation_max_angle_deg=orientation_max_angle_deg)
    before['loss'] = base_loss
    for scale in scales:
        objective = level_objective(scale, np.zeros(7))
        multiplier = max(1.0, np.sqrt(1.0 / scale))
        parameters, loss, search = stc.bounded_coordinate_search(
            objective, parameters, base_steps * multiplier, bounds,
            rounds=rounds_per_level)
        levels.append({'scale': scale, 'loss': loss, 'search': search,
                       'bounds_dt_s_xyz_m_rpy_rad': bounds.tolist()})

    expansions = []
    boundary_tolerance = base_steps * 0.5
    for expansion in range(auto_bound_expansions):
        touching = stc.boundary_axes(
            parameters, bounds, tolerance=boundary_tolerance)
        if not touching:
            break
        old_bounds = bounds.copy()
        for name in touching:
            axis = stc.PARAMETER_NAMES.index(name)
            bounds[axis] *= bound_expansion_factor
        objective = level_objective(scales[-1], np.zeros(7))
        parameters, loss, search = stc.bounded_coordinate_search(
            objective, parameters, base_steps * 0.5, bounds,
            rounds=rounds_per_level)
        expansions.append({
            'iteration': expansion + 1, 'trigger_axes': touching,
            'old_bounds_dt_s_xyz_m_rpy_rad': old_bounds.tolist(),
            'new_bounds_dt_s_xyz_m_rpy_rad': bounds.tolist(),
            'loss': loss, 'search': search,
        })

    final_loss, after = spatiotemporal_objective(
        points, samples, stamps, body_T_camera, K, images, parameters,
        edge_percentile=edge_percentile, max_distance=max_distance,
        reference_edge_points=before['edge_points'],
        depth_support=depth_support,
        contour_points_by_view=contour_points_by_view,
        orientation_max_angle_deg=orientation_max_angle_deg)
    after['loss'] = final_loss
    audit_objective = level_objective(observability_scale, parameters)
    observability = stc.finite_difference_observability(
        audit_objective, parameters, base_steps * 0.5,
        minimum_curvature=minimum_curvature,
        maximum_condition=maximum_condition,
        maximum_correlation=maximum_time_translation_correlation)
    report = {
        'mode': 'multi_resolution_production',
        'depth_support': depth_support,
        'fixed_contours': contour_points_by_view is not None,
        'orientation_max_angle_deg': orientation_max_angle_deg,
        'pyramid_scales': list(scales),
        'levels': levels,
        'initial_bounds_dt_s_xyz_m_rpy_rad': initial_bounds.tolist(),
        'final_bounds_dt_s_xyz_m_rpy_rad': bounds.tolist(),
        'bound_expansions': expansions,
        'boundary_tolerance_dt_s_xyz_m_rpy_rad':
            boundary_tolerance.tolist(),
        'boundary_axes': stc.boundary_axes(
            parameters, bounds, tolerance=boundary_tolerance),
        'observability_scale': observability_scale,
        'observability': observability,
    }
    return parameters, before, after, report


def calibration_acceptance(train_before: dict, train_after: dict,
                           heldout_before: dict, heldout_after: dict, *,
                           minimum_edge_points: int,
                           minimum_heldout_improvement: float,
                           minimum_supported_edge_fraction: float = 0.0
                           ) -> tuple[bool, str | None]:
    """Apply the independent validation gate used before exporting poses."""
    enough_edges = (
        train_before['edge_points'] >= minimum_edge_points and
        heldout_before['edge_points'] >= minimum_edge_points)
    if not enough_edges:
        return False, 'insufficient_edge_support'
    if (train_before.get('supported_edge_fraction', 1.0) <
            minimum_supported_edge_fraction or
            heldout_before.get('supported_edge_fraction', 1.0) <
            minimum_supported_edge_fraction):
        return False, 'insufficient_supported_edge_fraction'
    train_loss = train_after['loss']
    heldout_loss = heldout_after['loss']
    heldout_limit = heldout_before['loss'] * (
        1.0 - minimum_heldout_improvement)
    heldout_failed = (heldout_loss > heldout_limit or
                      (minimum_heldout_improvement == 0.0 and
                       heldout_loss >= heldout_limit))
    if (not np.isfinite(train_loss) or not np.isfinite(heldout_loss) or
            train_loss >= train_before['loss'] or heldout_failed):
        return False, 'heldout_or_training_loss_did_not_improve'
    return True, None


def write_recomposed_transforms(source: Path, output: Path,
                                viewmats: np.ndarray, *,
                                calibration: dict | None = None) -> Path:
    """Write continuous-time recomposed poses while preserving frame metadata."""
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output:
        raise ValueError('recomposed transforms output must differ from source')
    document = json.loads(source.read_text())
    dataset = tg.load_transforms(source)
    if len(document['frames']) != len(viewmats):
        raise ValueError('frame and recomposed viewmat counts differ')
    for frame, viewmat, image_path in zip(
            document['frames'], viewmats, dataset['image_paths']):
        frame['transform_matrix'] = (
            np.linalg.inv(viewmat) @ pi.ROS_OPTICAL_TO_OPENGL).tolist()
        frame['file_path'] = os.path.relpath(image_path, output.parent)
    if calibration is not None:
        document['spatiotemporal_calibration'] = calibration
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2) + '\n')
    return output


def calibration_metadata(optimization: dict) -> dict:
    """Return the compact uncertainty contract consumed by RGB fusion."""
    production = optimization.get('production_calibration')
    observability = (production.get('observability')
                     if production is not None else None)
    metadata = {
        'accepted': bool(optimization['accepted']),
        'parameters_dt_s_xyz_m_rpy_deg':
            optimization['parameters_dt_s_xyz_m_rpy_deg'],
        'boundary_axes': optimization['boundary_axes'],
    }
    if observability is not None:
        metadata.update({
            'uncertainty_dt_s_xyz_m_rpy_rad':
                observability['uncertainty_dt_s_xyz_m_rpy_rad'],
            'condition_number': observability['condition_number'],
            'maximum_abs_time_translation_correlation':
                observability[
                    'maximum_abs_time_translation_correlation'],
        })
    return metadata


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
    parser.add_argument('--max-points', type=int, default=0,
                        help='deterministically subsample this many geometry '
                             'points for evaluation (0 keeps all)')
    parser.add_argument('--edge-percentile', type=float, default=95.0,
                        help='retain this percentile of nonzero image gradients; '
                             '95 focuses the metric on structural edges')
    parser.add_argument('--max-distance', type=int, default=12)
    parser.add_argument('--depth-support-radius', type=int, default=0,
                        help='require same-surface neighbours around each LiDAR '
                             'edge pixel; 0 preserves the raw edge metric')
    parser.add_argument('--depth-support-min-neighbors', type=int, default=4)
    parser.add_argument('--depth-support-absolute', type=float, default=0.10,
                        help='metre tolerance for same-surface support')
    parser.add_argument('--depth-support-relative', type=float, default=0.01)
    parser.add_argument('--fixed-contours', action='store_true',
                        help='extract full-density 3D depth contours once and '
                             'reuse them throughout calibration')
    parser.add_argument('--contour-association-distance', type=int, default=0,
                        help='initial image-edge association radius; 0 keeps '
                             'geometry-only contours and avoids pose bias')
    parser.add_argument('--contour-max-points-per-view', type=int, default=50000)
    parser.add_argument('--contour-min-points-per-view', type=int, default=500)
    parser.add_argument('--orientation-max-angle-deg', type=float, default=0.0,
                        help='maximum normal-axis disagreement for contour '
                             'matching; 0 disables orientation gating')
    parser.add_argument('--diagnostics-dir', type=Path,
                        help='write worst-view residual overlays and summary')
    parser.add_argument('--worst-views', type=int, default=10,
                        help='number of residual overlays to write')
    optimization_mode = parser.add_mutually_exclusive_group()
    optimization_mode.add_argument('--optimize-extrinsic', action='store_true')
    optimization_mode.add_argument('--optimize-spatiotemporal', action='store_true')
    parser.add_argument('--production-calibration', action='store_true',
                        help='enable pyramid search, stratified holdout, bound '
                             'expansion, and observability rejection')
    parser.add_argument('--trajectory', type=Path,
                        help='dense TUM world<-body trajectory; required for '
                             'spatiotemporal optimization')
    parser.add_argument('--optimization-rounds', type=int, default=3)
    parser.add_argument('--time-step', type=float, default=0.02)
    parser.add_argument('--translation-step', type=float, default=0.02)
    parser.add_argument('--rotation-step-deg', type=float, default=0.2)
    parser.add_argument('--max-time-offset', type=float, default=0.1)
    parser.add_argument('--max-translation', type=float, default=0.1)
    parser.add_argument('--max-rotation-deg', type=float, default=2.0)
    parser.add_argument('--holdout-modulo', type=int, default=5,
                        help='every Nth selected view is validation-only')
    parser.add_argument('--holdout-fraction', type=float, default=0.2)
    parser.add_argument('--spatial-segments', type=int, default=4)
    parser.add_argument('--pyramid-scales', default='0.25,0.5,1.0')
    parser.add_argument('--rounds-per-pyramid-level', type=int, default=2)
    parser.add_argument('--auto-bound-expansions', type=int, default=2)
    parser.add_argument('--bound-expansion-factor', type=float, default=2.0)
    parser.add_argument('--observability-scale', type=float, default=1.0,
                        help='audit resolution; must match the finest pyramid '
                             'scale')
    parser.add_argument('--minimum-curvature', type=float, default=1e-6)
    parser.add_argument('--maximum-condition', type=float, default=1e6)
    parser.add_argument('--maximum-time-translation-correlation', type=float,
                        default=0.98)
    parser.add_argument('--minimum-edge-points', type=int, default=50)
    parser.add_argument('--minimum-supported-edge-fraction', type=float,
                        default=0.0,
                        help='reject calibration when surface filtering keeps '
                             'less than this raw-edge fraction')
    parser.add_argument('--minimum-heldout-improvement', type=float, default=0.0,
                        help='fractional held-out loss reduction required to '
                             'accept and export the correction')
    parser.add_argument('--corrected-transforms-out', type=Path)
    args = parser.parse_args()
    if (args.view_stride < 1 or args.max_distance < 1 or
            args.optimization_rounds < 1 or args.translation_step <= 0.0 or
            args.rotation_step_deg <= 0.0 or args.time_step <= 0.0 or
            args.max_time_offset < 0.0 or args.max_translation < 0.0 or
            args.max_rotation_deg < 0.0 or args.holdout_modulo < 2 or
            not 0.0 < args.holdout_fraction < 0.5 or
            args.spatial_segments < 1 or args.rounds_per_pyramid_level < 1 or
            args.auto_bound_expansions < 0 or
            args.bound_expansion_factor <= 1.0 or
            not 0.0 < args.observability_scale <= 1.0 or
            args.minimum_curvature <= 0.0 or args.maximum_condition <= 1.0 or
            not 0.0 <= args.maximum_time_translation_correlation < 1.0 or
            args.minimum_edge_points < 1 or args.max_points < 0 or
            args.worst_views < 1 or
            args.depth_support_radius < 0 or
            args.depth_support_min_neighbors < 1 or
            args.depth_support_absolute < 0.0 or
            args.depth_support_relative < 0.0 or
            (args.contour_association_distance != 0 and
             args.contour_association_distance < args.max_distance) or
            args.contour_max_points_per_view < 1 or
            args.contour_min_points_per_view < 1 or
            args.contour_min_points_per_view > args.contour_max_points_per_view or
            not 0.0 <= args.orientation_max_angle_deg <= 90.0 or
            not 0.0 <= args.minimum_supported_edge_fraction <= 1.0 or
            not 0.0 <= args.minimum_heldout_improvement < 1.0):
        raise SystemExit('stride, distance, rounds, and search steps must be > 0')
    if args.optimize_spatiotemporal and args.trajectory is None:
        raise SystemExit('--optimize-spatiotemporal requires --trajectory')
    if args.production_calibration and not args.optimize_spatiotemporal:
        raise SystemExit('--production-calibration requires '
                         '--optimize-spatiotemporal')
    if ((args.optimize_spatiotemporal or args.optimize_extrinsic) and
            args.fixed_contours and args.contour_association_distance > 0):
        raise SystemExit('image-associated fixed contours are diagnostic-only; '
                         'calibration requires geometry-only association 0')
    if args.orientation_max_angle_deg > 0.0 and not args.fixed_contours:
        raise SystemExit('--orientation-max-angle-deg requires '
                         '--fixed-contours')
    try:
        pyramid_scales = tuple(float(item) for item in
                               args.pyramid_scales.split(','))
    except ValueError as exc:
        raise SystemExit('--pyramid-scales must be comma-separated numbers') \
            from exc
    if (args.production_calibration and pyramid_scales and
            not np.isclose(args.observability_scale, pyramid_scales[-1])):
        raise SystemExit('--observability-scale must match the finest '
                         '--pyramid-scales value')
    if (args.depth_support_radius > 0 and
            args.depth_support_min_neighbors >
            (2 * args.depth_support_radius + 1) ** 2 - 1):
        raise SystemExit('--depth-support-min-neighbors exceeds its window')
    depth_support = None
    if args.depth_support_radius > 0:
        depth_support = {
            'radius': args.depth_support_radius,
            'min_neighbors': args.depth_support_min_neighbors,
            'absolute': args.depth_support_absolute,
            'relative': args.depth_support_relative,
        }
    import imageio as iio
    full_points, _ = pcio.read_ply_xyz(args.pointcloud)
    points = full_points
    if args.max_points and len(points) > args.max_points:
        indices = np.linspace(
            0, len(points) - 1, args.max_points, dtype=np.int64)
        points = points[indices]
    dataset = tg.load_transforms(args.transforms)
    viewmats = np.asarray(dataset['viewmats'], dtype=np.float64)
    selected = list(range(0, len(viewmats), args.view_stride))
    images = [np.asarray(iio.imread(dataset['image_paths'][index]))
              for index in selected]
    contour_points_by_view = None
    contour_report = None
    if args.fixed_contours:
        contour_points_by_view, contour_report = extract_fixed_contours(
            full_points, viewmats[selected], dataset['K'], images,
            edge_percentile=args.edge_percentile,
            association_distance=args.contour_association_distance,
            max_points_per_view=args.contour_max_points_per_view,
            depth_support=depth_support)
        insufficient = [
            selected[ordinal]
            for ordinal, bank in enumerate(contour_points_by_view)
            if len(bank) < args.contour_min_points_per_view]
        if insufficient:
            raise SystemExit(
                'fixed contour extraction has insufficient points in views: ' +
                ','.join(str(index) for index in insufficient))
    contour_by_index = (dict(zip(selected, contour_points_by_view))
                        if contour_points_by_view is not None else {})
    optimization = None
    delta = np.eye(4)
    effective_viewmats = viewmats.copy()
    if args.optimize_spatiotemporal:
        samples = pi.read_tum_trajectory(args.trajectory)
        stamps = frame_stamps(args.transforms)
        if len(stamps) != len(viewmats):
            raise SystemExit('frame stamp and camera pose counts differ')
        body_T_camera, consistency = infer_body_T_camera(
            samples, stamps, viewmats)
        if (consistency['translation_spread_p95_m'] > 0.001 or
                consistency['rotation_spread_p95_deg'] > 0.05):
            raise SystemExit(
                'camera poses are inconsistent with one static body-to-camera '
                'extrinsic (p95 spread exceeds 1 mm or 0.05 degree)')
        excitation = trajectory_excitation(samples, stamps[selected])
        if not excitation['time_offset_observable']:
            raise SystemExit(
                'time offset is unobservable: selected views contain less than '
                '0.05 m translation and 1 degree rotation')
        split_report = None
        if args.production_calibration:
            selected_poses = np.asarray([
                pi.interpolate_pose(samples, float(stamps[index]))
                for index in selected])
            train, heldout, split_report = stc.stratified_view_split(
                stamps, selected_poses, selected,
                holdout_fraction=args.holdout_fraction,
                spatial_segments=args.spatial_segments)
        else:
            heldout = [index for ordinal, index in enumerate(selected)
                       if ordinal % args.holdout_modulo == 0]
            train = [index for index in selected if index not in heldout]
        if not train or not heldout:
            raise SystemExit('spatiotemporal calibration needs train and held-out views')
        image_by_index = dict(zip(selected, images))
        train_images = [image_by_index[index] for index in train]
        heldout_images = [image_by_index[index] for index in heldout]
        train_contours = ([contour_by_index[index] for index in train]
                          if contour_points_by_view is not None else None)
        heldout_contours = ([contour_by_index[index] for index in heldout]
                            if contour_points_by_view is not None else None)
        production_report = None
        if args.production_calibration:
            (parameters, train_before, train_after,
             production_report) = optimize_spatiotemporal_production(
                points, samples, stamps[train], body_T_camera, dataset['K'],
                train_images, scales=pyramid_scales,
                rounds_per_level=args.rounds_per_pyramid_level,
                time_step=args.time_step,
                translation_step=args.translation_step,
                rotation_step_deg=args.rotation_step_deg,
                max_time_offset=args.max_time_offset,
                max_translation=args.max_translation,
                max_rotation_deg=args.max_rotation_deg,
                auto_bound_expansions=args.auto_bound_expansions,
                bound_expansion_factor=args.bound_expansion_factor,
                observability_scale=args.observability_scale,
                minimum_curvature=args.minimum_curvature,
                maximum_condition=args.maximum_condition,
                maximum_time_translation_correlation=(
                    args.maximum_time_translation_correlation),
                edge_percentile=args.edge_percentile,
                max_distance=args.max_distance,
                depth_support=depth_support,
                contour_points_by_view=train_contours,
                orientation_max_angle_deg=args.orientation_max_angle_deg)
        else:
            parameters, train_before, train_after = optimize_spatiotemporal(
                points, samples, stamps[train], body_T_camera, dataset['K'],
                train_images, rounds=args.optimization_rounds,
                time_step=args.time_step,
                translation_step=args.translation_step,
                rotation_step_deg=args.rotation_step_deg,
                max_time_offset=args.max_time_offset,
                max_translation=args.max_translation,
                max_rotation_deg=args.max_rotation_deg,
                edge_percentile=args.edge_percentile,
                max_distance=args.max_distance,
                depth_support=depth_support,
                contour_points_by_view=train_contours,
                orientation_max_angle_deg=args.orientation_max_angle_deg)
        zero = np.zeros(7)
        heldout_before_loss, heldout_before = spatiotemporal_objective(
            points, samples, stamps[heldout], body_T_camera, dataset['K'],
            heldout_images, zero, edge_percentile=args.edge_percentile,
            max_distance=args.max_distance, depth_support=depth_support,
            contour_points_by_view=heldout_contours,
            orientation_max_angle_deg=args.orientation_max_angle_deg)
        heldout_after_loss, heldout_after = spatiotemporal_objective(
            points, samples, stamps[heldout], body_T_camera, dataset['K'],
            heldout_images, parameters,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance,
            reference_edge_points=heldout_before['edge_points'],
            depth_support=depth_support,
            contour_points_by_view=heldout_contours,
            orientation_max_angle_deg=args.orientation_max_angle_deg)
        heldout_before['loss'], heldout_after['loss'] = (
            heldout_before_loss, heldout_after_loss)
        accepted, rejection_reason = calibration_acceptance(
            train_before, train_after, heldout_before, heldout_after,
            minimum_edge_points=args.minimum_edge_points,
            minimum_heldout_improvement=args.minimum_heldout_improvement,
            minimum_supported_edge_fraction=(
                args.minimum_supported_edge_fraction))
        rejection_reasons = ([rejection_reason]
                             if rejection_reason is not None else [])
        parameter_bounds = np.array([
            args.max_time_offset, args.max_translation,
            args.max_translation, args.max_translation,
            args.max_rotation_deg, args.max_rotation_deg,
            args.max_rotation_deg])
        reported_parameters = np.array(
            [parameters[0], *parameters[1:4],
             *np.rad2deg(parameters[4:])])
        names = ['dt', 'tx', 'ty', 'tz', 'roll', 'pitch', 'yaw']
        boundary_axes = [
            name for name, value, bound in zip(
                names, reported_parameters, parameter_bounds)
            if bound > 0.0 and abs(value) >= bound - 1e-12]
        if production_report is not None:
            boundary_axes = production_report['boundary_axes']
            if boundary_axes:
                accepted = False
                rejection_reasons.append('search_boundary_reached')
            if not production_report['observability']['observable']:
                accepted = False
                rejection_reasons.append('calibration_not_observable')
        rejection_reason = (rejection_reasons[-1]
                            if rejection_reasons else None)
        if accepted:
            effective_viewmats = recompose_viewmats(
                samples, stamps, body_T_camera, parameters)
        optimization = {
            'accepted': accepted,
            'parameters_dt_s_xyz_m_rpy_deg': [float(parameters[0])] +
            parameters[1:4].tolist() + np.rad2deg(parameters[4:]).tolist(),
            'body_T_camera_initial': body_T_camera.tolist(),
            'extrinsic_consistency': consistency,
            'trajectory_excitation': excitation,
            'train_view_indices': train,
            'heldout_view_indices': heldout,
            'validation_split': split_report,
            'minimum_edge_points': args.minimum_edge_points,
            'minimum_supported_edge_fraction':
                args.minimum_supported_edge_fraction,
            'search_bounds_dt_s_xyz_m_rpy_deg': parameter_bounds.tolist(),
            'boundary_axes': boundary_axes,
            'production_calibration': production_report,
            'train': {'before': train_before, 'after': train_after},
            'heldout': {'before': heldout_before, 'after': heldout_after},
            'rejection_reason': rejection_reason,
            'rejection_reasons': rejection_reasons,
        }
    elif args.optimize_extrinsic:
        parameters, before, after = optimize_correction(
            points, viewmats[selected], dataset['K'], images,
            rounds=args.optimization_rounds,
            translation_step=args.translation_step,
            rotation_step_deg=args.rotation_step_deg,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance,
            depth_support=depth_support,
            contour_points_by_view=contour_points_by_view,
            orientation_max_angle_deg=args.orientation_max_angle_deg)
        delta = correction_matrix(parameters)
        effective_viewmats = np.asarray(
            [delta @ viewmat for viewmat in viewmats])
        optimization = {
            'parameters_xyz_m_rpy_deg': parameters[:3].tolist() +
            np.rad2deg(parameters[3:]).tolist(),
            'camera_correction_matrix': delta.tolist(),
            'before': before, 'after': after,
        }
    per_view = []
    for ordinal, (index, image) in enumerate(zip(selected, images)):
        score = score_view(
            points, effective_viewmats[index], dataset['K'], image,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance, depth_support=depth_support,
            contour_points=(contour_points_by_view[ordinal]
                            if contour_points_by_view is not None else None),
            orientation_max_angle_deg=args.orientation_max_angle_deg)
        score['view_index'] = index
        per_view.append(score)
    valid = [item for item in per_view if item['edge_points'] > 0]
    if not valid:
        raise SystemExit('no LiDAR depth edges were measurable')
    weights = np.asarray([item['edge_points'] for item in valid], dtype=float)
    report = {'pointcloud': str(args.pointcloud.resolve()),
              'transforms': str(args.transforms.resolve()),
              'points_scored': len(points),
              'full_density_points': len(full_points),
              'views_scored': len(valid), 'edge_points': int(weights.sum()),
              'depth_support': depth_support,
              'fixed_contours': contour_report,
              'orientation_max_angle_deg': args.orientation_max_angle_deg}
    raw_edges = sum(item['raw_edge_points'] for item in valid)
    report['raw_edge_points'] = raw_edges
    report['supported_edge_fraction'] = float(
        weights.sum() / max(raw_edges, 1))
    for name in ('median_px', 'p90_px', 'inlier_2px',
                 'out_of_range_fraction'):
        report[f'weighted_{name}'] = float(np.average(
            [item[name] for item in valid], weights=weights))
    if args.optimize_spatiotemporal:
        report['spatiotemporal_optimization'] = optimization
        if args.corrected_transforms_out is not None:
            corrected = write_recomposed_transforms(
                args.transforms, args.corrected_transforms_out,
                effective_viewmats,
                calibration=calibration_metadata(optimization))
            report['corrected_transforms'] = str(corrected)
    elif optimization is not None:
        report['extrinsic_optimization'] = optimization
        if args.corrected_transforms_out is not None:
            corrected = write_corrected_transforms(
                args.transforms, args.corrected_transforms_out, delta)
            report['corrected_transforms'] = str(corrected)
    elif args.corrected_transforms_out is not None:
        raise SystemExit('--corrected-transforms-out requires an optimization mode')
    report['per_view'] = per_view
    if args.diagnostics_dir is not None:
        report['diagnostics'] = write_residual_diagnostics(
            args.diagnostics_dir, points, effective_viewmats, dataset['K'],
            selected, images, per_view, worst_views=args.worst_views,
            edge_percentile=args.edge_percentile,
            max_distance=args.max_distance, depth_support=depth_support,
            contour_points_by_view=contour_points_by_view,
            orientation_max_angle_deg=args.orientation_max_angle_deg)
        report['diagnostics']['directory'] = str(args.diagnostics_dir.resolve())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items()
                      if key != 'per_view'}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
