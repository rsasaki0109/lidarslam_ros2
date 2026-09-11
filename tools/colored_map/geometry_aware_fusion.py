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

"""Pure NumPy geometry guards for camera-to-LiDAR RGB fusion."""

from __future__ import annotations

from typing import Optional, Sequence

import numpy as np


def neighborhood_depth_statistics(
        depth: np.ndarray, u: np.ndarray, v: np.ndarray,
        radii: int | np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return finite min/max depth and support in each circular pixel window."""
    image = np.asarray(depth, dtype=np.float64)
    columns = np.asarray(u, dtype=np.int64)
    rows = np.asarray(v, dtype=np.int64)
    if image.ndim != 2:
        raise ValueError('depth must be HxW')
    if columns.shape != rows.shape:
        raise ValueError('u and v must have matching shapes')
    radius = np.broadcast_to(
        np.asarray(radii, dtype=np.int64), columns.shape)
    if np.any(radius < 0):
        raise ValueError('depth-neighborhood radii must be non-negative')
    if (np.any(columns < 0) or np.any(columns >= image.shape[1]) or
            np.any(rows < 0) or np.any(rows >= image.shape[0])):
        raise ValueError('query pixels must lie inside the depth image')
    minimum = np.full(columns.shape, np.inf, dtype=np.float64)
    maximum = np.full(columns.shape, -np.inf, dtype=np.float64)
    support = np.zeros(columns.shape, dtype=np.uint16)
    largest = int(radius.max()) if radius.size else 0
    for dy in range(-largest, largest + 1):
        for dx in range(-largest, largest + 1):
            active = radius * radius >= dx * dx + dy * dy
            query_u, query_v = columns + dx, rows + dy
            active &= ((query_u >= 0) & (query_u < image.shape[1]) &
                       (query_v >= 0) & (query_v < image.shape[0]))
            if not active.any():
                continue
            indices = np.flatnonzero(active)
            values = image[query_v[indices], query_u[indices]]
            finite = np.isfinite(values)
            indices, values = indices[finite], values[finite]
            minimum[indices] = np.minimum(minimum[indices], values)
            maximum[indices] = np.maximum(maximum[indices], values)
            support[indices] += 1
    return minimum, maximum, support


def mask_neighborhood_any(mask: np.ndarray, u: np.ndarray, v: np.ndarray,
                          radii: int | np.ndarray) -> np.ndarray:
    """Return whether any excluded mask pixel lies in each circular window."""
    excluded = np.asarray(mask, dtype=bool)
    if excluded.ndim != 2:
        raise ValueError('exclusion mask must be HxW')
    depth = np.where(excluded, 1.0, np.inf)
    _, _, support = neighborhood_depth_statistics(depth, u, v, radii)
    return support > 0


def camera_motion_rates(viewmats: np.ndarray,
                        timestamps: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    """Estimate per-view translational and angular camera speed."""
    views = np.asarray(viewmats, dtype=np.float64)
    stamps = np.asarray(timestamps, dtype=np.float64)
    if views.ndim != 3 or views.shape[1:] != (4, 4):
        raise ValueError('viewmats must be Nx4x4')
    if stamps.shape != (len(views),) or not np.all(np.isfinite(stamps)):
        raise ValueError('timestamps must contain one finite value per view')
    if len(views) == 0:
        return np.zeros(0), np.zeros(0)
    if len(views) == 1:
        return np.zeros(1), np.zeros(1)
    camera_poses = np.linalg.inv(views)
    centres = camera_poses[:, :3, 3]
    segment_dt = np.diff(stamps)
    if np.any(segment_dt <= 0.0):
        raise ValueError('timestamps must be strictly increasing')
    linear_segment = np.linalg.norm(np.diff(centres, axis=0), axis=1) / segment_dt
    angular_segment = []
    for first, second, duration in zip(
            camera_poses, camera_poses[1:], segment_dt):
        relative = first[:3, :3].T @ second[:3, :3]
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        angular_segment.append(np.arccos(cosine) / duration)
    angular_segment = np.asarray(angular_segment)
    linear = np.r_[linear_segment[0],
                   0.5 * (linear_segment[:-1] + linear_segment[1:]),
                   linear_segment[-1]]
    angular = np.r_[angular_segment[0],
                    0.5 * (angular_segment[:-1] + angular_segment[1:]),
                    angular_segment[-1]]
    return linear, angular


def calibration_pixel_radii(
        depth: np.ndarray, focal_px: float, calibration: Optional[dict], *,
        linear_speed: float = 0.0, angular_speed: float = 0.0,
        sigma_multiplier: float = 0.0,
        maximum_radius: int = 12) -> np.ndarray:
    """Propagate 7DoF calibration uncertainty to conservative pixel radii."""
    ranges = np.asarray(depth, dtype=np.float64)
    if np.any(ranges <= 0.0) or not np.all(np.isfinite(ranges)):
        raise ValueError('depth must contain finite positive ranges')
    if focal_px <= 0.0 or sigma_multiplier < 0.0 or maximum_radius < 0:
        raise ValueError('focal, sigma multiplier, and maximum radius are invalid')
    if sigma_multiplier == 0.0:
        return np.zeros(ranges.shape, dtype=np.int16)
    if calibration is None:
        raise ValueError('calibration uncertainty metadata is required')
    if not calibration.get('accepted', False):
        raise ValueError('calibration uncertainty requires an accepted calibration')
    uncertainty = np.asarray(
        calibration.get('uncertainty_dt_s_xyz_m_rpy_rad'), dtype=np.float64)
    if (uncertainty.shape != (7,) or not np.all(np.isfinite(uncertainty)) or
            np.any(uncertainty < 0.0)):
        raise ValueError('calibration uncertainty must contain seven finite '
                         'non-negative values')
    time_sigma = uncertainty[0]
    translation_sigma = np.sqrt(
        np.sum(uncertainty[1:4] ** 2) + (linear_speed * time_sigma) ** 2)
    rotation_sigma = np.sqrt(
        np.sum(uncertainty[4:7] ** 2) + (angular_speed * time_sigma) ** 2)
    pixel_sigma = float(focal_px) * np.sqrt(
        (translation_sigma / ranges) ** 2 + rotation_sigma ** 2)
    return np.clip(
        np.ceil(sigma_multiplier * pixel_sigma), 0, maximum_radius
    ).astype(np.int16)
