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

"""Plane-prior patch warping for direct camera alignment (numpy only)."""

from __future__ import annotations

import numpy as np


def plane_homography(K: np.ndarray, target_T_reference: np.ndarray,
                     normal_reference: np.ndarray,
                     point_reference: np.ndarray) -> np.ndarray:
    """Return the reference-pixel to target-pixel plane homography.

    Implements FAST-LIVO2 equation (13):
    ``K (R + t n^T / (n^T p)) K^-1``. Inputs use OpenCV camera coordinates
    (+z forward). The normal sign is irrelevant because it appears in both the
    numerator and denominator.
    """
    intr = np.asarray(K, dtype=np.float64)
    transform = np.asarray(target_T_reference, dtype=np.float64)
    normal = np.asarray(normal_reference, dtype=np.float64).reshape(3)
    point = np.asarray(point_reference, dtype=np.float64).reshape(3)
    if intr.shape != (3, 3):
        raise ValueError(f'K must be 3x3, got {intr.shape}')
    if transform.shape != (4, 4):
        raise ValueError(f'target_T_reference must be 4x4, got {transform.shape}')
    if not np.all(np.isfinite(intr)) or not np.all(np.isfinite(transform)):
        raise ValueError('K and transform must be finite')
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm <= 1.0e-12:
        raise ValueError('plane normal must be non-zero')
    normal = normal / normal_norm
    denominator = float(normal @ point)
    if abs(denominator) <= 1.0e-9:
        raise ValueError('plane passes through the reference camera centre')
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    affine = rotation + np.outer(translation, normal) / denominator
    return intr @ affine @ np.linalg.inv(intr)


def warp_pixels(homography: np.ndarray, pixels: np.ndarray
                ) -> tuple[np.ndarray, np.ndarray]:
    """Warp Nx2 pixels and return coordinates plus a finite/front mask."""
    matrix = np.asarray(homography, dtype=np.float64)
    uv = np.asarray(pixels, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f'homography must be 3x3, got {matrix.shape}')
    if uv.ndim != 2 or uv.shape[1] != 2:
        raise ValueError(f'pixels must be Nx2, got {uv.shape}')
    homogeneous = np.column_stack((uv, np.ones(len(uv)))) @ matrix.T
    valid = np.isfinite(homogeneous).all(axis=1) & (homogeneous[:, 2] > 1.0e-9)
    result = np.full((len(uv), 2), np.nan, dtype=np.float64)
    result[valid] = homogeneous[valid, :2] / homogeneous[valid, 2, None]
    return result, valid


def bilinear_sample(image: np.ndarray, pixels: np.ndarray
                    ) -> tuple[np.ndarray, np.ndarray]:
    """Sample a mono image at Nx2 pixels, returning values and in-frame mask."""
    source = np.asarray(image, dtype=np.float32)
    uv = np.asarray(pixels, dtype=np.float64)
    if source.ndim != 2:
        raise ValueError(f'image must be HxW mono, got {source.shape}')
    if uv.ndim != 2 or uv.shape[1] != 2:
        raise ValueError(f'pixels must be Nx2, got {uv.shape}')
    x, y = uv[:, 0], uv[:, 1]
    valid = (np.isfinite(uv).all(axis=1) & (x >= 0.0) & (y >= 0.0) &
             (x <= source.shape[1] - 1) & (y <= source.shape[0] - 1))
    values = np.full(len(uv), np.nan, dtype=np.float32)
    if not valid.any():
        return values, valid
    x0 = np.floor(x[valid]).astype(np.int64)
    y0 = np.floor(y[valid]).astype(np.int64)
    x1 = np.minimum(x0 + 1, source.shape[1] - 1)
    y1 = np.minimum(y0 + 1, source.shape[0] - 1)
    wx = (x[valid] - x0).astype(np.float32)
    wy = (y[valid] - y0).astype(np.float32)
    top = source[y0, x0] * (1.0 - wx) + source[y0, x1] * wx
    bottom = source[y1, x0] * (1.0 - wx) + source[y1, x1] * wx
    values[valid] = top * (1.0 - wy) + bottom * wy
    return values, valid


def zero_mean_ncc(reference: np.ndarray, target: np.ndarray,
                  valid: np.ndarray | None = None) -> float | None:
    """Return zero-mean NCC, or None for too few/textureless samples."""
    left = np.asarray(reference, dtype=np.float64).reshape(-1)
    right = np.asarray(target, dtype=np.float64).reshape(-1)
    if left.shape != right.shape:
        raise ValueError('reference and target patches must have equal shape')
    keep = np.isfinite(left) & np.isfinite(right)
    if valid is not None:
        mask = np.asarray(valid, dtype=bool).reshape(-1)
        if mask.shape != keep.shape:
            raise ValueError('valid mask must match patches')
        keep &= mask
    if keep.sum() < 4:
        return None
    left = left[keep] - left[keep].mean()
    right = right[keep] - right[keep].mean()
    denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denominator <= 1.0e-9:
        return None
    return float(np.clip(left @ right / denominator, -1.0, 1.0))


def project_point(K: np.ndarray, world_to_camera: np.ndarray,
                  point_world: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Project one world point, returning camera XYZ and pixel coordinates."""
    intr = np.asarray(K, dtype=np.float64)
    transform = np.asarray(world_to_camera, dtype=np.float64)
    point = np.asarray(point_world, dtype=np.float64).reshape(3)
    camera = transform[:3, :3] @ point + transform[:3, 3]
    if camera[2] <= 1.0e-9:
        raise ValueError('point is behind the camera')
    pixel_h = intr @ camera
    return camera, pixel_h[:2] / pixel_h[2]


def select_reference_patch(images: list[np.ndarray], K: np.ndarray,
                           world_to_cameras: np.ndarray,
                           point_world: np.ndarray,
                           normal_world: np.ndarray, *, radius: int = 4,
                           angle_weight: float = 0.25,
                           min_ncc: float = -1.0) -> tuple[int | None, np.ndarray]:
    """Select the most consistent, front-facing plane-warped reference patch.

    Each candidate patch is warped into every other observation with the local
    plane homography before zero-mean NCC is evaluated. The mean NCC rejects
    dynamic/blurred observations; a smaller front-facing term preserves detail,
    matching the two terms of FAST-LIVO2 equation (12).
    """
    views = np.asarray(world_to_cameras, dtype=np.float64)
    if len(images) != len(views) or views.ndim != 3 or views.shape[1:] != (4, 4):
        raise ValueError('images and world_to_cameras must have matching views')
    if radius < 1:
        raise ValueError('radius must be >= 1')
    if not 0.0 <= angle_weight <= 1.0:
        raise ValueError('angle_weight must be in [0, 1]')
    normal = np.asarray(normal_world, dtype=np.float64).reshape(3)
    if np.linalg.norm(normal) <= 1.0e-12:
        raise ValueError('normal_world must be non-zero')
    normal /= np.linalg.norm(normal)

    camera_points = []
    centres = []
    usable = []
    for index, view in enumerate(views):
        try:
            camera_point, centre = project_point(K, view, point_world)
        except ValueError:
            continue
        h, w = np.asarray(images[index]).shape[:2]
        if (centre[0] < radius or centre[1] < radius or
                centre[0] > w - 1 - radius or centre[1] > h - 1 - radius):
            continue
        camera_points.append(camera_point)
        centres.append(centre)
        usable.append(index)
    scores = np.full(len(images), -np.inf, dtype=np.float64)
    if len(usable) < 2:
        return None, scores

    offsets = np.array([(dx, dy) for dy in range(-radius, radius + 1)
                        for dx in range(-radius, radius + 1)], dtype=np.float64)
    for local_ref, ref_index in enumerate(usable):
        reference_pixels = centres[local_ref] + offsets
        reference_patch, reference_valid = bilinear_sample(
            np.asarray(images[ref_index]), reference_pixels)
        ref_view = views[ref_index]
        ref_camera_point = camera_points[local_ref]
        ref_normal = ref_view[:3, :3] @ normal
        ncc_values = []
        for local_target, target_index in enumerate(usable):
            if target_index == ref_index:
                continue
            target_T_reference = views[target_index] @ np.linalg.inv(ref_view)
            try:
                homography = plane_homography(
                    K, target_T_reference, ref_normal, ref_camera_point)
            except ValueError:
                continue
            target_pixels, warp_valid = warp_pixels(homography, reference_pixels)
            target_patch, sample_valid = bilinear_sample(
                np.asarray(images[target_index]), target_pixels)
            ncc = zero_mean_ncc(
                reference_patch, target_patch,
                reference_valid & warp_valid & sample_valid)
            if ncc is not None and ncc >= min_ncc:
                ncc_values.append(ncc)
        if not ncc_values:
            continue
        view_cosine = abs(float(ref_normal @ ref_camera_point)) / max(
            np.linalg.norm(ref_camera_point), 1.0e-12)
        scores[ref_index] = ((1.0 - angle_weight) * np.mean(ncc_values) +
                             angle_weight * view_cosine)
    if not np.isfinite(scores).any():
        return None, scores
    return int(np.argmax(scores)), scores


def select_planar_voxel_references(points: np.ndarray, images: list[np.ndarray],
                                   K: np.ndarray, world_to_cameras: np.ndarray, *,
                                   voxel_size: float = 1.0, min_points: int = 10,
                                   max_views: int = 6,
                                   max_planarity_ratio: float = 0.06,
                                   min_tangent_ratio: float = 0.04,
                                   patch_radius: int = 4,
                                   return_view_mask: bool = False,
                                   score_margin: float = 0.12
                                   ) -> np.ndarray | tuple[np.ndarray, np.ndarray]:
    """Return one reference-view index per point, shared by planar voxels.

    Candidate views are restricted to the nearest in-frame cameras before the
    plane-warped NCC update. Non-planar voxels and voxels without two valid
    observations retain ``-1`` for a downstream robust-colour fallback.
    """
    xyz = np.asarray(points, dtype=np.float64)
    views = np.asarray(world_to_cameras, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f'points must be Nx3, got {xyz.shape}')
    if len(images) != len(views):
        raise ValueError('images and world_to_cameras must match')
    if voxel_size <= 0.0 or min_points < 3 or max_views < 2:
        raise ValueError('invalid voxel_size, min_points, or max_views')
    references = np.full(len(xyz), -1, dtype=np.int32)
    view_mask = np.ones((len(xyz), len(views)), dtype=bool)
    if not len(xyz):
        return (references, view_mask) if return_view_mask else references
    if score_margin < 0.0:
        raise ValueError('score_margin must be >= 0')
    camera_centres = np.stack([
        np.linalg.inv(view)[:3, 3] for view in views])
    keys = np.floor(xyz / voxel_size).astype(np.int64)
    order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
    ordered = keys[order]
    changes = np.flatnonzero(np.any(ordered[1:] != ordered[:-1], axis=1)) + 1
    bounds = np.concatenate(([0], changes, [len(xyz)]))
    for begin, end in zip(bounds[:-1], bounds[1:]):
        ids = order[begin:end]
        if ids.size < min_points:
            continue
        block = xyz[ids]
        centre = block.mean(axis=0)
        centred = block - centre
        values, vectors = np.linalg.eigh(centred.T @ centred / ids.size)
        total = max(float(values.sum()), 1.0e-12)
        if values[0] / total > max_planarity_ratio:
            continue
        if values[1] / max(float(values[2]), 1.0e-12) < min_tangent_ratio:
            continue
        candidates = []
        for view_index, view in enumerate(views):
            try:
                _, pixel = project_point(K, view, centre)
            except ValueError:
                continue
            height, width = np.asarray(images[view_index]).shape[:2]
            if (patch_radius <= pixel[0] <= width - 1 - patch_radius and
                    patch_radius <= pixel[1] <= height - 1 - patch_radius):
                distance = np.linalg.norm(camera_centres[view_index] - centre)
                candidates.append((distance, view_index))
        candidates.sort()
        chosen_views = [index for _, index in candidates[:max_views]]
        if len(chosen_views) < 2:
            continue
        local, scores = select_reference_patch(
            [images[index] for index in chosen_views], K, views[chosen_views],
            centre, vectors[:, 0], radius=patch_radius)
        if local is not None:
            references[ids] = chosen_views[local]
            if return_view_mask:
                compatible = np.isfinite(scores) & (
                    scores >= scores[local] - score_margin)
                if compatible.sum() >= 2:
                    view_mask[ids] = False
                    view_mask[np.ix_(ids, np.asarray(chosen_views)[compatible])] = True
    return (references, view_mask) if return_view_mask else references


def apply_reference_colours(points: np.ndarray, images: list[np.ndarray],
                            K: np.ndarray, world_to_cameras: np.ndarray,
                            reference_indices: np.ndarray,
                            fallback_rgb: np.ndarray,
                            depth_tolerance: float = 0.15
                            ) -> tuple[np.ndarray, np.ndarray]:
    """Colour selected planar points from their reference view with z-buffering."""
    xyz = np.asarray(points, dtype=np.float64)
    refs = np.asarray(reference_indices, dtype=np.int32)
    colours = np.asarray(fallback_rgb, dtype=np.uint8).copy()
    if colours.shape != (len(xyz), 3) or refs.shape != (len(xyz),):
        raise ValueError('fallback_rgb/ref indices must match points')
    updated = np.zeros(len(xyz), dtype=bool)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    for view_index in np.unique(refs[refs >= 0]):
        image = np.asarray(images[view_index])
        if image.ndim == 2:
            image = np.repeat(image[:, :, None], 3, axis=2)
        view = np.asarray(world_to_cameras[view_index], dtype=np.float64)
        camera = xyz @ view[:3, :3].T + view[:3, 3]
        depth = camera[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            u = fx * camera[:, 0] / depth + cx
            v = fy * camera[:, 1] / depth + cy
        safe_u = np.nan_to_num(u, nan=-1.0, posinf=-1.0, neginf=-1.0)
        safe_v = np.nan_to_num(v, nan=-1.0, posinf=-1.0, neginf=-1.0)
        ui = np.round(safe_u).astype(np.int64)
        vi = np.round(safe_v).astype(np.int64)
        height, width = image.shape[:2]
        in_frame = ((depth > 1.0e-6) & (ui >= 0) & (ui < width) &
                    (vi >= 0) & (vi < height))
        zbuffer = np.full(height * width, np.inf, dtype=np.float64)
        pixels = vi[in_frame] * width + ui[in_frame]
        np.minimum.at(zbuffer, pixels, depth[in_frame])
        chosen = np.flatnonzero((refs == view_index) & in_frame)
        visible = depth[chosen] <= (
            zbuffer[vi[chosen] * width + ui[chosen]] + depth_tolerance +
            0.02 * depth[chosen])
        chosen = chosen[visible]
        colours[chosen] = np.clip(
            image[vi[chosen], ui[chosen], :3], 0, 255).astype(np.uint8)
        updated[chosen] = True
    return colours, updated
