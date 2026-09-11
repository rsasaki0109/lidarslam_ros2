#!/usr/bin/env python3
"""Run a sensor-only metric visual-motion feasibility diagnostic.

The diagnostic estimates a metric camera translation from tracked image
motion and sparse LiDAR depth.  It never reads a trajectory, reference pose,
or ground-truth file.  The result is a feasibility screen for a later
one-dimensional weak-axis velocity observation; it is not a SLAM score and it
does not modify any input bag.

The input may be one ROS1 bag, one ROS2 bag directory, or several bags whose
selected topics together form one timestamp-ordered sensor stream.  Camera
and LiDAR messages are matched by record time.  The LiDAR points are used
only as metric depth for the image-derived motion solve; no LiDAR registration
residual is used.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np
from rosbags.highlevel import AnyReader


POINT_FORMATS = {
    1: 'i1', 2: 'u1', 3: '<i2', 4: '<u2',
    5: '<i4', 6: '<u4', 7: '<f4', 8: '<f8',
}


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str
    distortion: tuple[float, float, float, float]

    @property
    def matrix(self) -> np.ndarray:
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)


@dataclass(frozen=True)
class PairResult:
    valid: bool
    reason: str
    features: int = 0
    tracks: int = 0
    inliers: int = 0
    scale_m: float = 0.0
    residual_norm: float = float('inf')
    dt_sec: float = 0.0
    direction_base: tuple[float, float, float] | None = None
    speed_mps: float = 0.0


def quaternion_xyzw_rotation(values: Iterable[float]) -> np.ndarray:
    """Convert a quaternion in xyzw order into a rotation matrix."""
    q = np.asarray(tuple(values), dtype=np.float64)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError('quaternion must contain four finite values')
    norm = float(np.linalg.norm(q))
    if norm <= 1.0e-12:
        raise ValueError('quaternion norm must be positive')
    x, y, z, w = q / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


def project_points(
        points: np.ndarray, model: CameraModel,
        rotation: np.ndarray | None = None,
        translation: np.ndarray | None = None,
        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Project 3D points and return pixels, camera points, and validity."""
    source = np.asarray(points, dtype=np.float64)
    if source.ndim != 2 or source.shape[1] != 3:
        raise ValueError('points must have shape Nx3')
    transformed = source if rotation is None else (rotation @ source.T).T
    if translation is not None:
        transformed = transformed + np.asarray(translation, dtype=np.float64)
    valid = np.all(np.isfinite(transformed), axis=1) & (transformed[:, 2] > 0.1)
    uv = np.full((len(source), 2), np.nan, dtype=np.float64)
    if not np.any(valid):
        return uv, transformed, valid
    xyz = transformed[valid]
    x = xyz[:, 0] / xyz[:, 2]
    y = xyz[:, 1] / xyz[:, 2]
    d = np.asarray(model.distortion, dtype=np.float64)
    if model.distortion_model == 'plumb_bob':
        radius2 = x * x + y * y
        radial = 1.0 + d[0] * radius2 + d[1] * radius2 * radius2
        xd = x * radial + 2.0 * d[2] * x * y + d[3] * (radius2 + 2.0 * x * x)
        yd = y * radial + d[2] * (radius2 + 2.0 * y * y) + 2.0 * d[3] * x * y
    elif model.distortion_model == 'equidistant':
        radius = np.hypot(x, y)
        theta = np.arctan(radius)
        theta2 = theta * theta
        distorted = theta * (
            1.0 + d[0] * theta2 + d[1] * theta2 * theta2 +
            d[2] * theta2 * theta2 * theta2 +
            d[3] * theta2 * theta2 * theta2 * theta2)
        scale = np.ones_like(radius)
        nonzero = radius > 1.0e-12
        scale[nonzero] = distorted[nonzero] / radius[nonzero]
        xd, yd = x * scale, y * scale
    else:
        raise ValueError(f'unsupported distortion model: {model.distortion_model}')
    projected = np.column_stack((model.fx * xd + model.cx,
                                 model.fy * yd + model.cy))
    uv[valid] = projected
    valid &= (
        np.isfinite(uv).all(axis=1) &
        (uv[:, 0] >= 2.0) & (uv[:, 0] < model.width - 2.0) &
        (uv[:, 1] >= 2.0) & (uv[:, 1] < model.height - 2.0))
    return uv, transformed, valid


def undistort_pixels(pixels: np.ndarray, model: CameraModel) -> np.ndarray:
    """Return normalized, undistorted image coordinates."""
    matrix = model.matrix
    distortion = np.asarray(model.distortion, dtype=np.float64)
    shaped = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    if model.distortion_model == 'equidistant':
        result = cv2.fisheye.undistortPoints(shaped, matrix, distortion)
    else:
        result = cv2.undistortPoints(shaped, matrix, distortion)
    return result.reshape(-1, 2)


def decode_image(message: Any) -> np.ndarray:
    """Decode a ROS Image or CompressedImage into packed grayscale pixels."""
    if hasattr(message, 'format'):
        encoded = np.asarray(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
        if image is None or image.ndim != 2:
            raise ValueError('compressed image could not be decoded')
        return np.ascontiguousarray(image)

    width = int(message.width)
    height = int(message.height)
    step = int(message.step)
    encoding = str(message.encoding).lower()
    channels = 1 if encoding in {'mono8', '8uc1'} else 3
    if encoding not in {'mono8', '8uc1', 'rgb8', 'bgr8'}:
        raise ValueError(f'unsupported image encoding: {message.encoding}')
    raw = np.asarray(message.data, dtype=np.uint8)
    if len(raw) < step * height:
        raise ValueError('image payload is shorter than its declared step')
    rows = np.ndarray(
        (height, width, channels), dtype=np.uint8,
        buffer=raw.tobytes(), strides=(step, channels, 1))
    if channels == 1:
        return np.ascontiguousarray(rows[:, :, 0])
    if encoding == 'rgb8':
        return np.ascontiguousarray(cv2.cvtColor(rows, cv2.COLOR_RGB2GRAY))
    return np.ascontiguousarray(cv2.cvtColor(rows, cv2.COLOR_BGR2GRAY))


def pointcloud_xyz(message: Any, max_points: int) -> np.ndarray:
    """Read finite XYZ points from a PointCloud2 without changing the bag."""
    fields = {str(field.name): field for field in message.fields}
    if not {'x', 'y', 'z'}.issubset(fields):
        raise ValueError('PointCloud2 lacks x/y/z fields')
    endian = '>' if bool(message.is_bigendian) else '<'
    names = ['x', 'y', 'z']
    formats = []
    offsets = []
    for name in names:
        field = fields[name]
        if int(field.count) != 1 or int(field.datatype) not in POINT_FORMATS:
            raise ValueError(f'unsupported XYZ field layout: {name}')
        fmt = POINT_FORMATS[int(field.datatype)]
        if fmt[-1].isdigit() or fmt[0] in '<>':
            fmt = fmt.lstrip('<>')
        formats.append(endian + fmt)
        offsets.append(int(field.offset))
    dtype = np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': int(message.point_step),
    })
    raw = np.asarray(message.data, dtype=np.uint8)
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step)
    rows = []
    payload = raw.tobytes()
    for row in range(height):
        start = row * row_step
        row_payload = payload[start:start + width * point_step]
        if len(row_payload) < width * point_step:
            break
        rows.append(np.frombuffer(row_payload, dtype=dtype, count=width))
    if not rows:
        return np.empty((0, 3), dtype=np.float64)
    structured = np.concatenate(rows)
    points = np.column_stack([
        structured[name].astype(np.float64, copy=False) for name in names])
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    points = points[np.linalg.norm(points, axis=1) > 0.2]
    if len(points) > max_points:
        indices = np.linspace(0, len(points) - 1, max_points, dtype=np.int64)
        points = points[indices]
    return np.ascontiguousarray(points)


def select_projected_points(
        image: np.ndarray, points: np.ndarray, model: CameraModel,
        camera_rotation: np.ndarray, camera_translation: np.ndarray,
        max_features: int, grid_cell_size: int,
        ) -> tuple[np.ndarray, np.ndarray]:
    """Keep high-gradient projected LiDAR points with deterministic spacing."""
    pixels, camera_points, valid = project_points(
        points, model, camera_rotation, camera_translation)
    indices = np.flatnonzero(valid)
    if not len(indices):
        return np.empty((0, 2)), np.empty((0, 3))
    xy = np.rint(pixels[indices]).astype(np.int32)
    gx = cv2.Sobel(image, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(image, cv2.CV_32F, 0, 1, ksize=3)
    score = gx[xy[:, 1], xy[:, 0]] ** 2 + gy[xy[:, 1], xy[:, 0]] ** 2
    order = np.argsort(-score, kind='stable')
    selected: list[int] = []
    cells: set[tuple[int, int]] = set()
    for ordered in order:
        pixel = pixels[indices[ordered]]
        cell = (int(pixel[0]) // grid_cell_size,
                int(pixel[1]) // grid_cell_size)
        if cell in cells:
            continue
        cells.add(cell)
        selected.append(int(indices[ordered]))
        if len(selected) >= max_features:
            break
    selected_indices = np.asarray(selected, dtype=np.int64)
    return pixels[selected_indices], camera_points[selected_indices]


def track_image_points(
        previous: np.ndarray, current: np.ndarray,
        pixels: np.ndarray, points: np.ndarray,
        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Track projected depth points with a forward/backward LK check."""
    if len(pixels) < 8:
        return (np.empty((0, 2)), np.empty((0, 2)), np.empty((0, 3)))
    source = pixels.astype(np.float32).reshape(-1, 1, 2)
    tracked, status, _ = cv2.calcOpticalFlowPyrLK(
        previous, current, source, None, winSize=(21, 21), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    if tracked is None or status is None:
        return (np.empty((0, 2)), np.empty((0, 2)), np.empty((0, 3)))
    backward, backward_status, _ = cv2.calcOpticalFlowPyrLK(
        current, previous, tracked, None, winSize=(21, 21), maxLevel=3,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    if backward is None or backward_status is None:
        return (np.empty((0, 2)), np.empty((0, 2)), np.empty((0, 3)))
    first = source[:, 0, :].astype(np.float64)
    second = tracked[:, 0, :].astype(np.float64)
    reverse = backward[:, 0, :].astype(np.float64)
    keep = (
        (status[:, 0] > 0) & (backward_status[:, 0] > 0) &
        np.isfinite(second).all(axis=1) &
        (np.linalg.norm(reverse - first, axis=1) <= 1.5) &
        (second[:, 0] >= 2.0) & (second[:, 0] < current.shape[1] - 2.0) &
        (second[:, 1] >= 2.0) & (second[:, 1] < current.shape[0] - 2.0))
    return first[keep], second[keep], points[keep]


def essential_candidates(matrix: np.ndarray) -> list[np.ndarray]:
    """Split OpenCV's stacked essential-matrix output deterministically."""
    if matrix.shape == (3, 3):
        return [matrix]
    if matrix.shape[1] == 3 and matrix.shape[0] % 3 == 0:
        return [matrix[row:row + 3] for row in range(0, matrix.shape[0], 3)]
    if matrix.shape[0] == 3 and matrix.shape[1] % 3 == 0:
        return [matrix[:, col:col + 3] for col in range(0, matrix.shape[1], 3)]
    return []


def _scale_cost(
        scale: float, direction: np.ndarray, rotation: np.ndarray,
        points: np.ndarray, observations: np.ndarray,
        ) -> tuple[float, np.ndarray]:
    """Return robust normalized reprojection cost and residuals."""
    transformed = (rotation @ points.T).T + scale * direction
    valid = transformed[:, 2] > 0.1
    predicted = np.full_like(observations, np.nan)
    predicted[valid] = transformed[valid, :2] / transformed[valid, 2:3]
    residual = observations - predicted
    norms = np.linalg.norm(residual, axis=1)
    finite = valid & np.isfinite(norms)
    if not np.any(finite):
        return float('inf'), np.empty(0)
    clipped = np.minimum(norms[finite], 0.02)
    return float(np.mean(clipped * clipped)), residual[finite]


def solve_metric_scale(
        points: np.ndarray, observations: np.ndarray,
        rotation: np.ndarray, translation_direction: np.ndarray,
        max_scale_m: float = 4.0,
        ) -> tuple[float, float, np.ndarray] | None:
    """Solve translation scale from known depth and image correspondences.

    ``translation_direction`` follows OpenCV's point-transform convention:
    ``P_current = R @ P_previous + scale * direction``.  The returned motion
    direction is the camera-centre displacement direction.
    """
    directions = []
    normalized = np.asarray(translation_direction, dtype=np.float64)
    normalized /= max(float(np.linalg.norm(normalized)), 1.0e-12)
    for sign in (1.0, -1.0):
        directions.append(sign * normalized)
    transformed = (rotation @ np.asarray(points, dtype=np.float64).T).T
    observed = np.asarray(observations, dtype=np.float64)
    finite = (
        np.isfinite(transformed).all(axis=1) &
        np.isfinite(observed).all(axis=1) &
        (transformed[:, 2] > 0.1))
    if not np.any(finite):
        return None
    transformed = transformed[finite]
    observed = observed[finite]
    best: tuple[float, float, np.ndarray] | None = None
    for direction in directions:
        # For normalized observations, the translation scale is a one-dimensional
        # projective equation.  Use a coarse global search to avoid local minima
        # caused by the robust reprojection loss, then refine it with IRLS.
        candidates = np.linspace(0.0, max_scale_m, 129)
        costs = [
            _scale_cost(float(scale), direction, rotation, points, observations)[0]
            for scale in candidates]
        coarse_scale = float(candidates[int(np.argmin(costs))])
        x0, y0, z0 = transformed.T
        u, v = observed.T
        design = np.column_stack((
            direction[0] - u * direction[2],
            direction[1] - v * direction[2]))
        target = np.column_stack((u * z0 - x0, v * z0 - y0))
        scale = coarse_scale
        for _ in range(8):
            equation_residual = scale * design - target
            norms = np.linalg.norm(equation_residual, axis=1)
            weights = np.where(norms <= 0.01, 1.0, 0.01 / np.maximum(norms, 1.0e-12))
            row_weights = np.repeat(weights, 2)
            flattened_design = design.reshape(-1)
            flattened_target = target.reshape(-1)
            denominator = float(np.sum(row_weights * flattened_design ** 2))
            numerator = float(np.sum(
                row_weights * flattened_design * flattened_target))
            if denominator <= 1.0e-12:
                break
            updated = float(np.clip(numerator / denominator, 0.0, max_scale_m))
            if abs(updated - scale) <= 1.0e-6:
                scale = updated
                break
            scale = updated
        for candidate_scale in (coarse_scale, scale):
            cost, residual = _scale_cost(
                candidate_scale, direction, rotation, points, observations)
            if len(residual):
                candidate = (cost, candidate_scale, direction)
                if best is None or candidate[0] < best[0]:
                    best = candidate
    if best is None:
        return None
    cost, scale, direction = best
    return scale, float(np.sqrt(cost)), direction


def estimate_pair(
        previous_image: np.ndarray, previous_points: np.ndarray,
        current_image: np.ndarray, model: CameraModel,
        lidar_to_camera_rotation: np.ndarray,
        camera_to_base_rotation: np.ndarray,
        camera_translation: np.ndarray,
        dt_sec: float, config: dict[str, float | int],
        ) -> PairResult:
    """Estimate one metric image-motion pair."""
    pixels, points = select_projected_points(
        previous_image, previous_points, model, lidar_to_camera_rotation,
        camera_translation, int(config['max_features']),
        int(config['grid_cell_size']))
    features = len(pixels)
    if features < int(config['min_features']):
        return PairResult(False, 'insufficient_projected_features', features=features,
                          dt_sec=dt_sec)
    first, second, tracked_points = track_image_points(
        previous_image, current_image, pixels, points)
    tracks = len(first)
    if tracks < int(config['min_tracks']):
        return PairResult(False, 'insufficient_tracks', features=features,
                          tracks=tracks, dt_sec=dt_sec)
    previous_normalized = undistort_pixels(first, model)
    current_normalized = undistort_pixels(second, model)
    threshold = float(config['ransac_threshold_px']) / max(model.fx, model.fy)
    essential, mask = cv2.findEssentialMat(
        previous_normalized, current_normalized, np.eye(3), cv2.RANSAC,
        0.999, threshold)
    if essential is None or mask is None:
        return PairResult(False, 'essential_failed', features=features,
                          tracks=tracks, dt_sec=dt_sec)
    best = None
    for candidate in essential_candidates(np.asarray(essential, dtype=np.float64)):
        try:
            inliers, rotation, translation, pose_mask = cv2.recoverPose(
                candidate, previous_normalized, current_normalized, np.eye(3),
                mask=np.asarray(mask, dtype=np.uint8).copy())
        except cv2.error:
            continue
        if best is None or int(inliers) > best[0]:
            best = (int(inliers), rotation, translation[:, 0], pose_mask)
    if best is None:
        return PairResult(False, 'pose_recovery_failed', features=features,
                          tracks=tracks, dt_sec=dt_sec)
    inliers, rotation, direction, pose_mask = best
    inlier_mask = np.asarray(pose_mask, dtype=np.uint8).reshape(-1) > 0
    if inliers < int(config['min_inliers']):
        return PairResult(False, 'insufficient_pose_inliers', features=features,
                          tracks=tracks, inliers=inliers, dt_sec=dt_sec)
    solved = solve_metric_scale(
        tracked_points[inlier_mask], current_normalized[inlier_mask], rotation,
        direction, float(config['max_scale_m']))
    if solved is None:
        return PairResult(False, 'metric_scale_failed', features=features,
                          tracks=tracks, inliers=inliers, dt_sec=dt_sec)
    scale, residual, point_transform_direction = solved
    if residual > float(config['max_residual_norm']):
        return PairResult(False, 'metric_residual_too_large', features=features,
                          tracks=tracks, inliers=inliers, scale_m=scale,
                          residual_norm=residual, dt_sec=dt_sec)
    # OpenCV's t maps points from the previous camera into the current camera.
    # Camera-centre displacement is therefore -R.T @ t; then rotate into base.
    camera_displacement = -rotation.T @ (scale * point_transform_direction)
    base_displacement = camera_to_base_rotation @ camera_displacement
    norm = float(np.linalg.norm(base_displacement))
    if not np.isfinite(norm) or norm <= 1.0e-6 or dt_sec <= 1.0e-6:
        return PairResult(False, 'degenerate_metric_motion', features=features,
                          tracks=tracks, inliers=inliers, scale_m=scale,
                          residual_norm=residual, dt_sec=dt_sec)
    direction_base = base_displacement / norm
    return PairResult(
        True, 'ok', features=features, tracks=tracks, inliers=inliers,
        scale_m=scale, residual_norm=residual, dt_sec=dt_sec,
        direction_base=tuple(float(value) for value in direction_base),
        speed_mps=norm / dt_sec)


def summarize(results: list[PairResult], counters: Counter[str],
              source_paths: list[Path], camera_topic: str,
              lidar_topic: str, camera_count: int, lidar_count: int,
              config: dict[str, float | int],) -> dict[str, Any]:
    """Create the ground-truth-free feasibility report."""
    valid = [result for result in results if result.valid]
    directions = np.asarray([
        result.direction_base for result in valid if result.direction_base is not None
    ], dtype=np.float64)
    speeds = np.asarray([result.speed_mps for result in valid], dtype=np.float64)
    scales = np.asarray([result.scale_m for result in valid], dtype=np.float64)
    if len(directions):
        direction_coherence = float(np.linalg.norm(np.mean(directions, axis=0)))
        axis_median_abs = np.median(np.abs(directions), axis=0).tolist()
    else:
        direction_coherence = 0.0
        axis_median_abs = [0.0, 0.0, 0.0]
    max_streak = 0
    streak = 0
    for result in results:
        if result.valid:
            streak += 1
            max_streak = max(max_streak, streak)
        else:
            streak = 0
    attempted = len(results)
    valid_fraction = len(valid) / max(1, attempted)
    feasible = (
        len(valid) >= int(config['min_valid_pairs']) and
        max_streak >= int(config['min_valid_streak']) and
        valid_fraction >= float(config['min_valid_fraction']) and
        direction_coherence >= float(config['min_direction_coherence']))
    return {
        'schema_version': 1,
        'status': 'sensor_only_before_ground_truth_access',
        'accuracy_ground_truth_accessed': False,
        'operation': 'metric_camera_motion_from_lk_and_sparse_lidar_depth',
        'inputs': {
            'sources': [str(path.resolve()) for path in source_paths],
            'camera_topic': camera_topic,
            'lidar_topic': lidar_topic,
        },
        'message_counts': {
            'camera_records_seen': camera_count,
            'lidar_records_seen': lidar_count,
            'pairs_attempted': attempted,
        },
        'pair_counts': {
            'valid': len(valid),
            'valid_fraction': valid_fraction,
            'max_valid_streak': max_streak,
        },
        'quality': {
            'direction_coherence': direction_coherence,
            'median_abs_direction_base_xyz': axis_median_abs,
            'median_speed_mps': float(np.median(speeds)) if len(speeds) else None,
            'p10_speed_mps': float(np.percentile(speeds, 10)) if len(speeds) else None,
            'p90_speed_mps': float(np.percentile(speeds, 90)) if len(speeds) else None,
            'median_metric_scale_m': float(np.median(scales)) if len(scales) else None,
            'median_inliers': float(np.median([r.inliers for r in valid])) if valid else None,
            'median_residual_norm': float(np.median([r.residual_norm for r in valid])) if valid else None,
        },
        'rejections': dict(sorted(counters.items())),
        'config': config,
        'decision': 'GO_SENSOR_MOTION' if feasible else 'NO_GO_SENSOR_MOTION',
        'decision_checks': {
            'min_valid_pairs': len(valid) >= int(config['min_valid_pairs']),
            'min_valid_streak': max_streak >= int(config['min_valid_streak']),
            'min_valid_fraction': valid_fraction >= float(config['min_valid_fraction']),
            'min_direction_coherence': direction_coherence >= float(config['min_direction_coherence']),
        },
    }


def compose_sensor_transforms(
        camera_to_base_rotation: np.ndarray,
        camera_to_base_translation: np.ndarray,
        lidar_to_base_rotation: np.ndarray,
        lidar_to_base_translation: np.ndarray,
        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return LiDAR-to-camera rotation, camera-to-base rotation, and offset."""
    camera_rotation = np.asarray(camera_to_base_rotation, dtype=np.float64)
    camera_translation = np.asarray(
        camera_to_base_translation, dtype=np.float64)
    lidar_rotation = np.asarray(lidar_to_base_rotation, dtype=np.float64)
    lidar_translation = np.asarray(lidar_to_base_translation, dtype=np.float64)
    lidar_to_camera_rotation = camera_rotation.T @ lidar_rotation
    lidar_to_camera_translation = camera_rotation.T @ (
        lidar_translation - camera_translation)
    return (lidar_to_camera_rotation, camera_rotation,
            lidar_to_camera_translation)


def run_diagnostic(
        sources: list[Path], camera_topic: str, lidar_topic: str,
        model: CameraModel, lidar_to_camera_rotation: np.ndarray,
        camera_to_base_rotation: np.ndarray,
        camera_translation: np.ndarray, stride: int, max_lidar_points: int,
        max_sync_sec: float, max_pairs: int, config: dict[str, float | int],
        include_pair_records: bool = False,
        ) -> dict[str, Any]:
    """Stream selected messages and estimate sparse metric image motion."""
    counters: Counter[str] = Counter()
    results: list[PairResult] = []
    camera_count = 0
    lidar_count = 0
    selected_camera_count = 0
    latest_lidar: tuple[int, Any, bytes] | None = None
    latest_lidar_points: tuple[int, np.ndarray] | None = None
    previous: tuple[int, np.ndarray, np.ndarray] | None = None
    selected_lidar_types: set[str] = set()
    pair_records: list[dict[str, Any]] = []

    with AnyReader(sources) as reader:
        connections = [
            connection for connection in reader.connections
            if connection.topic in {camera_topic, lidar_topic}
        ]
        if not any(connection.topic == camera_topic for connection in connections):
            raise ValueError(f'camera topic is absent: {camera_topic}')
        if not any(connection.topic == lidar_topic for connection in connections):
            raise ValueError(f'LiDAR topic is absent: {lidar_topic}')
        for connection, timestamp_ns, raw in reader.messages(connections=connections):
            if connection.topic == lidar_topic:
                lidar_count += 1
                latest_lidar = (timestamp_ns, connection.msgtype, raw)
                latest_lidar_points = None
                selected_lidar_types.add(connection.msgtype)
                continue

            camera_count += 1
            if camera_count % stride != 0:
                continue
            selected_camera_count += 1
            if latest_lidar is None:
                counters['no_previous_lidar'] += 1
                continue
            lidar_timestamp, _, lidar_raw = latest_lidar
            delta_sec = (timestamp_ns - lidar_timestamp) * 1.0e-9
            if delta_sec < -max_sync_sec or delta_sec > max_sync_sec:
                counters['camera_lidar_time_mismatch'] += 1
                continue
            if latest_lidar_points is None or latest_lidar_points[0] != lidar_timestamp:
                lidar_connection = next(
                    candidate for candidate in connections
                    if candidate.topic == lidar_topic and
                    candidate.msgtype == latest_lidar[1])
                message = reader.deserialize(lidar_raw, lidar_connection.msgtype)
                latest_lidar_points = (
                    lidar_timestamp, pointcloud_xyz(message, max_lidar_points))
            current_points = latest_lidar_points[1]
            try:
                message = reader.deserialize(raw, connection.msgtype)
                current_image = decode_image(message)
            except (ValueError, cv2.error) as error:
                counters[f'image_decode:{type(error).__name__}'] += 1
                continue
            if current_image.shape != (model.height, model.width):
                counters['unexpected_image_shape'] += 1
                continue
            if previous is not None:
                previous_timestamp, previous_image, previous_points = previous
                result = estimate_pair(
                    previous_image, previous_points, current_image, model,
                    lidar_to_camera_rotation, camera_to_base_rotation,
                    camera_translation,
                    (timestamp_ns - previous_timestamp) * 1.0e-9, config)
                results.append(result)
                if include_pair_records:
                    record = asdict(result)
                    record.update({
                        'camera_timestamp_sec': timestamp_ns * 1.0e-9,
                        'lidar_timestamp_sec': lidar_timestamp * 1.0e-9,
                        'camera_lidar_delta_sec': delta_sec,
                    })
                    pair_records.append(record)
                if not result.valid:
                    counters[result.reason] += 1
                if max_pairs > 0 and len(results) >= max_pairs:
                    break
            previous = (timestamp_ns, current_image, current_points)

    report = summarize(
        results, counters, sources, camera_topic, lidar_topic,
        camera_count, lidar_count, config)
    report['message_counts']['selected_camera_records'] = selected_camera_count
    report['message_counts']['lidar_message_types'] = sorted(selected_lidar_types)
    if include_pair_records:
        report['pair_records'] = pair_records
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', action='append', required=True, type=Path)
    parser.add_argument('--camera-topic', required=True)
    parser.add_argument('--lidar-topic', required=True)
    parser.add_argument('--width', required=True, type=int)
    parser.add_argument('--height', required=True, type=int)
    parser.add_argument('--fx', required=True, type=float)
    parser.add_argument('--fy', required=True, type=float)
    parser.add_argument('--cx', required=True, type=float)
    parser.add_argument('--cy', required=True, type=float)
    parser.add_argument('--distortion-model', choices=('plumb_bob', 'equidistant'),
                        default='plumb_bob')
    parser.add_argument('--distortion', nargs=4, type=float, default=(0.0, 0.0, 0.0, 0.0))
    parser.add_argument('--camera-to-base-quaternion-xyzw', nargs=4, required=True, type=float)
    parser.add_argument('--camera-to-base-translation', nargs=3, required=True, type=float)
    parser.add_argument('--lidar-to-base-quaternion-xyzw', nargs=4, required=True, type=float)
    parser.add_argument('--lidar-to-base-translation', nargs=3, required=True, type=float)
    parser.add_argument('--stride', type=int, default=10)
    parser.add_argument('--max-lidar-points', type=int, default=2000)
    parser.add_argument('--max-sync-sec', type=float, default=0.08)
    parser.add_argument('--max-pairs', type=int, default=0)
    parser.add_argument('--max-features', type=int, default=500)
    parser.add_argument('--grid-cell-size', type=int, default=24)
    parser.add_argument('--min-features', type=int, default=30)
    parser.add_argument('--min-tracks', type=int, default=20)
    parser.add_argument('--min-inliers', type=int, default=15)
    parser.add_argument('--ransac-threshold-px', type=float, default=1.5)
    parser.add_argument('--max-scale-m', type=float, default=4.0)
    parser.add_argument('--max-residual-norm', type=float, default=0.02)
    parser.add_argument('--min-valid-pairs', type=int, default=20)
    parser.add_argument('--min-valid-streak', type=int, default=5)
    parser.add_argument('--min-valid-fraction', type=float, default=0.05)
    parser.add_argument('--min-direction-coherence', type=float, default=0.35)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--pair-output', type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.stride < 1 or args.max_lidar_points < 20:
        raise ValueError('stride must be positive and max-lidar-points >= 20')
    camera_to_base_rotation = quaternion_xyzw_rotation(
        args.camera_to_base_quaternion_xyzw)
    lidar_to_base_rotation = quaternion_xyzw_rotation(
        args.lidar_to_base_quaternion_xyzw)
    camera_to_base_translation = np.asarray(args.camera_to_base_translation, dtype=np.float64)
    lidar_to_base_translation = np.asarray(args.lidar_to_base_translation, dtype=np.float64)
    lidar_to_camera_rotation, camera_to_base_rotation, camera_translation = (
        compose_sensor_transforms(
            camera_to_base_rotation, camera_to_base_translation,
            lidar_to_base_rotation, lidar_to_base_translation))
    config: dict[str, float | int] = {
        'stride': args.stride,
        'max_lidar_points': args.max_lidar_points,
        'max_sync_sec': args.max_sync_sec,
        'max_features': args.max_features,
        'grid_cell_size': args.grid_cell_size,
        'min_features': args.min_features,
        'min_tracks': args.min_tracks,
        'min_inliers': args.min_inliers,
        'ransac_threshold_px': args.ransac_threshold_px,
        'max_scale_m': args.max_scale_m,
        'max_residual_norm': args.max_residual_norm,
        'min_valid_pairs': args.min_valid_pairs,
        'min_valid_streak': args.min_valid_streak,
        'min_valid_fraction': args.min_valid_fraction,
        'min_direction_coherence': args.min_direction_coherence,
    }
    model = CameraModel(
        width=args.width, height=args.height, fx=args.fx, fy=args.fy,
        cx=args.cx, cy=args.cy, distortion_model=args.distortion_model,
        distortion=tuple(args.distortion))
    report = run_diagnostic(
        [path.resolve() for path in args.source], args.camera_topic,
        args.lidar_topic, model, lidar_to_camera_rotation,
        camera_to_base_rotation, camera_translation,
        args.stride, args.max_lidar_points, args.max_sync_sec,
        args.max_pairs, config, include_pair_records=args.pair_output is not None)
    pair_records = report.pop('pair_records', None)
    encoded = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output is not None:
        if args.output.exists():
            raise ValueError(f'refusing to overwrite: {args.output}')
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding='utf-8')
    if args.pair_output is not None:
        if args.pair_output.exists():
            raise ValueError(f'refusing to overwrite: {args.pair_output}')
        args.pair_output.parent.mkdir(parents=True, exist_ok=True)
        pair_document = {
            'schema_version': 1,
            'status': 'sensor_only_before_ground_truth_access',
            'accuracy_ground_truth_accessed': False,
            'inputs': report['inputs'],
            'config': report['config'],
            'pairs': pair_records or [],
        }
        args.pair_output.write_text(
            json.dumps(pair_document, indent=2, sort_keys=True) + '\n',
            encoding='utf-8')
    print(encoded, end='')
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, TypeError, KeyError, cv2.error) as error:
        print(f'error: {error}', file=__import__('sys').stderr)
        raise SystemExit(2)
