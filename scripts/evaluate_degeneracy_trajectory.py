#!/usr/bin/env python3
"""Compute accuracy-oriented, no-GT metrics for degeneracy trajectories."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np


def load_tum(path: Path) -> np.ndarray:
    trajectory = np.loadtxt(path, dtype=float)
    if trajectory.ndim == 1:
        trajectory = trajectory.reshape(1, -1)
    if trajectory.shape[1] < 8:
        raise ValueError(f"{path}: expected TUM rows with at least 8 columns")
    if len(trajectory) < 2:
        raise ValueError(f"{path}: expected at least two poses")
    if np.any(np.diff(trajectory[:, 0]) <= 0.0):
        raise ValueError(f"{path}: timestamps must be strictly increasing")
    return trajectory


def trajectory_metrics(
    path: Path,
    expected_endpoint_distance: float | None,
) -> tuple[np.ndarray, dict[str, Any]]:
    trajectory = load_tum(path)
    xyz = trajectory[:, 1:4]
    deltas = np.diff(xyz, axis=0)
    steps = np.linalg.norm(deltas, axis=1)
    centered = xyz - xyz.mean(axis=0)
    _, singular_values, principal_axes = np.linalg.svd(centered, full_matrices=False)
    along = centered @ principal_axes[0]
    transverse = centered - np.outer(along, principal_axes[0])
    transverse_norm = np.linalg.norm(transverse, axis=1)
    endpoint_distance = float(np.linalg.norm(xyz[-1] - xyz[0]))

    result: dict[str, Any] = {
        "trajectory": str(path.resolve()),
        "pose_count": int(len(trajectory)),
        "duration_sec": float(trajectory[-1, 0] - trajectory[0, 0]),
        "path_length_m": float(steps.sum()),
        "endpoint_distance_m": endpoint_distance,
        "max_step_m": float(steps.max()),
        "p99_step_m": float(np.quantile(steps, 0.99)),
        "principal_axis_span_m": float(along.max() - along.min()),
        "transverse_rms_m": float(np.sqrt(np.mean(np.square(transverse_norm)))),
        "transverse_max_m": float(transverse_norm.max()),
        "principal_variance_fraction": float(
            singular_values[0] ** 2 / np.square(singular_values).sum()
        ),
        "start_xyz_m": xyz[0].tolist(),
        "end_xyz_m": xyz[-1].tolist(),
    }
    if expected_endpoint_distance is not None:
        result["expected_endpoint_distance_m"] = expected_endpoint_distance
        result["endpoint_distance_error_m"] = endpoint_distance - expected_endpoint_distance
        result["endpoint_distance_error_percent"] = (
            100.0 * (endpoint_distance - expected_endpoint_distance)
            / expected_endpoint_distance
        )
    return trajectory, result


def _interpolate_reference(
    reference: np.ndarray,
    timestamps: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate a dense reference at candidate timestamps in the shared span."""
    in_range = (
        (timestamps >= reference[0, 0])
        & (timestamps <= reference[-1, 0])
    )
    matched_timestamps = timestamps[in_range]
    if len(matched_timestamps) < 3:
        raise ValueError("candidate and reference need at least three overlapping poses")
    xyz = np.column_stack(
        [
            np.interp(matched_timestamps, reference[:, 0], reference[:, axis])
            for axis in range(1, 4)
        ]
    )
    return in_range, xyz


def _rigid_alignment(
    reference_xyz: np.ndarray,
    candidate_xyz: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    reference_center = reference_xyz.mean(axis=0)
    candidate_center = candidate_xyz.mean(axis=0)
    covariance = (
        (candidate_xyz - candidate_center).T
        @ (reference_xyz - reference_center)
    )
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T
    translation = reference_center - rotation @ candidate_center
    return rotation, translation


def _interpolate_reference_quaternions(
    reference: np.ndarray,
    timestamps: np.ndarray,
) -> np.ndarray:
    """Slerp TUM quaternions at timestamps inside the reference span."""
    upper = np.searchsorted(reference[:, 0], timestamps, side="right")
    upper = np.clip(upper, 1, len(reference) - 1)
    lower = upper - 1
    interval = reference[upper, 0] - reference[lower, 0]
    alpha = (timestamps - reference[lower, 0]) / interval

    q0 = _normalize_quaternions(reference[lower, 4:8])
    q1 = _normalize_quaternions(reference[upper, 4:8])
    dot = np.sum(q0 * q1, axis=1)
    negative = dot < 0.0
    q1[negative] *= -1.0
    dot = np.clip(np.abs(dot), 0.0, 1.0)

    result = np.empty_like(q0)
    nearly_equal = dot > 0.9995
    result[nearly_equal] = (
        (1.0 - alpha[nearly_equal, None]) * q0[nearly_equal]
        + alpha[nearly_equal, None] * q1[nearly_equal]
    )
    separated = ~nearly_equal
    theta = np.arccos(dot[separated])
    sin_theta = np.sin(theta)
    result[separated] = (
        np.sin((1.0 - alpha[separated]) * theta)[:, None]
        / sin_theta[:, None]
        * q0[separated]
        + np.sin(alpha[separated] * theta)[:, None]
        / sin_theta[:, None]
        * q1[separated]
    )
    return _normalize_quaternions(result)


def _normalize_quaternions(quaternions: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(quaternions, axis=1, keepdims=True)
    if np.any(~np.isfinite(norms)) or np.any(norms <= 1.0e-12):
        raise ValueError("trajectory contains a non-finite or zero quaternion")
    return quaternions / norms


def _quaternion_rotation_matrices(quaternions: np.ndarray) -> np.ndarray:
    """Convert normalized TUM (x, y, z, w) quaternions to rotation matrices."""
    q = _normalize_quaternions(quaternions)
    x, y, z, w = q.T
    matrices = np.empty((len(q), 3, 3), dtype=float)
    matrices[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    matrices[:, 0, 1] = 2.0 * (x * y - z * w)
    matrices[:, 0, 2] = 2.0 * (x * z + y * w)
    matrices[:, 1, 0] = 2.0 * (x * y + z * w)
    matrices[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    matrices[:, 1, 2] = 2.0 * (y * z - x * w)
    matrices[:, 2, 0] = 2.0 * (x * z - y * w)
    matrices[:, 2, 1] = 2.0 * (y * z + x * w)
    matrices[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return matrices


def _point_projection_metrics(
    candidate_xyz_aligned: np.ndarray,
    candidate_rotations_aligned: np.ndarray,
    reference_xyz: np.ndarray,
    reference_rotations: np.ndarray,
) -> dict[str, Any]:
    """Measure world-placement error for representative sensor-frame points."""
    directions = np.vstack((np.eye(3), -np.eye(3)))
    by_range: dict[str, Any] = {}
    for point_range_m in (5.0, 10.0, 20.0):
        local_points = point_range_m * directions
        candidate_world = (
            np.einsum("nij,kj->nki", candidate_rotations_aligned, local_points)
            + candidate_xyz_aligned[:, None, :]
        )
        reference_world = (
            np.einsum("nij,kj->nki", reference_rotations, local_points)
            + reference_xyz[:, None, :]
        )
        errors = np.linalg.norm(candidate_world - reference_world, axis=2).ravel()
        by_range[f"{point_range_m:g}"] = {
            "rmse": float(math.sqrt(np.mean(np.square(errors)))),
            "mean": float(errors.mean()),
            "p95": float(np.quantile(errors, 0.95)),
            "max": float(errors.max()),
        }
    return {
        "method": "six_axis_sensor_points_after_se3_trajectory_alignment",
        "sample_directions": int(len(directions)),
        "ranges_m": by_range,
    }


def reference_metrics(
    candidate: np.ndarray,
    reference_path: Path,
    min_reference_reach_m: float,
) -> dict[str, Any]:
    reference = load_tum(reference_path)
    in_range, reference_xyz = _interpolate_reference(reference, candidate[:, 0])
    candidate_xyz = candidate[in_range, 1:4]

    candidate_reach = np.linalg.norm(
        candidate_xyz - candidate_xyz[0],
        axis=1,
    )
    reference_reach = np.linalg.norm(
        reference_xyz - reference_xyz[0],
        axis=1,
    )
    valid_ratio = reference_reach >= min_reference_reach_m
    if not np.any(valid_ratio):
        raise ValueError(
            "reference never reaches --min-reference-reach-m "
            f"({min_reference_reach_m}) in the overlapping span"
        )
    ratios = candidate_reach[valid_ratio] / reference_reach[valid_ratio]

    rotation, translation = _rigid_alignment(reference_xyz, candidate_xyz)
    candidate_aligned = (rotation @ candidate_xyz.T).T + translation
    aligned_errors = np.linalg.norm(candidate_aligned - reference_xyz, axis=1)
    matched_timestamps = candidate[in_range, 0]
    reference_quaternions = _interpolate_reference_quaternions(
        reference,
        matched_timestamps,
    )
    candidate_rotations = _quaternion_rotation_matrices(candidate[in_range, 4:8])
    reference_rotations = _quaternion_rotation_matrices(reference_quaternions)
    candidate_rotations_aligned = np.einsum(
        "ij,njk->nik",
        rotation,
        candidate_rotations,
    )

    return {
        "trajectory": str(reference_path.resolve()),
        "overlap_pose_count": int(len(candidate_xyz)),
        "overlap_duration_sec": float(
            candidate[in_range, 0][-1] - candidate[in_range, 0][0]
        ),
        "min_reach_for_ratio_m": min_reference_reach_m,
        "reach_ratio": {
            "sample_count": int(len(ratios)),
            "final": float(ratios[-1]),
            "p50": float(np.quantile(ratios, 0.50)),
            "p95": float(np.quantile(ratios, 0.95)),
            "max": float(ratios.max()),
        },
        "aligned_translation_delta_m": {
            "alignment": "se3_umeyama",
            "rmse": float(math.sqrt(np.mean(np.square(aligned_errors)))),
            "mean": float(aligned_errors.mean()),
            "p95": float(np.quantile(aligned_errors, 0.95)),
            "max": float(aligned_errors.max()),
        },
        "aligned_point_projection_delta_m": _point_projection_metrics(
            candidate_aligned,
            candidate_rotations_aligned,
            reference_xyz,
            reference_rotations,
        ),
    }


def evaluate(
    path: Path,
    expected_endpoint_distance: float | None,
    reference_path: Path | None = None,
    min_reference_reach_m: float = 10.0,
) -> dict[str, Any]:
    trajectory, result = trajectory_metrics(path, expected_endpoint_distance)
    if reference_path is not None:
        result["reference"] = reference_metrics(
            trajectory,
            reference_path,
            min_reference_reach_m,
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--expected-endpoint-distance", type=float)
    parser.add_argument(
        "--reference-trajectory",
        type=Path,
        help=(
            "Dense TUM reference for time-aligned reach ratios and an "
            "SE(3)-aligned translation delta"
        ),
    )
    parser.add_argument(
        "--min-reference-reach-m",
        type=float,
        default=10.0,
        help="Ignore startup reference reach below this value (default: 10.0)",
    )
    args = parser.parse_args()
    if args.expected_endpoint_distance is not None and args.expected_endpoint_distance <= 0:
        parser.error("--expected-endpoint-distance must be positive")
    if args.min_reference_reach_m <= 0:
        parser.error("--min-reference-reach-m must be positive")
    if args.reference_trajectory is not None and not args.reference_trajectory.is_file():
        parser.error(
            f"--reference-trajectory not found: {args.reference_trajectory}"
        )
    print(json.dumps(
        evaluate(
            args.trajectory,
            args.expected_endpoint_distance,
            args.reference_trajectory,
            args.min_reference_reach_m,
        ),
        indent=2,
        sort_keys=True,
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
