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

"""Score a TUM trajectory against timestamped position-only ground truth."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


def read_tum_positions(path: Path) -> tuple[np.ndarray, np.ndarray]:
    rows: list[list[float]] = []
    for line_number, raw in enumerate(
            path.read_text(encoding='utf-8', errors='replace').splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        fields = line.split()
        if len(fields) < 4:
            raise ValueError(f'{path}:{line_number}: expected timestamp x y z')
        rows.append([float(value) for value in fields[:4]])
    if len(rows) < 3:
        raise ValueError(f'{path}: expected at least three poses')
    values = np.asarray(rows, dtype=np.float64)
    if not np.all(np.isfinite(values)):
        raise ValueError(f'{path}: trajectory contains non-finite values')
    if np.any(np.diff(values[:, 0]) <= 0.0):
        raise ValueError(f'{path}: timestamps must be strictly increasing')
    return values[:, 0], values[:, 1:4]


def interpolate_estimate(
        reference_stamps: np.ndarray,
        estimate_stamps: np.ndarray,
        estimate_positions: np.ndarray,
        max_time_gap: float,
) -> tuple[np.ndarray, np.ndarray]:
    right = np.searchsorted(estimate_stamps, reference_stamps, side='left')
    exact = (
        (right < len(estimate_stamps))
        & (estimate_stamps[np.minimum(right, len(estimate_stamps) - 1)]
           == reference_stamps)
    )
    left = right - 1
    valid = exact | (
        (left >= 0)
        & (right < len(estimate_stamps))
        & ((reference_stamps - estimate_stamps[np.maximum(left, 0)])
           <= max_time_gap)
        & ((estimate_stamps[np.minimum(right, len(estimate_stamps) - 1)]
            - reference_stamps) <= max_time_gap)
    )
    interpolated = np.empty((int(valid.sum()), 3), dtype=np.float64)
    valid_indices = np.flatnonzero(valid)
    for output_index, reference_index in enumerate(valid_indices):
        right_index = right[reference_index]
        if exact[reference_index]:
            interpolated[output_index] = estimate_positions[right_index]
            continue
        left_index = right_index - 1
        span = estimate_stamps[right_index] - estimate_stamps[left_index]
        alpha = (
            (reference_stamps[reference_index] - estimate_stamps[left_index])
            / span
        )
        interpolated[output_index] = (
            (1.0 - alpha) * estimate_positions[left_index]
            + alpha * estimate_positions[right_index]
        )
    return valid, interpolated


def rigid_align(
        estimate_positions: np.ndarray,
        reference_positions: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    estimate_center = estimate_positions.mean(axis=0)
    reference_center = reference_positions.mean(axis=0)
    covariance = (
        (estimate_positions - estimate_center).T
        @ (reference_positions - reference_center)
    )
    left, _, right = np.linalg.svd(covariance)
    correction = np.eye(3)
    correction[2, 2] = np.sign(np.linalg.det(right.T @ left.T))
    rotation = right.T @ correction @ left.T
    translation = reference_center - rotation @ estimate_center
    aligned = (rotation @ estimate_positions.T).T + translation
    return aligned, rotation, translation


def segment_rte_percent(
        reference_positions: np.ndarray,
        aligned_positions: np.ndarray,
        segment_length: float,
) -> np.ndarray:
    cumulative = np.concatenate((
        np.array([0.0]),
        np.cumsum(np.linalg.norm(np.diff(reference_positions, axis=0), axis=1)),
    ))
    errors = []
    for start in range(len(cumulative)):
        end = int(np.searchsorted(
            cumulative, cumulative[start] + segment_length, side='left'))
        if end >= len(cumulative):
            break
        reference_delta = reference_positions[end] - reference_positions[start]
        estimate_delta = aligned_positions[end] - aligned_positions[start]
        reference_distance = np.linalg.norm(reference_delta)
        if reference_distance > 1e-9:
            errors.append(
                100.0 * np.linalg.norm(estimate_delta - reference_delta)
                / segment_length
            )
    if not errors:
        raise ValueError(
            f'trajectory has no complete {segment_length:g} m segments')
    return np.asarray(errors)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def score(
        reference_path: Path,
        estimate_path: Path,
        segment_length: float,
        max_time_gap: float,
) -> dict[str, Any]:
    reference_stamps, reference_positions = read_tum_positions(reference_path)
    estimate_stamps, estimate_positions = read_tum_positions(estimate_path)
    valid, interpolated = interpolate_estimate(
        reference_stamps, estimate_stamps, estimate_positions, max_time_gap)
    if int(valid.sum()) < 3:
        raise ValueError('fewer than three ground-truth poses could be matched')
    matched_reference = reference_positions[valid]
    aligned, rotation, translation = rigid_align(
        interpolated, matched_reference)
    absolute_errors = np.linalg.norm(aligned - matched_reference, axis=1)
    valid_indices = np.flatnonzero(valid)
    blocks = np.split(
        np.arange(len(valid_indices)),
        np.flatnonzero(np.diff(valid_indices) > 1) + 1,
    )
    relative_error_blocks = []
    for block in blocks:
        try:
            relative_error_blocks.append(segment_rte_percent(
                matched_reference[block], aligned[block], segment_length))
        except ValueError:
            continue
    if not relative_error_blocks:
        raise ValueError(
            f'matched trajectory has no complete {segment_length:g} m segments')
    relative_errors = np.concatenate(relative_error_blocks)
    return {
        'schema_version': 1,
        'alignment': {
            'type': 'se3_no_scale',
            'rotation_row_major': rotation.reshape(-1).tolist(),
            'translation_m': translation.tolist(),
        },
        'input': {
            'reference_path': str(reference_path.resolve()),
            'reference_sha256': sha256(reference_path),
            'estimate_path': str(estimate_path.resolve()),
            'estimate_sha256': sha256(estimate_path),
        },
        'association': {
            'reference_poses': len(reference_stamps),
            'matched_poses': int(valid.sum()),
            'matched_ground_truth_fraction': float(valid.mean()),
            'contiguous_blocks': len(blocks),
            'maximum_interpolation_time_gap_s': max_time_gap,
        },
        'trajectory': {
            'ate_rmse_m': float(np.sqrt(np.mean(absolute_errors ** 2))),
            'ate_mean_m': float(np.mean(absolute_errors)),
            'ate_median_m': float(np.median(absolute_errors)),
            'rte_translation_percent_10m': float(np.mean(relative_errors)),
            'rte_translation_percent_segment_mean': float(
                np.mean(relative_errors)),
            'rte_translation_percent_segment_median': float(
                np.median(relative_errors)),
            'rte_segment_length_m': segment_length,
            'rte_segments': len(relative_errors),
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            'Score an estimated TUM trajectory against position-only ground '
            'truth using scale-free SE(3) alignment and distance-based RTE.'
        ))
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--estimate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--segment-length', type=float, default=10.0)
    parser.add_argument('--max-time-gap', type=float, default=0.11)
    args = parser.parse_args()
    if args.segment_length <= 0.0 or args.max_time_gap < 0.0:
        parser.error('segment length must be positive and time gap non-negative')
    return args


def main() -> int:
    args = parse_args()
    document = score(
        args.reference, args.estimate, args.segment_length, args.max_time_gap)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(document, indent=2) + '\n')
    print(json.dumps(document['trajectory'], indent=2))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, np.linalg.LinAlgError) as error:
        print(f'error: {error}', file=sys.stderr)
        sys.exit(1)
