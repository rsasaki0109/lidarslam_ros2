#!/usr/bin/env python3
"""Propagate sparse pose-graph corrections onto a dense SLAM trajectory."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Sequence

import numpy as np

from lidarslam_benchmark_tools.gaussian_splatting import posed_images as pi


@dataclass(frozen=True)
class CorrectionAnchor:
    """One world-side pose correction at a graph keyframe timestamp."""

    stamp: float
    translation: np.ndarray
    quat_xyzw: np.ndarray


def matrix_to_quat_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a 3x3 rotation matrix to a normalized xyzw quaternion."""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError(f'rotation must be 3x3, got {matrix.shape}')
    # Shepperd's method selects the best-conditioned branch.
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * np.sqrt(trace + 1.0)
        quat = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        axis = int(np.argmax(np.diag(matrix)))
        if axis == 0:
            scale = 2.0 * np.sqrt(1.0 + matrix[0, 0] -
                                  matrix[1, 1] - matrix[2, 2])
            quat = np.array([
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ])
        elif axis == 1:
            scale = 2.0 * np.sqrt(1.0 + matrix[1, 1] -
                                  matrix[0, 0] - matrix[2, 2])
            quat = np.array([
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ])
        else:
            scale = 2.0 * np.sqrt(1.0 + matrix[2, 2] -
                                  matrix[0, 0] - matrix[1, 1])
            quat = np.array([
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])
    return pi.quat_normalize(quat)


def build_correction_anchors(
    raw: Sequence[pi.TrajectorySample],
    corrected: Sequence[pi.TrajectorySample],
    *,
    max_anchor_offset: float = 0.2,
) -> list[CorrectionAnchor]:
    """Compute ``corrected @ inverse(raw)`` at each graph timestamp."""
    if not raw:
        raise ValueError('raw trajectory is empty')
    if not corrected:
        raise ValueError('corrected trajectory is empty')
    anchors = []
    for sample in corrected:
        raw_pose = pi.interpolate_pose(
            raw, sample.stamp, max_extrapolation=max_anchor_offset)
        correction = sample.matrix() @ np.linalg.inv(raw_pose)
        anchors.append(CorrectionAnchor(
            stamp=sample.stamp,
            translation=correction[:3, 3].copy(),
            quat_xyzw=matrix_to_quat_xyzw(correction[:3, :3]),
        ))
    return anchors


def interpolate_correction(anchors: Sequence[CorrectionAnchor],
                           stamp: float) -> np.ndarray:
    """Interpolate a world-side correction, clamping outside anchor range."""
    if not anchors:
        raise ValueError('correction anchors are empty')
    if stamp <= anchors[0].stamp:
        anchor = anchors[0]
        return pi.make_transform(anchor.translation, anchor.quat_xyzw)
    if stamp >= anchors[-1].stamp:
        anchor = anchors[-1]
        return pi.make_transform(anchor.translation, anchor.quat_xyzw)

    lo, hi = 0, len(anchors) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if anchors[mid].stamp <= stamp:
            lo = mid
        else:
            hi = mid
    left, right = anchors[lo], anchors[hi]
    span = right.stamp - left.stamp
    alpha = 0.0 if span <= 0.0 else (stamp - left.stamp) / span
    translation = ((1.0 - alpha) * left.translation +
                   alpha * right.translation)
    quat = pi.quat_slerp(left.quat_xyzw, right.quat_xyzw, alpha)
    return pi.make_transform(translation, quat)


def densify_trajectory(
    raw: Sequence[pi.TrajectorySample],
    corrected: Sequence[pi.TrajectorySample],
    *,
    max_anchor_offset: float = 0.2,
) -> list[pi.TrajectorySample]:
    """Apply interpolated graph corrections to every dense raw pose."""
    anchors = build_correction_anchors(
        raw, corrected, max_anchor_offset=max_anchor_offset)
    dense = []
    for sample in raw:
        pose = interpolate_correction(anchors, sample.stamp) @ sample.matrix()
        dense.append(pi.TrajectorySample(
            stamp=sample.stamp,
            translation=pose[:3, 3].copy(),
            quat_xyzw=matrix_to_quat_xyzw(pose[:3, :3]),
        ))
    return dense


def write_tum(path: str | Path,
              samples: Sequence[pi.TrajectorySample]) -> Path:
    """Write trajectory samples in TUM timestamp/translation/xyzw format."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for sample in samples:
        values = [sample.stamp, *sample.translation, *sample.quat_xyzw]
        lines.append(' '.join(f'{float(value):.17g}' for value in values))
    target.write_text('\n'.join(lines) + ('\n' if lines else ''))
    return target


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True,
                        help='dense pre-optimization TUM trajectory')
    parser.add_argument('--corrected', type=Path, required=True,
                        help='sparse optimized graph-node TUM trajectory')
    parser.add_argument('--output', type=Path, required=True,
                        help='output dense corrected TUM trajectory')
    parser.add_argument('--max-anchor-offset', type=float, default=0.2,
                        help='allowed corrected-anchor extrapolation (s)')
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Generate and write the dense corrected trajectory."""
    args = build_parser().parse_args(argv)
    raw = pi.read_tum_trajectory(args.raw)
    corrected = pi.read_tum_trajectory(args.corrected)
    dense = densify_trajectory(
        raw, corrected, max_anchor_offset=args.max_anchor_offset)
    output = write_tum(args.output, dense)
    print(f'wrote {len(dense)} dense corrected poses from '
          f'{len(corrected)} graph anchors -> {output}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
