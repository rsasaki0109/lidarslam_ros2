#!/usr/bin/env python3
"""GPU/ROS-free core for the 3DGS post-process map deliverable.

This module is the testable heart of the LiDAR-primed 3D Gaussian Splatting
pipeline described in ``docs/research/3dgs-postprocess-map-design.md``. It
turns a SLAM trajectory (TUM) plus a static camera extrinsic plus camera
intrinsics into per-image ``world <- camera`` poses, and serialises them in
the Nerfstudio ``transforms.json`` convention that gsplat (Apache-2.0)
consumes.

Deliberately depends on numpy only -- no ROS, no torch, no CUDA -- so it runs
in the existing ament pytest harness on a CPU runner. The heavy bag reading
and gsplat training live in separate, opt-in modules.

Coordinate conventions
----------------------
* SLAM/TUM poses are ``world <- body`` in ROS (right-handed) convention.
* A ROS *camera optical* frame is x-right, y-down, z-forward.
* Nerfstudio / OpenGL cameras are x-right, y-up, z-back.

``transform_matrix`` in ``transforms.json`` is camera-to-world in the OpenGL
convention, so we right-multiply the ROS-optical pose by
``ROS_OPTICAL_TO_OPENGL = diag(1, -1, -1, 1)``.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Sequence

import numpy as np

# GL camera axes expressed in the ROS optical frame: x stays, y and z flip.
ROS_OPTICAL_TO_OPENGL = np.diag([1.0, -1.0, -1.0, 1.0])


# --------------------------------------------------------------------------- #
# Quaternion helpers (xyzw order, matching ROS geometry_msgs/Quaternion)
# --------------------------------------------------------------------------- #
def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Return a unit quaternion (xyzw). Raises on a zero quaternion."""
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        raise ValueError('cannot normalize a zero-norm quaternion')
    return q / n


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Convert an xyzw quaternion to a 3x3 rotation matrix."""
    x, y, z, w = quat_normalize(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two xyzw quaternions.

    ``t`` is clamped to ``[0, 1]``. Handles the double-cover by flipping the
    sign of ``q1`` when the dot product is negative, so the shorter arc is
    always taken.
    """
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    t = float(min(1.0, max(0.0, t)))

    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        # Nearly parallel -- linear interpolation is numerically safe.
        return quat_normalize(q0 + t * (q1 - q0))

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = np.sin(theta) / sin_theta_0
    return quat_normalize(s0 * q0 + s1 * q1)


def make_transform(translation: Sequence[float], quat_xyzw: Sequence[float]) -> np.ndarray:
    """Build a 4x4 homogeneous ``world <- body`` matrix."""
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(np.asarray(quat_xyzw, dtype=float))
    T[:3, 3] = np.asarray(translation, dtype=float)
    return T


# --------------------------------------------------------------------------- #
# Trajectory (TUM) parsing and interpolation
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class TrajectorySample:
    """A single TUM trajectory row: timestamp + translation + xyzw quat."""

    stamp: float
    translation: np.ndarray  # shape (3,)
    quat_xyzw: np.ndarray  # shape (4,)

    def matrix(self) -> np.ndarray:
        """Return this sample as a 4x4 ``world <- body`` matrix."""
        return make_transform(self.translation, self.quat_xyzw)


def read_tum_trajectory(path: str | Path) -> list[TrajectorySample]:
    """Parse a TUM trajectory file (``stamp tx ty tz qx qy qz qw`` per line).

    Blank lines and ``#`` comments are skipped. Samples are returned sorted by
    timestamp. Raises ``ValueError`` on a malformed row.
    """
    samples: list[TrajectorySample] = []
    for lineno, raw in enumerate(Path(path).read_text().splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        if len(parts) != 8:
            raise ValueError(f'{path}:{lineno}: expected 8 columns, got {len(parts)}')
        try:
            values = [float(p) for p in parts]
        except ValueError as exc:
            raise ValueError(f'{path}:{lineno}: non-numeric value: {exc}') from exc
        samples.append(
            TrajectorySample(
                stamp=values[0],
                translation=np.array(values[1:4]),
                quat_xyzw=np.array(values[4:8]),
            )
        )
    samples.sort(key=lambda s: s.stamp)
    return samples


def interpolate_pose(
    samples: Sequence[TrajectorySample],
    stamp: float,
    *,
    max_extrapolation: float = 0.0,
) -> np.ndarray:
    """Interpolate a ``world <- body`` pose at ``stamp``.

    Translation is linearly interpolated; rotation uses SLERP. Stamps outside
    ``[first, last]`` are allowed only within ``max_extrapolation`` seconds, in
    which case the nearest endpoint pose is clamped (held), not extrapolated.
    Beyond that tolerance a ``ValueError`` is raised so callers can drop the
    frame rather than silently invent a pose.
    """
    if not samples:
        raise ValueError('empty trajectory')

    first, last = samples[0], samples[-1]
    if stamp < first.stamp:
        if first.stamp - stamp > max_extrapolation:
            raise ValueError(
                f'stamp {stamp:.6f} precedes trajectory start {first.stamp:.6f} '
                f'by more than {max_extrapolation}s'
            )
        return first.matrix()
    if stamp > last.stamp:
        if stamp - last.stamp > max_extrapolation:
            raise ValueError(
                f'stamp {stamp:.6f} follows trajectory end {last.stamp:.6f} '
                f'by more than {max_extrapolation}s'
            )
        return last.matrix()

    # Binary search for the bracketing pair.
    lo, hi = 0, len(samples) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if samples[mid].stamp <= stamp:
            lo = mid
        else:
            hi = mid

    a, b = samples[lo], samples[hi]
    span = b.stamp - a.stamp
    alpha = 0.0 if span <= 0 else (stamp - a.stamp) / span

    translation = (1.0 - alpha) * a.translation + alpha * b.translation
    quat = quat_slerp(a.quat_xyzw, b.quat_xyzw, alpha)
    return make_transform(translation, quat)


# --------------------------------------------------------------------------- #
# Camera model + extrinsic composition + Nerfstudio export
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics, matching a ROS ``sensor_msgs/CameraInfo``."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...] = ()  # (k1, k2, p1, p2, k3) plumb-bob order
    distortion_model: str = 'plumb_bob'

    @classmethod
    def from_camera_info(cls, width: int, height: int, k: Sequence[float],
                         d: Sequence[float] = ()) -> 'CameraIntrinsics':
        """Build from a CameraInfo 3x3 ``K`` (row-major, len 9) and ``D``."""
        k = list(k)
        if len(k) != 9:
            raise ValueError(f'K must have 9 entries, got {len(k)}')
        return cls(
            width=int(width),
            height=int(height),
            fx=float(k[0]),
            fy=float(k[4]),
            cx=float(k[2]),
            cy=float(k[5]),
            distortion=tuple(float(x) for x in d),
        )


@dataclass
class PosedImage:
    """An image with its resolved ``world <- camera_optical`` (ROS) pose."""

    file_path: str
    world_T_cam_optical: np.ndarray  # 4x4, ROS optical convention
    stamp: float

    def opengl_c2w(self) -> np.ndarray:
        """Camera-to-world in the OpenGL/Nerfstudio convention."""
        return self.world_T_cam_optical @ ROS_OPTICAL_TO_OPENGL


def compose_world_T_camera(
    world_T_body: np.ndarray, body_T_camera_optical: np.ndarray
) -> np.ndarray:
    """Chain the SLAM body pose with the static camera extrinsic."""
    return np.asarray(world_T_body) @ np.asarray(body_T_camera_optical)


def build_transforms(
    intrinsics: CameraIntrinsics, frames: Sequence[PosedImage]
) -> dict:
    """Assemble a Nerfstudio-style ``transforms.json`` dict.

    The intrinsics live at the top level (shared pinhole), and each frame
    carries its ``file_path`` and OpenGL ``transform_matrix``.
    """
    d = list(intrinsics.distortion) + [0.0] * 5
    doc: dict = {
        'camera_model': 'OPENCV',
        'w': intrinsics.width,
        'h': intrinsics.height,
        'fl_x': intrinsics.fx,
        'fl_y': intrinsics.fy,
        'cx': intrinsics.cx,
        'cy': intrinsics.cy,
        'k1': d[0],
        'k2': d[1],
        'p1': d[2],
        'p2': d[3],
        'k3': d[4],
        'frames': [
            {
                'file_path': fr.file_path,
                'transform_matrix': fr.opengl_c2w().tolist(),
                'stamp': fr.stamp,
            }
            for fr in frames
        ],
    }
    return doc


def write_transforms(
    out_path: str | Path,
    intrinsics: CameraIntrinsics,
    frames: Sequence[PosedImage],
) -> Path:
    """Write ``transforms.json`` and return the path."""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(build_transforms(intrinsics, frames), indent=2))
    return out_path
