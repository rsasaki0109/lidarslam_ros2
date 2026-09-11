#!/usr/bin/env python3
"""Run the v44c synthetic numerical contracts without raw estimator inputs."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import resource
import struct
import time
from typing import Any, Callable, Iterable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


class ContractError(ValueError):
    """Raised when a frozen synthetic numerical contract is violated."""


class MemoryBudgetError(RuntimeError):
    """Raised before a synthetic allocation would exceed its bound."""


def canonical_json(payload: Any) -> str:
    return json.dumps(payload, sort_keys=True, separators=(',', ':'), allow_nan=False)


def payload_sha256(payload: Any) -> str:
    return hashlib.sha256(canonical_json(payload).encode('utf-8')).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def resolve_path(value: str | Path) -> Path:
    path = Path(value)
    return path if path.is_absolute() else ROOT / path


def current_rss_mib() -> float:
    status = Path('/proc/self/status')
    if status.is_file():
        for line in status.read_text(encoding='utf-8').splitlines():
            if line.startswith('VmRSS:'):
                return float(line.split()[1]) / 1024.0
    return float(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss) / 1024.0


class MemoryGuard:
    def __init__(self, maximum_rss_mib: float,
                 maximum_incremental_rss_mib: float | None = None) -> None:
        self.maximum_rss_mib = float(maximum_rss_mib)
        self.maximum_incremental_rss_mib = float(
            maximum_incremental_rss_mib
            if maximum_incremental_rss_mib is not None else maximum_rss_mib)
        self.baseline_rss_mib = current_rss_mib()
        self.peak_rss_mib = self.baseline_rss_mib
        self.peak_incremental_rss_mib = 0.0
        self.absolute_ceiling_enforced = (
            self.baseline_rss_mib <= self.maximum_rss_mib)

    def check(self, label: str) -> None:
        rss = current_rss_mib()
        self.peak_rss_mib = max(self.peak_rss_mib, rss)
        incremental = max(0.0, rss - self.baseline_rss_mib)
        self.peak_incremental_rss_mib = max(
            self.peak_incremental_rss_mib, incremental)
        if self.absolute_ceiling_enforced and rss > self.maximum_rss_mib:
            raise MemoryBudgetError(
                f'RSS {rss:.3f} MiB exceeds {self.maximum_rss_mib:.3f} MiB at {label}')
        if incremental > self.maximum_incremental_rss_mib:
            raise MemoryBudgetError(
                f'incremental RSS {incremental:.3f} MiB exceeds '
                f'{self.maximum_incremental_rss_mib:.3f} MiB at {label}')


def reserve_bytes(current: int, requested: int, maximum: int) -> int:
    current = int(current)
    requested = int(requested)
    maximum = int(maximum)
    if min(current, requested, maximum) < 0:
        raise MemoryBudgetError('byte counts must be non-negative')
    if requested > maximum - current:
        raise MemoryBudgetError(
            f'reservation {current}+{requested} exceeds {maximum} bytes')
    return current + requested


def require(condition: bool, message: str) -> None:
    if not bool(condition):
        raise ContractError(message)


def require_close(actual: Any, expected: Any, tolerance: float, label: str) -> float:
    actual_array = np.asarray(actual, dtype=np.float64)
    expected_array = np.asarray(expected, dtype=np.float64)
    if actual_array.shape != expected_array.shape:
        raise ContractError(
            f'{label} shape differs: {actual_array.shape} != {expected_array.shape}')
    if not np.all(np.isfinite(actual_array)):
        raise ContractError(f'{label} is non-finite')
    error = float(np.max(np.abs(actual_array - expected_array), initial=0.0))
    if error > float(tolerance):
        raise ContractError(f'{label} error {error:.12g} exceeds {tolerance:.12g}')
    return error


def skew(vector: Iterable[float]) -> np.ndarray:
    x, y, z = np.asarray(tuple(vector), dtype=np.float64)
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ], dtype=np.float64)


def vee(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    return np.array([
        matrix[2, 1] - matrix[1, 2],
        matrix[0, 2] - matrix[2, 0],
        matrix[1, 0] - matrix[0, 1],
    ], dtype=np.float64) * 0.5


def so3_exp(rotation_vector: Iterable[float]) -> np.ndarray:
    vector = np.asarray(tuple(rotation_vector), dtype=np.float64)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ContractError('SO3 exponential requires one finite 3-vector')
    theta2 = float(vector @ vector)
    matrix = skew(vector)
    if theta2 < 1e-16:
        a = 1.0 - theta2 / 6.0 + theta2 * theta2 / 120.0
        b = 0.5 - theta2 / 24.0 + theta2 * theta2 / 720.0
    else:
        theta = math.sqrt(theta2)
        a = math.sin(theta) / theta
        b = (1.0 - math.cos(theta)) / theta2
    return np.eye(3) + a * matrix + b * (matrix @ matrix)


def so3_log(rotation: np.ndarray) -> np.ndarray:
    rotation = np.asarray(rotation, dtype=np.float64)
    validate_rotation(rotation)
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    theta = math.acos(cosine)
    antisymmetric = vee(rotation - rotation.T)
    if theta < 1e-8:
        return 0.5 * antisymmetric
    sine = math.sin(theta)
    if abs(sine) < 1e-10:
        raise ContractError('SO3 logarithm near pi is outside the synthetic domain')
    return antisymmetric * (theta / (2.0 * sine))


def validate_rotation(rotation: np.ndarray, tolerance: float = 1e-9) -> None:
    rotation = np.asarray(rotation, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ContractError('rotation must be one finite 3x3 matrix')
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=tolerance, rtol=0.0):
        raise ContractError('rotation is not orthonormal')
    if not math.isclose(float(np.linalg.det(rotation)), 1.0,
                        abs_tol=tolerance, rel_tol=0.0):
        raise ContractError('rotation determinant differs from one')


def rotation_from_two_vectors(source: Iterable[float],
                              target: Iterable[float]) -> np.ndarray:
    first = np.asarray(tuple(source), dtype=np.float64)
    second = np.asarray(tuple(target), dtype=np.float64)
    if first.shape != (3,) or second.shape != (3,):
        raise ContractError('vector alignment requires two 3-vectors')
    first_norm = float(np.linalg.norm(first))
    second_norm = float(np.linalg.norm(second))
    if min(first_norm, second_norm) <= 1e-12:
        raise ContractError('cannot align a zero vector')
    first /= first_norm
    second /= second_norm
    cross = np.cross(first, second)
    cosine = float(np.clip(first @ second, -1.0, 1.0))
    sine = float(np.linalg.norm(cross))
    if sine <= 1e-12:
        if cosine > 0.0:
            return np.eye(3)
        basis = np.zeros(3)
        basis[int(np.argmin(np.abs(first)))] = 1.0
        axis = np.cross(first, basis)
        axis /= np.linalg.norm(axis)
        return so3_exp(axis * math.pi)
    axis = cross / sine
    return so3_exp(axis * math.atan2(sine, cosine))


def make_time_grid(duration_sec: float, period_sec: float) -> np.ndarray:
    duration_sec = float(duration_sec)
    period_sec = float(period_sec)
    if duration_sec <= 0.0 or period_sec <= 0.0:
        raise ContractError('duration and period must be positive')
    steps = int(round(duration_sec / period_sec))
    if not math.isclose(steps * period_sec, duration_sec, abs_tol=1e-12):
        raise ContractError('duration must be an integer number of sample periods')
    return np.rint(np.arange(steps + 1, dtype=np.float64)
                   * period_sec * 1e9).astype(np.int64)


def interpolate_vector(times_ns: np.ndarray, values: np.ndarray,
                       query_ns: int) -> np.ndarray:
    times = np.asarray(times_ns, dtype=np.int64)
    vectors = np.asarray(values, dtype=np.float64)
    if times.ndim != 1 or vectors.shape != (len(times), 3):
        raise ContractError('interpolation input dimensions differ')
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ContractError('interpolation timestamps must be strictly increasing')
    query = int(query_ns)
    if query < int(times[0]) or query > int(times[-1]):
        raise ContractError('interpolation query is outside coverage')
    right = int(np.searchsorted(times, query, side='left'))
    if right < len(times) and int(times[right]) == query:
        return vectors[right].copy()
    left = right - 1
    alpha = (query - int(times[left])) / (int(times[right]) - int(times[left]))
    return vectors[left] + alpha * (vectors[right] - vectors[left])


def extract_bracketed_interval(
        times_ns: np.ndarray, gyro: np.ndarray, accel: np.ndarray,
        start_ns: int, end_ns: int, maximum_gap_ns: int,
        maximum_boundary_distance_ns: int, minimum_samples: int,
        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    times = np.asarray(times_ns, dtype=np.int64)
    gyro = np.asarray(gyro, dtype=np.float64)
    accel = np.asarray(accel, dtype=np.float64)
    if times.ndim != 1 or gyro.shape != (len(times), 3) or accel.shape != (
            len(times), 3):
        raise ContractError('IMU interval input dimensions differ')
    if len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ContractError('IMU timestamps must be strictly increasing')
    start = int(start_ns)
    end = int(end_ns)
    if start >= end or start < int(times[0]) or end > int(times[-1]):
        raise ContractError('scan interval is outside IMU coverage')
    start_right = int(np.searchsorted(times, start, side='left'))
    end_right = int(np.searchsorted(times, end, side='left'))
    start_distances = []
    if start_right < len(times):
        start_distances.append(abs(int(times[start_right]) - start))
    if start_right > 0:
        start_distances.append(abs(start - int(times[start_right - 1])))
    end_distances = []
    if end_right < len(times):
        end_distances.append(abs(int(times[end_right]) - end))
    if end_right > 0:
        end_distances.append(abs(end - int(times[end_right - 1])))
    if max(min(start_distances), min(end_distances)) > int(
            maximum_boundary_distance_ns):
        raise ContractError('IMU boundary bracket distance exceeds the contract')
    interior = times[(times > start) & (times < end)]
    output_times = np.concatenate((
        np.array([start], dtype=np.int64), interior,
        np.array([end], dtype=np.int64)))
    if np.any(np.diff(output_times) > int(maximum_gap_ns)):
        raise ContractError('IMU timestamp gap exceeds the contract')
    if len(output_times) < int(minimum_samples):
        raise ContractError('too few IMU samples bracket the scan')
    output_gyro = np.vstack([
        interpolate_vector(times, gyro, int(stamp)) for stamp in output_times])
    output_accel = np.vstack([
        interpolate_vector(times, accel, int(stamp)) for stamp in output_times])
    return output_times, output_gyro, output_accel


@dataclass(frozen=True)
class PreintegrationResult:
    delta_R: np.ndarray
    delta_v: np.ndarray
    delta_p: np.ndarray
    covariance: np.ndarray
    duration_sec: float
    bias_gyro_reference: np.ndarray
    bias_accel_reference: np.ndarray
    J_R_bg: np.ndarray | None = None
    J_v_bg: np.ndarray | None = None
    J_v_ba: np.ndarray | None = None
    J_p_bg: np.ndarray | None = None
    J_p_ba: np.ndarray | None = None


def preintegrate_midpoint(
        times_ns: np.ndarray, gyro: np.ndarray, accel: np.ndarray,
        bias_gyro: Iterable[float], bias_accel: Iterable[float],
        noise: dict[str, Any], maximum_step_ns: int = 50000000,
        ) -> PreintegrationResult:
    times = np.asarray(times_ns, dtype=np.int64)
    gyro = np.asarray(gyro, dtype=np.float64)
    accel = np.asarray(accel, dtype=np.float64)
    bg = np.asarray(tuple(bias_gyro), dtype=np.float64)
    ba = np.asarray(tuple(bias_accel), dtype=np.float64)
    if times.ndim != 1 or len(times) < 2:
        raise ContractError('preintegration requires at least two timestamps')
    if gyro.shape != (len(times), 3) or accel.shape != (len(times), 3):
        raise ContractError('preintegration measurement dimensions differ')
    if bg.shape != (3,) or ba.shape != (3,):
        raise ContractError('preintegration biases must be 3-vectors')
    if (np.any(np.diff(times) <= 0)
            or np.any(np.diff(times) > int(maximum_step_ns))):
        raise ContractError('preintegration timestamp gap violates the contract')
    if not all(np.all(np.isfinite(item)) for item in (gyro, accel, bg, ba)):
        raise ContractError('preintegration input is non-finite')
    sigma_gyro = float(noise['gyroscope_white_noise_density_rad_s_sqrt_hz'])
    sigma_accel = float(noise['accelerometer_white_noise_density_m_s2_sqrt_hz'])
    require(min(sigma_gyro, sigma_accel) > 0.0, 'noise density must be positive')
    delta_R = np.eye(3)
    delta_v = np.zeros(3)
    delta_p = np.zeros(3)
    covariance = np.zeros((9, 9))
    continuous_noise = np.diag([
        sigma_gyro ** 2, sigma_gyro ** 2, sigma_gyro ** 2,
        sigma_accel ** 2, sigma_accel ** 2, sigma_accel ** 2,
    ])
    for index, difference_ns in enumerate(np.diff(times)):
        dt = float(difference_ns) * 1e-9
        omega = 0.5 * (gyro[index] + gyro[index + 1]) - bg
        accel_first = accel[index] - ba
        accel_second = accel[index + 1] - ba
        next_R = delta_R @ so3_exp(omega * dt)
        acceleration_first_frame = 0.5 * (
            delta_R @ accel_first + next_R @ accel_second)
        previous_v = delta_v.copy()
        delta_p = delta_p + previous_v * dt + 0.5 * acceleration_first_frame * dt * dt
        delta_v = previous_v + acceleration_first_frame * dt
        transition = np.eye(9)
        transition[0:3, 0:3] = so3_exp(-omega * dt)
        transition[3:6, 0:3] = -skew(acceleration_first_frame) * dt
        transition[6:9, 0:3] = -0.5 * skew(acceleration_first_frame) * dt * dt
        transition[6:9, 3:6] = np.eye(3) * dt
        rotation_midpoint = delta_R @ so3_exp(omega * (0.5 * dt))
        noise_map = np.zeros((9, 6))
        noise_map[0:3, 0:3] = -np.eye(3)
        noise_map[3:6, 3:6] = rotation_midpoint
        noise_map[6:9, 3:6] = 0.5 * rotation_midpoint * dt
        covariance = (transition @ covariance @ transition.T
                      + noise_map @ continuous_noise @ noise_map.T * dt)
        covariance = 0.5 * (covariance + covariance.T)
        delta_R = next_R
    return PreintegrationResult(
        delta_R=delta_R,
        delta_v=delta_v,
        delta_p=delta_p,
        covariance=covariance,
        duration_sec=float(times[-1] - times[0]) * 1e-9,
        bias_gyro_reference=bg.copy(),
        bias_accel_reference=ba.copy(),
    )


def preintegrate_with_bias_jacobians(
        times_ns: np.ndarray, gyro: np.ndarray, accel: np.ndarray,
        bias_gyro: Iterable[float], bias_accel: Iterable[float],
        noise: dict[str, Any], epsilon: float,
        maximum_step_ns: int = 50000000) -> PreintegrationResult:
    bg = np.asarray(tuple(bias_gyro), dtype=np.float64)
    ba = np.asarray(tuple(bias_accel), dtype=np.float64)
    epsilon = float(epsilon)
    if epsilon <= 0.0:
        raise ContractError('bias finite-difference epsilon must be positive')
    base = preintegrate_midpoint(
        times_ns, gyro, accel, bg, ba, noise, maximum_step_ns)
    J_R_bg = np.zeros((3, 3))
    J_v_bg = np.zeros((3, 3))
    J_v_ba = np.zeros((3, 3))
    J_p_bg = np.zeros((3, 3))
    J_p_ba = np.zeros((3, 3))
    for axis in range(3):
        perturbation = np.zeros(3)
        perturbation[axis] = epsilon
        gyro_plus = preintegrate_midpoint(
            times_ns, gyro, accel, bg + perturbation, ba, noise, maximum_step_ns)
        gyro_minus = preintegrate_midpoint(
            times_ns, gyro, accel, bg - perturbation, ba, noise, maximum_step_ns)
        J_R_bg[:, axis] = (
            so3_log(base.delta_R.T @ gyro_plus.delta_R)
            - so3_log(base.delta_R.T @ gyro_minus.delta_R)) / (2.0 * epsilon)
        J_v_bg[:, axis] = (
            gyro_plus.delta_v - gyro_minus.delta_v) / (2.0 * epsilon)
        J_p_bg[:, axis] = (
            gyro_plus.delta_p - gyro_minus.delta_p) / (2.0 * epsilon)
        accel_plus = preintegrate_midpoint(
            times_ns, gyro, accel, bg, ba + perturbation, noise, maximum_step_ns)
        accel_minus = preintegrate_midpoint(
            times_ns, gyro, accel, bg, ba - perturbation, noise, maximum_step_ns)
        J_v_ba[:, axis] = (
            accel_plus.delta_v - accel_minus.delta_v) / (2.0 * epsilon)
        J_p_ba[:, axis] = (
            accel_plus.delta_p - accel_minus.delta_p) / (2.0 * epsilon)
    return PreintegrationResult(
        delta_R=base.delta_R,
        delta_v=base.delta_v,
        delta_p=base.delta_p,
        covariance=base.covariance,
        duration_sec=base.duration_sec,
        bias_gyro_reference=base.bias_gyro_reference,
        bias_accel_reference=base.bias_accel_reference,
        J_R_bg=J_R_bg,
        J_v_bg=J_v_bg,
        J_v_ba=J_v_ba,
        J_p_bg=J_p_bg,
        J_p_ba=J_p_ba,
    )


def corrected_preintegration(
        result: PreintegrationResult, bias_gyro: Iterable[float],
        bias_accel: Iterable[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    jacobians = (
        result.J_R_bg, result.J_v_bg, result.J_v_ba,
        result.J_p_bg, result.J_p_ba)
    if any(item is None for item in jacobians):
        raise ContractError('bias-corrected preintegration requires Jacobians')
    delta_bg = np.asarray(tuple(bias_gyro), dtype=np.float64) - (
        result.bias_gyro_reference)
    delta_ba = np.asarray(tuple(bias_accel), dtype=np.float64) - (
        result.bias_accel_reference)
    rotation = result.delta_R @ so3_exp(result.J_R_bg @ delta_bg)
    velocity = (result.delta_v + result.J_v_bg @ delta_bg
                + result.J_v_ba @ delta_ba)
    position = (result.delta_p + result.J_p_bg @ delta_bg
                + result.J_p_ba @ delta_ba)
    return rotation, velocity, position


def imu_residual(
        state_i: dict[str, np.ndarray], state_j: dict[str, np.ndarray],
        result: PreintegrationResult, gravity_W: Iterable[float],
        ) -> np.ndarray:
    corrected_R, corrected_v, corrected_p = corrected_preintegration(
        result, state_i['b_g'], state_i['b_a'])
    rotation_i = np.asarray(state_i['R'], dtype=np.float64)
    rotation_j = np.asarray(state_j['R'], dtype=np.float64)
    gravity = np.asarray(tuple(gravity_W), dtype=np.float64)
    dt = result.duration_sec
    residual_rotation = so3_log(
        corrected_R.T @ rotation_i.T @ rotation_j)
    residual_velocity = (
        rotation_i.T @ (state_j['v'] - state_i['v'] - gravity * dt)
        - corrected_v)
    residual_position = (
        rotation_i.T @ (
            state_j['p'] - state_i['p'] - state_i['v'] * dt
            - 0.5 * gravity * dt * dt)
        - corrected_p)
    return np.concatenate((
        residual_rotation, residual_velocity, residual_position,
        state_j['b_g'] - state_i['b_g'],
        state_j['b_a'] - state_i['b_a']))


@dataclass(frozen=True)
class Pose:
    rotation: np.ndarray
    translation: np.ndarray


def pose_matrix(pose: Pose) -> np.ndarray:
    validate_rotation(pose.rotation)
    translation = np.asarray(pose.translation, dtype=np.float64)
    if translation.shape != (3,) or not np.all(np.isfinite(translation)):
        raise ContractError('pose translation must be a finite 3-vector')
    result = np.eye(4)
    result[:3, :3] = pose.rotation
    result[:3, 3] = translation
    return result


def transform_point(transform: np.ndarray, point: Iterable[float]) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64)
    point = np.asarray(tuple(point), dtype=np.float64)
    if transform.shape != (4, 4) or point.shape != (3,):
        raise ContractError('point transform dimensions differ')
    return transform[:3, :3] @ point + transform[:3, 3]


def deskew_point_to_scan_end(
        point_L: Iterable[float], world_from_body_at_point: Pose,
        world_from_body_at_end: Pose, body_from_lidar: Pose) -> np.ndarray:
    T_WB_point = pose_matrix(world_from_body_at_point)
    T_WB_end = pose_matrix(world_from_body_at_end)
    T_BL = pose_matrix(body_from_lidar)
    homogeneous = np.append(np.asarray(tuple(point_L), dtype=np.float64), 1.0)
    result = np.linalg.inv(T_WB_end) @ T_WB_point @ T_BL @ homogeneous
    return result[:3]


def lidar_point_to_plane_residual(
        source_pose: Pose, current_pose: Pose, point_current_B: Iterable[float],
        surfel_mean_source_B: Iterable[float],
        surfel_normal_source_B: Iterable[float]) -> float:
    point = np.asarray(tuple(point_current_B), dtype=np.float64)
    mean = np.asarray(tuple(surfel_mean_source_B), dtype=np.float64)
    normal = np.asarray(tuple(surfel_normal_source_B), dtype=np.float64)
    normal_norm = float(np.linalg.norm(normal))
    if min(len(point), len(mean), len(normal)) != 3 or normal_norm <= 1e-12:
        raise ContractError('LiDAR factor inputs differ')
    normal = normal / normal_norm
    world_point = current_pose.rotation @ point + current_pose.translation
    point_source = source_pose.rotation.T @ (
        world_point - source_pose.translation)
    return float(normal @ (point_source - mean))


def lidar_point_to_plane_jacobians(
        source_pose: Pose, current_pose: Pose, point_current_B: Iterable[float],
        surfel_normal_source_B: Iterable[float]) -> tuple[np.ndarray, np.ndarray]:
    point = np.asarray(tuple(point_current_B), dtype=np.float64)
    normal = np.asarray(tuple(surfel_normal_source_B), dtype=np.float64)
    normal /= np.linalg.norm(normal)
    world_point = current_pose.rotation @ point + current_pose.translation
    source_vector = source_pose.rotation.T @ (
        world_point - source_pose.translation)
    source_rotation = normal @ skew(source_vector)
    source_translation = -normal @ source_pose.rotation.T
    current_rotation = normal @ source_pose.rotation.T @ (
        -current_pose.rotation @ skew(point))
    current_translation = normal @ source_pose.rotation.T
    return (
        np.concatenate((source_rotation, source_translation)),
        np.concatenate((current_rotation, current_translation)),
    )


def perturb_pose(pose: Pose, delta: Iterable[float]) -> Pose:
    delta = np.asarray(tuple(delta), dtype=np.float64)
    if delta.shape != (6,):
        raise ContractError('pose perturbation must have six entries')
    return Pose(
        rotation=pose.rotation @ so3_exp(delta[:3]),
        translation=pose.translation + delta[3:],
    )


def numerical_lidar_jacobians(
        source_pose: Pose, current_pose: Pose, point_current_B: Iterable[float],
        surfel_mean_source_B: Iterable[float],
        surfel_normal_source_B: Iterable[float], epsilon: float,
        ) -> tuple[np.ndarray, np.ndarray]:
    source = np.zeros(6)
    current = np.zeros(6)
    for axis in range(6):
        delta = np.zeros(6)
        delta[axis] = epsilon
        source[axis] = (
            lidar_point_to_plane_residual(
                perturb_pose(source_pose, delta), current_pose,
                point_current_B, surfel_mean_source_B, surfel_normal_source_B)
            - lidar_point_to_plane_residual(
                perturb_pose(source_pose, -delta), current_pose,
                point_current_B, surfel_mean_source_B, surfel_normal_source_B)
        ) / (2.0 * epsilon)
        current[axis] = (
            lidar_point_to_plane_residual(
                source_pose, perturb_pose(current_pose, delta),
                point_current_B, surfel_mean_source_B, surfel_normal_source_B)
            - lidar_point_to_plane_residual(
                source_pose, perturb_pose(current_pose, -delta),
                point_current_B, surfel_mean_source_B, surfel_normal_source_B)
        ) / (2.0 * epsilon)
    return source, current


@dataclass(frozen=True)
class ObservableSubspace:
    singular_values: np.ndarray
    retained_mask: np.ndarray
    projector: np.ndarray
    rank: int
    threshold: float


def observable_subspace(
        jacobian: np.ndarray, minimum_singular_value: float,
        maximum_condition_number: float) -> ObservableSubspace:
    jacobian = np.asarray(jacobian, dtype=np.float64)
    if jacobian.ndim != 2 or jacobian.shape[1] == 0:
        raise ContractError('observability Jacobian must be a nonempty matrix')
    if not np.all(np.isfinite(jacobian)):
        raise ContractError('observability Jacobian is non-finite')
    _, singular_values, right = np.linalg.svd(jacobian, full_matrices=False)
    maximum = float(singular_values[0]) if len(singular_values) else 0.0
    threshold = max(
        float(minimum_singular_value), maximum / float(maximum_condition_number))
    retained = singular_values >= threshold
    basis = right[retained].T
    projector = basis @ basis.T if basis.size else np.zeros(
        (jacobian.shape[1], jacobian.shape[1]))
    return ObservableSubspace(
        singular_values=singular_values,
        retained_mask=retained,
        projector=projector,
        rank=int(np.count_nonzero(retained)),
        threshold=threshold,
    )


def observable_least_squares(
        jacobian: np.ndarray, residual: np.ndarray,
        minimum_singular_value: float,
        maximum_condition_number: float,
        ) -> tuple[np.ndarray, ObservableSubspace, np.ndarray]:
    jacobian = np.asarray(jacobian, dtype=np.float64)
    residual = np.asarray(residual, dtype=np.float64)
    left, singular_values, right = np.linalg.svd(jacobian, full_matrices=False)
    subspace = observable_subspace(
        jacobian, minimum_singular_value, maximum_condition_number)
    retained = subspace.retained_mask
    update = np.zeros(jacobian.shape[1])
    if np.any(retained):
        update = -right[retained].T @ (
            (left[:, retained].T @ residual) / singular_values[retained])
    information = right[retained].T @ np.diag(
        singular_values[retained] ** 2) @ right[retained]
    return update, subspace, information


def dynamic_bootstrap_linear_oracle(
        times_sec: np.ndarray, positions_W: np.ndarray,
        specific_force_body: np.ndarray, gravity_magnitude: float,
        ) -> dict[str, Any]:
    times = np.asarray(times_sec, dtype=np.float64)
    positions = np.asarray(positions_W, dtype=np.float64)
    specific_force = np.asarray(specific_force_body, dtype=np.float64)
    if times.ndim != 1 or positions.shape != (len(times), 3):
        raise ContractError('bootstrap LiDAR dimensions differ')
    if specific_force.ndim != 2 or specific_force.shape[1] != 3:
        raise ContractError('bootstrap IMU dimensions differ')
    if len(times) < 3 or np.any(np.diff(times) <= 0):
        raise ContractError('bootstrap requires increasing motion samples')
    design = np.column_stack((
        np.ones(len(times)), times, 0.5 * times * times))
    coefficients, _, rank, _ = np.linalg.lstsq(design, positions, rcond=None)
    if rank != 3:
        raise ContractError('bootstrap motion design is rank deficient')
    acceleration = coefficients[2]
    gravity = acceleration - np.mean(specific_force, axis=0)
    norm = float(np.linalg.norm(gravity))
    if norm <= 1e-12:
        raise ContractError('bootstrap gravity estimate is zero')
    gravity = gravity * (float(gravity_magnitude) / norm)
    return {
        'architecture_path': 'dynamic_joint_lidar_imu_bootstrap_for_every_sequence',
        'initial_position_W': coefficients[0],
        'initial_velocity_W': coefficients[1],
        'constant_acceleration_W': acceleration,
        'gravity_W': gravity,
        'orientation_message_used': False,
        'zero_velocity_factor_used': False,
        'stationary_branch_used': False,
    }


@dataclass(frozen=True)
class SquareRootPrior:
    matrix: np.ndarray
    target: np.ndarray
    retained_dimension: int
    marginalized_rank: int


def square_root_marginalize(
        matrix: np.ndarray, target: np.ndarray, marginal_dimension: int,
        rank_tolerance: float = 1e-12) -> SquareRootPrior:
    matrix = np.asarray(matrix, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    marginal_dimension = int(marginal_dimension)
    if matrix.ndim != 2 or target.shape != (matrix.shape[0],):
        raise ContractError('marginalization dimensions differ')
    if not 0 < marginal_dimension < matrix.shape[1]:
        raise ContractError('marginal dimension is outside the state')
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(target)):
        raise ContractError('marginalization input is non-finite')
    marginal = matrix[:, :marginal_dimension]
    retained = matrix[:, marginal_dimension:]
    left, singular_values, _ = np.linalg.svd(marginal, full_matrices=True)
    maximum = float(singular_values[0]) if len(singular_values) else 0.0
    threshold = max(float(rank_tolerance), maximum * float(rank_tolerance))
    rank = int(np.count_nonzero(singular_values > threshold))
    null_left = left[:, rank:]
    prior_matrix = null_left.T @ retained
    prior_target = null_left.T @ target
    return SquareRootPrior(
        matrix=prior_matrix,
        target=prior_target,
        retained_dimension=retained.shape[1],
        marginalized_rank=rank,
    )


class FEJPrior:
    def __init__(self, matrix: np.ndarray, target: np.ndarray,
                 linearization_point: np.ndarray) -> None:
        self.matrix = np.array(matrix, dtype='<f8', copy=True)
        self.target = np.array(target, dtype='<f8', copy=True)
        self.linearization_point = np.array(
            linearization_point, dtype='<f8', copy=True)
        if self.matrix.ndim != 2 or self.target.shape != (self.matrix.shape[0],):
            raise ContractError('FEJ prior dimensions differ')
        if self.linearization_point.shape != (self.matrix.shape[1],):
            raise ContractError('FEJ linearization point dimensions differ')
        self.matrix.setflags(write=False)
        self.target.setflags(write=False)
        self.linearization_point.setflags(write=False)

    def residual(self, value: np.ndarray) -> np.ndarray:
        value = np.asarray(value, dtype=np.float64)
        if value.shape != self.linearization_point.shape:
            raise ContractError('FEJ evaluation dimensions differ')
        return self.matrix @ (value - self.linearization_point) - self.target

    def payload_sha256(self) -> str:
        digest = hashlib.sha256()
        digest.update(self.matrix.tobytes(order='C'))
        digest.update(self.target.tobytes(order='C'))
        digest.update(self.linearization_point.tobytes(order='C'))
        return digest.hexdigest()


class FixedLagWindow:
    def __init__(self, lag_ns: int, maximum_knots: int) -> None:
        self.lag_ns = int(lag_ns)
        self.maximum_knots = int(maximum_knots)
        if self.lag_ns <= 0 or self.maximum_knots <= 1:
            raise ContractError('fixed-lag bounds must be positive')
        self.knots: list[tuple[int, int]] = []
        self.surfels_by_knot: dict[int, tuple[int, ...]] = {}
        self.evicted: list[int] = []

    def add(self, stamp_ns: int, knot_id: int,
            surfel_ids: Iterable[int] = ()) -> list[int]:
        stamp = int(stamp_ns)
        identifier = int(knot_id)
        if self.knots and stamp <= self.knots[-1][0]:
            raise ContractError('fixed-lag knot timestamps must be strictly increasing')
        if identifier in self.surfels_by_knot:
            raise ContractError('fixed-lag knot ID is duplicated')
        self.knots.append((stamp, identifier))
        self.surfels_by_knot[identifier] = tuple(int(item) for item in surfel_ids)
        removed: list[int] = []
        while (len(self.knots) > self.maximum_knots
               or stamp - self.knots[0][0] > self.lag_ns):
            _, old_id = self.knots.pop(0)
            self.surfels_by_knot.pop(old_id, None)
            self.evicted.append(old_id)
            removed.append(old_id)
        return removed


FACTOR_TYPE_ORDER = {
    'gauge': 0,
    'marginal_prior': 1,
    'imu_preintegration': 2,
    'bias_random_walk': 3,
    'lidar_point_to_plane': 4,
}


def deterministic_factor_order(factors: list[dict[str, Any]]) -> list[dict[str, Any]]:
    for factor in factors:
        if factor.get('type') not in FACTOR_TYPE_ORDER:
            raise ContractError(f'unknown factor type {factor.get("type")}')
    return sorted(
        (dict(item) for item in factors),
        key=lambda item: (
            FACTOR_TYPE_ORDER[item['type']], int(item['end_ns']),
            int(item['start_ns']), int(item['id'])))


def state_payload_bytes(timestamp_ns: int, state_values: Iterable[float],
                        counters: Iterable[int]) -> bytes:
    values = np.asarray(tuple(state_values), dtype='<f8')
    counter_values = np.asarray(tuple(counters), dtype='<i8')
    if values.shape != (19,) or counter_values.shape != (4,):
        raise ContractError('state payload must contain 19 floats and four counters')
    if not np.all(np.isfinite(values)):
        raise ContractError('state payload is non-finite')
    return (struct.pack('<q', int(timestamp_ns)) + values.tobytes(order='C')
            + counter_values.tobytes(order='C'))


def deterministic_state_factor_digest(
        timestamp_ns: int, state_values: Iterable[float], counters: Iterable[int],
        factors: list[dict[str, Any]]) -> str:
    digest = hashlib.sha256()
    digest.update(state_payload_bytes(timestamp_ns, state_values, counters))
    digest.update(canonical_json(deterministic_factor_order(factors)).encode('utf-8'))
    return digest.hexdigest()


def validate_event_stream(
        events: list[dict[str, Any]], event_type_order: list[str],
        maximum_message_bytes: int) -> list[dict[str, Any]]:
    type_order = {name: index for index, name in enumerate(event_type_order)}
    previous = -2**63
    identifiers: set[str] = set()
    for event in events:
        stamp = int(event['stamp_ns'])
        if stamp < previous:
            raise ContractError('event stream is out of timestamp order')
        previous = stamp
        if event['type'] not in type_order:
            raise ContractError('event type is unknown')
        identifier = str(event['message_id'])
        if identifier in identifiers:
            raise ContractError('event message would be consumed more than once')
        identifiers.add(identifier)
        if int(event['size_bytes']) > int(maximum_message_bytes):
            raise MemoryBudgetError('event exceeds maximum input message bytes')
    return sorted(
        (dict(event) for event in events),
        key=lambda event: (
            int(event['stamp_ns']), type_order[event['type']],
            int(event['source_index'])))


def require_protected_payload_unchanged(before: bytes, after: bytes) -> str:
    before_digest = hashlib.sha256(before).hexdigest()
    after_digest = hashlib.sha256(after).hexdigest()
    if before_digest != after_digest:
        raise ContractError('protected output identity changed')
    return before_digest


def synthetic_imu_signal(times_ns: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    seconds = (np.asarray(times_ns, dtype=np.float64)
               - float(times_ns[0])) * 1e-9
    gyro = np.column_stack((
        0.10 + 0.02 * np.sin(1.7 * seconds),
        -0.05 + 0.01 * np.cos(1.1 * seconds),
        0.20 + 0.015 * np.sin(0.7 * seconds + 0.2),
    ))
    accel = np.column_stack((
        0.30 * np.sin(1.3 * seconds),
        0.20 * np.cos(0.9 * seconds),
        9.80665 + 0.10 * np.sin(0.5 * seconds),
    ))
    return gyro, accel


def random_orthogonal(generator: np.random.Generator, dimension: int) -> np.ndarray:
    matrix = generator.normal(size=(dimension, dimension))
    orthogonal, upper = np.linalg.qr(matrix)
    signs = np.sign(np.diag(upper))
    signs[signs == 0.0] = 1.0
    return orthogonal * signs


def case_SO3_preintegration_direction(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    times = make_time_grid(case['duration_sec'], case['sample_period_sec'])
    angular_rate = np.asarray(case['angular_rate_rad_s'], dtype=np.float64)
    gyro = np.repeat(angular_rate[None, :], len(times), axis=0)
    accel = np.zeros_like(gyro)
    result = preintegrate_midpoint(
        times, gyro, accel, np.zeros(3), np.zeros(3), context['noise'])
    expected_rotation = so3_exp(angular_rate * float(case['duration_sec']))
    rotation_error = float(np.linalg.norm(
        so3_log(expected_rotation.T @ result.delta_R)))
    require(rotation_error <= context['numeric']['SO3_absolute_tolerance'],
            'SO3 preintegration direction differs')
    probe = np.asarray(case['probe_vector'], dtype=np.float64)
    observed = result.delta_R @ probe
    require(observed[1] > 0.0 and observed[0] > 0.0,
            'positive z angular rate must rotate +x toward +y')
    return {
        'rotation_error_rad': rotation_error,
        'rotated_probe': observed.tolist(),
        'duration_sec': result.duration_sec,
    }


def case_constant_motion_zero_residual(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    times = make_time_grid(case['duration_sec'], case['sample_period_sec'])
    gravity = np.asarray(case['gravity_W_m_s2'], dtype=np.float64)
    gyro = np.zeros((len(times), 3))
    accel = np.repeat((-gravity)[None, :], len(times), axis=0)
    result = preintegrate_with_bias_jacobians(
        times, gyro, accel, np.zeros(3), np.zeros(3), context['noise'],
        context['numeric']['finite_difference_epsilon'])
    velocity = np.asarray(case['velocity_W_m_s'], dtype=np.float64)
    duration = float(case['duration_sec'])
    state_i = {
        'R': np.eye(3), 'p': np.zeros(3), 'v': velocity,
        'b_g': np.zeros(3), 'b_a': np.zeros(3),
    }
    state_j = {
        'R': np.eye(3), 'p': velocity * duration, 'v': velocity,
        'b_g': np.zeros(3), 'b_a': np.zeros(3),
    }
    residual = imu_residual(state_i, state_j, result, gravity)
    maximum = float(np.max(np.abs(residual)))
    require(maximum <= context['numeric']['zero_residual_absolute_tolerance'],
            'constant-motion IMU residual is not zero')
    return {
        'maximum_absolute_residual': maximum,
        'residual_dimension': len(residual),
    }


def case_gravity_sign_and_rebase(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    estimate = np.asarray(case['gravity_estimate_W_m_s2'], dtype=np.float64)
    target = np.asarray(case['target_gravity_W_m_s2'], dtype=np.float64)
    rebase = rotation_from_two_vectors(estimate, target)
    rebased = rebase @ (estimate / np.linalg.norm(estimate) * np.linalg.norm(target))
    error = require_close(
        rebased, target, context['numeric']['SO3_absolute_tolerance'],
        'gravity rebase')
    validate_rotation(rebase)
    require(rebased[2] < 0.0, 'gravity must point along world negative z')
    return {
        'maximum_gravity_error_m_s2': error,
        'rebased_gravity_W_m_s2': rebased.tolist(),
        'rebase_rotation_vector_rad': so3_log(rebase).tolist(),
    }


def case_gyro_bias_Jacobian_finite_difference(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    times = make_time_grid(case['duration_sec'], case['sample_period_sec'])
    gyro, accel = synthetic_imu_signal(times)
    epsilon = context['numeric']['finite_difference_epsilon']
    reference = preintegrate_with_bias_jacobians(
        times, gyro, accel, np.zeros(3), np.zeros(3), context['noise'], epsilon)
    delta = np.asarray(case['bias_delta_rad_s'], dtype=np.float64)
    direct = preintegrate_midpoint(
        times, gyro, accel, delta, np.zeros(3), context['noise'])
    predicted_R, predicted_v, predicted_p = corrected_preintegration(
        reference, delta, np.zeros(3))
    rotation_error = float(np.linalg.norm(
        so3_log(predicted_R.T @ direct.delta_R)))
    velocity_error = float(np.linalg.norm(predicted_v - direct.delta_v))
    position_error = float(np.linalg.norm(predicted_p - direct.delta_p))
    tolerance = float(case['prediction_tolerance'])
    require(rotation_error <= tolerance, 'gyro-bias rotation Jacobian differs')
    require(velocity_error <= 5.0 * tolerance,
            'gyro-bias velocity Jacobian differs')
    require(position_error <= 5.0 * tolerance,
            'gyro-bias position Jacobian differs')
    return {
        'rotation_prediction_error_rad': rotation_error,
        'velocity_prediction_error_m_s': velocity_error,
        'position_prediction_error_m': position_error,
        'J_R_bg_frobenius_norm': float(np.linalg.norm(reference.J_R_bg)),
    }


def case_accelerometer_bias_Jacobians_finite_difference(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    times = make_time_grid(case['duration_sec'], case['sample_period_sec'])
    gyro, accel = synthetic_imu_signal(times)
    reference = preintegrate_with_bias_jacobians(
        times, gyro, accel, np.zeros(3), np.zeros(3), context['noise'],
        context['numeric']['finite_difference_epsilon'])
    delta = np.asarray(case['bias_delta_m_s2'], dtype=np.float64)
    direct = preintegrate_midpoint(
        times, gyro, accel, np.zeros(3), delta, context['noise'])
    _, predicted_v, predicted_p = corrected_preintegration(
        reference, np.zeros(3), delta)
    velocity_error = float(np.linalg.norm(predicted_v - direct.delta_v))
    position_error = float(np.linalg.norm(predicted_p - direct.delta_p))
    require(velocity_error <= float(case['velocity_prediction_tolerance']),
            'accelerometer-bias velocity Jacobian differs')
    require(position_error <= float(case['position_prediction_tolerance']),
            'accelerometer-bias position Jacobian differs')
    return {
        'velocity_prediction_error_m_s': velocity_error,
        'position_prediction_error_m': position_error,
        'J_v_ba_frobenius_norm': float(np.linalg.norm(reference.J_v_ba)),
        'J_p_ba_frobenius_norm': float(np.linalg.norm(reference.J_p_ba)),
    }


def case_preintegration_covariance_PSD_and_dt_scaling(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    traces = []
    minimum_eigenvalues = []
    for duration_key in ('short_duration_sec', 'long_duration_sec'):
        times = make_time_grid(case[duration_key], case['sample_period_sec'])
        gyro, accel = synthetic_imu_signal(times)
        result = preintegrate_midpoint(
            times, gyro, accel, np.zeros(3), np.zeros(3), context['noise'])
        eigenvalues = np.linalg.eigvalsh(result.covariance)
        minimum = float(eigenvalues[0])
        require(minimum >= -context['numeric'][
            'covariance_negative_eigenvalue_tolerance'],
            'preintegration covariance is not positive semidefinite')
        traces.append(float(np.trace(result.covariance)))
        minimum_eigenvalues.append(minimum)
    ratio = traces[1] / traces[0]
    require(float(case['minimum_trace_ratio']) <= ratio <= float(
        case['maximum_trace_ratio']), 'covariance duration scaling differs')
    return {
        'short_trace': traces[0],
        'long_trace': traces[1],
        'trace_ratio': ratio,
        'minimum_eigenvalues': minimum_eigenvalues,
    }


def case_scan_boundary_interpolation(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    times = np.asarray(case['sample_times_ns'], dtype=np.int64)
    slope = np.asarray(case['linear_vector_slope_per_second'], dtype=np.float64)
    values = (times.astype(np.float64) * 1e-9)[:, None] * slope[None, :]
    output_times, output_gyro, output_accel = extract_bracketed_interval(
        times, values, 2.0 * values, case['start_ns'], case['end_ns'],
        context['architecture']['timing']['maximum_imu_gap_ns'],
        context['architecture']['timing'][
            'maximum_imu_boundary_bracket_distance_ns'],
        context['architecture']['timing']['minimum_imu_samples_per_scan'])
    expected_start = float(case['start_ns']) * 1e-9 * slope
    expected_end = float(case['end_ns']) * 1e-9 * slope
    start_error = require_close(
        output_gyro[0], expected_start, 1e-15, 'interpolated start')
    end_error = require_close(
        output_gyro[-1], expected_end, 1e-15, 'interpolated end')
    require_close(output_accel, 2.0 * output_gyro, 1e-15,
                  'paired boundary interpolation')
    return {
        'output_sample_count': len(output_times),
        'start_error': start_error,
        'end_error': end_error,
        'output_times_ns': output_times.tolist(),
    }


def case_timestamp_gap_fail_closed(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    times = np.asarray(case['sample_times_ns'], dtype=np.int64)
    values = np.zeros((len(times), 3))
    rejected = False
    try:
        extract_bracketed_interval(
            times, values, values, int(times[0]), int(times[-1]),
            case['maximum_gap_ns'],
            context['architecture']['timing'][
                'maximum_imu_boundary_bracket_distance_ns'],
            case['minimum_samples'])
    except ContractError as error:
        rejected = 'gap exceeds' in str(error)
    require(rejected, 'timestamp gap did not fail closed')
    require(not np.any(np.diff(times) == 0), 'gap case unexpectedly has duplicates')
    return {
        'gap_rejected': rejected,
        'maximum_observed_gap_ns': int(np.max(np.diff(times))),
        'maximum_allowed_gap_ns': int(case['maximum_gap_ns']),
        'state_output_count': 0,
    }


def case_point_deskew_direction(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    velocity = np.asarray(case['body_velocity_W_m_s'], dtype=np.float64)
    duration = float(case['scan_duration_sec'])
    identity = np.eye(3)
    observed = deskew_point_to_scan_end(
        case['point_L_at_scan_start_m'],
        Pose(identity, np.zeros(3)),
        Pose(identity, velocity * duration),
        Pose(identity, np.zeros(3)))
    error = require_close(
        observed, case['expected_point_B_at_scan_end_m'],
        context['numeric']['SO3_absolute_tolerance'], 'point deskew direction')
    require(observed[0] < float(case['point_L_at_scan_start_m'][0]),
            'forward body motion must move an early point backward at scan end')
    return {'deskewed_point_B_m': observed.tolist(), 'maximum_error_m': error}


def case_lidar_to_body_extrinsic_direction(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    angle = math.radians(float(case['rotation_z_deg']))
    body_from_lidar = Pose(
        so3_exp([0.0, 0.0, angle]),
        np.asarray(case['translation_BL_m'], dtype=np.float64))
    observed = transform_point(pose_matrix(body_from_lidar), case['point_L_m'])
    error = require_close(
        observed, case['expected_point_B_m'],
        context['numeric']['SO3_absolute_tolerance'], 'T_BL direction')
    inverse_result = transform_point(
        np.linalg.inv(pose_matrix(body_from_lidar)), case['point_L_m'])
    require(not np.allclose(inverse_result, observed, atol=1e-6),
            'extrinsic direction challenge is not discriminative')
    return {
        'point_B_m': observed.tolist(),
        'inverse_direction_point_m': inverse_result.tolist(),
        'maximum_error_m': error,
    }


def case_binary_point_to_plane_Jacobians_finite_difference(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    source = Pose(
        so3_exp(case['source_rotation_vector_rad']),
        np.asarray(case['source_translation_W_m'], dtype=np.float64))
    current = Pose(
        so3_exp(case['current_rotation_vector_rad']),
        np.asarray(case['current_translation_W_m'], dtype=np.float64))
    analytic = lidar_point_to_plane_jacobians(
        source, current, case['point_current_B_m'],
        case['surfel_normal_source_B'])
    numerical = numerical_lidar_jacobians(
        source, current, case['point_current_B_m'],
        case['surfel_mean_source_B_m'], case['surfel_normal_source_B'],
        context['numeric']['finite_difference_epsilon'])
    source_error = require_close(
        analytic[0], numerical[0], context['numeric'][
            'Jacobian_absolute_tolerance'], 'source LiDAR Jacobian')
    current_error = require_close(
        analytic[1], numerical[1], context['numeric'][
            'Jacobian_absolute_tolerance'], 'current LiDAR Jacobian')
    return {
        'source_maximum_error': source_error,
        'current_maximum_error': current_error,
        'source_jacobian': analytic[0].tolist(),
        'current_jacobian': analytic[1].tolist(),
    }


def case_observable_subspace_rotation_invariance(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    seed = int(context['numeric']['random_seed']) + int(
        case['coordinate_rotation_seed_offset'])
    generator = np.random.default_rng(seed)
    singular_values = np.asarray(case['singular_values'], dtype=np.float64)
    left = random_orthogonal(generator, len(singular_values))
    right = random_orthogonal(generator, len(singular_values))
    coordinate_rotation = random_orthogonal(generator, len(singular_values))
    jacobian = left @ np.diag(singular_values) @ right.T
    rotated = jacobian @ coordinate_rotation
    config = context['architecture']['observability']
    first = observable_subspace(
        jacobian, config['minimum_whitened_singular_value'],
        config['maximum_retained_condition_number'])
    second = observable_subspace(
        rotated, config['minimum_whitened_singular_value'],
        config['maximum_retained_condition_number'])
    singular_error = require_close(
        first.singular_values, second.singular_values,
        context['numeric']['SO3_absolute_tolerance'],
        'observable singular-value invariance')
    projector_error = require_close(
        second.projector, coordinate_rotation.T @ first.projector
        @ coordinate_rotation, context['numeric']['Jacobian_absolute_tolerance'],
        'observable projector coordinate rotation')
    require(first.rank == second.rank == len(singular_values),
            'observable rank changed under coordinate rotation')
    return {
        'rank': first.rank,
        'singular_value_error': singular_error,
        'projector_error': projector_error,
    }


def case_weak_axis_information_removal_without_state_clamp(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    singular_values = np.asarray(case['singular_values'], dtype=np.float64)
    jacobian = np.diag(singular_values)
    residual = np.ones(len(singular_values))
    update, subspace, information = observable_least_squares(
        jacobian, residual, case['minimum_retained_singular_value'],
        context['architecture']['observability'][
            'maximum_retained_condition_number'])
    require(subspace.rank == int(case['expected_rank']),
            'weak-axis retained rank differs')
    weak_index = int(np.argmin(singular_values))
    require(abs(float(update[weak_index])) <= 1e-15,
            'weak LiDAR mode produced a state update')
    require(float(np.linalg.norm(information[weak_index])) <= 1e-15,
            'weak LiDAR mode retained measurement information')
    process_increment = np.zeros(len(singular_values))
    process_increment[weak_index] = float(case['weak_process_increment'])
    combined = process_increment + update
    require(math.isclose(
        float(combined[weak_index]), float(case['weak_process_increment']),
        abs_tol=1e-15), 'weak process state was clamped')
    return {
        'retained_rank': subspace.rank,
        'nullspace_dimension': len(singular_values) - subspace.rank,
        'weak_measurement_update': float(update[weak_index]),
        'weak_information_norm': float(np.linalg.norm(information[weak_index])),
        'weak_process_increment_after_measurement': float(combined[weak_index]),
    }


def bootstrap_case(case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    duration = float(case['duration_sec'])
    period = float(case['lidar_sample_period_sec'])
    sample_count = int(round(duration / period)) + 1
    times = np.arange(sample_count, dtype=np.float64) * period
    velocity = np.asarray(case['initial_velocity_W_m_s'], dtype=np.float64)
    acceleration = np.asarray(
        case['constant_acceleration_W_m_s2'], dtype=np.float64)
    gravity = np.asarray(case['gravity_W_m_s2'], dtype=np.float64)
    positions = (times[:, None] * velocity[None, :]
                 + 0.5 * times[:, None] ** 2 * acceleration[None, :])
    specific_force = np.repeat(
        (acceleration - gravity)[None, :], 201, axis=0)
    result = dynamic_bootstrap_linear_oracle(
        times, positions, specific_force, np.linalg.norm(gravity))
    velocity_error = require_close(
        result['initial_velocity_W'], velocity,
        context['numeric']['Jacobian_absolute_tolerance'],
        'bootstrap initial velocity')
    acceleration_error = require_close(
        result['constant_acceleration_W'], acceleration,
        context['numeric']['Jacobian_absolute_tolerance'],
        'bootstrap acceleration')
    gravity_error = require_close(
        result['gravity_W'], gravity,
        context['numeric']['Jacobian_absolute_tolerance'], 'bootstrap gravity')
    require(result['orientation_message_used'] is False,
            'bootstrap used message orientation')
    require(result['zero_velocity_factor_used'] is False,
            'bootstrap used a zero-velocity factor')
    return {
        'architecture_path': result['architecture_path'],
        'initial_velocity_W_m_s': result['initial_velocity_W'].tolist(),
        'gravity_W_m_s2': result['gravity_W'].tolist(),
        'velocity_error': velocity_error,
        'acceleration_error': acceleration_error,
        'gravity_error': gravity_error,
        'orientation_message_used': False,
        'zero_velocity_factor_used': False,
        'stationary_branch_used': False,
    }


def case_dynamic_startup_without_orientation_or_zero_velocity_prior(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    result = bootstrap_case(case, context)
    require(float(np.linalg.norm(result['initial_velocity_W_m_s'])) > 0.5,
            'dynamic startup collapsed to zero velocity')
    return result


def case_stationary_startup_uses_same_architecture(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    result = bootstrap_case(case, context)
    require(result['architecture_path'] == (
        'dynamic_joint_lidar_imu_bootstrap_for_every_sequence'),
        'stationary startup selected another architecture')
    require(result['stationary_branch_used'] is False,
            'stationary startup used a branch')
    return result


def case_square_root_marginalization_equivalence(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    rows = int(case['rows'])
    marginal_dimension = int(case['marginal_dimension'])
    retained_dimension = int(case['retained_dimension'])
    seed = int(context['numeric']['random_seed']) + int(case['matrix_seed_offset'])
    generator = np.random.default_rng(seed)
    matrix = generator.normal(size=(rows, marginal_dimension + retained_dimension))
    matrix[:marginal_dimension + retained_dimension] += np.eye(
        marginal_dimension + retained_dimension)
    target = generator.normal(size=rows)
    prior = square_root_marginalize(matrix, target, marginal_dimension)
    full_solution, _, full_rank, _ = np.linalg.lstsq(matrix, target, rcond=None)
    prior_solution, _, prior_rank, _ = np.linalg.lstsq(
        prior.matrix, prior.target, rcond=None)
    require(full_rank == marginal_dimension + retained_dimension,
            'full synthetic system is rank deficient')
    require(prior_rank == retained_dimension,
            'square-root prior is rank deficient')
    solution_error = require_close(
        prior_solution, full_solution[marginal_dimension:],
        context['numeric']['marginalization_absolute_tolerance'],
        'marginal retained solution')
    normal = matrix.T @ matrix
    gradient = matrix.T @ target
    H_mm = normal[:marginal_dimension, :marginal_dimension]
    H_mr = normal[:marginal_dimension, marginal_dimension:]
    H_rr = normal[marginal_dimension:, marginal_dimension:]
    g_m = gradient[:marginal_dimension]
    g_r = gradient[marginal_dimension:]
    schur_H = H_rr - H_mr.T @ np.linalg.solve(H_mm, H_mr)
    schur_g = g_r - H_mr.T @ np.linalg.solve(H_mm, g_m)
    information_error = require_close(
        prior.matrix.T @ prior.matrix, schur_H,
        context['numeric']['marginalization_absolute_tolerance'],
        'square-root Schur information')
    gradient_error = require_close(
        prior.matrix.T @ prior.target, schur_g,
        context['numeric']['marginalization_absolute_tolerance'],
        'square-root Schur gradient')
    return {
        'full_rank': int(full_rank),
        'prior_rank': int(prior_rank),
        'solution_error': solution_error,
        'information_error': information_error,
        'gradient_error': gradient_error,
    }


def case_FEJ_prior_consistency_and_gauge_preservation(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    require(int(case['relative_chain_state_count']) == 3,
            'FEJ synthetic chain must have three states')
    chain = np.array([[-1.0, 1.0, 0.0], [0.0, -1.0, 1.0]])
    target = np.zeros(2)
    prior = square_root_marginalize(
        chain, target, int(case['marginalize_state_count']))
    gauge = np.asarray(case['gauge_vector'], dtype=np.float64)
    gauge_error = float(np.linalg.norm(prior.matrix @ gauge))
    require(gauge_error <= context['numeric']['marginalization_absolute_tolerance'],
            'marginal prior destroyed translation gauge')
    source_matrix = prior.matrix.copy()
    source_target = prior.target.copy()
    linearization = np.array([0.4, -0.2])
    fej = FEJPrior(source_matrix, source_target, linearization)
    digest_before = fej.payload_sha256()
    residual_first = fej.residual(np.array([0.7, 0.1]))
    source_matrix[:] = 999.0
    source_target[:] = 999.0
    residual_second = fej.residual(np.array([1.7, 1.1]))
    digest_after = fej.payload_sha256()
    require(digest_before == digest_after, 'FEJ prior payload changed')
    require_close(
        residual_second, residual_first,
        context['numeric']['marginalization_absolute_tolerance'],
        'FEJ gauge-shift residual')
    return {
        'gauge_error': gauge_error,
        'prior_rank': int(np.linalg.matrix_rank(prior.matrix)),
        'prior_payload_sha256': digest_before,
        'payload_immutable': True,
    }


def case_fixed_lag_capacity_and_deterministic_eviction(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    rate = float(case['lidar_rate_hz'])
    count = int(round(float(case['duration_sec']) * rate)) + 1
    period_ns = int(round(1e9 / rate))
    lag_ns = int(round(float(case['lag_duration_sec']) * 1e9))
    window = FixedLagWindow(lag_ns, int(case['maximum_active_knots']))
    removed_sequence = []
    for index in range(count):
        removed_sequence.extend(window.add(
            index * period_ns, index, surfel_ids=(index * 10, index * 10 + 1)))
    require(len(window.knots) == int(case['expected_final_active_knots']),
            'fixed-lag final active knot count differs')
    require(len(window.evicted) == int(case['expected_evicted_knots']),
            'fixed-lag eviction count differs')
    require(window.evicted == list(range(len(window.evicted))),
            'fixed-lag eviction order is not deterministic oldest-first')
    require(set(window.surfels_by_knot) == {
        identifier for _, identifier in window.knots},
        'marginalized source surfels remain active')
    return {
        'input_knot_count': count,
        'final_active_knot_count': len(window.knots),
        'evicted_knot_count': len(window.evicted),
        'first_active_knot_id': window.knots[0][1],
        'last_active_knot_id': window.knots[-1][1],
        'eviction_payload_sha256': payload_sha256(removed_sequence),
    }


def case_deterministic_factor_order_and_state_payload_hash(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    factors = [
        {'type': 'lidar_point_to_plane', 'start_ns': 20, 'end_ns': 30, 'id': 8},
        {'type': 'imu_preintegration', 'start_ns': 10, 'end_ns': 20, 'id': 3},
        {'type': 'gauge', 'start_ns': 0, 'end_ns': 0, 'id': 0},
        {'type': 'bias_random_walk', 'start_ns': 10, 'end_ns': 20, 'id': 4},
        {'type': 'marginal_prior', 'start_ns': 10, 'end_ns': 10, 'id': 2},
        {'type': 'lidar_point_to_plane', 'start_ns': 10, 'end_ns': 30, 'id': 7},
    ]
    digests = []
    orders = []
    for offset in case['factor_shuffle_seed_offsets']:
        generator = np.random.default_rng(
            int(context['numeric']['random_seed']) + int(offset))
        shuffled = [factors[index] for index in generator.permutation(len(factors))]
        orders.append([item['id'] for item in deterministic_factor_order(shuffled)])
        digests.append(deterministic_state_factor_digest(
            case['state_timestamp_ns'], case['state_values'],
            case['state_counters'], shuffled))
    require(digests[0] == digests[1], 'factor shuffle changed state payload hash')
    require(orders[0] == orders[1], 'factor shuffle changed factor order')
    return {
        'state_factor_payload_sha256': digests[0],
        'ordered_factor_ids': orders[0],
        'shuffle_repetitions': len(digests),
    }


def case_duplicate_out_of_order_memory_and_protected_output_fail_closed(
        case: dict[str, Any], context: dict[str, Any]) -> dict[str, Any]:
    valid_events = [
        {'stamp_ns': 10, 'type': 'lidar', 'source_index': 2,
         'message_id': 'lidar-2', 'size_bytes': 64},
        {'stamp_ns': 10, 'type': 'imu', 'source_index': 1,
         'message_id': 'imu-1', 'size_bytes': 32},
        {'stamp_ns': 11, 'type': 'imu', 'source_index': 3,
         'message_id': 'imu-3', 'size_bytes': 32},
    ]
    ordered = validate_event_stream(
        valid_events, case['event_type_order'], case['maximum_input_message_bytes'])
    require([item['message_id'] for item in ordered] == [
        'imu-1', 'lidar-2', 'imu-3'], 'equal-time event order differs')
    rejected = {}
    try:
        validate_event_stream([
            dict(valid_events[2]), dict(valid_events[0])],
            case['event_type_order'], case['maximum_input_message_bytes'])
    except ContractError:
        rejected['out_of_order'] = True
    try:
        duplicate = [dict(valid_events[0]), dict(valid_events[0])]
        validate_event_stream(
            duplicate, case['event_type_order'], case['maximum_input_message_bytes'])
    except ContractError:
        rejected['duplicate'] = True
    try:
        reserve_bytes(0, case['overflow_request_bytes'],
                      case['maximum_reserved_bytes'])
    except MemoryBudgetError:
        rejected['reservation_overflow'] = True
    try:
        oversized = [dict(valid_events[0])]
        oversized[0]['size_bytes'] = int(case['maximum_input_message_bytes']) + 1
        validate_event_stream(
            oversized, case['event_type_order'], case['maximum_input_message_bytes'])
    except MemoryBudgetError:
        rejected['message_overflow'] = True
    payload = str(case['protected_payload']).encode('utf-8')
    protected_digest = require_protected_payload_unchanged(payload, bytes(payload))
    try:
        require_protected_payload_unchanged(payload, payload + b'-changed')
    except ContractError:
        rejected['protected_write'] = True
    expected = {
        'out_of_order', 'duplicate', 'reservation_overflow',
        'message_overflow', 'protected_write'}
    require(set(rejected) == expected, 'combined fail-closed challenge is incomplete')
    return {
        'valid_event_order': [item['message_id'] for item in ordered],
        'valid_event_count': len(ordered),
        'rejected_challenges': sorted(rejected),
        'rejected_challenge_count': len(rejected),
        'protected_payload_sha256': protected_digest,
        'bytes_reserved_after_overflow': 0,
        'state_output_count_after_failures': 0,
    }


CASE_RUNNERS: dict[str, Callable[[dict[str, Any], dict[str, Any]], dict[str, Any]]] = {
    'SO3_preintegration_direction': case_SO3_preintegration_direction,
    'constant_motion_zero_residual': case_constant_motion_zero_residual,
    'gravity_sign_and_rebase': case_gravity_sign_and_rebase,
    'gyro_bias_Jacobian_finite_difference':
        case_gyro_bias_Jacobian_finite_difference,
    'accelerometer_bias_Jacobians_finite_difference':
        case_accelerometer_bias_Jacobians_finite_difference,
    'preintegration_covariance_PSD_and_dt_scaling':
        case_preintegration_covariance_PSD_and_dt_scaling,
    'scan_boundary_interpolation': case_scan_boundary_interpolation,
    'timestamp_gap_fail_closed': case_timestamp_gap_fail_closed,
    'point_deskew_direction': case_point_deskew_direction,
    'lidar_to_body_extrinsic_direction': case_lidar_to_body_extrinsic_direction,
    'binary_point_to_plane_Jacobians_finite_difference':
        case_binary_point_to_plane_Jacobians_finite_difference,
    'observable_subspace_rotation_invariance':
        case_observable_subspace_rotation_invariance,
    'weak_axis_information_removal_without_state_clamp':
        case_weak_axis_information_removal_without_state_clamp,
    'dynamic_startup_without_orientation_or_zero_velocity_prior':
        case_dynamic_startup_without_orientation_or_zero_velocity_prior,
    'stationary_startup_uses_same_architecture':
        case_stationary_startup_uses_same_architecture,
    'square_root_marginalization_equivalence':
        case_square_root_marginalization_equivalence,
    'FEJ_prior_consistency_and_gauge_preservation':
        case_FEJ_prior_consistency_and_gauge_preservation,
    'fixed_lag_capacity_and_deterministic_eviction':
        case_fixed_lag_capacity_and_deterministic_eviction,
    'deterministic_factor_order_and_state_payload_hash':
        case_deterministic_factor_order_and_state_payload_hash,
    'duplicate_out_of_order_memory_and_protected_output_fail_closed':
        case_duplicate_out_of_order_memory_and_protected_output_fail_closed,
}


def require_finite_tree(value: Any, label: str = 'payload') -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            require_finite_tree(child, f'{label}.{key}')
    elif isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            require_finite_tree(child, f'{label}[{index}]')
    elif isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        raise ContractError(f'{label} is non-finite')


def require_exact_keys(value: dict[str, Any], expected: Iterable[str],
                       label: str) -> None:
    expected_set = set(expected)
    actual = set(value)
    if actual != expected_set:
        raise ContractError(
            f'{label} keys differ: missing={sorted(expected_set - actual)}, '
            f'extra={sorted(actual - expected_set)}')


def bound_json(path_value: str, expected_sha256: str,
               label: str) -> tuple[Path, dict[str, Any]]:
    path = resolve_path(path_value)
    if not path.is_file():
        raise ContractError(f'{label} does not exist: {path}')
    digest = sha256_file(path)
    if digest != expected_sha256:
        raise ContractError(
            f'{label} hash differs: expected {expected_sha256}, observed {digest}')
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f'{label} is not readable JSON: {path}') from error
    return path, value


def validate_numeric_policy(value: dict[str, Any]) -> None:
    require_exact_keys(value, {
        'floating_point', 'integer_time', 'random_seed',
        'finite_difference_epsilon', 'SO3_absolute_tolerance',
        'zero_residual_absolute_tolerance', 'Jacobian_absolute_tolerance',
        'covariance_negative_eigenvalue_tolerance',
        'marginalization_absolute_tolerance', 'deterministic_serialization',
        'nonfinite_policy',
    }, 'numeric_policy')
    if value['floating_point'] != 'IEEE754_float64':
        raise ContractError('synthetic floating-point policy differs')
    if value['integer_time'] != 'signed_int64_nanoseconds':
        raise ContractError('synthetic clock policy differs')
    if int(value['random_seed']) != 4403001:
        raise ContractError('synthetic random seed differs')
    for key in (
            'finite_difference_epsilon', 'SO3_absolute_tolerance',
            'zero_residual_absolute_tolerance', 'Jacobian_absolute_tolerance',
            'covariance_negative_eigenvalue_tolerance',
            'marginalization_absolute_tolerance'):
        number = float(value[key])
        if not math.isfinite(number) or number <= 0.0:
            raise ContractError(f'numeric_policy.{key} must be positive')
    if value['nonfinite_policy'] != 'terminal_fail_closed':
        raise ContractError('nonfinite values must fail closed')


def validate_prerequisite(contract: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    prerequisite = contract['prerequisite']
    require_exact_keys(prerequisite, {
        'architecture_contract_path', 'architecture_contract_sha256',
        'architecture_aggregate_path', 'architecture_aggregate_sha256',
        'required_architecture_contract_id', 'required_architecture_decision',
        'required_architecture_aggregate_payload_sha256',
    }, 'prerequisite')
    _, architecture = bound_json(
        prerequisite['architecture_contract_path'],
        prerequisite['architecture_contract_sha256'], 'v44b architecture contract')
    _, aggregate = bound_json(
        prerequisite['architecture_aggregate_path'],
        prerequisite['architecture_aggregate_sha256'], 'v44b architecture aggregate')
    expected_id = prerequisite['required_architecture_contract_id']
    expected_decision = prerequisite['required_architecture_decision']
    if architecture.get('contract_id') != expected_id:
        raise ContractError('v44b architecture contract ID differs')
    if aggregate.get('contract_id') != expected_id:
        raise ContractError('v44b aggregate contract ID differs')
    if aggregate.get('contract_sha256') != prerequisite[
            'architecture_contract_sha256']:
        raise ContractError('v44b aggregate does not bind the architecture contract')
    if aggregate.get('aggregate_payload_sha256') != prerequisite[
            'required_architecture_aggregate_payload_sha256']:
        raise ContractError('v44b aggregate payload hash differs')
    if aggregate.get('status') != 'PASS' or aggregate.get('decision') != (
            expected_decision):
        raise ContractError('v44b does not authorize synthetic contracts')
    if aggregate.get('stage3_synthetic_contract_implementation_authorized') is not True:
        raise ContractError('v44b Stage-3 authorization is absent')
    if aggregate.get('fixed_lag_shadow_estimator_implementation_authorized') is not False:
        raise ContractError('v44b prematurely authorizes estimator implementation')
    summary = {
        'architecture_contract_id': expected_id,
        'architecture_contract_sha256': prerequisite['architecture_contract_sha256'],
        'architecture_aggregate_sha256': prerequisite['architecture_aggregate_sha256'],
        'architecture_aggregate_payload_sha256': prerequisite[
            'required_architecture_aggregate_payload_sha256'],
        'architecture_decision': expected_decision,
    }
    return architecture, summary


def validate_cases(contract: dict[str, Any], architecture: dict[str, Any]) -> None:
    cases = contract['cases']
    identifiers = [item.get('id') for item in cases]
    expected = architecture['stage3_synthetic_contracts']
    if identifiers != expected:
        raise ContractError('v44c case order differs from the v44b inventory')
    if len(identifiers) != len(set(identifiers)) or len(identifiers) != 20:
        raise ContractError('v44c requires exactly 20 unique cases')
    if set(identifiers) != set(CASE_RUNNERS):
        raise ContractError('synthetic case implementation inventory differs')
    case_by_id = {item['id']: item for item in cases}
    fixed_lag = case_by_id['fixed_lag_capacity_and_deterministic_eviction']
    if float(fixed_lag['lag_duration_sec']) != float(
            architecture['optimizer']['lag_duration_sec']):
        raise ContractError('fixed-lag synthetic duration differs from architecture')
    if int(fixed_lag['maximum_active_knots']) != int(
            architecture['optimizer']['maximum_active_knots']):
        raise ContractError('fixed-lag synthetic capacity differs from architecture')
    gap = case_by_id['timestamp_gap_fail_closed']
    if int(gap['maximum_gap_ns']) != int(
            architecture['timing']['maximum_imu_gap_ns']):
        raise ContractError('gap challenge differs from architecture')
    weak = case_by_id['weak_axis_information_removal_without_state_clamp']
    if float(weak['minimum_retained_singular_value']) != float(
            architecture['observability']['minimum_whitened_singular_value']):
        raise ContractError('weak-axis singular-value threshold differs')
    state = case_by_id['deterministic_factor_order_and_state_payload_hash']
    if len(state['state_values']) != 19 or len(state['state_counters']) != 4:
        raise ContractError('deterministic state payload dimensions differ')


def validate_resource_and_decision(contract: dict[str, Any]) -> None:
    resources = contract['resource_bounds']
    require_exact_keys(resources, {
        'rss_measurement_scope', 'maximum_rss_mib',
        'maximum_incremental_rss_mib', 'maximum_case_matrix_dimension',
        'maximum_case_payload_bytes', 'maximum_report_bytes',
        'capacity_violation_policy',
    }, 'resource_bounds')
    for key in (
            'maximum_rss_mib', 'maximum_incremental_rss_mib',
            'maximum_case_matrix_dimension',
            'maximum_case_payload_bytes', 'maximum_report_bytes'):
        if float(resources[key]) <= 0.0:
            raise ContractError(f'resource_bounds.{key} must be positive')
    if resources['rss_measurement_scope'] != (
            'absolute_for_standalone_process_incremental_for_preloaded_host'):
        raise ContractError('synthetic RSS measurement scope differs')
    if float(resources['maximum_rss_mib']) > 128.0:
        raise ContractError('synthetic harness RSS ceiling is too permissive')
    if float(resources['maximum_incremental_rss_mib']) > 64.0:
        raise ContractError('synthetic incremental RSS ceiling is too permissive')
    if resources['capacity_violation_policy'] != (
            'terminal_fail_closed_before_allocation_or_write'):
        raise ContractError('synthetic capacity violations must fail closed')
    decision = contract['decision']
    require_exact_keys(decision, {
        'required_validation_repetitions', 'required_passing_contracts',
        'on_pass', 'on_fail',
        'report_only_shadow_estimator_implementation_authorized_on_pass',
        'raw_shadow_replay_authorized_on_pass',
        'accuracy_or_reference_map_inputs_authorized_on_pass',
        'primary_trajectory_or_map_mutation_authorized_on_pass',
    }, 'decision')
    if int(decision['required_validation_repetitions']) != 2:
        raise ContractError('v44c requires two validation repetitions')
    if int(decision['required_passing_contracts']) != 20:
        raise ContractError('v44c requires all 20 contracts')
    if decision['on_pass'] != (
            'AUTHORIZE_V44_STAGE4_REPORT_ONLY_SHADOW_IMPLEMENTATION'):
        raise ContractError('v44c pass decision differs')
    if decision[
            'report_only_shadow_estimator_implementation_authorized_on_pass'] is not True:
        raise ContractError('v44c pass must authorize report-only implementation')
    for key in (
            'raw_shadow_replay_authorized_on_pass',
            'accuracy_or_reference_map_inputs_authorized_on_pass',
            'primary_trajectory_or_map_mutation_authorized_on_pass'):
        if decision[key] is not False:
            raise ContractError(f'v44c decision {key} must remain false')


def resolve_contract_document(path: Path) -> tuple[dict[str, Any], str]:
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f'cannot read v44c contract: {path}') from error
    document_sha = sha256_file(path)
    if 'base_contract' not in document:
        return document, document_sha
    require_exact_keys(document, {
        'schema_version', 'contract_id', 'stage', 'base_contract',
        'rejected_preflight', 'correction',
    }, 'v44c1 correction contract')
    if document['schema_version'] != 1 or document['contract_id'] != (
            'v44c1-fixed-lag-synthetic-contracts-20260810'):
        raise ContractError('v44c1 correction identity differs')
    if document['stage'] != 'synthetic_numerical_validation_only':
        raise ContractError('v44c1 correction stage differs')
    base_binding = document['base_contract']
    require_exact_keys(base_binding, {'path', 'sha256', 'contract_id'},
                       'v44c1 base contract')
    _, base = bound_json(
        base_binding['path'], base_binding['sha256'], 'v44c rejected preflight contract')
    if base.get('contract_id') != base_binding['contract_id'] or base_binding[
            'contract_id'] != 'v44c-fixed-lag-synthetic-contracts-20260810':
        raise ContractError('v44c1 base contract identity differs')
    rejected = document['rejected_preflight']
    require_exact_keys(rejected, {
        'aggregate_path', 'aggregate_sha256', 'aggregate_payload_sha256', 'reason',
    }, 'v44c1 rejected preflight')
    _, preflight = bound_json(
        rejected['aggregate_path'], rejected['aggregate_sha256'],
        'v44c rejected preflight aggregate')
    if preflight.get('contract_sha256') != base_binding['sha256']:
        raise ContractError('rejected preflight does not bind the base contract')
    if preflight.get('aggregate_payload_sha256') != rejected[
            'aggregate_payload_sha256']:
        raise ContractError('rejected preflight payload hash differs')
    correction = document['correction']
    require_exact_keys(correction, {
        'scope', 'performance_or_numerical_threshold_changed',
        'case_seed_or_tolerance_changed', 'resource_bounds_override',
    }, 'v44c1 correction')
    if correction['scope'] != 'resource_measurement_semantics_only':
        raise ContractError('v44c1 correction scope differs')
    if correction['performance_or_numerical_threshold_changed'] is not False:
        raise ContractError('v44c1 cannot change numerical thresholds')
    if correction['case_seed_or_tolerance_changed'] is not False:
        raise ContractError('v44c1 cannot change cases, seeds, or tolerances')
    contract = json.loads(json.dumps(base))
    contract['contract_id'] = document['contract_id']
    contract['rationale'] = (
        base['rationale'] + ' Resource measurement is corrected by the hash-bound '
        'v44c1 overlay: standalone execution retains the absolute ceiling while a '
        'preloaded regression host is charged only incremental RSS.')
    contract['resource_bounds'] = correction['resource_bounds_override']
    return contract, document_sha


def load_and_validate_contract(
        path: Path) -> tuple[dict[str, Any], str, dict[str, Any], dict[str, Any]]:
    contract, contract_sha = resolve_contract_document(path)
    require_finite_tree(contract, 'contract')
    require_exact_keys(contract, {
        'schema_version', 'contract_id', 'stage', 'rationale', 'prerequisite',
        'numeric_policy', 'cases', 'resource_bounds', 'decision',
    }, 'contract')
    if contract['schema_version'] != 1:
        raise ContractError('unsupported v44c schema_version')
    if contract['contract_id'] != 'v44c1-fixed-lag-synthetic-contracts-20260810':
        raise ContractError('v44c1 contract ID differs')
    if contract['stage'] != 'synthetic_numerical_validation_only':
        raise ContractError('v44c must remain synthetic-only')
    serialized = canonical_json(contract).lower()
    for token in ('navinst', 'oxford', 'urbannav'):
        if token in serialized:
            raise ContractError(f'v44c contains dataset branch token {token}')
    validate_numeric_policy(contract['numeric_policy'])
    architecture, prerequisite = validate_prerequisite(contract)
    validate_cases(contract, architecture)
    validate_resource_and_decision(contract)
    return contract, contract_sha, architecture, prerequisite


def to_builtin(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): to_builtin(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_builtin(child) for child in value]
    if isinstance(value, np.ndarray):
        return to_builtin(value.tolist())
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        return float(value)
    return value


def run_all_cases(contract: dict[str, Any], architecture: dict[str, Any],
                  memory: MemoryGuard) -> list[dict[str, Any]]:
    context = {
        'numeric': contract['numeric_policy'],
        'architecture': architecture,
        'noise': architecture['noise_model'],
    }
    results = []
    for case in contract['cases']:
        identifier = case['id']
        memory.check(f'before_{identifier}')
        metrics = to_builtin(CASE_RUNNERS[identifier](case, context))
        require_finite_tree(metrics, identifier)
        result = {
            'id': identifier,
            'status': 'PASS',
            'metrics': metrics,
            'metrics_payload_sha256': payload_sha256(metrics),
        }
        case_bytes = len(canonical_json(result).encode('utf-8'))
        if case_bytes > int(contract['resource_bounds']['maximum_case_payload_bytes']):
            raise MemoryBudgetError(f'{identifier} result exceeds case payload budget')
        results.append(result)
        memory.check(f'after_{identifier}')
    return results


def write_json_bounded(path: Path, payload: dict[str, Any], maximum_bytes: int) -> None:
    encoded = (json.dumps(payload, indent=2, sort_keys=True, allow_nan=False)
               + '\n').encode('utf-8')
    if len(encoded) > int(maximum_bytes):
        raise MemoryBudgetError('report exceeds the frozen output byte budget')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded)


def validate_once(contract_path: Path, repetition: int,
                  output: Path) -> dict[str, Any]:
    started = time.perf_counter()
    contract, contract_sha, architecture, prerequisite = (
        load_and_validate_contract(contract_path))
    memory = MemoryGuard(
        contract['resource_bounds']['maximum_rss_mib'],
        contract['resource_bounds']['maximum_incremental_rss_mib'])
    case_results = run_all_cases(contract, architecture, memory)
    required = int(contract['decision']['required_passing_contracts'])
    all_passed = len(case_results) == required and all(
        item['status'] == 'PASS' for item in case_results)
    require(all_passed, 'not every v44c synthetic contract passed')
    deterministic = {
        'all_synthetic_contracts_passed': True,
        'contract_id': contract['contract_id'],
        'prerequisite': prerequisite,
        'case_count': len(case_results),
        'case_order': [item['id'] for item in case_results],
        'case_results': case_results,
        'authorizations_on_aggregate_pass': {
            'report_only_shadow_estimator_implementation': True,
            'raw_shadow_replay': False,
            'accuracy_or_reference_map_inputs': False,
            'primary_trajectory_or_map_mutation': False,
        },
    }
    report = {
        'schema_version': 1,
        'audit': 'v44c_fixed_lag_synthetic_numerical_validation',
        'status': 'PASS',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'implementation_sha256': sha256_file(Path(__file__)),
        'repetition': int(repetition),
        'deterministic': deterministic,
        'deterministic_payload_sha256': payload_sha256(deterministic),
        'runtime': {
            'wall_seconds': time.perf_counter() - started,
            'baseline_rss_mib': memory.baseline_rss_mib,
            'peak_rss_mib': memory.peak_rss_mib,
            'peak_incremental_rss_mib': memory.peak_incremental_rss_mib,
            'maximum_rss_mib': memory.maximum_rss_mib,
            'maximum_incremental_rss_mib': memory.maximum_incremental_rss_mib,
            'absolute_ceiling_enforced': memory.absolute_ceiling_enforced,
        },
    }
    write_json_bounded(
        output, report, contract['resource_bounds']['maximum_report_bytes'])
    return report


def aggregate_reports(contract_path: Path, reports: list[Path], output: Path,
                      markdown_output: Path | None = None) -> dict[str, Any]:
    contract, contract_sha, architecture, prerequisite = (
        load_and_validate_contract(contract_path))
    required = int(contract['decision']['required_validation_repetitions'])
    loaded: list[tuple[Path, dict[str, Any]]] = []
    for path in reports:
        try:
            report = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as error:
            raise ContractError(f'cannot read v44c report: {path}') from error
        loaded.append((path.resolve(), report))
    repetitions = sorted(item.get('repetition') for _, item in loaded)
    complete = len(loaded) == required and repetitions == list(range(1, required + 1))
    valid = complete
    implementations: set[str] = set()
    deterministic_hashes: set[str] = set()
    deterministic_payloads: list[dict[str, Any]] = []
    for _, report in loaded:
        deterministic = report.get('deterministic')
        report_valid = (
            report.get('status') == 'PASS'
            and report.get('contract_id') == contract['contract_id']
            and report.get('contract_sha256') == contract_sha
            and isinstance(deterministic, dict)
            and report.get('deterministic_payload_sha256') ==
                payload_sha256(deterministic)
            and deterministic.get('all_synthetic_contracts_passed') is True
            and deterministic.get('case_count') == int(
                contract['decision']['required_passing_contracts']))
        valid = valid and report_valid
        implementations.add(str(report.get('implementation_sha256')))
        deterministic_hashes.add(str(report.get('deterministic_payload_sha256')))
        if isinstance(deterministic, dict):
            deterministic_payloads.append(deterministic)
    repeatable = (
        valid and len(implementations) == 1 and len(deterministic_hashes) == 1
        and len(deterministic_payloads) == required
        and all(item == deterministic_payloads[0]
                for item in deterministic_payloads[1:]))
    passed = bool(complete and valid and repeatable)
    deterministic = {
        'synthetic_contracts_validated': passed,
        'validation_complete': complete,
        'validation_repeatable': repeatable,
        'validation_repetition_count': len(loaded),
        'passing_contract_count': (
            deterministic_payloads[0]['case_count']
            if deterministic_payloads else 0),
        'case_payload_sha256': (
            payload_sha256(deterministic_payloads[0]['case_results'])
            if deterministic_payloads else None),
        'prerequisite': prerequisite,
        'architecture_contract_sha256': sha256_file(resolve_path(
            contract['prerequisite']['architecture_contract_path'])),
    }
    result = {
        'schema_version': 1,
        'audit': 'v44c_fixed_lag_synthetic_numerical_aggregate',
        'status': 'PASS' if passed else 'FAIL',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'implementation_sha256': (
            next(iter(implementations)) if len(implementations) == 1 else None),
        'decision': (
            contract['decision']['on_pass'] if passed
            else contract['decision']['on_fail']),
        'report_only_shadow_estimator_implementation_authorized': passed,
        'raw_shadow_replay_authorized': False,
        'accuracy_or_reference_map_inputs_authorized': False,
        'primary_trajectory_or_map_mutation_authorized': False,
        'deterministic': deterministic,
        'aggregate_payload_sha256': payload_sha256(deterministic),
        'source_reports': [
            {'path': str(path), 'sha256': sha256_file(path)}
            for path, _ in loaded
        ],
    }
    write_json_bounded(
        output, result, contract['resource_bounds']['maximum_report_bytes'])
    if markdown_output is not None:
        lines = [
            '# v44c synthetic numerical validation', '',
            f'- Status: `{result["status"]}`',
            f'- Decision: `{result["decision"]}`',
            f'- Passing contracts: `{deterministic["passing_contract_count"]}/20`',
            f'- Repeatable validations: `{len(loaded)}/{required}`',
            '- Report-only shadow implementation authorized: '
            f'`{str(passed).lower()}`',
            '- Raw shadow replay authorized: `false`',
            '- Accuracy/reference-map inputs authorized: `false`',
            '',
        ]
        encoded = '\n'.join(lines).encode('utf-8')
        if len(encoded) > int(contract['resource_bounds']['maximum_report_bytes']):
            raise MemoryBudgetError('markdown report exceeds output byte budget')
        markdown_output.parent.mkdir(parents=True, exist_ok=True)
        markdown_output.write_bytes(encoded)
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    validate = subparsers.add_parser('validate')
    validate.add_argument('--contract', required=True, type=Path)
    validate.add_argument('--repetition', required=True, type=int)
    validate.add_argument('--output', required=True, type=Path)
    aggregate = subparsers.add_parser('aggregate')
    aggregate.add_argument('--contract', required=True, type=Path)
    aggregate.add_argument('--report', required=True, action='append', type=Path)
    aggregate.add_argument('--output', required=True, type=Path)
    aggregate.add_argument('--markdown-output', type=Path)
    return parser


def main() -> int:
    arguments = build_parser().parse_args()
    if arguments.command == 'validate':
        validate_once(arguments.contract, arguments.repetition, arguments.output)
    else:
        aggregate_reports(
            arguments.contract, arguments.report, arguments.output,
            arguments.markdown_output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
