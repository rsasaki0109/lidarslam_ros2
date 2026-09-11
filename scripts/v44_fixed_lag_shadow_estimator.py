#!/usr/bin/env python3
"""Isolated report-only fixed-lag LiDAR/IMU shadow estimator core.

This module intentionally has no raw-bag, ROS, filesystem, network, wall-clock,
accuracy, map, or publication adapter.  A later, separately authorized runtime
adapter may decode sealed messages into the immutable input records below and
may serialize the returned diagnostics.  The estimator itself only consumes
sensor time and returns in-memory report data.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import struct
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


ARCHITECTURE_CONTRACT_ID = 'v44b-fixed-lag-shadow-architecture-20260810'
SYNTHETIC_CONTRACT_ID = 'v44c1-fixed-lag-synthetic-contracts-20260810'
SOURCE_STAGE = 'report_only_shadow_estimator_core'
STATE_DOF = 15
GRAVITY_MAGNITUDE_M_S2 = 9.80665
FACTOR_TYPE_ORDER = {
    'gauge': 0,
    'marginal_prior': 1,
    'imu_preintegration': 2,
    'bias_random_walk': 3,
    'lidar_point_to_plane': 4,
}
SOURCE_AUTHORITY = {
    'raw_shadow_replay': False,
    'accuracy_or_reference_map_inputs': False,
    'primary_trajectory_or_map_mutation': False,
    'ros_publication': False,
    'filesystem_output': False,
}


class ContractViolation(ValueError):
    """An input or algorithm invariant differs from the frozen architecture."""


class BoundaryCoverageViolation(ContractViolation):
    """A scan boundary lacks the specifically permitted startup coverage."""


class CapacityViolation(RuntimeError):
    """A bounded allocation or output would exceed the frozen capacity."""


class EstimatorTerminalFailure(RuntimeError):
    """The shadow estimator has entered a terminal fail-closed state."""


def canonical_json(value: Any) -> str:
    return json.dumps(
        value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def payload_sha256(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode('utf-8')).hexdigest()


def require(condition: bool, message: str) -> None:
    if not bool(condition):
        raise ContractViolation(message)


def finite_vector(value: Iterable[float], size: int, label: str) -> np.ndarray:
    result = np.asarray(tuple(value), dtype=np.float64)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ContractViolation(f'{label} must be one finite {size}-vector')
    return result


def skew(vector: Iterable[float]) -> np.ndarray:
    x, y, z = finite_vector(vector, 3, 'skew input')
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ], dtype=np.float64)


def vee(matrix: np.ndarray) -> np.ndarray:
    value = np.asarray(matrix, dtype=np.float64)
    if value.shape != (3, 3) or not np.all(np.isfinite(value)):
        raise ContractViolation('vee input must be one finite 3x3 matrix')
    return 0.5 * np.array([
        value[2, 1] - value[1, 2],
        value[0, 2] - value[2, 0],
        value[1, 0] - value[0, 1],
    ], dtype=np.float64)


def validate_rotation(rotation: np.ndarray, tolerance: float = 1e-8) -> None:
    value = np.asarray(rotation, dtype=np.float64)
    if value.shape != (3, 3) or not np.all(np.isfinite(value)):
        raise ContractViolation('rotation must be one finite 3x3 matrix')
    if not np.allclose(value.T @ value, np.eye(3), atol=tolerance, rtol=0.0):
        raise ContractViolation('rotation is not orthonormal')
    if not math.isclose(
            float(np.linalg.det(value)), 1.0, abs_tol=tolerance, rel_tol=0.0):
        raise ContractViolation('rotation determinant differs from one')


def so3_exp(rotation_vector: Iterable[float]) -> np.ndarray:
    vector = finite_vector(rotation_vector, 3, 'SO3 exponential input')
    theta_squared = float(vector @ vector)
    matrix = skew(vector)
    if theta_squared < 1e-16:
        first = 1.0 - theta_squared / 6.0 + theta_squared ** 2 / 120.0
        second = 0.5 - theta_squared / 24.0 + theta_squared ** 2 / 720.0
    else:
        theta = math.sqrt(theta_squared)
        first = math.sin(theta) / theta
        second = (1.0 - math.cos(theta)) / theta_squared
    return np.eye(3) + first * matrix + second * (matrix @ matrix)


def so3_log(rotation: np.ndarray) -> np.ndarray:
    value = np.asarray(rotation, dtype=np.float64)
    validate_rotation(value)
    cosine = float(np.clip((np.trace(value) - 1.0) * 0.5, -1.0, 1.0))
    theta = math.acos(cosine)
    antisymmetric = vee(value - value.T)
    if theta < 1e-8:
        return 0.5 * antisymmetric
    sine = math.sin(theta)
    if abs(sine) < 1e-10:
        raise ContractViolation('SO3 logarithm near pi is outside the core domain')
    return antisymmetric * (theta / (2.0 * sine))


def rotation_from_two_vectors(
        source: Iterable[float], target: Iterable[float]) -> np.ndarray:
    first = finite_vector(source, 3, 'alignment source')
    second = finite_vector(target, 3, 'alignment target')
    first_norm = float(np.linalg.norm(first))
    second_norm = float(np.linalg.norm(second))
    if min(first_norm, second_norm) <= 1e-12:
        raise ContractViolation('cannot align a zero vector')
    first = first / first_norm
    second = second / second_norm
    cross = np.cross(first, second)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(first @ second, -1.0, 1.0))
    if sine <= 1e-12:
        if cosine > 0.0:
            return np.eye(3)
        basis = np.zeros(3)
        basis[int(np.argmin(np.abs(first)))] = 1.0
        axis = np.cross(first, basis)
        axis /= np.linalg.norm(axis)
        return so3_exp(axis * math.pi)
    return so3_exp((cross / sine) * math.atan2(sine, cosine))


def rotation_to_quaternion_xyzw(rotation: np.ndarray) -> tuple[float, ...]:
    value = np.asarray(rotation, dtype=np.float64)
    validate_rotation(value)
    trace = float(np.trace(value))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (value[2, 1] - value[1, 2]) / scale,
            (value[0, 2] - value[2, 0]) / scale,
            (value[1, 0] - value[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        axis = int(np.argmax(np.diag(value)))
        if axis == 0:
            scale = math.sqrt(1.0 + value[0, 0] - value[1, 1] - value[2, 2]) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (value[0, 1] + value[1, 0]) / scale,
                (value[0, 2] + value[2, 0]) / scale,
                (value[2, 1] - value[1, 2]) / scale,
            ])
        elif axis == 1:
            scale = math.sqrt(1.0 + value[1, 1] - value[0, 0] - value[2, 2]) * 2.0
            quaternion = np.array([
                (value[0, 1] + value[1, 0]) / scale,
                0.25 * scale,
                (value[1, 2] + value[2, 1]) / scale,
                (value[0, 2] - value[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(1.0 + value[2, 2] - value[0, 0] - value[1, 1]) * 2.0
            quaternion = np.array([
                (value[0, 2] + value[2, 0]) / scale,
                (value[1, 2] + value[2, 1]) / scale,
                0.25 * scale,
                (value[1, 0] - value[0, 1]) / scale,
            ])
    quaternion /= np.linalg.norm(quaternion)
    if quaternion[3] < 0.0:
        quaternion *= -1.0
    return tuple(float(item) for item in quaternion)


@dataclass(frozen=True)
class ImuSample:
    timestamp_ns: int
    angular_velocity_B_rad_s: tuple[float, float, float]
    linear_acceleration_B_m_s2: tuple[float, float, float]
    source_index: int
    serialized_size_bytes: int

    def __post_init__(self) -> None:
        require(-(2 ** 63) <= int(self.timestamp_ns) < 2 ** 63,
                'IMU timestamp is outside signed int64')
        finite_vector(self.angular_velocity_B_rad_s, 3, 'IMU angular velocity')
        finite_vector(self.linear_acceleration_B_m_s2, 3, 'IMU acceleration')
        require(int(self.source_index) >= 0, 'IMU source index must be nonnegative')
        require(int(self.serialized_size_bytes) > 0,
                'IMU serialized size must be positive')


@dataclass(frozen=True)
class LidarPoint:
    point_L_m: tuple[float, float, float]
    offset_ns: int
    ring: int
    source_index: int

    def __post_init__(self) -> None:
        finite_vector(self.point_L_m, 3, 'LiDAR point')
        require(0 <= int(self.offset_ns) <= 0xffffffff,
                'LiDAR point offset must be uint32 nanoseconds')
        require(int(self.ring) >= 0, 'LiDAR ring must be nonnegative')
        require(int(self.source_index) >= 0,
                'LiDAR point source index must be nonnegative')


@dataclass(frozen=True)
class LidarScan:
    scan_index: int
    header_stamp_ns: int
    points: tuple[LidarPoint, ...]
    source_index: int
    serialized_size_bytes: int

    def __post_init__(self) -> None:
        require(int(self.scan_index) >= 0, 'scan index must be nonnegative')
        require(-(2 ** 63) <= int(self.header_stamp_ns) < 2 ** 63,
                'scan header timestamp is outside signed int64')
        require(bool(self.points), 'LiDAR scan must contain at least one point')
        require(all(isinstance(item, LidarPoint) for item in self.points),
                'LiDAR scan contains an invalid point record')
        require(int(self.source_index) >= 0,
                'LiDAR source index must be nonnegative')
        require(int(self.serialized_size_bytes) > 0,
                'LiDAR serialized size must be positive')
        require(self.end_ns < 2 ** 63, 'scan end timestamp exceeds signed int64')

    @property
    def end_ns(self) -> int:
        return int(self.header_stamp_ns) + max(
            int(item.offset_ns) for item in self.points)


@dataclass(frozen=True)
class BodyFromLidar:
    rotation_BL: tuple[tuple[float, float, float], ...]
    translation_BL_m: tuple[float, float, float]

    def arrays(self) -> tuple[np.ndarray, np.ndarray]:
        rotation = np.asarray(self.rotation_BL, dtype=np.float64)
        validate_rotation(rotation)
        translation = finite_vector(
            self.translation_BL_m, 3, 'LiDAR-to-body translation')
        return rotation, translation


@dataclass(frozen=True)
class FixedLagShadowConfig:
    lag_ns: int
    maximum_active_knots: int
    maximum_iterations: int
    line_search_scales: tuple[float, ...]
    maximum_imu_gap_ns: int
    maximum_boundary_distance_ns: int
    minimum_imu_samples_per_scan: int
    maximum_prefix_drops: int
    maximum_suffix_drops: int
    bootstrap_minimum_ns: int
    bootstrap_maximum_ns: int
    bootstrap_minimum_scans: int
    bootstrap_minimum_imu: int
    bootstrap_velocity_prior_std_m_s: float
    bootstrap_gyro_bias_prior_std_rad_s: float
    bootstrap_accel_bias_prior_std_m_s2: float
    bootstrap_gravity_direction_prior_std_rad: float
    gyro_noise: float
    accel_noise: float
    gyro_bias_walk: float
    accel_bias_walk: float
    lidar_sigma_m: float
    lidar_huber_delta_sigma: float
    minimum_lidar_singular_value: float
    maximum_lidar_condition_number: float
    minimum_range_m: float
    maximum_range_m: float
    voxel_size_m: float
    maximum_selected_points_per_scan: int
    maximum_source_keyframes_per_scan: int
    maximum_active_surfels: int
    minimum_correspondences: int
    maximum_correspondence_distance_m: float
    maximum_rss_mib: float
    maximum_processing_rtf: float
    maximum_input_message_bytes: int
    maximum_state_dimension: int
    maximum_active_correspondences: int
    maximum_materialized_rows: int
    streaming_block_rows: int
    maximum_dense_solver_bytes: int
    maximum_diagnostic_output_bytes: int
    covariance_floor: float
    covariance_negative_tolerance: float
    gyro_reintegration_threshold: float
    accel_reintegration_threshold: float

    @classmethod
    def from_architecture(
            cls, architecture: Mapping[str, Any]) -> 'FixedLagShadowConfig':
        require(architecture.get('contract_id') == ARCHITECTURE_CONTRACT_ID,
                'architecture contract ID differs')
        timing = architecture['timing']
        initialization = architecture['initialization']
        noise = architecture['noise_model']
        lidar = architecture['lidar_factor']
        observability = architecture['observability']
        optimizer = architecture['optimizer']
        preintegration = architecture['preintegration']
        resources = architecture['resource_bounds']
        diagnostics = architecture['diagnostics']
        governance = architecture['governance']
        require(governance['configuration_policy'] == 'one_global_algorithm_contract',
                'source requires one global algorithm contract')
        require(governance['dataset_identity_available_to_algorithm'] is False,
                'dataset identity must be unavailable')
        require(diagnostics['ros_publishers_allowed'] is False,
                'ROS publication must remain forbidden')
        require(diagnostics['primary_output_paths_allowed'] is False,
                'primary output paths must remain forbidden')
        require(timing['wall_clock_used_for_estimator_time'] is False,
                'wall clock must not be estimator time')
        require(optimizer['thread_count'] == 1,
                'optimizer must remain single-threaded')
        require(optimizer['wall_clock_stopping_allowed'] is False,
                'wall-clock stopping must remain forbidden')
        require(architecture['state_model']['local_dof_per_knot'] == STATE_DOF,
                'state local dimension differs from 15')
        require(preintegration['required_bias_jacobians'] == [
            'J_R_bg', 'J_v_bg', 'J_v_ba', 'J_p_bg', 'J_p_ba'],
            'preintegration Jacobian inventory differs')
        require(lidar['global_or_persistent_map_allowed'] is False,
                'persistent map must remain forbidden')
        require(lidar['loop_candidate_search_allowed'] is False,
                'loop search must remain forbidden')
        require(observability['direct_bias_or_velocity_update_allowed'] is False,
                'direct bias or velocity updates must remain forbidden')
        return cls(
            lag_ns=int(round(float(optimizer['lag_duration_sec']) * 1e9)),
            maximum_active_knots=int(optimizer['maximum_active_knots']),
            maximum_iterations=int(optimizer['maximum_iterations_per_update']),
            line_search_scales=tuple(float(item) for item in
                                     optimizer['line_search_step_scales']),
            maximum_imu_gap_ns=int(timing['maximum_imu_gap_ns']),
            maximum_boundary_distance_ns=int(
                timing['maximum_imu_boundary_bracket_distance_ns']),
            minimum_imu_samples_per_scan=int(
                timing['minimum_imu_samples_per_scan']),
            maximum_prefix_drops=int(
                timing['maximum_dropped_unbracketed_prefix_scans']),
            maximum_suffix_drops=int(
                timing['maximum_dropped_unbracketed_suffix_scans']),
            bootstrap_minimum_ns=int(round(
                float(initialization['minimum_window_sec']) * 1e9)),
            bootstrap_maximum_ns=int(round(
                float(initialization['maximum_window_sec']) * 1e9)),
            bootstrap_minimum_scans=int(initialization['minimum_lidar_scans']),
            bootstrap_minimum_imu=int(initialization['minimum_imu_messages']),
            bootstrap_velocity_prior_std_m_s=float(
                noise['bootstrap_prior_std']['velocity_m_s']),
            bootstrap_gyro_bias_prior_std_rad_s=float(
                noise['bootstrap_prior_std']['gyro_bias_rad_s']),
            bootstrap_accel_bias_prior_std_m_s2=float(
                noise['bootstrap_prior_std']['accel_bias_m_s2']),
            bootstrap_gravity_direction_prior_std_rad=math.radians(float(
                noise['bootstrap_prior_std']['gravity_direction_deg'])),
            gyro_noise=float(
                noise['gyroscope_white_noise_density_rad_s_sqrt_hz']),
            accel_noise=float(
                noise['accelerometer_white_noise_density_m_s2_sqrt_hz']),
            gyro_bias_walk=float(
                noise['gyroscope_bias_random_walk_density_rad_s2_sqrt_hz']),
            accel_bias_walk=float(
                noise['accelerometer_bias_random_walk_density_m_s3_sqrt_hz']),
            lidar_sigma_m=float(noise['lidar_point_to_plane_sigma_m']),
            lidar_huber_delta_sigma=float(noise['lidar_huber_delta_sigma']),
            minimum_lidar_singular_value=float(
                observability['minimum_whitened_singular_value']),
            maximum_lidar_condition_number=float(
                observability['maximum_retained_condition_number']),
            minimum_range_m=float(lidar['minimum_range_m']),
            maximum_range_m=float(lidar['maximum_range_m']),
            voxel_size_m=float(lidar['voxel_size_m']),
            maximum_selected_points_per_scan=int(
                lidar['maximum_selected_points_per_scan']),
            maximum_source_keyframes_per_scan=int(
                lidar['maximum_source_keyframes_per_scan']),
            maximum_active_surfels=int(lidar['maximum_active_surfels']),
            minimum_correspondences=int(lidar['minimum_correspondences']),
            maximum_correspondence_distance_m=float(
                lidar['maximum_correspondence_distance_m']),
            maximum_rss_mib=float(resources['maximum_rss_mib']),
            maximum_processing_rtf=float(resources['maximum_processing_rtf']),
            maximum_input_message_bytes=int(
                resources['maximum_input_message_bytes']),
            maximum_state_dimension=int(
                resources['maximum_active_state_dimension']),
            maximum_active_correspondences=int(
                resources['maximum_active_lidar_correspondences']),
            maximum_materialized_rows=int(
                resources['maximum_materialized_jacobian_rows']),
            streaming_block_rows=int(
                resources['streaming_linearization_block_rows']),
            maximum_dense_solver_bytes=int(
                resources['maximum_dense_solver_bytes']),
            maximum_diagnostic_output_bytes=int(
                resources['maximum_diagnostic_output_bytes']),
            covariance_floor=float(noise['covariance_eigenvalue_floor']),
            covariance_negative_tolerance=float(
                noise['covariance_negative_eigenvalue_tolerance']),
            gyro_reintegration_threshold=float(
                preintegration['bias_reintegration_threshold']['gyro_rad_s']),
            accel_reintegration_threshold=float(
                preintegration['bias_reintegration_threshold']['accel_m_s2']),
        )


@dataclass(frozen=True)
class PreintegratedImu:
    delta_R: np.ndarray
    delta_v: np.ndarray
    delta_p: np.ndarray
    covariance: np.ndarray
    duration_sec: float
    bias_gyro_reference: np.ndarray
    bias_accel_reference: np.ndarray
    J_R_bg: np.ndarray
    J_v_bg: np.ndarray
    J_v_ba: np.ndarray
    J_p_bg: np.ndarray
    J_p_ba: np.ndarray


def _preintegrate_base(
        times_ns: np.ndarray, gyro: np.ndarray, accel: np.ndarray,
        bias_gyro: np.ndarray, bias_accel: np.ndarray,
        config: FixedLagShadowConfig) -> tuple[np.ndarray, np.ndarray, np.ndarray,
                                                    np.ndarray, float]:
    times = np.asarray(times_ns, dtype=np.int64)
    angular = np.asarray(gyro, dtype=np.float64)
    linear = np.asarray(accel, dtype=np.float64)
    bg = finite_vector(bias_gyro, 3, 'gyroscope bias')
    ba = finite_vector(bias_accel, 3, 'accelerometer bias')
    if times.ndim != 1 or len(times) < 2:
        raise ContractViolation('preintegration requires at least two samples')
    if angular.shape != (len(times), 3) or linear.shape != (len(times), 3):
        raise ContractViolation('preintegration array dimensions differ')
    differences = np.diff(times)
    if np.any(differences <= 0) or np.any(
            differences > config.maximum_imu_gap_ns):
        raise ContractViolation('preintegration timestamp gap violates contract')
    if not np.all(np.isfinite(angular)) or not np.all(np.isfinite(linear)):
        raise ContractViolation('preintegration measurement is non-finite')
    delta_R = np.eye(3)
    delta_v = np.zeros(3)
    delta_p = np.zeros(3)
    covariance = np.zeros((9, 9))
    continuous_noise = np.diag([
        config.gyro_noise ** 2, config.gyro_noise ** 2,
        config.gyro_noise ** 2, config.accel_noise ** 2,
        config.accel_noise ** 2, config.accel_noise ** 2,
    ])
    for index, difference_ns in enumerate(differences):
        dt = float(difference_ns) * 1e-9
        omega = 0.5 * (angular[index] + angular[index + 1]) - bg
        accel_first = linear[index] - ba
        accel_second = linear[index + 1] - ba
        next_R = delta_R @ so3_exp(omega * dt)
        midpoint_accel = 0.5 * (
            delta_R @ accel_first + next_R @ accel_second)
        previous_v = delta_v.copy()
        delta_p += previous_v * dt + 0.5 * midpoint_accel * dt * dt
        delta_v += midpoint_accel * dt
        transition = np.eye(9)
        transition[0:3, 0:3] = so3_exp(-omega * dt)
        transition[3:6, 0:3] = -skew(midpoint_accel) * dt
        transition[6:9, 0:3] = -0.5 * skew(midpoint_accel) * dt * dt
        transition[6:9, 3:6] = np.eye(3) * dt
        rotation_midpoint = delta_R @ so3_exp(omega * 0.5 * dt)
        noise_map = np.zeros((9, 6))
        noise_map[0:3, 0:3] = -np.eye(3)
        noise_map[3:6, 3:6] = rotation_midpoint
        noise_map[6:9, 3:6] = 0.5 * rotation_midpoint * dt
        covariance = transition @ covariance @ transition.T + (
            noise_map @ continuous_noise @ noise_map.T * dt)
        covariance = 0.5 * (covariance + covariance.T)
        delta_R = next_R
    eigenvalues = np.linalg.eigvalsh(covariance)
    if float(eigenvalues[0]) < -config.covariance_negative_tolerance:
        raise ContractViolation('preintegration covariance is not PSD')
    return delta_R, delta_v, delta_p, covariance, (
        float(times[-1] - times[0]) * 1e-9)


def preintegrate_midpoint(
        times_ns: np.ndarray, gyro: np.ndarray, accel: np.ndarray,
        bias_gyro: Iterable[float], bias_accel: Iterable[float],
        config: FixedLagShadowConfig, epsilon: float = 1e-6,
        ) -> PreintegratedImu:
    bg = finite_vector(bias_gyro, 3, 'gyroscope bias')
    ba = finite_vector(bias_accel, 3, 'accelerometer bias')
    require(float(epsilon) > 0.0, 'bias Jacobian epsilon must be positive')
    base = _preintegrate_base(times_ns, gyro, accel, bg, ba, config)
    J_R_bg = np.zeros((3, 3))
    J_v_bg = np.zeros((3, 3))
    J_v_ba = np.zeros((3, 3))
    J_p_bg = np.zeros((3, 3))
    J_p_ba = np.zeros((3, 3))
    for axis in range(3):
        perturbation = np.zeros(3)
        perturbation[axis] = epsilon
        gyro_plus = _preintegrate_base(
            times_ns, gyro, accel, bg + perturbation, ba, config)
        gyro_minus = _preintegrate_base(
            times_ns, gyro, accel, bg - perturbation, ba, config)
        J_R_bg[:, axis] = (
            so3_log(base[0].T @ gyro_plus[0])
            - so3_log(base[0].T @ gyro_minus[0])) / (2.0 * epsilon)
        J_v_bg[:, axis] = (gyro_plus[1] - gyro_minus[1]) / (2.0 * epsilon)
        J_p_bg[:, axis] = (gyro_plus[2] - gyro_minus[2]) / (2.0 * epsilon)
        accel_plus = _preintegrate_base(
            times_ns, gyro, accel, bg, ba + perturbation, config)
        accel_minus = _preintegrate_base(
            times_ns, gyro, accel, bg, ba - perturbation, config)
        J_v_ba[:, axis] = (accel_plus[1] - accel_minus[1]) / (2.0 * epsilon)
        J_p_ba[:, axis] = (accel_plus[2] - accel_minus[2]) / (2.0 * epsilon)
    return PreintegratedImu(
        delta_R=base[0], delta_v=base[1], delta_p=base[2],
        covariance=base[3], duration_sec=base[4],
        bias_gyro_reference=bg.copy(), bias_accel_reference=ba.copy(),
        J_R_bg=J_R_bg, J_v_bg=J_v_bg, J_v_ba=J_v_ba,
        J_p_bg=J_p_bg, J_p_ba=J_p_ba)


def corrected_preintegration(
        value: PreintegratedImu, bias_gyro: Iterable[float],
        bias_accel: Iterable[float]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    delta_bg = finite_vector(bias_gyro, 3, 'gyroscope bias') - (
        value.bias_gyro_reference)
    delta_ba = finite_vector(bias_accel, 3, 'accelerometer bias') - (
        value.bias_accel_reference)
    return (
        value.delta_R @ so3_exp(value.J_R_bg @ delta_bg),
        value.delta_v + value.J_v_bg @ delta_bg + value.J_v_ba @ delta_ba,
        value.delta_p + value.J_p_bg @ delta_bg + value.J_p_ba @ delta_ba,
    )


class ImuBuffer:
    """Stable, exactly-once sensor-time IMU buffer with boundary interpolation."""

    def __init__(self, config: FixedLagShadowConfig) -> None:
        self.config = config
        self.samples: list[ImuSample] = []
        self._last_key: tuple[int, int] | None = None

    def add(self, sample: ImuSample) -> None:
        key = (int(sample.timestamp_ns), int(sample.source_index))
        if self._last_key is not None and key <= self._last_key:
            raise ContractViolation('IMU samples are out of stable sensor-time order')
        if sample.serialized_size_bytes > self.config.maximum_input_message_bytes:
            raise CapacityViolation('IMU message exceeds input byte capacity')
        self.samples.append(sample)
        self._last_key = key

    @property
    def latest_timestamp_ns(self) -> int | None:
        return None if not self.samples else int(self.samples[-1].timestamp_ns)

    def count_between(self, start_ns: int, end_ns: int) -> int:
        return sum(start_ns <= item.timestamp_ns <= end_ns for item in self.samples)

    def _unique_arrays(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.samples:
            return (np.empty(0, dtype=np.int64), np.empty((0, 3)),
                    np.empty((0, 3)))
        unique: list[ImuSample] = []
        for sample in self.samples:
            if unique and sample.timestamp_ns == unique[-1].timestamp_ns:
                unique[-1] = sample
            else:
                unique.append(sample)
        return (
            np.asarray([item.timestamp_ns for item in unique], dtype=np.int64),
            np.asarray([item.angular_velocity_B_rad_s for item in unique],
                       dtype=np.float64),
            np.asarray([item.linear_acceleration_B_m_s2 for item in unique],
                       dtype=np.float64),
        )

    def interval(
            self, start_ns: int, end_ns: int,
            minimum_samples: int | None = None,
            ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        start = int(start_ns)
        end = int(end_ns)
        if end <= start:
            raise ContractViolation('IMU interval must have positive duration')
        times, gyro, accel = self._unique_arrays()
        if len(times) < 2 or start < int(times[0]) or end > int(times[-1]):
            raise BoundaryCoverageViolation('IMU interval is not bracketed')
        start_right = int(np.searchsorted(times, start, side='left'))
        end_right = int(np.searchsorted(times, end, side='left'))
        start_left = start_right if (
            start_right < len(times) and int(times[start_right]) == start
        ) else start_right - 1
        end_left = end_right if (
            end_right < len(times) and int(times[end_right]) == end
        ) else end_right - 1
        boundary_indices = (start_left, start_right, end_left, end_right)
        if min(boundary_indices) < 0 or max(boundary_indices) >= len(times):
            raise BoundaryCoverageViolation('IMU boundary bracket is incomplete')
        boundary_distances = (
            start - int(times[start_left]), int(times[start_right]) - start,
            end - int(times[end_left]), int(times[end_right]) - end,
        )
        if max(boundary_distances) > self.config.maximum_boundary_distance_ns:
            raise BoundaryCoverageViolation(
                'IMU boundary bracket distance exceeds contract')

        def interpolate(values: np.ndarray, query: int) -> np.ndarray:
            right = int(np.searchsorted(times, query, side='left'))
            if right < len(times) and int(times[right]) == query:
                return values[right].copy()
            left = right - 1
            alpha = (query - int(times[left])) / (
                int(times[right]) - int(times[left]))
            return values[left] + alpha * (values[right] - values[left])

        internal = times[(times > start) & (times < end)]
        output_times = np.concatenate((
            np.array([start], dtype=np.int64), internal,
            np.array([end], dtype=np.int64)))
        output_gyro = np.vstack([
            interpolate(gyro, int(stamp)) for stamp in output_times])
        output_accel = np.vstack([
            interpolate(accel, int(stamp)) for stamp in output_times])
        required = self.config.minimum_imu_samples_per_scan if (
            minimum_samples is None) else int(minimum_samples)
        if len(output_times) < required:
            raise ContractViolation('IMU interval has too few samples')
        if np.any(np.diff(output_times) > self.config.maximum_imu_gap_ns):
            raise ContractViolation('IMU interval contains an oversized gap')
        return output_times, output_gyro, output_accel

    def prune_before(self, timestamp_ns: int) -> None:
        threshold = int(timestamp_ns)
        keep_from = 0
        for index, sample in enumerate(self.samples):
            if sample.timestamp_ns < threshold:
                keep_from = max(0, index)
            else:
                break
        if keep_from:
            self.samples = self.samples[keep_from:]


@dataclass
class Knot:
    knot_id: int
    scan_index: int
    timestamp_ns: int
    R_WB: np.ndarray
    p_WB: np.ndarray
    v_WB: np.ndarray
    b_g: np.ndarray
    b_a: np.ndarray

    def copy(self) -> 'Knot':
        return Knot(
            self.knot_id, self.scan_index, self.timestamp_ns,
            self.R_WB.copy(), self.p_WB.copy(), self.v_WB.copy(),
            self.b_g.copy(), self.b_a.copy())

    def validate(self) -> None:
        validate_rotation(self.R_WB)
        for value, label in (
                (self.p_WB, 'position'), (self.v_WB, 'velocity'),
                (self.b_g, 'gyroscope bias'),
                (self.b_a, 'accelerometer bias')):
            finite_vector(value, 3, label)

    def apply_delta(self, delta: np.ndarray) -> None:
        value = finite_vector(delta, STATE_DOF, 'state increment')
        self.R_WB = self.R_WB @ so3_exp(value[0:3])
        self.p_WB = self.p_WB + value[3:6]
        self.v_WB = self.v_WB + value[6:9]
        self.b_g = self.b_g + value[9:12]
        self.b_a = self.b_a + value[12:15]
        self.validate()

    def diagnostic_state(self) -> dict[str, list[float]]:
        return {
            'state_R_WB_xyzw': list(rotation_to_quaternion_xyzw(self.R_WB)),
            'state_p_WB_m': [float(item) for item in self.p_WB],
            'state_v_WB_m_s': [float(item) for item in self.v_WB],
            'state_b_g_rad_s': [float(item) for item in self.b_g],
            'state_b_a_m_s2': [float(item) for item in self.b_a],
        }


@dataclass(frozen=True)
class FrozenKnot:
    knot_id: int
    scan_index: int
    timestamp_ns: int
    R_WB: np.ndarray
    p_WB: np.ndarray
    v_WB: np.ndarray
    b_g: np.ndarray
    b_a: np.ndarray

    @classmethod
    def from_knot(cls, knot: Knot) -> 'FrozenKnot':
        arrays = [
            np.array(value, dtype='<f8', copy=True)
            for value in (knot.R_WB, knot.p_WB, knot.v_WB, knot.b_g, knot.b_a)]
        for value in arrays:
            value.setflags(write=False)
        return cls(
            knot_id=int(knot.knot_id), scan_index=int(knot.scan_index),
            timestamp_ns=int(knot.timestamp_ns), R_WB=arrays[0],
            p_WB=arrays[1], v_WB=arrays[2], b_g=arrays[3], b_a=arrays[4])


@dataclass(frozen=True)
class ImuFactor:
    start_knot_id: int
    end_knot_id: int
    times_ns: np.ndarray
    gyro: np.ndarray
    accel: np.ndarray
    preintegrated: PreintegratedImu


@dataclass(frozen=True)
class Surfel:
    voxel: tuple[int, int, int]
    mean_B: tuple[float, float, float]
    normal_B: tuple[float, float, float]
    support: int


@dataclass(frozen=True)
class PreparedScan:
    scan: LidarScan
    selected_points_B: np.ndarray
    imu_times_ns: np.ndarray
    imu_gyro: np.ndarray
    imu_accel: np.ndarray
    maximum_imu_gap_ns: int


@dataclass(frozen=True)
class LidarBatch:
    source_knot_id: int
    current_knot_id: int
    residual: np.ndarray
    source_jacobian: np.ndarray
    current_jacobian: np.ndarray
    relative_jacobian: np.ndarray
    singular_values: np.ndarray
    retained_left: np.ndarray
    rank: int


class FirstEstimatePrior:
    """Immutable square-root separator prior at first-estimate Jacobians."""

    def __init__(
            self, knot_ids: Sequence[int], matrix: np.ndarray,
            target: np.ndarray, snapshots: Sequence[Knot]) -> None:
        self.knot_ids = tuple(int(item) for item in knot_ids)
        self.matrix = np.array(matrix, dtype='<f8', copy=True)
        self.target = np.array(target, dtype='<f8', copy=True)
        self.snapshots = tuple(FrozenKnot.from_knot(item) for item in snapshots)
        if self.matrix.ndim != 2 or self.matrix.shape[1] != (
                len(self.knot_ids) * STATE_DOF):
            raise ContractViolation('FEJ prior matrix dimension differs')
        if self.target.shape != (self.matrix.shape[0],):
            raise ContractViolation('FEJ prior target dimension differs')
        if tuple(item.knot_id for item in self.snapshots) != self.knot_ids:
            raise ContractViolation('FEJ prior snapshots differ from knot IDs')
        if not np.all(np.isfinite(self.matrix)) or not np.all(
                np.isfinite(self.target)):
            raise ContractViolation('FEJ prior is non-finite')
        self.matrix.setflags(write=False)
        self.target.setflags(write=False)

    def payload_sha256(self) -> str:
        digest = hashlib.sha256()
        digest.update(np.asarray(self.knot_ids, dtype='<i8').tobytes())
        digest.update(self.matrix.tobytes(order='C'))
        digest.update(self.target.tobytes(order='C'))
        for knot in self.snapshots:
            digest.update(state_payload_bytes(knot, (0, 0, 0, 0)))
        return digest.hexdigest()


def state_delta(reference: Knot | FrozenKnot, value: Knot) -> np.ndarray:
    require(reference.knot_id == value.knot_id, 'state delta knot ID differs')
    return np.concatenate((
        so3_log(reference.R_WB.T @ value.R_WB),
        value.p_WB - reference.p_WB,
        value.v_WB - reference.v_WB,
        value.b_g - reference.b_g,
        value.b_a - reference.b_a,
    ))


def state_payload_bytes(
        knot: Knot | FrozenKnot, counters: Iterable[int]) -> bytes:
    quaternion = rotation_to_quaternion_xyzw(knot.R_WB)
    values = tuple(quaternion) + tuple(knot.p_WB) + tuple(knot.v_WB) + (
        tuple(knot.b_g) + tuple(knot.b_a) + (0.0, 0.0, -GRAVITY_MAGNITUDE_M_S2))
    counter_values = tuple(int(item) for item in counters)
    return (struct.pack('<q', int(knot.timestamp_ns))
            + np.asarray(values, dtype='<f8').tobytes(order='C')
            + np.asarray(counter_values, dtype='<i8').tobytes(order='C'))


def robust_acceleration_mean(values: np.ndarray) -> np.ndarray:
    samples = np.asarray(values, dtype=np.float64)
    if samples.ndim != 2 or samples.shape[1] != 3 or len(samples) < 3:
        raise ContractViolation('gravity seed requires finite 3D samples')
    if not np.all(np.isfinite(samples)):
        raise ContractViolation('gravity seed samples are non-finite')
    median = np.median(samples, axis=0)
    distances = np.linalg.norm(samples - median, axis=1)
    distance_median = float(np.median(distances))
    mad = float(np.median(np.abs(distances - distance_median)))
    threshold = distance_median + max(1e-9, 3.0 * 1.4826 * mad)
    retained = samples[distances <= threshold]
    if len(retained) < 3:
        retained = samples
    result = np.mean(retained, axis=0, dtype=np.float64)
    if float(np.linalg.norm(result)) <= 1e-9:
        raise ContractViolation('gravity seed is zero')
    return result


def gravity_tangent_basis(gravity_W: Iterable[float]) -> np.ndarray:
    gravity = finite_vector(gravity_W, 3, 'gravity direction')
    norm = float(np.linalg.norm(gravity))
    if norm <= 1e-12:
        raise ContractViolation('gravity direction is zero')
    direction = gravity / norm
    seed = np.zeros(3)
    seed[int(np.argmin(np.abs(direction)))] = 1.0
    first = np.cross(direction, seed)
    first /= np.linalg.norm(first)
    second = np.cross(direction, first)
    second /= np.linalg.norm(second)
    return np.column_stack((first, second))


def perturb_gravity_direction(
        gravity_W: Iterable[float], tangent_delta: Iterable[float]) -> np.ndarray:
    gravity = finite_vector(gravity_W, 3, 'gravity direction')
    delta = finite_vector(tangent_delta, 2, 'gravity tangent increment')
    basis = gravity_tangent_basis(gravity)
    result = so3_exp(basis @ delta) @ gravity
    return result * (GRAVITY_MAGNITUDE_M_S2 / np.linalg.norm(result))


def gravity_direction_delta(
        reference_W: Iterable[float], value_W: Iterable[float]) -> np.ndarray:
    reference = finite_vector(reference_W, 3, 'gravity reference')
    value = finite_vector(value_W, 3, 'gravity value')
    reference *= GRAVITY_MAGNITUDE_M_S2 / np.linalg.norm(reference)
    value *= GRAVITY_MAGNITUDE_M_S2 / np.linalg.norm(value)
    alignment = rotation_from_two_vectors(reference, value)
    return gravity_tangent_basis(reference).T @ so3_log(alignment)


def integrate_scan_trajectory(
        times_ns: np.ndarray, gyro: np.ndarray, accel: np.ndarray,
        query_offsets_ns: Iterable[int], initial_velocity_B_m_s: Iterable[float],
        gravity_B_m_s2: Iterable[float], bias_gyro: Iterable[float],
        bias_accel: Iterable[float], config: FixedLagShadowConfig,
        ) -> dict[int, tuple[np.ndarray, np.ndarray]]:
    """Integrate exact queried point times in the scan-start body frame."""
    times = np.asarray(times_ns, dtype=np.int64)
    angular = np.asarray(gyro, dtype=np.float64)
    linear = np.asarray(accel, dtype=np.float64)
    if times.ndim != 1 or len(times) < 2 or np.any(np.diff(times) <= 0):
        raise ContractViolation('scan trajectory requires increasing IMU time')
    if angular.shape != (len(times), 3) or linear.shape != (len(times), 3):
        raise ContractViolation('scan trajectory IMU dimensions differ')
    if np.any(np.diff(times) > config.maximum_imu_gap_ns):
        raise ContractViolation('scan trajectory contains an oversized IMU gap')
    velocity = finite_vector(
        initial_velocity_B_m_s, 3, 'scan-start local velocity').copy()
    gravity = finite_vector(gravity_B_m_s2, 3, 'scan-start local gravity')
    bg = finite_vector(bias_gyro, 3, 'scan gyroscope bias')
    ba = finite_vector(bias_accel, 3, 'scan accelerometer bias')
    duration_ns = int(times[-1] - times[0])
    offsets = sorted(set(int(item) for item in query_offsets_ns))
    if not offsets or offsets[0] < 0 or offsets[-1] > duration_ns:
        raise ContractViolation('scan trajectory query is outside the interval')
    query_times = np.asarray(
        [int(times[0]) + item for item in offsets], dtype=np.int64)
    merged_times = np.unique(np.concatenate((times, query_times)))

    def interpolate(values: np.ndarray) -> np.ndarray:
        output = np.empty((len(merged_times), 3), dtype=np.float64)
        for index, stamp in enumerate(merged_times):
            right = int(np.searchsorted(times, stamp, side='left'))
            if right < len(times) and int(times[right]) == int(stamp):
                output[index] = values[right]
            else:
                left = right - 1
                alpha = (int(stamp) - int(times[left])) / (
                    int(times[right]) - int(times[left]))
                output[index] = values[left] + alpha * (
                    values[right] - values[left])
        return output

    merged_gyro = interpolate(angular)
    merged_accel = interpolate(linear)
    rotation = np.eye(3)
    position = np.zeros(3)
    query_set = set(int(item) for item in query_times)
    poses: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    if int(merged_times[0]) in query_set:
        poses[0] = (rotation.copy(), position.copy())
    for index, difference_ns in enumerate(np.diff(merged_times)):
        dt = float(difference_ns) * 1e-9
        omega = 0.5 * (merged_gyro[index] + merged_gyro[index + 1]) - bg
        next_rotation = rotation @ so3_exp(omega * dt)
        specific_force = 0.5 * (
            rotation @ (merged_accel[index] - ba)
            + next_rotation @ (merged_accel[index + 1] - ba))
        acceleration_local = specific_force + gravity
        previous_velocity = velocity.copy()
        position += previous_velocity * dt + 0.5 * acceleration_local * dt * dt
        velocity += acceleration_local * dt
        rotation = next_rotation
        stamp = int(merged_times[index + 1])
        if stamp in query_set:
            poses[stamp - int(times[0])] = (rotation.copy(), position.copy())
    if set(poses) != set(offsets):
        raise ContractViolation('scan trajectory did not produce every point time')
    return poses


def deterministic_voxel_select(
        scan: LidarScan, body_from_lidar: BodyFromLidar,
        config: FixedLagShadowConfig,
        relative_rotation_at_end: np.ndarray,
        relative_translation_at_end: np.ndarray,
        relative_poses_by_offset: Mapping[
            int, tuple[np.ndarray, np.ndarray]] | None = None,
        ) -> np.ndarray:
    rotation_BL, translation_BL = body_from_lidar.arrays()
    end_offset = max(int(item.offset_ns) for item in scan.points)
    end_rotation_inverse = relative_rotation_at_end.T
    selected: dict[tuple[int, int, int], tuple[tuple[int, int, int], np.ndarray]] = {}
    for point in scan.points:
        point_L = np.asarray(point.point_L_m, dtype=np.float64)
        distance = float(np.linalg.norm(point_L))
        if distance < config.minimum_range_m or distance > config.maximum_range_m:
            continue
        if relative_poses_by_offset is None:
            alpha = 0.0 if end_offset == 0 else float(point.offset_ns) / end_offset
            rotation_at_point = so3_exp(
                so3_log(relative_rotation_at_end) * alpha)
            translation_at_point = relative_translation_at_end * alpha
        else:
            pose = relative_poses_by_offset.get(int(point.offset_ns))
            if pose is None:
                raise ContractViolation('point trajectory pose is absent')
            rotation_at_point = np.asarray(pose[0], dtype=np.float64)
            translation_at_point = finite_vector(
                pose[1], 3, 'point trajectory translation')
            validate_rotation(rotation_at_point)
        point_B = rotation_BL @ point_L + translation_BL
        point_at_end = end_rotation_inverse @ (
            rotation_at_point @ point_B + translation_at_point
            - relative_translation_at_end)
        voxel = tuple(int(item) for item in np.floor(
            point_at_end / config.voxel_size_m).astype(np.int64))
        representative_key = (
            int(point.offset_ns), int(point.ring), int(point.source_index))
        previous = selected.get(voxel)
        if previous is None or representative_key < previous[0]:
            selected[voxel] = (representative_key, point_at_end)
    ordered = [selected[key][1] for key in sorted(selected)]
    if len(ordered) > config.maximum_selected_points_per_scan:
        ordered = ordered[:config.maximum_selected_points_per_scan]
    if not ordered:
        raise ContractViolation('LiDAR scan has no point after deterministic filtering')
    return np.asarray(ordered, dtype=np.float64)


def build_surfels(points_B: np.ndarray, voxel_size_m: float) -> dict[
        tuple[int, int, int], Surfel]:
    points = np.asarray(points_B, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise ContractViolation('surfel input must be finite Nx3 points')
    buckets: dict[tuple[int, int, int], list[np.ndarray]] = {}
    for point in points:
        key = tuple(int(item) for item in np.floor(
            point / float(voxel_size_m)).astype(np.int64))
        buckets.setdefault(key, []).append(point)
    surfels: dict[tuple[int, int, int], Surfel] = {}
    for key in sorted(buckets):
        local = np.asarray(buckets[key], dtype=np.float64)
        if len(local) < 3:
            neighbours = []
            for offset_x in (-1, 0, 1):
                for offset_y in (-1, 0, 1):
                    for offset_z in (-1, 0, 1):
                        neighbour = (
                            key[0] + offset_x, key[1] + offset_y,
                            key[2] + offset_z)
                        neighbours.extend(buckets.get(neighbour, ()))
            local = np.asarray(neighbours, dtype=np.float64)
        if len(local) < 3:
            continue
        mean = np.mean(local, axis=0, dtype=np.float64)
        centered = local - mean
        covariance = centered.T @ centered / max(1, len(local) - 1)
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        if not np.all(np.isfinite(eigenvalues)) or eigenvalues[-1] <= 1e-12:
            continue
        normal = eigenvectors[:, 0]
        dominant = int(np.argmax(np.abs(normal)))
        if normal[dominant] < 0.0:
            normal *= -1.0
        surfels[key] = Surfel(
            voxel=key, mean_B=tuple(float(item) for item in mean),
            normal_B=tuple(float(item) for item in normal), support=len(local))
    return surfels


def lidar_residual_and_jacobians(
        source: Knot, current: Knot, point_current_B: np.ndarray,
        surfel: Surfel) -> tuple[float, np.ndarray, np.ndarray]:
    point = finite_vector(point_current_B, 3, 'current LiDAR point')
    mean = finite_vector(surfel.mean_B, 3, 'surfel mean')
    normal = finite_vector(surfel.normal_B, 3, 'surfel normal')
    normal /= np.linalg.norm(normal)
    world_point = current.R_WB @ point + current.p_WB
    source_vector = source.R_WB.T @ (world_point - source.p_WB)
    residual = float(normal @ (source_vector - mean))
    source_jacobian = np.concatenate((
        normal @ skew(source_vector), -normal @ source.R_WB.T))
    current_jacobian = np.concatenate((
        normal @ source.R_WB.T @ (-current.R_WB @ skew(point)),
        normal @ source.R_WB.T))
    return residual, source_jacobian, current_jacobian


def _neighbour_voxels(point: np.ndarray, voxel_size_m: float) -> list[
        tuple[int, int, int]]:
    centre = np.floor(point / voxel_size_m).astype(np.int64)
    return [
        (int(centre[0] + x), int(centre[1] + y), int(centre[2] + z))
        for x in (-1, 0, 1)
        for y in (-1, 0, 1)
        for z in (-1, 0, 1)
    ]


def build_lidar_batches(
        knots: Sequence[Knot], points_by_knot: Mapping[int, np.ndarray],
        surfels_by_knot: Mapping[int, Mapping[tuple[int, int, int], Surfel]],
        config: FixedLagShadowConfig) -> tuple[list[LidarBatch], int]:
    knot_by_id = {item.knot_id: item for item in knots}
    batches_raw: dict[tuple[int, int], list[tuple[
        float, np.ndarray, np.ndarray]]] = {}
    correspondence_count = 0
    for current_index, current in enumerate(knots):
        sources = list(knots[max(
            0, current_index - config.maximum_source_keyframes_per_scan
        ):current_index])
        if not sources:
            continue
        for point in points_by_knot.get(current.knot_id, np.empty((0, 3))):
            world_point = current.R_WB @ point + current.p_WB
            best: tuple[float, int, Surfel] | None = None
            for source in reversed(sources):
                point_source = source.R_WB.T @ (world_point - source.p_WB)
                for voxel in _neighbour_voxels(
                        point_source, config.voxel_size_m):
                    surfel = surfels_by_knot.get(source.knot_id, {}).get(voxel)
                    if surfel is None:
                        continue
                    distance = float(np.linalg.norm(
                        point_source - np.asarray(surfel.mean_B)))
                    candidate = (distance, source.knot_id, surfel)
                    if best is None or (candidate[0], candidate[1], candidate[2].voxel) < (
                            best[0], best[1], best[2].voxel):
                        best = candidate
            if best is None or best[0] > config.maximum_correspondence_distance_m:
                continue
            source = knot_by_id[best[1]]
            residual, source_jacobian, current_jacobian = (
                lidar_residual_and_jacobians(source, current, point, best[2]))
            whitened = residual / config.lidar_sigma_m
            absolute = abs(whitened)
            weight = 1.0 if absolute <= config.lidar_huber_delta_sigma else (
                math.sqrt(config.lidar_huber_delta_sigma / absolute))
            batches_raw.setdefault((source.knot_id, current.knot_id), []).append((
                whitened * weight,
                source_jacobian * weight / config.lidar_sigma_m,
                current_jacobian * weight / config.lidar_sigma_m))
            correspondence_count += 1
            if correspondence_count > config.maximum_active_correspondences:
                raise CapacityViolation('active correspondence capacity exceeded')
    batches: list[LidarBatch] = []
    for key in sorted(batches_raw, key=lambda pair: (pair[1], pair[0])):
        entries = batches_raw[key]
        if len(entries) < config.minimum_correspondences:
            continue
        residual = np.asarray([item[0] for item in entries], dtype=np.float64)
        source_jacobian = np.asarray([item[1] for item in entries], dtype=np.float64)
        current_jacobian = np.asarray([item[2] for item in entries], dtype=np.float64)
        relative_jacobian = current_jacobian.copy()
        left, singular_values, _ = np.linalg.svd(
            relative_jacobian, full_matrices=False)
        maximum = float(singular_values[0]) if len(singular_values) else 0.0
        threshold = max(
            config.minimum_lidar_singular_value,
            maximum / config.maximum_lidar_condition_number)
        retained = singular_values >= threshold
        rank = int(np.count_nonzero(retained))
        retained_left = left[:, retained]
        batches.append(LidarBatch(
            source_knot_id=key[0], current_knot_id=key[1],
            residual=residual, source_jacobian=source_jacobian,
            current_jacobian=current_jacobian,
            relative_jacobian=relative_jacobian,
            singular_values=singular_values,
            retained_left=retained_left, rank=rank))
    return batches, correspondence_count


class StreamingHouseholderSystem:
    """Bounded block Householder QR followed by rank-revealing SVD of R."""

    def __init__(
            self, dimension: int, config: FixedLagShadowConfig,
            additional_global_dof: int = 0) -> None:
        self.dimension = int(dimension)
        require(self.dimension > 0, 'linear-system dimension must be positive')
        additional = int(additional_global_dof)
        if additional < 0 or self.dimension > (
                config.maximum_state_dimension + additional):
            raise CapacityViolation('active state dimension exceeds contract')
        self.config = config
        self.augmented_R = np.empty((0, self.dimension + 1), dtype=np.float64)
        self.total_rows = 0
        self.cost = 0.0

    def add_local(
            self, jacobian: np.ndarray, residual: np.ndarray,
            columns: Sequence[int]) -> None:
        local = np.asarray(jacobian, dtype=np.float64)
        value = np.asarray(residual, dtype=np.float64)
        column_indices = tuple(int(item) for item in columns)
        if local.ndim != 2 or value.shape != (local.shape[0],):
            raise ContractViolation('factor block dimensions differ')
        if local.shape[1] != len(column_indices):
            raise ContractViolation('factor block column map differs')
        if len(set(column_indices)) != len(column_indices):
            raise ContractViolation('factor block repeats a global column')
        if column_indices and (min(column_indices) < 0 or max(column_indices) >= (
                self.dimension)):
            raise ContractViolation('factor block column is outside state')
        if not np.all(np.isfinite(local)) or not np.all(np.isfinite(value)):
            raise ContractViolation('factor block is non-finite')
        if local.shape[0] > self.config.maximum_materialized_rows:
            raise CapacityViolation('materialized Jacobian row capacity exceeded')
        safe_rows = int(
            self.config.maximum_dense_solver_bytes
            // (8 * (self.dimension + 1)) - min(
                self.dimension, len(self.augmented_R)))
        block_rows = min(
            self.config.streaming_block_rows,
            self.config.maximum_materialized_rows, max(1, safe_rows))
        for start in range(0, len(local), block_rows):
            stop = min(len(local), start + block_rows)
            global_block = np.zeros(
                (stop - start, self.dimension + 1), dtype=np.float64)
            if column_indices:
                global_block[:, column_indices] = local[start:stop]
            global_block[:, -1] = -value[start:stop]
            combined_bytes = (
                self.augmented_R.nbytes + global_block.nbytes)
            if combined_bytes > self.config.maximum_dense_solver_bytes:
                raise CapacityViolation('dense solver storage capacity exceeded')
            combined = np.vstack((self.augmented_R, global_block))
            _, reduced = np.linalg.qr(combined, mode='reduced')
            self.augmented_R = reduced[:self.dimension]
            self.total_rows += stop - start
            self.cost += float(value[start:stop] @ value[start:stop])

    def solve(self) -> tuple[np.ndarray, int, np.ndarray]:
        if not len(self.augmented_R):
            raise ContractViolation('cannot solve an empty factor system')
        matrix = self.augmented_R[:, :self.dimension]
        target = self.augmented_R[:, self.dimension]
        _, singular_values, right = np.linalg.svd(matrix, full_matrices=False)
        maximum = float(singular_values[0]) if len(singular_values) else 0.0
        threshold = max(
            self.config.minimum_lidar_singular_value,
            maximum / self.config.maximum_lidar_condition_number)
        retained = singular_values >= threshold
        update = np.zeros(self.dimension)
        if np.any(retained):
            left_projection = matrix @ right[retained].T
            coefficients, _, _, _ = np.linalg.lstsq(
                left_projection, target, rcond=None)
            update = right[retained].T @ coefficients
        if not np.all(np.isfinite(update)):
            raise ContractViolation('linear solve produced a non-finite update')
        return update, int(np.count_nonzero(retained)), singular_values


def _imu_residual(first: Knot, second: Knot, factor: ImuFactor,
                  gravity_W: np.ndarray) -> np.ndarray:
    corrected_R, corrected_v, corrected_p = corrected_preintegration(
        factor.preintegrated, first.b_g, first.b_a)
    dt = factor.preintegrated.duration_sec
    return np.concatenate((
        so3_log(corrected_R.T @ first.R_WB.T @ second.R_WB),
        first.R_WB.T @ (second.v_WB - first.v_WB - gravity_W * dt)
        - corrected_v,
        first.R_WB.T @ (
            second.p_WB - first.p_WB - first.v_WB * dt
            - 0.5 * gravity_W * dt * dt) - corrected_p,
        second.b_g - first.b_g,
        second.b_a - first.b_a,
    ))


def _numerical_binary_state_jacobian(
        residual_function: Any, first: Knot, second: Knot,
        epsilon: float = 1e-6) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    base = np.asarray(residual_function(first, second), dtype=np.float64)
    first_jacobian = np.zeros((len(base), STATE_DOF))
    second_jacobian = np.zeros((len(base), STATE_DOF))
    for axis in range(STATE_DOF):
        delta = np.zeros(STATE_DOF)
        delta[axis] = epsilon
        plus = first.copy()
        minus = first.copy()
        plus.apply_delta(delta)
        minus.apply_delta(-delta)
        first_jacobian[:, axis] = (
            residual_function(plus, second) - residual_function(minus, second)
        ) / (2.0 * epsilon)
        plus = second.copy()
        minus = second.copy()
        plus.apply_delta(delta)
        minus.apply_delta(-delta)
        second_jacobian[:, axis] = (
            residual_function(first, plus) - residual_function(first, minus)
        ) / (2.0 * epsilon)
    return base, first_jacobian, second_jacobian


def square_root_separator_prior(
        matrix: np.ndarray, target: np.ndarray, marginal_dimension: int,
        config: FixedLagShadowConfig) -> tuple[np.ndarray, np.ndarray, int,
                                                   np.ndarray]:
    value = np.asarray(matrix, dtype=np.float64)
    right_hand_side = np.asarray(target, dtype=np.float64)
    if value.ndim != 2 or right_hand_side.shape != (value.shape[0],):
        raise ContractViolation('marginalization dimensions differ')
    if not 0 < int(marginal_dimension) < value.shape[1]:
        raise ContractViolation('marginalization dimension is outside system')
    marginal = value[:, :marginal_dimension]
    retained = value[:, marginal_dimension:]
    complete_left, triangular = np.linalg.qr(marginal, mode='complete')
    singular_values = np.linalg.svd(
        triangular[:marginal_dimension], compute_uv=False)
    maximum = float(singular_values[0]) if len(singular_values) else 0.0
    threshold = max(
        config.minimum_lidar_singular_value,
        maximum / config.maximum_lidar_condition_number)
    rank = int(np.count_nonzero(singular_values >= threshold))
    null_left = complete_left[:, rank:]
    prior_matrix = null_left.T @ retained
    prior_target = null_left.T @ right_hand_side
    if len(prior_matrix):
        _, reduced = np.linalg.qr(
            np.column_stack((prior_matrix, prior_target)), mode='reduced')
        prior_matrix = reduced[:, :-1]
        prior_target = reduced[:, -1]
    return prior_matrix, prior_target, rank, singular_values


class ProtectedOutputGuard:
    """Compares externally supplied digests without reading protected paths."""

    def __init__(self, before: Mapping[str, str]) -> None:
        self.before = dict(sorted((str(key), str(value)) for key, value in before.items()))
        require(bool(self.before), 'protected-output digest set must be nonempty')
        require(all(len(value) == 64 for value in self.before.values()),
                'protected-output digest must be SHA-256')

    def verify(self, after: Mapping[str, str]) -> str:
        value = dict(sorted((str(key), str(item)) for key, item in after.items()))
        if value != self.before:
            raise ContractViolation('protected output identity changed')
        return payload_sha256(self.before)


class FixedLagShadowEstimator:
    """One-way local estimator core; all externally visible data is diagnostic."""

    def __init__(
            self, config: FixedLagShadowConfig,
            body_from_lidar: BodyFromLidar,
            protected_output_guard: ProtectedOutputGuard) -> None:
        self.config = config
        self.body_from_lidar = body_from_lidar
        self.body_from_lidar.arrays()
        self.protected_output_guard = protected_output_guard
        self.gravity_W = np.array([0.0, 0.0, -GRAVITY_MAGNITUDE_M_S2])
        self.imu = ImuBuffer(config)
        self.pending_scans: list[LidarScan] = []
        self.bootstrap_scans: list[PreparedScan] = []
        self.knots: list[Knot] = []
        self.points_by_knot: dict[int, np.ndarray] = {}
        self.surfels_by_knot: dict[int, dict[tuple[int, int, int], Surfel]] = {}
        self.imu_factors: dict[tuple[int, int], ImuFactor] = {}
        self.prior: FirstEstimatePrior | None = None
        self.diagnostics: list[dict[str, Any]] = []
        self._diagnostic_bytes = 0
        self._last_event_key: tuple[int, int, int] | None = None
        self._seen_scan_indices: set[int] = set()
        self._recorded_scan_indices: set[int] = set()
        self._next_knot_id = 0
        self._prefix_drops = 0
        self._suffix_drops = 0
        self._marginalized_count = 0
        self._latest_prior_rank = 0
        self._latest_prior_hash: str | None = None
        self._latest_bias_information_rank = 0
        self._latest_bias_information_singular_values: list[float] = []
        self._bootstrap_gravity_jointly_optimized = False
        self._terminal_reason: str | None = None
        self._finalized = False
        self._runtime_observations: dict[int, tuple[float, float]] = {}

    @property
    def failed(self) -> bool:
        return self._terminal_reason is not None

    @property
    def bootstrapped(self) -> bool:
        return bool(self.knots)

    def _check_live(self) -> None:
        if self._finalized:
            raise EstimatorTerminalFailure('estimator is already finalized')
        if self.failed:
            raise EstimatorTerminalFailure(self._terminal_reason or 'terminal failure')

    def _consume_event_key(self, timestamp_ns: int, kind_order: int,
                           source_index: int) -> None:
        key = (int(timestamp_ns), int(kind_order), int(source_index))
        if self._last_event_key is not None and key <= self._last_event_key:
            raise ContractViolation('sensor event is duplicate or out of order')
        self._last_event_key = key

    def consume_imu(self, sample: ImuSample) -> None:
        self._check_live()
        try:
            self._consume_event_key(sample.timestamp_ns, 0, sample.source_index)
            self.imu.add(sample)
            self._process_ready_scans()
        except (ContractViolation, CapacityViolation) as error:
            self._fail(str(error))
            raise EstimatorTerminalFailure(str(error)) from error

    def consume_lidar(self, scan: LidarScan) -> None:
        self._check_live()
        try:
            self._consume_event_key(scan.header_stamp_ns, 1, scan.source_index)
            if scan.scan_index in self._seen_scan_indices:
                raise ContractViolation('LiDAR scan index is duplicated')
            self._seen_scan_indices.add(scan.scan_index)
            if scan.serialized_size_bytes > self.config.maximum_input_message_bytes:
                raise CapacityViolation('LiDAR message exceeds input byte capacity')
            if self.pending_scans and scan.end_ns <= self.pending_scans[-1].end_ns:
                raise ContractViolation('LiDAR scan ends are not strictly increasing')
            self.pending_scans.append(scan)
            self._process_ready_scans()
        except (ContractViolation, CapacityViolation) as error:
            self._fail(str(error), scan)
            raise EstimatorTerminalFailure(str(error)) from error

    def record_runtime_observation(
            self, scan_index: int, rss_mib: float,
            processing_seconds: float, sensor_duration_seconds: float) -> None:
        self._check_live()
        try:
            values = (float(rss_mib), float(processing_seconds),
                      float(sensor_duration_seconds))
            if not all(math.isfinite(item) and item >= 0.0 for item in values):
                raise ContractViolation('runtime observation is invalid')
            if values[2] <= 0.0:
                raise ContractViolation('sensor duration must be positive')
            rtf = values[1] / values[2]
            if values[0] > self.config.maximum_rss_mib:
                raise CapacityViolation('RSS capacity exceeded after scan')
            if rtf > self.config.maximum_processing_rtf:
                raise CapacityViolation('processing RTF capacity exceeded after scan')
            index = int(scan_index)
            if index in self._runtime_observations:
                raise ContractViolation('runtime observation is duplicated')
            self._runtime_observations[index] = (values[0], rtf)
            for record_index, record in enumerate(self.diagnostics):
                if (record.get('record_type') == 'scan'
                        and record.get('scan_index') == index):
                    replacement = dict(record)
                    replacement['rss_mib'] = values[0]
                    replacement['processing_rtf'] = rtf
                    old_size = len(canonical_json(record).encode('utf-8')) + 1
                    new_size = len(canonical_json(replacement).encode('utf-8')) + 1
                    if new_size > self.config.maximum_diagnostic_output_bytes - (
                            self._diagnostic_bytes - old_size):
                        raise CapacityViolation(
                            'diagnostic output byte capacity exceeded')
                    self.diagnostics[record_index] = replacement
                    self._diagnostic_bytes += new_size - old_size
                    break
        except (ContractViolation, CapacityViolation) as error:
            self._fail(str(error))
            raise EstimatorTerminalFailure(str(error)) from error

    def _process_ready_scans(self) -> None:
        latest = self.imu.latest_timestamp_ns
        while self.pending_scans and latest is not None and (
                latest > self.pending_scans[0].end_ns):
            scan = self.pending_scans.pop(0)
            self._process_scan(scan)

    def _prepare_scan(self, scan: LidarScan) -> PreparedScan:
        times, gyro, accel = self.imu.interval(
            scan.header_stamp_ns, scan.end_ns)
        bias_gyro = np.zeros(3)
        bias_accel = np.zeros(3)
        initial_velocity_B = np.zeros(3)
        if self.knots:
            previous = self.knots[-1]
            bias_gyro = previous.b_g.copy()
            bias_accel = previous.b_a.copy()
            if scan.header_stamp_ns == previous.timestamp_ns:
                rotation_start = previous.R_WB
                velocity_start = previous.v_WB
            elif scan.header_stamp_ns > previous.timestamp_ns:
                start_times, start_gyro, start_accel = self.imu.interval(
                    previous.timestamp_ns, scan.header_stamp_ns,
                    minimum_samples=2)
                start_integration = preintegrate_midpoint(
                    start_times, start_gyro, start_accel,
                    bias_gyro, bias_accel, self.config)
                corrected_R, corrected_v, _ = corrected_preintegration(
                    start_integration, bias_gyro, bias_accel)
                dt = start_integration.duration_sec
                rotation_start = previous.R_WB @ corrected_R
                velocity_start = (
                    previous.v_WB + self.gravity_W * dt
                    + previous.R_WB @ corrected_v)
            else:
                overlap_times, overlap_gyro, overlap_accel = self.imu.interval(
                    scan.header_stamp_ns, previous.timestamp_ns,
                    minimum_samples=2)
                overlap = preintegrate_midpoint(
                    overlap_times, overlap_gyro, overlap_accel,
                    bias_gyro, bias_accel, self.config)
                corrected_R, corrected_v, _ = corrected_preintegration(
                    overlap, bias_gyro, bias_accel)
                rotation_start = previous.R_WB @ corrected_R.T
                velocity_start = (
                    previous.v_WB - self.gravity_W * overlap.duration_sec
                    - rotation_start @ corrected_v)
            initial_velocity_B = rotation_start.T @ velocity_start
            gravity_B = rotation_start.T @ self.gravity_W
        else:
            acceleration_seed = robust_acceleration_mean(accel)
            gravity_B = -acceleration_seed * (
                GRAVITY_MAGNITUDE_M_S2 / np.linalg.norm(acceleration_seed))
        end_offset = max(int(item.offset_ns) for item in scan.points)
        query_offsets = {end_offset}
        for point in scan.points:
            distance = float(np.linalg.norm(point.point_L_m))
            if self.config.minimum_range_m <= distance <= self.config.maximum_range_m:
                query_offsets.add(int(point.offset_ns))
        trajectory = integrate_scan_trajectory(
            times, gyro, accel, query_offsets, initial_velocity_B, gravity_B,
            bias_gyro, bias_accel, self.config)
        end_rotation, end_translation = trajectory[end_offset]
        points = deterministic_voxel_select(
            scan, self.body_from_lidar, self.config,
            end_rotation, end_translation, trajectory)
        return PreparedScan(
            scan=scan, selected_points_B=points,
            imu_times_ns=times, imu_gyro=gyro, imu_accel=accel,
            maximum_imu_gap_ns=int(np.max(np.diff(times), initial=0)))

    def _process_scan(self, scan: LidarScan) -> None:
        try:
            prepared = self._prepare_scan(scan)
        except BoundaryCoverageViolation as error:
            if not self.bootstrapped and self._prefix_drops < (
                    self.config.maximum_prefix_drops):
                self._prefix_drops += 1
                self._append_scan_record(
                    scan, 'dropped_unbracketed_prefix', None, 0, 0, 0, (), 0,
                    str(error))
                return
            raise
        if not self.bootstrapped:
            self.bootstrap_scans.append(prepared)
            if len(self.bootstrap_scans) > self.config.maximum_active_knots:
                raise CapacityViolation('bootstrap knot capacity exceeded')
            self._attempt_bootstrap()
            return
        self._add_streaming_scan(prepared)

    def _bootstrap_minima_met(self) -> bool:
        if len(self.bootstrap_scans) < self.config.bootstrap_minimum_scans:
            return False
        first = self.bootstrap_scans[0].scan.end_ns
        last = self.bootstrap_scans[-1].scan.end_ns
        if last - first < self.config.bootstrap_minimum_ns:
            return False
        return self.imu.count_between(first, last) >= self.config.bootstrap_minimum_imu

    def _initial_bootstrap_knots(self) -> list[Knot]:
        prepared = self.bootstrap_scans
        all_accel = np.vstack([item.imu_accel for item in prepared])
        acceleration_seed = robust_acceleration_mean(all_accel)
        first_rotation = rotation_from_two_vectors(
            acceleration_seed, [0.0, 0.0, GRAVITY_MAGNITUDE_M_S2])
        rotations = [first_rotation]
        positions = [np.zeros(3)]
        centroids = [np.mean(item.selected_points_B, axis=0) for item in prepared]
        factors: list[tuple[np.ndarray, np.ndarray, np.ndarray, PreintegratedImu]] = []
        for index in range(1, len(prepared)):
            times, gyro, accel = self.imu.interval(
                prepared[index - 1].scan.end_ns,
                prepared[index].scan.end_ns, minimum_samples=2)
            integration = preintegrate_midpoint(
                times, gyro, accel, np.zeros(3), np.zeros(3), self.config)
            rotations.append(rotations[-1] @ integration.delta_R)
            positions.append(
                positions[-1] + rotations[-2] @ centroids[index - 1]
                - rotations[-1] @ centroids[index])
            factors.append((times, gyro, accel, integration))
        times_sec = np.asarray([
            (item.scan.end_ns - prepared[0].scan.end_ns) * 1e-9
            for item in prepared], dtype=np.float64)
        design = np.column_stack((
            np.ones(len(times_sec)), times_sec, 0.5 * times_sec ** 2))
        coefficients, _, rank, _ = np.linalg.lstsq(
            design, np.asarray(positions), rcond=None)
        if int(rank) < 3:
            raise ContractViolation('dynamic bootstrap motion design is rank deficient')
        first_delta_time = times_sec[1] - times_sec[0]
        if first_delta_time <= 0.0:
            raise ContractViolation('bootstrap LiDAR seed interval is invalid')
        velocity_zero = (
            np.asarray(positions[1]) - np.asarray(positions[0])) / first_delta_time
        acceleration = coefficients[2]
        knots: list[Knot] = []
        for index, item in enumerate(prepared):
            velocity = velocity_zero + acceleration * times_sec[index]
            knot = Knot(
                knot_id=self._next_knot_id + index,
                scan_index=item.scan.scan_index,
                timestamp_ns=item.scan.end_ns,
                R_WB=rotations[index], p_WB=np.asarray(positions[index]),
                v_WB=np.asarray(velocity), b_g=np.zeros(3), b_a=np.zeros(3))
            knot.validate()
            knots.append(knot)
        return knots

    def _attempt_bootstrap(self) -> None:
        elapsed = (self.bootstrap_scans[-1].scan.end_ns
                   - self.bootstrap_scans[0].scan.end_ns)
        if not self._bootstrap_minima_met():
            if elapsed >= self.config.bootstrap_maximum_ns:
                raise ContractViolation('dynamic bootstrap minima missing at maximum window')
            return
        candidate_knots = self._initial_bootstrap_knots()
        candidate_points = {
            knot.knot_id: prepared.selected_points_B
            for knot, prepared in zip(candidate_knots, self.bootstrap_scans)}
        candidate_surfels = {
            knot.knot_id: build_surfels(
                candidate_points[knot.knot_id], self.config.voxel_size_m)
            for knot in candidate_knots}
        total_surfels = sum(len(item) for item in candidate_surfels.values())
        if total_surfels > self.config.maximum_active_surfels:
            raise CapacityViolation('active surfel capacity exceeded during bootstrap')
        batches, _ = build_lidar_batches(
            candidate_knots, candidate_points, candidate_surfels, self.config)
        if not any(item.rank > 0 for item in batches):
            if elapsed >= self.config.bootstrap_maximum_ns:
                raise ContractViolation('bootstrap has no observable LiDAR subspace')
            return
        self.knots = candidate_knots
        self.points_by_knot = candidate_points
        self.surfels_by_knot = candidate_surfels
        for index in range(1, len(self.knots)):
            first = self.knots[index - 1]
            second = self.knots[index]
            times, gyro, accel = self.imu.interval(
                first.timestamp_ns, second.timestamp_ns, minimum_samples=2)
            integration = preintegrate_midpoint(
                times, gyro, accel, first.b_g, first.b_a, self.config)
            self.imu_factors[(first.knot_id, second.knot_id)] = ImuFactor(
                first.knot_id, second.knot_id, times, gyro, accel, integration)
        self._next_knot_id += len(self.knots)
        solver = self._joint_bootstrap_optimize()
        batches, correspondence_count = build_lidar_batches(
            self.knots, self.points_by_knot, self.surfels_by_knot, self.config)
        batch_by_current: dict[int, list[LidarBatch]] = {}
        for batch in batches:
            batch_by_current.setdefault(batch.current_knot_id, []).append(batch)
        for knot, prepared in zip(self.knots, self.bootstrap_scans):
            local = batch_by_current.get(knot.knot_id, [])
            singular = max(
                (item.singular_values for item in local),
                key=lambda item: float(item[0]) if len(item) else 0.0,
                default=np.empty(0))
            self._append_scan_record(
                prepared.scan, 'accepted', knot,
                len(prepared.imu_times_ns), prepared.maximum_imu_gap_ns,
                sum(len(item.residual) for item in local),
                singular, max((item.rank for item in local), default=0),
                None, solver)
        self.bootstrap_scans = []
        self._enforce_active_bounds()

    def _make_imu_factor(self, first: Knot, second_timestamp_ns: int) -> ImuFactor:
        times, gyro, accel = self.imu.interval(
            first.timestamp_ns, int(second_timestamp_ns), minimum_samples=2)
        integration = preintegrate_midpoint(
            times, gyro, accel, first.b_g, first.b_a, self.config)
        return ImuFactor(
            first.knot_id, self._next_knot_id, times, gyro, accel, integration)

    def _marginalize_until_room(self, new_timestamp_ns: int) -> None:
        while self.knots and (
                len(self.knots) >= self.config.maximum_active_knots
                or int(new_timestamp_ns) - self.knots[0].timestamp_ns
                > self.config.lag_ns):
            self._marginalize_oldest()

    def _add_streaming_scan(self, prepared: PreparedScan) -> None:
        self._marginalize_until_room(prepared.scan.end_ns)
        first = self.knots[-1]
        factor = self._make_imu_factor(first, prepared.scan.end_ns)
        corrected_R, corrected_v, corrected_p = corrected_preintegration(
            factor.preintegrated, first.b_g, first.b_a)
        dt = factor.preintegrated.duration_sec
        knot = Knot(
            knot_id=self._next_knot_id,
            scan_index=prepared.scan.scan_index,
            timestamp_ns=prepared.scan.end_ns,
            R_WB=first.R_WB @ corrected_R,
            p_WB=(first.p_WB + first.v_WB * dt
                  + 0.5 * self.gravity_W * dt * dt
                  + first.R_WB @ corrected_p),
            v_WB=first.v_WB + self.gravity_W * dt + first.R_WB @ corrected_v,
            b_g=first.b_g.copy(), b_a=first.b_a.copy())
        knot.validate()
        surfels = build_surfels(
            prepared.selected_points_B, self.config.voxel_size_m)
        if sum(len(item) for item in self.surfels_by_knot.values()) + len(
                surfels) > self.config.maximum_active_surfels:
            raise CapacityViolation('active surfel capacity exceeded')
        self.knots.append(knot)
        self.points_by_knot[knot.knot_id] = prepared.selected_points_B
        self.surfels_by_knot[knot.knot_id] = surfels
        self.imu_factors[(first.knot_id, knot.knot_id)] = ImuFactor(
            factor.start_knot_id, knot.knot_id, factor.times_ns,
            factor.gyro, factor.accel, factor.preintegrated)
        self._next_knot_id += 1
        solver = self._optimize()
        batches, correspondence_count = build_lidar_batches(
            self.knots, self.points_by_knot, self.surfels_by_knot, self.config)
        local = [item for item in batches if item.current_knot_id == knot.knot_id]
        singular = max(
            (item.singular_values for item in local),
            key=lambda item: float(item[0]) if len(item) else 0.0,
            default=np.empty(0))
        self._append_scan_record(
            prepared.scan, 'accepted', knot, len(prepared.imu_times_ns),
            prepared.maximum_imu_gap_ns,
            sum(len(item.residual) for item in local), singular,
            max((item.rank for item in local), default=0), None, solver)
        self._enforce_active_bounds()
        earliest = self.knots[0].timestamp_ns if self.knots else prepared.scan.end_ns
        self.imu.prune_before(earliest - self.config.maximum_boundary_distance_ns)

    def _refresh_preintegrations(self) -> None:
        knot_by_id = {item.knot_id: item for item in self.knots}
        for key in sorted(self.imu_factors):
            factor = self.imu_factors[key]
            first = knot_by_id.get(factor.start_knot_id)
            if first is None or factor.end_knot_id not in knot_by_id:
                continue
            gyro_change = float(np.linalg.norm(
                first.b_g - factor.preintegrated.bias_gyro_reference))
            accel_change = float(np.linalg.norm(
                first.b_a - factor.preintegrated.bias_accel_reference))
            if (gyro_change > self.config.gyro_reintegration_threshold
                    or accel_change > self.config.accel_reintegration_threshold):
                integration = preintegrate_midpoint(
                    factor.times_ns, factor.gyro, factor.accel,
                    first.b_g, first.b_a, self.config)
                self.imu_factors[key] = ImuFactor(
                    factor.start_knot_id, factor.end_knot_id,
                    factor.times_ns, factor.gyro, factor.accel, integration)

    def _factor_system(
            self, bootstrap_gravity_W: np.ndarray | None = None,
            bootstrap_reference: Mapping[str, np.ndarray] | None = None,
            bias_last_order: bool = False,
            ) -> tuple[StreamingHouseholderSystem, list[LidarBatch], int]:
        self._refresh_preintegrations()
        state_dimension = len(self.knots) * STATE_DOF
        bootstrap = bootstrap_gravity_W is not None
        if bootstrap and bias_last_order:
            raise ContractViolation(
                'bootstrap gravity and bias diagnostic orders cannot be combined')
        gravity = self.gravity_W if not bootstrap else finite_vector(
            bootstrap_gravity_W, 3, 'bootstrap gravity')
        global_dimension = 2 if bootstrap else 0
        dimension = state_dimension + global_dimension
        system = StreamingHouseholderSystem(
            dimension, self.config, additional_global_dof=global_dimension)
        knot_index = {item.knot_id: index for index, item in enumerate(self.knots)}

        def mapped_columns(columns: Sequence[int]) -> list[int]:
            if not bias_last_order:
                return [int(item) for item in columns]
            mapped: list[int] = []
            nuisance_dimension = len(self.knots) * 9
            for column in columns:
                value = int(column)
                if value >= state_dimension:
                    mapped.append(value)
                    continue
                knot_number, local = divmod(value, STATE_DOF)
                if local < 9:
                    mapped.append(knot_number * 9 + local)
                else:
                    mapped.append(
                        nuisance_dimension + knot_number * 6 + local - 9)
            return mapped

        first_columns = list(range(STATE_DOF))
        gauge = np.zeros((4, STATE_DOF))
        gauge[0, 2] = 1.0
        gauge[1, 3] = 1.0
        gauge[2, 4] = 1.0
        gauge[3, 5] = 1.0
        gauge_residual = np.array([
            so3_log(self.knots[0].R_WB)[2],
            *self.knots[0].p_WB,
        ]) * 1e6
        system.add_local(
            gauge * 1e6, gauge_residual, mapped_columns(first_columns))
        if bootstrap:
            if bootstrap_reference is None:
                raise ContractViolation('bootstrap factor system lacks references')
            require(set(bootstrap_reference) == {'velocity', 'b_g', 'b_a', 'gravity'},
                    'bootstrap reference inventory differs')
            first = self.knots[0]
            prior_J = np.zeros((9, STATE_DOF))
            prior_J[0:3, 6:9] = np.eye(3) / (
                self.config.bootstrap_velocity_prior_std_m_s)
            prior_J[3:6, 9:12] = np.eye(3) / (
                self.config.bootstrap_gyro_bias_prior_std_rad_s)
            prior_J[6:9, 12:15] = np.eye(3) / (
                self.config.bootstrap_accel_bias_prior_std_m_s2)
            prior_residual = np.concatenate((
                (first.v_WB - bootstrap_reference['velocity'])
                / self.config.bootstrap_velocity_prior_std_m_s,
                (first.b_g - bootstrap_reference['b_g'])
                / self.config.bootstrap_gyro_bias_prior_std_rad_s,
                (first.b_a - bootstrap_reference['b_a'])
                / self.config.bootstrap_accel_bias_prior_std_m_s2,
            ))
            system.add_local(
                prior_J, prior_residual, mapped_columns(first_columns))
            gravity_columns = [state_dimension, state_dimension + 1]
            gravity_residual = gravity_direction_delta(
                bootstrap_reference['gravity'], gravity) / (
                    self.config.bootstrap_gravity_direction_prior_std_rad)
            system.add_local(
                np.eye(2) / self.config.bootstrap_gravity_direction_prior_std_rad,
                gravity_residual, mapped_columns(gravity_columns))
        if self.prior is not None:
            if bootstrap:
                raise ContractViolation('bootstrap cannot consume a streaming FEJ prior')
            columns: list[int] = []
            current: list[Knot] = []
            for knot_id in self.prior.knot_ids:
                index = knot_index.get(knot_id)
                if index is None:
                    raise ContractViolation('FEJ separator knot is absent')
                columns.extend(range(index * STATE_DOF, (index + 1) * STATE_DOF))
                current.append(self.knots[index])
            delta = np.concatenate([
                state_delta(reference, value)
                for reference, value in zip(self.prior.snapshots, current)])
            residual = self.prior.matrix @ delta - self.prior.target
            system.add_local(
                self.prior.matrix, residual, mapped_columns(columns))
        bias_blocks: list[tuple[np.ndarray, np.ndarray, list[int]]] = []
        for key in sorted(self.imu_factors, key=lambda item: (
                knot_index.get(item[1], 10 ** 9), knot_index.get(item[0], 10 ** 9))):
            factor = self.imu_factors[key]
            if factor.start_knot_id not in knot_index or factor.end_knot_id not in knot_index:
                continue
            first = self.knots[knot_index[factor.start_knot_id]]
            second = self.knots[knot_index[factor.end_knot_id]]
            residual_function = lambda a, b, factor=factor: _imu_residual(
                a, b, factor, gravity)
            residual, first_J, second_J = _numerical_binary_state_jacobian(
                residual_function, first, second)
            gravity_J = np.zeros((len(residual), global_dimension))
            if bootstrap:
                for axis in range(2):
                    tangent = np.zeros(2)
                    tangent[axis] = 1e-6
                    plus = _imu_residual(
                        first, second, factor,
                        perturb_gravity_direction(gravity, tangent))
                    minus = _imu_residual(
                        first, second, factor,
                        perturb_gravity_direction(gravity, -tangent))
                    gravity_J[:, axis] = (plus - minus) / 2e-6
            covariance = factor.preintegrated.covariance
            dt = factor.preintegrated.duration_sec
            eigenvalues, eigenvectors = np.linalg.eigh(
                0.5 * (covariance + covariance.T))
            if float(eigenvalues[0]) < -self.config.covariance_negative_tolerance:
                raise ContractViolation('IMU factor covariance is not PSD')
            inverse_sqrt = eigenvectors @ np.diag(
                1.0 / np.sqrt(np.maximum(eigenvalues, self.config.covariance_floor))
            ) @ eigenvectors.T
            local_J = inverse_sqrt @ np.column_stack((
                first_J[:9], second_J[:9], gravity_J[:9]))
            local_r = inverse_sqrt @ residual[:9]
            first_index = knot_index[first.knot_id] * STATE_DOF
            second_index = knot_index[second.knot_id] * STATE_DOF
            columns = list(range(first_index, first_index + STATE_DOF)) + list(
                range(second_index, second_index + STATE_DOF))
            imu_columns = columns + (
                [state_dimension, state_dimension + 1] if bootstrap else [])
            system.add_local(local_J, local_r, mapped_columns(imu_columns))
            bias_scale = np.concatenate((
                np.full(3, 1.0 / max(
                    self.config.gyro_bias_walk * math.sqrt(dt),
                    math.sqrt(self.config.covariance_floor))),
                np.full(3, 1.0 / max(
                    self.config.accel_bias_walk * math.sqrt(dt),
                    math.sqrt(self.config.covariance_floor))),
            ))
            bias_blocks.append((
                np.column_stack((first_J[9:], second_J[9:]))
                * bias_scale[:, None],
                residual[9:] * bias_scale,
                columns))
        for local_J, local_r, columns in bias_blocks:
            system.add_local(local_J, local_r, mapped_columns(columns))
        batches, correspondence_count = build_lidar_batches(
            self.knots, self.points_by_knot, self.surfels_by_knot, self.config)
        for batch in batches:
            if batch.rank == 0:
                continue
            projected_residual = batch.retained_left.T @ batch.residual
            projected_source = batch.retained_left.T @ batch.source_jacobian
            projected_current = batch.retained_left.T @ batch.current_jacobian
            source_index = knot_index[batch.source_knot_id] * STATE_DOF
            current_index = knot_index[batch.current_knot_id] * STATE_DOF
            columns = list(range(source_index, source_index + 6)) + list(
                range(current_index, current_index + 6))
            system.add_local(
                np.column_stack((projected_source, projected_current)),
                projected_residual, mapped_columns(columns))
        return system, batches, correspondence_count

    def _nonlinear_cost(
            self, bootstrap_gravity_W: np.ndarray | None = None,
            bootstrap_reference: Mapping[str, np.ndarray] | None = None) -> float:
        system, _, _ = self._factor_system(
            bootstrap_gravity_W, bootstrap_reference)
        return float(system.cost)

    def _update_full_lag_bias_information_diagnostic(self) -> None:
        """Compute the bias Schur square root with pose/velocity ordered first."""
        system, _, _ = self._factor_system(bias_last_order=True)
        nuisance_dimension = len(self.knots) * 9
        matrix = system.augmented_R[:, :-1]
        if matrix.shape[0] <= nuisance_dimension:
            singular_values = np.empty(0)
        else:
            bias_square_root = matrix[
                nuisance_dimension:, nuisance_dimension:]
            singular_values = np.linalg.svd(
                bias_square_root, compute_uv=False) ** 2
        maximum = float(singular_values[0]) if len(singular_values) else 0.0
        threshold = max(
            self.config.minimum_lidar_singular_value ** 2,
            maximum / (self.config.maximum_lidar_condition_number ** 2))
        self._latest_bias_information_rank = int(np.count_nonzero(
            singular_values >= threshold))
        self._latest_bias_information_singular_values = [
            float(item) for item in singular_values]

    def _snapshot(self) -> list[Knot]:
        return [item.copy() for item in self.knots]

    def _restore(self, snapshot: Sequence[Knot]) -> None:
        self.knots = [item.copy() for item in snapshot]

    def _apply_global_delta(self, delta: np.ndarray, scale: float) -> None:
        value = np.asarray(delta, dtype=np.float64)
        if value.shape != (len(self.knots) * STATE_DOF,):
            raise ContractViolation('global update dimension differs')
        for index, knot in enumerate(self.knots):
            knot.apply_delta(value[
                index * STATE_DOF:(index + 1) * STATE_DOF] * float(scale))

    def _joint_bootstrap_optimize(self) -> dict[str, Any]:
        """Jointly optimize all bootstrap knots, both biases, and S2 gravity."""
        gravity = self.gravity_W.copy()
        reference = {
            'velocity': self.knots[0].v_WB.copy(),
            'b_g': self.knots[0].b_g.copy(),
            'b_a': self.knots[0].b_a.copy(),
            'gravity': gravity.copy(),
        }
        accepted_iterations = 0
        latest_rank = 0
        latest_singular_values = np.empty(0)
        cost = math.inf
        state_dimension = len(self.knots) * STATE_DOF
        for _ in range(self.config.maximum_iterations):
            system, _, _ = self._factor_system(gravity, reference)
            cost = float(system.cost)
            if not math.isfinite(cost):
                raise ContractViolation('bootstrap whitened cost is non-finite')
            update, latest_rank, latest_singular_values = system.solve()
            if update.shape != (state_dimension + 2,):
                raise ContractViolation('bootstrap joint update dimension differs')
            if float(np.linalg.norm(update)) <= 1e-12:
                break
            snapshot = self._snapshot()
            gravity_snapshot = gravity.copy()
            accepted = False
            for scale in self.config.line_search_scales:
                self._restore(snapshot)
                self._apply_global_delta(update[:state_dimension], scale)
                candidate_gravity = perturb_gravity_direction(
                    gravity_snapshot, update[state_dimension:] * float(scale))
                candidate_cost = self._nonlinear_cost(
                    candidate_gravity, reference)
                if math.isfinite(candidate_cost) and candidate_cost <= cost + 1e-9:
                    gravity = candidate_gravity
                    cost = candidate_cost
                    accepted = True
                    accepted_iterations += 1
                    break
            if not accepted:
                self._restore(snapshot)
                gravity = gravity_snapshot
                raise ContractViolation(
                    'bootstrap optimizer has no finite non-increasing step')
        self.gravity_W = gravity * (
            GRAVITY_MAGNITUDE_M_S2 / np.linalg.norm(gravity))
        self._rebase_bootstrap_world()
        self._bootstrap_gravity_jointly_optimized = True
        self._update_full_lag_bias_information_diagnostic()
        return {
            'solver_iterations': accepted_iterations,
            'whitened_cost': float(cost),
            'solver_rank': int(latest_rank),
            'solver_singular_values': [
                float(item) for item in latest_singular_values],
            'gravity_direction_jointly_optimized': True,
        }

    def _rebase_bootstrap_world(self) -> None:
        target = np.array([0.0, 0.0, -GRAVITY_MAGNITUDE_M_S2])
        gravity_alignment = rotation_from_two_vectors(self.gravity_W, target)
        first_position = self.knots[0].p_WB.copy()
        for knot in self.knots:
            knot.R_WB = gravity_alignment @ knot.R_WB
            knot.p_WB = gravity_alignment @ (knot.p_WB - first_position)
            knot.v_WB = gravity_alignment @ knot.v_WB
        self.gravity_W = gravity_alignment @ self.gravity_W
        first_rotation = self.knots[0].R_WB
        first_yaw = math.atan2(first_rotation[1, 0], first_rotation[0, 0])
        yaw_alignment = so3_exp([0.0, 0.0, -first_yaw])
        for knot in self.knots:
            knot.R_WB = yaw_alignment @ knot.R_WB
            knot.p_WB = yaw_alignment @ knot.p_WB
            knot.v_WB = yaw_alignment @ knot.v_WB
            knot.validate()
        self.gravity_W = yaw_alignment @ self.gravity_W
        if np.linalg.norm(self.gravity_W - target) > 1e-9:
            raise ContractViolation('bootstrap gravity rebase differs from negative z')
        self.gravity_W = target
        if np.linalg.norm(self.knots[0].p_WB) > 1e-9:
            raise ContractViolation('bootstrap first-position gauge differs from origin')
        rebased_yaw = math.atan2(
            self.knots[0].R_WB[1, 0], self.knots[0].R_WB[0, 0])
        if abs(rebased_yaw) > 1e-9:
            raise ContractViolation('bootstrap first-yaw gauge differs from zero')

    def _optimize(self) -> dict[str, Any]:
        accepted_iterations = 0
        latest_rank = 0
        latest_singular_values = np.empty(0)
        cost = math.inf
        for _ in range(self.config.maximum_iterations):
            system, _, _ = self._factor_system()
            cost = float(system.cost)
            if not math.isfinite(cost):
                raise ContractViolation('whitened cost is non-finite')
            update, latest_rank, latest_singular_values = system.solve()
            if float(np.linalg.norm(update)) <= 1e-12:
                break
            snapshot = self._snapshot()
            accepted = False
            for scale in self.config.line_search_scales:
                self._restore(snapshot)
                self._apply_global_delta(update, scale)
                candidate_cost = self._nonlinear_cost()
                if math.isfinite(candidate_cost) and candidate_cost <= cost + 1e-9:
                    cost = candidate_cost
                    accepted = True
                    accepted_iterations += 1
                    break
            if not accepted:
                self._restore(snapshot)
                raise ContractViolation('optimizer has no finite non-increasing step')
        self._update_full_lag_bias_information_diagnostic()
        return {
            'solver_iterations': accepted_iterations,
            'whitened_cost': float(cost),
            'solver_rank': int(latest_rank),
            'solver_singular_values': [
                float(item) for item in latest_singular_values],
        }

    def _marginalize_oldest(self) -> None:
        if len(self.knots) <= 1:
            raise CapacityViolation('cannot marginalize the final active knot')
        system, _, _ = self._factor_system()
        matrix = system.augmented_R[:, :-1]
        target = system.augmented_R[:, -1]
        prior_matrix, prior_target, marginal_rank, dropped = (
            square_root_separator_prior(
                matrix, target, STATE_DOF, self.config))
        retained = self.knots[1:]
        self.prior = FirstEstimatePrior(
            [item.knot_id for item in retained], prior_matrix, prior_target,
            retained)
        old = self.knots.pop(0)
        self.points_by_knot.pop(old.knot_id, None)
        self.surfels_by_knot.pop(old.knot_id, None)
        self.imu_factors = {
            key: value for key, value in self.imu_factors.items()
            if old.knot_id not in key}
        self._marginalized_count += 1
        self._latest_prior_rank = int(np.linalg.matrix_rank(prior_matrix))
        self._latest_prior_hash = self.prior.payload_sha256()
        if marginal_rank < 0 or not np.all(np.isfinite(dropped)):
            raise ContractViolation('marginalization diagnostics are invalid')

    def _enforce_active_bounds(self) -> None:
        if len(self.knots) > self.config.maximum_active_knots:
            raise CapacityViolation('active knot capacity exceeded')
        if len(self.knots) * STATE_DOF > self.config.maximum_state_dimension:
            raise CapacityViolation('active state dimension capacity exceeded')
        if self.knots and self.knots[-1].timestamp_ns - self.knots[0].timestamp_ns > (
                self.config.lag_ns):
            raise CapacityViolation('active lag duration exceeds contract')
        if sum(len(item) for item in self.surfels_by_knot.values()) > (
                self.config.maximum_active_surfels):
            raise CapacityViolation('active surfel capacity exceeded')

    def _append_record(self, record: dict[str, Any]) -> None:
        encoded_size = len(canonical_json(record).encode('utf-8')) + 1
        if encoded_size > self.config.maximum_diagnostic_output_bytes - (
                self._diagnostic_bytes):
            raise CapacityViolation('diagnostic output byte capacity exceeded')
        self.diagnostics.append(record)
        self._diagnostic_bytes += encoded_size

    def _append_scan_record(
            self, scan: LidarScan, status: str, knot: Knot | None,
            imu_sample_count: int, maximum_imu_gap_ns: int,
            correspondence_count: int,
            singular_values: Iterable[float], observable_rank: int,
            reason: str | None,
            solver: Mapping[str, Any] | None = None) -> None:
        if scan.scan_index in self._recorded_scan_indices:
            raise ContractViolation('diagnostic scan record is duplicated')
        runtime = self._runtime_observations.get(scan.scan_index, (0.0, 0.0))
        state = knot.diagnostic_state() if knot is not None else {
            'state_R_WB_xyzw': None,
            'state_p_WB_m': None,
            'state_v_WB_m_s': None,
            'state_b_g_rad_s': None,
            'state_b_a_m_s2': None,
        }
        singular = [float(item) for item in singular_values]
        solver_value = dict(solver or {})
        record = {
            'record_type': 'scan',
            'scan_index': int(scan.scan_index),
            'scan_start_ns': int(scan.header_stamp_ns),
            'scan_end_ns': int(scan.end_ns),
            'status': str(status),
            'reason': reason,
            'active_knot_count': len(self.knots),
            'imu_sample_count': int(imu_sample_count),
            'maximum_imu_gap_ns': int(maximum_imu_gap_ns),
            'lidar_correspondence_count': int(correspondence_count),
            'lidar_observable_rank': int(observable_rank),
            'lidar_singular_values': singular,
            **state,
            'gravity_W_m_s2': [float(item) for item in self.gravity_W],
            'solver_iterations': int(solver_value.get('solver_iterations', 0)),
            'whitened_cost': float(solver_value.get('whitened_cost', 0.0)),
            'marginal_prior_rank': int(self._latest_prior_rank),
            'nullspace_dimension': max(0, 6 - int(observable_rank)),
            'full_lag_bias_information_rank': int(
                self._latest_bias_information_rank),
            'full_lag_bias_information_singular_values': list(
                self._latest_bias_information_singular_values),
            'rss_mib': float(runtime[0]),
            'processing_rtf': float(runtime[1]),
            'prior_payload_sha256': self._latest_prior_hash,
        }
        self._append_record(record)
        self._recorded_scan_indices.add(scan.scan_index)

    def _fail(self, reason: str, scan: LidarScan | None = None) -> None:
        if self.failed:
            return
        self._terminal_reason = str(reason)
        if scan is not None and scan.scan_index not in self._recorded_scan_indices:
            try:
                self._append_scan_record(
                    scan, 'rejected_terminal', None, 0, 0, 0, (), 0, reason)
            except CapacityViolation:
                pass
        terminal = {
            'record_type': 'terminal',
            'status': 'FAIL',
            'reason': str(reason),
            'valid_shadow_result': False,
            'valid_state_count': 0,
        }
        try:
            self._append_record(terminal)
        except CapacityViolation:
            self.diagnostics = [terminal]
            self._diagnostic_bytes = len(canonical_json(terminal)) + 1

    def finalize(self, protected_after: Mapping[str, str]) -> dict[str, Any]:
        if self._finalized:
            raise EstimatorTerminalFailure('estimator is already finalized')
        if self.failed:
            try:
                self.protected_output_guard.verify(protected_after)
            except ContractViolation as error:
                self._terminal_reason = (
                    f'{self._terminal_reason}; {error}')
            self._finalized = True
            return {
                'stage': SOURCE_STAGE,
                'architecture_contract_id': ARCHITECTURE_CONTRACT_ID,
                'synthetic_contract_id': SYNTHETIC_CONTRACT_ID,
                'status': 'FAIL',
                'valid_shadow_result': False,
                'active_state_count': 0,
                'diagnostic_record_count': len(self.diagnostics),
                'diagnostic_payload_sha256': payload_sha256(self.diagnostics),
                'state_payload_sha256': None,
                'reason': self._terminal_reason,
                'authority': dict(SOURCE_AUTHORITY),
            }
        try:
            self.protected_output_guard.verify(protected_after)
            if self.pending_scans:
                if len(self.pending_scans) > self.config.maximum_suffix_drops:
                    raise ContractViolation('unbracketed suffix scan count exceeds contract')
                for scan in self.pending_scans:
                    self._suffix_drops += 1
                    self._append_scan_record(
                        scan, 'dropped_unbracketed_suffix', None, 0, 0, 0, (), 0,
                        'end-of-stream IMU bracket is incomplete')
                self.pending_scans = []
            if not self.bootstrapped:
                for prepared in self.bootstrap_scans:
                    if prepared.scan.scan_index not in self._recorded_scan_indices:
                        self._append_scan_record(
                            prepared.scan, 'rejected_bootstrap_incomplete', None,
                            len(prepared.imu_times_ns),
                            prepared.maximum_imu_gap_ns, 0, (), 0,
                            'dynamic bootstrap did not complete')
                raise ContractViolation('dynamic bootstrap did not complete')
            missing_runtime = sorted(
                self._recorded_scan_indices - set(self._runtime_observations))
            if missing_runtime:
                raise ContractViolation(
                    f'missing per-scan runtime observations: {missing_runtime[:4]}')
            state_digest = hashlib.sha256()
            for knot in self.knots:
                state_digest.update(state_payload_bytes(knot, (
                    len(self.knots), self._marginalized_count,
                    self._prefix_drops, self._suffix_drops)))
            result = {
                'stage': SOURCE_STAGE,
                'architecture_contract_id': ARCHITECTURE_CONTRACT_ID,
                'synthetic_contract_id': SYNTHETIC_CONTRACT_ID,
                'status': 'PASS',
                'valid_shadow_result': True,
                'active_state_count': len(self.knots),
                'marginalized_state_count': self._marginalized_count,
                'prefix_drop_count': self._prefix_drops,
                'suffix_drop_count': self._suffix_drops,
                'bootstrap_gravity_direction_jointly_optimized': (
                    self._bootstrap_gravity_jointly_optimized),
                'full_lag_bias_information_rank': int(
                    self._latest_bias_information_rank),
                'diagnostic_record_count': len(self.diagnostics),
                'diagnostic_payload_sha256': payload_sha256(self.diagnostics),
                'state_payload_sha256': state_digest.hexdigest(),
                'protected_output_payload_sha256': payload_sha256(
                    dict(sorted(protected_after.items()))),
                'authority': dict(SOURCE_AUTHORITY),
            }
            self._finalized = True
            return result
        except (ContractViolation, CapacityViolation) as error:
            self._fail(str(error))
            self._finalized = True
            return {
                'stage': SOURCE_STAGE,
                'architecture_contract_id': ARCHITECTURE_CONTRACT_ID,
                'synthetic_contract_id': SYNTHETIC_CONTRACT_ID,
                'status': 'FAIL',
                'valid_shadow_result': False,
                'active_state_count': 0,
                'diagnostic_record_count': len(self.diagnostics),
                'diagnostic_payload_sha256': payload_sha256(self.diagnostics),
                'state_payload_sha256': None,
                'reason': str(error),
                'authority': dict(SOURCE_AUTHORITY),
            }


__all__ = [
    'ARCHITECTURE_CONTRACT_ID', 'SYNTHETIC_CONTRACT_ID', 'SOURCE_STAGE',
    'SOURCE_AUTHORITY', 'ContractViolation', 'BoundaryCoverageViolation',
    'CapacityViolation',
    'EstimatorTerminalFailure', 'ImuSample', 'LidarPoint', 'LidarScan',
    'BodyFromLidar', 'FixedLagShadowConfig', 'PreintegratedImu', 'ImuBuffer',
    'Knot', 'FrozenKnot', 'Surfel', 'LidarBatch', 'FirstEstimatePrior',
    'ProtectedOutputGuard', 'FixedLagShadowEstimator', 'so3_exp', 'so3_log',
    'preintegrate_midpoint', 'corrected_preintegration',
    'gravity_tangent_basis', 'perturb_gravity_direction',
    'gravity_direction_delta',
    'integrate_scan_trajectory', 'deterministic_voxel_select',
    'build_surfels', 'build_lidar_batches',
    'lidar_residual_and_jacobians', 'StreamingHouseholderSystem',
    'square_root_separator_prior', 'state_payload_bytes', 'payload_sha256',
]
