#!/usr/bin/env python3
"""Validate the v44b architecture boundary without implementing an estimator."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[1]


class ContractError(ValueError):
    """Raised when the architecture is incomplete or internally inconsistent."""


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


def require_exact_keys(value: dict[str, Any], expected: Iterable[str],
                       label: str) -> None:
    expected_set = set(expected)
    actual = set(value)
    if actual != expected_set:
        missing = sorted(expected_set - actual)
        extra = sorted(actual - expected_set)
        raise ContractError(f'{label} keys differ: missing={missing}, extra={extra}')


def require_finite_tree(value: Any, label: str = 'contract') -> None:
    if isinstance(value, dict):
        for key, child in value.items():
            require_finite_tree(child, f'{label}.{key}')
    elif isinstance(value, list):
        for index, child in enumerate(value):
            require_finite_tree(child, f'{label}[{index}]')
    elif isinstance(value, float) and not math.isfinite(value):
        raise ContractError(f'{label} must be finite')


def positive(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ContractError(f'{label} must be finite and positive')
    return result


def bound_json(path_value: str, expected_sha256: str,
               label: str) -> tuple[Path, dict[str, Any]]:
    path = resolve_path(path_value)
    if not path.is_file():
        raise ContractError(f'{label} does not exist: {path}')
    observed = sha256_file(path)
    if observed != expected_sha256:
        raise ContractError(
            f'{label} hash differs: expected {expected_sha256}, observed {observed}')
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f'{label} is not readable JSON: {path}') from error
    return path, payload


def validate_prerequisite(contract: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    prerequisite = contract['prerequisite']
    require_exact_keys(prerequisite, {
        'readiness_contract_path', 'readiness_contract_sha256',
        'source_binding_path', 'source_binding_sha256',
        'readiness_aggregate_path', 'readiness_aggregate_sha256',
        'required_readiness_contract_id', 'required_readiness_decision',
    }, 'prerequisite')
    _, readiness = bound_json(
        prerequisite['readiness_contract_path'],
        prerequisite['readiness_contract_sha256'], 'v44a readiness contract')
    bound_json(
        prerequisite['source_binding_path'],
        prerequisite['source_binding_sha256'], 'v44a source binding')
    _, aggregate = bound_json(
        prerequisite['readiness_aggregate_path'],
        prerequisite['readiness_aggregate_sha256'], 'v44a readiness aggregate')
    expected_id = prerequisite['required_readiness_contract_id']
    expected_decision = prerequisite['required_readiness_decision']
    if readiness.get('contract_id') != expected_id:
        raise ContractError('v44a readiness contract ID differs')
    if aggregate.get('contract_id') != expected_id:
        raise ContractError('v44a aggregate contract ID differs')
    if aggregate.get('contract_sha256') != prerequisite['readiness_contract_sha256']:
        raise ContractError('v44a aggregate does not bind the readiness contract')
    if aggregate.get('status') != 'PASS' or aggregate.get('decision') != expected_decision:
        raise ContractError('v44a readiness decision does not authorize architecture work')
    if aggregate.get('fixed_lag_architecture_definition_authorized') is not True:
        raise ContractError('v44a architecture authorization is absent')
    if aggregate.get('shadow_estimator_implementation_authorized') is not False:
        raise ContractError('v44a must not authorize estimator implementation')
    required_architecture = {
        'explicit_noise_model_required': True,
        'orientation_independent_initialization_required': True,
        'dynamic_startup_initialization_required': True,
        'dataset_specific_algorithm_thresholds_allowed': False,
        'loop_closure_or_global_map_correction_allowed': False,
    }
    if aggregate.get('architecture_requirements') != required_architecture:
        raise ContractError('v44a architecture requirements differ')
    summary = {
        'readiness_contract_id': expected_id,
        'readiness_contract_sha256': prerequisite['readiness_contract_sha256'],
        'readiness_aggregate_sha256': prerequisite['readiness_aggregate_sha256'],
        'readiness_decision': expected_decision,
        'ready_sequence_count': len(aggregate.get('sequence_results', [])),
    }
    return readiness, summary


def validate_governance(value: dict[str, Any], serialized: str) -> None:
    if value['configuration_policy'] != 'one_global_algorithm_contract':
        raise ContractError('configuration must remain global')
    for key in (
            'dataset_identity_available_to_algorithm',
            'sequence_specific_algorithm_parameters_allowed'):
        if value[key] is not False:
            raise ContractError(f'{key} must remain false')
    if value['calibrated_sensor_extrinsics_are_source_bindings_not_algorithm_branches'] is not True:
        raise ContractError('calibration/source-binding distinction is missing')
    for token in ('navinst', 'oxford', 'urbannav'):
        if token in serialized:
            raise ContractError(f'architecture contains dataset branch token {token}')
    required_forbidden_inputs = {
        'dataset_identity', 'ground_truth_trajectory', 'accuracy_metric',
        'reference_map', 'camera', 'gnss', 'wheel_odometry', 'external_pose',
        'message_orientation', 'message_covariance_as_noise_source',
        'v17_estimator_state',
    }
    if set(value['forbidden_inputs']) != required_forbidden_inputs:
        raise ContractError('forbidden input boundary differs')
    required_retired = {
        'direct_vertical_velocity_correction',
        'direct_accelerometer_bias_correction',
        'hard_weak_axis_velocity_freeze',
        'velocity_magnitude_observability_gate',
        'history_threshold_bias_gate',
        'producer_frozen_axis_projection',
        'wall_clock_measurement_age',
        'repeated_measurement_consumption',
        'visual_velocity_feedback',
        'loop_closure',
        'global_map_correction',
    }
    if set(value['retired_mechanisms_forbidden']) != required_retired:
        raise ContractError('retired mechanism boundary differs')
    if len(value['forbidden_outputs_or_feedback']) != len(
            set(value['forbidden_outputs_or_feedback'])):
        raise ContractError('forbidden outputs contain duplicates')


def validate_frames(value: dict[str, Any]) -> None:
    expected = {
        'world_frame': 'map',
        'body_frame': 'base_link',
        'transform_notation': 'T_destination_source',
        'pose_state': 'T_WB_maps_body_coordinates_into_world',
        'lidar_extrinsic': 'T_BL_maps_lidar_coordinates_into_body',
        'imu_extrinsic': 'T_BI_maps_imu_coordinates_into_body',
    }
    for key, expected_value in expected.items():
        if value.get(key) != expected_value:
            raise ContractError(f'frame convention differs at {key}')
    gravity = [float(item) for item in value['gravity_after_bootstrap_m_s2']]
    if gravity != [0.0, 0.0, -9.80665]:
        raise ContractError('streaming gravity must be exactly world negative-z')
    if not math.isclose(math.sqrt(sum(item * item for item in gravity)),
                        9.80665, rel_tol=0.0, abs_tol=1e-12):
        raise ContractError('gravity magnitude differs')
    if value['rotation_increment'] != 'right_multiplicative_SO3':
        raise ContractError('SO3 perturbation convention differs')
    equation = value['point_transform_equation']
    for token in ('R_WB', 'R_BL', 'p_L', 't_BL', 'p_WB'):
        if token not in equation:
            raise ContractError(f'point transform omits {token}')


def validate_timing(value: dict[str, Any], readiness: dict[str, Any]) -> None:
    ready_timing = readiness['timing']
    if value['clock_representation'] != 'signed_int64_nanoseconds':
        raise ContractError('estimator clock must use integer nanoseconds')
    if value['state_timestamp_semantics'] != 'scan_end':
        raise ContractError('state timestamp must be scan end')
    if value['consume_each_message_exactly_once'] is not True:
        raise ContractError('message consumption must be exactly once')
    if value['wall_clock_used_for_estimator_time'] is not False:
        raise ContractError('wall clock must not drive estimator time')
    if value['online_timestamp_offset_estimation_allowed'] is not False:
        raise ContractError('time-offset estimation is out of scope')
    inherited = {
        'maximum_imu_gap_ns': 'maximum_imu_gap_ns',
        'maximum_imu_boundary_bracket_distance_ns':
            'maximum_imu_boundary_bracket_distance_ns',
        'minimum_imu_samples_per_scan':
            'minimum_imu_samples_per_fully_bracketed_scan',
        'maximum_dropped_unbracketed_prefix_scans':
            'maximum_unbracketed_prefix_scans',
        'maximum_dropped_unbracketed_suffix_scans':
            'maximum_unbracketed_suffix_scans',
    }
    for architecture_key, readiness_key in inherited.items():
        if int(value[architecture_key]) != int(ready_timing[readiness_key]):
            raise ContractError(f'timing bound {architecture_key} differs from v44a')
    if value['missing_scan_boundary_policy'] != (
            'drop_bounded_prefix_before_first_state_then_terminal_fail_closed'):
        raise ContractError('missing boundary policy must preserve only bounded startup')
    if value['end_of_stream_suffix_policy'] != (
            'drop_bounded_suffix_and_report_count'):
        raise ContractError('bounded end-of-stream suffix policy differs')


def validate_state(value: dict[str, Any]) -> int:
    variables = value['knot_variables']
    expected = [
        ('R_WB', 'SO3', 3), ('p_WB', 'R3', 3), ('v_WB', 'R3', 3),
        ('b_g', 'R3', 3), ('b_a', 'R3', 3),
    ]
    actual = [(item['name'], item['manifold'], int(item['local_dof']))
              for item in variables]
    if actual != expected:
        raise ContractError('15-DoF knot state order differs')
    dof = sum(item[2] for item in actual)
    if dof != 15 or int(value['local_dof_per_knot']) != dof:
        raise ContractError('knot local dimension must be 15')
    global_variables = value['bootstrap_global_variables']
    if len(global_variables) != 1 or global_variables[0].get('name') != (
            'gravity_direction_W') or global_variables[0].get('manifold') != 'S2':
        raise ContractError('bootstrap gravity must be one S2 variable')
    if float(global_variables[0].get('magnitude_m_s2', 0.0)) != 9.80665:
        raise ContractError('bootstrap gravity magnitude differs')
    for key in (
            'online_extrinsic_calibration_allowed',
            'online_time_offset_calibration_allowed',
            'online_scale_state_allowed',
            'auxiliary_weak_axis_state_allowed',
            'direct_state_component_overwrite_allowed'):
        if value[key] is not False:
            raise ContractError(f'{key} must remain false')
    return dof


def validate_initialization(value: dict[str, Any], readiness: dict[str, Any]) -> None:
    if value['mode'] != 'dynamic_joint_lidar_imu_bootstrap_for_every_sequence':
        raise ContractError('dynamic bootstrap mode differs')
    minimum_window = positive(value['minimum_window_sec'], 'minimum_window_sec')
    maximum_window = positive(value['maximum_window_sec'], 'maximum_window_sec')
    if minimum_window > maximum_window:
        raise ContractError('bootstrap windows are reversed')
    timing = readiness['timing']
    if int(value['minimum_lidar_scans']) > math.floor(
            float(timing['minimum_lidar_rate_hz']) * minimum_window):
        raise ContractError('bootstrap LiDAR minimum is impossible at readiness rate')
    if int(value['minimum_imu_messages']) > math.floor(
            float(timing['minimum_imu_rate_hz']) * minimum_window):
        raise ContractError('bootstrap IMU minimum is impossible at readiness rate')
    for key in (
            'gravity_seed_is_measurement_factor', 'orientation_message_used',
            'stationary_assumption_used', 'zero_velocity_factor_allowed',
            'zero_angular_rate_factor_allowed', 'zero_velocity_seed_is_constraint'):
        if value[key] is not False:
            raise ContractError(f'initialization {key} must remain false')
    if set(value['gauge_constraints']) != {
            'first_position_is_world_origin', 'first_pose_yaw_is_zero'}:
        raise ContractError('bootstrap gauge constraints differ')
    required = {
        'all_bootstrap_knot_states', 'gravity_direction_W',
        'gyro_bias', 'accelerometer_bias',
    }
    if set(value['jointly_optimized_variables']) != required:
        raise ContractError('bootstrap joint state differs')


def validate_noise(value: dict[str, Any]) -> None:
    if value['policy'] != 'one_explicit_continuous_time_model_for_all_sources':
        raise ContractError('explicit global noise policy differs')
    if value['message_covariance_policy'] != 'inventory_only_ignored_by_estimator':
        raise ContractError('message covariance cannot branch the noise model')
    for key in (
            'gyroscope_white_noise_density_rad_s_sqrt_hz',
            'accelerometer_white_noise_density_m_s2_sqrt_hz',
            'gyroscope_bias_random_walk_density_rad_s2_sqrt_hz',
            'accelerometer_bias_random_walk_density_m_s3_sqrt_hz',
            'lidar_point_to_plane_sigma_m', 'lidar_huber_delta_sigma',
            'covariance_negative_eigenvalue_tolerance',
            'covariance_eigenvalue_floor'):
        positive(value[key], f'noise_model.{key}')
    for key, item in value['bootstrap_prior_std'].items():
        positive(item, f'bootstrap_prior_std.{key}')


def validate_preintegration(value: dict[str, Any], timing: dict[str, Any]) -> None:
    if value['method'] != 'bias_linearized_midpoint_Forster_SO3':
        raise ContractError('preintegration method differs')
    if value['preintegrated_terms'] != [
            'DeltaR_ij', 'DeltaV_ij', 'DeltaP_ij', 'covariance_9x9']:
        raise ContractError('preintegrated term order differs')
    required_jacobians = {
        'J_R_bg', 'J_v_bg', 'J_v_ba', 'J_p_bg', 'J_p_ba'}
    if set(value['required_bias_jacobians']) != required_jacobians:
        raise ContractError('bias Jacobian set differs')
    equation_tokens = {
        'rotation_residual': ('DeltaR_ij', 'J_R_bg', 'R_WB_i', 'R_WB_j'),
        'velocity_residual': ('DeltaV_ij', 'J_v_bg', 'J_v_ba', 'g_W', 'dt'),
        'position_residual': ('DeltaP_ij', 'J_p_bg', 'J_p_ba', '0.5', 'dt^2'),
    }
    for key, tokens in equation_tokens.items():
        equation = value[key]
        for token in tokens:
            if token not in equation:
                raise ContractError(f'{key} omits {token}')
    if int(value['imu_residual_dimension']) != 9:
        raise ContractError('IMU residual dimension must be 9')
    if int(value['bias_random_walk_residual_dimension']) != 6:
        raise ContractError('bias random-walk dimension must be 6')
    if int(value['maximum_single_step_ns']) != int(timing['maximum_imu_gap_ns']):
        raise ContractError('preintegration step and readiness gap differ')
    for key, item in value['bias_reintegration_threshold'].items():
        positive(item, f'bias_reintegration_threshold.{key}')
    if value['gap_violation_policy'] != 'terminal_fail_closed_without_state_output':
        raise ContractError('preintegration gaps must fail closed')


def validate_lidar(value: dict[str, Any], maximum_active_knots: int) -> None:
    if value['type'] != 'causal_active_window_binary_point_to_plane':
        raise ContractError('LiDAR factor type differs')
    if value['intensity_used'] is not False:
        raise ContractError('v44b LiDAR factors must be geometry-only')
    minimum_range = positive(value['minimum_range_m'], 'minimum_range_m')
    maximum_range = positive(value['maximum_range_m'], 'maximum_range_m')
    if minimum_range >= maximum_range:
        raise ContractError('LiDAR range bounds are reversed')
    for key in (
            'voxel_size_m', 'maximum_selected_points_per_scan',
            'maximum_source_keyframes_per_scan', 'maximum_active_surfels',
            'minimum_correspondences', 'maximum_correspondence_distance_m'):
        positive(value[key], f'lidar_factor.{key}')
    if int(value['maximum_source_keyframes_per_scan']) >= maximum_active_knots:
        raise ContractError('LiDAR source fan-in must be less than active knots')
    for token in ('n_B_s', 'R_WB_s', 'R_WB_k', 'p_B_k', 'mu_B_s'):
        if token not in value['residual']:
            raise ContractError(f'LiDAR residual omits {token}')
    for token in ('T_WB(t_end)^-1', 'T_WB(t_point)', 'T_BL', 'p_L'):
        if token not in value['deskew_equation']:
            raise ContractError(f'deskew equation omits {token}')
    if value['global_or_persistent_map_allowed'] is not False:
        raise ContractError('persistent map is outside the local architecture')
    if value['loop_candidate_search_allowed'] is not False:
        raise ContractError('loop search is outside the local architecture')


def validate_observability(value: dict[str, Any]) -> None:
    if value['decomposition'] != 'deterministic_SVD':
        raise ContractError('observability decomposition must be deterministic SVD')
    if value['factor_observability_coordinates'] != (
            'six_dof_current_from_source_relative_pose'):
        raise ContractError('LiDAR observability must use relative-pose coordinates')
    positive(value['rotation_characteristic_length_m_per_rad'],
             'rotation_characteristic_length_m_per_rad')
    positive(value['translation_scale'], 'translation_scale')
    positive(value['minimum_whitened_singular_value'],
             'minimum_whitened_singular_value')
    maximum_condition = positive(
        value['maximum_retained_condition_number'],
        'maximum_retained_condition_number')
    if maximum_condition > 1e6:
        raise ContractError('observability condition bound is too permissive')
    for key in (
            'direct_bias_or_velocity_update_allowed', 'hard_axis_freeze_allowed',
            'velocity_magnitude_gate_allowed', 'dataset_condition_gate_allowed'):
        if value[key] is not False:
            raise ContractError(f'observability {key} must remain false')
    if value['bias_update_source'] != (
            'joint_lag_normal_equations_through_preintegration_only'):
        raise ContractError('bias updates must remain factor-graph coupled')


def validate_optimizer_and_resources(contract: dict[str, Any], readiness: dict[str, Any],
                                     state_dof: int) -> dict[str, int | float]:
    optimizer = contract['optimizer']
    resources = contract['resource_bounds']
    lag = positive(optimizer['lag_duration_sec'], 'lag_duration_sec')
    maximum_knots = int(positive(
        optimizer['maximum_active_knots'], 'maximum_active_knots'))
    minimum_capacity = math.ceil(
        float(readiness['timing']['maximum_lidar_rate_hz']) * lag) + 2
    if maximum_knots < minimum_capacity:
        raise ContractError(
            f'active knot capacity {maximum_knots} is below {minimum_capacity}')
    if optimizer['type'] != 'deterministic_square_root_fixed_lag_Gauss_Newton':
        raise ContractError('optimizer type differs')
    if optimizer['linear_solver'] != (
            'streaming_Householder_QR_then_rank_revealing_SVD_of_R'):
        raise ContractError('square-root linear solver differs')
    if int(optimizer['thread_count']) != 1:
        raise ContractError('deterministic shadow solver must be single-threaded')
    if optimizer['wall_clock_stopping_allowed'] is not False:
        raise ContractError('wall-clock stopping is forbidden')
    if int(optimizer['maximum_iterations_per_update']) <= 0:
        raise ContractError('optimizer iteration count must be positive')
    scales = [float(item) for item in optimizer['line_search_step_scales']]
    if scales != sorted(set(scales), reverse=True) or scales[0] != 1.0:
        raise ContractError('line-search scales must be fixed, unique, and descending')
    active_dimension = maximum_knots * state_dof
    if int(resources['maximum_active_state_dimension']) != active_dimension:
        raise ContractError('maximum active state dimension differs from knot contract')
    selected_points = int(contract['lidar_factor']['maximum_selected_points_per_scan'])
    active_correspondences = maximum_knots * selected_points
    if int(resources['maximum_active_lidar_correspondences']) != active_correspondences:
        raise ContractError('active correspondence bound differs from knot/point contract')
    minimum_dense_bytes = 2 * active_dimension * active_dimension * 8
    if int(resources['maximum_dense_solver_bytes']) < minimum_dense_bytes:
        raise ContractError('dense solver byte bound cannot hold two square matrices')
    if int(resources['streaming_linearization_block_rows']) > int(
            resources['maximum_materialized_jacobian_rows']):
        raise ContractError('linearization block exceeds materialized row bound')
    if int(resources['maximum_input_message_bytes']) != int(
            readiness['memory']['maximum_message_bytes']):
        raise ContractError('input message budget differs from v44a')
    if positive(resources['maximum_rss_mib'], 'maximum_rss_mib') > 330.0:
        raise ContractError('shadow RSS exceeds the retained development ceiling')
    if positive(resources['maximum_processing_rtf'], 'maximum_processing_rtf') > 0.85:
        raise ContractError('shadow RTF exceeds the retained development ceiling')
    if resources['capacity_violation_policy'] != (
            'terminal_fail_closed_before_allocation_or_write'):
        raise ContractError('capacity violation must fail closed')
    return {
        'minimum_required_active_knots': minimum_capacity,
        'maximum_active_knots': maximum_knots,
        'maximum_active_state_dimension': active_dimension,
        'maximum_active_lidar_correspondences': active_correspondences,
        'minimum_two_dense_square_matrices_bytes': minimum_dense_bytes,
        'maximum_rss_mib': float(resources['maximum_rss_mib']),
        'maximum_processing_rtf': float(resources['maximum_processing_rtf']),
    }


def validate_marginalization(value: dict[str, Any], active_dimension: int) -> None:
    if value['strategy'] != 'square_root_QR_separator_prior':
        raise ContractError('marginalization must remain square-root')
    if value['linearization_policy'] != (
            'first_estimate_Jacobians_for_marginal_prior'):
        raise ContractError('marginal prior must use FEJ')
    if int(value['maximum_prior_dimension']) > active_dimension:
        raise ContractError('marginal prior dimension exceeds active state')
    if value['prior_reset_allowed'] is not False:
        raise ContractError('marginal prior reset is forbidden')
    if value['ad_hoc_covariance_inflation_allowed'] is not False:
        raise ContractError('ad-hoc covariance inflation is forbidden')
    required = {
        'marginalized_knot_id', 'prior_rank', 'dropped_singular_values',
        'separator_dimension', 'prior_payload_sha256',
    }
    if set(value['marginalization_diagnostics_required']) != required:
        raise ContractError('marginalization diagnostics differ')


def validate_diagnostics(value: dict[str, Any], decision: dict[str, Any]) -> None:
    required_fields = {
        'scan_index', 'scan_start_ns', 'scan_end_ns', 'status',
        'active_knot_count', 'imu_sample_count', 'maximum_imu_gap_ns',
        'lidar_correspondence_count', 'lidar_observable_rank',
        'lidar_singular_values', 'state_R_WB_xyzw', 'state_p_WB_m',
        'state_v_WB_m_s', 'state_b_g_rad_s', 'state_b_a_m_s2',
        'gravity_W_m_s2', 'solver_iterations', 'whitened_cost',
        'marginal_prior_rank', 'nullspace_dimension', 'rss_mib',
    }
    if set(value['required_record_fields']) != required_fields:
        raise ContractError('diagnostic record schema differs')
    if value['ros_publishers_allowed'] is not False:
        raise ContractError('shadow diagnostics must not publish ROS output')
    if value['primary_output_paths_allowed'] is not False:
        raise ContractError('primary output paths are forbidden')
    if int(value['deterministic_repetitions_required']) != int(
            decision['required_validation_repetitions']):
        raise ContractError('diagnostic and decision repetition counts differ')
    if value['protected_v17_input_output_hashes_before_after_required'] is not True:
        raise ContractError('protected v17 hash audit is required')


def validate_decision(value: dict[str, Any], synthetic_count: int) -> None:
    if int(value['required_validation_repetitions']) != 2:
        raise ContractError('architecture validation requires two repetitions')
    if int(value['minimum_stage3_synthetic_contracts']) > synthetic_count:
        raise ContractError('stage-3 synthetic contract inventory is incomplete')
    if value['on_pass'] != 'AUTHORIZE_V44_STAGE3_SYNTHETIC_CONTRACT_IMPLEMENTATION':
        raise ContractError('stage-2 pass decision differs')
    for key in (
            'fixed_lag_shadow_estimator_implementation_authorized',
            'raw_shadow_replay_authorized',
            'accuracy_or_reference_map_inputs_authorized',
            'primary_trajectory_or_map_mutation_authorized'):
        if value[key] is not False:
            raise ContractError(f'decision {key} must remain false')


def load_and_validate_contract(path: Path) -> tuple[dict[str, Any], str, dict[str, Any]]:
    try:
        contract = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise ContractError(f'cannot read architecture contract: {path}') from error
    require_finite_tree(contract)
    require_exact_keys(contract, {
        'schema_version', 'contract_id', 'stage', 'rationale', 'prerequisite',
        'governance', 'frames', 'timing', 'state_model', 'initialization',
        'noise_model', 'preintegration', 'lidar_factor', 'observability',
        'optimizer', 'marginalization', 'resource_bounds', 'diagnostics',
        'stage3_synthetic_contracts', 'decision',
    }, 'contract')
    if contract['schema_version'] != 1:
        raise ContractError('unsupported schema_version')
    if contract['contract_id'] != 'v44b-fixed-lag-shadow-architecture-20260810':
        raise ContractError('v44b contract ID differs')
    if contract['stage'] != 'architecture_definition_only':
        raise ContractError('v44b must remain architecture-only')
    serialized = canonical_json(contract).lower()
    readiness, prerequisite_summary = validate_prerequisite(contract)
    validate_governance(contract['governance'], serialized)
    validate_frames(contract['frames'])
    validate_timing(contract['timing'], readiness)
    state_dof = validate_state(contract['state_model'])
    validate_initialization(contract['initialization'], readiness)
    validate_noise(contract['noise_model'])
    validate_preintegration(contract['preintegration'], contract['timing'])
    maximum_knots = int(contract['optimizer']['maximum_active_knots'])
    validate_lidar(contract['lidar_factor'], maximum_knots)
    validate_observability(contract['observability'])
    derived = validate_optimizer_and_resources(contract, readiness, state_dof)
    validate_marginalization(
        contract['marginalization'], int(derived['maximum_active_state_dimension']))
    validate_diagnostics(contract['diagnostics'], contract['decision'])
    synthetic = contract['stage3_synthetic_contracts']
    if len(synthetic) != len(set(synthetic)):
        raise ContractError('stage-3 synthetic contracts must be unique')
    validate_decision(contract['decision'], len(synthetic))
    deterministic = {
        'architecture_ready_for_stage3_synthetic_contracts': True,
        'contract_id': contract['contract_id'],
        'prerequisite': prerequisite_summary,
        'state_model': {
            'local_dof_per_knot': state_dof,
            'gravity_bootstrap_dof': 2,
            'message_orientation_used': False,
            'message_covariance_used': False,
        },
        'derived_resource_bounds': derived,
        'architecture_invariants': {
            'local_and_causal': True,
            'fixed_lag': True,
            'explicit_preintegration_and_bias_state': True,
            'dynamic_orientation_independent_bootstrap': True,
            'per_factor_observability_projection': True,
            'square_root_FEJ_marginalization': True,
            'single_global_algorithm_contract': True,
            'retired_direct_corrections_absent': True,
            'loop_or_global_map_correction_absent': True,
            'primary_writeback_absent': True,
        },
        'stage3_synthetic_contracts': list(synthetic),
        'stage3_synthetic_contract_count': len(synthetic),
        'authorizations': {
            'stage3_synthetic_contract_implementation': True,
            'fixed_lag_shadow_estimator_implementation': False,
            'raw_shadow_replay': False,
            'accuracy_or_reference_map_inputs': False,
            'primary_trajectory_or_map_mutation': False,
        },
    }
    return contract, sha256_file(path), deterministic


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(
        payload, indent=2, sort_keys=True, allow_nan=False) + '\n', encoding='utf-8')


def validate_once(contract_path: Path, repetition: int,
                  output: Path) -> dict[str, Any]:
    started = time.perf_counter()
    contract, contract_sha, deterministic = load_and_validate_contract(contract_path)
    report = {
        'schema_version': 1,
        'audit': 'v44b_fixed_lag_architecture_validation',
        'status': 'PASS',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'implementation_sha256': sha256_file(Path(__file__)),
        'repetition': int(repetition),
        'deterministic': deterministic,
        'deterministic_payload_sha256': payload_sha256(deterministic),
        'runtime': {'wall_seconds': time.perf_counter() - started},
    }
    write_json(output, report)
    return report


def aggregate_reports(contract_path: Path, reports: list[Path],
                      output: Path, markdown_output: Path | None = None) -> dict[str, Any]:
    contract, contract_sha, expected_deterministic = load_and_validate_contract(
        contract_path)
    required = int(contract['decision']['required_validation_repetitions'])
    loaded: list[tuple[Path, dict[str, Any]]] = []
    for path in reports:
        try:
            report = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as error:
            raise ContractError(f'cannot read validation report: {path}') from error
        loaded.append((path.resolve(), report))
    repetitions = sorted(report.get('repetition') for _, report in loaded)
    complete = len(loaded) == required and repetitions == list(range(1, required + 1))
    valid_reports = complete
    implementation_hashes: set[str] = set()
    deterministic_hashes: set[str] = set()
    for _, report in loaded:
        deterministic = report.get('deterministic')
        valid_reports = valid_reports and (
            report.get('status') == 'PASS'
            and report.get('contract_id') == contract['contract_id']
            and report.get('contract_sha256') == contract_sha
            and isinstance(deterministic, dict)
            and report.get('deterministic_payload_sha256') ==
                payload_sha256(deterministic)
            and deterministic == expected_deterministic)
        implementation_hashes.add(str(report.get('implementation_sha256')))
        deterministic_hashes.add(str(report.get('deterministic_payload_sha256')))
    repeatable = (
        valid_reports and len(implementation_hashes) == 1
        and len(deterministic_hashes) == 1)
    passed = bool(complete and valid_reports and repeatable)
    deterministic = {
        'architecture_contract_validated': passed,
        'validation_repetition_count': len(loaded),
        'validation_complete': complete,
        'validation_repeatable': repeatable,
        'architecture': expected_deterministic,
    }
    result = {
        'schema_version': 1,
        'audit': 'v44b_fixed_lag_architecture_aggregate',
        'status': 'PASS' if passed else 'FAIL',
        'contract_id': contract['contract_id'],
        'contract_sha256': contract_sha,
        'implementation_sha256': (
            next(iter(implementation_hashes)) if len(implementation_hashes) == 1
            else None),
        'decision': (
            contract['decision']['on_pass'] if passed
            else contract['decision']['on_fail']),
        'stage3_synthetic_contract_implementation_authorized': passed,
        'fixed_lag_shadow_estimator_implementation_authorized': False,
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
    write_json(output, result)
    if markdown_output is not None:
        lines = [
            '# v44b fixed-lag architecture validation', '',
            f'- Status: `{result["status"]}`',
            f'- Decision: `{result["decision"]}`',
            f'- Contract SHA-256: `{contract_sha}`',
            f'- Repeatable validations: `{len(loaded)}/{required}`',
            '- Estimator implementation authorized: `false`',
            '- Raw shadow replay authorized: `false`',
            '- Accuracy/reference-map inputs authorized: `false`',
            '',
        ]
        markdown_output.parent.mkdir(parents=True, exist_ok=True)
        markdown_output.write_text('\n'.join(lines), encoding='utf-8')
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
