"""Contract-level tests for the v44b architecture-definition boundary."""

import copy
import importlib.util
import json
import math
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'validate_v44b_architecture',
    ROOT / 'scripts/validate_v44b_fixed_lag_architecture.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/v44b_fixed_lag_shadow_architecture.json')


def contract():
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def write_contract(tmp_path: Path, value: dict) -> Path:
    path = tmp_path / 'architecture.json'
    path.write_text(json.dumps(value, allow_nan=True), encoding='utf-8')
    return path


def test_contract_is_architecture_only_global_and_prerequisite_bound():
    value, digest, deterministic = MODULE.load_and_validate_contract(CONTRACT_PATH)
    assert len(digest) == 64
    assert value['contract_id'] == 'v44b-fixed-lag-shadow-architecture-20260810'
    assert value['stage'] == 'architecture_definition_only'
    assert value['governance']['configuration_policy'] == (
        'one_global_algorithm_contract')
    assert deterministic['architecture_ready_for_stage3_synthetic_contracts'] is True
    serialized = MODULE.canonical_json(value).lower()
    assert all(name not in serialized for name in ('navinst', 'oxford', 'urbannav'))


def test_prerequisite_is_exact_passed_v44a_readiness_evidence():
    value = contract()['prerequisite']
    assert value['required_readiness_contract_id'] == (
        'v44a-raw-lidar-imu-readiness-20260810')
    assert value['required_readiness_decision'] == (
        'AUTHORIZE_V44_FIXED_LAG_ARCHITECTURE_DEFINITION')
    _, _, deterministic = MODULE.load_and_validate_contract(CONTRACT_PATH)
    prerequisite = deterministic['prerequisite']
    assert prerequisite['ready_sequence_count'] == 3
    assert prerequisite['readiness_aggregate_sha256'] == (
        '532b62ad534cb533c1ecfaa735a844f68dfb38a0a4c5184ef598ab98f3b959fb')


def test_cli_has_no_raw_replay_accuracy_or_estimator_implementation_surface():
    source = (ROOT / 'scripts/validate_v44b_fixed_lag_architecture.py').read_text(
        encoding='utf-8')
    for prohibited in (
            "add_argument('--bag", "add_argument('--ground-truth",
            "add_argument('--reference-map", "add_argument('--trajectory",
            "add_argument('--state-output", "add_argument('--map-output"):
        assert prohibited not in source
    assert set(MODULE.build_parser()._subparsers._group_actions[0].choices) == {
        'validate', 'aggregate'}


def test_state_is_one_ordered_15_dof_inertial_knot():
    state = contract()['state_model']
    assert [item['name'] for item in state['knot_variables']] == [
        'R_WB', 'p_WB', 'v_WB', 'b_g', 'b_a']
    assert sum(item['local_dof'] for item in state['knot_variables']) == 15
    assert state['local_dof_per_knot'] == 15
    assert state['bootstrap_global_variables'] == [{
        'name': 'gravity_direction_W', 'manifold': 'S2', 'local_dof': 2,
        'magnitude_m_s2': 9.80665}]


def test_frame_and_gravity_sign_conventions_are_explicit():
    frames = contract()['frames']
    assert frames['transform_notation'] == 'T_destination_source'
    assert frames['pose_state'].startswith('T_WB_')
    assert frames['lidar_extrinsic'].startswith('T_BL_')
    assert frames['gravity_after_bootstrap_m_s2'] == [0.0, 0.0, -9.80665]
    assert frames['rotation_increment'] == 'right_multiplicative_SO3'


def test_scan_end_state_time_and_integer_point_time_are_frozen():
    timing = contract()['timing']
    assert timing['clock_representation'] == 'signed_int64_nanoseconds'
    assert timing['point_timestamp_equation'] == (
        't_point_ns = lidar_header_stamp_ns + uint32_t_ns')
    assert timing['state_timestamp_semantics'] == 'scan_end'
    assert 'maximum_uint32_t_ns_in_scan' in timing['state_timestamp_equation']
    assert timing['wall_clock_used_for_estimator_time'] is False
    assert timing['consume_each_message_exactly_once'] is True


def test_timing_bounds_are_inherited_from_v44a_not_retuned():
    architecture = contract()['timing']
    readiness = json.loads((ROOT / (
        'configs/sota_v6/development/v44_raw_lidar_imu_readiness_audit.json'
    )).read_text(encoding='utf-8'))['timing']
    assert architecture['maximum_imu_gap_ns'] == readiness['maximum_imu_gap_ns']
    assert architecture['maximum_imu_boundary_bracket_distance_ns'] == (
        readiness['maximum_imu_boundary_bracket_distance_ns'])
    assert architecture['minimum_imu_samples_per_scan'] == (
        readiness['minimum_imu_samples_per_fully_bracketed_scan'])
    assert architecture['maximum_dropped_unbracketed_prefix_scans'] == (
        readiness['maximum_unbracketed_prefix_scans'])
    assert architecture['maximum_dropped_unbracketed_suffix_scans'] == (
        readiness['maximum_unbracketed_suffix_scans'])
    assert architecture['missing_scan_boundary_policy'] == (
        'drop_bounded_prefix_before_first_state_then_terminal_fail_closed')


def test_bootstrap_is_dynamic_orientation_independent_and_has_no_zupt():
    value = contract()['initialization']
    assert value['mode'] == 'dynamic_joint_lidar_imu_bootstrap_for_every_sequence'
    assert value['minimum_window_sec'] == 2.0
    assert value['maximum_window_sec'] == 5.0
    for key in (
            'gravity_seed_is_measurement_factor', 'orientation_message_used',
            'stationary_assumption_used', 'zero_velocity_factor_allowed',
            'zero_angular_rate_factor_allowed', 'zero_velocity_seed_is_constraint'):
        assert value[key] is False
    assert set(value['jointly_optimized_variables']) == {
        'all_bootstrap_knot_states', 'gravity_direction_W',
        'gyro_bias', 'accelerometer_bias'}


def test_noise_is_explicit_global_and_ignores_message_covariance():
    value = contract()['noise_model']
    assert value['policy'] == 'one_explicit_continuous_time_model_for_all_sources'
    assert value['message_covariance_policy'] == (
        'inventory_only_ignored_by_estimator')
    assert value['gyroscope_white_noise_density_rad_s_sqrt_hz'] == 0.01
    assert value['accelerometer_white_noise_density_m_s2_sqrt_hz'] == 1.0
    assert value['gyroscope_bias_random_walk_density_rad_s2_sqrt_hz'] == 0.0001
    assert value['accelerometer_bias_random_walk_density_m_s3_sqrt_hz'] == 0.0001


def test_preintegration_residuals_bind_gravity_and_bias_jacobians():
    value = contract()['preintegration']
    assert value['method'] == 'bias_linearized_midpoint_Forster_SO3'
    assert value['imu_residual_dimension'] == 9
    assert value['bias_random_walk_residual_dimension'] == 6
    equations = ' '.join(value[key] for key in (
        'rotation_residual', 'velocity_residual', 'position_residual'))
    for token in (
            'DeltaR_ij', 'DeltaV_ij', 'DeltaP_ij', 'J_R_bg', 'J_v_bg',
            'J_v_ba', 'J_p_bg', 'J_p_ba', 'g_W'):
        assert token in equations


def test_lidar_factor_is_binary_causal_geometry_only_and_deskewed():
    value = contract()['lidar_factor']
    assert value['type'] == 'causal_active_window_binary_point_to_plane'
    assert value['intensity_used'] is False
    assert value['connected_variables'] == [
        'current_R_WB_p_WB', 'source_R_WB_p_WB']
    assert 'T_WB(t_end)^-1' in value['deskew_equation']
    assert 'T_BL' in value['deskew_equation']
    assert value['global_or_persistent_map_allowed'] is False
    assert value['loop_candidate_search_allowed'] is False


def test_observability_projects_information_without_direct_correction():
    value = contract()['observability']
    assert value['decomposition'] == 'deterministic_SVD'
    assert value['factor_observability_coordinates'] == (
        'six_dof_current_from_source_relative_pose')
    assert value['retained_mode_rule'] == (
        'sigma >= max(1.0, sigma_max / 1000000.0)')
    assert value['bias_update_source'] == (
        'joint_lag_normal_equations_through_preintegration_only')
    for key in (
            'direct_bias_or_velocity_update_allowed', 'hard_axis_freeze_allowed',
            'velocity_magnitude_gate_allowed', 'dataset_condition_gate_allowed'):
        assert value[key] is False


def test_retired_v22_to_v39_mechanisms_are_explicitly_forbidden():
    retired = set(contract()['governance']['retired_mechanisms_forbidden'])
    assert {
        'direct_vertical_velocity_correction',
        'direct_accelerometer_bias_correction',
        'hard_weak_axis_velocity_freeze',
        'velocity_magnitude_observability_gate',
        'producer_frozen_axis_projection',
        'wall_clock_measurement_age',
        'repeated_measurement_consumption',
        'visual_velocity_feedback',
    } <= retired


def test_fixed_lag_capacity_covers_worst_allowed_lidar_rate():
    _, _, deterministic = MODULE.load_and_validate_contract(CONTRACT_PATH)
    derived = deterministic['derived_resource_bounds']
    assert derived['minimum_required_active_knots'] == 62
    assert derived['maximum_active_knots'] == 64
    assert derived['maximum_active_state_dimension'] == 960
    assert derived['maximum_active_lidar_correspondences'] == 768000


def test_solver_is_deterministic_fixed_work_not_wall_clock_bounded():
    value = contract()['optimizer']
    assert value['thread_count'] == 1
    assert value['maximum_iterations_per_update'] == 4
    assert value['wall_clock_stopping_allowed'] is False
    assert value['line_search_step_scales'] == [1.0, 0.5, 0.25, 0.125]
    assert value['variable_order'] == 'timestamp_then_R_p_v_b_g_b_a'


def test_marginalization_is_square_root_fej_and_cannot_reset_prior():
    value = contract()['marginalization']
    assert value['strategy'] == 'square_root_QR_separator_prior'
    assert value['linearization_policy'] == (
        'first_estimate_Jacobians_for_marginal_prior')
    assert value['prior_reset_allowed'] is False
    assert value['ad_hoc_covariance_inflation_allowed'] is False
    assert value['source_surfel_policy'] == (
        'remove_surfels_when_source_knot_is_marginalized')


def test_resource_bounds_are_closed_and_derived_consistently():
    value = contract()
    resources = value['resource_bounds']
    assert resources['maximum_rss_mib'] == 330.0
    assert resources['maximum_processing_rtf'] == 0.85
    assert resources['maximum_active_state_dimension'] == (
        value['optimizer']['maximum_active_knots']
        * value['state_model']['local_dof_per_knot'])
    assert resources['maximum_active_lidar_correspondences'] == (
        value['optimizer']['maximum_active_knots']
        * value['lidar_factor']['maximum_selected_points_per_scan'])
    assert resources['capacity_violation_policy'].startswith('terminal_fail_closed')


def test_diagnostics_are_output_only_deterministic_and_protected():
    value = contract()['diagnostics']
    assert value['ros_publishers_allowed'] is False
    assert value['primary_output_paths_allowed'] is False
    assert value['deterministic_repetitions_required'] == 2
    assert value['protected_v17_input_output_hashes_before_after_required'] is True
    for field in (
            'state_b_g_rad_s', 'state_b_a_m_s2', 'lidar_observable_rank',
            'marginal_prior_rank', 'nullspace_dimension', 'rss_mib'):
        assert field in value['required_record_fields']


def test_stage3_inventory_has_twenty_unique_fail_closed_contracts():
    value = contract()
    inventory = value['stage3_synthetic_contracts']
    assert len(inventory) == len(set(inventory)) == 20
    assert 'timestamp_gap_fail_closed' in inventory
    assert 'weak_axis_information_removal_without_state_clamp' in inventory
    assert 'FEJ_prior_consistency_and_gauge_preservation' in inventory
    assert inventory[-1] == (
        'duplicate_out_of_order_memory_and_protected_output_fail_closed')


def test_decision_authorizes_only_stage3_synthetic_work():
    value = contract()['decision']
    assert value['on_pass'] == (
        'AUTHORIZE_V44_STAGE3_SYNTHETIC_CONTRACT_IMPLEMENTATION')
    assert value['fixed_lag_shadow_estimator_implementation_authorized'] is False
    assert value['raw_shadow_replay_authorized'] is False
    assert value['accuracy_or_reference_map_inputs_authorized'] is False
    assert value['primary_trajectory_or_map_mutation_authorized'] is False


def test_dataset_branch_and_direct_state_overwrite_fail_closed(tmp_path):
    branched = contract()
    branched['rationale'] += ' Oxford special case'
    with pytest.raises(MODULE.ContractError, match='dataset branch'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, branched))
    overwrite = contract()
    overwrite['state_model']['direct_state_component_overwrite_allowed'] = True
    with pytest.raises(MODULE.ContractError, match='must remain false'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, overwrite))


def test_impossible_capacity_and_relaxed_resource_ceiling_fail_closed(tmp_path):
    too_small = contract()
    too_small['optimizer']['maximum_active_knots'] = 61
    too_small['resource_bounds']['maximum_active_state_dimension'] = 61 * 15
    too_small['resource_bounds']['maximum_active_lidar_correspondences'] = 61 * 12000
    with pytest.raises(MODULE.ContractError, match='below 62'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, too_small))
    too_large = contract()
    too_large['resource_bounds']['maximum_rss_mib'] = 330.1
    with pytest.raises(MODULE.ContractError, match='RSS exceeds'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, too_large))


def test_wrong_prerequisite_hash_and_nonfinite_value_fail_closed(tmp_path):
    wrong_hash = contract()
    wrong_hash['prerequisite']['readiness_aggregate_sha256'] = '0' * 64
    with pytest.raises(MODULE.ContractError, match='hash differs'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, wrong_hash))
    nonfinite = contract()
    nonfinite['noise_model']['lidar_point_to_plane_sigma_m'] = math.nan
    with pytest.raises(MODULE.ContractError, match='must be finite'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, nonfinite))


def test_unknown_contract_field_requires_a_versioned_schema_change(tmp_path):
    value = contract()
    value['unregistered_extension'] = True
    with pytest.raises(MODULE.ContractError, match='keys differ'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, value))


def test_two_validation_reports_have_identical_deterministic_payload(tmp_path):
    reports = []
    for repetition in (1, 2):
        path = tmp_path / f'run_{repetition:02d}.json'
        reports.append(MODULE.validate_once(CONTRACT_PATH, repetition, path))
    assert reports[0]['deterministic'] == reports[1]['deterministic']
    assert reports[0]['deterministic_payload_sha256'] == (
        reports[1]['deterministic_payload_sha256'])
    assert reports[0]['runtime'] != reports[1]['runtime']


def test_aggregate_authorizes_stage3_but_not_estimator_or_replay(tmp_path):
    report_paths = []
    for repetition in (1, 2):
        path = tmp_path / f'run_{repetition:02d}.json'
        MODULE.validate_once(CONTRACT_PATH, repetition, path)
        report_paths.append(path)
    result = MODULE.aggregate_reports(
        CONTRACT_PATH, report_paths, tmp_path / 'aggregate.json',
        tmp_path / 'aggregate.md')
    assert result['status'] == 'PASS'
    assert result['decision'] == (
        'AUTHORIZE_V44_STAGE3_SYNTHETIC_CONTRACT_IMPLEMENTATION')
    assert result['stage3_synthetic_contract_implementation_authorized'] is True
    assert result['fixed_lag_shadow_estimator_implementation_authorized'] is False
    assert result['raw_shadow_replay_authorized'] is False
    assert result['accuracy_or_reference_map_inputs_authorized'] is False


def test_aggregate_rejects_nonrepeatable_or_incomplete_validation(tmp_path):
    first = tmp_path / 'run_01.json'
    second = tmp_path / 'run_02.json'
    MODULE.validate_once(CONTRACT_PATH, 1, first)
    MODULE.validate_once(CONTRACT_PATH, 2, second)
    changed = json.loads(second.read_text(encoding='utf-8'))
    changed['deterministic']['stage3_synthetic_contract_count'] = 19
    changed['deterministic_payload_sha256'] = MODULE.payload_sha256(
        changed['deterministic'])
    second.write_text(json.dumps(changed), encoding='utf-8')
    result = MODULE.aggregate_reports(
        CONTRACT_PATH, [first, second], tmp_path / 'aggregate.json')
    assert result['status'] == 'FAIL'
    assert result['decision'] == 'REJECT_V44B_ARCHITECTURE_CONTRACT'
    incomplete = MODULE.aggregate_reports(
        CONTRACT_PATH, [first], tmp_path / 'incomplete.json')
    assert incomplete['status'] == 'FAIL'
