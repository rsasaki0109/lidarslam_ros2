"""Synthetic numerical and fail-closed tests for the v44c Stage-3 gate."""

import copy
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'validate_v44c_synthetic',
    ROOT / 'scripts/validate_v44c_fixed_lag_synthetic_contracts.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/v44c1_fixed_lag_synthetic_contracts.json')
REJECTED_PREFLIGHT_CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/v44c_fixed_lag_synthetic_contracts.json')
ARCHITECTURE_PATH = (
    ROOT / 'configs/sota_v6/development/v44b_fixed_lag_shadow_architecture.json')


def contract():
    return MODULE.resolve_contract_document(CONTRACT_PATH)[0]


def architecture():
    return json.loads(ARCHITECTURE_PATH.read_text(encoding='utf-8'))


def context():
    value = contract()
    design = architecture()
    return {
        'numeric': value['numeric_policy'],
        'architecture': design,
        'noise': design['noise_model'],
    }


def cases():
    return {item['id']: item for item in contract()['cases']}


def write_contract(tmp_path: Path, value: dict) -> Path:
    path = tmp_path / 'v44c.json'
    path.write_text(json.dumps(value, allow_nan=True), encoding='utf-8')
    return path


def test_contract_is_synthetic_only_and_exactly_bound_to_v44b():
    value, digest, design, prerequisite = MODULE.load_and_validate_contract(
        CONTRACT_PATH)
    assert len(digest) == 64
    assert value['contract_id'] == 'v44c1-fixed-lag-synthetic-contracts-20260810'
    assert value['stage'] == 'synthetic_numerical_validation_only'
    assert design['contract_id'] == 'v44b-fixed-lag-shadow-architecture-20260810'
    assert prerequisite['architecture_contract_sha256'] == (
        '74a80cca0e3dc7de9a10950d50c4fea7518dfe3d4fed8c5e1aeec77c3efee8e7')
    assert prerequisite['architecture_aggregate_sha256'] == (
        '9bc723886d3faa067de979e203eb9e49d534b2252fb733e05f08de386fb64407')


def test_rejected_absolute_host_rss_preflight_is_hash_preserved():
    assert MODULE.sha256_file(REJECTED_PREFLIGHT_CONTRACT_PATH) == (
        '0f6c57303f9889b0fd1a21b92ef80bf7d14a632010ad68222a7b6efd816e326f')
    overlay = json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))
    assert overlay['base_contract']['sha256'] == MODULE.sha256_file(
        REJECTED_PREFLIGHT_CONTRACT_PATH)
    assert overlay['rejected_preflight']['aggregate_sha256'] == (
        'f3cb78a04efad5615bc37d9ee32242b64238375c71bb21b7a52f1401d6e0a55f')
    assert overlay['correction']['performance_or_numerical_threshold_changed'] is False
    assert overlay['correction']['case_seed_or_tolerance_changed'] is False


def test_case_inventory_exactly_matches_the_twenty_preregistered_ids():
    identifiers = [item['id'] for item in contract()['cases']]
    assert identifiers == architecture()['stage3_synthetic_contracts']
    assert len(identifiers) == len(set(identifiers)) == 20
    assert set(identifiers) == set(MODULE.CASE_RUNNERS)


def test_cli_has_no_raw_ros_accuracy_map_or_estimator_runtime_surface():
    source = (ROOT / 'scripts/validate_v44c_fixed_lag_synthetic_contracts.py').read_text(
        encoding='utf-8')
    for prohibited in (
            "add_argument('--bag", "add_argument('--source-manifest",
            "add_argument('--ground-truth", "add_argument('--reference-map",
            "add_argument('--trajectory", "add_argument('--map-output",
            'import rosbags', 'import sensor_msgs', 'rclpy'):
        assert prohibited not in source
    assert set(MODULE.build_parser()._subparsers._group_actions[0].choices) == {
        'validate', 'aggregate'}


def test_so3_exp_log_are_inverse_and_preserve_positive_rotation_direction():
    vector = np.array([0.2, -0.1, 0.4])
    rotation = MODULE.so3_exp(vector)
    assert np.allclose(MODULE.so3_log(rotation), vector, atol=1e-12, rtol=0.0)
    MODULE.validate_rotation(rotation)
    rotated_x = MODULE.so3_exp([0.0, 0.0, 0.4]) @ np.array([1.0, 0.0, 0.0])
    assert rotated_x[0] > 0.0
    assert rotated_x[1] > 0.0


def test_so3_log_rejects_reflection_and_alignment_handles_antiparallel():
    reflection = np.diag([-1.0, 1.0, 1.0])
    with pytest.raises(MODULE.ContractError, match='determinant'):
        MODULE.so3_log(reflection)
    rotation = MODULE.rotation_from_two_vectors([0, 0, 1], [0, 0, -1])
    assert np.allclose(rotation @ [0, 0, 1], [0, 0, -1], atol=1e-10)
    MODULE.validate_rotation(rotation)


def test_boundary_interpolation_is_linear_and_gap_is_fail_closed():
    interpolation = MODULE.case_scan_boundary_interpolation(
        cases()['scan_boundary_interpolation'], context())
    assert interpolation['output_sample_count'] == 6
    assert interpolation['start_error'] <= 1e-15
    gap = MODULE.case_timestamp_gap_fail_closed(
        cases()['timestamp_gap_fail_closed'], context())
    assert gap['gap_rejected'] is True
    assert gap['state_output_count'] == 0


def test_preintegration_zero_residual_and_covariance_contracts():
    zero = MODULE.case_constant_motion_zero_residual(
        cases()['constant_motion_zero_residual'], context())
    assert zero['residual_dimension'] == 15
    assert zero['maximum_absolute_residual'] < 1e-12
    covariance = MODULE.case_preintegration_covariance_PSD_and_dt_scaling(
        cases()['preintegration_covariance_PSD_and_dt_scaling'], context())
    assert min(covariance['minimum_eigenvalues']) >= -1e-12
    assert 1.5 <= covariance['trace_ratio'] <= 10.0


def test_preintegration_rejects_duplicate_or_oversized_timestamp_step():
    value = architecture()['noise_model']
    gyro = np.zeros((3, 3))
    accel = np.zeros((3, 3))
    for times in (
            np.array([0, 0, 10_000_000]),
            np.array([0, 10_000_000, 70_000_001])):
        with pytest.raises(MODULE.ContractError, match='timestamp gap'):
            MODULE.preintegrate_midpoint(
                times, gyro, accel, np.zeros(3), np.zeros(3), value)


def test_bias_jacobian_predictions_are_independent_direct_reintegrations():
    gyro = MODULE.case_gyro_bias_Jacobian_finite_difference(
        cases()['gyro_bias_Jacobian_finite_difference'], context())
    assert gyro['rotation_prediction_error_rad'] < 2e-7
    assert gyro['J_R_bg_frobenius_norm'] > 0.8
    accel = MODULE.case_accelerometer_bias_Jacobians_finite_difference(
        cases()['accelerometer_bias_Jacobians_finite_difference'], context())
    assert accel['velocity_prediction_error_m_s'] < 2e-9
    assert accel['position_prediction_error_m'] < 2e-9


def test_gravity_rebase_deskew_and_extrinsic_direction_contracts():
    gravity = MODULE.case_gravity_sign_and_rebase(
        cases()['gravity_sign_and_rebase'], context())
    assert gravity['rebased_gravity_W_m_s2'][2] == pytest.approx(-9.80665)
    deskew = MODULE.case_point_deskew_direction(
        cases()['point_deskew_direction'], context())
    assert deskew['deskewed_point_B_m'] == [4.9, 0.0, 0.0]
    extrinsic = MODULE.case_lidar_to_body_extrinsic_direction(
        cases()['lidar_to_body_extrinsic_direction'], context())
    assert extrinsic['point_B_m'] == [1.0, 1.0, 0.0]
    assert extrinsic['inverse_direction_point_m'] != extrinsic['point_B_m']


def test_binary_lidar_jacobians_match_central_finite_difference():
    result = MODULE.case_binary_point_to_plane_Jacobians_finite_difference(
        cases()['binary_point_to_plane_Jacobians_finite_difference'], context())
    assert result['source_maximum_error'] < 1e-9
    assert result['current_maximum_error'] < 1e-9
    assert np.allclose(
        np.asarray(result['source_jacobian'])[3:],
        -np.asarray(result['current_jacobian'])[3:], atol=1e-12)


def test_observability_is_coordinate_invariant_and_removes_only_weak_information():
    invariant = MODULE.case_observable_subspace_rotation_invariance(
        cases()['observable_subspace_rotation_invariance'], context())
    assert invariant['rank'] == 6
    assert invariant['projector_error'] < 1e-12
    weak = MODULE.case_weak_axis_information_removal_without_state_clamp(
        cases()['weak_axis_information_removal_without_state_clamp'], context())
    assert weak['retained_rank'] == 5
    assert weak['nullspace_dimension'] == 1
    assert weak['weak_measurement_update'] == 0.0
    assert weak['weak_information_norm'] == 0.0
    assert weak['weak_process_increment_after_measurement'] == 0.75


def test_observable_subspace_rejects_nonfinite_jacobian():
    matrix = np.eye(6)
    matrix[0, 0] = math.nan
    with pytest.raises(MODULE.ContractError, match='non-finite'):
        MODULE.observable_subspace(matrix, 1.0, 1e6)


def test_dynamic_and_stationary_bootstrap_use_the_same_unbranched_path():
    dynamic = MODULE.case_dynamic_startup_without_orientation_or_zero_velocity_prior(
        cases()['dynamic_startup_without_orientation_or_zero_velocity_prior'],
        context())
    stationary = MODULE.case_stationary_startup_uses_same_architecture(
        cases()['stationary_startup_uses_same_architecture'], context())
    assert dynamic['architecture_path'] == stationary['architecture_path']
    assert np.linalg.norm(dynamic['initial_velocity_W_m_s']) > 1.0
    for result in (dynamic, stationary):
        assert result['orientation_message_used'] is False
        assert result['zero_velocity_factor_used'] is False
        assert result['stationary_branch_used'] is False


def test_square_root_marginalization_matches_full_and_schur_solutions():
    result = MODULE.case_square_root_marginalization_equivalence(
        cases()['square_root_marginalization_equivalence'], context())
    assert result['full_rank'] == 8
    assert result['prior_rank'] == 5
    assert result['solution_error'] < 1e-12
    assert result['information_error'] < 1e-12
    assert result['gradient_error'] < 1e-12


def test_fej_prior_is_immutable_and_preserves_the_chain_gauge():
    result = MODULE.case_FEJ_prior_consistency_and_gauge_preservation(
        cases()['FEJ_prior_consistency_and_gauge_preservation'], context())
    assert result['gauge_error'] == 0.0
    assert result['prior_rank'] == 1
    assert result['payload_immutable'] is True
    assert len(result['prior_payload_sha256']) == 64


def test_fixed_lag_evicts_oldest_and_removes_their_surfels():
    result = MODULE.case_fixed_lag_capacity_and_deterministic_eviction(
        cases()['fixed_lag_capacity_and_deterministic_eviction'], context())
    assert result['input_knot_count'] == 81
    assert result['final_active_knot_count'] == 61
    assert result['evicted_knot_count'] == 20
    assert result['first_active_knot_id'] == 20
    window = MODULE.FixedLagWindow(3_000_000_000, 64)
    window.add(0, 0)
    with pytest.raises(MODULE.ContractError, match='strictly increasing'):
        window.add(0, 1)


def test_factor_order_and_binary_state_payload_are_shuffle_independent():
    result = MODULE.case_deterministic_factor_order_and_state_payload_hash(
        cases()['deterministic_factor_order_and_state_payload_hash'], context())
    assert result['ordered_factor_ids'] == [0, 2, 3, 4, 7, 8]
    assert result['shuffle_repetitions'] == 2
    assert len(result['state_factor_payload_sha256']) == 64
    with pytest.raises(MODULE.ContractError, match='19 floats'):
        MODULE.state_payload_bytes(1, [0.0] * 18, [0] * 4)


def test_combined_fail_closed_case_allocates_and_writes_nothing():
    result = (
        MODULE.case_duplicate_out_of_order_memory_and_protected_output_fail_closed(
            cases()['duplicate_out_of_order_memory_and_protected_output_fail_closed'],
            context()))
    assert result['rejected_challenge_count'] == 5
    assert result['bytes_reserved_after_overflow'] == 0
    assert result['state_output_count_after_failures'] == 0
    assert result['valid_event_order'] == ['imu-1', 'lidar-2', 'imu-3']


@pytest.mark.parametrize('case_id', [item['id'] for item in contract()['cases']])
def test_each_preregistered_case_passes_independently(case_id):
    metrics = MODULE.CASE_RUNNERS[case_id](cases()[case_id], context())
    MODULE.require_finite_tree(MODULE.to_builtin(metrics), case_id)


def test_all_cases_produce_ordered_finite_hash_bound_results():
    value = contract()
    memory = MODULE.MemoryGuard(
        value['resource_bounds']['maximum_rss_mib'],
        value['resource_bounds']['maximum_incremental_rss_mib'])
    results = MODULE.run_all_cases(value, architecture(), memory)
    assert [item['id'] for item in results] == [
        item['id'] for item in value['cases']]
    assert len(results) == 20
    assert all(item['status'] == 'PASS' for item in results)
    assert all(len(item['metrics_payload_sha256']) == 64 for item in results)
    if memory.absolute_ceiling_enforced:
        assert memory.peak_rss_mib <= memory.maximum_rss_mib
    else:
        assert memory.peak_incremental_rss_mib <= (
            memory.maximum_incremental_rss_mib)


def test_dataset_branch_case_reorder_and_relaxed_authority_fail_closed(tmp_path):
    branched = contract()
    branched['rationale'] += ' Oxford special case'
    with pytest.raises(MODULE.ContractError, match='dataset branch'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, branched))
    reordered = contract()
    reordered['cases'][0], reordered['cases'][1] = (
        reordered['cases'][1], reordered['cases'][0])
    with pytest.raises(MODULE.ContractError, match='case order'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, reordered))
    relaxed = contract()
    relaxed['decision']['raw_shadow_replay_authorized_on_pass'] = True
    with pytest.raises(MODULE.ContractError, match='must remain false'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, relaxed))


def test_wrong_prerequisite_resource_relaxation_and_nonfinite_fail_closed(tmp_path):
    wrong_hash = contract()
    wrong_hash['prerequisite']['architecture_aggregate_sha256'] = '0' * 64
    with pytest.raises(MODULE.ContractError, match='hash differs'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, wrong_hash))
    resource = contract()
    resource['resource_bounds']['maximum_rss_mib'] = 128.1
    with pytest.raises(MODULE.ContractError, match='too permissive'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, resource))
    nonfinite = contract()
    nonfinite['numeric_policy']['finite_difference_epsilon'] = math.nan
    with pytest.raises(MODULE.ContractError, match='non-finite'):
        MODULE.load_and_validate_contract(write_contract(tmp_path, nonfinite))


def test_two_validation_reports_are_deterministic(tmp_path):
    reports = []
    for repetition in (1, 2):
        path = tmp_path / f'run_{repetition:02d}.json'
        reports.append(MODULE.validate_once(CONTRACT_PATH, repetition, path))
    assert reports[0]['deterministic'] == reports[1]['deterministic']
    assert reports[0]['deterministic_payload_sha256'] == (
        reports[1]['deterministic_payload_sha256'])
    assert reports[0]['runtime'] != reports[1]['runtime']


def test_aggregate_authorizes_only_report_only_shadow_implementation(tmp_path):
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
        'AUTHORIZE_V44_STAGE4_REPORT_ONLY_SHADOW_IMPLEMENTATION')
    assert result['report_only_shadow_estimator_implementation_authorized'] is True
    assert result['raw_shadow_replay_authorized'] is False
    assert result['accuracy_or_reference_map_inputs_authorized'] is False
    assert result['primary_trajectory_or_map_mutation_authorized'] is False
    assert result['deterministic']['passing_contract_count'] == 20


def test_aggregate_rejects_nonrepeatable_and_incomplete_reports(tmp_path):
    first = tmp_path / 'run_01.json'
    second = tmp_path / 'run_02.json'
    MODULE.validate_once(CONTRACT_PATH, 1, first)
    MODULE.validate_once(CONTRACT_PATH, 2, second)
    changed = json.loads(second.read_text(encoding='utf-8'))
    changed['deterministic']['case_count'] = 19
    changed['deterministic_payload_sha256'] = MODULE.payload_sha256(
        changed['deterministic'])
    second.write_text(json.dumps(changed), encoding='utf-8')
    rejected = MODULE.aggregate_reports(
        CONTRACT_PATH, [first, second], tmp_path / 'rejected.json')
    assert rejected['status'] == 'FAIL'
    assert rejected['decision'] == 'REJECT_V44C_SYNTHETIC_NUMERICAL_CONTRACTS'
    incomplete = MODULE.aggregate_reports(
        CONTRACT_PATH, [first], tmp_path / 'incomplete.json')
    assert incomplete['status'] == 'FAIL'
