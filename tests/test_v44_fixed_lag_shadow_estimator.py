"""Source-level and synthetic-stream tests for the v44 shadow estimator core."""

from dataclasses import fields, replace
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PATH = ROOT / 'scripts/v44_fixed_lag_shadow_estimator.py'
ARCHITECTURE_PATH = (
    ROOT / 'configs/sota_v6/development/v44b_fixed_lag_shadow_architecture.json')
SPEC = importlib.util.spec_from_file_location('v44_shadow_core', SOURCE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def architecture():
    return json.loads(ARCHITECTURE_PATH.read_text(encoding='utf-8'))


def config():
    return MODULE.FixedLagShadowConfig.from_architecture(architecture())


def extrinsic():
    return MODULE.BodyFromLidar(
        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        (0.0, 0.0, 0.0))


def protected():
    return {'primary_state': '0' * 64, 'primary_map': '1' * 64}


def wall_points():
    result = []
    source_index = 0
    for ring, y in enumerate(np.arange(-6.0, 6.01, 0.5)):
        for z in np.arange(-4.0, 4.01, 0.5):
            result.append(MODULE.LidarPoint(
                (10.0, float(y), float(z)),
                (source_index % 6) * 10_000_000,
                ring, source_index))
            source_index += 1
    return tuple(result)


def make_scan(index, start_ns, points=None, size=None):
    value = wall_points() if points is None else tuple(points)
    return MODULE.LidarScan(
        index, start_ns, value, index,
        len(value) * 16 if size is None else size)


def run_frozen_static_shadow():
    estimator = MODULE.FixedLagShadowEstimator(
        config(), extrinsic(), MODULE.ProtectedOutputGuard(protected()))
    points = wall_points()
    scan_times = {index * 200_000_000: index for index in range(11)}
    for timestamp_ns in range(0, 2_070_000_000, 10_000_000):
        estimator.consume_imu(MODULE.ImuSample(
            timestamp_ns, (0.0, 0.0, 0.0), (0.0, 0.0, 9.80665),
            timestamp_ns // 10_000_000, 64))
        if timestamp_ns in scan_times:
            index = scan_times[timestamp_ns]
            estimator.consume_lidar(make_scan(index, timestamp_ns, points))
            estimator.record_runtime_observation(index, 80.0, 0.01, 0.2)
    return estimator, estimator.finalize(protected())


@pytest.fixture(scope='module')
def frozen_runs():
    return run_frozen_static_shadow(), run_frozen_static_shadow()


def test_configuration_is_loaded_from_the_one_frozen_architecture():
    value = config()
    assert value.lag_ns == 3_000_000_000
    assert value.maximum_active_knots == 64
    assert value.maximum_iterations == 4
    assert value.maximum_state_dimension == 960
    assert value.maximum_active_correspondences == 768_000
    assert value.maximum_rss_mib == 330.0
    assert value.maximum_processing_rtf == 0.85
    assert value.bootstrap_velocity_prior_std_m_s == 5.0
    assert value.bootstrap_gyro_bias_prior_std_rad_s == 0.05
    assert value.bootstrap_accel_bias_prior_std_m_s2 == 1.0
    assert value.bootstrap_gravity_direction_prior_std_rad == pytest.approx(
        np.deg2rad(30.0))


def test_public_sensor_records_have_no_identity_orientation_or_covariance_input():
    imu_fields = {item.name for item in fields(MODULE.ImuSample)}
    lidar_fields = {item.name for item in fields(MODULE.LidarScan)}
    assert imu_fields == {
        'timestamp_ns', 'angular_velocity_B_rad_s',
        'linear_acceleration_B_m_s2', 'source_index',
        'serialized_size_bytes'}
    assert lidar_fields == {
        'scan_index', 'header_stamp_ns', 'points', 'source_index',
        'serialized_size_bytes'}
    assert not ({'dataset', 'sequence', 'orientation', 'covariance'}
                & (imu_fields | lidar_fields))


def test_so3_direction_and_round_trip_match_the_stage3_oracle():
    vector = np.array([0.2, -0.1, 0.4])
    rotation = MODULE.so3_exp(vector)
    assert np.allclose(MODULE.so3_log(rotation), vector, atol=1e-12, rtol=0.0)
    probe = MODULE.so3_exp([0.0, 0.0, 0.4]) @ [1.0, 0.0, 0.0]
    assert probe[0] > 0.0 and probe[1] > 0.0


def test_midpoint_preintegration_has_all_bias_jacobians_and_psd_covariance():
    times = np.arange(0, 510_000_000, 10_000_000, dtype=np.int64)
    gyro = np.tile([0.1, -0.05, 0.2], (len(times), 1))
    accel = np.tile([0.3, -0.2, 9.7], (len(times), 1))
    result = MODULE.preintegrate_midpoint(
        times, gyro, accel, np.zeros(3), np.zeros(3), config())
    for name in ('J_R_bg', 'J_v_bg', 'J_v_ba', 'J_p_bg', 'J_p_ba'):
        assert getattr(result, name).shape == (3, 3)
        assert np.all(np.isfinite(getattr(result, name)))
    assert np.min(np.linalg.eigvalsh(result.covariance)) >= -1e-12


def test_imu_buffer_coalesces_equal_stamps_stably_and_interpolates_boundaries():
    buffer = MODULE.ImuBuffer(config())
    samples = [
        (0, 0, 0.0), (0, 1, 2.0), (10_000_000, 2, 4.0),
        (20_000_000, 3, 6.0), (30_000_000, 4, 8.0),
        (40_000_000, 5, 10.0), (50_000_000, 6, 12.0),
    ]
    for timestamp, index, value in samples:
        buffer.add(MODULE.ImuSample(
            timestamp, (value, 0.0, 0.0), (0.0, 0.0, 9.80665), index, 64))
    times, gyro, _ = buffer.interval(5_000_000, 45_000_000)
    assert times.tolist() == [
        5_000_000, 10_000_000, 20_000_000, 30_000_000,
        40_000_000, 45_000_000]
    assert gyro[0, 0] == pytest.approx(3.0)
    assert gyro[-1, 0] == pytest.approx(11.0)


def test_imu_gap_and_missing_boundary_fail_with_distinct_contracts():
    value = MODULE.ImuBuffer(config())
    for index, timestamp in enumerate((0, 10_000_000, 70_000_000, 80_000_000)):
        value.add(MODULE.ImuSample(
            timestamp, (0.0, 0.0, 0.0), (0.0, 0.0, 9.80665), index, 64))
    with pytest.raises(MODULE.ContractViolation, match='oversized gap'):
        value.interval(0, 80_000_000, minimum_samples=2)
    with pytest.raises(MODULE.BoundaryCoverageViolation, match='not bracketed'):
        value.interval(-1, 10_000_000, minimum_samples=2)


def test_voxel_selection_uses_scan_end_deskew_and_fixed_extrinsic_direction():
    local_config = replace(
        config(), minimum_range_m=0.1, maximum_range_m=20.0,
        voxel_size_m=0.1)
    point = MODULE.LidarPoint((1.0, 0.0, 0.0), 0, 0, 0)
    scan = make_scan(0, 0, (point,), 64)
    body_from_lidar = MODULE.BodyFromLidar(
        ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        (1.0, 0.0, 0.0))
    selected = MODULE.deterministic_voxel_select(
        scan, body_from_lidar, local_config, np.eye(3), np.zeros(3))
    assert selected.tolist() == [[1.0, 1.0, 0.0]]


def test_scan_trajectory_integrates_every_exact_point_time_without_linear_pose_proxy():
    times = np.arange(0, 110_000_000, 10_000_000, dtype=np.int64)
    gyro = np.zeros((len(times), 3))
    accel = np.tile([0.0, 0.0, 9.80665], (len(times), 1))
    poses = MODULE.integrate_scan_trajectory(
        times, gyro, accel, [0, 50_000_000, 100_000_000],
        [1.0, 0.0, 0.0], [0.0, 0.0, -9.80665],
        [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], config())
    assert set(poses) == {0, 50_000_000, 100_000_000}
    assert np.allclose(poses[0][1], [0.0, 0.0, 0.0], atol=1e-15)
    assert np.allclose(poses[50_000_000][1], [0.05, 0.0, 0.0], atol=1e-15)
    assert np.allclose(poses[100_000_000][1], [0.1, 0.0, 0.0], atol=1e-15)
    assert all(np.array_equal(value[0], np.eye(3)) for value in poses.values())


def test_surfel_construction_is_deterministic_and_orients_normal_sign():
    points = np.asarray([
        [5.0, y, z] for y in np.linspace(-1.0, 1.0, 7)
        for z in np.linspace(-1.0, 1.0, 7)])
    first = MODULE.build_surfels(points, 0.5)
    second = MODULE.build_surfels(points[::-1], 0.5)
    assert list(first) == list(second)
    assert first
    assert all(item.normal_B[0] >= 0.99 for item in first.values())


def test_binary_point_plane_jacobians_match_finite_difference():
    source = MODULE.Knot(
        0, 0, 0, MODULE.so3_exp([0.1, -0.05, 0.02]),
        np.array([0.3, -0.2, 0.1]), np.zeros(3), np.zeros(3), np.zeros(3))
    current = MODULE.Knot(
        1, 1, 1, MODULE.so3_exp([-0.03, 0.08, 0.04]),
        np.array([1.1, 0.4, -0.2]), np.zeros(3), np.zeros(3), np.zeros(3))
    point = np.array([2.0, -0.5, 0.7])
    surfel = MODULE.Surfel(
        (0, 0, 0), (2.4, 0.1, 0.3),
        tuple(np.asarray([0.2, -0.3, 0.9327379053])
              / np.linalg.norm([0.2, -0.3, 0.9327379053])), 10)
    residual, source_J, current_J = MODULE.lidar_residual_and_jacobians(
        source, current, point, surfel)
    assert np.isfinite(residual)
    for which, analytic in (('source', source_J), ('current', current_J)):
        numerical = np.zeros(6)
        for axis in range(6):
            delta = np.zeros(MODULE.STATE_DOF)
            delta[axis] = 1e-6
            plus_source, minus_source = source.copy(), source.copy()
            plus_current, minus_current = current.copy(), current.copy()
            if which == 'source':
                plus_source.apply_delta(delta)
                minus_source.apply_delta(-delta)
            else:
                plus_current.apply_delta(delta)
                minus_current.apply_delta(-delta)
            plus = MODULE.lidar_residual_and_jacobians(
                plus_source, plus_current, point, surfel)[0]
            minus = MODULE.lidar_residual_and_jacobians(
                minus_source, minus_current, point, surfel)[0]
            numerical[axis] = (plus - minus) / 2e-6
        assert np.max(np.abs(numerical - analytic)) < 1e-8


def test_plane_correspondences_are_projected_to_the_observable_subspace():
    local = replace(config(), minimum_correspondences=20)
    points = np.asarray([
        [10.0, y, z] for y in np.arange(-3.0, 3.01, 0.5)
        for z in np.arange(-2.0, 2.01, 0.5)])
    knots = [
        MODULE.Knot(index, index, index, np.eye(3), np.zeros(3),
                    np.zeros(3), np.zeros(3), np.zeros(3))
        for index in range(2)]
    surfels = MODULE.build_surfels(points, local.voxel_size_m)
    batches, count = MODULE.build_lidar_batches(
        knots, {0: points, 1: points}, {0: surfels, 1: surfels}, local)
    assert count >= 20
    assert len(batches) == 1
    assert 0 < batches[0].rank < 6
    assert batches[0].retained_left.shape[1] == batches[0].rank


def test_streaming_householder_solution_matches_dense_least_squares():
    local = replace(config(), minimum_lidar_singular_value=1e-12)
    matrix = np.array([
        [3.0, 0.0, 1.0], [0.0, 4.0, -1.0], [2.0, 1.0, 0.0],
        [1.0, -2.0, 3.0], [0.0, 1.0, 2.0]])
    residual = np.array([1.0, -2.0, 0.5, 3.0, -1.0])
    system = MODULE.StreamingHouseholderSystem(3, local)
    system.add_local(matrix[:2], residual[:2], [0, 1, 2])
    system.add_local(matrix[2:], residual[2:], [0, 1, 2])
    update, rank, _ = system.solve()
    expected, _, _, _ = np.linalg.lstsq(matrix, -residual, rcond=None)
    assert rank == 3
    assert np.allclose(update, expected, atol=1e-12, rtol=0.0)


def test_solver_rejects_dimension_and_materialization_capacity_before_allocation():
    with pytest.raises(MODULE.CapacityViolation, match='dimension'):
        MODULE.StreamingHouseholderSystem(961, config())
    system = MODULE.StreamingHouseholderSystem(2, replace(
        config(), maximum_materialized_rows=2))
    with pytest.raises(MODULE.CapacityViolation, match='row capacity'):
        system.add_local(np.ones((3, 2)), np.ones(3), [0, 1])


def test_square_root_separator_prior_matches_full_batch_solution():
    generator = np.random.default_rng(4404016)
    matrix = generator.normal(size=(30, 8))
    target = generator.normal(size=30)
    full, _, _, _ = np.linalg.lstsq(matrix, target, rcond=None)
    prior_matrix, prior_target, rank, _ = MODULE.square_root_separator_prior(
        matrix, target, 3, replace(config(), minimum_lidar_singular_value=1e-12))
    retained, _, _, _ = np.linalg.lstsq(
        prior_matrix, prior_target, rcond=None)
    assert rank == 3
    assert np.allclose(retained, full[3:], atol=1e-10, rtol=0.0)


def test_fej_prior_payload_and_snapshots_are_immutable():
    knot = MODULE.Knot(
        0, 0, 0, np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
    prior = MODULE.FirstEstimatePrior(
        [0], np.eye(15), np.zeros(15), [knot])
    digest = prior.payload_sha256()
    with pytest.raises(ValueError):
        prior.matrix[0, 0] = 2.0
    with pytest.raises(ValueError):
        prior.snapshots[0].p_WB[0] = 2.0
    assert prior.payload_sha256() == digest


def test_protected_output_guard_accepts_identity_and_rejects_mutation():
    guard = MODULE.ProtectedOutputGuard(protected())
    assert len(guard.verify(protected())) == 64
    changed = dict(protected())
    changed['primary_map'] = '2' * 64
    with pytest.raises(MODULE.ContractViolation, match='identity changed'):
        guard.verify(changed)


def test_two_frozen_global_configuration_runs_are_byte_deterministic(frozen_runs):
    (first_estimator, first), (second_estimator, second) = frozen_runs
    assert first['status'] == second['status'] == 'PASS'
    assert first['active_state_count'] == second['active_state_count'] == 11
    assert first['bootstrap_gravity_direction_jointly_optimized'] is True
    assert second['bootstrap_gravity_direction_jointly_optimized'] is True
    assert first['full_lag_bias_information_rank'] == 62
    assert second['full_lag_bias_information_rank'] == 62
    assert first['state_payload_sha256'] == second['state_payload_sha256']
    assert first['diagnostic_payload_sha256'] == second['diagnostic_payload_sha256']
    assert first_estimator.diagnostics == second_estimator.diagnostics
    assert any(record['lidar_observable_rank'] > 0
               for record in first_estimator.diagnostics)
    assert np.array_equal(
        first_estimator.gravity_W, np.array([0.0, 0.0, -9.80665]))
    assert np.linalg.norm(first_estimator.knots[0].p_WB) <= 1e-12
    assert abs(np.arctan2(first_estimator.knots[0].R_WB[1, 0],
                          first_estimator.knots[0].R_WB[0, 0])) <= 1e-12
    reference = {
        'velocity': first_estimator.knots[0].v_WB.copy(),
        'b_g': first_estimator.knots[0].b_g.copy(),
        'b_a': first_estimator.knots[0].b_a.copy(),
        'gravity': first_estimator.gravity_W.copy(),
    }
    joint_system, _, _ = first_estimator._factor_system(
        first_estimator.gravity_W, reference)
    assert joint_system.dimension == 11 * MODULE.STATE_DOF + 2


def test_frozen_stream_records_every_scan_and_external_resource_observation(frozen_runs):
    estimator, result = frozen_runs[0]
    records = [item for item in estimator.diagnostics
               if item['record_type'] == 'scan']
    assert len(records) == result['diagnostic_record_count'] == 11
    assert all(item['status'] == 'accepted' for item in records)
    assert all(item['rss_mib'] == 80.0 for item in records)
    assert all(item['processing_rtf'] == pytest.approx(0.05) for item in records)
    assert all(item['maximum_imu_gap_ns'] == 10_000_000 for item in records)
    assert all(item['full_lag_bias_information_rank'] == 62 for item in records)
    assert all(len(item['full_lag_bias_information_singular_values']) == 66
               for item in records)


def test_result_authority_remains_report_only_after_success(frozen_runs):
    result = frozen_runs[0][1]
    assert result['authority'] == {
        'raw_shadow_replay': False,
        'accuracy_or_reference_map_inputs': False,
        'primary_trajectory_or_map_mutation': False,
        'ros_publication': False,
        'filesystem_output': False,
    }


def test_streaming_core_marginalizes_oldest_with_immutable_separator_prior():
    local = replace(
        config(), bootstrap_minimum_ns=200_000_000,
        bootstrap_maximum_ns=500_000_000, bootstrap_minimum_scans=3,
        bootstrap_minimum_imu=20, maximum_active_knots=4,
        maximum_state_dimension=60, lag_ns=300_000_000,
        minimum_correspondences=20, maximum_iterations=1)
    estimator = MODULE.FixedLagShadowEstimator(
        local, extrinsic(), MODULE.ProtectedOutputGuard(protected()))
    points = tuple(
        MODULE.LidarPoint(
            (10.0, float(y), float(z)), (index % 6) * 10_000_000,
            0, index)
        for index, (y, z) in enumerate(
            (pair for y in np.arange(-3.0, 3.01, 0.5)
             for pair in ((y, z) for z in np.arange(-2.0, 2.01, 0.5)))))
    scan_times = {index * 100_000_000: index for index in range(7)}
    for timestamp_ns in range(0, 670_000_000, 10_000_000):
        estimator.consume_imu(MODULE.ImuSample(
            timestamp_ns, (0.0, 0.0, 0.0), (0.0, 0.0, 9.80665),
            timestamp_ns // 10_000_000, 64))
        if timestamp_ns in scan_times:
            index = scan_times[timestamp_ns]
            estimator.consume_lidar(make_scan(index, timestamp_ns, points))
            estimator.record_runtime_observation(index, 50.0, 0.005, 0.1)
    result = estimator.finalize(protected())
    assert result['status'] == 'PASS'
    assert result['active_state_count'] == 4
    assert result['marginalized_state_count'] == 3
    assert [item.knot_id for item in estimator.knots] == [3, 4, 5, 6]
    assert set(estimator.points_by_knot) == {3, 4, 5, 6}
    assert set(estimator.surfels_by_knot) == {3, 4, 5, 6}
    assert estimator.prior is not None
    assert estimator.prior.knot_ids == (3, 4, 5)
    assert estimator._latest_prior_hash == estimator.prior.payload_sha256()


def test_duplicate_event_enters_terminal_failure_and_returns_no_state_hash():
    estimator = MODULE.FixedLagShadowEstimator(
        config(), extrinsic(), MODULE.ProtectedOutputGuard(protected()))
    sample = MODULE.ImuSample(
        0, (0.0, 0.0, 0.0), (0.0, 0.0, 9.80665), 0, 64)
    estimator.consume_imu(sample)
    with pytest.raises(MODULE.EstimatorTerminalFailure, match='duplicate'):
        estimator.consume_imu(sample)
    result = estimator.finalize(protected())
    assert result['status'] == 'FAIL'
    assert result['state_payload_sha256'] is None
    assert estimator.diagnostics[-1]['record_type'] == 'terminal'


def test_input_and_runtime_capacity_fail_before_valid_output():
    estimator = MODULE.FixedLagShadowEstimator(
        config(), extrinsic(), MODULE.ProtectedOutputGuard(protected()))
    estimator.consume_imu(MODULE.ImuSample(
        0, (0.0, 0.0, 0.0), (0.0, 0.0, 9.80665), 0, 64))
    with pytest.raises(MODULE.EstimatorTerminalFailure, match='input byte'):
        estimator.consume_lidar(make_scan(
            0, 0, size=config().maximum_input_message_bytes + 1))
    assert estimator.finalize(protected())['valid_shadow_result'] is False


def test_external_rss_or_rtf_violation_is_terminal_and_not_a_stopping_heuristic():
    estimator = MODULE.FixedLagShadowEstimator(
        config(), extrinsic(), MODULE.ProtectedOutputGuard(protected()))
    with pytest.raises(MODULE.EstimatorTerminalFailure, match='RSS capacity'):
        estimator.record_runtime_observation(0, 331.0, 0.01, 0.1)
    assert estimator.finalize(protected())['state_payload_sha256'] is None


def test_only_boundary_coverage_can_use_the_bounded_prefix_drop():
    estimator = MODULE.FixedLagShadowEstimator(
        config(), extrinsic(), MODULE.ProtectedOutputGuard(protected()))
    scan = make_scan(0, 0)
    estimator.consume_lidar(scan)
    estimator.record_runtime_observation(0, 50.0, 0.001, 0.1)
    estimator.consume_imu(MODULE.ImuSample(
        60_000_000, (0.0, 0.0, 0.0), (0.0, 0.0, 9.80665), 0, 64))
    assert estimator.diagnostics[0]['status'] == 'dropped_unbracketed_prefix'
    assert estimator._prefix_drops == 1


def test_malformed_geometry_is_terminal_not_a_prefix_coverage_drop():
    estimator = MODULE.FixedLagShadowEstimator(
        config(), extrinsic(), MODULE.ProtectedOutputGuard(protected()))
    estimator.consume_imu(MODULE.ImuSample(
        0, (0.0, 0.0, 0.0), (0.0, 0.0, 9.80665), 0, 64))
    near = tuple(MODULE.LidarPoint((0.1, 0.0, 0.0), index * 10_000_000,
                                   0, index) for index in range(6))
    estimator.consume_lidar(make_scan(0, 0, near, 96))
    for index, timestamp in enumerate(
            range(10_000_000, 60_000_000, 10_000_000), 1):
        estimator.consume_imu(MODULE.ImuSample(
            timestamp, (0.0, 0.0, 0.0), (0.0, 0.0, 9.80665), index, 64))
    with pytest.raises(MODULE.EstimatorTerminalFailure, match='no point'):
        estimator.consume_imu(MODULE.ImuSample(
            60_000_000, (0.0, 0.0, 0.0), (0.0, 0.0, 9.80665), 6, 64))
    assert estimator._prefix_drops == 0


def test_protected_output_mutation_converts_an_otherwise_live_result_to_failure():
    estimator = MODULE.FixedLagShadowEstimator(
        config(), extrinsic(), MODULE.ProtectedOutputGuard(protected()))
    estimator.knots = [MODULE.Knot(
        0, 0, 0, np.eye(3), np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))]
    changed = dict(protected())
    changed['primary_state'] = 'f' * 64
    result = estimator.finalize(changed)
    assert result['status'] == 'FAIL'
    assert result['state_payload_sha256'] is None


def test_source_has_no_raw_ros_filesystem_accuracy_or_wall_clock_surface():
    source = SOURCE_PATH.read_text(encoding='utf-8')
    for prohibited in (
            'import rclpy', 'import rosbags', 'import rosbag', 'import pathlib',
            'from pathlib', 'import time', 'import socket', 'import requests',
            'argparse', 'open(', '.write_text(', '.write_bytes(',
            'ground_truth', 'publish(', 'Oxford', 'UrbanNav', 'NavINST'):
        assert prohibited not in source
    assert 'SOURCE_AUTHORITY' in source


def test_factor_order_is_the_frozen_chronological_type_order():
    assert MODULE.FACTOR_TYPE_ORDER == {
        'gauge': 0,
        'marginal_prior': 1,
        'imu_preintegration': 2,
        'bias_random_walk': 3,
        'lidar_point_to_plane': 4,
    }
