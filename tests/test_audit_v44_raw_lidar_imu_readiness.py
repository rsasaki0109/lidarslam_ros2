"""Synthetic contracts for the v44 exact-raw LiDAR/IMU readiness gate."""

import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'audit_v44_raw_lidar_imu',
    ROOT / 'scripts/audit_v44_raw_lidar_imu_readiness.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/v44_raw_lidar_imu_readiness_audit.json')
SOURCES_PATH = (
    ROOT / 'configs/sota_v6/development/v44_raw_lidar_imu_sources_20260810.json')
REJECTED_PREFLIGHT_CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/'
    'v44_raw_lidar_imu_readiness_rejected_preflight.json')


def contract():
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def canonical_fields():
    return [SimpleNamespace(**item)
            for item in contract()['messages']['required_lidar_fields']]


def point_message(*, frame='lidar', point_count=8):
    messages = contract()['messages']
    values = np.zeros(point_count, dtype=MODULE.point_dtype(messages))
    values['x'] = np.arange(point_count, dtype=np.float32)
    values['t'] = np.arange(point_count, dtype=np.uint32) * 10_000_000
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=123), frame_id=frame),
        height=1,
        width=point_count,
        fields=canonical_fields(),
        is_bigendian=False,
        point_step=48,
        row_step=point_count * 48,
        data=np.frombuffer(values.tobytes(), dtype=np.uint8),
        is_dense=True,
    ), values


def test_contract_is_global_report_only_and_two_repetitions():
    value, digest = MODULE.load_contract(CONTRACT_PATH)
    assert len(digest) == 64
    assert value['contract_id'] == 'v44a-raw-lidar-imu-readiness-20260810'
    assert value['frames']['world'] == 'map'
    assert value['messages']['point_time_semantics'] == (
        'uint32_nanoseconds_from_scan_start')
    assert value['timing']['minimum_fully_bracketed_lidar_fraction'] == 0.999
    assert value['timing']['minimum_point_time_offset_ns'] == 0
    assert value['timing']['require_exact_normalization_timing_digest'] is True
    assert 'minimum_zero_origin_scan_fraction' not in value['timing']
    assert value['memory']['maximum_rss_mib'] == 128.0
    assert value['decision']['required_repetitions'] == 2
    assert value['decision']['published_trajectory_or_map_writes_allowed'] is False
    serialized = MODULE.canonical_json(value).lower()
    assert all(name not in serialized for name in ('navinst', 'oxford', 'urbannav'))


def test_rejected_preflight_contract_is_preserved_exactly():
    assert MODULE.sha256_file(REJECTED_PREFLIGHT_CONTRACT_PATH) == (
        '2336c29dce06e38f4be3dd4f6d9495ca42e2bd4bef68015f05540dbfae8771ea')
    rejected = json.loads(
        REJECTED_PREFLIGHT_CONTRACT_PATH.read_text(encoding='utf-8'))
    assert rejected['contract_id'] == 'v44-raw-lidar-imu-readiness-20260810'
    assert rejected['timing']['minimum_zero_origin_scan_fraction'] == 1.0


def test_source_manifest_binds_frames_topics_counts_and_provenance():
    source = json.loads(SOURCES_PATH.read_text(encoding='utf-8'))
    assert [item['sequence_id'] for item in source['sequences']] == [
        'navinst_indoor02', 'oxford_spires_keble_05', 'urbannav_hk_tunnel_1']
    for item in source['sequences']:
        assert len(item['bag_sha256']) == 64
        assert item['expected_lidar_messages'] > 3000
        assert item['expected_imu_messages'] > 40000
        assert item['expected_lidar_points'] > 80_000_000
        assert item['lidar_topic'].startswith('/')
        assert item['imu_topic'].startswith('/')
        assert len(item['sensor_manifest_sha256']) == 64
        assert len(item['normalization_report_sha256']) == 64
        assert len(item['sensor_adapter_sha256']) == 64


def test_cli_has_no_accuracy_reference_or_estimator_output_surface():
    source = (ROOT / 'scripts/audit_v44_raw_lidar_imu_readiness.py').read_text(
        encoding='utf-8')
    for prohibited in (
            '--ground-truth', '--reference-map', '--trajectory', '--map-output',
            '--state-output', '--bias-output'):
        assert prohibited not in source


def test_exact_pointcloud_parser_preserves_scan_start_offsets():
    message, expected = point_message()
    parsed = MODULE.pointcloud_view(
        message, contract()['messages'], 'lidar', 4096)
    assert np.array_equal(parsed['x'], expected['x'])
    assert np.array_equal(parsed['t'], expected['t'])
    assert MODULE.header_stamp_ns(message) == 1_000_000_123


def test_pointcloud_parser_rejects_schema_frame_and_budget():
    messages = contract()['messages']
    message, _ = point_message(frame='wrong')
    with pytest.raises(MODULE.ContractError, match='frame'):
        MODULE.pointcloud_view(message, messages, 'lidar', 4096)
    message, _ = point_message()
    message.fields[-1].offset = 36
    with pytest.raises(MODULE.ContractError, match='fields'):
        MODULE.pointcloud_view(message, messages, 'lidar', 4096)
    message, _ = point_message()
    with pytest.raises(MODULE.MemoryBudgetError, match='maximum_message_bytes'):
        MODULE.pointcloud_view(message, messages, 'lidar', 16)


def test_quantile_and_interval_summaries_are_deterministic():
    summary = MODULE.quantile_summary(np.array([1, 2, 3, 4, 5]))
    assert summary['minimum'] == 1
    assert summary['median'] == 3
    assert summary['maximum'] == 5
    timing = MODULE.interval_summary(
        np.array([0, 10_000_000, 20_000_000], dtype=np.int64))
    assert timing['duration_sec'] == 0.02
    assert timing['mean_rate_hz'] == 100.0
    assert timing['interval_ns']['maximum'] == 10_000_000


def test_vector_stats_retain_axis_dynamic_ranges_and_nonfinite_count():
    stats = MODULE.VectorStats()
    assert stats.add(np.array([1.0, 2.0, 3.0])) is True
    assert stats.add(np.array([2.0, 4.0, 8.0])) is True
    assert stats.add(np.array([np.nan, 0.0, 0.0])) is False
    result = stats.result()
    assert result['count'] == 3
    assert result['finite_count'] == 2
    assert result['finite_fraction'] == pytest.approx(2/3)
    assert result['dynamic_range'] == [1.0, 2.0, 5.0]


def test_covariance_classifier_distinguishes_unavailable_and_invalid():
    config = contract()['imu']
    sentinel = np.zeros(9)
    sentinel[0] = -1
    assert MODULE.classify_covariance(sentinel, config, True) == 'unknown_sentinel'
    assert MODULE.classify_covariance(np.zeros(9), config, False) == 'zero_unavailable'
    assert MODULE.classify_covariance(np.eye(3).ravel(), config, False) == 'provided_psd'
    asymmetric = np.eye(3)
    asymmetric[0, 1] = 1
    assert MODULE.classify_covariance(asymmetric.ravel(), config, False) == 'invalid'
    negative = np.diag([1.0, 1.0, -0.1])
    assert MODULE.classify_covariance(negative.ravel(), config, False) == 'invalid'


def test_startup_classifier_accepts_stationary_gravity_window():
    config = contract()['startup_classifier']
    startup = MODULE.StartupAccumulator(0, config)
    for index in range(200):
        startup.add(index * 10_000_000, np.zeros(3), np.array([0.0, 0.0, 9.80665]))
    result = startup.result()
    assert result['stationary_candidate'] is True
    assert result['sample_count'] == 200
    assert result['gyro_rms_rad_s'] == 0.0


def test_startup_classifier_marks_dynamic_motion_without_blocking_inventory():
    config = contract()['startup_classifier']
    startup = MODULE.StartupAccumulator(0, config)
    for index in range(200):
        sign = -1.0 if index % 2 else 1.0
        startup.add(
            index * 10_000_000,
            np.array([sign, 0.0, 0.0]),
            np.array([2.0 * sign, 0.0, 9.80665]))
    result = startup.result()
    assert result['stationary_candidate'] is False
    assert result['checks']['gyro_rms'] is False
    assert result['checks']['acceleration_residual_rms'] is False


def test_unbracketed_regions_split_prefix_interior_and_suffix():
    mask = np.array([True, True, False, True, False, True])
    assert MODULE.unbracketed_regions(mask) == {
        'total': 4, 'prefix': 2, 'interior': 1, 'suffix': 1}


def relaxed_timing():
    config = copy.deepcopy(contract()['timing'])
    config['minimum_fully_bracketed_lidar_fraction'] = 0.7
    return config


def test_synchronization_allows_bounded_startup_drop():
    lidar = np.array([0, 100, 200, 300], dtype=np.int64) * 1_000_000
    ends = lidar + 90_000_000
    imu = np.arange(10, 401, 10, dtype=np.int64) * 1_000_000
    result = MODULE.synchronization_summary(lidar, ends, imu, relaxed_timing())
    assert result['ready'] is True, result
    assert result['fully_bracketed_scan_count'] == 3
    assert result['unbracketed_scans'] == {
        'total': 1, 'prefix': 1, 'interior': 0, 'suffix': 0}
    assert result['maximum_boundary_bracket_distance_ns'] == 10_000_000


def test_synchronization_rejects_interior_imu_coverage_hole():
    lidar = np.array([0, 100, 200, 300], dtype=np.int64) * 1_000_000
    ends = lidar + 90_000_000
    # No IMU samples surrounding the second scan's end, while later coverage resumes.
    imu_ms = [*range(-10, 151, 10), *range(250, 411, 10)]
    imu = np.array(imu_ms, dtype=np.int64) * 1_000_000
    config = relaxed_timing()
    config['maximum_imu_boundary_bracket_distance_ns'] = 30_000_000
    result = MODULE.synchronization_summary(lidar, ends, imu, config)
    assert result['ready'] is False
    assert result['checks']['boundary_bracket_distance'] is False


def fake_report(contract_sha, repetition, deterministic):
    return {
        'status': 'PASS',
        'sequence_id': 'synthetic',
        'repetition': repetition,
        'contract_sha256': contract_sha,
        'deterministic': deterministic,
        'deterministic_payload_sha256': MODULE.payload_sha256(deterministic),
    }


def ready_deterministic():
    return {
        'ready_for_fixed_lag_architecture_definition': True,
        'inventory': {
            'lidar': {'timing': {
                'message_count': 100, 'mean_rate_hz': 10.0,
                'interval_ns': {'maximum': 100_000_000}}},
            'imu': {
                'timing': {'message_count': 1000, 'mean_rate_hz': 100.0,
                           'interval_ns': {'maximum': 10_000_000}},
                'orientation_universally_available': False,
                'angular_velocity_covariance_universally_available': False,
                'linear_acceleration_covariance_universally_available': False,
            },
            'synchronization': {
                'fully_bracketed_scan_fraction': 1.0,
                'unbracketed_scans': {'total': 0, 'prefix': 0,
                                      'interior': 0, 'suffix': 0},
                'maximum_boundary_bracket_distance_ns': 10_000_000,
            },
            'startup': {'stationary_candidate': False},
            'architecture_requirements': {
                'explicit_noise_model_required': True,
                'orientation_independent_initialization_required': True,
                'dynamic_startup_initialization_required': True,
            },
        },
    }


def write_repeated_reports(tmp_path, deterministic):
    _, contract_sha = MODULE.load_contract(CONTRACT_PATH)
    paths = []
    for repetition in (1, 2):
        path = tmp_path / f'run_{repetition}.json'
        path.write_text(json.dumps(fake_report(
            contract_sha, repetition, deterministic)), encoding='utf-8')
        paths.append(path)
    return paths


def test_aggregate_authorizes_architecture_definition_not_implementation(tmp_path):
    result = MODULE.aggregate_reports(
        contract_path=CONTRACT_PATH,
        reports=write_repeated_reports(tmp_path, ready_deterministic()),
        expected_sequences=['synthetic'],
        output=tmp_path / 'aggregate.json')
    assert result['status'] == 'PASS'
    assert result['decision'] == 'AUTHORIZE_V44_FIXED_LAG_ARCHITECTURE_DEFINITION'
    assert result['fixed_lag_architecture_definition_authorized'] is True
    assert result['shadow_estimator_implementation_authorized'] is False
    assert result['architecture_requirements'] == {
        'explicit_noise_model_required': True,
        'orientation_independent_initialization_required': True,
        'dynamic_startup_initialization_required': True,
        'dataset_specific_algorithm_thresholds_allowed': False,
        'loop_closure_or_global_map_correction_allowed': False,
    }


def test_aggregate_blocks_when_one_raw_stream_is_not_ready(tmp_path):
    deterministic = ready_deterministic()
    deterministic['ready_for_fixed_lag_architecture_definition'] = False
    result = MODULE.aggregate_reports(
        contract_path=CONTRACT_PATH,
        reports=write_repeated_reports(tmp_path, deterministic),
        expected_sequences=['synthetic'],
        output=tmp_path / 'aggregate.json')
    assert result['decision'] == 'BLOCK_V44_FIXED_LAG_ARCHITECTURE_RAW_STREAM_NOT_READY'
    assert result['sequences_not_ready'] == ['synthetic']


def test_aggregate_rejects_nonrepeatable_payload(tmp_path):
    _, contract_sha = MODULE.load_contract(CONTRACT_PATH)
    first = ready_deterministic()
    second = copy.deepcopy(first)
    second['inventory']['lidar']['timing']['message_count'] = 101
    paths = []
    for repetition, deterministic in ((1, first), (2, second)):
        path = tmp_path / f'run_{repetition}.json'
        path.write_text(json.dumps(fake_report(
            contract_sha, repetition, deterministic)), encoding='utf-8')
        paths.append(path)
    result = MODULE.aggregate_reports(
        contract_path=CONTRACT_PATH, reports=paths,
        expected_sequences=['synthetic'], output=tmp_path / 'aggregate.json')
    assert result['status'] == 'FAIL'
    assert result['decision'] == (
        'REJECT_V44_INCOMPLETE_OR_NONREPEATABLE_READINESS_AUDIT')


def test_memory_exhaustion_fails_closed():
    guard = MODULE.MemoryGuard(1024.0)
    guard.maximum_rss_mib = 0.001
    with pytest.raises(MODULE.MemoryBudgetError, match='exceeds'):
        guard.check('synthetic_exhaustion')
