"""Synthetic contracts for the v43b exact-raw intensity challenge."""

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
    'audit_v43b_raw_intensity',
    ROOT / 'scripts/audit_v43b_raw_intensity_identity.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/v43b_raw_intensity_identity_audit.json')
SOURCES_PATH = (
    ROOT / 'configs/sota_v6/development/v43b_raw_intensity_sources_20260810.json')


def contract():
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def canonical_fields():
    return [SimpleNamespace(**item)
            for item in contract()['raw_pointcloud']['required_fields']]


def point_message(*, values=None, frame='lidar', stamp_ns=1_000_000_000):
    dtype = MODULE.point_dtype(contract()['raw_pointcloud'])
    points = np.zeros(8, dtype=dtype) if values is None else values
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000),
            frame_id=frame),
        height=1,
        width=len(points),
        fields=canonical_fields(),
        is_bigendian=False,
        point_step=48,
        row_step=len(points) * 48,
        data=np.frombuffer(points.tobytes(), dtype=np.uint8),
        is_dense=True,
    )


def random_grid(seed=7):
    x, y = np.meshgrid(
        np.arange(20, dtype=np.float32) * 0.5,
        np.arange(20, dtype=np.float32) * 0.5,
        indexing='ij')
    points = np.zeros((x.size, 4), dtype=np.float32)
    points[:, 0] = x.ravel()
    points[:, 1] = y.ravel()
    points[:, 3] = np.random.default_rng(seed).integers(
        0, 256, size=x.size).astype(np.float32)
    return points


def test_contract_is_global_fixed_and_preregistered():
    value, digest = MODULE.load_contract(CONTRACT_PATH)
    assert len(digest) == 64
    assert value['frames']['world'] == 'map'
    assert value['identity']['sequence_offsets'] == [-6, -3, 0, 3, 6]
    assert value['identity']['minimum_overlap_local_pearson'] == 0.6
    assert value['identity']['minimum_spatial_peak_margin'] == 0.08
    assert value['decision']['required_repetitions'] == 2
    serialized = MODULE.canonical_json(value).lower()
    assert all(name not in serialized for name in ('navinst', 'oxford', 'urbannav'))


def test_source_manifest_binds_all_exact_raw_inputs():
    value = json.loads(SOURCES_PATH.read_text(encoding='utf-8'))
    assert [item['sequence_id'] for item in value['sequences']] == [
        'navinst_indoor02', 'oxford_spires_keble_05', 'urbannav_hk_tunnel_1']
    for item in value['sequences']:
        assert len(item['bag_sha256']) == 64
        assert item['bag_size_bytes'] > 2_000_000_000
        assert item['bag_path'].endswith('_canonical_ros1.bag')
        assert len(item['state_sha256']) == 64
        assert len(item['v42_report_sha256']) == 64


def test_cli_has_no_accuracy_reference_or_output_mutation_surface():
    source = (ROOT / 'scripts/audit_v43b_raw_intensity_identity.py').read_text(
        encoding='utf-8')
    for prohibited in (
            '--ground-truth', '--reference-map', '--trajectory-output',
            '--map-output', '--optimized-trajectory'):
        assert prohibited not in source


def test_exact_pointcloud_parser_and_header_stamp():
    config = contract()
    dtype = MODULE.point_dtype(config['raw_pointcloud'])
    values = np.zeros(8, dtype=dtype)
    values['x'] = np.arange(8, dtype=np.float32)
    values['intensity'] = np.arange(8, dtype=np.float32) * 10
    message = point_message(values=values, stamp_ns=1_234_567_890)
    parsed = MODULE.pointcloud_view(
        message, config['raw_pointcloud'], 'lidar', 100, 4096)
    assert np.array_equal(parsed['x'], values['x'])
    assert np.array_equal(parsed['intensity'], values['intensity'])
    assert MODULE.header_stamp_ns(message) == 1_234_567_890
    message.point_step = 32
    with pytest.raises(MODULE.ContractError, match='point_step'):
        MODULE.pointcloud_view(
            message, config['raw_pointcloud'], 'lidar', 100, 4096)


def test_pointcloud_parser_rejects_frame_schema_and_budget():
    config = contract()
    message = point_message(frame='wrong')
    with pytest.raises(MODULE.ContractError, match='frame'):
        MODULE.pointcloud_view(
            message, config['raw_pointcloud'], 'lidar', 100, 4096)
    message = point_message()
    message.fields[-1].offset = 36
    with pytest.raises(MODULE.ContractError, match='fields'):
        MODULE.pointcloud_view(
            message, config['raw_pointcloud'], 'lidar', 100, 4096)
    message = point_message()
    with pytest.raises(MODULE.MemoryBudgetError, match='width'):
        MODULE.pointcloud_view(
            message, config['raw_pointcloud'], 'lidar', 4, 4096)


def test_nearest_timestamp_matching_is_deterministic_and_bounded():
    stamps = np.array([1000, 2000, 3000], dtype=np.int64)
    assert MODULE.nearest_required_index(stamps, 2499) == 1
    assert MODULE.nearest_required_index(stamps, 2500) == 1
    assert MODULE.nearest_required_index(stamps, 2501) == 2
    assert MODULE.nearest_required_index(np.array([], dtype=np.int64), 10) is None


def test_lidar_to_body_transform_uses_frozen_forward_direction():
    transform = np.eye(4)
    transform[:3, :3] = np.array([
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ])
    transform[:3, 3] = (1.0, 2.0, 3.0)
    point = np.array([[2.0, 5.0, 7.0]])
    assert np.allclose(MODULE.transform_xyz(point, transform), [[-4.0, 4.0, 10.0]])


def test_voxel_mean_is_order_independent_and_bounded():
    points = np.array([
        [0.1, 0.1, 0.1, 10.0],
        [0.2, 0.2, 0.2, 30.0],
        [1.1, 0.1, 0.1, 50.0],
    ], dtype=np.float32)
    first = MODULE.voxel_mean_xyzi(points, 0.5, 10)
    second = MODULE.voxel_mean_xyzi(points[::-1], 0.5, 10)
    assert np.array_equal(first, second)
    assert len(first) == 2
    assert np.allclose(first[0], [0.15, 0.15, 0.15, 20.0])
    assert len(MODULE.voxel_mean_xyzi(points, 0.5, 1)) == 1


def test_raw_intensity_health_accepts_real_channel_shape_and_rejects_zero():
    health = contract()['intensity_health']
    varied = MODULE.IntensityAccumulator(health)
    zero = MODULE.IntensityAccumulator(health)
    for shift in range(10):
        varied.add(np.arange(256, dtype=np.float32) + shift)
        zero.add(np.zeros(256, dtype=np.float32))
    assert varied.result()['available'] is True
    result = zero.result()
    assert result['available'] is False
    assert result['checks']['finite_fraction'] is True
    assert result['nonzero_count'] == 0


def test_overlap_local_pearson_rejects_zero_variance():
    first = np.arange(100, dtype=np.float32)
    assert MODULE.overlap_local_pearson(first, first * 2 + 5) == pytest.approx(1.0)
    assert MODULE.overlap_local_pearson(first, first[::-1]) == pytest.approx(-1.0)
    assert MODULE.overlap_local_pearson(first, np.ones(100)) is None


def test_identity_pair_passes_unique_texture_with_spatial_margin():
    points = random_grid()
    result = MODULE.challenge_pair(
        points, points.copy(), np.eye(4), contract()['identity'])
    assert result['passed'] is True, result
    assert result['selected']['overlap_local_pearson'] == pytest.approx(1.0)
    assert result['selected']['mutual_correspondence_count'] == len(points)
    assert result['valid_spatial_decoy_count'] == 4
    assert result['spatial_peak_margin'] > 0.8


def test_identity_pair_rejects_periodic_texture_peak_ambiguity():
    points = random_grid()
    cells = np.rint(points[:, :2] / 0.5).astype(int)
    points[:, 3] = ((cells[:, 0] + cells[:, 1]) % 2) * 255.0
    result = MODULE.challenge_pair(
        points, points.copy(), np.eye(4), contract()['identity'])
    assert result['passed'] is False
    assert 'spatial_peak_margin' in result['reasons']
    assert result['selected']['overlap_local_pearson'] == pytest.approx(1.0)


def test_required_scans_include_anchor_radius_and_enforce_limit():
    anchors = list(range(20))
    candidate = {'geometry': {'pairs': [
        {'source_anchor': 2, 'target_anchor': 10},
        {'source_anchor': 3, 'target_anchor': 11},
    ]}}
    assert MODULE.required_state_indices([candidate], anchors, 1, 20) == [
        1, 2, 3, 4, 9, 10, 11, 12]
    with pytest.raises(MODULE.MemoryBudgetError):
        MODULE.required_state_indices([candidate], anchors, 1, 4)


def test_legacy_matching_and_new_constraint_deduplication():
    assert MODULE.matching_legacy_edges(6, 303, [(3, 300), (66, 362)], 8) == [
        [3, 300]]
    identity = {
        'passed': True,
        'minimum_pair_pearson': 0.9,
        'minimum_spatial_peak_margin': 0.2,
    }
    records = [
        {'source_anchor': 10, 'target_anchor': 100,
         'matching_legacy_edges': [], 'identity': identity},
        {'source_anchor': 12, 'target_anchor': 102,
         'matching_legacy_edges': [], 'identity': copy.deepcopy(identity)},
        {'source_anchor': 40, 'target_anchor': 200,
         'matching_legacy_edges': [[38, 198]], 'identity': copy.deepcopy(identity)},
    ]
    result = MODULE.deduplicate_new_constraints(records, 8)
    assert [(item['source_anchor'], item['target_anchor']) for item in result] == [
        (10, 100)]


def fake_report(contract_sha, repetition, deterministic):
    return {
        'status': 'PASS',
        'sequence_id': 'synthetic',
        'repetition': repetition,
        'contract_sha256': contract_sha,
        'deterministic': deterministic,
        'deterministic_payload_sha256': MODULE.payload_sha256(deterministic),
    }


def test_aggregate_closes_route_after_all_ambiguities_are_rejected(tmp_path):
    _, contract_sha = MODULE.load_contract(CONTRACT_PATH)
    deterministic = {
        'raw': {'intensity': {
            'available': True, 'scan_count': 10, 'point_count': 1000,
            'nonzero_fraction': 1.0, 'dynamic_range': 255.0}},
        'challenge': {
            'complete': True, 'v42_geometry_pass_count': 3,
            'challenged_count': 3, 'identity_pass_count_before_dedup': 0,
            'legacy_survivors': [], 'new_verified_constraints': []},
    }
    paths = []
    for repetition in (1, 2):
        path = tmp_path / f'run_{repetition}.json'
        path.write_text(json.dumps(fake_report(
            contract_sha, repetition, deterministic)), encoding='utf-8')
        paths.append(path)
    result = MODULE.aggregate_reports(
        contract_path=CONTRACT_PATH,
        reports=paths,
        expected_sequences=['synthetic'],
        output=tmp_path / 'aggregate.json')
    assert result['status'] == 'PASS'
    assert result['decision'] == (
        'CLOSE_V43B_GLOBAL_CORRECTION_ROUTE_NO_NEW_UNAMBIGUOUS_CONSTRAINT_SET')
    assert result['external_sparse_pose_graph_authorized'] is False


def test_aggregate_rejects_legacy_survivor_before_new_constraint_count(tmp_path):
    _, contract_sha = MODULE.load_contract(CONTRACT_PATH)
    deterministic = {
        'raw': {'intensity': {
            'available': True, 'scan_count': 10, 'point_count': 1000,
            'nonzero_fraction': 1.0, 'dynamic_range': 255.0}},
        'challenge': {
            'complete': True, 'v42_geometry_pass_count': 1,
            'challenged_count': 1, 'identity_pass_count_before_dedup': 1,
            'legacy_survivors': [{'source_anchor': 1, 'target_anchor': 10}],
            'new_verified_constraints': []},
    }
    paths = []
    for repetition in (1, 2):
        path = tmp_path / f'run_{repetition}.json'
        path.write_text(json.dumps(fake_report(
            contract_sha, repetition, deterministic)), encoding='utf-8')
        paths.append(path)
    result = MODULE.aggregate_reports(
        contract_path=CONTRACT_PATH, reports=paths,
        expected_sequences=['synthetic'], output=tmp_path / 'aggregate.json')
    assert result['decision'] == 'REJECT_V43B_LEGACY_AMBIGUITY_SURVIVED'


def test_aggregate_rejects_nonrepeatable_payload(tmp_path):
    _, contract_sha = MODULE.load_contract(CONTRACT_PATH)
    first = {
        'raw': {'intensity': {'available': True}},
        'challenge': {'complete': True, 'v42_geometry_pass_count': 0,
                      'legacy_survivors': [], 'new_verified_constraints': []},
    }
    second = copy.deepcopy(first)
    second['raw']['intensity']['point_count'] = 1
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
    assert result['decision'] == 'REJECT_V43B_INCOMPLETE_OR_NONREPEATABLE_AUDIT'


def test_memory_exhaustion_fails_closed():
    guard = MODULE.MemoryGuard(1024.0)
    guard.maximum_rss_mib = 0.001
    with pytest.raises(MODULE.MemoryBudgetError, match='exceeds'):
        guard.check('synthetic_exhaustion')
