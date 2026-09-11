"""Synthetic contracts for the ground-truth-free v42 feasibility audit."""

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
    'audit_v42_streaming_pose_graph',
    ROOT / 'scripts/audit_v42_streaming_pose_graph_feasibility.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
CONTRACT_PATH = ROOT / 'configs/sota_v6/development/v42_streaming_pose_graph_audit.json'


def contract():
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def write_pcd(path: Path, points: np.ndarray) -> None:
    points = np.asarray(points, dtype='<f4').reshape(-1, 4)
    header = (
        '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n'
        'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n'
    ).encode('ascii')
    path.write_bytes(header + points.tobytes())


def transform_xyz(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def test_frozen_contract_is_global_and_symmetric():
    value, digest = MODULE.load_contract(CONTRACT_PATH)
    assert len(digest) == 64
    assert value['frames'] == {'world': 'map', 'body': 'base_link'}
    assert value['sequence']['offsets'] == [-6, -3, 0, 3, 6]
    assert value['decision']['required_repetitions'] == 2
    assert value['memory']['maximum_rss_mib'] == 256.0
    serialized = MODULE.canonical_json(value).lower()
    assert all(dataset not in serialized for dataset in ('navinst', 'oxford', 'urbannav'))


def test_cli_has_no_accuracy_or_reference_input_surface():
    source = (ROOT / 'scripts/audit_v42_streaming_pose_graph_feasibility.py').read_text(
        encoding='utf-8')
    assert '--ground-truth' not in source
    assert '--reference-map' not in source
    assert 'trajectory_optimized' not in source


def test_frame_identity_fails_closed():
    MODULE.validate_frame_contract({'world': 'map', 'body': 'base_link'})
    for value in (
            {'world': 'odom', 'body': 'base_link'},
            {'world': 'map', 'body': 'lidar'}):
        with pytest.raises(MODULE.ContractError):
            MODULE.validate_frame_contract(value)


def test_transform_direction_matches_pose_graph_convention():
    world_from_source = np.eye(4)
    world_from_target = np.eye(4)
    world_from_target[0, 3] = 2.0
    target_from_source = np.linalg.inv(world_from_target) @ world_from_source
    measurement = MODULE.pose_graph_measurement(target_from_source)
    expected = np.linalg.inv(world_from_source) @ world_from_target
    assert np.allclose(measurement, expected)
    target_origin_in_source = transform_xyz(np.zeros((1, 3)), measurement)[0]
    assert np.allclose(target_origin_in_source, [2.0, 0.0, 0.0])


def test_invalid_se3_and_covariance_are_rejected():
    reflection = np.eye(4)
    reflection[0, 0] = -1.0
    with pytest.raises(MODULE.ContractError):
        MODULE.validate_se3(reflection)
    for diagonal in (
            [1.0] * 5,
            [1.0, 1.0, 1.0, 1.0, 1.0, 0.0],
            [1.0, 1.0, 1.0, 1.0, 1.0, math.nan],
            [1e-9, 1.0, 1.0, 1.0, 1.0, 1.0]):
        with pytest.raises(MODULE.ContractError):
            MODULE.validate_covariance(diagonal, maximum_condition=1e6)


def test_state_loading_rejects_duplicate_time_and_zero_quaternion(tmp_path):
    duplicate = tmp_path / 'duplicate.txt'
    duplicate.write_text(
        '1 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n', encoding='utf-8')
    with pytest.raises(MODULE.ContractError, match='not increasing'):
        MODULE.load_states(duplicate)
    invalid = tmp_path / 'invalid.txt'
    invalid.write_text('1 0 0 0 0 0 0 0\n', encoding='utf-8')
    with pytest.raises(MODULE.ContractError, match='quaternion'):
        MODULE.load_states(invalid)


def test_anchor_selection_reconstructs_exact_distance_policy():
    transforms = np.repeat(np.eye(4)[None, :, :], 5, axis=0)
    transforms[:, 0, 3] = [0.0, 1.49, 1.5, 2.99, 3.0]
    assert MODULE.select_anchor_indices(transforms, 1.5) == [0, 2, 4]


def test_binary_pcd_reader_is_strict_and_exact(tmp_path):
    path = tmp_path / '0.pcd'
    expected = np.array([[1, 2, 3, 4], [5, 6, 7, 8]], dtype=np.float32)
    write_pcd(path, expected)
    assert np.array_equal(MODULE.read_binary_xyzi_pcd(path), expected)
    path.write_bytes(path.read_bytes()[:-1])
    with pytest.raises(MODULE.ContractError, match='byte count'):
        MODULE.read_binary_xyzi_pcd(path)


def test_submap_uses_map_from_body_transform_direction(tmp_path):
    chunks = []
    for index in range(2):
        path = tmp_path / f'{index}.pcd'
        write_pcd(path, np.array([[2.0, 0.0, 0.0, index]], dtype=np.float32))
        chunks.append(path)
    transforms = np.repeat(np.eye(4)[None, :, :], 2, axis=0)
    transforms[1, 0, 3] = 1.0
    config = copy.deepcopy(contract()['submap'])
    config.update({
        'anchor_radius': 1,
        'minimum_range_m': 0.0,
        'voxel_size_m': 0.01,
        'maximum_points': 10,
    })
    loader = MODULE.StreamingSubmapLoader(
        transforms, [0, 1], chunks, config, MODULE.MemoryGuard(1024.0))
    cloud = loader.build(0)
    assert np.allclose(np.sort(cloud[:, 0]), [2.0, 3.0])


def test_scan_context_similarity_is_rotation_invariant():
    sector_width = 2 * math.pi / 60.0
    angles = (np.arange(60) + 0.25) * sector_width
    radius = 5.0 + 0.7 * np.sin(3 * angles)
    points = np.column_stack((
        radius * np.cos(angles), radius * np.sin(angles),
        np.linspace(-1.0, 2.0, len(angles)), np.ones(len(angles))))
    rotation = np.array([
        [math.cos(7 * sector_width), -math.sin(7 * sector_width), 0, 0],
        [math.sin(7 * sector_width), math.cos(7 * sector_width), 0, 0],
        [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float64)
    rotated = points.copy()
    rotated[:, :3] = transform_xyz(points[:, :3], rotation)
    config = contract()['descriptor']
    first = MODULE.scan_context_descriptor(points, config)
    second = MODULE.scan_context_descriptor(rotated, config)
    spectra = np.stack((
        np.fft.rfft(first, axis=-1).astype(np.complex64),
        np.fft.rfft(second, axis=-1).astype(np.complex64)))
    index = MODULE.DescriptorIndex(
        spectra, np.array([np.linalg.norm(first), np.linalg.norm(second)]), 60, [60, 60])
    assert index.similarity(0, 1) > 0.999


def test_icp_recovers_source_to_target_transform():
    generator = np.random.default_rng(42)
    xyz = generator.uniform([-2, -1, -0.5], [3, 2, 1.5], size=(600, 3))
    source = np.column_stack((xyz, np.ones(len(xyz))))
    angle = math.radians(4.0)
    expected = np.array([
        [math.cos(angle), -math.sin(angle), 0, 0.2],
        [math.sin(angle), math.cos(angle), 0, -0.1],
        [0, 0, 1, 0.05], [0, 0, 0, 1]], dtype=np.float64)
    target = source.copy()
    target[:, :3] = transform_xyz(source[:, :3], expected)
    config = copy.deepcopy(contract()['registration'])
    config.update({
        'minimum_correspondences': 100,
        'minimum_correspondence_ratio': 0.5,
        'maximum_correction_rotation_deg': 10.0,
        'overlap_distance_m': 0.05,
        'minimum_pair_mutual_overlap': 0.95,
        'maximum_support_rmse_m': 0.05,
        'maximum_support_p90_m': 0.05,
    })
    result = MODULE.register_clouds(source, target, np.eye(4), config)
    assert result['passed'], result
    assert np.allclose(result['target_from_source'], expected, atol=1e-3)


def record(source: int, target: int, overlap: float) -> dict:
    constraint = {
        'source_anchor': source,
        'target_anchor': target,
        'measurement_matrix': np.eye(4).tolist(),
    }
    return {
        'source_anchor': source,
        'target_anchor': target,
        'sequence_similarity': 0.9,
        'geometry': {
            'passed': True,
            'minimum_mutual_overlap': overlap,
            'pairs': [{'support_rmse_m': 0.1}],
            'constraint': constraint,
        },
    }


def test_duplicate_edges_are_canonical_and_input_order_independent():
    values = [record(10, 100, 0.7), record(12, 103, 0.8), record(30, 130, 0.75)]
    first = MODULE.deduplicate_constraints(copy.deepcopy(values), 8)
    second = MODULE.deduplicate_constraints(copy.deepcopy(list(reversed(values))), 8)
    assert first == second
    assert [(item['source_anchor'], item['target_anchor']) for item in first] == [
        (12, 103), (30, 130)]


def test_disconnected_graph_is_detected():
    assert MODULE.graph_connected(4, [(0, 1), (1, 2), (2, 3)])
    assert not MODULE.graph_connected(4, [(0, 1), (2, 3)])
    with pytest.raises(MODULE.ContractError, match='out of range'):
        MODULE.graph_connected(3, [(0, 3)])


def test_memory_exhaustion_fails_closed():
    guard = MODULE.MemoryGuard(1024.0)
    guard.maximum_rss_mib = 0.001
    with pytest.raises(MODULE.MemoryBudgetError, match='exceeds'):
        guard.check('synthetic_exhaustion')


def test_legacy_challenge_matching_uses_frozen_dedup_window():
    constraints = [
        {'source_anchor': 100, 'target_anchor': 200},
        {'source_anchor': 300, 'target_anchor': 400},
    ]
    survivors = MODULE.legacy_survivors(constraints, [(95, 205), (1, 2)], 8)
    assert survivors == [{
        'source_anchor': 100,
        'target_anchor': 200,
        'matching_legacy_edges': [[95, 205]],
    }]


def fake_report(contract_sha: str, repetition: int, deterministic: dict) -> dict:
    return {
        'status': 'PASS',
        'sequence_id': 'synthetic',
        'repetition': repetition,
        'contract_sha256': contract_sha,
        'deterministic': deterministic,
        'deterministic_payload_sha256': MODULE.payload_sha256(deterministic),
    }


def test_aggregate_requires_integrity_and_repeatability(tmp_path):
    _, contract_sha = MODULE.load_contract(CONTRACT_PATH)
    deterministic = {
        'candidate': {'episode_query_count': 1, 'descriptor_qualified_count': 1},
        'constraints': [
            {'source_anchor': 1, 'target_anchor': 10},
            {'source_anchor': 20, 'target_anchor': 30},
        ],
        'legacy_challenge': {'survivors': []},
    }
    paths = []
    for repetition in (1, 2):
        path = tmp_path / f'report{repetition}.json'
        path.write_text(json.dumps(fake_report(contract_sha, repetition, deterministic)),
                        encoding='utf-8')
        paths.append(path)
    output = tmp_path / 'aggregate.json'
    result = MODULE.aggregate_reports(
        contract_path=CONTRACT_PATH, reports=paths,
        expected_sequences=['synthetic'], output=output)
    assert result['status'] == 'PASS'
    assert result['external_sparse_pose_graph_authorized'] is True

    changed = copy.deepcopy(deterministic)
    changed['candidate']['episode_query_count'] = 2
    paths[1].write_text(json.dumps(fake_report(contract_sha, 2, changed)), encoding='utf-8')
    result = MODULE.aggregate_reports(
        contract_path=CONTRACT_PATH, reports=paths,
        expected_sequences=['synthetic'], output=output)
    assert result['status'] == 'FAIL'
    assert result['decision'] == 'REJECT_V42_INCOMPLETE_OR_NONREPEATABLE_AUDIT'
