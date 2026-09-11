"""Synthetic contracts for the v43 independent-signal availability audit."""

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
    'audit_v43_place_identity',
    ROOT / 'scripts/audit_v43_place_identity_feasibility.py')
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
CONTRACT_PATH = ROOT / 'configs/sota_v6/development/v43_place_identity_audit.json'


def contract():
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


def write_pcd(path: Path, intensity: np.ndarray) -> None:
    intensity = np.asarray(intensity, dtype='<f4')
    points = np.zeros((len(intensity), 4), dtype='<f4')
    points[:, 0] = np.arange(len(intensity), dtype=np.float32) + 1.0
    points[:, 3] = intensity
    header = (
        '# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n'
        'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n'
    ).encode('ascii')
    path.write_bytes(header + points.tobytes())


def write_states(path: Path, count: int) -> None:
    path.write_text(''.join(
        f'{index + 1} {index} 0 0 0 0 0 1\n' for index in range(count)),
        encoding='utf-8')


def test_contract_is_global_fixed_memory_and_two_repetitions():
    value, digest = MODULE.load_contract(CONTRACT_PATH)
    assert len(digest) == 64
    assert value['frames'] == {'world': 'map', 'body': 'base_link'}
    assert value['memory']['maximum_rss_mib'] == 128.0
    assert value['decision']['required_repetitions'] == 2
    serialized = MODULE.canonical_json(value).lower()
    assert all(dataset not in serialized for dataset in ('navinst', 'oxford', 'urbannav'))


def test_cli_has_no_accuracy_geometry_or_reference_surface():
    source = (ROOT / 'scripts/audit_v43_place_identity_feasibility.py').read_text(
        encoding='utf-8')
    for prohibited in ('--ground-truth', '--reference-map', '--trajectory', '--loop-edge'):
        assert prohibited not in source


def test_all_zero_intensity_fails_every_identity_gate_except_finite():
    accumulator = MODULE.IntensityAccumulator(contract()['intensity'])
    for _ in range(4):
        accumulator.add(np.zeros(1000, dtype=np.float32))
    result = accumulator.result()
    assert result['available'] is False
    assert result['point_count'] == 4000
    assert result['nonzero_count'] == 0
    assert result['dynamic_range'] == 0.0
    assert result['distinct_values_observed_up_to_cap'] == 1
    assert result['varying_chunk_count'] == 0
    assert result['checks']['finite_fraction'] is True


def test_varied_finite_intensity_passes_global_health_contract():
    accumulator = MODULE.IntensityAccumulator(contract()['intensity'])
    values = np.arange(32, dtype=np.float32)
    for shift in range(10):
        accumulator.add(values + shift)
    result = accumulator.result()
    assert result['available'] is True
    assert result['nonzero_fraction'] > 0.99
    assert result['dynamic_range'] == 40.0
    assert result['distinct_values_observed_up_to_cap'] == 17
    assert result['varying_chunk_fraction'] == 1.0


def test_nonfinite_intensity_fails_closed():
    accumulator = MODULE.IntensityAccumulator(contract()['intensity'])
    values = np.arange(32, dtype=np.float32)
    values[0] = math.nan
    accumulator.add(values)
    result = accumulator.result()
    assert result['available'] is False
    assert result['checks']['finite_fraction'] is False


def test_strict_pcd_parser_and_chunk_budget(tmp_path):
    path = tmp_path / '0.pcd'
    expected = np.arange(20, dtype=np.float32)
    write_pcd(path, expected)
    assert np.array_equal(MODULE.pcd_intensity(path, 4096), expected)
    with pytest.raises(MODULE.MemoryBudgetError):
        MODULE.pcd_intensity(path, 16)
    path.write_bytes(path.read_bytes()[:-1])
    with pytest.raises(MODULE.ContractError, match='byte count'):
        MODULE.pcd_intensity(path, 4096)


def test_state_parser_rejects_duplicate_time_and_zero_quaternion(tmp_path):
    path = tmp_path / 'states.txt'
    path.write_text('1 0 0 0 0 0 0 1\n1 0 0 0 0 0 0 1\n', encoding='utf-8')
    with pytest.raises(MODULE.ContractError, match='not increasing'):
        MODULE.load_state_count(path)
    path.write_text('1 0 0 0 0 0 0 0\n', encoding='utf-8')
    with pytest.raises(MODULE.ContractError, match='quaternion'):
        MODULE.load_state_count(path)


def test_camera_manifest_is_explicit_and_sequence_bound(tmp_path):
    config = contract()['camera_manifest']
    assert MODULE.camera_manifest_status(None, 'sequence', config) == {
        'available': False,
        'reason': 'camera_manifest_not_retained',
    }
    path = tmp_path / 'camera.json'
    manifest = {
        'schema_version': 1,
        'sequence_id': 'sequence',
        'frame_count': 100,
        'timestamps_sha256': 'a' * 64,
        'image_manifest_sha256': 'b' * 64,
    }
    path.write_text(json.dumps(manifest), encoding='utf-8')
    assert MODULE.camera_manifest_status(path, 'sequence', config)['available'] is True
    manifest['sequence_id'] = 'wrong'
    path.write_text(json.dumps(manifest), encoding='utf-8')
    result = MODULE.camera_manifest_status(path, 'sequence', config)
    assert result['available'] is False
    assert 'sequence_id' in result['failed_checks']


def test_memory_exhaustion_fails_closed():
    guard = MODULE.MemoryGuard(1024.0)
    guard.maximum_rss_mib = 0.001
    with pytest.raises(MODULE.MemoryBudgetError, match='exceeds'):
        guard.check('synthetic_exhaustion')


def test_tiny_report_only_audit_preserves_inputs(tmp_path):
    voxel = tmp_path / 'voxel'
    voxel.mkdir()
    write_states(voxel / 'alidarState.txt', 2)
    write_pcd(voxel / '0.pcd', np.arange(32, dtype=np.float32))
    write_pcd(voxel / '1.pcd', np.arange(32, dtype=np.float32) + 1)
    protected_map = tmp_path / 'map.pcd'
    protected_map.write_bytes(b'protected-map')
    state_before = (voxel / 'alidarState.txt').read_bytes()
    map_before = protected_map.read_bytes()
    integration_contract = contract()
    # The combined v40-v43 pytest process can already exceed the standalone
    # audit ceiling before this test starts; resource exhaustion has its own
    # explicit contract above.
    integration_contract['memory']['maximum_rss_mib'] = 512.0
    integration_contract_path = tmp_path / 'integration_contract.json'
    integration_contract_path.write_text(
        json.dumps(integration_contract), encoding='utf-8')
    output = tmp_path / 'report.json'
    result = MODULE.audit_sequence(
        contract_path=integration_contract_path,
        voxel_dir=voxel,
        sequence_id='synthetic',
        repetition=1,
        output=output,
        protected_map=protected_map,
        camera_manifest=None,
    )
    assert result['status'] == 'PASS', result
    assert result['deterministic']['intensity']['available'] is True
    assert result['deterministic']['independent_place_identity_signal_available'] is True
    assert (voxel / 'alidarState.txt').read_bytes() == state_before
    assert protected_map.read_bytes() == map_before


def fake_report(contract_sha: str, repetition: int, deterministic: dict) -> dict:
    return {
        'status': 'PASS',
        'sequence_id': 'synthetic',
        'repetition': repetition,
        'contract_sha256': contract_sha,
        'deterministic': deterministic,
        'deterministic_payload_sha256': MODULE.payload_sha256(deterministic),
    }


def test_aggregate_closes_route_repeatably_when_signal_is_missing(tmp_path):
    _, contract_sha = MODULE.load_contract(CONTRACT_PATH)
    deterministic = {
        'independent_place_identity_signal_available': False,
        'intensity': {
            'available': False,
            'point_count': 100,
            'nonzero_count': 0,
            'dynamic_range': 0.0,
            'intensity_payload_sha256': '0' * 64,
        },
        'camera': {'available': False},
    }
    paths = []
    for repetition in (1, 2):
        path = tmp_path / f'run_{repetition}.json'
        path.write_text(json.dumps(fake_report(contract_sha, repetition, deterministic)),
                        encoding='utf-8')
        paths.append(path)
    output = tmp_path / 'aggregate.json'
    result = MODULE.aggregate_reports(
        contract_path=CONTRACT_PATH,
        reports=paths,
        expected_sequences=['synthetic'],
        output=output,
    )
    assert result['status'] == 'PASS'
    assert result['decision'] == (
        'CLOSE_OXFORD_GLOBAL_CORRECTION_ROUTE_NO_INDEPENDENT_PLACE_IDENTITY')
    assert result['oxford_global_correction_route_open'] is False


def test_aggregate_rejects_nonrepeatable_payload(tmp_path):
    _, contract_sha = MODULE.load_contract(CONTRACT_PATH)
    first = {'independent_place_identity_signal_available': False,
             'intensity': {}, 'camera': {}}
    second = copy.deepcopy(first)
    second['intensity']['point_count'] = 1
    paths = []
    for repetition, deterministic in ((1, first), (2, second)):
        path = tmp_path / f'run_{repetition}.json'
        path.write_text(json.dumps(fake_report(contract_sha, repetition, deterministic)),
                        encoding='utf-8')
        paths.append(path)
    result = MODULE.aggregate_reports(
        contract_path=CONTRACT_PATH,
        reports=paths,
        expected_sequences=['synthetic'],
        output=tmp_path / 'aggregate.json',
    )
    assert result['status'] == 'FAIL'
    assert result['decision'] == 'REJECT_V43_INCOMPLETE_OR_NONREPEATABLE_AUDIT'
