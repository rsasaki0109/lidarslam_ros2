"""Unit and fail-closed tests for the v44e read-only replay adapter."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
ADAPTER_PATH = ROOT / 'scripts/v44_raw_shadow_replay_adapter.py'
CORE_PATH = ROOT / 'scripts/v44_fixed_lag_shadow_estimator.py'
CONTRACT_PATH = (
    ROOT / 'configs/sota_v6/development/'
    'v44e_raw_shadow_replay_execution_contract.json')


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ADAPTER = load_module('v44_replay_adapter_test', ADAPTER_PATH)
CORE = load_module('v44_shadow_core_adapter_test', CORE_PATH)


def contract():
    return json.loads(CONTRACT_PATH.read_text(encoding='utf-8'))


SEALED_WORKSPACE = (
    Path(contract()['execution']['required_contract_path']).resolve()
    == CONTRACT_PATH.resolve()
)
SEALED_TESTS = {
    'test_contract_authority_and_cli_are_narrow',
    'test_runtime_context_fails_before_any_raw_bag_can_open',
}


@pytest.fixture(autouse=True)
def require_sealed_workspace(request):
    if not SEALED_WORKSPACE and request.node.name in SEALED_TESTS:
        pytest.skip('requires the original hash-sealed v44e workspace')


def source(index=0):
    return contract()['source_binding']['sequences'][index]


def vector(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def header(stamp_ns, frame_id):
    return SimpleNamespace(
        stamp=SimpleNamespace(
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000),
        frame_id=frame_id)


def point_message(value=None, source_value=None, contract_value=None):
    contract_value = contract_value or contract()
    source_value = source_value or source()
    messages = contract_value['messages']
    dtype = ADAPTER.point_dtype(messages)
    points = np.zeros(2, dtype=dtype) if value is None else value
    if value is None:
        points['x'] = [1.0, 2.0]
        points['y'] = [0.5, -0.5]
        points['z'] = [0.25, 0.75]
        points['t'] = [0, 100_000_000]
        points['ring'] = [1, 2]
    fields = [SimpleNamespace(**item)
              for item in messages['required_lidar_fields']]
    return SimpleNamespace(
        header=header(10_000_000_000, source_value['lidar_frame']),
        height=1, width=len(points), fields=fields, is_bigendian=False,
        point_step=48, row_step=48 * len(points),
        data=points.view(np.uint8), is_dense=True)


def imu_message(source_value=None):
    source_value = source_value or source()
    return SimpleNamespace(
        header=header(9_950_000_000, source_value['imu_frame']),
        angular_velocity=vector(0.1, -0.2, 0.3),
        linear_acceleration=vector(1.0, 2.0, 9.6),
        orientation=SimpleNamespace(x=float('nan')),
        orientation_covariance=[float('nan')] * 9,
        angular_velocity_covariance=[float('nan')] * 9,
        linear_acceleration_covariance=[float('nan')] * 9)


def authorization_payload(contract_value=None):
    contract_value = contract_value or contract()
    value = {
        'audit': contract_value['authorization']['required_aggregate_audit'],
        'contract_id': contract_value['contract_id'],
        'contract_sha256': 'a' * 64,
        'adapter_sha256': contract_value['adapter']['sha256'],
        'auditor_sha256': contract_value['static_auditor']['sha256'],
        'status': 'PASS',
        'decision': ADAPTER.REQUIRED_AUTHORIZATION_DECISION,
        'deterministic': {'complete': True},
        'raw_shadow_replay_execution_authorized': True,
        'raw_replay_executed': False,
        'accuracy_or_reference_map_inputs_authorized': False,
        'primary_trajectory_or_map_mutation_authorized': False,
        'ros_publication_authorized': False,
    }
    value['aggregate_payload_sha256'] = ADAPTER.payload_sha256(
        value['deterministic'])
    return value


def buffered(stamp, kind, index, receive=None, size=10, message=None):
    return ADAPTER.BufferedSensorMessage(
        (stamp, ADAPTER.EVENT_KIND_ORDER[kind], index),
        stamp if receive is None else receive, kind, index, size,
        object() if message is None else message)


def test_contract_authority_and_cli_are_narrow():
    value = contract()
    ADAPTER.validate_contract_for_runtime(value, CONTRACT_PATH.resolve())
    assert value['authority'] == ADAPTER.ADAPTER_AUTHORITY
    assert value['source_binding']['required_sequence_count'] == 3
    parser = ADAPTER.build_parser()
    choices = parser._subparsers._group_actions[0].choices
    assert set(choices) == {'replay'}
    options = {item for action in choices['replay']._actions
               for item in action.option_strings}
    assert options == {
        '-h', '--help', '--contract', '--sequence-id', '--repetition'}
    assert '--bag' not in ADAPTER_PATH.read_text(encoding='utf-8')
    assert '--output' not in ADAPTER_PATH.read_text(encoding='utf-8')


def test_canonical_pointcloud_decodes_exact_core_records():
    value = contract()
    message = point_message(contract_value=value)
    scan = ADAPTER.decode_lidar_message(
        message, source(), value['messages'], CORE, 4, 512,
        value['messages']['maximum_message_bytes'])
    assert isinstance(scan, CORE.LidarScan)
    assert scan.scan_index == 4
    assert scan.header_stamp_ns == 10_000_000_000
    assert scan.end_ns == 10_100_000_000
    assert [item.point_L_m for item in scan.points] == [
        (1.0, 0.5, 0.25), (2.0, -0.5, 0.75)]
    assert [item.ring for item in scan.points] == [1, 2]
    assert [item.source_index for item in scan.points] == [0, 1]


def test_imu_decode_uses_only_header_gyro_and_acceleration():
    value = contract()
    decoded = ADAPTER.decode_imu_message(
        imu_message(), source(), CORE, 12, 300,
        value['messages']['maximum_message_bytes'])
    assert isinstance(decoded, CORE.ImuSample)
    assert decoded.timestamp_ns == 9_950_000_000
    assert decoded.angular_velocity_B_rad_s == (0.1, -0.2, 0.3)
    assert decoded.linear_acceleration_B_m_s2 == (1.0, 2.0, 9.6)
    assert decoded.source_index == 12


@pytest.mark.parametrize('mutation, message', [
    (lambda item: setattr(item, 'is_bigendian', True), 'endianness'),
    (lambda item: setattr(item, 'height', 2), 'height'),
    (lambda item: setattr(item, 'point_step', 32), 'point_step'),
    (lambda item: setattr(item, 'row_step', 1), 'contiguous'),
    (lambda item: setattr(item.header, 'frame_id', 'wrong'), 'frame'),
    (lambda item: item.fields.pop(), 'fields'),
])
def test_pointcloud_contract_mutations_are_rejected(mutation, message):
    value = contract()
    item = point_message(contract_value=value)
    mutation(item)
    with pytest.raises(ADAPTER.ContractError, match=message):
        ADAPTER.decode_lidar_message(
            item, source(), value['messages'], CORE, 0, 512,
            value['messages']['maximum_message_bytes'])


def test_nonfinite_sensor_values_and_bad_header_are_rejected():
    value = contract()
    item = imu_message()
    item.angular_velocity.x = float('nan')
    with pytest.raises(ADAPTER.ContractError, match='non-finite'):
        ADAPTER.decode_imu_message(
            item, source(), CORE, 0, 200,
            value['messages']['maximum_message_bytes'])
    item = imu_message()
    item.header.stamp.nanosec = 1_000_000_000
    with pytest.raises(ADAPTER.ContractError, match='nanoseconds'):
        ADAPTER.decode_imu_message(
            item, source(), CORE, 0, 200,
            value['messages']['maximum_message_bytes'])


def test_message_capacity_is_checked_before_decode():
    value = contract()
    maximum = value['messages']['maximum_message_bytes']
    with pytest.raises(ADAPTER.ContractError, match='serialized size'):
        ADAPTER.decode_lidar_message(
            point_message(contract_value=value), source(), value['messages'],
            CORE, 0, maximum + 1, maximum)
    with pytest.raises(ADAPTER.ContractError, match='serialized size'):
        ADAPTER.decode_imu_message(
            imu_message(), source(), CORE, 0, 0, maximum)


def test_reorder_restores_header_kind_and_index_order():
    reorder = ADAPTER.DeterministicEventReorder(60, 16, 1024)
    inputs = [
        buffered(100, 'lidar', 0, 140),
        buffered(90, 'imu', 0, 141),
        buffered(100, 'imu', 1, 145),
        buffered(160, 'imu', 2, 160),
    ]
    output = []
    for item in inputs:
        output.extend(reorder.push(item))
    output.extend(reorder.finish())
    assert [item.sort_key for item in output] == [
        (90, 0, 0), (100, 0, 1), (100, 1, 0), (160, 0, 2)]
    assert reorder.peak_messages <= 4
    assert reorder.peak_bytes <= 40


def test_reorder_rejects_delay_receive_order_key_and_capacities():
    reorder = ADAPTER.DeterministicEventReorder(10, 2, 20)
    with pytest.raises(ADAPTER.ContractError, match='delay'):
        reorder.push(buffered(1, 'imu', 0, 12))
    reorder.push(buffered(10, 'imu', 0, 10, size=10))
    with pytest.raises(ADAPTER.ContractError, match='receive timestamps'):
        reorder.push(buffered(9, 'imu', 1, 9, size=10))
    bad_key = ADAPTER.BufferedSensorMessage(
        (11, 1, 1), 11, 'imu', 1, 1, object())
    with pytest.raises(ADAPTER.ContractError, match='sort key'):
        reorder.push(bad_key)
    with pytest.raises(ADAPTER.CapacityError, match='byte'):
        reorder.push(buffered(11, 'imu', 1, 11, size=11))
    reorder.push(buffered(11, 'imu', 1, 11, size=10))
    with pytest.raises(ADAPTER.CapacityError, match='message'):
        reorder.push(buffered(12, 'imu', 2, 12, size=1))


@pytest.mark.parametrize('field,value,match', [
    ('status', 'FAIL', 'did not pass'),
    ('decision', 'REJECT', 'decision differs'),
    ('contract_sha256', 'b' * 64, 'contract SHA-256'),
    ('adapter_sha256', 'b' * 64, 'adapter SHA-256'),
    ('raw_shadow_replay_execution_authorized', False, 'not authorized'),
    ('raw_replay_executed', True, 'must not contain a replay'),
    ('accuracy_or_reference_map_inputs_authorized', True, 'unexpectedly opens'),
])
def test_authorization_mutations_fail_closed(field, value, match):
    contract_value = contract()
    authorization = authorization_payload(contract_value)
    authorization[field] = value
    with pytest.raises(ADAPTER.ContractError, match=match):
        ADAPTER.validate_authorization_payload(
            authorization, contract_value, 'a' * 64,
            contract_value['adapter']['sha256'])


def test_valid_authorization_payload_passes():
    contract_value = contract()
    ADAPTER.validate_authorization_payload(
        authorization_payload(contract_value), contract_value, 'a' * 64,
        contract_value['adapter']['sha256'])


def test_authorization_revalidates_both_hash_bound_source_reports(tmp_path):
    contract_value = contract()
    contract_value['output']['evidence_root'] = str(tmp_path.resolve())
    deterministic = {
        'raw_replay_executed': False, 'raw_bag_opened': False,
        'validated': True}
    report_payload = ADAPTER.payload_sha256(deterministic)
    bindings = []
    for repetition in (1, 2):
        report = {
            'audit': 'v44e_raw_shadow_replay_contract_static_validation',
            'contract_id': contract_value['contract_id'],
            'contract_sha256': 'a' * 64,
            'adapter_sha256': contract_value['adapter']['sha256'],
            'auditor_sha256': contract_value['static_auditor']['sha256'],
            'status': 'PASS',
            'decision': ADAPTER.REQUIRED_AUTHORIZATION_DECISION,
            'repetition': repetition,
            'deterministic': deterministic,
            'report_payload_sha256': report_payload,
            'raw_shadow_replay_execution_authorized': True,
        }
        path = tmp_path / f'run_{repetition:02d}.json'
        path.write_text(json.dumps(report), encoding='utf-8')
        bindings.append({
            'path': str(path.resolve()),
            'sha256': ADAPTER.sha256_file(path)})
    aggregate = {
        'deterministic': {'report_payload_sha256': report_payload},
        'source_reports': bindings}
    ADAPTER.validate_authorization_source_reports(
        aggregate, contract_value, 'a' * 64,
        contract_value['adapter']['sha256'])
    (tmp_path / 'run_02.json').write_text('{}', encoding='utf-8')
    with pytest.raises(ADAPTER.ContractError, match='SHA-256 differs'):
        ADAPTER.validate_authorization_source_reports(
            aggregate, contract_value, 'a' * 64,
            contract_value['adapter']['sha256'])


def test_source_and_run_directory_are_contract_derived(tmp_path):
    value = contract()
    assert ADAPTER.source_binding(value, 'navinst_indoor02')['bag']['sha256'] == (
        'b8afd9649a310669dcc39737a8f3e1d40f00adfa619ff61afda712c2924fc8ff')
    with pytest.raises(ADAPTER.ContractError, match='source binding'):
        ADAPTER.source_binding(value, 'unknown')
    value['output']['evidence_root'] = str(tmp_path.resolve())
    expected = ADAPTER.expected_run_directory(
        value, 'navinst_indoor02', 2)
    assert expected == (
        tmp_path.resolve() / 'raw_replay/navinst_indoor02/run_02')
    with pytest.raises(ADAPTER.ContractError, match='repetition'):
        ADAPTER.expected_run_directory(value, 'navinst_indoor02', 3)
    with pytest.raises(ADAPTER.ContractError, match='path-safe'):
        ADAPTER.expected_run_directory(value, '../escape', 1)


def test_all_three_extrinsics_are_valid_source_bindings():
    value = contract()
    translations = []
    for item in value['source_binding']['sequences']:
        transform = ADAPTER.body_from_lidar(item, CORE)
        rotation, translation = transform.arrays()
        assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        assert np.isclose(np.linalg.det(rotation), 1.0, atol=1e-12)
        translations.append(tuple(translation))
    assert translations == [
        (0.0, 0.0, 0.21941),
        (-0.0006517086833720872, -0.015510342560109067,
         0.08463265780318904),
        (0.0, 0.0, 0.28),
    ]


def test_protected_artifacts_are_hashed_and_mutation_is_rejected(tmp_path):
    state = tmp_path / 'state'
    map_path = tmp_path / 'map'
    state.write_bytes(b'state')
    map_path.write_bytes(b'map')
    item = {
        'protected_v17_artifacts': [
            {'name': 'v17_state', 'path': str(state.resolve()),
             'sha256': ADAPTER.sha256_file(state), 'bytes': state.stat().st_size},
            {'name': 'v17_map', 'path': str(map_path.resolve()),
             'sha256': ADAPTER.sha256_file(map_path),
             'bytes': map_path.stat().st_size},
        ]}
    before = ADAPTER.hash_protected_artifacts(item)
    assert set(before) == {'v17_map', 'v17_state'}
    state.write_bytes(b'changed')
    with pytest.raises(ADAPTER.ContractError, match='byte size differs'):
        ADAPTER.hash_protected_artifacts(item)


def test_bound_file_rejects_symlink_even_with_matching_content(tmp_path):
    target = tmp_path / 'target'
    link = tmp_path / 'link'
    target.write_bytes(b'x')
    link.symlink_to(target)
    with pytest.raises(ADAPTER.ContractError, match='symlink'):
        ADAPTER.verify_bound_file(
            link, ADAPTER.sha256_file(target), 1, 'test file')


def test_evidence_writer_is_exclusive_bounded_and_layout_checked(tmp_path):
    root = tmp_path.resolve()
    run = root / 'raw_replay' / 'sequence' / 'run_01'
    writer = ADAPTER.BoundedEvidenceWriter(run, root, 1024)
    writer.write_diagnostics([{'record_type': 'scan', 'scan_index': 0}])
    writer.write_report({'status': 'PASS'})
    assert sorted(path.name for path in run.iterdir()) == [
        'diagnostics.jsonl', 'run.json']
    with pytest.raises(ADAPTER.ContractError, match='overwrite'):
        writer.write_report({'status': 'PASS'})
    with pytest.raises(ADAPTER.ContractError, match='already exists'):
        ADAPTER.BoundedEvidenceWriter(run, root, 1024)
    another = root / 'raw_replay' / 'sequence' / 'run_02'
    tiny = ADAPTER.BoundedEvidenceWriter(another, root, 1)
    with pytest.raises(ADAPTER.ContractError, match='byte capacity'):
        tiny.write_report({'status': 'PASS'})
    with pytest.raises(ADAPTER.ContractError, match='outside'):
        ADAPTER.BoundedEvidenceWriter(root / 'wrong/run_03', root, 1024)


def test_evidence_writer_rejects_symlinked_parent(tmp_path):
    root = tmp_path.resolve()
    outside = tmp_path / 'outside'
    outside.mkdir()
    (root / 'raw_replay').symlink_to(outside, target_is_directory=True)
    with pytest.raises(ADAPTER.ContractError, match='real directory'):
        ADAPTER.BoundedEvidenceWriter(
            root / 'raw_replay/sequence/run_01', root, 1024)


def test_runtime_observation_is_attached_once_per_new_scan(monkeypatch):
    class FakeEstimator:
        def __init__(self):
            self.diagnostics = [
                {'record_type': 'scan', 'scan_index': 1,
                 'scan_end_ns': 2_000_000_000},
                {'record_type': 'terminal', 'status': 'PASS'},
            ]
            self.calls = []

        def record_runtime_observation(self, **kwargs):
            self.calls.append(kwargs)

    estimator = FakeEstimator()
    observed = set()
    monkeypatch.setattr(ADAPTER, 'current_rss_mib', lambda: 42.0)
    monkeypatch.setattr(ADAPTER.time, 'perf_counter', lambda: 3.0)
    ADAPTER.record_runtime_after_scans(
        estimator, observed, processing_started=2.0,
        sensor_origin_ns=1_000_000_000)
    ADAPTER.record_runtime_after_scans(
        estimator, observed, processing_started=2.0,
        sensor_origin_ns=1_000_000_000)
    assert observed == {1}
    assert estimator.calls == [{
        'scan_index': 1, 'rss_mib': 42.0, 'processing_seconds': 1.0,
        'sensor_duration_seconds': 1.0}]


def test_runtime_context_fails_before_any_raw_bag_can_open():
    aggregate = Path(contract()['authorization']['required_aggregate_path'])
    if aggregate.exists():
        value, digest, authorization, _ = ADAPTER.load_runtime_context(
            CONTRACT_PATH)
        assert value['contract_id'] == ADAPTER.EXECUTION_CONTRACT_ID
        assert len(digest) == 64
        assert authorization['raw_shadow_replay_execution_authorized'] is True
        assert authorization['raw_replay_executed'] is False
        return
    with pytest.raises(ADAPTER.ContractError, match='cannot read JSON'):
        ADAPTER.load_runtime_context(CONTRACT_PATH)
