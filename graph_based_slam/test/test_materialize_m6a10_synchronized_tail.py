# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Synthetic contract tests for synchronized-tail bag materialization."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from rosbags.interfaces import (
    Qos,
    QosDurability,
    QosHistory,
    QosLiveliness,
    QosReliability,
    QosTime,
)
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_typestore, Stores


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'materialize_m6a10_synchronized_tail.py'
SPEC = importlib.util.spec_from_file_location('materialize_m6a10_tail', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

ANALYZER_SCRIPT = ROOT / 'scripts' / 'analyze_m6a10_synchronized_tail.py'
ANALYZER_SPEC = importlib.util.spec_from_file_location(
    'analyze_m6a10_synchronized_tail_for_materializer', ANALYZER_SCRIPT)
ANALYZER = importlib.util.module_from_spec(ANALYZER_SPEC)
assert ANALYZER_SPEC.loader is not None
ANALYZER_SPEC.loader.exec_module(ANALYZER)


def _key(index: int) -> tuple:
    return (index, index + 1, index + 2, index + 3, f'{index:064x}')


def _record(topic: str, index: int, *, tail_key: tuple | None = None) -> dict:
    return {
        'connection_id': 1 if topic == '/imu/imu' else 2,
        'topic': topic,
        'msgtype': 'sensor_msgs/msg/Imu' if topic == '/imu/imu'
        else 'sensor_msgs/msg/PointCloud2',
        'timestamp_ns': index,
        'raw': f'{topic}:{index}'.encode(),
        'tail_key': tail_key,
    }


def _write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True) + '\n', encoding='utf-8')


def _make_ros2_bag(tmp_path: Path, lidar_headers: list[int]) -> tuple[Path, Path]:
    """Create a tiny real rosbag2 plus its analyzer receipt."""
    typestore = get_typestore(Stores.LATEST)
    types = typestore.types
    time_type = types['builtin_interfaces/msg/Time']
    header_type = types['std_msgs/msg/Header']
    imu_type = types['sensor_msgs/msg/Imu']
    quaternion_type = types['geometry_msgs/msg/Quaternion']
    vector_type = types['geometry_msgs/msg/Vector3']
    cloud_type = types['sensor_msgs/msg/PointCloud2']
    field_type = types['sensor_msgs/msg/PointField']
    bag = tmp_path / 'input_bag'

    def imu_message(timestamp: int):
        header = header_type(
            time_type(timestamp // 1_000_000_000, timestamp % 1_000_000_000), 'imu')
        vector = vector_type(0.0, 0.0, 0.0)
        quaternion = quaternion_type(0.0, 0.0, 0.0, 1.0)
        covariance = np.zeros(9, dtype=np.float64)
        return imu_type(
            header, quaternion, covariance, vector, covariance, vector, covariance)

    def cloud_message(timestamp: int):
        header = header_type(
            time_type(timestamp // 1_000_000_000, timestamp % 1_000_000_000), 'lidar')
        payload = np.asarray([1], dtype=np.uint32).tobytes()
        return cloud_type(
            header, 1, 1, [field_type('t', 0, 6, 1)], False, 4, 4,
            np.frombuffer(payload, dtype=np.uint8), True)

    with Writer(bag, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
        imu_connection = writer.add_connection(
            '/imu/imu', 'sensor_msgs/msg/Imu', typestore=typestore)
        lidar_connection = writer.add_connection(
            '/os1_cloud_node1/points', 'sensor_msgs/msg/PointCloud2', typestore=typestore)
        for timestamp in (100, 300):
            writer.write(
                imu_connection, timestamp,
                typestore.serialize_cdr(imu_message(timestamp), 'sensor_msgs/msg/Imu'))
        for timestamp in lidar_headers:
            writer.write(
                lidar_connection, timestamp,
                typestore.serialize_cdr(
                    cloud_message(timestamp), 'sensor_msgs/msg/PointCloud2'))

    result = ANALYZER.analyze_bag(
        bag, '/os1_cloud_node1/points', '/imu/imu')
    receipt_path = tmp_path / 'analyzer.json'
    receipt_path.write_text(
        json.dumps(ANALYZER.build_receipt(result, receipt_path), sort_keys=True) + '\n',
        encoding='utf-8')
    return bag, receipt_path


def _analyzer_receipt(input_root: Path, *, terminal: list[dict] | None = None) -> dict:
    tree = MODULE.sha256_tree(input_root)
    analyzer = ROOT / 'scripts' / 'analyze_m6a10_synchronized_tail.py'
    terminal = [] if terminal is None else terminal
    return {
        'schema_version': 1,
        'contract_id': 'm6a10-v2a-synchronized-tail-v1',
        'status': 'PASS',
        'mode': 'dry_run',
        'generated_by': {
            'path': 'scripts/analyze_m6a10_synchronized_tail.py',
            'sha256': hashlib.sha256(analyzer.read_bytes()).hexdigest(),
        },
        'input': {'path': str(input_root.resolve()), 'tree': tree},
        'topics': {
            'lidar': {
                'topic': '/os1_cloud_node1/points',
                'type': 'sensor_msgs/msg/PointCloud2',
            },
            'imu': {'topic': '/imu/imu', 'type': 'sensor_msgs/msg/Imu'},
        },
        'tail_evaluation': {
            'comparator': 'point_timestamp_max_ns < last_imu_header_timestamp_ns',
            'comparison_is_strict': True,
            'ineligible_lidar_count': len(terminal),
            'terminal_lidar': terminal,
        },
        'safety': {
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'output_bag_written': False,
        },
    }


def test_zero_drop_and_interleaved_order_are_preserved():
    records = [
        _record('/imu/imu', 0),
        _record('/os1_cloud_node1/points', 1),
        _record('/imu/imu', 2),
    ]
    kept, dropped = MODULE.select_records(records, [])
    assert kept == records
    assert dropped == []


def test_one_and_multiple_terminal_rows_are_the_only_drops():
    first, second = _key(1), _key(3)
    records = [
        _record('/imu/imu', 0),
        _record('/os1_cloud_node1/points', 1, tail_key=first),
        _record('/imu/imu', 2),
        _record('/os1_cloud_node1/points', 3, tail_key=second),
        _record('/imu/imu', 4),
    ]
    kept, dropped = MODULE.select_records(records, [first, second])
    assert [row['timestamp_ns'] for row in kept] == [0, 2, 4]
    assert [row['timestamp_ns'] for row in dropped] == [1, 3]


def test_missing_or_duplicate_terminal_identity_fails_closed():
    key = _key(1)
    with pytest.raises(MODULE.MaterializationError, match='not found'):
        MODULE.select_records([_record('/imu/imu', 0)], [key])
    with pytest.raises(MODULE.MaterializationError, match='occurred twice'):
        MODULE.select_records([
            _record('/os1_cloud_node1/points', 1, tail_key=key),
            _record('/os1_cloud_node1/points', 1, tail_key=key),
        ], [key])


def test_ordered_payload_hash_detects_reordering():
    records = [_record('/imu/imu', 0), _record('/imu/imu', 1)]
    forward = MODULE.ordered_payload_stream_sha256(records)
    reverse = MODULE.ordered_payload_stream_sha256(list(reversed(records)))
    assert forward != reverse


def test_record_stream_tamper_is_rejected():
    expected = [_record('/imu/imu', 0), _record('/os1_cloud_node1/points', 1)]
    tampered = copy.deepcopy(expected)
    tampered[1]['raw'] = b'changed'
    with pytest.raises(MODULE.MaterializationError, match='mismatch'):
        MODULE.verify_record_stream(expected, tampered)


def test_record_verifier_consumes_single_pass_streams_without_materializing():
    record = _record('/imu/imu', 0)
    record['raw'] = b'x' * (2 * 1024 * 1024)

    class SinglePass:
        def __init__(self, value):
            self.value = value
            self.used = False

        def __iter__(self):
            if self.used:
                raise AssertionError('stream was iterated more than once')
            self.used = True
            yield self.value

    MODULE.verify_record_stream(SinglePass(record), SinglePass(record.copy()))


def test_path_overlap_existing_and_staging_are_rejected(tmp_path):
    input_root = tmp_path / 'input'
    input_root.mkdir()
    (input_root / 'metadata.yaml').write_text('metadata', encoding='utf-8')
    with pytest.raises(MODULE.MaterializationError, match='overlap'):
        MODULE.assert_output_paths(input_root, input_root / 'output', tmp_path / 'receipt.json')
    output = tmp_path / 'output'
    output.mkdir()
    with pytest.raises(MODULE.MaterializationError, match='output already exists'):
        MODULE.assert_output_paths(input_root, output, tmp_path / 'receipt.json')
    output.rmdir()
    (tmp_path / 'output.part').mkdir()
    with pytest.raises(MODULE.MaterializationError, match='staging'):
        MODULE.assert_output_paths(input_root, output, tmp_path / 'receipt.json')
    (tmp_path / 'output.part').rmdir()
    (tmp_path / 'output.staging').mkdir()
    with pytest.raises(MODULE.MaterializationError, match='staging'):
        MODULE.assert_output_paths(input_root, output, tmp_path / 'receipt.json')


def test_analyzer_receipt_requires_strict_unique_terminal_rows(tmp_path):
    input_root = tmp_path / 'input'
    input_root.mkdir()
    (input_root / 'metadata.yaml').write_text('metadata', encoding='utf-8')
    row = dict(zip(MODULE.TAIL_KEY_FIELDS, _key(1)))
    receipt = _analyzer_receipt(input_root, terminal=[row, row])
    with pytest.raises(MODULE.MaterializationError, match='ambiguous'):
        MODULE.validate_analyzer_receipt(receipt, input_root)
    receipt = _analyzer_receipt(input_root, terminal=[row])
    receipt['tail_evaluation']['comparison_is_strict'] = False
    with pytest.raises(MODULE.MaterializationError, match='strict'):
        MODULE.validate_analyzer_receipt(receipt, input_root)


def test_analyzer_receipt_path_and_tree_are_bound(tmp_path):
    input_root = tmp_path / 'input'
    input_root.mkdir()
    (input_root / 'metadata.yaml').write_text('metadata', encoding='utf-8')
    receipt = _analyzer_receipt(input_root)
    MODULE.validate_analyzer_receipt(receipt, input_root)
    (input_root / 'later').write_text('changed', encoding='utf-8')
    with pytest.raises(MODULE.MaterializationError, match='changed'):
        MODULE.validate_analyzer_receipt(receipt, input_root)


def test_ros1_semantic_report_requires_all_topics_equal(tmp_path):
    report = tmp_path / 'semantic.json'
    _write_json(report, {
        'schema_version': 1,
        'all_topics_equal': True,
        'topics': [{'topic': '/imu/imu', 'equal': True}],
    })
    assert MODULE.validate_ros1_semantic_report(report)['status'] == 'PASS'
    document = json.loads(report.read_text(encoding='utf-8'))
    document['all_topics_equal'] = False
    _write_json(report, document)
    with pytest.raises(MODULE.MaterializationError, match='not PASS'):
        MODULE.validate_ros1_semantic_report(report)


def test_receipt_external_to_output_and_rerun_is_rejected(tmp_path):
    input_root = tmp_path / 'input'
    input_root.mkdir()
    (input_root / 'metadata.yaml').write_text('metadata', encoding='utf-8')
    output = tmp_path / 'output'
    receipt = tmp_path / 'receipt.json'
    MODULE.assert_output_paths(input_root, output, receipt)
    output.mkdir()
    with pytest.raises(MODULE.MaterializationError, match='output already exists'):
        MODULE.assert_output_paths(input_root, output, receipt)


@pytest.mark.parametrize(
    ('lidar_headers', 'expected_drop'),
    [([200], 0), ([200, 300], 1), ([200, 300, 301], 2)],
)
def test_real_rosbags_writer_materializes_zero_one_multiple_drops(
        tmp_path, lidar_headers, expected_drop):
    bag, analyzer_receipt = _make_ros2_bag(tmp_path, lidar_headers)
    output = tmp_path / 'materialized'
    receipt = tmp_path / 'materialized-receipt.json'
    document = MODULE.materialize_bag(bag, output, receipt, analyzer_receipt)
    assert document['status'] == 'PASS'
    assert document['contract_id'] == MODULE.MATERIALIZATION_CONTRACT_ID
    assert document['analyzer']['contract_id'] == ANALYZER.CONTRACT_ID
    assert document['dropped_lidar']['count'] == expected_drop
    assert document['safety']['ground_truth_content_opened'] is False
    assert document['safety']['scorer_invoked'] is False
    assert document['connections']['output'] == document['connections']['source']
    assert document['verification'] == {
        'status': 'PASS',
        'method': 'independent_raw_stream_reread',
        'output_record_count': document['output']['record_count'],
        'dropped_count': expected_drop,
        'ordered_payload_stream_sha256': document['output'][
            'ordered_payload_stream_sha256'],
        'connection_metadata_equal': True,
    }
    assert document['order_definition'] == MODULE.ORDER_DEFINITION
    persisted = json.loads(receipt.read_text(encoding='utf-8'))
    assert persisted['receipt_path'] == str(receipt.resolve())
    assert persisted['contract_id'] == MODULE.MATERIALIZATION_CONTRACT_ID
    assert persisted['analyzer']['contract_id'] == ANALYZER.CONTRACT_ID
    assert persisted['verification']['status'] == 'PASS'
    assert persisted['order_definition'] == MODULE.ORDER_DEFINITION
    assert not output.with_name(output.name + '.part').exists()
    assert not output.with_name(output.name + '.staging').exists()
    assert not any('.part' in path.name or '.staging' in path.name
                   for path in output.iterdir())
    assert not (output / receipt.name).exists()


def test_real_materialization_rerun_and_tampered_analyzer_fail_closed(tmp_path):
    bag, analyzer_receipt = _make_ros2_bag(tmp_path, [200, 300])
    output = tmp_path / 'materialized'
    receipt = tmp_path / 'materialized-receipt.json'
    MODULE.materialize_bag(bag, output, receipt, analyzer_receipt)
    with pytest.raises(MODULE.MaterializationError, match='output already exists'):
        MODULE.materialize_bag(bag, output, tmp_path / 'rerun.json', analyzer_receipt)
    tampered = json.loads(analyzer_receipt.read_text(encoding='utf-8'))
    tampered['tail_evaluation']['comparison_is_strict'] = False
    bad_receipt = tmp_path / 'bad-analyzer.json'
    _write_json(bad_receipt, tampered)
    with pytest.raises(MODULE.MaterializationError, match='strict'):
        MODULE.materialize_bag(
            bag, tmp_path / 'tampered-output', tmp_path / 'tampered.json', bad_receipt)


def test_failed_verification_retains_staging_artifact(tmp_path, monkeypatch):
    bag, analyzer_receipt = _make_ros2_bag(tmp_path, [200, 300])
    output = tmp_path / 'materialized'
    receipt = tmp_path / 'materialized-receipt.json'

    def fail_verification(*args, **kwargs):
        raise MODULE.MaterializationError('forced_verify_failure', 'fixture failure')

    monkeypatch.setattr(MODULE, '_verify_bag_stream', fail_verification)
    with pytest.raises(MODULE.MaterializationError, match='fixture failure'):
        MODULE.materialize_bag(bag, output, receipt, analyzer_receipt)
    staging = output.with_name(output.name + '.staging')
    assert staging.is_dir()
    assert (staging / output.name).is_dir()
    assert not output.exists()
    assert not receipt.exists()


def test_verification_accumulator_mismatch_never_publishes_output(
        tmp_path, monkeypatch):
    bag, analyzer_receipt = _make_ros2_bag(tmp_path, [200, 300])
    output = tmp_path / 'materialized'
    receipt = tmp_path / 'materialized-receipt.json'
    original_verify = MODULE._verify_bag_stream

    def mismatch_verification(*args, **kwargs):
        result = original_verify(*args, **kwargs)
        result['output_records'] += 1
        return result

    monkeypatch.setattr(MODULE, '_verify_bag_stream', mismatch_verification)
    with pytest.raises(MODULE.MaterializationError, match='statistics differ'):
        MODULE.materialize_bag(bag, output, receipt, analyzer_receipt)
    staging = output.with_name(output.name + '.staging')
    assert not output.exists()
    assert staging.is_dir()
    assert (staging / output.name).is_dir()
    assert not receipt.exists()


def test_duplicate_connection_signature_is_fail_closed():
    def fake_connection(connection_id: int):
        return SimpleNamespace(
            id=connection_id,
            topic='/imu/imu',
            msgtype='sensor_msgs/msg/Imu',
            msgdef=SimpleNamespace(data='message definition'),
            digest='a' * 64,
            ext=SimpleNamespace(serialization_format='cdr', offered_qos_profiles=[]),
        )

    with pytest.raises(MODULE.MaterializationError, match='duplicate'):
        MODULE._assert_unique_connections([fake_connection(1), fake_connection(2)])


def test_real_writer_keeps_distinct_same_topic_connections_by_metadata(tmp_path):
    typestore = get_typestore(Stores.LATEST)
    qos_time = QosTime(0, 0)
    reliable = Qos(
        QosHistory.KEEP_LAST, 10, QosReliability.RELIABLE,
        QosDurability.VOLATILE, qos_time, qos_time,
        QosLiveliness.AUTOMATIC, qos_time, False)
    best_effort = Qos(
        QosHistory.KEEP_LAST, 10, QosReliability.BEST_EFFORT,
        QosDurability.VOLATILE, qos_time, qos_time,
        QosLiveliness.AUTOMATIC, qos_time, False)
    bag = tmp_path / 'same-topic-bag'
    with Writer(bag, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
        first = writer.add_connection(
            '/imu/imu', 'sensor_msgs/msg/Imu', typestore=typestore,
            offered_qos_profiles=[reliable])
        second = writer.add_connection(
            '/imu/imu', 'sensor_msgs/msg/Imu', typestore=typestore,
            offered_qos_profiles=[best_effort])
        assert first.id != second.id
        assert MODULE._assert_unique_connections([first, second])


def test_ros1_conversion_and_semantic_argv_are_pinned(tmp_path):
    ros2 = tmp_path / 'ros2'
    ros2.mkdir()
    ros1_stage = tmp_path / 'ros1.part'
    command = MODULE.build_ros1_conversion_argv(ros2, ros1_stage)
    assert command == [
        'rosbags-convert', '--src', str(ros2.resolve()),
        '--dst', str(ros1_stage.resolve()), '--compress', 'none',
        '--src-typestore', 'ros2_humble', '--dst-typestore', 'ros1_noetic',
    ]
    comparator = MODULE.build_semantic_comparator_argv(
        ros1_stage, ros2, ['/imu/imu', '/os1_cloud_node1/points'],
        tmp_path / 'semantic.json')
    assert comparator[0] == MODULE.sys.executable
    assert comparator[1] == str(MODULE.SEMANTIC_COMPARATOR)
    assert comparator[-2:] == ['--output', str((tmp_path / 'semantic.json').resolve())]
