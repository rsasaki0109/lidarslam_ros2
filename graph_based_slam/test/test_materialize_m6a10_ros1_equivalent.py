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

"""Contract tests for the M6a10 ROS1 equivalence materializer."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
import pytest
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import get_typestore, Stores
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'materialize_m6a10_ros1_equivalent.py'
SPEC = importlib.util.spec_from_file_location('m6a10_ros1_equivalent', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


COUNTS = {
    '/imu/imu': 1,
    '/left/image_raw': 1,
    '/os1_cloud_node1/points': 1,
}


def _make_ros2_bag(tmp_path: Path) -> Path:
    typestore = get_typestore(Stores.LATEST)
    types = typestore.types
    time_type = types['builtin_interfaces/msg/Time']
    header_type = types['std_msgs/msg/Header']
    imu_type = types['sensor_msgs/msg/Imu']
    quaternion_type = types['geometry_msgs/msg/Quaternion']
    vector_type = types['geometry_msgs/msg/Vector3']
    image_type = types['sensor_msgs/msg/Image']
    cloud_type = types['sensor_msgs/msg/PointCloud2']
    field_type = types['sensor_msgs/msg/PointField']
    bag = tmp_path / 'ros2_input'

    def header(timestamp: int, frame: str):
        return header_type(time_type(0, timestamp), frame)

    covariance = np.zeros(9, dtype=np.float64)
    imu = imu_type(
        header(1, 'imu'),
        quaternion_type(0.0, 0.0, 0.0, 1.0), covariance,
        vector_type(0.0, 0.0, 0.0), covariance,
        vector_type(0.0, 0.0, 0.0), covariance)
    image = image_type(
        header(2, 'camera'), 1, 1, 'rgba8', 0, 4,
        np.asarray([1, 2, 3, 4], dtype=np.uint8))
    cloud = cloud_type(
        header(3, 'lidar'), 1, 1, [field_type('t', 0, 6, 1)],
        False, 4, 4,
        np.frombuffer(np.asarray([0], dtype=np.uint32).tobytes(), dtype=np.uint8),
        True)

    with Writer(bag, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
        imu_connection = writer.add_connection(
            '/imu/imu', 'sensor_msgs/msg/Imu', typestore=typestore)
        image_connection = writer.add_connection(
            '/left/image_raw', 'sensor_msgs/msg/Image', typestore=typestore)
        cloud_connection = writer.add_connection(
            '/os1_cloud_node1/points', 'sensor_msgs/msg/PointCloud2', typestore=typestore)
        writer.write(
            imu_connection, 1,
            typestore.serialize_cdr(imu, 'sensor_msgs/msg/Imu'))
        writer.write(
            image_connection, 2,
            typestore.serialize_cdr(image, 'sensor_msgs/msg/Image'))
        writer.write(
            cloud_connection, 3,
            typestore.serialize_cdr(cloud, 'sensor_msgs/msg/PointCloud2'))
    return bag


def _fixed_receipt(bag: Path, path: Path) -> tuple[Path, dict]:
    tree = MODULE.sha256_tree(bag)
    generator = ROOT / 'scripts' / 'materialize_m6a10_synchronized_tail.py'
    per_topic = {
        topic: {'count': 1, 'ordered_payload_stream_sha256': '0' * 64}
        for topic in MODULE.EXPECTED_TOPICS}
    document = {
        'schema_version': 1,
        'receipt_kind': 'm6a10_v2a_synchronized_tail_generation',
        'contract_id': MODULE.MATERIALIZATION_CONTRACT_ID,
        'status': 'PASS',
        'receipt_path': str(path.resolve()),
        'analyzer': {'contract_id': MODULE.ANALYZER_CONTRACT_ID},
        'generator': {
            'path': MODULE.GENERATOR_RELATIVE_PATH,
            'sha256': hashlib.sha256(generator.read_bytes()).hexdigest(),
        },
        'output': {
            'path': str(bag.resolve()),
            'tree': tree,
            'record_count': 3,
            'per_topic': per_topic,
        },
        'verification': {
            'status': 'PASS',
            'connection_metadata_equal': True,
            'output_record_count': 3,
            'dropped_count': 0,
        },
        'order_definition': MODULE.ORDER_DEFINITION,
        'safety': {
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'input_modified': False,
            'receipt_external_to_output': True,
        },
    }
    path.write_text(json.dumps(document, sort_keys=True) + '\n', encoding='utf-8')
    return path, document


def _profile(tmp_path: Path, bag: Path, fixed_receipt: Path, final: Path) -> Path:
    converter = MODULE.CONVERTER_PATH
    comparator = MODULE.COMPARATOR_PATH
    generator = MODULE.GENERATOR_PATH
    profile = {
        'competitive_slam_profile': {
            'runtime_policy': {
                'phase_contract_v2': {
                    'synchronized_tail_materialization': {
                        'contract_id': MODULE.MATERIALIZATION_CONTRACT_ID,
                        'status': 'ros2_materialized_verified_ros1_pending',
                        'producer': {
                            'path': MODULE.GENERATOR_RELATIVE_PATH,
                            'sha256': hashlib.sha256(generator.read_bytes()).hexdigest(),
                        },
                        'generated_output': {
                            'path': str(bag.resolve()),
                            'tree_sha256': MODULE.sha256_tree(bag)['sha256'],
                            'output_record_count': 3,
                        },
                        'generated_receipt': {
                            'path': str(fixed_receipt.resolve()),
                            'sha256': hashlib.sha256(fixed_receipt.read_bytes()).hexdigest(),
                        },
                        'ros1_fast_livo2': {
                            'conversion_executed_here': False,
                            'status': 'ros1_pending',
                            'converter': MODULE.CONVERTER,
                            'converter_version': MODULE.CONVERTER_VERSION,
                            'converter_executable_path': str(converter),
                            'converter_executable_sha256': hashlib.sha256(
                                converter.read_bytes()).hexdigest(),
                            'converter_help_sha256': MODULE._converter_help_sha256(converter),
                            'ros1_output_path': str(final.resolve()),
                            'ros1_staging_container_path': str(
                                final.with_suffix('.staging').resolve()),
                            'ros1_staging_output_path': str(
                                (final.with_suffix('.staging') / final.name).resolve()),
                            'destination_extension': '.bag',
                            'staging_policy': 'sibling_container_exact_final_basename',
                            'conversion_argv': list(MODULE.PROFILE_CONVERSION_ARGV),
                            'semantic_comparator': {
                                'path': MODULE.COMPARATOR_RELATIVE_PATH,
                                'sha256': hashlib.sha256(comparator.read_bytes()).hexdigest(),
                                'topics': list(MODULE.EXPECTED_TOPICS),
                                'all_topics_equal_required': True,
                                'report_required_before_fast_run': True,
                                'status': 'pending_not_run',
                            },
                        },
                    },
                },
            },
        },
    }
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile, sort_keys=False), encoding='utf-8')
    return path


def _fixture(tmp_path: Path):
    tmp_path.mkdir(parents=True, exist_ok=True)
    bag = _make_ros2_bag(tmp_path)
    fixed_receipt, _ = _fixed_receipt(bag, tmp_path / 'fixed10.json')
    final = tmp_path / 'ros1_equivalent.bag'
    profile = _profile(tmp_path, bag, fixed_receipt, final)
    return bag, fixed_receipt, final, profile


def test_real_rosbag2_to_ros1_conversion_and_three_topic_semantic_pass(tmp_path):
    bag, fixed_receipt, final, profile = _fixture(tmp_path)
    receipt_path = tmp_path / 'ros1-receipt.json'
    result = MODULE.materialize_ros1_equivalent(
        bag, fixed_receipt, final, receipt_path,
        profile_path=profile, expected_topic_counts=COUNTS)
    assert result['receipt']['status'] == 'PASS'
    assert final.is_file()
    assert result['receipt']['ros1_output']['sha256'] == hashlib.sha256(
        final.read_bytes()).hexdigest()
    assert result['receipt']['semantic_equivalence']['all_topics_equal'] is True
    assert result['receipt']['semantic_equivalence']['topics']
    assert result['receipt']['ros1_output']['topic_counts'] == COUNTS
    assert result['receipt']['ros1_output']['inspection']['status'] == 'PASS'
    assert result['receipt']['ros1_output']['inspection']['total_message_count'] == 3
    assert result['receipt']['conversion']['stdout']['sha256']
    assert result['receipt']['conversion']['stderr']['sha256']
    assert result['receipt']['semantic_equivalence']['stdout']['sha256']
    assert result['receipt']['semantic_equivalence']['stderr']['sha256']
    assert not final.with_suffix('.staging').exists()
    assert not final.with_name(final.name + '.part').exists()
    persisted = json.loads(receipt_path.read_text(encoding='utf-8'))
    assert 'receipt_sha256' not in persisted
    assert result['receipt_sha256'] == hashlib.sha256(receipt_path.read_bytes()).hexdigest()


def test_converter_argv_is_list_pinned_and_not_shell_interpreted(tmp_path):
    converter = tmp_path / 'converter;touch-host'
    stage = tmp_path / 'ros1;bad.bag'
    argv = MODULE.build_conversion_argv(converter, tmp_path / 'ros2 input', stage)
    assert argv[0] == str(converter)
    assert argv[4] == str(stage)
    assert all(isinstance(value, str) for value in argv)
    assert ';' in argv[0] and ';' in argv[4]


def test_missing_tool_hash_and_fixed_receipt_contract_fail_closed(tmp_path):
    bag, fixed_receipt, final, profile = _fixture(tmp_path)
    document = yaml.safe_load(profile.read_text(encoding='utf-8'))
    contract = document['competitive_slam_profile']['runtime_policy'][
        'phase_contract_v2']['synchronized_tail_materialization']
    contract['ros1_fast_livo2']['converter_executable_sha256'] = '0' * 64
    profile.write_text(yaml.safe_dump(document, sort_keys=False), encoding='utf-8')
    with pytest.raises(MODULE.Ros1EquivalenceError, match='converter executable hash'):
        MODULE.materialize_ros1_equivalent(
            bag, fixed_receipt, final, tmp_path / 'receipt.json',
            profile_path=profile, expected_topic_counts=COUNTS)

    bag2, fixed_receipt2, final, profile = _fixture(tmp_path / 'second')
    fixed_document = json.loads(fixed_receipt2.read_text(encoding='utf-8'))
    fixed_document['contract_id'] = 'wrong'
    fixed_receipt2.write_text(json.dumps(fixed_document) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.Ros1EquivalenceError, match='fixed receipt hash'):
        MODULE.materialize_ros1_equivalent(
            bag2, fixed_receipt2, final, tmp_path / 'second-receipt.json',
            profile_path=profile, expected_topic_counts=COUNTS)


def test_semantic_failure_keeps_staging_and_never_publishes(tmp_path):
    bag, fixed_receipt, final, profile = _fixture(tmp_path)
    receipt_path = tmp_path / 'receipt.json'
    calls = []

    def runner(argv):
        calls.append(list(argv))
        if argv[0] == str(MODULE.CONVERTER_PATH):
            Path(argv[argv.index('--dst') + 1]).write_bytes(b'not-a-real-bag')
            return subprocess.CompletedProcess(argv, 0, '', '')
        report = Path(argv[argv.index('--output') + 1])
        report.write_text(json.dumps({
            'schema_version': 1,
            'all_topics_equal': False,
            'topics': [],
        }) + '\n', encoding='utf-8')
        return subprocess.CompletedProcess(argv, 1, '', 'semantic mismatch')

    with pytest.raises(MODULE.Ros1EquivalenceError, match='semantic comparator exited'):
        MODULE.materialize_ros1_equivalent(
            bag, fixed_receipt, final, receipt_path,
            profile_path=profile, runner=runner, expected_topic_counts=COUNTS)
    assert len(calls) == 2
    assert not final.exists()
    assert final.with_suffix('.staging').is_dir()
    assert not receipt_path.exists()


def test_converter_failure_retains_staging_and_rerun_or_stale_stage_is_rejected(tmp_path):
    bag, fixed_receipt, final, profile = _fixture(tmp_path)

    def failed_runner(argv):
        if argv[0] == str(MODULE.CONVERTER_PATH):
            Path(argv[argv.index('--dst') + 1]).write_bytes(b'partial')
        return subprocess.CompletedProcess(argv, 17, '', 'failed')

    with pytest.raises(MODULE.Ros1EquivalenceError, match='rosbags-convert exited'):
        MODULE.materialize_ros1_equivalent(
            bag, fixed_receipt, final, tmp_path / 'receipt.json',
            profile_path=profile, runner=failed_runner, expected_topic_counts=COUNTS)
    assert final.with_suffix('.staging').is_dir()
    with pytest.raises(MODULE.Ros1EquivalenceError, match='staging container exists'):
        MODULE.materialize_ros1_equivalent(
            bag, fixed_receipt, final, tmp_path / 'second-receipt.json',
            profile_path=profile, runner=failed_runner, expected_topic_counts=COUNTS)


def test_invalid_basename_and_semantic_missing_topic_fail_closed(tmp_path):
    bag, fixed_receipt, final, profile = _fixture(tmp_path)
    bad_final = tmp_path / 'ros1_equivalent.db3'
    with pytest.raises(MODULE.Ros1EquivalenceError, match=r'\.bag'):
        MODULE.materialize_ros1_equivalent(
            bag, fixed_receipt, bad_final, tmp_path / 'bad-receipt.json',
            profile_path=profile, expected_topic_counts=COUNTS)
    report = tmp_path / 'semantic.json'
    report.write_text(json.dumps({
        'schema_version': 1,
        'all_topics_equal': True,
        'topics': [],
    }) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.Ros1EquivalenceError, match='semantic topic set'):
        MODULE._validate_semantic_report(report, COUNTS)


def test_overlap_and_symlink_rejected_before_output_parent_creation(tmp_path):
    bag, _, _, _ = _fixture(tmp_path)
    nested = bag / 'untrusted-output' / 'result.bag'
    with pytest.raises(MODULE.Ros1EquivalenceError, match='overlaps input'):
        MODULE._assert_paths(
            bag, nested, tmp_path / 'receipt.json', tmp_path / 'semantic.json')
    assert not nested.parent.exists()

    real_parent = tmp_path / 'real-parent'
    real_parent.mkdir()
    symlink_parent = tmp_path / 'symlink-parent'
    symlink_parent.symlink_to(real_parent, target_is_directory=True)
    with pytest.raises(MODULE.Ros1EquivalenceError, match='symlink ancestor'):
        MODULE._assert_paths(
            bag,
            symlink_parent / 'result.bag',
            tmp_path / 'receipt2.json',
            tmp_path / 'semantic2.json')


def test_input_and_staging_identity_are_bound_to_profile(tmp_path):
    bag, fixed_receipt, final, profile = _fixture(tmp_path)
    wrong_input = tmp_path / 'wrong-input'
    wrong_input.mkdir()
    with pytest.raises(MODULE.Ros1EquivalenceError, match='input ROS2 path'):
        MODULE.validate_profile_and_tools(
            profile, fixed_receipt, final, wrong_input, expected_topic_counts=COUNTS)

    document = yaml.safe_load(profile.read_text(encoding='utf-8'))
    contract = document['competitive_slam_profile']['runtime_policy'][
        'phase_contract_v2']['synchronized_tail_materialization']
    contract['ros1_fast_livo2']['ros1_staging_container_path'] = str(
        final.with_name(final.name + '.staging'))
    profile.write_text(yaml.safe_dump(document, sort_keys=False), encoding='utf-8')
    with pytest.raises(MODULE.Ros1EquivalenceError, match='staging paths'):
        MODULE.validate_profile_and_tools(
            profile, fixed_receipt, final, bag, expected_topic_counts=COUNTS)


def test_production_topic_order_and_semantic_aggregate_tamper_fail_closed(tmp_path):
    bag, fixed_receipt, final, profile = _fixture(tmp_path)
    assert list(MODULE.EXPECTED_TOPICS) == [
        '/os1_cloud_node1/points', '/imu/imu', '/left/image_raw']
    document = yaml.safe_load(profile.read_text(encoding='utf-8'))
    semantic = document['competitive_slam_profile']['runtime_policy'][
        'phase_contract_v2']['synchronized_tail_materialization'][
            'ros1_fast_livo2']['semantic_comparator']
    assert semantic['topics'] == list(MODULE.EXPECTED_TOPICS)

    rows = [{
        'topic': topic,
        'equal': True,
        'message_count_left': 1,
        'message_count_right': 1,
        'aggregate_sha256_left': '1' * 64,
        'aggregate_sha256_right': '2' * 64,
    } for topic in MODULE.EXPECTED_TOPICS]
    report = tmp_path / 'aggregate-tamper.json'
    report.write_text(json.dumps({
        'schema_version': 1, 'all_topics_equal': True, 'topics': rows,
    }) + '\n', encoding='utf-8')
    with pytest.raises(MODULE.Ros1EquivalenceError, match='aggregate digests differ'):
        MODULE._validate_semantic_report(report, COUNTS)


def test_extra_ros1_connection_is_rejected_before_publish(monkeypatch, tmp_path):
    from rosbags import highlevel

    connections = [SimpleNamespace(
        id=index,
        topic=topic,
        msgtype=MODULE.EXPECTED_TOPIC_TYPES[topic],
        msgdef=SimpleNamespace(data='definition'),
        serialization_format=None,
        offered_qos_profiles=None,
        rihs01=None,
    ) for index, topic in enumerate(MODULE.EXPECTED_TOPICS)]
    connections.append(SimpleNamespace(
        id=99,
        topic='/unexpected',
        msgtype='sensor_msgs/msg/Image',
        msgdef=SimpleNamespace(data='definition'),
        serialization_format=None,
        offered_qos_profiles=None,
        rihs01=None,
    ))

    class FakeReader:
        def __init__(self, _paths):
            self.connections = connections

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def messages(self):
            return iter(())

    monkeypatch.setattr(highlevel, 'AnyReader', FakeReader)
    with pytest.raises(MODULE.Ros1EquivalenceError, match='unexpected connection count'):
        MODULE._inspect_ros1_bag(tmp_path / 'fake.bag', COUNTS)
