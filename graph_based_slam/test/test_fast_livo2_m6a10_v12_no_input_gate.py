# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
Unit gates for the v12 host no-input lifecycle runner.

All external Docker/ROS operations are replaced with in-process fakes.  The
tests therefore exercise argv construction, protocol ordering, fail-closed
cleanup, and immutable fresh-root handling without starting a container.
"""

from __future__ import annotations

import ast
import copy
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any

import pytest

ROOT = Path(__file__).resolve().parents[2]
RUNNER_PATH = ROOT / 'scripts/run_fast_livo2_m6a10_v12_no_input_gate.py'
SPEC = importlib.util.spec_from_file_location('m6a10_v12_no_input_gate', RUNNER_PATH)
assert SPEC is not None and SPEC.loader is not None
RUNNER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = RUNNER
SPEC.loader.exec_module(RUNNER)


def _callback() -> dict[str, Any]:
    return {
        'schema_version': 3,
        'contract_version': RUNNER.PHASE_CONTRACT,
        'transport_contract_version': RUNNER.TRANSPORT_CONTRACT,
        'phase_mode': RUNNER.PHASE_MODE,
        'system': 'fast_livo2',
        'status': 'pass',
        'benchmark_only': True,
        'consumer': {
            'ack_source_kind': 'consumer_callback',
            'ack_source': 'LIVMapper subscriber callback return',
            'ack_semantics': 'callback_acceptance_not_backend_completion',
            'publisher_count_used': False,
            'eof_observed': True,
            'eof_source': '/m6a10/consumer_eof service',
            'expected_messages': 1,
            'expected_topic_counts': dict(RUNNER.EXPECTED_COUNTS),
            'received_topic_counts': dict(RUNNER.EXPECTED_COUNTS),
            'received_messages': 1,
            'processed_messages': 1,
            'acked_messages': 1,
            'dropped_messages': 0,
            'queue_overflow': 0,
            'queue_overflow_observable': True,
            'processing_failures': 0,
            'backlog_at_drain': 1,
            'maximum_backlog_messages': 1,
            'maximum_allowed_backlog_messages': 1,
            'maximum_callback_latency_seconds': 0.01,
            'maximum_allowed_callback_latency_seconds': 0.25,
            'first_processed_timestamp_seconds': 10.0,
            'last_processed_timestamp_seconds': 10.0,
            'required_end_timestamp_seconds': 10.0,
            'paced_input_rate': 1.0,
            'paced_input_rate_verified': False,
            'ack_backpressure_verified': True,
            'acknowledgement_contract': 'one_publish_waits_for_callback_then_one_ack_service_call',
            'queue_capacity_messages': 1,
            'queue_drop_detection': 'exact_counts_plus_single_inflight_ack',
            'single_message_buffer_verified': False,
            'counter_evidence_path': RUNNER.CALLBACK_RAW,
            'drain_complete': True,
            'ack_exact': True,
            'transport_outstanding_at_drain': 0,
            'maximum_transport_outstanding_messages': 1,
            'maximum_allowed_transport_outstanding_messages': 1,
            'mapper_internal_deque_current_messages': 81,
            'mapper_internal_deque_peak_messages': 81,
        },
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


def _terminal() -> dict[str, Any]:
    zero = {'lidar': 0, 'imu': 0, 'image': 0}
    return {
        'schema_version': 1,
        'contract_id': RUNNER.TERMINAL_CONTRACT,
        'phase_mode': RUNNER.PHASE_MODE,
        'system': 'fast_livo2',
        'status': 'invalid',
        'received_topic_counts': dict(RUNNER.EXPECTED_COUNTS),
        'completed_counts': dict(zero),
        'dropped_counts': dict(zero),
        'overflow_counts': dict(zero),
        'processing_failures': 0,
        'backend': {
            'quiescent': False,
            'quiescence_observed': False,
            'completed_boundary': {
                'observed': False,
                'timestamp_seconds': None,
                'sequence': None,
                'source': '',
            },
            'completed_counts': {'lidar': None, 'imu': None, 'image': None},
            'completed_synchronization_units': None,
            'dropped_counts': dict(zero),
            'overflow_counts': dict(zero),
            'processing_failures': 0,
            'in_flight': {'active': True},
        },
        'buffers': {
            'lidar': {
                'count': 1,
                'oldest_timestamp_seconds': 10.0,
                'newest_timestamp_seconds': 10.0,
                'records': [
                    {
                        'record_id': 'lidar-0',
                        'topic': 'lidar',
                        'timestamp_seconds': 10.0,
                        'support_context_proven': False,
                        'post_boundary': False,
                        'equal_to_completed_boundary': False,
                        'sync_predicate_evaluated': True,
                        'can_form_synchronization_unit': False,
                        'reason_code': 'residual_lidar_rejected',
                    }
                ],
            },
            'imu': {
                'count': 0,
                'oldest_timestamp_seconds': None,
                'newest_timestamp_seconds': None,
                'records': [],
            },
            'image': {
                'count': 0,
                'oldest_timestamp_seconds': None,
                'newest_timestamp_seconds': None,
                'records': [],
            },
        },
        'terminal_support_context': {
            'classification': 'nonlidar_at_or_after_boundary_support_context',
            'total_count': 0,
            'by_topic': dict(zero),
            'records': [],
        },
        'terminal_observation': {
            'eof_observed': True,
            'stable': True,
            'poll_count': 2,
            'stable_poll_count': 2,
            'minimum_poll_wall_seconds': 0.05,
            'first_poll_wall_seconds': 10.0,
            'stable_poll_wall_seconds': 10.05,
            'identical_snapshot': True,
            'sync_predicate_evaluated': True,
        },
        'trajectory': {
            'coverage_verified': True,
            'last_timestamp_seconds': None,
            'end_gap_seconds': None,
        },
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }


class _FakeProcess:
    def __init__(self, events: list[tuple[str, Any]]) -> None:
        self.events = events
        self.stdin = object()

    def wait(self, timeout: float | None = None) -> int:
        self.events.append(('payload_wait', timeout))
        return 0


class _FakeLifecycle:
    def __init__(
        self,
        callback: dict[str, Any],
        terminal: dict[str, Any],
        *,
        ack_ok: bool = True,
        artifact_failure: str | None = None,
    ) -> None:
        self.callback = callback
        self.terminal = terminal
        self.ack_ok = ack_ok
        self.artifact_failure = artifact_failure
        self.events: list[tuple[str, Any]] = []
        self.ack_count = 0
        self.service_calls: list[str] = []
        self.read_calls: list[str] = []

    def image_probe(self, image_id: str) -> dict[str, Any]:
        self.events.append(('image_probe', image_id))
        return {
            'id': image_id,
            'tag': RUNNER.IMAGE_TAG,
            'labels': {},
            'entrypoint': [RUNNER.ROS_ENTRYPOINT],
        }

    def run_container(self, argv: list[str], log_path: Path) -> str:
        self.events.append(('run', list(argv)))
        log_path.write_text('container-id\n', encoding='utf-8')
        return 'container-id'

    def exec_payload(self, name: str, payload: bytes, log_path: Path) -> _FakeProcess:
        self.events.append(('exec', (name, payload)))
        log_path.write_bytes(payload)
        return _FakeProcess(self.events)

    def wait_ready(self, name: str, process: _FakeProcess) -> dict[str, Any]:
        self.events.append(('ready', name))
        return {'ready': True, 'services': list(RUNNER.EXPECTED_SERVICES)}

    def publish(self, name: str) -> dict[str, Any]:
        self.events.append(('publish', name))
        return {'published_lidar_callbacks': 1}

    def service(self, name: str, service: str) -> dict[str, Any]:
        self.events.append(('service', service))
        self.service_calls.append(service)
        if service == '/m6a10/consumer_status':
            return {
                'success': True,
                'received_messages': 1,
                'received_topic_counts': dict(RUNNER.EXPECTED_COUNTS),
            }
        if service == '/m6a10/consumer_ack':
            self.ack_count += 1
            return {'success': self.ack_ok and self.ack_count == 1}
        return {'success': True}

    def manifest(self, name: str) -> dict[str, Any]:
        self.events.append(('manifest', name))
        return {
            'argv': RUNNER.build_manifest_argv(name),
            'returncode': 0,
            'paths': list(RUNNER.PINNED_RAW_PATHS + RUNNER.PINNED_DIAGNOSTIC_PATHS),
            'stdout': '\n'.join(RUNNER.PINNED_RAW_PATHS + RUNNER.PINNED_DIAGNOSTIC_PATHS) + '\n',
            'stderr': '',
        }

    def read_file(self, name: str, source: str) -> bytes:
        self.events.append(('cat', source))
        self.read_calls.append(source)
        if source == self.artifact_failure:
            raise RUNNER.GateError('ARTIFACT_READ_FAILURE', source)
        if source == RUNNER.CALLBACK_RAW:
            return json.dumps(self.callback).encode('utf-8')
        elif source == RUNNER.TERMINAL_RAW:
            return json.dumps(self.terminal).encode('utf-8')
        return (source + '\n').encode('utf-8')

    def inspect(self, name: str) -> dict[str, Any]:
        self.events.append(('inspect', name))
        return {'status': 'running', 'exit_code': None, 'oom_killed': False}

    def stats(self, name: str) -> dict[str, Any]:
        self.events.append(('stats', name))
        return {'rss': 0}

    def top(self, name: str) -> dict[str, Any]:
        self.events.append(('top', name))
        return {'processes': []}

    def stop(self, name: str) -> dict[str, Any]:
        self.events.append(('stop', name))
        return {'returncode': 0}

    def wait_container(self, name: str, timeout: float) -> dict[str, Any]:
        self.events.append(('wait_container', timeout))
        return {'status': 'exited', 'exit_code': 0, 'oom_killed': False}

    def remove(self, name: str) -> dict[str, Any]:
        self.events.append(('remove', name))
        return {'returncode': 0}

    def sleep(self, seconds: float) -> None:
        self.events.append(('sleep', seconds))

    def hooks(self) -> Any:
        return RUNNER.GateHooks(
            image_probe=self.image_probe,
            run_container=self.run_container,
            exec_payload=self.exec_payload,
            wait_ready=self.wait_ready,
            publish=self.publish,
            service=self.service,
            manifest=self.manifest,
            read_file=self.read_file,
            inspect=self.inspect,
            stats=self.stats,
            top=self.top,
            stop=self.stop,
            wait_container=self.wait_container,
            remove=self.remove,
            sleep=self.sleep,
        )


def _patch_preflight(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(RUNNER, 'verify_build_receipt', lambda: {'status': 'PASS'})
    monkeypatch.setattr(
        RUNNER,
        'verify_prior_attempts',
        lambda: [
            {
                'attempt_index': 1,
                'path': 'prior1',
                'sha256': 'b5e705...',
                'cause': 'missing_docker_interactive_stdin',
            },
            {
                'attempt_index': 2,
                'path': 'prior2',
                'sha256': 'c85f425...',
                'cause': 'mapper_pid_assignment_typo',
            },
            {
                'attempt_index': 3,
                'path': 'prior3',
                'sha256': '7579b04...',
                'cause': 'synthetic_publisher_syntax_error',
            },
            {
                'attempt_index': 4,
                'path': 'prior4',
                'sha256': '6f8ca7d...',
                'cause': 'ros_entrypoint_not_used_for_ros_python',
            },
        ],
    )
    monkeypatch.setattr(
        RUNNER, 'verify_sources', lambda repo_root: {'payload': RUNNER.PAYLOAD_SHA256}
    )


def test_run_argv_is_digest_bound_isolated_and_has_zero_host_mounts():
    argv = RUNNER.build_run_argv('m6a10-v12-no-input-test')
    assert argv[:4] == ['docker', 'run', '-d', '-i']
    assert argv.count('-d') == 1
    assert argv.count('-i') == 1
    assert argv.count('--tmpfs') == 3
    assert argv.count('--mount') == 0
    assert '--network' in argv and argv[argv.index('--network') + 1] == 'none'
    assert '--read-only' in argv
    assert '--rm' not in argv
    assert RUNNER.IMAGE_ID in argv and RUNNER.IMAGE_TAG not in argv
    rendered = ' '.join(argv).lower()
    for forbidden in ('/input/', 'rosbag', 'ground_truth', 'scorer', 'map_save'):
        assert forbidden not in rendered


def test_exec_uses_interactive_bash_stdin_and_payload_is_the_only_startup_source():
    assert RUNNER.build_exec_argv('m6a10-v12-no-input-test') == [
        'docker',
        'exec',
        '-i',
        'm6a10-v12-no-input-test',
        'bash',
        '-s',
    ]
    assert RUNNER.PAYLOAD_PATH.name == 'fast_livo2_m6a10_v12_no_input_container_payload.sh'
    assert (
        RUNNER.PAYLOAD_SHA256 == '2c1c9b86fadc0d1d83e7874542f92ff33b99b23421397374ecd9e880ad1b73aa'
    )
    assert 'rospy.Time.from_sec(10.0)' in RUNNER.SYNTHETIC_PUBLISHER_SOURCE
    assert RUNNER.build_publish_argv('m6a10-v12-no-input-test')[:5] == [
        'docker',
        'exec',
        'm6a10-v12-no-input-test',
        '/ros_entrypoint.sh',
        'python3',
    ]
    assert RUNNER.build_service_argv('m6a10-v12-no-input-test', '/m6a10/consumer_status') == [
        'docker',
        'exec',
        'm6a10-v12-no-input-test',
        '/ros_entrypoint.sh',
        'rosservice',
        'call',
        '/m6a10/consumer_status',
        '{}',
    ]
    assert RUNNER.build_publish_argv('m6a10-v12-no-input-test')[3] == RUNNER.ROS_ENTRYPOINT
    assert (
        RUNNER.build_service_argv('m6a10-v12-no-input-test', '/m6a10/consumer_status')[3]
        == RUNNER.ROS_ENTRYPOINT
    )
    assert RUNNER.ROS_ENTRYPOINT == '/ros_entrypoint.sh'
    text = RUNNER_PATH.read_text(encoding='utf-8')
    assert 'roscore &' not in text
    assert 'roslaunch ' not in text
    assert '"bash", "-s"' in text


def test_artifact_manifest_and_cat_argv_are_fixed_and_no_docker_cp_exists():
    name = 'm6a10-v12-no-input-test'
    assert RUNNER.build_manifest_argv(name) == [
        'docker',
        'exec',
        name,
        'find',
        '/out',
        '-maxdepth',
        '2',
        '-type',
        'f',
        '-printf',
        '%p\\n',
    ]
    assert RUNNER.build_artifact_cat_argv(name, RUNNER.CALLBACK_RAW) == [
        'docker',
        'exec',
        name,
        'cat',
        RUNNER.CALLBACK_RAW,
    ]
    with pytest.raises(RUNNER.GateError):
        RUNNER.build_artifact_cat_argv(name, '/out/../etc/passwd')
    with pytest.raises(RUNNER.GateError):
        RUNNER.build_artifact_cat_argv(name, '/out/unpinned.json')
    source = RUNNER_PATH.read_text(encoding='utf-8')
    assert '"docker", "cp"' not in source
    assert 'docker cp' not in source


def test_synthetic_publisher_is_valid_multiline_exact_once_and_stamp10():
    source = RUNNER.SYNTHETIC_PUBLISHER_SOURCE
    compile(source, '<synthetic-publisher>', 'exec')
    tree = ast.parse(source, filename='<synthetic-publisher>')
    while_nodes = [node for node in ast.walk(tree) if isinstance(node, ast.While)]
    assert len(while_nodes) == 1
    while_line = source.splitlines()[while_nodes[0].lineno - 1]
    assert while_line.rstrip().endswith(':')
    assert any(
        isinstance(node, ast.Assert)
        and any(
            isinstance(child, ast.Call)
            and isinstance(child.func, ast.Attribute)
            and child.func.attr == 'get_num_connections'
            for child in ast.walk(node.test)
        )
        for node in ast.walk(tree)
    )
    publish_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'publish'
    ]
    assert len(publish_calls) == 1
    stamp_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'from_sec'
        and len(node.args) == 1
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == 10.0
    ]
    assert len(stamp_calls) == 1
    assert not any(
        line.lstrip().startswith('while ') and ':' in line and line.split(':', 1)[1].strip()
        for line in source.splitlines()
    )


def test_synthetic_publisher_matches_v9_ouster_layout_with_only_declared_deltas():
    source = RUNNER.SYNTHETIC_PUBLISHER_SOURCE
    tree = ast.parse(source, filename='<synthetic-publisher-layout>')

    def value(node: ast.AST) -> object:
        if isinstance(node, ast.Constant):
            return node.value
        if isinstance(node, ast.Attribute):
            return node.attr
        if isinstance(node, ast.Name):
            return node.id
        raise AssertionError(f'unexpected AST value: {ast.dump(node)}')

    field_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'PointField'
    ]
    assert [tuple(value(argument) for argument in call.args) for call in field_calls] == [
        ('x', 0, 'FLOAT32', 1),
        ('y', 4, 'FLOAT32', 1),
        ('z', 8, 'FLOAT32', 1),
        ('intensity', 16, 'FLOAT32', 1),
        ('t', 20, 'UINT32', 1),
        ('reflectivity', 24, 'UINT16', 1),
        ('ring', 26, 'UINT8', 1),
        ('ambient', 28, 'UINT16', 1),
        ('range', 32, 'UINT32', 1),
    ]
    payload_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'bytearray'
    ]
    assert len(payload_calls) == 1 and value(payload_calls[0].args[0]) == 36
    pack_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'pack_into'
    ]
    assert [(value(call.args[0]), value(call.args[2])) for call in pack_calls] == [
        ('<f', 0),
        ('<f', 4),
        ('<f', 8),
        ('<f', 16),
        ('<I', 20),
        ('<H', 24),
        ('<B', 26),
        ('<H', 28),
        ('<I', 32),
    ]
    pointcloud_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'PointCloud2'
    ]
    assert len(pointcloud_calls) == 1
    pointcloud = {keyword.arg: keyword.value for keyword in pointcloud_calls[0].keywords}
    assert value(pointcloud['height']) == 1
    assert value(pointcloud['width']) == 1
    assert value(pointcloud['is_bigendian']) is False
    assert value(pointcloud['point_step']) == 36
    assert value(pointcloud['row_step']) == 36
    assert value(pointcloud['fields']) == 'fields'
    assert isinstance(pointcloud['data'], ast.Call)
    assert (
        value(pointcloud['data'].func) == 'bytes'
        and value(pointcloud['data'].args[0]) == 'payload'
    )
    assert value(pointcloud['is_dense']) is True
    header = pointcloud['header']
    assert isinstance(header, ast.Call) and value(header.func) == 'Header'
    header_values = {keyword.arg: keyword.value for keyword in header.keywords}
    assert value(header_values['frame_id']) == 'os_sensor'
    assert value(header_values['stamp'].func) == 'from_sec'
    assert value(header_values['stamp'].args[0]) == 10.0
    publisher_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == 'Publisher'
    ]
    assert len(publisher_calls) == 1
    assert value(publisher_calls[0].args[0]) == '/os1_cloud_node1/points'
    publisher_kwargs = {keyword.arg: keyword.value for keyword in publisher_calls[0].keywords}
    assert value(publisher_kwargs['queue_size']) == 1 and value(publisher_kwargs['latch']) is False

    v9_source = (ROOT / 'scripts/preflight_fast_livo2_m6a10_v2c_v9_handshake.py').read_text(
        encoding='utf-8'
    )
    for token in (
        "PointField('x', 0, PointField.FLOAT32, 1)",
        "PointField('reflectivity', 24, PointField.UINT16, 1)",
        'payload = bytearray(36)',
        'point_step=36, row_step=36',
        "frame_id='os_sensor'",
    ):
        assert token in v9_source and token in source
    assert 'rospy.Time.from_sec(1.0)' in v9_source
    assert 'rospy.Time.from_sec(10.0)' in source
    assert 'm6a10_v9_synthetic_lidar_publisher' in v9_source
    assert 'm6a10_no_input_lidar_once' in source
    assert 'm6a10_v9_publish_marker' in v9_source
    assert 'm6a10_v9_publish_marker' not in source


def test_host_runner_is_executable_and_prior_causes_have_sealed_evidence():
    assert RUNNER_PATH.stat().st_mode & 0o111 == 0o111
    checked = RUNNER.verify_prior_attempts()
    assert [item['cause'] for item in checked] == [
        'missing_docker_interactive_stdin',
        'mapper_pid_assignment_typo',
        'synthetic_publisher_syntax_error',
        'ros_entrypoint_not_used_for_ros_python',
        'artifact_copy_callback_raw_failure',
    ]
    first = checked[0]['verified_cause_evidence']
    assert first['kind'] == 'docker_run_argv_missing_interactive_stdin'
    assert (
        first['argv_sha256'] == '79c09830e4d896e25996b1cb5752e66d9197846690ba2e10fec18da38f17d694'
    )
    assert (
        first['argv_canonical_sha256']
        == '03af075918f4f6ca94e7f6cea41799dac7a8c1e46e39f25ae56feed90b21fc8b'
    )
    second = checked[1]['verified_cause_evidence']
    assert second['kind'] == 'mapper_pid_assignment_typo'
    assert second['log_exact_fragment'] == 'MAPPER_PID: command not found'
    assert (
        second['log_sha256'] == '139228f498382d8b37e8438638ccaa1fd114b86d76de471a4b62f5df7645d71c'
    )
    third = checked[2]['verified_cause_evidence']
    assert third['kind'] == 'synthetic_publisher_syntax_error_before_callback'
    assert (
        third['receipt_sidecar_sha256']
        == '6f192d3872cf6aa733bdf79a38f470ac2399f1159dd49532f937e0980e85b659'
    )
    assert third['services_reached'] == 7
    assert third['start_count'] == 1
    assert third['published_lidar_callbacks'] == 0
    assert third['publisher_failed_before_callback'] is True
    assert third['container_exit_code'] == 143
    assert third['safety_all_false'] is True
    assert third['cleanup_stopped_only'] is True
    assert third['cleanup_remove_returncode'] == 0
    assert third['cleanup_oom_killed'] is False
    fourth = checked[3]['verified_cause_evidence']
    assert fourth['kind'] == 'ros_entrypoint_not_used_for_rospy_import'
    assert (
        fourth['receipt_sidecar_sha256']
        == '2ebad3e9c01399f093352c9e7fdcbbe5ef187a68bab218f0df6aa8e0e824c215'
    )
    assert fourth['receipt_attempt_index_observed'] == 3
    assert fourth['services_reached'] == 7
    assert fourth['start_count'] == 1
    assert fourth['published_lidar_callbacks'] == 0
    assert fourth['publisher_failed_before_callback'] is True
    assert fourth['container_exit_code'] == 143
    assert fourth['safety_all_false'] is True
    assert fourth['cleanup_stopped_only'] is True
    assert fourth['cleanup_remove_returncode'] == 0
    assert fourth['cleanup_oom_killed'] is False
    fifth = checked[4]['verified_cause_evidence']
    assert fifth['kind'] == 'artifact_copy_callback_raw_failure'
    assert (
        fifth['receipt_sidecar_sha256']
        == 'dba4a5e3e913e9f7dab99b575e3e8e80626c3ba99846482a8f6c16ab86595d1c'
    )
    assert fifth['artifact_path'] == RUNNER.CALLBACK_RAW
    assert fifth['services_reached'] == 7
    assert fifth['start_count'] == 1
    assert fifth['published_lidar_callbacks'] == 1
    assert fifth['container_exit_code'] == 143
    assert fifth['safety_all_false'] is True
    assert fifth['cleanup_stopped_only'] is True
    assert fifth['cleanup_remove_returncode'] == 0
    assert fifth['cleanup_oom_killed'] is False


def test_prior_attempts_must_be_contiguous_before_next_attempt():
    with pytest.raises(RUNNER.GateError) as error:
        RUNNER.verify_prior_attempts([RUNNER.PRIOR_ATTEMPTS[0], RUNNER.PRIOR_ATTEMPTS[2]])
    assert error.value.kind == 'PRIOR_ATTEMPT_SEQUENCE_INVALID'


def test_sources_record_runner_and_test_hashes_as_observed_not_pins():
    sources = RUNNER.verify_sources(ROOT)
    runner = sources['runner_runtime_observed']
    focused = sources['focused_test_runtime_observed']
    assert runner['binding'] == 'observed_runtime_self_hash_not_source_pin'
    assert focused['binding'] == 'observed_test_evidence_not_source_pin'
    assert len(runner['sha256']) == 64 and len(focused['sha256']) == 64
    assert Path(runner['path']).resolve() == RUNNER_PATH.resolve()
    assert Path(focused['path']).resolve() == (ROOT / RUNNER.FOCUSED_TEST_PATH).resolve()


def test_prior_cause_evidence_rejects_symlinked_artifact(tmp_path: Path):
    item = copy.deepcopy(RUNNER.PRIOR_ATTEMPTS[0])
    linked = tmp_path / 'docker_argv.json'
    linked.symlink_to(item['cause_evidence']['argv_path'])
    item['cause_evidence']['argv_path'] = linked
    with pytest.raises(RUNNER.GateError) as error:
        RUNNER.verify_prior_attempts([item, RUNNER.PRIOR_ATTEMPTS[1]])
    assert error.value.kind == 'SOURCE_MISSING'


def test_fresh_root_reservation_refuses_same_root(tmp_path: Path):
    root = tmp_path / 'attempt'
    assert RUNNER.reserve_output_root(root) == root
    with pytest.raises(RUNNER.GateError, match='fresh output root') as error:
        RUNNER.reserve_output_root(root)
    assert error.value.kind == 'ROOT_REUSE'


def test_fake_lifecycle_passes_once_and_cleanup_is_diagnostics_first(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _patch_preflight(monkeypatch)
    fake = _FakeLifecycle(_callback(), _terminal())
    result = RUNNER.run_no_input_gate(tmp_path / 'pass', hooks=fake.hooks())
    assert result['status'] == 'PASS'
    assert result['attempt_index'] == 5
    assert result['execution']['start_count'] == 1
    assert result['execution']['retry_count'] == 0
    assert result['execution']['host_mounts'] == []
    assert result['safety'] == {
        'input_opened': False,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'map_saved': False,
        'formal_replay_started': False,
        'host_mounts_exposed': False,
    }
    assert result['protocol']['first_ack_success'] is True
    assert result['protocol']['duplicate_ack_rejected'] is True
    assert result['terminal']['status'] == 'invalid'
    assert result['cleanup']['stopped_only'] is True
    assert result['cleanup']['stopped_state'] == 'exited'
    assert result['cleanup']['oom_killed'] is False
    assert result['cleanup']['remove_success'] is True
    assert result['artifact_manifest']['container_paths'] == list(
        RUNNER.PINNED_RAW_PATHS + RUNNER.PINNED_DIAGNOSTIC_PATHS
    )
    assert set(result['artifacts']) == set(RUNNER.PINNED_RAW_PATHS)
    assert set(result['artifact_diagnostics']) == set(RUNNER.PINNED_DIAGNOSTIC_PATHS)
    journal_path = Path(result['service_journal_path'])
    journal = [json.loads(line) for line in journal_path.read_text(encoding='utf-8').splitlines()]
    assert [record['order'] for record in journal] == list(range(1, len(fake.service_calls) + 1))
    assert [record['service'] for record in journal] == fake.service_calls
    assert journal_path.stat().st_mode & 0o222 == 0
    assert not any(event[0] == 'copy' for event in fake.events)
    assert fake.read_calls == list(RUNNER.PINNED_RAW_PATHS + RUNNER.PINNED_DIAGNOSTIC_PATHS)
    assert Path(result['receipt_path']).stat().st_mode & 0o222 == 0
    names = [event[0] for event in fake.events]
    assert names.count('run') == 1
    assert names.count('exec') == 1
    assert names.index('run') < names.index('exec') < names.index('ready') < names.index('publish')
    assert (
        names.index('inspect')
        < names.index('stop')
        < names.index('wait_container')
        < names.index('remove')
    )
    assert fake.events.index(
        next(event for event in fake.events if event[0] == 'inspect')
    ) < fake.events.index(next(event for event in fake.events if event[0] == 'stop'))
    assert fake.service_calls == [
        '/m6a10/consumer_status',
        '/m6a10/consumer_ack',
        '/m6a10/consumer_ack',
        '/m6a10/consumer_eof',
        '/m6a10/consumer_status',
        '/m6a10/consumer_finalize',
        '/m6a10/terminal_eof',
        '/m6a10/terminal_status',
        '/m6a10/terminal_status',
        '/m6a10/terminal_finalize',
    ]


def test_duplicate_ack_or_protocol_failure_is_fail_closed_without_retry(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _patch_preflight(monkeypatch)
    fake = _FakeLifecycle(_callback(), _terminal(), ack_ok=False)
    result = RUNNER.run_no_input_gate(tmp_path / 'fail', hooks=fake.hooks())
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'ACK_FAILURE'
    assert result['execution']['start_count'] == 1
    assert result['execution']['retry_count'] == 0
    assert sum(event[0] == 'run' for event in fake.events) == 1
    assert any(event[0] == 'stop' for event in fake.events)
    assert any(event[0] == 'remove' for event in fake.events)


def test_raw_artifact_failure_is_fail_closed_but_keeps_service_journal_and_diagnostics(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
):
    _patch_preflight(monkeypatch)
    fake = _FakeLifecycle(_callback(), _terminal(), artifact_failure=RUNNER.CALLBACK_RAW)
    result = RUNNER.run_no_input_gate(tmp_path / 'artifact-fail', hooks=fake.hooks())
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] == 'ARTIFACT_READ_FAILURE'
    assert result['artifact_errors']
    assert fake.read_calls == [
        RUNNER.CALLBACK_RAW,
        RUNNER.TERMINAL_RAW,
        *RUNNER.PINNED_DIAGNOSTIC_PATHS,
    ]
    journal = Path(result['service_journal_path'])
    records = [json.loads(line) for line in journal.read_text(encoding='utf-8').splitlines()]
    assert len(records) == len(fake.service_calls)
    assert [record['service'] for record in records] == fake.service_calls
    assert journal.stat().st_mode & 0o222 == 0


@pytest.mark.parametrize('cleanup_drift', ['oom', 'not_stopped', 'remove_failure'])
def test_cleanup_drift_cannot_leave_a_pass(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, cleanup_drift: str
):
    _patch_preflight(monkeypatch)
    fake = _FakeLifecycle(_callback(), _terminal())
    hooks = fake.hooks()
    if cleanup_drift == 'oom':
        hooks.wait_container = lambda name, timeout: {
            'status': 'exited',
            'exit_code': 0,
            'oom_killed': True,
        }
    elif cleanup_drift == 'not_stopped':
        hooks.wait_container = lambda name, timeout: {
            'status': 'running',
            'exit_code': None,
            'oom_killed': False,
        }
    else:
        hooks.remove = lambda name: {'returncode': 1}
    result = RUNNER.run_no_input_gate(tmp_path / cleanup_drift, hooks=hooks)
    assert result['status'] == 'FAIL_CLOSED'
    assert result['failure_kind'] in {'CLEANUP_FAILURE', 'CLEANUP_NOT_STOPPED'}


def test_raw_validators_reject_authority_and_terminal_boundary_drift():
    callback = _callback()
    callback['profile_sha256'] = 'host-authority'
    with pytest.raises(RUNNER.GateError) as callback_error:
        RUNNER.validate_callback(callback)
    assert callback_error.value.kind == 'CALLBACK_AUTHORITY_CONTAMINATION'

    terminal = _terminal()
    terminal['backend']['completed_boundary']['observed'] = True
    with pytest.raises(RUNNER.GateError) as terminal_error:
        RUNNER.validate_terminal(terminal)
    assert terminal_error.value.kind == 'TERMINAL_REASON'


@pytest.mark.parametrize(
    'mutation',
    [
        'phase_mode',
        'system',
        'benchmark_only',
        'expected_counts',
        'processed',
        'ack_semantics',
        'eof',
        'transport',
        'queue_capacity',
        'counter',
        'latency',
        'timestamp',
        'safety',
    ],
)
def test_callback_validator_rejects_each_contract_drift(mutation: str):
    value = _callback()
    if mutation == 'phase_mode':
        value['phase_mode'] = 'paced_1x'
    elif mutation == 'system':
        value['system'] = 'other'
    elif mutation == 'benchmark_only':
        value['benchmark_only'] = False
    elif mutation == 'expected_counts':
        value['consumer']['expected_topic_counts']['lidar'] = 0
    elif mutation == 'processed':
        value['consumer']['processed_messages'] = 0
    elif mutation == 'ack_semantics':
        value['consumer']['ack_semantics'] = 'backend_completion'
    elif mutation == 'eof':
        value['consumer']['eof_observed'] = False
    elif mutation == 'transport':
        value['consumer']['transport_outstanding_at_drain'] = 1
    elif mutation == 'queue_capacity':
        value['consumer']['queue_capacity_messages'] = 2
    elif mutation == 'counter':
        value['consumer']['processing_failures'] = 1
    elif mutation == 'latency':
        value['consumer']['maximum_callback_latency_seconds'] = 0.3
    elif mutation == 'timestamp':
        value['consumer']['required_end_timestamp_seconds'] = 9.0
    elif mutation == 'safety':
        value['scorer_invoked'] = True
    with pytest.raises(RUNNER.GateError):
        RUNNER.validate_callback(value)


@pytest.mark.parametrize(
    'mutation',
    [
        'phase_mode',
        'system',
        'top_counts',
        'backend_state',
        'boundary',
        'backend_counts',
        'inflight',
        'buffer',
        'lidar_record',
        'support',
        'poll_count',
        'poll_gap',
        'trajectory',
        'trajectory_timestamp',
        'safety',
    ],
)
def test_terminal_validator_rejects_each_no_boundary_contract_drift(mutation: str):
    value = _terminal()
    if mutation == 'phase_mode':
        value['phase_mode'] = 'paced_1x'
    elif mutation == 'system':
        value['system'] = 'other'
    elif mutation == 'top_counts':
        value['completed_counts']['lidar'] = 1
    elif mutation == 'backend_state':
        value['backend']['quiescent'] = True
    elif mutation == 'boundary':
        value['backend']['completed_boundary']['timestamp_seconds'] = 10.0
    elif mutation == 'backend_counts':
        value['backend']['completed_counts']['lidar'] = 1
    elif mutation == 'inflight':
        value['backend']['in_flight']['active'] = False
    elif mutation == 'buffer':
        value['buffers']['lidar']['count'] = 0
    elif mutation == 'lidar_record':
        value['buffers']['lidar']['records'][0]['support_context_proven'] = True
    elif mutation == 'support':
        value['terminal_support_context']['total_count'] = 1
    elif mutation == 'poll_count':
        value['terminal_observation']['poll_count'] = 1
    elif mutation == 'poll_gap':
        value['terminal_observation']['stable_poll_wall_seconds'] = 10.01
    elif mutation == 'trajectory':
        value['trajectory']['coverage_verified'] = False
    elif mutation == 'trajectory_timestamp':
        value['trajectory']['end_gap_seconds'] = 0.1
    elif mutation == 'safety':
        value['ground_truth_content_opened'] = True
    with pytest.raises(RUNNER.GateError):
        RUNNER.validate_terminal(value)


def test_run_argv_rejects_mount_and_bare_rm_tampering():
    argv = RUNNER.build_run_argv('m6a10-v12-no-input-test')
    with pytest.raises(RUNNER.GateError):
        RUNNER.validate_run_argv([*argv, '--mount', 'type=bind'])
    with pytest.raises(RUNNER.GateError):
        RUNNER.validate_run_argv([*argv[:2], '--rm', *argv[2:]])
