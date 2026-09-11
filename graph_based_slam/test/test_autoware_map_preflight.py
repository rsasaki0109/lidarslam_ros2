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

"""Regression tests for the Autoware map preflight helper."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import shlex
import subprocess
import sys
from types import SimpleNamespace

import jsonschema
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'preflight_autoware_map_bag.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('preflight_autoware_map_bag', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_product_reader_logging_hides_info_but_keeps_warnings(
    monkeypatch,
    capfd,
):
    rclpy_logging = pytest.importorskip('rclpy.logging')
    if not hasattr(rclpy_logging, 'get_logger_level'):
        pytest.skip('rclpy logger-level inspection is unavailable')
    module = _load_module()
    logger_name = module.ROSBAG_STORAGE_LOGGER
    previous_level = rclpy_logging.get_logger_level(logger_name)
    monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map doctor')

    try:
        rclpy_logging.set_logger_level(
            logger_name,
            rclpy_logging.LoggingSeverity.UNSET,
        )
        logger = rclpy_logging.get_logger(logger_name)
        with module._product_rosbag_open_logging():
            assert rclpy_logging.get_logger_level(logger_name) == (
                rclpy_logging.LoggingSeverity.WARN
            )
            logger.info('routine product reader info')
            logger.warning('actionable product reader warning')
        assert rclpy_logging.get_logger_level(logger_name) == (
            rclpy_logging.LoggingSeverity.UNSET
        )
    finally:
        rclpy_logging.set_logger_level(logger_name, previous_level)

    stderr = capfd.readouterr().err
    assert 'routine product reader info' not in stderr
    assert 'actionable product reader warning' in stderr


def test_reader_logging_preserves_direct_and_explicit_levels(monkeypatch):
    rclpy_logging = pytest.importorskip('rclpy.logging')
    if not hasattr(rclpy_logging, 'get_logger_level'):
        pytest.skip('rclpy logger-level inspection is unavailable')
    module = _load_module()
    logger_name = module.ROSBAG_STORAGE_LOGGER
    previous_level = rclpy_logging.get_logger_level(logger_name)

    try:
        rclpy_logging.set_logger_level(
            logger_name,
            rclpy_logging.LoggingSeverity.UNSET,
        )
        monkeypatch.delenv('LIDARSLAM_CLI_COMMAND', raising=False)
        with module._product_rosbag_open_logging():
            assert rclpy_logging.get_logger_level(logger_name) == (
                rclpy_logging.LoggingSeverity.UNSET
            )

        rclpy_logging.set_logger_level(
            logger_name,
            rclpy_logging.LoggingSeverity.DEBUG,
        )
        monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map doctor')
        with module._product_rosbag_open_logging():
            assert rclpy_logging.get_logger_level(logger_name) == (
                rclpy_logging.LoggingSeverity.DEBUG
            )
    finally:
        rclpy_logging.set_logger_level(logger_name, previous_level)


def _write_metadata(tmp_path: Path, topics: list[tuple[str, str, int]]) -> Path:
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 12_500_000_000},
            'message_count': sum(count for _, _, count in topics),
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': name,
                        'type': msg_type,
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': count,
                }
                for name, msg_type, count in topics
            ],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    return bag_dir


def _compatible_inspection(_bag_path: Path, topic: str, _storage_id: str) -> dict:
    return {
        'status': 'inspected',
        'topic': topic,
        'fields': [
            {'name': 'x', 'datatype': 7, 'count': 1},
            {'name': 'y', 'datatype': 7, 'count': 1},
            {'name': 'z', 'datatype': 7, 'count': 1},
            {'name': 'time', 'datatype': 7, 'count': 1},
        ],
        'rko_lio_compatible': True,
        'timestamp_field': 'time',
        'reason': "RKO-LIO-compatible per-point timestamp field 'time' was found.",
    }


def _monotonic_timestamp_inspection(
    _bag_path: Path,
    topics,
    _storage_id: str,
    max_records_per_topic: int,
) -> dict:
    return {
        'status': 'passed',
        'timestamp_source': 'header.stamp',
        'max_records_per_topic': max_records_per_topic,
        'failed_topics': [],
        'topics': [
            {
                'topic': topic.name,
                'msg_type': topic.msg_type,
                'expected_records': topic.message_count,
                'records_scanned': topic.message_count,
                'complete': True,
                'readable': True,
                'first_stamp_ns': 1_000_000_000,
                'last_stamp_ns': 2_000_000_000,
                'reversal_count': 0,
                'invalid_stamp_count': 0,
                'max_backward_jump_ns': 0,
            }
            for topic in topics
        ],
        'reason': (
            'All selected PointCloud2/Imu header timestamps are monotonic.'
        ),
    }


def _odometry_tf_inspection(
    *,
    status: str,
    tf_path: list[str] | None = None,
    dynamic_path: bool | None = None,
    frame_ids: list[str] | None = None,
    child_frame_ids: list[str] | None = None,
    invalid_frame_count: int = 0,
    readable: bool = True,
):
    def inspect(
        _bag_path: Path,
        odometry_topic,
        tf_topics,
        _storage_id: str,
        max_records_per_topic: int,
    ) -> dict:
        parents = ['odom'] if frame_ids is None else frame_ids
        children = ['base_link'] if child_frame_ids is None else child_frame_ids
        path = [] if tf_path is None else tf_path
        path_edges = [
            {
                'from_frame': left,
                'to_frame': right,
                'dynamic': dynamic_path is True and index == 0,
            }
            for index, (left, right) in enumerate(zip(path, path[1:]))
        ]
        return {
            'status': status,
            'odometry_topic': odometry_topic.name,
            'expected_records': odometry_topic.message_count,
            'records_scanned': odometry_topic.message_count if readable else 0,
            'complete': True,
            'readable': readable,
            'header_frame_id': parents[0] if len(parents) == 1 else None,
            'child_frame_id': children[0] if len(children) == 1 else None,
            'frame_ids': parents,
            'child_frame_ids': children,
            'invalid_frame_count': invalid_frame_count,
            'tf_topics': [
                {
                    'topic': topic.name,
                    'expected_records': topic.message_count,
                    'records_scanned': topic.message_count,
                    'complete': True,
                    'readable': topic.message_count > 0,
                }
                for topic in tf_topics
            ],
            'tf_records_scanned': sum(topic.message_count for topic in tf_topics),
            'invalid_transform_count': 0,
            'tf_path': path,
            'path_edges': path_edges,
            'dynamic_path': dynamic_path,
            'max_records_per_topic': max_records_per_topic,
            'reason': {
                'passed': 'A dynamic TF path connects the Odometry frames.',
                'sampled': 'A sampled dynamic TF path connects the Odometry frames.',
                'failed': 'The Odometry frames do not have a usable dynamic TF path.',
            }[status],
        }

    return inspect


def _future_tf_timing_inspection(
    _bag_path: Path,
    pointcloud_topic,
    tf_topics,
    path_edges,
    _storage_id: str,
    max_records_per_topic: int,
) -> dict:
    dynamic_edges = [edge for edge in path_edges if edge['dynamic']]
    return {
        'status': 'failed',
        'sample_basis': 'bag_record_order_and_header_stamp',
        'pointcloud_topic': pointcloud_topic.name,
        'expected_pointcloud_records': pointcloud_topic.message_count,
        'pointcloud_records_scanned': pointcloud_topic.message_count,
        'complete': True,
        'readable': True,
        'dynamic_path_edges': dynamic_edges,
        'tf_topics': [
            {
                'topic': topic.name,
                'expected_records': topic.message_count,
                'records_scanned': topic.message_count,
                'complete': True,
                'readable': True,
            }
            for topic in tf_topics
        ],
        'tf_records_scanned': sum(topic.message_count for topic in tf_topics),
        'invalid_pointcloud_stamp_count': 0,
        'invalid_transform_stamp_count': 0,
        'clouds_before_all_dynamic_tf': 0,
        'future_extrapolation_count': 1,
        'first_future_cloud_stamp_ns': 20_364_000_000,
        'max_future_gap_ns': 18_000_000,
        'max_future_gap_edge': dynamic_edges[0],
        'latest_required_tf_stamp_ns_at_max_gap': 20_346_000_000,
        'max_records_per_topic': max_records_per_topic,
        'reason': 'A cloud requested a future dynamic TF.',
    }


def test_rko_lio_public_path_is_preferred_for_pointcloud_and_imu(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/imu/data', 'sensor_msgs/msg/Imu', 2000),
            ('/gnss/fix', 'sensor_msgs/msg/NavSatFix', 500),
        ],
    )

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
    )

    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'preflight-v6.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(payload, schema)
    assert payload['schema_version'] == 6
    assert payload['schema_uri'].endswith('/schemas/preflight-v6.schema.json')
    assert payload['summary']['pointcloud_inspection']['timestamp_field'] == 'time'
    assert payload['summary']['timestamp_order']['status'] == 'passed'
    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'
    assert payload['findings'] == []
    assert payload['beginner_commands'][0]['command'].startswith(
        'bash scripts/run_autoware_map_beginner.sh'
    )
    assert payload['recommendations'][0]['command'].startswith(
        'ros2 launch lidarslam rko_lio_slam.launch.py'
    )
    assert any(item['id'] == 'pointcloud_gnss_smoke' for item in payload['recommendations'])
    report = module.render_text_report(payload)
    assert 'Recommended path: RKO-LIO + graph_based_slam public path' in report
    assert 'Beginner command:' in report
    assert 'Beginner command with browser viewer:' in report
    assert 'run_autoware_map_beginner.sh' in report
    assert 'inspect_navsatfix_covariance.py' in report


def test_public_doctor_evidence_is_schema_valid_and_path_free(tmp_path: Path):
    module = _load_module()
    private_root = tmp_path / 'private-site-alice'
    private_root.mkdir()
    bag_dir = _write_metadata(
        private_root,
        [
            ('/private_vehicle/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/private_vehicle/imu', 'sensor_msgs/msg/Imu', 2000),
        ],
    )
    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
    )

    evidence = module.build_public_preflight_evidence(payload)
    schema = json.loads((
        REPO_ROOT
        / 'docs'
        / 'schemas'
        / 'public-doctor-evidence-v1.schema.json'
    ).read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(evidence, schema)

    encoded = json.dumps(evidence, sort_keys=True)
    assert evidence['status'] == 'ready'
    assert evidence['recommended_profile_id'] == (
        'rko_lio_graph_public_path'
    )
    assert evidence['finding_codes'] == []
    assert evidence['first_action_code'] is None
    assert evidence['input']['topic_type_counts'] == [
        {
            'msg_type': 'sensor_msgs/msg/Imu',
            'topic_count': 1,
            'message_count': 2000,
        },
        {
            'msg_type': 'sensor_msgs/msg/PointCloud2',
            'topic_count': 1,
            'message_count': 200,
        },
    ]
    assert evidence['privacy'] == {
        'bag_path_included': False,
        'topic_or_frame_names_included': False,
        'local_commands_included': False,
        'raw_sensor_data_included': False,
        'raw_logs_included': False,
        'free_text_messages_included': False,
        'review_before_sharing': True,
    }
    for private_value in (
        str(bag_dir),
        'private-site-alice',
        '/private_vehicle/points',
        '/private_vehicle/imu',
        'run_autoware_map_beginner.sh',
    ):
        assert private_value not in encoded


def test_human_doctor_prints_one_shell_safe_public_support_handoff(
    tmp_path: Path,
    monkeypatch,
):
    module = _load_module()
    private_root = tmp_path / 'private site alice'
    private_root.mkdir()
    bag_dir = _write_metadata(
        private_root,
        [
            ('/private_vehicle/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/private_vehicle/imu', 'sensor_msgs/msg/Imu', 2000),
        ],
    )
    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
    )
    monkeypatch.delenv('LIDARSLAM_CLI_COMMAND', raising=False)

    direct_report = module.render_text_report(payload)
    direct_command = direct_report.splitlines()[-1].strip()

    assert 'Autoware-Compatible Map Preflight' in direct_report
    assert 'Detected inputs:' in direct_report
    assert 'Beginner command:' in direct_report
    assert 'run_autoware_map_beginner.sh' in direct_report
    assert shlex.split(direct_command) == [
        'python3',
        'scripts/preflight_autoware_map_bag.py',
        str(bag_dir),
        '--public-json',
    ]

    monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map doctor')

    report = module.render_text_report(payload)
    report_lines = report.splitlines()
    delegated_command = report.splitlines()[-1].strip()

    assert report.count('Need public support?') == 1
    assert 'Need local details?' in report
    assert 'Keep this JSON local' in report
    assert 'review it before sharing' in report
    assert 'Status:   READY' in report
    assert 'Inputs:   PointCloud2, Imu' in report
    assert 'Profile:  RKO-LIO + graph_based_slam public path' in report
    assert 'Detected inputs:' not in report
    assert '/private_vehicle/points' not in report
    assert '/private_vehicle/imu' not in report
    assert len(report_lines) <= 26
    action_index = report_lines.index('Do this now:')
    assert shlex.split(report_lines[action_index + 1].strip()) == [
        'lidarslam-map',
        'start',
        str(bag_dir),
    ]
    assert 'Beginner command:' not in report
    assert 'Other compatible paths:' not in report
    assert 'run_autoware_map_beginner.sh' not in report
    detail_index = report_lines.index('Need local details?')
    assert shlex.split(report_lines[detail_index + 2].strip()) == [
        'lidarslam-map',
        'doctor',
        str(bag_dir),
        '--json',
    ]
    assert shlex.split(delegated_command) == [
        'lidarslam-map',
        'doctor',
        str(bag_dir),
        '--public-json',
    ]

    action_required = {
        **payload,
        'findings': [{
            'code': 'calibration-review-required',
            'message': 'Review the measured sensor transforms.',
            'next_action': 'Record and review the measured transforms.',
        }],
    }
    blocked_report = module.render_text_report(action_required)
    blocked_lines = blocked_report.splitlines()
    retry_index = blocked_lines.index('Rerun after that action:')
    assert 'Status:   ACTION REQUIRED' in blocked_report
    assert blocked_report.count('Do this now:') == 1
    assert '[calibration-review-required]' in blocked_report
    assert shlex.split(blocked_lines[retry_index + 1].strip()) == [
        'lidarslam-map',
        'doctor',
        str(bag_dir),
    ]
    assert 'lidarslam-map start' not in blocked_report

    action_required['findings'].append({
        'code': 'timestamp-order-invalid',
        'message': 'This detailed follow-up message stays out of the card.',
        'next_action': 'This detailed follow-up action stays out of the card.',
    })
    bounded_report = module.render_text_report(action_required)
    assert 'Follow-up finding codes: timestamp-order-invalid' in bounded_report
    assert 'detailed follow-up message' not in bounded_report
    assert 'detailed follow-up action' not in bounded_report


def test_public_doctor_input_error_never_echoes_the_private_path(
    tmp_path: Path,
):
    private_path = tmp_path / 'private-site-alice' / 'missing-bag'
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(private_path),
            '--public-json',
        ],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert result.stderr == ''
    assert str(private_path) not in result.stdout
    evidence = json.loads(result.stdout)
    schema = json.loads((
        REPO_ROOT
        / 'docs'
        / 'schemas'
        / 'public-doctor-evidence-v1.schema.json'
    ).read_text(encoding='utf-8'))
    jsonschema.validate(evidence, schema)
    assert evidence['status'] == 'input_error'
    assert evidence['finding_codes'] == ['bag-preflight-input-error']
    assert evidence['first_action_code'] == 'bag-preflight-input-error'


def test_odometry_without_tf_is_visible_beside_compatible_path(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/imu/data', 'sensor_msgs/msg/Imu', 2000),
            ('/odom', 'nav_msgs/msg/Odometry', 1000),
        ],
    )

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
        odometry_tf_inspector=_odometry_tf_inspection(status='failed'),
    )

    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'
    assert payload['summary']['capabilities']['has_odometry'] is True
    assert payload['summary']['topics']['odometry'][0]['name'] == '/odom'
    assert payload['findings'][-1]['code'] == 'odometry-tf-missing'
    assert 'does not publish that transform by itself' in payload['findings'][-1][
        'message'
    ]
    report = module.render_text_report(payload)
    assert 'Recommended path: RKO-LIO + graph_based_slam public path' in report
    assert '[odometry-tf-missing]' in report

    evidence = module.build_public_preflight_evidence(payload)
    schema = json.loads((
        REPO_ROOT
        / 'docs'
        / 'schemas'
        / 'public-doctor-evidence-v1.schema.json'
    ).read_text(encoding='utf-8'))
    jsonschema.validate(evidence, schema)
    encoded = json.dumps(evidence, sort_keys=True)
    assert evidence['status'] == 'action_required'
    assert evidence['finding_codes'] == ['odometry-tf-missing']
    assert evidence['first_action_code'] == 'odometry-tf-missing'
    assert evidence['checks']['odometry_tf']['status'] == 'failed'
    for private_value in (
        str(bag_dir),
        '/points',
        '/imu/data',
        '/odom',
        'does not publish that transform by itself',
        'run_autoware_map_beginner.sh',
    ):
        assert private_value not in encoded


def test_future_tf_timing_is_actionable_but_keeps_compatible_path(
    tmp_path: Path,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/imu/data', 'sensor_msgs/msg/Imu', 2000),
            ('/odom', 'nav_msgs/msg/Odometry', 1000),
            ('/tf', 'tf2_msgs/msg/TFMessage', 1000),
        ],
    )

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
        odometry_tf_inspector=_odometry_tf_inspection(
            status='passed',
            tf_path=['odom', 'base_link'],
            dynamic_path=True,
        ),
        odometry_tf_timing_inspector=_future_tf_timing_inspection,
    )

    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'preflight-v6.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.validate(payload, schema)
    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'
    finding = next(
        item for item in payload['findings']
        if item['code'] == 'odometry-tf-future-gap'
    )
    assert '18.000 ms' in finding['message']
    assert 'Do not silence' in finding['next_action']
    report = module.render_text_report(payload)
    assert 'Odometry TF timing: failed' in report
    assert '[odometry-tf-future-gap]' in report


def test_multihop_dynamic_odometry_tf_path_passes_without_finding(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/odom', 'nav_msgs/msg/Odometry', 100),
            ('/tf', 'tf2_msgs/msg/TFMessage', 100),
            ('/tf_static', 'tf2_msgs/msg/TFMessage', 1),
        ],
    )

    payload = module.build_preflight_payload(
        bag_dir,
        odometry_tf_inspector=_odometry_tf_inspection(
            status='passed',
            tf_path=['odom', 'base_footprint', 'base_link'],
            dynamic_path=True,
        ),
    )

    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'preflight-v6.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.validate(payload, schema)
    assert payload['summary']['odometry_tf']['status'] == 'passed'
    assert not any(
        finding['code'].startswith('odometry-')
        for finding in payload['findings']
    )


def test_static_only_odometry_tf_path_has_specific_action(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/odom', 'nav_msgs/msg/Odometry', 100),
            ('/tf_static', 'tf2_msgs/msg/TFMessage', 1),
        ],
    )

    payload = module.build_preflight_payload(
        bag_dir,
        odometry_tf_inspector=_odometry_tf_inspection(
            status='failed',
            tf_path=['odom', 'base_link'],
            dynamic_path=False,
        ),
    )

    finding = next(
        item for item in payload['findings']
        if item['code'] == 'odometry-tf-static-only'
    )
    assert 'dynamic /tf broadcaster' in finding['next_action']


def test_inconsistent_odometry_frames_have_specific_action(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [('/odom', 'nav_msgs/msg/Odometry', 100)],
    )

    payload = module.build_preflight_payload(
        bag_dir,
        odometry_tf_inspector=_odometry_tf_inspection(
            status='failed',
            frame_ids=['map', 'odom'],
            invalid_frame_count=1,
        ),
    )

    finding = next(
        item for item in payload['findings']
        if item['code'] == 'odometry-frame-invalid'
    )
    assert 'header.frame_id and child_frame_id' in finding['next_action']


def test_tf_path_selection_is_deterministic_and_requires_motion_edge():
    module = _load_module()
    edges = {
        ('base_footprint', 'odom'): True,
        ('base_footprint', 'base_link'): False,
        ('base_link', 'sensor'): False,
        ('map', 'odom'): False,
    }

    assert module.find_tf_path(
        edges,
        'odom',
        'base_link',
        require_dynamic=True,
    ) == (['odom', 'base_footprint', 'base_link'], True)
    assert module.find_tf_path(
        {('base_link', 'odom'): False},
        'odom',
        'base_link',
        require_dynamic=True,
    ) is None


def _message_with_stamp(stamp_ns: int):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=stamp_ns // 1_000_000_000,
                nanosec=stamp_ns % 1_000_000_000,
            ),
        ),
    )


def _transform_with_stamp(parent: str, child: str, stamp_ns: int):
    transform = _message_with_stamp(stamp_ns)
    transform.header.frame_id = parent
    transform.child_frame_id = child
    return transform


def _timing_state(module, path_edges):
    return module._odometry_tf_timing_state(
        module.TopicRecord('/points', module.POINTCLOUD2, 1),
        [module.TopicRecord('/tf', module.TFMESSAGE, len(path_edges))],
        path_edges,
        100,
    )


def test_odometry_tf_timing_detects_issue_64_future_gap():
    module = _load_module()
    state = _timing_state(module, [{
        'from_frame': 'odom',
        'to_frame': 'base_link',
        'dynamic': True,
    }])
    transform = _transform_with_stamp(
        'odom',
        'base_link',
        20_346_000_000,
    )

    module._record_odometry_tf_timing_transform(
        state,
        '/tf',
        SimpleNamespace(transforms=[transform]),
    )
    module._record_odometry_tf_timing_pointcloud(
        state,
        _message_with_stamp(20_364_000_000),
    )
    result = module._finalize_odometry_tf_timing_scan(
        state,
        exhausted=True,
    )

    assert result['status'] == 'failed'
    assert result['future_extrapolation_count'] == 1
    assert result['first_future_cloud_stamp_ns'] == 20_364_000_000
    assert result['max_future_gap_ns'] == 18_000_000
    assert result['latest_required_tf_stamp_ns_at_max_gap'] == 20_346_000_000
    assert result['max_future_gap_edge'] == {
        'from_frame': 'odom',
        'to_frame': 'base_link',
        'dynamic': True,
    }


def test_odometry_tf_timing_separates_startup_from_future_gap():
    module = _load_module()
    state = _timing_state(module, [{
        'from_frame': 'odom',
        'to_frame': 'base_link',
        'dynamic': True,
    }])

    module._record_odometry_tf_timing_pointcloud(
        state,
        _message_with_stamp(1_000_000_000),
    )
    result = module._finalize_odometry_tf_timing_scan(
        state,
        exhausted=True,
    )

    assert result['status'] == 'failed'
    assert result['clouds_before_all_dynamic_tf'] == 1
    assert result['future_extrapolation_count'] == 0


def test_odometry_tf_timing_uses_limiting_dynamic_edge_and_ignores_static():
    module = _load_module()
    path_edges = [
        {
            'from_frame': 'odom',
            'to_frame': 'base_footprint',
            'dynamic': True,
        },
        {
            'from_frame': 'base_footprint',
            'to_frame': 'base_link',
            'dynamic': True,
        },
        {
            'from_frame': 'base_link',
            'to_frame': 'lidar',
            'dynamic': False,
        },
    ]
    state = _timing_state(module, path_edges)
    state['_tf_topics']['/tf']['expected_records'] = 1
    transforms = [
        _transform_with_stamp('odom', 'base_footprint', 2_000_000_000),
        _transform_with_stamp('base_footprint', 'base_link', 1_900_000_000),
        _transform_with_stamp('base_link', 'lidar', 1_000_000_000),
    ]

    module._record_odometry_tf_timing_transform(
        state,
        '/tf',
        SimpleNamespace(transforms=transforms),
    )
    module._record_odometry_tf_timing_pointcloud(
        state,
        _message_with_stamp(1_950_000_000),
    )
    result = module._finalize_odometry_tf_timing_scan(
        state,
        exhausted=True,
    )

    assert result['future_extrapolation_count'] == 1
    assert result['max_future_gap_ns'] == 50_000_000
    assert result['max_future_gap_edge']['from_frame'] == 'base_footprint'
    assert result['dynamic_path_edges'] == path_edges[:2]


def test_odometry_tf_timing_passes_clean_replay_order():
    module = _load_module()
    state = _timing_state(module, [{
        'from_frame': 'odom',
        'to_frame': 'base_link',
        'dynamic': True,
    }])
    transform = _transform_with_stamp(
        'odom',
        'base_link',
        2_000_000_000,
    )

    module._record_odometry_tf_timing_transform(
        state,
        '/tf',
        SimpleNamespace(transforms=[transform]),
    )
    module._record_odometry_tf_timing_pointcloud(
        state,
        _message_with_stamp(2_000_000_000),
    )
    result = module._finalize_odometry_tf_timing_scan(
        state,
        exhausted=True,
    )

    assert result['status'] == 'passed'
    assert result['future_extrapolation_count'] == 0
    assert result['clouds_before_all_dynamic_tf'] == 0


def test_packet_applanix_path_is_recommended_when_packet_topics_exist(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/front/packets', 'velodyne_msgs/msg/VelodyneScan', 300),
            ('/gsof49', 'applanix_msgs/msg/NavigationSolutionGsof49', 1500),
            ('/gsof50', 'applanix_msgs/msg/NavigationPerformanceGsof50', 150),
        ],
    )

    payload = module.build_preflight_payload(bag_dir)

    assert payload['recommended_profile_id'] == 'packet_applanix_smoke'
    assert payload['recommendations'][0]['command'].startswith(
        'bash scripts/run_open_data_applanix_velodyne_gnss_smoke.sh'
    )
    assert payload['advisory'][0]['command'].startswith(
        'python3 scripts/inspect_applanix_gsof50_quality.py'
    )


def test_cli_json_output_matches_machine_readable_payload(tmp_path: Path):
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 100),
            ('/tf', 'tf2_msgs/msg/TFMessage', 200),
        ],
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(bag_dir), '--json'],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(result.stdout)

    assert payload['recommended_profile_id'] is None
    assert payload['beginner_commands'] == []
    assert payload['summary']['capabilities']['has_pointcloud2'] is True
    assert payload['summary']['capabilities']['has_imu'] is False
    assert any('No Imu topic was found' in item for item in payload['missing_requirements'])
    assert [finding['code'] for finding in payload['findings']] == [
        'timestamp-inspection-unavailable',
        'imu-input-missing',
        'navsatfix-input-missing',
        'applanix-gsof49-input-missing',
    ]
    assert all(finding['next_action'] for finding in payload['findings'])

    public_result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(bag_dir), '--public-json'],
        check=True,
        capture_output=True,
        text=True,
    )
    evidence = json.loads(public_result.stdout)
    assert evidence['status'] == 'action_required'
    assert evidence['first_action_code'] == (
        'timestamp-inspection-unavailable'
    )
    assert str(bag_dir) not in public_result.stdout
    assert '/points' not in public_result.stdout
    assert '/tf' not in public_result.stdout
    assert 'Run ros2 bag info' not in public_result.stdout


def test_cli_help_is_user_facing():
    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'Autoware-compatible map workflow' in result.stdout
    assert 'The input must be the rosbag2 directory' in result.stdout
    assert 'Pass /path/to/rosbag2, not /path/to/rosbag2_0.db3.' in result.stdout
    assert 'Profiles this tool can recommend:' in result.stdout
    assert 'rko_lio_graph_public_path' in result.stdout
    assert '--json' in result.stdout


def test_cli_rejects_missing_bag_without_traceback(tmp_path: Path):
    missing_bag = tmp_path / 'missing_bag'

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(missing_bag)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'rosbag2 directory does not exist' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_rejects_db3_file_without_traceback(tmp_path: Path):
    db3_path = tmp_path / 'demo_0.db3'
    db3_path.write_text('', encoding='utf-8')

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(db3_path)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'not the .db3 file' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_rejects_missing_metadata_without_traceback(tmp_path: Path):
    bag_dir = tmp_path / 'bag_without_metadata'
    bag_dir.mkdir()

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(bag_dir)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'metadata.yaml not found' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_rejects_invalid_metadata_yaml_without_traceback(tmp_path: Path):
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    (bag_dir / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: [invalid',
        encoding='utf-8',
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(bag_dir)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'failed to parse' in result.stderr
    assert 'Traceback' not in result.stderr


def test_livox_mid360_bag_emits_tuned_preset_hint(tmp_path: Path):
    module = _load_module()
    bag_dir = tmp_path / 'mid360_demo_bag'
    bag_dir.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 2_000_000_000},
            'message_count': 200,
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': '/livox/lidar',
                        'type': 'sensor_msgs/msg/PointCloud2',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': 20,
                },
                {
                    'topic_metadata': {
                        'name': '/livox/imu',
                        'type': 'sensor_msgs/msg/Imu',
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': 180,
                },
            ],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
    )
    recommendation_ids = [item['id'] for item in payload['recommendations']]

    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'
    assert 'rko_lio_graph_mid360_preset' in recommendation_ids
    tuned = next(
        item for item in payload['recommendations']
        if item['id'] == 'rko_lio_graph_mid360_preset'
    )
    assert 'lidarslam_mid360_rko_graph.yaml' in tuned['command']
    assert 'rko_lio_mid360.yaml' in tuned['command']


def test_pointcloud_field_assessment_accepts_supported_timestamp():
    module = _load_module()

    assessment = module.assess_pointcloud_fields([
        {'name': 'x', 'datatype': 7, 'count': 1},
        {'name': 'y', 'datatype': 7, 'count': 1},
        {'name': 'z', 'datatype': 7, 'count': 1},
        {'name': 'timestamp', 'datatype': 8, 'count': 1},
    ])

    assert assessment['rko_lio_compatible'] is True
    assert assessment['timestamp_field'] == 'timestamp'


def test_pointcloud_field_assessment_rejects_missing_timestamp():
    module = _load_module()

    assessment = module.assess_pointcloud_fields([
        {'name': 'x', 'datatype': 7, 'count': 1},
        {'name': 'y', 'datatype': 7, 'count': 1},
        {'name': 'z', 'datatype': 7, 'count': 1},
        {'name': 'intensity', 'datatype': 7, 'count': 1},
    ])

    assert assessment['rko_lio_compatible'] is False
    assert assessment['timestamp_field'] is None
    assert 'expected t/timestamp/time/stamps' in assessment['reason']


def test_pointcloud_field_assessment_rejects_unsupported_timestamp_type():
    module = _load_module()

    assessment = module.assess_pointcloud_fields([
        {'name': 'x', 'datatype': 7, 'count': 1},
        {'name': 'y', 'datatype': 7, 'count': 1},
        {'name': 'z', 'datatype': 7, 'count': 1},
        {'name': 'time', 'datatype': 2, 'count': 1},
    ])

    assert assessment['rko_lio_compatible'] is False
    assert 'UINT32, FLOAT32, or FLOAT64' in assessment['reason']


def test_missing_point_timestamp_prevents_rko_recommendation(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/imu/data', 'sensor_msgs/msg/Imu', 2000),
        ],
    )

    def incompatible(_bag_path: Path, topic: str, _storage_id: str) -> dict:
        return {
            'status': 'inspected',
            'topic': topic,
            'fields': [
                {'name': 'x', 'datatype': 7, 'count': 1},
                {'name': 'y', 'datatype': 7, 'count': 1},
                {'name': 'z', 'datatype': 7, 'count': 1},
            ],
            'rko_lio_compatible': False,
            'timestamp_field': None,
            'reason': (
                'PointCloud2 has no per-point timestamp field required for '
                'RKO-LIO deskewing (expected t/timestamp/time/stamps).'
            ),
        }

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=incompatible,
    )

    assert payload['recommended_profile_id'] is None
    assert payload['recommendations'] == []
    assert payload['beginner_commands'] == []
    assert any(
        'expected t/timestamp/time/stamps' in item
        for item in payload['missing_requirements']
    )
    layout_finding = next(
        finding
        for finding in payload['findings']
        if finding['code'] == 'pointcloud-layout-incompatible'
    )
    assert 'lidarslam-map doctor <rosbag2_dir>' in layout_finding['next_action']


def test_reader_failure_is_not_mislabeled_as_pointcloud_layout_failure(
    tmp_path: Path,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/imu/data', 'sensor_msgs/msg/Imu', 2000),
        ],
    )

    def failed_pointcloud(_bag_path: Path, topic: str, _storage_id: str) -> dict:
        return {
            'status': 'error',
            'topic': topic,
            'fields': [],
            'rko_lio_compatible': None,
            'timestamp_field': None,
            'reason': 'metadata could not be parsed',
        }

    def failed_timestamps(
        _bag_path: Path,
        _topics,
        _storage_id: str,
        max_records_per_topic: int,
    ) -> dict:
        return {
            'status': 'error',
            'timestamp_source': 'header.stamp',
            'max_records_per_topic': max_records_per_topic,
            'failed_topics': [],
            'topics': [],
            'reason': 'metadata could not be parsed',
        }

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=failed_pointcloud,
        timestamp_inspector=failed_timestamps,
    )

    codes = [finding['code'] for finding in payload['findings']]
    assert codes[:2] == [
        'pointcloud-inspection-unavailable',
        'timestamp-inspection-unavailable',
    ]
    assert 'pointcloud-layout-incompatible' not in codes
    assert all(
        'ros2 bag info <rosbag2_dir>' in finding['next_action']
        for finding in payload['findings'][:2]
    )


def test_timestamp_scan_detects_reversal_and_maximum_jump():
    module = _load_module()
    topic = module.TopicRecord(
        '/points',
        'sensor_msgs/msg/PointCloud2',
        4,
    )
    states = module._timestamp_scan_state([topic], 10)

    for stamp_ns in (1_000, 2_000, 1_500, 3_000):
        module._record_timestamp_sample(states['/points'], stamp_ns)
    result = module._finalize_timestamp_scan(
        states,
        exhausted=True,
        max_records_per_topic=10,
    )

    assert result['status'] == 'failed'
    assert result['failed_topics'] == ['/points']
    assert result['topics'][0]['records_scanned'] == 4
    assert result['topics'][0]['complete'] is True
    assert result['topics'][0]['reversal_count'] == 1
    assert result['topics'][0]['max_backward_jump_ns'] == 500


def test_timestamp_scan_distinguishes_bounded_sample_from_full_pass():
    module = _load_module()
    topic = module.TopicRecord(
        '/imu',
        'sensor_msgs/msg/Imu',
        100,
    )
    states = module._timestamp_scan_state([topic], 3)

    for stamp_ns in (1_000, 2_000, 3_000):
        module._record_timestamp_sample(states['/imu'], stamp_ns)
    result = module._finalize_timestamp_scan(
        states,
        exhausted=False,
        max_records_per_topic=3,
    )

    assert result['status'] == 'sampled'
    assert result['failed_topics'] == []
    assert result['topics'][0]['complete'] is False
    assert result['topics'][0]['readable'] is True


def test_timestamp_scan_rejects_invalid_and_unreadable_stamps():
    module = _load_module()
    points = module.TopicRecord(
        '/points',
        'sensor_msgs/msg/PointCloud2',
        1,
    )
    imu = module.TopicRecord('/imu', 'sensor_msgs/msg/Imu', 1)
    states = module._timestamp_scan_state([points, imu], 10)
    module._record_timestamp_sample(states['/points'], None)

    result = module._finalize_timestamp_scan(
        states,
        exhausted=True,
        max_records_per_topic=10,
    )

    assert result['status'] == 'failed'
    assert result['failed_topics'] == ['/points', '/imu']
    assert result['topics'][0]['invalid_stamp_count'] == 1
    assert result['topics'][1]['readable'] is False


def test_timestamp_reversal_blocks_affected_mapping_profile(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 200),
            ('/imu/data', 'sensor_msgs/msg/Imu', 2000),
        ],
    )

    def reversed_timestamps(
        _bag_path: Path,
        topics,
        _storage_id: str,
        max_records_per_topic: int,
    ) -> dict:
        result = _monotonic_timestamp_inspection(
            _bag_path,
            topics,
            _storage_id,
            max_records_per_topic,
        )
        result['status'] = 'failed'
        result['failed_topics'] = ['/points']
        result['topics'][0]['reversal_count'] = 2
        result['topics'][0]['max_backward_jump_ns'] = 750_000_000
        result['reason'] = (
            'Non-monotonic header timestamps were detected on /points.'
        )
        return result

    payload = module.build_preflight_payload(
        bag_dir,
        pointcloud_inspector=_compatible_inspection,
        timestamp_inspector=reversed_timestamps,
    )

    assert payload['recommended_profile_id'] is None
    assert payload['beginner_commands'] == []
    assert any(
        'Header timestamp disorder on /points' in item
        and '0.750000000 s' in item
        for item in payload['missing_requirements']
    )
    timestamp_finding = next(
        finding
        for finding in payload['findings']
        if finding['code'] == 'timestamp-order-invalid'
    )
    assert 'Rewrite header.stamp on /points' in timestamp_finding['next_action']
    report = module.render_text_report(payload)
    assert 'Header timestamp order: failed' in report
    assert '[timestamp-order-invalid]' in report
    assert 'Next:' in report


def test_rosbag2_reader_accepts_dynamic_multihop_odometry_tf(
    tmp_path: Path,
):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from geometry_msgs.msg import TransformStamped  # noqa: PLC0415
    from nav_msgs.msg import Odometry  # noqa: PLC0415
    from rclpy.serialization import serialize_message  # noqa: PLC0415
    from tf2_msgs.msg import TFMessage  # noqa: PLC0415

    module = _load_module()
    bag_dir = tmp_path / 'odometry_tf_bag'

    def topic_metadata(topic_id: int, name: str, msg_type: str):
        kwargs = {
            'name': name,
            'type': msg_type,
            'serialization_format': 'cdr',
        }
        try:
            return rosbag2_py.TopicMetadata(id=topic_id, **kwargs)
        except TypeError:  # Humble TopicMetadata predates the numeric id
            return rosbag2_py.TopicMetadata(**kwargs)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(topic_metadata(0, '/odom', 'nav_msgs/msg/Odometry'))
    writer.create_topic(topic_metadata(1, '/tf', 'tf2_msgs/msg/TFMessage'))
    writer.create_topic(
        topic_metadata(2, '/tf_static', 'tf2_msgs/msg/TFMessage')
    )

    odometry = Odometry()
    odometry.header.frame_id = 'odom'
    odometry.child_frame_id = 'base_link'
    odom_to_footprint = TransformStamped()
    odom_to_footprint.header.frame_id = 'odom'
    odom_to_footprint.child_frame_id = 'base_footprint'
    footprint_to_base = TransformStamped()
    footprint_to_base.header.frame_id = 'base_footprint'
    footprint_to_base.child_frame_id = 'base_link'
    writer.write('/odom', serialize_message(odometry), 1_000_000_000)
    writer.write(
        '/tf',
        serialize_message(TFMessage(transforms=[odom_to_footprint])),
        1_000_000_001,
    )
    writer.write(
        '/tf_static',
        serialize_message(TFMessage(transforms=[footprint_to_base])),
        1_000_000_002,
    )
    if hasattr(writer, 'close'):
        writer.close()

    payload = module.build_preflight_payload(bag_dir)
    result = payload['summary']['odometry_tf']

    assert result['status'] == 'passed'
    assert result['tf_path'] == ['odom', 'base_footprint', 'base_link']
    assert result['dynamic_path'] is True
    assert result['records_scanned'] == 1
    assert result['tf_records_scanned'] == 2


def test_rosbag2_reader_detects_exact_issue_64_future_tf_gap(
    tmp_path: Path,
):
    rosbag2_py = pytest.importorskip('rosbag2_py')
    from geometry_msgs.msg import TransformStamped  # noqa: PLC0415
    from nav_msgs.msg import Odometry  # noqa: PLC0415
    from rclpy.serialization import serialize_message  # noqa: PLC0415
    from sensor_msgs.msg import PointCloud2  # noqa: PLC0415
    from tf2_msgs.msg import TFMessage  # noqa: PLC0415

    module = _load_module()
    bag_dir = tmp_path / 'future_tf_bag'

    def topic_metadata(topic_id: int, name: str, msg_type: str):
        kwargs = {
            'name': name,
            'type': msg_type,
            'serialization_format': 'cdr',
        }
        try:
            return rosbag2_py.TopicMetadata(id=topic_id, **kwargs)
        except TypeError:  # Humble TopicMetadata predates the numeric id
            return rosbag2_py.TopicMetadata(**kwargs)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(topic_metadata(0, '/odom', 'nav_msgs/msg/Odometry'))
    writer.create_topic(topic_metadata(1, '/tf', 'tf2_msgs/msg/TFMessage'))
    writer.create_topic(
        topic_metadata(2, '/points', 'sensor_msgs/msg/PointCloud2')
    )

    odometry = Odometry()
    odometry.header.frame_id = 'odom'
    odometry.child_frame_id = 'base_link'
    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'base_link'
    transform.header.stamp.sec = 20
    transform.header.stamp.nanosec = 346_000_000
    pointcloud = PointCloud2()
    pointcloud.header.frame_id = 'base_link'
    pointcloud.header.stamp.sec = 20
    pointcloud.header.stamp.nanosec = 364_000_000
    writer.write('/odom', serialize_message(odometry), 1_000_000_000)
    writer.write(
        '/tf',
        serialize_message(TFMessage(transforms=[transform])),
        1_000_000_001,
    )
    writer.write(
        '/points',
        serialize_message(pointcloud),
        1_000_000_002,
    )
    if hasattr(writer, 'close'):
        writer.close()

    payload = module.build_preflight_payload(bag_dir)
    result = payload['summary']['odometry_tf_timing']

    assert payload['summary']['odometry_tf']['status'] == 'passed'
    assert result['status'] == 'failed'
    assert result['sample_basis'] == 'bag_record_order_and_header_stamp'
    assert result['future_extrapolation_count'] == 1
    assert result['max_future_gap_ns'] == 18_000_000
    assert any(
        finding['code'] == 'odometry-tf-future-gap'
        for finding in payload['findings']
    )


@pytest.mark.skipif(
    importlib.util.find_spec('rosbags') is None,
    reason='rosbags is required for the real rosbag2 fallback fixture',
)
def test_rosbags_reader_detects_reversal_in_serialized_records(
    tmp_path: Path,
):
    module = _load_module()
    if str(REPO_ROOT / 'scripts') not in sys.path:
        sys.path.insert(0, str(REPO_ROOT / 'scripts'))
    from mid360_robot_sample_bag import (  # noqa: PLC0415
        Mid360SampleBagWriter,
        SampleBagConfig,
    )
    from rosbags.highlevel import AnyReader  # noqa: PLC0415
    from rosbags.rosbag2 import Writer  # noqa: PLC0415
    from rosbags.typesys import Stores, get_typestore  # noqa: PLC0415

    source = tmp_path / 'source'
    reversed_bag = tmp_path / 'reversed'
    Mid360SampleBagWriter(
        SampleBagConfig(
            output_path=source,
            duration_sec=0.5,
            pointcloud_rate_hz=10.0,
            imu_rate_hz=100.0,
            force=True,
        )
    ).write()

    typestore = get_typestore(Stores.LATEST)
    previous_point_stamp = None
    point_index = 0
    with AnyReader(
        [source],
        default_typestore=typestore,
    ) as reader, Writer(reversed_bag, version=8) as writer:
        output_connections = {
            connection.topic: writer.add_connection(
                connection.topic,
                connection.msgtype,
                typestore=typestore,
            )
            for connection in reader.connections
        }
        for connection, receive_stamp_ns, serialized in reader.messages():
            if connection.msgtype == 'sensor_msgs/msg/PointCloud2':
                message = reader.deserialize(
                    serialized,
                    connection.msgtype,
                )
                header_stamp_ns = (
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec)
                )
                if point_index == 3:
                    header_stamp_ns = previous_point_stamp - 50_000_000
                    message.header.stamp.sec = (
                        header_stamp_ns // 1_000_000_000
                    )
                    message.header.stamp.nanosec = (
                        header_stamp_ns % 1_000_000_000
                    )
                    serialized = typestore.serialize_cdr(
                        message,
                        connection.msgtype,
                    )
                previous_point_stamp = header_stamp_ns
                point_index += 1
            writer.write(
                output_connections[connection.topic],
                receive_stamp_ns,
                serialized,
            )

    payload = module.build_preflight_payload(reversed_bag)
    timestamp_order = payload['summary']['timestamp_order']

    assert payload['summary']['pointcloud_inspection'][
        'rko_lio_compatible'
    ] is True
    assert timestamp_order['status'] == 'failed'
    assert timestamp_order['failed_topics'] == ['/livox/lidar']
    points = next(
        topic
        for topic in timestamp_order['topics']
        if topic['topic'] == '/livox/lidar'
    )
    assert points['reversal_count'] == 1
    assert points['max_backward_jump_ns'] == 50_000_000
    assert payload['recommended_profile_id'] is None
