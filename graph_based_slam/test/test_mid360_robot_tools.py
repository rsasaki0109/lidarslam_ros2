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

"""Unit tests for MID-360 robot planning helpers."""

from __future__ import annotations

import importlib
from pathlib import Path
import sys

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = REPO_ROOT / 'scripts'


def _load_module():
    if str(SCRIPTS_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPTS_DIR))
    return importlib.import_module('mid360_robot_tools')


class FakeAutowarePreflightAdapter:
    def __init__(self, payload):
        self.payload = payload

    def build_payload(self, bag_path: Path):
        self.payload['summary']['bag_path'] = str(bag_path)
        return self.payload


class FakeSampleReader:
    def __init__(self, samples):
        self.samples = samples

    def read_samples(self, bag_path: Path, topics: list[str], limit_per_topic: int):
        return {
            topic: self.samples.get(topic, [])[:limit_per_topic]
            for topic in topics
        }


def _autoware_payload(
    pointcloud=True,
    imu=True,
    tf=True,
    mid360=True,
    pointcloud_count=50,
    imu_count=500,
):
    topics = {
        'pointcloud2': (
            [{
                'name': '/livox/lidar',
                'msg_type': 'sensor_msgs/msg/PointCloud2',
                'message_count': pointcloud_count,
            }]
            if pointcloud else []
        ),
        'imu': (
            [{
                'name': '/livox/imu',
                'msg_type': 'sensor_msgs/msg/Imu',
                'message_count': imu_count,
            }]
            if imu else []
        ),
        'navsatfix': [],
        'velodyne_scan': [],
        'applanix_gsof49': [],
        'applanix_gsof50': [],
        'tf': (
            [{'name': '/tf_static', 'msg_type': 'tf2_msgs/msg/TFMessage', 'message_count': 1}]
            if tf else []
        ),
        'velocity_report': [],
    }
    return {
        'summary': {
            'bag_path': '',
            'duration_sec': 5.0,
            'message_count': (
                (pointcloud_count if pointcloud else 0)
                + (imu_count if imu else 0)
                + (1 if tf else 0)
            ),
            'topics': topics,
            'capabilities': {
                'has_pointcloud2': pointcloud,
                'has_imu': imu,
                'has_navsatfix': False,
                'has_velodyne_scan': False,
                'has_applanix_gsof49': False,
                'has_applanix_gsof50': False,
                'has_tf': tf,
                'has_velocity_report': False,
            },
        },
        'recommendations': (
            [{'id': 'rko_lio_graph_mid360_preset', 'label': 'MID360'}] if mid360 else []
        ),
    }


def test_preflight_builder_isolated_from_autoware_script(tmp_path: Path):
    module = _load_module()
    preflight = module.Mid360RobotPreflight(
        FakeAutowarePreflightAdapter(_autoware_payload())
    )

    payload = preflight.build_payload(
        tmp_path / 'bag',
        module.RobotFrames(
            base_frame='trunk',
            lidar_frame='mid360_link',
            imu_frame='mid360_imu',
        ),
    )

    assert payload['ready_for_mid360_launch'] is True
    assert payload['selected_topics'] == {
        'pointcloud': '/livox/lidar',
        'imu': '/livox/imu',
    }
    assert 'base_frame:=trunk' in payload['launch_command']
    assert any(
        check['id'] == 'mid360_preset' and check['status'] == 'ok'
        for check in payload['checks']
    )


def test_preflight_builder_adds_bag_diagnostics_from_samples(tmp_path: Path):
    module = _load_module()
    samples = {
        '/livox/lidar': [
            module.MessageSample(
                '/livox/lidar', 'sensor_msgs/msg/PointCloud2', 0, frame_id='mid360_link',
            ),
            module.MessageSample(
                '/livox/lidar',
                'sensor_msgs/msg/PointCloud2',
                100_000_000,
                frame_id='mid360_link',
            ),
            module.MessageSample(
                '/livox/lidar',
                'sensor_msgs/msg/PointCloud2',
                200_000_000,
                frame_id='mid360_link',
            ),
        ],
        '/livox/imu': [
            module.MessageSample('/livox/imu', 'sensor_msgs/msg/Imu', 0, frame_id='mid360_imu'),
            module.MessageSample(
                '/livox/imu',
                'sensor_msgs/msg/Imu',
                10_000_000,
                frame_id='mid360_imu',
            ),
        ],
        '/tf_static': [
            module.MessageSample(
                '/tf_static',
                'tf2_msgs/msg/TFMessage',
                0,
                tf_pairs=(
                    ('base_link', 'mid360_link'),
                    ('base_link', 'mid360_imu'),
                ),
            ),
        ],
    }
    diagnostics = module.Mid360BagDiagnosticsBuilder(
        sample_reader=FakeSampleReader(samples),
        sample_limit=10,
    )
    preflight = module.Mid360RobotPreflight(
        FakeAutowarePreflightAdapter(_autoware_payload()),
        diagnostics_builder=diagnostics,
    )

    payload = preflight.build_payload(
        tmp_path / 'bag',
        module.RobotFrames(
            base_frame='base_link',
            lidar_frame='mid360_link',
            imu_frame='mid360_imu',
        ),
    )

    bag_diagnostics = payload['bag_diagnostics']
    assert bag_diagnostics['sample_reader']['available'] is True
    assert bag_diagnostics['topics']['pointcloud']['metadata_rate_hz'] == 10.0
    assert bag_diagnostics['topics']['pointcloud']['sample_observed_rate_hz'] == 10.0
    assert bag_diagnostics['topics']['pointcloud']['sampled_frame_ids'] == ['mid360_link']
    assert bag_diagnostics['tf']['base_to_lidar_connected'] is True
    assert bag_diagnostics['tf']['base_to_imu_connected'] is True
    assert any(
        check['id'] == 'pointcloud_frame_id' and check['status'] == 'ok'
        for check in payload['checks']
    )
    assert any(
        check['id'] == 'tf_base_to_lidar_connected' and check['status'] == 'ok'
        for check in payload['checks']
    )


def test_preflight_builder_fails_sampled_lidar_frame_mismatch(tmp_path: Path):
    module = _load_module()
    samples = {
        '/livox/lidar': [
            module.MessageSample(
                '/livox/lidar',
                'sensor_msgs/msg/PointCloud2',
                0,
                frame_id='wrong_lidar',
            ),
        ],
        '/livox/imu': [
            module.MessageSample('/livox/imu', 'sensor_msgs/msg/Imu', 0, frame_id='livox_frame'),
        ],
    }
    diagnostics = module.Mid360BagDiagnosticsBuilder(
        sample_reader=FakeSampleReader(samples),
        sample_limit=10,
    )
    preflight = module.Mid360RobotPreflight(
        FakeAutowarePreflightAdapter(_autoware_payload(tf=False)),
        diagnostics_builder=diagnostics,
    )

    payload = preflight.build_payload(
        tmp_path / 'bag',
        module.RobotFrames(lidar_frame='livox_frame', imu_frame='livox_frame'),
    )

    assert payload['ready_for_mid360_launch'] is False
    assert any(
        check['id'] == 'pointcloud_frame_id' and check['status'] == 'fail'
        for check in payload['checks']
    )


def test_preflight_builder_fails_readable_empty_sampled_frame(
    tmp_path: Path,
):
    module = _load_module()
    samples = {
        '/livox/lidar': [
            module.MessageSample(
                '/livox/lidar',
                'sensor_msgs/msg/PointCloud2',
                0,
                frame_id='',
            ),
            module.MessageSample(
                '/livox/lidar',
                'sensor_msgs/msg/PointCloud2',
                100_000_000,
                frame_id='',
            ),
        ],
        '/livox/imu': [
            module.MessageSample(
                '/livox/imu',
                'sensor_msgs/msg/Imu',
                0,
                frame_id='livox_frame',
            ),
        ],
    }
    preflight = module.Mid360RobotPreflight(
        FakeAutowarePreflightAdapter(_autoware_payload()),
        diagnostics_builder=module.Mid360BagDiagnosticsBuilder(
            sample_reader=FakeSampleReader(samples),
            sample_limit=10,
        ),
    )

    payload = preflight.build_payload(
        tmp_path / 'bag',
        module.RobotFrames(lidar_frame='livox_frame'),
    )

    assert payload['ready_for_mid360_launch'] is False
    empty_frame_check = next(
        check for check in payload['checks']
        if check['id'] == 'pointcloud_frame_id'
    )
    assert empty_frame_check['status'] == 'fail'
    assert 'no non-empty frame_id' in empty_frame_check['message']
    assert 'repeat preflight' in empty_frame_check['message']


def test_preflight_builder_keeps_unavailable_sampling_advisory(
    tmp_path: Path,
):
    module = _load_module()
    preflight = module.Mid360RobotPreflight(
        FakeAutowarePreflightAdapter(_autoware_payload()),
        diagnostics_builder=module.Mid360BagDiagnosticsBuilder(
            sample_reader=FakeSampleReader({}),
            sample_limit=10,
        ),
    )

    payload = preflight.build_payload(
        tmp_path / 'bag',
        module.RobotFrames(),
    )

    assert payload['ready_for_mid360_launch'] is True
    assert not any(
        check['id'] == 'pointcloud_frame_id'
        for check in payload['checks']
    )


def test_preflight_builder_warns_on_low_metadata_rate(tmp_path: Path):
    module = _load_module()
    preflight = module.Mid360RobotPreflight(
        FakeAutowarePreflightAdapter(_autoware_payload(pointcloud_count=5))
    )

    payload = preflight.build_payload(tmp_path / 'bag', module.RobotFrames())

    assert payload['ready_for_mid360_launch'] is True
    assert any(
        check['id'] == 'pointcloud_metadata_rate' and check['status'] == 'warn'
        for check in payload['checks']
    )


def test_preflight_builder_reports_missing_imu_without_launch(tmp_path: Path):
    module = _load_module()
    preflight = module.Mid360RobotPreflight(
        FakeAutowarePreflightAdapter(_autoware_payload(imu=False))
    )

    payload = preflight.build_payload(tmp_path / 'bag', module.RobotFrames())

    assert payload['ready_for_mid360_launch'] is False
    assert payload['launch_command'] == ''
    assert any(
        check['id'] == 'imu' and check['status'] == 'fail'
        for check in payload['checks']
    )


def test_map_run_planner_builds_dogfood_and_foxglove_commands(tmp_path: Path):
    module = _load_module()
    repo_root = tmp_path / 'repo'
    planner = module.Mid360MapRunPlanner(repo_root)
    payload = {
        'ready_for_mid360_launch': True,
        'selected_topics': {'pointcloud': '/livox/lidar', 'imu': '/livox/imu'},
    }

    plan = planner.build_plan(
        bag_path=tmp_path / 'bag',
        payload=payload,
        frames=module.RobotFrames(base_frame='trunk'),
        options=module.MapRunOptions(
            output_dir=tmp_path / 'map_out',
            viewer='foxglove',
            work_dir='/tmp/viewer_ws',
        ),
    )

    assert '--lidar-topic' in plan.dogfood_command
    assert '/livox/lidar' in plan.dogfood_command
    assert '--base-frame' in plan.dogfood_command
    assert 'trunk' in plan.dogfood_command
    assert '--skip-viewer' in plan.dogfood_command
    assert plan.foxglove_command[-2:] == ['--work-dir', '/tmp/viewer_ws']


def test_diagnosis_planner_builds_write_command(tmp_path: Path):
    module = _load_module()
    repo_root = tmp_path / 'repo'
    output_dir = tmp_path / 'map_out'
    bag_path = tmp_path / 'bag'

    plan = module.Mid360RunDiagnosisPlanner(repo_root).build_plan(output_dir, bag_path)

    assert plan.command == [
        'python3',
        str(repo_root / 'scripts' / 'diagnose_autoware_map_run.py'),
        str(output_dir),
        '--bag',
        str(bag_path),
        '--write',
    ]
    assert plan.markdown_path == output_dir / 'autoware_map_diagnosis.md'
    assert plan.json_path == output_dir / 'autoware_map_diagnosis.json'
    assert plan.to_dict()['ran'] is False


def test_run_manifest_writer_persists_json_and_markdown(tmp_path: Path):
    module = _load_module()
    output_dir = tmp_path / 'map_out'
    payload = {
        'preflight': {
            'summary': {'bag_path': '/tmp/bag'},
            'selected_topics': {'pointcloud': '/livox/lidar', 'imu': '/livox/imu'},
            'frames': {
                'base_frame': 'base_link',
                'lidar_frame': 'livox_frame',
                'imu_frame': 'livox_frame',
            },
            'robot_profile': {'robot_name': 'test_robot'},
            'checks': [{'id': 'imu', 'status': 'ok', 'message': 'Imu topic: /livox/imu'}],
            'ready_for_mid360_launch': True,
        },
        'plan': {
            'output_dir': str(output_dir),
            'dogfood_command': ['bash', 'run.sh'],
            'dogfood_command_shell': 'bash run.sh',
            'foxglove_command': [],
            'foxglove_command_shell': '',
        },
        'diagnosis': {
            'command': ['python3', 'diagnose.py'],
            'command_shell': 'python3 diagnose.py',
            'markdown_path': str(output_dir / 'autoware_map_diagnosis.md'),
            'json_path': str(output_dir / 'autoware_map_diagnosis.json'),
            'ran': False,
        },
    }

    paths = module.Mid360RunManifestWriter().write(payload)

    assert paths['json'].is_file()
    assert paths['markdown'].is_file()
    assert 'test_robot' in paths['json'].read_text(encoding='utf-8')
    markdown = paths['markdown'].read_text(encoding='utf-8')
    assert 'bash run.sh' in markdown
    assert 'python3 diagnose.py' in markdown


def test_robot_profile_loader_reads_frames_topics_and_mount(tmp_path: Path):
    module = _load_module()
    profile_path = tmp_path / 'robot.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'go2_mid360',
            'base_frame': 'trunk',
            'lidar_frame': 'mid360_link',
            'imu_frame': 'mid360_imu',
            'expected_pointcloud_topic': '/points_mid360',
            'expected_imu_topic': '/imu_mid360',
            'mount': {
                'xyz': [0.1, 0.0, 0.2],
                'q_xyzw': [0.0, 0.0, 0.0, 1.0],
            },
        }),
        encoding='utf-8',
    )

    profile = module.RobotProfileLoader().load(profile_path)

    assert profile.robot_name == 'go2_mid360'
    assert profile.frames.base_frame == 'trunk'
    assert profile.frames.lidar_frame == 'mid360_link'
    assert profile.frames.imu_frame == 'mid360_imu'
    assert profile.expected_pointcloud_topic == '/points_mid360'
    assert profile.expected_imu_topic == '/imu_mid360'
    assert profile.mount['xyz'] == [0.1, 0.0, 0.2]
    assert profile.mount['q_xyzw'] == [0.0, 0.0, 0.0, 1.0]


def test_robot_profile_loader_rejects_invalid_mount_vector(tmp_path: Path):
    module = _load_module()
    profile_path = tmp_path / 'robot.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'bad_mount',
            'mount': {
                'q_xyzw': [0.0, 0.0, 1.0],
            },
        }),
        encoding='utf-8',
    )

    try:
        module.RobotProfileLoader().load(profile_path)
    except ValueError as exc:
        assert 'mount.q_xyzw' in str(exc)
    else:
        raise AssertionError('expected invalid profile to raise ValueError')


def test_profile_expected_topics_are_enforced(tmp_path: Path):
    module = _load_module()
    profile = module.RobotProfile(
        robot_name='strict_robot',
        frames=module.RobotFrames(),
        expected_pointcloud_topic='/expected/lidar',
        expected_imu_topic='/expected/imu',
    )
    preflight = module.Mid360RobotPreflight(
        FakeAutowarePreflightAdapter(_autoware_payload())
    )

    payload = preflight.build_payload(tmp_path / 'bag', profile.frames, profile=profile)

    assert payload['ready_for_mid360_launch'] is False
    assert payload['selected_topics'] == {'pointcloud': None, 'imu': None}
    assert any(
        check['id'] == 'expected_pointcloud_topic' and check['status'] == 'fail'
        for check in payload['checks']
    )


def test_profile_expected_topics_select_matching_topics(tmp_path: Path):
    module = _load_module()
    profile = module.RobotProfile(
        robot_name='default_robot',
        frames=module.RobotFrames(base_frame='trunk'),
        expected_pointcloud_topic='/livox/lidar',
        expected_imu_topic='/livox/imu',
    )
    preflight = module.Mid360RobotPreflight(
        FakeAutowarePreflightAdapter(_autoware_payload())
    )

    payload = preflight.build_payload(tmp_path / 'bag', profile.frames, profile=profile)

    assert payload['ready_for_mid360_launch'] is True
    assert payload['robot_profile']['robot_name'] == 'default_robot'
    assert payload['selected_topics'] == {
        'pointcloud': '/livox/lidar',
        'imu': '/livox/imu',
    }
