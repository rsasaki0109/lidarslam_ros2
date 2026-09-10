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

"""Tests for the MID-360 production-candidate session runner."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
RECORDING_SCRIPT = SCRIPT_DIR / 'check_mid360_robot_recording.sh'
SCRIPT_PATH = SCRIPT_DIR / 'run_mid360_robot_production_candidate_session.py'
WRAPPER_PATH = SCRIPT_DIR / 'run_mid360_robot_production_candidate_session.sh'


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding='utf-8')


def _write_profile(tmp_path: Path) -> Path:
    profile_path = tmp_path / 'profile.yaml'
    profile_path.write_text(
        yaml.safe_dump({
            'robot_name': 'production_robot',
            'base_frame': 'base_link',
            'lidar_frame': 'livox_frame',
            'imu_frame': 'livox_frame',
            'expected_pointcloud_topic': '/livox/lidar',
            'expected_imu_topic': '/livox/imu',
        }),
        encoding='utf-8',
    )
    return profile_path


def _fake_ros2_bin(tmp_path: Path) -> Path:
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    ros2 = fake_bin / 'ros2'
    ros2.write_text(
        f"""#!{sys.executable}
from pathlib import Path
import sys

args = sys.argv[1:]
if args[:2] == ['bag', 'record'] and '-o' in args:
    bag_path = Path(args[args.index('-o') + 1])
    bag_path.mkdir(parents=True, exist_ok=True)
    bag_path.joinpath('metadata.yaml').write_text('''rosbag2_bagfile_information:
  duration:
    nanoseconds: 1000000000
  message_count: 11
  topics_with_message_count:
  - topic_metadata:
      name: /livox/lidar
      type: sensor_msgs/msg/PointCloud2
      serialization_format: cdr
      offered_qos_profiles: ''
    message_count: 1
  - topic_metadata:
      name: /livox/imu
      type: sensor_msgs/msg/Imu
      serialization_format: cdr
      offered_qos_profiles: ''
    message_count: 10
''', encoding='utf-8')
    raise SystemExit(0)
raise SystemExit(0)
""",
        encoding='utf-8',
    )
    ros2.chmod(0o755)
    return fake_bin


def _steps_by_id(report: dict) -> dict[str, dict]:
    return {step['id']: step for step in report['steps']}


def _write_existing_candidate_artifacts(
    output_dir: Path,
    bag_path: Path,
    *,
    pointcloud_messages: int = 6000,
    imu_messages: int = 120000,
) -> None:
    _write_json(
        output_dir / 'jetson_mid360_host_readiness.json',
        {
            'status': 'PASS',
            'checks': [{'id': 'jetson_model', 'status': 'ok', 'message': 'Jetson'}],
        },
    )
    _write_json(
        output_dir / 'mid360_robot_recording_check.json',
        {
            'status': 'PASS',
            'bag_path': str(bag_path),
            'readiness_status': 'PASS',
            'checks': [{'id': 'readiness_status', 'status': 'ok', 'message': 'passed'}],
        },
    )
    _write_json(
        output_dir / 'mid360_robot_readiness.json',
        {
            'status': 'PASS',
            'bag_path': str(bag_path),
            'ready_for_mid360_launch': True,
            'selected_topics': {'pointcloud': '/livox/lidar', 'imu': '/livox/imu'},
            'frames': {
                'base_frame': 'base_link',
                'lidar_frame': 'livox_frame',
                'imu_frame': 'livox_frame',
            },
            'checks': [{'id': 'pointcloud2', 'status': 'ok', 'message': 'PointCloud2 topic'}],
            'bag_diagnostics': {
                'topics': {
                    'pointcloud': {
                        'metadata_message_count': pointcloud_messages,
                        'metadata_rate_hz': 10.0,
                        'sampled_message_count': 20,
                        'stable_frame_id': True,
                        'matches_expected_frame': True,
                    },
                    'imu': {
                        'metadata_message_count': imu_messages,
                        'metadata_rate_hz': 200.0,
                        'sampled_message_count': 20,
                        'stable_frame_id': True,
                        'matches_expected_frame': True,
                    },
                }
            },
        },
    )
    _write_json(
        output_dir / 'mid360_robot_run_plan.json',
        {
            'status': 'PASS',
            'dogfood_command_shell': 'bash scripts/run_rko_lio_graph_autoware_dogfood.sh',
        },
    )
    _write_json(
        output_dir / 'autoware_map_diagnosis.json',
        {
            'status': 'success',
            'verify': {'result': 'PASS'},
        },
    )
    _write_json(
        output_dir / 'public_rko_adoption_gate' / 'mid360_robot_public_rko_adoption_gate.json',
        {
            'status': 'PASS',
            'decision': {
                'matched_case': 'voxel_0p50_min_1p00_dd_on',
                'recommended_case': 'voxel_0p50_min_1p00_dd_on',
                'gate_pass_cases': 4,
            },
            'checks': [{'id': 'matched_config', 'status': 'PASS', 'message': 'matched'}],
        },
    )


def _write_segment_map_alignment_artifacts(root: Path) -> Path:
    json_path = root / 'mid360_robot_public_segment_map_cloud_alignment.json'
    _write_json(
        json_path,
        {
            'status': 'PASS',
            'clouds': {
                'start': {'analysis_points': 4525},
                'end': {'analysis_points': 7291},
            },
            'crop': {'crop_radius_m': 20.0},
            'aligned_overlap': {
                'symmetric_median_nn_m': 0.632,
                'symmetric_p90_nn_m': 2.107,
                'coverage_within_1m': 0.690,
            },
            'transform_start_to_end': {
                'translation_norm_m': 8.54,
                'yaw_deg': 73.12,
            },
            'artifacts': {
                'ply': str(root / 'mid360_robot_public_segment_map_cloud_alignment.ply'),
            },
            'checks': [
                {'id': 'median_overlap', 'status': 'PASS', 'message': 'median=0.632m'},
            ],
        },
    )
    (root / 'mid360_robot_public_segment_map_cloud_alignment.md').write_text(
        '# Alignment\n',
        encoding='utf-8',
    )
    (root / 'mid360_robot_public_segment_map_cloud_alignment.ply').write_text(
        'ply\n',
        encoding='utf-8',
    )
    return json_path


def test_production_candidate_dry_run_writes_session_plan(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    bag_root = tmp_path / 'bags'
    output_dir = tmp_path / 'out'
    public_sweep = tmp_path / 'public' / 'mid360_robot_public_rko_sweep.json'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            'candidate_01',
            '--duration-sec',
            '600',
            '--output-dir',
            str(output_dir),
            '--public-rko-sweep',
            str(public_sweep),
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)
    steps = _steps_by_id(report)

    assert report['status'] == 'PASS'
    assert report['dry_run'] is True
    assert report['counts']['ok'] == 1
    assert report['counts']['planned'] == 6
    assert steps['recording_plan']['status'] == 'ok'
    assert steps['host_readiness']['status'] == 'planned'
    assert steps['recording']['status'] == 'planned'
    assert steps['post_recording_check']['status'] == 'planned'
    assert steps['map']['status'] == 'planned'
    assert steps['public_rko_adoption_gate']['status'] == 'planned'
    assert steps['production_readiness']['status'] == 'planned'
    assert steps['recording']['command'][-1] == '--dry-run'
    assert str(public_sweep) in steps['public_rko_adoption_gate']['command']
    assert (bag_root / 'candidate_01_record_plan.json').is_file()
    assert (bag_root / 'candidate_01_profile.yaml').is_file()
    assert (output_dir / 'mid360_robot_production_candidate_session.json').is_file()
    assert (output_dir / 'mid360_robot_production_candidate_session.md').is_file()
    assert (output_dir / 'mid360_robot_session_dashboard.html').is_file()
    assert report['map_diagnosis_json_path'].endswith('autoware_map_diagnosis.json')
    assert report['production_readiness_json_path'].endswith(
        'mid360_robot_production_readiness.json',
    )
    assert report['dashboard_html_path'].endswith('mid360_robot_session_dashboard.html')
    assert report['artifact_paths']['segment_map_alignment_json'].endswith(
        'mid360_robot_public_segment_map_cloud_alignment.json',
    )


@pytest.mark.skipif(
    not RECORDING_SCRIPT.is_file(),
    reason='requires check_mid360_robot_recording.sh from a follow-up recording-cascade PR',
)
def test_production_candidate_record_only_run_records_fake_bag(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    fake_bin = _fake_ros2_bin(tmp_path)
    bag_root = tmp_path / 'bags'
    output_dir = tmp_path / 'out'
    env = os.environ.copy()
    env['PATH'] = f"{fake_bin}:{env['PATH']}"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--run',
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            'record_only_01',
            '--duration-sec',
            '1',
            '--output-dir',
            str(output_dir),
            '--skip-host-readiness',
            '--record-only',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
        env=env,
    )
    report = json.loads(result.stdout)
    steps = _steps_by_id(report)

    assert report['status'] == 'PASS'
    assert report['dry_run'] is False
    assert (bag_root / 'record_only_01' / 'metadata.yaml').is_file()
    assert steps['recording']['status'] == 'ok'
    assert steps['post_recording_check']['status'] == 'skipped'
    assert steps['map']['status'] == 'skipped'
    assert steps['public_rko_adoption_gate']['status'] == 'skipped'
    assert steps['production_readiness']['status'] == 'skipped'


def test_production_candidate_existing_artifacts_pass_production_gate(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    bag_root = tmp_path / 'robot_bags'
    output_dir = tmp_path / 'out'
    run_id = 'existing_pass_01'
    _write_existing_candidate_artifacts(output_dir, bag_root / run_id)
    alignment_json = _write_segment_map_alignment_artifacts(tmp_path / 'alignment')

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--run',
            '--from-existing-artifacts',
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            run_id,
            '--duration-sec',
            '600',
            '--output-dir',
            str(output_dir),
            '--segment-map-alignment',
            str(alignment_json),
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)
    steps = _steps_by_id(report)
    readiness = json.loads((output_dir / 'mid360_robot_production_readiness.json').read_text())
    dashboard = (output_dir / 'mid360_robot_session_dashboard.html').read_text()

    assert report['status'] == 'PASS'
    assert report['from_existing_artifacts'] is True
    assert steps['host_readiness']['status'] == 'ok'
    assert steps['recording']['status'] == 'ok'
    assert steps['post_recording_check']['status'] == 'ok'
    assert steps['map']['status'] == 'ok'
    assert steps['public_rko_adoption_gate']['status'] == 'ok'
    assert steps['production_readiness']['status'] == 'ok'
    assert readiness['status'] == 'PASS'
    assert readiness['production_ready'] is True
    assert readiness['evidence']['adoption_matched_case'] == 'voxel_0p50_min_1p00_dd_on'
    assert report['artifact_paths']['segment_map_alignment_json'] == str(alignment_json)
    assert report['artifact_paths']['segment_map_alignment_markdown'].endswith(
        'mid360_robot_public_segment_map_cloud_alignment.md',
    )
    assert report['artifact_paths']['segment_map_alignment_ply'].endswith(
        'mid360_robot_public_segment_map_cloud_alignment.ply',
    )
    assert 'MID-360 Robot Production Candidate' in dashboard
    assert 'Prod Gate' in dashboard
    assert 'Segment Map Cloud Alignment' in dashboard
    assert 'Aligned median NN' in dashboard
    assert 'median_overlap' in dashboard
    assert 'route-node ok' in dashboard
    assert 'voxel_0p50_min_1p00_dd_on' in json.dumps(readiness)


def test_production_candidate_existing_artifacts_fail_on_short_bag(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    bag_root = tmp_path / 'robot_bags'
    output_dir = tmp_path / 'out'
    run_id = 'existing_fail_01'
    _write_existing_candidate_artifacts(
        output_dir,
        bag_root / run_id,
        pointcloud_messages=3000,
        imu_messages=60000,
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            '--run',
            '--from-existing-artifacts',
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            run_id,
            '--duration-sec',
            '600',
            '--output-dir',
            str(output_dir),
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    report = json.loads(result.stdout)
    steps = _steps_by_id(report)
    readiness = json.loads((output_dir / 'mid360_robot_production_readiness.json').read_text())
    dashboard = (output_dir / 'mid360_robot_session_dashboard.html').read_text()

    assert result.returncode == 1
    assert report['status'] == 'FAIL'
    assert steps['production_readiness']['status'] == 'fail'
    assert readiness['status'] == 'FAIL'
    assert readiness['production_ready'] is False
    assert any(check['id'] == 'bag_duration' and check['status'] == 'FAIL'
               for check in readiness['checks'])
    assert any('longer' in action for action in readiness['next_actions'])
    assert 'Prod Gate' in dashboard
    assert 'bag_duration' in dashboard
    assert 'Record a longer stationary/walking production candidate bag' in dashboard
    assert 'route-node fail' in dashboard


def test_production_candidate_shell_wrapper_dry_run(tmp_path: Path):
    profile_path = _write_profile(tmp_path)
    bag_root = tmp_path / 'bags'
    output_dir = tmp_path / 'out'

    result = subprocess.run(
        [
            'bash',
            str(WRAPPER_PATH),
            '--robot-profile',
            str(profile_path),
            '--bag-root',
            str(bag_root),
            '--run-id',
            'wrapper_01',
            '--output-dir',
            str(output_dir),
            '--skip-public-gate',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    assert 'MID-360 Robot Production Candidate Session' in result.stdout
    report = json.loads(
        (output_dir / 'mid360_robot_production_candidate_session.json').read_text(),
    )
    steps = _steps_by_id(report)
    assert steps['public_rko_adoption_gate']['status'] == 'skipped'
    assert steps['production_readiness']['status'] == 'planned'
