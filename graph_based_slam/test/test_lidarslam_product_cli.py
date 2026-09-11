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

"""Tests for the repo-local golden-path product CLI."""

from __future__ import annotations

import json
from pathlib import Path
import struct
import subprocess


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI = REPO_ROOT / 'scripts' / 'lidarslam'
VERSION = (REPO_ROOT / 'VERSION').read_text(encoding='utf-8').strip()


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(CLI), *args],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def _write_bag_metadata(path: Path) -> None:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2, PointField

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
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(
        topic_metadata(0, '/points', 'sensor_msgs/msg/PointCloud2')
    )
    writer.create_topic(topic_metadata(1, '/imu', 'sensor_msgs/msg/Imu'))

    points = PointCloud2()
    points.header.frame_id = 'lidar'
    points.height = 1
    points.width = 1
    points.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='time', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    points.is_bigendian = False
    points.point_step = 16
    points.row_step = 16
    points.data = list(struct.pack('<ffff', 1.0, 2.0, 3.0, 0.0))
    points.is_dense = True
    writer.write('/points', serialize_message(points), 1_000_000_000)
    writer.write('/imu', serialize_message(Imu()), 1_000_000_001)
    if hasattr(writer, 'close'):
        writer.close()


def test_global_help_version_and_usage_contract():
    help_result = _run('--help')
    assert help_result.returncode == 0
    assert 'start <rosbag2_dir>' in help_result.stdout
    assert 'doctor [rosbag2_dir]' in help_result.stdout
    assert 'setup <rosbag2_dir>' in help_result.stdout
    assert 'run <rosbag2_dir>' in help_result.stdout
    assert 'inspect <output_dir>' in help_result.stdout
    assert 'view <output_dir>' in help_result.stdout
    assert 'edit <output_dir>' in help_result.stdout
    assert 'merge <map_outputs...>' in help_result.stdout

    version_result = _run('--version')
    assert version_result.returncode == 0
    assert version_result.stdout.strip() == f'lidarslam_ros2 {VERSION}'

    missing_result = _run()
    assert missing_result.returncode == 2
    assert 'Usage:' in missing_result.stderr

    unknown_result = _run('unknown')
    assert unknown_result.returncode == 2
    assert 'unknown command: unknown' in unknown_result.stderr


def test_subcommand_help_uses_product_names_and_option_groups():
    start = _run('start', '--help')
    doctor = _run('doctor', '--help')
    setup = _run('setup', '--help')
    run = _run('run', '--help')
    inspect = _run('inspect', '--help')
    view = _run('view', '--help')
    edit = _run('edit', '--help')
    merge = _run('merge', '--help')

    results = (start, doctor, setup, run, inspect, view, edit, merge)
    assert all(item.returncode == 0 for item in results)
    assert 'usage: lidarslam start' in start.stdout
    assert 'usage: lidarslam doctor' in doctor.stdout
    assert 'usage: lidarslam setup' in setup.stdout
    assert 'usage: lidarslam run' in run.stdout
    assert 'usage: lidarslam inspect' in inspect.stdout
    assert 'usage: lidarslam view' in view.stdout
    assert 'usage: lidarslam edit' in edit.stdout
    assert 'usage: lidarslam merge' in merge.stdout
    assert 'python3 scripts/' not in doctor.stdout
    assert 'python3 scripts/' not in start.stdout
    assert 'python3 scripts/' not in setup.stdout
    assert 'python3 scripts/' not in run.stdout
    assert 'python3 scripts/' not in inspect.stdout
    assert 'python3 scripts/' not in view.stdout
    assert 'python3 scripts/' not in edit.stdout
    assert 'python3 scripts/' not in merge.stdout
    assert 'map selection and output:' in run.stdout
    assert 'safety and lifecycle:' in run.stdout
    assert '--guided' in run.stdout
    assert '--yes' in start.stdout
    assert '--dry-run' in start.stdout
    assert '--viewer {none,browser,autoware,foxglove}' in start.stdout
    assert 'Open the completed map in this viewer' in start.stdout
    assert '--yes' in run.stdout
    assert '--help-all' in run.stdout
    assert '--public-json' in doctor.stdout
    assert 'deprecated viewer compatibility options:' not in run.stdout
    assert 'deprecated advanced viewer compatibility options:' not in run.stdout
    assert 'verification:' in run.stdout
    assert '--verification {required,off}' in run.stdout
    assert '--viewer {browser,autoware,foxglove}' in view.stdout
    assert '--runtime-dir' not in view.stdout
    assert '--plan <json>' in edit.stdout
    assert '--output-dir <dir>' in edit.stdout
    assert '--backend-input' not in edit.stdout
    assert '--merge-voxel-size <m>' in merge.stdout
    assert '--initial-transform <index:tx,ty,tz,yaw_deg>' in merge.stdout

    run_all = _run('run', '--help-all')
    view_all = _run('view', '--help-all')
    edit_all = _run('edit', '--help-all')
    assert run_all.returncode == view_all.returncode == edit_all.returncode == 0
    assert 'deprecated viewer compatibility options:' in run_all.stdout
    assert 'deprecated advanced viewer compatibility options:' in run_all.stdout
    assert '--no-verify-map' in run_all.stdout
    assert 'viewer runtime:' in view_all.stdout
    assert '--runtime-dir' in view_all.stdout
    assert '--backend-input <rosbag2_dir>' in edit_all.stdout
    assert '--params <yaml>' in edit_all.stdout


def test_help_all_rejects_ambiguous_combinations():
    result = _run('run', '--help-all', '--dry-run')

    assert result.returncode == 2
    assert '--help-all cannot be combined with other options' in result.stderr


def test_run_rejects_viewer_options_that_would_be_ignored(tmp_path: Path):
    missing_bag = tmp_path / 'missing'

    no_viewer = _run('run', str(missing_bag), '--viewer-rebuild')
    assert no_viewer.returncode == 2
    expected_error = (
        '--viewer-rebuild requires --viewer autoware or --viewer foxglove'
    )
    assert expected_error in no_viewer.stderr
    assert 'rosbag2 directory does not exist' not in no_viewer.stderr

    wrong_viewer = _run(
        'run',
        str(missing_bag),
        '--viewer',
        'foxglove',
        '--autoware-core-dir',
        '/tmp/autoware',
    )
    assert wrong_viewer.returncode == 2
    assert (
        '--autoware-core-dir requires --viewer autoware'
        in wrong_viewer.stderr
    )


def test_doctor_emits_machine_readable_preflight(tmp_path: Path):
    bag = tmp_path / 'sample_bag'
    _write_bag_metadata(bag)

    result = _run('doctor', str(bag), '--json')

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload['recommended_profile_id'] == 'rko_lio_graph_public_path'
    assert payload['summary']['capabilities']['has_pointcloud2'] is True
    assert payload['summary']['capabilities']['has_imu'] is True

    public_result = _run('doctor', str(bag), '--public-json')

    assert public_result.returncode == 0, public_result.stderr
    evidence = json.loads(public_result.stdout)
    assert evidence['status'] == 'ready'
    assert evidence['recommended_profile_id'] == 'rko_lio_graph_public_path'
    assert str(bag) not in public_result.stdout
    assert '/points' not in public_result.stdout
    assert '/imu' not in public_result.stdout
    assert 'run_autoware_map_beginner.sh' not in public_result.stdout

    human_result = _run('doctor', str(bag))

    assert human_result.returncode == 0, human_result.stderr
    assert 'lidarslam-map doctor — own-bag readiness' in human_result.stdout
    assert 'Status:   READY' in human_result.stdout
    assert 'Inputs:   PointCloud2, Imu' in human_result.stdout
    assert '/points' not in human_result.stdout
    assert '/imu' not in human_result.stdout
    assert 'Detected inputs:' not in human_result.stdout
    assert 'Do this now:' in human_result.stdout
    assert f'start {bag}' in human_result.stdout
    assert 'run_autoware_map_beginner.sh' not in human_result.stdout
    assert 'Other compatible paths:' not in human_result.stdout
    assert 'Need local details?' in human_result.stdout
    assert f'doctor {bag} --json' in human_result.stdout
    assert 'Need public support?' in human_result.stdout
    assert f'doctor {bag} --public-json' in human_result.stdout
    assert len(human_result.stdout.splitlines()) <= 26


def test_run_dry_run_and_inspect_delegate_to_proven_tools(tmp_path: Path):
    bag = tmp_path / 'sample_bag'
    output = tmp_path / 'map_output'
    _write_bag_metadata(bag)

    run_result = _run(
        'run',
        str(bag),
        '--output-dir',
        str(output),
        '--min-free-space-gib',
        '0.001',
        '--dry-run',
    )
    assert run_result.returncode == 0, run_result.stderr
    assert 'Selected profile:' in run_result.stdout
    assert str(output) in run_result.stdout
    assert not output.exists()

    output.mkdir()
    inspect_result = _run('inspect', str(output), '--json')
    assert inspect_result.returncode == 0, inspect_result.stderr
    diagnosis = json.loads(inspect_result.stdout)
    assert diagnosis['status'] == 'incomplete'
    assert diagnosis['run_dir'] == str(output)

    symptom_result = _run(
        'inspect',
        str(output),
        '--symptom',
        'map-is-not-visible',
        '--json',
    )
    assert symptom_result.returncode == 0, symptom_result.stderr
    symptom_diagnosis = json.loads(symptom_result.stdout)
    triage = symptom_diagnosis['symptom_triage']
    assert triage['basis'] == 'USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED'
    assert ' view ' in f" {triage['next_commands'][0]} "
    assert str(output) in triage['next_commands'][0]
    assert all(
        'python3 scripts/' not in command
        for command in triage['next_commands']
    )
    assert symptom_diagnosis['suggested_next_steps'] == (
        triage['next_commands']
    )


def test_child_input_error_is_propagated(tmp_path: Path):
    result = _run('doctor', str(tmp_path / 'missing'))

    assert result.returncode == 2
    assert 'rosbag2 directory does not exist' in result.stderr
