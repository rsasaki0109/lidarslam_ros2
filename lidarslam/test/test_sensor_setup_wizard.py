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
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Contract tests for the no-handwritten-YAML sensor setup wizard."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time

import jsonschema

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI = REPO_ROOT / 'scripts' / 'lidarslam'
PRODUCT_COMMANDS = ('./scripts/lidarslam', 'lidarslam-map')
SCHEMA = REPO_ROOT / 'docs' / 'schemas' / 'sensor-setup-v1.schema.json'
REJECTION_SCHEMA = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'sensor-setup-rejection-v1.schema.json'
)
RECOVERY_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'map-session-recovery-v1.schema.json'
)
SESSION_SCHEMA = (
    REPO_ROOT / 'docs' / 'schemas' / 'map-session-index-v1.schema.json'
)
SCRIPT = REPO_ROOT / 'scripts' / 'sensor_setup_wizard.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'sensor_setup_wizard',
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_bag(path: Path) -> None:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import Imu, PointCloud2, PointField

    def metadata(topic_id: int, name: str, msg_type: str):
        kwargs = {
            'name': name,
            'type': msg_type,
            'serialization_format': 'cdr',
        }
        try:
            return rosbag2_py.TopicMetadata(id=topic_id, **kwargs)
        except TypeError:
            return rosbag2_py.TopicMetadata(**kwargs)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(
        metadata(0, '/livox/lidar', 'sensor_msgs/msg/PointCloud2')
    )
    writer.create_topic(metadata(1, '/livox/imu', 'sensor_msgs/msg/Imu'))

    points = PointCloud2()
    points.header.frame_id = 'livox_frame'
    points.header.stamp.sec = 1
    points.height = 1
    points.width = 1
    points.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name='timestamp',
            offset=12,
            datatype=PointField.FLOAT64,
            count=1,
        ),
    ]
    points.point_step = 20
    points.row_step = 20
    points.data = list(struct.pack('<fffd', 1.0, 2.0, 3.0, 0.0))
    points.is_dense = True
    imu = Imu()
    imu.header.frame_id = 'imu_link'
    imu.header.stamp.sec = 1
    writer.write('/livox/lidar', serialize_message(points), 1_000_000_000)
    writer.write('/livox/imu', serialize_message(imu), 1_000_000_001)
    if hasattr(writer, 'close'):
        writer.close()


def _write_gnss_bag(path: Path) -> None:
    import rosbag2_py
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import NavSatFix, PointCloud2, PointField

    def metadata(topic_id: int, name: str, msg_type: str):
        kwargs = {
            'name': name,
            'type': msg_type,
            'serialization_format': 'cdr',
        }
        try:
            return rosbag2_py.TopicMetadata(id=topic_id, **kwargs)
        except TypeError:
            return rosbag2_py.TopicMetadata(**kwargs)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(
        metadata(0, '/points_raw', 'sensor_msgs/msg/PointCloud2')
    )
    writer.create_topic(metadata(1, '/fix', 'sensor_msgs/msg/NavSatFix'))

    points = PointCloud2()
    points.header.frame_id = 'velodyne'
    points.header.stamp.sec = 1
    points.height = 1
    points.width = 1
    points.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    points.point_step = 12
    points.row_step = 12
    points.data = list(struct.pack('<fff', 1.0, 2.0, 3.0))
    points.is_dense = True
    fix = NavSatFix()
    fix.header.frame_id = 'gnss'
    fix.header.stamp.sec = 1
    fix.latitude = 35.0
    fix.longitude = 139.0
    writer.write('/points_raw', serialize_message(points), 1_000_000_000)
    writer.write('/fix', serialize_message(fix), 1_000_000_001)
    if hasattr(writer, 'close'):
        writer.close()


def _write_empty_metadata_bag(path: Path) -> None:
    path.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 1_000_000_000},
            'message_count': 0,
            'topics_with_message_count': [],
        },
    }
    (path / 'metadata.yaml').write_text(
        yaml.safe_dump(metadata),
        encoding='utf-8',
    )


def _run(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(CLI), *args],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )


def _session_manifest(
    tmp_path: Path,
    *,
    map_output: Path | None = None,
    verification: str = 'required',
) -> dict:
    """Create the start-owned subset needed for recovery handoff tests."""
    bag = tmp_path / 'bag'
    bag.mkdir(exist_ok=True)
    setup = tmp_path / 'setup'
    setup.mkdir(exist_ok=True)
    output = map_output or tmp_path / 'map'
    command = [
        './scripts/lidarslam',
        'run',
        str(bag),
        '--profile',
        'rko_lio_graph_mid360_preset',
        '--output-dir',
        str(output),
        '--lidarslam-param',
        str(setup / 'params' / 'lidarslam.yaml'),
        '--rko-param',
        str(setup / 'params' / 'rko_lio.yaml'),
        '--verification',
        verification,
    ]
    return {
        'bundle_path': str(setup),
        'input': {'bag_path': str(bag)},
        'profile': {
            'id': 'rko_lio_graph_mid360_preset',
            'label': 'RKO-LIO graph + MID-360 preset',
        },
        'run': {
            'output_dir': str(output),
            'argv': command,
            'command_shell': ' '.join(command),
        },
    }


def _write_run_manifest(
    run_dir: Path,
    *,
    status: str = 'failed',
    stage: str = 'complete',
    workflow_exit_code: int = 17,
) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / 'run_manifest.json').write_text(
        json.dumps({
            'schema_version': 2,
            'status': status,
            'lifecycle': {'stage': stage},
            'execution': {
                'finished_at': '2026-08-12T00:00:00Z',
                'exit_code': workflow_exit_code,
            },
        }),
        encoding='utf-8',
    )


def _write_diagnosis(
    run_dir: Path,
    *,
    status: str,
    hints: list[str],
    verify_result: str = 'unknown',
) -> None:
    (run_dir / 'autoware_map_diagnosis.json').write_text(
        json.dumps({
            'status': status,
            'problem_hints': hints,
            'verify': {'result': verify_result},
        }),
        encoding='utf-8',
    )


def _write_validation_receipt(
    run_dir: Path,
    *,
    diagnosis_status: str = 'success',
    autoware_status: str = 'PASS',
) -> Path:
    """Write one schema-valid terminal receipt for session quality tests."""
    run_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = run_dir / 'run_manifest.json'
    diagnosis_path = run_dir / 'autoware_map_diagnosis.json'
    verify_path = run_dir / 'verify_autoware_map.log'
    manifest_path.write_text(
        json.dumps({
            'status': 'succeeded',
            'lifecycle': {'stage': 'complete', 'runner_exit_code': 0},
        }),
        encoding='utf-8',
    )
    diagnosis_path.write_text(
        json.dumps({'status': diagnosis_status}),
        encoding='utf-8',
    )
    if autoware_status != 'missing':
        verify_path.write_text(
            f'RESULT: {autoware_status}\n',
            encoding='utf-8',
        )
    else:
        verify_path.unlink(missing_ok=True)

    def digest(path: Path) -> str:
        return hashlib.sha256(path.read_bytes()).hexdigest()

    manifest_digest = digest(manifest_path)
    diagnosis_digest = digest(diagnosis_path)
    verify_digest = (
        digest(verify_path) if verify_path.is_file() else None
    )
    check_results = {
        'manifest_succeeded': True,
        'lifecycle_complete': True,
        'runner_exit_zero': True,
        'diagnosis_success': diagnosis_status == 'success',
        'autoware_verification_pass': autoware_status == 'PASS',
        'diagnosis_bound_to_manifest': True,
        'verify_log_bound_to_manifest': autoware_status == 'PASS',
    }
    receipt = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/first-map-validation-receipt-v1.schema.json'
        ),
        'status': (
            'PASS' if all(check_results.values()) else 'FAIL'
        ),
        'run': {
            'run_id': 'quality-session-fixture',
            'product_version': '0.9.0',
            'git_commit': 'a' * 40,
            'profile_id': 'rko_lio_graph_mid360_preset',
        },
        'verification': {
            'manifest_status': 'succeeded',
            'diagnosis_status': diagnosis_status,
            'autoware_status': autoware_status,
            'manifest_sha256': manifest_digest,
        },
        'evidence': {
            'manifest': {
                'filename': 'run_manifest.json',
                'sha256': manifest_digest,
            },
            'diagnosis': {
                'filename': 'autoware_map_diagnosis.json',
                'available': True,
                'sha256': diagnosis_digest,
            },
            'verify_log': {
                'filename': 'verify_autoware_map.log',
                'available': autoware_status != 'missing',
                'sha256': verify_digest,
            },
        },
        'checks': [
            {
                'id': check_id,
                'passed': passed,
                'observed': 'expected' if passed else 'not-performed',
            }
            for check_id, passed in check_results.items()
        ],
        'shareability': {
            'contains_map_geometry': False,
            'contains_private_paths': False,
            'contains_exact_command': False,
            'review_before_sharing': True,
        },
    }
    destination = run_dir / 'first_map_validation_receipt.json'
    destination.write_text(json.dumps(receipt), encoding='utf-8')
    return destination


def test_review_pass_detects_sensor_contract_without_writing(tmp_path: Path):
    """A real-looking MID360 bag must stop before calibration approval."""
    bag = tmp_path / 'mid360_bag'
    bundle = tmp_path / 'setup'
    _write_bag(bag)

    result = _run('setup', str(bag), '--output-dir', str(bundle), '--json')

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload['status'] == 'review_required'
    assert payload['profile']['id'] == 'rko_lio_graph_mid360_preset'
    assert payload['detected'] == {
        'lidar_topic': '/livox/lidar',
        'imu_topic': '/livox/imu',
        'point_timestamp_field': 'timestamp',
        'timestamp_order': 'passed',
    }
    assert '--accept-profile-extrinsics' in payload['next_command']
    assert not bundle.exists()


def test_start_requires_explicit_noninteractive_calibration_acceptance(
    tmp_path: Path,
):
    """One-command start must still fail closed on unreviewed extrinsics."""
    bag = tmp_path / 'mid360_bag'
    session = tmp_path / 'session'
    _write_bag(bag)

    result = subprocess.run(
        [str(CLI), 'start', str(bag), '--output-dir', str(session)],
        cwd=REPO_ROOT,
        check=False,
        capture_output=True,
        text=True,
        input='',
    )

    assert result.returncode == 2
    assert 'Sensor setup: REVIEW REQUIRED' in result.stdout
    assert 'If they match, generate the bundle with:' in result.stdout
    assert '--yes' in result.stdout
    assert 'start needs confirmation on a terminal' in result.stderr
    assert not session.exists()


def test_start_yes_dry_run_prints_complete_plan_without_writing(
    tmp_path: Path,
):
    """Automation can inspect the exact session without creating artifacts."""
    bag = tmp_path / 'mid360_bag'
    session = tmp_path / 'session'
    _write_bag(bag)

    result = _run(
        'start',
        str(bag),
        '--output-dir',
        str(session),
        '--yes',
        '--dry-run',
        '--editable',
        '--viewer',
        'none',
        '--json',
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload['status'] == 'dry_run'
    assert payload['bundle_path'] == str(session)
    assert payload['run']['output_dir'] == str(session / 'map')
    assert '--editable' in payload['run']['argv']
    assert payload['run']['argv'][-4:] == [
        '--min-free-space-gib',
        '5.0',
        '--verification',
        'required',
    ]
    assert not session.exists()


def test_start_pins_setup_then_delegates_map_and_viewer(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """The friendly path composes setup, run, and view once each."""
    module = _load_module()
    bag = tmp_path / 'mid360_bag'
    session = tmp_path / 'session'
    _write_bag(bag)
    monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map start')
    monkeypatch.setattr(
        module,
        '_runtime_readiness',
        lambda _profile: ([], []),
    )
    calls: list[list[str]] = []

    def fake_run(command, **_kwargs):
        calls.append(list(command))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)
    monkeypatch.setattr(module, '_run_delegated_session', fake_run)

    result = module.main([
        '--run',
        str(bag),
        '--output-dir',
        str(session),
        '--yes',
        '--editable',
        '--viewer',
        'browser',
    ])

    assert result == 0
    assert (session / 'sensor_setup.json').is_file()
    manifest = json.loads(
        (session / 'sensor_setup.json').read_text(encoding='utf-8')
    )
    assert manifest['status'] == 'ready'
    assert manifest['calibration']['source'] == 'accepted_profile_extrinsics'
    assert len(calls) == 2
    assert calls[0][:2] == ['./scripts/lidarslam', 'run']
    assert '--lidarslam-param' in calls[0]
    assert '--rko-param' in calls[0]
    assert '--editable' in calls[0]
    assert calls[1] == [
        './scripts/lidarslam',
        'view',
        str(session / 'map'),
        '--viewer',
        'browser',
        '--no-open',
    ]
    output = capsys.readouterr().out
    assert 'Sensor session: READY' not in output
    assert 'Map command:' not in output
    assert 'Review:' not in output
    assert 'Starting the verified map session' in output
    assert 'Map session: VERIFIED' in output
    session_index = json.loads(
        (session / 'session.json').read_text(encoding='utf-8')
    )
    session_schema = json.loads(SESSION_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(session_index, session_schema)
    assert session_index['status'] == 'verified'
    assert (session / 'session.html').is_file()


def test_start_interactive_confirmation_completes_one_command(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """A terminal user should not need to copy and submit a second command."""
    module = _load_module()
    bag = tmp_path / 'mid360_bag'
    session = tmp_path / 'session'
    _write_bag(bag)
    monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map start')
    monkeypatch.setattr(
        module,
        '_runtime_readiness',
        lambda _profile: ([], []),
    )
    monkeypatch.setattr(module.sys.stdin, 'isatty', lambda: True)
    monkeypatch.setattr('builtins.input', lambda _prompt: 'yes')
    calls: list[list[str]] = []

    def fake_run(command, **_kwargs):
        calls.append(list(command))
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)
    monkeypatch.setattr(module, '_run_delegated_session', fake_run)

    result = module.main([
        '--run',
        str(bag),
        '--output-dir',
        str(session),
        '--viewer',
        'none',
    ])

    assert result == 0
    assert (session / 'sensor_setup.json').is_file()
    assert len(calls) == 1
    assert calls[0][:2] == ['./scripts/lidarslam', 'run']
    output = capsys.readouterr().out
    assert 'Sensor setup: REVIEW REQUIRED' in output
    assert (
        'Review the values above, then answer the confirmation prompt below.'
        in output
    )
    assert 'If they match, generate the bundle with:' not in output
    assert '--yes' not in output
    assert 'Sensor session: READY' not in output
    assert 'Map command:' not in output
    assert 'Review:' not in output
    assert 'Map session: VERIFIED' in output


def test_start_runtime_failure_stops_before_mapping(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """Missing artifacts must be actionable before ROS is launched."""
    module = _load_module()
    bag = tmp_path / 'mid360_bag'
    session = tmp_path / 'session'
    _write_bag(bag)
    monkeypatch.setenv('LIDARSLAM_CLI_COMMAND', 'lidarslam-map start')
    monkeypatch.setattr(
        module,
        '_runtime_readiness',
        lambda _profile: (
            ['missing runtime artifact: rko_lio/lib/rko_lio/offline_node'],
            ['Build and source the workspace.'],
        ),
    )

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError('runtime-incomplete start launched a subprocess')

    monkeypatch.setattr(module.subprocess, 'run', fail_if_called)
    monkeypatch.setattr(module, '_run_delegated_session', fail_if_called)

    result = module.main([
        '--run',
        str(bag),
        '--output-dir',
        str(session),
        '--yes',
        '--viewer',
        'none',
    ])

    assert result == 2
    assert (session / 'sensor_setup.json').is_file()
    assert not (session / 'map').exists()
    error = capsys.readouterr().err
    assert 'local runtime is incomplete' in error
    assert 'rko_lio/offline_node' in error
    assert 'Build and source the workspace.' in error


def test_explicit_calibration_writes_ready_pinned_bundle(tmp_path: Path):
    """Measured transforms should produce a complete immutable bundle."""
    bag = tmp_path / 'mid360_bag'
    bundle = tmp_path / 'setup'
    map_output = tmp_path / 'map'
    _write_bag(bag)

    result = _run(
        'setup',
        str(bag),
        '--output-dir',
        str(bundle),
        '--map-output-dir',
        str(map_output),
        '--lidar-to-base',
        '0,0,0,1,0.1,0,0.2',
        '--imu-to-base',
        '0,0,0,1,0,0,0',
        '--json',
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    persisted = json.loads(
        (bundle / 'sensor_setup.json').read_text(encoding='utf-8')
    )
    schema = json.loads(SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(persisted, schema)
    assert payload == persisted
    assert payload['status'] == 'ready'
    assert payload['frames']['lidar'] == {
        'id': 'livox_frame',
        'source': 'bag_header',
    }
    assert payload['frames']['imu'] == {
        'id': 'imu_link',
        'source': 'bag_header',
    }
    assert payload['calibration']['source'] == 'explicit_cli_transforms'
    assert len(payload['parameters']) == 2
    assert all(len(item['sha256']) == 64 for item in payload['parameters'])
    assert '--lidarslam-param' in payload['run']['argv']
    assert '--lidar-frame' in payload['run']['argv']
    assert str(map_output) in payload['run']['argv']
    rko_params = yaml.safe_load(
        (bundle / 'params' / 'rko_lio.yaml').read_text(encoding='utf-8')
    )
    assert rko_params['extrinsic_lidar2base_quat_xyzw_xyz'] == [
        0.0, 0.0, 0.0, 1.0, 0.1, 0.0, 0.2,
    ]
    assert (bundle / 'README.md').is_file()


def test_setup_rejects_partial_or_non_unit_calibration(tmp_path: Path):
    """Partial and invalid transforms must fail before bag inspection."""
    partial = _run(
        'setup',
        str(tmp_path / 'missing'),
        '--lidar-to-base',
        '0,0,0,1,0,0,0',
    )
    non_unit = _run(
        'setup',
        str(tmp_path / 'missing'),
        '--lidar-to-base',
        '0,0,0,2,0,0,0',
        '--imu-to-base',
        '0,0,0,1,0,0,0',
    )

    assert partial.returncode == 2
    assert 'must be provided together' in partial.stderr
    assert non_unit.returncode == 2
    assert 'quaternion must be unit length' in non_unit.stderr
    assert 'Traceback' not in partial.stderr + non_unit.stderr


def test_start_not_ready_lists_stable_findings_and_copy_ready_actions(
    tmp_path: Path,
):
    """An unsupported bag should explain every detected input gap."""
    bag = tmp_path / 'empty_bag'
    session = tmp_path / 'session'
    _write_empty_metadata_bag(bag)

    result = _run(
        'start',
        str(bag),
        '--output-dir',
        str(session),
        '--yes',
        '--dry-run',
    )

    assert result.returncode == 2
    assert 'Sensor session: NOT READY' in result.stdout
    assert '[no-maintained-profile]' in result.stdout
    assert 'Detected inputs:' in result.stdout
    assert 'PointCloud2: not found' in result.stdout
    assert '[range-input-missing]' in result.stdout
    assert '[imu-input-missing]' in result.stdout
    assert '[navsatfix-input-missing]' in result.stdout
    assert '[applanix-gsof49-input-missing]' in result.stdout
    assert any(
        f'{command} doctor {bag}' in result.stdout
        for command in PRODUCT_COMMANDS
    )
    assert '<rosbag2_dir>' not in result.stdout
    assert 'No files were written' in result.stdout
    assert 'Traceback' not in result.stdout + result.stderr
    assert not session.exists()


def test_setup_not_ready_json_is_a_machine_readable_failure_contract(
    tmp_path: Path,
):
    """Automation should receive the same stable codes without text parsing."""
    bag = tmp_path / 'empty_bag'
    bundle = tmp_path / 'bundle'
    _write_empty_metadata_bag(bag)

    result = _run(
        'setup',
        str(bag),
        '--output-dir',
        str(bundle),
        '--json',
    )

    assert result.returncode == 2
    payload = json.loads(result.stdout)
    schema = json.loads(REJECTION_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(payload, schema)
    assert payload['status'] == 'not_ready'
    assert payload['schema_uri'].endswith(
        '/schemas/sensor-setup-rejection-v1.schema.json'
    )
    assert payload['reason']['code'] == 'no-maintained-profile'
    assert payload['files_written'] is False
    assert payload['detected']['topics'] == {
        'pointcloud2': [],
        'imu': [],
        'navsatfix': [],
        'velodyne_scan': [],
        'applanix_gsof49': [],
        'applanix_gsof50': [],
    }
    assert [item['code'] for item in payload['findings']] == [
        'range-input-missing',
        'imu-input-missing',
        'navsatfix-input-missing',
        'applanix-gsof49-input-missing',
    ]
    assert all(str(bag) in item['next_action'] for item in payload['findings'])
    assert payload['next_command'] in {
        f'{command} doctor {bag}' for command in PRODUCT_COMMANDS
    }
    assert result.stderr == ''
    assert not bundle.exists()


def test_forced_incompatible_profile_returns_safe_retry_without_writing(
    tmp_path: Path,
):
    """A forced profile mismatch should not collapse into an index error."""
    bag = tmp_path / 'gnss_bag'
    session = tmp_path / 'session'
    _write_gnss_bag(bag)

    result = _run(
        'start',
        str(bag),
        '--profile',
        'packet_applanix_smoke',
        '--output-dir',
        str(session),
        '--yes',
        '--dry-run',
        '--json',
    )

    assert result.returncode == 2
    payload = json.loads(result.stdout)
    schema = json.loads(REJECTION_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(payload, schema)
    assert payload['reason']['code'] == 'profile-incompatible'
    assert 'pointcloud_gnss_smoke' in payload['reason']['message']
    assert payload['findings'][0]['code'] == 'profile-incompatible'
    assert '--profile' not in payload['findings'][0]['next_action']
    assert '--yes --dry-run' in payload['findings'][0]['next_action']
    assert not session.exists()


def test_packet_applanix_start_dry_run_pins_consumed_topics(tmp_path: Path):
    """Packet users should receive the same one-command setup contract."""
    bag = tmp_path / 'packet_bag'
    bag.mkdir()
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 1_000_000_000},
            'message_count': 3,
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': '/velodyne_packets',
                        'type': 'velodyne_msgs/msg/VelodyneScan',
                        'serialization_format': 'cdr',
                    },
                    'message_count': 1,
                },
                {
                    'topic_metadata': {
                        'name': '/gsof49',
                        'type': (
                            'applanix_msgs/msg/NavigationSolutionGsof49'
                        ),
                        'serialization_format': 'cdr',
                    },
                    'message_count': 1,
                },
                {
                    'topic_metadata': {
                        'name': '/gsof50',
                        'type': (
                            'applanix_msgs/msg/'
                            'NavigationPerformanceGsof50'
                        ),
                        'serialization_format': 'cdr',
                    },
                    'message_count': 1,
                },
            ],
        },
    }
    (bag / 'metadata.yaml').write_text(
        yaml.safe_dump(metadata), encoding='utf-8'
    )

    session = tmp_path / 'session'
    result = _run(
        'start',
        str(bag),
        '--output-dir',
        str(session),
        '--yes',
        '--dry-run',
        '--json',
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    assert payload['status'] == 'dry_run'
    assert payload['profile']['id'] == 'packet_applanix_smoke'
    assert payload['topics'] == {
        'lidar': '/velodyne_packets',
        'lidar_type': 'velodyne_msgs/msg/VelodyneScan',
        'imu': None,
        'gnss': None,
        'navigation': '/gsof49',
        'navigation_quality': '/gsof50',
    }
    assert payload['frames']['base'] == {
        'id': None,
        'source': 'not_applicable',
    }
    assert payload['pointcloud']['inspection_status'] == 'not_applicable'
    assert payload['calibration']['source'] == 'not_applicable'
    assert payload['parameters'] == []
    assert payload['run']['argv'][0:2] in (
        ['./scripts/lidarslam', 'run'],
        ['lidarslam-map', 'run'],
    )
    assert not session.exists()


def test_pointcloud_gnss_setup_is_ready_without_rko_calibration(
    tmp_path: Path,
):
    """PointCloud2 + GNSS pins its topics and detected LiDAR frame."""
    bag = tmp_path / 'gnss_bag'
    bundle = tmp_path / 'setup'
    _write_gnss_bag(bag)

    result = _run(
        'setup',
        str(bag),
        '--output-dir',
        str(bundle),
        '--json',
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(result.stdout)
    schema = json.loads(SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(payload, schema)
    assert payload['status'] == 'ready'
    assert payload['profile']['id'] == 'pointcloud_gnss_smoke'
    assert payload['topics'] == {
        'lidar': '/points_raw',
        'lidar_type': 'sensor_msgs/msg/PointCloud2',
        'imu': None,
        'gnss': '/fix',
        'navigation': None,
        'navigation_quality': None,
    }
    assert payload['frames']['lidar'] == {
        'id': 'velodyne',
        'source': 'bag_header',
    }
    assert payload['frames']['imu'] == {
        'id': None,
        'source': 'not_applicable',
    }
    assert payload['calibration']['source'] == 'not_applicable'
    assert payload['parameters'] == []
    assert (bundle / 'README.md').is_file()


def test_setup_text_retains_complete_sensor_and_command_review(tmp_path: Path):
    """Setup-only output must keep the detail omitted from confirmed start."""
    bag = tmp_path / 'gnss_bag'
    bundle = tmp_path / 'setup'
    _write_gnss_bag(bag)

    result = _run(
        'setup',
        str(bag),
        '--output-dir',
        str(bundle),
    )

    assert result.returncode == 0, result.stderr
    assert 'Sensor session: READY' in result.stdout
    assert 'LiDAR: /points_raw [sensor_msgs/msg/PointCloud2]' in result.stdout
    assert 'GNSS: /fix' in result.stdout
    assert 'Map command:' in result.stdout
    assert f'Review: {bundle / "README.md"}' in result.stdout


def test_non_rko_profiles_reject_ignored_rko_controls(tmp_path: Path):
    """Profile-specific controls must never appear to work when unused."""
    bag = tmp_path / 'gnss_bag'
    _write_gnss_bag(bag)

    calibration = _run(
        'setup',
        str(bag),
        '--profile',
        'pointcloud_gnss_smoke',
        '--accept-profile-extrinsics',
    )
    editable = _run(
        'start',
        str(bag),
        '--profile',
        'pointcloud_gnss_smoke',
        '--editable',
        '--yes',
        '--dry-run',
    )

    assert calibration.returncode == 2
    assert 'does not consume these RKO-LIO-only options' in calibration.stderr
    assert '--accept-profile-extrinsics' in calibration.stderr
    assert editable.returncode == 2
    assert '--editable' in editable.stderr
    assert 'Traceback' not in calibration.stderr + editable.stderr


def test_ctrl_c_waits_for_terminal_evidence_and_renders_one_recovery(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """One Ctrl-C should finish runner cleanup before the recovery card."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    run_dir = Path(manifest['run']['output_dir'])

    class InterruptedProcess:
        pid = 4241
        wait_calls: list[float | None] = []
        signals: list[int] = []
        terminated = False
        killed = False

        def wait(self, timeout=None):
            self.wait_calls.append(timeout)
            if len(self.wait_calls) == 1:
                raise KeyboardInterrupt
            assert timeout == module.SESSION_INTERRUPT_GRACE_SECONDS
            _write_run_manifest(
                run_dir,
                status='interrupted',
                stage='complete',
                workflow_exit_code=130,
            )
            _write_diagnosis(
                run_dir,
                status='runtime_failed',
                hints=['Map workflow interrupted by SIGINT.'],
            )
            return 130

        def poll(self):
            return None

        def send_signal(self, signum):
            self.signals.append(signum)

        def terminate(self):
            self.terminated = True

        def kill(self):
            self.killed = True

    process = InterruptedProcess()
    monkeypatch.setattr(
        module.os,
        'killpg',
        lambda pid, signum: process.signals.append((pid, signum)),
    )

    def fake_popen(command, *, cwd, env, start_new_session):
        assert command == manifest['run']['argv']
        assert cwd == module.WORK_ROOT
        assert env[module.PRODUCT_SESSION_OUTPUT_ENV] == (
            module.PRODUCT_SESSION_OUTPUT_CONCISE
        )
        assert start_new_session is True
        return process

    monkeypatch.setattr(module.subprocess, 'Popen', fake_popen)

    result = module._run_session(
        type('Args', (), {'viewer': 'none'})(),
        manifest,
    )

    assert result == 130
    assert process.wait_calls == [
        None,
        module.SESSION_INTERRUPT_GRACE_SECONDS,
    ]
    assert process.signals == [(process.pid, module.signal.SIGINT)]
    assert process.terminated is False
    assert process.killed is False
    output = capsys.readouterr()
    assert 'Stop requested. Waiting up to 20 seconds' in output.err
    assert 'Traceback' not in output.err
    assert 'map session needs attention' not in output.err
    assert 'Map session: ACTION REQUIRED' in output.out
    assert '[workflow-interrupted]' in output.out
    assert output.out.count('\nNext:\n') == 1
    receipt = json.loads(
        (Path(manifest['bundle_path']) / 'map_session_recovery.json')
        .read_text(encoding='utf-8')
    )
    assert receipt['reason']['code'] == 'workflow-interrupted'
    session = json.loads(
        (Path(manifest['bundle_path']) / 'session.json').read_text(
            encoding='utf-8'
        )
    )
    assert session['status'] == 'action_required'
    assert session['runner_exit_code'] == 130


def test_ctrl_c_forwards_sigint_and_reaps_real_delegated_process(
    tmp_path: Path,
):
    """The product boundary should forward one real SIGINT and reap the child."""
    child_pid_path = tmp_path / 'delegated.pid'
    child_code = '\n'.join([
        'import os',
        'from pathlib import Path',
        'import signal',
        'import sys',
        'import time',
        'signal.signal(signal.SIGINT, lambda *_args: sys.exit(130))',
        f'Path({str(child_pid_path)!r}).write_text(str(os.getpid()))',
        'time.sleep(60)',
    ])
    probe_code = '\n'.join([
        'import importlib.util',
        'import json',
        'from pathlib import Path',
        'import sys',
        f'script = Path({str(SCRIPT)!r})',
        "spec = importlib.util.spec_from_file_location('setup_probe', script)",
        'module = importlib.util.module_from_spec(spec)',
        'spec.loader.exec_module(module)',
        (
            'completed = module._run_delegated_session('
            f'[sys.executable, "-c", {child_code!r}])'
        ),
        'print(json.dumps({"returncode": completed.returncode}))',
    ])
    probe = subprocess.Popen(
        [sys.executable, '-c', probe_code],
        cwd=REPO_ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    child_pid = None
    try:
        deadline = time.monotonic() + 5
        while (
            not child_pid_path.is_file()
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
        assert child_pid_path.is_file()
        child_pid = int(child_pid_path.read_text(encoding='utf-8'))

        os.kill(probe.pid, signal.SIGINT)
        stdout, stderr = probe.communicate(timeout=10)

        assert probe.returncode == 0, stderr
        assert json.loads(stdout) == {'returncode': 130}
        assert 'Stop requested. Waiting up to 20 seconds' in stderr
        assert 'Traceback' not in stderr
        try:
            os.kill(child_pid, 0)
        except ProcessLookupError:
            pass
        else:
            raise AssertionError('delegated child was not reaped')
    finally:
        if probe.poll() is None:
            probe.kill()
            probe.communicate()
        if child_pid is not None:
            try:
                os.kill(child_pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def test_ctrl_c_force_reaps_after_bounded_cleanup_windows(
    monkeypatch,
    capsys,
):
    """A stuck delegated runner should be reaped after both bounded waits."""
    module = _load_module()
    command = ['./scripts/lidarslam', 'run', '/tmp/input']

    class SlowProcess:
        pid = 4242
        wait_calls: list[float | None] = []
        signals: list[int] = []
        terminated = False
        killed = False

        def wait(self, timeout=None):
            self.wait_calls.append(timeout)
            if len(self.wait_calls) == 1:
                raise KeyboardInterrupt
            if len(self.wait_calls) in {2, 3}:
                raise subprocess.TimeoutExpired(command, timeout)
            assert timeout is None
            return -9

        def poll(self):
            return None

        def send_signal(self, signum):
            self.signals.append(signum)

        def terminate(self):
            self.terminated = True

        def kill(self):
            self.killed = True

    process = SlowProcess()
    monkeypatch.setattr(
        module.os,
        'killpg',
        lambda pid, signum: process.signals.append((pid, signum)),
    )

    def fake_popen(delegated, *, cwd, env, start_new_session):
        assert delegated == command
        assert cwd == module.WORK_ROOT
        assert env[module.PRODUCT_SESSION_OUTPUT_ENV] == (
            module.PRODUCT_SESSION_OUTPUT_CONCISE
        )
        assert start_new_session is True
        return process

    monkeypatch.setattr(module.subprocess, 'Popen', fake_popen)

    completed = module._run_delegated_session(command)

    assert completed.returncode == -9
    assert process.wait_calls == [
        None,
        module.SESSION_INTERRUPT_GRACE_SECONDS,
        module.SESSION_INTERRUPT_TERMINATE_SECONDS,
        None,
    ]
    assert process.signals == [
        (process.pid, module.signal.SIGINT),
        (process.pid, module.signal.SIGTERM),
        (process.pid, module.signal.SIGKILL),
    ]
    assert process.terminated is False
    assert process.killed is False
    error = capsys.readouterr().err
    assert '[runner-shutdown-grace-expired]' in error
    assert '[runner-shutdown-forced]' in error


def test_failed_start_writes_schema_valid_storage_recovery(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """Disk exhaustion should retain evidence and provide a safe retry."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    run_dir = Path(manifest['run']['output_dir'])
    _write_run_manifest(run_dir)
    _write_diagnosis(
        run_dir,
        status='runtime_failed',
        hints=[
            'The output filesystem ran out of writable space or quota. '
            'Preserve the run evidence, free storage, and rerun into a new '
            'output directory.'
        ],
    )
    (run_dir / 'map_save.log').write_text(
        '[Errno 28] No space left on device\n', encoding='utf-8'
    )
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 17),
    )
    monkeypatch.setattr(
        module,
        '_run_delegated_session',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 17),
    )

    result = module._run_session(
        type('Args', (), {'viewer': 'none'})(), manifest
    )

    assert result == 17
    receipt_path = Path(manifest['bundle_path']) / 'map_session_recovery.json'
    report_path = Path(manifest['bundle_path']) / 'session.html'
    session_path = Path(manifest['bundle_path']) / 'session.json'
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    schema = json.loads(RECOVERY_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(receipt, schema)
    assert receipt['reason']['code'] == 'storage-exhausted'
    assert receipt['next_command'].startswith('df -h ')
    assert receipt['run_dir'] == str(run_dir)
    assert receipt['evidence']['map_save_log'].endswith('/map_save.log')
    assert receipt['resume'] == {'available': False, 'command': None}
    assert receipt['retry']['available'] is True
    assert receipt['retry']['output_dir'] == f'{run_dir}.retry'
    assert '--output-dir' in receipt['retry']['command']
    assert report_path.is_file()
    session = json.loads(session_path.read_text(encoding='utf-8'))
    session_schema = json.loads(SESSION_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(session_schema)
    jsonschema.validate(session, session_schema)
    assert session['status'] == 'action_required'
    assert session['reason']['code'] == 'storage-exhausted'
    assert session['actions'][0]['command'].startswith('df -h ')
    assert session['actions'][-1]['kind'] == 'support'
    assert ' support ' in session['actions'][-1]['command']
    assert session['artifacts']['recovery_receipt'] == str(receipt_path)
    assert session['quality']['overall'] == 'action_required'
    assert session['quality']['checks'][0]['status'] == 'fail'
    assert session['quality']['checks'][3]['status'] == 'pass'
    report = report_path.read_text(encoding='utf-8')
    assert 'Mapping needs attention.' in report
    assert 'storage-exhausted' in report
    assert 'Copy command' in report
    assert 'Content-Security-Policy' in report
    assert '<script src=' not in report
    assert '<link ' not in report
    output = capsys.readouterr()
    assert '[storage-exhausted]' in output.err
    assert 'Map session: ACTION REQUIRED' in output.out
    card = output.out[output.out.index('Map session: ACTION REQUIRED'):]
    assert 'Reason: [storage-exhausted]' in card
    assert card.count('Next:') == 1
    assert receipt['next_command'] in card
    assert 'After correcting the cause' not in card
    assert receipt['retry']['command'] not in card
    assert receipt['inspect_command'] not in card
    assert f'Details: {report_path}' in card
    assert len(card.splitlines()) <= 8


def test_recovery_card_keeps_secondary_findings_in_details_only(tmp_path: Path):
    """The terminal shows stable extra codes without competing actions."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    recovery = module._session_recovery_payload(manifest, 2)
    recovery['findings'].append({
        'code': 'secondary-test-finding',
        'message': 'Secondary detail belongs in retained evidence.',
        'next_action': 'echo secondary-action',
    })
    receipt = tmp_path / 'map_session_recovery.json'
    session = tmp_path / 'session.json'

    card = module._render_session_recovery(
        recovery,
        receipt,
        session,
        None,
    )

    assert '[secondary-test-finding]' in card
    assert 'Secondary detail belongs' not in card
    assert 'echo secondary-action' not in card
    assert card.count('Next:') == 1
    assert recovery['next_command'] in card
    assert f'Details: {receipt}' in card

    fallback = module._render_session_recovery(
        recovery,
        None,
        None,
        None,
    )
    assert f'Details: {manifest["bundle_path"]}' in fallback


def test_failed_start_prioritizes_exact_postprocessing_resume(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """A terminal partial run should resume evidence work, not remap data."""
    module = _load_module()
    final_output = tmp_path / 'map'
    manifest = _session_manifest(tmp_path, map_output=final_output)
    partial = final_output.with_name(f'{final_output.name}.partial')
    _write_run_manifest(
        partial,
        status='succeeded',
        stage='diagnosing',
        workflow_exit_code=0,
    )
    _write_diagnosis(partial, status='incomplete', hints=[])
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 70),
    )
    monkeypatch.setattr(
        module,
        '_run_delegated_session',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 70),
    )

    result = module._run_session(
        type('Args', (), {'viewer': 'none'})(), manifest
    )

    assert result == 70
    receipt = json.loads(
        (Path(manifest['bundle_path']) / 'map_session_recovery.json')
        .read_text(encoding='utf-8')
    )
    assert receipt['reason']['code'] == 'postprocessing-incomplete'
    assert receipt['run_dir'] == str(partial)
    assert receipt['resume']['available'] is True
    assert receipt['resume']['command'].endswith(' --resume')
    assert receipt['next_command'] == receipt['resume']['command']
    assert receipt['retry'] == {
        'available': False,
        'command': None,
        'output_dir': None,
        'preserves_pinned_setup': True,
    }
    assert ' --resume' in capsys.readouterr().out


def test_failed_start_reports_map_verification_code_and_log(
    monkeypatch,
    tmp_path: Path,
):
    """Map-quality rejection should point directly to the verifier evidence."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    run_dir = Path(manifest['run']['output_dir'])
    _write_run_manifest(run_dir, workflow_exit_code=0)
    _write_diagnosis(
        run_dir,
        status='verify_failed',
        hints=['Autoware map verification failed.'],
        verify_result='FAIL',
    )
    verify_log = run_dir / 'verify_autoware_map.log'
    verify_log.write_text('RESULT: FAIL\n', encoding='utf-8')
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 1),
    )
    monkeypatch.setattr(
        module,
        '_run_delegated_session',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 1),
    )

    assert module._run_session(
        type('Args', (), {'viewer': 'none'})(), manifest
    ) == 1

    receipt = json.loads(
        (Path(manifest['bundle_path']) / 'map_session_recovery.json')
        .read_text(encoding='utf-8')
    )
    assert receipt['reason']['code'] == 'map-verification-failed'
    assert receipt['next_command'] == f'less {verify_log}'
    assert receipt['inspect_command'].endswith(
        f'{manifest["input"]["bag_path"]} --write'
    )


def test_runtime_recovery_taxonomy_covers_recognized_failure_classes(
    tmp_path: Path,
):
    """Select every published runtime reason from retained evidence."""
    module = _load_module()
    cases = [
        (
            'workflow-interrupted',
            'runtime_failed',
            [],
            'interrupted',
        ),
        (
            'ros-parameters-invalid',
            'runtime_failed',
            ['The run hit a ROS parameter-file parsing error.'],
            'failed',
        ),
        (
            'tf-messages-invalid',
            'runtime_failed',
            ['TF messages were malformed or incomplete.'],
            'failed',
        ),
        (
            'tf-tree-disconnected',
            'runtime_failed',
            ['TF tree connectivity was missing for the requested frames.'],
            'failed',
        ),
        (
            'map-save-failed',
            'runtime_failed',
            ['The /map_save service call failed or timed out.'],
            'failed',
        ),
        (
            'ros-node-died',
            'runtime_failed',
            [
                'A ROS node died during the run. Check the launch log tail '
                'for the crashing process.'
            ],
            'failed',
        ),
        (
            'gnss-constraints-missing',
            'runtime_failed',
            ['GNSS was enabled but the backend accepted zero GNSS edges.'],
            'failed',
        ),
        ('map-output-incomplete', 'incomplete', [], 'failed'),
        ('workflow-failed', 'runtime_failed', [], 'failed'),
    ]

    for expected, diagnosis_status, hints, manifest_status in cases:
        case_root = tmp_path / expected
        case_root.mkdir()
        manifest = _session_manifest(case_root)
        run_dir = Path(manifest['run']['output_dir'])
        _write_run_manifest(run_dir, status=manifest_status)
        _write_diagnosis(
            run_dir,
            status=diagnosis_status,
            hints=hints,
        )

        recovery = module._session_recovery_payload(manifest, 17)

        assert recovery['reason']['code'] == expected


def test_recovery_refuses_unsafe_or_ambiguous_resume_states(tmp_path: Path):
    """Offer resume only when one terminal manifest-v2 proves it."""
    module = _load_module()

    ambiguous_root = tmp_path / 'ambiguous'
    ambiguous_root.mkdir()
    ambiguous = _session_manifest(ambiguous_root)
    final_dir = Path(ambiguous['run']['output_dir'])
    partial_dir = final_dir.with_name(f'{final_dir.name}.partial')
    _write_run_manifest(final_dir)
    _write_run_manifest(partial_dir, stage='diagnosing')
    recovery = module._session_recovery_payload(ambiguous, 70)
    assert recovery['reason']['code'] == 'ambiguous-output-state'
    assert recovery['resume'] == {'available': False, 'command': None}

    unreadable_root = tmp_path / 'unreadable'
    unreadable_root.mkdir()
    unreadable = _session_manifest(unreadable_root)
    unreadable_dir = Path(unreadable['run']['output_dir'])
    unreadable_dir.mkdir()
    (unreadable_dir / 'run_manifest.json').write_text(
        '{not json', encoding='utf-8'
    )
    _write_diagnosis(unreadable_dir, status='runtime_failed', hints=[])
    recovery = module._session_recovery_payload(unreadable, 70)
    assert recovery['reason']['code'] == 'run-manifest-unreadable'
    assert recovery['resume'] == {'available': False, 'command': None}

    running_root = tmp_path / 'running'
    running_root.mkdir()
    running = _session_manifest(running_root)
    running_dir = Path(running['run']['output_dir'])
    _write_run_manifest(
        running_dir,
        status='running',
        stage='workflow_running',
    )
    _write_diagnosis(running_dir, status='runtime_failed', hints=[])
    recovery = module._session_recovery_payload(running, 70)
    assert recovery['reason']['code'] == 'workflow-state-uncertain'
    assert recovery['resume'] == {'available': False, 'command': None}


def test_recovery_browser_report_escapes_untrusted_text(tmp_path: Path):
    """Bag paths, diagnostics, and commands must not inject active HTML."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    recovery = module._session_recovery_payload(manifest, 2)
    recovery['reason']['message'] = '<script>alert("reason")</script>'
    recovery['findings'][0]['message'] = '<img src=x onerror=alert(1)>'
    recovery['findings'][0]['next_action'] = 'echo "<unsafe>"'
    recovery['next_command'] = 'echo "<unsafe>"'

    recovery_path = module._write_session_recovery(
        Path(manifest['bundle_path']),
        recovery,
    )
    session = module._session_index_payload(
        type('Args', (), {'verification': 'required'})(),
        manifest,
        runner_exit_code=2,
        recovery=recovery,
        recovery_path=recovery_path,
    )
    module._write_session_index(Path(manifest['bundle_path']), session)

    report = (
        Path(manifest['bundle_path']) / 'session.html'
    ).read_text(encoding='utf-8')
    assert '<script>alert("reason")</script>' not in report
    assert '<img src=x onerror=alert(1)>' not in report
    assert '&lt;script&gt;alert(&quot;reason&quot;)&lt;/script&gt;' in report
    assert '&lt;img src=x onerror=alert(1)&gt;' in report
    assert 'echo &quot;&lt;unsafe&gt;&quot;' in report
    assert "default-src 'none'" in report
    assert 'navigator.clipboard' in report


def test_session_json_survives_optional_html_write_failure(
    monkeypatch,
    tmp_path: Path,
):
    """A derived browser report must never suppress the recovery contract."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    recovery = module._session_recovery_payload(manifest, 2)
    original = module._atomic_write_text

    def fail_html(destination: Path, content: str) -> None:
        if destination.suffix == '.html':
            raise OSError('injected HTML write failure')
        original(destination, content)

    monkeypatch.setattr(module, '_atomic_write_text', fail_html)

    receipt = module._write_session_recovery(
        Path(manifest['bundle_path']),
        recovery,
    )
    session = module._session_index_payload(
        type('Args', (), {'verification': 'required'})(),
        manifest,
        runner_exit_code=2,
        recovery=recovery,
        recovery_path=receipt,
    )
    session_index, report = module._write_session_index(
        Path(manifest['bundle_path']),
        session,
    )

    assert receipt.is_file()
    assert session_index.is_file()
    assert report is None
    assert not (Path(manifest['bundle_path']) / 'session.html').exists()
    schema = json.loads(RECOVERY_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(
        json.loads(receipt.read_text(encoding='utf-8')),
        schema,
    )
    jsonschema.validate(
        json.loads(session_index.read_text(encoding='utf-8')),
        json.loads(SESSION_SCHEMA.read_text(encoding='utf-8')),
    )


def test_atomic_session_writer_does_not_follow_predictable_temp_symlink(
    tmp_path: Path,
):
    """A stale legacy temp name must never redirect a session write."""
    module = _load_module()
    protected = tmp_path / 'protected.txt'
    protected.write_text('keep me', encoding='utf-8')
    legacy_temp = tmp_path / '.session.json.tmp'
    legacy_temp.symlink_to(protected)
    destination = tmp_path / 'session.json'

    module._atomic_write_text(destination, '{"status":"running"}\n')

    assert protected.read_text(encoding='utf-8') == 'keep me'
    assert destination.read_text(encoding='utf-8') == '{"status":"running"}\n'
    assert legacy_temp.is_symlink()


def test_failed_start_opens_recovery_report_for_browser_viewer(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """The default desktop path should open the explanation, not a bad map."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    run_dir = Path(manifest['run']['output_dir'])
    _write_run_manifest(run_dir)
    _write_diagnosis(run_dir, status='runtime_failed', hints=[])
    opened: list[str] = []

    class Browser:
        @staticmethod
        def desktop_session_available() -> bool:
            return True

        @staticmethod
        def open_browser(uri: str) -> bool:
            opened.append(uri)
            return True

    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 17),
    )
    monkeypatch.setattr(
        module,
        '_run_delegated_session',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 17),
    )
    monkeypatch.setattr(
        module,
        '_load_script_module',
        lambda name, _module_name: Browser
        if name == 'view_autoware_map.py'
        else None,
    )

    result = module._run_session(
        type('Args', (), {'viewer': 'browser'})(), manifest
    )

    assert result == 17
    assert len(opened) == 1
    assert opened[0].startswith('file://')
    assert opened[0].endswith('/session.html')
    assert 'Session browser opened.' in capsys.readouterr().out


def test_recovery_browser_open_failure_preserves_original_handoff(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """Desktop integration errors must remain non-authoritative warnings."""
    module = _load_module()
    report = tmp_path / 'session.html'
    report.write_text('<!doctype html>', encoding='utf-8')

    class BrokenBrowser:
        @staticmethod
        def desktop_session_available() -> bool:
            return True

        @staticmethod
        def open_browser(_uri: str) -> bool:
            raise RuntimeError('injected browser failure')

    monkeypatch.setattr(
        module,
        '_load_script_module',
        lambda *_args: BrokenBrowser,
    )

    module._maybe_open_session_report(
        type('Args', (), {'viewer': 'browser'})(),
        report,
    )

    error = capsys.readouterr().err
    assert 'could not open the session browser' in error
    assert 'injected browser failure' in error


def test_failed_start_without_output_has_stable_dry_run_recovery(
    monkeypatch,
    tmp_path: Path,
):
    """Runner initialization failures still need one copy-ready next action."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 2),
    )
    monkeypatch.setattr(
        module,
        '_run_delegated_session',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 2),
    )

    assert module._run_session(
        type('Args', (), {'viewer': 'none'})(), manifest
    ) == 2

    receipt = json.loads(
        (Path(manifest['bundle_path']) / 'map_session_recovery.json')
        .read_text(encoding='utf-8')
    )
    assert receipt['reason']['code'] == 'runner-start-failed'
    assert receipt['run_dir'] is None
    assert receipt['inspect_command'] is None
    assert receipt['next_command'].endswith(' --dry-run')
    assert receipt['files_preserved'] is True


def test_completed_map_with_failed_viewer_keeps_map_success_clear(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """A browser failure must never be presented as a failed map."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    results = iter([
        subprocess.CompletedProcess([], 0),
        subprocess.CompletedProcess([], 9),
    ])
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda *_args, **_kwargs: next(results),
    )
    monkeypatch.setattr(
        module,
        '_run_delegated_session',
        lambda *_args, **_kwargs: next(results),
    )

    assert module._run_session(
        type('Args', (), {'viewer': 'browser'})(), manifest
    ) == 9

    output = capsys.readouterr()
    assert 'Map session: VERIFIED' in output.out
    assert 'Reopen later:' not in output.out
    reopen_command = (
        f'./scripts/lidarslam view {manifest["run"]["output_dir"]}'
    )
    assert output.out.count(reopen_command) == 1
    assert '[viewer-failed]' in output.err
    assert not (
        Path(manifest['bundle_path']) / 'map_session_recovery.json'
    ).exists()
    session = json.loads(
        (Path(manifest['bundle_path']) / 'session.json').read_text(
            encoding='utf-8'
        )
    )
    assert session['status'] == 'verified'
    assert session['artifacts']['map_preview_html'] is None


def test_success_terminal_summary_keeps_paths_and_next_command_together(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """Headless completion should expose every copy-ready handoff in one place."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    map_output = Path(manifest['run']['output_dir'])
    _write_validation_receipt(map_output)
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 0),
    )
    monkeypatch.setattr(
        module,
        '_run_delegated_session',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 0),
    )

    result = module._run_session(
        type('Args', (), {'viewer': 'none', 'verification': 'required'})(),
        manifest,
    )

    assert result == 0
    output = capsys.readouterr().out
    assert 'Map session: VERIFIED' in output
    assert f'Map output:        {map_output}' in output
    assert 'Verification:      PASS' in output
    assert 'Viewer:            not opened (--viewer none)' in output
    assert (
        f'Session index:     {Path(manifest["bundle_path"]) / "session.json"}'
        in output
    )
    assert (
        f'Session page:      {Path(manifest["bundle_path"]) / "session.html"}'
        in output
    )
    assert f'Run manifest:      {map_output / "run_manifest.json"}' in output
    assert (
        'First-map receipt:  '
        f'{map_output / "first_map_validation_receipt.json"}'
    ) in output
    assert (
        'Next:              '
        f'./scripts/lidarslam view {map_output}'
    ) in output
    assert (
        'Report:            '
        f'./scripts/lidarslam report {Path(manifest["bundle_path"])}'
    ) in output


def test_completed_map_keeps_one_fallback_when_session_index_fails(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """Derived-index failure must not hide success or duplicate the handoff."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    map_output = Path(manifest['run']['output_dir'])
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 0),
    )
    monkeypatch.setattr(
        module,
        '_run_delegated_session',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 0),
    )

    def fail_index(*_args, **_kwargs):
        raise OSError('injected session index failure')

    monkeypatch.setattr(module, '_write_session_index', fail_index)

    result = module._run_session(
        type('Args', (), {'viewer': 'none', 'verification': 'required'})(),
        manifest,
    )

    assert result == 0
    output = capsys.readouterr()
    assert 'Map session: COMPLETED' in output.out
    assert f'Map output:        {map_output}' in output.out
    assert output.out.count(f'./scripts/lidarslam view {map_output}') == 1
    assert '[session-index-write-failed]' in output.err


def test_unverified_start_is_honest_and_offers_fresh_verified_output(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """Verification-off success must never be presented as a verified map."""
    module = _load_module()
    manifest = _session_manifest(tmp_path, verification='off')
    monkeypatch.setattr(
        module.subprocess,
        'run',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 0),
    )
    monkeypatch.setattr(
        module,
        '_run_delegated_session',
        lambda *_args, **_kwargs: subprocess.CompletedProcess([], 0),
    )

    result = module._run_session(
        type('Args', (), {'viewer': 'none', 'verification': 'off'})(),
        manifest,
    )

    assert result == 0
    output = capsys.readouterr().out
    assert 'Starting a diagnostic map session without verification' in output
    assert 'Map session: UNVERIFIED' in output
    assert 'verification was skipped; do not rely on this map as verified' in output
    assert 'Map session: VERIFIED' not in output
    session_path = Path(manifest['bundle_path']) / 'session.json'
    session = json.loads(session_path.read_text(encoding='utf-8'))
    schema = json.loads(SESSION_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(session, schema)
    assert session['status'] == 'unverified'
    assert session['verification'] == {'mode': 'off', 'result': 'not_run'}
    assert session['actions'][0]['kind'] == 'verify'
    assert '.verified' in session['actions'][0]['command']
    assert '--verification required' in session['actions'][0]['command']
    assert str(manifest['run']['output_dir']) in session['map_output']
    report = (Path(manifest['bundle_path']) / 'session.html').read_text(
        encoding='utf-8'
    )
    assert 'Map created without verification.' in report
    assert 'Create a fresh verified map' in report


def test_verified_session_renders_receipt_bound_quality_summary(
    tmp_path: Path,
):
    """A passing receipt should become four understandable PASS checks."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    map_output = Path(manifest['run']['output_dir'])
    (map_output / 'pointcloud_map').mkdir(parents=True)
    _write_validation_receipt(map_output)

    session = module._session_index_payload(
        type('Args', (), {'verification': 'required'})(),
        manifest,
        runner_exit_code=0,
    )

    schema = json.loads(SESSION_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(session, schema)
    assert session['quality']['overall'] == 'pass'
    assert session['quality']['source']['status'] == 'valid'
    assert [item['id'] for item in session['quality']['checks']] == [
        'workflow',
        'map_output',
        'verification',
        'evidence',
    ]
    assert all(
        item['status'] == 'pass'
        for item in session['quality']['checks']
    )
    assert [item['kind'] for item in session['actions']] == [
        'view',
        'share',
        'inspect',
    ]
    assert session['actions'][1]['label'] == 'Prepare a first-map report'
    assert session['actions'][1]['command'] == (
        f'./scripts/lidarslam report {tmp_path / "setup"}'
    )
    report = module._render_session_html(session)
    assert 'Map quality' in report
    assert 'All required map quality evidence passed.' in report
    assert 'autoware_verification_pass' in report
    assert 'Prepare a first-map report' in report


def test_unverified_quality_distinguishes_not_run_from_failure(
    tmp_path: Path,
):
    """Diagnostic mode should not turn an intentionally skipped check red."""
    module = _load_module()
    manifest = _session_manifest(tmp_path, verification='off')
    map_output = Path(manifest['run']['output_dir'])
    (map_output / 'pointcloud_map').mkdir(parents=True)
    _write_validation_receipt(
        map_output,
        diagnosis_status='map_saved',
        autoware_status='missing',
    )

    session = module._session_index_payload(
        type('Args', (), {'verification': 'off'})(),
        manifest,
        runner_exit_code=0,
    )

    schema = json.loads(SESSION_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(session, schema)
    quality = session['quality']
    assert quality['overall'] == 'not_verified'
    assert quality['source']['status'] == 'valid'
    assert [item['status'] for item in quality['checks']] == [
        'pass',
        'pass',
        'not_run',
        'pass',
    ]
    report = module._render_session_html(session)
    assert 'NOT VERIFIED' in report
    assert 'Disabled for this diagnostic run.' in report


def test_semantically_incomplete_quality_receipt_is_not_trusted(
    tmp_path: Path,
):
    """Schema-valid but unrelated checks must not produce a quality PASS."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    map_output = Path(manifest['run']['output_dir'])
    receipt_path = _write_validation_receipt(map_output)
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    receipt['status'] = 'FAIL'
    receipt['checks'] = [
        {
            'id': f'unrelated_{index}',
            'passed': True,
            'observed': '<img src=x onerror=alert(1)>',
        }
        for index in range(7)
    ]
    receipt_path.write_text(json.dumps(receipt), encoding='utf-8')

    session = module._session_index_payload(
        type('Args', (), {'verification': 'required'})(),
        manifest,
        runner_exit_code=0,
    )

    assert session['quality']['overall'] == 'unavailable'
    assert session['quality']['source']['status'] == 'invalid'
    report = module._render_session_html(session)
    assert '<img src=x onerror=alert(1)>' not in report
    assert 'The validation receipt is unavailable.' in report


def test_quality_receipt_with_unrecognized_check_is_not_trusted(
    tmp_path: Path,
):
    """Extra checks must not broaden the trusted validation vocabulary."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    map_output = Path(manifest['run']['output_dir'])
    receipt_path = _write_validation_receipt(map_output)
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    receipt['checks'].append({
        'id': 'unexpected_quality_override',
        'passed': True,
        'observed': 'This check is outside the product contract.',
    })
    receipt_path.write_text(json.dumps(receipt), encoding='utf-8')

    session = module._session_index_payload(
        type('Args', (), {'verification': 'required'})(),
        manifest,
        runner_exit_code=0,
    )

    assert session['quality']['overall'] == 'unavailable'
    assert session['quality']['source']['status'] == 'invalid'


def test_stale_quality_receipt_is_rejected_after_evidence_changes(
    tmp_path: Path,
):
    """A copied or stale receipt must not validate changed run evidence."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    map_output = Path(manifest['run']['output_dir'])
    _write_validation_receipt(map_output)
    (map_output / 'autoware_map_diagnosis.json').write_text(
        json.dumps({'status': 'runtime_failed'}),
        encoding='utf-8',
    )

    session = module._session_index_payload(
        type('Args', (), {'verification': 'required'})(),
        manifest,
        runner_exit_code=0,
    )

    assert session['quality']['overall'] == 'unavailable'
    assert session['quality']['source']['status'] == 'invalid'


def test_success_session_links_generated_3d_preview(
    monkeypatch,
    tmp_path: Path,
):
    """The default browser opens one landing page with the 3D review link."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    map_output = Path(manifest['run']['output_dir'])
    calls: list[list[str]] = []
    opened: list[str] = []

    def fake_run(command, **_kwargs):
        calls.append(list(command))
        if len(calls) == 1:
            assert opened == [
                (Path(manifest['bundle_path']) / 'session.html').as_uri()
            ]
            live = json.loads(
                (Path(manifest['bundle_path']) / 'session.json').read_text(
                    encoding='utf-8'
                )
            )
            assert live['status'] == 'running'
            assert live['progress']['stage'] == 'preparing'
        if len(calls) == 2:
            preview_progress = json.loads(
                (Path(manifest['bundle_path']) / 'session.json').read_text(
                    encoding='utf-8'
                )
            )
            assert preview_progress['status'] == 'running'
            assert preview_progress['progress']['stage'] == (
                'preparing_preview'
            )
            preview = map_output / 'preview' / (
                'mid360_robot_3d_map_preview.html'
            )
            preview.parent.mkdir(parents=True)
            preview.write_text('<!doctype html>', encoding='utf-8')
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)
    monkeypatch.setattr(module, '_run_delegated_session', fake_run)

    class Browser:
        @staticmethod
        def desktop_session_available() -> bool:
            return True

        @staticmethod
        def open_browser(uri: str) -> bool:
            opened.append(uri)
            return True

    monkeypatch.setattr(
        module,
        '_load_script_module',
        lambda *_args: Browser,
    )

    result = module._run_session(
        type('Args', (), {'viewer': 'browser', 'verification': 'required'})(),
        manifest,
    )

    assert result == 0
    assert calls[1][-1] == '--no-open'
    assert opened == [
        (Path(manifest['bundle_path']) / 'session.html').as_uri()
    ]
    session = json.loads(
        (Path(manifest['bundle_path']) / 'session.json').read_text(
            encoding='utf-8'
        )
    )
    preview_path = map_output / 'preview' / (
        'mid360_robot_3d_map_preview.html'
    )
    assert session['artifacts']['map_preview_html'] == str(preview_path)
    report = (Path(manifest['bundle_path']) / 'session.html').read_text(
        encoding='utf-8'
    )
    assert 'Open 3D review' in report
    assert preview_path.as_uri() in report
    assert 'http-equiv="refresh"' not in report


def test_start_mirrors_durable_runner_stages_into_live_session(
    monkeypatch,
    tmp_path: Path,
):
    """The common page should advance from mapping through verification."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    partial = Path(manifest['run']['output_dir']).with_name('map.partial')
    mapping_seen = threading.Event()
    verifying_seen = threading.Event()
    observed: list[str] = []
    original = module._write_running_session

    def record_progress(args, session_manifest, stage, active_run_dir):
        result = original(args, session_manifest, stage, active_run_dir)
        observed.append(stage)
        if stage == 'workflow_running':
            mapping_seen.set()
        if stage == 'verifying':
            verifying_seen.set()
        return result

    def fake_run(command, **_kwargs):
        _write_run_manifest(
            partial,
            status='running',
            stage='workflow_running',
        )
        assert mapping_seen.wait(timeout=2)
        _write_run_manifest(
            partial,
            status='running',
            stage='verifying',
        )
        assert verifying_seen.wait(timeout=2)
        live = json.loads(
            (Path(manifest['bundle_path']) / 'session.json').read_text(
                encoding='utf-8'
            )
        )
        jsonschema.validate(
            live,
            json.loads(SESSION_SCHEMA.read_text(encoding='utf-8')),
        )
        assert live['status'] == 'running'
        assert live['progress'] == {
            'phase': 'verifying',
            'stage': 'verifying',
            'current_step': 3,
            'total_steps': 6,
            'label': 'Verifying map quality',
            'updated_at': live['progress']['updated_at'],
        }
        live_html = (
            Path(manifest['bundle_path']) / 'session.html'
        ).read_text(encoding='utf-8')
        assert '<meta http-equiv="refresh" content="2">' in live_html
        assert 'Verifying map quality' in live_html
        assert 'Step 3 of 6' in live_html
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module, '_write_running_session', record_progress)
    monkeypatch.setattr(module.subprocess, 'run', fake_run)
    monkeypatch.setattr(module, '_run_delegated_session', fake_run)

    result = module._run_session(
        type('Args', (), {'viewer': 'none', 'verification': 'required'})(),
        manifest,
    )

    assert result == 0
    assert observed[:3] == [
        'preparing',
        'workflow_running',
        'verifying',
    ]
    final = json.loads(
        (Path(manifest['bundle_path']) / 'session.json').read_text(
            encoding='utf-8'
        )
    )
    schema = json.loads(SESSION_SCHEMA.read_text(encoding='utf-8'))
    jsonschema.validate(final, schema)
    assert final['status'] == 'verified'
    assert final['progress']['phase'] == 'complete'


def test_unchanged_stage_emits_bounded_no_eta_heartbeat(
    monkeypatch,
    tmp_path: Path,
    capsys,
):
    """A long stage should look active without inventing forward progress."""
    module = _load_module()
    manifest = _session_manifest(tmp_path)
    run_dir = Path(manifest['run']['output_dir'])
    writes: list[str] = []

    class StopAfterTwoPolls:
        polls = 0

        def wait(self, _seconds):
            self.polls += 1
            return self.polls > 2

    def write_stage(_args, _manifest, stage, _run_dir):
        writes.append(stage)
        return Path('session.json'), None, {
            'progress': module._session_progress(stage),
        }

    monotonic = iter([0.0, 1.0, 31.0])
    monkeypatch.setattr(module.time, 'monotonic', lambda: next(monotonic))
    monkeypatch.setattr(module, 'SESSION_PROGRESS_HEARTBEAT_SECONDS', 30.0)
    monkeypatch.setattr(
        module,
        '_observed_session_progress',
        lambda _map_output: ('workflow_running', run_dir),
    )
    monkeypatch.setattr(module, '_write_running_session', write_stage)

    module._monitor_session_progress(
        type('Args', (), {})(),
        manifest,
        StopAfterTwoPolls(),
    )

    output = capsys.readouterr().out
    assert writes == ['workflow_running']
    assert output.count('Map progress [2/6]: Mapping sensor data') == 1
    assert (
        'Map heartbeat [2/6]: Mapping sensor data; still running, '
        'elapsed 00:31'
    ) in output
    assert '%' not in output
    assert 'ETA' not in output
