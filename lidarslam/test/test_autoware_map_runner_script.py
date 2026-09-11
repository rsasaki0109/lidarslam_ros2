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

"""Regression tests for the one-shot Autoware map runner script."""

from __future__ import annotations

import errno
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import jsonschema
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_autoware_map_from_bag.py'
BEGINNER_SCRIPT_PATH = REPO_ROOT / 'scripts' / 'run_autoware_map_beginner.sh'
TEST_MIN_FREE_SPACE_GIB = 0.001


def _load_module():
    spec = importlib.util.spec_from_file_location('run_autoware_map_from_bag', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.DEFAULT_MIN_FREE_SPACE_GIB = TEST_MIN_FREE_SPACE_GIB
    return module


def _write_metadata(tmp_path: Path, bag_name: str, topics: list[tuple[str, str, int]]) -> Path:
    bag_dir = tmp_path / bag_name
    bag_dir.mkdir()
    storage_name = f'{bag_name}_0.db3'
    metadata = {
        'rosbag2_bagfile_information': {
            'duration': {'nanoseconds': 2_000_000_000},
            'message_count': sum(count for _, _, count in topics),
            'storage_identifier': 'sqlite3',
            'relative_file_paths': [storage_name],
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
    (bag_dir / storage_name).write_bytes(b'rosbag2 fixture\n')
    (bag_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata), encoding='utf-8')
    return bag_dir


def _compatible_pointcloud_inspection(
    _bag_path: Path,
    topic: str,
    _storage_id: str,
) -> dict:
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


def test_runner_script_supports_profiles_and_viewers():
    script = SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'preflight_autoware_map_bag.py' in script
    assert 'diagnose_autoware_map_run.py' in script
    assert 'rko_lio_graph_public_path' in script
    assert 'rko_lio_graph_mid360_preset' in script
    assert 'pointcloud_gnss_smoke' in script
    assert 'packet_applanix_smoke' in script
    assert '--viewer' in script
    assert 'verify_autoware_map.py' in script
    assert 'Next steps:' in script
    assert 'view_autoware_map.py' in script
    assert '--dry-run' in script
    assert '--editable' in script
    assert '--resume' in script
    assert 'run_manifest.json' in script
    assert 'artifact_checksums' in script


def test_runner_help_is_user_facing():
    result = subprocess.run(
        ['python3', str(SCRIPT_PATH), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'Autoware-compatible map workflow' in result.stdout
    assert 'The input must be the rosbag2 directory' in result.stdout
    assert 'Workflow profiles:' in result.stdout
    assert 'Expected successful outputs:' in result.stdout
    assert '--output-dir <dir>' in result.stdout
    assert '--min-free-space-gib <GiB>' in result.stdout
    assert '--editable' in result.stdout
    assert '--resume' in result.stdout
    assert '--guided' in result.stdout
    assert '--yes' in result.stdout
    assert 'map selection and output:' in result.stdout
    assert 'safety and lifecycle:' in result.stdout
    assert 'deprecated viewer compatibility options:' in result.stdout
    assert 'deprecated advanced viewer compatibility options:' in result.stdout
    assert 'verification:' in result.stdout
    assert '--verification {required,off}' in result.stdout


def test_start_session_summary_hides_internal_execution_detail(
    monkeypatch,
    capsys,
    tmp_path: Path,
):
    module = _load_module()
    output_dir = tmp_path / 'map'
    working_dir = tmp_path / 'map.partial'
    plan = {
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': [
            'bash',
            '/private/internal/workflow.sh',
            '--output-dir',
            str(working_dir),
        ],
    }
    storage = {
        'observed_free_bytes': 7.25 * 1024**3,
        'required_free_bytes': 5 * 1024**3,
        'probe_path': str(tmp_path),
    }
    monkeypatch.setenv(
        module.PRODUCT_SESSION_OUTPUT_ENV,
        module.PRODUCT_SESSION_OUTPUT_CONCISE,
    )

    module._print_execution_summary(
        plan,
        output_dir,
        working_dir,
        storage,
    )
    module._announce_lifecycle('workflow_running', 'run the mapping workflow')

    output = capsys.readouterr().out
    assert output.splitlines() == [
        'Map profile: RKO-LIO + graph_based_slam public path',
        'Storage check: 7.25 GiB available (5.00 GiB required)',
    ]
    assert str(output_dir) not in output
    assert str(working_dir) not in output
    assert '/private/internal/workflow.sh' not in output
    assert 'Lifecycle stage:' not in output


def test_start_session_retains_workflow_output_without_rendering_internals(
    monkeypatch,
    capsys,
    tmp_path: Path,
):
    module = _load_module()
    workflow_log = tmp_path / module.CONCISE_WORKFLOW_LOG_NAME
    internal_lines = [
        'Calling /map_save ...',
        'Mapping is ready; processing the recorded sensor data ...',
        'Generating Lanelet2 map from corrected trajectory ...',
    ]
    command = [
        sys.executable,
        '-c',
        '; '.join(f'print({line!r}, flush=True)' for line in internal_lines),
    ]

    assert module._run_workflow(
        command,
        tmp_path,
        True,
        workflow_log,
    ) == (0, False, None)

    terminal = capsys.readouterr()
    assert terminal.out.splitlines() == [internal_lines[1]]
    assert terminal.err == ''
    assert workflow_log.read_text(encoding='utf-8').splitlines() == internal_lines


def test_start_session_replays_retained_workflow_tail_on_failure(
    capsys,
    tmp_path: Path,
):
    module = _load_module()
    workflow_log = tmp_path / module.CONCISE_WORKFLOW_LOG_NAME
    command = [
        sys.executable,
        '-c',
        "print('post-process detail', flush=True); raise SystemExit(7)",
    ]

    assert module._run_workflow(
        command,
        tmp_path,
        True,
        workflow_log,
    ) == (7, False, None)

    terminal = capsys.readouterr()
    assert terminal.out == ''
    assert 'Map workflow failed. Recent workflow details:' in terminal.err
    assert 'post-process detail' in terminal.err
    assert f'Complete workflow output: {workflow_log}' in terminal.err


def test_start_session_success_defers_terminal_summary_to_parent(
    monkeypatch,
    capsys,
    tmp_path: Path,
):
    module = _load_module()
    monkeypatch.setenv(
        module.PRODUCT_SESSION_OUTPUT_ENV,
        module.PRODUCT_SESSION_OUTPUT_CONCISE,
    )
    monkeypatch.setattr(module, 'maybe_open_viewer', lambda *_args: None)

    assert module._finish_run(
        module.argparse.Namespace(),
        {'command': ['internal-workflow']},
        tmp_path / 'map',
        0,
    ) == 0

    terminal = capsys.readouterr()
    assert terminal.out == ''
    assert terminal.err == ''


def test_deprecated_run_viewer_routes_through_view_command(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    """The old combined spelling should warn and delegate without drift."""
    module = _load_module()
    recorded: list[list[str]] = []

    def fake_run(command, **kwargs):
        recorded.append(command)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, 'run', fake_run)
    args = module.argparse.Namespace(
        viewer='foxglove',
        autoware_core_dir=None,
        work_dir='/tmp/view-work',
        viewer_run_dir='/tmp/view-runtime',
        viewer_rebuild=True,
        auto_exit_secs=30,
    )
    module.maybe_open_viewer(args, tmp_path / 'map_output')

    assert recorded
    command = recorded[0]
    assert command[:2] == [
        sys.executable,
        str(module.SCRIPT_DIR / 'view_autoware_map.py'),
    ]
    assert command[command.index('--viewer') + 1] == 'foxglove'
    assert command[command.index('--runtime-dir') + 1] == '/tmp/view-runtime'
    assert '--rebuild' in command
    warning = capsys.readouterr().err
    assert 'warning: run viewer options are deprecated' in warning
    assert 'lidarslam-map view <output_dir> --viewer foxglove' in warning


def test_verification_mode_defaults_safe_and_migrates_old_alias(
    capsys: pytest.CaptureFixture[str],
):
    """Verification should default on and make every disabled mode visible."""
    module = _load_module()

    required = module.parse_args(['/tmp/bag'])
    assert module.resolve_verification_mode(required) is True
    assert capsys.readouterr().err == ''

    disabled = module.parse_args([
        '/tmp/bag',
        '--verification',
        'off',
    ])
    assert module.resolve_verification_mode(disabled) is False
    warning = capsys.readouterr().err
    assert 'map verification is disabled' in warning
    assert 'deprecated' not in warning

    legacy = module.parse_args(['/tmp/bag', '--no-verify-map'])
    assert module.resolve_verification_mode(legacy) is False
    warning = capsys.readouterr().err
    assert '--no-verify-map is deprecated' in warning
    assert '--verification off' in warning

    conflicting = module.parse_args([
        '/tmp/bag',
        '--verification',
        'off',
        '--no-verify-map',
    ])
    with pytest.raises(ValueError, match='cannot be combined'):
        module.resolve_verification_mode(conflicting)


def test_storage_preflight_records_budget_and_refuses_low_space(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = _load_module()
    output_dir = tmp_path / 'nested' / 'map_output'
    usage_type = type('DiskUsage', (), {})

    enough = usage_type()
    enough.free = 7 * 1024**3
    monkeypatch.setattr(module.shutil, 'disk_usage', lambda path: enough)
    evidence = module.check_output_storage(output_dir, 5.0)

    assert evidence == {
        'probe_path': str(tmp_path.resolve()),
        'required_free_bytes': 5 * 1024**3,
        'observed_free_bytes': 7 * 1024**3,
    }

    low = usage_type()
    low.free = 256 * 1024**2
    monkeypatch.setattr(module.shutil, 'disk_usage', lambda path: low)
    with pytest.raises(ValueError, match='insufficient free space for map output'):
        module.check_output_storage(output_dir, 5.0)


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf'])
def test_storage_preflight_rejects_unsafe_budget_values(value: str):
    module = _load_module()

    with pytest.raises(
        module.argparse.ArgumentTypeError,
        match='finite number greater than zero',
    ):
        module._minimum_free_space_gib(value)


def test_terminal_evidence_reserve_allocates_real_blocks_and_releases(
    tmp_path: Path,
):
    module = _load_module()
    before = os.statvfs(tmp_path).f_bavail
    reserve = module._allocate_emergency_evidence_reserve(tmp_path)
    after_allocate = os.statvfs(tmp_path).f_bavail

    assert reserve.stat().st_size == module.EMERGENCY_EVIDENCE_RESERVE_BYTES
    assert after_allocate < before

    module._release_emergency_evidence_reserve(reserve)
    assert not reserve.exists()


def test_main_refuses_low_space_before_creating_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    usage_type = type('DiskUsage', (), {})
    usage = usage_type()
    usage.free = 1024
    monkeypatch.setattr(module.shutil, 'disk_usage', lambda path: usage)
    monkeypatch.setattr(
        module,
        'build_execution_plan',
        lambda **kwargs: pytest.fail('workflow planning must follow storage refusal'),
    )
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--dry-run',
        ],
    )

    assert module.main() == 2
    assert 'insufficient free space for map output' in capsys.readouterr().err
    assert not output_dir.exists()
    assert not output_dir.with_name('map_output.partial').exists()


def test_atomic_manifest_write_preserves_previous_file_on_enospc(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = _load_module()
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    destination = run_dir / 'run_manifest.json'
    destination.write_text('{"status":"durable"}\n', encoding='utf-8')

    def fail_replace(source, target):
        del source, target
        raise OSError(errno.ENOSPC, 'No space left on device')

    monkeypatch.setattr(module.os, 'replace', fail_replace)
    with pytest.raises(OSError) as error:
        module._write_manifest(run_dir, {'status': 'new'})

    assert error.value.errno == errno.ENOSPC
    assert destination.read_text(encoding='utf-8') == '{"status":"durable"}\n'
    assert not (run_dir / '.run_manifest.json.tmp').exists()


def test_initial_manifest_enospc_removes_empty_partial_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow'],
        'output_dir': working_dir,
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)

    def fail_manifest_write(*args, **kwargs):
        del args, kwargs
        raise OSError(errno.ENOSPC, 'No space left on device')

    monkeypatch.setattr(module, '_write_manifest', fail_manifest_write)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--min-free-space-gib',
            '0.001',
        ],
    )

    assert module.main() == 2
    assert 'No space left on device' in capsys.readouterr().err
    assert not output_dir.exists()
    assert not working_dir.exists()


def test_emergency_reserve_failure_removes_empty_partial_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture,
):
    """A failed reserve allocation must not claim that a workflow started."""
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow'],
        'output_dir': working_dir,
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(
        module,
        '_allocate_emergency_evidence_reserve',
        lambda _path: (_ for _ in ()).throw(
            OSError(errno.ENOSPC, 'No space left on device')
        ),
    )
    monkeypatch.setattr(
        module,
        '_run_workflow',
        lambda *_args: pytest.fail('workflow must not start'),
    )
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--min-free-space-gib',
            '0.001',
        ],
    )

    assert module.main() == 2
    assert 'No space left on device' in capsys.readouterr().err
    assert not output_dir.exists()
    assert not working_dir.exists()


def test_runner_rejects_db3_file_without_traceback(tmp_path: Path):
    db3_path = tmp_path / 'demo_0.db3'
    db3_path.write_text('', encoding='utf-8')

    result = subprocess.run(
        ['python3', str(SCRIPT_PATH), str(db3_path), '--dry-run'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'not the .db3 file' in result.stderr
    assert 'Traceback' not in result.stderr


def test_runner_rejects_output_file_without_traceback(tmp_path: Path):
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_file = tmp_path / 'map_output'
    output_file.write_text('', encoding='utf-8')

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_file),
            '--dry-run',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'output directory path is a file' in result.stderr
    assert 'Traceback' not in result.stderr


def test_manifest_helpers_capture_identity_and_finalize_atomically(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    final_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    working_dir.mkdir()
    plan = {
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['python3', '-c', 'pass'],
    }

    manifest = module._build_manifest(bag_dir, final_dir, working_dir, plan)
    module._write_manifest(working_dir, manifest)
    (working_dir / 'artifact.txt').write_text('map artifact\n', encoding='utf-8')
    manifest['output']['artifact_checksums'] = module._artifact_checksums(working_dir)
    module._write_manifest(working_dir, manifest)
    module._finalize_output(working_dir, final_dir)

    saved = json.loads((final_dir / 'run_manifest.json').read_text(encoding='utf-8'))
    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v2.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(saved, schema)
    assert saved['schema_version'] == 2
    assert saved['lifecycle'] == {
        'last_error': None,
        'resume_count': 0,
        'runner_exit_code': None,
        'stage': 'initialized',
        'verification_enabled': True,
    }
    assert saved['input']['metadata_sha256'] == module._sha256(
        bag_dir / 'metadata.yaml'
    )
    assert saved['input']['storage_identifier'] == 'sqlite3'
    assert saved['input']['storage_files'] == [{
        'path': 'demo_bag_0.db3',
        'sha256': module._sha256(bag_dir / 'demo_bag_0.db3'),
        'size_bytes': 16,
    }]
    product_version = (REPO_ROOT / 'VERSION').read_text(
        encoding='utf-8'
    ).strip()
    assert saved['software']['product_version'] == product_version
    assert (
        saved['software']['package_versions']['lidarslam']
        == product_version
    )
    assert isinstance(saved['software']['git_dirty'], bool)
    assert saved['profile']['id'] == 'rko_lio_graph_public_path'
    assert saved['output']['artifact_checksums'][0]['path'] == 'artifact.txt'
    assert not working_dir.exists()


def test_finalize_rewrites_transaction_local_ros_latest_link(tmp_path: Path):
    """Finalization makes ROS's absolute transaction link portable."""
    module = _load_module()
    working_dir = tmp_path / 'map_output.partial'
    final_dir = tmp_path / 'map_output'
    session_dir = working_dir / '.ros_log' / 'session-1'
    session_dir.mkdir(parents=True)
    latest = working_dir / '.ros_log' / 'latest'
    latest.symlink_to(session_dir, target_is_directory=True)

    module._finalize_output(working_dir, final_dir)

    finalized_latest = final_dir / '.ros_log' / 'latest'
    assert finalized_latest.is_symlink()
    assert os.readlink(finalized_latest) == 'session-1'
    assert finalized_latest.resolve() == final_dir / '.ros_log' / 'session-1'
    assert not working_dir.exists()


def test_finalize_removes_dangling_transaction_local_ros_latest_link(
    tmp_path: Path,
):
    """Finalization removes a dangling transaction-local ROS link."""
    module = _load_module()
    working_dir = tmp_path / 'map_output.partial'
    final_dir = tmp_path / 'map_output'
    log_dir = working_dir / '.ros_log'
    log_dir.mkdir(parents=True)
    latest = log_dir / 'latest'
    latest.symlink_to(log_dir / 'missing-session', target_is_directory=True)

    module._finalize_output(working_dir, final_dir)

    finalized_latest = final_dir / '.ros_log' / 'latest'
    assert not finalized_latest.exists()
    assert not finalized_latest.is_symlink()


def test_finalize_preserves_external_ros_latest_link(tmp_path: Path):
    """Finalization does not rewrite a deliberately external ROS link."""
    module = _load_module()
    working_dir = tmp_path / 'map_output.partial'
    final_dir = tmp_path / 'map_output'
    external_session = tmp_path / 'external-session'
    external_session.mkdir()
    log_dir = working_dir / '.ros_log'
    log_dir.mkdir(parents=True)
    latest = log_dir / 'latest'
    latest.symlink_to(external_session, target_is_directory=True)

    module._finalize_output(working_dir, final_dir)

    finalized_latest = final_dir / '.ros_log' / 'latest'
    assert finalized_latest.is_symlink()
    assert Path(os.readlink(finalized_latest)) == external_session
    assert finalized_latest.resolve() == external_session


def test_manifest_rejects_storage_path_outside_bag(tmp_path: Path):
    module = _load_module()
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    (tmp_path / 'outside.db3').write_bytes(b'outside\n')
    metadata = {
        'rosbag2_bagfile_information': {
            'storage_identifier': 'sqlite3',
            'relative_file_paths': ['../outside.db3'],
            'topics_with_message_count': [],
        },
    }
    (bag_dir / 'metadata.yaml').write_text(
        yaml.safe_dump(metadata),
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='outside the bag'):
        module._bag_identity(bag_dir)


def test_postprocessing_lock_rejects_concurrent_owner(tmp_path: Path):
    module = _load_module()
    output_dir = tmp_path / 'map_output'

    with module._postprocess_lock(output_dir):
        with pytest.raises(RuntimeError, match='post-processing is already active'):
            with module._postprocess_lock(output_dir):
                pytest.fail('a second post-processing owner acquired the lock')


def test_main_writes_success_manifest_and_rejects_collision(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'

    def fake_plan(bag_path, profile_id, output_dir, verify_map):
        del bag_path, profile_id, verify_map
        working_dir = output_dir
        command_script = '\n'.join([
            'from pathlib import Path',
            f'root = Path({str(working_dir)!r})',
            "(root / 'pointcloud_map').mkdir(parents=True, exist_ok=True)",
            (
                "(root / 'pointcloud_map' / 'pointcloud_map_metadata.yaml')"
                ".write_text('tiles: []\\n')"
            ),
            "(root / 'map_projector_info.yaml').write_text('projector_type: Local\\n')",
        ])
        return {
            'payload': {},
            'profile_id': 'rko_lio_graph_public_path',
            'label': 'RKO-LIO + graph_based_slam public path',
            'command': ['python3', '-c', command_script],
            'output_dir': working_dir,
        }

    def fake_verify(run_dir: Path, enabled: bool):
        assert enabled is True
        (run_dir / 'verify_autoware_map.log').write_text(
            'RESULT: PASS\nPASS: 1 | WARN: 0 | FAIL: 0\n',
            encoding='utf-8',
        )

    monkeypatch.setattr(module, 'build_execution_plan', fake_plan)
    monkeypatch.setattr(module, 'maybe_verify_map', fake_verify)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
        ],
    )

    assert module.main() == 0
    stdout = capsys.readouterr().out
    for stage in (
        'workflow_running',
        'verifying',
        'finalizing',
        'diagnosing',
        'checksumming',
    ):
        assert f'Lifecycle stage: {stage}' in stdout
    assert (
        f'Review shareable receipt: '
        f'{output_dir / "first_map_validation_receipt.json"}'
    ) in stdout
    assert (
        'Report this run (PASS or FAIL): '
        f'{module.VALIDATION_ISSUE_URL}'
    ) in stdout
    assert output_dir.is_dir()
    assert not output_dir.with_name('map_output.partial').exists()
    manifest = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v2.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.validate(manifest, schema)
    assert manifest['status'] == 'succeeded'
    assert manifest['execution']['exit_code'] == 0
    assert manifest['lifecycle']['stage'] == 'complete'
    assert manifest['lifecycle']['runner_exit_code'] == 0
    assert manifest['output']['finalized'] is True
    assert manifest['output']['diagnosis_status'] == 'success'
    assert any(
        item['path'] == 'autoware_map_diagnosis.json'
        for item in manifest['output']['artifact_checksums']
    )
    receipt_path = output_dir / 'first_map_validation_receipt.json'
    receipt = json.loads(receipt_path.read_text(encoding='utf-8'))
    receipt_schema = json.loads(
        (
            REPO_ROOT
            / 'docs'
            / 'schemas'
            / 'first-map-validation-receipt-v1.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.validate(receipt, receipt_schema)
    assert receipt['status'] == 'PASS'
    assert (output_dir / 'first_map_validation_receipt.md').is_file()
    assert all(
        item['path'] not in {
            'first_map_validation_receipt.json',
            'first_map_validation_receipt.md',
        }
        for item in manifest['output']['artifact_checksums']
    )

    assert module.main() == 2


def test_diagnostic_only_run_keeps_success_with_failing_receipt(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'diagnostic_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'diagnostic_map'

    def fake_plan(bag_path, profile_id, output_dir, verify_map):
        del bag_path, profile_id
        assert verify_map is False
        command_script = '\n'.join([
            'from pathlib import Path',
            f'root = Path({str(output_dir)!r})',
            "(root / 'pointcloud_map').mkdir(parents=True)",
            (
                "(root / 'pointcloud_map' / 'pointcloud_map_metadata.yaml')"
                ".write_text('tiles: []\\n')"
            ),
            (
                "(root / 'map_projector_info.yaml')"
                ".write_text('projector_type: Local\\n')"
            ),
        ])
        return {
            'payload': {},
            'profile_id': 'rko_lio_graph_public_path',
            'label': 'diagnostic-only fixture',
            'command': ['python3', '-c', command_script],
            'output_dir': output_dir,
        }

    monkeypatch.setattr(module, 'build_execution_plan', fake_plan)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--verification',
            'off',
        ],
    )

    assert module.main() == 0
    manifest = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    receipt = json.loads(
        (
            output_dir / 'first_map_validation_receipt.json'
        ).read_text(encoding='utf-8')
    )
    assert manifest['status'] == 'succeeded'
    assert manifest['lifecycle']['verification_enabled'] is False
    assert manifest['lifecycle']['runner_exit_code'] == 0
    assert receipt['status'] == 'FAIL'
    assert receipt['verification']['autoware_status'] == 'missing'


@pytest.mark.parametrize(
    (
        'workflow_result',
        'expected_status',
        'expected_exit_code',
        'expected_error',
        'expected_stop_label',
    ),
    [
        (
            (17, False, None),
            'failed',
            17,
            'map workflow exited with code 17',
            None,
        ),
        (
            (130, False, None),
            'failed',
            130,
            'map workflow exited with code 130',
            None,
        ),
        (
            (130, True, 'map workflow interrupted by SIGINT'),
            'interrupted',
            130,
            'map workflow interrupted by SIGINT',
            'Ctrl-C',
        ),
        (
            (143, True, 'map workflow interrupted by SIGTERM'),
            'interrupted',
            143,
            'map workflow interrupted by SIGTERM',
            'SIGTERM',
        ),
    ],
)
def test_main_retains_terminal_manifest_for_failed_and_interrupted_runs(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    workflow_result,
    expected_status: str,
    expected_exit_code: int,
    expected_error: str,
    expected_stop_label: str | None,
    capsys,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'failed_map'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow'],
        'output_dir': output_dir.with_name('failed_map.partial'),
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, '_git_state', lambda: {
        'commit': '0' * 40,
        'dirty': False,
    })
    monkeypatch.setattr(module, 'maybe_verify_map', lambda *args, **kwargs: None)

    monkeypatch.setattr(module, '_run_workflow', lambda *args: workflow_result)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
        ],
    )

    assert module.main() == expected_exit_code
    manifest = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    assert manifest['status'] == expected_status
    assert manifest['execution']['exit_code'] == expected_exit_code
    assert manifest['lifecycle']['stage'] == 'complete'
    assert manifest['lifecycle']['runner_exit_code'] == expected_exit_code
    assert manifest['lifecycle']['last_error'] == expected_error
    assert manifest['output']['finalized'] is True
    assert not output_dir.with_name('failed_map.partial').exists()
    receipt = json.loads(
        (
            output_dir / 'first_map_validation_receipt.json'
        ).read_text(encoding='utf-8')
    )
    assert receipt['status'] == 'FAIL'
    assert receipt['verification']['manifest_status'] == expected_status
    terminal = capsys.readouterr()
    if expected_stop_label is None:
        assert (
            f'error: map run failed with exit code {expected_exit_code}.'
            in terminal.err
        )
        assert 'failed command: map-workflow' in terminal.err
        assert 'Map stopped by ' not in terminal.err
    else:
        assert (
            f'Map stopped by {expected_stop_label}; retained evidence: '
            f'{output_dir}'
        ) in terminal.err
        assert 'error: map run failed' not in terminal.err
        assert 'failed command:' not in terminal.err


def test_map_write_enospc_is_preserved_and_diagnosed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'disk_pressure_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'disk_pressure_map'
    working_dir = tmp_path / 'disk_pressure_map.partial'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'disk-pressure injection fixture',
        'command': ['map-workflow'],
        'output_dir': working_dir,
    }

    def inject_map_write_enospc(*args):
        del args
        (working_dir / 'map_save.log').write_text(
            'write failed: [Errno 28] No space left on device\n',
            encoding='utf-8',
        )
        return 28, False, None

    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, '_run_workflow', inject_map_write_enospc)
    monkeypatch.setattr(module, 'maybe_verify_map', lambda *args, **kwargs: None)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--min-free-space-gib',
            '0.001',
        ],
    )

    assert module.main() == 28
    manifest = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    diagnosis = json.loads(
        (output_dir / 'autoware_map_diagnosis.json').read_text(encoding='utf-8')
    )
    assert manifest['status'] == 'failed'
    assert manifest['execution']['exit_code'] == 28
    assert manifest['lifecycle']['runner_exit_code'] == 28
    assert manifest['output']['finalized'] is True
    assert diagnosis['status'] == 'runtime_failed'
    assert any(
        'output filesystem ran out of writable space or quota' in hint
        for hint in diagnosis['problem_hints']
    )


@pytest.mark.parametrize('concise_output', [False, True])
def test_workflow_supervisor_forwards_sigterm_and_reaps_process_group(
    tmp_path: Path,
    concise_output: bool,
):
    child_pid_path = tmp_path / 'child.pid'
    workflow_log = tmp_path / 'workflow.log'
    child_code = '\n'.join([
        'import os',
        'from pathlib import Path',
        'import time',
        f'Path({str(child_pid_path)!r}).write_text(str(os.getpid()))',
        'time.sleep(60)',
    ])
    probe_code = '\n'.join([
        'import importlib.util',
        'import json',
        'from pathlib import Path',
        'import sys',
        f'script = Path({str(SCRIPT_PATH)!r})',
        "spec = importlib.util.spec_from_file_location('runner_probe', script)",
        'module = importlib.util.module_from_spec(spec)',
        'spec.loader.exec_module(module)',
        (
            'result = module._run_workflow('
            f'[sys.executable, "-c", {child_code!r}], Path.cwd(), '
            f'{concise_output!r}, Path({str(workflow_log)!r}))'
        ),
        'print(json.dumps(result))',
    ])

    probe = subprocess.Popen(
        [sys.executable, '-c', probe_code],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    deadline = time.monotonic() + 5
    while not child_pid_path.is_file() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert child_pid_path.is_file()
    child_pid = int(child_pid_path.read_text(encoding='utf-8'))

    os.kill(probe.pid, signal.SIGTERM)
    stdout, stderr = probe.communicate(timeout=10)

    assert probe.returncode == 0, stderr
    assert json.loads(stdout) == [
        143,
        True,
        'map workflow interrupted by SIGTERM',
    ]
    assert workflow_log.is_file() is concise_output
    with pytest.raises(ProcessLookupError):
        os.kill(child_pid, 0)


def test_workflow_supervisor_forces_cleanup_after_grace_period(tmp_path: Path):
    module = _load_module()
    descendant_pid_path = tmp_path / 'descendant.pid'
    descendant_code = '\n'.join([
        'import signal',
        'import time',
        'signal.signal(signal.SIGTERM, signal.SIG_IGN)',
        'time.sleep(60)',
    ])
    leader_code = '\n'.join([
        'from pathlib import Path',
        'import signal',
        'import subprocess',
        'import sys',
        'import time',
        'signal.signal(signal.SIGTERM, signal.SIG_IGN)',
        (
            'child = subprocess.Popen('
            f'[sys.executable, "-c", {descendant_code!r}])'
        ),
        f'Path({str(descendant_pid_path)!r}).write_text(str(child.pid))',
        'time.sleep(60)',
    ])
    leader = subprocess.Popen(
        [sys.executable, '-c', leader_code],
        start_new_session=True,
    )
    deadline = time.monotonic() + 5
    while not descendant_pid_path.is_file() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert descendant_pid_path.is_file()
    descendant_pid = int(descendant_pid_path.read_text(encoding='utf-8'))

    module._terminate_process_group(
        leader,
        signal.SIGTERM,
        grace_secs=0.1,
    )

    assert leader.returncode == -signal.SIGKILL
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        try:
            os.kill(descendant_pid, 0)
        except ProcessLookupError:
            break
        try:
            state = Path(f'/proc/{descendant_pid}/stat').read_text(
                encoding='utf-8'
            ).split()[2]
        except (FileNotFoundError, ProcessLookupError):
            break
        if state == 'Z':
            # Minimal container PID 1 implementations may leave a killed,
            # orphaned descendant as a zombie until the container exits.
            break
        time.sleep(0.01)
    else:
        pytest.fail('forced process-group cleanup left a descendant alive')


def test_sigterm_failure_injection_preserves_recoverable_terminal_run(
    tmp_path: Path,
):
    bag_dir = _write_metadata(
        tmp_path,
        'termination_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'terminated_map'
    working_dir = tmp_path / 'terminated_map.partial'
    child_pid_path = tmp_path / 'workflow.pid'
    child_code = '\n'.join([
        'import os',
        'from pathlib import Path',
        'import time',
        f'Path({str(child_pid_path)!r}).write_text(str(os.getpid()))',
        'time.sleep(60)',
    ])
    probe_code = '\n'.join([
        'import importlib.util',
        'from pathlib import Path',
        'import sys',
        f'script = Path({str(SCRIPT_PATH)!r})',
        "spec = importlib.util.spec_from_file_location('runner_e2e_probe', script)",
        'module = importlib.util.module_from_spec(spec)',
        'spec.loader.exec_module(module)',
        f'module.DEFAULT_MIN_FREE_SPACE_GIB = {TEST_MIN_FREE_SPACE_GIB!r}',
        'module.build_execution_plan = lambda **kwargs: {',
        "    'payload': {},",
        "    'profile_id': 'rko_lio_graph_public_path',",
        "    'label': 'termination injection fixture',",
        f"    'command': [sys.executable, '-c', {child_code!r}],",
        f"    'output_dir': Path({str(working_dir)!r}),",
        '}',
        'sys.argv = [',
        '    str(script),',
        f'    {str(bag_dir)!r},',
        "    '--output-dir',",
        f'    {str(output_dir)!r},',
        ']',
        'raise SystemExit(module.main())',
    ])

    runner = subprocess.Popen(
        [sys.executable, '-c', probe_code],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    deadline = time.monotonic() + 5
    while not child_pid_path.is_file() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert child_pid_path.is_file()
    child_pid = int(child_pid_path.read_text(encoding='utf-8'))

    os.kill(runner.pid, signal.SIGTERM)
    stdout, stderr = runner.communicate(timeout=15)

    assert runner.returncode == 143, (stdout, stderr)
    assert output_dir.is_dir()
    assert not working_dir.exists()
    assert (output_dir / 'autoware_map_diagnosis.md').is_file()
    assert (output_dir / 'autoware_map_diagnosis.json').is_file()
    manifest = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'run-manifest-v2.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.validate(manifest, schema)
    assert manifest['status'] == 'interrupted'
    assert manifest['execution']['exit_code'] == 143
    assert manifest['lifecycle']['stage'] == 'complete'
    assert manifest['lifecycle']['runner_exit_code'] == 143
    assert manifest['lifecycle']['last_error'] == (
        'map workflow interrupted by SIGTERM'
    )
    assert manifest['output']['finalized'] is True
    assert manifest['output']['diagnosis_status'] == 'runtime_failed'
    with pytest.raises(ProcessLookupError):
        os.kill(child_pid, 0)


def test_main_preserves_failed_manifest_when_post_finalize_diagnosis_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['python3', '-c', 'pass'],
        'output_dir': output_dir.with_name('map_output.partial'),
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, 'maybe_verify_map', lambda *args, **kwargs: None)

    def fail_diagnosis(*args, **kwargs):
        del args, kwargs
        raise RuntimeError('diagnosis fixture failure')

    monkeypatch.setattr(module, 'write_diagnostics', fail_diagnosis)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
        ],
    )

    assert module.main() == 70
    manifest = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    assert manifest['status'] == 'failed'
    assert manifest['execution']['exit_code'] == 0
    assert manifest['lifecycle']['runner_exit_code'] == 70
    assert manifest['lifecycle']['last_error'] == 'diagnosis fixture failure'
    assert manifest['output']['finalized'] is True


@pytest.mark.parametrize(
    ('resume_from_final', 'stage'),
    [
        (False, 'workflow_finished'),
        (True, 'finalizing'),
    ],
)
def test_main_resumes_only_terminal_postprocessing_without_rerunning_workflow(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    resume_from_final: bool,
    stage: str,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow', '--output-dir', str(working_dir)],
        'output_dir': working_dir,
    }
    software = {
        'product_version': '0.6.0',
        'git_commit': '0' * 40,
        'git_dirty': False,
        'package_versions': {'lidarslam': '0.6.0'},
        'ros_distro': 'jazzy',
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, '_software_identity', lambda: software)

    manifest = module._build_manifest(bag_dir, output_dir, working_dir, plan)
    manifest['status'] = 'succeeded'
    manifest['execution'].update({
        'started_at': '2026-07-27T00:00:00Z',
        'finished_at': '2026-07-27T00:01:00Z',
        'exit_code': 0,
    })
    manifest['lifecycle']['stage'] = stage
    run_dir = output_dir if resume_from_final else working_dir
    run_dir.mkdir()
    (run_dir / 'pointcloud_map').mkdir()
    (run_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml').write_text(
        'tiles: []\n',
        encoding='utf-8',
    )
    (run_dir / 'map_projector_info.yaml').write_text(
        'projector_type: Local\n',
        encoding='utf-8',
    )
    module._write_manifest(run_dir, manifest)

    def fake_verify(current_dir: Path, enabled: bool):
        assert enabled is True
        (current_dir / 'verify_autoware_map.log').write_text(
            'RESULT: PASS\n',
            encoding='utf-8',
        )

    def fake_diagnosis(current_dir: Path, current_bag: Path):
        assert current_bag == bag_dir
        (current_dir / 'autoware_map_diagnosis.md').write_text(
            '# success\n',
            encoding='utf-8',
        )
        (current_dir / 'autoware_map_diagnosis.json').write_text(
            '{"status":"success"}\n',
            encoding='utf-8',
        )
        return {'status': 'success'}

    def fail_if_workflow_runs(*args, **kwargs):
        del args, kwargs
        raise AssertionError('resume must not execute the map workflow')

    monkeypatch.setattr(module, 'maybe_verify_map', fake_verify)
    monkeypatch.setattr(module, 'write_diagnostics', fake_diagnosis)
    monkeypatch.setattr(module.subprocess, 'run', fail_if_workflow_runs)
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--resume',
        ],
    )

    assert module.main() == 0
    assert output_dir.is_dir()
    assert not working_dir.exists()
    saved = json.loads(
        (output_dir / 'run_manifest.json').read_text(encoding='utf-8')
    )
    assert saved['status'] == 'succeeded'
    assert saved['execution']['exit_code'] == 0
    assert saved['lifecycle'] == {
        'last_error': None,
        'resume_count': 1,
        'runner_exit_code': 0,
        'stage': 'complete',
        'verification_enabled': True,
    }
    assert saved['output']['finalized'] is True


@pytest.mark.parametrize(
    ('schema_version', 'stage', 'expected_error'),
    [
        (2, 'workflow_running', 'workflow may still be running'),
        (1, 'workflow_finished', 'resume requires run manifest schema v2'),
    ],
)
def test_main_refuses_unsafe_or_legacy_resume_state(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture,
    schema_version: int,
    stage: str,
    expected_error: str,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow'],
        'output_dir': working_dir,
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, '_software_identity', lambda: {
        'product_version': '0.6.0',
        'git_commit': '0' * 40,
        'git_dirty': False,
        'package_versions': {'lidarslam': '0.6.0'},
        'ros_distro': 'jazzy',
    })
    manifest = module._build_manifest(bag_dir, output_dir, working_dir, plan)
    manifest['schema_version'] = schema_version
    manifest['lifecycle']['stage'] = stage
    manifest['status'] = 'running' if stage == 'workflow_running' else 'succeeded'
    manifest['execution'].update({
        'started_at': '2026-07-27T00:00:00Z',
        'finished_at': (
            None if stage == 'workflow_running' else '2026-07-27T00:01:00Z'
        ),
        'exit_code': None if stage == 'workflow_running' else 0,
    })
    working_dir.mkdir()
    module._write_manifest(working_dir, manifest)
    original_manifest = (working_dir / 'run_manifest.json').read_text(encoding='utf-8')
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--resume',
        ],
    )

    assert module.main() == 2
    assert expected_error in capsys.readouterr().err
    assert (working_dir / 'run_manifest.json').read_text(
        encoding='utf-8'
    ) == original_manifest


def test_main_refuses_resume_when_bag_identity_changed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture,
):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'map_output'
    working_dir = tmp_path / 'map_output.partial'
    plan = {
        'payload': {},
        'profile_id': 'rko_lio_graph_public_path',
        'label': 'RKO-LIO + graph_based_slam public path',
        'command': ['map-workflow'],
        'output_dir': working_dir,
    }
    monkeypatch.setattr(module, 'build_execution_plan', lambda **kwargs: plan)
    monkeypatch.setattr(module, '_software_identity', lambda: {
        'product_version': '0.6.0',
        'git_commit': '0' * 40,
        'git_dirty': False,
        'package_versions': {'lidarslam': '0.6.0'},
        'ros_distro': 'jazzy',
    })
    manifest = module._build_manifest(bag_dir, output_dir, working_dir, plan)
    manifest['status'] = 'succeeded'
    manifest['execution'].update({
        'started_at': '2026-07-27T00:00:00Z',
        'finished_at': '2026-07-27T00:01:00Z',
        'exit_code': 0,
    })
    manifest['lifecycle']['stage'] = 'workflow_finished'
    working_dir.mkdir()
    module._write_manifest(working_dir, manifest)
    (bag_dir / 'demo_bag_0.db3').write_bytes(b'changed bag fixture\n')
    monkeypatch.setattr(
        module.sys,
        'argv',
        [
            str(SCRIPT_PATH),
            str(bag_dir),
            '--output-dir',
            str(output_dir),
            '--resume',
        ],
    )

    assert module.main() == 2
    assert 'resume input identity mismatch' in capsys.readouterr().err
    assert working_dir.is_dir()
    assert not output_dir.exists()


def test_runner_rejects_incompatible_profile_with_available_hint(tmp_path: Path):
    bag_dir = _write_metadata(
        tmp_path,
        'demo_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )

    result = subprocess.run(
        [
            'python3',
            str(SCRIPT_PATH),
            str(bag_dir),
            '--profile',
            'pointcloud_gnss_smoke',
            '--min-free-space-gib',
            str(TEST_MIN_FREE_SPACE_GIB),
            '--dry-run',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert (
        'profile is not compatible with this bag: pointcloud_gnss_smoke'
        in result.stderr
    )
    assert 'Available profiles: none' in result.stderr
    assert 'Traceback' not in result.stderr


def test_dogfood_script_can_skip_viewer():
    script = (REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh').read_text(
        encoding='utf-8'
    )

    assert '--skip-viewer' in script
    assert 'if [[ "$SKIP_VIEWER" == "false" ]]; then' in script


def test_beginner_wrapper_exposes_simple_viewer_flags():
    script = BEGINNER_SCRIPT_PATH.read_text(encoding='utf-8')

    assert 'run_autoware_map_from_bag.py' in script
    assert '--foxglove' in script
    assert '--autoware' in script
    assert '--no-viewer' in script
    assert '--dry-run' in script
    assert '--resume' in script
    assert '--preflight-only' in script
    assert 'metadata.yaml not found' in script
    assert 'not a .db3 file' in script


def test_beginner_wrapper_help_is_user_facing():
    result = subprocess.run(
        ['bash', str(BEGINNER_SCRIPT_PATH), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'Autoware-compatible map workflow' in result.stderr
    assert 'The input must be the rosbag2 directory' in result.stderr
    assert 'Expected successful outputs:' in result.stderr
    assert '--output-dir <dir>' in result.stderr
    assert '--viewer-rebuild' in result.stderr
    assert '--resume' in result.stderr
    assert '--verification <mode>' in result.stderr


def test_beginner_wrapper_rejects_missing_option_value_before_bag_validation(
    tmp_path: Path,
):
    result = subprocess.run(
        [
            'bash',
            str(BEGINNER_SCRIPT_PATH),
            str(tmp_path / 'missing_bag'),
            '--output-dir',
            '--dry-run',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error: option requires a value: --output-dir' in result.stderr
    assert 'metadata.yaml not found' not in result.stderr


def test_beginner_wrapper_rejects_conflicting_viewer_flags(tmp_path: Path):
    result = subprocess.run(
        [
            'bash',
            str(BEGINNER_SCRIPT_PATH),
            str(tmp_path / 'missing_bag'),
            '--foxglove',
            '--autoware',
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error: viewer already set to foxglove; cannot also use --autoware' in result.stderr
    assert 'metadata.yaml not found' not in result.stderr


def test_runner_prefers_mid360_preset_for_livox_bag(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'mid360_demo_bag',
        [
            ('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 20),
            ('/livox/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )

    plan = module.build_execution_plan(
        bag_path=bag_dir,
        profile_id=None,
        output_dir=tmp_path / 'out',
        verify_map=True,
        pointcloud_inspector=_compatible_pointcloud_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
    )

    assert plan['profile_id'] == 'rko_lio_graph_mid360_preset'
    command = ' '.join(plan['command'])
    assert str(
        REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam_mid360_rko_graph.yaml'
    ) in command
    assert str(
        REPO_ROOT / 'lidarslam' / 'param' / 'rko_lio_mid360.yaml'
    ) in command


def test_public_plan_pins_both_installed_parameter_files(tmp_path: Path):
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'generic_pointcloud_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )

    plan = module.build_execution_plan(
        bag_path=bag_dir,
        profile_id=None,
        output_dir=tmp_path / 'out',
        verify_map=True,
        pointcloud_inspector=_compatible_pointcloud_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
    )

    command = ' '.join(plan['command'])
    assert plan['profile_id'] == 'rko_lio_graph_public_path'
    assert str(
        REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam.yaml'
    ) in command
    assert str(
        REPO_ROOT / 'lidarslam' / 'param' / 'rko_lio_ntu_viral.yaml'
    ) in command


def test_editable_rko_plan_wraps_mapping_with_backend_recorder(tmp_path: Path):
    """Editable runs must retain replay input inside the atomic output."""
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'editable_mid360_bag',
        [
            ('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 20),
            ('/livox/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    output_dir = tmp_path / 'out.partial'

    plan = module.build_execution_plan(
        bag_path=bag_dir,
        profile_id=None,
        output_dir=output_dir,
        verify_map=True,
        pointcloud_inspector=_compatible_pointcloud_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
        editable=True,
    )

    command = plan['command']
    assert command[:2] == [
        'bash',
        str(REPO_ROOT / 'scripts' / 'record_backend_input.sh'),
    ]
    assert command[command.index('--output-dir') + 1] == str(
        output_dir / 'backend_input'
    )
    separator = command.index('--')
    assert command[separator + 1:separator + 3] == [
        'bash',
        str(REPO_ROOT / 'scripts' / 'run_rko_lio_graph_autoware_dogfood.sh'),
    ]


def test_editable_rejects_profiles_without_deterministic_loop_replay(
    tmp_path: Path,
):
    """Opt-in editability must fail closed for unsupported frontends."""
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'gnss_bag',
        [
            ('/points', 'sensor_msgs/msg/PointCloud2', 20),
            ('/fix', 'sensor_msgs/msg/NavSatFix', 20),
        ],
    )

    with pytest.raises(ValueError, match='rko_lio_graph profile'):
        module.build_execution_plan(
            bag_path=bag_dir,
            profile_id='pointcloud_gnss_smoke',
            output_dir=tmp_path / 'out',
            verify_map=True,
            pointcloud_inspector=_compatible_pointcloud_inspection,
            timestamp_inspector=_monotonic_timestamp_inspection,
            editable=True,
        )


def test_sensor_setup_bundle_overrides_params_and_frames(tmp_path: Path):
    """A setup bundle must reach the normal runner without losing fields."""
    module = _load_module()
    bag_dir = _write_metadata(
        tmp_path,
        'mid360_setup_bag',
        [
            ('/livox/lidar', 'sensor_msgs/msg/PointCloud2', 20),
            ('/livox/imu', 'sensor_msgs/msg/Imu', 180),
        ],
    )
    lidarslam_param = tmp_path / 'lidarslam.yaml'
    rko_param = tmp_path / 'rko_lio.yaml'
    lidarslam_param.write_text('graph_based_slam: {}\n', encoding='utf-8')
    rko_param.write_text('initialization_phase: false\n', encoding='utf-8')

    plan = module.build_execution_plan(
        bag_path=bag_dir,
        profile_id='rko_lio_graph_mid360_preset',
        output_dir=tmp_path / 'out',
        verify_map=True,
        pointcloud_inspector=_compatible_pointcloud_inspection,
        timestamp_inspector=_monotonic_timestamp_inspection,
        lidarslam_param=lidarslam_param,
        rko_param=rko_param,
        base_frame='robot_base',
        lidar_frame='livox_frame',
        imu_frame='imu_link',
    )

    command = plan['command']
    assert command[command.index('--lidarslam-param') + 1] == str(
        lidarslam_param
    )
    assert command[command.index('--rko-param') + 1] == str(rko_param)
    assert command[command.index('--base-frame') + 1] == 'robot_base'
    assert command[command.index('--lidar-frame') + 1] == 'livox_frame'
    assert command[command.index('--imu-frame') + 1] == 'imu_link'
