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

"""Regression tests for the Autoware map run diagnosis helper."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import jsonschema
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'diagnose_autoware_map_run.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('diagnose_autoware_map_run', SCRIPT_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_summary_marks_success_when_map_and_verify_pass_exist(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'run'
    pointcloud_dir = run_dir / 'pointcloud_map'
    pointcloud_dir.mkdir(parents=True)
    meta = pointcloud_dir / 'pointcloud_map_metadata.yaml'
    meta.write_text('tile_size: 20\n', encoding='utf-8')
    (run_dir / 'map_projector_info.yaml').write_text(
        yaml.safe_dump({'projector_type': 'LocalCartesian'}),
        encoding='utf-8',
    )
    (run_dir / 'verify_autoware_map.log').write_text(
        'PASS: 8  |  WARN: 0  |  FAIL: 0\nRESULT: PASS -- map is Autoware-compatible\n',
        encoding='utf-8',
    )
    (run_dir / 'lidarslam.launch.log').write_text(
        'RKO LIO Node is up!\n[graph_based_slam]: initialization end\n',
        encoding='utf-8',
    )

    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    (bag_dir / 'metadata.yaml').write_text(
        yaml.safe_dump({
            'rosbag2_bagfile_information': {
                'duration': {'nanoseconds': 1},
                'message_count': 0,
                'topics_with_message_count': [],
            },
        }),
        encoding='utf-8',
    )
    summary = module.summarize_run(run_dir, bag_dir)

    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'diagnosis-v1.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.validate(summary, schema)
    assert summary['bag_preflight']['schema_version'] == 6
    assert summary['schema_version'] == 1
    assert summary['schema_uri'].endswith('/schemas/diagnosis-v1.schema.json')
    assert summary['status'] == 'success'
    assert summary['verify']['result'] == 'PASS'
    assert summary['projector_type'] == 'LocalCartesian'
    assert any(
        'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh' in step
        for step in summary['suggested_next_steps']
    )


def test_summary_reports_tf_issue_hints(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    (run_dir / 'lidarslam.launch.log').write_text(
        "Could not find a connection between 'odom' and 'velodyne_front'\n"
        'TF_NO_FRAME_ID\n'
        'process has died\n',
        encoding='utf-8',
    )
    (run_dir / 'map_save.log').write_text(
        'map_save service call failed\n',
        encoding='utf-8',
    )

    summary = module.summarize_run(run_dir)
    hints = '\n'.join(summary['problem_hints'])

    assert summary['status'] == 'runtime_failed'
    assert 'TF tree connectivity was missing' in hints
    assert 'The /map_save service call failed' in hints
    assert 'A ROS node died during the run' in hints
    assert any('tail -n 120' in step for step in summary['suggested_next_steps'])


def test_summary_reports_map_write_disk_exhaustion(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    (run_dir / 'map_save.log').write_text(
        'write failed: [Errno 28] No space left on device\n',
        encoding='utf-8',
    )

    summary = module.summarize_run(run_dir)
    hints = '\n'.join(summary['problem_hints'])

    assert summary['status'] == 'runtime_failed'
    assert 'output filesystem ran out of writable space or quota' in hints
    assert 'free storage' in hints


def test_summary_recognizes_pcl_raw_fallocate_enospc(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    (run_dir / 'slam.launch.log').write_text(
        (
            '[pcl::PCDWriter::writeBinaryCompressed] '
            'raw_fallocate(length=1140644) returned 28. '
            'errno: 2 strerror: No such file or directory\n'
            "terminate called after throwing an instance of 'pcl::IOException'\n"
            'what(): [pcl::PCDWriter::writeBinaryCompressed] '
            'Error during raw_fallocate ()!\n'
        ),
        encoding='utf-8',
    )

    summary = module.summarize_run(run_dir)
    hints = '\n'.join(summary['problem_hints'])

    assert summary['status'] == 'runtime_failed'
    assert 'output filesystem ran out of writable space or quota' in hints
    assert 'free storage' in hints


def test_summary_uses_terminal_manifest_as_runtime_failure_evidence(
    tmp_path: Path,
):
    module = _load_module()
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    (run_dir / 'run_manifest.json').write_text(
        json.dumps({
            'status': 'interrupted',
            'lifecycle': {
                'last_error': 'map workflow interrupted by SIGTERM',
            },
        }),
        encoding='utf-8',
    )

    summary = module.summarize_run(run_dir)

    assert summary['status'] == 'runtime_failed'
    assert 'map workflow interrupted by SIGTERM' in summary['problem_hints']


def test_summary_reports_unreadable_manifest_without_traceback(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    (run_dir / 'run_manifest.json').write_text('{not json', encoding='utf-8')

    summary = module.summarize_run(run_dir)

    assert summary['status'] == 'runtime_failed'
    assert any(
        hint.startswith('The run manifest is unreadable:')
        for hint in summary['problem_hints']
    )


def test_reported_map_symptoms_are_bounded_and_schema_valid(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'run with spaces'
    bag_dir = tmp_path / 'bag with spaces'
    run_dir.mkdir()
    bag_dir.mkdir()
    (run_dir / 'run_manifest.json').write_text(
        json.dumps({'input': {'bag_path': str(bag_dir)}}),
        encoding='utf-8',
    )
    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'diagnosis-v1.schema.json'
        ).read_text(encoding='utf-8')
    )

    for symptom in module.SYMPTOM_CHOICES:
        summary = module.summarize_run(run_dir, symptom=symptom)
        jsonschema.validate(summary, schema)
        triage = summary['symptom_triage']

        assert triage['symptom'] == symptom
        assert triage['code'] == symptom
        assert triage['basis'] == (
            'USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED'
        )
        assert triage['checks']
        assert triage['avoid']
        assert summary['suggested_next_steps'] == triage['next_commands']
        assert all('<' not in command for command in triage['next_commands'])
        assert all('launch' not in command for command in triage['next_commands'])
        assert triage['next_commands'][-1].startswith(
            'lidarslam-map support '
        )
        if symptom == 'map-is-not-visible':
            assert triage['next_commands'][0].startswith(
                'lidarslam-map view '
            )
        else:
            assert triage['next_commands'][0].startswith(
                'lidarslam-map doctor '
            )


def test_symptom_triage_does_not_invent_missing_bag_or_root_cause(
    tmp_path: Path,
):
    module = _load_module()
    run_dir = tmp_path / 'run'
    run_dir.mkdir()

    summary = module.summarize_run(
        run_dir,
        symptom='pose-drifts-or-oscillates',
    )
    rendered = module.render_markdown(summary)
    commands = summary['symptom_triage']['next_commands']

    assert len(commands) == 2
    assert commands[0].startswith('lidarslam-map inspect ')
    assert commands[1].startswith('lidarslam-map support ')
    assert all('/path/to/' not in command for command in commands)
    assert 'not an automatic root-cause or accuracy diagnosis' in rendered
    assert 'Do not change graph weights' in rendered


def test_symptom_triage_rejects_unknown_internal_value(tmp_path: Path):
    module = _load_module()
    run_dir = tmp_path / 'run'
    run_dir.mkdir()

    with pytest.raises(ValueError, match='unsupported map symptom'):
        module.summarize_run(run_dir, symptom='guess-the-fix')


def test_cli_help_is_user_facing():
    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), '--help'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert 'Autoware-compatible map workflow output directory' in result.stdout
    assert 'not its pointcloud_map/ child' in result.stdout
    assert 'Files this tool checks when present:' in result.stdout
    assert 'diagnose_autoware_map_run.py output/my_map_run --write' in result.stdout
    assert '--symptom' in result.stdout
    assert 'map-spins-or-spirals' in result.stdout
    assert 'this records a user report' in result.stdout
    assert 'automatic root cause.' in result.stdout


def test_cli_rejects_missing_run_dir_without_traceback(tmp_path: Path):
    missing_run_dir = tmp_path / 'missing_run'

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(missing_run_dir)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'run directory does not exist' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_rejects_pointcloud_map_child_without_traceback(tmp_path: Path):
    pointcloud_map_dir = tmp_path / 'run' / 'pointcloud_map'
    pointcloud_map_dir.mkdir(parents=True)
    (pointcloud_map_dir / 'pointcloud_map_metadata.yaml').write_text(
        'tile_size: 20\n',
        encoding='utf-8',
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(pointcloud_map_dir)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'nested pointcloud_map directory' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_write_creates_diagnosis_files(tmp_path: Path):
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    (run_dir / 'verify_autoware_map.log').write_text(
        'PASS: 8  |  WARN: 0  |  FAIL: 0\nRESULT: PASS -- map is Autoware-compatible\n',
        encoding='utf-8',
    )

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(run_dir), '--write'],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert '# Autoware Map Run Diagnosis' in result.stdout
    assert (run_dir / 'autoware_map_diagnosis.md').is_file()
    assert (run_dir / 'autoware_map_diagnosis.json').is_file()


def test_cli_rejects_missing_bag_context_without_traceback(tmp_path: Path):
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    missing_bag = tmp_path / 'missing_bag'

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(run_dir), '--bag', str(missing_bag)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'rosbag2 directory does not exist' in result.stderr
    assert 'Traceback' not in result.stderr


def test_cli_rejects_db3_bag_context_without_traceback(tmp_path: Path):
    run_dir = tmp_path / 'run'
    run_dir.mkdir()
    db3_path = tmp_path / 'demo_0.db3'
    db3_path.write_text('', encoding='utf-8')

    result = subprocess.run(
        [sys.executable, str(SCRIPT_PATH), str(run_dir), '--bag', str(db3_path)],
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert 'error:' in result.stderr
    assert 'not the .db3 file' in result.stderr
    assert 'Traceback' not in result.stderr
