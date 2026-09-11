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

"""Tests for public MID-360 dataset map candidate selection."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
RUNNER_SCRIPT = SCRIPT_DIR / 'run_mid360_robot_public_dataset_map_candidates.py'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_public_dataset_map_runner import (  # noqa: E402
    PUBLIC_DATASET_MAP_CANDIDATES_JSON,
    PUBLIC_DATASET_MAP_CANDIDATES_MARKDOWN,
    PublicDatasetMapCandidateSelector,
    PublicDatasetMapRunner,
    PublicDatasetMapRunOptions,
    PublicDatasetMapSafetyOptions,
    PublicDatasetMapSelectionOptions,
    render_map_candidates_markdown,
)


TEST_SAFETY_OPTIONS = PublicDatasetMapSafetyOptions(min_free_bytes=0)


def _write_bag(path: Path, duration_ns: int = 10_000_000_000) -> None:
    path.mkdir(parents=True)
    (path / 'metadata.yaml').write_text(
        json.dumps({
            'rosbag2_bagfile_information': {
                'duration': {'nanoseconds': duration_ns},
                'message_count': 1,
                'topics_with_message_count': [],
            },
        }),
        encoding='utf-8',
    )
    (path / 'sample.db3').write_bytes(b'bag-bytes')


def _report(root: Path, command: str | None = None) -> dict:
    driving_bag = root / 'bags' / 'driving'
    hard_bag = root / 'bags' / 'hard'
    driving_out = root / 'out' / 'driving_slam_mid360'
    hard_out = root / 'out' / 'hard_pointcloud_mid360_outdoor_kidnap_a'
    _write_bag(driving_bag)
    _write_bag(hard_bag, duration_ns=20_000_000_000)
    driving_out.mkdir(parents=True)
    hard_out.mkdir(parents=True)
    driving_command = command or f'{sys.executable} -c "print(1)" --output-dir {driving_out}'
    hard_command = command or f'{sys.executable} -c "print(1)" --output-dir {hard_out}'
    return {
        'status': 'WARN',
        'datasets': [
            {
                'dataset_id': 'driving_slam_mid360',
                'title': 'Driving',
                'status': 'WARN',
                'ready_for_mid360_launch': True,
                'selected_bag_path': str(driving_bag),
                'selected_topics': {'pointcloud': '/livox/lidar', 'imu': '/livox/imu'},
                'rates_hz': {'pointcloud': 10.0, 'imu': 200.0},
                'sampled_frames': {'pointcloud': ['livox_frame'], 'imu': ['livox_frame']},
                'warnings': [{'id': 'tf_metadata', 'status': 'warn'}],
                'artifact_paths': {
                    'run_plan_json': str(driving_out / 'mid360_robot_run_plan.json')
                },
                'map_command_shell': driving_command,
            },
            {
                'dataset_id': 'hard_pointcloud_mid360_outdoor_kidnap_a',
                'title': 'Hard',
                'status': 'WARN',
                'ready_for_mid360_launch': True,
                'selected_bag_path': str(hard_bag),
                'selected_topics': {'pointcloud': '/livox/points', 'imu': '/livox/imu'},
                'rates_hz': {'pointcloud': 7.0, 'imu': 200.0},
                'sampled_frames': {'pointcloud': ['livox_frame'], 'imu': ['livox_frame']},
                'warnings': [{'id': 'tf_metadata', 'status': 'warn'}],
                'artifact_paths': {'run_plan_json': str(hard_out / 'mid360_robot_run_plan.json')},
                'map_command_shell': hard_command,
            },
            {
                'dataset_id': 'failed_mid360',
                'title': 'Failed',
                'status': 'FAIL',
                'ready_for_mid360_launch': False,
                'selected_bag_path': '',
                'selected_topics': {},
                'rates_hz': {},
                'sampled_frames': {},
                'warnings': [{'id': 'pointcloud2', 'status': 'fail'}],
                'map_command_shell': '',
            },
        ],
    }


def test_selector_allows_warn_ready_candidates_by_default(tmp_path: Path):
    selected, skipped = PublicDatasetMapCandidateSelector().select(
        _report(tmp_path),
        PublicDatasetMapSelectionOptions(),
    )

    assert [item['dataset_id'] for item in selected] == [
        'driving_slam_mid360',
        'hard_pointcloud_mid360_outdoor_kidnap_a',
    ]
    assert skipped[0]['dataset_id'] == 'failed_mid360'
    assert 'status FAIL' in skipped[0]['skip_reason']


def test_selector_can_require_pass_status(tmp_path: Path):
    selected, skipped = PublicDatasetMapCandidateSelector().select(
        _report(tmp_path),
        PublicDatasetMapSelectionOptions(allow_warn=False),
    )

    assert selected == []
    assert {item['dataset_id'] for item in skipped} == {
        'driving_slam_mid360',
        'hard_pointcloud_mid360_outdoor_kidnap_a',
        'failed_mid360',
    }


def test_runner_writes_plan_manifest(tmp_path: Path):
    report_path = tmp_path / 'report.json'
    output_dir = tmp_path / 'out'
    report_path.write_text(json.dumps(_report(tmp_path)), encoding='utf-8')

    runner = PublicDatasetMapRunner(report_path=report_path, output_dir=output_dir)
    manifest = runner.build_manifest(
        PublicDatasetMapSelectionOptions(limit=1),
        safety_options=TEST_SAFETY_OPTIONS,
        run=False,
    )
    paths = runner.write_manifest(manifest)
    markdown = render_map_candidates_markdown(manifest)

    assert manifest['status'] == 'READY'
    assert manifest['mode'] == 'PLAN'
    assert len(manifest['candidates']) == 1
    assert manifest['candidates'][0]['safety']['status'] == 'OK'
    assert manifest['candidates'][0]['safety']['bag_duration_sec'] == 10.0
    assert paths['json'] == output_dir / PUBLIC_DATASET_MAP_CANDIDATES_JSON
    assert paths['markdown'] == output_dir / PUBLIC_DATASET_MAP_CANDIDATES_MARKDOWN
    assert 'driving_slam_mid360' in markdown
    assert json.loads(paths['json'].read_text(encoding='utf-8'))['status'] == 'READY'


def test_runner_executes_candidates_when_run_is_requested(tmp_path: Path):
    marker = tmp_path / 'marker.txt'
    command = (
        f'{sys.executable} -c "from pathlib import Path; '
        f'Path({str(marker)!r}).write_text({chr(39)}ok{chr(39)})"'
    )
    report_path = tmp_path / 'report.json'
    output_dir = tmp_path / 'out'
    report_path.write_text(json.dumps(_report(tmp_path, command)), encoding='utf-8')

    manifest = PublicDatasetMapRunner(
        report_path=report_path,
        output_dir=output_dir,
    ).build_manifest(
        PublicDatasetMapSelectionOptions(dataset_ids=('driving_slam_mid360',)),
        safety_options=TEST_SAFETY_OPTIONS,
        run=True,
    )

    assert manifest['status'] == 'PASS'
    assert manifest['runs'][0]['returncode'] == 0
    assert marker.read_text(encoding='utf-8') == 'ok'


def test_runner_times_out_and_cleans_up_process_group(tmp_path: Path):
    report_path = tmp_path / 'report.json'
    output_dir = tmp_path / 'out'
    command = (
        f'{sys.executable} -c "import subprocess, time; '
        'subprocess.Popen([\'sleep\', \'30\']); time.sleep(30)"'
    )
    report_path.write_text(json.dumps(_report(tmp_path, command)), encoding='utf-8')

    manifest = PublicDatasetMapRunner(
        report_path=report_path,
        output_dir=output_dir,
    ).build_manifest(
        PublicDatasetMapSelectionOptions(dataset_ids=('driving_slam_mid360',)),
        safety_options=TEST_SAFETY_OPTIONS,
        run_options=PublicDatasetMapRunOptions(timeout_sec=1),
        run=True,
    )

    assert manifest['status'] == 'FAIL'
    assert manifest['runs'][0]['returncode'] == 124
    assert manifest['runs'][0]['timed_out'] is True


def test_runner_blocks_candidate_when_map_outputs_already_exist(tmp_path: Path):
    report = _report(tmp_path)
    collision_dir = tmp_path / 'out' / 'driving_slam_mid360' / 'pointcloud_map'
    collision_dir.mkdir()
    report_path = tmp_path / 'report.json'
    report_path.write_text(json.dumps(report), encoding='utf-8')

    manifest = PublicDatasetMapRunner(
        report_path=report_path,
        output_dir=tmp_path / 'manifest',
    ).build_manifest(
        PublicDatasetMapSelectionOptions(dataset_ids=('driving_slam_mid360',)),
        safety_options=TEST_SAFETY_OPTIONS,
        run=False,
    )

    assert manifest['status'] == 'BLOCKED'
    assert manifest['candidates'][0]['safety']['map_output_collision'] is True
    assert 'map output collision detected' in manifest['blocked'][0]['safety']['failures']


def test_runner_blocks_candidate_when_free_space_reserve_is_too_high(tmp_path: Path):
    report_path = tmp_path / 'report.json'
    report_path.write_text(json.dumps(_report(tmp_path)), encoding='utf-8')

    manifest = PublicDatasetMapRunner(
        report_path=report_path,
        output_dir=tmp_path / 'manifest',
    ).build_manifest(
        PublicDatasetMapSelectionOptions(dataset_ids=('driving_slam_mid360',)),
        safety_options=PublicDatasetMapSafetyOptions(min_free_bytes=10**30),
        run=False,
    )

    assert manifest['status'] == 'BLOCKED'
    assert manifest['candidates'][0]['safety']['capacity_ok'] is False
    assert 'free space is below estimated output plus reserve' in (
        manifest['blocked'][0]['safety']['failures']
    )


def test_runner_cli_outputs_json(tmp_path: Path):
    report_path = tmp_path / 'report.json'
    output_dir = tmp_path / 'out'
    report_path.write_text(json.dumps(_report(tmp_path)), encoding='utf-8')

    result = subprocess.run(
        [
            sys.executable,
            str(RUNNER_SCRIPT),
            '--report',
            str(report_path),
            '--output-dir',
            str(output_dir),
            '--datasets',
            'hard_pointcloud_mid360_outdoor_kidnap_a',
            '--run-timeout-sec',
            '10',
            '--min-free-gb',
            '0',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    manifest = json.loads(result.stdout)

    assert manifest['status'] == 'READY'
    assert [item['dataset_id'] for item in manifest['candidates']] == [
        'hard_pointcloud_mid360_outdoor_kidnap_a',
    ]
    assert (output_dir / PUBLIC_DATASET_MAP_CANDIDATES_JSON).is_file()
