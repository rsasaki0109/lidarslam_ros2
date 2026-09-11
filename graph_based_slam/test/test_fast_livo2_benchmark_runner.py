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


"""Unit tests for FAST-LIVO2 benchmark artifact parsers."""

import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    'run_fast_livo2_benchmark', ROOT / 'scripts/run_fast_livo2_benchmark.py')
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)
SUMMARY_SPEC = importlib.util.spec_from_file_location(
    'summarize_fast_livo2_benchmark',
    ROOT / 'scripts/summarize_fast_livo2_benchmark.py')
SUMMARY = importlib.util.module_from_spec(SUMMARY_SPEC)
SUMMARY_SPEC.loader.exec_module(SUMMARY)


def test_direct_source_cli_help_works_without_pythonpath():
    result = subprocess.run(
        [sys.executable, str(ROOT / 'scripts/run_fast_livo2_benchmark.py'),
         '--help'],
        cwd=ROOT,
        env={'PATH': str(Path(sys.executable).parent)},
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert 'usage: run_fast_livo2_benchmark.py' in result.stdout
    assert '--phase-contract' in result.stdout


def test_parse_time_report(tmp_path):
    report = tmp_path / 'time.txt'
    report.write_text(
        'Elapsed (wall clock) time (h:mm:ss or m:ss): 2:05.50\n'
        'Maximum resident set size (kbytes): 2048\n')
    assert RUNNER.parse_time_report(report) == {
        'wall_seconds': 125.5, 'peak_rss_kb': 2048, 'peak_rss_mb': 2.0}


def test_processing_rtf_upper_bound_includes_full_drain_window():
    assert RUNNER.processing_rtf_upper_bound(95.0, 5.0, 125.0) == 0.8
    assert RUNNER.processing_rtf_upper_bound(None, 5.0, 125.0) is None
    assert RUNNER.processing_rtf_upper_bound(95.0, 5.0, 0.0) is None


def test_machine_fingerprint_is_stable_and_does_not_expose_private_ids():
    first = RUNNER.benchmark_machine_fingerprint()
    second = RUNNER.benchmark_machine_fingerprint()
    assert first == second
    assert re.fullmatch(r'[0-9a-f]{64}', first['machine_id'])
    assert first['logical_cpu_count'] > 0
    assert 'private_identifiers' not in first


def test_external_bag_is_mounted_read_only_without_copy(tmp_path):
    asset_root = tmp_path / 'assets'
    asset_root.mkdir()
    external = tmp_path / 'datasets' / 'holdout.bag'
    external.parent.mkdir()
    external.touch()
    inside, mount = RUNNER.bag_container_binding(external, asset_root)
    assert inside == '/input/input.bag'
    assert mount == ['-v', f'{external}:/input/input.bag:ro']

    local = asset_root / 'existing.bag'
    local.touch()
    assert RUNNER.bag_container_binding(local, asset_root) == (
        '/bench/existing.bag', [])


def test_frozen_input_manifest_must_match_selected_representation(tmp_path):
    manifest = tmp_path / 'manifest.json'
    manifest.write_text(json.dumps({
        'status': 'frozen', 'slot': 'holdout_1', 'sequence': 'exp99',
        'hashes': {'raw_rosbag1_sha256': 'a' * 64,
                   'canonical_rosbag2_tree_sha256': 'b' * 64}}))
    provenance = RUNNER.validate_frozen_input_manifest(
        manifest, 'raw_rosbag1_sha256', 'a' * 64)
    assert provenance['slot'] == 'holdout_1'
    try:
        RUNNER.validate_frozen_input_manifest(
            manifest, 'canonical_rosbag2_tree_sha256', 'c' * 64)
    except ValueError as error:
        assert 'differs from frozen manifest' in str(error)
    else:
        raise AssertionError('mismatched frozen bag was accepted')


def test_map_export_mount_is_separate_and_opt_in(tmp_path):
    assert RUNNER.map_output_binding(tmp_path, False) == []
    mount = RUNNER.map_output_binding(tmp_path, True)
    assert mount == ['-v', f"{tmp_path / 'fast_log'}:/bench/FAST-LIVO2/Log"]
def test_fast_log_mount_always_isolates_exact_trajectory_and_optional_map(tmp_path):
    mount = RUNNER.fast_log_binding(tmp_path, False)
    assert mount == ['-v', f'{tmp_path / "fast_log"}:/bench/FAST-LIVO2/Log']
    assert (tmp_path / 'fast_log' / 'result').is_dir()
    assert not (tmp_path / 'fast_log' / 'pcd').exists()
    mount = RUNNER.fast_log_binding(tmp_path, True)
    assert (tmp_path / 'fast_log' / 'pcd').is_dir()


def test_fast_map_prefers_complete_downsampled_official_output(tmp_path):
    pcd = tmp_path / 'fast_log' / 'pcd'
    pcd.mkdir(parents=True)
    raw = pcd / 'all_raw_points.pcd'
    raw.write_bytes(b'raw')
    assert RUNNER.select_fast_map(tmp_path) == raw
    downsampled = pcd / 'all_downsampled_points.pcd'
    downsampled.write_bytes(b'downsampled')
    assert RUNNER.select_fast_map(tmp_path) == downsampled


def test_odometry_csv_to_tum(tmp_path):
    source = tmp_path / 'odom.csv'
    destination = tmp_path / 'trajectory.tum'
    source.write_text(
        'field.header.stamp,field.pose.pose.position.x,'
        'field.pose.pose.position.y,field.pose.pose.position.z,'
        'field.pose.pose.orientation.x,field.pose.pose.orientation.y,'
        'field.pose.pose.orientation.z,field.pose.pose.orientation.w\n'
        '10.25,1,2,3,0,0,0,1\n')
    result = RUNNER.odometry_csv_to_tum(source, destination)
    assert result['samples'] == 1
    assert result['last_stamp'] == 10.25
    assert destination.read_text() == '10.250000000 1 2 3 0 0 0 1\n'


def test_odometry_csv_normalizes_ros1_nanosecond_stamp(tmp_path):
    source = tmp_path / 'odom.csv'
    destination = tmp_path / 'trajectory.tum'
    source.write_text(
        'field.header.stamp,field.pose.pose.position.x,'
        'field.pose.pose.position.y,field.pose.pose.position.z,'
        'field.pose.pose.orientation.x,field.pose.pose.orientation.y,'
        'field.pose.pose.orientation.z,field.pose.pose.orientation.w\n'
        '1646304541918177886,1,2,3,0,0,0,1\n')
    result = RUNNER.odometry_csv_to_tum(source, destination)
    assert abs(result['last_stamp'] - 1646304541.9181778) < 1e-6


def test_official_state_trajectory_and_imu_prism_offset(tmp_path):
    result_dir = tmp_path / 'fast_log' / 'result'
    result_dir.mkdir(parents=True)
    (result_dir / 'benchmark.txt').write_text(
        '10.0 1 2 3 0 0 0 1\n'
        '10.1 1 2 3 0 0 0.7071067811865475 0.7071067811865476\n')
    info, source = RUNNER.collect_official_state_trajectory(tmp_path)
    assert info['samples'] == 2
    assert source.endswith('benchmark.txt')
    output = tmp_path / 'trajectory_prism.tum'
    RUNNER.apply_tum_translation_offset(
        tmp_path / 'trajectory_imu.tum', output, (1.0, 0.0, 0.0))
    rows = [[float(value) for value in line.split()]
            for line in output.read_text().splitlines()]
    assert rows[0][1:4] == [2.0, 2.0, 3.0]
    assert abs(rows[1][1] - 1.0) < 1e-9
    assert abs(rows[1][2] - 3.0) < 1e-9


def test_reference_offset_requires_explicit_source_frame_key(tmp_path):
    metadata = tmp_path / 'reference.json'
    metadata.write_text(json.dumps({
        'imu_to_prism_translation_m': {'x': -0.2, 'y': 0.1, 'z': -0.3}}))
    assert RUNNER.load_reference_offset(metadata, 'imu') == (-0.2, 0.1, -0.3)
    try:
        RUNNER.load_reference_offset(metadata, 'lidar')
    except ValueError as error:
        assert 'lidar_to_prism' in str(error)
    else:
        raise AssertionError('missing source-frame lever arm was accepted')


def test_scored_summary_rejects_mixed_provenance(tmp_path):
    reference = tmp_path / 'gt.tum'
    reference.write_text('1 0 0 0 0 0 0 1\n')
    closure_identity = SUMMARY.current_rival_source_closure_identity(
        yaml.safe_load((ROOT / 'configs/slam_benchmark_profiles/'
                       'competitive_slam_v1.yaml').read_text()), root=ROOT)
    for index, bag_hash in enumerate(('a', 'b'), start=1):
        run = tmp_path / f'run_{index:02d}'
        run.mkdir()
        (run / 'run.json').write_text(json.dumps({
            'provenance': {'bag_sha256': bag_hash,
                           'source': {'revision': 'revision'},
                           'container_image_id': 'image',
                           'rival_source_closure': closure_identity},
            'completion': {'trajectory_complete': True,
                           'process_exit_status': 0,
                           'trajectory_end_gap_seconds': 0.01},
            'trajectory': {'samples': 2},
            'runtime': {'realtime_factor': 1.0,
                        'mapper': {'peak_rss_mb': 100.0}},
        }))
        (run / 'ape_vs_gt.txt').write_text('rmse: 0.1\npairs: 3\n')
    try:
        SUMMARY.summarize(tmp_path, reference)
    except ValueError as error:
        assert 'provenance differs' in str(error)
    else:
        raise AssertionError('mixed provenance was accepted')
