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

"""Regression tests for the recommended RKO-LIO benchmark helpers."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import struct
import subprocess
import sys

import jsonschema
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
REFERENCE_SCRIPT = REPO_ROOT / 'scripts' / 'generate_ntu_viral_tnp01_reference.py'
WRITE_METRICS_SCRIPT = REPO_ROOT / 'scripts' / 'write_rko_lio_benchmark_metrics.py'
BENCHMARK_PROVENANCE_SCRIPT = REPO_ROOT / 'scripts' / 'benchmark_provenance.py'
sys.path.insert(0, str(REPO_ROOT / 'scripts'))


def _load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REFERENCE_MODULE = _load_module(REFERENCE_SCRIPT, 'generate_ntu_viral_tnp01_reference')
WRITE_METRICS_MODULE = _load_module(
    WRITE_METRICS_SCRIPT, 'write_rko_lio_benchmark_metrics')
BENCHMARK_PROVENANCE_MODULE = _load_module(
    BENCHMARK_PROVENANCE_SCRIPT, 'benchmark_provenance_for_test')


def test_git_provenance_explicitly_trusts_only_selected_repo(
        monkeypatch, tmp_path: Path):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        stdout = 'a' * 40 + '\n' if 'rev-parse' in command else ''
        return subprocess.CompletedProcess(command, 0, stdout=stdout, stderr='')

    monkeypatch.setattr(BENCHMARK_PROVENANCE_MODULE.subprocess, 'run', fake_run)

    state = BENCHMARK_PROVENANCE_MODULE._git_state(tmp_path)

    expected_prefix = ['git', '-c', f'safe.directory={tmp_path}']
    assert [call[0][:3] for call in calls] == [
        expected_prefix, expected_prefix]
    assert all(call[1]['cwd'] == tmp_path for call in calls)
    assert state == {'git_commit': 'a' * 40, 'git_dirty': False}


def test_git_provenance_rejects_unidentifiable_commit(
        monkeypatch, tmp_path: Path):
    def fake_run(command, **kwargs):
        return subprocess.CompletedProcess(
            command, 128, stdout='', stderr='dubious ownership')

    monkeypatch.setattr(BENCHMARK_PROVENANCE_MODULE.subprocess, 'run', fake_run)

    with pytest.raises(RuntimeError, match='dubious ownership'):
        BENCHMARK_PROVENANCE_MODULE._git_state(tmp_path)


def _write_binary_xyz_pcd(path, points):
    header = '\n'.join([
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        'FIELDS x y z',
        'SIZE 4 4 4',
        'TYPE F F F',
        'COUNT 1 1 1',
        f'WIDTH {len(points)}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {len(points)}',
        'DATA binary',
        '',
    ])
    payload = b''.join(struct.pack('<fff', *point) for point in points)
    path.write_bytes(header.encode('ascii') + payload)


def _create_map_bundle(root: Path) -> None:
    pointcloud_dir = root / 'pointcloud_map'
    pointcloud_dir.mkdir(parents=True, exist_ok=True)
    tile_name = '0_0.pcd'
    _write_binary_xyz_pcd(
        pointcloud_dir / tile_name,
        [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0)],
    )
    metadata = {
        'x_resolution': 20,
        'y_resolution': 20,
        tile_name: [0, 0],
    }
    (pointcloud_dir / 'pointcloud_map_metadata.yaml').write_text(
        yaml.safe_dump(metadata, sort_keys=False),
        encoding='utf-8',
    )
    (root / 'map_projector_info.yaml').write_text(
        yaml.safe_dump(
            {'projector_type': 'Local', 'scale_factor': 1.0},
            sort_keys=False,
        ),
        encoding='utf-8',
    )


def test_reference_parser_derives_existing_prism_offset(tmp_path):
    """The helper should reproduce the prism offset used in prior sweeps."""
    prism_yaml = tmp_path / 'leica_prism.yaml'
    prism_yaml.write_text(
        '\n'.join([
            '%YAML:1.0',
            '',
            'T_Body2Imu: !!opencv-matrix',
            '   rows: 4',
            '   cols: 4',
            '   dt: d',
            '   data: [ 1.0, 0.0, 0.0, -0.293656,',
            '           0.0, 1.0, 0.0, -0.012288,',
            '           0.0, 0.0, 1.0, -0.273095,',
            '           0.0, 0.0, 0.0, 1.0 ]',
        ]),
        encoding='utf-8',
    )
    rko_yaml = tmp_path / 'rko.yaml'
    rko_yaml.write_text(
        yaml.safe_dump(
            {
                'extrinsic_lidar2base_quat_xyzw_xyz': [
                    0.0, 0.0, 0.0, 1.0, 0.05, 0.0, -0.055,
                ]
            },
            sort_keys=False,
        ),
        encoding='utf-8',
    )

    body_to_imu = REFERENCE_MODULE.parse_opencv_matrix_translation(
        prism_yaml,
        'T_Body2Imu',
    )
    lidar_to_base = REFERENCE_MODULE.parse_lidar_to_base_translation(
        rko_yaml,
    )
    prism_offset = REFERENCE_MODULE.derive_prism_offset(
        body_to_imu,
        lidar_to_base,
    )

    assert body_to_imu == (-0.293656, -0.012288, -0.273095)
    assert lidar_to_base == (0.05, 0.0, -0.055)
    assert all(
        math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-12)
        for actual, expected in zip(
            prism_offset,
            (-0.243656, -0.012288, -0.328095),
        )
    )


def test_metrics_writer_prefers_generic_reference_offset():
    metadata = {
        'imu_to_reference_translation_m': {'x': 1, 'y': 2, 'z': 3},
        'imu_to_prism_translation_m': {'x': 9, 'y': 9, 'z': 9},
    }
    assert WRITE_METRICS_MODULE._trajectory_offset(metadata, 'imu') == {
        'x': 1, 'y': 2, 'z': 3}
    assert WRITE_METRICS_MODULE._infer_reference_kind(
        'rtk_slam_construction_seq1_gt', {}) == 'ground_truth'


def test_write_rko_lio_metrics_generates_compatible_metrics_json(tmp_path):
    """The metrics writer should emit a report-consumable metrics.json."""
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    (bag_dir / 'metadata.yaml').write_text(
        '\n'.join([
            'rosbag2_bagfile_information:',
            '  storage_identifier: sqlite3',
            '  duration:',
            '    nanoseconds: 2000000000',
            '  topics_with_message_count:',
            '    - topic_metadata:',
            '        name: /os1_cloud_node1/points',
            '      message_count: 2',
        ]),
        encoding='utf-8',
    )
    (bag_dir / 'data.db3').write_bytes(b'synthetic bag payload')
    out_dir = tmp_path / 'bench'
    out_dir.mkdir()
    _create_map_bundle(out_dir)

    raw_tum = out_dir / 'traj_raw_prism.tum'
    corrected_tum = out_dir / 'traj_corrected_prism.tum'
    reference_tum = tmp_path / 'reference.tum'
    reference_tum.write_text(
        '1.0 0 0 0 0 0 0 1\n'
        '2.0 1 0 0 0 0 0 1\n',
        encoding='utf-8',
    )
    raw_tum.write_text(reference_tum.read_text(encoding='utf-8'), encoding='utf-8')
    corrected_tum.write_text(reference_tum.read_text(encoding='utf-8'), encoding='utf-8')

    raw_ape = out_dir / 'ape_raw_vs_gt.txt'
    corrected_ape = out_dir / 'ape_corrected_vs_gt.txt'
    raw_ape.write_text(
        '\n'.join([
            'APE translation (m)',
            'pairs: 2',
            'alignment: se3_umeyama',
            'rmse: 0.2',
            'median: 0.1',
            'max: 0.3',
        ]) + '\n',
        encoding='utf-8',
    )
    corrected_ape.write_text(
        '\n'.join([
            'APE translation (m)',
            'pairs: 2',
            'alignment: se3_umeyama',
            'rmse: 0.1',
            'median: 0.05',
            'max: 0.2',
        ]) + '\n',
        encoding='utf-8',
    )
    reference_meta = tmp_path / 'reference.json'
    reference_meta.write_text(
        json.dumps(
            {
                'source': 'leica_prism_gt',
                'topic': '/leica/pose/relative',
                'source_bag': '/tmp/source_bag',
                'body_to_prism_translation_m': {
                    'x': -0.293656,
                    'y': -0.012288,
                    'z': -0.273095,
                },
                'lidar_to_prism_translation_m': {
                    'x': -0.243656,
                    'y': -0.012288,
                    'z': -0.328095,
                },
            },
        ) + '\n',
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            'python3',
            str(WRITE_METRICS_SCRIPT),
            '--out-dir',
            str(out_dir),
            '--bag',
            str(bag_dir),
            '--reference-tum',
            str(reference_tum),
            '--reference-meta',
            str(reference_meta),
            '--trajectory-source-frame',
            'body',
            '--wall-sec',
            '1.0',
            '--completion-reason',
            'offline_completion_marker',
            '--completion-end-margin-secs',
            '0.25',
            '--runtime-artifact',
            'test_runtime=/bin/true',
            '--benchmark-harness',
            str(WRITE_METRICS_SCRIPT),
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    metrics_path = out_dir / 'metrics.json'
    assert metrics_path.is_file()
    metrics = json.loads(metrics_path.read_text(encoding='utf-8'))
    schema = json.loads(
        (REPO_ROOT / 'docs/schemas/benchmark-metrics-v1.schema.json').read_text(
            encoding='utf-8'))
    jsonschema.validate(metrics, schema)

    assert metrics['reference']['source'] == 'leica_prism_gt'
    assert metrics['lidarslam']['success'] is True
    assert metrics['lidarslam']['tum_path'] == str(corrected_tum)
    assert metrics['evo']['ape']['rmse'] == 0.1
    assert metrics['evo']['raw_ape']['rmse'] == 0.2
    assert metrics['graph_based_slam']['map_verify']['ok'] is True
    assert metrics.get('pipeline') == 'rko_lio'
    assert metrics['rko_lio']['available'] is True
    assert metrics['rko_lio']['trajectory_source_frame'] == 'body'
    assert metrics['rko_lio']['prism_offset_m']['x'] == -0.293656
    assert metrics['schema_version'] == 1
    assert metrics['completion']['reason'] == 'offline_completion_marker'
    assert metrics['completion']['input_points_messages'] == 2
    assert metrics['completion']['raw_output_pose_count'] == 2
    assert metrics['completion']['raw_output_pose_ratio'] == 1.0
    assert metrics['reference']['kind'] == 'ground_truth'
    assert metrics['provenance']['input']['bag']['identity_algorithm'] == 'sha256'
    assert metrics['provenance']['software']['runtime_artifacts'][0][
        'label'] == 'test_runtime'

    skipped_metrics = out_dir / 'metrics_skip_map.json'
    skipped = subprocess.run(
        [
            'python3',
            str(WRITE_METRICS_SCRIPT),
            '--out-dir',
            str(out_dir),
            '--bag',
            str(bag_dir),
            '--reference-tum',
            str(reference_tum),
            '--reference-meta',
            str(reference_meta),
            '--trajectory-source-frame',
            'body',
            '--metrics-out',
            str(skipped_metrics),
            '--skip-map-verify',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )
    assert skipped.returncode == 0, skipped.stderr
    trajectory_only = json.loads(skipped_metrics.read_text(encoding='utf-8'))
    graph = trajectory_only['graph_based_slam']
    assert graph['map_verify'] is None
    assert graph['pointcloud_map_dir'] == ''
    assert graph['map_projector_info_path'] == ''


def test_write_lo_metrics_sets_scanmatcher_payload(tmp_path):
    """LO pipeline metrics should tag scanmatcher_lo and disable rko_lio."""
    bag_dir = tmp_path / 'bag'
    bag_dir.mkdir()
    (bag_dir / 'metadata.yaml').write_text(
        '\n'.join([
            'rosbag2_bagfile_information:',
            '  duration:',
            '    nanoseconds: 1000000000',
        ]),
        encoding='utf-8',
    )
    out_dir = tmp_path / 'lo_bench'
    out_dir.mkdir()
    _create_map_bundle(out_dir)

    raw_tum = out_dir / 'traj_raw_prism.tum'
    corrected_tum = out_dir / 'traj_corrected_prism.tum'
    reference_tum = tmp_path / 'reference.tum'
    reference_tum.write_text('1.0 0 0 0 0 0 0 1\n', encoding='utf-8')
    raw_tum.write_text(reference_tum.read_text(encoding='utf-8'), encoding='utf-8')
    corrected_tum.write_text(reference_tum.read_text(encoding='utf-8'), encoding='utf-8')

    raw_ape = out_dir / 'ape_raw_vs_gt.txt'
    corrected_ape = out_dir / 'ape_corrected_vs_gt.txt'
    for target in (raw_ape, corrected_ape):
        target.write_text(
            '\n'.join([
                'APE translation (m)',
                'pairs: 1',
                'alignment: se3_umeyama',
                'rmse: 0.42',
            ]) + '\n',
            encoding='utf-8',
        )

    lidarslam_yaml = tmp_path / 'lidarslam_lo.yaml'
    lidarslam_yaml.write_text('scan_matcher:\n  ros__parameters: {}\n', encoding='utf-8')
    reference_meta = tmp_path / 'ref_lo.json'
    reference_meta.write_text(
        json.dumps({'lidar_to_prism_translation_m': {'x': 0.0, 'y': 0.0, 'z': 0.0}}) + '\n',
        encoding='utf-8',
    )

    result = subprocess.run(
        [
            'python3',
            str(WRITE_METRICS_SCRIPT),
            '--pipeline',
            'lo',
            '--out-dir',
            str(out_dir),
            '--bag',
            str(bag_dir),
            '--reference-tum',
            str(reference_tum),
            '--reference-meta',
            str(reference_meta),
            '--lidarslam-param',
            str(lidarslam_yaml),
            '--rko-param',
            str(lidarslam_yaml),
            '--robot-frame-id',
            'velodyne',
            '--reference-source',
            'test_lo',
        ],
        capture_output=True,
        text=True,
        check=False,
        cwd=REPO_ROOT,
    )

    assert result.returncode == 0, result.stderr
    metrics = json.loads((out_dir / 'metrics.json').read_text(encoding='utf-8'))
    assert metrics['pipeline'] == 'lo'
    assert metrics['rko_lio']['available'] is False
    assert metrics['scanmatcher_lo']['raw_ape']['rmse'] == 0.42
    assert metrics['frames']['robot_frame_id'] == 'velodyne'
