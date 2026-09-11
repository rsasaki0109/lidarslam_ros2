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

"""Contracts for verified multi-session map projects."""

from __future__ import annotations

import json
import math
from pathlib import Path
import struct
import subprocess
import sys

import jsonschema

import numpy as np
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
sys.path.insert(0, str(SCRIPT_DIR))

from map_edit import (  # noqa: E402,I100
    _binary_records,
    _retile_full_map,
    _write_binary_pcd,
    PcdLayout,
)

from map_merge import (  # noqa: E402
    MapMergeError,
    merge_map_sessions,
    MergeOptions,
)

from mid360_robot_3d_map_preview import (  # noqa: E402
    MAP_PREVIEW_OVERLAY_JSON,
    MapPreviewOptions,
    Mid360MapPreviewExporter,
)

from verify_autoware_map import MapVerifier  # noqa: E402

from verify_map_bundle import BundleVerifier  # noqa: E402


PROJECT_SCHEMA = REPO_ROOT / 'docs/schemas/map-project-v1.schema.json'
RECEIPT_SCHEMA = REPO_ROOT / 'docs/schemas/map-merge-receipt-v1.schema.json'
CLI = SCRIPT_DIR / 'merge_map_sessions.py'


def _points() -> np.ndarray:
    values = []
    for x_index in range(8):
        for y_index in range(5):
            if x_index >= 5 and y_index >= 3:
                continue
            x = float(x_index) + 0.13
            y = float(y_index) + 0.17
            values.append((x, y, 0.05 * x + 0.12 * y))
    values.extend([(8.63, 0.42, 0.6), (2.38, 5.67, 0.4), (-0.62, 1.67, -0.1)])
    return np.asarray(values, dtype=np.float64)


def _rotation(yaw: float) -> np.ndarray:
    return np.asarray([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0],
    ])


def _layout() -> PcdLayout:
    return PcdLayout(
        fields=['x', 'y', 'z', 'intensity'],
        sizes=[4, 4, 4, 4],
        types=['F', 'F', 'F', 'F'],
        counts=[1, 1, 1, 1],
        points=0,
        data='binary',
        header_bytes=0,
    )


def _records(points: np.ndarray, intensity_offset: float) -> list[bytes]:
    return [
        struct.pack(
            '<ffff',
            float(point[0]), float(point[1]), float(point[2]),
            float(index) + intensity_offset,
        )
        for index, point in enumerate(points)
    ]


def _yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def _write_bundle(
    root: Path,
    points: np.ndarray,
    *,
    trajectory_points: np.ndarray,
    yaw: float,
    intensity_offset: float,
    projector_origin: float = 0.0,
) -> Path:
    root.mkdir()
    _write_binary_pcd(root / 'map.pcd', _layout(), _records(points, intensity_offset))
    pointcloud_dir = root / 'pointcloud_map'
    pointcloud_dir.mkdir()
    (pointcloud_dir / 'pointcloud_map_metadata.yaml').write_text(
        'x_resolution: 20.0\ny_resolution: 20.0\n', encoding='utf-8'
    )
    quaternion = _yaw_quaternion(yaw)
    trajectory = []
    for index, point in enumerate(trajectory_points):
        orientation = ' '.join(str(value) for value in quaternion)
        trajectory.append(
            f'{index + 1}.0 {point[0]} {point[1]} {point[2]} {orientation}'
        )
    (root / 'trajectory_optimized.tum').write_text(
        '\n'.join(trajectory) + '\n', encoding='utf-8'
    )
    pose_lines = [
        f'VERTEX_SE3:QUAT {index} ' + ' '.join(line.split()[1:])
        for index, line in enumerate(trajectory)
    ]
    (root / 'pose_graph.g2o').write_text('\n'.join(pose_lines) + '\n', encoding='utf-8')
    (root / 'loop_edges.csv').write_text(
        'from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n', encoding='utf-8'
    )
    (root / 'map_projector_info.yaml').write_text(
        'projector_type: Local\nmap_origin:\n'
        f'  latitude: {projector_origin}\n  longitude: 0.0\n',
        encoding='utf-8',
    )
    manifest = {
        'format_version': 1,
        'frame_id': 'map',
        'submap_count': len(trajectory),
        'loop_edge_count': 0,
        'map_leaf_size_m': 0.2,
        'grid_size_x_m': 20.0,
        'grid_size_y_m': 20.0,
        'artifacts': {
            'full_map': 'map.pcd',
            'pointcloud_map': 'pointcloud_map',
            'trajectory': 'trajectory_optimized.tum',
            'pose_graph': 'pose_graph.g2o',
            'loop_edges': 'loop_edges.csv',
            'projector_info': 'map_projector_info.yaml',
        },
    }
    (root / 'map_bundle.yaml').write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding='utf-8'
    )
    _retile_full_map(root)
    assert BundleVerifier(root).run()
    assert MapVerifier(root / 'pointcloud_map', check_bounds=True).run()
    return root


def _fixture(tmp_path: Path) -> tuple[Path, Path, np.ndarray, np.ndarray]:
    local = _points()
    rotation = _rotation(0.35)
    translation = np.asarray([3.2, 2.4, 0.25])
    anchor_points = local @ rotation.T + translation
    local_trajectory = np.asarray([
        [0.13, 0.17, 0.02],
        [3.13, 1.17, 0.30],
        [7.13, 3.17, 0.74],
    ])
    anchor_trajectory = local_trajectory @ rotation.T + translation
    anchor = _write_bundle(
        tmp_path / 'anchor',
        anchor_points,
        trajectory_points=anchor_trajectory,
        yaw=0.35,
        intensity_offset=100.0,
    )
    session = _write_bundle(
        tmp_path / 'session',
        local,
        trajectory_points=local_trajectory,
        yaw=0.0,
        intensity_offset=200.0,
    )
    return anchor, session, rotation, translation


def _options() -> MergeOptions:
    return MergeOptions(
        merge_voxel_size_m=0.20,
        alignment_voxel_size_m=0.01,
        max_alignment_points=1000,
        icp_trim_fraction=1.0,
        icp_yaw_samples=72,
        max_overlap_median_m=0.05,
        max_overlap_p90_m=0.05,
        min_overlap_within_1m=1.0,
    )


def test_merge_aligns_preserves_fields_and_publishes_verified_project(tmp_path: Path):
    anchor, session, expected_rotation, expected_translation = _fixture(tmp_path)
    candidate = tmp_path / 'project'

    receipt = merge_map_sessions(
        source_dirs=[anchor, session],
        output_dir=candidate,
        options=_options(),
    )

    assert receipt['status'] == 'PASS'
    assert receipt['session_count'] == 2
    assert receipt['duplicate_points_removed'] >= len(_points()) - 2
    assert receipt['validation']['map_bundle'] == 'PASS'
    assert receipt['validation']['autoware_pointcloud_map'] == 'PASS'
    assert BundleVerifier(candidate).run()
    assert MapVerifier(candidate / 'pointcloud_map', check_bounds=True).run()
    layout, records = _binary_records(candidate / 'map.pcd')
    assert layout.fields == ['x', 'y', 'z', 'intensity']
    assert records
    project = json.loads((candidate / 'map_project.json').read_text())
    alignment = project['transforms_to_anchor'][1]
    assert np.asarray(alignment['rotation']) == pytest.approx(expected_rotation, abs=1e-5)
    assert np.asarray(alignment['translation']) == pytest.approx(expected_translation, abs=1e-5)
    assert len(list((candidate / 'sessions').glob('*/trajectory_transformed.tum'))) == 2
    jsonschema.Draft7Validator(
        json.loads(PROJECT_SCHEMA.read_text(encoding='utf-8')),
        format_checker=jsonschema.FormatChecker(),
    ).validate(project)
    jsonschema.Draft7Validator(
        json.loads(RECEIPT_SCHEMA.read_text(encoding='utf-8')),
        format_checker=jsonschema.FormatChecker(),
    ).validate(receipt)
    preview_dir = tmp_path / 'preview'
    preview = Mid360MapPreviewExporter().export(
        MapPreviewOptions(run_dir=candidate, output_dir=preview_dir)
    )
    overlay = json.loads(
        (preview_dir / MAP_PREVIEW_OVERLAY_JSON).read_text(encoding='utf-8')
    )
    assert preview['counts']['map_sessions'] == 2
    assert len(overlay['session_trajectories']) == 2
    assert all(item['trajectory'] for item in overlay['session_trajectories'])


def test_merge_dry_run_performs_alignment_without_writing(tmp_path: Path):
    anchor, session, _, _ = _fixture(tmp_path)
    candidate = tmp_path / 'dry-project'

    receipt = merge_map_sessions(
        source_dirs=[anchor, session],
        output_dir=candidate,
        options=_options(),
        dry_run=True,
    )

    assert receipt['status'] == 'DRY_RUN'
    assert len(receipt['alignments']) == 2
    assert not candidate.exists()


def test_merge_rejects_projector_mismatch_before_output(tmp_path: Path):
    anchor, session, _, _ = _fixture(tmp_path)
    (session / 'map_projector_info.yaml').write_text(
        'projector_type: Local\nmap_origin:\n  latitude: 35.0\n  longitude: 0.0\n',
        encoding='utf-8',
    )
    candidate = tmp_path / 'bad-project'

    with pytest.raises(MapMergeError, match='projector'):
        merge_map_sessions(
            source_dirs=[anchor, session],
            output_dir=candidate,
            options=_options(),
        )

    assert not candidate.exists()


def test_merge_never_overwrites_output(tmp_path: Path):
    anchor, session, _, _ = _fixture(tmp_path)
    candidate = tmp_path / 'existing'
    candidate.mkdir()

    with pytest.raises(MapMergeError, match='already exists'):
        merge_map_sessions(
            source_dirs=[anchor, session],
            output_dir=candidate,
            options=_options(),
        )


def test_merge_cli_json_is_clean_and_source_preserving(tmp_path: Path):
    anchor, session, _, _ = _fixture(tmp_path)
    candidate = tmp_path / 'cli-project'
    before = (anchor / 'map.pcd').read_bytes(), (session / 'map.pcd').read_bytes()

    completed = subprocess.run(
        [
            sys.executable,
            str(CLI),
            str(anchor),
            str(session),
            '--output-dir',
            str(candidate),
            '--alignment-voxel-size',
            '0.01',
            '--max-alignment-points',
            '1000',
            '--max-median-error',
            '0.05',
            '--max-p90-error',
            '0.05',
            '--min-overlap',
            '1.0',
            '--json',
        ],
        check=False,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )

    assert completed.returncode == 0, completed.stderr
    assert json.loads(completed.stdout)['status'] == 'PASS'
    assert completed.stderr == ''
    assert before == (
        (anchor / 'map.pcd').read_bytes(),
        (session / 'map.pcd').read_bytes(),
    )
