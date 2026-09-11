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

"""Tests for MID-360 browser 3D map previews."""

from __future__ import annotations

import json
from pathlib import Path
import struct
import subprocess
import sys

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
SCRIPT_PATH = SCRIPT_DIR / 'export_mid360_robot_3d_map_preview.py'
sys.path.insert(0, str(SCRIPT_DIR))

from mid360_robot_3d_map_preview import (  # noqa: E402
    MAP_PREVIEW_HTML,
    MAP_PREVIEW_JSON,
    MAP_PREVIEW_OVERLAY_JSON,
    MAP_PREVIEW_PLY,
    MapPreviewOptions,
    Mid360MapPreviewExporter,
)

import yaml  # noqa: E402


def _lzf_literal_payload(data: bytes) -> bytes:
    payload = bytearray()
    for offset in range(0, len(data), 32):
        chunk = data[offset:offset + 32]
        payload.append(len(chunk) - 1)
        payload.extend(chunk)
    return bytes(payload)


def _write_binary_compressed_xyz_pcd(
    path: Path,
    points: list[tuple[float, float, float]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
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
        'DATA binary_compressed',
        '',
    ])
    uncompressed = b''.join(
        struct.pack('<f', point[axis])
        for axis in range(3)
        for point in points
    )
    compressed = _lzf_literal_payload(uncompressed)
    path.write_bytes(
        header.encode('ascii') + struct.pack('<II', len(compressed), len(uncompressed))
        + compressed
    )


def _write_fixture_run(run_dir: Path) -> Path:
    points = [
        (float(x), float(y), float((x + y) % 3) * 0.1)
        for x in range(-3, 4)
        for y in range(-3, 4)
    ]
    map_dir = run_dir / 'pointcloud_map'
    map_dir.mkdir(parents=True)
    (map_dir / 'pointcloud_map_metadata.yaml').write_text(
        yaml.safe_dump({
            'x_resolution': 20,
            'y_resolution': 20,
            '0_0.pcd': [0, 0],
        }),
        encoding='utf-8',
    )
    _write_binary_compressed_xyz_pcd(map_dir / '0_0.pcd', points)
    _write_binary_compressed_xyz_pcd(run_dir / 'map.pcd', points)

    trajectory = run_dir / 'trajectory_tum.txt'
    trajectory.write_text(
        '\n'.join([
            '0.0 -3.0 0.0 0.0 0 0 0 1',
            '1.0 -1.0 0.0 0.0 0 0 0 1',
            '2.0 1.0 0.0 0.0 0 0 0 1',
            '3.0 3.0 0.0 0.0 0 0 0 1',
        ]) + '\n',
        encoding='utf-8',
    )
    (run_dir / 'mid360_robot_loop_alignment.json').write_text(
        json.dumps({
            'status': 'PASS',
            'loop_candidates': [
                {
                    'start_index': 0,
                    'end_index': 3,
                    'distance_m': 0.2,
                    'midpoint': [0.0, 0.0, 0.0],
                }
            ],
        }),
        encoding='utf-8',
    )
    (run_dir / 'pose_graph.g2o').write_text(
        '\n'.join(
            f'VERTEX_SE3:QUAT {index} {x} 0 0 0 0 0 1'
            for index, x in enumerate((-3, -1, 1, 3))
        ) + '\n',
        encoding='utf-8',
    )
    (run_dir / 'loop_edges.csv').write_text(
        'from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n'
        '0,3,0.2,6,0,0,0,0,0,1\n',
        encoding='utf-8',
    )
    (run_dir / 'map_projector_info.yaml').write_text(
        'projector_type: Local\n', encoding='utf-8'
    )
    (run_dir / 'map_bundle.yaml').write_text(
        yaml.safe_dump({
            'format_version': 1,
            'frame_id': 'map',
            'submap_count': 4,
            'loop_edge_count': 1,
            'artifacts': {
                'full_map': 'map.pcd',
                'pointcloud_map': 'pointcloud_map',
                'trajectory': 'trajectory_tum.txt',
                'pose_graph': 'pose_graph.g2o',
                'loop_edges': 'loop_edges.csv',
                'projector_info': 'map_projector_info.yaml',
            },
        }, sort_keys=False),
        encoding='utf-8',
    )
    return trajectory


def test_map_preview_exporter_writes_browser_artifacts(tmp_path: Path):
    run_dir = tmp_path / 'run'
    trajectory = _write_fixture_run(run_dir)
    output_dir = tmp_path / 'preview'

    manifest = Mid360MapPreviewExporter().export(
        MapPreviewOptions(
            run_dir=run_dir,
            trajectory_path=trajectory,
            output_dir=output_dir,
            max_points=100,
            html_max_points=25,
        )
    )

    assert manifest['status'] == 'PASS'
    assert manifest['counts']['cloud_points'] == 49
    assert manifest['counts']['html_points'] == 25
    assert manifest['counts']['trajectory_poses'] == 4
    assert manifest['counts']['loop_candidates'] == 1
    assert manifest['counts']['accepted_loop_edges'] == 1
    assert manifest['edit']['source']['editable'] is True
    assert len(manifest['edit']['source']['map_bundle_sha256']) == 64
    assert len(manifest['edit']['source']['full_map_sha256']) == 64
    assert len(manifest['edit']['source']['pointcloud_map_sha256']) == 64
    assert (output_dir / MAP_PREVIEW_JSON).is_file()
    assert (output_dir / MAP_PREVIEW_HTML).is_file()
    assert (output_dir / MAP_PREVIEW_PLY).is_file()
    assert (output_dir / MAP_PREVIEW_OVERLAY_JSON).is_file()

    ply = (output_dir / MAP_PREVIEW_PLY).read_text(encoding='utf-8')
    html = (output_dir / MAP_PREVIEW_HTML).read_text(encoding='utf-8')
    overlay = json.loads((output_dir / MAP_PREVIEW_OVERLAY_JSON).read_text(encoding='utf-8'))

    assert 'element vertex 49' in ply
    assert 'lidar_slam 3D Map Review' in html
    assert 'Drag to rotate' in html
    assert 'Download edit plan' in html
    assert "type: 'remove_box'" in html
    assert "type: 'disable_loop_edge'" in html
    assert overlay['loop_candidates'][0]['midpoint'] == [0.0, 0.0, 0.0]
    assert overlay['accepted_loop_edges'][0]['from'] == 0
    assert overlay['accepted_loop_edges'][0]['to'] == 3


def test_map_preview_cli_exports_json_manifest(tmp_path: Path):
    run_dir = tmp_path / 'run'
    trajectory = _write_fixture_run(run_dir)
    output_dir = tmp_path / 'preview'

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_PATH),
            str(run_dir),
            '--trajectory',
            str(trajectory),
            '--output-dir',
            str(output_dir),
            '--max-points',
            '100',
            '--html-max-points',
            '20',
            '--json',
        ],
        check=True,
        capture_output=True,
        text=True,
        cwd=REPO_ROOT,
    )
    manifest = json.loads(result.stdout)

    assert manifest['status'] == 'PASS'
    assert manifest['counts']['html_points'] == 20
    assert Path(manifest['artifacts']['html']).name == MAP_PREVIEW_HTML
    assert (output_dir / MAP_PREVIEW_HTML).is_file()
