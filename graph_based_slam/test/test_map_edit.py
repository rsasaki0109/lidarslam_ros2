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

"""Tests for non-destructive browser map edits."""

from __future__ import annotations

import json
from pathlib import Path
import struct
import subprocess
import sys

import jsonschema
import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'scripts'
PLAN_SCHEMA = REPO_ROOT / 'docs/schemas/map-edit-plan-v1.schema.json'
RECEIPT_SCHEMA = REPO_ROOT / 'docs/schemas/map-edit-receipt-v1.schema.json'
sys.path.insert(0, str(SCRIPT_DIR))

import map_edit as MAP_EDIT  # noqa: E402,I100
from map_edit import (  # noqa: E402
    _binary_records,
    _xyz_from_record,
    accepted_loop_edges,
    apply_map_edit,
    EDIT_RECEIPT_NAME,
    MapEditError,
    sha256_directory,
    sha256_file,
)


def _lzf_literal_payload(data: bytes) -> bytes:
    payload = bytearray()
    for offset in range(0, len(data), 32):
        chunk = data[offset:offset + 32]
        payload.append(len(chunk) - 1)
        payload.extend(chunk)
    return bytes(payload)


def _write_pcd(path: Path, points: list[tuple[float, float, float, float]]) -> None:
    header = '\n'.join([
        '# .PCD v0.7 - Point Cloud Data file format',
        'VERSION 0.7',
        'FIELDS x y z intensity',
        'SIZE 4 4 4 4',
        'TYPE F F F F',
        'COUNT 1 1 1 1',
        f'WIDTH {len(points)}',
        'HEIGHT 1',
        'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {len(points)}',
        'DATA binary_compressed',
        '',
    ])
    raw = b''.join(
        struct.pack('<f', point[field])
        for field in range(4)
        for point in points
    )
    compressed = _lzf_literal_payload(raw)
    payload = b''.join([
        header.encode('ascii'),
        struct.pack('<II', len(compressed), len(raw)),
        compressed,
    ])
    path.write_bytes(payload)


def _write_bundle(root: Path) -> Path:
    root.mkdir()
    pointcloud = root / 'pointcloud_map'
    pointcloud.mkdir()
    points = [
        (0.0, 0.0, 0.0, 11.0),
        (1.0, 1.0, 1.0, 22.0),
        (2.0, 2.0, 2.0, 33.0),
    ]
    _write_pcd(root / 'map.pcd', points)
    _write_pcd(pointcloud / '0_0.pcd', points)
    (pointcloud / 'pointcloud_map_metadata.yaml').write_text(
        yaml.safe_dump({
            'x_resolution': 20,
            'y_resolution': 20,
            '0_0.pcd': [0, 0],
        }, sort_keys=False),
        encoding='utf-8',
    )
    (root / 'trajectory_optimized.tum').write_text(
        '0 0 0 0 0 0 0 1\n1 2 2 0 0 0 0 1\n',
        encoding='utf-8',
    )
    (root / 'pose_graph.g2o').write_text(
        'VERTEX_SE3:QUAT 0 0 0 0 0 0 0 1\n'
        'VERTEX_SE3:QUAT 1 2 2 0 0 0 0 1\n',
        encoding='utf-8',
    )
    (root / 'loop_edges.csv').write_text(
        'from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n'
        '0,1,0.1,2,2,0,0,0,0,1\n',
        encoding='utf-8',
    )
    (root / 'map_projector_info.yaml').write_text(
        'projector_type: Local\n', encoding='utf-8'
    )
    (root / 'map_bundle.yaml').write_text(
        yaml.safe_dump({
            'format_version': 1,
            'frame_id': 'map',
            'submap_count': 2,
            'loop_edge_count': 1,
            'artifacts': {
                'full_map': 'map.pcd',
                'pointcloud_map': 'pointcloud_map',
                'trajectory': 'trajectory_optimized.tum',
                'pose_graph': 'pose_graph.g2o',
                'loop_edges': 'loop_edges.csv',
                'projector_info': 'map_projector_info.yaml',
            },
        }, sort_keys=False),
        encoding='utf-8',
    )
    return root


def _write_plan(
    path: Path,
    bundle: Path,
    operations: list[dict[str, object]],
) -> Path:
    path.write_text(
        json.dumps({
            'schema_version': 1,
            'source': {
                'map_bundle_sha256': sha256_file(bundle / 'map_bundle.yaml'),
                'full_map_sha256': sha256_file(bundle / 'map.pcd'),
                'pointcloud_map_sha256': sha256_directory(bundle / 'pointcloud_map'),
                'loop_edges_sha256': sha256_file(bundle / 'loop_edges.csv'),
            },
            'operations': operations,
        }),
        encoding='utf-8',
    )
    return path


def test_remove_box_creates_verified_candidate_and_preserves_fields(tmp_path: Path):
    """A box edit must preserve fields and publish only a verified candidate."""
    source = _write_bundle(tmp_path / 'source')
    plan = _write_plan(
        tmp_path / 'plan.json',
        source,
        [{
            'id': 'remove-origin',
            'type': 'remove_box',
            'min_xyz': [-0.5, -0.5, -0.5],
            'max_xyz': [0.5, 0.5, 0.5],
        }],
    )
    candidate = tmp_path / 'candidate'

    receipt = apply_map_edit(
        source_dir=source,
        plan_path=plan,
        output_dir=candidate,
    )

    assert receipt['status'] == 'PASS'
    jsonschema.validate(
        json.loads(plan.read_text(encoding='utf-8')),
        json.loads(PLAN_SCHEMA.read_text(encoding='utf-8')),
    )
    jsonschema.validate(
        receipt,
        json.loads(RECEIPT_SCHEMA.read_text(encoding='utf-8')),
    )
    assert receipt['point_edit']['tile_points_before'] == 3
    assert receipt['point_edit']['tile_points_after'] == 2
    assert receipt['point_edit']['fields_preserved'] is True
    assert (candidate / EDIT_RECEIPT_NAME).is_file()
    assert (source / 'pointcloud_map' / '0_0.pcd').is_file()
    source_layout, source_records = _binary_records(source / 'map.pcd')
    result_layout, result_records = _binary_records(candidate / 'map.pcd')
    assert source_layout.fields == result_layout.fields == ['x', 'y', 'z', 'intensity']
    assert len(source_records) == 3
    assert [_xyz_from_record(result_layout, row) for row in result_records] == [
        (1.0, 1.0, 1.0),
        (2.0, 2.0, 2.0),
    ]
    assert struct.unpack_from('<f', result_records[0], 12)[0] == 22.0


def test_plan_identity_mismatch_fails_without_writing_output(tmp_path: Path):
    """A plan for another source map must fail before output is created."""
    source = _write_bundle(tmp_path / 'source')
    plan = _write_plan(
        tmp_path / 'plan.json',
        source,
        [{
            'id': 'remove-origin',
            'type': 'remove_box',
            'min_xyz': [-0.5, -0.5, -0.5],
            'max_xyz': [0.5, 0.5, 0.5],
        }],
    )
    with (source / 'map_bundle.yaml').open('a', encoding='utf-8') as stream:
        stream.write('changed: true\n')

    with pytest.raises(MapEditError, match='source mismatch'):
        apply_map_edit(
            source_dir=source,
            plan_path=plan,
            output_dir=tmp_path / 'candidate',
        )
    assert not (tmp_path / 'candidate').exists()


def test_candidate_cannot_be_created_inside_source_bundle(tmp_path: Path):
    """A nested destination must be refused before recursive staging starts."""
    source = _write_bundle(tmp_path / 'source')
    plan = _write_plan(
        tmp_path / 'plan.json',
        source,
        [{
            'id': 'remove-origin',
            'type': 'remove_box',
            'min_xyz': [-0.5, -0.5, -0.5],
            'max_xyz': [0.5, 0.5, 0.5],
        }],
    )

    with pytest.raises(MapEditError, match='must not be inside'):
        apply_map_edit(
            source_dir=source,
            plan_path=plan,
            output_dir=source / 'edited',
        )
    assert not (source / 'edited').exists()


def test_metadata_tile_path_cannot_escape_staging(tmp_path: Path):
    """A hostile metadata key must not read, rewrite, or remove an external PCD."""
    source = _write_bundle(tmp_path / 'source')
    outside = tmp_path / 'outside.pcd'
    outside.write_bytes((source / 'map.pcd').read_bytes())
    outside_before = outside.read_bytes()
    metadata_path = source / 'pointcloud_map' / 'pointcloud_map_metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    metadata['../../../outside.pcd'] = [0, 0]
    metadata_path.write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8'
    )
    plan = _write_plan(
        tmp_path / 'plan.json',
        source,
        [{
            'id': 'remove-origin',
            'type': 'remove_box',
            'min_xyz': [-0.5, -0.5, -0.5],
            'max_xyz': [0.5, 0.5, 0.5],
        }],
    )

    with pytest.raises(MapEditError, match='unsafe tile path'):
        apply_map_edit(
            source_dir=source,
            plan_path=plan,
            output_dir=tmp_path / 'candidate',
        )

    assert outside.read_bytes() == outside_before
    assert not (tmp_path / 'candidate').exists()


def test_source_bundle_symlink_cannot_import_external_content(tmp_path: Path):
    """A source symlink must not be dereferenced into the edited bundle."""
    source = _write_bundle(tmp_path / 'source')
    external = tmp_path / 'private.txt'
    external.write_text('must stay outside\n', encoding='utf-8')
    (source / 'operator-note.txt').symlink_to(external)
    plan = _write_plan(
        tmp_path / 'plan.json',
        source,
        [{
            'id': 'remove-origin',
            'type': 'remove_box',
            'min_xyz': [-0.5, -0.5, -0.5],
            'max_xyz': [0.5, 0.5, 0.5],
        }],
    )

    with pytest.raises(
        MapEditError, match='source map bundle must not contain symlinks'
    ):
        apply_map_edit(
            source_dir=source,
            plan_path=plan,
            output_dir=tmp_path / 'candidate',
        )

    assert external.read_text(encoding='utf-8') == 'must stay outside\n'
    assert not (tmp_path / 'candidate').exists()


def test_geometry_change_invalidates_plan_even_when_manifest_is_unchanged(
    tmp_path: Path,
):
    """Source pinning must cover geometry, not only the YAML manifest."""
    source = _write_bundle(tmp_path / 'source')
    plan = _write_plan(
        tmp_path / 'plan.json',
        source,
        [{
            'id': 'remove-origin',
            'type': 'remove_box',
            'min_xyz': [-0.5, -0.5, -0.5],
            'max_xyz': [0.5, 0.5, 0.5],
        }],
    )
    with (source / 'map.pcd').open('ab') as stream:
        stream.write(b'changed-after-plan')

    with pytest.raises(MapEditError, match='full_map changed'):
        apply_map_edit(
            source_dir=source,
            plan_path=plan,
            output_dir=tmp_path / 'candidate',
        )
    assert not (tmp_path / 'candidate').exists()


def test_apply_cli_json_is_machine_readable(tmp_path: Path):
    """Automation mode must emit one receipt without verifier console noise."""
    source = _write_bundle(tmp_path / 'source')
    plan = _write_plan(
        tmp_path / 'plan.json',
        source,
        [{
            'id': 'remove-origin',
            'type': 'remove_box',
            'min_xyz': [-0.5, -0.5, -0.5],
            'max_xyz': [0.5, 0.5, 0.5],
        }],
    )
    candidate = tmp_path / 'candidate'

    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT_DIR / 'apply_map_edit.py'),
            str(source),
            '--plan', str(plan),
            '--output-dir', str(candidate),
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


def test_loop_disable_requires_backend_replay_and_names_next_inputs(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """Loop edits must fail closed when replay evidence is unavailable."""
    monkeypatch.delenv('ROS_DISTRO', raising=False)
    source = _write_bundle(tmp_path / 'source')
    plan = _write_plan(
        tmp_path / 'loop-plan.json',
        source,
        [{'id': 'disable-0-1', 'type': 'disable_loop_edge', 'from': 0, 'to': 1}],
    )

    with pytest.raises(MapEditError, match='deterministic backend replay') as error:
        apply_map_edit(
            source_dir=source,
            plan_path=plan,
            output_dir=tmp_path / 'candidate',
        )
    assert '--backend-input' in str(error.value)
    assert '--params' in str(error.value)
    assert '--setup' in str(error.value)
    assert not (tmp_path / 'candidate').exists()


def test_accepted_loops_are_read_from_csv_not_inferred(tmp_path: Path):
    """Only accepted CSV constraints may be offered as disable operations."""
    source = _write_bundle(tmp_path / 'source')

    loops = accepted_loop_edges(source)

    assert [(item['from'], item['to']) for item in loops] == [(0, 1)]
    assert loops[0]['fitness'] == pytest.approx(0.1)


def test_loop_disable_replays_then_retiles_and_verifies_candidate(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
):
    """A retained loop set must drive replay, retiling, and candidate validation."""
    source = _write_bundle(tmp_path / 'source')
    plan = _write_plan(
        tmp_path / 'loop-plan.json',
        source,
        [{'id': 'disable-0-1', 'type': 'disable_loop_edge', 'from': 0, 'to': 1}],
    )
    backend = source / 'backend_input'
    backend.mkdir()
    (backend / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: {}\n', encoding='utf-8'
    )
    params = source / 'graph_params.ros.yaml'
    params.write_text('/**:\n  ros__parameters: {}\n', encoding='utf-8')
    replay_script = tmp_path / 'run_offline_determinism_check.sh'
    replay_script.write_text('# test replay\n', encoding='utf-8')
    commands: list[list[str]] = []
    monkeypatch.setenv('ROS_DISTRO', 'jazzy')
    real_which = MAP_EDIT.shutil.which
    monkeypatch.setattr(
        MAP_EDIT.shutil,
        'which',
        lambda name: '/opt/ros/jazzy/bin/ros2' if name == 'ros2' else real_which(name),
    )

    def fake_run(command, **_kwargs):
        commands.append(command)
        replay_output = Path(command[command.index('--output-dir') + 1]) / 'run1'
        replay_output.mkdir(parents=True)
        (replay_output / 'map_optimized.pcd').write_bytes(
            (source / 'map.pcd').read_bytes()
        )
        (replay_output / 'trajectory_optimized.tum').write_text(
            (source / 'trajectory_optimized.tum').read_text(encoding='utf-8'),
            encoding='utf-8',
        )
        (replay_output / 'pose_graph.g2o').write_text(
            (source / 'pose_graph.g2o').read_text(encoding='utf-8'),
            encoding='utf-8',
        )
        (replay_output / 'loop_edges.csv').write_text(
            'from,to,fitness,tx,ty,tz,qx,qy,qz,qw\n', encoding='utf-8'
        )
        (replay_output / 'runner.log').write_text(
            'offline replay complete\n', encoding='utf-8'
        )
        (replay_output.parent / 'offline_determinism_summary.md').write_text(
            'edge_sets_identical: true\n', encoding='utf-8'
        )
        return MAP_EDIT.subprocess.CompletedProcess(command, 0, '', '')

    monkeypatch.setattr(MAP_EDIT.subprocess, 'run', fake_run)
    candidate = tmp_path / 'candidate'

    receipt = apply_map_edit(
        source_dir=source,
        plan_path=plan,
        output_dir=candidate,
        replay_script=replay_script,
    )

    assert receipt['status'] == 'PASS'
    assert receipt['loop_replay']['performed'] is True
    assert receipt['loop_replay']['accepted_edges_before'] == 1
    assert receipt['loop_replay']['accepted_edges_after'] == 0
    assert receipt['loop_replay']['backend_input'] == str(backend)
    assert receipt['loop_replay']['params_path'] == str(params)
    assert receipt['loop_replay']['setup_path'] == 'active_environment'
    assert '--save-maps' in commands[0]
    assert '--setup' not in commands[0]
    assert any(item.startswith('fixed_loop_edges_path:=') for item in commands[0])
    for item in receipt['loop_replay']['evidence'].values():
        assert (candidate / item['path']).is_file()
        assert len(item['sha256']) == 64
    assert yaml.safe_load((candidate / 'map_bundle.yaml').read_text())['loop_edge_count'] == 0
    assert (candidate / 'loop_edges.csv').read_text().count('\n') == 1
    metadata = yaml.safe_load(
        (candidate / 'pointcloud_map/pointcloud_map_metadata.yaml').read_text()
    )
    assert metadata['0_0.pcd'] == [0, 0]
