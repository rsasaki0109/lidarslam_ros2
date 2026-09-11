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

"""Tests for the version-pinned standalone Docker launcher asset."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'build_docker_launcher_asset.py'
SOURCE = ROOT / 'scripts' / 'docker_map_bag.sh'
SPEC = importlib.util.spec_from_file_location(
    'build_docker_launcher_asset',
    SCRIPT,
)
assert SPEC is not None and SPEC.loader is not None
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)
REVISION = 'a' * 40


def _build(tmp_path: Path, *, name: str = 'lidarslam-map-docker') -> Path:
    output = tmp_path / name
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--tag',
            'v0.9.1',
            '--source-revision',
            REVISION,
            '--output',
            str(output),
        ],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert 'SHA-256:' in result.stdout
    return output


def test_release_asset_is_executable_and_pins_versioned_images(tmp_path):
    asset = _build(tmp_path)
    assert asset.stat().st_mode & 0o111

    version = subprocess.run(
        [str(asset), '--version'],
        check=False,
        capture_output=True,
        text=True,
    )
    assert version.returncode == 0
    assert version.stdout == f'lidarslam-map-docker v0.9.1 ({REVISION})\n'

    bag = tmp_path / 'field bag'
    bag.mkdir()
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information: {}\n',
        encoding='utf-8',
    )
    dry_run = subprocess.run(
        [str(asset), '--dry-run', str(bag)],
        cwd=tmp_path,
        check=False,
        capture_output=True,
        text=True,
    )
    assert dry_run.returncode == 0, dry_run.stderr
    assert (
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.1-humble'
        in dry_run.stdout
    )
    assert not (tmp_path / 'lidarslam_output').exists()

    json_dry_run = subprocess.run(
        [str(asset), '--dry-run', '--json', str(bag)],
        cwd=tmp_path,
        check=False,
        capture_output=True,
        text=True,
    )
    assert json_dry_run.returncode == 0, json_dry_run.stderr
    plan = json.loads(json_dry_run.stdout)
    assert plan['schema_uri'].endswith(
        '/schemas/docker-map-bag-plan-v1.schema.json'
    )
    assert plan['launcher'] == {
        'version': 'v0.9.1',
        'revision': REVISION,
    }
    assert plan['image']['reference'] == (
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.1-humble'
    )
    assert plan['side_effects']['filesystem_writes'] is False
    assert not (tmp_path / 'lidarslam_output').exists()


def test_build_is_deterministic_and_create_only(tmp_path):
    first_dir = tmp_path / 'first'
    second_dir = tmp_path / 'second'
    first_dir.mkdir()
    second_dir.mkdir()
    first = _build(first_dir)
    second = _build(second_dir)
    assert first.read_bytes() == second.read_bytes()

    repeated = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--tag',
            'v0.9.1',
            '--source-revision',
            REVISION,
            '--output',
            str(first),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert repeated.returncode == 2
    assert 'output already exists' in repeated.stderr


@pytest.mark.parametrize(
    ('tag', 'revision', 'message'),
    [
        ('0.9.1', REVISION, 'tag must match'),
        ('v0.9.1', 'A' * 40, 'lowercase 40-character Git SHA'),
        ('v0.9.1', 'a' * 39, 'lowercase 40-character Git SHA'),
    ],
)
def test_invalid_release_identity_is_rejected(tag, revision, message):
    with pytest.raises(BUILDER.LauncherBuildError, match=message):
        BUILDER.render_launcher(SOURCE, tag=tag, revision=revision)


def test_marker_drift_and_symlink_source_are_rejected(tmp_path):
    drifted = tmp_path / 'drifted.sh'
    drifted.write_text(
        SOURCE.read_text(encoding='utf-8').replace(
            BUILDER.VERSION_MARKER,
            'LIDARSLAM_DOCKER_LAUNCHER_VERSION="changed"',
        ),
        encoding='utf-8',
    )
    with pytest.raises(BUILDER.LauncherBuildError, match='version marker'):
        BUILDER.render_launcher(drifted, tag='v0.9.1', revision=REVISION)

    linked = tmp_path / 'linked.sh'
    linked.symlink_to(SOURCE)
    with pytest.raises(BUILDER.LauncherBuildError, match='non-symlink'):
        BUILDER.render_launcher(linked, tag='v0.9.1', revision=REVISION)
