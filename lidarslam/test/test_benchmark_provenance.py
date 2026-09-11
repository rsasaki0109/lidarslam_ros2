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
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
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

"""Tests for reproducible benchmark input/software identities."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'benchmark_provenance.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'benchmark_provenance',
        SCRIPT_PATH,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_bag_identity_hashes_metadata_and_storage(tmp_path: Path):
    """Bag identity covers its metadata and referenced storage."""
    module = _load_module()
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'data.db3').write_bytes(b'bag payload')
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  storage_identifier: sqlite3\n'
        '  relative_file_paths:\n'
        '    - data.db3\n',
        encoding='utf-8',
    )

    identity = module.bag_identity(bag)

    assert identity['metadata']['path'] == 'metadata.yaml'
    assert len(identity['metadata']['sha256']) == 64
    assert identity['storage_files'][0]['path'] == 'data.db3'
    assert identity['storage_files'][0]['size_bytes'] == len(b'bag payload')
    assert len(identity['storage_files'][0]['sha256']) == 64


def test_bag_identity_rejects_path_traversal(tmp_path: Path):
    """A metadata path cannot escape the bag directory."""
    module = _load_module()
    bag = tmp_path / 'bag'
    bag.mkdir()
    (tmp_path / 'outside.db3').write_bytes(b'outside')
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  relative_file_paths:\n'
        '    - ../outside.db3\n',
        encoding='utf-8',
    )

    with pytest.raises(ValueError, match='outside the bag'):
        module.bag_identity(bag)


def test_file_identity_rejects_missing_artifact(tmp_path: Path):
    """Missing runtime evidence fails instead of producing a partial record."""
    module = _load_module()
    with pytest.raises(ValueError, match='provenance file is missing'):
        module.file_identity(tmp_path / 'missing')


def test_git_state_treats_untracked_build_input_as_dirty(tmp_path: Path):
    """Untracked source must invalidate a clean release-evidence claim."""
    module = _load_module()
    repo = tmp_path / 'repo'
    repo.mkdir()
    subprocess.run(['git', 'init', '-q'], cwd=repo, check=True)
    subprocess.run(
        ['git', 'config', 'user.email', 'test@example.com'],
        cwd=repo,
        check=True,
    )
    subprocess.run(
        ['git', 'config', 'user.name', 'Benchmark Test'],
        cwd=repo,
        check=True,
    )
    (repo / 'tracked.txt').write_text('tracked\n', encoding='utf-8')
    subprocess.run(['git', 'add', 'tracked.txt'], cwd=repo, check=True)
    subprocess.run(
        ['git', 'commit', '-q', '-m', 'fixture'],
        cwd=repo,
        check=True,
    )
    assert module._git_state(repo)['git_dirty'] is False

    (repo / 'untracked.cpp').write_text('untracked\n', encoding='utf-8')

    assert module._git_state(repo)['git_dirty'] is True


def test_metrics_writer_emits_schema_valid_provenance(tmp_path: Path):
    """The common writer emits a schema-valid, complete provenance record."""
    jsonschema = pytest.importorskip('jsonschema')
    bag = tmp_path / 'bag'
    bag.mkdir()
    (bag / 'data.db3').write_bytes(b'bag payload')
    (bag / 'metadata.yaml').write_text(
        'rosbag2_bagfile_information:\n'
        '  storage_identifier: sqlite3\n'
        '  duration:\n'
        '    nanoseconds: 2000000000\n'
        '  relative_file_paths:\n'
        '    - data.db3\n',
        encoding='utf-8',
    )
    trajectory = '\n'.join(
        (
            '0 0 0 0 0 0 0 1',
            '1 1 0 0 0 0 0 1',
            '2 2 0 0 0 0 0 1',
        )
    ) + '\n'
    reference = tmp_path / 'reference.tum'
    estimate = tmp_path / 'estimate.tum'
    reference.write_text(trajectory, encoding='utf-8')
    estimate.write_text(trajectory, encoding='utf-8')
    out_dir = tmp_path / 'out'

    subprocess.run(
        [
            'python3',
            str(REPO_ROOT / 'scripts' / 'write_aligned_trajectory_metrics.py'),
            '--out-dir', str(out_dir),
            '--bag', str(bag),
            '--reference-tum', str(reference),
            '--corrected-tum', str(estimate),
            '--lidarslam-param',
            str(REPO_ROOT / 'lidarslam' / 'param' / 'lidarslam.yaml'),
            '--runtime-artifact', 'true=/bin/true',
            '--benchmark-harness',
            str(REPO_ROOT / 'scripts' / 'write_aligned_trajectory_metrics.py'),
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    metrics = json.loads(
        (out_dir / 'metrics.json').read_text(encoding='utf-8')
    )
    schema = json.loads(
        (
            REPO_ROOT / 'docs' / 'schemas' / 'benchmark-metrics-v1.schema.json'
        ).read_text(encoding='utf-8')
    )
    jsonschema.validate(metrics, schema)
    assert (
        metrics['provenance']['input']['bag']['storage_files'][0]['sha256']
    )
    assert (
        metrics['provenance']['software']['runtime_artifacts'][0]['label']
        == 'true'
    )
