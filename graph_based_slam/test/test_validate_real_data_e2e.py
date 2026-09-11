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

"""Tests for the pinned real-data E2E evidence validator."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_PATH = REPO_ROOT / 'scripts' / 'validate_real_data_e2e.py'


def _load_module():
    spec = importlib.util.spec_from_file_location('real_data_e2e', SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_json(path: Path, payload: dict) -> Path:
    path.write_text(json.dumps(payload), encoding='utf-8')
    return path


def _fixture(tmp_path: Path) -> tuple[Path, Path, Path]:
    archive = tmp_path / 'sample.zip'
    archive.write_bytes(b'pinned-real-data-fixture')
    archive_md5 = hashlib.md5(archive.read_bytes()).hexdigest()
    archive_sha256 = hashlib.sha256(archive.read_bytes()).hexdigest()
    storage = {
        'path': 'sample.db3',
        'size_bytes': 123,
        'sha256': 'a' * 64,
    }
    source_url = 'https://example.test/records/1'
    contract = {
        'schema_version': 1,
        'id': 'fixture_v1',
        'dataset': {
            'id': 'fixture',
            'source_url': source_url,
            'file_id': 'bag',
            'filename': archive.name,
            'size_bytes': archive.stat().st_size,
            'md5': archive_md5,
            'sha256': archive_sha256,
        },
        'input': {
            'metadata_size_bytes': 50,
            'metadata_sha256': 'b' * 64,
            'storage_identifier': 'sqlite3',
            'storage_files': [storage],
            'duration_sec': 12.5,
            'duration_tolerance_sec': 0.001,
            'message_count': 30,
            'topics': {
                '/points': {
                    'type': 'sensor_msgs/msg/PointCloud2',
                    'message_count': 10,
                },
                '/imu': {
                    'type': 'sensor_msgs/msg/Imu',
                    'message_count': 20,
                },
            },
        },
        'execution': {
            'profile_id': 'fixture_profile',
            'ros_distro': 'jazzy',
            'required_argv_fragments': ['--skip-viewer', 'fixture.yaml'],
            'maximum_runtime_sec': 60,
        },
        'output': {
            'minimum_raw_poses': 2,
            'minimum_corrected_poses': 1,
            'minimum_pointcloud_tiles': 2,
            'minimum_pointcloud_bytes': 6,
            'minimum_verify_passes': 8,
        },
    }
    intake = {
        'status': 'READY',
        'archive_path': str(archive),
        'dataset': {'id': 'fixture', 'source_url': source_url},
        'file': {
            'id': 'bag',
            'filename': archive.name,
            'url': source_url + '/files/sample.zip',
        },
    }
    run_dir = tmp_path / 'run'
    pointcloud_dir = run_dir / 'pointcloud_map'
    pointcloud_dir.mkdir(parents=True)
    (pointcloud_dir / '0_0.pcd').write_bytes(b'abcd')
    (pointcloud_dir / '0_1.pcd').write_bytes(b'efgh')
    (run_dir / 'traj_raw.tum').write_text('raw1\nraw2\n', encoding='utf-8')
    (run_dir / 'traj_corrected.tum').write_text(
        'corrected\n',
        encoding='utf-8',
    )
    manifest = {
        'schema_version': 2,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/run-manifest-v2.schema.json'
        ),
        'run_id': '12345678-1234-5678-9234-567812345678',
        'status': 'succeeded',
        'lifecycle': {
            'stage': 'complete',
            'resume_count': 0,
            'verification_enabled': True,
            'runner_exit_code': 0,
            'last_error': None,
        },
        'input': {
            'bag_path': '/tmp/bag',
            'metadata_path': '/tmp/bag/metadata.yaml',
            'metadata_size_bytes': 50,
            'metadata_sha256': 'b' * 64,
            'storage_identifier': 'sqlite3',
            'storage_files': [storage],
            'identity_algorithm': 'sha256',
        },
        'software': {
            'product_version': '0.9.0',
            'git_commit': 'c' * 40,
            'git_dirty': False,
            'package_versions': {'lidarslam': '0.9.0'},
            'ros_distro': 'jazzy',
        },
        'profile': {'id': 'fixture_profile', 'label': 'Fixture'},
        'execution': {
            'argv': ['runner', '--skip-viewer', 'fixture.yaml'],
            'command_shell': 'runner --skip-viewer fixture.yaml',
            'started_at': '2026-07-27T00:00:00Z',
            'finished_at': '2026-07-27T00:00:10Z',
            'exit_code': 0,
        },
        'output': {
            'requested_dir': str(run_dir),
            'working_dir': str(run_dir) + '.partial',
            'finalized': True,
            'diagnosis_status': 'success',
            'artifact_checksums': [],
        },
    }
    diagnosis = {
        'schema_version': 1,
        'schema_uri': (
            'https://rsasaki0109.github.io/lidar_slam_ros2/'
            'schemas/diagnosis-v1.schema.json'
        ),
        'run_dir': str(run_dir),
        'status': 'success',
        'files': {},
        'launch_flags': {},
        'verify': {
            'result': 'PASS',
            'counts': {'pass': 8, 'warn': 0, 'fail': 0},
        },
        'projector_type': 'Local',
        'bag_preflight': {
            'summary': {
                'duration_sec': 12.5,
                'message_count': 30,
                'topics': {
                    'pointcloud2': [{
                        'name': '/points',
                        'msg_type': 'sensor_msgs/msg/PointCloud2',
                        'message_count': 10,
                    }],
                    'imu': [{
                        'name': '/imu',
                        'msg_type': 'sensor_msgs/msg/Imu',
                        'message_count': 20,
                    }],
                },
            },
        },
        'problem_hints': [],
        'suggested_next_steps': [],
    }
    _write_json(run_dir / 'run_manifest.json', manifest)
    _write_json(run_dir / 'autoware_map_diagnosis.json', diagnosis)
    return (
        _write_json(tmp_path / 'contract.json', contract),
        _write_json(tmp_path / 'intake.json', intake),
        run_dir,
    )


def test_validator_accepts_complete_pinned_evidence(tmp_path: Path):
    """Complete pinned artifacts should pass every evidence assertion."""
    module = _load_module()
    contract, intake, run_dir = _fixture(tmp_path)

    report = module.validate(contract, intake, run_dir)

    assert report['status'] == 'PASS'
    assert report['summary']['fail'] == 0
    assert report['summary']['raw_poses'] == 2
    assert report['summary']['pointcloud_tiles'] == 2
    assert '**PASS**' in module.render_markdown(report)


def test_validator_rejects_tampered_cached_archive(tmp_path: Path):
    """A cache entry whose content changed must fail archive identity."""
    module = _load_module()
    contract, intake, run_dir = _fixture(tmp_path)
    intake_payload = json.loads(intake.read_text(encoding='utf-8'))
    Path(intake_payload['archive_path']).write_bytes(b'tampered')

    report = module.validate(contract, intake, run_dir)

    assert report['status'] == 'FAIL'
    archive_check = next(
        row for row in report['checks'] if row['id'] == 'archive_identity'
    )
    assert archive_check['status'] == 'FAIL'


def test_validator_rejects_contract_without_archive_sha256(tmp_path: Path):
    """The pinned real-data contract must carry a strong archive digest."""
    module = _load_module()
    contract, intake, run_dir = _fixture(tmp_path)
    contract_payload = json.loads(contract.read_text(encoding='utf-8'))
    contract_payload['dataset'].pop('sha256')
    contract.write_text(json.dumps(contract_payload), encoding='utf-8')

    report = module.validate(contract, intake, run_dir)

    assert report['status'] == 'FAIL'
    archive_check = next(
        row for row in report['checks'] if row['id'] == 'archive_identity'
    )
    assert archive_check['status'] == 'FAIL'
