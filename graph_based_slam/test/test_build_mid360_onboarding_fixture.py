# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Tests for the deterministic MID-360 onboarding fixture builder."""

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import zipfile

import pytest
from rosbags.rosbag2 import Writer
from rosbags.typesys import get_typestore, Stores


ROOT = Path(__file__).resolve().parents[2]
EVIDENCE_ROOT = ROOT / 'docs' / 'evidence' / 'onboarding'
sys.path.insert(0, str(ROOT / 'scripts'))
SPEC = importlib.util.spec_from_file_location(
    'build_mid360_onboarding_fixture',
    ROOT / 'scripts/build_mid360_onboarding_fixture.py',
)
BUILDER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BUILDER)


def _source_fixture(root: Path) -> tuple[Path, Path, Path]:
    archive = root / 'source.zip'
    archive.write_bytes(b'pinned public source archive\n')
    bag = root / 'source_bag'
    typestore = get_typestore(Stores.ROS2_JAZZY)
    with Writer(bag, version=8) as writer:
        lidar = writer.add_connection(
            '/livox/lidar',
            'sensor_msgs/msg/PointCloud2',
            typestore=typestore,
        )
        imu = writer.add_connection(
            '/livox/imu',
            'sensor_msgs/msg/Imu',
            typestore=typestore,
        )
        for index, timestamp in enumerate(
            (1_000_000_000, 2_000_000_000, 3_000_000_000)
        ):
            writer.write(lidar, timestamp, f'lidar-{index}'.encode())
        for index, timestamp in enumerate(
            (1_100_000_000, 1_500_000_000, 2_500_000_000, 3_100_000_000)
        ):
            writer.write(imu, timestamp, f'imu-{index}'.encode())

    bag_identity = BUILDER._bag_identity(bag)
    archive_digests = BUILDER._file_digests(archive, ('md5', 'sha256'))
    contract = {
        'schema_version': 1,
        'id': 'source_contract_v1',
        'dataset': {
            'id': 'source_dataset',
            'title': 'Public MID-360 test source',
            'creator': 'Fixture Author',
            'citation': 'Fixture Author. Public MID-360 test source.',
            'source_url': 'https://example.com/source',
            'doi': '10.1234/example.fixture',
            'download_url': 'https://example.com/source.zip',
            'license': {
                'spdx': 'CC-BY-4.0',
                'name': 'Creative Commons Attribution 4.0 International',
                'url': 'https://creativecommons.org/licenses/by/4.0/',
            },
            'file_id': 'source_archive',
            'filename': archive.name,
            'size_bytes': archive.stat().st_size,
            **archive_digests,
        },
        'input': {
            'bag_directory': bag.name,
            'metadata_size_bytes': bag_identity['metadata']['size_bytes'],
            'metadata_sha256': bag_identity['metadata']['sha256'],
            'storage_identifier': bag_identity['storage_identifier'],
            'storage_files': bag_identity['storage_files'],
            'duration_sec': bag_identity['duration_ns'] / 1e9,
            'duration_tolerance_sec': 0.0,
            'message_count': bag_identity['message_count'],
            'topics': {
                row['name']: {
                    'type': row['type'],
                    'message_count': row['message_count'],
                }
                for row in bag_identity['topics']
            },
        },
    }
    contract_path = root / 'source_contract.json'
    contract_path.write_text(json.dumps(contract), encoding='utf-8')
    return archive, bag, contract_path


def _fixed_revision(*_args, **_kwargs):
    return {'git_commit': 'a' * 40, 'git_dirty': False}


def test_builder_is_byte_deterministic_and_attributed(tmp_path, monkeypatch):
    archive, bag, contract = _source_fixture(tmp_path)
    monkeypatch.setattr(BUILDER, 'REPO_ROOT', tmp_path)
    monkeypatch.setattr(BUILDER, '_git_identity', _fixed_revision)
    first_dir = tmp_path / 'first'
    second_dir = tmp_path / 'second'

    first = BUILDER.build_fixture(
        archive,
        bag,
        first_dir,
        contract_path=contract,
        fixture_id='mid360_fixture_test_v1',
        duration_seconds=2.0,
        max_archive_bytes=1_000_000,
    )
    second = BUILDER.build_fixture(
        archive,
        bag,
        second_dir,
        contract_path=contract,
        fixture_id='mid360_fixture_test_v1',
        duration_seconds=2.0,
        max_archive_bytes=1_000_000,
    )

    first_zip = first_dir / first['artifact']['filename']
    second_zip = second_dir / second['artifact']['filename']
    first_manifest = first_dir / 'mid360_fixture_test_v1.manifest.json'
    second_manifest = second_dir / 'mid360_fixture_test_v1.manifest.json'
    assert first_zip.read_bytes() == second_zip.read_bytes()
    assert first_manifest.read_bytes() == second_manifest.read_bytes()
    assert first == second
    assert first['clip']['message_count'] == 5
    assert first['clip']['raw_message_copy'] is True
    assert first['artifact']['size_gate_pass'] is True
    assert first['publication'] == {
        'onboarding_only': True,
        'replaces_full_real_data_gate': False,
        'contains_map_geometry': True,
        'track_archive_in_git': False,
        'review_before_publishing': True,
        'map_validation_status': 'NOT_RUN',
        'attribution_member': (
            'mid360_fixture_test_v1/ATTRIBUTION.md'
        ),
    }
    assert str(tmp_path) not in first_manifest.read_text(encoding='utf-8')

    with zipfile.ZipFile(first_zip) as stream:
        assert stream.testzip() is None
        assert stream.namelist() == sorted(stream.namelist())
        assert stream.namelist() == [
            'mid360_fixture_test_v1/ATTRIBUTION.md',
            'mid360_fixture_test_v1/bag/bag.db3',
            'mid360_fixture_test_v1/bag/metadata.yaml',
        ]
        for info in stream.infolist():
            assert info.date_time == BUILDER.ZIP_MEMBER_TIMESTAMP
            assert info.external_attr >> 16 == BUILDER.ZIP_MEMBER_MODE
        attribution = stream.read(
            'mid360_fixture_test_v1/ATTRIBUTION.md'
        ).decode()
    assert 'Fixture Author' in attribution
    assert 'https://doi.org/10.1234/example.fixture' in attribution
    assert 'Creative Commons Attribution 4.0 International' in attribution
    assert 'does not replace the full 277-second' in attribution


def test_builder_rejects_changed_source_before_writing(tmp_path, monkeypatch):
    archive, bag, contract = _source_fixture(tmp_path)
    monkeypatch.setattr(BUILDER, 'REPO_ROOT', tmp_path)
    monkeypatch.setattr(BUILDER, '_git_identity', _fixed_revision)
    changed = bytearray(archive.read_bytes())
    changed[0] ^= 1
    archive.write_bytes(changed)
    output = tmp_path / 'output'

    with pytest.raises(ValueError, match='source archive MD5'):
        BUILDER.build_fixture(
            archive,
            bag,
            output,
            contract_path=contract,
            fixture_id='mid360_fixture_test_v1',
            duration_seconds=2.0,
        )

    assert not output.exists()


def test_builder_enforces_archive_size_gate_without_partial_outputs(
    tmp_path, monkeypatch
):
    archive, bag, contract = _source_fixture(tmp_path)
    monkeypatch.setattr(BUILDER, 'REPO_ROOT', tmp_path)
    monkeypatch.setattr(BUILDER, '_git_identity', _fixed_revision)
    output = tmp_path / 'output'

    with pytest.raises(ValueError, match='archive size gate failed'):
        BUILDER.build_fixture(
            archive,
            bag,
            output,
            contract_path=contract,
            fixture_id='mid360_fixture_test_v1',
            duration_seconds=2.0,
            max_archive_bytes=100,
        )

    assert list(output.iterdir()) == []


def test_builder_never_overwrites_existing_output(tmp_path, monkeypatch):
    archive, bag, contract = _source_fixture(tmp_path)
    monkeypatch.setattr(BUILDER, 'REPO_ROOT', tmp_path)
    monkeypatch.setattr(BUILDER, '_git_identity', _fixed_revision)
    output = tmp_path / 'output'
    output.mkdir()
    existing = output / 'mid360_fixture_test_v1.zip'
    existing.write_bytes(b'owner data\n')

    with pytest.raises(ValueError, match='output already exists'):
        BUILDER.build_fixture(
            archive,
            bag,
            output,
            contract_path=contract,
            fixture_id='mid360_fixture_test_v1',
            duration_seconds=2.0,
        )

    assert existing.read_bytes() == b'owner data\n'
    assert not (output / 'mid360_fixture_test_v1.manifest.json').exists()


def test_builder_rejects_source_symlink_before_resolution(tmp_path, monkeypatch):
    archive, bag, contract = _source_fixture(tmp_path)
    monkeypatch.setattr(BUILDER, 'REPO_ROOT', tmp_path)
    monkeypatch.setattr(BUILDER, '_git_identity', _fixed_revision)
    archive_link = tmp_path / 'archive-link.zip'
    archive_link.symlink_to(archive)
    output = tmp_path / 'output'

    with pytest.raises(ValueError, match='must not use symlink components'):
        BUILDER.build_fixture(
            archive_link,
            bag,
            output,
            contract_path=contract,
            fixture_id='mid360_fixture_test_v1',
            duration_seconds=2.0,
        )

    assert not output.exists()


def test_storage_identity_rejects_symlinked_parent(tmp_path):
    bag = tmp_path / 'bag'
    outside = tmp_path / 'outside'
    bag.mkdir()
    outside.mkdir()
    (outside / 'data.db3').write_bytes(b'outside storage\n')
    (bag / 'nested').symlink_to(outside, target_is_directory=True)

    with pytest.raises(ValueError, match='storage path uses a symlink'):
        BUILDER._storage_rows(
            bag, {'relative_file_paths': ['nested/data.db3']}
        )


def test_committed_fixture_evidence_is_schema_valid_and_geometry_free():
    build_path = (
        EVIDENCE_ROOT /
        'mid360-onboarding-50s-v1-build-20260810.json'
    )
    receipt_path = (
        EVIDENCE_ROOT /
        'mid360-onboarding-50s-v1-map-receipt-20260810.json'
    )
    evidence_doc = (
        EVIDENCE_ROOT /
        'mid360-onboarding-fixture-pilot-2026-08-10.md'
    ).read_text(encoding='utf-8')
    build_text = build_path.read_text(encoding='utf-8')
    receipt_text = receipt_path.read_text(encoding='utf-8')
    build = json.loads(build_text)
    receipt = json.loads(receipt_text)

    BUILDER.validate_contract(build, BUILDER.SCHEMA_FILENAME)
    BUILDER.validate_contract(
        receipt, 'first-map-validation-receipt-v1.schema.json'
    )
    assert '/tmp/' not in build_text + receipt_text
    assert '/home/' not in build_text + receipt_text
    assert build['artifact']['size_bytes'] == 98_873_952
    assert build['artifact']['sha256'] == (
        '20e5151728522877bff75021a473e91c5ae900448fa9e6977bf88653fa464bd3'
    )
    assert build['generation']['revision']['git_dirty'] is False
    assert build['publication']['map_validation_status'] == 'NOT_RUN'
    assert build['publication']['track_archive_in_git'] is False
    assert receipt['status'] == 'PASS'
    assert len(receipt['checks']) == 7
    assert all(check['passed'] for check in receipt['checks'])
    assert receipt['shareability']['contains_map_geometry'] is False
    assert hashlib.sha256(build_path.read_bytes()).hexdigest() in evidence_doc
    assert hashlib.sha256(receipt_path.read_bytes()).hexdigest() in evidence_doc
    assert not list(EVIDENCE_ROOT.glob('mid360_onboarding_50s_v1.zip'))
