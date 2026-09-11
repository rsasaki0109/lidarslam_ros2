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

"""Tests for the fail-closed fixture publication review."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import stat
import sys
from typing import Any
import zipfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
EVIDENCE = ROOT / 'docs' / 'evidence' / 'onboarding'
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / 'check_fixture_publication.py'
SPEC = importlib.util.spec_from_file_location(
    'check_fixture_publication', SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)

GENERATOR_REVISION = 'a' * 40
REVIEW_REVISION = 'b' * 40
FIXTURE_ID = 'mid360_onboarding_50s_v1'
ATTRIBUTION_NAME = f'{FIXTURE_ID}/ATTRIBUTION.md'
DATABASE_NAME = f'{FIXTURE_ID}/bag/bag.db3'
METADATA_NAME = f'{FIXTURE_ID}/bag/metadata.yaml'


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _attribution(manifest: dict[str, Any]) -> bytes:
    source = manifest['source']
    return f"""# Attribution and license

This ROS 2 bag is a derivative onboarding fixture made from
\"{source['title']}\" by {source['creator']}.

- Original source: {source['source_url']}
- DOI: https://doi.org/{source['doi']}
- Citation: {source['citation']}
- License: {source['license_name']} ({source['license_url']})

Changes made by the lidar_slam_ros2 project: retained the first
50 seconds of messages on `/livox/lidar` and `/livox/imu`.

This fixture is for onboarding only. It does not replace the full 277-second
real-data release gate. No endorsement by the original creator is implied.
""".encode()


def _write_zip(
    path: Path,
    entries: dict[str, bytes],
    *,
    modes: dict[str, int] | None = None,
) -> None:
    with zipfile.ZipFile(
        path,
        mode='w',
        compression=zipfile.ZIP_DEFLATED,
        compresslevel=9,
    ) as archive:
        for name in sorted(entries):
            info = zipfile.ZipInfo(name, CHECKER.ZIP_MEMBER_TIMESTAMP)
            info.create_system = 3
            info.compress_type = zipfile.ZIP_DEFLATED
            mode = (modes or {}).get(name, stat.S_IFREG | 0o644)
            info.external_attr = mode << 16
            archive.writestr(info, entries[name])


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )


def _bind_manifest(
    manifest: dict[str, Any],
    zip_path: Path,
    entries: dict[str, bytes],
    *,
    bind_members: bool = True,
) -> None:
    artifact = manifest['artifact']
    artifact['size_bytes'] = zip_path.stat().st_size
    artifact['sha256'] = _sha256(zip_path.read_bytes())
    artifact['max_size_bytes'] = 1_000_000
    artifact['size_gate_pass'] = True
    if not bind_members:
        return
    artifact['members'] = [
        {
            'path': name,
            'size_bytes': len(entries[name]),
            'sha256': _sha256(entries[name]),
        }
        for name in sorted(entries)
    ]
    metadata = manifest['clip']['bag']['metadata']
    metadata['size_bytes'] = len(entries[METADATA_NAME])
    metadata['sha256'] = _sha256(entries[METADATA_NAME])
    storage = manifest['clip']['bag']['storage_files'][0]
    storage['size_bytes'] = len(entries[DATABASE_NAME])
    storage['sha256'] = _sha256(entries[DATABASE_NAME])


def _publication_packet(tmp_path: Path) -> dict[str, Any]:
    manifest = json.loads((
        EVIDENCE / 'mid360-onboarding-50s-v1-build-20260810.json'
    ).read_text(encoding='utf-8'))
    receipt = json.loads((
        EVIDENCE / 'mid360-onboarding-50s-v1-map-receipt-20260810.json'
    ).read_text(encoding='utf-8'))
    manifest['generation']['revision'] = {
        'git_commit': GENERATOR_REVISION,
        'git_dirty': False,
    }
    receipt['run']['git_commit'] = GENERATOR_REVISION
    entries = {
        ATTRIBUTION_NAME: _attribution(manifest),
        DATABASE_NAME: b'synthetic sqlite payload\n',
        METADATA_NAME: b'rosbag2_bagfile_information:\n  version: 8\n',
    }
    zip_path = tmp_path / f'{FIXTURE_ID}.zip'
    manifest_path = tmp_path / 'fixture-build.json'
    receipt_path = tmp_path / 'map-receipt.json'
    _write_zip(zip_path, entries)
    _bind_manifest(manifest, zip_path, entries)
    _write_json(manifest_path, manifest)
    _write_json(receipt_path, receipt)
    return {
        'manifest': manifest,
        'receipt': receipt,
        'entries': entries,
        'zip_path': zip_path,
        'manifest_path': manifest_path,
        'receipt_path': receipt_path,
    }


def _review(packet: dict[str, Any], **overrides: Any) -> dict[str, Any]:
    arguments = {
        'expected_manifest_sha256': _sha256(
            packet['manifest_path'].read_bytes()
        ),
        'expected_receipt_sha256': _sha256(
            packet['receipt_path'].read_bytes()
        ),
        'review_id': 'fixture-publication-test-v1',
        'reviewed_on': '2026-08-11',
        'repository': 'example/lidar_slam_ros2',
        'review_revision': REVIEW_REVISION,
        'generator_revision_remote_status': 'RESOLVABLE',
        'review_revision_remote_status': 'RESOLVABLE',
        'clean_rebuilds': 2,
        'rebuild_artifacts_byte_identical': True,
        'rebuild_manifests_byte_identical': True,
        'host': 'github-release',
        'upload_authorized': True,
    }
    arguments.update(overrides)
    return CHECKER.review_fixture_publication(
        packet['zip_path'],
        packet['manifest_path'],
        packet['receipt_path'],
        **arguments,
    )


def test_verified_packet_is_publication_ready(tmp_path):
    """A complete local packet and explicit decision can become ready."""
    packet = _publication_packet(tmp_path)

    report = _review(packet)

    assert report['status'] == 'PUBLICATION_READY'
    assert report['local_validation']['status'] == 'LOCAL_ARTIFACT_PASS'
    assert len(report['local_validation']['checks']) == 13
    assert all(
        row['status'] == 'PASS'
        for row in report['local_validation']['checks']
    )
    assert report['local_validation']['archive'] == {
        'safe_member_count': 3,
        'expected_member_count': 3,
        'member_digests_verified': 3,
        'member_bytes_verified': sum(
            len(payload) for payload in packet['entries'].values()
        ),
        'unexpected_members': 0,
        'sorted_members': True,
        'fixed_timestamp': True,
        'unix_mode_0644': True,
        'deflate_only': True,
        'encrypted_members': 0,
        'archive_comment_empty': True,
        'member_metadata_empty': True,
    }
    assert report['decision']['blockers'] == []
    assert report['publication'] == {
        'host': 'github-release',
        'authorization': 'GRANTED',
        'upload_performed': False,
        'immutable_record_id': None,
        'immutable_artifact_url': None,
    }
    rendered = json.dumps(report)
    assert str(tmp_path) not in rendered
    assert '/home/' not in rendered


def test_local_pass_waits_for_remote_host_and_authorization(tmp_path):
    """Local PASS remains distinct from permission to publish."""
    packet = _publication_packet(tmp_path)

    report = _review(
        packet,
        generator_revision_remote_status='UNRESOLVABLE',
        review_revision_remote_status='UNRESOLVABLE',
        host=None,
        upload_authorized=False,
    )

    assert report['status'] == 'AWAITING_PUBLICATION_DECISION'
    assert report['local_validation']['status'] == 'LOCAL_ARTIFACT_PASS'
    assert report['decision']['blockers'] == [
        'generator-revision-not-remotely-resolvable',
        'review-revision-not-remotely-resolvable',
        'publication-host-unset',
        'explicit-upload-authorization-missing',
    ]


def test_changed_artifact_is_rejected_before_archive_review(tmp_path):
    """The manifest-bound whole-archive identity cannot drift."""
    packet = _publication_packet(tmp_path)
    packet['zip_path'].write_bytes(packet['zip_path'].read_bytes() + b'x')

    with pytest.raises(
        CHECKER.FixturePublicationError,
        match='fixture artifact size',
    ):
        _review(packet)


def test_unsafe_zip_member_is_rejected(tmp_path):
    """A traversal member fails before any extraction is possible."""
    packet = _publication_packet(tmp_path)
    entries = {**packet['entries'], '../escape': b'escape\n'}
    _write_zip(packet['zip_path'], entries)
    _bind_manifest(
        packet['manifest'],
        packet['zip_path'],
        entries,
        bind_members=False,
    )
    _write_json(packet['manifest_path'], packet['manifest'])

    with pytest.raises(
        CHECKER.FixturePublicationError,
        match='unsafe or wrong-root ZIP member',
    ):
        _review(packet)


def test_symlink_zip_member_is_rejected(tmp_path):
    """A ZIP member masquerading as a symlink is never publishable."""
    packet = _publication_packet(tmp_path)
    _write_zip(
        packet['zip_path'],
        packet['entries'],
        modes={DATABASE_NAME: stat.S_IFLNK | 0o777},
    )
    _bind_manifest(
        packet['manifest'], packet['zip_path'], packet['entries']
    )
    _write_json(packet['manifest_path'], packet['manifest'])

    with pytest.raises(
        CHECKER.FixturePublicationError,
        match='not a regular file',
    ):
        _review(packet)


def test_member_digest_mismatch_is_rejected(tmp_path):
    """Each member must match the manifest, not only the outer ZIP."""
    packet = _publication_packet(tmp_path)
    changed = copy.deepcopy(packet['entries'])
    changed[DATABASE_NAME] = b'S' + changed[DATABASE_NAME][1:]
    _write_zip(packet['zip_path'], changed)
    _bind_manifest(
        packet['manifest'],
        packet['zip_path'],
        changed,
        bind_members=False,
    )
    _write_json(packet['manifest_path'], packet['manifest'])

    with pytest.raises(
        CHECKER.FixturePublicationError,
        match='ZIP member digest mismatch',
    ):
        _review(packet)


def test_incomplete_attribution_is_rejected(tmp_path):
    """The derivative fixture retains all mandatory attribution notices."""
    packet = _publication_packet(tmp_path)
    entries = copy.deepcopy(packet['entries'])
    entries[ATTRIBUTION_NAME] = entries[ATTRIBUTION_NAME].replace(
        b'No endorsement by the original creator is implied.\n', b''
    )
    _write_zip(packet['zip_path'], entries)
    _bind_manifest(packet['manifest'], packet['zip_path'], entries)
    _write_json(packet['manifest_path'], packet['manifest'])

    with pytest.raises(
        CHECKER.FixturePublicationError,
        match='missing required fields: no_endorsement',
    ):
        _review(packet)


def test_receipt_must_match_generator_revision(tmp_path):
    """A map receipt from another product revision cannot be substituted."""
    packet = _publication_packet(tmp_path)
    packet['receipt']['run']['git_commit'] = 'c' * 40
    _write_json(packet['receipt_path'], packet['receipt'])

    with pytest.raises(
        CHECKER.FixturePublicationError,
        match='revision does not match',
    ):
        _review(packet)


def test_receipt_must_be_seven_of_seven_pass(tmp_path):
    """A schema-valid failed map receipt still blocks publication."""
    packet = _publication_packet(tmp_path)
    packet['receipt']['status'] = 'FAIL'
    _write_json(packet['receipt_path'], packet['receipt'])

    with pytest.raises(
        CHECKER.FixturePublicationError,
        match='not seven-of-seven PASS',
    ):
        _review(packet)


@pytest.mark.parametrize(
    'overrides, message',
    [
        ({'clean_rebuilds': 1}, 'at least two clean fixture rebuilds'),
        (
            {'rebuild_artifacts_byte_identical': False},
            'artifact bytes are not attested identical',
        ),
        (
            {'rebuild_manifests_byte_identical': False},
            'manifest bytes are not attested identical',
        ),
    ],
)
def test_reproducibility_attestation_fails_closed(
    tmp_path, overrides, message
):
    """Two byte-identical clean builds are a hard local requirement."""
    packet = _publication_packet(tmp_path)

    with pytest.raises(CHECKER.FixturePublicationError, match=message):
        _review(packet, **overrides)


def test_checker_is_curated_into_release_bundle():
    """The publication gate and schema ship with product evidence tooling."""
    release_builder = (SCRIPTS / 'build_release_bundle.py').read_text(
        encoding='utf-8'
    )

    assert 'scripts/check_fixture_publication.py' in release_builder
    assert (ROOT / 'docs' / 'schemas' / CHECKER.SCHEMA_NAME).is_file()


def test_committed_review_is_waiting_and_privacy_bounded():
    """Tracked evidence records local PASS without implying publication."""
    review_path = (
        EVIDENCE /
        'mid360-onboarding-50s-v1-publication-review-20260811.json'
    )
    review_doc = (
        EVIDENCE / 'mid360-fixture-publication-review-2026-08-11.md'
    ).read_text(encoding='utf-8')
    review_bytes = review_path.read_bytes()
    review = json.loads(review_bytes)

    CHECKER.validate_contract(review, CHECKER.SCHEMA_NAME)
    assert review['status'] == 'AWAITING_PUBLICATION_DECISION'
    assert review['local_validation']['status'] == 'LOCAL_ARTIFACT_PASS'
    assert len(review['local_validation']['checks']) == 13
    assert review['fixture']['artifact']['sha256'] == (
        '20e5151728522877bff75021a473e91c5ae900448fa9e6977bf88653fa464bd3'
    )
    assert review['publication']['host'] is None
    assert review['publication']['authorization'] == 'NOT_GRANTED'
    assert review['publication']['upload_performed'] is False
    assert b'/home/' not in review_bytes
    assert b'/tmp/' not in review_bytes
    assert hashlib.sha256(review_bytes).hexdigest() in review_doc
    assert not list(EVIDENCE.glob('mid360_onboarding_50s_v1.zip'))
