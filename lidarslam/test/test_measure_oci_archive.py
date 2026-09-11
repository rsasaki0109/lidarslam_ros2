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

"""Tests for fail-closed OCI archive measurement."""

from __future__ import annotations

from decimal import Decimal
import gzip
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
import tarfile
from typing import Any

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / 'measure_oci_archive.py'
SPEC = importlib.util.spec_from_file_location('measure_oci_archive', SCRIPT)
assert SPEC is not None and SPEC.loader is not None
OCI = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OCI)

REVISION = 'a' * 40
VERSION = '0.9.0'
BASELINE_REFERENCE = 'ghcr.io/example/lidarslam:v0.9.0-jazzy'
BASELINE_INDEX_DIGEST = 'sha256:' + ('b' * 64)
BASELINE_MANIFEST_DIGEST = 'sha256:' + ('c' * 64)


def _json_bytes(value: dict[str, Any]) -> bytes:
    return json.dumps(
        value,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('utf-8')


def _digest(payload: bytes) -> str:
    return 'sha256:' + hashlib.sha256(payload).hexdigest()


def _descriptor(payload: bytes, media_type: str) -> dict[str, Any]:
    return {
        'mediaType': media_type,
        'digest': _digest(payload),
        'size': len(payload),
    }


def _layer_tar() -> bytes:
    output = io.BytesIO()
    with tarfile.open(fileobj=output, mode='w') as archive:
        payload = b'installed lidar slam\n'
        member = tarfile.TarInfo('opt/lidarslam/version.txt')
        member.size = len(payload)
        member.mtime = 0
        archive.addfile(member, io.BytesIO(payload))
    return output.getvalue()


def _oci_files(
    *,
    diff_id: str | None = None,
    layer_media_type: str = OCI.OCI_GZIP_LAYER_MEDIA_TYPE,
    revision: str = REVISION,
) -> tuple[list[tuple[str, bytes]], int]:
    layer_tar = _layer_tar()
    layer = gzip.compress(layer_tar, mtime=0)
    config = _json_bytes({
        'architecture': 'amd64',
        'os': 'linux',
        'config': {
            'Labels': {
                'org.opencontainers.image.revision': revision,
                'org.opencontainers.image.version': VERSION,
            },
        },
        'rootfs': {
            'type': 'layers',
            'diff_ids': [diff_id or _digest(layer_tar)],
        },
    })
    manifest = _json_bytes({
        'schemaVersion': 2,
        'mediaType': OCI.OCI_MANIFEST_MEDIA_TYPE,
        'config': _descriptor(config, OCI.OCI_CONFIG_MEDIA_TYPE),
        'layers': [_descriptor(layer, layer_media_type)],
    })
    index = _json_bytes({
        'schemaVersion': 2,
        'mediaType': OCI.OCI_INDEX_MEDIA_TYPE,
        'manifests': [{
            **_descriptor(manifest, OCI.OCI_MANIFEST_MEDIA_TYPE),
            'platform': {'architecture': 'amd64', 'os': 'linux'},
        }],
    })
    files = [
        ('oci-layout', _json_bytes({'imageLayoutVersion': '1.0.0'})),
        ('index.json', index),
        (OCI._blob_name(_digest(manifest)), manifest),
        (OCI._blob_name(_digest(config)), config),
        (OCI._blob_name(_digest(layer)), layer),
    ]
    return files, len(layer)


def _write_archive(
    path: Path,
    files: list[tuple[str, bytes]],
) -> None:
    with tarfile.open(path, mode='w') as archive:
        for name, payload in files:
            member = tarfile.TarInfo(name)
            member.size = len(payload)
            member.mtime = 0
            archive.addfile(member, io.BytesIO(payload))


def test_verified_archive_passes_exact_descriptor_and_reduction_gate(
    tmp_path: Path,
) -> None:
    """A complete gzip OCI graph produces schema-valid exact measurements."""
    archive = tmp_path / 'candidate.oci.tar'
    files, compressed_bytes = _oci_files()
    _write_archive(archive, files)

    report = OCI.measure_oci_archive(
        archive,
        expected_revision=REVISION,
        expected_version=VERSION,
        baseline_compressed_bytes=compressed_bytes * 2,
        baseline_reference=BASELINE_REFERENCE,
        baseline_index_digest=BASELINE_INDEX_DIGEST,
        baseline_manifest_digest=BASELINE_MANIFEST_DIGEST,
        minimum_reduction_percent=Decimal('25'),
    )

    assert report['status'] == 'PASS'
    assert report['image']['compressed_layer_bytes'] == compressed_bytes
    assert report['image']['uncompressed_layer_bytes'] == len(_layer_tar())
    assert report['image']['layer_count'] == 1
    assert report['verification'] == {
        'status': 'PASS',
        'safe_archive_members': 5,
        'descriptors_verified': 3,
        'diff_ids_verified': 1,
        'unreferenced_blobs': 0,
        'all_layers_gzip': True,
    }
    assert report['gate']['status'] == 'PASS'
    assert report['gate']['reduction_percent'] == 50.0


def test_valid_archive_returns_fail_when_size_gate_is_missed(
    tmp_path: Path,
) -> None:
    """A valid image remains measured but cannot claim a missed reduction."""
    archive = tmp_path / 'candidate.oci.tar'
    files, compressed_bytes = _oci_files()
    _write_archive(archive, files)

    report = OCI.measure_oci_archive(
        archive,
        baseline_compressed_bytes=compressed_bytes,
        baseline_reference=BASELINE_REFERENCE,
        baseline_index_digest=BASELINE_INDEX_DIGEST,
        baseline_manifest_digest=BASELINE_MANIFEST_DIGEST,
    )

    assert report['verification']['status'] == 'PASS'
    assert report['status'] == 'FAIL'
    assert report['gate']['status'] == 'FAIL'
    assert report['gate']['reduction_bytes'] == 0
    assert report['gate']['margin_bytes'] < 0


def test_baseline_size_and_identity_are_an_atomic_gate_input(
    tmp_path: Path,
) -> None:
    """A size claim without immutable baseline identity is not evidence."""
    archive = tmp_path / 'candidate.oci.tar'
    files, compressed_bytes = _oci_files()
    _write_archive(archive, files)

    with pytest.raises(OCI.OciArchiveError, match='baseline bytes require'):
        OCI.measure_oci_archive(
            archive,
            baseline_compressed_bytes=compressed_bytes * 2,
        )


def test_schema_rejects_a_status_that_disagrees_with_the_gate(
    tmp_path: Path,
) -> None:
    """A hand-edited record cannot invert the measured gate status."""
    archive = tmp_path / 'candidate.oci.tar'
    files, compressed_bytes = _oci_files()
    _write_archive(archive, files)
    report = OCI.measure_oci_archive(
        archive,
        baseline_compressed_bytes=compressed_bytes * 2,
        baseline_reference=BASELINE_REFERENCE,
        baseline_index_digest=BASELINE_INDEX_DIGEST,
        baseline_manifest_digest=BASELINE_MANIFEST_DIGEST,
    )
    report['status'] = 'FAIL'

    with pytest.raises(ValueError, match='validation failed'):
        OCI.validate_contract(report, OCI.SCHEMA_NAME)


@pytest.mark.parametrize(
    ('mutation', 'error_pattern'),
    [
        ('diff-id', 'diffID'),
        ('media-type', 'mediaType must be'),
        ('revision', 'image label'),
    ],
)
def test_diff_id_media_type_and_revision_are_fail_closed(
    tmp_path: Path,
    mutation: str,
    error_pattern: str,
) -> None:
    """Config, layer encoding, and identity cannot be inferred or ignored."""
    options: dict[str, Any] = {}
    if mutation == 'diff-id':
        options['diff_id'] = 'sha256:' + ('f' * 64)
    elif mutation == 'media-type':
        options['layer_media_type'] = (
            'application/vnd.oci.image.layer.v1.tar'
        )
    elif mutation == 'revision':
        options['revision'] = 'b' * 40
    archive = tmp_path / f'{mutation}.oci.tar'
    files, _ = _oci_files(**options)
    _write_archive(archive, files)

    with pytest.raises(OCI.OciArchiveError, match=error_pattern):
        OCI.measure_oci_archive(
            archive,
            expected_revision=REVISION,
            expected_version=VERSION,
        )


def test_corrupt_blob_digest_is_rejected(tmp_path: Path) -> None:
    """Descriptor size equality cannot conceal changed blob bytes."""
    archive = tmp_path / 'corrupt.oci.tar'
    files, _ = _oci_files()
    name, layer = files[-1]
    files[-1] = (name, bytes([layer[0] ^ 0x01]) + layer[1:])
    _write_archive(archive, files)

    with pytest.raises(OCI.OciArchiveError):
        OCI.measure_oci_archive(archive)


@pytest.mark.parametrize(
    'bad_member',
    ['../escape', '/absolute', './ambiguous'],
)
def test_unsafe_archive_paths_are_rejected(
    tmp_path: Path,
    bad_member: str,
) -> None:
    """An OCI tar cannot smuggle traversal or absolute paths."""
    archive = tmp_path / 'unsafe.oci.tar'
    files, _ = _oci_files()
    files.append((bad_member, b'unsafe'))
    _write_archive(archive, files)

    with pytest.raises(OCI.OciArchiveError, match='unsafe archive member'):
        OCI.measure_oci_archive(archive)


def test_duplicate_members_and_unreferenced_blobs_are_rejected(
    tmp_path: Path,
) -> None:
    """Ambiguous paths and blob payload outside the image graph fail closed."""
    duplicate = tmp_path / 'duplicate.oci.tar'
    files, _ = _oci_files()
    files.append(('index.json', files[1][1]))
    _write_archive(duplicate, files)
    with pytest.raises(OCI.OciArchiveError, match='duplicate archive member'):
        OCI.measure_oci_archive(duplicate)

    extra = tmp_path / 'extra.oci.tar'
    files, _ = _oci_files()
    files.append(('blobs/sha256/' + ('e' * 64), b'extra'))
    _write_archive(extra, files)
    with pytest.raises(OCI.OciArchiveError, match='blob closure mismatch'):
        OCI.measure_oci_archive(extra)

    outside = tmp_path / 'outside.oci.tar'
    files, _ = _oci_files()
    files.append(('unexpected.txt', b'extra'))
    _write_archive(outside, files)
    with pytest.raises(OCI.OciArchiveError, match='outside the OCI image'):
        OCI.measure_oci_archive(outside)


def test_platform_must_match_the_selected_single_manifest(
    tmp_path: Path,
) -> None:
    """A measurement cannot silently select a different architecture."""
    archive = tmp_path / 'platform.oci.tar'
    files, _ = _oci_files()
    _write_archive(archive, files)

    with pytest.raises(OCI.OciArchiveError, match='platform must be'):
        OCI.measure_oci_archive(archive, architecture='arm64')
