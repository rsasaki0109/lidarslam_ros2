#!/usr/bin/env python3
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

"""Verify and measure a single-platform gzip-compressed OCI image archive."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import re
import sys
import tarfile
from decimal import Decimal, InvalidOperation, ROUND_CEILING
from pathlib import Path, PurePosixPath
from typing import Any, BinaryIO

from product_schema import validate_contract


SCHEMA_NAME = 'oci-image-measurement-v1.schema.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/oci-image-measurement-v1.schema.json'
)
OCI_INDEX_MEDIA_TYPE = 'application/vnd.oci.image.index.v1+json'
OCI_MANIFEST_MEDIA_TYPE = 'application/vnd.oci.image.manifest.v1+json'
OCI_CONFIG_MEDIA_TYPE = 'application/vnd.oci.image.config.v1+json'
OCI_GZIP_LAYER_MEDIA_TYPE = 'application/vnd.oci.image.layer.v1.tar+gzip'
DIGEST_PATTERN = re.compile(r'^sha256:([0-9a-f]{64})$')
READ_CHUNK_BYTES = 1024 * 1024
MAX_CONTROL_FILE_BYTES = 16 * 1024 * 1024


class OciArchiveError(ValueError):
    """The OCI archive cannot support a trusted measurement."""


class _DigestingReader:
    """Hash compressed bytes while gzip consumes a tar member stream."""

    def __init__(self, raw: BinaryIO) -> None:
        self.raw = raw
        self.sha256 = hashlib.sha256()
        self.size = 0

    def read(self, size: int = -1) -> bytes:
        data = self.raw.read(size)
        self.sha256.update(data)
        self.size += len(data)
        return data


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as file_obj:
        for chunk in iter(lambda: file_obj.read(READ_CHUNK_BYTES), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_members(
    archive: tarfile.TarFile,
) -> dict[str, tarfile.TarInfo]:
    members: dict[str, tarfile.TarInfo] = {}
    normalized_names: set[str] = set()
    try:
        archive_members = archive.getmembers()
    except (OSError, tarfile.TarError) as exc:
        raise OciArchiveError(
            f'cannot read OCI archive members: {exc}'
        ) from exc
    for member in archive_members:
        name = member.name
        path = PurePosixPath(name)
        path_text = name[:-1] if name.endswith('/') else name
        path_parts = path_text.split('/')
        if (
            not name
            or name.startswith('/')
            or '\\' in name
            or any(part in ('', '.', '..') for part in path_parts)
        ):
            raise OciArchiveError(f'unsafe archive member path: {name!r}')
        normalized = path.as_posix()
        if normalized in normalized_names:
            raise OciArchiveError(
                f'duplicate archive member path: {normalized!r}'
            )
        normalized_names.add(normalized)
        if not (member.isfile() or member.isdir()):
            raise OciArchiveError(
                f'unsupported archive member type: {name!r}'
            )
        members[normalized] = member
    return members


def _member_bytes(
    archive: tarfile.TarFile,
    members: dict[str, tarfile.TarInfo],
    name: str,
    *,
    expected_size: int | None = None,
) -> bytes:
    member = members.get(name)
    if member is None or not member.isfile():
        raise OciArchiveError(f'missing regular archive member: {name}')
    if expected_size is not None and member.size != expected_size:
        raise OciArchiveError(
            f'{name} member size {member.size} does not match descriptor '
            f'size {expected_size}'
        )
    if member.size > MAX_CONTROL_FILE_BYTES:
        raise OciArchiveError(f'control file is unexpectedly large: {name}')
    file_obj = archive.extractfile(member)
    if file_obj is None:
        raise OciArchiveError(f'cannot read archive member: {name}')
    with file_obj:
        payload = file_obj.read()
    if len(payload) != member.size:
        raise OciArchiveError(f'truncated archive member: {name}')
    return payload


def _json_object(payload: bytes, label: str) -> dict[str, Any]:
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise OciArchiveError(f'{label} is not valid JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise OciArchiveError(f'{label} root is not an object')
    return value


def _descriptor_fields(
    descriptor: object,
    label: str,
) -> tuple[str, str, int]:
    if not isinstance(descriptor, dict):
        raise OciArchiveError(f'{label} descriptor is not an object')
    media_type = descriptor.get('mediaType')
    digest = descriptor.get('digest')
    size = descriptor.get('size')
    if not isinstance(media_type, str) or not media_type:
        raise OciArchiveError(f'{label} descriptor has no mediaType')
    if not isinstance(digest, str) or DIGEST_PATTERN.fullmatch(digest) is None:
        raise OciArchiveError(f'{label} descriptor has invalid digest')
    if isinstance(size, bool) or not isinstance(size, int) or size < 0:
        raise OciArchiveError(f'{label} descriptor has invalid size')
    return media_type, digest, size


def _blob_name(digest: str) -> str:
    match = DIGEST_PATTERN.fullmatch(digest)
    if match is None:
        raise OciArchiveError(f'unsupported blob digest: {digest!r}')
    return f'blobs/sha256/{match.group(1)}'


def _verified_control_blob(
    archive: tarfile.TarFile,
    members: dict[str, tarfile.TarInfo],
    descriptor: object,
    label: str,
) -> tuple[bytes, str, str, int]:
    media_type, digest, size = _descriptor_fields(descriptor, label)
    payload = _member_bytes(
        archive,
        members,
        _blob_name(digest),
        expected_size=size,
    )
    actual = 'sha256:' + hashlib.sha256(payload).hexdigest()
    if actual != digest:
        raise OciArchiveError(
            f'{label} blob digest {actual} does not match {digest}'
        )
    return payload, media_type, digest, size


def _verified_layer(
    archive: tarfile.TarFile,
    members: dict[str, tarfile.TarInfo],
    descriptor: object,
    expected_diff_id: str,
    position: int,
) -> dict[str, Any]:
    label = f'layer {position}'
    media_type, digest, size = _descriptor_fields(descriptor, label)
    if media_type != OCI_GZIP_LAYER_MEDIA_TYPE:
        raise OciArchiveError(
            f'{label} mediaType must be {OCI_GZIP_LAYER_MEDIA_TYPE}, '
            f'found {media_type}'
        )
    if DIGEST_PATTERN.fullmatch(expected_diff_id) is None:
        raise OciArchiveError(f'{label} has invalid config diffID')
    name = _blob_name(digest)
    member = members.get(name)
    if member is None or not member.isfile():
        raise OciArchiveError(f'missing regular layer blob: {name}')
    if member.size != size:
        raise OciArchiveError(
            f'{label} member size {member.size} does not match descriptor '
            f'size {size}'
        )
    raw = archive.extractfile(member)
    if raw is None:
        raise OciArchiveError(f'cannot read {label} blob')
    compressed = _DigestingReader(raw)
    uncompressed_digest = hashlib.sha256()
    uncompressed_size = 0
    try:
        with raw, gzip.GzipFile(fileobj=compressed, mode='rb') as gzip_file:
            for chunk in iter(
                lambda: gzip_file.read(READ_CHUNK_BYTES),
                b'',
            ):
                uncompressed_digest.update(chunk)
                uncompressed_size += len(chunk)
    except (EOFError, OSError) as exc:
        raise OciArchiveError(
            f'{label} is not a complete gzip stream'
        ) from exc
    actual_digest = 'sha256:' + compressed.sha256.hexdigest()
    if compressed.size != size:
        raise OciArchiveError(
            f'{label} consumed {compressed.size} compressed bytes, '
            f'expected {size}'
        )
    if actual_digest != digest:
        raise OciArchiveError(
            f'{label} blob digest {actual_digest} does not match {digest}'
        )
    actual_diff_id = 'sha256:' + uncompressed_digest.hexdigest()
    if actual_diff_id != expected_diff_id:
        raise OciArchiveError(
            f'{label} diffID {actual_diff_id} does not match '
            f'{expected_diff_id}'
        )
    return {
        'position': position,
        'digest': digest,
        'media_type': media_type,
        'compressed_bytes': size,
        'diff_id': actual_diff_id,
        'uncompressed_bytes': uncompressed_size,
    }


def _labels(config: dict[str, Any]) -> dict[str, str]:
    config_section = config.get('config')
    if not isinstance(config_section, dict):
        raise OciArchiveError('image config.config is not an object')
    labels = config_section.get('Labels')
    if not isinstance(labels, dict):
        raise OciArchiveError('image config has no labels object')
    if not all(
        isinstance(key, str) and isinstance(value, str)
        for key, value in labels.items()
    ):
        raise OciArchiveError('image labels must have string keys and values')
    return dict(sorted(labels.items()))


def _check_expected_label(
    labels: dict[str, str],
    key: str,
    expected: str | None,
) -> None:
    if expected is not None and labels.get(key) != expected:
        raise OciArchiveError(
            f'image label {key}={labels.get(key)!r}, expected {expected!r}'
        )


def _gate(
    candidate_bytes: int,
    baseline_bytes: int,
    baseline_reference: str,
    baseline_index_digest: str,
    baseline_manifest_digest: str,
    minimum_reduction_percent: Decimal,
) -> dict[str, Any]:
    if baseline_bytes <= 0:
        raise OciArchiveError('baseline compressed bytes must be positive')
    if not Decimal('0') <= minimum_reduction_percent < Decimal('100'):
        raise OciArchiveError(
            'minimum reduction percent must be at least 0 and below 100'
        )
    if not baseline_reference.strip():
        raise OciArchiveError('baseline reference must not be empty')
    for label, digest in (
        ('baseline index', baseline_index_digest),
        ('baseline manifest', baseline_manifest_digest),
    ):
        if DIGEST_PATTERN.fullmatch(digest) is None:
            raise OciArchiveError(f'{label} digest is invalid')
    fraction = (
        Decimal('100') - minimum_reduction_percent
    ) / Decimal('100')
    ceiling = int(
        (Decimal(baseline_bytes) * fraction).to_integral_value(
            rounding=ROUND_CEILING,
        )
    )
    reduction = baseline_bytes - candidate_bytes
    reduction_percent = (
        Decimal(reduction) * Decimal('100') / Decimal(baseline_bytes)
    )
    status = 'PASS' if candidate_bytes <= ceiling else 'FAIL'
    return {
        'status': status,
        'baseline_reference': baseline_reference,
        'baseline_index_digest': baseline_index_digest,
        'baseline_manifest_digest': baseline_manifest_digest,
        'baseline_compressed_layer_bytes': baseline_bytes,
        'minimum_reduction_percent': float(minimum_reduction_percent),
        'ceiling_compressed_layer_bytes': ceiling,
        'reduction_bytes': reduction,
        'reduction_percent': float(
            reduction_percent.quantize(Decimal('0.000001'))
        ),
        'margin_bytes': ceiling - candidate_bytes,
    }


def measure_oci_archive(
    path: Path,
    *,
    os_name: str = 'linux',
    architecture: str = 'amd64',
    expected_revision: str | None = None,
    expected_version: str | None = None,
    baseline_compressed_bytes: int | None = None,
    baseline_reference: str | None = None,
    baseline_index_digest: str | None = None,
    baseline_manifest_digest: str | None = None,
    minimum_reduction_percent: Decimal = Decimal('25'),
) -> dict[str, Any]:
    """Return a schema-valid measurement after verifying the OCI graph."""
    baseline_identity = (
        baseline_reference,
        baseline_index_digest,
        baseline_manifest_digest,
    )
    if baseline_compressed_bytes is not None and any(
        value is None for value in baseline_identity
    ):
        raise OciArchiveError(
            'baseline bytes require reference, index digest, and '
            'platform manifest digest'
        )
    if baseline_compressed_bytes is None and any(
        value is not None for value in baseline_identity
    ):
        raise OciArchiveError(
            'baseline identity cannot be recorded without compressed bytes'
        )
    archive_path = path.expanduser().resolve()
    if not archive_path.is_file():
        raise OciArchiveError(f'OCI archive is not a regular file: {path}')
    archive_size = archive_path.stat().st_size
    archive_sha256 = _sha256_file(archive_path)

    try:
        archive = tarfile.open(archive_path, mode='r:*')
    except (OSError, tarfile.TarError) as exc:
        raise OciArchiveError(f'cannot open OCI archive: {exc}') from exc
    with archive:
        members = _safe_members(archive)
        layout = _json_object(
            _member_bytes(archive, members, 'oci-layout'),
            'oci-layout',
        )
        if layout != {'imageLayoutVersion': '1.0.0'}:
            raise OciArchiveError('unsupported oci-layout content')
        index = _json_object(
            _member_bytes(archive, members, 'index.json'),
            'OCI index',
        )
        if index.get('schemaVersion') != 2:
            raise OciArchiveError('OCI index schemaVersion must be 2')
        if index.get('mediaType') != OCI_INDEX_MEDIA_TYPE:
            raise OciArchiveError(
                f'OCI index mediaType must be {OCI_INDEX_MEDIA_TYPE}'
            )
        manifests = index.get('manifests')
        if not isinstance(manifests, list) or len(manifests) != 1:
            raise OciArchiveError(
                'OCI index must contain exactly one platform manifest'
            )
        manifest_descriptor = manifests[0]
        if not isinstance(manifest_descriptor, dict):
            raise OciArchiveError('platform manifest descriptor is invalid')
        platform = manifest_descriptor.get('platform')
        if platform != {'architecture': architecture, 'os': os_name}:
            raise OciArchiveError(
                f'platform must be {os_name}/{architecture}, found {platform}'
            )
        manifest_payload, manifest_media_type, manifest_digest, _ = (
            _verified_control_blob(
                archive,
                members,
                manifest_descriptor,
                'manifest',
            )
        )
        if manifest_media_type != OCI_MANIFEST_MEDIA_TYPE:
            raise OciArchiveError(
                f'manifest mediaType must be {OCI_MANIFEST_MEDIA_TYPE}'
            )
        manifest = _json_object(manifest_payload, 'OCI manifest')
        if manifest.get('schemaVersion') != 2:
            raise OciArchiveError('OCI manifest schemaVersion must be 2')
        if manifest.get('mediaType') != OCI_MANIFEST_MEDIA_TYPE:
            raise OciArchiveError('OCI manifest has an invalid mediaType')

        config_payload, config_media_type, config_digest, _ = (
            _verified_control_blob(
                archive,
                members,
                manifest.get('config'),
                'config',
            )
        )
        if config_media_type != OCI_CONFIG_MEDIA_TYPE:
            raise OciArchiveError(
                f'config mediaType must be {OCI_CONFIG_MEDIA_TYPE}'
            )
        config = _json_object(config_payload, 'OCI config')
        if config.get('os') != os_name:
            raise OciArchiveError('config operating system does not match')
        if config.get('architecture') != architecture:
            raise OciArchiveError('config architecture does not match')
        rootfs = config.get('rootfs')
        if not isinstance(rootfs, dict) or rootfs.get('type') != 'layers':
            raise OciArchiveError('config rootfs must have type layers')
        diff_ids = rootfs.get('diff_ids')
        layer_descriptors = manifest.get('layers')
        if not isinstance(diff_ids, list) or not isinstance(
            layer_descriptors,
            list,
        ):
            raise OciArchiveError('manifest layers or config diff_ids missing')
        if len(layer_descriptors) == 0:
            raise OciArchiveError('OCI image must contain at least one layer')
        if len(layer_descriptors) != len(diff_ids):
            raise OciArchiveError(
                'manifest layer count does not match config diffID count'
            )
        layers = [
            _verified_layer(
                archive,
                members,
                descriptor,
                diff_id,
                position,
            )
            for position, (descriptor, diff_id) in enumerate(
                zip(layer_descriptors, diff_ids),
                start=1,
            )
        ]
        labels = _labels(config)
        _check_expected_label(
            labels,
            'org.opencontainers.image.revision',
            expected_revision,
        )
        _check_expected_label(
            labels,
            'org.opencontainers.image.version',
            expected_version,
        )
        referenced_blobs = {
            _blob_name(manifest_digest),
            _blob_name(config_digest),
            *(_blob_name(layer['digest']) for layer in layers),
        }
        present_blobs = {
            name for name, member in members.items()
            if name.startswith('blobs/') and member.isfile()
        }
        if present_blobs != referenced_blobs:
            unexpected = sorted(present_blobs - referenced_blobs)
            missing = sorted(referenced_blobs - present_blobs)
            raise OciArchiveError(
                'archive blob closure mismatch: '
                f'unexpected={unexpected}, missing={missing}'
            )
        allowed_members = {
            'oci-layout',
            'index.json',
            'blobs',
            'blobs/sha256',
            *referenced_blobs,
        }
        unexpected_members = sorted(set(members) - allowed_members)
        if unexpected_members:
            raise OciArchiveError(
                'archive contains entries outside the OCI image closure: '
                + ', '.join(unexpected_members)
            )

    compressed_bytes = sum(layer['compressed_bytes'] for layer in layers)
    uncompressed_bytes = sum(
        layer['uncompressed_bytes'] for layer in layers
    )
    gate = None
    if baseline_compressed_bytes is not None:
        assert baseline_reference is not None
        assert baseline_index_digest is not None
        assert baseline_manifest_digest is not None
        gate = _gate(
            compressed_bytes,
            baseline_compressed_bytes,
            baseline_reference,
            baseline_index_digest,
            baseline_manifest_digest,
            minimum_reduction_percent,
        )
    status = 'PASS' if gate is None or gate['status'] == 'PASS' else 'FAIL'
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': status,
        'archive': {
            'name': archive_path.name,
            'size_bytes': archive_size,
            'sha256': archive_sha256,
            'member_count': len(members),
        },
        'platform': {
            'os': os_name,
            'architecture': architecture,
        },
        'image': {
            'manifest_digest': manifest_digest,
            'config_digest': config_digest,
            'layer_count': len(layers),
            'compressed_layer_bytes': compressed_bytes,
            'uncompressed_layer_bytes': uncompressed_bytes,
            'labels': labels,
            'layers': layers,
        },
        'verification': {
            'status': 'PASS',
            'safe_archive_members': len(members),
            'descriptors_verified': len(layers) + 2,
            'diff_ids_verified': len(layers),
            'unreferenced_blobs': 0,
            'all_layers_gzip': True,
        },
        'gate': gate,
    }
    validate_contract(report, SCHEMA_NAME)
    return report


def parse_args() -> argparse.Namespace:
    """Parse the local OCI measurement command line."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_OCI_MEASURE_COMMAND'),
        description=(
            'Verify one linux/amd64 gzip OCI archive and measure its layers.'
        ),
    )
    parser.add_argument('archive')
    parser.add_argument('--os', default='linux')
    parser.add_argument('--architecture', default='amd64')
    parser.add_argument('--expected-revision')
    parser.add_argument('--expected-version')
    parser.add_argument('--baseline-compressed-bytes', type=int)
    parser.add_argument('--baseline-reference')
    parser.add_argument('--baseline-index-digest')
    parser.add_argument('--baseline-manifest-digest')
    parser.add_argument('--minimum-reduction-percent', default='25')
    parser.add_argument('--output')
    return parser.parse_args()


def main() -> int:
    """Write JSON; distinguish invalid input from a measured gate miss."""
    args = parse_args()
    try:
        minimum = Decimal(args.minimum_reduction_percent)
        report = measure_oci_archive(
            Path(args.archive),
            os_name=args.os,
            architecture=args.architecture,
            expected_revision=args.expected_revision,
            expected_version=args.expected_version,
            baseline_compressed_bytes=args.baseline_compressed_bytes,
            baseline_reference=args.baseline_reference,
            baseline_index_digest=args.baseline_index_digest,
            baseline_manifest_digest=args.baseline_manifest_digest,
            minimum_reduction_percent=minimum,
        )
        rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
        if args.output:
            output = Path(args.output).expanduser().resolve()
            if output.exists():
                raise OciArchiveError(
                    f'refusing to overwrite OCI measurement: {output}'
                )
            output.write_text(rendered, encoding='utf-8')
        else:
            print(rendered, end='')
    except (InvalidOperation, OciArchiveError, OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
