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

"""Fail closed before publishing the geometry-bearing onboarding fixture."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import stat
import sys
import zipfile
from datetime import date
from pathlib import Path, PurePosixPath
from typing import Any, BinaryIO

from product_schema import validate_contract


SCHEMA_NAME = 'fixture-publication-readiness-v1.schema.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/fixture-publication-readiness-v1.schema.json'
)
FIXTURE_SCHEMA = 'mid360-onboarding-fixture-v1.schema.json'
RECEIPT_SCHEMA = 'first-map-validation-receipt-v1.schema.json'
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')
COMMIT_PATTERN = re.compile(r'^[0-9a-f]{40}$')
SLUG_PATTERN = re.compile(r'^[a-z0-9][a-z0-9._-]{2,79}$')
REPOSITORY_PATTERN = re.compile(
    r'^[A-Za-z0-9_.-]{1,100}/[A-Za-z0-9_.-]{1,100}$'
)
REMOTE_STATUSES = frozenset({'RESOLVABLE', 'UNRESOLVABLE', 'NOT_CHECKED'})
HOSTS = frozenset({'github-release', 'zenodo'})
READ_CHUNK_BYTES = 1024 * 1024
MAX_JSON_BYTES = 16 * 1024 * 1024
ZIP_MEMBER_TIMESTAMP = (1980, 1, 1, 0, 0, 0)
REQUIRED_RECEIPT_CHECKS = frozenset({
    'manifest_succeeded',
    'lifecycle_complete',
    'runner_exit_zero',
    'diagnosis_success',
    'autoware_verification_pass',
    'diagnosis_bound_to_manifest',
    'verify_log_bound_to_manifest',
})
PRIVATE_PATH_MARKERS = (
    b'/home/',
    b'/tmp/',
    b'/Users/',
    b'C:\\Users\\',
)


class FixturePublicationError(ValueError):
    """Local evidence cannot support a trusted publication decision."""


def _sha256_stream(stream: BinaryIO) -> tuple[int, str]:
    digest = hashlib.sha256()
    size = 0
    for chunk in iter(lambda: stream.read(READ_CHUNK_BYTES), b''):
        digest.update(chunk)
        size += len(chunk)
    return size, digest.hexdigest()


def _sha256_file(path: Path) -> tuple[int, str]:
    with path.open('rb') as stream:
        return _sha256_stream(stream)


def _regular_path(path: Path, label: str) -> Path:
    candidate = Path(os.path.abspath(path.expanduser()))
    current = Path(candidate.anchor)
    for part in candidate.parts[1:]:
        current /= part
        if current.is_symlink():
            raise FixturePublicationError(
                f'{label} must not use symlink components: {path}'
            )
    if not candidate.is_file():
        raise FixturePublicationError(
            f'{label} is not a regular file: {path}'
        )
    return candidate


def _json_object(
    path: Path,
    label: str,
    schema_name: str,
    expected_sha256: str,
) -> tuple[dict[str, Any], dict[str, Any], bytes]:
    if SHA256_PATTERN.fullmatch(expected_sha256) is None:
        raise FixturePublicationError(
            f'{label} expected SHA-256 is invalid'
        )
    checked_path = _regular_path(path, label)
    payload = checked_path.read_bytes()
    if len(payload) > MAX_JSON_BYTES:
        raise FixturePublicationError(f'{label} is unexpectedly large')
    actual_sha256 = hashlib.sha256(payload).hexdigest()
    if actual_sha256 != expected_sha256:
        raise FixturePublicationError(
            f'{label} SHA-256 {actual_sha256} does not match '
            f'{expected_sha256}'
        )
    try:
        value = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise FixturePublicationError(
            f'{label} is not readable JSON: {exc}'
        ) from exc
    if not isinstance(value, dict):
        raise FixturePublicationError(f'{label} root is not an object')
    try:
        validate_contract(value, schema_name)
    except (OSError, ValueError) as exc:
        raise FixturePublicationError(f'{label} schema failed: {exc}') from exc
    for marker in PRIVATE_PATH_MARKERS:
        if marker in payload:
            raise FixturePublicationError(
                f'{label} contains a private path marker'
            )
    identity = {
        'filename': checked_path.name,
        'size_bytes': len(payload),
        'sha256': actual_sha256,
        'schema_status': 'PASS',
    }
    return value, identity, payload


def _safe_member_name(name: str, fixture_id: str) -> PurePosixPath:
    path = PurePosixPath(name)
    if (
        not name
        or name.startswith('/')
        or name.endswith('/')
        or '\\' in name
        or '\x00' in name
        or path.as_posix() != name
        or any(part in ('', '.', '..') for part in name.split('/'))
        or not path.parts
        or path.parts[0] != fixture_id
    ):
        raise FixturePublicationError(
            f'unsafe or wrong-root ZIP member: {name!r}'
        )
    return path


def _expected_bag_members(manifest: dict[str, Any]) -> set[str]:
    fixture_id = manifest['fixture_id']
    bag = manifest['clip']['bag']
    directory = bag['directory_name']
    members = {
        manifest['publication']['attribution_member'],
        f"{fixture_id}/{directory}/{bag['metadata']['filename']}",
    }
    members.update(
        f"{fixture_id}/{directory}/{row['path']}"
        for row in bag['storage_files']
    )
    return members


def _attribution_fields(
    payload: bytes,
    manifest: dict[str, Any],
) -> list[str]:
    try:
        text = payload.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise FixturePublicationError(
            'ATTRIBUTION.md is not valid UTF-8'
        ) from exc
    source = manifest['source']
    required = {
        'creator': source['creator'],
        'title': source['title'],
        'source_url': source['source_url'],
        'doi': f"https://doi.org/{source['doi']}",
        'citation': source['citation'],
        'license': source['license_name'],
        'license_url': source['license_url'],
        'change_notice': 'Changes made by the lidar_slam_ros2 project',
        'onboarding_limit': 'for onboarding only',
        'full_gate_limit': 'does not replace the full 277-second',
        'no_endorsement': (
            'No endorsement by the original creator is implied.'
        ),
    }
    missing = [
        field for field, snippet in required.items()
        if snippet not in text
    ]
    if missing:
        raise FixturePublicationError(
            'ATTRIBUTION.md is missing required fields: '
            + ', '.join(missing)
        )
    return sorted(required)


def _verify_archive(
    artifact_path: Path,
    manifest: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any], list[str]]:
    checked_path = _regular_path(artifact_path, 'fixture artifact')
    expected_artifact = manifest['artifact']
    if checked_path.name != expected_artifact['filename']:
        raise FixturePublicationError(
            'fixture artifact filename does not match the build manifest'
        )
    artifact_size, artifact_sha256 = _sha256_file(checked_path)
    if artifact_size != expected_artifact['size_bytes']:
        raise FixturePublicationError(
            f'fixture artifact size {artifact_size} does not match '
            f"{expected_artifact['size_bytes']}"
        )
    if artifact_sha256 != expected_artifact['sha256']:
        raise FixturePublicationError(
            f'fixture artifact SHA-256 {artifact_sha256} does not match '
            f"{expected_artifact['sha256']}"
        )
    if artifact_size > expected_artifact['max_size_bytes']:
        raise FixturePublicationError('fixture artifact exceeds its size gate')

    expected_rows = expected_artifact['members']
    expected_names = [row['path'] for row in expected_rows]
    if expected_names != sorted(expected_names):
        raise FixturePublicationError(
            'build manifest archive members are not sorted'
        )
    if len(expected_names) != len(set(expected_names)):
        raise FixturePublicationError(
            'build manifest archive members are not unique'
        )
    if set(expected_names) != _expected_bag_members(manifest):
        raise FixturePublicationError(
            'build manifest archive members do not match the bag closure'
        )
    expected = {row['path']: row for row in expected_rows}
    attribution_name = manifest['publication']['attribution_member']
    attribution_payload: bytes | None = None
    verified_bytes = 0

    try:
        archive = zipfile.ZipFile(checked_path, mode='r')
    except (OSError, zipfile.BadZipFile) as exc:
        raise FixturePublicationError(
            f'fixture artifact is not a readable ZIP: {exc}'
        ) from exc
    try:
        with archive:
            if archive.comment:
                raise FixturePublicationError(
                    'ZIP archive comment is forbidden'
                )
            infos = archive.infolist()
            names = [info.filename for info in infos]
            if names != sorted(names):
                raise FixturePublicationError('ZIP members are not sorted')
            if len(names) != len(set(names)):
                raise FixturePublicationError('ZIP has duplicate member names')
            if len({name.casefold() for name in names}) != len(names):
                raise FixturePublicationError(
                    'ZIP has case-insensitive duplicate member names'
                )
            for name in names:
                _safe_member_name(name, manifest['fixture_id'])
            if set(names) != set(expected):
                unexpected = sorted(set(names) - set(expected))
                missing = sorted(set(expected) - set(names))
                raise FixturePublicationError(
                    'ZIP member closure mismatch: '
                    f'unexpected={unexpected}, missing={missing}'
                )

            for info in infos:
                row = expected[info.filename]
                mode = info.external_attr >> 16
                if info.is_dir() or not stat.S_ISREG(mode):
                    raise FixturePublicationError(
                        f'ZIP member is not a regular file: {info.filename}'
                    )
                if info.create_system != 3 or stat.S_IMODE(mode) != 0o644:
                    raise FixturePublicationError(
                        f'ZIP member mode is not Unix 0644: {info.filename}'
                    )
                if info.compress_type != zipfile.ZIP_DEFLATED:
                    raise FixturePublicationError(
                        f'ZIP member is not deflated: {info.filename}'
                    )
                if info.flag_bits & 0x41:
                    raise FixturePublicationError(
                        f'ZIP member is encrypted: {info.filename}'
                    )
                if info.date_time != ZIP_MEMBER_TIMESTAMP:
                    raise FixturePublicationError(
                        f'ZIP member timestamp is not fixed: {info.filename}'
                    )
                if info.extra or info.comment:
                    raise FixturePublicationError(
                        f'ZIP member metadata is not empty: {info.filename}'
                    )
                if info.file_size != row['size_bytes']:
                    raise FixturePublicationError(
                        f'ZIP member size mismatch: {info.filename}'
                    )
                try:
                    with archive.open(info, mode='r') as stream:
                        size, digest = _sha256_stream(stream)
                except (OSError, RuntimeError, zipfile.BadZipFile) as exc:
                    raise FixturePublicationError(
                        'ZIP member cannot be verified: '
                        f'{info.filename}: {exc}'
                    ) from exc
                if size != row['size_bytes'] or digest != row['sha256']:
                    raise FixturePublicationError(
                        f'ZIP member digest mismatch: {info.filename}'
                    )
                verified_bytes += size
                if info.filename == attribution_name:
                    if size > 64 * 1024:
                        raise FixturePublicationError(
                            'ATTRIBUTION.md is unexpectedly large'
                        )
                    with archive.open(info, mode='r') as stream:
                        attribution_payload = stream.read()
    except zipfile.BadZipFile as exc:
        raise FixturePublicationError(
            f'fixture ZIP integrity failed: {exc}'
        ) from exc

    if attribution_payload is None:
        raise FixturePublicationError('ZIP has no attribution member')
    attribution_fields = _attribution_fields(attribution_payload, manifest)
    artifact_identity = {
        'filename': checked_path.name,
        'size_bytes': artifact_size,
        'sha256': artifact_sha256,
        'member_count': len(expected),
    }
    archive_report = {
        'safe_member_count': len(expected),
        'expected_member_count': len(expected),
        'member_digests_verified': len(expected),
        'member_bytes_verified': verified_bytes,
        'unexpected_members': 0,
        'sorted_members': True,
        'fixed_timestamp': True,
        'unix_mode_0644': True,
        'deflate_only': True,
        'encrypted_members': 0,
        'archive_comment_empty': True,
        'member_metadata_empty': True,
    }
    return artifact_identity, archive_report, attribution_fields


def _verify_receipt(
    receipt: dict[str, Any],
    generator_revision: str,
) -> dict[str, Any]:
    checks = receipt['checks']
    check_ids = [row['id'] for row in checks]
    if len(check_ids) != len(set(check_ids)):
        raise FixturePublicationError('map receipt check IDs are not unique')
    if set(check_ids) != REQUIRED_RECEIPT_CHECKS:
        raise FixturePublicationError(
            'map receipt does not contain the exact seven required checks'
        )
    if receipt['status'] != 'PASS' or not all(
        row['passed'] for row in checks
    ):
        raise FixturePublicationError('map receipt is not seven-of-seven PASS')
    if receipt['run']['git_commit'] != generator_revision:
        raise FixturePublicationError(
            'map receipt revision does not match the fixture generator'
        )
    if receipt['run']['profile_id'] != 'rko_lio_graph_mid360_preset':
        raise FixturePublicationError(
            'map receipt used the wrong product profile'
        )
    verification = receipt['verification']
    if (
        verification['manifest_status'] != 'succeeded'
        or verification['diagnosis_status'] != 'success'
        or verification['autoware_status'] != 'PASS'
    ):
        raise FixturePublicationError(
            'map receipt verification outcome is not successful'
        )
    evidence = receipt['evidence']
    if evidence['manifest']['sha256'] != verification['manifest_sha256']:
        raise FixturePublicationError(
            'map receipt manifest evidence is not hash-bound'
        )
    for key in ('diagnosis', 'verify_log'):
        if not evidence[key]['available'] or evidence[key]['sha256'] is None:
            raise FixturePublicationError(
                f'map receipt {key} evidence is unavailable'
            )
    return {
        'status': 'PASS',
        'passed_checks': len(checks),
        'total_checks': len(checks),
        'generator_revision_match': True,
        'geometry_free': True,
    }


def _validate_review_inputs(
    *,
    review_id: str,
    reviewed_on: str,
    repository: str,
    review_revision: str,
    generator_revision_remote_status: str,
    review_revision_remote_status: str,
    clean_rebuilds: int,
    rebuild_artifacts_byte_identical: bool,
    rebuild_manifests_byte_identical: bool,
    host: str | None,
) -> None:
    if SLUG_PATTERN.fullmatch(review_id) is None:
        raise FixturePublicationError('review ID is invalid')
    try:
        parsed_date = date.fromisoformat(reviewed_on)
    except ValueError as exc:
        raise FixturePublicationError('review date is invalid') from exc
    if parsed_date.isoformat() != reviewed_on:
        raise FixturePublicationError('review date must use YYYY-MM-DD')
    if REPOSITORY_PATTERN.fullmatch(repository) is None:
        raise FixturePublicationError('repository identity is invalid')
    if COMMIT_PATTERN.fullmatch(review_revision) is None:
        raise FixturePublicationError('review revision is invalid')
    for label, value in (
        ('generator revision', generator_revision_remote_status),
        ('review revision', review_revision_remote_status),
    ):
        if value not in REMOTE_STATUSES:
            raise FixturePublicationError(
                f'{label} remote status is invalid: {value}'
            )
    if clean_rebuilds < 2:
        raise FixturePublicationError(
            'at least two clean fixture rebuilds are required'
        )
    if not rebuild_artifacts_byte_identical:
        raise FixturePublicationError(
            'clean rebuild artifact bytes are not attested identical'
        )
    if not rebuild_manifests_byte_identical:
        raise FixturePublicationError(
            'clean rebuild manifest bytes are not attested identical'
        )
    if host is not None and host not in HOSTS:
        raise FixturePublicationError(f'unsupported publication host: {host}')


def _publication_decision(
    *,
    generator_remote_status: str,
    review_remote_status: str,
    host: str | None,
    upload_authorized: bool,
) -> dict[str, Any]:
    blockers: list[str] = []
    actions: list[str] = []
    if generator_remote_status != 'RESOLVABLE':
        suffix = (
            'not-checked'
            if generator_remote_status == 'NOT_CHECKED'
            else 'not-remotely-resolvable'
        )
        blockers.append(f'generator-revision-{suffix}')
        actions.append(
            'Make the exact clean generator revision resolvable from the '
            'public repository, then repeat the read-only commit check.'
        )
    if review_remote_status != 'RESOLVABLE':
        suffix = (
            'not-checked'
            if review_remote_status == 'NOT_CHECKED'
            else 'not-remotely-resolvable'
        )
        blockers.append(f'review-revision-{suffix}')
        actions.append(
            'Make the reviewed product revision publicly resolvable before '
            'using it for source onboarding trials.'
        )
    if host is None:
        blockers.append('publication-host-unset')
        actions.append(
            'Choose either a checksum-pinned GitHub Release asset or a '
            'metadata-rich Zenodo derivative record.'
        )
    if not upload_authorized:
        blockers.append('explicit-upload-authorization-missing')
        actions.append(
            'Obtain an explicit maintainer decision before uploading the '
            'geometry-bearing fixture.'
        )
    if blockers:
        actions.append(
            'Run this gate again and require PUBLICATION_READY before any '
            'upload or default-download change.'
        )
        status = 'AWAITING_PUBLICATION_DECISION'
    else:
        actions.append(
            'Upload only the verified artifact identity, then perform a '
            'separate immutable remote-download audit.'
        )
        status = 'PUBLICATION_READY'
    return {
        'status': status,
        'blockers': blockers,
        'next_actions': actions,
    }


def review_fixture_publication(
    artifact_path: Path,
    manifest_path: Path,
    receipt_path: Path,
    *,
    expected_manifest_sha256: str,
    expected_receipt_sha256: str,
    review_id: str,
    reviewed_on: str,
    repository: str,
    review_revision: str,
    generator_revision_remote_status: str,
    review_revision_remote_status: str,
    clean_rebuilds: int,
    rebuild_artifacts_byte_identical: bool,
    rebuild_manifests_byte_identical: bool,
    host: str | None = None,
    upload_authorized: bool = False,
) -> dict[str, Any]:
    """Return a schema-valid local review without publishing anything."""
    _validate_review_inputs(
        review_id=review_id,
        reviewed_on=reviewed_on,
        repository=repository,
        review_revision=review_revision,
        generator_revision_remote_status=(
            generator_revision_remote_status
        ),
        review_revision_remote_status=review_revision_remote_status,
        clean_rebuilds=clean_rebuilds,
        rebuild_artifacts_byte_identical=(
            rebuild_artifacts_byte_identical
        ),
        rebuild_manifests_byte_identical=(
            rebuild_manifests_byte_identical
        ),
        host=host,
    )
    manifest, manifest_identity, _ = _json_object(
        manifest_path,
        'fixture build manifest',
        FIXTURE_SCHEMA,
        expected_manifest_sha256,
    )
    receipt, receipt_identity, _ = _json_object(
        receipt_path,
        'first-map receipt',
        RECEIPT_SCHEMA,
        expected_receipt_sha256,
    )
    generator_revision = manifest['generation']['revision']['git_commit']
    if manifest['generation']['revision']['git_dirty']:
        raise FixturePublicationError(
            'fixture generator revision was dirty'
        )
    artifact_identity, archive_report, attribution_fields = _verify_archive(
        artifact_path,
        manifest,
    )
    receipt_report = _verify_receipt(receipt, generator_revision)
    receipt_identity.update(receipt_report)

    checks = [
        {'id': 'fixture_manifest_schema', 'status': 'PASS'},
        {'id': 'manifest_identity', 'status': 'PASS'},
        {'id': 'artifact_identity', 'status': 'PASS'},
        {'id': 'archive_member_safety', 'status': 'PASS'},
        {'id': 'archive_recipe', 'status': 'PASS'},
        {'id': 'archive_member_closure', 'status': 'PASS'},
        {'id': 'archive_member_digests', 'status': 'PASS'},
        {'id': 'attribution_and_limits', 'status': 'PASS'},
        {'id': 'map_receipt_schema', 'status': 'PASS'},
        {'id': 'map_receipt_outcome', 'status': 'PASS'},
        {'id': 'map_receipt_revision', 'status': 'PASS'},
        {'id': 'clean_rebuild_reproducibility', 'status': 'PASS'},
        {'id': 'geometry_privacy_boundary', 'status': 'PASS'},
    ]
    decision = _publication_decision(
        generator_remote_status=generator_revision_remote_status,
        review_remote_status=review_revision_remote_status,
        host=host,
        upload_authorized=upload_authorized,
    )
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': decision['status'],
        'review': {
            'review_id': review_id,
            'reviewed_on': reviewed_on,
        },
        'fixture': {
            'fixture_id': manifest['fixture_id'],
            'artifact': artifact_identity,
            'build_manifest': manifest_identity,
            'map_receipt': receipt_identity,
        },
        'provenance': {
            'repository': repository,
            'generator_revision': generator_revision,
            'generator_revision_remote_status': (
                generator_revision_remote_status
            ),
            'review_revision': review_revision,
            'review_revision_remote_status': review_revision_remote_status,
            'clean_rebuilds': clean_rebuilds,
            'rebuild_artifacts_byte_identical': True,
            'rebuild_manifests_byte_identical': True,
        },
        'local_validation': {
            'status': 'LOCAL_ARTIFACT_PASS',
            'checks': checks,
            'archive': archive_report,
            'attribution': {
                'status': 'PASS',
                'member': manifest['publication']['attribution_member'],
                'verified_fields': attribution_fields,
            },
            'private_path_markers_found': 0,
        },
        'publication': {
            'host': host,
            'authorization': (
                'GRANTED' if upload_authorized else 'NOT_GRANTED'
            ),
            'upload_performed': False,
            'immutable_record_id': None,
            'immutable_artifact_url': None,
        },
        'decision': decision,
    }
    try:
        validate_contract(report, SCHEMA_NAME)
    except (OSError, ValueError) as exc:
        raise FixturePublicationError(
            f'publication report schema failed: {exc}'
        ) from exc
    return report


def parse_args() -> argparse.Namespace:
    """Parse the local, non-publishing review command line."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_FIXTURE_PUBLICATION_COMMAND'),
        description=(
            'Verify a fixture publication packet without uploading it.'
        ),
    )
    parser.add_argument('artifact')
    parser.add_argument('--manifest', required=True)
    parser.add_argument('--map-receipt', required=True)
    parser.add_argument('--expected-manifest-sha256', required=True)
    parser.add_argument('--expected-map-receipt-sha256', required=True)
    parser.add_argument('--review-id', required=True)
    parser.add_argument('--reviewed-on', required=True)
    parser.add_argument(
        '--repository', default='rsasaki0109/lidar_slam_ros2'
    )
    parser.add_argument('--review-revision', required=True)
    parser.add_argument(
        '--generator-revision-remote-status',
        choices=sorted(REMOTE_STATUSES),
        required=True,
    )
    parser.add_argument(
        '--review-revision-remote-status',
        choices=sorted(REMOTE_STATUSES),
        required=True,
    )
    parser.add_argument('--clean-rebuilds', type=int, required=True)
    parser.add_argument(
        '--rebuild-artifacts-byte-identical', action='store_true'
    )
    parser.add_argument(
        '--rebuild-manifests-byte-identical', action='store_true'
    )
    parser.add_argument('--host', choices=sorted(HOSTS))
    parser.add_argument('--upload-authorized', action='store_true')
    parser.add_argument('--output')
    return parser.parse_args()


def main() -> int:
    """Write a review; return one while publication still needs a decision."""
    args = parse_args()
    try:
        report = review_fixture_publication(
            Path(args.artifact),
            Path(args.manifest),
            Path(args.map_receipt),
            expected_manifest_sha256=args.expected_manifest_sha256,
            expected_receipt_sha256=args.expected_map_receipt_sha256,
            review_id=args.review_id,
            reviewed_on=args.reviewed_on,
            repository=args.repository,
            review_revision=args.review_revision,
            generator_revision_remote_status=(
                args.generator_revision_remote_status
            ),
            review_revision_remote_status=(
                args.review_revision_remote_status
            ),
            clean_rebuilds=args.clean_rebuilds,
            rebuild_artifacts_byte_identical=(
                args.rebuild_artifacts_byte_identical
            ),
            rebuild_manifests_byte_identical=(
                args.rebuild_manifests_byte_identical
            ),
            host=args.host,
            upload_authorized=args.upload_authorized,
        )
        rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
        if args.output:
            output = Path(args.output).expanduser().resolve()
            if output.exists():
                raise FixturePublicationError(
                    f'refusing to overwrite publication review: {output}'
                )
            output.write_text(rendered, encoding='utf-8')
        else:
            print(rendered, end='')
    except (FixturePublicationError, OSError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    return 0 if report['status'] == 'PUBLICATION_READY' else 1


if __name__ == '__main__':
    raise SystemExit(main())
