#!/usr/bin/env python3
"""Audit one published onboarding fixture without mutating its host."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, TextIO

from mid360_robot_public_datasets import (
    PublicDatasetFile,
    download_verified_artifact,
)

from product_schema import validate_contract

try:
    from github_api_auth import (
        github_api_authorization,
        is_github_api_url,
    )
except ModuleNotFoundError:  # pragma: no cover - importlib test path
    from scripts.github_api_auth import (
        github_api_authorization,
        is_github_api_url,
    )


SCHEMA_NAME = 'fixture-publication-audit-v1.schema.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/fixture-publication-audit-v1.schema.json'
)
READINESS_SCHEMA = 'fixture-publication-readiness-v1.schema.json'
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')
MD5_PATTERN = re.compile(r'^[0-9a-f]{32}$')
RELEASE_TAG_PATTERN = re.compile(
    r'^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$'
)
ZENODO_RECORD_PATTERN = re.compile(r'^[1-9][0-9]{0,19}$')
MAX_REVIEW_BYTES = 16 * 1024 * 1024
MAX_METADATA_BYTES = 4 * 1024 * 1024
READ_CHUNK_BYTES = 1024 * 1024
REQUEST_ATTEMPTS = 3
RETRYABLE_HTTP_CODES = frozenset((408, 425, 429, 500, 502, 503, 504))
GITHUB_API_VERSION = '2026-03-10'
PRIVATE_PATH_MARKERS = (
    b'/home/',
    b'/tmp/',
    b'/Users/',
    b'C:\\Users\\',
)


class PublishedFixtureAuditError(ValueError):
    """The remote publication cannot support a trusted audit receipt."""


MetadataFetcher = Callable[[str], dict[str, Any]]


def _regular_file(path: Path, label: str) -> Path:
    candidate = Path(os.path.abspath(path.expanduser()))
    current = Path(candidate.anchor)
    for part in candidate.parts[1:]:
        current /= part
        if current.is_symlink():
            raise PublishedFixtureAuditError(
                f'{label} must not use symlink components: {path}'
            )
    if not candidate.is_file():
        raise PublishedFixtureAuditError(
            f'{label} is not a regular file: {path}'
        )
    return candidate


def _readiness_review(
    path: Path,
    expected_sha256: str,
) -> tuple[dict[str, Any], str]:
    if SHA256_PATTERN.fullmatch(expected_sha256) is None:
        raise PublishedFixtureAuditError(
            'expected readiness-review SHA-256 is invalid'
        )
    checked = _regular_file(path, 'readiness review')
    if checked.stat().st_size > MAX_REVIEW_BYTES:
        raise PublishedFixtureAuditError('readiness review is too large')
    payload = checked.read_bytes()
    actual_sha256 = hashlib.sha256(payload).hexdigest()
    if actual_sha256 != expected_sha256:
        raise PublishedFixtureAuditError(
            f'readiness review SHA-256 {actual_sha256} does not match '
            f'{expected_sha256}'
        )
    for marker in PRIVATE_PATH_MARKERS:
        if marker in payload:
            raise PublishedFixtureAuditError(
                'readiness review contains a private path marker'
            )
    try:
        review = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PublishedFixtureAuditError(
            f'readiness review is not readable JSON: {exc}'
        ) from exc
    if not isinstance(review, dict):
        raise PublishedFixtureAuditError(
            'readiness review root is not an object'
        )
    try:
        validate_contract(review, READINESS_SCHEMA)
    except (OSError, ValueError) as exc:
        raise PublishedFixtureAuditError(
            f'readiness review schema failed: {exc}'
        ) from exc

    publication = review['publication']
    decision = review['decision']
    provenance = review['provenance']
    if (
        review['status'] != 'PUBLICATION_READY'
        or decision['status'] != 'PUBLICATION_READY'
        or decision['blockers']
        or review['local_validation']['status'] != 'LOCAL_ARTIFACT_PASS'
        or publication['authorization'] != 'GRANTED'
        or publication['upload_performed'] is not False
        or publication['host'] not in ('github-release', 'zenodo')
        or provenance['generator_revision_remote_status'] != 'RESOLVABLE'
        or provenance['review_revision_remote_status'] != 'RESOLVABLE'
    ):
        raise PublishedFixtureAuditError(
            'readiness review is not an authorized PUBLICATION_READY packet'
        )
    return review, actual_sha256


def _request_json(url: str) -> dict[str, Any]:
    headers = {
        'Accept': 'application/json',
        'User-Agent': 'lidarslam-published-fixture-audit/1',
    }
    if is_github_api_url(url):
        headers['Accept'] = 'application/vnd.github+json'
        headers['X-GitHub-Api-Version'] = GITHUB_API_VERSION
    headers.update(github_api_authorization(url, method='GET'))
    request = urllib.request.Request(url, headers=headers)
    for attempt in range(REQUEST_ATTEMPTS):
        try:
            with urllib.request.urlopen(request, timeout=30) as response:
                payload = response.read(MAX_METADATA_BYTES + 1)
                if len(payload) > MAX_METADATA_BYTES:
                    raise PublishedFixtureAuditError(
                        'host metadata response is too large'
                    )
        except urllib.error.HTTPError as exc:
            retryable = exc.code in RETRYABLE_HTTP_CODES
            if not retryable or attempt + 1 == REQUEST_ATTEMPTS:
                raise PublishedFixtureAuditError(
                    f'host metadata request failed with HTTP {exc.code}'
                ) from exc
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            if attempt + 1 == REQUEST_ATTEMPTS:
                raise PublishedFixtureAuditError(
                    f'host metadata request failed: {exc}'
                ) from exc
        else:
            try:
                value = json.loads(payload)
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise PublishedFixtureAuditError(
                    f'host metadata is not readable JSON: {exc}'
                ) from exc
            if not isinstance(value, dict):
                raise PublishedFixtureAuditError(
                    'host metadata root is not an object'
                )
            return value
        time.sleep(2 ** attempt)
    raise AssertionError('metadata request retry loop exhausted')


def _positive_integer(value: Any, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise PublishedFixtureAuditError(f'{label} is not a positive integer')
    return value


def _https_url(value: Any, *, host: str, label: str) -> str:
    if not isinstance(value, str):
        raise PublishedFixtureAuditError(f'{label} is not a URL')
    parsed = urllib.parse.urlsplit(value)
    if (
        parsed.scheme != 'https'
        or parsed.hostname != host
        or parsed.port is not None
        or parsed.username is not None
        or parsed.password is not None
        or parsed.fragment
    ):
        raise PublishedFixtureAuditError(
            f'{label} is not a trusted {host} URL'
        )
    return value


def _github_metadata(
    review: dict[str, Any],
    release_tag: str,
    fetcher: MetadataFetcher,
) -> dict[str, Any]:
    if RELEASE_TAG_PATTERN.fullmatch(release_tag) is None:
        raise PublishedFixtureAuditError('GitHub release tag is invalid')
    repository = review['provenance']['repository']
    encoded_tag = urllib.parse.quote(release_tag, safe='')
    metadata_url = (
        f'https://api.github.com/repos/{repository}/releases/tags/'
        f'{encoded_tag}'
    )
    release = fetcher(metadata_url)
    if (
        release.get('tag_name') != release_tag
        or release.get('draft') is not False
        or release.get('prerelease') is not False
        or release.get('immutable') is not True
    ):
        raise PublishedFixtureAuditError(
            'GitHub release is not final and immutable'
        )
    release_id = _positive_integer(release.get('id'), 'GitHub release ID')
    published_at = release.get('published_at')
    if not isinstance(published_at, str) or not published_at:
        raise PublishedFixtureAuditError(
            'GitHub release has no publication timestamp'
        )
    assets = release.get('assets')
    if not isinstance(assets, list) or len(assets) > 100:
        raise PublishedFixtureAuditError(
            'GitHub release asset list is not safely bounded'
        )
    artifact = review['fixture']['artifact']
    matches = [
        asset for asset in assets
        if isinstance(asset, dict)
        and asset.get('name') == artifact['filename']
    ]
    if len(matches) != 1:
        raise PublishedFixtureAuditError(
            'GitHub release does not have the unique reviewed fixture asset'
        )
    asset = matches[0]
    asset_id = _positive_integer(asset.get('id'), 'GitHub asset ID')
    expected_digest = f"sha256:{artifact['sha256']}"
    if (
        asset.get('state') != 'uploaded'
        or asset.get('size') != artifact['size_bytes']
        or asset.get('digest') != expected_digest
    ):
        raise PublishedFixtureAuditError(
            'GitHub asset metadata does not match the reviewed artifact'
        )
    download_count = asset.get('download_count')
    if (
        not isinstance(download_count, int)
        or isinstance(download_count, bool)
        or download_count < 0
    ):
        raise PublishedFixtureAuditError(
            'GitHub asset download count is not a non-negative integer'
        )
    expected_url = (
        f'https://github.com/{repository}/releases/download/'
        f'{encoded_tag}/'
        f'{urllib.parse.quote(artifact["filename"], safe="")}'
    )
    download_url = _https_url(
        asset.get('browser_download_url'),
        host='github.com',
        label='GitHub asset download URL',
    )
    if download_url != expected_url:
        raise PublishedFixtureAuditError(
            'GitHub asset download URL does not match its release and name'
        )
    return {
        'publication': {
            'host': 'github-release',
            'record_id': str(release_id),
            'asset_id': str(asset_id),
            'release_tag': release_tag,
            'doi': None,
            'published_at': published_at,
            'public_access': True,
            'version_pinned': True,
            'host_immutability': 'ENFORCED',
            'metadata_url': metadata_url,
        },
        'remote_asset': {
            'filename': artifact['filename'],
            'size_bytes': artifact['size_bytes'],
            'digest_algorithm': 'sha256',
            'digest': artifact['sha256'],
            'download_url': download_url,
            'download_count': download_count,
            'state': 'PUBLISHED',
        },
    }


def _zenodo_metadata(
    review: dict[str, Any],
    record_id: str,
    fetcher: MetadataFetcher,
) -> dict[str, Any]:
    if ZENODO_RECORD_PATTERN.fullmatch(record_id) is None:
        raise PublishedFixtureAuditError('Zenodo record ID is invalid')
    metadata_url = f'https://zenodo.org/api/records/{record_id}'
    record = fetcher(metadata_url)
    numeric_record_id = int(record_id)
    metadata = record.get('metadata')
    if not isinstance(metadata, dict):
        raise PublishedFixtureAuditError(
            'Zenodo record has no metadata object'
        )
    if (
        record.get('id') != numeric_record_id
        or record.get('status') != 'published'
        or record.get('submitted') is not True
        or metadata.get('access_right') != 'open'
    ):
        raise PublishedFixtureAuditError(
            'Zenodo record is not the requested open published version'
        )
    expected_doi = f'10.5281/zenodo.{record_id}'
    if record.get('doi') != expected_doi:
        raise PublishedFixtureAuditError(
            'Zenodo record DOI does not identify the version record'
        )
    published_at = record.get('created')
    if not isinstance(published_at, str) or not published_at:
        raise PublishedFixtureAuditError(
            'Zenodo record has no publication timestamp'
        )
    links = record.get('links')
    if not isinstance(links, dict):
        raise PublishedFixtureAuditError('Zenodo record has no links object')
    if _https_url(
        links.get('self'),
        host='zenodo.org',
        label='Zenodo record API URL',
    ) != metadata_url:
        raise PublishedFixtureAuditError(
            'Zenodo record API URL does not match its version ID'
        )
    files = record.get('files')
    if not isinstance(files, list) or len(files) > 100:
        raise PublishedFixtureAuditError(
            'Zenodo record file list is not safely bounded'
        )
    artifact = review['fixture']['artifact']
    matches = [
        item for item in files
        if isinstance(item, dict) and item.get('key') == artifact['filename']
    ]
    if len(matches) != 1:
        raise PublishedFixtureAuditError(
            'Zenodo record does not have the unique reviewed fixture file'
        )
    remote_file = matches[0]
    checksum = remote_file.get('checksum')
    if (
        remote_file.get('size') != artifact['size_bytes']
        or not isinstance(checksum, str)
        or not checksum.startswith('md5:')
        or MD5_PATTERN.fullmatch(checksum[4:]) is None
    ):
        raise PublishedFixtureAuditError(
            'Zenodo file metadata does not match the reviewed artifact size'
        )
    asset_id = remote_file.get('id')
    if (
        not isinstance(asset_id, str)
        or re.fullmatch(r'[A-Za-z0-9-]{1,128}', asset_id) is None
    ):
        raise PublishedFixtureAuditError(
            'Zenodo file ID is invalid'
        )
    file_links = remote_file.get('links')
    if not isinstance(file_links, dict):
        raise PublishedFixtureAuditError(
            'Zenodo file has no links object'
        )
    expected_url = (
        f'https://zenodo.org/api/records/{record_id}/files/'
        f'{urllib.parse.quote(artifact["filename"], safe="")}/content'
    )
    download_url = _https_url(
        file_links.get('self'),
        host='zenodo.org',
        label='Zenodo file download URL',
    )
    if download_url != expected_url:
        raise PublishedFixtureAuditError(
            'Zenodo file download URL does not match its version and name'
        )
    return {
        'publication': {
            'host': 'zenodo',
            'record_id': record_id,
            'asset_id': asset_id,
            'release_tag': None,
            'doi': expected_doi,
            'published_at': published_at,
            'public_access': True,
            'version_pinned': True,
            'host_immutability': 'CHECKSUM_PINNED_VERSION',
            'metadata_url': metadata_url,
        },
        'remote_asset': {
            'filename': artifact['filename'],
            'size_bytes': artifact['size_bytes'],
            'digest_algorithm': 'md5',
            'digest': checksum[4:],
            'download_url': download_url,
            'download_count': None,
            'state': 'PUBLISHED',
        },
    }


def _output_directory(path: Path) -> Path:
    candidate = Path(os.path.abspath(path.expanduser()))
    current = Path(candidate.anchor)
    for part in candidate.parts[1:]:
        current /= part
        if current.is_symlink():
            raise PublishedFixtureAuditError(
                f'audit output directory must not use symlinks: {path}'
            )
    if candidate.exists() and not candidate.is_dir():
        raise PublishedFixtureAuditError(
            f'audit output path is not a directory: {path}'
        )
    candidate.mkdir(parents=True, exist_ok=True)
    return candidate


def _hash_artifact(path: Path) -> tuple[int, dict[str, str]]:
    checked = _regular_file(path, 'downloaded fixture')
    hashers = {
        'sha256': hashlib.sha256(),
        'md5': hashlib.md5(usedforsecurity=False),
    }
    size = 0
    with checked.open('rb') as stream:
        for chunk in iter(lambda: stream.read(READ_CHUNK_BYTES), b''):
            size += len(chunk)
            for hasher in hashers.values():
                hasher.update(chunk)
    return size, {
        name: hasher.hexdigest()
        for name, hasher in hashers.items()
    }


def _verified_download(
    raw: dict[str, Any],
    *,
    size_bytes: int,
    digests: dict[str, str],
    artifact: dict[str, Any],
    remote_asset: dict[str, Any],
) -> dict[str, Any]:
    if (
        raw.get('status') != 'VERIFIED'
        or raw.get('source') not in {
            'network', 'resume', 'complete-part', 'cache'
        }
        or raw.get('expected_size_bytes') != artifact['size_bytes']
        or raw.get('expected_sha256') != artifact['sha256']
        or raw.get('size_bytes') != size_bytes
        or raw.get('sha256') != digests['sha256']
        or raw.get('md5') != digests['md5']
        or raw.get('size_verified') is not True
        or raw.get('sha256_verified') is not True
    ):
        raise PublishedFixtureAuditError(
            'download report does not match the independently verified file'
        )
    if (
        size_bytes != artifact['size_bytes']
        or digests['sha256'] != artifact['sha256']
    ):
        raise PublishedFixtureAuditError(
            'downloaded fixture does not match the reviewed artifact'
        )
    algorithm = remote_asset['digest_algorithm']
    if digests[algorithm] != remote_asset['digest']:
        raise PublishedFixtureAuditError(
            'downloaded file does not match the host-reported digest'
        )
    resumed = raw.get('resumed_bytes')
    transferred = raw.get('transferred_bytes')
    if (
        not isinstance(resumed, int)
        or isinstance(resumed, bool)
        or resumed < 0
        or not isinstance(transferred, int)
        or isinstance(transferred, bool)
        or transferred < 0
    ):
        raise PublishedFixtureAuditError(
            'download metadata has invalid byte totals'
        )
    expected_totals = {
        'network': (0, size_bytes),
        'cache': (0, 0),
        'complete-part': (size_bytes, 0),
    }
    source = raw['source']
    if source == 'resume':
        totals_valid = (
            0 < resumed < size_bytes
            and resumed + transferred == size_bytes
        )
    else:
        totals_valid = (resumed, transferred) == expected_totals[source]
    if not totals_valid:
        raise PublishedFixtureAuditError(
            'download source and byte totals are inconsistent'
        )
    return {
        'status': 'VERIFIED',
        'source': source,
        'expected_size_bytes': artifact['size_bytes'],
        'expected_sha256': artifact['sha256'],
        'size_bytes': size_bytes,
        'sha256': digests['sha256'],
        'md5': digests['md5'],
        'resumed_bytes': resumed,
        'transferred_bytes': transferred,
        'remote_digest_verified': True,
    }


def audit_published_fixture(
    readiness_review: Path,
    *,
    expected_review_sha256: str,
    output_dir: Path,
    github_release_tag: str | None = None,
    zenodo_record_id: str | None = None,
    force: bool = False,
    audited_at: str | None = None,
    metadata_fetcher: MetadataFetcher | None = None,
    downloader: Callable[..., tuple[dict[str, Any], list[str]]] | None = None,
    progress_stream: TextIO | None = sys.stderr,
) -> dict[str, Any]:
    """Return a privacy-bounded receipt for one exact published fixture."""
    review, review_sha256 = _readiness_review(
        readiness_review,
        expected_review_sha256,
    )
    host = review['publication']['host']
    fetcher = metadata_fetcher or _request_json
    if host == 'github-release':
        if github_release_tag is None or zenodo_record_id is not None:
            raise PublishedFixtureAuditError(
                'GitHub readiness requires only --github-release-tag'
            )
        host_report = _github_metadata(
            review,
            github_release_tag,
            fetcher,
        )
    elif host == 'zenodo':
        if zenodo_record_id is None or github_release_tag is not None:
            raise PublishedFixtureAuditError(
                'Zenodo readiness requires only --zenodo-record-id'
            )
        host_report = _zenodo_metadata(
            review,
            zenodo_record_id,
            fetcher,
        )
    else:
        raise PublishedFixtureAuditError(
            f'unsupported publication host: {host!r}'
        )

    artifact = review['fixture']['artifact']
    remote_asset = host_report['remote_asset']
    destination = _output_directory(output_dir) / artifact['filename']
    file_record = PublicDatasetFile(
        id=review['fixture']['fixture_id'],
        filename=artifact['filename'],
        url=remote_asset['download_url'],
        md5=(
            remote_asset['digest']
            if remote_asset['digest_algorithm'] == 'md5'
            else ''
        ),
        sha256=artifact['sha256'],
        size_bytes=artifact['size_bytes'],
    )
    download = downloader or download_verified_artifact
    raw_download, _ = download(
        file_record,
        destination,
        force=force,
        verify_md5=True,
        progress_stream=progress_stream,
    )
    size_bytes, digests = _hash_artifact(destination)
    download_report = _verified_download(
        raw_download,
        size_bytes=size_bytes,
        digests=digests,
        artifact=artifact,
        remote_asset=remote_asset,
    )
    provenance = review['provenance']
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': 'REMOTE_ARTIFACT_PASS',
        'audited_at': audited_at or datetime.now(timezone.utc).isoformat(),
        'readiness': {
            'review_id': review['review']['review_id'],
            'reviewed_on': review['review']['reviewed_on'],
            'review_sha256': review_sha256,
            'fixture_id': review['fixture']['fixture_id'],
            'artifact': dict(artifact),
            'repository': provenance['repository'],
            'generator_revision': provenance['generator_revision'],
            'review_revision': provenance['review_revision'],
            'host': host,
        },
        'publication': host_report['publication'],
        'remote_asset': remote_asset,
        'download': download_report,
        'checks': [
            {'id': check_id, 'status': 'PASS'}
            for check_id in (
                'readiness_review_identity',
                'publication_decision',
                'public_host_metadata',
                'version_identity',
                'remote_asset_identity',
                'downloaded_size',
                'downloaded_sha256',
                'remote_digest',
            )
        ],
    }
    try:
        validate_contract(report, SCHEMA_NAME)
    except (OSError, ValueError) as exc:
        raise PublishedFixtureAuditError(
            f'published fixture audit schema failed: {exc}'
        ) from exc
    rendered = json.dumps(report, sort_keys=True).encode()
    for marker in PRIVATE_PATH_MARKERS:
        if marker in rendered:
            raise PublishedFixtureAuditError(
                'published fixture audit contains a private path marker'
            )
    return report


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    """Parse the remote, read-only publication audit command line."""
    parser = argparse.ArgumentParser(
        description=(
            'Verify host metadata and a downloaded fixture against an '
            'authorized publication-readiness review.'
        )
    )
    parser.add_argument('--readiness-review', required=True)
    parser.add_argument('--expected-review-sha256', required=True)
    parser.add_argument('--output-dir', required=True)
    host = parser.add_mutually_exclusive_group(required=True)
    host.add_argument('--github-release-tag')
    host.add_argument('--zenodo-record-id')
    parser.add_argument(
        '--force',
        action='store_true',
        help='Restart a partial transfer and atomically replace an archive.',
    )
    parser.add_argument(
        '--output',
        help='Write the privacy-bounded audit JSON; refuses overwrite.',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run the audit without uploading, editing, or deleting host state."""
    args = parse_args(argv)
    try:
        output = Path(args.output).expanduser() if args.output else None
        if output is not None and (output.exists() or output.is_symlink()):
            raise PublishedFixtureAuditError(
                f'refusing to overwrite audit report: {output}'
            )
        report = audit_published_fixture(
            Path(args.readiness_review),
            expected_review_sha256=args.expected_review_sha256,
            output_dir=Path(args.output_dir),
            github_release_tag=args.github_release_tag,
            zenodo_record_id=args.zenodo_record_id,
            force=args.force,
        )
        rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
        if output is None:
            print(rendered, end='')
        else:
            output.parent.mkdir(parents=True, exist_ok=True)
            with output.open('x', encoding='utf-8') as stream:
                stream.write(rendered)
            print(f'Published fixture audit: {output}')
    except (PublishedFixtureAuditError, OSError, ValueError) as exc:
        print(f'published fixture audit error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
