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

"""Tests for the host-aware published fixture audit."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from typing import Any

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
EVIDENCE = ROOT / 'docs' / 'evidence' / 'onboarding'
sys.path.insert(0, str(SCRIPTS))
SCRIPT = SCRIPTS / 'audit_published_fixture.py'
SPEC = importlib.util.spec_from_file_location(
    'audit_published_fixture', SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
AUDITOR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDITOR)


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _md5(payload: bytes) -> str:
    return hashlib.md5(payload).hexdigest()


def _ready_review(
    tmp_path: Path,
    *,
    host: str,
    payload: bytes,
) -> tuple[Path, str, dict[str, Any]]:
    review = json.loads((
        EVIDENCE /
        'mid360-onboarding-50s-v1-publication-review-20260811.json'
    ).read_text(encoding='utf-8'))
    review['status'] = 'PUBLICATION_READY'
    review['fixture']['artifact'] = {
        'filename': 'fixture.zip',
        'size_bytes': len(payload),
        'sha256': _sha256(payload),
        'member_count': 3,
    }
    review['provenance']['generator_revision_remote_status'] = 'RESOLVABLE'
    review['provenance']['review_revision_remote_status'] = 'RESOLVABLE'
    review['publication'] = {
        'host': host,
        'authorization': 'GRANTED',
        'upload_performed': False,
        'immutable_record_id': None,
        'immutable_artifact_url': None,
    }
    review['decision'] = {
        'status': 'PUBLICATION_READY',
        'blockers': [],
        'next_actions': [
            'Upload only the verified artifact identity, then audit it.'
        ],
    }
    path = tmp_path / 'review.json'
    path.write_text(
        json.dumps(review, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    digest = _sha256(path.read_bytes())
    AUDITOR.validate_contract(review, AUDITOR.READINESS_SCHEMA)
    return path, digest, review


def _downloader(payload: bytes):
    def download(file_record, destination, **_kwargs):
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(payload)
        sha256 = _sha256(payload)
        md5 = _md5(payload)
        return ({
            'status': 'VERIFIED',
            'source': 'network',
            'expected_size_bytes': file_record.size_bytes,
            'expected_sha256': file_record.sha256,
            'expected_md5': file_record.md5 or None,
            'size_bytes': len(payload),
            'sha256': sha256,
            'md5': md5,
            'size_verified': len(payload) == file_record.size_bytes,
            'sha256_verified': sha256 == file_record.sha256,
            'md5_verified': (
                md5 == file_record.md5 if file_record.md5 else None
            ),
            'resumed_bytes': 0,
            'transferred_bytes': len(payload),
        }, [])

    return download


def _github_metadata(
    review: dict[str, Any],
    payload: bytes,
    *,
    immutable: bool = True,
    digest: str | None = None,
) -> dict[str, Any]:
    repository = review['provenance']['repository']
    return {
        'id': 123,
        'tag_name': 'fixture-v1',
        'draft': False,
        'prerelease': False,
        'immutable': immutable,
        'published_at': '2026-08-11T00:00:00Z',
        'assets': [{
            'id': 456,
            'name': 'fixture.zip',
            'state': 'uploaded',
            'size': len(payload),
            'digest': digest or f'sha256:{_sha256(payload)}',
            'download_count': 0,
            'browser_download_url': (
                f'https://github.com/{repository}/releases/download/'
                'fixture-v1/fixture.zip'
            ),
        }],
    }


def _zenodo_metadata(
    payload: bytes,
    *,
    status: str = 'published',
    access_right: str = 'open',
) -> dict[str, Any]:
    return {
        'id': 987654,
        'status': status,
        'submitted': True,
        'doi': '10.5281/zenodo.987654',
        'created': '2026-08-11T00:00:00+00:00',
        'metadata': {'access_right': access_right},
        'links': {
            'self': 'https://zenodo.org/api/records/987654',
        },
        'files': [{
            'id': 'abc123-def456',
            'key': 'fixture.zip',
            'size': len(payload),
            'checksum': f'md5:{_md5(payload)}',
            'links': {
                'self': (
                    'https://zenodo.org/api/records/987654/files/'
                    'fixture.zip/content'
                ),
            },
        }],
    }


def test_github_immutable_release_download_passes(tmp_path):
    """The GitHub metadata and independently downloaded bytes must agree."""
    payload = b'published github fixture payload\n'
    review_path, review_sha256, review = _ready_review(
        tmp_path,
        host='github-release',
        payload=payload,
    )
    requested_urls = []

    def fetch(url):
        requested_urls.append(url)
        return _github_metadata(review, payload)

    report = AUDITOR.audit_published_fixture(
        review_path,
        expected_review_sha256=review_sha256,
        output_dir=tmp_path / 'downloads',
        github_release_tag='fixture-v1',
        metadata_fetcher=fetch,
        downloader=_downloader(payload),
        audited_at='2026-08-11T01:02:03Z',
        progress_stream=None,
    )

    assert requested_urls == [
        'https://api.github.com/repos/'
        f"{review['provenance']['repository']}/releases/tags/fixture-v1"
    ]
    assert report['status'] == 'REMOTE_ARTIFACT_PASS'
    assert report['publication']['host_immutability'] == 'ENFORCED'
    assert report['publication']['record_id'] == '123'
    assert report['remote_asset']['digest_algorithm'] == 'sha256'
    assert report['download']['sha256'] == _sha256(payload)
    assert report['download']['remote_digest_verified'] is True
    assert len(report['checks']) == 8
    assert all(item['status'] == 'PASS' for item in report['checks'])
    rendered = json.dumps(report)
    assert str(tmp_path) not in rendered
    assert '/home/' not in rendered
    AUDITOR.validate_contract(report, AUDITOR.SCHEMA_NAME)


def test_github_metadata_client_uses_scoped_read_credential(monkeypatch):
    """Release metadata auth is attached only to the exact GitHub API GET."""
    captured = {}

    class Response:
        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self, _limit):
            return b'{"id": 123}'

    def urlopen(request, timeout):
        captured['request'] = request
        captured['timeout'] = timeout
        return Response()

    monkeypatch.setenv('GITHUB_TOKEN', 'read-only-test-token')
    monkeypatch.setattr(AUDITOR.urllib.request, 'urlopen', urlopen)

    payload = AUDITOR._request_json(
        'https://api.github.com/repos/owner/repo/releases/tags/fixture-v1'
    )

    request = captured['request']
    assert payload == {'id': 123}
    assert request.get_method() == 'GET'
    assert request.data is None
    assert request.get_header('Authorization') == (
        'Bearer read-only-test-token'
    )
    assert captured['timeout'] == 30


def test_github_release_must_have_immutability_enabled(tmp_path):
    """A final release is insufficient when its assets can still change."""
    payload = b'fixture\n'
    review_path, review_sha256, review = _ready_review(
        tmp_path,
        host='github-release',
        payload=payload,
    )

    with pytest.raises(
        AUDITOR.PublishedFixtureAuditError,
        match='not final and immutable',
    ):
        AUDITOR.audit_published_fixture(
            review_path,
            expected_review_sha256=review_sha256,
            output_dir=tmp_path / 'downloads',
            github_release_tag='fixture-v1',
            metadata_fetcher=lambda _url: _github_metadata(
                review, payload, immutable=False
            ),
            downloader=lambda *_args, **_kwargs: pytest.fail(
                'download must not run'
            ),
        )


def test_github_metadata_digest_must_match_review(tmp_path):
    """The host-reported SHA cannot drift from the authorized packet."""
    payload = b'fixture\n'
    review_path, review_sha256, review = _ready_review(
        tmp_path,
        host='github-release',
        payload=payload,
    )

    with pytest.raises(
        AUDITOR.PublishedFixtureAuditError,
        match='does not match the reviewed artifact',
    ):
        AUDITOR.audit_published_fixture(
            review_path,
            expected_review_sha256=review_sha256,
            output_dir=tmp_path / 'downloads',
            github_release_tag='fixture-v1',
            metadata_fetcher=lambda _url: _github_metadata(
                review,
                payload,
                digest='sha256:' + ('0' * 64),
            ),
            downloader=lambda *_args, **_kwargs: pytest.fail(
                'download must not run'
            ),
        )


def test_zenodo_published_version_download_passes(tmp_path):
    """Zenodo's version ID and MD5 complement the reviewed SHA-256 identity."""
    payload = b'published zenodo fixture payload\n'
    review_path, review_sha256, _ = _ready_review(
        tmp_path,
        host='zenodo',
        payload=payload,
    )

    report = AUDITOR.audit_published_fixture(
        review_path,
        expected_review_sha256=review_sha256,
        output_dir=tmp_path / 'downloads',
        zenodo_record_id='987654',
        metadata_fetcher=lambda _url: _zenodo_metadata(payload),
        downloader=_downloader(payload),
        audited_at='2026-08-11T01:02:03Z',
        progress_stream=None,
    )

    assert report['publication']['host'] == 'zenodo'
    assert report['publication']['host_immutability'] == (
        'CHECKSUM_PINNED_VERSION'
    )
    assert report['publication']['doi'] == '10.5281/zenodo.987654'
    assert report['remote_asset']['digest_algorithm'] == 'md5'
    assert report['remote_asset']['digest'] == _md5(payload)
    assert report['download']['sha256'] == _sha256(payload)
    assert report['download']['remote_digest_verified'] is True


@pytest.mark.parametrize(
    'status, access_right',
    [('draft', 'open'), ('published', 'restricted')],
)
def test_zenodo_record_must_be_published_and_open(
    tmp_path,
    status,
    access_right,
):
    """A draft or restricted record is not a public onboarding route."""
    payload = b'fixture\n'
    review_path, review_sha256, _ = _ready_review(
        tmp_path,
        host='zenodo',
        payload=payload,
    )

    with pytest.raises(
        AUDITOR.PublishedFixtureAuditError,
        match='not the requested open published version',
    ):
        AUDITOR.audit_published_fixture(
            review_path,
            expected_review_sha256=review_sha256,
            output_dir=tmp_path / 'downloads',
            zenodo_record_id='987654',
            metadata_fetcher=lambda _url: _zenodo_metadata(
                payload,
                status=status,
                access_right=access_right,
            ),
            downloader=lambda *_args, **_kwargs: pytest.fail(
                'download must not run'
            ),
        )


def test_waiting_review_cannot_be_used_as_audit_authority(tmp_path):
    """Local artifact PASS does not authorize a remote publication audit."""
    path = (
        EVIDENCE /
        'mid360-onboarding-50s-v1-publication-review-20260811.json'
    )
    digest = _sha256(path.read_bytes())

    with pytest.raises(
        AUDITOR.PublishedFixtureAuditError,
        match='not an authorized PUBLICATION_READY packet',
    ):
        AUDITOR.audit_published_fixture(
            path,
            expected_review_sha256=digest,
            output_dir=tmp_path / 'downloads',
            github_release_tag='fixture-v1',
            metadata_fetcher=lambda _url: pytest.fail(
                'metadata request must not run'
            ),
            downloader=lambda *_args, **_kwargs: pytest.fail(
                'download must not run'
            ),
        )


def test_readiness_review_identity_is_pinned(tmp_path):
    """A different review document cannot silently authorize the audit."""
    payload = b'fixture\n'
    review_path, _, _ = _ready_review(
        tmp_path,
        host='github-release',
        payload=payload,
    )

    with pytest.raises(
        AUDITOR.PublishedFixtureAuditError,
        match='does not match',
    ):
        AUDITOR.audit_published_fixture(
            review_path,
            expected_review_sha256='0' * 64,
            output_dir=tmp_path / 'downloads',
            github_release_tag='fixture-v1',
        )


def test_downloaded_bytes_are_rehashed_independently(tmp_path):
    """A downloader report cannot conceal bytes unlike the reviewed ZIP."""
    payload = b'fixture\n'
    changed = b'X' + payload[1:]
    review_path, review_sha256, review = _ready_review(
        tmp_path,
        host='github-release',
        payload=payload,
    )

    with pytest.raises(
        AUDITOR.PublishedFixtureAuditError,
        match='does not match the independently verified file',
    ):
        AUDITOR.audit_published_fixture(
            review_path,
            expected_review_sha256=review_sha256,
            output_dir=tmp_path / 'downloads',
            github_release_tag='fixture-v1',
            metadata_fetcher=lambda _url: _github_metadata(
                review, payload
            ),
            downloader=_downloader(changed),
            progress_stream=None,
        )


def test_download_source_and_byte_totals_must_agree(tmp_path):
    """Transfer evidence cannot claim a network fetch with zero bytes."""
    payload = b'fixture\n'
    review_path, review_sha256, review = _ready_review(
        tmp_path,
        host='github-release',
        payload=payload,
    )
    base_downloader = _downloader(payload)

    def inconsistent_download(*args, **kwargs):
        report, messages = base_downloader(*args, **kwargs)
        report['transferred_bytes'] = 0
        return report, messages

    with pytest.raises(
        AUDITOR.PublishedFixtureAuditError,
        match='source and byte totals are inconsistent',
    ):
        AUDITOR.audit_published_fixture(
            review_path,
            expected_review_sha256=review_sha256,
            output_dir=tmp_path / 'downloads',
            github_release_tag='fixture-v1',
            metadata_fetcher=lambda _url: _github_metadata(
                review, payload
            ),
            downloader=inconsistent_download,
            progress_stream=None,
        )


def test_audit_output_directory_must_not_be_a_symlink(tmp_path):
    """The downloaded public fixture stays inside the selected audit root."""
    payload = b'fixture\n'
    review_path, review_sha256, review = _ready_review(
        tmp_path,
        host='github-release',
        payload=payload,
    )
    real_output = tmp_path / 'real-output'
    real_output.mkdir()
    linked_output = tmp_path / 'linked-output'
    linked_output.symlink_to(real_output, target_is_directory=True)

    with pytest.raises(
        AUDITOR.PublishedFixtureAuditError,
        match='must not use symlinks',
    ):
        AUDITOR.audit_published_fixture(
            review_path,
            expected_review_sha256=review_sha256,
            output_dir=linked_output,
            github_release_tag='fixture-v1',
            metadata_fetcher=lambda _url: _github_metadata(
                review, payload
            ),
            downloader=lambda *_args, **_kwargs: pytest.fail(
                'download must not run'
            ),
            progress_stream=None,
        )

    assert not list(real_output.iterdir())


def test_review_host_and_cli_selector_must_agree(tmp_path):
    """Host selection remains bound to the authorized readiness review."""
    payload = b'fixture\n'
    review_path, review_sha256, _ = _ready_review(
        tmp_path,
        host='zenodo',
        payload=payload,
    )

    with pytest.raises(
        AUDITOR.PublishedFixtureAuditError,
        match='Zenodo readiness requires only',
    ):
        AUDITOR.audit_published_fixture(
            review_path,
            expected_review_sha256=review_sha256,
            output_dir=tmp_path / 'downloads',
            github_release_tag='fixture-v1',
        )


def test_audit_tool_and_schema_are_in_the_release_bundle():
    """A maintainer can reproduce the remote audit from a release bundle."""
    release_builder = (SCRIPTS / 'build_release_bundle.py').read_text(
        encoding='utf-8'
    )

    assert 'scripts/audit_published_fixture.py' in release_builder
    assert (ROOT / 'docs' / 'schemas' / AUDITOR.SCHEMA_NAME).is_file()


def test_cli_help_describes_host_specific_identity():
    """The command exposes mutually exclusive host-version selectors."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT), '--help'],
        check=True,
        capture_output=True,
        text=True,
        cwd=ROOT,
    )

    assert '--github-release-tag' in result.stdout
    assert '--zenodo-record-id' in result.stdout
    assert '--expected-review-sha256' in result.stdout
