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
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Tests for exact-revision GitHub Pages deployment provenance."""

from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'scripts'
GENERATOR_PATH = SCRIPTS / 'generate_docs_deployment_manifest.py'
AUDITOR_PATH = SCRIPTS / 'check_public_docs_deployment.py'


def _load(path: Path, name: str):
    sys.path.insert(0, str(SCRIPTS))
    try:
        spec = importlib.util.spec_from_file_location(name, path)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS))


def _html(*, source_fragment: str = '1-install-and-build-from-source') -> bytes:
    return (
        '<!doctype html><html><body>'
        '<h2 id="docker-first-map-no-ros-2-workspace">Docker</h2>'
        f'<h2 id="{source_fragment}">Source</h2>'
        '<p>For those paths, pass the output directory itself.</p>'
        '<p>The published v0.9.0 image predates the lidarslam-map report '
        'handoff command.</p>'
        '<p>Review the receipt and submit only that receipt.</p>'
        '<p>python3 scripts/create_first_map_validation_receipt.py</p>'
        '<p>share/lidarslam/product/scripts/create_first_map_validation_receipt.py</p>'
        '<p>docker image inspect</p>'
        '</body></html>'
    ).encode()


def _site(tmp_path: Path) -> tuple[Path, Path]:
    site = tmp_path / 'site'
    site.mkdir()
    (site / 'getting-started.html').write_bytes(_html())
    version = tmp_path / 'VERSION'
    version.write_text('0.9.1\n', encoding='utf-8')
    return site, version


def _manifest(tmp_path: Path) -> tuple[object, dict, bytes]:
    generator = _load(GENERATOR_PATH, 'docs_manifest_generator_test')
    site, version = _site(tmp_path)
    manifest = generator.build_manifest(site, 'a' * 40, version)
    payload = (json.dumps(manifest, sort_keys=True) + '\n').encode()
    return generator, manifest, payload


def _fetcher(auditor, manifest_payload: bytes, page_payload: bytes):
    payloads = {
        auditor.MANIFEST_URL: manifest_payload,
        auditor.SITE_URL + 'getting-started.html': page_payload,
    }

    def fetch(url: str, limit: int) -> bytes:
        payload = payloads[url]
        assert len(payload) <= limit
        return payload

    return fetch


def test_generator_binds_revision_version_page_bytes_and_routes(tmp_path: Path):
    generator, manifest, _ = _manifest(tmp_path)

    assert manifest['source_revision'] == 'a' * 40
    assert manifest['product_version'] == '0.9.1'
    assert manifest['page']['size_bytes'] == len(_html())
    assert len(manifest['page']['sha256']) == 64
    assert manifest['page']['routes'] == [
        {
            'route_id': 'docker-first-map',
            'fragment': 'docker-first-map-no-ros-2-workspace',
        },
        {
            'route_id': 'source-quickstart',
            'fragment': '1-install-and-build-from-source',
        },
    ]
    assert manifest['page']['content_markers'] == [
        'fixed-demo-output-handoff',
        'stable-image-report-boundary',
        'receipt-only-attachment-rule',
        'source-receipt-helper-fallback',
        'stable-image-receipt-helper',
        'immutable-image-identity-check',
    ]
    scripts = str(SCRIPTS)
    sys.path.insert(0, scripts)
    try:
        from product_schema import validate_contract

        validate_contract(
            manifest,
            'docs-deployment-manifest-v1.schema.json',
        )
    finally:
        sys.path.remove(scripts)
    assert generator.MANIFEST_NAME == 'docs-deployment-v1.json'


def test_generator_rejects_missing_route_and_existing_manifest(tmp_path: Path):
    generator = _load(GENERATOR_PATH, 'docs_manifest_rejection_test')
    site, version = _site(tmp_path)
    (site / 'getting-started.html').write_bytes(
        _html(source_fragment='old-source-heading')
    )
    with pytest.raises(generator.ManifestError, match='lacks canonical route'):
        generator.build_manifest(site, 'a' * 40, version)

    (site / 'getting-started.html').write_bytes(_html())
    manifest = generator.build_manifest(site, 'a' * 40, version)
    output = generator.write_manifest(site, manifest)
    with pytest.raises(generator.ManifestError, match='refusing to overwrite'):
        generator.write_manifest(site, manifest)
    assert json.loads(output.read_text(encoding='utf-8')) == manifest


def test_generator_rejects_missing_first_map_handoff_marker(tmp_path: Path):
    generator = _load(GENERATOR_PATH, 'docs_manifest_content_rejection_test')
    site, version = _site(tmp_path)
    page = site / 'getting-started.html'
    page.write_bytes(page.read_bytes().replace(
        b'pass the output directory itself',
        b'pass the output directory',
    ))
    with pytest.raises(generator.ManifestError, match='content marker'):
        generator.build_manifest(site, 'a' * 40, version)


def test_generator_rejects_schema_invalid_manifest_before_write(
    tmp_path: Path,
):
    """Invalid manifest bytes never enter the Pages artifact."""
    generator, manifest, _ = _manifest(tmp_path)
    invalid_root = tmp_path / 'invalid'
    invalid_root.mkdir()
    site, _ = _site(invalid_root)
    manifest['source_revision'] = 'not-an-exact-revision'

    with pytest.raises(generator.ManifestError, match='schema failed'):
        generator.write_manifest(site, manifest)

    assert not (site / generator.MANIFEST_NAME).exists()


def test_generator_rejects_symlinked_page(tmp_path: Path):
    generator = _load(GENERATOR_PATH, 'docs_manifest_symlink_test')
    site, version = _site(tmp_path)
    page = site / 'getting-started.html'
    target = tmp_path / 'outside.html'
    target.write_bytes(page.read_bytes())
    page.unlink()
    page.symlink_to(target)

    with pytest.raises(generator.ManifestError, match='non-symlink'):
        generator.build_manifest(site, 'a' * 40, version)


def test_auditor_verifies_exact_manifest_page_and_route(tmp_path: Path):
    _, manifest, manifest_payload = _manifest(tmp_path)
    auditor = _load(AUDITOR_PATH, 'docs_deployment_auditor_success_test')
    report = auditor.audit_deployment(
        'a' * 40,
        '0.9.1',
        'source-quickstart',
        fetcher=_fetcher(auditor, manifest_payload, _html()),
    )

    assert report['status'] == 'VERIFIED'
    assert report['finding_codes'] == []
    assert all(report['checks'].values())
    assert report['observed']['page_sha256'] == manifest['page']['sha256']
    assert report['network_requested'] is True
    assert report['writes_performed'] is False


def test_auditor_rejects_revision_and_version_drift(tmp_path: Path):
    _, _, manifest_payload = _manifest(tmp_path)
    auditor = _load(AUDITOR_PATH, 'docs_deployment_auditor_drift_test')
    report = auditor.audit_deployment(
        'b' * 40,
        '0.9.2',
        'docker-first-map',
        fetcher=_fetcher(auditor, manifest_payload, _html()),
    )

    assert report['status'] == 'NOT_READY'
    assert report['finding_codes'] == [
        'source-revision-mismatch',
        'product-version-mismatch',
    ]


def test_auditor_rejects_page_tampering(tmp_path: Path):
    _, _, manifest_payload = _manifest(tmp_path)
    auditor = _load(AUDITOR_PATH, 'docs_deployment_auditor_tamper_test')
    report = auditor.audit_deployment(
        'a' * 40,
        '0.9.1',
        'source-quickstart',
        fetcher=_fetcher(auditor, manifest_payload, _html() + b' '),
    )

    assert report['status'] == 'NOT_READY'
    assert 'page-size-mismatch' in report['finding_codes']
    assert 'page-digest-mismatch' in report['finding_codes']


def test_auditor_rejects_manifest_route_pairing_drift(tmp_path: Path):
    _, manifest, _ = _manifest(tmp_path)
    auditor = _load(AUDITOR_PATH, 'docs_deployment_auditor_route_test')
    changed = copy.deepcopy(manifest)
    changed['page']['routes'][0]['fragment'] = (
        '1-install-and-build-from-source'
    )
    payload = (json.dumps(changed, sort_keys=True) + '\n').encode()
    report = auditor.audit_deployment(
        'a' * 40,
        '0.9.1',
        'docker-first-map',
        fetcher=_fetcher(auditor, payload, _html()),
    )

    assert report['status'] == 'NOT_READY'
    assert report['finding_codes'] == ['manifest-route-contract-mismatch']


def test_auditor_rejects_missing_first_map_handoff_marker(tmp_path: Path):
    _, _, manifest_payload = _manifest(tmp_path)
    auditor = _load(AUDITOR_PATH, 'docs_deployment_auditor_content_test')
    page = _html().replace(
        b'pass the output directory itself',
        b'pass the output directory',
    )
    report = auditor.audit_deployment(
        'a' * 40,
        '0.9.1',
        'docker-first-map',
        fetcher=_fetcher(auditor, manifest_payload, page),
    )

    assert report['status'] == 'NOT_READY'
    assert 'content-marker-missing' in report['finding_codes']
    assert 'fixed-demo-output-handoff' in report['detail']
    assert report['checks']['manifest_content_contract_valid'] is True
    assert report['checks']['content_markers_present'] is False


def test_auditor_rejects_legacy_manifest_without_content_contract(tmp_path: Path):
    _, manifest, _ = _manifest(tmp_path)
    legacy = copy.deepcopy(manifest)
    del legacy['page']['content_markers']
    payload = (json.dumps(legacy, sort_keys=True) + '\n').encode()
    auditor = _load(AUDITOR_PATH, 'docs_deployment_auditor_legacy_test')
    report = auditor.audit_deployment(
        'a' * 40,
        '0.9.1',
        'source-quickstart',
        fetcher=_fetcher(auditor, payload, _html()),
    )

    assert report['status'] == 'NOT_READY'
    assert report['finding_codes'] == ['manifest-content-contract-mismatch']
    assert report['checks']['manifest_schema_valid'] is True
    assert report['checks']['manifest_content_contract_valid'] is False


def test_auditor_reports_unavailable_manifest_without_writes():
    auditor = _load(AUDITOR_PATH, 'docs_deployment_auditor_blocked_test')

    def unavailable(url: str, limit: int) -> bytes:
        del limit
        raise auditor.FetchError(f'{url} returned HTTP 404')

    report = auditor.audit_deployment(
        'a' * 40,
        '0.9.1',
        'source-quickstart',
        fetcher=unavailable,
    )

    assert report['status'] == 'BLOCKED'
    assert report['finding_codes'] == ['manifest-unavailable']
    assert report['writes_performed'] is False
