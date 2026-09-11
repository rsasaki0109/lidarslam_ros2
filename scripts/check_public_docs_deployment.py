#!/usr/bin/env python3
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

"""Read-only audit that public first-map docs match one exact source commit."""

from __future__ import annotations

import argparse
import hashlib
from html.parser import HTMLParser
import json
import re
import sys
from typing import Any, Callable, Sequence
import urllib.error
import urllib.request

from docs_deployment_contract import (
    manifest_content_markers,
    missing_content_markers,
)
from product_schema import validate_contract


REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
SITE_URL = 'https://rsasaki0109.github.io/lidar_slam_ros2/'
MANIFEST_URL = SITE_URL + 'docs-deployment-v1.json'
MANIFEST_SCHEMA_NAME = 'docs-deployment-manifest-v1.schema.json'
REPORT_SCHEMA_NAME = 'docs-deployment-audit-v1.schema.json'
REPORT_SCHEMA_URI = SITE_URL + 'schemas/docs-deployment-audit-v1.schema.json'
MAX_MANIFEST_BYTES = 256 * 1024
MAX_PAGE_BYTES = 16 * 1024 * 1024
REVISION_PATTERN = re.compile(r'^[0-9a-f]{40}$')
VERSION_PATTERN = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')
ROUTES = {
    'docker-first-map': 'docker-first-map-no-ros-2-workspace',
    'source-quickstart': '1-install-and-build-from-source',
}


class DocsAuditError(ValueError):
    """The audit command or fetched public evidence is invalid."""


class FetchError(DocsAuditError):
    """Public documentation could not be read safely."""


class _IdCollector(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.ids: set[str] = set()

    def handle_starttag(
        self,
        tag: str,
        attrs: list[tuple[str, str | None]],
    ) -> None:
        del tag
        for name, value in attrs:
            if name == 'id' and value is not None:
                self.ids.add(value)


Fetcher = Callable[[str, int], bytes]


def _fetch(url: str, limit: int) -> bytes:
    request = urllib.request.Request(
        url,
        headers={
            'Accept': 'application/json, text/html;q=0.9',
            'User-Agent': 'lidarslam-public-docs-audit/1',
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            if response.status != 200:
                raise FetchError(f'{url} returned HTTP {response.status}')
            final_url = response.geturl()
            if final_url != url:
                raise FetchError(
                    f'{url} redirected outside its exact identity to {final_url}'
                )
            payload = response.read(limit + 1)
    except urllib.error.HTTPError as exc:
        raise FetchError(f'{url} returned HTTP {exc.code}') from exc
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise FetchError(f'cannot read {url}: {exc}') from exc
    if len(payload) > limit:
        raise FetchError(f'{url} exceeds the {limit}-byte response limit')
    if not payload:
        raise FetchError(f'{url} returned an empty response')
    return payload


def _base_report(
    expected_revision: str,
    expected_version: str,
    route: str,
) -> dict[str, Any]:
    return {
        'schema_version': 1,
        'schema_uri': REPORT_SCHEMA_URI,
        'status': 'BLOCKED',
        'repository': REPOSITORY,
        'manifest_url': MANIFEST_URL,
        'network_requested': True,
        'writes_performed': False,
        'expectations': {
            'source_revision': expected_revision,
            'product_version': expected_version,
            'route_id': route,
        },
        'observed': {
            'source_revision': None,
            'product_version': None,
            'page_url': None,
            'page_size_bytes': None,
            'page_sha256': None,
        },
        'checks': {
            'manifest_schema_valid': False,
            'manifest_route_contract_valid': False,
            'manifest_content_contract_valid': False,
            'source_revision_matches': False,
            'product_version_matches': False,
            'page_size_matches': False,
            'page_sha256_matches': False,
            'route_fragment_present': False,
            'content_markers_present': False,
        },
        'finding_codes': [],
        'detail': '',
    }


def _finish(
    report: dict[str, Any],
    *,
    status: str,
    findings: list[str],
    detail: str,
) -> dict[str, Any]:
    report['status'] = status
    report['finding_codes'] = findings
    report['detail'] = detail
    validate_contract(report, REPORT_SCHEMA_NAME)
    return report


def _decode_manifest(payload: bytes) -> dict[str, Any]:
    try:
        manifest = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise DocsAuditError(f'deployment manifest is not UTF-8 JSON: {exc}') from exc
    if not isinstance(manifest, dict):
        raise DocsAuditError('deployment manifest root is not an object')
    try:
        validate_contract(manifest, MANIFEST_SCHEMA_NAME)
    except (FileNotFoundError, ValueError) as exc:
        raise DocsAuditError(f'deployment manifest schema failed: {exc}') from exc
    return manifest


def _route_contract(manifest: dict[str, Any]) -> bool:
    routes = manifest['page']['routes']
    observed = {
        item['route_id']: item['fragment']
        for item in routes
    }
    return observed == ROUTES and len(routes) == len(ROUTES)


def _content_contract(manifest: dict[str, Any]) -> bool:
    return manifest_content_markers(
        (manifest.get('page') or {}).get('content_markers')
    )


def _html_ids(payload: bytes) -> set[str]:
    try:
        html = payload.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise DocsAuditError('public Getting Started page is not UTF-8') from exc
    parser = _IdCollector()
    try:
        parser.feed(html)
        parser.close()
    except ValueError as exc:
        raise DocsAuditError(
            f'public Getting Started page cannot be parsed: {exc}'
        ) from exc
    return parser.ids


def audit_deployment(
    expected_revision: str,
    expected_version: str,
    route: str,
    *,
    fetcher: Fetcher = _fetch,
) -> dict[str, Any]:
    """Fetch and verify one deployed route without writing local state."""
    report = _base_report(expected_revision, expected_version, route)
    try:
        manifest_payload = fetcher(MANIFEST_URL, MAX_MANIFEST_BYTES)
    except FetchError as exc:
        return _finish(
            report,
            status='BLOCKED',
            findings=['manifest-unavailable'],
            detail=str(exc),
        )
    try:
        manifest = _decode_manifest(manifest_payload)
    except DocsAuditError as exc:
        return _finish(
            report,
            status='NOT_READY',
            findings=['manifest-invalid'],
            detail=str(exc),
        )

    report['checks']['manifest_schema_valid'] = True
    route_contract_valid = _route_contract(manifest)
    report['checks']['manifest_route_contract_valid'] = route_contract_valid
    content_contract_valid = _content_contract(manifest)
    report['checks']['manifest_content_contract_valid'] = content_contract_valid
    page = manifest['page']
    page_url = SITE_URL + page['path']
    report['observed'].update({
        'source_revision': manifest['source_revision'],
        'product_version': manifest['product_version'],
        'page_url': page_url,
        'page_size_bytes': page['size_bytes'],
        'page_sha256': page['sha256'],
    })
    findings: list[str] = []
    if not route_contract_valid:
        findings.append('manifest-route-contract-mismatch')
    if not content_contract_valid:
        findings.append('manifest-content-contract-mismatch')
    revision_matches = manifest['source_revision'] == expected_revision
    version_matches = manifest['product_version'] == expected_version
    report['checks']['source_revision_matches'] = revision_matches
    report['checks']['product_version_matches'] = version_matches
    if not revision_matches:
        findings.append('source-revision-mismatch')
    if not version_matches:
        findings.append('product-version-mismatch')

    missing_markers: tuple[str, ...] = ()
    try:
        page_payload = fetcher(page_url, MAX_PAGE_BYTES)
    except FetchError as exc:
        findings.append('page-unavailable')
        return _finish(
            report,
            status='BLOCKED',
            findings=findings,
            detail=str(exc),
        )
    size_matches = len(page_payload) == page['size_bytes']
    digest_matches = hashlib.sha256(page_payload).hexdigest() == page['sha256']
    report['checks']['page_size_matches'] = size_matches
    report['checks']['page_sha256_matches'] = digest_matches
    if not size_matches:
        findings.append('page-size-mismatch')
    if not digest_matches:
        findings.append('page-digest-mismatch')
    try:
        ids = _html_ids(page_payload)
    except DocsAuditError as exc:
        findings.append('page-invalid')
        return _finish(
            report,
            status='NOT_READY',
            findings=findings,
            detail=str(exc),
        )
    route_present = ROUTES[route] in ids
    report['checks']['route_fragment_present'] = route_present
    if not route_present:
        findings.append('route-fragment-missing')
    try:
        missing_markers = missing_content_markers(page_payload)
    except (UnicodeDecodeError, ValueError) as exc:
        findings.append('page-invalid')
        return _finish(
            report,
            status='NOT_READY',
            findings=list(dict.fromkeys(findings)),
            detail=f'public Getting Started page text cannot be parsed: {exc}',
        )
    report['checks']['content_markers_present'] = not missing_markers
    if missing_markers:
        findings.append('content-marker-missing')

    status = 'VERIFIED' if not findings else 'NOT_READY'
    detail = (
        'public documentation bytes, routes, and first-map handoff content '
        'match the exact source revision'
        if status == 'VERIFIED'
        else 'public documentation does not match every required identity check'
    )
    if missing_markers:
        detail += '; missing content markers: ' + ', '.join(missing_markers)
    return _finish(report, status=status, findings=findings, detail=detail)


def _revision(value: str) -> str:
    normalized = value.strip().lower()
    if REVISION_PATTERN.fullmatch(normalized) is None:
        raise DocsAuditError(
            'expected revision must be exactly 40 lowercase hexadecimal characters'
        )
    return normalized


def _version(value: str) -> str:
    normalized = value.strip()
    if VERSION_PATTERN.fullmatch(normalized) is None:
        raise DocsAuditError('expected product version must be MAJOR.MINOR.PATCH')
    return normalized


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse one exact public-documentation audit request."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-revision', required=True)
    parser.add_argument('--expected-product-version', required=True)
    parser.add_argument('--route', choices=tuple(ROUTES), required=True)
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def _render(report: dict[str, Any]) -> str:
    findings = ', '.join(report['finding_codes']) or 'none'
    return '\n'.join([
        'Public documentation deployment audit',
        f"Status: {report['status']}",
        f"Manifest: {report['manifest_url']}",
        f"Expected revision: {report['expectations']['source_revision']}",
        f"Observed revision: {report['observed']['source_revision'] or 'unavailable'}",
        f'Findings: {findings}',
        f"Detail: {report['detail']}",
        'Writes performed: no',
    ])


def main(argv: Sequence[str] | None = None) -> int:
    """Run the read-only audit and preserve unmet-gate exit semantics."""
    args = parse_args(argv)
    try:
        report = audit_deployment(
            _revision(args.expected_revision),
            _version(args.expected_product_version),
            args.route,
        )
    except (DocsAuditError, FileNotFoundError, ValueError) as exc:
        print(f'public docs audit error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(_render(report))
    return 0 if report['status'] == 'VERIFIED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
