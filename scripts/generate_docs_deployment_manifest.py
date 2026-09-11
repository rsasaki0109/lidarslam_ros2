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

"""Bind a rendered GitHub Pages entrypoint to one exact source revision."""

from __future__ import annotations

import argparse
import hashlib
from html.parser import HTMLParser
import json
from pathlib import Path
import re
import sys
from typing import Sequence

from docs_deployment_contract import (
    CONTENT_MARKER_IDS,
    missing_content_markers,
)
from product_schema import validate_contract


REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
SITE_URL = 'https://rsasaki0109.github.io/lidar_slam_ros2/'
SCHEMA_URI = SITE_URL + 'schemas/docs-deployment-manifest-v1.schema.json'
SCHEMA_NAME = 'docs-deployment-manifest-v1.schema.json'
MANIFEST_NAME = 'docs-deployment-v1.json'
PAGE_NAME = 'getting-started.html'
SOURCE_PATH = 'docs/getting-started.md'
MAX_PAGE_BYTES = 16 * 1024 * 1024
REVISION_PATTERN = re.compile(r'^[0-9a-f]{40}$')
VERSION_PATTERN = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')
ROUTES = (
    ('docker-first-map', 'docker-first-map-no-ros-2-workspace'),
    ('source-quickstart', '1-install-and-build-from-source'),
)


class ManifestError(ValueError):
    """The rendered site cannot provide trustworthy deployment identity."""


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


def _regular_file(path: Path, label: str) -> bytes:
    if path.is_symlink() or not path.is_file():
        raise ManifestError(f'{label} must be a regular non-symlink file: {path}')
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise ManifestError(f'cannot read {label} {path}: {exc}') from exc
    if not payload:
        raise ManifestError(f'{label} is empty: {path}')
    return payload


def _source_revision(value: str) -> str:
    revision = value.strip().lower()
    if REVISION_PATTERN.fullmatch(revision) is None:
        raise ManifestError(
            'source revision must be exactly 40 lowercase hexadecimal characters'
        )
    return revision


def _product_version(path: Path) -> str:
    payload = _regular_file(path, 'VERSION')
    try:
        version = payload.decode('utf-8').strip()
    except UnicodeDecodeError as exc:
        raise ManifestError(f'VERSION is not UTF-8: {path}') from exc
    if VERSION_PATTERN.fullmatch(version) is None:
        raise ManifestError(f'VERSION is not a stable semantic version: {version!r}')
    return version


def _page_ids(payload: bytes, path: Path) -> set[str]:
    try:
        html = payload.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise ManifestError(f'rendered page is not UTF-8: {path}') from exc
    parser = _IdCollector()
    try:
        parser.feed(html)
        parser.close()
    except ValueError as exc:
        raise ManifestError(f'rendered page HTML cannot be parsed: {path}: {exc}') from exc
    return parser.ids


def build_manifest(
    site_dir: Path,
    source_revision: str,
    version_path: Path,
) -> dict[str, object]:
    """Build deterministic identity for the deployed beginner entrypoint."""
    if site_dir.is_symlink() or not site_dir.is_dir():
        raise ManifestError(
            f'site directory must be a regular non-symlink directory: {site_dir}'
        )
    revision = _source_revision(source_revision)
    page_path = site_dir / PAGE_NAME
    page_payload = _regular_file(page_path, 'rendered Getting Started page')
    if len(page_payload) > MAX_PAGE_BYTES:
        raise ManifestError(
            f'rendered Getting Started page exceeds {MAX_PAGE_BYTES} bytes'
        )
    page_ids = _page_ids(page_payload, page_path)
    missing = [fragment for _, fragment in ROUTES if fragment not in page_ids]
    if missing:
        raise ManifestError(
            'rendered Getting Started page lacks canonical route fragment(s): '
            + ', '.join(missing)
        )
    missing_markers = missing_content_markers(page_payload)
    if missing_markers:
        raise ManifestError(
            'rendered Getting Started page lacks first-map content marker(s): '
            + ', '.join(missing_markers)
        )
    return {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'repository': REPOSITORY,
        'source_revision': revision,
        'product_version': _product_version(version_path),
        'site_url': SITE_URL,
        'workflow': {
            'name': 'docs-site',
            'path': '.github/workflows/docs-site.yml',
            'deployment_ref': 'refs/heads/develop',
        },
        'page': {
            'path': PAGE_NAME,
            'source_path': SOURCE_PATH,
            'size_bytes': len(page_payload),
            'sha256': hashlib.sha256(page_payload).hexdigest(),
            'routes': [
                {'route_id': route_id, 'fragment': fragment}
                for route_id, fragment in ROUTES
            ],
            'content_markers': list(CONTENT_MARKER_IDS),
        },
    }


def write_manifest(site_dir: Path, manifest: dict[str, object]) -> Path:
    """Write once into the fresh Pages artifact; never replace evidence."""
    try:
        validate_contract(manifest, SCHEMA_NAME)
    except (FileNotFoundError, ValueError) as exc:
        raise ManifestError(
            f'deployment manifest schema failed: {exc}'
        ) from exc
    output = site_dir / MANIFEST_NAME
    if output.exists() or output.is_symlink():
        raise ManifestError(f'refusing to overwrite deployment manifest: {output}')
    rendered = json.dumps(manifest, indent=2, sort_keys=True) + '\n'
    try:
        with output.open('x', encoding='utf-8') as stream:
            stream.write(rendered)
    except OSError as exc:
        raise ManifestError(f'cannot write deployment manifest {output}: {exc}') from exc
    return output


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the docs workflow interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--site-dir', type=Path, required=True)
    parser.add_argument('--source-revision', required=True)
    parser.add_argument('--version-file', type=Path, required=True)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Generate one immutable manifest or reject the rendered artifact."""
    args = parse_args(argv)
    try:
        manifest = build_manifest(
            args.site_dir.expanduser(),
            args.source_revision,
            args.version_file.expanduser(),
        )
        output = write_manifest(args.site_dir.expanduser(), manifest)
    except (ManifestError, OSError) as exc:
        print(f'docs deployment manifest error: {exc}', file=sys.stderr)
        return 2
    print(output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
