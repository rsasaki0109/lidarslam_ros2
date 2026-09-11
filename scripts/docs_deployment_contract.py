#!/usr/bin/env python3
"""Shared content contract for the public first-map documentation page."""

from __future__ import annotations

from html import unescape
from html.parser import HTMLParser
import re


# These markers describe operator-visible behavior rather than page styling.
# They prevent an old Getting Started page from being treated as deployable
# when it cannot explain the fixed-demo receipt handoff.
CONTENT_MARKERS = (
    (
        'fixed-demo-output-handoff',
        'pass the output directory itself',
    ),
    (
        'stable-image-report-boundary',
        'published v0.9.0 image predates the lidarslam-map report handoff command',
    ),
    (
        'receipt-only-attachment-rule',
        'submit only that receipt',
    ),
    (
        'source-receipt-helper-fallback',
        'python3 scripts/create_first_map_validation_receipt.py',
    ),
    (
        'stable-image-receipt-helper',
        'share/lidarslam/product/scripts/create_first_map_validation_receipt.py',
    ),
    (
        'immutable-image-identity-check',
        'docker image inspect',
    ),
)
CONTENT_MARKER_IDS = tuple(identifier for identifier, _ in CONTENT_MARKERS)


class _TextCollector(HTMLParser):
    """Collect rendered text while ignoring navigation markup."""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.parts: list[str] = []

    def handle_data(self, data: str) -> None:
        self.parts.append(data)


def normalized_html_text(payload: bytes) -> str:
    """Return lower-case rendered text with markup and whitespace removed."""
    html = payload.decode('utf-8')
    parser = _TextCollector()
    parser.feed(html)
    parser.close()
    return re.sub(r'\s+', ' ', unescape(' '.join(parser.parts))).strip().lower()


def missing_content_markers(payload: bytes) -> tuple[str, ...]:
    """Return marker IDs absent from one rendered Getting Started page."""
    text = normalized_html_text(payload)
    return tuple(
        identifier
        for identifier, marker in CONTENT_MARKERS
        if marker not in text
    )


def manifest_content_markers(value: object) -> bool:
    """Require a manifest to advertise exactly this content contract."""
    return value == list(CONTENT_MARKER_IDS)
