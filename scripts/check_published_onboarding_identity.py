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

"""Verify that one published release still matches an onboarding packet."""

from __future__ import annotations

import argparse
import json
import re
import sys
from typing import Any, Callable, Sequence

from check_published_release import evaluate_publication, inspect_remote
from product_schema import validate_contract


REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
SITE_URL = 'https://rsasaki0109.github.io/lidar_slam_ros2/'
SCHEMA_NAME = 'published-onboarding-identity-v1.schema.json'
SCHEMA_URI = SITE_URL + 'schemas/' + SCHEMA_NAME
VERSION_PATTERN = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')
COMMIT_PATTERN = re.compile(r'^[0-9a-f]{40}$')
DIGEST_PATTERN = re.compile(r'^sha256:[0-9a-f]{64}$')
DISTROS = ('humble', 'jazzy')


class IdentityAuditError(ValueError):
    """The expected or observed release identity is unsafe."""


ReleaseAuditor = Callable[[str], dict[str, Any]]


def _live_release_audit(version: str) -> dict[str, Any]:
    snapshot = inspect_remote(version)
    return evaluate_publication(version=version, snapshot=snapshot)


def _validate_expected(
    version: str,
    source_commit: str,
    docker_digests: dict[str, str],
) -> None:
    if VERSION_PATTERN.fullmatch(version) is None:
        raise IdentityAuditError('version must be MAJOR.MINOR.PATCH')
    if COMMIT_PATTERN.fullmatch(source_commit) is None:
        raise IdentityAuditError(
            'source commit must be one lowercase 40-character SHA'
        )
    if set(docker_digests) != set(DISTROS):
        raise IdentityAuditError(
            'one expected Docker digest is required for Humble and Jazzy'
        )
    if any(
        DIGEST_PATTERN.fullmatch(digest) is None
        for digest in docker_digests.values()
    ):
        raise IdentityAuditError(
            'Docker digests must be sha256 plus 64 lowercase hexadecimal '
            'characters'
        )
    if docker_digests['humble'] == docker_digests['jazzy']:
        raise IdentityAuditError('Humble and Jazzy digests must differ')


def _observed_digests(
    release: dict[str, Any],
    version: str,
) -> tuple[dict[str, str | None], bool]:
    images = release.get('images')
    if not isinstance(images, list):
        return {distro: None for distro in DISTROS}, False
    expected_tags = {
        distro: f'ghcr.io/{REPOSITORY}:v{version}-{distro}'
        for distro in DISTROS
    }
    observed: dict[str, str | None] = {
        distro: None for distro in DISTROS
    }
    valid = len(images) == len(DISTROS)
    seen: set[str] = set()
    for image in images:
        if not isinstance(image, dict):
            valid = False
            continue
        tag = image.get('tag')
        matches = [
            distro for distro, expected in expected_tags.items()
            if tag == expected
        ]
        if len(matches) != 1 or matches[0] in seen:
            valid = False
            continue
        distro = matches[0]
        seen.add(distro)
        digest = image.get('digest')
        if (
            image.get('status') != 'PUBLISHED'
            or not isinstance(digest, str)
            or DIGEST_PATTERN.fullmatch(digest) is None
        ):
            valid = False
            continue
        observed[distro] = digest
    return observed, valid and seen == set(DISTROS)


def audit_identity(
    version: str,
    source_commit: str,
    docker_humble_digest: str,
    docker_jazzy_digest: str,
    *,
    release_auditor: ReleaseAuditor = _live_release_audit,
) -> dict[str, Any]:
    """Compare live release identity with one exact four-row packet."""
    expected_digests = {
        'humble': docker_humble_digest,
        'jazzy': docker_jazzy_digest,
    }
    _validate_expected(version, source_commit, expected_digests)
    release = release_auditor(version)
    if not isinstance(release, dict):
        raise IdentityAuditError('published release auditor returned no object')

    release_status = release.get('status')
    remote = release.get('remote')
    observed_commit = (
        remote.get('tag_commit') if isinstance(remote, dict) else None
    )
    observed_digests, images_valid = _observed_digests(release, version)
    checks = {
        'release_published': release_status == 'PUBLISHED',
        'release_image_contract_valid': images_valid,
        'source_commit_matches': observed_commit == source_commit,
        'humble_digest_matches': (
            observed_digests['humble'] == expected_digests['humble']
        ),
        'jazzy_digest_matches': (
            observed_digests['jazzy'] == expected_digests['jazzy']
        ),
    }
    findings: list[str] = []
    if release_status == 'BLOCKED':
        findings.append('release-audit-blocked')
    elif release_status != 'PUBLISHED':
        findings.append('release-not-published')
    if release_status == 'PUBLISHED' and not images_valid:
        findings.append('release-image-contract-invalid')
    if release_status == 'PUBLISHED' and observed_commit != source_commit:
        findings.append('source-commit-mismatch')
    for distro in DISTROS:
        if (
            release_status == 'PUBLISHED'
            and observed_digests[distro] != expected_digests[distro]
        ):
            findings.append(f'{distro}-digest-mismatch')

    if not findings and all(checks.values()):
        status = 'READY'
        detail = (
            'published tag commit and both live image digests match the '
            'onboarding identity'
        )
    elif release_status == 'BLOCKED':
        status = 'BLOCKED'
        detail = 'published release audit is blocked; no trial may start'
    else:
        status = 'NOT_READY'
        detail = (
            'published release is absent or differs from the onboarding '
            'identity'
        )
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': status,
        'repository': REPOSITORY,
        'network_requested': True,
        'writes_performed': False,
        'expected': {
            'product_version': version,
            'source_commit': source_commit,
            'docker_digests': expected_digests,
        },
        'observed': {
            'release_status': (
                release_status if isinstance(release_status, str) else None
            ),
            'source_commit': observed_commit,
            'docker_digests': observed_digests,
        },
        'checks': checks,
        'finding_codes': findings,
        'detail': detail,
    }
    validate_contract(report, SCHEMA_NAME)
    return report


def _render(report: dict[str, Any]) -> str:
    findings = ', '.join(report['finding_codes']) or 'none'
    return '\n'.join([
        'Published onboarding identity preflight',
        f"Status: {report['status']}",
        f"Version: {report['expected']['product_version']}",
        f"Expected commit: {report['expected']['source_commit']}",
        f"Observed commit: {report['observed']['source_commit'] or 'absent'}",
        f'Findings: {findings}',
        f"Detail: {report['detail']}",
        'Writes performed: no',
    ])


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse one exact published onboarding identity."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', required=True)
    parser.add_argument('--source-commit', required=True)
    parser.add_argument('--docker-humble-digest', required=True)
    parser.add_argument('--docker-jazzy-digest', required=True)
    parser.add_argument('--json', action='store_true')
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run a bounded network-read preflight without changing remote state."""
    args = parse_args(argv)
    try:
        report = audit_identity(
            args.version,
            args.source_commit,
            args.docker_humble_digest,
            args.docker_jazzy_digest,
        )
    except (IdentityAuditError, OSError, ValueError) as exc:
        print(f'published onboarding identity error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        print(_render(report))
    return 0 if report['status'] == 'READY' else 1


if __name__ == '__main__':
    raise SystemExit(main())
