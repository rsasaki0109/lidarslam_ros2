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

"""Prepare a path-free, exact-identity G0 observer packet.

The packet is a local plan, not onboarding evidence. It validates that the
source revision and both Docker image digests use one product version. Docker
identity is either a published release or one retained candidate-image set;
the two modes never invent or reuse each other's tags and preflights. The tool
performs no network request, trial, cleanup, or GitHub/community mutation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import sys
from typing import Any

from audit_candidate_image_set import (
    audit_candidate_bundle,
    load_candidate_evidence_bundle,
)

import jsonschema

REPOSITORY = 'rsasaki0109/lidar_slam_ros2'
DATASET_ID = 'mid360-public-zenodo-14841855'
DISTROS = ('humble', 'jazzy')
OS_FAMILY = {'humble': 'ubuntu-22.04', 'jazzy': 'ubuntu-24.04'}
VERSION_RE = re.compile(
    r'^[0-9]+\.[0-9]+\.[0-9]+(?:[-+][A-Za-z0-9.-]+)?$'
)
COMMIT_RE = re.compile(r'^[0-9a-f]{40}$')
DIGEST_RE = re.compile(r'^sha256:[0-9a-f]{64}$')
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/onboarding-matrix-observer-packet-v3.schema.json'
)
CANDIDATE_EVIDENCE_PLACEHOLDER = '<CANDIDATE_EVIDENCE_DIR>'
MAX_RELEASE_REPORT_BYTES = 2 * 1024 * 1024
REQUIRED_RELEASE_CHECKS = {
    'tag-commit',
    'release-tag',
    'release-finalized',
    'stable-release-channel',
    'release-url',
    'required-assets',
    'cross-asset-identity',
    'live-image-tag-digests',
}


class PacketError(ValueError):
    """The supplied identity inputs cannot produce a safe packet."""


def _read_release_report(source: str) -> bytes:
    if source == '-':
        payload = sys.stdin.buffer.read(MAX_RELEASE_REPORT_BYTES + 1)
        label = 'standard input'
    else:
        path = Path(source).expanduser()
        if path.is_symlink() or not path.is_file():
            raise PacketError(
                'published release report must be a regular non-symlink '
                f'file: {path}'
            )
        try:
            payload = path.read_bytes()
        except OSError as exc:
            raise PacketError(
                f'cannot read published release report {path}: {exc}'
            ) from exc
        label = str(path)
    if not payload:
        raise PacketError(f'published release report is empty: {label}')
    if len(payload) > MAX_RELEASE_REPORT_BYTES:
        raise PacketError(
            'published release report exceeds the bounded input size'
        )
    return payload


def _release_identity_from_report(
    payload: bytes,
) -> tuple[dict[str, Any], str, str, dict[str, str]]:
    try:
        report = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PacketError(
            f'published release report is not UTF-8 JSON: {exc}'
        ) from exc
    if not isinstance(report, dict):
        raise PacketError('published release report root is not an object')
    schema_path = (
        Path(__file__).resolve().parents[1]
        / 'docs'
        / 'schemas'
        / 'published-release-v1.schema.json'
    )
    try:
        schema = json.loads(schema_path.read_text(encoding='utf-8'))
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(
            schema,
            format_checker=jsonschema.FormatChecker(),
        ).validate(report)
    except (OSError, json.JSONDecodeError, jsonschema.SchemaError) as exc:
        raise PacketError(f'release report schema cannot be loaded: {exc}') from exc
    except jsonschema.ValidationError as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise PacketError(
            'published release report schema failed at '
            f'{location or "<root>"}: {exc.message}'
        ) from exc
    if report['status'] != 'PUBLISHED':
        raise PacketError(
            'published release report must have status PUBLISHED; found '
            f"{report['status']}"
        )
    version = report['expected_version']
    expected_tag = f'v{version}'
    remote = report['remote']
    source_commit = remote['tag_commit']
    expected_url = (
        f'https://github.com/{REPOSITORY}/releases/tag/{expected_tag}'
    )
    if (
        report['expected_tag'] != expected_tag
        or not isinstance(source_commit, str)
        or COMMIT_RE.fullmatch(source_commit) is None
        or remote['html_url'] != expected_url
    ):
        raise PacketError(
            'published release tag, commit, or URL identity is incomplete'
        )
    check_ids = [check['id'] for check in report['checks']]
    if len(check_ids) != len(set(check_ids)):
        raise PacketError('published release report repeats a check identity')
    missing_checks = sorted(REQUIRED_RELEASE_CHECKS - set(check_ids))
    if missing_checks:
        raise PacketError(
            'published release report lacks required PASS checks: '
            + ', '.join(missing_checks)
        )
    expected_tags = {
        distro: f'ghcr.io/{REPOSITORY}:v{version}-{distro}'
        for distro in DISTROS
    }
    digests: dict[str, str] = {}
    for image in report.get('images', []):
        matches = [
            distro for distro, tag in expected_tags.items()
            if image['tag'] == tag
        ]
        if len(matches) != 1 or matches[0] in digests:
            raise PacketError(
                'published release report contains an ambiguous image tag'
            )
        digest = image['digest']
        if (
            image['status'] != 'PUBLISHED'
            or not isinstance(digest, str)
            or DIGEST_RE.fullmatch(digest) is None
        ):
            raise PacketError(
                'published release report contains an unverified image'
            )
        digests[matches[0]] = digest
    _validate_identity(version, source_commit, digests)
    evidence = {
        'release_report_sha256': hashlib.sha256(payload).hexdigest(),
        'release_tag': expected_tag,
        'release_commit': source_commit,
        'release_url': expected_url,
    }
    return evidence, version, source_commit, digests


def _validate_identity(
    product_version: str,
    source_commit: str,
    docker_digests: dict[str, str],
) -> None:
    if not VERSION_RE.fullmatch(product_version):
        raise PacketError(
            'product_version must be a semantic version, optionally with a '
            'pre-release or build suffix'
        )
    if not COMMIT_RE.fullmatch(source_commit):
        raise PacketError(
            'source_commit must be one lowercase 40-character hexadecimal SHA'
        )
    if set(docker_digests) != set(DISTROS):
        raise PacketError('one Docker digest is required for Humble and Jazzy')
    for distro, digest in docker_digests.items():
        if not DIGEST_RE.fullmatch(digest):
            raise PacketError(
                f'{distro} Docker digest must be sha256 plus 64 lowercase '
                'hexadecimal characters'
            )
    if docker_digests['humble'] == docker_digests['jazzy']:
        raise PacketError(
            'Humble and Jazzy image digests must differ; a copied digest '
            'would make the row identity ambiguous'
        )


def _command(parts: list[str]) -> str:
    return shlex.join(parts)


def _required_measurements() -> list[str]:
    return [
        'input.download_bytes',
        'measurements.workflow_download_bytes',
        'measurements.wall_time_sec',
        'measurements.active_operator_time_sec',
        'measurements.command_count',
        'measurements.peak_disk_bytes',
        'measurements.output_bytes',
    ]


def _docker_row(
    product_version: str,
    distro: str,
    digest: str,
    *,
    candidate_set: dict[str, Any] | None = None,
    candidate_set_sha256: str | None = None,
    candidate_bundle_sha256: str | None = None,
    release_preflight_command: str | None = None,
) -> dict[str, Any]:
    image_tag: str | None
    immutable_ref: str
    if candidate_set is None:
        if release_preflight_command is None:
            raise PacketError(
                'release rows require an exact live identity preflight'
            )
        image_tag = f'ghcr.io/{REPOSITORY}:v{product_version}-{distro}'
        immutable_ref = f'{image_tag}@{digest}'
        preflight_command = release_preflight_command
        image_arguments = ['--image-tag', image_tag]
        candidate_arguments: list[str] = []
    else:
        if candidate_set_sha256 is None or candidate_bundle_sha256 is None:
            raise PacketError(
                'candidate packet requires exact set and bundle hashes'
            )
        image_tag = None
        by_distro = {
            image['ros_distro']: image
            for image in candidate_set['images']
        }
        immutable_ref = by_distro[distro]['immutable_ref']
        preflight_command = _command([
            'python3',
            'scripts/audit_candidate_image_set.py',
            '--candidate-evidence-dir',
            CANDIDATE_EVIDENCE_PLACEHOLDER,
            '--remote',
            '--json',
        ])
        image_arguments = ['--candidate-image-ref', immutable_ref]
        candidate_arguments = [
            '--candidate-image-set-sha256',
            candidate_set_sha256,
            '--candidate-evidence-bundle-sha256',
            candidate_bundle_sha256,
            '--candidate-source-pr',
            str(candidate_set['source_pr']),
            '--candidate-source-commit',
            candidate_set['source_commit'],
            '--candidate-workflow-run-url',
            candidate_set['workflow_run_url'],
        ]
    return {
        'row_id': f'docker-{distro}',
        'route': 'docker',
        'ros_distro': distro,
        'os_family': OS_FAMILY[distro],
        'product_version': product_version,
        'identity': {
            'kind': 'image-digest',
            'value': digest,
            'tag': image_tag,
            'immutable_ref': immutable_ref,
        },
        'preflight_command': preflight_command,
        'observer_command': _command([
            'python3',
            'scripts/run_docker_onboarding_probe.py',
            '--trial-id',
            f'g0-docker-{distro}-<UTC_DATE>-a',
            '--ros-distro',
            distro,
            *image_arguments,
            '--image-digest',
            digest,
            '--product-version',
            product_version,
            *candidate_arguments,
            '--record',
            '<TRIAL_RECORD_OUTSIDE_CHECKOUT>',
            '--validation-receipt-output',
            '<VALIDATION_RECEIPT_OUTSIDE_CHECKOUT>',
            '--disk-scope',
            '/',
            '--acknowledge-dedicated-filesystem',
            '--prompt-human-measurements',
            '--allow-privileged-container-host',
        ]),
        'required_measurements': _required_measurements(),
    }


def _source_row(
    product_version: str,
    distro: str,
    commit: str,
) -> dict[str, Any]:
    return {
        'row_id': f'source-{distro}',
        'route': 'source',
        'ros_distro': distro,
        'os_family': OS_FAMILY[distro],
        'product_version': product_version,
        'identity': {
            'kind': 'git-commit',
            'value': commit,
            'tag': None,
            'immutable_ref': None,
        },
        'preflight_command': _command([
            'python3',
            'scripts/run_source_onboarding_probe.py',
            '--public-preflight',
            '--source-commit',
            commit,
            '--product-version',
            product_version,
        ]),
        'observer_command': _command([
            'python3',
            'scripts/run_source_onboarding_probe.py',
            '--trial-id',
            f'g0-source-{distro}-<UTC_DATE>-a',
            '--ros-distro',
            distro,
            '--source-commit',
            commit,
            '--product-version',
            product_version,
            '--trial-root',
            '<TRIAL_ROOT_OUTSIDE_CHECKOUT>',
            '--observer-parent',
            '<OBSERVER_PARENT_OUTSIDE_CHECKOUT>',
            '--disk-scope',
            '/',
            '--record',
            '<TRIAL_RECORD_OUTSIDE_CHECKOUT>',
            '--validation-receipt-output',
            '<VALIDATION_RECEIPT_OUTSIDE_CHECKOUT>',
            '--prompt-human-measurements',
            '--acknowledge-disposable-host',
            '--acknowledge-isolated-network',
        ]),
        'required_measurements': _required_measurements(),
    }


def _validate_packet(packet: dict[str, Any]) -> None:
    schema_path = (
        Path(__file__).resolve().parents[1]
        / 'docs'
        / 'schemas'
        / 'onboarding-matrix-observer-packet-v3.schema.json'
    )
    try:
        schema = json.loads(schema_path.read_text(encoding='utf-8'))
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(
            schema,
            format_checker=jsonschema.FormatChecker(),
        ).validate(packet)
    except (OSError, json.JSONDecodeError, jsonschema.SchemaError) as exc:
        raise PacketError(f'packet schema cannot be loaded: {exc}') from exc
    except jsonschema.ValidationError as exc:
        location = '.'.join(str(item) for item in exc.absolute_path)
        raise PacketError(
            f'packet schema failed at {location or "<root>"}: {exc.message}'
        ) from exc


def _build_packet(
    product_version: str,
    source_commit: str,
    docker_humble_digest: str,
    docker_jazzy_digest: str,
    *,
    candidate_set: dict[str, Any] | None = None,
    candidate_set_sha256: str | None = None,
    candidate_bundle_sha256: str | None = None,
    release_evidence: dict[str, str] | None = None,
) -> dict[str, Any]:
    """Build one schema-validated release or candidate observer packet."""
    docker_digests = {
        'humble': docker_humble_digest,
        'jazzy': docker_jazzy_digest,
    }
    _validate_identity(product_version, source_commit, docker_digests)
    if candidate_set is None:
        if release_evidence is None:
            raise PacketError(
                'release packet identity must come from one published '
                'release report'
            )
        release_preflight_command = _command([
            'python3',
            'scripts/check_published_onboarding_identity.py',
            '--version',
            product_version,
            '--source-commit',
            source_commit,
            '--docker-humble-digest',
            docker_digests['humble'],
            '--docker-jazzy-digest',
            docker_digests['jazzy'],
            '--json',
        ])
    else:
        if release_evidence is not None:
            raise PacketError(
                'candidate packet cannot contain published release evidence'
            )
        release_preflight_command = None
    rows = [
        _docker_row(
            product_version,
            distro,
            docker_digests[distro],
            candidate_set=candidate_set,
            candidate_set_sha256=candidate_set_sha256,
            candidate_bundle_sha256=candidate_bundle_sha256,
            release_preflight_command=release_preflight_command,
        )
        for distro in DISTROS
    ]
    rows.extend(
        _source_row(product_version, distro, source_commit)
        for distro in DISTROS
    )
    if candidate_set is None:
        if release_evidence is None:
            raise AssertionError('release evidence was validated above')
        docker_mode = 'release'
        packet_suffix = 'release'
        docker_command = release_preflight_command
        docker_required_status = 'READY'
        docker_evidence = {
            'mode': docker_mode,
            **release_evidence,
            'candidate_set_sha256': None,
            'candidate_bundle_sha256': None,
            'source_pr': None,
            'source_commit': None,
            'workflow_run_url': None,
            'requested_by': None,
            'registry_retention_status': 'NOT_APPLICABLE',
            'evidence_retention_days': None,
        }
        identity_action = (
            'Rerun the exact read-only published onboarding identity check '
            'and require READY before provisioning a trial host.'
        )
    else:
        if candidate_set_sha256 is None or candidate_bundle_sha256 is None:
            raise PacketError(
                'candidate packet requires exact set and bundle hashes'
            )
        docker_mode = 'candidate-image-set'
        run_id = candidate_set['workflow_run_url'].rsplit('/', 1)[1]
        packet_suffix = f'candidate-{run_id}'
        docker_command = _command([
            'python3',
            'scripts/audit_candidate_image_set.py',
            '--candidate-evidence-dir',
            CANDIDATE_EVIDENCE_PLACEHOLDER,
            '--remote',
            '--json',
        ])
        docker_required_status = 'REMOTE_AUDIT_PASS'
        docker_evidence = {
            'mode': docker_mode,
            'release_report_sha256': None,
            'release_tag': None,
            'release_commit': None,
            'release_url': None,
            'candidate_set_sha256': candidate_set_sha256,
            'candidate_bundle_sha256': candidate_bundle_sha256,
            'source_pr': candidate_set['source_pr'],
            'source_commit': candidate_set['source_commit'],
            'workflow_run_url': candidate_set['workflow_run_url'],
            'requested_by': candidate_set['requested_by'],
            'registry_retention_status': (
                candidate_set['registry_retention_status']
            ),
            'evidence_retention_days': (
                candidate_set['evidence_retention_days']
            ),
        }
        identity_action = (
            'Retain the exact four-file candidate evidence directory, run '
            'the read-only byte-comparing remote audit, and require '
            'REMOTE_AUDIT_PASS before provisioning a trial host.'
        )
    source_command = _command([
        'python3',
        'scripts/run_source_onboarding_probe.py',
        '--public-preflight',
        '--source-commit',
        source_commit,
        '--product-version',
        product_version,
    ])
    packet = {
        'schema_version': 3,
        'schema_uri': SCHEMA_URI,
        'packet_id': f'g0-onboarding-{product_version}-{packet_suffix}',
        'repository': REPOSITORY,
        'product_version': product_version,
        'dataset_id': DATASET_ID,
        'status': 'READY_FOR_READ_ONLY_PREFLIGHT',
        'docker_identity_mode': docker_mode,
        'docker_evidence': docker_evidence,
        'public_checks': {
            'docker': {
                'command': docker_command,
                'required_status': docker_required_status,
                'read_only': True,
            },
            'source': {
                'command': source_command,
                'required_status': 'READY',
                'read_only': True,
            },
        },
        'rows': rows,
        'next_actions': [
            identity_action,
            'Run the source read-only public check and require READY before '
            'provisioning a source-row trial host.',
            'Use one disposable host and one independent observer per row; '
            'do not use a moving tag or a shared filesystem.',
            'Record all seven required measurements, including human active '
            'time and submitted command count; blank means non-comparable.',
            'Replace a checked-in matrix row only with the immutable record, '
            'its exact retained first-map validation receipt, and any '
            'exclusive measurement supplement bound to the record bytes.',
            'This packet is not evidence and authorizes no release, image, '
            'issue, community, or telemetry write.',
        ],
        'authority': {
            'network_reads_performed': False,
            'trial_executed': False,
            'github_writes_authorized': False,
            'community_posts_authorized': False,
            'remote_mutations_performed': False,
        },
    }
    _validate_packet(packet)
    return packet


def build_release_packet_from_report(payload: bytes) -> dict[str, Any]:
    """Derive one release packet from exact audited report bytes."""
    evidence, version, source_commit, digests = _release_identity_from_report(
        payload
    )
    return _build_packet(
        version,
        source_commit,
        digests['humble'],
        digests['jazzy'],
        release_evidence=evidence,
    )


def build_candidate_packet(
    evidence_bundle: dict[str, Any],
) -> dict[str, Any]:
    """Build a tag-free packet derived from one four-file evidence bundle."""
    try:
        audit_candidate_bundle(evidence_bundle)
    except ValueError as exc:
        raise PacketError(
            f'candidate image evidence bundle is invalid: {exc}'
        ) from exc
    candidate_set = evidence_bundle['candidate_set']
    candidate_set_sha256 = evidence_bundle['file_hashes'][
        'candidate-image-set.json'
    ]
    by_distro = {
        image['ros_distro']: image for image in candidate_set['images']
    }
    return _build_packet(
        candidate_set['product_version'],
        candidate_set['source_commit'],
        by_distro['humble']['digest'],
        by_distro['jazzy']['digest'],
        candidate_set=candidate_set,
        candidate_set_sha256=candidate_set_sha256,
        candidate_bundle_sha256=evidence_bundle['bundle_sha256'],
    )


def render_packet(packet: dict[str, Any]) -> str:
    """Render a concise operator card without private paths or evidence."""
    lines = [
        '# G0 onboarding observer packet',
        '',
        f"- Status: **{packet['status']}**",
        f"- Product version: `{packet['product_version']}`",
        f"- Dataset: `{packet['dataset_id']}`",
        f"- Docker identity mode: `{packet['docker_identity_mode']}`",
        '- This is a plan only; it is not a trial record or a release claim.',
        '',
    ]
    if packet['docker_evidence']['candidate_set_sha256'] is not None:
        lines.extend([
            '- Candidate bundle SHA-256: '
            f"`{packet['docker_evidence']['candidate_bundle_sha256']}`",
            '- Candidate set SHA-256: '
            f"`{packet['docker_evidence']['candidate_set_sha256']}`",
            '- Candidate workflow: '
            f"`{packet['docker_evidence']['workflow_run_url']}`",
            '',
        ])
    else:
        lines.extend([
            '- Published release tag: '
            f"`{packet['docker_evidence']['release_tag']}`",
            '- Published release commit: '
            f"`{packet['docker_evidence']['release_commit']}`",
            '- Published release URL: '
            f"`{packet['docker_evidence']['release_url']}`",
            '- Published release audit SHA-256: '
            f"`{packet['docker_evidence']['release_report_sha256']}`",
            '',
        ])
    lines.extend(['## Public checks', ''])
    for name, check in packet['public_checks'].items():
        lines.extend([
            f'### {name}',
            f"- Required status: `{check['required_status']}`",
            f"- Read-only command: `{check['command']}`",
            '',
        ])
    lines.extend([
        '## Fixed rows',
        '',
        '| Row | Identity | Required human observations |',
        '| --- | --- | --- |',
    ])
    for row in packet['rows']:
        identity = row['identity']['immutable_ref'] or row['identity']['value']
        lines.append(
            f"| `{row['row_id']}` | `{identity}` | "
            '`active_operator_time_sec`, `command_count` |'
        )
    lines.extend(['', '## Observer commands', ''])
    for row in packet['rows']:
        lines.extend([
            f"### `{row['row_id']}`",
            '```bash',
            row['observer_command'],
            '```',
            '',
        ])
    lines.extend(['## Required next actions', ''])
    lines.extend(f'- {action}' for action in packet['next_actions'])
    return '\n'.join(lines) + '\n'


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    identity = parser.add_mutually_exclusive_group()
    identity.add_argument(
        '--published-release-report',
        help=(
            'derive the release commit and both live image digests from one '
            'PUBLISHED check_published_release.py JSON report; use - for '
            'bounded standard input'
        ),
    )
    identity.add_argument(
        '--candidate-evidence-dir',
        type=Path,
        help=(
            'derive all Docker and source identities from one directory '
            'containing the four retained candidate artifacts; cannot be '
            'mixed with release inputs'
        ),
    )
    parser.add_argument('--product-version', help=argparse.SUPPRESS)
    parser.add_argument('--source-commit', help=argparse.SUPPRESS)
    parser.add_argument('--docker-humble-digest', help=argparse.SUPPRESS)
    parser.add_argument('--docker-jazzy-digest', help=argparse.SUPPRESS)
    output = parser.add_mutually_exclusive_group()
    output.add_argument('--json', action='store_true')
    output.add_argument('--render', action='store_true')
    parser.add_argument(
        '--output',
        type=Path,
        help=(
            'write the selected JSON or Markdown packet once; refuse an '
            'existing path'
        ),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Build and optionally write one local-only observer packet."""
    args = _parse_args(argv)
    try:
        release_inputs = {
            '--product-version': args.product_version,
            '--source-commit': args.source_commit,
            '--docker-humble-digest': args.docker_humble_digest,
            '--docker-jazzy-digest': args.docker_jazzy_digest,
        }
        supplied_release_inputs = [
            name for name, value in release_inputs.items()
            if value is not None
        ]
        if supplied_release_inputs:
            raise PacketError(
                'manual release identity inputs are not accepted; pipe or '
                'provide one --published-release-report so commit and '
                'digests are derived together'
            )
        if args.candidate_evidence_dir is not None:
            evidence_bundle = load_candidate_evidence_bundle(
                args.candidate_evidence_dir
            )
            packet = build_candidate_packet(evidence_bundle)
        else:
            if args.published_release_report is None:
                raise PacketError(
                    'choose --published-release-report or '
                    '--candidate-evidence-dir'
                )
            payload = _read_release_report(args.published_release_report)
            packet = build_release_packet_from_report(
                payload,
            )
    except (OSError, ValueError) as exc:
        print(f'onboarding observer packet error: {exc}', file=sys.stderr)
        return 2
    if args.json:
        payload = json.dumps(packet, indent=2, sort_keys=True) + '\n'
    else:
        payload = render_packet(packet)
    try:
        if args.output is None:
            sys.stdout.write(payload)
        else:
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(payload)
            print(
                f'Wrote local-only observer packet: {args.output}',
                file=sys.stderr,
            )
            print(
                'The packet is a plan, not trial evidence or publication '
                'authority.',
                file=sys.stderr,
            )
    except OSError as exc:
        print(f'onboarding observer packet error: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
