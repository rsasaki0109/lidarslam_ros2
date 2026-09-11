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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS, "AS IS"
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

"""Audit the local first-map verification package without network or writes."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Iterable

from build_release_bundle import release_bundle_paths
from docs_deployment_contract import CONTENT_MARKERS, CONTENT_MARKER_IDS
from product_schema import validate_contract


REPO_ROOT = Path(__file__).resolve().parents[1]
SCHEMA_NAME = 'first-map-verification-package-v1.schema.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/first-map-verification-package-v1.schema.json'
)

CHECK_IDS = (
    'required_files_present',
    'docker_source_documentation',
    'receipt_handoff_contract',
    'runtime_handoff_contract',
    'docs_marker_contract',
    'package_audit_documentation',
    'ci_gate_contract',
    'release_bundle_contract',
)

REQUIRED_FILES = (
    'README.md',
    'RELEASING.md',
    'VERSION',
    'Dockerfile',
    '.github/workflows/main.yml',
    '.github/workflows/candidate-image.yml',
    '.github/workflows/docker.yml',
    '.github/workflows/docs-site.yml',
    '.github/workflows/release.yml',
    'docs/contracts/cli-v1.json',
    'docs/contracts/first-map-validator-cohort-v1.json',
    'docs/evidence/external-first-map-validations.json',
    'docs/external-first-map-validation.md',
    'docs/getting-started-ja.md',
    'docs/getting-started.md',
    'docs/schemas/external-first-map-validations-v1.schema.json',
    'docs/schemas/first-map-demo-plan-v1.schema.json',
    'docs/schemas/first-map-handoff-v1.schema.json',
    'docs/schemas/first-map-validator-cohort-v1.schema.json',
    'docs/schemas/first-map-validation-receipt-v1.schema.json',
    'docs/schemas/first-map-verification-package-v1.schema.json',
    'docs/schemas/source-quickstart-plan-v1.schema.json',
    'docs/schemas/docs-deployment-manifest-v1.schema.json',
    'scripts/check_first_map_verification_package.py',
    'scripts/check_installed_product_cli.py',
    'scripts/create_first_map_validation_receipt.py',
    'scripts/docs_deployment_contract.py',
    'scripts/first_map_demo.py',
    'scripts/first_map_validation_receipt.py',
    'scripts/first_map_validator_cohort.py',
    'scripts/install_source_dependencies.sh',
    'scripts/lidarslam_cli.py',
    'scripts/run_first_map_demo.sh',
    'scripts/source_quickstart.sh',
    'scripts/support_bundle.py',
)

DOCKER_SOURCE_TEXT: dict[str, tuple[str, ...]] = {
    'docs/getting-started.md': (
        'docker run --rm --network none',
        '-v "$PWD/lidarslam_output:/output:ro"',
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
        'share/lidarslam/product/scripts/create_first_map_validation_receipt.py',
        'docker image inspect',
    ),
    'docs/getting-started-ja.md': (
        'docker run --rm --network none',
        '-v "$PWD/lidarslam_output:/output:ro"',
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
        'share/lidarslam/product/scripts/create_first_map_validation_receipt.py',
        'docker image inspect',
    ),
    'docs/external-first-map-validation.md': (
        'docker run --rm --network none',
        '-v "$PWD/lidarslam_output:/output:ro"',
        'ghcr.io/rsasaki0109/lidar_slam_ros2:v0.9.0-humble',
        'share/lidarslam/product/scripts/create_first_map_validation_receipt.py',
        'docker image inspect',
    ),
}

SOURCE_TEXT: dict[str, tuple[str, ...]] = {
    'docs/getting-started.md': (
        'lidarslam-map report ~/ros2_ws/output/mid360_demo --json',
        'python3 scripts/create_first_map_validation_receipt.py',
    ),
    'docs/getting-started-ja.md': (
        'lidarslam-map report /path/to/output/mid360_demo --json',
        'python3 scripts/create_first_map_validation_receipt.py',
    ),
    'docs/external-first-map-validation.md': (
        'lidarslam-map report /lidarslam_ws/output/mid360_demo',
        'python3 scripts/create_first_map_validation_receipt.py',
    ),
    'README.md': (
        'lidarslam-map report /path/to/output/mid360_demo --json',
        'docs/external-first-map-validation.md',
    ),
    'scripts/source_quickstart.sh': (
        'install_source_dependencies.sh',
        '--repo-only',
    ),
}

RUNTIME_GUARDS = (
    'Dockerfile',
    '.github/workflows/docker.yml',
    '.github/workflows/candidate-image.yml',
    '.github/workflows/release.yml',
)


def _regular_file(root: Path, relative: str) -> bool:
    path = root / relative
    return path.is_file() and not path.is_symlink()


def _read_text(root: Path, relative: str) -> str | None:
    path = root / relative
    if not _regular_file(root, relative):
        return None
    try:
        return path.read_text(encoding='utf-8')
    except (OSError, UnicodeDecodeError):
        return None


def _missing_text(
    root: Path,
    requirements: dict[str, tuple[str, ...]],
) -> list[str]:
    missing: list[str] = []
    for relative, needles in requirements.items():
        text = _read_text(root, relative)
        if text is None:
            missing.append(relative)
            continue
        missing.extend(
            f'{relative}:{needle}'
            for needle in needles
            if needle not in text
        )
    return missing


def _load_json(root: Path, relative: str) -> dict[str, Any] | None:
    text = _read_text(root, relative)
    if text is None:
        return None
    try:
        payload = json.loads(text)
    except json.JSONDecodeError:
        return None
    return payload if isinstance(payload, dict) else None


def _receipt_contract(root: Path) -> bool:
    contract = _load_json(root, 'docs/contracts/cli-v1.json')
    if contract is None:
        return False
    try:
        report = contract['commands']['report']
        positional = report['positional']
        accepted = positional['accepted_evidence']
        return (
            positional['kind'] == 'directory'
            and positional['name'] == 'session_bundle_or_demo_output'
            and positional['stability'] == 'stable'
            and accepted == [
                'session.json',
                'first_map_validation_receipt.json',
            ]
        )
    except (KeyError, TypeError):
        return False


def _runtime_handoff_contract(root: Path) -> bool:
    return all(
        (text := _read_text(root, relative)) is not None
        and "grep -Fq 'report <output>'" in text
        for relative in RUNTIME_GUARDS
    )


def _docs_marker_contract(root: Path) -> bool:
    text = _read_text(root, 'docs/getting-started.md')
    if text is None:
        return False
    expected = {
        'fixed-demo-output-handoff',
        'stable-image-report-boundary',
        'receipt-only-attachment-rule',
        'source-receipt-helper-fallback',
        'stable-image-receipt-helper',
        'immutable-image-identity-check',
    }
    normalized = ' '.join(text.replace('`', '').split()).lower()
    return (
        set(CONTENT_MARKER_IDS) == expected
        and all(marker in normalized for _, marker in CONTENT_MARKERS)
    )


def _package_audit_documentation(root: Path) -> bool:
    requirements = {
        'README.md': (
            'scripts/check_first_map_verification_package.py --json',
        ),
        'RELEASING.md': (
            'python3 scripts/check_first_map_verification_package.py --json',
        ),
        'docs/getting-started.md': (
            'scripts/check_first_map_verification_package.py --json',
        ),
        'docs/getting-started-ja.md': (
            'scripts/check_first_map_verification_package.py --json',
        ),
        'docs/external-first-map-validation.md': (
            'scripts/check_first_map_verification_package.py --json',
        ),
    }
    return not _missing_text(root, requirements)


def _ci_gate_contract(root: Path) -> bool:
    docs_site = _read_text(root, '.github/workflows/docs-site.yml')
    main = _read_text(root, '.github/workflows/main.yml')
    release = _read_text(root, '.github/workflows/release.yml')
    if docs_site is None or main is None or release is None:
        return False
    docs_site_contract = all(
        needle in docs_site
        for needle in (
            "'scripts/check_first_map_verification_package.py'",
            "'docs/schemas/first-map-verification-package-v1.schema.json'",
            'python3 scripts/check_first_map_verification_package.py --json',
        )
    )
    main_contract = (
        'python3 scripts/check_first_map_verification_package.py --json'
        in main
    )
    return docs_site_contract and main_contract and (
        'python3 scripts/check_first_map_verification_package.py --json'
        in release
    )


def _release_bundle_contract(root: Path) -> bool:
    version_text = _read_text(root, 'VERSION')
    if version_text is None:
        return False
    version = version_text.strip()
    try:
        paths = set(release_bundle_paths(root, f'v{version}'))
    except (OSError, ValueError):
        return False
    # The audit itself is a release-bundle deliverable.  Require the complete
    # package surface here as well, so a bundle that merely carries the audit
    # script cannot falsely advertise a usable third-party handoff.
    return set(REQUIRED_FILES).issubset(paths)


def audit_package(root: Path = REPO_ROOT) -> dict[str, Any]:
    """Return a schema-valid local readiness report without side effects."""
    checks = {check_id: True for check_id in CHECK_IDS}
    findings: list[str] = []
    detail_parts: list[str] = []

    missing_files = [
        relative
        for relative in REQUIRED_FILES
        if not _regular_file(root, relative)
    ]
    if missing_files:
        checks['required_files_present'] = False
        findings.append('required-file-contract-mismatch')
        detail_parts.append('missing required files: ' + ', '.join(missing_files))

    missing_docker = _missing_text(root, DOCKER_SOURCE_TEXT)
    if missing_docker:
        checks['docker_source_documentation'] = False
        findings.append('docker-source-documentation-mismatch')
        detail_parts.append(
            'Docker fallback documentation gaps: ' + ', '.join(missing_docker)
        )

    missing_source = _missing_text(root, SOURCE_TEXT)
    if missing_source:
        checks['docker_source_documentation'] = False
        if 'docker-source-documentation-mismatch' not in findings:
            findings.append('docker-source-documentation-mismatch')
        detail_parts.append(
            'source handoff documentation gaps: '
            + ', '.join(missing_source)
        )

    if not _receipt_contract(root):
        checks['receipt_handoff_contract'] = False
        findings.append('receipt-handoff-contract-mismatch')

    if not _runtime_handoff_contract(root):
        checks['runtime_handoff_contract'] = False
        findings.append('runtime-handoff-contract-mismatch')

    if not _docs_marker_contract(root):
        checks['docs_marker_contract'] = False
        findings.append('docs-marker-contract-mismatch')

    if not _package_audit_documentation(root):
        checks['package_audit_documentation'] = False
        findings.append('package-audit-documentation-mismatch')

    if not _ci_gate_contract(root):
        checks['ci_gate_contract'] = False
        findings.append('ci-gate-contract-mismatch')

    if not _release_bundle_contract(root):
        checks['release_bundle_contract'] = False
        findings.append('release-bundle-contract-mismatch')

    status = 'READY' if all(checks.values()) else 'NOT_READY'
    if not detail_parts:
        detail_parts.append(
            'local first-map verification package contains the documented '
            'Docker/source handoff, receipt contract, runtime guards, '
            'deployment markers, CI gate, and release inventory'
        )
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'scope': 'local-first-map-verification-package',
        'status': status,
        'checks': checks,
        'finding_codes': list(dict.fromkeys(findings)),
        'detail': '; '.join(detail_parts),
        'network_requested': False,
        'writes_performed': False,
    }
    validate_contract(report, SCHEMA_NAME)
    return report


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Audit the local first-map verification package without network '
            'access or filesystem writes.'
        )
    )
    parser.add_argument(
        '--root',
        type=Path,
        default=REPO_ROOT,
        help='source checkout root (default: repository containing this script)',
    )
    parser.add_argument('--json', action='store_true', help='emit JSON only')
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    root = args.root.expanduser().resolve()
    try:
        report = audit_package(root)
    except (FileNotFoundError, OSError, ValueError) as exc:
        print(f'first-map verification package audit failed: {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(f"first-map verification package: {report['status']}")
        print(f"checks: {', '.join(key for key, value in report['checks'].items() if value)}")
        if report['finding_codes']:
            print('findings: ' + ', '.join(report['finding_codes']))
        print(report['detail'])
    return 0 if report['status'] == 'READY' else 1


if __name__ == '__main__':
    raise SystemExit(main())
