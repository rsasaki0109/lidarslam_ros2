#!/usr/bin/env python3
"""Audit required lidarslam_ros2 dependency versions in public ROS apt."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'ros-apt-dependency-readiness-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/'
    'ros-apt-dependency-readiness-v1.schema.json'
)
DISTROS = ('humble', 'jazzy')
CHANNELS = ('main', 'testing')
DEPENDENCY_MINIMUMS = {
    'ndt-omp-ros2': '0.1.0',
    'rko-lio': '0.3.2',
}
DEBIAN_UPSTREAM = re.compile(
    r'^(?:[0-9]+:)?(?P<version>[0-9]+\.[0-9]+\.[0-9]+)(?:[-+~].*)?$'
)


class AptDependencyError(ValueError):
    """Public apt dependency state cannot be inspected or trusted."""


def _version_tuple(version: str) -> tuple[int, int, int]:
    try:
        parts = tuple(int(part) for part in version.split('.'))
    except ValueError as exc:
        raise AptDependencyError(
            f'invalid semantic version {version!r}') from exc
    if len(parts) != 3:
        raise AptDependencyError(f'invalid semantic version {version!r}')
    return parts  # type: ignore[return-value]


def _upstream_version(debian_version: str) -> str | None:
    match = DEBIAN_UPSTREAM.fullmatch(debian_version)
    return match.group('version') if match else None


def _probe_command(distro: str) -> list[str]:
    packages = ' '.join(
        f'ros-{distro}-{name}' for name in sorted(DEPENDENCY_MINIMUMS)
    )
    script = f"""
set -euo pipefail
apt-get update -qq
for package in {packages}; do
  apt-cache madison "$package" \
    | awk -v package="$package" '
      $0 ~ /packages\\.ros\\.org\\/ros2\\/ubuntu/ {{
        print "main\\t" package "\\t" $3
      }}'
done
apt-get install -yqq ros2-testing-apt-source >/dev/null
apt-get update -qq
for package in {packages}; do
  apt-cache madison "$package" \
    | awk -v package="$package" '
      $0 ~ /packages\\.ros\\.org\\/ros2-testing\\/ubuntu/ {{
        print "testing\\t" package "\\t" $3
      }}'
done
"""
    return [
        'docker',
        'run',
        '--rm',
        f'ros:{distro}-ros-core',
        'bash',
        '-lc',
        script,
    ]


def _probe_distro(distro: str) -> dict[str, dict[str, list[str]]]:
    result = subprocess.run(
        _probe_command(distro),
        check=False,
        capture_output=True,
        text=True,
        timeout=180,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise AptDependencyError(
            f'{distro} apt probe failed: {detail[-1000:]}')
    channels = {
        channel: {
            package: []
            for package in sorted(DEPENDENCY_MINIMUMS)
        }
        for channel in CHANNELS
    }
    for line in result.stdout.splitlines():
        fields = line.strip().split('\t')
        if len(fields) != 3:
            continue
        channel, debian_name, version = fields
        prefix = f'ros-{distro}-'
        if channel not in channels or not debian_name.startswith(prefix):
            continue
        package = debian_name[len(prefix):]
        if package in channels[channel]:
            channels[channel][package].append(version)
    for channel in CHANNELS:
        for package in channels[channel]:
            channels[channel][package] = sorted(
                set(channels[channel][package])
            )
    return channels


def inspect_live() -> dict[str, Any]:
    """Collect public apt versions in disposable Humble/Jazzy containers."""
    distros: dict[str, Any] = {}
    errors: list[str] = []
    try:
        subprocess.run(
            ['docker', 'version', '--format', '{{.Server.Version}}'],
            check=True,
            capture_output=True,
            text=True,
            timeout=15,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return {
            'inspected': False,
            'errors': [f'Docker is unavailable: {exc}'],
            'distros': {},
        }
    with ThreadPoolExecutor(max_workers=len(DISTROS)) as executor:
        futures = {
            executor.submit(_probe_distro, distro): distro
            for distro in DISTROS
        }
        for future in as_completed(futures):
            distro = futures[future]
            try:
                distros[distro] = future.result()
            except (
                OSError,
                subprocess.SubprocessError,
                AptDependencyError,
            ) as exc:
                errors.append(str(exc))
    return {
        'inspected': not errors and set(distros) == set(DISTROS),
        'errors': sorted(errors),
        'distros': distros,
    }


def _check(
    check_id: str,
    passed: bool,
    detail: str,
) -> dict[str, str]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'detail': detail,
    }


def evaluate_readiness(snapshot: dict[str, Any]) -> dict[str, Any]:
    """Evaluate an injected or live apt snapshot as a versioned contract."""
    errors = list(snapshot.get('errors', []))
    distros = snapshot.get('distros', {})
    checks: list[dict[str, str]] = []
    channel_ready = {channel: True for channel in CHANNELS}
    observed: dict[str, Any] = {}

    for distro in DISTROS:
        observed[distro] = {}
        distro_state = (
            distros.get(distro, {}) if isinstance(distros, dict) else {}
        )
        for channel in CHANNELS:
            observed[distro][channel] = {}
            channel_state = (
                distro_state.get(channel, {})
                if isinstance(distro_state, dict)
                else {}
            )
            for package, minimum in sorted(DEPENDENCY_MINIMUMS.items()):
                versions = (
                    channel_state.get(package, [])
                    if isinstance(channel_state, dict)
                    else []
                )
                if not isinstance(versions, list):
                    versions = []
                valid_versions = [
                    value for value in versions
                    if isinstance(value, str)
                ]
                upstream = [
                    parsed
                    for parsed in (
                        _upstream_version(value) for value in valid_versions
                    )
                    if parsed is not None
                ]
                passed = any(
                    _version_tuple(version) >= _version_tuple(minimum)
                    for version in upstream
                )
                channel_ready[channel] &= passed
                observed[distro][channel][package] = {
                    'minimum_version': minimum,
                    'debian_versions': valid_versions,
                    'ready': passed,
                }
                checks.append(_check(
                    f'{distro}-{channel}-{package}',
                    passed,
                    (
                        f'minimum={minimum}; '
                        f"observed={valid_versions or ['not-published']}"
                    ),
                ))

    if errors or snapshot.get('inspected') is not True:
        status = 'BLOCKED'
        actions = [
            'Restore Docker and public ROS apt access, then rerun the audit.'
        ]
    elif channel_ready['main']:
        status = 'MAIN_READY'
        actions = []
    elif channel_ready['testing']:
        status = 'TESTING_READY'
        actions = [
            (
                'Run testing-channel package-manager E2E for the '
                'release candidate.'
            ),
            'Wait for the normal ROS repository sync, then rerun this audit.',
        ]
    else:
        status = 'IN_PROGRESS'
        missing = [
            check['id']
            for check in checks
            if check['status'] == 'FAIL' and '-testing-' in check['id']
        ]
        actions = [
            'Wait for required testing-channel packages: ' + ', '.join(missing)
        ]

    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': status,
        'dependency_minimums': DEPENDENCY_MINIMUMS,
        'remote': {
            'inspected': snapshot.get('inspected') is True,
            'errors': errors,
        },
        'channels': {
            channel: {'ready': channel_ready[channel]}
            for channel in CHANNELS
        },
        'distros': observed,
        'checks': checks,
        'actions': actions,
    }
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.Draft7Validator(schema).validate(report)
    return report


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--json', action='store_true')
    parser.add_argument('--output-json', type=Path)
    parser.add_argument(
        '--require',
        choices=('none', 'testing', 'main'),
        default='none',
        help='Exit 1 unless the requested apt channel is ready.',
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Run the live audit with fail-closed inspection and stable exit codes."""
    args = _parse_args(argv)
    try:
        report = evaluate_readiness(inspect_live())
    except (
        OSError,
        AptDependencyError,
        json.JSONDecodeError,
        jsonschema.SchemaError,
        jsonschema.ValidationError,
    ) as exc:
        print(f'ROS apt dependency readiness error: {exc}', file=sys.stderr)
        return 2
    rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
    if args.output_json:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(rendered, encoding='utf-8')
    if args.json:
        print(rendered, end='')
    else:
        print(f"ROS apt dependency gate: {report['status']}")
        for action in report['actions']:
            print(f'  - {action}')
    required_status = {
        'none': None,
        'testing': {'TESTING_READY', 'MAIN_READY'},
        'main': {'MAIN_READY'},
    }[args.require]
    if required_status is not None and report['status'] not in required_status:
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
