#!/usr/bin/env python3
"""Verify a ROS apt installation or upgrade without changing packages."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import sys
from typing import Any

import jsonschema


REPO_ROOT = Path(__file__).resolve().parents[1]
SCHEMA_PATH = (
    REPO_ROOT
    / 'docs'
    / 'schemas'
    / 'package-manager-install-v1.schema.json'
)
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/package-manager-install-v1.schema.json'
)
PRODUCT_PACKAGES = (
    'lidarslam',
    'lidarslam-msgs',
    'scanmatcher',
    'graph-based-slam',
)
DEPENDENCY_MINIMUMS = {
    'ndt-omp-ros2': '0.1.0',
    'rko-lio': '0.3.2',
}
SEMVER = re.compile(r'^[0-9]+\.[0-9]+\.[0-9]+$')
DEBIAN_UPSTREAM = re.compile(
    r'^(?:[0-9]+:)?(?P<version>[0-9]+\.[0-9]+\.[0-9]+)(?:[-+~].*)?$'
)


class InstallCheckError(ValueError):
    """Installed package state or evidence input is invalid."""


def _run(command: list[str]) -> str:
    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise InstallCheckError(
            f"{' '.join(command)} failed: "
            f'{result.stderr.strip() or result.stdout.strip()}'
        )
    return result.stdout


def _deb_name(ros_distro: str, package: str) -> str:
    return f'ros-{ros_distro}-{package}'


def _source_state() -> dict[str, Any]:
    commit = _run([
        'git',
        '-c',
        f'safe.directory={REPO_ROOT}',
        '-C',
        str(REPO_ROOT),
        'rev-parse',
        'HEAD',
    ]).strip()
    dirty = bool(_run([
        'git',
        '-c',
        f'safe.directory={REPO_ROOT}',
        '-C',
        str(REPO_ROOT),
        'status',
        '--porcelain',
    ]).strip())
    if re.fullmatch(r'[a-f0-9]{40}', commit) is None:
        raise InstallCheckError(f'invalid contract git commit: {commit!r}')
    return {'git_commit': commit, 'git_dirty': dirty}


def _upstream_version(debian_version: str) -> str:
    match = DEBIAN_UPSTREAM.fullmatch(debian_version)
    if match is None:
        raise InstallCheckError(
            f'cannot extract semantic upstream version from '
            f'{debian_version!r}'
        )
    return match.group('version')


def _version_tuple(version: str) -> tuple[int, int, int]:
    if SEMVER.fullmatch(version) is None:
        raise InstallCheckError(
            f'expected MAJOR.MINOR.PATCH version, found {version!r}')
    return tuple(int(part) for part in version.split('.'))  # type: ignore[return-value]


def collect_package_state(
    ros_distro: str,
    *,
    package_names: tuple[str, ...] | None = None,
) -> dict[str, dict[str, Any]]:
    """Collect dpkg versions and owned paths for the product package set."""
    logical_names = package_names or (
        *PRODUCT_PACKAGES,
        *DEPENDENCY_MINIMUMS,
    )
    state: dict[str, dict[str, Any]] = {}
    for logical_name in logical_names:
        debian_name = _deb_name(ros_distro, logical_name)
        query = _run([
            'dpkg-query',
            '-W',
            '-f=${db:Status-Abbrev}\\t${Version}\\n',
            debian_name,
        ]).strip()
        try:
            status, version = query.split('\t', 1)
        except ValueError as exc:
            raise InstallCheckError(
                f'unexpected dpkg-query result for {debian_name}: {query!r}'
            ) from exc
        if status != 'ii ':
            raise InstallCheckError(
                f'{debian_name} is not fully installed: {status!r}')
        paths = sorted({
            line.strip()
            for line in _run(['dpkg-query', '-L', debian_name]).splitlines()
            if line.strip().startswith('/')
        })
        if not paths:
            raise InstallCheckError(
                f'{debian_name} has no package-owned paths')
        state[logical_name] = {
            'debian_name': debian_name,
            'debian_version': version,
            'upstream_version': _upstream_version(version),
            'owned_paths': paths,
        }
    return state


def _installed_cli_check(
    ros_distro: str,
    *,
    mode: str,
    expected_version: str,
) -> tuple[bool, str]:
    prefix = Path('/opt/ros') / ros_distro
    if mode == 'upgrade-baseline':
        command = prefix / 'bin' / 'lidarslam-map'
        try:
            observed = _run([str(command), '--version']).strip()
        except InstallCheckError as exc:
            return False, str(exc)
        expected = f'lidarslam_ros2 {expected_version}'
        return (
            observed == expected,
            f'expected {expected!r}; found {observed!r}',
        )

    checker_path = REPO_ROOT / 'scripts' / 'check_installed_product_cli.py'
    spec = importlib.util.spec_from_file_location(
        'installed_product_cli_for_package_manager',
        checker_path,
    )
    if spec is None or spec.loader is None:
        raise InstallCheckError(
            f'cannot load installed CLI checker: {checker_path}')
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)
    try:
        checker.validate_install(prefix)
    except (OSError, RuntimeError, ValueError) as exc:
        return False, str(exc)
    return True, f'installed CLI contract passed under {prefix}'


def _check(check_id: str, passed: bool, detail: str) -> dict[str, Any]:
    return {
        'id': check_id,
        'status': 'PASS' if passed else 'FAIL',
        'detail': detail,
    }


def _snapshot_digest(package_state: dict[str, dict[str, Any]]) -> str:
    canonical = json.dumps(
        {
            name: {
                'debian_name': value['debian_name'],
                'debian_version': value['debian_version'],
                'owned_paths': value['owned_paths'],
            }
            for name, value in sorted(package_state.items())
        },
        sort_keys=True,
        separators=(',', ':'),
    ).encode()
    return hashlib.sha256(canonical).hexdigest()


def _load_baseline(path: Path) -> dict[str, Any]:
    try:
        report = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        raise InstallCheckError(
            f'cannot read baseline report {path}: {exc}') from exc
    if not isinstance(report, dict):
        raise InstallCheckError('baseline report root must be an object')
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    try:
        jsonschema.Draft7Validator(schema).validate(report)
    except jsonschema.ValidationError as exc:
        raise InstallCheckError(
            f'baseline report is not schema-valid: {exc.message}') from exc
    return report


def evaluate_install(
    *,
    ros_distro: str,
    expected_version: str,
    mode: str,
    channel: str,
    package_state: dict[str, dict[str, Any]],
    cli_result: tuple[bool, str],
    source_state: dict[str, Any],
    baseline_report: dict[str, Any] | None = None,
    existing_paths: set[str] | None = None,
) -> dict[str, Any]:
    """Evaluate collected package state as one schema-valid evidence report."""
    if ros_distro not in {'humble', 'jazzy'}:
        raise InstallCheckError(f'unsupported ROS distro: {ros_distro}')
    if mode not in {'clean-install', 'upgrade-baseline', 'upgrade-candidate'}:
        raise InstallCheckError(f'unsupported mode: {mode}')
    if channel not in {'main', 'testing'}:
        raise InstallCheckError(f'unsupported apt channel: {channel}')
    _version_tuple(expected_version)

    expected_names = {
        *PRODUCT_PACKAGES,
        *DEPENDENCY_MINIMUMS,
    }
    if set(package_state) != expected_names:
        raise InstallCheckError(
            f'package state differs from required set: '
            f'missing={sorted(expected_names - set(package_state))}, '
            f'extra={sorted(set(package_state) - expected_names)}'
        )

    checks: list[dict[str, Any]] = [
        _check(
            'contract-worktree-clean',
            (
                re.fullmatch(
                    r'[a-f0-9]{40}',
                    str(source_state.get('git_commit', '')),
                ) is not None
                and source_state.get('git_dirty') is False
            ),
            f"commit={source_state.get('git_commit')}, "
            f"dirty={source_state.get('git_dirty')}",
        ),
    ]
    for name in PRODUCT_PACKAGES:
        observed = package_state[name]['upstream_version']
        checks.append(_check(
            f'product-version-{name}',
            observed == expected_version,
            f'expected {expected_version}; found {observed} '
            f"({package_state[name]['debian_version']})",
        ))
    for name, minimum in DEPENDENCY_MINIMUMS.items():
        observed = package_state[name]['upstream_version']
        checks.append(_check(
            f'dependency-minimum-{name}',
            _version_tuple(observed) >= _version_tuple(minimum),
            f'required >= {minimum}; found {observed} '
            f"({package_state[name]['debian_version']})",
        ))
    checks.append(_check('installed-cli-contract', *cli_result))

    baseline = None
    if mode == 'upgrade-baseline':
        if baseline_report is not None:
            raise InstallCheckError(
                'upgrade-baseline does not accept a baseline report')
    elif mode == 'upgrade-candidate':
        if baseline_report is None:
            raise InstallCheckError(
                'upgrade-candidate requires --baseline-report')
        baseline_version = baseline_report['expected_version']
        baseline = {
            'expected_version': baseline_version,
            'status': baseline_report['status'],
            'snapshot_sha256': baseline_report['snapshot']['sha256'],
        }
        checks.append(_check(
            'baseline-report-pass',
            (
                baseline_report['status'] == 'PASS'
                and baseline_report['mode'] == 'upgrade-baseline'
                and baseline_report['ros_distro'] == ros_distro
                and baseline_report['apt_channel'] == 'main'
            ),
            f"status={baseline_report['status']}, "
            f"mode={baseline_report['mode']}, "
            f"distro={baseline_report['ros_distro']}, "
            f"channel={baseline_report['apt_channel']}",
        ))
        checks.append(_check(
            'version-increased',
            _version_tuple(expected_version) > _version_tuple(
                baseline_version),
            f'baseline={baseline_version}, candidate={expected_version}',
        ))
        current_owned = {
            path
            for package in package_state.values()
            for path in package['owned_paths']
        }
        previous_owned = {
            path
            for package in baseline_report['packages'].values()
            for path in package['owned_paths']
        }
        removed_ownership = previous_owned - current_owned
        present = existing_paths
        if present is None:
            stale_paths = sorted(
                path for path in removed_ownership if Path(path).exists())
        else:
            stale_paths = sorted(removed_ownership & present)
        checks.append(_check(
            'no-stale-package-paths',
            not stale_paths,
            'no removed package-owned paths remain'
            if not stale_paths else
            f'stale paths remain: {stale_paths}',
        ))
    elif baseline_report is not None:
        raise InstallCheckError(
            'clean-install does not accept a baseline report')

    status = (
        'PASS'
        if all(check['status'] == 'PASS' for check in checks)
        else 'FAIL'
    )
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'status': status,
        'mode': mode,
        'ros_distro': ros_distro,
        'apt_channel': channel,
        'expected_version': expected_version,
        'source': source_state,
        'packages': package_state,
        'snapshot': {
            'package_count': len(package_state),
            'owned_path_count': sum(
                len(package['owned_paths'])
                for package in package_state.values()
            ),
            'sha256': _snapshot_digest(package_state),
        },
        'baseline': baseline,
        'checks': checks,
    }
    schema = json.loads(SCHEMA_PATH.read_text(encoding='utf-8'))
    jsonschema.Draft7Validator.check_schema(schema)
    jsonschema.Draft7Validator(schema).validate(report)
    return report


def _summary(report: dict[str, Any]) -> str:
    lines = [
        f"Package-manager {report['mode']}: {report['status']}",
        f"ROS {report['ros_distro']} / {report['apt_channel']} / "
        f"product {report['expected_version']}",
    ]
    lines.extend(
        f"  [{check['status']}] {check['id']}: {check['detail']}"
        for check in report['checks']
    )
    return '\n'.join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--ros-distro',
        required=True,
        choices=('humble', 'jazzy'),
    )
    parser.add_argument('--expected-version', required=True)
    parser.add_argument(
        '--mode',
        required=True,
        choices=('clean-install', 'upgrade-baseline', 'upgrade-candidate'),
    )
    parser.add_argument('--channel', required=True, choices=('main', 'testing'))
    parser.add_argument('--baseline-report', type=Path)
    parser.add_argument('--output-json', type=Path, required=True)
    parser.add_argument('--json', action='store_true')
    args = parser.parse_args(argv)
    try:
        state = collect_package_state(args.ros_distro)
        cli_result = _installed_cli_check(
            args.ros_distro,
            mode=args.mode,
            expected_version=args.expected_version,
        )
        source_state = _source_state()
        baseline = (
            _load_baseline(args.baseline_report)
            if args.baseline_report else None
        )
        report = evaluate_install(
            ros_distro=args.ros_distro,
            expected_version=args.expected_version,
            mode=args.mode,
            channel=args.channel,
            package_state=state,
            cli_result=cli_result,
            source_state=source_state,
            baseline_report=baseline,
        )
        rendered = json.dumps(report, indent=2, sort_keys=True) + '\n'
        args.output_json.write_text(rendered, encoding='utf-8')
        print(rendered if args.json else _summary(report))
    except (
        OSError,
        InstallCheckError,
        json.JSONDecodeError,
        jsonschema.SchemaError,
        jsonschema.ValidationError,
    ) as exc:
        print(f'package-manager install check error: {exc}', file=sys.stderr)
        return 2
    return 0 if report['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
