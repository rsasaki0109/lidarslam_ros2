#!/usr/bin/env python3
"""Validate an in-place lidar_slam_ros2 package upgrade against a fresh install."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys
import tarfile
import tempfile
import time
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[1]
INSTALL_CHECKER = REPO_ROOT / 'scripts' / 'check_installed_product_cli.py'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/install-upgrade-v1.schema.json'
)
REPORT_NAME = 'install_upgrade_report.json'


def _git_command(*args: str) -> list[str]:
    return [
        'git',
        '-c',
        f'safe.directory={REPO_ROOT}',
        *args,
    ]


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(
        json.dumps(value, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    temporary.replace(path)


def _git_output(*args: str) -> str:
    return subprocess.run(
        _git_command(*args),
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _package_version(package_root: Path) -> str:
    import xml.etree.ElementTree as ET

    root = ET.parse(package_root / 'package.xml').getroot()
    value = root.findtext('version')
    if value is None or not value.strip():
        raise ValueError(f'package version is missing: {package_root}')
    return value.strip()


def _extract_baseline(reference: str, target: Path) -> str:
    commit = _git_output('rev-parse', f'{reference}^{{commit}}')
    archive = target.parent / 'baseline.tar'
    subprocess.run(
        _git_command(
            'archive',
            '--format=tar',
            f'--output={archive}',
            commit,
            'lidarslam',
        ),
        cwd=REPO_ROOT,
        check=True,
    )
    target.mkdir(parents=True, exist_ok=False)
    with tarfile.open(archive) as stream:
        stream.extractall(target)
    archive.unlink()
    return commit


def _run_logged(
    command: list[str],
    log_path: Path,
    cwd: Path,
) -> dict[str, Any]:
    started = time.monotonic()
    completed = subprocess.run(
        command,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )
    duration = time.monotonic() - started
    log_path.write_text(
        '$ ' + ' '.join(command) + '\n\n'
        + completed.stdout
        + completed.stderr,
        encoding='utf-8',
    )
    return {
        'exit_code': completed.returncode,
        'duration_sec': duration,
        'log': log_path.name,
        'log_sha256': _sha256(log_path),
    }


def _build(
    source: Path,
    build_base: Path,
    install_base: Path,
    log_base: Path,
    log_path: Path,
    cmake_build_type: str,
    source_revision: str | None = None,
    source_dirty: bool | None = None,
) -> dict[str, Any]:
    cmake_args = [f'-DCMAKE_BUILD_TYPE={cmake_build_type}']
    if source_revision is not None:
        cmake_args.append(
            f'-DLIDARSLAM_SOURCE_REVISION:STRING={source_revision}'
        )
    if source_dirty is not None:
        cmake_args.append(
            '-DLIDARSLAM_SOURCE_DIRTY:STRING='
            + ('true' if source_dirty else 'false')
        )
    return _run_logged(
        [
            'colcon',
            '--log-base',
            str(log_base),
            'build',
            '--base-paths',
            str(source),
            '--build-base',
            str(build_base),
            '--install-base',
            str(install_base),
            '--merge-install',
            '--event-handlers',
            'console_direct+',
            '--cmake-args',
            *cmake_args,
        ],
        log_path,
        REPO_ROOT,
    )


def _owned_paths(prefix: Path) -> list[Path]:
    candidates: list[Path] = []
    path_command = prefix / 'bin' / 'lidarslam-map'
    if path_command.exists() or path_command.is_symlink():
        candidates.append(path_command)
    for root in (prefix / 'lib' / 'lidarslam', prefix / 'share' / 'lidarslam'):
        if root.is_dir():
            candidates.extend(
                path
                for path in root.rglob('*')
                if path.is_file() or path.is_symlink()
            )
    index_root = prefix / 'share' / 'ament_index' / 'resource_index'
    if index_root.is_dir():
        candidates.extend(
            path
            for path in index_root.glob('*/lidarslam')
            if path.is_file() or path.is_symlink()
        )
    return sorted(set(candidates))


def _normalized_text_sha256(path: Path, prefix: Path) -> str | None:
    if path.is_symlink():
        text = os.readlink(path).replace(str(prefix), '<PREFIX>')
    else:
        data = path.read_bytes()
        if b'\0' in data:
            return None
        try:
            text = data.decode('utf-8')
        except UnicodeDecodeError:
            return None
        text = text.replace(str(prefix), '<PREFIX>')
    return hashlib.sha256(text.encode()).hexdigest()


def snapshot_prefix(prefix: Path) -> dict[str, dict[str, Any]]:
    """Return package-owned paths without including binary contents."""
    snapshot: dict[str, dict[str, Any]] = {}
    for path in _owned_paths(prefix):
        relative = str(path.relative_to(prefix))
        mode = path.lstat().st_mode
        is_symlink = stat.S_ISLNK(mode)
        snapshot[relative] = {
            'kind': 'symlink' if is_symlink else 'file',
            'executable': bool(mode & stat.S_IXUSR),
            'size_bytes': (
                len(os.readlink(path).encode())
                if is_symlink
                else path.stat().st_size
            ),
            'normalized_text_sha256': _normalized_text_sha256(path, prefix),
        }
    return snapshot


def compare_snapshots(
    upgraded: dict[str, dict[str, Any]],
    fresh: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    """Compare package-owned install shape and normalized text resources."""
    upgraded_paths = set(upgraded)
    fresh_paths = set(fresh)
    stale = sorted(upgraded_paths - fresh_paths)
    missing = sorted(fresh_paths - upgraded_paths)
    mismatched: list[dict[str, Any]] = []
    for path in sorted(upgraded_paths & fresh_paths):
        left = upgraded[path]
        right = fresh[path]
        fields = [
            field
            for field in ('kind', 'executable', 'normalized_text_sha256')
            if left[field] != right[field]
        ]
        if fields:
            mismatched.append({'path': path, 'fields': fields})
    return {
        'stale_paths': stale,
        'missing_paths': missing,
        'mismatched_paths': mismatched,
    }


def _check(check_id: str, passed: bool, observed: str) -> dict[str, Any]:
    return {'id': check_id, 'passed': passed, 'observed': observed}


def run(args: argparse.Namespace) -> int:
    evidence_dir = args.evidence_dir.expanduser().resolve()
    if evidence_dir.exists() and any(evidence_dir.iterdir()):
        raise ValueError(f'evidence directory is not empty: {evidence_dir}')
    evidence_dir.mkdir(parents=True, exist_ok=True)
    started_at = _utc_now()
    started = time.monotonic()

    candidate_commit = _git_output('rev-parse', 'HEAD')
    candidate_dirty = bool(_git_output('status', '--porcelain'))
    baseline_commit = _git_output(
        'rev-parse',
        f'{args.baseline_ref}^{{commit}}',
    )

    with tempfile.TemporaryDirectory(prefix='lidarslam-upgrade-') as temporary:
        work = Path(temporary)
        baseline_source = work / 'baseline-source'
        _extract_baseline(args.baseline_ref, baseline_source)
        baseline_package = baseline_source / 'lidarslam'
        candidate_package = REPO_ROOT / 'lidarslam'
        upgrade_prefix = work / 'upgrade-prefix'
        fresh_prefix = work / 'fresh-prefix'

        print(f'==> Building baseline {args.baseline_ref} ({baseline_commit})')
        baseline_build = _build(
            baseline_package,
            work / 'baseline-build',
            upgrade_prefix,
            work / 'baseline-log',
            evidence_dir / 'baseline-build.log',
            args.cmake_build_type,
        )
        baseline_snapshot = (
            snapshot_prefix(upgrade_prefix)
            if baseline_build['exit_code'] == 0
            else {}
        )

        print(f'==> Upgrading prefix to candidate {candidate_commit}')
        upgrade_build = _build(
            candidate_package,
            work / 'upgrade-build',
            upgrade_prefix,
            work / 'upgrade-log',
            evidence_dir / 'upgrade-build.log',
            args.cmake_build_type,
            candidate_commit,
            candidate_dirty,
        )

        print('==> Building fresh candidate comparison prefix')
        fresh_build = _build(
            candidate_package,
            work / 'fresh-build',
            fresh_prefix,
            work / 'fresh-log',
            evidence_dir / 'fresh-build.log',
            args.cmake_build_type,
            candidate_commit,
            candidate_dirty,
        )

        upgraded_snapshot = (
            snapshot_prefix(upgrade_prefix)
            if upgrade_build['exit_code'] == 0
            else {}
        )
        fresh_snapshot = (
            snapshot_prefix(fresh_prefix)
            if fresh_build['exit_code'] == 0
            else {}
        )
        comparison = compare_snapshots(upgraded_snapshot, fresh_snapshot)

        upgrade_validation = _run_logged(
            [
                sys.executable,
                str(INSTALL_CHECKER),
                '--prefix',
                str(upgrade_prefix),
                '--expected-source-revision',
                candidate_commit,
            ],
            evidence_dir / 'upgrade-validation.log',
            REPO_ROOT,
        ) if upgrade_build['exit_code'] == 0 else {
            'exit_code': 125,
            'duration_sec': 0.0,
            'log': 'upgrade-validation.log',
            'log_sha256': None,
        }
        fresh_validation = _run_logged(
            [
                sys.executable,
                str(INSTALL_CHECKER),
                '--prefix',
                str(fresh_prefix),
                '--expected-source-revision',
                candidate_commit,
            ],
            evidence_dir / 'fresh-validation.log',
            REPO_ROOT,
        ) if fresh_build['exit_code'] == 0 else {
            'exit_code': 125,
            'duration_sec': 0.0,
            'log': 'fresh-validation.log',
            'log_sha256': None,
        }

        checks = [
            _check(
                'candidate_revision_clean',
                not candidate_dirty,
                f'commit={candidate_commit}, dirty={candidate_dirty}',
            ),
            _check(
                'baseline_build_passed',
                baseline_build['exit_code'] == 0,
                f"exit_code={baseline_build['exit_code']}",
            ),
            _check(
                'candidate_upgrade_build_passed',
                upgrade_build['exit_code'] == 0,
                f"exit_code={upgrade_build['exit_code']}",
            ),
            _check(
                'candidate_fresh_build_passed',
                fresh_build['exit_code'] == 0,
                f"exit_code={fresh_build['exit_code']}",
            ),
            _check(
                'upgrade_has_no_stale_paths',
                not comparison['stale_paths'],
                f"stale_paths={comparison['stale_paths']}",
            ),
            _check(
                'upgrade_has_no_missing_paths',
                not comparison['missing_paths'],
                f"missing_paths={comparison['missing_paths']}",
            ),
            _check(
                'upgrade_metadata_matches_fresh',
                not comparison['mismatched_paths'],
                f"mismatched_paths={comparison['mismatched_paths']}",
            ),
            _check(
                'upgraded_product_cli_valid',
                upgrade_validation['exit_code'] == 0,
                f"exit_code={upgrade_validation['exit_code']}",
            ),
            _check(
                'fresh_product_cli_valid',
                fresh_validation['exit_code'] == 0,
                f"exit_code={fresh_validation['exit_code']}",
            ),
            _check(
                'historical_node_preserved',
                (
                    'lib/lidarslam/lidarslam' in upgraded_snapshot
                    and upgraded_snapshot[
                        'lib/lidarslam/lidarslam'
                    ]['executable']
                ),
                'lib/lidarslam/lidarslam',
            ),
            _check(
                'product_cli_added_or_preserved',
                (
                    'bin/lidarslam-map' in upgraded_snapshot
                    and upgraded_snapshot['bin/lidarslam-map']['executable']
                ),
                'bin/lidarslam-map',
            ),
        ]
        report = {
            'schema_version': 1,
            'schema_uri': SCHEMA_URI,
            'status': (
                'passed'
                if all(check['passed'] for check in checks)
                else 'failed'
            ),
            'started_at': started_at,
            'finished_at': _utc_now(),
            'duration_sec': time.monotonic() - started,
            'hardware_label': args.hardware_label,
            'ros_distro': os.environ.get('ROS_DISTRO'),
            'baseline': {
                'reference': args.baseline_ref,
                'commit': baseline_commit,
                'package_version': _package_version(baseline_package),
                'owned_file_count': len(baseline_snapshot),
                'has_product_cli': 'bin/lidarslam-map' in baseline_snapshot,
            },
            'candidate': {
                'commit': candidate_commit,
                'dirty': candidate_dirty,
                'package_version': _package_version(candidate_package),
            },
            'builds': {
                'baseline': baseline_build,
                'upgrade': upgrade_build,
                'fresh': fresh_build,
            },
            'validation': {
                'upgrade': upgrade_validation,
                'fresh': fresh_validation,
            },
            'snapshots': {
                'upgraded_file_count': len(upgraded_snapshot),
                'fresh_file_count': len(fresh_snapshot),
                **comparison,
            },
            'checks': checks,
        }
        _atomic_json(evidence_dir / REPORT_NAME, report)
        print(f"install upgrade validation: {report['status']}")
        print(f'- report: {evidence_dir / REPORT_NAME}')
        return 0 if report['status'] == 'passed' else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            'Build a baseline release, upgrade the same non-symlinked prefix, '
            'and compare it with a fresh candidate install.'
        )
    )
    parser.add_argument(
        '--baseline-ref',
        default='v0.6.0',
        help='Immutable release tag or commit to install first (default: v0.6.0).',
    )
    parser.add_argument(
        '--evidence-dir',
        type=Path,
        required=True,
        help='New or empty directory for the report and build logs.',
    )
    parser.add_argument(
        '--hardware-label',
        required=True,
        help='Auditable runner/environment label.',
    )
    parser.add_argument(
        '--cmake-build-type',
        choices=['Release', 'RelWithDebInfo'],
        default='Release',
    )
    args = parser.parse_args()
    try:
        return run(args)
    except (
        FileNotFoundError,
        OSError,
        RuntimeError,
        subprocess.CalledProcessError,
        ValueError,
    ) as exc:
        print(f'error: install upgrade validation could not run: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
