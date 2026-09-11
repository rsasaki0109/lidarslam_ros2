#!/usr/bin/env python3
"""Import and recheck MID-360 production-candidate bundles."""

from __future__ import annotations

import json
import shutil
import tarfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_production_candidate_bundle import (
    BUNDLE_MANIFEST_JSON,
    BUNDLE_MANIFEST_MARKDOWN,
    render_bundle_markdown,
)
from lidarslam_benchmark_tools.mid360_robot_production_candidate_session import (
    Mid360ProductionCandidateSessionRunner,
    ProductionCandidateSessionOptions,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
BUNDLE_IMPORT_JSON = 'mid360_robot_production_candidate_bundle_import.json'
BUNDLE_IMPORT_MARKDOWN = 'mid360_robot_production_candidate_bundle_import.md'


@dataclass(frozen=True)
class ImportOptions:
    """Options for importing and optionally rechecking a bundle."""

    bundle_path: Path
    output_dir: Path | None = None
    recheck: bool = False
    verify: bool = False
    force: bool = False
    bag_root: Path | None = None
    min_bag_duration_sec: float | None = None


class Mid360ProductionCandidateBundleImporter:
    """Import a portable production-candidate bundle."""

    def __init__(self, repo_root: Path = REPO_ROOT) -> None:
        self._repo_root = repo_root

    def import_bundle(self, options: ImportOptions, *, quiet: bool = False) -> dict[str, Any]:
        """Import the bundle and optionally re-run the production gate."""
        source = options.bundle_path.expanduser().resolve()
        output_dir = _resolve_output_dir(source, options.output_dir)
        _prepare_output_dir(output_dir, options.force)
        bundle_dir = _stage_bundle(source, output_dir)
        manifest_path = bundle_dir / BUNDLE_MANIFEST_JSON
        manifest = _load_json(manifest_path)
        verification = _verify_imported_bundle(bundle_dir, manifest)
        recheck = self._maybe_recheck(
            bundle_dir=bundle_dir,
            manifest=manifest,
            verification=verification,
            options=options,
            quiet=quiet,
        )
        report = _build_import_report(
            source=source,
            bundle_dir=bundle_dir,
            manifest=manifest,
            verification=verification,
            recheck=recheck,
            requested_recheck=options.recheck,
        )
        _write_import_report(bundle_dir, report)
        _update_bundle_manifest(bundle_dir, manifest, report)
        return report

    def _maybe_recheck(
        self,
        *,
        bundle_dir: Path,
        manifest: dict[str, Any],
        verification: dict[str, Any],
        options: ImportOptions,
        quiet: bool,
    ) -> dict[str, Any]:
        if not options.recheck:
            return {
                'requested': False,
                'status': 'SKIPPED',
                'message': 'Recheck was not requested.',
                'command': _recheck_command(bundle_dir, manifest, options),
            }
        if verification['status'] != 'PASS':
            return {
                'requested': True,
                'status': 'SKIPPED',
                'message': 'Recheck skipped because bundle verification failed.',
                'command': _recheck_command(bundle_dir, manifest, options),
            }
        session_path = bundle_dir / 'artifacts' / 'mid360_robot_production_candidate_session.json'
        session = _load_json(session_path)
        command = _recheck_command(bundle_dir, manifest, options)
        try:
            report = Mid360ProductionCandidateSessionRunner(self._repo_root).run(
                _recheck_options(bundle_dir, manifest, session, options),
                quiet=quiet,
            )
        except Exception as exc:
            return {
                'requested': True,
                'status': 'FAIL',
                'message': f'Recheck failed to execute: {exc}',
                'command': command,
                'returncode': 1,
            }
        return {
            'requested': True,
            'status': str(report.get('status') or 'FAIL').upper(),
            'message': 'Production candidate artifacts rechecked.',
            'command': command,
            'returncode': 0 if report.get('status') == 'PASS' else 1,
            'production_readiness_json': str(bundle_dir / 'artifacts' / 'mid360_robot_production_readiness.json'),
            'dashboard_html': str(bundle_dir / 'artifacts' / 'mid360_robot_session_dashboard.html'),
            'session_report': report,
        }


def render_import_markdown(report: dict[str, Any]) -> str:
    """Render an import/recheck report."""
    verification = report.get('verification') or {}
    recheck = report.get('recheck') or {}
    lines = [
        '# MID-360 Production Candidate Bundle Import',
        '',
        f"- status: `{report.get('status', '')}`",
        f"- created_at: `{report.get('created_at', '')}`",
        f"- source_bundle: `{report.get('source_bundle', '')}`",
        f"- bundle_dir: `{report.get('bundle_dir', '')}`",
        f"- manifest_status: `{report.get('manifest_status', '')}`",
        f"- verification_status: `{verification.get('status', '')}`",
        f"- recheck_status: `{recheck.get('status', '')}`",
        '',
        '## Recheck Command',
        '',
        '```bash',
        ' '.join(str(item) for item in recheck.get('command') or []),
        '```',
        '',
        '## Verification Errors',
        '',
    ]
    errors = verification.get('errors') or []
    if errors:
        lines.extend(f'- {error}' for error in errors)
    else:
        lines.append('- none')
    return '\n'.join(lines)


def _stage_bundle(source: Path, output_dir: Path) -> Path:
    if source.is_dir():
        _copy_dir_contents(source, output_dir)
        return output_dir
    if not source.is_file():
        raise FileNotFoundError(f'bundle not found: {source}')
    _safe_extract_tarball(source, output_dir)
    return output_dir


def _copy_dir_contents(source: Path, output_dir: Path) -> None:
    for child in source.iterdir():
        target = output_dir / child.name
        if child.is_dir():
            shutil.copytree(child, target)
        elif child.is_file():
            shutil.copy2(child, target)


def _safe_extract_tarball(tarball: Path, output_dir: Path) -> None:
    with tarfile.open(tarball, 'r:gz') as archive:
        members = archive.getmembers()
        top_level = _single_top_level(members)
        for member in members:
            if member.isdir():
                relative = _safe_member_path(member.name, top_level)
                if relative is not None:
                    (output_dir / relative).mkdir(parents=True, exist_ok=True)
                continue
            if not member.isfile():
                raise ValueError(f'unsupported tar member type: {member.name}')
            relative = _safe_member_path(member.name, top_level)
            if relative is None:
                continue
            destination = output_dir / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            extracted = archive.extractfile(member)
            if extracted is None:
                raise ValueError(f'failed to read tar member: {member.name}')
            with extracted, destination.open('wb') as handle:
                shutil.copyfileobj(extracted, handle)


def _safe_member_path(name: str, top_level: str | None) -> Path | None:
    pure = PurePosixPath(name)
    if pure.is_absolute() or any(part == '..' for part in pure.parts):
        raise ValueError(f'unsafe tar member path: {name}')
    parts = list(pure.parts)
    if top_level and parts and parts[0] == top_level:
        parts = parts[1:]
    if not parts:
        return None
    return Path(*parts)


def _single_top_level(members: list[tarfile.TarInfo]) -> str | None:
    top = {
        PurePosixPath(member.name).parts[0]
        for member in members
        if PurePosixPath(member.name).parts
    }
    return next(iter(top)) if len(top) == 1 else None


def _verify_imported_bundle(bundle_dir: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    errors = []
    if not manifest:
        errors.append(f'missing or unreadable {BUNDLE_MANIFEST_JSON}')
        return {
            'status': 'FAIL',
            'errors': errors,
            'checked_required_files': [],
            'missing_required_files': [],
        }
    missing = []
    for path_text in manifest.get('required_files') or []:
        path = bundle_dir / str(path_text)
        if not path.is_file():
            missing.append(str(path_text))
    for item in manifest.get('missing_required') or []:
        destination = str(item.get('destination') or '')
        if destination and destination not in missing:
            missing.append(destination)
    if missing:
        errors.extend(f'missing required file: {path}' for path in missing)
    if manifest.get('status') != 'PASS':
        errors.append(f"bundle manifest status is {manifest.get('status')}")
    return {
        'status': 'FAIL' if errors else 'PASS',
        'errors': errors,
        'checked_required_files': list(manifest.get('required_files') or []),
        'missing_required_files': missing,
    }


def _recheck_options(
    bundle_dir: Path,
    manifest: dict[str, Any],
    session: dict[str, Any],
    options: ImportOptions,
) -> ProductionCandidateSessionOptions:
    thresholds = session.get('thresholds') or {}
    min_duration = (
        options.min_bag_duration_sec
        if options.min_bag_duration_sec is not None
        else float(thresholds.get('min_bag_duration_sec') or 600.0)
    )
    return ProductionCandidateSessionOptions(
        profile_path=_profile_path(bundle_dir, manifest),
        bag_root=(options.bag_root.expanduser().resolve() if options.bag_root else bundle_dir / 'recheck_recording'),
        output_dir=bundle_dir / 'artifacts',
        run_id=str(manifest.get('run_id') or session.get('run_id') or bundle_dir.name),
        duration_sec=f'{min_duration:g}',
        from_existing_artifacts=True,
        execute=True,
        min_bag_duration_sec=float(min_duration),
        min_pointcloud_hz=float(thresholds.get('min_pointcloud_hz') or 5.0),
        min_imu_hz=float(thresholds.get('min_imu_hz') or 50.0),
        allow_warnings=bool(thresholds.get('allow_warnings')),
        allow_public_bag=bool(thresholds.get('allow_public_bag')),
    )


def _recheck_command(
    bundle_dir: Path,
    manifest: dict[str, Any],
    options: ImportOptions,
) -> list[str]:
    min_duration = options.min_bag_duration_sec
    duration = f'{min_duration:g}' if min_duration is not None else '<bundle threshold>'
    bag_root = options.bag_root.expanduser().resolve() if options.bag_root else bundle_dir / 'recheck_recording'
    return [
        'bash',
        'scripts/run_mid360_robot_production_candidate_session.sh',
        '--robot-profile',
        str(_profile_path(bundle_dir, manifest)),
        '--bag-root',
        str(bag_root),
        '--run-id',
        str(manifest.get('run_id') or bundle_dir.name),
        '--duration-sec',
        duration,
        '--output-dir',
        str(bundle_dir / 'artifacts'),
        '--from-existing-artifacts',
        '--run',
    ]


def _profile_path(bundle_dir: Path, manifest: dict[str, Any]) -> Path:
    for item in manifest.get('included') or []:
        if item.get('key') == 'profile_snapshot':
            return bundle_dir / str(item.get('destination'))
    matches = sorted((bundle_dir / 'recording').glob('*_profile.yaml'))
    if matches:
        return matches[0]
    return bundle_dir / 'recording' / 'profile_snapshot.yaml'


def _build_import_report(
    *,
    source: Path,
    bundle_dir: Path,
    manifest: dict[str, Any],
    verification: dict[str, Any],
    recheck: dict[str, Any],
    requested_recheck: bool,
) -> dict[str, Any]:
    status = 'PASS'
    if verification.get('status') != 'PASS':
        status = 'FAIL'
    if requested_recheck and recheck.get('status') != 'PASS':
        status = 'FAIL'
    return {
        'created_at': datetime.now(timezone.utc).isoformat(),
        'status': status,
        'source_bundle': str(source),
        'bundle_dir': str(bundle_dir),
        'manifest_path': str(bundle_dir / BUNDLE_MANIFEST_JSON),
        'manifest_status': manifest.get('status', 'MISSING') if manifest else 'MISSING',
        'run_id': manifest.get('run_id', '') if manifest else '',
        'verification': verification,
        'recheck': recheck,
    }


def _write_import_report(bundle_dir: Path, report: dict[str, Any]) -> None:
    (bundle_dir / BUNDLE_IMPORT_JSON).write_text(payload_to_json(report) + '\n', encoding='utf-8')
    (bundle_dir / BUNDLE_IMPORT_MARKDOWN).write_text(render_import_markdown(report) + '\n', encoding='utf-8')


def _update_bundle_manifest(bundle_dir: Path, manifest: dict[str, Any], report: dict[str, Any]) -> None:
    if not manifest:
        return
    manifest['last_import'] = {
        'created_at': report.get('created_at', ''),
        'status': report.get('status', ''),
        'bundle_dir': str(bundle_dir),
        'verification_status': (report.get('verification') or {}).get('status', ''),
        'recheck_status': (report.get('recheck') or {}).get('status', ''),
        'import_report_json': BUNDLE_IMPORT_JSON,
    }
    manifest['files'] = _list_files(bundle_dir)
    (bundle_dir / BUNDLE_MANIFEST_JSON).write_text(payload_to_json(manifest) + '\n', encoding='utf-8')
    (bundle_dir / BUNDLE_MANIFEST_MARKDOWN).write_text(render_bundle_markdown(manifest) + '\n', encoding='utf-8')


def _resolve_output_dir(source: Path, output_dir: Path | None) -> Path:
    if output_dir is not None:
        return output_dir.expanduser().resolve()
    if source.suffixes[-2:] == ['.tar', '.gz']:
        return source.with_suffix('').with_suffix('')
    if source.suffix == '.tgz':
        return source.with_suffix('')
    return source.parent / f'{source.name}_imported'


def _prepare_output_dir(output_dir: Path, force: bool) -> None:
    if output_dir.exists():
        if not force:
            raise FileExistsError(f'import output dir already exists: {output_dir}')
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=False)


def _list_files(bundle_dir: Path) -> list[str]:
    return sorted(
        str(path.relative_to(bundle_dir))
        for path in bundle_dir.rglob('*')
        if path.is_file()
    )


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


if __name__ == "__main__":
    raise SystemExit("mid360_robot_production_candidate_bundle_import is a library module; import it instead of running it directly.")
