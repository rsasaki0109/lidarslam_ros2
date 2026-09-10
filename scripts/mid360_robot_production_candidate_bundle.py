#!/usr/bin/env python3
"""Export MID-360 production-candidate artifacts as a portable bundle."""

from __future__ import annotations

import json
import shutil
import tarfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


BUNDLE_MANIFEST_JSON = 'mid360_robot_production_candidate_bundle.json'
BUNDLE_MANIFEST_MARKDOWN = 'mid360_robot_production_candidate_bundle.md'
DEFAULT_SESSION_JSON = 'mid360_robot_production_candidate_session.json'
DEFAULT_SESSION_MARKDOWN = 'mid360_robot_production_candidate_session.md'
DEFAULT_DASHBOARD_HTML = 'mid360_robot_session_dashboard.html'
DEFAULT_PUBLIC_GATE_JSON = 'public_rko_adoption_gate/mid360_robot_public_rko_adoption_gate.json'


@dataclass(frozen=True)
class BundleOptions:
    """Options for exporting a production-candidate bundle."""

    artifact_dir: Path
    output_path: Path | None = None
    label: str = ''
    force: bool = False


@dataclass(frozen=True)
class BundlePath:
    """One file that may be included in a production-candidate bundle."""

    key: str
    source: Path
    destination: Path
    required: bool


class Mid360ProductionCandidateBundleExporter:
    """Stage and archive production-candidate artifacts."""

    def export(self, options: BundleOptions) -> dict[str, Any]:
        """Export a bundle and return its manifest."""
        artifact_dir = options.artifact_dir.expanduser().resolve()
        output_path = _resolve_output_path(artifact_dir, options.output_path)
        bundle_dir, tarball_path = _bundle_targets(output_path)
        _prepare_bundle_dir(bundle_dir, options.force)

        session = _load_json(artifact_dir / DEFAULT_SESSION_JSON)
        paths = _bundle_paths(artifact_dir, session)
        included, missing = self._copy_paths(bundle_dir, paths)

        manifest = self._build_manifest(
            artifact_dir=artifact_dir,
            bundle_dir=bundle_dir,
            tarball_path=tarball_path,
            label=options.label,
            session=session,
            included=included,
            missing=missing,
        )
        self._write_manifest(bundle_dir, manifest)
        manifest['files'] = _list_bundle_files(bundle_dir)
        self._write_manifest(bundle_dir, manifest)

        if tarball_path is not None:
            _write_tarball(bundle_dir, tarball_path, options.force)
            manifest['tarball_path'] = str(tarball_path)
            manifest['tarball_verified'] = _verify_tarball(tarball_path, bundle_dir.name, manifest)
            self._write_manifest(bundle_dir, manifest)

        return manifest

    @staticmethod
    def _copy_paths(bundle_dir: Path, paths: list[BundlePath]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
        included = []
        missing = []
        for item in paths:
            source = item.source.expanduser().resolve()
            entry = {
                'key': item.key,
                'source': str(source),
                'destination': str(item.destination),
                'required': item.required,
            }
            if not source.is_file():
                if item.required:
                    missing.append(entry)
                continue
            destination = bundle_dir / item.destination
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, destination)
            entry['size_bytes'] = destination.stat().st_size
            included.append(entry)
        return included, missing

    @staticmethod
    def _build_manifest(
        *,
        artifact_dir: Path,
        bundle_dir: Path,
        tarball_path: Path | None,
        label: str,
        session: dict[str, Any],
        included: list[dict[str, Any]],
        missing: list[dict[str, Any]],
    ) -> dict[str, Any]:
        required_files = sorted(
            item['destination'] for item in included if item.get('required')
        )
        status = 'FAIL' if missing else 'PASS'
        run_id = str(session.get('run_id') or artifact_dir.name)
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'bundle_label': label or f'{run_id}_production_candidate_bundle',
            'run_id': run_id,
            'source_artifact_dir': str(artifact_dir),
            'bundle_dir': str(bundle_dir),
            'tarball_path': str(tarball_path) if tarball_path is not None else '',
            'tarball_verified': False,
            'session_status': session.get('status', ''),
            'production_readiness_json': 'artifacts/mid360_robot_production_readiness.json',
            'recheck_command': [
                'bash',
                'scripts/run_mid360_robot_production_candidate_session.sh',
                '--robot-profile',
                f'<bundle_dir>/recording/{_profile_snapshot_name(session)}',
                '--bag-root',
                str(session.get('bag_root') or '<bag_root>'),
                '--run-id',
                run_id,
                '--duration-sec',
                str((session.get('thresholds') or {}).get('min_bag_duration_sec') or 600),
                '--output-dir',
                '<bundle_dir>/artifacts',
                '--from-existing-artifacts',
                '--run',
            ],
            'included': included,
            'missing_required': missing,
            'required_files': required_files,
            'files': [],
        }

    @staticmethod
    def _write_manifest(bundle_dir: Path, manifest: dict[str, Any]) -> None:
        (bundle_dir / BUNDLE_MANIFEST_JSON).write_text(
            payload_to_json(manifest) + '\n',
            encoding='utf-8',
        )
        (bundle_dir / BUNDLE_MANIFEST_MARKDOWN).write_text(
            render_bundle_markdown(manifest) + '\n',
            encoding='utf-8',
        )


def render_bundle_markdown(manifest: dict[str, Any]) -> str:
    """Render a production-candidate bundle manifest."""
    lines = [
        '# MID-360 Production Candidate Bundle',
        '',
        f"- status: `{manifest.get('status', '')}`",
        f"- bundle_label: `{manifest.get('bundle_label', '')}`",
        f"- run_id: `{manifest.get('run_id', '')}`",
        f"- source_artifact_dir: `{manifest.get('source_artifact_dir', '')}`",
        f"- bundle_dir: `{manifest.get('bundle_dir', '')}`",
        f"- tarball_path: `{manifest.get('tarball_path', '')}`",
        f"- tarball_verified: `{manifest.get('tarball_verified')}`",
        '',
        '## Recheck Command',
        '',
        '```bash',
        ' '.join(str(item) for item in manifest.get('recheck_command') or []),
        '```',
        '',
        '## Missing Required',
        '',
    ]
    missing = manifest.get('missing_required') or []
    if missing:
        for item in missing:
            lines.append(f"- `{item.get('destination', '')}` from `{item.get('source', '')}`")
    else:
        lines.append('- none')
    lines.extend(['', '## Included', ''])
    included = manifest.get('included') or []
    if included:
        for item in included:
            marker = 'required' if item.get('required') else 'optional'
            lines.append(f"- `{item.get('destination', '')}` ({marker})")
    else:
        lines.append('- none')
    return '\n'.join(lines)


def verify_bundle_manifest(manifest: dict[str, Any]) -> tuple[bool, list[str]]:
    """Verify a bundle manifest has all required files and archive members."""
    errors = []
    if manifest.get('missing_required'):
        errors.append('required artifact missing')
    if manifest.get('status') != 'PASS':
        errors.append(f"bundle status is {manifest.get('status')}")
    tarball_path = str(manifest.get('tarball_path') or '')
    if tarball_path and not manifest.get('tarball_verified'):
        errors.append('tarball does not contain all required members')
    return not errors, errors


def _bundle_paths(artifact_dir: Path, session: dict[str, Any]) -> list[BundlePath]:
    artifacts = session.get('artifact_paths') or {}
    return [
        _path('production_candidate_session_json', artifact_dir / DEFAULT_SESSION_JSON, 'artifacts/' + DEFAULT_SESSION_JSON, True),
        _path('production_candidate_session_markdown', artifact_dir / DEFAULT_SESSION_MARKDOWN, 'artifacts/' + DEFAULT_SESSION_MARKDOWN, False),
        _path('dashboard_html', _source(artifact_dir, artifacts, 'dashboard_html', artifact_dir / DEFAULT_DASHBOARD_HTML), 'artifacts/' + DEFAULT_DASHBOARD_HTML, False),
        _path('host_readiness_json', _source(artifact_dir, artifacts, 'host_readiness_json', artifact_dir / 'jetson_mid360_host_readiness.json'), 'artifacts/jetson_mid360_host_readiness.json', True),
        _path('host_readiness_markdown', artifact_dir / 'jetson_mid360_host_readiness.md', 'artifacts/jetson_mid360_host_readiness.md', False),
        _path('recording_check_json', _source(artifact_dir, artifacts, 'recording_check_json', artifact_dir / 'mid360_robot_recording_check.json'), 'artifacts/mid360_robot_recording_check.json', True),
        _path('recording_check_markdown', artifact_dir / 'mid360_robot_recording_check.md', 'artifacts/mid360_robot_recording_check.md', False),
        _path('readiness_json', _source(artifact_dir, artifacts, 'readiness_json', artifact_dir / 'mid360_robot_readiness.json'), 'artifacts/mid360_robot_readiness.json', True),
        _path('readiness_markdown', artifact_dir / 'mid360_robot_readiness.md', 'artifacts/mid360_robot_readiness.md', False),
        _path('map_plan_json', _source(artifact_dir, artifacts, 'map_plan_json', artifact_dir / 'mid360_robot_run_plan.json'), 'artifacts/mid360_robot_run_plan.json', False),
        _path('map_plan_markdown', artifact_dir / 'mid360_robot_run_plan.md', 'artifacts/mid360_robot_run_plan.md', False),
        _path('map_diagnosis_json', _source(artifact_dir, artifacts, 'map_diagnosis_json', artifact_dir / 'autoware_map_diagnosis.json'), 'artifacts/autoware_map_diagnosis.json', True),
        _path('map_diagnosis_markdown', artifact_dir / 'autoware_map_diagnosis.md', 'artifacts/autoware_map_diagnosis.md', False),
        _path('public_rko_adoption_gate_json', _source(artifact_dir, artifacts, 'public_rko_adoption_gate_json', artifact_dir / DEFAULT_PUBLIC_GATE_JSON), 'artifacts/' + DEFAULT_PUBLIC_GATE_JSON, True),
        _path('public_rko_adoption_gate_markdown', _source(artifact_dir, artifacts, 'public_rko_adoption_gate_markdown', artifact_dir / 'public_rko_adoption_gate/mid360_robot_public_rko_adoption_gate.md'), 'artifacts/public_rko_adoption_gate/mid360_robot_public_rko_adoption_gate.md', False),
        _path('production_readiness_json', _source(artifact_dir, artifacts, 'production_readiness_json', artifact_dir / 'mid360_robot_production_readiness.json'), 'artifacts/mid360_robot_production_readiness.json', True),
        _path('production_readiness_markdown', artifact_dir / 'mid360_robot_production_readiness.md', 'artifacts/mid360_robot_production_readiness.md', False),
        _path('loop_alignment_json', _source(artifact_dir, artifacts, 'loop_alignment_json', artifact_dir / 'mid360_robot_loop_alignment.json'), 'artifacts/mid360_robot_loop_alignment.json', False),
        _path('loop_alignment_markdown', _source(artifact_dir, artifacts, 'loop_alignment_markdown', artifact_dir / 'mid360_robot_loop_alignment.md'), 'artifacts/mid360_robot_loop_alignment.md', False),
        _path('segment_map_alignment_json', _source(artifact_dir, artifacts, 'segment_map_alignment_json', artifact_dir / 'mid360_robot_public_segment_map_cloud_alignment.json'), 'artifacts/mid360_robot_public_segment_map_cloud_alignment.json', False),
        _path('segment_map_alignment_markdown', _source(artifact_dir, artifacts, 'segment_map_alignment_markdown', artifact_dir / 'mid360_robot_public_segment_map_cloud_alignment.md'), 'artifacts/mid360_robot_public_segment_map_cloud_alignment.md', False),
        _path('segment_map_alignment_ply', _source(artifact_dir, artifacts, 'segment_map_alignment_ply', artifact_dir / 'mid360_robot_public_segment_map_cloud_alignment.ply'), 'artifacts/mid360_robot_public_segment_map_cloud_alignment.ply', False),
        _path('map_preview_json', _source(artifact_dir, artifacts, 'map_preview_json', artifact_dir / 'mid360_robot_3d_map_preview.json'), 'artifacts/mid360_robot_3d_map_preview.json', False),
        _path('map_preview_html', _source(artifact_dir, artifacts, 'map_preview_html', artifact_dir / 'mid360_robot_3d_map_preview.html'), 'artifacts/mid360_robot_3d_map_preview.html', False),
        _path('map_preview_ply', _source(artifact_dir, artifacts, 'map_preview_ply', artifact_dir / 'mid360_robot_3d_map_preview.ply'), 'artifacts/mid360_robot_3d_map_preview.ply', False),
        _path('map_preview_overlay_json', _source(artifact_dir, artifacts, 'map_preview_overlay_json', artifact_dir / 'mid360_robot_3d_map_preview_overlay.json'), 'artifacts/mid360_robot_3d_map_preview_overlay.json', False),
        _path('profile_snapshot', _session_source(session, 'profile_snapshot_path'), f'recording/{_profile_snapshot_name(session)}', True),
        _path('record_plan_json', _session_source(session, 'record_plan_json_path'), f'recording/{_record_plan_name(session, ".json")}', True),
        _path('record_plan_markdown', _session_source(session, 'record_plan_markdown_path'), f'recording/{_record_plan_name(session, ".md")}', False),
    ]


def _path(key: str, source: Path, destination: str, required: bool) -> BundlePath:
    return BundlePath(key, source, Path(destination), required)


def _source(artifact_dir: Path, artifacts: dict[str, Any], key: str, default: Path) -> Path:
    value = artifacts.get(key)
    if not value:
        return default
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else artifact_dir / path


def _session_source(session: dict[str, Any], key: str) -> Path:
    value = session.get(key) or (session.get('artifact_paths') or {}).get(key.replace('_path', ''))
    return Path(str(value)).expanduser() if value else Path('__missing__') / key


def _profile_snapshot_name(session: dict[str, Any]) -> str:
    return Path(str(session.get('profile_snapshot_path') or 'profile_snapshot.yaml')).name


def _record_plan_name(session: dict[str, Any], suffix: str) -> str:
    key = 'record_plan_json_path' if suffix == '.json' else 'record_plan_markdown_path'
    fallback = 'record_plan' + suffix
    return Path(str(session.get(key) or fallback)).name


def _resolve_output_path(artifact_dir: Path, output_path: Path | None) -> Path:
    if output_path is not None:
        return output_path.expanduser().resolve()
    return artifact_dir.parent / f'{artifact_dir.name}_production_candidate_bundle.tar.gz'


def _bundle_targets(output_path: Path) -> tuple[Path, Path | None]:
    suffixes = output_path.suffixes
    if suffixes[-2:] == ['.tar', '.gz']:
        return output_path.with_suffix('').with_suffix(''), output_path
    if output_path.suffix == '.tgz':
        return output_path.with_suffix(''), output_path
    return output_path, None


def _prepare_bundle_dir(bundle_dir: Path, force: bool) -> None:
    if bundle_dir.exists():
        if not force:
            raise FileExistsError(f'bundle dir already exists: {bundle_dir}')
        shutil.rmtree(bundle_dir)
    bundle_dir.mkdir(parents=True, exist_ok=False)


def _write_tarball(bundle_dir: Path, tarball_path: Path, force: bool) -> None:
    if tarball_path.exists():
        if not force:
            raise FileExistsError(f'tarball already exists: {tarball_path}')
        tarball_path.unlink()
    tarball_path.parent.mkdir(parents=True, exist_ok=True)
    with tarfile.open(tarball_path, 'w:gz') as archive:
        archive.add(bundle_dir, arcname=bundle_dir.name)


def _verify_tarball(tarball_path: Path, root_name: str, manifest: dict[str, Any]) -> bool:
    required = {f'{root_name}/{path}' for path in manifest.get('required_files') or []}
    required.add(f'{root_name}/{BUNDLE_MANIFEST_JSON}')
    try:
        with tarfile.open(tarball_path, 'r:gz') as archive:
            names = set(archive.getnames())
    except Exception:
        return False
    return required.issubset(names)


def _list_bundle_files(bundle_dir: Path) -> list[str]:
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
