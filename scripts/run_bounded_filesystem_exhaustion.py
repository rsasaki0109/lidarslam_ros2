#!/usr/bin/env python3
"""Run the installed map product against a capacity-limited Docker tmpfs."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any
import uuid

import jsonschema
import yaml


SCHEMA_VERSION = 1
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/bounded-filesystem-exhaustion-v1.schema.json'
)
DEFAULT_TMPFS_MIB = 32
DEFAULT_TIMEOUT_SECS = 600
TEXT_EVIDENCE_NAMES = (
    'run_manifest.json',
    'autoware_map_diagnosis.json',
    'autoware_map_diagnosis.md',
    'slam.launch.log',
    'map_save.log',
    'verify_autoware_map.log',
)
STORAGE_HINT = 'output filesystem ran out of writable space or quota'
REPORT_SCHEMA_NAME = 'bounded-filesystem-exhaustion-v1.schema.json'


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_name(f'.{path.name}.tmp')
    temporary.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary, path)


def _positive_int(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a positive integer') from exc
    if parsed <= 0:
        raise argparse.ArgumentTypeError('must be a positive integer')
    return parsed


def _load_json(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _validate_report(repo_root: Path, report: dict[str, Any]) -> None:
    """Validate the report and its embedded public product contracts."""
    schemas = repo_root / 'docs' / 'schemas'
    format_checker = jsonschema.FormatChecker()
    for payload, schema_name in (
        (report, REPORT_SCHEMA_NAME),
        (report['observed']['manifest'], 'run-manifest-v2.schema.json'),
        (report['observed']['diagnosis'], 'diagnosis-v1.schema.json'),
    ):
        schema = json.loads(
            (schemas / schema_name).read_text(encoding='utf-8')
        )
        jsonschema.Draft7Validator.check_schema(schema)
        jsonschema.Draft7Validator(
            schema,
            format_checker=format_checker,
        ).validate(payload)


def _bounded_filesystem_state(path: Path) -> dict[str, int]:
    stats = os.statvfs(path)
    return {
        'capacity_bytes': stats.f_blocks * stats.f_frsize,
        'available_bytes': stats.f_bavail * stats.f_frsize,
        'used_bytes': (stats.f_blocks - stats.f_bfree) * stats.f_frsize,
    }


def _find_run_dir(output_root: Path) -> Path | None:
    final_dir = output_root / 'run'
    partial_dir = output_root / 'run.partial'
    if final_dir.is_dir():
        return final_dir
    if partial_dir.is_dir():
        return partial_dir
    return None


def _text_matches(run_dir: Path | None) -> list[dict[str, Any]]:
    if run_dir is None:
        return []
    matches = []
    needles = (
        'No space left on device',
        'ENOSPC',
        'Disk quota exceeded',
        'raw_fallocate',
        'returned 28',
        'Error during raw_fallocate',
    )
    for name in TEXT_EVIDENCE_NAMES:
        path = run_dir / name
        if not path.is_file() or path.stat().st_size > 16 * 1024 * 1024:
            continue
        text = path.read_text(encoding='utf-8', errors='replace')
        for line_number, line in enumerate(text.splitlines(), start=1):
            if any(needle in line for needle in needles):
                matches.append({
                    'path': name,
                    'line': line_number,
                    'text': line[:1000],
                })
    return matches


def _copy_text_evidence(run_dir: Path | None, evidence_dir: Path) -> list[str]:
    if run_dir is None:
        return []
    captured_dir = evidence_dir / 'captured'
    captured_dir.mkdir(parents=True, exist_ok=True)
    copied = []
    for name in TEXT_EVIDENCE_NAMES:
        source = run_dir / name
        if not source.is_file() or source.stat().st_size > 16 * 1024 * 1024:
            continue
        destination = captured_dir / name
        shutil.copy2(source, destination)
        copied.append(f'captured/{name}')
    return copied


def collect_state(
    output_root: Path,
    evidence_dir: Path,
    product_exit_code: int,
) -> dict[str, Any]:
    """Collect non-geometry state while the stopped product still owns tmpfs."""
    run_dir = _find_run_dir(output_root)
    manifest = _load_json(run_dir / 'run_manifest.json') if run_dir else None
    diagnosis = (
        _load_json(run_dir / 'autoware_map_diagnosis.json')
        if run_dir else None
    )
    state = {
        'product_exit_code': product_exit_code,
        'output_layout': (
            'final' if run_dir == output_root / 'run'
            else ('partial' if run_dir is not None else 'missing')
        ),
        'filesystem': _bounded_filesystem_state(output_root),
        'manifest': manifest,
        'diagnosis': diagnosis,
        'storage_signatures': _text_matches(run_dir),
        'captured_files': _copy_text_evidence(run_dir, evidence_dir),
    }
    _atomic_json(evidence_dir / 'bounded_state.json', state)
    return state


def _bag_identity(bag_path: Path) -> dict[str, Any]:
    metadata_path = bag_path / 'metadata.yaml'
    if not metadata_path.is_file():
        raise ValueError(f'metadata.yaml not found under {bag_path}')
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8')) or {}
    bag_info = metadata.get('rosbag2_bagfile_information', {}) or {}
    storage_files = []
    for relative in bag_info.get('relative_file_paths', []) or []:
        path = (bag_path / str(relative)).resolve()
        try:
            path.relative_to(bag_path.resolve())
        except ValueError as exc:
            raise ValueError(f'storage path escapes bag: {relative}') from exc
        if not path.is_file():
            raise ValueError(f'storage file is missing: {path}')
        storage_files.append({
            'path': str(relative),
            'size_bytes': path.stat().st_size,
            'sha256': _sha256(path),
        })
    if not storage_files:
        raise ValueError('metadata.yaml lists no storage files')
    return {
        'bag_name': bag_path.name,
        'metadata_sha256': _sha256(metadata_path),
        'storage_identifier': str(bag_info.get('storage_identifier', '')),
        'message_count': int(bag_info.get('message_count', 0)),
        'storage_files': storage_files,
    }


def _docker_image_identity(image: str) -> dict[str, Any]:
    result = subprocess.run(
        [
            'docker',
            'image',
            'inspect',
            image,
            '--format',
            '{{json .}}',
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    payload = json.loads(result.stdout)
    labels = payload.get('Config', {}).get('Labels', {}) or {}
    return {
        'reference': image,
        'image_id': str(payload.get('Id', '')),
        'repo_digests': sorted(payload.get('RepoDigests', []) or []),
        'source_revision': str(
            labels.get('org.opencontainers.image.revision', '')
        ),
    }


def _git_identity(repo_root: Path) -> dict[str, Any]:
    commit = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    dirty = bool(subprocess.run(
        ['git', 'status', '--porcelain'],
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout)
    return {'commit': commit, 'dirty': dirty}


def _runtime_payload_identity(repo_root: Path) -> dict[str, Any]:
    files = [
        {
            'path': relative,
            'sha256': _sha256(repo_root / relative),
        }
        for relative in (
            'scripts/diagnose_autoware_map_run.py',
            'scripts/run_autoware_map_from_bag.py',
        )
    ]
    canonical = json.dumps(
        files,
        separators=(',', ':'),
        sort_keys=True,
    ).encode()
    return {
        'files': files,
        'sha256': hashlib.sha256(canonical).hexdigest(),
    }


def evaluate_state(
    state: dict[str, Any],
    tmpfs_bytes: int,
    container_exit_code: int,
    timed_out: bool,
    harness_commit: str = '',
    harness_dirty: bool = False,
    image_revision: str = '',
) -> list[dict[str, Any]]:
    """Evaluate the terminal evidence without accepting a false success."""
    manifest = state.get('manifest') or {}
    diagnosis = state.get('diagnosis') or {}
    filesystem = state.get('filesystem') or {}
    signatures = state.get('storage_signatures') or []
    signature_text = '\n'.join(str(item.get('text', '')) for item in signatures)
    hints = '\n'.join(str(item) for item in diagnosis.get('problem_hints', []))
    checks = [
        {
            'id': 'harness_revision_clean',
            'passed': bool(harness_commit) and not harness_dirty,
            'observed': (
                f'commit={harness_commit}, dirty={harness_dirty}'
            ),
        },
        {
            'id': 'runtime_matches_harness_revision',
            'passed': bool(harness_commit) and image_revision == harness_commit,
            'observed': (
                f'image_revision={image_revision}, '
                f'harness_commit={harness_commit}'
            ),
        },
        {
            'id': 'run_completed_within_timeout',
            'passed': not timed_out,
            'observed': f'timed_out={timed_out}',
        },
        {
            'id': 'product_failed_closed',
            'passed': (
                int(state.get('product_exit_code', 0)) != 0
                and container_exit_code != 0
            ),
            'observed': (
                f'product_exit_code={state.get("product_exit_code")}, '
                f'container_exit_code={container_exit_code}'
            ),
        },
        {
            'id': 'tmpfs_capacity_enforced',
            'passed': filesystem.get('capacity_bytes') == tmpfs_bytes,
            'observed': (
                f'capacity_bytes={filesystem.get("capacity_bytes")}, '
                f'expected={tmpfs_bytes}'
            ),
        },
        {
            'id': 'real_enospc_signature_observed',
            'passed': (
                'raw_fallocate' in signature_text
                and (
                    'returned 28' in signature_text
                    or 'Error during raw_fallocate' in signature_text
                )
            ),
            'observed': f'signature_count={len(signatures)}',
        },
        {
            'id': 'filesystem_nearly_exhausted',
            'passed': (
                int(filesystem.get('available_bytes', tmpfs_bytes))
                <= tmpfs_bytes // 10
            ),
            'observed': (
                f'available_bytes={filesystem.get("available_bytes")}, '
                f'maximum_bytes={tmpfs_bytes // 10}'
            ),
        },
        {
            'id': 'manifest_records_failed_terminal_state',
            'passed': (
                manifest.get('status') == 'failed'
                and manifest.get('lifecycle', {}).get('stage') == 'complete'
                and manifest.get('lifecycle', {}).get('runner_exit_code') != 0
            ),
            'observed': (
                f'status={manifest.get("status")}, '
                f'stage={manifest.get("lifecycle", {}).get("stage")}, '
                'runner_exit_code='
                f'{manifest.get("lifecycle", {}).get("runner_exit_code")}'
            ),
        },
        {
            'id': 'diagnosis_identifies_storage_exhaustion',
            'passed': STORAGE_HINT in hints,
            'observed': f'problem_hints={diagnosis.get("problem_hints", [])}',
        },
        {
            'id': 'success_not_claimed',
            'passed': (
                manifest.get('status') != 'succeeded'
                and diagnosis.get('status') != 'success'
            ),
            'observed': (
                f'manifest_status={manifest.get("status")}, '
                f'diagnosis_status={diagnosis.get("status")}'
            ),
        },
    ]
    return checks


def _run_container(
    image: str,
    bag_path: Path,
    evidence_dir: Path,
    tmpfs_mib: int,
    timeout_secs: int,
    script_path: Path,
) -> tuple[int, bool, float]:
    container_name = f'lidarslam-enospc-{uuid.uuid4().hex[:12]}'
    container_script = (
        'set +e\n'
        'lidarslam-map run /input '
        '--output-dir /bounded-output/run '
        '--min-free-space-gib 0.001\n'
        'product_rc=$?\n'
        'python3 /harness.py collect '
        '--output-root /bounded-output '
        '--evidence-dir /evidence '
        '--product-exit-code \"$product_rc\"\n'
        'collect_rc=$?\n'
        'if [ \"$collect_rc\" -ne 0 ]; then exit 70; fi\n'
        'exit \"$product_rc\"\n'
    )
    command = [
        'docker',
        'run',
        '--name',
        container_name,
        '--shm-size',
        '1g',
        '--tmpfs',
        f'/bounded-output:rw,size={tmpfs_mib}m,mode=1777',
        '--mount',
        f'type=bind,src={bag_path},dst=/input,readonly',
        '--mount',
        f'type=bind,src={evidence_dir},dst=/evidence',
        '--mount',
        f'type=bind,src={script_path},dst=/harness.py,readonly',
        image,
        'bash',
        '-lc',
        container_script,
    ]
    log_path = evidence_dir / 'container.log'
    started = time.monotonic()
    timed_out = False
    with log_path.open('w', encoding='utf-8') as log:
        process = subprocess.Popen(
            command,
            stdout=log,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            exit_code = process.wait(timeout=timeout_secs)
        except subprocess.TimeoutExpired:
            timed_out = True
            subprocess.run(
                ['docker', 'stop', '--time', '20', container_name],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            try:
                exit_code = process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                subprocess.run(
                    ['docker', 'kill', container_name],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                exit_code = process.wait(timeout=30)
    subprocess.run(
        ['docker', 'rm', '-f', container_name],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return exit_code, timed_out, time.monotonic() - started


def run(args: argparse.Namespace) -> int:
    repo_root = Path(__file__).resolve().parent.parent
    script_path = Path(__file__).resolve()
    bag_path = Path(args.bag).expanduser().resolve()
    evidence_dir = Path(args.evidence_dir).expanduser().resolve()
    if evidence_dir.exists() and any(evidence_dir.iterdir()):
        raise ValueError(f'evidence directory is not empty: {evidence_dir}')
    evidence_dir.mkdir(parents=True, exist_ok=True)

    started_at = _utc_now()
    image_identity = _docker_image_identity(args.image)
    input_identity = _bag_identity(bag_path)
    harness_identity = {
        **_git_identity(repo_root),
        'script_sha256': _sha256(script_path),
        'runtime_payload': _runtime_payload_identity(repo_root),
    }
    exit_code, timed_out, duration_sec = _run_container(
        image=args.image,
        bag_path=bag_path,
        evidence_dir=evidence_dir,
        tmpfs_mib=args.tmpfs_mib,
        timeout_secs=args.timeout_secs,
        script_path=script_path,
    )
    state = _load_json(evidence_dir / 'bounded_state.json') or {}
    tmpfs_bytes = args.tmpfs_mib * 1024 * 1024
    checks = evaluate_state(
        state,
        tmpfs_bytes,
        exit_code,
        timed_out,
        harness_commit=harness_identity['commit'],
        harness_dirty=harness_identity['dirty'],
        image_revision=image_identity['source_revision'],
    )
    report = {
        'schema_version': SCHEMA_VERSION,
        'schema_uri': SCHEMA_URI,
        'status': 'passed' if all(item['passed'] for item in checks) else 'failed',
        'started_at': started_at,
        'finished_at': _utc_now(),
        'duration_sec': duration_sec,
        'hardware_label': args.hardware_label,
        'image': image_identity,
        'harness': harness_identity,
        'input': input_identity,
        'bounded_filesystem': {
            'type': 'docker_tmpfs',
            'limit_bytes': tmpfs_bytes,
            'mount_path': '/bounded-output',
        },
        'execution': {
            'container_exit_code': exit_code,
            'timed_out': timed_out,
            'timeout_secs': args.timeout_secs,
            'container_log_sha256': _sha256(evidence_dir / 'container.log'),
        },
        'observed': state,
        'checks': checks,
    }
    _validate_report(repo_root, report)
    _atomic_json(evidence_dir / 'bounded_filesystem_exhaustion_report.json', report)
    print(json.dumps({
        'status': report['status'],
        'report': str(
            evidence_dir / 'bounded_filesystem_exhaustion_report.json'
        ),
        'checks': checks,
    }, indent=2))
    return 0 if report['status'] == 'passed' else 1


def parse_args() -> argparse.Namespace:
    if len(sys.argv) > 1 and sys.argv[1] == 'collect':
        collect_parser = argparse.ArgumentParser(add_help=False)
        collect_parser.set_defaults(command='collect')
        collect_parser.add_argument('command')
        collect_parser.add_argument('--output-root', required=True)
        collect_parser.add_argument('--evidence-dir', required=True)
        collect_parser.add_argument(
            '--product-exit-code',
            type=int,
            required=True,
        )
        return collect_parser.parse_args()

    parser = argparse.ArgumentParser(
        description=(
            'Run the installed lidarslam-map product on a real bag while '
            'confining all output to a capacity-limited Docker tmpfs.'
        ),
    )
    parser.set_defaults(command='run')
    parser.add_argument('bag', help='rosbag2 directory to mount read-only.')
    parser.add_argument(
        '--evidence-dir',
        required=True,
        help='Empty host directory that receives non-geometry evidence.',
    )
    parser.add_argument(
        '--image',
        required=True,
        help='Locally available image built from the exact clean harness commit.',
    )
    parser.add_argument(
        '--tmpfs-mib',
        type=_positive_int,
        default=DEFAULT_TMPFS_MIB,
    )
    parser.add_argument(
        '--timeout-secs',
        type=_positive_int,
        default=DEFAULT_TIMEOUT_SECS,
    )
    parser.add_argument(
        '--hardware-label',
        required=False,
        default='unnamed-local-machine',
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.command == 'collect':
            collect_state(
                Path(args.output_root),
                Path(args.evidence_dir),
                args.product_exit_code,
            )
            return 0
        return run(args)
    except (
        FileNotFoundError,
        OSError,
        subprocess.CalledProcessError,
        ValueError,
        jsonschema.SchemaError,
        jsonschema.ValidationError,
        yaml.YAMLError,
    ) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
