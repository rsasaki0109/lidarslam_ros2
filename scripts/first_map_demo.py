#!/usr/bin/env python3
"""Run the fixed public first-map demo through the stable product CLI."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
from typing import Any, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
CANONICAL_DEMO = SCRIPT_DIR / 'run_first_map_demo.sh'
DEMO_PLAN_SCHEMA = 'first-map-demo-plan-v1.schema.json'
DEMO_PLAN_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/first-map-demo-plan-v1.schema.json'
)
DATASET_ID = 'driving_slam_mid360'
DATASET_TITLE = 'Driving SLAM Test with Livox MID360'
DATASET_SOURCE = 'https://zenodo.org/records/14841855'
DATASET_DOI = '10.5281/zenodo.14841855'
DATASET_LICENSE = 'Creative Commons Attribution 4.0 International'
ARCHIVE_SIZE_BYTES = 517088133
ARCHIVE_NAME = 'rosbag2_2024_04_16-14_17_01.zip'
ARCHIVE_SHA256 = (
    'f8f89eebf2aaf9cc1d465bfa5451bbb5'
    '99cd92d079b59949104bb4e5cb619bdd'
)
BAG_NAME = 'rosbag2_2024_04_16-14_17_01'
METADATA_SIZE_BYTES = 5590
STORAGE_NAME = 'rosbag2_2024_04_16-14_17_01_0.db3'
STORAGE_SIZE_BYTES = 1468932096
DEFAULT_MIN_FREE_SPACE_GIB = 8.0
GIB = 1024 ** 3
MAX_EVIDENCE_BYTES = 2 * 1024 * 1024
EXPECTED_PROFILE = 'rko_lio_graph_mid360_preset'
RESUMABLE_STAGES = frozenset({
    'workflow_finished',
    'verifying',
    'verified',
    'finalizing',
    'finalized',
    'diagnosing',
    'diagnosed',
    'checksumming',
})


def _load_script_module(script_name: str, module_name: str):
    path = SCRIPT_DIR / script_name
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'failed to load {module_name} from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _positive_float(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a number') from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError(
            'must be finite and greater than zero'
        )
    return parsed


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the one-command public demo options."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Download the fixed public MID-360 bag, build a verified map, '
            'and optionally open the offline review.'
        ),
    )
    parser.add_argument(
        'work_root',
        nargs='?',
        metavar='work_dir',
        default='.',
        help='Root for default dataset and output directories (default: .).',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all options (this command has no hidden options).',
    )
    parser.add_argument(
        '--data-dir',
        metavar='<dir>',
        help='Dataset cache (default: <work_dir>/datasets/mid360_public).',
    )
    parser.add_argument(
        '--output-dir',
        metavar='<dir>',
        help='Map output (default: <work_dir>/output/mid360_demo).',
    )
    parser.add_argument(
        '--viewer',
        choices=('browser', 'none'),
        default='browser',
        metavar='{browser,none}',
        help=(
            'Open the offline map review after verification '
            '(default: browser).'
        ),
    )
    parser.add_argument(
        '--min-free-space-gib',
        type=_positive_float,
        default=DEFAULT_MIN_FREE_SPACE_GIB,
        metavar='<GiB>',
        help='Required free space before download and mapping (default: 8).',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Inspect the exact plan without downloading or writing output.',
    )
    parser.add_argument(
        '--resume',
        action='store_true',
        help=(
            'Resume only safe terminal map post-processing retained by a '
            'previous demo run.'
        ),
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='With --dry-run, print the schema-valid plan as JSON.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        metavar='<plan-file>',
        help=(
            'With --dry-run, write the human or JSON plan once; refuse an '
            'existing path.'
        ),
    )
    return parser.parse_args(argv)


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _absolute(path: Path) -> Path:
    expanded = path.expanduser()
    if not expanded.is_absolute():
        expanded = Path.cwd() / expanded
    return expanded.resolve(strict=False)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _read_bounded_object(path: Path) -> dict[str, Any] | None:
    if path.is_symlink() or not path.is_file():
        return None
    try:
        if path.stat().st_size > MAX_EVIDENCE_BYTES:
            return None
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _verified_output(output_dir: Path) -> tuple[bool, str | None]:
    """Accept only receipt-bound, schema-valid, complete demo output."""
    required_paths = (
        output_dir / 'run_manifest.json',
        output_dir / 'first_map_validation_receipt.json',
        output_dir / 'autoware_map_diagnosis.json',
        output_dir / 'verify_autoware_map.log',
    )
    if any(
        path.is_symlink()
        or not path.is_file()
        or path.stat().st_size > MAX_EVIDENCE_BYTES
        for path in required_paths
    ):
        return False, None
    manifest = _read_bounded_object(required_paths[0])
    receipt = _read_bounded_object(required_paths[1])
    if manifest is None or receipt is None:
        return False, None
    try:
        product_schema = _load_script_module(
            'product_schema.py',
            'first_map_demo_product_schema',
        )
        product_schema.validate_contract(
            manifest,
            'run-manifest-v2.schema.json',
        )
        product_schema.validate_contract(
            receipt,
            'first-map-validation-receipt-v1.schema.json',
        )
        receipt_builder = _load_script_module(
            'first_map_validation_receipt.py',
            'first_map_demo_receipt_builder',
        )
        rebuilt = receipt_builder.build_receipt(output_dir)
    except (ImportError, OSError, RuntimeError, ValueError):
        return False, None
    if receipt != rebuilt:
        return False, None
    if not (
        manifest['status'] == 'succeeded'
        and manifest['lifecycle']['stage'] == 'complete'
        and manifest['lifecycle']['runner_exit_code'] == 0
        and manifest['output']['finalized'] is True
        and manifest['profile']['id'] == EXPECTED_PROFILE
        and receipt['status'] == 'PASS'
        and all(item['passed'] for item in receipt['checks'])
    ):
        return False, None
    return True, _sha256(required_paths[1])


def _empty_output_state(status: str) -> dict[str, Any]:
    return {
        'status': status,
        'receipt_sha256': None,
        'lifecycle_stage': None,
        'resume_available': False,
        'run_dir': None,
    }


def _resume_projection(
    run_dir: Path,
    output_dir: Path,
    partial: Path,
) -> dict[str, Any]:
    projection = _empty_output_state('partial')
    projection['run_dir'] = str(run_dir)
    manifest = _read_bounded_object(run_dir / 'run_manifest.json')
    if manifest is None:
        return projection
    lifecycle = manifest.get('lifecycle')
    execution = manifest.get('execution')
    output = manifest.get('output')
    profile = manifest.get('profile')
    if not all(
        isinstance(item, dict)
        for item in (lifecycle, execution, output, profile)
    ):
        return projection
    stage = lifecycle.get('stage')
    projection['lifecycle_stage'] = (
        stage if isinstance(stage, str) else None
    )
    try:
        product_schema = _load_script_module(
            'product_schema.py',
            'first_map_demo_resume_schema',
        )
        product_schema.validate_contract(
            manifest,
            'run-manifest-v2.schema.json',
        )
    except (ImportError, OSError, RuntimeError, ValueError):
        return projection
    safe = (
        stage in RESUMABLE_STAGES
        and execution.get('finished_at') is not None
        and isinstance(execution.get('exit_code'), int)
        and not isinstance(execution.get('exit_code'), bool)
        and lifecycle.get('verification_enabled') is True
        and profile.get('id') == EXPECTED_PROFILE
        and output.get('requested_dir') == str(output_dir)
        and output.get('working_dir') == str(partial)
        and not (run_dir == partial and output.get('finalized') is True)
    )
    if safe:
        projection['status'] = 'resumable'
        projection['resume_available'] = True
    return projection


def _output_state(output_dir: Path) -> dict[str, Any]:
    partial = output_dir.with_name(f'{output_dir.name}.partial')
    if output_dir.exists() and partial.exists():
        return _empty_output_state('conflict')
    if partial.exists():
        if partial.is_symlink() or not partial.is_dir():
            return _empty_output_state('partial')
        return _resume_projection(partial, output_dir, partial)
    if not output_dir.exists():
        return _empty_output_state('absent')
    if output_dir.is_symlink() or not output_dir.is_dir():
        return _empty_output_state('conflict')
    verified, receipt_sha256 = _verified_output(output_dir)
    if verified:
        return {
            'status': 'verified',
            'receipt_sha256': receipt_sha256,
            'lifecycle_stage': 'complete',
            'resume_available': False,
            'run_dir': str(output_dir),
        }
    projection = _resume_projection(output_dir, output_dir, partial)
    if projection['resume_available']:
        return projection
    projection['status'] = 'conflict'
    return projection


def _regular_file_with_size(path: Path, size: int) -> bool:
    try:
        return (
            not path.is_symlink()
            and path.is_file()
            and path.stat().st_size == size
        )
    except OSError:
        return False


def _cache_projection(data_dir: Path) -> dict[str, Any]:
    """Describe fixed-path cache presence without expensive hashing."""
    dataset_dir = data_dir / DATASET_ID
    archive = dataset_dir / 'archives' / ARCHIVE_NAME
    bag = dataset_dir / 'extracted' / BAG_NAME / BAG_NAME
    archive_present = _regular_file_with_size(
        archive,
        ARCHIVE_SIZE_BYTES,
    )
    bag_present = (
        _regular_file_with_size(
            bag / 'metadata.yaml',
            METADATA_SIZE_BYTES,
        )
        and _regular_file_with_size(
            bag / STORAGE_NAME,
            STORAGE_SIZE_BYTES,
        )
    )
    if archive_present and bag_present:
        status = 'prepared_unverified'
    elif archive_present:
        status = 'archive_unverified'
    else:
        status = 'download_required'
    return {
        'status': status,
        'bag_path': str(bag.resolve()) if bag_present else None,
        'download_required': not archive_present,
    }


def _contains(parent: Path, child: Path) -> bool:
    try:
        child.relative_to(parent)
    except ValueError:
        return False
    return True


def _nearest_existing(path: Path) -> Path:
    probe = path
    while not probe.exists():
        if probe.parent == probe:
            break
        probe = probe.parent
    if not probe.exists():
        raise OSError(f'no existing filesystem ancestor for {path}')
    if probe.is_file():
        probe = probe.parent
    return probe.resolve()


def _storage_projection(
    paths: Sequence[Path],
    minimum_free_gib: float,
    retry_command: str,
) -> tuple[dict[str, Any], list[dict[str, str]]]:
    required = math.ceil(minimum_free_gib * GIB)
    volumes = []
    findings = []
    devices: set[int] = set()
    for path in paths:
        try:
            probe = _nearest_existing(path)
            device = probe.stat().st_dev
            if device in devices:
                continue
            devices.add(device)
            free = shutil.disk_usage(probe).free
        except OSError as exc:
            findings.append({
                'code': 'storage-unavailable',
                'message': f'Cannot inspect storage for {path}: {exc}',
                'next_action': (
                    'Restore access to the selected filesystem, then run: '
                    f'{retry_command}'
                ),
            })
            continue
        enough = free >= required
        additional_bytes_required = max(required - free, 0)
        volumes.append({
            'probe_path': str(probe),
            'free_bytes': free,
            'additional_bytes_required': additional_bytes_required,
            'enough': enough,
        })
        if not enough:
            shortfall = _gib_shortfall(additional_bytes_required)
            findings.append({
                'code': 'insufficient-free-space',
                'message': (
                    f'{probe} has {free / 1024**3:.2f} GiB free; '
                    f'{minimum_free_gib:.2f} GiB is required, so at least '
                    f'{shortfall} more is needed.'
                ),
                'next_action': (
                    f'Free at least {shortfall} on {probe}, then run: '
                    f'{retry_command}'
                ),
            })
    if not volumes:
        volumes.append({
            'probe_path': str(Path('/')),
            'free_bytes': 0,
            'additional_bytes_required': required,
            'enough': False,
        })
    return {
        'minimum_free_gib': minimum_free_gib,
        'minimum_free_bytes': required,
        'volumes': volumes,
    }, findings


def _gib_shortfall(byte_count: int) -> str:
    """Format a positive byte shortfall without understating it."""
    hundredths = (byte_count * 100 + GIB - 1) // GIB
    return f'{hundredths / 100:.2f} GiB'


def _cli_prefix() -> list[str]:
    configured = os.environ.get('LIDARSLAM_CLI_COMMAND')
    if configured:
        try:
            parsed = shlex.split(configured)
        except ValueError:
            parsed = []
        if parsed:
            return parsed
    return ['lidarslam-map', 'demo']


def _finding(
    code: str,
    message: str,
    next_action: str,
) -> dict[str, str]:
    return {
        'code': code,
        'message': message,
        'next_action': next_action,
    }


def _demo_argv(
    args: argparse.Namespace,
    work_root: Path,
    data_dir: Path,
    output_dir: Path,
    *,
    resume: bool | None = None,
) -> list[str]:
    argv = [
        *_cli_prefix(),
        str(work_root),
        '--data-dir',
        str(data_dir),
        '--output-dir',
        str(output_dir),
        '--viewer',
        args.viewer,
        '--min-free-space-gib',
        f'{args.min_free_space_gib:g}',
    ]
    resume_requested = args.resume if resume is None else resume
    if resume_requested:
        argv.append('--resume')
    return argv


def build_demo_plan(args: argparse.Namespace) -> dict[str, Any]:
    """Build a schema-valid read-only execution plan."""
    work_requested = Path(args.work_root).expanduser()
    data_requested = (
        Path(args.data_dir).expanduser()
        if args.data_dir else
        work_requested / 'datasets' / 'mid360_public'
    )
    output_requested = (
        Path(args.output_dir).expanduser()
        if args.output_dir else
        work_requested / 'output' / 'mid360_demo'
    )
    work_root = _absolute(work_requested)
    data_dir = _absolute(data_requested)
    output_dir = _absolute(output_requested)
    partial = output_dir.with_name(f'{output_dir.name}.partial')
    findings: list[dict[str, str]] = []

    for label, requested, resolved in (
        ('work directory', work_requested, work_root),
        ('dataset cache', data_requested, data_dir),
        ('map output', output_requested, output_dir),
        ('partial map output', partial, partial),
    ):
        absolute_requested = (
            requested if requested.is_absolute() else Path.cwd() / requested
        )
        if absolute_requested.is_symlink():
            findings.append(_finding(
                'path-symlink',
                f'The {label} may not be a symlink: {absolute_requested}',
                'Choose a regular directory path.',
            ))
        if resolved.exists() and not resolved.is_dir():
            findings.append(_finding(
                'path-not-directory',
                f'The {label} is not a directory: {resolved}',
                'Choose a directory path.',
            ))

    if _contains(data_dir, output_dir) or _contains(output_dir, data_dir):
        findings.append(_finding(
            'path-overlap',
            'The dataset cache and map output may not contain one another.',
            'Choose separate dataset and output directories.',
        ))

    output = _output_state(output_dir)
    resume_command = shlex.join(_demo_argv(
        args,
        work_root,
        data_dir,
        output_dir,
        resume=True,
    ))
    if args.resume:
        if output['status'] == 'verified':
            findings.append(_finding(
                'resume-not-needed',
                f'The demo output is already verified: {output_dir}',
                f'Open it with lidarslam-map view {output_dir}.',
            ))
        elif output['status'] == 'absent':
            findings.append(_finding(
                'resume-unavailable',
                'No retained demo output is available to resume.',
                'Run the same command without --resume.',
            ))
        elif output['status'] != 'resumable':
            findings.append(_finding(
                'resume-unsafe',
                'The retained output is not at a safe terminal '
                'post-processing stage.',
                'Inspect it, then retry with a fresh --output-dir.',
            ))
    elif output['status'] == 'resumable':
        findings.append(_finding(
            'resume-available',
            'Mapping finished and safe terminal post-processing can resume '
            f'from stage {output["lifecycle_stage"]}.',
            resume_command,
        ))
    elif output['status'] == 'partial':
        findings.append(_finding(
            'partial-output',
            f'An unsafe or incomplete demo output exists: {partial}',
            'Inspect it, then choose a fresh --output-dir.',
        ))
    elif output['status'] == 'conflict':
        findings.append(_finding(
            'output-conflict',
            f'The output exists but is not a verified demo: {output_dir}',
            'Inspect it, then choose a fresh --output-dir.',
        ))

    argv = _demo_argv(args, work_root, data_dir, output_dir)
    storage, storage_findings = _storage_projection(
        (data_dir, output_dir),
        args.min_free_space_gib,
        shlex.join(argv),
    )
    findings.extend(storage_findings)
    cache = _cache_projection(data_dir)
    if args.resume and output['status'] == 'resumable' and not findings:
        status = 'resume_ready'
    elif output['status'] == 'verified' and not findings:
        status = 'already_verified'
    elif findings:
        status = 'not_ready'
    else:
        status = 'ready'

    if status == 'already_verified':
        mode = 'reuse_verified'
        steps = ['reuse_verified_map']
    else:
        if cache['status'] == 'prepared_unverified':
            data_step = 'verify_prepared_dataset_cache'
        elif cache['status'] == 'archive_unverified':
            data_step = 'verify_archive_and_prepare_dataset'
        else:
            data_step = 'download_and_verify_dataset'
        if args.resume:
            mode = 'resume_postprocessing'
            map_step = 'resume_and_verify_map'
        else:
            mode = 'fresh'
            map_step = 'build_and_verify_map'
        steps = [data_step, map_step]
    if args.viewer == 'browser' and status != 'not_ready':
        steps.append('open_offline_review')

    plan = {
        'schema_version': 1,
        'schema_uri': DEMO_PLAN_SCHEMA_URI,
        'created_at': _utc_now(),
        'status': status,
        'ready': status != 'not_ready',
        'mode': mode,
        'dataset': {
            'id': DATASET_ID,
            'title': DATASET_TITLE,
            'source_url': DATASET_SOURCE,
            'doi': DATASET_DOI,
            'license': DATASET_LICENSE,
            'archive_size_bytes': ARCHIVE_SIZE_BYTES,
            'archive_sha256': ARCHIVE_SHA256,
        },
        'paths': {
            'work_root': str(work_root),
            'data_dir': str(data_dir),
            'output_dir': str(output_dir),
        },
        'cache': cache,
        'output': output,
        'storage': storage,
        'viewer': args.viewer,
        'steps': steps,
        'command': {
            'argv': argv,
            'shell': shlex.join(argv),
        },
        'findings': findings,
    }
    product_schema = _load_script_module(
        'product_schema.py',
        'first_map_demo_plan_schema',
    )
    product_schema.validate_contract(plan, DEMO_PLAN_SCHEMA)
    return plan


def _render_plan(plan: dict[str, Any]) -> str:
    cache = plan['cache']
    output = plan['output']
    storage = plan['storage']
    lines = [
        'Public first-map demo plan',
        f"  Status: {plan['status']}",
        f"  Mode: {plan['mode']}",
        f"  Dataset: {plan['dataset']['title']}",
        f"  License: {plan['dataset']['license']}",
        f"  Cache: {cache['status']}",
        f"  Output: {output['status']} — {plan['paths']['output_dir']}",
        f"  Viewer: {plan['viewer']}",
        f"  Required free space: {storage['minimum_free_gib']:g} GiB",
        '  Steps: ' + ' -> '.join(plan['steps']),
    ]
    if output['lifecycle_stage'] is not None:
        lines.append(
            f"  Durable lifecycle stage: {output['lifecycle_stage']}"
        )
    if plan['findings']:
        lines.append('Action required:')
        for finding in plan['findings']:
            lines.append(
                f"  [{finding['code']}] {finding['message']} "
                f"Next: {finding['next_action']}"
            )
    else:
        lines.append(f"  Command: {plan['command']['shell']}")
    return '\n'.join(lines)


def _write_dry_run_plan(
    plan: dict[str, Any],
    *,
    json_output: bool,
    output: Path | None,
) -> bool:
    """Write one read-only plan, refusing to overwrite an existing file."""
    payload = (
        json.dumps(plan, indent=2, sort_keys=True) + '\n'
        if json_output else
        _render_plan(plan) + '\n'
    )
    if output is None:
        sys.stdout.write(payload)
        return True
    try:
        with output.open('x', encoding='utf-8') as stream:
            stream.write(payload)
    except OSError as exc:
        print(
            f'error: [demo-plan-output-failed] {exc}',
            file=sys.stderr,
        )
        return False
    print(f'Wrote read-only demo plan: {output}', file=sys.stderr)
    print(
        'The plan performs no download, mapping, or publication action.',
        file=sys.stderr,
    )
    return True


def _open_review(output_dir: Path) -> None:
    executable = shutil.which('lidarslam-map')
    if executable is None:
        print(
            'warning: [demo-viewer-unavailable] lidarslam-map is not in PATH',
            file=sys.stderr,
        )
        return
    completed = subprocess.run(
        [executable, 'view', str(output_dir), '--viewer', 'browser'],
        check=False,
    )
    if completed.returncode != 0:
        print(
            'warning: [demo-viewer-failed] the verified map is retained; '
            f'viewer exit code {completed.returncode}',
            file=sys.stderr,
        )


def _render_success(
    output_dir: Path,
    receipt_sha256: str,
    *,
    reused: bool,
) -> str:
    return '\n'.join([
        '',
        (
            'First map verified.'
            if not reused else
            'Verified demo output reused.'
        ),
        f'  Map output: {output_dir}',
        '  Verifier: PASS',
        f'  Receipt: {output_dir / "first_map_validation_receipt.json"}',
        f'  Receipt SHA-256: {receipt_sha256}',
        f'  Review again: lidarslam-map view {shlex.quote(str(output_dir))}',
        f'  Diagnose: lidarslam-map inspect {shlex.quote(str(output_dir))}',
    ])


def _next_retry_output(output_dir: Path) -> Path:
    for number in range(1, 1000):
        candidate = output_dir.with_name(
            f'{output_dir.name}-retry-{number}'
        )
        partial = candidate.with_name(f'{candidate.name}.partial')
        if not candidate.exists() and not partial.exists():
            return candidate
    raise RuntimeError(
        f'no unused retry output name is available beside {output_dir}'
    )


def _replace_output_argument(argv: list[str], output_dir: Path) -> list[str]:
    updated = [item for item in argv if item != '--resume']
    try:
        index = updated.index('--output-dir') + 1
    except ValueError as exc:
        raise RuntimeError('demo command has no --output-dir') from exc
    updated[index] = str(output_dir)
    return updated


def _render_failure(
    plan: dict[str, Any],
    returncode: int,
) -> str:
    output_dir = Path(plan['paths']['output_dir'])
    partial = output_dir.with_name(f'{output_dir.name}.partial')
    state = _output_state(output_dir)
    run_dir = state['run_dir']
    if run_dir is None:
        if partial.exists():
            run_dir = str(partial)
        elif output_dir.exists():
            run_dir = str(output_dir)
    stage = state['lifecycle_stage'] or 'no durable stage recorded'
    lines = [
        '',
        f'Demo needs attention (exit {returncode}).',
        f'  Last durable stage: {stage}',
    ]
    command = list(plan['command']['argv'])
    if state['resume_available']:
        if '--resume' not in command:
            command.append('--resume')
        lines.extend([
            '  Mapping is complete; only terminal post-processing will '
            'resume.',
            f'  Next: {shlex.join(command)}',
        ])
    else:
        if run_dir is not None:
            lines.append(
                '  Inspect: '
                f'lidarslam-map inspect {shlex.quote(run_dir)} --write'
            )
        try:
            retry_output = (
                output_dir
                if state['status'] == 'absent'
                else _next_retry_output(output_dir)
            )
            retry_command = _replace_output_argument(
                command,
                retry_output,
            )
            lines.append(
                '  Fresh retry: ' + shlex.join(retry_command)
            )
        except RuntimeError as exc:
            lines.append(f'  Recovery planning failed: {exc}')
    lines.append('  Retained evidence was not deleted or overwritten.')
    return '\n'.join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    """Plan or run the canonical public first-map lifecycle."""
    args = parse_args(argv)
    if args.output is not None and not args.dry_run:
        print(
            'error: [demo-output-requires-dry-run] --output requires '
            '--dry-run',
            file=sys.stderr,
        )
        return 2
    if args.json and not args.dry_run:
        print(
            'error: [demo-json-requires-dry-run] --json requires --dry-run',
            file=sys.stderr,
        )
        return 2
    try:
        plan = build_demo_plan(args)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'error: [demo-plan-invalid] {exc}', file=sys.stderr)
        return 2

    if args.dry_run:
        if not _write_dry_run_plan(
            plan,
            json_output=args.json,
            output=args.output,
        ):
            return 2
        return 0 if plan['ready'] else 2

    if not plan['ready']:
        print(_render_plan(plan), file=sys.stderr)
        return 2

    output_dir = Path(plan['paths']['output_dir'])
    if plan['status'] == 'already_verified':
        print(_render_success(
            output_dir,
            plan['output']['receipt_sha256'],
            reused=True,
        ))
        if args.viewer == 'browser':
            _open_review(output_dir)
        return 0

    if not CANONICAL_DEMO.is_file():
        print(
            f'error: [demo-helper-missing] {CANONICAL_DEMO}',
            file=sys.stderr,
        )
        return 70
    print(_render_plan(plan))
    print(
        'Progress reports completed durable stages only; no percentage or '
        'ETA is estimated.'
    )
    child_env = os.environ.copy()
    child_env['DEMO_DATA_DIR'] = plan['paths']['data_dir']
    child_env['DEMO_OUTPUT_DIR'] = plan['paths']['output_dir']
    child_env['DEMO_RESUME'] = (
        '1' if plan['mode'] == 'resume_postprocessing' else '0'
    )
    try:
        completed = subprocess.run(
            ['/bin/bash', str(CANONICAL_DEMO)],
            check=False,
            env=child_env,
        )
    except OSError as exc:
        print(f'error: [demo-start-failed] {exc}', file=sys.stderr)
        return 70
    if completed.returncode != 0:
        print(
            _render_failure(plan, completed.returncode),
            file=sys.stderr,
        )
        return completed.returncode

    verified, receipt_sha256 = _verified_output(output_dir)
    if not verified or receipt_sha256 is None:
        print(
            'error: [demo-evidence-invalid] the demo returned success without '
            'a complete receipt-bound verified output',
            file=sys.stderr,
        )
        print(_render_failure(plan, 1), file=sys.stderr)
        return 1
    print(_render_success(
        output_dir,
        receipt_sha256,
        reused=False,
    ))
    if args.viewer == 'browser':
        _open_review(output_dir)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
