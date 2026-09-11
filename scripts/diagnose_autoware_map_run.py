#!/usr/bin/env python3
"""Diagnose a map-authoring run directory."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import sys
from typing import Any

import yaml

SCHEMA_VERSION = 1
SCHEMA_URI = 'https://rsasaki0109.github.io/lidar_slam_ros2/schemas/diagnosis-v1.schema.json'

SYMPTOM_CHOICES = (
    'map-spins-or-spirals',
    'pose-drifts-or-oscillates',
    'map-stops-early',
    'map-is-too-sparse',
    'map-is-not-visible',
)

SYMPTOM_GUIDANCE = {
    'map-spins-or-spirals': {
        'title': 'Map rotates, spirals, or scatters around the origin',
        'checks': (
            'Recheck the recorded cloud topic, frame, fields, and timestamps '
            'with doctor before changing registration or loop-closure tuning.',
            'Verify the measured LiDAR-to-base transform, axis convention, '
            'units, and quaternion against the physical rig.',
            'Verify point-time units, scan period, and deskew inputs before '
            'treating the result as an algorithm-quality problem.',
        ),
        'avoid': (
            'Do not use an identity transform unless it describes the measured '
            'physical rig.',
            'Do not tune loop-closure thresholds to compensate for a broken '
            'frame, extrinsic, or timestamp contract.',
        ),
    },
    'pose-drifts-or-oscillates': {
        'title': 'Pose drifts, jumps, or oscillates',
        'checks': (
            'Recheck frame connectivity, physical extrinsics, point-time '
            'units, scan period, and deskew inputs first.',
            'If odometry is enabled, verify that the Odometry message frames '
            'also exist in TF; a frame_id string does not publish a transform.',
            'Preserve the exact run and inspect whether the environment is '
            'geometrically degenerate before selecting an opt-in preset.',
        ),
        'avoid': (
            'Do not change graph weights or registration thresholds before '
            'the sensor, time, and TF contracts pass.',
            'Do not present a visual improvement as an accuracy result without '
            'an external reference trajectory.',
        ),
    },
    'map-stops-early': {
        'title': 'The saved map stops before the bag or route ends',
        'checks': (
            'Check session status and the retained launch, map-save, and '
            'verification logs for an early node exit or save failure.',
            'Confirm whether the raw map ended early or only the modified '
            'loop-closure output is incomplete.',
            'Keep the full run evidence before comparing a fresh output; never '
            'overwrite the failed or partial session.',
        ),
        'avoid': (
            'Do not increase loop-search ranges until runtime completion and '
            'the input trajectory have been verified.',
            'Do not treat file presence alone as a complete verified map.',
        ),
    },
    'map-is-too-sparse': {
        'title': 'The completed map is unexpectedly sparse',
        'checks': (
            'Confirm that map saving and verification completed and that the '
            'retained map metadata describes the expected output.',
            'Review recorded range limits, input/map voxel sizes, and active '
            'filters one at a time against the same retained session.',
            'Distinguish a sparse saved map from an incomplete run or a viewer '
            'display setting before changing map density.',
        ),
        'avoid': (
            'Do not lower every voxel size at once; that hides the responsible '
            'setting and can exhaust memory or storage.',
            'Do not infer geometric accuracy from visual density.',
        ),
    },
    'map-is-not-visible': {
        'title': 'The map is not visible in the viewer',
        'checks': (
            'First separate a viewer problem from a mapping problem by checking '
            'whether the retained session and verification result are PASS.',
            'Open the offline session view and confirm the selected fixed frame '
            'and map artifact before changing the launch file.',
            'If the map artifacts are absent, follow the retained runtime '
            'finding instead of changing RViz topics by trial and error.',
        ),
        'avoid': (
            'Do not edit tracked launch or YAML files merely to make a topic '
            'appear in a viewer.',
            'Do not call a verified map missing solely because one viewer failed '
            'to open it.',
        ),
    },
}


def _load_preflight_module():
    import importlib.util

    script_path = Path(__file__).resolve().parent / 'preflight_autoware_map_bag.py'
    spec = importlib.util.spec_from_file_location('preflight_autoware_map_bag', script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'failed to load preflight module from {script_path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _read_text(path: Path | None) -> str:
    if path is None or not path.is_file():
        return ''
    return path.read_text(encoding='utf-8', errors='replace')


def _find_first(run_dir: Path, candidates: list[str]) -> Path | None:
    for candidate in candidates:
        path = run_dir / candidate
        if path.is_file():
            return path
    return None


def _read_run_manifest(run_dir: Path) -> tuple[dict[str, Any] | None, list[str]]:
    manifest_path = run_dir / 'run_manifest.json'
    if not manifest_path.is_file():
        return None, []
    try:
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        return None, [f'The run manifest is unreadable: {exc}']
    if not isinstance(manifest, dict):
        return None, ['The run manifest root is not a JSON object.']
    return manifest, []


def _product_command(*args: str) -> str:
    """Render one shell-safe command through the stable product surface."""
    configured = os.environ.get('LIDARSLAM_CLI_COMMAND')
    if configured:
        try:
            prefix = shlex.split(configured)
        except ValueError:
            prefix = ['lidarslam-map', 'inspect']
        if prefix and prefix[-1] == 'inspect':
            prefix.pop()
    else:
        prefix = ['lidarslam-map']
    return shlex.join([*prefix, *args])


def _manifest_bag_path(manifest: dict[str, Any] | None) -> Path | None:
    """Return a still-present source bag without trusting arbitrary fields."""
    if not isinstance(manifest, dict):
        return None
    input_payload = manifest.get('input')
    if not isinstance(input_payload, dict):
        return None
    value = input_payload.get('bag_path')
    if not isinstance(value, str) or not value:
        return None
    path = Path(value).expanduser()
    if not path.is_dir():
        return None
    return path.resolve()


def _build_symptom_triage(
    symptom: str,
    run_dir: Path,
    bag_path: Path | None,
) -> dict[str, Any]:
    """Translate one user-reported symptom into bounded review steps."""
    if symptom not in SYMPTOM_GUIDANCE:
        raise ValueError(f'unsupported map symptom: {symptom}')
    guidance = SYMPTOM_GUIDANCE[symptom]
    inspect_args = [
        'inspect',
        str(run_dir),
        '--symptom',
        symptom,
    ]
    if bag_path is not None:
        inspect_args.extend(['--bag', str(bag_path)])
    inspect_args.append('--write')

    commands: list[str] = []
    if symptom == 'map-is-not-visible':
        commands.append(_product_command('view', str(run_dir)))
    elif bag_path is not None:
        commands.append(_product_command('doctor', str(bag_path)))
    commands.append(_product_command(*inspect_args))
    commands.append(_product_command('support', str(run_dir)))

    return {
        'symptom': symptom,
        'code': symptom,
        'basis': 'USER_REPORTED_NOT_AUTOMATICALLY_DIAGNOSED',
        'title': guidance['title'],
        'checks': list(guidance['checks']),
        'next_commands': commands,
        'avoid': list(guidance['avoid']),
    }


def _parse_verify_log(text: str) -> dict[str, Any]:
    result = 'unknown'
    if 'RESULT: PASS' in text:
        result = 'PASS'
    elif 'RESULT: FAIL' in text:
        result = 'FAIL'

    counts_match = re.search(r'PASS:\s*(\d+)\s*\|\s*WARN:\s*(\d+)\s*\|\s*FAIL:\s*(\d+)', text)
    counts = None
    if counts_match:
        counts = {
            'pass': int(counts_match.group(1)),
            'warn': int(counts_match.group(2)),
            'fail': int(counts_match.group(3)),
        }

    details = []
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith('- '):
            details.append(stripped[2:])
    return {'result': result, 'counts': counts, 'details': details[:10]}


def _parse_projector_type(run_dir: Path) -> str | None:
    path = run_dir / 'map_projector_info.yaml'
    if not path.is_file():
        return None
    try:
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except yaml.YAMLError as exc:
        raise ValueError(f'failed to parse {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError(f'{path} must contain a YAML mapping.')
    return data.get('projector_type')


def _extract_launch_flags(launch_log: str) -> dict[str, bool]:
    return {
        'rko_started': 'RKO LIO Node is up!' in launch_log,
        'graph_initialized': '[graph_based_slam]: initialization end' in launch_log,
        'scanmatcher_initialized': '[scan_matcher]: initialization end' in launch_log,
    }


def _extract_problem_hints(
    launch_log: str,
    map_save_log: str,
    verify_summary: dict[str, Any],
) -> list[str]:
    hints: list[str] = []
    combined = '\n'.join([launch_log, map_save_log])
    pcl_fallocate_enospc = (
        'raw_fallocate' in combined
        and (
            'returned 28' in combined
            or 'Error during raw_fallocate' in combined
        )
    )

    if 'process has died' in combined:
        hints.append(
            'A ROS node died during the run. '
            'Check the launch log tail for the crashing process.'
        )
    if 'failed to initialize rcl' in combined or "Couldn't parse params file" in combined:
        hints.append('The run hit a ROS parameter-file parsing error.')
    if 'TF_NO_FRAME_ID' in combined or 'TF_NO_CHILD_FRAME_ID' in combined:
        hints.append('TF messages were malformed or incomplete.')
    if 'Could not find a connection between' in combined:
        hints.append('TF tree connectivity was missing for the requested frames.')
    if 'map_save service call failed' in combined:
        hints.append('The /map_save service call failed or timed out.')
    if (
        'No space left on device' in combined
        or 'ENOSPC' in combined
        or 'Disk quota exceeded' in combined
        or pcl_fallocate_enospc
    ):
        hints.append(
            'The output filesystem ran out of writable space or quota. '
            'Preserve the run evidence, free storage, and rerun into a new output directory.'
        )
    if 'Added 0 GNSS position constraint edges' in combined:
        hints.append('GNSS was enabled but the backend accepted zero GNSS edges.')
    if verify_summary['result'] == 'FAIL':
        hints.append('Autoware map verification failed.')
    return hints


def _suggest_next_steps(summary: dict[str, Any]) -> list[str]:
    run_dir = summary['run_dir']
    pointcloud_map_dir = f'{run_dir}/pointcloud_map'
    steps: list[str] = []

    if summary['status'] == 'success':
        steps.extend([
            f'bash scripts/run_graph_slam_pointcloud_map_in_autoware_foxglove.sh {run_dir}',
            f'bash scripts/run_graph_slam_pointcloud_map_in_autoware.sh {run_dir}',
        ])
    elif summary['status'] == 'map_saved':
        steps.append(f'python3 scripts/verify_autoware_map.py {pointcloud_map_dir}')
    elif summary['status'] == 'verify_failed':
        steps.extend([
            f'python3 scripts/verify_autoware_map.py {pointcloud_map_dir}',
            f'python3 scripts/diagnose_autoware_map_run.py {run_dir} --write',
        ])
    else:
        launch_log = summary['files']['launch_log']
        if launch_log:
            steps.append(f'tail -n 120 {launch_log}')
        if 'bag_preflight' in summary:
            steps.append(
                'python3 scripts/preflight_autoware_map_bag.py '
                f"{summary['bag_preflight']['summary']['bag_path']}"
            )

    if summary['files']['verify_log']:
        steps.append(f"less {summary['files']['verify_log']}")
    elif summary['files']['pointcloud_map_metadata']:
        steps.append(f'python3 scripts/verify_autoware_map.py {pointcloud_map_dir}')

    return steps[:5]


def summarize_run(
    run_dir: Path,
    bag_path: Path | None = None,
    symptom: str | None = None,
) -> dict[str, Any]:
    run_dir = run_dir.expanduser().resolve()
    launch_log_path = _find_first(run_dir, ['lidarslam.launch.log', 'slam.launch.log'])
    map_save_log_path = _find_first(run_dir, ['map_save.log'])
    verify_log_path = _find_first(run_dir, ['verify_autoware_map.log'])
    pointcloud_metadata_path = run_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml'
    map_projector_path = run_dir / 'map_projector_info.yaml'

    launch_log = _read_text(launch_log_path)
    map_save_log = _read_text(map_save_log_path)
    verify_log = _read_text(verify_log_path)
    verify_summary = _parse_verify_log(verify_log)
    launch_flags = _extract_launch_flags(launch_log)
    problem_hints = _extract_problem_hints(launch_log, map_save_log, verify_summary)
    run_manifest, manifest_hints = _read_run_manifest(run_dir)
    problem_hints.extend(manifest_hints)
    manifest_status = run_manifest.get('status') if run_manifest else None
    manifest_lifecycle = run_manifest.get('lifecycle') if run_manifest else None
    if not isinstance(manifest_lifecycle, dict):
        manifest_lifecycle = {}
    if manifest_status == 'interrupted':
        terminal_error = manifest_lifecycle.get('last_error')
        problem_hints.append(
            terminal_error
            or 'The map workflow was interrupted before successful completion.'
        )
    elif manifest_status == 'failed':
        terminal_error = manifest_lifecycle.get('last_error')
        problem_hints.append(
            terminal_error
            or 'The map workflow recorded a terminal failure.'
        )

    pointcloud_map_exists = pointcloud_metadata_path.is_file()
    map_projector_exists = map_projector_path.is_file()
    projector_type = _parse_projector_type(run_dir)

    status = 'incomplete'
    if manifest_status in ('failed', 'interrupted'):
        status = 'runtime_failed'
    elif pointcloud_map_exists and map_projector_exists and verify_summary['result'] == 'PASS':
        status = 'success'
    elif verify_summary['result'] == 'FAIL':
        status = 'verify_failed'
    elif pointcloud_map_exists and map_projector_exists:
        status = 'map_saved'
    elif (
        launch_flags['rko_started']
        or launch_flags['graph_initialized']
        or launch_flags['scanmatcher_initialized']
        or problem_hints
    ):
        status = 'runtime_failed'

    summary: dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'schema_uri': SCHEMA_URI,
        'run_dir': str(run_dir),
        'status': status,
        'files': {
            'launch_log': str(launch_log_path) if launch_log_path else None,
            'map_save_log': str(map_save_log_path) if map_save_log_path else None,
            'verify_log': str(verify_log_path) if verify_log_path else None,
            'pointcloud_map_metadata': (
                str(pointcloud_metadata_path) if pointcloud_map_exists else None
            ),
            'map_projector_info': str(map_projector_path) if map_projector_exists else None,
        },
        'launch_flags': launch_flags,
        'verify': verify_summary,
        'projector_type': projector_type,
        'problem_hints': problem_hints,
    }

    if bag_path is not None:
        module = _load_preflight_module()
        if hasattr(module, 'validate_bag_path'):
            module.validate_bag_path(bag_path)
        summary['bag_preflight'] = module.build_preflight_payload(bag_path)
    summary['suggested_next_steps'] = _suggest_next_steps(summary)
    if symptom is not None:
        triage_bag_path = bag_path or _manifest_bag_path(run_manifest)
        symptom_triage = _build_symptom_triage(
            symptom,
            run_dir,
            triage_bag_path,
        )
        summary['symptom_triage'] = symptom_triage
        summary['suggested_next_steps'] = symptom_triage['next_commands']
    return summary


def render_markdown(summary: dict[str, Any]) -> str:
    lines = [
        '# Autoware Map Run Diagnosis',
        '',
        f"- run_dir: `{summary['run_dir']}`",
        f"- status: `{summary['status']}`",
        f"- projector_type: `{summary['projector_type'] or 'missing'}`",
        f"- verify_result: `{summary['verify']['result']}`",
        '',
        '## Files',
    ]
    for key, value in summary['files'].items():
        lines.append(f"- `{key}`: `{value or 'missing'}`")

    lines.extend([
        '',
        '## Runtime Flags',
        f"- rko_started: `{summary['launch_flags']['rko_started']}`",
        f"- scanmatcher_initialized: `{summary['launch_flags']['scanmatcher_initialized']}`",
        f"- graph_initialized: `{summary['launch_flags']['graph_initialized']}`",
        '',
        '## Hints',
    ])
    if summary['problem_hints']:
        for hint in summary['problem_hints']:
            lines.append(f'- {hint}')
    else:
        lines.append('- No obvious failure signature was detected.')

    if summary['verify']['details']:
        lines.extend(['', '## Verify Details'])
        for detail in summary['verify']['details']:
            lines.append(f'- {detail}')

    symptom_triage = summary.get('symptom_triage')
    if symptom_triage is not None:
        lines.extend([
            '',
            '## Reported Symptom Triage',
            f"- code: `{symptom_triage['code']}`",
            f"- symptom: {symptom_triage['title']}",
            '- evidence boundary: user-reported symptom; this is not an '
            'automatic root-cause or accuracy diagnosis.',
            '',
            '### Check In Order',
        ])
        for check in symptom_triage['checks']:
            lines.append(f'- {check}')
        lines.extend(['', '### Avoid'])
        for warning in symptom_triage['avoid']:
            lines.append(f'- {warning}')

    if summary['suggested_next_steps']:
        lines.extend(['', '## Suggested Next Commands'])
        for step in summary['suggested_next_steps']:
            lines.append(f'- `{step}`')

    if 'bag_preflight' in summary:
        bag_preflight = summary['bag_preflight']
        lines.extend([
            '',
            '## Bag Preflight',
            f"- recommended_profile_id: `{bag_preflight['recommended_profile_id']}`",
        ])
        if bag_preflight['recommendations']:
            lines.append('- recommended_label: '
                         f"`{bag_preflight['recommendations'][0]['label']}`")

    return '\n'.join(lines)


def validate_run_dir(run_dir: Path) -> None:
    """Validate a CLI run directory input."""
    if run_dir.is_file():
        raise FileNotFoundError(
            f'run directory path is a file, not a directory: {run_dir}. '
            'Pass the output directory created by the map workflow.'
        )
    if not run_dir.exists():
        raise FileNotFoundError(
            f'run directory does not exist: {run_dir}. '
            'Pass the output directory created by the map workflow.'
        )
    if not run_dir.is_dir():
        raise FileNotFoundError(
            f'run directory path is not a directory: {run_dir}. '
            'Pass the output directory created by the map workflow.'
        )
    if run_dir.name == 'pointcloud_map' and (run_dir / 'pointcloud_map_metadata.yaml').is_file():
        raise ValueError(
            f'run_dir points to the nested pointcloud_map directory: {run_dir}. '
            'Pass its parent output directory instead.'
        )


def _help_epilog() -> str:
    command = os.environ.get(
        'LIDARSLAM_CLI_COMMAND',
        'python3 scripts/diagnose_autoware_map_run.py',
    )
    return '\n'.join([
        'The input must be the map workflow output directory.',
        'Pass output/autoware_map_authoring_<bag>_<timestamp>, not its pointcloud_map/ child.',
        '',
        'Files this tool checks when present:',
        '  lidarslam.launch.log or slam.launch.log',
        '  map_save.log',
        '  verify_autoware_map.log',
        '  pointcloud_map/pointcloud_map_metadata.yaml',
        '  map_projector_info.yaml',
        '',
        'Examples:',
        f'  {command} output/my_map_run',
        f'  {command} output/my_map_run --write',
        f'  {command} output/my_map_run --bag /path/to/rosbag2',
        f'  {command} output/my_map_run --symptom map-spins-or-spirals',
        f'  {command} output/my_map_run --json',
    ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Diagnose an Autoware-compatible map workflow output directory and '
            'suggest the next command.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_help_epilog(),
    )
    parser.add_argument(
        'run_dir',
        metavar='output_dir',
        help='Map workflow output directory containing logs and map artifacts.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all options (this command has no advanced options).',
    )
    parser.add_argument(
        '--bag',
        metavar='<rosbag2_dir>',
        help='Optional source rosbag2 directory to include preflight context.',
    )
    parser.add_argument(
        '--symptom',
        choices=SYMPTOM_CHOICES,
        metavar='{map-spins-or-spirals,pose-drifts-or-oscillates,'
        'map-stops-early,map-is-too-sparse,map-is-not-visible}',
        help=(
            'Add bounded checks for one observed visual symptom; this records '
            'a user report and never claims an automatic root cause.'
        ),
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of markdown.')
    parser.add_argument(
        '--write',
        action='store_true',
        help='Write autoware_map_diagnosis.md/json into the run directory.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dir = Path(args.run_dir).expanduser().resolve()
    bag_path = Path(args.bag).expanduser().resolve() if args.bag else None
    try:
        validate_run_dir(run_dir)
        summary = summarize_run(run_dir, bag_path, args.symptom)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2

    markdown = render_markdown(summary)

    if args.write:
        try:
            (run_dir / 'autoware_map_diagnosis.md').write_text(markdown, encoding='utf-8')
            (run_dir / 'autoware_map_diagnosis.json').write_text(
                json.dumps(summary, indent=2, sort_keys=True),
                encoding='utf-8',
            )
        except OSError as exc:
            print(
                f'error: failed to write diagnosis files under {run_dir}: {exc}',
                file=sys.stderr,
            )
            return 2

    if args.json:
        print(json.dumps(summary, indent=2, sort_keys=True))
    else:
        print(markdown)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
