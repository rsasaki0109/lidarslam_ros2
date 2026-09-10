#!/usr/bin/env python3
"""Run the standard MID-360 robot field-session workflow."""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_dashboard import DASHBOARD_HTML, write_dashboard
from lidarslam_benchmark_tools.mid360_robot_record_tools import Mid360RobotRecordPlanner, RecordOptions
from lidarslam_benchmark_tools.mid360_robot_tools import RobotProfileLoader, payload_to_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE = REPO_ROOT / 'configs' / 'mid360_robot' / 'livox_mid360_default.yaml'
FIELD_SESSION_JSON = 'mid360_robot_field_session.json'
FIELD_SESSION_MARKDOWN = 'mid360_robot_field_session.md'


@dataclass(frozen=True)
class FieldStep:
    """One field-session workflow step."""

    id: str
    status: str
    message: str
    command: list[str]
    returncode: int | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Run host readiness, recording, and post-checks for a MID-360 robot session.'
    )
    parser.add_argument(
        '--robot-profile',
        default=str(DEFAULT_PROFILE),
        help='Robot profile YAML with expected MID-360 topics.',
    )
    parser.add_argument(
        '--bag-root',
        required=True,
        help='Directory where rosbag2 output and recording sidecars are written.',
    )
    parser.add_argument('--run-id', default='', help='Recording run id and bag directory name.')
    parser.add_argument('--duration-sec', default='', help='Optional recording duration.')
    parser.add_argument(
        '--extra-topic',
        action='append',
        default=[],
        help='Additional topic to record. Can be passed more than once.',
    )
    parser.add_argument('--no-tf', action='store_true', help='Do not record /tf.')
    parser.add_argument('--no-tf-static', action='store_true', help='Do not record /tf_static.')
    parser.add_argument('--storage-id', default='', help='rosbag2 storage id.')
    parser.add_argument('--max-cache-size', default='', help='ros2 bag record --max-cache-size.')
    parser.add_argument('--compression-mode', default='', help='ros2 bag record compression mode.')
    parser.add_argument('--compression-format', default='', help='ros2 bag record compression format.')
    parser.add_argument(
        '--output-dir',
        help='Directory for field-session, recording-check, readiness, and map-plan reports.',
    )
    parser.add_argument('--host-root', default='/', help='Host filesystem root for host readiness.')
    parser.add_argument(
        '--skip-host-readiness',
        action='store_true',
        help='Skip Jetson host readiness. Useful when planning from a development machine.',
    )
    parser.add_argument(
        '--record-only',
        action='store_true',
        help='Stop after recording; skip post-recording check and map dry-run.',
    )
    parser.add_argument(
        '--run-map',
        action='store_true',
        help='After recording and post-check, run mapping with diagnosis. Off by default.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Write plans and commands without recording or mapping.',
    )
    parser.add_argument('--json', action='store_true', help='Print field-session JSON.')
    return parser.parse_args()


def _record_options(args: argparse.Namespace) -> RecordOptions:
    return RecordOptions(
        bag_root=Path(args.bag_root).expanduser().resolve(),
        run_id=args.run_id,
        include_tf=not args.no_tf,
        include_tf_static=not args.no_tf_static,
        extra_topics=tuple(args.extra_topic or []),
        storage_id=args.storage_id,
        max_cache_size=args.max_cache_size,
        compression_mode=args.compression_mode,
        compression_format=args.compression_format,
        duration_sec=args.duration_sec,
    )


def _output_dir(args: argparse.Namespace, run_id: str) -> Path:
    if args.output_dir:
        return Path(args.output_dir).expanduser().resolve()
    return REPO_ROOT / 'output' / f'mid360_robot_field_session_{run_id}'


def _record_command(args: argparse.Namespace, run_id: str, dry_run: bool) -> list[str]:
    command = [
        'bash',
        str(REPO_ROOT / 'scripts' / 'record_mid360_robot_bag.sh'),
        '--robot-profile',
        str(Path(args.robot_profile).expanduser().resolve()),
        '--bag-root',
        str(Path(args.bag_root).expanduser().resolve()),
    ]
    command.extend(['--run-id', run_id])
    if args.duration_sec:
        command.extend(['--duration-sec', args.duration_sec])
    for topic in args.extra_topic or []:
        command.extend(['--extra-topic', topic])
    if args.no_tf:
        command.append('--no-tf')
    if args.no_tf_static:
        command.append('--no-tf-static')
    if args.storage_id:
        command.extend(['--storage-id', args.storage_id])
    if args.max_cache_size:
        command.extend(['--max-cache-size', args.max_cache_size])
    if args.compression_mode:
        command.extend(['--compression-mode', args.compression_mode])
    if args.compression_format:
        command.extend(['--compression-format', args.compression_format])
    if dry_run:
        command.append('--dry-run')
    return command


def _host_command(args: argparse.Namespace, output_dir: Path) -> list[str]:
    return [
        'python3',
        str(REPO_ROOT / 'scripts' / 'check_jetson_mid360_host_readiness.py'),
        '--bag-dir',
        str(Path(args.bag_root).expanduser().resolve()),
        '--output-dir',
        str(output_dir),
        '--host-root',
        str(Path(args.host_root).expanduser().resolve()),
    ]


def _post_check_command(plan: Any, output_dir: Path) -> list[str]:
    return [
        'bash',
        str(REPO_ROOT / 'scripts' / 'check_mid360_robot_recording.sh'),
        '--bag',
        str(plan.bag_path),
        '--robot-profile',
        str(plan.profile_snapshot_path),
        '--record-plan',
        str(plan.manifest_json_path),
        '--output-dir',
        str(output_dir),
    ]


def _map_command(plan: Any, output_dir: Path) -> list[str]:
    return [
        'bash',
        str(REPO_ROOT / 'scripts' / 'run_mid360_robot_map.sh'),
        str(plan.bag_path),
        '--robot-profile',
        str(plan.profile_snapshot_path),
        '--output-dir',
        str(output_dir),
        '--write-manifest',
        '--write-diagnosis',
    ]


def _run_step(
    step_id: str,
    message: str,
    command: list[str],
    quiet: bool = False,
) -> FieldStep:
    completed = subprocess.run(
        command,
        check=False,
        cwd=REPO_ROOT,
        stdout=subprocess.DEVNULL if quiet else None,
        stderr=subprocess.DEVNULL if quiet else None,
    )
    return FieldStep(
        id=step_id,
        status='ok' if completed.returncode == 0 else 'fail',
        message=message if completed.returncode == 0 else f'{message} failed.',
        command=command,
        returncode=completed.returncode,
    )


def _planned_step(step_id: str, message: str, command: list[str]) -> FieldStep:
    return FieldStep(
        id=step_id,
        status='planned',
        message=message,
        command=command,
    )


def _skipped_step(step_id: str, message: str, command: list[str] | None = None) -> FieldStep:
    return FieldStep(
        id=step_id,
        status='skipped',
        message=message,
        command=command or [],
    )


def _status_from_steps(steps: list[FieldStep]) -> str:
    if any(step.status == 'fail' for step in steps):
        return 'FAIL'
    return 'PASS'


def _count_steps(steps: list[FieldStep]) -> dict[str, int]:
    return {
        'ok': sum(1 for step in steps if step.status == 'ok'),
        'planned': sum(1 for step in steps if step.status == 'planned'),
        'skipped': sum(1 for step in steps if step.status == 'skipped'),
        'fail': sum(1 for step in steps if step.status == 'fail'),
    }


def _build_report(
    args: argparse.Namespace,
    plan: Any,
    output_dir: Path,
    steps: list[FieldStep],
) -> dict[str, Any]:
    return {
        'created_at': datetime.now(timezone.utc).isoformat(),
        'status': _status_from_steps(steps),
        'dry_run': args.dry_run,
        'run_map': args.run_map,
        'record_only': args.record_only,
        'run_id': plan.run_id,
        'bag_root': str(plan.bag_root),
        'bag_path': str(plan.bag_path),
        'output_dir': str(output_dir),
        'profile_path': str(Path(args.robot_profile).expanduser().resolve()),
        'profile_snapshot_path': str(plan.profile_snapshot_path),
        'record_plan_json_path': str(plan.manifest_json_path),
        'record_plan_markdown_path': str(plan.manifest_markdown_path),
        'host_readiness_json_path': str(output_dir / 'jetson_mid360_host_readiness.json'),
        'recording_check_json_path': str(output_dir / 'mid360_robot_recording_check.json'),
        'readiness_json_path': str(output_dir / 'mid360_robot_readiness.json'),
        'map_plan_json_path': str(output_dir / 'mid360_robot_run_plan.json'),
        'field_session_json_path': str(output_dir / FIELD_SESSION_JSON),
        'field_session_markdown_path': str(output_dir / FIELD_SESSION_MARKDOWN),
        'dashboard_html_path': str(output_dir / DASHBOARD_HTML),
        'steps': [asdict(step) for step in steps],
        'counts': _count_steps(steps),
    }


def _write_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / FIELD_SESSION_JSON
    markdown_path = output_dir / FIELD_SESSION_MARKDOWN
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(_render_markdown(report) + '\n', encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path}


def _render_markdown(report: dict[str, Any]) -> str:
    lines = [
        '# MID-360 Robot Field Session',
        '',
        f"- status: `{report['status']}`",
        f"- created_at: `{report['created_at']}`",
        f"- dry_run: `{report['dry_run']}`",
        f"- run_map: `{report['run_map']}`",
        f"- run_id: `{report['run_id']}`",
        f"- bag_path: `{report['bag_path']}`",
        f"- output_dir: `{report['output_dir']}`",
        '',
        '## Artifacts',
        '',
        f"- profile_snapshot: `{report['profile_snapshot_path']}`",
        f"- record_plan_json: `{report['record_plan_json_path']}`",
        f"- host_readiness_json: `{report['host_readiness_json_path']}`",
        f"- recording_check_json: `{report['recording_check_json_path']}`",
        f"- readiness_json: `{report['readiness_json_path']}`",
        f"- map_plan_json: `{report['map_plan_json_path']}`",
        f"- dashboard_html: `{report['dashboard_html_path']}`",
        '',
        '## Steps',
        '',
    ]
    for step in report['steps']:
        lines.append(f"- `{step['status']}` `{step['id']}`: {step['message']}")
        if step.get('command'):
            lines.extend(['', '```bash', shlex.join(step['command']), '```', ''])
    return '\n'.join(lines)


def _print_report(report: dict[str, Any], paths: dict[str, Path]) -> None:
    print(_render_markdown(report))
    print(f"Field session JSON: {paths['json']}")
    print(f"Field session Markdown: {paths['markdown']}")


def main() -> int:
    args = parse_args()
    profile = RobotProfileLoader().load(Path(args.robot_profile).expanduser().resolve())
    plan = Mid360RobotRecordPlanner().build_plan(profile, _record_options(args))
    output_dir = _output_dir(args, plan.run_id)
    steps: list[FieldStep] = []

    if args.skip_host_readiness:
        steps.append(_skipped_step('host_readiness', 'Host readiness was skipped.'))
    elif args.dry_run:
        steps.append(_planned_step('host_readiness', 'Host readiness command planned.', _host_command(args, output_dir)))
    else:
        steps.append(
            _run_step(
                'host_readiness',
                'Host readiness completed.',
                _host_command(args, output_dir),
                quiet=args.json,
            )
        )

    if not any(step.status == 'fail' for step in steps):
        steps.append(
            _run_step(
                'recording_plan' if args.dry_run else 'recording',
                'Recording plan completed.' if args.dry_run else 'Recording completed.',
                _record_command(args, run_id=plan.run_id, dry_run=args.dry_run),
                quiet=args.json,
            )
        )

    if not any(step.status == 'fail' for step in steps):
        post_command = _post_check_command(plan, output_dir)
        if args.record_only:
            steps.append(_skipped_step('post_recording_check', 'Post-recording check skipped by --record-only.', post_command))
        elif args.dry_run:
            steps.append(_planned_step('post_recording_check', 'Post-recording check command planned.', post_command))
        else:
            steps.append(
                _run_step(
                    'post_recording_check',
                    'Post-recording check completed.',
                    post_command,
                    quiet=args.json,
                )
            )

    if not any(step.status == 'fail' for step in steps):
        map_command = _map_command(plan, output_dir)
        if args.record_only:
            steps.append(_skipped_step('map', 'Mapping skipped by --record-only.', map_command))
        elif args.run_map:
            if args.dry_run:
                steps.append(_planned_step('map', 'Mapping command planned by --run-map.', map_command))
            else:
                steps.append(_run_step('map', 'Mapping completed.', map_command, quiet=args.json))
        else:
            steps.append(
                _planned_step(
                    'map',
                    'Mapping command not executed; post-recording check writes the map dry-run plan.',
                    map_command,
                )
            )

    report = _build_report(args, plan, output_dir, steps)
    paths = _write_report(report, output_dir)
    dashboard_path = write_dashboard(output_dir)
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    else:
        _print_report(report, paths)
        print(f"Session dashboard HTML: {dashboard_path}")
    return 1 if report['status'] == 'FAIL' else 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
