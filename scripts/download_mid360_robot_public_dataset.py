#!/usr/bin/env python3
"""Download and prepare a public MID-360 dataset for the robot checks."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_public_datasets import (
    PublicDatasetIntake,
    PublicDatasetIntakeOptions,
    payload_to_json,
    public_dataset_payload,
    public_dataset_registry,
    render_public_dataset_intake_markdown,
    render_public_dataset_list,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


SCRIPT_DIR = Path(__file__).resolve().parent
PRODUCT_ROOT = SCRIPT_DIR.parent
SOURCE_LAYOUT = (
    (PRODUCT_ROOT / 'Dockerfile').is_file()
    and (PRODUCT_ROOT / 'lidarslam' / 'package.xml').is_file()
)
DEFAULT_WORK_ROOT = PRODUCT_ROOT if SOURCE_LAYOUT else Path.cwd()
DEFAULT_DATASET_ROOT = DEFAULT_WORK_ROOT / 'datasets' / 'mid360_public'
RECORDING_CHECK_SCRIPT = SCRIPT_DIR / 'check_mid360_robot_recording.py'


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(
        description='Download a public MID-360 dataset and prepare recording-check inputs.'
    )
    parser.add_argument('--list', action='store_true', help='List known public datasets.')
    parser.add_argument(
        '--dataset',
        default='driving_slam_mid360',
        help='Public dataset id. Use --list to show ids.',
    )
    parser.add_argument('--file', default='', help='Dataset file id. Defaults to dataset default.')
    parser.add_argument(
        '--dataset-root',
        default=str(DEFAULT_DATASET_ROOT),
        help='Root directory for public MID-360 downloads.',
    )
    parser.add_argument(
        '--output-dir',
        default='',
        help='Output directory for the generated recording check command.',
    )
    parser.add_argument('--dry-run', action='store_true', help='Write a plan without downloading.')
    parser.add_argument(
        '--force',
        action='store_true',
        help=(
            'Restart partial work and atomically replace an existing '
            'archive/extract directory.'
        ),
    )
    parser.add_argument('--no-extract', action='store_true', help='Download only; do not extract zip.')
    parser.add_argument(
        '--skip-md5',
        action='store_true',
        help=(
            'Skip legacy MD5 verification; registered SHA-256 and size '
            'checks remain mandatory.'
        ),
    )
    parser.add_argument(
        '--check',
        action='store_true',
        help='Run check_mid360_robot_recording.py after a bag directory is found.',
    )
    parser.add_argument('--json', action='store_true', help='Print JSON instead of Markdown.')
    return parser.parse_args()


def main() -> int:
    """Entry point."""
    args = parse_args()
    if args.list:
        if args.json:
            print(payload_to_json(public_dataset_payload()))
        else:
            print(render_public_dataset_list(public_dataset_registry()))
        return 0

    options = PublicDatasetIntakeOptions(
        dataset_id=args.dataset,
        dataset_root=Path(args.dataset_root),
        file_id=args.file,
        output_dir=Path(args.output_dir) if args.output_dir else None,
        dry_run=args.dry_run,
        force=args.force,
        extract=not args.no_extract,
        verify_md5=not args.skip_md5,
    )
    try:
        report = PublicDatasetIntake(PRODUCT_ROOT).run(options)
        if args.check:
            report = _run_recording_check(report)
            _write_updated_report(report)
    except Exception as exc:
        print(f'failed to prepare public MID-360 dataset: {exc}', file=sys.stderr)
        return 1

    if args.json:
        print(payload_to_json(report))
    else:
        print(render_public_dataset_intake_markdown(report))
        print(f"Public dataset intake JSON: {report['manifest_json']}")
        print(f"Public dataset intake Markdown: {report['manifest_markdown']}")
    if report['status'] == 'FAIL':
        return 1
    return 0


def _run_recording_check(report: dict[str, Any]) -> dict[str, Any]:
    if not RECORDING_CHECK_SCRIPT.is_file():
        updated = dict(report)
        updated['status'] = 'FAIL'
        updated['recording_check'] = {
            'returncode': 127,
            'stdout': '',
            'stderr': (
                '--check is unavailable in this curated runtime; use a source '
                'checkout or run the installed lidarslam-map preflight.'
            ),
            'report': {},
        }
        messages = list(updated.get('messages') or [])
        messages.append(updated['recording_check']['stderr'])
        updated['messages'] = messages
        return updated
    if not report.get('selected_bag_path'):
        updated = dict(report)
        updated['status'] = 'FAIL'
        updated['recording_check'] = {
            'returncode': 1,
            'stdout': '',
            'stderr': 'No selected_bag_path; cannot run recording check.',
            'report': {},
        }
        return updated

    command = [
        sys.executable,
        str(RECORDING_CHECK_SCRIPT),
        '--bag',
        report['selected_bag_path'],
        '--robot-profile',
        report['profile_path'],
        '--output-dir',
        report['output_dir'],
        '--json',
    ]
    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
        cwd=PRODUCT_ROOT,
    )
    check_report: dict[str, Any] = {}
    if result.stdout.strip():
        try:
            check_report = json.loads(result.stdout)
        except json.JSONDecodeError:
            check_report = {}

    updated = dict(report)
    updated['recording_check'] = {
        'returncode': result.returncode,
        'stdout': result.stdout,
        'stderr': result.stderr,
        'report': check_report,
    }
    if result.returncode != 0 or check_report.get('status') == 'FAIL':
        updated['status'] = 'FAIL'
    else:
        updated['status'] = check_report.get('status') or report['status']
    messages = list(updated.get('messages') or [])
    messages.append(
        'Recording check status: '
        f'{check_report.get("status", "unknown")} (returncode {result.returncode}).'
    )
    updated['messages'] = messages
    return updated


def _write_updated_report(report: dict[str, Any]) -> None:
    json_path = Path(report['manifest_json'])
    markdown_path = Path(report['manifest_markdown'])
    json_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(
        render_public_dataset_intake_markdown(report) + '\n',
        encoding='utf-8',
    )


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('Interrupted', file=sys.stderr)
        raise SystemExit(130)
