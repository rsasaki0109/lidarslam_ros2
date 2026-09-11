#!/usr/bin/env python3
"""Open a verified map-run output in an optional product viewer."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import webbrowser
from pathlib import Path
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
EX_USAGE = 2
EX_SOFTWARE = 70
BROWSER_PREVIEW_HTML = 'mid360_robot_3d_map_preview.html'


def _positive_seconds(value: str) -> int:
    try:
        seconds = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be a positive integer') from exc
    if seconds <= 0:
        raise argparse.ArgumentTypeError('must be a positive integer')
    return seconds


def _help_epilog(*, show_advanced: bool = True) -> str:
    command = os.environ.get(
        'LIDARSLAM_CLI_COMMAND',
        'python3 scripts/view_autoware_map.py',
    )
    lines = [
        'The input must be a completed map-run output containing:',
        '  pointcloud_map/pointcloud_map_metadata.yaml',
        '  map_projector_info.yaml',
        '',
        'Examples:',
        f'  {command} output/my_map',
        f'  {command} output/my_map --viewer foxglove',
        f'  {command} output/my_map --no-open',
    ]
    if show_advanced:
        lines.append(
            f'  {command} output/my_map --viewer autoware --rebuild'
        )
    return '\n'.join(lines)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the public viewer command options."""
    show_all_help = os.environ.get('LIDARSLAM_CLI_HELP_MODE') != 'core'

    def advanced_help(text: str) -> str:
        return text if show_all_help else argparse.SUPPRESS

    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Validate an existing map-run output and open a lightweight '
            'browser preview or an optional live viewer.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_help_epilog(show_advanced=show_all_help),
    )
    parser.add_argument(
        'output_dir',
        help='Completed map-run output directory.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show advanced viewer-runtime options.',
    )
    viewer_options = parser.add_argument_group('viewer')
    viewer_options.add_argument(
        '--viewer',
        choices=['browser', 'autoware', 'foxglove'],
        default='browser',
        help='Viewer to open (default: browser; self-contained and offline).',
    )
    viewer_options.add_argument(
        '--no-open',
        action='store_true',
        help='Create the browser preview without opening a desktop browser.',
    )
    runtime_options = parser.add_argument_group('viewer runtime')
    runtime_options.add_argument(
        '--preview-dir',
        metavar='<dir>',
        help=advanced_help(
            'Browser preview output directory (default: <output_dir>/preview).'
        ),
    )
    runtime_options.add_argument(
        '--autoware-core-dir',
        metavar='<dir>',
        help=advanced_help('autoware_core checkout used by the map loaders.'),
    )
    runtime_options.add_argument(
        '--work-dir',
        metavar='<dir>',
        help=advanced_help('Runtime workspace directory used by the viewer.'),
    )
    runtime_options.add_argument(
        '--runtime-dir',
        metavar='<dir>',
        help=advanced_help('Existing built Docker viewer runtime to reuse.'),
    )
    runtime_options.add_argument(
        '--rebuild',
        action='store_true',
        help=advanced_help('Rebuild the viewer runtime before opening.'),
    )
    runtime_options.add_argument(
        '--auto-exit-secs',
        type=_positive_seconds,
        metavar='<seconds>',
        help=advanced_help('Automatically close the viewer after N seconds.'),
    )
    return parser.parse_args(argv)


def validate_output_dir(output_dir: Path) -> None:
    """Reject paths that are not complete map-run outputs."""
    if not output_dir.is_dir():
        raise ValueError(f'map output directory does not exist: {output_dir}')

    required = (
        output_dir / 'pointcloud_map' / 'pointcloud_map_metadata.yaml',
        output_dir / 'map_projector_info.yaml',
    )
    missing = [str(path.relative_to(output_dir)) for path in required if not path.is_file()]
    if missing:
        raise ValueError(
            'map output is incomplete; missing: '
            + ', '.join(missing)
            + '. Run "lidarslam-map inspect <output_dir>" for diagnosis.'
        )


def build_viewer_command(
    args: argparse.Namespace,
    output_dir: Path,
) -> list[str]:
    """Translate product options into the maintained viewer helper command."""
    if args.viewer == 'browser':
        preview_dir = (
            Path(args.preview_dir).expanduser().resolve()
            if args.preview_dir
            else output_dir / 'preview'
        )
        return [
            sys.executable,
            str(SCRIPT_DIR / 'export_mid360_robot_3d_map_preview.py'),
            str(output_dir),
            '--output-dir',
            str(preview_dir),
        ]

    script_name = (
        'run_graph_slam_pointcloud_map_in_autoware_foxglove.sh'
        if args.viewer == 'foxglove'
        else 'run_graph_slam_pointcloud_map_in_autoware.sh'
    )
    command = ['bash', str(SCRIPT_DIR / script_name), str(output_dir)]
    if args.autoware_core_dir:
        command.extend(['--autoware-core-dir', args.autoware_core_dir])
    if args.work_dir:
        command.extend(['--work-dir', args.work_dir])
    if args.runtime_dir:
        command.extend(['--run-dir', args.runtime_dir])
    if args.rebuild:
        command.append('--rebuild')
    if args.auto_exit_secs is not None:
        command.extend(['--auto-exit-secs', str(args.auto_exit_secs)])
    return command


def validate_viewer_options(args: argparse.Namespace) -> None:
    """Reject viewer-specific options that would otherwise be ignored."""
    heavy_runtime_options = {
        '--autoware-core-dir': args.autoware_core_dir,
        '--work-dir': args.work_dir,
        '--runtime-dir': args.runtime_dir,
        '--rebuild': args.rebuild,
        '--auto-exit-secs': args.auto_exit_secs,
    }
    if args.viewer == 'browser':
        invalid = [name for name, value in heavy_runtime_options.items() if value]
        if invalid:
            raise ValueError(
                f'{", ".join(invalid)} cannot be used with --viewer browser; '
                'select --viewer autoware or --viewer foxglove.'
            )
        return

    browser_options = {
        '--no-open': args.no_open,
        '--preview-dir': args.preview_dir,
    }
    invalid = [name for name, value in browser_options.items() if value]
    if invalid:
        raise ValueError(
            f'{", ".join(invalid)} can only be used with --viewer browser.'
        )
    if args.autoware_core_dir and args.viewer != 'autoware':
        raise ValueError('--autoware-core-dir requires --viewer autoware.')


def browser_preview_path(args: argparse.Namespace, output_dir: Path) -> Path:
    """Return the HTML artifact produced by the browser viewer command."""
    preview_dir = (
        Path(args.preview_dir).expanduser().resolve()
        if args.preview_dir
        else output_dir / 'preview'
    )
    return preview_dir / BROWSER_PREVIEW_HTML


def desktop_session_available() -> bool:
    """Return whether opening a local browser is likely to reach the user."""
    if sys.platform == 'darwin' or os.name == 'nt':
        return True
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


def open_browser(uri: str) -> bool:
    """Open the preview URI through the platform browser registry."""
    return bool(webbrowser.open(uri, new=2))


def finish_browser_preview(
    args: argparse.Namespace,
    output_dir: Path,
) -> int:
    """Report and optionally open a generated self-contained preview."""
    html_path = browser_preview_path(args, output_dir)
    if not html_path.is_file():
        print(
            f'error: browser preview did not produce {html_path}.',
            file=sys.stderr,
        )
        return EX_SOFTWARE

    uri = html_path.as_uri()
    print('')
    print('3D browser preview ready')
    print(f'  HTML: {html_path}')
    print(f'  URI:  {uri}')
    if args.no_open:
        print('Browser opening skipped by --no-open.')
        return 0
    if not desktop_session_available():
        print('No desktop session detected; open the HTML file on the host.')
        return 0
    try:
        opened = open_browser(uri)
    except (OSError, webbrowser.Error) as exc:
        print(f'warning: could not open a browser automatically: {exc}', file=sys.stderr)
        return 0
    if not opened:
        print('warning: no browser accepted the preview URI; open the HTML file manually.', file=sys.stderr)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    """Validate the map output and propagate the selected viewer result."""
    args = parse_args(argv)
    output_dir = Path(args.output_dir).expanduser().resolve()
    try:
        validate_output_dir(output_dir)
        validate_viewer_options(args)
        completed = subprocess.run(
            build_viewer_command(args, output_dir),
            check=False,
        )
    except ValueError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return EX_USAGE
    except OSError as exc:
        print(f'error: failed to start viewer: {exc}', file=sys.stderr)
        return EX_SOFTWARE

    if completed.returncode != 0:
        print(
            f'error: {args.viewer} viewer failed with exit code '
            f'{completed.returncode}.',
            file=sys.stderr,
        )
        return completed.returncode
    if args.viewer == 'browser':
        return finish_browser_preview(args, output_dir)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
