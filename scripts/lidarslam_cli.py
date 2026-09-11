#!/usr/bin/env python3
"""Stable product command surface over the proven map-authoring tools."""

from __future__ import annotations

import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
from typing import Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
VERSION_PATH = REPO_ROOT / 'VERSION'
COMMANDS = {
    'demo': 'first_map_demo.py',
    'start': 'sensor_setup_wizard.py',
    'sessions': 'session_history.py',
    'compare': 'session_compare.py',
    'report': 'support_bundle.py',
    'support': 'support_bundle.py',
    'doctor': 'lidarslam_doctor.py',
    'setup': 'sensor_setup_wizard.py',
    'run': 'run_autoware_map_from_bag.py',
    'inspect': 'diagnose_autoware_map_run.py',
    'view': 'view_autoware_map.py',
    'edit': 'apply_map_edit.py',
    'merge': 'merge_map_sessions.py',
    'migrate-manifest': 'migrate_run_manifest.py',
    'rollback-plan': 'plan_image_rollback.py',
}
EX_USAGE = 2
EX_SOFTWARE = 70
HELP_MODE_ENV = 'LIDARSLAM_CLI_HELP_MODE'
SUPPORT_MODE_ENV = 'LIDARSLAM_SUPPORT_MODE'
EX_INTERRUPT = 130
COMMAND_SUPERVISOR_DEPTH_ENV = 'LIDARSLAM_CLI_SUPERVISOR_DEPTH'
COMMAND_INTERRUPT_GRACE_SECONDS = 35.0
COMMAND_INTERRUPT_TERMINATE_SECONDS = 10.0


class _CommandTerminationRequested(Exception):
    """Carry SIGTERM through a delegated-command wait."""

    def __init__(self, signum: int):
        super().__init__(signum)
        self.signum = signum


def command_name() -> str:
    """Return the source or installed command spelling."""
    configured = os.environ.get('LIDARSLAM_CLI_NAME')
    if configured:
        return configured
    return './scripts/lidarslam'


def render_help(*, include_all: bool = False) -> str:
    """Return top-level CLI help."""
    executable = command_name()
    lines = [
        f'Usage: {executable} <command> [options]',
        '',
        'Run with no command in an interactive terminal for a guided home.',
        '',
        'Fastest verified first map:',
        (
            '  demo [work_dir]      Download public data, map, verify, and '
            'open the result'
        ),
        (
            '  start <rosbag2_dir>    Detect, configure, map, verify, and open '
            'the result'
        ),
        '  sessions [output_dir]  Reopen recent map sessions and next actions',
        (
            '  compare <left> <right> Compare two sessions using retained '
            'evidence'
        ),
        '  report <output>      Prepare a verified first-map report',
        '  support <session>    Prepare a privacy-first maintainer ZIP',
        '',
        'Core rosbag2-to-map commands:',
        (
            '  doctor [rosbag2_dir] Check the install; with a bag, inspect '
            'compatibility'
        ),
        '  setup <rosbag2_dir>    Generate a reviewed sensor configuration bundle',
        '  run <rosbag2_dir>      Build and verify a map bundle (--guided for people)',
        '  inspect <output_dir>   Diagnose an existing map-authoring output',
        '',
        'Optional post-processing:',
        '  view <output_dir>      Open a completed map in the offline 3D preview',
        '  edit <output_dir>      Apply a 3D edit plan to a new verified candidate',
        '  merge <map_outputs...> Align sessions into one verified map project',
        '',
        'Global options:',
        '  --version              Print the repository product version',
        '  --help                 Show this help',
        '  --help-all             Show advanced and deprecated help',
        '',
        f'Run "{executable} <command> --help" for command options.',
        '',
        'Exit codes:',
        '  0   command completed successfully',
        '  2   invalid usage, input, profile, or output path',
        '  70  the command could not start because of an internal/tooling error',
        '  other non-zero values are propagated from a delegated workflow',
    ]
    if include_all:
        lines.extend([
            '',
            'Advanced recovery commands:',
            (
                '  migrate-manifest <output_dir>'
                '  Convert terminal v1 metadata for inspection'
            ),
            (
                '  rollback-plan <release_image_record>'
                '  Plan an immutable image rollback'
            ),
            '',
            'Help levels:',
            f'  {executable} <command> --help',
            '      Show the stable options needed for normal operation.',
            f'  {executable} <command> --help-all',
            '      Also show advanced runtime and deprecated compatibility options.',
        ])
    return '\n'.join(lines)


def read_version() -> str:
    """Read the root VERSION source of truth."""
    try:
        version = VERSION_PATH.read_text(encoding='utf-8').strip()
    except OSError as exc:
        raise RuntimeError(f'failed to read product version: {exc}') from exc
    if not version:
        raise RuntimeError(f'product version is empty: {VERSION_PATH}')
    return version


def _interactive_terminal() -> bool:
    """Return whether a human can safely answer the no-argument home prompt."""
    try:
        return sys.stdin.isatty() and sys.stdout.isatty()
    except AttributeError:
        return False


def _render_command(args: Sequence[str]) -> str:
    """Render one copy-ready command without asking a shell to interpret it."""
    return shlex.join([command_name(), *args])


def _confirm(prompt: str, *, default: bool) -> bool:
    """Ask one bounded yes/no question; ambiguous input never starts work."""
    accepted = {'y', 'yes'}
    rejected = {'n', 'no'}
    for _ in range(3):
        answer = input(prompt).strip().lower()
        if not answer:
            return default
        if answer in accepted:
            return True
        if answer in rejected:
            return False
        print('Please answer yes or no.')
    print('Cancelled after three unrecognized answers.')
    return False


def _interactive_home() -> tuple[list[str] | None, int]:
    """Turn a no-argument TTY launch into one existing beginner workflow."""
    print('lidarslam_ros2 — rosbag2 to a verified Autoware map')
    print('')
    print('What would you like to do?')
    print('  1. Try the fixed public demo')
    print('  2. Map my own PointCloud2 rosbag2')
    print('  3. Reopen previous map sessions')
    print('  4. Check this installation')
    print('  5. Show every command')
    print('  q. Quit')

    try:
        choice = ''
        for _ in range(3):
            choice = input('Choose 1–5 or q: ').strip().lower()
            if choice in {'1', '2', '3', '4', '5', 'q', 'quit'}:
                break
            print('Please choose 1, 2, 3, 4, 5, or q.')
        else:
            print('No workflow selected.')
            return None, EX_USAGE

        if choice in {'q', 'quit'}:
            print('No changes made.')
            return None, 0
        if choice == '5':
            print('')
            print(render_help())
            return None, 0

        if choice == '4':
            args = ['doctor']
            print('')
            print(f'Next command: {_render_command(args)}')
            print('This check uses no network and writes no files.')
            return args, 0

        if choice == '1':
            work_dir = input(
                'Work directory (Enter for the current directory): '
            ).strip()
            args = ['demo']
            if work_dir:
                args.append(str(Path(work_dir).expanduser()))
            print('')
            print('The demo downloads 517 MB and requires at least 8 GiB free.')
            print(f'Next command: {_render_command(args)}')
            if not _confirm('Download, map, and verify now? [y/N] ', default=False):
                print('No changes made. Run the displayed command when ready.')
                return None, 0
            return args, 0

        if choice == '2':
            bag_dir = input(
                'Rosbag2 directory containing metadata.yaml: '
            ).strip()
            if not bag_dir:
                print('No bag selected; no changes made.')
                return None, 0
            args = ['start', str(Path(bag_dir).expanduser())]
            print('')
            print(f'Next command: {_render_command(args)}')
            print(
                'No launch-file or YAML edits are needed for this guided path.'
            )
            print(
                'Topics, frames, timestamps, profile, and calibration are '
                'reviewed before mapping.'
            )
            if not _confirm('Inspect this bag now? [Y/n] ', default=True):
                print('No changes made. Run the displayed command when ready.')
                return None, 0
            return args, 0

        sessions_root = input(
            'Sessions directory (Enter for ./output): '
        ).strip()
        args = ['sessions']
        if sessions_root:
            args.append(str(Path(sessions_root).expanduser()))
        print('')
        print(f'Next command: {_render_command(args)}')
        if not _confirm('Open the local session catalog now? [Y/n] ', default=True):
            print('No changes made. Run the displayed command when ready.')
            return None, 0
        return args, 0
    except EOFError:
        print('\nInput closed; no changes made.')
        return None, 0
    except KeyboardInterrupt:
        print('\nCancelled; no changes made.')
        return None, EX_INTERRUPT


def command_argv(command: str, args: Sequence[str]) -> list[str]:
    """Build a delegated product command."""
    helper_name = COMMANDS[command]
    helper_path = SCRIPT_DIR / helper_name
    if not helper_path.is_file():
        raise RuntimeError(f'product helper is missing: {helper_path}')
    delegated_args = ['--run', *args] if command == 'start' else list(args)
    return [sys.executable, str(helper_path), *delegated_args]


def _supervisor_depth() -> int:
    """Return the internal dispatch depth, defaulting malformed input safely."""
    try:
        depth = int(os.environ.get(COMMAND_SUPERVISOR_DEPTH_ENV, '0'))
    except ValueError:
        return 0
    return max(depth, 0)


def _signal_command_group(process: subprocess.Popen, signum: int) -> None:
    """Signal an isolated delegated command and every process below it."""
    try:
        os.killpg(process.pid, signum)
    except ProcessLookupError:
        pass


def _run_delegated_command(
    command: list[str],
    *,
    env: dict[str, str],
) -> subprocess.CompletedProcess:
    """Wait for one command without letting Ctrl-C abandon its cleanup."""
    depth = _supervisor_depth()
    owns_process_group = depth == 0
    child_env = env.copy()
    child_env[COMMAND_SUPERVISOR_DEPTH_ENV] = str(depth + 1)
    process: subprocess.Popen | None = None

    def request_termination(signum, _frame):
        raise _CommandTerminationRequested(signum)

    previous_sigterm = signal.signal(signal.SIGTERM, request_termination)
    try:
        process = subprocess.Popen(
            command,
            env=child_env,
            start_new_session=owns_process_group,
        )
        try:
            return_code = process.wait()
            return subprocess.CompletedProcess(command, return_code)
        except KeyboardInterrupt:
            requested_signal = signal.SIGINT
        except _CommandTerminationRequested as exc:
            requested_signal = exc.signum

        # The outermost CLI owns an isolated group and must forward the signal.
        # A nested CLI shares the group its parent already signalled, so sending
        # a second signal would prematurely shorten the child's cleanup window.
        if owns_process_group and process.poll() is None:
            _signal_command_group(process, requested_signal)
        try:
            process.wait(timeout=COMMAND_INTERRUPT_GRACE_SECONDS)
        except (
            KeyboardInterrupt,
            _CommandTerminationRequested,
            subprocess.TimeoutExpired,
        ):
            if process.poll() is None:
                if owns_process_group:
                    _signal_command_group(process, signal.SIGTERM)
                else:
                    process.terminate()
            try:
                process.wait(timeout=COMMAND_INTERRUPT_TERMINATE_SECONDS)
            except (
                KeyboardInterrupt,
                _CommandTerminationRequested,
                subprocess.TimeoutExpired,
            ):
                if process.poll() is None:
                    if owns_process_group:
                        _signal_command_group(process, signal.SIGKILL)
                    else:
                        process.kill()
                process.wait()
        return subprocess.CompletedProcess(
            command,
            128 + requested_signal,
        )
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)


def main(argv: Sequence[str] | None = None) -> int:
    """Dispatch the stable product command surface."""
    args = list(sys.argv[1:] if argv is None else argv)
    if args in (['--help'], ['-h']):
        print(render_help())
        return 0
    if args == ['--help-all']:
        print(render_help(include_all=True))
        return 0
    if args == ['--version']:
        try:
            print(f'lidarslam_ros2 {read_version()}')
        except RuntimeError as exc:
            print(f'error: {exc}', file=sys.stderr)
            return EX_SOFTWARE
        return 0
    if not args:
        if _interactive_terminal():
            args, status = _interactive_home()
            if args is None:
                return status
        else:
            print(render_help(), file=sys.stderr)
            return EX_USAGE

    command = args.pop(0)
    if command not in COMMANDS:
        print(f'error: unknown command: {command}', file=sys.stderr)
        print(f'hint: run "{command_name()} --help".', file=sys.stderr)
        return EX_USAGE

    try:
        child_env = os.environ.copy()
        child_env.pop(HELP_MODE_ENV, None)
        child_env.pop(SUPPORT_MODE_ENV, None)
        if '--help-all' in args:
            if args != ['--help-all']:
                print(
                    'error: --help-all cannot be combined with other options',
                    file=sys.stderr,
                )
                return EX_USAGE
            args = ['--help']
            child_env[HELP_MODE_ENV] = 'all'
        elif '--help' in args or '-h' in args:
            child_env[HELP_MODE_ENV] = 'core'
        child_env['LIDARSLAM_CLI_COMMAND'] = shlex.join([
            command_name(),
            command,
        ])
        if command == 'report':
            child_env[SUPPORT_MODE_ENV] = 'first-map-report'
        completed = _run_delegated_command(
            command_argv(command, args),
            env=child_env,
        )
    except (OSError, RuntimeError) as exc:
        print(f'error: failed to start {command}: {exc}', file=sys.stderr)
        return EX_SOFTWARE
    return completed.returncode


if __name__ == '__main__':
    raise SystemExit(main())
