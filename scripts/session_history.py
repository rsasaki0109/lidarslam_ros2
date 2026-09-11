#!/usr/bin/env python3
"""List and reopen recent local lidarslam map sessions."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import html
import importlib.util
import json
import os
from pathlib import Path
import shlex
from string import Template
import sys
import tempfile
from typing import Any, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
SOURCE_LAYOUT = (REPO_ROOT / 'lidarslam' / 'package.xml').is_file()
WORK_ROOT = REPO_ROOT if SOURCE_LAYOUT else Path.cwd()
CATALOG_SCHEMA = 'map-session-catalog-v1.schema.json'
CATALOG_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/map-session-catalog-v1.schema.json'
)
SESSION_SCHEMA = 'map-session-index-v1.schema.json'
SESSION_NAME = 'session.json'
SESSION_PAGE_NAME = 'session.html'
CATALOG_PAGE_NAME = 'sessions.html'
MAX_SESSION_BYTES = 2 * 1024 * 1024
MAX_LIMIT = 200
STATUSES = (
    'all',
    'running',
    'verified',
    'unverified',
    'action_required',
)


def _load_script_module(script_name: str, module_name: str):
    path = SCRIPT_DIR / script_name
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'failed to load {module_name} from {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace('+00:00', 'Z')


def _positive_limit(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('must be an integer') from exc
    if not 1 <= parsed <= MAX_LIMIT:
        raise argparse.ArgumentTypeError(
            f'must be between 1 and {MAX_LIMIT}'
        )
    return parsed


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the bounded local-history command surface."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'List recent map sessions and reopen their local browser pages.'
        ),
    )
    parser.add_argument(
        'root',
        nargs='?',
        metavar='sessions_root',
        help='Directory containing session bundles (default: ./output).',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all options (this command has no hidden options).',
    )
    parser.add_argument(
        '--status',
        choices=STATUSES,
        default='all',
        metavar='{all,running,verified,unverified,action_required}',
        help='Show only one session state (default: all).',
    )
    parser.add_argument(
        '--limit',
        type=_positive_limit,
        default=20,
        metavar='<count>',
        help=(
            'Maximum recent sessions to show '
            f'(default: 20, max: {MAX_LIMIT}).'
        ),
    )
    parser.add_argument(
        '--viewer',
        choices=('browser', 'none'),
        default='browser',
        metavar='{browser,none}',
        help=(
            'Open the local catalog or only print its path '
            '(default: browser).'
        ),
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print the catalog JSON without writing or opening HTML.',
    )
    return parser.parse_args(argv)


def _read_session(path: Path) -> dict[str, Any] | None:
    """Read a bounded regular file without following a file symlink."""
    if path.is_symlink() or not path.is_file():
        return None
    try:
        if path.stat().st_size > MAX_SESSION_BYTES:
            return None
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _session_entry(
    bundle: Path,
    product_schema,
) -> dict[str, Any] | None:
    """Return a catalog projection of one schema-valid session."""
    session_path = bundle / SESSION_NAME
    payload = _read_session(session_path)
    if payload is None:
        return None
    try:
        product_schema.validate_contract(payload, SESSION_SCHEMA)
    except (OSError, RuntimeError, ValueError):
        return None
    page = bundle / SESSION_PAGE_NAME
    page_path = (
        str(page.resolve())
        if not page.is_symlink() and page.is_file() else None
    )
    action = payload['actions'][0] if payload['actions'] else None
    return {
        'bundle_path': str(bundle.resolve()),
        'session_name': bundle.name,
        'session_path': str(session_path.resolve()),
        'page_path': page_path,
        'created_at': payload['created_at'],
        'status': payload['status'],
        'profile': dict(payload['profile']),
        'verification': dict(payload['verification']),
        'quality': {
            'overall': payload['quality']['overall'],
            'headline': payload['quality']['headline'],
        },
        'summary': dict(payload['summary']),
        'bag_path': payload['bag_path'],
        'map_output': payload['map_output'],
        'recommended_action': dict(action) if action is not None else None,
    }


def _created_at_key(entry: dict[str, Any]) -> float:
    rendered = entry['created_at'].replace('Z', '+00:00')
    try:
        return datetime.fromisoformat(rendered).timestamp()
    except ValueError:
        return 0.0


def build_catalog(
    root: Path,
    *,
    status: str,
    limit: int,
    allow_missing: bool = False,
) -> dict[str, Any]:
    """Build a bounded catalog from direct, non-symlink child bundles."""
    root = root.expanduser().resolve()
    if root.exists() and not root.is_dir():
        raise ValueError(f'session root is not a directory: {root}')
    if not root.exists() and not allow_missing:
        raise FileNotFoundError(f'session root does not exist: {root}')
    product_schema = _load_script_module(
        'product_schema.py',
        'session_history_product_schema',
    )
    candidates = 0
    valid_entries: list[dict[str, Any]] = []
    skipped_invalid = 0
    children = sorted(root.iterdir()) if root.is_dir() else []
    for bundle in children:
        if bundle.is_symlink() or not bundle.is_dir():
            continue
        session_path = bundle / SESSION_NAME
        if not session_path.exists() and not session_path.is_symlink():
            continue
        candidates += 1
        entry = _session_entry(bundle, product_schema)
        if entry is None:
            skipped_invalid += 1
            continue
        valid_entries.append(entry)
    valid_entries.sort(key=_created_at_key, reverse=True)
    filtered = [
        entry
        for entry in valid_entries
        if status == 'all' or entry['status'] == status
    ]
    shown = filtered[:limit]
    payload = {
        'schema_version': 1,
        'schema_uri': CATALOG_SCHEMA_URI,
        'created_at': _utc_now(),
        'source': {
            'root': str(root),
            'scan_depth': 1,
            'follows_symlinks': False,
        },
        'filter': {'status': status, 'limit': limit},
        'summary': {
            'candidates': candidates,
            'valid': len(valid_entries),
            'skipped_invalid': skipped_invalid,
            'displayed': len(shown),
        },
        'sessions': shown,
    }
    product_schema.validate_contract(payload, CATALOG_SCHEMA)
    return payload


def _status_label(status: str) -> str:
    return {
        'running': 'RUNNING',
        'verified': 'VERIFIED',
        'unverified': 'UNVERIFIED',
        'action_required': 'ACTION REQUIRED',
    }[status]


def _quality_label(status: str) -> str:
    return {
        'pending': 'PENDING',
        'pass': 'PASS',
        'not_verified': 'NOT VERIFIED',
        'action_required': 'ACTION REQUIRED',
        'unavailable': 'UNAVAILABLE',
    }[status]


def _product_command_prefix(command: str) -> str:
    configured = os.environ.get('LIDARSLAM_CLI_COMMAND')
    if not configured:
        return f'./scripts/lidarslam {command}'
    try:
        parts = shlex.split(configured)
    except ValueError:
        return f'lidarslam-map {command}'
    if parts and parts[-1] == 'sessions':
        parts.pop()
    parts.append(command)
    return shlex.join(parts)


def _compare_command_prefix() -> str:
    return _product_command_prefix('compare')


def _support_command(bundle_path: str) -> str:
    return f'{_product_command_prefix("support")} {shlex.quote(bundle_path)}'


def _first_map_report_command(bundle_path: str) -> str:
    return (
        f'{_product_command_prefix("report")} '
        f'{shlex.quote(bundle_path)}'
    )


def _terminal_summary(entry: dict[str, Any]) -> str | None:
    """Return one compact retained-session detail for terminal users."""
    if entry.get('status') == 'verified':
        return None
    summary = entry.get('summary')
    if not isinstance(summary, dict):
        return None
    title = summary.get('title')
    message = summary.get('message')
    values = [value for value in (title, message) if isinstance(value, str)]
    if not values:
        return None
    # Session text can contain operator-controlled values. Keep the terminal
    # record one-line and prevent retained newlines from imitating output.
    return ' — '.join(
        ' '.join(value.replace('\r', ' ').replace('\n', ' ').split())
        for value in values
    )


def _render_session_card(entry: dict[str, Any]) -> str:
    status = html.escape(entry['status'])
    quality = html.escape(entry['quality']['overall'])
    page_path = entry['page_path']
    if page_path is None:
        open_control = '<span class="button disabled">Page unavailable</span>'
    else:
        page_uri = Path(page_path).as_uri()
        open_control = (
            f'<a class="button" href="{html.escape(page_uri, quote=True)}">'
            'Open session</a>'
        )
    action = entry['recommended_action']
    action_html = ''
    if action is not None:
        action_html = (
            '<details><summary>'
            f'{html.escape(action["label"])}'
            '</summary><code>'
            f'{html.escape(action["command"])}'
            '</code></details>'
        )
    paths_html = (
        '<details><summary>Show local paths</summary>'
        '<span class="path-label">Bag</span>'
        f'<code>{html.escape(entry["bag_path"])}</code>'
        '<span class="path-label">Map</span>'
        f'<code>{html.escape(entry["map_output"])}</code>'
        '</details>'
    )
    compare_argument = html.escape(
        shlex.quote(entry['bundle_path']),
        quote=True,
    )
    support_command = _support_command(entry['bundle_path'])
    report_html = ''
    if entry['quality']['overall'] == 'pass':
        report_command = _first_map_report_command(entry['bundle_path'])
        report_html = (
            '<details class="support-action report-action">'
            '<summary>Prepare a first-map report</summary>'
            '<p>Revalidate the retained receipt, then review and attach only '
            'the privacy-bounded JSON receipt.</p>'
            f'<code>{html.escape(report_command)}</code>'
            '<button class="copy-report button" type="button" '
            f'data-command="{html.escape(report_command, quote=True)}">'
            'Copy report command</button></details>'
        )
    support_html = (
        '<details class="support-action"><summary>Get support</summary>'
        '<p>Create a privacy-first ZIP to review and attach to an issue.</p>'
        f'<code>{html.escape(support_command)}</code>'
        '<button class="copy-support button" type="button" '
        f'data-command="{html.escape(support_command, quote=True)}">'
        'Copy support command</button></details>'
    )
    return (
        '<article class="session-card">'
        '<div class="card-top"><div class="badges">'
        f'<span class="badge {status}">{_status_label(entry["status"])}</span>'
        f'<span class="badge quality {quality}">Quality '
        f'{_quality_label(entry["quality"]["overall"])}</span>'
        '</div>'
        f'<time>{html.escape(entry["created_at"])}</time></div>'
        '<label class="compare-select">'
        '<input class="compare-choice" type="checkbox" '
        f'data-argument="{compare_argument}">'
        '<span>Select for comparison</span></label>'
        f'<h2>{html.escape(entry["summary"]["title"])}</h2>'
        f'<p>{html.escape(entry["summary"]["message"])}</p>'
        '<dl>'
        '<div><dt>Session</dt><dd>'
        f'{html.escape(entry["session_name"])}</dd></div>'
        '<div><dt>Profile</dt><dd>'
        f'{html.escape(entry["profile"]["label"])}</dd></div>'
        '<div><dt>Bag</dt><dd>'
        f'{html.escape(Path(entry["bag_path"]).name)}</dd></div>'
        '</dl>'
        f'<div class="card-actions">{open_control}</div>'
        f'{action_html}{report_html}{support_html}{paths_html}</article>'
    )


def render_catalog_html(payload: dict[str, Any]) -> str:
    """Render a responsive, dependency-free local session catalog."""
    cards = ''.join(_render_session_card(item) for item in payload['sessions'])
    if not cards:
        cards = (
            '<section class="empty"><h2>No matching sessions</h2>'
            '<p>Create a map with <code>lidarslam-map start '
            '&lt;rosbag2_dir&gt;'
            '</code>, or choose another status filter.</p></section>'
        )
    summary = payload['summary']
    skipped = ''
    if summary['skipped_invalid']:
        skipped = (
            f'<span>{summary["skipped_invalid"]} invalid skipped</span>'
        )
    template = Template("""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>lidarslam map sessions</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #081018; --panel: #101b26; --line: #26384a;
      --text: #eef6ff; --muted: #9eb1c5; --blue: #66d4ff;
      --green: #67dea7; --yellow: #f4ca64; --orange: #ff9c58;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: radial-gradient(
        circle at 90% 0, #182b42, var(--bg) 42%
      );
      color: var(--text); font: 16px/1.5 system-ui, sans-serif;
    }
    main {
      width: min(1180px, calc(100% - 32px));
      margin: 0 auto; padding: 48px 0 72px;
    }
    header { margin-bottom: 30px; }
    .eyebrow {
      color: var(--blue); letter-spacing: .14em;
      text-transform: uppercase; font-size: 12px;
    }
    h1 { margin: 8px 0; font-size: clamp(34px, 6vw, 64px); line-height: 1.05; }
    header p { color: var(--muted); max-width: 720px; }
    .summary { display: flex; flex-wrap: wrap; gap: 9px; margin-top: 18px; }
    .summary span, .badge {
      border: 1px solid var(--line); border-radius: 999px; padding: 5px 10px;
      font-size: 12px; font-weight: 750; text-transform: uppercase;
    }
    .sessions {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 18px;
    }
    .session-card, .empty {
      background: color-mix(in srgb, var(--panel) 94%, transparent);
      border: 1px solid var(--line); border-radius: 18px; padding: 22px;
    }
    .compare-select {
      display: inline-flex; align-items: center; gap: 9px; margin-top: 16px;
      color: var(--text); font-weight: 700; cursor: pointer;
    }
    .compare-select input {
      width: 18px; height: 18px; accent-color: var(--blue);
    }
    .card-top {
      display: flex; justify-content: space-between;
      gap: 12px; align-items: start;
    }
    .badges { display: flex; flex-wrap: wrap; gap: 7px; }
    .badge.verified, .badge.pass {
      color: var(--green); border-color: #2a624d;
    }
    .badge.running, .badge.pending {
      color: var(--blue); border-color: #2a6178;
    }
    .badge.unverified, .badge.not_verified {
      color: var(--yellow); border-color: #68572e;
    }
    .badge.action_required { color: var(--orange); border-color: #754427; }
    .badge.unavailable { color: var(--muted); }
    time { color: var(--muted); font: 12px ui-monospace, monospace; }
    h2 { margin: 18px 0 7px; line-height: 1.2; }
    p { color: var(--muted); }
    dl { display: grid; gap: 9px; margin: 20px 0; }
    dl div { border-top: 1px solid var(--line); padding-top: 9px; }
    dt {
      color: var(--muted); font-size: 11px;
      text-transform: uppercase; letter-spacing: .1em;
    }
    dd { margin: 4px 0 0; overflow-wrap: anywhere; }
    .card-actions { margin-top: 18px; }
    .button {
      display: inline-block; border: 1px solid #3c6688; border-radius: 9px;
      background: #19324a; color: var(--text); padding: 9px 13px;
      text-decoration: none; font-weight: 750;
    }
    .button.disabled {
      color: var(--muted); background: transparent;
      border-color: var(--line);
    }
    button.button { cursor: pointer; }
    button.button:disabled { cursor: not-allowed; opacity: .55; }
    .compare-tray {
      position: sticky; top: 14px; z-index: 2; margin: 0 0 22px;
      padding: 16px 18px; border: 1px solid #3c6688; border-radius: 14px;
      background: color-mix(in srgb, #12263a 96%, transparent);
      box-shadow: 0 16px 42px #0008;
    }
    .compare-tray strong { display: block; }
    .compare-tray p { margin: 3px 0 10px; }
    .compare-tray code { margin: 0 0 12px; }
    details { margin-top: 16px; }
    summary { cursor: pointer; color: var(--muted); }
    .support-action p { margin-bottom: 8px; }
    .support-action .button { margin-top: 12px; }
    .path-label {
      display: block; margin-top: 10px; color: var(--muted);
      font-size: 11px; text-transform: uppercase; letter-spacing: .1em;
    }
    code {
      display: block; margin-top: 9px;
      color: var(--green); overflow-wrap: anywhere;
    }
    footer {
      margin-top: 28px; color: var(--muted);
      font-size: 13px; overflow-wrap: anywhere;
    }
    @media (max-width: 760px) {
      main { width: min(100% - 20px, 620px); padding-top: 28px; }
      .sessions { grid-template-columns: 1fr; }
      .card-top { display: block; }
      time { display: block; margin-top: 10px; }
    }
  </style>
</head>
<body><main>
  <header>
    <span class="eyebrow">Local session history</span>
    <h1>Recent map sessions</h1>
    <p>Reopen the durable page for a previous run without searching
      through output folders.</p>
    <div class="summary">
      <span>$displayed shown</span><span>$valid valid</span>
      <span>Filter: $filter</span>$skipped
    </div>
  </header>
  <section class="compare-tray" data-prefix="$compare_prefix">
    <strong>Compare two sessions ·
      <span id="compare-count">0 / 2</span></strong>
    <p id="compare-help">Select two session cards to build a safe,
      copy-ready comparison command.</p>
    <code id="compare-command" hidden></code>
    <button class="button" id="copy-compare" type="button" disabled>
      Copy compare command</button>
  </section>
  <section class="sessions">$cards</section>
  <footer>Root: $root<br>Generated $created_at · direct child bundles only
    · symlinks not followed.</footer>
</main>
<script>
(() => {
  const choices = Array.from(document.querySelectorAll('.compare-choice'));
  const tray = document.querySelector('.compare-tray');
  const count = document.querySelector('#compare-count');
  const help = document.querySelector('#compare-help');
  const command = document.querySelector('#compare-command');
  const copy = document.querySelector('#copy-compare');
  const selected = () => choices.filter((item) => item.checked);
  const refresh = () => {
    const active = selected();
    count.textContent = active.length + ' / 2';
    const ready = active.length === 2;
    command.hidden = !ready;
    copy.disabled = !ready;
    help.textContent = ready
      ? 'Ready. Review both sessions side by side; no winner is inferred.'
      : 'Select two session cards to build a safe, copy-ready command.';
    command.textContent = ready
      ? tray.dataset.prefix + ' ' + active.map(
          (item) => item.dataset.argument
        ).join(' ')
      : '';
  };
  choices.forEach((choice) => choice.addEventListener('change', () => {
    if (choice.checked && selected().length > 2) {
      choice.checked = false;
      help.textContent = 'Only two sessions can be compared at once.';
      return;
    }
    refresh();
  }));
  copy.addEventListener('click', async () => {
    try {
      await navigator.clipboard.writeText(command.textContent);
      copy.textContent = 'Copied';
    } catch (error) {
      copy.textContent = 'Select and copy the command above';
    }
  });
  document.querySelectorAll('.copy-support, .copy-report').forEach((button) => {
    button.addEventListener('click', async () => {
      try {
        await navigator.clipboard.writeText(button.dataset.command);
        button.textContent = 'Copied';
      } catch (error) {
        button.textContent = 'Select and copy the command above';
      }
    });
  });
  refresh();
})();
</script>
</body></html>
""")
    return template.substitute(
        displayed=summary['displayed'],
        valid=summary['valid'],
        filter=html.escape(payload['filter']['status']),
        skipped=skipped,
        cards=cards,
        compare_prefix=html.escape(_compare_command_prefix(), quote=True),
        root=html.escape(payload['source']['root']),
        created_at=html.escape(payload['created_at']),
    )


def write_catalog_html(root: Path, payload: dict[str, Any]) -> Path:
    """Atomically replace the generated catalog without following a symlink."""
    target = root / CATALOG_PAGE_NAME
    if target.is_symlink():
        raise OSError(f'refusing to replace symlink: {target}')
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=root,
            prefix=f'.{CATALOG_PAGE_NAME}.writing-',
            suffix='.tmp',
            delete=False,
        ) as stream:
            stream.write(render_catalog_html(payload))
            temporary = Path(stream.name)
        os.replace(temporary, target)
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
    return target


def _open_catalog(path: Path) -> bool:
    viewer = _load_script_module(
        'view_autoware_map.py',
        'session_history_browser',
    )
    if not viewer.desktop_session_available():
        return False
    return bool(viewer.open_browser(path.as_uri()))


def _render_terminal(payload: dict[str, Any]) -> str:
    summary = payload['summary']
    lines = [
        f"Recent map sessions: {summary['displayed']} shown "
        f"({summary['valid']} valid, "
        f"{summary['skipped_invalid']} invalid skipped)",
    ]
    if not payload['sessions']:
        lines.append(
            'No matching sessions. Create one with: '
            'lidarslam-map start <rosbag2_dir>'
        )
        return '\n'.join(lines)
    for entry in payload['sessions']:
        lines.extend([
            '',
            f"[{_status_label(entry['status'])} / "
            f"{_quality_label(entry['quality']['overall'])}] "
            f"{entry['created_at']}",
            f"  {entry['profile']['label']}",
            f"  {entry['bundle_path']}",
        ])
        detail = _terminal_summary(entry)
        if detail is not None:
            lines.append(f'  Details: {detail}')
        if entry['page_path'] is not None:
            lines.append(f"  Open: {entry['page_path']}")
        if entry['quality']['overall'] == 'pass':
            lines.append(
                '  Report: '
                f"{_first_map_report_command(entry['bundle_path'])}"
            )
        elif entry['recommended_action'] is not None:
            lines.append(
                '  Next: '
                f"{entry['recommended_action']['command']}"
            )
    return '\n'.join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    """Build, print, and optionally open the local session history."""
    args = parse_args(argv)
    explicit_root = args.root is not None
    root = Path(args.root) if explicit_root else WORK_ROOT / 'output'
    try:
        payload = build_catalog(
            root,
            status=args.status,
            limit=args.limit,
            allow_missing=not explicit_root,
        )
    except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
        print(f'error: [session-history-unavailable] {exc}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0
    print(_render_terminal(payload))
    resolved_root = Path(payload['source']['root'])
    if not resolved_root.is_dir():
        return 0
    try:
        report = write_catalog_html(resolved_root, payload)
    except OSError as exc:
        print(
            f'warning: [session-history-write-failed] {exc}',
            file=sys.stderr,
        )
        return 0
    print(f'Catalog page: {report}')
    if args.viewer == 'browser':
        try:
            opened = _open_catalog(report)
        except Exception as exc:
            print(
                f'warning: [session-history-open-failed] {exc}',
                file=sys.stderr,
            )
        else:
            if opened:
                print('Session history opened in the browser.')
            else:
                print(
                    'No desktop browser detected; '
                    'open the catalog manually.'
                )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
