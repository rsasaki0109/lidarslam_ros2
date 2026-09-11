#!/usr/bin/env python3
"""Compare two local map sessions without inventing a winner or score."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import html
import importlib.util
import json
import os
from pathlib import Path
import re
from string import Template
import sys
import tempfile
from typing import Any, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
COMPARISON_SCHEMA = 'map-session-comparison-v1.schema.json'
COMPARISON_SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/map-session-comparison-v1.schema.json'
)
SESSION_SCHEMA = 'map-session-index-v1.schema.json'
SETUP_SCHEMA = 'sensor-setup-v1.schema.json'
SESSION_NAME = 'session.json'
SESSION_PAGE_NAME = 'session.html'
SETUP_NAME = 'sensor_setup.json'
MAX_JSON_BYTES = 2 * 1024 * 1024
GENERATOR_MARKER = (
    '<meta name="generator" content="lidarslam-session-compare-v1">'
)
QUALITY_CHECK_IDS = (
    'workflow',
    'map_output',
    'verification',
    'evidence',
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


def _safe_stem(value: str) -> str:
    rendered = re.sub(r'[^A-Za-z0-9._-]+', '_', value).strip('._-')
    return rendered[:48] or 'session'


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the two-session comparison command."""
    parser = argparse.ArgumentParser(
        prog=os.environ.get('LIDARSLAM_CLI_COMMAND'),
        description=(
            'Compare quality, setup, and retained evidence for two sessions.'
        ),
    )
    parser.add_argument(
        'left',
        metavar='left_session',
        help='First session bundle containing session.json.',
    )
    parser.add_argument(
        'right',
        metavar='right_session',
        help='Second session bundle containing session.json.',
    )
    parser.add_argument(
        '--help-all',
        action='help',
        help='Show all options (this command has no hidden options).',
    )
    parser.add_argument(
        '--output',
        metavar='<file>',
        help=(
            'Comparison HTML path (default: beside the session bundles).'
        ),
    )
    parser.add_argument(
        '--viewer',
        choices=('browser', 'none'),
        default='browser',
        metavar='{browser,none}',
        help='Open the comparison or only print its path (default: browser).',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='Print comparison JSON without writing or opening HTML.',
    )
    return parser.parse_args(argv)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _read_json(path: Path) -> dict[str, Any] | None:
    if path.is_symlink() or not path.is_file():
        return None
    try:
        if path.stat().st_size > MAX_JSON_BYTES:
            return None
        payload = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError):
        return None
    return payload if isinstance(payload, dict) else None


def _same_path(recorded: str, actual: Path) -> bool:
    try:
        return Path(recorded).expanduser().resolve() == actual.resolve()
    except OSError:
        return False


def _load_session_bundle(
    value: str,
    product_schema,
) -> dict[str, Any]:
    requested = Path(value).expanduser()
    if requested.is_symlink():
        raise ValueError(f'session bundle may not be a symlink: {requested}')
    bundle = requested.resolve()
    if not bundle.is_dir():
        raise ValueError(f'session bundle is not a directory: {bundle}')
    session_path = bundle / SESSION_NAME
    payload = _read_json(session_path)
    if payload is None:
        raise ValueError(
            f'session.json is missing, invalid, symlinked, or oversized: '
            f'{session_path}'
        )
    product_schema.validate_contract(payload, SESSION_SCHEMA)
    if not _same_path(payload['setup_bundle'], bundle):
        raise ValueError(
            f'session setup_bundle does not identify its directory: {bundle}'
        )
    page = bundle / SESSION_PAGE_NAME
    page_path = (
        str(page.resolve())
        if not page.is_symlink() and page.is_file() else None
    )
    return {
        'bundle': bundle,
        'session_path': session_path,
        'page_path': page_path,
        'payload': payload,
    }


def _parameter_files_valid(bundle: Path, setup: dict[str, Any]) -> bool:
    root = bundle.resolve()
    for item in setup['parameters']:
        relative = Path(item['bundle_path'])
        if relative.is_absolute() or '..' in relative.parts:
            return False
        requested = bundle / relative
        cursor = bundle
        for part in relative.parts:
            cursor /= part
            if cursor.is_symlink():
                return False
        candidate = requested.resolve()
        try:
            candidate.relative_to(root)
        except ValueError:
            return False
        if candidate.is_symlink() or not candidate.is_file():
            return False
        try:
            if candidate.stat().st_size != item['size_bytes']:
                return False
            if _sha256(candidate) != item['sha256']:
                return False
        except OSError:
            return False
    return True


def _load_setup_source(
    record: dict[str, Any],
    product_schema,
) -> dict[str, Any]:
    bundle = record['bundle']
    session = record['payload']
    path = bundle / SETUP_NAME
    recorded = session['artifacts']['setup_manifest']
    if not path.exists() and not path.is_symlink():
        return {
            'status': 'missing' if recorded is None else 'invalid',
            'artifact': None,
            'payload': None,
        }
    payload = _read_json(path)
    if payload is None:
        return {'status': 'invalid', 'artifact': str(path), 'payload': None}
    try:
        product_schema.validate_contract(payload, SETUP_SCHEMA)
    except (OSError, RuntimeError, ValueError):
        return {'status': 'invalid', 'artifact': str(path), 'payload': None}
    identities_match = (
        recorded is not None
        and _same_path(recorded, path)
        and _same_path(payload['bundle_path'], bundle)
        and payload['profile'] == session['profile']
        and _same_path(payload['input']['bag_path'], Path(session['bag_path']))
        and _same_path(
            payload['run']['output_dir'],
            Path(session['map_output']),
        )
        and _parameter_files_valid(bundle, payload)
    )
    if not identities_match:
        return {'status': 'invalid', 'artifact': str(path), 'payload': None}
    return {'status': 'valid', 'artifact': str(path), 'payload': payload}


def _available(display: str, values: Sequence[str]) -> dict[str, Any]:
    unique_values = list(dict.fromkeys(str(item) for item in values))
    return {
        'availability': 'available',
        'display': display,
        'value': unique_values,
    }


def _unavailable(status: str) -> dict[str, Any]:
    return {
        'availability': 'unavailable',
        'display': f'Setup evidence {status}.',
        'value': [],
    }


def _comparison(
    row_id: str,
    category: str,
    label: str,
    source: str,
    left: dict[str, Any],
    right: dict[str, Any],
) -> dict[str, Any]:
    if (
        left['availability'] == 'unavailable'
        or right['availability'] == 'unavailable'
    ):
        result = 'unavailable'
    elif left['value'] == right['value']:
        result = 'same'
    else:
        result = 'different'
    return {
        'id': row_id,
        'category': category,
        'label': label,
        'source': source,
        'result': result,
        'left': left,
        'right': right,
    }


def _quality_value(check: dict[str, Any]) -> dict[str, Any]:
    values = [
        f'status={check["status"]}',
        f'observed={check["observed"]}',
        *[f'source={item}' for item in check['source_checks']],
    ]
    return _available(
        f'{check["status"].upper()} · {check["observed"]}',
        values,
    )


def _setup_value(
    source: dict[str, Any],
    values: Sequence[str],
    display: str,
) -> dict[str, Any]:
    if source['status'] != 'valid':
        return _unavailable(source['status'])
    return _available(display, values)


def _input_identity(source: dict[str, Any]) -> dict[str, Any]:
    if source['status'] != 'valid':
        return _unavailable(source['status'])
    payload = source['payload']['input']
    values = [f'metadata={payload["metadata_sha256"]}']
    values.extend(
        f'storage={Path(item["path"]).name}:{item["sha256"]}'
        for item in payload['storage_files']
    )
    return _available(
        f'{len(payload["storage_files"])} storage file(s) · metadata '
        f'{payload["metadata_sha256"][:12]}…',
        values,
    )


def _topics(source: dict[str, Any]) -> dict[str, Any]:
    if source['status'] != 'valid':
        return _unavailable(source['status'])
    topics = source['payload']['topics']
    values = [
        f'{key}={topics[key] if topics[key] is not None else "<none>"}'
        for key in sorted(topics)
    ]
    active = [item for item in values if not item.endswith('=<none>')]
    return _available(' · '.join(active), values)


def _frames(source: dict[str, Any]) -> dict[str, Any]:
    if source['status'] != 'valid':
        return _unavailable(source['status'])
    frames = source['payload']['frames']
    values = [
        f'{key}={frames[key]["id"] or "<none>"} ({frames[key]["source"]})'
        for key in ('base', 'lidar', 'imu')
    ]
    return _available(' · '.join(values), values)


def _calibration(source: dict[str, Any]) -> dict[str, Any]:
    if source['status'] != 'valid':
        return _unavailable(source['status'])
    calibration = source['payload']['calibration']
    values = [f'source={calibration["source"]}']
    for key in (
        'lidar_to_base_quat_xyzw_xyz',
        'imu_to_base_quat_xyzw_xyz',
    ):
        values.append(
            f'{key}=' + json.dumps(calibration[key], separators=(',', ':'))
        )
    return _available(calibration['source'], values)


def _parameters(source: dict[str, Any]) -> dict[str, Any]:
    if source['status'] != 'valid':
        return _unavailable(source['status'])
    records = source['payload']['parameters']
    values = sorted(f'{item["role"]}={item["sha256"]}' for item in records)
    display = (
        'No parameter snapshots.'
        if not values else
        ' · '.join(
            f'{item["role"]} {item["sha256"][:12]}…' for item in records
        )
    )
    return _available(display, values)


def _artifact_value(session: dict[str, Any]) -> dict[str, Any]:
    names = sorted(
        key for key, value in session['artifacts'].items() if value is not None
    )
    display = ' · '.join(names) if names else 'No artifacts recorded.'
    return _available(display, names)


def _snapshot(
    record: dict[str, Any],
    setup: dict[str, Any],
) -> dict[str, Any]:
    payload = record['payload']
    return {
        'name': record['bundle'].name,
        'bundle_path': str(record['bundle']),
        'session_path': str(record['session_path']),
        'page_path': record['page_path'],
        'created_at': payload['created_at'],
        'status': payload['status'],
        'summary': dict(payload['summary']),
        'profile': dict(payload['profile']),
        'verification': dict(payload['verification']),
        'quality': {
            'overall': payload['quality']['overall'],
            'headline': payload['quality']['headline'],
        },
        'setup_source': {
            'status': setup['status'],
            'artifact': setup['artifact'],
        },
    }


def build_comparison(left_path: str, right_path: str) -> dict[str, Any]:
    """Build a schema-valid, descriptive comparison for two sessions."""
    product_schema = _load_script_module(
        'product_schema.py',
        'session_compare_product_schema',
    )
    left_record = _load_session_bundle(left_path, product_schema)
    right_record = _load_session_bundle(right_path, product_schema)
    if left_record['session_path'].resolve() == (
        right_record['session_path'].resolve()
    ):
        raise ValueError('[same-session] choose two different session bundles')
    left_session = left_record['payload']
    right_session = right_record['payload']
    left_setup = _load_setup_source(left_record, product_schema)
    right_setup = _load_setup_source(right_record, product_schema)
    left_checks = {
        item['id']: item for item in left_session['quality']['checks']
    }
    right_checks = {
        item['id']: item for item in right_session['quality']['checks']
    }
    comparisons = [
        _comparison(
            'session_status', 'readiness', 'Session status', 'session.json',
            _available(left_session['status'], [left_session['status']]),
            _available(right_session['status'], [right_session['status']]),
        ),
        _comparison(
            'quality_overall', 'readiness', 'Overall quality', 'session.json',
            _available(
                left_session['quality']['headline'],
                [
                    left_session['quality']['overall'],
                    left_session['quality']['headline'],
                ],
            ),
            _available(
                right_session['quality']['headline'],
                [
                    right_session['quality']['overall'],
                    right_session['quality']['headline'],
                ],
            ),
        ),
    ]
    quality_labels = {
        'workflow': 'Workflow completion',
        'map_output': 'Map output',
        'verification': 'Quality verification check',
        'evidence': 'Evidence integrity',
    }
    for check_id in QUALITY_CHECK_IDS:
        comparisons.append(_comparison(
            f'quality_{check_id}',
            'readiness',
            quality_labels[check_id],
            'session.json',
            _quality_value(left_checks[check_id]),
            _quality_value(right_checks[check_id]),
        ))
    comparisons.extend([
        _comparison(
            'verification', 'readiness', 'Verification mode and result',
            'session.json',
            _available(
                f"{left_session['verification']['mode']} · "
                f"{left_session['verification']['result']}",
                [
                    left_session['verification']['mode'],
                    left_session['verification']['result'],
                ],
            ),
            _available(
                f"{right_session['verification']['mode']} · "
                f"{right_session['verification']['result']}",
                [
                    right_session['verification']['mode'],
                    right_session['verification']['result'],
                ],
            ),
        ),
        _comparison(
            'profile', 'setup', 'Maintained profile', 'sensor_setup.json',
            _setup_value(
                left_setup,
                [left_session['profile']['id']],
                left_session['profile']['label'],
            ),
            _setup_value(
                right_setup,
                [right_session['profile']['id']],
                right_session['profile']['label'],
            ),
        ),
        _comparison(
            'input_identity', 'setup', 'Recorded bag identity',
            'sensor_setup.json',
            _input_identity(left_setup), _input_identity(right_setup),
        ),
        _comparison(
            'topics', 'setup', 'Sensor topics', 'sensor_setup.json',
            _topics(left_setup), _topics(right_setup),
        ),
        _comparison(
            'frames', 'setup', 'Frames', 'sensor_setup.json',
            _frames(left_setup), _frames(right_setup),
        ),
        _comparison(
            'calibration', 'setup', 'Calibration source and transforms',
            'sensor_setup.json',
            _calibration(left_setup), _calibration(right_setup),
        ),
        _comparison(
            'parameters', 'setup', 'Parameter snapshots',
            'sensor_setup.json',
            _parameters(left_setup), _parameters(right_setup),
        ),
        _comparison(
            'recorded_artifacts', 'artifacts', 'Recorded artifacts',
            'session.json',
            _artifact_value(left_session), _artifact_value(right_session),
        ),
    ])
    counts = {
        result: sum(item['result'] == result for item in comparisons)
        for result in ('same', 'different', 'unavailable')
    }
    payload = {
        'schema_version': 1,
        'schema_uri': COMPARISON_SCHEMA_URI,
        'created_at': _utc_now(),
        'policy': {
            'numeric_score': False,
            'winner_selected': False,
            'missing_evidence': 'unavailable_not_inferred',
        },
        'left': _snapshot(left_record, left_setup),
        'right': _snapshot(right_record, right_setup),
        'summary': {**counts, 'total': len(comparisons)},
        'comparisons': comparisons,
    }
    if len(comparisons) != 14 or sum(counts.values()) != 14:
        raise RuntimeError('comparison rows are internally inconsistent')
    product_schema.validate_contract(payload, COMPARISON_SCHEMA)
    return payload


def _status_label(value: str) -> str:
    return value.replace('_', ' ').upper()


def _session_card(side: str, session: dict[str, Any]) -> str:
    page_path = session['page_path']
    if page_path is None:
        control = '<span class="button disabled">Page unavailable</span>'
    else:
        page_uri = html.escape(Path(page_path).as_uri(), quote=True)
        control = (
            f'<a class="button" href="{page_uri}">'
            'Open session</a>'
        )
    return (
        f'<article class="hero-card {html.escape(session["status"])}">'
        f'<span class="side-label">{html.escape(side)}</span>'
        '<div class="badges">'
        f'<strong>{_status_label(session["status"])}</strong>'
        '<strong>QUALITY '
        f'{_status_label(session["quality"]["overall"])}</strong>'
        '</div>'
        f'<h2>{html.escape(session["name"])}</h2>'
        f'<p>{html.escape(session["summary"]["title"])}</p>'
        '<dl><dt>Profile</dt><dd>'
        f'{html.escape(session["profile"]["label"])}</dd>'
        f'<dt>Created</dt><dd>{html.escape(session["created_at"])}</dd>'
        '<dt>Setup evidence</dt><dd>'
        f'{html.escape(session["setup_source"]["status"])}</dd></dl>'
        f'{control}</article>'
    )


def _comparison_row(row: dict[str, Any]) -> str:
    return (
        f'<article class="comparison-row {html.escape(row["result"])}">'
        '<div class="row-label">'
        f'<span>{html.escape(row["source"])}</span>'
        f'<h3>{html.escape(row["label"])}</h3></div>'
        '<div class="value"><b>LEFT</b><p>'
        f'{html.escape(row["left"]["display"])}</p></div>'
        '<div class="value"><b>RIGHT</b><p>'
        f'{html.escape(row["right"]["display"])}</p></div>'
        f'<strong class="result">{row["result"].upper()}</strong>'
        '</article>'
    )


def render_comparison_html(payload: dict[str, Any]) -> str:
    """Render a self-contained, no-winner comparison page."""
    category_titles = {
        'readiness': 'Readiness and quality',
        'setup': 'Recorded sensor setup',
        'artifacts': 'Retained outputs',
    }
    sections = []
    for category in ('readiness', 'setup', 'artifacts'):
        rows = ''.join(
            _comparison_row(item)
            for item in payload['comparisons']
            if item['category'] == category
        )
        sections.append(
            f'<section><h2>{category_titles[category]}</h2>{rows}</section>'
        )
    summary = payload['summary']
    template = Template("""<!doctype html>
<html lang="en"><head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta name="generator" content="lidarslam-session-compare-v1">
  <title>lidarslam session comparison</title>
  <style>
    :root {
      color-scheme: dark; --bg: #071018; --panel: #101c27;
      --line: #283b4d; --text: #eef7ff; --muted: #9db2c7;
      --blue: #66d4ff; --green: #6cdfa7; --yellow: #f2c866;
      --orange: #ff9d5a;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0; color: var(--text); font: 16px/1.5 system-ui, sans-serif;
      background: radial-gradient(circle at 90% 0, #193049, var(--bg) 42%);
    }
    main {
      width: min(1220px, calc(100% - 32px));
      margin: auto; padding: 48px 0 80px;
    }
    .eyebrow {
      color: var(--blue); letter-spacing: .14em;
      text-transform: uppercase; font-size: 12px;
    }
    h1 { margin: 8px 0; font-size: clamp(36px, 6vw, 64px); line-height: 1.05; }
    header > p { color: var(--muted); max-width: 760px; }
    .summary, .badges { display: flex; flex-wrap: wrap; gap: 8px; }
    .summary span, .badges strong, .result {
      border: 1px solid var(--line); border-radius: 999px;
      padding: 5px 9px; font-size: 12px;
    }
    .summary { margin: 18px 0 28px; }
    .heroes { display: grid; grid-template-columns: 1fr 1fr; gap: 18px; }
    .hero-card, .comparison-row {
      background: color-mix(in srgb, var(--panel) 95%, transparent);
      border: 1px solid var(--line); border-radius: 17px;
    }
    .hero-card { padding: 22px; }
    .side-label, .row-label span {
      color: var(--muted); font-size: 11px; letter-spacing: .12em;
      text-transform: uppercase;
    }
    .hero-card h2 { margin: 14px 0 4px; overflow-wrap: anywhere; }
    .hero-card p { color: var(--muted); }
    dl {
      display: grid;
      grid-template-columns: max-content 1fr; gap: 6px 14px;
    }
    dt { color: var(--muted); }
    dd { margin: 0; overflow-wrap: anywhere; }
    .button {
      display: inline-block; margin-top: 12px; padding: 9px 13px;
      border: 1px solid #3e698a; border-radius: 9px;
      color: var(--text); background: #1a344c; text-decoration: none;
      font-weight: 750;
    }
    .button.disabled { color: var(--muted); background: transparent; }
    section > h2 { margin: 38px 0 14px; }
    .comparison-row {
      display: grid; grid-template-columns: 1.05fr 1.4fr 1.4fr auto;
      gap: 18px; align-items: center; padding: 17px 18px; margin-bottom: 10px;
    }
    .row-label h3 { margin: 3px 0 0; }
    .value b { display: none; color: var(--muted); font-size: 11px; }
    .value p { margin: 0; color: var(--muted); overflow-wrap: anywhere; }
    .result { justify-self: end; }
    .same .result { color: var(--green); border-color: #2c654e; }
    .different .result { color: var(--yellow); border-color: #6f5b28; }
    .unavailable .result { color: var(--muted); }
    footer { color: var(--muted); margin-top: 28px; font-size: 13px; }
    @media (max-width: 760px) {
      main { width: min(100% - 20px, 620px); padding-top: 28px; }
      .heroes { grid-template-columns: 1fr; }
      .comparison-row { grid-template-columns: 1fr; gap: 9px; }
      .value b { display: block; }
      .result { justify-self: start; }
    }
  </style>
</head><body><main>
  <header>
    <span class="eyebrow">Evidence-backed comparison</span>
    <h1>Compare map sessions</h1>
    <p>This report describes recorded differences. It does not invent a numeric
      score, infer missing evidence, or select a winner.</p>
    <div class="summary"><span>$same same</span>
      <span>$different different</span>
      <span>$unavailable unavailable</span></div>
  </header>
  <div class="heroes">$left_card$right_card</div>
  $sections
  <footer>Generated $created_at · session.json remains authoritative for status
    and quality; schema-valid sensor_setup.json is used only when its
    identities
    and parameter snapshots still match.</footer>
</main></body></html>
""")
    return template.substitute(
        same=summary['same'],
        different=summary['different'],
        unavailable=summary['unavailable'],
        left_card=_session_card('LEFT', payload['left']),
        right_card=_session_card('RIGHT', payload['right']),
        sections=''.join(sections),
        created_at=html.escape(payload['created_at']),
    )


def _default_output(payload: dict[str, Any]) -> Path:
    left = Path(payload['left']['bundle_path'])
    right = Path(payload['right']['bundle_path'])
    parent = left.parent if left.parent == right.parent else Path.cwd()
    name = (
        f'comparison-{_safe_stem(payload["left"]["name"])}-vs-'
        f'{_safe_stem(payload["right"]["name"])}.html'
    )
    return parent / name


def _generated_comparison(path: Path) -> bool:
    try:
        with path.open('r', encoding='utf-8') as stream:
            return GENERATOR_MARKER in stream.read(4096)
    except (OSError, UnicodeDecodeError):
        return False


def write_comparison_html(path: Path, payload: dict[str, Any]) -> Path:
    """Atomically write only to a new or recognized comparison artifact."""
    requested = path.expanduser()
    if requested.is_symlink():
        raise OSError(f'refusing to replace symlink: {requested}')
    path = requested.resolve()
    if path.exists() and not _generated_comparison(path):
        raise OSError(f'refusing to replace non-comparison file: {path}')
    if not path.parent.is_dir():
        raise OSError(f'comparison parent does not exist: {path.parent}')
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=path.parent,
            prefix=f'.{path.name}.writing-',
            suffix='.tmp',
            delete=False,
        ) as stream:
            stream.write(render_comparison_html(payload))
            temporary = Path(stream.name)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass
    return path


def _open_comparison(path: Path) -> bool:
    viewer = _load_script_module(
        'view_autoware_map.py',
        'session_compare_browser',
    )
    if not viewer.desktop_session_available():
        return False
    return bool(viewer.open_browser(path.as_uri()))


def _render_terminal(payload: dict[str, Any]) -> str:
    summary = payload['summary']
    lines = [
        'Map session comparison (descriptive; no winner or numeric score)',
        f"  Left:  {payload['left']['name']} "
        f"[{payload['left']['status']} / "
        f"{payload['left']['quality']['overall']}]",
        f"  Right: {payload['right']['name']} "
        f"[{payload['right']['status']} / "
        f"{payload['right']['quality']['overall']}]",
        f"  Rows:  {summary['same']} same, {summary['different']} different, "
        f"{summary['unavailable']} unavailable",
    ]
    for row in payload['comparisons']:
        if row['result'] != 'same':
            lines.append(
                f"  [{row['result'].upper()}] {row['label']}: "
                f"{row['left']['display']} | {row['right']['display']}"
            )
    return '\n'.join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    """Compare two sessions and optionally open a local HTML report."""
    args = parse_args(argv)
    try:
        payload = build_comparison(args.left, args.right)
    except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
        code = 'same-session' if '[same-session]' in str(exc) else (
            'session-compare-invalid'
        )
        message = str(exc).replace('[same-session] ', '')
        print(f'error: [{code}] {message}', file=sys.stderr)
        return 2
    if args.json:
        print(json.dumps(payload, indent=2, sort_keys=True))
        return 0
    print(_render_terminal(payload))
    output = Path(args.output) if args.output else _default_output(payload)
    try:
        report = write_comparison_html(output, payload)
    except OSError as exc:
        print(f'error: [comparison-write-failed] {exc}', file=sys.stderr)
        return 2
    print(f'Comparison page: {report}')
    if args.viewer == 'browser':
        try:
            opened = _open_comparison(report)
        except Exception as exc:
            print(
                f'warning: [comparison-open-failed] {exc}',
                file=sys.stderr,
            )
        else:
            if opened:
                print('Session comparison opened in the browser.')
            else:
                print(
                    'No desktop browser detected; '
                    'open the comparison manually.'
                )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
