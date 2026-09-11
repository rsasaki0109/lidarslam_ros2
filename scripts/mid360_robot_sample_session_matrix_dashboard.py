#!/usr/bin/env python3
"""Static HTML dashboard for the MID-360 sample-session QA matrix."""

from __future__ import annotations

import html
import json
from pathlib import Path
from typing import Any


MATRIX_HTML = 'mid360_robot_sample_session_matrix.html'
MATRIX_JSON = 'mid360_robot_sample_session_matrix.json'


def load_matrix_report(output_dir: Path) -> dict[str, Any]:
    """Load the matrix JSON report from an output directory."""
    path = output_dir / MATRIX_JSON
    return json.loads(path.read_text(encoding='utf-8'))


def write_matrix_dashboard(
    output_dir: Path,
    output_path: Path | None = None,
    report: dict[str, Any] | None = None,
) -> Path:
    """Render and write the matrix dashboard HTML."""
    output_dir = output_dir.resolve()
    output_path = output_path or (output_dir / MATRIX_HTML)
    payload = report or load_matrix_report(output_dir)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(render_matrix_dashboard(payload), encoding='utf-8')
    return output_path


def render_matrix_dashboard(report: dict[str, Any]) -> str:
    """Render a self-contained HTML matrix dashboard."""
    status = str(report.get('status') or 'UNKNOWN').upper()
    scenarios = report.get('scenarios') or []
    output_dir = Path(str(report.get('output_dir') or '.')).expanduser()
    return '\n'.join([
        '<!doctype html>',
        '<html lang="en">',
        '<head>',
        '<meta charset="utf-8">',
        '<meta name="viewport" content="width=device-width, initial-scale=1">',
        '<title>MID-360 Sample Session Matrix</title>',
        '<style>',
        _css(),
        '</style>',
        '</head>',
        '<body>',
        '<main>',
        '<section class="hero">',
        '<div>',
        '<p class="eyebrow">MID-360 Sample Session Matrix</p>',
        '<h1>Scenario QA</h1>',
        f'<p class="subtle">{_h(report.get("output_dir") or "output unavailable")}</p>',
        '</div>',
        f'<div class="status {status.lower()}">{_h(status)}</div>',
        '</section>',
        _summary_cards(report),
        _scenario_cards(scenarios, output_dir),
        '<section>',
        '<h2>Matrix Detail</h2>',
        _scenario_table(scenarios, output_dir),
        '</section>',
        _errors_panel(scenarios),
        '</main>',
        '</body>',
        '</html>',
    ])


def _summary_cards(report: dict[str, Any]) -> str:
    counts = report.get('counts') or {}
    cards = [
        _metric_card('Matched', counts.get('matched', 0)),
        _metric_card('Mismatched', counts.get('mismatched', 0)),
        _metric_card('Total', counts.get('total', 0)),
    ]
    return '<section class="metrics">' + ''.join(cards) + '</section>'


def _scenario_cards(scenarios: list[dict[str, Any]], output_dir: Path) -> str:
    if not scenarios:
        return '<section><p class="empty">No scenarios found.</p></section>'
    cards = []
    for item in scenarios:
        observed = str(item.get('observed_status') or 'UNKNOWN').upper()
        match = bool(item.get('matched_expected'))
        link = _relative_href(output_dir, item.get('dashboard_html_path', ''))
        cards.append(
            '<article class="scenario-card">'
            f'<div class="card-head"><h3>{_h(item.get("scenario"))}</h3>'
            f'<span class="pill {observed.lower()}">{_h(observed)}</span></div>'
            f'<p>Expected <strong>{_h(item.get("expected_status"))}</strong></p>'
            f'<p>Match <strong>{_h("yes" if match else "no")}</strong></p>'
            f'<a href="{_h(link)}">Open scenario dashboard</a>'
            '</article>'
        )
    return '<section class="scenario-grid">' + ''.join(cards) + '</section>'


def _scenario_table(scenarios: list[dict[str, Any]], output_dir: Path) -> str:
    if not scenarios:
        return '<p class="empty">No matrix rows found.</p>'
    rows = []
    for item in scenarios:
        observed = str(item.get('observed_status') or 'UNKNOWN').upper()
        match = 'yes' if item.get('matched_expected') else 'no'
        dashboard = _relative_href(output_dir, item.get('dashboard_html_path', ''))
        readiness = _relative_href(output_dir, item.get('readiness_json_path', ''))
        rows.append(
            '<tr>'
            f'<td>{_h(item.get("scenario"))}</td>'
            f'<td>{_h(item.get("expected_status"))}</td>'
            f'<td><span class="pill {observed.lower()}">{_h(observed)}</span></td>'
            f'<td>{_h(match)}</td>'
            f'<td><a href="{_h(dashboard)}">dashboard</a></td>'
            f'<td><a href="{_h(readiness)}">readiness</a></td>'
            '</tr>'
        )
    return (
        '<table>'
        '<thead><tr>'
        '<th>Scenario</th><th>Expected</th><th>Observed</th><th>Match</th>'
        '<th>Dashboard</th><th>Readiness</th>'
        '</tr></thead>'
        '<tbody>'
        + ''.join(rows)
        + '</tbody></table>'
    )


def _errors_panel(scenarios: list[dict[str, Any]]) -> str:
    errors = [item for item in scenarios if item.get('error')]
    if not errors:
        return ''
    rows = ''.join(
        f'<li><strong>{_h(item.get("scenario"))}</strong>: {_h(item.get("error"))}</li>'
        for item in errors
    )
    return '<section class="errors"><h2>Errors</h2><ul>' + rows + '</ul></section>'


def _metric_card(label: str, value: Any) -> str:
    return f'<article class="metric"><h2>{_h(label)}</h2><p>{_h(value)}</p></article>'


def _relative_href(output_dir: Path, target: Any) -> str:
    if not target:
        return '#'
    target_path = Path(str(target)).expanduser()
    try:
        return str(target_path.resolve().relative_to(output_dir.resolve()))
    except Exception:
        return str(target)


def _h(value: Any) -> str:
    return html.escape('' if value is None else str(value))


def _css() -> str:
    return """
:root {
  color-scheme: light;
  font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  background: #f5f7fb;
  color: #172033;
}
body {
  margin: 0;
}
main {
  max-width: 1180px;
  margin: 0 auto;
  padding: 28px;
}
.hero {
  display: flex;
  align-items: flex-start;
  justify-content: space-between;
  gap: 24px;
  padding: 28px 0 22px;
  border-bottom: 1px solid #d9e0eb;
}
.eyebrow {
  margin: 0 0 8px;
  font-size: 12px;
  font-weight: 700;
  letter-spacing: 0;
  text-transform: uppercase;
  color: #526070;
}
h1, h2, h3, p {
  margin-top: 0;
}
h1 {
  margin-bottom: 8px;
  font-size: 36px;
  line-height: 1.12;
}
h2 {
  font-size: 18px;
}
h3 {
  font-size: 16px;
}
.subtle {
  color: #526070;
  overflow-wrap: anywhere;
}
.status {
  min-width: 96px;
  padding: 10px 14px;
  border-radius: 6px;
  text-align: center;
  font-weight: 800;
  border: 1px solid transparent;
}
.status.pass,
.pill.pass {
  background: #dff4e8;
  color: #14683b;
  border-color: #9ed9b8;
}
.status.fail,
.pill.fail,
.pill.error {
  background: #ffe3e3;
  color: #a12121;
  border-color: #f0a4a4;
}
.pill.warn {
  background: #fff2d4;
  color: #8b5a00;
  border-color: #e7c36a;
}
.metrics,
.scenario-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(190px, 1fr));
  gap: 14px;
  margin: 22px 0;
}
.metric,
.scenario-card {
  background: #ffffff;
  border: 1px solid #d9e0eb;
  border-radius: 8px;
  padding: 16px;
}
.metric p {
  margin: 0;
  font-size: 28px;
  font-weight: 800;
}
.card-head {
  display: flex;
  justify-content: space-between;
  gap: 12px;
}
.pill {
  align-self: flex-start;
  padding: 4px 8px;
  border: 1px solid transparent;
  border-radius: 999px;
  font-size: 12px;
  font-weight: 800;
}
a {
  color: #2458c2;
  font-weight: 700;
}
table {
  width: 100%;
  border-collapse: collapse;
  background: #ffffff;
  border: 1px solid #d9e0eb;
  border-radius: 8px;
  overflow: hidden;
}
th,
td {
  padding: 11px 12px;
  text-align: left;
  border-bottom: 1px solid #edf1f7;
  overflow-wrap: anywhere;
}
th {
  background: #e9eef6;
  color: #334155;
  font-size: 13px;
}
.errors {
  margin-top: 22px;
  padding: 16px;
  border: 1px solid #f0a4a4;
  border-radius: 8px;
  background: #fff1f1;
}
.empty {
  color: #526070;
}
@media (max-width: 720px) {
  main {
    padding: 18px;
  }
  .hero {
    display: block;
  }
  .status {
    margin-top: 16px;
    display: inline-block;
  }
  table {
    display: block;
    overflow-x: auto;
  }
}
"""


if __name__ == "__main__":
    raise SystemExit("mid360_robot_sample_session_matrix_dashboard is a library module; import it instead of running it directly.")
