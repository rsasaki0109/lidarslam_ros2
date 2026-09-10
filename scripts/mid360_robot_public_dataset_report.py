#!/usr/bin/env python3
"""Comparison reports for public MID-360 dataset intake results."""

from __future__ import annotations

import html
import json
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.mid360_robot_public_datasets import (
    PUBLIC_DATASET_INTAKE_JSON,
    PublicDataset,
    get_public_dataset,
    public_dataset_registry,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json


PUBLIC_DATASET_REPORT_JSON = 'mid360_robot_public_dataset_report.json'
PUBLIC_DATASET_REPORT_MARKDOWN = 'mid360_robot_public_dataset_report.md'
PUBLIC_DATASET_REPORT_HTML = 'mid360_robot_public_dataset_report.html'
PUBLIC_RKO_SWEEP_JSON = 'mid360_robot_public_rko_sweep.json'


@dataclass(frozen=True)
class PublicDatasetArtifactPaths:
    """Known artifacts for one public MID-360 dataset run."""

    intake_json: Path
    recording_check_json: Path
    readiness_json: Path
    run_plan_json: Path

    def to_dict(self) -> dict[str, str]:
        return {key: str(value) for key, value in asdict(self).items()}


class PublicDatasetReportBuilder:
    """Build comparison rows from public dataset intake artifacts."""

    def __init__(
        self,
        dataset_root: Path,
        output_root: Path,
        dataset_ids: list[str] | None = None,
        map_sweep_paths: list[Path] | None = None,
    ) -> None:
        self._dataset_root = dataset_root.expanduser().resolve()
        self._output_root = output_root.expanduser().resolve()
        self._dataset_ids = dataset_ids or [dataset.id for dataset in public_dataset_registry()]
        self._map_sweep_paths = (
            [path.expanduser().resolve() for path in map_sweep_paths]
            if map_sweep_paths is not None
            else _discover_map_sweep_paths(self._output_root)
        )
        self._map_sweeps = [_load_json(path) for path in self._map_sweep_paths]

    def build_report(self) -> dict[str, Any]:
        """Build a report for selected public MID-360 datasets."""
        rows = [self._build_row(dataset_id) for dataset_id in self._dataset_ids]
        return {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': self._overall_status(rows),
            'dataset_root': str(self._dataset_root),
            'output_root': str(self._output_root),
            'datasets': rows,
            'counts': self._counts(rows),
        }

    def _build_row(self, dataset_id: str) -> dict[str, Any]:
        dataset = get_public_dataset(dataset_id)
        paths = self._paths(dataset)
        intake = _load_json(paths.intake_json)
        recording = _load_json(paths.recording_check_json)
        readiness = _load_json(paths.readiness_json)
        run_plan = _load_json(paths.run_plan_json)

        status = _first_status(recording, readiness, intake) or 'MISSING'
        selected_topics = _first_mapping(recording, readiness, key='selected_topics')
        diagnostics = readiness.get('bag_diagnostics') or {}
        topic_diagnostics = diagnostics.get('topics') or {}
        pointcloud_diagnostics = topic_diagnostics.get('pointcloud') or {}
        imu_diagnostics = topic_diagnostics.get('imu') or {}
        warnings = _collect_warnings(recording, readiness)

        return {
            'dataset_id': dataset.id,
            'title': dataset.title,
            'source_url': dataset.source_url,
            'status': status,
            'ready_for_mid360_launch': bool(run_plan.get('ready_for_mid360_launch')),
            'selected_bag_path': intake.get('selected_bag_path') or recording.get('bag_path', ''),
            'selected_topics': {
                'pointcloud': selected_topics.get('pointcloud', ''),
                'imu': selected_topics.get('imu', ''),
            },
            'rates_hz': {
                'pointcloud': _maybe_float(pointcloud_diagnostics.get('metadata_rate_hz')),
                'imu': _maybe_float(imu_diagnostics.get('metadata_rate_hz')),
            },
            'sampled_frames': {
                'pointcloud': pointcloud_diagnostics.get('sampled_frame_ids') or [],
                'imu': imu_diagnostics.get('sampled_frame_ids') or [],
            },
            'message_counts': {
                'pointcloud': pointcloud_diagnostics.get('metadata_message_count', 0),
                'imu': imu_diagnostics.get('metadata_message_count', 0),
            },
            'sample_reader_available': bool((diagnostics.get('sample_reader') or {}).get('available')),
            'warnings': warnings,
            'artifact_paths': paths.to_dict(),
            'artifact_exists': {
                'intake_json': paths.intake_json.is_file(),
                'recording_check_json': paths.recording_check_json.is_file(),
                'readiness_json': paths.readiness_json.is_file(),
                'run_plan_json': paths.run_plan_json.is_file(),
            },
            'map_command_shell': run_plan.get('dogfood_command_shell', ''),
            'map_validation': _map_validation_for_dataset(
                dataset_id=dataset.id,
                manifests=self._map_sweeps,
                manifest_paths=self._map_sweep_paths,
            ),
        }

    def _paths(self, dataset: PublicDataset) -> PublicDatasetArtifactPaths:
        dataset_dir = self._dataset_root / dataset.id
        output_dir = self._output_root / dataset.id
        return PublicDatasetArtifactPaths(
            intake_json=dataset_dir / PUBLIC_DATASET_INTAKE_JSON,
            recording_check_json=output_dir / 'mid360_robot_recording_check.json',
            readiness_json=output_dir / 'mid360_robot_readiness.json',
            run_plan_json=output_dir / 'mid360_robot_run_plan.json',
        )

    @staticmethod
    def _overall_status(rows: list[dict[str, Any]]) -> str:
        statuses = [str(row.get('status') or 'MISSING').upper() for row in rows]
        if any(status == 'FAIL' for status in statuses):
            return 'FAIL'
        if any(status == 'MISSING' for status in statuses):
            return 'INCOMPLETE'
        if any(status == 'WARN' for status in statuses):
            return 'WARN'
        if rows:
            return 'PASS'
        return 'INCOMPLETE'

    @staticmethod
    def _counts(rows: list[dict[str, Any]]) -> dict[str, int]:
        statuses = [str(row.get('status') or 'MISSING').upper() for row in rows]
        return {
            'total': len(rows),
            'pass': sum(1 for status in statuses if status == 'PASS'),
            'warn': sum(1 for status in statuses if status == 'WARN'),
            'fail': sum(1 for status in statuses if status == 'FAIL'),
            'missing': sum(1 for status in statuses if status == 'MISSING'),
            'ready_for_mid360_launch': sum(
                1 for row in rows if row.get('ready_for_mid360_launch')
            ),
            'map_verified': sum(
                1
                for row in rows
                if (row.get('map_validation') or {}).get('status') == 'MAP_VERIFIED'
            ),
        }


def write_public_dataset_report(report: dict[str, Any], output_dir: Path) -> dict[str, Path]:
    """Write JSON, Markdown, and HTML public dataset reports."""
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / PUBLIC_DATASET_REPORT_JSON
    markdown_path = output_dir / PUBLIC_DATASET_REPORT_MARKDOWN
    html_path = output_dir / PUBLIC_DATASET_REPORT_HTML
    json_path.write_text(payload_to_json(report) + '\n', encoding='utf-8')
    markdown_path.write_text(render_public_dataset_report_markdown(report) + '\n', encoding='utf-8')
    html_path.write_text(render_public_dataset_report_html(report), encoding='utf-8')
    return {'json': json_path, 'markdown': markdown_path, 'html': html_path}


def render_public_dataset_report_markdown(report: dict[str, Any]) -> str:
    """Render the public dataset report as Markdown."""
    counts = report.get('counts') or {}
    lines = [
        '# MID-360 Public Dataset Report',
        '',
        f"- status: `{report.get('status', 'UNKNOWN')}`",
        f"- created_at: `{report.get('created_at', '')}`",
        f"- dataset_root: `{report.get('dataset_root', '')}`",
        f"- output_root: `{report.get('output_root', '')}`",
        f"- total: `{counts.get('total', 0)}`",
        f"- ready_for_mid360_launch: `{counts.get('ready_for_mid360_launch', 0)}`",
        f"- map_verified: `{counts.get('map_verified', 0)}`",
        '',
        '## Dataset Summary',
        '',
        (
            '| Dataset | Status | PointCloud2 | IMU | PointCloud Hz | IMU Hz | '
            'Frames | Ready | Map | Warnings |'
        ),
        '| --- | --- | --- | --- | ---: | ---: | --- | --- | --- | --- |',
    ]
    for row in report.get('datasets') or []:
        lines.append(
            '| '
            + ' | '.join([
                f"[{row['dataset_id']}]({row.get('source_url', '')})",
                f"`{row.get('status', '')}`",
                f"`{(row.get('selected_topics') or {}).get('pointcloud', '')}`",
                f"`{(row.get('selected_topics') or {}).get('imu', '')}`",
                _fmt_rate((row.get('rates_hz') or {}).get('pointcloud')),
                _fmt_rate((row.get('rates_hz') or {}).get('imu')),
                _frames_text(row),
                '`yes`' if row.get('ready_for_mid360_launch') else '`no`',
                _map_validation_text(row),
                str(len(row.get('warnings') or [])),
            ])
            + ' |'
        )

    for row in report.get('datasets') or []:
        lines.extend([
            '',
            f"## {row['dataset_id']}",
            '',
            f"- title: `{row.get('title', '')}`",
            f"- selected_bag_path: `{row.get('selected_bag_path', '')}`",
            f"- sample_reader_available: `{row.get('sample_reader_available')}`",
            f"- run_plan_json: `{(row.get('artifact_paths') or {}).get('run_plan_json', '')}`",
            f"- map_validation: `{(row.get('map_validation') or {}).get('status', 'NO_DATA')}`",
            f"- map_verified_cases: `{(row.get('map_validation') or {}).get('map_verified', 0)}`",
            f"- map_sweep_manifest: `{(row.get('map_validation') or {}).get('manifest_path', '')}`",
            '',
            '### Warnings',
            '',
        ])
        warnings = row.get('warnings') or []
        if warnings:
            for warning in warnings:
                lines.append(
                    f"- `{warning.get('artifact')}` `{warning.get('status')}` "
                    f"`{warning.get('id')}`: {warning.get('message')}"
                )
        else:
            lines.append('- none')

    return '\n'.join(lines)


def render_public_dataset_report_html(report: dict[str, Any]) -> str:
    """Render the public dataset report as self-contained HTML."""
    status = str(report.get('status') or 'UNKNOWN').upper()
    rows = report.get('datasets') or []
    return '\n'.join([
        '<!doctype html>',
        '<html lang="en">',
        '<head>',
        '<meta charset="utf-8">',
        '<meta name="viewport" content="width=device-width, initial-scale=1">',
        '<title>MID-360 Public Dataset Report</title>',
        '<style>',
        _css(),
        '</style>',
        '</head>',
        '<body>',
        '<main>',
        '<section class="hero">',
        '<div>',
        '<p class="eyebrow">MID-360 Public Dataset Report</p>',
        '<h1>Real Bag Intake Comparison</h1>',
        f'<p class="subtle">{_h(report.get("dataset_root", ""))}</p>',
        '</div>',
        f'<div class="status {status.lower()}">{_h(status)}</div>',
        '</section>',
        _metric_cards(report),
        _dataset_table(rows),
        _warning_sections(rows),
        '</main>',
        '</body>',
        '</html>',
    ])


def _load_json(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}


def _discover_map_sweep_paths(output_root: Path) -> list[Path]:
    paths: set[Path] = set()
    direct = output_root / PUBLIC_RKO_SWEEP_JSON
    if direct.is_file():
        paths.add(direct.resolve())
    if output_root.is_dir():
        for path in output_root.glob(f'*/{PUBLIC_RKO_SWEEP_JSON}'):
            if path.is_file():
                paths.add(path.resolve())
    return sorted(paths)


def _map_validation_for_dataset(
    dataset_id: str,
    manifests: list[dict[str, Any]],
    manifest_paths: list[Path],
) -> dict[str, Any]:
    candidates = []
    for manifest, path in zip(manifests, manifest_paths):
        if not manifest:
            continue
        bag_path = str(manifest.get('bag_path') or '')
        if dataset_id not in bag_path:
            continue
        candidates.append((manifest, path))
    if not candidates:
        return {
            'status': 'NO_DATA',
            'manifest_path': '',
            'bag_path': '',
            'map_verified': 0,
            'map_saved': 0,
            'verify_failed': 0,
            'offline_completed': 0,
            'keypoint_drop_cases': 0,
            'lidar_delta_cases': 0,
            'best_case': '',
            'best_case_output_dir': '',
        }

    manifest, path = sorted(
        candidates,
        key=lambda item: str(item[0].get('created_at') or ''),
    )[-1]
    counts = manifest.get('counts') or {}
    diagnostics = manifest.get('diagnostics') or []
    verified = [
        row for row in diagnostics
        if (row.get('verification') or {}).get('result') == 'PASS'
    ]
    best = _best_map_validation_case(diagnostics)
    status = 'MAP_VERIFIED' if verified and int(counts.get('verify_failed') or 0) == 0 else (
        'VERIFY_FAILED' if int(counts.get('verify_failed') or 0) > 0 else (
            'MAP_SAVED' if int(counts.get('map_saved') or 0) > 0 else 'NO_MAP'
        )
    )
    return {
        'status': status,
        'manifest_path': str(path),
        'manifest_status': manifest.get('status', ''),
        'bag_path': manifest.get('bag_path', ''),
        'output_dir': manifest.get('output_dir', ''),
        'map_verified': int(counts.get('map_verified') or 0),
        'map_saved': int(counts.get('map_saved') or 0),
        'verify_failed': int(counts.get('verify_failed') or 0),
        'offline_completed': sum(
            1 for row in diagnostics if (row.get('runtime') or {}).get('offline_completed')
        ),
        'keypoint_drop_cases': int(counts.get('keypoint_drop_cases') or 0),
        'lidar_delta_cases': int(counts.get('lidar_delta_cases') or 0),
        'best_case': best.get('case_id', '') if best else '',
        'best_case_status': best.get('status', '') if best else '',
        'best_case_output_dir': best.get('output_dir', '') if best else '',
    }


def _best_map_validation_case(diagnostics: list[dict[str, Any]]) -> dict[str, Any]:
    if not diagnostics:
        return {}

    def score(row: dict[str, Any]) -> tuple[int, int, int, float]:
        runtime = row.get('runtime') or {}
        verification = row.get('verification') or {}
        run_result = row.get('run_result') or {}
        return (
            0 if verification.get('result') == 'PASS' else 1,
            int(runtime.get('keypoints_too_few_count') or 0),
            int(runtime.get('lidar_delta_error_count') or 0),
            float(run_result.get('duration_sec') or 0.0),
        )

    return sorted(diagnostics, key=score)[0]


def _first_status(*payloads: dict[str, Any]) -> str:
    for payload in payloads:
        status = payload.get('status')
        if status:
            return str(status).upper()
    return ''


def _first_mapping(*payloads: dict[str, Any], key: str) -> dict[str, Any]:
    for payload in payloads:
        value = payload.get(key)
        if isinstance(value, dict):
            return value
    return {}


def _collect_warnings(*payloads: dict[str, Any]) -> list[dict[str, str]]:
    warnings: list[dict[str, str]] = []
    seen: set[tuple[str, str, str]] = set()
    for payload, artifact in zip(payloads, ('recording_check', 'readiness')):
        for check in payload.get('checks') or []:
            status = str(check.get('status') or '').lower()
            if status not in ('warn', 'fail'):
                continue
            item = {
                'artifact': artifact,
                'id': str(check.get('id', '')),
                'status': status,
                'message': str(check.get('message', '')),
            }
            key = (item['artifact'], item['id'], item['message'])
            if key in seen:
                continue
            seen.add(key)
            warnings.append(item)
    return warnings


def _maybe_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def _fmt_rate(value: Any) -> str:
    number = _maybe_float(value)
    return '' if number is None else f'{number:.2f}'


def _frames_text(row: dict[str, Any]) -> str:
    frames = row.get('sampled_frames') or {}
    pointcloud = ','.join(str(item) for item in frames.get('pointcloud') or [])
    imu = ','.join(str(item) for item in frames.get('imu') or [])
    return f'pc:{pointcloud or "-"} imu:{imu or "-"}'


def _map_validation_text(row: dict[str, Any]) -> str:
    validation = row.get('map_validation') or {}
    status = str(validation.get('status') or 'NO_DATA')
    verified = validation.get('map_verified', 0)
    saved = validation.get('map_saved', 0)
    if status == 'NO_DATA':
        return '`no data`'
    return f'`{status}` {verified}/{saved}'


def _metric_cards(report: dict[str, Any]) -> str:
    counts = report.get('counts') or {}
    cards = [
        _metric_card('Datasets', counts.get('total', 0)),
        _metric_card('Ready', counts.get('ready_for_mid360_launch', 0)),
        _metric_card('Map Verified', counts.get('map_verified', 0)),
        _metric_card('Warnings', counts.get('warn', 0)),
    ]
    return '<section class="metrics">' + ''.join(cards) + '</section>'


def _metric_card(label: str, value: Any) -> str:
    return f'<article class="metric"><h2>{_h(label)}</h2><p>{_h(value)}</p></article>'


def _dataset_table(rows: list[dict[str, Any]]) -> str:
    if not rows:
        return '<section><p class="empty">No datasets selected.</p></section>'
    body = []
    for row in rows:
        status = str(row.get('status') or 'UNKNOWN').upper()
        rates = row.get('rates_hz') or {}
        topics = row.get('selected_topics') or {}
        map_validation = row.get('map_validation') or {}
        map_status = str(map_validation.get('status') or 'NO_DATA').upper()
        body.append(
            '<tr>'
            f'<td><a href="{_h(row.get("source_url", ""))}">{_h(row.get("dataset_id", ""))}</a></td>'
            f'<td><span class="pill {status.lower()}">{_h(status)}</span></td>'
            f'<td><code>{_h(topics.get("pointcloud", ""))}</code></td>'
            f'<td><code>{_h(topics.get("imu", ""))}</code></td>'
            f'<td>{_h(_fmt_rate(rates.get("pointcloud")))}</td>'
            f'<td>{_h(_fmt_rate(rates.get("imu")))}</td>'
            f'<td>{_h(_frames_text(row))}</td>'
            f'<td>{_h("yes" if row.get("ready_for_mid360_launch") else "no")}</td>'
            f'<td><span class="pill {map_status.lower()}">{_h(map_status)}</span></td>'
            f'<td>{_h(len(row.get("warnings") or []))}</td>'
            '</tr>'
        )
    return (
        '<section>'
        '<h2>Dataset Summary</h2>'
        '<table>'
        '<thead><tr>'
        '<th>Dataset</th><th>Status</th><th>PointCloud2</th><th>IMU</th>'
        '<th>PointCloud Hz</th><th>IMU Hz</th><th>Frames</th><th>Ready</th>'
        '<th>Map</th><th>Warnings</th>'
        '</tr></thead>'
        '<tbody>'
        + ''.join(body)
        + '</tbody></table></section>'
    )


def _warning_sections(rows: list[dict[str, Any]]) -> str:
    sections = []
    for row in rows:
        warnings = row.get('warnings') or []
        items = ''.join(
            '<li>'
            f'<code>{_h(item.get("artifact"))}</code> '
            f'<code>{_h(item.get("status"))}</code> '
            f'<code>{_h(item.get("id"))}</code>: {_h(item.get("message"))}'
            '</li>'
            for item in warnings
        ) or '<li>none</li>'
        sections.append(
            '<article class="warning-card">'
            f'<h3>{_h(row.get("dataset_id", ""))}</h3>'
            f'<p class="subtle">{_h(row.get("selected_bag_path", ""))}</p>'
            f'<ul>{items}</ul>'
            '</article>'
        )
    return '<section><h2>Warnings</h2><div class="warning-grid">' + ''.join(sections) + '</div></section>'


def _h(value: Any) -> str:
    return html.escape('' if value is None else str(value))


def _css() -> str:
    return """
:root {
  color-scheme: light;
  font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  background: #f6f8fb;
  color: #1f2937;
}
body { margin: 0; }
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
  border-bottom: 1px solid #d7dee8;
}
.eyebrow {
  margin: 0 0 8px;
  font-size: 12px;
  font-weight: 700;
  letter-spacing: 0;
  text-transform: uppercase;
  color: #506074;
}
h1, h2, h3, p { margin-top: 0; }
h1 { margin-bottom: 8px; font-size: 34px; line-height: 1.12; }
h2 { margin: 30px 0 14px; font-size: 19px; }
h3 { font-size: 16px; }
.subtle { color: #5b6778; overflow-wrap: anywhere; }
.status, .pill {
  border-radius: 6px;
  font-weight: 800;
  text-align: center;
}
.status { min-width: 102px; padding: 10px 14px; }
.pill { display: inline-block; min-width: 62px; padding: 4px 8px; font-size: 12px; }
.pass { background: #dcfce7; color: #166534; }
.warn { background: #fef3c7; color: #92400e; }
.fail { background: #fee2e2; color: #991b1b; }
.map_verified { background: #dcfce7; color: #166534; }
.map_saved { background: #dbeafe; color: #1e40af; }
.verify_failed, .no_map { background: #fee2e2; color: #991b1b; }
.no_data { background: #e5e7eb; color: #374151; }
.missing, .incomplete { background: #e5e7eb; color: #374151; }
.metrics {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 12px;
  margin: 20px 0;
}
.metric, .warning-card {
  background: #ffffff;
  border: 1px solid #d7dee8;
  border-radius: 8px;
  padding: 16px;
}
.metric h2 { margin: 0 0 8px; font-size: 13px; color: #5b6778; }
.metric p { margin: 0; font-size: 28px; font-weight: 800; }
table {
  width: 100%;
  border-collapse: collapse;
  background: #ffffff;
  border: 1px solid #d7dee8;
}
th, td {
  padding: 10px 12px;
  border-bottom: 1px solid #e5eaf1;
  text-align: left;
  vertical-align: top;
  font-size: 14px;
}
th { color: #506074; font-size: 12px; text-transform: uppercase; letter-spacing: 0; }
code {
  font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
  font-size: 12px;
  overflow-wrap: anywhere;
}
a { color: #0f5ea8; text-decoration: none; }
.warning-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 14px;
}
li { margin: 7px 0; }
@media (max-width: 820px) {
  main { padding: 18px; }
  .hero { flex-direction: column; }
  .metrics, .warning-grid { grid-template-columns: 1fr; }
  table { display: block; overflow-x: auto; }
}
"""
