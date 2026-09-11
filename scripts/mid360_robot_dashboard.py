#!/usr/bin/env python3
"""Static HTML dashboard for MID-360 robot session artifacts."""

from __future__ import annotations

import html
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DASHBOARD_HTML = 'mid360_robot_session_dashboard.html'
SEGMENT_MAP_ALIGNMENT_PLY = 'mid360_robot_public_segment_map_cloud_alignment.ply'

ARTIFACTS = (
    ('production_candidate_session', 'mid360_robot_production_candidate_session.json'),
    ('field_session', 'mid360_robot_field_session.json'),
    ('host_readiness', 'jetson_mid360_host_readiness.json'),
    ('recording_check', 'mid360_robot_recording_check.json'),
    ('readiness', 'mid360_robot_readiness.json'),
    ('map_plan', 'mid360_robot_run_plan.json'),
    ('diagnosis', 'autoware_map_diagnosis.json'),
    ('public_rko_adoption_gate', 'public_rko_adoption_gate/mid360_robot_public_rko_adoption_gate.json'),
    ('production_readiness', 'mid360_robot_production_readiness.json'),
    ('loop_alignment', 'mid360_robot_loop_alignment.json'),
    ('continuous_relocalization_gate', 'mid360_robot_public_continuous_relocalization_gate.json'),
    ('segment_map_alignment', 'mid360_robot_public_segment_map_cloud_alignment.json'),
    ('map_preview', 'mid360_robot_3d_map_preview.json'),
)


@dataclass(frozen=True)
class DashboardArtifact:
    """A loaded or missing session artifact."""

    key: str
    filename: str
    path: Path
    exists: bool
    data: dict[str, Any]
    error: str = ''


def build_dashboard_payload(output_dir: Path) -> dict[str, Any]:
    """Load known session artifacts from an output directory."""
    artifacts = [_load_artifact(output_dir, key, filename) for key, filename in ARTIFACTS]
    by_key = {artifact.key: artifact for artifact in artifacts}
    session_artifact = _session_artifact(by_key)
    artifacts = _load_dynamic_session_artifacts(output_dir, artifacts, session_artifact.data)
    by_key = {artifact.key: artifact for artifact in artifacts}
    session_artifact = _session_artifact(by_key)
    session = session_artifact.data
    readiness = by_key['readiness'].data
    recording_check = by_key['recording_check'].data

    return {
        'output_dir': str(output_dir),
        'overall_status': _overall_status(artifacts),
        'session_kind': session_artifact.key,
        'session_label': _session_label(session_artifact.key),
        'run_id': session.get('run_id', ''),
        'bag_path': session.get('bag_path') or readiness.get('bag_path') or recording_check.get('bag_path', ''),
        'created_at': session.get('created_at') or readiness.get('created_at') or '',
        'artifacts': artifacts,
        'steps': session.get('steps', []),
        'checks': _collect_checks(by_key),
        'topics': readiness.get('selected_topics', {}),
        'frames': readiness.get('frames', {}),
        'rates': _topic_rates(readiness),
        'commands': _commands(by_key),
        'next_action': _next_action(artifacts, session_artifact),
        'loop_alignment': _loop_alignment_summary(by_key['loop_alignment']),
        'continuous_relocalization_gate': _continuous_relocalization_summary(
            by_key['continuous_relocalization_gate']
        ),
        'segment_map_alignment': _segment_map_alignment_summary(by_key['segment_map_alignment']),
        'map_preview': _map_preview_summary(by_key['map_preview'], output_dir),
    }


def write_dashboard(output_dir: Path, output_path: Path | None = None) -> Path:
    """Render and write the dashboard HTML."""
    output_dir = output_dir.resolve()
    output_path = output_path or (output_dir / DASHBOARD_HTML)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload = build_dashboard_payload(output_dir)
    output_path.write_text(render_dashboard(payload), encoding='utf-8')
    return output_path


def render_dashboard(payload: dict[str, Any]) -> str:
    """Render a self-contained dashboard."""
    status = str(payload['overall_status'])
    return '\n'.join([
        '<!doctype html>',
        '<html lang="en">',
        '<head>',
        '<meta charset="utf-8">',
        '<meta name="viewport" content="width=device-width, initial-scale=1">',
        '<title>MID-360 Robot Session Dashboard</title>',
        '<style>',
        _css(),
        '</style>',
        '</head>',
        '<body>',
        '<main>',
        '<section class="summary">',
        '<div>',
        f'<p class="eyebrow">{_h(payload.get("session_label") or "MID-360 Robot Session")}</p>',
        f'<h1>{_h(payload.get("run_id") or "Session")}</h1>',
        f'<p class="path">{_h(payload.get("bag_path") or "bag path unavailable")}</p>',
        '</div>',
        f'<div class="status {status.lower()}">{_h(status)}</div>',
        '</section>',
        '<section class="grid metrics">',
        _metric_card('Output', payload.get('output_dir', '')),
        _metric_card('Created', payload.get('created_at', '')),
        _metric_card('Next Action', payload.get('next_action', '')),
        '</section>',
        '<section>',
        '<h2>Workflow</h2>',
        _timeline(payload.get('steps', [])),
        '</section>',
        '<section>',
        '<h2>Route Sketch</h2>',
        _route_sketch(payload),
        '</section>',
        '<section class="grid two">',
        '<div>',
        '<h2>Artifacts</h2>',
        _artifact_table(payload.get('artifacts', [])),
        '</div>',
        '<div>',
        '<h2>Topics And Frames</h2>',
        _topics_panel(payload),
        '</div>',
        '</section>',
        '<section>',
        '<h2>3D Map Preview</h2>',
        _map_preview_panel(payload.get('map_preview', {})),
        '</section>',
        '<section>',
        '<h2>Loop Alignment</h2>',
        _loop_alignment_panel(payload.get('loop_alignment', {})),
        '</section>',
        '<section>',
        '<h2>Continuous Relocalization Gate</h2>',
        _continuous_relocalization_panel(payload.get('continuous_relocalization_gate', {})),
        '</section>',
        '<section>',
        '<h2>Segment Map Cloud Alignment</h2>',
        _segment_map_alignment_panel(payload.get('segment_map_alignment', {})),
        '</section>',
        '<section>',
        '<h2>Checks</h2>',
        _checks_table(payload.get('checks', [])),
        '</section>',
        '<section>',
        '<h2>Commands</h2>',
        _commands_panel(payload.get('commands', [])),
        '</section>',
        '</main>',
        '</body>',
        '</html>',
    ])


def _load_artifact(output_dir: Path, key: str, filename: str) -> DashboardArtifact:
    path = output_dir / filename
    return _load_artifact_path(key, filename, path)


def _load_artifact_path(key: str, filename: str, path: Path) -> DashboardArtifact:
    if not path.is_file():
        return DashboardArtifact(key, filename, path, False, {})
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except Exception as exc:
        return DashboardArtifact(key, filename, path, True, {}, str(exc))
    return DashboardArtifact(key, filename, path, True, data)


def _session_artifact(by_key: dict[str, DashboardArtifact]) -> DashboardArtifact:
    production_candidate = by_key.get('production_candidate_session')
    if production_candidate and production_candidate.data:
        return production_candidate
    return by_key['field_session']


def _session_label(session_key: str) -> str:
    if session_key == 'production_candidate_session':
        return 'MID-360 Robot Production Candidate'
    return 'MID-360 Robot Field Session'


def _load_dynamic_session_artifacts(
    output_dir: Path,
    artifacts: list[DashboardArtifact],
    session: dict[str, Any],
) -> list[DashboardArtifact]:
    artifact_paths = session.get('artifact_paths') or {}
    dynamic_paths = {
        'diagnosis': artifact_paths.get('map_diagnosis_json'),
        'public_rko_adoption_gate': artifact_paths.get('public_rko_adoption_gate_json'),
        'production_readiness': artifact_paths.get('production_readiness_json'),
        'segment_map_alignment': artifact_paths.get('segment_map_alignment_json'),
    }
    replacements = {}
    for key, path_text in dynamic_paths.items():
        if not path_text:
            continue
        path = Path(str(path_text)).expanduser()
        if not path.is_absolute():
            path = output_dir / path
        replacements[key] = _load_artifact_path(key, path.name, path.resolve())
    if not replacements:
        return artifacts
    return [replacements.get(artifact.key, artifact) for artifact in artifacts]


def _overall_status(artifacts: list[DashboardArtifact]) -> str:
    statuses = []
    for artifact in artifacts:
        if artifact.key in ('map_preview', 'segment_map_alignment'):
            continue
        if artifact.error:
            statuses.append('FAIL')
        if artifact.data.get('status'):
            statuses.append(str(artifact.data['status']).upper())
    if 'FAIL' in statuses:
        return 'FAIL'
    if 'WARN' in statuses:
        return 'WARN'
    if statuses:
        return 'PASS'
    return 'INCOMPLETE'


def _collect_checks(by_key: dict[str, DashboardArtifact]) -> list[dict[str, str]]:
    checks = []
    for artifact_key in (
        'host_readiness',
        'recording_check',
        'readiness',
        'public_rko_adoption_gate',
        'production_readiness',
        'loop_alignment',
        'continuous_relocalization_gate',
        'segment_map_alignment',
    ):
        artifact = by_key[artifact_key]
        for check in artifact.data.get('checks', []):
            checks.append({
                'artifact': artifact.filename,
                'id': str(check.get('id', '')),
                'status': str(check.get('status', '')).upper(),
                'message': str(check.get('message', '')),
            })
    return checks


def _loop_alignment_summary(artifact: DashboardArtifact) -> dict[str, Any]:
    if not artifact.exists or not artifact.data:
        return {'present': False}
    data = artifact.data
    trajectory = data.get('trajectory') or {}
    cloud = data.get('cloud') or {}
    nearest = data.get('nearest_revisit') or {}
    candidates = data.get('loop_candidates') or []
    local_checks = data.get('local_cloud_checks') or []

    ratios = [
        c.get('largest_component_ratio')
        for c in local_checks
        if isinstance(c.get('largest_component_ratio'), (int, float))
    ]
    best_ratio = max(ratios) if ratios else None
    failed_local = sum(1 for c in local_checks if str(c.get('status', '')).upper() == 'FAIL')
    pass_local = sum(1 for c in local_checks if str(c.get('status', '')).upper() == 'PASS')

    return {
        'present': True,
        'status': str(data.get('status', '')).upper(),
        'poses': trajectory.get('poses'),
        'path_length_m': trajectory.get('path_length_m'),
        'duration_sec': trajectory.get('duration_sec'),
        'sampled_points': cloud.get('sampled_points'),
        'tile_count': cloud.get('tile_count'),
        'nearest_revisit_m': nearest.get('distance_m'),
        'loop_candidates': len(candidates),
        'local_checks_total': len(local_checks),
        'local_checks_pass': pass_local,
        'local_checks_fail': failed_local,
        'best_largest_component_ratio': best_ratio,
        'thresholds': data.get('thresholds') or {},
        'next_actions': data.get('next_actions') or [],
    }


def _continuous_relocalization_summary(artifact: DashboardArtifact) -> dict[str, Any]:
    if not artifact.exists or not artifact.data:
        return {'present': False}
    data = artifact.data
    evidence = data.get('evidence') or {}
    trajectory = evidence.get('trajectory') or {}
    recovery = evidence.get('recovery') or {}
    loop = evidence.get('loop_alignment') or {}
    verify = evidence.get('autoware_map_verify') or {}
    config = evidence.get('config') or {}
    return {
        'present': True,
        'status': str(data.get('status', '')).upper(),
        'completion_ready': data.get('completion_ready'),
        'scope': data.get('scope'),
        'poses': trajectory.get('poses'),
        'duration_sec': trajectory.get('duration_sec'),
        'relocalization_events': recovery.get('relocalization_events'),
        'recovery_accept_events': recovery.get('recovery_accept_events'),
        'dropped_scan_events': recovery.get('dropped_scan_events'),
        'loop_candidates': loop.get('loop_candidates'),
        'nearest_revisit_m': loop.get('nearest_revisit_distance_m'),
        'max_loop_distance_m': loop.get('max_loop_distance_m'),
        'autoware_status': verify.get('status'),
        'config_matches': config.get('matches_tracked_config'),
        'next_actions': data.get('next_actions') or [],
    }


def _segment_map_alignment_summary(artifact: DashboardArtifact) -> dict[str, Any]:
    if not artifact.exists or not artifact.data:
        return {'present': False}
    data = artifact.data
    clouds = data.get('clouds') or {}
    start_cloud = clouds.get('start') or {}
    end_cloud = clouds.get('end') or {}
    aligned = data.get('aligned_overlap') or {}
    transform = data.get('transform_start_to_end') or {}
    crop = data.get('crop') or {}
    artifacts = data.get('artifacts') or {}
    local_ply = artifact.path.with_name(SEGMENT_MAP_ALIGNMENT_PLY)
    return {
        'present': True,
        'status': str(data.get('status', '')).upper(),
        'start_points': start_cloud.get('analysis_points'),
        'end_points': end_cloud.get('analysis_points'),
        'crop_radius_m': crop.get('crop_radius_m'),
        'median_nn_m': aligned.get('symmetric_median_nn_m'),
        'p90_nn_m': aligned.get('symmetric_p90_nn_m'),
        'coverage_within_1m': aligned.get('coverage_within_1m'),
        'translation_norm_m': transform.get('translation_norm_m'),
        'yaw_deg': transform.get('yaw_deg'),
        'ply_path': str(local_ply if local_ply.is_file() else artifacts.get('ply') or ''),
        'next_actions': data.get('next_actions') or [],
    }


def _map_preview_summary(artifact: DashboardArtifact, output_dir: Path) -> dict[str, Any]:
    if not artifact.exists or not artifact.data:
        html_path = output_dir / 'mid360_robot_3d_map_preview.html'
        if html_path.is_file():
            return {
                'present': True,
                'status': 'FOUND',
                'html_href': html_path.name,
                'html_path': str(html_path),
            }
        return {'present': False}
    data = artifact.data
    artifacts = data.get('artifacts') or {}
    html_path = Path(str(artifacts.get('html') or output_dir / 'mid360_robot_3d_map_preview.html'))
    counts = data.get('counts') or {}
    return {
        'present': True,
        'status': str(data.get('status') or 'FOUND').upper(),
        'html_href': _relative_artifact_href(output_dir, html_path),
        'html_path': str(html_path),
        'ply_path': str(artifacts.get('ply') or ''),
        'overlay_json': str(artifacts.get('overlay_json') or ''),
        'cloud_points': counts.get('cloud_points'),
        'html_points': counts.get('html_points'),
        'trajectory_poses': counts.get('trajectory_poses'),
        'loop_candidates': counts.get('loop_candidates'),
        'pointcloud_map_dir': data.get('pointcloud_map_dir', ''),
        'next_actions': data.get('next_actions') or [],
    }


def _relative_artifact_href(output_dir: Path, path: Path) -> str:
    path = path.expanduser()
    if not path.is_absolute():
        return str(path)
    try:
        return str(path.resolve().relative_to(output_dir.resolve()))
    except ValueError:
        return str(path)


def _topic_rates(readiness: dict[str, Any]) -> dict[str, Any]:
    diagnostics = readiness.get('bag_diagnostics') or {}
    topics = diagnostics.get('topics') or {}
    return {
        'pointcloud': (topics.get('pointcloud') or {}).get('metadata_rate_hz'),
        'imu': (topics.get('imu') or {}).get('metadata_rate_hz'),
    }


def _commands(by_key: dict[str, DashboardArtifact]) -> list[dict[str, str]]:
    commands = []
    session = _session_artifact(by_key).data
    for step in session.get('steps', []):
        command = step.get('command') or []
        if command:
            commands.append({
                'label': str(step.get('id', 'step')),
                'command': ' '.join(str(item) for item in command),
            })

    map_plan = by_key['map_plan'].data
    if map_plan.get('dogfood_command_shell'):
        commands.append({
            'label': 'map_dry_run',
            'command': str(map_plan['dogfood_command_shell']),
        })
    return commands


def _next_action(artifacts: list[DashboardArtifact], session_artifact: DashboardArtifact) -> str:
    production_readiness = next(
        (artifact for artifact in artifacts if artifact.key == 'production_readiness'),
        None,
    )
    production_actions = (production_readiness.data.get('next_actions') if production_readiness else None) or []
    production_status = str((production_readiness.data if production_readiness else {}).get('status') or '').upper()
    if production_actions and production_status in ('FAIL', 'WARN'):
        return '; '.join(str(action) for action in production_actions[:2])
    session_actions = session_artifact.data.get('next_actions') or []
    if session_actions:
        return '; '.join(str(action) for action in session_actions[:2])
    if production_actions:
        return '; '.join(str(action) for action in production_actions[:2])
    missing_required = [
        artifact.filename
        for artifact in artifacts
        if artifact.key in _required_artifact_keys(session_artifact.key)
        and not artifact.exists
    ]
    if missing_required:
        return f'Create missing artifacts: {", ".join(missing_required)}'
    status = _overall_status(artifacts)
    if status == 'FAIL':
        return 'Fix failing checks before the next production-candidate run.'
    if status == 'WARN':
        return 'Review warnings, then decide whether to rerun mapping or gate checks.'
    if session_artifact.key == 'production_candidate_session':
        return 'Ready for production-readiness review.'
    return 'Ready for map run review.'


def _required_artifact_keys(session_key: str) -> tuple[str, ...]:
    if session_key == 'production_candidate_session':
        return (
            'production_candidate_session',
            'recording_check',
            'readiness',
            'map_plan',
            'diagnosis',
            'production_readiness',
        )
    return ('field_session', 'recording_check', 'readiness', 'map_plan')


def _metric_card(label: str, value: Any) -> str:
    return f'<article class="card"><h3>{_h(label)}</h3><p>{_h(value or "n/a")}</p></article>'


def _timeline(steps: list[dict[str, Any]]) -> str:
    if not steps:
        return '<p class="empty">No field-session steps found.</p>'
    items = []
    for step in steps:
        status = str(step.get('status', 'unknown')).lower()
        items.append(
            '<li>'
            f'<span class="dot {status}"></span>'
            f'<strong>{_h(step.get("id", ""))}</strong>'
            f'<em>{_h(step.get("status", ""))}</em>'
            f'<p>{_h(step.get("message", ""))}</p>'
            '</li>'
        )
    return '<ol class="timeline">' + ''.join(items) + '</ol>'


def _route_sketch(payload: dict[str, Any]) -> str:
    nodes = _route_nodes(payload)
    width = max(840, 180 * len(nodes))
    spacing = (width - 180) / max(1, len(nodes) - 1)
    x_positions = [90 + round(index * spacing) for index in range(len(nodes))]
    svg_parts = [
        f'<svg class="route-sketch" viewBox="0 0 {width} 210" role="img" aria-label="MID-360 session route sketch">',
        '<defs>',
        '<marker id="arrow" markerWidth="10" markerHeight="10" refX="8" refY="3" orient="auto" markerUnits="strokeWidth">',
        '<path d="M0,0 L0,6 L9,3 z" fill="#7d8da1"></path>',
        '</marker>',
        '</defs>',
        f'<path class="route-ground" d="M56 154 C174 110, 278 184, 396 138 S{width - 204} 104, {width - 56} 144"></path>',
    ]
    for start, end in zip(x_positions, x_positions[1:]):
        svg_parts.append(
            f'<line class="route-link" x1="{start + 54}" y1="78" x2="{end - 54}" y2="78"></line>'
        )
    for node, x in zip(nodes, x_positions):
        status = _status_class(node['status'])
        svg_parts.extend([
            f'<g class="route-node {status}">',
            f'<circle cx="{x}" cy="78" r="42"></circle>',
            f'<text class="route-index" x="{x}" y="72">{_h(node["index"])}</text>',
            f'<text class="route-state" x="{x}" y="94">{_h(node["status"])}</text>',
            f'<text class="route-label" x="{x}" y="142">{_h(node["label"])}</text>',
            f'<text class="route-detail" x="{x}" y="164">{_h(node["detail"])}</text>',
            '</g>',
        ])
    svg_parts.append('</svg>')
    return '<div class="sketch-panel">' + ''.join(svg_parts) + '</div>'


def _route_nodes(payload: dict[str, Any]) -> list[dict[str, str]]:
    steps = {str(step.get('id', '')): step for step in payload.get('steps', [])}
    artifacts = {artifact.key: artifact for artifact in payload.get('artifacts', [])}
    recording_step = steps.get('recording') or steps.get('recording_plan') or {}
    map_step = steps.get('map') or {}

    if payload.get('session_kind') == 'production_candidate_session':
        return [
            {
                'index': '1',
                'label': 'Host Check',
                'status': _step_or_artifact_status(steps.get('host_readiness'), artifacts.get('host_readiness')),
                'detail': 'Jetson',
            },
            {
                'index': '2',
                'label': 'Record Bag',
                'status': _display_status(str(recording_step.get('status') or 'missing')),
                'detail': 'rosbag2',
            },
            {
                'index': '3',
                'label': 'Post Check',
                'status': _step_or_artifact_status(steps.get('post_recording_check'), artifacts.get('recording_check')),
                'detail': 'readiness',
            },
            {
                'index': '4',
                'label': 'Map Run',
                'status': _step_or_artifact_status(map_step, artifacts.get('diagnosis')),
                'detail': 'diagnosis',
            },
            {
                'index': '5',
                'label': 'Public Gate',
                'status': _step_or_artifact_status(
                    steps.get('public_rko_adoption_gate'),
                    artifacts.get('public_rko_adoption_gate'),
                ),
                'detail': 'RKO evidence',
            },
            {
                'index': '6',
                'label': 'Prod Gate',
                'status': _step_or_artifact_status(
                    steps.get('production_readiness'),
                    artifacts.get('production_readiness'),
                ),
                'detail': 'readiness',
            },
        ]

    return [
        {
            'index': '1',
            'label': 'Record Bag',
            'status': _display_status(str(recording_step.get('status') or 'missing')),
            'detail': 'rosbag2',
        },
        {
            'index': '2',
            'label': 'Post Check',
            'status': _artifact_status(artifacts.get('recording_check')),
            'detail': 'readiness',
        },
        {
            'index': '3',
            'label': 'Map Dry-Run',
            'status': _artifact_status(artifacts.get('map_plan')),
            'detail': 'command plan',
        },
        {
            'index': '4',
            'label': 'Map Run',
            'status': _map_run_status(artifacts.get('diagnosis'), map_step),
            'detail': 'diagnosis',
        },
    ]


def _step_or_artifact_status(
    step: dict[str, Any] | None,
    artifact: DashboardArtifact | None,
) -> str:
    if artifact and artifact.exists:
        return _artifact_status(artifact)
    if step and step.get('status'):
        return _display_status(str(step.get('status')))
    return 'MISSING'


def _artifact_status(artifact: DashboardArtifact | None) -> str:
    if artifact is None or not artifact.exists:
        return 'MISSING'
    if artifact.error:
        return 'FAIL'
    return _display_status(str(artifact.data.get('status') or 'PASS'))


def _map_run_status(artifact: DashboardArtifact | None, map_step: dict[str, Any]) -> str:
    if artifact and artifact.exists:
        return _artifact_status(artifact)
    step_status = str(map_step.get('status') or '')
    if step_status:
        return _display_status(step_status)
    return 'MISSING'


def _display_status(status: str) -> str:
    normalized = status.upper()
    if normalized == 'OK':
        return 'OK'
    if normalized == 'PASS':
        return 'OK'
    if normalized == 'SUCCESS':
        return 'OK'
    if normalized in ('WARN', 'FAIL', 'PLANNED', 'SKIPPED', 'MISSING'):
        return normalized
    return normalized or 'MISSING'


def _status_class(status: str) -> str:
    return status.lower().replace('_', '-')


def _artifact_table(artifacts: list[DashboardArtifact]) -> str:
    rows = []
    for artifact in artifacts:
        status = 'ERROR' if artifact.error else ('FOUND' if artifact.exists else 'MISSING')
        detail = artifact.error or str(artifact.path)
        rows.append(
            '<tr>'
            f'<td>{_h(artifact.key)}</td>'
            f'<td><span class="pill {status.lower()}">{status}</span></td>'
            f'<td>{_h(detail)}</td>'
            '</tr>'
        )
    return '<table><thead><tr><th>Artifact</th><th>Status</th><th>Path</th></tr></thead><tbody>' + ''.join(rows) + '</tbody></table>'


def _topics_panel(payload: dict[str, Any]) -> str:
    topics = payload.get('topics') or {}
    frames = payload.get('frames') or {}
    rates = payload.get('rates') or {}
    return ''.join([
        '<div class="panel-list">',
        _kv('PointCloud', topics.get('pointcloud')),
        _rate('PointCloud Rate', rates.get('pointcloud'), 10.0),
        _kv('IMU', topics.get('imu')),
        _rate('IMU Rate', rates.get('imu'), 100.0),
        _kv('Base Frame', frames.get('base_frame')),
        _kv('LiDAR Frame', frames.get('lidar_frame')),
        _kv('IMU Frame', frames.get('imu_frame')),
        '</div>',
    ])


def _kv(label: str, value: Any) -> str:
    return f'<div class="kv"><span>{_h(label)}</span><strong>{_h(value or "n/a")}</strong></div>'


def _rate(label: str, value: Any, target: float) -> str:
    if isinstance(value, (int, float)):
        width = min(100.0, max(2.0, (float(value) / target) * 100.0))
        text = f'{float(value):.2f} Hz'
    else:
        width = 2.0
        text = 'n/a'
    return ''.join([
        '<div class="rate">',
        f'<div class="rate-label"><span>{_h(label)}</span><strong>{_h(text)}</strong></div>',
        '<div class="bar"><span style="width:',
        f'{width:.1f}%',
        '"></span></div>',
        '</div>',
    ])


def _checks_table(checks: list[dict[str, str]]) -> str:
    if not checks:
        return '<p class="empty">No checks found.</p>'
    rows = []
    for check in checks:
        status = check['status'].lower()
        rows.append(
            '<tr>'
            f'<td><span class="pill {status}">{_h(check["status"])}</span></td>'
            f'<td>{_h(check["artifact"])}</td>'
            f'<td>{_h(check["id"])}</td>'
            f'<td>{_h(check["message"])}</td>'
            '</tr>'
        )
    return '<table><thead><tr><th>Status</th><th>Artifact</th><th>Check</th><th>Message</th></tr></thead><tbody>' + ''.join(rows) + '</tbody></table>'


def _loop_alignment_panel(summary: dict[str, Any]) -> str:
    if not summary or not summary.get('present'):
        return '<p class="empty">No loop alignment analyzer artifact found.</p>'

    status = str(summary.get('status') or 'MISSING').upper()
    thresholds = summary.get('thresholds') or {}

    def _fmt_num(value: Any, suffix: str = '', digits: int = 2) -> str:
        if isinstance(value, (int, float)):
            return f'{float(value):.{digits}f}{suffix}'
        return 'n/a'

    def _fmt_int(value: Any) -> str:
        if isinstance(value, (int, float)):
            return f'{int(value)}'
        return 'n/a'

    nearest_threshold = thresholds.get('max_loop_distance_m')
    ratio_threshold = thresholds.get('min_largest_component_ratio')
    nearest_value = summary.get('nearest_revisit_m')
    nearest_display = _fmt_num(nearest_value, ' m')
    if isinstance(nearest_value, (int, float)) and isinstance(nearest_threshold, (int, float)):
        nearest_display += f' (≤ {nearest_threshold:.1f} m)'
    ratio_value = summary.get('best_largest_component_ratio')
    ratio_display = _fmt_num(ratio_value, '', digits=3)
    if isinstance(ratio_value, (int, float)) and isinstance(ratio_threshold, (int, float)):
        ratio_display += f' (≥ {ratio_threshold:.2f})'

    local_total = summary.get('local_checks_total') or 0
    local_fail = summary.get('local_checks_fail') or 0
    local_pass = summary.get('local_checks_pass') or 0
    local_summary = f'{local_pass} PASS / {local_fail} FAIL / {local_total} total'

    next_actions = summary.get('next_actions') or []

    kvs = ''.join([
        _kv('Status', status),
        _kv('Trajectory poses', _fmt_int(summary.get('poses'))),
        _kv('Path length', _fmt_num(summary.get('path_length_m'), ' m')),
        _kv('Duration', _fmt_num(summary.get('duration_sec'), ' s', digits=1)),
        _kv('Sampled cloud points', _fmt_int(summary.get('sampled_points'))),
        _kv('Pointcloud tiles', _fmt_int(summary.get('tile_count'))),
        _kv('Loop candidates', _fmt_int(summary.get('loop_candidates'))),
        _kv('Nearest revisit', nearest_display),
        _kv('Best largest_component_ratio', ratio_display),
        _kv('Local cloud checks', local_summary),
    ])
    parts = [
        f'<div class="loop-alignment-status status {status.lower()}">{_h(status)}</div>',
        '<div class="panel-list">',
        kvs,
        '</div>',
    ]
    if next_actions:
        parts.append('<h3>Next Actions</h3>')
        parts.append('<ul class="next-actions">')
        for action in next_actions[:4]:
            parts.append(f'<li>{_h(action)}</li>')
        parts.append('</ul>')
    return ''.join(parts)


def _continuous_relocalization_panel(summary: dict[str, Any]) -> str:
    if not summary or not summary.get('present'):
        return '<p class="empty">No continuous relocalization gate artifact found.</p>'

    status = str(summary.get('status') or 'MISSING').upper()

    def _fmt_num(value: Any, suffix: str = '', digits: int = 2) -> str:
        if isinstance(value, (int, float)):
            return f'{float(value):.{digits}f}{suffix}'
        return 'n/a'

    def _fmt_int(value: Any) -> str:
        if isinstance(value, (int, float)):
            return f'{int(value)}'
        return 'n/a'

    kvs = ''.join([
        _kv('Status', status),
        _kv('Completion ready', str(summary.get('completion_ready'))),
        _kv('Scope', summary.get('scope')),
        _kv('RKO poses', _fmt_int(summary.get('poses'))),
        _kv('Trajectory duration', _fmt_num(summary.get('duration_sec'), ' s', digits=1)),
        _kv('Relocalization events', _fmt_int(summary.get('relocalization_events'))),
        _kv('Accepted recovery scans', _fmt_int(summary.get('recovery_accept_events'))),
        _kv('Dropped invalid scans', _fmt_int(summary.get('dropped_scan_events'))),
        _kv('Loop candidates', _fmt_int(summary.get('loop_candidates'))),
        _kv('Nearest revisit', _fmt_num(summary.get('nearest_revisit_m'), ' m', digits=3)),
        _kv('Max loop distance', _fmt_num(summary.get('max_loop_distance_m'), ' m', digits=3)),
        _kv('Autoware verify', summary.get('autoware_status')),
        _kv('Tracked config match', str(summary.get('config_matches'))),
    ])
    parts = [
        f'<div class="loop-alignment-status status {status.lower()}">{_h(status)}</div>',
        '<div class="panel-list">',
        kvs,
        '</div>',
    ]
    next_actions = summary.get('next_actions') or []
    if next_actions:
        parts.append('<h3>Next Actions</h3>')
        parts.append('<ul class="next-actions">')
        for action in next_actions[:3]:
            parts.append(f'<li>{_h(action)}</li>')
        parts.append('</ul>')
    return ''.join(parts)


def _segment_map_alignment_panel(summary: dict[str, Any]) -> str:
    if not summary or not summary.get('present'):
        return '<p class="empty">No segment map cloud alignment artifact found.</p>'

    status = str(summary.get('status') or 'MISSING').upper()

    def _fmt_num(value: Any, suffix: str = '', digits: int = 2) -> str:
        if isinstance(value, (int, float)):
            return f'{float(value):.{digits}f}{suffix}'
        return 'n/a'

    def _fmt_int(value: Any) -> str:
        if isinstance(value, (int, float)):
            return f'{int(value)}'
        return 'n/a'

    kvs = ''.join([
        _kv('Status', status),
        _kv('Start analysis points', _fmt_int(summary.get('start_points'))),
        _kv('End analysis points', _fmt_int(summary.get('end_points'))),
        _kv('Crop radius', _fmt_num(summary.get('crop_radius_m'), ' m')),
        _kv('Aligned median NN', _fmt_num(summary.get('median_nn_m'), ' m', digits=3)),
        _kv('Aligned p90 NN', _fmt_num(summary.get('p90_nn_m'), ' m', digits=3)),
        _kv('Coverage within 1m', _fmt_num(summary.get('coverage_within_1m'), '', digits=3)),
        _kv('Start→end translation', _fmt_num(summary.get('translation_norm_m'), ' m')),
        _kv('Start→end yaw', _fmt_num(summary.get('yaw_deg'), ' deg')),
        _kv('Aligned PLY', summary.get('ply_path')),
    ])
    parts = [
        f'<div class="loop-alignment-status status {status.lower()}">{_h(status)}</div>',
        '<div class="panel-list">',
        kvs,
        '</div>',
    ]
    next_actions = summary.get('next_actions') or []
    if next_actions:
        parts.append('<h3>Next Actions</h3>')
        parts.append('<ul class="next-actions">')
        for action in next_actions[:3]:
            parts.append(f'<li>{_h(action)}</li>')
        parts.append('</ul>')
    return ''.join(parts)


def _map_preview_panel(summary: dict[str, Any]) -> str:
    if not summary or not summary.get('present'):
        return '<p class="empty">No 3D map preview artifact found.</p>'

    status = str(summary.get('status') or 'FOUND').upper()

    def _fmt_int(value: Any) -> str:
        if isinstance(value, (int, float)):
            return f'{int(value)}'
        return 'n/a'

    href = str(summary.get('html_href') or '')
    open_link = ''
    if href:
        open_link = (
            '<p class="preview-link">'
            f'<a href="{_h(href)}">Open 3D map preview</a>'
            '</p>'
        )

    kvs = ''.join([
        _kv('Status', status),
        _kv('Cloud points', _fmt_int(summary.get('cloud_points'))),
        _kv('Browser preview points', _fmt_int(summary.get('html_points'))),
        _kv('Trajectory poses', _fmt_int(summary.get('trajectory_poses'))),
        _kv('Loop candidates', _fmt_int(summary.get('loop_candidates'))),
        _kv('Pointcloud map', summary.get('pointcloud_map_dir')),
        _kv('PLY', summary.get('ply_path')),
        _kv('Overlay JSON', summary.get('overlay_json')),
    ])
    parts = [
        f'<div class="loop-alignment-status status {status.lower()}">{_h(status)}</div>',
        open_link,
        '<div class="panel-list">',
        kvs,
        '</div>',
    ]
    next_actions = summary.get('next_actions') or []
    if next_actions:
        parts.append('<h3>Next Actions</h3>')
        parts.append('<ul class="next-actions">')
        for action in next_actions[:3]:
            parts.append(f'<li>{_h(action)}</li>')
        parts.append('</ul>')
    return ''.join(parts)


def _commands_panel(commands: list[dict[str, str]]) -> str:
    if not commands:
        return '<p class="empty">No commands found.</p>'
    blocks = []
    for command in commands:
        blocks.append(
            '<article class="command">'
            f'<h3>{_h(command["label"])}</h3>'
            f'<pre><code>{_h(command["command"])}</code></pre>'
            '</article>'
        )
    return ''.join(blocks)


def _h(value: Any) -> str:
    return html.escape(str(value), quote=True)


def _css() -> str:
    return """
:root {
  color-scheme: light;
  --bg: #f6f7f9;
  --panel: #ffffff;
  --text: #17202a;
  --muted: #667085;
  --line: #d7dde5;
  --ok: #16805b;
  --warn: #b7791f;
  --fail: #bd2f37;
  --planned: #3366a8;
  --accent: #245b73;
}
* { box-sizing: border-box; }
body {
  margin: 0;
  background: var(--bg);
  color: var(--text);
  font: 14px/1.45 system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
}
main {
  width: min(1240px, calc(100vw - 32px));
  margin: 24px auto 48px;
}
.summary {
  min-height: 180px;
  display: flex;
  align-items: flex-end;
  justify-content: space-between;
  gap: 24px;
  padding: 28px;
  background: #102332;
  color: #fff;
  border-radius: 8px;
}
.eyebrow {
  margin: 0 0 8px;
  color: #9bc4d7;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0;
}
h1, h2, h3, p { margin-top: 0; }
h1 { font-size: 34px; line-height: 1.1; margin-bottom: 10px; letter-spacing: 0; }
h2 { margin: 28px 0 12px; font-size: 18px; letter-spacing: 0; }
h3 { font-size: 13px; color: var(--muted); letter-spacing: 0; text-transform: uppercase; }
.path { margin: 0; color: #c7d5dd; overflow-wrap: anywhere; }
.status {
  min-width: 118px;
  text-align: center;
  padding: 12px 16px;
  border-radius: 8px;
  font-weight: 800;
  background: #6b7280;
}
.status.pass, .pill.ok, .pill.pass, .dot.ok { background: var(--ok); color: #fff; }
.status.warn, .pill.warn, .dot.warn { background: var(--warn); color: #fff; }
.status.fail, .pill.fail, .dot.fail, .pill.error { background: var(--fail); color: #fff; }
.status.incomplete, .pill.missing, .dot.skipped { background: #5d6673; color: #fff; }
.dot.planned { background: var(--planned); }
.grid { display: grid; gap: 14px; }
.metrics { grid-template-columns: repeat(3, minmax(0, 1fr)); margin-top: 14px; }
.two { grid-template-columns: minmax(0, 1.2fr) minmax(320px, 0.8fr); align-items: start; }
.card, table, .panel-list, .command {
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 8px;
}
.card { padding: 16px; min-height: 92px; }
.card p { margin: 0; overflow-wrap: anywhere; }
table {
  width: 100%;
  border-collapse: collapse;
  overflow: hidden;
}
th, td {
  padding: 10px 12px;
  border-bottom: 1px solid var(--line);
  text-align: left;
  vertical-align: top;
}
th { color: var(--muted); font-size: 12px; text-transform: uppercase; letter-spacing: 0; }
td { overflow-wrap: anywhere; }
tr:last-child td { border-bottom: 0; }
.pill {
  display: inline-block;
  min-width: 68px;
  padding: 3px 8px;
  border-radius: 8px;
  text-align: center;
  font-size: 12px;
  font-weight: 800;
  background: #64748b;
  color: #fff;
}
.pill.found { background: var(--accent); }
.timeline {
  list-style: none;
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
  gap: 12px;
  padding: 0;
  margin: 0;
}
.timeline li {
  position: relative;
  min-height: 132px;
  padding: 16px;
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 8px;
}
.dot {
  display: inline-block;
  width: 12px;
  height: 12px;
  border-radius: 50%;
  margin-right: 8px;
  vertical-align: middle;
  background: #64748b;
}
.timeline strong { display: inline-block; margin-bottom: 8px; }
.timeline em { display: block; color: var(--muted); font-style: normal; font-size: 12px; }
.timeline p { margin: 8px 0 0; color: #344054; }
.sketch-panel {
  background: var(--panel);
  border: 1px solid var(--line);
  border-radius: 8px;
  padding: 12px;
  overflow-x: auto;
}
.route-sketch {
  display: block;
  min-width: 760px;
  width: 100%;
  height: auto;
}
.route-ground {
  fill: none;
  stroke: #c5ced9;
  stroke-width: 5;
  stroke-linecap: round;
  stroke-dasharray: 10 12;
}
.route-link {
  stroke: #7d8da1;
  stroke-width: 3;
  marker-end: url(#arrow);
}
.route-node circle {
  fill: #64748b;
  stroke: #ffffff;
  stroke-width: 5;
}
.route-node.ok circle { fill: var(--ok); }
.route-node.warn circle { fill: var(--warn); }
.route-node.fail circle { fill: var(--fail); }
.route-node.planned circle { fill: var(--planned); }
.route-node.skipped circle,
.route-node.missing circle { fill: #5d6673; }
.route-index,
.route-state {
  fill: #ffffff;
  text-anchor: middle;
  font-weight: 800;
}
.route-index { font-size: 22px; }
.route-state { font-size: 11px; }
.route-label {
  fill: var(--text);
  text-anchor: middle;
  font-weight: 800;
  font-size: 14px;
}
.route-detail {
  fill: var(--muted);
  text-anchor: middle;
  font-size: 12px;
}
.panel-list { padding: 14px; }
.kv, .rate-label {
  display: flex;
  justify-content: space-between;
  gap: 14px;
  padding: 9px 0;
  border-bottom: 1px solid var(--line);
}
.kv:last-child { border-bottom: 0; }
.kv span, .rate-label span { color: var(--muted); }
.kv strong { overflow-wrap: anywhere; text-align: right; }
.rate { padding: 8px 0 12px; border-bottom: 1px solid var(--line); }
.bar {
  height: 10px;
  background: #e7ebf0;
  border-radius: 8px;
  overflow: hidden;
}
.bar span { display: block; height: 100%; background: #2f7d73; }
.command { padding: 14px; margin-bottom: 12px; }
.command h3 { margin-bottom: 8px; }
pre {
  margin: 0;
  white-space: pre-wrap;
  overflow-wrap: anywhere;
  background: #111827;
  color: #e5e7eb;
  padding: 12px;
  border-radius: 8px;
}
.empty {
  padding: 14px;
  border: 1px dashed var(--line);
  border-radius: 8px;
  color: var(--muted);
  background: #fff;
}
.preview-link a {
  display: inline-block;
  padding: 9px 12px;
  border-radius: 8px;
  background: var(--accent);
  color: #fff;
  font-weight: 800;
  text-decoration: none;
}
@media (max-width: 760px) {
  main { width: min(100vw - 20px, 720px); margin-top: 10px; }
  .summary { display: block; min-height: 0; padding: 20px; }
  .status { margin-top: 18px; width: 100%; }
  .metrics, .two { grid-template-columns: 1fr; }
  h1 { font-size: 26px; }
}
"""


if __name__ == "__main__":
    raise SystemExit("mid360_robot_dashboard is a library module; import it instead of running it directly.")
