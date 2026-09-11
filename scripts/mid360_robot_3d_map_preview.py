#!/usr/bin/env python3
"""Browser preview artifacts for MID-360 map outputs."""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.map_edit import (
    MapEditError,
    accepted_loop_edges,
    resolve_bundle_dir,
    sha256_directory,
    sha256_file,
)

from lidarslam_benchmark_tools.mid360_robot_loop_alignment_analyzer import (
    LOOP_ALIGNMENT_JSON,
    LoopAlignmentThresholds,
    find_loop_candidates,
    load_pointcloud_map_points,
    load_tum_trajectory,
    resolve_pointcloud_map_dir,
    resolve_trajectory_path,
)
from lidarslam_benchmark_tools.mid360_robot_tools import payload_to_json

import yaml
MAP_PREVIEW_JSON = 'mid360_robot_3d_map_preview.json'
MAP_PREVIEW_HTML = 'mid360_robot_3d_map_preview.html'
MAP_PREVIEW_PLY = 'mid360_robot_3d_map_preview.ply'
MAP_PREVIEW_OVERLAY_JSON = 'mid360_robot_3d_map_preview_overlay.json'


@dataclass(frozen=True)
class MapPreviewOptions:
    """Inputs and limits for a browser-ready MID-360 map preview."""

    run_dir: Path
    pointcloud_map_dir: Path | None = None
    trajectory_path: Path | None = None
    loop_alignment_path: Path | None = None
    output_dir: Path | None = None
    max_points: int = 50000
    max_points_per_tile: int = 5000
    html_max_points: int = 15000
    max_trajectory_poses: int = 2000
    max_loop_candidates: int = 20


class Mid360MapPreviewExporter:
    """Export a lightweight 3D map preview from map artifacts."""

    def export(self, options: MapPreviewOptions) -> dict[str, Any]:
        """Build preview data and write artifacts."""
        run_dir = options.run_dir.expanduser().resolve()
        output_dir = (options.output_dir.expanduser().resolve()
                      if options.output_dir else run_dir)
        map_dir = resolve_pointcloud_map_dir(run_dir, options.pointcloud_map_dir)
        trajectory_path = resolve_trajectory_path(run_dir, options.trajectory_path)
        cloud = load_pointcloud_map_points(
            map_dir,
            max_points_per_tile=max(1, options.max_points_per_tile),
            max_total_points=max(1, options.max_points),
        )
        points = _decorate_points(cloud.get('points') or [])
        poses = load_tum_trajectory(trajectory_path) if trajectory_path else []
        trajectory = _sample_trajectory(poses, max(1, options.max_trajectory_poses))
        session_trajectories = _load_session_trajectories(
            run_dir,
            max(1, options.max_trajectory_poses),
        )
        visible_trajectory = (
            [pose for session in session_trajectories for pose in session['trajectory']]
            if session_trajectories else trajectory
        )
        loop_report = _load_loop_alignment(run_dir, options.loop_alignment_path)
        loop_candidates = _loop_candidates(loop_report, poses, max(1, options.max_loop_candidates))
        edit_source, accepted_loops = _edit_source(run_dir, poses)
        overlay = {
            'trajectory_path': str(trajectory_path) if trajectory_path else '',
            'trajectory': trajectory,
            'session_trajectories': session_trajectories,
            'loop_candidates': loop_candidates,
            'accepted_loop_edges': accepted_loops,
        }
        bounds = _bounds(points, visible_trajectory, loop_candidates)
        html_points = _sample_items(points, max(1, options.html_max_points))
        output_dir.mkdir(parents=True, exist_ok=True)
        ply_path = output_dir / MAP_PREVIEW_PLY
        overlay_path = output_dir / MAP_PREVIEW_OVERLAY_JSON
        html_path = output_dir / MAP_PREVIEW_HTML
        json_path = output_dir / MAP_PREVIEW_JSON
        ply_path.write_text(_render_ply(points), encoding='utf-8')
        overlay_path.write_text(
            json.dumps(overlay, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        manifest = {
            'created_at': datetime.now(timezone.utc).isoformat(),
            'status': 'PASS' if points else 'FAIL',
            'run_dir': str(run_dir),
            'pointcloud_map_dir': str(map_dir),
            'trajectory_path': str(trajectory_path) if trajectory_path else '',
            'loop_alignment_path': _loop_alignment_path(run_dir, options.loop_alignment_path),
            'output_dir': str(output_dir),
            'artifacts': {
                'html': str(html_path),
                'ply': str(ply_path),
                'overlay_json': str(overlay_path),
                'manifest_json': str(json_path),
            },
            'counts': {
                'cloud_points': len(points),
                'html_points': len(html_points),
                'trajectory_poses': len(visible_trajectory),
                'map_sessions': len(session_trajectories) or 1,
                'loop_candidates': len(loop_candidates),
                'accepted_loop_edges': len(accepted_loops),
            },
            'bounds': bounds,
            'cloud': {key: value for key, value in cloud.items() if key != 'points'},
            'edit': {
                'schema_version': 1,
                'source': edit_source,
                'accepted_loop_edges': len(accepted_loops),
                'workflow': 'download_plan_then_apply_non_destructively',
            },
            'next_actions': _next_actions(points, trajectory, loop_candidates),
        }
        html_path.write_text(
            _render_html(
                manifest=manifest,
                points=html_points,
                trajectory=trajectory,
                session_trajectories=session_trajectories,
                loop_candidates=loop_candidates,
                accepted_loop_edges=accepted_loops,
            ),
            encoding='utf-8',
        )
        json_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        return manifest


def _decorate_points(points: list[tuple[float, float, float]]) -> list[dict[str, Any]]:
    if not points:
        return []
    min_z = min(point[2] for point in points)
    max_z = max(point[2] for point in points)
    span = max(max_z - min_z, 1e-6)
    decorated = []
    for x, y, z in points:
        color = _height_color((z - min_z) / span)
        decorated.append({'x': x, 'y': y, 'z': z, 'rgb': color})
    return decorated


def _height_color(value: float) -> list[int]:
    value = max(0.0, min(1.0, value))
    if value < 0.5:
        ratio = value / 0.5
        red = 40
        green = int(120 + 90 * ratio)
        blue = int(210 - 150 * ratio)
    else:
        ratio = (value - 0.5) / 0.5
        red = int(40 + 210 * ratio)
        green = int(210 - 60 * ratio)
        blue = int(60 - 30 * ratio)
    return [red, green, blue]


def _sample_trajectory(poses: list[Any], max_poses: int) -> list[dict[str, float]]:
    sampled = _sample_items(poses, max_poses)
    return [
        {
            'index': float(pose.index),
            'stamp': float(pose.stamp),
            'x': float(pose.x),
            'y': float(pose.y),
            'z': float(pose.z),
        }
        for pose in sampled
    ]


def _load_session_trajectories(
    run_dir: Path,
    max_poses: int,
) -> list[dict[str, Any]]:
    """Load independently transformed paths from a multi-session project."""
    try:
        bundle_dir = resolve_bundle_dir(run_dir)
    except MapEditError:
        return []
    project_path = bundle_dir / 'map_project.json'
    sessions_dir = bundle_dir / 'sessions'
    if not project_path.is_file() or not sessions_dir.is_dir():
        return []
    paths = sorted(sessions_dir.glob('*/trajectory_transformed.tum'))
    if not paths:
        return []
    per_session = max(2, max_poses // len(paths))
    result = []
    palette = ['#63d6ff', '#ffb454', '#9be564', '#df7bff', '#ff6f91', '#66e0c2']
    for index, path in enumerate(paths):
        poses = load_tum_trajectory(path)
        result.append({
            'session_index': index,
            'name': path.parent.name,
            'trajectory_path': str(path),
            'color': palette[index % len(palette)],
            'trajectory': _sample_trajectory(poses, per_session),
        })
    return result


def _sample_items(items: list[Any], max_items: int) -> list[Any]:
    if len(items) <= max_items:
        return list(items)
    if max_items <= 1:
        return [items[0]]
    return [
        items[round(index * (len(items) - 1) / (max_items - 1))]
        for index in range(max_items)
    ]


def _load_loop_alignment(run_dir: Path, loop_alignment_path: Path | None) -> dict[str, Any]:
    path_text = _loop_alignment_path(run_dir, loop_alignment_path)
    if not path_text:
        return {}
    path = Path(path_text)
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _loop_alignment_path(run_dir: Path, loop_alignment_path: Path | None) -> str:
    if loop_alignment_path:
        return str(loop_alignment_path.expanduser().resolve())
    candidate = run_dir / LOOP_ALIGNMENT_JSON
    return str(candidate) if candidate.is_file() else ''


def _loop_candidates(
    loop_report: dict[str, Any],
    poses: list[Any],
    max_candidates: int,
) -> list[dict[str, Any]]:
    report_candidates = loop_report.get('loop_candidates') or []
    if isinstance(report_candidates, list) and report_candidates:
        return report_candidates[:max_candidates]
    if not poses:
        return []
    return find_loop_candidates(
        poses,
        LoopAlignmentThresholds(),
        max_candidates=max_candidates,
    )


def _edit_source(
    run_dir: Path,
    poses: list[Any],
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Return exact edit identities and accepted loop constraints."""
    try:
        bundle_dir = resolve_bundle_dir(run_dir)
        loops = accepted_loop_edges(bundle_dir)
    except MapEditError as exc:
        return ({
            'editable': False,
            'bundle_dir': '',
            'map_bundle_sha256': '',
            'full_map_sha256': '',
            'pointcloud_map_sha256': '',
            'loop_edges_sha256': '',
            'reason': str(exc),
        }, [])
    pose_by_index = {int(pose.index): pose for pose in poses}
    decorated = []
    for loop in loops:
        source = pose_by_index.get(int(loop['from']))
        target = pose_by_index.get(int(loop['to']))
        item = dict(loop)
        if source is not None and target is not None:
            item['start'] = {'x': source.x, 'y': source.y, 'z': source.z}
            item['end'] = {'x': target.x, 'y': target.y, 'z': target.z}
            item['midpoint'] = [
                (source.x + target.x) * 0.5,
                (source.y + target.y) * 0.5,
                (source.z + target.z) * 0.5,
            ]
        decorated.append(item)
    return ({
        'editable': True,
        'bundle_dir': str(bundle_dir),
        'map_bundle_sha256': sha256_file(bundle_dir / 'map_bundle.yaml'),
        'full_map_sha256': sha256_file(
            bundle_dir / _artifact_relative_path(bundle_dir, 'full_map')
        ),
        'pointcloud_map_sha256': sha256_directory(
            bundle_dir / _artifact_relative_path(bundle_dir, 'pointcloud_map')
        ),
        'loop_edges_sha256': sha256_file(
            bundle_dir / _artifact_relative_path(bundle_dir, 'loop_edges')
        ),
        'reason': '',
    }, decorated)


def _artifact_relative_path(bundle_dir: Path, name: str) -> Path:
    manifest = yaml.safe_load(
        (bundle_dir / 'map_bundle.yaml').read_text(encoding='utf-8')
    )
    return Path(str(manifest['artifacts'][name]))


def _bounds(
    points: list[dict[str, Any]],
    trajectory: list[dict[str, float]],
    loop_candidates: list[dict[str, Any]],
) -> dict[str, float]:
    xyz: list[tuple[float, float, float]] = [
        (float(point['x']), float(point['y']), float(point['z']))
        for point in points
    ]
    xyz.extend(
        (float(pose['x']), float(pose['y']), float(pose['z']))
        for pose in trajectory
    )
    for candidate in loop_candidates:
        midpoint = candidate.get('midpoint') or []
        if len(midpoint) >= 3:
            xyz.append((float(midpoint[0]), float(midpoint[1]), float(midpoint[2])))
    if not xyz:
        return {}
    return {
        'min_x': min(item[0] for item in xyz),
        'max_x': max(item[0] for item in xyz),
        'min_y': min(item[1] for item in xyz),
        'max_y': max(item[1] for item in xyz),
        'min_z': min(item[2] for item in xyz),
        'max_z': max(item[2] for item in xyz),
    }


def _render_ply(points: list[dict[str, Any]]) -> str:
    lines = [
        'ply',
        'format ascii 1.0',
        f'element vertex {len(points)}',
        'property float x',
        'property float y',
        'property float z',
        'property uchar red',
        'property uchar green',
        'property uchar blue',
        'end_header',
    ]
    for point in points:
        red, green, blue = point['rgb']
        lines.append(
            f"{float(point['x']):.6f} {float(point['y']):.6f} {float(point['z']):.6f} "
            f'{red} {green} {blue}'
        )
    return '\n'.join(lines) + '\n'


def _render_html(
    *,
    manifest: dict[str, Any],
    points: list[dict[str, Any]],
    trajectory: list[dict[str, float]],
    session_trajectories: list[dict[str, Any]],
    loop_candidates: list[dict[str, Any]],
    accepted_loop_edges: list[dict[str, Any]],
) -> str:
    data = {
        'manifest': manifest,
        'points': points,
        'trajectory': trajectory,
        'session_trajectories': session_trajectories,
        'loop_candidates': loop_candidates,
        'accepted_loop_edges': accepted_loop_edges,
    }
    data_json = json.dumps(data, sort_keys=True).replace('</', '<\\/')
    template = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>lidar_slam 3D Map Review</title>
  <style>
    :root { color-scheme: dark; }
    * { box-sizing: border-box; }
    html, body { margin: 0; height: 100%; background: #111318; color: #e9eef6; font-family: system-ui, sans-serif; }
    #wrap { display: grid; grid-template-columns: 1fr 380px; height: 100%; }
    canvas { width: 100%; height: 100%; display: block; cursor: grab; background: #0b0d12; }
    aside { border-left: 1px solid #2b3240; padding: 16px; overflow: auto; background: #171b22; }
    h1 { font-size: 18px; margin: 0 0 8px; }
    h2 { font-size: 14px; margin: 20px 0 8px; color: #dce7f5; }
    dl { display: grid; grid-template-columns: auto 1fr; gap: 6px 12px; font-size: 12px; }
    dt { color: #99a6b8; }
    dd { margin: 0; word-break: break-word; }
    button { border: 1px solid #45536a; border-radius: 6px; padding: 7px 10px; background: #273246; color: #edf4ff; cursor: pointer; }
    button:hover:not(:disabled) { background: #34445f; }
    button.primary { width: 100%; border-color: #2673b8; background: #145e96; font-weight: 650; }
    button.danger { border-color: #9c4e57; background: #512d35; }
    button:disabled { opacity: .45; cursor: not-allowed; }
    input[type=number] { width: 100%; border: 1px solid #3c4657; border-radius: 4px; background: #0e1117; color: #eef4ff; padding: 5px; }
    .hint { color: #b7c2d0; font-size: 12px; line-height: 1.45; }
    .status { display: inline-block; padding: 2px 8px; border-radius: 999px; background: #244a32; color: #9df1b5; }
    .toolbar { display: flex; gap: 6px; flex-wrap: wrap; }
    .bounds { display: grid; grid-template-columns: 22px 1fr 1fr; gap: 5px; align-items: center; font-size: 12px; margin: 8px 0; }
    .bounds .head { color: #8e9aac; text-align: center; }
    .panel { border: 1px solid #2d3645; border-radius: 8px; padding: 10px; background: #12161d; }
    .message { min-height: 18px; margin: 8px 0; color: #9ed3ff; font-size: 12px; line-height: 1.4; }
    .message.error { color: #ff9ba4; }
    .operations, .loops { margin: 6px 0; padding: 0; list-style: none; font-size: 12px; }
    .operations li, .loops li { border-top: 1px solid #29313e; padding: 6px 0; }
    .loops label { display: flex; gap: 7px; align-items: start; }
    .session-legend { margin: 4px 0; padding: 0; list-style: none; font-size: 12px; }
    .session-legend li { display: flex; align-items: center; gap: 8px; padding: 3px 0; }
    .swatch { width: 18px; height: 4px; border-radius: 2px; flex: 0 0 auto; }
    code { white-space: pre-wrap; word-break: break-all; font-size: 11px; color: #bed8f5; }
    @media (max-width: 850px) { #wrap { grid-template-columns: 1fr; grid-template-rows: minmax(45vh, 1fr) auto; } aside { border-left: 0; border-top: 1px solid #2b3240; max-height: 55vh; } }
  </style>
</head>
<body>
  <div id="wrap">
    <canvas id="map"></canvas>
    <aside>
      <h1>lidar_slam 3D Map Review</h1>
      <p class="hint">Drag to rotate, wheel to zoom. Colored lines are trajectories, red is a detected revisit, and magenta is an accepted loop constraint.</p>
      <dl id="stats"></dl>
      <section id="session-section" hidden>
        <h2>Session paths</h2>
        <ul id="session-legend" class="session-legend"></ul>
      </section>
      <h2>Remove an unwanted region</h2>
      <div class="panel">
        <div class="toolbar">
          <button id="top-view">Top view</button>
          <button id="select-box">Select 2 corners</button>
          <button id="add-box" class="danger">Add removal</button>
        </div>
        <p class="hint">Select two map points for the XY corners, then adjust XYZ limits if needed. Orange points are the current removal preview.</p>
        <div class="bounds">
          <span></span><span class="head">minimum</span><span class="head">maximum</span>
          <span>X</span><input id="min-x" type="number" step="any"><input id="max-x" type="number" step="any">
          <span>Y</span><input id="min-y" type="number" step="any"><input id="max-y" type="number" step="any">
          <span>Z</span><input id="min-z" type="number" step="any"><input id="max-z" type="number" step="any">
        </div>
      </div>
      <h2>Accepted loop constraints</h2>
      <p class="hint">Uncheck a constraint to disable it. This requires retained backend input so the poses and map can be optimized again.</p>
      <ul id="loops" class="loops"></ul>
      <h2>Edit plan</h2>
      <div class="panel">
        <ul id="operations" class="operations"></ul>
        <div class="toolbar"><button id="undo">Undo last region</button><button id="clear">Clear regions</button></div>
        <p id="message" class="message"></p>
        <button id="download" class="primary">Download edit plan</button>
        <p class="hint">The plan does not alter this map. Apply it in a terminal to create and verify a separate candidate.</p>
        <code id="command"></code>
      </div>
    </aside>
  </div>
  <script id="preview-data" type="application/json">__PREVIEW_DATA__</script>
  <script>
    const data = JSON.parse(document.getElementById('preview-data').textContent);
    const canvas = document.getElementById('map');
    const ctx = canvas.getContext('2d');
    const stats = document.getElementById('stats');
    const sessionSection = document.getElementById('session-section');
    const sessionLegend = document.getElementById('session-legend');
    const source = ((data.manifest.edit || {}).source || {});
    const inputIds = ['min-x', 'max-x', 'min-y', 'max-y', 'min-z', 'max-z'];
    const inputs = Object.fromEntries(inputIds.map((id) => [id, document.getElementById(id)]));
    const operationsElement = document.getElementById('operations');
    const loopsElement = document.getElementById('loops');
    const message = document.getElementById('message');
    const command = document.getElementById('command');
    let yaw = -0.7, pitch = 0.85, zoom = 1.0, dragging = false, lastX = 0, lastY = 0;
    let selecting = false, selectedCorners = [], removalOperations = [];

    function resize() {
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(canvas.clientWidth * ratio));
      canvas.height = Math.max(1, Math.floor(canvas.clientHeight * ratio));
      draw();
    }

    function centerAndScale() {
      const b = data.manifest.bounds || {};
      const cx = ((b.min_x || 0) + (b.max_x || 0)) / 2;
      const cy = ((b.min_y || 0) + (b.max_y || 0)) / 2;
      const cz = ((b.min_z || 0) + (b.max_z || 0)) / 2;
      const span = Math.max(
        (b.max_x || 1) - (b.min_x || 0),
        (b.max_y || 1) - (b.min_y || 0),
        (b.max_z || 1) - (b.min_z || 0),
        1
      );
      return {cx, cy, cz, scale: Math.min(canvas.width, canvas.height) * 0.75 / span * zoom};
    }

    function project(p, cs) {
      const x0 = p.x - cs.cx, y0 = p.y - cs.cy, z0 = p.z - cs.cz;
      const cy = Math.cos(yaw), sy = Math.sin(yaw);
      const cp = Math.cos(pitch), sp = Math.sin(pitch);
      const x1 = cy * x0 - sy * y0;
      const y1 = sy * x0 + cy * y0;
      const z1 = z0;
      const y2 = cp * y1 - sp * z1;
      return {x: canvas.width / 2 + x1 * cs.scale, y: canvas.height / 2 - y2 * cs.scale, depth: sp * y1 + cp * z1};
    }

    function drawPolyline(items, color, width) {
      if (items.length < 2) return;
      const cs = centerAndScale();
      ctx.strokeStyle = color;
      ctx.lineWidth = width * (window.devicePixelRatio || 1);
      ctx.beginPath();
      items.forEach((item, index) => {
        const p = project(item, cs);
        if (index === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      });
      ctx.stroke();
    }

    function currentBox() {
      const values = inputIds.map((id) => Number(inputs[id].value));
      if (!values.every(Number.isFinite)) return null;
      const box = {min_xyz: [values[0], values[2], values[4]], max_xyz: [values[1], values[3], values[5]]};
      if (box.min_xyz.some((value, index) => value >= box.max_xyz[index])) return null;
      return box;
    }

    function inBox(point, box) {
      if (!box) return false;
      return ['x', 'y', 'z'].every((axis, index) => point[axis] >= box.min_xyz[index] && point[axis] <= box.max_xyz[index]);
    }

    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const cs = centerAndScale();
      const box = currentBox();
      const pts = data.points.map((p) => [project(p, cs), p]);
      pts.sort((a, b) => a[0].depth - b[0].depth);
      const radius = Math.max(1, Math.min(3, 1.4 * (window.devicePixelRatio || 1) * zoom));
      for (const [p, item] of pts) {
        const rgb = inBox(item, box) ? [255, 151, 51] : item.rgb;
        ctx.fillStyle = `rgb(${rgb[0]},${rgb[1]},${rgb[2]})`;
        ctx.fillRect(p.x, p.y, radius, radius);
      }
      if (data.session_trajectories.length) {
        for (const session of data.session_trajectories) {
          if (session.visible === false) continue;
          drawPolyline(session.trajectory, session.color, 2);
        }
      } else {
        drawPolyline(data.trajectory, '#63d6ff', 2);
      }
      ctx.fillStyle = '#ff4b5c';
      for (const c of data.loop_candidates) {
        const m = c.midpoint || [];
        if (m.length < 3) continue;
        const p = project({x: m[0], y: m[1], z: m[2]}, cs);
        ctx.beginPath();
        ctx.arc(p.x, p.y, 7 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
        ctx.fill();
      }
      for (const edge of data.accepted_loop_edges) {
        if (!edge.start || !edge.end) continue;
        drawPolyline([edge.start, edge.end], '#f275ff', 2);
      }
    }

    function setMessage(text, error = false) {
      message.textContent = text;
      message.className = error ? 'message error' : 'message';
    }

    function renderSessionLegend() {
      if (!data.session_trajectories.length) return;
      sessionSection.hidden = false;
      for (const session of data.session_trajectories) {
        const item = document.createElement('li');
        const toggle = document.createElement('input');
        toggle.type = 'checkbox'; toggle.checked = true; session.visible = true;
        toggle.addEventListener('change', () => { session.visible = toggle.checked; draw(); });
        const swatch = document.createElement('span');
        swatch.className = 'swatch'; swatch.style.background = session.color;
        const label = document.createElement('span');
        label.textContent = `${session.session_index}: ${session.name} (${session.trajectory.length} poses)`;
        item.append(toggle, swatch, label); sessionLegend.appendChild(item);
      }
    }

    function setBoundsFromCorners() {
      if (selectedCorners.length !== 2) return;
      const bounds = data.manifest.bounds || {};
      inputs['min-x'].value = Math.min(selectedCorners[0].x, selectedCorners[1].x).toFixed(3);
      inputs['max-x'].value = Math.max(selectedCorners[0].x, selectedCorners[1].x).toFixed(3);
      inputs['min-y'].value = Math.min(selectedCorners[0].y, selectedCorners[1].y).toFixed(3);
      inputs['max-y'].value = Math.max(selectedCorners[0].y, selectedCorners[1].y).toFixed(3);
      inputs['min-z'].value = Number(bounds.min_z || 0).toFixed(3);
      inputs['max-z'].value = Number(bounds.max_z || 0).toFixed(3);
      selecting = false;
      canvas.style.cursor = 'grab';
      setMessage('Region selected. Adjust the bounds, then choose “Add removal”.');
      draw();
    }

    function nearestPoint(event) {
      const ratio = window.devicePixelRatio || 1;
      const targetX = event.offsetX * ratio, targetY = event.offsetY * ratio;
      const cs = centerAndScale();
      let nearest = null, distance = Infinity;
      for (const point of data.points) {
        const projected = project(point, cs);
        const candidate = Math.hypot(projected.x - targetX, projected.y - targetY);
        if (candidate < distance) { nearest = point; distance = candidate; }
      }
      return nearest;
    }

    function disabledLoopOperations() {
      return [...loopsElement.querySelectorAll('input[type=checkbox]')]
        .filter((input) => !input.checked)
        .map((input) => ({
          id: `disable-loop-${input.dataset.from}-${input.dataset.to}`,
          type: 'disable_loop_edge',
          from: Number(input.dataset.from),
          to: Number(input.dataset.to),
        }));
    }

    function allOperations() { return [...removalOperations, ...disabledLoopOperations()]; }

    function renderOperations() {
      operationsElement.replaceChildren();
      const operations = allOperations();
      if (!operations.length) {
        const item = document.createElement('li');
        item.textContent = 'No edits selected yet.';
        operationsElement.appendChild(item);
      }
      for (const operation of operations) {
        const item = document.createElement('li');
        item.textContent = operation.type === 'remove_box'
          ? `Remove box: ${operation.min_xyz.join(', ')} → ${operation.max_xyz.join(', ')}`
          : `Disable accepted loop: ${operation.from} → ${operation.to}`;
        operationsElement.appendChild(item);
      }
    }

    function renderLoops() {
      loopsElement.replaceChildren();
      if (!data.accepted_loop_edges.length) {
        const item = document.createElement('li');
        item.textContent = 'No accepted loop constraints in this map.';
        loopsElement.appendChild(item);
        return;
      }
      for (const edge of data.accepted_loop_edges) {
        const item = document.createElement('li');
        const label = document.createElement('label');
        const input = document.createElement('input');
        input.type = 'checkbox'; input.checked = true;
        input.dataset.from = edge.from; input.dataset.to = edge.to;
        input.addEventListener('change', renderOperations);
        const text = document.createElement('span');
        text.textContent = `Keep ${edge.from} → ${edge.to} (fitness ${Number(edge.fitness).toFixed(4)})`;
        label.append(input, text); item.appendChild(label); loopsElement.appendChild(item);
      }
    }

    function appendStat(name, value, status = false) {
      const term = document.createElement('dt'); term.textContent = name;
      const detail = document.createElement('dd');
      if (status) { const badge = document.createElement('span'); badge.className = 'status'; badge.textContent = value; detail.appendChild(badge); }
      else detail.textContent = value;
      stats.append(term, detail);
    }

    canvas.addEventListener('mousedown', (event) => {
      if (selecting) {
        const point = nearestPoint(event);
        if (point) selectedCorners.push(point);
        if (selectedCorners.length === 2) setBoundsFromCorners();
        else setMessage('First corner selected. Choose the opposite corner.');
        return;
      }
      dragging = true; lastX = event.clientX; lastY = event.clientY; canvas.style.cursor = 'grabbing';
    });
    window.addEventListener('mouseup', () => { dragging = false; canvas.style.cursor = selecting ? 'crosshair' : 'grab'; });
    window.addEventListener('mousemove', (event) => {
      if (!dragging) return;
      yaw += (event.clientX - lastX) * 0.008;
      pitch = Math.max(-1.35, Math.min(1.35, pitch + (event.clientY - lastY) * 0.008));
      lastX = event.clientX; lastY = event.clientY;
      draw();
    });
    canvas.addEventListener('wheel', (event) => {
      event.preventDefault();
      zoom = Math.max(0.15, Math.min(8, zoom * Math.exp(-event.deltaY * 0.001)));
      draw();
    }, {passive: false});

    document.getElementById('top-view').addEventListener('click', () => { yaw = 0; pitch = 0; draw(); });
    document.getElementById('select-box').addEventListener('click', () => {
      yaw = 0; pitch = 0; selecting = true; selectedCorners = []; canvas.style.cursor = 'crosshair';
      setMessage('Select the first XY corner on the map.'); draw();
    });
    document.getElementById('add-box').addEventListener('click', () => {
      const box = currentBox();
      if (!box) { setMessage('Enter valid bounds with minimum below maximum on X, Y, and Z.', true); return; }
      removalOperations.push({id: `remove-box-${removalOperations.length + 1}`, type: 'remove_box', ...box});
      setMessage('Removal added to the plan. The source map is still unchanged.'); renderOperations();
    });
    document.getElementById('undo').addEventListener('click', () => { removalOperations.pop(); renderOperations(); });
    document.getElementById('clear').addEventListener('click', () => { removalOperations = []; renderOperations(); });
    inputIds.forEach((id) => inputs[id].addEventListener('input', draw));

    document.getElementById('download').addEventListener('click', () => {
      if (!source.editable) { setMessage(source.reason || 'This preview is not backed by an editable map bundle.', true); return; }
      const operations = allOperations();
      if (!operations.length) { setMessage('Select at least one removal or disabled loop first.', true); return; }
      const plan = {
        schema_version: 1,
        created_at: new Date().toISOString(),
        source: {
          map_bundle_sha256: source.map_bundle_sha256,
          full_map_sha256: source.full_map_sha256,
          pointcloud_map_sha256: source.pointcloud_map_sha256,
          loop_edges_sha256: source.loop_edges_sha256,
        },
        operations,
      };
      const blob = new Blob([JSON.stringify(plan, null, 2) + '\\n'], {type: 'application/json'});
      const link = document.createElement('a');
      link.href = URL.createObjectURL(blob);
      link.download = `map-edit-plan-${new Date().toISOString().replace(/[:.]/g, '-')}.json`;
      link.click(); URL.revokeObjectURL(link.href);
      setMessage('Edit plan downloaded. Run the command below to create a verified candidate.');
    });

    const counts = data.manifest.counts || {};
    appendStat('Status', data.manifest.status, true);
    appendStat('Cloud points', String(counts.cloud_points || 0));
    appendStat('Preview points', String(counts.html_points || 0));
    appendStat('Trajectory poses', String(counts.trajectory_poses || 0));
    appendStat('Map sessions', String(counts.map_sessions || 1));
    appendStat('Revisit candidates', String(counts.loop_candidates || 0));
    appendStat('Accepted loops', String(counts.accepted_loop_edges || 0));
    appendStat('Editable', source.editable ? 'yes' : 'no');
    const shellSource = source.bundle_dir || '<map_bundle>';
    command.textContent = `lidarslam-map edit "${shellSource}" --plan <downloaded-plan.json> --output-dir "${shellSource}_edited"`;
    document.getElementById('download').disabled = !source.editable;
    if (!source.editable) setMessage(source.reason || 'Editing requires map_bundle.yaml.', true);
    renderSessionLegend(); renderLoops(); renderOperations();
    window.addEventListener('resize', resize);
    resize();
  </script>
</body>
</html>
"""
    return template.replace('__PREVIEW_DATA__', data_json)


def _next_actions(
    points: list[dict[str, Any]],
    trajectory: list[dict[str, float]],
    loop_candidates: list[dict[str, Any]],
) -> list[str]:
    actions = []
    if not points:
        actions.append('Provide a readable pointcloud_map directory before opening the preview.')
    if not trajectory:
        actions.append('Provide a TUM trajectory if the preview should show the robot path.')
    if not loop_candidates:
        actions.append('Run loop-alignment analysis first if loop candidate markers are needed.')
    return actions


if __name__ == "__main__":
    raise SystemExit("mid360_robot_3d_map_preview is a library module; import it instead of running it directly.")
