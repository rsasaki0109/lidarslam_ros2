#!/usr/bin/env python3
"""Browser preview artifacts for MID-360 map outputs."""

from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

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
        loop_report = _load_loop_alignment(run_dir, options.loop_alignment_path)
        loop_candidates = _loop_candidates(loop_report, poses, max(1, options.max_loop_candidates))
        overlay = {
            'trajectory_path': str(trajectory_path) if trajectory_path else '',
            'trajectory': trajectory,
            'loop_candidates': loop_candidates,
        }
        bounds = _bounds(points, trajectory, loop_candidates)
        html_points = _sample_items(points, max(1, options.html_max_points))
        output_dir.mkdir(parents=True, exist_ok=True)
        ply_path = output_dir / MAP_PREVIEW_PLY
        overlay_path = output_dir / MAP_PREVIEW_OVERLAY_JSON
        html_path = output_dir / MAP_PREVIEW_HTML
        json_path = output_dir / MAP_PREVIEW_JSON
        ply_path.write_text(_render_ply(points), encoding='utf-8')
        overlay_path.write_text(payload_to_json(overlay) + '\n', encoding='utf-8')
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
                'trajectory_poses': len(trajectory),
                'loop_candidates': len(loop_candidates),
            },
            'bounds': bounds,
            'cloud': {key: value for key, value in cloud.items() if key != 'points'},
            'next_actions': _next_actions(points, trajectory, loop_candidates),
        }
        html_path.write_text(
            _render_html(
                manifest=manifest,
                points=html_points,
                trajectory=trajectory,
                loop_candidates=loop_candidates,
            ),
            encoding='utf-8',
        )
        json_path.write_text(payload_to_json(manifest) + '\n', encoding='utf-8')
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
    loop_candidates: list[dict[str, Any]],
) -> str:
    data = {
        'manifest': manifest,
        'points': points,
        'trajectory': trajectory,
        'loop_candidates': loop_candidates,
    }
    data_json = json.dumps(data, sort_keys=True).replace('</', '<\\/')
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MID-360 3D Map Preview</title>
  <style>
    html, body {{ margin: 0; height: 100%; background: #111318; color: #e9eef6; font-family: system-ui, sans-serif; }}
    #wrap {{ display: grid; grid-template-columns: 1fr 320px; height: 100%; }}
    canvas {{ width: 100%; height: 100%; display: block; cursor: grab; background: #0b0d12; }}
    aside {{ border-left: 1px solid #2b3240; padding: 16px; overflow: auto; background: #171b22; }}
    h1 {{ font-size: 18px; margin: 0 0 12px; }}
    dl {{ display: grid; grid-template-columns: auto 1fr; gap: 6px 12px; font-size: 13px; }}
    dt {{ color: #99a6b8; }}
    dd {{ margin: 0; word-break: break-word; }}
    .hint {{ color: #b7c2d0; font-size: 13px; line-height: 1.45; }}
    .status {{ display: inline-block; padding: 2px 8px; border-radius: 999px; background: #244a32; color: #9df1b5; }}
    @media (max-width: 800px) {{ #wrap {{ grid-template-columns: 1fr; grid-template-rows: 1fr auto; }} aside {{ border-left: 0; border-top: 1px solid #2b3240; max-height: 42vh; }} }}
  </style>
</head>
<body>
  <div id="wrap">
    <canvas id="map"></canvas>
    <aside>
      <h1>MID-360 3D Map Preview</h1>
      <p class="hint">Drag to rotate, wheel to zoom. Cloud points are height-colored; trajectory is cyan; loop candidates are red.</p>
      <dl id="stats"></dl>
    </aside>
  </div>
  <script id="preview-data" type="application/json">{data_json}</script>
  <script>
    const data = JSON.parse(document.getElementById('preview-data').textContent);
    const canvas = document.getElementById('map');
    const ctx = canvas.getContext('2d');
    const stats = document.getElementById('stats');
    let yaw = -0.7, pitch = 0.85, zoom = 1.0, dragging = false, lastX = 0, lastY = 0;

    function resize() {{
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.max(1, Math.floor(canvas.clientWidth * ratio));
      canvas.height = Math.max(1, Math.floor(canvas.clientHeight * ratio));
      draw();
    }}

    function centerAndScale() {{
      const b = data.manifest.bounds || {{}};
      const cx = ((b.min_x || 0) + (b.max_x || 0)) / 2;
      const cy = ((b.min_y || 0) + (b.max_y || 0)) / 2;
      const cz = ((b.min_z || 0) + (b.max_z || 0)) / 2;
      const span = Math.max(
        (b.max_x || 1) - (b.min_x || 0),
        (b.max_y || 1) - (b.min_y || 0),
        (b.max_z || 1) - (b.min_z || 0),
        1
      );
      return {{cx, cy, cz, scale: Math.min(canvas.width, canvas.height) * 0.75 / span * zoom}};
    }}

    function project(p, cs) {{
      const x0 = p.x - cs.cx, y0 = p.y - cs.cy, z0 = p.z - cs.cz;
      const cy = Math.cos(yaw), sy = Math.sin(yaw);
      const cp = Math.cos(pitch), sp = Math.sin(pitch);
      const x1 = cy * x0 - sy * y0;
      const y1 = sy * x0 + cy * y0;
      const z1 = z0;
      const y2 = cp * y1 - sp * z1;
      return {{x: canvas.width / 2 + x1 * cs.scale, y: canvas.height / 2 - y2 * cs.scale, depth: sp * y1 + cp * z1}};
    }}

    function drawPolyline(items, color, width) {{
      if (items.length < 2) return;
      const cs = centerAndScale();
      ctx.strokeStyle = color;
      ctx.lineWidth = width * (window.devicePixelRatio || 1);
      ctx.beginPath();
      items.forEach((item, index) => {{
        const p = project(item, cs);
        if (index === 0) ctx.moveTo(p.x, p.y); else ctx.lineTo(p.x, p.y);
      }});
      ctx.stroke();
    }}

    function draw() {{
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      const cs = centerAndScale();
      const pts = data.points.map((p) => [project(p, cs), p.rgb]);
      pts.sort((a, b) => a[0].depth - b[0].depth);
      const radius = Math.max(1, Math.min(3, 1.4 * (window.devicePixelRatio || 1) * zoom));
      for (const [p, rgb] of pts) {{
        ctx.fillStyle = `rgb(${{rgb[0]}},${{rgb[1]}},${{rgb[2]}})`;
        ctx.fillRect(p.x, p.y, radius, radius);
      }}
      drawPolyline(data.trajectory, '#63d6ff', 2);
      ctx.fillStyle = '#ff4b5c';
      for (const c of data.loop_candidates) {{
        const m = c.midpoint || [];
        if (m.length < 3) continue;
        const p = project({{x: m[0], y: m[1], z: m[2]}}, cs);
        ctx.beginPath();
        ctx.arc(p.x, p.y, 7 * (window.devicePixelRatio || 1), 0, Math.PI * 2);
        ctx.fill();
      }}
    }}

    canvas.addEventListener('mousedown', (event) => {{ dragging = true; lastX = event.clientX; lastY = event.clientY; canvas.style.cursor = 'grabbing'; }});
    window.addEventListener('mouseup', () => {{ dragging = false; canvas.style.cursor = 'grab'; }});
    window.addEventListener('mousemove', (event) => {{
      if (!dragging) return;
      yaw += (event.clientX - lastX) * 0.008;
      pitch = Math.max(-1.35, Math.min(1.35, pitch + (event.clientY - lastY) * 0.008));
      lastX = event.clientX; lastY = event.clientY;
      draw();
    }});
    canvas.addEventListener('wheel', (event) => {{
      event.preventDefault();
      zoom = Math.max(0.15, Math.min(8, zoom * Math.exp(-event.deltaY * 0.001)));
      draw();
    }}, {{passive: false}});

    const counts = data.manifest.counts || {{}};
    stats.innerHTML = `
      <dt>Status</dt><dd><span class="status">${{data.manifest.status}}</span></dd>
      <dt>Cloud points</dt><dd>${{counts.cloud_points || 0}}</dd>
      <dt>Preview points</dt><dd>${{counts.html_points || 0}}</dd>
      <dt>Trajectory poses</dt><dd>${{counts.trajectory_poses || 0}}</dd>
      <dt>Loop candidates</dt><dd>${{counts.loop_candidates || 0}}</dd>
      <dt>Pointcloud map</dt><dd>${{data.manifest.pointcloud_map_dir || ''}}</dd>
      <dt>Trajectory</dt><dd>${{data.manifest.trajectory_path || ''}}</dd>
    `;
    window.addEventListener('resize', resize);
    resize();
  </script>
</body>
</html>
"""


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
