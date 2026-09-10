#!/usr/bin/env python3
"""Render the GLIM MID-360 sample HTML report from the latest run."""
import argparse
import json
import math
from html import escape
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output"
REPORT = OUTPUT / "glim_sample_report.html"


def _parser():
    parser = argparse.ArgumentParser(
        description='Render glim_sample_report.html from the latest '
                    'glim_mid360_sample_* run.')
    parser.add_argument('--output-root', type=Path, default=OUTPUT,
                        help=f'run directory root (default: {OUTPUT})')
    parser.add_argument('--report', type=Path, default=REPORT,
                        help=f'HTML output path (default: {REPORT})')
    return parser


def load_traj(path: Path):
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text().splitlines():
        parts = line.strip().split()
        if len(parts) < 8:
            continue
        try:
            t, x, y, z, qx, qy, qz, qw = map(float, parts[:8])
        except ValueError:
            continue
        rows.append(
            {
                "t": t,
                "x": x,
                "y": y,
                "z": z,
                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw,
            }
        )
    return rows


def quat_to_rpy(qx, qy, qz, qw):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def path_length(rows):
    length = 0.0
    for a, b in zip(rows, rows[1:]):
        length += math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"]))
    return length


def bbox(rows, key):
    vals = [r[key] for r in rows]
    return (min(vals), max(vals)) if vals else (0.0, 0.0)


def find_latest_run(output_root=OUTPUT):
    runs = sorted(output_root.glob("glim_mid360_sample_*"))
    return runs[-1] if runs else None


def load_log_markers(path: Path):
    keys = [
        "waiting for odometry estimation",
        "waiting for local mapping",
        "waiting for global mapping",
        "IMU bias estimation seems inaccurate",
        "previous submap has only a small overlap",
        "error",
        "critical",
    ]
    hits = []
    if not path.exists():
        return hits
    for i, line in enumerate(path.read_text(errors="replace").splitlines(), start=1):
        if any(k in line for k in keys):
            hits.append((i, line))
    return hits


def verdict(rows):
    if len(rows) < 1000:
        return "NG", "trajectory is too short"
    length = path_length(rows)
    if length < 100.0:
        return "NG", "path length is too short"
    return "OK", "trajectory length and density look healthy"


def fmt(x, digits=3):
    return f"{x:.{digits}f}"


def make_xy_svg(rows, width=900, height=420, margin=24):
    if len(rows) < 2:
        return '<div class="empty">trajectory missing</div>'
    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1e-9)
    span_y = max(max_y - min_y, 1e-9)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

    def proj(r):
        x = margin + (r["x"] - min_x) * scale
        y = height - margin - (r["y"] - min_y) * scale
        return f"{x:.2f},{y:.2f}"

    pts = " ".join(proj(r) for r in rows)
    start = proj(rows[0])
    end = proj(rows[-1])
    return (
        f'<svg viewBox="0 0 {width} {height}" class="xy">'
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#f8fbff" stroke="#d8e3ef"/>'
        f'<polyline points="{pts}" fill="none" stroke="#0b6bcb" stroke-width="2.25" vector-effect="non-scaling-stroke"/>'
        f'<circle cx="{start.split(",")[0]}" cy="{start.split(",")[1]}" r="5" fill="#0a7c2f"/>'
        f'<circle cx="{end.split(",")[0]}" cy="{end.split(",")[1]}" r="5" fill="#b42318"/>'
        f'<text x="{margin}" y="18" fill="#0a7c2f" font-size="12">start</text>'
        f'<text x="{width - 50}" y="18" fill="#b42318" font-size="12">end</text>'
        "</svg>"
    )


def main(argv=None):
    args = _parser().parse_args(argv)
    run_dir = find_latest_run(args.output_root)
    if run_dir is None:
        raise SystemExit(
            f"no glim_mid360_sample_* run found under {args.output_root}")

    traj_path = run_dir / "dump" / "traj_lidar.txt"
    log_path = run_dir / "glim_rosbag.log"
    rows = load_traj(traj_path)
    status, reason = verdict(rows)
    markers = load_log_markers(log_path)

    times = [r["t"] - rows[0]["t"] for r in rows]
    xs = [r["x"] for r in rows]
    ys = [r["y"] for r in rows]
    zs = [r["z"] for r in rows]
    rpys = [quat_to_rpy(r["qx"], r["qy"], r["qz"], r["qw"]) for r in rows]
    rolls = [math.degrees(v[0]) for v in rpys]
    pitches = [math.degrees(v[1]) for v in rpys]
    yaws = [math.degrees(v[2]) for v in rpys]

    length = path_length(rows)
    duration = times[-1] if times else 0.0
    avg_speed = length / duration if duration > 1e-9 else 0.0

    summary = {
        "run_dir": str(run_dir.relative_to(ROOT)),
        "traj_path": str(traj_path.relative_to(ROOT)),
        "log_path": str(log_path.relative_to(ROOT)),
        "status": status,
        "reason": reason,
        "traj_lines": len(rows),
        "path_length_m": length,
        "duration_s": duration,
        "avg_speed_mps": avg_speed,
        "bbox_x": bbox(rows, "x"),
        "bbox_y": bbox(rows, "y"),
        "bbox_z": bbox(rows, "z"),
        "marker_count": len(markers),
    }
    (run_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    plot_data = {
        "t": times,
        "x": xs,
        "y": ys,
        "z": zs,
        "roll": rolls,
        "pitch": pitches,
        "yaw": yaws,
    }

    html = f"""<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>GLIM Sample Report</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 24px; color: #102030; background: #eef4f8; }}
    section {{ background: #fff; border: 1px solid #d7e0e8; border-radius: 14px; padding: 20px; margin-bottom: 18px; }}
    h1,h2,h3 {{ margin: 0 0 12px; }}
    .lead {{ margin: 0 0 10px; font-size: 16px; }}
    .ok {{ color: #0a7c2f; font-weight: 700; }}
    .ng {{ color: #b42318; font-weight: 700; }}
    .grid {{ display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; }}
    .card {{ background: #f7fafc; border: 1px solid #d7e0e8; border-radius: 12px; padding: 12px; }}
    .label {{ font-size: 12px; color: #486070; }}
    .value {{ font-size: 22px; font-weight: 700; margin-top: 4px; }}
    .mono {{ font-family: ui-monospace, SFMono-Regular, Menlo, monospace; white-space: pre-wrap; font-size: 12px; }}
    .xy {{ width: 100%; height: auto; }}
    .plot {{ width: 100%; height: 520px; }}
    .small {{ font-size: 13px; color: #486070; }}
    table {{ border-collapse: collapse; width: 100%; }}
    th, td {{ border: 1px solid #d7e0e8; padding: 8px; text-align: left; vertical-align: top; }}
    th {{ background: #f3f7fa; width: 220px; }}
    @media (max-width: 900px) {{
      .grid {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }}
    }}
  </style>
</head>
<body>
  <section>
    <h1>GLIM Sample Report</h1>
    <p class="lead">GLIM 公式 Livox MID360 sample bag の fresh 実行結果です。</p>
    <div class="{ 'ok' if status == 'OK' else 'ng' }">Verdict: {escape(status)} - {escape(reason)}</div>
    <div class="small">run: {escape(str(run_dir.relative_to(ROOT)))}</div>
  </section>

  <section>
    <h2>Summary</h2>
    <div class="grid">
      <div class="card"><div class="label">Trajectory lines</div><div class="value">{len(rows)}</div></div>
      <div class="card"><div class="label">Path length</div><div class="value">{fmt(length, 1)} m</div></div>
      <div class="card"><div class="label">Duration</div><div class="value">{fmt(duration, 1)} s</div></div>
      <div class="card"><div class="label">Avg speed</div><div class="value">{fmt(avg_speed, 2)} m/s</div></div>
    </div>
  </section>

  <section>
    <h2>XY trajectory</h2>
    {make_xy_svg(rows)}
  </section>

  <section>
    <h2>Interactive 3D XYZ</h2>
    <div id="plot3d" class="plot"></div>
  </section>

  <section>
    <h2>XYZRPY time series</h2>
    <div id="xyzrpy" class="plot" style="height: 860px;"></div>
  </section>

  <section>
    <h2>Run details</h2>
    <table>
      <tr><th>Status</th><td>{escape(status)}</td></tr>
      <tr><th>Reason</th><td>{escape(reason)}</td></tr>
      <tr><th>Trajectory</th><td class="mono">{escape(str(traj_path.relative_to(ROOT)))}</td></tr>
      <tr><th>Log</th><td class="mono">{escape(str(log_path.relative_to(ROOT)))}</td></tr>
      <tr><th>BBox X</th><td>{fmt(summary['bbox_x'][0])} .. {fmt(summary['bbox_x'][1])}</td></tr>
      <tr><th>BBox Y</th><td>{fmt(summary['bbox_y'][0])} .. {fmt(summary['bbox_y'][1])}</td></tr>
      <tr><th>BBox Z</th><td>{fmt(summary['bbox_z'][0])} .. {fmt(summary['bbox_z'][1])}</td></tr>
      <tr><th>Marker count</th><td>{len(markers)}</td></tr>
    </table>
  </section>

  <section>
    <h2>Log markers</h2>
    <div class="mono">{escape(chr(10).join(f"{ln}: {line}" for ln, line in markers[-80:]) if markers else "no marker lines matched")}</div>
  </section>

  <script>
    const data = {json.dumps(plot_data)};
    Plotly.newPlot('plot3d', [{{
      type: 'scatter3d',
      mode: 'lines',
      x: data.x,
      y: data.y,
      z: data.z,
      line: {{ color: '#0b6bcb', width: 4 }},
      name: 'GLIM'
    }}], {{
      margin: {{l: 0, r: 0, t: 0, b: 0}},
      scene: {{
        xaxis: {{title: 'X [m]'}},
        yaxis: {{title: 'Y [m]'}},
        zaxis: {{title: 'Z [m]'}},
        aspectmode: 'data'
      }}
    }}, {{responsive: true}});

    const names = ['x','y','z','roll','pitch','yaw'];
    const colors = ['#0b6bcb','#12805c','#c77d00','#b42318','#7a3cff','#005f73'];
    const units = ['m','m','m','deg','deg','deg'];
    const traces = names.map((name, i) => {{
      const axis = i + 1;
      return {{
        type: 'scatter',
        mode: 'lines',
        x: data.t,
        y: data[name],
        name: `${{name}} [${{units[i]}}]`,
        line: {{color: colors[i], width: 1.8}},
        xaxis: `x${{axis}}`,
        yaxis: `y${{axis}}`
      }};
    }});
    const layout = {{
      grid: {{rows: 3, columns: 2, pattern: 'independent'}},
      margin: {{l: 52, r: 20, t: 30, b: 40}},
      showlegend: false,
      annotations: names.map((name, i) => {{
        const row = Math.floor(i / 2);
        const col = i % 2;
        return {{
          text: `${{name.toUpperCase()}} [${{units[i]}}]`,
          xref: 'paper',
          yref: 'paper',
          x: col === 0 ? 0.19 : 0.81,
          y: 1.0 - row * 0.335,
          yanchor: 'bottom',
          showarrow: false,
          font: {{size: 13}}
        }};
      }})
    }};
    Plotly.newPlot('xyzrpy', traces, layout, {{responsive: true}});
  </script>
</body>
</html>
"""

    args.report.write_text(html, encoding="utf-8")
    print(args.report)


if __name__ == "__main__":
    main()
