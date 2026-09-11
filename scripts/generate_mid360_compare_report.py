#!/usr/bin/env python3
"""Render the GLIM-vs-lidarslam MID-360 comparison HTML report."""
from __future__ import annotations

import argparse
import bisect
import json
import math
from html import escape
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output"
REPORT = OUTPUT / "mid360_compare_report.html"


def _parser():
    parser = argparse.ArgumentParser(
        description='Render mid360_compare_report.html from the latest '
                    'GLIM and lidarslam MID-360 runs.')
    parser.add_argument('--output-root', type=Path, default=OUTPUT,
                        help=f'run directory root (default: {OUTPUT})')
    parser.add_argument('--report', type=Path, default=REPORT,
                        help=f'HTML output path (default: {REPORT})')
    return parser


def load_tum(path: Path):
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


def path_length(rows):
    return sum(
        math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"]))
        for a, b in zip(rows, rows[1:])
    )


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


def quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def rotation_matrix_to_quat(rot):
    trace = np.trace(rot)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    else:
        i = int(np.argmax(np.diag(rot)))
        if i == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s
    q = np.array([qx, qy, qz, qw])
    q /= np.linalg.norm(q)
    return q


def match_rows(ref_rows, est_rows, tolerance=0.05):
    est_times = [r["t"] for r in est_rows]
    pairs = []
    for ref in ref_rows:
        idx = bisect.bisect_left(est_times, ref["t"])
        candidates = []
        if idx < len(est_rows):
            candidates.append(est_rows[idx])
        if idx > 0:
            candidates.append(est_rows[idx - 1])
        best = None
        best_dt = None
        for cand in candidates:
            dt = abs(cand["t"] - ref["t"])
            if best is None or dt < best_dt:
                best = cand
                best_dt = dt
        if best is not None and best_dt is not None and best_dt <= tolerance:
            pairs.append((ref, best))
    return pairs


def rigid_align(pairs):
    ref = np.array([[a["x"], a["y"], a["z"]] for a, _ in pairs], dtype=float)
    est = np.array([[b["x"], b["y"], b["z"]] for _, b in pairs], dtype=float)
    ref_centroid = ref.mean(axis=0)
    est_centroid = est.mean(axis=0)
    w = (est - est_centroid).T @ (ref - ref_centroid)
    u, _, vt = np.linalg.svd(w)
    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1, :] *= -1.0
        rot = vt.T @ u.T
    trans = ref_centroid - rot @ est_centroid
    return rot, trans


def apply_alignment(rows, rot, trans):
    q_rot = rotation_matrix_to_quat(rot)
    aligned = []
    for row in rows:
        pos = np.array([row["x"], row["y"], row["z"]], dtype=float)
        pos_aligned = rot @ pos + trans
        q_est = np.array([row["qx"], row["qy"], row["qz"], row["qw"]], dtype=float)
        q_aligned = quat_multiply(q_rot, q_est)
        q_aligned /= np.linalg.norm(q_aligned)
        aligned.append(
            {
                **row,
                "x": float(pos_aligned[0]),
                "y": float(pos_aligned[1]),
                "z": float(pos_aligned[2]),
                "qx": float(q_aligned[0]),
                "qy": float(q_aligned[1]),
                "qz": float(q_aligned[2]),
                "qw": float(q_aligned[3]),
            }
        )
    return aligned


def find_latest_any(patterns, output_root=OUTPUT):
    candidates = []
    for pattern in patterns:
        candidates.extend(output_root.glob(pattern))
    candidates = [path for path in candidates if path.exists()]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def make_xy_svg(ref_rows, est_rows, width=960, height=420, margin=24):
    all_rows = ref_rows + est_rows
    if len(all_rows) < 2:
        return '<div class="empty">trajectory missing</div>'
    xs = [r["x"] for r in all_rows]
    ys = [r["y"] for r in all_rows]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1e-9)
    span_y = max(max_y - min_y, 1e-9)
    scale = min((width - 2 * margin) / span_x, (height - 2 * margin) / span_y)

    def proj(row):
        x = margin + (row["x"] - min_x) * scale
        y = height - margin - (row["y"] - min_y) * scale
        return f"{x:.2f},{y:.2f}"

    ref_pts = " ".join(proj(r) for r in ref_rows)
    est_pts = " ".join(proj(r) for r in est_rows)
    return (
        f'<svg viewBox="0 0 {width} {height}" class="xy">'
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#fbfcfd" stroke="#d8e3ef"/>'
        f'<polyline points="{ref_pts}" fill="none" stroke="#0b6bcb" stroke-width="2.3" vector-effect="non-scaling-stroke"/>'
        f'<polyline points="{est_pts}" fill="none" stroke="#bc4b2f" stroke-width="2.1" vector-effect="non-scaling-stroke"/>'
        f'<text x="{margin}" y="18" fill="#0b6bcb" font-size="12">GLIM</text>'
        f'<text x="{margin + 60}" y="18" fill="#bc4b2f" font-size="12">lidarslam</text>'
        "</svg>"
    )


def main(argv=None):
    args = _parser().parse_args(argv)
    glim_dir = find_latest_any(["glim_mid360_sample_*"], args.output_root)
    lid_dir = find_latest_any(
        [
            "lidarslam_mid360_auto_*",
            "lidarslam_mid360_noimu_nograph_fix_*",
            "lidarslam_mid360_noimu_*",
            "lidarslam_mid360_clean_*",
        ],
        args.output_root,
    )
    if glim_dir is None or lid_dir is None:
        raise SystemExit(
            f"required MID360 runs not found under {args.output_root}")

    glim_rows = load_tum(glim_dir / "dump" / "traj_lidar.txt")
    lid_rows = load_tum(lid_dir / "traj_lidarslam.tum")
    if not glim_rows or not lid_rows:
        raise SystemExit("trajectory file missing")

    pairs = match_rows(glim_rows, lid_rows, tolerance=0.05)
    if len(pairs) < 10:
        pairs = match_rows(glim_rows, lid_rows, tolerance=0.15)
    if len(pairs) < 10:
        raise SystemExit("not enough matched poses")

    rot, trans = rigid_align(pairs)
    lid_aligned = apply_alignment(lid_rows, rot, trans)
    aligned_pairs = match_rows(glim_rows, lid_aligned, tolerance=0.05)
    if len(aligned_pairs) < 10:
        aligned_pairs = match_rows(glim_rows, lid_aligned, tolerance=0.15)

    errors = []
    t_ref = []
    glim_xyz = {"x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": []}
    lid_xyz = {"x": [], "y": [], "z": [], "roll": [], "pitch": [], "yaw": []}
    for ref, est in aligned_pairs:
        t_ref.append(ref["t"] - aligned_pairs[0][0]["t"])
        errors.append(
            math.dist((ref["x"], ref["y"], ref["z"]), (est["x"], est["y"], est["z"]))
        )
        rr = quat_to_rpy(ref["qx"], ref["qy"], ref["qz"], ref["qw"])
        er = quat_to_rpy(est["qx"], est["qy"], est["qz"], est["qw"])
        for key, val in zip(("x", "y", "z"), (ref["x"], ref["y"], ref["z"])):
            glim_xyz[key].append(val)
        for key, val in zip(("x", "y", "z"), (est["x"], est["y"], est["z"])):
            lid_xyz[key].append(val)
        for key, val in zip(("roll", "pitch", "yaw"), (rr[0], rr[1], rr[2])):
            glim_xyz[key].append(math.degrees(val))
        for key, val in zip(("roll", "pitch", "yaw"), (er[0], er[1], er[2])):
            lid_xyz[key].append(math.degrees(val))

    rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
    median = sorted(errors)[len(errors) // 2]
    max_err = max(errors)

    summary = {
        "glim_run": str(glim_dir.relative_to(ROOT)),
        "lidarslam_run": str(lid_dir.relative_to(ROOT)),
        "matched_poses": len(aligned_pairs),
        "glim_path_length_m": path_length(glim_rows),
        "lidarslam_path_length_m": path_length(lid_rows),
        "lidarslam_aligned_path_length_m": path_length(lid_aligned),
        "ape_rmse_m": rmse,
        "ape_median_m": median,
        "ape_max_m": max_err,
    }
    (OUTPUT / "mid360_compare_summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )

    plot_data = {
        "t": t_ref,
        "glim": {
            "x": [r["x"] for r in glim_rows],
            "y": [r["y"] for r in glim_rows],
            "z": [r["z"] for r in glim_rows],
        },
        "lid": {
            "x": [r["x"] for r in lid_aligned],
            "y": [r["y"] for r in lid_aligned],
            "z": [r["z"] for r in lid_aligned],
        },
        "series": {
            "glim": glim_xyz,
            "lid": lid_xyz,
        },
        "err": errors,
    }

    html = f"""<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MID360 GLIM vs lidarslam</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    body {{ font-family: Arial, sans-serif; margin: 24px; color: #13202b; background: #edf3f7; }}
    section {{ background: #fff; border: 1px solid #d7e0e8; border-radius: 14px; padding: 20px; margin-bottom: 18px; }}
    h1,h2 {{ margin: 0 0 12px; }}
    .grid {{ display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 12px; }}
    .card {{ background: #f7fafc; border: 1px solid #d7e0e8; border-radius: 12px; padding: 12px; }}
    .label {{ font-size: 12px; color: #486070; }}
    .value {{ font-size: 22px; font-weight: 700; margin-top: 4px; }}
    .mono {{ font-family: ui-monospace, SFMono-Regular, Menlo, monospace; white-space: pre-wrap; font-size: 12px; }}
    .xy {{ width: 100%; height: auto; }}
    .plot {{ width: 100%; height: 520px; }}
    table {{ border-collapse: collapse; width: 100%; }}
    th, td {{ border: 1px solid #d7e0e8; padding: 8px; text-align: left; vertical-align: top; }}
    th {{ background: #f3f7fa; width: 220px; }}
    @media (max-width: 900px) {{ .grid {{ grid-template-columns: repeat(2, minmax(0, 1fr)); }} }}
  </style>
</head>
<body>
  <section>
    <h1>MID360 Sample: GLIM vs lidarslam</h1>
    <div class="mono">glim: {escape(summary['glim_run'])}
lidarslam: {escape(summary['lidarslam_run'])}</div>
  </section>

  <section>
    <h2>Summary</h2>
    <div class="grid">
      <div class="card"><div class="label">Matched poses</div><div class="value">{summary['matched_poses']}</div></div>
      <div class="card"><div class="label">APE RMSE</div><div class="value">{summary['ape_rmse_m']:.3f} m</div></div>
      <div class="card"><div class="label">APE median</div><div class="value">{summary['ape_median_m']:.3f} m</div></div>
      <div class="card"><div class="label">APE max</div><div class="value">{summary['ape_max_m']:.3f} m</div></div>
    </div>
    <div class="grid" style="margin-top:12px;">
      <div class="card"><div class="label">GLIM path length</div><div class="value">{summary['glim_path_length_m']:.1f} m</div></div>
      <div class="card"><div class="label">lidarslam path length</div><div class="value">{summary['lidarslam_path_length_m']:.1f} m</div></div>
      <div class="card"><div class="label">Path ratio</div><div class="value">{summary['lidarslam_path_length_m'] / summary['glim_path_length_m']:.3f}</div></div>
      <div class="card"><div class="label">Aligned path ratio</div><div class="value">{summary['lidarslam_aligned_path_length_m'] / summary['glim_path_length_m']:.3f}</div></div>
    </div>
  </section>

  <section>
    <h2>XY overlay</h2>
    {make_xy_svg(glim_rows, lid_aligned)}
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
    <h2>Position error norm</h2>
    <div id="errplot" class="plot" style="height: 320px;"></div>
  </section>

  <section>
    <h2>Details</h2>
    <table>
      <tr><th>GLIM trajectory</th><td class="mono">{escape(str((glim_dir / 'dump' / 'traj_lidar.txt').relative_to(ROOT)))}</td></tr>
      <tr><th>lidarslam trajectory</th><td class="mono">{escape(str((lid_dir / 'traj_lidarslam.tum').relative_to(ROOT)))}</td></tr>
      <tr><th>Summary JSON</th><td class="mono">output/mid360_compare_summary.json</td></tr>
    </table>
  </section>

  <script>
    const data = {json.dumps(plot_data)};
    Plotly.newPlot('plot3d', [
      {{type:'scatter3d', mode:'lines', x:data.glim.x, y:data.glim.y, z:data.glim.z, line:{{color:'#0b6bcb', width:4}}, name:'GLIM'}},
      {{type:'scatter3d', mode:'lines', x:data.lid.x, y:data.lid.y, z:data.lid.z, line:{{color:'#bc4b2f', width:4}}, name:'lidarslam'}}
    ], {{
      margin: {{l:0, r:0, t:0, b:0}},
      scene: {{xaxis:{{title:'X [m]'}}, yaxis:{{title:'Y [m]'}}, zaxis:{{title:'Z [m]'}}, aspectmode:'data'}}
    }}, {{responsive:true}});

    const names = ['x','y','z','roll','pitch','yaw'];
    const units = ['m','m','m','deg','deg','deg'];
    const traces = [];
    names.forEach((name, i) => {{
      const axis = i + 1;
      traces.push({{
        type:'scatter', mode:'lines', x:data.t, y:data.series.glim[name], name:`GLIM ${{name}}`,
        line:{{color:'#0b6bcb', width:1.8}}, xaxis:`x${{axis}}`, yaxis:`y${{axis}}`
      }});
      traces.push({{
        type:'scatter', mode:'lines', x:data.t, y:data.series.lid[name], name:`lidarslam ${{name}}`,
        line:{{color:'#bc4b2f', width:1.6}}, xaxis:`x${{axis}}`, yaxis:`y${{axis}}`
      }});
    }});
    Plotly.newPlot('xyzrpy', traces, {{
      grid: {{rows:3, columns:2, pattern:'independent'}},
      margin: {{l:52, r:20, t:30, b:40}},
      showlegend: false,
      annotations: names.map((name, i) => {{
        const row = Math.floor(i / 2);
        const col = i % 2;
        return {{
          text: `${{name.toUpperCase()}} [${{units[i]}}]`,
          xref:'paper', yref:'paper',
          x: col === 0 ? 0.19 : 0.81,
          y: 1.0 - row * 0.335,
          yanchor:'bottom',
          showarrow:false,
          font:{{size:13}}
        }};
      }})
    }}, {{responsive:true}});

    Plotly.newPlot('errplot', [{{
      type:'scatter', mode:'lines', x:data.t, y:data.err,
      line:{{color:'#7a3cff', width:1.8}}, name:'Position error'
    }}], {{
      margin: {{l:52, r:20, t:20, b:40}},
      xaxis: {{title:'Time [s]'}},
      yaxis: {{title:'Error [m]'}}
    }}, {{responsive:true}});
  </script>
</body>
</html>
"""
    args.report.write_text(html, encoding="utf-8")
    print(args.report)


if __name__ == "__main__":
    main()
