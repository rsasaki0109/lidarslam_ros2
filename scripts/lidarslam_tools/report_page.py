"""HTML page shell, styling, and browser interactions for benchmark reports."""

from __future__ import annotations

import html
import statistics
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

from lidarslam_benchmark_tools.lidarslam_tools.report_charts import GLIM_COLOR, LIDAR_COLOR
from lidarslam_benchmark_tools.lidarslam_tools.report_model import slugify


def render_page(
    output_root: Path,
    groups: list[dict[str, Any]],
    render_section: Callable[[dict[str, Any], Path], str],
    format_float: Callable[..., str],
    format_ratio: Callable[[int, int], str],
) -> str:
    highlights = {}
    for prefix in ("ape_cycle_", "crossbag_ape_", "tight_tune_"):
        matches = [group for group in groups if group["group"].startswith(prefix)]
        if matches:
            highlights[prefix] = max(matches, key=lambda group: group["latest_mtime"])

    all_apes = [
        rec.ape_rmse
        for group in groups
        for rec in group["records"]
        if rec.ape_rmse is not None
    ]
    fresh_glim = sum(
        1
        for group in groups
        for rec in group["records"]
        if rec.glim_ok and rec.reference_source == "fresh"
    )
    total_runs = sum(group["count"] for group in groups)
    best_run = None
    for group in groups:
        for rec in group["records"]:
            if rec.ape_rmse is None:
                continue
            if best_run is None or rec.ape_rmse < best_run.ape_rmse:
                best_run = rec

    hero_cards = [
        ("Best APE", format_float(best_run.ape_rmse if best_run else None, 3, " m"), best_run.run if best_run else "-"),
        ("Median APE", format_float(statistics.median(all_apes) if all_apes else None, 3, " m"), f"{len(all_apes)} runs"),
        ("Fresh GLIM", str(fresh_glim), f"{total_runs} total runs"),
    ]

    spotlight = []
    labels = {
        "ape_cycle_": "Latest Auto Cycle",
        "crossbag_ape_": "Cross-Bag Check",
        "tight_tune_": "Tuned Param Check",
    }
    for prefix, label in labels.items():
        summary = highlights.get(prefix)
        if summary is None:
            continue
        spotlight.append(
            f"""
            <article class="spotlight">
              <p class="eyebrow">{label}</p>
              <h3>{html.escape(summary['group'])}</h3>
              <p class="spot-value">{format_float(summary['best_ape'], 3, ' m')}</p>
              <p class="muted">median {format_float(summary['median_ape'], 3, ' m')} · GLIM {format_ratio(summary['glim_success'], summary['count'])}</p>
            </article>
            """
        )

    shown_groups = groups[:8]
    sections_html = "\n".join(
        render_section(group, output_root) for group in shown_groups)
    if len(groups) > len(shown_groups):
        sections_html += (
            "\n<p class=\"muted\">Showing the 8 most recent of "
            f"{len(groups)} experiment groups (latest first).</p>"
        )
    spotlights_html = (
        f"\n        <div class=\"spotlights\">\n"
        f"          {''.join(spotlight)}\n"
        f"        </div>"
        if spotlight else ""
    )
    hero_grid_class = "hero-grid" if spotlight else "hero-grid solo"
    toc_items = "\n".join(
        f"<li><a href=\"#group-{slugify(group['group'])}\">"
        f"{html.escape(group['group'])}</a></li>"
        for group in shown_groups
    )
    toc_html = (
        f"\n    <nav class=\"toc\" aria-label=\"Experiment groups\">"
        f"<ul>\n{toc_items}\n</ul></nav>"
        if len(shown_groups) > 1 else ""
    )
    generated_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SLAM Experiment Report</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    :root {{
      --bg: #f3ede1;
      --ink: #201815;
      --muted: #6d6058;
      --panel: rgba(255, 251, 245, 0.82);
      --line: rgba(32, 24, 21, 0.12);
      --accent: {LIDAR_COLOR};
      --good: {GLIM_COLOR};
      --good-bg: #d7efdf;
      --warn: #8b5b0c;
      --warn-bg: #f6e3bb;
      --bad: #8d2f24;
      --bad-bg: #f7d8d3;
      --shadow: 0 20px 60px rgba(62, 42, 24, 0.12);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(188, 75, 47, 0.18), transparent 30%),
        radial-gradient(circle at top right, rgba(36, 87, 61, 0.12), transparent 25%),
        linear-gradient(180deg, #f7f2e9 0%, var(--bg) 100%);
      font-family: "Iowan Old Style", "Palatino Linotype", "Book Antiqua", Palatino, "Noto Serif JP", serif;
      line-height: 1.45;
    }}
    a {{ color: var(--accent); text-decoration: none; }}
    a:hover {{ text-decoration: underline; }}
    code {{
      padding: 0.1rem 0.35rem;
      border-radius: 999px;
      background: rgba(255, 255, 255, 0.66);
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
      font-size: 0.9em;
    }}
    .page {{
      width: min(1240px, calc(100% - 32px));
      margin: 0 auto;
      padding: 32px 0 56px;
    }}
    .hero {{
      padding: 28px;
      border: 1px solid var(--line);
      border-radius: 28px;
      background: linear-gradient(145deg, rgba(255,255,255,0.72), rgba(255,248,240,0.86));
      box-shadow: var(--shadow);
    }}
    .eyebrow {{
      margin: 0 0 10px;
      color: var(--muted);
      text-transform: uppercase;
      letter-spacing: 0.14em;
      font-size: 0.72rem;
    }}
    h1, h2, h3 {{
      margin: 0;
      font-weight: 700;
    }}
    h1 {{
      font-size: clamp(2rem, 4vw, 3.6rem);
      line-height: 0.98;
      max-width: 10ch;
    }}
    .sub {{
      margin: 14px 0 0;
      color: var(--muted);
      max-width: 72ch;
    }}
    .hero-grid {{
      display: grid;
      grid-template-columns: 1.2fr 1fr;
      gap: 20px;
      margin-top: 26px;
    }}
    .hero-grid.solo {{
      grid-template-columns: 1fr;
    }}
    .toc {{
      margin-top: 18px;
      padding: 12px 18px;
      border: 1px solid var(--line);
      border-radius: 18px;
      background: rgba(255, 252, 247, 0.84);
    }}
    .toc ul {{
      display: flex;
      flex-wrap: wrap;
      gap: 8px 18px;
      margin: 0;
      padding: 0;
      list-style: none;
    }}
    .toc a {{
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
      font-size: 0.85rem;
    }}
    .hero-cards, .spotlights {{
      display: grid;
      gap: 16px;
    }}
    .hero-cards {{
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }}
    .card, .spotlight, .panel {{
      border: 1px solid var(--line);
      border-radius: 24px;
      background: rgba(255, 252, 247, 0.84);
      backdrop-filter: blur(14px);
    }}
    .card {{
      padding: 18px;
    }}
    .card strong {{
      display: block;
      font-size: 1.5rem;
      margin-bottom: 4px;
    }}
    .card span, .muted {{
      color: var(--muted);
    }}
    .spotlights {{
      grid-template-columns: repeat(3, minmax(0, 1fr));
    }}
    .spotlight {{
      padding: 18px;
    }}
    .spot-value {{
      font-size: 1.9rem;
      margin: 14px 0 6px;
      color: var(--accent);
    }}
    .panel {{
      margin-top: 24px;
      padding: 22px;
      box-shadow: var(--shadow);
    }}
    .panel-head {{
      display: flex;
      gap: 16px;
      justify-content: space-between;
      align-items: start;
      margin-bottom: 18px;
    }}
    .panel-head h2 {{
      font-size: 1.5rem;
    }}
    .meta {{
      display: flex;
      flex-wrap: wrap;
      justify-content: end;
      gap: 12px;
      font-size: 0.92rem;
    }}
    .stats {{
      display: grid;
      grid-template-columns: repeat(7, minmax(0, 1fr));
      gap: 12px;
      margin-bottom: 18px;
    }}
    .stat {{
      padding: 14px 16px;
      border: 1px solid var(--line);
      border-radius: 18px;
      background: rgba(255,255,255,0.45);
    }}
    .label {{
      display: block;
      color: var(--muted);
      font-size: 0.82rem;
      margin-bottom: 6px;
    }}
    .stat strong {{
      font-size: 1.05rem;
    }}
    .table-wrap {{
      overflow-x: auto;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      min-width: 1180px;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
      font-size: 0.86rem;
    }}
    th, td {{
      text-align: left;
      padding: 12px 10px;
      border-bottom: 1px solid var(--line);
      vertical-align: middle;
    }}
    th {{
      color: var(--muted);
      font-weight: 600;
      font-size: 0.78rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }}
    .badge {{
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 4px 9px;
      font-size: 0.74rem;
      font-weight: 700;
      letter-spacing: 0.06em;
      text-transform: uppercase;
    }}
    .badge-good {{
      color: var(--good);
      background: var(--good-bg);
    }}
    .badge-bad {{
      color: var(--bad);
      background: var(--bad-bg);
    }}
    .badge-warn {{
      color: var(--warn);
      background: var(--warn-bg);
    }}
    .metric-cell {{
      min-width: 170px;
    }}
    .bar {{
      height: 6px;
      margin-top: 6px;
      border-radius: 999px;
      background: rgba(188, 75, 47, 0.1);
      overflow: hidden;
    }}
    .bar span {{
      display: block;
      height: 100%;
      border-radius: 999px;
      background: linear-gradient(90deg, #d17d64, var(--accent));
    }}
    .mini-link {{
      margin-top: 4px;
      font-size: 0.75rem;
    }}
    .plot-stack {{
      display: grid;
      gap: 14px;
      margin-top: 18px;
    }}
    .plot-detail {{
      border: 1px solid var(--line);
      border-radius: 20px;
      background: rgba(255,255,255,0.5);
      overflow: hidden;
    }}
    .plot-detail summary {{
      cursor: pointer;
      list-style: none;
      padding: 16px 18px;
      font-weight: 700;
    }}
    .plot-detail summary::-webkit-details-marker {{
      display: none;
    }}
    .plot-detail[open] summary {{
      border-bottom: 1px solid var(--line);
      background: rgba(255,255,255,0.35);
    }}
    .legend {{
      display: flex;
      gap: 14px;
      padding: 14px 18px 0;
      color: var(--muted);
      font-size: 0.88rem;
    }}
    .legend span {{
      display: inline-flex;
      align-items: center;
      gap: 8px;
    }}
    .legend i {{
      width: 12px;
      height: 12px;
      border-radius: 999px;
      display: inline-block;
    }}
    .plot-meta {{
      margin: 8px 18px 0;
      color: var(--muted);
      font-size: 0.9rem;
    }}
    .plot-subhead {{
      margin: 14px 18px 0;
      color: var(--muted);
      font-size: 0.78rem;
      font-weight: 700;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    .alert-grid {{
      display: grid;
      gap: 10px;
      padding: 12px 18px 18px;
    }}
    .alert-card {{
      border: 1px solid var(--line);
      border-radius: 14px;
      background: rgba(255,255,255,0.72);
      padding: 12px 14px;
    }}
    .alert-src {{
      margin-left: 8px;
      color: var(--muted);
      font-size: 0.82rem;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    .alert-card p {{
      margin: 8px 0 0;
      color: var(--ink);
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
      font-size: 0.82rem;
      line-height: 1.45;
    }}
    .xy-wrap {{
      padding: 10px 18px 0;
    }}
    .plot-grid {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      padding: 12px 18px 18px;
    }}
    .plot-card {{
      border: 1px solid var(--line);
      border-radius: 16px;
      background: rgba(255,255,255,0.78);
      overflow: hidden;
      min-height: 196px;
    }}
    .plotly-card {{
      min-height: 420px;
    }}
    .plotly-3d {{
      width: 100%;
      height: 420px;
    }}
    .plot-card svg, .xy-wrap svg {{
      display: block;
    }}
    .plot-title {{
      fill: var(--ink);
      font-size: 12px;
      font-weight: 700;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    .axis {{
      fill: var(--muted);
      font-size: 10px;
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    .plot-empty {{
      padding: 18px;
      color: var(--muted);
      font-family: "IBM Plex Mono", "SFMono-Regular", Consolas, monospace;
    }}
    footer {{
      margin-top: 18px;
      color: var(--muted);
      font-size: 0.9rem;
    }}
    @media (max-width: 920px) {{
      .hero-grid, .hero-cards, .spotlights, .stats, .plot-grid {{
        grid-template-columns: 1fr;
      }}
      .panel-head {{
        flex-direction: column;
      }}
      .meta {{
        justify-content: start;
      }}
    }}
  </style>
</head>
<body>
  <div class="page">
    <section class="hero">
      <p class="eyebrow">SLAM Experiment Report</p>
      <h1>Trajectory overlays and axis-by-axis drift, side by side.</h1>
      <p class="sub">Generated from <code>output/**/metrics.json</code> and the paired trajectory files. Each run includes an interactive <code>3D XYZ</code> trajectory view, an <code>XY</code> overlay, <code>X/Y/Z/Roll/Pitch/Yaw</code> time-series plots, signed error traces against GLIM, an auto-extracted peak window, and local log warnings so bad runs are easier to classify and debug from one page.</p>
      <div class="{hero_grid_class}">
        <div class="hero-cards">
          {''.join(f"<article class='card'><span>{html.escape(title)}</span><strong>{html.escape(value)}</strong><span>{html.escape(note)}</span></article>" for title, value, note in hero_cards)}
        </div>{spotlights_html}
      </div>
    </section>{toc_html}
    {sections_html}
    <footer>Generated at {html.escape(generated_at)} from {len(groups)} experiment groups.</footer>
  </div>
</body>
</html>
"""
