"""Run, plot, and experiment-section rendering for benchmark reports."""

from __future__ import annotations

import html
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.lidarslam_tools.report_charts import (
    GLIM_COLOR,
    LIDAR_COLOR,
    diff_chart_svg,
    line_chart_svg,
    plotly_3d_chart,
    xy_chart_svg,
)
from lidarslam_benchmark_tools.lidarslam_tools.report_diagnostics import render_log_alerts
from lidarslam_benchmark_tools.lidarslam_tools.report_model import (
    RunRecord,
    ape_spike_ratio,
    badge,
    fmt_float,
    fmt_ratio,
    metric_link,
    quality_markup,
    slugify,
    stability_markup,
)
from lidarslam_benchmark_tools.lidarslam_tools.report_page import render_page
from lidarslam_benchmark_tools.lidarslam_tools.trajectory_analysis import build_aligned_series


def plot_bundle(rec: RunRecord, output_root: Path, open_default: bool) -> str:
    series = build_aligned_series(rec)
    plot_id = slugify(f"{rec.group}-{rec.run}")
    if series is None:
        return (
            f"<details class='plot-detail' id='{plot_id}'{' open' if open_default else ''}>"
            f"<summary>{html.escape(rec.run)} plots</summary>"
            "<p class='muted'>trajectory artifacts are missing for this run.</p>"
            "</details>"
        )

    xy_svg = xy_chart_svg(
        series["ref_xyz"]["x"],
        series["ref_xyz"]["y"],
        series["est_xyz"]["x"],
        series["est_xyz"]["y"],
    )
    xyz_3d_plot = plotly_3d_chart(series, plot_id)
    axis_plots = []
    for axis in ("x", "y", "z"):
        axis_plots.append(
            line_chart_svg(
                series["times"],
                series["ref_xyz"][axis],
                series["est_xyz"][axis],
                f"{axis.upper()} over Time",
                "m",
            )
        )
    for axis, label in (("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")):
        axis_plots.append(
            line_chart_svg(
                series["times"],
                series["ref_rpy"][axis],
                series["est_rpy"][axis],
                f"{label} over Time",
                "deg",
            )
        )

    error_plots = [
        diff_chart_svg(
            series["times"],
            series["err_norm"],
            "Position Error Norm",
            "m",
            color=GLIM_COLOR,
        )
    ]
    for axis in ("x", "y", "z"):
        error_plots.append(
            diff_chart_svg(
                series["times"],
                series["err_xyz"][axis],
                f"{axis.upper()} Error vs GLIM",
                "m",
            )
        )
    for axis, label in (("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")):
        error_plots.append(
            diff_chart_svg(
                series["times"],
                series["err_rpy"][axis],
                f"{label} Error vs GLIM",
                "deg",
            )
        )

    window_radius = min(max(len(series["times"]) // 12, 20), 80)
    peak_start = max(0, series["peak_idx"] - window_radius)
    peak_stop = min(len(series["times"]), series["peak_idx"] + window_radius + 1)
    zoom_xy_svg = xy_chart_svg(
        series["ref_xyz"]["x"][peak_start:peak_stop],
        series["ref_xyz"]["y"][peak_start:peak_stop],
        series["est_xyz"]["x"][peak_start:peak_stop],
        series["est_xyz"]["y"][peak_start:peak_stop],
        width=520,
        height=280,
    )
    zoom_err_svg = diff_chart_svg(
        series["times"][peak_start:peak_stop],
        series["err_norm"][peak_start:peak_stop],
        f"Position Error near Peak ({series['peak_time']:.1f}s)",
        "m",
        width=520,
        height=280,
        color=GLIM_COLOR,
    )

    spike_ratio = ape_spike_ratio(rec)
    spike_text = f" · spike x{spike_ratio:.1f}" if spike_ratio is not None else ""
    detail_line = (
        f"<p class='plot-meta'>pairs {series['pairs']} · alignment {html.escape(series['alignment'])} · "
        f"APE rmse {fmt_float(rec.ape_rmse, 3, ' m')} · "
        f"median {fmt_float(rec.ape_median, 3, ' m')} · "
        f"max {fmt_float(rec.ape_max, 3, ' m')}{spike_text} · "
        f"<a href='{metric_link(rec.metrics_path, output_root)}'>metrics.json</a></p>"
    )
    return (
        f"<details class='plot-detail' id='{plot_id}'{' open' if open_default else ''}>"
        f"<summary>{html.escape(rec.run)} trajectory plots</summary>"
        "<div class='legend'>"
        f"<span><i style='background:{GLIM_COLOR}'></i>GLIM</span>"
        f"<span><i style='background:{LIDAR_COLOR}'></i>lidarslam</span>"
        "</div>"
        + detail_line
        + "<p class='plot-subhead'>Interactive 3D XYZ</p>"
        + "<div class='plot-grid'>"
        + xyz_3d_plot
        + "</div>"
        + "<p class='plot-subhead'>XY Overlay</p>"
        + f"<div class='xy-wrap'>{xy_svg}</div>"
        + "<p class='plot-subhead'>XYZRPY Time Series</p>"
        + "<div class='plot-grid'>"
        + "".join(f"<div class='plot-card'>{svg}</div>" for svg in axis_plots)
        + "</div>"
        + "<p class='plot-subhead'>Error vs GLIM</p>"
        + "<div class='plot-grid'>"
        + "".join(f"<div class='plot-card'>{svg}</div>" for svg in error_plots)
        + "</div>"
        + "<p class='plot-subhead'>Peak Window</p>"
        + f"<p class='plot-meta'>largest translation error {fmt_float(series['peak_error'], 3, ' m')} at t={series['peak_time']:.2f}s</p>"
        + "<div class='plot-grid'>"
        + f"<div class='plot-card'>{zoom_xy_svg}</div>"
        + f"<div class='plot-card'>{zoom_err_svg}</div>"
        + "</div>"
        + "<p class='plot-subhead'>Observed Warnings</p>"
        + render_log_alerts(rec)
        + "</details>"
    )


def run_row(rec: RunRecord, output_root: Path) -> str:
    ape_rmse = fmt_float(rec.ape_rmse, 3, " m")
    ape_median = fmt_float(rec.ape_median, 3, " m")
    ape_max = fmt_float(rec.ape_max, 3, " m")
    ape_width = 0.0
    if rec.ape_rmse is not None:
        ape_width = min(100.0, (rec.ape_rmse / 0.05) * 100.0)
    anchor = slugify(f"{rec.group}-{rec.run}")
    return (
        "<tr>"
        f"<td><a href='#{anchor}'>{html.escape(rec.run)}</a><div class='mini-link'><a href='{metric_link(rec.metrics_path, output_root)}'>metrics.json</a></div></td>"
        f"<td>{html.escape(rec.bag_name)}</td>"
        f"<td>{quality_markup(rec)}</td>"
        f"<td class='metric-cell'><span>{ape_rmse}</span><div class='bar'><span style='width:{ape_width:.1f}%'></span></div></td>"
        f"<td>{ape_median}</td>"
        f"<td>{ape_max}</td>"
        f"<td>{stability_markup(rec)}</td>"
        f"<td>{fmt_float(rec.lid_rtf, 3)}</td>"
        f"<td>{fmt_float(rec.glim_rtf, 3)}</td>"
        f"<td>{badge('OK' if rec.lid_ok else 'FAIL', 'good' if rec.lid_ok else 'bad')}</td>"
        f"<td>{badge('OK' if rec.glim_ok else 'FAIL', 'good' if rec.glim_ok else 'bad')}</td>"
        f"<td>{html.escape(rec.reference_kind)}</td>"
        f"<td>{html.escape(rec.reference_source)}</td>"
        f"<td>{html.escape(rec.param_name)}</td>"
        "</tr>"
    )


def section(summary: dict[str, Any], output_root: Path) -> str:
    records = summary["records"]
    links = []
    if summary["summary_md"] is not None:
        links.append(f"<a href='{metric_link(summary['summary_md'], output_root)}'>summary.md</a>")
    if summary["summary_csv"] is not None:
        links.append(f"<a href='{metric_link(summary['summary_csv'], output_root)}'>summary.csv</a>")
    links_html = " ".join(links) if links else "<span class='muted'>no aggregate files</span>"
    rows = "\n".join(run_row(rec, output_root) for rec in records)
    plots = "\n".join(plot_bundle(rec, output_root, open_default=(idx == 0)) for idx, rec in enumerate(records))
    return f"""
    <section class="panel" id="group-{slugify(summary['group'])}">
      <div class="panel-head">
        <div>
          <p class="eyebrow">Experiment Group</p>
          <h2>{html.escape(summary['group'])}</h2>
        </div>
        <div class="meta">
          <span>{links_html}</span>
          <span class="muted">updated {html.escape(summary['latest_iso'])}</span>
        </div>
      </div>
      <div class="stats">
        <div class="stat"><span class="label">Runs</span><strong>{summary['count']}</strong></div>
        <div class="stat"><span class="label">Best APE</span><strong>{fmt_float(summary['best_ape'], 3, ' m')}</strong></div>
        <div class="stat"><span class="label">Median APE</span><strong>{fmt_float(summary['median_ape'], 3, ' m')}</strong></div>
        <div class="stat"><span class="label">Good</span><strong>{summary['good_runs']}</strong></div>
        <div class="stat"><span class="label">Unstable</span><strong>{summary['unstable_runs']}</strong></div>
        <div class="stat"><span class="label">Bad</span><strong>{summary['bad_runs']}</strong></div>
        <div class="stat"><span class="label">lidarslam</span><strong>{fmt_ratio(summary['lid_success'], summary['count'])}</strong></div>
        <div class="stat"><span class="label">GLIM</span><strong>{fmt_ratio(summary['glim_success'], summary['count'])}</strong></div>
      </div>
      <div class="table-wrap">
        <table>
          <thead>
            <tr>
              <th>Run</th>
              <th>Bag</th>
              <th>Quality</th>
              <th>APE RMSE</th>
              <th>APE Med</th>
              <th>APE Max</th>
              <th>Stability</th>
              <th>Lidar RTF</th>
              <th>GLIM RTF</th>
              <th>Lidar</th>
              <th>GLIM</th>
              <th>Ref Kind</th>
              <th>Ref Src</th>
              <th>Params</th>
            </tr>
          </thead>
          <tbody>
            {rows}
          </tbody>
        </table>
      </div>
      <div class="plot-stack">
        {plots}
      </div>
    </section>
    """


def build_page(output_root: Path, groups: list[dict[str, Any]]) -> str:
    return render_page(output_root, groups, section, fmt_float, fmt_ratio)
