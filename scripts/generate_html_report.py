#!/usr/bin/env python3

from __future__ import annotations

import argparse
import html
from collections import defaultdict
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
from lidarslam_benchmark_tools.lidarslam_tools.report_diagnostics import collect_log_alerts, render_log_alerts
from lidarslam_benchmark_tools.lidarslam_tools.report_model import (
    RunRecord,
    ape_spike_ratio,
    as_bool,
    as_float,
    badge,
    fmt_float,
    fmt_ratio,
    infer_reference_kind,
    is_spiky_run,
    load_record,
    median,
    metric_link,
    quality_markup,
    resolve_artifact_path,
    run_quality,
    slugify,
    stability_markup,
    summarize_group,
)
from lidarslam_benchmark_tools.lidarslam_tools.report_page import render_page
from lidarslam_benchmark_tools.lidarslam_tools.report_sections import build_page, plot_bundle, run_row, section
from lidarslam_benchmark_tools.lidarslam_tools.trajectory_analysis import Pose, build_aligned_series


__all__ = [
    'GLIM_COLOR',
    'LIDAR_COLOR',
    'Pose',
    'RunRecord',
    'ape_spike_ratio',
    'as_bool',
    'as_float',
    'badge',
    'build_aligned_series',
    'build_page',
    'collect_log_alerts',
    'diff_chart_svg',
    'fmt_float',
    'fmt_ratio',
    'infer_reference_kind',
    'is_spiky_run',
    'line_chart_svg',
    'load_record',
    'main',
    'median',
    'metric_link',
    'plot_bundle',
    'plotly_3d_chart',
    'quality_markup',
    'render_log_alerts',
    'render_page',
    'resolve_artifact_path',
    'run_quality',
    'run_row',
    'section',
    'slugify',
    'stability_markup',
    'summarize_group',
    'xy_chart_svg',
]


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate a static HTML report from benchmark metrics.")
    ap.add_argument("--root", default="output", help="Output root containing metrics.json files")
    ap.add_argument("--out", default="output/latest_report.html", help="HTML output path")
    args = ap.parse_args()

    output_root = Path(args.root).expanduser().resolve()
    repo_root = output_root.parent
    out_path = Path(args.out).expanduser().resolve()
    metrics_paths = sorted(output_root.rglob("metrics.json"))
    records = []
    for metrics_path in metrics_paths:
        record = load_record(metrics_path, output_root, repo_root)
        if record is not None:
            records.append(record)
    if not records:
        raise SystemExit("no metrics.json found")

    grouped: dict[str, list[RunRecord]] = defaultdict(list)
    for record in records:
        grouped[record.group].append(record)

    groups = [summarize_group(group, recs, output_root) for group, recs in grouped.items()]
    groups.sort(key=lambda item: item["latest_mtime"], reverse=True)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(build_page(output_root, groups), encoding="utf-8")
    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
