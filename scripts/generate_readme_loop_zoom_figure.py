#!/usr/bin/env python3
"""Generate a README image that zooms into the MID360 loop-closure area."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

import lidarslam_benchmark_tools.generate_readme_mid360_figures as mid360_fig
from lidarslam_benchmark_tools.generate_readme_large_loop_map_figure import DEFAULT_MID360_METRICS
from lidarslam_benchmark_tools.generate_readme_large_loop_map_figure import DEFAULT_MID360_TRAJ


ROOT = Path(__file__).resolve().parents[1]
IMAGE_DIR = ROOT / "lidarslam" / "images"
DEFAULT_OUT = IMAGE_DIR / "mid360_loop_closure_zoom.png"


def _load_metrics(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _pick_zoom_bounds(path_xy: np.ndarray, from_index: int, to_index: int, padding: float) -> tuple[float, float, float, float]:
    loop_points = path_xy[[from_index, to_index], :]
    min_xy = loop_points.min(axis=0)
    max_xy = loop_points.max(axis=0)
    center = (min_xy + max_xy) * 0.5
    span = np.maximum(max_xy - min_xy, 1.0)
    half = max(float(np.max(span)) * 0.5 + padding, 12.0)
    return (
        float(center[0] - half),
        float(center[0] + half),
        float(center[1] - half),
        float(center[1] + half),
    )


def build_loop_zoom_png(
    bag_path: Path,
    traj_path: Path,
    metrics_path: Path,
    output_path: Path,
    scan_stride: int,
    point_stride: int,
    padding: float,
) -> None:
    metrics = _load_metrics(metrics_path)
    loop_edge = (metrics.get("graph_based_slam") or {}).get("last_loop_edge") or {}
    from_index = int(loop_edge.get("from_index", 0))
    to_index = int(loop_edge.get("to_index", 0))

    traj_rows = mid360_fig.load_tum(traj_path)
    if not traj_rows:
        raise SystemExit(f"trajectory file missing or empty: {traj_path}")

    cloud_world, _ = mid360_fig.sample_map_points(
        bag_path,
        traj_rows,
        scan_stride=scan_stride,
        point_stride=point_stride,
    )
    full_path_xy = np.array([[row["x"], row["y"]] for row in traj_rows], dtype=float)
    if cloud_world.size == 0 or full_path_xy.size == 0:
        raise SystemExit("failed to build sampled point-cloud map")

    if from_index >= len(full_path_xy) or to_index >= len(full_path_xy):
        raise SystemExit(
            f"loop indices out of range for trajectory: {from_index} -> {to_index}, path size={len(full_path_xy)}"
        )

    x0, x1, y0, y1 = _pick_zoom_bounds(full_path_xy, from_index, to_index, padding)
    in_zoom = (
        (cloud_world[:, 0] >= x0)
        & (cloud_world[:, 0] <= x1)
        & (cloud_world[:, 1] >= y0)
        & (cloud_world[:, 1] <= y1)
    )
    zoom_cloud = cloud_world[in_zoom]
    if zoom_cloud.size == 0:
        raise SystemExit("zoom window captured no point-cloud samples")

    combined_z = np.clip(cloud_world[:, 2], np.percentile(cloud_world[:, 2], 2), np.percentile(cloud_world[:, 2], 98))
    z_lo = float(np.min(combined_z))
    z_hi = float(np.max(combined_z))

    fig = plt.figure(figsize=(13.8, 6.8), dpi=180, facecolor="#f6f8fb")
    gs = fig.add_gridspec(
        2,
        3,
        height_ratios=[0.18, 1.0],
        width_ratios=[1.0, 1.0, 0.05],
        left=0.05,
        right=0.96,
        top=0.94,
        bottom=0.09,
        wspace=0.14,
        hspace=0.14,
    )
    ax_header = fig.add_subplot(gs[0, :2])
    ax_header.axis("off")
    ax_full = fig.add_subplot(gs[1, 0])
    ax_zoom = fig.add_subplot(gs[1, 1])
    cax = fig.add_subplot(gs[1, 2])

    ax_header.text(
        0.0,
        0.80,
        "MID360 current default: loop-closure zoom",
        fontsize=18,
        fontweight="bold",
        ha="left",
        va="center",
        color="#13202b",
    )
    ax_header.text(
        0.0,
        0.40,
        (
            f"Same current default run as the overview above, accepted loop {from_index} -> {to_index}. "
            "The right panel zooms into the closing segment so duplicated structure is easier to spot."
        ),
        fontsize=10.5,
        ha="left",
        va="center",
        color="#516679",
    )
    ax_header.text(
        1.0,
        0.80,
        (
            f"APE RMSE {((metrics.get('evo') or {}).get('ape') or {}).get('rmse', float('nan')):.3f} m\n"
            f"loop count {((metrics.get('graph_based_slam') or {}).get('loop_count', 0))}\n"
            f"search distance {((metrics.get('graph_based_slam') or {}).get('max_loop_search_distance_m', float('nan'))):.1f} m"
        ),
        fontsize=10.5,
        ha="right",
        va="top",
        color="#13202b",
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "white", "edgecolor": "#d8e3ef"},
    )

    def style_axes(ax: plt.Axes, title: str) -> None:
        ax.set_facecolor("white")
        ax.set_title(title, fontsize=13, fontweight="bold", pad=10)
        ax.grid(True, color="#e9eef4", linewidth=0.8)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_aspect("equal", adjustable="box")
        for spine in ax.spines.values():
            spine.set_edgecolor("#d8e3ef")

    full_sc = ax_full.scatter(
        cloud_world[:, 0],
        cloud_world[:, 1],
        c=np.clip(cloud_world[:, 2], z_lo, z_hi),
        s=0.18,
        cmap="viridis",
        linewidths=0.0,
        alpha=0.70,
    )
    ax_full.plot(full_path_xy[:, 0], full_path_xy[:, 1], color="white", linewidth=1.0, alpha=0.9)
    ax_full.plot(full_path_xy[:, 0], full_path_xy[:, 1], color="#13202b", linewidth=0.45, alpha=0.90)
    full_bounds = np.vstack([cloud_world[:, :2], full_path_xy[:, :2]])
    min_xy = full_bounds.min(axis=0)
    max_xy = full_bounds.max(axis=0)
    span = np.maximum(max_xy - min_xy, 1.0)
    pad = span * 0.04
    ax_full.set_xlim(float(min_xy[0] - pad[0]), float(max_xy[0] + pad[0]))
    ax_full.set_ylim(float(min_xy[1] - pad[1]), float(max_xy[1] + pad[1]))
    style_axes(ax_full, "Full map with zoom window")
    ax_full.add_patch(
        patches.Rectangle(
            (x0, y0),
            x1 - x0,
            y1 - y0,
            linewidth=1.8,
            edgecolor="#bc4b2f",
            facecolor="none",
            linestyle="--",
        )
    )

    ax_zoom.scatter(
        zoom_cloud[:, 0],
        zoom_cloud[:, 1],
        c=np.clip(zoom_cloud[:, 2], z_lo, z_hi),
        s=0.55,
        cmap="viridis",
        linewidths=0.0,
        alpha=0.85,
    )
    ax_zoom.plot(full_path_xy[:, 0], full_path_xy[:, 1], color="white", linewidth=1.8, alpha=0.95)
    ax_zoom.plot(full_path_xy[:, 0], full_path_xy[:, 1], color="#13202b", linewidth=1.0, alpha=0.95)
    loop_points = full_path_xy[[from_index, to_index], :]
    loop_colors = ["#0b6bcb", "#bc4b2f"]
    loop_labels = [f"loop start #{from_index}", f"loop close #{to_index}"]
    for point, color, label in zip(loop_points, loop_colors, loop_labels):
        ax_zoom.scatter(
            point[0],
            point[1],
            s=95,
            facecolor=color,
            edgecolor="white",
            linewidth=1.6,
            zorder=5,
        )
        ax_zoom.text(
            point[0] + 0.8,
            point[1] + 0.8,
            label,
            fontsize=9.2,
            color=color,
            bbox={"boxstyle": "round,pad=0.22", "facecolor": "white", "edgecolor": "#d8e3ef"},
        )
    ax_zoom.set_xlim(x0, x1)
    ax_zoom.set_ylim(y0, y1)
    style_axes(ax_zoom, "Loop area zoom")

    cbar = fig.colorbar(full_sc, cax=cax)
    cbar.set_label("height [m]")
    cbar.outline.set_edgecolor("#d8e3ef")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, facecolor=fig.get_facecolor())
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the README loop-closure zoom image for the MID360 default run.",
    )
    parser.add_argument("--bag", default=str(mid360_fig.BAG_PATH), help="ROS 2 bag used to reconstruct sampled map points")
    parser.add_argument("--traj", default=str(DEFAULT_MID360_TRAJ), help="Corrected TUM trajectory for the published run")
    parser.add_argument("--metrics", default=str(DEFAULT_MID360_METRICS), help="metrics.json containing the accepted loop edge")
    parser.add_argument("--out", default=str(DEFAULT_OUT), help="Output PNG path")
    parser.add_argument("--scan-stride", type=int, default=10, help="Use every Nth point-cloud message")
    parser.add_argument("--point-stride", type=int, default=24, help="Use every Nth point in each sampled cloud")
    parser.add_argument("--padding", type=float, default=18.0, help="Extra XY padding around the accepted loop edge [m]")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    build_loop_zoom_png(
        bag_path=Path(args.bag).expanduser().resolve(),
        traj_path=Path(args.traj).expanduser().resolve(),
        metrics_path=Path(args.metrics).expanduser().resolve(),
        output_path=Path(args.out).expanduser().resolve(),
        scan_stride=args.scan_stride,
        point_stride=args.point_stride,
        padding=args.padding,
    )
    print(Path(args.out).expanduser().resolve())


if __name__ == "__main__":
    main()
