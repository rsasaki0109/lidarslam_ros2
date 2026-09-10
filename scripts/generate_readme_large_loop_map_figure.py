#!/usr/bin/env python3
"""Generate the README overview image for a large-loop MID360 run.

The historical output path is kept as `mid360_glim_map_compare.png` so the
release bundle and docs links stay stable, but the image now focuses on the
current large-loop `graph_based_slam` map rather than a GLIM comparison.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import lidarslam_benchmark_tools.generate_readme_mid360_figures as mid360_fig


ROOT = Path(__file__).resolve().parents[1]
IMAGE_DIR = ROOT / "lidarslam" / "images"
DEFAULT_OUT = IMAGE_DIR / "mid360_glim_map_compare.png"
DEFAULT_MID360_METRICS = (
    ROOT / "output" / "bench_rko_lio_mid360_current_default_20260325" / "metrics.json"
)
DEFAULT_MID360_TRAJ = (
    ROOT / "output" / "bench_rko_lio_mid360_current_default_20260325" / "traj_corrected.tum"
)


def _load_metrics(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_large_loop_png(
    bag_path: Path,
    traj_path: Path,
    metrics_path: Path,
    output_path: Path,
    scan_stride: int,
    point_stride: int,
) -> None:
    metrics = _load_metrics(metrics_path)
    traj_rows = mid360_fig.load_tum(traj_path)
    if not traj_rows:
        raise SystemExit(f"trajectory file missing or empty: {traj_path}")

    cloud_world, path_xy = mid360_fig.sample_map_points(
        bag_path,
        traj_rows,
        scan_stride=scan_stride,
        point_stride=point_stride,
    )
    if cloud_world.size == 0 or path_xy.size == 0:
        raise SystemExit("failed to build sampled point-cloud map")

    full_path_xy = np.array([[row["x"], row["y"]] for row in traj_rows], dtype=float)
    loop_edge = (metrics.get("graph_based_slam") or {}).get("last_loop_edge") or {}
    from_index = int(loop_edge.get("from_index", 0))
    to_index = int(loop_edge.get("to_index", 0))
    if from_index >= len(full_path_xy) or to_index >= len(full_path_xy):
        raise SystemExit(
            f"loop indices out of range for trajectory: {from_index} -> {to_index}, "
            f"path size={len(full_path_xy)}"
        )

    loop_points = full_path_xy[[from_index, to_index], :]
    combined_z = np.clip(
        cloud_world[:, 2],
        np.percentile(cloud_world[:, 2], 2),
        np.percentile(cloud_world[:, 2], 98),
    )
    z_lo = float(np.min(combined_z))
    z_hi = float(np.max(combined_z))

    bounds = np.vstack([cloud_world[:, :2], full_path_xy])
    min_xy = bounds.min(axis=0)
    max_xy = bounds.max(axis=0)
    span = np.maximum(max_xy - min_xy, 1.0)
    pad = span * 0.05

    fig = plt.figure(figsize=(13.8, 7.6), dpi=180, facecolor="#f6f8fb")
    gs = fig.add_gridspec(
        2,
        2,
        height_ratios=[0.20, 1.0],
        width_ratios=[1.0, 0.05],
        left=0.05,
        right=0.96,
        top=0.94,
        bottom=0.08,
        wspace=0.10,
        hspace=0.14,
    )
    ax_header = fig.add_subplot(gs[0, 0])
    ax_header.axis("off")
    ax = fig.add_subplot(gs[1, 0])
    cax = fig.add_subplot(gs[1, 1])

    ax_header.text(
        0.0,
        0.82,
        "MID360 current default: large-loop point-cloud map",
        fontsize=18,
        fontweight="bold",
        ha="left",
        va="center",
        color="#13202b",
    )
    ax_header.text(
        0.0,
        0.42,
        (
            "Representative large-loop map from the current documented MID360 "
            "default run. Colors encode height. The black trace is the corrected "
            "trajectory, with the accepted loop edge highlighted."
        ),
        fontsize=10.5,
        ha="left",
        va="center",
        color="#516679",
    )
    ax_header.text(
        1.0,
        0.82,
        (
            f"APE RMSE {((metrics.get('evo') or {}).get('ape') or {}).get('rmse', float('nan')):.3f} m\n"
            f"accepted loop {from_index} -> {to_index}\n"
            f"search distance {((metrics.get('graph_based_slam') or {}).get('max_loop_search_distance_m', float('nan'))):.1f} m"
        ),
        fontsize=10.5,
        ha="right",
        va="top",
        color="#13202b",
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "white", "edgecolor": "#d8e3ef"},
    )

    sc = ax.scatter(
        cloud_world[:, 0],
        cloud_world[:, 1],
        c=np.clip(cloud_world[:, 2], z_lo, z_hi),
        s=0.20,
        cmap="viridis",
        linewidths=0.0,
        alpha=0.82,
    )
    ax.plot(full_path_xy[:, 0], full_path_xy[:, 1], color="white", linewidth=1.1, alpha=0.95)
    ax.plot(full_path_xy[:, 0], full_path_xy[:, 1], color="#13202b", linewidth=0.55, alpha=0.95)
    ax.plot(
        loop_points[:, 0],
        loop_points[:, 1],
        color="#bc4b2f",
        linewidth=2.0,
        linestyle="--",
        alpha=0.95,
    )
    loop_colors = ["#0b6bcb", "#bc4b2f"]
    loop_labels = [f"loop start #{from_index}", f"loop close #{to_index}"]
    for point, color, label in zip(loop_points, loop_colors, loop_labels):
        ax.scatter(
            point[0],
            point[1],
            s=110,
            facecolor=color,
            edgecolor="white",
            linewidth=1.8,
            zorder=5,
        )
        ax.text(
            point[0] + 3.0,
            point[1] + 3.0,
            label,
            fontsize=9.5,
            color=color,
            bbox={"boxstyle": "round,pad=0.24", "facecolor": "white", "edgecolor": "#d8e3ef"},
        )

    ax.set_facecolor("white")
    ax.set_title("Full loop overview", fontsize=13, fontweight="bold", pad=10)
    ax.set_xlim(float(min_xy[0] - pad[0]), float(max_xy[0] + pad[0]))
    ax.set_ylim(float(min_xy[1] - pad[1]), float(max_xy[1] + pad[1]))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#e9eef4", linewidth=0.8)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    for spine in ax.spines.values():
        spine.set_edgecolor("#d8e3ef")

    cbar = fig.colorbar(sc, cax=cax)
    cbar.set_label("height [m]")
    cbar.outline.set_edgecolor("#d8e3ef")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, facecolor=fig.get_facecolor())
    plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the README large-loop overview image for the MID360 default run.",
    )
    parser.add_argument("--bag", default=str(mid360_fig.BAG_PATH))
    parser.add_argument("--traj", default=str(DEFAULT_MID360_TRAJ))
    parser.add_argument("--metrics", default=str(DEFAULT_MID360_METRICS))
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--scan-stride", type=int, default=10)
    parser.add_argument("--point-stride", type=int, default=24)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    build_large_loop_png(
        bag_path=Path(args.bag).expanduser().resolve(),
        traj_path=Path(args.traj).expanduser().resolve(),
        metrics_path=Path(args.metrics).expanduser().resolve(),
        output_path=Path(args.out).expanduser().resolve(),
        scan_stride=args.scan_stride,
        point_stride=args.point_stride,
    )
    print(Path(args.out).expanduser().resolve())


if __name__ == "__main__":
    main()
