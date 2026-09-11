#!/usr/bin/env python3
"""Generate outreach figures for the LiDAR degeneracy-resilience article.

Reads TUM trajectories produced by the fog/tunnel A/B runs recorded in
docs/research/lidar-degeneracy-radar-intensity-ab-2026-07.md and renders
three PNGs into this directory:

  fog_trajectory_xy.png       fog baseline vs radar-disagreement-gate (w=1.0)
  tunnel_trajectory_xy.png    tunnel baseline vs radar(scale1.05) vs intensity-only
  gate_coverage_concept.png   Hessian eigenvalue gate vs sensor-disagreement gate,
                               scan-coverage timeline (conceptual, real counts)

Usage:
    python3 make_figures.py

All numbers plotted/annotated are taken verbatim from the research note
table or recomputed directly from the TUM files below (verified to match
the note to the cm). No numbers are invented.
"""
from __future__ import annotations

import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent

# ---------------------------------------------------------------------------
# Data sources (absolute paths, per task brief)
# ---------------------------------------------------------------------------
BENCH = pathlib.Path(
    "/media/sasaki/aiueo/benchmarks/lidar_degeneracy_datasets_v1/runs"
)

FOG_BASELINE = BENCH / "fog_rko_lio_baseline_v1/traj_raw.tum"
FOG_RADAR = BENCH / "fog_rko_lio_radar_disagreement_v1/fog_radar_v6_0/fog_radar_v6_tum_0.txt"

TUNNEL_BASELINE = BENCH / "tunnel_rko_lio_baseline_v1/tunnel_baseline_0/tunnel_baseline_tum_0.txt"
TUNNEL_RADAR_SCALE105 = (
    BENCH / "tunnel_rko_lio_radar_scale105_v1/tunnel_scale_105_0/tunnel_scale_105_tum_0.txt"
)
TUNNEL_INTENSITY = (
    BENCH
    / "tunnel_rko_lio_intensity_disagreement_v1"
    / "tunnel_intensity_disagreement_v1_0"
    / "tunnel_intensity_disagreement_v1_tum_0.txt"
)

# ---------------------------------------------------------------------------
# Palette (validated categorical set, light-surface slots 1/2/6; see the
# dataviz skill's references/palette.md for the CVD validation)
# ---------------------------------------------------------------------------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_SECONDARY = "#52514e"
INK_MUTED = "#898781"
GRID = "#e1e0d9"
BASELINE_AXIS = "#c3c2b7"

BLUE = "#2a78d6"    # slot 1 -- baseline / uncorrected
GREEN = "#008300"   # slot 2 -- best / adopted correction
ORANGE = "#eb6834"  # slot 6 -- partial / intermediate correction
RED = "#e34948"     # slot 8 -- reserved for negative/failure emphasis

plt.rcParams.update(
    {
        "font.family": "sans-serif",
        "font.sans-serif": ["DejaVu Sans", "Arial", "Helvetica"],
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "axes.edgecolor": BASELINE_AXIS,
        "axes.labelcolor": INK_SECONDARY,
        "text.color": INK,
        "xtick.color": INK_MUTED,
        "ytick.color": INK_MUTED,
        "axes.grid": True,
        "grid.color": GRID,
        "grid.linewidth": 0.8,
        "font.size": 11,
    }
)


def load_tum(path: pathlib.Path) -> np.ndarray:
    """Load a TUM file into an (N, 8) array: t x y z qx qy qz qw."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            rows.append([float(v) for v in line.split()])
    return np.asarray(rows)


def clean_axes(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(BASELINE_AXIS)
    ax.set_axisbelow(True)


def endpoint_dist(arr: np.ndarray) -> float:
    x, y, z = arr[-1, 1], arr[-1, 2], arr[-1, 3]
    return float(np.sqrt(x * x + y * y + z * z))


# ---------------------------------------------------------------------------
# Figure 1: fog trajectory, baseline vs radar-disagreement-gate
# ---------------------------------------------------------------------------
def make_fog_trajectory():
    base = load_tum(FOG_BASELINE)
    radar = load_tum(FOG_RADAR)

    d_base = endpoint_dist(base)
    d_radar = endpoint_dist(radar)

    fig, ax = plt.subplots(figsize=(7.2, 6.4), dpi=175)

    ax.plot(base[:, 1], base[:, 2], color=BLUE, lw=1.8, label="baseline (no correction)", zorder=3)
    ax.plot(
        radar[:, 1],
        radar[:, 2],
        color=GREEN,
        lw=1.8,
        label="radar disagreement gate (w=1.0, adopted)",
        zorder=4,
    )

    # start marker (shared, both start at origin)
    ax.scatter([0], [0], s=70, marker="o", color=INK, zorder=6, label="start (t=0)")
    ax.annotate("start", (0, 0), textcoords="offset points", xytext=(8, 8), fontsize=9, color=INK_SECONDARY)

    # end markers
    ax.scatter([base[-1, 1]], [base[-1, 2]], s=70, marker="X", color=BLUE, zorder=6, edgecolor=SURFACE, linewidth=0.8)
    ax.scatter([radar[-1, 1]], [radar[-1, 2]], s=70, marker="X", color=GREEN, zorder=6, edgecolor=SURFACE, linewidth=0.8)

    # endpoint-drift annotation lines back to origin
    for arr, color, dist, label_off in (
        (base, BLUE, d_base, (-40, -6)),
        (radar, GREEN, d_radar, (10, -18)),
    ):
        ax.plot([0, arr[-1, 1]], [0, arr[-1, 2]], color=color, lw=1.0, ls=(0, (3, 3)), alpha=0.6, zorder=2)

    ax.annotate(
        f"baseline drift: {d_base:.2f} m",
        xy=(base[-1, 1], base[-1, 2]),
        xytext=(-95, 18),
        textcoords="offset points",
        fontsize=10,
        color=BLUE,
        fontweight="bold",
        path_effects=[pe.withStroke(linewidth=3, foreground=SURFACE)],
    )
    ax.annotate(
        f"radar-gate drift: {d_radar:.2f} m (−31%)",
        xy=(radar[-1, 1], radar[-1, 2]),
        xytext=(-30, -34),
        textcoords="offset points",
        fontsize=10,
        color=GREEN,
        fontweight="bold",
        path_effects=[pe.withStroke(linewidth=3, foreground=SURFACE)],
    )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        "Fog corridor: start–end drift, baseline vs radar disagreement gate",
        fontsize=12.5,
        color=INK,
        pad=12,
    )
    ax.set_aspect("equal", adjustable="datalim")
    clean_axes(ax)
    ax.legend(loc="upper left", frameon=False, fontsize=9.5)
    fig.text(
        0.01,
        0.01,
        "NTNU LiDAR Degeneracy Datasets – fog sequence (loop, true endpoint offset = 0 m). "
        "Both trajectories start at the origin.",
        fontsize=7.5,
        color=INK_MUTED,
    )
    fig.tight_layout(rect=(0, 0.02, 1, 1))
    out = HERE / "fog_trajectory_xy.png"
    fig.savefig(out, dpi=175)
    plt.close(fig)
    print(f"wrote {out}  (baseline drift={d_base:.3f} m, radar-gate drift={d_radar:.3f} m)")


# ---------------------------------------------------------------------------
# Figure 2: tunnel trajectory, baseline vs radar(scale1.05) vs intensity-only
# ---------------------------------------------------------------------------
def make_tunnel_trajectory():
    base = load_tum(TUNNEL_BASELINE)
    radar = load_tum(TUNNEL_RADAR_SCALE105)
    inten = load_tum(TUNNEL_INTENSITY)

    d_base = endpoint_dist(base)
    d_radar = endpoint_dist(radar)
    d_inten = endpoint_dist(inten)
    expected = 500.0

    fig, ax = plt.subplots(figsize=(9.2, 4.6), dpi=175)

    # expected reach reference line (along principal x-ish axis)
    ax.axvline(expected, color=INK_MUTED, lw=1.2, ls=(0, (5, 3)), zorder=1)
    ax.text(
        expected,
        ax.get_ylim()[1] if False else 0,
        "",
    )

    ax.plot(base[:, 1], base[:, 2], color=BLUE, lw=1.8, label=f"baseline → {d_base:.1f} m (20% of truth)", zorder=3)
    ax.plot(
        inten[:, 1],
        inten[:, 2],
        color=ORANGE,
        lw=1.8,
        label=f"intensity disagreement gate (no radar) → {d_inten:.1f} m",
        zorder=4,
    )
    ax.plot(
        radar[:, 1],
        radar[:, 2],
        color=GREEN,
        lw=1.8,
        label=f"radar (both gates, scale 1.05) → {d_radar:.1f} m (−0.9% of truth)",
        zorder=5,
    )

    ax.scatter([0], [0], s=70, marker="o", color=INK, zorder=6)
    ax.annotate("start", (0, 0), textcoords="offset points", xytext=(6, 10), fontsize=9, color=INK_SECONDARY)
    for arr, color in ((base, BLUE), (inten, ORANGE), (radar, GREEN)):
        ax.scatter([arr[-1, 1]], [arr[-1, 2]], s=65, marker="X", color=color, zorder=6, edgecolor=SURFACE, linewidth=0.8)

    ax.annotate(
        f"expected reach ≈ {expected:.0f} m",
        xy=(expected, ax.get_ylim()[0]),
        xytext=(-70, 14),
        textcoords="offset points",
        fontsize=9.5,
        color=INK_MUTED,
        fontweight="bold",
    )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        "Tunnel corridor: distance reached before tracking degrades",
        fontsize=12.5,
        color=INK,
        pad=12,
    )
    clean_axes(ax)
    ax.legend(loc="upper left", frameon=False, fontsize=9)
    fig.text(
        0.01,
        0.01,
        "NTNU LiDAR Degeneracy Datasets – tunnel sequence, one-way traverse, ground-truth reach ≈ 500 m. "
        "All trajectories start at the origin.",
        fontsize=7.5,
        color=INK_MUTED,
    )
    fig.tight_layout(rect=(0, 0.03, 1, 1))
    out = HERE / "tunnel_trajectory_xy.png"
    fig.savefig(out, dpi=175)
    plt.close(fig)
    print(
        f"wrote {out}  (baseline={d_base:.2f} m, radar-scale1.05={d_radar:.2f} m, "
        f"intensity-only={d_inten:.2f} m)"
    )


# ---------------------------------------------------------------------------
# Figure 3: gate coverage concept (fog sequence)
# ---------------------------------------------------------------------------
def make_gate_coverage_concept():
    # Real, verified numbers (fog radar-disagreement run, see
    # degeneracy_persistence.csv / radar_velocity_fusion_summary.json):
    #   total scans considered:            1723
    #   Hessian multiscan-observability
    #     gate CONFIRMED:                  85 scans, all within t=162.1-172.2s
    #     (last ~10 s of the 172.4 s recording)
    #   radar disagreement gate CORRECTED: 849 scans (w=0.25 sweep row in the
    #     research note table; the adopted w=1.0 run corrected 823/1723 scans
    #     with the same qualitative "corrects across the whole clutter-lock
    #     stretch" pattern -- exact per-scan firing times for the
    #     disagreement gate are not logged, so the shaded span below is
    #     illustrative of *where* it is active (the 0-110s clutter-lock
    #     window described in the note), not a scan-by-scan reconstruction).
    total_scans = 1723
    duration_s = 172.4

    eigen_fired = 85
    eigen_start_s, eigen_end_s = 162.1, 172.2

    disagreement_corrected = 849
    clutter_lock_start_s, clutter_lock_end_s = 0.0, 110.0

    fig, ax = plt.subplots(figsize=(9.5, 3.6), dpi=175)

    lane_h = 0.6
    y_eigen, y_dis = 2.0, 1.0

    # full-duration baseline track for each lane
    for y in (y_eigen, y_dis):
        ax.barh(y, duration_s, left=0, height=lane_h, color=GRID, zorder=1)

    # Hessian eigenvalue gate: real, narrow window
    ax.barh(
        y_eigen,
        eigen_end_s - eigen_start_s,
        left=eigen_start_s,
        height=lane_h,
        color=BLUE,
        zorder=2,
        label=f"Hessian eigenvalue gate – fired {eigen_fired}/{total_scans} scans",
    )

    # Disagreement gate: illustrative broad coverage over the clutter-lock window
    ax.barh(
        y_dis,
        clutter_lock_end_s - clutter_lock_start_s,
        left=clutter_lock_start_s,
        height=lane_h,
        color=ORANGE,
        alpha=0.55,
        zorder=2,
        hatch="///",
        edgecolor=ORANGE,
        linewidth=0,
        label=f"Sensor disagreement gate – corrected {disagreement_corrected}/{total_scans} scans (illustrative span)",
    )

    ax.annotate(
        f"{eigen_fired} scans\n(last ~10s only)",
        xy=(eigen_start_s, y_eigen),
        xytext=(eigen_start_s - 8, y_eigen + 0.55),
        ha="right",
        va="bottom",
        fontsize=8.5,
        color=BLUE,
        fontweight="bold",
        arrowprops=dict(arrowstyle="-", color=BLUE, lw=1.0),
    )
    ax.text(
        (clutter_lock_start_s + clutter_lock_end_s) / 2,
        y_dis,
        f"{disagreement_corrected} scans",
        ha="center",
        va="center",
        fontsize=9.5,
        color=INK,
        fontweight="bold",
    )

    ax.set_yticks([y_eigen, y_dis])
    ax.set_yticklabels(
        [
            "Hessian eigenvalue gate\n(hard degeneracy only)",
            "Sensor disagreement gate\n(radar/intensity vs ICP)",
        ],
        fontsize=9.5,
    )
    ax.set_xlim(0, duration_s)
    ax.set_ylim(0.4, 2.95)
    ax.set_xlabel("time since recording start [s]  (fog sequence, 172.4 s total)")
    ax.set_title(
        "Correction impact is bounded by gate coverage (fog sequence)",
        fontsize=12.5,
        color=INK,
        pad=12,
    )
    ax.grid(axis="x", color=GRID, linewidth=0.8)
    ax.grid(axis="y", visible=False)
    clean_axes(ax)
    ax.spines["left"].set_visible(False)
    ax.tick_params(axis="y", length=0)

    fig.text(
        0.01,
        0.02,
        "Eigenvalue-gate window and scan count are exact (degeneracy_persistence.csv). Disagreement-gate scan\n"
        "count is exact (research note, w=0.25 sweep); its shaded span is illustrative, matching the 0–110s\n"
        "clutter-lock window described in the note -- per-scan firing times were not logged for that run.",
        fontsize=7.3,
        color=INK_MUTED,
    )
    fig.tight_layout(rect=(0, 0.09, 1, 1))
    out = HERE / "gate_coverage_concept.png"
    fig.savefig(out, dpi=175)
    plt.close(fig)
    print(f"wrote {out}")


def main():
    make_fog_trajectory()
    make_tunnel_trajectory()
    make_gate_coverage_concept()


if __name__ == "__main__":
    main()
