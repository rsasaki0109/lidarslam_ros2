#!/usr/bin/env python3

import argparse
import bisect
import math
import statistics
import sys
from pathlib import Path
from typing import Any


def read_tum(path: Path) -> list[tuple[float, tuple[float, float, float]]]:
    poses: list[tuple[float, tuple[float, float, float]]] = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            try:
                ts = float(parts[0])
                xyz = (float(parts[1]), float(parts[2]), float(parts[3]))
            except Exception:
                continue
            poses.append((ts, xyz))
    poses.sort(key=lambda item: item[0])
    return poses


def associate(
    ref: list[tuple[float, tuple[float, float, float]]],
    est: list[tuple[float, tuple[float, float, float]]],
    max_diff: float,
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]], dict[str, Any]]:
    """Nearest-neighbour association: pair each reference sample with the
    temporally closest estimate sample, provided it is within max_diff.

    No interpolation is performed -- the position used for a matched
    reference sample is the estimate's own recorded (raw or corrected)
    position at its own timestamp. This is the right approach for sparse
    estimated trajectories (e.g. submap-rate corrected poses): it never
    fabricates a position by blending two estimate samples that may
    bracket a large time gap, it simply reports (and, via the returned
    diagnostics, exposes) how temporally stale the nearest available
    estimate sample is for each reference point.
    """
    est_times = [ts for ts, _ in est]
    est_xyz = [xyz for _, xyz in est]
    ref_xyz_matched: list[tuple[float, float, float]] = []
    est_xyz_matched: list[tuple[float, float, float]] = []
    matched_gaps: list[float] = []
    rejected = 0

    for ref_ts, ref_xyz in ref:
        idx = bisect.bisect_left(est_times, ref_ts)
        candidates = []
        if idx < len(est_times):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        best_idx = None
        best_dt = None
        for cand in candidates:
            dt = abs(est_times[cand] - ref_ts)
            if dt <= max_diff and (best_dt is None or dt < best_dt):
                best_idx = cand
                best_dt = dt
        if best_idx is None:
            rejected += 1
            continue
        ref_xyz_matched.append(ref_xyz)
        est_xyz_matched.append(est_xyz[best_idx])
        matched_gaps.append(best_dt if best_dt is not None else 0.0)

    diagnostics = {
        "total_ref_points": len(ref),
        "pairs": len(ref_xyz_matched),
        "rejected_ref_points": rejected,
        "max_time_gap": max(matched_gaps) if matched_gaps else None,
    }
    return ref_xyz_matched, est_xyz_matched, diagnostics


def interpolate_association(
    ref: list[tuple[float, tuple[float, float, float]]],
    est: list[tuple[float, tuple[float, float, float]]],
    max_edge_diff: float,
) -> tuple[list[tuple[float, float, float]], list[tuple[float, float, float]], dict[str, Any]]:
    """Sample the estimated trajectory at the reference timestamps.

    Linear position interpolation between the two estimated poses that
    bracket each reference timestamp. Made for sparse checkpoint references
    (e.g. total-station checkpoints taken while the platform is stationary,
    which fall into the time gaps of a submap-rate trajectory). Outside the
    estimated time range the nearest endpoint is used only when it is
    within max_edge_diff.

    Caveat (see scripts/write_aligned_trajectory_metrics.py and docs/research/project-plan-archive.md
    Sec. 2.4): this mode is appropriate when `est` is dense (e.g. raw
    per-scan odometry), so the bracketing samples are always close in time
    to the reference. If `est` itself is sparse (e.g. a submap-rate
    corrected trajectory), the two bracketing samples can be many seconds
    apart and the straight-line assumption between them does not hold,
    fabricating a position that can be several metres off the true one.
    For sparse estimated trajectories, use --sparse-match instead (see
    `associate`). The returned diagnostics' max_time_gap is the widest
    interpolation bracket span (or edge clamp distance) used for any
    matched reference point, so this risk is visible even when
    --interpolate is (mis)used on a sparse trajectory.
    """
    est_times = [ts for ts, _ in est]
    est_xyz = [xyz for _, xyz in est]
    ref_xyz_matched: list[tuple[float, float, float]] = []
    est_xyz_matched: list[tuple[float, float, float]] = []
    matched_spans: list[float] = []
    rejected = 0

    for ref_ts, ref_xyz in ref:
        idx = bisect.bisect_left(est_times, ref_ts)
        if idx == 0:
            gap = est_times[0] - ref_ts
            if gap > max_edge_diff:
                rejected += 1
                continue
            sampled = est_xyz[0]
            span = gap
        elif idx == len(est_times):
            gap = ref_ts - est_times[-1]
            if gap > max_edge_diff:
                rejected += 1
                continue
            sampled = est_xyz[-1]
            span = gap
        else:
            t0, t1 = est_times[idx - 1], est_times[idx]
            p0, p1 = est_xyz[idx - 1], est_xyz[idx]
            w = 0.0 if t1 == t0 else (ref_ts - t0) / (t1 - t0)
            sampled = (
                p0[0] + w * (p1[0] - p0[0]),
                p0[1] + w * (p1[1] - p0[1]),
                p0[2] + w * (p1[2] - p0[2]),
            )
            span = t1 - t0
        ref_xyz_matched.append(ref_xyz)
        est_xyz_matched.append(sampled)
        matched_spans.append(span)

    diagnostics = {
        "total_ref_points": len(ref),
        "pairs": len(ref_xyz_matched),
        "rejected_ref_points": rejected,
        "max_time_gap": max(matched_spans) if matched_spans else None,
    }
    return ref_xyz_matched, est_xyz_matched, diagnostics


def align_first_pose(
    ref_xyz: list[tuple[float, float, float]],
    est_xyz: list[tuple[float, float, float]],
) -> list[tuple[float, float, float]]:
    dx = ref_xyz[0][0] - est_xyz[0][0]
    dy = ref_xyz[0][1] - est_xyz[0][1]
    dz = ref_xyz[0][2] - est_xyz[0][2]
    return [(x + dx, y + dy, z + dz) for x, y, z in est_xyz]


def try_align_umeyama(
    ref_xyz: list[tuple[float, float, float]],
    est_xyz: list[tuple[float, float, float]],
) -> tuple[str, list[tuple[float, float, float]]]:
    try:
        import numpy as np
    except Exception:
        return "first_pose", align_first_pose(ref_xyz, est_xyz)

    ref = np.asarray(ref_xyz, dtype=float)
    est = np.asarray(est_xyz, dtype=float)
    if ref.shape[0] < 3:
        return "first_pose", align_first_pose(ref_xyz, est_xyz)

    mu_ref = ref.mean(axis=0)
    mu_est = est.mean(axis=0)
    ref_centered = ref - mu_ref
    est_centered = est - mu_est
    cov = est_centered.T @ ref_centered / ref.shape[0]

    try:
        u, _, vt = np.linalg.svd(cov)
    except Exception:
        return "first_pose", align_first_pose(ref_xyz, est_xyz)

    rot = vt.T @ u.T
    if np.linalg.det(rot) < 0:
        vt[-1, :] *= -1
        rot = vt.T @ u.T
    trans = mu_ref - rot @ mu_est
    aligned = (rot @ est.T).T + trans
    return "se3_umeyama", [tuple(row.tolist()) for row in aligned]


def calc_errors(
    ref_xyz: list[tuple[float, float, float]],
    est_xyz: list[tuple[float, float, float]],
) -> list[float]:
    out: list[float] = []
    for (rx, ry, rz), (ex, ey, ez) in zip(ref_xyz, est_xyz):
        out.append(math.sqrt((rx - ex) ** 2 + (ry - ey) ** 2 + (rz - ez) ** 2))
    return out


def write_report(
    path: Path,
    errors: list[float],
    pairs: int,
    alignment: str,
    mode: str,
    diagnostics: dict[str, Any],
) -> None:
    rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
    mean = statistics.fmean(errors)
    median = statistics.median(errors)
    std = statistics.pstdev(errors) if len(errors) > 1 else 0.0
    min_v = min(errors)
    max_v = max(errors)

    max_time_gap = diagnostics.get("max_time_gap")
    lines = [
        "APE translation (m)",
        f"pairs: {pairs}",
        f"alignment: {alignment}",
        f"rmse: {rmse}",
        f"mean: {mean}",
        f"median: {median}",
        f"std: {std}",
        f"min: {min_v}",
        f"max: {max_v}",
        f"mode: {mode}",
        f"total_ref_points: {diagnostics.get('total_ref_points', pairs)}",
        f"rejected_ref_points: {diagnostics.get('rejected_ref_points', 0)}",
        f"max_time_gap: {max_time_gap if max_time_gap is not None else 'nan'}",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description="Compute APE from two TUM trajectories.")
    ap.add_argument("--ref", required=True, help="Reference TUM trajectory")
    ap.add_argument("--est", required=True, help="Estimated TUM trajectory")
    ap.add_argument("--out", required=True, help="Output report path")
    ap.add_argument(
        "--max-time-diff",
        type=float,
        default=None,
        help="Max timestamp association gap in seconds. Default 0.05s for "
        "nearest-neighbour matching, 0.05s edge-extrapolation bound for "
        "--interpolate, or 10.0s when --sparse-match is set (all "
        "overridable).",
    )
    ap.add_argument(
        "--interpolate",
        action="store_true",
        help="Linearly interpolate the estimated trajectory at the reference "
        "timestamps instead of nearest-neighbour matching. For a DENSE "
        "estimated trajectory (e.g. raw per-scan odometry) whose samples "
        "are always close in time to the sparse reference; --max-time-diff "
        "then only bounds extrapolation at the ends. Do not use this for a "
        "sparse estimated trajectory (e.g. a submap-rate corrected "
        "trajectory) -- the interpolation brackets can span many seconds "
        "and fabricate a position that is off by several metres if the "
        "true path is not a straight line between them (see docs/research/project-plan-archive.md Sec. "
        "2.4). Use --sparse-match instead. Mutually exclusive with "
        "--sparse-match.",
    )
    ap.add_argument(
        "--sparse-match",
        action="store_true",
        help="Score a sparse estimated trajectory (e.g. submap-rate "
        "corrected poses) by nearest-neighbour matching instead of "
        "interpolation: each reference sample is paired with the "
        "temporally closest estimate sample, using the estimate's own "
        "recorded position (never fabricated by blending). Reference "
        "samples with no estimate pose within --max-time-diff (default "
        "10.0s in this mode) are rejected, not silently dropped -- see "
        "the 'rejected_ref_points' / 'max_time_gap' diagnostics in the "
        "output report. Opt-in; default behaviour (no flags) is "
        "unchanged. Mutually exclusive with --interpolate.",
    )
    args = ap.parse_args()

    if args.interpolate and args.sparse_match:
        ap.error("--interpolate and --sparse-match are mutually exclusive")

    if args.max_time_diff is not None:
        effective_max_diff = args.max_time_diff
    elif args.sparse_match:
        effective_max_diff = 10.0
    else:
        effective_max_diff = 0.05

    ref = read_tum(Path(args.ref).expanduser().resolve())
    est = read_tum(Path(args.est).expanduser().resolve())
    if not ref or not est:
        return 1

    if args.interpolate:
        mode = "interpolate"
        ref_xyz, est_xyz, diagnostics = interpolate_association(ref, est, effective_max_diff)
    else:
        mode = "sparse_match" if args.sparse_match else "nearest_neighbor"
        ref_xyz, est_xyz, diagnostics = associate(ref, est, effective_max_diff)

    if diagnostics.get("rejected_ref_points", 0) > 0:
        print(
            "warning: ape_from_tum rejected "
            f"{diagnostics['rejected_ref_points']}/{diagnostics['total_ref_points']} "
            f"reference point(s) with no estimate pose within {effective_max_diff}s "
            f"(mode={mode}); see 'rejected_ref_points' / 'max_time_gap' in {args.out}",
            file=sys.stderr,
        )

    if len(ref_xyz) < 2:
        return 1

    alignment, aligned_est = try_align_umeyama(ref_xyz, est_xyz)
    errors = calc_errors(ref_xyz, aligned_est)
    write_report(Path(args.out).expanduser().resolve(), errors, len(errors), alignment, mode, diagnostics)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
