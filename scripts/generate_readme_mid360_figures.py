#!/usr/bin/env python3
from __future__ import annotations

import bisect
import math
from pathlib import Path
from xml.sax.saxutils import escape

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "output"
IMAGE_DIR = ROOT / "lidarslam" / "images"

XY_OUT = IMAGE_DIR / "mid360_glim_compare_xy.svg"
ERR_OUT = IMAGE_DIR / "mid360_glim_compare_error.svg"
MAP_OUT = IMAGE_DIR / "mid360_glim_map_compare.png"
ATTITUDE_OUT = IMAGE_DIR / "mid360_glim_attitude_compare.png"
BAG_PATH = ROOT / "demo_data" / "glim_mid360" / "rosbag2_2024_04_16-14_17_01"
POINTS_TOPIC = "/livox/lidar"


def find_latest_any(patterns: list[str]) -> Path | None:
    candidates = []
    for pattern in patterns:
        candidates.extend(OUTPUT.glob(pattern))
    candidates = [path for path in candidates if path.exists()]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def load_tum(path: Path) -> list[dict[str, float]]:
    rows = []
    for line in path.read_text().splitlines():
        parts = line.strip().split()
        if len(parts) < 8:
            continue
        t, x, y, z, qx, qy, qz, qw = map(float, parts[:8])
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


def quat_to_mat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    xx, yy, zz = qx * qx, qy * qy, qz * qz
    xy, xz, yz = qx * qy, qx * qz, qy * qz
    wx, wy, wz = qw * qx, qw * qy, qw * qz
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def mat_to_quat(rot: np.ndarray) -> np.ndarray:
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
    quat = np.array([qx, qy, qz, qw], dtype=float)
    quat /= np.linalg.norm(quat)
    return quat


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float]:
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


def path_length(rows: list[dict[str, float]]) -> float:
    total = 0.0
    for a, b in zip(rows, rows[1:]):
        total += math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"]))
    return total


def match_rows(
    ref_rows: list[dict[str, float]],
    est_rows: list[dict[str, float]],
    tolerance: float,
) -> list[tuple[dict[str, float], dict[str, float]]]:
    est_times = [row["t"] for row in est_rows]
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


def rigid_align(
    pairs: list[tuple[dict[str, float], dict[str, float]]],
) -> tuple[np.ndarray, np.ndarray]:
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


def apply_alignment(
    rows: list[dict[str, float]], rot: np.ndarray, trans: np.ndarray
) -> list[dict[str, float]]:
    aligned = []
    for row in rows:
        pos = np.array([row["x"], row["y"], row["z"]], dtype=float)
        pos_aligned = rot @ pos + trans
        pose_rot = quat_to_mat(row["qx"], row["qy"], row["qz"], row["qw"])
        pose_aligned = rot @ pose_rot
        quat_aligned = mat_to_quat(pose_aligned)
        aligned.append(
            {
                **row,
                "x": float(pos_aligned[0]),
                "y": float(pos_aligned[1]),
                "z": float(pos_aligned[2]),
                "qx": float(quat_aligned[0]),
                "qy": float(quat_aligned[1]),
                "qz": float(quat_aligned[2]),
                "qw": float(quat_aligned[3]),
            }
        )
    return aligned


def interp_pose(rows: list[dict[str, float]], ts: list[float], t: float) -> tuple[np.ndarray, np.ndarray] | None:
    idx = bisect.bisect_left(ts, t)
    if idx == 0 or idx == len(rows):
        return None
    a = rows[idx - 1]
    b = rows[idx]
    t0 = a["t"]
    t1 = b["t"]
    if t1 <= t0:
        alpha = 0.0
    else:
        alpha = (t - t0) / (t1 - t0)
    pos0 = np.array([a["x"], a["y"], a["z"]], dtype=float)
    pos1 = np.array([b["x"], b["y"], b["z"]], dtype=float)
    pos = pos0 * (1.0 - alpha) + pos1 * alpha
    quat0 = np.array([a["qx"], a["qy"], a["qz"], a["qw"]], dtype=float)
    quat1 = np.array([b["qx"], b["qy"], b["qz"], b["qw"]], dtype=float)
    quat = quat0 * (1.0 - alpha) + quat1 * alpha
    quat /= np.linalg.norm(quat)
    return pos, quat


def sample_map_points(
    bag_path: Path,
    traj_rows: list[dict[str, float]],
    scan_stride: int = 12,
    point_stride: int = 32,
) -> tuple[np.ndarray, np.ndarray]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(
            'generate_readme_mid360_figures map sampling requires ROS2 '
            'rosbag2_py, rclpy, and rosidl_runtime_py at execution time'
        ) from error
    reader = rosbag2_py.SequentialReader()
    storage = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage, converter)
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    msg_type = get_message(topics[POINTS_TOPIC])
    traj_ts = [row["t"] for row in traj_rows]

    cloud_world = []
    path_xy = []
    dtype = None
    msg_count = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != POINTS_TOPIC:
            continue
        msg_count += 1
        if msg_count % scan_stride != 0:
            continue
        msg = deserialize_message(data, msg_type)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = interp_pose(traj_rows, traj_ts, stamp)
        if pose is None:
            continue
        pos, quat = pose
        path_xy.append(pos[:2].copy())
        if dtype is None:
            dtype = np.dtype(
                {
                    "names": ["x", "y", "z"],
                    "formats": ["<f4", "<f4", "<f4"],
                    "offsets": [0, 4, 8],
                    "itemsize": msg.point_step,
                }
            )
        arr = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
        pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1)
        pts = pts[::point_stride]
        pts = pts[np.isfinite(pts).all(axis=1)]
        if pts.size == 0:
            continue
        rot = quat_to_mat(quat[0], quat[1], quat[2], quat[3])
        world = pts @ rot.T + pos
        cloud_world.append(world)

    if not cloud_world:
        return np.empty((0, 3)), np.empty((0, 2))
    return np.concatenate(cloud_world, axis=0), np.stack(path_xy, axis=0)


def build_map_png(
    glim_rows: list[dict[str, float]],
    lid_rows: list[dict[str, float]],
    summary: dict[str, float],
) -> None:
    glim_cloud, glim_path = sample_map_points(BAG_PATH, glim_rows)
    lid_cloud, lid_path = sample_map_points(BAG_PATH, lid_rows)
    if glim_cloud.size == 0 or lid_cloud.size == 0:
        raise RuntimeError("failed to build sampled point-cloud map")

    combined = np.concatenate([glim_cloud[:, 2], lid_cloud[:, 2]])
    z_lo = float(np.percentile(combined, 2))
    z_hi = float(np.percentile(combined, 98))

    fig = plt.figure(figsize=(13.8, 7.0), dpi=180, facecolor="#f6f8fb")
    gs = fig.add_gridspec(
        2,
        3,
        height_ratios=[0.18, 1.0],
        width_ratios=[1.0, 1.0, 0.055],
        left=0.055,
        right=0.96,
        top=0.94,
        bottom=0.09,
        wspace=0.12,
        hspace=0.16,
    )
    ax_header = fig.add_subplot(gs[0, :2])
    ax_header.axis("off")
    axes = [fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])]
    cax = fig.add_subplot(gs[1, 2])

    ax_header.text(
        0.0,
        0.82,
        "GLIM MID360 sample: top-down point-cloud map",
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
            "Same bag, same viewpoint. Colors encode height. "
            "The black trace is the estimated path."
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
            f"RMSE {summary['rmse']:.3f} m\n"
            f"median {summary['median']:.3f} m\n"
            f"max {summary['max']:.3f} m"
        ),
        fontsize=10.5,
        ha="right",
        va="top",
        color="#13202b",
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "white", "edgecolor": "#d8e3ef"},
    )

    bounds = np.vstack([glim_cloud[:, :2], lid_cloud[:, :2]])
    min_xy = bounds.min(axis=0)
    max_xy = bounds.max(axis=0)
    span = np.maximum(max_xy - min_xy, 1.0)
    pad = span * 0.04
    xlim = (min_xy[0] - pad[0], max_xy[0] + pad[0])
    ylim = (min_xy[1] - pad[1], max_xy[1] + pad[1])

    sc = None
    for ax, title, cloud, path in [
        (axes[0], "GLIM reference", glim_cloud, glim_path),
        (axes[1], "lidarslam aligned", lid_cloud, lid_path),
    ]:
        ax.set_facecolor("white")
        sc = ax.scatter(
            cloud[:, 0],
            cloud[:, 1],
            c=np.clip(cloud[:, 2], z_lo, z_hi),
            s=0.22,
            cmap="viridis",
            linewidths=0.0,
            alpha=0.85,
        )
        ax.plot(path[:, 0], path[:, 1], color="white", linewidth=1.0, alpha=0.9)
        ax.plot(path[:, 0], path[:, 1], color="#13202b", linewidth=0.45, alpha=0.85)
        ax.set_title(title, fontsize=13, fontweight="bold", pad=10)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, color="#e9eef4", linewidth=0.8)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        for spine in ax.spines.values():
            spine.set_edgecolor("#d8e3ef")
    cbar = fig.colorbar(sc, cax=cax)
    cbar.set_label("height [m]")
    cbar.outline.set_edgecolor("#d8e3ef")
    fig.savefig(MAP_OUT, facecolor=fig.get_facecolor())
    plt.close(fig)


def build_attitude_png(
    aligned_pairs: list[tuple[dict[str, float], dict[str, float]]],
    summary: dict[str, float],
) -> None:
    times = [ref["t"] - aligned_pairs[0][0]["t"] for ref, _ in aligned_pairs]
    glim_rpy = np.array(
        [quat_to_rpy(ref["qx"], ref["qy"], ref["qz"], ref["qw"]) for ref, _ in aligned_pairs],
        dtype=float,
    )
    lid_rpy = np.array(
        [quat_to_rpy(est["qx"], est["qy"], est["qz"], est["qw"]) for _, est in aligned_pairs],
        dtype=float,
    )
    glim_rpy = np.degrees(glim_rpy)
    lid_rpy = np.degrees(lid_rpy)

    fig, axes = plt.subplots(
        3,
        1,
        figsize=(12.8, 7.2),
        dpi=180,
        sharex=True,
        facecolor="#f6f8fb",
    )
    fig.subplots_adjust(left=0.08, right=0.97, top=0.86, bottom=0.09, hspace=0.20)
    fig.suptitle(
        "GLIM MID360 sample: attitude time series",
        fontsize=18,
        fontweight="bold",
        y=0.965,
    )
    fig.text(
        0.08,
        0.915,
        (
            "Rigid alignment applied before comparison. "
            f"Reference metrics: RMSE {summary['rmse']:.3f} m, median {summary['median']:.3f} m, max {summary['max']:.3f} m"
        ),
        ha="left",
        va="center",
        fontsize=10.5,
        color="#516679",
    )

    labels = [("Roll", 0), ("Pitch", 1), ("Yaw", 2)]
    for ax, (label, idx) in zip(axes, labels):
        ax.set_facecolor("white")
        ax.plot(times, glim_rpy[:, idx], color="#0b6bcb", linewidth=1.8, label="GLIM")
        ax.plot(times, lid_rpy[:, idx], color="#bc4b2f", linewidth=1.6, label="lidarslam")
        ax.set_ylabel(f"{label} [deg]")
        ax.grid(True, color="#e9eef4", linewidth=0.8)
        for spine in ax.spines.values():
            spine.set_edgecolor("#d8e3ef")
        if idx == 0:
            ax.legend(loc="upper right", frameon=True, facecolor="white", edgecolor="#d8e3ef")
    axes[-1].set_xlabel("Time [s]")
    fig.savefig(ATTITUDE_OUT, facecolor=fig.get_facecolor())
    plt.close(fig)


def ticks(min_v: float, max_v: float, count: int = 6) -> list[float]:
    if math.isclose(min_v, max_v):
        return [min_v]
    step = (max_v - min_v) / max(count - 1, 1)
    return [min_v + step * i for i in range(count)]


def fmt_meter(value: float) -> str:
    return f"{value:.1f}"


def line(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def build_xy_svg(
    glim_rows: list[dict[str, float]],
    lid_rows: list[dict[str, float]],
    summary: dict[str, float],
) -> str:
    width = 1080
    height = 640
    left = 96
    right = 24
    top = 116
    bottom = 72
    plot_w = width - left - right
    plot_h = height - top - bottom

    all_x = [row["x"] for row in glim_rows + lid_rows]
    all_y = [row["y"] for row in glim_rows + lid_rows]
    min_x = min(all_x)
    max_x = max(all_x)
    min_y = min(all_y)
    max_y = max(all_y)
    pad_x = max((max_x - min_x) * 0.04, 1.0)
    pad_y = max((max_y - min_y) * 0.04, 1.0)
    min_x -= pad_x
    max_x += pad_x
    min_y -= pad_y
    max_y += pad_y

    span_x = max_x - min_x
    span_y = max_y - min_y
    scale = min(plot_w / span_x, plot_h / span_y)
    draw_w = span_x * scale
    draw_h = span_y * scale
    x_offset = left + (plot_w - draw_w) / 2.0
    y_offset = top + (plot_h - draw_h) / 2.0

    def project(row: dict[str, float]) -> tuple[float, float]:
        px = x_offset + (row["x"] - min_x) * scale
        py = y_offset + draw_h - (row["y"] - min_y) * scale
        return px, py

    glim_poly = line([project(row) for row in glim_rows])
    lid_poly = line([project(row) for row in lid_rows])

    x_ticks = ticks(min_x, max_x)
    y_ticks = ticks(min_y, max_y)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="#f6f8fb"/>',
        '<text x="28" y="38" font-family="Arial, sans-serif" font-size="28" font-weight="700" fill="#13202b">GLIM MID360 sample: XY trajectory overlay</text>',
        '<text x="28" y="66" font-family="Arial, sans-serif" font-size="16" fill="#516679">Rigid alignment applied. The plot uses a fixed aspect ratio to preserve geometry.</text>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#ffffff" stroke="#d8e3ef"/>',
    ]

    for value in x_ticks:
        x = x_offset + (value - min_x) * scale
        parts.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#e9eef4" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{x:.2f}" y="{height - 28}" text-anchor="middle" font-family="Arial, sans-serif" font-size="13" fill="#607487">{escape(fmt_meter(value))}</text>'
        )
    for value in y_ticks:
        y = y_offset + draw_h - (value - min_y) * scale
        parts.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#e9eef4" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="84" y="{y + 4:.2f}" text-anchor="end" font-family="Arial, sans-serif" font-size="13" fill="#607487">{escape(fmt_meter(value))}</text>'
        )

    parts.extend(
        [
            f'<polyline points="{glim_poly}" fill="none" stroke="#0b6bcb" stroke-width="3.0" vector-effect="non-scaling-stroke"/>',
            f'<polyline points="{lid_poly}" fill="none" stroke="#bc4b2f" stroke-width="2.6" vector-effect="non-scaling-stroke"/>',
        ]
    )

    for row, color, label in [
        (glim_rows[0], "#0b6bcb", "GLIM start"),
        (glim_rows[-1], "#0b6bcb", "GLIM end"),
        (lid_rows[0], "#bc4b2f", "lidarslam start"),
        (lid_rows[-1], "#bc4b2f", "lidarslam end"),
    ]:
        x, y = project(row)
        parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5.5" fill="{color}" stroke="#ffffff" stroke-width="2"/>')

    parts.extend(
        [
            f'<text x="{left + plot_w / 2:.2f}" y="{height - 10}" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#425568">X [m]</text>',
            f'<text x="22" y="{top + plot_h / 2:.2f}" transform="rotate(-90 22 {top + plot_h / 2:.2f})" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#425568">Y [m]</text>',
            '<rect x="760" y="18" width="292" height="82" rx="10" fill="#ffffff" stroke="#d8e3ef"/>',
            '<line x1="780" y1="42" x2="816" y2="42" stroke="#0b6bcb" stroke-width="4"/>',
            '<text x="826" y="47" font-family="Arial, sans-serif" font-size="15" fill="#0b6bcb">GLIM reference</text>',
            '<line x1="780" y1="68" x2="816" y2="68" stroke="#bc4b2f" stroke-width="4"/>',
            '<text x="826" y="73" font-family="Arial, sans-serif" font-size="15" fill="#bc4b2f">lidarslam aligned</text>',
            f'<text x="780" y="92" font-family="Arial, sans-serif" font-size="13" fill="#425568">RMSE {summary["rmse"]:.3f} m, median {summary["median"]:.3f} m, max {summary["max"]:.3f} m</text>',
            f'<text x="780" y="108" font-family="Arial, sans-serif" font-size="13" fill="#425568">Path lengths: GLIM {summary["glim_path"]:.2f} m, lidarslam {summary["lid_path"]:.2f} m</text>',
        ]
    )
    parts.append("</svg>")
    return "\n".join(parts)


def build_error_svg(
    errors: list[tuple[float, float]],
    summary: dict[str, float],
) -> str:
    width = 1080
    height = 460
    left = 88
    right = 24
    top = 92
    bottom = 68
    plot_w = width - left - right
    plot_h = height - top - bottom

    min_t = 0.0
    max_t = max(t for t, _ in errors)
    min_e = 0.0
    max_e = max(e for _, e in errors)
    y_max = max_e * 1.08

    def project(t: float, e: float) -> tuple[float, float]:
        x = left + (t - min_t) / max(max_t - min_t, 1e-9) * plot_w
        y = top + plot_h - (e - min_e) / max(y_max - min_e, 1e-9) * plot_h
        return x, y

    err_poly = line([project(t, e) for t, e in errors])
    x_ticks = ticks(min_t, max_t)
    y_ticks = ticks(min_e, y_max)

    rmse_y = project(0.0, summary["rmse"])[1]
    median_y = project(0.0, summary["median"])[1]

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="#f6f8fb"/>',
        '<text x="28" y="38" font-family="Arial, sans-serif" font-size="28" font-weight="700" fill="#13202b">GLIM MID360 sample: position error after rigid alignment</text>',
        '<text x="28" y="66" font-family="Arial, sans-serif" font-size="16" fill="#516679">Lower is better. Error is the 3D distance between time-matched poses after SE(3) alignment.</text>',
        f'<rect x="{left}" y="{top}" width="{plot_w}" height="{plot_h}" fill="#ffffff" stroke="#d8e3ef"/>',
    ]

    for value in x_ticks:
        x = left + (value - min_t) / max(max_t - min_t, 1e-9) * plot_w
        parts.append(
            f'<line x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + plot_h}" stroke="#e9eef4" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="{x:.2f}" y="{height - 26}" text-anchor="middle" font-family="Arial, sans-serif" font-size="13" fill="#607487">{value:.0f}</text>'
        )
    for value in y_ticks:
        y = top + plot_h - (value - min_e) / max(y_max - min_e, 1e-9) * plot_h
        parts.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_w}" y2="{y:.2f}" stroke="#e9eef4" stroke-width="1"/>'
        )
        parts.append(
            f'<text x="76" y="{y + 4:.2f}" text-anchor="end" font-family="Arial, sans-serif" font-size="13" fill="#607487">{value:.1f}</text>'
        )

    parts.extend(
        [
            f'<line x1="{left}" y1="{rmse_y:.2f}" x2="{left + plot_w}" y2="{rmse_y:.2f}" stroke="#0ea5a3" stroke-dasharray="8 6" stroke-width="2"/>',
            f'<line x1="{left}" y1="{median_y:.2f}" x2="{left + plot_w}" y2="{median_y:.2f}" stroke="#f59e0b" stroke-dasharray="8 6" stroke-width="2"/>',
            f'<polyline points="{err_poly}" fill="none" stroke="#7a3cff" stroke-width="2.4" vector-effect="non-scaling-stroke"/>',
            f'<text x="{left + plot_w / 2:.2f}" y="{height - 10}" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#425568">Time [s]</text>',
            f'<text x="22" y="{top + plot_h / 2:.2f}" transform="rotate(-90 22 {top + plot_h / 2:.2f})" text-anchor="middle" font-family="Arial, sans-serif" font-size="15" fill="#425568">Position error [m]</text>',
            '<rect x="714" y="18" width="338" height="88" rx="10" fill="#ffffff" stroke="#d8e3ef"/>',
            '<line x1="734" y1="42" x2="770" y2="42" stroke="#7a3cff" stroke-width="4"/>',
            '<text x="780" y="47" font-family="Arial, sans-serif" font-size="15" fill="#7a3cff">error trace</text>',
            '<line x1="734" y1="68" x2="770" y2="68" stroke="#0ea5a3" stroke-dasharray="8 6" stroke-width="3"/>',
            f'<text x="780" y="73" font-family="Arial, sans-serif" font-size="15" fill="#0ea5a3">RMSE {summary["rmse"]:.3f} m</text>',
            '<line x1="734" y1="94" x2="770" y2="94" stroke="#f59e0b" stroke-dasharray="8 6" stroke-width="3"/>',
            f'<text x="780" y="99" font-family="Arial, sans-serif" font-size="15" fill="#f59e0b">median {summary["median"]:.3f} m, max {summary["max"]:.3f} m</text>',
        ]
    )
    parts.append("</svg>")
    return "\n".join(parts)


def main() -> None:
    glim_dir = find_latest_any(["glim_mid360_sample_*"])
    lid_dir = find_latest_any(
        [
            "lidarslam_mid360_auto_*",
            "lidarslam_mid360_noimu_nograph_fix_*",
            "lidarslam_mid360_noimu_*",
            "lidarslam_mid360_clean_*",
        ]
    )
    if glim_dir is None or lid_dir is None:
        raise SystemExit("required MID360 runs not found")

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
    for ref, est in aligned_pairs:
        errors.append(
            (
                ref["t"] - aligned_pairs[0][0]["t"],
                math.dist((ref["x"], ref["y"], ref["z"]), (est["x"], est["y"], est["z"])),
            )
        )

    err_values = [value for _, value in errors]
    summary = {
        "glim_path": path_length(glim_rows),
        "lid_path": path_length(lid_rows),
        "rmse": math.sqrt(sum(v * v for v in err_values) / len(err_values)),
        "median": sorted(err_values)[len(err_values) // 2],
        "max": max(err_values),
    }

    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    XY_OUT.write_text(build_xy_svg(glim_rows, lid_aligned, summary), encoding="utf-8")
    ERR_OUT.write_text(build_error_svg(errors, summary), encoding="utf-8")
    build_map_png(glim_rows, lid_aligned, summary)
    build_attitude_png(aligned_pairs, summary)
    print(XY_OUT)
    print(ERR_OUT)
    print(MAP_OUT)
    print(ATTITUDE_OUT)


if __name__ == "__main__":
    main()
