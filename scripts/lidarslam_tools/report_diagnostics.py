"""Log anomaly detection and alert rendering for benchmark reports."""

from __future__ import annotations

import html
import re
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.lidarslam_tools.report_model import RunRecord, badge


def collect_log_alerts(run_dir: Path, limit: int = 6) -> list[dict[str, Any]]:
    patterns = [
        ("fatal", "bad", re.compile(r"(fatal|traceback|segmentation fault|core dumped|terminate called)", re.IGNORECASE)),
        ("pose_jump", "warn", re.compile(r"(POSE_JUMP|POSE_REJECT|POSE_STAMP_NONMONOTONIC)", re.IGNORECASE)),
        ("tf", "warn", re.compile(r"(TF_OLD_DATA|from the past for frame|extrapolation.+past)", re.IGNORECASE)),
        ("timeout", "warn", re.compile(r"(timeout|timed out|rc=124|exit status 124)", re.IGNORECASE)),
        ("ndt", "warn", re.compile(r"(ndt.+(warn|fail|error|degener|conver)|degeneracy|no transform available)", re.IGNORECASE)),
        ("drop", "warn", re.compile(r"(drop(ped)?|queue full|skipping message|discard)", re.IGNORECASE)),
        ("numeric", "bad", re.compile(r"(\bnan\b|\binf\b|invalid value)", re.IGNORECASE)),
    ]
    alerts: list[dict[str, Any]] = []
    seen: set[tuple[str, str]] = set()
    for log_path in sorted(run_dir.glob("*.log")):
        try:
            with log_path.open("r", encoding="utf-8", errors="replace") as f:
                for lineno, raw in enumerate(f, start=1):
                    line = " ".join(raw.strip().split())
                    if not line:
                        continue
                    for label, tone, pattern in patterns:
                        if not pattern.search(line):
                            continue
                        snippet = line[:220]
                        key = (label, snippet)
                        if key in seen:
                            break
                        seen.add(key)
                        alerts.append(
                            {
                                "label": label.upper(),
                                "tone": tone,
                                "file": log_path.name,
                                "line": lineno,
                                "snippet": snippet,
                            }
                        )
                        break
                    if len(alerts) >= limit:
                        return alerts
        except Exception:
            continue
    return alerts


def render_log_alerts(rec: RunRecord) -> str:
    alerts = collect_log_alerts(rec.metrics_path.parent)
    if not alerts:
        return "<p class='plot-meta'>no obvious TF / timeout / NDT warnings found in local logs.</p>"
    cards = []
    for alert in alerts:
        cards.append(
            "<div class='alert-card'>"
            f"{badge(str(alert['label']), str(alert['tone']))}"
            f"<span class='alert-src'>{html.escape(str(alert['file']))}:{alert['line']}</span>"
            f"<p>{html.escape(str(alert['snippet']))}</p>"
            "</div>"
        )
    return "<div class='alert-grid'>" + "".join(cards) + "</div>"
