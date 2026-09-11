#!/usr/bin/env python3

import argparse
import csv
import json
import re
import statistics
from pathlib import Path
from typing import Any

try:
    import yaml  # type: ignore
except ImportError:  # pragma: no cover - PyYAML missing only on minimal builds
    yaml = None  # type: ignore


# Metric kinds supported by release profiles (extend cautiously - each value
# must be derivable from a per-run record built below in main()).
_PROFILE_METRICS = (
    "ape_rmse_gt_m",
    "ape_rmse_vs_reference_m",
)


def _fmt_float(v: Any, digits: int = 3) -> str:
    if v is None:
        return ""
    try:
        return f"{float(v):.{digits}f}"
    except Exception:
        return ""


def _fmt_int(v: Any) -> str:
    if v is None:
        return ""
    try:
        return str(int(v))
    except Exception:
        return ""


def _as_bool(v: Any):
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        vv = v.strip().lower()
        if vv in ("true", "1", "yes", "y", "on"):
            return True
        if vv in ("false", "0", "no", "n", "off"):
            return False
    return None


def _as_float(v: Any) -> float | None:
    if v is None:
        return None
    try:
        return float(v)
    except Exception:
        return None


def _valid_file_identity(value: Any) -> bool:
    return (
        isinstance(value, dict)
        and isinstance(value.get("path"), str)
        and bool(value["path"])
        and isinstance(value.get("size_bytes"), int)
        and value["size_bytes"] >= 0
        and isinstance(value.get("sha256"), str)
        and re.fullmatch(r"[0-9a-f]{64}", value["sha256"]) is not None
    )


def _provenance_state(
    run: dict[str, Any],
) -> tuple[bool, bool | None, str | None]:
    provenance = run.get("provenance")
    if not isinstance(provenance, dict):
        return False, None, None
    input_identity = provenance.get("input")
    software = provenance.get("software")
    if not isinstance(input_identity, dict) or not isinstance(software, dict):
        return False, None, None
    bag = input_identity.get("bag")
    storage_files = bag.get("storage_files") if isinstance(bag, dict) else None
    parameter_files = software.get("parameter_files")
    runtime_artifacts = software.get("runtime_artifacts")
    commit = software.get("git_commit")
    dirty = software.get("git_dirty")
    complete = (
        isinstance(bag, dict)
        and _valid_file_identity(bag.get("metadata"))
        and isinstance(storage_files, list)
        and bool(storage_files)
        and all(_valid_file_identity(item) for item in storage_files)
        and _valid_file_identity(input_identity.get("reference_trajectory"))
        and isinstance(commit, str)
        and re.fullmatch(r"[0-9a-f]{40}", commit) is not None
        and isinstance(dirty, bool)
        and isinstance(parameter_files, list)
        and bool(parameter_files)
        and all(_valid_file_identity(item) for item in parameter_files)
        and isinstance(runtime_artifacts, list)
        and bool(runtime_artifacts)
        and all(
            isinstance(item, dict)
            and isinstance(item.get("label"), str)
            and bool(item["label"])
            and _valid_file_identity(item)
            for item in runtime_artifacts
        )
        and _valid_file_identity(software.get("benchmark_harness"))
        and _valid_file_identity(software.get("metrics_writer"))
    )
    valid_commit = (
        commit
        if isinstance(commit, str)
        and re.fullmatch(r"[0-9a-f]{40}", commit) is not None
        else None
    )
    return complete, dirty if isinstance(dirty, bool) else None, valid_commit


def _load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _infer_reference_kind(reference_source: Any, explicit_kind: Any) -> str:
    if explicit_kind:
        return str(explicit_kind)
    lowered = str(reference_source or "").strip().lower()
    if "gt" in lowered or "ground_truth" in lowered:
        return "ground_truth"
    if "glim" in lowered or "cross" in lowered:
        return "cross_validation"
    return ""


def _median(vals: list[float]) -> float | None:
    vals = [v for v in vals if v is not None]
    if not vals:
        return None
    return float(statistics.median(vals))


def _write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _primary_sort_key(primary: str, rec: dict[str, Any]) -> tuple[int, float]:
    missing = bool(rec.get("primary_missing"))
    raw = rec.get("primary_raw")
    raw_f = _as_float(raw)
    return (
        1 if missing or raw_f is None else 0,
        float("inf") if raw_f is None else raw_f,
    )


def _primary_value_text(primary: str, rec: dict[str, Any]) -> str:
    if primary == "lid_rtf":
        return rec.get("lid_rtf", "")
    if primary == "glim_rtf":
        return rec.get("glim_rtf", "")
    if primary == "wall_sec":
        return rec.get("lid_wall_s", "")
    return rec.get("ape_rmse_m", "")


def load_release_profiles(path: Path) -> list[dict[str, Any]]:
    """Load and minimally validate a release-profile YAML file."""
    if yaml is None:
        raise RuntimeError(
            "PyYAML is required to load release profiles; install python3-yaml"
        )
    with path.open("r", encoding="utf-8") as f:
        doc = yaml.safe_load(f) or {}
    profiles = doc.get("release_profiles")
    if not isinstance(profiles, list):
        raise ValueError(
            f"{path}: expected top-level key 'release_profiles' to be a list"
        )
    validated: list[dict[str, Any]] = []
    seen: set[str] = set()
    for i, prof in enumerate(profiles):
        if not isinstance(prof, dict):
            raise ValueError(f"{path}: profile index {i} is not a mapping")
        name = prof.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError(f"{path}: profile index {i} missing 'name'")
        if name in seen:
            raise ValueError(f"{path}: duplicate profile name '{name}'")
        seen.add(name)
        metric = prof.get("metric")
        if metric not in _PROFILE_METRICS:
            raise ValueError(
                f"{path}: profile '{name}' metric must be one of {_PROFILE_METRICS}, "
                f"got {metric!r}"
            )
        if not isinstance(prof.get("pass"), (int, float)):
            raise ValueError(f"{path}: profile '{name}' missing numeric 'pass'")
        remediation = prof.get("remediation")
        if remediation is not None and (
            not isinstance(remediation, str) or not remediation.strip()
        ):
            raise ValueError(
                f"{path}: profile '{name}' remediation must be a non-empty string"
            )
        match = prof.get("match") or {}
        if not isinstance(match, dict):
            raise ValueError(f"{path}: profile '{name}' 'match' must be a mapping")
        require_clean = match.get("require_clean_provenance")
        if require_clean is not None and not isinstance(require_clean, bool):
            raise ValueError(
                f"{path}: profile '{name}' require_clean_provenance must be boolean"
            )
        validated.append(prof)
    return validated


def _profile_match(
    profile: dict[str, Any],
    rec: dict[str, Any],
    *,
    check_provenance: bool = True,
    required_git_commit: str | None = None,
) -> bool:
    match = profile.get("match") or {}
    commit_bound = bool(
        required_git_commit and not profile.get("report_only_until")
    )
    bag_substr = match.get("bag_name_contains")
    if bag_substr and bag_substr not in (rec.get("bag") or ""):
        return False
    points_topic = match.get("points_topic")
    if points_topic and points_topic != (rec.get("points_topic") or ""):
        return False
    ref_kind = match.get("reference_kind")
    if ref_kind and ref_kind != (rec.get("ape_ref_kind") or ""):
        return False
    ref_substr = match.get("reference_source_contains")
    if ref_substr and ref_substr not in (rec.get("ape_ref_src") or ""):
        return False
    min_pairs = match.get("min_ape_pairs")
    if min_pairs is not None:
        pairs = _as_float(rec.get("ape_pairs"))
        if pairs is None or pairs < float(min_pairs):
            return False
    if check_provenance and (
        match.get("require_clean_provenance") or commit_bound
    ):
        if (
            rec.get("provenance_complete") is not True
            or rec.get("provenance_git_dirty") is not False
        ):
            return False
    if (
        check_provenance
        and commit_bound
        and rec.get("provenance_git_commit") != required_git_commit
    ):
        return False
    return True


def _profile_metric_value(profile: dict[str, Any], rec: dict[str, Any]) -> float | None:
    metric = profile.get("metric")
    if metric == "ape_rmse_gt_m":
        if (rec.get("ape_ref_kind") or "") != "ground_truth":
            return None
        return _as_float(rec.get("ape_rmse_m"))
    if metric == "ape_rmse_vs_reference_m":
        return _as_float(rec.get("ape_rmse_m"))
    return None


def evaluate_release_profiles(
    profiles: list[dict[str, Any]],
    records: list[dict[str, Any]],
    *,
    required_git_commit: str | None = None,
) -> list[dict[str, Any]]:
    """For each profile, find the best matching run and assign a status.

    Status values:
      PASS        - best matching metric <= pass threshold
      FAIL        - best matching metric > pass threshold (gate active)
      WARN        - same as FAIL but profile has report_only_until set
      NO_DATA     - no matching runs found
      TARGET_MET  - PASS and metric <= target (display-only bonus)
    """
    results: list[dict[str, Any]] = []
    for prof in profiles:
        candidates = [
            rec for rec in records
            if _profile_match(prof, rec, check_provenance=False)
            and _profile_metric_value(prof, rec) is not None
        ]
        matched = [
            rec for rec in candidates
            if _profile_match(
                prof,
                rec,
                required_git_commit=required_git_commit,
            )
        ]
        provenance_rejections: dict[str, list[str]] = {
            "incomplete": [],
            "dirty": [],
            "commit_mismatch": [],
        }
        commit_bound = bool(
            required_git_commit and not prof.get("report_only_until")
        )
        if (
            (prof.get("match") or {}).get("require_clean_provenance")
            or commit_bound
        ):
            for rec in candidates:
                if rec.get("provenance_complete") is not True:
                    label = str(rec.get("run") or "<unnamed>")
                    if label not in provenance_rejections["incomplete"]:
                        provenance_rejections["incomplete"].append(label)
                elif rec.get("provenance_git_dirty") is not False:
                    label = str(rec.get("run") or "<unnamed>")
                    if label not in provenance_rejections["dirty"]:
                        provenance_rejections["dirty"].append(label)
                elif (
                    commit_bound
                    and rec.get("provenance_git_commit") != required_git_commit
                ):
                    label = (
                        f"{rec.get('run') or '<unnamed>'}"
                        f"@{str(rec.get('provenance_git_commit') or 'unknown')[:12]}"
                    )
                    if label not in provenance_rejections["commit_mismatch"]:
                        provenance_rejections["commit_mismatch"].append(label)
        result: dict[str, Any] = {
            "name": prof["name"],
            "description": prof.get("description", ""),
            "metric": prof["metric"],
            "pass": float(prof["pass"]),
            "target": _as_float(prof.get("target")),
            "report_only_until": prof.get("report_only_until"),
            "remediation": prof.get("remediation"),
            "matched_runs": len(matched),
            "candidate_runs": len(candidates),
            "provenance_rejections": provenance_rejections,
            "required_git_commit": (
                required_git_commit
                if required_git_commit and not prof.get("report_only_until")
                else None
            ),
        }
        if not matched:
            result["status"] = "NO_DATA"
            result["best_run"] = None
            result["best_value"] = None
            if any(provenance_rejections.values()):
                reasons = []
                if provenance_rejections["incomplete"]:
                    reasons.append(
                        "incomplete provenance: "
                        + ", ".join(provenance_rejections["incomplete"])
                    )
                if provenance_rejections["dirty"]:
                    reasons.append(
                        "dirty revision: "
                        + ", ".join(provenance_rejections["dirty"])
                    )
                if provenance_rejections["commit_mismatch"]:
                    reasons.append(
                        f"candidate commit mismatch (required "
                        f"{required_git_commit[:12]}): "
                        + ", ".join(provenance_rejections["commit_mismatch"])
                    )
                result["no_data_reason"] = "; ".join(reasons)
            else:
                result["no_data_reason"] = "no matching run"
            results.append(result)
            continue
        scored = [
            (rec, _profile_metric_value(prof, rec))
            for rec in matched
        ]
        scored.sort(key=lambda item: item[1] if item[1] is not None else float("inf"))
        best_rec, best_value = scored[0]
        result["best_run"] = best_rec.get("run")
        result["best_value"] = best_value
        if best_value <= result["pass"]:
            if result["target"] is not None and best_value <= result["target"]:
                result["status"] = "TARGET_MET"
            else:
                result["status"] = "PASS"
        else:
            result["status"] = "WARN" if result["report_only_until"] else "FAIL"
        results.append(result)
    return results


def render_release_profile_section(results: list[dict[str, Any]]) -> list[str]:
    """Format the release-profile evaluation as markdown lines."""
    if not results:
        return []
    lines = ["", "## Release profile gate", ""]
    header = [
        "profile",
        "status",
        "metric",
        "best_run",
        "best_value",
        "pass",
        "target",
        "evidence",
        "report_only_until",
    ]
    lines.append("| " + " | ".join(header) + " |")
    lines.append("| " + " | ".join(["---"] * len(header)) + " |")
    for r in results:
        evidence = str(r.get("no_data_reason") or "")
        if not evidence and r.get("best_run"):
            evidence = "clean provenance"
            if r.get("required_git_commit"):
                evidence += " @ " + str(r["required_git_commit"])[:12]
        lines.append(
            "| "
            + " | ".join(
                [
                    str(r["name"]),
                    str(r["status"]),
                    str(r["metric"]),
                    str(r.get("best_run") or ""),
                    _fmt_float(r.get("best_value")),
                    _fmt_float(r.get("pass")),
                    _fmt_float(r.get("target")),
                    evidence,
                    str(r.get("report_only_until") or ""),
                ]
            )
            + " |"
        )
    blocking_remediation = [
        r for r in results
        if r.get("status") in {"FAIL", "NO_DATA"}
        and not r.get("report_only_until")
        and r.get("remediation")
    ]
    if blocking_remediation:
        lines.extend(["", "### Blocking profile remediation", ""])
        for r in blocking_remediation:
            lines.append(f"- `{r['name']}`: {r['remediation']}")
    return lines


def main() -> int:
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent

    ap = argparse.ArgumentParser(description="Summarize lidarslam_ros2 benchmark runs (success/RTF/APE).")
    ap.add_argument(
        "--root",
        default=str(repo_root / "output"),
        help="Root output directory (default: ./output under the repo)",
    )
    ap.add_argument("--write-md", default="", help="Write markdown summary to this file")
    ap.add_argument("--write-csv", default="", help="Write CSV summary to this file")
    ap.add_argument(
        "--primary",
        default="ape",
        choices=["ape", "lid_rtf", "glim_rtf", "wall_sec"],
        help="Primary objective for ranking (default: ape)",
    )
    ap.add_argument(
        "--ape-threshold",
        type=float,
        default=None,
        help="Optional APE rmse threshold (m) for pass/fail flag",
    )
    ap.add_argument(
        "--fail-on-ape-threshold",
        action="store_true",
        help=(
            "Return a non-zero exit code when any run is missing APE or "
            "exceeds --ape-threshold"
        ),
    )
    ap.add_argument(
        "--ape-threshold-reference-kind",
        default="all",
        choices=["all", "ground_truth", "cross_validation", "unknown"],
        help=(
            "Limit threshold pass/fail checks to runs whose reference kind matches "
            "this value (default: all)"
        ),
    )
    ap.add_argument(
        "--release-profile",
        default="",
        help=(
            "Path to a release-profiles YAML (see scripts/release_profiles.yaml). "
            "Each profile evaluates one dataset/reference combination against its "
            "own pass/target thresholds and is reported as a separate section."
        ),
    )
    ap.add_argument(
        "--fail-on-profiles",
        action="store_true",
        help=(
            "Return a non-zero exit code when any blocking release profile is "
            "FAIL or has NO_DATA (report_only_until profiles never block)."
        ),
    )
    ap.add_argument(
        "--required-git-commit",
        default="",
        help=(
            "Require blocking release-profile evidence from this exact 40-character "
            "lowercase Git commit. Required with --fail-on-profiles; report-only "
            "profiles remain historical comparisons."
        ),
    )
    args = ap.parse_args()

    if args.required_git_commit and re.fullmatch(
        r"[0-9a-f]{40}", args.required_git_commit
    ) is None:
        print(
            "error: --required-git-commit must be exactly 40 lowercase "
            "hexadecimal characters"
        )
        return 1
    if args.required_git_commit and not args.release_profile:
        print("error: --required-git-commit requires --release-profile")
        return 1
    if args.fail_on_profiles and not args.required_git_commit:
        print("error: --fail-on-profiles requires --required-git-commit")
        return 1

    root = Path(args.root).expanduser().resolve()
    metrics_paths = sorted(root.rglob("metrics.json"))

    if not metrics_paths and not args.release_profile:
        print(f"no metrics.json found under: {root}")
        return 1

    runs: list[dict[str, Any]] = []
    for p in metrics_paths:
        try:
            runs.append(_load_json(p))
        except Exception as e:
            print(f"warn: failed to read {p}: {e}")

    if not runs and not args.release_profile:
        print("no readable metrics.json found")
        return 1

    lid_ok = 0
    glim_ok = 0
    lid_rtfs: list[float] = []
    glim_rtfs: list[float] = []
    ape_rmses: list[float] = []

    records: list[dict[str, Any]] = []
    for r in runs:
        out_dir = r.get("out_dir") or ""
        bag_path = r.get("bag_path") or ""
        bag_name = Path(bag_path).name if bag_path else ""
        dur = r.get("bag_duration_sec")

        frames = r.get("frames") or {}
        points_topic = r.get("points_topic") or ""
        points_frame = (frames.get("points_frame_id") or "") if isinstance(frames, dict) else ""

        lid = r.get("lidarslam") or {}
        lid_success = _as_bool(lid.get("success")) if isinstance(lid, dict) else None
        lid_wall = lid.get("wall_sec") if isinstance(lid, dict) else None
        lid_rtf = lid.get("rtf") if isinstance(lid, dict) else None

        glim = r.get("glim") or {}
        glim_avail = _as_bool(glim.get("available")) if isinstance(glim, dict) else None
        glim_success = _as_bool(glim.get("success")) if isinstance(glim, dict) else None
        glim_wall = glim.get("wall_sec") if isinstance(glim, dict) else None
        glim_rtf = glim.get("rtf") if isinstance(glim, dict) else None
        reference = r.get("reference") or {}
        reference_source = (
            (reference.get("source") or "")
            if isinstance(reference, dict) else ""
        )
        reference_kind = _infer_reference_kind(
            reference_source,
            (reference.get("kind") if isinstance(reference, dict) else ""),
        )
        glim_ref = (
            reference_source
            or ((glim.get("reference_source") or "") if isinstance(glim, dict) else "")
        )

        evo = r.get("evo") or {}
        ape = evo.get("ape") if isinstance(evo, dict) else None
        ape_rmse = (ape.get("rmse") if isinstance(ape, dict) else None) if ape is not None else None
        ape_pairs = (ape.get("pairs") if isinstance(ape, dict) else None) if ape is not None else None
        (
            provenance_complete,
            provenance_git_dirty,
            provenance_git_commit,
        ) = _provenance_state(r)
        provenance_status = (
            "clean"
            if provenance_complete and provenance_git_dirty is False
            else (
                "dirty"
                if provenance_complete and provenance_git_dirty is True
                else "incomplete"
            )
        )

        if lid_success is True:
            lid_ok += 1
        if glim_success is True:
            glim_ok += 1

        try:
            if lid_success is True and lid_rtf is not None:
                lid_rtfs.append(float(lid_rtf))
        except Exception:
            pass
        try:
            if glim_success is True and glim_rtf is not None:
                glim_rtfs.append(float(glim_rtf))
        except Exception:
            pass
        try:
            if lid_success is True and ape_rmse is not None:
                ape_rmses.append(float(ape_rmse))
        except Exception:
            pass

        ape_raw = _as_float(ape_rmse)
        lid_raw = _as_float(lid_rtf)
        glim_raw = _as_float(glim_rtf)
        lid_wall_raw = _as_float(lid_wall)

        if args.primary == "ape":
            primary_raw = ape_raw
            primary_missing = ape_raw is None
        elif args.primary == "lid_rtf":
            primary_raw = lid_raw
            primary_missing = lid_raw is None or lid_success is not True
        elif args.primary == "glim_rtf":
            primary_raw = glim_raw
            primary_missing = glim_raw is None or glim_success is not True
        elif args.primary == "wall_sec":
            primary_raw = lid_wall_raw
            primary_missing = lid_wall_raw is None or lid_success is not True
        else:
            primary_raw = ape_raw
            primary_missing = ape_raw is None

        ape_ok = ""
        if args.ape_threshold is not None and ape_raw is not None:
            ape_ok = "true" if ape_raw <= args.ape_threshold else "false"

        records.append(
            {
                "run": Path(out_dir).name if out_dir else "",
                "bag": bag_name,
                "bag_s": _fmt_float(dur, 1),
                "points_topic": points_topic,
                "points_frame": points_frame,
                "lid_ok": str(lid_success).lower() if lid_success is not None else "",
                "lid_rtf": _fmt_float(lid_rtf),
                "lid_wall_s": _fmt_float(lid_wall, 2),
                "glim_avail": "true" if glim_avail is True else ("false" if glim_avail is False else ""),
                "glim_ok": str(glim_success).lower() if glim_success is not None else "",
                "ape_ref_kind": reference_kind,
                "ape_ref_src": glim_ref,
                "glim_rtf": _fmt_float(glim_rtf),
                "glim_wall_s": _fmt_float(glim_wall, 2),
                "ape_rmse_m": _fmt_float(ape_raw),
                "ape_ok": ape_ok,
                "ape_pairs": ape_pairs,
                "provenance_complete": provenance_complete,
                "provenance_git_dirty": provenance_git_dirty,
                "provenance_git_commit": provenance_git_commit,
                "provenance_status": provenance_status,
                "primary_raw": primary_raw,
                "primary_missing": primary_missing,
            }
        )

    records_sorted = sorted(records, key=lambda rec: _primary_sort_key(args.primary, rec))
    for idx, rec in enumerate(records_sorted, start=1):
        rec["primary_rank"] = str(idx)

    total = len(records)
    lid_rate = (100.0 * lid_ok / total) if total else 0.0
    glim_rate = (100.0 * glim_ok / total) if total else 0.0

    lid_med = _median(lid_rtfs)
    glim_med = _median(glim_rtfs)
    ape_med = _median(ape_rmses)

    if args.primary == "ape" and records_sorted:
        best = records_sorted[0]
        best_note = (
            f"{best['run']} ({best.get('ape_rmse_m')}m)"
            if best and not bool(best.get("primary_missing"))
            else "N/A"
        )
    elif records_sorted:
        best_note = f"{records_sorted[0]['run']} ({_primary_value_text(args.primary, records_sorted[0])})"
    else:
        best_note = "N/A"

    pass_count = 0
    threshold_records = records
    if args.ape_threshold_reference_kind != "all":
        threshold_records = [
            rec for rec in records
            if rec.get("ape_ref_kind") == args.ape_threshold_reference_kind
        ]
    if args.ape_threshold is not None:
        for rec in threshold_records:
            v = _as_float(rec.get("ape_rmse_m"))
            if v is not None and v <= args.ape_threshold:
                pass_count += 1

    header = [
        "run",
        "primary_rank",
        "bag",
        "bag_s",
        "points_topic",
        "points_frame",
        "lid_ok",
        "lid_rtf",
        "lid_wall_s",
        "glim_avail",
        "glim_ok",
        "ape_ref_kind",
        "ape_ref_src",
        "glim_rtf",
        "glim_wall_s",
        "ape_rmse_m",
        "ape_ok",
        "provenance",
    ]

    md_lines: list[str] = []
    md_lines.append(f"# Benchmark summary (primary={args.primary})")
    md_lines.append("")
    md_lines.append(f"- runs: {total}")
    md_lines.append(f"- lidarslam success: {lid_ok}/{total} ({lid_rate:.1f}%)")
    md_lines.append(f"- lidarslam median RTF (success-only): {_fmt_float(lid_med)}")
    md_lines.append(f"- GLIM success: {glim_ok}/{total} ({glim_rate:.1f}%)")
    md_lines.append(f"- GLIM median RTF (success-only): {_fmt_float(glim_med)}")
    if args.primary == "ape":
        md_lines.append(f"- APE-primary best: {best_note}")
    else:
        md_lines.append(f"- primary_best: {best_note}")
    if ape_med is not None:
        md_lines.append(f"- APE RMSE median vs selected reference (m): {_fmt_float(ape_med)} (n={len(ape_rmses)})")
    else:
        md_lines.append("- APE RMSE: (not available; install evo to enable)")
    if args.ape_threshold is not None:
        md_lines.append(
            f"- APE threshold <= {args.ape_threshold}m "
            f"(reference_kind={args.ape_threshold_reference_kind}): "
            f"{pass_count}/{len(threshold_records)}"
        )

    md_lines.append("")
    md_lines.append("| " + " | ".join(header) + " |")
    md_lines.append("| " + " | ".join(["---"] * len(header)) + " |")
    for rec in records_sorted:
        row = [
            rec["run"],
            rec.get("primary_rank", ""),
            rec["bag"],
            rec["bag_s"],
            rec["points_topic"],
            rec["points_frame"],
            rec["lid_ok"],
            rec["lid_rtf"],
            rec["lid_wall_s"],
            rec["glim_avail"],
            rec["glim_ok"],
            rec["ape_ref_kind"],
            rec["ape_ref_src"],
            rec["glim_rtf"],
            rec["glim_wall_s"],
            rec["ape_rmse_m"],
            rec["ape_ok"],
            rec["provenance_status"],
        ]
        md_lines.append("| " + " | ".join(row) + " |")

    profile_results: list[dict[str, Any]] = []
    if args.release_profile:
        profile_path = Path(args.release_profile).expanduser().resolve()
        profiles = load_release_profiles(profile_path)
        profile_results = evaluate_release_profiles(
            profiles,
            records_sorted,
            required_git_commit=args.required_git_commit or None,
        )
        md_lines.extend(render_release_profile_section(profile_results))

    md = "\n".join(md_lines) + "\n"

    print(md, end="")

    if args.write_md:
        _write_text(Path(args.write_md).expanduser().resolve(), md)
    if args.write_csv:
        out_csv = Path(args.write_csv).expanduser().resolve()
        out_csv.parent.mkdir(parents=True, exist_ok=True)
        with out_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(header)
            for rec in records_sorted:
                w.writerow(
                    [
                        rec["run"],
                        rec.get("primary_rank", ""),
                        rec["bag"],
                        rec["bag_s"],
                        rec["points_topic"],
                        rec["points_frame"],
                        rec["lid_ok"],
                        rec["lid_rtf"],
                        rec["lid_wall_s"],
                        rec["glim_avail"],
                        rec["glim_ok"],
                        rec["ape_ref_kind"],
                        rec["ape_ref_src"],
                        rec["glim_rtf"],
                        rec["glim_wall_s"],
                        rec["ape_rmse_m"],
                        rec["ape_ok"],
                        rec["provenance_status"],
                    ]
                )

    if args.fail_on_ape_threshold:
        if args.ape_threshold is None:
            print("error: --fail-on-ape-threshold requires --ape-threshold")
            return 1

        if not threshold_records:
            print(
                "error: APE threshold gate has no eligible metrics.json "
                "evidence"
            )
            return 2

        failing_runs = [
            rec["run"]
            for rec in records_sorted
            if (
                args.ape_threshold_reference_kind == "all"
                or rec.get("ape_ref_kind") == args.ape_threshold_reference_kind
            )
            if rec.get("ape_ok") != "true"
        ]
        if failing_runs:
            print(
                "error: APE threshold failed for runs: "
                + ", ".join(failing_runs),
            )
            return 2

    if args.fail_on_profiles:
        if not args.release_profile:
            print("error: --fail-on-profiles requires --release-profile")
            return 1
        failing = [
            r for r in profile_results
            if (
                r["status"] == "FAIL"
                or (
                    r["status"] == "NO_DATA"
                    and not r.get("report_only_until")
                )
            )
        ]
        if failing:
            failures = ", ".join(
                f"{r['name']} ({r['status']})"
                for r in failing
            )
            print(f"error: release profile gate FAILED for: {failures}")
            for result in failing:
                if result.get("remediation"):
                    print(
                        f"hint: {result['name']}: "
                        f"{result['remediation']}"
                    )
            return 2

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
