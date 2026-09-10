#!/usr/bin/env python3
"""Acquire one fresh NTU VIRAL candidate under the preregistered mount.

The command is intentionally a thin production entry point.  It does not
resolve moving URLs, edit a profile, or reuse a previous candidate directory.
Tests should call :func:`lidarslam_benchmark_tools.ntu_viral_acquisition.acquire_candidate` with
an injected mount observation and fixture transport instead of using this
networking entry point.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from lidarslam_benchmark_tools.ntu_viral_acquisition import (
    NtuAcquisitionError,
    ROOT,
    acquire_candidate,
    load_context,
)


def _read_observation(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise NtuAcquisitionError("MOUNT_INSPECTION", "mount observation fixture must be a JSON object")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--candidate-root", type=Path)
    parser.add_argument("--mount-observation-json", type=Path,
                        help="synthetic-only mount observation; production uses findmnt")
    args = parser.parse_args(argv)
    try:
        context = load_context(args.selection, args.profile, root=ROOT)
        candidate = args.candidate_root
        if candidate is None:
            relative = context.selection["acquisition"]["candidate_root_relative"]
            if not isinstance(relative, str) or Path(relative).is_absolute() or ".." in Path(relative).parts:
                raise NtuAcquisitionError("ROOT_SCOPE", "selection candidate_root_relative is unsafe")
            candidate = args.evidence_root / relative
        result = acquire_candidate(
            args.selection, args.profile, args.evidence_root, candidate,
            mount_observation=_read_observation(args.mount_observation_json),
            root=ROOT,
        )
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result.get("status") == "PASS" else 2
    except (NtuAcquisitionError, OSError, UnicodeError, json.JSONDecodeError) as exc:
        print(json.dumps({"status": "NOT_ACQUIRED", "error": str(exc)}, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
