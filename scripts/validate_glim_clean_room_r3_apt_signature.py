#!/usr/bin/env python3
"""Validate the non-executable r3 apt signature plan/fixture contract."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_release_signature.py"
PLANNER = ROOT / "scripts/plan_glim_clean_room_r3_apt_signature.py"


def _module():
    spec = importlib.util.spec_from_file_location("r3_apt_release_signature_validator", MODULE)
    if spec is None or spec.loader is None:
        raise ValueError("signature module cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _planner():
    spec = importlib.util.spec_from_file_location("r3_apt_signature_planner", PLANNER)
    if spec is None or spec.loader is None:
        raise ValueError("signature planner cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate_plan(path: Path) -> dict[str, object]:
    module = _module()
    data = json.loads(module._regular(Path(path), "signature plan", limit=module.MAX_JSON_BYTES))
    return _planner().validate_plan_static(data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--help-contract", action="store_true")
    args = parser.parse_args()
    if args.help_contract:
        print(__doc__)
        return 0
    if args.plan is None:
        parser.error("--plan is required; runtime signature execution is not implemented")
    try:
        print(json.dumps(validate_plan(args.plan), sort_keys=True))
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
