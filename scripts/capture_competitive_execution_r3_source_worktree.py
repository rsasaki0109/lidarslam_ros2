#!/usr/bin/env python3
# flake8: noqa
"""Canonical checkout entry point for the r3 source-worktree snapshot."""

from pathlib import Path
import sys

_SOURCE_ROOT = Path(__file__).resolve().parents[1]
if str(_SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(_SOURCE_ROOT))

from scripts import capture_competitive_execution_source_worktree as _impl
from scripts.capture_competitive_execution_source_worktree import *  # noqa: F401,F403
from scripts.capture_competitive_execution_source_worktree import _main

# Keep the canonical entry point patchable for fixture tests while the
# implementation remains shared with the compatibility module.  The wrapper
# deliberately forwards only pure capture/validation calls; no command can be
# supplied by a caller beyond the module's fixed Git probe runner.
_candidate_contract = _impl._candidate_contract


def capture_snapshot(*, root=ROOT, output, runner=_impl._default_runner,
                     captured_at=None):
    previous = _impl._candidate_contract
    _impl._candidate_contract = _candidate_contract
    try:
        return _impl.capture_snapshot(
            root=root, output=output, runner=runner, captured_at=captured_at)
    finally:
        _impl._candidate_contract = previous


def validate_snapshot(path, *, root=ROOT, runner=_impl._default_runner):
    previous = _impl._candidate_contract
    _impl._candidate_contract = _candidate_contract
    try:
        return _impl.validate_snapshot(path, root=root, runner=runner)
    finally:
        _impl._candidate_contract = previous


if __name__ == '__main__':
    raise SystemExit(_main())
