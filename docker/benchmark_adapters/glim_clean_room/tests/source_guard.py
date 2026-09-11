#!/usr/bin/env python3
"""Static clean-room guard for the production adapter surface."""

from __future__ import annotations

import pathlib
import re
import sys


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: source_guard.py <adapter-root>")
    root = pathlib.Path(sys.argv[1]).resolve()
    production = [root / "CMakeLists.txt"]
    production += list((root / "include").rglob("*"))
    production += list((root / "src").rglob("*"))
    # Phase 1 is an additive recipe surface.  Keep its downloader/build
    # instructions under the same contamination guard as the C++ boundary.
    phase1 = root / "phase1"
    if phase1.is_dir():
        production += list(phase1.rglob("*"))
    phase2 = root / "phase2"
    if phase2.is_dir():
        production += list(phase2.rglob("*"))
    phase3a = root / "phase3a"
    if phase3a.is_dir():
        production += list(phase3a.rglob("*"))
    phase3b = root / "phase3b"
    if phase3b.is_dir():
        production += list(phase3b.rglob("*"))
    phase3d = root / "phase3d"
    if phase3d.is_dir():
        production += list(phase3d.rglob("*"))
    forbidden = re.compile(r"glim[_-]?ros|glim_ros2", re.IGNORECASE)

    def check_text(label: str, text: str) -> None:
        if forbidden.search(text):
            raise SystemExit(f"forbidden upstream bridge token in: {label}")
        if re.search(
            r"(?:find_package|target_link_libraries|add_subdirectory).*ros",
            text,
            re.IGNORECASE,
        ):
            # ROS 2 message conversion is deliberately header-only and opt-in;
            # the production CMake surface must not discover an upstream bridge.
            if "sensor_msgs" not in text:
                raise SystemExit(f"unexpected ROS dependency discovery in: {label}")

    # Keep the guard's forbidden-token matcher adversarially tested without
    # putting a forbidden bridge token in the production tree it scans.
    synthetic = "glim_" + "ros2/include"
    if not forbidden.search(synthetic):
        raise AssertionError("synthetic forbidden bridge fixture was not detected")

    checked = 0
    for path in production:
        if (not path.is_file() or path.suffix == ".pyc" or
                "__pycache__" in path.parts):
            continue
        checked += 1
        text = path.read_text(encoding="utf-8")
        check_text(str(path), text)
    print(f"clean-room source guard: {checked} production files checked")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
