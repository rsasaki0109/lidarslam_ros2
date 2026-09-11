#!/usr/bin/env python3
"""Validate and hash an extracted Boost source tree deterministically."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
import sys
from typing import Dict, List, Optional, Tuple


HASH_KIND = "relative_path_content_sha256_v1_excluding_git_metadata"
HEX64 = set("0123456789abcdef")


def _tree_hash(root: Path) -> Tuple[str, int]:
    root = Path(root)
    if root.is_symlink() or not root.is_dir():
        raise ValueError("source tree is not a real directory: " + str(root))
    root = root.resolve()
    entries: List[Tuple[str, Path]] = []
    for directory, directory_names, file_names in os.walk(root, followlinks=False):
        directory_path = Path(directory)
        directory_names[:] = sorted(
            name for name in directory_names if name != ".git")
        for name in directory_names:
            path = directory_path / name
            if path.is_symlink() or not path.is_dir():
                raise ValueError("invalid source-tree directory: " + str(path))
        for name in sorted(file_names):
            path = directory_path / name
            metadata = path.lstat()
            if (stat.S_ISLNK(metadata.st_mode) or
                    not stat.S_ISREG(metadata.st_mode) or
                    metadata.st_nlink != 1):
                raise ValueError("invalid source-tree file: " + str(path))
            entries.append((path.relative_to(root).as_posix(), path))
    entries.sort(key=lambda item: item[0])
    digest = hashlib.sha256()
    for relative, path in entries:
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest(), len(entries)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    parser.add_argument("--expected", required=True,
                        help="expected canonical tree SHA-256")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if (len(args.expected) != 64 or
            any(character not in HEX64 for character in args.expected)):
        parser.error("--expected must be a lower-case SHA-256")
    try:
        observed, file_count = _tree_hash(args.root)
        result: Dict[str, object] = {
            "status": "PASS" if observed == args.expected else "FAIL",
            "root": str(args.root.resolve()),
            "hash_kind": HASH_KIND,
            "file_count": file_count,
            "tree_sha256": observed,
            "expected_tree_sha256": args.expected,
        }
        encoded = (json.dumps(result, indent=2, sort_keys=True) + "\n")
        if args.output:
            if args.output.exists():
                raise ValueError("refusing to overwrite hash receipt: " + str(args.output))
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(encoded, encoding="utf-8")
        print(encoded, end="")
        return 0 if observed == args.expected else 1
    except (OSError, ValueError) as error:
        print("Boost tree hash: FAIL_CLOSED: " + str(error), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
