#!/usr/bin/env python3
"""Classify a product version as prerelease or stable publication."""

from __future__ import annotations

import argparse
import re


SEMVER = re.compile(r'^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$')


def release_channel(version: str) -> str:
    """Return the GitHub release channel for one strict semantic version."""
    match = SEMVER.fullmatch(version)
    if match is None:
        raise ValueError(
            f'version must be canonical MAJOR.MINOR.PATCH: {version!r}')
    major, minor, _ = (int(part) for part in match.groups())
    return 'prerelease' if major == 0 and minor < 9 else 'stable'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('version')
    args = parser.parse_args()
    try:
        print(release_channel(args.version))
    except ValueError as exc:
        parser.error(str(exc))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
