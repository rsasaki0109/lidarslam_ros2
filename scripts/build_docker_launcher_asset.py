#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Build a deterministic, version-pinned standalone Docker launcher."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
from typing import Sequence


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / 'scripts' / 'docker_map_bag.sh'
VERSION_MARKER = 'LIDARSLAM_DOCKER_LAUNCHER_VERSION="development"'
REVISION_MARKER = 'LIDARSLAM_DOCKER_LAUNCHER_REVISION="working-tree"'
TAG_PATTERN = re.compile(r'^v[0-9]+\.[0-9]+\.[0-9]+$')
REVISION_PATTERN = re.compile(r'^[0-9a-f]{40}$')


class LauncherBuildError(ValueError):
    """The standalone launcher cannot be built safely."""


def render_launcher(source: Path, *, tag: str, revision: str) -> bytes:
    """Return a syntax-checked launcher bound to one release identity."""
    if TAG_PATTERN.fullmatch(tag) is None:
        raise LauncherBuildError(
            f'tag must match v<major>.<minor>.<patch>: {tag!r}')
    if REVISION_PATTERN.fullmatch(revision) is None:
        raise LauncherBuildError(
            'source revision must be a lowercase 40-character Git SHA')
    if source.is_symlink() or not source.is_file():
        raise LauncherBuildError(
            f'launcher source must be a regular non-symlink file: {source}')
    try:
        text = source.read_text(encoding='utf-8')
    except (OSError, UnicodeDecodeError) as exc:
        raise LauncherBuildError(f'cannot read launcher source: {exc}') from exc
    if not text.startswith('#!/usr/bin/env bash\n'):
        raise LauncherBuildError('launcher source has an unexpected shebang')
    if text.count(VERSION_MARKER) != 1:
        raise LauncherBuildError('launcher version marker is missing or duplicated')
    if text.count(REVISION_MARKER) != 1:
        raise LauncherBuildError(
            'launcher revision marker is missing or duplicated')
    text = text.replace(
        VERSION_MARKER,
        f'LIDARSLAM_DOCKER_LAUNCHER_VERSION="{tag}"',
    ).replace(
        REVISION_MARKER,
        f'LIDARSLAM_DOCKER_LAUNCHER_REVISION="{revision}"',
    )
    syntax = subprocess.run(
        ['/bin/bash', '-n'],
        input=text,
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    if syntax.returncode != 0:
        detail = syntax.stderr.strip() or syntax.stdout.strip()
        raise LauncherBuildError(
            f'rendered launcher failed bash syntax validation: {detail}')
    return text.encode('utf-8')


def write_create_only(output: Path, payload: bytes) -> None:
    """Create one executable output without replacing an existing path."""
    if output.is_symlink() or output.exists():
        raise LauncherBuildError(f'output already exists: {output}')
    if not output.parent.is_dir():
        raise LauncherBuildError(
            f'output parent directory does not exist: {output.parent}')
    descriptor: int | None = None
    try:
        descriptor = os.open(
            output,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL,
            0o755,
        )
        with os.fdopen(descriptor, 'wb') as stream:
            descriptor = None
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        output.chmod(
            stat.S_IRUSR | stat.S_IWUSR | stat.S_IXUSR
            | stat.S_IRGRP | stat.S_IXGRP
            | stat.S_IROTH | stat.S_IXOTH
        )
    except OSError as exc:
        if descriptor is not None:
            os.close(descriptor)
        raise LauncherBuildError(f'cannot create launcher output: {exc}') from exc


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--source-revision', required=True)
    parser.add_argument('--source', type=Path, default=DEFAULT_SOURCE)
    parser.add_argument('--output', type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Build the requested launcher and print its immutable identity."""
    args = _parser().parse_args(argv)
    try:
        payload = render_launcher(
            args.source,
            tag=args.tag,
            revision=args.source_revision,
        )
        write_create_only(args.output, payload)
    except LauncherBuildError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    print(f'Launcher: {args.output}')
    print(f'Size: {len(payload)} bytes')
    print(f'SHA-256: {hashlib.sha256(payload).hexdigest()}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
