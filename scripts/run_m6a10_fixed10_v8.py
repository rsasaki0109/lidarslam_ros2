#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Fixed10-v8 launcher with the materialization tree-identity contract.

v7 is an immutable failed attempt and is never edited or reused as an
execution record.  v8 reuses the exact v7 launch/preflight/completion
implementation from its pinned source file, but binds the input tree hash to
the materializer's ``relative_path_size_content_sha256_v1`` definition.  The
in-memory adapter changes no v7 file and refuses to load a different v7 SHA.
"""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import sys
from typing import Any, Sequence

from lidarslam_benchmark_tools import module_path, package_root

ROOT = package_root()
V7_PATH = module_path('run_m6a10_fixed10_v7')
V7_SHA256 = (
    '47ab6b059bc9736da5fa69933d7a1d1024db186b78b1c5d30b987544fb0d20d4')
CONTRACT_ID = 'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v8'
TREE_HASH_KIND = 'relative_path_size_content_sha256_v1'
MATERIALIZER_HELPER_PATH = (
    module_path('materialize_m6a10_synchronized_tail'))
MATERIALIZER_HELPER_SHA256 = (
    '2a2ff6476c6996a20b3e27d375ef33528ad08f20ea8b9bb539c8359075a5ee2b')
EXPECTED_INPUT_TREE_SHA256 = (
    '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263')
DEFAULT_INPUT_ROOT = Path(
    '/media/sasaki/aiueo1/datasets/ntu_viral_release/'
    'tnp_01_m6a10_v2a_sync_materialization_v1_ros2')


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


if _file_sha256(V7_PATH) != V7_SHA256:
    raise RuntimeError('pinned v7 launcher implementation has changed')


_SPEC = importlib.util.spec_from_file_location('m6a10_fixed10_v7_pinned', V7_PATH)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError('cannot load pinned v7 launcher implementation')
_IMPL = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _IMPL
_SPEC.loader.exec_module(_IMPL)


def canonical_tree_sha256(path: Path) -> str:
    """Return the materializer's path+size+content tree digest."""
    # Importing this existing helper keeps the v8 method identical to the
    # fixed10 materialization receipt rather than maintaining a second format.
    if _file_sha256(MATERIALIZER_HELPER_PATH) != MATERIALIZER_HELPER_SHA256:
        raise RuntimeError('pinned materializer tree helper has changed')
    from lidarslam_benchmark_tools.materialize_m6a10_synchronized_tail import sha256_tree
    return str(sha256_tree(path)['sha256'])


def tree_sha256(path: Path) -> str:
    """Compatibility name used by identity tests and the v8 contract."""
    return canonical_tree_sha256(path)


# Bind the imported implementation to the v8 identity without changing its
# source.  Its functions resolve these names through their own module globals.
_IMPL.__file__ = str(Path(__file__).resolve())
_IMPL.CONTRACT_ID = CONTRACT_ID
_IMPL.EXPECTED_INPUT_TREE_SHA256 = EXPECTED_INPUT_TREE_SHA256
_IMPL.DEFAULT_INPUT_ROOT = DEFAULT_INPUT_ROOT
_IMPL.tree_sha256 = tree_sha256

LaunchError = _IMPL.LaunchError
IMAGE_TAG = _IMPL.IMAGE_TAG
IMAGE_DIGEST = _IMPL.IMAGE_DIGEST


class LaunchConfig:
    """v8 configuration with the corrected 64-hex input default."""

    def __init__(
            self, root: Path, repo_root: Path, input_root: Path = DEFAULT_INPUT_ROOT,
            image_tag: str = IMAGE_TAG, image_digest: str = IMAGE_DIGEST,
            quiescence_script: Path = Path('scripts/check_m6a10_quiescence.py'),
            sample_seconds: float = 5.0, max_busy_percent: float = 5.0,
            max_load_per_cpu: float = 0.5,
            expected_input_tree_sha256: str | None = EXPECTED_INPUT_TREE_SHA256) -> None:
        self.root = root
        self.repo_root = repo_root
        self.input_root = input_root
        self.image_tag = image_tag
        self.image_digest = image_digest
        self.quiescence_script = quiescence_script
        self.sample_seconds = sample_seconds
        self.max_busy_percent = max_busy_percent
        self.max_load_per_cpu = max_load_per_cpu
        self.expected_input_tree_sha256 = expected_input_tree_sha256


def run_v8(*args: Any, **kwargs: Any) -> int:
    """Run the pinned v7 gates under the v8 contract identity."""
    return int(_IMPL.run_v7(*args, **kwargs))


def main(argv: Sequence[str] | None = None) -> int:
    """Use the pinned launcher's fixed, non-shell CLI."""
    args = _IMPL._parser().parse_args(argv)
    config = LaunchConfig(
        root=args.root, repo_root=args.repo_root, input_root=args.input_root,
        quiescence_script=args.quiescence_script,
        sample_seconds=args.sample_seconds,
        max_busy_percent=args.max_busy_percent,
        max_load_per_cpu=args.max_load_per_cpu)
    return run_v8(config)


if __name__ == '__main__':
    sys.exit(main())
