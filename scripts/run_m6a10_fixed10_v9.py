#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Fixed10-v9 launcher with corrected image and materialization identities.

The v7 and v8 launchers and their attempt roots are immutable.  v9 loads the
pinned v7 implementation in memory, binds the complete local image ID and the
materializer tree hash, and exposes a public configuration whose defaults are
explicit rather than inherited from the v7 dataclass.  The identity-capture
helper performs only read-only source/input/image inspection; it never runs
quiescence, Docker, replay, GT access, or scoring.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from typing import Any, Callable, Sequence

from lidarslam_benchmark_tools import module_path, package_root

ROOT = package_root()
V7_PATH = module_path('run_m6a10_fixed10_v7')
V7_SHA256 = (
    '47ab6b059bc9736da5fa69933d7a1d1024db186b78b1c5d30b987544fb0d20d4')
CONTRACT_ID = 'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v9'
LAUNCHER_CONTRACT_ID = 'm6a10-v2a-fixed10-v9-single-process-launcher-v1'
TREE_HASH_KIND = 'relative_path_size_content_sha256_v1'
MATERIALIZER_HELPER_PATH = (
    module_path('materialize_m6a10_synchronized_tail'))
MATERIALIZER_HELPER_SHA256 = (
    'caddcf0ae85d74444ae65ea85ed33d5e561a2569dc8ef180b2496a9d87c132c9')
EXPECTED_INPUT_TREE_SHA256 = (
    '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263')
IMAGE_TAG = 'm6a10-v2a-fixed10-v2-lidarslam-ours:jazzy'
# This is the complete 64-hex Docker image ID.  Do not shorten it or inherit
# the malformed v7 value.
IMAGE_DIGEST = (
    'sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69')
DEFAULT_INPUT_ROOT = Path(
    '/media/sasaki/aiueo1/datasets/ntu_viral_release/'
    'tnp_01_m6a10_v2a_sync_materialization_v1_ros2')
V9_IDENTITY_RECEIPT_PATH = Path(
    '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/'
    'ours_m6a10_v2a_unpaced_ack_fixed10_v9_identity_preflight_v2/'
    'identity_receipt.json')


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


if file_sha256(V7_PATH) != V7_SHA256:
    raise RuntimeError('pinned v7 launcher implementation has changed')


_SPEC = importlib.util.spec_from_file_location(
    'm6a10_fixed10_v7_pinned_for_v9', V7_PATH)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError('cannot load pinned v7 launcher implementation')
_IMPL = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _IMPL
_SPEC.loader.exec_module(_IMPL)


def canonical_tree_sha256(path: Path) -> str:
    """Use the materializer's path+size+content tree identity exactly."""
    if file_sha256(MATERIALIZER_HELPER_PATH) != MATERIALIZER_HELPER_SHA256:
        raise RuntimeError('pinned materializer tree helper has changed')
    from lidarslam_benchmark_tools.materialize_m6a10_synchronized_tail import sha256_tree
    return str(sha256_tree(path)['sha256'])


def tree_sha256(path: Path) -> str:
    """Compatibility name used by the pinned v7 implementation."""
    return canonical_tree_sha256(path)


# The adapter changes only the imported module's globals.  No v7 file is
# edited, and every v9 path below is bound to the complete image ID.
_IMPL.__file__ = str(Path(__file__).resolve())
_IMPL.CONTRACT_ID = CONTRACT_ID
_IMPL.IMAGE_TAG = IMAGE_TAG
_IMPL.IMAGE_DIGEST = IMAGE_DIGEST
_IMPL.EXPECTED_INPUT_TREE_SHA256 = EXPECTED_INPUT_TREE_SHA256
_IMPL.DEFAULT_INPUT_ROOT = DEFAULT_INPUT_ROOT
_IMPL.tree_sha256 = tree_sha256

LaunchError = _IMPL.LaunchError


class LaunchConfig:
    """Public v9 configuration with explicit, validated identity defaults."""

    def __init__(
            self, root: Path, repo_root: Path,
            input_root: Path = DEFAULT_INPUT_ROOT,
            image_tag: str = IMAGE_TAG,
            image_digest: str = IMAGE_DIGEST,
            quiescence_script: Path = Path('scripts/check_m6a10_quiescence.py'),
            sample_seconds: float = 5.0,
            max_busy_percent: float = 5.0,
            max_load_per_cpu: float = 0.5,
            expected_input_tree_sha256: str | None = EXPECTED_INPUT_TREE_SHA256,
    ) -> None:
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


def validate_preflight_identity(
        config: LaunchConfig, *,
        image_probe: Callable[[LaunchConfig], dict[str, Any]] | None = None,
        expected_input_tree_sha256: str | None = None,
) -> dict[str, Any]:
    """Run the same source/input/image identity gate used before launch.

    The explicit config check prevents a malformed v7 digest from being
    accepted by a test double or a future adapter.  The imported validator
    then performs the complete read-only source/tree/image inspection.
    """
    if config.image_tag != IMAGE_TAG or config.image_digest != IMAGE_DIGEST:
        raise LaunchError(
            'IMAGE_IDENTITY_MISMATCH',
            'v9 requires the complete pinned local image ID')
    identity = _IMPL.validate_preflight_identity(
        config, image_probe=image_probe,
        expected_input_tree_sha256=expected_input_tree_sha256)
    observed_id = (identity.get('image') or {}).get('id')
    if observed_id != IMAGE_DIGEST:
        raise LaunchError(
            'IMAGE_IDENTITY_MISMATCH',
            'observed image ID differs from the v9 complete image ID')
    return identity


def _json_bytes(value: dict[str, Any]) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True) + '\n').encode('utf-8')


def _atomic_json(path: Path, value: dict[str, Any]) -> str:
    if path.exists() or path.is_symlink():
        raise LaunchError('RECEIPT_OVERWRITE', f'refusing to overwrite {path}')
    part = path.with_name(path.name + '.part')
    if part.exists() or part.is_symlink():
        raise LaunchError('RECEIPT_OVERWRITE', f'stale receipt part exists: {part}')
    payload = _json_bytes(value)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd = None
    try:
        fd = part.open('xb')
        fd.write(payload)
        fd.flush()
        import os
        os.fsync(fd.fileno())
        fd.close()
        fd = None
        part.replace(path)
    except Exception:
        if fd is not None:
            fd.close()
        raise
    return hashlib.sha256(payload).hexdigest()


def capture_identity_receipt(
        config: LaunchConfig, output: Path,
        *, image_probe: Callable[[LaunchConfig], dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Capture a PASS identity receipt without creating an execution root."""
    identity = validate_preflight_identity(config, image_probe=image_probe)
    receipt = {
        'schema_version': 1,
        'receipt_kind': 'm6a10_fixed10_v9_identity_preflight',
        'contract_id': CONTRACT_ID,
        'launcher_contract_id': LAUNCHER_CONTRACT_ID,
        'status': 'PASS',
        'launcher': {
            'path': 'scripts/run_m6a10_fixed10_v9.py',
            'sha256': file_sha256(Path(__file__).resolve()),
            'pinned_v7_sha256': V7_SHA256,
            'tree_hash_kind': TREE_HASH_KIND,
            'tree_hash_helper_path': 'scripts/materialize_m6a10_synchronized_tail.py',
            'tree_hash_helper_sha256': MATERIALIZER_HELPER_SHA256,
        },
        'identity_definition': {
            'image_tag': IMAGE_TAG,
            'image_digest': IMAGE_DIGEST,
            'input_tree_sha256': EXPECTED_INPUT_TREE_SHA256,
            'input_tree_hash_kind': TREE_HASH_KIND,
        },
        'identity': identity,
        'execution': {
            'runner_start_attempted': False,
            'quiescence_started': False,
            'docker_run_started': False,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
        },
    }
    receipt_sha256 = _atomic_json(output, receipt)
    return {'path': str(output), 'sha256': receipt_sha256, 'receipt': receipt}


def run_v9(*args: Any, **kwargs: Any) -> int:
    """Run the pinned v7 gates under the corrected v9 identity."""
    return int(_IMPL.run_v7(*args, **kwargs))


def build_runner_argv(config: LaunchConfig, output_dir: Path) -> list[str]:
    """Expose the fixed argv builder after binding the complete image ID."""
    return list(_IMPL.build_runner_argv(config, output_dir))


def main(argv: Sequence[str] | None = None) -> int:
    """Use the pinned launcher's fixed, non-shell CLI."""
    args = _IMPL._parser().parse_args(argv)
    config = LaunchConfig(
        root=args.root, repo_root=args.repo_root, input_root=args.input_root,
        quiescence_script=args.quiescence_script,
        sample_seconds=args.sample_seconds,
        max_busy_percent=args.max_busy_percent,
        max_load_per_cpu=args.max_load_per_cpu)
    return run_v9(config)


if __name__ == '__main__':
    sys.exit(main())
