#!/usr/bin/env python3
# Copyright 2026 Sasaki
# All rights reserved.

"""Fixed10-v10 launcher with an independently bound identity contract.

The v9 launcher is an immutable source checkpoint.  v10 verifies that exact
source SHA, then uses the pinned implementation underneath it while defining
its own contract, image/tree constants, LaunchConfig, identity validator, and
receipt writer.  No v9 runtime configuration object or mutable default is
inherited.  Identity capture is read-only and never starts quiescence, Docker,
replay, GT access, or scoring.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any, Callable, Sequence

from lidarslam_benchmark_tools import module_path, package_root

ROOT = package_root()
V9_PATH = module_path('run_m6a10_fixed10_v9')
V9_SHA256 = (
    '45704d041e260942136bcaffa31f9d385ad3ebb3bbfd21c0482d7715ea86a10b')
CONTRACT_ID = 'm6a10-v2a-ours-rko-unpaced-ack-fixed10-v10'
LAUNCHER_CONTRACT_ID = 'm6a10-v2a-fixed10-v10-single-process-launcher-v1'
TREE_HASH_KIND = 'relative_path_size_content_sha256_v1'
MATERIALIZER_HELPER_PATH = (
    module_path('materialize_m6a10_synchronized_tail'))
MATERIALIZER_HELPER_SHA256 = (
    '2a2ff6476c6996a20b3e27d375ef33528ad08f20ea8b9bb539c8359075a5ee2b')
EXPECTED_INPUT_TREE_SHA256 = (
    '0a45497ab4ed94bf8e9757bab3f37e5786fee4991beea16c1efdc49e38cb9263')
IMAGE_TAG = 'm6a10-v2a-fixed10-v2-lidarslam-ours:jazzy'
IMAGE_DIGEST = (
    'sha256:385b6eeedae3014bcd893849f2ec3a49f5176f0ef3cdd7e96559690e8dc25a69')
DEFAULT_INPUT_ROOT = Path(
    '/media/sasaki/aiueo1/datasets/ntu_viral_release/'
    'tnp_01_m6a10_v2a_sync_materialization_v1_ros2')
V10_IDENTITY_RECEIPT_PATH = Path(
    '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260822/'
    'ours_m6a10_v2a_unpaced_ack_fixed10_v10_identity_preflight/'
    'identity_receipt.json')


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def verify_pinned_v9() -> str:
    """Reject any v9 source drift before loading its implementation."""
    observed = file_sha256(V9_PATH)
    if observed != V9_SHA256:
        raise RuntimeError('pinned v9 launcher implementation has changed')
    return observed


verify_pinned_v9()
_SPEC = importlib.util.spec_from_file_location(
    'm6a10_fixed10_v9_pinned_for_v10', V9_PATH)
if _SPEC is None or _SPEC.loader is None:
    raise RuntimeError('cannot load pinned v9 launcher implementation')
_V9 = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = _V9
_SPEC.loader.exec_module(_V9)

# Use only the implementation object underneath the verified v9 source.  v10
# owns all public identity/configuration names below and does not use
# _V9.LaunchConfig, _V9.IMAGE_DIGEST, or _V9.run_v9.
_IMPL = _V9._IMPL
_IMPL.__file__ = str(Path(__file__).resolve())
_IMPL.CONTRACT_ID = CONTRACT_ID
_IMPL.IMAGE_TAG = IMAGE_TAG
_IMPL.IMAGE_DIGEST = IMAGE_DIGEST
_IMPL.EXPECTED_INPUT_TREE_SHA256 = EXPECTED_INPUT_TREE_SHA256
_IMPL.DEFAULT_INPUT_ROOT = DEFAULT_INPUT_ROOT


def canonical_tree_sha256(path: Path) -> str:
    """Use the materializer's path+size+content identity exactly."""
    if file_sha256(MATERIALIZER_HELPER_PATH) != MATERIALIZER_HELPER_SHA256:
        raise RuntimeError('pinned materializer tree helper has changed')
    from lidarslam_benchmark_tools.materialize_m6a10_synchronized_tail import sha256_tree
    return str(sha256_tree(path)['sha256'])


def tree_sha256(path: Path) -> str:
    return canonical_tree_sha256(path)


_IMPL.tree_sha256 = tree_sha256
LaunchError = _IMPL.LaunchError


class LaunchConfig:
    """Independent v10 configuration; no v9 dataclass/default inheritance."""

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
    """Run v10's complete source/input/image identity gate read-only."""
    if config.image_tag != IMAGE_TAG or config.image_digest != IMAGE_DIGEST:
        raise LaunchError(
            'IMAGE_IDENTITY_MISMATCH',
            'v10 requires the complete pinned local image ID')
    identity = _IMPL.validate_preflight_identity(
        config, image_probe=image_probe,
        expected_input_tree_sha256=expected_input_tree_sha256)
    observed_id = (identity.get('image') or {}).get('id')
    if observed_id != IMAGE_DIGEST:
        raise LaunchError(
            'IMAGE_IDENTITY_MISMATCH',
            'observed image ID differs from the v10 complete image ID')
    return identity


def build_runner_argv(config: LaunchConfig, output_dir: Path) -> list[str]:
    """Expose the fixed argv after v10 identity binding."""
    return list(_IMPL.build_runner_argv(config, output_dir))


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
    """Capture PASS identity evidence without creating the execution root."""
    identity = validate_preflight_identity(config, image_probe=image_probe)
    receipt = {
        'schema_version': 1,
        'receipt_kind': 'm6a10_fixed10_v10_identity_preflight',
        'contract_id': CONTRACT_ID,
        'launcher_contract_id': LAUNCHER_CONTRACT_ID,
        'status': 'PASS',
        'launcher': {
            'path': 'scripts/run_m6a10_fixed10_v10.py',
            'sha256': file_sha256(Path(__file__).resolve()),
            'pinned_v9_sha256': V9_SHA256,
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


def run_v10(*args: Any, **kwargs: Any) -> int:
    """Run the pinned implementation under the independent v10 contract."""
    return int(_IMPL.run_v7(*args, **kwargs))


def main(argv: Sequence[str] | None = None) -> int:
    """Use the pinned launcher parser with an independent v10 config."""
    args = _IMPL._parser().parse_args(argv)
    config = LaunchConfig(
        root=args.root, repo_root=args.repo_root, input_root=args.input_root,
        quiescence_script=args.quiescence_script,
        sample_seconds=args.sample_seconds,
        max_busy_percent=args.max_busy_percent,
        max_load_per_cpu=args.max_load_per_cpu)
    return run_v10(config)


if __name__ == '__main__':
    sys.exit(main())
