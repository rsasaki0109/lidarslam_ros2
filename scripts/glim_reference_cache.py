#!/usr/bin/env python3
"""Create and verify content-bound local GLIM trajectory cache entries."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import sys
import tempfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable, Sequence

from product_schema import load_json_object, validate_contract


SCHEMA_NAME = 'glim-reference-cache-v1.schema.json'
SCHEMA_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/glim-reference-cache-v1.schema.json'
)
MAX_TRAJECTORY_BYTES = 512 * 1024 * 1024
COPY_CHUNK_BYTES = 1024 * 1024


class GlimReferenceCacheError(ValueError):
    """Raised when a cache identity or entry cannot be trusted."""


def _canonical_json(payload: Any) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=True,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('utf-8')


def _sha256_file(path: Path) -> tuple[str, int]:
    digest = hashlib.sha256()
    total = 0
    with path.open('rb') as stream:
        while True:
            chunk = stream.read(COPY_CHUNK_BYTES)
            if not chunk:
                break
            digest.update(chunk)
            total += len(chunk)
    return digest.hexdigest(), total


def _safe_tree_files(root: Path, label: str) -> list[Path]:
    if root.is_symlink():
        raise GlimReferenceCacheError(f'{label} root must not be a symlink')
    if not root.is_dir():
        raise GlimReferenceCacheError(f'{label} directory is missing: {root}')
    files: list[Path] = []
    for path in sorted(root.rglob('*')):
        if path.is_symlink():
            raise GlimReferenceCacheError(
                f'{label} must not contain symlinks: '
                f'{path.relative_to(root).as_posix()}'
            )
        if path.is_dir():
            continue
        if not path.is_file():
            raise GlimReferenceCacheError(
                f'{label} contains a non-regular entry: '
                f'{path.relative_to(root).as_posix()}'
            )
        files.append(path)
    if not files:
        raise GlimReferenceCacheError(f'{label} contains no regular files')
    return files


def _tree_identity(root: Path, label: str) -> dict[str, Any]:
    if root.is_symlink():
        raise GlimReferenceCacheError(f'{label} root must not be a symlink')
    root = root.resolve()
    files = _safe_tree_files(root, label)
    digest = hashlib.sha256()
    total_bytes = 0
    for path in files:
        relative = path.relative_to(root).as_posix().encode('utf-8')
        file_sha256, size = _sha256_file(path)
        total_bytes += size
        digest.update(b'file\0')
        digest.update(str(len(relative)).encode('ascii'))
        digest.update(b'\0')
        digest.update(relative)
        digest.update(b'\0')
        digest.update(str(size).encode('ascii'))
        digest.update(b'\0')
        digest.update(file_sha256.encode('ascii'))
        digest.update(b'\0')
    return {
        'sha256': digest.hexdigest(),
        'file_count': len(files),
        'total_bytes': total_bytes,
    }


def _runtime_artifact(path: Path) -> dict[str, Any]:
    resolved = path.resolve(strict=True)
    if resolved.is_dir():
        identity = _tree_identity(resolved, 'GLIM runtime artifact')
        return {'name': resolved.name, **identity}
    if not resolved.is_file():
        raise GlimReferenceCacheError(
            f'GLIM runtime artifact is not a regular file: {path}'
        )
    sha256, size = _sha256_file(resolved)
    return {
        'name': resolved.name,
        'sha256': sha256,
        'file_count': 1,
        'total_bytes': size,
    }


def _runtime_identity(
    kind: str,
    paths: Iterable[Path],
    tokens: Iterable[str],
) -> dict[str, Any]:
    token_values = sorted(set(tokens))
    if any(
        not token or '\n' in token or '\r' in token
        for token in token_values
    ):
        raise GlimReferenceCacheError(
            'GLIM runtime tokens must be non-empty single-line values'
        )
    artifacts = sorted(
        (_runtime_artifact(path) for path in paths),
        key=lambda item: (
            item['name'], item['sha256'], item['file_count'],
            item['total_bytes'],
        ),
    )
    if not artifacts and not token_values:
        raise GlimReferenceCacheError(
            'GLIM runtime identity requires an artifact or immutable token'
        )
    payload = {'artifacts': artifacts, 'tokens': token_values}
    return {
        'kind': kind,
        'sha256': hashlib.sha256(_canonical_json(payload)).hexdigest(),
        'artifact_count': sum(item['file_count'] for item in artifacts),
        'total_bytes': sum(item['total_bytes'] for item in artifacts),
    }


def _bool_text(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized == 'true':
        return True
    if normalized == 'false':
        return False
    raise argparse.ArgumentTypeError('expected true or false')


def build_identity(
    *,
    bag_dir: Path,
    config_dir: Path,
    runtime_kind: str,
    runtime_paths: Iterable[Path],
    runtime_tokens: Iterable[str],
    harness_path: Path,
    points_topic: str,
    imu_topic: str,
    mode: str,
    preset: str,
    no_imu: bool,
    viewer: bool,
    omp_threads: str | None,
) -> dict[str, Any]:
    """Build a path-free identity from exact bag, config, and runtime bytes."""
    if mode not in {'lidar-only', 'lidar-imu'}:
        raise GlimReferenceCacheError(f'unsupported GLIM mode: {mode!r}')
    if preset not in {'cpu', 'gpu'}:
        raise GlimReferenceCacheError(f'unsupported GLIM preset: {preset!r}')
    for label, value in (
        ('points topic', points_topic),
        ('IMU topic', imu_topic),
    ):
        if not value or '\n' in value or '\r' in value or len(value) > 256:
            raise GlimReferenceCacheError(
                f'{label} must be a non-empty single-line value'
            )
    if omp_threads is not None and (
        not omp_threads.isdigit() or int(omp_threads) < 1
    ):
        raise GlimReferenceCacheError(
            'OMP thread count must be an integer greater than zero'
        )
    harness = harness_path.resolve(strict=True)
    if not harness.is_file():
        raise GlimReferenceCacheError(
            f'comparison harness is not a regular file: {harness_path}'
        )
    harness_sha256, _ = _sha256_file(harness)
    helper_sha256, _ = _sha256_file(Path(__file__).resolve(strict=True))
    identity = {
        'bag': _tree_identity(bag_dir, 'rosbag2 input'),
        'config': _tree_identity(config_dir, 'effective GLIM config'),
        'runtime': _runtime_identity(
            runtime_kind,
            runtime_paths,
            runtime_tokens,
        ),
        'request': {
            'points_topic': points_topic,
            'imu_topic': imu_topic,
            'mode': mode,
            'preset': preset,
            'no_imu': no_imu,
            'viewer': viewer,
            'omp_threads': omp_threads,
            'harness_sha256': harness_sha256,
            'cache_helper_sha256': helper_sha256,
        },
    }
    cache_key = hashlib.sha256(_canonical_json(identity)).hexdigest()
    report = {
        'schema_version': 1,
        'schema_uri': SCHEMA_URI,
        'scope': 'local-glim-cross-validation-cache',
        'entry_state': 'IDENTITY_ONLY',
        'cache_key_sha256': cache_key,
        'identity': identity,
        'trajectory': None,
        'created_at': None,
        'authority': {
            'reference_kind': 'cross_validation',
            'ground_truth': False,
            'remote_mutations_performed': False,
        },
    }
    validate_contract(report, SCHEMA_NAME)
    return report


def _validate_identity(report: dict[str, Any]) -> None:
    validate_contract(report, SCHEMA_NAME)
    expected = hashlib.sha256(
        _canonical_json(report['identity'])
    ).hexdigest()
    if report['cache_key_sha256'] != expected:
        raise GlimReferenceCacheError(
            'cache key does not match the recorded identity'
        )


def _load_identity(path: Path) -> dict[str, Any]:
    report = load_json_object(path, 'GLIM cache identity')
    _validate_identity(report)
    return report


def _validate_trajectory(path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise GlimReferenceCacheError(
            f'GLIM trajectory must be a regular non-symlink file: {path}'
        )
    size = path.stat().st_size
    if size <= 0 or size > MAX_TRAJECTORY_BYTES:
        raise GlimReferenceCacheError(
            'GLIM trajectory size is outside the supported cache boundary'
        )
    digest = hashlib.sha256()
    line_count = 0
    previous_timestamp: float | None = None
    with path.open('rb') as stream:
        for raw_line in stream:
            digest.update(raw_line)
            if len(raw_line) > 4096:
                raise GlimReferenceCacheError(
                    'GLIM trajectory contains an overlong line'
                )
            try:
                line = raw_line.decode('utf-8').strip()
            except UnicodeDecodeError as exc:
                raise GlimReferenceCacheError(
                    'GLIM trajectory must be UTF-8 text'
                ) from exc
            if not line or line.startswith('#'):
                continue
            fields = line.split()
            if len(fields) != 8:
                raise GlimReferenceCacheError(
                    'GLIM trajectory rows must contain 8 TUM fields'
                )
            try:
                values = [float(field) for field in fields]
            except ValueError as exc:
                raise GlimReferenceCacheError(
                    'GLIM trajectory contains a non-numeric TUM field'
                ) from exc
            if not all(math.isfinite(value) for value in values):
                raise GlimReferenceCacheError(
                    'GLIM trajectory contains a non-finite TUM field'
                )
            timestamp = values[0]
            if (
                previous_timestamp is not None
                and timestamp <= previous_timestamp
            ):
                raise GlimReferenceCacheError(
                    'GLIM trajectory timestamps must be strictly increasing'
                )
            previous_timestamp = timestamp
            line_count += 1
    if line_count < 2:
        raise GlimReferenceCacheError(
            'GLIM trajectory must contain at least two TUM poses'
        )
    return {
        'sha256': digest.hexdigest(),
        'line_count': line_count,
        'total_bytes': size,
    }


def _cache_paths(
    cache_dir: Path,
    cache_key: str,
    *,
    create: bool,
) -> tuple[Path, Path]:
    if cache_dir.is_symlink():
        raise GlimReferenceCacheError('GLIM cache directory is a symlink')
    if create:
        cache_dir.mkdir(parents=True, exist_ok=True)
    if not cache_dir.is_dir():
        raise GlimReferenceCacheError(
            'no verified GLIM cache directory is available'
        )
    return (
        cache_dir / f'{cache_key}.traj_lidar.txt',
        cache_dir / f'{cache_key}.manifest.json',
    )


def _write_json_once(path: Path, payload: dict[str, Any]) -> None:
    encoded = (
        json.dumps(payload, indent=2, sort_keys=True).encode('utf-8')
        + b'\n'
    )
    descriptor: int | None = None
    temporary: Path | None = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f'.{path.name}.',
            dir=path.parent,
        )
        temporary = Path(temporary_name)
        with os.fdopen(descriptor, 'wb') as stream:
            descriptor = None
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        temporary.chmod(0o644)
        os.link(temporary, path)
    except FileExistsError:
        if (
            path.is_symlink()
            or not path.is_file()
            or path.read_bytes() != encoded
        ):
            raise GlimReferenceCacheError(
                f'refusing to replace an existing cache artifact: {path}'
            )
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def write_identity(path: Path, report: dict[str, Any]) -> None:
    """Write an identity once without replacing a different observation."""
    path.parent.mkdir(parents=True, exist_ok=True)
    _write_json_once(path, report)


def _copy_once(source: Path, target: Path) -> bool:
    """Publish a complete copy exactly once and report target ownership."""
    descriptor: int | None = None
    temporary: Path | None = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f'.{target.name}.',
            dir=target.parent,
        )
        temporary = Path(temporary_name)
        with source.open('rb') as input_stream, os.fdopen(
            descriptor, 'wb'
        ) as output_stream:
            descriptor = None
            shutil.copyfileobj(
                input_stream,
                output_stream,
                length=COPY_CHUNK_BYTES,
            )
            output_stream.flush()
            os.fsync(output_stream.fileno())
        temporary.chmod(0o644)
        try:
            os.link(temporary, target)
        except FileExistsError:
            return False
        return True
    finally:
        if descriptor is not None:
            os.close(descriptor)
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def store_entry(
    identity_path: Path,
    cache_dir: Path,
    trajectory_path: Path,
) -> dict[str, Any]:
    """Store one immutable trajectory and manifest, or verify idempotence."""
    identity = _load_identity(identity_path)
    trajectory = _validate_trajectory(trajectory_path)
    trajectory_target, manifest_target = _cache_paths(
        cache_dir,
        identity['cache_key_sha256'],
        create=True,
    )
    if trajectory_target.exists():
        observed = _validate_trajectory(trajectory_target)
        if observed != trajectory:
            raise GlimReferenceCacheError(
                'cache key collision or nondeterministic GLIM trajectory; '
                'the existing entry was preserved'
            )
    else:
        target_created = _copy_once(trajectory_path, trajectory_target)
        try:
            observed = _validate_trajectory(trajectory_target)
        except GlimReferenceCacheError:
            if target_created:
                trajectory_target.unlink(missing_ok=True)
            raise
        if observed != trajectory:
            if target_created:
                trajectory_target.unlink(missing_ok=True)
            raise GlimReferenceCacheError(
                'cache key collision or nondeterministic GLIM trajectory; '
                'the existing entry was preserved'
            )
    manifest = {
        **identity,
        'entry_state': 'CACHED',
        'trajectory': trajectory,
        'created_at': datetime.now(timezone.utc).isoformat().replace(
            '+00:00', 'Z'
        ),
    }
    validate_contract(manifest, SCHEMA_NAME)

    def matching_existing_manifest() -> dict[str, Any]:
        if manifest_target.is_symlink() or not manifest_target.is_file():
            raise GlimReferenceCacheError(
                'existing GLIM cache manifest is not a regular file'
            )
        existing = load_json_object(manifest_target, 'GLIM cache manifest')
        _validate_identity(existing)
        if (
            existing['entry_state'] != 'CACHED'
            or existing['identity'] != manifest['identity']
            or existing['trajectory'] != trajectory
        ):
            raise GlimReferenceCacheError(
                'existing GLIM cache manifest contradicts the exact entry'
            )
        return existing

    if manifest_target.exists() or manifest_target.is_symlink():
        return matching_existing_manifest()
    try:
        _write_json_once(manifest_target, manifest)
    except GlimReferenceCacheError:
        if manifest_target.exists() or manifest_target.is_symlink():
            return matching_existing_manifest()
        raise
    return manifest


def lookup_entry(identity_path: Path, cache_dir: Path) -> tuple[Path, Path]:
    """Return an exact cache hit after manifest and trajectory verification."""
    identity = _load_identity(identity_path)
    trajectory_path, manifest_path = _cache_paths(
        cache_dir,
        identity['cache_key_sha256'],
        create=False,
    )
    if not manifest_path.is_file() or manifest_path.is_symlink():
        raise GlimReferenceCacheError(
            'no verified GLIM cache manifest matches this exact identity'
        )
    manifest = load_json_object(manifest_path, 'GLIM cache manifest')
    _validate_identity(manifest)
    if manifest['entry_state'] != 'CACHED':
        raise GlimReferenceCacheError('GLIM cache manifest is not complete')
    if manifest['identity'] != identity['identity']:
        raise GlimReferenceCacheError(
            'GLIM cache manifest identity does not match this run'
        )
    observed = _validate_trajectory(trajectory_path)
    if observed != manifest['trajectory']:
        raise GlimReferenceCacheError(
            'GLIM cache trajectory bytes contradict the manifest'
        )
    return trajectory_path, manifest_path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Build and verify path-free, content-bound local GLIM '
            'trajectory cache entries.'
        )
    )
    subparsers = parser.add_subparsers(dest='command', required=True)

    identity = subparsers.add_parser('identity')
    identity.add_argument('--bag-dir', type=Path, required=True)
    identity.add_argument('--config-dir', type=Path, required=True)
    identity.add_argument(
        '--runtime-kind',
        choices=('local-install', 'docker-image'),
        required=True,
    )
    identity.add_argument(
        '--runtime-path', type=Path, action='append', default=[]
    )
    identity.add_argument(
        '--runtime-token', action='append', default=[]
    )
    identity.add_argument('--harness', type=Path, required=True)
    identity.add_argument('--points-topic', required=True)
    identity.add_argument('--imu-topic', required=True)
    identity.add_argument(
        '--mode', choices=('lidar-only', 'lidar-imu'), required=True
    )
    identity.add_argument('--preset', choices=('cpu', 'gpu'), required=True)
    identity.add_argument('--no-imu', type=_bool_text, required=True)
    identity.add_argument('--viewer', type=_bool_text, required=True)
    identity.add_argument('--omp-threads')
    identity.add_argument('--output', type=Path, required=True)

    lookup = subparsers.add_parser('lookup')
    lookup.add_argument('--identity', type=Path, required=True)
    lookup.add_argument('--cache-dir', type=Path, required=True)

    store = subparsers.add_parser('store')
    store.add_argument('--identity', type=Path, required=True)
    store.add_argument('--cache-dir', type=Path, required=True)
    store.add_argument('--trajectory', type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the content-bound cache helper."""
    args = _parser().parse_args(argv)
    try:
        if args.command == 'identity':
            report = build_identity(
                bag_dir=args.bag_dir,
                config_dir=args.config_dir,
                runtime_kind=args.runtime_kind,
                runtime_paths=args.runtime_path,
                runtime_tokens=args.runtime_token,
                harness_path=args.harness,
                points_topic=args.points_topic,
                imu_topic=args.imu_topic,
                mode=args.mode,
                preset=args.preset,
                no_imu=args.no_imu,
                viewer=args.viewer,
                omp_threads=args.omp_threads,
            )
            write_identity(args.output, report)
            print(report['cache_key_sha256'])
            return 0
        if args.command == 'lookup':
            trajectory, _ = lookup_entry(args.identity, args.cache_dir)
            print(trajectory)
            return 0
        manifest = store_entry(
            args.identity,
            args.cache_dir,
            args.trajectory,
        )
        print(manifest['cache_key_sha256'])
        return 0
    except (GlimReferenceCacheError, OSError, ValueError) as exc:
        print(f'GLIM reference cache error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
