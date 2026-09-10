#!/usr/bin/env python3
"""Validate the additive r3 apt allowlist candidate and its file bindings."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import stat
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
R3 = ROOT / 'docker/benchmark_adapters/glim_clean_room/phase3d/r3'
MANIFEST = R3 / 'apt_allowlist_candidate.json'
SCHEMA = R3 / 'apt_allowlist_candidate.schema.json'

if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts import registration_plugin_dependency_closure as dependency_closure  # noqa: E402


class ValidationError(ValueError):
    """Raised for source, status or candidate identity drift."""


def _canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode()


def _sha(path: Path) -> str:
    try:
        info = path.lstat()
    except OSError as error:
        raise ValidationError(f'candidate file cannot be inspected: {path}: {error}') from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or \
            info.st_nlink != 1 or info.st_size <= 0:
        raise ValidationError(f'candidate file is not a regular single-link file: {path}')
    data = path.read_bytes()
    if len(data) != info.st_size:
        raise ValidationError(f'candidate file changed while being read: {path}')
    return hashlib.sha256(data).hexdigest()


def validate(root: Path = ROOT) -> dict[str, Any]:
    root = Path(root)
    manifest_path = root / MANIFEST.relative_to(ROOT)
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if manifest.get('status') != 'OPT_IN_NOT_READY' or \
            manifest.get('benchmark_eligible') is not False or \
            manifest.get('production_manifest_modified') is not False or \
            manifest.get('active_r2_modified') is not False:
        raise ValidationError('candidate is not explicitly opt-in/non-promoting')
    if manifest.get('capture_contract') != dependency_closure.capture_contract():
        raise ValidationError('capture contract binding drift')
    if any(value == 'PENDING_RESEAL' for section in ('tool_hashes', 'schemas')
           for value in manifest.get(section, {}).values()):
        raise ValidationError('candidate manifest still has pending source hashes')
    for section in ('tool_hashes', 'schemas'):
        for relative, expected in manifest[section].items():
            actual = _sha(root / relative)
            if actual != expected:
                raise ValidationError(f'{section} hash drift: {relative}')
    policy_binding = manifest.get('authorization_policy')
    if not isinstance(policy_binding, dict) or set(policy_binding) != {
            'schema', 'path', 'file_sha256', 'canonical_sha256', 'status',
            'authorized_runtime'}:
        raise ValidationError('authorization policy binding is incomplete')
    if policy_binding['status'] != 'NOT_READY' or \
            policy_binding['authorized_runtime'] is not False:
        raise ValidationError('authorization policy is not explicitly non-promoting')
    policy_path = root / policy_binding['path']
    policy_data = policy_path.read_bytes()
    if hashlib.sha256(policy_data).hexdigest() != policy_binding['file_sha256']:
        raise ValidationError('authorization policy file hash drift')
    policy = json.loads(policy_data.decode('utf-8'))
    if policy.get('schema') != policy_binding['schema'] or \
            policy.get('status') != 'NOT_READY' or \
            policy.get('canonical_sha256') != policy_binding['canonical_sha256'] or \
            policy.get('canonical_sha256') != hashlib.sha256(_canonical(
                {key: value for key, value in policy.items()
                 if key != 'canonical_sha256'})).hexdigest():
        raise ValidationError('authorization policy identity/status drift')
    candidate_binding = policy.get('candidate_manifest')
    if not isinstance(candidate_binding, dict) or \
            candidate_binding.get('candidate_id') != manifest.get('candidate_id') or \
            candidate_binding.get('revision') != manifest.get('revision'):
        raise ValidationError('authorization policy candidate binding drift')
    expected = hashlib.sha256(_canonical({key: value for key, value in manifest.items()
                                          if key != 'canonical_sha256'})).hexdigest()
    if manifest.get('canonical_sha256') != expected:
        raise ValidationError('candidate manifest canonical identity drift')
    return {'status': 'PASS', 'candidate_id': manifest['candidate_id'],
            'benchmark_eligible': False, 'candidate_sha256': _sha(manifest_path)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    try:
        print(json.dumps(validate(), sort_keys=True))
    except (ValidationError, OSError, json.JSONDecodeError) as error:
        parser.error(str(error))
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
