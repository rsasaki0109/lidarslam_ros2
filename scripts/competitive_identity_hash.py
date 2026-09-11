"""Canonical hashes shared by the competitive execution identity tools.

The profile registers the full-file SHA of the execution-selection receipt,
while that receipt records a hash of the profile.  The profile's registered
receipt-SHA field is therefore excluded from the profile hash contract.  The
remaining parsed profile mapping is serialized as sorted, compact JSON so
YAML formatting and key order cannot change the identity.
"""

from __future__ import annotations

import copy
import hashlib
import json
from typing import Any


PROFILE_CANONICAL_HASH_KIND = 'canonical_profile_sha256_v1'
PROFILE_RECEIPT_SHA_PATH = (
    'competitive_slam_profile', 'evidence_gate_v2',
    'execution_selection_receipt_sha256')


def canonical_json_sha256(value: Any) -> str:
    """Hash a JSON-compatible value using the repository canonical encoding."""
    encoded = json.dumps(
        value, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode('utf-8')
    return hashlib.sha256(encoded).hexdigest()


def canonical_profile_payload(profile: Any) -> dict[str, Any]:
    """Return the normalized profile mapping used by the v1 hash contract.

    Callers may pass the complete YAML document or only its
    ``competitive_slam_profile`` mapping.  A copy is always made and the
    registered receipt SHA is removed at the one explicit path above.
    """
    if not isinstance(profile, dict):
        raise ValueError('competitive profile must be a mapping')
    if 'competitive_slam_profile' in profile:
        normalized = copy.deepcopy(profile)
    else:
        normalized = {'competitive_slam_profile': copy.deepcopy(profile)}
    contract = normalized.get('competitive_slam_profile')
    if not isinstance(contract, dict):
        raise ValueError('competitive_slam_profile must be a mapping')
    evidence_gate = contract.get('evidence_gate_v2')
    if not isinstance(evidence_gate, dict):
        raise ValueError('competitive_slam_profile.evidence_gate_v2 must be a mapping')
    evidence_gate.pop('execution_selection_receipt_sha256', None)
    return normalized


def canonical_profile_sha256(profile: Any) -> str:
    """Return the non-cyclic canonical profile SHA-256."""
    return canonical_json_sha256(canonical_profile_payload(profile))


if __name__ == "__main__":
    raise SystemExit("competitive_identity_hash is a library module; import it instead of running it directly.")
